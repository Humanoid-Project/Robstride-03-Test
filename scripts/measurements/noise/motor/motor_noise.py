#!/usr/bin/env python3
import argparse
import csv
import math
import os
import sys
import threading
import time
from datetime import datetime

import can

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from common import (
    DEFAULT_INTERFACE, HOST_ID, SPECS,
    RUN_MODE_INDEX, RUN_MODE_OPERATION,
    FeedbackHub, Motor, channel_for_id, active_brake,
)

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
FEEDBACK_SOURCE = "type_0x02"

MOTOR_MODEL = {
    1: "rs02", 2: "rs03", 3: "rs03", 4: "rs03", 5: "rs02", 6: "rs02",
    7: "rs02", 8: "rs03", 9: "rs03", 10: "rs03", 11: "rs02", 12: "rs02",
}

DEFAULT_DURATION = 60.0
DEFAULT_KD = 1.0
DEFAULT_FEEDBACK_TIMEOUT = 0.05
MAX_FEEDBACK_TIMEOUT = 0.5


def parse_args():
    parser = argparse.ArgumentParser(
        description="Capture type 0x02 CAN feedback from stationary motors.")
    parser.add_argument("--motor-id", dest="motor_id", nargs="+",
                        type=lambda v: int(v, 0), default=list(range(1, 13)),
                        help="One or more motor IDs to capture")
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION,
                        help="Capture time per channel in seconds")
    parser.set_defaults(
        interface=DEFAULT_INTERFACE, host_id=HOST_ID, kd=DEFAULT_KD,
        feedback_timeout=DEFAULT_FEEDBACK_TIMEOUT,
    )
    return parser.parse_args()


def validate_args(args, motor_ids):
    problems = []
    if not math.isfinite(args.duration) or args.duration <= 0.0:
        problems.append("--duration must be a finite value greater than zero.")
    if not math.isfinite(args.kd) or args.kd < 0.0:
        problems.append("Internal kd must be finite and nonnegative.")
    else:
        exceeded = [
            motor_id for motor_id in motor_ids
            if args.kd > SPECS[MOTOR_MODEL[motor_id]].kd_max
        ]
        if exceeded:
            problems.append(f"Internal kd exceeds the limit for motor IDs {exceeded}.")
    if (not math.isfinite(args.feedback_timeout)
            or args.feedback_timeout <= 0.0
            or args.feedback_timeout > MAX_FEEDBACK_TIMEOUT):
        problems.append(
            f"Internal feedback timeout must be in (0, {MAX_FEEDBACK_TIMEOUT}]."
        )
    return problems


def confirm(args):
    print(f"Motors: {sorted(set(args.motor_id))}; duration: {args.duration:.1f}s per channel")
    print(f"WARNING: Motors will be enabled with velocity damping (kd={args.kd}).")
    if not sys.stdin.isatty():
        print("ERROR: Run this command in an interactive terminal.")
        return False
    try:
        answer = input(
            "Press Enter after securing the robot and preparing the E-stop, or Ctrl-C to cancel: "
        )
    except (KeyboardInterrupt, EOFError):
        print("\nCancelled.")
        return False
    if answer.strip():
        print("Cancelled.")
        return False
    return True


def capture_channel(channel, motor_ids, args, rows, lock, notes, barrier, stop_event):
    try:
        bus = can.Bus(channel=channel, interface=args.interface)
    except (OSError, can.CanError) as error:
        with lock:
            notes.append(f"[{channel}] open failed: {error}  "
                         f"(sudo ip link set {channel} up type can bitrate 1000000)")
        stop_event.set()
        barrier.abort()
        return

    motors = {}
    normal_completion = False
    try:
        for motor_id in motor_ids:
            if stop_event.is_set():
                return
            motor = Motor(bus, motor_id, SPECS[MOTOR_MODEL[motor_id]], host_id=args.host_id)
            motors[motor_id] = motor
            motor.stop()
            if stop_event.wait(0.01):
                return
            motor.write_param_u8(RUN_MODE_INDEX, RUN_MODE_OPERATION)
            if stop_event.wait(0.01):
                return
            motor.enable()
            if stop_event.wait(0.01):
                return

        hub = FeedbackHub(bus, motors.values(), host_id=args.host_id)

        try:
            barrier.wait(timeout=5.0)
        except threading.BrokenBarrierError:
            with lock:
                notes.append(f"[{channel}] cancelled because another channel failed to initialize.")
            stop_event.set()
            return

        t_end = time.monotonic() + args.duration
        while time.monotonic() < t_end and not stop_event.is_set():
            for motor_id, motor in motors.items():
                if stop_event.is_set():
                    break
                motor.control(pos=0.0, vel=0.0, kp=0.0, kd=args.kd, torque=0.0)
                position = hub.wait_for(motor_id, timeout=args.feedback_timeout)
                t_recv = time.monotonic()
                if position is None:
                    with lock:
                        rows.append((t_recv, motor_id, "", "", "", "", "miss"))
                    continue
                with lock:
                    rows.append((t_recv, motor_id, f"{motor.last_position:.6f}", f"{motor.last_velocity:.6f}",
                                 f"{motor.last_torque:.4f}", f"{motor.last_temp:.1f}", "ok"))
        normal_completion = not stop_event.is_set()
    except (OSError, can.CanError) as error:
        with lock:
            notes.append(f"[{channel}] CAN error: {error}")
        stop_event.set()
        barrier.abort()
    except Exception as error:
        with lock:
            notes.append(f"[{channel}] unexpected error: {type(error).__name__}: {error}")
        stop_event.set()
        barrier.abort()
    finally:
        if normal_completion and not stop_event.is_set():
            for motor in motors.values():
                if stop_event.is_set():
                    break
                try:
                    active_brake(motor, duration=0.1)
                except Exception:
                    pass
        stop_errors = []
        for motor_id, motor in motors.items():
            try:
                motor.stop()
            except Exception as error:
                stop_errors.append(f"ID {motor_id}: {error}")
        if stop_errors:
            with lock:
                notes.append(f"[{channel}] stop failed: {'; '.join(stop_errors)}")
        try:
            bus.shutdown()
        except Exception as error:
            with lock:
                notes.append(f"[{channel}] CAN shutdown failed: {type(error).__name__}: {error}")


def save_csv(rows, args, motor_ids):
    os.makedirs(DATA_DIR, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%dT%H%M%S")
    fname = f"motor_noise_{ts}.csv"
    path = os.path.join(DATA_DIR, fname)
    with open(path, "w", newline="", encoding="utf-8") as f:
        f.write(f"# feedback_source: {FEEDBACK_SOURCE}\n")
        f.write(f"# motor_ids: {sorted(set(motor_ids))}\n")
        f.write(f"# interface: {args.interface}\n")
        f.write(f"# duration_s: {args.duration}\n")
        f.write(f"# kd: {args.kd}\n")
        f.write(f"# feedback_timeout_s: {args.feedback_timeout}\n")
        f.write(f"# started_at: {datetime.now().isoformat(timespec='seconds')}\n")
        writer = csv.writer(f)
        writer.writerow(["t_monotonic_s", "motor_id", "pos_rad", "vel_rad_s",
                          "torque_Nm", "temp_C", "status"])
        for row in rows:
            writer.writerow(row)
    return path


def main():
    args = parse_args()
    motor_ids = sorted(set(args.motor_id))
    unknown = [m for m in motor_ids if m not in MOTOR_MODEL]
    if unknown:
        print(f"ERROR: Unsupported motor IDs: {unknown}")
        return 1
    problems = validate_args(args, motor_ids)
    if problems:
        print("Invalid arguments:")
        for problem in problems:
            print(f"  {problem}")
        return 1
    if not confirm(args):
        return 1

    channels = sorted({channel_for_id(m) for m in motor_ids})
    rows = []
    lock = threading.Lock()
    notes = []
    barrier = threading.Barrier(len(channels))
    stop_event = threading.Event()

    threads = [
        threading.Thread(
            target=capture_channel,
            args=(channel, [m for m in motor_ids if channel_for_id(m) == channel],
                  args, rows, lock, notes, barrier, stop_event),
        )
        for channel in channels
    ]
    print(f"Capturing {len(motor_ids)} motors on {len(channels)} channels...")
    interrupted = False
    main_error = None
    started_threads = []
    try:
        for t in threads:
            t.start()
            started_threads.append(t)
        while any(t.is_alive() for t in started_threads):
            for t in started_threads:
                t.join(timeout=0.1)
    except KeyboardInterrupt:
        interrupted = True
        print("\nInterrupted. Stopping all motors...")
    except Exception as error:
        main_error = error
        print(f"\nERROR: {type(error).__name__}: {error}. Stopping all motors...")
    finally:
        stop_event.set()
        barrier.abort()
        while any(t.is_alive() for t in started_threads):
            try:
                for t in started_threads:
                    t.join(timeout=0.1)
            except KeyboardInterrupt:
                print("Stopping safely. Use the E-stop if the robot moves unexpectedly.")

    for note in notes:
        print(note)
    if interrupted:
        print("Capture interrupted; data was not saved.")
        return 130
    if main_error is not None:
        print("Capture failed; data was not saved.")
        return 1
    if notes:
        print("Capture failed; data was not saved.")
        return 1
    if not rows:
        print("ERROR: No samples recorded.")
        return 1

    path = save_csv(rows, args, motor_ids)
    n_ok = sum(1 for r in rows if r[-1] == "ok")
    n_miss = sum(1 for r in rows if r[-1] == "miss")
    print(f"Saved: {path} (samples={len(rows)}, ok={n_ok}, miss={n_miss})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
