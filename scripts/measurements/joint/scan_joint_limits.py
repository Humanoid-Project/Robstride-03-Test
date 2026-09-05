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

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common import (
    DEFAULT_INTERFACE, HOST_ID, JOINT_MAP, JOINT_LIMITS_RAD, MECH_POS_INDEX,
    SPECS, Motor, channel_for_id,
)

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "joint_limit")

MOTOR_MODEL = {
    1: "rs02", 2: "rs03", 3: "rs03", 4: "rs03", 5: "rs02", 6: "rs02",
    7: "rs02", 8: "rs03", 9: "rs03", 10: "rs03", 11: "rs02", 12: "rs02",
}

DEFAULT_TIMEOUT = 0.1
DEFAULT_INTERVAL = 0.1


def parse_args():
    parser = argparse.ArgumentParser(
        description="Track joint minimum and maximum positions until Enter is pressed.")
    parser.add_argument("--motor-id", "--motor-ids", dest="motor_id", nargs="+",
                        type=lambda v: int(v, 0), default=list(range(1, 13)),
                        help="Motor IDs to scan")
    parser.set_defaults(
        interface=DEFAULT_INTERFACE, host_id=HOST_ID,
        timeout=DEFAULT_TIMEOUT, interval=DEFAULT_INTERVAL,
    )
    return parser.parse_args()


def format_deg(value):
    if value is None or not math.isfinite(value):
        return "--"
    return f"{math.degrees(value):+7.2f}"


def poll_worker(channel, motor_ids, interface, host_id, timeout,
                 state, minmax, samples, lock, notes, stop):
    try:
        bus = can.Bus(channel=channel, interface=interface)
    except OSError as error:
        with lock:
            notes.append(f"[{channel}] open failed: {error}  "
                         f"(sudo ip link set {channel} up type can bitrate 1000000)")
        return
    motors = {
        motor_id: Motor(bus, motor_id, SPECS[MOTOR_MODEL[motor_id]], host_id=host_id)
        for motor_id in motor_ids
    }
    try:
        while not stop.is_set():
            for motor_id, motor in motors.items():
                position = motor.read_param_f32(MECH_POS_INDEX, timeout=timeout)
                if position is None or not math.isfinite(position):
                    continue
                with lock:
                    state[motor_id] = position
                    samples[motor_id] += 1
                    lo, hi = minmax[motor_id]
                    minmax[motor_id] = (min(lo, position), max(hi, position))
    except can.CanError as error:
        with lock:
            notes.append(f"[{channel}] CAN error: {error}")
    finally:
        bus.shutdown()


def wait_for_enter(stop):
    try:
        input()
    except EOFError:
        pass
    stop.set()


def render(motor_ids, state, minmax, samples, notes):
    lines = ["\033[2J\033[3J\033[H"]
    lines.append(f"joint limit scan   {time.strftime('%H:%M:%S')}   "
                 "Move each joint slowly through its range; press Enter to stop.\n")
    lines.append(f"  {'ID':>3}  {'joint':<18}  {'current':>8}  {'min':>8}  {'max':>8}  "
                 f"{'span':>7}  {'defined limit':>18}  {'n':>6}")
    lines.append("  " + "-" * 88)
    for motor_id in motor_ids:
        lo, hi = minmax[motor_id]
        span = f"{math.degrees(hi - lo):7.2f}" if math.isfinite(lo) and math.isfinite(hi) else f"{'--':>7}"
        lim_lo, lim_hi = JOINT_LIMITS_RAD[motor_id]
        lim_txt = f"{math.degrees(lim_lo):+.1f}~{math.degrees(lim_hi):+.1f}"
        lines.append(
            f"  {motor_id:>3}  {JOINT_MAP.get(motor_id, '?'):<18}  "
            f"{format_deg(state[motor_id]):>8}  {format_deg(lo):>8}  {format_deg(hi):>8}  "
            f"{span}  {lim_txt:>18}  {samples[motor_id]:>6}"
        )
    if notes:
        lines.append("")
        lines.extend(notes)
    sys.stdout.write("\n".join(lines) + "\n")
    sys.stdout.flush()


def print_summary(motor_ids, minmax, samples):
    print(f"\n{'ID':>3}  {'joint':<18}  {'min':>8}  {'max':>8}  {'span':>7}  {'defined limit':>18}  {'n':>6}")
    print("-" * 88)
    for motor_id in motor_ids:
        lo, hi = minmax[motor_id]
        lim_lo, lim_hi = JOINT_LIMITS_RAD[motor_id]
        lim_txt = f"{math.degrees(lim_lo):+.1f}~{math.degrees(lim_hi):+.1f}"
        span = f"{math.degrees(hi - lo):7.2f}" if math.isfinite(lo) and math.isfinite(hi) else f"{'--':>7}"
        print(f"{motor_id:>3}  {JOINT_MAP.get(motor_id, '?'):<18}  "
              f"{format_deg(lo):>8}  {format_deg(hi):>8}  {span}  {lim_txt:>18}  {samples[motor_id]:>6}")


def save_csv(motor_ids, minmax, samples):
    os.makedirs(DATA_DIR, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%dT%H%M%S")
    path = os.path.join(DATA_DIR, f"joint_limit_{ts}.csv")
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            "motor_id", "joint", "samples",
            "min_rad", "max_rad", "min_deg", "max_deg", "span_deg",
            "current_limit_lo_deg", "current_limit_hi_deg",
        ])
        for motor_id in motor_ids:
            lo, hi = minmax[motor_id]
            lim_lo, lim_hi = JOINT_LIMITS_RAD[motor_id]
            row = [motor_id, JOINT_MAP.get(motor_id, "?"), samples[motor_id]]
            if math.isfinite(lo) and math.isfinite(hi):
                row += [
                    f"{lo:.6f}", f"{hi:.6f}",
                    f"{math.degrees(lo):.4f}", f"{math.degrees(hi):.4f}",
                    f"{math.degrees(hi - lo):.4f}",
                ]
            else:
                row += ["", "", "", "", ""]
            row += [f"{math.degrees(lim_lo):.4f}", f"{math.degrees(lim_hi):.4f}"]
            writer.writerow(row)
    return path


def main():
    args = parse_args()
    motor_ids = sorted(set(args.motor_id))
    unknown = [motor_id for motor_id in motor_ids if motor_id not in JOINT_MAP]
    if unknown:
        print(f"ERROR: Unsupported motor IDs: {unknown}")
        return 1

    channels = sorted({channel_for_id(motor_id) for motor_id in motor_ids})
    state = {motor_id: None for motor_id in motor_ids}
    minmax = {motor_id: (float("inf"), float("-inf")) for motor_id in motor_ids}
    samples = {motor_id: 0 for motor_id in motor_ids}
    notes = []
    lock = threading.Lock()
    stop = threading.Event()

    threads = [
        threading.Thread(
            target=poll_worker,
            args=(
                channel,
                [motor_id for motor_id in motor_ids if channel_for_id(motor_id) == channel],
                args.interface, args.host_id, args.timeout,
                state, minmax, samples, lock, notes, stop,
            ),
            daemon=True,
        )
        for channel in channels
    ]
    for thread in threads:
        thread.start()

    print("Move each joint slowly through its full range. Press Enter to save or Ctrl-C to stop.")
    enter_thread = threading.Thread(target=wait_for_enter, args=(stop,), daemon=True)
    enter_thread.start()

    try:
        while not stop.is_set():
            with lock:
                state_snapshot = dict(state)
                minmax_snapshot = dict(minmax)
                samples_snapshot = dict(samples)
                notes_snapshot = list(notes)
            render(motor_ids, state_snapshot, minmax_snapshot, samples_snapshot, notes_snapshot)
            time.sleep(args.interval)
    except KeyboardInterrupt:
        stop.set()

    for thread in threads:
        thread.join(timeout=2.0)

    with lock:
        final_minmax = dict(minmax)
        final_samples = dict(samples)

    print_summary(motor_ids, final_minmax, final_samples)
    missing = [motor_id for motor_id in motor_ids if final_samples[motor_id] == 0]
    if missing:
        print(f"WARNING: No samples from motor IDs {missing}.")

    path = save_csv(motor_ids, final_minmax, final_samples)
    print(f"Saved: {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
