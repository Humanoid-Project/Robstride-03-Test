#!/usr/bin/env python3





import argparse
import math
import os
import subprocess
import sys
import time
from datetime import datetime

import can

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common import (
    HOST_ID, DEFAULT_INTERFACE, SPECS,
    RUN_MODE_INDEX, RUN_MODE_OPERATION, MECH_POS_INDEX, FAULT_STA_INDEX,
    Motor, channel_for_id, decode_fault_bits, active_brake,
    validate_args, report_invalid_args,
    JOINT_LIMITS_RAD, DEFAULT_LIMIT_MARGIN_RAD, joint_limit_for, exceeds_joint_limit,
)

ARG_CHECKS = [
    ("speed", "speed"), ("kd", "positive"), ("hold-time", "positive"),
    ("ramp-time", "positive"), ("max-turns", "positive"),
    ("feedback-timeout", "positive"),
]

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
ANALYZE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "analyze_damping.py")
PAUSE_S = 1.0


def positive_int(value):
    value = int(value)
    if value < 1:
        raise argparse.ArgumentTypeError("Must be an integer of 1 or greater")
    return value


def parse_args():
    p = argparse.ArgumentParser(description="Measure and analyze motor damping.")
    p.add_argument("--motor-id", type=lambda v: int(v, 0), required=True)
    p.add_argument("--model", choices=list(SPECS.keys()), required=True)
    p.add_argument("--speeds", type=float, nargs="+", required=True,
                   help="Signed target speeds in rad/s")
    p.add_argument("--repeats", type=positive_int, default=1, help="Repeats per speed")
    p.add_argument("--ignore-joint-limit", action="store_true",
                   help="Disable joint-limit checks for unloaded bench tests only")
    p.set_defaults(
        interface=DEFAULT_INTERFACE, host_id=HOST_ID,
        kd=2.0, hold_time=1.5, ramp_time=0.5, max_turns=0.5,
        limit_margin=DEFAULT_LIMIT_MARGIN_RAD,
        feedback_timeout=0.3, out=None,
    )
    return p.parse_args()


def confirm(args, model):
    print(f"Motor: ID {args.motor_id}, {model.upper()}, {args.channel}")
    print(f"Speeds: {args.speeds} rad/s x {args.repeats}")
    if args.ignore_joint_limit:
        print("WARNING: Joint-limit checks are disabled. Use only with an unloaded output.")
    else:
        lo, hi = JOINT_LIMITS_RAD.get(args.motor_id, (None, None))
        if lo is None:
            print(f"ERROR: No joint limit is defined for motor ID {args.motor_id}.")
        else:
            lo_m, hi_m = joint_limit_for(args.motor_id, args.limit_margin)
            print(f"Joint limit: {math.degrees(lo_m):+.1f} to {math.degrees(hi_m):+.1f} deg")
            worst_travel = max(abs(speed) for speed in args.speeds) * (args.ramp_time + args.hold_time)
            usable_range = hi_m - lo_m
            print(f"Maximum planned travel: {math.degrees(worst_travel):.1f} deg "
                  f"({100*worst_travel/usable_range:.0f}% of usable range)")
            if worst_travel > usable_range:
                print("WARNING: Planned travel exceeds the usable joint range.")
    print("WARNING: Verify zero calibration, secure the robot, and prepare the E-stop.")
    if not sys.stdin.isatty():
        print("ERROR: Run this command in an interactive terminal.")
        return False
    try:
        input("Press Enter to continue or Ctrl-C to cancel: ")
    except (KeyboardInterrupt, EOFError):
        print("\nCancelled.")
        return False
    return True


def ramp_to_speed(motor, target_speed, kd, pos_ref, args, label):

    deadline = time.monotonic() + args.ramp_time
    last_pos, last_vel = pos_ref, 0.0
    last_print = 0.0
    last_ok = time.monotonic()
    while time.monotonic() < deadline:
        motor.control(pos=0.0, vel=target_speed, kp=0.0, kd=kd, torque=0.0)
        fb = motor.poll_feedback(timeout=0.05)
        now = time.monotonic()
        if fb is None:
            if now - last_ok > args.feedback_timeout:
                print()
                return False, last_pos, last_vel, "feedback_lost"
            status = f"[{label}] no reply ({now - last_ok:.2f}s)"
        else:
            last_ok = now
            _, pos, vel, tq, temp, fault = fb
            last_pos, last_vel = pos, vel
            if pos_ref is not None and abs(pos - pos_ref) >= args.max_turns * 2.0 * math.pi:
                print()
                return False, last_pos, last_vel, "max_turns"
            if (not args.ignore_joint_limit and target_speed != 0.0
                    and exceeds_joint_limit(pos, args.motor_id, args.limit_margin)):
                print()
                print(f"[{label}] Joint limit reached at {pos:+.4f} rad ({math.degrees(pos):+.1f} deg).")
                return False, last_pos, last_vel, "joint_limit"
            status = (f"[{label}] pos={pos:+.4f}  vel={vel:+.3f}  target={target_speed:+.3f}  "
                      f"tq={tq:+.3f} N*m")
        if now - last_print >= 0.1:
            print(f"\r  {status}    ", end="", flush=True)
            last_print = now
    print()
    return True, last_pos, last_vel, "ok"


def return_to_zero(motor, timeout=2.5, kp=15.0, kd=3.0, pos_tol=0.02, max_speed=0.4):
    fb = motor.poll_feedback(timeout=0.1)
    if fb is None:
        return None
    _, start_pos, _, _, _, _ = fb
    ramp_time = max(0.5, abs(start_pos) / max_speed)
    deadline = time.monotonic() + max(timeout, ramp_time + 0.5)
    ramp_start = time.monotonic()
    last_pos = start_pos
    while time.monotonic() < deadline:
        progress = min(1.0, (time.monotonic() - ramp_start) / ramp_time)
        smooth = progress * progress * (3.0 - 2.0 * progress)
        target = start_pos * (1.0 - smooth)
        motor.control(pos=target, vel=0.0, kp=kp, kd=kd, torque=0.0)
        fb = motor.poll_feedback(timeout=0.05)
        if fb is None:
            continue
        _, pos, vel, tq, temp, fault = fb
        last_pos = pos
        if progress >= 1.0 and abs(pos) <= pos_tol and abs(vel) < 0.05:
            break
    return last_pos


def run_hold(motor, target_speed, kd, pos_ref, args):
    rows = []
    t0 = time.monotonic()
    stop_reason = "hold_time"
    limit = max(2.0, abs(target_speed) * 2.0 + 2.0)
    last_ok = t0
    while True:
        now = time.monotonic()
        if now - t0 >= args.hold_time:
            stop_reason = "hold_time"
            break
        motor.control(pos=0.0, vel=target_speed, kp=0.0, kd=kd, torque=0.0)
        fb = motor.poll_feedback(timeout=0.05)
        if fb is None:
            if time.monotonic() - last_ok > args.feedback_timeout:
                stop_reason = "feedback_lost"
                break
            continue
        last_ok = time.monotonic()
        t_recv, pos, vel, tq, temp, fault = fb
        rows.append((t_recv - t0, pos, vel, tq, temp, fault))
        if abs(pos - pos_ref) >= args.max_turns * 2.0 * math.pi:
            stop_reason = "max_turns"
            break
        if not args.ignore_joint_limit and exceeds_joint_limit(pos, args.motor_id, args.limit_margin):
            print(f"\nJoint limit reached at {pos:+.4f} rad ({math.degrees(pos):+.1f} deg).")
            stop_reason = "joint_limit"
            break
        if abs(vel) >= limit:
            stop_reason = "overspeed"
            break
    return rows, stop_reason


def save_csv(path, rows, meta):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        for key, value in meta.items():
            f.write(f"# {key}: {value}\n")
        f.write("t_s,pos_rad,vel_rad_s,torque_meas_Nm,temp_C,fault_hex\n")
        for t_s, pos, vel, tq, temp, fault in rows:
            f.write(f"{t_s:.6f},{pos:.6f},{vel:.6f},{tq:.4f},{temp:.1f},{fault:02X}\n")


def capture_once(args, speed, run_index):
    args.speed = speed
    spec = SPECS[args.model]
    bus = can.Bus(channel=args.channel, interface=args.interface)
    motor = None
    rows, stop_reason = [], "not_started"
    try:
        motor = Motor(bus, args.motor_id, spec, host_id=args.host_id)

        motor.stop()
        time.sleep(0.02)
        initial_pos = motor.read_param_f32(MECH_POS_INDEX, timeout=0.3)
        if initial_pos is None:
            print(f"ERROR: Motor ID {args.motor_id} did not reply on {args.channel}.")
            return 1, None, "no_response"
        print(f"Position: {initial_pos:+.4f} rad ({math.degrees(initial_pos):+.2f} deg)")

        fault_reg = motor.read_param(FAULT_STA_INDEX, fmt="<I", timeout=0.3)
        if fault_reg is None:
            print("WARNING: Fault register 0x3022 could not be read.")
        else:
            bits = decode_fault_bits(fault_reg)
            print(f"Fault register: 0x{fault_reg:08X}" + (f" ({', '.join(bits)})" if bits else " (OK)"))

        motor.write_param_u8(RUN_MODE_INDEX, RUN_MODE_OPERATION)
        time.sleep(0.01)
        motor.enable()
        time.sleep(0.01)

        pos_ref = None
        for _ in range(5):
            motor.control(pos=0.0, vel=0.0, kp=0.0, kd=0.0, torque=0.0)
            fb0 = motor.poll_feedback(timeout=0.2)
            if fb0 is not None:
                pos_ref = fb0[1]
                break
        if pos_ref is None:
            print("ERROR: No type 0x02 feedback; safety reference unavailable.")
            return 1, None, "no_realtime_feedback"

        if not args.ignore_joint_limit:
            lo, hi = JOINT_LIMITS_RAD.get(args.motor_id, (None, None))
            if lo is None:
                print(f"ERROR: No joint limit is defined for motor ID {args.motor_id}.")
                return 1, None, "missing_joint_limit"
            print(f"URDF joint limit: {math.degrees(lo):+.1f} to {math.degrees(hi):+.1f} deg")
            if exceeds_joint_limit(pos_ref, args.motor_id, args.limit_margin):
                print(f"ERROR: Position {pos_ref:+.4f} rad ({math.degrees(pos_ref):+.1f} deg) is outside the joint limit.")
                motor.stop()
                return 1, None, "initial_joint_limit"
            lo_m, hi_m = joint_limit_for(args.motor_id, args.limit_margin)
            room = (hi_m - pos_ref) if args.speed > 0 else (pos_ref - lo_m)
            print(f"Available travel: {math.degrees(room):.1f} deg")

        print(f"Ramping to {args.speed:+.3f} rad/s...")
        ok, pos_now, vel_now, ramp_reason = ramp_to_speed(
            motor, args.speed, args.kd, pos_ref, args, "ramp-up")
        if not ok:
            print(f"ERROR: Ramp stopped ({ramp_reason}).")
            return 1, None, ramp_reason
        if args.speed != 0 and (vel_now == 0.0 or vel_now / args.speed < 0):
            print(f"WARNING: Velocity {vel_now:+.3f} does not track target {args.speed:+.3f} rad/s.")
        print(f"Recording at pos={pos_now:+.4f} rad, vel={vel_now:+.3f} rad/s...")

        rows, stop_reason = run_hold(motor, args.speed, args.kd, pos_ref, args)

        print("Decelerating...")
        ramp_to_speed(motor, 0.0, args.kd, None, args, "ramp-down")
        print("Returning to zero...")
        final_pos = return_to_zero(motor)
        print(f"Return position: {final_pos:+.4f} rad ({math.degrees(final_pos):+.2f} deg)")
    except KeyboardInterrupt:
        print("\nInterrupted.")
        stop_reason = "keyboard_interrupt"
    finally:
        if motor is not None:
            try:
                active_brake(motor)
            except can.CanError:
                pass
            try:
                motor.stop()
            except can.CanError:
                pass
        bus.shutdown()

    if not rows:
        print(f"ERROR: No samples recorded ({stop_reason}).")
        return 1, None, stop_reason

    duration = rows[-1][0] - rows[0][0]
    mean_vel = sum(r[2] for r in rows) / len(rows)
    mean_tq = sum(r[3] for r in rows) / len(rows)
    achieved_hz = (len(rows) - 1) / duration if duration > 0 else 0.0

    print(f"Capture: {stop_reason}, samples={len(rows)}, duration={duration*1000:.1f} ms, "
          f"rate={achieved_hz:.0f} Hz, mean vel={mean_vel:+.4f}, mean torque={mean_tq:+.4f} N*m")
    if len(rows) < 30:
        print("WARNING: Fewer than 30 samples were recorded.")

    ts = datetime.now().strftime("%Y%m%dT%H%M%S_%f")
    fname = (f"id{args.motor_id}_{args.model}_{args.speed:+.3f}rads_{ts}_r{run_index:02d}.csv"
             .replace("+", "p").replace("-", "m"))
    out_path = os.path.join(DATA_DIR, fname)

    meta = {
        "motor_id": args.motor_id,
        "model": args.model,
        "channel": args.channel,
        "host_id": f"0x{args.host_id:02X}",
        "speed_cmd_rad_s": f"{args.speed:.4f}",
        "kd": f"{args.kd:.2f}",
        "started_at": datetime.now().isoformat(timespec="seconds"),
        "stop_reason": stop_reason,
        "samples": len(rows),
        "duration_s": f"{duration:.4f}",
    }
    save_csv(out_path, rows, meta)
    print(f"Saved: {out_path}")
    return (0 if stop_reason == "hold_time" else 1), out_path, stop_reason


def main():
    args = parse_args()
    args.channel = channel_for_id(args.motor_id)

    for speed in args.speeds:
        args.speed = speed
        if report_invalid_args(validate_args(args, args.model, ARG_CHECKS)):
            return 1
    if args.kd > SPECS[args.model].kd_max:
        print(f"ERROR: Internal kd {args.kd} exceeds the {args.model.upper()} limit.")
        return 1
    if not confirm(args, args.model):
        return 1

    plan = [speed for speed in args.speeds for _ in range(args.repeats)]
    paths = []
    failed = False
    for index, speed in enumerate(plan, 1):
        print(f"\n[{index}/{len(plan)}] speed={speed:+.3f} rad/s")
        rc, path, stop_reason = capture_once(args, speed, index)
        if rc == 0 and path is not None:
            paths.append(path)
        if rc != 0 or stop_reason == "keyboard_interrupt":
            failed = True
            print("Stopping the remaining runs for safety.")
            break
        if index < len(plan):
            time.sleep(PAUSE_S)

    if paths:
        print("\nAnalyzing captured CSV files...")
        analysis = subprocess.run([sys.executable, ANALYZE, *paths])
        failed = failed or analysis.returncode != 0
    return 1 if failed or not paths else 0


if __name__ == "__main__":
    sys.exit(main())
