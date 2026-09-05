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
    HOST_ID, DEFAULT_INTERFACE, SPECS, RATED_TORQUE, PEAK_TORQUE,
    RUN_MODE_INDEX, RUN_MODE_OPERATION, MECH_POS_INDEX, FAULT_STA_INDEX,
    Motor, channel_for_id, decode_fault_bits, active_brake,
    validate_args, report_invalid_args,
    JOINT_LIMITS_RAD, DEFAULT_LIMIT_MARGIN_RAD, joint_limit_for, exceeds_joint_limit,
)

ARG_CHECKS = [
    ("start-torque", "torque"), ("step", "positive"), ("max-torque", "torque"),
    ("probe-time", "positive"), ("move-threshold", "positive"),
    ("max-vel", "speed"), ("max-time", "positive"),
    ("settle-pos-tol", "positive"), ("settle-time", "positive"),
    ("settle-timeout", "positive"), ("settle-max-vel", "speed"),
    ("feedback-timeout", "positive"),
]

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
ANALYZE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "analyze_friction.py")
PAUSE_S = 1.0


def positive_int(value):
    value = int(value)
    if value < 1:
        raise argparse.ArgumentTypeError("Must be an integer of 1 or greater")
    return value


def parse_args():
    p = argparse.ArgumentParser(description="Measure and analyze motor breakaway friction.")
    p.add_argument("--motor-id", type=lambda v: int(v, 0), required=True)
    p.add_argument("--model", choices=list(SPECS.keys()), required=True)
    p.add_argument("--signs", type=int, nargs="+", choices=[1, -1], default=[1, -1],
                   help="Motor torque directions")
    p.add_argument("--repeats", type=positive_int, default=1, help="Repeats per direction")
    p.add_argument("--ignore-joint-limit", action="store_true",
                   help="Disable joint-limit checks for unloaded bench tests only")
    p.set_defaults(
        interface=DEFAULT_INTERFACE, host_id=HOST_ID,
        start_torque=0.02, step=0.02, probe_time=0.3,
        move_threshold=0.03, max_torque=1.0, max_vel=3.0, max_time=30.0,
        settle_pos_tol=0.01, settle_time=0.3, settle_timeout=5.0,
        settle_max_vel=3.0, feedback_timeout=0.3, out=None,
        limit_margin=DEFAULT_LIMIT_MARGIN_RAD,
    )
    return p.parse_args()


def confirm(args, model):
    rated = RATED_TORQUE[model]
    n_steps_est = int(math.ceil((args.max_torque - args.start_torque) / args.step)) + 1
    print(f"Motor: ID {args.motor_id}, {model.upper()}, {args.channel}")
    print(f"Directions: {args.signs} x {args.repeats}")
    print(f"Torque search: {args.start_torque} to {args.max_torque} N*m, step={args.step} N*m")
    if args.ignore_joint_limit:
        print("WARNING: Joint-limit checks are disabled.")
    else:
        lo, hi = JOINT_LIMITS_RAD.get(args.motor_id, (None, None))
        if lo is None:
            print(f"ERROR: No joint limit is defined for motor ID {args.motor_id}.")
        else:
            print(f"Joint limit: {math.degrees(lo):+.1f} to {math.degrees(hi):+.1f} deg")
    print("WARNING: Verify zero calibration, secure the robot, and prepare the E-stop.")
    if args.max_torque > rated:
        print(f"WARNING: Maximum torque exceeds the {rated} N*m rated torque.")
    if args.max_torque > PEAK_TORQUE[model]:
        print(f"ERROR: Maximum torque exceeds the {PEAK_TORQUE[model]} N*m peak torque.")
        return False
    if not sys.stdin.isatty():
        print("ERROR: Run this command in an interactive terminal.")
        return False
    try:
        input("Press Enter to continue or Ctrl-C to cancel: ")
    except (KeyboardInterrupt, EOFError):
        print("\nCancelled.")
        return False
    return True


def wait_settled(motor, hold_pos, args):

    deadline = time.monotonic() + args.settle_timeout
    ref_pos = None
    ref_time = time.monotonic()
    last_pos = hold_pos
    misses = 0
    last_print = 0.0
    while time.monotonic() < deadline:
        motor.control(pos=0.0, vel=0.0, kp=0.0, kd=2.0, torque=0.0)
        fb = motor.poll_feedback(timeout=0.05)
        now = time.monotonic()
        if fb is None:
            misses += 1
            status = f"No reply (miss {misses})"
        else:
            _, pos, vel, tq, temp, fault = fb
            last_pos = pos
            if abs(vel) >= args.settle_max_vel:
                print()
                print(f"EMERGENCY STOP: |vel|={vel:+.3f} rad/s exceeds {args.settle_max_vel}.")
                motor.stop()
                return None
            if ref_pos is None or abs(pos - ref_pos) > args.settle_pos_tol:
                ref_pos = pos
                ref_time = now
            elif now - ref_time >= args.settle_time:
                print()
                return last_pos
            status = f"pos={pos:+.4f} rad  quiet={now - ref_time:4.2f}/{args.settle_time:.2f}s"
        if now - last_print >= 0.1:
            print(f"\r  {status}    ", end="", flush=True)
            last_print = now
    print()
    return None


def return_to_zero(motor, timeout=2.5, kp=15.0, kd=3.0, pos_tol=0.02, max_speed=0.4):
    fb = motor.poll_feedback(timeout=0.1)
    if fb is None:
        print("WARNING: Return position unavailable; sending stop only.")
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
            return last_pos
    print(f"WARNING: Failed to return to zero within {timeout}s; position={last_pos:+.4f} rad.")
    return last_pos


def run_search(motor, pos_start, args):
    rows = []
    misses = 0
    t0 = time.monotonic()
    current_torque = args.sign * args.start_torque
    step = args.sign * args.step
    max_torque_signed = args.sign * args.max_torque
    step_deadline = t0 + args.probe_time
    stop_reason = "max_time"
    last_print = 0.0
    step_start_index = 0
    mismatch_streak = 0
    steps_taken = 0
    last_ok = t0
    try:
        while True:
            now = time.monotonic()
            elapsed = now - t0
            if elapsed >= args.max_time:
                stop_reason = "max_time"
                break
            if now >= step_deadline:
                finished_rows = rows[step_start_index:]
                if finished_rows:
                    mean_tq = sum(r[4] for r in finished_rows) / len(finished_rows)
                    if abs(mean_tq - current_torque) > max(0.08, 0.5 * abs(current_torque)):
                        mismatch_streak += 1
                    else:
                        mismatch_streak = 0
                    if mismatch_streak >= 2:
                        stop_reason = "comm_mismatch"
                        break
                step_start_index = len(rows)
                current_torque += step
                steps_taken += 1
                step_deadline = now + args.probe_time
                print()
                print(f"Torque: {current_torque:+.3f} N*m")
                if abs(current_torque) > abs(max_torque_signed):
                    stop_reason = "max_torque_no_movement"
                    break
            motor.control(pos=0.0, vel=0.0, kp=0.0, kd=0.0, torque=current_torque)
            fb = motor.poll_feedback(timeout=0.05)
            if fb is None:
                misses += 1
                if time.monotonic() - last_ok > args.feedback_timeout:
                    stop_reason = "feedback_lost"
                    break
                continue
            last_ok = time.monotonic()
            t_recv, pos, vel, tq, temp, fault = fb
            rows.append((t_recv - t0, pos - pos_start, vel, current_torque, tq, temp, fault))
            if now - last_print >= 0.1:
                print(f"\r  torque={current_torque:+.3f} N*m  pos={pos - pos_start:+.4f} rad  "
                      f"vel={vel:+.3f} rad/s  tq={tq:+.3f} N*m    ", end="", flush=True)
                last_print = now
            if abs(vel) >= args.max_vel:
                stop_reason = "max_vel"
                break
            if not args.ignore_joint_limit and exceeds_joint_limit(pos, args.motor_id, args.limit_margin):
                print(f"\nJoint limit reached at {pos:+.4f} rad ({math.degrees(pos):+.1f} deg).")
                stop_reason = "joint_limit"
                break
            if abs(pos - pos_start) >= args.move_threshold:
                if steps_taken == 0:
                    stop_reason = "movement_at_start_suspicious"
                else:
                    stop_reason = "movement_detected"
                break
    except KeyboardInterrupt:
        stop_reason = "keyboard_interrupt"
    print()
    return rows, stop_reason, current_torque


def save_csv(path, rows, meta):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        for key, value in meta.items():
            f.write(f"# {key}: {value}\n")
        f.write("t_s,pos_rel_rad,vel_rad_s,torque_cmd_Nm,torque_meas_Nm,temp_C,fault_hex\n")
        for t_s, pos, vel, tcmd, tq, temp, fault in rows:
            f.write(f"{t_s:.6f},{pos:.6f},{vel:.6f},{tcmd:.4f},{tq:.4f},{temp:.1f},{fault:02X}\n")


def capture_once(args, sign, run_index):
    args.sign = sign
    spec = SPECS[args.model]
    bus = can.Bus(channel=args.channel, interface=args.interface)
    motor = None
    rows, stop_reason, torque_at_stop = [], "not_started", 0.0
    pos_start = None
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

        print("Waiting for the motor to settle...")
        pos_start = wait_settled(motor, initial_pos, args)
        if pos_start is None:
            print("ERROR: The motor did not settle before the safety timeout.")
            motor.stop()
            return 1, None, "not_settled"
        print(f"Settled at {pos_start:+.4f} rad. Starting torque search...")

        if not args.ignore_joint_limit and exceeds_joint_limit(pos_start, args.motor_id, args.limit_margin):
            print(f"ERROR: Start position {pos_start:+.4f} rad is outside the joint limit.")
            motor.stop()
            return 1, None, "initial_joint_limit"

        rows, stop_reason, torque_at_stop = run_search(motor, pos_start, args)
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
                return_to_zero(motor)
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
    achieved_hz = (len(rows) - 1) / duration if duration > 0 else 0.0

    print(f"Search: {stop_reason}, samples={len(rows)}, duration={duration*1000:.1f} ms, rate={achieved_hz:.0f} Hz")

    breakaway_low = breakaway_high = breakaway_mid = None
    if stop_reason == "movement_detected":
        breakaway_high = torque_at_stop
        breakaway_low = torque_at_stop - args.sign * args.step
        breakaway_mid = (breakaway_low + breakaway_high) / 2.0
        print(f"Breakaway bracket: [{min(breakaway_low, breakaway_high):.3f}, "
              f"{max(breakaway_low, breakaway_high):.3f}] N*m")
        print(f"Breakaway estimate: {breakaway_mid:+.3f} N*m (±{abs(args.step)/2:.3f})")
    elif stop_reason == "movement_at_start_suspicious":
        print(f"WARNING: Movement started at the first torque step ({torque_at_stop:+.3f} N*m); discard and repeat.")
    elif stop_reason == "max_torque_no_movement":
        print(f"WARNING: No movement up to {args.max_torque} N*m; check the mechanism.")
    elif stop_reason == "max_vel":
        print("WARNING: Safety speed limit reached before movement detection.")
    elif stop_reason == "feedback_lost":
        print(f"ERROR: Feedback was lost for more than {args.feedback_timeout}s; discard this result.")
    elif stop_reason == "comm_mismatch":
        print("ERROR: Commanded and measured torque disagree; discard this result and check CAN.")
    elif stop_reason == "joint_limit":
        print(f"WARNING: Joint limit reached at {torque_at_stop:+.3f} N*m before breakaway.")

    ts = datetime.now().strftime("%Y%m%dT%H%M%S_%f")
    sign_tag = "pos" if args.sign > 0 else "neg"
    fname = f"id{args.motor_id}_{args.model}_{sign_tag}_{ts}_r{run_index:02d}.csv"
    out_path = os.path.join(DATA_DIR, fname)

    meta = {
        "motor_id": args.motor_id,
        "model": args.model,
        "channel": args.channel,
        "host_id": f"0x{args.host_id:02X}",
        "sign": args.sign,
        "start_torque_Nm": f"{args.start_torque:.4f}",
        "step_Nm": f"{args.step:.4f}",
        "probe_time_s": f"{args.probe_time:.3f}",
        "move_threshold_rad": f"{args.move_threshold:.4f}",
        "max_torque_Nm": f"{args.max_torque:.4f}",
        "started_at": datetime.now().isoformat(timespec="seconds"),
        "pos_start_rad": f"{pos_start:.6f}" if pos_start is not None else "",
        "stop_reason": stop_reason,
        "breakaway_low_Nm": f"{breakaway_low:.4f}" if breakaway_low is not None else "",
        "breakaway_high_Nm": f"{breakaway_high:.4f}" if breakaway_high is not None else "",
        "breakaway_mid_Nm": f"{breakaway_mid:.4f}" if breakaway_mid is not None else "",
        "samples": len(rows),
        "duration_s": f"{duration:.4f}",
    }
    save_csv(out_path, rows, meta)
    print(f"Saved: {out_path}")
    return (0 if stop_reason == "movement_detected" else 1), out_path, stop_reason


def main():
    args = parse_args()
    args.channel = channel_for_id(args.motor_id)

    for sign in args.signs:
        args.sign = sign
        if report_invalid_args(validate_args(args, args.model, ARG_CHECKS)):
            return 1
    if args.start_torque > args.max_torque:
        print("ERROR: Internal start torque exceeds maximum torque.")
        return 1
    if not confirm(args, args.model):
        return 1

    plan = [sign for sign in args.signs for _ in range(args.repeats)]
    paths = []
    failed = False
    for index, sign in enumerate(plan, 1):
        print(f"\n[{index}/{len(plan)}] sign={sign:+d}")
        rc, path, stop_reason = capture_once(args, sign, index)
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
