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
    DEFAULT_LIMIT_MARGIN_RAD, exceeds_joint_limit,
)

ARG_CHECKS = [
    ("torque", "torque"), ("max-vel", "speed"), ("max-time", "positive"),
    ("max-turns", "positive"), ("settle-pos-tol", "positive"),
    ("settle-time", "positive"), ("settle-timeout", "positive"),
    ("settle-max-vel", "speed"), ("feedback-timeout", "positive"),
    ("poll-timeout", "positive"), ("rate", "nonneg"),
]

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
ANALYZE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "analyze_armature.py")
PAUSE_S = 1.0


def positive_int(value):
    value = int(value)
    if value < 1:
        raise argparse.ArgumentTypeError("Must be an integer of 1 or greater")
    return value


def parse_args():
    p = argparse.ArgumentParser(description="Measure and analyze motor armature response.")
    p.add_argument("--motor-id", type=lambda v: int(v, 0), required=True)
    p.add_argument("--model", choices=list(SPECS.keys()), required=True)
    p.add_argument("--torques", type=float, nargs="+", required=True,
                   help="Signed feedforward torques in N*m")
    p.add_argument("--repeats", type=positive_int, default=1, help="Repeats per torque")
    p.add_argument("--ignore-joint-limit", action="store_true",
                   help="Disable joint-limit checks for unloaded bench tests only")
    p.set_defaults(
        interface=DEFAULT_INTERFACE, host_id=HOST_ID,
        max_vel=5.0, max_time=2.0, max_turns=3.0,
        settle_pos_tol=0.01, settle_time=0.3, settle_timeout=5.0,
        settle_max_vel=3.0, feedback_timeout=0.3,
        poll_timeout=0.03, rate=0.0, out=None,
        limit_margin=DEFAULT_LIMIT_MARGIN_RAD,
    )
    return p.parse_args()


def confirm(args, model):
    rated = RATED_TORQUE[model]
    print(f"Motor: ID {args.motor_id}, {model.upper()}, {args.channel}")
    print(f"Torques: {args.torques} N*m x {args.repeats}")
    print(f"Stop limits: |vel|>={args.max_vel} rad/s, "
          f"t>={args.max_time}s, |dpos|>={args.max_turns} turn, "
          f"joint limit {'off' if args.ignore_joint_limit else f'margin {args.limit_margin} rad'}")
    print("WARNING: Secure the unloaded motor to the bench before continuing.")
    if any(abs(torque) > rated for torque in args.torques):
        print(f"WARNING: A test torque exceeds the {rated} N*m rated torque.")
    if any(abs(torque) > PEAK_TORQUE[model] for torque in args.torques):
        print(f"ERROR: A test torque exceeds the {PEAK_TORQUE[model]} N*m peak torque.")
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
            status = (f"pos={pos:+.4f} rad  vel={vel:+.4f} rad/s  tq={tq:+.3f} N*m  "
                      f"fault={fault:02X}  settled={now - ref_time:4.2f}/{args.settle_time:.2f}s")
        if now - last_print >= 0.05:
            print(f"\r  {status}    ", end="", flush=True)
            last_print = now
    print()
    if misses > 0:
        print(f"WARNING: {misses} replies were missed while waiting for the motor to settle.")
    return None


def run_capture(motor, torque_cmd, pos_start, args):
    rows = []
    misses = 0
    fault_count = 0
    t0 = time.monotonic()
    period = (1.0 / args.rate) if args.rate > 0 else 0.0
    stop_reason = "max_time"
    last_ok = t0
    try:
        while True:
            now = time.monotonic()
            elapsed = now - t0
            if elapsed >= args.max_time:
                stop_reason = "max_time"
                break
            motor.control(pos=0.0, vel=0.0, kp=0.0, kd=0.0, torque=torque_cmd)
            fb = motor.poll_feedback(timeout=args.poll_timeout)
            if fb is None:
                misses += 1
                if time.monotonic() - last_ok > args.feedback_timeout:
                    stop_reason = "feedback_lost"
                    break
                continue
            last_ok = time.monotonic()
            t_recv, pos, vel, tq, temp, fault = fb
            rows.append((t_recv - t0, pos - pos_start, vel, tq, temp, fault))
            if fault:
                fault_count += 1
            if abs(vel) >= args.max_vel:
                stop_reason = "max_vel"
                break
            if not args.ignore_joint_limit and exceeds_joint_limit(pos, args.motor_id, args.limit_margin):
                print(f"\nJoint limit reached at {pos:+.4f} rad ({math.degrees(pos):+.1f} deg).")
                stop_reason = "joint_limit"
                break
            if abs(pos - pos_start) >= args.max_turns * 2.0 * math.pi:
                stop_reason = "max_turns"
                break
            if period > 0:
                sleep_left = period - (time.monotonic() - now)
                if sleep_left > 0:
                    time.sleep(sleep_left)
    except KeyboardInterrupt:
        stop_reason = "keyboard_interrupt"
    return rows, stop_reason, fault_count


def save_csv(path, rows, meta):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        for key, value in meta.items():
            f.write(f"# {key}: {value}\n")
        f.write("t_s,pos_rel_rad,vel_rad_s,torque_meas_Nm,temp_C,fault_hex\n")
        for t_s, pos, vel, tq, temp, fault in rows:
            f.write(f"{t_s:.6f},{pos:.6f},{vel:.6f},{tq:.4f},{temp:.1f},{fault:02X}\n")


def capture_once(args, torque, run_index):
    args.torque = torque
    spec = SPECS[args.model]
    bus = can.Bus(channel=args.channel, interface=args.interface)
    motor = None
    rows, stop_reason, fault_count = [], "not_started", 0
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
            print(f"Fault register: 0x{fault_reg:08X}" +
                  (f" ({', '.join(bits)})" if bits else " (OK)"))

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
        if not args.ignore_joint_limit and exceeds_joint_limit(pos_start, args.motor_id, args.limit_margin):
            print(f"ERROR: Start position {pos_start:+.4f} rad "
                  f"({math.degrees(pos_start):+.1f} deg) is outside the joint limit.")
            motor.stop()
            return 1, None, "initial_joint_limit"
        print(f"Settled at {pos_start:+.4f} rad. Applying torque...")

        rows, stop_reason, fault_count = run_capture(motor, args.torque, pos_start, args)
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
    max_vel_reached = max(abs(r[2]) for r in rows)
    achieved_hz = (len(rows) - 1) / duration if duration > 0 else 0.0

    print(f"Capture: {stop_reason}, samples={len(rows)}, duration={duration*1000:.1f} ms, "
          f"rate={achieved_hz:.0f} Hz, max|vel|={max_vel_reached:.3f} rad/s")
    if len(rows) < 15:
        print("WARNING: Fewer than 15 samples were recorded.")
    if stop_reason == "feedback_lost":
        print(f"ERROR: Feedback was lost for more than {args.feedback_timeout}s. Discard this result.")
    if fault_count:
        print(f"WARNING: {fault_count}/{len(rows)} feedback frames reported a fault byte.")

    ts = datetime.now().strftime("%Y%m%dT%H%M%S_%f")
    fname = (f"id{args.motor_id}_{args.model}_{args.torque:+.3f}Nm_{ts}_r{run_index:02d}.csv"
             .replace("+", "p").replace("-", "m"))
    out_path = os.path.join(DATA_DIR, fname)

    meta = {
        "motor_id": args.motor_id,
        "model": args.model,
        "channel": args.channel,
        "host_id": f"0x{args.host_id:02X}",
        "torque_cmd_Nm": f"{args.torque:.4f}",
        "started_at": datetime.now().isoformat(timespec="seconds"),
        "stop_reason": stop_reason,
        "samples": len(rows),
        "duration_s": f"{duration:.4f}",
    }
    save_csv(out_path, rows, meta)
    print(f"Saved: {out_path}")
    return (0 if stop_reason in {"max_time", "max_vel", "max_turns"} else 1), out_path, stop_reason


def main():
    args = parse_args()
    args.channel = channel_for_id(args.motor_id)

    for torque in args.torques:
        args.torque = torque
        if report_invalid_args(validate_args(args, args.model, ARG_CHECKS)):
            return 1
    if not confirm(args, args.model):
        return 1

    plan = [torque for torque in args.torques for _ in range(args.repeats)]
    paths = []
    failed = False
    for index, torque in enumerate(plan, 1):
        print(f"\n[{index}/{len(plan)}] torque={torque:+.3f} N*m")
        rc, path, stop_reason = capture_once(args, torque, index)
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
