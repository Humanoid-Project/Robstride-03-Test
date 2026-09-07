#!/usr/bin/env python3
import argparse
import math
import sys
import time

import can
from robonex_common.can import FeedbackHub, Motor
from robonex_common.joints import ACTUATED_JOINTS
from robonex_common.protocol import DEFAULT_INTERFACE, HOST_ID

REFRESH_DEFAULT_HZ = 10.0
STALE_DEFAULT_S = 0.3


def parse_args():
    parser = argparse.ArgumentParser(
        description="Passively display type 0x02 torque feedback from all 12 RoboNex motors."
    )
    parser.add_argument("--refresh", type=float, default=REFRESH_DEFAULT_HZ,
                        help="Terminal refresh rate in Hz")
    parser.add_argument("--stale-after", type=float, default=STALE_DEFAULT_S,
                        help="Mark feedback stale after this many seconds")
    return parser.parse_args()


def validate_args(args):
    problems = []
    if not math.isfinite(args.refresh) or args.refresh <= 0.0:
        problems.append("--refresh must be finite and positive.")
    if not math.isfinite(args.stale_after) or args.stale_after <= 0.0:
        problems.append("--stale-after must be finite and positive.")
    return problems


def open_monitor(interface, host_id):
    buses = {}
    motors = {}
    try:
        for channel in sorted({joint.channel for joint in ACTUATED_JOINTS}):
            buses[channel] = can.Bus(channel=channel, interface=interface)
        for joint in ACTUATED_JOINTS:
            motors[joint.motor_id] = Motor(
                buses[joint.channel], joint.motor_id, joint.motor_model, host_id=host_id
            )
    except Exception:
        for bus in buses.values():
            try:
                bus.shutdown()
            except Exception:
                pass
        raise
    hubs = {
        channel: FeedbackHub(
            bus,
            [motors[joint.motor_id] for joint in ACTUATED_JOINTS if joint.channel == channel],
            host_id,
        )
        for channel, bus in buses.items()
    }
    return buses, motors, hubs


def status_text(motor, now, stale_after):
    if motor.last_feedback_time <= 0.0 or motor.last_position is None:
        return "--", "--", "--", "WAITING"
    age = max(0.0, now - motor.last_feedback_time)
    if motor.last_fault:
        status = f"FAULT 0x{motor.last_fault:02X}"
    elif age > stale_after:
        status = "STALE"
    else:
        status = "OK"
    return (
        f"{motor.last_torque:+8.3f}",
        f"{motor.last_temp:6.1f}",
        f"{age * 1000.0:7.1f}",
        status,
    )


def render(motors, started_at, stale_after, first_frame):
    now = time.monotonic()
    lines = [
        "RoboNex torque monitor | passive type 0x02 receiver | Ctrl-C to exit",
        f"elapsed {now - started_at:8.1f} s",
        "",
        f"{'ID':>2}  {'CAN':<4}  {'joint':<19} {'model':<5} "
        f"{'torque N*m':>10} {'temp C':>7} {'age ms':>8}  status",
        "-" * 79,
    ]
    for joint in ACTUATED_JOINTS:
        torque, temp, age, status = status_text(
            motors[joint.motor_id], now, stale_after
        )
        lines.append(
            f"{joint.motor_id:>2}  {joint.channel:<4}  {joint.hardware_name:<19} "
            f"{joint.motor_model.upper():<5} {torque:>10} {temp:>7} {age:>8}  {status}"
        )
    lines.extend([
        "",
        "This monitor sends no CAN frames. Start the motor controller separately.",
    ])
    prefix = "\033[2J\033[H" if first_frame else "\033[H"
    sys.stdout.write(prefix + "\n".join(lines) + "\033[J")
    sys.stdout.flush()


def run(args):
    buses = {}
    try:
        buses, motors, hubs = open_monitor(DEFAULT_INTERFACE, HOST_ID)
        period = 1.0 / args.refresh
        started_at = time.monotonic()
        next_render = started_at
        first_frame = True
        while True:
            for hub in hubs.values():
                hub.pump()
            now = time.monotonic()
            if now >= next_render:
                render(motors, started_at, args.stale_after, first_frame)
                first_frame = False
                next_render = now + period
            sleep_time = min(0.002, max(0.0, next_render - time.monotonic()))
            if sleep_time > 0.0:
                time.sleep(sleep_time)
    except KeyboardInterrupt:
        return 0
    except (OSError, can.CanError) as error:
        print(f"ERROR: CAN monitor failed: {error}", file=sys.stderr)
        return 1
    finally:
        for bus in buses.values():
            try:
                bus.shutdown()
            except Exception:
                pass
        if sys.stdout.isatty():
            sys.stdout.write("\033[0m\n")
            sys.stdout.flush()


def main():
    args = parse_args()
    problems = validate_args(args)
    if problems:
        print("Invalid arguments:")
        for problem in problems:
            print(f"  {problem}")
        return 1
    if not sys.stdout.isatty():
        print("ERROR: Run this monitor in an interactive terminal.")
        return 1
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
