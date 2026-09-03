#!/usr/bin/env python3
import argparse
import math
import sys
import time

import can
from robonex_common.can import FeedbackHub, Motor
from robonex_common.joints import ACTUATED_JOINTS
from robonex_common.motors import MOTOR_SPECS
from robonex_common.protocol import DEFAULT_INTERFACE, HOST_ID, clamp
from robonex_common.joints import channel_for_motor_id as channel_for_id

CHANNEL_ID_RANGES = {
    "can0": range(1, 7),
    "can1": range(7, 13),
}

MOTORS = {
    1: {"target_rad": 0.0119, "model": "rs02"},
    2: {"target_rad": 0.4013, "model": "rs03"},
    3: {"target_rad": -0.1473, "model": "rs03"},
    4: {"target_rad": -0.5475, "model": "rs03"},
    5: {"target_rad": 0.0, "model": "rs02"},
    6: {"target_rad": 0.0, "model": "rs02"},
    7: {"target_rad": 0.0016, "model": "rs02"},
    8: {"target_rad": -0.6227, "model": "rs03"},
    9: {"target_rad": 0.0114, "model": "rs03"},
    10: {"target_rad": 0.7454, "model": "rs03"},
    11: {"target_rad": 0.0, "model": "rs02"},
    12: {"target_rad": 0.0001, "model": "rs02"},
}

MOVE_SPEED = 0.4
MIN_MOVE_TIME = 3.0
RATE = 100.0
HOLD_KP = 40.0
HOLD_KD = 2.0
OVERSPEED_STOP = 2.0
FEEDBACK_TIMEOUT = 0.3
SPECS = MOTOR_SPECS
JOINT_MAP = {joint.motor_id: joint.hardware_name for joint in ACTUATED_JOINTS}




def fmt(rad):
    return f"{rad:+8.4f} rad ({math.degrees(rad):+8.2f} deg)"


def main():
    parser = argparse.ArgumentParser(
        description="Move motors slowly to target angles and hold until stopped.")
    parser.add_argument("--channels", nargs="+", default=list(CHANNEL_ID_RANGES.keys()),
                        choices=list(CHANNEL_ID_RANGES.keys()),
                        help=f"CAN channels. Default: {' '.join(CHANNEL_ID_RANGES.keys())}")
    parser.add_argument("--interface", default=DEFAULT_INTERFACE, help="python-can interface")
    parser.add_argument("--host-id", type=lambda v: int(v, 0), default=HOST_ID, help="Host CAN ID")
    parser.add_argument("--yes", action="store_true", help="Skip the confirmation prompt")
    args = parser.parse_args()

    active_motor_ids = [mid for mid in MOTORS if channel_for_id(mid) in args.channels]
    if not active_motor_ids:
        print("No configured motor uses the selected channels.")
        return 1

    buses = {}
    motors = {}
    try:
        needed_channels = sorted({channel_for_id(mid) for mid in active_motor_ids})
        for channel in needed_channels:
            buses[channel] = can.Bus(channel=channel, interface=args.interface)

        for motor_id in active_motor_ids:
            cfg = MOTORS[motor_id]
            spec = SPECS[cfg["model"]]
            bus = buses[channel_for_id(motor_id)]
            motors[motor_id] = {
                "motor": Motor(bus, motor_id, spec, host_id=args.host_id),
                "target_cfg": cfg["target_rad"],
                "target": None,
                "start": None,
            }

        hubs = {
            channel: FeedbackHub(
                bus,
                [m["motor"] for mid, m in motors.items() if channel_for_id(mid) == channel],
                args.host_id,
            )
            for channel, bus in buses.items()
        }
        hub_of = {mid: hubs[channel_for_id(mid)] for mid in motors}

        print(f"Reading current positions on {', '.join(needed_channels)}...\n")
        print(f"{'ID':>3}  {'ch':<5}  {'joint':<18}  {'model':<5}  {'current':>26}  {'target':>26}")
        print("-" * 104)
        move_time = MIN_MOVE_TIME
        for motor_id, m in motors.items():
            current = m["motor"].read_mech_position(timeout=0.3)
            channel = channel_for_id(motor_id)
            if current is None:
                print(f"{motor_id:>3}  {channel:<5}  {JOINT_MAP.get(motor_id, '?'):<18}  "
                      f"{m['motor'].spec.name:<5}  {'no response':>26}")
                print(f"\nMotor ID {motor_id} did not respond. Check power, wiring, and ID.")
                return 1
            target = current if m["target_cfg"] is None else m["target_cfg"]
            m["target"] = clamp(target, m["motor"].spec.p_min, m["motor"].spec.p_max)
            travel = abs(m["target"] - current)
            move_time = max(move_time, travel / MOVE_SPEED)
            note = "" if m["target_cfg"] is not None else "  (no target: hold current position)"
            print(f"{motor_id:>3}  {channel:<5}  {JOINT_MAP.get(motor_id, '?'):<18}  "
                  f"{m['motor'].spec.name:<5}  {fmt(current):>26}  {fmt(m['target']):>26}{note}")

        print(f"\nMove time: about {move_time:.1f} s at {MOVE_SPEED} rad/s, "
              f"hold gains kp={HOLD_KP}, kd={HOLD_KD}")

        if not args.yes:
            if not sys.stdin.isatty():
                print("Confirmation requires an interactive terminal or --yes.")
                return 1
            try:
                answer = input("\nPress Enter to start, or Ctrl-C to cancel: ")
                del answer
            except (KeyboardInterrupt, EOFError):
                print("\nCancelled.")
                return 0

        def emergency_stop(reason):
            print(f"\nEmergency stop: {reason}")
            for mm in motors.values():
                try:
                    mm["motor"].stop()
                except can.CanError:
                    pass

        print("\nEnabling...")
        for motor_id, m in motors.items():
            m["motor"].write_run_mode_operation()
            time.sleep(0.005)
            m["motor"].enable()
            start = hub_of[motor_id].wait_for(motor_id, timeout=0.3)
            if start is None:
                emergency_stop(f"ID {m['motor'].motor_id} has no live feedback")
                return 1
            m["start"] = start
            if m["target_cfg"] is None:
                m["target"] = start
            m["motor"].control(pos=start, vel=0.0, kp=HOLD_KP, kd=HOLD_KD)

        move_time = MIN_MOVE_TIME
        for m in motors.values():
            move_time = max(move_time, abs(m["target"] - m["start"]) / MOVE_SPEED)

        period = 1.0 / RATE

        def check_safety(now):

            for mid, mm in motors.items():
                motor = mm["motor"]
                if abs(motor.last_velocity) > OVERSPEED_STOP:
                    return f"ID {mid} overspeed ({motor.last_velocity:+.2f} rad/s)"
                if motor.last_feedback_time <= 0.0:
                    return f"ID {mid} never received feedback"
                age = now - motor.last_feedback_time
                if age > FEEDBACK_TIMEOUT:
                    return f"ID {mid} feedback timeout ({age:.2f} s > {FEEDBACK_TIMEOUT} s)"
            return None

        print(f"Starting move ({move_time:.1f} s)...")
        start_time = time.monotonic()
        while True:
            now = time.monotonic()
            progress = min(1.0, (now - start_time) / move_time)
            smooth = progress * progress * (3.0 - 2.0 * progress)
            smooth_vel = 6.0 * progress * (1.0 - progress) / move_time
            for m in motors.values():
                travel = m["target"] - m["start"]
                m["motor"].control(
                    pos=m["start"] + travel * smooth,
                    vel=clamp(travel * smooth_vel, -MOVE_SPEED, MOVE_SPEED),
                    kp=HOLD_KP, kd=HOLD_KD,
                )
            for hub in hubs.values():
                hub.pump()
            reason = check_safety(time.monotonic())
            if reason:
                emergency_stop(reason)
                return 1
            if progress >= 1.0:
                break
            sleep = period - (time.monotonic() - now)
            if sleep > 0:
                time.sleep(sleep)

        print("\nTargets reached. Holding position. Press Ctrl-C to stop.\n")
        last_print = 0.0
        while True:
            now = time.monotonic()
            for m in motors.values():
                m["motor"].control(pos=m["target"], vel=0.0, kp=HOLD_KP, kd=HOLD_KD)
            for hub in hubs.values():
                hub.pump()
            reason = check_safety(time.monotonic())
            if reason:
                emergency_stop(reason)
                return 1
            if now - last_print >= 1.0:
                last_print = now
                print(f"[{time.strftime('%H:%M:%S')}] Holding position")
                print(f"  {'ID':>3} {'joint':<16} {'current→target deg':>20} "
                      f"{'error deg':>10} {'torque N·m':>18} {'temp':>6}")
                for mid, m in motors.items():
                    mm = m["motor"]
                    tmax = mm.spec.t_max
                    cur = mm.last_position
                    if cur is None:
                        line = f"  {mid:>3} {JOINT_MAP.get(mid, '?'):<16} {'(no feedback)':>20}"
                    else:
                        err = math.degrees(m["target"] - cur)
                        tq_pct = abs(mm.last_torque) / tmax * 100.0 if tmax else 0.0
                        line = (f"  {mid:>3} {JOINT_MAP.get(mid, '?'):<16} "
                                f"{math.degrees(cur):+7.1f}→{math.degrees(m['target']):+7.1f}"
                                f" {err:+10.1f}"
                                f" {mm.last_torque:+7.2f}/{tmax:.0f}({tq_pct:3.0f}%)"
                                f" {mm.last_temp:5.1f}C")
                    print(line)
                print()
            sleep = period - (time.monotonic() - now)
            if sleep > 0:
                time.sleep(sleep)

    except KeyboardInterrupt:
        print("\n\nStop requested. Stopping motors...")
    finally:
        for m in motors.values():
            try:
                m["motor"].stop()
            except can.CanError:
                pass
        for bus in buses.values():
            bus.shutdown()
        print("Stopped.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
