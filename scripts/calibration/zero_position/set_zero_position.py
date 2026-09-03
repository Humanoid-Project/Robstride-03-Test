#!/usr/bin/env python3
import argparse
import math
import struct
import sys
import time

import can

from robonex_common.joints import ACTUATED_JOINTS, CHANNEL_MOTOR_IDS
from robonex_common.protocol import (
    COMM_PARAMETER_READ,
    COMM_PARAMETER_WRITE,
    COMM_SAVE,
    COMM_SET_ZERO,
    COMM_STOP,
    DEFAULT_INTERFACE,
    HOST_ID,
    MECHANICAL_POSITION_INDEX,
    ZERO_STATUS_INDEX,
    build_arbitration_id,
    parse_arbitration_id,
)
from robonex_common.joints import channel_for_motor_id as channel_for_id

SAVE_PAYLOAD = bytes([0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08])
TWO_PI = 2.0 * math.pi
JOINT_MAP = {joint.motor_id: joint.hardware_name for joint in ACTUATED_JOINTS}




def joint_name(motor_id):
    return JOINT_MAP.get(motor_id, f"ID{motor_id}")


def read_mech_position(bus, host_id, motor_id, timeout=0.2):
    data = bytearray(8)
    struct.pack_into("<H", data, 0, MECHANICAL_POSITION_INDEX)
    bus.send(can.Message(
        arbitration_id=build_arbitration_id(COMM_PARAMETER_READ, host_id, motor_id),
        data=bytes(data), is_extended_id=True))

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=max(0.0, deadline - time.monotonic()))
        if msg is None or not msg.is_extended_id:
            continue
        comm_type, data16, destination = parse_arbitration_id(msg.arbitration_id)
        if comm_type != COMM_PARAMETER_READ or destination != host_id or (data16 & 0xFF) != motor_id:
            continue
        payload = bytes(msg.data)
        if len(payload) < 8 or int.from_bytes(payload[0:2], "little") != MECHANICAL_POSITION_INDEX:
            continue
        return struct.unpack_from("<f", payload, 4)[0]
    return None


def read_mech_position_retry(bus, host_id, motor_id, attempts=5, timeout=0.2):
    for _ in range(attempts):
        position = read_mech_position(bus, host_id, motor_id, timeout=timeout)
        if position is not None:
            return position
    return None


def stop_motor(bus, host_id, motor_id):
    bus.send(can.Message(
        arbitration_id=build_arbitration_id(COMM_STOP, host_id, motor_id),
        data=bytes(8), is_extended_id=True))
    time.sleep(0.02)


def set_mechanical_zero(bus, host_id, motor_id):
    data = bytearray(8)
    data[0] = 1
    bus.send(can.Message(
        arbitration_id=build_arbitration_id(COMM_SET_ZERO, host_id, motor_id),
        data=bytes(data), is_extended_id=True))
    deadline = time.monotonic() + 0.2
    while time.monotonic() < deadline:
        if bus.recv(timeout=max(0.0, deadline - time.monotonic())) is None:
            break


def write_uint8_parameter(bus, host_id, motor_id, index, value):
    data = bytearray(8)
    struct.pack_into("<H", data, 0, index)
    data[4] = value & 0xFF
    bus.send(can.Message(
        arbitration_id=build_arbitration_id(COMM_PARAMETER_WRITE, host_id, motor_id),
        data=bytes(data), is_extended_id=True))
    time.sleep(0.05)


def save_parameters(bus, host_id, motor_id):
    bus.send(can.Message(
        arbitration_id=build_arbitration_id(COMM_SAVE, host_id, motor_id),
        data=SAVE_PAYLOAD, is_extended_id=True))
    time.sleep(0.3)


def fmt(rad):
    return f"{rad:+8.4f} rad ({math.degrees(rad):+8.2f} deg)"


def angular_diff(a, b):
    return (a - b + math.pi) % TWO_PI - math.pi


def parse_args():
    parser = argparse.ArgumentParser(
        description="Set the current position of motors 1-12 as mechanical zero.")
    parser.add_argument("--channels", nargs="+", default=list(CHANNEL_MOTOR_IDS),
                        choices=list(CHANNEL_MOTOR_IDS),
                        help=f"CAN channels. Default: {' '.join(CHANNEL_MOTOR_IDS)}")
    parser.add_argument("--interface", default=DEFAULT_INTERFACE, help="python-can interface")
    parser.add_argument("--host-id", type=lambda v: int(v, 0), default=HOST_ID, help="Host CAN ID")
    parser.add_argument("--save", action="store_true", help="Send a type-22 save frame after zeroing")
    parser.add_argument("--zero-sta", type=int, choices=[0, 1],
                        help="Power-on wrap: 0=0..2pi, 1=-pi..pi (implies --save)")
    parser.add_argument("--tolerance", type=float, default=0.05,
                        help="Success band around 0 rad. Default: 0.05")
    parser.add_argument("--yes", action="store_true", help="Skip the confirmation prompt")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.zero_sta is not None:
        args.save = True

    if not sys.stdin.isatty() and not args.yes:
        print("Need a confirmation prompt. Use --yes or run in a terminal.")
        return 1

    active_ids = sorted(
        motor_id
        for channel in args.channels
        for motor_id in CHANNEL_MOTOR_IDS[channel]
    )

    buses = {}
    try:
        for channel in args.channels:
            buses[channel] = can.Bus(channel=channel, interface=args.interface)

        print("Reading current positions...\n")
        print(f"{'ID':>3}  {'ch':<5}  {'joint':<18}  {'position':>26}")
        print("-" * 62)
        before = {}
        missing = []
        for motor_id in active_ids:
            channel = channel_for_id(motor_id)
            bus = buses[channel]
            position = read_mech_position_retry(bus, args.host_id, motor_id)
            if position is None:
                missing.append(motor_id)
                print(f"{motor_id:>3}  {channel:<5}  {joint_name(motor_id):<18}  {'no reply':>26}")
                continue
            before[motor_id] = position
            print(f"{motor_id:>3}  {channel:<5}  {joint_name(motor_id):<18}  {fmt(position):>26}")

        if missing:
            print(f"\nNo-reply IDs (skipped): {', '.join(str(i) for i in missing)}")

        if not before:
            print("\nNo motors available to zero. Stopping.")
            return 1

        print(f"\nWill set the current position of {len(before)} motors as mechanical zero.")
        print("Each motor is disabled first. Older firmware can lunge toward the old")
        print("target if you zero while still enabled.\n")

        if not args.yes:
            answer = input("Press Enter to continue, Ctrl-C to cancel: ")
            del answer

        print("\nSetting zero...")
        print(f"{'ID':>3}  {'ch':<5}  {'joint':<18}  {'after':>26}  result")
        print("-" * 80)
        ok_count = 0
        for motor_id in sorted(before):
            channel = channel_for_id(motor_id)
            bus = buses[channel]

            stop_motor(bus, args.host_id, motor_id)

            if args.zero_sta is not None:
                write_uint8_parameter(bus, args.host_id, motor_id, ZERO_STATUS_INDEX, args.zero_sta)

            set_mechanical_zero(bus, args.host_id, motor_id)

            if args.save:
                save_parameters(bus, args.host_id, motor_id)

            after = read_mech_position_retry(bus, args.host_id, motor_id)
            if after is None:
                print(f"{motor_id:>3}  {channel:<5}  {joint_name(motor_id):<18}  {'no reply':>26}  fail")
                continue

            success = abs(angular_diff(after, 0.0)) <= args.tolerance
            if success:
                ok_count += 1
            result = "OK" if success else "WARNING: not at 0"
            print(f"{motor_id:>3}  {channel:<5}  {joint_name(motor_id):<18}  {fmt(after):>26}  {result}")

        print(f"\nZeroed {ok_count}/{len(before)} motors.")
        return 0
    except KeyboardInterrupt:
        print("\nCancelled.")
        return 0
    finally:
        for bus in buses.values():
            bus.shutdown()


if __name__ == "__main__":
    sys.exit(main())
