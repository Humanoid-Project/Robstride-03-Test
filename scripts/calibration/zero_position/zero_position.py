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
ZERO_TOLERANCE = 0.05
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
    parser.add_argument("--pos-range", type=int, choices=[0, 1], default=1,
                        help="Set power-on position wrapping: 0=0..2pi, 1=-pi..pi")
    return parser.parse_args()


def main():
    args = parse_args()
    if not sys.stdin.isatty():
        print("This command requires an interactive terminal for confirmation.")
        return 1

    active_ids = sorted(
        motor_id
        for channel in CHANNEL_MOTOR_IDS
        for motor_id in CHANNEL_MOTOR_IDS[channel]
    )

    buses = {}
    try:
        for channel in CHANNEL_MOTOR_IDS:
            buses[channel] = can.Bus(channel=channel, interface=DEFAULT_INTERFACE)

        print("Scanning motors...")
        before = {}
        missing = []
        for motor_id in active_ids:
            channel = channel_for_id(motor_id)
            bus = buses[channel]
            position = read_mech_position_retry(bus, HOST_ID, motor_id)
            if position is None:
                missing.append(motor_id)
                continue
            before[motor_id] = position

        print(f"Found {len(before)}/{len(active_ids)} motors.")
        if missing:
            print(f"Skipped IDs (no reply): {', '.join(str(i) for i in missing)}")

        if not before:
            print("No motors available. Stopping.")
            return 1

        position_range = "-pi to pi" if args.pos_range == 1 else "0 to 2pi"
        print(f"Position range: {position_range}.")
        print("WARNING: Motors will be disabled, zeroed at their current positions, and saved.")

        answer = input("Press Enter to continue or Ctrl-C to cancel: ")
        del answer

        print("Zeroing motors...")
        ok_count = 0
        for motor_id in sorted(before):
            channel = channel_for_id(motor_id)
            bus = buses[channel]

            stop_motor(bus, HOST_ID, motor_id)

            write_uint8_parameter(bus, HOST_ID, motor_id, ZERO_STATUS_INDEX, args.pos_range)

            set_mechanical_zero(bus, HOST_ID, motor_id)

            save_parameters(bus, HOST_ID, motor_id)

            after = read_mech_position_retry(bus, HOST_ID, motor_id)
            if after is None:
                print(f"ID {motor_id} ({channel}, {joint_name(motor_id)}): FAILED - no reply")
                continue

            success = abs(angular_diff(after, 0.0)) <= ZERO_TOLERANCE
            if success:
                ok_count += 1
            result = "OK" if success else "WARNING - not at zero"
            print(f"ID {motor_id} ({channel}, {joint_name(motor_id)}): {result}, {fmt(after)}")

        print(f"Result: {ok_count}/{len(before)} motors zeroed.")
        return 0 if ok_count == len(before) else 1
    except KeyboardInterrupt:
        print("\nCancelled.")
        return 130
    finally:
        for bus in buses.values():
            bus.shutdown()


if __name__ == "__main__":
    sys.exit(main())
