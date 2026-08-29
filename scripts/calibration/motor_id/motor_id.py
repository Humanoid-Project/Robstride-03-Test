#!/usr/bin/env python3
import argparse
import struct
import sys
import time
from pathlib import Path

import can

from robonex_common.joints import CHANNEL_MOTOR_IDS
from robonex_common.protocol import (
    COMM_DEVICE_ID,
    COMM_PARAMETER_READ,
    COMM_SET_CAN_ID,
    COMM_STOP,
    DEFAULT_INTERFACE,
    DEVICE_ID_DESTINATION,
    HOST_ID,
    MECHANICAL_POSITION_INDEX,
    build_arbitration_id,
    parse_arbitration_id,
)

QUERY_TIMEOUT = 0.25
SCAN_TIMEOUT = 0.08


class MotorIdError(RuntimeError):
    pass


def parse_id(value):
    motor_id = int(value, 0)
    if not 0 <= motor_id <= 127:
        raise argparse.ArgumentTypeError("ID must be 0-127.")
    return motor_id


def parse_new_id(value):
    motor_id = int(value, 0)
    if not 1 <= motor_id <= 127:
        raise argparse.ArgumentTypeError("New ID must be 1-127.")
    return motor_id


def send(bus, comm_type, data16, target_id, data=None):
    message = can.Message(
        arbitration_id=build_arbitration_id(comm_type, data16, target_id),
        data=bytes(8) if data is None else data,
        is_extended_id=True,
    )
    try:
        bus.send(message)
    except can.CanError as exc:
        raise MotorIdError(f"CAN send failed: {exc}") from exc


def receive_matching(bus, comm_type, destination, target_id, timeout, index=None):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            message = bus.recv(timeout=max(0.0, deadline - time.monotonic()))
        except can.CanError as exc:
            raise MotorIdError(f"CAN recv failed: {exc}") from exc
        if message is None or not message.is_extended_id:
            continue
        reply_type, data16, reply_destination = parse_arbitration_id(message.arbitration_id)
        payload = bytes(message.data)
        if reply_type != comm_type or reply_destination != destination:
            continue
        if (data16 & 0xFF) != target_id:
            continue
        if index is not None:
            if len(payload) < 2 or int.from_bytes(payload[:2], "little") != index:
                continue
        return payload
    return None


def query_device_id(bus, target_id, timeout=QUERY_TIMEOUT):
    send(bus, COMM_DEVICE_ID, HOST_ID, target_id)
    payload = receive_matching(
        bus,
        COMM_DEVICE_ID,
        DEVICE_ID_DESTINATION,
        target_id,
        timeout,
    )
    if payload is None:
        return None
    return {"motor_id": target_id, "uid": payload.hex(), "source": "device-id"}


def query_mech_position(bus, target_id, timeout=QUERY_TIMEOUT):
    payload = bytearray(8)
    struct.pack_into("<H", payload, 0, MECHANICAL_POSITION_INDEX)
    send(bus, COMM_PARAMETER_READ, HOST_ID, target_id, bytes(payload))
    reply = receive_matching(
        bus,
        COMM_PARAMETER_READ,
        HOST_ID,
        target_id,
        timeout,
        index=MECHANICAL_POSITION_INDEX,
    )
    if reply is None:
        return None
    return {"motor_id": target_id, "uid": None, "source": "mechPos"}


def probe_motor(bus, target_id, timeout=QUERY_TIMEOUT):
    result = query_device_id(bus, target_id, timeout)
    if result is not None:
        return result
    return query_mech_position(bus, target_id, timeout)


def link_state(channel):
    try:
        return Path(f"/sys/class/net/{channel}/operstate").read_text().strip().lower()
    except OSError:
        return None


def open_bus(channel):
    if link_state(channel) == "down":
        raise MotorIdError(
            f"{channel} is DOWN. Bring the CAN interface up first."
        )
    try:
        return can.Bus(channel=channel, interface=DEFAULT_INTERFACE)
    except (can.CanError, OSError) as exc:
        raise MotorIdError(f"Failed to open {channel}: {exc}") from exc


def print_found(result):
    suffix = f"  UID={result['uid']}" if result["uid"] else "  mechPos reply"
    print(f"  [OK] ID {result['motor_id']:3d} (0x{result['motor_id']:02X}){suffix}")


def scan_ids(bus, targets, timeout):
    found = {}
    for target_id in targets:
        result = probe_motor(bus, target_id, timeout)
        if result is None:
            print(f"  [--] ID {target_id:3d} (0x{target_id:02X})  no reply")
        else:
            found[target_id] = result
            print_found(result)
        time.sleep(0.005)
    return found


def run_check(args):
    failed = False
    for channel in args.channels:
        targets = CHANNEL_MOTOR_IDS[channel]
        print(f"{channel}: checking standard IDs {targets[0]}-{targets[-1]}")
        bus = None
        try:
            bus = open_bus(channel)
            found = scan_ids(bus, targets, SCAN_TIMEOUT)
        except MotorIdError as exc:
            print(f"  [ERROR] {exc}")
            failed = True
            continue
        finally:
            if bus is not None:
                bus.shutdown()
        missing = [motor_id for motor_id in targets if motor_id not in found]
        if missing:
            failed = True
            print(f"  missing: {', '.join(map(str, missing))}")
        else:
            print("  All standard IDs replied.")
    return 1 if failed else 0


def run_find(args):
    bus = open_bus(args.channel)
    try:
        if args.motor_id is not None:
            result = probe_motor(bus, args.motor_id, QUERY_TIMEOUT)
            if result is None:
                print(f"ID {args.motor_id} (0x{args.motor_id:02X}): no reply")
                return 1
            print_found(result)
            return 0
        print(f"{args.channel}: scanning IDs 0-{args.scan_max}")
        found = scan_ids(bus, range(args.scan_max + 1), SCAN_TIMEOUT)
        if not found:
            print("No motors replied.")
            return 1
        print(f"Found IDs: {', '.join(map(str, sorted(found)))}")
        return 0
    finally:
        bus.shutdown()


def send_stop(bus, motor_id):
    send(bus, COMM_STOP, HOST_ID, motor_id)


def send_set_id(bus, current_id, new_id):
    data16 = ((new_id & 0xFF) << 8) | HOST_ID
    send(bus, COMM_SET_CAN_ID, data16, current_id)


def verify_change(bus, current_id, new_id, previous_uid):
    new_result = None
    for _ in range(10):
        new_result = probe_motor(bus, new_id, QUERY_TIMEOUT)
        if new_result is not None:
            break
        time.sleep(0.2)
    old_result = probe_motor(bus, current_id, QUERY_TIMEOUT)
    if new_result is not None and old_result is None:
        if previous_uid and new_result["uid"] and previous_uid != new_result["uid"]:
            print("Verify failed: a different UID replied on the new ID. Possible ID collision.")
            return 1
        print(f"OK: motor replies only on ID {new_id} (0x{new_id:02X}).")
        return 0
    if new_result is None and old_result is not None:
        print(f"Change failed: motor still replies on old ID {current_id}.")
        return 1
    if new_result is not None and old_result is not None:
        print("Verify failed: both old and new IDs reply. Duplicate motors or ID collision.")
        return 1
    print("Cannot verify: neither ID replies. Power-cycle and run find.")
    return 1


def run_set(args):
    if args.current_id == args.new_id:
        print("Current ID and new ID are the same; nothing to change.")
        return 0
    bus = open_bus(args.channel)
    try:
        current = probe_motor(bus, args.current_id, QUERY_TIMEOUT)
        if current is None:
            print(f"Stopped: current ID {args.current_id} did not reply.")
            return 1
        occupied = probe_motor(bus, args.new_id, QUERY_TIMEOUT)
        if occupied is not None:
            print(f"Stopped: new ID {args.new_id} already replies on {args.channel}.")
            return 1
        print("Warning: every motor that currently uses this ID will change to the new ID.")
        print("Connect only the target motor before changing IDs.")
        expected = f"CHANGE {args.current_id} {args.new_id}"
        try:
            reply = input(f"Type '{expected}' exactly to make a permanent change: ")
        except EOFError:
            reply = ""
        if reply.strip() != expected:
            print("Cancelled.")
            return 1
        send_stop(bus, args.current_id)
        time.sleep(0.05)
        send_set_id(bus, args.current_id, args.new_id)
        time.sleep(0.2)
        print(f"Sent ID {args.current_id} -> {args.new_id}. Verifying.")
        return verify_change(bus, args.current_id, args.new_id, current["uid"])
    finally:
        bus.shutdown()


def build_parser():
    parser = argparse.ArgumentParser(description="Check, find, or change a Robstride motor CAN ID.")
    commands = parser.add_subparsers(dest="command", required=True)

    check = commands.add_parser("check", help="Check the standard can0/can1 ID layout")
    check.add_argument(
        "--channels",
        nargs="+",
        choices=tuple(CHANNEL_MOTOR_IDS),
        default=list(CHANNEL_MOTOR_IDS),
    )
    check.set_defaults(handler=run_check)

    find = commands.add_parser("find", help="Search for motor IDs on one channel")
    find.add_argument("--channel", choices=tuple(CHANNEL_MOTOR_IDS), required=True)
    find.add_argument("--motor-id", type=parse_id)
    find.add_argument("--scan-max", type=parse_id, default=127)
    find.set_defaults(handler=run_find)

    set_id = commands.add_parser("set", help="Permanently change a motor CAN ID")
    set_id.add_argument("--channel", choices=tuple(CHANNEL_MOTOR_IDS), required=True)
    set_id.add_argument("--current-id", type=parse_id, required=True)
    set_id.add_argument("--new-id", type=parse_new_id, required=True)
    set_id.set_defaults(handler=run_set)
    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()
    try:
        return args.handler(args)
    except MotorIdError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\nInterrupted.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
