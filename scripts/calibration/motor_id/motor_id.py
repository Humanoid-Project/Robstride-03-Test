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


def format_found(result):
    detail = f"UID={result['uid']}" if result["uid"] else "mechPos reply"
    return f"ID {result['motor_id']} ({detail})"


def scan_ids(bus, targets, timeout):
    found = {}
    for target_id in targets:
        result = probe_motor(bus, target_id, timeout)
        if result is not None:
            found[target_id] = result
        time.sleep(0.005)
    return found


def run_check(_args):
    failed = False
    for channel in CHANNEL_MOTOR_IDS:
        targets = CHANNEL_MOTOR_IDS[channel]
        bus = None
        try:
            bus = open_bus(channel)
            found = scan_ids(bus, targets, SCAN_TIMEOUT)
        except MotorIdError as exc:
            print(f"{channel}: ERROR - {exc}")
            failed = True
            continue
        finally:
            if bus is not None:
                bus.shutdown()
        missing = [motor_id for motor_id in targets if motor_id not in found]
        if missing:
            failed = True
            print(f"{channel}: missing IDs {', '.join(map(str, missing))}")
        else:
            print(f"{channel}: OK")
    return 1 if failed else 0


def run_find(args):
    found_any = False
    failed = False
    for channel in CHANNEL_MOTOR_IDS:
        bus = None
        try:
            bus = open_bus(channel)
            if args.motor_id is not None:
                result = probe_motor(bus, args.motor_id, QUERY_TIMEOUT)
                if result is None:
                    print(f"{channel}: ID {args.motor_id} not found")
                else:
                    print(f"{channel}: {format_found(result)}")
                    found_any = True
                continue
            found = scan_ids(bus, range(128), SCAN_TIMEOUT)
            if found:
                found_any = True
                results = ", ".join(format_found(found[motor_id]) for motor_id in sorted(found))
                print(f"{channel}: {results}")
            else:
                print(f"{channel}: no motors found")
        except MotorIdError as exc:
            print(f"{channel}: ERROR - {exc}")
            failed = True
        finally:
            if bus is not None:
                bus.shutdown()
    return 1 if failed or not found_any else 0


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
            print("ERROR: the new ID belongs to a different motor")
            return 1
        print(f"ID changed: {current_id} -> {new_id}")
        return 0
    if new_result is None and old_result is not None:
        print(f"ERROR: motor still uses ID {current_id}")
        return 1
    if new_result is not None and old_result is not None:
        print("ERROR: both old and new IDs replied")
        return 1
    print("ERROR: ID change could not be verified")
    return 1


def run_set(args):
    if args.current_id == args.new_id:
        print(f"No change: ID is already {args.current_id}")
        return 0
    buses = {}
    try:
        for channel in CHANNEL_MOTOR_IDS:
            buses[channel] = open_bus(channel)
        matches = []
        for channel, candidate_bus in buses.items():
            result = probe_motor(candidate_bus, args.current_id, QUERY_TIMEOUT)
            if result is not None:
                matches.append((channel, candidate_bus, result))
        if not matches:
            print(f"ERROR: ID {args.current_id} not found")
            return 1
        if len(matches) > 1:
            channels = ", ".join(channel for channel, _, _ in matches)
            print(f"ERROR: ID {args.current_id} found on multiple channels: {channels}")
            return 1
        channel, bus, current = matches[0]
        if args.new_id not in CHANNEL_MOTOR_IDS[channel]:
            valid_ids = ", ".join(str(motor_id) for motor_id in CHANNEL_MOTOR_IDS[channel])
            print(f"ERROR: ID {args.new_id} is not assigned to {channel}; use one of: {valid_ids}")
            return 1
        occupied = probe_motor(bus, args.new_id, QUERY_TIMEOUT)
        if occupied is not None:
            print(f"ERROR: ID {args.new_id} is already in use on {channel}")
            return 1
        print(f"WARNING: connect only the target motor ({channel}, ID {args.current_id})")
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
        return verify_change(bus, args.current_id, args.new_id, current["uid"])
    finally:
        for bus in buses.values():
            bus.shutdown()


def build_parser():
    parser = argparse.ArgumentParser(description="Check, find, or change a Robstride motor CAN ID.")
    commands = parser.add_subparsers(dest="command", required=True)

    check = commands.add_parser("check", help="Check the standard can0/can1 ID layout")
    check.set_defaults(handler=run_check)

    find = commands.add_parser("find", help="Search for motor IDs on can0 and can1")
    find.add_argument("--motor-id", type=parse_id)
    find.set_defaults(handler=run_find)

    set_id = commands.add_parser("set", help="Permanently change a motor CAN ID")
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
