#!/usr/bin/env python3
import argparse
import struct
import sys
import time
from pathlib import Path

import can

HOST_ID = 0xFD
DEVICE_ID_DESTINATION = 0xFE
INTERFACE = "socketcan"
MECH_POS_INDEX = 0x7019
QUERY_TIMEOUT = 0.25
SCAN_TIMEOUT = 0.08
STANDARD_ID_RANGES = {
    "can0": range(1, 7),
    "can1": range(7, 13),
}


class MotorIdError(RuntimeError):
    pass


def parse_id(value):
    motor_id = int(value, 0)
    if not 0 <= motor_id <= 127:
        raise argparse.ArgumentTypeError("ID는 0~127 범위여야 합니다.")
    return motor_id


def parse_new_id(value):
    motor_id = int(value, 0)
    if not 1 <= motor_id <= 127:
        raise argparse.ArgumentTypeError("새 ID는 1~127 범위여야 합니다.")
    return motor_id


def build_arb(comm_type, data16, target_id):
    return ((comm_type & 0x1F) << 24) | ((data16 & 0xFFFF) << 8) | (target_id & 0xFF)


def parse_arb(arbitration_id):
    return (
        (arbitration_id >> 24) & 0x1F,
        (arbitration_id >> 8) & 0xFFFF,
        arbitration_id & 0xFF,
    )


def send(bus, comm_type, data16, target_id, data=None):
    message = can.Message(
        arbitration_id=build_arb(comm_type, data16, target_id),
        data=bytes(8) if data is None else data,
        is_extended_id=True,
    )
    try:
        bus.send(message)
    except can.CanError as exc:
        raise MotorIdError(f"CAN 전송 실패: {exc}") from exc


def receive_matching(bus, comm_type, destination, target_id, timeout, index=None):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            message = bus.recv(timeout=max(0.0, deadline - time.monotonic()))
        except can.CanError as exc:
            raise MotorIdError(f"CAN 수신 실패: {exc}") from exc
        if message is None or not message.is_extended_id:
            continue
        reply_type, data16, reply_destination = parse_arb(message.arbitration_id)
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
    send(bus, 0x00, HOST_ID, target_id)
    payload = receive_matching(
        bus,
        0x00,
        DEVICE_ID_DESTINATION,
        target_id,
        timeout,
    )
    if payload is None:
        return None
    return {"motor_id": target_id, "uid": payload.hex(), "source": "device-id"}


def query_mech_position(bus, target_id, timeout=QUERY_TIMEOUT):
    payload = bytearray(8)
    struct.pack_into("<H", payload, 0, MECH_POS_INDEX)
    send(bus, 0x11, HOST_ID, target_id, bytes(payload))
    reply = receive_matching(
        bus,
        0x11,
        HOST_ID,
        target_id,
        timeout,
        index=MECH_POS_INDEX,
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
            f"{channel} 인터페이스가 DOWN입니다. CAN Interface 설정을 먼저 확인하세요."
        )
    try:
        return can.Bus(channel=channel, interface=INTERFACE)
    except (can.CanError, OSError) as exc:
        raise MotorIdError(f"{channel} 열기 실패: {exc}") from exc


def print_found(result):
    suffix = f"  UID={result['uid']}" if result["uid"] else "  mechPos 응답"
    print(f"  [OK] ID {result['motor_id']:3d} (0x{result['motor_id']:02X}){suffix}")


def scan_ids(bus, targets, timeout):
    found = {}
    for target_id in targets:
        result = probe_motor(bus, target_id, timeout)
        if result is None:
            print(f"  [--] ID {target_id:3d} (0x{target_id:02X})  무응답")
        else:
            found[target_id] = result
            print_found(result)
        time.sleep(0.005)
    return found


def run_check(args):
    failed = False
    for channel in args.channels:
        targets = STANDARD_ID_RANGES[channel]
        print(f"{channel}: 표준 ID {targets.start}~{targets.stop - 1} 확인")
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
            print(f"  누락: {', '.join(map(str, missing))}")
        else:
            print("  모든 표준 ID가 응답했습니다.")
    return 1 if failed else 0


def run_find(args):
    bus = open_bus(args.channel)
    try:
        if args.motor_id is not None:
            result = probe_motor(bus, args.motor_id, QUERY_TIMEOUT)
            if result is None:
                print(f"ID {args.motor_id} (0x{args.motor_id:02X}): 무응답")
                return 1
            print_found(result)
            return 0
        print(f"{args.channel}: ID 0~{args.scan_max} 검색")
        found = scan_ids(bus, range(args.scan_max + 1), SCAN_TIMEOUT)
        if not found:
            print("응답한 모터가 없습니다.")
            return 1
        print(f"발견된 ID: {', '.join(map(str, sorted(found)))}")
        return 0
    finally:
        bus.shutdown()


def send_stop(bus, motor_id):
    send(bus, 0x04, HOST_ID, motor_id)


def send_set_id(bus, current_id, new_id):
    data16 = ((new_id & 0xFF) << 8) | HOST_ID
    send(bus, 0x07, data16, current_id)


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
            print("검증 실패: 새 ID에서 다른 UID가 응답했습니다. ID 충돌 가능성이 있습니다.")
            return 1
        print(f"성공: 모터가 ID {new_id} (0x{new_id:02X})로만 응답합니다.")
        return 0
    if new_result is None and old_result is not None:
        print(f"변경 실패: 모터가 이전 ID {current_id}로 계속 응답합니다.")
        return 1
    if new_result is not None and old_result is not None:
        print("검증 실패: 이전 ID와 새 ID가 모두 응답합니다. 중복 모터 또는 ID 충돌 상태입니다.")
        return 1
    print("검증 불가: 이전 ID와 새 ID가 모두 무응답입니다. 전원 재인가 후 find로 확인하세요.")
    return 1


def run_set(args):
    if args.current_id == args.new_id:
        print("현재 ID와 새 ID가 같아 변경하지 않았습니다.")
        return 0
    bus = open_bus(args.channel)
    try:
        current = probe_motor(bus, args.current_id, QUERY_TIMEOUT)
        if current is None:
            print(f"중단: 현재 ID {args.current_id}가 응답하지 않습니다.")
            return 1
        occupied = probe_motor(bus, args.new_id, QUERY_TIMEOUT)
        if occupied is not None:
            print(f"중단: 새 ID {args.new_id}가 이미 {args.channel}에서 응답합니다.")
            return 1
        print("경고: 현재 ID에 여러 모터가 연결되어 있으면 모두 같은 새 ID로 변경됩니다.")
        print("ID 변경은 대상 모터 한 대만 연결한 상태에서 수행하세요.")
        expected = f"CHANGE {args.current_id} {args.new_id}"
        try:
            reply = input(f"영구 변경하려면 '{expected}'를 정확히 입력하세요: ")
        except EOFError:
            reply = ""
        if reply.strip() != expected:
            print("취소했습니다.")
            return 1
        send_stop(bus, args.current_id)
        time.sleep(0.05)
        send_set_id(bus, args.current_id, args.new_id)
        time.sleep(0.2)
        print(f"ID {args.current_id} -> {args.new_id} 변경 명령을 전송했습니다. 응답을 검증합니다.")
        return verify_change(bus, args.current_id, args.new_id, current["uid"])
    finally:
        bus.shutdown()


def build_parser():
    parser = argparse.ArgumentParser(description="Robstride 모터 CAN ID 확인, 검색, 변경")
    commands = parser.add_subparsers(dest="command", required=True)

    check = commands.add_parser("check", help="표준 can0/can1 ID 배치를 확인")
    check.add_argument(
        "--channels",
        nargs="+",
        choices=tuple(STANDARD_ID_RANGES),
        default=list(STANDARD_ID_RANGES),
    )
    check.set_defaults(handler=run_check)

    find = commands.add_parser("find", help="한 채널에서 모터 ID를 검색")
    find.add_argument("--channel", choices=tuple(STANDARD_ID_RANGES), required=True)
    find.add_argument("--motor-id", type=parse_id)
    find.add_argument("--scan-max", type=parse_id, default=127)
    find.set_defaults(handler=run_find)

    set_id = commands.add_parser("set", help="모터 CAN ID를 영구 변경")
    set_id.add_argument("--channel", choices=tuple(STANDARD_ID_RANGES), required=True)
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
        print(f"오류: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\n중단했습니다.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
