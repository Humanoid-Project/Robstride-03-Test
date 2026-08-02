#!/usr/bin/env python3
import argparse
import math
import struct
import sys
import time

import can

HOST_ID = 0xFD
DEFAULT_INTERFACE = "socketcan"

MECH_POS_INDEX = 0x7019
ZERO_STA_INDEX = 0x7029

TYPE_SET_ZERO = 0x06
TYPE_WRITE_PARAM = 0x12
TYPE_SAVE = 0x16
SAVE_PAYLOAD = bytes([0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08])

TWO_PI = 2.0 * math.pi

CHANNEL_ID_RANGES = {
    "can0": range(1, 7),
    "can1": range(7, 13),
}

JOINT_MAP = {
    1: "left_hip_yaw", 2: "left_hip_pitch", 3: "left_hip_roll",
    4: "left_knee_pitch", 5: "left_ankle_pitch", 6: "left_ankle_roll",
    7: "right_hip_yaw", 8: "right_hip_pitch", 9: "right_hip_roll",
    10: "right_knee_pitch", 11: "right_ankle_pitch", 12: "right_ankle_roll",
}


def channel_for_id(motor_id):
    for channel, id_range in CHANNEL_ID_RANGES.items():
        if motor_id in id_range:
            return channel
    raise ValueError(f"모터 ID {motor_id} 에 대한 채널을 찾을 수 없습니다 (CHANNEL_ID_RANGES 확인).")


def joint_name(motor_id):
    return JOINT_MAP.get(motor_id, f"ID{motor_id}")


def build_arb(comm_type, data16, target_id):
    return ((comm_type & 0x1F) << 24) | ((data16 & 0xFFFF) << 8) | (target_id & 0xFF)


def parse_arb(arbitration_id):
    comm_type = (arbitration_id >> 24) & 0x1F
    data16 = (arbitration_id >> 8) & 0xFFFF
    destination = arbitration_id & 0xFF
    return comm_type, data16, destination


def read_mech_position(bus, host_id, motor_id, timeout=0.2):
    data = bytearray(8)
    struct.pack_into("<H", data, 0, MECH_POS_INDEX)
    bus.send(can.Message(arbitration_id=build_arb(0x11, host_id, motor_id),
                         data=bytes(data), is_extended_id=True))

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=max(0.0, deadline - time.monotonic()))
        if msg is None or not msg.is_extended_id:
            continue
        comm_type, data16, destination = parse_arb(msg.arbitration_id)
        if comm_type != 0x11 or destination != host_id or (data16 & 0xFF) != motor_id:
            continue
        payload = bytes(msg.data)
        if len(payload) < 8 or int.from_bytes(payload[0:2], "little") != MECH_POS_INDEX:
            continue
        return struct.unpack_from("<f", payload, 4)[0]
    return None


def read_mech_position_retry(bus, host_id, motor_id, attempts=5, timeout=0.2):
    for _ in range(attempts):
        position = read_mech_position(bus, host_id, motor_id, timeout=timeout)
        if position is not None:
            return position
    return None


def set_mechanical_zero(bus, host_id, motor_id):
    data = bytearray(8)
    data[0] = 1
    bus.send(can.Message(arbitration_id=build_arb(TYPE_SET_ZERO, host_id, motor_id),
                         data=bytes(data), is_extended_id=True))
    deadline = time.monotonic() + 0.2
    while time.monotonic() < deadline:
        if bus.recv(timeout=max(0.0, deadline - time.monotonic())) is None:
            break


def write_uint8_parameter(bus, host_id, motor_id, index, value):
    data = bytearray(8)
    struct.pack_into("<H", data, 0, index)
    data[4] = value & 0xFF
    bus.send(can.Message(arbitration_id=build_arb(TYPE_WRITE_PARAM, host_id, motor_id),
                         data=bytes(data), is_extended_id=True))
    time.sleep(0.05)


def save_parameters(bus, host_id, motor_id):
    bus.send(can.Message(arbitration_id=build_arb(TYPE_SAVE, host_id, motor_id),
                         data=SAVE_PAYLOAD, is_extended_id=True))
    time.sleep(0.3)


def fmt(rad):
    return f"{rad:+8.4f} rad ({math.degrees(rad):+8.2f} deg)"


def angular_diff(a, b):
    return (a - b + math.pi) % TWO_PI - math.pi


def parse_args():
    parser = argparse.ArgumentParser(
        description="can0/can1 에 연결된 모터 ID 1~12 모두의 현재 위치를 기계적 영점(zero)으로 지정한다.")
    parser.add_argument("--channels", nargs="+", default=list(CHANNEL_ID_RANGES.keys()),
                        choices=list(CHANNEL_ID_RANGES.keys()),
                        help=f"대상 CAN 채널들, 기본값: {' '.join(CHANNEL_ID_RANGES.keys())}")
    parser.add_argument("--interface", default=DEFAULT_INTERFACE, help="python-can 인터페이스, 기본값: socketcan")
    parser.add_argument("--host-id", type=lambda v: int(v, 0), default=HOST_ID, help="호스트 CAN ID, 기본값: 0xFD")
    parser.add_argument("--save", action="store_true", help="영점 설정 후 type 22 저장 프레임 전송")
    parser.add_argument("--zero-sta", type=int, choices=[0, 1],
                        help="전원인가 시 위치 범위 설정: 0=0..2pi, 1=-pi..pi (지정 시 --save 자동 적용)")
    parser.add_argument("--tolerance", type=float, default=0.05,
                        help="영점 성공 판정 허용 오차(rad), 기본값: 0.05")
    parser.add_argument("--yes", action="store_true", help="확인 프롬프트 없이 바로 진행")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.zero_sta is not None:
        args.save = True

    if not sys.stdin.isatty() and not args.yes:
        print("확인 입력이 필요합니다. --yes 로 실행하거나 대화형 터미널에서 실행하세요.")
        return 1

    active_ids = sorted(
        motor_id
        for channel in args.channels
        for motor_id in CHANNEL_ID_RANGES[channel]
    )

    buses = {}
    try:
        for channel in args.channels:
            buses[channel] = can.Bus(channel=channel, interface=args.interface)

        print("현재 위치 확인 중...\n")
        print(f"{'ID':>3}  {'ch':<5}  {'joint':<18}  {'현재각':>26}")
        print("-" * 62)
        before = {}
        missing = []
        for motor_id in active_ids:
            channel = channel_for_id(motor_id)
            bus = buses[channel]
            position = read_mech_position_retry(bus, args.host_id, motor_id)
            if position is None:
                missing.append(motor_id)
                print(f"{motor_id:>3}  {channel:<5}  {joint_name(motor_id):<18}  {'무응답':>26}")
                continue
            before[motor_id] = position
            print(f"{motor_id:>3}  {channel:<5}  {joint_name(motor_id):<18}  {fmt(position):>26}")

        if missing:
            print(f"\n무응답 ID (건너뜀): {', '.join(str(i) for i in missing)}")

        if not before:
            print("\n영점을 설정할 수 있는 모터가 없습니다. 중단합니다.")
            return 1

        print(f"\n대상 {len(before)}개 모터의 현재 위치를 기계적 영점(0)으로 지정합니다.")
        print("주의: 모터가 enable 상태가 아니어야 합니다. 구버전 펌웨어는 enable 상태에서")
        print("영점을 설정하면 모터가 목표 위치로 급하게 움직일 수 있습니다.\n")

        if not args.yes:
            answer = input("계속하려면 Enter, 취소하려면 Ctrl-C: ")
            del answer

        print("\n영점 설정 중...")
        print(f"{'ID':>3}  {'ch':<5}  {'joint':<18}  {'설정 후':>26}  결과")
        print("-" * 80)
        ok_count = 0
        for motor_id in sorted(before):
            channel = channel_for_id(motor_id)
            bus = buses[channel]

            if args.zero_sta is not None:
                write_uint8_parameter(bus, args.host_id, motor_id, ZERO_STA_INDEX, args.zero_sta)

            set_mechanical_zero(bus, args.host_id, motor_id)

            if args.save:
                save_parameters(bus, args.host_id, motor_id)

            after = read_mech_position_retry(bus, args.host_id, motor_id)
            if after is None:
                print(f"{motor_id:>3}  {channel:<5}  {joint_name(motor_id):<18}  {'무응답':>26}  실패")
                continue

            success = abs(angular_diff(after, 0.0)) <= args.tolerance
            if success:
                ok_count += 1
            result = "OK" if success else "WARNING: 0으로 가지 않음"
            print(f"{motor_id:>3}  {channel:<5}  {joint_name(motor_id):<18}  {fmt(after):>26}  {result}")

        print(f"\n{ok_count}/{len(before)} 모터의 영점 설정 완료.")
        return 0
    except KeyboardInterrupt:
        print("\n취소됨.")
        return 0
    finally:
        for bus in buses.values():
            bus.shutdown()


if __name__ == "__main__":
    sys.exit(main())
