#!/usr/bin/env python3
"""리스트에 지정한 모터 ID들의 현재 관절값(기계각)을 출력한다.

아래 MOTOR_IDS 리스트를 편집해서 읽고 싶은 모터 ID만 넣으면 된다.
(명령줄에서 --ids 로 덮어쓸 수도 있다.)

사용 예:
    python3 read_joint_values.py                 # MOTOR_IDS 리스트 그대로 1회 출력
    python3 read_joint_values.py --ids 6 7 8      # 리스트 대신 6,7,8만
    python3 read_joint_values.py --watch          # 0.5초마다 반복 출력 (Ctrl-C 종료)
"""
import argparse
import math
import struct
import time

import can

HOST_ID = 0xFD
DEFAULT_CHANNEL = "can0"
DEFAULT_INTERFACE = "socketcan"
MECH_POS_INDEX = 0x7019

# ── 여기를 편집하세요: 관절값을 읽을 모터 ID 리스트 ───────────────
MOTOR_IDS = [7, 8, 9, 10, 11]
# ──────────────────────────────────────────────────────────────

JOINT_MAP = {
    1:  "left_hip_yaw",
    2:  "left_hip_pitch",
    3:  "left_hip_roll",
    4:  "left_knee_pitch",
    5:  "left_ankle_pitch",
    6:  "left_ankle_roll",
    7:  "right_hip_yaw",
    8:  "right_hip_pitch",
    9:  "right_hip_roll",
    10: "right_knee_pitch",
    11: "right_ankle_pitch",
    12: "right_ankle_roll",
}


def build_arb(comm_type, data16, target_id):
    return ((comm_type & 0x1F) << 24) | ((data16 & 0xFFFF) << 8) | (target_id & 0xFF)


def parse_arb(arbitration_id):
    comm_type = (arbitration_id >> 24) & 0x1F
    data16 = (arbitration_id >> 8) & 0xFFFF
    destination = arbitration_id & 0xFF
    return comm_type, data16, destination


def read_mech_position(bus, host_id, motor_id, timeout=0.1):
    """모터의 현재 기계각(rad)을 읽는다. 응답 없으면 None."""
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
        if len(payload) < 8:
            continue
        if int.from_bytes(payload[0:2], "little") != MECH_POS_INDEX:
            continue
        return struct.unpack_from("<f", payload, 4)[0]
    return None


def joint_name(motor_id):
    return JOINT_MAP.get(motor_id, f"ID{motor_id}")


def read_all(bus, host_id, motor_ids, timeout):
    print(f"{'ID':>3}  {'joint':<18}  {'position':>26}")
    print("-" * 52)
    for motor_id in motor_ids:
        position = read_mech_position(bus, host_id, motor_id, timeout=timeout)
        if position is None:
            value = "no response"
        else:
            value = f"{position:+8.4f} rad ({math.degrees(position):+8.2f} deg)"
        print(f"{motor_id:>3}  {joint_name(motor_id):<18}  {value:>26}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="리스트에 지정한 모터 ID들의 현재 관절값(기계각)을 출력한다.")
    parser.add_argument("--ids", type=lambda v: int(v, 0), nargs="+", default=None,
                        help="MOTOR_IDS 리스트 대신 사용할 모터 ID들 (예: --ids 6 7 8)")
    parser.add_argument("--channel", default=DEFAULT_CHANNEL, help="CAN 채널, 기본값: can0")
    parser.add_argument("--interface", default=DEFAULT_INTERFACE,
                        help="python-can 인터페이스, 기본값: socketcan")
    parser.add_argument("--host-id", type=lambda v: int(v, 0), default=HOST_ID,
                        help="호스트 CAN ID, 기본값: 0xFD")
    parser.add_argument("--timeout", type=float, default=0.1,
                        help="모터당 응답 대기 시간(초), 기본값: 0.1")
    parser.add_argument("--watch", action="store_true",
                        help="한 번만 출력하지 않고 --interval 마다 반복")
    parser.add_argument("--interval", type=float, default=0.5,
                        help="--watch 시 갱신 주기(초), 기본값: 0.5")
    return parser.parse_args()


def main():
    args = parse_args()
    motor_ids = args.ids if args.ids is not None else MOTOR_IDS

    bus = can.Bus(channel=args.channel, interface=args.interface)
    try:
        if not args.watch:
            read_all(bus, args.host_id, motor_ids, args.timeout)
            return

        print("Ctrl-C 로 종료.\n")
        while True:
            print(f"[{time.strftime('%H:%M:%S')}]  channel={args.channel}")
            read_all(bus, args.host_id, motor_ids, args.timeout)
            print()
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
