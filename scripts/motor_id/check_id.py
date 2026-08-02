#!/usr/bin/env python3
"""데이지 체인으로 연결된 Robstride 모터의 ID를 탐색해 출력한다.

모터를 구동하지 않고, 각 ID로 장치 정보(get device ID, comm type 0x00)를
요청해 응답하는 모터만 목록으로 출력한다.

사용 예:
    python3 check_id.py                       # can0, can1 에서 ID 1..12 확인
    python3 check_id.py --channels can0       # can0 만 확인
    python3 check_id.py --id-min 1 --id-max 32
"""
import argparse
import time

import can

HOST_ID = 0xFD
DEFAULT_CHANNELS = ["can0", "can1"]
DEFAULT_INTERFACE = "socketcan"
DEFAULT_ID_MIN = 1
DEFAULT_ID_MAX = 12


def build_arb(comm_type, data16, target_id):
    return ((comm_type & 0x1F) << 24) | ((data16 & 0xFFFF) << 8) | (target_id & 0xFF)


def parse_arb(arbitration_id):
    comm_type = (arbitration_id >> 24) & 0x1F
    data16 = (arbitration_id >> 8) & 0xFFFF
    destination = arbitration_id & 0xFF
    return comm_type, data16, destination


def get_device_id(bus, host_id, target_id, timeout=0.2):
    """comm type 0x00 (get device ID) 요청. 응답 시 (motor_id, uid) 반환.

    Robstride 의 device-ID 응답 arbitration ID 는
    0x00 | (motor_id << 8) | 0xFE 형식이라, 끝 바이트가 호스트 ID 가 아니라
    0xFE 로 돌아온다. 모터 ID 는 data16 필드에 실려 온다.
    """
    arb = build_arb(0x00, host_id, target_id)
    try:
        bus.send(can.Message(arbitration_id=arb, data=bytes(8), is_extended_id=True))
    except can.CanError:
        # "Transmit buffer full" 등: 버스에서 아무도 ACK 하지 않으면 전송 프레임이
        # 계속 재전송되며 쌓인다. 버스에 통신 가능한 노드가 없다는 신호.
        time.sleep(0.05)
        return None
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=max(0.0, deadline - time.monotonic()))
        if msg is None or not msg.is_extended_id:
            continue
        comm_type, data16, destination = parse_arb(msg.arbitration_id)
        # 응답의 data16 하위 바이트가 우리가 물은 target_id 와 같으면 그 모터의 응답.
        # (우리가 보낸 프레임의 echo 는 data16 하위 바이트가 host_id 라 걸러진다.)
        if comm_type == 0x00 and (data16 & 0xFF) == target_id:
            motor_id = data16 & 0xFF
            uid = bytes(msg.data)
            return motor_id, uid
    return None


def check_ids(bus, host_id, id_range):
    found = []
    for target in id_range:
        result = get_device_id(bus, host_id, target)
        if result is not None:
            motor_id, uid = result
            found.append((motor_id, uid))
            print(f"  [OK]  ID {motor_id:2d} (0x{motor_id:02X})  UID={uid.hex()}")
        else:
            print(f"  [--]  ID {target:2d} (0x{target:02X})  no response")
    return found


def check_channel(channel, interface, host_id, id_range):
    print(f"Checking IDs {id_range.start}..{id_range.stop - 1} on {channel} ...")
    bus = can.Bus(channel=channel, interface=interface)
    try:
        found = check_ids(bus, host_id, id_range)
    finally:
        bus.shutdown()
    print()
    return {motor_id for motor_id, _ in found}


def main():
    parser = argparse.ArgumentParser(
        description="데이지 체인에 연결된 모터의 ID를 구동 없이 탐색해 출력한다."
    )
    parser.add_argument("--channels", nargs="+", default=DEFAULT_CHANNELS,
                        help=f"확인할 CAN 채널들, 기본값: {' '.join(DEFAULT_CHANNELS)}")
    parser.add_argument("--interface", default=DEFAULT_INTERFACE, help="python-can 인터페이스, 기본값: socketcan")
    parser.add_argument("--host-id", type=lambda v: int(v, 0), default=HOST_ID, help="호스트 CAN ID, 기본값: 0xFD")
    parser.add_argument("--id-min", type=int, default=DEFAULT_ID_MIN, help="탐색 시작 ID, 기본값: 1")
    parser.add_argument("--id-max", type=int, default=DEFAULT_ID_MAX, help="탐색 종료 ID, 기본값: 12")
    args = parser.parse_args()

    id_range = range(args.id_min, args.id_max + 1)
    expected = list(id_range)

    found_by_channel = {}
    for channel in args.channels:
        found_by_channel[channel] = check_channel(channel, args.interface, args.host_id, id_range)

    all_found = set().union(*found_by_channel.values()) if found_by_channel else set()
    missing_everywhere = [i for i in expected if i not in all_found]

    print("=" * 60)
    print("Summary")
    print("=" * 60)
    print(f"{'ID':>3}  " + "  ".join(f"{ch:<8}" for ch in args.channels))
    for motor_id in expected:
        row = "  ".join(
            f"{'OK':<8}" if motor_id in found_by_channel[ch] else f"{'--':<8}"
            for ch in args.channels
        )
        print(f"{motor_id:>3}  {row}")

    print()
    for channel in args.channels:
        found_ids = sorted(found_by_channel[channel])
        print(f"{channel}: {len(found_ids)}/{len(expected)} "
              f"{', '.join(str(i) for i in found_ids) if found_ids else '(none)'}")

    print()
    if missing_everywhere:
        print(f"어느 채널에서도 응답하지 않은 ID: {', '.join(str(i) for i in missing_everywhere)}")
    else:
        print(f"ID {args.id_min}..{args.id_max} 전부 (can0 또는 can1 중) 하나 이상에서 응답함.")


if __name__ == "__main__":
    main()
