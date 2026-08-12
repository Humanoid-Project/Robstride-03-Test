#!/usr/bin/env python3
import argparse
import time

import can

HOST_ID = 0xFD
DEFAULT_INTERFACE = "socketcan"

CHANNEL_ID_RANGES = {
    "can0": range(1, 7),
    "can1": range(7, 13),
}


def build_arb(comm_type, data16, target_id):
    return ((comm_type & 0x1F) << 24) | ((data16 & 0xFFFF) << 8) | (target_id & 0xFF)


def parse_arb(arbitration_id):
    comm_type = (arbitration_id >> 24) & 0x1F
    data16 = (arbitration_id >> 8) & 0xFFFF
    destination = arbitration_id & 0xFF
    return comm_type, data16, destination


def get_device_id(bus, host_id, target_id, timeout=0.2):
    arb = build_arb(0x00, host_id, target_id)
    try:
        bus.send(can.Message(arbitration_id=arb, data=bytes(8), is_extended_id=True))
    except can.CanError:
        time.sleep(0.05)
        return None
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=max(0.0, deadline - time.monotonic()))
        if msg is None or not msg.is_extended_id:
            continue
        comm_type, data16, destination = parse_arb(msg.arbitration_id)
        if (comm_type == 0x00 and destination == host_id
                and (data16 & 0xFF) == target_id):
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
        description="데이지 체인에 연결된 모터의 ID를 채널별로 구동 없이 탐색해 출력한다."
    )
    parser.add_argument("--channels", nargs="+", default=list(CHANNEL_ID_RANGES.keys()),
                        choices=list(CHANNEL_ID_RANGES.keys()),
                        help=f"확인할 CAN 채널들, 기본값: {' '.join(CHANNEL_ID_RANGES.keys())}")
    parser.add_argument("--interface", default=DEFAULT_INTERFACE, help="python-can 인터페이스, 기본값: socketcan")
    parser.add_argument("--host-id", type=lambda v: int(v, 0), default=HOST_ID, help="호스트 CAN ID, 기본값: 0xFD")
    args = parser.parse_args()

    found_by_channel = {}
    for channel in args.channels:
        id_range = CHANNEL_ID_RANGES[channel]
        found_by_channel[channel] = check_channel(channel, args.interface, args.host_id, id_range)

    print("=" * 60)
    print("Summary")
    print("=" * 60)
    for channel in args.channels:
        id_range = CHANNEL_ID_RANGES[channel]
        found_ids = sorted(found_by_channel[channel])
        missing = [i for i in id_range if i not in found_ids]
        print(f"{channel} (ID {id_range.start}..{id_range.stop - 1}): "
              f"{len(found_ids)}/{len(list(id_range))} found "
              f"{', '.join(str(i) for i in found_ids) if found_ids else '(none)'}")
        if missing:
            print(f"  missing: {', '.join(str(i) for i in missing)}")

    print()
    all_missing = [
        (channel, i)
        for channel in args.channels
        for i in CHANNEL_ID_RANGES[channel]
        if i not in found_by_channel[channel]
    ]
    if all_missing:
        print("일부 ID가 응답하지 않았습니다. 위 missing 목록을 확인하세요.")
    else:
        print("설정된 모든 ID가 각 채널에서 응답했습니다.")


if __name__ == "__main__":
    main()
