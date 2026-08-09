#!/usr/bin/env python3
import argparse
import math
import struct
import sys
import threading
import time

import can

HOST_ID = 0xFD
DEFAULT_INTERFACE = "socketcan"
MECH_POS_INDEX = 0x7019
# 1회 출력은 넉넉하게 기다리고, --watch 는 갱신 속도를 위해 짧게 끊는다.
ONESHOT_TIMEOUT = 0.1
WATCH_TIMEOUT = 0.02

CHANNEL_MOTOR_IDS = {
    "can0": [1, 2, 3, 4, 5, 6],
    "can1": [7, 8, 9, 10, 11, 12],
}

JOINT_MAP = {
    1:  "left_hip_yaw",
    2:  "left_hip_pitch",
    3:  "left_hip_roll",
    4:  "left_knee_pitch",
    5:  "left_ankle_upper",
    6:  "left_ankle_lower",
    7:  "right_hip_yaw",
    8:  "right_hip_pitch",
    9:  "right_hip_roll",
    10: "right_knee_pitch",
    11: "right_ankle_upper",
    12: "right_ankle_lower",
}


def build_arb(comm_type, data16, target_id):
    return ((comm_type & 0x1F) << 24) | ((data16 & 0xFFFF) << 8) | (target_id & 0xFF)


def parse_arb(arbitration_id):
    comm_type = (arbitration_id >> 24) & 0x1F
    data16 = (arbitration_id >> 8) & 0xFFFF
    destination = arbitration_id & 0xFF
    return comm_type, data16, destination


def read_mech_position(bus, host_id, motor_id, timeout=0.1):
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


def format_position(position):
    if position is None:
        return "no response"
    return f"{position:+8.4f} rad ({math.degrees(position):+8.2f} deg)"


def read_channel(channel, interface, host_id, motor_ids, timeout):
    print(f"[{channel}]")
    print(f"  {'ID':>3}  {'joint':<18}  {'position':>26}")
    print("  " + "-" * 50)
    bus = can.Bus(channel=channel, interface=interface)
    try:
        for motor_id in motor_ids:
            position = read_mech_position(bus, host_id, motor_id, timeout=timeout)
            print(f"  {motor_id:>3}  {joint_name(motor_id):<18}  "
                  f"{format_position(position):>26}")
    finally:
        bus.shutdown()


def read_all_channels(channels, interface, host_id, timeout):
    for channel in channels:
        read_channel(channel, interface, host_id, CHANNEL_MOTOR_IDS[channel], timeout)


def poll_worker(channel, interface, host_id, timeout, state, rate, notes, lock, stop):
    """버스를 한 번만 열고 종료 신호가 올 때까지 해당 채널을 계속 폴링한다."""
    motor_ids = CHANNEL_MOTOR_IDS[channel]
    try:
        bus = can.Bus(channel=channel, interface=interface)
    except OSError as e:
        with lock:
            notes.append(f"[{channel}] 열기 실패: {e}  "
                         f"(sudo ip link set {channel} up type can bitrate 1000000)")
        return

    t0, cnt = time.monotonic(), 0
    try:
        while not stop.is_set():
            for motor_id in motor_ids:
                position = read_mech_position(bus, host_id, motor_id, timeout=timeout)
                with lock:
                    state[motor_id] = position
            cnt += 1
            now = time.monotonic()
            if now - t0 >= 0.5:
                with lock:
                    rate[channel] = cnt / (now - t0)
                t0, cnt = now, 0
    except can.CanError as e:
        with lock:
            notes.append(f"[{channel}] CAN 오류: {e}")
    finally:
        bus.shutdown()


def watch_channels(channels, interface, host_id, timeout, interval):
    """채널마다 폴링 스레드를 띄우고, 최신값을 interval 마다 제자리에 다시 그린다."""
    state = {motor_id: None for ch in channels for motor_id in CHANNEL_MOTOR_IDS[ch]}
    rate = {ch: 0.0 for ch in channels}
    notes = []
    lock = threading.Lock()
    stop = threading.Event()

    threads = [threading.Thread(
        target=poll_worker,
        args=(ch, interface, host_id, timeout, state, rate, notes, lock, stop),
        daemon=True) for ch in channels]
    for t in threads:
        t.start()

    try:
        while True:
            with lock:
                snapshot = dict(state)
                rt = dict(rate)
                current_notes = list(notes)

            hz = "   ".join(f"{ch} {rt[ch]:6.1f} Hz" for ch in channels)
            out = ["\033[2J\033[3J\033[H"]
            out.append(f"joint monitor   {hz}   {time.strftime('%H:%M:%S')}"
                       f"    (Ctrl-C 종료)\n")
            for channel in channels:
                out.append(f"[{channel}]")
                out.append(f"  {'ID':>3}  {'joint':<18}  {'position':>26}")
                out.append("  " + "-" * 50)
                for motor_id in CHANNEL_MOTOR_IDS[channel]:
                    out.append(f"  {motor_id:>3}  {joint_name(motor_id):<18}  "
                               f"{format_position(snapshot[motor_id]):>26}")
                out.append("")
            out.extend(current_notes)

            sys.stdout.write("\n".join(out) + "\n")
            sys.stdout.flush()
            time.sleep(interval)
    finally:
        stop.set()
        for t in threads:
            t.join(timeout=2.0)


def parse_args():
    parser = argparse.ArgumentParser(
        description="채널별로 지정한 모터 ID들의 현재 관절값(기계각)을 출력한다.")
    parser.add_argument("--channels", nargs="+", default=list(CHANNEL_MOTOR_IDS.keys()),
                        choices=list(CHANNEL_MOTOR_IDS.keys()),
                        help=f"확인할 CAN 채널들, 기본값: {' '.join(CHANNEL_MOTOR_IDS.keys())}")
    parser.add_argument("--interface", default=DEFAULT_INTERFACE,
                        help="python-can 인터페이스, 기본값: socketcan")
    parser.add_argument("--host-id", type=lambda v: int(v, 0), default=HOST_ID,
                        help="호스트 CAN ID, 기본값: 0xFD")
    parser.add_argument("--timeout", type=float, default=None,
                        help=f"모터당 응답 대기 시간(초), 기본값: {ONESHOT_TIMEOUT} "
                             f"(--watch 시 {WATCH_TIMEOUT})")
    parser.add_argument("--watch", action="store_true",
                        help="한 번만 출력하지 않고 실시간으로 계속 갱신")
    parser.add_argument("--interval", type=float, default=0.1,
                        help="--watch 시 화면 갱신 주기(초), 기본값: 0.1")
    return parser.parse_args()


def main():
    args = parse_args()
    timeout = args.timeout
    if timeout is None:
        timeout = WATCH_TIMEOUT if args.watch else ONESHOT_TIMEOUT

    try:
        if args.watch:
            watch_channels(args.channels, args.interface, args.host_id,
                           timeout, args.interval)
        else:
            read_all_channels(args.channels, args.interface, args.host_id, timeout)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
