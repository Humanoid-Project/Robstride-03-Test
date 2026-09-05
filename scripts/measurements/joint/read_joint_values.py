#!/usr/bin/env python3
import argparse
import math
import struct
import sys
import threading
import time

import can

from robonex_common.joints import ACTUATED_JOINTS, CHANNEL_MOTOR_IDS
from robonex_common.protocol import (
    COMM_PARAMETER_READ,
    DEFAULT_INTERFACE,
    HOST_ID,
    MECHANICAL_POSITION_INDEX,
    build_arbitration_id,
    parse_arbitration_id,
)

ONESHOT_TIMEOUT = 0.1
WATCH_TIMEOUT = 0.02
JOINT_MAP = {joint.motor_id: joint.hardware_name for joint in ACTUATED_JOINTS}


def read_mech_position(bus, host_id, motor_id, timeout=0.1):
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
        if len(payload) < 8:
            continue
        if int.from_bytes(payload[0:2], "little") != MECHANICAL_POSITION_INDEX:
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
    motor_ids = CHANNEL_MOTOR_IDS[channel]
    try:
        bus = can.Bus(channel=channel, interface=interface)
    except OSError as e:
        with lock:
            notes.append(f"[{channel}] open failed: {e}  "
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
            notes.append(f"[{channel}] CAN error: {e}")
    finally:
        bus.shutdown()


def watch_channels(channels, interface, host_id, timeout, interval):
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
                       f"    (Ctrl-C to quit)\n")
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
        description="Print mechanical joint angles for the selected motor IDs.")
    parser.add_argument("--watch", action="store_true",
                        help="Keep refreshing instead of printing once")
    parser.set_defaults(
        channels=list(CHANNEL_MOTOR_IDS), interface=DEFAULT_INTERFACE,
        host_id=HOST_ID, timeout=None, interval=0.1,
    )
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
