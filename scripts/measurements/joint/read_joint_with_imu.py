#!/usr/bin/env python3
import argparse
import math
import struct
import sys
import threading
import time
from pathlib import Path

import can

from robonex_common.joints import ACTUATED_JOINTS, CHANNEL_MOTOR_IDS
from robonex_common.protocol import (
    COMM_PARAMETER_READ,
    DEFAULT_INTERFACE,
    HOST_ID,
    MECHANICAL_POSITION_INDEX,
    MECHANICAL_VELOCITY_INDEX,
    build_arbitration_id,
    parse_arbitration_id,
)

DEG = math.pi / 180.0
MOUNT_ROLL_DEG = 180.0
PRINT_HZ = 10
JOINT_MAP = {joint.motor_id: joint.hardware_name for joint in ACTUATED_JOINTS}

parser = argparse.ArgumentParser(description="Print live motor pos/vel plus N100 IMU values.")
parser.add_argument("--imu-port", default="/dev/ttyUSB0")
parser.add_argument("--channels", nargs="+", default=list(CHANNEL_MOTOR_IDS),
                    choices=list(CHANNEL_MOTOR_IDS))
parser.add_argument("--n100-dir",
                    default=str(Path(__file__).resolve().parents[2]
                                / "motor_control" / "motor_with_imu_test"),
                    help="Folder that contains the built n100*.so")
parser.add_argument("--no-imu", action="store_true", help="Motors only")
parser.add_argument("--timeout", type=float, default=0.02, help="Motor reply wait, seconds")
args = parser.parse_args()

notes = []
imu_status = "off (--no-imu)"

sys.path.insert(0, args.n100_dir)
n100 = None
if not args.no_imu:
    try:
        import n100
    except ImportError as e:
        imu_status = "n100 module missing — build it first"
        notes.append(f"[IMU] import failed: {e}")
        notes.append(f"      cd {args.n100_dir} && cmake -S . -B build "
                     f"-DCMAKE_BUILD_TYPE=Release && cmake --build build -j")
        notes.append("      use --n100-dir if the .so lives elsewhere")


def read_param(bus, motor_id, index, timeout):
    data = bytearray(8)
    struct.pack_into("<H", data, 0, index)
    bus.send(can.Message(
        arbitration_id=build_arbitration_id(COMM_PARAMETER_READ, HOST_ID, motor_id),
        data=bytes(data), is_extended_id=True))
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=max(0.0, deadline - time.monotonic()))
        if msg is None or not msg.is_extended_id:
            continue
        comm_type, data16, dest = parse_arbitration_id(msg.arbitration_id)
        if comm_type != COMM_PARAMETER_READ or dest != HOST_ID or (data16 & 0xFF) != motor_id:
            continue
        payload = bytes(msg.data)
        if len(payload) >= 8 and int.from_bytes(payload[0:2], "little") == index:
            return struct.unpack_from("<f", payload, 4)[0]
    return None


state = {i: {"pos": None, "vel": None} for i in range(1, 13)}
rate = {ch: 0.0 for ch in CHANNEL_MOTOR_IDS}
lock = threading.Lock()
stop = threading.Event()


def can_worker(channel):
    ids = CHANNEL_MOTOR_IDS[channel]
    try:
        bus = can.Bus(channel=channel, interface=DEFAULT_INTERFACE)
    except OSError as e:
        notes.append(f"[{channel}] open failed: {e}  "
                     f"(sudo ip link set {channel} up type can bitrate 1000000)")
        return
    t0, cnt = time.time(), 0
    try:
        while not stop.is_set():
            for motor_id in ids:
                pos = read_param(bus, motor_id, MECHANICAL_POSITION_INDEX, args.timeout)
                vel = read_param(bus, motor_id, MECHANICAL_VELOCITY_INDEX, args.timeout)
                with lock:
                    state[motor_id]["pos"] = pos
                    state[motor_id]["vel"] = vel
            cnt += 1
            now = time.time()
            if now - t0 >= 0.5:
                with lock:
                    rate[channel] = cnt / (now - t0)
                t0, cnt = now, 0
    except can.CanError as e:
        notes.append(f"[{channel}] CAN error: {e}")
    finally:
        bus.shutdown()


driver = None
if n100 is not None:
    driver = n100.ImuDriver(n100.DriverConfig(
        port=args.imu_port,
        mount_rotation=n100.Quat.from_axis_angle_x(MOUNT_ROLL_DEG * DEG),
    ))
    try:
        driver.start()
        if driver.wait_for_sample(timeout=3.0) is None:
            imu_status = "port opened, no data"
            notes.append(f"[IMU] no sample in 3 s: {driver.last_error() or 'unknown'}")
            notes.append(f"      ./build/n100_cpp/read_imu {args.imu_port} 921600 180 0 0")
        else:
            imu_status = "ok"
    except RuntimeError as e:
        imu_status = "failed to open port"
        driver = None
        notes.append(f"[IMU] start failed: {e}")
        notes.append(f"      ls /dev/ttyUSB* /dev/ttyACM*   "
                     f"(permission: sudo chmod 666 {args.imu_port})")
        notes.append("      CANable uses gs_usb, not tty. Confirm ttyUSB0 is the IMU")

threads = [threading.Thread(target=can_worker, args=(ch,), daemon=True)
           for ch in args.channels]
for t in threads:
    t.start()
time.sleep(0.5)

imu_prev_seq, imu_t0, imu_hz = 0, time.time(), 0.0
try:
    while True:
        with lock:
            snap = {i: dict(v) for i, v in state.items()}
            rt = dict(rate)

        sample = driver.latest() if driver is not None else None
        now = time.time()
        if sample is not None and now - imu_t0 >= 0.5:
            imu_hz = (sample.seq - imu_prev_seq) / (now - imu_t0)
            imu_prev_seq, imu_t0 = sample.seq, now
        if driver is not None and not driver.is_running and imu_status == "ok":
            imu_status = f"reader stopped: {driver.last_error() or 'unknown'}"

        shown_ids = [i for ch in args.channels for i in CHANNEL_MOTOR_IDS[ch]]

        out = ["\033[2J\033[3J\033[H"]
        hz = "   ".join(f"{ch} {rt[ch]:6.1f} Hz" for ch in args.channels)
        out.append(f"EVE state monitor   {hz}   IMU {imu_hz:5.1f} Hz    (Ctrl-C to quit)\n")
        out.append(f"  {'ID':>3}  {'joint':<18}  {'pos[rad]':>10}  {'vel[rad/s]':>11}")
        out.append("  " + "-" * 48)
        for motor_id in sorted(shown_ids):
            s = snap[motor_id]
            p = f"{s['pos']:+10.4f}" if s["pos"] is not None else f"{'--':>10}"
            v = f"{s['vel']:+11.4f}" if s["vel"] is not None else f"{'--':>11}"
            out.append(f"  {motor_id:>3}  {JOINT_MAP[motor_id]:<18}  {p}  {v}")

        out.append("")
        if sample is None:
            out.append(f"IMU: {imu_status}")
        else:
            q, e = sample.orientation, sample.euler
            w, wr = sample.angular_velocity, sample.angular_velocity_raw
            a, m, g = (sample.linear_acceleration, sample.magnetic_field,
                       sample.projected_gravity)
            out.append(f"IMU  {args.imu_port}   seq {sample.seq}   [{imu_status}]")
            out.append(f"  quat    w {q.w:+8.4f}  x {q.x:+8.4f}  y {q.y:+8.4f}  z {q.z:+8.4f}")
            out.append(f"  rpy     r {e.roll/DEG:+8.2f}  p {e.pitch/DEG:+8.2f}  "
                       f"y {e.yaw/DEG:+8.2f}  [deg]")
            out.append(f"  gyro    x {w.x:+8.4f}  y {w.y:+8.4f}  z {w.z:+8.4f}  [rad/s]")
            out.append(f"  gyroR   x {wr.x:+8.4f}  y {wr.y:+8.4f}  z {wr.z:+8.4f}  [rad/s, raw]")
            out.append(f"  accel   x {a.x:+8.4f}  y {a.y:+8.4f}  z {a.z:+8.4f}  [m/s^2]")
            out.append(f"  mag     x {m.x:+8.2e}  y {m.y:+8.2e}  z {m.z:+8.2e}  [T]")
            out.append(f"  gproj   x {g.x:+8.4f}  y {g.y:+8.4f}  z {g.z:+8.4f}")
            out.append(f"  temp {sample.imu_temperature:5.1f} C   "
                       f"pressure {sample.pressure:9.1f} Pa")

        if notes:
            out.append("")
            out.extend(notes)

        sys.stdout.write("\n".join(out) + "\n")
        sys.stdout.flush()
        time.sleep(1.0 / PRINT_HZ)
except KeyboardInterrupt:
    pass
finally:
    stop.set()
    for t in threads:
        t.join(timeout=2.0)
    if driver is not None:
        driver.stop()
    print("stopped")
