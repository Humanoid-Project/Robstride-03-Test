#!/usr/bin/env python3
import argparse, math, struct, sys, threading, time
from pathlib import Path

import can

HOST_ID = 0xFD
MECH_POS_INDEX = 0x7019
MECH_VEL_INDEX = 0x701B
DEG = math.pi / 180.0
MOUNT_ROLL_DEG = 180.0
PRINT_HZ = 10

CHANNEL_MOTOR_IDS = {"can0": [1, 2, 3, 4, 5, 6], "can1": [7, 8, 9, 10, 11, 12]}
JOINT_MAP = {
    1: "left_hip_yaw", 2: "left_hip_pitch", 3: "left_hip_roll",
    4: "left_knee_pitch", 5: "left_ankle_upper", 6: "left_ankle_lower",
    7: "right_hip_yaw", 8: "right_hip_pitch", 9: "right_hip_roll",
    10: "right_knee_pitch", 11: "right_ankle_upper", 12: "right_ankle_lower",
}

parser = argparse.ArgumentParser(description="모터 pos/vel + IMU 값 실시간 출력")
parser.add_argument("--imu-port", default="/dev/ttyUSB0")
parser.add_argument("--channels", nargs="+", default=list(CHANNEL_MOTOR_IDS),
                    choices=list(CHANNEL_MOTOR_IDS))
parser.add_argument("--n100-dir",
                    default=str(Path(__file__).resolve().parents[2]
                                / "motor_control" / "motor_with_imu_test"),
                    help="n100*.so 가 있는 폴더")
parser.add_argument("--no-imu", action="store_true", help="모터만 출력")
parser.add_argument("--timeout", type=float, default=0.02, help="모터 응답 대기(초)")
args = parser.parse_args()

notes = []
imu_status = "비활성 (--no-imu)"

sys.path.insert(0, args.n100_dir)
n100 = None
if not args.no_imu:
    try:
        import n100
    except ImportError as e:
        imu_status = "n100 모듈 없음 — 빌드 필요"
        notes.append(f"[IMU] 임포트 실패: {e}")
        notes.append(f"      cd {args.n100_dir} && cmake -S . -B build "
                     f"-DCMAKE_BUILD_TYPE=Release && cmake --build build -j")
        notes.append(f"      경로가 다르면 --n100-dir 로 지정")

def build_arb(comm_type, data16, target_id):
    return ((comm_type & 0x1F) << 24) | ((data16 & 0xFFFF) << 8) | (target_id & 0xFF)

def parse_arb(arbitration_id):
    return ((arbitration_id >> 24) & 0x1F,
            (arbitration_id >> 8) & 0xFFFF,
            arbitration_id & 0xFF)

def read_param(bus, motor_id, index, timeout):
    data = bytearray(8)
    struct.pack_into("<H", data, 0, index)
    bus.send(can.Message(arbitration_id=build_arb(0x11, HOST_ID, motor_id),
                         data=bytes(data), is_extended_id=True))
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=max(0.0, deadline - time.monotonic()))
        if msg is None or not msg.is_extended_id:
            continue
        comm_type, data16, dest = parse_arb(msg.arbitration_id)
        if comm_type != 0x11 or dest != HOST_ID or (data16 & 0xFF) != motor_id:
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
        bus = can.Bus(channel=channel, interface="socketcan")
    except OSError as e:
        notes.append(f"[{channel}] 열기 실패: {e}  "
                     f"(sudo ip link set {channel} up type can bitrate 1000000)")
        return
    t0, cnt = time.time(), 0
    try:
        while not stop.is_set():
            for motor_id in ids:
                pos = read_param(bus, motor_id, MECH_POS_INDEX, args.timeout)
                vel = read_param(bus, motor_id, MECH_VEL_INDEX, args.timeout)
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
        notes.append(f"[{channel}] CAN 오류: {e}")
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
            imu_status = "포트는 열렸으나 데이터 없음"
            notes.append(f"[IMU] 3초 내 무응답: {driver.last_error() or '원인 불명'}")
            notes.append(f"      ./build/n100_cpp/read_imu {args.imu_port} 921600 180 0 0")
        else:
            imu_status = "정상"
    except RuntimeError as e:
        imu_status = "포트 열기 실패"
        driver = None
        notes.append(f"[IMU] 시작 실패: {e}")
        notes.append(f"      ls /dev/ttyUSB* /dev/ttyACM*   "
                     f"(권한: sudo chmod 666 {args.imu_port})")
        notes.append("      CANable 은 gs_usb 라 tty 로 안 잡힘. ttyUSB0 가 IMU 인지 확인")

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
        if driver is not None and not driver.is_running and imu_status == "정상":
            imu_status = f"리더 스레드 중단: {driver.last_error() or '원인 불명'}"

        shown_ids = [i for ch in args.channels for i in CHANNEL_MOTOR_IDS[ch]]

        out = ["\033[2J\033[3J\033[H"]
        hz = "   ".join(f"{ch} {rt[ch]:6.1f} Hz" for ch in args.channels)
        out.append(f"EVE state monitor   {hz}   IMU {imu_hz:5.1f} Hz    (Ctrl-C 종료)\n")
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
    print("종료")
