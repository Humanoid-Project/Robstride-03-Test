#!/usr/bin/env python3








import argparse
import math
import struct
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import can
import n100

try:
    import numpy as np
except ImportError:
    sys.exit("numpy 가 필요합니다: pip install numpy")

try:
    import onnxruntime as ort
except ImportError:
    sys.exit("onnxruntime 가 필요합니다: pip install onnxruntime")

DEG = math.pi / 180.0

JOINT_ORDER = [
    "l_hip_yaw", "l_hip_pitch", "l_hip_roll", "l_knee_pitch",
    "l_ankle_upper", "l_ankle_lower",
    "r_hip_yaw", "r_hip_pitch", "r_hip_roll", "r_knee_pitch",
    "r_ankle_upper", "r_ankle_lower",
]

JOINT_TO_MOTOR = {
    "l_hip_yaw": 1, "l_hip_pitch": 2, "l_hip_roll": 3,
    "l_knee_pitch": 4, "l_ankle_upper": 5, "l_ankle_lower": 6,
    "r_hip_yaw": 7, "r_hip_pitch": 8, "r_hip_roll": 9,
    "r_knee_pitch": 10, "r_ankle_upper": 11, "r_ankle_lower": 12,
}

ACTION_SCALE = {
    "hip_yaw": 0.12, "hip_pitch": 0.15, "hip_roll": 0.12,
    "knee_pitch": 0.15, "ankle_upper": 0.08, "ankle_lower": 0.08,
}


def scale_of(joint):
    return ACTION_SCALE["_".join(joint.split("_")[1:])]


MOUNT_ROLL_DEG = 180.0

PRINT_HZ = 10.0
CAN_TIMEOUT = 0.02

HOST_ID = 0xFD
DEFAULT_INTERFACE = "socketcan"
MECH_POS_INDEX = 0x7019
MECH_VEL_INDEX = 0x701B

CHANNEL_MOTOR_IDS = {
    "can0": [1, 2, 3, 4, 5, 6],
    "can1": [7, 8, 9, 10, 11, 12],
}

MOTOR_LABEL = {
    1: "left_hip_yaw", 2: "left_hip_pitch", 3: "left_hip_roll", 4: "left_knee_pitch",
    5: "left_ankle_upper", 6: "left_ankle_lower",
    7: "right_hip_yaw", 8: "right_hip_pitch", 9: "right_hip_roll", 10: "right_knee_pitch",
    11: "right_ankle_upper", 12: "right_ankle_lower",
}


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


class CanReader(threading.Thread):

    def __init__(self, channel, motor_ids, interface, timeout, state, lock, notes):
        super().__init__(daemon=True)
        self.channel = channel
        self.motor_ids = motor_ids
        self.interface = interface
        self.timeout = timeout
        self.state = state
        self.lock = lock
        self.notes = notes
        self.rate_hz = 0.0
        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()

    def run(self):
        try:
            bus = can.Bus(channel=self.channel, interface=self.interface)
        except OSError as error:
            self.notes.append(f"[{self.channel}] 열기 실패: {error}  "
                              f"(sudo ip link set {self.channel} up type can bitrate 1000000)")
            return
        t0, cycles = time.monotonic(), 0
        try:
            while not self._stop_event.is_set():
                for motor_id in self.motor_ids:
                    pos = read_param(bus, motor_id, MECH_POS_INDEX, self.timeout)
                    vel = read_param(bus, motor_id, MECH_VEL_INDEX, self.timeout)
                    with self.lock:
                        self.state[motor_id] = (pos, vel)
                cycles += 1
                now = time.monotonic()
                if now - t0 >= 0.5:
                    self.rate_hz = cycles / (now - t0)
                    t0, cycles = now, 0
        except can.CanError as error:
            self.notes.append(f"[{self.channel}] CAN 오류: {error}")
        finally:
            bus.shutdown()


def build_observation(snapshot, ang_vel, gravity, prev_action):

    pos = np.zeros(12, dtype=np.float32)
    vel = np.zeros(12, dtype=np.float32)
    for i, joint in enumerate(JOINT_ORDER):
        motor_id = JOINT_TO_MOTOR[joint]
        p, v = snapshot.get(motor_id, (None, None))
        pos[i] = p if p is not None else 0.0
        vel[i] = v if v is not None else 0.0

    obs = np.concatenate([
        pos, vel,
        [ang_vel.x, ang_vel.y, ang_vel.z],
        [gravity.x, gravity.y, gravity.z],
        prev_action,
    ]).astype(np.float32)
    return obs.reshape(1, -1)


def main():
    parser = argparse.ArgumentParser(
        description="robonex_balancing 정책으로 관측값을 만들어 action 을 추정해 "
                    "출력한다. 읽기 전용, 모터에 명령을 보내지 않는다.")
    parser.add_argument("--policy", type=Path, required=True,
                        help="closed-loop motor-joint 순서로 학습한 policy.onnx 경로")
    parser.add_argument("--imu-port", default="/dev/ttyUSB0", help="IMU 시리얼 포트")
    parser.add_argument("--channels", nargs="+", default=list(CHANNEL_MOTOR_IDS),
                        choices=list(CHANNEL_MOTOR_IDS), help="사용할 CAN 채널")
    parser.add_argument("--interface", default=DEFAULT_INTERFACE, help="python-can 인터페이스")
    parser.add_argument("--timeout", type=float, default=CAN_TIMEOUT,
                        help="모터 1개, 파라미터 1개 요청의 응답 대기(초)")
    parser.add_argument("--rate", type=float, default=PRINT_HZ, help="출력 갱신 주파수 Hz")
    args = parser.parse_args()

    if args.rate <= 0:
        print("--rate 는 양수여야 합니다.")
        return 1
    if not args.policy.exists():
        print(f"정책 파일이 없습니다: {args.policy}")
        return 1

    session = ort.InferenceSession(str(args.policy), providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name
    input_shape = session.get_inputs()[0].shape
    output_shape = session.get_outputs()[0].shape
    if input_shape[-1] not in (None, "None", 42):
        print(f"정책 입력 크기가 42가 아닙니다: {input_shape}")
        return 1
    if output_shape[-1] not in (None, "None", 12):
        print(f"정책 출력 크기가 12가 아닙니다: {output_shape}")
        return 1
    print(f"정책: {args.policy}")
    print(f"  입력 {input_shape}  출력 {output_shape}")
    print("  의미: closed-loop motor-joint 12축 정책 (legacy output-joint 정책 사용 금지)\n")

    notes = []
    state = {mid: (None, None) for mid in MOTOR_LABEL}
    lock = threading.Lock()

    readers = [CanReader(channel, CHANNEL_MOTOR_IDS[channel], args.interface, args.timeout,
                         state, lock, notes)
              for channel in args.channels]
    for reader in readers:
        reader.start()

    driver = n100.ImuDriver(n100.DriverConfig(
        port=args.imu_port,
        mount_rotation=n100.Quat.from_axis_angle_x(MOUNT_ROLL_DEG * DEG),
    ))
    imu_status = "시작 중..."
    try:
        driver.start()
        if driver.wait_for_sample(timeout=3.0) is None:
            imu_status = "3초 내 무응답"
            notes.append(f"[IMU] {driver.last_error() or '원인 불명'}")
        else:
            imu_status = "정상"
    except RuntimeError as error:
        imu_status = "시작 실패"
        notes.append(f"[IMU] {error}")
        notes.append(f"      ls /dev/ttyUSB* /dev/ttyACM*  "
                     f"(권한: sudo chmod 666 {args.imu_port})")

    time.sleep(0.3)

    prev_action = np.zeros(12, dtype=np.float32)

    print("Ctrl-C 로 종료.\n")
    try:
        while True:
            with lock:
                snapshot = dict(state)
            sample = driver.latest()
            if imu_status == "정상" and not driver.is_running:
                imu_status = f"리더 스레드 중단: {driver.last_error() or '원인 불명'}"

            lines = ["\033[2J\033[3J\033[H"]
            can_hz = "  ".join(f"{r.channel} {r.rate_hz:5.1f} Hz" for r in readers)
            lines.append(f"정책 action 추정 (읽기 전용)   {can_hz}   (Ctrl-C 종료)\n")

            lines.append("closed-loop 구동 관절 (12개, CAN 모터축과 동일)")
            lines.append(f"  {'관절':<14}  {'ID':>3}  {'pos [rad]':>10}  {'vel [rad/s]':>12}")
            for joint in JOINT_ORDER:
                motor_id = JOINT_TO_MOTOR[joint]
                p, v = snapshot.get(motor_id, (None, None))
                pv = f"{p:+10.4f}" if p is not None else f"{'--':>10}"
                vv = f"{v:+12.4f}" if v is not None else f"{'--':>12}"
                lines.append(f"  {joint:<14}  {motor_id:>3}  {pv}  {vv}")

            if sample is None:
                lines.append("\nIMU 아직 샘플 없음, 관측은 0/gravity(0,0,-1)로 대체")
                ang_vel, gravity = n100.Vec3(), n100.Vec3(0.0, 0.0, -1.0)
            else:
                ang_vel, gravity = sample.angular_velocity_raw, sample.projected_gravity
                lines.append(f"\nIMU [{imu_status}]  각속도(raw) x {ang_vel.x:+8.4f}  "
                             f"y {ang_vel.y:+8.4f}  z {ang_vel.z:+8.4f}  [rad/s]")
                lines.append(f"        중력벡터   x {gravity.x:+8.4f}  y {gravity.y:+8.4f}  "
                             f"z {gravity.z:+8.4f}")

            obs = build_observation(snapshot, ang_vel, gravity, prev_action)
            t0 = time.perf_counter()
            action = session.run(None, {input_name: obs})[0][0]
            infer_ms = (time.perf_counter() - t0) * 1000.0

            lines.append(f"\n정책 출력 action (raw, {infer_ms:.2f} ms) → 목표각")
            lines.append(f"  {'관절':<14}  {'raw':>8}  {'목표각[rad]':>12}  {'[deg]':>8}")
            for joint, a in zip(JOINT_ORDER, action):
                target = a * scale_of(joint)
                lines.append(f"  {joint:<14}  {a:+8.4f}  {target:+12.4f}  "
                             f"{target / DEG:+7.2f}")

            lines.append("\n읽기 전용: 이 도구는 action을 CAN으로 전송하지 않는다.")

            if notes:
                lines.append("")
                lines.extend(notes)

            sys.stdout.write("\n".join(lines) + "\n")
            sys.stdout.flush()

            prev_action = action.astype(np.float32)
            time.sleep(1.0 / args.rate)
    except KeyboardInterrupt:
        pass
    finally:
        for reader in readers:
            reader.stop()
        for reader in readers:
            reader.join(timeout=2.0)
        driver.stop()
        print("종료")
    return 0


if __name__ == "__main__":
    sys.exit(main())
