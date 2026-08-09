#!/usr/bin/env python3
"""robonex_balancing 에서 학습한 정책으로 매 스텝 action 을 추정해 출력한다.

읽기 전용이다. 모터에 명령을 보내지 않는다 — 정책이 무엇을 원하는지만
보여준다. 실제로 모터를 그 방향으로 움직이는 것은 이 스크립트의 범위 밖이다.

관측 벡터는 robonex_balancing 학습 로그의 params/env.yaml 에 기록된 순서를
그대로 따른다 (observations.policy 섹션에서 확인, concatenate_terms: true):

    joint_pos_rel(12) + joint_vel_rel(12) + imu_ang_vel(3) + projected_gravity(3)
    + last_action(12)  =  42

관절 순서(학습 시 asset_cfg.joint_names): l_hip_yaw, l_hip_pitch, l_hip_roll,
l_knee, l_ankle_roll, l_ankle_pitch, r_hip_yaw, r_hip_pitch, r_hip_roll, r_knee,
r_ankle_roll, r_ankle_pitch. default_joint_pos/vel 는 전부 0 이므로 *_rel 은
그냥 현재 값과 같다.

## ⚠ 무릎·발목 6개는 근사치다 (알려진 gap, 지어낸 값이 아님)

시뮬레이터의 l/r_knee_joint, l/r_ankle_roll_joint, l/r_ankle_pitch_joint 는
전부 "정강이/출력축" 값인데, CAN 모터가 실제로 재는 건 그 축이 아니다:

- **무릎**: CAN 모터(ID4·10)는 4절링크의 **크랭크** 축을 잰다. 크랭크와
  정강이(l_knee_joint)는 1입력→1출력이라 관계 자체는 있지만(중립에서 비율
  0.705, 전체 구간 0.21~1.23로 비선형), 크랭크 각도를 그대로 정강이 각도로
  써도 되는 게 아니다 — `robonex_description/scripts/robonex_serial.py`의
  `SERIAL_GAINS` 계산(`k_joint = MOTOR_KP / r^2`)이 그 증거: 게인에 r^2을
  곱해 변환한다는 것 자체가 두 각도가 다른 값이라는 뜻이다.
- **발목**: CAN 모터 4개(upper/lower, ID5·6·11·12)가 만드는 건 차동
  (differential)이라 둘 다 순수한 roll도 pitch도 아니다
  (`robonex_serial.py` 주석: "each ankle motor ~1.0 into roll and pitch" —
  모터 하나가 두 출력축에 같이 걸린다).

두 경우 다 정확히 변환하려면 크랭크/차동 기구학(CAD 링크 길이 기반)이
필요한데, project-unmeasured-params 메모 기준 **이 변환은 아직 유도되지
않았다**(무릎 링크 길이도 코드에 없고, 발목 크랭크 각도범위조차 URDF
플레이스홀더). 무릎은 원리상 어렵지 않은 문제(닫힌형 해 존재)지만, 링크
길이를 CAD에서 뽑아오기 전까지는 "쉬운 문제"와 "이미 푼 문제"는 다르다.

그래서 이 스크립트는 무릎·발목 관측 슬롯 6개를 실제 모터값으로 채우지
않고 0.0 으로 둔다. 틀린 근사치를 그럴듯하게 보여주는 것보다, 모른다는 걸
분명히 하는 쪽을 선택했다. 크랭크 모터 원시값은 참고용으로 별도 표시한다.
마찬가지 이유로 정책이 내놓는 무릎·발목 action 도 "출력축 목표각(가상)"일
뿐, 실제 모터 명령으로 바꿀 방법이 아직 없다 — 절대 그대로 CAN 에 실어
보내지 말 것.

## 각속도: raw 를 쓴다

시뮬레이터의 imu_ang_vel 은 물리엔진이 계산한 실제 각속도에 가우시안 잡음만
더한 값이라, AHRS 융합(내부 필터라서 위상 지연이 있음)보다 IMU 프레임의 원시
자이로(angular_velocity_raw)가 실제로 더 가깝다. print_policy_values.py 는
반대로 fused 를 기본으로 썼는데, 그건 그 스크립트가 정책 없이 그냥 사람이
보기 좋은 값을 보여주는 용도였기 때문이고 이유가 다르다.

    cd scripts/policy_test
    python3 print_policy_action.py
"""
import argparse
import math
import struct
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))   # n100*.so

import can  # noqa: E402
import n100  # noqa: E402

try:
    import numpy as np
except ImportError:
    sys.exit("numpy 가 필요합니다: pip install numpy")

try:
    import onnxruntime as ort
except ImportError:
    sys.exit("onnxruntime 가 필요합니다: pip install onnxruntime")

DEG = math.pi / 180.0

# 정책이 학습된 로그 폴더. 여러 런이 있으면 가장 최근(디렉터리명이 타임스탬프라
# 이름순 정렬이 시간순과 같다) 것의 exported/policy.onnx 를 기본값으로 쓴다.
BALANCING_LOGS = (Path.home() / "humanoid_project" / "robonex_balancing" /
                  "logs" / "rsl_rl" / "robonex_balancing")

# 학습 시 observations.policy.joint_pos_rel/joint_vel_rel 의 asset_cfg.joint_names
# 순서 (env.yaml 확인). default_joint_pos/vel 이 전부 0 이라 *_rel = 그냥 현재값.
JOINT_ORDER = [
    "l_hip_yaw", "l_hip_pitch", "l_hip_roll", "l_knee", "l_ankle_roll", "l_ankle_pitch",
    "r_hip_yaw", "r_hip_pitch", "r_hip_roll", "r_knee", "r_ankle_roll", "r_ankle_pitch",
]

# 이 6개(힙만)는 직결이라 CAN 모터 하나가 그 관절 그대로다 (hw-canbus.md
# 매핑표 확인). 무릎은 크랭크를 거치므로 여기 넣지 않는다 - 위 docstring 참고.
DIRECT_JOINT_TO_MOTOR = {
    "l_hip_yaw": 1, "l_hip_pitch": 2, "l_hip_roll": 3,
    "r_hip_yaw": 7, "r_hip_pitch": 8, "r_hip_roll": 9,
}
# 이 6개(무릎 2 + 발목 4)는 모터 값이 곧 관절 값이 아니라서(위 docstring
# 참고) 관측에 0.0 을 쓴다.
APPROX_JOINTS = {"l_knee", "r_knee", "l_ankle_roll", "l_ankle_pitch", "r_ankle_roll", "r_ankle_pitch"}

# 크랭크 모터(무릎 2 + 발목 4)는 참고 표시용으로만 읽는다. (motor_id, label)
CRANK_REFERENCE_MOTORS = [(4, "left_knee_pitch"), (5, "left_ankle_upper"), (6, "left_ankle_lower"),
                          (10, "right_knee_pitch"), (11, "right_ankle_upper"), (12, "right_ankle_lower")]

# actions.joint_pos.scale (env.yaml). offset 0, use_default_offset 이지만
# default_joint_pos 가 0 이라 target = scale * raw_action 그대로다.
ACTION_SCALE = {
    "hip_yaw": 0.12, "hip_pitch": 0.15, "hip_roll": 0.12,
    "knee": 0.15, "ankle_roll": 0.08, "ankle_pitch": 0.08,
}


def scale_of(joint):
    return ACTION_SCALE["_".join(joint.split("_")[1:])]


# IMU 장착 보정 (motor_imu_test 에서 실측 확인). print_policy_values.py 와 동일.
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


def find_latest_policy():
    candidates = sorted(BALANCING_LOGS.glob("*/exported/policy.onnx"))
    return candidates[-1] if candidates else None


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
    """한 채널의 모터들을 계속 순회하며 pos/vel 을 읽어 state 에 채운다. 읽기
    전용이다 (control 프레임을 보내지 않는다). print_policy_values.py 와 동일.
    """

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
        self._stop_event = threading.Event()   # threading.Thread._stop() 과 충돌 방지

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
    """학습 순서 그대로 42차원 관측 벡터를 만든다.

    무릎·발목 6개(APPROX_JOINTS)는 pos/vel 모두 0.0 — 위 docstring 참고.
    """
    pos = np.zeros(12, dtype=np.float32)
    vel = np.zeros(12, dtype=np.float32)
    for i, joint in enumerate(JOINT_ORDER):
        motor_id = DIRECT_JOINT_TO_MOTOR.get(joint)
        if motor_id is None:
            continue   # 무릎·발목 근사 슬롯: 0.0 유지
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
    parser.add_argument("--policy", type=Path, default=find_latest_policy(),
                        help="policy.onnx 경로. 기본값: logs/rsl_rl 아래 가장 최근 런")
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
    if args.policy is None:
        print(f"{BALANCING_LOGS} 아래에서 policy.onnx 를 찾지 못했습니다. "
              "--policy 로 직접 지정하세요.")
        return 1
    if not args.policy.exists():
        print(f"정책 파일이 없습니다: {args.policy}")
        return 1

    session = ort.InferenceSession(str(args.policy), providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name
    print(f"정책: {args.policy}")
    print(f"  입력 {session.get_inputs()[0].shape}  출력 {session.get_outputs()[0].shape}\n")

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

    time.sleep(0.3)   # CAN 리더가 첫 값을 채울 시간

    prev_action = np.zeros(12, dtype=np.float32)   # 다음 스텝 관측의 last_action 으로 먹인다.

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

            lines.append("직접 매핑 관절 (6개, CAN 모터 = 관절, 힙만)")
            lines.append(f"  {'관절':<14}  {'ID':>3}  {'pos [rad]':>10}  {'vel [rad/s]':>12}")
            for joint in JOINT_ORDER:
                motor_id = DIRECT_JOINT_TO_MOTOR.get(joint)
                if motor_id is None:
                    continue
                p, v = snapshot.get(motor_id, (None, None))
                pv = f"{p:+10.4f}" if p is not None else f"{'--':>10}"
                vv = f"{v:+12.4f}" if v is not None else f"{'--':>12}"
                lines.append(f"  {joint:<14}  {motor_id:>3}  {pv}  {vv}")

            lines.append("\n크랭크 모터 원시값 (무릎+발목, 참고용, 관측에는 포함되지 않음 — 위 docstring)")
            for motor_id, label in CRANK_REFERENCE_MOTORS:
                p, v = snapshot.get(motor_id, (None, None))
                pv = f"{p:+10.4f}" if p is not None else f"{'--':>10}"
                vv = f"{v:+12.4f}" if v is not None else f"{'--':>12}"
                lines.append(f"  {label:<18}  {motor_id:>3}  {pv}  {vv}")

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
                tag = "  (가상, 크랭크 기구학 미해결)" if joint in APPROX_JOINTS else ""
                lines.append(f"  {joint:<14}  {a:+8.4f}  {target:+12.4f}  "
                             f"{target / DEG:+7.2f}{tag}")

            lines.append("\n⚠ 무릎·발목 목표각은 정강이/출력축(pitch·roll) 기준의 정책 희망값일 뿐이다.")
            lines.append("   실제 크랭크/upper/lower 모터 명령으로 바꾸는 변환이 아직 없으니 그대로")
            lines.append("   CAN 에 실어 보내지 말 것 (무릎 2 + 발목 4, docstring 참고).")

            if notes:
                lines.append("")
                lines.extend(notes)

            sys.stdout.write("\n".join(lines) + "\n")
            sys.stdout.flush()

            prev_action = action.astype(np.float32)   # 다음 관측의 last_action.
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
