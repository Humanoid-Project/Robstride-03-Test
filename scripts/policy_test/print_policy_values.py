#!/usr/bin/env python3
"""보행 정책이 매 스텝 받는 관측값을 터미널에 출력한다. 읽기 전용, 모터에
명령을 보내지 않는다 (같은 폴더의 read_joint_values.py, read_motor_imu.py 와
같은 성격).

찍는 값:
  - 관절 위치   (12, rad)      CAN 으로 읽은 기계각
  - 관절 속도   (12, rad/s)    CAN 으로 읽은 기계각속도
  - 각속도      (3, rad/s)     IMU, AHRS 융합값, 로봇 베이스 프레임
  - 중력벡터    (3)            IMU projected_gravity, 직립이면 (0, 0, -1)
  - 이전 action (12)           placeholder, 항상 0

이전 action 은 이 스크립트가 정책을 실행하지 않아서 실제 값이 없다. 정책을
붙일 때는 매 스텝 끝에서 방금 내보낸 action 을 여기에 넣고, 다음 스텝의
관측값으로 그대로 먹여야 한다 (아래 PREVIOUS_ACTION 참고).

관측 벡터의 실제 순서와 스케일은 학습 시 정의한 설정을 따라야 하며, 아래
"관측 벡터 미리보기" 는 값을 눈으로 확인하기 위한 것일 뿐 그 순서를 그대로
정책에 넣어도 된다는 보장은 아니다.

    cd scripts/policy_test
    python3 print_policy_values.py
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

DEG = math.pi / 180.0

# IMU 장착 보정. 케이스가 x축 기준 180도 뒤집혀 있다 (motor_imu_test 에서 실측
# 확인). 직립 상태에서 gravity 가 (0, 0, -1) 이 되어야 한다.
MOUNT_ROLL_DEG = 180.0

PRINT_HZ = 10.0
CAN_TIMEOUT = 0.02   # 모터 1개, 파라미터 1개 요청의 응답 대기, 초

HOST_ID = 0xFD
DEFAULT_INTERFACE = "socketcan"
MECH_POS_INDEX = 0x7019
MECH_VEL_INDEX = 0x701B

CHANNEL_MOTOR_IDS = {
    "can0": [1, 2, 3, 4, 5, 6],
    "can1": [7, 8, 9, 10, 11, 12],
}

JOINT_MAP = {
    1: "left_hip_yaw", 2: "left_hip_pitch", 3: "left_hip_roll",
    4: "left_knee_pitch", 5: "left_ankle_upper", 6: "left_ankle_lower",
    7: "right_hip_yaw", 8: "right_hip_pitch", 9: "right_hip_roll",
    10: "right_knee_pitch", 11: "right_ankle_upper", 12: "right_ankle_lower",
}

# 이 스크립트는 정책을 실행하지 않으므로 항상 0 벡터다. 정책을 붙일 때 매 스텝
# 끝에서 실제로 내보낸 action 으로 갱신할 것.
PREVIOUS_ACTION = [0.0] * 12


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
    """한 채널의 모터들을 계속 순회하며 pos/vel 을 읽어 state 에 채운다.

    메인 루프는 락으로 보호된 스냅샷만 읽으므로, CAN 왕복 시간 때문에
    출력이 멈추지 않는다. 모터에 read_param(0x11) 요청만 보내고 control
    프레임은 보내지 않는다 — 읽기 전용이다.
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
        # threading.Thread 는 내부적으로 self._stop() 이라는 이름을 이미 쓰고
        # 있어서, 같은 이름의 속성을 붙이면 join() 이 깨진다. 그래서 _stop_event
        # 로 이름을 피해서 쓴다.
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


def main():
    parser = argparse.ArgumentParser(
        description="보행 정책 관측값(관절 위치/속도, IMU 각속도/중력벡터, "
                    "이전 action)을 터미널에 출력한다. 읽기 전용.")
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

    notes = []
    state = {mid: (None, None) for mid in JOINT_MAP}
    lock = threading.Lock()

    motor_ids = sorted(mid for channel in args.channels for mid in CHANNEL_MOTOR_IDS[channel])
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
            lines.append(f"정책 관측값   {can_hz}   (Ctrl-C 종료)\n")

            lines.append(f"관절 위치/속도 ({len(motor_ids)}개)")
            lines.append(f"  {'ID':>3}  {'joint':<18}  {'pos [rad]':>10}  {'vel [rad/s]':>12}")
            lines.append("  " + "-" * 50)
            joint_pos, joint_vel = [], []
            for motor_id in motor_ids:
                pos, vel = snapshot.get(motor_id, (None, None))
                joint_pos.append(pos if pos is not None else 0.0)
                joint_vel.append(vel if vel is not None else 0.0)
                p = f"{pos:+10.4f}" if pos is not None else f"{'--':>10}"
                v = f"{vel:+12.4f}" if vel is not None else f"{'--':>12}"
                lines.append(f"  {motor_id:>3}  {JOINT_MAP[motor_id]:<18}  {p}  {v}")

            lines.append(f"\nIMU [{imu_status}]  포트 {args.imu_port}")
            if sample is None:
                lines.append("  아직 샘플 없음")
                ang_vel, gravity = n100.Vec3(), n100.Vec3(0.0, 0.0, -1.0)
            else:
                ang_vel, gravity = sample.angular_velocity, sample.projected_gravity
                raw = sample.angular_velocity_raw
                lines.append(f"  각속도    x {ang_vel.x:+8.4f}  y {ang_vel.y:+8.4f}  "
                             f"z {ang_vel.z:+8.4f}  [rad/s, AHRS fused]")
                lines.append(f"  각속도R   x {raw.x:+8.4f}  y {raw.y:+8.4f}  z {raw.z:+8.4f}  "
                             f"[rad/s, raw, 참고용]")
                lines.append(f"  중력벡터  x {gravity.x:+8.4f}  y {gravity.y:+8.4f}  "
                             f"z {gravity.z:+8.4f}")

            lines.append("\n이전 action")
            lines.append("  " + "  ".join(f"{v:+.3f}" for v in PREVIOUS_ACTION))

            obs = [*joint_pos, *joint_vel, ang_vel.x, ang_vel.y, ang_vel.z,
                  gravity.x, gravity.y, gravity.z, *PREVIOUS_ACTION]
            lines.append(f"\n관측 벡터 ({len(obs)} = {len(joint_pos)} pos + "
                         f"{len(joint_vel)} vel + 3 ang_vel + 3 gravity + "
                         f"{len(PREVIOUS_ACTION)} prev_action)")
            lines.append("  [" + ", ".join(f"{v:+.3f}" for v in obs) + "]")

            if notes:
                lines.append("")
                lines.extend(notes)

            sys.stdout.write("\n".join(lines) + "\n")
            sys.stdout.flush()
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
