#!/usr/bin/env python3
"""IMU 가 앞으로 기울면 동작1, 뒤로 기울면 동작2 를 수행한다.

IMU 값은 전부 받아서 출력하고, 그 중 pitch 로 앞/뒤를 판정한다.

보정이 두 가지 들어간다.
  1) 장착 보정 - IMU 가 x축 기준 180도 뒤집혀 달려 있어서, 보정 전에는
     직립인데도 gproj = (0, 0, +0.99) 로 나온다. MOUNT_ROLL_DEG 로 되돌리면
     gproj = (0, 0, -0.99) 가 되고 roll 도 -177 도에서 +2 도로 정상화된다.
  2) 영점 보정 - 장착 보정을 해도 직립 자세의 pitch 가 0 이 아니다(+5.3 도).
     시작할 때 현재 pitch 를 재서 그만큼 빼고, 그 이후로는 그 자세를
     pitch 0 으로 본다. 그래서 시작 시점에 로봇이 똑바로 서 있어야 한다.

동작 중에 기울기가 반대로 바뀌면, 동작을 끄지 않고 그 순간의 위치에서
반대쪽 동작으로 그대로 이어서 이동한다.

    cd scripts/motor_imu_test
    python3 motor_imu_run.py

! 모터가 실제로 구동된다. 지지대에 올리고 비상정지를 확보한 뒤 실행할 것.
"""
import argparse
import math
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))   # n100*.so

import can  # noqa: E402
import n100  # noqa: E402

DEG = math.pi / 180.0

# ═════════════════════════════════════════════════════════════════════════════
# 설정
# ═════════════════════════════════════════════════════════════════════════════

# 동작 정의. {모터 ID: 목표각 rad}
MOTIONS = {
    +1: {
        "label": "동작1 (앞으로 기울어짐)",
        "targets": {
            1: -0.1545,   # left_hip_yaw
            2: +0.3979,   # left_hip_pitch
            3: -0.0318,   # left_hip_roll
            4: -0.5482,   # left_knee_pitch
            5: -0.0001,   # left_ankle_upper
            6: +0.0000,   # left_ankle_lower
            7: -0.0398,   # right_hip_yaw
            8: +0.0188,   # right_hip_pitch
            9: +0.0629,   # right_hip_roll
            10: +0.0000,  # right_knee_pitch
            11: -0.0000,  # right_ankle_upper
            12: -0.0000,  # right_ankle_lower
        },
    },
    -1: {
        "label": "동작2 (뒤로 기울어짐)",
        "targets": {
            1: +0.0135,   # left_hip_yaw
            2: -0.0601,   # left_hip_pitch
            3: +0.0105,   # left_hip_roll
            4: +0.0177,   # left_knee_pitch
            5: +0.0000,   # left_ankle_upper
            6: -0.0000,   # left_ankle_lower
            7: +0.0940,   # right_hip_yaw
            8: -0.4013,   # right_hip_pitch
            9: +0.0706,   # right_hip_roll
            10: +0.5172,  # right_knee_pitch
            11: -0.0001,  # right_ankle_upper
            12: -0.0000,  # right_ankle_lower
        },
    },
}

# IMU 장착 보정. IMU 케이스 -> 로봇 베이스 회전, 도 단위.
# 직립 상태에서 gproj 가 (0, 0, -1) 이 되는 값. 현재 장비는 x축 180도.
MOUNT_ROLL_DEG = 180.0

# 기울기 판정. 영점 보정 후의 pitch 기준, 도 단위.
PITCH_THRESHOLD_DEG = 10.0
# 앞으로 기울였는데 pitch 가 음수로 나오면 -1 로 바꿀 것.
PITCH_SIGN = +1
# 시작할 때 이 시간(초) 동안 pitch 를 평균내어 영점으로 삼는다.
ZERO_TIME = 0.5

MOVE_TIME = 1.5         # 동작 이동 시간, 초
RATE = 100.0            # 제어 루프 주파수, Hz

MOVE_SPEED = 0.4        # 이동 속도 상한, rad/s
HOLD_KP = 40.0          # 위치 유지 강성
HOLD_KD = 2.0
OVERSPEED_STOP = 2.0    # 이 속도를 넘으면 비상정지, rad/s

MOTOR_MODELS = {
    1: "rs02", 2: "rs03", 3: "rs03", 4: "rs03", 5: "rs02", 6: "rs02",
    7: "rs02", 8: "rs03", 9: "rs03", 10: "rs03", 11: "rs02", 12: "rs02",
}

JOINT_MAP = {
    1: "left_hip_yaw", 2: "left_hip_pitch", 3: "left_hip_roll",
    4: "left_knee_pitch", 5: "left_ankle_upper", 6: "left_ankle_lower",
    7: "right_hip_yaw", 8: "right_hip_pitch", 9: "right_hip_roll",
    10: "right_knee_pitch", 11: "right_ankle_upper", 12: "right_ankle_lower",
}

CHANNEL_ID_RANGES = {
    "can0": range(1, 7),
    "can1": range(7, 13),
}

# ═════════════════════════════════════════════════════════════════════════════
# CAN 프로토콜. scripts/motor_run/motor_pose_run.py 와 동일한 구현이다.
# ═════════════════════════════════════════════════════════════════════════════

HOST_ID = 0xFD
DEFAULT_INTERFACE = "socketcan"

RUN_MODE_INDEX = 0x7005
OPERATION_RUN_MODE = 0
MECH_POS_INDEX = 0x7019


class MotorSpec:
    def __init__(self, name, p_min, p_max, v_min, v_max, t_min, t_max, kp_max, kd_max):
        self.name = name
        self.p_min, self.p_max = p_min, p_max
        self.v_min, self.v_max = v_min, v_max
        self.t_min, self.t_max = t_min, t_max
        self.kp_max, self.kd_max = kp_max, kd_max


SPECS = {
    "rs03": MotorSpec("RS03", -12.57, 12.57, -20.0, 20.0, -60.0, 60.0, 5000.0, 100.0),
    "rs02": MotorSpec("RS02", -12.57, 12.57, -44.0, 44.0, -17.0, 17.0, 500.0, 5.0),
}


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def float_to_uint(x, x_min, x_max, bits):
    x = clamp(x, x_min, x_max)
    return int((x - x_min) / (x_max - x_min) * ((1 << bits) - 1))


def build_arb(comm_type, data16, target_id):
    return ((comm_type & 0x1F) << 24) | ((data16 & 0xFFFF) << 8) | (target_id & 0xFF)


def parse_arb(arbitration_id):
    comm_type = (arbitration_id >> 24) & 0x1F
    data16 = (arbitration_id >> 8) & 0xFFFF
    destination = arbitration_id & 0xFF
    return comm_type, data16, destination


def channel_for_id(motor_id):
    for channel, id_range in CHANNEL_ID_RANGES.items():
        if motor_id in id_range:
            return channel
    raise ValueError(f"모터 ID {motor_id} 에 대한 채널을 찾을 수 없습니다.")


class Motor:
    def __init__(self, bus, motor_id, spec, host_id=HOST_ID):
        self.bus = bus
        self.motor_id = motor_id
        self.spec = spec
        self.host_id = host_id
        self.last_velocity = 0.0
        self.last_position = None
        self.last_torque = 0.0
        self.last_temp = 0.0

    def _send(self, comm_type, data16, data):
        self.bus.send(can.Message(arbitration_id=build_arb(comm_type, data16, self.motor_id),
                                  data=bytes(data), is_extended_id=True))

    def read_mech_position(self, timeout=0.2):
        data = bytearray(8)
        struct.pack_into("<H", data, 0, MECH_POS_INDEX)
        self._send(0x11, self.host_id, data)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self.bus.recv(timeout=max(0.0, deadline - time.monotonic()))
            if msg is None or not msg.is_extended_id:
                continue
            comm_type, data16, destination = parse_arb(msg.arbitration_id)
            if comm_type != 0x11 or destination != self.host_id or (data16 & 0xFF) != self.motor_id:
                continue
            payload = bytes(msg.data)
            if len(payload) >= 8 and int.from_bytes(payload[0:2], "little") == MECH_POS_INDEX:
                return struct.unpack_from("<f", payload, 4)[0]
        return None

    def write_run_mode_operation(self):
        data = bytearray(8)
        struct.pack_into("<H", data, 0, RUN_MODE_INDEX)
        data[4] = OPERATION_RUN_MODE & 0xFF
        self._send(0x12, self.host_id, data)

    def enable(self):
        self._send(0x03, self.host_id, bytes(8))

    def stop(self, clear_fault=False):
        data = bytearray(8)
        if clear_fault:
            data[0] = 1
        self._send(0x04, self.host_id, data)

    def control(self, pos, vel, kp, kd, torque=0.0):
        s = self.spec
        data16 = float_to_uint(torque, s.t_min, s.t_max, 16)
        raw_pos = float_to_uint(pos, s.p_min, s.p_max, 16)
        raw_vel = float_to_uint(vel, s.v_min, s.v_max, 16)
        raw_kp = float_to_uint(kp, 0.0, s.kp_max, 16)
        raw_kd = float_to_uint(kd, 0.0, s.kd_max, 16)
        data = bytes([
            (raw_pos >> 8) & 0xFF, raw_pos & 0xFF,
            (raw_vel >> 8) & 0xFF, raw_vel & 0xFF,
            (raw_kp >> 8) & 0xFF, raw_kp & 0xFF,
            (raw_kd >> 8) & 0xFF, raw_kd & 0xFF,
        ])
        self._send(0x01, data16, data)

    def drain_feedback(self):
        s = self.spec
        while True:
            msg = self.bus.recv(timeout=0.0)
            if msg is None:
                return
            if not msg.is_extended_id:
                continue
            comm_type, data16, destination = parse_arb(msg.arbitration_id)
            if comm_type == 0x02 and destination == self.host_id and (data16 & 0xFF) == self.motor_id:
                data = bytes(msg.data)
                if len(data) >= 8:
                    raw_pos = (data[0] << 8) | data[1]
                    raw_vel = (data[2] << 8) | data[3]
                    raw_torque = (data[4] << 8) | data[5]
                    raw_temp = (data[6] << 8) | data[7]
                    self.last_position = raw_pos / 65535.0 * (s.p_max - s.p_min) + s.p_min
                    self.last_velocity = raw_vel / 65535.0 * (s.v_max - s.v_min) + s.v_min
                    self.last_torque = raw_torque / 65535.0 * (s.t_max - s.t_min) + s.t_min
                    self.last_temp = raw_temp / 10.0


# ═════════════════════════════════════════════════════════════════════════════
# 기울기 판정과 동작
# ═════════════════════════════════════════════════════════════════════════════


class Motion:
    """smoothstep 보간 이동을 매 틱 한 스텝씩 진행한다.

    블로킹하지 않으므로 이동 중에도 루프가 IMU 를 계속 읽는다. 진행 중인
    위치를 current 에 들고 있어서, 도중에 방향이 바뀌면 그 위치를 출발점으로
    삼아 반대쪽 동작으로 곧바로 이어갈 수 있다.
    """

    def __init__(self, motors, start, targets, label):
        self.motors = motors
        self.label = label
        self.start = dict(start)
        self.current = dict(start)
        # targets 에 없는 모터는 출발 자세를 그대로 유지한다.
        self.target = {mid: targets.get(mid, start[mid]) for mid in motors}

        travel = max((abs(self.target[mid] - self.start[mid]) for mid in motors),
                     default=0.0)
        self.move_time = max(MOVE_TIME, travel / MOVE_SPEED)
        self.began = time.monotonic()

    def step(self, now):
        """한 틱 진행. 이동이 끝났으면 True."""
        progress = min(1.0, (now - self.began) / self.move_time)
        smooth = progress * progress * (3.0 - 2.0 * progress)
        smooth_vel = 6.0 * progress * (1.0 - progress) / self.move_time

        for motor_id, motor in self.motors.items():
            travel = self.target[motor_id] - self.start[motor_id]
            position = self.start[motor_id] + travel * smooth
            self.current[motor_id] = position
            motor.control(
                pos=position,
                vel=clamp(travel * smooth_vel, -MOVE_SPEED, MOVE_SPEED),
                kp=HOLD_KP, kd=HOLD_KD,
            )
            motor.drain_feedback()
            if abs(motor.last_velocity) > OVERSPEED_STOP:
                raise RuntimeError(
                    f"ID {motor_id} 과속 ({motor.last_velocity:+.2f} rad/s)")
        return progress >= 1.0


def pitch_deg(sample, offset):
    """영점 보정한 pitch, 도 단위. 앞으로 기울면 양수."""
    return PITCH_SIGN * (sample.euler.pitch / DEG - offset)


def tilt_direction(pitch):
    """앞으로 기울었으면 +1, 뒤로 기울었으면 -1, 중립이면 0."""
    if pitch > PITCH_THRESHOLD_DEG:
        return +1
    if pitch < -PITCH_THRESHOLD_DEG:
        return -1
    return 0


def print_imu(sample, offset):
    """IMU 에서 받은 값을 전부 출력한다."""
    q, e = sample.orientation, sample.euler
    w, wr = sample.angular_velocity, sample.angular_velocity_raw
    a, m, g = sample.linear_acceleration, sample.magnetic_field, sample.projected_gravity
    pitch = pitch_deg(sample, offset)
    label = {0: "중립", +1: "앞", -1: "뒤"}[tilt_direction(pitch)]
    print(f"[{time.strftime('%H:%M:%S')}] seq {sample.seq}")
    print(f"  quat    w {q.w:+8.4f}  x {q.x:+8.4f}  y {q.y:+8.4f}  z {q.z:+8.4f}")
    print(f"  rpy     r {e.roll / DEG:+8.2f}  p {e.pitch / DEG:+8.2f}  "
          f"y {e.yaw / DEG:+8.2f}  [deg, 원본]")
    print(f"  gyro    x {w.x:+8.4f}  y {w.y:+8.4f}  z {w.z:+8.4f}  [rad/s]")
    print(f"  gyroR   x {wr.x:+8.4f}  y {wr.y:+8.4f}  z {wr.z:+8.4f}  [rad/s, raw]")
    print(f"  accel   x {a.x:+8.4f}  y {a.y:+8.4f}  z {a.z:+8.4f}  [m/s^2]")
    print(f"  mag     x {m.x:+8.2e}  y {m.y:+8.2e}  z {m.z:+8.2e}  [T]")
    print(f"  gproj   x {g.x:+8.4f}  y {g.y:+8.4f}  z {g.z:+8.4f}")
    print(f"  temp {sample.imu_temperature:5.1f} C   pressure {sample.pressure:9.1f} Pa")
    print(f"  pitch   {pitch:+8.2f} deg (영점 {offset:+.2f} 보정)  "
          f"판정 {label}  임계 +-{PITCH_THRESHOLD_DEG:.0f}\n")


def measure_pitch_offset(driver):
    """직립 자세의 pitch 를 평균내어 영점으로 삼는다."""
    total = 0.0
    count = 0
    last_seq = 0
    end = time.monotonic() + ZERO_TIME
    while time.monotonic() < end:
        sample = driver.wait_for_sample(timeout=0.2, last_seq=last_seq)
        if sample is None:
            continue
        last_seq = sample.seq
        total += sample.euler.pitch / DEG
        count += 1
    if count == 0:
        raise RuntimeError("영점 보정용 IMU 샘플을 받지 못했습니다.")
    return total / count


def main():
    parser = argparse.ArgumentParser(
        description="IMU 기울기로 모터 동작을 트리거한다.")
    parser.add_argument("--imu-port", default="/dev/ttyUSB0", help="IMU 시리얼 포트")
    parser.add_argument("--channels", nargs="+", default=list(CHANNEL_ID_RANGES),
                        choices=list(CHANNEL_ID_RANGES), help="사용할 CAN 채널")
    parser.add_argument("--yes", action="store_true", help="확인 프롬프트 없이 시작")
    args = parser.parse_args()

    # IMU 를 먼저 띄운다. 여기서 실패하면 모터는 아직 아무 상태도 아니다.
    driver = n100.ImuDriver(n100.DriverConfig(
        port=args.imu_port,
        mount_rotation=n100.Quat.from_axis_angle_x(MOUNT_ROLL_DEG * DEG),
    ))
    try:
        driver.start()
    except RuntimeError as error:
        print(f"IMU 시작 실패: {error}")
        print(f"포트를 확인하세요: ls /dev/ttyUSB* /dev/ttyACM*  "
              f"(권한이 없으면 sudo chmod 666 {args.imu_port})")
        return 1

    buses = {}
    motors = {}
    enabled = False
    try:
        print(f"IMU {args.imu_port}, 첫 샘플 대기 중...")
        if driver.wait_for_sample(timeout=3.0) is None:
            print(f"3초 안에 IMU 데이터가 오지 않았습니다: {driver.last_error() or '무응답'}")
            return 1
        print(f"IMU 스트림 정상. 장착 보정 Rx({MOUNT_ROLL_DEG:.0f}도) 적용됨.\n")

        print(f"영점 보정 중 ({ZERO_TIME:.1f}s), 로봇을 똑바로 세운 채 두세요...")
        offset = measure_pitch_offset(driver)
        print(f"  pitch 영점: {offset:+.2f} deg\n")
        print_imu(driver.latest(), offset)

        g = driver.latest().projected_gravity
        if g.z > 0:
            print(f"경고: 직립인데 gproj.z 가 {g.z:+.3f} 입니다 (정상은 -1 부근).")
            print("      MOUNT_ROLL_DEG 를 확인하세요.\n")

        # CAN 버스를 연다.
        motor_ids = [mid for mid in MOTOR_MODELS if channel_for_id(mid) in args.channels]
        for channel in sorted({channel_for_id(mid) for mid in motor_ids}):
            buses[channel] = can.Bus(channel=channel, interface=DEFAULT_INTERFACE)
        for motor_id in motor_ids:
            motors[motor_id] = Motor(buses[channel_for_id(motor_id)], motor_id,
                                     SPECS[MOTOR_MODELS[motor_id]])

        # 현재 자세를 읽어 기준으로 삼는다. 시작하자마자 로봇이 튀지 않게 한다.
        print("현재 위치 확인 중...\n")
        pose = {}
        for motor_id, motor in motors.items():
            current = motor.read_mech_position(timeout=0.3)
            if current is None:
                print(f"오류: 모터 ID {motor_id} 가 응답하지 않습니다.")
                return 1
            pose[motor_id] = current
            print(f"  {motor_id:>3}  {JOINT_MAP.get(motor_id, '?'):<18}  "
                  f"{current:+8.4f} rad ({math.degrees(current):+7.2f} deg)")

        print("\n모터가 실제로 구동됩니다.")
        if not args.yes:
            if not sys.stdin.isatty():
                print("확인 입력이 필요합니다. --yes 로 실행하세요.")
                return 1
            try:
                input("시작하려면 Enter, 취소하려면 Ctrl-C: ")
            except (KeyboardInterrupt, EOFError):
                print("\n취소됨.")
                return 0

        print("\nEnable 중...")
        for motor_id, motor in motors.items():
            motor.write_run_mode_operation()
            time.sleep(0.005)
            motor.enable()
            time.sleep(0.005)
            motor.control(pos=pose[motor_id], vel=0.0, kp=HOLD_KP, kd=HOLD_KD)
        enabled = True

        print(f"\n{RATE:.0f} Hz 루프 시작. Ctrl-C 로 종료.\n")

        period = 1.0 / RATE
        motion = None
        last_fired = 0
        last_print = 0.0
        next_tick = time.monotonic()

        while True:
            next_tick += period
            sleep = next_tick - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            now = time.monotonic()

            # ── 모터 ──
            if motion is not None:
                if motion.step(now):
                    pose = motion.target      # 도달한 자세를 계속 유지한다
                    print(f"    {motion.label} 완료\n")
                    motion = None
            else:
                for motor_id, motor in motors.items():
                    motor.control(pos=pose[motor_id], vel=0.0, kp=HOLD_KP, kd=HOLD_KD)
                    motor.drain_feedback()
                    if abs(motor.last_velocity) > OVERSPEED_STOP:
                        raise RuntimeError(
                            f"ID {motor_id} 과속 ({motor.last_velocity:+.2f} rad/s)")

            # ── IMU ──
            sample = driver.latest()
            if sample is None:
                continue
            if not driver.is_running:
                raise RuntimeError(f"IMU 리더 스레드 중단: {driver.last_error() or '원인 불명'}")

            if now - last_print >= 1.0:
                last_print = now
                print_imu(sample, offset)

            # ── 판정 ──
            # 중립으로 돌아오면 다시 발동할 수 있게 풀어준다. 같은 쪽으로
            # 계속 기울어 있어도 반복 발동하지는 않는다.
            pitch = pitch_deg(sample, offset)
            direction = tilt_direction(pitch)
            if direction == 0:
                last_fired = 0
            elif direction != last_fired:
                last_fired = direction
                spec = MOTIONS[direction]
                if motion is None:
                    start = pose
                    action = "발동"
                else:
                    # 진행 중이면 그 순간의 위치에서 반대쪽으로 이어간다.
                    start = motion.current
                    action = f"전환 ({motion.label} 도중)"
                motion = Motion(motors, start, spec["targets"], spec["label"])
                print(f"[{time.strftime('%H:%M:%S')}] {action}: {spec['label']}  "
                      f"(pitch {pitch:+.2f} deg, {motion.move_time:.1f}s 이동)")

    except KeyboardInterrupt:
        print("\n종료 요청됨.")
        return 0
    except RuntimeError as error:
        print(f"\n중단: {error}")
        return 1
    except (OSError, can.CanError) as error:
        print(f"\nCAN 오류: {error}")
        print("인터페이스가 올라와 있는지 확인하세요:")
        print("  sudo modprobe gs_usb")
        for channel in args.channels:
            print(f"  sudo ip link set {channel} up type can bitrate 1000000")
        return 1
    finally:
        if enabled:
            print("모터 정지 중...")
            for motor in motors.values():
                try:
                    motor.stop()
                except can.CanError:
                    pass
        for bus in buses.values():
            bus.shutdown()
        driver.stop()
        print("정지 완료.")


if __name__ == "__main__":
    sys.exit(main())
