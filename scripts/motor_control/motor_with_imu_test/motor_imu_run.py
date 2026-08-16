#!/usr/bin/env python3





import argparse
import math
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import can
import n100

DEG = math.pi / 180.0


MOTIONS = {
    +1: {
        "label": "동작1 (앞으로 기울어짐)",
        "targets": {
            1: -0.1545,
            2: +0.3979,
            3: -0.0318,
            4: -0.5482,
            5: -0.0001,
            6: +0.0000,
            7: -0.0398,
            8: +0.0188,
            9: +0.0629,
            10: +0.0000,
            11: -0.0000,
            12: -0.0000,
        },
    },
    -1: {
        "label": "동작2 (뒤로 기울어짐)",
        "targets": {
            1: +0.0135,
            2: -0.0601,
            3: +0.0105,
            4: +0.0177,
            5: +0.0000,
            6: -0.0000,
            7: +0.0940,
            8: -0.4013,
            9: +0.0706,
            10: +0.5172,
            11: -0.0001,
            12: -0.0000,
        },
    },
}

MOUNT_ROLL_DEG = 180.0

PITCH_THRESHOLD_DEG = 10.0
PITCH_SIGN = +1
ZERO_TIME = 0.5

MOVE_TIME = 1.5
RATE = 100.0

MOVE_SPEED = 0.4
HOLD_KP = 40.0
HOLD_KD = 2.0
OVERSPEED_STOP = 2.0
FEEDBACK_TIMEOUT = 0.3

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
        self.last_feedback_time = 0.0

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

    def ingest_feedback(self, data, now=None):
        s = self.spec
        if len(data) < 8:
            return
        raw_pos = (data[0] << 8) | data[1]
        raw_vel = (data[2] << 8) | data[3]
        raw_torque = (data[4] << 8) | data[5]
        raw_temp = (data[6] << 8) | data[7]
        self.last_position = raw_pos / 65535.0 * (s.p_max - s.p_min) + s.p_min
        self.last_velocity = raw_vel / 65535.0 * (s.v_max - s.v_min) + s.v_min
        self.last_torque = raw_torque / 65535.0 * (s.t_max - s.t_min) + s.t_min
        self.last_temp = raw_temp / 10.0
        self.last_feedback_time = time.monotonic() if now is None else now


class FeedbackHub:


    def __init__(self, bus, motors, host_id):
        self.bus = bus
        self.motors = {m.motor_id: m for m in motors}
        self.host_id = host_id

    def _route(self, msg, now):
        if not msg.is_extended_id:
            return None
        comm_type, data16, destination = parse_arb(msg.arbitration_id)
        if comm_type != 0x02 or destination != self.host_id:
            return None
        motor = self.motors.get(data16 & 0xFF)
        if motor is not None:
            motor.ingest_feedback(bytes(msg.data), now)
        return motor

    def pump(self, max_frames=512):
        now = time.monotonic()
        for _ in range(max_frames):
            msg = self.bus.recv(timeout=0.0)
            if msg is None:
                return
            self._route(msg, now)

    def wait_for(self, motor_id, timeout):
        deadline = time.monotonic() + timeout
        target = self.motors.get(motor_id)
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            msg = self.bus.recv(timeout=remaining)
            if msg is None:
                return None
            if self._route(msg, time.monotonic()) is target and target is not None:
                return target.last_position




def pump_and_check(hubs, motors):

    for hub in hubs:
        hub.pump()
    now = time.monotonic()
    for motor_id, motor in motors.items():
        if abs(motor.last_velocity) > OVERSPEED_STOP:
            raise RuntimeError(f"ID {motor_id} 과속 ({motor.last_velocity:+.2f} rad/s)")
        if motor.last_feedback_time <= 0.0:
            raise RuntimeError(f"ID {motor_id} 피드백을 한 번도 못 받음")
        age = now - motor.last_feedback_time
        if age > FEEDBACK_TIMEOUT:
            raise RuntimeError(
                f"ID {motor_id} 피드백 끊김 ({age:.2f}s 무응답, 한계 {FEEDBACK_TIMEOUT}s)")


class Motion:


    def __init__(self, motors, hubs, start, targets, label):
        self.motors = motors
        self.hubs = hubs
        self.label = label
        self.start = dict(start)
        self.current = dict(start)
        self.target = {mid: targets.get(mid, start[mid]) for mid in motors}

        travel = max((abs(self.target[mid] - self.start[mid]) for mid in motors),
                     default=0.0)
        self.move_time = max(MOVE_TIME, travel / MOVE_SPEED)
        self.began = time.monotonic()

    def step(self, now):
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
        pump_and_check(self.hubs, self.motors)
        return progress >= 1.0


def pitch_deg(sample, offset):
    return PITCH_SIGN * (sample.euler.pitch / DEG - offset)


def tilt_direction(pitch):
    if pitch > PITCH_THRESHOLD_DEG:
        return +1
    if pitch < -PITCH_THRESHOLD_DEG:
        return -1
    return 0


def print_imu(sample, offset):
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

        motor_ids = [mid for mid in MOTOR_MODELS if channel_for_id(mid) in args.channels]
        for channel in sorted({channel_for_id(mid) for mid in motor_ids}):
            buses[channel] = can.Bus(channel=channel, interface=DEFAULT_INTERFACE)
        for motor_id in motor_ids:
            motors[motor_id] = Motor(buses[channel_for_id(motor_id)], motor_id,
                                     SPECS[MOTOR_MODELS[motor_id]])

        hub_by_channel = {
            channel: FeedbackHub(
                bus,
                [m for mid, m in motors.items() if channel_for_id(mid) == channel],
                HOST_ID,
            )
            for channel, bus in buses.items()
        }
        hubs = list(hub_by_channel.values())

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
        enabled = True
        for motor_id, motor in motors.items():
            motor.write_run_mode_operation()
            time.sleep(0.005)
            motor.enable()
            current = hub_by_channel[channel_for_id(motor_id)].wait_for(motor_id, timeout=0.3)
            if current is None:
                raise RuntimeError(f"ID {motor_id} 실시간 피드백 없음")
            pose[motor_id] = current
            motor.control(pos=current, vel=0.0, kp=HOLD_KP, kd=HOLD_KD)
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

            if motion is not None:
                if motion.step(now):
                    pose = motion.target
                    print(f"    {motion.label} 완료\n")
                    motion = None
            else:
                for motor_id, motor in motors.items():
                    motor.control(pos=pose[motor_id], vel=0.0, kp=HOLD_KP, kd=HOLD_KD)
                pump_and_check(hubs, motors)

            sample = driver.latest()
            if sample is None:
                continue
            if not driver.is_running:
                raise RuntimeError(f"IMU 리더 스레드 중단: {driver.last_error() or '원인 불명'}")

            if now - last_print >= 1.0:
                last_print = now
                print_imu(sample, offset)

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
                    start = motion.current
                    action = f"전환 ({motion.label} 도중)"
                motion = Motion(motors, hubs, start, spec["targets"], spec["label"])
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
