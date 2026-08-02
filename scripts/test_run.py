#!/usr/bin/env python3
"""지정한 모터들을 현재 위치에서 목표각까지 아주 천천히 이동시킨 뒤,
코드가 종료(Ctrl-C)될 때까지 그 자리를 능동적으로 유지한다.

동작 순서:
  1) 각 모터의 현재 기계각을 읽는다 (응답 없으면 중단).
  2) 이동 계획을 출력하고 Enter 로 확인받는다.
  3) 각 모터를 enable(운영 모드) 한다.
  4) 현재각 -> 목표각을 smoothstep 궤적으로 "천천히" 이동한다.
  5) Ctrl-C 를 누를 때까지 목표각을 계속 유지한다.
  6) 종료 시 각 모터를 정지(disable)한다.

⚠️  실제 모터가 움직인다. 주변을 정리하고, 비상시 즉시 전원을 차단할 수 있게 한 뒤 실행할 것.

사용 예:
    python3 test_run.py
    python3 test_run.py --channel can0 --yes
"""
import argparse
import math
import struct
import sys
import time

import can

HOST_ID = 0xFD
DEFAULT_CHANNEL = "can0"
DEFAULT_INTERFACE = "socketcan"

RUN_MODE_INDEX = 0x7005
OPERATION_RUN_MODE = 0
MECH_POS_INDEX = 0x7019

# ── 설정 ────────────────────────────────────────────────────────────────
# 모터별 목표각(rad)과 모델("rs02"/"rs03").
#   target_rad : 이동해서 유지할 목표 기계각 (아래는 방금 읽은 값으로 채워둠)
#   model      : kp/kd 인코딩 스케일 결정. 실제 모델로 바로잡으면 홀딩이 정확해짐.
#                (기본 rs03 은 안전측: 실제가 rs02 여도 의도보다 약하게 적용됨)
MOTORS = {
    7:  {"target_rad": +1.2723, "model": "rs02"},
    8:  {"target_rad": -1.1790, "model": "rs03"},
    9:  {"target_rad": +0.0041, "model": "rs03"},
    10: {"target_rad": +1.5633, "model": "rs03"},
    11: {"target_rad": +3.2712, "model": "rs02"},
}

MOVE_SPEED = 0.25        # rad/s, 이동 속도 (작을수록 느림)
MIN_MOVE_TIME = 3.0      # s, 최소 이동 시간
RATE = 100.0            # Hz, 제어 주기
HOLD_KP = 40.0           # 위치 유지 강성 (처지면 키우고, 급격히 키우지 말 것)
HOLD_KD = 2.0            # 감쇠 (kp 올릴 때 같이 올려 진동 억제)
OVERSPEED_STOP = 2.0     # rad/s, 이 속도 초과 피드백이 오면 전체 비상정지
# ────────────────────────────────────────────────────────────────────────

JOINT_MAP = {
    1: "left_hip_yaw", 2: "left_hip_pitch", 3: "left_hip_roll",
    4: "left_knee_pitch", 5: "left_ankle_pitch", 6: "left_ankle_roll",
    7: "right_hip_yaw", 8: "right_hip_pitch", 9: "right_hip_roll",
    10: "right_knee_pitch", 11: "right_ankle_pitch", 12: "right_ankle_roll",
}


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
        """대기 중인 피드백(0x02)을 비우고, 위치/속도/토크/온도를 갱신한다."""
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


def fmt(rad):
    return f"{rad:+8.4f} rad ({math.degrees(rad):+8.2f} deg)"


def main():
    parser = argparse.ArgumentParser(
        description="모터들을 목표각으로 천천히 이동시킨 뒤 종료 시까지 위치를 유지한다.")
    parser.add_argument("--channel", default=DEFAULT_CHANNEL, help="CAN 채널, 기본값: can0")
    parser.add_argument("--interface", default=DEFAULT_INTERFACE, help="python-can 인터페이스")
    parser.add_argument("--host-id", type=lambda v: int(v, 0), default=HOST_ID, help="호스트 CAN ID")
    parser.add_argument("--yes", action="store_true", help="확인 프롬프트 없이 바로 시작")
    args = parser.parse_args()

    bus = can.Bus(channel=args.channel, interface=args.interface)
    motors = {}
    try:
        # 1) 각 모터의 목표각을 스펙 범위로 클램프하고 spec 준비
        for motor_id, cfg in MOTORS.items():
            spec = SPECS[cfg["model"]]
            motors[motor_id] = {
                "motor": Motor(bus, motor_id, spec, host_id=args.host_id),
                "target": clamp(cfg["target_rad"], spec.p_min, spec.p_max),
                "start": None,
            }

        # 2) 현재 위치 읽기
        print(f"채널 {args.channel} 에서 현재 위치 확인 중...\n")
        print(f"{'ID':>3}  {'joint':<18}  {'model':<5}  {'현재각':>26}  {'목표각':>26}")
        print("-" * 96)
        move_time = MIN_MOVE_TIME
        for motor_id, m in motors.items():
            current = m["motor"].read_mech_position(timeout=0.3)
            if current is None:
                print(f"{motor_id:>3}  {JOINT_MAP.get(motor_id, '?'):<18}  "
                      f"{m['motor'].spec.name:<5}  {'무응답':>26}")
                print(f"\n오류: 모터 ID {motor_id} 가 응답하지 않음. 배선/ID/전원을 확인하세요. 중단합니다.")
                return 1
            m["start"] = current
            travel = abs(m["target"] - current)
            move_time = max(move_time, travel / MOVE_SPEED)
            print(f"{motor_id:>3}  {JOINT_MAP.get(motor_id, '?'):<18}  "
                  f"{m['motor'].spec.name:<5}  {fmt(current):>26}  {fmt(m['target']):>26}")

        print(f"\n이동 시간: 약 {move_time:.1f} 초 (속도 {MOVE_SPEED} rad/s), "
              f"유지 강성 kp={HOLD_KP}, kd={HOLD_KD}")

        # 3) 확인
        if not args.yes:
            if not sys.stdin.isatty():
                print("확인 입력이 필요합니다. --yes 로 실행하거나 대화형 터미널에서 실행하세요.")
                return 1
            try:
                answer = input("\n시작하려면 Enter, 취소하려면 Ctrl-C: ")
                del answer
            except (KeyboardInterrupt, EOFError):
                print("\n취소됨.")
                return 0

        # 4) enable 후 현재 위치로 첫 홀딩 명령 (급점프 방지)
        print("\nEnable 중...")
        for m in motors.values():
            m["motor"].write_run_mode_operation()
            time.sleep(0.005)
            m["motor"].enable()
            time.sleep(0.005)
            m["motor"].control(pos=m["start"], vel=0.0, kp=HOLD_KP, kd=HOLD_KD)

        period = 1.0 / RATE

        def emergency_stop(reason):
            print(f"\n비상정지: {reason}")
            for mm in motors.values():
                try:
                    mm["motor"].stop()
                except can.CanError:
                    pass

        # 5) 천천히 이동 (smoothstep)
        print(f"이동 시작 ({move_time:.1f}s)...")
        start_time = time.monotonic()
        while True:
            now = time.monotonic()
            progress = min(1.0, (now - start_time) / move_time)
            smooth = progress * progress * (3.0 - 2.0 * progress)
            smooth_vel = 6.0 * progress * (1.0 - progress) / move_time
            for m in motors.values():
                travel = m["target"] - m["start"]
                m["motor"].control(
                    pos=m["start"] + travel * smooth,
                    vel=clamp(travel * smooth_vel, -MOVE_SPEED, MOVE_SPEED),
                    kp=HOLD_KP, kd=HOLD_KD,
                )
                m["motor"].drain_feedback()
                if abs(m["motor"].last_velocity) > OVERSPEED_STOP:
                    emergency_stop(f"ID {m['motor'].motor_id} 과속 "
                                   f"({m['motor'].last_velocity:+.2f} rad/s)")
                    return 1
            if progress >= 1.0:
                break
            sleep = period - (time.monotonic() - now)
            if sleep > 0:
                time.sleep(sleep)

        # 6) 종료(Ctrl-C)까지 위치 유지
        print("\n목표 위치 도달. 이제 위치를 유지합니다. 종료하려면 Ctrl-C.\n")
        last_print = 0.0
        while True:
            now = time.monotonic()
            for m in motors.values():
                m["motor"].control(pos=m["target"], vel=0.0, kp=HOLD_KP, kd=HOLD_KD)
                m["motor"].drain_feedback()
                if abs(m["motor"].last_velocity) > OVERSPEED_STOP:
                    emergency_stop(f"ID {m['motor'].motor_id} 과속 "
                                   f"({m['motor'].last_velocity:+.2f} rad/s)")
                    return 1
            if now - last_print >= 1.0:
                last_print = now
                print(f"[{time.strftime('%H:%M:%S')}] 위치 유지 중 "
                      f"(오차 큰 축 = 토크 부족/포화 의심)")
                print(f"  {'ID':>3} {'joint':<16} {'현재→목표(deg)':>20} "
                      f"{'오차(deg)':>10} {'토크(N·m)':>18} {'온도':>6}")
                for mid, m in motors.items():
                    mm = m["motor"]
                    tmax = mm.spec.t_max
                    cur = mm.last_position
                    if cur is None:
                        line = f"  {mid:>3} {JOINT_MAP.get(mid, '?'):<16} {'(피드백 없음)':>20}"
                    else:
                        err = math.degrees(m["target"] - cur)
                        tq_pct = abs(mm.last_torque) / tmax * 100.0 if tmax else 0.0
                        line = (f"  {mid:>3} {JOINT_MAP.get(mid, '?'):<16} "
                                f"{math.degrees(cur):+7.1f}→{math.degrees(m['target']):+7.1f}"
                                f" {err:+10.1f}"
                                f" {mm.last_torque:+7.2f}/{tmax:.0f}({tq_pct:3.0f}%)"
                                f" {mm.last_temp:5.1f}C")
                    print(line)
                print()
            sleep = period - (time.monotonic() - now)
            if sleep > 0:
                time.sleep(sleep)

    except KeyboardInterrupt:
        print("\n\n종료 요청됨. 모터 정지 중...")
    finally:
        for m in motors.values():
            try:
                m["motor"].stop()
            except can.CanError:
                pass
        bus.shutdown()
        print("정지 완료.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
