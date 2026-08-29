#!/usr/bin/env python3
# 파워 커넥터를 뽑기 전에 실행: can0/can1에 연결된 모터를 전부 부드럽게 정지시키고
# disable(motor.stop())한다. 어느 스크립트로 마지막에 뭘 제어하고 있었는지와 무관하게
# 항상 "다 꺼졌다"를 보장하는 게 목적이라, 다른 motor_control 스크립트와 마찬가지로
# 이 파일 하나로 완결되게 만들었다 (common.py 등 다른 모듈에 의존하지 않음).
import argparse
import sys
import time

import can

HOST_ID = 0xFD
DEFAULT_INTERFACE = "socketcan"

CHANNEL_ID_RANGES = {
    "can0": range(1, 7),
    "can1": range(7, 13),
}

JOINT_MAP = {
    1: "left_hip_yaw", 2: "left_hip_pitch", 3: "left_hip_roll",
    4: "left_knee_pitch", 5: "left_ankle_upper", 6: "left_ankle_lower",
    7: "right_hip_yaw", 8: "right_hip_pitch", 9: "right_hip_roll",
    10: "right_knee_pitch", 11: "right_ankle_upper", 12: "right_ankle_lower",
}

MOTOR_MODELS = {
    1: "rs02", 2: "rs03", 3: "rs03", 4: "rs03", 5: "rs02", 6: "rs02",
    7: "rs02", 8: "rs03", 9: "rs03", 10: "rs03", 11: "rs02", 12: "rs02",
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

CT_OPERATION = 0x01
CT_FEEDBACK = 0x02
CT_STOP = 0x04

# mode_status 값 (feedback 프레임 arbitration id 상위 2비트): 0=Reset, 1=Cali, 2=Motor(구동 중)
MODE_NAMES = {0: "Reset(정지됨)", 1: "Cali", 2: "Motor(구동중)"}


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def float_to_uint(x, x_min, x_max, bits):
    x = clamp(x, x_min, x_max)
    return int((x - x_min) / (x_max - x_min) * ((1 << bits) - 1))


def uint_to_float(raw, x_min, x_max, bits):
    return raw / float((1 << bits) - 1) * (x_max - x_min) + x_min


def build_arb(comm_type, data16, target_id):
    return ((comm_type & 0x1F) << 24) | ((data16 & 0xFFFF) << 8) | (target_id & 0xFF)


def parse_arb(arbitration_id):
    comm_type = (arbitration_id >> 24) & 0x1F
    data16 = (arbitration_id >> 8) & 0xFFFF
    destination = arbitration_id & 0xFF
    return comm_type, data16, destination


def parse_feedback(msg, host_id, motor_id, spec):
    if msg is None or not msg.is_extended_id:
        return None
    comm_type, data16, destination = parse_arb(msg.arbitration_id)
    if comm_type != CT_FEEDBACK or destination != host_id or (data16 & 0xFF) != motor_id:
        return None
    data = bytes(msg.data)
    if len(data) < 8:
        return None
    raw_vel = (data[2] << 8) | data[3]
    return {
        "velocity": uint_to_float(raw_vel, spec.v_min, spec.v_max, 16),
        "mode_status": (data16 >> 14) & 0x03,
    }


class Motor:
    def __init__(self, bus, motor_id, spec, host_id=HOST_ID):
        self.bus = bus
        self.motor_id = motor_id
        self.spec = spec
        self.host_id = host_id

    def _send(self, comm_type, data16, data):
        self.bus.send(can.Message(arbitration_id=build_arb(comm_type, data16, self.motor_id),
                                   data=bytes(data), is_extended_id=True))

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
        self._send(CT_OPERATION, data16, data)

    def stop(self, clear_fault=False):
        data = bytearray(8)
        if clear_fault:
            data[0] = 1
        self._send(CT_STOP, self.host_id, data)

    def poll_feedback(self, timeout=0.1):
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            msg = self.bus.recv(timeout=remaining)
            feedback = parse_feedback(msg, self.host_id, self.motor_id, self.spec)
            if feedback is not None:
                return feedback


def channel_for_id(motor_id):
    for channel, id_range in CHANNEL_ID_RANGES.items():
        if motor_id in id_range:
            return channel
    raise ValueError(f"모터 ID {motor_id} 에 대한 채널을 찾을 수 없습니다 (CHANNEL_ID_RANGES 확인).")


def parse_args():
    parser = argparse.ArgumentParser(
        description="파워를 뽑기 전에 can0/can1의 모터를 전부 부드럽게 세우고 disable한다.")
    parser.add_argument("--channels", nargs="+", default=list(CHANNEL_ID_RANGES.keys()),
                         choices=list(CHANNEL_ID_RANGES.keys()),
                         help=f"대상 CAN 채널들, 기본값: {' '.join(CHANNEL_ID_RANGES.keys())}")
    parser.add_argument("--ids", type=int, nargs="+", default=None,
                         help="대상 모터 ID (기본값: --channels 범위 전체, 1~12)")
    parser.add_argument("--interface", default=DEFAULT_INTERFACE, help="python-can 인터페이스, 기본값: socketcan")
    parser.add_argument("--host-id", type=lambda v: int(v, 0), default=HOST_ID, help="호스트 CAN ID, 기본값: 0xFD")
    parser.add_argument("--brake-time", type=float, default=0.3,
                         help="disable 전에 kd만 걸어 감속시키는 시간(초), 기본값: 0.3, 0이면 감속 없이 바로 disable")
    parser.add_argument("--kd", type=float, default=3.0, help="감속 구간 kd, 기본값: 3.0")
    return parser.parse_args()


def main():
    args = parse_args()
    target_ids = args.ids if args.ids else sorted(
        motor_id for channel in args.channels for motor_id in CHANNEL_ID_RANGES[channel]
    )

    buses = {}
    try:
        for channel in args.channels:
            buses[channel] = can.Bus(channel=channel, interface=args.interface)

        motors = {}
        for motor_id in target_ids:
            channel = channel_for_id(motor_id)
            if channel not in buses:
                continue
            model = MOTOR_MODELS.get(motor_id, "rs03")
            motors[motor_id] = Motor(buses[channel], motor_id, SPECS[model], args.host_id)

        if not motors:
            print("대상 모터가 없습니다.")
            return 1

        # 1) 잔여 속도가 있으면 kp=0, kd만 걸어서 부드럽게 0속도로 감속.
        #    (mujoco_hardware_twin.py의 brake_and_stop()과 같은 방식)
        if args.brake_time > 0.0:
            print(f"감속 중... ({args.brake_time:.2f}s, kd={args.kd})")
            deadline = time.monotonic() + args.brake_time
            while time.monotonic() < deadline:
                for motor in motors.values():
                    try:
                        motor.control(pos=0.0, vel=0.0, kp=0.0,
                                       kd=min(args.kd, motor.spec.kd_max), torque=0.0)
                        motor.poll_feedback(timeout=0.0)
                    except can.CanError:
                        pass
                time.sleep(0.01)

        # 2) 전부 disable.
        print("\ndisable 중...")
        print(f"{'ID':>3}  {'ch':<5}  {'joint':<18}  {'속도':>10}  결과")
        print("-" * 60)
        ok_count = 0
        for motor_id, motor in motors.items():
            channel = channel_for_id(motor_id)
            try:
                motor.stop()
            except can.CanError as e:
                print(f"{motor_id:>3}  {channel:<5}  {JOINT_MAP.get(motor_id, '?'):<18}  {'-':>10}  전송 실패 ({e})")
                continue

            feedback = motor.poll_feedback(timeout=0.2)
            if feedback is None:
                print(f"{motor_id:>3}  {channel:<5}  {JOINT_MAP.get(motor_id, '?'):<18}  {'-':>10}  무응답 (원래 꺼져있었을 수 있음)")
                continue

            mode = MODE_NAMES.get(feedback["mode_status"], f"?({feedback['mode_status']})")
            success = feedback["mode_status"] == 0
            if success:
                ok_count += 1
            result = "OK" if success else f"WARNING: {mode} 상태 (재시도 필요)"
            print(f"{motor_id:>3}  {channel:<5}  {JOINT_MAP.get(motor_id, '?'):<18}  "
                  f"{feedback['velocity']:>+9.3f}  {result}")

        print(f"\n{ok_count}/{len(motors)} 모터 disable 확인됨. "
              f"(무응답은 실패가 아니라 원래 꺼져 있었거나 배선이 없는 경우일 수 있습니다)")
        print("이제 파워 커넥터를 뽑아도 됩니다.")
        return 0 if ok_count == len(motors) else 1
    except KeyboardInterrupt:
        print("\n취소됨 — 그래도 지금까지 보낸 stop 명령은 유효합니다.")
        return 0
    finally:
        for bus in buses.values():
            bus.shutdown()


if __name__ == "__main__":
    sys.exit(main())
