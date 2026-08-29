#!/usr/bin/env python3




import math
import struct
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

JOINT_LIMITS_RAD = {
    1: (-0.698132, 0.698132),
    2: (-0.872665, 0.872665),
    3: (-1.047198, 0.087266),
    4: (-0.872665, 0.087266),
    5: (-0.610865, 0.436332),
    6: (-0.436332, 0.610865),
    7: (-0.698132, 0.698132),
    8: (-0.872665, 0.872665),
    9: (-0.087266, 1.047198),
    10: (-0.087266, 0.872665),
    11: (-0.436332, 0.610865),
    12: (-0.610865, 0.436332),
}

DEFAULT_LIMIT_MARGIN_RAD = 0.05


def joint_limit_for(motor_id, margin=DEFAULT_LIMIT_MARGIN_RAD):
    lo, hi = JOINT_LIMITS_RAD[motor_id]
    return lo + margin, hi - margin


def exceeds_joint_limit(pos, motor_id, margin=DEFAULT_LIMIT_MARGIN_RAD):
    lo, hi = joint_limit_for(motor_id, margin)
    return pos <= lo or pos >= hi

RUN_MODE_INDEX = 0x7005
SPD_REF_INDEX = 0x700A
LIMIT_TORQUE_INDEX = 0x700B
LIMIT_CUR_INDEX = 0x7018
MECH_POS_INDEX = 0x7019
MECH_VEL_INDEX = 0x701B
ACC_RAD_INDEX = 0x7022
EPSCAN_TIME_INDEX = 0x7026
FAULT_STA_INDEX = 0x3022

RUN_MODE_OPERATION = 0
RUN_MODE_VELOCITY = 2

CT_OPERATION = 0x01
CT_FEEDBACK = 0x02
CT_ENABLE = 0x03
CT_STOP = 0x04
CT_PARAM_READ = 0x11
CT_PARAM_WRITE = 0x12
CT_ACTIVE_REPORT = 0x18
DEFAULT_VEL_LIMIT_CUR = {"rs02": 4.0, "rs03": 8.0}
DEFAULT_VEL_ACCEL = {"rs02": 20.0, "rs03": 10.0}


def channel_for_id(motor_id):
    for channel, id_range in CHANNEL_ID_RANGES.items():
        if motor_id in id_range:
            return channel
    raise ValueError(f"모터 ID {motor_id} 에 대한 채널을 찾을 수 없습니다 (CHANNEL_ID_RANGES 확인).")


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

RATED_TORQUE = {"rs02": 6.0, "rs03": 20.0}
PEAK_TORQUE = {"rs02": 17.0, "rs03": 60.0}

PLACEHOLDER_ARMATURE = {"rs02": 0.003, "rs03": 0.017}
PLACEHOLDER_DAMPING = {"rs02": 0.2, "rs03": 0.2}

FAULT_BIT_NAMES = {
    0: "과온(>145C)",
    1: "드라이버칩 폴트",
    2: "저전압(<12V)",
    3: "과전압(>60V)",
    4: "B상 과전류",
    5: "C상 과전류",
    7: "엔코더 미캘리브",
    8: "하드웨어 식별 폴트",
    9: "위치 초기화 폴트",
    14: "스톨 과부하",
    16: "A상 과전류",
}


def decode_fault_bits(value):
    return [name for bit, name in FAULT_BIT_NAMES.items() if value & (1 << bit)]


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


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


class Motor:

    def __init__(self, bus, motor_id, spec, host_id=HOST_ID):
        self.bus = bus
        self.motor_id = motor_id
        self.spec = spec
        self.host_id = host_id
        self.last_position = None
        self.last_velocity = 0.0
        self.last_torque = 0.0
        self.last_temp = 0.0
        self.last_fault = 0

    def _send(self, comm_type, data16, data):
        self.bus.send(can.Message(arbitration_id=build_arb(comm_type, data16, self.motor_id),
                                   data=bytes(data), is_extended_id=True))

    def enable(self):
        self._send(CT_ENABLE, self.host_id, bytes(8))

    def stop(self, clear_fault=False):
        data = bytearray(8)
        if clear_fault:
            data[0] = 1
        self._send(CT_STOP, self.host_id, data)

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

    def write_param_u8(self, index, value):
        data = bytearray(8)
        struct.pack_into("<H", data, 0, index)
        data[4] = value & 0xFF
        self._send(CT_PARAM_WRITE, self.host_id, data)

    def write_param_f32(self, index, value):
        data = bytearray(8)
        struct.pack_into("<H", data, 0, index)
        struct.pack_into("<f", data, 4, value)
        self._send(CT_PARAM_WRITE, self.host_id, data)

    def write_param_u16(self, index, value):
        data = bytearray(8)
        struct.pack_into("<H", data, 0, index)
        struct.pack_into("<H", data, 4, value)
        self._send(CT_PARAM_WRITE, self.host_id, data)

    def set_active_report(self, enable):
        data = bytearray(8)
        data[0] = 1 if enable else 0
        self._send(CT_ACTIVE_REPORT, self.host_id, data)

    def read_param(self, index, fmt="<f", timeout=0.2):
        data = bytearray(8)
        struct.pack_into("<H", data, 0, index)
        self._send(CT_PARAM_READ, self.host_id, data)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self.bus.recv(timeout=max(0.0, deadline - time.monotonic()))
            if msg is None or not msg.is_extended_id:
                continue
            comm_type, data16, destination = parse_arb(msg.arbitration_id)
            if comm_type != CT_PARAM_READ or destination != self.host_id or (data16 & 0xFF) != self.motor_id:
                continue
            payload = bytes(msg.data)
            if len(payload) >= 8 and int.from_bytes(payload[0:2], "little") == index:
                return struct.unpack_from(fmt, payload, 4)[0]
        return None

    def read_param_f32(self, index, timeout=0.2):
        return self.read_param(index, fmt="<f", timeout=timeout)

    def poll_feedback(self, timeout=0.05):


        s = self.spec
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            msg = self.bus.recv(timeout=remaining)
            if msg is None or not msg.is_extended_id:
                return None
            comm_type, data16, destination = parse_arb(msg.arbitration_id)
            if comm_type != CT_FEEDBACK or destination != self.host_id or (data16 & 0xFF) != self.motor_id:
                continue
            data = bytes(msg.data)
            if len(data) < 8:
                continue
            raw_pos = (data[0] << 8) | data[1]
            raw_vel = (data[2] << 8) | data[3]
            raw_torque = (data[4] << 8) | data[5]
            raw_temp = (data[6] << 8) | data[7]
            pos = uint_to_float(raw_pos, s.p_min, s.p_max, 16)
            vel = uint_to_float(raw_vel, s.v_min, s.v_max, 16)
            torque = uint_to_float(raw_torque, s.t_min, s.t_max, 16)
            temp = raw_temp / 10.0
            fault = (data16 >> 8) & 0xFF
            self.last_position, self.last_velocity = pos, vel
            self.last_torque, self.last_temp, self.last_fault = torque, temp, fault
            return time.monotonic(), pos, vel, torque, temp, fault


def validate_args(args, model, checks):


    spec = SPECS[model]
    problems = []
    for name, kind in checks:
        attr = name.replace("-", "_")
        if not hasattr(args, attr):
            continue
        value = getattr(args, attr)
        if value is None:
            continue
        if not math.isfinite(value):
            problems.append(f"--{name}: 유한한 숫자가 아님 ({value})")
            continue
        if kind == "positive" and value <= 0:
            problems.append(f"--{name}: 0보다 커야 함 (지금 {value})")
        elif kind == "nonneg" and value < 0:
            problems.append(f"--{name}: 0 이상이어야 함 (지금 {value})")
        elif kind == "torque" and abs(value) > PEAK_TORQUE[model]:
            problems.append(f"--{name}: |{value}| 가 {model.upper()} 피크토크 "
                            f"{PEAK_TORQUE[model]} N*m 초과 (모터에는 clamp된 값이 나가 "
                            f"안전한계가 의도와 달라짐)")
        elif kind == "speed" and abs(value) > spec.v_max:
            problems.append(f"--{name}: |{value}| 가 {model.upper()} 최대속도 "
                            f"{spec.v_max} rad/s 초과 (모터에는 clamp된 값이 나감)")
    return problems


def report_invalid_args(problems):
    if not problems:
        return False
    print("인자 오류 — 실행하지 않습니다:")
    for p in problems:
        print(f"  {p}")
    return True


def active_brake(motor, duration=0.3, kd=3.0):

    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        motor.control(pos=0.0, vel=0.0, kp=0.0, kd=kd, torque=0.0)
        motor.poll_feedback(timeout=0.05)
