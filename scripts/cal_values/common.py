#!/usr/bin/env python3
"""RS02/RS03 사설 프로토콜 공용 헬퍼 — scripts/cal_values 전용.

기존 scripts/motor_run, scripts/calibration 등과 같은 사설 프로토콜(확장 CAN 프레임,
comm_type 0x01/0x02/0x03/0x04/0x11/0x12)을 쓴다. build_arb/parse_arb/SPECS 등은
scripts/motor_run/motor_pose_run.py 와 동일한 값이다 — 두 곳을 고칠 일이 생기면 같이 볼 것.

참고 문서:
  - docs/RS03_RSP1000_reference.md ("RS03 Private Protocol Command Types",
    "RS03 Parameter Indexes", "RS03 Operation Control Scaling")
  - docs/RS02User Manual260428.pdf (RS02는 RS03과 프로토콜 공통, 스케일만 다름)

핵심 확인 사항 (2026-08-09):
  0x7019 mechPos / 0x701B mechVel 은 매뉴얼에 "Load-end mechanical angle/speed"로
  명시되어 있다 — 즉 감속기 이후 출력축 기준 값이다. 실시간 제어에 쓰는 type 0x02
  피드백 프레임의 위치/속도도 같은 절대엔코더(출력축, 14-bit)에서 나온 값으로 봐도
  된다 (motor_pose_run.py/motor_calibration.py가 이미 그렇게 취급 중). 따라서 이
  값들로 구한 관성/댐핑/마찰은 감속비 보정 없이 그대로 URDF armature/damping/friction에
  대응한다.

  ⚠⚠ 사고 기록(2026-08-09): 위 "같은 값으로 봐도 된다"는 **절댓값이 인코딩 범위
  (±4π≈±12.57rad, control()/poll_feedback()이 쓰는 범위) 안에 있을 때만** 유효하다.
  반복측정으로 누적 회전이 이 범위를 넘으면(그날 최대 39rad, 12바퀴 이상까지 누적됨
  확인) mechPos(0x7019, 절대/멀티턴 값으로 보임)와 실시간 피드백이 서로 다른 wrap
  구간을 쓰게 되어 값이 최대 8π(≈25rad)까지 어긋난다. 이 상태에서 mechPos를
  control()의 위치 목표(pos, kp>0)로 쓰면 목표-현재 오차가 터무니없이 크게 계산되어
  모터가 그 가짜 오차를 없애려 전력으로 돌아버린다 — 실제로 이 버그 때문에 모터가
  급회전해 CAN 어댑터 케이블이 뽑히는 사고가 있었다(armature/friction의 정지판정
  단계). **규칙: mechPos(read_param_f32(MECH_POS_INDEX))는 정보 출력에만 쓰고,
  control()의 pos 인자로는 절대 넘기지 말 것.** 위치가 필요하면(안전한계 기준점 등)
  반드시 poll_feedback()으로 받은 값을 쓸 것 — 이 프로토콜에서 kp=0으로 두면(속도만
  제어) 애초에 위치 목표 자체가 없어 이 문제에서 자유롭다.
"""
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

# 파라미터 인덱스 (0x70xx, 사설 프로토콜, RS02/RS03 공통)
RUN_MODE_INDEX = 0x7005      # uint8: 0=운동제어(operation), 2=속도모드, 5=CSP ...
SPD_REF_INDEX = 0x700A       # float: 속도모드 목표속도 rad/s
LIMIT_TORQUE_INDEX = 0x700B  # float: N*m
LIMIT_CUR_INDEX = 0x7018     # float: A
MECH_POS_INDEX = 0x7019      # float, read-only, load-end 기계각 rad
MECH_VEL_INDEX = 0x701B      # float, read-only, load-end 속도 rad/s
ACC_RAD_INDEX = 0x7022       # float: 속도모드 가속도 rad/s^2
EPSCAN_TIME_INDEX = 0x7026   # uint16: 능동보고 주기, 1=10ms, +1마다 5ms씩 증가
FAULT_STA_INDEX = 0x3022     # uint32, read-only, 폴트 상태 레지스터 (host-computer table)

RUN_MODE_OPERATION = 0
RUN_MODE_VELOCITY = 2

# comm_type 값
CT_OPERATION = 0x01
CT_FEEDBACK = 0x02
CT_ENABLE = 0x03
CT_STOP = 0x04
CT_PARAM_READ = 0x11
CT_PARAM_WRITE = 0x12
# ⚠ CT_ACTIVE_REPORT(0x18)과 RUN_MODE_VELOCITY 경로는 2026-08-09에 damping 측정에서
# 시도했으나 실패했다(능동보고 프레임이 사실상 안 옴) — 페이로드 정확한 형식이 매뉴얼에
# 없어 data[0]=1로 추측한 게 틀렸을 가능성이 높다. damping 측정은 대신 운영제어(type 0x01,
# kp=0/kd>0로 속도추종)로 우회해서 성공했다 → damping/capture_velocity_hold.py 참고.
# 아래 값들은 나중에 능동보고 형식을 알아내면 다시 쓸 수 있어 남겨둔 것— 지금은 미사용.
CT_ACTIVE_REPORT = 0x18
DEFAULT_VEL_LIMIT_CUR = {"rs02": 4.0, "rs03": 8.0}   # A
DEFAULT_VEL_ACCEL = {"rs02": 20.0, "rs03": 10.0}      # rad/s^2, 속도 램프 가속도


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


# motor_pose_run.py 의 SPECS 와 동일 (출처: docs/RS03_RSP1000_reference.md +
# RS02 매뉴얼 v1.0). RS02/RS03 스케일이 서로 다르니 혼동 주의.
SPECS = {
    "rs03": MotorSpec("RS03", -12.57, 12.57, -20.0, 20.0, -60.0, 60.0, 5000.0, 100.0),
    "rs02": MotorSpec("RS02", -12.57, 12.57, -44.0, 44.0, -17.0, 17.0, 500.0, 5.0),
}

# RS02 정격/피크 토크 (hw-rs02.md) — 무부하 벤치 테스트에서 이 이상 토크는 보통 불필요.
RATED_TORQUE = {"rs02": 6.0, "rs03": 20.0}
PEAK_TORQUE = {"rs02": 17.0, "rs03": 60.0}

# 현재 시뮬레이터에 들어가 있는 추정값 (memory/hw-rs02.md, hw-rs03.md, 미검증) — 비교용.
PLACEHOLDER_ARMATURE = {"rs02": 0.003, "rs03": 0.017}
# robonex_description의 구동 조인트 damping 임의값(class="act", RS02/RS03 구분 없이 동일값 사용 중) — 비교용.
PLACEHOLDER_DAMPING = {"rs02": 0.2, "rs03": 0.2}

# 0x3022 faultSta 비트 (docs/RS03_RSP1000_reference.md "RS03 Fault Bits").
# decode_fault_bits()는 motor.read_param(FAULT_STA_INDEX, fmt="<I") 로 읽은 "진짜" 레지스터
# 값에만 쓸 것 — poll_feedback()이 CAN ID 필드에서 뽑아내는 fault_byte는 2026-08-09 실측으로
# 이 레지스터와 다른 값임이 확인됐다(운영제어 중 fault_byte=0x80 인데 동시에 읽은 진짜
# 레지스터는 0x00000000). fault_byte의 실제 의미는 불명.
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
    """모터 1개에 대한 사설 프로토콜 래퍼. bus는 이미 열려 있는 python-can Bus."""

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
        """type 0x01 운영제어. kp=kd=0 이면 torque가 순수 피드포워드 토크가 된다."""
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
        """type 0x18. 속도/전류 모드처럼 type 0x01을 계속 안 보내는 상태에서도 모터가
        type 0x02 피드백을 주기적으로 스스로 보내게 한다 (주기는 EPSCAN_TIME_INDEX)."""
        data = bytearray(8)
        data[0] = 1 if enable else 0
        self._send(CT_ACTIVE_REPORT, self.host_id, data)

    def read_param(self, index, fmt="<f", timeout=0.2):
        """type 0x11 단일 파라미터 읽기. fmt는 struct 포맷(예: "<f" float32, "<I" uint32)."""
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
        """type 0x02 피드백 프레임 하나를 기다려서 파싱한다 (블로킹, timeout 초).

        control()/enable()/stop() 직후 모터가 자동으로 이 프레임을 보내온다.
        반환: (t_recv, pos, vel, torque, temp, fault_byte) 또는 None (타임아웃/불일치).

        fault_byte는 data16의 상위 바이트 — 애초 "폴트 축약 코드"로 추정했으나 2026-08-09
        실측으로 틀린 것으로 확인됨(운영제어 중 계속 0x80이 뜨는데 같은 순간 진짜 0x3022
        레지스터를 읽으면 0x00000000/정상). 실제 의미 불명 — 안전 판단에 쓰지 말고 참고
        출력에만 쓸 것. capture_torque_step.py도 이걸 자동중단 트리거로 안 씀.
        """
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
    """측정 인자를 검사한다. 문제가 있으면 사유 목록을 반환(빈 리스트면 통과).

    ⚠ 왜 필요한가: control()의 float_to_uint()는 범위를 벗어난 값을 **조용히 clamp**한다.
    그래서 사용자가 --max-vel 100 처럼 스펙 밖 값을 줘도 에러 없이 통과하고, "내가 건 안전
    한계"와 "실제 모터에 나간 값"이 달라진다. NaN/inf/0/음수도 마찬가지로 통과해버려서
    루프가 영원히 안 끝나거나 안전검사가 무력화된다 — 실기 구동 전에 반드시 걸러야 한다.

    checks: [(인자명, 종류)] 목록. 종류는
      "positive"   : 0보다 큰 유한값
      "nonneg"     : 0 이상 유한값
      "torque"     : 유한값이고 |값| <= 피크토크
      "speed"      : 유한값이고 |값| <= 모터 최대속도
    """
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
    """validate_args 결과를 출력한다. 문제가 있으면 True(=중단해야 함)."""
    if not problems:
        return False
    print("인자 오류 — 실행하지 않습니다:")
    for p in problems:
        print(f"  {p}")
    return True


def active_brake(motor, duration=0.3, kd=3.0):
    """측정 종료 직후 곧바로 stop()(전원 차단)하기 전에 짧게 능동 감쇠를 건다.

    2026-08-09 확인: 이 로봇의 RS02는 damping이 사실상 0이라(b≈0.001) 그냥 stop()만
    하면 정지마찰(운동마찰 ~0.14N*m)만으로 감속해야 해서 한동안 관성으로 미끄러지듯
    돈다. kp=0, vel=0, kd만 걸면 위치 목표 없이 순수 속도감쇠라 안전하다(mechPos
    도메인 문제와 무관 — 모듈 docstring의 사고 기록 참고)."""
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        motor.control(pos=0.0, vel=0.0, kp=0.0, kd=kd, torque=0.0)
        motor.poll_feedback(timeout=0.05)
