"""Single source of truth for the standalone RoboNex communication stack."""

from __future__ import annotations

from dataclasses import dataclass
import math


CONTROL_HZ = 60.0
CONTROL_PERIOD_S = 1.0 / CONTROL_HZ

HOST_ID = 0xFD
DEFAULT_CAN_INTERFACE = "socketcan"
DEFAULT_CAN_TIMEOUT_S = 0.020

CHANNEL_MOTOR_IDS = {
    "can0": (1, 2, 3, 4, 5, 6),
    "can1": (7, 8, 9, 10, 11, 12),
}
MOTOR_IDS = tuple(motor_id for ids in CHANNEL_MOTOR_IDS.values() for motor_id in ids)

JOINT_NAMES = {
    1: "left_hip_yaw",
    2: "left_hip_pitch",
    3: "left_hip_roll",
    4: "left_knee_pitch",
    5: "left_ankle_upper",
    6: "left_ankle_lower",
    7: "right_hip_yaw",
    8: "right_hip_pitch",
    9: "right_hip_roll",
    10: "right_knee_pitch",
    11: "right_ankle_upper",
    12: "right_ankle_lower",
}

MOTOR_MODELS = {
    1: "rs02",
    2: "rs03",
    3: "rs03",
    4: "rs03",
    5: "rs02",
    6: "rs02",
    7: "rs02",
    8: "rs03",
    9: "rs03",
    10: "rs03",
    11: "rs02",
    12: "rs02",
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


@dataclass(frozen=True)
class MotorSpec:
    name: str
    position_min: float
    position_max: float
    velocity_min: float
    velocity_max: float
    torque_min: float
    torque_max: float
    kp_max: float
    kd_max: float


MOTOR_SPECS = {
    "rs03": MotorSpec("RS03", -12.57, 12.57, -20.0, 20.0, -60.0, 60.0, 5000.0, 100.0),
    "rs02": MotorSpec("RS02", -12.57, 12.57, -44.0, 44.0, -17.0, 17.0, 500.0, 5.0),
}

# Robstride private CAN protocol communication types.
CT_OPERATION = 0x01
CT_FEEDBACK = 0x02
CT_ENABLE = 0x03
CT_STOP = 0x04
CT_PARAM_READ = 0x11
CT_PARAM_WRITE = 0x12
CT_ACTIVE_REPORT = 0x18

# Robstride parameter indexes used by the communication stack.
RUN_MODE_INDEX = 0x7005
SPEED_REFERENCE_INDEX = 0x700A
TORQUE_LIMIT_INDEX = 0x700B
CURRENT_LIMIT_INDEX = 0x7018
MECHANICAL_POSITION_INDEX = 0x7019
MECHANICAL_VELOCITY_INDEX = 0x701B
ACCELERATION_INDEX = 0x7022
ACTIVE_REPORT_INTERVAL_INDEX = 0x7026
FAULT_STATUS_INDEX = 0x3022

RUN_MODE_OPERATION = 0
RUN_MODE_VELOCITY = 2

DEFAULT_IMU_PORT = "/dev/ttyUSB0"
DEFAULT_IMU_BAUDRATE = 921600
IMU_MOUNT_ROLL_RAD = math.pi
EXPECTED_UPRIGHT_GRAVITY = (-0.049, -0.036, -0.998)

# These values must come from the exact RoboNex training configuration. Empty
# tuples deliberately make later observation/control code fail configuration
# validation instead of silently using guessed values.
POLICY_JOINT_ORDER: tuple[str, ...] = ()
DEFAULT_JOINT_POSITIONS_RAD: tuple[float, ...] = ()
ACTION_SCALE_RAD: tuple[float, ...] = ()


def channel_for_motor(motor_id: int) -> str:
    for channel, motor_ids in CHANNEL_MOTOR_IDS.items():
        if motor_id in motor_ids:
            return channel
    raise ValueError(f"모터 ID {motor_id}에 대응하는 CAN 채널이 없습니다.")


def joint_limit_for(
    motor_id: int,
    margin: float = DEFAULT_LIMIT_MARGIN_RAD,
) -> tuple[float, float]:
    lower, upper = JOINT_LIMITS_RAD[motor_id]
    return lower + margin, upper - margin


def exceeds_joint_limit(
    position: float,
    motor_id: int,
    margin: float = DEFAULT_LIMIT_MARGIN_RAD,
) -> bool:
    lower, upper = joint_limit_for(motor_id, margin)
    return position <= lower or position >= upper
