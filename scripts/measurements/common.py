import math
import time

from robonex_common.can import Motor
from robonex_common.joints import ACTUATED_JOINTS, JOINT_LIMITS_BY_ID
from robonex_common.limits import DEFAULT_LIMIT_MARGIN_RAD, exceeds_joint_limit, joint_limit_for
from robonex_common.motors import MOTOR_SPECS, PEAK_TORQUE, RATED_TORQUE
from robonex_common.protocol import (
    DEFAULT_INTERFACE,
    FAULT_STATUS_INDEX,
    HOST_ID,
    MECHANICAL_POSITION_INDEX,
    RUN_MODE_INDEX,
    RUN_MODE_OPERATION,
)

SPECS = MOTOR_SPECS
JOINT_MAP = {joint.motor_id: joint.hardware_name for joint in ACTUATED_JOINTS}
JOINT_LIMITS_RAD = JOINT_LIMITS_BY_ID
MECH_POS_INDEX = MECHANICAL_POSITION_INDEX
FAULT_STA_INDEX = FAULT_STATUS_INDEX
PLACEHOLDER_ARMATURE = {"rs02": 0.003, "rs03": 0.017}
PLACEHOLDER_DAMPING = {"rs02": 0.2, "rs03": 0.2}

FAULT_BIT_NAMES = {
    0: "Overtemperature (>145C)",
    1: "Driver chip fault",
    2: "Undervoltage (<12V)",
    3: "Overvoltage (>60V)",
    4: "Phase B overcurrent",
    5: "Phase C overcurrent",
    7: "Encoder not calibrated",
    8: "Hardware identification fault",
    9: "Position initialization fault",
    14: "Stall overload",
    16: "Phase A overcurrent",
}


def channel_for_id(motor_id):
    joint = next((joint for joint in ACTUATED_JOINTS if joint.motor_id == motor_id), None)
    if joint is None:
        raise ValueError(f"No CAN channel for motor ID {motor_id}")
    return joint.channel


def decode_fault_bits(value):
    return [name for bit, name in FAULT_BIT_NAMES.items() if value & (1 << bit)]


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
            problems.append(f"--{name} must be finite ({value})")
            continue
        if kind == "positive" and value <= 0:
            problems.append(f"--{name} must be positive ({value})")
        elif kind == "nonneg" and value < 0:
            problems.append(f"--{name} must be non-negative ({value})")
        elif kind == "torque" and abs(value) > PEAK_TORQUE[model]:
            problems.append(
                f"--{name}: |{value}| exceeds the {model.upper()} peak torque {PEAK_TORQUE[model]} N*m"
            )
        elif kind == "speed" and abs(value) > spec.v_max:
            problems.append(f"--{name}: |{value}| exceeds the {model.upper()} speed limit {spec.v_max} rad/s")
    return problems


def report_invalid_args(problems):
    if not problems:
        return False
    print("Invalid arguments; nothing will run:")
    for problem in problems:
        print(f"  {problem}")
    return True


def active_brake(motor, duration=0.3, kd=3.0):
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        motor.control(pos=0.0, vel=0.0, kp=0.0, kd=kd, torque=0.0)
        motor.poll_feedback(timeout=0.05)
