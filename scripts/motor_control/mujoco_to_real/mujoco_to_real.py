#!/usr/bin/env python3
from __future__ import annotations

import argparse
import contextlib
from dataclasses import dataclass
import math
from pathlib import Path
import sys
import time

import can
import mujoco
import mujoco.viewer
import numpy as np

THIS_FILE = Path(__file__).resolve()
REPO_ROOT = THIS_FILE.parents[3]
PROJECT_ROOT = THIS_FILE.parents[4]
MOTOR_TEST_DIR = THIS_FILE.parents[1] / "motor_test"
MEASUREMENTS_DIR = REPO_ROOT / "scripts" / "measurements"
DEFAULT_MODEL_PATH = (
    PROJECT_ROOT / "robonex_description" / "mujoco" / "scene_fixed.xml"
)

sys.path.insert(0, str(MOTOR_TEST_DIR))
sys.path.insert(0, str(MEASUREMENTS_DIR))

from common import JOINT_LIMITS_RAD
from set_motor_pose import (
    DEFAULT_INTERFACE,
    FeedbackHub,
    HOST_ID,
    JOINT_MAP,
    Motor,
    SPECS,
    build_arb,
    channel_for_id,
    clamp,
    parse_arb,
)

MOTOR_MODELS = {
    1: "rs02", 2: "rs03", 3: "rs03", 4: "rs03", 5: "rs02", 6: "rs02",
    7: "rs02", 8: "rs03", 9: "rs03", 10: "rs03", 11: "rs02", 12: "rs02",
}

MOTOR_ACTUATORS = {
    1: "l_hip_yaw",
    2: "l_hip_pitch",
    3: "l_hip_roll",
    4: "l_knee_pitch",
    5: "l_ankle_upper",
    6: "l_ankle_lower",
    7: "r_hip_yaw",
    8: "r_hip_pitch",
    9: "r_hip_roll",
    10: "r_knee_pitch",
    11: "r_ankle_upper",
    12: "r_ankle_lower",
}

MAX_CONFIG_SPEED = 0.5
MAX_CONFIG_ACCEL = 2.0
MAX_CONFIG_RATE = 200.0
ZERO_SETTLE_TIMEOUT = 5.0


@dataclass
class AxisLimiter:

    position: float
    velocity: float = 0.0

    def step(self, target: float, dt: float, max_speed: float, max_accel: float):
        if not all(math.isfinite(v) for v in (target, dt, max_speed, max_accel)):
            raise ValueError("rate limiter received a non-finite value")
        if dt <= 0.0 or max_speed <= 0.0 or max_accel <= 0.0:
            raise ValueError("dt, max_speed and max_accel must be positive")

        error = target - self.position
        if abs(error) <= 1e-12 and abs(self.velocity) <= 1e-12:
            self.position = target
            self.velocity = 0.0
            return self.position, self.velocity

        braking_speed = max(
            0.0,
            math.sqrt((max_accel * dt) ** 2 + 2.0 * max_accel * abs(error))
            - max_accel * dt,
        )
        desired_velocity = math.copysign(min(max_speed, braking_speed), error)
        dv = clamp(desired_velocity - self.velocity, -max_accel * dt, max_accel * dt)
        next_velocity = clamp(self.velocity + dv, -max_speed, max_speed)
        next_position = self.position + next_velocity * dt

        if error * (target - next_position) <= 0.0:
            next_position = target
            next_velocity = 0.0

        self.position = next_position
        self.velocity = next_velocity
        return self.position, self.velocity


def wrap_to_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def align_angle(current, target):
    return current + wrap_to_pi(target - current)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            "MuJoCo fixed-base Control 목표를 RoboNex 모터가 제한속도로 추종. "
            "기본은 시뮬 전용이며 --hardware 없이는 CAN 명령을 보내지 않음."
        )
    )
    parser.add_argument("--hardware", action="store_true",
                        help="실물 CAN 추종 활성화(기본: 시뮬레이션 전용)")
    parser.add_argument("--motor-id", "--motor-ids", dest="motor_id",
                        nargs="+", type=lambda v: int(v, 0),
                        default=list(range(1, 13)),
                        help=("실물 추종 대상 ID 한 개 이상. 예: --motor-id 5 또는 "
                              "--motor-id 5 6. 기본: 1~12 전부"))
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL_PATH,
                        help=f"fixed-base MJCF scene, 기본: {DEFAULT_MODEL_PATH}")
    parser.add_argument("--interface", default=DEFAULT_INTERFACE,
                        help="python-can 인터페이스, 기본: socketcan")
    parser.add_argument("--host-id", type=lambda v: int(v, 0), default=HOST_ID)
    parser.add_argument("--rate", type=float, default=100.0,
                        help="실물 명령 주기 Hz, 기본: 100")
    parser.add_argument("--max-speed", type=float, default=0.10,
                        help="실물 목표 최대속도 rad/s, 기본: 0.10(약 5.7deg/s)")
    parser.add_argument("--max-accel", type=float, default=0.25,
                        help="실물 목표 최대가속도 rad/s^2, 기본: 0.25")
    parser.add_argument("--kp", type=float, default=40.0)
    parser.add_argument("--kd", type=float, default=2.0)
    parser.add_argument("--zero-tolerance-deg", type=float, default=3.0,
                        help="zero 이동 완료로 판정할 실제 위치 허용범위, 기본: 3deg")
    parser.add_argument("--limit-margin-deg", type=float, default=3.0,
                        help="URDF 하드스톱 안쪽 여유, 기본: 3deg")
    parser.add_argument("--feedback-timeout", type=float, default=0.30,
                        help="type-0x02 freshness watchdog, 기본: 0.30s")
    parser.add_argument("--overspeed", type=float, default=2.0,
                        help="실측 속도 비상정지 한계 rad/s, 기본: 2.0")
    parser.add_argument("--max-error-deg", type=float, default=25.0,
                        help="명령-실측 추종오차 비상정지 한계, 기본: 25deg")
    parser.add_argument("--max-temp", type=float, default=70.0,
                        help="모터 온도 비상정지 한계 degC, 기본: 70")
    parser.add_argument("--brake-time", type=float, default=0.20,
                        help="종료 전 kp=0 능동감쇠 시간, 기본: 0.20s")
    parser.add_argument("--yes", action="store_true",
                        help="실물 시작 확인 문구 생략(--hardware와 함께만 의미 있음)")
    parser.add_argument("--headless", action="store_true",
                        help="뷰어 없이 실행(검증용; --duration 필요)")
    parser.add_argument("--duration", type=float, default=None,
                        help="지정 초 후 자동 종료(주로 dry-run 검증용)")
    return parser.parse_args(argv)


def validate_args(args):
    problems = []
    motor_ids = sorted(set(args.motor_id))
    unknown = [mid for mid in motor_ids if mid not in MOTOR_MODELS]
    if unknown:
        problems.append(f"지원하지 않는 motor-id: {unknown}")

    numeric_positive = (
        "rate", "max_speed", "max_accel", "feedback_timeout", "overspeed",
        "max_error_deg", "max_temp",
    )
    for name in numeric_positive:
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0.0:
            problems.append(f"--{name.replace('_', '-')}: 0보다 큰 유한값이어야 함 ({value})")
    for name in ("kp", "kd", "limit_margin_deg", "brake_time"):
        value = getattr(args, name)
        if not math.isfinite(value) or value < 0.0:
            problems.append(f"--{name.replace('_', '-')}: 0 이상의 유한값이어야 함 ({value})")
    if not math.isfinite(args.zero_tolerance_deg) or args.zero_tolerance_deg <= 0.0:
        problems.append(
            f"--zero-tolerance-deg: 0보다 큰 유한값이어야 함 ({args.zero_tolerance_deg})"
        )
    if args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0.0):
        problems.append(f"--duration: 0보다 큰 유한값이어야 함 ({args.duration})")
    if args.headless and args.duration is None:
        problems.append("--headless에는 종료 가능한 --duration이 필요함")
    if args.rate > MAX_CONFIG_RATE:
        problems.append(f"--rate는 {MAX_CONFIG_RATE:g}Hz 이하여야 함")
    if args.max_speed > MAX_CONFIG_SPEED:
        problems.append(f"--max-speed는 이 저속 도구에서 {MAX_CONFIG_SPEED:g}rad/s 이하여야 함")
    if args.max_accel > MAX_CONFIG_ACCEL:
        problems.append(f"--max-accel은 {MAX_CONFIG_ACCEL:g}rad/s^2 이하여야 함")
    if args.overspeed <= args.max_speed:
        problems.append("--overspeed는 --max-speed보다 커야 함")
    if motor_ids and not unknown:
        min_kp_max = min(SPECS[MOTOR_MODELS[mid]].kp_max for mid in motor_ids)
        min_kd_max = min(SPECS[MOTOR_MODELS[mid]].kd_max for mid in motor_ids)
        if args.kp > min_kp_max:
            problems.append(f"--kp {args.kp}가 선택 모터 허용값 {min_kp_max} 초과")
        if args.kd > min_kd_max:
            problems.append(f"--kd {args.kd}가 선택 모터 허용값 {min_kd_max} 초과")
    return motor_ids, problems


def safe_limits(motor_ids, margin_rad):
    result = {}
    for mid in motor_ids:
        lower, upper = JOINT_LIMITS_RAD[mid]
        inner = (lower + margin_rad, upper - margin_rad)
        if inner[0] >= inner[1]:
            raise ValueError(f"ID {mid}: limit margin이 가동범위보다 큼")
        result[mid] = inner
    return result


def load_fixed_model(path, motor_ids):
    path = path.expanduser().resolve()
    if not path.is_file():
        raise RuntimeError(
            f"fixed-base 모델이 없습니다: {path}\n"
            "먼저 robonex_description에서 `python3 mujoco/build_mjcf.py --fixed-base`를 실행하세요."
        )
    model = mujoco.MjModel.from_xml_path(str(path))
    if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root") >= 0:
        raise RuntimeError(f"freejoint 'root'가 있는 모델입니다. fixed-base scene을 사용하세요: {path}")

    actuator_ids = {}
    for mid in motor_ids:
        actuator_name = MOTOR_ACTUATORS[mid]
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
        if aid < 0:
            raise RuntimeError(f"MuJoCo actuator 없음: ID {mid} -> {actuator_name}")
        joint_name = actuator_name + "_joint"
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if jid < 0 or int(model.actuator_trnid[aid, 0]) != jid:
            raise RuntimeError(f"MuJoCo actuator-joint 매핑 불일치: {actuator_name} -> {joint_name}")
        actuator_ids[mid] = aid
    return path, model, actuator_ids


def verify_model_limits(model, actuator_ids, motor_ids):
    for mid in motor_ids:
        got = tuple(float(v) for v in model.actuator_ctrlrange[actuator_ids[mid]])
        want = JOINT_LIMITS_RAD[mid]
        if not np.allclose(got, want, atol=5e-6, rtol=0.0):
            raise RuntimeError(
                f"ID {mid} limit 불일치: MuJoCo={got}, Motor-Test={want}. "
                "소스/생성물을 동기화한 뒤 실행하세요."
            )


def open_hardware(motor_ids, interface, host_id):
    buses = {}
    motors = {}
    channels = sorted({channel_for_id(mid) for mid in motor_ids})
    try:
        for channel in channels:
            buses[channel] = can.Bus(channel=channel, interface=interface)
        for mid in motor_ids:
            motors[mid] = Motor(
                buses[channel_for_id(mid)], mid, SPECS[MOTOR_MODELS[mid]], host_id=host_id
            )
    except Exception:
        for bus in buses.values():
            try:
                bus.shutdown()
            except Exception:
                pass
        raise
    hubs = {
        channel: FeedbackHub(
            bus,
            [motor for mid, motor in motors.items() if channel_for_id(mid) == channel],
            host_id,
        )
        for channel, bus in buses.items()
    }
    return buses, motors, hubs


def inspect_zero_positions(motors, tolerance_rad, limits):
    print("\n시작 전 zero-position 확인(mechPos 읽기 전용):")
    blocking_failures = []
    measured = {}
    print(f"  {'ID':>2} {'joint':<18} {'raw/wrap':>18} {'zero까지 이동':>15}  상태")
    for mid in sorted(motors):
        position = motors[mid].read_mech_position(timeout=0.3)
        measured[mid] = position
        if position is None:
            blocking_failures.append(f"ID {mid} 응답 없음")
            print(f"  {mid:>2} {JOINT_MAP[mid]:<18} {'응답 없음':>15} {'--':>15}  BLOCK")
            continue
        if not math.isfinite(position):
            blocking_failures.append(f"ID {mid} mechPos가 NaN/inf")
            print(f"  {mid:>2} {JOINT_MAP[mid]:<18} {'NaN/inf':>15} {'--':>15}  BLOCK")
            continue
        wrapped = wrap_to_pi(position)
        degrees = math.degrees(position)
        wrapped_deg = math.degrees(wrapped)
        lower, upper = limits[mid]
        if not lower <= wrapped <= upper:
            status = "BLOCK"
            blocking_failures.append(
                f"ID {mid} {degrees:+.3f}deg (wrap {wrapped_deg:+.3f}deg)가 "
                f"보행한계 "
                f"{math.degrees(lower):+.3f}..{math.degrees(upper):+.3f}deg 밖"
            )
        elif abs(wrapped) <= tolerance_rad:
            status = "ZERO 근처"
        else:
            status = "zero로 이동 예정"
        print(
            f"  {mid:>2} {JOINT_MAP[mid]:<18} "
            f"{degrees:+7.2f}/{wrapped_deg:+7.2f}deg "
            f"{(-wrapped_deg):+11.3f} deg  {status}"
        )
    return measured, blocking_failures


def confirm_hardware(args, motor_ids, model_path):
    print("\n실물 모터가 실제로 움직입니다.")
    print(f"  model       : {model_path}")
    print(f"  motor IDs   : {motor_ids}")
    print(f"  max speed   : {args.max_speed:.3f} rad/s ({math.degrees(args.max_speed):.2f} deg/s)")
    print(f"  max accel   : {args.max_accel:.3f} rad/s^2")
    print(f"  kp / kd     : {args.kp:g} / {args.kd:g}")
    print("  시작 동작    : 선택 모터를 zero로 천천히 이동한 뒤 트윈 시작")
    print("  전제         : 고정 스탠드 체결, 다리 주변 비움, 물리 E-stop 준비")
    if args.yes:
        return
    if not sys.stdin.isatty():
        raise RuntimeError("실물 확인 입력이 필요합니다. 대화형 터미널 또는 --yes를 사용하세요.")
    answer = input("zero 이동 및 트윈을 시작하려면 Enter, 취소하려면 Ctrl-C: ")
    if answer.strip():
        raise RuntimeError("빈 Enter가 아니어서 사용자 취소")


def enable_with_runtime_feedback(motors, hubs, kp, kd, limits):
    starts = {}
    enabled_ids = []

    stop_errors = []
    for mid, motor in motors.items():
        try:
            motor.stop()
        except (OSError, can.CanError) as error:
            stop_errors.append(f"ID {mid}: {error}")
    if stop_errors:
        raise RuntimeError("사전 stop 실패: " + "; ".join(stop_errors))
    time.sleep(0.05)

    for mid in sorted(motors):
        motor = motors[mid]
        motor.write_run_mode_operation()
        time.sleep(0.005)
        motor.enable()
        enabled_ids.append(mid)
        hub = hubs[channel_for_id(mid)]
        start = hub.wait_for(mid, timeout=0.3)
        if start is None:
            error = RuntimeError(f"ID {mid} enable 후 type-0x02 실시간 피드백 없음")
            error.enabled_ids = enabled_ids
            raise error
        lower, upper = limits[mid]
        wrapped = wrap_to_pi(start) if math.isfinite(start) else start
        if not math.isfinite(start) or not lower <= wrapped <= upper:
            error = RuntimeError(
                f"ID {mid} enable 후 실시간 위치가 비유한값 또는 안전범위 밖: "
                f"{math.degrees(start):+.3f}deg (wrap {math.degrees(wrapped):+.3f}deg)"
            )
            error.enabled_ids = enabled_ids
            raise error
        starts[mid] = start
        motor.control(pos=start, vel=0.0, kp=kp, kd=kd, torque=0.0)
    return starts, enabled_ids


def move_to_zero(motors, hubs, starts, limits, args):
    tolerance_rad = math.radians(args.zero_tolerance_deg)
    period = 1.0 / args.rate
    commands = dict(starts)
    zero_targets = {mid: align_angle(starts[mid], 0.0) for mid in starts}
    limiters = {mid: AxisLimiter(starts[mid]) for mid in starts}
    next_tick = time.monotonic()
    last_tick = next_tick
    last_print = 0.0
    commanded_zero_at = None

    print("\nzero-position으로 제한속도 이동을 시작합니다.")
    while True:
        now = time.monotonic()

        for hub in hubs.values():
            hub.pump()
        reason = runtime_safety_reason(motors, commands, limits, now, args)
        if reason:
            raise RuntimeError("zero 이동 안전 중단: " + reason)

        dt = clamp(now - last_tick, period * 0.25, period * 2.0)
        last_tick = now
        velocities = {}
        for mid in sorted(motors):
            position, velocity = limiters[mid].step(
                zero_targets[mid], dt, args.max_speed, args.max_accel
            )
            commands[mid] = position
            velocities[mid] = velocity

        for mid, motor in motors.items():
            motor.control(
                pos=commands[mid], vel=velocities[mid],
                kp=args.kp, kd=args.kd, torque=0.0,
            )

        if now - last_print >= 1.0:
            last_print = now
            print(f"[{time.strftime('%H:%M:%S')}] zero-position 이동 중")
            print(f"  {'ID':>2} {'joint':<18} {'limited cmd':>12} {'actual':>11}")
            for mid in sorted(motors):
                actual = motors[mid].last_position
                actual_text = (
                    "--" if actual is None
                    else f"{math.degrees(wrap_to_pi(actual)):+8.2f}deg"
                )
                print(
                    f"  {mid:>2} {JOINT_MAP[mid]:<18} "
                    f"{math.degrees(wrap_to_pi(commands[mid])):+9.2f}deg {actual_text:>11}"
                )

        command_done = all(
            abs(commands[mid] - zero_targets[mid]) <= 1e-12
            and abs(velocities[mid]) <= 1e-12
            for mid in motors
        )
        actual_done = all(
            motors[mid].last_position is not None
            and abs(wrap_to_pi(motors[mid].last_position)) <= tolerance_rad
            for mid in motors
        )
        if command_done and actual_done:
            print(
                "zero-position 도달 확인 완료 "
                f"(실제 위치 ±{args.zero_tolerance_deg:g}deg 이내)."
            )
            return dict(commands)
        if command_done:
            if commanded_zero_at is None:
                commanded_zero_at = now
            elif now - commanded_zero_at > ZERO_SETTLE_TIMEOUT:
                outside = [
                    f"ID {mid} {math.degrees(motors[mid].last_position):+.2f}deg "
                    f"(wrap {math.degrees(wrap_to_pi(motors[mid].last_position)):+.2f}deg)"
                    for mid in sorted(motors)
                    if motors[mid].last_position is not None
                    and abs(wrap_to_pi(motors[mid].last_position)) > tolerance_rad
                ]
                raise RuntimeError(
                    f"zero 명령 후 {ZERO_SETTLE_TIMEOUT:.1f}s 내 도달하지 못함: "
                    + ", ".join(outside)
                )
        else:
            commanded_zero_at = None

        next_tick += period
        sleep = next_tick - time.monotonic()
        if sleep > 0.0:
            time.sleep(sleep)
        elif time.monotonic() - next_tick > period:
            next_tick = time.monotonic()


def runtime_safety_reason(motors, commands, limits, now, args):
    for mid, motor in motors.items():
        if motor.last_feedback_time <= 0.0 or motor.last_position is None:
            return f"ID {mid} type-0x02 피드백을 한 번도 받지 못함"
        age = now - motor.last_feedback_time
        if age > args.feedback_timeout:
            return f"ID {mid} 피드백 끊김({age:.3f}s > {args.feedback_timeout:.3f}s)"
        if not all(math.isfinite(v) for v in (
            motor.last_position, motor.last_velocity, motor.last_torque, motor.last_temp
        )):
            return f"ID {mid} 피드백에 NaN/inf"
        if abs(motor.last_velocity) > args.overspeed:
            return f"ID {mid} 과속({motor.last_velocity:+.3f}rad/s)"
        if motor.last_temp > args.max_temp:
            return f"ID {mid} 과온({motor.last_temp:.1f}degC)"
        lower, upper = limits[mid]
        wrapped = wrap_to_pi(motor.last_position)
        if not lower <= wrapped <= upper:
            return (
                f"ID {mid} 안전 관절범위 이탈("
                f"{math.degrees(motor.last_position):+.2f}deg "
                f"wrap {math.degrees(wrapped):+.2f}deg, "
                f"허용 {math.degrees(lower):+.2f}..{math.degrees(upper):+.2f}deg)"
            )
        if mid in commands:
            error = wrap_to_pi(commands[mid] - motor.last_position)
            if abs(error) > math.radians(args.max_error_deg):
                return f"ID {mid} 추종오차 과다({math.degrees(error):+.2f}deg)"
    return None


def brake_and_stop(motors, buses, enabled_ids, stop_ids, duration, kd):
    enabled = [mid for mid in sorted(set(enabled_ids)) if mid in motors]
    if enabled and duration > 0.0:
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            for mid in enabled:
                try:
                    motor = motors[mid]
                    motor.control(
                        pos=0.0, vel=0.0, kp=0.0,
                        kd=min(kd, motor.spec.kd_max), torque=0.0,
                    )
                except (OSError, can.CanError):
                    pass
            time.sleep(0.01)

    for mid in sorted(set(stop_ids)):
        motor = motors.get(mid)
        if motor is None:
            continue
        try:
            motor.stop()
        except (OSError, can.CanError):
            pass
    for bus in buses.values():
        try:
            bus.shutdown()
        except Exception:
            pass


def print_status(motors, commands, targets, hardware):
    print(f"[{time.strftime('%H:%M:%S')}] " + ("hardware following" if hardware else "dry-run"))
    print(f"  {'ID':>2} {'joint':<18} {'sim target':>11} {'limited cmd':>12} {'actual':>11}")
    for mid in sorted(commands):
        actual = motors[mid].last_position if hardware else None
        actual_text = (
            "--" if actual is None
            else f"{math.degrees(wrap_to_pi(actual)):+8.2f}deg"
        )
        print(
            f"  {mid:>2} {JOINT_MAP[mid]:<18} "
            f"{math.degrees(targets[mid]):+8.2f}deg "
            f"{math.degrees(wrap_to_pi(commands[mid])):+9.2f}deg {actual_text:>11}"
        )


def run(args):
    motor_ids, problems = validate_args(args)
    if problems:
        raise RuntimeError("인자 오류:\n  " + "\n  ".join(problems))

    margin_rad = math.radians(args.limit_margin_deg)
    hard_limits = {mid: JOINT_LIMITS_RAD[mid] for mid in motor_ids}
    command_limits = safe_limits(motor_ids, margin_rad)
    model_path, model, actuator_ids = load_fixed_model(args.model, motor_ids)
    verify_model_limits(model, actuator_ids, motor_ids)
    data = mujoco.MjData(model)
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)

    buses = {}
    motors = {}
    hubs = {}
    enabled_ids = []
    stop_ids = []
    commands = {mid: 0.0 for mid in motor_ids}
    limiters = {mid: AxisLimiter(0.0) for mid in motor_ids}

    try:
        if args.hardware:
            buses, motors, hubs = open_hardware(motor_ids, args.interface, args.host_id)
            _, zero_failures = inspect_zero_positions(
                motors, math.radians(args.zero_tolerance_deg), hard_limits
            )
            if zero_failures:
                raise RuntimeError(
                    "zero-position 이동 전 안전검사 실패 — enable하지 않습니다:\n  "
                    + "\n  ".join(zero_failures)
                )
            confirm_hardware(args, motor_ids, model_path)
            stop_ids = list(motor_ids)
            try:
                starts, enabled_ids = enable_with_runtime_feedback(
                    motors, hubs, args.kp, args.kd, hard_limits,
                )
            except RuntimeError as error:
                enabled_ids = getattr(error, "enabled_ids", enabled_ids)
                raise
            commands.update(starts)
            commands.update(move_to_zero(motors, hubs, starts, hard_limits, args))
            limiters = {mid: AxisLimiter(commands[mid]) for mid in motor_ids}
        else:
            print("시뮬레이션 전용 dry-run입니다. CAN 버스를 열거나 모터 명령을 보내지 않습니다.")
            print("실물 추종은 물리 안전 준비 후 --hardware를 명시해야 활성화됩니다.")

        viewer_context = (
            contextlib.nullcontext(None)
            if args.headless else mujoco.viewer.launch_passive(model, data)
        )
        with viewer_context as viewer:
            if args.hardware:
                print("\n실물 추종 활성화. MuJoCo viewer의 Control 슬라이더만 조작하세요.")

            period = 1.0 / args.rate
            sim_steps = max(1, int(round(period / model.opt.timestep)))
            next_tick = time.monotonic()
            last_tick = next_tick
            start_time = next_tick
            last_print = 0.0

            while viewer is None or viewer.is_running():
                now = time.monotonic()
                if args.duration is not None and now - start_time >= args.duration:
                    break

                if args.hardware:
                    for hub in hubs.values():
                        hub.pump()
                    reason = runtime_safety_reason(motors, commands, hard_limits, now, args)
                    if reason:
                        raise RuntimeError("안전 중단: " + reason)

                lock = viewer.lock() if viewer is not None else contextlib.nullcontext()
                with lock:
                    if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
                        raise RuntimeError("MuJoCo 상태에 NaN/inf — 실물 명령 중단")
                    targets = {}
                    for mid in motor_ids:
                        raw_target = float(data.ctrl[actuator_ids[mid]])
                        if not math.isfinite(raw_target):
                            raise RuntimeError(f"ID {mid} MuJoCo target이 NaN/inf")
                        lower, upper = command_limits[mid]
                        targets[mid] = clamp(raw_target, lower, upper)

                dt = clamp(now - last_tick, period * 0.25, period * 2.0)
                last_tick = now
                command_velocities = {}
                for mid in motor_ids:
                    position, velocity = limiters[mid].step(
                        align_angle(limiters[mid].position, targets[mid]),
                        dt, args.max_speed, args.max_accel,
                    )
                    commands[mid] = position
                    command_velocities[mid] = velocity

                if args.hardware:
                    for mid, motor in motors.items():
                        motor.control(
                            pos=commands[mid], vel=command_velocities[mid],
                            kp=args.kp, kd=args.kd, torque=0.0,
                        )

                lock = viewer.lock() if viewer is not None else contextlib.nullcontext()
                with lock:
                    mujoco.mj_step(model, data, nstep=sim_steps)
                if viewer is not None:
                    viewer.sync()

                if now - last_print >= 1.0:
                    last_print = now
                    print_status(motors, commands, targets, args.hardware)

                next_tick += period
                sleep = next_tick - time.monotonic()
                if sleep > 0.0:
                    time.sleep(sleep)
                elif time.monotonic() - next_tick > period:
                    next_tick = time.monotonic()
    finally:
        if args.hardware:
            brake_and_stop(
                motors, buses, enabled_ids, stop_ids, args.brake_time, args.kd
            )
            if stop_ids:
                print("능동감쇠와 stop/disable 종료 절차를 마쳤습니다.")
            else:
                print("CAN 버스를 닫았습니다. zero/확인 단계에서 모터 쓰기 명령은 보내지 않았습니다.")


def main(argv=None):
    args = parse_args(argv)
    try:
        run(args)
    except KeyboardInterrupt:
        print("\n종료 요청됨.")
        return 0
    except (RuntimeError, ValueError, OSError, can.CanError) as error:
        print(f"\n중단: {error}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
