from __future__ import annotations

import argparse
import contextlib
import math
from pathlib import Path
import sys
import time

import can
import mujoco
import mujoco.viewer
import numpy as np

THIS_FILE = Path(__file__).resolve()
sys.path.insert(0, str(THIS_FILE.parent))

from mujoco_hardware_twin import (
    DEFAULT_INTERFACE,
    DEFAULT_MODEL_PATH,
    HOST_ID,
    JOINT_LIMITS_RAD,
    JOINT_MAP,
    MOTOR_ACTUATORS,
    MOTOR_MODELS,
    clamp,
    load_fixed_model,
    open_hardware,
    verify_model_limits,
)

MAX_RATE = 100.0


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            "손으로 움직인 RoboNex 모터의 mechPos를 fixed-base MuJoCo 모델에 반영. "
            "선택 모터에는 시작과 종료 시 stop만 보내며 enable이나 구동 명령은 보내지 않음."
        )
    )
    parser.add_argument(
        "--hardware",
        action="store_true",
        help="실물 CAN 읽기 활성화. 안전상 명시적으로 지정해야 함",
    )
    parser.add_argument(
        "--motor-id",
        "--motor-ids",
        dest="motor_id",
        nargs="+",
        type=lambda value: int(value, 0),
        default=list(range(1, 13)),
        help="반영할 모터 ID 한 개 이상. 기본: 1~12",
    )
    parser.add_argument(
        "--model",
        type=Path,
        default=DEFAULT_MODEL_PATH,
        help=f"fixed-base MJCF scene, 기본: {DEFAULT_MODEL_PATH}",
    )
    parser.add_argument(
        "--interface",
        default=DEFAULT_INTERFACE,
        help="python-can 인터페이스, 기본: socketcan",
    )
    parser.add_argument("--host-id", type=lambda value: int(value, 0), default=HOST_ID)
    parser.add_argument(
        "--rate",
        type=float,
        default=30.0,
        help="모터별 mechPos 갱신 주기 Hz, 기본: 30",
    )
    parser.add_argument(
        "--read-timeout",
        type=float,
        default=0.03,
        help="개별 mechPos 응답 대기시간 초, 기본: 0.03",
    )
    parser.add_argument(
        "--startup-timeout",
        type=float,
        default=2.0,
        help="최초 위치 수집 제한시간 초, 기본: 2.0",
    )
    parser.add_argument(
        "--stale-timeout",
        type=float,
        default=0.5,
        help="마지막 정상 위치 이후 중단시간 초, 기본: 0.5",
    )
    parser.add_argument(
        "--limit-tolerance-deg",
        type=float,
        default=1.0,
        help="모델 범위 밖 측정값에 허용할 엔코더 오차, 기본: 1deg",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="시작 확인 입력 생략",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="뷰어 없이 실행. --duration 필요",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="지정 초 후 자동 종료",
    )
    return parser.parse_args(argv)


def validate_args(args):
    problems = []
    motor_ids = sorted(set(args.motor_id))
    unknown = [motor_id for motor_id in motor_ids if motor_id not in MOTOR_MODELS]
    if unknown:
        problems.append(f"지원하지 않는 motor-id: {unknown}")
    if not args.hardware:
        problems.append("실물 위치 읽기를 승인하려면 --hardware를 지정해야 함")
    for name in (
        "rate",
        "read_timeout",
        "startup_timeout",
        "stale_timeout",
        "limit_tolerance_deg",
    ):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0.0:
            problems.append(f"--{name.replace('_', '-')}: 0보다 큰 유한값이어야 함")
    if args.rate > MAX_RATE:
        problems.append(f"--rate는 {MAX_RATE:g}Hz 이하여야 함")
    if args.read_timeout >= args.stale_timeout:
        problems.append("--read-timeout은 --stale-timeout보다 작아야 함")
    if args.duration is not None and (
        not math.isfinite(args.duration) or args.duration <= 0.0
    ):
        problems.append("--duration은 0보다 큰 유한값이어야 함")
    if args.headless and args.duration is None:
        problems.append("--headless에는 --duration이 필요함")
    return motor_ids, problems


def confirm_hardware(args, motor_ids, model_path):
    print("\n실물 모터는 구동하지 않고 손으로 움직인 위치만 읽습니다.")
    print(f"  model       : {model_path}")
    print(f"  motor IDs   : {motor_ids}")
    print(f"  update rate : {args.rate:g} Hz per motor")
    print("  CAN write   : 시작·종료 시 stop만 전송")
    print("  CAN read    : mechPos(0x7019) 반복 조회")
    print("  미전송       : enable, run-mode write, type-0x01 control")
    print("  전제         : 로봇 고정, 관절 주변 비움, 손 끼임 주의")
    if args.yes:
        return
    if not sys.stdin.isatty():
        raise RuntimeError("시작 확인 입력이 필요합니다. 대화형 터미널 또는 --yes를 사용하세요.")
    answer = input("stop 후 수동 위치 반영을 시작하려면 Enter, 취소하려면 Ctrl-C: ")
    if answer.strip():
        raise RuntimeError("빈 Enter가 아니어서 사용자 취소")


def stop_motors(motors, required):
    failures = []
    for motor_id in sorted(motors):
        try:
            motors[motor_id].stop()
        except (OSError, can.CanError) as error:
            failures.append(f"ID {motor_id}: {error}")
    if required and failures:
        raise RuntimeError("stop 전송 실패: " + "; ".join(failures))


def collect_initial_positions(motors, timeout, read_timeout):
    deadline = time.monotonic() + timeout
    positions = {}
    while len(positions) < len(motors) and time.monotonic() < deadline:
        for motor_id in sorted(motors):
            if motor_id in positions:
                continue
            value = motors[motor_id].read_mech_position(timeout=read_timeout)
            if value is not None:
                positions[motor_id] = value
    missing = sorted(set(motors) - set(positions))
    if missing:
        raise RuntimeError(f"최초 mechPos 응답 없음: {missing}")
    return positions


def validate_positions(positions, tolerance_rad):
    targets = {}
    clamped = set()
    for motor_id, value in positions.items():
        if not math.isfinite(value):
            raise RuntimeError(f"ID {motor_id} mechPos가 NaN/inf")
        lower, upper = JOINT_LIMITS_RAD[motor_id]
        if value < lower - tolerance_rad or value > upper + tolerance_rad:
            raise RuntimeError(
                f"ID {motor_id} mechPos {math.degrees(value):+.2f}deg가 모델 범위 "
                f"{math.degrees(lower):+.2f}..{math.degrees(upper):+.2f}deg 밖"
            )
        target = clamp(value, lower, upper)
        if target != value:
            clamped.add(motor_id)
        targets[motor_id] = target
    return targets, clamped


def poll_positions(motors, positions, last_seen, read_timeout, stale_timeout):
    for motor_id in sorted(motors):
        value = motors[motor_id].read_mech_position(timeout=read_timeout)
        now = time.monotonic()
        if value is not None:
            positions[motor_id] = value
            last_seen[motor_id] = now
    now = time.monotonic()
    stale = [
        f"ID {motor_id} {now - last_seen[motor_id]:.3f}s"
        for motor_id in sorted(motors)
        if now - last_seen[motor_id] > stale_timeout
    ]
    if stale:
        raise RuntimeError("mechPos 피드백 끊김: " + ", ".join(stale))


def initialize_simulation(model, data, actuator_ids, targets):
    qpos_addresses = {}
    dof_addresses = {}
    for motor_id, actuator_id in actuator_ids.items():
        joint_id = int(model.actuator_trnid[actuator_id, 0])
        qpos_address = int(model.jnt_qposadr[joint_id])
        dof_address = int(model.jnt_dofadr[joint_id])
        qpos_addresses[motor_id] = qpos_address
        dof_addresses[motor_id] = dof_address
        data.ctrl[actuator_id] = targets[motor_id]
        data.qpos[qpos_address] = targets[motor_id]
        data.qvel[dof_address] = 0.0
    mujoco.mj_forward(model, data)
    warmup_steps = max(1, int(round(0.2 / model.opt.timestep)))
    mujoco.mj_step(model, data, nstep=warmup_steps)
    return qpos_addresses


def print_status(data, qpos_addresses, positions, targets, clamped):
    print(f"[{time.strftime('%H:%M:%S')}] real -> MuJoCo")
    print(f"  {'ID':>2} {'joint':<18} {'real':>11} {'sim target':>11} {'sim qpos':>11} {'state':>7}")
    for motor_id in sorted(targets):
        state = "CLAMP" if motor_id in clamped else "OK"
        print(
            f"  {motor_id:>2} {JOINT_MAP[motor_id]:<18} "
            f"{math.degrees(positions[motor_id]):+8.2f}deg "
            f"{math.degrees(targets[motor_id]):+8.2f}deg "
            f"{math.degrees(data.qpos[qpos_addresses[motor_id]]):+8.2f}deg "
            f"{state:>7}"
        )


def run(args):
    motor_ids, problems = validate_args(args)
    if problems:
        raise RuntimeError("인자 오류:\n  " + "\n  ".join(problems))
    model_path, model, actuator_ids = load_fixed_model(args.model, motor_ids)
    verify_model_limits(model, actuator_ids, motor_ids)
    model.opt.gravity[:] = 0.0
    confirm_hardware(args, motor_ids, model_path)

    buses = {}
    motors = {}
    positions = {}
    try:
        buses, motors, _ = open_hardware(
            motor_ids, args.interface, args.host_id
        )
        stop_motors(motors, required=True)
        time.sleep(0.05)
        positions = collect_initial_positions(
            motors, args.startup_timeout, args.read_timeout
        )
        targets, clamped = validate_positions(
            positions, math.radians(args.limit_tolerance_deg)
        )
        last_seen = {motor_id: time.monotonic() for motor_id in motor_ids}

        data = mujoco.MjData(model)
        data.ctrl[:] = 0.0
        qpos_addresses = initialize_simulation(
            model, data, actuator_ids, targets
        )
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
            raise RuntimeError("초기 MuJoCo 상태에 NaN/inf")

        viewer_context = (
            contextlib.nullcontext(None)
            if args.headless
            else mujoco.viewer.launch_passive(model, data)
        )
        period = 1.0 / args.rate
        sim_steps = max(1, int(round(period / model.opt.timestep)))
        start_time = time.monotonic()
        next_tick = start_time
        last_print = 0.0

        print("\n수동 위치 반영을 시작합니다. 종료하려면 viewer를 닫거나 Ctrl-C를 누르세요.")
        with viewer_context as viewer:
            while viewer is None or viewer.is_running():
                now = time.monotonic()
                if args.duration is not None and now - start_time >= args.duration:
                    break
                poll_positions(
                    motors,
                    positions,
                    last_seen,
                    args.read_timeout,
                    args.stale_timeout,
                )
                targets, clamped = validate_positions(
                    positions, math.radians(args.limit_tolerance_deg)
                )
                lock = viewer.lock() if viewer is not None else contextlib.nullcontext()
                with lock:
                    for motor_id, target in targets.items():
                        data.ctrl[actuator_ids[motor_id]] = target
                    mujoco.mj_step(model, data, nstep=sim_steps)
                    if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
                        raise RuntimeError("MuJoCo 상태에 NaN/inf")
                if viewer is not None:
                    viewer.sync()
                if now - last_print >= 1.0:
                    last_print = now
                    print_status(data, qpos_addresses, positions, targets, clamped)
                next_tick += period
                sleep = next_tick - time.monotonic()
                if sleep > 0.0:
                    time.sleep(sleep)
                elif time.monotonic() - next_tick > period:
                    next_tick = time.monotonic()
    finally:
        if motors:
            stop_motors(motors, required=False)
        for bus in buses.values():
            try:
                bus.shutdown()
            except Exception:
                pass
    print("stop 상태 유지 및 CAN 종료를 완료했습니다.")


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
