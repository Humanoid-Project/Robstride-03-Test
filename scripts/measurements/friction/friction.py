#!/usr/bin/env python3





import argparse
import math
import os
import subprocess
import sys
import time
from datetime import datetime

import can

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common import (
    HOST_ID, DEFAULT_INTERFACE, SPECS, RATED_TORQUE, PEAK_TORQUE,
    RUN_MODE_INDEX, RUN_MODE_OPERATION, MECH_POS_INDEX, FAULT_STA_INDEX,
    Motor, channel_for_id, decode_fault_bits, active_brake,
    validate_args, report_invalid_args,
    JOINT_LIMITS_RAD, DEFAULT_LIMIT_MARGIN_RAD, joint_limit_for, exceeds_joint_limit,
)

ARG_CHECKS = [
    ("start-torque", "torque"), ("step", "positive"), ("max-torque", "torque"),
    ("probe-time", "positive"), ("move-threshold", "positive"),
    ("max-vel", "speed"), ("max-time", "positive"),
    ("settle-pos-tol", "positive"), ("settle-time", "positive"),
    ("settle-timeout", "positive"), ("settle-max-vel", "speed"),
    ("feedback-timeout", "positive"),
]

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
ANALYZE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "analyze_friction.py")
PAUSE_S = 1.0


def positive_int(value):
    value = int(value)
    if value < 1:
        raise argparse.ArgumentTypeError("1 이상의 정수여야 합니다")
    return value


def parse_args():
    p = argparse.ArgumentParser(description="friction 측정 및 분석")
    p.add_argument("--motor-id", type=lambda v: int(v, 0), required=True)
    p.add_argument("--model", choices=list(SPECS.keys()), required=True)
    p.add_argument("--channel", default=None, help="기본값: motor-id로 자동 판단 (can0/can1)")
    p.add_argument("--signs", type=int, nargs="+", choices=[1, -1], default=[1, -1],
                   help="측정 방향, 기본: 1 -1")
    p.add_argument("--repeats", type=positive_int, default=3, help="방향별 반복 횟수, 기본 3")
    p.set_defaults(
        interface=DEFAULT_INTERFACE, host_id=HOST_ID,
        start_torque=0.02, step=0.02, probe_time=0.3,
        move_threshold=0.03, max_torque=1.0, max_vel=3.0, max_time=30.0,
        settle_pos_tol=0.01, settle_time=0.3, settle_timeout=5.0,
        settle_max_vel=3.0, feedback_timeout=0.3, out=None,
        limit_margin=DEFAULT_LIMIT_MARGIN_RAD, ignore_joint_limit=False,
    )
    return p.parse_args()


def confirm(args, model):
    rated = RATED_TORQUE[model]
    n_steps_est = int(math.ceil((args.max_torque - args.start_torque) / args.step)) + 1
    print("=" * 70)
    print("friction 측정 및 분석")
    print(f"  모터 ID       : {args.motor_id} ({model.upper()})")
    print(f"  채널          : {args.channel}")
    print(f"  방향          : {args.signs} x {args.repeats}회")
    print(f"  탐색 설정     : 시작 {args.start_torque} N*m, 단계 {args.step} N*m, "
          f"최대 {args.max_torque} N*m, 단계당 {args.probe_time}s "
          f"(1회 최악 {n_steps_est*args.probe_time:.1f}s)")
    print(f"  움직임 판정   : |dpos|>={args.move_threshold} rad ({math.degrees(args.move_threshold):.2f} deg)")
    if args.ignore_joint_limit:
        print("  ⚠⚠ --ignore-joint-limit 켜짐 — 관절한계 체크 없이 실행합니다.")
    else:
        lo, hi = JOINT_LIMITS_RAD.get(args.motor_id, (None, None))
        if lo is None:
            print(f"  오류: motor-id {args.motor_id}에 대한 JOINT_LIMITS_RAD 항목이 없습니다 — "
                  "common.py에 추가하거나 --ignore-joint-limit(무부하일 때만)로 실행하세요.")
        else:
            print(f"  관절한계(URDF): {math.degrees(lo):+.1f} ~ {math.degrees(hi):+.1f} deg "
                  f"(여유 {math.degrees(args.limit_margin):.1f}deg 적용해 자동 정지 — "
                  "움직임 판정이 보통 더 먼저 걸리지만 이중 안전장치)")
    print("  전제: zero_position 캘리브레이션 완료(모터 zero == URDF zero), 링크 연결된 실물 상태")
    print("=" * 70)
    if args.max_torque > rated:
        print(f"경고: max-torque가 정격({rated} N*m)을 넘습니다. 무부하 시험엔 보통 불필요하게 큽니다.")
    if args.max_torque > PEAK_TORQUE[model]:
        print(f"오류: max-torque가 피크 토크({PEAK_TORQUE[model]} N*m)를 초과합니다. 중단합니다.")
        return False
    if not sys.stdin.isatty():
        print("확인 입력이 필요합니다. 대화형 터미널에서 실행하세요.")
        return False
    try:
        input("\n시작하려면 Enter, 취소하려면 Ctrl-C: ")
    except (KeyboardInterrupt, EOFError):
        print("\n취소됨.")
        return False
    return True


def wait_settled(motor, hold_pos, args):

    deadline = time.monotonic() + args.settle_timeout
    ref_pos = None
    ref_time = time.monotonic()
    last_pos = hold_pos
    misses = 0
    last_print = 0.0
    while time.monotonic() < deadline:
        motor.control(pos=0.0, vel=0.0, kp=0.0, kd=2.0, torque=0.0)
        fb = motor.poll_feedback(timeout=0.05)
        now = time.monotonic()
        if fb is None:
            misses += 1
            status = f"응답 없음 (miss {misses})"
        else:
            _, pos, vel, tq, temp, fault = fb
            last_pos = pos
            if abs(vel) >= args.settle_max_vel:
                print()
                print(f"  긴급정지: 정지판정 중 |vel|={vel:+.3f} rad/s 감지(한계 {args.settle_max_vel}).")
                motor.stop()
                return None
            if ref_pos is None or abs(pos - ref_pos) > args.settle_pos_tol:
                ref_pos = pos
                ref_time = now
            elif now - ref_time >= args.settle_time:
                print()
                return last_pos
            status = f"pos={pos:+.4f} rad  quiet={now - ref_time:4.2f}/{args.settle_time:.2f}s"
        if now - last_print >= 0.1:
            print(f"\r  {status}    ", end="", flush=True)
            last_print = now
    print()
    return None


def return_to_zero(motor, timeout=2.5, kp=15.0, kd=3.0, pos_tol=0.02):


    deadline = time.monotonic() + timeout
    last_pos = 0.0
    while time.monotonic() < deadline:
        motor.control(pos=0.0, vel=0.0, kp=kp, kd=kd, torque=0.0)
        fb = motor.poll_feedback(timeout=0.05)
        if fb is None:
            continue
        _, pos, vel, tq, temp, fault = fb
        last_pos = pos
        if abs(pos) <= pos_tol and abs(vel) < 0.05:
            return last_pos
    print(f"  경고: {timeout}s 안에 0으로 복귀 못 함 (마지막 위치 {last_pos:+.4f} rad) — "
          "다음 측정 전 read_joint_values.py로 확인 권장.")
    return last_pos


def run_search(motor, pos_start, args):
    rows = []
    misses = 0
    t0 = time.monotonic()
    current_torque = args.sign * args.start_torque
    step = args.sign * args.step
    max_torque_signed = args.sign * args.max_torque
    step_deadline = t0 + args.probe_time
    stop_reason = "max_time"
    last_print = 0.0
    step_start_index = 0
    mismatch_streak = 0
    steps_taken = 0
    last_ok = t0
    try:
        while True:
            now = time.monotonic()
            elapsed = now - t0
            if elapsed >= args.max_time:
                stop_reason = "max_time"
                break
            if now >= step_deadline:
                finished_rows = rows[step_start_index:]
                if finished_rows:
                    mean_tq = sum(r[4] for r in finished_rows) / len(finished_rows)
                    if abs(mean_tq - current_torque) > max(0.08, 0.5 * abs(current_torque)):
                        mismatch_streak += 1
                    else:
                        mismatch_streak = 0
                    if mismatch_streak >= 2:
                        stop_reason = "comm_mismatch"
                        break
                step_start_index = len(rows)
                current_torque += step
                steps_taken += 1
                step_deadline = now + args.probe_time
                print()
                print(f"  -> 단계 증가: {current_torque:+.3f} N*m")
                if abs(current_torque) > abs(max_torque_signed):
                    stop_reason = "max_torque_no_movement"
                    break
            motor.control(pos=0.0, vel=0.0, kp=0.0, kd=0.0, torque=current_torque)
            fb = motor.poll_feedback(timeout=0.05)
            if fb is None:
                misses += 1
                if time.monotonic() - last_ok > args.feedback_timeout:
                    stop_reason = "feedback_lost"
                    break
                continue
            last_ok = time.monotonic()
            t_recv, pos, vel, tq, temp, fault = fb
            rows.append((t_recv - t0, pos - pos_start, vel, current_torque, tq, temp, fault))
            if now - last_print >= 0.1:
                print(f"\r  torque={current_torque:+.3f} N*m  pos={pos - pos_start:+.4f} rad  "
                      f"vel={vel:+.3f} rad/s(참고용)  tq={tq:+.3f} N*m    ", end="", flush=True)
                last_print = now
            if abs(vel) >= args.max_vel:
                stop_reason = "max_vel"
                break
            if not args.ignore_joint_limit and exceeds_joint_limit(pos, args.motor_id, args.limit_margin):
                print(f"\n  관절한계 도달: pos={pos:+.4f} rad ({math.degrees(pos):+.1f} deg) — "
                      "즉시 정지합니다.")
                stop_reason = "joint_limit"
                break
            if abs(pos - pos_start) >= args.move_threshold:
                if steps_taken == 0:
                    stop_reason = "movement_at_start_suspicious"
                else:
                    stop_reason = "movement_detected"
                break
    except KeyboardInterrupt:
        stop_reason = "keyboard_interrupt"
    print()
    return rows, stop_reason, current_torque


def save_csv(path, rows, meta):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        for key, value in meta.items():
            f.write(f"# {key}: {value}\n")
        f.write("t_s,pos_rel_rad,vel_rad_s,torque_cmd_Nm,torque_meas_Nm,temp_C,fault_hex\n")
        for t_s, pos, vel, tcmd, tq, temp, fault in rows:
            f.write(f"{t_s:.6f},{pos:.6f},{vel:.6f},{tcmd:.4f},{tq:.4f},{temp:.1f},{fault:02X}\n")


def capture_once(args, sign, run_index):
    args.sign = sign
    spec = SPECS[args.model]
    bus = can.Bus(channel=args.channel, interface=args.interface)
    motor = None
    rows, stop_reason, torque_at_stop = [], "not_started", 0.0
    pos_start = None
    try:
        motor = Motor(bus, args.motor_id, spec, host_id=args.host_id)

        motor.stop()
        time.sleep(0.02)
        initial_pos = motor.read_param_f32(MECH_POS_INDEX, timeout=0.3)
        if initial_pos is None:
            print(f"오류: 모터 ID {args.motor_id} 가 {args.channel} 에서 응답하지 않습니다.")
            return 1, None, "no_response"
        print(f"현재 위치: {initial_pos:+.4f} rad ({math.degrees(initial_pos):+.2f} deg)")

        fault_reg = motor.read_param(FAULT_STA_INDEX, fmt="<I", timeout=0.3)
        if fault_reg is None:
            print("폴트 레지스터(0x3022) 읽기 실패 (무시하고 진행)")
        else:
            bits = decode_fault_bits(fault_reg)
            print(f"폴트 레지스터(0x3022) = 0x{fault_reg:08X}" +
                  (f"  -> {', '.join(bits)}" if bits else "  (정상)"))

        motor.write_param_u8(RUN_MODE_INDEX, RUN_MODE_OPERATION)
        time.sleep(0.01)
        motor.enable()
        time.sleep(0.01)

        print("정지 상태 확인 중...")
        pos_start = wait_settled(motor, initial_pos, args)
        if pos_start is None:
            print("오류: 정지 상태가 확인되지 않았습니다 (settle-timeout 초과 또는 긴급정지, "
                  "위 메시지 참고). 모터 상태를 확인하세요.")
            motor.stop()
            return 1, None, "not_settled"
        print(f"정지 확인됨 (기준 위치 {pos_start:+.4f} rad). 계단식 토크 탐색 시작...")

        if not args.ignore_joint_limit and exceeds_joint_limit(pos_start, args.motor_id, args.limit_margin):
            print(f"오류: 정지 위치({pos_start:+.4f} rad = {math.degrees(pos_start):+.1f} deg)가 "
                  "이미 관절한계(margin 포함) 밖입니다. zero_position 정렬이 URDF와 맞는지, "
                  "또는 실제로 하드스톱 근처인지 확인하세요. 중단합니다.")
            motor.stop()
            return 1, None, "initial_joint_limit"

        rows, stop_reason, torque_at_stop = run_search(motor, pos_start, args)
    except KeyboardInterrupt:
        print("\n\n중단됨 (Ctrl-C).")
        stop_reason = "keyboard_interrupt"
    finally:
        if motor is not None:
            try:
                active_brake(motor)
            except can.CanError:
                pass
            try:
                return_to_zero(motor)
            except can.CanError:
                pass
            try:
                motor.stop()
            except can.CanError:
                pass
        bus.shutdown()

    if not rows:
        print(f"샘플이 하나도 기록되지 않았습니다 (stop_reason={stop_reason}). CAN 응답을 확인하세요.")
        return 1, None, stop_reason

    duration = rows[-1][0] - rows[0][0]
    achieved_hz = (len(rows) - 1) / duration if duration > 0 else 0.0

    print(f"\n탐색 종료: {stop_reason}")
    print(f"  샘플 수       : {len(rows)}")
    print(f"  구간          : {duration*1000:.1f} ms")
    print(f"  달성 폴링율   : {achieved_hz:.0f} Hz")

    breakaway_low = breakaway_high = breakaway_mid = None
    if stop_reason == "movement_detected":
        breakaway_high = torque_at_stop
        breakaway_low = torque_at_stop - args.sign * args.step
        breakaway_mid = (breakaway_low + breakaway_high) / 2.0
        print(f"\n  breakaway 브래킷 : [{min(breakaway_low, breakaway_high):.3f}, "
              f"{max(breakaway_low, breakaway_high):.3f}] N*m")
        print(f"  breakaway 추정값 : {breakaway_mid:+.3f} N*m (±{abs(args.step)/2:.3f})")
    elif stop_reason == "movement_at_start_suspicious":
        print(f"\n  경고: 정지 확인 직후, 첫 단계(토크 {torque_at_stop:+.3f} N*m)에서부터 "
              "움직임이 감지됐습니다 — 이 정도로 작은 토크에 실제 breakaway일 가능성은 낮고, "
              "잔류 움직임/글리치로 의심됩니다. 이 결과는 breakaway로 쓰지 말고 재측정하세요.")
    elif stop_reason == "max_torque_no_movement":
        print(f"\n  경고: max-torque({args.max_torque} N*m)까지 전혀 안 움직였습니다. "
              "기계적 결속/과도한 마찰 의심 — 배선·조립 상태 확인 필요.")
    elif stop_reason == "max_vel":
        print(f"\n  경고: 움직임 판정 전에 안전속도한계에 도달했습니다. "
              "--move-threshold를 낮추거나 --probe-time을 줄여 재시도 권장.")
    elif stop_reason == "feedback_lost":
        print(f"\n  오류: 피드백이 {args.feedback_timeout}s 넘게 끊겨 중단했습니다 — 토크를 걸어둔 채 "
              "움직임을 감지 못 하는 상태라 위험합니다. CAN 상태 확인 후(ip link show) 재측정하세요.")
    elif stop_reason == "comm_mismatch":
        print("\n  오류: 명령토크와 실측토크가 여러 단계째 안 맞습니다 — 정지마찰이 큰 게 아니라 "
              "통신 이상(프레임 유실 등)일 가능성이 높습니다. 이 결과는 저장은 되지만 "
              "breakaway로 쓰지 말고, CAN 상태 확인 후(ip link show) 재측정하세요.")
    elif stop_reason == "joint_limit":
        print(f"\n  경고: 관절한계(margin 포함)에 도달해 중단했습니다 (토크 {torque_at_stop:+.3f} N*m까지). "
              "breakaway가 이 토크보다 커서 못 찾았을 수도 있고, 시작 위치가 한계에 너무 "
              "가까웠을 수도 있습니다 — 관절을 중립 자세 쪽으로 옮기고 재측정 권장.")

    ts = datetime.now().strftime("%Y%m%dT%H%M%S_%f")
    sign_tag = "pos" if args.sign > 0 else "neg"
    fname = f"id{args.motor_id}_{args.model}_{sign_tag}_{ts}_r{run_index:02d}.csv"
    out_path = os.path.join(DATA_DIR, fname)

    meta = {
        "motor_id": args.motor_id,
        "model": args.model,
        "channel": args.channel,
        "host_id": f"0x{args.host_id:02X}",
        "sign": args.sign,
        "start_torque_Nm": f"{args.start_torque:.4f}",
        "step_Nm": f"{args.step:.4f}",
        "probe_time_s": f"{args.probe_time:.3f}",
        "move_threshold_rad": f"{args.move_threshold:.4f}",
        "max_torque_Nm": f"{args.max_torque:.4f}",
        "started_at": datetime.now().isoformat(timespec="seconds"),
        "pos_start_rad": f"{pos_start:.6f}" if pos_start is not None else "",
        "stop_reason": stop_reason,
        "breakaway_low_Nm": f"{breakaway_low:.4f}" if breakaway_low is not None else "",
        "breakaway_high_Nm": f"{breakaway_high:.4f}" if breakaway_high is not None else "",
        "breakaway_mid_Nm": f"{breakaway_mid:.4f}" if breakaway_mid is not None else "",
        "samples": len(rows),
        "duration_s": f"{duration:.4f}",
    }
    save_csv(out_path, rows, meta)
    print(f"\n저장됨: {out_path}")
    return (0 if stop_reason == "movement_detected" else 1), out_path, stop_reason


def main():
    args = parse_args()
    if args.channel is None:
        args.channel = channel_for_id(args.motor_id)

    for sign in args.signs:
        args.sign = sign
        if report_invalid_args(validate_args(args, args.model, ARG_CHECKS)):
            return 1
    if args.start_torque > args.max_torque:
        print("내부 설정 오류: start_torque가 max_torque보다 큽니다.")
        return 1
    if not confirm(args, args.model):
        return 1

    plan = [sign for sign in args.signs for _ in range(args.repeats)]
    paths = []
    failed = False
    for index, sign in enumerate(plan, 1):
        print(f"\n{'=' * 70}\n[{index}/{len(plan)}] sign={sign:+d}\n{'=' * 70}")
        rc, path, stop_reason = capture_once(args, sign, index)
        if rc == 0 and path is not None:
            paths.append(path)
        if rc != 0 or stop_reason == "keyboard_interrupt":
            failed = True
            print("안전을 위해 남은 반복 측정을 중단합니다.")
            break
        if index < len(plan):
            time.sleep(PAUSE_S)

    if paths:
        print("\n이번 실행에서 생성된 CSV를 분석합니다.\n")
        analysis = subprocess.run([sys.executable, ANALYZE, *paths])
        failed = failed or analysis.returncode != 0
    return 1 if failed or not paths else 0


if __name__ == "__main__":
    sys.exit(main())
