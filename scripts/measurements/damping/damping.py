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
    HOST_ID, DEFAULT_INTERFACE, SPECS,
    RUN_MODE_INDEX, RUN_MODE_OPERATION, MECH_POS_INDEX, FAULT_STA_INDEX,
    Motor, channel_for_id, decode_fault_bits, active_brake,
    validate_args, report_invalid_args,
    JOINT_LIMITS_RAD, DEFAULT_LIMIT_MARGIN_RAD, joint_limit_for, exceeds_joint_limit,
)

ARG_CHECKS = [
    ("speed", "speed"), ("kd", "positive"), ("hold-time", "positive"),
    ("ramp-time", "positive"), ("max-turns", "positive"),
    ("feedback-timeout", "positive"),
]

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
ANALYZE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "analyze_damping.py")
PAUSE_S = 1.0


def positive_int(value):
    value = int(value)
    if value < 1:
        raise argparse.ArgumentTypeError("1 이상의 정수여야 합니다")
    return value


def parse_args():
    p = argparse.ArgumentParser(description="damping 측정 및 분석")
    p.add_argument("--motor-id", type=lambda v: int(v, 0), required=True)
    p.add_argument("--model", choices=list(SPECS.keys()), required=True)
    p.add_argument("--channel", default=None, help="기본값: motor-id로 자동 판단 (can0/can1)")
    p.add_argument("--speeds", type=float, nargs="+", required=True,
                   help="측정할 목표 속도 목록(rad/s, 부호 있음)")
    p.add_argument("--repeats", type=positive_int, default=1, help="속도별 반복 횟수, 기본 1")
    p.set_defaults(
        interface=DEFAULT_INTERFACE, host_id=HOST_ID,
        kd=2.0, hold_time=1.5, ramp_time=0.5, max_turns=0.5,
        limit_margin=DEFAULT_LIMIT_MARGIN_RAD, ignore_joint_limit=False,
        feedback_timeout=0.3, out=None,
    )
    return p.parse_args()


def confirm(args, model):
    print("=" * 70)
    print("damping 측정 및 분석")
    print(f"  모터 ID       : {args.motor_id} ({model.upper()})")
    print(f"  채널          : {args.channel}")
    print(f"  목표 속도     : {args.speeds} rad/s x {args.repeats}회")
    print(f"  램프 대기     : {args.ramp_time}s (고정, vel 노이즈로 자동판정 안 함)")
    print(f"  기록 시간     : {args.hold_time}s")
    print(f"  안전한계(부차): |dpos|>={args.max_turns} turn")
    if args.ignore_joint_limit:
        print("  ⚠⚠ --ignore-joint-limit 켜짐 — 관절한계 체크 없이 자유회전 전제로 실행합니다.")
        print("     출력축이 실제로 무부하 상태가 아니면 위험합니다.")
    else:
        lo, hi = JOINT_LIMITS_RAD.get(args.motor_id, (None, None))
        if lo is None:
            print(f"  오류: motor-id {args.motor_id}에 대한 JOINT_LIMITS_RAD 항목이 없습니다 — "
                  "common.py에 추가하거나 --ignore-joint-limit(무부하일 때만)로 실행하세요.")
        else:
            lo_m, hi_m = joint_limit_for(args.motor_id, args.limit_margin)
            print(f"  관절한계(URDF): {math.degrees(lo):+.1f} ~ {math.degrees(hi):+.1f} deg"
                  f"  (여유 {math.degrees(args.limit_margin):.1f}deg 적용 후 "
                  f"{math.degrees(lo_m):+.1f} ~ {math.degrees(hi_m):+.1f} deg에서 자동 정지)")
            worst_travel = max(abs(speed) for speed in args.speeds) * (args.ramp_time + args.hold_time)
            usable_range = hi_m - lo_m
            print(f"  최악 이동각   : |speed|x(ramp+hold) = {math.degrees(worst_travel):.1f} deg "
                  f"(가동범위 {math.degrees(usable_range):.1f}deg의 {100*worst_travel/usable_range:.0f}%)")
            if worst_travel > usable_range:
                print("  ⚠ 목표속도로 끝까지 못 갈 가능성이 높습니다 — 관절한계에 먼저 걸려 "
                      "hold_time을 다 못 채우고 조기 중단될 것으로 예상됩니다. 정속구간 데이터가 "
                      "부족하면 --speed를 낮추거나 --hold-time을 줄이세요.")
    print("  전제: zero_position 캘리브레이션 완료(모터 zero == URDF zero), 링크 연결된 실물 상태")
    print("=" * 70)
    if not sys.stdin.isatty():
        print("확인 입력이 필요합니다. 대화형 터미널에서 실행하세요.")
        return False
    try:
        input("\n시작하려면 Enter, 취소하려면 Ctrl-C: ")
    except (KeyboardInterrupt, EOFError):
        print("\n취소됨.")
        return False
    return True


def ramp_to_speed(motor, target_speed, kd, pos_ref, args, label):

    deadline = time.monotonic() + args.ramp_time
    last_pos, last_vel = pos_ref, 0.0
    last_print = 0.0
    last_ok = time.monotonic()
    while time.monotonic() < deadline:
        motor.control(pos=0.0, vel=target_speed, kp=0.0, kd=kd, torque=0.0)
        fb = motor.poll_feedback(timeout=0.05)
        now = time.monotonic()
        if fb is None:
            if now - last_ok > args.feedback_timeout:
                print()
                return False, last_pos, last_vel, "feedback_lost"
            status = f"[{label}] 응답 없음 ({now - last_ok:.2f}s)"
        else:
            last_ok = now
            _, pos, vel, tq, temp, fault = fb
            last_pos, last_vel = pos, vel
            if pos_ref is not None and abs(pos - pos_ref) >= args.max_turns * 2.0 * math.pi:
                print()
                return False, last_pos, last_vel, "max_turns"
            if (not args.ignore_joint_limit and target_speed != 0.0
                    and exceeds_joint_limit(pos, args.motor_id, args.limit_margin)):
                print()
                print(f"  [{label}] 관절한계 도달: pos={pos:+.4f} rad "
                      f"({math.degrees(pos):+.1f} deg) — 즉시 정지합니다.")
                return False, last_pos, last_vel, "joint_limit"
            status = (f"[{label}] pos={pos:+.4f}  vel={vel:+.3f}  target={target_speed:+.3f}  "
                      f"tq={tq:+.3f} N*m")
        if now - last_print >= 0.1:
            print(f"\r  {status}    ", end="", flush=True)
            last_print = now
    print()
    return True, last_pos, last_vel, "ok"


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
            break
    return last_pos


def run_hold(motor, target_speed, kd, pos_ref, args):
    rows = []
    t0 = time.monotonic()
    stop_reason = "hold_time"
    limit = max(2.0, abs(target_speed) * 2.0 + 2.0)
    last_ok = t0
    while True:
        now = time.monotonic()
        if now - t0 >= args.hold_time:
            stop_reason = "hold_time"
            break
        motor.control(pos=0.0, vel=target_speed, kp=0.0, kd=kd, torque=0.0)
        fb = motor.poll_feedback(timeout=0.05)
        if fb is None:
            if time.monotonic() - last_ok > args.feedback_timeout:
                stop_reason = "feedback_lost"
                break
            continue
        last_ok = time.monotonic()
        t_recv, pos, vel, tq, temp, fault = fb
        rows.append((t_recv - t0, pos, vel, tq, temp, fault))
        if abs(pos - pos_ref) >= args.max_turns * 2.0 * math.pi:
            stop_reason = "max_turns"
            break
        if not args.ignore_joint_limit and exceeds_joint_limit(pos, args.motor_id, args.limit_margin):
            print(f"\n  관절한계 도달: pos={pos:+.4f} rad ({math.degrees(pos):+.1f} deg) — "
                  "기록 중단, 즉시 감속합니다.")
            stop_reason = "joint_limit"
            break
        if abs(vel) >= limit:
            stop_reason = "overspeed"
            break
    return rows, stop_reason


def save_csv(path, rows, meta):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        for key, value in meta.items():
            f.write(f"# {key}: {value}\n")
        f.write("t_s,pos_rad,vel_rad_s,torque_meas_Nm,temp_C,fault_hex\n")
        for t_s, pos, vel, tq, temp, fault in rows:
            f.write(f"{t_s:.6f},{pos:.6f},{vel:.6f},{tq:.4f},{temp:.1f},{fault:02X}\n")


def capture_once(args, speed, run_index):
    args.speed = speed
    spec = SPECS[args.model]
    bus = can.Bus(channel=args.channel, interface=args.interface)
    motor = None
    rows, stop_reason = [], "not_started"
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

        pos_ref = None
        for _ in range(5):
            motor.control(pos=0.0, vel=0.0, kp=0.0, kd=0.0, torque=0.0)
            fb0 = motor.poll_feedback(timeout=0.2)
            if fb0 is not None:
                pos_ref = fb0[1]
                break
        if pos_ref is None:
            print("오류: 실시간 피드백(type 0x02)을 받지 못해 안전한계 기준점을 못 잡았습니다. "
                  "CAN 상태를 확인하세요 (ip link show).")
            return 1, None, "no_realtime_feedback"

        if not args.ignore_joint_limit:
            lo, hi = JOINT_LIMITS_RAD.get(args.motor_id, (None, None))
            if lo is None:
                print(f"오류: motor-id {args.motor_id}에 대한 JOINT_LIMITS_RAD 항목이 없습니다.")
                return 1, None, "missing_joint_limit"
            print(f"관절한계(URDF, margin 적용 전): {math.degrees(lo):+.1f} ~ {math.degrees(hi):+.1f} deg")
            if exceeds_joint_limit(pos_ref, args.motor_id, args.limit_margin):
                print(f"오류: 현재 위치({pos_ref:+.4f} rad = {math.degrees(pos_ref):+.1f} deg)가 "
                      "이미 관절한계(margin 포함) 밖입니다. zero_position 정렬이 URDF와 맞는지, "
                      "또는 실제로 하드스톱 근처인지 확인하세요. 중단합니다.")
                motor.stop()
                return 1, None, "initial_joint_limit"
            lo_m, hi_m = joint_limit_for(args.motor_id, args.limit_margin)
            room = (hi_m - pos_ref) if args.speed > 0 else (pos_ref - lo_m)
            print(f"명령 방향으로 남은 여유: {math.degrees(room):.1f} deg "
                  f"(목표속도 {args.speed:+.2f} rad/s 기준)")

        print(f"목표 속도 {args.speed:+.3f} rad/s 로 {args.ramp_time}s 접근...")
        ok, pos_now, vel_now, ramp_reason = ramp_to_speed(
            motor, args.speed, args.kd, pos_ref, args, "ramp-up")
        if not ok:
            print(f"오류: 램프 중 중단됨 (사유={ramp_reason}).")
            return 1, None, ramp_reason
        if args.speed != 0 and (vel_now == 0.0 or vel_now / args.speed < 0):
            print(f"경고: 속도가 목표 방향으로 안 붙는 것 같습니다 (vel={vel_now:+.3f}, "
                  f"target={args.speed:+.3f}). kd를 올리거나 ramp-time을 늘려보세요.")
        print(f"램프 종료 (pos={pos_now:+.4f} rad, vel={vel_now:+.3f} rad/s). 정속 구간 기록 시작...")

        rows, stop_reason = run_hold(motor, args.speed, args.kd, pos_ref, args)

        print("감속 중...")
        ramp_to_speed(motor, 0.0, args.kd, None, args, "ramp-down")
        print("0으로 복귀 중...")
        final_pos = return_to_zero(motor)
        print(f"복귀 완료: pos={final_pos:+.4f} rad ({math.degrees(final_pos):+.2f} deg)")
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
                motor.stop()
            except can.CanError:
                pass
        bus.shutdown()

    if not rows:
        print(f"샘플이 하나도 기록되지 않았습니다 (stop_reason={stop_reason}). CAN 응답을 확인하세요.")
        return 1, None, stop_reason

    duration = rows[-1][0] - rows[0][0]
    mean_vel = sum(r[2] for r in rows) / len(rows)
    mean_tq = sum(r[3] for r in rows) / len(rows)
    achieved_hz = (len(rows) - 1) / duration if duration > 0 else 0.0

    print(f"\n캡처 종료: {stop_reason}")
    print(f"  샘플 수       : {len(rows)}")
    print(f"  구간          : {duration*1000:.1f} ms")
    print(f"  달성 폴링율   : {achieved_hz:.0f} Hz")
    print(f"  평균 vel      : {mean_vel:+.4f} rad/s (명령 {args.speed:+.3f}, kd={args.kd})")
    print(f"  평균 tq       : {mean_tq:+.4f} N*m")
    if len(rows) < 30:
        print("  경고: 샘플이 30개 미만입니다. --hold-time 을 늘려 재시도 권장.")

    ts = datetime.now().strftime("%Y%m%dT%H%M%S_%f")
    fname = (f"id{args.motor_id}_{args.model}_{args.speed:+.3f}rads_{ts}_r{run_index:02d}.csv"
             .replace("+", "p").replace("-", "m"))
    out_path = os.path.join(DATA_DIR, fname)

    meta = {
        "motor_id": args.motor_id,
        "model": args.model,
        "channel": args.channel,
        "host_id": f"0x{args.host_id:02X}",
        "speed_cmd_rad_s": f"{args.speed:.4f}",
        "kd": f"{args.kd:.2f}",
        "started_at": datetime.now().isoformat(timespec="seconds"),
        "stop_reason": stop_reason,
        "samples": len(rows),
        "duration_s": f"{duration:.4f}",
    }
    save_csv(out_path, rows, meta)
    print(f"\n저장됨: {out_path}")
    return (0 if stop_reason == "hold_time" else 1), out_path, stop_reason


def main():
    args = parse_args()
    if args.channel is None:
        args.channel = channel_for_id(args.motor_id)

    for speed in args.speeds:
        args.speed = speed
        if report_invalid_args(validate_args(args, args.model, ARG_CHECKS)):
            return 1
    if args.kd > SPECS[args.model].kd_max:
        print(f"내부 설정 오류: kd({args.kd})가 {args.model.upper()} 상한을 초과합니다.")
        return 1
    if not confirm(args, args.model):
        return 1

    plan = [speed for speed in args.speeds for _ in range(args.repeats)]
    paths = []
    failed = False
    for index, speed in enumerate(plan, 1):
        print(f"\n{'=' * 70}\n[{index}/{len(plan)}] speed={speed:+.3f} rad/s\n{'=' * 70}")
        rc, path, stop_reason = capture_once(args, speed, index)
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
