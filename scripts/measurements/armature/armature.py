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
)

ARG_CHECKS = [
    ("torque", "torque"), ("max-vel", "speed"), ("max-time", "positive"),
    ("max-turns", "positive"), ("settle-pos-tol", "positive"),
    ("settle-time", "positive"), ("settle-timeout", "positive"),
    ("settle-max-vel", "speed"), ("feedback-timeout", "positive"),
    ("poll-timeout", "positive"), ("rate", "nonneg"),
]

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
ANALYZE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "analyze_armature.py")
PAUSE_S = 1.0


def positive_int(value):
    value = int(value)
    if value < 1:
        raise argparse.ArgumentTypeError("1 이상의 정수여야 합니다")
    return value


def parse_args():
    p = argparse.ArgumentParser(description="armature 측정 및 분석")
    p.add_argument("--motor-id", type=lambda v: int(v, 0), required=True)
    p.add_argument("--model", choices=list(SPECS.keys()), required=True)
    p.add_argument("--channel", default=None, help="기본값: motor-id로 자동 판단 (can0/can1)")
    p.add_argument("--torques", type=float, nargs="+", required=True,
                   help="측정할 피드포워드 토크 목록(N*m, 부호 있음)")
    p.add_argument("--repeats", type=positive_int, default=1, help="토크별 반복 횟수, 기본 1")
    p.set_defaults(
        interface=DEFAULT_INTERFACE, host_id=HOST_ID,
        max_vel=5.0, max_time=2.0, max_turns=3.0,
        settle_pos_tol=0.01, settle_time=0.3, settle_timeout=5.0,
        settle_max_vel=3.0, feedback_timeout=0.3,
        poll_timeout=0.03, rate=0.0, out=None,
    )
    return p.parse_args()


def confirm(args, model):
    rated = RATED_TORQUE[model]
    print("=" * 70)
    print("armature 측정 및 분석")
    print(f"  모터 ID       : {args.motor_id} ({model.upper()})")
    print(f"  채널          : {args.channel}")
    print(f"  인가 토크     : {args.torques} N*m x {args.repeats}회")
    print(f"  정지 조건     : |vel|>={args.max_vel} rad/s, "
          f"t>={args.max_time}s, |dpos|>={args.max_turns} turn")
    print("  전제: 출력축에 부하(크랭크/링크) 없음, 벤치에 고정된 상태")
    print("=" * 70)
    if any(abs(torque) > rated for torque in args.torques):
        print(f"경고: 일부 인가 토크가 정격({rated} N*m)을 넘습니다.")
    if any(abs(torque) > PEAK_TORQUE[model] for torque in args.torques):
        print(f"오류: 피크 토크({PEAK_TORQUE[model]} N*m)를 초과하는 값이 있습니다. 중단합니다.")
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
            status = (f"pos={pos:+.4f} rad  vel={vel:+.4f} rad/s(참고용)  tq={tq:+.3f} N*m  "
                      f"fault={fault:02X}(미검증)  quiet={now - ref_time:4.2f}/{args.settle_time:.2f}s")
        if now - last_print >= 0.05:
            print(f"\r  {status}    ", end="", flush=True)
            last_print = now
    print()
    if misses > 0:
        print(f"  (타임아웃 동안 응답 없음 {misses}회 발생 — CAN 통신 자체가 불안정했을 수 있음)")
    return None


def run_capture(motor, torque_cmd, pos_start, args):
    rows = []
    misses = 0
    fault_count = 0
    t0 = time.monotonic()
    period = (1.0 / args.rate) if args.rate > 0 else 0.0
    stop_reason = "max_time"
    last_ok = t0
    try:
        while True:
            now = time.monotonic()
            elapsed = now - t0
            if elapsed >= args.max_time:
                stop_reason = "max_time"
                break
            motor.control(pos=0.0, vel=0.0, kp=0.0, kd=0.0, torque=torque_cmd)
            fb = motor.poll_feedback(timeout=args.poll_timeout)
            if fb is None:
                misses += 1
                if time.monotonic() - last_ok > args.feedback_timeout:
                    stop_reason = "feedback_lost"
                    break
                continue
            last_ok = time.monotonic()
            t_recv, pos, vel, tq, temp, fault = fb
            rows.append((t_recv - t0, pos - pos_start, vel, tq, temp, fault))
            if fault:
                fault_count += 1
            if abs(vel) >= args.max_vel:
                stop_reason = "max_vel"
                break
            if abs(pos - pos_start) >= args.max_turns * 2.0 * math.pi:
                stop_reason = "max_turns"
                break
            if period > 0:
                sleep_left = period - (time.monotonic() - now)
                if sleep_left > 0:
                    time.sleep(sleep_left)
    except KeyboardInterrupt:
        stop_reason = "keyboard_interrupt"
    return rows, stop_reason, fault_count


def save_csv(path, rows, meta):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        for key, value in meta.items():
            f.write(f"# {key}: {value}\n")
        f.write("t_s,pos_rel_rad,vel_rad_s,torque_meas_Nm,temp_C,fault_hex\n")
        for t_s, pos, vel, tq, temp, fault in rows:
            f.write(f"{t_s:.6f},{pos:.6f},{vel:.6f},{tq:.4f},{temp:.1f},{fault:02X}\n")


def capture_once(args, torque, run_index):
    args.torque = torque
    spec = SPECS[args.model]
    bus = can.Bus(channel=args.channel, interface=args.interface)
    motor = None
    rows, stop_reason, fault_count = [], "not_started", 0
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
            print("폴트 레지스터(0x3022) 읽기 실패 (인덱스가 이 방식으로 안 읽힐 수 있음, 무시하고 진행)")
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
        print(f"정지 확인됨 (기준 위치 {pos_start:+.4f} rad). 토크 스텝 인가...")

        rows, stop_reason, fault_count = run_capture(motor, args.torque, pos_start, args)
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
    max_vel_reached = max(abs(r[2]) for r in rows)
    achieved_hz = (len(rows) - 1) / duration if duration > 0 else 0.0

    print(f"\n캡처 종료: {stop_reason}")
    print(f"  샘플 수       : {len(rows)}")
    print(f"  구간          : {duration*1000:.1f} ms")
    print(f"  달성 폴링율   : {achieved_hz:.0f} Hz")
    print(f"  최대 |vel|    : {max_vel_reached:.3f} rad/s")
    if len(rows) < 15:
        print("  경고: 샘플이 15개 미만입니다. --torque 를 낮추거나 --max-vel 을 높여 재시도 권장.")
    if stop_reason == "feedback_lost":
        print(f"  오류: 피드백이 {args.feedback_timeout}s 넘게 끊겨 중단했습니다 — 토크를 건 채 "
              "속도/회전량을 감지 못 하는 상태라 위험합니다. 이 결과는 쓰지 말고 CAN 상태 확인 후 재측정하세요.")
    if fault_count:
        print(f"  참고: 피드백 프레임 중 {fault_count}/{len(rows)}개에서 fault 바이트가 0이 아니었습니다 "
              "(해석 미검증 — 위 0x3022 레지스터 값과 비교해볼 것). 캡처는 정상 진행됨.")

    ts = datetime.now().strftime("%Y%m%dT%H%M%S_%f")
    fname = (f"id{args.motor_id}_{args.model}_{args.torque:+.3f}Nm_{ts}_r{run_index:02d}.csv"
             .replace("+", "p").replace("-", "m"))
    out_path = os.path.join(DATA_DIR, fname)

    meta = {
        "motor_id": args.motor_id,
        "model": args.model,
        "channel": args.channel,
        "host_id": f"0x{args.host_id:02X}",
        "torque_cmd_Nm": f"{args.torque:.4f}",
        "started_at": datetime.now().isoformat(timespec="seconds"),
        "stop_reason": stop_reason,
        "samples": len(rows),
        "duration_s": f"{duration:.4f}",
    }
    save_csv(out_path, rows, meta)
    print(f"\n저장됨: {out_path}")
    return (0 if stop_reason in {"max_time", "max_vel", "max_turns"} else 1), out_path, stop_reason


def main():
    args = parse_args()
    if args.channel is None:
        args.channel = channel_for_id(args.motor_id)

    for torque in args.torques:
        args.torque = torque
        if report_invalid_args(validate_args(args, args.model, ARG_CHECKS)):
            return 1
    if not confirm(args, args.model):
        return 1

    plan = [torque for torque in args.torques for _ in range(args.repeats)]
    paths = []
    failed = False
    for index, torque in enumerate(plan, 1):
        print(f"\n{'=' * 70}\n[{index}/{len(plan)}] torque={torque:+.3f} N*m\n{'=' * 70}")
        rc, path, stop_reason = capture_once(args, torque, index)
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
