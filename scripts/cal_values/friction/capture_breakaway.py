#!/usr/bin/env python3
"""항목 4 (정지마찰/breakaway) 측정 — 계단식 토크 증가 탐색.

정지 상태에서 아주 낮은 피드포워드 토크(kp=kd=0)부터 시작해, --probe-time마다
--step씩 토크를 계단식으로 올린다. armature 측정과 같은 운영제어(type 0x01) 메커니즘을
그대로 쓴다. 위치가 --move-threshold(기본 0.03 rad ≈ 1.7°, 정지 노이즈 ±0.09°보다
훨씬 큼)만큼 벗어나는 순간을 "움직이기 시작함"으로 보고 멈춘다 — 그 직전 단계(안 움직임)와
그 단계(움직임) 사이가 breakaway 토크의 브래킷이다.

⚠ 속도(vel) 필드는 정지 근처에서 노이즈가 크다는 게(armature/damping 측정에서 반복 확인)
이 스크립트에서도 똑같이 적용된다 — 그래서 "움직였는지" 판정은 반드시 pos로 한다.

전제 조건 (armature/damping 측정과 동일):
  - 모터 출력축에 아무 부하(크랭크, 링크 등)도 연결되어 있지 않아야 한다.
  - 벤치에 고정되어 있어 안전하게 자유회전 가능해야 한다.

방법: 모터 1개당 방향(+/-)별로 1회 이상 실행. 반복 측정하면 재현성도 확인 가능.

  python3 capture_breakaway.py --motor-id 11 --model rs02 --sign 1
  python3 capture_breakaway.py --motor-id 11 --model rs02 --sign -1
  python3 capture_breakaway.py --motor-id 12 --model rs02 --sign 1
  python3 capture_breakaway.py --motor-id 12 --model rs02 --sign -1
"""
import argparse
import math
import os
import sys
import time
from datetime import datetime

import can

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # cal_values/ (common.py)
from common import (  # noqa: E402
    HOST_ID, DEFAULT_INTERFACE, SPECS, RATED_TORQUE, PEAK_TORQUE,
    RUN_MODE_INDEX, RUN_MODE_OPERATION, MECH_POS_INDEX, FAULT_STA_INDEX,
    Motor, channel_for_id, decode_fault_bits, active_brake,
    validate_args, report_invalid_args,
)

# 실기 구동 전 반드시 검사할 인자들 (validate_args docstring: 안 하면 조용히 clamp됨)
ARG_CHECKS = [
    ("start-torque", "torque"), ("step", "positive"), ("max-torque", "torque"),
    ("probe-time", "positive"), ("move-threshold", "positive"),
    ("max-vel", "speed"), ("max-time", "positive"),
    ("settle-pos-tol", "positive"), ("settle-time", "positive"),
    ("settle-timeout", "positive"), ("settle-max-vel", "speed"),
    ("feedback-timeout", "positive"),
]

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")


def parse_args():
    p = argparse.ArgumentParser(description="계단식 토크 증가로 정지마찰(breakaway) 탐색")
    p.add_argument("--motor-id", type=lambda v: int(v, 0), required=True)
    p.add_argument("--model", choices=list(SPECS.keys()), required=True)
    p.add_argument("--sign", type=int, choices=[1, -1], default=1, help="탐색 방향, 기본 +1")
    p.add_argument("--channel", default=None, help="기본값: motor-id로 자동 판단 (can0/can1)")
    p.add_argument("--interface", default=DEFAULT_INTERFACE)
    p.add_argument("--host-id", type=lambda v: int(v, 0), default=HOST_ID)
    p.add_argument("--start-torque", type=float, default=0.02, help="시작 토크 크기 N*m, 기본 0.02")
    p.add_argument("--step", type=float, default=0.02, help="단계별 토크 증가량 N*m, 기본 0.02")
    p.add_argument("--probe-time", type=float, default=0.3, help="각 단계 유지시간(초), 기본 0.3")
    p.add_argument("--move-threshold", type=float, default=0.03,
                   help="이만큼 pos가 벗어나면 '움직임 시작'으로 판정 (rad), 기본 0.03")
    p.add_argument("--max-torque", type=float, default=1.0,
                   help="이 크기까지 안 움직이면 이상으로 보고 중단 (N*m), 기본 1.0")
    p.add_argument("--max-vel", type=float, default=3.0, help="안전한계 속도(rad/s), 기본 3.0")
    p.add_argument("--max-time", type=float, default=30.0, help="전체 탐색 최대 시간(초), 기본 30")
    p.add_argument("--settle-pos-tol", type=float, default=0.01, help="시작 전 정지판정 위치허용범위 rad")
    p.add_argument("--settle-time", type=float, default=0.3, help="정지 확인 최소 지속시간(초)")
    p.add_argument("--settle-timeout", type=float, default=5.0, help="정지 대기 최대 시간(초)")
    p.add_argument("--settle-max-vel", type=float, default=3.0,
                   help="정지판정 중 이 속도(rad/s) 이상이면 즉시 긴급정지, 기본 3.0 (2026-08-09 사고 대응)")
    p.add_argument("--feedback-timeout", type=float, default=0.3,
                   help="이 시간(초) 넘게 피드백이 없으면 중단, 기본 0.3. 토크를 걸어둔 채 "
                        "움직임 감지를 못 하는 상태가 길어지지 않게 함")
    p.add_argument("--out", default=None, help="출력 CSV 경로, 기본: data/ 아래 자동 생성")
    p.add_argument("--yes", action="store_true", help="확인 프롬프트 없이 바로 시작")
    return p.parse_args()


def confirm(args, model):
    rated = RATED_TORQUE[model]
    n_steps_est = int(math.ceil((args.max_torque - args.start_torque) / args.step)) + 1
    print("=" * 70)
    print("계단식 정지마찰(breakaway) 탐색")
    print(f"  모터 ID       : {args.motor_id} ({model.upper()})")
    print(f"  채널          : {args.channel}")
    print(f"  방향          : {'+' if args.sign > 0 else '-'}  "
          f"(시작 {args.start_torque} N*m, 단계 {args.step} N*m, 최대 {args.max_torque} N*m,"
          f" 단계당 {args.probe_time}s → 최악 {n_steps_est*args.probe_time:.1f}s)")
    print(f"  움직임 판정   : |dpos|>={args.move_threshold} rad ({math.degrees(args.move_threshold):.2f} deg)")
    print("  전제: 출력축에 부하(크랭크/링크) 없음, 벤치에 고정된 상태")
    print("=" * 70)
    if args.max_torque > rated:
        print(f"경고: max-torque가 정격({rated} N*m)을 넘습니다. 무부하 시험엔 보통 불필요하게 큽니다.")
    if args.max_torque > PEAK_TORQUE[model]:
        print(f"오류: max-torque가 피크 토크({PEAK_TORQUE[model]} N*m)를 초과합니다. 중단합니다.")
        return False
    if args.yes:
        return True
    if not sys.stdin.isatty():
        print("확인 입력이 필요합니다. --yes 로 실행하거나 대화형 터미널에서 실행하세요.")
        return False
    try:
        input("\n시작하려면 Enter, 취소하려면 Ctrl-C: ")
    except (KeyboardInterrupt, EOFError):
        print("\n취소됨.")
        return False
    return True


def wait_settled(motor, hold_pos, args):
    """armature 측정과 동일한 위치기반 정지판정 (vel은 노이즈가 커서 안 씀).

    ⚠ 2026-08-09 사고 이후 kp=0(위치 목표 없이 kd로만 속도 감쇠)로 변경 — 예전엔 kp>0으로
    hold_pos(마지막으로 읽은 mechPos, 절대/멀티턴 값)를 위치 목표로 계속 명령했는데, 반복측정
    으로 절대위치가 인코딩 범위(±4π≈±12.57rad)를 넘어 누적되자 mechPos와 실시간 제어루프가
    보는 위치가 서로 다른 wrap 구간을 쓰게 됐다(실측 오차가 정확히 8π). 목표-현재 오차가
    25rad 가까이로 잘못 계산되면서 모터가 그 가짜 오차를 없애려 순간 5+ rad/s로 확 돌아버렸다
    (armature/capture_torque_step.py와 동일 버그, 같이 고침)."""
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
                # 방어적 안전장치(2026-08-09 사고 이후 추가) — 정지판정 단계는 원래 정지
                # 상태를 확인하는 곳이라 이 정도 속도가 나오면 원인이 뭐든 일단 즉시 멈춘다.
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


def run_search(motor, pos_start, args):
    """torque를 계단식으로 올리며 pos가 move_threshold를 넘는 순간을 찾는다.
    반환: rows, stop_reason, torque_at_stop (움직임이 감지된 단계의 토크, 또는 마지막 시도값)"""
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
                # 방금 끝난 단계에서 실측토크가 명령토크를 실제로 따라갔는지 확인.
                # 계속 안 따라가면 "안 움직인 것"이 아니라 통신 이상일 가능성이 높다.
                # ⚠ 2026-08-09: 처음엔 abs(current_torque)>0.15 일 때만 검사했는데, 저토크
                # 구간(예: 0.04N*m 명령에 실측 0.15~0.22N*m)에서도 실제로 불일치가 났던 게
                # 이 게이트 때문에 통과해버려서(ID11 +방향, breakaway가 가짜로 낮게 잡힘)
                # 전 구간을 검사하도록 수정. 저토크 구간 노이즈 허용을 위해 절대 하한(0.08)을 둠.
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
                # ⚠ 예전엔 "500회 미스"로만 끊었는데, 폴링이 3kHz대라 500회가 수십 ms가
                # 아니라 최악 25초였다 — 그동안 토크 명령은 계속 나가고 움직임 감지(pos)는
                # 못 하니 안전한계가 사실상 없는 상태였다. 시간 기준으로 바꾼다.
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
            if abs(pos - pos_start) >= args.move_threshold:
                if steps_taken == 0:
                    # 정지 확인 직후, 아직 한 단계도 못 올렸는데(=시작토크에서) 움직임 감지.
                    # 이 정도로 작은 토크에 실제로 움직였을 리 없다(2026-08-09 ID12 +방향에서
                    # 실제로 겪음: mid=+0.01 이상치) — 글리치/잔류움직임 의심, 정상 결과와 구분.
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


def main():
    args = parse_args()
    if args.channel is None:
        args.channel = channel_for_id(args.motor_id)

    if report_invalid_args(validate_args(args, args.model, ARG_CHECKS)):
        return 1
    if args.start_torque > args.max_torque:
        print(f"인자 오류 — 실행하지 않습니다:\n  --start-torque({args.start_torque}) 가 "
              f"--max-torque({args.max_torque}) 보다 큽니다.")
        return 1

    if not confirm(args, args.model):
        return 1

    spec = SPECS[args.model]
    bus = can.Bus(channel=args.channel, interface=args.interface)
    motor = None
    rows, stop_reason, torque_at_stop = [], "not_started", 0.0
    try:
        motor = Motor(bus, args.motor_id, spec, host_id=args.host_id)

        motor.stop()
        time.sleep(0.02)
        initial_pos = motor.read_param_f32(MECH_POS_INDEX, timeout=0.3)
        if initial_pos is None:
            print(f"오류: 모터 ID {args.motor_id} 가 {args.channel} 에서 응답하지 않습니다.")
            return 1
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
            return 1
        print(f"정지 확인됨 (기준 위치 {pos_start:+.4f} rad). 계단식 토크 탐색 시작...")

        rows, stop_reason, torque_at_stop = run_search(motor, pos_start, args)
    except KeyboardInterrupt:
        # wait_settled()는 자체 KeyboardInterrupt 처리가 없어(run_search만 있음) 여기서
        # 받는다 — 모터 정지는 finally가 항상 처리하니 안전엔 영향 없고, 트레이스백 대신
        # 깔끔한 메시지만 내기 위함.
        print("\n\n중단됨 (Ctrl-C).")
        stop_reason = "keyboard_interrupt"
    finally:
        if motor is not None:
            # ⚠ brake와 stop을 같은 try에 두면 brake 전송 실패 시 stop까지 건너뛴다 —
            # 정지 명령은 무슨 일이 있어도 나가야 하므로 분리한다.
            try:
                active_brake(motor)  # damping≈0이라 바로 끊으면 관성으로 미끄러짐(2026-08-09 확인)
            except can.CanError:
                pass
            try:
                motor.stop()
            except can.CanError:
                pass
        bus.shutdown()

    if not rows:
        print(f"샘플이 하나도 기록되지 않았습니다 (stop_reason={stop_reason}). CAN 응답을 확인하세요.")
        return 1

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

    out_path = args.out
    if out_path is None:
        ts = datetime.now().strftime("%Y%m%dT%H%M%S")
        sign_tag = "pos" if args.sign > 0 else "neg"
        fname = f"id{args.motor_id}_{args.model}_{sign_tag}_{ts}.csv"
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
        "stop_reason": stop_reason,
        "breakaway_low_Nm": f"{breakaway_low:.4f}" if breakaway_low is not None else "",
        "breakaway_high_Nm": f"{breakaway_high:.4f}" if breakaway_high is not None else "",
        "breakaway_mid_Nm": f"{breakaway_mid:.4f}" if breakaway_mid is not None else "",
        "samples": len(rows),
        "duration_s": f"{duration:.4f}",
    }
    save_csv(out_path, rows, meta)
    print(f"\n저장됨: {out_path}")
    print("다른 방향(--sign)이나 다른 모터로 반복 실행한 뒤 analyze_breakaway.py 에 CSV들을 같이 넘기세요.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
