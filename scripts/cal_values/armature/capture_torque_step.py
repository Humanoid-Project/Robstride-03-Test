#!/usr/bin/env python3
"""항목 2 (armature/회전자 관성) 측정 — 토크 스텝 응답 캡처.

정지 상태에서 일정한 피드포워드 토크(kp=kd=0)를 순간적으로 인가하고, 초반 가속
구간의 위치/속도/토크를 최대한 빠르게 기록해 CSV로 저장한다. analyze_armature.py
가 이 CSV들을(서로 다른 torque 값으로 여러 번 받아서) 묶어 J를 계산한다.

전제 조건 (중요):
  - 모터 출력축에 아무 부하(크랭크, 링크 등)도 연결되어 있지 않아야 한다.
    순수 로터+감속기 관성만 측정하기 위함.
  - 벤치에 고정되어 있어 안전하게 자유회전 가능해야 한다 (케이블 감김 주의 →
    --max-turns 로 회전량 제한).

방법: 같은 모터에 서로 다른 --torque 값으로 이 스크립트를 여러 번(3~4회 권장)
실행한다. 각 실행은 CSV 1개를 만든다.

  python3 capture_torque_step.py --motor-id 11 --model rs02 --torque 0.5
  python3 capture_torque_step.py --motor-id 11 --model rs02 --torque 1.0
  python3 capture_torque_step.py --motor-id 11 --model rs02 --torque 1.5
  python3 capture_torque_step.py --motor-id 11 --model rs02 --torque 2.0

캡처 후 --max-vel 도달까지 시간이 몇 ms 뿐이라 샘플이 너무 적으면(<15개 정도)
--torque 를 낮추거나 --max-vel 을 높여 다시 시도할 것 (실행 후 콘솔에 안내 출력됨).
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
    ("torque", "torque"), ("max-vel", "speed"), ("max-time", "positive"),
    ("max-turns", "positive"), ("settle-pos-tol", "positive"),
    ("settle-time", "positive"), ("settle-timeout", "positive"),
    ("settle-max-vel", "speed"), ("feedback-timeout", "positive"),
    ("poll-timeout", "positive"), ("rate", "nonneg"),
]

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")


def parse_args():
    p = argparse.ArgumentParser(description="토크 스텝 응답 캡처 (armature 측정용)")
    p.add_argument("--motor-id", type=lambda v: int(v, 0), required=True)
    p.add_argument("--model", choices=list(SPECS.keys()), required=True)
    p.add_argument("--torque", type=float, required=True, help="인가할 피드포워드 토크 (N*m, 부호 있음)")
    p.add_argument("--channel", default=None, help="기본값: motor-id로 자동 판단 (can0/can1)")
    p.add_argument("--interface", default=DEFAULT_INTERFACE)
    p.add_argument("--host-id", type=lambda v: int(v, 0), default=HOST_ID)
    p.add_argument("--max-vel", type=float, default=5.0, help="이 속도(rad/s) 도달 시 정지, 기본 5.0")
    p.add_argument("--max-time", type=float, default=2.0, help="최대 캡처 시간(초), 기본 2.0")
    p.add_argument("--max-turns", type=float, default=3.0, help="시작 위치 대비 최대 회전수, 기본 3.0")
    p.add_argument("--settle-pos-tol", type=float, default=0.01,
                   help="정지로 간주할 위치 변동 허용범위 rad, 기본 0.01 (vel 필드는 근처에서 노이즈가 커서 안 씀)")
    p.add_argument("--settle-time", type=float, default=0.3, help="정지 상태 확인 최소 지속시간(초)")
    p.add_argument("--settle-timeout", type=float, default=5.0, help="정지 대기 최대 시간(초)")
    p.add_argument("--settle-max-vel", type=float, default=3.0,
                   help="정지판정 중 이 속도(rad/s) 이상이면 즉시 긴급정지, 기본 3.0 (2026-08-09 사고 대응)")
    p.add_argument("--feedback-timeout", type=float, default=0.3,
                   help="이 시간(초) 넘게 피드백이 없으면 중단, 기본 0.3. 토크를 건 채 속도/회전량 "
                        "감지를 못 하는 상태가 길어지지 않게 함")
    p.add_argument("--poll-timeout", type=float, default=0.03, help="피드백 프레임 대기 타임아웃(초)")
    p.add_argument("--rate", type=float, default=0.0, help="캡처 루프 속도 제한 Hz, 0=제한 없음(기본)")
    p.add_argument("--out", default=None, help="출력 CSV 경로, 기본: data/armature/ 아래 자동 생성")
    p.add_argument("--yes", action="store_true", help="확인 프롬프트 없이 바로 시작")
    return p.parse_args()


def confirm(args, model):
    rated = RATED_TORQUE[model]
    print("=" * 70)
    print("토크 스텝 캡처 (armature 측정)")
    print(f"  모터 ID       : {args.motor_id} ({model.upper()})")
    print(f"  채널          : {args.channel}")
    print(f"  인가 토크     : {args.torque:+.3f} N*m  (정격 {rated:.1f} N*m)")
    print(f"  정지 조건     : |vel|>={args.max_vel} rad/s, "
          f"t>={args.max_time}s, |dpos|>={args.max_turns} turn")
    print("  전제: 출력축에 부하(크랭크/링크) 없음, 벤치에 고정된 상태")
    print("=" * 70)
    if abs(args.torque) > rated:
        print(f"경고: 인가 토크가 정격({rated} N*m)을 넘습니다. 무부하 시험엔 보통 불필요하게 큽니다.")
    if abs(args.torque) > PEAK_TORQUE[model]:
        print(f"오류: 인가 토크가 피크 토크({PEAK_TORQUE[model]} N*m)를 초과합니다. 중단합니다.")
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
    """작은 kd로 속도만 감쇠시키며(위치 목표 없음), pos가 settle_pos_tol 범위 안에서
    settle_time 이상 안 벗어날 때까지 대기. vel 필드는 정지 근처에서 노이즈가 커서
    (2026-08-09 실측, ±0.1~0.2 rad/s) 판정에 안 쓰고 참고 출력에만 남긴다.

    ⚠ 2026-08-09 사고: 예전엔 kp>0으로 hold_pos(마지막으로 읽은 mechPos, 절대/멀티턴 값)를
    위치 목표로 계속 명령했는데, 반복측정으로 절대위치가 인코딩 범위(±4π≈±12.57rad)를 넘어
    누적되자 mechPos와 실시간 제어루프가 보는 위치가 서로 다른 wrap 구간을 쓰게 됐다
    (실측 오차가 정확히 8π). 목표-현재 오차가 25rad 가까이로 잘못 계산되면서 모터가 그
    가짜 오차를 없애려 순간 5+ rad/s로 확 돌아버렸다. damping 측정은 애초에 위치 목표 없이
    kp=0으로만 속도를 다뤄서 이 문제가 없었다 — 여기도 그 방식으로 바꿔 원천 차단."""
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
    """실제 캡처 루프. 정지 판정은 pos/vel(둘 다 CAN 피드백, 신뢰 근거 있음)만 쓴다.
    ID 필드에서 뽑은 fault 바이트는 해석이 아직 검증 안 돼(위 wait_settled 참고) 자동
    중단 트리거로 쓰지 않고 건수만 세어 나중에 경고로 보여준다 — 이상 시엔 사용자가
    직접 Ctrl-C로 멈춘다."""
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
                # 미스 "횟수"가 아니라 "시간"으로 끊는다 — 폴링이 3kHz대라 횟수 기준은
                # 실제 경과시간과 전혀 안 맞고, 그동안 토크는 계속 나가면서 max_vel/
                # max_turns 감지는 못 하는(=안전한계가 없는) 상태가 된다.
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


def main():
    args = parse_args()
    if args.channel is None:
        args.channel = channel_for_id(args.motor_id)

    if report_invalid_args(validate_args(args, args.model, ARG_CHECKS)):
        return 1

    if not confirm(args, args.model):
        return 1

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
            return 1
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
            return 1
        print(f"정지 확인됨 (기준 위치 {pos_start:+.4f} rad). 토크 스텝 인가...")

        rows, stop_reason, fault_count = run_capture(motor, args.torque, pos_start, args)
    except KeyboardInterrupt:
        # wait_settled()는 자체 KeyboardInterrupt 처리가 없어(run_capture만 있음) 여기서
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

    out_path = args.out
    if out_path is None:
        ts = datetime.now().strftime("%Y%m%dT%H%M%S")
        fname = f"id{args.motor_id}_{args.model}_{args.torque:+.3f}Nm_{ts}.csv".replace("+", "p").replace("-", "m")
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
    print("다른 토크값으로 반복 실행한 뒤 analyze_armature.py 에 CSV들을 같이 넘기세요.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
