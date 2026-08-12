#!/usr/bin/env python3
"""항목 3 (점성 damping) 측정 — 정속 구간 캡처.

운영제어(type 0x01, armature 측정과 동일 메커니즘)로 kp=0, kd>0, v_set=목표속도를
계속 보낸다. 이러면 t_ref = kd*(v_set - v_actual) 가 되어 "속도만 추종하는 P제어"가
되고, 정상상태(가속도=0)에 도달하면 τ_실측 = b*ω_실측 + c*sign(ω_실측) 관계가 성립한다.
analyze_damping.py 가 이 CSV들을(서로 다른 speed로 여러 번 받아서) 묶어 b(damping)를 구한다.

⚠ 처음엔 run_mode=2(속도모드)+능동보고(type 0x18) 조합으로 시도했으나 능동보고
페이로드 형식이 매뉴얼에 명시돼 있지 않아 실패했다(2026-08-09) — 그래서 armature와
똑같이 검증된 운영제어 방식으로 바꿨다. 정상상태 물리 관계는 어떤 방식으로 그
속도/토크에 도달했든 동일하게 성립하므로 정확도에 문제없다.

전제 조건 (armature 측정과 동일):
  - 모터 출력축에 아무 부하(크랭크, 링크 등)도 연결되어 있지 않아야 한다.
  - 벤치에 고정되어 있어 안전하게 자유회전 가능해야 한다.

방법: 같은 모터에 서로 다른 --speed 값으로 여러 번(3~4개 이상, 방향도 섞어서 권장)
실행한다. 각 실행은 CSV 1개를 만든다.

  python3 capture_velocity_hold.py --motor-id 11 --model rs02 --speed 1.0
  python3 capture_velocity_hold.py --motor-id 11 --model rs02 --speed 2.0
  python3 capture_velocity_hold.py --motor-id 11 --model rs02 --speed 3.0
  python3 capture_velocity_hold.py --motor-id 11 --model rs02 --speed -2.0
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
    HOST_ID, DEFAULT_INTERFACE, SPECS,
    RUN_MODE_INDEX, RUN_MODE_OPERATION, MECH_POS_INDEX, FAULT_STA_INDEX,
    Motor, channel_for_id, decode_fault_bits, active_brake,
    validate_args, report_invalid_args,
)

# 실기 구동 전 반드시 검사할 인자들 (validate_args docstring: 안 하면 조용히 clamp됨)
ARG_CHECKS = [
    ("speed", "speed"), ("kd", "positive"), ("hold-time", "positive"),
    ("ramp-time", "positive"), ("max-turns", "positive"),
    ("feedback-timeout", "positive"),
]

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")


def parse_args():
    p = argparse.ArgumentParser(description="정속 구간 캡처 (damping 측정용)")
    p.add_argument("--motor-id", type=lambda v: int(v, 0), required=True)
    p.add_argument("--model", choices=list(SPECS.keys()), required=True)
    p.add_argument("--speed", type=float, required=True, help="목표 속도 (rad/s, 부호 있음)")
    p.add_argument("--kd", type=float, default=2.0,
                   help="속도추종 kd (kp=0, t_ref=kd*(v_set-v_actual)), 기본 2.0")
    p.add_argument("--channel", default=None, help="기본값: motor-id로 자동 판단 (can0/can1)")
    p.add_argument("--interface", default=DEFAULT_INTERFACE)
    p.add_argument("--host-id", type=lambda v: int(v, 0), default=HOST_ID)
    p.add_argument("--hold-time", type=float, default=1.5, help="정속 구간 기록 시간(초), 기본 1.5")
    p.add_argument("--ramp-time", type=float, default=0.5,
                   help="목표속도로 접근할 때 그냥 기다리는 시간(초), 기본 0.5. "
                        "J~0.0037, kd=2 기준 시정수가 ~2ms라 노이즈 많은 vel로 정착판정하는 것보다 "
                        "고정 대기가 더 안정적임(2026-08-09 실측으로 확인)")
    p.add_argument("--max-turns", type=float, default=8.0, help="시작 위치 대비 최대 누적 회전수, 기본 8.0")
    p.add_argument("--feedback-timeout", type=float, default=0.3,
                   help="이 시간(초) 넘게 피드백이 없으면 중단, 기본 0.3. "
                        "회전 명령 중 통신이 끊기면 안전한계도 못 걸리므로 반드시 필요")
    p.add_argument("--out", default=None, help="출력 CSV 경로, 기본: data/ 아래 자동 생성")
    p.add_argument("--yes", action="store_true", help="확인 프롬프트 없이 바로 시작")
    return p.parse_args()


def confirm(args, model):
    print("=" * 70)
    print("정속 구간 캡처 (damping 측정)")
    print(f"  모터 ID       : {args.motor_id} ({model.upper()})")
    print(f"  채널          : {args.channel}")
    print(f"  목표 속도     : {args.speed:+.3f} rad/s  (kd={args.kd}, kp=0 순수 속도추종)")
    print(f"  램프 대기     : {args.ramp_time}s (고정, vel 노이즈로 자동판정 안 함)")
    print(f"  기록 시간     : {args.hold_time}s")
    print(f"  안전한계      : |dpos|>={args.max_turns} turn")
    print("  전제: 출력축에 부하(크랭크/링크) 없음, 벤치에 고정된 상태")
    print("=" * 70)
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


def ramp_to_speed(motor, target_speed, kd, pos_ref, args, label):
    """control(pos=0,vel=target,kp=0,kd=kd)를 --ramp-time 동안 계속 보낸다.

    vel 신호는 armature 정지판정 때와 마찬가지로 노이즈가 커서(2026-08-09 실측,
    목표 부근에서도 샘플간 ±0.2~0.3 rad/s씩 튐) 그 노이즈로 "정착됐는지" 자동판정하는
    건 오히려 불안정하다. 대신 물리적으로 시정수가 매우 짧다는 걸 이용해(J~0.0037,
    kd=2 기준 τ=J/kd~2ms) 고정 시간만 기다린다 — 사람이 "이 정도면 됐겠지"라고
    판단하는 것과 같은 방식이고, 여기선 실측 시정수 대비 훨씬 넉넉한 값(기본 0.5s)이다.
    ⚠ 피드백이 끊기면 즉시 중단한다(--feedback-timeout). 이게 없으면 통신이 죽은 뒤에도
    회전 명령만 계속 나가서, 모터가 실제로 얼마나 돌고 있는지 모르는 채 max-turns 같은
    안전한계도 전혀 못 걸린다(2026-08-09 지적으로 발견, 모의 CAN에서 5만회 명령 확인).
    반환: (계속 진행해도 되는지, 마지막 pos, 마지막 vel, 중단 사유)"""
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
            status = (f"[{label}] pos={pos:+.4f}  vel={vel:+.3f}  target={target_speed:+.3f}  "
                      f"tq={tq:+.3f} N*m")
        if now - last_print >= 0.1:
            print(f"\r  {status}    ", end="", flush=True)
            last_print = now
    print()
    return True, last_pos, last_vel, "ok"


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
            # 눈 감은 채 계속 돌리지 않는다 — ramp_to_speed의 docstring 참고.
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


def main():
    args = parse_args()
    if args.channel is None:
        args.channel = channel_for_id(args.motor_id)

    if report_invalid_args(validate_args(args, args.model, ARG_CHECKS)):
        return 1
    # kd는 MIT 스케일 상한(RS02=5, RS03=100)을 넘으면 조용히 clamp돼 실제 추종 강성이 달라진다.
    if args.kd > SPECS[args.model].kd_max:
        print(f"인자 오류 — 실행하지 않습니다:\n  --kd({args.kd}) 가 {args.model.upper()} "
              f"kd 상한({SPECS[args.model].kd_max})을 초과합니다.")
        return 1

    if not confirm(args, args.model):
        return 1

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

        # ⚠ 2026-08-09: mechPos(0x7019, 절대/멀티턴)와 실시간 제어루프 피드백(type 0x02)이
        # 반복측정으로 절대위치가 인코딩 범위(±4π)를 넘으면 서로 다른 wrap 구간을 쓰게 됨을
        # armature/friction에서 확인함(오차가 정확히 8π) — 여기서도 안전한계(max-turns) 기준점을
        # mechPos가 아니라 실시간 피드백에서 새로 한 번 읽어서 도메인을 맞춘다.
        pos_ref = None
        for _ in range(5):  # 잠깐 응답이 느릴 수 있으니 몇 번 재시도
            motor.control(pos=0.0, vel=0.0, kp=0.0, kd=0.0, torque=0.0)
            fb0 = motor.poll_feedback(timeout=0.2)
            if fb0 is not None:
                pos_ref = fb0[1]
                break
        if pos_ref is None:
            # mechPos로 폴백하지 않는다 — 도메인이 달라(멀티턴) max-turns 판정이 어긋나고,
            # 그건 "안전한계가 있는 줄 알았는데 실제로는 없는" 최악의 상태다. 그냥 중단한다.
            print("오류: 실시간 피드백(type 0x02)을 받지 못해 안전한계 기준점을 못 잡았습니다. "
                  "CAN 상태를 확인하세요 (ip link show).")
            return 1

        print(f"목표 속도 {args.speed:+.3f} rad/s 로 {args.ramp_time}s 접근...")
        ok, pos_now, vel_now, ramp_reason = ramp_to_speed(
            motor, args.speed, args.kd, pos_ref, args, "ramp-up")
        if not ok:
            print(f"오류: 램프 중 중단됨 (사유={ramp_reason}).")
            return 1
        if args.speed != 0 and (vel_now == 0.0 or vel_now / args.speed < 0):
            print(f"경고: 속도가 목표 방향으로 안 붙는 것 같습니다 (vel={vel_now:+.3f}, "
                  f"target={args.speed:+.3f}). kd를 올리거나 ramp-time을 늘려보세요.")
        print(f"램프 종료 (pos={pos_now:+.4f} rad, vel={vel_now:+.3f} rad/s). 정속 구간 기록 시작...")

        rows, stop_reason = run_hold(motor, args.speed, args.kd, pos_ref, args)

        print("감속 중...")
        ramp_to_speed(motor, 0.0, args.kd, None, args, "ramp-down")
    except KeyboardInterrupt:
        # ramp_to_speed/run_hold는 자체 KeyboardInterrupt 처리가 없다 — 모터 정지는
        # finally가 항상 처리하니 안전엔 영향 없고, 트레이스백 대신 깔끔한 메시지만 내기 위함.
        print("\n\n중단됨 (Ctrl-C).")
        stop_reason = "keyboard_interrupt"
    finally:
        if motor is not None:
            # ⚠ brake와 stop을 같은 try에 두면 brake 전송 실패 시 stop까지 건너뛴다 —
            # 정지 명령은 무슨 일이 있어도 나가야 하므로 분리한다.
            try:
                active_brake(motor)  # 정상 경로는 이미 ramp-down으로 감속되지만, 예외 경로 대비
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

    out_path = args.out
    if out_path is None:
        ts = datetime.now().strftime("%Y%m%dT%H%M%S")
        fname = f"id{args.motor_id}_{args.model}_{args.speed:+.3f}rads_{ts}.csv".replace("+", "p").replace("-", "m")
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
    print("다른 speed 값(방향도 섞어서)으로 반복 실행한 뒤 analyze_damping.py 에 CSV들을 같이 넘기세요.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
