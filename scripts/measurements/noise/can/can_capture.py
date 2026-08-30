#!/usr/bin/env python3
import argparse
import csv
import math
import os
import sys
import threading
import time
from datetime import datetime

import can

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from common import (
    DEFAULT_INTERFACE, HOST_ID, SPECS,
    RUN_MODE_INDEX, RUN_MODE_OPERATION,
    FeedbackHub, Motor, channel_for_id, active_brake,
)

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
FEEDBACK_SOURCE = "type_0x02"

MOTOR_MODEL = {
    1: "rs02", 2: "rs03", 3: "rs03", 4: "rs03", 5: "rs02", 6: "rs02",
    7: "rs02", 8: "rs03", 9: "rs03", 10: "rs03", 11: "rs02", 12: "rs02",
}

DEFAULT_DURATION = 10.0
DEFAULT_KD = 1.0
DEFAULT_FEEDBACK_TIMEOUT = 0.05
MAX_FEEDBACK_TIMEOUT = 0.5


def parse_args():
    parser = argparse.ArgumentParser(
        description="정지 상태에서 CAN 피드백(type 0x02, control()+poll_feedback() 왕복)을 "
                     "최대 속도로 반복 수신하며 타임스탬프/pos/vel/torque/temp를 원시 기록한다. "
                     "분석은 analyze_can_rate.py(주파수/지터)와 analyze_can_noise.py(pos/vel 노이즈)가 담당한다.")
    parser.add_argument("--motor-id", "--motor-ids", dest="motor_id", nargs="+",
                        type=lambda v: int(v, 0), default=list(range(1, 13)),
                        help="측정할 모터 ID, 기본: 1~12")
    parser.add_argument("--interface", default=DEFAULT_INTERFACE,
                        help="python-can 인터페이스, 기본: socketcan "
                             "('virtual'이면 실제 모터가 아니므로 확인 입력 없이 즉시 실행 — "
                             "하드웨어 없는 로직 검증용)")
    parser.add_argument("--host-id", type=lambda v: int(v, 0), default=HOST_ID)
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION,
                        help="채널당 측정 시간(초), 기본 10")
    parser.add_argument("--kd", type=float, default=DEFAULT_KD,
                        help="정지 유지용 kd (kp=0, torque=0 — wait_settled/active_brake와 동일한 "
                             "순수 속도 댐핑, 위치 목표가 없어 현재 자세를 끌어당기지 않음), 기본 1.0")
    parser.add_argument("--feedback-timeout", type=float, default=DEFAULT_FEEDBACK_TIMEOUT,
                        help="폴링 1회당 피드백 대기시간(초), 기본 0.05")
    parser.add_argument("--tag", default="", help="출력 CSV 파일명에 붙일 태그")
    return parser.parse_args()


def validate_args(args, motor_ids):
    problems = []
    if not math.isfinite(args.duration) or args.duration <= 0.0:
        problems.append("--duration은 0보다 큰 유한한 수여야 합니다.")
    if not math.isfinite(args.kd) or args.kd < 0.0:
        problems.append("--kd는 0 이상의 유한한 수여야 합니다.")
    else:
        exceeded = [
            motor_id for motor_id in motor_ids
            if args.kd > SPECS[MOTOR_MODEL[motor_id]].kd_max
        ]
        if exceeded:
            problems.append(f"--kd가 모터 허용범위를 초과합니다: ID {exceeded}")
    if (not math.isfinite(args.feedback_timeout)
            or args.feedback_timeout <= 0.0
            or args.feedback_timeout > MAX_FEEDBACK_TIMEOUT):
        problems.append(
            f"--feedback-timeout은 0보다 크고 {MAX_FEEDBACK_TIMEOUT}초 이하여야 합니다."
        )
    return problems


def confirm(args):
    print("=" * 70)
    print("CAN 피드백 원시 캡처 (control-loop 주파수 / pos·vel 노이즈 분석용)")
    print(f"  모터 ID    : {sorted(set(args.motor_id))}")
    print(f"  인터페이스 : {args.interface}")
    print(f"  피드백 출처 : {FEEDBACK_SOURCE}")
    print(f"  측정 시간  : 채널당 {args.duration:.1f}s")
    print(f"  안전 홀드  : kp=0, kd={args.kd}, torque=0 "
          "(wait_settled와 동일한 순수 속도 댐핑 — 위치를 끌어당기지 않음)")
    print("  실물 동작  : 모터 enable 및 감쇠 토크 인가 — 미세 떨림·저항 가능")
    print("  전제       : 로봇이 이미 정지해 있는 임의의 자세 (움직이지 않음)")
    print("=" * 70)
    if args.interface == "virtual":
        print("  (virtual 인터페이스 — 실제 모터 아님, 확인 절차 생략)")
        return True
    if not sys.stdin.isatty():
        print("확인 입력이 필요합니다. 대화형 터미널에서 실행하세요.")
        return False
    try:
        answer = input(
            "\n로봇 고정·주변 통제·E-stop 준비를 확인했으면 Enter, 취소는 Ctrl-C: "
        )
    except (KeyboardInterrupt, EOFError):
        print("\n취소됨.")
        return False
    if answer.strip():
        print("빈 Enter가 아니므로 취소됨.")
        return False
    return True


def capture_channel(channel, motor_ids, args, rows, lock, notes, barrier, stop_event):
    try:
        bus = can.Bus(channel=channel, interface=args.interface)
    except (OSError, can.CanError) as error:
        with lock:
            notes.append(f"[{channel}] 열기 실패: {error}  "
                         f"(sudo ip link set {channel} up type can bitrate 1000000)")
        stop_event.set()
        barrier.abort()
        return

    motors = {}
    normal_completion = False
    try:
        for motor_id in motor_ids:
            if stop_event.is_set():
                return
            motor = Motor(bus, motor_id, SPECS[MOTOR_MODEL[motor_id]], host_id=args.host_id)
            motors[motor_id] = motor
            motor.stop()
            if stop_event.wait(0.01):
                return
            motor.write_param_u8(RUN_MODE_INDEX, RUN_MODE_OPERATION)
            if stop_event.wait(0.01):
                return
            motor.enable()
            if stop_event.wait(0.01):
                return

        hub = FeedbackHub(bus, motors.values(), host_id=args.host_id)

        try:
            barrier.wait(timeout=5.0)
        except threading.BrokenBarrierError:
            with lock:
                notes.append(f"[{channel}] 다른 채널이 준비 실패해 시작을 취소했습니다.")
            stop_event.set()
            return

        t_end = time.monotonic() + args.duration
        while time.monotonic() < t_end and not stop_event.is_set():
            for motor_id, motor in motors.items():
                if stop_event.is_set():
                    break
                motor.control(pos=0.0, vel=0.0, kp=0.0, kd=args.kd, torque=0.0)
                position = hub.wait_for(motor_id, timeout=args.feedback_timeout)
                t_recv = time.monotonic()
                if position is None:
                    with lock:
                        rows.append((t_recv, motor_id, "", "", "", "", "miss"))
                    continue
                with lock:
                    rows.append((t_recv, motor_id, f"{motor.last_position:.6f}", f"{motor.last_velocity:.6f}",
                                 f"{motor.last_torque:.4f}", f"{motor.last_temp:.1f}", "ok"))
        normal_completion = not stop_event.is_set()
    except (OSError, can.CanError) as error:
        with lock:
            notes.append(f"[{channel}] CAN 오류: {error}")
        stop_event.set()
        barrier.abort()
    except Exception as error:
        with lock:
            notes.append(f"[{channel}] 예기치 않은 오류: {type(error).__name__}: {error}")
        stop_event.set()
        barrier.abort()
    finally:
        if normal_completion and not stop_event.is_set():
            for motor in motors.values():
                if stop_event.is_set():
                    break
                try:
                    active_brake(motor, duration=0.1)
                except Exception:
                    pass
        stop_errors = []
        for motor_id, motor in motors.items():
            try:
                motor.stop()
            except Exception as error:
                stop_errors.append(f"ID {motor_id}: {error}")
        if stop_errors:
            with lock:
                notes.append(f"[{channel}] stop 전송 실패: {'; '.join(stop_errors)}")
        try:
            bus.shutdown()
        except Exception as error:
            with lock:
                notes.append(f"[{channel}] CAN 종료 실패: {type(error).__name__}: {error}")


def save_csv(rows, args, motor_ids):
    os.makedirs(DATA_DIR, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%dT%H%M%S")
    tag = f"_{args.tag}" if args.tag else ""
    fname = f"can_capture{tag}_{ts}.csv"
    path = os.path.join(DATA_DIR, fname)
    with open(path, "w", newline="", encoding="utf-8") as f:
        f.write(f"# feedback_source: {FEEDBACK_SOURCE}\n")
        f.write(f"# motor_ids: {sorted(set(motor_ids))}\n")
        f.write(f"# interface: {args.interface}\n")
        f.write(f"# duration_s: {args.duration}\n")
        f.write(f"# kd: {args.kd}\n")
        f.write(f"# feedback_timeout_s: {args.feedback_timeout}\n")
        f.write(f"# started_at: {datetime.now().isoformat(timespec='seconds')}\n")
        writer = csv.writer(f)
        writer.writerow(["t_monotonic_s", "motor_id", "pos_rad", "vel_rad_s",
                          "torque_Nm", "temp_C", "status"])
        for row in rows:
            writer.writerow(row)
    return path


def main():
    args = parse_args()
    motor_ids = sorted(set(args.motor_id))
    unknown = [m for m in motor_ids if m not in MOTOR_MODEL]
    if unknown:
        print(f"지원하지 않는 motor-id: {unknown}")
        return 1
    problems = validate_args(args, motor_ids)
    if problems:
        print("인자 오류 — 실행하지 않습니다:")
        for problem in problems:
            print(f"  {problem}")
        return 1
    if not confirm(args):
        return 1

    channels = sorted({channel_for_id(m) for m in motor_ids})
    rows = []
    lock = threading.Lock()
    notes = []
    barrier = threading.Barrier(len(channels))
    stop_event = threading.Event()

    threads = [
        threading.Thread(
            target=capture_channel,
            args=(channel, [m for m in motor_ids if channel_for_id(m) == channel],
                  args, rows, lock, notes, barrier, stop_event),
        )
        for channel in channels
    ]
    print(f"\n캡처 시작 ({len(channels)}개 채널, {len(motor_ids)}개 모터, {args.duration:.1f}s)...")
    interrupted = False
    main_error = None
    started_threads = []
    try:
        for t in threads:
            t.start()
            started_threads.append(t)
        while any(t.is_alive() for t in started_threads):
            for t in started_threads:
                t.join(timeout=0.1)
    except KeyboardInterrupt:
        interrupted = True
        print("\n사용자 중단 — 모든 모터에 stop 전송 중...")
    except Exception as error:
        main_error = error
        print(f"\n실행 오류 — 모든 모터에 stop 전송 중: {type(error).__name__}: {error}")
    finally:
        stop_event.set()
        barrier.abort()
        while any(t.is_alive() for t in started_threads):
            try:
                for t in started_threads:
                    t.join(timeout=0.1)
            except KeyboardInterrupt:
                print("안전 종료 중입니다. 실물 이상 동작 시 E-stop을 누르세요.")

    for note in notes:
        print(note)
    if interrupted:
        print("사용자 중단으로 측정 데이터를 저장하지 않습니다.")
        return 130
    if main_error is not None:
        print("실행 오류로 측정 데이터를 저장하지 않습니다.")
        return 1
    if notes:
        print("오류가 발생해 측정 데이터를 저장하지 않습니다.")
        return 1
    if not rows:
        print("샘플이 하나도 기록되지 않았습니다.")
        return 1

    path = save_csv(rows, args, motor_ids)
    n_ok = sum(1 for r in rows if r[-1] == "ok")
    n_miss = sum(1 for r in rows if r[-1] == "miss")
    print(f"저장됨: {path}")
    print(f"  샘플 {len(rows)}개 (ok={n_ok}, miss={n_miss})")
    print("  -> analyze_can_rate.py / analyze_can_noise.py 로 분석하세요.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
