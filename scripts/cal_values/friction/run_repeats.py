#!/usr/bin/env python3
"""breakaway 반복측정 자동화 — capture_breakaway.py를 방향(+1/-1)별로 여러 번 실행하고
끝나면 analyze_breakaway.py까지 자동으로 돌린다.

내부적으로 capture_breakaway.py를 그대로 서브프로세스로 반복 호출한다(로직 중복 없음,
안전장치도 원본 그대로 적용됨). 확인 프롬프트는 맨 처음 한 번만 뜨고, 이후 각 실행은
--yes로 넘어간다 — 중간에 멈추고 싶으면 Ctrl-C.

사용법:
  python3 run_repeats.py --motor-id 12 --model rs02
  python3 run_repeats.py --motor-id 12 --model rs02 --repeats 5 --signs 1
  python3 run_repeats.py --motor-id 11 --model rs02 --repeats 3 --pause 2.0
"""
import argparse
import os
import subprocess
import sys
import time
from datetime import datetime

FRICTION_DIR = os.path.dirname(os.path.abspath(__file__))
CAPTURE = os.path.join(FRICTION_DIR, "capture_breakaway.py")
ANALYZE = os.path.join(FRICTION_DIR, "analyze_breakaway.py")
DATA_DIR = os.path.join(FRICTION_DIR, "data")


def parse_args():
    p = argparse.ArgumentParser(description="breakaway 반복측정 자동화 (+1/-1 방향별 N회)")
    p.add_argument("--motor-id", type=lambda v: int(v, 0), required=True)
    p.add_argument("--model", choices=["rs02", "rs03"], required=True)
    p.add_argument("--signs", type=int, nargs="+", choices=[1, -1], default=[1, -1],
                   help="측정할 방향들, 기본 +1과 -1 둘 다")
    p.add_argument("--repeats", type=int, default=3, help="방향당 반복 횟수, 기본 3")
    p.add_argument("--pause", type=float, default=1.0, help="실행 사이 대기시간(초), 기본 1.0")
    p.add_argument("--channel", default=None)
    p.add_argument("--host-id", default=None)
    # capture_breakaway.py로 그대로 넘길 튜닝 옵션들 (지정 안 하면 그쪽 기본값 사용)
    p.add_argument("--start-torque", type=float, default=None)
    p.add_argument("--step", type=float, default=None)
    p.add_argument("--probe-time", type=float, default=None)
    p.add_argument("--move-threshold", type=float, default=None)
    p.add_argument("--max-torque", type=float, default=None)
    p.add_argument("--max-vel", type=float, default=None)
    p.add_argument("--max-time", type=float, default=None)
    p.add_argument("--feedback-timeout", type=float, default=None)
    p.add_argument("--no-analyze", action="store_true", help="끝나고 analyze_breakaway.py 자동 실행 생략")
    p.add_argument("--yes", action="store_true", help="맨 처음 확인 프롬프트도 생략")
    return p.parse_args()


def build_argv(args, sign):
    argv = [sys.executable, CAPTURE,
            "--motor-id", str(args.motor_id),
            "--model", args.model,
            "--sign", str(sign),
            "--yes"]
    if args.channel:
        argv += ["--channel", args.channel]
    if args.host_id:
        argv += ["--host-id", args.host_id]
    passthrough = [
        ("--start-torque", args.start_torque), ("--step", args.step),
        ("--probe-time", args.probe_time), ("--move-threshold", args.move_threshold),
        ("--max-torque", args.max_torque), ("--max-vel", args.max_vel),
        ("--max-time", args.max_time), ("--feedback-timeout", args.feedback_timeout),
    ]
    for name, value in passthrough:
        if value is not None:
            argv += [name, str(value)]
    return argv


def main():
    args = parse_args()
    plan = [sign for sign in args.signs for _ in range(args.repeats)]
    max_torque_est = args.max_torque if args.max_torque is not None else 1.0
    step_est = args.step if args.step is not None else 0.02
    probe_est = args.probe_time if args.probe_time is not None else 0.3
    worst_per_run = (max_torque_est / step_est) * probe_est
    worst_total = worst_per_run * len(plan) + args.pause * (len(plan) - 1)

    print("=" * 70)
    print("breakaway 반복측정 자동화")
    print(f"  모터 ID       : {args.motor_id} ({args.model.upper()})")
    print(f"  방향          : {args.signs}  x  {args.repeats}회 = 총 {len(plan)}회 실행")
    print(f"  실행 사이 대기: {args.pause}s")
    print(f"  각 실행 최악값: ~{worst_per_run:.1f}s (안 움직이면) → 전체 최악 ~{worst_total:.0f}s")
    print("  각 실행 자체의 확인 프롬프트는 생략(--yes)되고, 여기서 한 번만 확인합니다.")
    print("=" * 70)
    if not args.yes:
        if not sys.stdin.isatty():
            print("확인 입력이 필요합니다. --yes 로 실행하거나 대화형 터미널에서 실행하세요.")
            return 1
        try:
            input("\n시작하려면 Enter, 취소하려면 Ctrl-C: ")
        except (KeyboardInterrupt, EOFError):
            print("\n취소됨.")
            return 0

    results = []
    try:
        for i, sign in enumerate(plan, 1):
            print(f"\n{'='*70}")
            print(f"[{i}/{len(plan)}] ID{args.motor_id} sign={sign:+d} "
                  f"({datetime.now().strftime('%H:%M:%S')})")
            print("=" * 70)
            argv = build_argv(args, sign)
            proc = subprocess.run(argv)
            results.append((sign, proc.returncode))
            if proc.returncode != 0:
                print(f"  참고: 이번 실행이 returncode={proc.returncode}로 끝났습니다 "
                      "(오류이거나 max-torque까지 안 움직인 경우일 수 있음).")
            if i < len(plan) and args.pause > 0:
                time.sleep(args.pause)
    except KeyboardInterrupt:
        print(f"\n\n중단됨. {len(results)}/{len(plan)}개 실행 완료 후 멈춥니다.")

    print(f"\n{'='*70}")
    print(f"완료: {len(results)}개 실행 ({sum(1 for _, rc in results if rc == 0)}개 정상, "
          f"{sum(1 for _, rc in results if rc != 0)}개 이상)")
    print("=" * 70)

    if results and not args.no_analyze:
        pattern = os.path.join(DATA_DIR, f"id{args.motor_id}_{args.model}_*.csv")
        print(f"\nanalyze_breakaway.py 실행 ({pattern})...\n")
        subprocess.run([sys.executable, ANALYZE, pattern])

    return 0


if __name__ == "__main__":
    sys.exit(main())
