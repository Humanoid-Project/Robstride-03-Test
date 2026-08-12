#!/usr/bin/env python3
"""armature 측정 자동화 — capture_torque_step.py를 여러 토크값 x N회 반복 실행하고
끝나면 analyze_armature.py까지 자동으로 돌린다.

내부적으로 capture_torque_step.py를 그대로 서브프로세스로 반복 호출한다(로직 중복
없음, 안전장치도 원본 그대로 적용됨). 확인 프롬프트는 맨 처음 한 번만 뜨고, 이후 각
실행은 --yes로 넘어간다 — 중간에 멈추고 싶으면 Ctrl-C.

사용법:
  python3 run_sweep.py --motor-id 11 --model rs02
  python3 run_sweep.py --motor-id 12 --model rs02 --repeats 2
  python3 run_sweep.py --motor-id 11 --model rs02 --torques 0.3 0.5 1.0 1.5 2.0
"""
import argparse
import os
import subprocess
import sys
import time
from datetime import datetime

ARMATURE_DIR = os.path.dirname(os.path.abspath(__file__))
CAPTURE = os.path.join(ARMATURE_DIR, "capture_torque_step.py")
ANALYZE = os.path.join(ARMATURE_DIR, "analyze_armature.py")
DATA_DIR = os.path.join(ARMATURE_DIR, "data")

# 2026-08-09 RS02 ID11/ID12 측정에 실제로 쓴 세트. RS03은 아직 실측 전이라 상대적 비율로
# 추정한 값(정격 20N*m의 5~40%) — 필요하면 --torques로 얼마든지 바꿀 것.
DEFAULT_TORQUES = {
    "rs02": [0.3, 0.5, 1.0, 1.5, 2.0],
    "rs03": [1.0, 2.0, 4.0, 6.0, 8.0],
}


def parse_args():
    p = argparse.ArgumentParser(description="armature 측정 자동화 (여러 토크값 x N회)")
    p.add_argument("--motor-id", type=lambda v: int(v, 0), required=True)
    p.add_argument("--model", choices=list(DEFAULT_TORQUES.keys()), required=True)
    p.add_argument("--torques", type=float, nargs="+", default=None,
                   help="사용할 토크값들(N*m), 기본값: 모델별 권장 세트")
    p.add_argument("--repeats", type=int, default=1, help="각 토크값당 반복 횟수, 기본 1")
    p.add_argument("--pause", type=float, default=1.0, help="실행 사이 대기시간(초), 기본 1.0")
    p.add_argument("--channel", default=None)
    p.add_argument("--host-id", default=None)
    # capture_torque_step.py로 그대로 넘길 튜닝 옵션들 (지정 안 하면 그쪽 기본값 사용)
    p.add_argument("--max-vel", type=float, default=None)
    p.add_argument("--max-time", type=float, default=None)
    p.add_argument("--max-turns", type=float, default=None)
    p.add_argument("--poll-timeout", type=float, default=None)
    p.add_argument("--rate", type=float, default=None)
    p.add_argument("--feedback-timeout", type=float, default=None)
    p.add_argument("--no-analyze", action="store_true", help="끝나고 analyze_armature.py 자동 실행 생략")
    p.add_argument("--yes", action="store_true", help="맨 처음 확인 프롬프트도 생략")
    return p.parse_args()


def build_argv(args, torque):
    argv = [sys.executable, CAPTURE,
            "--motor-id", str(args.motor_id),
            "--model", args.model,
            "--torque", str(torque),
            "--yes"]
    if args.channel:
        argv += ["--channel", args.channel]
    if args.host_id:
        argv += ["--host-id", args.host_id]
    passthrough = [
        ("--max-vel", args.max_vel), ("--max-time", args.max_time),
        ("--max-turns", args.max_turns), ("--poll-timeout", args.poll_timeout),
        ("--rate", args.rate), ("--feedback-timeout", args.feedback_timeout),
    ]
    for name, value in passthrough:
        if value is not None:
            argv += [name, str(value)]
    return argv


def main():
    args = parse_args()
    torques = args.torques if args.torques is not None else DEFAULT_TORQUES[args.model]
    plan = [t for t in torques for _ in range(args.repeats)]

    print("=" * 70)
    print("armature 측정 자동화")
    print(f"  모터 ID       : {args.motor_id} ({args.model.upper()})")
    print(f"  토크값        : {torques}  x  {args.repeats}회 = 총 {len(plan)}회 실행")
    print(f"  실행 사이 대기: {args.pause}s")
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
        for i, torque in enumerate(plan, 1):
            print(f"\n{'='*70}")
            print(f"[{i}/{len(plan)}] ID{args.motor_id} torque={torque:+.3f} N*m "
                  f"({datetime.now().strftime('%H:%M:%S')})")
            print("=" * 70)
            argv = build_argv(args, torque)
            proc = subprocess.run(argv)
            results.append((torque, proc.returncode))
            if proc.returncode != 0:
                print(f"  참고: 이번 실행이 returncode={proc.returncode}로 끝났습니다.")
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
        print(f"\nanalyze_armature.py 실행 ({pattern})...\n")
        subprocess.run([sys.executable, ANALYZE, pattern])

    return 0


if __name__ == "__main__":
    sys.exit(main())
