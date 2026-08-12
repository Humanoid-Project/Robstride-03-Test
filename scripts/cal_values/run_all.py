#!/usr/bin/env python3
"""armature + damping + friction 전체를 한 모터에 대해 자동으로 순서대로 반복 측정.

각 항목의 자동화 스크립트(armature/run_sweep.py, damping/run_sweep.py,
friction/run_repeats.py)를 그대로 서브프로세스로 호출한다(로직 중복 없음, 각 항목의
안전장치·자동분석이 원본 그대로 적용됨). 확인 프롬프트는 맨 처음 한 번만 뜨고, 이후
세 항목 모두 --yes로 자동 진행된다 — 중간에 멈추고 싶으면 Ctrl-C(현재 진행 중인 항목이
끝나는 대로 나머지를 건너뛰고 종료).

순서는 armature → damping → friction. 각 항목이 끝날 때마다 그 항목의 analyze_*.py가
자동으로 실행되어 결과가 바로 출력된다.

사용법:
  python3 run_all.py --motor-id 11 --model rs02
  python3 run_all.py --motor-id 12 --model rs02 --repeats 3
  python3 run_all.py --motor-id 11 --model rs02 --items friction damping
"""
import argparse
import os
import subprocess
import sys
import time
from datetime import datetime

CAL_VALUES_DIR = os.path.dirname(os.path.abspath(__file__))
ARMATURE_SWEEP = os.path.join(CAL_VALUES_DIR, "armature", "run_sweep.py")
DAMPING_SWEEP = os.path.join(CAL_VALUES_DIR, "damping", "run_sweep.py")
FRICTION_REPEATS = os.path.join(CAL_VALUES_DIR, "friction", "run_repeats.py")
LOGS_DIR = os.path.join(CAL_VALUES_DIR, "logs")

ITEM_ORDER = ["armature", "damping", "friction"]
ITEM_SCRIPT = {"armature": ARMATURE_SWEEP, "damping": DAMPING_SWEEP, "friction": FRICTION_REPEATS}
ITEM_DATA_DIR = {name: os.path.join(CAL_VALUES_DIR, name, "data") for name in ITEM_ORDER}


def run_and_tee(argv, log_path):
    """서브프로세스를 실행하면서 출력을 화면에 그대로 보여줌과 동시에 log_path에도 저장한다
    (raw byte pass-through라 캡처 스크립트들의 캐리지리턴(\\r) 실시간 갱신도 화면에서는
    똑같이 보인다 — 다만 파일로 저장된 로그를 그냥 cat으로 보면 그 갱신 줄들은 겹쳐 보일 수
    있음, 실제 리포트/요약 줄들은 개행이 있어 정상적으로 남는다)."""
    os.makedirs(os.path.dirname(log_path), exist_ok=True)
    with open(log_path, "wb") as logf:
        proc = subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, bufsize=0)
        try:
            while True:
                chunk = proc.stdout.read(4096)
                if not chunk:
                    break
                sys.stdout.buffer.write(chunk)
                sys.stdout.buffer.flush()
                logf.write(chunk)
                logf.flush()
        except KeyboardInterrupt:
            # 자식도 같은 터미널의 SIGINT를 이미 받아 자체적으로 안전정지(finally: motor.stop())
            # 절차를 밟고 있는 중이다 — SIGTERM으로 덮어씌우면 그 절차가 안 끝나 모터가 계속
            # 도는 채로 남을 수 있으므로 **절대 강제종료하지 않는다.**
            #
            # 자식의 정지 절차는 active_brake(0.3s) + stop() 정도라 길어야 1초 미만이다.
            # 그보다 오래 걸리면 CAN이 이미 죽어서 어차피 정지 명령도 못 나가는 상황이므로,
            # 프로세스를 죽인다고 모터가 멈추지도 않는다 → 죽이는 건 이득 없이 위험만 있다.
            # 사용자에게 물리적 E-stop을 안내하고 계속 기다린다.
            print("\n(자식 프로세스의 자체 안전정지 대기 중...)")
            waited = 0.0
            while True:
                try:
                    proc.wait(timeout=5)
                    break
                except subprocess.TimeoutExpired:
                    waited += 5
                    print(f"  경고: 자식이 {waited:.0f}s째 안 끝납니다. CAN이 죽었을 수 있습니다 — "
                          "모터가 계속 돌고 있으면 **물리 E-stop을 누르세요**. "
                          "(프로세스를 강제로 죽여도 모터는 안 멈춥니다)")
                except KeyboardInterrupt:
                    print("  (계속 대기 중 — 강제종료는 모터를 멈추지 못하므로 하지 않습니다)")
            raise
        finally:
            proc.wait()
        return proc.returncode


def parse_args():
    p = argparse.ArgumentParser(description="armature+damping+friction 전체 자동 반복측정")
    p.add_argument("--motor-id", type=lambda v: int(v, 0), required=True)
    p.add_argument("--model", choices=["rs02", "rs03"], required=True)
    p.add_argument("--repeats", type=int, default=5, help="항목별 반복 횟수, 기본 5")
    p.add_argument("--pause", type=float, default=1.0,
                   help="같은 항목 내 실행 사이 대기시간(초), 기본 1.0 (각 스크립트로 전달됨)")
    p.add_argument("--items", nargs="+", choices=ITEM_ORDER, default=list(ITEM_ORDER),
                   help="실행할 항목들과 순서, 기본 armature damping friction 전부")
    p.add_argument("--channel", default=None)
    p.add_argument("--host-id", default=None)
    p.add_argument("--yes", action="store_true", help="맨 처음 확인 프롬프트도 생략")
    return p.parse_args()


def common_flags(args):
    flags = ["--motor-id", str(args.motor_id), "--model", args.model,
              "--repeats", str(args.repeats), "--pause", str(args.pause), "--yes"]
    if args.channel:
        flags += ["--channel", args.channel]
    if args.host_id:
        flags += ["--host-id", args.host_id]
    return flags


def main():
    args = parse_args()

    print("=" * 70)
    print("전체 자동 반복측정 (armature + damping + friction)")
    print(f"  모터 ID   : {args.motor_id} ({args.model.upper()})")
    print(f"  항목/순서 : {' -> '.join(args.items)}")
    print(f"  반복      : 항목당 {args.repeats}회 (armature/damping은 기본 파라미터 세트 x {args.repeats},")
    print(f"              friction은 방향(+1/-1)당 {args.repeats}회)")
    print(f"  저장 위치 : 각 항목의 cal_values/<항목>/data/ 에 자동 저장")
    print(f"  로그      : cal_values/logs/ 에 항목별 전체 출력 저장(analyze 리포트 포함, 유실 방지)")
    print("  각 항목 내부 확인 프롬프트도 생략되고, 여기서 한 번만 확인합니다.")
    print("  전제: 모터 출력축에 부하 없음, 벤치에 고정된 상태 (세 항목 공통)")
    print("  전체 실행에 수 분 이상 걸릴 수 있습니다 (특히 friction은 안 움직이면 항목당 최대 15초).")
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

    session_ts = datetime.now().strftime("%Y%m%dT%H%M%S")
    results = []
    try:
        for i, name in enumerate(args.items, 1):
            argv = [sys.executable, ITEM_SCRIPT[name]] + common_flags(args)
            log_path = os.path.join(LOGS_DIR,
                                     f"{session_ts}_id{args.motor_id}_{args.model}_{name}.log")
            print(f"\n\n{'#'*70}")
            print(f"# [{i}/{len(args.items)}] {name.upper()} 시작 ({datetime.now().strftime('%H:%M:%S')})")
            print(f"# -> 데이터: {ITEM_DATA_DIR[name]}")
            print(f"# -> 로그  : {log_path}")
            print(f"{'#'*70}")
            rc = run_and_tee(argv, log_path)
            results.append((name, rc, log_path))
            if rc != 0:
                print(f"  참고: {name} 항목이 returncode={rc}로 끝났습니다.")
            if i < len(args.items) and args.pause > 0:
                time.sleep(args.pause)
    except KeyboardInterrupt:
        print(f"\n\n중단됨. {len(results)}/{len(args.items)}개 항목 완료 후 멈춥니다.")

    print(f"\n{'='*70}")
    print("전체 완료")
    for name, rc, log_path in results:
        status = "정상" if rc == 0 else f"이상(returncode={rc})"
        print(f"  {name:<10}: {status}")
        print(f"    로그: {log_path}")
    if len(results) < len(args.items):
        skipped = args.items[len(results):]
        print(f"  건너뜀    : {skipped}")
    print("=" * 70)

    return 0 if all(rc == 0 for _, rc, _ in results) else 1


if __name__ == "__main__":
    sys.exit(main())
