#!/usr/bin/env python3
"""항목 4 (정지마찰) 분석 — capture_breakaway.py 결과 CSV들을 모아 정리한다.

각 CSV 헤더에 이미 그 실행의 breakaway 브래킷/추정값이 들어있으므로(계단 탐색이라
armature/damping처럼 회귀가 필요 없음), 여기서는 그냥 모아서 (motor_id, sign)별로
묶고, 같은 조건이 여러 번 있으면 평균/표준편차로 재현성을 보여준다.

사용법:
    python3 analyze_breakaway.py data/id11_rs02_*.csv data/id12_rs02_*.csv
"""
import argparse
import glob
import os
import sys
from collections import defaultdict


def load_meta(path):
    meta = {}
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            if not line.startswith("#"):
                break
            key, _, val = line[1:].partition(":")
            meta[key.strip()] = val.strip()
    return meta


def main():
    p = argparse.ArgumentParser(description="breakaway 측정 CSV들을 모아 정리")
    p.add_argument("csv_files", nargs="+", help="capture_breakaway.py 출력 CSV들 (glob 가능)")
    args = p.parse_args()

    paths = []
    for pattern in args.csv_files:
        matched = sorted(glob.glob(pattern))
        paths.extend(matched if matched else [pattern])
    paths = list(dict.fromkeys(paths))

    groups = defaultdict(list)
    skipped = []
    for path in paths:
        meta = load_meta(path)
        if meta.get("stop_reason") != "movement_detected" or not meta.get("breakaway_mid_Nm"):
            skipped.append((path, meta.get("stop_reason", "?")))
            continue
        key = (meta.get("motor_id"), meta.get("model"), meta.get("sign"))
        groups[key].append({
            "path": path,
            "low": float(meta["breakaway_low_Nm"]),
            "high": float(meta["breakaway_high_Nm"]),
            "mid": float(meta["breakaway_mid_Nm"]),
        })

    if skipped:
        print("건너뜀 (breakaway 못 찾음):")
        for path, reason in skipped:
            print(f"  {os.path.basename(path)}: stop_reason={reason}")
        print()

    if not groups:
        print("유효한 breakaway 측정 결과가 없습니다.")
        return 1

    print("=" * 72)
    print(f"{'motor_id':>8} {'model':>6} {'sign':>5} {'n':>3} {'mid 평균':>12} {'표준편차':>10} {'브래킷 범위':>20}")
    print("-" * 72)
    summary = {}
    for (motor_id, model, sign), entries in sorted(groups.items()):
        mids = [e["mid"] for e in entries]
        mean = sum(mids) / len(mids)
        std = (sum((m - mean) ** 2 for m in mids) / len(mids)) ** 0.5 if len(mids) > 1 else 0.0
        lo = min(e["low"] for e in entries)
        hi = max(e["high"] for e in entries)
        summary[(motor_id, model, sign)] = mean
        print(f"{motor_id:>8} {model:>6} {sign:>5} {len(entries):>3} {mean:>+12.4f} {std:>10.4f} "
              f"[{lo:>+.3f}, {hi:>+.3f}]")
        for e in entries:
            print(f"    - {os.path.basename(e['path'])}: mid={e['mid']:+.4f}  "
                  f"bracket=[{e['low']:+.3f}, {e['high']:+.3f}]")
    print("=" * 72)

    # 개체(motor_id)간 비교
    motor_ids = sorted({k[0] for k in summary})
    if len(motor_ids) > 1:
        print("\n개체간 비교 (같은 model, 같은 sign 기준):")
        by_model_sign = defaultdict(dict)
        for (motor_id, model, sign), mean in summary.items():
            by_model_sign[(model, sign)][motor_id] = mean
        for (model, sign), per_motor in sorted(by_model_sign.items()):
            if len(per_motor) > 1:
                vals = list(per_motor.items())
                desc = ", ".join(f"ID{mid}={v:+.4f}" for mid, v in vals)
                spread = max(v for _, v in vals) - min(v for _, v in vals)
                print(f"  {model} sign={sign}: {desc}  (범위 {spread:.4f} N*m)")

    # 방향간 비교 (같은 motor_id 기준)
    print("\n방향간 비교 (같은 motor_id 기준):")
    by_motor = defaultdict(dict)
    for (motor_id, model, sign), mean in summary.items():
        by_motor[motor_id][sign] = mean
    for motor_id, per_sign in sorted(by_motor.items()):
        if "1" in per_sign and "-1" in per_sign:
            pos_v, neg_v = per_sign["1"], per_sign["-1"]
            asym = abs(abs(pos_v) - abs(neg_v))
            print(f"  ID{motor_id}: +방향={pos_v:+.4f}  -방향={neg_v:+.4f}  "
                  f"(크기차 {asym:.4f} N*m)")
        else:
            print(f"  ID{motor_id}: 방향이 하나만 측정됨({list(per_sign.keys())}) — 반대방향도 재보면 좋음")

    return 0


if __name__ == "__main__":
    sys.exit(main())
