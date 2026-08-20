#!/usr/bin/env python3
import argparse
import csv
import glob
import math
import os
import sys
from collections import defaultdict

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common import JOINT_MAP


def load_rows(path):
    meta = {}
    with open(path, "r", encoding="utf-8", newline="") as f:
        lines = f.readlines()
    body_start = 0
    for i, line in enumerate(lines):
        if line.startswith("#"):
            key, _, val = line[1:].partition(":")
            meta[key.strip()] = val.strip()
            body_start = i + 1
        else:
            break
    reader = csv.DictReader(lines[body_start:])
    return meta, list(reader)


def stats(values):
    n = len(values)
    if n == 0:
        return None
    mean = sum(values) / n
    var = sum((v - mean) ** 2 for v in values) / n if n > 1 else 0.0
    std = var ** 0.5
    return {
        "n": n, "mean": mean, "std": std,
        "min": min(values), "max": max(values),
        "pp": max(values) - min(values),
    }


def main():
    p = argparse.ArgumentParser(
        description="can_capture.py 출력 CSV를 모아 정지 상태의 pos/vel 노이즈 통계(평균/표준편차/"
                     "최대-최소)를 계산한다 (project-open-items #10: RL observation noise model 근거).")
    p.add_argument("csv_files", nargs="+", help="can_capture.py 출력 CSV들 (glob 가능)")
    args = p.parse_args()

    paths = []
    for pattern in args.csv_files:
        matched = sorted(glob.glob(pattern))
        paths.extend(matched if matched else [pattern])
    paths = list(dict.fromkeys(paths))

    by_motor = defaultdict(list)
    for path in paths:
        _, rows = load_rows(path)
        for row in rows:
            if row["status"] != "ok":
                continue
            by_motor[int(row["motor_id"])].append(row)

    if not by_motor:
        print("유효한(status=ok) 샘플이 없습니다.")
        return 1

    print("=" * 104)
    print(f"{'ID':>3} {'joint':<18} {'n':>6} "
          f"{'pos평균(rad)':>13} {'pos표준편차(rad)':>16} {'pos p-p(rad)':>13} "
          f"{'vel표준편차(rad/s)':>18}")
    print("-" * 104)
    for motor_id in sorted(by_motor):
        rows = by_motor[motor_id]
        pos = stats([float(r["pos_rad"]) for r in rows])
        vel = stats([float(r["vel_rad_s"]) for r in rows])
        joint = JOINT_MAP.get(motor_id, "?")
        print(f"{motor_id:>3} {joint:<18} {pos['n']:>6} "
              f"{pos['mean']:>+13.6f} {pos['std']:>16.6f} {pos['pp']:>13.6f} "
              f"{vel['std']:>18.6f}")
        drift_flag = "  ⚠ 첫/끝 표본 간 pos 차이가 표준편차의 5배 이상 — 진짜 정지였는지 확인 필요"
        first_last_gap = abs(float(rows[-1]["pos_rad"]) - float(rows[0]["pos_rad"]))
        if pos["std"] > 0 and first_last_gap > 5 * pos["std"]:
            print(drift_flag)
    print("=" * 104)
    print("\n주의: 이 표준편차는 순수 encoder/신호 노이즈만이 아니라 CAN 프레임 타이밍 지터, "
          "정지마찰 근처의 미세한 안착(settling) 움직임이 섞인 값이다. RL observation noise cfg에 "
          "넣기 전에 관절별로 이상치(비정상적으로 큰 std)가 없는지 먼저 확인할 것.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
