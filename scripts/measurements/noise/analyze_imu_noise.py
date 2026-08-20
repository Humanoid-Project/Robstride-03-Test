#!/usr/bin/env python3
import argparse
import csv
import glob
import os
import sys
from collections import defaultdict

AXES = ("x", "y", "z")
FUSED_COLS = {"x": "gyro_x", "y": "gyro_y", "z": "gyro_z"}
RAW_COLS = {"x": "gyro_raw_x", "y": "gyro_raw_y", "z": "gyro_raw_z"}

REFERENCE_DESK_NOISE_RAD_S = (0.001, 0.002)  # hw-imu-n100.md: 책상 위 정지 상태 참고값


def load_rows(path):
    with open(path, "r", encoding="utf-8", newline="") as f:
        lines = f.readlines()
    body_start = 0
    for i, line in enumerate(lines):
        if not line.startswith("#"):
            body_start = i
            break
    reader = csv.DictReader(lines[body_start:])
    return list(reader)


def stats(values):
    n = len(values)
    mean = sum(values) / n
    var = sum((v - mean) ** 2 for v in values) / n if n > 1 else 0.0
    return mean, var ** 0.5


def main():
    p = argparse.ArgumentParser(
        description="imu_capture.py 출력 CSV를 모아 gyro bias(평균)/noise(표준편차)를 축별로 "
                     "계산한다 (project-open-items #12: RL observation noise cfg 근거).")
    p.add_argument("csv_files", nargs="+", help="imu_capture.py 출력 CSV들 (glob 가능)")
    args = p.parse_args()

    paths = []
    for pattern in args.csv_files:
        matched = sorted(glob.glob(pattern))
        paths.extend(matched if matched else [pattern])
    paths = list(dict.fromkeys(paths))

    rows = []
    for path in paths:
        rows.extend(load_rows(path))

    if not rows:
        print("샘플이 없습니다.")
        return 1

    print(f"총 샘플 {len(rows)}개 ({len(paths)}개 파일)\n")
    print("=" * 78)
    print(f"{'signal':<12} {'axis':>4} {'bias(mean, rad/s)':>20} {'noise(std, rad/s)':>20}")
    print("-" * 78)
    for label, cols in (("fused (AHRS)", FUSED_COLS), ("raw", RAW_COLS)):
        for axis in AXES:
            values = [float(r[cols[axis]]) for r in rows]
            mean, std = stats(values)
            print(f"{label:<12} {axis:>4} {mean:>+20.6f} {std:>20.6f}")
    print("=" * 78)

    fused_std = [stats([float(r[FUSED_COLS[a]]) for r in rows])[1] for a in AXES]
    lo, hi = REFERENCE_DESK_NOISE_RAD_S
    worst = max(fused_std)
    print(f"\n참고: hw-imu-n100.md 기록상 책상 위 정지 노이즈는 {lo}~{hi} rad/s 수준이었음.")
    if worst > hi:
        ratio = worst / hi
        print(f"  이번 측정 최대 축 표준편차 {worst:.6f} rad/s 는 참고값의 약 {ratio:.1f}배 — "
              "장착 전 허공에 매달린 상태(3~7배)였던 패턴과 비슷한지, 아니면 브래킷 장착 후에도 "
              "여전히 흔들림이 남아있는 것인지 확인 필요.")
    else:
        print(f"  이번 측정 최대 축 표준편차 {worst:.6f} rad/s 는 참고 범위 안 — 브래킷 장착이 "
              "흔들림 문제를 해결한 것으로 보임.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
