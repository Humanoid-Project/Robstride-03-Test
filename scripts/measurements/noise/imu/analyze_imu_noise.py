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
        description="Calculate per-axis gyroscope bias and noise from imu_noise.py CSV files.")
    p.add_argument("csv_files", nargs="+", help="CSV paths or glob patterns")
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
        print("ERROR: No samples.")
        return 1

    print(f"Samples: {len(rows)} from {len(paths)} files")
    print(f"{'signal':<12} {'axis':>4} {'bias(mean, rad/s)':>20} {'noise(std, rad/s)':>20}")
    print("-" * 78)
    for label, cols in (("fused (AHRS)", FUSED_COLS), ("raw", RAW_COLS)):
        for axis in AXES:
            values = [float(r[cols[axis]]) for r in rows]
            mean, std = stats(values)
            print(f"{label:<12} {axis:>4} {mean:>+20.6f} {std:>20.6f}")

    fused_std = [stats([float(r[FUSED_COLS[a]]) for r in rows])[1] for a in AXES]
    lo, hi = REFERENCE_DESK_NOISE_RAD_S
    worst = max(fused_std)
    print(f"Reference stationary noise: {lo} to {hi} rad/s.")
    if worst > hi:
        ratio = worst / hi
        print(f"WARNING: Maximum axis noise is {worst:.6f} rad/s ({ratio:.1f}x reference).")
    else:
        print(f"Maximum axis noise is within the reference range: {worst:.6f} rad/s.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
