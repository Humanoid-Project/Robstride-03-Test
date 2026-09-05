#!/usr/bin/env python3
import argparse
import csv
import glob
import math
import os
import sys
from collections import defaultdict

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from common import JOINT_MAP

EXPECTED_FEEDBACK_SOURCE = "type_0x02"


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
        description="Calculate stationary motor position and velocity noise from motor_noise.py CSV files.")
    p.add_argument("csv_files", nargs="+", help="CSV paths or glob patterns")
    args = p.parse_args()

    paths = []
    for pattern in args.csv_files:
        matched = sorted(glob.glob(pattern))
        paths.extend(matched if matched else [pattern])
    paths = list(dict.fromkeys(paths))

    by_motor = defaultdict(list)
    for path in paths:
        meta, rows = load_rows(path)
        source = meta.get("feedback_source")
        if source != EXPECTED_FEEDBACK_SOURCE:
            print(f"ERROR: Unsupported feedback source in {path} "
                  f"(expected={EXPECTED_FEEDBACK_SOURCE}, got={source or 'missing'})")
            return 1
        for row in rows:
            if row["status"] != "ok":
                continue
            by_motor[int(row["motor_id"])].append(row)

    if not by_motor:
        print("ERROR: No valid samples.")
        return 1

    print(f"{'ID':>3} {'joint':<18} {'n':>6} "
          f"{'pos mean':>13} {'pos std':>16} {'pos p-p':>13} {'vel std':>18}")
    print("-" * 104)
    for motor_id in sorted(by_motor):
        rows = by_motor[motor_id]
        pos = stats([float(r["pos_rad"]) for r in rows])
        vel = stats([float(r["vel_rad_s"]) for r in rows])
        joint = JOINT_MAP.get(motor_id, "?")
        print(f"{motor_id:>3} {joint:<18} {pos['n']:>6} "
              f"{pos['mean']:>+13.6f} {pos['std']:>16.6f} {pos['pp']:>13.6f} "
              f"{vel['std']:>18.6f}")
        drift_flag = "  WARNING: Position drift exceeds 5x the standard deviation."
        first_last_gap = abs(float(rows[-1]["pos_rad"]) - float(rows[0]["pos_rad"]))
        if pos["std"] > 0 and first_last_gap > 5 * pos["std"]:
            print(drift_flag)
    print("Noise includes encoder noise, CAN timing jitter, and physical settling motion.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
