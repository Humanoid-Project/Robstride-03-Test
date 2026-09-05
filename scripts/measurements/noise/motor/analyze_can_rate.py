#!/usr/bin/env python3
import argparse
import csv
import glob
import os
import sys
from collections import defaultdict

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from common import channel_for_id

EXPECTED_FEEDBACK_SOURCE = "type_0x02"


def load_rows(path):
    meta = {}
    rows = []
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
    for row in reader:
        rows.append(row)
    return meta, rows


def main():
    p = argparse.ArgumentParser(
        description="Calculate motor feedback rate and polling jitter from motor_noise.py CSV files.")
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
            by_motor[int(row["motor_id"])].append(row)

    if not by_motor:
        print("ERROR: No samples.")
        return 1

    print(f"{'ID':>3} {'ch':>4} {'n':>6} {'ok':>6} {'miss':>5} {'ok%':>6} "
          f"{'mean Hz':>8} {'mean dt':>10} {'dt std':>14} {'max dt':>10}")
    print("-" * 92)
    per_channel_ok = defaultdict(int)
    per_channel_span = {}
    for motor_id in sorted(by_motor):
        rows = sorted(by_motor[motor_id], key=lambda r: float(r["t_monotonic_s"]))
        n = len(rows)
        ok_rows = [r for r in rows if r["status"] == "ok"]
        n_ok = len(ok_rows)
        n_miss = n - n_ok
        ch = channel_for_id(motor_id)
        per_channel_ok[ch] += n_ok
        if rows:
            span = float(rows[-1]["t_monotonic_s"]) - float(rows[0]["t_monotonic_s"])
            lo, hi = per_channel_span.get(ch, (float("inf"), float("-inf")))
            per_channel_span[ch] = (min(lo, float(rows[0]["t_monotonic_s"])),
                                     max(hi, float(rows[-1]["t_monotonic_s"])))
        if n_ok >= 2:
            ts = [float(r["t_monotonic_s"]) for r in ok_rows]
            dts = [b - a for a, b in zip(ts, ts[1:])]
            mean_dt = sum(dts) / len(dts)
            var = sum((d - mean_dt) ** 2 for d in dts) / len(dts)
            std_dt = var ** 0.5
            max_dt = max(dts)
            hz = 1.0 / mean_dt if mean_dt > 0 else float("nan")
            print(f"{motor_id:>3} {ch:>4} {n:>6} {n_ok:>6} {n_miss:>5} "
                  f"{100.0*n_ok/n:>5.1f}% {hz:>8.1f} {mean_dt*1000:>10.3f} "
                  f"{std_dt*1000:>14.3f} {max_dt*1000:>10.3f}")
        else:
            print(f"{motor_id:>3} {ch:>4} {n:>6} {n_ok:>6} {n_miss:>5}  "
                  f"(fewer than 2 valid samples)")

    print("\nChannel throughput:")
    for ch in sorted(per_channel_ok):
        lo, hi = per_channel_span[ch]
        span = hi - lo
        if span > 0:
            print(f"  {ch}: {per_channel_ok[ch]} valid samples / {span:.2f}s "
                  f"= {per_channel_ok[ch]/span:.1f} Hz total")
    print("Per-motor update rate decreases as more motors share a channel.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
