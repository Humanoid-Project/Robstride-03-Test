#!/usr/bin/env python3


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
    p = argparse.ArgumentParser(description="Summarize breakaway measurements.")
    p.add_argument("csv_files", nargs="+", help="CSV paths or glob patterns")
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
        print("Skipped files without a breakaway result:")
        for path, reason in skipped:
            print(f"  {os.path.basename(path)}: stop_reason={reason}")
        print()

    if not groups:
        print("ERROR: No valid breakaway results.")
        return 1

    print("=" * 72)
    print(f"{'motor_id':>8} {'model':>6} {'sign':>5} {'n':>3} {'mean':>12} {'std':>10} {'bracket range':>20}")
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

    motor_ids = sorted({k[0] for k in summary})
    if len(motor_ids) > 1:
        print("\nMotor comparison by model and sign:")
        by_model_sign = defaultdict(dict)
        for (motor_id, model, sign), mean in summary.items():
            by_model_sign[(model, sign)][motor_id] = mean
        for (model, sign), per_motor in sorted(by_model_sign.items()):
            if len(per_motor) > 1:
                vals = list(per_motor.items())
                desc = ", ".join(f"ID{mid}={v:+.4f}" for mid, v in vals)
                spread = max(v for _, v in vals) - min(v for _, v in vals)
                print(f"  {model} sign={sign}: {desc} (range {spread:.4f} N*m)")

    print("\nDirection comparison by motor:")
    by_motor = defaultdict(dict)
    for (motor_id, model, sign), mean in summary.items():
        by_motor[motor_id][sign] = mean
    for motor_id, per_sign in sorted(by_motor.items()):
        if "1" in per_sign and "-1" in per_sign:
            pos_v, neg_v = per_sign["1"], per_sign["-1"]
            asym = abs(abs(pos_v) - abs(neg_v))
            print(f"  ID{motor_id}: positive={pos_v:+.4f}, negative={neg_v:+.4f}, "
                  f"magnitude difference={asym:.4f} N*m")
        else:
            print(f"  ID{motor_id}: only signs {list(per_sign.keys())} were measured")

    return 0


if __name__ == "__main__":
    sys.exit(main())
