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
        description="can_capture.py 출력 CSV를 모아 모터별 달성 Hz / 폴링 지터를 계산한다 "
                     "(project-open-items #9: control-loop 주파수 상한).")
    p.add_argument("csv_files", nargs="+", help="can_capture.py 출력 CSV들 (glob 가능)")
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
            print(f"지원하지 않는 피드백 출처: {path} "
                  f"(expected={EXPECTED_FEEDBACK_SOURCE}, got={source or '메타데이터 없음'})")
            return 1
        for row in rows:
            by_motor[int(row["motor_id"])].append(row)

    if not by_motor:
        print("샘플이 없습니다.")
        return 1

    print("=" * 92)
    print(f"{'ID':>3} {'ch':>4} {'n':>6} {'ok':>6} {'miss':>5} {'ok%':>6} "
          f"{'평균Hz':>8} {'평균dt(ms)':>10} {'dt표준편차(ms)':>14} {'최대dt(ms)':>10}")
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
                  f"(ok 샘플이 2개 미만이라 Hz 계산 불가)")
    print("=" * 92)

    print("\n채널별 실제 처리량 (그 채널에 물린 모든 모터가 순번대로 도는 실측 throughput):")
    for ch in sorted(per_channel_ok):
        lo, hi = per_channel_span[ch]
        span = hi - lo
        if span > 0:
            print(f"  {ch}: ok 합계 {per_channel_ok[ch]}개 / {span:.2f}s "
                  f"= {per_channel_ok[ch]/span:.1f} Hz (해당 채널 모터 전체 합산)")
    print("\n주의: 이 값은 request/response 왕복(motor.control()+poll_feedback())의 실측 상한이며, "
          "채널 하나에 물린 모터 수만큼 개별 모터의 갱신 주기는 늘어난다 "
          "(예: 채널당 6개면 개별 모터 Hz ≈ 채널 throughput/6). "
          "sim.dt/decimation은 이 실측값보다 여유를 두고 정해야 한다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
