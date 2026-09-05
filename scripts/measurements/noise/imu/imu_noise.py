#!/usr/bin/env python3
import argparse
import csv
import os
import sys
import time
from datetime import datetime
from pathlib import Path

from robonex_common.imu import DEFAULT_IMU_BAUDRATE as DEFAULT_BAUD
from robonex_common.imu import DEFAULT_IMU_PORT as DEFAULT_PORT
from robonex_common.imu import MOUNT_ROLL_DEG
DEFAULT_DURATION = 60.0
DEG = 3.141592653589793 / 180.0

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")


def parse_args():
    p = argparse.ArgumentParser(
        description="Capture raw and fused N100 gyroscope samples while stationary.")
    p.add_argument("--port", default=DEFAULT_PORT)
    p.add_argument("--duration", type=float, default=DEFAULT_DURATION,
                   help="Capture time in seconds")
    p.set_defaults(
        baud=DEFAULT_BAUD,
        n100_dir=str(Path(__file__).resolve().parent),
    )
    return p.parse_args()


def main():
    args = parse_args()

    sys.path.insert(0, args.n100_dir)
    try:
        import n100
    except ImportError as e:
        print(f"ERROR: Failed to import n100: {e}")
        print(f"  cd {args.n100_dir} && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release "
              f"&& cmake --build build -j")
        return 1

    print(f"Capturing IMU data from {args.port} at {args.baud} baud for {args.duration:.1f}s.")
    print("Keep the IMU completely stationary.")

    driver = n100.ImuDriver(n100.DriverConfig(
        port=args.port, baudrate=args.baud,
        mount_rotation=n100.Quat.from_axis_angle_x(MOUNT_ROLL_DEG * DEG),
    ))
    try:
        driver.start()
    except RuntimeError as e:
        print(f"ERROR: Failed to start the IMU: {e}")
        return 1

    first = driver.wait_for_sample(timeout=3.0)
    if first is None:
        print(f"ERROR: No IMU sample within 3 seconds: {driver.last_error() or 'unknown error'}")
        driver.stop()
        return 1

    rows = []
    last_seq = first.seq
    t0 = time.monotonic()
    t_end = t0 + args.duration
    n = 0
    try:
        while time.monotonic() < t_end:
            sample = driver.wait_for_sample(timeout=0.5, last_seq=last_seq)
            if sample is None:
                continue
            last_seq = sample.seq
            n += 1
            w, wr = sample.angular_velocity, sample.angular_velocity_raw
            a = sample.linear_acceleration
            rows.append((
                time.monotonic() - t0, sample.seq,
                f"{w.x:.6f}", f"{w.y:.6f}", f"{w.z:.6f}",
                f"{wr.x:.6f}", f"{wr.y:.6f}", f"{wr.z:.6f}",
                f"{a.x:.4f}", f"{a.y:.4f}", f"{a.z:.4f}",
                f"{sample.imu_temperature:.2f}",
            ))
            if n % 50 == 0:
                elapsed = time.monotonic() - t0
                print(f"\rSamples: {n}, elapsed: {elapsed:5.1f}/{args.duration:.1f}s", end="", flush=True)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        drv_stats = driver.stats()
        driver.stop()

    print()
    if not rows:
        print("ERROR: No samples recorded.")
        return 1

    os.makedirs(DATA_DIR, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%dT%H%M%S")
    fname = f"imu_noise_{ts}.csv"
    path = os.path.join(DATA_DIR, fname)
    with open(path, "w", newline="", encoding="utf-8") as f:
        f.write(f"# port: {args.port}\n")
        f.write(f"# baud: {args.baud}\n")
        f.write(f"# duration_s: {args.duration}\n")
        f.write(f"# mount_roll_deg: {MOUNT_ROLL_DEG}\n")
        f.write(f"# samples: {len(rows)}\n")
        f.write(f"# driver_stats: {drv_stats}\n")
        f.write(f"# started_at: {datetime.now().isoformat(timespec='seconds')}\n")
        writer = csv.writer(f)
        writer.writerow(["t_s", "seq", "gyro_x", "gyro_y", "gyro_z",
                          "gyro_raw_x", "gyro_raw_y", "gyro_raw_z",
                          "accel_x", "accel_y", "accel_z", "temp_C"])
        for row in rows:
            writer.writerow(row)

    print(f"Saved: {path} ({len(rows)} samples, stats={drv_stats})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
