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
DEFAULT_DURATION = 10.0
DEG = 3.141592653589793 / 180.0

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")


def parse_args():
    p = argparse.ArgumentParser(
        description="IMU(N100)가 완전히 정지한 상태에서 gyro(fused/raw) 샘플을 원시 기록한다. "
                     "분석은 analyze_imu_noise.py가 담당한다 (project-open-items #12: "
                     "RL observation noise cfg 근거).")
    p.add_argument("--port", default=DEFAULT_PORT)
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    p.add_argument("--duration", type=float, default=DEFAULT_DURATION,
                   help="측정 시간(초), 기본 10")
    p.add_argument("--n100-dir",
                   default=str(Path(__file__).resolve().parents[3]
                               / "motor_control" / "motor_with_imu_test"),
                   help="n100*.so 가 있는 폴더")
    p.add_argument("--tag", default="", help="출력 CSV 파일명에 붙일 태그")
    return p.parse_args()


def main():
    args = parse_args()

    sys.path.insert(0, args.n100_dir)
    try:
        import n100
    except ImportError as e:
        print(f"n100 모듈 임포트 실패: {e}")
        print(f"  cd {args.n100_dir} && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release "
              f"&& cmake --build build -j")
        print("  경로가 다르면 --n100-dir 로 지정")
        return 1

    print("=" * 70)
    print("IMU gyro bias/noise 원시 캡처")
    print(f"  포트      : {args.port} @ {args.baud}")
    print(f"  측정 시간 : {args.duration:.1f}s")
    print("  전제      : 로봇/IMU가 완전히 정지한 상태 (흔들리면 노이즈가 과장되게 측정됨)")
    print("=" * 70)

    driver = n100.ImuDriver(n100.DriverConfig(
        port=args.port, baudrate=args.baud,
        mount_rotation=n100.Quat.from_axis_angle_x(MOUNT_ROLL_DEG * DEG),
    ))
    try:
        driver.start()
    except RuntimeError as e:
        print(f"IMU 시작 실패: {e}")
        print(f"  ls /dev/ttyUSB* /dev/ttyACM*   (권한: sudo chmod 666 {args.port})")
        return 1

    first = driver.wait_for_sample(timeout=3.0)
    if first is None:
        print(f"3초 내 무응답: {driver.last_error() or '원인 불명'}")
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
                print(f"\r  샘플 {n}개  경과 {elapsed:5.1f}/{args.duration:.1f}s", end="", flush=True)
    except KeyboardInterrupt:
        print("\n중단됨 (Ctrl-C).")
    finally:
        drv_stats = driver.stats()
        driver.stop()

    print()
    if not rows:
        print("샘플이 하나도 기록되지 않았습니다.")
        return 1

    os.makedirs(DATA_DIR, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%dT%H%M%S")
    tag = f"_{args.tag}" if args.tag else ""
    fname = f"imu_capture{tag}_{ts}.csv"
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

    print(f"저장됨: {path}")
    print(f"  샘플 {len(rows)}개, driver.stats() = {drv_stats}")
    print("  -> analyze_imu_noise.py 로 분석하세요.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
