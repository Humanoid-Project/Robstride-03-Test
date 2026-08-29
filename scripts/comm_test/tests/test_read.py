#!/usr/bin/env python3
"""Read-only stage 1 integration test for all motors and the N100 IMU."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time

import can

COMM_TEST_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(COMM_TEST_DIR))

from core.can_bus import CanBus, ReadCycle  # noqa: E402
from core.constants import (  # noqa: E402
    CHANNEL_MOTOR_IDS,
    CONTROL_HZ,
    DEFAULT_CAN_INTERFACE,
    DEFAULT_CAN_TIMEOUT_S,
    DEFAULT_IMU_PORT,
    EXPECTED_UPRIGHT_GRAVITY,
    HOST_ID,
    JOINT_NAMES,
    MOTOR_MODELS,
)
from core.imu import ImuError, ImuReading, N100Imu  # noqa: E402


def positive_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("0보다 큰 유한한 숫자여야 합니다.")
    return parsed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "모터 12개의 위치·속도와 N100 IMU를 읽고 응답 완전성 및 속도를 측정합니다. "
            "모터 enable이나 제어 명령은 보내지 않습니다."
        )
    )
    parser.add_argument(
        "--channels",
        nargs="+",
        choices=tuple(CHANNEL_MOTOR_IDS),
        default=list(CHANNEL_MOTOR_IDS),
        help="읽을 CAN 채널 (기본: can0 can1)",
    )
    parser.add_argument("--interface", default=DEFAULT_CAN_INTERFACE, help="python-can 인터페이스")
    parser.add_argument(
        "--host-id",
        type=lambda value: int(value, 0),
        default=HOST_ID,
        help="호스트 CAN ID (기본: 0xFD)",
    )
    parser.add_argument(
        "--timeout",
        type=positive_float,
        default=DEFAULT_CAN_TIMEOUT_S,
        help="파라미터 요청당 응답 대기시간(초, 기본: 0.02)",
    )
    parser.add_argument("--imu-port", default=DEFAULT_IMU_PORT, help="N100 시리얼 포트")
    parser.add_argument(
        "--duration",
        type=positive_float,
        default=5.0,
        help="측정 시간(초, 기본: 5)",
    )
    parser.add_argument(
        "--print-hz",
        type=positive_float,
        default=2.0,
        help="중간값 출력 주파수(기본: 2 Hz)",
    )
    parser.add_argument("--no-can", action="store_true", help="IMU만 검사")
    parser.add_argument("--no-imu", action="store_true", help="CAN 모터만 검사")
    return parser.parse_args()


def format_value(value: float | None, width: int = 11) -> str:
    if value is None:
        return f"{'--':>{width}}"
    if not math.isfinite(value):
        return f"{'NaN/Inf':>{width}}"
    return f"{value:+{width}.4f}"


def print_snapshot(cycle: ReadCycle | None, imu_reading: ImuReading | None) -> None:
    print(f"\n[{time.strftime('%H:%M:%S')}] 읽기 전용 센서 상태")
    if cycle is not None:
        for channel in cycle.channel_stats:
            stats = cycle.channel_stats[channel]
            print(
                f"  {channel}: {stats.elapsed_s * 1000.0:6.2f} ms/scan, "
                f"응답 {stats.received_responses}/{stats.expected_responses}, "
                f"환산 {stats.scan_hz:6.1f} scan/s"
            )
        print(f"  {'ID':>3}  {'joint':<18}  {'model':<5}  {'pos [rad]':>11}  {'vel [rad/s]':>11}")
        for motor_id, reading in sorted(cycle.readings.items()):
            print(
                f"  {motor_id:>3}  {JOINT_NAMES[motor_id]:<18}  "
                f"{MOTOR_MODELS[motor_id].upper():<5}  "
                f"{format_value(reading.position_rad)}  "
                f"{format_value(reading.velocity_rad_s)}"
            )
    if imu_reading is not None:
        fused = imu_reading.angular_velocity
        raw = imu_reading.angular_velocity_raw
        gravity = imu_reading.projected_gravity
        gravity_error = math.dist(gravity, EXPECTED_UPRIGHT_GRAVITY)
        print(f"  IMU seq={imu_reading.sequence}")
        print(f"    gyro fused: ({fused[0]:+.4f}, {fused[1]:+.4f}, {fused[2]:+.4f}) rad/s")
        print(f"    gyro raw:   ({raw[0]:+.4f}, {raw[1]:+.4f}, {raw[2]:+.4f}) rad/s")
        print(
            f"    gravity:    ({gravity[0]:+.4f}, {gravity[1]:+.4f}, {gravity[2]:+.4f}) "
            f"기준점 오차={gravity_error:.4f}"
        )


def main() -> int:
    args = parse_args()
    if args.no_can and args.no_imu:
        print("--no-can과 --no-imu를 동시에 사용할 수 없습니다.")
        return 2

    bus = None
    imu = None
    startup_failed = False
    if not args.no_can:
        try:
            bus = CanBus(
                channels=tuple(args.channels),
                interface=args.interface,
                host_id=args.host_id,
                timeout_s=args.timeout,
            )
            bus.open()
            first_can = bus.wait_for_first(
                timeout_s=max(1.0, args.timeout * 24.0 + 0.5),
            )
            print(f"CAN 열기 완료: {', '.join(args.channels)}")
        except (OSError, can.CanError, RuntimeError, ValueError) as error:
            print(f"CAN 열기 실패: {error}")
            print("먼저 ./scripts/comm_test/can_up.sh를 직접 실행해 인터페이스를 올리세요.")
            startup_failed = True

    first_imu = None
    if not args.no_imu:
        try:
            imu = N100Imu(port=args.imu_port)
            first_imu = imu.start(wait_timeout_s=3.0)
            print(f"IMU 열기 완료: {args.imu_port}, 첫 seq={first_imu.sequence}")
        except (ImuError, RuntimeError, OSError) as error:
            print(f"IMU 열기 실패: {error}")
            startup_failed = True

    if startup_failed:
        if bus is not None:
            bus.close()
        if imu is not None:
            imu.close()
        return 1

    complete_cycles = 0
    total_cycles = 0
    last_cycle = first_can if bus is not None else None
    last_imu = first_imu
    first_imu_sequence = first_imu.sequence if first_imu is not None else None
    last_imu_sequence = first_imu_sequence

    started_at = time.monotonic()
    deadline = started_at + args.duration
    next_print = started_at
    next_tick = started_at
    try:
        while time.monotonic() < deadline:
            if bus is not None:
                last_cycle = bus.read_all()
                total_cycles += 1
                complete_cycles += int(last_cycle.complete)

            if imu is not None:
                last_imu = imu.latest()
                if last_imu is not None:
                    last_imu_sequence = last_imu.sequence

            now = time.monotonic()
            if now >= next_print:
                print_snapshot(last_cycle, last_imu)
                next_print = now + 1.0 / args.print_hz

            next_tick += 1.0 / CONTROL_HZ
            sleep_s = next_tick - time.monotonic()
            if sleep_s > 0.0:
                time.sleep(sleep_s)
            else:
                next_tick = time.monotonic()
    except KeyboardInterrupt:
        print("\n사용자가 측정을 중단했습니다.")
    except (RuntimeError, can.CanError, OSError) as error:
        print(f"\n읽기 중 오류: {error}")
        return 1
    finally:
        if bus is not None:
            bus.close()
        if imu is not None:
            imu.close()

    measured_s = time.monotonic() - started_at
    success = True
    print("\n측정 요약")
    if bus is not None:
        policy_sample_hz = total_cycles / measured_s if measured_s > 0.0 else 0.0
        complete_percent = 100.0 * complete_cycles / total_cycles if total_cycles else 0.0
        print(
            f"  60 Hz 스냅샷: 실제 {policy_sample_hz:.1f} Hz, "
            f"완전한 선택 축 {complete_cycles}/{total_cycles} ({complete_percent:.1f}%)"
        )
        sensor_scan_rates = []
        for channel in args.channels:
            stats = last_cycle.channel_stats[channel]
            scan_hz = stats.average_scan_hz
            sensor_scan_rates.append(scan_hz)
            complete_scan_percent = 100.0 * stats.complete_scans / stats.total_scans
            print(
                f"  {channel}: 평균 {stats.mean_elapsed_s * 1000.0:.2f} ms, "
                f"p95 {stats.p95_elapsed_s * 1000.0:.2f} ms, {scan_hz:.1f} scan/s, "
                f"{stats.average_response_hz:.1f} parameter responses/s, "
                f"응답률 {stats.response_ratio * 100.0:.1f}%, "
                f"완전 scan {complete_scan_percent:.1f}%"
            )
            print(
                f"    판정: 60 Hz full scan {'가능' if scan_hz >= CONTROL_HZ else '미달'}, "
                f"200 responses/s {'충족' if stats.average_response_hz >= 200.0 else '미달'}"
            )
            success &= stats.response_ratio == 1.0 and scan_hz >= CONTROL_HZ
        if sensor_scan_rates:
            print(f"  병렬 센서 갱신 병목: {min(sensor_scan_rates):.1f} Hz")
        success &= complete_cycles > 0
    if imu is not None:
        if last_imu is None:
            print("  IMU 판정: 샘플 없음")
            success = False
        else:
            sequence_delta = max(0, last_imu_sequence - first_imu_sequence)
            imu_update_hz = sequence_delta / measured_s if measured_s > 0.0 else 0.0
            print(
                f"  IMU: seq {first_imu_sequence}→{last_imu_sequence}, "
                f"센서 갱신 약 {imu_update_hz:.1f} Hz"
            )
            gravity_error = math.dist(last_imu.projected_gravity, EXPECTED_UPRIGHT_GRAVITY)
            print(f"  IMU 중력벡터 기준점 오차: {gravity_error:.4f}")
            success &= sequence_delta > 0

    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
