"""Read-only CAN acquisition for all RoboNex leg motors."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import math
import struct
import threading
import time
from typing import Callable

import can

from .constants import (
    CHANNEL_MOTOR_IDS,
    CT_PARAM_READ,
    DEFAULT_CAN_INTERFACE,
    DEFAULT_CAN_TIMEOUT_S,
    HOST_ID,
    MECHANICAL_POSITION_INDEX,
    MECHANICAL_VELOCITY_INDEX,
)


@dataclass(frozen=True)
class ParameterResponse:
    motor_id: int
    index: int
    value: float
    received_at_s: float


@dataclass(frozen=True)
class MotorReading:
    motor_id: int
    position_rad: float | None
    velocity_rad_s: float | None
    position_received_at_s: float | None
    velocity_received_at_s: float | None

    @property
    def complete(self) -> bool:
        return self.position_rad is not None and self.velocity_rad_s is not None


@dataclass(frozen=True)
class ChannelReadStats:
    channel: str
    elapsed_s: float
    expected_responses: int
    received_responses: int

    @property
    def scan_hz(self) -> float:
        return 1.0 / self.elapsed_s if self.elapsed_s > 0.0 else math.inf

    @property
    def response_hz(self) -> float:
        return self.received_responses / self.elapsed_s if self.elapsed_s > 0.0 else math.inf


@dataclass(frozen=True)
class ReadCycle:
    readings: dict[int, MotorReading]
    channel_stats: dict[str, ChannelReadStats]
    started_at_s: float
    finished_at_s: float

    @property
    def elapsed_s(self) -> float:
        return self.finished_at_s - self.started_at_s

    @property
    def complete(self) -> bool:
        return all(reading.complete for reading in self.readings.values())

    @property
    def missing_parameters(self) -> tuple[tuple[int, str], ...]:
        missing: list[tuple[int, str]] = []
        for motor_id, reading in sorted(self.readings.items()):
            if reading.position_rad is None:
                missing.append((motor_id, "position"))
            if reading.velocity_rad_s is None:
                missing.append((motor_id, "velocity"))
        return tuple(missing)


def _validate_unsigned(value: int, bits: int, name: str) -> None:
    if not isinstance(value, int) or not 0 <= value < (1 << bits):
        raise ValueError(f"{name} 값은 {bits}비트 부호 없는 정수여야 합니다: {value!r}")


def build_arbitration_id(comm_type: int, data16: int, target_id: int) -> int:
    _validate_unsigned(comm_type, 5, "comm_type")
    _validate_unsigned(data16, 16, "data16")
    _validate_unsigned(target_id, 8, "target_id")
    return (comm_type << 24) | (data16 << 8) | target_id


def parse_arbitration_id(arbitration_id: int) -> tuple[int, int, int]:
    _validate_unsigned(arbitration_id, 29, "arbitration_id")
    return (
        (arbitration_id >> 24) & 0x1F,
        (arbitration_id >> 8) & 0xFFFF,
        arbitration_id & 0xFF,
    )


def build_parameter_read_request(motor_id: int, index: int, host_id: int = HOST_ID) -> can.Message:
    _validate_unsigned(motor_id, 8, "motor_id")
    _validate_unsigned(index, 16, "index")
    _validate_unsigned(host_id, 8, "host_id")
    payload = bytearray(8)
    struct.pack_into("<H", payload, 0, index)
    return can.Message(
        arbitration_id=build_arbitration_id(CT_PARAM_READ, host_id, motor_id),
        data=bytes(payload),
        is_extended_id=True,
    )


def parse_parameter_response(
    message: can.Message | None,
    host_id: int = HOST_ID,
    received_at_s: float | None = None,
) -> ParameterResponse | None:
    if message is None or not message.is_extended_id:
        return None
    try:
        comm_type, data16, destination = parse_arbitration_id(message.arbitration_id)
    except ValueError:
        return None
    if comm_type != CT_PARAM_READ or destination != host_id:
        return None
    payload = bytes(message.data)
    if len(payload) < 8:
        return None
    return ParameterResponse(
        motor_id=data16 & 0xFF,
        index=struct.unpack_from("<H", payload, 0)[0],
        value=struct.unpack_from("<f", payload, 4)[0],
        received_at_s=time.monotonic() if received_at_s is None else received_at_s,
    )


class CanBus:
    """Open each CAN channel once and acquire both channels in parallel."""

    def __init__(
        self,
        channels: tuple[str, ...] = tuple(CHANNEL_MOTOR_IDS),
        interface: str = DEFAULT_CAN_INTERFACE,
        host_id: int = HOST_ID,
        timeout_s: float = DEFAULT_CAN_TIMEOUT_S,
        bus_factory: Callable[..., can.BusABC] = can.Bus,
    ) -> None:
        if not channels:
            raise ValueError("CAN 채널을 하나 이상 지정해야 합니다.")
        if len(set(channels)) != len(channels):
            raise ValueError(f"CAN 채널이 중복되었습니다: {channels}")
        unknown = [channel for channel in channels if channel not in CHANNEL_MOTOR_IDS]
        if unknown:
            raise ValueError(f"지원하지 않는 CAN 채널입니다: {unknown}")
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError(f"CAN timeout은 양수여야 합니다: {timeout_s}")

        self.channels = channels
        self.interface = interface
        self.host_id = host_id
        self.timeout_s = timeout_s
        self._bus_factory = bus_factory
        self._buses: dict[str, can.BusABC] = {}
        self._executor: ThreadPoolExecutor | None = None
        self._read_lock = threading.Lock()

    @property
    def is_open(self) -> bool:
        return bool(self._buses)

    def open(self) -> None:
        if self.is_open:
            return
        try:
            for channel in self.channels:
                self._buses[channel] = self._bus_factory(
                    channel=channel,
                    interface=self.interface,
                )
            self._executor = ThreadPoolExecutor(
                max_workers=len(self.channels),
                thread_name_prefix="robonex-can",
            )
        except BaseException:
            self.close()
            raise

    def close(self) -> None:
        executor, self._executor = self._executor, None
        if executor is not None:
            executor.shutdown(wait=True, cancel_futures=True)
        buses, self._buses = self._buses, {}
        for bus in buses.values():
            bus.shutdown()

    def __enter__(self) -> "CanBus":
        self.open()
        return self

    def __exit__(self, _exc_type, _exc, _traceback) -> None:
        self.close()

    def _read_channel(self, channel: str) -> tuple[dict[int, MotorReading], ChannelReadStats]:
        bus = self._buses[channel]
        motor_ids = CHANNEL_MOTOR_IDS[channel]
        expected = {
            (motor_id, index)
            for motor_id in motor_ids
            for index in (MECHANICAL_POSITION_INDEX, MECHANICAL_VELOCITY_INDEX)
        }
        responses: dict[tuple[int, int], ParameterResponse] = {}
        started_at = time.monotonic()

        for motor_id, index in sorted(expected):
            bus.send(build_parameter_read_request(motor_id, index, self.host_id))

        deadline = time.monotonic() + self.timeout_s
        while len(responses) < len(expected):
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            response = parse_parameter_response(
                bus.recv(timeout=remaining),
                host_id=self.host_id,
            )
            if response is None:
                continue
            key = (response.motor_id, response.index)
            if key in expected:
                responses[key] = response

        readings: dict[int, MotorReading] = {}
        for motor_id in motor_ids:
            position = responses.get((motor_id, MECHANICAL_POSITION_INDEX))
            velocity = responses.get((motor_id, MECHANICAL_VELOCITY_INDEX))
            readings[motor_id] = MotorReading(
                motor_id=motor_id,
                position_rad=position.value if position is not None else None,
                velocity_rad_s=velocity.value if velocity is not None else None,
                position_received_at_s=(
                    position.received_at_s if position is not None else None
                ),
                velocity_received_at_s=(
                    velocity.received_at_s if velocity is not None else None
                ),
            )

        elapsed = time.monotonic() - started_at
        return readings, ChannelReadStats(
            channel=channel,
            elapsed_s=elapsed,
            expected_responses=len(expected),
            received_responses=len(responses),
        )

    def read_all(self) -> ReadCycle:
        if not self.is_open or self._executor is None:
            raise RuntimeError("CAN 버스가 열리지 않았습니다. open()을 먼저 호출하세요.")
        if not self._read_lock.acquire(blocking=False):
            raise RuntimeError("read_all()을 동시에 호출할 수 없습니다.")
        try:
            started_at = time.monotonic()
            futures = {
                channel: self._executor.submit(self._read_channel, channel)
                for channel in self.channels
            }
            readings: dict[int, MotorReading] = {}
            stats: dict[str, ChannelReadStats] = {}
            for channel, future in futures.items():
                channel_readings, channel_stats = future.result()
                readings.update(channel_readings)
                stats[channel] = channel_stats
            return ReadCycle(
                readings=readings,
                channel_stats=stats,
                started_at_s=started_at,
                finished_at_s=time.monotonic(),
            )
        finally:
            self._read_lock.release()
