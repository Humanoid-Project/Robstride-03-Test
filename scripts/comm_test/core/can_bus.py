"""Read-only CAN acquisition for all RoboNex leg motors."""

from __future__ import annotations

from collections import deque
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
    total_scans: int
    complete_scans: int
    total_elapsed_s: float
    total_expected_responses: int
    total_received_responses: int
    mean_elapsed_s: float
    p95_elapsed_s: float
    completed_at_s: float

    @property
    def scan_hz(self) -> float:
        return 1.0 / self.elapsed_s if self.elapsed_s > 0.0 else math.inf

    @property
    def response_hz(self) -> float:
        return self.received_responses / self.elapsed_s if self.elapsed_s > 0.0 else math.inf

    @property
    def average_scan_hz(self) -> float:
        return self.total_scans / self.total_elapsed_s if self.total_elapsed_s > 0.0 else 0.0

    @property
    def average_response_hz(self) -> float:
        if self.total_elapsed_s <= 0.0:
            return 0.0
        return self.total_received_responses / self.total_elapsed_s

    @property
    def response_ratio(self) -> float:
        if self.total_expected_responses <= 0:
            return 0.0
        return self.total_received_responses / self.total_expected_responses


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


def _percentile95(values: deque[float]) -> float:
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, math.ceil(len(ordered) * 0.95) - 1)]


class _ChannelReader(threading.Thread):
    """The proven policy_test read loop, isolated to one CAN channel."""

    def __init__(
        self,
        channel: str,
        bus: can.BusABC,
        motor_ids: tuple[int, ...],
        host_id: int,
        timeout_s: float,
        publish: Callable[[str, dict[int, MotorReading], ChannelReadStats], None],
        fail: Callable[[str, BaseException], None],
    ) -> None:
        super().__init__(name=f"robonex-{channel}", daemon=True)
        self.channel = channel
        self.bus = bus
        self.motor_ids = motor_ids
        self.host_id = host_id
        self.timeout_s = timeout_s
        self._publish = publish
        self._fail = fail
        self._stop_event = threading.Event()
        self._elapsed_samples: deque[float] = deque(maxlen=10000)
        self._total_scans = 0
        self._complete_scans = 0
        self._total_elapsed_s = 0.0
        self._total_expected_responses = 0
        self._total_received_responses = 0

    def stop(self) -> None:
        self._stop_event.set()

    def _read_parameter(self, motor_id: int, index: int) -> ParameterResponse | None:
        self.bus.send(build_parameter_read_request(motor_id, index, self.host_id))
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            response = parse_parameter_response(
                self.bus.recv(timeout=max(0.0, deadline - time.monotonic())),
                host_id=self.host_id,
            )
            if response is None:
                continue
            if response.motor_id == motor_id and response.index == index:
                return response
        return None

    def run(self) -> None:
        expected_responses = len(self.motor_ids) * 2
        try:
            while not self._stop_event.is_set():
                started_at = time.monotonic()
                readings: dict[int, MotorReading] = {}
                received_responses = 0
                for motor_id in self.motor_ids:
                    if self._stop_event.is_set():
                        return
                    position = self._read_parameter(motor_id, MECHANICAL_POSITION_INDEX)
                    velocity = self._read_parameter(motor_id, MECHANICAL_VELOCITY_INDEX)
                    received_responses += int(position is not None) + int(velocity is not None)
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

                completed_at = time.monotonic()
                elapsed_s = completed_at - started_at
                self._elapsed_samples.append(elapsed_s)
                self._total_scans += 1
                self._complete_scans += int(received_responses == expected_responses)
                self._total_elapsed_s += elapsed_s
                self._total_expected_responses += expected_responses
                self._total_received_responses += received_responses
                self._publish(
                    self.channel,
                    readings,
                    ChannelReadStats(
                        channel=self.channel,
                        elapsed_s=elapsed_s,
                        expected_responses=expected_responses,
                        received_responses=received_responses,
                        total_scans=self._total_scans,
                        complete_scans=self._complete_scans,
                        total_elapsed_s=self._total_elapsed_s,
                        total_expected_responses=self._total_expected_responses,
                        total_received_responses=self._total_received_responses,
                        mean_elapsed_s=self._total_elapsed_s / self._total_scans,
                        p95_elapsed_s=_percentile95(self._elapsed_samples),
                        completed_at_s=completed_at,
                    ),
                )
        except Exception as error:
            self._fail(self.channel, error)


class CanBus:
    """Continuously read each channel and expose a non-blocking latest snapshot."""

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
        self._readers: dict[str, _ChannelReader] = {}
        self._latest: dict[str, tuple[dict[int, MotorReading], ChannelReadStats]] = {}
        self._errors: dict[str, BaseException] = {}
        self._condition = threading.Condition()

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
            with self._condition:
                self._latest.clear()
                self._errors.clear()
            self._readers = {
                channel: _ChannelReader(
                    channel=channel,
                    bus=self._buses[channel],
                    motor_ids=CHANNEL_MOTOR_IDS[channel],
                    host_id=self.host_id,
                    timeout_s=self.timeout_s,
                    publish=self._publish,
                    fail=self._publish_error,
                )
                for channel in self.channels
            }
            for reader in self._readers.values():
                reader.start()
        except BaseException:
            self.close()
            raise

    def close(self) -> None:
        readers, self._readers = self._readers, {}
        for reader in readers.values():
            reader.stop()
        for reader in readers.values():
            reader.join(timeout=max(2.0, self.timeout_s * 24.0))
        buses, self._buses = self._buses, {}
        for bus in buses.values():
            bus.shutdown()

    def __enter__(self) -> "CanBus":
        self.open()
        return self

    def __exit__(self, _exc_type, _exc, _traceback) -> None:
        self.close()

    def _publish(
        self,
        channel: str,
        readings: dict[int, MotorReading],
        stats: ChannelReadStats,
    ) -> None:
        with self._condition:
            self._latest[channel] = (readings, stats)
            self._condition.notify_all()

    def _publish_error(self, channel: str, error: BaseException) -> None:
        with self._condition:
            self._errors[channel] = error
            self._condition.notify_all()

    def _snapshot_locked(self) -> ReadCycle:
        if self._errors:
            channel, error = next(iter(self._errors.items()))
            raise RuntimeError(f"{channel} CAN 리더 중단: {error}") from error
        missing = [channel for channel in self.channels if channel not in self._latest]
        if missing:
            raise RuntimeError(f"아직 첫 CAN scan이 없습니다: {missing}")
        readings: dict[int, MotorReading] = {}
        stats: dict[str, ChannelReadStats] = {}
        for channel in self.channels:
            channel_readings, channel_stats = self._latest[channel]
            readings.update(dict(channel_readings))
            stats[channel] = channel_stats
        return ReadCycle(
            readings=readings,
            channel_stats=stats,
            started_at_s=min(
                channel_stats.completed_at_s - channel_stats.elapsed_s
                for channel_stats in stats.values()
            ),
            finished_at_s=time.monotonic(),
        )

    def read_all(self) -> ReadCycle:
        if not self.is_open:
            raise RuntimeError("CAN 버스가 열리지 않았습니다. open()을 먼저 호출하세요.")
        with self._condition:
            return self._snapshot_locked()

    def wait_for_first(self, timeout_s: float = 1.0) -> ReadCycle:
        if not self.is_open:
            raise RuntimeError("CAN 버스가 열리지 않았습니다. open()을 먼저 호출하세요.")
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError(f"첫 CAN scan 대기시간은 양수여야 합니다: {timeout_s}")
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while True:
                if self._errors:
                    return self._snapshot_locked()
                if all(channel in self._latest for channel in self.channels):
                    return self._snapshot_locked()
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    missing = [channel for channel in self.channels if channel not in self._latest]
                    raise RuntimeError(f"첫 CAN scan 대기시간 초과: {missing}")
                self._condition.wait(timeout=remaining)
