#!/usr/bin/env python3
"""Hardware-free tests for stage 1 CAN and IMU components."""

from __future__ import annotations

from collections import deque
from pathlib import Path
import struct
import sys
import time
from types import SimpleNamespace
import unittest

import can

COMM_TEST_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(COMM_TEST_DIR))

from core.can_bus import (  # noqa: E402
    CanBus,
    build_arbitration_id,
    build_parameter_read_request,
    parse_arbitration_id,
    parse_parameter_response,
)
from core.constants import (  # noqa: E402
    CHANNEL_MOTOR_IDS,
    CT_PARAM_READ,
    HOST_ID,
    JOINT_LIMITS_RAD,
    JOINT_NAMES,
    MECHANICAL_POSITION_INDEX,
    MECHANICAL_VELOCITY_INDEX,
    MOTOR_IDS,
    MOTOR_MODELS,
)
from core.imu import ImuError, N100Imu  # noqa: E402


def parameter_response(motor_id: int, index: int, value: float) -> can.Message:
    payload = bytearray(8)
    struct.pack_into("<H", payload, 0, index)
    struct.pack_into("<f", payload, 4, value)
    return can.Message(
        arbitration_id=build_arbitration_id(CT_PARAM_READ, motor_id, HOST_ID),
        data=bytes(payload),
        is_extended_id=True,
    )


class FakeBus:
    def __init__(self, channel: str, missing: set[tuple[int, int]] | None = None) -> None:
        self.channel = channel
        self.missing = missing or set()
        self.responses: deque[can.Message] = deque()
        self.sent: list[can.Message] = []
        self.shutdown_called = False

    def send(self, message: can.Message) -> None:
        self.sent.append(message)
        _comm_type, _host_id, motor_id = parse_arbitration_id(message.arbitration_id)
        index = struct.unpack_from("<H", bytes(message.data), 0)[0]
        if (motor_id, index) in self.missing:
            return
        value = motor_id + (0.25 if index == MECHANICAL_POSITION_INDEX else 0.5)
        self.responses.append(parameter_response(motor_id, index, value))

    def recv(self, timeout: float | None = None) -> can.Message | None:
        if self.responses:
            return self.responses.popleft()
        if timeout:
            time.sleep(min(timeout, 0.0001))
        return None

    def shutdown(self) -> None:
        self.shutdown_called = True


class FakeN100Module:
    class Quat:
        @staticmethod
        def from_axis_angle_x(angle):
            return ("rx", angle)

    class DriverConfig:
        def __init__(self, **kwargs):
            self.kwargs = kwargs

    def __init__(self, sample=None):
        self.sample = sample
        module = self

        class ImuDriver:
            def __init__(self, config):
                self.config = config
                self.is_running = False
                self.stop_called = False

            def start(self):
                self.is_running = True

            def wait_for_sample(self, timeout):
                return module.sample

            def latest(self):
                return module.sample

            def last_error(self):
                return ""

            def stop(self):
                self.stop_called = True
                self.is_running = False

        self.ImuDriver = ImuDriver


def fake_imu_sample():
    vec = lambda x, y, z: SimpleNamespace(x=x, y=y, z=z)
    return SimpleNamespace(
        angular_velocity=vec(1.0, 2.0, 3.0),
        angular_velocity_raw=vec(4.0, 5.0, 6.0),
        projected_gravity=vec(-0.049, -0.036, -0.998),
        seq=17,
        host_timestamp_ns=123456,
    )


class ConstantsTest(unittest.TestCase):
    def test_all_physical_motors_have_one_mapping(self):
        self.assertEqual(MOTOR_IDS, tuple(range(1, 13)))
        self.assertEqual(set(MOTOR_IDS), set(JOINT_NAMES))
        self.assertEqual(set(MOTOR_IDS), set(MOTOR_MODELS))
        self.assertEqual(set(MOTOR_IDS), set(JOINT_LIMITS_RAD))
        self.assertEqual(set(CHANNEL_MOTOR_IDS), {"can0", "can1"})


class CanProtocolTest(unittest.TestCase):
    def test_arbitration_id_round_trip(self):
        arbitration_id = build_arbitration_id(CT_PARAM_READ, HOST_ID, 12)
        self.assertEqual(arbitration_id, 0x1100FD0C)
        self.assertEqual(parse_arbitration_id(arbitration_id), (CT_PARAM_READ, HOST_ID, 12))

    def test_parameter_request_payload(self):
        request = build_parameter_read_request(4, MECHANICAL_POSITION_INDEX)
        self.assertTrue(request.is_extended_id)
        self.assertEqual(bytes(request.data[:2]), b"\x19\x70")

    def test_parameter_response_parser(self):
        payload = bytearray(8)
        struct.pack_into("<H", payload, 0, MECHANICAL_VELOCITY_INDEX)
        struct.pack_into("<f", payload, 4, -1.25)
        message = can.Message(
            arbitration_id=0x110007FD,
            data=bytes(payload),
            is_extended_id=True,
        )
        response = parse_parameter_response(message, received_at_s=10.0)
        self.assertIsNotNone(response)
        self.assertEqual(response.motor_id, 7)
        self.assertEqual(response.index, MECHANICAL_VELOCITY_INDEX)
        self.assertAlmostEqual(response.value, -1.25)
        self.assertEqual(response.received_at_s, 10.0)

    def test_two_channels_are_read_and_closed(self):
        buses: dict[str, FakeBus] = {}

        def factory(channel, interface):
            self.assertEqual(interface, "virtual-test")
            buses[channel] = FakeBus(channel)
            return buses[channel]

        with CanBus(interface="virtual-test", timeout_s=0.005, bus_factory=factory) as bus:
            cycle = bus.wait_for_first(timeout_s=0.1)

        self.assertTrue(cycle.complete)
        self.assertEqual(set(cycle.readings), set(MOTOR_IDS))
        self.assertEqual(cycle.missing_parameters, ())
        self.assertEqual(set(cycle.channel_stats), set(CHANNEL_MOTOR_IDS))
        self.assertTrue(all(stats.total_scans >= 1 for stats in cycle.channel_stats.values()))
        self.assertTrue(all(stats.response_ratio == 1.0 for stats in cycle.channel_stats.values()))
        for motor_id, reading in cycle.readings.items():
            self.assertAlmostEqual(reading.position_rad, motor_id + 0.25)
            self.assertAlmostEqual(reading.velocity_rad_s, motor_id + 0.5)
        self.assertTrue(all(bus.shutdown_called for bus in buses.values()))

    def test_missing_response_is_not_replaced_with_zero(self):
        missing = {(1, MECHANICAL_VELOCITY_INDEX)}

        def factory(channel, interface):
            return FakeBus(channel, missing=missing)

        with CanBus(timeout_s=0.001, bus_factory=factory) as bus:
            cycle = bus.wait_for_first(timeout_s=0.1)

        self.assertFalse(cycle.complete)
        self.assertIsNone(cycle.readings[1].velocity_rad_s)
        self.assertIn((1, "velocity"), cycle.missing_parameters)


class ImuWrapperTest(unittest.TestCase):
    def test_start_latest_and_close(self):
        module = FakeN100Module(fake_imu_sample())
        imu = N100Imu(n100_module=module)
        first = imu.start(wait_timeout_s=0.1)
        latest = imu.latest()
        self.assertEqual(first.projected_gravity, (-0.049, -0.036, -0.998))
        self.assertEqual(latest.angular_velocity_raw, (4.0, 5.0, 6.0))
        self.assertEqual(latest.sequence, 17)
        imu.close()
        self.assertFalse(imu.is_running)

    def test_start_timeout_closes_driver(self):
        imu = N100Imu(n100_module=FakeN100Module(sample=None))
        with self.assertRaises(ImuError):
            imu.start(wait_timeout_s=0.01)
        self.assertFalse(imu.is_running)


if __name__ == "__main__":
    unittest.main()
