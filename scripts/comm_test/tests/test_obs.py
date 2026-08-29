#!/usr/bin/env python3
"""Hardware-free tests for stage 2 policy observation assembly."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import math
import sys
import unittest

import numpy as np

COMM_TEST_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(COMM_TEST_DIR))

from core.can_bus import MotorReading, ReadCycle  # noqa: E402
from core.constants import (  # noqa: E402
    DEFAULT_JOINT_POSITIONS_RAD,
    POLICY_JOINT_ORDER,
    POLICY_MOTOR_IDS,
    POLICY_OBSERVATION_SIZE,
)
from core.imu import ImuReading  # noqa: E402
from core.observation import (  # noqa: E402
    ObservationError,
    ObservationSpec,
    build_observation,
)


def make_cycle(
    positions: dict[int, float] | None = None,
    velocities: dict[int, float] | None = None,
    motor_ids: tuple[int, ...] = POLICY_MOTOR_IDS,
) -> ReadCycle:
    positions = positions or {motor_id: motor_id * 0.005 for motor_id in motor_ids}
    velocities = velocities or {motor_id: motor_id * 0.1 for motor_id in motor_ids}
    readings = {
        motor_id: MotorReading(
            motor_id=motor_id,
            position_rad=positions[motor_id],
            velocity_rad_s=velocities[motor_id],
            position_received_at_s=10.0,
            velocity_received_at_s=10.0,
        )
        for motor_id in motor_ids
    }
    return ReadCycle(
        readings=readings,
        channel_stats={},
        started_at_s=9.9,
        finished_at_s=10.0,
    )


def make_imu(
    raw: tuple[float, float, float] = (1.1, 1.2, 1.3),
    gravity: tuple[float, float, float] = (0.0, 0.0, -1.0),
) -> ImuReading:
    return ImuReading(
        angular_velocity=(9.1, 9.2, 9.3),
        angular_velocity_raw=raw,
        projected_gravity=gravity,
        sequence=7,
        host_timestamp_ns=123,
        observed_at_s=10.0,
    )


class PolicyContractTest(unittest.TestCase):
    def test_balancing_contract_is_exact(self):
        self.assertEqual(POLICY_MOTOR_IDS, (1, 7, 2, 8, 3, 9, 4, 10, 6, 5, 12, 11))
        self.assertEqual(
            POLICY_JOINT_ORDER,
            (
                "l_hip_yaw_joint",
                "r_hip_yaw_joint",
                "l_hip_pitch_joint",
                "r_hip_pitch_joint",
                "l_hip_roll_joint",
                "r_hip_roll_joint",
                "l_knee_pitch_joint",
                "r_knee_pitch_joint",
                "l_ankle_lower_joint",
                "l_ankle_upper_joint",
                "r_ankle_lower_joint",
                "r_ankle_upper_joint",
            ),
        )
        self.assertEqual(DEFAULT_JOINT_POSITIONS_RAD, (0.0,) * 12)
        self.assertEqual(POLICY_OBSERVATION_SIZE, 42)


class ObservationAssemblyTest(unittest.TestCase):
    def setUp(self):
        self.previous_action = np.linspace(-1.1, 1.1, 12, dtype=np.float32)

    def test_layout_order_dtype_and_raw_gyro(self):
        cycle = make_cycle()
        observation = build_observation(cycle, make_imu(), self.previous_action)

        expected_positions = np.asarray(
            [motor_id * 0.005 for motor_id in POLICY_MOTOR_IDS],
            dtype=np.float32,
        )
        expected_velocities = np.asarray(
            [motor_id * 0.1 for motor_id in POLICY_MOTOR_IDS],
            dtype=np.float32,
        )
        expected = np.concatenate(
            (
                expected_positions,
                expected_velocities,
                np.asarray((1.1, 1.2, 1.3), dtype=np.float32),
                np.asarray((0.0, 0.0, -1.0), dtype=np.float32),
                self.previous_action,
            )
        )

        self.assertEqual(observation.shape, (42,))
        self.assertEqual(observation.dtype, np.float32)
        np.testing.assert_allclose(observation, expected, rtol=0.0, atol=1.0e-7)

    def test_position_is_wrapped_and_default_is_subtracted(self):
        positions = {motor_id: 0.0 for motor_id in POLICY_MOTOR_IDS}
        positions[1] = 2.0 * math.pi - 0.1
        defaults = list(DEFAULT_JOINT_POSITIONS_RAD)
        defaults[0] = -0.2
        spec = ObservationSpec(default_positions_rad=tuple(defaults))

        observation = build_observation(
            make_cycle(positions=positions),
            make_imu(),
            self.previous_action,
            spec=spec,
        )

        self.assertAlmostEqual(float(observation[0]), 0.1, places=6)

    def test_joint_count_is_derived_from_spec(self):
        spec = ObservationSpec(
            motor_ids=(1, 7),
            joint_names=("l_hip_yaw_joint", "r_hip_yaw_joint"),
            default_positions_rad=(0.0, 0.0),
        )
        cycle = make_cycle(motor_ids=spec.motor_ids)

        observation = build_observation(cycle, make_imu(), (0.0, 0.0), spec=spec)

        self.assertEqual(spec.observation_size, 12)
        self.assertEqual(observation.shape, (12,))

    def test_missing_motor_or_parameter_is_rejected(self):
        missing_motor = make_cycle(motor_ids=POLICY_MOTOR_IDS[:-1])
        with self.assertRaisesRegex(ObservationError, "누락"):
            build_observation(missing_motor, make_imu(), self.previous_action)

        cycle = make_cycle()
        broken = replace(cycle.readings[1], velocity_rad_s=None)
        readings = dict(cycle.readings)
        readings[1] = broken
        with self.assertRaisesRegex(ObservationError, "누락"):
            build_observation(replace(cycle, readings=readings), make_imu(), self.previous_action)

    def test_missing_sensor_timestamp_is_rejected(self):
        cycle = make_cycle()
        readings = dict(cycle.readings)
        readings[1] = replace(readings[1], position_received_at_s=None)
        with self.assertRaisesRegex(ObservationError, "수신 시각"):
            build_observation(replace(cycle, readings=readings), make_imu(), self.previous_action)

    def test_nonfinite_values_are_rejected(self):
        cycle = make_cycle()
        readings = dict(cycle.readings)
        readings[1] = replace(readings[1], velocity_rad_s=math.nan)
        with self.assertRaisesRegex(ObservationError, "NaN/inf"):
            build_observation(replace(cycle, readings=readings), make_imu(), self.previous_action)

        with self.assertRaisesRegex(ObservationError, "NaN/inf"):
            build_observation(cycle, make_imu(raw=(math.inf, 0.0, 0.0)), self.previous_action)

        bad_action = self.previous_action.copy()
        bad_action[3] = math.nan
        with self.assertRaisesRegex(ObservationError, "NaN/inf"):
            build_observation(cycle, make_imu(), bad_action)

    def test_position_velocity_imu_gravity_and_action_ranges_are_rejected(self):
        cases = []

        positions = {motor_id: 0.0 for motor_id in POLICY_MOTOR_IDS}
        positions[1] = 1.0
        cases.append(
            (make_cycle(positions=positions), make_imu(), self.previous_action, "관절 범위")
        )

        velocities = {motor_id: 0.0 for motor_id in POLICY_MOTOR_IDS}
        velocities[2] = 20.1
        cases.append(
            (make_cycle(velocities=velocities), make_imu(), self.previous_action, "모터 범위")
        )

        cases.append(
            (make_cycle(), make_imu(raw=(100.1, 0.0, 0.0)), self.previous_action, "각속도")
        )
        cases.append(
            (make_cycle(), make_imu(gravity=(0.0, 0.0, 0.0)), self.previous_action, "중력")
        )

        action = self.previous_action.copy()
        action[0] = 3.01
        cases.append((make_cycle(), make_imu(), action, "clip 범위"))

        for cycle, imu, action, message in cases:
            with self.subTest(message=message):
                with self.assertRaisesRegex(ObservationError, message):
                    build_observation(cycle, imu, action)

    def test_wrong_previous_action_length_is_rejected(self):
        with self.assertRaisesRegex(ObservationError, "길이는 12"):
            build_observation(make_cycle(), make_imu(), (0.0,) * 11)


if __name__ == "__main__":
    unittest.main()
