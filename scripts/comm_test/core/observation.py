"""Build the RoboNex balancing policy observation from live sensor snapshots."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np

from .can_bus import ReadCycle
from .constants import (
    DEFAULT_JOINT_POSITIONS_RAD,
    JOINT_LIMITS_RAD,
    MODEL_JOINT_NAMES,
    MOTOR_MODELS,
    MOTOR_SPECS,
    POLICY_JOINT_ORDER,
    POLICY_MOTOR_IDS,
    RUNNER_ACTION_CLIP,
)
from .imu import ImuReading


MAX_ABS_ANGULAR_VELOCITY_RAD_S = 100.0
MIN_GRAVITY_NORM = 0.8
MAX_GRAVITY_NORM = 1.2


class ObservationError(ValueError):
    """Raised when a sensor snapshot cannot form a safe policy observation."""


@dataclass(frozen=True)
class ObservationSpec:
    motor_ids: tuple[int, ...] = POLICY_MOTOR_IDS
    joint_names: tuple[str, ...] = POLICY_JOINT_ORDER
    default_positions_rad: tuple[float, ...] = DEFAULT_JOINT_POSITIONS_RAD
    runner_action_clip: float = RUNNER_ACTION_CLIP
    max_abs_angular_velocity_rad_s: float = MAX_ABS_ANGULAR_VELOCITY_RAD_S
    min_gravity_norm: float = MIN_GRAVITY_NORM
    max_gravity_norm: float = MAX_GRAVITY_NORM

    def __post_init__(self) -> None:
        joint_count = len(self.motor_ids)
        if joint_count == 0:
            raise ValueError("정책 관절이 비어 있습니다.")
        if len(set(self.motor_ids)) != joint_count:
            raise ValueError(f"정책 모터 ID가 중복되었습니다: {self.motor_ids}")
        if len(self.joint_names) != joint_count:
            raise ValueError("정책 관절 이름 수와 모터 ID 수가 다릅니다.")
        if len(self.default_positions_rad) != joint_count:
            raise ValueError("기본 관절 위치 수와 모터 ID 수가 다릅니다.")
        for motor_id, joint_name, default_position in zip(
            self.motor_ids,
            self.joint_names,
            self.default_positions_rad,
        ):
            expected_name = MODEL_JOINT_NAMES.get(motor_id)
            if expected_name != joint_name:
                raise ValueError(
                    f"정책 관절 매핑이 다릅니다: ID {motor_id}, "
                    f"기대 {expected_name!r}, 입력 {joint_name!r}"
                )
            if not math.isfinite(default_position):
                raise ValueError(f"{joint_name} 기본 위치가 NaN/inf입니다.")
            lower, upper = JOINT_LIMITS_RAD[motor_id]
            if not lower <= default_position <= upper:
                raise ValueError(
                    f"{joint_name} 기본 위치가 관절 범위 밖입니다: {default_position}"
                )
        if not math.isfinite(self.runner_action_clip) or self.runner_action_clip <= 0.0:
            raise ValueError("정책 action clip은 양수여야 합니다.")
        if (
            not math.isfinite(self.max_abs_angular_velocity_rad_s)
            or self.max_abs_angular_velocity_rad_s <= 0.0
        ):
            raise ValueError("IMU 각속도 한계는 양수여야 합니다.")
        if not 0.0 < self.min_gravity_norm < self.max_gravity_norm:
            raise ValueError("중력 벡터 크기 범위가 잘못되었습니다.")

    @property
    def joint_count(self) -> int:
        return len(self.motor_ids)

    @property
    def observation_size(self) -> int:
        return 3 * self.joint_count + 6


DEFAULT_OBSERVATION_SPEC = ObservationSpec()


def wrap_to_pi(angle: float) -> float:
    if not math.isfinite(angle):
        raise ObservationError(f"관절 위치가 NaN/inf입니다: {angle}")
    return math.atan2(math.sin(angle), math.cos(angle))


def _finite_vector(name: str, values: Sequence[float], expected_size: int) -> np.ndarray:
    try:
        vector = np.asarray(values, dtype=np.float64)
    except (TypeError, ValueError) as error:
        raise ObservationError(f"{name}을 숫자 벡터로 변환할 수 없습니다.") from error
    if vector.shape != (expected_size,):
        raise ObservationError(
            f"{name} 길이는 {expected_size}여야 합니다: shape={vector.shape}"
        )
    if not np.isfinite(vector).all():
        raise ObservationError(f"{name}에 NaN/inf가 있습니다.")
    return vector


def _validate_timestamp(value: float | None, label: str) -> None:
    if value is None or not math.isfinite(value) or value < 0.0:
        raise ObservationError(f"{label} 수신 시각이 없습니다.")


def build_observation(
    can_cycle: ReadCycle,
    imu_reading: ImuReading,
    previous_action: Sequence[float],
    spec: ObservationSpec = DEFAULT_OBSERVATION_SPEC,
) -> np.ndarray:
    """Return ``[q-q_default, qdot, gyro_raw, gravity, last_action]``."""

    expected_ids = set(spec.motor_ids)
    actual_ids = set(can_cycle.readings)
    missing_ids = sorted(expected_ids - actual_ids)
    unexpected_ids = sorted(actual_ids - expected_ids)
    if missing_ids or unexpected_ids:
        raise ObservationError(
            f"CAN 관절 구성이 정책과 다릅니다: "
            f"누락={missing_ids}, 추가={unexpected_ids}"
        )

    relative_positions: list[float] = []
    velocities: list[float] = []
    for motor_id, joint_name, default_position in zip(
        spec.motor_ids,
        spec.joint_names,
        spec.default_positions_rad,
    ):
        reading = can_cycle.readings[motor_id]
        if reading.position_rad is None or reading.velocity_rad_s is None:
            raise ObservationError(
                f"ID {motor_id} {joint_name} 위치 또는 속도가 누락되었습니다."
            )
        _validate_timestamp(reading.position_received_at_s, f"ID {motor_id} 위치")
        _validate_timestamp(reading.velocity_received_at_s, f"ID {motor_id} 속도")

        position = wrap_to_pi(float(reading.position_rad))
        lower, upper = JOINT_LIMITS_RAD[motor_id]
        if not lower <= position <= upper:
            raise ObservationError(
                f"ID {motor_id} {joint_name} 위치 {position:+.6f} rad가 "
                f"관절 범위 {lower:+.6f}..{upper:+.6f} rad 밖입니다."
            )

        velocity = float(reading.velocity_rad_s)
        if not math.isfinite(velocity):
            raise ObservationError(f"ID {motor_id} {joint_name} 속도가 NaN/inf입니다.")
        motor_spec = MOTOR_SPECS[MOTOR_MODELS[motor_id]]
        if not motor_spec.velocity_min <= velocity <= motor_spec.velocity_max:
            raise ObservationError(
                f"ID {motor_id} {joint_name} 속도 {velocity:+.6f} rad/s가 "
                f"모터 범위 {motor_spec.velocity_min:+.1f}.."
                f"{motor_spec.velocity_max:+.1f} rad/s 밖입니다."
            )

        relative_positions.append(position - default_position)
        velocities.append(velocity)

    angular_velocity = _finite_vector(
        "IMU raw 각속도",
        imu_reading.angular_velocity_raw,
        3,
    )
    if np.max(np.abs(angular_velocity)) > spec.max_abs_angular_velocity_rad_s:
        raise ObservationError(
            f"IMU raw 각속도가 범위를 넘었습니다: {angular_velocity.tolist()}"
        )

    gravity = _finite_vector("중력 벡터", imu_reading.projected_gravity, 3)
    gravity_norm = float(np.linalg.norm(gravity))
    if not spec.min_gravity_norm <= gravity_norm <= spec.max_gravity_norm:
        raise ObservationError(
            f"중력 벡터 크기가 범위를 벗어났습니다: {gravity_norm:.6f}"
        )
    if not math.isfinite(imu_reading.observed_at_s) or imu_reading.observed_at_s < 0.0:
        raise ObservationError("IMU 관측 시각이 없습니다.")

    action = _finite_vector("직전 action", previous_action, spec.joint_count)
    if np.max(np.abs(action)) > spec.runner_action_clip:
        raise ObservationError(
            f"직전 action이 ±{spec.runner_action_clip:g} clip 범위를 넘었습니다."
        )

    observation = np.concatenate(
        (
            np.asarray(relative_positions, dtype=np.float64),
            np.asarray(velocities, dtype=np.float64),
            angular_velocity,
            gravity,
            action,
        )
    ).astype(np.float32)
    if observation.shape != (spec.observation_size,) or not np.isfinite(observation).all():
        raise ObservationError(
            f"관측 벡터가 올바르지 않습니다: shape={observation.shape}"
        )
    return observation
