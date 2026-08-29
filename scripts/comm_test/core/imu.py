"""Non-blocking N100 IMU wrapper with the RoboNex mount correction."""

from __future__ import annotations

from dataclasses import dataclass
import importlib
import math
import time
from types import ModuleType

from .constants import DEFAULT_IMU_BAUDRATE, DEFAULT_IMU_PORT, IMU_MOUNT_ROLL_RAD


class ImuError(RuntimeError):
    pass


@dataclass(frozen=True)
class ImuReading:
    angular_velocity: tuple[float, float, float]
    angular_velocity_raw: tuple[float, float, float]
    projected_gravity: tuple[float, float, float]
    sequence: int
    host_timestamp_ns: int
    observed_at_s: float


def _vec3(value) -> tuple[float, float, float]:
    return float(value.x), float(value.y), float(value.z)


class N100Imu:
    def __init__(
        self,
        port: str = DEFAULT_IMU_PORT,
        baudrate: int = DEFAULT_IMU_BAUDRATE,
        mount_roll_rad: float = IMU_MOUNT_ROLL_RAD,
        n100_module: ModuleType | None = None,
    ) -> None:
        if not port:
            raise ValueError("IMU port is empty")
        if baudrate <= 0:
            raise ValueError(f"IMU baud rate must be positive: {baudrate}")
        if not math.isfinite(mount_roll_rad):
            raise ValueError(f"IMU mount angle must be finite: {mount_roll_rad}")
        self.port = port
        self.baudrate = baudrate
        self.mount_roll_rad = mount_roll_rad
        self._module = n100_module
        self._driver = None

    @property
    def is_running(self) -> bool:
        return self._driver is not None and bool(self._driver.is_running)

    def start(self, wait_timeout_s: float = 3.0) -> ImuReading:
        if self._driver is not None:
            raise ImuError("IMU is already running")
        if not math.isfinite(wait_timeout_s) or wait_timeout_s <= 0.0:
            raise ValueError(f"IMU timeout must be positive: {wait_timeout_s}")
        try:
            module = self._module or importlib.import_module("n100")
        except ImportError as error:
            raise ImuError("n100 module not found; build it with CMake first") from error

        config = module.DriverConfig(
            port=self.port,
            baudrate=self.baudrate,
            mount_rotation=module.Quat.from_axis_angle_x(self.mount_roll_rad),
        )
        driver = module.ImuDriver(config)
        self._driver = driver
        try:
            driver.start()
            sample = driver.wait_for_sample(timeout=wait_timeout_s)
            if sample is None:
                detail = driver.last_error() or "no response"
                raise ImuError(f"No IMU sample within {wait_timeout_s:.1f} seconds: {detail}")
            return self._convert(sample)
        except BaseException:
            self.close()
            raise

    def latest(self) -> ImuReading | None:
        if self._driver is None:
            raise ImuError("IMU is not running; call start() first")
        if not self._driver.is_running:
            raise ImuError(f"IMU reader stopped: {self._driver.last_error() or 'unknown error'}")
        sample = self._driver.latest()
        return None if sample is None else self._convert(sample)

    def close(self) -> None:
        driver, self._driver = self._driver, None
        if driver is not None:
            driver.stop()

    def __enter__(self) -> "N100Imu":
        self.start()
        return self

    def __exit__(self, _exc_type, _exc, _traceback) -> None:
        self.close()

    @staticmethod
    def _convert(sample) -> ImuReading:
        return ImuReading(
            angular_velocity=_vec3(sample.angular_velocity),
            angular_velocity_raw=_vec3(sample.angular_velocity_raw),
            projected_gravity=_vec3(sample.projected_gravity),
            sequence=int(sample.seq),
            host_timestamp_ns=int(sample.host_timestamp_ns),
            observed_at_s=time.monotonic(),
        )
