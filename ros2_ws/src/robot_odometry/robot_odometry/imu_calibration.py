"""Pure timestamp and yaw-rate calibration for the estimated-state IMU."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ImuYawCalibrationConfig:
    """Bounded parameters for the raw-to-corrected IMU seam."""

    yaw_scale: float = 0.9294
    yaw_bias_rad_s: float = 0.0
    yaw_variance: float = 1.0e-4

    def __post_init__(self):
        if (
            not math.isfinite(self.yaw_scale)
            or not 0.5 <= self.yaw_scale <= 1.5
        ):
            raise ValueError('yaw_scale must be finite and within [0.5, 1.5]')
        if not math.isfinite(self.yaw_bias_rad_s):
            raise ValueError('yaw_bias_rad_s must be finite')
        if not math.isfinite(self.yaw_variance) or self.yaw_variance <= 0.0:
            raise ValueError('yaw_variance must be finite and positive')


@dataclass(frozen=True)
class ImuYawCalibrationResult:
    """One accepted correction or a fail-closed rejection."""

    accepted: bool
    reason: str
    angular_velocity_z: float | None = None


class ImuYawCalibration:
    """Reject unsafe stamps/rates and calibrate accepted yaw rates once."""

    def __init__(self, config: ImuYawCalibrationConfig):
        self.config = config
        self.last_stamp_ns: int | None = None
        self.counters = {
            'accepted': 0,
            'nonfinite': 0,
            'backward': 0,
            'duplicate': 0,
        }

    def calibrate(
        self, stamp_ns: int, raw_angular_velocity_z: float
    ) -> ImuYawCalibrationResult:
        """Return one corrected value without advancing on rejected input."""
        stamp_ns = int(stamp_ns)
        raw_angular_velocity_z = float(raw_angular_velocity_z)
        if not math.isfinite(raw_angular_velocity_z):
            return self._reject('nonfinite')
        if stamp_ns <= 0 or (
            self.last_stamp_ns is not None and stamp_ns < self.last_stamp_ns
        ):
            return self._reject('backward')
        if self.last_stamp_ns is not None and stamp_ns == self.last_stamp_ns:
            return self._reject('duplicate')

        corrected = (
            raw_angular_velocity_z - self.config.yaw_bias_rad_s
        ) * self.config.yaw_scale
        if not math.isfinite(corrected):
            return self._reject('nonfinite')
        self.last_stamp_ns = stamp_ns
        self.counters['accepted'] += 1
        return ImuYawCalibrationResult(True, 'accepted', corrected)

    def reset_stamp(self) -> None:
        """Accept the next positive stamp after an explicit simulation reset."""
        self.last_stamp_ns = None

    def _reject(self, reason: str) -> ImuYawCalibrationResult:
        self.counters[reason] += 1
        return ImuYawCalibrationResult(False, reason)
