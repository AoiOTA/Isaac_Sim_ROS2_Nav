"""Pure-Python four-wheel skid-steer odometry."""

from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple


@dataclass(frozen=True)
class WheelOdometryConfig:
    """Geometry and safety bounds used by the integrator."""

    wheel_radius: float
    track_width: float
    left_joint_names: Tuple[str, ...]
    right_joint_names: Tuple[str, ...]
    max_integration_step: float = 0.25

    def __post_init__(self):
        if not math.isfinite(self.wheel_radius) or self.wheel_radius <= 0.0:
            raise ValueError('wheel_radius must be finite and positive')
        if not math.isfinite(self.track_width) or self.track_width <= 0.0:
            raise ValueError('track_width must be finite and positive')
        if (not math.isfinite(self.max_integration_step)
                or self.max_integration_step <= 0.0):
            raise ValueError('max_integration_step must be finite and positive')
        all_joints = self.left_joint_names + self.right_joint_names
        if not self.left_joint_names or not self.right_joint_names:
            raise ValueError('both sides must contain at least one wheel joint')
        if len(set(all_joints)) != len(all_joints):
            raise ValueError('wheel joint names must be unique')


@dataclass(frozen=True)
class OdometrySample:
    """Integrated planar pose and the velocity that produced it."""

    stamp_s: float
    x: float
    y: float
    yaw: float
    linear_velocity: float
    angular_velocity: float
    dt: float


@dataclass(frozen=True)
class UpdateResult:
    """Result of one attempted integration step."""

    accepted: bool
    reason: str
    sample: Optional[OdometrySample]


class WheelOdometry:
    """Integrate four wheel velocities into a planar odometry estimate."""

    def __init__(self, config: WheelOdometryConfig):
        self.config = config
        self.reset()

    @property
    def pose(self):
        """Return the current ``(x, y, yaw)`` pose."""
        return self._x, self._y, self._yaw

    @property
    def last_stamp_s(self):
        """Return the last consumed timestamp, if any."""
        return self._last_stamp_s

    def reset(self, x=0.0, y=0.0, yaw=0.0, stamp_s=None):
        """Reset pose and timestamp without retaining a stale velocity."""
        values = (x, y, yaw)
        if not all(math.isfinite(value) for value in values):
            raise ValueError('reset pose must contain only finite values')
        if stamp_s is not None and not math.isfinite(stamp_s):
            raise ValueError('reset timestamp must be finite')
        self._x = float(x)
        self._y = float(y)
        self._yaw = _normalize_angle(float(yaw))
        self._last_stamp_s = stamp_s

    def update(
        self,
        names: Sequence[str],
        velocities: Sequence[float],
        stamp_s: float,
    ) -> UpdateResult:
        """Consume a complete wheel sample and integrate it safely."""
        if not math.isfinite(stamp_s):
            return UpdateResult(False, 'invalid_stamp', None)

        if self._last_stamp_s is not None and stamp_s < self._last_stamp_s:
            self.reset(stamp_s=stamp_s)
            return UpdateResult(False, 'time_regression_reset', None)

        if len(names) != len(velocities):
            self._last_stamp_s = stamp_s
            return UpdateResult(False, 'name_velocity_length_mismatch', None)

        wheel_velocity = dict(zip(names, velocities))
        required = self.config.left_joint_names + self.config.right_joint_names
        if any(name not in wheel_velocity for name in required):
            self._last_stamp_s = stamp_s
            return UpdateResult(False, 'missing_required_joint', None)

        selected = [wheel_velocity[name] for name in required]
        if not all(math.isfinite(value) for value in selected):
            self._last_stamp_s = stamp_s
            return UpdateResult(False, 'invalid_wheel_velocity', None)

        left = _mean(
            [wheel_velocity[name] for name in self.config.left_joint_names])
        right = _mean(
            [wheel_velocity[name] for name in self.config.right_joint_names])
        linear = self.config.wheel_radius * (left + right) * 0.5
        angular = (
            self.config.wheel_radius * (right - left)
            / self.config.track_width
        )

        if self._last_stamp_s is None:
            self._last_stamp_s = stamp_s
            return UpdateResult(True, 'initialized', self._sample(
                stamp_s, linear, angular, 0.0))

        dt = stamp_s - self._last_stamp_s
        self._last_stamp_s = stamp_s
        if dt == 0.0:
            return UpdateResult(False, 'duplicate_stamp', None)
        if dt > self.config.max_integration_step:
            return UpdateResult(False, 'integration_gap_skipped', None)

        heading_midpoint = self._yaw + 0.5 * angular * dt
        self._x += linear * math.cos(heading_midpoint) * dt
        self._y += linear * math.sin(heading_midpoint) * dt
        self._yaw = _normalize_angle(self._yaw + angular * dt)
        return UpdateResult(True, 'integrated', self._sample(
            stamp_s, linear, angular, dt))

    def _sample(self, stamp_s, linear, angular, dt):
        return OdometrySample(
            stamp_s=stamp_s,
            x=self._x,
            y=self._y,
            yaw=self._yaw,
            linear_velocity=linear,
            angular_velocity=angular,
            dt=dt,
        )


def covariance_from_diagonal(diagonal: Sequence[float]):
    """Expand six diagonal values into a ROS 6x6 covariance array."""
    if len(diagonal) != 6:
        raise ValueError('covariance diagonal must contain six values')
    if not all(math.isfinite(value) and value >= 0.0 for value in diagonal):
        raise ValueError('covariance values must be finite and non-negative')
    covariance = [0.0] * 36
    for index, value in enumerate(diagonal):
        covariance[index * 6 + index] = float(value)
    return covariance


def _mean(values):
    return sum(values) / len(values)


def _normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))
