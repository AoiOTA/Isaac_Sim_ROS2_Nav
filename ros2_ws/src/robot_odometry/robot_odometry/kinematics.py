"""Pure-Python four-wheel skid-steer odometry."""

from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple


DEFAULT_LEFT_JOINTS = (
    'front_left_wheel_joint',
    'rear_left_wheel_joint',
)
DEFAULT_RIGHT_JOINTS = (
    'front_right_wheel_joint',
    'rear_right_wheel_joint',
)

_YAW_GUARD_CONFIRM_SAMPLES = 3
_YAW_GUARD_CLEAR_SAMPLES = 3
_YAW_GUARD_EXIT_RATIO = 0.2
_YAW_GUARD_MAX_LINEAR_SPEED = 0.05


@dataclass(frozen=True)
class WheelOdometryConfig:
    """Geometry and safety bounds used by the integrator."""

    wheel_radius: float = 0.098
    track_width: float = 0.800
    left_joint_names: Tuple[str, ...] = DEFAULT_LEFT_JOINTS
    right_joint_names: Tuple[str, ...] = DEFAULT_RIGHT_JOINTS
    max_integration_step: float = 0.25
    yaw_disagreement_guard_enabled: bool = False
    yaw_disagreement_entry_threshold: float = 0.10
    yaw_disagreement_imu_timeout: float = 0.05

    def __post_init__(self):
        if not math.isfinite(self.wheel_radius) or self.wheel_radius <= 0.0:
            raise ValueError('wheel_radius must be finite and positive')
        if not math.isfinite(self.track_width) or self.track_width <= 0.0:
            raise ValueError('track_width must be finite and positive')
        if (not math.isfinite(self.max_integration_step)
                or self.max_integration_step <= 0.0):
            raise ValueError('max_integration_step must be finite and positive')
        if (not math.isfinite(self.yaw_disagreement_entry_threshold)
                or self.yaw_disagreement_entry_threshold <= 0.0):
            raise ValueError(
                'yaw_disagreement_entry_threshold must be finite and positive')
        if (not math.isfinite(self.yaw_disagreement_imu_timeout)
                or self.yaw_disagreement_imu_timeout < 0.0):
            raise ValueError(
                'yaw_disagreement_imu_timeout must be finite and non-negative')
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


class WheelYawDisagreementGuard:
    """Bound forward wheel speed during confirmed wheel/IMU yaw conflicts."""

    def __init__(self, *, enabled, entry_threshold, imu_timeout):
        self.enabled = bool(enabled)
        self.entry_threshold = float(entry_threshold)
        self.imu_timeout = float(imu_timeout)
        self.exit_threshold = _YAW_GUARD_EXIT_RATIO * self.entry_threshold
        self.reset()

    @property
    def active(self):
        """Return whether the confirmed disagreement state is active."""
        return self._active

    def reset(self):
        """Clear confirmation and hysteresis state."""
        self._active = False
        self._entry_count = 0
        self._clear_count = 0

    def apply(
        self,
        linear_velocity,
        wheel_angular_velocity,
        joint_stamp_s,
        imu_angular_velocity=None,
        imu_stamp_s=None,
    ):
        """Apply the fixed candidate-C policy to one wheel sample."""
        if not self.enabled:
            return linear_velocity

        imu_usable = (
            imu_angular_velocity is not None
            and imu_stamp_s is not None
            and math.isfinite(imu_angular_velocity)
            and math.isfinite(imu_stamp_s)
            and imu_stamp_s <= joint_stamp_s
            and joint_stamp_s - imu_stamp_s <= self.imu_timeout
        )
        signs_opposed = (
            imu_usable
            and wheel_angular_velocity * imu_angular_velocity < 0.0
        )
        entry = (
            signs_opposed
            and abs(wheel_angular_velocity) >= self.entry_threshold
            and abs(imu_angular_velocity) >= self.entry_threshold
        )

        if not self._active:
            self._entry_count = self._entry_count + 1 if entry else 0
            if self._entry_count >= _YAW_GUARD_CONFIRM_SAMPLES:
                self._active = True
                self._clear_count = 0
        else:
            min_rate = min(
                abs(wheel_angular_velocity),
                abs(imu_angular_velocity),
            ) if imu_usable else 0.0
            clear = (
                not imu_usable
                or not signs_opposed
                or min_rate <= self.exit_threshold
            )
            self._clear_count = self._clear_count + 1 if clear else 0
            if self._clear_count >= _YAW_GUARD_CLEAR_SAMPLES:
                self._active = False
                self._entry_count = 0

        # Missing, stale, future, or non-finite IMU is explicitly fail-open.
        if not self._active or not imu_usable:
            return linear_velocity
        return math.copysign(
            min(abs(linear_velocity), _YAW_GUARD_MAX_LINEAR_SPEED),
            linear_velocity,
        )


class WheelOdometry:
    """Integrate four wheel velocities into a planar odometry estimate."""

    def __init__(self, config: WheelOdometryConfig):
        self.config = config
        self._yaw_guard = WheelYawDisagreementGuard(
            enabled=config.yaw_disagreement_guard_enabled,
            entry_threshold=config.yaw_disagreement_entry_threshold,
            imu_timeout=config.yaw_disagreement_imu_timeout,
        )
        self.reset()

    @property
    def pose(self):
        """Return the current ``(x, y, yaw)`` pose."""
        return self._x, self._y, self._yaw

    @property
    def last_stamp_s(self):
        """Return the last consumed timestamp, if any."""
        return self._last_stamp_s

    @property
    def yaw_disagreement_guard_active(self):
        """Return the detector state for focused diagnostics and tests."""
        return self._yaw_guard.active

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
        self._yaw_guard.reset()

    def update(
        self,
        names: Sequence[str],
        velocities: Sequence[float],
        stamp_s: float,
        imu_angular_velocity: Optional[float] = None,
        imu_stamp_s: Optional[float] = None,
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
        linear = self._yaw_guard.apply(
            linear,
            angular,
            stamp_s,
            imu_angular_velocity,
            imu_stamp_s,
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
