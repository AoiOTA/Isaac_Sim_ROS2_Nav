"""Pure readiness tracking for the Nav2 activation gate."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ReadinessConfig:
    """Timing and transform tolerances for activation readiness."""

    freshness_timeout: float = 0.50
    tf_stable_duration: float = 1.00
    tf_translation_tolerance: float = 0.05
    tf_yaw_tolerance: float = 0.0523598776

    def __post_init__(self):
        values = (
            self.freshness_timeout,
            self.tf_stable_duration,
            self.tf_translation_tolerance,
            self.tf_yaw_tolerance,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError('readiness timing and tolerances must be positive')


class ReadinessTracker:
    """Track fresh inputs and a continuously stable map-to-odom transform."""

    def __init__(self, config):
        self.config = config
        self._clock_received_at = None
        self._clock_stamp_s = None
        self._clock_nonzero = False
        self._scan_received_at = None
        self._odom_received_at = None
        self._map_received = False
        self._tf_received_at = None
        self._tf_stamp_s = None
        self._tf_anchor = None
        self._tf_stable_since = None

    def mark_clock(self, stamp_s, received_at):
        """Record a clock sample and whether simulation time has started."""
        stamp_s = float(stamp_s)
        if (self._clock_stamp_s is not None
                and stamp_s < self._clock_stamp_s):
            self._clear_transform()
        self._clock_stamp_s = stamp_s
        self._clock_received_at = received_at
        self._clock_nonzero = stamp_s > 0.0

    def mark_scan(self, received_at):
        """Record receipt of a scan."""
        self._scan_received_at = received_at

    def mark_odom(self, received_at):
        """Record receipt of odometry."""
        self._odom_received_at = received_at

    def mark_map(self):
        """Record receipt of the transient-local occupancy map.

        A static map is intentionally latched, so receipt is required but wall
        clock freshness is not.
        """
        self._map_received = True

    def observe_transform(self, x, y, yaw, stamp_s, received_at):
        """Record a newly stamped map-to-odom transform.

        Re-reading the same cached TF does not refresh its receipt time. This
        prevents a dead localization publisher from satisfying the gate.
        """
        transform = (float(x), float(y), _normalize_angle(float(yaw)))
        stamp_s = float(stamp_s)
        if (not all(math.isfinite(value) for value in transform)
                or not math.isfinite(stamp_s)
                or stamp_s <= 0.0):
            self._clear_transform()
            return

        if self._tf_stamp_s is not None:
            if stamp_s == self._tf_stamp_s:
                return
            if stamp_s < self._tf_stamp_s:
                self._clear_transform()

        self._tf_stamp_s = stamp_s
        self._tf_received_at = received_at
        if self._tf_anchor is None or not self._within_tolerance(transform):
            self._tf_anchor = transform
            self._tf_stable_since = received_at

    def missing_requirements(self, now):
        """Return human-readable readiness conditions not currently met."""
        missing = []
        if not self._clock_nonzero:
            missing.append('non-zero /clock')
        if not self._fresh(self._clock_received_at, now):
            missing.append('fresh /clock')
        if not self._fresh(self._scan_received_at, now):
            missing.append('fresh /scan')
        if not self._fresh(self._odom_received_at, now):
            missing.append('fresh /odom')
        if not self._map_received:
            missing.append('latched /map')
        if (not self._fresh(self._tf_received_at, now)
                or not self._transform_is_fresh_in_sim_time()):
            missing.append('map->odom transform')
        if (self._tf_stable_since is None
                or now - self._tf_stable_since
                < self.config.tf_stable_duration):
            missing.append('stable map->odom transform')
        return missing

    def ready(self, now):
        """Return true only when every activation precondition is met."""
        return not self.missing_requirements(now)

    def _fresh(self, received_at, now):
        return (
            received_at is not None
            and 0.0 <= now - received_at <= self.config.freshness_timeout
        )

    def _transform_is_fresh_in_sim_time(self):
        if self._clock_stamp_s is None or self._tf_stamp_s is None:
            return False
        age = self._clock_stamp_s - self._tf_stamp_s
        # Localization broadcasters may intentionally future-date transforms
        # by their configured transform timeout. Both stale and implausibly
        # far-future samples are rejected by the same bounded tolerance.
        return abs(age) <= self.config.freshness_timeout

    def _clear_transform(self):
        self._tf_received_at = None
        self._tf_stamp_s = None
        self._tf_anchor = None
        self._tf_stable_since = None

    def _within_tolerance(self, transform):
        x, y, yaw = transform
        anchor_x, anchor_y, anchor_yaw = self._tf_anchor
        translation_error = math.hypot(x - anchor_x, y - anchor_y)
        yaw_error = abs(_normalize_angle(yaw - anchor_yaw))
        return (
            translation_error <= self.config.tf_translation_tolerance
            and yaw_error <= self.config.tf_yaw_tolerance
        )


def _normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))
