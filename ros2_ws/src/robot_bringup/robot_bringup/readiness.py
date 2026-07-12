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
    clock_jump_tolerance: float = 5.00

    def __post_init__(self):
        values = (
            self.freshness_timeout,
            self.tf_stable_duration,
            self.tf_translation_tolerance,
            self.tf_yaw_tolerance,
            self.clock_jump_tolerance,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError('readiness timing and tolerances must be positive')


@dataclass(frozen=True)
class ClockJump:
    """Describe a simulation-time epoch transition."""

    kind: str
    previous_stamp_s: float
    stamp_s: float
    epoch: int


class ReadinessTracker:
    """Track fresh inputs and a continuously stable map-to-odom transform."""

    def __init__(self, config):
        self.config = config
        self._clock_received_at = None
        self._clock_stamp_s = None
        self._clock_nonzero = False
        self._epoch = 0
        self._scan_received_at = None
        self._scan_stamp_s = None
        self._scan_epoch = None
        self._odom_received_at = None
        self._odom_stamp_s = None
        self._odom_epoch = None
        self._map_received = False
        self._tf_received_at = None
        self._tf_stamp_s = None
        self._tf_anchor = None
        self._tf_stable_since = None

    def mark_clock(self, stamp_s, received_at):
        """Record a clock sample and return an epoch jump when detected."""
        stamp_s = float(stamp_s)
        received_at = float(received_at)
        if not math.isfinite(stamp_s) or stamp_s < 0.0:
            raise ValueError('clock stamp must be finite and non-negative')
        if not math.isfinite(received_at):
            raise ValueError('clock receipt time must be finite')

        event = None
        if self._clock_stamp_s is not None:
            previous = self._clock_stamp_s
            if stamp_s < previous:
                event = self._start_new_epoch('rollback', previous, stamp_s)
            else:
                # A large stamp delta alone is not a discontinuity: /clock
                # samples may simply have been dropped while wall time kept
                # advancing. Only the advance unexplained by the wall-time
                # receipt gap starts a new epoch. Explicit reset events remain
                # the authoritative signal when simulation time is monotonic.
                wall_elapsed = max(
                    0.0,
                    received_at - self._clock_received_at,
                )
                unexplained_advance = stamp_s - previous - wall_elapsed
                if (unexplained_advance
                        > self.config.clock_jump_tolerance):
                    event = self._start_new_epoch(
                        'forward_jump', previous, stamp_s)
        self._clock_stamp_s = stamp_s
        self._clock_received_at = received_at
        self._clock_nonzero = stamp_s > 0.0
        return event

    def mark_scan(self, stamp_s, received_at):
        """Record a scan only when its header belongs to this clock epoch."""
        if not self._sample_is_in_current_epoch(stamp_s):
            return False
        self._scan_stamp_s = float(stamp_s)
        self._scan_received_at = received_at
        self._scan_epoch = self._epoch
        return True

    def mark_odom(self, stamp_s, received_at):
        """Record odometry only when its header belongs to this epoch."""
        if not self._sample_is_in_current_epoch(stamp_s):
            return False
        self._odom_stamp_s = float(stamp_s)
        self._odom_received_at = received_at
        self._odom_epoch = self._epoch
        return True

    def mark_map(self):
        """Record receipt of the transient-local occupancy map.

        A static map is intentionally latched, so receipt is required but wall
        clock freshness is not.
        """
        self._map_received = True

    def mark_reset(self):
        """Start a new readiness epoch for a reset without clock rollback."""
        stamp_s = self._clock_stamp_s if self._clock_stamp_s is not None else 0.0
        return self._start_new_epoch('reset', stamp_s, stamp_s)

    def observe_transform(self, x, y, yaw, stamp_s, received_at):
        """Record a newly stamped map-to-odom transform.

        Re-reading the same cached TF does not refresh its receipt time. This
        prevents a dead localization publisher from satisfying the gate.
        """
        transform = (float(x), float(y), _normalize_angle(float(yaw)))
        stamp_s = float(stamp_s)
        if (not all(math.isfinite(value) for value in transform)
                or not math.isfinite(stamp_s)
                or stamp_s <= 0.0
                or not self._sample_is_in_current_epoch(stamp_s)):
            self._clear_transform()
            return

        if self._tf_stamp_s is not None:
            if stamp_s == self._tf_stamp_s:
                return
            if stamp_s < self._tf_stamp_s:
                self._clear_transform()

        self._tf_stamp_s = stamp_s
        self._tf_received_at = received_at
        if (self._tf_anchor is None
                or self._tf_stable_since is None
                or not self._within_tolerance(transform)):
            self._tf_anchor = transform
            self._tf_stable_since = received_at

    def missing_requirements(self, now):
        """Return human-readable readiness conditions not currently met."""
        missing = []
        if not self._clock_nonzero:
            missing.append('non-zero /clock')
        if not self._fresh(self._clock_received_at, now):
            missing.append('fresh /clock')
        if not self._fresh_sample(
                self._scan_received_at, self._scan_stamp_s,
                self._scan_epoch, now):
            missing.append('fresh /scan')
        if not self._fresh_sample(
                self._odom_received_at, self._odom_stamp_s,
                self._odom_epoch, now):
            missing.append('fresh /odom')
        if not self._map_received:
            missing.append('latched /map')
        transform_fresh = (
            self._fresh(self._tf_received_at, now)
            and self._transform_is_fresh_in_sim_time()
        )
        if not transform_fresh:
            missing.append('map->odom transform')
            # Stability means a continuous interval of fresh transforms. Once
            # freshness is broken, a later sample must start a new window even
            # when its pose remains within the old tolerance anchor.
            self._tf_stable_since = None
        if (self._tf_stable_since is None
                or now - self._tf_stable_since
                < self.config.tf_stable_duration):
            missing.append('stable map->odom transform')
        return missing

    def ready(self, now):
        """Return true only when every activation precondition is met."""
        return not self.missing_requirements(now)

    @property
    def epoch(self):
        """Return the current simulation-time epoch number."""
        return self._epoch

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

    def _fresh_sample(self, received_at, stamp_s, epoch, now):
        return (
            epoch == self._epoch
            and stamp_s is not None
            and self._fresh(received_at, now)
            and self._clock_stamp_s is not None
            and abs(self._clock_stamp_s - stamp_s)
            <= self.config.freshness_timeout
        )

    def _sample_is_in_current_epoch(self, stamp_s):
        stamp_s = float(stamp_s)
        return (
            math.isfinite(stamp_s)
            and stamp_s >= 0.0
            and self._clock_stamp_s is not None
            and self._clock_nonzero
            and abs(self._clock_stamp_s - stamp_s)
            <= self.config.freshness_timeout
        )

    def _start_new_epoch(self, kind, previous_stamp_s, stamp_s):
        self._epoch += 1
        self._scan_received_at = None
        self._scan_stamp_s = None
        self._scan_epoch = None
        self._odom_received_at = None
        self._odom_stamp_s = None
        self._odom_epoch = None
        self._clear_transform()
        return ClockJump(kind, previous_stamp_s, stamp_s, self._epoch)

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
