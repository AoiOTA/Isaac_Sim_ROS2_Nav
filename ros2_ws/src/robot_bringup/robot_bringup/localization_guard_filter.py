"""
Continuity filter for AMCL-derived map->odom corrections.

Pure state machine, ROS-free for unit testing.  AMCL is subject to mode
capture in self-similar corridors: every few runs it snaps 20+ degrees and
0.2-0.4 m onto a wrong alignment and, in this apartment, never recovers
before the frozen Nav2 margins turn the jump into a route failure.  The
EKF odometry is far better behaved (~1 percent linear), so the safest
published map->odom is one that accepts small continuous AMCL corrections,
freezes through large discontinuities, deliberately re-bases only when a
far candidate stays stable (a genuine relocalization such as the reset
reseed), and resumes tracking when candidates return near the frozen
estimate.
"""

from dataclasses import dataclass
import math


STATE_INIT = 'init'
STATE_TRACKING = 'tracking'
STATE_HOLDING = 'holding'


def wrap_angle(angle: float) -> float:
    """Wrap an angle to the half-open interval (-pi, pi]."""
    wrapped = (angle + math.pi) % (2.0 * math.pi) - math.pi
    return wrapped if wrapped != -math.pi else math.pi


@dataclass(frozen=True)
class PlanarPose:
    """A planar pose (map->odom expressed as x, y, yaw)."""

    x: float
    y: float
    yaw: float

    def delta_to(self, other: 'PlanarPose') -> 'PlanarPose':
        """Return the transform that takes ``self`` to ``other`` in self's frame."""
        dx = other.x - self.x
        dy = other.y - self.y
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        return PlanarPose(
            x=cos_yaw * dx + sin_yaw * dy,
            y=-sin_yaw * dx + cos_yaw * dy,
            yaw=wrap_angle(other.yaw - self.yaw),
        )


@dataclass(frozen=True)
class GuardConfig:
    """Thresholds for the continuity state machine."""

    accept_translation_m: float = 0.08
    accept_yaw_deg: float = 3.0
    far_translation_m: float = 0.25
    far_yaw_deg: float = 10.0
    far_accept_samples: int = 30
    resume_samples: int = 5
    blend_rate: float = 0.5

    def __post_init__(self):
        values = (
            self.accept_translation_m,
            self.accept_yaw_deg,
            self.far_translation_m,
            self.far_yaw_deg,
            self.blend_rate,
        )
        if not all(math.isfinite(v) and v > 0.0 for v in values):
            raise ValueError('guard thresholds must be finite and positive')
        if self.far_translation_m <= self.accept_translation_m:
            raise ValueError('far threshold must exceed accept threshold')
        if self.far_yaw_deg <= self.accept_yaw_deg:
            raise ValueError('far yaw threshold must exceed accept yaw')
        if self.blend_rate > 1.0:
            raise ValueError('blend_rate must not exceed 1')
        if self.far_accept_samples < 1 or self.resume_samples < 1:
            raise ValueError('sample counters must be positive')


class ContinuityGuard:
    """Accept small AMCL corrections, freeze through jumps, rebase on evidence."""

    def __init__(self, config: GuardConfig):
        self.config = config
        self.state = STATE_INIT
        self._estimate = None
        self._far_streak = 0
        self._resume_streak = 0

    @property
    def estimate(self):
        """Return the currently published PlanarPose (None until initialized)."""
        return self._estimate

    def reset(self) -> None:
        """Drop all state (simulation reset; next candidate re-initializes)."""
        self.state = STATE_INIT
        self._estimate = None
        self._far_streak = 0
        self._resume_streak = 0

    def observe(self, candidate: PlanarPose) -> str:
        """Consume one AMCL-derived map->odom candidate; return the decision."""
        if self.state == STATE_INIT or self._estimate is None:
            self._estimate = candidate
            self.state = STATE_TRACKING
            self._far_streak = 0
            self._resume_streak = 0
            return 'initialize'

        delta = self._estimate.delta_to(candidate)
        jump_translation = math.hypot(delta.x, delta.y)
        jump_yaw = abs(math.degrees(delta.yaw))

        near = (
            jump_translation < self.config.accept_translation_m
            and jump_yaw < self.config.accept_yaw_deg
        )
        far = (
            jump_translation > self.config.far_translation_m
            or jump_yaw > self.config.far_yaw_deg
        )

        if near:
            # Blend toward the candidate in world coordinates; the yaw term
            # is wrapped so the blend never takes the long way around.
            rate = self.config.blend_rate
            self._estimate = PlanarPose(
                self._estimate.x + (candidate.x - self._estimate.x) * rate,
                self._estimate.y + (candidate.y - self._estimate.y) * rate,
                wrap_angle(
                    self._estimate.yaw
                    + wrap_angle(candidate.yaw - self._estimate.yaw) * rate
                ),
            )
            self._resume_streak += 1
            self._far_streak = 0
            if self.state == STATE_HOLDING and (
                self._resume_streak >= self.config.resume_samples
            ):
                self.state = STATE_TRACKING
                self._resume_streak = 0
                return 'resume'
            return 'accept'

        if far:
            self._far_streak += 1
            self._resume_streak = 0
            if self._far_streak >= self.config.far_accept_samples:
                self._estimate = candidate
                self.state = STATE_TRACKING
                self._far_streak = 0
                return 'rebase'
            self.state = STATE_HOLDING
            return 'hold_far'

        self._far_streak = 0
        self._resume_streak = 0
        self.state = STATE_HOLDING
        return 'hold_mid'
