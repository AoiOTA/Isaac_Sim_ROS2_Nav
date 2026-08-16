"""Pure metric helpers for the odometry calibration driver."""

from dataclasses import dataclass
import math
from typing import Tuple


@dataclass(frozen=True)
class PlanarPose:
    """A planar pose sampled from one odometry source."""

    x: float
    y: float
    yaw: float


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Return the planar yaw component of a quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(angle: float) -> float:
    """Wrap an angle to the half-open interval (-pi, pi]."""
    wrapped = (angle + math.pi) % (2.0 * math.pi) - math.pi
    return wrapped if wrapped != -math.pi else math.pi


def segment_motion(
    start: PlanarPose,
    end: PlanarPose,
) -> Tuple[float, float, float]:
    """
    Measure one segment in the source's own start-heading frame.

    Returns ``(forward, lateral, dyaw)``: translation along and
    perpendicular to the start heading, plus the wrapped yaw change.
    Comparing the same triple across sources (ground truth, EKF, wheel
    odometry) is frame-independent because each source is measured in its
    own start pose.
    """
    dx = end.x - start.x
    dy = end.y - start.y
    cos_yaw = math.cos(start.yaw)
    sin_yaw = math.sin(start.yaw)
    forward = cos_yaw * dx + sin_yaw * dy
    lateral = -sin_yaw * dx + cos_yaw * dy
    dyaw = wrap_angle(end.yaw - start.yaw)
    return forward, lateral, dyaw
