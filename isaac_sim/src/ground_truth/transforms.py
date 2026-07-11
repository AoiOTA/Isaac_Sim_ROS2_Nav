"""Planar map/USD transform math used by ground-truth evaluation."""

from __future__ import annotations

from dataclasses import dataclass
import math


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float

    @classmethod
    def from_degrees(cls, x: float, y: float, yaw_deg: float) -> "Pose2D":
        return cls(float(x), float(y), math.radians(float(yaw_deg)))


def compose(parent_t_child: Pose2D, child_t_object: Pose2D) -> Pose2D:
    c = math.cos(parent_t_child.yaw)
    s = math.sin(parent_t_child.yaw)
    return Pose2D(
        parent_t_child.x + c * child_t_object.x - s * child_t_object.y,
        parent_t_child.y + s * child_t_object.x + c * child_t_object.y,
        wrap_angle(parent_t_child.yaw + child_t_object.yaw),
    )


def inverse(transform: Pose2D) -> Pose2D:
    c = math.cos(transform.yaw)
    s = math.sin(transform.yaw)
    return Pose2D(
        -c * transform.x - s * transform.y,
        s * transform.x - c * transform.y,
        wrap_angle(-transform.yaw),
    )


def compute_map_t_usd(usd_t_base_start: Pose2D, map_t_base_start: Pose2D) -> Pose2D:
    """Return ``map_T_usd = map_T_base_start * inverse(usd_T_base_start)``."""

    return compose(map_t_base_start, inverse(usd_t_base_start))


def usd_pose_to_map(map_t_usd: Pose2D, usd_t_base: Pose2D) -> Pose2D:
    return compose(map_t_usd, usd_t_base)
