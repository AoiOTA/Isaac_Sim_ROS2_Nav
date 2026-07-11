"""Load calibrated USD/map spawn-pose pairs without importing ROS."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from .configuration import (
    ConfigurationError,
    load_yaml_mapping,
    require_finite,
    require_mapping,
    require_vector,
)


@dataclass(frozen=True)
class PoseDefinition:
    position: tuple[float, ...]
    yaw_deg: float

    def as_dict(self) -> dict[str, object]:
        return {"position": list(self.position), "yaw_deg": self.yaw_deg}


@dataclass(frozen=True)
class SpawnPose:
    name: str
    usd: PoseDefinition
    map: PoseDefinition
    map_calibrated: bool
    position_stddev_m: float
    yaw_stddev_deg: float


def load_spawn_pose(
    path: str | Path,
    pose_name: str,
    *,
    require_calibrated: bool = True,
) -> SpawnPose:
    document = load_yaml_mapping(path)
    if document.get("schema_version") != 1:
        raise ConfigurationError("spawn pose schema_version must be 1")
    poses = require_mapping(document.get("spawn_poses"), "spawn_poses")
    if pose_name not in poses:
        available = ", ".join(sorted(str(name) for name in poses)) or "<none>"
        raise ConfigurationError(f"unknown spawn pose {pose_name!r}; available: {available}")
    pose = require_mapping(poses[pose_name], f"spawn_poses.{pose_name}")
    usd = require_mapping(pose.get("usd"), f"spawn_poses.{pose_name}.usd")
    map_pose = require_mapping(pose.get("map"), f"spawn_poses.{pose_name}.map")
    calibrated = map_pose.get("calibrated")
    if not isinstance(calibrated, bool):
        raise ConfigurationError(f"spawn_poses.{pose_name}.map.calibrated must be boolean")
    if require_calibrated and calibrated is not True:
        raise ConfigurationError(
            f"spawn pose {pose_name!r} has no calibrated map pose; refusing localization"
        )
    position_stddev = require_finite(
        map_pose.get("position_stddev_m", 0.05),
        f"spawn_poses.{pose_name}.map.position_stddev_m",
    )
    yaw_stddev = require_finite(
        map_pose.get("yaw_stddev_deg", 5.0),
        f"spawn_poses.{pose_name}.map.yaw_stddev_deg",
    )
    if position_stddev < 0.0 or yaw_stddev < 0.0:
        raise ConfigurationError("spawn-pose standard deviations must be non-negative")
    return SpawnPose(
        name=pose_name,
        usd=PoseDefinition(
            position=require_vector(usd.get("position"), 3, f"spawn_poses.{pose_name}.usd.position"),
            yaw_deg=require_finite(usd.get("yaw_deg"), f"spawn_poses.{pose_name}.usd.yaw_deg"),
        ),
        map=PoseDefinition(
            position=require_vector(
                map_pose.get("position"), 2, f"spawn_poses.{pose_name}.map.position"
            ),
            yaw_deg=require_finite(
                map_pose.get("yaw_deg"), f"spawn_poses.{pose_name}.map.yaw_deg"
            ),
        ),
        map_calibrated=calibrated,
        position_stddev_m=position_stddev,
        yaw_stddev_deg=yaw_stddev,
    )
