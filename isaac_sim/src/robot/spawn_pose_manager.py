"""Spawn-pose parsing, calibration gates, and robot reset operations."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import re
from typing import Protocol, Sequence

from isaac_sim.src.yaml_utils import (
    YamlConfigError,
    load_mapping,
    reject_unknown,
    require_keys,
    require_number,
    require_vector,
)


class SpawnPoseError(RuntimeError):
    pass


@dataclass(frozen=True)
class UsdSpawnPose:
    position: tuple[float, float, float]
    yaw_deg: float


@dataclass(frozen=True)
class MapSpawnPose:
    position: tuple[float, float]
    yaw_deg: float
    calibrated: bool
    position_stddev_m: float
    yaw_stddev_deg: float
    map_version: str | None
    map_bundle_sha256: str | None


@dataclass(frozen=True)
class SpawnPose:
    name: str
    usd: UsdSpawnPose
    map: MapSpawnPose


class RobotPosePort(Protocol):
    @property
    def num_dof(self) -> int: ...

    def set_world_pose(self, position: Sequence[float], orientation_wxyz: Sequence[float]) -> None: ...

    def set_base_velocities(self, linear: Sequence[float], angular: Sequence[float]) -> None: ...

    def set_joint_velocities(self, values: Sequence[float]) -> None: ...

    def set_joint_velocity_targets(self, values: Sequence[float]) -> None: ...


def quaternion_from_yaw_deg(yaw_deg: float) -> tuple[float, float, float, float]:
    half = math.radians(yaw_deg) * 0.5
    return (math.cos(half), 0.0, 0.0, math.sin(half))


def load_spawn_poses(path: str | Path) -> dict[str, SpawnPose]:
    data = load_mapping(path)
    reject_unknown(data, {"schema_version", "spawn_poses"}, context="spawn pose file")
    require_keys(data, {"schema_version", "spawn_poses"}, context="spawn pose file")
    if data["schema_version"] != 1:
        raise YamlConfigError("spawn pose schema_version must be 1")
    raw_poses = data["spawn_poses"]
    if not isinstance(raw_poses, dict) or not raw_poses:
        raise YamlConfigError("spawn_poses must be a non-empty mapping")
    parsed: dict[str, SpawnPose] = {}
    for name, raw in raw_poses.items():
        if not isinstance(name, str) or not name or not isinstance(raw, dict):
            raise YamlConfigError("spawn pose names and values must be valid mappings")
        reject_unknown(raw, {"usd", "map"}, context=f"spawn_poses.{name}")
        require_keys(raw, {"usd", "map"}, context=f"spawn_poses.{name}")
        usd = raw["usd"]
        map_pose = raw["map"]
        if not isinstance(usd, dict) or not isinstance(map_pose, dict):
            raise YamlConfigError(f"spawn_poses.{name}.usd/map must be mappings")
        reject_unknown(usd, {"position", "yaw_deg"}, context=f"spawn_poses.{name}.usd")
        reject_unknown(
            map_pose,
            {
                "position",
                "yaw_deg",
                "calibrated",
                "position_stddev_m",
                "yaw_stddev_deg",
                "map_version",
                "map_bundle_sha256",
            },
            context=f"spawn_poses.{name}.map",
        )
        require_keys(usd, {"position", "yaw_deg"}, context=f"spawn_poses.{name}.usd")
        require_keys(map_pose, {"position", "yaw_deg", "calibrated"}, context=f"spawn_poses.{name}.map")
        calibrated = map_pose["calibrated"]
        if not isinstance(calibrated, bool):
            raise YamlConfigError(f"spawn_poses.{name}.map.calibrated must be boolean")
        map_version = map_pose.get("map_version")
        map_bundle_sha256 = map_pose.get("map_bundle_sha256")
        if calibrated:
            if (
                not isinstance(map_version, str)
                or len(map_version) > 64
                or re.fullmatch(r"[A-Za-z0-9._-]+", map_version) is None
                or not any(character != "." for character in map_version)
            ):
                raise YamlConfigError(
                    f"spawn_poses.{name}.map.map_version must identify the "
                    "calibrated map"
                )
            if (
                not isinstance(map_bundle_sha256, str)
                or re.fullmatch(r"[0-9a-f]{64}", map_bundle_sha256) is None
            ):
                raise YamlConfigError(
                    f"spawn_poses.{name}.map.map_bundle_sha256 must bind the "
                    "calibrated map bundle"
                )
        position_stddev_m = require_number(
            map_pose.get("position_stddev_m", 0.05),
            context=f"spawn_poses.{name}.map.position_stddev_m",
        )
        yaw_stddev_deg = require_number(
            map_pose.get("yaw_stddev_deg", 5.0),
            context=f"spawn_poses.{name}.map.yaw_stddev_deg",
        )
        if position_stddev_m < 0.0 or yaw_stddev_deg < 0.0:
            raise YamlConfigError(
                f"spawn_poses.{name}.map standard deviations must be non-negative"
            )
        parsed[name] = SpawnPose(
            name=name,
            usd=UsdSpawnPose(
                position=require_vector(usd["position"], 3, context=f"spawn_poses.{name}.usd.position"),  # type: ignore[arg-type]
                yaw_deg=require_number(usd["yaw_deg"], context=f"spawn_poses.{name}.usd.yaw_deg"),
            ),
            map=MapSpawnPose(
                position=require_vector(map_pose["position"], 2, context=f"spawn_poses.{name}.map.position"),  # type: ignore[arg-type]
                yaw_deg=require_number(map_pose["yaw_deg"], context=f"spawn_poses.{name}.map.yaw_deg"),
                calibrated=calibrated,
                position_stddev_m=position_stddev_m,
                yaw_stddev_deg=yaw_stddev_deg,
                map_version=map_version,
                map_bundle_sha256=map_bundle_sha256,
            ),
        )
    return parsed


def require_map_calibration(pose: SpawnPose, purpose: str) -> None:
    if not pose.map.calibrated:
        raise SpawnPoseError(
            f"spawn pose {pose.name!r} has map.calibrated=false; {purpose} requires a measured map pose"
        )


class SpawnPoseManager:
    def __init__(self, robot: RobotPosePort, poses: dict[str, SpawnPose]):
        if not poses:
            raise SpawnPoseError("at least one spawn pose is required")
        self.robot = robot
        self.poses = poses

    def get(self, pose_name: str) -> SpawnPose:
        try:
            return self.poses[pose_name]
        except KeyError as exc:
            raise SpawnPoseError(f"unknown spawn pose {pose_name!r}; available={sorted(self.poses)}") from exc

    def apply_usd_pose(self, pose_name: str) -> SpawnPose:
        pose = self.get(pose_name)
        zeros = [0.0] * self.robot.num_dof
        self.robot.set_world_pose(pose.usd.position, quaternion_from_yaw_deg(pose.usd.yaw_deg))
        self.robot.set_base_velocities([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        self.robot.set_joint_velocities(zeros)
        self.robot.set_joint_velocity_targets(zeros)
        return pose

    def get_map_pose(self, pose_name: str, *, purpose: str) -> MapSpawnPose:
        pose = self.get(pose_name)
        require_map_calibration(pose, purpose)
        return pose.map
