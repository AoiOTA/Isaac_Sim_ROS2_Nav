"""Strict, environment-overridable project configuration.

Only standard Python and PyYAML are used here so configuration can be tested
with the system interpreter.  Isaac/Omniverse modules must never be imported
from this module.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import os
import re
from typing import Any, Mapping

import yaml


class ConfigError(ValueError):
    """Raised when configuration is incomplete, unknown, or inconsistent."""


_ENV_PATTERN = re.compile(r"\$(?:\{([A-Za-z_][A-Za-z0-9_]*)\}|([A-Za-z_][A-Za-z0-9_]*))")
_IDENTIFIER_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")
_PRIM_PATH_PATTERN = re.compile(
    r"^/(?:[A-Za-z_][A-Za-z0-9_]*)(?:/[A-Za-z_][A-Za-z0-9_]*)*$"
)


def project_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _expect_mapping(value: Any, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ConfigError(f"{name} must be a mapping")
    return dict(value)


def _expect_keys(value: Mapping[str, Any], allowed: set[str], name: str) -> None:
    unknown = sorted(set(value) - allowed)
    if unknown:
        raise ConfigError(f"unknown {name} keys: {unknown}")


def _required(value: Mapping[str, Any], key: str, name: str) -> Any:
    if key not in value:
        raise ConfigError(f"missing required key {name}.{key}")
    return value[key]


def _positive_number(value: Any, name: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or value <= 0
    ):
        raise ConfigError(f"{name} must be a positive number")
    return float(value)


def _absolute_prim(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value.startswith("/") or "//" in value:
        raise ConfigError(f"{name} must be an absolute USD prim path")
    return value


def _strict_prim_path(value: Any, name: str) -> str:
    path = _absolute_prim(value, name)
    if not _PRIM_PATH_PATTERN.fullmatch(path):
        raise ConfigError(f"{name} must be a valid absolute USD prim path")
    return path


def _expand_string(value: str, env: Mapping[str, str]) -> str:
    def replace(match: re.Match[str]) -> str:
        key = match.group(1) or match.group(2)
        if key not in env:
            raise ConfigError(f"environment variable {key} is required by configuration")
        return env[key]

    return _ENV_PATTERN.sub(replace, value)


def _expand(value: Any, env: Mapping[str, str]) -> Any:
    if isinstance(value, str):
        return _expand_string(value, env)
    if isinstance(value, list):
        return [_expand(item, env) for item in value]
    if isinstance(value, dict):
        return {key: _expand(item, env) for key, item in value.items()}
    return value


def _apply_nested_overrides(data: dict[str, Any], env: Mapping[str, str]) -> None:
    prefix = "ISAAC_NAV__"
    for env_key, raw_value in sorted(env.items()):
        if not env_key.startswith(prefix):
            continue
        path = [part.lower() for part in env_key[len(prefix) :].split("__") if part]
        if not path:
            raise ConfigError(f"invalid override name {env_key}")
        cursor: dict[str, Any] = data
        for part in path[:-1]:
            child = cursor.get(part)
            if not isinstance(child, dict):
                raise ConfigError(f"override {env_key} targets unknown mapping {part}")
            cursor = child
        leaf = path[-1]
        if leaf not in cursor:
            raise ConfigError(f"override {env_key} targets unknown key {leaf}")
        cursor[leaf] = yaml.safe_load(raw_value)


@dataclass(frozen=True)
class GroundColliderResolverConfig:
    """Strict recipe for resolving the colliders that act as ground."""

    required_prim_paths: tuple[str, ...]
    semantic_classes: tuple[str, ...]
    expected_enabled_count: int


@dataclass(frozen=True)
class EnvironmentConfig:
    identifier: str
    project_stage: Path
    source_asset: Path
    composition: str
    ground_colliders: GroundColliderResolverConfig


@dataclass(frozen=True)
class RobotConfig:
    asset_path: Path
    default_prim: str
    runtime_prim_path: str
    articulation_root: str
    base_link_prim: str
    wheel_joints: tuple[str, str, str, str]
    front_wheel_joints: tuple[str, str]
    rear_wheel_joints: tuple[str, str]


@dataclass(frozen=True)
class SimulationConfig:
    physics_hz: float
    rendering_hz: float
    expected_physics_scene: str
    headless: bool
    renderer: str
    navigation_mode: str
    odometry_mode: str
    structure_tf_source: str
    pacing_mode: str
    target_realtime_factor: float
    max_frames: int


@dataclass(frozen=True)
class SpawnConfig:
    poses_file: Path
    selected: str


@dataclass(frozen=True)
class Ros2Config:
    domain_id: int
    rmw_implementation: str
    namespace: str


@dataclass(frozen=True)
class GroundTruthConfig:
    enabled: bool
    frame_id: str
    child_frame_id: str
    odom_topic: str
    path_topic: str
    odom_hz: float
    path_hz: float


@dataclass(frozen=True)
class ConfigFiles:
    robot: Path
    lidar: Path
    imu: Path
    camera: Path
    topics: Path
    qos: Path
    dynamic_obstacles: Path
    ground_topology_profile: Path
    contact_profile: Path


@dataclass(frozen=True)
class ProjectConfig:
    schema_version: int
    asset_root: Path
    environment: EnvironmentConfig
    robot: RobotConfig
    simulation: SimulationConfig
    spawn: SpawnConfig
    ros2: Ros2Config
    ground_truth: GroundTruthConfig
    extensions: tuple[str, ...]
    files: ConfigFiles

    def require_runtime_paths(self) -> None:
        missing = [
            path
            for path in (
                self.asset_root,
                self.environment.source_asset,
                self.robot.asset_path,
                self.spawn.poses_file,
                self.files.robot,
                self.files.lidar,
                self.files.imu,
                self.files.camera,
                self.files.topics,
                self.files.qos,
                self.files.dynamic_obstacles,
                self.files.ground_topology_profile,
                self.files.contact_profile,
            )
            if not path.exists()
        ]
        if missing:
            joined = "\n".join(f"- {path}" for path in missing)
            raise ConfigError(f"required runtime paths are missing:\n{joined}")


def _parse_environment(raw: Any) -> EnvironmentConfig:
    data = _expect_mapping(raw, "environment")
    _expect_keys(
        data,
        {
            "id",
            "project_stage",
            "source_asset",
            "composition",
            "ground_colliders",
        },
        "environment",
    )
    identifier = _required(data, "id", "environment")
    if not isinstance(identifier, str) or not _IDENTIFIER_PATTERN.fullmatch(
        identifier
    ):
        raise ConfigError(
            "environment.id must be a path-safe identifier starting with an "
            "alphanumeric character"
        )
    composition = _required(data, "composition", "environment")
    if composition != "sublayer":
        raise ConfigError("environment.composition must be 'sublayer'")
    ground_data = _expect_mapping(
        _required(data, "ground_colliders", "environment"),
        "environment.ground_colliders",
    )
    _expect_keys(
        ground_data,
        {
            "required_prim_paths",
            "semantic_classes",
            "expected_enabled_count",
        },
        "environment.ground_colliders",
    )
    required_paths_raw = _required(
        ground_data,
        "required_prim_paths",
        "environment.ground_colliders",
    )
    if not isinstance(required_paths_raw, list) or not required_paths_raw:
        raise ConfigError(
            "environment.ground_colliders.required_prim_paths must be a "
            "non-empty list"
        )
    required_paths = tuple(
        _strict_prim_path(
            value,
            "environment.ground_colliders.required_prim_paths",
        )
        for value in required_paths_raw
    )
    if len(set(required_paths)) != len(required_paths):
        raise ConfigError(
            "environment.ground_colliders.required_prim_paths must not "
            "contain duplicates"
        )
    semantic_classes_raw = _required(
        ground_data,
        "semantic_classes",
        "environment.ground_colliders",
    )
    if not isinstance(semantic_classes_raw, list) or not all(
        isinstance(value, str) and _IDENTIFIER_PATTERN.fullmatch(value)
        for value in semantic_classes_raw
    ):
        raise ConfigError(
            "environment.ground_colliders.semantic_classes must be a list "
            "of path-safe identifiers"
        )
    semantic_classes = tuple(semantic_classes_raw)
    if len(set(semantic_classes)) != len(semantic_classes):
        raise ConfigError(
            "environment.ground_colliders.semantic_classes must not contain "
            "duplicates"
        )
    expected_count = _required(
        ground_data,
        "expected_enabled_count",
        "environment.ground_colliders",
    )
    if (
        isinstance(expected_count, bool)
        or not isinstance(expected_count, int)
        or expected_count < len(required_paths)
    ):
        raise ConfigError(
            "environment.ground_colliders.expected_enabled_count must be an "
            "integer no smaller than the required path count"
        )
    return EnvironmentConfig(
        identifier=identifier,
        project_stage=Path(_required(data, "project_stage", "environment")).resolve(),
        source_asset=Path(_required(data, "source_asset", "environment")).resolve(),
        composition=composition,
        ground_colliders=GroundColliderResolverConfig(
            required_prim_paths=required_paths,
            semantic_classes=semantic_classes,
            expected_enabled_count=expected_count,
        ),
    )


def _string_tuple(value: Any, size: int, name: str) -> tuple[str, ...]:
    if not isinstance(value, list) or len(value) != size or not all(isinstance(v, str) and v for v in value):
        raise ConfigError(f"{name} must contain exactly {size} non-empty strings")
    if len(set(value)) != size:
        raise ConfigError(f"{name} must not contain duplicates")
    return tuple(value)


def _parse_robot(raw: Any) -> RobotConfig:
    data = _expect_mapping(raw, "robot")
    allowed = {
        "asset_path",
        "default_prim",
        "runtime_prim_path",
        "articulation_root",
        "base_link_prim",
        "wheel_joints",
        "front_wheel_joints",
        "rear_wheel_joints",
    }
    _expect_keys(data, allowed, "robot")
    wheels = _string_tuple(_required(data, "wheel_joints", "robot"), 4, "robot.wheel_joints")
    front = _string_tuple(_required(data, "front_wheel_joints", "robot"), 2, "robot.front_wheel_joints")
    rear = _string_tuple(_required(data, "rear_wheel_joints", "robot"), 2, "robot.rear_wheel_joints")
    if set(front + rear) != set(wheels):
        raise ConfigError("front/rear wheel groups must partition robot.wheel_joints")
    default_prim = _required(data, "default_prim", "robot")
    if (
        not isinstance(default_prim, str)
        or not default_prim
        or "/" in default_prim
    ):
        raise ConfigError("robot.default_prim must be a non-empty USD prim name")
    return RobotConfig(
        asset_path=Path(_required(data, "asset_path", "robot")).resolve(),
        default_prim=default_prim,
        runtime_prim_path=_absolute_prim(_required(data, "runtime_prim_path", "robot"), "robot.runtime_prim_path"),
        articulation_root=_absolute_prim(_required(data, "articulation_root", "robot"), "robot.articulation_root"),
        base_link_prim=_absolute_prim(_required(data, "base_link_prim", "robot"), "robot.base_link_prim"),
        wheel_joints=wheels,  # type: ignore[arg-type]
        front_wheel_joints=front,  # type: ignore[arg-type]
        rear_wheel_joints=rear,  # type: ignore[arg-type]
    )


def _parse_simulation(raw: Any) -> SimulationConfig:
    data = _expect_mapping(raw, "simulation")
    allowed = {
        "physics_hz",
        "rendering_hz",
        "expected_physics_scene",
        "headless",
        "renderer",
        "navigation_mode",
        "odometry_mode",
        "structure_tf_source",
        "pacing_mode",
        "target_realtime_factor",
        "max_frames",
    }
    _expect_keys(data, allowed, "simulation")
    headless = _required(data, "headless", "simulation")
    if not isinstance(headless, bool):
        raise ConfigError("simulation.headless must be boolean")
    navigation_mode = _required(data, "navigation_mode", "simulation")
    if navigation_mode not in {"mapping", "localization"}:
        raise ConfigError("simulation.navigation_mode must be mapping or localization")
    odometry_mode = _required(data, "odometry_mode", "simulation")
    if odometry_mode not in {"ideal", "realistic"}:
        raise ConfigError("simulation.odometry_mode must be ideal or realistic")
    structure_tf_source = _required(data, "structure_tf_source", "simulation")
    if structure_tf_source not in {"isaac", "rsp"}:
        raise ConfigError("simulation.structure_tf_source must be isaac or rsp")
    if odometry_mode == "ideal" and structure_tf_source == "rsp":
        raise ConfigError(
            "ideal odometry requires Isaac-owned structure TF; "
            "structure_tf_source=rsp is valid only in realistic mode"
        )
    pacing_mode = _required(data, "pacing_mode", "simulation")
    if pacing_mode not in {"realtime", "unbounded"}:
        raise ConfigError(
            "simulation.pacing_mode must be realtime or unbounded"
        )
    max_frames = _required(data, "max_frames", "simulation")
    if isinstance(max_frames, bool) or not isinstance(max_frames, int) or max_frames < 0:
        raise ConfigError("simulation.max_frames must be a non-negative integer")
    renderer = _required(data, "renderer", "simulation")
    if renderer not in {"RaytracedLighting", "PathTracing"}:
        raise ConfigError("simulation.renderer is unsupported")
    return SimulationConfig(
        physics_hz=_positive_number(_required(data, "physics_hz", "simulation"), "simulation.physics_hz"),
        rendering_hz=_positive_number(
            _required(data, "rendering_hz", "simulation"), "simulation.rendering_hz"
        ),
        expected_physics_scene=_absolute_prim(
            _required(data, "expected_physics_scene", "simulation"), "simulation.expected_physics_scene"
        ),
        headless=headless,
        renderer=renderer,
        navigation_mode=navigation_mode,
        odometry_mode=odometry_mode,
        structure_tf_source=structure_tf_source,
        pacing_mode=pacing_mode,
        target_realtime_factor=_positive_number(
            _required(data, "target_realtime_factor", "simulation"),
            "simulation.target_realtime_factor",
        ),
        max_frames=max_frames,
    )


def _parse_spawn(raw: Any) -> SpawnConfig:
    data = _expect_mapping(raw, "spawn")
    _expect_keys(data, {"poses_file", "selected"}, "spawn")
    selected = _required(data, "selected", "spawn")
    if not isinstance(selected, str) or not selected:
        raise ConfigError("spawn.selected must be non-empty")
    return SpawnConfig(Path(_required(data, "poses_file", "spawn")).resolve(), selected)


def _parse_ros2(raw: Any) -> Ros2Config:
    data = _expect_mapping(raw, "ros2")
    _expect_keys(data, {"domain_id", "rmw_implementation", "namespace"}, "ros2")
    domain_id = _required(data, "domain_id", "ros2")
    if isinstance(domain_id, bool) or not isinstance(domain_id, int) or not 0 <= domain_id <= 232:
        raise ConfigError("ros2.domain_id must be an integer in [0, 232]")
    rmw = _required(data, "rmw_implementation", "ros2")
    if rmw != "rmw_fastrtps_cpp":
        raise ConfigError("ros2.rmw_implementation must be rmw_fastrtps_cpp")
    namespace = _required(data, "namespace", "ros2")
    if not isinstance(namespace, str):
        raise ConfigError("ros2.namespace must be a string")
    return Ros2Config(domain_id, rmw, namespace.strip("/"))


def _parse_ground_truth(raw: Any) -> GroundTruthConfig:
    data = _expect_mapping(raw, "ground_truth")
    allowed = {"enabled", "frame_id", "child_frame_id", "odom_topic", "path_topic", "odom_hz", "path_hz"}
    _expect_keys(data, allowed, "ground_truth")
    enabled = _required(data, "enabled", "ground_truth")
    if not isinstance(enabled, bool):
        raise ConfigError("ground_truth.enabled must be boolean")
    frame_id = _required(data, "frame_id", "ground_truth")
    if frame_id != "map":
        raise ConfigError("ground_truth.frame_id must be map; USD/world frames are not ROS frames")
    return GroundTruthConfig(
        enabled=enabled,
        frame_id=frame_id,
        child_frame_id=str(_required(data, "child_frame_id", "ground_truth")),
        odom_topic=str(_required(data, "odom_topic", "ground_truth")),
        path_topic=str(_required(data, "path_topic", "ground_truth")),
        odom_hz=_positive_number(_required(data, "odom_hz", "ground_truth"), "ground_truth.odom_hz"),
        path_hz=_positive_number(_required(data, "path_hz", "ground_truth"), "ground_truth.path_hz"),
    )


def _parse_files(raw: Any) -> ConfigFiles:
    data = _expect_mapping(raw, "files")
    allowed = {
        "robot",
        "lidar",
        "imu",
        "camera",
        "topics",
        "qos",
        "dynamic_obstacles",
        "ground_topology_profile",
        "contact_profile",
    }
    _expect_keys(data, allowed, "files")
    return ConfigFiles(**{key: Path(_required(data, key, "files")).resolve() for key in sorted(allowed)})


def load_project_config(path: str | Path | None = None, env: Mapping[str, str] | None = None) -> ProjectConfig:
    """Load and strictly validate ``project.yaml``.

    Generic overrides use ``ISAAC_NAV__SECTION__FIELD`` and are parsed as YAML
    scalars. Unknown override paths are errors, preventing misspelled settings
    from being silently ignored.
    """

    path = Path(path) if path is not None else project_root() / "isaac_sim/configs/project.yaml"
    effective_env = dict(os.environ if env is None else env)
    effective_env.setdefault("PROJECT_ROOT", str(project_root()))
    effective_env.setdefault(
        "ISAAC_ASSET_ROOT",
        effective_env.get("ISAAC_NAV__ASSET_ROOT", "/home/lyb/isaacsim_assets/Assets/Isaac/6.0"),
    )
    with path.open("r", encoding="utf-8") as stream:
        loaded = yaml.safe_load(stream)
    data = _expect_mapping(loaded, "project")
    data = _expand(data, effective_env)
    _apply_nested_overrides(data, effective_env)
    allowed = {
        "schema_version",
        "asset_root",
        "environment",
        "robot",
        "simulation",
        "spawn",
        "ros2",
        "ground_truth",
        "extensions",
        "files",
    }
    _expect_keys(data, allowed, "project")
    version = _required(data, "schema_version", "project")
    if version != 1:
        raise ConfigError(f"unsupported schema_version {version!r}")
    extensions = _required(data, "extensions", "project")
    if not isinstance(extensions, list) or not extensions or not all(isinstance(v, str) and v for v in extensions):
        raise ConfigError("extensions must be a non-empty list of extension IDs")
    if len(set(extensions)) != len(extensions):
        raise ConfigError("extensions must not contain duplicates")
    return ProjectConfig(
        schema_version=version,
        asset_root=Path(_required(data, "asset_root", "project")).resolve(),
        environment=_parse_environment(_required(data, "environment", "project")),
        robot=_parse_robot(_required(data, "robot", "project")),
        simulation=_parse_simulation(_required(data, "simulation", "project")),
        spawn=_parse_spawn(_required(data, "spawn", "project")),
        ros2=_parse_ros2(_required(data, "ros2", "project")),
        ground_truth=_parse_ground_truth(_required(data, "ground_truth", "project")),
        extensions=tuple(extensions),
        files=_parse_files(_required(data, "files", "project")),
    )


def configure_process_environment(config: ProjectConfig) -> None:
    """Set bridge-critical environment values before enabling the bridge."""

    os.environ["ROS_DOMAIN_ID"] = str(config.ros2.domain_id)
    os.environ["RMW_IMPLEMENTATION"] = config.ros2.rmw_implementation
