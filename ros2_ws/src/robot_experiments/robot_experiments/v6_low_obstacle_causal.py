"""V6 M0--M3 low-obstacle runner, recorder, and offline evaluator.

The live command uses three explicit process adapters and reuses the existing
V6 formal episode node for reset, route dispatch, and terminal zero.  Ground
Truth is recorded passively and is consumed only after the episode.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import math
import os
from pathlib import Path
import shlex
import signal
import statistics
import struct
import subprocess
import sys
import time
from typing import Any, Iterable, Mapping, Sequence

import yaml


SCHEMA_VERSION = "bio_nav_v6_low_obstacle_causal_manifest_v1"
QUALIFICATION = "ENGINEERING_CAUSAL_NOT_RUN"
RUN_QUALIFICATION = "ENGINEERING_CAUSAL_RUN_UNEVALUATED"
EXPECTED_ORDER = (
    "M0", "M1", "M2", "M3",
    "M3", "M2", "M1", "M0",
    "M1", "M3", "M0", "M2",
)
GT_PREFIX = "/" + "ground_truth/"

# The runtime dispatcher/recorder is estimated-state only.  Passive Ground
# Truth evidence is captured by a separate process and joined offline.
DISPATCHER_TOPICS = (
    "/odom",
    "/amcl_pose",
    "/scan",
    "/camera/front/depth/image_raw",
    "/camera/front/camera_info",
    "/tf",
    "/tf_static",
    "/bio_nav/module2/cognitive_obstacles",
    "/bio_nav/cognitive_obstacle_layer/status",
    "/bio_nav/local_risk_layer/status",
    "/bio_nav/cognitive_risk_critic/status",
    "/bio_nav/module2/planning_prior",
    "/bio_nav/module2/goal_planning_prior",
    "/global_costmap/costmap",
    "/local_costmap/costmap",
    "/plan",
    "/optimal_trajectory",
    "/cmd_vel",
    "/bio_nav/canonical_route",
    "/bio_nav/route_progress",
    "/bio_nav/route_goal_complete",
)
PASSIVE_EVALUATOR_TOPICS = (
    "/ground_truth/odom",
    "/simulation/collision",
    "/simulation/collision_diagnostics",
)

# These topics are captured solely to prove that the Phase-F isolation knobs
# remained off.  They are not consumed by the dispatcher.
ISOLATION_AUDIT_TOPICS = (
    "/bio_nav/module2/edge_priors",
    "/bio_nav/module2/cognitive_place_graph",
)

PILOT_ARMS = ("M0", "M1", "M2", "M3")
DEFAULT_SYNC_TOLERANCE_NS = 100_000_000
DEFAULT_SHUTDOWN_TIMEOUT_SEC = 20.0
MODULE3_RESOURCE_PREFIX = "module3://"
MODULE3_ROOT_ENV = "BIO_NAV_MODULE3_ROOT"

if any(topic.startswith(GT_PREFIX) for topic in DISPATCHER_TOPICS):
    raise RuntimeError("V6 causal dispatcher Ground Truth firewall violated")


class CausalContractError(RuntimeError):
    """A frozen manifest or recorded-evidence contract violation."""


@dataclass(frozen=True)
class ArmContract:
    name: str
    module2_uds_enabled: bool
    integration_bridge_enabled: bool
    integration_process_required: bool
    localization_contract: str
    module3_mode: str
    obstacle_layer_mode: str
    critic_mode: str


@dataclass(frozen=True)
class RunContract:
    run_id: str
    repeat: int
    arm: str


@dataclass(frozen=True)
class CausalManifest:
    path: Path
    module3_root: Path | None
    identity: Mapping[str, Any]
    localization_contract: Mapping[str, Any]
    freshness: Mapping[str, Any]
    criteria: Mapping[str, Any]
    arms: Mapping[str, ArmContract]
    runs: tuple[RunContract, ...]


@dataclass(frozen=True)
class AdapterTemplates:
    """Three explicit process adapters required by the live campaign.

    Scene and stack are long-running process templates.  Episode is a
    foreground command which must execute exactly one V6 formal episode and
    write its JSONL trace to ``{episode_jsonl}``.
    """

    scene: str
    stack: str
    episode: str
    producer_stop: str | None = None


@dataclass(frozen=True)
class RecordedMessage:
    topic: str
    stamp_ns: int
    message: Any


def exact_adapter_templates(manifest: CausalManifest) -> AdapterTemplates:
    root = manifest.module3_root
    if root is None:
        raise CausalContractError(
            f"exact adapters require {MODULE3_ROOT_ENV} when using an installed manifest"
        )
    return AdapterTemplates(
        scene=(
            f"{root}/scripts/run_v6_r5_phase_b_kujiale.sh "
            "--domain {ros_domain_id} isaac --dynamic-obstacle-config "
            "{obstacle_config} --dynamic-obstacles"
        ),
        stack=(
            f"{root}/scripts/run_v6_low_obstacle_phase_f_stack.sh "
            "{arm} --domain {ros_domain_id} --run-dir {run_dir} "
            "--socket {module2_socket}"
        ),
        episode=(
            f"{root}/scripts/run_v6_low_obstacle_causal.sh dispatch-episode "
            "--run-id {run_id} --output-jsonl {episode_jsonl}"
        ),
        producer_stop=(
            f"{root}/scripts/run_v6_low_obstacle_phase_f_stack.sh "
            "stop-producer --run-dir {run_dir}"
        ),
    )


@dataclass(frozen=True)
class RunResult:
    run_id: str
    repeat: int
    arm: str
    verdict: str
    reasons: tuple[str, ...]
    synchronized_frames: int
    scan_invisible_rgbd_pairs: int
    typed_spatial_matches: int
    typed_spatial_total: int
    path_length_m: float
    local_trajectory_length_m: float
    near_obstacle_speed_mps: float
    minimum_clearance_m: float
    collision: bool
    success: bool
    reroute_direction: str
    critic_participation: str
    critic_ttl_status: str
    critic_post_expiry_applied: bool | None
    critic_stale_active_probe: str
    evidence_file: str


@dataclass(frozen=True)
class PairResult:
    repeat: int
    lhs_arm: str
    rhs_arm: str
    trajectory_source: str
    hausdorff_m: float
    length_delta_fraction: float
    near_obstacle_speed_delta_mps: float
    clearance_gain_m: float
    direction_consistent: bool


@dataclass(frozen=True)
class CausalSummary:
    qualification: str
    verdict: str
    reasons: tuple[str, ...]
    runs: tuple[RunResult, ...]
    m1_vs_m0: tuple[PairResult, ...]
    m2_vs_m1: tuple[PairResult, ...]
    m3_vs_m1: tuple[PairResult, ...]
    m3_vs_m2: tuple[PairResult, ...]
    visualization_inputs: tuple[Mapping[str, Any], ...]


def _mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise CausalContractError(f"{name} must be a mapping")
    return value


def _bool(value: Any, name: str) -> bool:
    if not isinstance(value, bool):
        raise CausalContractError(f"{name} must be boolean")
    return value


def _source_module3_root(manifest_path: Path) -> Path | None:
    configured = os.environ.get(MODULE3_ROOT_ENV)
    if configured:
        root = Path(configured).expanduser().resolve()
        if not (root / "ros2_ws/src/robot_experiments").is_dir():
            raise CausalContractError(f"{MODULE3_ROOT_ENV} is not a Module3 source root: {root}")
        return root
    for parent in manifest_path.parents:
        if (
            (parent / "ros2_ws/src/robot_experiments").is_dir()
            and (parent / "scripts/run_v6_low_obstacle_causal.sh").is_file()
        ):
            return parent
    return None


def _installed_package_share() -> Path | None:
    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError:
        return None
    try:
        return Path(get_package_share_directory("robot_experiments")).resolve()
    except (LookupError, OSError):
        return None


def _resolve_phase_f_resource(
    value: Any,
    *,
    manifest_path: Path,
    module3_root: Path | None,
) -> Path:
    text = str(value or "")
    if not text:
        raise CausalContractError("Phase-F resource path must be configured")
    if text.startswith(MODULE3_RESOURCE_PREFIX):
        relative = Path(text[len(MODULE3_RESOURCE_PREFIX):])
        if relative.is_absolute() or ".." in relative.parts:
            raise CausalContractError(f"invalid {MODULE3_RESOURCE_PREFIX} resource: {text}")
        candidates: list[Path] = []
        if module3_root is not None:
            candidates.append(module3_root / relative)
        share = _installed_package_share()
        if share is not None:
            candidates.append(share / "phase_f_assets" / relative)
        for candidate in candidates:
            if candidate.is_file():
                return candidate.resolve()
        raise CausalContractError(f"Phase-F resource is unavailable: {text}")
    candidate = Path(text).expanduser()
    if not candidate.is_absolute():
        candidate = manifest_path.parent / candidate
    candidate = candidate.resolve()
    if not candidate.is_file():
        raise CausalContractError(f"Phase-F resource is unavailable: {candidate}")
    return candidate


def load_manifest(path: str | Path) -> CausalManifest:
    manifest_path = Path(path).expanduser().resolve()
    module3_root = _source_module3_root(manifest_path)
    raw = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    raw = _mapping(raw, "manifest")
    if raw.get("schema_version") != SCHEMA_VERSION:
        raise CausalContractError(f"schema_version must be {SCHEMA_VERSION}")

    identity = _mapping(raw.get("identity"), "identity")
    fixed_identity = {
        "scene_id": "v6_kujiale_low_obstacles_static",
        "obstacle_layout_id": "kujiale_v6_low_obstacles_phase_f_r2_20260826",
        "scene_contract_frozen": True,
        "seed": 8601,
        "ros_domain_id": 150,
        "timeout_sec": 180.0,
        "route_backend": "primary",
        "graph_backend": "gvg",
        "direct_rgbd_costmap_enabled": False,
        "exactly_once_reset": True,
        "low_obstacles_enabled": True,
        "dynamic_actors_enabled": False,
        "module1_amcl_prior_enabled": False,
        "cognitive_place_graph_enabled": False,
    }
    for key, expected in fixed_identity.items():
        if identity.get(key) != expected:
            raise CausalContractError(f"identity.{key} must be {expected!r}")
    if _mapping(identity.get("start"), "identity.start").get("id") != "G1":
        raise CausalContractError("identity.start.id must be G1")
    if _mapping(identity.get("goal"), "identity.goal").get("id") != "G2":
        raise CausalContractError("identity.goal.id must be G2")
    expected_asset_suffixes = {
        "scene_asset": "/kujiale_0026/kujiale_0026_A_to_B_door_open.usd",
        "occupancy_map": "/data/maps/occupancy/v6_kujiale_isaacgen_v1.yaml",
        "spawn_manifest": (
            "/isaac_sim/configs/environments/"
            "kujiale_0026_A_to_B_door_open.v6_isaacgen_v1.spawn.yaml"
        ),
        "route_graph": (
            "/ros2_ws/src/robot_route_planner/config/"
            "v6_kujiale_isaacgen_v1_gvg_v1.geojson"
        ),
        "obstacle_config": (
            "/isaac_sim/configs/experiments/"
            "v6_kujiale_low_obstacles_frozen.yaml"
        ),
        "obstacle_manifest": (
            "/isaac_sim/configs/experiments/"
            "v6_kujiale_low_obstacles_frozen_manifest.yaml"
        ),
        "navigation_overlay": (
            "/ros2_ws/src/robot_navigation/config/"
            "nav2_v6_low_obstacle_isolation.yaml"
        ),
    }
    resolved_identity = dict(identity)
    for key, suffix in expected_asset_suffixes.items():
        candidate = _resolve_phase_f_resource(
            identity.get(key), manifest_path=manifest_path, module3_root=module3_root
        )
        if not str(candidate).endswith(suffix):
            raise CausalContractError(f"identity.{key} is not the frozen Phase-F asset")
        resolved_identity[key] = str(candidate)
    identity = resolved_identity

    localization = _mapping(raw.get("localization_contract"), "localization_contract")
    if localization.get("startup_profile") != "estimated_autonomy":
        raise CausalContractError("localization_contract.startup_profile must be estimated_autonomy")
    if localization.get("preserve_when_module2_disabled") is not True:
        raise CausalContractError("M0 must preserve the estimated-autonomy localization contract")

    raw_arms = _mapping(raw.get("arms"), "arms")
    arms: dict[str, ArmContract] = {}
    for name in ("M0", "M1", "M2", "M3"):
        row = _mapping(raw_arms.get(name), f"arms.{name}")
        arms[name] = ArmContract(
            name=name,
            module2_uds_enabled=_bool(row.get("module2_uds_enabled"), f"arms.{name}.module2_uds_enabled"),
            integration_bridge_enabled=_bool(row.get("integration_bridge_enabled"), f"arms.{name}.integration_bridge_enabled"),
            integration_process_required=_bool(row.get("integration_process_required"), f"arms.{name}.integration_process_required"),
            localization_contract=str(row.get("localization_contract", "")),
            module3_mode=str(row.get("module3_mode", "")),
            obstacle_layer_mode=str(row.get("obstacle_layer_mode", "")),
            critic_mode=str(row.get("critic_mode", "")),
        )
    expected_arms = {
        "M0": (False, False, "M0", "off", "off"),
        "M1": (True, True, "M1", "shadow", "shadow"),
        "M2": (True, True, "M2", "active", "off"),
        "M3": (True, True, "M3", "active", "active"),
    }
    for name, expected in expected_arms.items():
        arm = arms[name]
        actual = (
            arm.module2_uds_enabled,
            arm.integration_bridge_enabled,
            arm.module3_mode,
            arm.obstacle_layer_mode,
            arm.critic_mode,
        )
        if actual != expected:
            raise CausalContractError(f"arms.{name} does not match the frozen arm contract")
        expected_process = name != "M0"
        if arm.integration_process_required is not expected_process:
            raise CausalContractError(
                f"arms.{name}.integration_process_required must be {expected_process}"
            )
        if arm.localization_contract != "same_estimated_autonomy":
            raise CausalContractError(f"arms.{name} must keep the same Integration localization contract")

    raw_runs = raw.get("run_order")
    if not isinstance(raw_runs, list) or len(raw_runs) != 12:
        raise CausalContractError("run_order must contain exactly 12 rows")
    runs = tuple(
        RunContract(
            run_id=str(_mapping(row, f"run_order[{index}]").get("run_id", "")),
            repeat=int(_mapping(row, f"run_order[{index}]").get("repeat", 0)),
            arm=str(_mapping(row, f"run_order[{index}]").get("arm", "")),
        )
        for index, row in enumerate(raw_runs)
    )
    if tuple(run.arm for run in runs) != EXPECTED_ORDER:
        raise CausalContractError("run_order arms do not match the frozen counterbalanced order")
    if tuple(run.repeat for run in runs) != (1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3):
        raise CausalContractError("run_order repeats do not match the frozen order")
    if len({run.run_id for run in runs}) != 12 or any(not run.run_id for run in runs):
        raise CausalContractError("run_order requires 12 unique non-empty run_id values")

    capture = _mapping(raw.get("capture"), "capture")
    manifest_dispatcher = tuple(capture.get("dispatcher_topics", ()))
    manifest_passive = tuple(capture.get("passive_evaluator_topics", ()))
    if set(manifest_dispatcher) != set(DISPATCHER_TOPICS):
        raise CausalContractError("capture.dispatcher_topics must match the zero-GT recorder contract")
    if any(str(topic).startswith(GT_PREFIX) for topic in manifest_dispatcher):
        raise CausalContractError("dispatcher Ground Truth firewall violated")
    if set(manifest_passive) != set(PASSIVE_EVALUATOR_TOPICS):
        raise CausalContractError("capture.passive_evaluator_topics must match the passive GT contract")
    manifest_isolation = tuple(capture.get("isolation_audit_topics", ()))
    if set(manifest_isolation) != set(ISOLATION_AUDIT_TOPICS):
        raise CausalContractError("capture.isolation_audit_topics must match the Phase-F isolation contract")

    freshness = _mapping(raw.get("freshness"), "freshness")
    if float(freshness.get("typed_obstacle_ttl_sec", 0.0)) <= 0.0:
        raise CausalContractError("freshness.typed_obstacle_ttl_sec must be positive")
    if float(freshness.get("post_producer_stop_observation_margin_sec", 0.0)) <= 0.0:
        raise CausalContractError(
            "freshness.post_producer_stop_observation_margin_sec must be positive"
        )
    if freshness.get("stale_action") != "stop_and_fail_open":
        raise CausalContractError("freshness.stale_action must be stop_and_fail_open")
    criteria = _mapping(raw.get("criteria"), "criteria")
    return CausalManifest(
        path=manifest_path,
        module3_root=module3_root,
        identity=identity,
        localization_contract=localization,
        freshness=freshness,
        criteria=criteria,
        arms=arms,
        runs=runs,
    )


def selected_runs(manifest: CausalManifest, *, pilot: bool) -> tuple[RunContract, ...]:
    """Return one paired four-arm seed for pilot, otherwise all twelve rows."""

    if not pilot:
        return manifest.runs
    rows = tuple(run for run in manifest.runs if run.repeat == 1)
    if tuple(run.arm for run in rows) != PILOT_ARMS:
        raise CausalContractError("pilot requires one M0,M1,M2,M3 repeat")
    return rows


def _adapter_values(
    manifest: CausalManifest,
    run: RunContract,
    output_root: Path,
) -> dict[str, str]:
    run_dir = output_root / run.run_id
    arm = manifest.arms[run.arm]
    integration_profile = {
        "M0": "off",
        "M1": "estimated_shadow",
        "M2": "module2_causal_obstacle_active",
        "M3": "module2_causal_obstacle_active",
    }[run.arm]
    effect_scope = {
        "M0": "none",
        "M1": "shadow",
        "M2": "obstacle_only",
        "M3": "obstacle_only",
    }[run.arm]
    if manifest.module3_root is None:
        module3_root = os.environ.get(MODULE3_ROOT_ENV, "")
    else:
        module3_root = str(manifest.module3_root)
    return {
        "run_id": run.run_id,
        "repeat": str(run.repeat),
        "arm": run.arm,
        "seed": str(manifest.identity["seed"]),
        "ros_domain_id": str(manifest.identity["ros_domain_id"]),
        "run_dir": str(run_dir),
        "bag_dir": str(run_dir / "bag"),
        "episode_jsonl": str(run_dir / "episode.jsonl"),
        "episode_result": str(run_dir / "episode_result.json"),
        "evidence_json": str(run_dir / f"{run.run_id}.json"),
        "causal_config": str(manifest.path),
        "module3_root": module3_root,
        "scene_asset": str(manifest.identity["scene_asset"]),
        "occupancy_map": str(manifest.identity["occupancy_map"]),
        "spawn_manifest": str(manifest.identity["spawn_manifest"]),
        "route_graph": str(manifest.identity["route_graph"]),
        "obstacle_config": str(manifest.identity["obstacle_config"]),
        "obstacle_manifest": str(manifest.identity["obstacle_manifest"]),
        "navigation_overlay": str(manifest.identity["navigation_overlay"]),
        "cognitive_profile": run.arm,
        "module2_enabled": "true" if arm.module2_uds_enabled else "false",
        "module2_mode": "off" if run.arm == "M0" else (
            "shadow" if run.arm == "M1" else "active"
        ),
        "obstacle_layer_mode": arm.obstacle_layer_mode,
        "critic_mode": arm.critic_mode,
        "integration_startup_profile": integration_profile,
        "active_effect_scope": effect_scope,
        "module2_socket": f"/tmp/bnv6f-r{run.repeat}{run.arm.lower()}.sock",
        "module1_amcl_prior_enabled": "false",
        "cognitive_graph_mode": "gvg",
        "dynamic_actors_enabled": "false",
    }


def render_adapter_command(template: str, values: Mapping[str, str]) -> tuple[str, ...]:
    """Expand a reviewable argv template without invoking a shell."""

    try:
        rendered = template.format_map(values)
    except KeyError as exc:
        raise CausalContractError(f"unknown adapter placeholder: {exc.args[0]}") from exc
    command = tuple(shlex.split(rendered))
    if not command:
        raise CausalContractError("adapter command must not be empty")
    return command


def _field(value: Any, path: str, default: Any = None) -> Any:
    current = value
    for name in path.split("."):
        if isinstance(current, Mapping):
            if name not in current:
                return default
            current = current[name]
        else:
            if not hasattr(current, name):
                return default
            current = getattr(current, name)
    return current


def _time_ns(value: Any) -> int | None:
    if value is None:
        return None
    if isinstance(value, bool):
        return None
    if isinstance(value, int):
        return value
    sec = _field(value, "sec")
    nanosec = _field(value, "nanosec")
    if isinstance(sec, int) and isinstance(nanosec, int):
        return sec * 1_000_000_000 + nanosec
    return None


def _message_stamp_ns(record: RecordedMessage) -> int:
    for path in ("header.stamp", "stamp", "validation_stamp"):
        stamp = _time_ns(_field(record.message, path))
        if stamp is not None and stamp > 0:
            return stamp
    return int(record.stamp_ns)


def _xy_from_pose(value: Any) -> tuple[float, float] | None:
    candidates = (
        _field(value, "pose.pose.position"),
        _field(value, "pose.position"),
        _field(value, "position"),
        value,
    )
    for candidate in candidates:
        x = _field(candidate, "x")
        y = _field(candidate, "y")
        if x is None and isinstance(candidate, Sequence) and not isinstance(candidate, (str, bytes)):
            if len(candidate) >= 2:
                x, y = candidate[0], candidate[1]
        try:
            return float(x), float(y)
        except (TypeError, ValueError):
            continue
    return None


def _path_points(message: Any) -> list[list[float]]:
    raw = _field(message, "poses")
    if raw is None and isinstance(message, Mapping):
        raw = message.get("points", message.get("path"))
    if not isinstance(raw, Sequence) or isinstance(raw, (str, bytes)):
        return []
    points: list[list[float]] = []
    for value in raw:
        point = _xy_from_pose(value)
        if point is not None and all(math.isfinite(item) for item in point):
            points.append([point[0], point[1]])
    return points


def _odom_point(message: Any) -> list[float] | None:
    point = _xy_from_pose(message)
    if point is None or not all(math.isfinite(item) for item in point):
        return None
    return [point[0], point[1]]


def _quaternion_yaw(message: Any) -> float:
    orientation = _field(message, "pose.pose.orientation", _field(message, "pose.orientation"))
    try:
        x = float(_field(orientation, "x", 0.0))
        y = float(_field(orientation, "y", 0.0))
        z = float(_field(orientation, "z", 0.0))
        w = float(_field(orientation, "w", 1.0))
    except (TypeError, ValueError):
        return 0.0
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _nearest(records: Sequence[RecordedMessage], stamp_ns: int) -> RecordedMessage | None:
    if not records:
        return None
    return min(records, key=lambda row: abs(_message_stamp_ns(row) - stamp_ns))


def _load_frozen_obstacle(manifest: CausalManifest) -> dict[str, Any]:
    document = _mapping(
        yaml.safe_load(Path(manifest.identity["obstacle_manifest"]).read_text(encoding="utf-8")),
        "obstacle_manifest",
    )
    obstacles = document.get("obstacles")
    if not isinstance(obstacles, list) or len(obstacles) != 1:
        raise CausalContractError("Phase-F obstacle manifest must contain one obstacle")
    row = _mapping(obstacles[0], "obstacle_manifest.obstacles[0]")
    position = row.get("map_position")
    size = row.get("size")
    if not isinstance(position, list) or len(position) < 2 or not isinstance(size, list) or len(size) < 2:
        raise CausalContractError("frozen obstacle requires map_position and size")
    geometry = _mapping(document.get("geometry_contract"), "obstacle_manifest.geometry_contract")
    return {
        "id": str(row.get("id", "")),
        "center": [float(position[0]), float(position[1])],
        "size": [float(size[0]), float(size[1]), float(size[2])],
        "z_bounds": [
            float(geometry["obstacle_bottom_z_m"]),
            float(geometry["obstacle_top_z_m"]),
        ],
        "robot_radius_m": 0.5 * float(geometry["robot_max_footprint_dimension_m"]),
    }


def _typed_obstacles(message: Any) -> list[dict[str, Any]]:
    accepted = bool(
        _field(message, "input_healthy", True)
        and _field(message, "module2_healthy", False)
        and _field(message, "observation_valid", False)
    )
    trusted_write = bool(_field(message, "trusted_write", False))
    result: list[dict[str, Any]] = []
    for obstacle in _field(message, "obstacles", ()) or ():
        pose = _field(obstacle, "pose_xy_m")
        if not isinstance(pose, Sequence) or len(pose) < 2:
            continue
        result.append({
            "id": str(_field(obstacle, "id", "")),
            "x": float(pose[0]),
            "y": float(pose[1]),
            "radius_m": float(_field(obstacle, "radius_m", 0.0)),
            "confidence": float(_field(obstacle, "confidence", 0.0)),
            "accepted": accepted,
            "trusted_write": trusted_write,
        })
    return result


def _scan_metrics(
    scan: RecordedMessage | None,
    pose: RecordedMessage | None,
    obstacle: Mapping[str, Any],
) -> tuple[int, int]:
    if scan is None:
        return 0, 0
    explicit_points = _field(scan.message, "scan_point_count")
    explicit_hits = _field(scan.message, "scan_hits_in_obstacle_footprints")
    if isinstance(explicit_points, int) and isinstance(explicit_hits, int):
        return explicit_points, explicit_hits
    ranges = _field(scan.message, "ranges", ()) or ()
    try:
        angle = float(_field(scan.message, "angle_min"))
        increment = float(_field(scan.message, "angle_increment"))
    except (TypeError, ValueError):
        return 0, 0
    valid = [(index, float(distance)) for index, distance in enumerate(ranges) if math.isfinite(float(distance))]
    if pose is None:
        return len(valid), 0
    origin = _odom_point(pose.message)
    if origin is None:
        return len(valid), 0
    yaw = _quaternion_yaw(pose.message)
    center = obstacle["center"]
    half_x = 0.5 * float(obstacle["size"][0])
    half_y = 0.5 * float(obstacle["size"][1])
    hits = 0
    for index, distance in valid:
        heading = yaw + angle + index * increment
        x = origin[0] + distance * math.cos(heading)
        y = origin[1] + distance * math.sin(heading)
        if abs(x - center[0]) <= half_x and abs(y - center[1]) <= half_y:
            hits += 1
    return len(valid), hits


RigidTransform = tuple[
    tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
    tuple[float, float, float],
]


def _identity_transform() -> RigidTransform:
    return (
        ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
        (0.0, 0.0, 0.0),
    )


def _rotate(rotation: Sequence[Sequence[float]], point: Sequence[float]) -> tuple[float, float, float]:
    return tuple(
        sum(float(rotation[row][column]) * float(point[column]) for column in range(3))
        for row in range(3)
    )  # type: ignore[return-value]


def _compose_transform(outer: RigidTransform, inner: RigidTransform) -> RigidTransform:
    outer_rotation, outer_translation = outer
    inner_rotation, inner_translation = inner
    rotation = tuple(
        tuple(
            sum(outer_rotation[row][index] * inner_rotation[index][column] for index in range(3))
            for column in range(3)
        )
        for row in range(3)
    )
    rotated_translation = _rotate(outer_rotation, inner_translation)
    translation = tuple(rotated_translation[index] + outer_translation[index] for index in range(3))
    return rotation, translation  # type: ignore[return-value]


def _inverse_transform(value: RigidTransform) -> RigidTransform:
    rotation, translation = value
    inverse_rotation = tuple(tuple(rotation[column][row] for column in range(3)) for row in range(3))
    rotated = _rotate(inverse_rotation, translation)
    return inverse_rotation, tuple(-item for item in rotated)  # type: ignore[return-value]


def _apply_transform(value: RigidTransform, point: Sequence[float]) -> tuple[float, float, float]:
    rotation, translation = value
    rotated = _rotate(rotation, point)
    return tuple(rotated[index] + translation[index] for index in range(3))  # type: ignore[return-value]


def _transform_from_message(value: Any) -> RigidTransform | None:
    transform = _field(value, "transform", value)
    translation = _field(transform, "translation")
    rotation = _field(transform, "rotation")
    try:
        tx = float(_field(translation, "x"))
        ty = float(_field(translation, "y"))
        tz = float(_field(translation, "z"))
        qx = float(_field(rotation, "x"))
        qy = float(_field(rotation, "y"))
        qz = float(_field(rotation, "z"))
        qw = float(_field(rotation, "w"))
    except (TypeError, ValueError):
        return None
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not math.isfinite(norm) or norm <= 1.0e-12:
        return None
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    matrix = (
        (1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)),
        (2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)),
        (2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)),
    )
    if not all(math.isfinite(item) for row in matrix for item in row) or not all(
        math.isfinite(item) for item in (tx, ty, tz)
    ):
        return None
    return matrix, (tx, ty, tz)


def _frame(value: Any) -> str:
    return str(value or "").strip().lstrip("/")


def _lookup_recorded_transform(
    tf_records: Sequence[RecordedMessage],
    tf_static_records: Sequence[RecordedMessage],
    *,
    target_frame: str,
    source_frame: str,
    stamp_ns: int,
) -> RigidTransform | None:
    """Resolve target_T_source from bagged TF without using passive GT."""

    selected: dict[tuple[str, str], tuple[int, RigidTransform]] = {}
    for records, is_static in ((tf_static_records, True), (tf_records, False)):
        for record in records:
            for raw in _field(record.message, "transforms", ()) or ():
                parent = _frame(_field(raw, "header.frame_id"))
                child = _frame(_field(raw, "child_frame_id"))
                transform = _transform_from_message(raw)
                if not parent or not child or transform is None:
                    continue
                transform_stamp = _time_ns(_field(raw, "header.stamp"))
                if transform_stamp is None or transform_stamp <= 0:
                    transform_stamp = _message_stamp_ns(record)
                if is_static:
                    age = 0
                else:
                    age = stamp_ns - transform_stamp
                    if age < 0 or age > DEFAULT_SYNC_TOLERANCE_NS:
                        continue
                key = (parent, child)
                if key not in selected or age < selected[key][0]:
                    selected[key] = (age, transform)

    graph: dict[str, list[tuple[str, RigidTransform]]] = {}
    for (parent, child), (_, parent_from_child) in selected.items():
        graph.setdefault(child, []).append((parent, parent_from_child))
        graph.setdefault(parent, []).append((child, _inverse_transform(parent_from_child)))
    source = _frame(source_frame)
    target = _frame(target_frame)
    if not source or not target:
        return None
    queue: list[tuple[str, RigidTransform]] = [(source, _identity_transform())]
    visited = {source}
    while queue:
        current, current_from_source = queue.pop(0)
        if current == target:
            return current_from_source
        for adjacent, adjacent_from_current in graph.get(current, ()):
            if adjacent in visited:
                continue
            visited.add(adjacent)
            queue.append((adjacent, _compose_transform(adjacent_from_current, current_from_source)))
    return None


def _decode_depth_pixels(
    message: Any,
    *,
    stride: int,
    maximum_depth_m: float,
) -> tuple[list[tuple[int, int, float]], str | None]:
    try:
        width = int(_field(message, "width"))
        height = int(_field(message, "height"))
        step = int(_field(message, "step"))
        data = bytes(_field(message, "data"))
    except (TypeError, ValueError):
        return [], "invalid_depth_layout"
    encoding = str(_field(message, "encoding", "")).lower()
    if encoding in {"32fc1", "32fc"}:
        format_code, bytes_per_pixel, scale = "f", 4, 1.0
    elif encoding in {"16uc1", "mono16"}:
        format_code, bytes_per_pixel, scale = "H", 2, 0.001
    else:
        return [], "unsupported_depth_encoding"
    if (
        width <= 0 or height <= 0 or step < width * bytes_per_pixel
        or len(data) < step * height or stride <= 0
    ):
        return [], "invalid_depth_layout"
    prefix = ">" if bool(_field(message, "is_bigendian", False)) else "<"
    pixels: list[tuple[int, int, float]] = []
    try:
        for v in range(0, height, stride):
            for u in range(0, width, stride):
                raw = struct.unpack_from(prefix + format_code, data, v * step + u * bytes_per_pixel)[0]
                depth = float(raw) * scale
                if math.isfinite(depth) and 0.0 < depth <= maximum_depth_m:
                    pixels.append((u, v, depth))
    except struct.error:
        return [], "invalid_depth_layout"
    return pixels, None


def _project_depth_obstacle(
    depth: RecordedMessage | None,
    camera_info: RecordedMessage | None,
    tf_records: Sequence[RecordedMessage],
    tf_static_records: Sequence[RecordedMessage],
    obstacle: Mapping[str, Any],
    criteria: Mapping[str, Any],
) -> dict[str, Any]:
    if depth is None:
        return {"valid": False, "reason": "depth_missing", "point_count": 0, "hit_count": 0, "footprints": []}
    if camera_info is None:
        return {"valid": False, "reason": "camera_info_missing", "point_count": 0, "hit_count": 0, "footprints": []}
    depth_stamp = _message_stamp_ns(depth)
    if abs(_message_stamp_ns(camera_info) - depth_stamp) > DEFAULT_SYNC_TOLERANCE_NS:
        return {"valid": False, "reason": "camera_info_unsynchronized", "point_count": 0, "hit_count": 0, "footprints": []}
    depth_frame = _frame(_field(depth.message, "header.frame_id"))
    info_frame = _frame(_field(camera_info.message, "header.frame_id"))
    if not depth_frame or not info_frame or depth_frame != info_frame:
        return {"valid": False, "reason": "camera_frame_mismatch", "point_count": 0, "hit_count": 0, "footprints": []}
    intrinsic = _field(camera_info.message, "k")
    if intrinsic is None:
        return {"valid": False, "reason": "invalid_camera_intrinsics", "point_count": 0, "hit_count": 0, "footprints": []}
    try:
        if len(intrinsic) < 9:
            raise ValueError
        fx, cx, fy, cy = float(intrinsic[0]), float(intrinsic[2]), float(intrinsic[4]), float(intrinsic[5])
    except (TypeError, ValueError):
        return {"valid": False, "reason": "invalid_camera_intrinsics", "point_count": 0, "hit_count": 0, "footprints": []}
    if not all(math.isfinite(value) for value in (fx, fy, cx, cy)) or fx <= 0.0 or fy <= 0.0:
        return {"valid": False, "reason": "invalid_camera_intrinsics", "point_count": 0, "hit_count": 0, "footprints": []}
    pixels, decode_error = _decode_depth_pixels(
        depth.message,
        stride=max(1, int(criteria.get("depth_subsample_stride", 4))),
        maximum_depth_m=float(criteria.get("depth_max_range_m", 8.0)),
    )
    if decode_error is not None:
        return {"valid": False, "reason": decode_error, "point_count": 0, "hit_count": 0, "footprints": []}
    map_from_camera = _lookup_recorded_transform(
        tf_records,
        tf_static_records,
        target_frame="map",
        source_frame=depth_frame,
        stamp_ns=depth_stamp,
    )
    if map_from_camera is None:
        return {"valid": False, "reason": "map_camera_tf_missing", "point_count": len(pixels), "hit_count": 0, "footprints": []}
    xy_tolerance = float(criteria.get("depth_obstacle_bounds_tolerance_m", 0.10))
    z_tolerance = float(criteria.get("depth_low_z_tolerance_m", 0.05))
    center = obstacle["center"]
    half_x = 0.5 * float(obstacle["size"][0]) + xy_tolerance
    half_y = 0.5 * float(obstacle["size"][1]) + xy_tolerance
    lower_z = float(obstacle["z_bounds"][0]) - z_tolerance
    upper_z = float(obstacle["z_bounds"][1]) + z_tolerance
    hits: list[tuple[float, float, float]] = []
    for u, v, depth_m in pixels:
        camera_point = ((u - cx) * depth_m / fx, (v - cy) * depth_m / fy, depth_m)
        point = _apply_transform(map_from_camera, camera_point)
        if (
            abs(point[0] - float(center[0])) <= half_x
            and abs(point[1] - float(center[1])) <= half_y
            and lower_z <= point[2] <= upper_z
        ):
            hits.append(point)
    minimum_points = max(1, int(criteria.get("depth_min_obstacle_points", 1)))
    if len(hits) < minimum_points:
        return {
            "valid": True,
            "reason": "no_low_points_in_obstacle_bounds",
            "point_count": len(pixels),
            "hit_count": len(hits),
            "footprints": [],
        }
    mins = [min(point[index] for point in hits) for index in range(3)]
    maxs = [max(point[index] for point in hits) for index in range(3)]
    footprint = {
        "id": obstacle["id"],
        "center": [(mins[index] + maxs[index]) * 0.5 for index in range(3)],
        "size": [maxs[index] - mins[index] for index in range(3)],
        "source": "projected_depth_points",
        "point_count": len(hits),
    }
    return {
        "valid": True,
        "reason": "observed",
        "point_count": len(pixels),
        "hit_count": len(hits),
        "footprints": [footprint],
    }


def _minimum_clearance(points: Sequence[Sequence[float]], obstacle: Mapping[str, Any]) -> float:
    if not points:
        raise CausalContractError("passive Ground Truth path is missing")
    center = obstacle["center"]
    half_x = 0.5 * float(obstacle["size"][0])
    half_y = 0.5 * float(obstacle["size"][1])
    radius = float(obstacle["robot_radius_m"])
    clearances = []
    for point in points:
        dx = max(abs(float(point[0]) - center[0]) - half_x, 0.0)
        dy = max(abs(float(point[1]) - center[1]) - half_y, 0.0)
        clearances.append(max(0.0, math.hypot(dx, dy) - radius))
    return min(clearances)


def _near_obstacle_speed(
    command_records: Sequence[RecordedMessage],
    gt_records: Sequence[RecordedMessage],
    obstacle: Mapping[str, Any],
) -> float:
    speeds: list[float] = []
    center = obstacle["center"]
    for command in command_records:
        pose = _nearest(gt_records, _message_stamp_ns(command))
        point = _odom_point(pose.message) if pose is not None else None
        if point is None or math.dist(point, center) > 1.5:
            continue
        speeds.append(abs(float(_field(command.message, "linear.x", 0.0))))
    return statistics.fmean(speeds) if speeds else 0.0


def _near_obstacle_trajectory(
    trajectory_records: Sequence[RecordedMessage],
    gt_records: Sequence[RecordedMessage],
    obstacle: Mapping[str, Any],
) -> list[list[float]]:
    candidates: list[tuple[float, list[list[float]]]] = []
    for record in trajectory_records:
        path = _path_points(record.message)
        if not path:
            continue
        pose = _nearest(gt_records, _message_stamp_ns(record))
        point = _odom_point(pose.message) if pose is not None else None
        distance = math.dist(point, obstacle["center"]) if point is not None else math.inf
        candidates.append((distance, path))
    if not candidates:
        return []
    return min(candidates, key=lambda item: item[0])[1]


def _status_summary(records: Sequence[RecordedMessage]) -> dict[str, Any]:
    return {
        "status_count": len(records),
        "cells": max((int(_field(row.message, "raised_cell_count", 0)) for row in records), default=0),
        "active_cells": max((int(_field(row.message, "active_cell_count", 0)) for row in records), default=0),
        "max_cost": max((int(_field(row.message, "maximum_cost", 0)) for row in records), default=0),
        "max_cost_increase": max((int(_field(row.message, "maximum_cost_increase", 0)) for row in records), default=0),
        "applied_count": sum(bool(_field(row.message, "applied", False)) for row in records),
        "rejected_count": sum(bool(_field(row.message, "rejected", False)) for row in records),
    }


def build_recorded_evidence(
    manifest: CausalManifest,
    run: RunContract,
    records: Iterable[RecordedMessage],
    episode_result: Mapping[str, Any],
) -> dict[str, Any]:
    """Reduce real recorded ROS messages into the existing causal evaluator JSON."""

    by_topic: dict[str, list[RecordedMessage]] = {}
    for record in records:
        by_topic.setdefault(record.topic, []).append(record)
    for values in by_topic.values():
        values.sort(key=_message_stamp_ns)

    obstacle = _load_frozen_obstacle(manifest)
    typed_records = by_topic.get("/bio_nav/module2/cognitive_obstacles", [])
    scan_records = by_topic.get("/scan", [])
    depth_records = by_topic.get("/camera/front/depth/image_raw", [])
    camera_info_records = by_topic.get("/camera/front/camera_info", [])
    tf_records = by_topic.get("/tf", [])
    tf_static_records = by_topic.get("/tf_static", [])
    gt_records = by_topic.get("/ground_truth/odom", [])
    if typed_records:
        anchors = typed_records
    else:
        anchors = []
        last_stamp = -10**30
        for record in depth_records:
            stamp = _message_stamp_ns(record)
            if stamp - last_stamp >= 500_000_000:
                anchors.append(record)
                last_stamp = stamp
    samples: list[dict[str, Any]] = []
    for anchor in anchors:
        stamp = _message_stamp_ns(anchor)
        scan = _nearest(scan_records, stamp)
        depth = _nearest(depth_records, stamp)
        camera_info = _nearest(camera_info_records, _message_stamp_ns(depth)) if depth is not None else None
        typed = _nearest(typed_records, stamp)
        pose = _nearest(gt_records, stamp)
        depth_synchronized = depth is not None and abs(_message_stamp_ns(depth) - stamp) <= DEFAULT_SYNC_TOLERANCE_NS
        scan_synchronized = scan is not None and abs(_message_stamp_ns(scan) - stamp) <= DEFAULT_SYNC_TOLERANCE_NS
        if not depth_synchronized:
            depth = None
            camera_info = None
        if not scan_synchronized:
            scan = None
        scan_points, scan_hits = _scan_metrics(scan, pose, obstacle)
        depth_observation = _project_depth_obstacle(
            depth,
            camera_info,
            tf_records,
            tf_static_records,
            obstacle,
            manifest.criteria,
        )
        typed_values = []
        if typed is not None and abs(_message_stamp_ns(typed) - stamp) <= DEFAULT_SYNC_TOLERANCE_NS:
            typed_values = _typed_obstacles(typed.message)
        observed_centers = [
            _footprint_center(value) for value in depth_observation["footprints"]
        ]
        for value in typed_values:
            value["observed_spatial_error_m"] = (
                min(
                    math.dist((float(value["x"]), float(value["y"])), center)
                    for center in observed_centers
                )
                if value["accepted"] and observed_centers else None
            )
        samples.append({
            "stamp_ns": stamp,
            "frame_id": "map",
            "scan_valid": scan_synchronized,
            "scan_point_count": scan_points,
            "scan_hits_in_obstacle_footprints": scan_hits,
            "depth_observation_valid": depth_observation["valid"],
            "depth_observation_reason": depth_observation["reason"],
            "depth_point_count": depth_observation["point_count"],
            "depth_hits_in_obstacle_bounds": depth_observation["hit_count"],
            "rgbd_obstacle_footprints": depth_observation["footprints"],
            "typed_obstacles": typed_values,
            "depth_stamp_ns": _message_stamp_ns(depth) if depth is not None else None,
            "camera_info_stamp_ns": _message_stamp_ns(camera_info) if camera_info is not None else None,
            "scan_stamp_ns": _message_stamp_ns(scan) if scan is not None else None,
        })

    layer_records = by_topic.get("/bio_nav/cognitive_obstacle_layer/status", [])
    global_status = [row for row in layer_records if "global" in str(_field(row.message, "consumer", "")).lower()]
    local_status = [row for row in layer_records if "local" in str(_field(row.message, "consumer", "")).lower()]
    critic_records = by_topic.get("/bio_nav/cognitive_risk_critic/status", [])
    critic_reasons = [str(_field(row.message, "fallback_reason", "")) for row in critic_records]
    critic_applied = [row for row in critic_records if bool(_field(row.message, "applied", False))]
    message_ages = [
        max(0.0, float(_field(row.message, "source_age.sec", 0.0)) + float(_field(row.message, "source_age.nanosec", 0.0)) * 1e-9)
        for row in typed_records
    ]
    status_ages = [
        max(0.0, float(_field(row.message, "message_age_ms", 0.0)) * 1e-3)
        for row in layer_records + critic_records
        if math.isfinite(float(_field(row.message, "message_age_ms", 0.0)))
    ]
    max_age = max(message_ages + status_ages, default=0.0)
    ttl = float(manifest.freshness["typed_obstacle_ttl_sec"])
    typed_stamps = [_message_stamp_ns(row) for row in typed_records]
    cadence_hz = 0.0
    if len(typed_stamps) >= 2 and typed_stamps[-1] > typed_stamps[0]:
        cadence_hz = (len(typed_stamps) - 1) * 1.0e9 / (typed_stamps[-1] - typed_stamps[0])
    active_ttl = run.arm in {"M2", "M3"}
    latest_typed: RecordedMessage | None = None
    latest_validation_ns: int | None = None
    latest_validation_ttl_ns: int | None = None
    latest_sequence: int | None = None
    for row in typed_records:
        validation_ns = _time_ns(_field(row.message, "validation_stamp"))
        validation_ttl_ns = _time_ns(_field(row.message, "validation_ttl"))
        if validation_ns is None or validation_ttl_ns is None or validation_ttl_ns <= 0:
            continue
        if latest_validation_ns is None or validation_ns > latest_validation_ns:
            latest_typed = row
            latest_validation_ns = validation_ns
            latest_validation_ttl_ns = validation_ttl_ns
            latest_sequence = int(_field(row.message, "sequence", 0))
    expiry_ns = None
    if latest_typed is not None and latest_validation_ns is not None and latest_validation_ttl_ns is not None:
        expiry_ns = latest_validation_ns + min(latest_validation_ttl_ns, int(ttl * 1.0e9))

    def post_expiry(records: Sequence[RecordedMessage]) -> list[RecordedMessage]:
        return [
            row for row in records
            if expiry_ns is not None
            and latest_sequence is not None
            and _message_stamp_ns(row) >= expiry_ns
            and int(_field(row.message, "source_sequence", -1)) == latest_sequence
        ]

    expired_layers = post_expiry(layer_records)
    expired_critics = post_expiry(critic_records)
    critic_post_expiry_applied = any(
        bool(_field(row.message, "applied", False))
        or "cost_delta_applied=true" in str(_field(row.message, "fallback_reason", ""))
        for row in expired_critics
    ) if run.arm == "M3" else None
    stale_applied_count = sum(
        bool(_field(row.message, "applied", False)) for row in expired_layers
    ) + (
        sum(
            bool(_field(row.message, "applied", False))
            or "cost_delta_applied=true" in str(_field(row.message, "fallback_reason", ""))
            for row in expired_critics
        ) if run.arm == "M3" else 0
    )

    def layer_scope(row: RecordedMessage) -> str:
        consumer = str(_field(row.message, "consumer", "")).lower()
        if "global" in consumer:
            return "global"
        if "local" in consumer:
            return "local"
        return "unknown"

    clear_layers = [
        row for row in expired_layers
        if "rejection_reason=validation_stale" in str(_field(row.message, "fallback_reason", ""))
        and not bool(_field(row.message, "applied", False))
        and int(_field(row.message, "raised_cell_count", -1)) == 0
        and int(_field(row.message, "active_cell_count", -1)) == 0
        and int(_field(row.message, "maximum_cost_increase", -1)) == 0
    ]
    clear_consumers = {layer_scope(row) for row in clear_layers}
    ttl_expiry_zero_write = {"global", "local"}.issubset(clear_consumers) if active_ttl else None
    clear_critics = [
        row for row in expired_critics
        if "obstacle_rejected=validation_stale" in str(_field(row.message, "fallback_reason", ""))
        and "cost_delta_applied=true" not in str(_field(row.message, "fallback_reason", ""))
        and not bool(_field(row.message, "applied", False))
    ]
    if run.arm != "M3":
        critic_ttl_status = None
        ttl_expiry_critic_not_applied = None
    elif not expired_critics:
        critic_ttl_status = "N/A_NO_CONTROLLER_SCORING"
        ttl_expiry_critic_not_applied = None
    elif len(clear_critics) == len(expired_critics):
        critic_ttl_status = "STALE_REJECTED"
        ttl_expiry_critic_not_applied = True
    elif critic_post_expiry_applied:
        critic_ttl_status = "FAIL_POST_EXPIRY_APPLIED"
        ttl_expiry_critic_not_applied = False
    else:
        critic_ttl_status = "FAIL_NOT_STALE_REJECTED"
        ttl_expiry_critic_not_applied = False
    # The nominal producer-stop drain proves the two costmap consumers clear.
    # A controller which already terminated need not emit another critic score;
    # absence of that callback is explicit N/A, not stale-rejection evidence.
    ttl_expiry_observed = bool(ttl_expiry_zero_write) if active_ttl else None

    plans = [_path_points(row.message) for row in by_topic.get("/plan", [])]
    local_trajectory_records = by_topic.get("/optimal_trajectory", [])
    local_trajectories = [_path_points(row.message) for row in local_trajectory_records]
    plan = max((item for item in plans if item), key=len, default=[])
    optimal = _near_obstacle_trajectory(local_trajectory_records, gt_records, obstacle)
    odom = [point for row in by_topic.get("/odom", []) if (point := _odom_point(row.message)) is not None]
    gt = [point for row in gt_records if (point := _odom_point(row.message)) is not None]
    commands = []
    command_records = by_topic.get("/cmd_vel", [])
    for row in command_records:
        commands.append({
            "stamp_ns": _message_stamp_ns(row),
            "linear_x": float(_field(row.message, "linear.x", 0.0)),
            "angular_z": float(_field(row.message, "angular.z", 0.0)),
        })

    obstacle_messages = [_typed_obstacles(row.message) for row in typed_records]
    obstacle_validation = [item for values in obstacle_messages for item in values if item["accepted"]]
    module2_health = {
        "message_count": len(typed_records),
        "healthy_count": sum(bool(_field(row.message, "module2_healthy", False)) for row in typed_records),
        "trusted_write_count": sum(bool(_field(row.message, "trusted_write", False)) for row in typed_records),
        "observation_valid_count": sum(bool(_field(row.message, "observation_valid", False)) for row in typed_records),
        "candidate_cadence_hz": cadence_hz,
        "scope": "low_obstacle_only",
    }
    isolation_counts = {topic: len(by_topic.get(topic, [])) for topic in ISOLATION_AUDIT_TOPICS}
    collision = any(bool(_field(row.message, "data", False)) for row in by_topic.get("/simulation/collision", []))
    terminal_zero = bool(episode_result.get("terminal_zero_confirmed", False))
    success = episode_result.get("state") == "SUCCEEDED" and terminal_zero and not collision
    arm = manifest.arms[run.arm]
    return {
        "run_id": run.run_id,
        "repeat": run.repeat,
        "arm": run.arm,
        "identity": dict(manifest.identity),
        "module2_uds_connected": bool(typed_records) if run.arm != "M0" else False,
        "module2_health": module2_health,
        "isolation": {
            "module1_amcl_prior_enabled": False,
            "cognitive_place_graph_enabled": False,
            "dynamic_actors_enabled": False,
            "unexpected_topic_counts": isolation_counts,
        },
        "reset": {
            "calls": int(episode_result.get("reset_calls", 0)),
            "events": int(episode_result.get("reset_events", 0)),
            "goal_publications": int(episode_result.get("goal_publications", 0)),
            "localization_contract": "same_estimated_autonomy",
        },
        "action": {
            "state": str(episode_result.get("state", "UNKNOWN")),
            "stop_reason": str(episode_result.get("stop_reason", "")),
            "completed_leg_ids": list(episode_result.get("completed_leg_ids", [])),
            "terminal_zero_confirmed": terminal_zero,
        },
        "route": {
            "goal_results": list(episode_result.get("route_goal_results", [])),
            "progress_messages": int(episode_result.get("route_progress_messages", 0)),
            "completion_messages": int(episode_result.get("route_completion_messages", 0)),
        },
        "freshness": {
            "ttl_clear_applicability": (
                "required_active_write" if active_ttl else "not_applicable_inactive"
            ),
            "ttl_source_sequence": latest_sequence if active_ttl else None,
            "ttl_expiry_stamp_ns": expiry_ns if active_ttl else None,
            "max_typed_obstacle_age_sec": max_age,
            "stale_applied_count": stale_applied_count,
            "stopped_before_dispatch": (
                bool(ttl_expiry_observed) and stale_applied_count == 0
            ) if active_ttl else None,
            "layer_zero_write": ttl_expiry_zero_write,
            "critic_not_applied": ttl_expiry_critic_not_applied,
            "ttl_expiry_observed": ttl_expiry_observed,
            "ttl_expiry_zero_write": ttl_expiry_zero_write,
            "ttl_expiry_critic_not_applied": ttl_expiry_critic_not_applied,
            "critic_ttl_status": critic_ttl_status,
            "critic_post_expiry_applied": critic_post_expiry_applied,
            "critic_stale_active_probe": "NOT_RUN" if run.arm == "M3" else None,
        },
        "synchronized_samples": samples,
        "obstacle_validation": obstacle_validation,
        "layer": {
            "mode": arm.obstacle_layer_mode,
            "global": _status_summary(global_status),
            "local": _status_summary(local_status),
        },
        "critic": {
            "mode": arm.critic_mode,
            "applied": bool(critic_applied),
            "reason": ";".join(reason for reason in critic_reasons if reason) or "no_status_reason",
            "status_count": len(critic_records),
            "applied_count": len(critic_applied),
            "cost_delta_nonzero_count": sum("cost_delta_applied=true" in reason for reason in critic_reasons),
            "near_obstacle_speed_mps": _near_obstacle_speed(command_records, gt_records, obstacle),
            "offline_reconstructed_scores": [],
        },
        "planning_prior": [
            {"stamp_ns": _message_stamp_ns(row), "healthy": bool(_field(row.message, "healthy", False))}
            for topic in ("/bio_nav/module2/planning_prior", "/bio_nav/module2/goal_planning_prior")
            for row in by_topic.get(topic, [])
        ],
        "costmaps": {
            "global": {"recorded": bool(by_topic.get("/global_costmap/costmap"))},
            "local": {"recorded": bool(by_topic.get("/local_costmap/costmap"))},
        },
        "plan": plan,
        "optimal_trajectory": optimal,
        "odom": odom,
        "cmd_vel": commands,
        "passive": {
            "ground_truth_odom": gt,
            "minimum_clearance_m": _minimum_clearance(gt, obstacle),
            "collision": collision,
            "success": success,
        },
        "navigation_metrics": {
            "recorded_duration_sec": (
                (max((_message_stamp_ns(row) for rows in by_topic.values() for row in rows), default=0)
                 - min((_message_stamp_ns(row) for rows in by_topic.values() for row in rows), default=0))
                * 1.0e-9
            ),
            "global_plan_updates": len(plans),
            "local_trajectory_updates": len(local_trajectories),
            "nonzero_command_count": sum(abs(item["linear_x"]) > 1.0e-6 or abs(item["angular_z"]) > 1.0e-6 for item in commands),
            "near_obstacle_speed_mps": _near_obstacle_speed(command_records, gt_records, obstacle),
            "dynamic_risk_exposure": "not_applicable_dynamic_actors_off",
            "false_positive_deadlock": bool(
                episode_result.get("state") != "SUCCEEDED" and not obstacle_validation
            ),
            "stale_residual_applied_count": stale_applied_count,
        },
    }


def build_plan(
    manifest: CausalManifest,
    *,
    adapters: AdapterTemplates | None = None,
    pilot: bool = False,
    output_root: str | Path = "v6r5_module2_causal",
) -> dict[str, Any]:
    """Return the concrete pilot/formal command plan; perform no mutation."""

    rows = []
    root = Path(output_root).expanduser().resolve()
    for run in selected_runs(manifest, pilot=pilot):
        arm = manifest.arms[run.arm]
        values = _adapter_values(manifest, run, root)
        row = {
            "run_id": run.run_id,
            "repeat": run.repeat,
            "arm": run.arm,
            "identity": dict(manifest.identity),
            "setup": {
                "integration_startup_profile": "estimated_autonomy",
                "integration_process_required": arm.integration_process_required,
                "module2_uds_enabled": arm.module2_uds_enabled,
                "integration_bridge_enabled": arm.integration_bridge_enabled,
                "integration_startup_profile": {
                    "M0": "off",
                    "M1": "estimated_shadow",
                    "M2": "module2_causal_obstacle_active",
                    "M3": "module2_causal_obstacle_active",
                }[run.arm],
                "active_effect_scope": {
                    "M0": "none",
                    "M1": "shadow",
                    "M2": "obstacle_only",
                    "M3": "obstacle_only",
                }[run.arm],
                "module2_socket": values["module2_socket"],
                "module3_mode": arm.module3_mode,
                "obstacle_layer_mode": arm.obstacle_layer_mode,
                "critic_mode": arm.critic_mode,
                "graph_backend": "gvg",
                "direct_rgbd_costmap_enabled": False,
                "low_obstacles_enabled": True,
                "dynamic_actors_enabled": False,
                "module1_amcl_prior_enabled": False,
                "cognitive_place_graph_enabled": False,
                "scene_asset": manifest.identity["scene_asset"],
                "occupancy_map": manifest.identity["occupancy_map"],
                "spawn_manifest": manifest.identity["spawn_manifest"],
                "route_graph": manifest.identity["route_graph"],
                "obstacle_config": manifest.identity["obstacle_config"],
                "obstacle_manifest": manifest.identity["obstacle_manifest"],
                "navigation_overlay": manifest.identity["navigation_overlay"],
            },
            "reset_state_machine": (
                "wait_readiness", "set_seed_8601", "one_reset_call",
                "one_reset_event", "bridge_epoch_plus_one", "localization_seeded",
                "one_route_goal_G2",
            ),
            "dispatcher_topics": DISPATCHER_TOPICS,
            "passive_evaluator_topics": PASSIVE_EVALUATOR_TOPICS,
            "isolation_audit_topics": ISOLATION_AUDIT_TOPICS,
            "bag_topics": (
                *DISPATCHER_TOPICS,
                *PASSIVE_EVALUATOR_TOPICS,
                *ISOLATION_AUDIT_TOPICS,
            ),
            "run_directory": values["run_dir"],
            "evidence_file": values["evidence_json"],
        }
        if adapters is not None:
            row["commands"] = {
                "scene": render_adapter_command(adapters.scene, values),
                "stack": render_adapter_command(adapters.stack, values),
                "episode": render_adapter_command(adapters.episode, values),
            }
            if adapters.producer_stop is not None:
                row["commands"]["producer_stop"] = render_adapter_command(
                    adapters.producer_stop, values
                )
        rows.append(row)
    return {
        "qualification": QUALIFICATION,
        "mode": "pilot" if pilot else "formal_12",
        "dispatch": adapters is not None,
        "reason": None if adapters is not None else "external_scene_stack_episode_adapters_required",
        "exactly_once_reset_contract": "reuse_v6_formal_episode_guard",
        "integration_cli_contract": {
            "M0": "no Module2 server or Bridge",
            "M1": "estimated_shadow; no navigation write",
            "M2_M3_server": (
                "scripts/run_v6_module2_causal_obstacle_server.sh "
                "--startup-profile module2_causal_obstacle_active "
                "--active-effect-scope obstacle_only --socket {module2_socket} "
                "--module2-root <MODULE2_ROOT> --shadow-config "
                "configs/kujiale_0026_module1_visual_shadow_v310.yaml"
            ),
            "planning_prior": "untrusted",
            "obstacle_transport": "trusted",
            "edge_prior": "off",
            "cognitive_place_graph": "off",
            "module1_initialpose_writer": "off",
        },
        "episode_adapter_contract": (
            "{module3_root}/scripts/run_v6_low_obstacle_causal.sh dispatch-episode "
            "--run-id {run_id} --output-jsonl {episode_jsonl}"
        ),
        "recommended_adapter_templates": {
            "scene": (
                "{module3_root}/scripts/run_v6_r5_phase_b_kujiale.sh "
                "--domain {ros_domain_id} isaac --dynamic-obstacle-config "
                "{obstacle_config} --dynamic-obstacles"
            ),
            "stack": (
                "{module3_root}/scripts/run_v6_low_obstacle_phase_f_stack.sh "
                "{arm} --domain {ros_domain_id} --run-dir {run_dir} "
                "--socket {module2_socket}"
            ),
            "episode": (
                "{module3_root}/scripts/run_v6_low_obstacle_causal.sh "
                "dispatch-episode --run-id {run_id} --output-jsonl {episode_jsonl}"
            ),
            "producer_stop": (
                "{module3_root}/scripts/run_v6_low_obstacle_phase_f_stack.sh "
                "stop-producer --run-dir {run_dir}"
            ),
        },
        "ordered_shutdown": (
            "episode", "module2_producer", "ttl_clear_observation",
            "stack", "recorder", "scene",
        ),
        "runs": rows,
    }


def read_rosbag_records(
    bag_dir: str | Path,
    *,
    topics: Sequence[str] | None = None,
) -> Iterable[RecordedMessage]:
    """Yield deserialized messages from an MCAP/rosbag2 directory."""

    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise CausalContractError("rosbag2 Python runtime is unavailable") from exc
    uri = str(Path(bag_dir).expanduser().resolve())
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=uri, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    type_by_topic = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    selected = set(topics or type_by_topic)
    message_types = {
        topic: get_message(type_name)
        for topic, type_name in type_by_topic.items()
        if topic in selected
    }
    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        message_type = message_types.get(topic)
        if message_type is None:
            continue
        yield RecordedMessage(topic, int(stamp_ns), deserialize_message(data, message_type))


def _episode_result_from_jsonl(path: Path) -> Mapping[str, Any]:
    result: Mapping[str, Any] | None = None
    for line in path.read_text(encoding="utf-8").splitlines():
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(row, Mapping) and row.get("event") == "episode_result":
            result = row
    if result is None:
        raise CausalContractError(f"episode_result event missing from {path}")
    return result


def record_evidence_from_bag(
    manifest: CausalManifest,
    run: RunContract,
    bag_dir: str | Path,
    episode_jsonl: str | Path,
    output: str | Path,
) -> dict[str, Any]:
    result = _episode_result_from_jsonl(Path(episode_jsonl).expanduser().resolve())
    evidence = build_recorded_evidence(
        manifest,
        run,
        read_rosbag_records(
            bag_dir,
            topics=(*DISPATCHER_TOPICS, *PASSIVE_EVALUATOR_TOPICS, *ISOLATION_AUDIT_TOPICS),
        ),
        result,
    )
    destination = Path(output).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(evidence, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return evidence


def dispatch_episode(
    manifest: CausalManifest,
    run: RunContract,
    output_jsonl: str | Path,
    *,
    readiness_timeout_sec: float,
    reset_timeout_sec: float,
    navigation_timeout_sec: float,
) -> Mapping[str, Any]:
    """Reuse the existing V6 formal reset/route/terminal-zero node for one arm."""

    from robot_experiments.v6_formal import (
        ENGINEERING_PILOT,
        Episode,
        Manifest,
        MissionLeg,
        V6FormalNode,
    )

    arm = manifest.arms[run.arm]
    runtime = {
        "canonical_odom": {"topic": "/odom", "owner": "isaac_compute_odometry", "tf": "odom->base_link"},
        "global_localization": {"pose_topic": "/amcl_pose", "owner": "amcl", "tf": "map->odom"},
        "module1_odom": {
            "topic": "/bio_nav/module1/odom", "owner": "wheel_imu_ekf", "publish_tf": False,
        },
        "recovery_enabled": False,
        "module1_amcl_prior_enabled": False,
        "module2_navigation_write_enabled": run.arm in {"M2", "M3"},
        "module2_active_effect_scope": "obstacle_only" if run.arm in {"M2", "M3"} else (
            "shadow" if run.arm == "M1" else "none"
        ),
        "cognitive_place_graph_enabled": False,
        "route_backend": "gvg",
        "low_obstacles_enabled": True,
        "dynamic_actors_enabled": False,
        "goal_checker": "position_xy",
        "cognitive_profile": run.arm,
        "obstacle_layer_mode": arm.obstacle_layer_mode,
        "critic_mode": arm.critic_mode,
    }
    formal_manifest = Manifest(
        path=manifest.path,
        raw={},
        scene_id="kujiale_0026_A_to_B_door_open",
        category="static",
        runtime=runtime,
        assets={
            "scene_asset": str(manifest.identity["scene_asset"]),
            "occupancy_map": str(manifest.identity["occupancy_map"]),
            "spawn_manifest": str(manifest.identity["spawn_manifest"]),
            "route_graph": str(manifest.identity["route_graph"]),
            "obstacle_config": str(manifest.identity["obstacle_config"]),
            "obstacle_manifest": str(manifest.identity["obstacle_manifest"]),
        },
        reset_pose={"id": "G1", "frame_id": "map", "x": 0.45, "y": -5.35, "yaw_deg": 90.0},
        mission_legs=(MissionLeg("G2", "map", 0.80, 4.80),),
        dynamic_schedule=(),
        episodes=(),
    )
    episode = Episode(
        seed=int(manifest.identity["seed"]),
        variant_id="v6_phase_f_r2",
        appearance_profile_id=None,
        reset_pose_name="long_route_start_g1",
        dynamic_case_id="static",
    )
    import rclpy
    rclpy.init(args=None)
    adapter = V6FormalNode(
        formal_manifest,
        episode,
        Path(output_jsonl).expanduser().resolve(),
        qualification=ENGINEERING_PILOT,
    )
    try:
        return adapter.run(
            readiness_timeout_sec=readiness_timeout_sec,
            reset_timeout_sec=reset_timeout_sec,
            navigation_timeout_sec=navigation_timeout_sec,
        )
    finally:
        adapter.destroy()
        rclpy.shutdown()


@dataclass
class _ManagedProcess:
    name: str
    process: subprocess.Popen[Any]
    stream: Any


def _startup_process_failure(managed: Sequence[_ManagedProcess]) -> str | None:
    for item in managed:
        returncode = item.process.poll()
        if returncode is not None:
            return f"{item.name} exited before startup readiness (returncode={returncode})"
    return None


def _wait_for_startup_ready(
    managed: Sequence[_ManagedProcess],
    timeout_sec: float,
) -> dict[str, Any]:
    """Wait for the completed and released Isaac-owned startup reset epoch.

    The reset-stop status is transient-local, so a subscriber created after
    the startup event observes the current gate state without replaying the
    startup reset event into the formal episode guard.  Generation one is the
    Isaac startup reset; generation two is reserved for the dispatcher.
    """

    if not math.isfinite(timeout_sec) or timeout_sec <= 0.0:
        raise CausalContractError("startup readiness timeout must be finite and positive")
    try:
        import rclpy
        from rclpy.context import Context
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from std_msgs.msg import String
    except ImportError as exc:
        return {"ready": False, "reason": f"ROS startup probe unavailable: {exc}"}

    context = Context()
    node = None
    observed: dict[str, Any] = {}
    try:
        rclpy.init(args=None, context=context)
        node = rclpy.create_node(
            f"v6_phase_f_startup_probe_{os.getpid()}", context=context
        )
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        def receive(message: Any) -> None:
            try:
                value = json.loads(message.data)
            except (AttributeError, json.JSONDecodeError, TypeError):
                return
            if isinstance(value, Mapping):
                observed.clear()
                observed.update(value)

        subscription = node.create_subscription(
            String, "/simulation/reset_stop_gate/status", receive, qos
        )
        deadline = time.monotonic() + timeout_sec
        while True:
            failure = _startup_process_failure(managed)
            if failure is not None:
                return {"ready": False, "reason": failure, "last_status": dict(observed)}
            generation = observed.get("generation")
            held = observed.get("held")
            reason = str(observed.get("reason", ""))
            if generation == 1 and held is False and reason.startswith("released:"):
                return {
                    "ready": True,
                    "generation": generation,
                    "held": held,
                    "reason": reason,
                }
            if isinstance(generation, int) and not isinstance(generation, bool) and generation > 1:
                return {
                    "ready": False,
                    "reason": f"unexpected startup reset generation {generation}",
                    "last_status": dict(observed),
                }
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return {
                    "ready": False,
                    "reason": "startup reset generation 1 was not released before timeout",
                    "last_status": dict(observed),
                }
            rclpy.spin_once(node, timeout_sec=min(remaining, 0.5))
    except Exception as exc:
        return {
            "ready": False,
            "reason": f"startup readiness probe failed: {type(exc).__name__}: {exc}",
            "last_status": dict(observed),
        }
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if context.ok():
            try:
                rclpy.shutdown(context=context)
            except Exception:
                pass


def _start_process(
    name: str,
    command: Sequence[str],
    log_path: Path,
    *,
    env: Mapping[str, str] | None = None,
) -> _ManagedProcess:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    stream = log_path.open("w", encoding="utf-8")
    try:
        process = subprocess.Popen(
            list(command),
            stdout=stream,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            text=True,
            env=dict(env) if env is not None else None,
        )
    except Exception:
        stream.close()
        raise
    return _ManagedProcess(name, process, stream)


def _stop_process(managed: _ManagedProcess, timeout_sec: float) -> dict[str, Any]:
    process = managed.process
    groups = _managed_process_groups(process.pid)
    try:
        if process.poll() is None or _running_process_groups(groups):
            _signal_process_groups(groups, signal.SIGINT)
            if not _wait_process_groups(groups, timeout_sec):
                _signal_process_groups(groups, signal.SIGTERM)
                if not _wait_process_groups(groups, 5.0):
                    _signal_process_groups(groups, signal.SIGKILL)
                    _wait_process_groups(groups, 5.0)
        if process.poll() is None:
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                pass
    finally:
        managed.stream.close()
    remaining = sorted(_running_process_groups(groups))
    return {
        "name": managed.name,
        "returncode": process.returncode,
        "cleanup_ok": process.poll() is not None and not remaining,
        "tracked_process_groups": sorted(groups),
        "remaining_process_groups": remaining,
    }


def _process_table() -> dict[int, tuple[int, int, str]]:
    table: dict[int, tuple[int, int, str]] = {}
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            raw = (entry / "stat").read_text(encoding="utf-8")
            tail = raw[raw.rindex(")") + 2:].split()
            table[int(entry.name)] = (int(tail[1]), int(tail[2]), tail[0])
        except (FileNotFoundError, PermissionError, ValueError, IndexError):
            continue
    return table


def _managed_process_groups(root_pid: int) -> set[int]:
    table = _process_table()
    descendants = {root_pid}
    while True:
        added = {
            pid for pid, (parent, _group, _state) in table.items()
            if parent in descendants and pid not in descendants
        }
        if not added:
            break
        descendants.update(added)
    own_group = os.getpgrp()
    return {
        group for pid in descendants
        if (row := table.get(pid)) is not None
        for group in (row[1],)
        if group > 1 and group != own_group
    }


def _running_process_groups(groups: Iterable[int]) -> set[int]:
    selected = set(groups)
    return {
        group for _pid, (_parent, group, state) in _process_table().items()
        if group in selected and state != "Z"
    }


def _signal_process_groups(groups: Iterable[int], signal_number: int) -> None:
    for group in sorted(set(groups), reverse=True):
        try:
            os.killpg(group, signal_number)
        except ProcessLookupError:
            continue


def _wait_process_groups(groups: Iterable[int], timeout_sec: float) -> bool:
    deadline = time.monotonic() + max(0.0, timeout_sec)
    while _running_process_groups(groups):
        if time.monotonic() >= deadline:
            return False
        time.sleep(0.05)
    return True


def _lock_is_free(path: Path) -> bool:
    import fcntl

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as stream:
        try:
            fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return False
        fcntl.flock(stream.fileno(), fcntl.LOCK_UN)
    return True


def _confirm_arm_cleanup(
    run_dir: Path,
    module2_socket: Path,
    shutdown: Sequence[Mapping[str, Any]],
    env: Mapping[str, str],
) -> dict[str, Any]:
    runtime_dir = Path(
        env.get("ISAAC_NAV_RUNTIME_DIR", f"/tmp/isaac_sim_ros2_nav_{os.getuid()}")
    ).expanduser().resolve()
    locks = {
        name: _lock_is_free(runtime_dir / f"{name}.lock")
        for name in ("ros", "isaac")
    }
    stale_runtime_files = sorted(
        str(path)
        for pattern in ("*.pid", "*.pgid")
        for path in run_dir.glob(pattern)
    )
    process_cleanup_ok = all(
        row.get("cleanup_ok", row.get("returncode") is not None)
        and not row.get("remaining_process_groups")
        for row in shutdown
        if row.get("name") in {"scene", "stack", "recorder"}
    )
    result = {
        "ok": (
            process_cleanup_ok
            and all(locks.values())
            and not stale_runtime_files
            and not module2_socket.exists()
        ),
        "processes_clean": process_cleanup_ok,
        "locks_free": locks,
        "stale_runtime_files": stale_runtime_files,
        "module2_socket_absent": not module2_socket.exists(),
    }
    return result


def _rosbag_command(bag_dir: Path) -> tuple[str, ...]:
    return (
        "ros2", "bag", "record", "--storage", "mcap",
        "--include-unpublished-topics", "--output", str(bag_dir),
        *DISPATCHER_TOPICS, *PASSIVE_EVALUATOR_TOPICS, *ISOLATION_AUDIT_TOPICS,
    )


def run_campaign(
    manifest: CausalManifest,
    adapters: AdapterTemplates,
    output_root: str | Path,
    *,
    pilot: bool,
    shutdown_timeout_sec: float = DEFAULT_SHUTDOWN_TIMEOUT_SEC,
) -> dict[str, Any]:
    """Run ordered independent episodes; stop if an arm cannot be cleaned."""

    root = Path(output_root).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    plan = build_plan(manifest, adapters=adapters, pilot=pilot, output_root=root)
    existing = [row["run_directory"] for row in plan["runs"] if Path(row["run_directory"]).exists()]
    if existing:
        raise CausalContractError("refusing to overwrite run directories: " + ",".join(existing))
    campaign_env = dict(os.environ)
    campaign_env["ROS_DOMAIN_ID"] = str(manifest.identity["ros_domain_id"])
    campaign_env["ISAAC_NAV_EXPECTED_DOMAIN_ID"] = str(manifest.identity["ros_domain_id"])
    campaign_env.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    if manifest.module3_root is None:
        raise CausalContractError(
            f"run requires {MODULE3_ROOT_ENV} when using an installed manifest"
        )
    fastdds_profile = manifest.module3_root / "isaac_sim/configs/ros2_bridge/fastdds_udp_only.xml"
    campaign_env.setdefault("FASTRTPS_DEFAULT_PROFILES_FILE", str(fastdds_profile))
    campaign_env.setdefault("FASTDDS_DEFAULT_PROFILES_FILE", str(fastdds_profile))
    results: list[dict[str, Any]] = []
    for row in plan["runs"]:
        run = next(item for item in manifest.runs if item.run_id == row["run_id"])
        run_dir = Path(row["run_directory"])
        run_dir.mkdir(parents=True, exist_ok=True)
        managed: list[_ManagedProcess] = []
        status: dict[str, Any] = {
            "run_id": run.run_id,
            "repeat": run.repeat,
            "arm": run.arm,
            "state": "STARTED",
            "commands": row["commands"],
            "shutdown": [],
        }
        cleanup_failed = False
        try:
            managed.append(_start_process(
                "scene", row["commands"]["scene"], run_dir / "scene.log", env=campaign_env
            ))
            managed.append(_start_process(
                "stack", row["commands"]["stack"], run_dir / "stack.log", env=campaign_env
            ))
            startup = _wait_for_startup_ready(
                managed,
                float(manifest.identity["timeout_sec"]),
            )
            status["startup"] = startup
            if startup.get("ready") is not True:
                status["state"] = "STARTUP_NOT_READY"
                status["reason"] = str(startup.get("reason", "startup readiness failed"))
            else:
                managed.append(_start_process(
                    "recorder", _rosbag_command(run_dir / "bag"), run_dir / "recorder.log", env=campaign_env
                ))
                with (run_dir / "episode.stdout.log").open("w", encoding="utf-8") as stdout:
                    completed = subprocess.run(
                        list(row["commands"]["episode"]),
                        stdout=stdout,
                        stderr=subprocess.STDOUT,
                        text=True,
                        check=False,
                        env=campaign_env,
                    )
                status["episode_returncode"] = completed.returncode
                status["state"] = "EPISODE_FINISHED" if completed.returncode == 0 else "EPISODE_FAILED"
                if run.arm in {"M2", "M3"}:
                    producer_stop = row["commands"].get("producer_stop")
                    if not producer_stop:
                        raise CausalContractError("active arm requires producer_stop adapter")
                    with (run_dir / "producer_stop.log").open("w", encoding="utf-8") as stdout:
                        stopped = subprocess.run(
                            list(producer_stop),
                            stdout=stdout,
                            stderr=subprocess.STDOUT,
                            text=True,
                            check=False,
                            env=campaign_env,
                        )
                    status["producer_stop_returncode"] = stopped.returncode
                    if stopped.returncode != 0:
                        raise CausalContractError("Module2 producer stop adapter failed")
                    drain_sec = float(manifest.freshness["typed_obstacle_ttl_sec"]) + float(
                        manifest.freshness["post_producer_stop_observation_margin_sec"]
                    )
                    status["ttl_observation_wait_sec"] = drain_sec
                    time.sleep(drain_sec)
                else:
                    status["ttl_observation_wait_sec"] = None
        except (OSError, CausalContractError) as exc:
            if status["state"] != "STARTUP_NOT_READY":
                status["state"] = "ADAPTER_FAILED"
            status["reason"] = str(exc)
        finally:
            by_name = {process.name: process for process in managed}
            for name in ("stack", "recorder", "scene"):
                process = by_name.get(name)
                if process is None:
                    continue
                try:
                    status["shutdown"].append(_stop_process(process, shutdown_timeout_sec))
                except OSError as exc:
                    status["shutdown"].append({"name": process.name, "error": str(exc)})
            try:
                cleanup = _confirm_arm_cleanup(
                    run_dir,
                    Path(row["setup"]["module2_socket"]),
                    status["shutdown"],
                    campaign_env,
                )
            except OSError as exc:
                cleanup = {"ok": False, "error": str(exc)}
            status["cleanup"] = cleanup
            if cleanup.get("ok") is not True:
                cleanup_failed = True
                status["state_before_cleanup"] = status["state"]
                status["state"] = "ARM_CLEANUP_FAILED"
                status["reason"] = "arm children or runtime locks remained after shutdown"

        evidence_path = run_dir / f"{run.run_id}.json"
        try:
            evidence = record_evidence_from_bag(
                manifest,
                run,
                run_dir / "bag",
                run_dir / "episode.jsonl",
                evidence_path,
            )
            status["evidence_file"] = str(evidence_path)
            status["evidence_recorded"] = True
            if run.arm in {"M2", "M3"}:
                freshness = _mapping(evidence.get("freshness"), "freshness")
                if (
                    freshness.get("ttl_expiry_observed") is not True
                    or freshness.get("ttl_expiry_zero_write") is not True
                    or (
                        run.arm == "M3"
                        and (
                            freshness.get("critic_post_expiry_applied") is not False
                            or freshness.get("critic_ttl_status") not in {
                                "N/A_NO_CONTROLLER_SCORING", "STALE_REJECTED",
                            }
                            or freshness.get("critic_stale_active_probe") != "NOT_RUN"
                        )
                    )
                ):
                    status["state"] = "TTL_CLEAR_FAILED"
                    status["reason"] = "post-producer TTL lifecycle was not clean"
        except (OSError, CausalContractError, RuntimeError, ValueError) as exc:
            status["evidence_recorded"] = False
            status["evidence_error"] = str(exc)
            if status["state"] == "EPISODE_FINISHED":
                status["state"] = "EVIDENCE_FAILED"
        (run_dir / "run_status.json").write_text(
            json.dumps(status, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        results.append(status)
        if cleanup_failed:
            break

    summary = {
        "qualification": RUN_QUALIFICATION,
        "mode": plan["mode"],
        "state": "FINISHED_WITH_FAILURES" if any(
            row["state"] != "EPISODE_FINISHED" for row in results
        ) else "FINISHED",
        "output_root": str(root),
        "runs": results,
    }
    (root / "campaign_result.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return summary


def _points(value: Any, name: str) -> tuple[tuple[float, float], ...]:
    if not isinstance(value, list) or not value:
        raise CausalContractError(f"{name} must be a non-empty point list")
    result: list[tuple[float, float]] = []
    for index, point in enumerate(value):
        if isinstance(point, Mapping):
            x, y = point.get("x"), point.get("y")
        elif isinstance(point, Sequence) and not isinstance(point, (str, bytes)) and len(point) >= 2:
            x, y = point[0], point[1]
        else:
            raise CausalContractError(f"{name}[{index}] must be a point")
        try:
            result.append((float(x), float(y)))
        except (TypeError, ValueError) as exc:
            raise CausalContractError(f"{name}[{index}] has non-numeric coordinates") from exc
    return tuple(result)


def path_length(points: Sequence[tuple[float, float]]) -> float:
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def path_hausdorff(lhs: Sequence[tuple[float, float]], rhs: Sequence[tuple[float, float]]) -> float:
    def directed(a: Sequence[tuple[float, float]], b: Sequence[tuple[float, float]]) -> float:
        return max(min(math.dist(point, candidate) for candidate in b) for point in a)
    return max(directed(lhs, rhs), directed(rhs, lhs))


def path_direction(points: Sequence[tuple[float, float]]) -> str:
    start, goal = points[0], points[-1]
    vx, vy = goal[0] - start[0], goal[1] - start[1]
    norm = math.hypot(vx, vy)
    if norm <= 1e-9:
        return "unknown"
    signed = [vx * (point[1] - start[1]) - vy * (point[0] - start[0]) for point in points[1:-1]]
    if not signed:
        return "straight"
    mean_offset = statistics.fmean(signed) / norm
    if mean_offset > 0.05:
        return "left"
    if mean_offset < -0.05:
        return "right"
    return "straight"


def _footprint_center(value: Any) -> tuple[float, float]:
    row = _mapping(value, "rgbd_obstacle_footprint")
    if isinstance(row.get("center"), Sequence):
        center = row["center"]
        return float(center[0]), float(center[1])
    return float(row["x"]), float(row["y"])


def _scan_and_spatial_metrics(samples: Any, tolerance_m: float) -> tuple[int, int, int, int]:
    if not isinstance(samples, list) or not samples:
        raise CausalContractError("synchronized_samples must be a non-empty list")
    synchronized = invisible = matched = total = 0
    for index, raw_sample in enumerate(samples):
        sample = _mapping(raw_sample, f"synchronized_samples[{index}]")
        stamp = sample.get("stamp_ns")
        frame = sample.get("frame_id")
        if isinstance(stamp, bool) or not isinstance(stamp, int) or stamp < 0 or not isinstance(frame, str) or not frame:
            raise CausalContractError(f"synchronized_samples[{index}] missing valid frame/time")
        synchronized += 1
        footprints = sample.get("rgbd_obstacle_footprints")
        typed = sample.get("typed_obstacles")
        if not isinstance(footprints, list) or not isinstance(typed, list):
            raise CausalContractError(f"synchronized_samples[{index}] missing obstacle arrays")
        depth_valid = sample.get("depth_observation_valid")
        if not isinstance(depth_valid, bool):
            raise CausalContractError(
                f"synchronized_samples[{index}].depth_observation_valid invalid"
            )
        depth_reason = sample.get("depth_observation_reason")
        if not isinstance(depth_reason, str) or not depth_reason:
            raise CausalContractError(
                f"synchronized_samples[{index}].depth_observation_reason invalid"
            )
        depth_points = sample.get("depth_point_count")
        depth_hits = sample.get("depth_hits_in_obstacle_bounds")
        if (
            isinstance(depth_points, bool) or not isinstance(depth_points, int) or depth_points < 0
            or isinstance(depth_hits, bool) or not isinstance(depth_hits, int) or depth_hits < 0
        ):
            raise CausalContractError(f"synchronized_samples[{index}] depth counts invalid")
        if footprints and (not depth_valid or depth_hits <= 0):
            raise CausalContractError(
                f"synchronized_samples[{index}] footprint lacks real depth hits"
            )
        for footprint in footprints:
            footprint_row = _mapping(footprint, "rgbd_obstacle_footprint")
            if footprint_row.get("source") != "projected_depth_points":
                raise CausalContractError(
                    f"synchronized_samples[{index}] footprint source is not projected depth"
                )
            if int(footprint_row.get("point_count", 0)) <= 0:
                raise CausalContractError(
                    f"synchronized_samples[{index}] footprint point_count invalid"
                )
        scan_points = sample.get("scan_point_count")
        scan_hits = sample.get("scan_hits_in_obstacle_footprints")
        if isinstance(scan_points, bool) or not isinstance(scan_points, int) or scan_points < 0:
            raise CausalContractError(f"synchronized_samples[{index}].scan_point_count invalid")
        if isinstance(scan_hits, bool) or not isinstance(scan_hits, int) or scan_hits < 0:
            raise CausalContractError(f"synchronized_samples[{index}].scan_hits_in_obstacle_footprints invalid")
        scan_valid = sample.get("scan_valid")
        if not isinstance(scan_valid, bool):
            raise CausalContractError(f"synchronized_samples[{index}].scan_valid invalid")
        if footprints and scan_valid and scan_points > 0 and scan_hits == 0:
            invisible += 1
        centers = tuple(_footprint_center(item) for item in footprints)
        for obstacle_value in typed:
            obstacle = _mapping(obstacle_value, "typed_obstacle")
            if obstacle.get("accepted") is not True:
                continue
            total += 1
            point = (float(obstacle["x"]), float(obstacle["y"]))
            computed_error = min(
                (math.dist(point, center) for center in centers), default=None
            )
            reported_error = obstacle.get("observed_spatial_error_m")
            if computed_error is None:
                if reported_error is not None:
                    raise CausalContractError(
                        f"synchronized_samples[{index}] spatial error lacks depth footprint"
                    )
            elif (
                isinstance(reported_error, bool)
                or not isinstance(reported_error, (int, float))
                or not math.isfinite(float(reported_error))
                or not math.isclose(float(reported_error), computed_error, abs_tol=1.0e-9)
            ):
                raise CausalContractError(
                    f"synchronized_samples[{index}] spatial error is not observed geometry"
                )
            if computed_error is not None and computed_error <= tolerance_m:
                matched += 1
    return synchronized, invisible, matched, total


REQUIRED_EVIDENCE_KEYS = (
    "run_id", "repeat", "arm", "identity", "reset", "freshness",
    "synchronized_samples", "obstacle_validation", "layer", "critic",
    "planning_prior", "costmaps", "plan", "optimal_trajectory", "odom",
    "cmd_vel", "passive", "action", "route", "module2_health", "isolation",
    "navigation_metrics",
)


def _evaluate_run(manifest: CausalManifest, run: RunContract, path: Path) -> tuple[RunResult, dict[str, Any]]:
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
        row = _mapping(raw, str(path))
        missing = [key for key in REQUIRED_EVIDENCE_KEYS if key not in row]
        if missing:
            raise CausalContractError("missing evidence: " + ", ".join(missing))
        if (row["run_id"], row["repeat"], row["arm"]) != (run.run_id, run.repeat, run.arm):
            raise CausalContractError("run identity does not match manifest row")
        recorded_identity = _mapping(row["identity"], "identity")
        for key in (
            "scene_id", "obstacle_layout_id", "seed", "start", "goal", "timeout_sec",
            "route_backend", "graph_backend", "direct_rgbd_costmap_enabled",
        ):
            if recorded_identity.get(key) != manifest.identity.get(key):
                raise CausalContractError(f"identity.{key} differs from frozen manifest")

        reset = _mapping(row["reset"], "reset")
        if (reset.get("calls"), reset.get("events"), reset.get("goal_publications")) != (1, 1, 1):
            raise CausalContractError("exactly-once reset/goal evidence invalid")
        if reset.get("localization_contract") != "same_estimated_autonomy":
            raise CausalContractError("localization contract differs across arms")

        arm = manifest.arms[run.arm]
        if run.arm == "M0" and row.get("module2_uds_connected", False):
            raise CausalContractError("M0 must not connect Module2 UDS")
        if run.arm != "M0" and row.get("module2_uds_connected") is not True:
            raise CausalContractError(f"{run.arm} requires Module2 UDS evidence")
        health = _mapping(row["module2_health"], "module2_health")
        if health.get("scope") != "low_obstacle_only":
            raise CausalContractError("Module2 scope must remain low_obstacle_only")
        if run.arm == "M0":
            if any(int(health.get(key, 0)) for key in ("message_count", "healthy_count", "trusted_write_count")):
                raise CausalContractError("M0 must not record Module2 obstacle output")
        elif (
            int(health.get("message_count", 0)) <= 0
            or int(health.get("healthy_count", 0)) <= 0
            or int(health.get("observation_valid_count", 0)) <= 0
            or float(health.get("candidate_cadence_hz", 0.0)) <= 0.0
        ):
            raise CausalContractError(f"{run.arm} lacks healthy valid Module2 output")
        if run.arm in {"M2", "M3"} and int(health.get("trusted_write_count", 0)) <= 0:
            raise CausalContractError(f"{run.arm} lacks trusted obstacle transport")

        isolation = _mapping(row["isolation"], "isolation")
        if (
            isolation.get("module1_amcl_prior_enabled") is not False
            or isolation.get("cognitive_place_graph_enabled") is not False
            or isolation.get("dynamic_actors_enabled") is not False
        ):
            raise CausalContractError("Phase-F isolation knobs differ across arms")
        unexpected = _mapping(isolation.get("unexpected_topic_counts"), "isolation.unexpected_topic_counts")
        if set(unexpected) != set(ISOLATION_AUDIT_TOPICS) or any(int(value) for value in unexpected.values()):
            raise CausalContractError("Module1 prior/edge-prior/CPG isolation traffic observed")

        layer = _mapping(row["layer"], "layer")
        critic = _mapping(row["critic"], "critic")
        if layer.get("mode") != arm.obstacle_layer_mode or critic.get("mode") != arm.critic_mode:
            raise CausalContractError("recorded layer/critic arm does not match manifest")
        layer_cells: list[int] = []
        for scope in ("global", "local"):
            layer_status = _mapping(layer.get(scope), f"layer.{scope}")
            if run.arm != "M0" and int(layer_status.get("status_count", 0)) <= 0:
                raise CausalContractError(f"layer.{scope} status evidence missing")
            cells = int(layer_status.get("cells", -1))
            max_cost = int(layer_status.get("max_cost", -1))
            if cells < 0 or not 0 <= max_cost <= 255:
                raise CausalContractError(f"layer.{scope} cells/max_cost invalid")
            layer_cells.append(cells)
        if arm.obstacle_layer_mode in {"off", "shadow"} and any(layer_cells):
            raise CausalContractError("off/shadow obstacle layer wrote Costmap cells")
        if arm.obstacle_layer_mode == "active" and any(cells <= 0 for cells in layer_cells):
            raise CausalContractError("active obstacle layer lacks global/local applied cells")
        if arm.critic_mode != "active" and critic.get("applied") is not False:
            raise CausalContractError("off/shadow critic must not be applied")
        if not isinstance(critic.get("reason"), str) or not critic["reason"]:
            raise CausalContractError("critic reason evidence missing")
        if not isinstance(row["planning_prior"], list):
            raise CausalContractError("planning_prior must be a list")
        if run.arm == "M0" and row["planning_prior"]:
            raise CausalContractError("M0 planning_prior must remain empty")
        for key in ("global", "local"):
            costmap = _mapping(row["costmaps"], "costmaps").get(key)
            if not isinstance(costmap, Mapping) or not costmap.get("recorded"):
                raise CausalContractError(f"costmaps.{key} missing")

        tolerance = float(manifest.criteria["typed_spatial_match_tolerance_m"])
        synchronized, invisible, matches, spatial_total = _scan_and_spatial_metrics(
            row["synchronized_samples"], tolerance
        )
        if run.arm != "M0" and spatial_total == 0:
            raise CausalContractError("typed obstacle spatial evidence missing")
        if run.arm != "M0" and matches != spatial_total:
            raise CausalContractError("typed obstacle spatial match failed")
        validations = row["obstacle_validation"]
        if not isinstance(validations, list):
            raise CausalContractError("obstacle_validation must be a list")
        if run.arm != "M0" and not validations:
            raise CausalContractError("typed obstacle validation evidence missing")

        freshness = _mapping(row["freshness"], "freshness")
        expected_ttl_applicability = (
            "required_active_write" if run.arm in {"M2", "M3"}
            else "not_applicable_inactive"
        )
        if freshness.get("ttl_clear_applicability") != expected_ttl_applicability:
            raise CausalContractError("TTL-clear applicability does not match arm")
        max_age = float(freshness.get("max_typed_obstacle_age_sec", 0.0))
        stale_applied = int(freshness.get("stale_applied_count", 0))
        if run.arm in {"M2", "M3"} and (
            not isinstance(freshness.get("ttl_source_sequence"), int)
            or int(freshness.get("ttl_source_sequence", 0)) <= 0
            or not isinstance(freshness.get("ttl_expiry_stamp_ns"), int)
            or freshness.get("ttl_expiry_observed") is not True
            or freshness.get("ttl_expiry_zero_write") is not True
        ):
            raise CausalContractError("active arm lacks clean TTL-expiry evidence")
        if run.arm == "M3":
            critic_ttl_status = freshness.get("critic_ttl_status")
            critic_post_expiry_applied = _bool(
                freshness.get("critic_post_expiry_applied"),
                "freshness.critic_post_expiry_applied",
            )
            if freshness.get("critic_stale_active_probe") != "NOT_RUN":
                raise CausalContractError("M3 critic stale active probe state is invalid")
            if critic_post_expiry_applied or stale_applied > 0:
                verdict = "INVALID"
                reasons = ("stale_input_applied_after_expiry",)
            elif critic_ttl_status not in {
                "N/A_NO_CONTROLLER_SCORING", "STALE_REJECTED",
            }:
                raise CausalContractError("M3 post-expiry critic callback was not safely rejected")
            else:
                verdict, reasons = "VALID", ()
        elif stale_applied > 0:
            verdict = "INVALID"
            reasons = ("stale_input_applied_after_expiry",)
        else:
            verdict, reasons = "VALID", ()

        plan = _points(row["plan"], "plan")
        optimal_trajectory = _points(row["optimal_trajectory"], "optimal_trajectory")
        _points(row["odom"], "odom")
        if not isinstance(row["cmd_vel"], list) or not row["cmd_vel"]:
            raise CausalContractError("cmd_vel evidence missing")
        passive = _mapping(row["passive"], "passive")
        _points(passive.get("ground_truth_odom"), "passive.ground_truth_odom")
        clearance = float(passive["minimum_clearance_m"])
        collision = _bool(passive.get("collision"), "passive.collision")
        success = _bool(passive.get("success"), "passive.success")
        action = _mapping(row["action"], "action")
        terminal_zero = _bool(action.get("terminal_zero_confirmed"), "action.terminal_zero_confirmed")
        if collision:
            verdict, reasons = "STOP_COLLISION", ("collision",)
        elif not terminal_zero:
            verdict, reasons = "STOP_TERMINAL_ZERO", ("terminal_zero_not_confirmed",)
        elif not success or action.get("state") != "SUCCEEDED":
            verdict, reasons = "STOP_ROUTE", (str(action.get("stop_reason") or "route_not_succeeded"),)

        critic_reason = str(critic.get("reason", critic.get("fallback_reason", "")))
        online_applied = (
            critic.get("applied") is True
            and "cost_delta_applied=true" in critic_reason
        )
        offline_scores = critic.get("offline_reconstructed_scores", [])
        if online_applied:
            critic_participation = "online_applied"
        elif isinstance(offline_scores, list) and offline_scores:
            critic_participation = "offline_reconstructed"
        else:
            critic_participation = "none"
        result = RunResult(
            run_id=run.run_id,
            repeat=run.repeat,
            arm=run.arm,
            verdict=verdict,
            reasons=reasons,
            synchronized_frames=synchronized,
            scan_invisible_rgbd_pairs=invisible,
            typed_spatial_matches=matches,
            typed_spatial_total=spatial_total,
            path_length_m=path_length(plan),
            local_trajectory_length_m=path_length(optimal_trajectory),
            near_obstacle_speed_mps=float(critic.get("near_obstacle_speed_mps", 0.0)),
            minimum_clearance_m=clearance,
            collision=collision,
            success=success,
            reroute_direction=path_direction(plan),
            critic_participation=critic_participation,
            critic_ttl_status=str(
                freshness.get("critic_ttl_status") or "NOT_APPLICABLE"
            ),
            critic_post_expiry_applied=(
                freshness.get("critic_post_expiry_applied")
                if run.arm == "M3" else None
            ),
            critic_stale_active_probe=str(
                freshness.get("critic_stale_active_probe") or "NOT_APPLICABLE"
            ),
            evidence_file=str(path),
        )
        return result, {"raw": row, "plan": plan, "local_trajectory": optimal_trajectory}
    except (OSError, json.JSONDecodeError, KeyError, TypeError, ValueError, CausalContractError) as exc:
        return RunResult(
            run_id=run.run_id,
            repeat=run.repeat,
            arm=run.arm,
            verdict="INVALID",
            reasons=(str(exc),),
            synchronized_frames=0,
            scan_invisible_rgbd_pairs=0,
            typed_spatial_matches=0,
            typed_spatial_total=0,
            path_length_m=0.0,
            local_trajectory_length_m=0.0,
            near_obstacle_speed_mps=0.0,
            minimum_clearance_m=0.0,
            collision=False,
            success=False,
            reroute_direction="unknown",
            critic_participation="none",
            critic_ttl_status="UNAVAILABLE",
            critic_post_expiry_applied=None,
            critic_stale_active_probe="UNAVAILABLE",
            evidence_file=str(path),
        ), {"raw": {}, "plan": (), "local_trajectory": ()}


def _pair(
    lhs: tuple[RunResult, Mapping[str, Any]],
    rhs: tuple[RunResult, Mapping[str, Any]],
    *,
    trajectory_source: str = "global_plan",
) -> PairResult:
    lhs_result, lhs_data = lhs
    rhs_result, rhs_data = rhs
    key = "local_trajectory" if trajectory_source == "local_trajectory" else "plan"
    lhs_plan = lhs_data[key]
    rhs_plan = rhs_data[key]
    hausdorff = path_hausdorff(lhs_plan, rhs_plan)
    lhs_length = path_length(lhs_plan)
    rhs_length = path_length(rhs_plan)
    denominator = max(lhs_length, 1e-9)
    return PairResult(
        repeat=lhs_result.repeat,
        lhs_arm=lhs_result.arm,
        rhs_arm=rhs_result.arm,
        trajectory_source=trajectory_source,
        hausdorff_m=hausdorff,
        length_delta_fraction=abs(rhs_length - lhs_length) / denominator,
        near_obstacle_speed_delta_mps=(
            rhs_result.near_obstacle_speed_mps - lhs_result.near_obstacle_speed_mps
        ),
        clearance_gain_m=rhs_result.minimum_clearance_m - lhs_result.minimum_clearance_m,
        direction_consistent=(
            lhs_result.reroute_direction == rhs_result.reroute_direction
            and lhs_result.reroute_direction not in {"unknown", "straight"}
        ),
    )


def evaluate(
    manifest: CausalManifest,
    evidence_dir: str | Path,
    *,
    pilot: bool = False,
) -> CausalSummary:
    root = Path(evidence_dir).expanduser().resolve()
    repeats = (1,) if pilot else (1, 2, 3)
    evaluated: dict[tuple[int, str], tuple[RunResult, Mapping[str, Any]]] = {}
    ordered_results: list[RunResult] = []
    for run in selected_runs(manifest, pilot=pilot):
        nested_path = root / run.run_id / f"{run.run_id}.json"
        evidence_path = nested_path if nested_path.is_file() else root / f"{run.run_id}.json"
        result, data = _evaluate_run(manifest, run, evidence_path)
        ordered_results.append(result)
        evaluated[(run.repeat, run.arm)] = (result, data)

    invalid = [result.run_id for result in ordered_results if result.verdict != "VALID"]
    reasons: list[str] = []
    m1_m0: list[PairResult] = []
    m2_m1: list[PairResult] = []
    m3_m1: list[PairResult] = []
    m3_m2: list[PairResult] = []
    if invalid:
        verdict = "INVALID"
        reasons.append("invalid_or_stopped_runs:" + ",".join(invalid))
    else:
        for repeat in repeats:
            m1_m0.append(_pair(evaluated[(repeat, "M0")], evaluated[(repeat, "M1")]))
            m2_m1.append(_pair(evaluated[(repeat, "M1")], evaluated[(repeat, "M2")]))
            m3_m1.append(_pair(evaluated[(repeat, "M1")], evaluated[(repeat, "M3")]))
            m3_m2.append(_pair(
                evaluated[(repeat, "M2")],
                evaluated[(repeat, "M3")],
                trajectory_source="local_trajectory",
            ))

        isolation_ok = all(
            pair.hausdorff_m <= float(manifest.criteria["m1_m0_path_hausdorff_max_m"])
            and pair.length_delta_fraction <= float(manifest.criteria["m1_m0_path_length_delta_max_fraction"])
            for pair in m1_m0
        )
        if not isolation_ok:
            reasons.append("M1_vs_M0_isolation_failed")

        clearance_threshold = float(manifest.criteria["active_clearance_gain_min_m"])
        for arm, pairs in (("M2", m2_m1), ("M3", m3_m1)):
            if statistics.median(pair.clearance_gain_m for pair in pairs) < clearance_threshold:
                reasons.append(f"{arm}_median_clearance_gain_below_threshold")
            active_results = [evaluated[(repeat, arm)][0] for repeat in repeats]
            baselines = [evaluated[(repeat, "M1")][0] for repeat in repeats]
            if any(active.collision and not baseline.collision for active, baseline in zip(active_results, baselines)):
                reasons.append(f"{arm}_new_collision")
            directions = {result.reroute_direction for result in active_results}
            if len(directions) != 1 or directions & {"unknown", "straight"}:
                reasons.append(f"{arm}_reroute_direction_inconsistent")

        m3_results = [evaluated[(repeat, "M3")][0] for repeat in repeats]
        if any(result.critic_participation == "none" for result in m3_results):
            reasons.append("M3_critic_participation_missing")
        if any(
            int(_mapping(evaluated[(repeat, "M3")][1]["raw"].get("critic"), "critic").get(
                "cost_delta_nonzero_count", 0
            )) <= 0
            for repeat in repeats
        ):
            reasons.append("M3_critic_nonzero_cost_delta_missing")
        separation = statistics.median(pair.hausdorff_m for pair in m3_m2)
        separation_min = float(manifest.criteria["m3_m2_trajectory_separation_min_m"])
        no_separation = separation < separation_min
        if no_separation:
            reasons.append("M3_critic_has_no_trajectory_separation")

        hard_fail = any(
            reason != "M3_critic_has_no_trajectory_separation"
            for reason in reasons
        )
        if hard_fail:
            verdict = "FAIL"
        elif no_separation:
            verdict = "AMBIGUOUS"
        else:
            verdict = "PASS_ENGINEERING_PILOT" if pilot else "PASS_ENGINEERING_CAUSAL"

    visualizations: list[Mapping[str, Any]] = []
    for repeat in repeats:
        visualizations.append({
            "repeat": repeat,
            "scene_id": manifest.identity["scene_id"],
            "overlay": "map_costmap_rgbd_scan_typed_obstacles_paths",
            "runs": {
                arm: {
                    "evidence_file": evaluated[(repeat, arm)][0].evidence_file,
                    "path_field": "plan",
                    "local_trajectory_field": "optimal_trajectory",
                    "costmap_fields": ("costmaps.global", "costmaps.local"),
                    "obstacle_field": "synchronized_samples",
                }
                for arm in ("M0", "M1", "M2", "M3")
            },
        })
    return CausalSummary(
        qualification="ENGINEERING_ONLY_NOT_FORMAL",
        verdict=verdict,
        reasons=tuple(reasons),
        runs=tuple(ordered_results),
        m1_vs_m0=tuple(m1_m0),
        m2_vs_m1=tuple(m2_m1),
        m3_vs_m1=tuple(m3_m1),
        m3_vs_m2=tuple(m3_m2),
        visualization_inputs=tuple(visualizations),
    )


def _jsonable(value: Any) -> Any:
    if hasattr(value, "__dataclass_fields__"):
        return {key: _jsonable(item) for key, item in asdict(value).items()}
    if isinstance(value, Mapping):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


def _write_or_print(payload: Any, output: str | None) -> None:
    text = json.dumps(_jsonable(payload), indent=2, sort_keys=True) + "\n"
    if output:
        destination = Path(output).expanduser().resolve()
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(text, encoding="utf-8")
    else:
        print(text, end="")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    manifest_parser = subparsers.add_parser("manifest")
    manifest_parser.add_argument("--config", required=True)
    manifest_parser.add_argument("--output")
    plan_parser = subparsers.add_parser("plan")
    plan_parser.add_argument("--config", required=True)
    plan_parser.add_argument("--output")
    plan_parser.add_argument("--output-root", default="v6r5_module2_causal")
    plan_parser.add_argument("--pilot", action="store_true")
    plan_parser.add_argument("--exact-adapters", action="store_true")
    for option in (
        "scene-adapter", "reset-adapter", "live-adapter", "producer-stop-adapter",
    ):
        plan_parser.add_argument(f"--{option}")
    evaluate_parser = subparsers.add_parser("evaluate")
    evaluate_parser.add_argument("--config", required=True)
    evaluate_parser.add_argument("--evidence-dir", required=True)
    evaluate_parser.add_argument("--output")
    evaluate_parser.add_argument("--pilot", action="store_true")
    run_parser = subparsers.add_parser("run")
    run_parser.add_argument("--config", required=True)
    run_parser.add_argument("--scene-adapter")
    run_parser.add_argument("--reset-adapter", "--stack-adapter", dest="reset_adapter")
    run_parser.add_argument("--live-adapter", "--episode-adapter", dest="live_adapter")
    run_parser.add_argument("--producer-stop-adapter")
    run_parser.add_argument("--exact-adapters", action="store_true")
    run_parser.add_argument("--output-root", required=True)
    run_parser.add_argument("--pilot", action="store_true")
    run_parser.add_argument("--shutdown-timeout-sec", type=float, default=DEFAULT_SHUTDOWN_TIMEOUT_SEC)
    run_parser.add_argument("--output")
    record_parser = subparsers.add_parser("record-evidence")
    record_parser.add_argument("--config", required=True)
    record_parser.add_argument("--run-id", required=True)
    record_parser.add_argument("--bag-dir", required=True)
    record_parser.add_argument("--episode-jsonl", required=True)
    record_parser.add_argument("--output", required=True)
    episode_parser = subparsers.add_parser("dispatch-episode")
    episode_parser.add_argument("--config", required=True)
    episode_parser.add_argument("--run-id", required=True)
    episode_parser.add_argument("--output-jsonl", required=True)
    episode_parser.add_argument("--readiness-timeout-sec", type=float, default=120.0)
    episode_parser.add_argument("--reset-timeout-sec", type=float, default=120.0)
    episode_parser.add_argument("--navigation-timeout-sec", type=float, default=180.0)
    return parser


def cli(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        manifest = load_manifest(args.config)
        if args.command == "manifest":
            _write_or_print({
                "qualification": QUALIFICATION,
                "identity": manifest.identity,
                "localization_contract": manifest.localization_contract,
                "arms": manifest.arms,
                "runs": manifest.runs,
            }, args.output)
            return 0
        if args.command == "plan":
            raw_adapters = (
                args.scene_adapter, args.reset_adapter, args.live_adapter,
                args.producer_stop_adapter,
            )
            supplied = [value is not None for value in raw_adapters]
            if args.exact_adapters and any(supplied):
                raise CausalContractError("--exact-adapters cannot be combined with custom adapters")
            if any(supplied) and not all(supplied):
                raise CausalContractError("plan adapters must be supplied together")
            adapters = (
                exact_adapter_templates(manifest)
                if args.exact_adapters else (
                    AdapterTemplates(*raw_adapters) if all(supplied) else None
                )
            )
            _write_or_print(build_plan(
                manifest,
                adapters=adapters,
                pilot=args.pilot,
                output_root=args.output_root,
            ), args.output)
            return 0
        if args.command == "evaluate":
            summary = evaluate(manifest, args.evidence_dir, pilot=args.pilot)
            _write_or_print(summary, args.output)
            return 0 if summary.verdict in {"PASS_ENGINEERING_CAUSAL", "PASS_ENGINEERING_PILOT"} else 2
        if args.command == "record-evidence":
            candidates = [run for run in manifest.runs if run.run_id == args.run_id]
            if len(candidates) != 1:
                raise CausalContractError(f"unknown run_id: {args.run_id}")
            payload = record_evidence_from_bag(
                manifest, candidates[0], args.bag_dir, args.episode_jsonl, args.output
            )
            print(json.dumps({
                "state": "RECORDED",
                "run_id": args.run_id,
                "output": str(Path(args.output).expanduser().resolve()),
                "synchronized_frames": len(payload["synchronized_samples"]),
            }, sort_keys=True))
            return 0
        if args.command == "dispatch-episode":
            candidates = [run for run in manifest.runs if run.run_id == args.run_id]
            if len(candidates) != 1:
                raise CausalContractError(f"unknown run_id: {args.run_id}")
            result = dispatch_episode(
                manifest,
                candidates[0],
                args.output_jsonl,
                readiness_timeout_sec=args.readiness_timeout_sec,
                reset_timeout_sec=args.reset_timeout_sec,
                navigation_timeout_sec=args.navigation_timeout_sec,
            )
            print(json.dumps(result, sort_keys=True))
            return 0 if result.get("state") == "SUCCEEDED" else 2
        raw_adapters = (
            args.scene_adapter, args.reset_adapter, args.live_adapter,
            args.producer_stop_adapter,
        )
        supplied = [value is not None for value in raw_adapters]
        if args.exact_adapters and any(supplied):
            raise CausalContractError("--exact-adapters cannot be combined with custom adapters")
        if not args.exact_adapters and not all(supplied):
            raise CausalContractError(
                "run requires --exact-adapters or all scene/stack/episode/producer-stop adapters"
            )
        adapters = (
            exact_adapter_templates(manifest)
            if args.exact_adapters else AdapterTemplates(*raw_adapters)
        )
        summary = run_campaign(
            manifest,
            adapters,
            args.output_root,
            pilot=args.pilot,
            shutdown_timeout_sec=args.shutdown_timeout_sec,
        )
        _write_or_print(summary, args.output)
        return 0 if summary["state"] == "FINISHED" else 2
    except (OSError, yaml.YAMLError, CausalContractError) as exc:
        print(f"INVALID: {exc}", file=sys.stderr)
        return 2


def main() -> None:
    raise SystemExit(cli())


if __name__ == "__main__":
    main()
