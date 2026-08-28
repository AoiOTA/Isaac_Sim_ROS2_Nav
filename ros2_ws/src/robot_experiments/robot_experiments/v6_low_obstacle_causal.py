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
import socket
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
    "/clock",
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
    "/cmd_vel_nav",
    "/cmd_vel_smoothed",
    "/cmd_vel_sim",
    "/collision_monitor_state",
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
DEFAULT_CLEANUP_CONFIRM_TIMEOUT_SEC = 5.0
DEFAULT_CLEANUP_QUIET_SEC = 0.25
DEFAULT_CLEANUP_POLL_SEC = 0.05
DEFAULT_COGNITIVE_READY_TIMEOUT_SEC = 120.0
NOMINAL_TTL_STATUS = "N/A_SEPARATE_ACTIVE_CONTROLLER_PROBE"
VALIDATION_STATIC_DEPTH_REVALIDATED = 2
VALIDATION_SENSOR_DEPTH = 1
MOTION_STATIC = 1
SHADOW_REJECTION_UNTRUSTED = 4
PHYSICAL_DEPTH_FOOTPRINT_SOURCE = "physical_low_box_aabb_depth_hits"
PHASE_F_QOS_CONFIG = "v6_low_obstacle_phase_f_rosbag_qos.yaml"
PHASE_F_RECORDED_CHILDREN = (
    "module3_ros",
    "module2_server",
    "integration_bridge",
)
MODULE3_RESOURCE_PREFIX = "module3://"
MODULE3_ROOT_ENV = "BIO_NAV_MODULE3_ROOT"
KUJIALE_MAP_ID = "v6_kujiale_isaacgen_v1"
KUJIALE_T_MAP_CANVAS = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
KUJIALE_VALID_STATE_IDS = (
    39, 40, 56, 69, 70, 72, 84, 85, 86, 87, 88, 101, 102, 103, 104,
    115, 116, 117, 118, 119, 120, 121, 122, 133, 134, 135, 136, 137,
    147, 148, 149, 150, 151, 153, 165, 166, 167, 169, 180, 181, 182,
    183, 184, 185, 199, 200, 201, 202, 215, 216, 217,
)

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
    module2_asset_root: str | None = None


@dataclass(frozen=True)
class RecordedMessage:
    topic: str
    stamp_ns: int
    message: Any


def exact_adapter_templates(
    manifest: CausalManifest,
    module2_asset_root: str | Path,
) -> AdapterTemplates:
    root = manifest.module3_root
    if root is None:
        raise CausalContractError(
            f"exact adapters require {MODULE3_ROOT_ENV} when using an installed manifest"
        )
    asset_root = str(module2_asset_root)
    if not asset_root:
        raise CausalContractError("module2 asset root must not be empty")
    return AdapterTemplates(
        scene=(
            f"{root}/scripts/run_v6_r5_phase_b_kujiale.sh "
            "--domain {ros_domain_id} isaac --dynamic-obstacle-config "
            "{obstacle_config} --dynamic-obstacles"
        ),
        stack=(
            f"{root}/scripts/run_v6_low_obstacle_phase_f_stack.sh "
            "{arm} --domain {ros_domain_id} --run-dir {run_dir} "
            "--socket {module2_socket} {module2_asset_root_arg}"
        ),
        episode=(
            f"{root}/scripts/run_v6_low_obstacle_causal.sh dispatch-episode "
            "--run-id {run_id} --output-jsonl {episode_jsonl}"
        ),
        producer_stop=(
            f"{root}/scripts/run_v6_low_obstacle_phase_f_stack.sh "
            "stop-producer --run-dir {run_dir} --socket {module2_socket}"
        ),
        module2_asset_root=asset_root,
    )


@dataclass(frozen=True)
class RunResult:
    run_id: str
    repeat: int
    arm: str
    verdict: str
    reasons: tuple[str, ...]
    synchronized_frames: int | None
    scan_invisible_rgbd_pairs: int | None
    typed_spatial_matches: int | None
    typed_spatial_total: int | None
    source_visible_count: int | None
    source_matched_count: int | None
    source_recall: float | None
    candidate_true_positive_count: int | None
    candidate_false_positive_count: int | None
    candidate_precision: float | None
    candidate_radius_max_m: float | None
    best_center_errors_m: tuple[float, ...] | None
    path_length_m: float | None
    local_trajectory_length_m: float | None
    near_obstacle_speed_mps: float | None
    minimum_clearance_m: float | None
    collision: bool | None
    success: bool | None
    action_state: str | None
    terminal_zero_confirmed: bool | None
    reroute_direction: str
    critic_participation: str
    critic_applied: bool | None
    critic_status_count: int | None
    critic_applied_count: int | None
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
    diagnostic_when_invalid: bool
    hausdorff_m: float
    length_delta_fraction: float
    near_obstacle_speed_delta_mps: float
    clearance_gain_m: float
    direction_consistent: bool


@dataclass(frozen=True)
class CausalSummary:
    qualification: str
    formal_qualification: bool
    phase_f_complete: bool
    verdict: str
    reasons: tuple[str, ...]
    selected_arm: str | None
    selected_arm_active_ttl_status: str
    selection_outcome: str
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


def _known_bool(value: Any) -> bool | None:
    return value if isinstance(value, bool) else None


def _known_int(value: Any) -> int | None:
    return value if isinstance(value, int) and not isinstance(value, bool) else None


def _known_float(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    result = float(value)
    return result if math.isfinite(result) else None


def _known_mapping(value: Any) -> Mapping[str, Any]:
    return value if isinstance(value, Mapping) else {}


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
        "route_backend": "gvg",
        "route_prior_enabled": False,
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
    depth_xy_tolerance = float(criteria.get("depth_obstacle_bounds_tolerance_m", 0.02))
    if not 0.0 <= depth_xy_tolerance <= 0.02:
        raise CausalContractError(
            "criteria.depth_obstacle_bounds_tolerance_m must be within [0, 0.02]"
        )
    if float(criteria.get("depth_min_height_above_floor_m", 0.0)) < 0.02:
        raise CausalContractError(
            "criteria.depth_min_height_above_floor_m must be at least 0.02"
        )
    for key in ("source_recall_min", "candidate_precision_min"):
        value = float(criteria.get(key, -1.0))
        if not 0.0 <= value <= 1.0:
            raise CausalContractError(f"criteria.{key} must be within [0, 1]")
    if float(criteria.get("candidate_radius_max_m", 0.0)) <= 0.0:
        raise CausalContractError("criteria.candidate_radius_max_m must be positive")
    if criteria.get("selection_policy") != "simplest_valid_arm_with_observed_net_benefit":
        raise CausalContractError(
            "criteria.selection_policy must select the simplest valid arm with observed net benefit"
        )
    for key in (
        "m1_m0_path_similarity_diagnostic_only",
        "active_clearance_gain_diagnostic_only",
        "m3_m2_trajectory_separation_diagnostic_only",
        "selected_arm_active_ttl_required",
    ):
        if criteria.get(key) is not True:
            raise CausalContractError(f"criteria.{key} must be true")
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


def _finite_flat_floats(
    value: Any,
    *,
    minimum_length: int | None = None,
    exact_length: int | None = None,
    minimum: float | None = None,
    maximum: float | None = None,
) -> tuple[float, ...] | None:
    """Flatten list/array-like numeric fields without depending on NumPy."""

    if value is None or isinstance(value, (str, bytes, bytearray, Mapping)):
        return None
    if hasattr(value, "tolist"):
        try:
            value = value.tolist()
        except (TypeError, ValueError):
            return None

    flattened: list[float] = []

    def append(raw: Any) -> bool:
        if isinstance(raw, (str, bytes, bytearray, Mapping)):
            return False
        if isinstance(raw, Iterable):
            try:
                items = list(raw)
            except (TypeError, ValueError):
                return False
            return all(append(item) for item in items)
        try:
            number = float(raw)
        except (TypeError, ValueError):
            return False
        if not math.isfinite(number):
            return False
        if minimum is not None and number < minimum:
            return False
        if maximum is not None and number > maximum:
            return False
        flattened.append(number)
        return True

    if not append(value):
        return None
    if exact_length is not None and len(flattened) != exact_length:
        return None
    if minimum_length is not None and len(flattened) < minimum_length:
        return None
    return tuple(flattened)


def _finite_float(
    value: Any,
    *,
    minimum: float | None = None,
    maximum: float | None = None,
) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(result):
        return None
    if minimum is not None and result < minimum:
        return None
    if maximum is not None and result > maximum:
        return None
    return result


def _typed_obstacles(
    message: Any,
    *,
    tf_records: Sequence[RecordedMessage] = (),
    tf_static_records: Sequence[RecordedMessage] = (),
    target_frame: str | None = None,
) -> list[dict[str, Any]]:
    accepted = bool(
        _field(message, "input_healthy", True)
        and _field(message, "module2_healthy", False)
        and _field(message, "observation_valid", False)
    )
    trusted_write = bool(_field(message, "trusted_write", False))
    try:
        validation_mode = int(_field(message, "validation_mode", 0))
        validation_sensor_mask = int(_field(message, "validation_sensor_mask", 0))
        rejection_mask = int(_field(message, "rejection_mask", 0))
    except (TypeError, ValueError, OverflowError):
        return []
    transform: RigidTransform | None = None
    if target_frame is not None:
        source_frame = _frame(_field(message, "header.frame_id"))
        requested_frame = _frame(target_frame)
        if not source_frame or not requested_frame:
            return []
        if source_frame != requested_frame:
            validation_stamp_ns = _time_ns(_field(message, "validation_stamp"))
            if validation_stamp_ns is None or validation_stamp_ns <= 0:
                return []
            transform = _lookup_recorded_transform(
                tf_records,
                tf_static_records,
                target_frame=requested_frame,
                source_frame=source_frame,
                stamp_ns=validation_stamp_ns,
            )
            if transform is None:
                return []
    result: list[dict[str, Any]] = []
    for obstacle in _field(message, "obstacles", ()) or ():
        pose = _finite_flat_floats(_field(obstacle, "pose_xy_m"), minimum_length=2)
        radius = _finite_float(_field(obstacle, "radius_m"), minimum=0.0)
        confidence = _finite_float(_field(obstacle, "confidence"), minimum=0.0, maximum=1.0)
        obstacle_id = str(_field(obstacle, "id", ""))
        if pose is None or radius is None or radius <= 0.0 or confidence is None or not obstacle_id:
            continue

        # Fixed-size ROS arrays decode as NumPy arrays in rosbag2_py on Jazzy.
        # Validate optional message-contract fields when present, while retaining
        # compatibility with the small dictionary fixtures used by offline tests.
        position_stddev = _field(obstacle, "position_stddev_m")
        if position_stddev is not None and _finite_flat_floats(
            position_stddev, exact_length=2, minimum=0.0
        ) is None:
            continue
        for field_name in ("height_m",):
            field_value = _field(obstacle, field_name)
            if field_value is not None and _finite_float(field_value, minimum=0.0) is None:
                break
        else:
            for field_name in ("reliability", "ood_probability"):
                field_value = _field(obstacle, field_name)
                if field_value is not None and _finite_float(
                    field_value, minimum=0.0, maximum=1.0
                ) is None:
                    break
            else:
                count = _field(obstacle, "count")
                if count is not None:
                    try:
                        if int(count) <= 0:
                            continue
                    except (TypeError, ValueError, OverflowError):
                        continue
                class_id = _field(obstacle, "class_id")
                if class_id is not None and str(class_id) != "unknown_low_obstacle":
                    continue
                try:
                    motion_class = int(_field(obstacle, "motion_class", 0))
                except (TypeError, ValueError, OverflowError):
                    continue
                static_confirmed = bool(_field(obstacle, "static_confirmed", False))
                x, y = pose[0], pose[1]
                if transform is not None:
                    x, y, _ = _apply_transform(transform, (x, y, 0.0))
                    if not math.isfinite(x) or not math.isfinite(y):
                        continue
                result.append({
                    "id": obstacle_id,
                    "x": x,
                    "y": y,
                    "radius_m": radius,
                    "confidence": confidence,
                    "accepted": accepted,
                    "trusted_write": trusted_write,
                    "validation_mode": validation_mode,
                    "validation_sensor_mask": validation_sensor_mask,
                    "rejection_mask": rejection_mask,
                    "motion_class": motion_class,
                    "static_confirmed": static_confirmed,
                })
    return result


def _m1_shadow_candidate_summary(
    records: Sequence[RecordedMessage],
) -> dict[str, int]:
    """Summarize typed shadow candidates without treating them as writes."""

    trusted_write_count = 0
    shadow_rejection_count = 0
    nonempty_message_count = 0
    static_depth_revalidated_geometry_count = 0
    invalid_geometry_count = 0
    for row in records:
        trusted_write = bool(_field(row.message, "trusted_write", False))
        try:
            rejection_mask = int(_field(row.message, "rejection_mask", -1))
        except (TypeError, ValueError, OverflowError):
            rejection_mask = -1
        trusted_write_count += int(trusted_write)
        obstacles = _typed_obstacles(row.message)
        nonempty_message_count += int(bool(obstacles))
        # A producer may emit one empty startup message before geometry is
        # available.  Shadow semantics apply to messages carrying candidates;
        # the empty startup message is neither a rejection nor invalid geometry.
        if obstacles:
            shadow_rejection_count += int(
                not trusted_write and rejection_mask == SHADOW_REJECTION_UNTRUSTED
            )
        for obstacle in obstacles:
            static_depth_revalidated = (
                obstacle["validation_mode"] == VALIDATION_STATIC_DEPTH_REVALIDATED
                and obstacle["validation_sensor_mask"] & VALIDATION_SENSOR_DEPTH
                and obstacle["motion_class"] == MOTION_STATIC
                and obstacle["static_confirmed"] is True
            )
            static_depth_revalidated_geometry_count += int(static_depth_revalidated)
            invalid_geometry_count += int(not static_depth_revalidated)
    return {
        "message_count": len(records),
        "nonempty_message_count": nonempty_message_count,
        "static_depth_revalidated_geometry_count": (
            static_depth_revalidated_geometry_count
        ),
        "trusted_write_count": trusted_write_count,
        "shadow_rejection_count": shadow_rejection_count,
        "invalid_geometry_count": invalid_geometry_count,
    }


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
    xy_tolerance = float(criteria.get("depth_obstacle_bounds_tolerance_m", 0.02))
    z_tolerance = float(criteria.get("depth_low_z_tolerance_m", 0.05))
    minimum_height_above_floor = float(
        criteria.get("depth_min_height_above_floor_m", 0.02)
    )
    center = obstacle["center"]
    half_x = 0.5 * float(obstacle["size"][0]) + xy_tolerance
    half_y = 0.5 * float(obstacle["size"][1]) + xy_tolerance
    lower_z = float(obstacle["z_bounds"][0]) + minimum_height_above_floor
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
    physical_half_x = 0.5 * float(obstacle["size"][0])
    physical_half_y = 0.5 * float(obstacle["size"][1])
    footprint = {
        "id": obstacle["id"],
        "center": [
            float(center[0]),
            float(center[1]),
            0.5 * (float(obstacle["z_bounds"][0]) + float(obstacle["z_bounds"][1])),
        ],
        "size": [float(value) for value in obstacle["size"]],
        "rectangle": [
            float(center[0]) - physical_half_x,
            float(center[1]) - physical_half_y,
            float(center[0]) + physical_half_x,
            float(center[1]) + physical_half_y,
        ],
        "source": PHYSICAL_DEPTH_FOOTPRINT_SOURCE,
        "point_count": len(hits),
        "hit_count": len(hits),
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


def _minimum_dynamic_clearance(
    gt_records: Sequence[RecordedMessage],
    obstacle_at_stamp: Callable[[int], Mapping[str, Any] | None],
) -> float:
    clearances: list[float] = []
    for record in gt_records:
        point = _odom_point(record.message)
        obstacle = obstacle_at_stamp(_message_stamp_ns(record))
        if point is None or obstacle is None:
            continue
        center = obstacle["center"]
        half_x = 0.5 * float(obstacle["size"][0])
        half_y = 0.5 * float(obstacle["size"][1])
        radius = float(obstacle["robot_radius_m"])
        dx = max(abs(point[0] - center[0]) - half_x, 0.0)
        dy = max(abs(point[1] - center[1]) - half_y, 0.0)
        clearances.append(max(0.0, math.hypot(dx, dy) - radius))
    if not clearances:
        raise CausalContractError("dynamic obstacle/GT overlap is missing")
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


def _near_dynamic_obstacle_speed(
    command_records: Sequence[RecordedMessage],
    gt_records: Sequence[RecordedMessage],
    obstacle_at_stamp: Callable[[int], Mapping[str, Any] | None],
) -> float:
    speeds: list[float] = []
    for command in command_records:
        stamp_ns = _message_stamp_ns(command)
        pose = _nearest(gt_records, stamp_ns)
        obstacle = obstacle_at_stamp(stamp_ns)
        point = _odom_point(pose.message) if pose is not None else None
        if point is None or obstacle is None:
            continue
        if math.dist(point, obstacle["center"]) <= 1.5:
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
    *,
    physical_obstacle: Mapping[str, Any] | None = None,
    obstacle_at_stamp: Callable[[int], Mapping[str, Any] | None] | None = None,
    dynamic_actors_enabled: bool = False,
) -> dict[str, Any]:
    """Reduce recorded ROS messages into the causal evaluator JSON.

    ``obstacle_at_stamp`` is the only dynamic specialization: callers may
    provide the physical actor AABB for each synchronized stamp.  The Phase-F
    static path keeps using its frozen obstacle unchanged.
    """

    reset_receipt = episode_result.get("reset_receipt")
    receipt_generation = (
        reset_receipt.get("generation")
        if isinstance(reset_receipt, Mapping)
        else None
    )
    if (
        not isinstance(receipt_generation, int)
        or isinstance(receipt_generation, bool)
        or receipt_generation <= 0
    ):
        raise CausalContractError(
            "reset_receipt.generation must be a positive integer"
        )
    if "target_reset_epoch" not in episode_result:
        # The exactly-once episode reset receipt reports the actual generation
        # stamped by cognitive producers for this episode.
        target_reset_epoch = receipt_generation
    else:
        target_reset_epoch = episode_result["target_reset_epoch"]
        if (
            not isinstance(target_reset_epoch, int)
            or isinstance(target_reset_epoch, bool)
            or target_reset_epoch <= 0
        ):
            raise CausalContractError(
                "target_reset_epoch must be a positive integer"
            )
        # Legacy static artifacts can record the producer epoch immediately
        # after the reset receipt; no wider cross-episode override is valid.
        if target_reset_epoch not in (receipt_generation, receipt_generation + 1):
            raise CausalContractError(
                "target_reset_epoch must equal reset_receipt.generation "
                "or reset_receipt.generation + 1"
            )
    evidence_window = episode_result.get("_evidence_window", {})
    window_start_ns = (
        int(evidence_window["start_ns"])
        if isinstance(evidence_window, Mapping)
        and isinstance(evidence_window.get("start_ns"), int)
        and not isinstance(evidence_window.get("start_ns"), bool)
        else None
    )
    window_end_ns = (
        int(evidence_window["end_ns"])
        if isinstance(evidence_window, Mapping)
        and isinstance(evidence_window.get("end_ns"), int)
        and not isinstance(evidence_window.get("end_ns"), bool)
        else None
    )

    def in_episode(record: RecordedMessage) -> bool:
        if window_start_ns is not None and record.stamp_ns < window_start_ns:
            return False
        if window_end_ns is not None and record.stamp_ns > window_end_ns:
            return False
        message_epoch = _field(record.message, "reset_epoch")
        if message_epoch is not None and target_reset_epoch is not None:
            try:
                return int(message_epoch) == target_reset_epoch
            except (TypeError, ValueError, OverflowError):
                return False
        return True

    by_topic: dict[str, list[RecordedMessage]] = {}
    excluded_record_count = 0
    for record in records:
        # Static transforms are timeless context for proving a transform chain;
        # they are not counted as episode observations.
        if record.topic != "/tf_static" and not in_episode(record):
            excluded_record_count += 1
            continue
        by_topic.setdefault(record.topic, []).append(record)
    for values in by_topic.values():
        values.sort(key=_message_stamp_ns)

    obstacle = dict(physical_obstacle or _load_frozen_obstacle(manifest))
    typed_records = by_topic.get("/bio_nav/module2/cognitive_obstacles", [])
    scan_records = by_topic.get("/scan", [])
    depth_records = by_topic.get("/camera/front/depth/image_raw", [])
    camera_info_records = by_topic.get("/camera/front/camera_info", [])
    tf_records = by_topic.get("/tf", [])
    tf_static_records = by_topic.get("/tf_static", [])
    gt_records = by_topic.get("/ground_truth/odom", [])
    planning_prior_records = [
        row
        for topic in ("/bio_nav/module2/planning_prior", "/bio_nav/module2/goal_planning_prior")
        for row in by_topic.get(topic, [])
    ]
    planning_prior_records.sort(key=_message_stamp_ns)
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
        anchor_is_typed = anchor.topic == "/bio_nav/module2/cognitive_obstacles"
        validation_stamp = _time_ns(_field(anchor.message, "validation_stamp"))
        stamp = (
            validation_stamp
            if anchor_is_typed and validation_stamp is not None and validation_stamp > 0
            else _message_stamp_ns(anchor)
        )
        scan = _nearest(scan_records, stamp)
        depth = _nearest(depth_records, stamp)
        camera_info = _nearest(camera_info_records, _message_stamp_ns(depth)) if depth is not None else None
        typed = anchor if anchor_is_typed else _nearest(typed_records, stamp)
        pose = _nearest(gt_records, stamp)
        depth_synchronized = depth is not None and abs(_message_stamp_ns(depth) - stamp) <= DEFAULT_SYNC_TOLERANCE_NS
        scan_synchronized = scan is not None and abs(_message_stamp_ns(scan) - stamp) <= DEFAULT_SYNC_TOLERANCE_NS
        if not depth_synchronized:
            depth = None
            camera_info = None
        if not scan_synchronized:
            scan = None
        stamped_obstacle = (
            obstacle_at_stamp(stamp) if obstacle_at_stamp is not None else obstacle
        )
        if stamped_obstacle is None:
            scan_points, _ = _scan_metrics(scan, pose, obstacle)
            scan_hits = 0
            depth_observation = {
                "valid": False,
                "reason": "physical_actor_not_visible",
                "point_count": 0,
                "hit_count": 0,
                "footprints": [],
            }
        else:
            stamped_obstacle = dict(stamped_obstacle)
            scan_points, scan_hits = _scan_metrics(scan, pose, stamped_obstacle)
            depth_observation = _project_depth_obstacle(
                depth,
                camera_info,
                tf_records,
                tf_static_records,
                stamped_obstacle,
                manifest.criteria,
            )
        typed_values = []
        if typed is not None and (
            anchor_is_typed
            or abs(_message_stamp_ns(typed) - stamp) <= DEFAULT_SYNC_TOLERANCE_NS
        ):
            typed_values = _typed_obstacles(
                typed.message,
                tf_records=tf_records,
                tf_static_records=tf_static_records,
                target_frame="map",
            )
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
            "physical_obstacle": stamped_obstacle,
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
        observed_critic_ttl_status = None
        ttl_expiry_critic_not_applied = None
    elif not expired_critics:
        observed_critic_ttl_status = "N/A_NO_CONTROLLER_SCORING"
        ttl_expiry_critic_not_applied = None
    elif len(clear_critics) == len(expired_critics):
        observed_critic_ttl_status = "STALE_REJECTED"
        ttl_expiry_critic_not_applied = True
    elif critic_post_expiry_applied:
        observed_critic_ttl_status = "FAIL_POST_EXPIRY_APPLIED"
        ttl_expiry_critic_not_applied = False
    else:
        observed_critic_ttl_status = "FAIL_NOT_STALE_REJECTED"
        ttl_expiry_critic_not_applied = False
    # After a nominal route terminates, costmap/controller callbacks are not
    # guaranteed.  Preserve the drain observations as diagnostics only; the
    # separate active-controller probe owns the Phase-F TTL decision.
    ttl_expiry_observed = bool(ttl_expiry_zero_write) if active_ttl else None

    plans = [_path_points(row.message) for row in by_topic.get("/plan", [])]
    local_trajectory_records = by_topic.get("/optimal_trajectory", [])
    local_trajectories = [_path_points(row.message) for row in local_trajectory_records]
    plan = max((item for item in plans if item), key=len, default=[])
    if obstacle_at_stamp is None:
        optimal = _near_obstacle_trajectory(local_trajectory_records, gt_records, obstacle)
    else:
        dynamic_trajectories: list[tuple[float, list[list[float]]]] = []
        for record in local_trajectory_records:
            path = _path_points(record.message)
            pose = _nearest(gt_records, _message_stamp_ns(record))
            actor = obstacle_at_stamp(_message_stamp_ns(record))
            point = _odom_point(pose.message) if pose is not None else None
            if path and point is not None and actor is not None:
                dynamic_trajectories.append(
                    (math.dist(point, actor["center"]), path)
                )
        optimal = min(dynamic_trajectories, key=lambda item: item[0])[1] \
            if dynamic_trajectories else []
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

    obstacle_messages = [
        _typed_obstacles(
            row.message,
            tf_records=tf_records,
            tf_static_records=tf_static_records,
            target_frame="map",
        )
        for row in typed_records
    ]
    obstacle_validation = [item for values in obstacle_messages for item in values if item["accepted"]]
    shadow_candidate = _m1_shadow_candidate_summary(typed_records)
    health_records = planning_prior_records if run.arm == "M1" else typed_records
    health_stamps = [_message_stamp_ns(row) for row in health_records]
    health_cadence_hz = 0.0
    if len(health_stamps) >= 2 and health_stamps[-1] > health_stamps[0]:
        health_cadence_hz = (
            (len(health_stamps) - 1) * 1.0e9 / (health_stamps[-1] - health_stamps[0])
        )
    module2_health = {
        "message_count": len(health_records),
        "healthy_count": sum(
            bool(_field(row.message, "module2_healthy", False))
            and bool(_field(row.message, "observation_valid", False))
            for row in health_records
        ),
        "trusted_write_count": sum(
            bool(_field(row.message, "trusted_write", False)) for row in health_records
        ),
        "observation_valid_count": sum(
            bool(_field(row.message, "observation_valid", False)) for row in health_records
        ),
        "candidate_cadence_hz": health_cadence_hz,
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
        "module2_uds_connected": bool(health_records) if run.arm != "M0" else False,
        "module2_health": module2_health,
        "isolation": {
            "module1_amcl_prior_enabled": False,
            "cognitive_place_graph_enabled": False,
            "dynamic_actors_enabled": dynamic_actors_enabled,
            "unexpected_topic_counts": isolation_counts,
        },
        "reset": {
            "calls": int(episode_result.get("reset_calls", 0)),
            "events": int(episode_result.get("reset_events", 0)),
            "goal_publications": int(episode_result.get("goal_publications", 0)),
            "localization_contract": "same_estimated_autonomy",
            "target_reset_epoch": target_reset_epoch,
            "evidence_window_start_ns": window_start_ns,
            "evidence_window_end_ns": window_end_ns,
            "excluded_record_count": excluded_record_count,
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
                NOMINAL_TTL_STATUS if active_ttl else "not_applicable_inactive"
            ),
            "external_active_controller_probe_required": active_ttl,
            "external_active_controller_probe_status": (
                "NOT_EVALUATED_BY_NOMINAL_RUN" if active_ttl else None
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
            "critic_ttl_status": NOMINAL_TTL_STATUS if run.arm == "M3" else None,
            "nominal_post_route_critic_observation": observed_critic_ttl_status,
            "critic_post_expiry_applied": critic_post_expiry_applied,
            "critic_stale_active_probe": "NOT_RUN" if run.arm == "M3" else None,
        },
        "sensor_counts": {
            "scan_message_count": len(scan_records),
            "depth_message_count": len(depth_records),
            "camera_info_message_count": len(camera_info_records),
        },
        "synchronized_samples": samples,
        "obstacle_validation": obstacle_validation,
        "shadow_obstacle_candidate": shadow_candidate,
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
            "near_obstacle_speed_mps": (
                _near_obstacle_speed(command_records, gt_records, obstacle)
                if obstacle_at_stamp is None
                else _near_dynamic_obstacle_speed(
                    command_records, gt_records, obstacle_at_stamp
                )
            ),
            "offline_reconstructed_scores": [],
        },
        "planning_prior": [{
            "stamp_ns": _message_stamp_ns(row),
            "module2_healthy": bool(_field(row.message, "module2_healthy", False)),
            "observation_valid": bool(_field(row.message, "observation_valid", False)),
            "trusted_write": bool(_field(row.message, "trusted_write", False)),
        } for row in planning_prior_records],
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
            "minimum_clearance_m": (
                _minimum_clearance(gt, obstacle)
                if obstacle_at_stamp is None
                else _minimum_dynamic_clearance(gt_records, obstacle_at_stamp)
            ),
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
            "near_obstacle_speed_mps": (
                _near_obstacle_speed(command_records, gt_records, obstacle)
                if obstacle_at_stamp is None
                else _near_dynamic_obstacle_speed(
                    command_records, gt_records, obstacle_at_stamp
                )
            ),
            "dynamic_risk_exposure": (
                "recorded_actor_aabb_per_stamp"
                if dynamic_actors_enabled else "not_applicable_dynamic_actors_off"
            ),
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

    _require_phase_f_route_identity(manifest)
    rows = []
    root = Path(output_root).expanduser().resolve()
    for run in selected_runs(manifest, pilot=pilot):
        arm = manifest.arms[run.arm]
        values = _adapter_values(manifest, run, root)
        values["module2_asset_root_arg"] = (
            "--module2-asset-root " + shlex.quote(adapters.module2_asset_root)
            if adapters is not None
            and adapters.module2_asset_root is not None
            and run.arm != "M0"
            else ""
        )
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
                "route_backend": manifest.identity["route_backend"],
                "route_prior_enabled": manifest.identity["route_prior_enabled"],
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
    integration_cli_contract: dict[str, str] = {
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
    }
    recommended_stack: str | dict[str, str] = (
        "{module3_root}/scripts/run_v6_low_obstacle_phase_f_stack.sh "
        "{arm} --domain {ros_domain_id} --run-dir {run_dir} "
        "--socket {module2_socket}"
    )
    if adapters is not None and adapters.module2_asset_root is not None:
        asset_option = (
            "--module2-asset-root " + shlex.quote(adapters.module2_asset_root)
        )
        active_server = (
            "scripts/run_v6_module2_causal_obstacle_server.sh "
            "--startup-profile module2_causal_obstacle_active "
            "--active-effect-scope obstacle_only --socket {module2_socket} "
            "--module2-root <MODULE2_ROOT> " + asset_option + " --shadow-config "
            "configs/kujiale_0026_module1_visual_shadow_v310.yaml"
        )
        integration_cli_contract = {
            **integration_cli_contract,
            "M1": (
                "scripts/run_module2_v310_server.sh "
                "--module2-root <MODULE2_ROOT> " + asset_option + " --shadow-config "
                "configs/kujiale_0026_module1_visual_shadow_v310.yaml "
                "--socket {module2_socket}"
            ),
            "M2": active_server,
            "M3": active_server,
        }
        integration_cli_contract.pop("M2_M3_server")
        stack_prefix = (
            "{module3_root}/scripts/run_v6_low_obstacle_phase_f_stack.sh "
        )
        stack_suffix = (
            " --domain {ros_domain_id} --run-dir {run_dir} "
            "--socket {module2_socket}"
        )
        recommended_stack = {
            "M0": stack_prefix + "M0" + stack_suffix,
            **{
                arm_name: (
                    stack_prefix + arm_name + stack_suffix + " " + asset_option
                )
                for arm_name in ("M1", "M2", "M3")
            },
        }
    return {
        "qualification": QUALIFICATION,
        "mode": "pilot" if pilot else "formal_12",
        "dispatch": adapters is not None,
        "reason": None if adapters is not None else "external_scene_stack_episode_adapters_required",
        "exactly_once_reset_contract": "reuse_v6_formal_episode_guard",
        "integration_cli_contract": integration_cli_contract,
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
            "stack": recommended_stack,
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
    latest_clock_topics: Sequence[str] = (),
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
    clock_stamped = set(latest_clock_topics)
    decoded = selected | ({"/clock"} if clock_stamped else set())
    message_types = {
        topic: get_message(type_name)
        for topic, type_name in type_by_topic.items()
        if topic in decoded
    }

    def deserialized_records() -> Iterable[RecordedMessage]:
        while reader.has_next():
            topic, data, stamp_ns = reader.read_next()
            message_type = message_types.get(topic)
            if message_type is None:
                continue
            yield RecordedMessage(
                topic, int(stamp_ns), deserialize_message(data, message_type)
            )

    for record in _latest_clock_stamped_records(
        deserialized_records(), clock_stamped
    ):
        if record.topic in selected:
            yield record


def _latest_clock_stamped_records(
    records: Iterable[RecordedMessage],
    topics: Sequence[str] | set[str],
) -> Iterable[RecordedMessage]:
    """Put selected header-less records on the bag's ordered simulation clock."""

    selected = set(topics)
    latest_clock_ns: int | None = None
    for record in records:
        if record.topic == "/clock":
            clock_ns = _time_ns(_field(record.message, "clock"))
            if clock_ns is not None and clock_ns >= 0:
                latest_clock_ns = clock_ns
            yield record
            continue
        if record.topic in selected:
            if latest_clock_ns is None:
                continue
            yield RecordedMessage(record.topic, latest_clock_ns, record.message)
            continue
        yield record


def _episode_result_from_jsonl(path: Path) -> Mapping[str, Any]:
    result: dict[str, Any] | None = None
    reset_receipt_ns: int | None = None
    terminal_ns: int | None = None
    result_ns: int | None = None
    for line in path.read_text(encoding="utf-8").splitlines():
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            continue
        if not isinstance(row, Mapping):
            continue
        event = row.get("event")
        wall_time_ns = row.get("wall_time_ns")
        valid_wall_time = (
            isinstance(wall_time_ns, int)
            and not isinstance(wall_time_ns, bool)
            and wall_time_ns > 0
        )
        if event == "reset_receipt" and valid_wall_time:
            reset_receipt_ns = int(wall_time_ns)
        elif event == "terminal_zero_confirmed" and valid_wall_time:
            terminal_ns = int(wall_time_ns)
        elif event == "episode_result":
            result = dict(row)
            if valid_wall_time:
                result_ns = int(wall_time_ns)
    if result is None:
        raise CausalContractError(f"episode_result event missing from {path}")
    if reset_receipt_ns is None or result_ns is None:
        raise CausalContractError(
            f"reset_receipt/result evidence window missing from {path}"
        )
    end_ns = terminal_ns if terminal_ns is not None else result_ns
    if end_ns < reset_receipt_ns:
        raise CausalContractError(f"episode evidence window is reversed in {path}")
    result["_evidence_window"] = {
        "start_ns": reset_receipt_ns,
        "end_ns": end_ns,
        "end_event": "terminal_zero_confirmed" if terminal_ns is not None else "episode_result",
    }
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
        DynamicScheduleEntry,
        ENGINEERING_PILOT,
        Episode,
        Manifest,
        MissionLeg,
        V6FormalNode,
    )

    dynamic_actors_enabled = manifest.identity.get("dynamic_actors_enabled") is True
    dynamic_group = str(manifest.identity.get("trigger_group", ""))
    dynamic_case_id = str(manifest.identity.get("dynamic_case_id", "static"))
    dynamic_variant_id = str(
        manifest.identity.get("dynamic_variant_id", "v6_phase_f_r2")
    )
    if dynamic_actors_enabled and not dynamic_group:
        raise CausalContractError("dynamic experiment requires identity.trigger_group")
    runtime = _phase_f_runtime(
        manifest,
        run,
        dynamic_actors_enabled=dynamic_actors_enabled,
    )
    formal_manifest = Manifest(
        path=manifest.path,
        raw={},
        scene_id="kujiale_0026_A_to_B_door_open",
        category="dynamic" if dynamic_actors_enabled else "static",
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
        dynamic_schedule=(
            (DynamicScheduleEntry("G2", dynamic_group),)
            if dynamic_actors_enabled else ()
        ),
        episodes=(),
    )
    episode = Episode(
        seed=int(manifest.identity["seed"]),
        variant_id=dynamic_variant_id,
        appearance_profile_id=None,
        reset_pose_name="long_route_start_g1",
        dynamic_case_id=dynamic_case_id,
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


def _require_phase_f_route_identity(manifest: CausalManifest) -> None:
    if manifest.identity.get("route_backend") != "gvg":
        raise CausalContractError("identity.route_backend must be 'gvg'")
    if manifest.identity.get("route_prior_enabled") is not False:
        raise CausalContractError("identity.route_prior_enabled must be False")


def _phase_f_runtime(
    manifest: CausalManifest,
    run: RunContract,
    *,
    dynamic_actors_enabled: bool,
) -> dict[str, Any]:
    _require_phase_f_route_identity(manifest)
    arm = manifest.arms[run.arm]
    return {
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
        "route_backend": manifest.identity["route_backend"],
        "route_prior_enabled": manifest.identity["route_prior_enabled"],
        "low_obstacles_enabled": True,
        "dynamic_actors_enabled": dynamic_actors_enabled,
        "goal_checker": "position_xy",
        "cognitive_profile": run.arm,
        "obstacle_layer_mode": arm.obstacle_layer_mode,
        "critic_mode": arm.critic_mode,
    }


@dataclass
class _ManagedProcess:
    name: str
    process: subprocess.Popen[Any]
    stream: Any
    identity_dir: Path | None = None


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
        from rclpy.executors import SingleThreadedExecutor
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
    executor = None
    observed: dict[str, Any] = {}
    result: dict[str, Any] | None = None
    cleanup_errors: list[str] = []
    try:
        rclpy.init(args=None, context=context)
        node = rclpy.create_node(
            f"v6_phase_f_startup_probe_{os.getpid()}", context=context
        )
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
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
                result = {"ready": False, "reason": failure, "last_status": dict(observed)}
                break
            generation = observed.get("generation")
            held = observed.get("held")
            reason = str(observed.get("reason", ""))
            if generation == 1 and held is False and reason.startswith("released:"):
                result = {
                    "ready": True,
                    "generation": generation,
                    "held": held,
                    "reason": reason,
                }
                break
            if isinstance(generation, int) and not isinstance(generation, bool) and generation > 1:
                result = {
                    "ready": False,
                    "reason": f"unexpected startup reset generation {generation}",
                    "last_status": dict(observed),
                }
                break
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                result = {
                    "ready": False,
                    "reason": "startup reset generation 1 was not released before timeout",
                    "last_status": dict(observed),
                }
                break
            executor.spin_once(timeout_sec=min(remaining, 0.5))
    except Exception as exc:
        result = {
            "ready": False,
            "reason": f"startup readiness probe failed: {type(exc).__name__}: {exc}",
            "last_status": dict(observed),
        }
    finally:
        if executor is not None and node is not None:
            try:
                executor.remove_node(node)
            except Exception as exc:
                cleanup_errors.append(f"remove_node: {type(exc).__name__}: {exc}")
        if executor is not None:
            try:
                executor.shutdown(timeout_sec=1.0)
            except Exception as exc:
                cleanup_errors.append(f"executor_shutdown: {type(exc).__name__}: {exc}")
        if node is not None:
            try:
                node.destroy_node()
            except Exception as exc:
                cleanup_errors.append(f"destroy_node: {type(exc).__name__}: {exc}")
        if context.ok():
            try:
                context.shutdown()
            except Exception as exc:
                cleanup_errors.append(f"context_shutdown: {type(exc).__name__}: {exc}")
    if cleanup_errors:
        return {
            "ready": False,
            "reason": "startup readiness probe cleanup failed: " + "; ".join(cleanup_errors),
            "last_status": dict(observed),
        }
    return result or {
        "ready": False,
        "reason": "startup readiness probe ended without a result",
        "last_status": dict(observed),
    }


def _unix_socket_listener_active(path: Path) -> bool:
    """Return whether the exact filesystem UDS has a listening owner."""

    target = str(path)
    try:
        rows = Path("/proc/net/unix").read_text(encoding="utf-8").splitlines()[1:]
    except OSError:
        return False
    for row in rows:
        fields = row.split()
        if len(fields) < 8 or fields[7] != target:
            continue
        try:
            listening = bool(int(fields[3], 16) & 0x00010000)
            stream = int(fields[4], 16) == 1
        except ValueError:
            continue
        if listening and stream:
            return True
    return False


def _unix_socket_connects(path: Path) -> bool:
    """Fallback active-owner probe used only after no listener was observed."""

    probe = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    probe.settimeout(0.05)
    try:
        probe.connect(str(path))
    except (FileNotFoundError, ConnectionRefusedError, TimeoutError, OSError):
        return False
    finally:
        probe.close()
    return True


def _canonical_constraints_error(message: Any) -> str | None:
    map_version = str(getattr(message, "map_version", "")).strip()
    if not map_version:
        return "CognitiveMapConstraints map_version is empty"
    if str(getattr(message, "cognitive_tile_id", "")) != KUJIALE_MAP_ID:
        return "CognitiveMapConstraints tile differs from the Kujiale map_id"
    transform = tuple(float(value) for value in getattr(message, "t_map_canvas", ()))
    if len(transform) != 9 or any(
        not math.isclose(value, expected, abs_tol=1.0e-9)
        for value, expected in zip(transform, KUJIALE_T_MAP_CANVAS)
    ):
        return "CognitiveMapConstraints T_map_canvas is not the canonical identity"
    mask = tuple(bool(value) for value in getattr(message, "reachable_state_mask", ()))
    if len(mask) != 256:
        return "CognitiveMapConstraints reachable_state_mask does not contain 256 states"
    if tuple(index for index, enabled in enumerate(mask) if enabled) != KUJIALE_VALID_STATE_IDS:
        return "CognitiveMapConstraints mask differs from the canonical 51-state mask"
    return None


def _canonical_prior_error(
    message: Any,
    arm: str,
    *,
    expected_map_version: str | None = None,
) -> str | None:
    if str(getattr(message, "schema_version", "")) != "bio_nav_planning_prior_v310":
        return "PlanningPrior schema is not V3.10"
    map_version = str(getattr(message, "map_version", "")).strip()
    if not map_version:
        return "PlanningPrior map_version is empty"
    if str(getattr(message, "cognitive_tile_id", "")) != KUJIALE_MAP_ID:
        return "PlanningPrior tile differs from the Kujiale map_id"
    if expected_map_version is not None and map_version != expected_map_version:
        return "PlanningPrior map_version differs from live CognitiveMapConstraints"
    transform = tuple(float(value) for value in getattr(message, "t_map_canvas", ()))
    if len(transform) != 9 or any(
        not math.isclose(value, expected, abs_tol=1.0e-9)
        for value, expected in zip(transform, KUJIALE_T_MAP_CANVAS)
    ):
        return "PlanningPrior T_map_canvas is not the canonical identity"
    mask = tuple(bool(value) for value in getattr(message, "valid_state_mask", ()))
    if len(mask) != 256:
        return "PlanningPrior valid_state_mask does not contain 256 states"
    if tuple(index for index, enabled in enumerate(mask) if enabled) != KUJIALE_VALID_STATE_IDS:
        return "PlanningPrior valid_state_mask differs from the canonical 51-state mask"
    if arm == "M1" and bool(getattr(message, "trusted_write", True)):
        return "M1 PlanningPrior must remain untrusted shadow output"
    return None


def _expected_cognitive_parameters(arm: str) -> dict[str, str]:
    if arm == "M1":
        return {
            "startup_profile": "estimated_shadow",
            "module2_mode": "shadow",
            "active_effect_scope": "none",
        }
    if arm in {"M2", "M3"}:
        return {
            "startup_profile": "module2_causal_obstacle_active",
            "module2_mode": "active",
            "active_effect_scope": "obstacle_only",
        }
    return {}


def _uint8_value(value: Any) -> int:
    if isinstance(value, (bytes, bytearray)):
        if len(value) != 1:
            raise ValueError("uint8 field must contain one byte")
        return value[0]
    return int(value)


def _wait_for_cognitive_ready(
    manifest: CausalManifest,
    run: RunContract,
    managed: Sequence[_ManagedProcess],
    module2_socket: Path,
    timeout_sec: float = DEFAULT_COGNITIVE_READY_TIMEOUT_SEC,
) -> dict[str, Any]:
    """Wait on real UDS/ROS state before starting a cognitive-arm episode."""

    if run.arm == "M0":
        return {"ready": True, "applicability": "N/A_MODULE2_OFF"}
    if not math.isfinite(timeout_sec) or timeout_sec <= 0.0:
        raise CausalContractError("cognitive readiness timeout must be finite and positive")
    try:
        import rclpy
        from bio_nav_interfaces.msg import CognitiveMapConstraints, PlanningPrior
        from diagnostic_msgs.msg import DiagnosticArray
        from rcl_interfaces.msg import ParameterType
        from rcl_interfaces.srv import GetParameters
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    except ImportError as exc:
        return {"ready": False, "reason": f"cognitive readiness probe unavailable: {exc}"}

    context = Context()
    node = None
    executor = None
    result: dict[str, Any] | None = None
    cleanup_errors: list[str] = []
    diagnostic: dict[str, Any] = {}
    constraints: Any = None
    prior: Any = None
    parameters: dict[str, str] = {}
    parameter_future: Any = None
    expected_parameters = _expected_cognitive_parameters(run.arm)
    steady_since: float | None = None
    try:
        rclpy.init(args=None, context=context)
        node = rclpy.create_node(
            f"v6_phase_f_cognitive_probe_{run.arm.lower()}_{os.getpid()}",
            context=context,
        )
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        def receive_diagnostics(message: Any) -> None:
            for status in getattr(message, "status", ()):
                if str(getattr(status, "name", "")) != "bio_nav_ros_bridge":
                    continue
                diagnostic.clear()
                diagnostic.update({
                    "level": _uint8_value(getattr(status, "level", -1)),
                    "message": str(getattr(status, "message", "")),
                    "values": {
                        str(item.key): str(item.value)
                        for item in getattr(status, "values", ())
                    },
                })

        def receive_prior(message: Any) -> None:
            nonlocal prior
            prior = message

        def receive_constraints(message: Any) -> None:
            nonlocal constraints
            constraints = message

        diagnostic_subscription = node.create_subscription(
            DiagnosticArray, "/diagnostics", receive_diagnostics, qos
        )
        prior_subscription = node.create_subscription(
            PlanningPrior, "/bio_nav/module2/planning_prior", receive_prior, qos
        )
        constraints_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        constraints_subscription = node.create_subscription(
            CognitiveMapConstraints,
            "/bio_nav/cognitive_map/constraints",
            receive_constraints,
            constraints_qos,
        )
        parameter_client = node.create_client(
            GetParameters, "/bio_nav_ros_bridge/get_parameters"
        )
        deadline = time.monotonic() + timeout_sec
        last_reason = "waiting for Module2 listener, bridge health, parameters, and PlanningPrior"
        while True:
            failure = _startup_process_failure(managed)
            if failure is not None:
                result = {"ready": False, "reason": failure}
                break
            if parameter_future is None and parameter_client.service_is_ready():
                request = GetParameters.Request()
                request.names = list(expected_parameters)
                parameter_future = parameter_client.call_async(request)
            if parameter_future is not None and parameter_future.done() and not parameters:
                response = parameter_future.result()
                values = tuple(getattr(response, "values", ())) if response is not None else ()
                if len(values) == len(expected_parameters) and all(
                    _uint8_value(value.type) == int(ParameterType.PARAMETER_STRING)
                    for value in values
                ):
                    parameters.update({
                        name: str(value.string_value)
                        for name, value in zip(expected_parameters, values)
                    })
                else:
                    last_reason = "Bridge runtime parameters are unavailable or non-string"

            node_names = {
                (str(name), str(namespace))
                for name, namespace in node.get_node_names_and_namespaces()
            }
            bridge_node = any(
                name == "bio_nav_ros_bridge" for name, _namespace in node_names
            )
            listener = _unix_socket_listener_active(module2_socket)
            values = diagnostic.get("values", {})
            bridge_healthy = (
                diagnostic.get("level") == 0
                and values.get("state") == "RUNNING"
                and values.get("socket_connected", "").lower() == "true"
            )
            parameters_ready = parameters == expected_parameters
            constraints_error = (
                _canonical_constraints_error(constraints) if constraints is not None
                else "CognitiveMapConstraints has not been observed"
            )
            constraints_map_version = (
                str(constraints.map_version).strip()
                if constraints_error is None else None
            )
            prior_error = (
                _canonical_prior_error(
                    prior,
                    run.arm,
                    expected_map_version=constraints_map_version,
                ) if prior is not None
                else "PlanningPrior has not been observed"
            )
            ready_now = (
                listener and bridge_node and bridge_healthy
                and parameters_ready and constraints_error is None
                and prior_error is None
            )
            now = time.monotonic()
            if ready_now:
                if steady_since is None:
                    steady_since = now
                if now - steady_since >= 0.10:
                    result = {
                        "ready": True,
                        "applicability": "REQUIRED_COGNITIVE_ARM",
                        "socket_listener": True,
                        "bridge_node": True,
                        "bridge_connected_healthy": True,
                        "runtime_parameters": dict(parameters),
                        "cognitive_constraints": {
                            "map_id": str(constraints.cognitive_tile_id),
                            "map_version": constraints_map_version,
                            "valid_state_count": sum(
                                bool(value) for value in constraints.reachable_state_mask
                            ),
                        },
                        "planning_prior": {
                            "map_id": str(prior.cognitive_tile_id),
                            "map_version": str(prior.map_version),
                            "valid_state_count": sum(bool(value) for value in prior.valid_state_mask),
                            "trusted_write": bool(prior.trusted_write),
                        },
                    }
                    break
            else:
                steady_since = None
                missing: list[str] = []
                if not listener:
                    missing.append("exact UDS listener")
                if not bridge_node:
                    missing.append("Bridge node")
                if not bridge_healthy:
                    missing.append("Bridge connected health")
                if not parameters_ready:
                    missing.append("arm runtime parameters")
                if constraints_error is not None:
                    missing.append(constraints_error)
                if prior_error is not None:
                    missing.append(prior_error)
                last_reason = "waiting for " + ", ".join(missing)
            remaining = deadline - now
            if remaining <= 0.0:
                result = {
                    "ready": False,
                    "reason": "Module2 cognitive readiness timed out: " + last_reason,
                    "socket_listener": listener,
                    "bridge_node": bridge_node,
                    "bridge_diagnostic": dict(diagnostic),
                    "runtime_parameters": dict(parameters),
                    "cognitive_constraints_error": constraints_error,
                    "planning_prior_error": prior_error,
                }
                break
            executor.spin_once(timeout_sec=min(remaining, 0.10))
        # Keep explicit references until after the executor is stopped.
        _ = (
            diagnostic_subscription,
            prior_subscription,
            constraints_subscription,
            parameter_client,
        )
    except Exception as exc:
        result = {
            "ready": False,
            "reason": f"cognitive readiness probe failed: {type(exc).__name__}: {exc}",
        }
    finally:
        if executor is not None and node is not None:
            try:
                executor.remove_node(node)
            except Exception as exc:
                cleanup_errors.append(f"remove_node: {type(exc).__name__}: {exc}")
        if executor is not None:
            try:
                executor.shutdown(timeout_sec=1.0)
            except Exception as exc:
                cleanup_errors.append(f"executor_shutdown: {type(exc).__name__}: {exc}")
        if node is not None:
            try:
                node.destroy_node()
            except Exception as exc:
                cleanup_errors.append(f"destroy_node: {type(exc).__name__}: {exc}")
        if context.ok():
            try:
                context.shutdown()
            except Exception as exc:
                cleanup_errors.append(f"context_shutdown: {type(exc).__name__}: {exc}")
    if cleanup_errors:
        return {
            "ready": False,
            "reason": "cognitive readiness probe cleanup failed: " + "; ".join(cleanup_errors),
        }
    return result or {
        "ready": False,
        "reason": "cognitive readiness probe ended without a result",
    }


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
    return _ManagedProcess(
        name,
        process,
        stream,
        log_path.parent if name == "stack" else None,
    )


def _stop_process(managed: _ManagedProcess, timeout_sec: float) -> dict[str, Any]:
    """Stop one adapter, then sweep only its recorded process identities.

    The adapter leader receives the first signal so its own ordered trap can
    run.  After that leader exits, the bounded sweep repeatedly refreshes the
    exact Phase-F pid/pgid records and known descendants.  This closes the
    former one-snapshot race without using broad process-name matching.
    """

    process = managed.process
    root_pid = process.pid
    tracked_pids = {root_pid}
    tracked_groups = _managed_process_groups(root_pid)
    try:
        root_group = os.getpgid(root_pid)
    except ProcessLookupError:
        root_group = None
    if root_group is not None and root_group > 1 and root_group != os.getpgrp():
        tracked_groups.add(root_group)
    try:
        if process.poll() is None:
            if root_group is not None:
                _signal_process_groups((root_group,), signal.SIGINT)
            try:
                process.wait(timeout=max(0.0, timeout_sec))
            except subprocess.TimeoutExpired:
                if root_group is not None:
                    _signal_process_groups((root_group,), signal.SIGTERM)
                try:
                    process.wait(timeout=min(5.0, max(0.25, timeout_sec)))
                except subprocess.TimeoutExpired:
                    if root_group is not None:
                        _signal_process_groups((root_group,), signal.SIGKILL)
                    try:
                        process.wait(timeout=min(5.0, max(0.25, timeout_sec)))
                    except subprocess.TimeoutExpired:
                        pass

        deadline = time.monotonic() + max(0.25, timeout_sec)
        quiet_since: float | None = None
        first_seen: dict[int, float] = {}
        while True:
            tracked_pids, tracked_groups, _files = _refresh_cleanup_targets(
                managed.identity_dir,
                tracked_pids,
                tracked_groups,
            )
            running = _running_process_groups(tracked_groups)
            now = time.monotonic()
            if running:
                quiet_since = None
                for group in running:
                    seen_at = first_seen.setdefault(group, now)
                    age = now - seen_at
                    requested_signal = (
                        signal.SIGKILL if age >= 0.75
                        else signal.SIGTERM if age >= 0.25
                        else signal.SIGINT
                    )
                    _signal_process_groups((group,), requested_signal)
            else:
                if quiet_since is None:
                    quiet_since = now
                if now - quiet_since >= DEFAULT_CLEANUP_QUIET_SEC:
                    break
            if now >= deadline:
                break
            time.sleep(DEFAULT_CLEANUP_POLL_SEC)
        if process.poll() is None:
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                pass
    finally:
        managed.stream.close()
    tracked_pids, tracked_groups, _files = _refresh_cleanup_targets(
        managed.identity_dir,
        tracked_pids,
        tracked_groups,
    )
    remaining = sorted(_running_process_groups(tracked_groups))
    return {
        "name": managed.name,
        "root_pid": root_pid,
        "returncode": process.returncode,
        "cleanup_ok": process.poll() is not None and not remaining,
        "tracked_pids": sorted(tracked_pids),
        "tracked_process_groups": sorted(tracked_groups),
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


def _positive_int_file(path: Path) -> int | None:
    try:
        value = int(path.read_text(encoding="utf-8").strip())
    except (FileNotFoundError, PermissionError, ValueError, OSError):
        return None
    return value if value > 1 else None


def _recorded_cleanup_identities(
    run_dir: Path | None,
) -> tuple[set[int], set[int], set[Path]]:
    """Read only the child identities written by the Phase-F stack adapter."""

    pids: set[int] = set()
    groups: set[int] = set()
    files: set[Path] = set()
    if run_dir is None:
        return pids, groups, files
    for name in PHASE_F_RECORDED_CHILDREN:
        identity_path = run_dir / f"{name}.identity"
        pid_path = run_dir / f"{name}.pid"
        pgid_path = run_dir / f"{name}.pgid"
        if identity_path.is_file():
            files.add(identity_path)
            try:
                fields = identity_path.read_text(encoding="utf-8").split()
                if len(fields) == 2:
                    pid, group = (int(field) for field in fields)
                    if pid > 1:
                        pids.add(pid)
                    if group > 1:
                        groups.add(group)
            except (PermissionError, ValueError, OSError):
                pass
        if pid_path.is_file():
            files.add(pid_path)
            if (pid := _positive_int_file(pid_path)) is not None:
                pids.add(pid)
        if pgid_path.is_file():
            files.add(pgid_path)
            if (group := _positive_int_file(pgid_path)) is not None:
                groups.add(group)
    return pids, groups, files


def _descendants_from_table(
    roots: Iterable[int],
    table: Mapping[int, tuple[int, int, str]],
) -> set[int]:
    descendants = set(roots)
    while True:
        added = {
            pid for pid, (parent, _group, _state) in table.items()
            if parent in descendants and pid not in descendants
        }
        if not added:
            return descendants
        descendants.update(added)


def _refresh_cleanup_targets(
    run_dir: Path | None,
    tracked_pids: Iterable[int],
    tracked_groups: Iterable[int],
) -> tuple[set[int], set[int], set[Path]]:
    pids = set(tracked_pids)
    groups = set(tracked_groups)
    recorded_pids, recorded_groups, files = _recorded_cleanup_identities(run_dir)
    pids.update(recorded_pids)
    groups.update(recorded_groups)
    table = _process_table()
    pids.update(_descendants_from_table(pids, table))
    for pid in pids:
        row = table.get(pid)
        if row is not None:
            groups.add(row[1])
    own_group = os.getpgrp()
    groups = {group for group in groups if group > 1 and group != own_group}
    return pids, groups, files


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
    *,
    timeout_sec: float | None = None,
    quiet_sec: float | None = None,
    poll_sec: float = DEFAULT_CLEANUP_POLL_SEC,
) -> dict[str, Any]:
    """Require a stable clean window before permitting the next arm.

    Any late exact child identity, descendant group, socket, or runtime lock
    resets the quiet window.  Newly discovered task-owned groups are stopped;
    an unknown lock holder is never killed by name and therefore fails closed.
    """

    runtime_dir = Path(
        env.get("ISAAC_NAV_RUNTIME_DIR", f"/tmp/isaac_sim_ros2_nav_{os.getuid()}")
    ).expanduser().resolve()
    timeout = (
        float(env.get(
            "BIO_NAV_PHASE_F_CLEANUP_CONFIRM_TIMEOUT_SEC",
            DEFAULT_CLEANUP_CONFIRM_TIMEOUT_SEC,
        ))
        if timeout_sec is None else float(timeout_sec)
    )
    quiet = (
        float(env.get("BIO_NAV_PHASE_F_CLEANUP_QUIET_SEC", DEFAULT_CLEANUP_QUIET_SEC))
        if quiet_sec is None else float(quiet_sec)
    )
    if not all(math.isfinite(value) and value > 0.0 for value in (timeout, quiet, poll_sec)):
        raise CausalContractError("cleanup timeout, quiet window, and poll interval must be positive")

    relevant_rows = tuple(
        row for row in shutdown if row.get("name") in {"scene", "stack", "recorder"}
    )
    legacy_processes_clean = all(
        row.get("cleanup_ok", row.get("returncode") is not None)
        and not row.get("remaining_process_groups")
        for row in relevant_rows
    )
    shutdown_rows_valid = all("error" not in row for row in relevant_rows)
    root_pids = {
        int(row["root_pid"])
        for row in relevant_rows
        if isinstance(row.get("root_pid"), int) and not isinstance(row.get("root_pid"), bool)
    }
    tracked_pids = set(root_pids)
    tracked_groups = {
        int(group)
        for row in relevant_rows
        for group in row.get("tracked_process_groups", ())
        if isinstance(group, int) and not isinstance(group, bool) and group > 1
    }
    tracked_pids.update(
        int(pid)
        for row in relevant_rows
        for pid in row.get("tracked_pids", ())
        if isinstance(pid, int) and not isinstance(pid, bool) and pid > 1
    )

    deadline = time.monotonic() + timeout
    quiet_since: float | None = None
    first_seen: dict[int, float] = {}
    attempts = 0
    locks = {"ros": False, "isaac": False}
    stale_runtime_files: list[str] = []
    running: set[int] = set()
    processes_dead = False
    socket_absent = False
    socket_listener_active = False
    socket_connectable = False
    while True:
        attempts += 1
        tracked_pids, tracked_groups, identity_files = _refresh_cleanup_targets(
            run_dir,
            tracked_pids,
            tracked_groups,
        )
        table = _process_table()
        running = _running_process_groups(tracked_groups)
        tracked_processes_dead = all(
            pid not in table or table[pid][2] == "Z" for pid in tracked_pids
        )
        processes_dead = tracked_processes_dead and (
            True if root_pids else legacy_processes_clean
        )
        locks = {
            name: _lock_is_free(runtime_dir / f"{name}.lock")
            for name in ("ros", "isaac")
        }
        stale_runtime_files = sorted(str(path) for path in identity_files if path.exists())
        now = time.monotonic()

        if running:
            for group in running:
                seen_at = first_seen.setdefault(group, now)
                age = now - seen_at
                requested_signal = (
                    signal.SIGKILL if age >= 0.75
                    else signal.SIGTERM if age >= 0.25
                    else signal.SIGINT
                )
                _signal_process_groups((group,), requested_signal)

        groups_absent = not running
        if groups_absent and processes_dead:
            # These exact per-run identity files are no longer reusable once
            # their recorded processes and groups are gone.
            for path in identity_files:
                try:
                    path.unlink(missing_ok=True)
                except OSError:
                    pass
            _recorded_pids, _recorded_groups, remaining_files = (
                _recorded_cleanup_identities(run_dir)
            )
            stale_runtime_files = sorted(str(path) for path in remaining_files)

            # Only after every exact recorded process is dead may a stale
            # task-owned pathname be removed.  A live listener/connectable
            # socket is never unlinked because it has an active owner.
            socket_listener_active = _unix_socket_listener_active(module2_socket)
            socket_connectable = (
                False if socket_listener_active else _unix_socket_connects(module2_socket)
            )
            if (
                not socket_listener_active
                and not socket_connectable
                and module2_socket.exists()
            ):
                try:
                    module2_socket.unlink()
                except OSError:
                    pass
        socket_listener_active = _unix_socket_listener_active(module2_socket)
        socket_connectable = (
            False if socket_listener_active else _unix_socket_connects(module2_socket)
        )
        socket_absent = (
            not module2_socket.exists()
            and not socket_listener_active
            and not socket_connectable
        )

        clean_now = (
            shutdown_rows_valid
            and groups_absent
            and processes_dead
            and all(locks.values())
            and not stale_runtime_files
            and socket_absent
        )
        if clean_now:
            if quiet_since is None:
                quiet_since = now
            if now - quiet_since >= quiet:
                break
        else:
            quiet_since = None
        if now >= deadline:
            break
        time.sleep(poll_sec)

    process_cleanup_ok = shutdown_rows_valid and processes_dead and not running
    result = {
        "ok": (
            process_cleanup_ok
            and all(locks.values())
            and not stale_runtime_files
            and socket_absent
            and quiet_since is not None
            and time.monotonic() - quiet_since >= quiet
        ),
        "processes_clean": process_cleanup_ok,
        "locks_free": locks,
        "stale_runtime_files": stale_runtime_files,
        "module2_socket_absent": socket_absent,
        "module2_socket_listener_active": socket_listener_active,
        "module2_socket_connectable": socket_connectable,
        "tracked_pids": sorted(tracked_pids),
        "tracked_process_groups": sorted(tracked_groups),
        "remaining_process_groups": sorted(running),
        "quiet_window_sec": quiet,
        "attempts": attempts,
    }
    return result


def _rosbag_command(manifest: CausalManifest, bag_dir: Path) -> tuple[str, ...]:
    if manifest.module3_root is None:
        raise CausalContractError(
            f"Phase-F recorder requires {MODULE3_ROOT_ENV} when using an installed manifest"
        )
    qos_path = (
        manifest.module3_root
        / "ros2_ws/src/robot_experiments/config"
        / PHASE_F_QOS_CONFIG
    )
    if not qos_path.is_file():
        raise CausalContractError(f"Phase-F recorder QoS override is unavailable: {qos_path}")
    return (
        "ros2", "bag", "record", "--storage", "mcap",
        "--qos-profile-overrides-path", str(qos_path),
        "--include-unpublished-topics", "--output", str(bag_dir),
        *DISPATCHER_TOPICS, *PASSIVE_EVALUATOR_TOPICS, *ISOLATION_AUDIT_TOPICS,
    )


def run_campaign(
    manifest: CausalManifest,
    adapters: AdapterTemplates | None,
    output_root: str | Path,
    *,
    pilot: bool,
    shutdown_timeout_sec: float = DEFAULT_SHUTDOWN_TIMEOUT_SEC,
    prepared_plan: Mapping[str, Any] | None = None,
    recorder_command: Callable[[CausalManifest, Path], tuple[str, ...]] | None = None,
    evidence_recorder: Callable[
        [CausalManifest, RunContract, Path, Path, Path], dict[str, Any]
    ] | None = None,
    stop_producer_after_episode: bool = True,
    classify_baseline_collision: bool = True,
) -> dict[str, Any]:
    """Run ordered independent episodes; stop if an arm cannot be cleaned."""

    root = Path(output_root).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    plan = dict(prepared_plan) if prepared_plan is not None else build_plan(
        manifest, adapters=adapters, pilot=pilot, output_root=root
    )
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
                cognitive_ready = _wait_for_cognitive_ready(
                    manifest,
                    run,
                    managed,
                    Path(row["setup"]["module2_socket"]),
                    float(manifest.identity["timeout_sec"]),
                )
                status["cognitive_readiness"] = cognitive_ready
                if cognitive_ready.get("ready") is not True:
                    status["state"] = "MODULE2_NOT_READY"
                    status["reason"] = str(
                        cognitive_ready.get("reason", "Module2 cognitive readiness failed")
                    )
                else:
                    managed.append(_start_process(
                        "recorder",
                        (
                            recorder_command(manifest, run_dir / "bag")
                            if recorder_command is not None
                            else _rosbag_command(manifest, run_dir / "bag")
                        ),
                        run_dir / "recorder.log", env=campaign_env
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
                    if run.arm in {"M2", "M3"} and stop_producer_after_episode:
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
            except (OSError, CausalContractError, ValueError) as exc:
                cleanup = {"ok": False, "error": str(exc)}
            status["cleanup"] = cleanup
            if cleanup.get("ok") is not True:
                cleanup_failed = True
                status["state_before_cleanup"] = status["state"]
                status["state"] = "ARM_CLEANUP_FAILED"
                status["reason"] = "arm children or runtime locks remained after shutdown"

        evidence_path = run_dir / f"{run.run_id}.json"
        try:
            evidence = (
                evidence_recorder(
                    manifest, run, run_dir / "bag",
                    run_dir / "episode.jsonl", evidence_path,
                )
                if evidence_recorder is not None
                else record_evidence_from_bag(
                    manifest, run, run_dir / "bag",
                    run_dir / "episode.jsonl", evidence_path,
                )
            )
            status["evidence_file"] = str(evidence_path)
            status["evidence_recorded"] = True
            if (
                classify_baseline_collision
                and run.arm in {"M0", "M1"}
                and status.get("state") == "EPISODE_FAILED"
                and status.get("episode_returncode") == 2
                and status.get("cleanup", {}).get("ok") is True
            ):
                baseline_result, _ = _evaluate_run(
                    manifest, run, evidence_path
                )
                status["evidence_verdict"] = baseline_result.verdict
                status["evidence_reasons"] = list(baseline_result.reasons)
                if (
                    baseline_result.verdict == "VALID"
                    and baseline_result.collision is True
                    and baseline_result.success is False
                    and baseline_result.terminal_zero_confirmed is True
                ):
                    status["state"] = "BASELINE_OUTCOME_RECORDED"
                    status["baseline_outcome"] = "collision"
            if run.arm in {"M2", "M3"}:
                freshness = _mapping(evidence.get("freshness"), "freshness")
                status["nominal_ttl_status"] = freshness.get(
                    "ttl_clear_applicability"
                )
                status["external_active_controller_probe_required"] = (
                    freshness.get("external_active_controller_probe_required") is True
                )
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
            row["state"] not in {
                "EPISODE_FINISHED", "BASELINE_OUTCOME_RECORDED",
            }
            for row in results
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


def _circle_matches_rectangle(
    candidate: Mapping[str, Any],
    rectangle: Sequence[float],
    tolerance_m: float,
) -> bool:
    x = float(candidate["x"])
    y = float(candidate["y"])
    radius = float(candidate["radius_m"])
    min_x, min_y, max_x, max_y = (float(value) for value in rectangle)
    dx = max(min_x - x, 0.0, x - max_x)
    dy = max(min_y - y, 0.0, y - max_y)
    return math.hypot(dx, dy) <= radius + tolerance_m


def _scan_and_spatial_metrics(
    samples: Any,
    tolerance_m: float,
    *,
    physical_obstacle: Mapping[str, Any],
    validate_obstacles: bool = True,
) -> dict[str, Any]:
    if not isinstance(samples, list) or not samples:
        raise CausalContractError("synchronized_samples must be a non-empty list")
    synchronized = invisible = valid_scan_samples = 0
    source_visible = source_matched = candidate_tp = candidate_fp = 0
    best_center_errors: list[float] = []
    candidate_radii: list[float] = []
    for index, raw_sample in enumerate(samples):
        sample = _mapping(raw_sample, f"synchronized_samples[{index}]")
        sample_obstacle = sample.get("physical_obstacle")
        if sample_obstacle is None:
            sample_obstacle = physical_obstacle
        sample_obstacle = _mapping(
            sample_obstacle, f"synchronized_samples[{index}].physical_obstacle"
        )
        center = tuple(float(value) for value in sample_obstacle["center"][:2])
        half_x = 0.5 * float(sample_obstacle["size"][0])
        half_y = 0.5 * float(sample_obstacle["size"][1])
        rectangle = (
            center[0] - half_x,
            center[1] - half_y,
            center[0] + half_x,
            center[1] + half_y,
        )
        stamp = sample.get("stamp_ns")
        frame = sample.get("frame_id")
        if isinstance(stamp, bool) or not isinstance(stamp, int) or stamp < 0 or not isinstance(frame, str) or not frame:
            raise CausalContractError(f"synchronized_samples[{index}] missing valid frame/time")
        synchronized += 1
        footprints = sample.get("rgbd_obstacle_footprints")
        typed = sample.get("typed_obstacles")
        if not isinstance(footprints, list) or not isinstance(typed, list):
            raise CausalContractError(f"synchronized_samples[{index}] missing obstacle arrays")
        scan_points = sample.get("scan_point_count")
        scan_hits = sample.get("scan_hits_in_obstacle_footprints")
        if isinstance(scan_points, bool) or not isinstance(scan_points, int) or scan_points < 0:
            raise CausalContractError(f"synchronized_samples[{index}].scan_point_count invalid")
        if isinstance(scan_hits, bool) or not isinstance(scan_hits, int) or scan_hits < 0:
            raise CausalContractError(f"synchronized_samples[{index}].scan_hits_in_obstacle_footprints invalid")
        scan_valid = sample.get("scan_valid")
        if not isinstance(scan_valid, bool):
            raise CausalContractError(f"synchronized_samples[{index}].scan_valid invalid")
        if scan_hits != 0:
            raise CausalContractError(
                f"synchronized_samples[{index}] low-obstacle scan hit count must be zero"
            )
        if scan_valid and scan_points > 0:
            valid_scan_samples += 1
        if not validate_obstacles:
            continue
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
            if footprint_row.get("source") != PHYSICAL_DEPTH_FOOTPRINT_SOURCE:
                raise CausalContractError(
                    f"synchronized_samples[{index}] footprint source is not physical RGBD evidence"
                )
            point_count = int(footprint_row.get("point_count", 0))
            hit_count = int(footprint_row.get("hit_count", 0))
            footprint_rectangle = footprint_row.get("rectangle")
            if point_count <= 0 or hit_count != point_count:
                raise CausalContractError(
                    f"synchronized_samples[{index}] footprint real hit count invalid"
                )
            if (
                not isinstance(footprint_rectangle, Sequence)
                or isinstance(footprint_rectangle, (str, bytes))
                or len(footprint_rectangle) != 4
                or any(
                    not math.isclose(float(actual), expected, abs_tol=1.0e-9)
                    for actual, expected in zip(footprint_rectangle, rectangle)
                )
            ):
                raise CausalContractError(
                    f"synchronized_samples[{index}] footprint is not the physical box rectangle"
                )
        if footprints and scan_valid and scan_points > 0 and scan_hits == 0:
            invisible += 1
        visible = bool(footprints)
        if visible:
            source_visible += 1
        accepted: list[Mapping[str, Any]] = []
        for obstacle_value in typed:
            obstacle = _mapping(obstacle_value, "typed_obstacle")
            if obstacle.get("accepted") is not True:
                continue
            radius = _finite_float(obstacle.get("radius_m"), minimum=0.0)
            if radius is None or radius <= 0.0:
                raise CausalContractError(
                    f"synchronized_samples[{index}] accepted candidate radius invalid"
                )
            candidate_radii.append(radius)
            accepted.append(obstacle)
            point = (float(obstacle["x"]), float(obstacle["y"]))
            computed_error = math.dist(point, center) if visible else None
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
        matching = [
            obstacle for obstacle in accepted
            if _circle_matches_rectangle(obstacle, rectangle, tolerance_m)
        ]
        if matching:
            best = min(
                matching,
                key=lambda obstacle: math.dist(
                    (float(obstacle["x"]), float(obstacle["y"])), center
                ),
            )
            if visible:
                source_matched += 1
            candidate_tp += 1
            candidate_fp += len(accepted) - 1
            best_center_errors.append(math.dist(
                (float(best["x"]), float(best["y"])), center
            ))
        else:
            candidate_fp += len(accepted)
    candidate_total = candidate_tp + candidate_fp
    return {
        "synchronized_frames": synchronized,
        "valid_scan_samples": valid_scan_samples,
        "scan_invisible_rgbd_pairs": invisible,
        "source_visible_count": source_visible,
        "source_matched_count": source_matched,
        "source_recall": source_matched / source_visible if source_visible else 0.0,
        "candidate_true_positive_count": candidate_tp,
        "candidate_false_positive_count": candidate_fp,
        "candidate_precision": candidate_tp / candidate_total if candidate_total else 0.0,
        "candidate_radius_max_m": max(candidate_radii, default=0.0),
        "best_center_errors_m": tuple(best_center_errors),
    }


REQUIRED_EVIDENCE_KEYS = (
    "run_id", "repeat", "arm", "identity", "reset", "freshness",
    "sensor_counts", "synchronized_samples", "obstacle_validation", "layer", "critic",
    "shadow_obstacle_candidate", "planning_prior", "costmaps", "plan", "optimal_trajectory", "odom",
    "cmd_vel", "passive", "action", "route", "module2_health", "isolation",
    "navigation_metrics",
)


def _evaluate_run(manifest: CausalManifest, run: RunContract, path: Path) -> tuple[RunResult, dict[str, Any]]:
    row: Mapping[str, Any] = {}
    spatial: Mapping[str, Any] | None = None
    plan: tuple[tuple[float, float], ...] = ()
    optimal_trajectory: tuple[tuple[float, float], ...] = ()
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
        row = _mapping(raw, str(path))
        try:
            plan = _points(row.get("plan"), "plan")
        except (TypeError, ValueError, CausalContractError):
            pass
        try:
            optimal_trajectory = _points(
                row.get("optimal_trajectory"), "optimal_trajectory"
            )
        except (TypeError, ValueError, CausalContractError):
            pass
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

        shadow_candidate = _mapping(
            row["shadow_obstacle_candidate"], "shadow_obstacle_candidate"
        )
        if run.arm == "M1":
            shadow_messages = int(shadow_candidate.get("message_count", 0))
            nonempty_shadow_messages = int(
                shadow_candidate.get("nonempty_message_count", 0)
            )
            if (
                shadow_messages <= 0
                or nonempty_shadow_messages <= 0
                or int(shadow_candidate.get(
                    "static_depth_revalidated_geometry_count", 0
                )) <= 0
            ):
                raise CausalContractError(
                    "M1 lacks non-empty static-depth-revalidated typed shadow geometry"
                )
            if (
                int(shadow_candidate.get("trusted_write_count", 0)) != 0
                or int(shadow_candidate.get("shadow_rejection_count", 0))
                != nonempty_shadow_messages
                or int(shadow_candidate.get("invalid_geometry_count", 0)) != 0
            ):
                raise CausalContractError(
                    "M1 typed candidate violates untrusted shadow semantics"
                )

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
        if run.arm == "M1":
            for scope in ("global", "local"):
                layer_status = _mapping(layer.get(scope), f"layer.{scope}")
                if any(int(layer_status.get(key, 0)) for key in (
                    "applied_count", "cells", "active_cells", "max_cost_increase",
                )):
                    raise CausalContractError(
                        "M1 shadow obstacle layer applied or raised Costmap cells"
                    )
        if arm.obstacle_layer_mode == "active":
            if any(cells <= 0 for cells in layer_cells):
                raise CausalContractError("active obstacle layer lacks global/local applied cells")
            if any(
                int(_mapping(layer.get(scope), f"layer.{scope}").get(
                    "applied_count", 0
                )) <= 0
                for scope in ("global", "local")
            ):
                raise CausalContractError(
                    "active obstacle layer lacks global/local applied status"
                )
        if arm.critic_mode != "active" and critic.get("applied") is not False:
            raise CausalContractError("off/shadow critic must not be applied")
        if not isinstance(critic.get("reason"), str) or not critic["reason"]:
            raise CausalContractError("critic reason evidence missing")
        if not isinstance(row["planning_prior"], list):
            raise CausalContractError("planning_prior must be a list")
        if run.arm == "M0" and row["planning_prior"]:
            raise CausalContractError("M0 planning_prior must remain empty")
        if run.arm == "M1" and (
            not row["planning_prior"]
            or not any(
                prior.get("module2_healthy") is True
                and prior.get("observation_valid") is True
                for prior in row["planning_prior"]
                if isinstance(prior, Mapping)
            )
            or any(
                prior.get("trusted_write") is True
                for prior in row["planning_prior"]
                if isinstance(prior, Mapping)
            )
        ):
            raise CausalContractError("M1 healthy untrusted PlanningPrior evidence missing")
        for key in ("global", "local"):
            costmap = _mapping(row["costmaps"], "costmaps").get(key)
            if not isinstance(costmap, Mapping) or not costmap.get("recorded"):
                raise CausalContractError(f"costmaps.{key} missing")

        tolerance = float(manifest.criteria["typed_spatial_match_tolerance_m"])
        physical_obstacle = _load_frozen_obstacle(manifest)
        sensor_counts = _mapping(row["sensor_counts"], "sensor_counts")
        scan_message_count = sensor_counts.get("scan_message_count")
        if (
            isinstance(scan_message_count, bool)
            or not isinstance(scan_message_count, int)
            or scan_message_count <= 0
        ):
            raise CausalContractError("/scan message count must be positive")
        active_obstacle_validation = run.arm in {"M1", "M2", "M3"}
        spatial = _scan_and_spatial_metrics(
            row["synchronized_samples"],
            tolerance,
            physical_obstacle=physical_obstacle,
            validate_obstacles=active_obstacle_validation,
        )
        if spatial["valid_scan_samples"] == 0:
            raise CausalContractError(
                "no synchronized non-empty /scan sample was recorded"
            )
        spatial_gate_reasons: list[str] = []
        if active_obstacle_validation and spatial["source_visible_count"] == 0:
            if run.arm == "M1":
                raise CausalContractError("physical low-box RGBD visibility evidence missing")
            spatial_gate_reasons.append(
                "physical low-box RGBD visibility evidence missing"
            )
        validations = row["obstacle_validation"]
        if not isinstance(validations, list):
            raise CausalContractError("obstacle_validation must be a list")
        if active_obstacle_validation and not validations:
            raise CausalContractError("typed obstacle validation evidence missing")
        if run.arm in {"M2", "M3"}:
            if spatial["source_recall"] < float(manifest.criteria["source_recall_min"]):
                spatial_gate_reasons.append(
                    f"{run.arm} source recall below engineering threshold"
                )
            if spatial["candidate_precision"] < float(
                manifest.criteria["candidate_precision_min"]
            ):
                spatial_gate_reasons.append(
                    f"{run.arm} candidate precision below engineering threshold"
                )
            if spatial["candidate_radius_max_m"] > float(
                manifest.criteria["candidate_radius_max_m"]
            ):
                spatial_gate_reasons.append(
                    f"{run.arm} candidate radius exceeds engineering threshold"
                )

        freshness = _mapping(row["freshness"], "freshness")
        expected_ttl_applicability = (
            NOMINAL_TTL_STATUS if run.arm in {"M2", "M3"}
            else "not_applicable_inactive"
        )
        if freshness.get("ttl_clear_applicability") != expected_ttl_applicability:
            raise CausalContractError("TTL-clear applicability does not match arm")
        max_age = float(freshness.get("max_typed_obstacle_age_sec", 0.0))
        if run.arm in {"M2", "M3"} and (
            freshness.get("external_active_controller_probe_required") is not True
            or freshness.get("external_active_controller_probe_status")
            != "NOT_EVALUATED_BY_NOMINAL_RUN"
        ):
            raise CausalContractError(
                "nominal run must defer TTL to the separate active-controller probe"
            )
        if run.arm == "M3" and freshness.get("critic_ttl_status") != NOMINAL_TTL_STATUS:
            raise CausalContractError(
                "nominal M3 critic TTL status must defer to the separate active-controller probe"
            )
        verdict, reasons = (
            ("INVALID", tuple(spatial_gate_reasons))
            if spatial_gate_reasons else ("VALID", ())
        )

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
        if not terminal_zero:
            verdict, reasons = "INVALID", ("terminal_zero_not_confirmed", *reasons)
        if verdict == "VALID":
            if collision:
                reasons = ("collision",)
            elif not success or action.get("state") != "SUCCEEDED":
                reasons = (str(action.get("stop_reason") or "route_not_succeeded"),)

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
            synchronized_frames=spatial["synchronized_frames"],
            scan_invisible_rgbd_pairs=spatial["scan_invisible_rgbd_pairs"],
            typed_spatial_matches=spatial["candidate_true_positive_count"],
            typed_spatial_total=(
                spatial["candidate_true_positive_count"]
                + spatial["candidate_false_positive_count"]
            ),
            source_visible_count=spatial["source_visible_count"],
            source_matched_count=spatial["source_matched_count"],
            source_recall=spatial["source_recall"],
            candidate_true_positive_count=spatial["candidate_true_positive_count"],
            candidate_false_positive_count=spatial["candidate_false_positive_count"],
            candidate_precision=spatial["candidate_precision"],
            candidate_radius_max_m=spatial["candidate_radius_max_m"],
            best_center_errors_m=spatial["best_center_errors_m"],
            path_length_m=path_length(plan),
            local_trajectory_length_m=path_length(optimal_trajectory),
            near_obstacle_speed_mps=float(critic.get("near_obstacle_speed_mps", 0.0)),
            minimum_clearance_m=clearance,
            collision=collision,
            success=success,
            action_state=(
                action.get("state") if isinstance(action.get("state"), str) else None
            ),
            terminal_zero_confirmed=terminal_zero,
            reroute_direction=path_direction(plan),
            critic_participation=critic_participation,
            critic_applied=_known_bool(critic.get("applied")),
            critic_status_count=_known_int(critic.get("status_count")),
            critic_applied_count=_known_int(critic.get("applied_count")),
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
        passive = _known_mapping(row.get("passive"))
        action = _known_mapping(row.get("action"))
        critic = _known_mapping(row.get("critic"))
        freshness = _known_mapping(row.get("freshness"))
        if spatial is None:
            try:
                spatial = _scan_and_spatial_metrics(
                    row.get("synchronized_samples"),
                    float(manifest.criteria["typed_spatial_match_tolerance_m"]),
                    physical_obstacle=_load_frozen_obstacle(manifest),
                    validate_obstacles=run.arm in {"M1", "M2", "M3"},
                )
            except (KeyError, TypeError, ValueError, CausalContractError):
                spatial = {}
        candidate_true_positive = _known_int(
            spatial.get("candidate_true_positive_count")
        )
        candidate_false_positive = _known_int(
            spatial.get("candidate_false_positive_count")
        )
        critic_applied = _known_bool(critic.get("applied"))
        critic_reason = critic.get("reason", critic.get("fallback_reason"))
        if critic_applied is True:
            critic_participation = (
                "online_applied"
                if isinstance(critic_reason, str)
                and "cost_delta_applied=true" in critic_reason
                else "applied_unverified"
            )
        elif critic_applied is False:
            critic_participation = "none"
        else:
            critic_participation = "unavailable"
        action_state = action.get("state")
        best_center_errors = spatial.get("best_center_errors_m")
        return RunResult(
            run_id=run.run_id,
            repeat=run.repeat,
            arm=run.arm,
            verdict="INVALID",
            reasons=(str(exc),),
            synchronized_frames=_known_int(spatial.get("synchronized_frames")),
            scan_invisible_rgbd_pairs=_known_int(
                spatial.get("scan_invisible_rgbd_pairs")
            ),
            typed_spatial_matches=candidate_true_positive,
            typed_spatial_total=(
                candidate_true_positive + candidate_false_positive
                if candidate_true_positive is not None
                and candidate_false_positive is not None
                else None
            ),
            source_visible_count=_known_int(spatial.get("source_visible_count")),
            source_matched_count=_known_int(spatial.get("source_matched_count")),
            source_recall=_known_float(spatial.get("source_recall")),
            candidate_true_positive_count=candidate_true_positive,
            candidate_false_positive_count=candidate_false_positive,
            candidate_precision=_known_float(spatial.get("candidate_precision")),
            candidate_radius_max_m=_known_float(
                spatial.get("candidate_radius_max_m")
            ),
            best_center_errors_m=(
                tuple(float(value) for value in best_center_errors)
                if isinstance(best_center_errors, (list, tuple))
                else None
            ),
            path_length_m=path_length(plan) if plan else None,
            local_trajectory_length_m=(
                path_length(optimal_trajectory) if optimal_trajectory else None
            ),
            near_obstacle_speed_mps=_known_float(
                critic.get("near_obstacle_speed_mps")
            ),
            minimum_clearance_m=_known_float(
                passive.get("minimum_clearance_m")
            ),
            collision=_known_bool(passive.get("collision")),
            success=_known_bool(passive.get("success")),
            action_state=action_state if isinstance(action_state, str) else None,
            terminal_zero_confirmed=_known_bool(
                action.get("terminal_zero_confirmed")
            ),
            reroute_direction=path_direction(plan) if plan else "unavailable",
            critic_participation=critic_participation,
            critic_applied=critic_applied,
            critic_status_count=_known_int(critic.get("status_count")),
            critic_applied_count=_known_int(critic.get("applied_count")),
            critic_ttl_status=(
                str(freshness["critic_ttl_status"])
                if freshness.get("critic_ttl_status") is not None
                else "UNAVAILABLE"
            ),
            critic_post_expiry_applied=_known_bool(
                freshness.get("critic_post_expiry_applied")
            ),
            critic_stale_active_probe=(
                str(freshness["critic_stale_active_probe"])
                if freshness.get("critic_stale_active_probe") is not None
                else "UNAVAILABLE"
            ),
            evidence_file=str(path),
        ), {"raw": row, "plan": plan, "local_trajectory": optimal_trajectory}


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
    if not lhs_plan or not rhs_plan:
        raise CausalContractError(f"{trajectory_source} unavailable for pair")
    if (
        lhs_result.near_obstacle_speed_mps is None
        or rhs_result.near_obstacle_speed_mps is None
        or lhs_result.minimum_clearance_m is None
        or rhs_result.minimum_clearance_m is None
    ):
        raise CausalContractError(f"scalar metrics unavailable for {trajectory_source} pair")
    hausdorff = path_hausdorff(lhs_plan, rhs_plan)
    lhs_length = path_length(lhs_plan)
    rhs_length = path_length(rhs_plan)
    denominator = max(lhs_length, 1e-9)
    return PairResult(
        repeat=lhs_result.repeat,
        lhs_arm=lhs_result.arm,
        rhs_arm=rhs_result.arm,
        trajectory_source=trajectory_source,
        diagnostic_when_invalid=(
            lhs_result.verdict == "INVALID" or rhs_result.verdict == "INVALID"
        ),
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

    invalid_results = [
        result for result in ordered_results if result.verdict == "INVALID"
    ]
    invalid = [result.run_id for result in invalid_results]
    reasons: list[str] = []
    m1_m0: list[PairResult] = []
    m2_m1: list[PairResult] = []
    m3_m1: list[PairResult] = []
    m3_m2: list[PairResult] = []
    def add_reason(reason: str) -> None:
        if reason not in reasons:
            reasons.append(reason)

    def append_pair(
        destination: list[PairResult],
        lhs_arm: str,
        rhs_arm: str,
        repeat: int,
        *,
        trajectory_source: str = "global_plan",
    ) -> None:
        try:
            destination.append(_pair(
                evaluated[(repeat, lhs_arm)],
                evaluated[(repeat, rhs_arm)],
                trajectory_source=trajectory_source,
            ))
        except (KeyError, TypeError, ValueError, CausalContractError):
            return

    if invalid:
        add_reason("invalid_or_stopped_runs:" + ",".join(invalid))
        for result in invalid_results:
            for reason in result.reasons:
                add_reason(f"{result.run_id}:{reason}")

    for repeat in repeats:
        append_pair(m1_m0, "M0", "M1", repeat)
        append_pair(m2_m1, "M1", "M2", repeat)
        append_pair(m3_m1, "M1", "M3", repeat)
        append_pair(
            m3_m2, "M2", "M3", repeat,
            trajectory_source="local_trajectory",
        )

    if m1_m0 and any(
        pair.hausdorff_m > float(manifest.criteria["m1_m0_path_hausdorff_max_m"])
        or pair.length_delta_fraction > float(
            manifest.criteria["m1_m0_path_length_delta_max_fraction"]
        )
        for pair in m1_m0
    ):
        add_reason("M1_vs_M0_path_similarity_below_diagnostic_target")

    clearance_target = float(manifest.criteria["active_clearance_gain_min_m"])
    for arm, pairs in (("M2", m2_m1), ("M3", m3_m1)):
        if (
            pairs
            and statistics.median(pair.clearance_gain_m for pair in pairs)
            < clearance_target
        ):
            add_reason(f"{arm}_median_clearance_gain_below_diagnostic_target")
        active_results = [evaluated[(repeat, arm)][0] for repeat in repeats]
        if any(active.collision is True for active in active_results):
            add_reason(f"{arm}_collision_safety_stop")
        if any(
            active.success is False and active.collision is False
            for active in active_results
        ):
            add_reason(f"{arm}_navigation_failed")
        directions = {result.reroute_direction for result in active_results}
        if len(directions) != 1 or directions & {"unknown", "straight", "unavailable"}:
            add_reason(f"{arm}_reroute_direction_inconsistent")

    m3_results = [evaluated[(repeat, "M3")][0] for repeat in repeats]
    if any(result.critic_participation == "none" for result in m3_results):
        add_reason("M3_critic_participation_missing")
    if any(
        (_known_int(
            _known_mapping(evaluated[(repeat, "M3")][1]["raw"].get("critic")).get(
                "cost_delta_nonzero_count"
            )
        ) or 0) <= 0
        for repeat in repeats
    ):
        add_reason("M3_critic_nonzero_cost_delta_missing")
    if m3_m2:
        separation = statistics.median(pair.hausdorff_m for pair in m3_m2)
        separation_target = float(
            manifest.criteria["m3_m2_trajectory_separation_min_m"]
        )
        if separation < separation_target:
            add_reason("M3_trajectory_separation_below_diagnostic_target")

    def control_changed(pair: PairResult) -> bool:
        return (
            pair.hausdorff_m > 1.0e-9
            or pair.length_delta_fraction > 1.0e-9
            or abs(pair.near_obstacle_speed_delta_mps) > 1.0e-9
        )

    def arm_results(arm: str) -> list[RunResult]:
        return [evaluated[(repeat, arm)][0] for repeat in repeats]

    m0_results = arm_results("M0")
    m1_results = arm_results("M1")

    # Phase-F isolation is established by valid M0/M1 rows with the same
    # collision outcome while M1 remains zero-write. The zero-write property
    # is an individual M1 validity gate above; global-plan similarity is only a
    # diagnostic target because stochastic replanning can change an otherwise
    # isolated shadow trajectory.
    baseline_isolation = all(
        m0.verdict == "VALID"
        and m0.collision is True
        and m1.verdict == "VALID"
        and m1.collision is True
        for m0, m1 in zip(m0_results, m1_results)
    )

    # M2 admission deliberately keeps the model-data and write-path gates in
    # _evaluate_run. Its observed net benefit is the causal outcome change
    # from the isolated M1 collision to success without collision; clearance
    # and path-shape magnitudes remain useful diagnostics, not extra gates.
    m2_net_benefit = all(
        (m1 := evaluated[(repeat, "M1")][0]).verdict == "VALID"
        and m1.collision is True
        and (m2 := evaluated[(repeat, "M2")][0]).verdict == "VALID"
        and m2.success is True
        and m2.collision is False
        and m2.terminal_zero_confirmed is True
        for repeat in repeats
    )

    m3_pairs = {pair.repeat: pair for pair in m3_m2}
    m3_admitted = all(
        repeat in m3_pairs
        and (m3 := evaluated[(repeat, "M3")][0]).verdict == "VALID"
        and m3.success is True
        and m3.collision is False
        and m3.terminal_zero_confirmed is True
        and m3.critic_participation == "online_applied"
        and (_known_int(
            _known_mapping(evaluated[(repeat, "M3")][1]["raw"].get("critic")).get(
                "cost_delta_nonzero_count"
            )
        ) or 0) > 0
        for repeat in repeats
    )
    m3_incremental_benefit = m3_admitted and all(
        repeat in m3_pairs
        and (
            m3_pairs[repeat].clearance_gain_m > 0.0
            or m3_pairs[repeat].near_obstacle_speed_delta_mps < 0.0
        )
        and control_changed(m3_pairs[repeat])
        for repeat in repeats
    )
    if not m3_admitted:
        add_reason("M3_NOT_ADMITTED_EVIDENCE_INSUFFICIENT")
    elif not m3_incremental_benefit:
        add_reason("M3_NO_INCREMENTAL_BENEFIT_DIAGNOSTIC")

    selection_critical_invalid = [
        result for result in invalid_results if result.arm in {"M0", "M1", "M2"}
    ]
    if selection_critical_invalid:
        verdict = "INVALID"
        selected_arm = None
        selection_outcome = "NOT_SELECTED_INVALID_EVIDENCE"
    elif not baseline_isolation:
        add_reason("M0_M1_BASELINE_COLLISION_ISOLATION_NOT_DEMONSTRATED")
        verdict = "FAIL"
        selected_arm = None
        selection_outcome = "NOT_SELECTED_SHADOW_ISOLATION_FAILED"
    elif not m2_net_benefit:
        add_reason("M2_CAUSAL_NET_BENEFIT_NOT_DEMONSTRATED")
        verdict = "FAIL"
        selected_arm = None
        selection_outcome = "NO_CAUSAL_NET_BENEFIT"
    elif m3_incremental_benefit:
        verdict = "PASS_ENGINEERING_CAUSAL_CANDIDATE_TTL_PENDING"
        selected_arm = "M3"
        selection_outcome = "INCREMENTAL_BENEFIT_KEEP_M3_CRITIC_ON"
    else:
        verdict = "PASS_ENGINEERING_CAUSAL_CANDIDATE_TTL_PENDING"
        selected_arm = "M2"
        if not m3_admitted:
            selection_outcome = "M3_NOT_ADMITTED_EVIDENCE_INSUFFICIENT"
        else:
            add_reason("NO_INCREMENTAL_BENEFIT_KEEP_M2_CRITIC_OFF")
            selection_outcome = "NO_INCREMENTAL_BENEFIT_KEEP_M2_CRITIC_OFF"

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
        formal_qualification=False,
        phase_f_complete=False,
        verdict=verdict,
        reasons=tuple(reasons),
        selected_arm=selected_arm,
        selected_arm_active_ttl_status=(
            "PENDING" if selected_arm is not None else "NOT_APPLICABLE_NO_SELECTION"
        ),
        selection_outcome=selection_outcome,
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
    plan_parser.add_argument("--module2-asset-root")
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
    run_parser.add_argument("--module2-asset-root", required=True)
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
            if args.exact_adapters and args.module2_asset_root is None:
                raise CausalContractError(
                    "--module2-asset-root is required with --exact-adapters"
                )
            if any(supplied) and not all(supplied):
                raise CausalContractError("plan adapters must be supplied together")
            adapters = (
                exact_adapter_templates(manifest, args.module2_asset_root)
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
            # Exit zero is intentionally limited to a pilot arm-selection
            # candidate. The selected arm still needs the separate active TTL
            # probe, so this does not declare Phase F or formal qualification
            # complete.
            return 0 if (
                args.pilot
                and summary.verdict
                == "PASS_ENGINEERING_CAUSAL_CANDIDATE_TTL_PENDING"
            ) else 2
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
            exact_adapter_templates(manifest, args.module2_asset_root)
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
