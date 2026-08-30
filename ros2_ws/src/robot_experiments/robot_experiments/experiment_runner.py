"""Run deterministic NavigateToPose trials and write reproducible manifests."""

from __future__ import annotations

from dataclasses import dataclass, replace
from datetime import datetime, timezone
import csv
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import shutil
import signal
import subprocess
import time
from typing import Any, Mapping

import yaml

from action_msgs.msg import GoalInfo, GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CollisionMonitorState, Costmap
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Odometry, Path as NavPath
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, Empty as EmptyMessage, String
from std_srvs.srv import Empty, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from bio_nav_interfaces.msg import (
    CanonicalRoute,
    CognitiveObstacleArray,
    EdgePriorArray,
    NavigationGraph,
    PlanningPrior,
    RiskLayerStatus,
    RouteEdgeCostArray,
    RouteProgress,
    SRDREdgeDiagnosticArray,
)

from .configuration import ConfigurationError
from .metrics import (
    SingleRunObservation,
    SingleRunThresholds,
    evaluate_single_run,
    path_length,
    wrap_angle,
)
from .motion_benchmark import parse_reset_stop_gate_status
from .report import configuration_sha256, write_run_report
from .reset_receipt import parse_reset_receipt
from .scenario import (
    RunSelection,
    Scenario,
    load_scenario,
    validate_dynamic_physical_contract,
    validate_dynamic_runtime_contract,
    validate_navigation_runner_scenario,
)
from .spawn_poses import SpawnPose, load_spawn_pose
from .static_contact import (
    exceeds_overlap_tolerance,
    load_robot_footprint,
    select_declared_static_obstacles,
    static_contact_summary,
)


@dataclass(frozen=True)
class OdometrySample:
    x: float
    y: float
    yaw_rad: float
    linear_speed_mps: float
    angular_speed_radps: float
    stamp_s: float
    received_at: float


@dataclass(frozen=True)
class CommandSample:
    linear_speed_mps: float
    angular_speed_radps: float
    stamp_s: float


PREGOAL_AUTHORIZATION_RECEIPT = "R2C4_R2_PREGOAL_AUTHORIZED"
APPEARANCE_NAV2_PROFILES = frozenset({
    "stable",
    "dynamic_avoidance",
    "bio_nav_planning_only",
    "bio_nav_risk_only",
    "bio_nav_tiebreak_risk",
    "attempt21_static_collection",
    "bio_nav_rgbd_risk_shadow",
    "bio_nav_rgbd_risk_ab",
    "v6_low_obstacle_isolation",
    # Attempt-23: stock-critics global-prior profile; no camera, rendering,
    # or costmap-visual behavior changes, so appearance capture stays valid.
    "attempt23_global_prior",
})

# A scenario lists every physical actor so the startup contract can compare
# IDs and geometry.  These named Isaac schema-v4 case sets select the subset
# that is actually armed during one full-route run.
DYNAMIC_CASE_SET_MOTIONS = {
    "single_dynamic_g2_crossing": frozenset({"crossing"}),
    "full_route_three_stage": frozenset(
        {"local_bypass", "g2_g3_exit", "g5_g1_crossing"}
    ),
    "full_route_four_stage": frozenset(
        {"oncoming", "crossing", "same_direction_slow", "temporary_block"}
    ),
}
EXPERIMENT_ARMS = frozenset({"off", "sr_medium", "dr_medium", "medium"})
COMMAND_ZERO_TOLERANCE = 1.0e-3
TERMINAL_ZERO_CADENCE_TOLERANCE_SEC = 0.10
# Module2 is 5 Hz and the global costmap can publish at 1--2 Hz.  Two seconds
# covers one post-retirement source plus a processed status from both consumers
# without relaxing any source-stamp, sequence, identity, or zero-cell gate.
DYNAMIC_RETIREMENT_CLEAR_TIMEOUT_SEC = 2.0

COMMON_REQUIRED_RECORDED_TOPICS = (
    "/clock",
    "/ground_truth/odom",
    "/odom",
    "/bio_nav/module1/odom",
    "/tf",
    "/cmd_vel_sim",
    "/simulation/collision",
    "/simulation/reset_stop_gate/status",
)
ROUTE_GUIDED_REQUIRED_RECORDED_TOPICS = (
    "/bio_nav/navigation_graph",
    "/bio_nav/canonical_route",
    "/bio_nav/route_progress",
)
SCENE_REQUIRED_RECORDED_TOPICS = {
    "indoor": ("/amcl_pose",),
    # Outdoor localization is a fixed map->odom owner.  AMCL is deliberately
    # not a required source for this scene.  The runner's fresh transform
    # observation, rather than /tf_static, proves map->odom availability.
    "outdoor": (),
}
SCENE_FORBIDDEN_RECORDED_TOPICS = {
    "indoor": (),
    "outdoor": ("/amcl_pose",),
}
FINAL_V6_SCENARIO_IDS = frozenset(
    f"{prefix}_{category}"
    for prefix in ("v6_final_kujiale", "final_rivermark")
    for category in ("static", "dynamic", "appearance")
)
KNOWN_OUTDOOR_LOCALIZATION_CONFLICTS = frozenset({"amcl", "slam_toolbox"})


def _module2_readiness_required(scenario_id: str, configured: object) -> bool:
    is_final = scenario_id in FINAL_V6_SCENARIO_IDS
    if isinstance(configured, str) and configured.strip().lower() == "auto":
        return is_final
    required = _boolean_parameter(configured, "require_module2_planning_ready")
    if is_final and not required:
        raise ConfigurationError(
            "final V6 scenarios cannot disable Module2 planning readiness"
        )
    return required


def _localization_node_ownership_evidence(
    scene: str, node_names: list[tuple[str, str]] | list[str]
) -> dict[str, Any]:
    basenames: list[str] = []
    for entry in node_names:
        name = entry[0] if isinstance(entry, tuple) else entry
        if isinstance(name, str) and name.strip("/"):
            basenames.append(name.strip("/").rsplit("/", 1)[-1])
    counts = {name: basenames.count(name) for name in sorted(set(basenames))}
    forbidden = sorted(
        name for name in KNOWN_OUTDOOR_LOCALIZATION_CONFLICTS if counts.get(name, 0)
    )
    applicable = scene == "outdoor"
    passed = bool(
        not applicable
        or (counts.get("ideal_localization_tf", 0) == 1 and not forbidden)
    )
    return {
        "scene": scene,
        "applicable": applicable,
        "node_basenames": sorted(basenames),
        "basename_counts": counts,
        "required_owner": "ideal_localization_tf" if applicable else None,
        "required_owner_count": counts.get("ideal_localization_tf", 0),
        "known_forbidden_basenames": sorted(KNOWN_OUTDOOR_LOCALIZATION_CONFLICTS),
        "observed_forbidden_basenames": forbidden,
        "passed": passed,
        "scope": "known_localization_nodes_not_arbitrary_tf_publishers",
    }


def _evidence_scene(scenario_id: str, map_version: str) -> str:
    identity = f"{scenario_id} {map_version}".lower()
    return "outdoor" if "rivermark" in identity else "indoor"


def _mcap_required_topic_coverage(
    metadata_path: Path,
    *,
    scene: str,
    route_guided: bool = False,
    recorder_error: str | None = None,
) -> dict[str, Any]:
    """Read rosbag metadata and require only contract-critical topic samples."""

    if scene not in SCENE_REQUIRED_RECORDED_TOPICS:
        raise ValueError(f"unsupported evidence scene: {scene}")
    required_topics = tuple(
        dict.fromkeys(
            COMMON_REQUIRED_RECORDED_TOPICS
            + (ROUTE_GUIDED_REQUIRED_RECORDED_TOPICS if route_guided else ())
            + SCENE_REQUIRED_RECORDED_TOPICS[scene]
        )
    )
    forbidden_topics = SCENE_FORBIDDEN_RECORDED_TOPICS[scene]
    counts: dict[str, int] = {}
    metadata_error: str | None = None
    if not metadata_path.is_file():
        metadata_error = "metadata_missing"
    else:
        try:
            document = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
            information = document.get("rosbag2_bagfile_information", {})
            rows = information.get("topics_with_message_count", [])
            if not isinstance(rows, list):
                raise ValueError("topics_with_message_count must be a list")
            for row in rows:
                topic_metadata = row.get("topic_metadata", {})
                name = topic_metadata.get("name")
                count = row.get("message_count")
                if (
                    not isinstance(name, str)
                    or not isinstance(count, int)
                    or count < 0
                ):
                    raise ValueError("invalid topic metadata row")
                counts[name] = counts.get(name, 0) + count
        except (OSError, AttributeError, TypeError, ValueError, yaml.YAMLError) as exc:
            metadata_error = f"metadata_invalid:{type(exc).__name__}:{exc}"
    missing = [topic for topic in required_topics if topic not in counts]
    empty = [topic for topic in required_topics if counts.get(topic) == 0]
    observed_forbidden = [
        topic for topic in forbidden_topics if counts.get(topic, 0) > 0
    ]
    return {
        "scene": scene,
        "metadata_path": str(metadata_path),
        "metadata_present": metadata_path.is_file(),
        "metadata_error": metadata_error,
        "recorder_error": recorder_error,
        "required_topics": list(required_topics),
        "message_counts": {
            topic: counts.get(topic, 0) for topic in required_topics
        },
        "missing_topics": missing,
        "zero_message_topics": empty,
        "forbidden_topics": list(forbidden_topics),
        "forbidden_message_counts": {
            topic: counts.get(topic, 0) for topic in forbidden_topics
        },
        "observed_forbidden_topics": observed_forbidden,
        "passed": bool(
            not recorder_error
            and not metadata_error
            and not missing
            and not empty
            and not observed_forbidden
        ),
    }


def _route_prior_application_evidence(
    route_edge_costs: list[dict[str, Any]], *, required: bool
) -> dict[str, Any]:
    requested = 0
    applied = 0
    request_ids: set[int] = set()
    for record in route_edge_costs:
        try:
            request_ids.add(int(record["request_id"]))
        except (KeyError, TypeError, ValueError):
            pass
        edges = record.get("edges", [])
        if not isinstance(edges, list):
            continue
        for edge in edges:
            if not isinstance(edge, Mapping):
                continue
            requested_value = edge.get("requested_module2_delta_m")
            applied_value = edge.get("applied_module2_delta_m")
            if isinstance(requested_value, (int, float)) and requested_value > 0.0:
                requested += 1
            if isinstance(applied_value, (int, float)) and applied_value > 0.0:
                applied += 1
    return {
        "required": required,
        "positive_requested_count": requested,
        "positive_applied_count": applied,
        "request_ids": sorted(request_ids),
        "confirmed": not required or (requested > 0 and applied > 0),
    }


def _episode_validity(summary: Mapping[str, Any]) -> dict[str, Any]:
    reasons: list[str] = []
    coverage = summary.get("required_topic_coverage", {})
    route_prior = summary.get("route_prior_application", {})
    if summary.get("terminal_zero_confirmed") is not True:
        reasons.append("terminal_zero_not_confirmed")
    reset_receipt = summary.get("reset_receipt", {})
    if (
        not isinstance(reset_receipt, Mapping)
        or not reset_receipt
        or summary.get("reset_receipt_confirmed") is not True
    ):
        reasons.append("reset_receipt_missing")
    if summary.get("contact_sensor_evidence_confirmed") is not True:
        reasons.append("contact_sensor_evidence_missing")
    if summary.get("fixed_map_to_odom_evidence_confirmed") is not True:
        reasons.append("fixed_map_to_odom_evidence_missing")
    if summary.get("data_complete") is not True:
        reasons.append("data_incomplete")
    if summary.get("checksums_verified") is not True:
        reasons.append("checksums_unverified")
    if isinstance(coverage, Mapping) and coverage.get("required") is True:
        if coverage.get("passed") is not True:
            reasons.append("required_topic_coverage_incomplete")
    if isinstance(route_prior, Mapping) and route_prior.get("required") is True:
        if route_prior.get("confirmed") is not True:
            reasons.append("route_prior_application_unconfirmed")
    stack = summary.get("condition_stack_attestation", {})
    if isinstance(stack, Mapping) and stack.get("required") is True:
        if stack.get("confirmed") is not True:
            reasons.append("condition_stack_attestation_missing")
    reasons = list(dict.fromkeys(reasons))
    return {
        "valid": not reasons,
        "status": "valid" if not reasons else "invalid",
        "invalid_reasons": reasons,
    }


def _finalize_summary_acceptance(summary: dict[str, Any]) -> None:
    validity = _episode_validity(summary)
    summary["episode_validity"] = validity
    summary["strict_success"] = bool(
        summary.get("navigation_contract_success") is True
        and summary.get("physical_collision_free") is True
        and summary.get("route_prior_application_confirmed") is True
        and validity["valid"]
    )


def _strict_success_from_leg_count(
    result: object,
    leg_count: int,
    route_pose_count: int,
    *,
    terminal_zero_confirmed: bool,
) -> bool:
    """Bind strict success to the actual dispatch contract."""

    expected_leg_count = route_pose_count or 1
    return (
        result == "success"
        and leg_count == expected_leg_count
        and terminal_zero_confirmed
    )


def _parse_obstacle_completion(
    payload: str,
    *,
    expected_group: str,
    expected_ids: set[str],
) -> tuple[str, ...]:
    try:
        decoded = json.loads(payload)
    except json.JSONDecodeError as exc:
        raise RuntimeError(
            f"obstacle completion returned invalid JSON for {expected_group}: {payload!r}"
        ) from exc
    if not isinstance(decoded, dict) or decoded.get("group") != expected_group:
        raise RuntimeError(
            f"obstacle completion group mismatch for {expected_group}: {decoded!r}"
        )
    retired_raw = decoded.get("retired")
    if not isinstance(retired_raw, list) or not all(
        isinstance(identifier, str) and identifier for identifier in retired_raw
    ):
        raise RuntimeError(
            f"obstacle completion returned invalid retired IDs for {expected_group}: "
            f"{retired_raw!r}"
        )
    retired = tuple(retired_raw)
    if len(retired) != len(set(retired)) or set(retired) != expected_ids:
        raise RuntimeError(
            f"obstacle completion retired IDs mismatch for {expected_group}: "
            f"expected={sorted(expected_ids)!r}, actual={sorted(set(retired))!r}"
        )
    return retired


def _result_with_terminal_zero(
    reasons: list[str], terminal_zero_confirmed: bool
) -> tuple[str, list[str]]:
    """Make actuator terminal safety part of the manifest result."""

    combined = list(reasons)
    if not terminal_zero_confirmed:
        combined.append("terminal_zero_not_confirmed")
    combined = list(dict.fromkeys(combined))
    return ("success" if not combined else "failure"), combined


def _record_tracked_route_length(
    routes: list[dict[str, Any]], request_id: int, arc_length_m: float,
    remaining_m: float,
) -> None:
    """Replace full canonical-edge length with the trimmed Route geometry."""

    tracked_length_m = float(arc_length_m) + float(remaining_m)
    if not math.isfinite(tracked_length_m) or tracked_length_m < 0.0:
        return
    for route in reversed(routes):
        if int(route.get("request_id", -1)) != int(request_id):
            continue
        previous = route.get("planned_length_m")
        route.setdefault("canonical_full_edge_length_m", previous)
        measured = route.get("tracked_route_length_m")
        if not isinstance(measured, (int, float)) or tracked_length_m > float(measured):
            route["tracked_route_length_m"] = tracked_length_m
            route["planned_length_m"] = tracked_length_m
        return


def _edge_prior_statistics(priors: list[Any]) -> dict[str, float | int]:
    costs = [max(0.0, float(item.cost_delta_m)) for item in priors]
    risks = [max(0.0, float(item.learned_risk)) for item in priors]
    return {
        "prior_count": len(priors),
        "positive_cost_count": sum(value > 0.0 for value in costs),
        "total_cost_delta_m": sum(costs),
        "maximum_cost_delta_m": max(costs, default=0.0),
        "maximum_learned_risk": max(risks, default=0.0),
    }


def _diagnostic_float(value: Any) -> float | None:
    """Encode diagnostic-only NaN/Inf as JSON null, never as a cost value."""
    parsed = float(value)
    return parsed if math.isfinite(parsed) else None


def _pregoal_identity(scenario_id: str, run_index: int, selection: RunSelection) -> dict[str, object]:
    """Canonical, evidence-only identity for a fenced experiment route."""

    return {
        "scenario_id": str(scenario_id),
        "run_index": int(run_index),
        "seed": int(selection.seed),
        "condition_id": str(selection.condition_id),
        "appearance_profile_id": str(selection.appearance_profile_id or ""),
        # The audited contract reserves this field for dynamic actor variants.
        # Bind it from the scenario identity as well as the ordinary
        # ``dynamic_*`` condition namespace: authorization-only conditions use
        # a stage-prefixed name but still run the frozen dynamic actor contract.
        # Static matrices may still use a local scenario variant such as v1,
        # which is not a dynamic-runtime identity.
        "dynamic_variant_id": (
            str(selection.variant_id or "")
            if (
                str(selection.condition_id).startswith("dynamic_")
                or str(scenario_id).endswith("_dynamic")
            )
            else ""
        ),
    }


def validate_pregoal_authorization(
    path: Path, *, scenario_id: str, run_index: int, selection: RunSelection,
    expected_receipt: str = PREGOAL_AUTHORIZATION_RECEIPT,
    expected_schema: str = "", expected_campaign: str = "",
    expected_prereg_sha256: str = "",
) -> dict[str, object]:
    """Load an explicitly version-bound authorization without trusting a launcher flag."""

    if not path.is_file():
        raise ConfigurationError("pre-goal authorization receipt is missing")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ConfigurationError("pre-goal authorization receipt is invalid") from exc
    if not isinstance(value, dict):
        raise ConfigurationError("pre-goal authorization receipt must be an object")
    if value.get("pass") is not True or value.get("receipt") != expected_receipt:
        raise ConfigurationError("pre-goal authorization receipt is not passing")
    for key, expected in (("schema", expected_schema), ("campaign", expected_campaign), ("prereg_sha256", expected_prereg_sha256)):
        if expected and value.get(key) != expected:
            raise ConfigurationError(f"pre-goal authorization {key} mismatch")
    identity = value.get("identity")
    if identity != _pregoal_identity(scenario_id, run_index, selection):
        raise ConfigurationError("pre-goal authorization identity mismatch")
    completed_wall_ns = value.get("completed_wall_ns")
    if not isinstance(completed_wall_ns, int) or completed_wall_ns <= 0:
        raise ConfigurationError("pre-goal authorization completion timestamp is invalid")
    return value


def _boolean_parameter(value: object, name: str) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str) and value.strip().lower() in {"true", "false"}:
        return value.strip().lower() == "true"
    raise ConfigurationError(f"{name} must be boolean")


def _positive_finite_float(value: object, name: str) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise ConfigurationError(f"{name} must be positive and finite") from exc
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise ConfigurationError(f"{name} must be positive and finite")
    return parsed


def _reset_dynamic_selection(
    scenario_type: str, selection: RunSelection
) -> tuple[str | None, str | None]:
    """Only dynamic scenarios may select a dynamic-obstacle state machine."""

    if scenario_type == "dynamic":
        return selection.case_id, selection.variant_id
    return None, None


def _dynamic_interaction_acceptance(
    *,
    scenario_type: str,
    expected_ids: set[str],
    triggered_ids: set[str],
    completed_ids: set[str],
    retired_ids: set[str],
    clearance_by_actor: Mapping[str, float],
    evidence_complete: bool,
    maximum_pairing_clearance_m: float | None = None,
) -> dict[str, bool | float | str]:
    """Evaluate dynamic evidence under the collision-free acceptance policy.

    Clearance remains mandatory evidence and values below 0.10 m remain a
    report warning. They are not a failure by themselves: the physical
    collision topic is the authoritative contact gate.
    """
    if scenario_type != "dynamic":
        return {
            "complete": True,
            "minimum_clearance_complete": True,
            "clearance_warning_below_0_10m": False,
            "minimum_clearance_requirement_m": 0.0,
            "maximum_pairing_clearance_m": 0.0,
            "close_interaction_complete": True,
            "acceptance_policy": "not_applicable",
        }
    if not expected_ids:
        return {
            "complete": False,
            "minimum_clearance_complete": False,
            "clearance_warning_below_0_10m": False,
            "minimum_clearance_requirement_m": 0.0,
            "maximum_pairing_clearance_m": (
                float(maximum_pairing_clearance_m)
                if maximum_pairing_clearance_m is not None
                else 0.0
            ),
            "close_interaction_complete": False,
            "acceptance_policy": (
                "physical_collision_free_and_close_pairing"
                if maximum_pairing_clearance_m is not None
                else "physical_collision_free"
            ),
            "reason": "expected_dynamic_actor_ids_empty",
        }
    clearance_observed = expected_ids <= set(clearance_by_actor)
    close_interaction_complete = bool(
        maximum_pairing_clearance_m is None
        or (
            clearance_observed
            and all(
                float(clearance_by_actor[identifier])
                <= maximum_pairing_clearance_m
                for identifier in expected_ids
            )
        )
    )
    clearance_warning = (
        clearance_observed
        and any(float(value) < 0.10 for value in clearance_by_actor.values())
    )
    return {
        "complete": bool(
            expected_ids <= triggered_ids
            and expected_ids <= completed_ids
            and expected_ids <= retired_ids
            and clearance_observed
            and close_interaction_complete
            and evidence_complete
        ),
        # Preserve the historical field for report compatibility. Under the
        # collision-free policy it means every actor supplied a finite,
        # non-negative observation, not that clearance exceeded 0.10 m.
        "minimum_clearance_complete": bool(
            clearance_observed
            and all(float(value) >= 0.0 for value in clearance_by_actor.values())
        ),
        "clearance_warning_below_0_10m": bool(clearance_warning),
        "minimum_clearance_requirement_m": 0.0,
        "maximum_pairing_clearance_m": (
            float(maximum_pairing_clearance_m)
            if maximum_pairing_clearance_m is not None
            else 0.0
        ),
        "close_interaction_complete": close_interaction_complete,
        "acceptance_policy": (
            "physical_collision_free_and_close_pairing"
            if maximum_pairing_clearance_m is not None
            else "physical_collision_free"
        ),
    }


class ExperimentIsolationError(RuntimeError):
    """Raised when an old action may still contaminate the next trial."""


def _command_output(arguments: list[str], cwd: Path) -> str | None:
    try:
        result = subprocess.run(arguments, cwd=cwd, check=True, capture_output=True, text=True, timeout=10.0)
    except (OSError, subprocess.SubprocessError):
        return None
    return result.stdout.strip() or None


def _campaign_provenance(workspace: Path, map_version: str, posegraph_version: str) -> dict[str, Any]:
    """Capture enough immutable context to reproduce a formal run later."""
    status = _command_output(["git", "status", "--porcelain=v1"], workspace) or ""
    map_root = workspace / "data" / "maps"
    map_files = [map_root / "occupancy" / f"{map_version}{suffix}" for suffix in (".yaml", ".pgm")]
    posegraph_dir = map_root / "posegraphs"
    posegraph_files = sorted(posegraph_dir.glob(f"{posegraph_version}*")) if posegraph_dir.is_dir() else []
    hashes = {
        str(path.relative_to(workspace)): configuration_sha256(path)
        for path in [*map_files, *posegraph_files]
        if path.is_file()
    }
    return {
        "git_branch": _command_output(["git", "branch", "--show-current"], workspace),
        "git_head": _command_output(["git", "rev-parse", "HEAD"], workspace),
        "git_status_sha256": hashlib.sha256(status.encode("utf-8")).hexdigest(),
        "git_dirty": bool(status),
        "map_and_posegraph_hashes": hashes,
        "host": {"platform": platform.platform(), "python": platform.python_version()},
        "environment": {key: os.environ.get(key) for key in ("ROS_DISTRO", "RMW_IMPLEMENTATION", "ISAAC_SIM_VERSION") if os.environ.get(key)},
    }


def _yaw_from_quaternion(quaternion: Any) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def _sample_from_odometry(message: Odometry) -> OdometrySample | None:
    stamp_s = message.header.stamp.sec + message.header.stamp.nanosec * 1.0e-9
    values = (
        message.pose.pose.position.x,
        message.pose.pose.position.y,
        _yaw_from_quaternion(message.pose.pose.orientation),
        message.twist.twist.linear.x,
        message.twist.twist.angular.z,
        stamp_s,
    )
    if not all(math.isfinite(value) for value in values):
        return None
    return OdometrySample(*values, received_at=time.monotonic())


class ExperimentRunner(Node):
    """Sequential runner; ground truth is sampled but never republished or controlled from."""

    def __init__(self) -> None:
        super().__init__("experiment_runner")
        scenario_file = str(self.declare_parameter("scenario_file", "").value).strip()
        if not scenario_file:
            raise ConfigurationError("scenario_file is required")
        self._scenario: Scenario = load_scenario(scenario_file)
        self._condition_stack_id = str(
            self.declare_parameter("condition_stack_id", "").value
        ).strip()
        self._stack_session_id = str(
            self.declare_parameter("stack_session_id", "").value
        ).strip()
        self._formal_freeze_digest = str(
            self.declare_parameter("formal_freeze_digest", "").value
        ).strip()
        if bool(self._condition_stack_id) != bool(self._stack_session_id):
            raise ConfigurationError(
                "condition_stack_id and stack_session_id must be supplied together"
            )
        if self._condition_stack_id:
            scene = _evidence_scene(
                self._scenario.scenario_id, self._scenario.map_version
            )
            category = (
                "appearance"
                if self._scenario.appearance_config_file is not None
                else self._scenario.scenario_type
            )
            expected_condition_stack_id = f"{scene}_{category}"
            if self._condition_stack_id != expected_condition_stack_id:
                raise ConfigurationError(
                    "condition_stack_id does not match the scenario identity"
                )
            if len(self._stack_session_id) != 64 or any(
                character not in "0123456789abcdef"
                for character in self._stack_session_id
            ):
                raise ConfigurationError(
                    "stack_session_id must be a lowercase SHA-256 digest"
                )
        if bool(self._formal_freeze_digest) != bool(self._condition_stack_id):
            raise ConfigurationError(
                "formal_freeze_digest requires condition stack attestation"
            )
        if self._formal_freeze_digest and (
            len(self._formal_freeze_digest) != 64
            or any(
                character not in "0123456789abcdef"
                for character in self._formal_freeze_digest
            )
        ):
            raise ConfigurationError(
                "formal_freeze_digest must be a lowercase SHA-256 digest"
            )
        visual_case = str(self.declare_parameter("dynamic_case_id", "").value).strip()
        visual_variant = str(self.declare_parameter("dynamic_variant_id", "").value).strip()
        visual_seed = self.declare_parameter("dynamic_seed", 0).value
        if visual_case or visual_variant:
            if not (visual_case and visual_variant) or isinstance(visual_seed, bool) or not isinstance(visual_seed, int) or visual_seed < 0:
                raise ConfigurationError("dynamic visual override requires case_id, variant_id and non-negative seed")
            self._scenario = replace(self._scenario, seeds=(visual_seed,), run_matrix=(RunSelection(visual_seed, visual_case, visual_variant),))
        validate_navigation_runner_scenario(self._scenario)

        configured_spawn_file = str(
            self.declare_parameter("spawn_poses_file", "").value
        ).strip()
        spawn_file = configured_spawn_file or os.environ.get(
            "ISAAC_NAV_SPAWN_POSES", ""
        ).strip()
        if not spawn_file:
            raise ConfigurationError(
                "spawn_poses_file is required (or set ISAAC_NAV_SPAWN_POSES)"
            )
        self._spawn_pose: SpawnPose = load_spawn_pose(
            spawn_file,
            self._scenario.spawn_pose_name,
            require_calibrated=True,
        )

        robot_override = str(self.declare_parameter("robot_config_file", "").value).strip()
        nav2_override = str(self.declare_parameter("nav2_config_file", "").value).strip()
        self._nav2_profile = str(
            self.declare_parameter("nav2_profile", "").value
        ).strip()
        self._experiment_arm = str(
            self.declare_parameter("experiment_arm", "").value
        ).strip()
        if self._experiment_arm and self._experiment_arm not in EXPERIMENT_ARMS:
            raise ConfigurationError(
                f"experiment_arm must be one of {sorted(EXPERIMENT_ARMS)}"
            )
        if (
            self._scenario.appearance_config_file is not None
            and self._nav2_profile not in APPEARANCE_NAV2_PROFILES
        ):
            raise ConfigurationError(
                "appearance benchmark requires a registered appearance-safe "
                "Nav2 profile"
            )
        robot_config = (
            Path(robot_override).expanduser().resolve()
            if robot_override
            else self._scenario.resolve_path(self._scenario.robot_config_file)
        )
        nav2_config = (
            Path(nav2_override).expanduser().resolve()
            if nav2_override
            else self._scenario.resolve_path(self._scenario.nav2_config_file)
        )
        self._robot_config_hash = configuration_sha256(robot_config)
        self._robot_footprint = load_robot_footprint(robot_config)
        self._nav2_config_hash = configuration_sha256(nav2_config)
        self._workspace_root = Path(__file__).resolve().parents[4]
        self._provenance = _campaign_provenance(
            self._workspace_root, self._scenario.map_version, self._scenario.posegraph_version
        )
        self._dynamic_config_hash = None
        if self._scenario.dynamic_config_file is not None:
            dynamic_config = self._scenario.resolve_path(
                self._scenario.dynamic_config_file
            )
            validate_dynamic_physical_contract(
                self._scenario, self._spawn_pose, dynamic_config
            )
            self._dynamic_config_hash = configuration_sha256(dynamic_config)
        self._appearance_config_hash: str | None = None
        if self._scenario.appearance_config_file is not None:
            appearance_config = self._scenario.resolve_path(
                self._scenario.appearance_config_file
            )
            if not appearance_config.is_file():
                raise ConfigurationError(
                    f"appearance configuration does not exist: {appearance_config}"
                )
            self._appearance_config_hash = configuration_sha256(appearance_config)
        self._optimal_reference: Mapping[str, Any] | None = None
        self._optimal_reference_hash: str | None = None
        if self._scenario.optimal_reference_file is not None:
            reference_path = self._scenario.resolve_path(
                self._scenario.optimal_reference_file
            )
            try:
                reference = json.loads(reference_path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError) as exc:
                raise ConfigurationError(
                    f"optimal reference is invalid: {reference_path}: {exc}"
                ) from exc
            if (
                not isinstance(reference, Mapping)
                or not isinstance(reference.get("total_length_m_0_05"), (int, float))
                or not isinstance(reference.get("legs"), list)
                or not reference.get("converged")
            ):
                raise ConfigurationError("optimal reference is incomplete or unconverged")
            self._optimal_reference = reference
            self._optimal_reference_hash = configuration_sha256(reference_path)

        self._output_directory = Path(
            str(self.declare_parameter("output_directory", "data/experiment_runs").value)
        ).expanduser()
        requested_indices = str(
            self.declare_parameter("run_indices", "").value
        ).strip()
        if requested_indices:
            try:
                indices = tuple(
                    int(item.strip()) for item in requested_indices.split(",")
                )
            except ValueError as exc:
                raise ConfigurationError(
                    "run_indices must be comma-separated positive integers"
                ) from exc
            if not indices or min(indices) <= 0 or len(set(indices)) != len(indices):
                raise ConfigurationError(
                    "run_indices must be unique positive integers"
                )
            self._run_indices: tuple[int, ...] | None = indices
        else:
            self._run_indices = None
        self._resume = _boolean_parameter(
            self.declare_parameter("resume", False).value, "resume"
        )
        self._require_successful_resume = _boolean_parameter(
            self.declare_parameter("require_successful_resume", False).value,
            "require_successful_resume",
        )
        self._fail_stop = _boolean_parameter(
            self.declare_parameter("fail_stop", False).value,
            "fail_stop",
        )
        fail_stop_metric_contract = str(
            self.declare_parameter("fail_stop_metric_contract", "").value
        ).strip()
        self._fail_stop_metric_contract = (
            Path(fail_stop_metric_contract).expanduser().resolve()
            if fail_stop_metric_contract
            else None
        )
        if (
            self._fail_stop_metric_contract is not None
            and not self._fail_stop
        ):
            raise ConfigurationError(
                "fail_stop_metric_contract requires fail_stop"
            )
        if (
            self._fail_stop_metric_contract is not None
            and not self._fail_stop_metric_contract.is_file()
        ):
            raise ConfigurationError(
                "fail_stop metric contract does not exist"
            )
        self._record_evidence = _boolean_parameter(
            self.declare_parameter("record_evidence", True).value,
            "record_evidence",
        )
        self._record_bag = _boolean_parameter(
            self.declare_parameter("record_bag", True).value,
            "record_bag",
        )
        self._clear_slam_localization_buffer = _boolean_parameter(
            self.declare_parameter(
                "clear_slam_localization_buffer", True
            ).value,
            "clear_slam_localization_buffer",
        )
        self._require_module2_planning_ready = _module2_readiness_required(
            self._scenario.scenario_id,
            self.declare_parameter(
                "require_module2_planning_ready", "auto"
            ).value,
        )
        self._module2_planning_ready_timeout_sec = float(
            self.declare_parameter(
                "module2_planning_ready_timeout_sec", 30.0
            ).value
        )
        if self._module2_planning_ready_timeout_sec <= 0.0:
            raise ConfigurationError(
                "module2_planning_ready_timeout_sec must be positive"
            )
        if self._record_bag and not self._record_evidence:
            raise ConfigurationError(
                "record_bag requires record_evidence"
            )
        self._require_pregoal_authorization = _boolean_parameter(
            self.declare_parameter("require_pregoal_authorization", False).value,
            "require_pregoal_authorization",
        )
        self._authorization_only = _boolean_parameter(
            self.declare_parameter("authorization_only", False).value,
            "authorization_only",
        )
        self._pregoal_expected_receipt = str(
            self.declare_parameter("pregoal_expected_receipt", PREGOAL_AUTHORIZATION_RECEIPT).value
        ).strip()
        self._pregoal_expected_schema = str(
            self.declare_parameter("pregoal_expected_schema", "").value
        ).strip()
        self._pregoal_expected_campaign = str(
            self.declare_parameter("pregoal_expected_campaign", "").value
        ).strip()
        self._pregoal_expected_prereg_sha256 = str(
            self.declare_parameter("pregoal_expected_prereg_sha256", "").value
        ).strip()
        authorization_path = str(
            self.declare_parameter("pregoal_authorization_path", "").value
        ).strip()
        lifecycle_path = str(
            self.declare_parameter("lifecycle_jsonl_path", "").value
        ).strip()
        self._pregoal_authorization_path = (
            Path(authorization_path).expanduser().resolve()
            if authorization_path
            else None
        )
        self._lifecycle_jsonl_path = (
            Path(lifecycle_path).expanduser().resolve() if lifecycle_path else None
        )
        self._pregoal_authorization_sha256: str | None = None
        self._active_run_index: int | None = None
        self._goal_dispatch_recorded = False
        if self._require_pregoal_authorization:
            if self._pregoal_authorization_path is None:
                raise ConfigurationError("pre-goal authorization path is required")
            if self._lifecycle_jsonl_path is None:
                raise ConfigurationError("lifecycle JSONL path is required")
            if self._lifecycle_jsonl_path.exists():
                raise ConfigurationError("lifecycle JSONL must not reuse an existing file")
            if self._run_indices is None or len(self._run_indices) != 1:
                raise ConfigurationError("pre-goal authorization requires exactly one run index")
            if not self._pregoal_expected_receipt:
                raise ConfigurationError("pre-goal expected receipt is required")
        elif self._authorization_only:
            raise ConfigurationError("authorization_only requires pre-goal authorization")
        self._reset_service_name = str(
            self.declare_parameter("reset_service", "/simulation/reset").value
        )
        self._action_name = str(
            self.declare_parameter("navigate_action", "/navigate_to_pose").value
        )
        self._navigation_execution_backend = str(
            self.declare_parameter(
                "navigation_execution_backend", "navigate_to_pose"
            ).value
        ).strip()
        if self._navigation_execution_backend not in {
            "navigate_to_pose",
            "route_guided",
        }:
            raise ConfigurationError(
                "navigation_execution_backend must be navigate_to_pose or route_guided"
            )
        self._service_timeout_sec = float(
            self.declare_parameter("service_timeout_sec", 30.0).value
        )
        self._clock_timeout_sec = float(
            self.declare_parameter("clock_timeout_sec", 30.0).value
        )
        self._odom_max_age_sec = float(
            self.declare_parameter("odom_max_age_sec", 0.5).value
        )
        self._tf_gap_tolerance_sec = float(
            self.declare_parameter("tf_gap_tolerance_sec", 1.0).value
        )
        self._collision_lock_timeout_sec = float(
            self.declare_parameter("collision_lock_timeout_sec", 5.0).value
        )
        self._reset_recovery_timeout_sec = float(
            self.declare_parameter("reset_recovery_timeout_sec", 30.0).value
        )
        self._localization_seed_event_grace_sec = float(
            self.declare_parameter(
                "localization_seed_event_grace_sec", 2.0
            ).value
        )
        self._reset_tf_stability_sec = float(
            self.declare_parameter("reset_tf_stability_sec", 0.5).value
        )
        self._reset_tf_translation_tolerance_m = float(
            self.declare_parameter(
                "reset_tf_translation_tolerance_m", 0.05
            ).value
        )
        self._reset_map_base_translation_tolerance_m = _positive_finite_float(
            self.declare_parameter(
                "reset_map_base_translation_tolerance_m", 0.05
            ).value,
            "reset_map_base_translation_tolerance_m",
        )
        self._reset_tf_yaw_tolerance_rad = float(
            self.declare_parameter(
                "reset_tf_yaw_tolerance_rad", math.radians(3.0)
            ).value
        )
        self._nav2_active_stability_sec = float(
            self.declare_parameter("nav2_active_stability_sec", 1.0).value
        )
        if min(
            self._service_timeout_sec,
            self._clock_timeout_sec,
            self._odom_max_age_sec,
            self._tf_gap_tolerance_sec,
            self._collision_lock_timeout_sec,
            self._reset_recovery_timeout_sec,
            self._localization_seed_event_grace_sec,
            self._reset_tf_stability_sec,
            self._reset_tf_translation_tolerance_m,
            self._reset_tf_yaw_tolerance_rad,
            self._nav2_active_stability_sec,
        ) <= 0.0:
            raise ConfigurationError("runner timeouts must be positive")

        reliable = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        clock_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        command_observation_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._clock_ready = False
        self._clock_stamp = None
        self._clock_subscription = self.create_subscription(
            Clock, "/clock", self._clock_callback, clock_qos
        )
        self._gt_subscription = self.create_subscription(
            Odometry,
            str(self.declare_parameter("ground_truth_topic", "/ground_truth/odom").value),
            self._ground_truth_callback,
            reliable,
        )
        self._odom_subscription = self.create_subscription(
            Odometry,
            str(self.declare_parameter("odom_topic", "/odom").value),
            self._odom_callback,
            reliable,
        )
        self._command_subscription = self.create_subscription(
            Twist,
            str(self.declare_parameter("command_topic", "/cmd_vel").value),
            self._command_callback,
            reliable,
        )
        self._actuator_command_subscription = self.create_subscription(
            Twist,
            "/cmd_vel_sim",
            self._actuator_command_callback,
            command_observation_qos,
        )
        self._status_subscriptions = [
            self.create_subscription(
                Bool,
                str(self.declare_parameter("collision_topic", "/simulation/collision").value),
                self._collision_callback,
                reliable,
            ),
            self.create_subscription(
                CollisionMonitorState,
                str(
                    self.declare_parameter(
                        "collision_monitor_state_topic", "/collision_monitor_state"
                    ).value
                ),
                self._collision_lock_callback,
                reliable,
            ),
            self.create_subscription(
                EmptyMessage,
                str(
                    self.declare_parameter(
                        "localization_seeded_topic",
                        "/simulation/localization_seeded",
                    ).value
                ),
                self._localization_seeded_callback,
                reliable,
            ),
        ]
        self._obstacle_state_subscription = self.create_subscription(
            String,
            "/experiment/obstacles/state",
            self._obstacle_state_callback,
            reliable,
        )
        self._cognitive_obstacle_subscription = self.create_subscription(
            CognitiveObstacleArray,
            "/bio_nav/module2/cognitive_obstacles",
            self._cognitive_obstacle_callback,
            reliable,
        )
        self._cognitive_layer_status_subscription = self.create_subscription(
            RiskLayerStatus,
            "/bio_nav/cognitive_obstacle_layer/status",
            self._cognitive_layer_status_callback,
            reliable,
        )
        self._appearance_state_subscription = self.create_subscription(
            String,
            "/experiment/appearance/state",
            self._appearance_state_callback,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        # The RGB-D renderer publishes best-effort sensor data.  Keep a
        # latest-frame evidence snapshot in addition to the selected MCAP so
        # every formal run remains reviewable without a ROS player.
        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._depth_subscription = self.create_subscription(
            Image,
            "/camera/front/depth/image_raw",
            self._depth_callback,
            sensor_qos,
        )
        self._rgb_subscription = self.create_subscription(
            Image,
            "/camera/front/image_raw",
            self._rgb_callback,
            sensor_qos,
        )
        self._scan_subscription = self.create_subscription(
            LaserScan, "/scan", self._scan_callback, sensor_qos
        )
        self._safety_scan_subscription = self.create_subscription(
            LaserScan,
            "/scan_safety",
            self._safety_scan_callback,
            sensor_qos,
        )
        self._local_costmap_subscription = self.create_subscription(
            Costmap,
            "/local_costmap/costmap_raw",
            self._local_costmap_callback,
            reliable,
        )
        self._global_costmap_subscription = self.create_subscription(
            Costmap,
            "/global_costmap/costmap_raw",
            self._global_costmap_callback,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )
        route_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._reset_stop_gate_status = None
        self._reset_stop_gate_status_error: str | None = None
        self._reset_stop_gate_status_received = False
        self._reset_call_barrier_monotonic: float | None = None
        self._reset_stop_gate_status_subscription = self.create_subscription(
            String,
            "/simulation/reset_stop_gate/status",
            self._reset_stop_gate_status_callback,
            route_qos,
        )
        self._route_goal_publisher = self.create_publisher(
            PoseStamped, "/bio_nav/route_goal", reliable
        )
        self._navigation_graph_subscription = self.create_subscription(
            NavigationGraph,
            "/bio_nav/navigation_graph",
            self._navigation_graph_callback,
            route_qos,
        )
        self._canonical_route_subscription = self.create_subscription(
            CanonicalRoute,
            "/bio_nav/canonical_route",
            self._canonical_route_callback,
            route_qos,
        )
        self._route_goal_complete_subscription = self.create_subscription(
            Bool,
            "/bio_nav/route_goal_complete",
            self._route_goal_complete_callback,
            reliable,
        )
        self._edge_prior_subscription = self.create_subscription(
            EdgePriorArray,
            "/bio_nav/module2/edge_priors",
            self._edge_prior_callback,
            route_qos,
        )
        self._planning_prior_subscription = self.create_subscription(
            PlanningPrior,
            "/bio_nav/module2/planning_prior",
            self._planning_prior_callback,
            reliable,
        )
        self._srdr_edge_diagnostic_subscription = self.create_subscription(
            SRDREdgeDiagnosticArray,
            "/bio_nav/module2/srdr_edge_diagnostics",
            self._srdr_edge_diagnostic_callback,
            route_qos,
        )
        self._route_edge_cost_subscription = self.create_subscription(
            RouteEdgeCostArray,
            "/bio_nav/route_edge_costs",
            self._route_edge_cost_callback,
            route_qos,
        )
        self._route_progress_subscription = self.create_subscription(
            RouteProgress,
            "/bio_nav/route_progress",
            self._route_progress_callback,
            reliable,
        )
        self._smac_plan_subscription = self.create_subscription(
            NavPath,
            "/plan",
            self._smac_plan_callback,
            reliable,
        )
        self._reset_client = self.create_client(Trigger, self._reset_service_name)
        self._terminal_fence_arm_client = self.create_client(
            Trigger,
            "/bio_nav/route_coordinator/arm_next_terminal_fence",
        )
        self._localization_buffer_client = (
            self.create_client(
                Empty, "/slam_toolbox/clear_localization_buffer"
            )
            if self._clear_slam_localization_buffer
            else None
        )
        self._isaac_parameter_client = AsyncParameterClient(
            self,
            str(
                self.declare_parameter(
                    "isaac_node_name", "/isaac_navigation_sim"
                ).value
            ),
        )
        self._navigate_client = ActionClient(self, NavigateToPose, self._action_name)
        self._cancel_navigation_client = self.create_client(
            CancelGoal, f"{self._action_name}/_action/cancel_goal"
        )
        self._obstacle_trigger_clients: dict[str, object] = {}
        self._obstacle_complete_clients: dict[str, object] = {}
        self._collision_monitor_state_client = self.create_client(
            GetState,
            str(
                self.declare_parameter(
                    "collision_monitor_state_service",
                    "/collision_monitor/get_state",
                ).value
            ),
        )
        managed_node_names = [
            "controller_server",
            "planner_server",
            "behavior_server",
            "velocity_smoother",
            "collision_monitor",
            "bt_navigator",
        ]
        if self._navigation_execution_backend == "route_guided":
            managed_node_names.insert(0, "route_server")
        self._nav2_managed_state_clients = tuple(
            (
                node_name,
                self.create_client(GetState, f"/{node_name}/get_state"),
            )
            for node_name in managed_node_names
        )
        self._costmap_clear_clients = (
            (
                "global costmap",
                self.create_client(
                    ClearEntireCostmap,
                    "/global_costmap/clear_entirely_global_costmap",
                ),
            ),
            (
                "local costmap",
                self.create_client(
                    ClearEntireCostmap,
                    "/local_costmap/clear_entirely_local_costmap",
                ),
            ),
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self._dynamic_runtime_contract: dict[str, Any] = {
            "verified": False,
        }
        self._appearance_runtime_contract: dict[str, Any] = {
            "verified": self._scenario.appearance_config_file is None,
        }
        self._appearance_state: dict[str, Any] | None = None
        self._localization_seed_epoch = 0
        self._collision_monitor_active = False
        self._active_evidence_root: Path | None = None
        self._bag_process: subprocess.Popen[bytes] | None = None
        self._active_selection: RunSelection | None = None
        self._reset_receipt: dict[str, Any] | None = None
        self._navigation_graph: NavigationGraph | None = None
        self._canonical_route_epoch = 0
        self._route_goal_complete_epoch = 0
        self._latest_route_goal_complete = False
        self._clear_run_state()

    def _clear_run_state(self) -> None:
        self._ground_truth_samples: list[OdometrySample] = []
        self._odom_samples: list[OdometrySample] = []
        self._command_samples: list[CommandSample] = []
        self._navigation_active = False
        self._navigation_start_stamp_s: float | None = None
        self._navigation_end_stamp_s: float | None = None
        self._terminal_zero_observation_started_monotonic: float | None = None
        self._terminal_zero_barrier_monotonic: float | None = None
        self._terminal_zero_barrier_source = "not_observed"
        self._terminal_zero_barrier_leg_id: str | None = None
        self._terminal_zero_expected_route_completion_epoch: int | None = None
        self._terminal_zero_expected_route_leg_id: str | None = None
        self._terminal_zero_expected_route_leg_is_final = False
        self._terminal_zero_confirmed_monotonic: float | None = None
        self._terminal_zero_first_zero_monotonic: float | None = None
        self._terminal_zero_last_zero_monotonic: float | None = None
        self._terminal_zero_confirming_sample_count = 0
        self._terminal_zero_confirmed = False
        self._terminal_zero_reason = "not_checked"
        self._cmd_vel_sim_last_receive_monotonic: float | None = None
        self._cmd_vel_sim_last_nonzero_monotonic: float | None = None
        self._cmd_vel_sim_zero_stamps: list[float] = []
        self._route_feedback_count = 0
        self._minimum_poses_remaining: int | None = None
        self._maximum_route_recoveries = 0
        self._collision_seen = False
        self._collision_detected = False
        self._isaac_contact_sensor_collision_detected = False
        self._localization_seen = False
        self._localization_lost = False
        self._lock_status_seen = False
        self._lock_started_at: float | None = None
        self._collision_monitor_locked = False
        self._tf_ever_available = False
        self._last_tf_stamp_s: float | None = None
        self._last_tf_ok_at: float | None = None
        self._tf_interrupted = False
        self._leg_results: list[dict[str, Any]] = []
        self._obstacle_events: list[dict[str, Any]] = []
        self._obstacle_event_keys: set[str] = set()
        self._completed_dynamic_obstacle_ids: set[str] = set()
        self._obstacle_state: dict[str, Any] = {"obstacles": [], "events": []}
        self._obstacle_state_stamp_s: float | None = None
        self._obstacle_samples: list[dict[str, Any]] = []
        self._latest_cognitive_obstacles: CognitiveObstacleArray | None = None
        self._latest_cognitive_layer_statuses: dict[str, RiskLayerStatus] = {}
        self._dynamic_guard_aborted = False
        self._dynamic_safety_yield = False
        self._rgb_frame: dict[str, Any] | None = None
        self._appearance_rgb_snapshot_complete = False
        self._depth_frame: dict[str, Any] | None = None
        self._scan_frame: dict[str, Any] | None = None
        self._safety_scan_frame: dict[str, Any] | None = None
        self._local_costmap: Costmap | None = None
        self._global_costmap: Costmap | None = None
        self._minimum_safety_scan_range_m: float | None = None
        self._canonical_routes: list[dict[str, Any]] = []
        self._module2_prior_responses: list[dict[str, Any]] = []
        self._planning_prior_samples: list[dict[str, Any]] = []
        self._latest_planning_prior_readiness: dict[str, Any] | None = None
        self._planning_prior_ready_streak = 0
        self._last_planning_field_stamp_s: float | None = None
        self._srdr_edge_diagnostics: list[dict[str, Any]] = []
        self._route_edge_costs: list[dict[str, Any]] = []
        self._route_progress_samples: list[dict[str, Any]] = []
        self._smac_plans: list[dict[str, Any]] = []
        self._goal_dispatch_recorded = False

    def _lifecycle_event(self, event: str) -> None:
        """Append an immutable phase record for a topology-fenced run."""

        if not self._require_pregoal_authorization:
            return
        if (
            self._lifecycle_jsonl_path is None
            or self._active_selection is None
            or self._active_run_index is None
        ):
            raise ConfigurationError("lifecycle context is unavailable")
        value = {
            "event": str(event),
            "wall_ns": time.time_ns(),
            "monotonic_ns": time.monotonic_ns(),
            "ros_stamp_s": self._clock_seconds(),
            "identity": _pregoal_identity(
                self._scenario.scenario_id,
                self._active_run_index,
                self._active_selection,
            ),
            "pregoal_authorization_sha256": self._pregoal_authorization_sha256,
            # This records the value that the installed runner actually
            # consumed.  A launcher argument alone is not sufficient
            # evidence for authorization-only gates.
            "nav2_profile": self._nav2_profile,
            "experiment_arm": self._experiment_arm or None,
            "authorization_only": self._authorization_only,
        }
        try:
            self._lifecycle_jsonl_path.parent.mkdir(parents=True, exist_ok=True)
            with self._lifecycle_jsonl_path.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(value, sort_keys=True) + "\n")
                stream.flush()
                os.fsync(stream.fileno())
        except OSError as exc:
            raise ConfigurationError("lifecycle JSONL write failed") from exc

    def _record_trial_dispatched(self) -> None:
        """Persist the trial boundary immediately before the first goal write."""

        if self._goal_dispatch_recorded:
            return
        if not self._record_evidence:
            self._lifecycle_event("goal_dispatched")
            self._goal_dispatch_recorded = True
            return
        if (
            self._active_evidence_root is None
            or self._active_selection is None
            or self._active_run_index is None
        ):
            raise ConfigurationError(
                "trial dispatch requires an active evidence identity"
            )
        target = self._active_evidence_root / "TRIAL_DISPATCHED.json"
        if target.exists():
            raise ConfigurationError("trial dispatch receipt already exists")
        receipt = {
            "schema": "bio_nav.trial_dispatched.v1",
            "scenario_id": self._scenario.scenario_id,
            "run_index": self._active_run_index,
            "seed": self._active_selection.seed,
            "condition_id": self._active_selection.condition_id,
            "dynamic_case_id": self._active_selection.case_id,
            "dynamic_variant_id": self._active_selection.variant_id,
            "experiment_arm": self._experiment_arm or None,
            "navigation_execution_backend": self._navigation_execution_backend,
            "condition_stack_id": getattr(self, "_condition_stack_id", "") or None,
            "stack_session_id": getattr(self, "_stack_session_id", "") or None,
            "formal_freeze_digest": getattr(self, "_formal_freeze_digest", "") or None,
            "wall_time_utc": datetime.now(timezone.utc).isoformat(),
            "wall_ns": time.time_ns(),
            "monotonic_ns": time.monotonic_ns(),
            "ros_stamp_s": self._clock_seconds(),
        }
        try:
            with target.open("x", encoding="utf-8") as stream:
                stream.write(
                    json.dumps(receipt, indent=2, sort_keys=True) + "\n"
                )
                stream.flush()
                os.fsync(stream.fileno())
        except OSError as exc:
            raise ConfigurationError(
                "trial dispatch receipt write failed"
            ) from exc
        self._lifecycle_event("goal_dispatched")
        self._goal_dispatch_recorded = True

    def _clock_callback(self, message: Clock) -> None:
        if message.clock.sec != 0 or message.clock.nanosec != 0:
            self._clock_ready = True
            self._clock_stamp = message.clock

    def _ground_truth_callback(self, message: Odometry) -> None:
        sample = _sample_from_odometry(message)
        if sample is not None:
            self._ground_truth_samples.append(sample)

    def _odom_callback(self, message: Odometry) -> None:
        sample = _sample_from_odometry(message)
        if sample is not None:
            self._odom_samples.append(sample)

    def _command_callback(self, message: Twist) -> None:
        stamp_s = self._clock_seconds()
        values = (
            message.linear.x,
            message.angular.z,
            stamp_s,
        )
        if (
            self._navigation_active
            and stamp_s is not None
            and all(math.isfinite(value) for value in values)
        ):
            self._command_samples.append(
                CommandSample(
                    linear_speed_mps=float(message.linear.x),
                    angular_speed_radps=float(message.angular.z),
                    stamp_s=stamp_s,
                )
            )

    def _actuator_command_callback(self, message: Twist) -> None:
        """Observe executor-side commands without publishing or changing control."""

        now = time.monotonic()
        values = (
            float(message.linear.x),
            float(message.linear.y),
            float(message.angular.z),
        )
        self._cmd_vel_sim_last_receive_monotonic = now
        nonzero = not all(math.isfinite(value) for value in values) or any(
            abs(value) > COMMAND_ZERO_TOLERANCE for value in values
        )
        if nonzero:
            self._cmd_vel_sim_last_nonzero_monotonic = now
            self._cmd_vel_sim_zero_stamps.clear()
            if (
                self._terminal_zero_barrier_monotonic is not None
                and now > self._terminal_zero_barrier_monotonic
            ):
                self._terminal_zero_confirmed = False
                self._terminal_zero_reason = "terminal_nonzero_after_barrier"
                self._terminal_zero_confirmed_monotonic = None
                self._terminal_zero_first_zero_monotonic = None
                self._terminal_zero_last_zero_monotonic = None
                self._terminal_zero_confirming_sample_count = 0
        else:
            self._cmd_vel_sim_zero_stamps.append(now)

    def _collision_callback(self, message: Bool) -> None:
        self._collision_seen = True
        detected = bool(message.data)
        self._isaac_contact_sensor_collision_detected = (
            self._isaac_contact_sensor_collision_detected or detected
        )
        self._collision_detected = self._collision_detected or detected

    def _collision_lock_callback(self, message: CollisionMonitorState) -> None:
        self._lock_status_seen = True
        stopped = message.action_type == CollisionMonitorState.STOP
        if stopped and self._lock_started_at is None:
            self._lock_started_at = time.monotonic()
        elif not stopped:
            self._lock_started_at = None

    def _localization_seeded_callback(self, message: EmptyMessage) -> None:
        del message
        self._localization_seed_epoch += 1

    def _obstacle_state_callback(self, message: String) -> None:
        """Deduplicate Isaac's periodic state snapshots into evidence events."""
        try:
            state = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if not isinstance(state, dict):
            return
        obstacles = state.get("obstacles")
        events = state.get("events")
        if not isinstance(obstacles, list) or not isinstance(events, list):
            return
        self._obstacle_state = {"obstacles": obstacles, "events": events}
        stamp_s = self._clock_seconds()
        self._obstacle_state_stamp_s = stamp_s
        if stamp_s is not None:
            for obstacle in obstacles:
                if isinstance(obstacle, dict):
                    self._obstacle_samples.append({"stamp_s": stamp_s, **obstacle})
        for event in events:
            if not isinstance(event, dict):
                continue
            key = json.dumps(event, sort_keys=True, separators=(",", ":"))
            if key not in self._obstacle_event_keys:
                self._obstacle_event_keys.add(key)
                self._obstacle_events.append(dict(event))
                if event.get("event") == "near_contact_abort":
                    self._dynamic_guard_aborted = True
                if event.get("event") == "safety_yield":
                    self._dynamic_safety_yield = True

    def _cognitive_obstacle_callback(self, message: CognitiveObstacleArray) -> None:
        self._latest_cognitive_obstacles = message

    def _cognitive_layer_status_callback(self, message: RiskLayerStatus) -> None:
        consumer = str(message.consumer)
        if consumer:
            self._latest_cognitive_layer_statuses[consumer] = message

    def _appearance_state_callback(self, message: String) -> None:
        try:
            state = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if not isinstance(state, dict):
            return
        profile_id = state.get("profile_id")
        config_hash = state.get("config_sha256")
        if not isinstance(profile_id, str) or not isinstance(config_hash, str):
            return
        self._appearance_state = state

    def _rgb_callback(self, message: Image) -> None:
        self._rgb_frame = {
            "width": int(message.width), "height": int(message.height),
            "encoding": str(message.encoding), "is_bigendian": int(message.is_bigendian),
            "step": int(message.step),
            "stamp_s": message.header.stamp.sec + message.header.stamp.nanosec * 1.0e-9,
            "data": bytes(message.data),
        }

    def _depth_callback(self, message: Image) -> None:
        # Copy the message bytes: ROS message instances may be reused by the
        # middleware after this callback returns.
        self._depth_frame = {
            "width": int(message.width), "height": int(message.height),
            "encoding": str(message.encoding), "is_bigendian": int(message.is_bigendian),
            "step": int(message.step), "stamp_s": message.header.stamp.sec + message.header.stamp.nanosec * 1.0e-9,
            "data": bytes(message.data),
        }

    @staticmethod
    def _scan_value(message: LaserScan) -> dict[str, Any]:
        return {
            "frame_id": str(message.header.frame_id),
            "angle_min": float(message.angle_min), "angle_increment": float(message.angle_increment),
            "range_min": float(message.range_min), "range_max": float(message.range_max),
            "stamp_s": message.header.stamp.sec + message.header.stamp.nanosec * 1.0e-9,
            "ranges": [float(value) for value in message.ranges],
        }

    def _scan_callback(self, message: LaserScan) -> None:
        self._scan_frame = self._scan_value(message)

    def _safety_scan_callback(self, message: LaserScan) -> None:
        self._safety_scan_frame = self._scan_value(message)
        valid = [
            float(value)
            for value in message.ranges
            if math.isfinite(float(value))
            and float(message.range_min) <= float(value) <= float(message.range_max)
        ]
        if self._navigation_active and valid:
            sample_minimum = min(valid)
            self._minimum_safety_scan_range_m = (
                sample_minimum
                if self._minimum_safety_scan_range_m is None
                else min(self._minimum_safety_scan_range_m, sample_minimum)
            )

    def _navigation_graph_callback(self, message: NavigationGraph) -> None:
        self._navigation_graph = message

    def _canonical_route_callback(self, message: CanonicalRoute) -> None:
        self._canonical_route_epoch += 1
        graph = self._navigation_graph
        edge_lengths = (
            {int(edge.id): float(edge.length_m) for edge in graph.edges}
            if graph is not None
            and str(graph.graph_id) == str(message.graph_id)
            and int(graph.revision) == int(message.graph_revision)
            else {}
        )
        edge_ids = [int(value) for value in message.edge_ids]
        missing = [edge_id for edge_id in edge_ids if edge_id not in edge_lengths]
        planned_length = (
            sum(edge_lengths[edge_id] for edge_id in edge_ids)
            if edge_ids and not missing
            else None
        )
        if self._navigation_active:
            self._canonical_routes.append(
                {
                    "request_id": int(message.request_id),
                    "graph_id": str(message.graph_id),
                    "graph_revision": int(message.graph_revision),
                    "node_ids": [int(value) for value in message.node_ids],
                    "edge_ids": edge_ids,
                    "planned_length_m": planned_length,
                    "missing_edge_ids": missing,
                    "route_cost_m": float(message.total_cost_m),
                }
            )

    def _route_goal_complete_callback(self, message: Bool) -> None:
        self._route_goal_complete_epoch += 1
        self._latest_route_goal_complete = bool(message.data)
        expected_epoch = self._terminal_zero_expected_route_completion_epoch
        if expected_epoch != self._route_goal_complete_epoch:
            return
        leg_id = self._terminal_zero_expected_route_leg_id
        final_leg = self._terminal_zero_expected_route_leg_is_final
        self._terminal_zero_expected_route_completion_epoch = None
        self._terminal_zero_expected_route_leg_id = None
        self._terminal_zero_expected_route_leg_is_final = False
        if not bool(message.data) or final_leg:
            self._mark_terminal_zero_barrier(
                "route_goal_complete", leg_id=leg_id
            )

    def _reset_stop_gate_status_callback(self, message: String) -> None:
        self._reset_stop_gate_status_received = True
        try:
            self._reset_stop_gate_status = parse_reset_stop_gate_status(
                message.data,
                received_at=time.monotonic(),
            )
            self._reset_stop_gate_status_error = None
        except RuntimeError as exc:
            self._reset_stop_gate_status_error = str(exc)

    def _edge_prior_callback(self, message: EdgePriorArray) -> None:
        if not self._navigation_active:
            return
        self._module2_prior_responses.append(
            {
                "request_id": int(message.request_id),
                "graph_id": str(message.graph_id),
                "graph_revision": int(message.graph_revision),
                "healthy": bool(message.healthy),
                "model_id": str(message.model_id),
                **_edge_prior_statistics(list(message.priors)),
            }
        )

    def _planning_prior_callback(self, message: PlanningPrior) -> None:
        stamp_s = float(message.stamp.sec) + float(message.stamp.nanosec) / 1.0e9
        place = [float(value) for value in message.place_belief]
        dynamic = [float(value) for value in message.dynamic_cost]
        self._latest_planning_prior_readiness = {
            "stamp_s": stamp_s,
            "module2_healthy": bool(message.module2_healthy),
            "place_entropy_normalized": float(
                message.place_entropy_normalized
            ),
            "context_uncertainty": float(message.context_uncertainty),
        }
        if self._latest_planning_prior_readiness["module2_healthy"]:
            self._planning_prior_ready_streak += 1
        else:
            self._planning_prior_ready_streak = 0
        if not self._navigation_active:
            return
        scalar = {
            "stamp_s": stamp_s,
            "sequence": int(message.sequence),
            "model_id": str(message.model_id),
            "map_version": str(message.map_version),
            "cognitive_tile_id": str(message.cognitive_tile_id),
            "tile_revision": int(message.tile_revision),
            "graph_revision": int(message.graph_revision),
            "active_slot_id": int(message.active_slot_id),
            "module2_healthy": bool(message.module2_healthy),
            "trusted_write": bool(message.trusted_write),
            "place_peak": max(place) if place else 0.0,
            "place_argmax": int(max(range(len(place)), key=place.__getitem__)) if place else -1,
            "place_entropy_normalized": float(message.place_entropy_normalized),
            "full_posterior_mean_canvas_m_diagnostic": [
                float(value) for value in message.place_mean_canvas_m
            ],
            "dominant_mode_root_state_id": int(
                message.dominant_mode_root_state_id
            ),
            "dominant_mode_mass": float(message.dominant_mode_mass),
            "dominant_mode_expected_xy_m": [
                float(value) for value in message.dominant_mode_expected_xy_m
            ],
            "dominant_mode_covariance_m2": [
                float(value) for value in message.dominant_mode_covariance_m2
            ],
            "dominant_mode_ellipse_1sigma": {
                "semi_major_axis_m": float(
                    message.dominant_mode_ellipse_1sigma_semi_major_axis_m
                ),
                "semi_minor_axis_m": float(
                    message.dominant_mode_ellipse_1sigma_semi_minor_axis_m
                ),
                "yaw_rad": float(message.dominant_mode_ellipse_1sigma_yaw_rad),
            },
            "full_posterior_covariance_m2_diagnostic": [
                float(value)
                for value in message.full_posterior_covariance_m2_diagnostic
            ],
            "dynamic_presence_probability": float(
                message.dynamic_presence_probability
            ),
            "risk_exposure_rate": float(
                sum(probability * cost for probability, cost in zip(place, dynamic))
            ),
            "context_uncertainty": float(message.context_uncertainty),
        }
        # Keep every scalar sample for time integration, but only retain the
        # bulk state and mode-candidate fields at 5 s cadence. This is sufficient
        # for heatmaps without inflating each run manifest.
        if (
            self._last_planning_field_stamp_s is None
            or stamp_s - self._last_planning_field_stamp_s >= 5.0
        ):
            scalar.update(
                {
                    "place_belief": place,
                    "value_sr": [float(value) for value in message.value_sr],
                    "future_cost_dr": [
                        float(value) for value in message.future_cost_dr
                    ],
                    "dynamic_cost": dynamic,
                    "remap_rates": [
                        float(value) for value in message.remap_rates
                    ],
                    "transient_suppression": [
                        float(value) for value in message.transient_suppression
                    ],
                    "place_top_k": [
                        {
                            "state_id": int(candidate.state_id),
                            "probability": float(candidate.probability),
                            "canvas_xy_m": [
                                float(value) for value in candidate.canvas_xy_m
                            ],
                            "mode_root_state_id": int(
                                candidate.mode_root_state_id
                            ),
                            "mode_state_count": int(candidate.mode_state_count),
                            "mode_mass": float(candidate.mode_mass),
                            "mode_expected_xy_m": [
                                float(value)
                                for value in candidate.mode_expected_xy_m
                            ],
                            "mode_covariance_m2": [
                                float(value)
                                for value in candidate.mode_covariance_m2
                            ],
                            "mode_ellipse_1sigma": {
                                "semi_major_axis_m": float(
                                    candidate.mode_ellipse_1sigma_semi_major_axis_m
                                ),
                                "semi_minor_axis_m": float(
                                    candidate.mode_ellipse_1sigma_semi_minor_axis_m
                                ),
                                "yaw_rad": float(
                                    candidate.mode_ellipse_1sigma_yaw_rad
                                ),
                            },
                        }
                        for candidate in message.place_top_k
                    ],
                }
            )
            self._last_planning_field_stamp_s = stamp_s
        self._planning_prior_samples.append(scalar)

    def _srdr_edge_diagnostic_callback(
        self, message: SRDREdgeDiagnosticArray
    ) -> None:
        if not self._navigation_active:
            return
        self._srdr_edge_diagnostics.append(
            {
                "request_id": int(message.request_id),
                "graph_id": str(message.graph_id),
                "graph_revision": int(message.graph_revision),
                "model_id": str(message.model_id),
                "edges": [
                    {
                        "edge_id": int(item.edge_id),
                        "from_node": int(item.from_node),
                        "to_node": int(item.to_node),
                        "sample_count": int(item.sample_count),
                        "valid_sample_count": int(item.valid_sample_count),
                        "sample_coverage": float(item.sample_coverage),
                        "sr_score": _diagnostic_float(item.sr_score),
                        "sr_penalty_m": _diagnostic_float(item.sr_penalty_m),
                        "dr_score": _diagnostic_float(item.dr_score),
                        "dr_penalty_m": _diagnostic_float(item.dr_penalty_m),
                        "requested_delta_m": _diagnostic_float(
                            item.requested_delta_m
                        ),
                        "confidence": _diagnostic_float(item.confidence),
                        "usable": bool(item.usable),
                        "rejection_reason": str(item.rejection_reason),
                    }
                    for item in message.diagnostics
                ],
            }
        )

    def _route_edge_cost_callback(self, message: RouteEdgeCostArray) -> None:
        if not self._navigation_active:
            return
        self._route_edge_costs.append(
            {
                "request_id": int(message.request_id),
                "graph_id": str(message.graph_id),
                "graph_revision": int(message.graph_revision),
                "edges": [
                    {
                        "edge_id": int(item.edge_id),
                        "structural_cost_m": _diagnostic_float(
                            item.structural_cost_m
                        ),
                        "requested_module2_delta_m": _diagnostic_float(
                            item.requested_module2_delta_m
                        ),
                        "applied_module2_delta_m": _diagnostic_float(
                            item.applied_module2_delta_m
                        ),
                        "runtime_penalty_m": _diagnostic_float(
                            item.runtime_penalty_m
                        ),
                        "final_cost_m": _diagnostic_float(item.final_cost_m),
                        "blocked": bool(item.blocked),
                    }
                    for item in message.costs
                ],
            }
        )

    def _route_progress_callback(self, message: RouteProgress) -> None:
        if not self._navigation_active:
            return
        _record_tracked_route_length(
            self._canonical_routes,
            int(message.request_id),
            float(message.arc_length_m),
            float(message.remaining_m),
        )
        self._route_progress_samples.append(
            {
                "request_id": int(message.request_id),
                "edge_id": int(message.edge_id),
                "edge_index": int(message.edge_index),
                "arc_length_m": float(message.arc_length_m),
                "lateral_error_m": _diagnostic_float(message.lateral_error_m),
                "remaining_m": float(message.remaining_m),
                "projected_point": [
                    _diagnostic_float(message.projected_point.x),
                    _diagnostic_float(message.projected_point.y),
                ],
                "lookahead": [
                    float(message.lookahead_goal.pose.position.x),
                    float(message.lookahead_goal.pose.position.y),
                ],
            }
        )

    def _smac_plan_callback(self, message: NavPath) -> None:
        if not self._navigation_active or not message.poses:
            return
        self._smac_plans.append(
            {
                "frame_id": str(message.header.frame_id),
                "points": [
                    [float(pose.pose.position.x), float(pose.pose.position.y)]
                    for pose in message.poses
                ],
            }
        )

    def _local_costmap_callback(self, message: Costmap) -> None:
        self._local_costmap = message

    def _global_costmap_callback(self, message: Costmap) -> None:
        self._global_costmap = message

    def _global_costmap_covers_mission(self) -> bool:
        """Reject Nav2's temporary 100x100 default map before goal dispatch."""

        message = self._global_costmap
        if message is None or str(message.header.frame_id) != "map":
            return False
        metadata = message.metadata
        resolution = float(metadata.resolution)
        width = int(metadata.size_x)
        height = int(metadata.size_y)
        origin_x = float(metadata.origin.position.x)
        origin_y = float(metadata.origin.position.y)
        values = (resolution, origin_x, origin_y)
        if (
            width <= 0 or height <= 0 or resolution <= 0.0
            or not all(math.isfinite(value) for value in values)
        ):
            return False
        maximum_x = origin_x + width * resolution
        maximum_y = origin_y + height * resolution
        points = [self._spawn_pose.map.position]
        points.extend(specification.position for specification in self._scenario.route)
        if not self._scenario.route:
            points.append(self._scenario.goal.position)
        return all(
            origin_x <= float(point[0]) < maximum_x
            and origin_y <= float(point[1]) < maximum_y
            for point in points
        )

    @staticmethod
    def _raise_if_shutdown() -> None:
        if not rclpy.ok():
            raise ExternalShutdownException()

    def _clock_seconds(self) -> float | None:
        if self._clock_stamp is None:
            return None
        return self._clock_stamp.sec + self._clock_stamp.nanosec * 1.0e-9

    @staticmethod
    def _transform_stamp_seconds(transform: Any) -> float:
        stamp = transform.header.stamp
        return stamp.sec + stamp.nanosec * 1.0e-9

    def _lookup_fresh_map_to_odom(self) -> tuple[Any, float] | None:
        try:
            transform = self._tf_buffer.lookup_transform("map", "odom", Time())
        except TransformException:
            return None
        clock_s = self._clock_seconds()
        stamp_s = self._transform_stamp_seconds(transform)
        if clock_s is None or stamp_s <= 0.0:
            return None
        age_s = clock_s - stamp_s
        # SLAM Toolbox intentionally future-dates map->odom by
        # transform_timeout; bound the absolute gap instead of rejecting that
        # valid publication policy.
        if abs(age_s) > self._tf_gap_tolerance_sec:
            return None
        return transform, stamp_s

    def _latest_map_to_odom_stamp(self) -> float | None:
        try:
            transform = self._tf_buffer.lookup_transform("map", "odom", Time())
        except TransformException:
            return None
        stamp_s = self._transform_stamp_seconds(transform)
        return stamp_s if math.isfinite(stamp_s) and stamp_s > 0.0 else None

    def _update_health(self) -> None:
        now = time.monotonic()
        if self._lock_started_at is not None:
            if now - self._lock_started_at >= self._collision_lock_timeout_sec:
                self._collision_monitor_locked = True
        fresh_transform = self._lookup_fresh_map_to_odom()
        if fresh_transform is not None:
            _, stamp_s = fresh_transform
            self._localization_seen = True
            self._tf_ever_available = True
            if self._last_tf_stamp_s != stamp_s:
                self._last_tf_stamp_s = stamp_s
                self._last_tf_ok_at = now
        else:
            if self._tf_ever_available and self._last_tf_ok_at is not None:
                if now - self._last_tf_ok_at >= self._tf_gap_tolerance_sec:
                    self._tf_interrupted = True
                    self._localization_lost = True

    def _spin_once(self, timeout_sec: float = 0.05) -> None:
        rclpy.spin_once(self, timeout_sec=timeout_sec)
        self._update_health()

    def _wait_until(self, predicate, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while not predicate():
            if not rclpy.ok():
                raise ExternalShutdownException()
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False
            self._spin_once(min(0.1, remaining))
        return True

    def _wait_future(
        self,
        future,
        deadline: float,
        *,
        abort_on_dynamic_guard: bool = False,
    ) -> bool:
        while not future.done():
            if not rclpy.ok():
                raise ExternalShutdownException()
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False
            self._spin_once(min(0.1, remaining))
            if abort_on_dynamic_guard and self._dynamic_guard_aborted:
                return False
        return True

    def _set_reset_seed(
        self,
        seed: int,
        case_id: str | None = None,
        variant_id: str | None = None,
        appearance_profile_id: str | None = None,
    ) -> None:
        if not self._isaac_parameter_client.wait_for_services(
            timeout_sec=self._service_timeout_sec
        ):
            self._raise_if_shutdown()
            raise RuntimeError("Isaac reset parameter services are unavailable")
        parameters = [
            Parameter("reset_seed", value=seed),
            Parameter("reset_pose_name", value=self._scenario.spawn_pose_name),
            Parameter("dynamic_case_id", value=case_id or ""),
            Parameter("dynamic_variant_id", value=variant_id or ""),
        ]
        if appearance_profile_id is not None:
            parameters.append(Parameter("appearance_profile_id", value=appearance_profile_id))
        future = self._isaac_parameter_client.set_parameters(parameters)
        deadline = time.monotonic() + self._service_timeout_sec
        if not self._wait_future(future, deadline):
            raise TimeoutError("setting deterministic reset parameters timed out")
        response = future.result()
        if response is None:
            raise RuntimeError("setting deterministic reset parameters returned no response")
        failures = [result.reason for result in response.results if not result.successful]
        if failures:
            raise RuntimeError(f"Isaac rejected reset parameters: {failures}")
        if appearance_profile_id is not None:
            if not self._wait_until(
                lambda: self._appearance_state is not None
                and self._appearance_state.get("profile_id") == appearance_profile_id
                and self._appearance_state.get("config_sha256")
                == self._appearance_config_hash,
                self._service_timeout_sec,
            ):
                raise TimeoutError(
                    "timed out waiting for Isaac to publish the requested appearance profile state"
                )

    def _verify_appearance_runtime_contract(self) -> None:
        if self._scenario.appearance_config_file is None:
            return
        if not self._isaac_parameter_client.wait_for_services(
            timeout_sec=self._service_timeout_sec
        ):
            self._raise_if_shutdown()
            raise RuntimeError("Isaac appearance parameter services are unavailable")
        names = [
            "appearance_config_sha256",
            "appearance_inventory_sha256",
            "appearance_light_count",
            "appearance_material_color_input_count",
        ]
        future = self._isaac_parameter_client.get_parameters(names)
        if not self._wait_future(
            future, time.monotonic() + self._service_timeout_sec
        ):
            raise TimeoutError("reading Isaac appearance contract timed out")
        response = future.result()
        if response is None or len(response.values) != len(names):
            raise RuntimeError("Isaac returned an incomplete appearance contract")
        config_hash, inventory_hash, light_count, material_count = (
            parameter_value_to_python(value) for value in response.values
        )
        if config_hash != self._appearance_config_hash:
            raise RuntimeError(
                "Isaac appearance configuration hash does not match the scenario"
            )
        if (
            not isinstance(inventory_hash, str)
            or len(inventory_hash) != 64
            or not isinstance(light_count, int)
            or not isinstance(material_count, int)
            or light_count <= 0
            or material_count <= 0
        ):
            raise RuntimeError("Isaac appearance inventory contract is invalid")
        self._appearance_runtime_contract = {
            "verified": True,
            "config_sha256": config_hash,
            "inventory_sha256": inventory_hash,
            "light_count": light_count,
            "material_color_input_count": material_count,
        }

    def _verify_dynamic_runtime_contract(self) -> None:
        names = [
            "dynamic_obstacles_enabled",
            "dynamic_obstacles_config_sha256",
            "dynamic_obstacle_ids",
        ]
        # Isaac may advertise its parameter services before the first request
        # can be serviced after a cold simulation start.  Retry the read-only
        # verification before any reset or goal is issued; this is not a
        # navigation retry and preserves the pre-goal isolation boundary.
        response = None
        for attempt in range(3):
            if self._isaac_parameter_client.wait_for_services(
                timeout_sec=self._service_timeout_sec
            ):
                future = self._isaac_parameter_client.get_parameters(names)
                if self._wait_future(
                    future, time.monotonic() + self._service_timeout_sec
                ):
                    response = future.result()
                    break
            if attempt < 2:
                self.get_logger().warning(
                    "Isaac dynamic obstacle contract is not ready; retrying pre-goal verification"
                )
                self._spin_once(1.0)
        if response is None:
            self._raise_if_shutdown()
            raise TimeoutError("reading the Isaac dynamic obstacle contract timed out")
        if response is None or len(response.values) != len(names):
            raise RuntimeError(
                "Isaac returned an incomplete dynamic obstacle contract"
            )
        enabled, config_hash, obstacle_ids = (
            parameter_value_to_python(value) for value in response.values
        )
        if not isinstance(enabled, bool):
            raise RuntimeError("Isaac dynamic_obstacles_enabled is not boolean")
        if not isinstance(config_hash, str) or not config_hash:
            raise RuntimeError(
                "Isaac dynamic_obstacles_config_sha256 is invalid"
            )
        if (
            not isinstance(obstacle_ids, list)
            or not all(isinstance(value, str) for value in obstacle_ids)
        ):
            raise RuntimeError("Isaac dynamic_obstacle_ids is invalid")
        runtime_ids = tuple(obstacle_ids)
        validate_dynamic_runtime_contract(
            self._scenario,
            runtime_enabled=enabled,
            runtime_config_hash=config_hash,
            runtime_obstacle_ids=runtime_ids,
            expected_config_hash=self._dynamic_config_hash,
        )
        self._dynamic_runtime_contract = {
            "verified": True,
            "enabled": enabled,
            "config_sha256": config_hash,
            "obstacle_ids": list(runtime_ids),
        }

    def _verify_collision_monitor_active(self) -> None:
        """Require a stable ACTIVE monitor while tolerating lifecycle churn.

        A fresh runner can overlap the tail of the previous route's reset and
        shutdown work.  During that narrow window the lifecycle service may be
        discoverable while one GetState request never receives a response.
        Querying once for the full service timeout therefore creates a false
        formal failure.  Use short queries inside one bounded recovery window,
        and still fail closed unless ACTIVE is continuously observed.
        """

        deadline = time.monotonic() + self._reset_recovery_timeout_sec
        active_since: float | None = None
        latest_state = "service_unavailable"
        while rclpy.ok():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise TimeoutError(
                    "Collision Monitor did not become stably active before "
                    f"the recovery deadline: {latest_state}"
                )
            if not self._collision_monitor_state_client.wait_for_service(
                timeout_sec=min(0.1, remaining)
            ):
                latest_state = "service_unavailable"
                active_since = None
                self._spin_once(
                    min(0.1, max(0.0, deadline - time.monotonic()))
                )
                continue

            future = self._collision_monitor_state_client.call_async(
                GetState.Request()
            )
            query_deadline = min(deadline, time.monotonic() + 1.0)
            if not self._wait_future(future, query_deadline):
                future.cancel()
                latest_state = "query_timeout"
                active_since = None
                continue
            response = future.result()
            if response is None:
                latest_state = "no_response"
                active_since = None
                continue
            latest_state = response.current_state.label
            if response.current_state.id != State.PRIMARY_STATE_ACTIVE:
                active_since = None
                self._spin_once(
                    min(0.1, max(0.0, deadline - time.monotonic()))
                )
                continue
            if active_since is None:
                active_since = time.monotonic()
            elif (
                time.monotonic() - active_since
                >= self._nav2_active_stability_sec
            ):
                self._collision_monitor_active = True
                return
            self._spin_once(
                min(0.1, max(0.0, deadline - time.monotonic()))
            )
        raise ExternalShutdownException()

    def _wait_for_nav2_managed_nodes_active(self) -> None:
        """Wait for a stable active Nav2 set after a physical reset.

        Lifecycle services can report ``active`` a little before their bond
        heartbeats have resumed.  Requiring a short continuous active window
        prevents an action goal from racing that final recovery step.
        """

        deadline = time.monotonic() + self._reset_recovery_timeout_sec
        latest_states: dict[str, str] = {}
        active_since: float | None = None
        while rclpy.ok():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                state_summary = ", ".join(
                    f"{name}={latest_states.get(name, 'unavailable')}"
                    for name, _ in self._nav2_managed_state_clients
                )
                raise TimeoutError(
                    "simulation reset recovery timed out waiting for all Nav2 "
                    f"managed nodes to become active: {state_summary}"
                )

            services_ready = True
            for node_name, client in self._nav2_managed_state_clients:
                if not client.wait_for_service(timeout_sec=min(0.1, remaining)):
                    services_ready = False
                    latest_states[node_name] = "service_unavailable"
            if not services_ready:
                self._spin_once(min(0.1, max(0.0, deadline - time.monotonic())))
                continue

            futures = {
                node_name: client.call_async(GetState.Request())
                for node_name, client in self._nav2_managed_state_clients
            }
            query_deadline = min(deadline, time.monotonic() + 1.0)
            query_complete = True
            for future in futures.values():
                if not self._wait_future(future, query_deadline):
                    query_complete = False
            if query_complete:
                all_active = True
                for node_name, future in futures.items():
                    response = future.result()
                    if response is None:
                        latest_states[node_name] = "no_response"
                        all_active = False
                        continue
                    latest_states[node_name] = response.current_state.label
                    all_active = (
                        all_active
                        and response.current_state.id
                        == State.PRIMARY_STATE_ACTIVE
                    )
                if all_active:
                    if active_since is None:
                        active_since = time.monotonic()
                    elif (
                        time.monotonic() - active_since
                        >= self._nav2_active_stability_sec
                    ):
                        self._collision_monitor_active = True
                        return
                else:
                    active_since = None
            else:
                active_since = None
            self._spin_once(min(0.1, max(0.0, deadline - time.monotonic())))
        raise ExternalShutdownException()

    def _clear_navigation_costmaps(self) -> None:
        for label, client in self._costmap_clear_clients:
            if not client.wait_for_service(
                timeout_sec=self._service_timeout_sec
            ):
                self._raise_if_shutdown()
                raise RuntimeError(f"{label} clear service is unavailable")
            future = client.call_async(ClearEntireCostmap.Request())
            if not self._wait_future(
                future, time.monotonic() + self._service_timeout_sec
            ):
                raise TimeoutError(f"clearing {label} timed out")
            if future.result() is None:
                raise RuntimeError(f"clearing {label} returned no response")

    def _clear_localization_buffer(self) -> None:
        if not self._clear_slam_localization_buffer:
            return
        assert self._localization_buffer_client is not None
        if not self._localization_buffer_client.wait_for_service(
            timeout_sec=self._service_timeout_sec
        ):
            self._raise_if_shutdown()
            raise RuntimeError(
                "SLAM Toolbox localization buffer service is unavailable"
            )
        future = self._localization_buffer_client.call_async(Empty.Request())
        if not self._wait_future(
            future, time.monotonic() + self._service_timeout_sec
        ):
            raise TimeoutError("clearing the localization buffer timed out")
        if future.result() is None:
            raise RuntimeError(
                "clearing the localization buffer returned no response"
            )

    def _cancel_stale_navigation_goal(self) -> None:
        """Cancel every outstanding NavigateToPose goal before a physical reset.

        A runner may be interrupted while Nav2 is still executing.  Teleporting
        the robot and publishing one zero velocity is not an isolation boundary:
        the old action can publish again on the next controller cycle.
        """
        response = None
        for attempt in range(3):
            if self._cancel_navigation_client.wait_for_service(
                timeout_sec=self._service_timeout_sec
            ):
                request = CancelGoal.Request()
                request.goal_info = GoalInfo()
                future = self._cancel_navigation_client.call_async(request)
                if self._wait_future(
                    future, time.monotonic() + self._service_timeout_sec
                ):
                    response = future.result()
                    break
            if attempt < 2:
                self.get_logger().warning(
                    "NavigateToPose cancel service is not ready; retrying before reset"
                )
                self._spin_once(1.0)
        if response is None:
            self._raise_if_shutdown()
            raise TimeoutError("cancelling stale NavigateToPose goals timed out")
        accepted_codes = {
            CancelGoal.Response.ERROR_NONE,
            CancelGoal.Response.ERROR_REJECTED,
            CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID,
        }
        if response is None or response.return_code not in accepted_codes:
            detail = "no response" if response is None else str(response.return_code)
            raise RuntimeError(f"cancelling stale NavigateToPose goals failed: {detail}")

    def _wait_for_reset_recovery(
        self,
        tf_stamp_barrier_s: float | None,
        sample_stamp_barrier_s: float | None,
    ) -> None:
        deadline = time.monotonic() + self._reset_recovery_timeout_sec
        stable_since: float | None = None
        stable_anchor: tuple[float, float, float] | None = None
        last_status = "no recovery samples received"
        while rclpy.ok():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise TimeoutError(
                    "simulation reset recovery timed out waiting for the "
                    "spawn-aligned Ground Truth/odom/map->base state and "
                    f"stable map->odom; last_status={last_status}"
                )
            self._spin_once(min(0.1, remaining))
            now = time.monotonic()
            odom_ready = bool(
                self._odom_samples
                and now - self._odom_samples[-1].received_at <= self._odom_max_age_sec
            )
            ground_truth_ready = bool(
                self._ground_truth_samples
                and now - self._ground_truth_samples[-1].received_at
                <= self._odom_max_age_sec
            )
            if odom_ready:
                odom = self._odom_samples[-1]
                odom_ready = (
                    (
                        sample_stamp_barrier_s is None
                        or odom.stamp_s > sample_stamp_barrier_s
                    )
                    and
                    math.hypot(odom.x, odom.y)
                    <= self._reset_tf_translation_tolerance_m
                    and abs(wrap_angle(odom.yaw_rad))
                    <= self._reset_tf_yaw_tolerance_rad
                )
            if ground_truth_ready:
                ground_truth = self._ground_truth_samples[-1]
                expected_x, expected_y = self._spawn_pose.map.position
                expected_yaw = math.radians(
                    self._spawn_pose.map.yaw_deg
                )
                ground_truth_ready = (
                    (
                        sample_stamp_barrier_s is None
                        or ground_truth.stamp_s > sample_stamp_barrier_s
                    )
                    and math.hypot(
                        ground_truth.x - expected_x,
                        ground_truth.y - expected_y,
                    )
                    <= self._reset_tf_translation_tolerance_m
                    and abs(wrap_angle(
                        ground_truth.yaw_rad - expected_yaw
                    )) <= self._reset_tf_yaw_tolerance_rad
                )
            fresh_transform = self._lookup_fresh_map_to_odom()
            tf_ready = False
            transform_values: tuple[float, float, float] | None = None
            if fresh_transform is not None:
                transform, stamp_s = fresh_transform
                rotation = transform.transform.rotation
                yaw = math.atan2(
                    2.0 * (
                        rotation.w * rotation.z
                        + rotation.x * rotation.y
                    ),
                    1.0
                    - 2.0 * (
                        rotation.y * rotation.y
                        + rotation.z * rotation.z
                    ),
                )
                translation = transform.transform.translation
                transform_values = (translation.x, translation.y, yaw)
                tf_ready = (
                    tf_stamp_barrier_s is None
                    or stamp_s > tf_stamp_barrier_s
                )
            map_base_ready = False
            try:
                map_base = self._tf_buffer.lookup_transform(
                    "map", "base_link", Time()
                )
            except TransformException:
                map_base = None
            if map_base is not None:
                map_base_stamp = self._transform_stamp_seconds(map_base)
                clock_s = self._clock_seconds()
                rotation = map_base.transform.rotation
                map_base_yaw = math.atan2(
                    2.0 * (
                        rotation.w * rotation.z
                        + rotation.x * rotation.y
                    ),
                    1.0
                    - 2.0 * (
                        rotation.y * rotation.y
                        + rotation.z * rotation.z
                    ),
                )
                translation = map_base.transform.translation
                expected_x, expected_y = self._spawn_pose.map.position
                expected_yaw = math.radians(
                    self._spawn_pose.map.yaw_deg
                )
                map_base_ready = (
                    clock_s is not None
                    and map_base_stamp > 0.0
                    and abs(clock_s - map_base_stamp)
                    <= self._tf_gap_tolerance_sec
                    and math.hypot(
                        translation.x - expected_x,
                        translation.y - expected_y,
                    ) <= self._reset_map_base_translation_tolerance_m
                    and abs(wrap_angle(
                        map_base_yaw - expected_yaw
                    )) <= self._reset_tf_yaw_tolerance_rad
                )
            tf_ready = tf_ready and map_base_ready
            def pose_text(sample: OdometrySample | None) -> str:
                if sample is None:
                    return "none"
                return f"({sample.x:.3f},{sample.y:.3f},{math.degrees(sample.yaw_rad):.1f}deg)"
            map_base_text = "none"
            if map_base is not None:
                translation = map_base.transform.translation
                map_base_text = f"({translation.x:.3f},{translation.y:.3f},{math.degrees(map_base_yaw):.1f}deg)"
            last_status = (
                f"odom_ready={odom_ready} gt_ready={ground_truth_ready} "
                f"tf_ready={tf_ready} map_base_ready={map_base_ready} "
                f"odom={pose_text(self._odom_samples[-1] if self._odom_samples else None)} "
                f"gt={pose_text(self._ground_truth_samples[-1] if self._ground_truth_samples else None)} "
                f"map_base={map_base_text}"
            )
            if odom_ready and ground_truth_ready and tf_ready:
                if stable_anchor is None:
                    stable_anchor = transform_values
                    stable_since = now
                else:
                    assert transform_values is not None
                    translation_delta = math.hypot(
                        transform_values[0] - stable_anchor[0],
                        transform_values[1] - stable_anchor[1],
                    )
                    yaw_delta = abs(
                        wrap_angle(transform_values[2] - stable_anchor[2])
                    )
                    if (
                        translation_delta
                        > self._reset_tf_translation_tolerance_m
                        or yaw_delta > self._reset_tf_yaw_tolerance_rad
                    ):
                        stable_anchor = transform_values
                        stable_since = now
                if now - stable_since >= self._reset_tf_stability_sec:
                    return
            else:
                stable_since = None
                stable_anchor = None
        raise ExternalShutdownException()

    def _reset_simulation(
        self,
        seed: int,
        case_id: str | None = None,
        variant_id: str | None = None,
        appearance_profile_id: str | None = None,
    ) -> None:
        previous_seed_epoch = self._localization_seed_epoch
        self._cancel_stale_navigation_goal()
        self._clear_localization_buffer()
        self._set_reset_seed(seed, case_id, variant_id, appearance_profile_id)
        self._lifecycle_event("reset_requested")
        if not self._reset_client.wait_for_service(timeout_sec=self._service_timeout_sec):
            self._raise_if_shutdown()
            raise RuntimeError(f"reset service unavailable: {self._reset_service_name}")
        reset_call_barrier = time.monotonic()
        future = self._reset_client.call_async(Trigger.Request())
        deadline = time.monotonic() + self._service_timeout_sec
        if not self._wait_future(future, deadline):
            raise TimeoutError("simulation reset timed out")
        response = future.result()
        if response is None or not response.success:
            message = "no response" if response is None else response.message
            raise RuntimeError(f"simulation reset failed: {message}")
        receipt = parse_reset_receipt(
            response.message,
            requested_seed=seed,
            requested_case_id=case_id or "",
            requested_variant_id=variant_id or "",
        )
        self._lifecycle_event("reset_succeeded")
        self._clear_navigation_costmaps()
        self._clear_run_state()
        self._reset_receipt = receipt
        self._reset_call_barrier_monotonic = reset_call_barrier
        if not self._wait_until(
            lambda: self._localization_seed_epoch > previous_seed_epoch,
            self._localization_seed_event_grace_sec,
        ):
            # The notification is an observability receipt, not the actual
            # localization safety boundary.  DDS can occasionally drop this
            # one-shot volatile event even though /initialpose was consumed.
            # Continue to the strictly-new TF/sample gate below; it requires a
            # post-reset map->odom publication, spawn-aligned map->base, fresh
            # odom/Ground Truth, and a stable transform.  If initial seeding
            # genuinely did not occur, that objective gate still fails closed.
            self.get_logger().warning(
                "fresh-scan localization seed event was not observed within "
                f"{self._localization_seed_event_grace_sec:.1f}s; relying on "
                "the post-reset spawn-aligned TF/sample recovery gate"
            )
        # Snapshot after Isaac has published the initial-pose message.  A subsequent,
        # strictly newer map->odom publication is then required before the
        # pose/stability gate can pass.
        tf_stamp_barrier_s = self._latest_map_to_odom_stamp()
        sample_stamp_barrier_s = self._clock_seconds()
        self._wait_for_reset_recovery(
            tf_stamp_barrier_s,
            sample_stamp_barrier_s,
        )
        self._wait_for_nav2_managed_nodes_active()
        if not self._wait_until(
            self._global_costmap_covers_mission,
            self._reset_recovery_timeout_sec,
        ):
            raise TimeoutError(
                "simulation reset recovery timed out waiting for a full map-frame "
                "global costmap covering the complete mission"
            )
        if appearance_profile_id is not None and not self._wait_until(
            lambda: self._rgb_frame is not None,
            self._service_timeout_sec,
        ):
            raise TimeoutError("timed out waiting for post-reset RGB evidence frame")

    def _pose_message(self, specification) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = specification.frame_id
        if self._clock_stamp is not None:
            pose.header.stamp = self._clock_stamp
        pose.pose.position.x = specification.position[0]
        pose.pose.position.y = specification.position[1]
        yaw = math.radians(specification.yaw_deg)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _goal_message(self, specification=None) -> NavigateToPose.Goal:
        goal = NavigateToPose.Goal()
        goal.pose = self._pose_message(
            specification or self._scenario.goal
        )
        return goal

    def _navigation_feedback_callback(self, message) -> None:
        feedback = message.feedback
        self._route_feedback_count += 1
        self._maximum_route_recoveries = max(
            self._maximum_route_recoveries,
            int(feedback.number_of_recoveries),
        )

    def _selected_dynamic_groups_for_goal(self, goal_id: str | None) -> list[str]:
        """Return only obstacle groups enabled by the active case selection.

        Focused routes such as ``g2_g3_exit`` still travel through G2 before
        their own arm goal G3.  The scenario declares all physical actors for
        runtime-contract verification, but their trigger services must not be
        called for actors that this run deliberately left inactive.
        """
        if self._scenario.scenario_type != "dynamic" or not goal_id:
            return []
        return sorted({
            str(item["trigger_group"])
            for item in self._selected_dynamic_trajectories()
            if item.get("trigger_group") == goal_id
        })

    def _selected_dynamic_trajectories(self) -> tuple[Mapping[str, Any], ...]:
        if self._scenario.scenario_type != "dynamic":
            return ()
        selected_case = (
            self._active_selection.case_id
            if self._active_selection is not None
            else None
        )
        if selected_case is None:
            return tuple(self._scenario.obstacle_trajectories)
        selected_motions = DYNAMIC_CASE_SET_MOTIONS.get(
            selected_case,
            frozenset({selected_case}),
        )
        return tuple(
            item for item in self._scenario.obstacle_trajectories
            if item.get("motion") in selected_motions
        )

    def _validate_dynamic_episode_selection(self) -> None:
        if self._scenario.scenario_type != "dynamic":
            return
        selected = self._selected_dynamic_trajectories()
        groups = {
            str(item["trigger_group"])
            for item in selected
            if isinstance(item.get("trigger_group"), str) and item["trigger_group"]
        }
        expected_ids = {
            str(item["id"])
            for item in selected
            if isinstance(item.get("id"), str) and item["id"]
        }
        if not groups or not expected_ids:
            selected_case = (
                self._active_selection.case_id
                if self._active_selection is not None
                else None
            )
            raise ConfigurationError(
                "dynamic episode selection resolved to no trigger groups or expected "
                f"actor IDs: case_id={selected_case!r}"
            )

    def _trigger_obstacle_group(self, goal_id: str | None) -> None:
        if self._scenario.scenario_type != "dynamic" or not goal_id:
            return
        for group in self._selected_dynamic_groups_for_goal(goal_id):
            client = self._obstacle_trigger_clients.get(group)
            if client is None:
                client = self.create_client(
                    Trigger, f"/experiment/obstacles/{group}/trigger"
                )
                self._obstacle_trigger_clients[group] = client
            if not client.wait_for_service(timeout_sec=self._service_timeout_sec):
                raise RuntimeError(f"obstacle trigger service is unavailable for {group}")
            future = client.call_async(Trigger.Request())
            if not self._wait_future(
                future, time.monotonic() + self._service_timeout_sec
            ):
                raise TimeoutError(f"obstacle trigger timed out for {group}")
            response = future.result()
            if response is None or not response.success:
                detail = response.message if response is not None else "no response"
                raise RuntimeError(f"obstacle trigger failed for {group}: {detail}")

    def _dynamic_retirement_clearance_observed(
        self,
        retired_ids: set[str],
        barrier_ros_s: float,
        source_sequence_before: int,
        status_sequences_before: Mapping[str, int],
    ) -> bool:
        if (
            self._obstacle_state_stamp_s is None
            or self._obstacle_state_stamp_s <= barrier_ros_s
        ):
            return False
        obstacle_states = {
            str(item.get("id")): str(item.get("state"))
            for item in self._obstacle_state.get("obstacles", [])
            if isinstance(item, Mapping) and item.get("id")
        }
        if any(obstacle_states.get(identifier) != "retired" for identifier in retired_ids):
            return False

        source = self._latest_cognitive_obstacles
        if (
            source is None
            or int(source.sequence) <= source_sequence_before
            or (
                source.header.stamp.sec
                + source.header.stamp.nanosec * 1.0e-9
                <= barrier_ros_s
            )
            or (
                source.validation_stamp.sec
                + source.validation_stamp.nanosec * 1.0e-9
                <= barrier_ros_s
            )
            or bool(source.obstacles)
        ):
            return False

        required_consumers: dict[str, RiskLayerStatus] = {}
        for consumer, status in self._latest_cognitive_layer_statuses.items():
            status_stamp_s = status.stamp.sec + status.stamp.nanosec * 1.0e-9
            if status_stamp_s <= barrier_ros_s:
                continue
            if "global_costmap" in consumer:
                required_consumers["global"] = status
            elif "local_costmap" in consumer:
                required_consumers["local"] = status
        if set(required_consumers) != {"global", "local"}:
            return False
        return all(
            str(status.mode) == "active"
            and not bool(status.applied)
            and int(status.active_cell_count) == 0
            and (
                not bool(status.rejected)
                or "rejection_reason=no_costmap_cells"
                in str(status.fallback_reason)
            )
            and int(status.maximum_cost) == 0
            and int(status.raised_cell_count) == 0
            and int(status.maximum_cost_increase) == 0
            and int(status.source_sequence) == int(source.sequence)
            and int(status.source_sequence) > status_sequences_before.get(role, -1)
            and int(status.reset_epoch) == int(source.reset_epoch)
            and str(status.recurrent_session_id) == str(source.recurrent_session_id)
            and str(status.map_version) == str(source.map_version)
            for role, status in required_consumers.items()
        )

    def _requires_dynamic_retirement_clearance(
        self, retired_ids: set[str]
    ) -> bool:
        return bool(
            self._nav2_profile == "v6_low_obstacle_isolation"
            and self._scenario.map_version == "v6_kujiale_isaacgen_v1"
            and retired_ids == {"v6_dynamic_g2_crossing_box"}
        )

    def _complete_obstacle_group(self, goal_id: str | None) -> tuple[str, ...]:
        """Retire only the actors tied to a successfully completed route goal."""
        if self._scenario.scenario_type != "dynamic" or not goal_id:
            return ()
        source_sequence_before = (
            int(self._latest_cognitive_obstacles.sequence)
            if self._latest_cognitive_obstacles is not None
            else -1
        )
        status_sequences_before: dict[str, int] = {}
        for consumer, status in self._latest_cognitive_layer_statuses.items():
            if "global_costmap" in consumer:
                status_sequences_before["global"] = int(status.source_sequence)
            elif "local_costmap" in consumer:
                status_sequences_before["local"] = int(status.source_sequence)

        retired_ids: list[str] = []
        selected = self._selected_dynamic_trajectories()
        for group in self._selected_dynamic_groups_for_goal(goal_id):
            expected_ids = {
                str(item["id"])
                for item in selected
                if item.get("trigger_group") == group
                and isinstance(item.get("id"), str)
                and item["id"]
            }
            if not expected_ids:
                raise ConfigurationError(
                    f"dynamic completion group {group!r} has no selected actor IDs"
                )
            client = self._obstacle_complete_clients.get(group)
            if client is None:
                client = self.create_client(
                    Trigger, f"/experiment/obstacles/{group}/complete"
                )
                self._obstacle_complete_clients[group] = client
            if not client.wait_for_service(timeout_sec=self._service_timeout_sec):
                raise RuntimeError(f"obstacle completion service is unavailable for {group}")
            future = client.call_async(Trigger.Request())
            if not self._wait_future(
                future, time.monotonic() + self._service_timeout_sec
            ):
                raise TimeoutError(f"obstacle completion timed out for {group}")
            response = future.result()
            if response is None or not response.success:
                detail = response.message if response is not None else "no response"
                raise RuntimeError(f"obstacle completion failed for {group}: {detail}")
            retired_ids.extend(
                _parse_obstacle_completion(
                    str(response.message),
                    expected_group=group,
                    expected_ids=expected_ids,
                )
            )

        retired_set = set(retired_ids)
        if self._requires_dynamic_retirement_clearance(retired_set):
            barrier_ros_s = self._clock_seconds()
            if barrier_ros_s is None:
                raise RuntimeError(
                    "dynamic obstacle retirement clearance requires ROS time"
                )
            self._clear_navigation_costmaps()
            if not self._wait_until(
                lambda: self._dynamic_retirement_clearance_observed(
                    retired_set,
                    barrier_ros_s,
                    source_sequence_before,
                    status_sequences_before,
                ),
                DYNAMIC_RETIREMENT_CLEAR_TIMEOUT_SEC,
            ):
                raise RuntimeError(
                    "dynamic obstacle retirement did not clear the current Module2 "
                    "source and both cognitive costmap consumers before the next leg"
                )
        return tuple(retired_ids)

    def _navigate(self) -> tuple[bool, bool, int]:
        if self._navigation_execution_backend == "route_guided":
            return self._navigate_route_guided()
        return self._navigate_direct()

    def _navigate_direct(self) -> tuple[bool, bool, int]:
        if not self._navigate_client.wait_for_server(
            timeout_sec=self._service_timeout_sec
        ):
            self._raise_if_shutdown()
            raise RuntimeError(
                f"Nav2 action unavailable: {self._action_name}"
            )
        specifications = (
            self._scenario.route
            if self._scenario.route
            else (self._scenario.goal,)
        )
        overall_deadline = time.monotonic() + self._scenario.timeout_sec
        self._navigation_active = True
        self._navigation_start_stamp_s = self._clock_seconds()
        try:
            for index, specification in enumerate(specifications):
                leg_start_stamp = self._clock_seconds()
                leg_gt_start = len(self._ground_truth_samples)
                poses_remaining = len(specifications) - index
                self._minimum_poses_remaining = (
                    poses_remaining
                    if self._minimum_poses_remaining is None
                    else min(
                        self._minimum_poses_remaining,
                        poses_remaining,
                    )
                )
                if not self._goal_dispatch_recorded:
                    self._record_trial_dispatched()
                send_future = self._navigate_client.send_goal_async(
                    self._goal_message(specification),
                    feedback_callback=self._navigation_feedback_callback,
                )
                send_deadline = min(
                    overall_deadline,
                    time.monotonic() + self._service_timeout_sec,
                )
                if not self._wait_future(send_future, send_deadline):
                    raise TimeoutError(
                        "Nav2 goal acknowledgement timed out"
                )
                goal_handle = send_future.result()
                if goal_handle is None or not goal_handle.accepted:
                    self._leg_results.append({"id": specification.goal_id or f"G{index + 1}", "nav2_status": GoalStatus.STATUS_ABORTED, "accepted": False})
                    return False, False, GoalStatus.STATUS_ABORTED
                self._trigger_obstacle_group(specification.goal_id)

                result_future = goal_handle.get_result_async()
                leg_deadline = min(
                    overall_deadline,
                    time.monotonic() + self._scenario.leg_timeout_sec,
                )
                if not self._wait_future(
                    result_future,
                    leg_deadline,
                    abort_on_dynamic_guard=True,
                ):
                    guard_aborted = self._dynamic_guard_aborted
                    cancel_future = goal_handle.cancel_goal_async()
                    if not self._wait_future(
                        cancel_future,
                        time.monotonic() + self._service_timeout_sec,
                    ):
                        context = (
                            "after dynamic safety abort"
                            if guard_aborted
                            else "after timeout"
                        )
                        raise ExperimentIsolationError(
                            "Nav2 goal cancellation acknowledgement was not "
                            f"received {context}"
                        )
                    try:
                        cancel_response = cancel_future.result()
                    except Exception as exc:
                        raise ExperimentIsolationError(
                            f"Nav2 cancellation request failed: {exc}"
                        ) from exc
                    if cancel_response is None:
                        raise ExperimentIsolationError(
                            "Nav2 cancellation returned no response"
                        )
                    if not self._wait_future(
                        result_future,
                        time.monotonic() + self._service_timeout_sec,
                    ):
                        raise ExperimentIsolationError(
                            "Nav2 goal did not reach a terminal state "
                            "after cancellation"
                        )
                    wrapped_result = result_future.result()
                    if wrapped_result is None:
                        raise ExperimentIsolationError(
                            "Nav2 goal returned no terminal result "
                            "after cancellation"
                        )
                    self._leg_results.append({
                        "id": specification.goal_id or f"G{index + 1}",
                        "nav2_status": int(wrapped_result.status),
                        "accepted": True,
                        "timed_out": not guard_aborted,
                        "dynamic_safety_aborted": guard_aborted,
                    })
                    return False, not guard_aborted, int(wrapped_result.status)
                wrapped_result = result_future.result()
                if wrapped_result is None:
                    self._leg_results.append({"id": specification.goal_id or f"G{index + 1}", "nav2_status": GoalStatus.STATUS_UNKNOWN, "accepted": True})
                    return False, False, GoalStatus.STATUS_UNKNOWN
                if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
                    self._leg_results.append({"id": specification.goal_id or f"G{index + 1}", "nav2_status": int(wrapped_result.status), "accepted": True})
                    return False, False, int(wrapped_result.status)
                retired_ids = self._complete_obstacle_group(specification.goal_id)
                self._completed_dynamic_obstacle_ids.update(retired_ids)
                leg_gt = self._ground_truth_samples[leg_gt_start:]
                self._leg_results.append({
                    "id": specification.goal_id or f"G{index + 1}",
                    "nav2_status": int(wrapped_result.status),
                    "accepted": True,
                    "timed_out": False,
                    "duration_sec": max(0.0, self._clock_seconds() - leg_start_stamp),
                    "ground_truth_length_m": path_length([(sample.x, sample.y) for sample in leg_gt]),
                })
                self._minimum_poses_remaining = len(specifications) - index - 1
            return True, False, GoalStatus.STATUS_SUCCEEDED
        finally:
            self._navigation_end_stamp_s = self._clock_seconds()
            self._navigation_active = False

    def _navigate_route_guided(self) -> tuple[bool, bool, int]:
        """Execute every declared mission leg through the A21 coordinator.

        A leg is accepted only after a fresh CanonicalRoute arrives.  Its
        terminal result is the coordinator's fresh route_goal_complete Bool,
        which in turn reflects the internal Nav2 NavigateToPose result.  The
        runner never performs graph search itself.
        """

        if not self._wait_until(
            lambda: self._navigation_graph is not None
            and self._route_goal_publisher.get_subscription_count() > 0,
            self._service_timeout_sec,
        ):
            raise RuntimeError(
                "A21 route-guided backend unavailable: graph or coordinator missing"
            )
        self._wait_for_reset_stop_gate_release()
        specifications = (
            self._scenario.route
            if self._scenario.route
            else (self._scenario.goal,)
        )
        overall_deadline = time.monotonic() + self._scenario.timeout_sec
        self._navigation_active = True
        self._navigation_start_stamp_s = self._clock_seconds()
        try:
            for index, specification in enumerate(specifications):
                identifier = specification.goal_id or f"G{index + 1}"
                leg_start_stamp = self._clock_seconds()
                leg_gt_start = len(self._ground_truth_samples)
                route_epoch = self._canonical_route_epoch
                completion_epoch = self._route_goal_complete_epoch
                route_record_start = len(self._canonical_routes)
                poses_remaining = len(specifications) - index
                self._minimum_poses_remaining = (
                    poses_remaining
                    if self._minimum_poses_remaining is None
                    else min(self._minimum_poses_remaining, poses_remaining)
                )
                final_leg = index == len(specifications) - 1
                if final_leg:
                    self._arm_next_terminal_fence()
                self._terminal_zero_expected_route_completion_epoch = (
                    completion_epoch + 1
                )
                self._terminal_zero_expected_route_leg_id = identifier
                self._terminal_zero_expected_route_leg_is_final = final_leg
                if not self._goal_dispatch_recorded:
                    self._record_trial_dispatched()
                self._route_goal_publisher.publish(
                    self._pose_message(specification)
                )
                route_wait = min(
                    self._service_timeout_sec,
                    max(0.0, overall_deadline - time.monotonic()),
                )
                accepted = self._wait_until(
                    lambda: self._canonical_route_epoch > route_epoch
                    and len(self._canonical_routes) > route_record_start,
                    route_wait,
                )
                if not accepted:
                    self._leg_results.append(
                        {
                            "id": identifier,
                            "nav2_status": GoalStatus.STATUS_ABORTED,
                            "accepted": False,
                            "route_guided": True,
                            "failure_reason": "canonical_route_timeout",
                        }
                    )
                    return False, False, GoalStatus.STATUS_ABORTED

                initial_route = self._canonical_routes[route_record_start]
                self._trigger_obstacle_group(specification.goal_id)
                leg_deadline = min(
                    overall_deadline,
                    time.monotonic() + self._scenario.leg_timeout_sec,
                )
                while (
                    self._route_goal_complete_epoch <= completion_epoch
                    and time.monotonic() < leg_deadline
                    and not self._dynamic_guard_aborted
                ):
                    self._spin_once(
                        min(0.1, max(0.0, leg_deadline - time.monotonic()))
                    )
                terminal = self._route_goal_complete_epoch > completion_epoch
                guard_aborted = self._dynamic_guard_aborted
                if not terminal:
                    self._cancel_stale_navigation_goal()
                    # Let the coordinator observe the cancelled action and
                    # publish its false terminal result before the next reset.
                    self._wait_until(
                        lambda: self._route_goal_complete_epoch > completion_epoch,
                        min(2.0, self._service_timeout_sec),
                    )
                    self._leg_results.append(
                        {
                            "id": identifier,
                            "nav2_status": GoalStatus.STATUS_CANCELED,
                            "accepted": True,
                            "route_guided": True,
                            "timed_out": not guard_aborted,
                            "dynamic_safety_aborted": guard_aborted,
                            "planned_route_length_m": initial_route.get(
                                "planned_length_m"
                            ),
                            "route_request_id": initial_route.get("request_id"),
                            "route_edge_ids": initial_route.get("edge_ids", []),
                        }
                    )
                    return False, not guard_aborted, GoalStatus.STATUS_CANCELED

                succeeded = self._latest_route_goal_complete
                status = (
                    GoalStatus.STATUS_SUCCEEDED
                    if succeeded
                    else GoalStatus.STATUS_ABORTED
                )
                leg_gt = self._ground_truth_samples[leg_gt_start:]
                leg_result = {
                    "id": identifier,
                    "nav2_status": status,
                    "accepted": True,
                    "route_guided": True,
                    "timed_out": False,
                    "duration_sec": max(
                        0.0, self._clock_seconds() - leg_start_stamp
                    ),
                    "ground_truth_length_m": path_length(
                        [(sample.x, sample.y) for sample in leg_gt]
                    ),
                    "planned_route_length_m": initial_route.get(
                        "planned_length_m"
                    ),
                    "route_request_id": initial_route.get("request_id"),
                    "route_edge_ids": initial_route.get("edge_ids", []),
                    "route_history": self._canonical_routes[route_record_start:],
                }
                self._leg_results.append(leg_result)
                if not succeeded:
                    return False, False, status
                retired_ids = self._complete_obstacle_group(specification.goal_id)
                self._completed_dynamic_obstacle_ids.update(retired_ids)
                self._minimum_poses_remaining = len(specifications) - index - 1
            return True, False, GoalStatus.STATUS_SUCCEEDED
        finally:
            self._navigation_end_stamp_s = self._clock_seconds()
            self._navigation_active = False

    def _wait_for_reset_stop_gate_release(self) -> None:
        """Fence route dispatch on the release for this reset generation."""

        if (
            not self._reset_stop_gate_status_received
            and self._reset_stop_gate_status_subscription.get_publisher_count() == 0
        ):
            return
        receipt = self._reset_receipt
        barrier = self._reset_call_barrier_monotonic
        if receipt is None or barrier is None:
            raise RuntimeError(
                "route-guided dispatch is missing the reset receipt barrier"
            )
        generation = int(receipt["generation"])

        def released() -> bool:
            if self._reset_stop_gate_status_error is not None:
                raise RuntimeError(self._reset_stop_gate_status_error)
            status = self._reset_stop_gate_status
            if status is None or status.received_at <= barrier:
                return False
            if status.generation > generation:
                raise RuntimeError(
                    "reset stop gate generation advanced before route dispatch: "
                    f"receipt={generation}, status={status.generation}"
                )
            if status.generation < generation:
                return False
            return not status.held and status.eligible_generation is None

        if not self._wait_until(released, self._reset_recovery_timeout_sec):
            raise TimeoutError(
                "reset stop gate release timed out before route dispatch: "
                f"generation={generation}"
            )

    def _arm_next_terminal_fence(self) -> None:
        """Synchronously reserve the final route request before publishing it."""

        if not self._terminal_fence_arm_client.wait_for_service(
            timeout_sec=self._service_timeout_sec
        ):
            self._raise_if_shutdown()
            raise RuntimeError(
                "RouteCoordinator terminal fence arm service is unavailable"
            )
        future = self._terminal_fence_arm_client.call_async(Trigger.Request())
        if not self._wait_future(
            future, time.monotonic() + self._service_timeout_sec
        ):
            raise TimeoutError("arming final route terminal fence timed out")
        response = future.result()
        if response is None or not response.success:
            detail = "no response" if response is None else response.message
            raise RuntimeError(f"arming final route terminal fence failed: {detail}")

    @staticmethod
    def _motion_quality_metrics(
        samples: list[CommandSample] | list[OdometrySample],
    ) -> dict[str, Any]:
        translated_distance = 0.0
        reverse_distance = 0.0
        curved_distance = 0.0
        moving_time = 0.0
        observed_time = 0.0
        maximum_linear_acceleration = 0.0
        maximum_angular_acceleration = 0.0
        angular_direction_changes = 0
        previous_turn_sign = 0
        for previous, current in zip(samples, samples[1:]):
            dt = current.stamp_s - previous.stamp_s
            if not 0.005 <= dt <= 0.25:
                continue
            linear = previous.linear_speed_mps
            angular = previous.angular_speed_radps
            distance = abs(linear) * dt
            translated_distance += distance
            reverse_distance += max(-linear, 0.0) * dt
            if abs(linear) >= 0.05 and abs(angular) >= 0.15:
                curved_distance += distance
            if abs(linear) >= 0.03 or abs(angular) >= 0.10:
                moving_time += dt
            observed_time += dt
            maximum_linear_acceleration = max(
                maximum_linear_acceleration,
                abs(current.linear_speed_mps - linear) / dt,
            )
            maximum_angular_acceleration = max(
                maximum_angular_acceleration,
                abs(current.angular_speed_radps - angular) / dt,
            )
            if abs(angular) >= 0.25:
                turn_sign = 1 if angular > 0.0 else -1
                if previous_turn_sign and turn_sign != previous_turn_sign:
                    angular_direction_changes += 1
                previous_turn_sign = turn_sign
        return {
            "sample_count": len(samples),
            "observed_duration_sec": observed_time,
            "translated_distance_m": translated_distance,
            "reverse_distance_m": reverse_distance,
            "reverse_distance_fraction": (
                reverse_distance / translated_distance
                if translated_distance > 1.0e-6
                else 0.0
            ),
            "curved_distance_m": curved_distance,
            "curved_distance_fraction": (
                curved_distance / translated_distance
                if translated_distance > 1.0e-6
                else 0.0
            ),
            "stopped_time_fraction": (
                1.0 - moving_time / observed_time
                if observed_time > 1.0e-6
                else 1.0
            ),
            "maximum_linear_acceleration_mps2": maximum_linear_acceleration,
            "maximum_angular_acceleration_radps2": maximum_angular_acceleration,
            "angular_direction_changes": angular_direction_changes,
        }

    @staticmethod
    def _same_direction_overtake_metrics(
        ground_truth_samples: list[OdometrySample],
        obstacle_samples: list[dict[str, Any]],
        obstacle_id: str,
    ) -> dict[str, Any]:
        """Prove that a slow lead actor was passed before it stopped moving.

        The actor and robot streams have different publication rates.  Samples
        are paired with the nearest GT sample within 150 ms, then evidence is
        accumulated rather than relying on a single noisy frame.
        """
        moving = [
            item for item in obstacle_samples
            if item.get("id") == obstacle_id
            and item.get("state") == "moving"
            and isinstance(item.get("stamp_s"), (int, float))
            and isinstance(item.get("position"), list)
            and len(item["position"]) >= 2
        ]
        result: dict[str, Any] = {
            "required": True,
            "actor_id": obstacle_id,
            "moving_sample_count": len(moving),
            "paired_sample_count": 0,
            "lateral_bypass_seen": False,
            "passed_while_moving": False,
            "passed_before_actor_yielded_right": False,
            "complete": False,
        }
        if not moving or not ground_truth_samples:
            return result

        ground_truth = sorted(ground_truth_samples, key=lambda item: item.stamp_s)
        gt_index = 0
        for actor in sorted(moving, key=lambda item: float(item["stamp_s"])):
            stamp_s = float(actor["stamp_s"])
            while (
                gt_index + 1 < len(ground_truth)
                and abs(ground_truth[gt_index + 1].stamp_s - stamp_s)
                <= abs(ground_truth[gt_index].stamp_s - stamp_s)
            ):
                gt_index += 1
            robot = ground_truth[gt_index]
            if abs(robot.stamp_s - stamp_s) > 0.15:
                continue
            position = actor["position"]
            if not all(isinstance(value, (int, float)) and math.isfinite(value) for value in position[:2]):
                continue
            result["paired_sample_count"] += 1
            lateral_separation_m = abs(robot.x - float(position[0]))
            longitudinal_lead_m = robot.y - float(position[1])
            if abs(longitudinal_lead_m) <= 0.70 and lateral_separation_m >= 0.35:
                result["lateral_bypass_seen"] = True
            if longitudinal_lead_m >= 0.35:
                result["passed_while_moving"] = True
                # The same-direction actor begins in the lead lane at x=-0.35
                # and later turns right to clear the upcoming bottleneck.  A
                # pass after that turn is merely waiting for clearance, not a
                # dynamic overtaking response.
                if float(position[0]) <= -0.20:
                    result["passed_before_actor_yielded_right"] = True
        result["complete"] = bool(
            result["lateral_bypass_seen"]
            and result["passed_while_moving"]
            and result["passed_before_actor_yielded_right"]
        )
        return result

    @staticmethod
    def _local_right_bypass_metrics(
        ground_truth_samples: list[OdometrySample],
        obstacle_samples: list[dict[str, Any]],
        obstacle_id: str,
    ) -> dict[str, Any]:
        """Verify a left-to-right actor was passed on the robot's right.

        ``local_bypass`` has a deliberate, visible parking point.  Passing the
        actor after it reaches that point is valid. A collision-guard yield is
        retained as a warning under the physical-collision-free acceptance
        policy, while an actual collision remains a strict failure.
        """
        moving = [
            item for item in obstacle_samples
            if item.get("id") == obstacle_id
            and item.get("state") == "moving"
            and isinstance(item.get("stamp_s"), (int, float))
            and isinstance(item.get("position"), list)
            and len(item["position"]) >= 2
        ]
        parked = [
            item for item in obstacle_samples
            if item.get("id") == obstacle_id
            and item.get("state") == "parked"
            and isinstance(item.get("stamp_s"), (int, float))
            and isinstance(item.get("position"), list)
            and len(item["position"]) >= 2
        ]
        interaction = moving + parked
        result: dict[str, Any] = {
            "required": True,
            "actor_id": obstacle_id,
            "moving_sample_count": len(moving),
            "parked_sample_count": len(parked),
            "planned_park_seen": bool(parked),
            "paired_sample_count": 0,
            "right_side_bypass_seen": False,
            "passed_while_moving": False,
            "passed_after_planned_park": False,
            "complete": False,
        }
        if not moving or not interaction or not ground_truth_samples:
            return result
        ground_truth = sorted(ground_truth_samples, key=lambda item: item.stamp_s)
        gt_index = 0
        for actor in sorted(interaction, key=lambda item: float(item["stamp_s"])):
            stamp_s = float(actor["stamp_s"])
            while (
                gt_index + 1 < len(ground_truth)
                and abs(ground_truth[gt_index + 1].stamp_s - stamp_s)
                <= abs(ground_truth[gt_index].stamp_s - stamp_s)
            ):
                gt_index += 1
            robot = ground_truth[gt_index]
            if abs(robot.stamp_s - stamp_s) > 0.15:
                continue
            position = actor["position"]
            if not all(isinstance(value, (int, float)) and math.isfinite(value) for value in position[:2]):
                continue
            result["paired_sample_count"] += 1
            actor_x, actor_y = float(position[0]), float(position[1])
            if abs(robot.y - actor_y) <= 0.70 and robot.x - actor_x >= 0.35:
                result["right_side_bypass_seen"] = True
            if robot.y - actor_y >= 0.35 and actor["state"] == "moving":
                result["passed_while_moving"] = True
            if robot.y - actor_y >= 0.35 and actor["state"] == "parked":
                result["passed_after_planned_park"] = True
        result["complete"] = bool(
            result["right_side_bypass_seen"]
            and (
                result["passed_while_moving"]
                or result["passed_after_planned_park"]
            )
        )
        return result

    @staticmethod
    def _g2_g3_exit_metrics(
        ground_truth_samples: list[OdometrySample],
        obstacle_samples: list[dict[str, Any]],
        obstacle_id: str,
    ) -> dict[str, Any]:
        """Require a same-direction following interval and outlet-side turn."""
        interaction = [
            item for item in obstacle_samples
            if item.get("id") == obstacle_id
            and item.get("state") in {"moving", "parked"}
            and isinstance(item.get("stamp_s"), (int, float))
            and isinstance(item.get("position"), list)
            and len(item["position"]) >= 2
        ]
        result: dict[str, Any] = {
            "required": True,
            "actor_id": obstacle_id,
            "interaction_sample_count": len(interaction),
            "paired_sample_count": 0,
            "continuous_follow_seen": False,
            "outlet_left_turn_seen": False,
            "complete": False,
        }
        if not interaction or not ground_truth_samples:
            return result
        ground_truth = sorted(ground_truth_samples, key=lambda item: item.stamp_s)
        gt_index = 0
        for actor in sorted(interaction, key=lambda item: float(item["stamp_s"])):
            while (
                gt_index + 1 < len(ground_truth)
                and abs(ground_truth[gt_index + 1].stamp_s - float(actor["stamp_s"]))
                <= abs(ground_truth[gt_index].stamp_s - float(actor["stamp_s"]))
            ):
                gt_index += 1
            robot = ground_truth[gt_index]
            if abs(robot.stamp_s - float(actor["stamp_s"])) > 0.15:
                continue
            actor_x, actor_y = float(actor["position"][0]), float(actor["position"][1])
            result["paired_sample_count"] += 1
            # Both actors proceed south; a positive value therefore means the
            # square is in front of the robot while it is moving away.
            # The calibrated y=2.60 release gate produces a sustained
            # same-lane following interval just beyond 1.20 m in physical
            # pilot trace.  Keep the actor clearly in front and in the same
            # lane, while allowing the documented 1.40 m evidence window.
            if (
                actor["state"] == "moving"
                and 0.20 <= robot.y - actor_y <= 1.40
                and abs(robot.x - actor_x) <= 0.65
            ):
                result["continuous_follow_seen"] = True
            # The planned exit is to the left of the parked actor at x=-0.40.
            if actor["state"] == "parked" and robot.x <= actor_x - 0.35:
                result["outlet_left_turn_seen"] = True
        result["complete"] = bool(result["continuous_follow_seen"] and result["outlet_left_turn_seen"])
        return result

    @staticmethod
    def _g5_g1_left_bypass_metrics(
        ground_truth_samples: list[OdometrySample],
        obstacle_samples: list[dict[str, Any]],
        obstacle_id: str,
    ) -> dict[str, Any]:
        """Verify the robot passes the right-parked door actor on its left."""
        interaction = [
            item for item in obstacle_samples
            if item.get("id") == obstacle_id
            and item.get("state") in {"moving", "parked"}
            and isinstance(item.get("stamp_s"), (int, float))
            and isinstance(item.get("position"), list)
            and len(item["position"]) >= 2
        ]
        result: dict[str, Any] = {
            "required": True,
            "actor_id": obstacle_id,
            "interaction_sample_count": len(interaction),
            "paired_sample_count": 0,
            "left_side_bypass_seen": False,
            "passed_while_present": False,
            "complete": False,
        }
        if not interaction or not ground_truth_samples:
            return result
        ground_truth = sorted(ground_truth_samples, key=lambda item: item.stamp_s)
        gt_index = 0
        for actor in sorted(interaction, key=lambda item: float(item["stamp_s"])):
            while (
                gt_index + 1 < len(ground_truth)
                and abs(ground_truth[gt_index + 1].stamp_s - float(actor["stamp_s"]))
                <= abs(ground_truth[gt_index].stamp_s - float(actor["stamp_s"]))
            ):
                gt_index += 1
            robot = ground_truth[gt_index]
            if abs(robot.stamp_s - float(actor["stamp_s"])) > 0.15:
                continue
            actor_x, actor_y = float(actor["position"][0]), float(actor["position"][1])
            result["paired_sample_count"] += 1
            if abs(robot.y - actor_y) <= 0.70 and actor_x - robot.x >= 0.35:
                result["left_side_bypass_seen"] = True
            # The return path is southbound through the door.  This confirms
            # the robot made progress while the actor had not been retired.
            if robot.y <= actor_y - 0.35:
                result["passed_while_present"] = True
        result["complete"] = bool(result["left_side_bypass_seen"] and result["passed_while_present"])
        return result

    def _start_terminal_zero_observation(self) -> None:
        """Observe actuator commands continuously before the terminal barrier."""

        self._terminal_zero_observation_started_monotonic = time.monotonic()
        self._terminal_zero_barrier_monotonic = None
        self._terminal_zero_barrier_source = "not_observed"
        self._terminal_zero_barrier_leg_id = None
        self._terminal_zero_expected_route_completion_epoch = None
        self._terminal_zero_expected_route_leg_id = None
        self._terminal_zero_expected_route_leg_is_final = False
        self._terminal_zero_confirmed_monotonic = None
        self._terminal_zero_first_zero_monotonic = None
        self._terminal_zero_last_zero_monotonic = None
        self._terminal_zero_confirming_sample_count = 0
        self._terminal_zero_confirmed = False
        self._terminal_zero_reason = "terminal_barrier_not_observed"
        self._cmd_vel_sim_last_receive_monotonic = None
        self._cmd_vel_sim_last_nonzero_monotonic = None
        self._cmd_vel_sim_zero_stamps.clear()

    def _mark_terminal_zero_barrier(
        self, source: str, *, leg_id: str | None = None
    ) -> None:
        """Record the first observed terminal boundary without dropping samples."""

        if self._terminal_zero_barrier_monotonic is not None:
            return
        self._terminal_zero_barrier_monotonic = time.monotonic()
        self._terminal_zero_barrier_source = source
        self._terminal_zero_barrier_leg_id = leg_id
        self._terminal_zero_confirmed_monotonic = None
        self._terminal_zero_first_zero_monotonic = None
        self._terminal_zero_last_zero_monotonic = None
        self._terminal_zero_confirming_sample_count = 0
        self._terminal_zero_confirmed = False
        self._terminal_zero_reason = "terminal_zero_not_observed"

    def _invalidate_terminal_zero_confirmation(self, reason: str) -> None:
        self._terminal_zero_confirmed = False
        self._terminal_zero_reason = reason
        self._terminal_zero_confirmed_monotonic = None
        self._terminal_zero_first_zero_monotonic = None
        self._terminal_zero_last_zero_monotonic = None
        self._terminal_zero_confirming_sample_count = 0

    def _terminal_zero_observation_complete(
        self, now: float, quiet_window_sec: float
    ) -> bool:
        barrier = self._terminal_zero_barrier_monotonic
        if barrier is None:
            self._invalidate_terminal_zero_confirmation(
                "terminal_barrier_not_observed"
            )
            return False
        self._cmd_vel_sim_zero_stamps = [
            stamp for stamp in self._cmd_vel_sim_zero_stamps if stamp > barrier
        ]
        if (
            self._cmd_vel_sim_last_nonzero_monotonic is not None
            and self._cmd_vel_sim_last_nonzero_monotonic > barrier
        ):
            self._invalidate_terminal_zero_confirmation(
                "terminal_nonzero_after_barrier"
            )
            return False
        if not self._cmd_vel_sim_zero_stamps:
            self._invalidate_terminal_zero_confirmation(
                "terminal_zero_not_observed"
            )
            return False
        first_zero = self._cmd_vel_sim_zero_stamps[0]
        if first_zero - barrier > TERMINAL_ZERO_CADENCE_TOLERANCE_SEC:
            self._invalidate_terminal_zero_confirmation(
                "terminal_first_zero_late"
            )
            return False
        if len(self._cmd_vel_sim_zero_stamps) < 2:
            self._invalidate_terminal_zero_confirmation(
                "terminal_zero_repetition_pending"
            )
            return False
        last_zero = self._cmd_vel_sim_zero_stamps[-1]
        if last_zero - first_zero < quiet_window_sec:
            self._invalidate_terminal_zero_confirmation(
                "terminal_zero_quiet_window_pending"
            )
            return False
        if now - last_zero > TERMINAL_ZERO_CADENCE_TOLERANCE_SEC:
            self._invalidate_terminal_zero_confirmation(
                "terminal_zero_cadence_stale"
            )
            return False
        self._terminal_zero_confirmed = True
        self._terminal_zero_reason = "terminal_zero_confirmed"
        self._terminal_zero_confirmed_monotonic = now
        self._terminal_zero_first_zero_monotonic = first_zero
        self._terminal_zero_last_zero_monotonic = last_zero
        self._terminal_zero_confirming_sample_count = len(
            self._cmd_vel_sim_zero_stamps
        )
        return True

    def _terminal_zero_timing(self) -> dict[str, Any]:
        observation_started = self._terminal_zero_observation_started_monotonic
        barrier = self._terminal_zero_barrier_monotonic
        post_terminal_zeros = (
            [stamp for stamp in self._cmd_vel_sim_zero_stamps if stamp > barrier]
            if barrier is not None
            else []
        )

        def after_terminal(value: float | None) -> float | None:
            if barrier is None or value is None or value < barrier:
                return None
            return value - barrier

        settings = self._scenario.success
        return {
            "barrier_source": self._terminal_zero_barrier_source,
            "barrier_leg_id": self._terminal_zero_barrier_leg_id,
            "observation_lead_before_terminal_sec": (
                barrier - observation_started
                if barrier is not None and observation_started is not None
                else None
            ),
            "quiet_window_sec": settings.final_still_duration_sec,
            "timeout_sec": settings.final_still_timeout_sec,
            "first_zero_latency_limit_sec": (
                TERMINAL_ZERO_CADENCE_TOLERANCE_SEC
            ),
            "cadence_tolerance_sec": TERMINAL_ZERO_CADENCE_TOLERANCE_SEC,
            "first_zero_after_terminal_sec": after_terminal(
                post_terminal_zeros[0] if post_terminal_zeros else None
            ),
            "last_zero_after_terminal_sec": after_terminal(
                post_terminal_zeros[-1] if post_terminal_zeros else None
            ),
            "confirmed_after_terminal_sec": after_terminal(
                self._terminal_zero_confirmed_monotonic
            ),
            "last_receive_after_terminal_sec": after_terminal(
                self._cmd_vel_sim_last_receive_monotonic
            ),
            "last_nonzero_after_terminal_sec": after_terminal(
                self._cmd_vel_sim_last_nonzero_monotonic
            ),
            "confirming_zero_sample_count": (
                self._terminal_zero_confirming_sample_count
            ),
            "observed_zero_sample_count": len(post_terminal_zeros),
        }

    def _wait_for_final_stillness(self) -> bool:
        settings = self._scenario.success
        deadline = time.monotonic() + settings.final_still_timeout_sec
        stationary_since: float | None = None
        final_still_confirmed = False
        while time.monotonic() < deadline:
            self._raise_if_shutdown()
            self._spin_once(0.05)
            now = time.monotonic()
            if not self._odom_samples:
                stationary_since = None
                continue
            sample = self._odom_samples[-1]
            fresh = now - sample.received_at <= self._odom_max_age_sec
            stationary = (
                abs(sample.linear_speed_mps) <= settings.final_linear_speed_mps
                and abs(sample.angular_speed_radps) <= settings.final_angular_speed_radps
            )
            if fresh and stationary:
                stationary_since = stationary_since or now
                if now - stationary_since >= settings.final_still_duration_sec:
                    final_still_confirmed = True
            else:
                stationary_since = None
                final_still_confirmed = False
            terminal_zero_confirmed = self._terminal_zero_observation_complete(
                now, settings.final_still_duration_sec
            )
            if final_still_confirmed and terminal_zero_confirmed:
                return True
        if (
            not self._terminal_zero_confirmed
            and self._terminal_zero_reason in {
                "terminal_zero_repetition_pending",
                "terminal_zero_quiet_window_pending",
                "terminal_zero_cadence_stale",
            }
        ):
            self._terminal_zero_reason = "terminal_zero_timeout"
        return final_still_confirmed

    def _build_manifest(
        self,
        *,
        run_index: int,
        seed: int,
        nav2_succeeded: bool,
        timed_out: bool,
        nav2_status: int,
        final_still: bool,
        runner_error: str | None,
    ) -> dict[str, Any]:
        gt = self._ground_truth_samples[-1] if self._ground_truth_samples else None
        odom = self._odom_samples[-1] if self._odom_samples else None
        requires_static_contact_gate = bool(
            self._scenario.scenario_type == "static"
            and self._scenario.obstacles.get("static", [])
        )
        static_contact = static_contact_summary(
            self._ground_truth_samples,
            select_declared_static_obstacles(
                self._obstacle_state.get("obstacles", []),
                self._scenario.obstacles.get("static", []),
            ),
            self._robot_footprint,
        )
        maximum_static_overlap_m = float(
            static_contact["maximum_sat_overlap_m"]
        )
        allowed_static_overlap_m = (
            self._scenario.success.maximum_static_geometric_overlap_m
        )
        static_contact_exceeds_acceptance = exceeds_overlap_tolerance(
            static_contact, allowed_static_overlap_m
        )
        static_contact.update(
            {
                "maximum_sat_overlap_m": maximum_static_overlap_m,
                "maximum_accepted_overlap_m": allowed_static_overlap_m,
                "exceeds_acceptance_overlap": static_contact_exceeds_acceptance,
                "diagnostic_only": True,
                "acceptance_policy": "contact_sensor_only_static_geometry_diagnostic",
            }
        )
        goal_x, goal_y = self._scenario.goal.position
        position_error = math.hypot(gt.x - goal_x, gt.y - goal_y) if gt else 0.0
        goal_yaw = math.radians(self._scenario.goal.yaw_deg)
        orientation_error = wrap_angle(gt.yaw_rad - goal_yaw) if gt else None
        safety_complete = (
            self._collision_seen
            and self._localization_seen
            and (
                self._lock_status_seen
                or self._collision_monitor_active
            )
        ) or not self._scenario.success.require_safety_observations
        thresholds = SingleRunThresholds(
            position_tolerance_m=self._scenario.success.position_tolerance_m,
            orientation_tolerance_rad=math.radians(
                self._scenario.success.orientation_tolerance_deg
            ),
            final_linear_speed_tolerance_mps=self._scenario.success.final_linear_speed_mps,
            final_angular_speed_tolerance_radps=self._scenario.success.final_angular_speed_radps,
        )
        observation = SingleRunObservation(
            nav2_succeeded=nav2_succeeded,
            ground_truth_available=gt is not None,
            ground_truth_position_error_m=position_error,
            ground_truth_orientation_error_rad=orientation_error,
            orientation_required=self._scenario.goal.require_orientation,
            collision_detected=self._collision_detected,
            localization_lost=self._localization_lost,
            tf_interrupted=self._tf_interrupted or not self._tf_ever_available,
            timed_out=timed_out,
            collision_monitor_locked=self._collision_monitor_locked,
            final_linear_speed_mps=odom.linear_speed_mps if odom else 0.0,
            final_angular_speed_radps=odom.angular_speed_radps if odom else 0.0,
            safety_observability_complete=safety_complete,
        )
        evaluation = evaluate_single_run(observation, thresholds)
        navigation_odom = [
            sample
            for sample in self._odom_samples
            if (
                self._navigation_start_stamp_s is not None
                and self._navigation_end_stamp_s is not None
                and self._navigation_start_stamp_s
                <= sample.stamp_s
                <= self._navigation_end_stamp_s
            )
        ]
        ground_truth_path_length = path_length(
            [(sample.x, sample.y) for sample in self._ground_truth_samples]
        )
        reasons = list(evaluation.failure_reasons)
        reference_length = None
        path_deviation_percent = None
        planned_path_length = None
        planned_path_deviation_percent = None
        reference_legs: dict[str, float] = {}
        planned_leg_lengths = [
            float(leg["planned_route_length_m"])
            for leg in self._leg_results
            if isinstance(leg.get("planned_route_length_m"), (int, float))
            and math.isfinite(float(leg["planned_route_length_m"]))
        ]
        if planned_leg_lengths:
            planned_path_length = sum(planned_leg_lengths)
        if self._optimal_reference is not None:
            reference_length = float(self._optimal_reference["total_length_m_0_05"])
            for item in self._optimal_reference["legs"]:
                if isinstance(item, Mapping) and isinstance(item.get("id"), str):
                    value = item.get("length_m_0_05")
                    if isinstance(value, (int, float)) and math.isfinite(float(value)):
                        reference_legs[str(item["id"])] = float(value)
            if reference_length <= 0.0 or len(reference_legs) != len(self._leg_results):
                reasons.append("optimal_reference_incomplete")
            else:
                path_deviation_percent = abs(
                    ground_truth_path_length - reference_length
                ) / reference_length * 100.0
                if (
                    planned_path_length is not None
                    and len(planned_leg_lengths) == len(self._leg_results)
                ):
                    planned_path_deviation_percent = abs(
                        planned_path_length - reference_length
                    ) / reference_length * 100.0
                if path_deviation_percent > 20.0:
                    reasons.append("ground_truth_path_deviation_exceeds_20_percent")
                for leg in self._leg_results:
                    identifier = leg.get("id")
                    if isinstance(identifier, str) and identifier in reference_legs:
                        leg["reference_length_m"] = reference_legs[identifier]
        command_quality = self._motion_quality_metrics(
            self._command_samples
        )
        measured_quality = self._motion_quality_metrics(navigation_odom)
        quality_thresholds = self._scenario.success
        expected_dynamic_ids = {
            str(item["id"])
            for item in self._selected_dynamic_trajectories()
            if isinstance(item.get("id"), str)
        }
        triggered_ids = {
            str(item.get("obstacle_id"))
            for item in self._obstacle_events
            if item.get("event") in {"trigger", "armed"} and isinstance(item.get("obstacle_id"), str)
        }
        completed_ids = set(self._completed_dynamic_obstacle_ids)
        retired_ids = set(self._completed_dynamic_obstacle_ids)
        clearance_by_actor: dict[str, float] = {}
        for sample in self._obstacle_samples:
            identifier = sample.get("id")
            clearance = sample.get("min_clearance_m")
            if (
                isinstance(identifier, str)
                and identifier in expected_dynamic_ids
                and isinstance(clearance, (int, float))
                and math.isfinite(float(clearance))
            ):
                clearance_by_actor[identifier] = min(
                    clearance_by_actor.get(identifier, math.inf),
                    float(clearance),
                )
        selected_case = (
            self._active_selection.case_id
            if self._active_selection is not None
            else None
        )
        interaction_acceptance = _dynamic_interaction_acceptance(
            scenario_type=self._scenario.scenario_type,
            expected_ids=expected_dynamic_ids,
            triggered_ids=triggered_ids,
            completed_ids=completed_ids,
            retired_ids=retired_ids,
            clearance_by_actor=clearance_by_actor,
            evidence_complete=(
                self._depth_frame is not None
                and self._scan_frame is not None
                and self._local_costmap is not None
            ),
            maximum_pairing_clearance_m=(
                1.5 if selected_case == "full_route_four_stage" else None
            ),
        )
        clearance_complete = bool(
            interaction_acceptance["minimum_clearance_complete"]
        )
        dynamic_interaction_complete = bool(
            interaction_acceptance["complete"]
        )
        if not dynamic_interaction_complete:
            reasons.append("dynamic_obstacle_interaction_incomplete")
        warnings: list[str] = []
        if (
            requires_static_contact_gate
            and static_contact["contact_detected"]
        ):
            warnings.append("static_geometric_overlap_diagnostic_only")
        if interaction_acceptance["clearance_warning_below_0_10m"]:
            warnings.append("dynamic_min_clearance_below_0_10m")
        dynamic_behavior: dict[str, Any] = {"required": False, "complete": True}
        if selected_case == "local_bypass":
            dynamic_behavior = self._local_right_bypass_metrics(
                self._ground_truth_samples,
                self._obstacle_samples,
                "local_bypass_actor",
            )
            if not dynamic_behavior["complete"]:
                reasons.append("local_right_bypass_not_observed")
        elif selected_case == "g2_g3_exit":
            dynamic_behavior = self._g2_g3_exit_metrics(
                self._ground_truth_samples,
                self._obstacle_samples,
                "g2_g3_exit_actor",
            )
            if not dynamic_behavior["complete"]:
                reasons.append("g2_g3_follow_and_exit_not_observed")
        elif selected_case == "g5_g1_crossing":
            dynamic_behavior = self._g5_g1_left_bypass_metrics(
                self._ground_truth_samples,
                self._obstacle_samples,
                "g5_g1_crossing_actor",
            )
            if not dynamic_behavior["complete"]:
                reasons.append("g5_g1_left_bypass_not_observed")
        elif selected_case == "full_route_three_stage":
            segments = {
                "g1_g2": self._local_right_bypass_metrics(
                    self._ground_truth_samples, self._obstacle_samples,
                    "local_bypass_actor",
                ),
                "g2_g3": self._g2_g3_exit_metrics(
                    self._ground_truth_samples, self._obstacle_samples,
                    "g2_g3_exit_actor",
                ),
                "g5_g1": self._g5_g1_left_bypass_metrics(
                    self._ground_truth_samples, self._obstacle_samples,
                    "g5_g1_crossing_actor",
                ),
            }
            dynamic_behavior = {
                "required": True,
                "segments": segments,
                "complete": all(item["complete"] for item in segments.values()),
            }
            qualification_dynamic = self._scenario.scenario_id in {
                # This isolated Module3-only smoke shares the formal
                # telemetry/actor contract, but treats the calibrated
                # right-side-bypass classifier as diagnostic.  The smoke's
                # independent runner validates physical collision freedom and
                # the local-bypass minimum clearance instead.
                "kujiale_g2_dynamic_safety_smoke",
            }
            if not dynamic_behavior["complete"] and not qualification_dynamic:
                reasons.append("three_stage_dynamic_behavior_not_observed")
        elif selected_case == "full_route_four_stage":
            maximum_pairing_clearance_m = float(
                interaction_acceptance["maximum_pairing_clearance_m"]
            )
            close_actor_ids = sorted(
                identifier for identifier in expected_dynamic_ids
                if identifier in clearance_by_actor
                and clearance_by_actor[identifier] <= maximum_pairing_clearance_m
            )
            dynamic_behavior = {
                "required": True,
                "contract": "four_stage_close_pairing",
                "maximum_pairing_clearance_m": maximum_pairing_clearance_m,
                "close_actor_ids": close_actor_ids,
                "minimum_clearance_m_by_actor": clearance_by_actor,
                "complete": bool(
                    interaction_acceptance["close_interaction_complete"]
                ),
            }
            if not dynamic_behavior["complete"]:
                reasons.append("four_stage_dynamic_behavior_not_observed")
        if self._dynamic_guard_aborted:
            reasons.append("dynamic_near_contact_abort")
        if self._dynamic_safety_yield:
            warnings.append("dynamic_actor_safety_yield")
        requested_appearance = (
            self._active_selection.appearance_profile_id
            if self._active_selection is not None
            else None
        )
        appearance_ready = (
            requested_appearance is None
            or (
                self._appearance_runtime_contract.get("verified")
                and self._appearance_state is not None
                and self._appearance_state.get("profile_id") == requested_appearance
                and self._appearance_state.get("config_sha256")
                == self._appearance_config_hash
            )
        )
        if not appearance_ready:
            reasons.append("appearance_runtime_contract_incomplete")
        if odom is None:
            reasons.append("odom_unavailable")
        if not final_still:
            reasons.append("final_still_duration_not_met")
        if (
            ground_truth_path_length
            < quality_thresholds.minimum_ground_truth_path_length_m
        ):
            reasons.append("ground_truth_path_too_short")
        if (
            measured_quality["reverse_distance_m"]
            < quality_thresholds.minimum_reverse_distance_m
        ):
            reasons.append("insufficient_reverse_motion")
        if (
            measured_quality["reverse_distance_fraction"]
            > quality_thresholds.maximum_reverse_distance_fraction
        ):
            reasons.append("excessive_reverse_motion")
        if (
            measured_quality["curved_distance_fraction"]
            < quality_thresholds.minimum_curved_distance_fraction
        ):
            reasons.append("insufficient_curved_motion")
        if (
            measured_quality["stopped_time_fraction"]
            > quality_thresholds.maximum_stopped_time_fraction
        ):
            reasons.append("excessive_stopped_time")
        if runner_error:
            reasons.append(f"runner_error:{runner_error}")
        result, reasons = _result_with_terminal_zero(
            reasons, self._terminal_zero_confirmed
        )
        warnings = list(dict.fromkeys(warnings))
        footprint_radius_m = max(
            math.hypot(x, y) for x, y in self._robot_footprint
        )
        minimum_clearance_m = (
            max(0.0, self._minimum_safety_scan_range_m - footprint_radius_m)
            if self._minimum_safety_scan_range_m is not None
            else None
        )
        module2_response_count = len(self._module2_prior_responses)
        module2_healthy_count = sum(
            1 for item in self._module2_prior_responses if item["healthy"]
        )
        graph = self._navigation_graph
        navigation_graph = (
            {
                "graph_id": str(graph.graph_id),
                "revision": int(graph.revision),
                "map_version": str(graph.map_version),
                "nodes": [
                    {
                        "id": int(node.id),
                        "position": [
                            float(node.position.x), float(node.position.y)
                        ],
                        "degree": int(node.degree),
                        "node_type": int(node.node_type),
                        "clearance_m": float(node.clearance_m),
                    }
                    for node in graph.nodes
                ],
                "edges": [
                    {
                        "id": int(edge.id),
                        "from_node": int(edge.from_node),
                        "to_node": int(edge.to_node),
                        "length_m": float(edge.length_m),
                        "min_clearance_m": float(edge.min_clearance_m),
                        "polyline": [
                            [float(point.x), float(point.y)]
                            for point in edge.polyline
                        ],
                    }
                    for edge in graph.edges
                ],
            }
            if graph is not None
            else None
        )
        return {
            "scenario_id": self._scenario.scenario_id,
            "random_seed": seed,
            "reset_receipt": dict(getattr(self, "_reset_receipt", None) or {}),
            "map_version": self._scenario.map_version,
            "posegraph_version": self._scenario.posegraph_version,
            "provenance": dict(self._provenance),
            "robot_config_hash": self._robot_config_hash,
            "nav2_config_hash": self._nav2_config_hash,
            "nav2_profile": self._nav2_profile,
            "clear_slam_localization_buffer": (
                self._clear_slam_localization_buffer
            ),
            "reset_map_base_translation_tolerance_m": (
                self._reset_map_base_translation_tolerance_m
            ),
            # A launcher argument is not sufficient four-arm provenance;
            # persist the value consumed by the installed runner.
            "experiment_arm": self._experiment_arm or None,
            "navigation_execution_backend": self._navigation_execution_backend,
            "condition_stack_id": getattr(self, "_condition_stack_id", "") or None,
            "stack_session_id": getattr(self, "_stack_session_id", "") or None,
            "formal_freeze_digest": getattr(self, "_formal_freeze_digest", "") or None,
            "optimal_reference_hash": self._optimal_reference_hash,
            "dynamic_runtime_contract": dict(
                self._dynamic_runtime_contract
            ),
            "appearance_runtime_contract": dict(
                self._appearance_runtime_contract
            ),
            "appearance": {
                "profile_id": requested_appearance,
                "state": dict(self._appearance_state or {}),
                "ready": appearance_ready,
            },
            "condition_id": (
                self._active_selection.condition_id
                if self._active_selection is not None
                else None
            ),
            "spawn_pose_name": self._spawn_pose.name,
            "usd_start_pose": self._spawn_pose.usd.as_dict(),
            "map_start_pose": self._spawn_pose.map.as_dict(),
            "goal_pose": self._scenario.goal.as_dict(),
            "route_poses": [
                specification.as_dict()
                for specification in self._scenario.route
            ],
            "legs": list(self._leg_results),
            "canonical_routes": list(self._canonical_routes),
            "planning_prior_samples": list(self._planning_prior_samples),
            "srdr_edge_diagnostics": list(self._srdr_edge_diagnostics),
            "route_edge_costs": list(self._route_edge_costs),
            "navigation_graph": navigation_graph,
            "route_progress": list(self._route_progress_samples),
            "smac_plans": list(self._smac_plans),
            "obstacle_trajectories": list(self._scenario.obstacle_trajectories),
            "obstacle_events": list(self._obstacle_events),
            "dynamic_interaction": {
                "expected_ids": sorted(expected_dynamic_ids),
                "triggered_ids": sorted(triggered_ids),
                "completed_ids": sorted(completed_ids),
                "retired_ids": sorted(retired_ids),
                "minimum_clearance_m_by_actor": clearance_by_actor,
                "minimum_clearance_complete": clearance_complete,
                "minimum_clearance_requirement_m": interaction_acceptance[
                    "minimum_clearance_requirement_m"
                ],
                "maximum_pairing_clearance_m": interaction_acceptance[
                    "maximum_pairing_clearance_m"
                ],
                "close_interaction_complete": interaction_acceptance[
                    "close_interaction_complete"
                ],
                "clearance_warning_below_0_10m": interaction_acceptance[
                    "clearance_warning_below_0_10m"
                ],
                "acceptance_policy": interaction_acceptance[
                    "acceptance_policy"
                ],
                "complete": dynamic_interaction_complete,
                "guard_aborted": self._dynamic_guard_aborted,
                "safety_yield": self._dynamic_safety_yield,
            },
            "static_geometric_contact": {
                **static_contact,
                "required": requires_static_contact_gate,
            },
            "dynamic_behavior": dynamic_behavior,
            "physics_dt": self._scenario.physics_dt,
            "rtf": self._scenario.rtf,
            "terminal_zero_confirmed": self._terminal_zero_confirmed,
            "terminal_zero_reason": self._terminal_zero_reason,
            "terminal_zero_timing": self._terminal_zero_timing(),
            "result": result,
            "failure_reason": ";".join(reasons),
            "warning_reason": ";".join(warnings),
            "run_index": run_index,
            "scenario_type": self._scenario.scenario_type,
            "recorded_at_utc": datetime.now(timezone.utc).isoformat(),
            "nav2_status": nav2_status,
            "motion_acceptance": {
                "minimum_ground_truth_path_length_m": (
                    quality_thresholds.minimum_ground_truth_path_length_m
                ),
                "minimum_reverse_distance_m": (
                    quality_thresholds.minimum_reverse_distance_m
                ),
                "maximum_reverse_distance_fraction": (
                    quality_thresholds.maximum_reverse_distance_fraction
                ),
                "minimum_curved_distance_fraction": (
                    quality_thresholds.minimum_curved_distance_fraction
                ),
                "maximum_stopped_time_fraction": (
                    quality_thresholds.maximum_stopped_time_fraction
                ),
            },
            "sample_counts": {
                "ground_truth": len(self._ground_truth_samples),
                "odom": len(self._odom_samples),
                "navigation_odom": len(navigation_odom),
                "navigation_commands": len(self._command_samples),
            },
            "metrics": {
                "ground_truth_position_error_m": position_error,
                "ground_truth_orientation_error_rad": orientation_error,
                "ground_truth_path_length_m": ground_truth_path_length,
                "optimal_reference_path_length_m": reference_length,
                "path_deviation_percent": path_deviation_percent,
                "planned_path_length_m": planned_path_length,
                "planned_path_deviation_percent": planned_path_deviation_percent,
                "execution_time_sec": (
                    max(
                        0.0,
                        self._navigation_end_stamp_s
                        - self._navigation_start_stamp_s,
                    )
                    if self._navigation_start_stamp_s is not None
                    and self._navigation_end_stamp_s is not None
                    else None
                ),
                "minimum_safety_scan_range_m": self._minimum_safety_scan_range_m,
                "minimum_clearance_m": minimum_clearance_m,
                "minimum_clearance_method": (
                    "safety_scan_minus_footprint_circumscribed_radius"
                    if minimum_clearance_m is not None
                    else "unavailable"
                ),
                "odom_path_length_m": path_length(
                    [(sample.x, sample.y) for sample in self._odom_samples]
                ),
                "final_linear_speed_mps": odom.linear_speed_mps if odom else 0.0,
                "final_angular_speed_radps": odom.angular_speed_radps if odom else 0.0,
                "final_still_duration_met": final_still,
                "command_motion_quality": command_quality,
                "measured_motion_quality": measured_quality,
                "route_feedback_count": self._route_feedback_count,
                "minimum_poses_remaining": self._minimum_poses_remaining,
                "maximum_route_recoveries": self._maximum_route_recoveries,
            },
            "module2_health": {
                "response_count": module2_response_count,
                "healthy_count": module2_healthy_count,
                "healthy_fraction": (
                    module2_healthy_count / module2_response_count
                    if module2_response_count
                    else None
                ),
                "model_ids": sorted(
                    {item["model_id"] for item in self._module2_prior_responses}
                ),
                "responses": list(self._module2_prior_responses),
            },
            "observability": {
                "collision_status_seen": self._collision_seen,
                "localization_status_seen": self._localization_seen,
                "collision_monitor_status_seen": (
                    self._lock_status_seen
                    or self._collision_monitor_active
                ),
                "collision_monitor_state_message_seen": (
                    self._lock_status_seen
                ),
                "collision_monitor_lifecycle_active": (
                    self._collision_monitor_active
                ),
                "map_to_odom_seen": self._tf_ever_available,
                "localization_healthy": bool(
                    self._localization_seen
                    and not self._localization_lost
                    and self._tf_ever_available
                    and not self._tf_interrupted
                ),
            },
        }

    @staticmethod
    def _write_gzip_csv(path: Path, fieldnames: list[str], rows: list[dict[str, Any]]) -> None:
        with gzip.open(path, "wt", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(rows)

    @staticmethod
    def _write_pgm(path: Path, width: int, height: int, pixels: bytes) -> bool:
        if width <= 0 or height <= 0 or len(pixels) != width * height:
            return False
        path.write_bytes(f"P5\n{width} {height}\n255\n".encode("ascii") + pixels)
        return True

    def _write_depth_snapshot(self, root: Path) -> bool:
        """Save the latest raw depth sample as portable PGM plus exact metadata."""
        frame = self._depth_frame
        if frame is None:
            return False
        width, height = int(frame["width"]), int(frame["height"])
        encoding = str(frame["encoding"]).lower()
        step, data = int(frame["step"]), bytes(frame["data"])
        if width <= 0 or height <= 0 or step <= 0 or len(data) < step * height:
            return False
        values: list[float] = []
        try:
            import struct
            if encoding in {"32fc1", "32fc"}:
                prefix = ">" if frame["is_bigendian"] else "<"
                for row in range(height):
                    values.extend(struct.unpack_from(prefix + f"{width}f", data, row * step))
            elif encoding in {"16uc1", "mono16"}:
                prefix = ">" if frame["is_bigendian"] else "<"
                for row in range(height):
                    values.extend(value / 1000.0 for value in struct.unpack_from(prefix + f"{width}H", data, row * step))
            else:
                return False
        except (ValueError, struct.error):
            return False
        finite = [value for value in values if math.isfinite(value) and value > 0.0]
        if not finite:
            return False
        # Invert a clipped 0..5 m scale so close low obstacles are prominent.
        pixels = bytes(
            0 if not math.isfinite(value) or value <= 0.0 else max(1, min(255, int(255.0 * (1.0 - min(value, 5.0) / 5.0))))
            for value in values
        )
        metadata = {key: value for key, value in frame.items() if key != "data"}
        metadata.update({"valid_pixels": len(finite), "minimum_depth_m": min(finite), "maximum_depth_m": max(finite)})
        (root / "depth_frame.json").write_text(json.dumps(metadata, indent=2, sort_keys=True), encoding="utf-8")
        return self._write_pgm(root / "depth_frame.pgm", width, height, pixels)

    def _write_appearance_rgb_snapshot(self, root: Path) -> bool:
        """Persist the RGB frame available before the first navigation goal."""
        frame = self._rgb_frame
        if frame is None:
            return False
        width, height = int(frame["width"]), int(frame["height"])
        encoding = str(frame["encoding"]).lower()
        step, data = int(frame["step"]), bytes(frame["data"])
        if width <= 0 or height <= 0 or step <= 0 or len(data) < step * height:
            return False
        channels_by_encoding = {
            "rgb8": (3, (0, 1, 2)),
            "bgr8": (3, (2, 1, 0)),
            "rgba8": (4, (0, 1, 2)),
            "bgra8": (4, (2, 1, 0)),
        }
        if encoding not in channels_by_encoding:
            return False
        channels, order = channels_by_encoding[encoding]
        if step < width * channels:
            return False
        pixels = bytearray()
        for row in range(height):
            offset = row * step
            for column in range(width):
                base = offset + column * channels
                pixels.extend(data[base + component] for component in order)
        target = root / "appearance_rgb_before_goal.ppm"
        target.write_bytes(
            f"P6\n{width} {height}\n255\n".encode("ascii") + bytes(pixels)
        )
        metadata = {key: value for key, value in frame.items() if key != "data"}
        metadata["appearance_state"] = dict(self._appearance_state or {})
        (root / "appearance_rgb_before_goal.json").write_text(
            json.dumps(metadata, indent=2, sort_keys=True), encoding="utf-8"
        )
        return True

    def _write_costmap_snapshot(self, root: Path, name: str, grid: Costmap | None) -> bool:
        if grid is None:
            return False
        width, height = int(grid.metadata.size_x), int(grid.metadata.size_y)
        values = list(grid.data)
        if width <= 0 or height <= 0 or len(values) != width * height:
            return False
        # Unknown=medium gray, free=white, lethal=black; preserving the raw
        # data separately makes the image a readable proof rather than input.
        pixels = bytes(127 if value < 0 else max(0, min(255, 255 - int(value * 2.55))) for value in values)
        metadata = {
            "width": width, "height": height, "resolution_m": float(grid.metadata.resolution),
            "origin": {"x": float(grid.metadata.origin.position.x), "y": float(grid.metadata.origin.position.y)},
            "frame_id": str(grid.header.frame_id),
            "stamp_s": grid.header.stamp.sec + grid.header.stamp.nanosec * 1.0e-9,
        }
        (root / f"{name}.json").write_text(json.dumps(metadata, indent=2, sort_keys=True), encoding="utf-8")
        return self._write_pgm(root / f"{name}.pgm", width, height, pixels)

    @staticmethod
    def _write_scan_snapshot(
        root: Path,
        stem: str,
        frame: dict[str, Any] | None,
    ) -> bool:
        if frame is None or not frame["ranges"]:
            return False
        with (root / f"{stem}.csv").open(
            "w", newline="", encoding="utf-8"
        ) as stream:
            writer = csv.DictWriter(stream, fieldnames=["index", "angle_rad", "range_m"])
            writer.writeheader()
            writer.writerows({"index": index, "angle_rad": frame["angle_min"] + index * frame["angle_increment"], "range_m": value} for index, value in enumerate(frame["ranges"]))
        (root / f"{stem}.json").write_text(
            json.dumps(
                {key: value for key, value in frame.items() if key != "ranges"},
                indent=2,
                sort_keys=True,
            ),
            encoding="utf-8",
        )
        return True

    def _begin_run_evidence(self, run_index: int, seed: int) -> Path:
        root = self._evidence_root_for(run_index, seed)
        root.mkdir(parents=True, exist_ok=False)
        self._active_evidence_root = root
        self._bag_process = None
        self._bag_recorder_error = None
        self._appearance_rgb_snapshot_complete = (
            self._write_appearance_rgb_snapshot(root)
            if self._scenario.appearance_config_file is not None
            else False
        )
        if not self._record_bag:
            return root
        ros2 = shutil.which("ros2")
        if ros2 is None:
            self._bag_recorder_error = "ros2_not_found"
            return root
        topics = [
            "/clock", "/ground_truth/odom", "/odom", "/amcl_pose",
            "/bio_nav/module1/odom", "/tf", "/tf_static", "/cmd_vel",
            "/cmd_vel_nav", "/cmd_vel_smoothed", "/cmd_vel_sim",
            "/navigate_to_pose/_action/status", "/plan", "/transformed_global_plan",
            "/bio_nav/navigation_graph", "/bio_nav/canonical_route",
            "/bio_nav/route_progress", "/bio_nav/route_lookahead_goal",
            "/bio_nav/route_goal", "/bio_nav/route_goal_complete",
            "/bio_nav/module2/edge_priors",
            "/bio_nav/module2/srdr_edge_diagnostics",
            "/bio_nav/route_edge_costs",
            "/bio_nav/module2/cognitive_obstacles",
            "/bio_nav/cognitive_obstacle_layer/status",
            "/bio_nav/cognitive_risk_critic/status",
            "/optimal_trajectory", "/trajectories",
            "/local_costmap/costmap_raw", "/global_costmap/costmap_raw",
            "/lidar/points_raw", "/lidar/points_scan", "/scan", "/scan_safety",
            "/camera/front/image_raw", "/camera/front/camera_info", "/camera/front/depth/image_raw", "/camera/front/depth/points",
            "/experiment/paired_appearance/baseline/image_raw", "/experiment/paired_appearance/variant/image_raw", "/experiment/paired_appearance/state",
            "/simulation/collision", "/simulation/collision_diagnostics",
            "/simulation/reset_event", "/simulation/reset_stop_gate/status",
            "/initialpose",
            "/bio_nav/module2/planning_prior", "/diagnostics",
            "/experiment/obstacles/state", "/experiment/appearance/state", "/collision_monitor_state",
        ]
        log = (root / "bag_record.log").open("wb")
        try:
            self._bag_process = subprocess.Popen(
                [ros2, "bag", "record", "--use-sim-time", "--storage", "mcap", "--storage-preset-profile", "zstd_fast", "--output", str(root / "telemetry"), *topics],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
            )
        except OSError as exc:
            self._bag_recorder_error = f"recorder_start_failed:{type(exc).__name__}:{exc}"
            log.close()
        return root

    def _stop_run_bag(self) -> bool:
        process = self._bag_process
        self._bag_process = None
        forced_stop = False
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=20.0)
            except subprocess.TimeoutExpired:
                forced_stop = True
                process.kill()
                process.wait(timeout=5.0)
        if forced_stop:
            self._bag_recorder_error = "recorder_shutdown_timeout"
        elif process is not None and process.returncode not in {0, -signal.SIGINT}:
            self._bag_recorder_error = f"recorder_exit_code:{process.returncode}"
        elif process is None and self._record_bag and not getattr(
            self, "_bag_recorder_error", None
        ):
            self._bag_recorder_error = "recorder_not_started"
        root = self._active_evidence_root
        return bool(root and (root / "telemetry" / "metadata.yaml").is_file() and any((root / "telemetry").glob("*.mcap")))

    def _final_trial_metric_gate(
        self,
        root: Path,
        summary: Mapping[str, Any],
        manifest: Mapping[str, Any],
        run_index: int,
        seed: int,
    ) -> dict[str, Any]:
        if self._fail_stop_metric_contract is None:
            return {"applicable": False, "passed": True}
        contract = yaml.safe_load(
            self._fail_stop_metric_contract.read_text(encoding="utf-8")
        )
        if contract.get("schema") != "bio_nav.final_rivermark_metric_contract.v1":
            raise ConfigurationError("unsupported fail-stop metric contract")
        primary = contract["primary_navigation_metrics"]
        record = {
            "summary_path": str(root / "run_summary.json"),
            "summary": dict(summary),
            "manifest": dict(manifest),
            "scenario_id": self._scenario.scenario_id,
            "run_index": run_index,
            "seed": seed,
        }
        from .final_rivermark_qualification import (
            _dynamic_threat_coverage,
            _static_encounter_coverage,
        )

        if self._scenario.scenario_id == "final_rivermark_static":
            evaluation = _static_encounter_coverage(
                [record], primary["static"]
            )["runs"][0]
            detail = "static_obstacle_encounter_coverage"
        elif self._scenario.scenario_id == "final_rivermark_dynamic":
            evaluation = _dynamic_threat_coverage(
                [record], primary["dynamic"]
            )["runs"][0]
            detail = "dynamic_threat_coverage"
        elif self._scenario.scenario_id == "final_rivermark_appearance":
            appearance = manifest.get("appearance", {})
            profile = summary.get("appearance_profile_id")
            evaluation = {
                "profile_id": profile,
                "appearance_ready": bool(
                    isinstance(appearance, Mapping)
                    and appearance.get("ready") is True
                    and isinstance(profile, str)
                    and profile in primary["appearance"]["profiles"]
                ),
            }
            evaluation["passed"] = evaluation["appearance_ready"]
            detail = "appearance_application"
        else:
            raise ConfigurationError(
                "fail-stop metric contract is only valid for Final Rivermark scenarios"
            )
        return {
            "applicable": True,
            "passed": bool(evaluation["passed"]),
            "gate": detail,
            "contract_path": str(self._fail_stop_metric_contract),
            "contract_sha256": hashlib.sha256(
                self._fail_stop_metric_contract.read_bytes()
            ).hexdigest(),
            "evaluation": evaluation,
        }

    def _write_run_evidence(
        self,
        manifest: Mapping[str, Any],
        seed: int,
        run_index: int,
        root: Path,
        bag_complete: bool,
    ) -> dict[str, Any]:
        """Write the immutable per-run evidence set consumed by the campaign report."""
        scene = _evidence_scene(
            self._scenario.scenario_id, self._scenario.map_version
        )
        try:
            graph_nodes = list(self.get_node_names_and_namespaces())
            localization_node_ownership = _localization_node_ownership_evidence(
                scene, graph_nodes
            )
            localization_node_ownership["graph_error"] = None
        except Exception as exc:
            localization_node_ownership = _localization_node_ownership_evidence(
                scene, []
            )
            localization_node_ownership["graph_error"] = (
                f"{type(exc).__name__}:{exc}"
            )
            if scene == "outdoor":
                localization_node_ownership["passed"] = False
        if self._record_bag:
            required_topic_coverage = _mcap_required_topic_coverage(
                root / "telemetry" / "metadata.yaml",
                scene=scene,
                route_guided=(
                    self._navigation_execution_backend == "route_guided"
                ),
                recorder_error=getattr(self, "_bag_recorder_error", None),
            )
            required_topic_coverage["required"] = True
        else:
            required_topic_coverage = {
                "scene": scene,
                "required": False,
                "passed": True,
                "required_topics": [],
                "message_counts": {},
                "missing_topics": [],
                "zero_message_topics": [],
                "metadata_present": False,
                "metadata_error": None,
                "recorder_error": None,
                "forbidden_topics": [],
                "forbidden_message_counts": {},
                "observed_forbidden_topics": [],
            }
        route_prior_application = _route_prior_application_evidence(
            list(manifest.get("route_edge_costs", [])),
            required=bool(
                getattr(self, "_require_module2_planning_ready", False)
            ),
        )
        manifest["required_topic_coverage"] = required_topic_coverage
        manifest["route_prior_application"] = route_prior_application
        manifest["route_prior_application_confirmed"] = route_prior_application[
            "confirmed"
        ]
        condition_stack_id = getattr(self, "_condition_stack_id", "")
        stack_session_id = getattr(self, "_stack_session_id", "")
        formal_freeze_digest = getattr(self, "_formal_freeze_digest", "")
        condition_stack_attestation = {
            "required": bool(condition_stack_id),
            "condition_stack_id": condition_stack_id or None,
            "stack_session_id": stack_session_id or None,
            "formal_freeze_digest": formal_freeze_digest or None,
            "confirmed": bool(condition_stack_id and stack_session_id),
        }
        manifest["condition_stack_attestation"] = condition_stack_attestation
        manifest["localization_node_ownership"] = localization_node_ownership
        (root / "run_manifest.json").write_text(
            json.dumps(manifest, indent=2, sort_keys=True), encoding="utf-8"
        )
        timeline = [
            {"event": "leg", **item}
            for item in self._leg_results
        ] + [
            {"event": "obstacle", **item}
            for item in self._obstacle_events
        ]
        (root / "events.jsonl").write_text(
            "\n".join(json.dumps(item, sort_keys=True) for item in timeline) + "\n",
            encoding="utf-8",
        )
        self._write_gzip_csv(root / "ground_truth.csv.gz", ["x", "y", "yaw_rad", "linear_speed_mps", "angular_speed_radps", "stamp_s"], [sample.__dict__ for sample in self._ground_truth_samples])
        self._write_gzip_csv(root / "odom.csv.gz", ["x", "y", "yaw_rad", "linear_speed_mps", "angular_speed_radps", "stamp_s"], [sample.__dict__ for sample in self._odom_samples])
        self._write_gzip_csv(root / "cmd_vel.csv.gz", ["linear_speed_mps", "angular_speed_radps", "stamp_s"], [sample.__dict__ for sample in self._command_samples])
        obstacle_rows = self._obstacle_samples or [
            {"id": item.get("id"), "state": item.get("state"), "stamp_s": None}
            for item in self._obstacle_state.get("obstacles", []) if isinstance(item, Mapping)
        ]
        self._write_gzip_csv(root / "dynamic_obstacles.csv.gz", ["id", "state", "stamp_s", "position", "velocity_mps", "progress", "min_clearance_m"], obstacle_rows)
        self._write_gzip_csv(root / "obstacles.csv.gz", ["id", "state", "stamp_s"], obstacle_rows)
        legs = list(manifest.get("legs", []))
        with (root / "leg_metrics.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=["id", "nav2_status", "accepted", "timed_out", "duration_sec", "ground_truth_length_m", "reference_length_m", "planned_route_length_m", "route_request_id", "route_edge_ids"], extrasaction="ignore")
            writer.writeheader(); writer.writerows(legs)
        depth_complete = self._write_depth_snapshot(root)
        local_costmap_complete = self._write_costmap_snapshot(root, "local_costmap", self._local_costmap)
        global_costmap_complete = self._write_costmap_snapshot(root, "global_costmap", self._global_costmap)
        scan_complete = self._write_scan_snapshot(
            root, "scan", self._scan_frame)
        safety_scan_complete = self._write_scan_snapshot(
            root, "scan_safety", self._safety_scan_frame)
        required_files = {
            "TRIAL_DISPATCHED.json", "run_manifest.json", "events.jsonl", "ground_truth.csv.gz", "odom.csv.gz",
            "cmd_vel.csv.gz", "obstacles.csv.gz", "dynamic_obstacles.csv.gz", "leg_metrics.csv", "depth_frame.pgm",
            "depth_frame.json", "scan.csv", "scan.json",
            "scan_safety.csv", "scan_safety.json", "local_costmap.pgm",
            "local_costmap.json", "global_costmap.pgm", "global_costmap.json",
        }
        if self._scenario.appearance_config_file is not None:
            required_files |= {
                "appearance_rgb_before_goal.ppm",
                "appearance_rgb_before_goal.json",
            }
        data_complete = (
            (bag_complete or not self._record_bag)
            and required_topic_coverage["passed"]
            and depth_complete and scan_complete
            and safety_scan_complete and local_costmap_complete
            and global_costmap_complete
            and (
                self._scenario.appearance_config_file is None
                or self._appearance_rgb_snapshot_complete
            )
            and all((root / name).is_file() for name in required_files)
        )
        # A route matrix dispatches one leg per declared route pose.  Focused
        # scenarios omit ``route`` and dispatch the single ``goal`` instead,
        # so their successful one-leg evidence must not be compared with 0.
        navigation_contract_success = _strict_success_from_leg_count(
            manifest.get("result"),
            len(legs),
            len(self._scenario.route),
            terminal_zero_confirmed=(
                manifest.get("terminal_zero_confirmed") is True
            ),
        )
        summary = {
            "campaign": "kujiale_long_range",
            "kind": self._scenario.scenario_type,
            "seed": seed,
            "condition_id": manifest.get("condition_id"),
            "appearance_profile_id": manifest.get("appearance", {}).get("profile_id"),
            "nav2_profile": manifest.get("nav2_profile"),
            "experiment_arm": manifest.get("experiment_arm"),
            "navigation_execution_backend": manifest.get(
                "navigation_execution_backend"
            ),
            "navigation_contract_success": navigation_contract_success,
            "strict_success": False,
            "terminal_zero_confirmed": manifest.get(
                "terminal_zero_confirmed"
            ),
            "terminal_zero_reason": manifest.get("terminal_zero_reason"),
            "terminal_zero_timing": manifest.get("terminal_zero_timing", {}),
            "reset_receipt": dict(manifest.get("reset_receipt", {})),
            "reset_receipt_confirmed": bool(manifest.get("reset_receipt")),
            "physical_collision_free": not self._collision_detected,
            "contact_sensor_evidence_confirmed": bool(
                manifest.get("observability", {}).get("collision_status_seen")
            ),
            "fixed_map_to_odom_evidence_confirmed": bool(
                scene != "outdoor"
                or manifest.get("observability", {}).get("map_to_odom_seen")
                is True
                and localization_node_ownership["passed"] is True
            ),
            "localization_node_ownership": localization_node_ownership,
            "isaac_contact_sensor_collision_detected": (
                self._isaac_contact_sensor_collision_detected
            ),
            "static_geometric_contact": manifest.get(
                "static_geometric_contact", {}
            ),
            "data_complete": data_complete,
            "checksums_verified": False,
            "required_topic_coverage": required_topic_coverage,
            "route_prior_application": route_prior_application,
            "route_prior_application_confirmed": route_prior_application[
                "confirmed"
            ],
            "condition_stack_id": condition_stack_id or None,
            "stack_session_id": stack_session_id or None,
            "formal_freeze_digest": formal_freeze_digest or None,
            "condition_stack_attestation": condition_stack_attestation,
            "evidence": {
                "mcap_zstd": bag_complete,
                "mcap_required": self._record_bag,
                "depth_frame": depth_complete,
                "appearance_rgb_before_goal": self._appearance_rgb_snapshot_complete,
                "scan": scan_complete,
                "scan_safety": safety_scan_complete,
                "local_costmap": local_costmap_complete,
                "global_costmap": global_costmap_complete,
                "required_files": sorted(required_files),
            },
            "path_deviation_percent": manifest.get("metrics", {}).get(
                "path_deviation_percent"
            ),
            "planned_path_deviation_percent": manifest.get("metrics", {}).get(
                "planned_path_deviation_percent"
            ),
            "planned_path_length_m": manifest.get("metrics", {}).get(
                "planned_path_length_m"
            ),
            "execution_time_sec": manifest.get("metrics", {}).get(
                "execution_time_sec"
            ),
            "minimum_clearance_m": manifest.get("metrics", {}).get(
                "minimum_clearance_m"
            ),
            "localization_healthy": manifest.get("observability", {}).get(
                "localization_healthy"
            ),
            "module2_health": manifest.get("module2_health", {}),
            "unexpected_abort": bool(
                manifest.get("nav2_status") != GoalStatus.STATUS_SUCCEEDED
            ),
            "dynamic_interaction_complete": bool(
                manifest.get("dynamic_interaction", {}).get("complete", False)
            ),
            "warning_reason": str(manifest.get("warning_reason", "")),
            "legs": legs,
        }
        _finalize_summary_acceptance(summary)
        manifest["episode_validity"] = dict(summary["episode_validity"])
        (root / "run_manifest.json").write_text(
            json.dumps(manifest, indent=2, sort_keys=True), encoding="utf-8"
        )
        (root / "run_summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
        try:
            final_trial_metric_gate = self._final_trial_metric_gate(
                root, summary, manifest, run_index, seed
            )
        except Exception as exc:
            final_trial_metric_gate = {
                "applicable": self._fail_stop_metric_contract is not None,
                "passed": False,
                "error": f"{type(exc).__name__}:{exc}",
            }
        summary["final_trial_metric_gate"] = final_trial_metric_gate
        (root / "FINAL_TRIAL_METRICS.json").write_text(
            json.dumps(final_trial_metric_gate, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        (root / "run_summary.json").write_text(
            json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8"
        )
        self._finalize_checksums(root, summary, manifest)
        return summary

    @staticmethod
    def _finalize_checksums(
        root: Path,
        summary: dict[str, Any],
        manifest: dict[str, Any] | None = None,
    ) -> None:
        def inventory() -> list[str]:
            return [
                f"{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.relative_to(root)}"
                for path in sorted(
                    item
                    for item in root.rglob("*")
                    if item.is_file() and item.name != "checksums.sha256"
                )
            ]

        checksums = inventory()
        (root / "checksums.sha256").write_text(
            "\n".join(checksums) + "\n", encoding="utf-8"
        )
        # Verify the manifest just written instead of claiming checksum health.
        summary["checksums_verified"] = all(
            hashlib.sha256((root / relative).read_bytes()).hexdigest() == digest
            for digest, relative in (line.split("  ", 1) for line in checksums)
        )
        _finalize_summary_acceptance(summary)
        if manifest is not None:
            manifest["required_topic_coverage"] = summary[
                "required_topic_coverage"
            ]
            manifest["route_prior_application"] = summary[
                "route_prior_application"
            ]
            manifest["route_prior_application_confirmed"] = summary[
                "route_prior_application_confirmed"
            ]
            manifest["episode_validity"] = dict(summary["episode_validity"])
            (root / "run_manifest.json").write_text(
                json.dumps(manifest, indent=2, sort_keys=True), encoding="utf-8"
            )
        (root / "run_summary.json").write_text(
            json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8"
        )
        # run_summary changed after verification; regenerate exactly once so
        # the final file is itself included in the checksum inventory.
        (root / "checksums.sha256").write_text(
            "\n".join(inventory()) + "\n", encoding="utf-8"
        )

    def _evidence_root_for(self, run_index: int, seed: int) -> Path:
        return (
            self._output_directory
            / self._scenario.scenario_id
            / f"run-{run_index:04d}-seed-{seed}"
        )

    @staticmethod
    def _checksums_are_verified(root: Path) -> bool:
        checksum_file = root / "checksums.sha256"
        if not checksum_file.is_file():
            return False
        try:
            entries = [
                line.split("  ", 1)
                for line in checksum_file.read_text(encoding="utf-8").splitlines()
                if line.strip()
            ]
        except OSError:
            return False
        if not entries or any(len(item) != 2 or len(item[0]) != 64 for item in entries):
            return False
        for digest, relative in entries:
            candidate = root / relative
            try:
                candidate.resolve().relative_to(root.resolve())
            except ValueError:
                return False
            if not candidate.is_file() or hashlib.sha256(candidate.read_bytes()).hexdigest() != digest:
                return False
        return True

    def _completed_resume_manifest(
        self, root: Path, run_index: int, selection: RunSelection
    ) -> dict[str, Any] | None:
        summary_path = root / "run_summary.json"
        manifest_path = root / "run_manifest.json"
        if not summary_path.is_file() or not manifest_path.is_file():
            return None
        try:
            summary = json.loads(summary_path.read_text(encoding="utf-8"))
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return None
        if not (
            isinstance(summary, Mapping)
            and summary.get("data_complete") is True
            and summary.get("checksums_verified") is True
            and self._checksums_are_verified(root)
            and isinstance(manifest, dict)
        ):
            return None
        if summary.get("episode_validity", {}).get("valid") is not True:
            return None
        expected = {
            "random_seed": selection.seed,
            "run_index": run_index,
            "condition_id": selection.condition_id,
        }
        if any(manifest.get(key) != value for key, value in expected.items()):
            return None
        appearance = manifest.get("appearance", {})
        if selection.appearance_profile_id is not None and (
            not isinstance(appearance, Mapping)
            or appearance.get("profile_id") != selection.appearance_profile_id
        ):
            return None
        dynamic = manifest.get("dynamic_selection", {})
        if (
            not isinstance(dynamic, Mapping)
            or dynamic.get("case_id") != selection.case_id
            or dynamic.get("variant_id") != selection.variant_id
        ):
            return None
        # Complete valid evidence is immutable whether it passed or failed.
        # Successful-resume mode may skip only a passing episode; a valid
        # product failure blocks the campaign, while only invalid/incomplete
        # evidence returns None and is eligible for quarantine.
        if self._require_successful_resume:
            final_metric_gate = summary.get("final_trial_metric_gate", {})
            if (
                manifest.get("result") != "success"
                or manifest.get("terminal_zero_confirmed") is not True
                or summary.get("strict_success") is not True
                or not isinstance(final_metric_gate, Mapping)
                or final_metric_gate.get("passed") is not True
            ):
                raise ConfigurationError(
                    "resume blocked by an immutable valid product failure"
                )
        return manifest

    @staticmethod
    def _quarantine_incomplete_evidence(root: Path) -> Path:
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")
        target = root.with_name(f"{root.name}.incomplete-{timestamp}")
        suffix = 1
        while target.exists():
            target = root.with_name(f"{root.name}.incomplete-{timestamp}-{suffix}")
            suffix += 1
        root.rename(target)
        return target

    def run_all(self) -> list[dict[str, Any]]:
        if not self._wait_until(lambda: self._clock_ready, self._clock_timeout_sec):
            raise TimeoutError("timed out waiting for a non-zero /clock")
        self._verify_dynamic_runtime_contract()
        self._verify_appearance_runtime_contract()
        # Authorization-only cold starts never dispatch a goal or evaluate a
        # route.  Collision Monitor readiness is therefore not part of their
        # no-navigation contract and can race Nav2 lifecycle activation.
        # Route runs retain the authoritative monitor check below this branch.
        if not self._authorization_only:
            self._verify_collision_monitor_active()
        manifests: list[dict[str, Any]] = []
        selections = self._scenario.run_matrix or tuple(
            RunSelection(seed) for seed in self._scenario.seeds
        )
        for run_index, selection in enumerate(selections, start=1):
            if self._run_indices is not None and run_index not in self._run_indices:
                continue
            seed = selection.seed
            self._active_selection = selection
            self._active_run_index = run_index
            if self._require_pregoal_authorization:
                assert self._pregoal_authorization_path is not None
                validate_pregoal_authorization(
                    self._pregoal_authorization_path,
                    scenario_id=self._scenario.scenario_id,
                    run_index=run_index,
                    selection=selection,
                    expected_receipt=self._pregoal_expected_receipt,
                    expected_schema=self._pregoal_expected_schema,
                    expected_campaign=self._pregoal_expected_campaign,
                    expected_prereg_sha256=self._pregoal_expected_prereg_sha256,
                )
                self._pregoal_authorization_sha256 = hashlib.sha256(
                    self._pregoal_authorization_path.read_bytes()
                ).hexdigest()
                self._lifecycle_event("runner_started")
                if self._authorization_only:
                    return [{"authorization_only": True, "run_index": run_index, "seed": seed}]
            self._validate_dynamic_episode_selection()
            existing_root = self._evidence_root_for(run_index, seed)
            if self._record_evidence and existing_root.exists():
                if not self._resume:
                    raise ConfigurationError(
                        f"evidence directory already exists: {existing_root}; rerun with resume:=true"
                    )
                preserved = self._completed_resume_manifest(
                    existing_root, run_index, selection
                )
                if preserved is not None:
                    manifests.append(preserved)
                    self.get_logger().info(
                        f"resume verified {existing_root.name}; skipping completed run"
                    )
                    continue
                quarantined = self._quarantine_incomplete_evidence(existing_root)
                self.get_logger().warning(
                    f"quarantined incomplete evidence before retry: {quarantined.name}"
                )
            nav2_succeeded = False
            timed_out = False
            nav2_status = GoalStatus.STATUS_UNKNOWN
            final_still = False
            runner_error: str | None = None
            isolation_error: ExperimentIsolationError | None = None
            root: Path | None = None
            bag_complete = False
            run_summary: dict[str, Any] | None = None
            try:
                reset_case_id, reset_variant_id = _reset_dynamic_selection(
                    self._scenario.scenario_type, selection
                )
                self._reset_simulation(
                    seed,
                    reset_case_id,
                    reset_variant_id,
                    selection.appearance_profile_id,
                )
                if self._require_module2_planning_ready and not self._wait_until(
                    lambda: self._planning_prior_ready_streak >= 5,
                    self._module2_planning_ready_timeout_sec,
                ):
                    raise RuntimeError(
                        "Module2 planning prior did not become goal-query ready"
                    )
                if self._record_evidence:
                    root = self._begin_run_evidence(run_index, seed)
                    self._lifecycle_event("evidence_started")
                self._start_terminal_zero_observation()
                nav2_succeeded, timed_out, nav2_status = self._navigate()
                if (
                    self._navigation_execution_backend == "navigate_to_pose"
                    and self._terminal_zero_barrier_monotonic is None
                ):
                    self._mark_terminal_zero_barrier("navigate_action_return")
                final_still = self._wait_for_final_stillness()
            except Exception as exc:  # Preserve a manifest for every attempted run.
                if not rclpy.ok():
                    raise KeyboardInterrupt from exc
                # A failure before evidence recording means no goal could have
                # been dispatched.  It is an environment fault, not a trial.
                if root is None and self._record_evidence:
                    raise
                if isinstance(exc, ExperimentIsolationError):
                    isolation_error = exc
                runner_error = f"{type(exc).__name__}:{exc}"
                self.get_logger().error(runner_error)
            finally:
                bag_complete = self._stop_run_bag()
            manifest = self._build_manifest(
                run_index=run_index,
                seed=seed,
                nav2_succeeded=nav2_succeeded,
                timed_out=timed_out,
                nav2_status=nav2_status,
                final_still=final_still,
                runner_error=runner_error,
            )
            manifest["dynamic_selection"] = {
                "case_id": selection.case_id, "variant_id": selection.variant_id,
            }
            if self._record_evidence:
                timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
                stem = f"{self._scenario.scenario_id}-run-{run_index:04d}-seed-{seed}-{timestamp}"
                write_run_report(manifest, self._output_directory, stem)
                if root is None:
                    root = self._begin_run_evidence(run_index, seed)
                run_summary = self._write_run_evidence(
                    manifest, seed, run_index, root, bag_complete
                )
            else:
                stem = f"{self._scenario.scenario_id}-run-{run_index:04d}-seed-{seed}"
            manifests.append(manifest)
            self.get_logger().info(f"completed {stem}: {manifest['result']}")
            if isolation_error is not None:
                raise isolation_error
            if self._fail_stop and not (
                run_summary is not None
                and run_summary.get("strict_success") is True
                and run_summary.get("episode_validity", {}).get("valid") is True
                and run_summary.get("physical_collision_free") is True
                and run_summary.get("data_complete") is True
                and run_summary.get("checksums_verified") is True
                and run_summary.get("final_trial_metric_gate", {}).get(
                    "passed"
                ) is True
            ):
                raise RuntimeError(
                    "fail-stop campaign trial failed after dispatch: "
                    f"run_index={run_index}, result={manifest.get('result')}, "
                    "collision_free="
                    f"{None if run_summary is None else run_summary.get('physical_collision_free')}, "
                    "data_complete="
                    f"{None if run_summary is None else run_summary.get('data_complete')}, "
                    "checksums_verified="
                    f"{None if run_summary is None else run_summary.get('checksums_verified')}, "
                    "final_trial_metric_gate="
                    f"{None if run_summary is None else run_summary.get('final_trial_metric_gate', {}).get('passed')}"
                )
        return manifests


def main(args=None) -> None:
    rclpy.init(args=args)
    node: ExperimentRunner | None = None
    try:
        node = ExperimentRunner()
        node.run_all()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
