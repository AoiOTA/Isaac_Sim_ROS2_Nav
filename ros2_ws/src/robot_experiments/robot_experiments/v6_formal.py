"""Dispatch one V6-GRID Phase-1 empty-room full-house episode.

The dispatcher owns one physical reset and the five XY route goals. Global
pose ownership stays with the Grid localizer and its TF manager. Ground Truth
is reserved for the passive recorder/evaluator.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping

from robot_experiments.reset_receipt import ResetReceiptError, parse_reset_receipt

import yaml


SCHEMA_VERSION = "bio_nav_v6_single_episode_manifest_v1"
NOT_QUALIFIED = "NOT_QUALIFIED"
ENGINEERING_PILOT = "ENGINEERING_PILOT"
GT_PREFIX = "/" + "ground_truth/"
PRE_RESET_NEGATIVE_WINDOW_S = 1.0
PRE_RESET_STILL_SPAN_M = 0.10
COMMAND_ZERO_TOLERANCE = 1.0e-3
POST_RESET_ODOM_LANDING_M = 0.10
POST_RESET_ODOM_SPAN_M = 0.10
NAV2_PROBE_ATTEMPT_TIMEOUT_SEC = 5.0
FULL_HOUSE_LEGS = ("G2", "G3", "G4", "G5", "G1")
FINAL_ESTIMATED_POLICY = {
    "ekf_profile": "wheel_imu",
    "lidar_odometry_backend": "off",
    "lidar_odometry_validated": False,
    "rf2o_decision": "not_validated_off",
    "imu_calibration_profile": "isaac_v6_calibrated",
}
PHASE1_RUNTIME = {
    "phase": "phase1_empty_room",
    "odometry_mode": "estimated",
    "localization_backend": "grid",
    "nav2_profile": "stable",
    "cognitive_profile": "M0",
    "module2_enabled": False,
    "cognitive_graph_mode": "gvg",
    "low_obstacles_enabled": False,
    "dynamic_actors_enabled": False,
    "structure_tf_source": "isaac",
    "ground_truth_policy": "evaluator_only",
    "direct_rgbd_costmap_enabled": False,
}
SOLE_PUBLISHER_TOPICS = ("/odom", "/cmd_vel", "/cmd_vel_sim")

# This is the complete dispatcher subscription firewall. Ground Truth stays
# outside this process and is captured by the session recorder.
DISPATCH_SUBSCRIPTION_TOPICS = (
    "/clock",
    "/scan",
    "/flatscan",
    "/localization_result",
    "/bio_nav/localization/status",
    "/odom",
    "/tf",
    "/tf_static",
    "/map",
    "/simulation/reset_event",
    "/simulation/reset_stop_gate/status",
    "/bio_nav/navigation_graph",
    "/bio_nav/canonical_route",
    "/bio_nav/route_progress",
    "/bio_nav/route_goal_complete",
    "/bio_nav/route_goal_result",
    "/cmd_vel",
    "/cmd_vel_nav",
    "/cmd_vel_smoothed",
    "/cmd_vel_sim",
    "/simulation/collision",
    "/simulation/collision_diagnostics",
    "/diagnostics",
)

CAPTURE_SCHEMA = {
    "/flatscan": "FlatScan",
    "/localization_result": "PoseWithCovarianceStamped",
    "/bio_nav/localization/status": "DiagnosticArray",
    "/odom": "Odometry",
    "/bio_nav/navigation_graph": "NavigationGraph",
    "/bio_nav/canonical_route": "CanonicalRoute",
    "/bio_nav/route_progress": "RouteProgress",
    "/bio_nav/route_goal_result": "String",
    "/cmd_vel": "Twist",
    "/cmd_vel_nav": "Twist",
    "/cmd_vel_smoothed": "Twist",
    "/cmd_vel_sim": "Twist",
    "/simulation/collision": "Bool",
    "/simulation/collision_diagnostics": "String",
}

if any(topic.startswith(GT_PREFIX) for topic in DISPATCH_SUBSCRIPTION_TOPICS):
    raise RuntimeError("V6 dispatcher Ground Truth firewall violated")


class V6ContractError(RuntimeError):
    """A fail-closed V6 manifest or episode contract violation."""


def append_evidence_jsonl(path: Path, event: str, **payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    row = {"event": event, "wall_time_ns": time.time_ns(), **payload}
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(row, sort_keys=True) + "\n")


@dataclass(frozen=True)
class Episode:
    seed: int
    variant_id: str
    appearance_profile_id: str | None
    reset_pose_name: str
    dynamic_case_id: str


@dataclass(frozen=True)
class MissionLeg:
    goal_id: str
    frame_id: str
    x: float
    y: float


@dataclass(frozen=True)
class Manifest:
    path: Path
    raw: Mapping[str, Any]
    scene_id: str
    category: str
    phase1_enabled: bool
    scenario_intent: Mapping[str, Any]
    estimated_policy: Mapping[str, Any]
    frozen: bool
    reset_pose: Mapping[str, Any]
    mission_legs: tuple[MissionLeg, ...]
    episodes: tuple[Episode, ...]
    missing_required_values: tuple[str, ...]


def _mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise V6ContractError(f"{path} must be a mapping")
    return value


def _finite_float(value: Any, path: str) -> float:
    if isinstance(value, bool):
        raise V6ContractError(f"{path} must be finite numeric")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise V6ContractError(f"{path} must be finite numeric") from exc
    if not math.isfinite(result):
        raise V6ContractError(f"{path} must be finite numeric")
    return result


def _missing_required_values(raw: Mapping[str, Any]) -> tuple[str, ...]:
    required = _mapping(raw.get("required_runtime_values"), "required_runtime_values")
    return tuple(
        sorted(
            f"required_runtime_values.{name}"
            for name, value in required.items()
            if value is None or value == ""
        )
    )


def _estimated_policy(runtime: Mapping[str, Any]) -> Mapping[str, Any]:
    policy = {name: runtime.get(name) for name in FINAL_ESTIMATED_POLICY}
    for name in (
        "ekf_profile",
        "lidar_odometry_backend",
        "rf2o_decision",
        "imu_calibration_profile",
    ):
        if not isinstance(policy[name], str) or not policy[name]:
            raise V6ContractError(f"runtime.{name} must be a non-empty string")
    if not isinstance(policy["lidar_odometry_validated"], bool):
        raise V6ContractError("runtime.lidar_odometry_validated must be boolean")
    return policy


def _estimated_policy_mismatches(manifest: Manifest) -> tuple[str, ...]:
    return tuple(
        f"runtime.{name}={manifest.estimated_policy.get(name)!r}"
        for name, expected in FINAL_ESTIMATED_POLICY.items()
        if manifest.estimated_policy.get(name) != expected
    )


def _map_pose(
    raw: Mapping[str, Any], path: str, *, require_yaw: bool
) -> tuple[float, float]:
    if str(raw.get("frame_id", "")) != "map":
        raise V6ContractError(f"{path}.frame_id must be map")
    x = _finite_float(raw.get("x"), f"{path}.x")
    y = _finite_float(raw.get("y"), f"{path}.y")
    if require_yaw:
        _finite_float(raw.get("yaw_deg"), f"{path}.yaw_deg")
    elif "yaw" in raw or "yaw_deg" in raw:
        raise V6ContractError(f"{path} must be XY-only")
    return x, y


def load_manifest(path: str | Path) -> Manifest:
    manifest_path = Path(path).expanduser().resolve()
    raw = _mapping(
        yaml.safe_load(manifest_path.read_text(encoding="utf-8")), "manifest"
    )
    if raw.get("schema_version") != SCHEMA_VERSION:
        raise V6ContractError(f"schema_version must be {SCHEMA_VERSION}")

    runtime = _mapping(raw.get("runtime"), "runtime")
    for name, expected in PHASE1_RUNTIME.items():
        if runtime.get(name) != expected:
            raise V6ContractError(f"runtime.{name} must be {expected!r}")

    frozen = raw.get("scene_contract_frozen")
    phase1_enabled = raw.get("phase1_enabled")
    if not isinstance(frozen, bool):
        raise V6ContractError("scene_contract_frozen must be boolean")
    if not isinstance(phase1_enabled, bool):
        raise V6ContractError("phase1_enabled must be boolean")
    scene = _mapping(raw.get("scene"), "scene")
    category = str(scene.get("category", ""))
    if category not in {"static", "dynamic", "appearance"}:
        raise V6ContractError("scene.category must be static, dynamic, or appearance")
    scenario_intent = _mapping(raw.get("scenario_intent"), "scenario_intent")
    if category != "static" and phase1_enabled:
        raise V6ContractError("dynamic/appearance manifests cannot enable Phase 1")

    mission = _mapping(raw.get("mission"), "mission")
    reset_pose = _mapping(mission.get("reset_pose"), "mission.reset_pose")
    reset_x, reset_y = _map_pose(
        reset_pose, "mission.reset_pose", require_yaw=True
    )
    rows = mission.get("legs")
    if not isinstance(rows, list) or len(rows) != len(FULL_HOUSE_LEGS):
        raise V6ContractError("mission.legs must contain exactly five rows")
    mission_legs: list[MissionLeg] = []
    previous_xy = (reset_x, reset_y)
    for index, row_value in enumerate(rows):
        leg = _mapping(row_value, f"mission.legs[{index}]")
        if set(leg) != {"id", "frame_id", "x", "y"}:
            raise V6ContractError(
                f"mission.legs[{index}] must contain only id/frame_id/x/y"
            )
        goal_id = str(leg.get("id", ""))
        if goal_id != FULL_HOUSE_LEGS[index]:
            raise V6ContractError(
                f"mission.legs[{index}].id must be {FULL_HOUSE_LEGS[index]}"
            )
        x, y = _map_pose(leg, f"mission.legs[{index}]", require_yaw=False)
        if math.hypot(x - previous_xy[0], y - previous_xy[1]) <= 1.0e-6:
            raise V6ContractError(f"mission.legs[{index}] is a zero-distance goal")
        mission_legs.append(MissionLeg(goal_id, "map", x, y))
        previous_xy = (x, y)

    episode_rows = raw.get("episodes")
    if not isinstance(episode_rows, list) or len(episode_rows) != 20:
        raise V6ContractError("episodes must contain exactly 20 rows")
    episodes: list[Episode] = []
    for index, row_value in enumerate(episode_rows):
        row = _mapping(row_value, f"episodes[{index}]")
        seed = row.get("seed")
        if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
            raise V6ContractError(f"episodes[{index}].seed must be non-negative int")
        episodes.append(
            Episode(
                seed=seed,
                variant_id=str(row.get("variant_id", "")),
                appearance_profile_id=(
                    str(row["appearance_profile_id"])
                    if row.get("appearance_profile_id") is not None
                    else None
                ),
                reset_pose_name=str(row.get("reset_pose_name", "")),
                dynamic_case_id=str(row.get("dynamic_case_id", "")),
            )
        )
    return Manifest(
        path=manifest_path,
        raw=raw,
        scene_id=str(scene.get("id", "")),
        category=category,
        phase1_enabled=phase1_enabled,
        scenario_intent=scenario_intent,
        estimated_policy=_estimated_policy(runtime),
        frozen=frozen,
        reset_pose=reset_pose,
        mission_legs=tuple(mission_legs),
        episodes=tuple(episodes),
        missing_required_values=_missing_required_values(raw),
    )


def authorize_manifest(
    manifest: Manifest,
    *,
    mode: str,
    allow_engineering_policy_override: bool = False,
) -> str:
    """Return the evidence label without mutating a ROS graph."""

    if mode not in {"formal", "pilot"}:
        raise V6ContractError("mode must be formal or pilot")
    mismatches = _estimated_policy_mismatches(manifest)
    if mode == "formal":
        if allow_engineering_policy_override:
            raise V6ContractError("estimated policy override is pilot-only")
        if not manifest.phase1_enabled:
            raise V6ContractError("formal dispatch refused: Phase 1 is disabled")
        if mismatches:
            raise V6ContractError(
                "formal dispatch refused: final Estimated policy mismatch: "
                + ", ".join(mismatches)
            )
        if not manifest.frozen:
            raise V6ContractError(
                "formal dispatch refused: scene_contract_frozen is false"
            )
        if manifest.missing_required_values:
            raise V6ContractError(
                "formal dispatch refused: missing "
                + ", ".join(manifest.missing_required_values)
            )
        return "FORMAL_ELIGIBLE"
    if mismatches and not allow_engineering_policy_override:
        raise V6ContractError(
            "pilot Estimated policy mismatch requires explicit engineering override: "
            + ", ".join(mismatches)
        )
    return NOT_QUALIFIED


@dataclass
class ReadinessFacts:
    reset_service_ready: bool = False
    relocalize_service_ready: bool = False
    reset_event_publisher_ready: bool = False
    reset_subscriber_roster_ready: bool = False
    route_goal_subscriber_ready: bool = False
    flatscan_publisher_ready: bool = False
    localization_result_publisher_ready: bool = False
    localization_status_publisher_ready: bool = False
    clock_seen: bool = False
    scan_seen: bool = False
    flatscan_seen: bool = False
    map_seen: bool = False
    navigation_graph_seen: bool = False
    estimated_odom_seen: bool = False
    localization_status_seen: bool = False

    def missing(self) -> tuple[str, ...]:
        required_before_reset = (
            "reset_service_ready",
            "relocalize_service_ready",
            "reset_event_publisher_ready",
            "reset_subscriber_roster_ready",
            "flatscan_publisher_ready",
            "localization_result_publisher_ready",
            "localization_status_publisher_ready",
            "clock_seen",
            "scan_seen",
            "flatscan_seen",
            "map_seen",
            "estimated_odom_seen",
        )
        return tuple(
            name for name in required_before_reset if not getattr(self, name)
        )


@dataclass
class EpisodeGuard:
    """One reset, one new Grid generation, then five ordered XY goals."""

    state: str = "WAITING_READINESS"
    stop_reason: str = ""
    reset_calls: int = 0
    reset_events: int = 0
    localization_accepted_floor: int = 0
    localization_generation: int | None = None
    localization_waiting_seen: bool = False
    localization_accepted: bool = False
    localization_correction_ready: bool = False
    nav2_active: bool = False
    tf_active: bool = False
    route_ready: bool = False
    publisher_ownership_ready: bool = False
    reset_gate_generation: int | None = None
    reset_gate_released_generation: int | None = None
    goal_publications: int = 0
    route_progress_messages: int = 0
    route_completion_messages: int = 0
    mission_leg_ids: tuple[str, ...] = FULL_HOUSE_LEGS
    completed_leg_ids: list[str] = field(default_factory=list)
    current_leg_progress_messages: int = 0

    def stop(self, reason: str) -> None:
        if self.state != "STOP":
            self.state = "STOP"
            self.stop_reason = reason

    def arm_reset(
        self,
        facts: ReadinessFacts,
        *,
        pre_reset_route_messages: int,
        localization_accepted_floor: int,
    ) -> None:
        missing = facts.missing()
        if missing:
            raise V6ContractError(f"reset readiness missing: {', '.join(missing)}")
        if pre_reset_route_messages:
            raise V6ContractError("pre-reset route traffic violated cold boundary")
        if self.goal_publications:
            raise V6ContractError("reset_with_active_goal_forbidden")
        if self.reset_calls:
            raise V6ContractError("reset_retry_forbidden")
        self.localization_accepted_floor = int(localization_accepted_floor)
        self.state = "RESET_ARMED"

    def record_reset_call(self) -> None:
        if self.state != "RESET_ARMED" or self.reset_calls:
            self.stop("reset_retry_forbidden")
            raise V6ContractError("reset_retry_forbidden")
        self.reset_calls = 1
        self.state = "RESET_IN_FLIGHT"

    def record_reset_response(self, success: bool | None) -> None:
        if self.reset_calls != 1 or self.state == "STOP":
            self.stop("unexpected_reset_response")
            return
        if success is not True:
            self.stop("reset_response_unknown" if success is None else "reset_rejected")
            return
        self.state = "WAITING_GRID_LOCALIZATION"
        self._maybe_goal_ready()

    def record_reset_event(self) -> None:
        self.reset_events += 1
        if self.reset_events > 1:
            self.stop("second_reset_event")
        elif self.reset_calls != 1:
            self.stop("reset_event_without_call")
        else:
            self.state = "WAITING_GRID_LOCALIZATION"

    def record_localization_status(
        self,
        generation: int,
        state: str,
        accepted: bool,
        *,
        correction_ready: bool = False,
    ) -> None:
        if not self.reset_calls or self.state == "STOP":
            return
        if generation <= self.localization_accepted_floor:
            return
        if state in {"WAITING_FOR_SCAN", "WAITING_FOR_RESULT"}:
            if self.localization_generation is None:
                self.localization_generation = generation
            if generation == self.localization_generation:
                self.localization_waiting_seen = True
            return
        if state == "REJECTED" and generation == self.localization_generation:
            self.stop(f"grid_localization_rejected:{generation}")
            return
        if state == "ACCEPTED" and accepted:
            if not self.localization_waiting_seen:
                return
            if generation != self.localization_generation:
                return
            if not correction_ready:
                return
            self.localization_accepted = True
            self.localization_correction_ready = True
            self._maybe_goal_ready()

    def record_navigation_ready(
        self,
        *,
        nav2_active: bool,
        tf_active: bool,
        route_ready: bool,
        publisher_ownership_ready: bool,
    ) -> None:
        self.nav2_active = bool(nav2_active)
        self.tf_active = bool(tf_active)
        self.route_ready = bool(route_ready)
        self.publisher_ownership_ready = bool(publisher_ownership_ready)
        self._maybe_goal_ready()

    def record_reset_receipt_generation(self, generation: int) -> None:
        self.reset_gate_generation = int(generation)
        self._maybe_goal_ready()

    def record_reset_gate_status(self, generation: int, held: bool) -> None:
        if not held:
            self.reset_gate_released_generation = int(generation)
        self._maybe_goal_ready()

    @property
    def reset_gate_released(self) -> bool:
        return (
            self.reset_gate_generation is not None
            and self.reset_gate_released_generation == self.reset_gate_generation
        )

    def _maybe_goal_ready(self) -> None:
        if self.state in {
            "STOP",
            "NAVIGATING",
            "LEG_SUCCEEDED",
            "SUCCEEDED",
            "FAILED",
        }:
            return
        if all(
            (
                self.reset_calls == 1,
                self.reset_events == 1,
                self.localization_waiting_seen,
                self.localization_accepted,
                self.localization_correction_ready,
                self.nav2_active,
                self.tf_active,
                self.route_ready,
                self.publisher_ownership_ready,
                self.reset_gate_released,
            )
        ):
            self.state = "GOAL_READY"

    @property
    def goal_ready(self) -> bool:
        return self.state == "GOAL_READY" and not self.stop_reason

    def record_goal_publication(self, goal_id: str) -> None:
        first_leg = self.goal_publications == 0
        if (first_leg and not self.goal_ready) or (
            not first_leg and self.state != "LEG_SUCCEEDED"
        ):
            self.stop("route_goal_publication_not_authorized")
            raise V6ContractError(self.stop_reason)
        if self.goal_publications >= len(self.mission_leg_ids):
            self.stop("extra_route_goal_publication")
            raise V6ContractError(self.stop_reason)
        expected = self.mission_leg_ids[self.goal_publications]
        if goal_id != expected:
            self.stop(f"mission_leg_order:{goal_id}!={expected}")
            raise V6ContractError(self.stop_reason)
        self.goal_publications += 1
        self.current_leg_progress_messages = 0
        self.state = "NAVIGATING"

    def record_route_progress(self) -> None:
        if self.state == "NAVIGATING":
            self.route_progress_messages += 1
            self.current_leg_progress_messages += 1

    def record_route_completion(self, succeeded: bool) -> None:
        if self.state != "NAVIGATING":
            return
        self.route_completion_messages += 1
        if not self.current_leg_progress_messages:
            self.stop("route_completed_without_progress")
        elif not succeeded:
            self.state = "FAILED"
        else:
            self.completed_leg_ids.append(
                self.mission_leg_ids[self.goal_publications - 1]
            )
            self.state = (
                "SUCCEEDED"
                if self.goal_publications == len(self.mission_leg_ids)
                else "LEG_SUCCEEDED"
            )


def _message_summary(message: Any) -> dict[str, Any]:
    summary: dict[str, Any] = {"type": type(message).__name__}
    for name in (
        "sequence",
        "graph_id",
        "revision",
        "accepted",
        "success",
        "reason",
        "data",
    ):
        if hasattr(message, name):
            value = getattr(message, name)
            if isinstance(value, (str, bool, int, float)) or value is None:
                summary[name] = value
    return summary


class V6FormalNode:
    """ROS adapter for the canonical Phase-1 full-house episode."""

    def __init__(
        self,
        manifest: Manifest,
        episode: Episode,
        output_jsonl: Path,
        *,
        qualification: str = "FORMAL_ELIGIBLE",
    ):
        import rclpy
        from action_msgs.srv import CancelGoal
        from bio_nav_interfaces.msg import (
            CanonicalRoute,
            NavigationGraph,
            RouteProgress,
        )
        from diagnostic_msgs.msg import DiagnosticArray
        from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
        from nav_msgs.msg import OccupancyGrid, Odometry
        from rosgraph_msgs.msg import Clock
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.parameter_client import AsyncParameterClient
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from rosidl_runtime_py.utilities import get_message
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import Bool, Empty, String
        from std_srvs.srv import Trigger
        from tf2_msgs.msg import TFMessage

        class _Node(Node):
            pass

        FlatScan = get_message(
            "isaac_ros_pointcloud_interfaces/msg/FlatScan"
        )
        self._rclpy = rclpy
        self.node = _Node("bio_nav_v6_grid_phase1_episode")
        self.manifest = manifest
        self.episode = episode
        self.output_jsonl = output_jsonl
        self.qualification = qualification
        self.guard = EpisodeGuard(
            mission_leg_ids=tuple(item.goal_id for item in manifest.mission_legs)
        )
        self.facts = ReadinessFacts()
        self.pre_reset_route_messages = 0
        self.pre_reset_quiet_since: float | None = None
        self.latest_accepted_localization_generation = 0
        self._cmd_window: deque[tuple[float, bool]] = deque()
        self._odom_window: deque[tuple[float, float, float]] = deque()
        self.post_reset_odom_xy: list[tuple[float, float]] = []
        self.map_odom_tf_seen = False
        self.odom_base_tf_seen = False
        self.canonical_route_count = 0
        self.collision = False
        self.route_goal_results: list[dict[str, Any]] = []
        self.reset_receipt: dict[str, Any] | None = None
        self._terminal_cancel_requested = False
        self._terminal_cancel_future = None
        self._types = {
            "CancelGoal": CancelGoal,
            "PoseStamped": PoseStamped,
            "Trigger": Trigger,
            "Parameter": Parameter,
        }
        reliable = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        sensor = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.route_goal_publisher = self.node.create_publisher(
            PoseStamped, "/bio_nav/route_goal", reliable
        )
        self.reset_client = self.node.create_client(Trigger, "/simulation/reset")
        self.relocalize_client = self.node.create_client(
            Trigger, "/bio_nav/relocalize"
        )
        self.nav2_active_client = self.node.create_client(
            Trigger, "/lifecycle_manager_navigation/is_active"
        )
        self.navigate_cancel_client = self.node.create_client(
            CancelGoal, "/navigate_to_pose/_action/cancel_goal"
        )
        self.isaac_parameters = AsyncParameterClient(
            self.node, "/isaac_navigation_sim"
        )

        def sub(message_type, topic, callback, qos=reliable):
            return self.node.create_subscription(message_type, topic, callback, qos)

        self.subscriptions = [
            sub(
                Clock,
                "/clock",
                lambda m: self._fact("clock_seen", "/clock", m),
                sensor,
            ),
            sub(
                LaserScan,
                "/scan",
                lambda m: self._fact("scan_seen", "/scan", m),
                sensor,
            ),
            sub(
                FlatScan,
                "/flatscan",
                lambda m: self._fact("flatscan_seen", "/flatscan", m),
            ),
            sub(
                PoseWithCovarianceStamped,
                "/localization_result",
                self._localization_result,
            ),
            sub(
                DiagnosticArray,
                "/bio_nav/localization/status",
                self._localization_status,
                latched,
            ),
            sub(Odometry, "/odom", self._odom, sensor),
            sub(TFMessage, "/tf", self._tf),
            sub(TFMessage, "/tf_static", self._tf, latched),
            sub(
                OccupancyGrid,
                "/map",
                lambda m: self._fact("map_seen", "/map", m),
                latched,
            ),
            sub(Empty, "/simulation/reset_event", self._reset_event),
            sub(
                String,
                "/simulation/reset_stop_gate/status",
                self._reset_gate_status,
                latched,
            ),
            sub(
                NavigationGraph,
                "/bio_nav/navigation_graph",
                lambda m: self._fact(
                    "navigation_graph_seen", "/bio_nav/navigation_graph", m
                ),
                latched,
            ),
            sub(
                CanonicalRoute,
                "/bio_nav/canonical_route",
                self._canonical_route,
                latched,
            ),
            sub(RouteProgress, "/bio_nav/route_progress", self._route_progress),
            sub(Bool, "/bio_nav/route_goal_complete", self._route_complete),
            sub(String, "/bio_nav/route_goal_result", self._route_result),
            sub(Twist, "/cmd_vel", lambda m: self._track_command("/cmd_vel", m)),
            sub(
                Twist,
                "/cmd_vel_nav",
                lambda m: self._track_command("/cmd_vel_nav", m),
            ),
            sub(
                Twist,
                "/cmd_vel_smoothed",
                lambda m: self._track_command("/cmd_vel_smoothed", m),
            ),
            sub(
                Twist,
                "/cmd_vel_sim",
                lambda m: self._track_command("/cmd_vel_sim", m),
            ),
            sub(Bool, "/simulation/collision", self._collision),
            sub(
                String,
                "/simulation/collision_diagnostics",
                self._capture_callback("/simulation/collision_diagnostics"),
            ),
            sub(
                DiagnosticArray,
                "/diagnostics",
                self._capture_callback("/diagnostics"),
            ),
        ]

    def _write(self, event: str, **payload: Any) -> None:
        append_evidence_jsonl(self.output_jsonl, event, **payload)

    def _capture(self, topic: str, message: Any) -> None:
        self._write("topic", topic=topic, message=_message_summary(message))

    def _capture_callback(self, topic: str):
        return lambda message: self._capture(topic, message)

    def _fact(self, name: str, topic: str, message: Any) -> None:
        setattr(self.facts, name, True)
        self._capture(topic, message)

    def _odom(self, message: Any) -> None:
        self.facts.estimated_odom_seen = True
        now = time.monotonic()
        x = float(message.pose.pose.position.x)
        y = float(message.pose.pose.position.y)
        self._odom_window.append((now, x, y))
        while self._odom_window and now - self._odom_window[0][0] > 4.0:
            self._odom_window.popleft()
        if self.guard.reset_events == 1 and not self.guard.goal_publications:
            self.post_reset_odom_xy.append((x, y))
        self._capture("/odom", message)

    def _track_command(self, topic: str, message: Any) -> None:
        nonzero = any(
            abs(float(value)) > COMMAND_ZERO_TOLERANCE
            for value in (
                message.linear.x,
                message.linear.y,
                message.angular.z,
            )
        )
        now = time.monotonic()
        self._cmd_window.append((now, nonzero))
        while self._cmd_window and now - self._cmd_window[0][0] > 4.0:
            self._cmd_window.popleft()
        if nonzero and self.guard.reset_calls and not self.guard.goal_publications:
            self.guard.stop(f"post_reset_command_nonzero:{topic}")
        self._capture(topic, message)

    def _reset_event(self, message: Any) -> None:
        self.guard.record_reset_event()
        self._capture("/simulation/reset_event", message)

    def _reset_gate_status(self, message: Any) -> None:
        if self.guard.reset_calls:
            try:
                document = json.loads(str(message.data))
                generation = document["generation"]
                held = document["held"]
                valid = (
                    not isinstance(generation, bool)
                    and isinstance(generation, int)
                    and generation >= 0
                    and isinstance(held, bool)
                )
            except (AttributeError, KeyError, TypeError, json.JSONDecodeError):
                valid = False
            if not valid:
                self.guard.stop("reset_gate_status_invalid")
                return
            self.guard.record_reset_gate_status(generation, held)
        self._capture("/simulation/reset_stop_gate/status", message)

    @staticmethod
    def _diagnostic_values(status: Any) -> dict[str, str]:
        return {str(item.key): str(item.value) for item in status.values}

    def _localization_status(self, message: Any) -> None:
        for status in message.status:
            if str(status.name) != "grid_localization":
                continue
            values = self._diagnostic_values(status)
            try:
                generation = int(values["generation"])
            except (KeyError, ValueError):
                self.guard.stop("grid_localization_status_invalid")
                continue
            state = values.get("state", "")
            accepted = values.get("accepted", "").lower() == "true"
            self.facts.localization_status_seen = True
            if state == "ACCEPTED" and accepted:
                self.latest_accepted_localization_generation = max(
                    self.latest_accepted_localization_generation, generation
                )
            correction_ready = self._matching_correction_ready(values)
            was_accepted = self.guard.localization_accepted
            if (
                state == "ACCEPTED"
                and accepted
                and correction_ready
                and not was_accepted
            ):
                # Start the post-accept readiness epoch before admitting the
                # generation. Old TF/Nav2/route facts cannot make the guard
                # momentarily GOAL_READY.
                self.map_odom_tf_seen = False
                self.odom_base_tf_seen = False
                self.guard.record_navigation_ready(
                    nav2_active=False,
                    tf_active=False,
                    route_ready=False,
                    publisher_ownership_ready=False,
                )
            self.guard.record_localization_status(
                generation,
                state,
                accepted,
                correction_ready=correction_ready,
            )
        self._capture("/bio_nav/localization/status", message)

    @staticmethod
    def _matching_correction_ready(values: Mapping[str, str]) -> bool:
        try:
            expected_stamp_ns = int(values["expected_result_stamp_ns"])
            result_stamp_ns = int(values["result_stamp_ns"])
            correction = tuple(
                float(values[name])
                for name in (
                    "correction_x_m",
                    "correction_y_m",
                    "correction_yaw_rad",
                )
            )
        except (KeyError, TypeError, ValueError):
            return False
        return (
            expected_stamp_ns > 0
            and result_stamp_ns == expected_stamp_ns
            and all(math.isfinite(value) for value in correction)
        )

    def _localization_result(self, message: Any) -> None:
        self._capture("/localization_result", message)

    def _tf(self, message: Any) -> None:
        for transform in message.transforms:
            parent = str(transform.header.frame_id).lstrip("/")
            child = str(transform.child_frame_id).lstrip("/")
            self.map_odom_tf_seen |= parent == "map" and child == "odom"
            self.odom_base_tf_seen |= parent == "odom" and child in {
                "base_link",
                "base_footprint",
            }

    def _canonical_route(self, message: Any) -> None:
        self.canonical_route_count += 1
        self._capture("/bio_nav/canonical_route", message)

    def _route_progress(self, message: Any) -> None:
        self._track_route_signal("route_progress")
        self.guard.record_route_progress()
        self._capture("/bio_nav/route_progress", message)

    def _route_complete(self, message: Any) -> None:
        self._track_route_signal("route_goal_complete")
        self.guard.record_route_completion(bool(message.data))
        if self.guard.state in {"FAILED", "STOP"}:
            self._cancel_active_navigation_once(
                self.guard.stop_reason or "route_failed"
            )
        self._capture("/bio_nav/route_goal_complete", message)

    def _route_result(self, message: Any) -> None:
        self._track_route_signal("route_goal_result")
        self.route_goal_results.append(self._json_message(message))
        self._capture("/bio_nav/route_goal_result", message)

    def _track_route_signal(self, kind: str) -> None:
        if not self.guard.reset_calls:
            self.pre_reset_route_messages += 1
        elif not self.guard.goal_publications:
            self.guard.stop(f"stale_{kind}_after_reset")

    @staticmethod
    def _json_message(message: Any) -> dict[str, Any]:
        try:
            value = json.loads(str(message.data))
        except (AttributeError, json.JSONDecodeError):
            return {"raw": str(getattr(message, "data", ""))}
        return value if isinstance(value, dict) else {"value": value}

    def _collision(self, message: Any) -> None:
        self.collision = self.collision or bool(message.data)
        if message.data:
            self.guard.stop("collision")
            self._cancel_active_navigation_once("collision")
        self._capture("/simulation/collision", message)

    def _cancel_active_navigation_once(self, reason: str) -> None:
        active_goal = (
            self.guard.goal_publications > len(self.guard.completed_leg_ids)
        )
        if self._terminal_cancel_requested or not active_goal:
            return
        self._terminal_cancel_requested = True
        CancelGoal = self._types["CancelGoal"]
        self._terminal_cancel_future = self.navigate_cancel_client.call_async(
            CancelGoal.Request()
        )
        self._write("terminal_navigation_cancel_requested", reason=reason)

    def _spin_until(self, predicate, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while self._rclpy.ok() and time.monotonic() < deadline:
            self._rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return bool(predicate())

    def _refresh_endpoint_facts(self) -> None:
        by_topic = {sub.topic_name: sub for sub in self.subscriptions}
        self.facts.reset_service_ready = self.reset_client.service_is_ready()
        self.facts.relocalize_service_ready = (
            self.relocalize_client.service_is_ready()
        )
        self.facts.reset_event_publisher_ready = (
            by_topic["/simulation/reset_event"].get_publisher_count() > 0
        )
        self.facts.reset_subscriber_roster_ready = (
            self.node.count_subscribers("/simulation/reset_event") >= 3
        )
        self.facts.route_goal_subscriber_ready = (
            self.route_goal_publisher.get_subscription_count() > 0
        )
        self.facts.flatscan_publisher_ready = (
            by_topic["/flatscan"].get_publisher_count() > 0
        )
        self.facts.localization_result_publisher_ready = (
            by_topic["/localization_result"].get_publisher_count() > 0
        )
        self.facts.localization_status_publisher_ready = (
            by_topic["/bio_nav/localization/status"].get_publisher_count() > 0
        )

    def _publisher_ownership_violations(self) -> tuple[str, ...]:
        return tuple(
            f"{topic}={self.node.count_publishers(topic)}"
            for topic in SOLE_PUBLISHER_TOPICS
            if self.node.count_publishers(topic) != 1
        )

    def _pre_reset_still(self) -> bool:
        now = time.monotonic()
        horizon = now - PRE_RESET_NEGATIVE_WINDOW_S
        if any(nonzero for stamp, nonzero in self._cmd_window if stamp >= horizon):
            return False
        window = [
            (x, y) for stamp, x, y in self._odom_window if stamp >= horizon
        ]
        if not window:
            return False
        xs = [point[0] for point in window]
        ys = [point[1] for point in window]
        return (
            max(xs) - min(xs) <= PRE_RESET_STILL_SPAN_M
            and max(ys) - min(ys) <= PRE_RESET_STILL_SPAN_M
        )

    def _readiness_blockers(self) -> str:
        blockers: list[str] = []
        if self.facts.missing():
            blockers.append("facts:" + ",".join(self.facts.missing()))
        if self.pre_reset_route_messages:
            blockers.append("pre_reset_route_traffic")
        ownership = tuple(
            violation
            for violation in self._publisher_ownership_violations()
            if violation.startswith("/odom=")
        )
        if ownership:
            blockers.append("publisher_ownership:" + ",".join(ownership))
        if not self._pre_reset_still():
            blockers.append("pre_reset_not_still")
        return ";".join(blockers)

    def _pre_reset_ready(self) -> bool:
        self._refresh_endpoint_facts()
        if self._readiness_blockers():
            self.pre_reset_quiet_since = None
            return False
        now = time.monotonic()
        if self.pre_reset_quiet_since is None:
            self.pre_reset_quiet_since = now
            return False
        return now - self.pre_reset_quiet_since >= PRE_RESET_NEGATIVE_WINDOW_S

    def _check_post_reset_odom(self) -> None:
        samples = self.post_reset_odom_xy[1:]
        if len(samples) < 2:
            self.guard.stop("post_reset_odom_missing")
            return
        landing = math.hypot(samples[0][0], samples[0][1])
        if landing > POST_RESET_ODOM_LANDING_M:
            self.guard.stop(f"post_reset_odom_landing:{landing:.3f}")
            return
        xs = [point[0] for point in samples]
        ys = [point[1] for point in samples]
        span = max(max(xs) - min(xs), max(ys) - min(ys))
        if span > POST_RESET_ODOM_SPAN_M:
            self.guard.stop(f"post_reset_odom_span:{span:.3f}")

    def _nav2_is_active(self, timeout_sec: float) -> bool:
        if not self.nav2_active_client.wait_for_service(timeout_sec=timeout_sec):
            return False
        Trigger = self._types["Trigger"]
        future = self.nav2_active_client.call_async(Trigger.Request())
        if not self._spin_until(future.done, timeout_sec):
            return False
        response = future.result()
        return bool(response is not None and response.success is True)

    def _wait_nav2_and_tf_ready(self, timeout_sec: float) -> None:
        deadline = time.monotonic() + timeout_sec
        while True:
            self.guard.record_navigation_ready(
                nav2_active=self._nav2_is_active(
                    NAV2_PROBE_ATTEMPT_TIMEOUT_SEC
                ),
                tf_active=self.map_odom_tf_seen and self.odom_base_tf_seen,
                route_ready=(
                    self.route_goal_publisher.get_subscription_count() > 0
                    and self.facts.navigation_graph_seen
                ),
                publisher_ownership_ready=(
                    not self._publisher_ownership_violations()
                ),
            )
            if all(
                (
                    self.guard.nav2_active,
                    self.guard.tf_active,
                    self.guard.route_ready,
                    self.guard.publisher_ownership_ready,
                )
            ):
                return
            if self.guard.state == "STOP" or time.monotonic() >= deadline:
                self.guard.stop("nav2_or_tf_not_ready")
                return
            self._rclpy.spin_once(self.node, timeout_sec=0.5)

    def _set_episode_parameters(self, timeout_sec: float) -> None:
        Parameter = self._types["Parameter"]
        if not self.isaac_parameters.wait_for_services(timeout_sec=timeout_sec):
            raise V6ContractError("Isaac reset parameter services unavailable")
        params = [
            Parameter("reset_seed", value=self.episode.seed),
            Parameter("reset_pose_name", value=self.episode.reset_pose_name),
            Parameter("dynamic_case_id", value=""),
            Parameter("dynamic_variant_id", value=""),
        ]
        future = self.isaac_parameters.set_parameters(params)
        if not self._spin_until(future.done, timeout_sec):
            raise V6ContractError("setting episode parameters timed out")
        response = future.result()
        if response is None or any(
            not result.successful for result in response.results
        ):
            raise V6ContractError("Isaac rejected episode parameters")
        self._write("episode_parameters_set", seed=self.episode.seed)

    def _goal_message(self, leg: MissionLeg):
        PoseStamped = self._types["PoseStamped"]
        message = PoseStamped()
        message.header.frame_id = leg.frame_id
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.pose.position.x = leg.x
        message.pose.position.y = leg.y
        message.pose.orientation.w = 1.0
        return message

    def run(
        self,
        *,
        readiness_timeout_sec: float,
        reset_timeout_sec: float,
        navigation_timeout_sec: float,
    ) -> dict[str, Any]:
        if not self.manifest.phase1_enabled:
            raise V6ContractError(
                "Phase 1 dispatch is disabled for this scenario intent"
            )
        self._write(
            "episode_start",
            qualification=self.qualification,
            formal_qualification=(
                NOT_QUALIFIED
                if self.qualification == ENGINEERING_PILOT
                else "FORMAL_ELIGIBLE"
            ),
            manifest=str(self.manifest.path),
            seed=self.episode.seed,
            runtime=dict(self.manifest.raw["runtime"]),
        )
        if not self._spin_until(self._pre_reset_ready, readiness_timeout_sec):
            self.guard.stop(
                "readiness_timeout:"
                + (self._readiness_blockers() or "unknown")
            )
            return self.result()
        self.guard.arm_reset(
            self.facts,
            pre_reset_route_messages=self.pre_reset_route_messages,
            localization_accepted_floor=(
                self.latest_accepted_localization_generation
            ),
        )
        self._set_episode_parameters(reset_timeout_sec)

        Trigger = self._types["Trigger"]
        self.guard.record_reset_call()
        self.map_odom_tf_seen = False
        self.odom_base_tf_seen = False
        self._write(
            "reset_epoch_started",
            accepted_generation_floor=(
                self.guard.localization_accepted_floor
            ),
        )
        future = self.reset_client.call_async(Trigger.Request())
        if not self._spin_until(future.done, reset_timeout_sec):
            self.guard.record_reset_response(None)
            return self.result()
        response = future.result()
        self.guard.record_reset_response(
            response.success if response is not None else None
        )
        if self.guard.state == "STOP":
            return self.result()
        try:
            self.reset_receipt = parse_reset_receipt(
                response.message,
                requested_seed=self.episode.seed,
                requested_case_id="",
                requested_variant_id="",
                requested_pose=self.episode.reset_pose_name,
            )
        except ResetReceiptError as exc:
            self.guard.stop(f"reset_receipt_mismatch:{exc}")
            return self.result()
        self._write("reset_receipt", **self.reset_receipt)
        self.guard.record_reset_receipt_generation(
            int(self.reset_receipt["generation"])
        )

        if not self._spin_until(
            lambda: self.guard.localization_accepted
            or self.guard.state == "STOP",
            reset_timeout_sec,
        ):
            self.guard.stop("grid_localization_acceptance_timeout")
            return self.result()
        if self.guard.state == "STOP":
            return self.result()
        self._check_post_reset_odom()
        if self.guard.state == "STOP":
            return self.result()
        self._wait_nav2_and_tf_ready(reset_timeout_sec)
        if self.guard.state == "STOP":
            return self.result()
        if not self._spin_until(
            lambda: self.guard.goal_ready or self.guard.state == "STOP",
            reset_timeout_sec,
        ):
            self.guard.stop("activation_gate_release_timeout")
            return self.result()
        if self.guard.state == "STOP":
            return self.result()

        for index, leg in enumerate(self.manifest.mission_legs):
            route_baseline = self.canonical_route_count
            result_baseline = len(self.route_goal_results)
            self.guard.record_goal_publication(leg.goal_id)
            self.route_goal_publisher.publish(self._goal_message(leg))
            self._write(
                "route_goal_published",
                topic="/bio_nav/route_goal",
                leg_id=leg.goal_id,
                leg_index=index,
                orientation_placeholder_w=1.0,
            )
            completed = self._spin_until(
                lambda: self.guard.state
                in {"LEG_SUCCEEDED", "SUCCEEDED", "FAILED", "STOP"},
                navigation_timeout_sec,
            )
            if not completed:
                self.guard.stop(f"route_completion_timeout:{leg.goal_id}")
            if self.canonical_route_count <= route_baseline:
                self.guard.stop(f"canonical_route_missing:{leg.goal_id}")
            self._write(
                "mission_leg_result",
                leg_id=leg.goal_id,
                state=self.guard.state,
                route_progress_messages=(
                    self.guard.current_leg_progress_messages
                ),
                route_result_messages=(
                    len(self.route_goal_results) - result_baseline
                ),
            )
            if self.guard.state not in {"LEG_SUCCEEDED", "SUCCEEDED"}:
                break
        return self.result()

    def result(self) -> dict[str, Any]:
        if self.guard.state in {"FAILED", "STOP"}:
            self._cancel_active_navigation_once(
                self.guard.stop_reason or "route_failed"
            )
        row = {
            "qualification": self.qualification,
            "formal_qualification": (
                NOT_QUALIFIED
                if self.qualification == ENGINEERING_PILOT
                else "FORMAL_ELIGIBLE"
            ),
            "state": self.guard.state,
            "stop_reason": self.guard.stop_reason,
            "reset_calls": self.guard.reset_calls,
            "reset_events": self.guard.reset_events,
            "reset_receipt": dict(self.reset_receipt or {}),
            "localization_generation": self.guard.localization_generation,
            "localization_accepted": self.guard.localization_accepted,
            "goal_publications": self.guard.goal_publications,
            "route_progress_messages": self.guard.route_progress_messages,
            "route_completion_messages": self.guard.route_completion_messages,
            "completed_leg_ids": list(self.guard.completed_leg_ids),
            "route_goal_results": list(self.route_goal_results),
            "collision": self.collision,
        }
        self._write("episode_result", **row)
        return row

    def destroy(self) -> None:
        self.node.destroy_node()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--episode-index", type=int, default=0)
    parser.add_argument("--pilot", action="store_true")
    parser.add_argument("--dispatch-pilot", action="store_true")
    parser.add_argument(
        "--allow-engineering-estimated-policy-override", action="store_true"
    )
    parser.add_argument("--allow-formal-dispatch", action="store_true")
    parser.add_argument("--output-jsonl")
    parser.add_argument("--readiness-timeout-sec", type=float, default=120.0)
    parser.add_argument("--reset-timeout-sec", type=float, default=120.0)
    parser.add_argument("--navigation-timeout-sec", type=float, default=900.0)
    return parser


def cli(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.dispatch_pilot and not args.pilot:
            raise V6ContractError("--dispatch-pilot requires --pilot")
        if args.allow_engineering_estimated_policy_override and not args.pilot:
            raise V6ContractError(
                "--allow-engineering-estimated-policy-override requires --pilot"
            )
        manifest = load_manifest(args.manifest)
        qualification = authorize_manifest(
            manifest,
            mode="pilot" if args.pilot else "formal",
            allow_engineering_policy_override=(
                args.allow_engineering_estimated_policy_override
            ),
        )
        if args.pilot and not args.dispatch_pilot:
            print(
                json.dumps(
                    {
                        "qualification": ENGINEERING_PILOT,
                        "formal_qualification": qualification,
                        "dispatch": False,
                        "phase1_enabled": manifest.phase1_enabled,
                        "runtime": dict(manifest.raw["runtime"]),
                        "mission_leg_ids": [
                            leg.goal_id for leg in manifest.mission_legs
                        ],
                    },
                    sort_keys=True,
                )
            )
            return 0
        if not manifest.phase1_enabled:
            raise V6ContractError(
                "Phase 1 dispatch is disabled for this scenario intent"
            )
        if not args.pilot and not args.allow_formal_dispatch:
            raise V6ContractError("formal dispatch requires --allow-formal-dispatch")
        if args.output_jsonl is None:
            raise V6ContractError("dispatch requires --output-jsonl")
        if not 0 <= args.episode_index < len(manifest.episodes):
            raise V6ContractError("episode-index out of range")
        import rclpy

        rclpy.init(args=None)
        adapter = V6FormalNode(
            manifest,
            manifest.episodes[args.episode_index],
            Path(args.output_jsonl).expanduser().resolve(),
            qualification=(
                ENGINEERING_PILOT if args.pilot else "FORMAL_ELIGIBLE"
            ),
        )
        try:
            result = adapter.run(
                readiness_timeout_sec=args.readiness_timeout_sec,
                reset_timeout_sec=args.reset_timeout_sec,
                navigation_timeout_sec=args.navigation_timeout_sec,
            )
            print(json.dumps(result, sort_keys=True))
            return 0 if result["state"] == "SUCCEEDED" else 2
        finally:
            adapter.destroy()
            rclpy.shutdown()
    except (OSError, V6ContractError, yaml.YAMLError) as exc:
        print(f"STOP: {exc}", file=sys.stderr)
        return 2


def main() -> None:
    raise SystemExit(cli())


if __name__ == "__main__":
    main()
