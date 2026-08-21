"""V6 estimated-autonomy episode dispatcher and engineering pilot adapter.

This module deliberately does not share the legacy experiment runner.  The
dispatcher owns reset and RouteCoordinator goal sequencing only.  Ground Truth
is reserved for the independent ``estimated_state_evaluator``/recorder.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import sys
import time
from typing import Any, Mapping

import yaml


SCHEMA_VERSION = "bio_nav_v6_single_episode_manifest_v1"
NOT_QUALIFIED = "NOT_QUALIFIED"
ENGINEERING_PILOT = "ENGINEERING_PILOT"
GT_PREFIX = "/" + "ground_truth/"

# Runtime subscriptions are a reviewable firewall.  Keep Ground Truth in the
# passive evaluator, never in this dispatcher.
DISPATCH_SUBSCRIPTION_TOPICS = (
    "/odom",
    "/amcl_pose",
    "/map",
    "/simulation/reset_event",
    "/simulation/localization_seeded",
    "/bio_nav/cognitive_map/constraints",
    "/bio_nav/navigation_graph",
    "/bio_nav/canonical_route",
    "/bio_nav/route_progress",
    "/bio_nav/route_goal_complete",
    "/bio_nav/route_goal_result",
    "/bio_nav/module2/cognitive_place_graph",
    "/bio_nav/module2/goal_planning_prior",
    "/bio_nav/module3/cognitive_graph_validation_ack",
    "/bio_nav/module3/cognitive_edge_outcome",
    "/bio_nav/risk_layer/status",
    "/bio_nav/local_risk_layer/status",
    "/bio_nav/cognitive_obstacle_layer/status",
    "/bio_nav/cognitive_risk_critic/status",
    "/cmd_vel",
    "/simulation/collision",
    "/simulation/collision_diagnostics",
    "/diagnostics",
    "/experiment/obstacles/state",
    "/experiment/appearance/state",
)

CAPTURE_SCHEMA = {
    "/bio_nav/module2/cognitive_place_graph": "CognitivePlaceGraphCandidate",
    "/bio_nav/module3/cognitive_graph_validation_ack": "CognitiveGraphValidationAck",
    "/bio_nav/navigation_graph": "NavigationGraph",
    "/bio_nav/canonical_route": "CanonicalRoute",
    "/bio_nav/route_progress": "RouteProgress",
    "/bio_nav/module2/goal_planning_prior": "GoalPlanningPrior",
    "/bio_nav/risk_layer/status": "RiskLayerStatus",
    "/bio_nav/local_risk_layer/status": "RiskLayerStatus",
    "/bio_nav/cognitive_obstacle_layer/status": "RiskLayerStatus",
    "/bio_nav/cognitive_risk_critic/status": "RiskLayerStatus",
    "/bio_nav/module3/cognitive_edge_outcome": "CognitiveEdgeOutcome",
    "/cmd_vel": "Twist",
    "/simulation/collision": "Bool",
    "/simulation/collision_diagnostics": "String",
    "/bio_nav/route_goal_result": "String",
    "/experiment/obstacles/state": "String",
    "/experiment/appearance/state": "String",
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
    yaw_deg: float
    dynamic_trigger_group: str


@dataclass(frozen=True)
class Manifest:
    path: Path
    raw: Mapping[str, Any]
    scene_id: str
    category: str
    frozen: bool
    reset_pose: Mapping[str, Any]
    mission_legs: tuple[MissionLeg, ...]
    episodes: tuple[Episode, ...]
    missing_required_values: tuple[str, ...]


def _mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise V6ContractError(f"{path} must be a mapping")
    return value


def _missing_required_values(raw: Mapping[str, Any]) -> tuple[str, ...]:
    required = _mapping(raw.get("required_runtime_values"), "required_runtime_values")
    missing: list[str] = []
    for name, value in required.items():
        if value is None and name == "posegraph_file" and required.get("posegraph_required") is False:
            continue
        if value is None or value == "":
            missing.append(f"required_runtime_values.{name}")
    return tuple(sorted(missing))


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


def _pose(raw: Mapping[str, Any], path: str) -> tuple[str, float, float, float]:
    frame_id = str(raw.get("frame_id", ""))
    if frame_id != "map":
        raise V6ContractError(f"{path}.frame_id must be map")
    return (
        frame_id,
        _finite_float(raw.get("x"), f"{path}.x"),
        _finite_float(raw.get("y"), f"{path}.y"),
        _finite_float(raw.get("yaw_deg"), f"{path}.yaw_deg"),
    )


def load_manifest(path: str | Path) -> Manifest:
    manifest_path = Path(path).expanduser().resolve()
    raw = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    raw = _mapping(raw, "manifest")
    if raw.get("schema_version") != SCHEMA_VERSION:
        raise V6ContractError(f"schema_version must be {SCHEMA_VERSION}")

    runtime = _mapping(raw.get("runtime"), "runtime")
    expected_runtime = {
        "startup_profile": "estimated_autonomy",
        "cognitive_mode": "active",
        "causal_level": "L3",
        "module_level": "M3",
        "route_backend": "primary",
        "structure_tf_source": "isaac",
        "ground_truth_policy": "evaluator_only",
        "direct_rgbd_costmap_enabled": False,
    }
    for name, expected in expected_runtime.items():
        if runtime.get(name) != expected:
            raise V6ContractError(f"runtime.{name} must be {expected!r}")

    frozen = raw.get("scene_contract_frozen")
    if not isinstance(frozen, bool):
        raise V6ContractError("scene_contract_frozen must be boolean")
    scene = _mapping(raw.get("scene"), "scene")
    category = str(scene.get("category", ""))
    if category not in {"static", "dynamic", "appearance"}:
        raise V6ContractError("scene.category must be static, dynamic, or appearance")

    mission = _mapping(raw.get("mission"), "mission")
    reset_pose = _mapping(mission.get("reset_pose"), "mission.reset_pose")
    _, reset_x, reset_y, _ = _pose(reset_pose, "mission.reset_pose")
    mission_rows = mission.get("legs")
    if not isinstance(mission_rows, list) or len(mission_rows) != 5:
        raise V6ContractError("mission.legs must contain exactly five rows")
    mission_legs: list[MissionLeg] = []
    previous_xy = (reset_x, reset_y)
    seen_ids: set[str] = set()
    for index, leg_value in enumerate(mission_rows):
        leg = _mapping(leg_value, f"mission.legs[{index}]")
        goal_id = str(leg.get("id", ""))
        if not goal_id or goal_id in seen_ids:
            raise V6ContractError(f"mission.legs[{index}].id must be unique and non-empty")
        frame_id, x, y, yaw_deg = _pose(leg, f"mission.legs[{index}]")
        if math.hypot(x - previous_xy[0], y - previous_xy[1]) <= 1.0e-6:
            raise V6ContractError(f"mission.legs[{index}] is a zero-distance goal")
        trigger_group = str(leg.get("dynamic_trigger_group", ""))
        if category != "dynamic" and trigger_group:
            raise V6ContractError("static/appearance mission legs cannot trigger actors")
        mission_legs.append(MissionLeg(goal_id, frame_id, x, y, yaw_deg, trigger_group))
        seen_ids.add(goal_id)
        previous_xy = (x, y)
    if category == "dynamic" and not any(row.dynamic_trigger_group for row in mission_legs):
        raise V6ContractError("dynamic mission must declare trigger groups")

    rows = raw.get("episodes")
    if not isinstance(rows, list) or len(rows) != 20:
        raise V6ContractError("episodes must contain exactly 20 rows")
    episodes: list[Episode] = []
    for index, row_value in enumerate(rows):
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
        frozen=frozen,
        reset_pose=reset_pose,
        mission_legs=tuple(mission_legs),
        episodes=tuple(episodes),
        missing_required_values=_missing_required_values(raw),
    )


def authorize_manifest(manifest: Manifest, *, mode: str) -> str:
    """Return qualification label or fail before ROS/runtime mutation."""

    if mode not in {"formal", "pilot"}:
        raise V6ContractError("mode must be formal or pilot")
    if mode == "formal":
        if not manifest.frozen:
            raise V6ContractError("formal dispatch refused: scene_contract_frozen is false")
        if manifest.missing_required_values:
            missing = ", ".join(manifest.missing_required_values)
            raise V6ContractError(f"formal dispatch refused: missing {missing}")
        return "FORMAL_ELIGIBLE"
    return NOT_QUALIFIED


@dataclass
class ReadinessFacts:
    reset_service_ready: bool = False
    reset_event_publisher_ready: bool = False
    route_goal_subscriber_ready: bool = False
    localization_publisher_ready: bool = False
    bridge_prior_publisher_ready: bool = False
    map_seen: bool = False
    constraints_seen: bool = False
    navigation_graph_seen: bool = False
    estimated_odom_seen: bool = False
    amcl_pose_seen: bool = False

    def missing(self) -> tuple[str, ...]:
        return tuple(name for name, value in vars(self).items() if not value)


@dataclass
class EpisodeGuard:
    """Pure exactly-once reset and PRIMARY RouteCoordinator state machine."""

    state: str = "WAITING_READINESS"
    stop_reason: str = ""
    reset_calls: int = 0
    reset_events: int = 0
    bridge_epoch_baseline: int | None = None
    bridge_epoch_after_reset: int | None = None
    post_reset_prior_seen: bool = False
    localization_seeded: bool = False
    goal_publications: int = 0
    route_progress_messages: int = 0
    route_completion_messages: int = 0
    route_succeeded: bool = False
    mission_leg_ids: tuple[str, ...] = ()
    completed_leg_ids: list[str] = field(default_factory=list)
    current_leg_progress_messages: int = 0

    def stop(self, reason: str) -> None:
        if self.state != "STOP":
            self.state = "STOP"
            self.stop_reason = reason

    def arm_reset(self, facts: ReadinessFacts, bridge_epoch: int | None) -> None:
        missing = facts.missing()
        if missing:
            raise V6ContractError(f"reset readiness missing: {', '.join(missing)}")
        if bridge_epoch is None:
            raise V6ContractError("reset readiness missing bridge epoch baseline")
        if self.reset_calls:
            self.stop("reset_retry_forbidden")
            raise V6ContractError(self.stop_reason)
        self.bridge_epoch_baseline = bridge_epoch
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
        if not self.goal_ready:
            self.state = "WAITING_RESET_EPOCH"

    def record_reset_event(self) -> None:
        self.reset_events += 1
        if self.reset_events > 1:
            self.stop("second_reset_event")
            return
        if self.reset_calls != 1:
            self.stop("reset_event_without_call")

    def record_prior(self, reset_epoch: int) -> None:
        if self.bridge_epoch_baseline is None:
            self.bridge_epoch_baseline = reset_epoch
            return
        if self.reset_calls != 1 or self.reset_events != 1:
            return
        expected = self.bridge_epoch_baseline + 1
        if reset_epoch != expected:
            self.stop(f"bridge_epoch_mismatch:{reset_epoch}!={expected}")
            return
        self.bridge_epoch_after_reset = reset_epoch
        self.post_reset_prior_seen = True
        if self.localization_seeded:
            self.state = "GOAL_READY"

    def record_localization_seeded(self) -> None:
        if self.reset_events != 1:
            self.stop("localization_seeded_outside_reset_epoch")
            return
        self.localization_seeded = True
        if self.post_reset_prior_seen:
            self.state = "GOAL_READY"

    @property
    def goal_ready(self) -> bool:
        return self.state == "GOAL_READY" and not self.stop_reason

    def record_goal_publication(self, goal_id: str | None = None) -> None:
        first_leg = self.goal_publications == 0
        if (first_leg and not self.goal_ready) or (not first_leg and self.state != "LEG_SUCCEEDED"):
            self.stop("route_goal_publication_not_authorized")
            raise V6ContractError(self.stop_reason)
        if self.goal_publications >= max(1, len(self.mission_leg_ids)):
            self.stop("extra_route_goal_publication")
            raise V6ContractError(self.stop_reason)
        if self.mission_leg_ids:
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
        self.route_succeeded = bool(succeeded)
        if not self.current_leg_progress_messages:
            self.stop("route_completed_without_progress")
        elif not succeeded:
            self.state = "FAILED"
        else:
            if self.mission_leg_ids:
                self.completed_leg_ids.append(
                    self.mission_leg_ids[self.goal_publications - 1]
                )
            final_leg = self.goal_publications >= max(1, len(self.mission_leg_ids))
            self.state = "SUCCEEDED" if final_leg else "LEG_SUCCEEDED"


@dataclass
class DynamicActionLedger:
    """Claim each dynamic service action once; a failed claim is never retried."""

    claimed: set[tuple[str, str]] = field(default_factory=set)
    events: list[dict[str, Any]] = field(default_factory=list)

    def claim(self, group: str, action: str) -> None:
        if action not in {"trigger", "complete"}:
            raise V6ContractError(f"unknown dynamic action {action}")
        key = (group, action)
        if key in self.claimed:
            raise V6ContractError(f"dynamic action retry forbidden: {group}/{action}")
        if action == "complete" and (group, "trigger") not in self.claimed:
            raise V6ContractError(f"dynamic completion before trigger: {group}")
        self.claimed.add(key)
        self.events.append({"group": group, "action": action, "result": "claimed"})

    def record(self, group: str, action: str, result: str, detail: str = "") -> None:
        self.events.append(
            {"group": group, "action": action, "result": result, "detail": detail}
        )


def _message_summary(message: Any) -> dict[str, Any]:
    summary: dict[str, Any] = {"type": type(message).__name__}
    for name in (
        "reset_epoch",
        "sequence",
        "request_id",
        "graph_id",
        "revision",
        "event_sequence",
        "accepted",
        "success",
        "failure",
        "reason",
        "applied",
        "rejection_mask",
        "data",
    ):
        if hasattr(message, name):
            value = getattr(message, name)
            if isinstance(value, (str, bool, int, float)) or value is None:
                summary[name] = value
    return summary


class V6FormalNode:
    """Runtime adapter, imported lazily so manifest checks need no ROS graph."""

    def __init__(
        self,
        manifest: Manifest,
        episode: Episode,
        output_jsonl: Path,
        *,
        qualification: str = "FORMAL_ELIGIBLE",
    ):
        import rclpy
        from bio_nav_interfaces.msg import (
            CanonicalRoute,
            CognitiveEdgeOutcome,
            CognitiveGraphValidationAck,
            CognitiveMapConstraints,
            CognitivePlaceGraphCandidate,
            GoalPlanningPrior,
            NavigationGraph,
            RiskLayerStatus,
            RouteProgress,
        )
        from diagnostic_msgs.msg import DiagnosticArray
        from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
        from nav_msgs.msg import OccupancyGrid, Odometry
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.parameter_client import AsyncParameterClient
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Bool, Empty, String
        from std_srvs.srv import Trigger

        class _Node(Node):
            pass

        self._rclpy = rclpy
        self.node = _Node("bio_nav_v6_formal_single_episode")
        self.manifest = manifest
        self.episode = episode
        self.output_jsonl = output_jsonl
        self.qualification = qualification
        self.guard = EpisodeGuard(
            mission_leg_ids=tuple(item.goal_id for item in manifest.mission_legs)
        )
        self.facts = ReadinessFacts()
        self.latest_prior_epoch: int | None = None
        self.canonical_route_count = 0
        self.collision = False
        self.route_goal_results: list[dict[str, Any]] = []
        self.obstacle_state_messages: list[dict[str, Any]] = []
        self.appearance_state: dict[str, Any] | None = None
        self.dynamic_actions = DynamicActionLedger()
        self.dynamic_clients: dict[tuple[str, str], Any] = {}
        self._types = {
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
        self.isaac_parameters = AsyncParameterClient(self.node, "/isaac_navigation_sim")

        def sub(message_type, topic, callback, qos=reliable):
            return self.node.create_subscription(message_type, topic, callback, qos)

        self.subscriptions = [
            sub(Odometry, "/odom", lambda m: self._fact("estimated_odom_seen", "/odom", m), sensor),
            sub(PoseWithCovarianceStamped, "/amcl_pose", lambda m: self._fact("amcl_pose_seen", "/amcl_pose", m), reliable),
            sub(OccupancyGrid, "/map", lambda m: self._fact("map_seen", "/map", m), latched),
            sub(Empty, "/simulation/reset_event", self._reset_event),
            sub(Empty, "/simulation/localization_seeded", self._localization_seeded),
            sub(CognitiveMapConstraints, "/bio_nav/cognitive_map/constraints", lambda m: self._fact("constraints_seen", "/bio_nav/cognitive_map/constraints", m), latched),
            sub(NavigationGraph, "/bio_nav/navigation_graph", lambda m: self._fact("navigation_graph_seen", "/bio_nav/navigation_graph", m), latched),
            sub(CanonicalRoute, "/bio_nav/canonical_route", self._canonical_route, latched),
            sub(RouteProgress, "/bio_nav/route_progress", self._route_progress),
            sub(Bool, "/bio_nav/route_goal_complete", self._route_complete),
            sub(String, "/bio_nav/route_goal_result", self._route_result),
            sub(CognitivePlaceGraphCandidate, "/bio_nav/module2/cognitive_place_graph", self._capture_callback("/bio_nav/module2/cognitive_place_graph"), latched),
            sub(GoalPlanningPrior, "/bio_nav/module2/goal_planning_prior", self._goal_prior),
            sub(CognitiveGraphValidationAck, "/bio_nav/module3/cognitive_graph_validation_ack", self._capture_callback("/bio_nav/module3/cognitive_graph_validation_ack")),
            sub(CognitiveEdgeOutcome, "/bio_nav/module3/cognitive_edge_outcome", self._capture_callback("/bio_nav/module3/cognitive_edge_outcome")),
            sub(RiskLayerStatus, "/bio_nav/risk_layer/status", self._capture_callback("/bio_nav/risk_layer/status")),
            sub(RiskLayerStatus, "/bio_nav/local_risk_layer/status", self._capture_callback("/bio_nav/local_risk_layer/status")),
            sub(RiskLayerStatus, "/bio_nav/cognitive_obstacle_layer/status", self._capture_callback("/bio_nav/cognitive_obstacle_layer/status")),
            sub(RiskLayerStatus, "/bio_nav/cognitive_risk_critic/status", self._capture_callback("/bio_nav/cognitive_risk_critic/status")),
            sub(Twist, "/cmd_vel", self._capture_callback("/cmd_vel")),
            sub(Bool, "/simulation/collision", self._collision),
            sub(String, "/simulation/collision_diagnostics", self._capture_callback("/simulation/collision_diagnostics")),
            sub(DiagnosticArray, "/diagnostics", self._capture_callback("/diagnostics")),
            sub(String, "/experiment/obstacles/state", self._obstacle_state),
            sub(String, "/experiment/appearance/state", self._appearance_state, latched),
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

    def _reset_event(self, message: Any) -> None:
        self.guard.record_reset_event()
        self._capture("/simulation/reset_event", message)

    def _localization_seeded(self, message: Any) -> None:
        self.guard.record_localization_seeded()
        self._capture("/simulation/localization_seeded", message)

    def _goal_prior(self, message: Any) -> None:
        self.latest_prior_epoch = int(message.reset_epoch)
        self.guard.record_prior(self.latest_prior_epoch)
        self._capture("/bio_nav/module2/goal_planning_prior", message)

    def _canonical_route(self, message: Any) -> None:
        self.canonical_route_count += 1
        self._capture("/bio_nav/canonical_route", message)

    def _route_progress(self, message: Any) -> None:
        self.guard.record_route_progress()
        self._capture("/bio_nav/route_progress", message)

    def _route_complete(self, message: Any) -> None:
        self.guard.record_route_completion(bool(message.data))
        self._capture("/bio_nav/route_goal_complete", message)

    def _route_result(self, message: Any) -> None:
        row = self._json_message(message)
        self.route_goal_results.append(row)
        self._capture("/bio_nav/route_goal_result", message)

    @staticmethod
    def _json_message(message: Any) -> dict[str, Any]:
        try:
            value = json.loads(str(message.data))
        except (AttributeError, json.JSONDecodeError):
            return {"raw": str(getattr(message, "data", ""))}
        return value if isinstance(value, dict) else {"value": value}

    def _obstacle_state(self, message: Any) -> None:
        row = self._json_message(message)
        self.obstacle_state_messages.append(row)
        self._write("obstacle_state", state=row)

    def _appearance_state(self, message: Any) -> None:
        self.appearance_state = self._json_message(message)
        self._write("appearance_state", state=self.appearance_state)

    def _collision(self, message: Any) -> None:
        self.collision = self.collision or bool(message.data)
        if message.data:
            self.guard.stop("collision")
        self._capture("/simulation/collision", message)

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
        self.facts.reset_event_publisher_ready = (
            by_topic["/simulation/reset_event"].get_publisher_count() > 0
        )
        self.facts.localization_publisher_ready = (
            by_topic["/simulation/localization_seeded"].get_publisher_count() > 0
        )
        self.facts.bridge_prior_publisher_ready = (
            by_topic["/bio_nav/module2/goal_planning_prior"].get_publisher_count() > 0
        )
        self.facts.route_goal_subscriber_ready = (
            self.route_goal_publisher.get_subscription_count() > 0
        )

    def _set_episode_parameters(self, timeout_sec: float) -> None:
        Parameter = self._types["Parameter"]
        if not self.isaac_parameters.wait_for_services(timeout_sec=timeout_sec):
            raise V6ContractError("Isaac reset parameter services unavailable")
        params = [
            Parameter("reset_seed", value=self.episode.seed),
            Parameter("reset_pose_name", value=self.episode.reset_pose_name),
            Parameter("dynamic_case_id", value=self.episode.dynamic_case_id),
            Parameter("dynamic_variant_id", value=self.episode.variant_id),
        ]
        if self.episode.appearance_profile_id:
            params.append(Parameter("appearance_profile_id", value=self.episode.appearance_profile_id))
        future = self.isaac_parameters.set_parameters(params)
        if not self._spin_until(future.done, timeout_sec):
            raise V6ContractError("setting episode parameters timed out")
        response = future.result()
        if response is None or any(not result.successful for result in response.results):
            raise V6ContractError("Isaac rejected episode parameters")
        self._write("episode_parameters_set", seed=self.episode.seed)

    def _goal_message(self, leg: MissionLeg):
        PoseStamped = self._types["PoseStamped"]
        message = PoseStamped()
        message.header.frame_id = leg.frame_id
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.pose.position.x = leg.x
        message.pose.position.y = leg.y
        yaw = math.radians(leg.yaw_deg)
        message.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.orientation.w = math.cos(yaw / 2.0)
        return message

    def _call_dynamic_action(self, group: str, action: str, timeout_sec: float) -> bool:
        if not group:
            return True
        try:
            self.dynamic_actions.claim(group, action)
        except V6ContractError as exc:
            self.guard.stop(str(exc))
            return False
        Trigger = self._types["Trigger"]
        key = (group, action)
        client = self.dynamic_clients.get(key)
        if client is None:
            client = self.node.create_client(
                Trigger, f"/experiment/obstacles/{group}/{action}"
            )
            self.dynamic_clients[key] = client
        self._write("dynamic_action", group=group, action=action, phase="call")
        if not client.wait_for_service(timeout_sec=timeout_sec):
            reason = f"dynamic_{action}_service_unavailable:{group}"
            self.dynamic_actions.record(group, action, "service_unavailable")
            self.guard.stop(reason)
            return False
        future = client.call_async(Trigger.Request())
        if not self._spin_until(future.done, timeout_sec):
            reason = f"dynamic_{action}_timeout:{group}"
            self.dynamic_actions.record(group, action, "timeout")
            self.guard.stop(reason)
            return False
        response = future.result()
        if response is None or response.success is not True:
            detail = "no response" if response is None else str(response.message)
            reason = f"dynamic_{action}_rejected:{group}:{detail}"
            self.dynamic_actions.record(group, action, "rejected", detail)
            self.guard.stop(reason)
            return False
        detail = str(response.message)
        self.dynamic_actions.record(group, action, "accepted", detail)
        self._write(
            "dynamic_action",
            group=group,
            action=action,
            phase="response",
            success=True,
            detail=detail,
        )
        return True

    def run(self, *, readiness_timeout_sec: float, reset_timeout_sec: float, navigation_timeout_sec: float) -> dict[str, Any]:
        self._write(
            "episode_start",
            qualification=self.qualification,
            formal_qualification=NOT_QUALIFIED if self.qualification == ENGINEERING_PILOT else "FORMAL_ELIGIBLE",
            manifest=str(self.manifest.path),
            seed=self.episode.seed,
            scene_contract_frozen=self.manifest.frozen,
            runtime_values=dict(self.manifest.raw["required_runtime_values"]),
        )
        ready = self._spin_until(
            lambda: (self._refresh_endpoint_facts() is None)
            and not self.facts.missing()
            and self.latest_prior_epoch is not None,
            readiness_timeout_sec,
        )
        if not ready:
            self.guard.stop("readiness_timeout:" + ",".join(self.facts.missing()))
            return self.result()
        self.guard.arm_reset(self.facts, self.latest_prior_epoch)
        self._set_episode_parameters(reset_timeout_sec)
        if self.manifest.category == "appearance":
            expected_profile = self.episode.appearance_profile_id
            if not self._spin_until(
                lambda: self.appearance_state is not None
                and self.appearance_state.get("profile_id") == expected_profile,
                reset_timeout_sec,
            ):
                self.guard.stop(f"appearance_profile_not_observed:{expected_profile}")
                return self.result()

        Trigger = self._types["Trigger"]
        self.guard.record_reset_call()
        future = self.reset_client.call_async(Trigger.Request())
        if not self._spin_until(future.done, reset_timeout_sec):
            self.guard.record_reset_response(None)
            return self.result()
        response = future.result()
        self.guard.record_reset_response(response.success if response is not None else None)
        if self.guard.state == "STOP":
            return self.result()
        if not self._spin_until(lambda: self.guard.goal_ready or self.guard.state == "STOP", reset_timeout_sec):
            self.guard.stop("post_reset_readiness_timeout")
            return self.result()
        if not self.guard.goal_ready:
            return self.result()

        for index, leg in enumerate(self.manifest.mission_legs):
            route_baseline = self.canonical_route_count
            result_baseline = len(self.route_goal_results)
            triggered = False
            if leg.dynamic_trigger_group:
                triggered = self._call_dynamic_action(
                    leg.dynamic_trigger_group, "trigger", reset_timeout_sec
                )
                if not triggered:
                    break
            self.guard.record_goal_publication(leg.goal_id)
            self.route_goal_publisher.publish(self._goal_message(leg))
            self._write(
                "route_goal_published",
                topic="/bio_nav/route_goal",
                leg_id=leg.goal_id,
                leg_index=index,
                result_messages_before=result_baseline,
            )
            completed = self._spin_until(
                lambda: self.guard.state in {"LEG_SUCCEEDED", "SUCCEEDED", "FAILED", "STOP"},
                navigation_timeout_sec,
            )
            if not completed:
                self.guard.stop(f"route_completion_timeout:{leg.goal_id}")
            if triggered:
                self._call_dynamic_action(
                    leg.dynamic_trigger_group, "complete", reset_timeout_sec
                )
            if self.canonical_route_count <= route_baseline:
                self.guard.stop(f"canonical_route_missing:{leg.goal_id}")
            self._write(
                "mission_leg_result",
                leg_id=leg.goal_id,
                state=self.guard.state,
                route_progress_messages=self.guard.current_leg_progress_messages,
                route_result_messages=len(self.route_goal_results) - result_baseline,
            )
            if self.guard.state not in {"LEG_SUCCEEDED", "SUCCEEDED"}:
                break
        return self.result()

    def result(self) -> dict[str, Any]:
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
            "goal_publications": self.guard.goal_publications,
            "route_progress_messages": self.guard.route_progress_messages,
            "route_completion_messages": self.guard.route_completion_messages,
            "completed_leg_ids": list(self.guard.completed_leg_ids),
            "route_goal_results": list(self.route_goal_results),
            "dynamic_actions": list(self.dynamic_actions.events),
            "actor_states": self._actor_state_summary(),
            "appearance": self._appearance_summary(),
            "collision": self.collision,
        }
        self._write("episode_result", **row)
        return row

    def _actor_state_summary(self) -> dict[str, list[str]]:
        summary: dict[str, set[str]] = {}
        for snapshot in self.obstacle_state_messages:
            obstacles = snapshot.get("obstacles", [])
            if not isinstance(obstacles, list):
                continue
            for obstacle in obstacles:
                if not isinstance(obstacle, Mapping):
                    continue
                obstacle_id = str(obstacle.get("id", ""))
                state = str(obstacle.get("state", ""))
                if obstacle_id and state:
                    summary.setdefault(obstacle_id, set()).add(state)
        return {name: sorted(states) for name, states in sorted(summary.items())}

    def _appearance_summary(self) -> dict[str, Any]:
        if self.appearance_state is None:
            return {"observed": False}
        counts = self.appearance_state.get("applied_counts", {})
        return {
            "observed": True,
            "profile_id": self.appearance_state.get("profile_id"),
            "light_intensity_scale": self.appearance_state.get("overrides", {}).get(
                "light_intensity_scale"
            ),
            "material_hue_shift_deg": self.appearance_state.get("overrides", {}).get(
                "material_hue_shift_deg"
            ),
            "lights_applied": counts.get("lights") if isinstance(counts, Mapping) else None,
            "material_inputs_applied": (
                counts.get("material_color_inputs") if isinstance(counts, Mapping) else None
            ),
        }

    def destroy(self) -> None:
        self.node.destroy_node()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--episode-index", type=int, default=0)
    parser.add_argument("--pilot", action="store_true")
    parser.add_argument("--dispatch-pilot", action="store_true")
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
        manifest = load_manifest(args.manifest)
        mode = "pilot" if args.pilot else "formal"
        qualification = authorize_manifest(manifest, mode=mode)
        if args.pilot and not args.dispatch_pilot:
            print(json.dumps({
                "qualification": ENGINEERING_PILOT,
                "formal_qualification": qualification,
                "dispatch": False,
                "scene_contract_frozen": manifest.frozen,
                "missing_required_values": manifest.missing_required_values,
            }, sort_keys=True))
            return 0
        if not args.pilot and not args.allow_formal_dispatch:
            raise V6ContractError("formal dispatch requires --allow-formal-dispatch")
        if args.output_jsonl is None:
            kind = "pilot" if args.pilot else "formal"
            raise V6ContractError(f"{kind} dispatch requires --output-jsonl")
        if not 0 <= args.episode_index < len(manifest.episodes):
            raise V6ContractError("episode-index out of range")
        import rclpy
        rclpy.init(args=None)
        adapter = V6FormalNode(
            manifest,
            manifest.episodes[args.episode_index],
            Path(args.output_jsonl).expanduser().resolve(),
            qualification=ENGINEERING_PILOT if args.pilot else "FORMAL_ELIGIBLE",
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
