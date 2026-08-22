"""V6 estimated-autonomy episode dispatcher and engineering pilot adapter.

This module deliberately does not share the legacy experiment runner.  The
dispatcher owns reset and RouteCoordinator goal sequencing only.  Ground Truth
is reserved for the independent ``estimated_state_evaluator``/recorder.
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import sys
import time
from typing import Any, Mapping

import yaml

from robot_experiments.reset_receipt import (
    ResetReceiptError,
    parse_reset_receipt,
)


SCHEMA_VERSION = "bio_nav_v6_single_episode_manifest_v1"
NOT_QUALIFIED = "NOT_QUALIFIED"
ENGINEERING_PILOT = "ENGINEERING_PILOT"
GT_PREFIX = "/" + "ground_truth/"
PRE_RESET_NEGATIVE_WINDOW_S = 1.0
# Cold episode boundary: reset is only armed after the stack is provably
# idle and still.  The same 1.0 s quiet window doubles as the stillness
# observation window.
PRE_RESET_STILL_SPAN_M = 0.10
COMMAND_ZERO_TOLERANCE = 1.0e-3
# Post-reset odometry must land at the re-zeroed odom origin and stay
# bounded until the first goal (no stale drive replay, no estimator jump).
POST_RESET_ODOM_LANDING_M = 0.10
POST_RESET_ODOM_SPAN_M = 0.10
SOLE_PUBLISHER_TOPICS = ("/odom", "/cmd_vel", "/cmd_vel_sim", "/amcl_pose")
FINAL_ESTIMATED_POLICY = {
    "ekf_profile": "wheel_imu",
    "lidar_odometry_backend": "off",
    "lidar_odometry_validated": False,
    "rf2o_decision": "not_validated_off",
    "imu_calibration_profile": "isaac_v6_calibrated",
}

# Runtime subscriptions are a reviewable firewall.  Keep Ground Truth in the
# passive evaluator, never in this dispatcher.
DISPATCH_SUBSCRIPTION_TOPICS = (
    "/clock",
    "/scan",
    "/odom",
    "/amcl_pose",
    "/initialpose",
    "/tf",
    "/tf_static",
    "/map",
    "/simulation/reset_event",
    "/bio_nav/cognitive_map/constraints",
    "/bio_nav/localization/candidates",
    "/bio_nav/navigation_graph",
    "/bio_nav/canonical_route",
    "/bio_nav/route_progress",
    "/bio_nav/route_goal_complete",
    "/bio_nav/route_goal_result",
    "/bio_nav/module2/cognitive_place_graph",
    "/bio_nav/module2/planning_prior",
    "/bio_nav/module2/goal_planning_prior",
    "/bio_nav/module3/cognitive_graph_validation_ack",
    "/bio_nav/module3/cognitive_edge_outcome",
    "/bio_nav/risk_layer/status",
    "/bio_nav/local_risk_layer/status",
    "/bio_nav/cognitive_obstacle_layer/status",
    "/bio_nav/cognitive_risk_critic/status",
    "/cmd_vel",
    "/cmd_vel_sim",
    "/simulation/collision",
    "/simulation/collision_diagnostics",
    "/diagnostics",
    "/experiment/obstacles/state",
    "/experiment/appearance/state",
)

CAPTURE_SCHEMA = {
    "/bio_nav/module2/planning_prior": "PlanningPrior",
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
    "/cmd_vel_sim": "Twist",
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
    localization_seed_source: str
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
        "localization_seed_source": "b5_cognitive",
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
        localization_seed_source=str(runtime["localization_seed_source"]),
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
    """Return qualification label or fail before ROS/runtime mutation."""

    if mode not in {"formal", "pilot"}:
        raise V6ContractError("mode must be formal or pilot")
    mismatches = _estimated_policy_mismatches(manifest)
    if mode == "formal":
        if allow_engineering_policy_override:
            raise V6ContractError("estimated policy override is pilot-only")
        if mismatches:
            raise V6ContractError(
                "formal dispatch refused: final Estimated policy mismatch: "
                + ", ".join(mismatches)
            )
        if not manifest.frozen:
            raise V6ContractError("formal dispatch refused: scene_contract_frozen is false")
        if manifest.missing_required_values:
            missing = ", ".join(manifest.missing_required_values)
            raise V6ContractError(f"formal dispatch refused: missing {missing}")
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
    reset_event_publisher_ready: bool = False
    reset_subscriber_roster_ready: bool = False
    route_goal_subscriber_ready: bool = False
    candidate_publisher_ready: bool = False
    initialpose_publisher_ready: bool = False
    bridge_prior_publisher_ready: bool = False
    clock_seen: bool = False
    scan_seen: bool = False
    map_seen: bool = False
    constraints_seen: bool = False
    navigation_graph_seen: bool = False
    estimated_odom_seen: bool = False
    bridge_diagnostic_ready: bool = False
    b5_diagnostic_ready: bool = False
    b5_reset_ready: bool = False

    def missing(self) -> tuple[str, ...]:
        return tuple(name for name, value in vars(self).items() if not value)


@dataclass
class EpisodeGuard:
    """Cold-boundary exactly-once reset and RouteCoordinator state machine.

    The runner discovers the current bridge epoch as its baseline at arm
    time (any epoch, not only 0) and requires the post-reset epochs to roll
    as baseline+1 (physical reset) and baseline+2 (bootstrap initialpose).
    Reset may only be armed with no active goal/route and a quiet stack;
    one runner process drives exactly one reset and one episode.
    """

    state: str = "WAITING_READINESS"
    stop_reason: str = ""
    reset_calls: int = 0
    reset_events: int = 0
    bridge_epoch_baseline: int | None = None
    bridge_session_baseline: str = ""
    physical_epoch: int | None = None
    physical_session: str = ""
    bootstrap_epoch: int | None = None
    bootstrap_session: str = ""
    candidate_messages: int = 0
    initialpose_messages: int = 0
    amcl_messages: int = 0
    prior_messages: int = 0
    initialpose_stamp_ns: int | None = None
    post_initialpose_amcl_seen: bool = False
    startup_consensus_seen: bool = False
    b5_recovery_confirmed: bool = False
    post_reset_prior_seen: bool = False
    nav2_active: bool = False
    tf_active: bool = False
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

    def arm_reset(
        self,
        facts: ReadinessFacts,
        bridge_epoch: int | None,
        bridge_session: str,
        *,
        pre_reset_counts: Mapping[str, int],
    ) -> None:
        missing = facts.missing()
        if missing:
            raise V6ContractError(f"reset readiness missing: {', '.join(missing)}")
        if bridge_epoch is None or bridge_epoch < 0:
            raise V6ContractError("reset readiness missing bridge epoch baseline")
        if not bridge_session:
            raise V6ContractError("reset readiness missing bridge session baseline")
        required_zero = ("prior", "candidate", "initialpose", "route")
        nonzero = [name for name in required_zero if int(pre_reset_counts.get(name, -1)) != 0]
        if nonzero:
            raise V6ContractError(
                "active B5 pre-reset negative window violated: " + ",".join(nonzero)
            )
        if self.goal_publications:
            self.stop("reset_with_active_goal_forbidden")
            raise V6ContractError(self.stop_reason)
        if self.reset_calls:
            self.stop("reset_retry_forbidden")
            raise V6ContractError(self.stop_reason)
        self.bridge_epoch_baseline = bridge_epoch
        self.bridge_session_baseline = bridge_session
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
            return
        self.state = "WAITING_B5_PHYSICAL_EPOCH"

    def record_candidate(self) -> None:
        self.candidate_messages += 1

    def record_startup_consensus(self, passed: bool) -> None:
        if not passed:
            return
        if self.reset_events != 1 or self.physical_epoch is None:
            self.stop("startup_consensus_outside_physical_epoch")
            return
        self.startup_consensus_seen = True

    def record_initialpose(self, stamp_ns: int) -> None:
        self.initialpose_messages += 1
        if self.initialpose_messages > 1:
            self.stop("second_initialpose")
            return
        if self.reset_events != 1:
            self.stop("initialpose_outside_reset_epoch")
            return
        if stamp_ns <= 0:
            self.stop("initialpose_stamp_invalid")
            return
        self.initialpose_stamp_ns = int(stamp_ns)
        self.state = "WAITING_B5_CONFIRMATION"

    def record_amcl(self, stamp_ns: int) -> None:
        self.amcl_messages += 1
        if self.initialpose_stamp_ns is None:
            return
        if stamp_ns <= self.initialpose_stamp_ns:
            self.stop("amcl_not_strictly_newer_than_initialpose")
            return
        self.post_initialpose_amcl_seen = True
        self._maybe_goal_ready()

    def record_bridge(self, reset_epoch: int, recurrent_session_id: str, bootstrap_active: bool) -> None:
        if self.bridge_epoch_baseline is None:
            return
        physical_expected = self.bridge_epoch_baseline + 1
        bootstrap_expected = self.bridge_epoch_baseline + 2
        if reset_epoch == physical_expected:
            if not recurrent_session_id or recurrent_session_id == self.bridge_session_baseline:
                self.stop("physical_epoch_session_not_rolled")
                return
            self.physical_epoch = reset_epoch
            self.physical_session = recurrent_session_id
            if not bootstrap_active:
                self.stop("physical_epoch_bootstrap_not_active")
            return
        if reset_epoch == bootstrap_expected:
            if self.physical_epoch != physical_expected:
                self.stop("bootstrap_rollover_without_physical_epoch")
                return
            if self.initialpose_messages != 1:
                self.stop("bootstrap_rollover_without_initialpose")
                return
            if (
                not recurrent_session_id
                or recurrent_session_id in {self.bridge_session_baseline, self.physical_session}
            ):
                self.stop("bootstrap_session_not_new")
                return
            if bootstrap_active:
                self.stop("bootstrap_rollover_still_shadow")
                return
            self.bootstrap_epoch = reset_epoch
            self.bootstrap_session = recurrent_session_id
            self._maybe_goal_ready()
            return
        if reset_epoch > bootstrap_expected:
            self.stop(f"bridge_epoch_mismatch:{reset_epoch}!={bootstrap_expected}")

    def record_b5_diagnostic(
        self, *, state: str, recovery_result: str, seed_confirmation: str
    ) -> None:
        if seed_confirmation in {"failed", "seed_confirmation_failed"}:
            self.stop("b5_seed_confirmation_failed")
            return
        if recovery_result in {"timeout", "seed_confirmation_failed"}:
            self.stop(f"b5_recovery_{recovery_result}")
            return
        self.b5_recovery_confirmed = (
            state == "normal"
            and recovery_result == "succeeded"
            and seed_confirmation == "succeeded"
        )
        self._maybe_goal_ready()

    def record_prior(
        self,
        reset_epoch: int,
        recurrent_session_id: str,
        *,
        trusted_write: bool,
        module2_healthy: bool,
        observation_valid: bool,
        input_healthy: bool,
    ) -> None:
        self.prior_messages += 1
        if self.bootstrap_epoch is None:
            return
        if reset_epoch != self.bootstrap_epoch or recurrent_session_id != self.bootstrap_session:
            self.stop("active_prior_generation_mismatch")
            return
        if not all((trusted_write, module2_healthy, observation_valid, input_healthy)):
            self.stop("active_prior_not_trusted")
            return
        self.post_reset_prior_seen = True
        self._maybe_goal_ready()

    def record_navigation_ready(self, *, nav2_active: bool, tf_active: bool) -> None:
        self.nav2_active = bool(nav2_active)
        self.tf_active = bool(tf_active)
        self._maybe_goal_ready()

    def _maybe_goal_ready(self) -> None:
        if self.state == "STOP":
            return
        if all(
            (
                self.reset_calls == 1,
                self.reset_events == 1,
                self.physical_epoch is not None,
                self.startup_consensus_seen,
                self.initialpose_messages == 1,
                self.post_initialpose_amcl_seen,
                self.b5_recovery_confirmed,
                self.bootstrap_epoch is not None,
                self.post_reset_prior_seen,
                self.nav2_active,
                self.tf_active,
            )
        ):
            self.state = "GOAL_READY"

    @property
    def goal_ready(self) -> bool:
        return self.state == "GOAL_READY" and not self.stop_reason

    @property
    def b5_bootstrap_ready(self) -> bool:
        return bool(
            not self.stop_reason
            and self.reset_events == 1
            and self.physical_epoch is not None
            and self.startup_consensus_seen
            and self.initialpose_messages == 1
            and self.post_initialpose_amcl_seen
            and self.b5_recovery_confirmed
            and self.bootstrap_epoch is not None
            and self.post_reset_prior_seen
        )

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
            CognitiveLocalizationCandidateArray,
            GoalPlanningPrior,
            NavigationGraph,
            PlanningPrior,
            RiskLayerStatus,
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
        from std_msgs.msg import Bool, Empty, String
        from std_srvs.srv import Trigger
        from sensor_msgs.msg import LaserScan
        from tf2_msgs.msg import TFMessage

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
        self.latest_bridge_epoch: int | None = None
        self.latest_bridge_session = ""
        self.pre_reset_counts = {
            name: 0 for name in ("prior", "candidate", "initialpose", "route")
        }
        self.pre_reset_quiet_since: float | None = None
        self._cmd_window: deque[tuple[float, bool]] = deque()
        self._odom_window: deque[tuple[float, float, float]] = deque()
        self.post_reset_odom_xy: list[tuple[float, float]] = []
        self.map_odom_tf_seen = False
        self.odom_base_tf_seen = False
        self.canonical_route_count = 0
        self.collision = False
        self.route_goal_results: list[dict[str, Any]] = []
        self.obstacle_state_messages: list[dict[str, Any]] = []
        self.appearance_state: dict[str, Any] | None = None
        self.dynamic_actions = DynamicActionLedger()
        self.dynamic_clients: dict[tuple[str, str], Any] = {}
        self.reset_receipt: dict[str, Any] | None = None
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
        self.nav2_active_client = self.node.create_client(
            Trigger, "/lifecycle_manager_navigation/is_active"
        )
        self.isaac_parameters = AsyncParameterClient(self.node, "/isaac_navigation_sim")

        def sub(message_type, topic, callback, qos=reliable):
            return self.node.create_subscription(message_type, topic, callback, qos)

        self.subscriptions = [
            sub(Clock, "/clock", lambda m: self._fact("clock_seen", "/clock", m), sensor),
            sub(LaserScan, "/scan", lambda m: self._fact("scan_seen", "/scan", m), sensor),
            sub(Odometry, "/odom", self._odom, sensor),
            sub(PoseWithCovarianceStamped, "/amcl_pose", self._amcl_pose, reliable),
            sub(PoseWithCovarianceStamped, "/initialpose", self._initialpose, reliable),
            sub(TFMessage, "/tf", self._tf),
            sub(TFMessage, "/tf_static", self._tf, latched),
            sub(OccupancyGrid, "/map", lambda m: self._fact("map_seen", "/map", m), latched),
            sub(Empty, "/simulation/reset_event", self._reset_event),
            sub(CognitiveMapConstraints, "/bio_nav/cognitive_map/constraints", lambda m: self._fact("constraints_seen", "/bio_nav/cognitive_map/constraints", m), latched),
            sub(CognitiveLocalizationCandidateArray, "/bio_nav/localization/candidates", self._localization_candidates),
            sub(NavigationGraph, "/bio_nav/navigation_graph", lambda m: self._fact("navigation_graph_seen", "/bio_nav/navigation_graph", m), latched),
            sub(CanonicalRoute, "/bio_nav/canonical_route", self._canonical_route, latched),
            sub(RouteProgress, "/bio_nav/route_progress", self._route_progress),
            sub(Bool, "/bio_nav/route_goal_complete", self._route_complete),
            sub(String, "/bio_nav/route_goal_result", self._route_result),
            sub(CognitivePlaceGraphCandidate, "/bio_nav/module2/cognitive_place_graph", self._capture_callback("/bio_nav/module2/cognitive_place_graph"), latched),
            sub(PlanningPrior, "/bio_nav/module2/planning_prior", self._planning_prior),
            sub(GoalPlanningPrior, "/bio_nav/module2/goal_planning_prior", self._goal_prior),
            sub(CognitiveGraphValidationAck, "/bio_nav/module3/cognitive_graph_validation_ack", self._capture_callback("/bio_nav/module3/cognitive_graph_validation_ack")),
            sub(CognitiveEdgeOutcome, "/bio_nav/module3/cognitive_edge_outcome", self._capture_callback("/bio_nav/module3/cognitive_edge_outcome")),
            sub(RiskLayerStatus, "/bio_nav/risk_layer/status", self._capture_callback("/bio_nav/risk_layer/status")),
            sub(RiskLayerStatus, "/bio_nav/local_risk_layer/status", self._capture_callback("/bio_nav/local_risk_layer/status")),
            sub(RiskLayerStatus, "/bio_nav/cognitive_obstacle_layer/status", self._capture_callback("/bio_nav/cognitive_obstacle_layer/status")),
            sub(RiskLayerStatus, "/bio_nav/cognitive_risk_critic/status", self._capture_callback("/bio_nav/cognitive_risk_critic/status")),
            sub(Twist, "/cmd_vel", lambda m: self._track_command("/cmd_vel", m)),
            sub(Twist, "/cmd_vel_sim", lambda m: self._track_command("/cmd_vel_sim", m)),
            sub(Bool, "/simulation/collision", self._collision),
            sub(String, "/simulation/collision_diagnostics", self._capture_callback("/simulation/collision_diagnostics")),
            sub(DiagnosticArray, "/diagnostics", self._diagnostics),
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

    def _odom(self, message: Any) -> None:
        self.facts.estimated_odom_seen = True
        now = time.monotonic()
        x = float(message.pose.pose.position.x)
        y = float(message.pose.pose.position.y)
        self._odom_window.append((now, x, y))
        horizon = 4.0 * PRE_RESET_NEGATIVE_WINDOW_S
        while self._odom_window and now - self._odom_window[0][0] > horizon:
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
        horizon = 4.0 * PRE_RESET_NEGATIVE_WINDOW_S
        while self._cmd_window and now - self._cmd_window[0][0] > horizon:
            self._cmd_window.popleft()
        if nonzero and self.guard.reset_calls and not self.guard.goal_publications:
            self.guard.stop(f"post_reset_command_nonzero:{topic}")
        self._capture(topic, message)

    def _reset_event(self, message: Any) -> None:
        self.guard.record_reset_event()
        self._capture("/simulation/reset_event", message)

    @staticmethod
    def _header_stamp_ns(message: Any) -> int:
        stamp = message.header.stamp
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _amcl_pose(self, message: Any) -> None:
        if self.guard.reset_calls:
            self.guard.record_amcl(self._header_stamp_ns(message))
        self._capture("/amcl_pose", message)

    def _initialpose(self, message: Any) -> None:
        self.pre_reset_counts["initialpose"] += 1
        if self.guard.reset_calls:
            self.guard.record_initialpose(self._header_stamp_ns(message))
        self._capture("/initialpose", message)

    def _localization_candidates(self, message: Any) -> None:
        self.pre_reset_counts["candidate"] += 1
        if self.guard.reset_calls:
            self.guard.record_candidate()
        self._capture("/bio_nav/localization/candidates", message)

    def _tf(self, message: Any) -> None:
        for transform in message.transforms:
            parent = str(transform.header.frame_id).lstrip("/")
            child = str(transform.child_frame_id).lstrip("/")
            self.map_odom_tf_seen |= parent == "map" and child == "odom"
            self.odom_base_tf_seen |= parent == "odom" and child in {
                "base_link", "base_footprint"
            }

    @staticmethod
    def _diagnostic_values(status: Any) -> dict[str, str]:
        return {str(item.key): str(item.value) for item in status.values}

    def _diagnostics(self, message: Any) -> None:
        for status in message.status:
            values = self._diagnostic_values(status)
            if status.name == "bio_nav_ros_bridge":
                try:
                    epoch = int(values.get("reset_epoch", ""))
                except ValueError:
                    continue
                session = values.get("recurrent_session_id", "")
                bootstrap_active = values.get("v6_shadow_bootstrap_active") == "True"
                self.latest_bridge_epoch = epoch
                self.latest_bridge_session = session
                self.facts.bridge_diagnostic_ready = bool(session)
                self.guard.record_bridge(epoch, session, bootstrap_active)
            elif status.name == "bio_nav_localization_supervisor":
                self.facts.b5_diagnostic_ready = True
                generation = values.get("candidate_array_last_generation", "")
                event_reason = values.get("candidate_array_last_event_reason", "")
                self.facts.b5_reset_ready = (
                    generation in {
                        "not_received", "waiting_after_physical_reset"
                    }
                    or event_reason in {
                        "waiting_for_candidate_array", "waiting_after_physical_reset"
                    }
                    or self._b5_matches_bridge_baseline(generation)
                )
                try:
                    consensus_count = int(values.get("startup_consensus_count", "0"))
                except ValueError:
                    consensus_count = 0
                if self.guard.reset_calls:
                    self.guard.record_startup_consensus(consensus_count >= 2)
                    self.guard.record_b5_diagnostic(
                        state=values.get("state", ""),
                        recovery_result=values.get("recovery_result", ""),
                        seed_confirmation=values.get("seed_confirmation", ""),
                    )
        self._capture("/diagnostics", message)

    def _planning_prior(self, message: Any) -> None:
        self.pre_reset_counts["prior"] += 1
        if self.guard.reset_calls:
            self.guard.record_prior(
                int(message.reset_epoch),
                str(message.recurrent_session_id),
                trusted_write=bool(message.trusted_write),
                module2_healthy=bool(message.module2_healthy),
                observation_valid=bool(message.observation_valid),
                input_healthy=bool(message.input_healthy),
            )
        self._capture("/bio_nav/module2/planning_prior", message)

    def _goal_prior(self, message: Any) -> None:
        self._capture("/bio_nav/module2/goal_planning_prior", message)

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
        self._capture("/bio_nav/route_goal_complete", message)

    def _route_result(self, message: Any) -> None:
        self._track_route_signal("route_goal_result")
        row = self._json_message(message)
        self.route_goal_results.append(row)
        self._capture("/bio_nav/route_goal_result", message)

    def _track_route_signal(self, kind: str) -> None:
        """Route traffic is only legal after this runner's first goal."""
        if not self.guard.reset_calls:
            self.pre_reset_counts["route"] += 1
        elif not self.guard.goal_publications:
            self.guard.stop(f"stale_{kind}_after_reset")

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
        # Runner + Bridge + B5 supervisor must all be listening before the
        # single reset is allowed to mutate the episode.
        self.facts.reset_subscriber_roster_ready = (
            self.node.count_subscribers("/simulation/reset_event") >= 3
        )
        self.facts.candidate_publisher_ready = (
            by_topic["/bio_nav/localization/candidates"].get_publisher_count() > 0
        )
        self.facts.initialpose_publisher_ready = (
            by_topic["/initialpose"].get_publisher_count() > 0
        )
        self.facts.bridge_prior_publisher_ready = (
            by_topic["/bio_nav/module2/planning_prior"].get_publisher_count() > 0
        )
        self.facts.route_goal_subscriber_ready = (
            self.route_goal_publisher.get_subscription_count() > 0
        )

    def _b5_matches_bridge_baseline(self, generation: str) -> bool:
        """Warm-stack readiness: B5 is seeded in the discovered baseline."""
        if self.latest_bridge_epoch is None or not self.latest_bridge_session:
            return False
        fields = {}
        for item in generation.split(","):
            key, _, value = item.partition("=")
            fields[key] = value
        try:
            epoch = int(fields.get("epoch", ""))
        except ValueError:
            return False
        return bool(
            epoch == self.latest_bridge_epoch
            and fields.get("session", "") == self.latest_bridge_session
        )

    def _pre_reset_violations(self) -> tuple[str, ...]:
        return tuple(
            name
            for name in ("prior", "candidate", "initialpose", "route")
            if self.pre_reset_counts.get(name, 0)
        )

    def _publisher_ownership_violations(self) -> tuple[str, ...]:
        return tuple(
            f"{topic}={self.node.count_publishers(topic)}"
            for topic in SOLE_PUBLISHER_TOPICS
            if self.node.count_publishers(topic) != 1
        )

    def _pre_reset_still(self) -> bool:
        """Cold boundary: zero commands and a bounded odom span in-window."""
        now = time.monotonic()
        horizon = now - PRE_RESET_NEGATIVE_WINDOW_S
        if any(nonzero for stamp, nonzero in self._cmd_window if stamp >= horizon):
            return False
        window = [(x, y) for stamp, x, y in self._odom_window if stamp >= horizon]
        if not window:
            return False
        xs = [point[0] for point in window]
        ys = [point[1] for point in window]
        return bool(
            max(xs) - min(xs) <= PRE_RESET_STILL_SPAN_M
            and max(ys) - min(ys) <= PRE_RESET_STILL_SPAN_M
        )

    def _readiness_blockers(self) -> str:
        blockers: list[str] = []
        missing = self.facts.missing()
        if missing:
            blockers.append("facts:" + ",".join(missing))
        if self.latest_bridge_epoch is None or not self.latest_bridge_session:
            blockers.append("bridge_epoch_baseline")
        violations = self._pre_reset_violations()
        if violations:
            blockers.append("pre_reset_negative_window:" + ",".join(violations))
        ownership = self._publisher_ownership_violations()
        if ownership:
            blockers.append("publisher_ownership:" + ",".join(ownership))
        if not self._pre_reset_still():
            blockers.append("pre_reset_not_still")
        return ";".join(blockers)

    def _assert_ground_truth_firewall(self) -> None:
        offending = [
            subscription.topic_name
            for subscription in self.subscriptions
            if subscription.topic_name.startswith(GT_PREFIX)
        ]
        if offending:
            raise V6ContractError(
                "dispatcher Ground Truth firewall violated: " + ",".join(offending)
            )

    def _check_post_reset_odom(self) -> None:
        """Odometry must land at the re-zeroed origin and stay bounded."""
        samples = self.post_reset_odom_xy[1:]  # skip one straddling sample
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

    def _nav2_is_active(self, timeout_sec: float) -> bool:
        if not self.nav2_active_client.wait_for_service(timeout_sec=timeout_sec):
            return False
        Trigger = self._types["Trigger"]
        future = self.nav2_active_client.call_async(Trigger.Request())
        if not self._spin_until(future.done, timeout_sec):
            return False
        response = future.result()
        return bool(response is not None and response.success is True)

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
        self._assert_ground_truth_firewall()
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
            self._pre_reset_ready,
            readiness_timeout_sec,
        )
        if not ready:
            self.guard.stop("readiness_timeout:" + (self._readiness_blockers() or "unknown"))
            return self.result()
        self.guard.arm_reset(
            self.facts,
            self.latest_bridge_epoch,
            self.latest_bridge_session,
            pre_reset_counts=self.pre_reset_counts,
        )
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
        try:
            self.reset_receipt = parse_reset_receipt(
                response.message,
                requested_seed=self.episode.seed,
                requested_case_id=self.episode.dynamic_case_id,
                requested_variant_id=self.episode.variant_id,
                requested_pose=self.episode.reset_pose_name,
            )
        except ResetReceiptError as exc:
            self.guard.stop(f"reset_receipt_mismatch:{exc}")
            self._write("reset_receipt_rejected", detail=str(exc))
            return self.result()
        self._write("reset_receipt", **self.reset_receipt)
        if not self._spin_until(
            lambda: self.guard.b5_bootstrap_ready or self.guard.state == "STOP",
            reset_timeout_sec,
        ):
            self.guard.stop("post_reset_readiness_timeout")
            return self.result()
        if self.guard.state == "STOP":
            return self.result()
        self._check_post_reset_odom()
        if self.guard.state == "STOP":
            return self.result()
        self.guard.record_navigation_ready(
            nav2_active=self._nav2_is_active(reset_timeout_sec),
            tf_active=self.map_odom_tf_seen and self.odom_base_tf_seen,
        )
        if not self.guard.goal_ready:
            self.guard.stop("nav2_or_tf_not_ready")
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
            "reset_receipt": dict(getattr(self, "reset_receipt", None) or {}),
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
    parser.add_argument(
        "--allow-engineering-estimated-policy-override",
        action="store_true",
        help="pilot-only; the result remains NOT_QUALIFIED",
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
        mode = "pilot" if args.pilot else "formal"
        qualification = authorize_manifest(
            manifest,
            mode=mode,
            allow_engineering_policy_override=(
                args.allow_engineering_estimated_policy_override
            ),
        )
        if args.pilot and not args.dispatch_pilot:
            print(json.dumps({
                "qualification": ENGINEERING_PILOT,
                "formal_qualification": qualification,
                "dispatch": False,
                "estimated_policy": dict(manifest.estimated_policy),
                "engineering_estimated_policy_override": (
                    args.allow_engineering_estimated_policy_override
                ),
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
