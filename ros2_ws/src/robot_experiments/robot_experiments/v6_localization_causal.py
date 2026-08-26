"""Minimal single-round Phase D/E localization live adapter.

The adapter reuses :class:`V6FormalNode` for cold reset, readiness, route
dispatch, collision handling, and terminal-zero settling.  It adds only the
four localization arms required by the current plan:

* D/S0 -- one frozen broad runner ``/initialpose`` with supervisor shadow;
* D/S1 -- no runner seed, supervisor ``startup`` owns the first prior write;
* E/R0 -- AMCL ``/global_localization`` recovery;
* E/R1 -- supervisor ``active`` recovery after one explicit manual request.

Ground Truth is never subscribed here.  The independent evaluator owns the
passive Ground-Truth stream.
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass, replace
import json
import math
from pathlib import Path
import sys
import time
from typing import Any, Mapping, Protocol, Sequence

import yaml

from .reset_receipt import ResetReceiptError, parse_reset_receipt
from .v6_formal import (
    NOT_QUALIFIED,
    Episode,
    Manifest,
    MissionLeg,
    V6ContractError,
    V6FormalNode,
    append_evidence_jsonl,
    load_manifest as load_phase_b_manifest,
)


CONFIG_SCHEMA = "bio_nav_v6_phase_de_localization_live_v1"
EVENT_SCHEMA = "bio_nav_v6_phase_de_localization_event_v1"
QUALIFICATION = "ENGINEERING_CAUSAL"
ARMS = ("S0", "S1", "R0", "R1")
PHASE_BY_ARM = {"S0": "D", "S1": "D", "R0": "E", "R1": "E"}
SUPERVISOR_MODE_BY_ARM = {
    "S0": "shadow",
    "S1": "startup",
    "R0": "shadow",
    "R1": "active",
}
RECOVERY_BY_ARM = {
    "S0": "none",
    "S1": "none",
    "R0": "global_localization",
    "R1": "supervisor_manual_rescue",
}
RUN4_CANDIDATE_STATUS = "READ_ONLY_CAUSAL_CANDIDATE_STARTUP_ONLY"
RUN4_RECOVERY_QUALIFICATION = "NOT_ACTIVE_RECOVERY_QUALIFIED"
RUN4_ALLOWED_SUPERVISOR_MODES = ("shadow", "startup")
PHASE_D_STARTUP_INITIALPOSE = {
    "S0": {
        "source": "runner",
        "seed_kind": "broad_initialpose",
        "expected_total_count": 1,
        "expected_supervisor_count": 0,
    },
    "S1": {
        "source": "supervisor",
        "seed_kind": "cognitive_prior",
        "expected_total_count": 1,
        "expected_supervisor_count": 1,
    },
}
ALLOWED_EVENTS = frozenset(
    {
        "episode_start",
        "initialpose",
        "fault_injected",
        "pause_requested",
        "pause_confirmed",
        "prior_write",
        "localization_ready",
        "localization_recovered",
        "goal_dispatched",
        "goal_result",
        "supervisor_diagnostic",
        "estimated_pose",
        "odom_pose",
        "cmd_vel_sim",
        "collision",
        "module1_diagnostic",
        "episode_end",
    }
)


class LocalizationConfigError(V6ContractError):
    """The small Phase D/E config is invalid."""


def _mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise LocalizationConfigError(f"{name} must be a mapping")
    return value


def _finite(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise LocalizationConfigError(f"{name} must be finite numeric")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise LocalizationConfigError(f"{name} must be finite numeric") from exc
    if not math.isfinite(result):
        raise LocalizationConfigError(f"{name} must be finite numeric")
    return result


@dataclass(frozen=True)
class SeedPose:
    x: float
    y: float
    yaw_deg: float
    xy_variance_m2: float
    yaw_variance_rad2: float


@dataclass(frozen=True)
class LocalizationConfig:
    path: Path
    phase_b_manifest: Path
    phase_d_run4_candidate: Mapping[str, Any]
    seeds: Mapping[str, int]
    broad_seed: SeedPose
    wrong_region_seed: SeedPose
    route_ids: tuple[str, ...]
    fault_id: str
    fault_leg_id: str
    fault_min_arc_length_m: float
    stationary_hold_s: float
    recovery_timeout_s: float
    recovered_xy_variance_m2: float
    recovered_yaw_variance_rad2: float


def _seed_pose(raw: Any, name: str) -> SeedPose:
    row = _mapping(raw, name)
    expected = {
        "x",
        "y",
        "yaw_deg",
        "xy_variance_m2",
        "yaw_variance_rad2",
    }
    if set(row) != expected:
        raise LocalizationConfigError(f"{name} keys must be {sorted(expected)}")
    result = SeedPose(
        x=_finite(row["x"], f"{name}.x"),
        y=_finite(row["y"], f"{name}.y"),
        yaw_deg=_finite(row["yaw_deg"], f"{name}.yaw_deg"),
        xy_variance_m2=_finite(
            row["xy_variance_m2"], f"{name}.xy_variance_m2"
        ),
        yaw_variance_rad2=_finite(
            row["yaw_variance_rad2"], f"{name}.yaw_variance_rad2"
        ),
    )
    if result.xy_variance_m2 <= 0.0 or result.yaw_variance_rad2 <= 0.0:
        raise LocalizationConfigError(f"{name} variances must be positive")
    return result


def load_config(path: str | Path) -> LocalizationConfig:
    config_path = Path(path).expanduser().resolve()
    raw_value = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    raw = _mapping(raw_value, "config")
    expected_keys = {
        "schema_version",
        "phase_b_manifest",
        "held_constants",
        "phase_d_run4_candidate",
        "arms",
        "seeds",
        "startup",
        "route",
        "fault",
        "recovery",
    }
    if set(raw) != expected_keys:
        raise LocalizationConfigError(
            f"config keys must be {sorted(expected_keys)}"
        )
    if raw.get("schema_version") != CONFIG_SCHEMA:
        raise LocalizationConfigError(f"schema_version must be {CONFIG_SCHEMA}")

    base_value = str(raw.get("phase_b_manifest", "")).strip()
    if not base_value:
        raise LocalizationConfigError("phase_b_manifest must be non-empty")
    base_path = Path(base_value).expanduser()
    if not base_path.is_absolute():
        base_path = config_path.parent / base_path
    base_path = base_path.resolve()

    held = _mapping(raw.get("held_constants"), "held_constants")
    required_held = {
        "module2_navigation_write_enabled": False,
        "cognitive_place_graph_enabled": False,
        "low_obstacles_enabled": False,
        "dynamic_actors_enabled": False,
        "ground_truth_policy": "passive_evaluator_only",
        "initial_pose_source": "runner_or_supervisor_only",
    }
    if dict(held) != required_held:
        raise LocalizationConfigError(
            "held_constants must keep Module2/CPG/obstacles off and GT evaluator-only"
        )

    candidate = _mapping(
        raw.get("phase_d_run4_candidate"), "phase_d_run4_candidate"
    )
    candidate_keys = {
        "integration_manifest",
        "status",
        "recovery_qualification",
        "allowed_supervisor_modes",
        "model_id",
        "checkpoint",
        "checkpoint_sha256",
        "posterior_pregate_config",
        "posterior_pregate_config_sha256",
        "startup_initialpose",
    }
    if set(candidate) != candidate_keys:
        raise LocalizationConfigError(
            f"phase_d_run4_candidate keys must be {sorted(candidate_keys)}"
        )
    if candidate.get("status") != RUN4_CANDIDATE_STATUS:
        raise LocalizationConfigError(
            "Phase D Run4 candidate must have startup-only status"
        )
    if candidate.get("recovery_qualification") != RUN4_RECOVERY_QUALIFICATION:
        raise LocalizationConfigError(
            "Phase D Run4 candidate must remain non-recovery"
        )
    if tuple(candidate.get("allowed_supervisor_modes", ())) != (
        RUN4_ALLOWED_SUPERVISOR_MODES
    ):
        raise LocalizationConfigError(
            "Phase D Run4 candidate must allow only shadow and startup"
        )
    for name in (
        "integration_manifest",
        "model_id",
        "checkpoint",
        "checkpoint_sha256",
        "posterior_pregate_config",
        "posterior_pregate_config_sha256",
    ):
        if not str(candidate.get(name, "")).strip():
            raise LocalizationConfigError(
                f"phase_d_run4_candidate.{name} must be non-empty"
            )
    startup_initialpose = _mapping(
        candidate.get("startup_initialpose"),
        "phase_d_run4_candidate.startup_initialpose",
    )
    if dict(startup_initialpose) != PHASE_D_STARTUP_INITIALPOSE:
        raise LocalizationConfigError(
            "Phase D startup initialpose source/count contract changed"
        )

    arms = _mapping(raw.get("arms"), "arms")
    if set(arms) != set(ARMS):
        raise LocalizationConfigError(f"arms must be exactly {list(ARMS)}")
    for arm in ARMS:
        row = _mapping(arms[arm], f"arms.{arm}")
        expected = {
            "phase": PHASE_BY_ARM[arm],
            "supervisor_mode": SUPERVISOR_MODE_BY_ARM[arm],
            "recovery": RECOVERY_BY_ARM[arm],
        }
        if dict(row) != expected:
            raise LocalizationConfigError(f"arms.{arm} violates the frozen arm")

    seed_rows = _mapping(raw.get("seeds"), "seeds")
    if set(seed_rows) != set(ARMS):
        raise LocalizationConfigError(f"seeds must define exactly {list(ARMS)}")
    seeds: dict[str, int] = {}
    for arm in ARMS:
        value = seed_rows[arm]
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise LocalizationConfigError(f"seeds.{arm} must be non-negative int")
        seeds[arm] = value
    if seeds["S0"] != seeds["S1"] or seeds["R0"] != seeds["R1"]:
        raise LocalizationConfigError("paired D and E arms must use the same seed")

    startup = _mapping(raw.get("startup"), "startup")
    if set(startup) != {"broad_initialpose"}:
        raise LocalizationConfigError("startup must define broad_initialpose")
    broad_seed = _seed_pose(startup["broad_initialpose"], "startup.broad_initialpose")

    route = _mapping(raw.get("route"), "route")
    if set(route) != {"goal_ids"} or route.get("goal_ids") != [
        "G2",
        "G3",
        "G4",
        "G5",
        "G1",
    ]:
        raise LocalizationConfigError("route.goal_ids must be G2,G3,G4,G5,G1")

    fault = _mapping(raw.get("fault"), "fault")
    if set(fault) != {
        "id",
        "leg_id",
        "min_arc_length_m",
        "stationary_hold_s",
        "wrong_region_seed",
    }:
        raise LocalizationConfigError("fault keys changed")
    if fault.get("id") != "F2" or fault.get("leg_id") != "G3":
        raise LocalizationConfigError("the only supported fault is F2 on G2->G3")
    min_arc = _finite(fault["min_arc_length_m"], "fault.min_arc_length_m")
    if min_arc < 1.0:
        raise LocalizationConfigError("fault.min_arc_length_m must be >= 1.0")
    stationary_hold = _finite(
        fault["stationary_hold_s"], "fault.stationary_hold_s"
    )
    if stationary_hold <= 0.0:
        raise LocalizationConfigError("fault.stationary_hold_s must be positive")
    wrong_seed = _seed_pose(fault["wrong_region_seed"], "fault.wrong_region_seed")
    if (wrong_seed.x, wrong_seed.y, wrong_seed.yaw_deg) != (-2.20, -2.95, -42.0):
        raise LocalizationConfigError("F2 wrong-region seed must be G5 (-2.20,-2.95,-42deg)")

    recovery = _mapping(raw.get("recovery"), "recovery")
    if set(recovery) != {
        "timeout_s",
        "max_xy_variance_m2",
        "max_yaw_variance_rad2",
    }:
        raise LocalizationConfigError("recovery keys changed")
    recovery_timeout = _finite(recovery["timeout_s"], "recovery.timeout_s")
    max_xy = _finite(
        recovery["max_xy_variance_m2"], "recovery.max_xy_variance_m2"
    )
    max_yaw = _finite(
        recovery["max_yaw_variance_rad2"],
        "recovery.max_yaw_variance_rad2",
    )
    if min(recovery_timeout, max_xy, max_yaw) <= 0.0:
        raise LocalizationConfigError("recovery limits must be positive")

    return LocalizationConfig(
        path=config_path,
        phase_b_manifest=base_path,
        phase_d_run4_candidate=dict(candidate),
        seeds=seeds,
        broad_seed=broad_seed,
        wrong_region_seed=wrong_seed,
        route_ids=("G2", "G3", "G4", "G5", "G1"),
        fault_id="F2",
        fault_leg_id="G3",
        fault_min_arc_length_m=min_arc,
        stationary_hold_s=stationary_hold,
        recovery_timeout_s=recovery_timeout,
        recovered_xy_variance_m2=max_xy,
        recovered_yaw_variance_rad2=max_yaw,
    )


def route_actions(config: LocalizationConfig, arm: str) -> tuple[dict[str, Any], ...]:
    """Return the one-round action list; no 60-run matrix or R2 arm exists."""

    if arm not in ARMS:
        raise LocalizationConfigError(f"arm must be one of {list(ARMS)}")
    if PHASE_BY_ARM[arm] == "D":
        return tuple({"action": "goal", "leg_id": leg} for leg in config.route_ids)
    return (
        {"action": "goal", "leg_id": "G2"},
        {
            "action": "fault_leg",
            "leg_id": "G3",
            "fault_id": config.fault_id,
            "min_arc_length_m": config.fault_min_arc_length_m,
        },
        {"action": "recover", "method": RECOVERY_BY_ARM[arm]},
        {"action": "goal", "leg_id": "G3"},
        {"action": "goal", "leg_id": "G4"},
        {"action": "goal", "leg_id": "G5"},
        {"action": "goal", "leg_id": "G1"},
    )


def build_plan(config: LocalizationConfig) -> dict[str, Any]:
    rows = []
    for arm in ARMS:
        startup = "none" if arm == "S1" else "broad_initialpose"
        rows.append(
            {
                "run_id": f"v6-phase-{PHASE_BY_ARM[arm].lower()}-{arm.lower()}-{config.seeds[arm]}",
                "phase": PHASE_BY_ARM[arm],
                "arm": arm,
                "seed": config.seeds[arm],
                "startup": startup,
                "supervisor_mode": SUPERVISOR_MODE_BY_ARM[arm],
                "recovery": RECOVERY_BY_ARM[arm],
                "run4_candidate_enabled": PHASE_BY_ARM[arm] == "D",
                "expected_startup_initialpose": (
                    PHASE_D_STARTUP_INITIALPOSE[arm]
                    if PHASE_BY_ARM[arm] == "D"
                    else None
                ),
                "actions": list(route_actions(config, arm)),
            }
        )
    return {
        "schema_version": CONFIG_SCHEMA,
        "event_schema": EVENT_SCHEMA,
        "qualification": QUALIFICATION,
        "formal_qualification": NOT_QUALIFIED,
        "ground_truth_policy": "passive_evaluator_only",
        "phase_d_run4_candidate": dict(config.phase_d_run4_candidate),
        "phase_e_run4_candidate_enabled": False,
        "single_round_arm_count": 4,
        "runs": rows,
    }


class ActionAdapter(Protocol):
    def perform_action(self, action: Mapping[str, Any]) -> None: ...


def execute_route_actions(adapter: ActionAdapter, actions: Sequence[Mapping[str, Any]]) -> None:
    """Small seam used by focused fake-adapter tests and the ROS adapter."""

    for action in actions:
        adapter.perform_action(action)


def _yaw_deg(quaternion: Any) -> float:
    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    w = float(quaternion.w)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny, cosy))


def _contains_ground_truth(value: Any, *, key: str = "") -> bool:
    lowered = key.lower()
    if "ground_truth" in lowered or lowered in {"gt_pose", "gt_region", "gt_region_id"}:
        return True
    if isinstance(value, Mapping):
        return any(
            _contains_ground_truth(item, key=str(name))
            for name, item in value.items()
        )
    if isinstance(value, (list, tuple)):
        return any(_contains_ground_truth(item) for item in value)
    return isinstance(value, str) and (
        value.startswith("/ground_truth/") or value == "ground_truth_pose"
    )


class LocalizationCausalNode(V6FormalNode):
    """One-run ROS adapter layered on the proven Phase B episode node."""

    def __init__(
        self,
        manifest: Manifest,
        episode: Episode,
        output_jsonl: Path,
        *,
        config: LocalizationConfig,
        arm: str,
        run_id: str,
    ) -> None:
        if arm not in ARMS:
            raise LocalizationConfigError(f"arm must be one of {list(ARMS)}")
        self.config = config
        self.arm = arm
        self.phase = PHASE_BY_ARM[arm]
        self.run_id = run_id
        self._initialpose_source_queue: deque[tuple[int, str, str]] = deque()
        self._initialpose_count = 0
        self._manual_rescue_count = 0
        self._prior_write_count = 0
        self._amcl_count = 0
        self._last_amcl_covariance: tuple[float, float, float] | None = None
        self._last_supervisor: dict[str, str] = {}
        self._last_cmd_zero: bool | None = None
        self._cmd_vel_sim_zero_since: float | None = None
        self._fault_preview_active = False
        self._fault_arc_length_m = 0.0
        self._fault_cancel_future = None
        self._event_stream_started = False
        super().__init__(
            manifest,
            episode,
            output_jsonl,
            qualification=QUALIFICATION,
        )

        from bio_nav_interfaces.msg import PlanningPrior
        from diagnostic_msgs.msg import DiagnosticArray
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Empty
        from std_srvs.srv import Empty as EmptyService

        reliable = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self._types.update(
            {
                "Empty": Empty,
                "EmptyService": EmptyService,
                "PoseWithCovarianceStamped": PoseWithCovarianceStamped,
            }
        )
        self.initialpose_publisher = self.node.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", reliable
        )
        self.manual_rescue_publisher = self.node.create_publisher(
            Empty, "/bio_nav/localization/request_manual_rescue", reliable
        )
        self.global_localization_client = self.node.create_client(
            EmptyService, "/global_localization"
        )
        self.nomotion_update_client = self.node.create_client(
            EmptyService, "/request_nomotion_update"
        )
        self.subscriptions.extend(
            [
                self.node.create_subscription(
                    DiagnosticArray, "/diagnostics", self._diagnostics, reliable
                ),
                self.node.create_subscription(
                    PlanningPrior,
                    "/bio_nav/module2/planning_prior",
                    self._planning_prior,
                    reliable,
                ),
            ]
        )

    def _stamp_s(self) -> float:
        return float(self.node.get_clock().now().nanoseconds) / 1.0e9

    def _event(self, event: str, **payload: Any) -> None:
        if event not in ALLOWED_EVENTS:
            raise V6ContractError(f"unsupported localization event: {event}")
        if event == "episode_start":
            if self._event_stream_started:
                raise V6ContractError("duplicate episode_start")
            self._event_stream_started = True
        elif not self._event_stream_started:
            # Readiness callbacks may carry the previous simulation epoch.
            # Begin runtime evidence only after the accepted physical reset so
            # timestamps remain aligned with the passive post-reset GT stream.
            return
        if _contains_ground_truth(payload):
            raise V6ContractError("runtime localization event contains Ground Truth")
        append_evidence_jsonl(
            self.output_jsonl,
            event,
            schema=EVENT_SCHEMA,
            stamp_s=self._stamp_s(),
            run_id=self.run_id,
            phase=self.phase,
            arm=self.arm,
            seed=self.episode.seed,
            **payload,
        )

    def _emit_episode_start(self) -> None:
        self._event(
            "episode_start",
            qualification=QUALIFICATION,
            formal_qualification=NOT_QUALIFIED,
            config=str(self.config.path),
            phase_b_manifest=str(self.manifest.path),
            fault=(self.config.fault_id if self.phase == "E" else "none"),
            supervisor_mode=SUPERVISOR_MODE_BY_ARM[self.arm],
            recovery=RECOVERY_BY_ARM[self.arm],
            passive_evaluator_only=True,
            runtime=dict(self.manifest.runtime),
        )

    def _write(self, event: str, **payload: Any) -> None:
        """Translate the small subset of base-node events used by this run."""

        if event == "route_goal_published":
            self._event(
                "goal_dispatched",
                leg_id=str(payload.get("leg_id", "")),
                leg_index=int(payload.get("leg_index", -1)),
            )
        elif event == "mission_leg_result":
            self._event(
                "goal_result",
                leg_id=str(payload.get("leg_id", "")),
                state=str(payload.get("state", "")),
                success=str(payload.get("state", "")) in {"LEG_SUCCEEDED", "SUCCEEDED"},
            )
        elif event == "terminal_navigation_cancel_requested":
            self._event(
                "pause_requested",
                reason=str(payload.get("reason", "terminal")),
            )
        # Other base events remain in rosbag/logs.  Runtime JSONL intentionally
        # stays on the frozen small schema.

    def _odom(self, message: Any) -> None:
        super()._odom(message)
        pose = message.pose.pose
        self._event(
            "odom_pose",
            x=float(pose.position.x),
            y=float(pose.position.y),
            yaw_deg=_yaw_deg(pose.orientation),
        )

    def _amcl_pose(self, message: Any) -> None:
        super()._amcl_pose(message)
        self._amcl_count += 1
        covariance = tuple(float(value) for value in message.pose.covariance)
        self._last_amcl_covariance = (
            covariance[0], covariance[7], covariance[35]
        )
        pose = message.pose.pose
        self._event(
            "estimated_pose",
            x=float(pose.position.x),
            y=float(pose.position.y),
            yaw_deg=_yaw_deg(pose.orientation),
            covariance_xy_yaw=[covariance[0], covariance[7], covariance[35]],
        )

    def _initialpose(self, message: Any) -> None:
        super()._initialpose(message)
        self._initialpose_count += 1
        stamp_ns = self._header_stamp_ns(message)
        matching = next(
            (
                item
                for item in self._initialpose_source_queue
                if item[0] == stamp_ns
            ),
            None,
        )
        if matching is not None:
            self._initialpose_source_queue.remove(matching)
            _, source, seed_kind = matching
        elif SUPERVISOR_MODE_BY_ARM[self.arm] in {"startup", "active"}:
            source, seed_kind = "supervisor", "cognitive_prior"
        else:
            source, seed_kind = "unknown", "unknown"
        pose = message.pose.pose
        self._event(
            "initialpose",
            source=source,
            seed_kind=seed_kind,
            count=self._initialpose_count,
            x=float(pose.position.x),
            y=float(pose.position.y),
            yaw_deg=_yaw_deg(pose.orientation),
        )
        if source == "supervisor":
            self._prior_write_count += 1
            self._event(
                "prior_write", source="supervisor", count=self._prior_write_count
            )

    def _track_command(self, topic: str, message: Any) -> None:
        super()._track_command(topic, message)
        if topic != "/cmd_vel_sim":
            return
        values = (
            float(message.linear.x),
            float(message.linear.y),
            float(message.angular.z),
        )
        zero = all(abs(value) <= 1.0e-3 for value in values)
        now = time.monotonic()
        if zero:
            if self._cmd_vel_sim_zero_since is None:
                self._cmd_vel_sim_zero_since = now
        else:
            self._cmd_vel_sim_zero_since = None
        if zero != self._last_cmd_zero:
            self._last_cmd_zero = zero
            self._event(
                "cmd_vel_sim",
                zero=zero,
                linear_x=values[0],
                linear_y=values[1],
                angular_z=values[2],
            )

    def _collision(self, message: Any) -> None:
        previous = self.collision
        super()._collision(message)
        if bool(message.data) or bool(message.data) != previous:
            self._event(
                "collision",
                collision=bool(message.data),
                count=1 if self.collision else 0,
            )

    def _route_progress(self, message: Any) -> None:
        if self._fault_preview_active:
            self._fault_arc_length_m = max(
                self._fault_arc_length_m, float(message.arc_length_m)
            )
            return
        super()._route_progress(message)

    def _route_complete(self, message: Any) -> None:
        if self._fault_preview_active:
            if not bool(message.data):
                self._navigation_terminal_observed = True
            return
        super()._route_complete(message)

    def _route_result(self, message: Any) -> None:
        if self._fault_preview_active:
            return
        super()._route_result(message)

    @staticmethod
    def _diagnostic_values(status: Any) -> dict[str, str]:
        return {str(item.key): str(item.value) for item in status.values}

    def _diagnostics(self, message: Any) -> None:
        for status in message.status:
            values = self._diagnostic_values(status)
            if str(status.name) == "bio_nav_localization_supervisor":
                self._last_supervisor = values
                try:
                    attempts = int(values.get("reset_attempts", "0") or 0)
                except ValueError:
                    attempts = -1
                self._event(
                    "supervisor_diagnostic",
                    mode=values.get("mode", ""),
                    state=values.get("state", ""),
                    reason=values.get("reason", str(status.message)),
                    result=values.get("recovery_result", ""),
                    reset_attempts=attempts,
                    values=values,
                )
            elif str(status.name) == "bio_nav_ros_bridge":
                self._event(
                    "module1_diagnostic",
                    name=str(status.name),
                    state=values.get("state", ""),
                    reason=str(status.message),
                    values=values,
                )

    def _planning_prior(self, message: Any) -> None:
        values = {
            "region_id": int(message.dominant_mode_root_state_id),
            "entropy": float(message.place_entropy_normalized),
            "reliability": float(message.visual_reliability),
            "ood_probability": float(message.visual_ood_probability),
            "dominant_mass": float(message.dominant_mode_mass),
            "dominant_covariance_m2": [
                float(value) for value in message.dominant_mode_covariance_m2
            ],
        }
        self._event("module1_diagnostic", name="planning_prior", **values, values=values)

    def _publish_seed(self, pose: SeedPose, seed_kind: str) -> None:
        Message = self._types["PoseWithCovarianceStamped"]
        message = Message()
        message.header.frame_id = "map"
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.pose.pose.position.x = pose.x
        message.pose.pose.position.y = pose.y
        yaw = math.radians(pose.yaw_deg)
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)
        message.pose.covariance[0] = pose.xy_variance_m2
        message.pose.covariance[7] = pose.xy_variance_m2
        message.pose.covariance[35] = pose.yaw_variance_rad2
        stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        self._initialpose_source_queue.append((stamp_ns, "runner", seed_kind))
        self.initialpose_publisher.publish(message)

    def _call_empty_service(self, client: Any, name: str, timeout_s: float) -> bool:
        if not client.wait_for_service(timeout_sec=timeout_s):
            self.guard.stop(f"service_unavailable:{name}")
            return False
        Request = self._types["EmptyService"].Request
        future = client.call_async(Request())
        if not self._spin_until(future.done, timeout_s):
            self.guard.stop(f"service_timeout:{name}")
            return False
        if future.result() is None:
            self.guard.stop(f"service_failed:{name}")
            return False
        return True

    def _amcl_recovered(self, baseline_count: int) -> bool:
        covariance = self._last_amcl_covariance
        return bool(
            self._amcl_count >= baseline_count + 3
            and covariance is not None
            and max(covariance[0], covariance[1])
            <= self.config.recovered_xy_variance_m2
            and covariance[2] <= self.config.recovered_yaw_variance_rad2
        )

    def _supervisor_recovered(self) -> bool:
        return self._last_supervisor.get("recovery_result", "").lower() in {
            "succeeded", "success", "seed_confirmed"
        }

    def _cancel_fault_preview(self, reason: str) -> tuple[bool, bool]:
        """Cancel the unregistered preview goal and await cancel plus zero."""

        self._event(
            "pause_requested",
            reason=reason,
            fault_id=self.config.fault_id,
            leg_id=self.config.fault_leg_id,
            arc_length_m=self._fault_arc_length_m,
        )
        self._start_terminal_settle(cancel_navigation=False, reason=reason)
        if not self.navigate_cancel_client.wait_for_service(timeout_sec=2.0):
            return False, self._settle_terminal_zero()
        CancelGoal = self._types["CancelGoal"]
        future = self.navigate_cancel_client.call_async(CancelGoal.Request())
        self._fault_cancel_future = future
        # The preview goal intentionally bypasses EpisodeGuard so it can be
        # redispatched after recovery.  Bind the inherited zero-settle wait to
        # this explicit cancel future instead of letting it assume no goal.
        self._terminal_cancel_requested = True
        self._terminal_cancel_future = future
        zero_confirmed = self._settle_terminal_zero()
        if not future.done():
            return False, zero_confirmed
        response = future.result()
        cancel_confirmed = bool(
            response is not None and int(response.return_code) == 0
        )
        return cancel_confirmed, zero_confirmed

    def _fault_leg(self, leg: MissionLeg, timeout_s: float) -> None:
        self._fault_preview_active = True
        self._fault_arc_length_m = 0.0
        self._navigation_terminal_observed = False
        # Do not create an untracked active goal unless its cancel endpoint is
        # already present.  Every later exit funnels through explicit cancel.
        if not self.navigate_cancel_client.wait_for_service(timeout_sec=2.0):
            self.guard.stop("F2_cancel_service_unavailable")
            self._fault_preview_active = False
            return
        self.route_goal_publisher.publish(self._goal_message(leg))
        self._event(
            "goal_dispatched",
            leg_id=leg.goal_id,
            leg_index=1,
            preview_for_fault=True,
        )
        reached = self._spin_until(
            lambda: self._fault_arc_length_m >= self.config.fault_min_arc_length_m
            or self.collision,
            timeout_s,
        )
        if not reached or self.collision:
            self._cancel_fault_preview(
                "fault_arc_timeout" if not reached else "collision_before_fault"
            )
            self.guard.stop("fault_arc_timeout" if not reached else "collision_before_fault")
            self._fault_preview_active = False
            return

        cancel_confirmed, zero_confirmed = self._cancel_fault_preview("F2")
        stationary = self._spin_until(
            lambda: self._cmd_vel_sim_zero_since is not None
            and time.monotonic() - self._cmd_vel_sim_zero_since
            >= self.config.stationary_hold_s,
            self.config.stationary_hold_s + self.TERMINAL_ZERO_TIMEOUT_SEC,
        )
        self._event(
            "pause_confirmed",
            fault_id=self.config.fault_id,
            cancel_confirmed=cancel_confirmed,
            cmd_vel_sim_zero=zero_confirmed,
            stationary=stationary,
        )
        if not (cancel_confirmed and zero_confirmed and stationary):
            self.guard.stop("F2_cancel_or_pause_not_confirmed")
            self._fault_preview_active = False
            return

        baseline = self._initialpose_count
        self._publish_seed(self.config.wrong_region_seed, "wrong_region")
        observed = self._spin_until(
            lambda: self._initialpose_count > baseline,
            self.TERMINAL_ZERO_TIMEOUT_SEC,
        )
        self._event(
            "fault_injected",
            fault_id=self.config.fault_id,
            arc_length_m=self._fault_arc_length_m,
            x=self.config.wrong_region_seed.x,
            y=self.config.wrong_region_seed.y,
            yaw_deg=self.config.wrong_region_seed.yaw_deg,
        )
        if not observed:
            self.guard.stop("F2_initialpose_not_observed")

    def _recover(self, method: str) -> None:
        baseline = self._amcl_count
        if method == "global_localization":
            if not self._call_empty_service(
                self.global_localization_client,
                "/global_localization",
                min(10.0, self.config.recovery_timeout_s),
            ):
                return
            # F2 is injected only after the robot is confirmed stationary, so
            # explicitly ask AMCL to consume the current scan once rather than
            # requiring unsafe motion to trigger its first global update.
            if not self._call_empty_service(
                self.nomotion_update_client,
                "/request_nomotion_update",
                min(10.0, self.config.recovery_timeout_s),
            ):
                return
        elif method == "supervisor_manual_rescue":
            Empty = self._types["Empty"]
            self.manual_rescue_publisher.publish(Empty())
            self._manual_rescue_count += 1
        else:
            self.guard.stop(f"unsupported_recovery:{method}")
            return

        recovered = self._spin_until(
            lambda: self._amcl_recovered(baseline)
            and (method != "supervisor_manual_rescue" or self._supervisor_recovered()),
            self.config.recovery_timeout_s,
        )
        self._event(
            "localization_recovered",
            success=recovered,
            method=method,
            state=self._last_supervisor.get("state", ""),
            reason=self._last_supervisor.get("reason", ""),
            result=self._last_supervisor.get("recovery_result", ""),
        )
        if not recovered:
            self.guard.stop(f"localization_recovery_timeout:{method}")
            self._fault_preview_active = False
            return
        # Ignore the expected canceled-leg terminal traffic throughout the
        # recovery window; only the post-recovery G3 redispatch belongs to the
        # ordinary EpisodeGuard sequence.
        self._fault_preview_active = False
        # F2 used terminal-zero only as a bounded pause.  Rearm it so the
        # ordinary final boundary is checked after continued navigation.
        self._terminal_started_monotonic = None
        self._terminal_cancel_requested = False
        self._terminal_cancel_future = None
        self._terminal_zero_settled = False
        self._terminal_zero_confirmed = False
        self._terminal_zero_reason = "not_required"
        self._cmd_vel_sim_zero_stamps.clear()
        self._navigation_terminal_observed = False

    def perform_action(self, action: Mapping[str, Any]) -> None:
        if self.guard.state in {"STOP", "FAILED"}:
            return
        by_id = {leg.goal_id: leg for leg in self.manifest.mission_legs}
        kind = str(action.get("action", ""))
        if kind == "goal":
            leg_id = str(action["leg_id"])
            self._run_mission_leg(
                index=self.config.route_ids.index(leg_id),
                leg=by_id[leg_id],
                dynamic_group="",
                reset_timeout_sec=10.0,
                navigation_timeout_sec=self._navigation_timeout_sec,
            )
        elif kind == "fault_leg":
            self._fault_leg(by_id[str(action["leg_id"])], self._navigation_timeout_sec)
        elif kind == "recover":
            self._recover(str(action["method"]))
        else:
            self.guard.stop(f"unsupported_action:{kind}")

    def _reset_and_localize(
        self,
        *,
        readiness_timeout_sec: float,
        reset_timeout_sec: float,
    ) -> bool:
        self._assert_ground_truth_firewall()
        ready = self._spin_until(self._pre_reset_ready, readiness_timeout_sec)
        if not ready:
            self.guard.stop(
                "readiness_timeout:" + (self._readiness_blockers() or "unknown")
            )
            return False
        self.guard.arm_reset(self.facts)
        self._set_episode_parameters(reset_timeout_sec)

        Trigger = self._types["Trigger"]
        self.guard.record_reset_call()
        future = self.reset_client.call_async(Trigger.Request())
        if not self._spin_until(future.done, reset_timeout_sec):
            self.guard.record_reset_response(None)
            return False
        response = future.result()
        self.guard.record_reset_response(response.success if response is not None else None)
        if self.guard.state == "STOP":
            return False
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
            return False
        self.guard.record_reset_receipt_generation(int(self.reset_receipt["generation"]))
        if not self._spin_until(
            lambda: self.guard.reset_events == 1 or self.guard.state == "STOP",
            reset_timeout_sec,
        ):
            self.guard.stop("reset_event_timeout")
            return False
        if self.guard.state == "STOP":
            return False
        self._emit_episode_start()

        # R0/R1 share the same startup prior.  Their only Phase E switched
        # factor is the recovery action after the identical F2 injection.
        baseline_amcl_count = self._amcl_count
        if self.arm != "S1":
            self._publish_seed(self.config.broad_seed, "broad_initialpose")
        localized = self._spin_until(
            lambda: (
                self.guard.localization_ready
                and self._amcl_recovered(baseline_amcl_count)
            )
            or self.guard.state == "STOP",
            reset_timeout_sec,
        )
        if (
            not localized
            or not self.guard.localization_ready
            or not self._amcl_recovered(baseline_amcl_count)
        ):
            self.guard.stop("startup_localization_timeout")
            return False
        self._event(
            "localization_ready",
            ready=True,
            source=("supervisor" if self.arm == "S1" else "runner"),
            initialpose_count=self._initialpose_count,
        )
        self._check_post_reset_odom()
        if self.guard.state == "STOP":
            return False
        self._wait_nav2_and_tf_ready(reset_timeout_sec)
        if not (self.guard.nav2_active and self.guard.tf_active):
            return False
        released = self._spin_until(
            lambda: self.guard.goal_ready or self.guard.state == "STOP",
            reset_timeout_sec,
        )
        if not released or not self.guard.goal_ready:
            self.guard.stop("reset_gate_release_timeout")
            return False
        return True

    def run_causal(
        self,
        *,
        readiness_timeout_sec: float,
        reset_timeout_sec: float,
        navigation_timeout_sec: float,
    ) -> dict[str, Any]:
        self._navigation_timeout_sec = navigation_timeout_sec
        if self._reset_and_localize(
            readiness_timeout_sec=readiness_timeout_sec,
            reset_timeout_sec=reset_timeout_sec,
        ):
            execute_route_actions(self, route_actions(self.config, self.arm))
        if not self._event_stream_started:
            self._emit_episode_start()
        result = super().result()
        self._event(
            "episode_end",
            state=result["state"],
            stop_reason=result["stop_reason"],
            collision=result["collision"],
            terminal_zero_confirmed=result["terminal_zero_confirmed"],
            initialpose_count=self._initialpose_count,
            manual_rescue_count=self._manual_rescue_count,
            completed_leg_ids=result["completed_leg_ids"],
        )
        return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("config", "plan", "run"))
    parser.add_argument("--config", required=True)
    parser.add_argument("--arm", choices=ARMS)
    parser.add_argument("--run-id")
    parser.add_argument("--seed", type=int)
    parser.add_argument("--output-jsonl")
    parser.add_argument("--readiness-timeout-sec", type=float, default=120.0)
    parser.add_argument("--reset-timeout-sec", type=float, default=120.0)
    parser.add_argument("--navigation-timeout-sec", type=float, default=900.0)
    return parser


def cli(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        config = load_config(args.config)
        if args.command == "config":
            output = {
                "schema_version": CONFIG_SCHEMA,
                "event_schema": EVENT_SCHEMA,
                "phase_b_manifest": str(config.phase_b_manifest),
                "phase_d_run4_candidate": dict(config.phase_d_run4_candidate),
                "arms": list(ARMS),
                "qualification": QUALIFICATION,
                "formal_qualification": NOT_QUALIFIED,
            }
            print(json.dumps(output, indent=2, sort_keys=True))
            return 0
        if args.command == "plan":
            print(json.dumps(build_plan(config), indent=2, sort_keys=True))
            return 0
        if not args.arm or not args.output_jsonl:
            raise LocalizationConfigError("run requires --arm and --output-jsonl")
        arm = args.arm
        seed = config.seeds[arm] if args.seed is None else args.seed
        if seed < 0:
            raise LocalizationConfigError("--seed must be non-negative")
        run_id = args.run_id or f"v6-phase-{PHASE_BY_ARM[arm].lower()}-{arm.lower()}-{seed}"
        if not run_id.strip():
            raise LocalizationConfigError("--run-id must be non-empty")
        output_path = Path(args.output_jsonl).expanduser().resolve()
        if output_path.exists():
            raise LocalizationConfigError(f"refusing to overwrite {output_path}")

        manifest = load_phase_b_manifest(config.phase_b_manifest)
        for field in (
            "module2_navigation_write_enabled",
            "cognitive_place_graph_enabled",
            "low_obstacles_enabled",
            "dynamic_actors_enabled",
        ):
            if manifest.runtime.get(field) is not False:
                raise LocalizationConfigError(f"Phase B {field} must remain off")
        episode = replace(manifest.episodes[0], seed=seed)

        import rclpy

        rclpy.init(args=None)
        adapter = LocalizationCausalNode(
            manifest,
            episode,
            output_path,
            config=config,
            arm=arm,
            run_id=run_id,
        )
        try:
            result = adapter.run_causal(
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


def main(argv: Sequence[str] | None = None) -> None:
    raise SystemExit(cli(argv))


if __name__ == "__main__":
    main()
