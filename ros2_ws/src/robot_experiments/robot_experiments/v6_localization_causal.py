"""Minimal single-round Phase D/E localization live adapter.

The adapter reuses :class:`V6FormalNode` for cold reset, readiness, route
dispatch, collision handling, and terminal-zero settling.  It adds only the
four localization arms required by the current plan:

* D/S0 -- one frozen broad runner ``/initialpose`` with supervisor shadow;
* D/S1 -- no runner seed, supervisor ``startup`` owns the first prior write;
* E/R0 -- AMCL particle-spread fault with supervisor shadow;
* E/R1 -- the same fault followed by one explicit manual supervisor request.

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
    "R0": "amcl_no_cognitive_write",
    "R1": "supervisor_manual_rescue",
}
STARTUP_AMCL_POSES_REQUIRED = 3
RUN4_CANDIDATE_STATUS = "READ_ONLY_CAUSAL_CANDIDATE_STARTUP_ONLY"
RUN4_RECOVERY_QUALIFICATION = "NOT_ACTIVE_RECOVERY_QUALIFIED"
RUN4_ALLOWED_SUPERVISOR_MODES = ("shadow", "startup")
PHASE_E_MANUAL_RECOVERY_EXPERIMENT = {
    "status": "ENGINEERING_EXPLICIT_MANUAL_RECOVERY_ONLY",
    "allowed_supervisor_modes": ["active"],
    "requires_explicit_request": True,
    "auto_rescue_enabled": False,
}
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
        "particle_cloud",
        "manual_rescue_requested",
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


def _require_sim_time(
    node: Any, parameter_type: Any, ros_clock_type: Any
) -> None:
    results = node.set_parameters([parameter_type("use_sim_time", value=True)])
    if (
        len(results) != 1
        or not results[0].successful
        or node.get_parameter("use_sim_time").value is not True
        or node.get_clock().clock_type != ros_clock_type
    ):
        raise V6ContractError(
            "Phase D/E localization runner requires use_sim_time=true"
        )


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
    phase_e_run4_candidate: Mapping[str, Any]
    seeds: Mapping[str, int]
    broad_seed: SeedPose
    route_ids: tuple[str, ...]
    fault_id: str
    fault_kind: str
    fault_service: str
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
        "phase_e_run4_candidate",
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
        "use_sim_time": True,
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

    phase_e_candidate = _mapping(
        raw.get("phase_e_run4_candidate"), "phase_e_run4_candidate"
    )
    phase_e_candidate_keys = candidate_keys - {"startup_initialpose"} | {
        "manual_recovery_experiment"
    }
    if set(phase_e_candidate) != phase_e_candidate_keys:
        raise LocalizationConfigError(
            "phase_e_run4_candidate keys changed"
        )
    for name in candidate_keys - {"startup_initialpose"}:
        if phase_e_candidate.get(name) != candidate.get(name):
            raise LocalizationConfigError(
                f"Phase E Run4 candidate {name} must match Phase D"
            )
    manual_experiment = _mapping(
        phase_e_candidate.get("manual_recovery_experiment"),
        "phase_e_run4_candidate.manual_recovery_experiment",
    )
    if dict(manual_experiment) != PHASE_E_MANUAL_RECOVERY_EXPERIMENT:
        raise LocalizationConfigError(
            "Phase E manual recovery experiment block changed"
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
        "kind",
        "service",
        "stationary_hold_s",
    }:
        raise LocalizationConfigError("fault keys changed")
    if fault.get("id") != "F2":
        raise LocalizationConfigError("the only supported fault id is F2")
    if fault.get("kind") != "amcl_global_localization_particle_spread":
        raise LocalizationConfigError("F2 must use AMCL global particle spread")
    if fault.get("service") != "/reinitialize_global_localization":
        raise LocalizationConfigError(
            "F2 must use /reinitialize_global_localization"
        )
    stationary_hold = _finite(
        fault["stationary_hold_s"], "fault.stationary_hold_s"
    )
    if stationary_hold <= 0.0:
        raise LocalizationConfigError("fault.stationary_hold_s must be positive")
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
        phase_e_run4_candidate=dict(phase_e_candidate),
        seeds=seeds,
        broad_seed=broad_seed,
        route_ids=("G2", "G3", "G4", "G5", "G1"),
        fault_id="F2",
        fault_kind="amcl_global_localization_particle_spread",
        fault_service="/reinitialize_global_localization",
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
            "action": "fault",
            "fault_id": config.fault_id,
            "kind": config.fault_kind,
            "service": config.fault_service,
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
                "run4_candidate_enabled": True,
                "expected_startup_initialpose": (
                    PHASE_D_STARTUP_INITIALPOSE[arm]
                    if PHASE_BY_ARM[arm] == "D"
                    else {
                        "source": "runner",
                        "seed_kind": "broad_initialpose",
                        "expected_total_count": 1,
                        "expected_supervisor_count": 0,
                    }
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
        "phase_e_run4_candidate": dict(config.phase_e_run4_candidate),
        "phase_e_run4_candidate_enabled": True,
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
        self._supervisor_initialpose_count = 0
        self._manual_rescue_count = 0
        self._fault_service_request_count = 0
        self._nomotion_request_count = 0
        self._prior_write_count = 0
        self._amcl_count = 0
        self._particle_cloud_count = 0
        self._last_amcl_covariance: tuple[float, float, float] | None = None
        self._first_post_fault_amcl_covariance: tuple[float, float, float] | None = None
        self._fault_observation_active = False
        self._fault_amcl_baseline = 0
        self._fault_particle_baseline = 0
        self._fault_stamp_ns = 0
        self._fault_candidate_baseline: dict[str, int] = {}
        self._last_supervisor: dict[str, str] = {}
        self._last_cmd_zero: bool | None = None
        self._cmd_vel_sim_zero_since: float | None = None
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
        from nav2_msgs.msg import ParticleCloud
        from rcl_interfaces.msg import SetParametersResult
        from rclpy.clock import ClockType
        from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from rclpy.parameter import Parameter
        from std_msgs.msg import Empty
        from std_srvs.srv import Empty as EmptyService

        _require_sim_time(self.node, Parameter, ClockType.ROS_TIME)

        def lock_sim_time(parameters: Sequence[Any]) -> SetParametersResult:
            for parameter in parameters:
                if parameter.name == "use_sim_time" and parameter.value is not True:
                    return SetParametersResult(
                        successful=False,
                        reason=(
                            "Phase D/E localization runner requires "
                            "use_sim_time=true"
                        ),
                    )
            return SetParametersResult(successful=True)

        self.node.add_on_set_parameters_callback(lock_sim_time)
        self._ros_clock_type = ClockType.ROS_TIME

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
        self.reinitialize_global_localization_client = self.node.create_client(
            EmptyService, "/reinitialize_global_localization"
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
                self.node.create_subscription(
                    ParticleCloud,
                    "/particle_cloud",
                    self._particle_cloud,
                    qos_profile_sensor_data,
                ),
            ]
        )

    def _stamp_s(self) -> float:
        clock = self.node.get_clock()
        if (
            self.node.get_parameter("use_sim_time").value is not True
            or clock.clock_type != self._ros_clock_type
        ):
            raise V6ContractError(
                "Phase D/E localization runner lost use_sim_time=true"
            )
        return float(clock.now().nanoseconds) / 1.0e9

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
        post_fault = bool(
            self._fault_observation_active
            and self._amcl_count > self._fault_amcl_baseline
        )
        if post_fault and self._first_post_fault_amcl_covariance is None:
            self._first_post_fault_amcl_covariance = self._last_amcl_covariance
        pose = message.pose.pose
        self._event(
            "estimated_pose",
            x=float(pose.position.x),
            y=float(pose.position.y),
            yaw_deg=_yaw_deg(pose.orientation),
            covariance_xy_yaw=[covariance[0], covariance[7], covariance[35]],
            post_fault=post_fault,
        )

    def _particle_cloud(self, message: Any) -> None:
        self._particle_cloud_count += 1
        self._event(
            "particle_cloud",
            count=self._particle_cloud_count,
            particle_count=len(message.particles),
            post_fault=bool(
                self._fault_observation_active
                and self._particle_cloud_count > self._fault_particle_baseline
            ),
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
            self._supervisor_initialpose_count += 1
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

    def _call_nomotion_update(self, timeout_s: float) -> bool:
        self._nomotion_request_count += 1
        return self._call_empty_service(
            self.nomotion_update_client,
            "/request_nomotion_update",
            timeout_s,
        )

    def _covariance_recovered(
        self, covariance: tuple[float, float, float] | None
    ) -> bool:
        return bool(
            covariance is not None
            and max(covariance[0], covariance[1])
            <= self.config.recovered_xy_variance_m2
            and covariance[2] <= self.config.recovered_yaw_variance_rad2
        )

    def _amcl_recovered(self, baseline_count: int) -> bool:
        covariance = self._last_amcl_covariance
        return bool(
            self._amcl_count >= baseline_count + STARTUP_AMCL_POSES_REQUIRED
            and self._covariance_recovered(covariance)
        )

    def _request_stationary_amcl_updates(
        self, baseline_count: int, timeout_s: float
    ) -> bool:
        """Request only the missing Phase D startup poses while stationary."""

        target_count = baseline_count + STARTUP_AMCL_POSES_REQUIRED
        request_timeout_s = min(10.0, timeout_s)
        requests = 0
        while (
            self._amcl_count < target_count
            and requests < STARTUP_AMCL_POSES_REQUIRED
        ):
            previous_count = self._amcl_count
            if not self._call_nomotion_update(request_timeout_s):
                return False
            requests += 1
            observed = self._spin_until(
                lambda: self._amcl_count > previous_count
                or self.guard.state == "STOP",
                request_timeout_s,
            )
            if self.guard.state == "STOP":
                return False
            if not observed:
                self.guard.stop("startup_nomotion_update_pose_timeout")
                return False
        if self._amcl_count < target_count:
            self.guard.stop("startup_nomotion_update_exhausted")
            return False
        return True

    def _supervisor_recovered(self) -> bool:
        return self._last_supervisor.get("recovery_result", "").lower() in {
            "succeeded", "success", "seed_confirmed"
        }

    @staticmethod
    def _diagnostic_int(values: Mapping[str, str], name: str) -> int:
        try:
            return int(values.get(name, "0") or 0)
        except ValueError:
            return -1

    def _candidate_snapshot(self) -> dict[str, int]:
        return {
            name: self._diagnostic_int(self._last_supervisor, name)
            for name in (
                "candidate_array_last_validation_stamp_ns",
                "candidate_array_received_count",
                "candidate_array_accepted_count",
                "candidate_array_last_sequence",
            )
        }

    def _fresh_post_fault_validated_candidate(self) -> bool:
        values = self._last_supervisor
        baseline = self._fault_candidate_baseline
        return bool(
            self._diagnostic_int(
                values, "candidate_array_last_validation_stamp_ns"
            )
            > self._fault_stamp_ns
            and self._diagnostic_int(values, "candidate_array_received_count")
            > baseline["candidate_array_received_count"]
            and self._diagnostic_int(values, "candidate_array_accepted_count")
            > baseline["candidate_array_accepted_count"]
            and self._diagnostic_int(values, "candidate_array_last_sequence")
            > baseline["candidate_array_last_sequence"]
            and self._diagnostic_int(
                values, "candidate_array_last_candidate_count"
            )
            > 0
            and values.get("candidate_array_last_structural_rejection", "") == ""
            and values.get(
                "candidate_array_last_state_machine_decision_reason", ""
            )
            == "no_authorized_rescue_request"
            and values.get("candidate_array_last_event_reason", "")
            == "no_authorized_rescue_request"
            and values.get("candidate_validation", "") == "fresh"
            and str(values.get("candidate_identity", "")).strip()
            and values.get("state", "").upper() == "LOST"
            and "covariance" in values.get("reason", "").lower()
        )

    def _fault(self) -> None:
        if self._fault_service_request_count:
            self.guard.stop("F2_fault_service_retry_forbidden")
            return
        if (
            self._supervisor_initialpose_count != 0
            or self._diagnostic_int(self._last_supervisor, "reset_attempts") != 0
            or self._last_supervisor.get("state", "").upper() != "NORMAL"
            or self._last_supervisor.get("reason", "") != "amcl_healthy"
        ):
            self.guard.stop("pre_fault_supervisor_state_invalid")
            return
        no_active_goal = self.guard.goal_publications == len(
            self.guard.completed_leg_ids
        )
        self._event(
            "pause_requested",
            reason="post_G2_fault_boundary",
            fault_id=self.config.fault_id,
            active_goal=not no_active_goal,
        )
        stationary = self._spin_until(
            lambda: self._cmd_vel_sim_zero_since is not None
            and time.monotonic() - self._cmd_vel_sim_zero_since
            >= self.config.stationary_hold_s,
            self.config.stationary_hold_s + self.TERMINAL_ZERO_TIMEOUT_SEC,
        )
        self._event(
            "pause_confirmed",
            fault_id=self.config.fault_id,
            no_active_goal=no_active_goal,
            cmd_vel_sim_zero=self._last_cmd_zero is True,
            stationary=stationary,
        )
        if not (no_active_goal and self._last_cmd_zero is True and stationary):
            self.guard.stop("F2_stationary_boundary_not_confirmed")
            return

        self._fault_candidate_baseline = self._candidate_snapshot()
        self._fault_service_request_count = 1
        if not self._call_empty_service(
            self.reinitialize_global_localization_client,
            self.config.fault_service,
            min(10.0, self.config.recovery_timeout_s),
        ):
            return
        # Fence observations after the successful service future. Particle
        # cloud is best-effort corroboration; AMCL pose is the required sample.
        self._fault_stamp_ns = int(self.node.get_clock().now().nanoseconds)
        self._fault_amcl_baseline = self._amcl_count
        self._fault_particle_baseline = self._particle_cloud_count
        self._first_post_fault_amcl_covariance = None
        self._fault_observation_active = True
        observed = self._spin_until(
            lambda: self._amcl_count > self._fault_amcl_baseline
            or self.guard.state == "STOP",
            min(10.0, self.config.recovery_timeout_s),
        )
        if not observed or self.guard.state == "STOP":
            self.guard.stop("F2_first_post_fault_evidence_timeout")
            return
        lost_observed = False
        if not self._covariance_recovered(self._first_post_fault_amcl_covariance):
            lost_observed = self._spin_until(
                lambda: (
                    self._last_supervisor.get("state", "").upper() == "LOST"
                    and self._last_supervisor.get("reason", "")
                    == "amcl_covariance_lost"
                )
                or self.guard.state == "STOP",
                self.config.recovery_timeout_s,
            )
        fault_outcome = (
            "FAULT_DISCRIMINATIVE"
            if lost_observed
            else "INVALID_FAULT_NOT_DISCRIMINATIVE"
        )
        self._event(
            "fault_injected",
            fault_id=self.config.fault_id,
            kind=self.config.fault_kind,
            service=self.config.fault_service,
            service_request_count=self._fault_service_request_count,
            service_response_observed=True,
            first_post_fault_particle_cloud_observed=(
                self._particle_cloud_count > self._fault_particle_baseline
            ),
            first_post_fault_amcl_pose_observed=True,
            first_post_fault_covariance_xy_yaw=list(
                self._first_post_fault_amcl_covariance or ()
            ),
            outcome=fault_outcome,
            supervisor_lost_observed=lost_observed,
        )
        if fault_outcome == "INVALID_FAULT_NOT_DISCRIMINATIVE":
            self.guard.stop(fault_outcome)

    def _recover(self, method: str) -> None:
        baseline = self._amcl_count
        if method == "amcl_no_cognitive_write":
            if self._supervisor_initialpose_count:
                self.guard.stop("R0_supervisor_initialpose_forbidden")
                return
            if not self._call_nomotion_update(
                min(10.0, self.config.recovery_timeout_s),
            ):
                return
        elif method == "supervisor_manual_rescue":
            candidate_ready = self._spin_until(
                lambda: self._fresh_post_fault_validated_candidate()
                or self.guard.state == "STOP",
                self.config.recovery_timeout_s,
            )
            if not candidate_ready or not self._fresh_post_fault_validated_candidate():
                self.guard.stop("post_fault_validated_candidate_timeout")
                return
            if self._manual_rescue_count:
                self.guard.stop("manual_rescue_retry_forbidden")
                return
            supervisor_initialpose_baseline = self._supervisor_initialpose_count
            Empty = self._types["Empty"]
            self.manual_rescue_publisher.publish(Empty())
            self._manual_rescue_count += 1
            self._event(
                "manual_rescue_requested",
                count=self._manual_rescue_count,
                purpose="ENGINEERING_EXPLICIT_MANUAL_RECOVERY_ONLY",
            )
            initialpose_observed = self._spin_until(
                lambda: self._supervisor_initialpose_count
                > supervisor_initialpose_baseline
                or self.guard.state == "STOP",
                min(10.0, self.config.recovery_timeout_s),
            )
            if (
                not initialpose_observed
                or self._supervisor_initialpose_count
                != supervisor_initialpose_baseline + 1
            ):
                self.guard.stop("supervisor_initialpose_count_not_exactly_one")
                return
        else:
            self.guard.stop(f"unsupported_recovery:{method}")
            return

        recovered = self._spin_until(
            lambda: self._amcl_recovered(baseline)
            and (method != "supervisor_manual_rescue" or self._supervisor_recovered()),
            self.config.recovery_timeout_s,
        )
        if method == "amcl_no_cognitive_write" and self._supervisor_initialpose_count:
            self.guard.stop("R0_supervisor_initialpose_forbidden")
            return
        if (
            method == "supervisor_manual_rescue"
            and self._supervisor_initialpose_count != 1
        ):
            self.guard.stop("supervisor_initialpose_count_not_exactly_one")
            return
        self._event(
            "localization_recovered",
            success=recovered,
            method=method,
            state=self._last_supervisor.get("state", ""),
            reason=self._last_supervisor.get("reason", ""),
            result=self._last_supervisor.get("recovery_result", ""),
            fault_service_request_count=self._fault_service_request_count,
            nomotion_request_count=self._nomotion_request_count,
            manual_rescue_count=self._manual_rescue_count,
        )
        if not recovered:
            self.guard.stop(f"localization_recovery_timeout:{method}")
            return
        self._fault_observation_active = False

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
        elif kind == "fault":
            self._fault()
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
        # The startup supervisor can publish S1's sole seed while this runner
        # is spinning the reset future/event. Fence that asynchronous seed
        # before issuing reset; S0 remains fenced immediately before its own
        # deterministic runner publication below.
        s1_baseline_amcl_count = self._amcl_count
        s1_baseline_initialpose_count = self._initialpose_count
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
        if self.arm == "S1":
            baseline_amcl_count = s1_baseline_amcl_count
            baseline_initialpose_count = s1_baseline_initialpose_count
        else:
            baseline_amcl_count = self._amcl_count
            baseline_initialpose_count = self._initialpose_count
            self._publish_seed(self.config.broad_seed, "broad_initialpose")
        seed_observed = self._spin_until(
            lambda: self._initialpose_count > baseline_initialpose_count
            or self.guard.state == "STOP",
            reset_timeout_sec,
        )
        if (
            not seed_observed
            or self.guard.state == "STOP"
            or self._initialpose_count <= baseline_initialpose_count
        ):
            self.guard.stop("startup_initialpose_timeout")
            return False
        if not self._request_stationary_amcl_updates(
            baseline_amcl_count, reset_timeout_sec
        ):
            return False
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
            supervisor_initialpose_count=self._supervisor_initialpose_count,
            fault_service_request_count=self._fault_service_request_count,
            nomotion_request_count=self._nomotion_request_count,
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
                "phase_e_run4_candidate": dict(config.phase_e_run4_candidate),
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
