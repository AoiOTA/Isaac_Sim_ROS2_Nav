"""Active-controller TTL probe for the Phase-F M2/M3 arms.

This probe is deliberately separate from the nominal G1->G2 episode.  It
starts the same goal, waits until the cognitive obstacle has affected both
costmaps (and the M3 critic), stops only the Module2/Bridge producer while the
goal is still active, and observes fail-open clearing on simulation time.
Ground Truth is never subscribed by the dispatcher; it remains recorder-only
offline evidence.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import json
import math
import os
from pathlib import Path
import subprocess
import time
from typing import Any, Callable, Mapping, Protocol, Sequence

import yaml

from .v6_low_obstacle_causal import (
    CausalContractError,
    CausalManifest,
    RunContract,
    _ManagedProcess,
    _adapter_values,
    _confirm_arm_cleanup,
    _rosbag_command,
    _start_process,
    _stop_process,
    _wait_for_cognitive_ready,
    _wait_for_startup_ready,
    exact_adapter_templates,
    load_manifest,
    render_adapter_command,
)


SCHEMA_VERSION = "bio_nav_v6_phase_f_active_ttl_probe_v1"
ACTIVE_ARMS = ("M2", "M3")
PASS_STATE = "PASS_ACTIVE_CONTROLLER_TTL"
PROBE_NOT_ARMED = "PROBE_NOT_ARMED"
DEFAULT_ARMING_TIMEOUT_SEC = 180.0
DEFAULT_PROBE_TIMEOUT_SEC = 30.0
DEFAULT_SHUTDOWN_TIMEOUT_SEC = 20.0


def _time_ns(value: Any) -> int | None:
    if value is None:
        return None
    try:
        return int(value.sec) * 1_000_000_000 + int(value.nanosec)
    except (AttributeError, TypeError, ValueError):
        return None


def _consumer_scope(consumer: str) -> str | None:
    text = str(consumer).lower()
    if "global" in text:
        return "global"
    if "local" in text:
        return "local"
    return None


@dataclass
class ActiveTtlTimeline:
    """Pure simulation-time state used by both the live adapter and tests."""

    arm: str
    margin_ns: int
    clock_ns: int = 0
    goal_started: bool = False
    action_active: bool = False
    collision: bool = False
    terminal_reason: str = ""
    terminal_before_dropout: bool = False
    trusted_typed_seen: bool = False
    latest_source_sequence: int | None = None
    latest_validation_stamp_ns: int | None = None
    latest_validation_ttl_ns: int | None = None
    positive_layers: set[str] = field(default_factory=set)
    positive_critic: bool = False
    producer_stopped: bool = False
    active_at_dropout: bool = False
    armed_source_sequence: int | None = None
    expiry_target_ns: int | None = None
    clear_layers: set[str] = field(default_factory=set)
    critic_stale_rejected: bool = False
    post_expiry_layer_applied: bool = False
    post_expiry_critic_applied: bool = False
    post_expiry_applied: bool = False
    cancel_count: int = 0
    terminal_zero_confirmed: bool = False
    events: list[dict[str, Any]] = field(default_factory=list)

    def __post_init__(self) -> None:
        if self.arm not in ACTIVE_ARMS:
            raise CausalContractError("active TTL probe supports only M2 and M3")
        if self.margin_ns <= 0:
            raise CausalContractError("active TTL probe margin must be positive")

    def _event(self, kind: str, **payload: Any) -> None:
        self.events.append({"event": kind, "clock_ns": self.clock_ns, **payload})

    def observe_clock(self, stamp_ns: int) -> None:
        value = int(stamp_ns)
        if value < self.clock_ns:
            raise CausalContractError("simulation clock moved backwards during active TTL probe")
        self.clock_ns = value

    def start_goal(self) -> None:
        self.goal_started = True
        self.action_active = True
        self._event("goal_started")

    def observe_terminal(self, reason: str, *, collision: bool = False) -> None:
        self.action_active = False
        self.collision = self.collision or bool(collision)
        self.terminal_reason = str(reason)
        if not self.producer_stopped:
            self.terminal_before_dropout = True
        self._event("goal_terminal", reason=self.terminal_reason, collision=self.collision)

    def observe_typed(
        self,
        *,
        source_sequence: int,
        validation_stamp_ns: int,
        validation_ttl_ns: int,
        trusted_write: bool,
        healthy: bool,
        observation_valid: bool,
        obstacle_count: int,
    ) -> None:
        if (
            not trusted_write
            or not healthy
            or not observation_valid
            or int(obstacle_count) <= 0
            or int(validation_ttl_ns) <= 0
        ):
            return
        sequence = int(source_sequence)
        validation_stamp = int(validation_stamp_ns)
        if self.latest_source_sequence is not None and sequence < self.latest_source_sequence:
            return
        self.trusted_typed_seen = True
        self.latest_source_sequence = sequence
        self.latest_validation_stamp_ns = validation_stamp
        self.latest_validation_ttl_ns = int(validation_ttl_ns)
        self._event(
            "trusted_typed_obstacle",
            source_sequence=sequence,
            validation_stamp_ns=validation_stamp,
            validation_ttl_ns=int(validation_ttl_ns),
        )

    def _matches_armed_source(self, source_sequence: int) -> bool:
        return (
            self.armed_source_sequence is not None
            and int(source_sequence) >= self.armed_source_sequence
        )

    def observe_layer(
        self,
        *,
        consumer: str,
        source_sequence: int,
        applied: bool,
        raised_cell_count: int,
        active_cell_count: int,
        maximum_cost_increase: int,
        reason: str,
    ) -> None:
        scope = _consumer_scope(consumer)
        if scope is None:
            return
        positive = bool(applied) and int(raised_cell_count) > 0
        if not self.producer_stopped:
            if positive:
                self.positive_layers.add(scope)
                self._event(
                    "positive_layer_apply",
                    scope=scope,
                    source_sequence=int(source_sequence),
                    raised_cell_count=int(raised_cell_count),
                )
            return
        if self.expiry_target_ns is None or self.clock_ns < self.expiry_target_ns:
            return
        if not self._matches_armed_source(source_sequence):
            return
        stale = "validation_stale" in str(reason)
        zero = (
            not bool(applied)
            and int(raised_cell_count) == 0
            and int(active_cell_count) == 0
            and int(maximum_cost_increase) == 0
        )
        if bool(applied) or int(raised_cell_count) > 0 or int(maximum_cost_increase) > 0:
            self.post_expiry_layer_applied = True
            self.post_expiry_applied = True
        if stale and zero:
            self.clear_layers.add(scope)
            self._event(
                "post_expiry_layer_clear",
                scope=scope,
                source_sequence=int(source_sequence),
            )

    def observe_critic(
        self,
        *,
        source_sequence: int,
        applied: bool,
        reason: str,
    ) -> None:
        text = str(reason)
        cost_delta_applied = "cost_delta_applied=true" in text
        positive = bool(applied) and cost_delta_applied
        if not self.producer_stopped:
            if positive:
                self.positive_critic = True
                self._event(
                    "positive_critic_apply", source_sequence=int(source_sequence)
                )
            return
        if self.expiry_target_ns is None or self.clock_ns < self.expiry_target_ns:
            return
        if not self._matches_armed_source(source_sequence):
            return
        if bool(applied) or cost_delta_applied:
            self.post_expiry_critic_applied = True
            self.post_expiry_applied = True
        stale = "validation_stale" in text
        no_delta = "cost_delta_applied=false" in text or not cost_delta_applied
        if stale and not bool(applied) and no_delta:
            self.critic_stale_rejected = True
            self._event(
                "post_expiry_critic_stale_rejected",
                source_sequence=int(source_sequence),
            )

    @property
    def armed(self) -> bool:
        return bool(
            self.action_active
            and self.trusted_typed_seen
            and {"global", "local"}.issubset(self.positive_layers)
            and (self.arm != "M3" or self.positive_critic)
        )

    def mark_producer_stopped(self) -> None:
        if not self.armed:
            raise CausalContractError("active TTL probe cannot stop producer before positive apply")
        if (
            self.latest_source_sequence is None
            or self.latest_validation_stamp_ns is None
            or self.latest_validation_ttl_ns is None
        ):
            raise CausalContractError("active TTL probe lacks a trusted validation timeline")
        self.producer_stopped = True
        self.active_at_dropout = self.action_active
        self.armed_source_sequence = self.latest_source_sequence
        self.expiry_target_ns = (
            self.latest_validation_stamp_ns
            + self.latest_validation_ttl_ns
            + self.margin_ns
        )
        self._event(
            "producer_stopped",
            source_sequence=self.armed_source_sequence,
            expiry_target_ns=self.expiry_target_ns,
            action_active=self.active_at_dropout,
        )

    @property
    def clear_complete(self) -> bool:
        return bool(
            self.producer_stopped
            and self.expiry_target_ns is not None
            and self.clock_ns >= self.expiry_target_ns
            and {"global", "local"}.issubset(self.clear_layers)
            and (self.arm != "M3" or self.critic_stale_rejected)
            and not self.post_expiry_applied
        )

    def mark_cancel(self) -> None:
        self.cancel_count += 1
        self.action_active = False
        self._event("goal_cancel_requested", count=self.cancel_count)

    def mark_terminal_zero(self, confirmed: bool) -> None:
        self.terminal_zero_confirmed = bool(confirmed)
        self._event("terminal_zero", confirmed=self.terminal_zero_confirmed)

    def result(self, *, timeout_reason: str = "") -> dict[str, Any]:
        reasons: list[str] = []
        if self.terminal_before_dropout:
            state = "FAIL_EARLY_TERMINAL"
            reasons.append(self.terminal_reason or "goal_terminal_before_dropout")
        elif not self.producer_stopped:
            state = PROBE_NOT_ARMED
            reasons.append(timeout_reason or "positive_apply_not_observed")
        elif self.terminal_reason:
            state = "FAIL_GOAL_TERMINATED_DURING_PROBE"
            reasons.append(self.terminal_reason)
        elif self.post_expiry_applied:
            state = "FAIL_POST_EXPIRY_APPLIED"
            reasons.append("stale_input_applied_after_expiry")
        elif not {"global", "local"}.issubset(self.clear_layers):
            state = "FAIL_MISSING_LAYER_CLEAR"
            reasons.append("global_or_local_zero_write_missing")
        elif self.arm == "M3" and not self.critic_stale_rejected:
            state = "FAIL_MISSING_CRITIC_STALE_REJECTION"
            reasons.append("active_critic_stale_callback_missing")
        elif self.cancel_count != 1:
            state = "FAIL_CANCEL_COUNT"
            reasons.append(f"goal_cancel_count={self.cancel_count}")
        elif not self.terminal_zero_confirmed:
            state = "FAIL_TERMINAL_ZERO"
            reasons.append("terminal_zero_not_confirmed")
        else:
            state = PASS_STATE
        return {
            "schema_version": SCHEMA_VERSION,
            "arm": self.arm,
            "state": state,
            "reasons": reasons,
            "probe_armed": self.producer_stopped,
            "producer_stop_while_action_active": self.active_at_dropout,
            "sim_clock_ns": self.clock_ns,
            "armed_source_sequence": self.armed_source_sequence,
            "validation_stamp_ns": self.latest_validation_stamp_ns,
            "validation_ttl_ns": self.latest_validation_ttl_ns,
            "expiry_target_ns": self.expiry_target_ns,
            "positive_apply": {
                "typed_trusted": self.trusted_typed_seen,
                "global_layer": "global" in self.positive_layers,
                "local_layer": "local" in self.positive_layers,
                "critic": self.positive_critic if self.arm == "M3" else None,
            },
            "post_expiry": {
                "global_zero_write": "global" in self.clear_layers,
                "local_zero_write": "local" in self.clear_layers,
                "critic_stale_rejected": (
                    self.critic_stale_rejected if self.arm == "M3" else None
                ),
                "applied_after_expiry": self.post_expiry_applied,
            },
            "action": {
                "goal_started": self.goal_started,
                "terminal_before_dropout": self.terminal_before_dropout,
                "terminal_reason": self.terminal_reason,
                "cancel_count": self.cancel_count,
                "terminal_zero_confirmed": self.terminal_zero_confirmed,
            },
            # These names are intentionally ready for the existing Phase-F
            # evaluator without conflating the nominal post-route drain.
            "evaluator_fields": {
                "nominal_episode_ttl_clear_applicability": (
                    "N/A_SEPARATE_ACTIVE_CONTROLLER_PROBE"
                ),
                "active_controller_ttl_probe": state,
                "ttl_source_sequence": self.armed_source_sequence,
                "ttl_expiry_stamp_ns": self.expiry_target_ns,
                "ttl_expiry_observed": self.clear_complete,
                "ttl_expiry_zero_write": (
                    {"global", "local"}.issubset(self.clear_layers)
                    and not self.post_expiry_applied
                ),
                "critic_stale_active_probe": (
                    "STALE_REJECTED"
                    if self.arm == "M3" and self.critic_stale_rejected
                    else "NOT_APPLICABLE" if self.arm == "M2" else "MISSING"
                ),
                "critic_post_expiry_applied": (
                    self.post_expiry_critic_applied if self.arm == "M3" else None
                ),
            },
            "events": list(self.events),
        }


class ProbeAdapter(Protocol):
    timeline: ActiveTtlTimeline

    def start_goal(self) -> None: ...
    def wait_for_armed(self, timeout_sec: float) -> bool: ...
    def stop_producer(self) -> None: ...
    def wait_for_clear(self, timeout_sec: float) -> bool: ...
    def cancel_goal_once(self, reason: str) -> None: ...
    def confirm_terminal_zero(self) -> bool: ...
    def episode_result(self) -> Mapping[str, Any]: ...


def execute_probe_lifecycle(
    adapter: ProbeAdapter,
    *,
    arming_timeout_sec: float,
    probe_timeout_sec: float,
) -> dict[str, Any]:
    """Execute the fixed adapter order; no retry or fallback is permitted."""

    adapter.start_goal()
    armed = adapter.wait_for_armed(arming_timeout_sec)
    timeout_reason = ""
    if armed and adapter.timeline.action_active:
        adapter.stop_producer()
        adapter.wait_for_clear(probe_timeout_sec)
    elif adapter.timeline.terminal_before_dropout:
        timeout_reason = adapter.timeline.terminal_reason
    else:
        timeout_reason = "positive_apply_not_observed_before_arming_timeout"
    if adapter.timeline.action_active:
        adapter.cancel_goal_once("active_ttl_probe_complete")
    adapter.timeline.mark_terminal_zero(adapter.confirm_terminal_zero())
    payload = adapter.timeline.result(timeout_reason=timeout_reason)
    payload["episode_result"] = dict(adapter.episode_result())
    return payload


def _active_runs(manifest: CausalManifest) -> tuple[RunContract, RunContract]:
    rows = tuple(run for run in manifest.runs if run.repeat == 1 and run.arm in ACTIVE_ARMS)
    if tuple(run.arm for run in rows) != ACTIVE_ARMS:
        raise CausalContractError("active TTL probe requires repeat-1 M2 then M3")
    return rows  # type: ignore[return-value]


def build_probe_plan(
    manifest: CausalManifest,
    output_root: str | Path,
) -> dict[str, Any]:
    root = Path(output_root).expanduser().resolve()
    templates = exact_adapter_templates(manifest)
    rows: list[dict[str, Any]] = []
    for run in _active_runs(manifest):
        values = _adapter_values(manifest, run, root)
        run_dir = Path(values["run_dir"])
        rows.append({
            "run_id": run.run_id,
            "arm": run.arm,
            "repeat": run.repeat,
            "run_directory": str(run_dir),
            "scene_id": manifest.identity["scene_id"],
            "start": dict(manifest.identity["start"]),
            "goal": dict(manifest.identity["goal"]),
            "graph_backend": "gvg",
            "localization_contract": "same_estimated_autonomy",
            "cognitive_profile": run.arm,
            "obstacle_layer_mode": "active",
            "critic_mode": "active" if run.arm == "M3" else "off",
            "direct_rgbd_costmap_enabled": False,
            "module1_amcl_prior_enabled": False,
            "cognitive_place_graph_enabled": False,
            "commands": {
                "scene": render_adapter_command(templates.scene, values),
                "stack": render_adapter_command(templates.stack, values),
                "producer_stop": render_adapter_command(templates.producer_stop or "", values),
            },
            "probe_json": str(run_dir / "active_ttl_probe.json"),
            "mcap": str(run_dir / "bag"),
            "module2_socket": values["module2_socket"],
        })
    return {
        "schema_version": SCHEMA_VERSION,
        "mode": "M2_M3_ACTIVE_CONTROLLER_TTL_PROBE",
        "nominal_episode_ttl": "N/A_SEPARATE_ACTIVE_CONTROLLER_PROBE",
        "ordered_steps": (
            "start_scene", "start_active_stack", "wait_readiness",
            "start_recorder", "start_G1_to_G2_goal", "wait_positive_apply",
            "stop_module2_and_bridge_only", "wait_sim_time_expiry",
            "require_zero_write_and_stale_rejection", "cancel_goal_once",
            "terminal_zero", "stop_recorder", "cleanup",
        ),
        "runs": rows,
    }


class _LiveProbeAdapter:
    """ROS adapter built around the existing exact reset/goal implementation."""

    def __init__(
        self,
        manifest: CausalManifest,
        run: RunContract,
        output_jsonl: Path,
        producer_stop: Callable[[], None],
        *,
        readiness_timeout_sec: float,
        reset_timeout_sec: float,
    ) -> None:
        from robot_experiments.v6_formal import (
            ENGINEERING_PILOT,
            Episode,
            Manifest,
            MissionLeg,
            V6FormalNode,
        )
        import rclpy
        from bio_nav_interfaces.msg import CognitiveObstacleArray, RiskLayerStatus
        from rosgraph_msgs.msg import Clock
        from rclpy.qos import QoSProfile, ReliabilityPolicy

        arm = manifest.arms[run.arm]
        runtime = {
            "canonical_odom": {"topic": "/odom", "owner": "isaac_compute_odometry", "tf": "odom->base_link"},
            "global_localization": {"pose_topic": "/amcl_pose", "owner": "amcl", "tf": "map->odom"},
            "module1_odom": {"topic": "/bio_nav/module1/odom", "owner": "wheel_imu_ekf", "publish_tf": False},
            "recovery_enabled": False,
            "module1_amcl_prior_enabled": False,
            "module2_navigation_write_enabled": True,
            "module2_active_effect_scope": "obstacle_only",
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
                key: str(manifest.identity[key])
                for key in (
                    "scene_asset", "occupancy_map", "spawn_manifest",
                    "route_graph", "obstacle_config", "obstacle_manifest",
                )
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
        self._rclpy = rclpy
        self.formal = V6FormalNode(
            formal_manifest, episode, output_jsonl, qualification=ENGINEERING_PILOT
        )
        self.manifest = manifest
        self.run = run
        self.timeline = ActiveTtlTimeline(
            run.arm,
            int(float(manifest.freshness["post_producer_stop_observation_margin_sec"]) * 1e9),
        )
        self._producer_stop = producer_stop
        self._readiness_timeout_sec = readiness_timeout_sec
        self._reset_timeout_sec = reset_timeout_sec
        self._episode_result: Mapping[str, Any] = {}
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        clock_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        def clock(message: Any) -> None:
            stamp = _time_ns(message.clock)
            if stamp is not None:
                self.timeline.observe_clock(stamp)

        def typed(message: Any) -> None:
            validation_stamp = _time_ns(message.validation_stamp)
            validation_ttl = _time_ns(message.validation_ttl)
            if validation_stamp is None or validation_ttl is None:
                return
            self.timeline.observe_typed(
                source_sequence=int(message.sequence),
                validation_stamp_ns=validation_stamp,
                validation_ttl_ns=validation_ttl,
                trusted_write=bool(message.trusted_write),
                healthy=bool(message.input_healthy and message.module2_healthy),
                observation_valid=bool(message.observation_valid),
                obstacle_count=len(message.obstacles),
            )

        def layer(message: Any) -> None:
            self.timeline.observe_layer(
                consumer=str(message.consumer),
                source_sequence=int(message.source_sequence),
                applied=bool(message.applied),
                raised_cell_count=int(message.raised_cell_count),
                active_cell_count=int(message.active_cell_count),
                maximum_cost_increase=int(message.maximum_cost_increase),
                reason=str(message.fallback_reason),
            )

        def critic(message: Any) -> None:
            self.timeline.observe_critic(
                source_sequence=int(message.source_sequence),
                applied=bool(message.applied),
                reason=str(message.fallback_reason),
            )

        self._subscriptions = [
            self.formal.node.create_subscription(Clock, "/clock", clock, clock_qos),
            self.formal.node.create_subscription(
                CognitiveObstacleArray,
                "/bio_nav/module2/cognitive_obstacles",
                typed,
                qos,
            ),
            self.formal.node.create_subscription(
                RiskLayerStatus,
                "/bio_nav/cognitive_obstacle_layer/status",
                layer,
                qos,
            ),
            self.formal.node.create_subscription(
                RiskLayerStatus,
                "/bio_nav/cognitive_risk_critic/status",
                critic,
                qos,
            ),
        ]

    def _spin_until(self, predicate: Callable[[], bool], timeout_sec: float) -> bool:
        return self.formal._spin_until(predicate, timeout_sec)

    def start_goal(self) -> None:
        from robot_experiments.v6_formal import parse_reset_receipt, ResetReceiptError

        formal = self.formal
        formal._assert_ground_truth_firewall()
        if not self._spin_until(formal._pre_reset_ready, self._readiness_timeout_sec):
            formal.guard.stop("readiness_timeout:" + (formal._readiness_blockers() or "unknown"))
            self.timeline.observe_terminal(formal.guard.stop_reason)
            return
        formal.guard.arm_reset(formal.facts)
        formal._set_episode_parameters(self._reset_timeout_sec)
        Trigger = formal._types["Trigger"]
        formal.guard.record_reset_call()
        future = formal.reset_client.call_async(Trigger.Request())
        if not self._spin_until(future.done, self._reset_timeout_sec):
            formal.guard.record_reset_response(None)
            self.timeline.observe_terminal(formal.guard.stop_reason or "reset_timeout")
            return
        response = future.result()
        formal.guard.record_reset_response(response.success if response is not None else None)
        if formal.guard.state == "STOP":
            self.timeline.observe_terminal(formal.guard.stop_reason)
            return
        try:
            formal.reset_receipt = parse_reset_receipt(
                response.message,
                requested_seed=formal.episode.seed,
                requested_case_id=formal.episode.dynamic_case_id,
                requested_variant_id=formal.episode.variant_id,
                requested_pose=formal.episode.reset_pose_name,
            )
        except ResetReceiptError as exc:
            formal.guard.stop(f"reset_receipt_mismatch:{exc}")
            self.timeline.observe_terminal(formal.guard.stop_reason)
            return
        formal.guard.record_reset_receipt_generation(int(formal.reset_receipt["generation"]))
        if not self._spin_until(
            lambda: formal.guard.localization_ready or formal.guard.state == "STOP",
            self._reset_timeout_sec,
        ):
            formal.guard.stop("post_reset_readiness_timeout")
        if formal.guard.state != "STOP":
            formal._check_post_reset_odom()
        if formal.guard.state != "STOP":
            formal._wait_nav2_and_tf_ready(self._reset_timeout_sec)
        if formal.guard.state != "STOP" and not self._spin_until(
            lambda: formal.guard.goal_ready or formal.guard.state == "STOP",
            self._reset_timeout_sec,
        ):
            formal.guard.stop("reset_gate_release_timeout")
        if formal.guard.state == "STOP":
            self.timeline.observe_terminal(formal.guard.stop_reason)
            return
        leg = formal.manifest.mission_legs[0]
        formal.guard.record_goal_publication(leg.goal_id)
        formal.route_goal_publisher.publish(formal._goal_message(leg))
        formal._write("active_ttl_probe_goal_published", leg_id=leg.goal_id)
        self.timeline.start_goal()

    def _sync_terminal(self) -> None:
        if self.timeline.action_active and self.formal.guard.state in {
            "SUCCEEDED", "FAILED", "STOP"
        }:
            self.timeline.observe_terminal(
                self.formal.guard.stop_reason or self.formal.guard.state,
                collision=self.formal.collision,
            )

    def wait_for_armed(self, timeout_sec: float) -> bool:
        ready = self._spin_until(
            lambda: self.timeline.armed
            or self.formal.guard.state in {"SUCCEEDED", "FAILED", "STOP"},
            timeout_sec,
        )
        self._sync_terminal()
        return bool(ready and self.timeline.armed)

    def stop_producer(self) -> None:
        if not self.timeline.action_active:
            self.timeline.observe_terminal("goal_not_active_at_dropout")
            return
        self._producer_stop()
        self.timeline.mark_producer_stopped()

    def wait_for_clear(self, timeout_sec: float) -> bool:
        ready = self._spin_until(
            lambda: self.timeline.clear_complete
            or self.timeline.post_expiry_applied
            or self.formal.guard.state in {"SUCCEEDED", "FAILED", "STOP"},
            timeout_sec,
        )
        self._sync_terminal()
        return bool(ready and self.timeline.clear_complete)

    def cancel_goal_once(self, reason: str) -> None:
        if self.timeline.cancel_count:
            raise CausalContractError("active TTL probe attempted duplicate goal cancellation")
        self.timeline.mark_cancel()
        self.formal.guard.stop(reason)
        self.formal._start_terminal_settle(cancel_navigation=True, reason=reason)

    def confirm_terminal_zero(self) -> bool:
        if self.formal.guard.state in {"SUCCEEDED", "FAILED", "STOP"}:
            self._episode_result = self.formal.result()
        else:
            self.formal.guard.stop("active_ttl_probe_incomplete")
            self._episode_result = self.formal.result()
        return bool(self._episode_result.get("terminal_zero_confirmed", False))

    def episode_result(self) -> Mapping[str, Any]:
        return self._episode_result

    def destroy(self) -> None:
        self.formal.destroy()


def dispatch_live_probe(
    manifest: CausalManifest,
    run: RunContract,
    output_jsonl: Path,
    producer_stop: Callable[[], None],
    *,
    readiness_timeout_sec: float,
    reset_timeout_sec: float,
    arming_timeout_sec: float,
    probe_timeout_sec: float,
) -> dict[str, Any]:
    import rclpy

    rclpy.init(args=None)
    adapter = _LiveProbeAdapter(
        manifest,
        run,
        output_jsonl,
        producer_stop,
        readiness_timeout_sec=readiness_timeout_sec,
        reset_timeout_sec=reset_timeout_sec,
    )
    try:
        return execute_probe_lifecycle(
            adapter,
            arming_timeout_sec=arming_timeout_sec,
            probe_timeout_sec=probe_timeout_sec,
        )
    finally:
        adapter.destroy()
        rclpy.shutdown()


def run_probe_campaign(
    manifest: CausalManifest,
    output_root: str | Path,
    *,
    arming_timeout_sec: float,
    probe_timeout_sec: float,
    shutdown_timeout_sec: float,
) -> dict[str, Any]:
    root = Path(output_root).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    plan = build_probe_plan(manifest, root)
    existing = [row["run_directory"] for row in plan["runs"] if Path(row["run_directory"]).exists()]
    if existing:
        raise CausalContractError("refusing to overwrite probe directories: " + ",".join(existing))
    env = dict(os.environ)
    domain = str(manifest.identity["ros_domain_id"])
    env["ROS_DOMAIN_ID"] = domain
    env["ISAAC_NAV_EXPECTED_DOMAIN_ID"] = domain
    env.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    results: list[dict[str, Any]] = []
    for row in plan["runs"]:
        run = next(item for item in _active_runs(manifest) if item.run_id == row["run_id"])
        run_dir = Path(row["run_directory"])
        run_dir.mkdir(parents=True)
        managed: list[_ManagedProcess] = []
        status: dict[str, Any] = {"run_id": run.run_id, "arm": run.arm, "state": "STARTED"}
        cleanup_failed = False
        try:
            managed.append(_start_process("scene", row["commands"]["scene"], run_dir / "scene.log", env=env))
            managed.append(_start_process("stack", row["commands"]["stack"], run_dir / "stack.log", env=env))
            startup = _wait_for_startup_ready(managed, float(manifest.identity["timeout_sec"]))
            status["startup"] = startup
            if startup.get("ready") is not True:
                raise CausalContractError(str(startup.get("reason", "startup not ready")))
            cognitive = _wait_for_cognitive_ready(
                manifest,
                run,
                managed,
                Path(row["module2_socket"]),
                float(manifest.identity["timeout_sec"]),
            )
            status["cognitive_readiness"] = cognitive
            if cognitive.get("ready") is not True:
                raise CausalContractError(str(cognitive.get("reason", "cognitive stack not ready")))
            managed.append(_start_process(
                "recorder", _rosbag_command(manifest, Path(row["mcap"])), run_dir / "recorder.log", env=env
            ))

            def stop_producer() -> None:
                with (run_dir / "producer_stop.log").open("w", encoding="utf-8") as stream:
                    completed = subprocess.run(
                        list(row["commands"]["producer_stop"]),
                        stdout=stream,
                        stderr=subprocess.STDOUT,
                        text=True,
                        check=False,
                        env=env,
                    )
                if completed.returncode != 0:
                    raise CausalContractError("Module2/Bridge producer stop failed")

            payload = dispatch_live_probe(
                manifest,
                run,
                run_dir / "active_ttl_probe_episode.jsonl",
                stop_producer,
                readiness_timeout_sec=float(manifest.identity["timeout_sec"]),
                reset_timeout_sec=float(manifest.identity["timeout_sec"]),
                arming_timeout_sec=arming_timeout_sec,
                probe_timeout_sec=probe_timeout_sec,
            )
            payload["probe_json"] = row["probe_json"]
            payload["mcap"] = row["mcap"]
            Path(row["probe_json"]).write_text(
                json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
            )
            status.update({
                "state": payload["state"],
                "probe_json": row["probe_json"],
                "mcap": row["mcap"],
            })
        except (OSError, CausalContractError, RuntimeError, ValueError) as exc:
            status["state"] = "ADAPTER_FAILED"
            status["reason"] = str(exc)
        finally:
            status["shutdown"] = []
            by_name = {item.name: item for item in managed}
            for name in ("stack", "recorder", "scene"):
                process = by_name.get(name)
                if process is not None:
                    try:
                        status["shutdown"].append(_stop_process(process, shutdown_timeout_sec))
                    except OSError as exc:
                        status["shutdown"].append({"name": name, "error": str(exc)})
            try:
                status["cleanup"] = _confirm_arm_cleanup(
                    run_dir,
                    Path(row["module2_socket"]),
                    status["shutdown"],
                    env,
                )
            except (OSError, CausalContractError, ValueError) as exc:
                status["cleanup"] = {"ok": False, "error": str(exc)}
            if status["cleanup"].get("ok") is not True:
                cleanup_failed = True
                status["state_before_cleanup"] = status["state"]
                status["state"] = "ARM_CLEANUP_FAILED"
        (run_dir / "active_ttl_probe_status.json").write_text(
            json.dumps(status, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        results.append(status)
        if cleanup_failed:
            break
    summary = {
        "schema_version": SCHEMA_VERSION,
        "state": "PASS" if len(results) == 2 and all(row["state"] == PASS_STATE for row in results) else "FAILED",
        "output_root": str(root),
        "runs": results,
    }
    (root / "active_ttl_probe_campaign.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return summary


def _write(payload: Mapping[str, Any], output: str | None) -> None:
    text = json.dumps(payload, indent=2, sort_keys=True) + "\n"
    if output:
        destination = Path(output).expanduser().resolve()
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(text, encoding="utf-8")
    else:
        print(text, end="")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    plan = subparsers.add_parser("plan")
    plan.add_argument("--config", required=True)
    plan.add_argument("--output-root", required=True)
    plan.add_argument("--output")
    run = subparsers.add_parser("run")
    run.add_argument("--config", required=True)
    run.add_argument("--output-root", required=True)
    run.add_argument("--arming-timeout-sec", type=float, default=DEFAULT_ARMING_TIMEOUT_SEC)
    run.add_argument("--probe-timeout-sec", type=float, default=DEFAULT_PROBE_TIMEOUT_SEC)
    run.add_argument("--shutdown-timeout-sec", type=float, default=DEFAULT_SHUTDOWN_TIMEOUT_SEC)
    run.add_argument("--output")
    return parser


def cli(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        manifest = load_manifest(args.config)
        if args.command == "plan":
            _write(build_probe_plan(manifest, args.output_root), args.output)
            return 0
        for value, name in (
            (args.arming_timeout_sec, "arming timeout"),
            (args.probe_timeout_sec, "probe timeout"),
            (args.shutdown_timeout_sec, "shutdown timeout"),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise CausalContractError(f"{name} must be finite and positive")
        result = run_probe_campaign(
            manifest,
            args.output_root,
            arming_timeout_sec=args.arming_timeout_sec,
            probe_timeout_sec=args.probe_timeout_sec,
            shutdown_timeout_sec=args.shutdown_timeout_sec,
        )
        _write(result, args.output)
        return 0 if result["state"] == "PASS" else 2
    except (OSError, yaml.YAMLError, CausalContractError) as exc:
        print(f"INVALID: {exc}", file=os.sys.stderr)
        return 2


def main() -> None:
    raise SystemExit(cli())


if __name__ == "__main__":
    main()
