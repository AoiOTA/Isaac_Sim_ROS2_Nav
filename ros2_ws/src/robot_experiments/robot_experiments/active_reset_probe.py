"""Exactly-once live probe for resetting an active route.

The state machine is ROS-independent so its fail-stop ordering can be tested
without a simulator.  The ROS adapter only translates topic/service events.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import tempfile
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

from .reset_receipt import ResetReceiptError, parse_reset_receipt


COMMAND_TOPICS = (
    "/cmd_vel_nav", "/cmd_vel_smoothed", "/cmd_vel", "/cmd_vel_sim"
)
TERMINAL_PHASES = {"PASS", "STOP"}


@dataclass(frozen=True)
class ProbeConfig:
    """Thresholds and immutable identities for one active-reset episode."""

    reset_seed: int = 8601
    reset_generation: int = 2
    reset_pose: str = "long_route_start_g1"
    reset_odometry: str = "realistic"
    active_timeout_s: float = 6.0
    reset_call_max_delay_s: float = 0.5
    reset_timeout_s: float = 30.0
    quiet_s: float = 1.0
    fresh_timeout_s: float = 30.0
    postzero_s: float = 1.0
    active_displacement_m: float = 0.10
    reset_stable_drift_m: float = 0.02
    fresh_goal_xy: tuple[float, float] = (0.685, -3.975535)
    fresh_goal_error_m: float = 0.30
    fresh_edges: tuple[int, ...] = (51, 52)
    allow_fresh_support_equivalent: bool = False


@dataclass
class ProbeMachine:
    """ROS-independent exactly-once active-reset state machine."""

    config: ProbeConfig = field(default_factory=ProbeConfig)
    phase: str = "WAIT_ENDPOINTS"
    stop_reason: str | None = None
    timestamps: dict[str, float] = field(default_factory=dict)
    counts: dict[str, int] = field(default_factory=lambda: {
        "old_publish": 0, "fresh_publish": 0, "reset_call": 0,
        "canonical": 0, "progress": 0, "lookahead": 0,
        "goal_update": 0, "old_bool": 0, "old_result": 0,
        "fresh_bool": 0, "fresh_result": 0,
        "hold_cmd_vel_sim_nonzero": 0,
    })
    endpoint_detail: dict[str, Any] = field(default_factory=dict)
    old_request_id: int | None = None
    fresh_request_id: int | None = None
    old_edges: list[int] = field(default_factory=list)
    fresh_edges_observed: list[int] = field(default_factory=list)
    fresh_edges_equivalence: str | None = None
    old_progress_seen: bool = False
    old_lookahead_seen: bool = False
    old_goal_update_seen: bool = False
    old_cmd_nonzero: int = 0
    old_start_xy: tuple[float, float] | None = None
    latest_gt_xy: tuple[float, float] | None = None
    old_displacement_m: float = 0.0
    collision: bool = False
    reset_receipt: dict[str, Any] | None = None
    baseline_gate_released: bool = False
    gate_sequence: list[dict[str, Any]] = field(default_factory=list)
    reset_event_seen: bool = False
    reset_landing_xy: tuple[float, float] | None = None
    reset_stable_max_drift_m: float = 0.0
    old_bool_value: bool | None = None
    old_result_value: dict[str, Any] | None = None
    fresh_bool_value: bool | None = None
    fresh_result_value: dict[str, Any] | None = None
    old_activity_after_terminal: int = 0
    post_commands: dict[str, dict[str, Any]] = field(default_factory=lambda: {
        topic: {"total": 0, "nonzero": 0, "last_zero": False}
        for topic in COMMAND_TOPICS
    })

    def _time(self, name: str, now: float) -> None:
        self.timestamps.setdefault(name, float(now))

    def stop(self, reason: str, now: float) -> None:
        """Enter terminal STOP once while retaining the first reason."""
        if self.phase not in TERMINAL_PHASES:
            self.stop_reason = reason
            self._time("stop", now)
            self.phase = "STOP"

    def endpoints_ready(self, now: float, detail: dict[str, Any]) -> None:
        """Accept a complete endpoint snapshot and arm old publication."""
        if self.phase != "WAIT_ENDPOINTS":
            return
        self.endpoint_detail = dict(detail)
        if not all(bool(value) for value in detail.values()):
            return
        if self.latest_gt_xy is None:
            return
        self.phase = "PUBLISH_OLD_ONCE"
        self._time("endpoints_ready", now)

    def old_published(self, now: float) -> None:
        """Record the sole old-goal publication."""
        if self.phase != "PUBLISH_OLD_ONCE" or self.counts["old_publish"]:
            self.stop("old_goal_publish_not_exactly_once", now)
            return
        self.counts["old_publish"] = 1
        self.old_start_xy = self.latest_gt_xy
        self._time("old_published", now)
        self.phase = "WAIT_ACTIVE_READY"

    def fresh_published(self, now: float) -> None:
        """Record the sole fresh-goal publication."""
        if self.phase != "PUBLISH_FRESH_ONCE" or self.counts["fresh_publish"]:
            self.stop("fresh_goal_publish_not_exactly_once", now)
            return
        self.counts["fresh_publish"] = 1
        self._time("fresh_published", now)
        self.phase = "WAIT_SUCCESS"

    def canonical(self, request_id: int, edge_ids: list[int], now: float) -> None:
        """Consume a canonical-route observation."""
        self.counts["canonical"] += 1
        if self.counts["old_publish"] == 0:
            return
        if self._quiet_old_outputs(now):
            return
        if self.counts["fresh_publish"] == 0:
            if self.old_request_id is None:
                if not edge_ids:
                    self.stop("old_canonical_empty", now)
                    return
                self.old_request_id = int(request_id)
                self.old_edges = list(edge_ids)
            elif int(request_id) != self.old_request_id:
                self.stop("old_request_id_changed_before_reset", now)
        else:
            if self.fresh_request_id is None:
                self.fresh_request_id = int(request_id)
                self.fresh_edges_observed = list(edge_ids)
                if tuple(edge_ids) == self.config.fresh_edges:
                    self.fresh_edges_equivalence = "canonical_exact"
                elif self.config.allow_fresh_support_equivalent and edge_ids:
                    self.fresh_edges_equivalence = "support_equivalent_allowed"
                else:
                    self.stop("fresh_canonical_edges_mismatch", now)
            elif int(request_id) != self.fresh_request_id:
                self.stop("fresh_request_id_changed", now)
        self._maybe_active_ready(now)

    def progress(self, request_id: int, now: float) -> None:
        """Consume a route-progress observation."""
        self.counts["progress"] += 1
        if self.counts["old_publish"] == 0:
            return
        if self._quiet_old_outputs(now):
            return
        if self.counts["fresh_publish"] == 0:
            if self.old_request_id is not None and int(request_id) == self.old_request_id:
                self.old_progress_seen = True
        elif (
            int(request_id) == self.old_request_id
            or (
                self.fresh_request_id is not None
                and int(request_id) != self.fresh_request_id
            )
        ):
            self.stop("fresh_progress_request_id_mismatch", now)
        self._maybe_active_ready(now)

    def route_output(self, kind: str, now: float) -> None:
        """Consume lookahead or goal-update output."""
        if kind not in {"lookahead", "goal_update"}:
            self.stop("unknown_route_output", now)
            return
        self.counts[kind] += 1
        if self.counts["old_publish"] == 0:
            return
        if self._quiet_old_outputs(now):
            return
        if self.counts["fresh_publish"] == 0:
            if kind == "lookahead":
                self.old_lookahead_seen = True
            else:
                self.old_goal_update_seen = True
        self._maybe_active_ready(now)

    def command(self, topic: str, nonzero: bool, now: float) -> None:
        """Consume one command-chain sample."""
        if topic not in COMMAND_TOPICS:
            self.stop("unknown_command_topic", now)
            return
        if topic == "/cmd_vel_sim" and self.phase == "WAIT_ACTIVE_READY" and nonzero:
            self.old_cmd_nonzero += 1
            self._maybe_active_ready(now)
        hold_seen = any(item["reason"] == "hold" for item in self.gate_sequence)
        released = any(item["reason"].startswith("released:") for item in self.gate_sequence)
        if topic == "/cmd_vel_sim" and hold_seen and not released and nonzero:
            self.counts["hold_cmd_vel_sim_nonzero"] += 1
            self.stop("hold_cmd_vel_sim_nonzero", now)
        if self.phase == "POSTZERO":
            row = self.post_commands[topic]
            row["total"] += 1
            row["nonzero"] += int(nonzero)
            row["last_zero"] = not nonzero

    def ground_truth(self, x: float, y: float, now: float) -> None:
        """Consume one evaluator-only ground-truth position."""
        xy = (float(x), float(y))
        self.latest_gt_xy = xy
        if self.old_start_xy is not None and self.counts["reset_call"] == 0:
            self.old_displacement_m = max(
                self.old_displacement_m, math.dist(self.old_start_xy, xy)
            )
            self._maybe_active_ready(now)
        reset_complete_seen = any(
            item["reason"] == "reset_complete" for item in self.gate_sequence
        )
        if (
            self.reset_event_seen
            and reset_complete_seen
            and self.counts["fresh_publish"] == 0
        ):
            if self.reset_landing_xy is None:
                self.reset_landing_xy = xy
            else:
                self.reset_stable_max_drift_m = max(
                    self.reset_stable_max_drift_m,
                    math.dist(self.reset_landing_xy, xy),
                )
                if (
                    self.phase == "QUIET"
                    and self.reset_stable_max_drift_m
                    > self.config.reset_stable_drift_m
                ):
                    self.stop("reset_landing_drift_exceeded", now)
            self._advance_reset_observation(now)

    def collision_event(self, collided: bool, now: float) -> None:
        """Latch physical collision and fail immediately when true."""
        self.collision = self.collision or bool(collided)
        if collided:
            self.stop("physical_collision", now)

    def _maybe_active_ready(self, now: float) -> None:
        if self.phase != "WAIT_ACTIVE_READY":
            return
        ready = (
            self.old_request_id is not None
            and self.old_progress_seen
            and self.old_lookahead_seen
            and self.old_goal_update_seen
            and self.old_cmd_nonzero >= 5
            and self.old_displacement_m >= self.config.active_displacement_m
            and not self.collision
            and self.old_bool_value is None
            and self.old_result_value is None
        )
        if ready:
            self._time("active_ready", now)
            self.phase = "CALL_RESET_ONCE"

    def reset_call_started(self, now: float) -> None:
        """Record the exactly-once Trigger dispatch boundary."""
        if self.phase != "CALL_RESET_ONCE" or self.counts["reset_call"]:
            self.stop("reset_call_not_exactly_once", now)
            return
        active = self.timestamps.get("active_ready")
        if active is None or now - active > self.config.reset_call_max_delay_s:
            self.stop("reset_call_after_active_ready_too_late", now)
            return
        self.counts["reset_call"] = 1
        self._time("reset_call", now)
        self.phase = "OBSERVE_HOLD_ABORT"

    def reset_response(self, success: bool, message: str, now: float) -> None:
        """Validate the Trigger result and exact reset receipt."""
        if self.counts["reset_call"] != 1 or self.reset_receipt is not None:
            self.stop("unexpected_reset_response", now)
            return
        if not success:
            self.stop("reset_service_failed", now)
            return
        try:
            receipt = parse_reset_receipt(
                message, requested_seed=self.config.reset_seed
            )
        except ResetReceiptError as exc:
            self.stop(f"reset_receipt_invalid:{exc}", now)
            return
        if (
            receipt["generation"] != self.config.reset_generation
            or receipt["pose"] != self.config.reset_pose
            or receipt["odometry"] != self.config.reset_odometry
        ):
            self.stop("reset_receipt_contract_mismatch", now)
            return
        self.reset_receipt = receipt
        self._time("reset_receipt", now)
        self._advance_reset_observation(now)

    def gate_status(self, payload: str, now: float) -> None:
        """Consume and order one ResetStopGate status document."""
        try:
            item = json.loads(payload)
        except (TypeError, json.JSONDecodeError):
            self.stop("gate_status_malformed", now)
            return
        if not isinstance(item, dict) or set(item) != {
            "generation", "held", "eligible_generation", "reason"
        }:
            self.stop("gate_status_contract_mismatch", now)
            return
        generation = item["generation"]
        held = item["held"]
        eligible = item["eligible_generation"]
        reason = item["reason"]
        if (
            isinstance(generation, bool)
            or not isinstance(generation, int)
            or not isinstance(held, bool)
            or not isinstance(reason, str)
        ):
            self.stop("gate_status_field_invalid", now)
            return
        # Ignore the retained generation-1 released baseline.
        if generation != self.config.reset_generation:
            if (
                self.counts["reset_call"] == 0
                and generation == self.config.reset_generation - 1
                and not held
                and eligible is None
                and reason.startswith("released:")
            ):
                self.baseline_gate_released = True
            if self.counts["reset_call"] and generation > self.config.reset_generation:
                self.stop("gate_generation_advanced_unexpectedly", now)
            return
        if self.counts["reset_call"] == 0:
            self.stop("new_gate_generation_before_reset_call", now)
            return
        valid = (
            (reason == "hold" and held and eligible is None)
            or (reason == "reset_complete" and held and eligible == generation)
            or (reason.startswith("released:") and not held and eligible is None)
        )
        if not valid:
            self.stop("gate_status_incoherent", now)
            return
        if not self.gate_sequence or item != {
            key: self.gate_sequence[-1][key] for key in item
        }:
            self.gate_sequence.append({**item, "monotonic": float(now)})
        reasons = [row["reason"] for row in self.gate_sequence]
        expected_order = {"hold": 0, "reset_complete": 1}
        rank = 2 if reason.startswith("released:") else expected_order[reason]
        previous_rank = -1
        if len(reasons) > 1:
            prior = reasons[-2]
            previous_rank = 2 if prior.startswith("released:") else expected_order[prior]
        if rank < previous_rank or (rank > previous_rank + 1 and previous_rank >= 0):
            self.stop("gate_status_order_invalid", now)
            return
        self._advance_reset_observation(now)

    def reset_event(self, now: float) -> None:
        """Consume the one reset event belonging to this Trigger call."""
        # The official launch performs one startup reset before this probe is
        # armed.  Only the event after our exactly-once service call belongs
        # to the contract under test.
        if self.counts["reset_call"] == 0:
            return
        if self.reset_event_seen:
            self.stop("duplicate_reset_event", now)
            return
        self.reset_event_seen = True
        self._time("reset_event", now)
        self._advance_reset_observation(now)

    def terminal_bool(self, value: bool, now: float) -> None:
        """Consume one old or fresh Bool terminal."""
        if self.counts["old_publish"] == 0:
            return
        fresh = self.counts["fresh_publish"] == 1
        name = "fresh" if fresh else "old"
        key = f"{name}_bool"
        self.counts[key] += 1
        if self.counts[key] != 1:
            self.stop(f"duplicate_{name}_terminal_bool", now)
            return
        if fresh:
            self.fresh_bool_value = bool(value)
            if not value:
                self.stop("fresh_terminal_false", now)
                return
        else:
            if self.counts["reset_call"] == 0:
                self.stop("old_terminal_before_reset", now)
                return
            self.old_bool_value = bool(value)
            if value:
                self.stop("old_terminal_true", now)
                return
            self._time("old_terminal_bool", now)
        self._advance_reset_observation(now)
        self._maybe_fresh_success(now)

    def terminal_result(self, payload: str, now: float) -> None:
        """Consume one old or fresh structured result terminal."""
        if self.counts["old_publish"] == 0:
            return
        try:
            item = json.loads(payload)
        except (TypeError, json.JSONDecodeError):
            self.stop("terminal_result_malformed", now)
            return
        fresh = self.counts["fresh_publish"] == 1
        name = "fresh" if fresh else "old"
        key = f"{name}_result"
        self.counts[key] += 1
        if self.counts[key] != 1 or not isinstance(item, dict):
            self.stop(f"duplicate_or_invalid_{name}_result", now)
            return
        if fresh:
            self.fresh_result_value = item
            if (
                self.fresh_request_id is None
                or item.get("request_id") != self.fresh_request_id
                or item.get("status") != "succeeded"
            ):
                self.stop("fresh_result_contract_mismatch", now)
                return
        else:
            if self.counts["reset_call"] == 0:
                self.stop("old_terminal_before_reset", now)
                return
            self.old_result_value = item
            if (
                self.old_request_id is None
                or item.get("request_id") != self.old_request_id
                or item.get("status") != "aborted"
                or item.get("reason") != "simulation_reset"
            ):
                self.stop("old_result_contract_mismatch", now)
                return
            self._time("old_terminal_result", now)
        self._advance_reset_observation(now)
        self._maybe_fresh_success(now)

    def _old_pair_complete(self) -> bool:
        return self.old_bool_value is False and self.old_result_value is not None

    def _quiet_old_outputs(self, now: float) -> bool:
        if (
            self._old_pair_complete()
            and self.counts["fresh_publish"] == 0
            and self.phase in {"OBSERVE_HOLD_ABORT", "WAIT_RELEASE", "QUIET", "PUBLISH_FRESH_ONCE"}
        ):
            self.old_activity_after_terminal += 1
            self.stop("old_route_output_not_quiet", now)
            return True
        return False

    def _advance_reset_observation(self, now: float) -> None:
        if (
            self.phase in TERMINAL_PHASES
            or self.counts["reset_call"] == 0
            or self.counts["fresh_publish"] != 0
        ):
            return
        reasons = [row["reason"] for row in self.gate_sequence]
        hold = "hold" in reasons
        complete = "reset_complete" in reasons
        released = any(reason.startswith("released:") for reason in reasons)
        if (
            self.reset_receipt is not None
            and hold
            and complete
            and self._old_pair_complete()
            and self.reset_event_seen
        ):
            self.phase = "WAIT_RELEASE"
            if released:
                if self.reset_landing_xy is None:
                    return
                if self.reset_stable_max_drift_m > self.config.reset_stable_drift_m:
                    self.stop("reset_landing_drift_exceeded", now)
                    return
                anchor = max(
                    self.timestamps["old_terminal_bool"],
                    self.timestamps["old_terminal_result"],
                )
                self.timestamps["quiet_started"] = anchor
                self._time("gate_released", now)
                self.phase = "QUIET"

    def _maybe_fresh_success(self, now: float) -> None:
        if self.phase != "WAIT_SUCCESS":
            return
        if self.fresh_bool_value is True and self.fresh_result_value is not None:
            if self.latest_gt_xy is None:
                self.stop("fresh_success_without_ground_truth", now)
                return
            error = math.dist(self.latest_gt_xy, self.config.fresh_goal_xy)
            if error > self.config.fresh_goal_error_m:
                self.stop("fresh_goal_error_exceeded", now)
                return
            self.timestamps["fresh_success"] = float(now)
            self.timestamps["postzero_started"] = float(now)
            self.phase = "POSTZERO"

    def tick(self, now: float) -> None:
        """Advance monotonic deadlines and quiet/postzero dwell states."""
        if self.phase == "WAIT_ACTIVE_READY":
            if now - self.timestamps["old_published"] > self.config.active_timeout_s:
                self.stop("active_ready_timeout", now)
        elif self.phase in {"OBSERVE_HOLD_ABORT", "WAIT_RELEASE"}:
            if now - self.timestamps["reset_call"] > self.config.reset_timeout_s:
                self.stop("reset_observation_timeout", now)
        elif self.phase == "QUIET":
            if now - self.timestamps["quiet_started"] >= self.config.quiet_s:
                self.phase = "PUBLISH_FRESH_ONCE"
        elif self.phase == "WAIT_SUCCESS":
            if now - self.timestamps["fresh_published"] > self.config.fresh_timeout_s:
                self.stop("fresh_success_timeout", now)
        elif self.phase == "POSTZERO":
            if now - self.timestamps["postzero_started"] >= self.config.postzero_s:
                invalid = [
                    topic for topic, row in self.post_commands.items()
                    if row["total"] == 0 or row["nonzero"] != 0 or not row["last_zero"]
                ]
                if invalid:
                    self.stop("postzero_contract_failed:" + ",".join(invalid), now)
                elif self.collision:
                    self.stop("physical_collision", now)
                else:
                    self.timestamps["pass"] = float(now)
                    self.phase = "PASS"

    def document(self) -> dict[str, Any]:
        """Return the complete machine-readable evidence snapshot."""
        return {
            "schema": "bio_nav.active_reset_probe.v1",
            "phase": self.phase,
            "verdict": "PASS" if self.phase == "PASS" else (
                "STOP" if self.phase == "STOP" else "RUNNING"
            ),
            "stop_reason": self.stop_reason,
            "config": asdict(self.config),
            "timestamps_monotonic_s": dict(self.timestamps),
            "active_ready_to_reset_call_s": (
                None
                if (
                    "active_ready" not in self.timestamps
                    or "reset_call" not in self.timestamps
                )
                else self.timestamps["reset_call"] - self.timestamps["active_ready"]
            ),
            "counts": dict(self.counts),
            "endpoints": dict(self.endpoint_detail),
            "old": {
                "request_id": self.old_request_id,
                "edges": self.old_edges,
                "displacement_m": self.old_displacement_m,
                "cmd_vel_sim_nonzero": self.old_cmd_nonzero,
                "terminal_bool": self.old_bool_value,
                "result": self.old_result_value,
                "activity_after_terminal": self.old_activity_after_terminal,
            },
            "reset": {
                "receipt": self.reset_receipt,
                "gate_sequence": self.gate_sequence,
                "event_seen": self.reset_event_seen,
                "landing_xy": self.reset_landing_xy,
                "stable_max_drift_m": self.reset_stable_max_drift_m,
            },
            "fresh": {
                "request_id": self.fresh_request_id,
                "edges": self.fresh_edges_observed,
                "edge_equivalence": self.fresh_edges_equivalence,
                "terminal_bool": self.fresh_bool_value,
                "result": self.fresh_result_value,
                "latest_gt_xy": self.latest_gt_xy,
                "goal_error_m": None if self.latest_gt_xy is None else math.dist(
                    self.latest_gt_xy, self.config.fresh_goal_xy
                ),
            },
            "collision": self.collision,
            "postzero": self.post_commands,
        }


def atomic_write_json(path: Path, document: dict[str, Any]) -> None:
    """Durably replace one JSON snapshot without exposing partial content."""
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(document, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _parse_edges(value: str) -> tuple[int, ...]:
    edges = tuple(int(item) for item in value.split(",") if item.strip())
    if not edges:
        raise argparse.ArgumentTypeError("edge list must not be empty")
    return edges


def _arguments(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--old-goal", nargs=3, type=float, default=(0.8, 4.8, -2.792526803))
    parser.add_argument(
        "--fresh-goal",
        nargs=3,
        type=float,
        default=(0.685, -3.975535, 1.570796327),
    )
    parser.add_argument("--expected-route-subscribers", type=int, default=2)
    parser.add_argument("--reset-seed", type=int, default=8601)
    parser.add_argument("--fresh-edges", type=_parse_edges, default=(51, 52))
    parser.add_argument("--allow-fresh-support-equivalent", action="store_true")
    parser.add_argument("--endpoint-timeout", type=float, default=30.0)
    args = parser.parse_args(argv)
    if args.expected_route_subscribers < 1:
        parser.error("--expected-route-subscribers must be positive")
    if args.reset_seed < 0:
        parser.error("--reset-seed must be non-negative")
    if args.endpoint_timeout <= 0.0:
        parser.error("--endpoint-timeout must be positive")
    if not all(math.isfinite(value) for value in (*args.old_goal, *args.fresh_goal)):
        parser.error("goal coordinates must be finite")
    return args


def _run_ros(args: argparse.Namespace) -> int:
    import rclpy
    from bio_nav_interfaces.msg import CanonicalRoute, RouteProgress
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, Empty, String
    from std_srvs.srv import Trigger

    reliable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    latched = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    best_effort = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)

    class ActiveResetProbeNode(Node):
        def __init__(self) -> None:
            super().__init__(
                "v6_active_reset_probe",
                parameter_overrides=[Parameter("use_sim_time", value=True)],
            )
            config = ProbeConfig(
                reset_seed=args.reset_seed,
                fresh_goal_xy=(args.fresh_goal[0], args.fresh_goal[1]),
                fresh_edges=tuple(args.fresh_edges),
                allow_fresh_support_equivalent=args.allow_fresh_support_equivalent,
            )
            self.machine = ProbeMachine(config)
            self.started = time.monotonic()
            self._old_goal = tuple(args.old_goal)
            self._fresh_goal = tuple(args.fresh_goal)
            self._expected_subscribers = args.expected_route_subscribers
            self._endpoint_timeout = args.endpoint_timeout
            self._reset_future = None
            self._last_persist = -math.inf
            self._goal_pub = self.create_publisher(PoseStamped, "/bio_nav/route_goal", reliable)
            self._reset_client = self.create_client(Trigger, "/simulation/reset")
            self.create_subscription(
                CanonicalRoute,
                "/bio_nav/canonical_route",
                lambda m: self._event(
                    self.machine.canonical,
                    int(m.request_id),
                    [int(x) for x in m.edge_ids],
                ),
                latched,
            )
            self.create_subscription(
                RouteProgress,
                "/bio_nav/route_progress",
                lambda m: self._event(self.machine.progress, int(m.request_id)),
                reliable,
            )
            self.create_subscription(
                PoseStamped,
                "/bio_nav/route_lookahead_goal",
                lambda _m: self._event(self.machine.route_output, "lookahead"),
                reliable,
            )
            self.create_subscription(
                PoseStamped,
                "/goal_update",
                lambda _m: self._event(self.machine.route_output, "goal_update"),
                reliable,
            )
            self.create_subscription(
                Bool,
                "/bio_nav/route_goal_complete",
                lambda m: self._event(self.machine.terminal_bool, bool(m.data)),
                reliable,
            )
            self.create_subscription(
                String,
                "/bio_nav/route_goal_result",
                lambda m: self._event(self.machine.terminal_result, str(m.data)),
                reliable,
            )
            self.create_subscription(
                Odometry,
                "/ground_truth/odom",
                lambda m: self._event(
                    self.machine.ground_truth,
                    float(m.pose.pose.position.x),
                    float(m.pose.pose.position.y),
                ),
                best_effort,
            )
            self.create_subscription(
                Bool,
                "/simulation/collision",
                lambda m: self._event(
                    self.machine.collision_event, bool(m.data)
                ),
                best_effort,
            )
            self.create_subscription(
                String,
                "/simulation/reset_stop_gate/status",
                lambda m: self._event(self.machine.gate_status, str(m.data)),
                latched,
            )
            self.create_subscription(
                Empty,
                "/simulation/reset_event",
                lambda _m: self._event(self.machine.reset_event),
                reliable,
            )
            for topic in COMMAND_TOPICS:
                self.create_subscription(
                    Twist,
                    topic,
                    lambda m, t=topic: self._event(
                        self.machine.command, t, _twist_nonzero(m)
                    ),
                    best_effort,
                )
            self.create_timer(0.02, self._tick)
            self._persist(force=True)

        def _event(self, callback, *values) -> None:
            now = time.monotonic()
            previous = self.machine.phase
            callback(*values, now)
            self._drive(now)
            self._persist(force=self.machine.phase != previous)

        def _persist(self, *, force: bool = False) -> None:
            now = time.monotonic()
            # Topic callbacks can exceed 200 Hz.  A bounded 20 Hz atomic
            # stream is continuously observable without making NAS fsync the
            # latency-critical active-ready -> Trigger path.
            if not force and now - self._last_persist < 0.05:
                return
            atomic_write_json(args.output, self.machine.document())
            self._last_persist = now

        def _endpoints(self) -> dict[str, Any]:
            topics = (
                "/bio_nav/canonical_route", "/bio_nav/route_progress",
                "/bio_nav/route_lookahead_goal", "/goal_update",
                "/bio_nav/route_goal_complete", "/bio_nav/route_goal_result",
                "/ground_truth/odom", "/simulation/collision",
                "/simulation/reset_stop_gate/status", "/simulation/reset_event",
                *COMMAND_TOPICS,
            )
            return {
                "route_goal_subscribers": (
                    self._goal_pub.get_subscription_count()
                    >= self._expected_subscribers
                ),
                "reset_service": self._reset_client.service_is_ready(),
                "ground_truth_sample": self.machine.latest_gt_xy is not None,
                "gate_released_baseline": self.machine.baseline_gate_released,
                **{f"publisher:{topic}": self.count_publishers(topic) >= 1 for topic in topics},
            }

        def _publish_goal(self, values: tuple[float, float, float]) -> None:
            message = PoseStamped()
            message.header.frame_id = "map"
            message.header.stamp = self.get_clock().now().to_msg()
            message.pose.position.x = values[0]
            message.pose.position.y = values[1]
            message.pose.orientation.z = math.sin(values[2] * 0.5)
            message.pose.orientation.w = math.cos(values[2] * 0.5)
            self._goal_pub.publish(message)

        def _tick(self) -> None:
            now = time.monotonic()
            previous = self.machine.phase
            if self.machine.phase == "WAIT_ENDPOINTS":
                self.machine.endpoints_ready(now, self._endpoints())
                if now - self.started > self._endpoint_timeout:
                    self.machine.stop("endpoint_timeout", now)
            self.machine.tick(now)
            self._drive(now)
            self._persist(force=self.machine.phase != previous)

        def _drive(self, now: float) -> None:
            """Execute state-owned exactly-once side effects immediately."""
            if self.machine.phase == "PUBLISH_OLD_ONCE":
                self._publish_goal(self._old_goal)
                self.machine.old_published(now)
            elif self.machine.phase == "CALL_RESET_ONCE":
                self.machine.reset_call_started(now)
                if self.machine.phase != "STOP":
                    self._reset_future = self._reset_client.call_async(Trigger.Request())
                    self._reset_future.add_done_callback(self._reset_done)
            elif self.machine.phase == "PUBLISH_FRESH_ONCE":
                self._publish_goal(self._fresh_goal)
                self.machine.fresh_published(now)

        def _reset_done(self, future) -> None:
            now = time.monotonic()
            try:
                response = future.result()
            except Exception as exc:
                self.machine.stop(f"reset_service_exception:{type(exc).__name__}:{exc}", now)
            else:
                if response is None:
                    self.machine.stop("reset_service_no_response", now)
                else:
                    self.machine.reset_response(bool(response.success), str(response.message), now)
            self._persist(force=True)

    def _twist_nonzero(message) -> bool:
        values = (
            message.linear.x, message.linear.y, message.linear.z,
            message.angular.x, message.angular.y, message.angular.z,
        )
        return any(abs(float(value)) > 1.0e-6 for value in values)

    rclpy.init()
    node = ActiveResetProbeNode()
    try:
        while rclpy.ok() and node.machine.phase not in TERMINAL_PHASES:
            rclpy.spin_once(node, timeout_sec=0.1)
        node._persist(force=True)
        print(json.dumps(node.machine.document(), sort_keys=True))
        return 0 if node.machine.phase == "PASS" else 20
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(argv: list[str] | None = None) -> int:
    """Run one active-reset probe and return zero only on PASS."""
    return _run_ros(_arguments(argv))


if __name__ == "__main__":
    raise SystemExit(main())
