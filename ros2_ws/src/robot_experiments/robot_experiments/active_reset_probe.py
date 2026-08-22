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
from typing import Any, Callable

from .reset_receipt import ResetReceiptError, parse_reset_receipt


COMMAND_TOPICS = (
    "/cmd_vel_nav", "/cmd_vel_smoothed", "/cmd_vel", "/cmd_vel_sim"
)
RUN_EXIT_PHASES = {"PROVISIONAL_COMPLETE", "STOP"}
TOPOLOGY_CHECKPOINTS = ("prepublish", "pre_reset", "post_release", "pre_fresh")
DEFAULT_ROUTE_SUBSCRIBERS = (
    "/bio_nav_route_coordinator",
    "/rosbag2_recorder",
)
MAX_COVERAGE_GAP_S = 0.25
PROVISIONAL_MONITOR_S = 0.25
GROUND_TRUTH_TOPIC = "/ground_truth/odom"
ODOMETRY_TOPIC = "/odom"


def _normalized_frame(value: str) -> str:
    """Normalize the one explicitly tolerated leading slash."""
    frame = str(value)
    return frame[1:] if frame.startswith("/") else frame


def _all_finite(values: tuple[float, ...] | list[float]) -> bool:
    try:
        return all(math.isfinite(float(value)) for value in values)
    except (TypeError, ValueError):
        return False


def _twist_values(message: Any) -> tuple[float, ...]:
    values = (
        float(message.linear.x), float(message.linear.y),
        float(message.linear.z), float(message.angular.x),
        float(message.angular.y), float(message.angular.z),
    )
    if not _all_finite(values):
        raise ValueError("twist_nonfinite")
    return values


def _node_full_name(node_namespace: str, node_name: str) -> str:
    namespace = str(node_namespace or "/").rstrip("/")
    return f"{namespace}/{node_name}" if namespace else f"/{node_name}"


def _gid_hex(endpoint_gid: Any) -> str:
    try:
        return bytes(endpoint_gid).hex()
    except (TypeError, ValueError):
        return str(endpoint_gid)


def _endpoint_document(info: Any) -> dict[str, str]:
    return {
        "node": _node_full_name(info.node_namespace, info.node_name),
        "node_name": str(info.node_name),
        "node_namespace": str(info.node_namespace),
        "gid": _gid_hex(info.endpoint_gid),
    }


def validate_topology_snapshot(
    snapshot: dict[str, Any], expected_route_subscribers: tuple[str, ...]
) -> list[str]:
    """Return exact identity/count violations for the safety-critical graph."""
    requirements = (
        ("/cmd_vel", "publishers", ("/collision_monitor",)),
        ("/cmd_vel_sim", "publishers", ("/isaac_navigation_sim",)),
        ("/bio_nav/route_goal", "publishers", ("/v6_active_reset_probe",)),
        (
            "/bio_nav/route_goal",
            "subscriptions",
            tuple(sorted(expected_route_subscribers)),
        ),
    )
    errors: list[str] = []
    topics = snapshot.get("topics", {})
    for topic, direction, expected in requirements:
        observed = tuple(sorted(
            row.get("node", "") for row in topics.get(topic, {}).get(direction, [])
        ))
        if observed != tuple(sorted(expected)):
            errors.append(
                f"{topic}:{direction}:expected={sorted(expected)}:"
                f"observed={list(observed)}"
            )
    return errors


def _coverage_summary(
    samples: list[dict[str, Any]], start: float | None, end: float | None
) -> dict[str, Any]:
    """Summarize bounded callback coverage without claiming source order."""
    ordered = sorted(samples, key=lambda row: float(row["monotonic_s"]))
    times = [float(row["monotonic_s"]) for row in ordered]
    gaps = [later - earlier for earlier, later in zip(times, times[1:])]
    return {
        "count": len(ordered),
        "first_monotonic_s": times[0] if times else None,
        "last_monotonic_s": times[-1] if times else None,
        "max_gap_s": max(gaps, default=None),
        "start_edge_delay_s": None if not times or start is None else times[0] - start,
        "end_edge_lead_s": None if not times or end is None else end - times[-1],
        "samples": ordered,
    }


def _position_coverage_summary(
    samples: list[dict[str, Any]],
    start: float | None,
    end: float | None,
    expected_xy: tuple[float, float],
) -> dict[str, Any]:
    summary = _coverage_summary(samples, start, end)
    positions = [(float(row["x"]), float(row["y"])) for row in samples]
    anchor = positions[0] if positions else None
    summary.update({
        "span_m": None if anchor is None else max(
            (math.dist(anchor, xy) for xy in positions), default=0.0
        ),
        "landing_error_first_m": (
            None if anchor is None else math.dist(anchor, expected_xy)
        ),
        "landing_error_last_m": (
            None if not positions else math.dist(positions[-1], expected_xy)
        ),
    })
    return summary


@dataclass(frozen=True)
class ProbeConfig:
    """Thresholds and immutable identities for one active-reset episode."""

    reset_seed: int = 8601
    reset_generation: int = 2
    reset_pose: str = "long_route_start_g1"
    reset_odometry: str = "realistic"
    active_timeout_s: float = 6.0
    reset_call_max_delay_s: float = 0.5
    hold_observation_max_delay_s: float = 0.5
    reset_timeout_s: float = 30.0
    quiet_s: float = 1.0
    fresh_timeout_s: float = 30.0
    postzero_s: float = 1.0
    active_displacement_m: float = 0.10
    reset_stable_drift_m: float = 0.02
    reset_landing_xy: tuple[float, float] = (0.45, -5.35)
    reset_odom_landing_xy: tuple[float, float] = (0.0, 0.0)
    reset_landing_error_m: float = 0.10
    fresh_goal_xy: tuple[float, float] = (0.685, -3.975535)
    fresh_goal_error_m: float = 0.30
    fresh_edges: tuple[int, ...] = (51, 52)


@dataclass
class ProbeMachine:
    """ROS-independent exactly-once active-reset state machine."""

    config: ProbeConfig = field(default_factory=ProbeConfig)
    phase: str = "WAIT_ENDPOINTS"
    stop_reason: str | None = None
    timestamps: dict[str, float] = field(default_factory=dict)
    counts: dict[str, int] = field(default_factory=lambda: {
        "old_publish_attempt": 0, "old_publish": 0,
        "fresh_publish_attempt": 0, "fresh_publish": 0,
        "reset_call": 0, "reset_done_callback": 0,
        "canonical": 0, "progress": 0, "lookahead": 0,
        "goal_update": 0, "old_bool": 0, "old_result": 0,
        "fresh_bool": 0, "fresh_result": 0,
        "hold_cmd_vel_sim_nonzero": 0,
    })
    endpoint_detail: dict[str, Any] = field(default_factory=dict)
    topology_checks: list[dict[str, Any]] = field(default_factory=list)
    topology_baseline: dict[str, Any] | None = None
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
    reset_call_detail: dict[str, Any] = field(default_factory=dict)
    baseline_gate_released: bool = False
    gate_sequence: list[dict[str, Any]] = field(default_factory=list)
    gate_observations: list[dict[str, Any]] = field(default_factory=list)
    reset_event_seen: bool = False
    reset_landing_xy: tuple[float, float] | None = None
    reset_stable_max_drift_m: float = 0.0
    old_bool_value: bool | None = None
    old_result_value: dict[str, Any] | None = None
    fresh_bool_value: bool | None = None
    fresh_result_value: dict[str, Any] | None = None
    old_activity_after_terminal: int = 0
    old_outputs_after_hold_boundary: list[dict[str, Any]] = field(
        default_factory=list
    )
    pre_hold_inflight_outputs: list[dict[str, Any]] = field(
        default_factory=list
    )
    latest_odom_xy: tuple[float, float] | None = None
    pose_contract: dict[str, dict[str, Any]] = field(default_factory=lambda: {
        "ground_truth": {
            "topic": GROUND_TRUTH_TOPIC, "expected_frame": "map", "count": 0,
            "normalized_frame": None, "first_source_stamp_s": None,
            "last_source_stamp_s": None,
        },
        "odometry": {
            "topic": ODOMETRY_TOPIC, "expected_frame": "odom", "count": 0,
            "normalized_frame": None, "first_source_stamp_s": None,
            "last_source_stamp_s": None,
        },
    })
    hold_cmd_vel_sim_samples: list[dict[str, Any]] = field(default_factory=list)
    reset_position_samples: dict[str, list[dict[str, Any]]] = field(
        default_factory=lambda: {"ground_truth": [], "odometry": []}
    )
    hold_collision_samples: list[dict[str, Any]] = field(default_factory=list)
    post_commands: dict[str, dict[str, Any]] = field(default_factory=lambda: {
        topic: {"total": 0, "nonzero": 0, "last_zero": False, "samples": []}
        for topic in COMMAND_TOPICS
    })

    def _time(self, name: str, now: float) -> None:
        self.timestamps.setdefault(name, float(now))

    def stop(self, reason: str, now: float) -> None:
        """Enter STOP once, including from provisional completion."""
        safe_now = float(now) if _all_finite((now,)) else float(
            max(self.timestamps.values(), default=0.0)
        )
        if self.phase != "STOP":
            self.stop_reason = reason
            self._time("stop", safe_now)
            self.phase = "STOP"

    def _valid_time(self, event: str, now: float) -> bool:
        if _all_finite((now,)):
            return True
        self.stop(f"{event}_time_nonfinite", now)
        return False

    def _hold_boundary(self) -> float | None:
        return next((
            row["received_monotonic_s"] for row in self.gate_sequence
            if row["reason"] == "hold"
        ), None)

    def _reject_old_output(
        self,
        kind: str,
        now: float,
        *,
        request_id: int | None = None,
        detail: dict[str, Any] | None = None,
    ) -> bool:
        if self.counts["fresh_publish"] != 0:
            return False
        row = {
            "type": kind,
            "received_monotonic_s": float(now),
            "request_id": request_id,
            "detail": dict(detail or {}),
        }
        hold_boundary = self._hold_boundary()
        dispatch_boundary = self.timestamps.get("reset_call")
        if hold_boundary is None:
            if dispatch_boundary is not None and now >= dispatch_boundary:
                # Service dispatch and the received HOLD status are different
                # callback boundaries.  Outputs in this bounded interval are
                # observable in-flight work, not proof of post-HOLD leakage.
                self.pre_hold_inflight_outputs.append(row)
            return False
        if now < hold_boundary:
            self.pre_hold_inflight_outputs.append(row)
            return False
        self.old_outputs_after_hold_boundary.append(row)
        self.stop(f"old_output_after_hold_boundary:{kind}", now)
        return True

    def endpoints_ready(self, now: float, detail: dict[str, Any]) -> None:
        """Accept a complete endpoint snapshot and arm old publication."""
        if not self._valid_time("endpoints", now):
            return
        if self.phase != "WAIT_ENDPOINTS":
            return
        self.endpoint_detail = dict(detail)
        if detail.get("ready") is not True:
            return
        if self.latest_gt_xy is None:
            return
        self.phase = "PUBLISH_OLD_ONCE"
        self._time("endpoints_ready", now)

    def topology_checked(
        self,
        label: str,
        snapshot: dict[str, Any],
        errors: list[str],
        now: float,
    ) -> None:
        """Record and compare one exact safety-critical ROS graph snapshot."""
        if not self._valid_time(f"topology_{label}", now):
            return
        expected_index = len(self.topology_checks)
        if label not in TOPOLOGY_CHECKPOINTS or (
            expected_index >= len(TOPOLOGY_CHECKPOINTS)
            or label != TOPOLOGY_CHECKPOINTS[expected_index]
        ):
            self.stop(f"topology_checkpoint_order_invalid:{label}", now)
            return
        record = {
            "label": label,
            "monotonic_s": float(now),
            "snapshot": snapshot,
            "errors": list(errors),
        }
        self.topology_checks.append(record)
        if errors:
            self.stop(f"topology_contract_failed:{label}:{errors[0]}", now)
            return
        if self.topology_baseline is None:
            self.topology_baseline = snapshot
        elif snapshot != self.topology_baseline:
            self.stop(f"topology_changed:{label}", now)

    def old_published(self, now: float) -> None:
        """Record the sole old-goal publication."""
        if not self._valid_time("old_publish", now):
            return
        if self.phase != "PUBLISH_OLD_ONCE" or self.counts["old_publish"]:
            self.stop("old_goal_publish_not_exactly_once", now)
            return
        self.counts["old_publish"] = 1
        self.old_start_xy = self.latest_gt_xy
        self._time("old_published", now)
        self.phase = "WAIT_ACTIVE_READY"

    def fresh_published(self, now: float) -> None:
        """Record the sole fresh-goal publication."""
        if not self._valid_time("fresh_publish", now):
            return
        if self.phase != "PUBLISH_FRESH_ONCE" or self.counts["fresh_publish"]:
            self.stop("fresh_goal_publish_not_exactly_once", now)
            return
        self.counts["fresh_publish"] = 1
        self._time("fresh_published", now)
        self.phase = "WAIT_SUCCESS"

    def canonical(self, request_id: int, edge_ids: list[int], now: float) -> None:
        """Consume a canonical-route observation."""
        if not self._valid_time("canonical", now):
            return
        self.counts["canonical"] += 1
        if self.counts["old_publish"] == 0:
            return
        if self._reject_old_output(
            "canonical", now, request_id=int(request_id),
            detail={"edge_ids": [int(edge) for edge in edge_ids]},
        ):
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
                if (
                    self.old_request_id is None
                    or self.fresh_request_id <= self.old_request_id
                ):
                    self.stop("fresh_request_id_not_newer", now)
                elif tuple(edge_ids) == self.config.fresh_edges:
                    self.fresh_edges_equivalence = "canonical_exact"
                else:
                    self.stop("fresh_canonical_edges_mismatch", now)
            elif int(request_id) != self.fresh_request_id:
                self.stop("fresh_request_id_changed", now)
        self._maybe_active_ready(now)

    def progress(self, request_id: int, now: float) -> None:
        """Consume a route-progress observation."""
        if not self._valid_time("progress", now):
            return
        self.counts["progress"] += 1
        if self.counts["old_publish"] == 0:
            return
        if self._reject_old_output("progress", now, request_id=int(request_id)):
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
        if not self._valid_time(kind, now):
            return
        if kind not in {"lookahead", "goal_update"}:
            self.stop("unknown_route_output", now)
            return
        self.counts[kind] += 1
        if self.counts["old_publish"] == 0:
            return
        if self._reject_old_output(kind, now):
            return
        if self.counts["fresh_publish"] == 0:
            if kind == "lookahead":
                self.old_lookahead_seen = True
            else:
                self.old_goal_update_seen = True
        self._maybe_active_ready(now)

    def navigate_intent(self, now: float) -> None:
        """Consume the coordinator's `/goal_update` NavigateToPose intent."""
        if not self._valid_time("navigate_intent", now):
            return
        self.counts["goal_update"] += 1
        if self.counts["old_publish"] == 0:
            return
        if self._reject_old_output("navigate_intent", now):
            return
        if self.counts["fresh_publish"] == 0:
            self.old_goal_update_seen = True
        self._maybe_active_ready(now)

    def command(
        self,
        topic: str,
        nonzero: bool,
        now: float,
        values: tuple[float, ...] | None = None,
    ) -> None:
        """Consume one command-chain sample."""
        if not self._valid_time("command", now):
            return
        if values is not None and (len(values) != 6 or not _all_finite(values)):
            self.stop(f"twist_nonfinite:{topic}", now)
            return
        if not isinstance(nonzero, bool):
            self.stop(f"twist_nonzero_flag_invalid:{topic}", now)
            return
        if topic not in COMMAND_TOPICS:
            self.stop("unknown_command_topic", now)
            return
        if topic == "/cmd_vel_sim" and self.phase == "WAIT_ACTIVE_READY" and nonzero:
            self.old_cmd_nonzero += 1
            self._maybe_active_ready(now)
        hold_seen = any(item["reason"] == "hold" for item in self.gate_sequence)
        released = any(item["reason"].startswith("released:") for item in self.gate_sequence)
        if topic == "/cmd_vel_sim" and hold_seen and not released:
            self.hold_cmd_vel_sim_samples.append({
                "monotonic_s": float(now), "nonzero": bool(nonzero),
                "twist": None if values is None else list(values),
            })
            if nonzero:
                self.counts["hold_cmd_vel_sim_nonzero"] += 1
                self.stop("hold_cmd_vel_sim_nonzero", now)
        if self.phase == "PROVISIONAL_COMPLETE" and nonzero:
            self.stop(f"command_nonzero_after_provisional:{topic}", now)
            return
        if self.phase == "POSTZERO":
            row = self.post_commands[topic]
            row["total"] += 1
            row["nonzero"] += int(nonzero)
            row["last_zero"] = not nonzero
            row["samples"].append({
                "monotonic_s": float(now), "nonzero": bool(nonzero),
                "twist": None if values is None else list(values),
            })

    def _validate_pose_sample(
        self,
        source: str,
        x: float,
        y: float,
        z: float,
        quaternion: tuple[float, float, float, float],
        frame_id: str,
        source_stamp_s: float,
        now: float,
    ) -> bool:
        if not self._valid_time(source, now):
            return False
        values = (x, y, z, *quaternion, source_stamp_s)
        if len(quaternion) != 4 or not _all_finite(values):
            self.stop(f"{source}_sample_nonfinite", now)
            return False
        normalized = _normalized_frame(frame_id)
        contract = self.pose_contract[source]
        if normalized != contract["expected_frame"]:
            self.stop(
                f"{source}_frame_mismatch:expected={contract['expected_frame']}:"
                f"observed={normalized}",
                now,
            )
            return False
        contract["count"] += 1
        contract["normalized_frame"] = normalized
        contract["first_source_stamp_s"] = (
            float(source_stamp_s)
            if contract["first_source_stamp_s"] is None
            else contract["first_source_stamp_s"]
        )
        contract["last_source_stamp_s"] = float(source_stamp_s)
        contract["latest"] = {
            "received_monotonic_s": float(now),
            "source_stamp_s": float(source_stamp_s),
            "position": [float(x), float(y), float(z)],
            "quaternion": [float(value) for value in quaternion],
            "frame_id": str(frame_id),
            "normalized_frame": normalized,
        }
        return True

    def ground_truth(
        self,
        x: float,
        y: float,
        now: float,
        *,
        z: float = 0.0,
        quaternion: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
        frame_id: str = "map",
        source_stamp_s: float = 0.0,
    ) -> None:
        """Consume one evaluator-only ground-truth position."""
        if not self._validate_pose_sample(
            "ground_truth", x, y, z, quaternion, frame_id, source_stamp_s, now
        ):
            return
        xy = (float(x), float(y))
        self.latest_gt_xy = xy
        self._capture_reset_position("ground_truth", xy, now)
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

    def odometry(
        self,
        x: float,
        y: float,
        now: float,
        *,
        z: float = 0.0,
        quaternion: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
        frame_id: str = "odom",
        source_stamp_s: float = 0.0,
    ) -> None:
        """Consume estimated odometry for independent reset-stability coverage."""
        if not self._validate_pose_sample(
            "odometry", x, y, z, quaternion, frame_id, source_stamp_s, now
        ):
            return
        xy = (float(x), float(y))
        self.latest_odom_xy = xy
        self._capture_reset_position("odometry", xy, now)

    def _capture_reset_position(
        self, source: str, xy: tuple[float, float], now: float
    ) -> None:
        reasons = [item["reason"] for item in self.gate_sequence]
        if (
            self.reset_event_seen
            and "reset_complete" in reasons
            and not any(reason.startswith("released:") for reason in reasons)
        ):
            self.reset_position_samples[source].append({
                "monotonic_s": float(now), "x": xy[0], "y": xy[1]
            })

    def collision_event(self, collided: bool, now: float) -> None:
        """Latch physical collision and fail immediately when true."""
        if not self._valid_time("collision", now):
            return
        reasons = [item["reason"] for item in self.gate_sequence]
        if (
            "hold" in reasons
            and not any(reason.startswith("released:") for reason in reasons)
        ):
            self.hold_collision_samples.append({
                "monotonic_s": float(now), "collided": bool(collided)
            })
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
        if not self._valid_time("reset_call", now):
            return
        if self.phase != "CALL_RESET_ONCE" or self.counts["reset_call"]:
            self.stop("reset_call_not_exactly_once", now)
            return
        active = self.timestamps.get("active_ready")
        if not self.baseline_gate_released:
            self.stop("reset_call_without_generation_1_release", now)
            return
        if (
            active is None
            or now < active
            or now - active > self.config.reset_call_max_delay_s
        ):
            self.stop("reset_call_after_active_ready_too_late", now)
            return
        self.counts["reset_call"] = 1
        self._time("reset_call", now)
        self.reset_call_detail = {
            "status": "dispatch_boundary_accepted",
            "dispatch_monotonic_s": float(now),
            "service_ready": True,
        }
        self.phase = "OBSERVE_HOLD_ABORT"

    def reset_response(self, success: bool, message: str, now: float) -> None:
        """Validate the Trigger result and exact reset receipt."""
        if not self._valid_time("reset_response", now):
            return
        if self.counts["reset_call"] != 1 or self.reset_receipt is not None:
            self.stop("unexpected_reset_response", now)
            return
        self.reset_call_detail.update({
            "response_monotonic_s": float(now),
            "response_success": bool(success),
            "response_message": str(message),
        })
        if not success:
            self.reset_call_detail.update({
                "status": "response_failed",
                "response_monotonic_s": float(now),
                "response_success": False,
                "response_message": str(message),
            })
            self.stop("reset_service_failed", now)
            return
        try:
            receipt = parse_reset_receipt(
                message, requested_seed=self.config.reset_seed
            )
        except ResetReceiptError as exc:
            self.reset_call_detail.update({
                "status": "receipt_invalid",
                "exception_type": type(exc).__name__,
                "exception_message": str(exc),
            })
            self.stop(f"reset_receipt_invalid:{exc}", now)
            return
        if (
            receipt["generation"] != self.config.reset_generation
            or receipt["pose"] != self.config.reset_pose
            or receipt["odometry"] != self.config.reset_odometry
        ):
            self.reset_call_detail["status"] = "receipt_contract_mismatch"
            self.stop("reset_receipt_contract_mismatch", now)
            return
        self.reset_receipt = receipt
        self.reset_call_detail.update({
            "status": "response_valid",
            "response_monotonic_s": float(now),
            "response_success": True,
            "response_message": str(message),
        })
        self._time("reset_receipt", now)
        self._advance_reset_observation(now)

    def gate_status(self, payload: str, now: float) -> None:
        """Consume and order one ResetStopGate status document."""
        if not self._valid_time("gate_status", now):
            return
        observation = {
            "received_monotonic_s": float(now),
            "payload": str(payload),
        }
        self.gate_observations.append(observation)
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
            or isinstance(eligible, bool)
            or (eligible is not None and not isinstance(eligible, int))
            or not isinstance(reason, str)
        ):
            self.stop("gate_status_field_invalid", now)
            return
        observation["document"] = dict(item)
        if self.counts["reset_call"] == 0:
            expected_baseline = (
                generation == self.config.reset_generation - 1
                and not held
                and eligible is None
                and reason.startswith("released:")
            )
            if not expected_baseline:
                if generation >= self.config.reset_generation:
                    self.stop("gate_generation_before_reset_call", now)
                else:
                    self.stop("gate_baseline_sequence_invalid", now)
                return
            if self.baseline_gate_released:
                self.stop("gate_baseline_duplicate", now)
                return
            self.baseline_gate_released = True
            return

        if generation != self.config.reset_generation:
            self.stop(
                "gate_generation_advanced_unexpectedly"
                if generation > self.config.reset_generation
                else "gate_generation_regressed_unexpectedly",
                now,
            )
            return
        valid = (
            (reason == "hold" and held and eligible is None)
            or (reason == "reset_complete" and held and eligible == generation)
            or (reason.startswith("released:") and not held and eligible is None)
        )
        if not valid:
            self.stop("gate_status_incoherent", now)
            return
        if len(self.gate_sequence) >= 3:
            self.stop("gate_status_extra_after_release", now)
            return
        expected_reason = (
            "hold" if not self.gate_sequence else
            "reset_complete" if len(self.gate_sequence) == 1 else
            "released:"
        )
        if (
            (expected_reason == "released:" and not reason.startswith(expected_reason))
            or (expected_reason != "released:" and reason != expected_reason)
        ):
            self.stop("gate_status_order_invalid", now)
            return
        if self.gate_sequence and now <= self.gate_sequence[-1]["received_monotonic_s"]:
            self.stop("gate_status_received_time_not_increasing", now)
            return
        self.gate_sequence.append({**item, "received_monotonic_s": float(now)})
        if reason == "hold":
            dispatch = self.timestamps.get("reset_call")
            latency = None if dispatch is None else float(now) - float(dispatch)
            self._time("gate_hold", now)
            self.reset_call_detail.update({
                "hold_received_monotonic_s": float(now),
                "dispatch_to_hold_s": latency,
            })
            if (
                latency is None
                or latency < 0.0
                or latency > self.config.hold_observation_max_delay_s
            ):
                self.stop("gate_hold_after_reset_dispatch_too_late", now)
                return
        if reason.startswith("released:"):
            self._validate_release_coverage(now)
            if self.phase == "STOP":
                return
        self._advance_reset_observation(now)

    @staticmethod
    def _coverage_contract_errors(
        summary: dict[str, Any], *, require_zero_key: str | None = None
    ) -> list[str]:
        errors: list[str] = []
        if summary["count"] < 2:
            errors.append("count_lt_2")
        max_gap = summary["max_gap_s"]
        if max_gap is None or max_gap > MAX_COVERAGE_GAP_S:
            errors.append("max_gap")
        start_delay = summary["start_edge_delay_s"]
        if start_delay is None or abs(start_delay) > MAX_COVERAGE_GAP_S:
            errors.append("start_edge")
        end_lead = summary["end_edge_lead_s"]
        if end_lead is None or abs(end_lead) > MAX_COVERAGE_GAP_S:
            errors.append("end_edge")
        if require_zero_key is not None and any(
            bool(row[require_zero_key]) for row in summary["samples"]
        ):
            errors.append("nonzero_or_true")
        return errors

    def _validate_release_coverage(self, release_time: float) -> None:
        hold_time = self.gate_sequence[0]["received_monotonic_s"]
        stable_time = self.gate_sequence[1]["received_monotonic_s"]
        command = _coverage_summary(
            self.hold_cmd_vel_sim_samples, hold_time, release_time
        )
        errors = self._coverage_contract_errors(
            command, require_zero_key="nonzero"
        )
        if errors:
            self.stop("hold_cmd_vel_sim_coverage_failed:" + ",".join(errors), release_time)
            return
        collision = _coverage_summary(
            self.hold_collision_samples, hold_time, release_time
        )
        errors = self._coverage_contract_errors(
            collision, require_zero_key="collided"
        )
        if errors:
            self.stop("hold_collision_coverage_failed:" + ",".join(errors), release_time)
            return
        for source in ("ground_truth", "odometry"):
            expected_xy = (
                self.config.reset_landing_xy
                if source == "ground_truth"
                else self.config.reset_odom_landing_xy
            )
            summary = _position_coverage_summary(
                self.reset_position_samples[source],
                stable_time,
                release_time,
                expected_xy,
            )
            errors = self._coverage_contract_errors(summary)
            if summary["span_m"] is None or (
                summary["span_m"] > self.config.reset_stable_drift_m
            ):
                errors.append("span")
            landing_errors = (
                summary["landing_error_first_m"],
                summary["landing_error_last_m"],
            )
            if any(
                value is None or value > self.config.reset_landing_error_m
                for value in landing_errors
            ):
                errors.append("landing_error")
            if errors:
                self.stop(
                    f"reset_{source}_coverage_failed:" + ",".join(errors),
                    release_time,
                )
                return

    def reset_event(self, now: float) -> None:
        """Consume the one reset event belonging to this Trigger call."""
        if not self._valid_time("reset_event", now):
            return
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
        if not self._valid_time("terminal_bool", now):
            return
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
        if not self._valid_time("terminal_result", now):
            return
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
                or item.get("reason") != "final_goal_distance_confirmed"
                or item.get("reset_epoch") != self.config.reset_generation
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
                or item.get("reset_epoch") != self.config.reset_generation
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
            self.phase in RUN_EXIT_PHASES
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
        if not self._valid_time("tick", now):
            return
        if self.phase == "WAIT_ACTIVE_READY":
            if now - self.timestamps["old_published"] > self.config.active_timeout_s:
                self.stop("active_ready_timeout", now)
        elif self.phase in {"OBSERVE_HOLD_ABORT", "WAIT_RELEASE"}:
            if (
                self._hold_boundary() is None
                and now - self.timestamps["reset_call"]
                > self.config.hold_observation_max_delay_s
            ):
                self.stop("gate_hold_observation_timeout", now)
            elif now - self.timestamps["reset_call"] > self.config.reset_timeout_s:
                self.stop("reset_observation_timeout", now)
        elif self.phase == "QUIET":
            if now - self.timestamps["quiet_started"] >= self.config.quiet_s:
                self.phase = "PUBLISH_FRESH_ONCE"
        elif self.phase == "WAIT_SUCCESS":
            if now - self.timestamps["fresh_published"] > self.config.fresh_timeout_s:
                self.stop("fresh_success_timeout", now)
        elif self.phase == "POSTZERO":
            if now - self.timestamps["postzero_started"] >= self.config.postzero_s:
                start = self.timestamps["postzero_started"]
                end = start + self.config.postzero_s
                invalid = []
                for topic, row in self.post_commands.items():
                    summary = _coverage_summary(row["samples"], start, end)
                    errors = self._coverage_contract_errors(
                        summary, require_zero_key="nonzero"
                    )
                    if errors or row["nonzero"] != 0 or not row["last_zero"]:
                        invalid.append(topic)
                if invalid:
                    self.stop("postzero_contract_failed:" + ",".join(invalid), now)
                elif self.collision:
                    self.stop("physical_collision", now)
                else:
                    self.timestamps["provisional_complete"] = float(now)
                    self.phase = "PROVISIONAL_COMPLETE"

    def document(
        self,
        *,
        finalized: bool = False,
        lifecycle: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        """Return the complete machine-readable evidence snapshot."""
        hold_time = next((
            row["received_monotonic_s"] for row in self.gate_sequence
            if row["reason"] == "hold"
        ), None)
        stable_time = next((
            row["received_monotonic_s"] for row in self.gate_sequence
            if row["reason"] == "reset_complete"
        ), None)
        release_time = next((
            row["received_monotonic_s"] for row in self.gate_sequence
            if row["reason"].startswith("released:")
        ), None)
        postzero_start = self.timestamps.get("postzero_started")
        postzero_end = (
            None if postzero_start is None
            else postzero_start + self.config.postzero_s
        )
        return {
            "schema": "bio_nav.active_reset_probe.v1",
            "phase": self.phase,
            "verdict": (
                "PASS_REQUIRES_BAG"
                if finalized and self.phase == "PROVISIONAL_COMPLETE"
                else "PROVISIONAL_PASS_REQUIRES_BAG_ORDER"
                if self.phase == "PROVISIONAL_COMPLETE"
                else "STOP" if self.phase == "STOP" else "RUNNING"
            ),
            "finalized": bool(finalized),
            "lifecycle": dict(lifecycle or {}),
            "claim_boundary": {
                "engineering_pass": False,
                "requires_external_bag_order_analysis": True,
                "callback_times_are_receive_order_not_source_order": True,
            },
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
            "pose_contract": self.pose_contract,
            "topology_checks": list(self.topology_checks),
            "old": {
                "request_id": self.old_request_id,
                "edges": self.old_edges,
                "displacement_m": self.old_displacement_m,
                "cmd_vel_sim_nonzero": self.old_cmd_nonzero,
                "terminal_bool": self.old_bool_value,
                "result": self.old_result_value,
                "activity_after_terminal": self.old_activity_after_terminal,
                "pre_hold_inflight_outputs": self.pre_hold_inflight_outputs,
                "outputs_after_hold_boundary": self.old_outputs_after_hold_boundary,
            },
            "reset": {
                "receipt": self.reset_receipt,
                "call": self.reset_call_detail,
                "gate_sequence": self.gate_sequence,
                "gate_observations": self.gate_observations,
                "event_seen": self.reset_event_seen,
                "landing_xy": self.reset_landing_xy,
                "stable_max_drift_m": self.reset_stable_max_drift_m,
                "raw_callback_times_monotonic_s": {
                    "reset_event": self.timestamps.get("reset_event"),
                    "reset_receipt": self.timestamps.get("reset_receipt"),
                    "gate_hold": hold_time,
                    "gate_reset_complete": stable_time,
                    "gate_released": release_time,
                },
                "coverage": {
                    "cmd_vel_sim_hold_to_release": _coverage_summary(
                        self.hold_cmd_vel_sim_samples, hold_time, release_time
                    ),
                    "collision_hold_to_release": _coverage_summary(
                        self.hold_collision_samples, hold_time, release_time
                    ),
                    "ground_truth_stable_to_release": _position_coverage_summary(
                        self.reset_position_samples["ground_truth"],
                        stable_time,
                        release_time,
                        self.config.reset_landing_xy,
                    ),
                    "odometry_stable_to_release": _position_coverage_summary(
                        self.reset_position_samples["odometry"],
                        stable_time,
                        release_time,
                        self.config.reset_odom_landing_xy,
                    ),
                },
            },
            "fresh": {
                "request_id": self.fresh_request_id,
                "edges": self.fresh_edges_observed,
                "edge_equivalence": self.fresh_edges_equivalence,
                "terminal_bool": self.fresh_bool_value,
                "result": self.fresh_result_value,
                "latest_gt_xy": self.latest_gt_xy,
                "latest_odom_xy": self.latest_odom_xy,
                "goal_error_m": None if self.latest_gt_xy is None else math.dist(
                    self.latest_gt_xy, self.config.fresh_goal_xy
                ),
            },
            "collision": self.collision,
            "postzero": {
                topic: {
                    **row,
                    "coverage": _coverage_summary(
                        row["samples"], postzero_start, postzero_end
                    ),
                }
                for topic, row in self.post_commands.items()
            },
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
        directory_descriptor = os.open(path.parent, os.O_RDONLY)
        try:
            os.fsync(directory_descriptor)
        finally:
            os.close(directory_descriptor)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _emergency_path(path: Path) -> Path:
    return path.with_name(f"{path.name}.emergency-stop.json")


def _emergency_document(reason: str) -> dict[str, Any]:
    return {
        "schema": "bio_nav.active_reset_probe.v1",
        "phase": "STOP",
        "verdict": "STOP",
        "finalized": False,
        "stop_reason": reason,
        "claim_boundary": {
            "engineering_pass": False,
            "requires_external_bag_order_analysis": True,
        },
    }


def _best_effort_emergency_stop(path: Path, reason: str) -> dict[str, Any]:
    """Try both the final path and a distinct emergency STOP receipt."""
    document = _emergency_document(reason)
    errors: list[str] = []
    for target in (path, _emergency_path(path)):
        try:
            atomic_write_json(target, document)
        except Exception as exc:  # pragma: no cover - filesystem dependent
            errors.append(f"{target}:{type(exc).__name__}:{exc}")
    if errors:
        document["emergency_write_errors"] = errors
    return document


def dispatch_reset_once(
    machine: ProbeMachine,
    *,
    check_topology: Callable[[], bool],
    service_ready: Callable[[], bool],
    call_async: Callable[[], Any],
    done_callback: Callable[[Any], None],
    monotonic: Callable[[], float] = time.monotonic,
) -> Any | None:
    """Perform the fenced Trigger dispatch at its actual monotonic boundary."""
    if not check_topology():
        return None
    if not service_ready():
        machine.reset_call_detail = {
            "status": "service_unavailable_before_dispatch",
            "service_ready": False,
        }
        machine.stop("reset_service_disappeared_before_call", monotonic())
        return None
    dispatch_now = monotonic()
    machine.reset_call_started(dispatch_now)
    if machine.phase == "STOP":
        return None
    try:
        future = call_async()
        future.add_done_callback(done_callback)
        machine.reset_call_detail["status"] = "future_callback_registered"
        return future
    except Exception as exc:
        machine.reset_call_detail.update({
            "status": "dispatch_exception",
            "exception_type": type(exc).__name__,
            "exception_message": str(exc),
        })
        machine.stop(
            f"reset_dispatch_exception:{type(exc).__name__}:{exc}", monotonic()
        )
        return None


def teardown_ros_node(
    node: Any,
    rclpy_module: Any,
    *,
    monotonic: Callable[[], float] = time.monotonic,
) -> dict[str, bool]:
    """Destroy and shutdown while converting either failure into STOP."""
    lifecycle = {"node_destroyed": False, "rclpy_shutdown": False}
    try:
        result = node.destroy_node()
        if result is False:
            raise RuntimeError("destroy_node_returned_false")
        lifecycle["node_destroyed"] = True
    except Exception as exc:
        node.machine.stop(
            f"destroy_node_exception:{type(exc).__name__}:{exc}", monotonic()
        )
    try:
        rclpy_module.shutdown()
        lifecycle["rclpy_shutdown"] = True
    except Exception as exc:
        node.machine.stop(
            f"shutdown_exception:{type(exc).__name__}:{exc}", monotonic()
        )
    return lifecycle


def finalize_probe_output(
    machine: ProbeMachine,
    path: Path,
    lifecycle: dict[str, bool],
    *,
    persistence_failed: bool = False,
    writer: Callable[[Path, dict[str, Any]], None] | None = None,
) -> tuple[int, dict[str, Any]]:
    """Write the only final verdict after teardown and durable replacement."""
    teardown_complete = all(lifecycle.values())
    eligible = (
        machine.phase == "PROVISIONAL_COMPLETE"
        and teardown_complete
        and not persistence_failed
    )
    if not eligible and machine.phase != "STOP":
        machine.stop("finalization_contract_incomplete", time.monotonic())
    document = machine.document(finalized=eligible, lifecycle=lifecycle)
    try:
        (writer or atomic_write_json)(path, document)
    except Exception as exc:
        machine.stop(
            f"final_json_write_exception:{type(exc).__name__}:{exc}",
            time.monotonic(),
        )
        document = machine.document(finalized=False, lifecycle=lifecycle)
        emergency = _best_effort_emergency_stop(
            path, machine.stop_reason or "final_json_write_exception"
        )
        if emergency.get("emergency_write_errors"):
            document["emergency_write_errors"] = emergency[
                "emergency_write_errors"
            ]
        return 20, document
    return (0 if eligible else 20), document


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
    parser.add_argument(
        "--expected-route-subscriber",
        action="append",
        dest="expected_route_subscribers",
        help=(
            "exact fully-qualified subscriber identity; repeat once per expected "
            "subscriber (default: bio_nav_route_coordinator and rosbag2_recorder)"
        ),
    )
    parser.add_argument("--reset-seed", type=int, default=8601)
    parser.add_argument("--fresh-edges", type=_parse_edges, default=(51, 52))
    parser.add_argument(
        "--reset-landing-xy", nargs=2, type=float, default=(0.45, -5.35)
    )
    parser.add_argument(
        "--reset-odom-landing-xy", nargs=2, type=float, default=(0.0, 0.0)
    )
    parser.add_argument("--endpoint-timeout", type=float, default=30.0)
    args = parser.parse_args(argv)
    args.expected_route_subscribers = tuple(
        args.expected_route_subscribers or DEFAULT_ROUTE_SUBSCRIBERS
    )
    if (
        not args.expected_route_subscribers
        or len(set(args.expected_route_subscribers))
        != len(args.expected_route_subscribers)
        or any(not name.startswith("/") for name in args.expected_route_subscribers)
    ):
        parser.error("expected route subscriber identities must be unique absolute names")
    if args.reset_seed < 0:
        parser.error("--reset-seed must be non-negative")
    if args.endpoint_timeout <= 0.0:
        parser.error("--endpoint-timeout must be positive")
    if not all(math.isfinite(value) for value in (
        *args.old_goal, *args.fresh_goal,
        *args.reset_landing_xy, *args.reset_odom_landing_xy,
    )):
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
                reset_landing_xy=tuple(args.reset_landing_xy),
                reset_odom_landing_xy=tuple(args.reset_odom_landing_xy),
                fresh_goal_xy=(args.fresh_goal[0], args.fresh_goal[1]),
                fresh_edges=tuple(args.fresh_edges),
            )
            self.machine = ProbeMachine(config)
            self.started = time.monotonic()
            self._old_goal = tuple(args.old_goal)
            self._fresh_goal = tuple(args.fresh_goal)
            self._expected_subscribers = tuple(args.expected_route_subscribers)
            self._endpoint_timeout = args.endpoint_timeout
            self._reset_future = None
            self._last_persist = -math.inf
            self._persistence_failed = False
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
                lambda _m: self._event(self.machine.navigate_intent),
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
                GROUND_TRUTH_TOPIC,
                lambda m: self._pose_event("ground_truth", m),
                best_effort,
            )
            self.create_subscription(
                Odometry,
                ODOMETRY_TOPIC,
                lambda m: self._pose_event("odometry", m),
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
                    lambda m, t=topic: self._command_event(t, m),
                    best_effort,
                )
            self.create_timer(0.02, self._tick)
            self._persist(force=True)

        def _pose_event(self, source: str, message) -> None:
            pose = message.pose.pose
            stamp = message.header.stamp
            source_stamp_s = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
            callback = (
                self.machine.ground_truth
                if source == "ground_truth" else self.machine.odometry
            )
            self._event(
                callback,
                float(pose.position.x),
                float(pose.position.y),
                z=float(pose.position.z),
                quaternion=(
                    float(pose.orientation.x), float(pose.orientation.y),
                    float(pose.orientation.z), float(pose.orientation.w),
                ),
                frame_id=str(message.header.frame_id),
                source_stamp_s=source_stamp_s,
            )

        def _command_event(self, topic: str, message) -> None:
            try:
                values = _twist_values(message)
            except Exception as exc:
                now = time.monotonic()
                self.machine.stop(
                    f"twist_callback_exception:{type(exc).__name__}:{exc}", now
                )
                self._persist(force=True)
                return
            self._event(
                self.machine.command,
                topic,
                any(abs(value) > 1.0e-6 for value in values),
                values=values,
            )

        def _event(self, callback, *values, **keywords) -> None:
            now = time.monotonic()
            previous = self.machine.phase
            try:
                callback(*values, now, **keywords)
                self._drive(now)
            except Exception as exc:
                self.machine.stop(
                    f"callback_exception:{type(exc).__name__}:{exc}", now
                )
            self._persist(force=self.machine.phase != previous)

        def _persist(self, *, force: bool = False) -> bool:
            now = time.monotonic()
            # Topic callbacks can exceed 200 Hz.  A bounded 20 Hz atomic
            # stream is continuously observable without making NAS fsync the
            # latency-critical active-ready -> Trigger path.
            if not force and now - self._last_persist < 0.05:
                return True
            try:
                atomic_write_json(args.output, self.machine.document())
            except Exception as exc:
                self._persistence_failed = True
                self.machine.stop(
                    f"json_write_exception:{type(exc).__name__}:{exc}", now
                )
                emergency = _best_effort_emergency_stop(
                    args.output, self.machine.stop_reason or "json_write_exception"
                )
                self.machine.endpoint_detail["terminal_json_write_error"] = (
                    emergency.get("emergency_write_errors", [])
                )
                return False
            self._last_persist = now
            return True

        def _topology_snapshot(self) -> dict[str, Any]:
            topics: dict[str, Any] = {}
            for topic in ("/cmd_vel", "/cmd_vel_sim", "/bio_nav/route_goal"):
                publishers = sorted(
                    (_endpoint_document(info) for info in
                     self.get_publishers_info_by_topic(topic)),
                    key=lambda row: (row["node"], row["gid"]),
                )
                subscriptions = sorted(
                    (_endpoint_document(info) for info in
                     self.get_subscriptions_info_by_topic(topic)),
                    key=lambda row: (row["node"], row["gid"]),
                )
                topics[topic] = {
                    "publishers": publishers,
                    "subscriptions": subscriptions,
                    "publisher_count": len(publishers),
                    "subscription_count": len(subscriptions),
                }
            return {"topics": topics}

        def _check_topology(self, label: str) -> bool:
            snapshot = self._topology_snapshot()
            errors = validate_topology_snapshot(
                snapshot, self._expected_subscribers
            )
            self.machine.topology_checked(
                label, snapshot, errors, time.monotonic()
            )
            return self.machine.phase != "STOP"

        def _endpoints(self) -> dict[str, Any]:
            topics = (
                "/bio_nav/canonical_route", "/bio_nav/route_progress",
                "/bio_nav/route_lookahead_goal", "/goal_update",
                "/bio_nav/route_goal_complete", "/bio_nav/route_goal_result",
                GROUND_TRUTH_TOPIC, ODOMETRY_TOPIC, "/simulation/collision",
                "/simulation/reset_stop_gate/status", "/simulation/reset_event",
                *COMMAND_TOPICS,
            )
            topology = self._topology_snapshot()
            topology_errors = validate_topology_snapshot(
                topology, self._expected_subscribers
            )
            availability = {
                "reset_service": self._reset_client.service_is_ready(),
                "ground_truth_sample": self.machine.latest_gt_xy is not None,
                "odometry_sample": self.machine.latest_odom_xy is not None,
                "gate_released_baseline": self.machine.baseline_gate_released,
                **{
                    f"publisher:{topic}": self.count_publishers(topic) >= 1
                    for topic in topics
                },
            }
            return {
                "ready": not topology_errors and all(availability.values()),
                "availability": availability,
                "topology": topology,
                "topology_errors": topology_errors,
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
            try:
                if self.machine.phase == "WAIT_ENDPOINTS":
                    self.machine.endpoints_ready(now, self._endpoints())
                    if now - self.started > self._endpoint_timeout:
                        self.machine.stop("endpoint_timeout", now)
                self.machine.tick(now)
                self._drive(now)
            except Exception as exc:
                self.machine.stop(
                    f"tick_exception:{type(exc).__name__}:{exc}", now
                )
            self._persist(force=self.machine.phase != previous)

        def _drive(self, now: float) -> None:
            """Execute state-owned exactly-once side effects immediately."""
            if self.machine.phase == "PUBLISH_OLD_ONCE":
                if not self._check_topology("prepublish"):
                    return
                self.machine.counts["old_publish_attempt"] += 1
                if self.machine.counts["old_publish_attempt"] != 1:
                    self.machine.stop("old_goal_publish_attempt_repeated", now)
                    return
                self._publish_goal(self._old_goal)
                self.machine.old_published(time.monotonic())
            elif self.machine.phase == "CALL_RESET_ONCE":
                self._reset_future = dispatch_reset_once(
                    self.machine,
                    check_topology=lambda: self._check_topology("pre_reset"),
                    service_ready=self._reset_client.service_is_ready,
                    call_async=lambda: self._reset_client.call_async(
                        Trigger.Request()
                    ),
                    done_callback=self._reset_done,
                )
            elif self.machine.phase == "QUIET" and not any(
                row["label"] == "post_release" for row in self.machine.topology_checks
            ):
                self._check_topology("post_release")
            elif self.machine.phase == "PUBLISH_FRESH_ONCE":
                if not self._check_topology("pre_fresh"):
                    return
                self.machine.counts["fresh_publish_attempt"] += 1
                if self.machine.counts["fresh_publish_attempt"] != 1:
                    self.machine.stop("fresh_goal_publish_attempt_repeated", now)
                    return
                self._publish_goal(self._fresh_goal)
                self.machine.fresh_published(now)

        def _reset_done(self, future) -> None:
            now = time.monotonic()
            self.machine.reset_call_detail["done_callback_monotonic_s"] = float(now)
            try:
                self.machine.counts["reset_done_callback"] += 1
                if self.machine.counts["reset_done_callback"] != 1:
                    self.machine.stop("reset_done_callback_repeated", now)
                    self._persist(force=True)
                    return
                response = future.result()
            except Exception as exc:
                self.machine.reset_call_detail.update({
                    "status": "done_callback_exception",
                    "exception_type": type(exc).__name__,
                    "exception_message": str(exc),
                })
                self.machine.stop(f"reset_service_exception:{type(exc).__name__}:{exc}", now)
            else:
                if response is None:
                    self.machine.reset_call_detail["status"] = "response_missing"
                    self.machine.stop("reset_service_no_response", now)
                else:
                    self.machine.reset_response(bool(response.success), str(response.message), now)
            try:
                self._drive(now)
            except Exception as exc:
                self.machine.stop(
                    f"done_callback_exception:{type(exc).__name__}:{exc}", now
                )
            self._persist(force=True)

    node = None
    rclpy.init()
    lifecycle = {"node_destroyed": False, "rclpy_shutdown": False}
    try:
        node = ActiveResetProbeNode()
        while rclpy.ok() and node.machine.phase != "STOP":
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.machine.phase == "PROVISIONAL_COMPLETE" and (
                time.monotonic()
                - node.machine.timestamps["provisional_complete"]
                >= PROVISIONAL_MONITOR_S
            ):
                break
    except Exception as exc:
        if node is None:
            raise
        node.machine.stop(
            f"spin_exception:{type(exc).__name__}:{exc}", time.monotonic()
        )
    finally:
        if node is not None:
            lifecycle = teardown_ros_node(node, rclpy)
        else:
            rclpy.shutdown()
    assert node is not None
    status, document = finalize_probe_output(
        node.machine,
        args.output,
        lifecycle,
        persistence_failed=node._persistence_failed,
    )
    print(json.dumps(document, sort_keys=True))
    return status


def main(argv: list[str] | None = None) -> int:
    """Run one active-reset probe and return zero only on PASS."""
    args = _arguments(argv)
    try:
        return _run_ros(args)
    except Exception as exc:
        document = {
            "schema": "bio_nav.active_reset_probe.v1",
            "phase": "STOP",
            "verdict": "STOP",
            "stop_reason": f"entrypoint_exception:{type(exc).__name__}:{exc}",
            "claim_boundary": {
                "engineering_pass": False,
                "requires_external_bag_order_analysis": True,
            },
        }
        try:
            atomic_write_json(args.output, document)
        except Exception as write_exc:
            document["terminal_json_write_error"] = (
                f"{type(write_exc).__name__}:{write_exc}"
            )
            emergency = _best_effort_emergency_stop(
                args.output, document["stop_reason"]
            )
            if emergency.get("emergency_write_errors"):
                document["emergency_write_errors"] = emergency[
                    "emergency_write_errors"
                ]
        print(json.dumps(document, sort_keys=True))
        return 20


if __name__ == "__main__":
    raise SystemExit(main())
