"""Minimal Phase-G cognitive-graph causal runner and paired evaluator.

The live runner deliberately reuses :class:`V6FormalNode`.  One reset is
followed by the Phase-B full-house loop three times: loops 1--2 warm the CPG
and loop 3 is scored.  Ground Truth remains recorder/evaluator-only.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, replace
import json
import math
from pathlib import Path
import sys
import time
from typing import Any, Mapping, Sequence

import yaml

from .v6_formal import (
    ENGINEERING_PILOT,
    NOT_QUALIFIED,
    Manifest,
    MissionLeg,
    V6ContractError,
    V6FormalNode,
    append_evidence_jsonl,
    load_manifest as load_phase_b_manifest,
)


CONFIG_SCHEMA = "bio_nav_v6_phase_g_causal_v1"
RESULT_SCHEMA = "bio_nav_v6_phase_g_result_v1"
SUMMARY_SCHEMA = "bio_nav_v6_phase_g_summary_v1"
ARMS = ("G0", "G1", "G2", "G3")
GRAPH_MODE_BY_ARM = {
    "G0": "gvg",
    "G1": "shadow",
    "G2": "hybrid",
    "G3": "primary",
}
EXPECTED_SELECTED_KIND = {
    "G0": "gvg",
    "G1": "gvg",
    "G2": "hybrid",
    "G3": "primary",
}
ROUTE_PRIOR_BY_ARM = {"G0": False, "G1": False, "G2": True, "G3": True}
LOOP_COUNT = 3
WARMUP_LOOP_COUNT = 2
SCORING_LOOP_INDEX = 2
LOOP_ROUTE_IDS = ("G2", "G3", "G4", "G5", "G1")
OBSTACLE_ARMS = ("M3", "M2")
PARETO_METRICS = ("path_length_m", "duration_s", "replans", "fallback_count")
DEFAULT_SELECTION_THRESHOLDS = {metric: 0.0 for metric in PARETO_METRICS}
GT_PREFIX = "/" + "ground_truth/"


class PhaseGConfigError(V6ContractError):
    """The compact Phase-G config or result set is invalid."""


def _mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise PhaseGConfigError(f"{name} must be a mapping")
    return value


@dataclass(frozen=True)
class ArmConfig:
    graph_mode: str
    expected_selected_kind: str
    route_prior_enabled: bool


@dataclass(frozen=True)
class PhaseGConfig:
    path: Path
    phase_b_manifest: Path
    arms: Mapping[str, ArmConfig]
    route_ids: tuple[str, ...]
    loops_total: int
    warmup_loops: int
    scoring_loop: int
    reset_count: int
    no_reset_between_loops: bool
    default_obstacle_arm: str
    fallback_obstacle_arm: str
    selection_thresholds: Mapping[str, float]
    dynamic_actor_metrics: Mapping[str, Any]


@dataclass(frozen=True)
class TimelineLeg:
    loop_index: int
    loop_number: int
    role: str
    leg_index: int
    route_id: str


def load_config(path: str | Path) -> PhaseGConfig:
    config_path = Path(path).expanduser().resolve()
    raw = _mapping(
        yaml.safe_load(config_path.read_text(encoding="utf-8")), "config"
    )
    expected = {
        "schema_version",
        "phase_b_manifest",
        "arms",
        "experiment",
        "dynamic_actor_metrics",
    }
    if set(raw) != expected:
        raise PhaseGConfigError(f"config keys must be {sorted(expected)}")
    if raw.get("schema_version") != CONFIG_SCHEMA:
        raise PhaseGConfigError(f"schema_version must be {CONFIG_SCHEMA}")

    manifest_value = str(raw.get("phase_b_manifest", "")).strip()
    if not manifest_value:
        raise PhaseGConfigError("phase_b_manifest must be non-empty")
    manifest_path = Path(manifest_value).expanduser()
    if not manifest_path.is_absolute():
        manifest_path = config_path.parent / manifest_path
    manifest_path = manifest_path.resolve()

    arms_raw = _mapping(raw.get("arms"), "arms")
    if tuple(arms_raw) != ARMS:
        raise PhaseGConfigError(f"arms must be ordered {ARMS}")
    arms: dict[str, ArmConfig] = {}
    for name in ARMS:
        row = _mapping(arms_raw[name], f"arms.{name}")
        if set(row) != {
            "graph_mode",
            "expected_selected_kind",
            "route_prior_enabled",
        }:
            raise PhaseGConfigError(f"arms.{name} has unexpected keys")
        arm = ArmConfig(
            graph_mode=str(row["graph_mode"]),
            expected_selected_kind=str(row["expected_selected_kind"]),
            route_prior_enabled=bool(row["route_prior_enabled"]),
        )
        if arm.graph_mode != GRAPH_MODE_BY_ARM[name]:
            raise PhaseGConfigError(f"arms.{name}.graph_mode mismatch")
        if arm.expected_selected_kind != EXPECTED_SELECTED_KIND[name]:
            raise PhaseGConfigError(
                f"arms.{name}.expected_selected_kind mismatch"
            )
        if arm.route_prior_enabled != ROUTE_PRIOR_BY_ARM[name]:
            raise PhaseGConfigError(f"arms.{name}.route_prior_enabled mismatch")
        arms[name] = arm

    experiment = _mapping(raw.get("experiment"), "experiment")
    required_experiment = {
        "route_ids",
        "loops_total",
        "warmup_loops",
        "scoring_loop",
        "reset_count",
        "no_reset_between_loops",
        "default_obstacle_arm",
        "fallback_obstacle_arm",
        "selection_thresholds",
    }
    if set(experiment) != required_experiment:
        raise PhaseGConfigError(
            f"experiment keys must be {sorted(required_experiment)}"
        )
    route_ids = tuple(str(value) for value in experiment["route_ids"])
    if route_ids != LOOP_ROUTE_IDS:
        raise PhaseGConfigError(f"experiment.route_ids must be {LOOP_ROUTE_IDS}")
    fixed = {
        "loops_total": LOOP_COUNT,
        "warmup_loops": WARMUP_LOOP_COUNT,
        "scoring_loop": SCORING_LOOP_INDEX + 1,
        "reset_count": 1,
        "no_reset_between_loops": True,
        "default_obstacle_arm": "M3",
        "fallback_obstacle_arm": "M2",
    }
    for name, expected_value in fixed.items():
        if experiment.get(name) != expected_value:
            raise PhaseGConfigError(
                f"experiment.{name} must be {expected_value!r}"
            )
    thresholds = _mapping(
        experiment.get("selection_thresholds"),
        "experiment.selection_thresholds",
    )
    if set(thresholds) != set(PARETO_METRICS):
        raise PhaseGConfigError(
            f"selection thresholds must be {sorted(PARETO_METRICS)}"
        )
    selection_thresholds: dict[str, float] = {}
    for metric in PARETO_METRICS:
        value = thresholds[metric]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise PhaseGConfigError(
                f"selection threshold {metric} must be a non-negative number"
            )
        threshold = float(value)
        if not math.isfinite(threshold) or threshold < 0.0:
            raise PhaseGConfigError(
                f"selection threshold {metric} must be a non-negative number"
            )
        selection_thresholds[metric] = threshold

    dynamic = _mapping(raw.get("dynamic_actor_metrics"), "dynamic_actor_metrics")
    if set(dynamic) != {"enabled", "source_json_by_arm"}:
        raise PhaseGConfigError("dynamic_actor_metrics has unexpected keys")
    sources = _mapping(
        dynamic.get("source_json_by_arm"),
        "dynamic_actor_metrics.source_json_by_arm",
    )
    if any(str(name) not in ARMS for name in sources):
        raise PhaseGConfigError("dynamic metric source has unknown arm")

    return PhaseGConfig(
        path=config_path,
        phase_b_manifest=manifest_path,
        arms=arms,
        route_ids=route_ids,
        loops_total=LOOP_COUNT,
        warmup_loops=WARMUP_LOOP_COUNT,
        scoring_loop=SCORING_LOOP_INDEX + 1,
        reset_count=1,
        no_reset_between_loops=True,
        default_obstacle_arm="M3",
        fallback_obstacle_arm="M2",
        selection_thresholds=selection_thresholds,
        dynamic_actor_metrics=dict(dynamic),
    )


def build_timeline(route_ids: Sequence[str]) -> tuple[TimelineLeg, ...]:
    route = tuple(str(value) for value in route_ids)
    if route != LOOP_ROUTE_IDS:
        raise PhaseGConfigError(f"route must be {LOOP_ROUTE_IDS}")
    return tuple(
        TimelineLeg(
            loop_index=loop_index,
            loop_number=loop_index + 1,
            role="scoring" if loop_index == SCORING_LOOP_INDEX else "warmup",
            leg_index=leg_index,
            route_id=route_id,
        )
        for loop_index in range(LOOP_COUNT)
        for leg_index, route_id in enumerate(route)
    )


def graph_kind(graph_id: str) -> str:
    value = str(graph_id).strip()
    if ":hybrid:" in value:
        return "hybrid"
    if value.endswith(":primary"):
        return "primary"
    if ":gvg_" in value or value.endswith(":gvg"):
        return "gvg"
    return "unknown"


def candidate_is_mature(candidate: Mapping[str, Any]) -> bool:
    return bool(
        int(candidate.get("node_count", 0)) >= 2
        and int(candidate.get("edge_count", 0)) >= 1
        and candidate.get("all_edge_ids_nonempty", False)
    )


def candidate_is_current_trusted(
    candidate: Mapping[str, Any], *, reset_epoch: int
) -> bool:
    return bool(
        candidate_is_mature(candidate)
        and int(candidate.get("reset_epoch", -1)) == int(reset_epoch)
        and candidate.get("module2_healthy") is True
        and candidate.get("observation_valid") is True
        and candidate.get("trusted_write") is True
        and int(candidate.get("rejection_mask", -1)) == 0
    )


def causal_contrast_status(
    arm: str,
    selected_graph_id: str,
    candidates: Sequence[Mapping[str, Any]],
    validation_acks: Sequence[Mapping[str, Any]] = (),
    *,
    reset_epoch: int | None = None,
) -> tuple[bool, tuple[str, ...]]:
    if arm not in ARMS:
        raise PhaseGConfigError(f"unknown arm {arm}")
    reasons: list[str] = []
    actual_kind = graph_kind(selected_graph_id)
    expected_kind = EXPECTED_SELECTED_KIND[arm]
    if actual_kind != expected_kind:
        reasons.append(
            f"selected_graph_kind:{actual_kind}!={expected_kind}"
        )
    if arm in {"G2", "G3"}:
        if reset_epoch is None:
            reasons.append("current_reset_epoch_missing")
            trusted: list[Mapping[str, Any]] = []
        else:
            trusted = [
                item
                for item in candidates
                if candidate_is_current_trusted(item, reset_epoch=reset_epoch)
            ]
            if not trusted:
                reasons.append("current_trusted_mature_cognitive_graph_missing")
        matched = False
        for candidate in trusted:
            matched = any(
                ack.get("accepted") is True
                and int(ack.get("reset_epoch", -1)) == int(reset_epoch)
                and ack.get("candidate_graph_id") == candidate.get("graph_id")
                and int(ack.get("candidate_topology_revision", -1))
                == int(candidate.get("topology_revision", -2))
                and int(ack.get("candidate_value_sequence", -1))
                == int(candidate.get("value_sequence", -2))
                and ack.get("validated_graph_id") == selected_graph_id
                for ack in validation_acks
            )
            if matched:
                break
        if trusted and not matched:
            reasons.append("matching_accepted_validation_missing")
    return not reasons, tuple(reasons)


def scoring_route_contrast_status(
    arm: str, routes: Sequence[Mapping[str, Any]]
) -> tuple[bool, tuple[str, ...]]:
    if arm not in ARMS:
        raise PhaseGConfigError(f"unknown arm {arm}")
    expected = EXPECTED_SELECTED_KIND[arm]
    wrong = sorted(
        {
            graph_kind(str(route.get("graph_id", "")))
            for route in routes
            if graph_kind(str(route.get("graph_id", ""))) != expected
        }
    )
    if wrong:
        return False, (f"scoring_canonical_route_graph_kind:{'+'.join(wrong)}!={expected}",)
    return True, ()


def _stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _dynamic_metrics(config: PhaseGConfig, arm: str) -> dict[str, Any]:
    placeholder = {
        "configured": False,
        "source_json": None,
        "actor_id": None,
        "minimum_clearance_m": None,
        "time_to_yield_s": None,
        "stale_residue_s": None,
        "collision": None,
    }
    dynamic = config.dynamic_actor_metrics
    if not bool(dynamic.get("enabled", False)):
        return placeholder
    source_value = str(
        _mapping(dynamic.get("source_json_by_arm"), "source_json_by_arm").get(
            arm, ""
        )
    ).strip()
    if not source_value:
        return {**placeholder, "configured": True}
    source = Path(source_value).expanduser()
    if not source.is_absolute():
        source = config.path.parent / source
    source = source.resolve()
    try:
        payload = _mapping(
            json.loads(source.read_text(encoding="utf-8")),
            f"dynamic metrics {source}",
        )
    except (OSError, json.JSONDecodeError) as exc:
        raise PhaseGConfigError(f"dynamic metrics unreadable: {source}: {exc}") from exc
    return {
        **placeholder,
        **dict(payload),
        "configured": True,
        "source_json": str(source),
    }


def phase_g_manifest(
    config: PhaseGConfig, arm: str = "G0", obstacle_arm: str = "M3"
) -> Manifest:
    if arm not in ARMS:
        raise PhaseGConfigError(f"unknown arm {arm}")
    if obstacle_arm not in OBSTACLE_ARMS:
        raise PhaseGConfigError(f"unknown obstacle arm {obstacle_arm}")
    base = load_phase_b_manifest(config.phase_b_manifest)
    if tuple(leg.goal_id for leg in base.mission_legs) != config.route_ids:
        raise PhaseGConfigError("Phase-B manifest full-house route mismatch")
    repeated = tuple(base.mission_legs) * config.loops_total
    runtime = dict(base.runtime)
    runtime.update(
        {
            "module2_navigation_write_enabled": True,
            "cognitive_place_graph_enabled": arm != "G0",
            "route_backend": EXPECTED_SELECTED_KIND[arm],
            "low_obstacles_enabled": True,
            "dynamic_actors_enabled": False,
            "cognitive_profile": obstacle_arm,
            "cognitive_graph_mode": GRAPH_MODE_BY_ARM[arm],
            "route_prior_enabled": ROUTE_PRIOR_BY_ARM[arm],
            "obstacle_arm": obstacle_arm,
            "phase_g_three_loop_protocol": "warmup,warmup,scoring",
        }
    )
    return replace(base, runtime=runtime, mission_legs=repeated)


class PhaseGCausalNode(V6FormalNode):
    """One-reset, three-loop adapter with graph-causal observability."""

    def __init__(
        self,
        manifest: Manifest,
        episode: Any,
        output_jsonl: Path,
        *,
        config: PhaseGConfig,
        arm: str,
        obstacle_arm: str,
    ) -> None:
        if arm not in ARMS:
            raise PhaseGConfigError(f"unknown arm {arm}")
        if obstacle_arm not in OBSTACLE_ARMS:
            raise PhaseGConfigError(f"unknown obstacle arm {obstacle_arm}")
        super().__init__(
            manifest,
            episode,
            output_jsonl,
            qualification=ENGINEERING_PILOT,
        )
        from bio_nav_interfaces.msg import (
            CognitiveGraphValidationAck,
            CognitivePlaceGraphCandidate,
            NavigationGraph,
            RouteEdgeCostArray,
            SRDREdgeDiagnosticArray,
            StructuralGraphStatus,
        )
        from nav_msgs.msg import Path as NavPath
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

        self.config = config
        self.arm = arm
        self.obstacle_arm = obstacle_arm
        self.timeline = build_timeline(config.route_ids)
        self.graph_history: list[dict[str, Any]] = []
        self.graph_switch_history: list[dict[str, Any]] = []
        self.candidate_history: list[dict[str, Any]] = []
        self.validation_history: list[dict[str, Any]] = []
        self.canonical_route_history: list[dict[str, Any]] = []
        self.srdr_history: list[dict[str, Any]] = []
        self.route_cost_history: list[dict[str, Any]] = []
        self.structural_status_history: list[dict[str, Any]] = []
        self.loop_results = [self._new_loop(index) for index in range(LOOP_COUNT)]
        self._active_loop_index: int | None = None
        self._loop_last_xy: tuple[float, float] | None = None
        self._loop_started_monotonic: float | None = None
        self._invalid_contrast_reasons: tuple[str, ...] = ()

        latched = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        reliable = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscriptions.extend(
            [
                self.node.create_subscription(
                    NavigationGraph,
                    "/bio_nav/navigation_graph",
                    self._phase_g_graph,
                    latched,
                ),
                self.node.create_subscription(
                    CognitivePlaceGraphCandidate,
                    "/bio_nav/module2/cognitive_place_graph",
                    self._phase_g_candidate,
                    latched,
                ),
                self.node.create_subscription(
                    CognitiveGraphValidationAck,
                    "/bio_nav/module3/cognitive_graph_validation_ack",
                    self._phase_g_validation,
                    reliable,
                ),
                self.node.create_subscription(
                    StructuralGraphStatus,
                    "/bio_nav/structural_graph_status",
                    self._phase_g_structural_status,
                    latched,
                ),
                self.node.create_subscription(
                    SRDREdgeDiagnosticArray,
                    "/bio_nav/module2/srdr_edge_diagnostics",
                    self._phase_g_srdr,
                    latched,
                ),
                self.node.create_subscription(
                    RouteEdgeCostArray,
                    "/bio_nav/route_edge_costs",
                    self._phase_g_route_cost,
                    latched,
                ),
                self.node.create_subscription(
                    NavPath, "/plan", self._phase_g_plan, reliable
                ),
            ]
        )

    @staticmethod
    def _new_loop(index: int) -> dict[str, Any]:
        return {
            "loop_index": index,
            "loop_number": index + 1,
            "role": "scoring" if index == SCORING_LOOP_INDEX else "warmup",
            "started": False,
            "completed": False,
            "completed_leg_ids": [],
            "path_length_m": 0.0,
            "duration_s": None,
            "replans": 0,
            "selected_graph_ids": [],
            "canonical_routes": [],
        }

    def _loop_of_current_leg(self) -> int | None:
        publications = int(self.guard.goal_publications)
        if publications <= 0:
            return self._active_loop_index
        return min((publications - 1) // len(LOOP_ROUTE_IDS), SCORING_LOOP_INDEX)

    def _phase_g_graph(self, message: Any) -> None:
        record = {
            "stamp_ns": _stamp_ns(message.header.stamp),
            "graph_id": str(message.graph_id),
            "revision": int(message.revision),
            "map_version": str(message.map_version),
            "node_count": len(message.nodes),
            "edge_count": len(message.edges),
            "kind": graph_kind(str(message.graph_id)),
            "loop_index": self._active_loop_index,
        }
        if not self.graph_history or record != self.graph_history[-1]:
            if self.graph_history and (
                record["graph_id"], record["revision"]
            ) != (
                self.graph_history[-1]["graph_id"],
                self.graph_history[-1]["revision"],
            ):
                self.graph_switch_history.append(
                    {
                        "from_graph_id": self.graph_history[-1]["graph_id"],
                        "to_graph_id": record["graph_id"],
                        "to_revision": record["revision"],
                        "loop_index": self._active_loop_index,
                    }
                )
            self.graph_history.append(record)
            if self._active_loop_index is not None:
                ids = self.loop_results[self._active_loop_index][
                    "selected_graph_ids"
                ]
                if not ids or ids[-1] != record["graph_id"]:
                    ids.append(record["graph_id"])
        self._write("phase_g_navigation_graph", **record)
        if (
            self._active_loop_index == SCORING_LOOP_INDEX
            and record["kind"] != EXPECTED_SELECTED_KIND[self.arm]
        ):
            self._invalidate_contrast(
                f"scoring_graph_kind:{record['kind']}!={EXPECTED_SELECTED_KIND[self.arm]}"
            )

    def _phase_g_candidate(self, message: Any) -> None:
        edge_ids = [str(item.edge_id).strip() for item in message.edges]
        record = {
            "stamp_ns": _stamp_ns(message.header.stamp),
            "source_sequence": int(message.source_sequence),
            "recurrent_session_id": str(message.recurrent_session_id),
            "graph_id": str(message.graph_id),
            "topology_revision": int(message.topology_revision),
            "value_sequence": int(message.value_sequence),
            "map_version": str(message.map_version),
            "reset_epoch": int(message.reset_epoch),
            "cognitive_tile_id": str(message.cognitive_tile_id),
            "tile_revision": int(message.tile_revision),
            "source_physical_graph_id": str(message.source_physical_graph_id),
            "source_physical_graph_revision": int(
                message.source_physical_graph_revision
            ),
            "node_count": len(message.nodes),
            "edge_count": len(message.edges),
            "all_edge_ids_nonempty": bool(edge_ids) and all(edge_ids),
            "mature": len(message.nodes) >= 2 and bool(edge_ids) and all(edge_ids),
            "module2_healthy": bool(message.module2_healthy),
            "observation_valid": bool(message.observation_valid),
            "trusted_write": bool(message.trusted_write),
            "rejection_mask": int(message.rejection_mask),
            "loop_index": self._active_loop_index,
        }
        if not self.candidate_history or record != self.candidate_history[-1]:
            self.candidate_history.append(record)
            self.candidate_history = self.candidate_history[-200:]
        self._write("phase_g_cognitive_graph_candidate", **record)

    def _phase_g_validation(self, message: Any) -> None:
        record = {
            "stamp_ns": _stamp_ns(message.header.stamp),
            "recurrent_session_id": str(message.recurrent_session_id),
            "reset_epoch": int(message.reset_epoch),
            "generation": int(message.generation),
            "candidate_graph_id": str(message.candidate_graph_id),
            "candidate_topology_revision": int(message.candidate_topology_revision),
            "candidate_value_sequence": int(message.candidate_value_sequence),
            "candidate_edge_id": str(message.candidate_edge_id),
            "validated_graph_id": str(message.validated_graph_id),
            "validated_graph_revision": int(message.validated_graph_revision),
            "validated_edge_id": str(message.validated_edge_id),
            "event_sequence": int(message.event_sequence),
            "accepted": bool(message.accepted),
            "reason": str(message.reason),
            "reroute_revision": int(message.reroute_revision),
            "reroute_applied": bool(message.reroute_applied),
            "loop_index": self._active_loop_index,
        }
        self.validation_history.append(record)
        self.validation_history = self.validation_history[-1000:]
        self._write("phase_g_cognitive_graph_validation", **record)

    def _phase_g_structural_status(self, message: Any) -> None:
        record = {
            "stamp_ns": _stamp_ns(message.header.stamp),
            "graph_id": str(message.graph_id),
            "graph_revision": int(message.graph_revision),
            "state": int(message.state),
            "detail": str(message.detail),
            "fallback": "fallback" in str(message.detail).lower(),
            "loop_index": self._active_loop_index,
        }
        if not self.structural_status_history or record != self.structural_status_history[-1]:
            self.structural_status_history.append(record)
        self._write("phase_g_structural_status", **record)

    def _phase_g_srdr(self, message: Any) -> None:
        diagnostics = list(message.diagnostics)
        record = {
            "stamp_ns": _stamp_ns(message.header.stamp),
            "request_id": int(message.request_id),
            "graph_id": str(message.graph_id),
            "graph_revision": int(message.graph_revision),
            "model_id": str(message.model_id),
            "diagnostic_count": len(diagnostics),
            "usable_count": sum(bool(item.usable) for item in diagnostics),
            "positive_sr_count": sum(float(item.sr_penalty_m) > 0.0 for item in diagnostics),
            "positive_dr_count": sum(float(item.dr_penalty_m) > 0.0 for item in diagnostics),
            "rejection_reasons": sorted(
                {str(item.rejection_reason) for item in diagnostics if str(item.rejection_reason)}
            ),
            "loop_index": self._active_loop_index,
        }
        self.srdr_history.append(record)
        self.srdr_history = self.srdr_history[-1000:]
        self._write("phase_g_srdr_diagnostics", **record)

    def _phase_g_route_cost(self, message: Any) -> None:
        costs = list(message.costs)
        record = {
            "stamp_ns": _stamp_ns(message.header.stamp),
            "request_id": int(message.request_id),
            "graph_id": str(message.graph_id),
            "graph_revision": int(message.graph_revision),
            "edge_count": len(costs),
            "positive_requested_count": sum(
                float(item.requested_module2_delta_m) > 0.0 for item in costs
            ),
            "positive_applied_count": sum(
                float(item.applied_module2_delta_m) > 0.0 for item in costs
            ),
            "blocked_count": sum(bool(item.blocked) for item in costs),
            "loop_index": self._active_loop_index,
        }
        self.route_cost_history.append(record)
        self.route_cost_history = self.route_cost_history[-1000:]
        self._write("phase_g_route_edge_costs", **record)

    def _phase_g_plan(self, _message: Any) -> None:
        if self._active_loop_index is not None:
            self.loop_results[self._active_loop_index]["replans"] += 1

    def _canonical_route(self, message: Any) -> None:
        super()._canonical_route(message)
        record = {
            "request_id": int(message.request_id),
            "graph_id": str(message.graph_id),
            "graph_revision": int(message.graph_revision),
            "node_ids": [int(value) for value in message.node_ids],
            "edge_ids": [int(value) for value in message.edge_ids],
            "total_cost_m": float(message.total_cost_m),
            "loop_index": self._active_loop_index,
        }
        self.canonical_route_history.append(record)
        if self._active_loop_index is not None:
            self.loop_results[self._active_loop_index]["canonical_routes"].append(record)
        self._write("phase_g_canonical_route", **record)
        if self._active_loop_index == SCORING_LOOP_INDEX:
            valid, reasons = scoring_route_contrast_status(self.arm, [record])
            if not valid:
                self._invalidate_contrast(*reasons)

    def _odom(self, message: Any) -> None:
        x = float(message.pose.pose.position.x)
        y = float(message.pose.pose.position.y)
        if self._active_loop_index is not None and self._loop_last_xy is not None:
            step = math.hypot(x - self._loop_last_xy[0], y - self._loop_last_xy[1])
            if math.isfinite(step) and step <= 2.0:
                self.loop_results[self._active_loop_index]["path_length_m"] += step
        if self._active_loop_index is not None:
            self._loop_last_xy = (x, y)
        super()._odom(message)

    def _start_loop(self, loop_index: int) -> bool:
        if loop_index == SCORING_LOOP_INDEX:
            selected = self.graph_history[-1]["graph_id"] if self.graph_history else ""
            valid, reasons = causal_contrast_status(
                self.arm,
                selected,
                self.candidate_history,
                self.validation_history,
                reset_epoch=self.guard.reset_events,
            )
            if not valid:
                self._invalidate_contrast(*reasons, selected_graph_id=selected)
                return False
        self._active_loop_index = loop_index
        self._loop_last_xy = None
        self._loop_started_monotonic = time.monotonic()
        row = self.loop_results[loop_index]
        row["started"] = True
        self._write(
            "phase_g_loop_start",
            loop_index=loop_index,
            loop_number=loop_index + 1,
            role=row["role"],
            reset_calls=self.guard.reset_calls,
            reset_events=self.guard.reset_events,
        )
        return True

    def _invalidate_contrast(
        self, *reasons: str, selected_graph_id: str | None = None
    ) -> None:
        combined = list(self._invalid_contrast_reasons)
        for reason in reasons:
            if reason and reason not in combined:
                combined.append(reason)
        self._invalid_contrast_reasons = tuple(combined)
        self.guard.stop(
            "INVALID_NO_CAUSAL_CONTRAST:" + ",".join(self._invalid_contrast_reasons)
        )
        self._write(
            "phase_g_invalid_no_causal_contrast",
            arm=self.arm,
            selected_graph_id=(
                selected_graph_id
                if selected_graph_id is not None
                else self.graph_history[-1]["graph_id"] if self.graph_history else ""
            ),
            reasons=list(self._invalid_contrast_reasons),
        )

    def _finish_loop(self, loop_index: int) -> None:
        row = self.loop_results[loop_index]
        row["completed"] = bool(
            self.guard.state in {"LEG_SUCCEEDED", "SUCCEEDED"}
            and len(row["completed_leg_ids"]) == len(LOOP_ROUTE_IDS)
        )
        if self._loop_started_monotonic is not None:
            row["duration_s"] = max(
                0.0, time.monotonic() - self._loop_started_monotonic
            )
        self._write("phase_g_loop_end", **row)
        self._active_loop_index = None
        self._loop_last_xy = None
        self._loop_started_monotonic = None

    def _run_mission_leg(
        self,
        *,
        index: int,
        leg: MissionLeg,
        dynamic_group: str,
        reset_timeout_sec: float,
        navigation_timeout_sec: float,
    ) -> None:
        loop_index, leg_index = divmod(index, len(LOOP_ROUTE_IDS))
        if leg_index == 0 and not self._start_loop(loop_index):
            return
        completed_before = len(self.guard.completed_leg_ids)
        super()._run_mission_leg(
            index=index,
            leg=leg,
            dynamic_group=dynamic_group,
            reset_timeout_sec=reset_timeout_sec,
            navigation_timeout_sec=navigation_timeout_sec,
        )
        if len(self.guard.completed_leg_ids) > completed_before:
            self.loop_results[loop_index]["completed_leg_ids"].append(leg.goal_id)
        if leg_index == len(LOOP_ROUTE_IDS) - 1 or self.guard.state not in {
            "LEG_SUCCEEDED",
            "SUCCEEDED",
        }:
            self._finish_loop(loop_index)

    def result(self) -> dict[str, Any]:
        base = super().result()
        total_fallback_count = sum(
            bool(row.get("fallback")) for row in self.structural_status_history
        )
        fallback_count = sum(
            bool(row.get("fallback"))
            and row.get("loop_index") == SCORING_LOOP_INDEX
            for row in self.structural_status_history
        )
        scoring = dict(self.loop_results[SCORING_LOOP_INDEX])
        route_success = bool(
            scoring["completed"] and scoring["completed_leg_ids"] == list(LOOP_ROUTE_IDS)
        )
        invalid = bool(self._invalid_contrast_reasons)
        result = {
            "schema_version": RESULT_SCHEMA,
            "qualification": "ENGINEERING_CAUSAL_RUN",
            "formal_qualification": NOT_QUALIFIED,
            "verdict": (
                "INVALID_NO_CAUSAL_CONTRAST"
                if invalid
                else "RUN_COMPLETE" if route_success else "RUN_FAILED"
            ),
            "arm": self.arm,
            "graph_mode": GRAPH_MODE_BY_ARM[self.arm],
            "expected_selected_kind": EXPECTED_SELECTED_KIND[self.arm],
            "route_prior_enabled": ROUTE_PRIOR_BY_ARM[self.arm],
            "obstacle_arm": self.obstacle_arm,
            "pair_identity": {
                "scene_id": self.manifest.scene_id,
                "seed": int(self.episode.seed),
                "route_ids": list(self.config.route_ids),
                "loops": ["warmup", "warmup", "scoring"],
                "reset_count": 1,
            },
            "state": base["state"],
            "stop_reason": base["stop_reason"],
            "invalid_contrast_reasons": list(self._invalid_contrast_reasons),
            "reset_calls": base["reset_calls"],
            "reset_events": base["reset_events"],
            "no_reset_between_loops": bool(
                base["reset_calls"] == 1 and base["reset_events"] == 1
            ),
            "loops": self.loop_results,
            "scoring": {
                **scoring,
                "route_success": route_success,
                "collision": bool(base["collision"]),
                "terminal_zero_confirmed": bool(base["terminal_zero_confirmed"]),
                "fallback_count": fallback_count,
            },
            "graph_history": self.graph_history,
            "graph_switch_history": self.graph_switch_history,
            "candidate_graphs": self.candidate_history,
            "validation_acks": self.validation_history,
            "canonical_routes": self.canonical_route_history,
            "srdr_diagnostics": self.srdr_history,
            "route_edge_costs": self.route_cost_history,
            "structural_status": self.structural_status_history,
            "total_fallback_count": total_fallback_count,
            "dynamic_actor_metrics": _dynamic_metrics(self.config, self.arm),
        }
        append_evidence_jsonl(self.output_jsonl, "phase_g_result", **result)
        return result


def _read_result(path: str | Path) -> dict[str, Any]:
    source = Path(path).expanduser().resolve()
    selected: dict[str, Any] | None = None
    for line in source.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        row = json.loads(line)
        if row.get("event") == "phase_g_result":
            selected = dict(row)
    if selected is None:
        raise PhaseGConfigError(f"no phase_g_result in {source}")
    return selected


def result_is_eligible(result: Mapping[str, Any]) -> bool:
    score = _mapping(result.get("scoring"), "result.scoring")
    return bool(
        result.get("verdict") == "RUN_COMPLETE"
        and result.get("state") == "SUCCEEDED"
        and result.get("reset_calls") == 1
        and result.get("reset_events") == 1
        and result.get("no_reset_between_loops") is True
        and score.get("completed") is True
        and score.get("completed_leg_ids") == list(LOOP_ROUTE_IDS)
        and score.get("route_success") is True
        and score.get("collision") is False
        and score.get("terminal_zero_confirmed") is True
    )


def pareto_direction(
    candidate: Mapping[str, Any],
    baseline: Mapping[str, Any],
    selection_thresholds: Mapping[str, float] | None = None,
) -> dict[str, Any]:
    thresholds = dict(DEFAULT_SELECTION_THRESHOLDS)
    if selection_thresholds is not None:
        if set(selection_thresholds) != set(PARETO_METRICS):
            raise PhaseGConfigError(
                f"selection thresholds must be {sorted(PARETO_METRICS)}"
            )
        thresholds = {
            metric: float(selection_thresholds[metric]) for metric in PARETO_METRICS
        }
    candidate_score = _mapping(candidate.get("scoring"), "candidate.scoring")
    baseline_score = _mapping(baseline.get("scoring"), "baseline.scoring")
    directions: dict[str, str] = {}
    improvements: dict[str, float | None] = {}
    for metric in PARETO_METRICS:
        left = candidate_score.get(metric)
        right = baseline_score.get(metric)
        if left is None or right is None:
            directions[metric] = "missing"
            improvements[metric] = None
            continue
        left_value = float(left)
        right_value = float(right)
        improvements[metric] = right_value - left_value
        if left_value < right_value:
            directions[metric] = "better"
        elif left_value > right_value:
            directions[metric] = "worse"
        else:
            directions[metric] = "equal"
    comparable = all(value != "missing" for value in directions.values())
    not_worse = comparable and all(
        value in {"better", "equal"} for value in directions.values()
    )
    strictly_better = any(value == "better" for value in directions.values())
    mixed = comparable and any(value == "better" for value in directions.values()) and any(
        value == "worse" for value in directions.values()
    )
    threshold_met = any(
        improvement is not None
        and improvement > 0.0
        and improvement >= thresholds[metric]
        for metric, improvement in improvements.items()
    )
    return {
        "metrics": directions,
        "improvements": improvements,
        "selection_thresholds": thresholds,
        "comparable": comparable,
        "pareto_improves": bool(not_worse and strictly_better),
        "net_benefit": bool(not_worse and threshold_met),
        "mixed_tradeoff": mixed,
    }


def evaluate_group(
    results: Mapping[str, Mapping[str, Any]],
    selection_thresholds: Mapping[str, float] | None = None,
) -> dict[str, Any]:
    if set(results) != set(ARMS):
        raise PhaseGConfigError(f"results must contain exactly {ARMS}")
    for name in ARMS:
        if results[name].get("schema_version") != RESULT_SCHEMA:
            raise PhaseGConfigError(f"{name} result schema mismatch")
        if results[name].get("arm") != name:
            raise PhaseGConfigError(f"{name} result arm mismatch")
    pair_identities = {
        json.dumps(results[name].get("pair_identity"), sort_keys=True)
        for name in ARMS
    }
    if len(pair_identities) != 1 or results["G0"].get("pair_identity") is None:
        raise PhaseGConfigError(
            "G0-G3 must share one scene/seed/route/loop/reset pair identity"
        )
    obstacle_arms = {str(results[name].get("obstacle_arm", "")) for name in ARMS}
    if len(obstacle_arms) != 1 or next(iter(obstacle_arms)) not in OBSTACLE_ARMS:
        raise PhaseGConfigError(
            "all G0-G3 results must use one whole-group M3 or M2 obstacle arm"
        )
    obstacle_arm = next(iter(obstacle_arms))
    eligibility = {name: result_is_eligible(results[name]) for name in ARMS}
    comparisons = {
        "G3_vs_G0": pareto_direction(
            results["G3"], results["G0"], selection_thresholds
        ),
        "G2_vs_G0": pareto_direction(
            results["G2"], results["G0"], selection_thresholds
        ),
    }
    invalid_contrast = {
        name: results[name].get("verdict") == "INVALID_NO_CAUSAL_CONTRAST"
        for name in ARMS
    }
    selected = "GVG"
    verdict = "GVG_RETAINED"
    if any(invalid_contrast.values()):
        verdict = "INVALID_NO_CAUSAL_CONTRAST"
    elif not all(eligibility.values()):
        verdict = (
            "M3_GROUP_INCOMPLETE_TRY_WHOLE_GROUP_M2"
            if obstacle_arm == "M3"
            else "INVALID_INCOMPLETE_M2_GROUP"
        )
    elif eligibility["G3"] and comparisons["G3_vs_G0"]["net_benefit"]:
        selected = "PRIMARY"
        verdict = "PRIMARY_CANDIDATE"
    elif eligibility["G2"] and comparisons["G2_vs_G0"]["net_benefit"]:
        selected = "HYBRID"
        verdict = "HYBRID_CANDIDATE"
    elif any(
        comparison["mixed_tradeoff"] for comparison in comparisons.values()
    ):
        verdict = "AMBIGUOUS_KEEP_GVG"

    return {
        "schema_version": SUMMARY_SCHEMA,
        "qualification": "ENGINEERING_CAUSAL",
        "formal_qualification": NOT_QUALIFIED,
        "verdict": verdict,
        "selected_graph_mode": selected,
        "obstacle_arm": obstacle_arm,
        "pair_identity": dict(
            _mapping(results["G0"]["pair_identity"], "pair_identity")
        ),
        "whole_group_m2_fallback_allowed": (
            verdict == "M3_GROUP_INCOMPLETE_TRY_WHOLE_GROUP_M2"
        ),
        "mixed_obstacle_arm_switch_forbidden": True,
        "eligibility": eligibility,
        "invalid_contrast": invalid_contrast,
        "comparisons": comparisons,
        "shadow_control": {
            "arm": "G1",
            "eligible": eligibility["G1"],
            "selected_for_navigation": False,
        },
        "raw_scoring": {
            name: dict(_mapping(results[name].get("scoring"), f"{name}.scoring"))
            for name in ARMS
        },
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("config", "plan", "run", "evaluate"))
    parser.add_argument("--config", required=True)
    parser.add_argument("--arm", choices=ARMS)
    parser.add_argument("--obstacle-arm", choices=OBSTACLE_ARMS)
    parser.add_argument("--output-jsonl")
    parser.add_argument("--arm-result", action="append", default=[])
    parser.add_argument("--output-json")
    parser.add_argument("--readiness-timeout-sec", type=float, default=120.0)
    parser.add_argument("--reset-timeout-sec", type=float, default=120.0)
    parser.add_argument("--navigation-timeout-sec", type=float, default=900.0)
    return parser


def _parse_arm_results(values: Sequence[str]) -> dict[str, dict[str, Any]]:
    rows: dict[str, dict[str, Any]] = {}
    for value in values:
        name, separator, path = str(value).partition("=")
        if not separator or name not in ARMS or name in rows or not path:
            raise PhaseGConfigError(
                "--arm-result must be unique G0=PATH ... G3=PATH"
            )
        row = _read_result(path)
        if row.get("arm") != name:
            raise PhaseGConfigError(f"{path} result arm is not {name}")
        rows[name] = row
    return rows


def cli(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        config = load_config(args.config)
        if args.command == "config":
            print(
                json.dumps(
                    {
                        "schema_version": CONFIG_SCHEMA,
                        "phase_b_manifest": str(config.phase_b_manifest),
                        "route_ids": list(config.route_ids),
                        "loops": [
                            {
                                "number": index + 1,
                                "role": "scoring" if index == SCORING_LOOP_INDEX else "warmup",
                            }
                            for index in range(LOOP_COUNT)
                        ],
                        "default_obstacle_arm": config.default_obstacle_arm,
                        "fallback_obstacle_arm": config.fallback_obstacle_arm,
                        "selection_thresholds": dict(config.selection_thresholds),
                    },
                    sort_keys=True,
                )
            )
            return 0
        if args.command == "plan":
            if args.arm is None:
                raise PhaseGConfigError("plan requires --arm")
            arm = config.arms[args.arm]
            print(
                json.dumps(
                    {
                        "arm": args.arm,
                        "graph_mode": arm.graph_mode,
                        "expected_selected_kind": arm.expected_selected_kind,
                        "route_prior_enabled": arm.route_prior_enabled,
                        "timeline": [vars(item) for item in build_timeline(config.route_ids)],
                        "reset_count": 1,
                        "no_reset_between_loops": True,
                    },
                    sort_keys=True,
                )
            )
            return 0
        if args.command == "evaluate":
            rows = _parse_arm_results(args.arm_result)
            summary = evaluate_group(rows, config.selection_thresholds)
            encoded = json.dumps(summary, indent=2, sort_keys=True) + "\n"
            if args.output_json:
                target = Path(args.output_json).expanduser().resolve()
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_text(encoded, encoding="utf-8")
            print(encoded, end="")
            return 0 if summary["verdict"] not in {
                "M3_GROUP_INCOMPLETE_TRY_WHOLE_GROUP_M2",
                "INVALID_INCOMPLETE_M2_GROUP",
                "INVALID_NO_CAUSAL_CONTRAST",
            } else 2

        if args.arm is None or args.output_jsonl is None:
            raise PhaseGConfigError("run requires --arm and --output-jsonl")
        obstacle_arm = args.obstacle_arm or config.default_obstacle_arm
        manifest = phase_g_manifest(config, args.arm, obstacle_arm)
        import rclpy

        rclpy.init(args=None)
        node = PhaseGCausalNode(
            manifest,
            manifest.episodes[0],
            Path(args.output_jsonl).expanduser().resolve(),
            config=config,
            arm=args.arm,
            obstacle_arm=obstacle_arm,
        )
        try:
            result = node.run(
                readiness_timeout_sec=args.readiness_timeout_sec,
                reset_timeout_sec=args.reset_timeout_sec,
                navigation_timeout_sec=args.navigation_timeout_sec,
            )
            print(json.dumps(result, sort_keys=True))
            return 0 if result["verdict"] == "RUN_COMPLETE" else 2
        finally:
            node.destroy()
            rclpy.shutdown()
    except (OSError, ValueError, PhaseGConfigError, yaml.YAMLError) as exc:
        print(f"STOP: {exc}", file=sys.stderr)
        return 2


def main() -> None:
    raise SystemExit(cli())


if __name__ == "__main__":
    main()


__all__ = [
    "ARMS",
    "LOOP_ROUTE_IDS",
    "PhaseGConfigError",
    "build_timeline",
    "candidate_is_mature",
    "causal_contrast_status",
    "evaluate_group",
    "graph_kind",
    "load_config",
    "pareto_direction",
    "result_is_eligible",
    "scoring_route_contrast_status",
]
