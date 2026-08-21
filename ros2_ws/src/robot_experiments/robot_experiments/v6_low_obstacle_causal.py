"""Pure planning and offline evaluation for the V6 M0--M3 low-obstacle study.

This module intentionally does not launch ROS or Isaac.  It freezes the twelve
engineering runs, emits a reviewable external-adapter plan, and evaluates only
recorded evidence.  Ground Truth is an input to the passive evaluator and is
never part of the dispatcher contract.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import statistics
import sys
from typing import Any, Mapping, Sequence

import yaml


SCHEMA_VERSION = "bio_nav_v6_low_obstacle_causal_manifest_v1"
QUALIFICATION = "ENGINEERING_CAUSAL_NOT_RUN"
EXPECTED_ORDER = (
    "M0", "M1", "M2", "M3",
    "M3", "M2", "M1", "M0",
    "M1", "M3", "M0", "M2",
)
GT_PREFIX = "/" + "ground_truth/"

# The runtime dispatcher/recorder is estimated-state only.  Passive Ground
# Truth evidence is captured by a separate process and joined offline.
DISPATCHER_TOPICS = (
    "/odom",
    "/amcl_pose",
    "/scan",
    "/camera/depth/image_rect_raw",
    "/bio_nav/cognitive_obstacles",
    "/bio_nav/cognitive_obstacle_layer/status",
    "/bio_nav/local_risk_layer/status",
    "/bio_nav/cognitive_risk_critic/status",
    "/bio_nav/module2/goal_planning_prior",
    "/global_costmap/costmap",
    "/local_costmap/costmap",
    "/plan",
    "/optimal_trajectory",
    "/cmd_vel",
    "/bio_nav/canonical_route",
    "/bio_nav/route_progress",
    "/bio_nav/route_goal_complete",
)
PASSIVE_EVALUATOR_TOPICS = (
    "/ground_truth/odom",
    "/simulation/collision",
    "/simulation/collision_diagnostics",
)

if any(topic.startswith(GT_PREFIX) for topic in DISPATCHER_TOPICS):
    raise RuntimeError("V6 causal dispatcher Ground Truth firewall violated")


class CausalContractError(RuntimeError):
    """A frozen manifest or recorded-evidence contract violation."""


@dataclass(frozen=True)
class ArmContract:
    name: str
    module2_uds_enabled: bool
    integration_bridge_enabled: bool
    integration_process_required: bool
    localization_contract: str
    module3_mode: str
    obstacle_layer_mode: str
    critic_mode: str


@dataclass(frozen=True)
class RunContract:
    run_id: str
    repeat: int
    arm: str


@dataclass(frozen=True)
class CausalManifest:
    path: Path
    identity: Mapping[str, Any]
    localization_contract: Mapping[str, Any]
    freshness: Mapping[str, Any]
    criteria: Mapping[str, Any]
    arms: Mapping[str, ArmContract]
    runs: tuple[RunContract, ...]


@dataclass(frozen=True)
class RunResult:
    run_id: str
    repeat: int
    arm: str
    verdict: str
    reasons: tuple[str, ...]
    synchronized_frames: int
    scan_invisible_rgbd_pairs: int
    typed_spatial_matches: int
    typed_spatial_total: int
    path_length_m: float
    minimum_clearance_m: float
    collision: bool
    success: bool
    reroute_direction: str
    critic_participation: str
    evidence_file: str


@dataclass(frozen=True)
class PairResult:
    repeat: int
    lhs_arm: str
    rhs_arm: str
    hausdorff_m: float
    length_delta_fraction: float
    clearance_gain_m: float
    direction_consistent: bool


@dataclass(frozen=True)
class CausalSummary:
    qualification: str
    verdict: str
    reasons: tuple[str, ...]
    runs: tuple[RunResult, ...]
    m1_vs_m0: tuple[PairResult, ...]
    m2_vs_m1: tuple[PairResult, ...]
    m3_vs_m1: tuple[PairResult, ...]
    m3_vs_m2: tuple[PairResult, ...]
    visualization_inputs: tuple[Mapping[str, Any], ...]


def _mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise CausalContractError(f"{name} must be a mapping")
    return value


def _bool(value: Any, name: str) -> bool:
    if not isinstance(value, bool):
        raise CausalContractError(f"{name} must be boolean")
    return value


def load_manifest(path: str | Path) -> CausalManifest:
    manifest_path = Path(path).expanduser().resolve()
    raw = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    raw = _mapping(raw, "manifest")
    if raw.get("schema_version") != SCHEMA_VERSION:
        raise CausalContractError(f"schema_version must be {SCHEMA_VERSION}")

    identity = _mapping(raw.get("identity"), "identity")
    fixed_identity = {
        "scene_id": "v6_kujiale_low_obstacles_static",
        "obstacle_layout_id": "kujiale_v6_low_obstacles_frozen_r1_20260820",
        "scene_contract_frozen": True,
        "seed": 8601,
        "timeout_sec": 180.0,
        "route_backend": "primary",
        "graph_backend": "gvg",
        "direct_rgbd_costmap_enabled": False,
        "exactly_once_reset": True,
    }
    for key, expected in fixed_identity.items():
        if identity.get(key) != expected:
            raise CausalContractError(f"identity.{key} must be {expected!r}")
    if _mapping(identity.get("start"), "identity.start").get("id") != "G1":
        raise CausalContractError("identity.start.id must be G1")
    if _mapping(identity.get("goal"), "identity.goal").get("id") != "G2":
        raise CausalContractError("identity.goal.id must be G2")

    localization = _mapping(raw.get("localization_contract"), "localization_contract")
    if localization.get("startup_profile") != "estimated_autonomy":
        raise CausalContractError("localization_contract.startup_profile must be estimated_autonomy")
    if localization.get("preserve_when_module2_disabled") is not True:
        raise CausalContractError("M0 must preserve the estimated-autonomy localization contract")

    raw_arms = _mapping(raw.get("arms"), "arms")
    arms: dict[str, ArmContract] = {}
    for name in ("M0", "M1", "M2", "M3"):
        row = _mapping(raw_arms.get(name), f"arms.{name}")
        arms[name] = ArmContract(
            name=name,
            module2_uds_enabled=_bool(row.get("module2_uds_enabled"), f"arms.{name}.module2_uds_enabled"),
            integration_bridge_enabled=_bool(row.get("integration_bridge_enabled"), f"arms.{name}.integration_bridge_enabled"),
            integration_process_required=_bool(row.get("integration_process_required"), f"arms.{name}.integration_process_required"),
            localization_contract=str(row.get("localization_contract", "")),
            module3_mode=str(row.get("module3_mode", "")),
            obstacle_layer_mode=str(row.get("obstacle_layer_mode", "")),
            critic_mode=str(row.get("critic_mode", "")),
        )
    expected_arms = {
        "M0": (False, False, "M0", "off", "off"),
        "M1": (True, True, "M1", "shadow", "shadow"),
        "M2": (True, True, "M2", "active", "off"),
        "M3": (True, True, "M3", "active", "active"),
    }
    for name, expected in expected_arms.items():
        arm = arms[name]
        actual = (
            arm.module2_uds_enabled,
            arm.integration_bridge_enabled,
            arm.module3_mode,
            arm.obstacle_layer_mode,
            arm.critic_mode,
        )
        if actual != expected:
            raise CausalContractError(f"arms.{name} does not match the frozen arm contract")
        if not arm.integration_process_required or arm.localization_contract != "same_estimated_autonomy":
            raise CausalContractError(f"arms.{name} must keep the same Integration localization contract")

    raw_runs = raw.get("run_order")
    if not isinstance(raw_runs, list) or len(raw_runs) != 12:
        raise CausalContractError("run_order must contain exactly 12 rows")
    runs = tuple(
        RunContract(
            run_id=str(_mapping(row, f"run_order[{index}]").get("run_id", "")),
            repeat=int(_mapping(row, f"run_order[{index}]").get("repeat", 0)),
            arm=str(_mapping(row, f"run_order[{index}]").get("arm", "")),
        )
        for index, row in enumerate(raw_runs)
    )
    if tuple(run.arm for run in runs) != EXPECTED_ORDER:
        raise CausalContractError("run_order arms do not match the frozen counterbalanced order")
    if tuple(run.repeat for run in runs) != (1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3):
        raise CausalContractError("run_order repeats do not match the frozen order")
    if len({run.run_id for run in runs}) != 12 or any(not run.run_id for run in runs):
        raise CausalContractError("run_order requires 12 unique non-empty run_id values")

    capture = _mapping(raw.get("capture"), "capture")
    manifest_dispatcher = tuple(capture.get("dispatcher_topics", ()))
    manifest_passive = tuple(capture.get("passive_evaluator_topics", ()))
    if set(manifest_dispatcher) != set(DISPATCHER_TOPICS):
        raise CausalContractError("capture.dispatcher_topics must match the zero-GT recorder contract")
    if any(str(topic).startswith(GT_PREFIX) for topic in manifest_dispatcher):
        raise CausalContractError("dispatcher Ground Truth firewall violated")
    if set(manifest_passive) != set(PASSIVE_EVALUATOR_TOPICS):
        raise CausalContractError("capture.passive_evaluator_topics must match the passive GT contract")

    freshness = _mapping(raw.get("freshness"), "freshness")
    if float(freshness.get("typed_obstacle_ttl_sec", 0.0)) <= 0.0:
        raise CausalContractError("freshness.typed_obstacle_ttl_sec must be positive")
    if freshness.get("stale_action") != "stop_and_fail_open":
        raise CausalContractError("freshness.stale_action must be stop_and_fail_open")
    criteria = _mapping(raw.get("criteria"), "criteria")
    return CausalManifest(
        path=manifest_path,
        identity=identity,
        localization_contract=localization,
        freshness=freshness,
        criteria=criteria,
        arms=arms,
        runs=runs,
    )


def build_plan(manifest: CausalManifest) -> dict[str, Any]:
    """Return an external-adapter command/state manifest; perform no mutation."""

    rows = []
    for run in manifest.runs:
        arm = manifest.arms[run.arm]
        rows.append({
            "run_id": run.run_id,
            "repeat": run.repeat,
            "arm": run.arm,
            "identity": dict(manifest.identity),
            "setup": {
                "integration_startup_profile": "estimated_autonomy",
                "integration_process_required": True,
                "module2_uds_enabled": arm.module2_uds_enabled,
                "integration_bridge_enabled": arm.integration_bridge_enabled,
                "module3_mode": arm.module3_mode,
                "obstacle_layer_mode": arm.obstacle_layer_mode,
                "critic_mode": arm.critic_mode,
                "graph_backend": "gvg",
                "direct_rgbd_costmap_enabled": False,
            },
            "reset_state_machine": (
                "wait_readiness", "set_seed_8601", "one_reset_call",
                "one_reset_event", "bridge_epoch_plus_one", "localization_seeded",
                "one_route_goal_G2",
            ),
            "dispatcher_topics": DISPATCHER_TOPICS,
            "passive_evaluator_topics": PASSIVE_EVALUATOR_TOPICS,
            "evidence_file": f"{run.run_id}.json",
        })
    return {
        "qualification": QUALIFICATION,
        "dispatch": False,
        "reason": "external_scene_reset_live_adapter_required",
        "exactly_once_reset_contract": "reuse_v6_formal_episode_guard",
        "runs": rows,
    }


def _points(value: Any, name: str) -> tuple[tuple[float, float], ...]:
    if not isinstance(value, list) or not value:
        raise CausalContractError(f"{name} must be a non-empty point list")
    result: list[tuple[float, float]] = []
    for index, point in enumerate(value):
        if isinstance(point, Mapping):
            x, y = point.get("x"), point.get("y")
        elif isinstance(point, Sequence) and not isinstance(point, (str, bytes)) and len(point) >= 2:
            x, y = point[0], point[1]
        else:
            raise CausalContractError(f"{name}[{index}] must be a point")
        try:
            result.append((float(x), float(y)))
        except (TypeError, ValueError) as exc:
            raise CausalContractError(f"{name}[{index}] has non-numeric coordinates") from exc
    return tuple(result)


def path_length(points: Sequence[tuple[float, float]]) -> float:
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def path_hausdorff(lhs: Sequence[tuple[float, float]], rhs: Sequence[tuple[float, float]]) -> float:
    def directed(a: Sequence[tuple[float, float]], b: Sequence[tuple[float, float]]) -> float:
        return max(min(math.dist(point, candidate) for candidate in b) for point in a)
    return max(directed(lhs, rhs), directed(rhs, lhs))


def path_direction(points: Sequence[tuple[float, float]]) -> str:
    start, goal = points[0], points[-1]
    vx, vy = goal[0] - start[0], goal[1] - start[1]
    norm = math.hypot(vx, vy)
    if norm <= 1e-9:
        return "unknown"
    signed = [vx * (point[1] - start[1]) - vy * (point[0] - start[0]) for point in points[1:-1]]
    if not signed:
        return "straight"
    mean_offset = statistics.fmean(signed) / norm
    if mean_offset > 0.05:
        return "left"
    if mean_offset < -0.05:
        return "right"
    return "straight"


def _footprint_center(value: Any) -> tuple[float, float]:
    row = _mapping(value, "rgbd_obstacle_footprint")
    if isinstance(row.get("center"), Sequence):
        center = row["center"]
        return float(center[0]), float(center[1])
    return float(row["x"]), float(row["y"])


def _scan_and_spatial_metrics(samples: Any, tolerance_m: float) -> tuple[int, int, int, int]:
    if not isinstance(samples, list) or not samples:
        raise CausalContractError("synchronized_samples must be a non-empty list")
    synchronized = invisible = matched = total = 0
    for index, raw_sample in enumerate(samples):
        sample = _mapping(raw_sample, f"synchronized_samples[{index}]")
        stamp = sample.get("stamp_ns")
        frame = sample.get("frame_id")
        if isinstance(stamp, bool) or not isinstance(stamp, int) or stamp < 0 or not isinstance(frame, str) or not frame:
            raise CausalContractError(f"synchronized_samples[{index}] missing valid frame/time")
        synchronized += 1
        footprints = sample.get("rgbd_obstacle_footprints")
        typed = sample.get("typed_obstacles")
        if not isinstance(footprints, list) or not isinstance(typed, list):
            raise CausalContractError(f"synchronized_samples[{index}] missing obstacle arrays")
        scan_points = sample.get("scan_point_count")
        scan_hits = sample.get("scan_hits_in_obstacle_footprints")
        if isinstance(scan_points, bool) or not isinstance(scan_points, int) or scan_points < 0:
            raise CausalContractError(f"synchronized_samples[{index}].scan_point_count invalid")
        if isinstance(scan_hits, bool) or not isinstance(scan_hits, int) or scan_hits < 0:
            raise CausalContractError(f"synchronized_samples[{index}].scan_hits_in_obstacle_footprints invalid")
        if footprints and scan_hits == 0:
            invisible += 1
        centers = tuple(_footprint_center(item) for item in footprints)
        for obstacle_value in typed:
            obstacle = _mapping(obstacle_value, "typed_obstacle")
            if obstacle.get("accepted") is not True:
                continue
            total += 1
            point = (float(obstacle["x"]), float(obstacle["y"]))
            if centers and min(math.dist(point, center) for center in centers) <= tolerance_m:
                matched += 1
    return synchronized, invisible, matched, total


REQUIRED_EVIDENCE_KEYS = (
    "run_id", "repeat", "arm", "identity", "reset", "freshness",
    "synchronized_samples", "obstacle_validation", "layer", "critic",
    "planning_prior", "costmaps", "plan", "optimal_trajectory", "odom",
    "cmd_vel", "passive",
)


def _evaluate_run(manifest: CausalManifest, run: RunContract, path: Path) -> tuple[RunResult, dict[str, Any]]:
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
        row = _mapping(raw, str(path))
        missing = [key for key in REQUIRED_EVIDENCE_KEYS if key not in row]
        if missing:
            raise CausalContractError("missing evidence: " + ", ".join(missing))
        if (row["run_id"], row["repeat"], row["arm"]) != (run.run_id, run.repeat, run.arm):
            raise CausalContractError("run identity does not match manifest row")
        recorded_identity = _mapping(row["identity"], "identity")
        for key in (
            "scene_id", "obstacle_layout_id", "seed", "start", "goal", "timeout_sec",
            "route_backend", "graph_backend", "direct_rgbd_costmap_enabled",
        ):
            if recorded_identity.get(key) != manifest.identity.get(key):
                raise CausalContractError(f"identity.{key} differs from frozen manifest")

        reset = _mapping(row["reset"], "reset")
        if (reset.get("calls"), reset.get("events"), reset.get("goal_publications")) != (1, 1, 1):
            raise CausalContractError("exactly-once reset/goal evidence invalid")
        if reset.get("localization_contract") != "same_estimated_autonomy":
            raise CausalContractError("localization contract differs across arms")

        arm = manifest.arms[run.arm]
        if run.arm == "M0" and row.get("module2_uds_connected", False):
            raise CausalContractError("M0 must not connect Module2 UDS")
        if run.arm != "M0" and row.get("module2_uds_connected") is not True:
            raise CausalContractError(f"{run.arm} requires Module2 UDS evidence")

        layer = _mapping(row["layer"], "layer")
        critic = _mapping(row["critic"], "critic")
        if layer.get("mode") != arm.obstacle_layer_mode or critic.get("mode") != arm.critic_mode:
            raise CausalContractError("recorded layer/critic arm does not match manifest")
        layer_cells: list[int] = []
        for scope in ("global", "local"):
            layer_status = _mapping(layer.get(scope), f"layer.{scope}")
            if int(layer_status.get("status_count", 0)) <= 0:
                raise CausalContractError(f"layer.{scope} status evidence missing")
            cells = int(layer_status.get("cells", -1))
            max_cost = int(layer_status.get("max_cost", -1))
            if cells < 0 or not 0 <= max_cost <= 255:
                raise CausalContractError(f"layer.{scope} cells/max_cost invalid")
            layer_cells.append(cells)
        if arm.obstacle_layer_mode in {"off", "shadow"} and any(layer_cells):
            raise CausalContractError("off/shadow obstacle layer wrote Costmap cells")
        if arm.obstacle_layer_mode == "active" and not any(layer_cells):
            raise CausalContractError("active obstacle layer has no applied cells")
        if arm.critic_mode != "active" and critic.get("applied") is not False:
            raise CausalContractError("off/shadow critic must not be applied")
        if not isinstance(critic.get("reason"), str) or not critic["reason"]:
            raise CausalContractError("critic reason evidence missing")
        if not isinstance(row["planning_prior"], list):
            raise CausalContractError("planning_prior must be a list")
        if run.arm == "M0" and row["planning_prior"]:
            raise CausalContractError("M0 planning_prior must remain empty")
        for key in ("global", "local"):
            costmap = _mapping(row["costmaps"], "costmaps").get(key)
            if not isinstance(costmap, Mapping) or not costmap.get("recorded"):
                raise CausalContractError(f"costmaps.{key} missing")

        tolerance = float(manifest.criteria["typed_spatial_match_tolerance_m"])
        synchronized, invisible, matches, spatial_total = _scan_and_spatial_metrics(
            row["synchronized_samples"], tolerance
        )
        if run.arm != "M0" and spatial_total == 0:
            raise CausalContractError("typed obstacle spatial evidence missing")
        if run.arm != "M0" and matches != spatial_total:
            raise CausalContractError("typed obstacle spatial match failed")
        validations = row["obstacle_validation"]
        if not isinstance(validations, list):
            raise CausalContractError("obstacle_validation must be a list")
        if run.arm != "M0" and not validations:
            raise CausalContractError("typed obstacle validation evidence missing")

        freshness = _mapping(row["freshness"], "freshness")
        max_age = float(freshness.get("max_typed_obstacle_age_sec", math.inf))
        ttl = float(manifest.freshness["typed_obstacle_ttl_sec"])
        stale = max_age > ttl
        if stale:
            fail_open = (
                freshness.get("stopped_before_dispatch") is True
                and freshness.get("layer_zero_write") is True
                and freshness.get("critic_not_applied") is True
            )
            verdict = "STOP_FAIL_OPEN" if fail_open else "INVALID"
            reasons = ("typed_obstacle_ttl_exceeded",) if fail_open else ("stale_input_not_fail_open",)
        else:
            verdict, reasons = "VALID", ()

        plan = _points(row["plan"], "plan")
        _points(row["optimal_trajectory"], "optimal_trajectory")
        _points(row["odom"], "odom")
        if not isinstance(row["cmd_vel"], list) or not row["cmd_vel"]:
            raise CausalContractError("cmd_vel evidence missing")
        passive = _mapping(row["passive"], "passive")
        _points(passive.get("ground_truth_odom"), "passive.ground_truth_odom")
        clearance = float(passive["minimum_clearance_m"])
        collision = _bool(passive.get("collision"), "passive.collision")
        success = _bool(passive.get("success"), "passive.success")

        online_applied = critic.get("applied") is True
        offline_scores = critic.get("offline_reconstructed_scores", [])
        if online_applied:
            critic_participation = "online_applied"
        elif isinstance(offline_scores, list) and offline_scores:
            critic_participation = "offline_reconstructed"
        else:
            critic_participation = "none"
        result = RunResult(
            run_id=run.run_id,
            repeat=run.repeat,
            arm=run.arm,
            verdict=verdict,
            reasons=reasons,
            synchronized_frames=synchronized,
            scan_invisible_rgbd_pairs=invisible,
            typed_spatial_matches=matches,
            typed_spatial_total=spatial_total,
            path_length_m=path_length(plan),
            minimum_clearance_m=clearance,
            collision=collision,
            success=success,
            reroute_direction=path_direction(plan),
            critic_participation=critic_participation,
            evidence_file=str(path),
        )
        return result, {"raw": row, "plan": plan}
    except (OSError, json.JSONDecodeError, KeyError, TypeError, ValueError, CausalContractError) as exc:
        return RunResult(
            run_id=run.run_id,
            repeat=run.repeat,
            arm=run.arm,
            verdict="INVALID",
            reasons=(str(exc),),
            synchronized_frames=0,
            scan_invisible_rgbd_pairs=0,
            typed_spatial_matches=0,
            typed_spatial_total=0,
            path_length_m=0.0,
            minimum_clearance_m=0.0,
            collision=False,
            success=False,
            reroute_direction="unknown",
            critic_participation="none",
            evidence_file=str(path),
        ), {"raw": {}, "plan": ()}


def _pair(
    lhs: tuple[RunResult, Mapping[str, Any]],
    rhs: tuple[RunResult, Mapping[str, Any]],
) -> PairResult:
    lhs_result, lhs_data = lhs
    rhs_result, rhs_data = rhs
    lhs_plan = lhs_data["plan"]
    rhs_plan = rhs_data["plan"]
    hausdorff = path_hausdorff(lhs_plan, rhs_plan)
    denominator = max(lhs_result.path_length_m, 1e-9)
    return PairResult(
        repeat=lhs_result.repeat,
        lhs_arm=lhs_result.arm,
        rhs_arm=rhs_result.arm,
        hausdorff_m=hausdorff,
        length_delta_fraction=abs(rhs_result.path_length_m - lhs_result.path_length_m) / denominator,
        clearance_gain_m=rhs_result.minimum_clearance_m - lhs_result.minimum_clearance_m,
        direction_consistent=(
            lhs_result.reroute_direction == rhs_result.reroute_direction
            and lhs_result.reroute_direction not in {"unknown", "straight"}
        ),
    )


def evaluate(manifest: CausalManifest, evidence_dir: str | Path) -> CausalSummary:
    root = Path(evidence_dir).expanduser().resolve()
    evaluated: dict[tuple[int, str], tuple[RunResult, Mapping[str, Any]]] = {}
    ordered_results: list[RunResult] = []
    for run in manifest.runs:
        result, data = _evaluate_run(manifest, run, root / f"{run.run_id}.json")
        ordered_results.append(result)
        evaluated[(run.repeat, run.arm)] = (result, data)

    invalid = [result.run_id for result in ordered_results if result.verdict != "VALID"]
    reasons: list[str] = []
    m1_m0: list[PairResult] = []
    m2_m1: list[PairResult] = []
    m3_m1: list[PairResult] = []
    m3_m2: list[PairResult] = []
    if invalid:
        verdict = "INVALID"
        reasons.append("invalid_or_stopped_runs:" + ",".join(invalid))
    else:
        for repeat in (1, 2, 3):
            m1_m0.append(_pair(evaluated[(repeat, "M0")], evaluated[(repeat, "M1")]))
            m2_m1.append(_pair(evaluated[(repeat, "M1")], evaluated[(repeat, "M2")]))
            m3_m1.append(_pair(evaluated[(repeat, "M1")], evaluated[(repeat, "M3")]))
            m3_m2.append(_pair(evaluated[(repeat, "M2")], evaluated[(repeat, "M3")]))

        isolation_ok = all(
            pair.hausdorff_m <= float(manifest.criteria["m1_m0_path_hausdorff_max_m"])
            and pair.length_delta_fraction <= float(manifest.criteria["m1_m0_path_length_delta_max_fraction"])
            for pair in m1_m0
        )
        if not isolation_ok:
            reasons.append("M1_vs_M0_isolation_failed")

        clearance_threshold = float(manifest.criteria["active_clearance_gain_min_m"])
        for arm, pairs in (("M2", m2_m1), ("M3", m3_m1)):
            if statistics.median(pair.clearance_gain_m for pair in pairs) < clearance_threshold:
                reasons.append(f"{arm}_median_clearance_gain_below_threshold")
            active_results = [evaluated[(repeat, arm)][0] for repeat in (1, 2, 3)]
            baselines = [evaluated[(repeat, "M1")][0] for repeat in (1, 2, 3)]
            if any(active.collision and not baseline.collision for active, baseline in zip(active_results, baselines)):
                reasons.append(f"{arm}_new_collision")
            directions = {result.reroute_direction for result in active_results}
            if len(directions) != 1 or directions & {"unknown", "straight"}:
                reasons.append(f"{arm}_reroute_direction_inconsistent")

        m3_results = [evaluated[(repeat, "M3")][0] for repeat in (1, 2, 3)]
        if any(result.critic_participation == "none" for result in m3_results):
            reasons.append("M3_critic_participation_missing")
        separation = statistics.median(pair.hausdorff_m for pair in m3_m2)
        separation_min = float(manifest.criteria["m3_m2_trajectory_separation_min_m"])
        no_separation = separation < separation_min
        if no_separation:
            reasons.append("M3_critic_has_no_trajectory_separation")

        hard_fail = any(
            reason != "M3_critic_has_no_trajectory_separation"
            for reason in reasons
        )
        if hard_fail:
            verdict = "FAIL"
        elif no_separation:
            verdict = "AMBIGUOUS"
        else:
            verdict = "PASS_ENGINEERING_CAUSAL"

    visualizations: list[Mapping[str, Any]] = []
    for repeat in (1, 2, 3):
        visualizations.append({
            "repeat": repeat,
            "scene_id": manifest.identity["scene_id"],
            "overlay": "map_costmap_rgbd_scan_typed_obstacles_paths",
            "runs": {
                arm: {
                    "evidence_file": evaluated[(repeat, arm)][0].evidence_file,
                    "path_field": "plan",
                    "costmap_fields": ("costmaps.global", "costmaps.local"),
                    "obstacle_field": "synchronized_samples",
                }
                for arm in ("M0", "M1", "M2", "M3")
            },
        })
    return CausalSummary(
        qualification="ENGINEERING_ONLY_NOT_FORMAL",
        verdict=verdict,
        reasons=tuple(reasons),
        runs=tuple(ordered_results),
        m1_vs_m0=tuple(m1_m0),
        m2_vs_m1=tuple(m2_m1),
        m3_vs_m1=tuple(m3_m1),
        m3_vs_m2=tuple(m3_m2),
        visualization_inputs=tuple(visualizations),
    )


def _jsonable(value: Any) -> Any:
    if hasattr(value, "__dataclass_fields__"):
        return {key: _jsonable(item) for key, item in asdict(value).items()}
    if isinstance(value, Mapping):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


def _write_or_print(payload: Any, output: str | None) -> None:
    text = json.dumps(_jsonable(payload), indent=2, sort_keys=True) + "\n"
    if output:
        destination = Path(output).expanduser().resolve()
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(text, encoding="utf-8")
    else:
        print(text, end="")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    for name in ("manifest", "plan"):
        sub = subparsers.add_parser(name)
        sub.add_argument("--config", required=True)
        sub.add_argument("--output")
    evaluate_parser = subparsers.add_parser("evaluate")
    evaluate_parser.add_argument("--config", required=True)
    evaluate_parser.add_argument("--evidence-dir", required=True)
    evaluate_parser.add_argument("--output")
    run_parser = subparsers.add_parser("run")
    run_parser.add_argument("--config", required=True)
    run_parser.add_argument("--scene-adapter")
    run_parser.add_argument("--reset-adapter")
    run_parser.add_argument("--live-adapter")
    run_parser.add_argument("--output")
    return parser


def cli(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        manifest = load_manifest(args.config)
        if args.command == "manifest":
            _write_or_print({
                "qualification": QUALIFICATION,
                "identity": manifest.identity,
                "localization_contract": manifest.localization_contract,
                "arms": manifest.arms,
                "runs": manifest.runs,
            }, args.output)
            return 0
        if args.command == "plan":
            _write_or_print(build_plan(manifest), args.output)
            return 0
        if args.command == "evaluate":
            summary = evaluate(manifest, args.evidence_dir)
            _write_or_print(summary, args.output)
            return 0 if summary.verdict == "PASS_ENGINEERING_CAUSAL" else 2
        adapters = (args.scene_adapter, args.reset_adapter, args.live_adapter)
        missing = [name for name, value in zip(("scene", "reset", "live"), adapters) if not value]
        _write_or_print({
            "qualification": QUALIFICATION,
            "state": "NOT_RUN",
            "reason": (
                "missing_external_adapters:" + ",".join(missing)
                if missing else "external_adapter_execution_not_implemented"
            ),
            "plan": build_plan(manifest),
        }, args.output)
        return 2
    except (OSError, yaml.YAMLError, CausalContractError) as exc:
        print(f"INVALID: {exc}", file=sys.stderr)
        return 2


def main() -> None:
    raise SystemExit(cli())


if __name__ == "__main__":
    main()
