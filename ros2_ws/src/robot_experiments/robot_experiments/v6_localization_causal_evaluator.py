"""Passive absolute-map evaluator for the V6 L0--L3 localization study.

Ground Truth enters only through this offline module.  The runtime dispatcher
must record estimated-state and control topics without subscribing to GT.
Errors are absolute map-frame errors; no first-frame or trajectory alignment is
performed.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import statistics
import sys
from typing import Any, Iterable, Iterator, Mapping, Sequence


PHASE_DE_EVENT_SCHEMA = "bio_nav_v6_phase_de_localization_event_v1"
PHASE_DE_GT_SCHEMA = "bio_nav_v6_phase_de_localization_gt_v1"
PHASE_DE_EVENTS = frozenset(
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
PHASE_DE_PAIRS = {"D": ("S0", "S1"), "E": ("R0", "R1")}
GROUND_TRUTH_ODOM_TOPIC = "/ground_truth/odom"

# Frozen evaluator-only partition from the Kujiale Module1 enrollment contract.
# It is deliberately duplicated here so the passive exporter has no runtime
# dependency on Module2 and cannot feed Ground Truth back into navigation.
_CANONICAL_REGION_STATES = {
    "north_room": (199, 200, 201, 202, 215, 216, 217),
    "south_room": (69, 70, 84, 85, 86, 87, 101, 102),
    "south_corridor": (39, 40, 56, 72, 88, 103, 104),
    "east_corridor": (120, 121, 122, 136, 137, 153, 169, 184, 185),
    "central_intersection": (118, 119, 134, 135),
    "central_north_corridor": (151, 167, 183),
    "west_lower_corridor": (115, 116, 117),
    "west_upper_north_corridor": (133, 147, 148, 149, 150, 165, 166, 180, 181, 182),
}
_CANONICAL_STATE_TO_REGION = {
    state: region
    for region, states in _CANONICAL_REGION_STATES.items()
    for state in states
}


class EvaluationError(RuntimeError):
    """Recorded evidence is missing or violates the frozen contract."""


@dataclass(frozen=True)
class ErrorSample:
    stamp_s: float
    position_error_m: float
    yaw_error_deg: float


@dataclass(frozen=True)
class EpisodeResult:
    run_id: str
    arm: str
    case: str
    seed: int
    verdict: str
    reasons: tuple[str, ...]
    convergence_time_s: float | None
    lost_time_s: float | None
    recovery_time_s: float | None
    position_error_p95_m: float | None
    yaw_error_p95_deg: float | None
    initialpose_count: int
    integration_initialpose_count: int
    manual_rescue_count: int
    nonzero_cmd_during_pause: int
    collision_count: int


def _mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise EvaluationError(f"{name} must be a mapping")
    return value


def _rows(value: Any, name: str) -> list[Mapping[str, Any]]:
    if not isinstance(value, list):
        raise EvaluationError(f"{name} must be a list")
    return [_mapping(row, f"{name}[{index}]") for index, row in enumerate(value)]


def _finite(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise EvaluationError(f"{name} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise EvaluationError(f"{name} must be finite")
    return result


def _yaw_error_deg(lhs: float, rhs: float) -> float:
    return abs((lhs - rhs + 180.0) % 360.0 - 180.0)


def absolute_error_samples(
    estimates: Sequence[Mapping[str, Any]],
    ground_truth: Sequence[Mapping[str, Any]],
    *,
    synchronization_tolerance_s: float = 0.10,
) -> tuple[ErrorSample, ...]:
    """Pair estimates with nearest GT samples and compute unaligned errors."""

    if not estimates or not ground_truth:
        raise EvaluationError("estimated and Ground Truth samples are required")
    truth = sorted(
        (
            _finite(row.get("stamp_s"), "ground_truth.stamp_s"),
            _finite(row.get("x"), "ground_truth.x"),
            _finite(row.get("y"), "ground_truth.y"),
            _finite(row.get("yaw_deg"), "ground_truth.yaw_deg"),
        )
        for row in ground_truth
    )
    output: list[ErrorSample] = []
    for index, row in enumerate(sorted(estimates, key=lambda item: float(item["stamp_s"]))):
        stamp = _finite(row.get("stamp_s"), f"estimates[{index}].stamp_s")
        x = _finite(row.get("x"), f"estimates[{index}].x")
        y = _finite(row.get("y"), f"estimates[{index}].y")
        yaw = _finite(row.get("yaw_deg"), f"estimates[{index}].yaw_deg")
        nearest = min(truth, key=lambda item: abs(item[0] - stamp))
        if abs(nearest[0] - stamp) > synchronization_tolerance_s:
            continue
        output.append(
            ErrorSample(
                stamp_s=stamp,
                position_error_m=math.hypot(x - nearest[1], y - nearest[2]),
                yaw_error_deg=_yaw_error_deg(yaw, nearest[3]),
            )
        )
    if not output:
        raise EvaluationError("no estimate/GT samples synchronized within tolerance")
    return tuple(output)


def _held_onset(
    samples: Sequence[ErrorSample],
    predicate: Any,
    hold_s: float,
    *,
    not_before_s: float | None = None,
) -> float | None:
    onset: float | None = None
    for sample in samples:
        if not_before_s is not None and sample.stamp_s < not_before_s:
            continue
        if predicate(sample):
            if onset is None:
                onset = sample.stamp_s
            if sample.stamp_s - onset >= hold_s:
                return onset
        else:
            onset = None
    return None


def _p95(values: Iterable[float]) -> float:
    ordered = sorted(values)
    if not ordered:
        raise EvaluationError("P95 requires samples")
    return ordered[min(len(ordered) - 1, math.ceil(0.95 * len(ordered)) - 1)]


def evaluate_episode(
    run: Mapping[str, Any],
    evidence: Mapping[str, Any],
    criteria: Mapping[str, Any],
) -> EpisodeResult:
    """Evaluate one row without changing coordinates or runtime state."""

    run_id = str(run["run_id"])
    arm = str(run["arm"])
    case = str(run["case"])
    seed = int(run["seed"])
    reasons: list[str] = []
    if evidence.get("run_id") != run_id:
        reasons.append("run_id_mismatch")
    if evidence.get("arm") != arm or evidence.get("case") != case or evidence.get("seed") != seed:
        reasons.append("identity_mismatch")

    dispatcher = _mapping(evidence.get("dispatcher"), "dispatcher")
    if any(str(topic).startswith("/ground_truth/") for topic in dispatcher.get("topics", ())):
        reasons.append("dispatcher_ground_truth_firewall_violation")
    if any(key.startswith("ground_truth") for key in dispatcher):
        reasons.append("dispatcher_contains_ground_truth")
    passive = _mapping(evidence.get("passive_evaluator"), "passive_evaluator")
    errors = absolute_error_samples(
        _rows(dispatcher.get("estimated_map_poses"), "dispatcher.estimated_map_poses"),
        _rows(passive.get("ground_truth_map_poses"), "passive_evaluator.ground_truth_map_poses"),
        synchronization_tolerance_s=float(criteria["synchronization_tolerance_s"]),
    )
    episode_start = errors[0].stamp_s
    convergence_onset = _held_onset(
        errors,
        lambda item: (
            item.position_error_m <= float(criteria["converged_position_error_m"])
            and item.yaw_error_deg <= float(criteria["converged_yaw_error_deg"])
        ),
        float(criteria["convergence_hold_s"]),
    )
    lost_onset = _held_onset(
        errors,
        lambda item: (
            item.position_error_m > float(criteria["lost_position_error_m"])
            or item.yaw_error_deg > float(criteria["lost_yaw_error_deg"])
        ),
        float(criteria["lost_hold_s"]),
        not_before_s=(
            _finite(evidence.get("kidnap_trigger_stamp_s"), "kidnap_trigger_stamp_s")
            if case == "S3"
            else None
        ),
    )
    recovery_onset = None
    recovery_time = None
    if case == "S3" and lost_onset is not None:
        recovery_onset = _held_onset(
            errors,
            lambda item: (
                item.position_error_m <= float(criteria["converged_position_error_m"])
                and item.yaw_error_deg <= float(criteria["converged_yaw_error_deg"])
            ),
            float(criteria["convergence_hold_s"]),
            not_before_s=lost_onset,
        )
        if recovery_onset is not None:
            recovery_time = recovery_onset - lost_onset

    initialpose = _rows(dispatcher.get("initialpose_events", []), "dispatcher.initialpose_events")
    rescues = _rows(dispatcher.get("manual_rescue_events", []), "dispatcher.manual_rescue_events")
    integration_writes = sum(row.get("source") == "integration" for row in initialpose)
    expected = _mapping(run.get("expected"), "run.expected")
    integration_activity = _mapping(
        dispatcher.get("integration_activity"), "dispatcher.integration_activity"
    )
    if integration_activity.get("mode") != expected["integration_mode"]:
        reasons.append("integration_mode")
    if arm == "L1" and (
        int(integration_activity.get("initialpose_writes", -1)) != 0
        or int(integration_activity.get("pose_correction_writes", -1)) != 0
    ):
        reasons.append("L1_isolation")
    if integration_writes != int(expected["integration_initialpose_writes"]):
        reasons.append("integration_initialpose_write_count")
    if len(initialpose) != int(expected["total_initialpose_writes"]):
        reasons.append("total_initialpose_write_count")
    if len(rescues) != int(expected["manual_rescue_requests"]):
        reasons.append("manual_rescue_count")
    if arm == "L3" and case == "S3" and rescues:
        rescue_stamp = _finite(rescues[0].get("stamp_s"), "manual_rescue_events[0].stamp_s")
        if lost_onset is None or rescue_stamp < lost_onset:
            reasons.append("manual_rescue_before_lost_detection")

    pauses = _rows(dispatcher.get("pause_intervals", []), "dispatcher.pause_intervals")
    commands = _rows(dispatcher.get("cmd_vel", []), "dispatcher.cmd_vel")
    nonzero_during_pause = 0
    for command in commands:
        stamp = _finite(command.get("stamp_s"), "cmd_vel.stamp_s")
        speed = math.hypot(
            _finite(command.get("linear_x", 0.0), "cmd_vel.linear_x"),
            _finite(command.get("angular_z", 0.0), "cmd_vel.angular_z"),
        )
        if speed <= float(criteria["zero_cmd_epsilon"]):
            continue
        if any(float(row["start_s"]) <= stamp <= float(row["end_s"]) for row in pauses):
            nonzero_during_pause += 1
    if nonzero_during_pause:
        reasons.append("nonzero_cmd_during_pause")

    collision_count = int(passive.get("collision_count", 0))
    if collision_count:
        reasons.append("collision")
    publishers = _mapping(dispatcher.get("publisher_owners"), "dispatcher.publisher_owners")
    if publishers.get("/odom") != "robot_localization" or publishers.get("map->odom") != "amcl":
        reasons.append("publisher_ownership")
    if case == "S3" and evidence.get("kidnap_service_calls") != 1:
        reasons.append("kidnap_not_exactly_once")
    if case == "S3" and lost_onset is None:
        reasons.append("lost_not_observed")

    return EpisodeResult(
        run_id=run_id,
        arm=arm,
        case=case,
        seed=seed,
        verdict="PASS" if not reasons else "FAIL",
        reasons=tuple(reasons),
        convergence_time_s=(
            None if convergence_onset is None else convergence_onset - episode_start
        ),
        lost_time_s=lost_onset,
        recovery_time_s=recovery_time,
        position_error_p95_m=_p95(item.position_error_m for item in errors),
        yaw_error_p95_deg=_p95(item.yaw_error_deg for item in errors),
        initialpose_count=len(initialpose),
        integration_initialpose_count=integration_writes,
        manual_rescue_count=len(rescues),
        nonzero_cmd_during_pause=nonzero_during_pause,
        collision_count=collision_count,
    )


def _median(values: Sequence[float | None]) -> float | None:
    finite = [value for value in values if value is not None]
    return statistics.median(finite) if finite else None


def evaluate_campaign(manifest: Mapping[str, Any], evidence_dir: str | Path) -> dict[str, Any]:
    root = Path(evidence_dir).expanduser().resolve()
    criteria = _mapping(manifest["criteria"], "criteria")
    results: list[EpisodeResult] = []
    missing: list[str] = []
    for run in manifest["core_runs"]:
        path = root / f"{run['run_id']}.json"
        if not path.is_file():
            missing.append(run["run_id"])
            continue
        try:
            evidence = _mapping(json.loads(path.read_text(encoding="utf-8")), str(path))
            results.append(evaluate_episode(run, evidence, criteria))
        except (EvaluationError, KeyError, TypeError, ValueError) as exc:
            results.append(
                EpisodeResult(
                    run_id=run["run_id"], arm=run["arm"], case=run["case"], seed=run["seed"],
                    verdict="INVALID", reasons=(str(exc),), convergence_time_s=None,
                    lost_time_s=None, recovery_time_s=None, position_error_p95_m=None,
                    yaw_error_p95_deg=None, initialpose_count=0,
                    integration_initialpose_count=0, manual_rescue_count=0,
                    nonzero_cmd_during_pause=0, collision_count=0,
                )
            )
    if missing:
        return {
            "qualification": "ENGINEERING_CAUSAL_NOT_RUN",
            "verdict": "NOT_RUN",
            "missing_run_ids": missing,
            "results": [asdict(result) for result in results],
        }

    reasons: list[str] = []
    if any(result.verdict != "PASS" for result in results):
        reasons.append("episode_contract_failure")
    s0_l2 = [result for result in results if result.case == "S0" and result.arm == "L2"]
    s0_l0 = [result for result in results if result.case == "S0" and result.arm == "L0"]
    l2_fast = sum(
        result.convergence_time_s is not None
        and result.convergence_time_s <= float(criteria["l2_convergence_max_s"])
        for result in s0_l2
    )
    if l2_fast < int(criteria["required_successes_of_five"]):
        reasons.append("L2_convergence_rate")
    l2_median = _median([result.convergence_time_s for result in s0_l2])
    l0_median = _median([result.convergence_time_s for result in s0_l0])
    if (
        l2_median is None or l0_median is None or l0_median <= 0.0
        or (l0_median - l2_median) / l0_median < float(criteria["improvement_fraction"])
    ):
        reasons.append("L2_median_improvement")

    s3_l3 = [result for result in results if result.case == "S3" and result.arm == "L3"]
    s3_l2 = [result for result in results if result.case == "S3" and result.arm == "L2"]
    l3_fast = sum(
        result.recovery_time_s is not None
        and result.recovery_time_s <= float(criteria["l3_recovery_max_s"])
        for result in s3_l3
    )
    if l3_fast < int(criteria["required_successes_of_five"]):
        reasons.append("L3_recovery_rate")
    l3_median = _median([result.recovery_time_s for result in s3_l3])
    l2_recovery_median = _median([result.recovery_time_s for result in s3_l2])
    if l3_median is None or l2_recovery_median is None:
        reasons.append("L3_recovery_improvement")
    else:
        fractional = (
            (l2_recovery_median - l3_median) / l2_recovery_median
            if l2_recovery_median > 0.0 else -math.inf
        )
        absolute = l2_recovery_median - l3_median
        if fractional < float(criteria["improvement_fraction"]) and absolute < float(criteria["improvement_absolute_s"]):
            reasons.append("L3_recovery_improvement")
    if any(result.initialpose_count for result in results if result.case == "W0"):
        reasons.append("W0_wrong_reseed")

    return {
        "qualification": "ENGINEERING_CAUSAL_EVALUATION_ONLY",
        "verdict": "PASS_CRITERIA" if not reasons else "FAIL",
        "reasons": reasons,
        "results": [asdict(result) for result in results],
        "aggregate": {
            "L2_S0_fast_convergence": l2_fast,
            "L2_S0_median_s": l2_median,
            "L0_S0_median_s": l0_median,
            "L3_S3_fast_recovery": l3_fast,
            "L3_S3_median_s": l3_median,
            "L2_S3_median_s": l2_recovery_median,
        },
    }


def load_phase_de_jsonl(path: str | Path, *, ground_truth: bool = False) -> list[Mapping[str, Any]]:
    """Load one Phase D/E JSONL stream without mixing runtime and passive GT.

    The live runner owns the runtime stream.  A passive extractor may later
    create the Ground Truth stream from the recorded bag, but GT is never an
    accepted runtime event.
    """

    source = Path(path).expanduser().resolve()
    rows: list[Mapping[str, Any]] = []
    for line_number, line in enumerate(source.read_text(encoding="utf-8").splitlines(), start=1):
        if not line.strip():
            continue
        try:
            row = _mapping(json.loads(line), f"{source}:{line_number}")
        except json.JSONDecodeError as exc:
            raise EvaluationError(f"{source}:{line_number} is not valid JSON: {exc}") from exc
        expected_schema = PHASE_DE_GT_SCHEMA if ground_truth else PHASE_DE_EVENT_SCHEMA
        if row.get("schema") != expected_schema:
            raise EvaluationError(
                f"{source}:{line_number} schema must be {expected_schema}"
            )
        event = str(row.get("event", ""))
        if ground_truth:
            if event != "ground_truth_pose":
                raise EvaluationError(
                    f"{source}:{line_number} passive GT event must be ground_truth_pose"
                )
        elif event not in PHASE_DE_EVENTS:
            raise EvaluationError(f"{source}:{line_number} unknown runtime event {event!r}")
        rows.append(row)
    if not rows:
        raise EvaluationError(f"{source} contains no Phase D/E events")
    return rows


def _contains_ground_truth(value: Any, *, key: str = "") -> bool:
    lowered = key.lower()
    if "ground_truth" in lowered or lowered in {"gt_pose", "gt_region", "gt_region_id"}:
        return True
    if isinstance(value, Mapping):
        return any(_contains_ground_truth(item, key=str(name)) for name, item in value.items())
    if isinstance(value, (list, tuple)):
        return any(_contains_ground_truth(item) for item in value)
    return isinstance(value, str) and (
        value.startswith("/ground_truth/") or value == "ground_truth_pose"
    )


def _event_stamp(row: Mapping[str, Any], name: str) -> float:
    return _finite(row.get("stamp_s"), f"{name}.stamp_s")


def _event_value(row: Mapping[str, Any], *names: str) -> Any:
    values = row.get("values")
    nested = values if isinstance(values, Mapping) else {}
    for name in names:
        if name in row:
            return row[name]
        if name in nested:
            return nested[name]
    return None


def _numeric_summary(values: Sequence[float]) -> dict[str, Any]:
    if not values:
        return {"count": 0, "min": None, "median": None, "p95": None, "max": None}
    checked = [_finite(value, "metric") for value in values]
    return {
        "count": len(checked),
        "min": min(checked),
        "median": statistics.median(checked),
        "p95": _p95(checked),
        "max": max(checked),
    }


def _nearest_pose(
    rows: Sequence[Mapping[str, Any]], stamp_s: float, tolerance_s: float
) -> Mapping[str, Any] | None:
    if not rows:
        return None
    nearest = min(rows, key=lambda row: abs(float(row["stamp_s"]) - stamp_s))
    return nearest if abs(float(nearest["stamp_s"]) - stamp_s) <= tolerance_s else None


def _pose_path_length(rows: Sequence[Mapping[str, Any]], name: str) -> float | None:
    if not rows:
        return None
    total = 0.0
    previous: tuple[float, float] | None = None
    for index, row in enumerate(sorted(rows, key=lambda item: float(item["stamp_s"]))):
        point = (
            _finite(row.get("x"), f"{name}[{index}].x"),
            _finite(row.get("y"), f"{name}[{index}].y"),
        )
        if previous is not None:
            total += math.hypot(point[0] - previous[0], point[1] - previous[1])
        previous = point
    return total


def _phase_de_identity(events: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    if not events:
        raise EvaluationError("Phase D/E runtime events are required")
    first = events[0]
    identity = {
        "run_id": str(first.get("run_id", "")),
        "phase": str(first.get("phase", "")),
        "arm": str(first.get("arm", "")),
        "seed": first.get("seed"),
    }
    if not identity["run_id"]:
        raise EvaluationError("runtime run_id is required")
    if identity["phase"] not in PHASE_DE_PAIRS:
        raise EvaluationError("runtime phase must be D or E")
    if identity["arm"] not in PHASE_DE_PAIRS[identity["phase"]]:
        raise EvaluationError(
            f"runtime arm {identity['arm']!r} is invalid for Phase {identity['phase']}"
        )
    if isinstance(identity["seed"], bool) or not isinstance(identity["seed"], int):
        raise EvaluationError("runtime seed must be an integer")
    for index, row in enumerate(events):
        if row.get("schema") != PHASE_DE_EVENT_SCHEMA:
            raise EvaluationError(f"runtime[{index}] schema mismatch")
        if str(row.get("event", "")) not in PHASE_DE_EVENTS:
            raise EvaluationError(f"runtime[{index}] has an unknown event")
        if _contains_ground_truth(row):
            raise EvaluationError(f"runtime[{index}] violates the Ground Truth firewall")
        for name, expected in identity.items():
            if row.get(name) != expected:
                raise EvaluationError(f"runtime[{index}].{name} identity mismatch")
        _event_stamp(row, f"runtime[{index}]")
    if sum(row["event"] == "episode_start" for row in events) != 1:
        raise EvaluationError("runtime must contain exactly one episode_start")
    if sum(row["event"] == "episode_end" for row in events) != 1:
        raise EvaluationError("runtime must contain exactly one episode_end")
    return identity


def _phase_de_ground_truth(
    rows: Sequence[Mapping[str, Any]], identity: Mapping[str, Any]
) -> list[Mapping[str, Any]]:
    if not rows:
        raise EvaluationError("passive Ground Truth rows are required")
    output: list[Mapping[str, Any]] = []
    for index, row in enumerate(rows):
        if row.get("schema") != PHASE_DE_GT_SCHEMA or row.get("event") != "ground_truth_pose":
            raise EvaluationError(f"ground_truth[{index}] schema/event mismatch")
        for name in ("run_id", "phase", "arm", "seed"):
            if row.get(name) != identity[name]:
                raise EvaluationError(f"ground_truth[{index}].{name} identity mismatch")
        parsed = dict(row)
        parsed["stamp_s"] = _event_stamp(row, f"ground_truth[{index}]")
        parsed["x"] = _finite(row.get("x"), f"ground_truth[{index}].x")
        parsed["y"] = _finite(row.get("y"), f"ground_truth[{index}].y")
        parsed["yaw_deg"] = _finite(row.get("yaw_deg"), f"ground_truth[{index}].yaw_deg")
        region = row.get("region_id")
        if not isinstance(region, str) or not region:
            raise EvaluationError(f"ground_truth[{index}].region_id is required")
        output.append(parsed)
    return sorted(output, key=lambda row: float(row["stamp_s"]))


def _region_metrics(
    module1: Sequence[Mapping[str, Any]],
    ground_truth: Sequence[Mapping[str, Any]],
    tolerance_s: float,
) -> dict[str, Any]:
    samples: list[dict[str, Any]] = []
    for index, row in enumerate(sorted(module1, key=lambda item: float(item["stamp_s"]))):
        stamp = _event_stamp(row, f"module1_diagnostic[{index}]")
        predicted = _event_value(row, "region_id", "dominant_region_id")
        if predicted is None:
            continue
        truth = _nearest_pose(ground_truth, stamp, tolerance_s)
        if truth is None:
            continue
        samples.append(
            {
                "stamp_s": stamp,
                "predicted": str(predicted),
                "truth": str(truth["region_id"]),
                "correct": str(predicted) == str(truth["region_id"]),
                "x": float(truth["x"]),
                "y": float(truth["y"]),
            }
        )
    first_correct = next((sample for sample in samples if sample["correct"]), None)
    wrong_duration = 0.0
    wrong_distance = 0.0
    for left, right in zip(samples, samples[1:]):
        if left["correct"]:
            continue
        wrong_duration += max(0.0, right["stamp_s"] - left["stamp_s"])
        wrong_distance += math.hypot(right["x"] - left["x"], right["y"] - left["y"])
    return {
        "matched_sample_count": len(samples),
        "correct_sample_count": sum(sample["correct"] for sample in samples),
        "first_correct_region_id": None if first_correct is None else first_correct["truth"],
        "first_correct_region_time_s": None if first_correct is None else first_correct["stamp_s"],
        "wrong_region_duration_s": wrong_duration,
        "wrong_region_distance_m": wrong_distance,
    }


def _module1_metrics(rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    aliases = {
        "entropy": ("entropy", "place_entropy_normalized"),
        "reliability": ("reliability", "visual_reliability"),
        "ood_probability": ("ood_probability", "visual_ood_probability"),
        "dominant_mass": ("dominant_mass", "dominant_mode_mass"),
    }
    output: dict[str, Any] = {"sample_count": len(rows)}
    for output_name, names in aliases.items():
        values = []
        for row in rows:
            value = _event_value(row, *names)
            if value is not None:
                values.append(_finite(value, f"module1_diagnostic.{output_name}"))
        output[output_name] = _numeric_summary(values)

    covariance_rows: list[list[float]] = []
    for row in rows:
        value = _event_value(row, "dominant_covariance_m2", "dominant_mode_covariance_m2")
        if value is None:
            continue
        if not isinstance(value, (list, tuple)) or len(value) != 4:
            raise EvaluationError("module1 dominant covariance must contain four values")
        covariance_rows.append(
            [_finite(item, "module1_diagnostic.dominant_covariance_m2") for item in value]
        )
    names = ("xx", "xy", "yx", "yy")
    output["dominant_covariance_m2"] = {
        "count": len(covariance_rows),
        "component_median": {
            name: (None if not covariance_rows else statistics.median(row[index] for row in covariance_rows))
            for index, name in enumerate(names)
        },
        "trace": _numeric_summary([row[0] + row[3] for row in covariance_rows]),
    }
    return output


def _first_stamp(rows: Sequence[Mapping[str, Any]], event: str) -> float | None:
    matches = [_event_stamp(row, event) for row in rows if row["event"] == event]
    return min(matches) if matches else None


def _goal_success(row: Mapping[str, Any]) -> bool:
    if isinstance(row.get("success"), bool):
        return bool(row["success"])
    return str(row.get("state", "")).upper() in {
        "SUCCEEDED", "SUCCESS", "COMPLETE", "COMPLETED"
    }


def evaluate_phase_de_episode(
    runtime_events: Sequence[Mapping[str, Any]],
    ground_truth_rows: Sequence[Mapping[str, Any]],
    *,
    synchronization_tolerance_s: float = 0.15,
) -> dict[str, Any]:
    """Reduce one S0/S1/R0/R1 episode to raw engineering metrics."""

    if synchronization_tolerance_s < 0.0:
        raise EvaluationError("synchronization_tolerance_s must be non-negative")
    identity = _phase_de_identity(runtime_events)
    events = sorted(runtime_events, key=lambda row: float(row["stamp_s"]))
    ground_truth = _phase_de_ground_truth(ground_truth_rows, identity)
    by_event = {
        name: [row for row in events if row["event"] == name] for name in PHASE_DE_EVENTS
    }
    start_s = _first_stamp(events, "episode_start")
    end_s = _first_stamp(events, "episode_end")
    assert start_s is not None and end_s is not None
    if end_s < start_s:
        raise EvaluationError("episode_end precedes episode_start")

    estimated = by_event["estimated_pose"]
    error_samples = absolute_error_samples(
        estimated,
        ground_truth,
        synchronization_tolerance_s=synchronization_tolerance_s,
    )
    initialpose_sources: dict[str, int] = {}
    for row in by_event["initialpose"]:
        source = str(row.get("source", "unknown"))
        initialpose_sources[source] = initialpose_sources.get(source, 0) + 1

    supervisor_rows = [
        {
            "stamp_s": _event_stamp(row, "supervisor_diagnostic"),
            "mode": row.get("mode"),
            "state": row.get("state"),
            "reason": row.get("reason"),
            "result": row.get("result"),
            "reset_attempts": row.get("reset_attempts"),
        }
        for row in by_event["supervisor_diagnostic"]
    ]
    goal_dispatch = by_event["goal_dispatched"]
    goal_results = by_event["goal_result"]
    dispatched_legs = [str(row.get("leg_id", "")) for row in goal_dispatch]
    successful_legs = [str(row.get("leg_id", "")) for row in goal_results if _goal_success(row)]
    route_start = min((_event_stamp(row, "goal_dispatched") for row in goal_dispatch), default=None)
    route_end = max((_event_stamp(row, "goal_result") for row in goal_results), default=None)
    episode_end = by_event["episode_end"][0]
    completed = [str(value) for value in episode_end.get("completed_leg_ids", ())]
    route_success = bool(dispatched_legs) and set(dispatched_legs).issubset(
        set(successful_legs) | set(completed)
    )

    explicit_counts = [
        int(row["count"])
        for row in by_event["collision"]
        if isinstance(row.get("count"), int) and not isinstance(row.get("count"), bool)
    ]
    collision_observed = any(bool(row.get("collision", True)) for row in by_event["collision"])
    collision_observed = collision_observed or bool(episode_end.get("collision", False))
    collision_count = max(explicit_counts, default=1 if collision_observed else 0)

    pause_request_s = _first_stamp(events, "pause_requested")
    pause_confirm_s = _first_stamp(events, "pause_confirmed")
    pause_rows = by_event["pause_confirmed"]
    zero_after_request = any(
        bool(row.get("zero", False))
        and (pause_request_s is None or _event_stamp(row, "cmd_vel_sim") >= pause_request_s)
        for row in by_event["cmd_vel_sim"]
    )
    zero_confirmed = any(bool(row.get("cmd_vel_sim_zero", False)) for row in pause_rows)
    stationary_confirmed = any(bool(row.get("stationary", False)) for row in pause_rows)

    ready_s = _first_stamp(events, "localization_ready")
    recovered_s = _first_stamp(events, "localization_recovered")
    fault_s = _first_stamp(events, "fault_injected")
    prior_s = _first_stamp(events, "prior_write")
    initialpose_s = _first_stamp(events, "initialpose")
    seed_s = fault_s if identity["phase"] == "E" else initialpose_s
    time_to_ready = None if ready_s is None else ready_s - start_s
    time_to_recover = (
        None if recovered_s is None or fault_s is None else recovered_s - fault_s
    )
    if time_to_ready is not None and time_to_ready < 0.0:
        raise EvaluationError("localization_ready precedes episode_start")
    if time_to_recover is not None and time_to_recover < 0.0:
        raise EvaluationError("localization_recovered precedes fault_injected")

    module1_rows = by_event["module1_diagnostic"]
    return {
        **identity,
        "fault": (
            None
            if not by_event["fault_injected"]
            else by_event["fault_injected"][0].get("fault_id")
        ),
        "evaluation_kind": "ENGINEERING_RAW_METRICS_ONLY",
        "formal_gate": False,
        "ground_truth_policy": "separate_passive_offline_stream",
        "timestamps_s": {
            "episode_start": start_s,
            "seed": seed_s,
            "fault": fault_s,
            "pause_requested": pause_request_s,
            "pause_confirmed": pause_confirm_s,
            "prior_write": prior_s,
            "ready": ready_s,
            "recovered": recovered_s,
            "episode_end": end_s,
            "goals": [
                {
                    "event": row["event"],
                    "leg_id": row.get("leg_id"),
                    "stamp_s": _event_stamp(row, row["event"]),
                }
                for row in sorted(
                    goal_dispatch + goal_results,
                    key=lambda item: float(item["stamp_s"]),
                )
            ],
        },
        "initialpose": {
            "count": len(by_event["initialpose"]),
            "by_source": initialpose_sources,
            "events": [
                {
                    "stamp_s": _event_stamp(row, "initialpose"),
                    "source": row.get("source", "unknown"),
                    "count": row.get("count"),
                }
                for row in by_event["initialpose"]
            ],
        },
        "prior_write_count": len(by_event["prior_write"]),
        "supervisor_diagnostics": supervisor_rows,
        "region": _region_metrics(module1_rows, ground_truth, synchronization_tolerance_s),
        "localization": {
            "time_to_ready_s": time_to_ready,
            "time_to_recover_s": time_to_recover,
            "gt_position_error_m": _numeric_summary(
                [sample.position_error_m for sample in error_samples]
            ),
            "gt_yaw_error_deg": _numeric_summary(
                [sample.yaw_error_deg for sample in error_samples]
            ),
        },
        "route": {
            "success": route_success,
            "dispatched_leg_ids": dispatched_legs,
            "successful_leg_ids": successful_legs,
            "completed_leg_ids": completed,
            "collision": collision_observed,
            "collision_count": collision_count,
            "path_length_m": _pose_path_length(by_event["odom_pose"], "odom_pose"),
            "route_duration_s": (
                None if route_start is None or route_end is None else route_end - route_start
            ),
            "episode_duration_s": end_s - start_s,
        },
        "pause": {
            "latency_s": (
                None
                if pause_request_s is None or pause_confirm_s is None
                else pause_confirm_s - pause_request_s
            ),
            "cmd_vel_sim_zero_confirmed": zero_confirmed and zero_after_request,
            "stationary_confirmed": stationary_confirmed,
        },
        "module1": _module1_metrics(module1_rows),
        "episode_end": {
            "state": episode_end.get("state"),
            "stop_reason": episode_end.get("stop_reason"),
            "terminal_zero_confirmed": episode_end.get("terminal_zero_confirmed"),
        },
    }


def _nested_number(value: Mapping[str, Any], path: str) -> float | None:
    current: Any = value
    for name in path.split("."):
        if not isinstance(current, Mapping) or name not in current:
            return None
        current = current[name]
    if current is None or isinstance(current, bool) or not isinstance(current, (int, float)):
        return None
    return float(current)


def evaluate_phase_de_pair(
    baseline_runtime: Sequence[Mapping[str, Any]],
    experimental_runtime: Sequence[Mapping[str, Any]],
    baseline_ground_truth: Sequence[Mapping[str, Any]],
    experimental_ground_truth: Sequence[Mapping[str, Any]],
    *,
    synchronization_tolerance_s: float = 0.15,
) -> dict[str, Any]:
    """Return paired S0/S1 or R0/R1 raw metrics without a gate verdict."""

    baseline = evaluate_phase_de_episode(
        baseline_runtime,
        baseline_ground_truth,
        synchronization_tolerance_s=synchronization_tolerance_s,
    )
    experimental = evaluate_phase_de_episode(
        experimental_runtime,
        experimental_ground_truth,
        synchronization_tolerance_s=synchronization_tolerance_s,
    )
    phase = baseline["phase"]
    if experimental["phase"] != phase:
        raise EvaluationError("paired episodes must belong to the same phase")
    expected = PHASE_DE_PAIRS[phase]
    if (baseline["arm"], experimental["arm"]) != expected:
        raise EvaluationError(
            f"Phase {phase} pair must be ordered {expected[0]}/{expected[1]}"
        )
    if baseline["seed"] != experimental["seed"]:
        raise EvaluationError("paired episodes must use the same seed")
    if baseline["fault"] != experimental["fault"]:
        raise EvaluationError("paired episodes must use the same fault")

    paths = (
        "localization.time_to_ready_s",
        "localization.time_to_recover_s",
        "localization.gt_position_error_m.median",
        "localization.gt_position_error_m.p95",
        "localization.gt_yaw_error_deg.median",
        "localization.gt_yaw_error_deg.p95",
        "region.wrong_region_duration_s",
        "region.wrong_region_distance_m",
        "route.collision_count",
        "route.path_length_m",
        "route.route_duration_s",
        "route.episode_duration_s",
        "pause.latency_s",
        "module1.entropy.median",
        "module1.reliability.median",
        "module1.ood_probability.median",
        "module1.dominant_mass.median",
        "module1.dominant_covariance_m2.trace.median",
    )
    paired: dict[str, Any] = {}
    for path in paths:
        left = _nested_number(baseline, path)
        right = _nested_number(experimental, path)
        paired[path] = {
            "baseline": left,
            "experimental": right,
            "experimental_minus_baseline": (
                None if left is None or right is None else right - left
            ),
        }
    return {
        "schema": "bio_nav_v6_phase_de_localization_pair_metrics_v1",
        "phase": phase,
        "arms": list(expected),
        "seed": baseline["seed"],
        "fault": baseline["fault"],
        "evaluation_kind": "PAIRED_ENGINEERING_RAW_METRICS_ONLY",
        "formal_gate": False,
        "ground_truth_policy": "separate_passive_offline_stream",
        "baseline": baseline,
        "experimental": experimental,
        "paired_metrics": paired,
    }


def evaluate_phase_de_pair_files(
    baseline_runtime_path: str | Path,
    experimental_runtime_path: str | Path,
    baseline_ground_truth_path: str | Path,
    experimental_ground_truth_path: str | Path,
    *,
    synchronization_tolerance_s: float = 0.15,
) -> dict[str, Any]:
    return evaluate_phase_de_pair(
        load_phase_de_jsonl(baseline_runtime_path),
        load_phase_de_jsonl(experimental_runtime_path),
        load_phase_de_jsonl(baseline_ground_truth_path, ground_truth=True),
        load_phase_de_jsonl(experimental_ground_truth_path, ground_truth=True),
        synchronization_tolerance_s=synchronization_tolerance_s,
    )


def _ros_stamp_s(message: Any) -> float:
    try:
        stamp = message.header.stamp
        sec = stamp.sec
        nanosec = stamp.nanosec
        if (
            isinstance(sec, bool)
            or not isinstance(sec, int)
            or isinstance(nanosec, bool)
            or not isinstance(nanosec, int)
            or sec < 0
            or not 0 <= nanosec < 1_000_000_000
        ):
            raise ValueError
        seconds = sec + nanosec / 1.0e9
    except (AttributeError, TypeError, ValueError) as exc:
        raise EvaluationError("Ground Truth odometry requires header.stamp") from exc
    return _finite(seconds, "ground_truth_odom.header.stamp")


def _odom_pose(message: Any) -> tuple[float, float, float]:
    try:
        pose = message.pose.pose
        x = _finite(pose.position.x, "ground_truth_odom.pose.position.x")
        y = _finite(pose.position.y, "ground_truth_odom.pose.position.y")
        qx = _finite(pose.orientation.x, "ground_truth_odom.pose.orientation.x")
        qy = _finite(pose.orientation.y, "ground_truth_odom.pose.orientation.y")
        qz = _finite(pose.orientation.z, "ground_truth_odom.pose.orientation.z")
        qw = _finite(pose.orientation.w, "ground_truth_odom.pose.orientation.w")
    except AttributeError as exc:
        raise EvaluationError("Ground Truth message must be nav_msgs/msg/Odometry") from exc
    yaw = math.degrees(
        math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )
    )
    return x, y, yaw


def _canonical_region_id(x: float, y: float) -> str:
    column = math.floor(x + 8.0)
    row = math.floor(y + 8.0)
    if not (0 <= column < 16 and 0 <= row < 16):
        raise EvaluationError(f"Ground Truth pose ({x}, {y}) is outside the 16 m canvas")
    state = row * 16 + column
    try:
        return _CANONICAL_STATE_TO_REGION[state]
    except KeyError as exc:
        raise EvaluationError(
            f"Ground Truth pose ({x}, {y}) maps to non-enrolled state {state}"
        ) from exc


def _split_stamp_epochs(messages: Iterable[Any]) -> list[list[tuple[float, Any]]]:
    """Split ordered bag samples when simulation time moves backwards on reset."""

    epochs: list[list[tuple[float, Any]]] = []
    current: list[tuple[float, Any]] = []
    previous: float | None = None
    for message in messages:
        stamp_s = _ros_stamp_s(message)
        if previous is not None and stamp_s < previous:
            if current:
                epochs.append(current)
            current = []
        current.append((stamp_s, message))
        previous = stamp_s
    if current:
        epochs.append(current)
    if not epochs:
        raise EvaluationError(f"bag contains no {GROUND_TRUTH_ODOM_TOPIC} messages")
    return epochs


def extract_phase_de_ground_truth(
    runtime_events: Sequence[Mapping[str, Any]],
    odometry_messages: Iterable[Any],
    *,
    synchronization_tolerance_s: float = 0.15,
) -> list[dict[str, Any]]:
    """Align passive GT odometry to runtime estimate stamps after reset.

    This function is intentionally ROS-independent so focused tests can pass a
    fake message iterable.  The CLI is the only layer that opens an MCAP bag.
    """

    if synchronization_tolerance_s < 0.0:
        raise EvaluationError("synchronization_tolerance_s must be non-negative")
    identity = _phase_de_identity(runtime_events)
    starts = sorted(
        _event_stamp(row, "episode_start")
        for row in runtime_events
        if row["event"] == "episode_start"
    )
    ends = sorted(
        _event_stamp(row, "episode_end")
        for row in runtime_events
        if row["event"] == "episode_end"
    )
    start_s, end_s = starts[0], ends[0]
    if end_s < start_s:
        raise EvaluationError("episode_end precedes episode_start")
    target_stamps = sorted(
        _event_stamp(row, "estimated_pose")
        for row in runtime_events
        if row["event"] == "estimated_pose" and start_s <= float(row["stamp_s"]) <= end_s
    )
    if not target_stamps:
        raise EvaluationError("runtime contains no estimated_pose events in the episode")

    best: tuple[int, int, list[tuple[float, Any]]] | None = None
    for epoch_index, epoch in enumerate(_split_stamp_epochs(odometry_messages)):
        bounded = [item for item in epoch if start_s <= item[0] <= end_s]
        matched = sum(
            bool(bounded)
            and min(abs(item[0] - stamp_s) for item in bounded)
            <= synchronization_tolerance_s
            for stamp_s in target_stamps
        )
        candidate = (matched, epoch_index, bounded)
        if best is None or candidate[:2] > best[:2]:
            best = candidate
    assert best is not None
    matched_count, _, samples = best
    if matched_count != len(target_stamps):
        raise EvaluationError(
            "passive Ground Truth does not cover every estimated_pose stamp "
            f"within {synchronization_tolerance_s:.3f}s "
            f"({matched_count}/{len(target_stamps)} matched)"
        )

    selected: dict[int, tuple[float, Any]] = {}
    for target in target_stamps:
        sample_index, sample = min(
            enumerate(samples),
            key=lambda item: (abs(item[1][0] - target), item[1][0], item[0]),
        )
        selected[sample_index] = sample

    output: list[dict[str, Any]] = []
    for _, message in (selected[index] for index in sorted(selected)):
        stamp_s = _ros_stamp_s(message)
        x, y, yaw_deg = _odom_pose(message)
        output.append(
            {
                "schema": PHASE_DE_GT_SCHEMA,
                **identity,
                "event": "ground_truth_pose",
                "stamp_s": stamp_s,
                "x": x,
                "y": y,
                "yaw_deg": yaw_deg,
                "region_id": _canonical_region_id(x, y),
            }
        )
    return output


def _iter_ground_truth_odometry(bag_path: str | Path) -> Iterator[Any]:
    """Deserialize only `/ground_truth/odom` from one rosbag2 MCAP."""

    source = Path(bag_path).expanduser().resolve()
    if not source.exists():
        raise EvaluationError(f"bag does not exist: {source}")
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise EvaluationError(
            "rosbag2_py, rclpy, and rosidl_runtime_py are required to read MCAP"
        ) from exc

    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(
            rosbag2_py.StorageOptions(uri=str(source), storage_id="mcap"),
            rosbag2_py.ConverterOptions(
                input_serialization_format="", output_serialization_format=""
            ),
        )
    except RuntimeError as exc:
        raise EvaluationError(f"cannot open MCAP bag {source}: {exc}") from exc
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if GROUND_TRUTH_ODOM_TOPIC not in topic_types:
        raise EvaluationError(f"bag has no {GROUND_TRUTH_ODOM_TOPIC} topic")
    if topic_types[GROUND_TRUTH_ODOM_TOPIC] != "nav_msgs/msg/Odometry":
        raise EvaluationError(
            f"{GROUND_TRUTH_ODOM_TOPIC} must use nav_msgs/msg/Odometry"
        )
    message_type = get_message(topic_types[GROUND_TRUTH_ODOM_TOPIC])
    reader.set_filter(rosbag2_py.StorageFilter(topics=[GROUND_TRUTH_ODOM_TOPIC]))
    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        if topic == GROUND_TRUTH_ODOM_TOPIC:
            yield deserialize_message(serialized, message_type)


def _write_jsonl(path: str | Path, rows: Sequence[Mapping[str, Any]]) -> None:
    destination = Path(path).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        "".join(
            json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n"
            for row in rows
        ),
        encoding="utf-8",
    )


def _write_json(path: str | Path, value: Mapping[str, Any]) -> None:
    destination = Path(path).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def _evaluate_cli_inputs(
    runtime_paths: Sequence[str | Path],
    ground_truth_paths: Sequence[str | Path],
    *,
    synchronization_tolerance_s: float,
) -> dict[str, Any]:
    if len(runtime_paths) != len(ground_truth_paths) or len(runtime_paths) not in {1, 2}:
        raise EvaluationError(
            "evaluate requires one runtime/GT pair or two runtime/GT pairs"
        )
    runtimes = [load_phase_de_jsonl(path) for path in runtime_paths]
    truths = [load_phase_de_jsonl(path, ground_truth=True) for path in ground_truth_paths]
    if len(runtimes) == 1:
        return evaluate_phase_de_episode(
            runtimes[0],
            truths[0],
            synchronization_tolerance_s=synchronization_tolerance_s,
        )

    identities = [_phase_de_identity(events) for events in runtimes]
    if identities[0]["phase"] != identities[1]["phase"]:
        raise EvaluationError("paired episodes must belong to the same phase")
    expected = PHASE_DE_PAIRS[identities[0]["phase"]]
    by_arm = {
        identity["arm"]: (runtime, truth)
        for identity, runtime, truth in zip(identities, runtimes, truths)
    }
    if set(by_arm) != set(expected):
        raise EvaluationError(
            f"Phase {identities[0]['phase']} evaluate pair must contain {expected[0]}/{expected[1]}"
        )
    return evaluate_phase_de_pair(
        by_arm[expected[0]][0],
        by_arm[expected[1]][0],
        by_arm[expected[0]][1],
        by_arm[expected[1]][1],
        synchronization_tolerance_s=synchronization_tolerance_s,
    )


def _argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="v6_localization_causal_evaluator",
        description="Offline passive-GT extraction and Phase D/E evaluation",
    )
    commands = parser.add_subparsers(dest="command", required=True)
    extract = commands.add_parser("extract-gt")
    extract.add_argument("--bag", required=True)
    extract.add_argument("--episode-jsonl", required=True)
    extract.add_argument("--output", required=True)
    extract.add_argument("--synchronization-tolerance-s", type=float, default=0.15)

    evaluate = commands.add_parser("evaluate")
    evaluate.add_argument("--runtime-jsonl", action="append", required=True)
    evaluate.add_argument("--gt-jsonl", action="append", required=True)
    evaluate.add_argument("--output", required=True)
    evaluate.add_argument("--synchronization-tolerance-s", type=float, default=0.15)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = _argument_parser()
    args = parser.parse_args(argv)
    try:
        if args.command == "extract-gt":
            runtime = load_phase_de_jsonl(args.episode_jsonl)
            rows = extract_phase_de_ground_truth(
                runtime,
                _iter_ground_truth_odometry(args.bag),
                synchronization_tolerance_s=args.synchronization_tolerance_s,
            )
            _write_jsonl(args.output, rows)
        else:
            result = _evaluate_cli_inputs(
                args.runtime_jsonl,
                args.gt_jsonl,
                synchronization_tolerance_s=args.synchronization_tolerance_s,
            )
            _write_json(args.output, result)
    except (EvaluationError, ImportError, OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
