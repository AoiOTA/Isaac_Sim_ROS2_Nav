"""Passive absolute-map evaluator for the V6 L0--L3 localization study.

Ground Truth enters only through this offline module.  The runtime dispatcher
must record estimated-state and control topics without subscribing to GT.
Errors are absolute map-frame errors; no first-frame or trajectory alignment is
performed.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import statistics
from typing import Any, Iterable, Mapping, Sequence


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
