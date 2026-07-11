"""Pure, dependency-free metrics used by navigation experiments."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Iterable, Mapping, Sequence


PLAN_POSITION_TOLERANCE_M = 0.25
PLAN_ORIENTATION_TOLERANCE_RAD = math.radians(10.0)
PLAN_STATIC_SUCCESS_MIN_PERCENT = 95.0
PLAN_DYNAMIC_SUCCESS_MIN_PERCENT = 90.0
PLAN_NAVIGATION_SUCCESS_MIN_PERCENT = 90.0
PLAN_PATH_DEVIATION_MAX_PERCENT = 20.0
PLAN_INCREMENTAL_IMPROVEMENT_MIN_PERCENT = 30.0


def _finite(value: float, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def wrap_angle(angle_rad: float) -> float:
    """Wrap a radian angle to the half-open interval [-pi, pi)."""
    angle = _finite(angle_rad, "angle_rad")
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _point_xy(point: Sequence[float], name: str) -> tuple[float, float]:
    if len(point) < 2:
        raise ValueError(f"{name} must contain x and y")
    return _finite(point[0], f"{name}.x"), _finite(point[1], f"{name}.y")


def path_length(points: Iterable[Sequence[float]]) -> float:
    """Return planar polyline length in the input coordinate frame."""
    parsed = [_point_xy(point, "point") for point in points]
    return sum(
        math.hypot(x1 - x0, y1 - y0)
        for (x0, y0), (x1, y1) in zip(parsed, parsed[1:])
    )


def _paired(
    estimates: Iterable[Sequence[float]],
    ground_truth: Iterable[Sequence[float]],
) -> tuple[list[tuple[float, float]], list[tuple[float, float]]]:
    estimate_points = [_point_xy(point, "estimate") for point in estimates]
    truth_points = [_point_xy(point, "ground_truth") for point in ground_truth]
    if len(estimate_points) != len(truth_points):
        raise ValueError("estimate and ground-truth sample counts must match")
    if not estimate_points:
        raise ValueError("at least one paired sample is required")
    return estimate_points, truth_points


def translation_errors(
    estimates: Iterable[Sequence[float]],
    ground_truth: Iterable[Sequence[float]],
) -> tuple[float, ...]:
    estimate_points, truth_points = _paired(estimates, ground_truth)
    return tuple(
        math.hypot(ex - gx, ey - gy)
        for (ex, ey), (gx, gy) in zip(estimate_points, truth_points)
    )


def angular_errors(
    estimate_yaws_rad: Iterable[float],
    ground_truth_yaws_rad: Iterable[float],
) -> tuple[float, ...]:
    estimates = [_finite(value, "estimate_yaw") for value in estimate_yaws_rad]
    truth = [_finite(value, "ground_truth_yaw") for value in ground_truth_yaws_rad]
    if len(estimates) != len(truth):
        raise ValueError("estimate and ground-truth sample counts must match")
    if not estimates:
        raise ValueError("at least one paired sample is required")
    return tuple(wrap_angle(estimate - actual) for estimate, actual in zip(estimates, truth))


def percentile(values: Iterable[float], percentile_value: float) -> float:
    """Compute a linearly interpolated percentile without NumPy."""
    ordered = sorted(_finite(value, "sample") for value in values)
    if not ordered:
        raise ValueError("at least one sample is required")
    quantile = _finite(percentile_value, "percentile")
    if not 0.0 <= quantile <= 100.0:
        raise ValueError("percentile must be between 0 and 100")
    rank = (len(ordered) - 1) * quantile / 100.0
    lower = math.floor(rank)
    upper = math.ceil(rank)
    if lower == upper:
        return ordered[lower]
    fraction = rank - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


@dataclass(frozen=True)
class ErrorStatistics:
    rmse: float
    maximum: float
    percentile_95: float
    sample_count: int

    def as_dict(self) -> dict[str, float | int]:
        return asdict(self)


def error_statistics(errors: Iterable[float]) -> ErrorStatistics:
    absolute = [abs(_finite(value, "error")) for value in errors]
    if not absolute:
        raise ValueError("at least one error sample is required")
    return ErrorStatistics(
        rmse=math.sqrt(sum(value * value for value in absolute) / len(absolute)),
        maximum=max(absolute),
        percentile_95=percentile(absolute, 95.0),
        sample_count=len(absolute),
    )


def translation_error_statistics(
    estimates: Iterable[Sequence[float]],
    ground_truth: Iterable[Sequence[float]],
) -> ErrorStatistics:
    return error_statistics(translation_errors(estimates, ground_truth))


def angular_error_statistics(
    estimate_yaws_rad: Iterable[float],
    ground_truth_yaws_rad: Iterable[float],
) -> ErrorStatistics:
    return error_statistics(angular_errors(estimate_yaws_rad, ground_truth_yaws_rad))


def path_length_deviation_percent(executed_length: float, optimal_length: float) -> float:
    executed = _finite(executed_length, "executed_length")
    optimal = _finite(optimal_length, "optimal_length")
    if executed < 0.0:
        raise ValueError("executed_length must be non-negative")
    if optimal <= 0.0:
        raise ValueError("optimal_length must be positive")
    return abs(executed - optimal) / optimal * 100.0


def success_rate_percent(successes: int, total: int) -> float:
    if isinstance(successes, bool) or isinstance(total, bool):
        raise TypeError("successes and total must be integers")
    if not isinstance(successes, int) or not isinstance(total, int):
        raise TypeError("successes and total must be integers")
    if total <= 0:
        raise ValueError("total must be positive")
    if successes < 0 or successes > total:
        raise ValueError("successes must be between zero and total")
    return successes / total * 100.0


def navigation_success_rate(successes: int, total: int) -> float:
    return success_rate_percent(successes, total)


def static_avoidance_rate(successes: int, total: int) -> float:
    return success_rate_percent(successes, total)


def dynamic_avoidance_rate(successes: int, total: int) -> float:
    return success_rate_percent(successes, total)


def incremental_time_improvement_percent(full_time_sec: float, incremental_time_sec: float) -> float:
    full = _finite(full_time_sec, "full_time_sec")
    incremental = _finite(incremental_time_sec, "incremental_time_sec")
    if full <= 0.0:
        raise ValueError("full_time_sec must be positive")
    if incremental < 0.0:
        raise ValueError("incremental_time_sec must be non-negative")
    return (full - incremental) / full * 100.0


@dataclass(frozen=True)
class SingleRunThresholds:
    position_tolerance_m: float = PLAN_POSITION_TOLERANCE_M
    orientation_tolerance_rad: float = PLAN_ORIENTATION_TOLERANCE_RAD
    final_linear_speed_tolerance_mps: float = 0.02
    final_angular_speed_tolerance_radps: float = 0.05

    def __post_init__(self) -> None:
        for name, value in asdict(self).items():
            if _finite(value, name) < 0.0:
                raise ValueError(f"{name} must be non-negative")


@dataclass(frozen=True)
class SingleRunObservation:
    nav2_succeeded: bool
    ground_truth_available: bool
    ground_truth_position_error_m: float
    ground_truth_orientation_error_rad: float | None
    orientation_required: bool
    collision_detected: bool
    localization_lost: bool
    tf_interrupted: bool
    timed_out: bool
    collision_monitor_locked: bool
    final_linear_speed_mps: float
    final_angular_speed_radps: float
    safety_observability_complete: bool = True


@dataclass(frozen=True)
class SingleRunEvaluation:
    success: bool
    failure_reasons: tuple[str, ...]

    def as_dict(self) -> dict[str, object]:
        return {"success": self.success, "failure_reasons": list(self.failure_reasons)}


def evaluate_single_run(
    observation: SingleRunObservation,
    thresholds: SingleRunThresholds = SingleRunThresholds(),
) -> SingleRunEvaluation:
    """Evaluate all ten single-run conditions from plan.md section 12.1."""
    position_error = _finite(
        observation.ground_truth_position_error_m,
        "ground_truth_position_error_m",
    )
    linear_speed = abs(_finite(observation.final_linear_speed_mps, "final_linear_speed_mps"))
    angular_speed = abs(
        _finite(observation.final_angular_speed_radps, "final_angular_speed_radps")
    )
    reasons: list[str] = []
    if not observation.nav2_succeeded:
        reasons.append("nav2_action_failed")
    if not observation.ground_truth_available:
        reasons.append("ground_truth_unavailable")
    elif position_error > thresholds.position_tolerance_m:
        reasons.append("goal_position_error")
    if observation.orientation_required:
        if observation.ground_truth_orientation_error_rad is None:
            reasons.append("goal_orientation_unavailable")
        elif abs(wrap_angle(observation.ground_truth_orientation_error_rad)) > (
            thresholds.orientation_tolerance_rad
        ):
            reasons.append("goal_orientation_error")
    if observation.collision_detected:
        reasons.append("collision_detected")
    if observation.localization_lost:
        reasons.append("localization_lost")
    if observation.tf_interrupted:
        reasons.append("tf_interrupted")
    if observation.timed_out:
        reasons.append("timed_out")
    if observation.collision_monitor_locked:
        reasons.append("collision_monitor_locked")
    if (
        linear_speed > thresholds.final_linear_speed_tolerance_mps
        or angular_speed > thresholds.final_angular_speed_tolerance_radps
    ):
        reasons.append("final_velocity_not_zero")
    if not observation.safety_observability_complete:
        reasons.append("safety_status_unavailable")
    return SingleRunEvaluation(success=not reasons, failure_reasons=tuple(reasons))


def threshold_summary() -> Mapping[str, float]:
    return {
        "static_success_min_percent": PLAN_STATIC_SUCCESS_MIN_PERCENT,
        "dynamic_success_min_percent": PLAN_DYNAMIC_SUCCESS_MIN_PERCENT,
        "navigation_success_min_percent": PLAN_NAVIGATION_SUCCESS_MIN_PERCENT,
        "path_deviation_max_percent": PLAN_PATH_DEVIATION_MAX_PERCENT,
        "incremental_improvement_min_percent": PLAN_INCREMENTAL_IMPROVEMENT_MIN_PERCENT,
    }
