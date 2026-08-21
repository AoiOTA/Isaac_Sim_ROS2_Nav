"""Offline V6 IMU regime analysis; this module never controls a robot."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import statistics
from typing import Any, Iterable, Sequence

import yaml


K_MIN = 0.90
K_MAX = 1.02
K_STEP = 0.0001
YAW_LIMIT_RAD = math.radians(5.0)
MAX_SAMPLE_GAP_S = 0.25
MIN_WINDOW_DURATION_S = 0.50
EXPECTED_PRIMITIVE_IDS = (
    "cw_360",
    "ccw_360",
    "arc_v005_cw",
    "arc_v005_ccw",
    "arc_v010_cw",
    "arc_v010_ccw",
    "arc_v025_cw",
    "arc_v025_ccw",
    "s_route",
)
EXPECTED_SEEDS = tuple(range(8610, 8619))
EXPECTED_SEGMENT_COUNTS = (1, 1, 1, 1, 1, 1, 1, 1, 3)
EXPECTED_COMMANDS = (
    ((0.0, -0.50),),
    ((0.0, 0.50),),
    ((0.05, -0.50),),
    ((0.05, 0.50),),
    ((0.10, -0.50),),
    ((0.10, 0.50),),
    ((0.25, -0.50),),
    ((0.25, 0.50),),
    ((0.25, 0.45), (0.25, -0.45), (0.25, 0.45)),
)
EXPECTED_DURATIONS = (
    (12.566,),
    (12.566,),
    (4.0,),
    (4.0,),
    (4.0,),
    (4.0,),
    (4.0,),
    (4.0,),
    (2.5, 5.0, 2.5),
)
EXPECTED_THRESHOLDS = {
    "linear_mae_mps": 0.06,
    "angular_mae_radps": 0.12,
    "radius_relative_error_percent": 20.0,
    "tracking_fraction": 0.85,
    "transition_latency_sec": 0.45,
    "overshoot_ratio": 1.25,
    "wrong_direction_fraction": 0.05,
}
EXPECTED_STATIONARY = {
    "id": "stationary_reference",
    "duration_sec": 10.0,
    "reset_seed": 8609,
}
EXPECTED_TRACE_PROVENANCE = {
    "contract": "v6_imu_regime_flat20_v1",
    "environment_usd": (
        "/home/lyb/isaacsim_assets/Assets/Isaac/6.0/Isaac/Environments/"
        "Grid/default_environment.usd"
    ),
    "spawn_pose": "flat20_start",
    "odometry_mode": "realistic",
    "navigation_mode": "mapping",
    "dynamic_obstacles_enabled": False,
    "ground_truth_enabled": True,
}
EXPECTED_TOPIC_TYPES = {
    "/imu/data_raw": "sensor_msgs/msg/Imu",
    "/imu/data": "sensor_msgs/msg/Imu",
    "/ground_truth/odom": "nav_msgs/msg/Odometry",
}
GOAL_TOPIC_TYPES = {
    "/imu/data_raw": "sensor_msgs/msg/Imu",
    "/ground_truth/odom": "nav_msgs/msg/Odometry",
    "/cmd_vel": "geometry_msgs/msg/Twist",
    "/simulation/reset_event": "std_msgs/msg/Empty",
    "/simulation/collision": "std_msgs/msg/Bool",
    "/bio_nav/route_goal_complete": "std_msgs/msg/Bool",
    "/rosout": "rcl_interfaces/msg/Log",
}
GOAL_OPTIONAL_TOPIC_TYPES = {
    "/imu/data": "sensor_msgs/msg/Imu",
    "/bio_nav/route_goal": "geometry_msgs/msg/PoseStamped",
}
RESOURCE_MANIFEST = "v6_imu_regime_resources.json"


class EvidenceError(RuntimeError):
    """Evidence problem with an explicit FAIL or AMBIGUOUS classification."""

    def __init__(self, verdict: str, code: str, detail: str) -> None:
        super().__init__(detail)
        self.verdict = verdict
        self.code = code
        self.detail = detail


def _evidence_issue(verdict: str, code: str, detail: str) -> EvidenceError:
    if verdict not in {"FAIL", "AMBIGUOUS"}:
        raise ValueError("invalid evidence verdict")
    return EvidenceError(verdict, code, detail)


@dataclass(frozen=True)
class ScalarSample:
    stamp_s: float
    value: float


@dataclass(frozen=True)
class YawSample:
    stamp_s: float
    yaw_rad: float


class McapStreams(dict[str, list[ScalarSample | YawSample]]):
    """Topic samples plus the exact bag/type/count provenance used."""

    def __init__(self, *args: Any, provenance: dict[str, Any], **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self.provenance = provenance


@dataclass(frozen=True)
class DiagnosticResources:
    config_path: Path
    spawn_poses_path: Path
    config: dict[str, Any]
    identity: dict[str, Any]


def _strict_int(value: Any, *, name: str, minimum: int = 0) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise _evidence_issue("FAIL", "integer_contract", f"{name} must be an integer >= {minimum}")
    return value


def _finite(value: Any, *, name: str, positive: bool = False) -> float:
    if isinstance(value, bool):
        raise _evidence_issue("FAIL", "numeric_contract", f"{name} must be numeric")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise _evidence_issue("FAIL", "numeric_contract", f"{name} must be numeric") from exc
    if not math.isfinite(result) or (positive and result <= 0.0):
        qualifier = "positive and finite" if positive else "finite"
        raise _evidence_issue("FAIL", "numeric_contract", f"{name} must be {qualifier}")
    return result


def _installed_resource_paths() -> tuple[Path, Path]:
    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError as exc:
        raise _evidence_issue(
            "AMBIGUOUS", "installed_resources_unavailable",
            "ament_index_python is unavailable; pass explicit --config and --spawn-poses-file",
        ) from exc
    try:
        share = Path(get_package_share_directory("robot_experiments")).resolve()
    except Exception as exc:
        raise _evidence_issue(
            "AMBIGUOUS", "installed_resources_unavailable",
            f"robot_experiments package share is unavailable: {exc}",
        ) from exc
    manifest_path = share / "config" / RESOURCE_MANIFEST
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise _evidence_issue(
            "AMBIGUOUS", "resource_manifest_unreadable",
            f"installed IMU resource manifest is unavailable: {exc}",
        ) from exc
    if (
        not isinstance(manifest, dict)
        or manifest.get("schema_version") != 1
        or manifest.get("contract") != "v6_imu_regime_flat20_v2"
        or not isinstance(manifest.get("diagnostic_config"), str)
        or not isinstance(manifest.get("spawn_poses"), str)
    ):
        raise _evidence_issue("FAIL", "resource_manifest_invalid", "installed IMU resource manifest is invalid")
    return (
        (share / manifest["diagnostic_config"]).resolve(),
        (share / manifest["spawn_poses"]).resolve(),
    )


def resolve_diagnostic_resources(
    config_path: Path | None = None,
    spawn_poses_path: Path | None = None,
) -> DiagnosticResources:
    """Resolve and validate the exact source-first or installed contract."""

    if (config_path is None) != (spawn_poses_path is None):
        raise _evidence_issue(
            "FAIL", "resource_pair_required",
            "--config and --spawn-poses-file must be supplied together",
        )
    if config_path is None:
        config_path, spawn_poses_path = _installed_resource_paths()
    assert config_path is not None and spawn_poses_path is not None
    config_path = config_path.expanduser().resolve()
    spawn_poses_path = spawn_poses_path.expanduser().resolve()
    if not config_path.is_file() or not spawn_poses_path.is_file():
        raise _evidence_issue("AMBIGUOUS", "resource_missing", "diagnostic config or flat20 spawn resource is missing")
    try:
        config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        spawn = yaml.safe_load(spawn_poses_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise _evidence_issue("FAIL", "resource_yaml_invalid", f"diagnostic resource YAML is invalid: {exc}") from exc
    if not isinstance(config, dict) or config.get("schema_version") != 1:
        raise _evidence_issue("FAIL", "diagnostic_config_invalid", "diagnostic config schema is invalid")
    stationary = config.get("stationary_reference")
    primitives = config.get("primitives")
    thresholds = config.get("thresholds")
    if not isinstance(stationary, dict) or not isinstance(primitives, list) or not isinstance(thresholds, dict):
        raise _evidence_issue("FAIL", "diagnostic_config_invalid", "diagnostic config is incomplete")
    _strict_int(config.get("reset_seed"), name="config.reset_seed")
    _strict_int(stationary.get("reset_seed"), name="config.stationary_reference.reset_seed")
    if (
        config.get("spawn_pose_name") != "flat20_start"
        or stationary.get("id") != EXPECTED_STATIONARY["id"]
        or stationary.get("reset_seed") != EXPECTED_STATIONARY["reset_seed"]
        or _finite(stationary.get("duration_sec"), name="stationary duration", positive=True)
        != EXPECTED_STATIONARY["duration_sec"]
        or config.get("reset_seed") != EXPECTED_SEEDS[0]
        or tuple(item.get("id") if isinstance(item, dict) else None for item in primitives)
        != EXPECTED_PRIMITIVE_IDS
    ):
        raise _evidence_issue("FAIL", "diagnostic_config_mismatch", "diagnostic identity/order/seed/stationary contract changed")
    matrix: list[list[dict[str, float]]] = []
    for identifier, primitive, commands, durations in zip(
        EXPECTED_PRIMITIVE_IDS, primitives, EXPECTED_COMMANDS, EXPECTED_DURATIONS
    ):
        segments = primitive.get("segments") if isinstance(primitive, dict) else None
        if not isinstance(segments, list) or len(segments) != len(commands):
            raise _evidence_issue("FAIL", "diagnostic_config_mismatch", f"{identifier} segment count changed")
        observed: list[dict[str, float]] = []
        for index, (segment, command, duration) in enumerate(zip(segments, commands, durations)):
            if not isinstance(segment, dict):
                raise _evidence_issue("FAIL", "diagnostic_config_mismatch", f"{identifier}[{index}] is invalid")
            actual = (
                _finite(segment.get("linear_x"), name=f"{identifier}[{index}].linear_x"),
                _finite(segment.get("angular_z"), name=f"{identifier}[{index}].angular_z"),
            )
            actual_duration = _finite(segment.get("duration_sec"), name=f"{identifier}[{index}].duration_sec", positive=True)
            if actual != command or actual_duration != duration:
                raise _evidence_issue("FAIL", "diagnostic_config_mismatch", f"{identifier}[{index}] command/duration changed")
            observed.append({"duration_sec": actual_duration, "linear_x": actual[0], "angular_z": actual[1]})
        matrix.append(observed)
    threshold_identity = {
        str(key): _finite(value, name=f"thresholds.{key}", positive=True)
        for key, value in sorted(thresholds.items())
    }
    if (
        threshold_identity != EXPECTED_THRESHOLDS
        or _finite(config.get("command_rate_hz"), name="config.command_rate_hz", positive=True)
        != 20.0
    ):
        raise _evidence_issue(
            "FAIL", "diagnostic_config_mismatch",
            "diagnostic thresholds or command rate changed",
        )
    poses = spawn.get("spawn_poses") if isinstance(spawn, dict) else None
    flat20 = poses.get("flat20_start") if isinstance(poses, dict) else None
    mapping = flat20.get("map") if isinstance(flat20, dict) else None
    usd = flat20.get("usd") if isinstance(flat20, dict) else None
    if (
        spawn.get("schema_version") != 1
        or not isinstance(mapping, dict)
        or not isinstance(usd, dict)
        or mapping.get("calibrated") is not True
        or mapping.get("map_version") != "flat20_v1"
        or usd.get("position") != [0.0, 0.0, 0.0635]
        or usd.get("yaw_deg") != 0.0
    ):
        raise _evidence_issue("FAIL", "spawn_resource_mismatch", "flat20 spawn resource contract changed")
    identity = {
        "contract": "v6_imu_regime_flat20_v2",
        "resolved_config_path": str(config_path),
        "resolved_spawn_poses_path": str(spawn_poses_path),
        "stationary": {
            "id": stationary["id"],
            "duration_sec": float(stationary["duration_sec"]),
            "reset_seed": stationary["reset_seed"],
        },
        "primitive_ids": list(EXPECTED_PRIMITIVE_IDS),
        "segments": matrix,
        "thresholds": threshold_identity,
    }
    return DiagnosticResources(config_path, spawn_poses_path, config, identity)


def _unwrap(values: Sequence[float]) -> list[float]:
    if not values:
        return []
    result = [float(values[0])]
    for value in values[1:]:
        previous_wrapped = result[-1]
        delta = math.atan2(
            math.sin(float(value) - previous_wrapped),
            math.cos(float(value) - previous_wrapped),
        )
        result.append(result[-1] + delta)
    return result


def _stamp_quality(samples: Sequence[ScalarSample | YawSample]) -> dict[str, object]:
    stamps = [float(sample.stamp_s) for sample in samples]
    values = [
        float(sample.value if isinstance(sample, ScalarSample) else sample.yaw_rad)
        for sample in samples
    ]
    duplicate = sum(right == left for left, right in zip(stamps, stamps[1:]))
    backward = sum(right < left for left, right in zip(stamps, stamps[1:]))
    nonfinite = sum(not math.isfinite(value) for value in stamps)
    positive_deltas = [
        right - left
        for left, right in zip(stamps, stamps[1:])
        if right > left and math.isfinite(right - left)
    ]
    return {
        "count": len(stamps),
        "duplicate_count": duplicate,
        "backward_count": backward,
        "nonfinite_count": nonfinite,
        "nonfinite_value_count": sum(not math.isfinite(value) for value in values),
        "median_period_s": (
            statistics.median(positive_deltas) if positive_deltas else None
        ),
    }


def _maximum_gap(samples: Sequence[ScalarSample | YawSample]) -> float | None:
    gaps = [
        right.stamp_s - left.stamp_s
        for left, right in zip(samples, samples[1:])
        if math.isfinite(right.stamp_s - left.stamp_s)
    ]
    return max(gaps) if gaps else None


def _window_scalar(
    samples: Sequence[ScalarSample], start_s: float, end_s: float
) -> list[ScalarSample]:
    return [sample for sample in samples if start_s <= sample.stamp_s <= end_s]


def _window_yaw(
    samples: Sequence[YawSample], start_s: float, end_s: float
) -> list[YawSample]:
    return [sample for sample in samples if start_s <= sample.stamp_s <= end_s]


def _integral(samples: Sequence[ScalarSample]) -> tuple[list[float], list[float]]:
    if not samples:
        return [], []
    times = [float(item.stamp_s) for item in samples]
    values = [float(item.value) for item in samples]
    cumulative = [0.0]
    for index in range(1, len(samples)):
        dt = times[index] - times[index - 1]
        if dt <= 0.0 or not math.isfinite(dt):
            cumulative.append(cumulative[-1])
            continue
        cumulative.append(
            cumulative[-1] + 0.5 * (values[index - 1] + values[index]) * dt
        )
    return times, cumulative


def _interpolate(times: Sequence[float], values: Sequence[float], stamp: float) -> float | None:
    if not times or stamp < times[0] or stamp > times[-1]:
        return None
    low = 0
    high = len(times) - 1
    while low < high:
        middle = (low + high) // 2
        if times[middle] < stamp:
            low = middle + 1
        else:
            high = middle
    if times[low] == stamp or low == 0:
        return float(values[low])
    left = low - 1
    span = times[low] - times[left]
    if span <= 0.0:
        return None
    fraction = (stamp - times[left]) / span
    return float(values[left]) + fraction * (float(values[low]) - float(values[left]))


def _aligned_errors(
    imu_times: Sequence[float], integrated: Sequence[float], gt: Sequence[YawSample]
) -> list[float]:
    gt_times = [sample.stamp_s for sample in gt]
    gt_unwrapped = _unwrap([sample.yaw_rad for sample in gt])
    if not gt_unwrapped:
        return []
    origin = gt_unwrapped[0]
    errors: list[float] = []
    for stamp, value in zip(imu_times, integrated):
        expected = _interpolate(gt_times, gt_unwrapped, stamp)
        if expected is not None:
            errors.append(float(value) - (expected - origin))
    return errors


def _rmse(values: Sequence[float]) -> float | None:
    return (
        math.sqrt(sum(value * value for value in values) / len(values))
        if values
        else None
    )


def _p95(values: Sequence[float]) -> float | None:
    if not values:
        return None
    ordered = sorted(abs(value) for value in values)
    return ordered[min(len(ordered) - 1, math.ceil(0.95 * len(ordered)) - 1)]


def _grid() -> list[float]:
    count = int(round((K_MAX - K_MIN) / K_STEP))
    return [round(K_MIN + index * K_STEP, 4) for index in range(count + 1)]


def _contiguous_interval(valid: Sequence[float]) -> list[list[float]]:
    if not valid:
        return []
    intervals: list[list[float]] = []
    start = previous = valid[0]
    for value in valid[1:]:
        if value - previous > K_STEP * 1.5:
            intervals.append([start, previous])
            start = value
        previous = value
    intervals.append([start, previous])
    return intervals


def analyze_segment(
    *,
    identifier: str,
    reset_generation: int,
    command_linear_mps: float,
    command_angular_radps: float,
    raw: Sequence[ScalarSample],
    corrected: Sequence[ScalarSample],
    ground_truth: Sequence[YawSample],
) -> dict[str, object]:
    """Compute one window on one shared, bounded interpolation grid."""

    qualities = {
        "raw": _stamp_quality(raw),
        "corrected": _stamp_quality(corrected),
        "ground_truth": _stamp_quality(ground_truth),
    }
    invalid_stamps = any(
        quality["duplicate_count"]
        or quality["backward_count"]
        or quality["nonfinite_count"]
        or quality["nonfinite_value_count"]
        for quality in qualities.values()
    )
    base = {
        "id": identifier,
        "reset_generation": int(reset_generation),
        "command_linear_mps": float(command_linear_mps),
        "command_angular_radps": float(command_angular_radps),
        "direction": (
            "CCW" if command_angular_radps > 0
            else "CW" if command_angular_radps < 0
            else "ZERO"
        ),
        "speed_bin_mps": abs(float(command_linear_mps)),
        "stamp_quality": qualities,
        "scale_interval_le_5deg": [],
    }
    if not all(
        math.isfinite(value)
        for value in (command_linear_mps, command_angular_radps)
    ):
        return {**base, "status": "DATA_INVALID", "reason": "nonfinite_command"}
    if invalid_stamps:
        return {**base, "status": "DATA_INVALID", "reason": "stamp_or_value_invalid"}
    if len(raw) < 3 or len(corrected) < 3 or len(ground_truth) < 3:
        return {**base, "status": "INSUFFICIENT_DATA"}
    t0 = max(raw[0].stamp_s, corrected[0].stamp_s, ground_truth[0].stamp_s)
    t1 = min(raw[-1].stamp_s, corrected[-1].stamp_s, ground_truth[-1].stamp_s)
    maximum_gaps = {
        "raw": _maximum_gap(raw),
        "corrected": _maximum_gap(corrected),
        "ground_truth": _maximum_gap(ground_truth),
    }
    if (
        not math.isfinite(t0)
        or not math.isfinite(t1)
        or t1 - t0 < MIN_WINDOW_DURATION_S
        or any(
            gap is None or gap > MAX_SAMPLE_GAP_S
            for gap in maximum_gaps.values()
        )
    ):
        return {
            **base,
            "status": "INSUFFICIENT_COVERAGE",
            "coverage": {
                "t0_s": t0,
                "t1_s": t1,
                "duration_s": t1 - t0,
                "maximum_gap_s": maximum_gaps,
                "maximum_allowed_gap_s": MAX_SAMPLE_GAP_S,
            },
        }
    raw_times = [item.stamp_s for item in raw]
    raw_values = [item.value for item in raw]
    corrected_times = [item.stamp_s for item in corrected]
    corrected_values = [item.value for item in corrected]
    gt_times = [item.stamp_s for item in ground_truth]
    gt_unwrapped = _unwrap([sample.yaw_rad for sample in ground_truth])
    grid = sorted({
        t0,
        t1,
        *[value for value in raw_times if t0 <= value <= t1],
        *[value for value in corrected_times if t0 <= value <= t1],
        *[value for value in gt_times if t0 <= value <= t1],
    })
    raw_grid = [_interpolate(raw_times, raw_values, stamp) for stamp in grid]
    corrected_grid = [
        _interpolate(corrected_times, corrected_values, stamp) for stamp in grid
    ]
    gt_grid = [_interpolate(gt_times, gt_unwrapped, stamp) for stamp in grid]
    if any(value is None for value in (*raw_grid, *corrected_grid, *gt_grid)):
        return {**base, "status": "INSUFFICIENT_COVERAGE"}
    raw_common = [ScalarSample(stamp, float(value)) for stamp, value in zip(grid, raw_grid)]
    corrected_common = [
        ScalarSample(stamp, float(value))
        for stamp, value in zip(grid, corrected_grid)
    ]
    _, raw_integrated = _integral(raw_common)
    _, corrected_integrated = _integral(corrected_common)
    gt_relative = [float(value) - float(gt_grid[0]) for value in gt_grid]
    gt_delta = gt_relative[-1]
    raw_delta = raw_integrated[-1]
    corrected_delta = corrected_integrated[-1]
    k_star = gt_delta / raw_delta if abs(raw_delta) > 1.0e-12 else None
    raw_errors = [value - expected for value, expected in zip(raw_integrated, gt_relative)]
    corrected_errors = [
        value - expected for value, expected in zip(corrected_integrated, gt_relative)
    ]
    allowed = [
        k
        for k in _grid()
        if abs(k * raw_delta - gt_delta) <= YAW_LIMIT_RAD
    ]
    duration = t1 - t0
    steady_start = t0 + min(0.5, max(0.0, duration * 0.1))
    steady_end = t1 - min(0.5, max(0.0, duration * 0.1))
    steady_raw = [
        item.value for item in raw_common if steady_start <= item.stamp_s <= steady_end
    ]
    gt_rate = gt_delta / duration if duration > 0.0 else None
    raw_rate = statistics.median(steady_raw) if steady_raw else None
    return {
        **base,
        "status": "STAMP_INVALID" if invalid_stamps else "OK",
        "coverage": {
            "t0_s": t0,
            "t1_s": t1,
            "duration_s": duration,
            "common_grid_count": len(grid),
            "maximum_gap_s": maximum_gaps,
            "maximum_allowed_gap_s": MAX_SAMPLE_GAP_S,
            "interpolation": "linear_no_extrapolation",
            "integration": "shared_grid_trapezoidal",
        },
        "ground_truth_yaw_delta_rad": gt_delta,
        "raw_yaw_delta_rad": raw_delta,
        "corrected_yaw_delta_rad": corrected_delta,
        "k_star": k_star,
        "raw_endpoint_error_rad": raw_delta - gt_delta,
        "corrected_endpoint_error_rad": corrected_delta - gt_delta,
        "raw_aligned_rmse_rad": _rmse(raw_errors),
        "raw_aligned_p95_rad": _p95(raw_errors),
        "corrected_aligned_rmse_rad": _rmse(corrected_errors),
        "corrected_aligned_p95_rad": _p95(corrected_errors),
        "steady_rate_ratio_gt_over_raw": (
            gt_rate / raw_rate
            if gt_rate is not None and raw_rate is not None and abs(raw_rate) > 1.0e-12
            else None
        ),
        "scale_interval_le_5deg": _contiguous_interval(allowed),
    }


def _interval_intersection(interval_sets: Sequence[Sequence[Sequence[float]]]) -> list[list[float]]:
    current = [[K_MIN, K_MAX]]
    for intervals in interval_sets:
        next_values: list[list[float]] = []
        for left in current:
            for right in intervals:
                low = max(float(left[0]), float(right[0]))
                high = min(float(left[1]), float(right[1]))
                if low <= high:
                    next_values.append([round(low, 4), round(high, 4)])
        current = next_values
    return current


def validate_goal_evidence(
    goal: dict[str, Any] | None,
    *,
    expected_mcap: Path | None = None,
) -> dict[str, Any]:
    """Validate metadata plus arrays derived by :func:`load_goal_mcap`."""

    if goal is None:
        raise _evidence_issue("AMBIGUOUS", "goal_missing", "goal evidence is missing")
    if not isinstance(goal, dict):
        raise _evidence_issue("FAIL", "goal_wrong_type", "goal evidence must be an object")
    required = {
        "schema_version",
        "source",
        "source_mcap",
        "reset_receipt",
        "outcome",
        "collision_detected",
        "bag_verified",
        "goal_window",
        "attempt_provenance",
        "stream_coverage",
        "raw_integrated_yaw_rad",
        "ground_truth_relative_yaw_rad",
    }
    missing = required - set(goal)
    if missing:
        raise _evidence_issue(
            "AMBIGUOUS", "goal_truncated", f"goal evidence missing fields: {sorted(missing)}"
        )
    if (
        goal["schema_version"] != 1
        or goal["source"] != "goal_mcap_derived"
        or goal["bag_verified"] is not True
    ):
        raise _evidence_issue("FAIL", "goal_provenance_invalid", "goal schema/source is invalid")
    if not isinstance(goal["source_mcap"], str) or not goal["source_mcap"]:
        raise _evidence_issue("AMBIGUOUS", "goal_source_missing", "goal source MCAP is missing")
    try:
        source_mcap = Path(goal["source_mcap"]).expanduser().resolve()
    except (TypeError, ValueError, OSError) as exc:
        raise _evidence_issue("FAIL", "goal_source_invalid", "goal source MCAP path is invalid") from exc
    if expected_mcap is not None and source_mcap != expected_mcap.expanduser().resolve():
        raise _evidence_issue("FAIL", "goal_source_mismatch", "goal metadata source_mcap does not match --goal-mcap")
    receipt = goal["reset_receipt"]
    if (
        not isinstance(receipt, dict)
        or any(
            isinstance(receipt.get(key), bool)
            or not isinstance(receipt.get(key), int)
            or receipt[key] < 0
            for key in ("requested_seed", "actual_seed")
        )
        or receipt.get("requested_seed") != receipt.get("actual_seed")
        or isinstance(receipt.get("generation"), bool)
        or not isinstance(receipt.get("generation"), int)
        or receipt["generation"] < 1
        or not isinstance(receipt.get("pose"), str)
        or not receipt.get("pose")
    ):
        raise _evidence_issue("FAIL", "goal_reset_invalid", "goal reset receipt is invalid")
    if goal["outcome"] != "SUCCEEDED" or not isinstance(goal["collision_detected"], bool):
        raise _evidence_issue("FAIL", "goal_outcome_invalid", "goal outcome/collision field is invalid")
    if goal["collision_detected"]:
        raise _evidence_issue("FAIL", "goal_collision", "goal evidence reports a collision")
    window = goal["goal_window"]
    if not isinstance(window, dict):
        raise _evidence_issue("FAIL", "goal_window_invalid", "goal window is invalid")
    start_s = _finite(window.get("start_s"), name="goal_window.start_s")
    end_s = _finite(window.get("end_s"), name="goal_window.end_s")
    if end_s <= start_s:
        raise _evidence_issue("FAIL", "goal_window_invalid", "goal window is empty or reversed")
    attempt = goal["attempt_provenance"]
    if (
        not isinstance(attempt, dict)
        or attempt.get("terminal_count") != 1
        or attempt.get("terminal_values") != [True]
        or not isinstance(attempt.get("terminal_timestamps_s"), list)
        or len(attempt["terminal_timestamps_s"]) != 1
    ):
        raise _evidence_issue("FAIL", "goal_attempt_invalid", "goal evidence is not bound to one successful terminal")
    coverage = goal["stream_coverage"]
    if not isinstance(coverage, dict) or coverage.get("maximum_allowed_gap_s") != MAX_SAMPLE_GAP_S:
        raise _evidence_issue("FAIL", "goal_coverage_invalid", "goal stream coverage contract is invalid")
    maximum_gaps = coverage.get("maximum_gap_s")
    if not isinstance(maximum_gaps, dict) or not {"raw", "ground_truth"}.issubset(maximum_gaps):
        raise _evidence_issue("FAIL", "goal_coverage_invalid", "goal stream gap evidence is incomplete")
    for name, value in maximum_gaps.items():
        gap = _finite(value, name=f"goal.stream_coverage.maximum_gap_s.{name}")
        if gap > MAX_SAMPLE_GAP_S:
            raise _evidence_issue("AMBIGUOUS", "goal_sample_gap", f"goal {name} stream gap exceeds {MAX_SAMPLE_GAP_S} s")
    raw = goal.get("raw_integrated_yaw_rad")
    gt = goal.get("ground_truth_relative_yaw_rad")
    if not isinstance(raw, list) or not isinstance(gt, list) or len(raw) != len(gt) or len(raw) < 3:
        raise _evidence_issue("AMBIGUOUS", "goal_samples_insufficient", "goal yaw arrays are incomplete")
    try:
        raw_values = [float(value) for value in raw]
        gt_values = [float(value) for value in gt]
    except (TypeError, ValueError):
        raise _evidence_issue("FAIL", "goal_samples_invalid", "goal yaw arrays are not numeric")
    if not all(math.isfinite(value) for value in (*raw_values, *gt_values)):
        raise _evidence_issue("FAIL", "goal_samples_nonfinite", "goal yaw arrays contain non-finite values")
    return {
        **goal,
        "source_mcap": str(source_mcap),
        "raw_integrated_yaw_rad": raw_values,
        "ground_truth_relative_yaw_rad": gt_values,
    }


def goal_identity_non_degrade_interval(goal: dict[str, Any] | None) -> list[list[float]] | None:
    """Scan only validated, MCAP-derived goal yaw series."""

    validated = validate_goal_evidence(goal)
    raw_values = validated["raw_integrated_yaw_rad"]
    gt_values = validated["ground_truth_relative_yaw_rad"]
    identity = _rmse([left - right for left, right in zip(raw_values, gt_values)])
    if identity is None:
        return None
    valid = []
    for k in _grid():
        error = _rmse([k * left - right for left, right in zip(raw_values, gt_values)])
        if error is not None and error <= identity + 1.0e-12:
            valid.append(k)
    return _contiguous_interval(valid)


def _require_report_field(document: dict[str, Any], key: str, scope: str) -> Any:
    if key not in document:
        raise _evidence_issue(
            "AMBIGUOUS", "benchmark_truncated", f"{scope} is missing {key}"
        )
    return document[key]


def _validate_receipt(
    receipt: Any, *, seed: int, pose: str, scope: str
) -> dict[str, Any]:
    if not isinstance(receipt, dict):
        raise _evidence_issue("AMBIGUOUS", "receipt_missing", f"{scope} receipt is missing")
    required = {"requested_seed", "actual_seed", "generation", "pose"}
    if not required.issubset(receipt):
        raise _evidence_issue("AMBIGUOUS", "receipt_truncated", f"{scope} receipt is incomplete")
    if (
        isinstance(receipt["requested_seed"], bool)
        or not isinstance(receipt["requested_seed"], int)
        or isinstance(receipt["actual_seed"], bool)
        or not isinstance(receipt["actual_seed"], int)
        or isinstance(seed, bool)
        or not isinstance(seed, int)
        or receipt["requested_seed"] != seed
        or receipt["actual_seed"] != seed
        or receipt["pose"] != pose
        or isinstance(receipt["generation"], bool)
        or not isinstance(receipt["generation"], int)
        or receipt["generation"] < 1
    ):
        raise _evidence_issue("FAIL", "receipt_mismatch", f"{scope} receipt does not match its request")
    return receipt


def validate_benchmark_report(
    report: Any,
    resources: DiagnosticResources | None = None,
) -> dict[str, Any]:
    """Enforce the complete stationary + nine-primitive report contract."""

    if not isinstance(report, dict):
        raise _evidence_issue("FAIL", "benchmark_wrong_type", "benchmark report must be an object")
    if (
        report.get("passed") is False
        or report.get("stopped") is True
        or report.get("collision_detected") is True
    ):
        raise _evidence_issue("FAIL", "benchmark_explicit_failure", "benchmark reports failure/STOP/collision")
    for key in (
        "passed", "stopped", "collision_detected", "sample_count",
        "segment_count", "final_zero_published", "spawn_pose_name",
        "stationary_reference", "primitives", "reset_receipts",
        "primitive_count", "passed_primitive_count",
        "thresholds", "command_rate_hz",
    ):
        _require_report_field(report, key, "benchmark report")
    primitives = report["primitives"]
    if not isinstance(primitives, list):
        raise _evidence_issue("FAIL", "benchmark_primitives_type", "primitives must be a list")
    if len(primitives) < len(EXPECTED_PRIMITIVE_IDS):
        raise _evidence_issue("AMBIGUOUS", "benchmark_truncated", "primitive sequence is truncated")
    if len(primitives) > len(EXPECTED_PRIMITIVE_IDS):
        raise _evidence_issue("FAIL", "benchmark_extra", "primitive sequence has extra entries")
    ids = [item.get("id") if isinstance(item, dict) else None for item in primitives]
    if len(set(ids)) != len(ids):
        raise _evidence_issue("FAIL", "benchmark_duplicate", "primitive IDs are duplicated")
    if tuple(ids) != EXPECTED_PRIMITIVE_IDS:
        raise _evidence_issue("FAIL", "benchmark_order", "primitive IDs/order do not match the diagnostic")
    pose = report["spawn_pose_name"]
    if pose != "flat20_start":
        raise _evidence_issue("FAIL", "benchmark_pose", "benchmark pose is not flat20_start")
    expected_thresholds = (
        EXPECTED_THRESHOLDS
        if resources is None
        else resources.identity["thresholds"]
    )
    thresholds = report["thresholds"]
    if not isinstance(thresholds, dict) or set(thresholds) != set(expected_thresholds):
        raise _evidence_issue("FAIL", "benchmark_thresholds", "benchmark thresholds do not match the resolved config")
    for key, expected in expected_thresholds.items():
        if isinstance(thresholds[key], bool) or not isinstance(thresholds[key], (int, float)):
            raise _evidence_issue("FAIL", "benchmark_thresholds", f"benchmark threshold {key} has invalid type")
        actual = float(thresholds[key])
        if not math.isfinite(actual) or actual != float(expected):
            raise _evidence_issue("FAIL", "benchmark_thresholds", f"benchmark threshold {key} changed")
    command_rate_hz = _finite(report["command_rate_hz"], name="benchmark.command_rate_hz", positive=True)
    expected_rate = 20.0 if resources is None else _finite(
        resources.config.get("command_rate_hz"), name="config.command_rate_hz", positive=True
    )
    if command_rate_hz != expected_rate:
        raise _evidence_issue("FAIL", "benchmark_rate", "benchmark command rate does not match the resolved config")

    stationary = report["stationary_reference"]
    if not isinstance(stationary, dict):
        raise _evidence_issue("AMBIGUOUS", "stationary_missing", "stationary reference is missing")
    if (
        stationary.get("passed") is False
        or stationary.get("stopped") is True
        or stationary.get("collision_detected") is True
    ):
        raise _evidence_issue("FAIL", "stationary_explicit_failure", "stationary reference reports failure/STOP/collision")
    for key in (
        "id", "passed", "stopped", "collision_detected", "sample_count",
        "segments", "reset_receipt", "reset_seed", "requested_duration_sec",
        "measured_duration_sec", "zero_command_count", "final_zero_published",
        "max_odometry_displacement_m",
    ):
        _require_report_field(stationary, key, "stationary reference")
    if (
        stationary["id"] != EXPECTED_STATIONARY["id"]
        or isinstance(stationary["reset_seed"], bool)
        or not isinstance(stationary["reset_seed"], int)
        or stationary["reset_seed"] != EXPECTED_STATIONARY["reset_seed"]
        or stationary["requested_duration_sec"] != EXPECTED_STATIONARY["duration_sec"]
    ):
        raise _evidence_issue("FAIL", "stationary_contract", "stationary identity/seed/duration mismatch")
    if (
        stationary["passed"] is not True
        or stationary["stopped"] is not False
        or stationary["collision_detected"] is not False
        or stationary["final_zero_published"] is not True
    ):
        raise _evidence_issue("FAIL", "stationary_explicit_failure", "stationary reference did not pass")
    if (
        isinstance(stationary["sample_count"], bool)
        or not isinstance(stationary["sample_count"], int)
        or stationary["sample_count"] <= 0
        or not isinstance(stationary["segments"], list)
        or stationary["segments"]
        or not isinstance(stationary["zero_command_count"], int)
        or stationary["zero_command_count"] <= 0
    ):
        raise _evidence_issue("AMBIGUOUS", "stationary_samples", "stationary sample/zero evidence is insufficient")
    try:
        stationary_duration = float(stationary["measured_duration_sec"])
        stationary_displacement = float(stationary["max_odometry_displacement_m"])
    except (TypeError, ValueError):
        raise _evidence_issue("AMBIGUOUS", "stationary_measurement", "stationary measurement is missing")
    if not math.isfinite(stationary_duration) or not math.isfinite(stationary_displacement):
        raise _evidence_issue("FAIL", "stationary_nonfinite", "stationary measurement is non-finite")
    if stationary_duration + 0.05 < 10.0:
        raise _evidence_issue("AMBIGUOUS", "stationary_short", "stationary duration is below 10 s")
    if stationary_displacement > 0.02:
        raise _evidence_issue("FAIL", "stationary_moved", "stationary odometry displacement exceeds 0.02 m")

    receipts = [
        _validate_receipt(
            stationary["reset_receipt"], seed=8609, pose=pose, scope="stationary"
        )
    ]
    for index, (primitive, expected_id, seed, segment_count, expected_commands, expected_durations) in enumerate(
        zip(
            primitives, EXPECTED_PRIMITIVE_IDS, EXPECTED_SEEDS,
            EXPECTED_SEGMENT_COUNTS, EXPECTED_COMMANDS, EXPECTED_DURATIONS,
        )
    ):
        assert isinstance(primitive, dict)
        scope = f"primitive {expected_id}"
        if (
            primitive.get("passed") is False
            or primitive.get("stopped") is True
            or primitive.get("collision_detected") is True
        ):
            raise _evidence_issue("FAIL", "benchmark_explicit_failure", f"{scope} reports failure/STOP/collision")
        for key in (
            "passed", "stopped", "collision_detected", "sample_count", "segments",
            "reset_receipt", "reset_seed", "final_zero_published",
        ):
            _require_report_field(primitive, key, scope)
        if (
            isinstance(primitive["reset_seed"], bool)
            or not isinstance(primitive["reset_seed"], int)
            or primitive["reset_seed"] != seed
        ):
            raise _evidence_issue("FAIL", "benchmark_seed", f"{scope} seed mismatch")
        if (
            primitive["passed"] is not True
            or primitive["stopped"] is not False
            or primitive["collision_detected"] is not False
            or primitive["final_zero_published"] is not True
        ):
            raise _evidence_issue("FAIL", "benchmark_explicit_failure", f"{scope} did not pass")
        if (
            isinstance(primitive["sample_count"], bool)
            or not isinstance(primitive["sample_count"], int)
            or primitive["sample_count"] <= 0
            or not isinstance(primitive["segments"], list)
            or len(primitive["segments"]) != segment_count
        ):
            raise _evidence_issue("AMBIGUOUS", "benchmark_samples", f"{scope} samples/segments are incomplete")
        for segment_index, (segment, expected_command, expected_duration) in enumerate(
            zip(primitive["segments"], expected_commands, expected_durations)
        ):
            if not isinstance(segment, dict):
                raise _evidence_issue("FAIL", "benchmark_segment_type", f"{scope} segment is not an object")
            required_segment = {
                "segment_index", "command_linear_mps", "command_angular_radps",
                "steady_sample_count", "duration_sec",
            }
            if not required_segment.issubset(segment):
                raise _evidence_issue("AMBIGUOUS", "benchmark_segment_truncated", f"{scope} segment is incomplete")
            if (
                segment["segment_index"] != segment_index
                or segment["command_linear_mps"] != expected_command[0]
                or segment["command_angular_radps"] != expected_command[1]
                or isinstance(segment["duration_sec"], bool)
                or not isinstance(segment["duration_sec"], (int, float))
                or not math.isfinite(float(segment["duration_sec"]))
                or float(segment["duration_sec"]) != expected_duration
            ):
                raise _evidence_issue("FAIL", "benchmark_segment_contract", f"{scope} segment command/order mismatch")
            if (
                isinstance(segment["steady_sample_count"], bool)
                or not isinstance(segment["steady_sample_count"], int)
                or segment["steady_sample_count"] <= 0
            ):
                raise _evidence_issue("AMBIGUOUS", "benchmark_segment_samples", f"{scope} segment has no steady samples")
        receipts.append(
            _validate_receipt(primitive["reset_receipt"], seed=seed, pose=pose, scope=scope)
        )

    generations = [receipt["generation"] for receipt in receipts]
    if generations != list(range(generations[0], generations[0] + len(generations))):
        raise _evidence_issue("FAIL", "generation_gap", "stationary and nine primitive reset generations are not consecutive")
    top_receipts = report["reset_receipts"]
    if not isinstance(top_receipts, list):
        raise _evidence_issue("FAIL", "top_receipts_type", "top reset receipts must be a list")
    if len(top_receipts) < len(receipts):
        raise _evidence_issue("AMBIGUOUS", "top_receipts_truncated", "top reset receipts are truncated")
    if len(top_receipts) > len(receipts) or top_receipts != receipts:
        raise _evidence_issue("FAIL", "top_receipts_mismatch", "top reset receipts do not match entries")
    expected_sample_count = sum(int(item["sample_count"]) for item in primitives) + int(stationary["sample_count"])
    if (
        report["passed"] is not True
        or report["stopped"] is not False
        or report["collision_detected"] is not False
        or report["final_zero_published"] is not True
        or report["sample_count"] != expected_sample_count
        or report["segment_count"] != sum(EXPECTED_SEGMENT_COUNTS)
        or report["primitive_count"] != len(EXPECTED_PRIMITIVE_IDS)
        or report["passed_primitive_count"] != len(EXPECTED_PRIMITIVE_IDS)
    ):
        raise _evidence_issue("FAIL", "benchmark_top_mismatch", "top benchmark status/counts are inconsistent")
    return report


def summarize(
    segments: Sequence[dict[str, object]],
    *,
    goal: dict[str, Any] | None = None,
    benchmark_valid: bool = False,
    stationary_valid: bool = False,
    phase_valid: bool = False,
) -> dict[str, object]:
    usable = [segment for segment in segments if segment.get("status") == "OK"]
    invalid = [segment for segment in segments if segment.get("status") != "OK"]
    segment_intervals = [segment["scale_interval_le_5deg"] for segment in usable]
    goal_issue = None
    try:
        goal_interval = goal_identity_non_degrade_interval(goal)
    except EvidenceError as exc:
        goal_interval = None
        goal_issue = {
            "verdict": exc.verdict,
            "code": exc.code,
            "detail": exc.detail,
        }
    all_sets = list(segment_intervals)
    if goal_interval is not None:
        all_sets.append(goal_interval)
    intersection = _interval_intersection(all_sets) if all_sets else []
    complete_window_contract = (
        len(segments) == 12
        and sum(item.get("id") == "stationary_reference" for item in segments) == 1
        and all(
            sum(
                str(item.get("id", "")).split("[", 1)[0] == expected
                for item in segments
            ) == (3 if expected == "s_route" else 1)
            for expected in EXPECTED_PRIMITIVE_IDS
        )
    )
    explicit_invalid = any(
        item.get("status") in {
            "DATA_INVALID", "STAMP_INVALID", "PHASE_TRACE_INVALID",
            "DURATION_INVALID",
        }
        for item in invalid
    )
    any_empty_interval = (
        any(not intervals for intervals in segment_intervals)
        or goal_interval == []
    )
    evidence_complete = (
        benchmark_valid
        and stationary_valid
        and phase_valid
        and complete_window_contract
        and goal_issue is None
    )
    if explicit_invalid or (goal_issue and goal_issue["verdict"] == "FAIL"):
        verdict = "FAIL"
    elif invalid or not evidence_complete:
        verdict = "AMBIGUOUS"
    elif not usable:
        verdict = "AMBIGUOUS"
    elif any_empty_interval:
        verdict = "AMBIGUOUS"
    elif (
        not intersection
        and len(segment_intervals) >= 2
        and all(segment_intervals)
    ):
        verdict = "CONFIRMED_NO_GLOBAL_CONSTANT"
    elif not intersection or goal_interval is None:
        verdict = "AMBIGUOUS"
    else:
        verdict = "PASS_CANDIDATE"
    bins: dict[str, list[float]] = {}
    for segment in usable:
        key = (
            f"v={float(segment['speed_bin_mps']):.2f}/"
            f"{segment['direction']}"
        )
        if segment.get("k_star") is not None:
            bins.setdefault(key, []).append(float(segment["k_star"]))
    return {
        "verdict": verdict,
        "segments": list(segments),
        "bins": {
            key: {"count": len(values), "median_k_star": statistics.median(values)}
            for key, values in sorted(bins.items())
        },
        "goal_identity_non_degrade_interval": goal_interval,
        "goal_evidence_issue": goal_issue,
        "global_scale_intersection": intersection,
        "contract": {
            "benchmark_valid": benchmark_valid,
            "stationary_valid": stationary_valid,
            "phase_valid": phase_valid,
            "window_contract_valid": complete_window_contract,
        },
    }


def _load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _load_phase(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        try:
            row = json.loads(line)
        except json.JSONDecodeError as exc:
            raise _evidence_issue("AMBIGUOUS", "phase_truncated", f"invalid phase JSONL line {line_number}: {exc}") from exc
        if not isinstance(row, dict):
            raise _evidence_issue("FAIL", "phase_row_type", f"phase row {line_number} is not an object")
        rows.append(row)
    return rows


def _set_file_read_order(reader: Any, rosbag2_py: Any) -> None:
    """Require storage/file iteration order before consuming evidence."""

    order_type = getattr(rosbag2_py, "ReadOrder", None)
    sort_type = getattr(rosbag2_py, "ReadOrderSortBy", None)
    file_order = getattr(sort_type, "File", None) if sort_type is not None else None
    setter = getattr(reader, "set_read_order", None)
    if order_type is None or file_order is None or not callable(setter):
        raise _evidence_issue(
            "AMBIGUOUS", "mcap_file_order_unavailable",
            "rosbag2_py cannot explicitly select MCAP file read order",
        )
    try:
        try:
            order = order_type(sort_by=file_order, reverse=False)
        except TypeError:
            order = order_type(file_order, False)
        accepted = setter(order)
    except Exception as exc:
        raise _evidence_issue(
            "AMBIGUOUS", "mcap_file_order_unavailable",
            f"rosbag2_py rejected MCAP file read order: {exc}",
        ) from exc
    if accepted is not True:
        raise _evidence_issue(
            "AMBIGUOUS", "mcap_file_order_unavailable",
            "MCAP storage did not confirm file read order",
        )


def validate_phase_trace(
    phase: Sequence[dict[str, Any]],
    resources: DiagnosticResources,
) -> dict[str, Any]:
    if not phase:
        raise _evidence_issue("AMBIGUOUS", "phase_empty", "phase trace is empty")
    manifest = phase[0]
    if manifest.get("kind") != "manifest" or manifest.get("schema") != "bio_nav_v6_imu_regime_phase_trace_v1":
        raise _evidence_issue("FAIL", "phase_manifest", "phase manifest schema is invalid")
    if manifest.get("passive") is not True:
        raise _evidence_issue("FAIL", "phase_manifest", "phase trace is not passive")
    provenance = manifest.get("provenance")
    if not isinstance(provenance, dict):
        raise _evidence_issue("AMBIGUOUS", "phase_provenance_missing", "phase provenance is missing")
    expected = {
        **EXPECTED_TRACE_PROVENANCE,
        "spawn_poses_file": str(resources.spawn_poses_path),
        "diagnostic_config_file": str(resources.config_path),
    }
    mismatches = {
        key: {"expected": value, "actual": provenance.get(key)}
        for key, value in expected.items()
        if provenance.get(key) != value
    }
    if mismatches:
        raise _evidence_issue("FAIL", "phase_provenance_mismatch", f"phase provenance mismatch: {mismatches}")
    loops = [row for row in phase[1:] if row.get("kind") == "loop"]
    if len(loops) != len(phase) - 1:
        raise _evidence_issue("FAIL", "phase_extra_rows", "phase trace contains unexpected row kinds")
    if any(row.get("incomplete") is True for row in loops):
        raise _evidence_issue("AMBIGUOUS", "phase_incomplete", "phase trace contains an incomplete loop")
    sequences = [row.get("loop_sequence") for row in loops]
    if any(isinstance(value, bool) or not isinstance(value, int) for value in sequences):
        raise _evidence_issue("FAIL", "phase_sequence_type", "phase loop sequence is invalid")
    if any(right <= left for left, right in zip(sequences, sequences[1:])):
        raise _evidence_issue("FAIL", "phase_sequence_order", "phase loop sequence is not strictly increasing")
    return provenance


def load_mcap(path: Path) -> McapStreams:
    """Read the three required topics through the installed rosbag2 MCAP API."""

    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise _evidence_issue("AMBIGUOUS", "mcap_backend_missing", "rosbag2_py MCAP support is unavailable") from exc
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    _set_file_read_order(reader, rosbag2_py)
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    required = tuple(EXPECTED_TOPIC_TYPES)
    missing = [topic for topic in required if topic not in topic_types]
    if missing:
        raise _evidence_issue("AMBIGUOUS", "mcap_topics_missing", f"MCAP missing required topics: {missing}")
    wrong_types = {
        topic: {"expected": EXPECTED_TOPIC_TYPES[topic], "actual": topic_types[topic]}
        for topic in required
        if topic_types[topic] != EXPECTED_TOPIC_TYPES[topic]
    }
    if wrong_types:
        raise _evidence_issue("FAIL", "mcap_topic_type", f"MCAP topic types mismatch: {wrong_types}")
    message_types = {topic: get_message(topic_types[topic]) for topic in required}
    streams: dict[str, list[ScalarSample | YawSample]] = {topic: [] for topic in required}
    while reader.has_next():
        topic, payload, _recorded_ns = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(payload, message_types[topic])
        stamp = message.header.stamp
        stamp_s = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
        if not math.isfinite(stamp_s) or stamp_s <= 0.0:
            raise _evidence_issue("FAIL", "mcap_stamp_nonfinite", f"{topic} has an invalid header stamp")
        if topic == "/ground_truth/odom":
            q = message.pose.pose.orientation
            quaternion = [float(q.x), float(q.y), float(q.z), float(q.w)]
            if not all(math.isfinite(value) for value in quaternion):
                raise _evidence_issue("FAIL", "mcap_quaternion_nonfinite", "ground-truth quaternion is non-finite")
            norm = math.sqrt(sum(value * value for value in quaternion))
            if abs(norm - 1.0) > 1.0e-3:
                raise _evidence_issue("FAIL", "mcap_quaternion_norm", f"ground-truth quaternion norm is {norm}")
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            if not math.isfinite(yaw):
                raise _evidence_issue("FAIL", "mcap_yaw_nonfinite", "ground-truth yaw is non-finite")
            streams[topic].append(YawSample(stamp_s, yaw))
        else:
            angular = message.angular_velocity
            values = [float(angular.x), float(angular.y), float(angular.z)]
            if not all(math.isfinite(value) for value in values):
                raise _evidence_issue("FAIL", "mcap_angular_nonfinite", f"{topic} angular velocity is non-finite")
            streams[topic].append(
                ScalarSample(stamp_s, values[2])
            )
    for topic, samples in streams.items():
        if len(samples) < 3:
            raise _evidence_issue("AMBIGUOUS", "mcap_samples_insufficient", f"{topic} has fewer than 3 samples")
        quality = _stamp_quality(samples)
        if quality["duplicate_count"] or quality["backward_count"]:
            raise _evidence_issue("FAIL", "mcap_stamp_order", f"{topic} stamps are not strictly increasing")
    provenance = {
        "path": str(path.resolve()),
        "storage_id": "mcap",
        "read_order": "file",
        "yaw_time_basis": "header_stamp_in_file_publish_order",
        "topic_types": {topic: topic_types[topic] for topic in required},
        "topic_counts": {topic: len(streams[topic]) for topic in required},
    }
    return McapStreams(streams, provenance=provenance)


def _record_stamp_s(message: Any, recorded_ns: Any, *, topic: str) -> float:
    stamp = None
    header_topics = {"/imu/data_raw", "/imu/data", "/ground_truth/odom"}
    if topic in header_topics:
        stamp = getattr(getattr(message, "header", None), "stamp", None)
        if stamp is None:
            raise _evidence_issue("FAIL", "goal_stamp_invalid", f"{topic} has no header stamp")
    if stamp is not None:
        sec = getattr(stamp, "sec", None)
        nanosec = getattr(stamp, "nanosec", None)
        if (
            isinstance(sec, bool)
            or not isinstance(sec, int)
            or isinstance(nanosec, bool)
            or not isinstance(nanosec, int)
            or nanosec < 0
            or nanosec >= 1_000_000_000
        ):
            raise _evidence_issue("FAIL", "goal_stamp_invalid", f"{topic} has an invalid message stamp")
        value = float(sec) + float(nanosec) * 1.0e-9
    else:
        if isinstance(recorded_ns, bool) or not isinstance(recorded_ns, int):
            raise _evidence_issue("FAIL", "goal_stamp_invalid", f"{topic} has an invalid bag stamp")
        value = float(recorded_ns) * 1.0e-9
    if not math.isfinite(value) or value < 0.0:
        raise _evidence_issue("FAIL", "goal_stamp_invalid", f"{topic} has a non-finite/negative stamp")
    if topic in header_topics and value == 0.0:
        raise _evidence_issue("FAIL", "goal_stamp_invalid", f"{topic} has a zero header stamp")
    return value


def _reset_receipt_from_log(message: Any) -> dict[str, Any] | None:
    text = getattr(message, "msg", None)
    if not isinstance(text, str) or "reset_receipt=" not in text:
        return None
    tail = text.split("reset_receipt=", 1)[1]
    try:
        value, _end = json.JSONDecoder().raw_decode(tail)
    except json.JSONDecodeError as exc:
        raise _evidence_issue("FAIL", "goal_reset_log_invalid", "goal MCAP contains a malformed reset receipt log") from exc
    if not isinstance(value, dict):
        raise _evidence_issue("FAIL", "goal_reset_log_invalid", "goal MCAP reset receipt is not an object")
    return value


def _validate_goal_metadata(metadata: Any, goal_mcap: Path) -> dict[str, Any]:
    if not isinstance(metadata, dict):
        raise _evidence_issue("FAIL", "goal_wrong_type", "goal metadata must be an object")
    required = {
        "schema_version", "source", "source_mcap", "reset_receipt",
        "outcome", "collision_detected",
    }
    missing = required - set(metadata)
    if missing:
        raise _evidence_issue("AMBIGUOUS", "goal_truncated", f"goal metadata missing fields: {sorted(missing)}")
    if metadata["schema_version"] != 1 or metadata["source"] != "goal_mcap_outcome_metadata":
        raise _evidence_issue("FAIL", "goal_provenance_invalid", "goal metadata schema/source is invalid")
    if not isinstance(metadata["source_mcap"], str) or not metadata["source_mcap"]:
        raise _evidence_issue("AMBIGUOUS", "goal_source_missing", "goal metadata source_mcap is missing")
    try:
        source = Path(metadata["source_mcap"]).expanduser().resolve()
    except (OSError, ValueError) as exc:
        raise _evidence_issue("FAIL", "goal_source_invalid", "goal metadata source_mcap is invalid") from exc
    if source != goal_mcap.expanduser().resolve():
        raise _evidence_issue("FAIL", "goal_source_mismatch", "goal metadata source_mcap does not match --goal-mcap")
    receipt = metadata["reset_receipt"]
    if not isinstance(receipt, dict):
        raise _evidence_issue("FAIL", "goal_reset_invalid", "goal metadata reset receipt is invalid")
    for key in ("requested_seed", "actual_seed"):
        _strict_int(receipt.get(key), name=f"goal.reset_receipt.{key}")
    generation = _strict_int(receipt.get("generation"), name="goal.reset_receipt.generation", minimum=1)
    if (
        receipt["requested_seed"] != receipt["actual_seed"]
        or not isinstance(receipt.get("pose"), str)
        or not receipt["pose"]
        or generation < 1
    ):
        raise _evidence_issue("FAIL", "goal_reset_invalid", "goal metadata reset receipt is inconsistent")
    if metadata["outcome"] != "SUCCEEDED" or not isinstance(metadata["collision_detected"], bool):
        raise _evidence_issue("FAIL", "goal_outcome_invalid", "goal metadata outcome/collision is invalid")
    if metadata["collision_detected"]:
        raise _evidence_issue("FAIL", "goal_collision", "goal metadata reports a collision")
    return metadata


def _goal_request_identity(message: Any, recorded_s: float) -> dict[str, Any]:
    """Return the bounded PoseStamped identity available in a goal MCAP."""

    header = getattr(message, "header", None)
    frame_id = getattr(header, "frame_id", None)
    stamp = getattr(header, "stamp", None)
    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    pose = getattr(message, "pose", None)
    position = getattr(pose, "position", None)
    orientation = getattr(pose, "orientation", None)
    if (
        not isinstance(frame_id, str) or not frame_id
        or isinstance(sec, bool) or not isinstance(sec, int)
        or isinstance(nanosec, bool) or not isinstance(nanosec, int)
        or nanosec < 0 or nanosec >= 1_000_000_000
    ):
        raise _evidence_issue("FAIL", "goal_request_invalid", "route goal header identity is invalid")
    try:
        position_values = [float(getattr(position, key)) for key in ("x", "y", "z")]
        orientation_values = [float(getattr(orientation, key)) for key in ("x", "y", "z", "w")]
    except (AttributeError, TypeError, ValueError) as exc:
        raise _evidence_issue("FAIL", "goal_request_invalid", "route goal pose identity is invalid") from exc
    if not all(math.isfinite(value) for value in (*position_values, *orientation_values)):
        raise _evidence_issue("FAIL", "goal_request_invalid", "route goal pose identity is non-finite")
    norm = math.sqrt(sum(value * value for value in orientation_values))
    if abs(norm - 1.0) > 1.0e-3:
        raise _evidence_issue("FAIL", "goal_request_invalid", f"route goal quaternion norm is {norm}")
    return {
        "recorded_s": recorded_s,
        "header_stamp_s": float(sec) + float(nanosec) * 1.0e-9,
        "frame_id": frame_id,
        "position_m": position_values,
        "orientation_xyzw": orientation_values,
    }


def _check_goal_request_metadata(observed: dict[str, Any], metadata: dict[str, Any]) -> None:
    expected = metadata.get("route_goal_request")
    if not isinstance(expected, dict):
        raise _evidence_issue(
            "AMBIGUOUS", "goal_request_metadata_missing",
            "goal MCAP contains a route request but metadata has no route_goal_request identity",
        )
    if set(expected) != set(observed):
        raise _evidence_issue("FAIL", "goal_request_mismatch", "route goal metadata identity fields mismatch")
    for key in ("recorded_s", "header_stamp_s"):
        if _finite(expected.get(key), name=f"goal.route_goal_request.{key}") != observed[key]:
            raise _evidence_issue("FAIL", "goal_request_mismatch", f"route goal metadata {key} mismatch")
    if expected.get("frame_id") != observed["frame_id"]:
        raise _evidence_issue("FAIL", "goal_request_mismatch", "route goal metadata frame mismatch")
    for key in ("position_m", "orientation_xyzw"):
        values = expected.get(key)
        if not isinstance(values, list) or len(values) != len(observed[key]):
            raise _evidence_issue("FAIL", "goal_request_mismatch", f"route goal metadata {key} is invalid")
        try:
            numeric = [float(value) for value in values]
        except (TypeError, ValueError) as exc:
            raise _evidence_issue("FAIL", "goal_request_mismatch", f"route goal metadata {key} is invalid") from exc
        if numeric != observed[key]:
            raise _evidence_issue("FAIL", "goal_request_mismatch", f"route goal metadata {key} mismatch")


def load_goal_mcap(path: Path, metadata: Any) -> dict[str, Any]:
    """Derive goal yaw arrays and all authority facts directly from one MCAP."""

    path = path.expanduser().resolve()
    metadata = _validate_goal_metadata(metadata, path)
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise _evidence_issue("AMBIGUOUS", "mcap_backend_missing", "rosbag2_py MCAP support is unavailable") from exc
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    _set_file_read_order(reader, rosbag2_py)
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    missing = [topic for topic in GOAL_TOPIC_TYPES if topic not in topic_types]
    if missing:
        raise _evidence_issue("AMBIGUOUS", "goal_mcap_topics_missing", f"goal MCAP missing required topics: {missing}")
    wrong = {
        topic: {"expected": expected, "actual": topic_types[topic]}
        for topic, expected in GOAL_TOPIC_TYPES.items()
        if topic_types[topic] != expected
    }
    wrong.update({
        topic: {"expected": expected, "actual": topic_types[topic]}
        for topic, expected in GOAL_OPTIONAL_TOPIC_TYPES.items()
        if topic in topic_types and topic_types[topic] != expected
    })
    if wrong:
        raise _evidence_issue("FAIL", "goal_mcap_topic_type", f"goal MCAP topic types mismatch: {wrong}")
    selected_topic_types = {
        **GOAL_TOPIC_TYPES,
        **{
            topic: expected for topic, expected in GOAL_OPTIONAL_TOPIC_TYPES.items()
            if topic in topic_types
        },
    }
    message_types = {topic: get_message(topic_types[topic]) for topic in selected_topic_types}
    raw: list[ScalarSample] = []
    corrected: list[ScalarSample] = []
    gt: list[YawSample] = []
    commands: list[tuple[float, float, float]] = []
    reset_events: list[float] = []
    collisions: list[tuple[float, bool]] = []
    completions: list[tuple[float, bool]] = []
    route_requests: list[tuple[float, dict[str, Any]]] = []
    receipt_logs: list[tuple[float, dict[str, Any]]] = []
    topic_stamps: dict[str, list[float]] = {topic: [] for topic in selected_topic_types}
    epoch_started = False
    while reader.has_next():
        topic, payload, recorded_ns = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(payload, message_types[topic])
        stamp_s = _record_stamp_s(message, recorded_ns, topic=topic)
        if topic == "/simulation/reset_event":
            topic_stamps[topic].append(stamp_s)
            reset_events.append(stamp_s)
            epoch_started = True
            continue
        # A recorder normally starts before the requested reset. Pre-reset
        # sensor and command samples belong to the retired epoch and may have
        # larger header stamps than the fresh epoch; never mix them.
        if not epoch_started:
            continue
        topic_stamps[topic].append(stamp_s)
        if topic in {"/imu/data_raw", "/imu/data"}:
            angular = getattr(message, "angular_velocity", None)
            values = [getattr(angular, key, None) for key in ("x", "y", "z")]
            try:
                numbers = [float(value) for value in values]
            except (TypeError, ValueError) as exc:
                raise _evidence_issue("FAIL", "goal_imu_invalid", "goal raw IMU angular velocity is invalid") from exc
            if not all(math.isfinite(value) for value in numbers):
                raise _evidence_issue("FAIL", "goal_imu_nonfinite", "goal raw IMU angular velocity is non-finite")
            target = raw if topic == "/imu/data_raw" else corrected
            target.append(ScalarSample(stamp_s, numbers[2]))
        elif topic == "/ground_truth/odom":
            orientation = getattr(getattr(getattr(message, "pose", None), "pose", None), "orientation", None)
            try:
                quaternion = [float(getattr(orientation, key)) for key in ("x", "y", "z", "w")]
            except (AttributeError, TypeError, ValueError) as exc:
                raise _evidence_issue("FAIL", "goal_gt_invalid", "goal ground-truth quaternion is invalid") from exc
            if not all(math.isfinite(value) for value in quaternion):
                raise _evidence_issue("FAIL", "goal_gt_nonfinite", "goal ground-truth quaternion is non-finite")
            norm = math.sqrt(sum(value * value for value in quaternion))
            if abs(norm - 1.0) > 1.0e-3:
                raise _evidence_issue("FAIL", "goal_gt_quaternion_norm", f"goal ground-truth quaternion norm is {norm}")
            x, y, z, w = quaternion
            yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            gt.append(YawSample(stamp_s, yaw))
        elif topic == "/cmd_vel":
            linear = getattr(getattr(message, "linear", None), "x", None)
            angular = getattr(getattr(message, "angular", None), "z", None)
            try:
                values = (float(linear), float(angular))
            except (TypeError, ValueError) as exc:
                raise _evidence_issue("FAIL", "goal_command_invalid", "goal command is invalid") from exc
            if not all(math.isfinite(value) for value in values):
                raise _evidence_issue("FAIL", "goal_command_nonfinite", "goal command is non-finite")
            commands.append((stamp_s, values[0], values[1]))
        elif topic == "/simulation/collision":
            value = getattr(message, "data", None)
            if not isinstance(value, bool):
                raise _evidence_issue("FAIL", "goal_collision_type", "goal collision sample is not boolean")
            collisions.append((stamp_s, value))
        elif topic == "/bio_nav/route_goal_complete":
            value = getattr(message, "data", None)
            if not isinstance(value, bool):
                raise _evidence_issue("FAIL", "goal_outcome_type", "goal completion sample is not boolean")
            completions.append((stamp_s, value))
        elif topic == "/bio_nav/route_goal":
            route_requests.append((stamp_s, _goal_request_identity(message, stamp_s)))
        elif topic == "/rosout":
            receipt = _reset_receipt_from_log(message)
            if receipt is not None:
                receipt_logs.append((stamp_s, receipt))
    # The yaw streams retain file/publish order so a duplicate or backward
    # header cannot be hidden by sorting. Event topics instead use rosbag
    # received timestamps and are explicitly ordered only after collection;
    # received-time jitter must not reorder the yaw header evidence.
    for topic in ("/imu/data_raw", "/imu/data", "/ground_truth/odom"):
        stamps = topic_stamps.get(topic, [])
        if any(right <= left for left, right in zip(stamps, stamps[1:])):
            raise _evidence_issue("FAIL", "goal_stamp_order", f"{topic} stamps are not strictly increasing")
    reset_events.sort()
    commands.sort(key=lambda item: item[0])
    collisions.sort(key=lambda item: item[0])
    completions.sort(key=lambda item: item[0])
    route_requests.sort(key=lambda item: item[0])
    receipt_logs.sort(key=lambda item: item[0])
    if len(reset_events) != 1 or len(receipt_logs) != 1:
        verdict = "AMBIGUOUS" if not reset_events or not receipt_logs else "FAIL"
        raise _evidence_issue(verdict, "goal_reset_count", "goal MCAP must contain exactly one reset event and one reset receipt")
    bag_receipt = receipt_logs[0][1]
    for key in ("seed", "generation"):
        _strict_int(bag_receipt.get(key), name=f"goal bag reset {key}", minimum=1 if key == "generation" else 0)
    metadata_receipt = metadata["reset_receipt"]
    if (
        bag_receipt.get("seed") != metadata_receipt["actual_seed"]
        or bag_receipt.get("generation") != metadata_receipt["generation"]
        or bag_receipt.get("pose") != metadata_receipt["pose"]
    ):
        raise _evidence_issue("FAIL", "goal_reset_mismatch", "goal MCAP reset receipt does not match metadata")
    reset_s = reset_events[0]
    if len(completions) != 1:
        verdict = "AMBIGUOUS" if not completions else "FAIL"
        raise _evidence_issue(
            verdict, "goal_outcome_count",
            "goal MCAP must contain exactly one fresh route terminal; "
            f"count={len(completions)}, timestamps_s={[stamp for stamp, _value in completions]}, "
            f"values={[value for _stamp, value in completions]}",
        )
    completed_s, completed = completions[0]
    if not completed:
        raise _evidence_issue(
            "FAIL", "goal_outcome_false",
            f"the only fresh route terminal is false at {completed_s} s",
        )
    if not reset_s < completed_s:
        raise _evidence_issue("FAIL", "goal_outcome_order", "route terminal does not follow the reset")
    request_identity = None
    request_s = reset_s
    binding_source = "reset_terminal_single_command_attempt"
    if route_requests:
        if len(route_requests) != 1:
            raise _evidence_issue(
                "FAIL", "goal_request_count",
                "goal MCAP must contain one fresh route request when that topic is recorded; "
                f"count={len(route_requests)}, timestamps_s={[stamp for stamp, _identity in route_requests]}",
            )
        request_s, request_identity = route_requests[0]
        if not reset_s < request_s < completed_s:
            raise _evidence_issue("FAIL", "goal_request_order", "route goal request is outside the fresh terminal window")
        _check_goal_request_metadata(request_identity, metadata)
        binding_source = "route_goal_pose_stamped"
    collision_window = [value for stamp, value in collisions if reset_s <= stamp <= completed_s]
    if not collision_window:
        raise _evidence_issue("AMBIGUOUS", "goal_collision_missing", "goal MCAP has no collision samples in the goal epoch")
    if any(collision_window):
        raise _evidence_issue("FAIL", "goal_collision", "goal MCAP reports a collision")
    # A recorded request gives this epoch an authoritative lower motion
    # boundary.  Include the reset timestamp so a stale command emitted in the
    # same executor instant as reset cannot be attributed to the new request.
    # Without a recorded request, reset itself is the conservative attempt
    # boundary and a legitimate first command at that timestamp must remain in
    # the single-command-attempt window below.
    abnormal_before = (
        [
            item for item in commands
            if reset_s <= item[0] < request_s
            and (abs(item[1]) > 1.0e-12 or abs(item[2]) > 1.0e-12)
        ]
        if route_requests
        else []
    )
    abnormal_after = [
        item for item in commands
        if item[0] >= completed_s
        and (abs(item[1]) > 1.0e-12 or abs(item[2]) > 1.0e-12)
    ]
    if abnormal_before:
        raise _evidence_issue("FAIL", "goal_command_before_request", "nonzero command precedes the bound route request")
    if abnormal_after:
        raise _evidence_issue("FAIL", "goal_command_after_terminal", "nonzero command follows the route terminal")
    nonzero = [
        item for item in commands
        if request_s <= item[0] < completed_s
        and (abs(item[1]) > 1.0e-12 or abs(item[2]) > 1.0e-12)
    ]
    if len(nonzero) < 3:
        raise _evidence_issue("AMBIGUOUS", "goal_command_missing", "goal MCAP has insufficient nonzero command evidence")
    start_s = nonzero[0][0]
    end_s = nonzero[-1][0]
    if end_s - start_s < MIN_WINDOW_DURATION_S:
        raise _evidence_issue("AMBIGUOUS", "goal_window_short", "goal MCAP command window is too short")
    command_gaps = [right[0] - left[0] for left, right in zip(nonzero, nonzero[1:])]
    if command_gaps and max(command_gaps) > 1.0:
        raise _evidence_issue("AMBIGUOUS", "goal_command_gap", "goal MCAP nonzero command coverage has a gap above 1 s")
    raw_window = _window_scalar(raw, start_s, end_s)
    corrected_window = _window_scalar(corrected, start_s, end_s) if "/imu/data" in selected_topic_types else []
    gt_window = _window_yaw(gt, start_s, end_s)
    if len(raw_window) < 3 or len(gt_window) < 3 or ("/imu/data" in selected_topic_types and len(corrected_window) < 3):
        raise _evidence_issue("AMBIGUOUS", "goal_samples_insufficient", "goal MCAP yaw streams do not cover the command window")
    quality = {"raw": _stamp_quality(raw_window), "ground_truth": _stamp_quality(gt_window)}
    if "/imu/data" in selected_topic_types:
        quality["corrected"] = _stamp_quality(corrected_window)
    if any(
        item["duplicate_count"] or item["backward_count"]
        or item["nonfinite_count"] or item["nonfinite_value_count"]
        for item in quality.values()
    ):
        raise _evidence_issue("FAIL", "goal_samples_invalid", "goal MCAP yaw stamps/values are invalid")
    coverage_streams: dict[str, Sequence[ScalarSample | YawSample]] = {
        "raw": raw_window, "ground_truth": gt_window,
    }
    if "/imu/data" in selected_topic_types:
        coverage_streams["corrected"] = corrected_window
    maximum_gaps = {name: _maximum_gap(samples) for name, samples in coverage_streams.items()}
    t0 = max(samples[0].stamp_s for samples in coverage_streams.values())
    t1 = min(samples[-1].stamp_s for samples in coverage_streams.values())
    coverage_duration = t1 - t0
    window_duration = end_s - start_s
    if (
        coverage_duration < MIN_WINDOW_DURATION_S
        or t0 - start_s > MAX_SAMPLE_GAP_S
        or end_s - t1 > MAX_SAMPLE_GAP_S
        or any(gap is None or gap > MAX_SAMPLE_GAP_S for gap in maximum_gaps.values())
    ):
        raise _evidence_issue(
            "AMBIGUOUS", "goal_sample_gap",
            "goal yaw stream coverage is incomplete or contains a gap above 0.25 s",
        )
    grid = sorted({
        t0, t1,
        *[item.stamp_s for item in raw_window if t0 <= item.stamp_s <= t1],
        *[item.stamp_s for item in gt_window if t0 <= item.stamp_s <= t1],
    })
    raw_values = [_interpolate([item.stamp_s for item in raw_window], [item.value for item in raw_window], stamp) for stamp in grid]
    gt_unwrapped = _unwrap([item.yaw_rad for item in gt_window])
    gt_values = [_interpolate([item.stamp_s for item in gt_window], gt_unwrapped, stamp) for stamp in grid]
    corrected_values = (
        [_interpolate([item.stamp_s for item in corrected_window], [item.value for item in corrected_window], stamp) for stamp in grid]
        if corrected_window else []
    )
    if len(grid) < 3 or any(value is None for value in (*raw_values, *gt_values, *corrected_values)):
        raise _evidence_issue("AMBIGUOUS", "goal_common_grid", "goal MCAP has no complete common yaw grid")
    _, integrated = _integral([
        ScalarSample(stamp, float(value)) for stamp, value in zip(grid, raw_values)
    ])
    relative_gt = [float(value) - float(gt_values[0]) for value in gt_values]
    corrected_integrated = None
    if corrected_values:
        _, corrected_integrated = _integral([
            ScalarSample(stamp, float(value)) for stamp, value in zip(grid, corrected_values)
        ])
    derived = {
        "schema_version": 1,
        "source": "goal_mcap_derived",
        "source_mcap": str(path),
        "reset_receipt": dict(metadata_receipt),
        "outcome": "SUCCEEDED",
        "collision_detected": False,
        "bag_verified": True,
        "goal_window": {
            "start_s": start_s,
            "end_s": end_s,
            "duration_s": end_s - start_s,
            "route_completed_s": completed_s,
            "common_t0_s": t0,
            "common_t1_s": t1,
            "common_grid_count": len(grid),
            "nonzero_command_count": len(nonzero),
            "maximum_nonzero_command_gap_s": max(command_gaps) if command_gaps else None,
            "binding_source": binding_source,
        },
        "attempt_provenance": {
            "reset_event_s": reset_s,
            "route_request_count": len(route_requests),
            "route_request_timestamps_s": [stamp for stamp, _identity in route_requests],
            "route_goal_request": request_identity,
            "binding_source": binding_source,
            "terminal_count": len(completions),
            "terminal_timestamps_s": [stamp for stamp, _value in completions],
            "terminal_values": [value for _stamp, value in completions],
            "selected_terminal_s": completed_s,
            "command_window_source": "first_to_last_nonzero_command",
        },
        "stream_coverage": {
            "command_start_s": start_s,
            "command_end_s": end_s,
            "common_t0_s": t0,
            "common_t1_s": t1,
            "common_duration_s": coverage_duration,
            "command_duration_s": window_duration,
            "common_coverage_fraction": coverage_duration / window_duration,
            "maximum_gap_s": maximum_gaps,
            "maximum_allowed_gap_s": MAX_SAMPLE_GAP_S,
            "interpolation": "linear_no_extrapolation",
        },
        "raw_integrated_yaw_rad": integrated,
        "corrected_integrated_yaw_rad": corrected_integrated,
        "ground_truth_relative_yaw_rad": relative_gt,
        "mcap_provenance": {
            "path": str(path),
            "storage_id": "mcap",
            "read_order": "file",
            "yaw_time_basis": "header_stamp_in_file_publish_order",
            "event_time_basis": "received_timestamp_sorted_after_collection",
            "topic_types": {topic: topic_types[topic] for topic in selected_topic_types},
            "topic_counts": {topic: len(topic_stamps[topic]) for topic in selected_topic_types},
            "reset_event_count": len(reset_events),
            "reset_receipt_log_count": len(receipt_logs),
            "route_request_count": len(route_requests),
            "terminal_count": len(completions),
            "terminal_timestamps_s": [stamp for stamp, _value in completions],
        },
    }
    return validate_goal_evidence(derived, expected_mcap=path)


def command_windows(
    phase: Sequence[dict[str, Any]], report: dict[str, Any]
) -> list[dict[str, object]]:
    """Recover exactly one stationary, eight single, and three S windows."""

    loops = [row for row in phase if row.get("kind") == "loop"]
    if not loops:
        raise _evidence_issue("AMBIGUOUS", "phase_empty", "phase trace has no loop rows")
    stationary = report["stationary_reference"]
    primitives = report["primitives"]
    generation_contract = {
        int(stationary["reset_receipt"]["generation"]): (
            str(stationary["id"]), 1, True
        )
    }
    for primitive, count in zip(primitives, EXPECTED_SEGMENT_COUNTS):
        generation_contract[int(primitive["reset_receipt"]["generation"])] = (
            str(primitive["id"]), count, False
        )

    observed_generations = set()
    for row in loops:
        for key in ("reset_generation", "reset_generation_after_ground_truth"):
            value = row.get(key)
            if value is None and key == "reset_generation_after_ground_truth":
                continue
            if isinstance(value, bool) or not isinstance(value, int):
                raise _evidence_issue("FAIL", "phase_generation_type", "phase reset generation is not an integer")
            observed_generations.add(value)
    first_generation = min(generation_contract)
    hidden = sorted(
        generation for generation in observed_generations
        if generation >= first_generation and generation not in generation_contract
    )
    if hidden:
        raise _evidence_issue("FAIL", "phase_hidden_reset", f"phase trace contains hidden reset generations: {hidden}")

    runs: dict[int, list[dict[str, object]]] = {
        generation: [] for generation in generation_contract
    }
    active: dict[str, object] | None = None
    for row in loops:
        try:
            before_generation = row["reset_generation"]
            after_raw = row.get("reset_generation_after_ground_truth")
            generation = before_generation if after_raw is None else after_raw
            if (
                isinstance(before_generation, bool)
                or not isinstance(before_generation, int)
                or isinstance(generation, bool)
                or not isinstance(generation, int)
            ):
                raise TypeError("generation type")
            stamp = float(row["simulation_time_after_app_s"])
        except (KeyError, TypeError, ValueError):
            raise _evidence_issue("AMBIGUOUS", "phase_truncated", "phase loop is missing generation/time")
        if not math.isfinite(stamp):
            raise _evidence_issue("FAIL", "phase_time_nonfinite", "phase simulation time is non-finite")
        if generation != before_generation:
            if active is not None:
                runs[int(active["generation"])].append(active)
                active = None
            continue
        assist = row.get("assist") or row.get("pre_app_assist")
        target = assist.get("target") if isinstance(assist, dict) else None
        if not isinstance(target, list) or len(target) != 2:
            raise _evidence_issue("AMBIGUOUS", "phase_target_missing", "phase assist target is missing")
        try:
            command = (float(target[0]), float(target[1]))
        except (TypeError, ValueError):
            raise _evidence_issue("FAIL", "phase_target_invalid", "phase assist target is invalid")
        if not all(math.isfinite(value) for value in command):
            raise _evidence_issue("FAIL", "phase_target_nonfinite", "phase assist target is non-finite")
        if generation not in generation_contract:
            if abs(command[0]) > 1.0e-12 or abs(command[1]) > 1.0e-12:
                raise _evidence_issue("FAIL", "phase_extra_generation", "nonzero command belongs to an unreported reset")
            continue
        key = (generation, command)
        if active is None or active["key"] != key:
            if active is not None:
                runs[int(active["generation"])].append(active)
            active = {
                "key": key,
                "generation": generation,
                "linear": command[0],
                "angular": command[1],
                "start_s": stamp,
                "end_s": stamp,
                "stamps": [stamp],
            }
        else:
            active["end_s"] = stamp
            stamps = active["stamps"]
            assert isinstance(stamps, list)
            stamps.append(stamp)
    if active is not None:
        runs[int(active["generation"])].append(active)

    windows: list[dict[str, object]] = []
    primitive_contracts = {
        identifier: tuple(zip(commands, durations))
        for identifier, commands, durations in zip(
            EXPECTED_PRIMITIVE_IDS, EXPECTED_COMMANDS, EXPECTED_DURATIONS
        )
    }
    for generation, (identifier, expected_count, is_stationary) in generation_contract.items():
        candidates = runs[generation]
        if is_stationary:
            zero = [
                item for item in candidates
                if abs(float(item["linear"])) <= 1.0e-12
                and abs(float(item["angular"])) <= 1.0e-12
                and float(item["end_s"]) - float(item["start_s"]) >= 10.0
            ]
            if len(zero) != 1:
                raise _evidence_issue("AMBIGUOUS", "stationary_phase_window", "exact stationary 10 s phase window is unavailable")
            selected = dict(zero[0])
            selected["id"] = identifier
            selected["expected_duration_s"] = EXPECTED_STATIONARY["duration_sec"]
            selected["observed_command_duration_s"] = float(selected["end_s"]) - float(selected["start_s"])
            selected["duration_tolerance_s"] = 0.25
            selected["end_s"] = float(selected["start_s"]) + EXPECTED_STATIONARY["duration_sec"]
            selected.pop("key", None)
            selected.pop("stamps", None)
            windows.append(selected)
            continue
        nonzero = [
            item for item in candidates
            if abs(float(item["linear"])) > 1.0e-12
            or abs(float(item["angular"])) > 1.0e-12
        ]
        if len(nonzero) != expected_count:
            raise _evidence_issue(
                "AMBIGUOUS", "primitive_phase_windows",
                f"{identifier} expected {expected_count} command windows, got {len(nonzero)}",
            )
        observed_commands = tuple(
            (float(item["linear"]), float(item["angular"])) for item in nonzero
        )
        contracts = primitive_contracts[identifier]
        if observed_commands != tuple(item[0] for item in contracts):
            raise _evidence_issue(
                "FAIL", "primitive_phase_commands",
                f"{identifier} phase commands do not match the report contract",
            )
        for index, item in enumerate(nonzero):
            selected = dict(item)
            selected["id"] = (
                f"{identifier}[{index}]" if expected_count > 1 else identifier
            )
            selected["segment_index"] = index
            expected_duration = float(contracts[index][1])
            stamps = [float(value) for value in selected.get("stamps", [])]
            periods = [right - left for left, right in zip(stamps, stamps[1:]) if right > left]
            phase_period = statistics.median(periods) if periods else None
            if phase_period is None or not math.isfinite(phase_period) or phase_period <= 0.0:
                raise _evidence_issue("AMBIGUOUS", "primitive_phase_period", f"{selected['id']} phase period is unavailable")
            tolerance = min(max(2.0 * phase_period, 0.02 * expected_duration), 0.25)
            observed_duration = float(selected["end_s"]) - float(selected["start_s"])
            if observed_duration < expected_duration - tolerance:
                raise _evidence_issue(
                    "AMBIGUOUS", "primitive_phase_short",
                    f"{selected['id']} command window {observed_duration:.3f}s is shorter than {expected_duration:.3f}s",
                )
            if observed_duration > expected_duration + tolerance:
                raise _evidence_issue(
                    "FAIL", "primitive_phase_long",
                    f"{selected['id']} command window {observed_duration:.3f}s exceeds {expected_duration:.3f}s",
                )
            selected["expected_duration_s"] = expected_duration
            selected["observed_command_duration_s"] = observed_duration
            selected["duration_tolerance_s"] = tolerance
            selected["phase_period_s"] = phase_period
            selected.pop("key", None)
            selected.pop("stamps", None)
            windows.append(selected)
    return windows


def phase_window_metrics(
    phase: Sequence[dict[str, Any]],
    *,
    start_s: float,
    end_s: float,
    reset_generation: int | None = None,
) -> dict[str, object]:
    """Validate all four IMU graph attributes and loop phase boundaries."""

    rows = [
        row for row in phase
        if row.get("kind") == "loop"
        and start_s <= float(row.get("simulation_time_after_app_s", math.inf)) <= end_s
    ]
    attributes = {
        key: {"valid_count": 0, "null_count": 0, "error_count": 0}
        for key in (
            "read_imu_ang_vel",
            "read_imu_sensor_time_s",
            "publish_imu_angular_velocity",
            "publish_imu_timestamp_s",
        )
    }
    fail_reasons: list[str] = []
    ambiguous_reasons: list[str] = []
    sensor_stamps: list[float] = []
    publish_stamps: list[float] = []
    sensor_publish_deltas: list[float] = []
    forward_deltas: list[float] = []
    yaw_rate_deltas: list[float] = []
    if not rows:
        ambiguous_reasons.append("empty_phase_window")
    for row in rows:
        before_generation = row.get("reset_generation")
        after_generation = row.get("reset_generation_after_ground_truth")
        effective_after = before_generation if after_generation is None else after_generation
        if (
            isinstance(before_generation, bool)
            or not isinstance(before_generation, int)
            or isinstance(effective_after, bool)
            or not isinstance(effective_after, int)
            or before_generation != effective_after
            or (reset_generation is not None and before_generation != reset_generation)
        ):
            fail_reasons.append("reset_generation_crossing")
        boundaries = [
            row.get("before_app_monotonic_ns"),
            row.get("after_app_monotonic_ns"),
            row.get("after_assist_monotonic_ns"),
            row.get("before_ground_truth_monotonic_ns"),
            row.get("after_ground_truth_monotonic_ns"),
        ]
        if any(isinstance(value, bool) or not isinstance(value, int) for value in boundaries):
            fail_reasons.append("monotonic_boundary_invalid")
        elif any(right < left for left, right in zip(boundaries, boundaries[1:])):
            fail_reasons.append("monotonic_boundary_order")
        before = row.get("pre_assist_body")
        after = row.get("post_assist_body")
        if isinstance(before, dict) and isinstance(after, dict):
            for key, target in (
                ("forward_speed_mps", forward_deltas),
                ("yaw_rate_radps", yaw_rate_deltas),
            ):
                left = before.get(key)
                right = after.get(key)
                if left is not None and right is not None:
                    try:
                        delta = float(right) - float(left)
                    except (TypeError, ValueError):
                        fail_reasons.append(f"assist_{key}_invalid")
                    else:
                        if math.isfinite(delta):
                            target.append(delta)
                        else:
                            fail_reasons.append(f"assist_{key}_nonfinite")
        graph = row.get("imu_graph_after_app")
        if not isinstance(graph, dict):
            ambiguous_reasons.append("imu_graph_missing")
            continue
        row_values: dict[str, float | list[float] | None] = {}
        for key, counts in attributes.items():
            field = graph.get(key)
            if not isinstance(field, dict) or "value" not in field or "error" not in field:
                counts["null_count"] += 1
                ambiguous_reasons.append(f"{key}_missing")
                row_values[key] = None
                continue
            error = field["error"]
            value = field["value"]
            if error is not None:
                counts["error_count"] += 1
                detail = str(error).lower()
                if any(token in detail for token in ("non-finite", "3-vector", "timestamp must")):
                    fail_reasons.append(f"{key}_invalid")
                else:
                    ambiguous_reasons.append(f"{key}_error")
                row_values[key] = None
                continue
            if value is None:
                counts["null_count"] += 1
                ambiguous_reasons.append(f"{key}_null")
                row_values[key] = None
                continue
            try:
                if key in {"read_imu_ang_vel", "publish_imu_angular_velocity"}:
                    normalized: float | list[float] = [float(item) for item in value]
                    if len(normalized) != 3:
                        raise ValueError("shape")
                    numeric_values = normalized
                else:
                    normalized = float(value)
                    numeric_values = [normalized]
            except (TypeError, ValueError):
                fail_reasons.append(f"{key}_shape")
                row_values[key] = None
                continue
            if not all(math.isfinite(item) for item in numeric_values):
                fail_reasons.append(f"{key}_nonfinite")
                row_values[key] = None
                continue
            counts["valid_count"] += 1
            row_values[key] = normalized
        sensor = row_values.get("read_imu_sensor_time_s")
        publish = row_values.get("publish_imu_timestamp_s")
        if isinstance(sensor, float):
            sensor_stamps.append(sensor)
        if isinstance(publish, float):
            publish_stamps.append(publish)
        if isinstance(sensor, float) and isinstance(publish, float):
            sensor_publish_deltas.append(publish - sensor)

    for name, stamps in (("sensor", sensor_stamps), ("publish", publish_stamps)):
        if len(stamps) >= 2 and any(right <= left for left, right in zip(stamps, stamps[1:])):
            fail_reasons.append(f"{name}_stamp_not_strict")
    for key, counts in attributes.items():
        if counts["valid_count"] == 0:
            ambiguous_reasons.append(f"{key}_no_valid_samples")
    status = "FAIL" if fail_reasons else "AMBIGUOUS" if ambiguous_reasons else "OK"
    return {
        "status": status,
        "loop_count": len(rows),
        "attributes": attributes,
        "fail_reasons": sorted(set(fail_reasons)),
        "ambiguous_reasons": sorted(set(ambiguous_reasons)),
        "assist_forward_delta_mps_median": statistics.median(forward_deltas) if forward_deltas else None,
        "assist_forward_delta_mps_max_abs": max(map(abs, forward_deltas)) if forward_deltas else None,
        "assist_yaw_rate_delta_radps_median": statistics.median(yaw_rate_deltas) if yaw_rate_deltas else None,
        "assist_yaw_rate_delta_radps_max_abs": max(map(abs, yaw_rate_deltas)) if yaw_rate_deltas else None,
        "sensor_publish_stamp_delta_s_max_abs": max(map(abs, sensor_publish_deltas)) if sensor_publish_deltas else None,
        "read_imu_stamp_quality": _stamp_quality([ScalarSample(value, 0.0) for value in sensor_stamps]),
        "publish_imu_stamp_quality": _stamp_quality([ScalarSample(value, 0.0) for value in publish_stamps]),
        "monotonic_order_violation_count": sum(reason.startswith("monotonic_") for reason in fail_reasons),
        "imu_graph_error_count": sum(item["error_count"] for item in attributes.values()),
    }


def run_analysis(
    *,
    mcap: Path,
    phase_jsonl: Path,
    benchmark_report: Path,
    goal_evaluator: Path | None = None,
    goal_mcap: Path | None = None,
    config_path: Path | None = None,
    spawn_poses_path: Path | None = None,
) -> dict[str, object]:
    inputs = {
        "mcap": str(mcap),
        "phase_jsonl": str(phase_jsonl),
        "benchmark_report": str(benchmark_report),
        "goal_evaluator": None if goal_evaluator is None else str(goal_evaluator),
        "goal_mcap": None if goal_mcap is None else str(goal_mcap),
        "config": None if config_path is None else str(config_path),
        "spawn_poses_file": None if spawn_poses_path is None else str(spawn_poses_path),
    }
    try:
        resources = resolve_diagnostic_resources(config_path, spawn_poses_path)
        inputs["config"] = str(resources.config_path)
        inputs["spawn_poses_file"] = str(resources.spawn_poses_path)
        try:
            report = _load_json(benchmark_report)
        except (OSError, json.JSONDecodeError) as exc:
            raise _evidence_issue("AMBIGUOUS", "benchmark_unreadable", f"benchmark report is unavailable/truncated: {exc}") from exc
        validate_benchmark_report(report, resources)
        try:
            phase = _load_phase(phase_jsonl)
        except OSError as exc:
            raise _evidence_issue("AMBIGUOUS", "phase_unreadable", f"phase trace is unavailable: {exc}") from exc
        phase_provenance = validate_phase_trace(phase, resources)
        try:
            streams = load_mcap(mcap)
        except EvidenceError:
            raise
        except Exception as exc:
            raise _evidence_issue("AMBIGUOUS", "mcap_unreadable", f"MCAP could not be read: {type(exc).__name__}: {exc}") from exc
        windows = command_windows(phase, report)
        results = []
        for window in windows:
            start_s = float(window["start_s"])
            end_s = float(window["end_s"])
            result = analyze_segment(
                identifier=str(window["id"]),
                reset_generation=int(window["generation"]),
                command_linear_mps=float(window["linear"]),
                command_angular_radps=float(window["angular"]),
                raw=_window_scalar(streams["/imu/data_raw"], start_s, end_s),
                corrected=_window_scalar(streams["/imu/data"], start_s, end_s),
                ground_truth=_window_yaw(streams["/ground_truth/odom"], start_s, end_s),
            )
            result["phase_metrics"] = phase_window_metrics(
                phase,
                start_s=start_s,
                end_s=end_s,
                reset_generation=int(window["generation"]),
            )
            phase_status = result["phase_metrics"]["status"]
            if phase_status == "FAIL" and result.get("status") == "OK":
                result["status"] = "PHASE_TRACE_INVALID"
            elif phase_status == "AMBIGUOUS" and result.get("status") == "OK":
                result["status"] = "PHASE_TRACE_INSUFFICIENT"
            expected_duration = float(window["expected_duration_s"])
            tolerance = float(window["duration_tolerance_s"])
            observed_command_duration = float(window["observed_command_duration_s"])
            coverage = result.get("coverage")
            coverage_duration = (
                float(coverage["duration_s"])
                if isinstance(coverage, dict)
                and isinstance(coverage.get("duration_s"), (int, float))
                else None
            )
            duration_status = "OK"
            if coverage_duration is None:
                duration_status = "AMBIGUOUS"
            elif coverage_duration < expected_duration - tolerance - 1.0e-6:
                duration_status = "AMBIGUOUS"
                if result.get("status") == "OK":
                    result["status"] = "DURATION_INSUFFICIENT"
            elif coverage_duration > expected_duration + tolerance + 1.0e-6:
                duration_status = "FAIL"
                result["status"] = "DURATION_INVALID"
            result["duration_contract"] = {
                "status": duration_status,
                "expected_duration_s": expected_duration,
                "observed_command_duration_s": observed_command_duration,
                "common_stream_coverage_duration_s": coverage_duration,
                "tolerance_s": tolerance,
                "tolerance_policy": "max(two_phase_periods,2_percent),capped_0.25s",
            }
            results.append(result)
        goal = None
        if goal_mcap is not None and goal_evaluator is None:
            raise _evidence_issue("AMBIGUOUS", "goal_metadata_missing", "--goal-mcap requires --goal-evaluator metadata")
        if goal_mcap is not None and goal_evaluator is not None:
            try:
                goal_metadata = _load_json(goal_evaluator)
            except (OSError, json.JSONDecodeError) as exc:
                raise _evidence_issue("AMBIGUOUS", "goal_unreadable", f"goal evidence is unavailable/truncated: {exc}") from exc
            try:
                goal = load_goal_mcap(goal_mcap, goal_metadata)
            except EvidenceError:
                raise
            except Exception as exc:
                raise _evidence_issue("AMBIGUOUS", "goal_mcap_unreadable", f"goal MCAP could not be read: {type(exc).__name__}: {exc}") from exc
        phase_valid = all(
            item.get("phase_metrics", {}).get("status") == "OK"
            for item in results
        )
        stationary_valid = any(
            item.get("id") == "stationary_reference" and item.get("status") == "OK"
            for item in results
        )
        summary = summarize(
            results,
            goal=goal,
            benchmark_valid=True,
            stationary_valid=stationary_valid,
            phase_valid=phase_valid,
        )
        if goal_mcap is None:
            summary["goal_evidence_issue"] = {
                "verdict": "AMBIGUOUS",
                "code": "goal_mcap_missing",
                "detail": "--goal-mcap is required; JSON arrays are never goal authority",
            }
            if summary["verdict"] not in {"FAIL"}:
                summary["verdict"] = "AMBIGUOUS"
        summary["inputs"] = inputs
        summary["mcap_provenance"] = getattr(streams, "provenance", None)
        summary["phase_provenance"] = phase_provenance
        summary["resolved_diagnostic_config"] = resources.identity
        summary["goal_mcap_provenance"] = (
            None if goal is None else goal.get("mcap_provenance")
        )
        summary["phase_trace_loop_count"] = sum(row.get("kind") == "loop" for row in phase)
        summary["command_window_count"] = len(windows)
        summary["evidence_errors"] = []
        return summary
    except EvidenceError as exc:
        return {
            "verdict": exc.verdict,
            "segments": [],
            "bins": {},
            "goal_identity_non_degrade_interval": None,
            "global_scale_intersection": [],
            "inputs": inputs,
            "command_window_count": 0,
            "phase_trace_loop_count": 0,
            "evidence_errors": [
                {"verdict": exc.verdict, "code": exc.code, "detail": exc.detail}
            ],
        }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mcap", type=Path, required=True)
    parser.add_argument("--phase-jsonl", type=Path, required=True)
    parser.add_argument("--benchmark-report", type=Path, required=True)
    parser.add_argument("--goal-evaluator", type=Path)
    parser.add_argument("--goal-mcap", type=Path)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--spawn-poses-file", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    result = run_analysis(
        mcap=args.mcap.expanduser().resolve(),
        phase_jsonl=args.phase_jsonl.expanduser().resolve(),
        benchmark_report=args.benchmark_report.expanduser().resolve(),
        goal_evaluator=(
            None
            if args.goal_evaluator is None
            else args.goal_evaluator.expanduser().resolve()
        ),
        goal_mcap=(
            None if args.goal_mcap is None else args.goal_mcap.expanduser().resolve()
        ),
        config_path=(
            None if args.config is None else args.config.expanduser().resolve()
        ),
        spawn_poses_path=(
            None
            if args.spawn_poses_file is None
            else args.spawn_poses_file.expanduser().resolve()
        ),
    )
    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"verdict": result["verdict"], "output": str(output)}, sort_keys=True))
    return 0 if result["verdict"] in {"PASS_CANDIDATE", "CONFIRMED_NO_GLOBAL_CONSTANT"} else 2


if __name__ == "__main__":
    raise SystemExit(main())
