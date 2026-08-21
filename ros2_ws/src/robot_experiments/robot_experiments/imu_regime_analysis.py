"""Offline V6 IMU regime analysis; this module never controls a robot."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import statistics
from typing import Any, Iterable, Sequence


K_MIN = 0.90
K_MAX = 1.02
K_STEP = 0.0001
YAW_LIMIT_RAD = math.radians(5.0)


@dataclass(frozen=True)
class ScalarSample:
    stamp_s: float
    value: float


@dataclass(frozen=True)
class YawSample:
    stamp_s: float
    yaw_rad: float


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
        "median_period_s": (
            statistics.median(positive_deltas) if positive_deltas else None
        ),
    }


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
    """Compute one reset/command-window result from measured samples."""

    qualities = {
        "raw": _stamp_quality(raw),
        "corrected": _stamp_quality(corrected),
        "ground_truth": _stamp_quality(ground_truth),
    }
    invalid_stamps = any(
        quality["duplicate_count"]
        or quality["backward_count"]
        or quality["nonfinite_count"]
        for quality in qualities.values()
    )
    if len(raw) < 3 or len(corrected) < 3 or len(ground_truth) < 3:
        return {
            "id": identifier,
            "reset_generation": int(reset_generation),
            "status": "INSUFFICIENT_DATA",
            "stamp_quality": qualities,
            "scale_interval_le_5deg": [],
        }
    raw_times, raw_integrated = _integral(raw)
    corrected_times, corrected_integrated = _integral(corrected)
    gt_unwrapped = _unwrap([sample.yaw_rad for sample in ground_truth])
    gt_delta = gt_unwrapped[-1] - gt_unwrapped[0]
    raw_delta = raw_integrated[-1]
    corrected_delta = corrected_integrated[-1]
    k_star = gt_delta / raw_delta if abs(raw_delta) > 1.0e-12 else None
    raw_errors = _aligned_errors(raw_times, raw_integrated, ground_truth)
    corrected_errors = _aligned_errors(
        corrected_times, corrected_integrated, ground_truth
    )
    allowed = [
        k
        for k in _grid()
        if abs(k * raw_delta - gt_delta) <= YAW_LIMIT_RAD
    ]
    duration = min(raw[-1].stamp_s, ground_truth[-1].stamp_s) - max(
        raw[0].stamp_s, ground_truth[0].stamp_s
    )
    steady_start = raw[0].stamp_s + min(0.5, max(0.0, duration * 0.1))
    steady_end = raw[-1].stamp_s - min(0.5, max(0.0, duration * 0.1))
    steady_raw = [
        item.value for item in raw if steady_start <= item.stamp_s <= steady_end
    ]
    gt_rate = gt_delta / duration if duration > 0.0 else None
    raw_rate = statistics.median(steady_raw) if steady_raw else None
    return {
        "id": identifier,
        "reset_generation": int(reset_generation),
        "status": "STAMP_INVALID" if invalid_stamps else "OK",
        "command_linear_mps": float(command_linear_mps),
        "command_angular_radps": float(command_angular_radps),
        "direction": "CCW" if command_angular_radps > 0 else "CW" if command_angular_radps < 0 else "ZERO",
        "speed_bin_mps": abs(float(command_linear_mps)),
        "stamp_quality": qualities,
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


def goal_identity_non_degrade_interval(goal: dict[str, Any] | None) -> list[list[float]] | None:
    """Scan goal yaw series when supplied; absence is explicitly unknown."""

    if goal is None:
        return None
    raw = goal.get("raw_integrated_yaw_rad")
    gt = goal.get("ground_truth_relative_yaw_rad")
    if not isinstance(raw, list) or not isinstance(gt, list) or len(raw) != len(gt) or len(raw) < 3:
        return None
    try:
        raw_values = [float(value) for value in raw]
        gt_values = [float(value) for value in gt]
    except (TypeError, ValueError):
        return None
    identity = _rmse([left - right for left, right in zip(raw_values, gt_values)])
    if identity is None:
        return None
    valid = []
    for k in _grid():
        error = _rmse([k * left - right for left, right in zip(raw_values, gt_values)])
        if error is not None and error <= identity + 1.0e-12:
            valid.append(k)
    return _contiguous_interval(valid)


def summarize(
    segments: Sequence[dict[str, object]], *, goal: dict[str, Any] | None = None
) -> dict[str, object]:
    usable = [segment for segment in segments if segment.get("status") == "OK"]
    invalid = [segment for segment in segments if segment.get("status") != "OK"]
    segment_intervals = [segment["scale_interval_le_5deg"] for segment in usable]
    goal_interval = goal_identity_non_degrade_interval(goal)
    all_sets = list(segment_intervals)
    if goal_interval is not None:
        all_sets.append(goal_interval)
    intersection = _interval_intersection(all_sets) if all_sets else []
    if invalid:
        verdict = "FAIL" if any(item.get("status") == "STAMP_INVALID" for item in invalid) else "AMBIGUOUS"
    elif not usable:
        verdict = "FAIL"
    elif not intersection:
        verdict = "CONFIRMED_NO_GLOBAL_CONSTANT"
    elif goal_interval is None:
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
        "global_scale_intersection": intersection,
    }


def _load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _load_phase(path: Path) -> list[dict[str, Any]]:
    return [
        json.loads(line)
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]


def load_mcap(path: Path) -> dict[str, list[ScalarSample | YawSample]]:
    """Read the three required topics through the installed rosbag2 MCAP API."""

    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RuntimeError("rosbag2_py MCAP support is unavailable") from exc
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    required = ("/imu/data_raw", "/imu/data", "/ground_truth/odom")
    missing = [topic for topic in required if topic not in topic_types]
    if missing:
        raise RuntimeError(f"MCAP missing required topics: {missing}")
    message_types = {topic: get_message(topic_types[topic]) for topic in required}
    streams: dict[str, list[ScalarSample | YawSample]] = {topic: [] for topic in required}
    while reader.has_next():
        topic, payload, _recorded_ns = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(payload, message_types[topic])
        stamp = message.header.stamp
        stamp_s = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
        if topic == "/ground_truth/odom":
            q = message.pose.pose.orientation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            streams[topic].append(YawSample(stamp_s, yaw))
        else:
            streams[topic].append(
                ScalarSample(stamp_s, float(message.angular_velocity.z))
            )
    return streams


def command_windows(
    phase: Sequence[dict[str, Any]], report: dict[str, Any]
) -> list[dict[str, object]]:
    """Associate nonzero target windows with benchmark reset receipts and IDs."""

    loops = [row for row in phase if row.get("kind") == "loop"]
    primitives = report.get("primitives", [])
    by_generation = {
        int(item["reset_receipt"]["generation"]): str(item["id"])
        for item in primitives
        if isinstance(item, dict)
        and isinstance(item.get("reset_receipt"), dict)
        and isinstance(item["reset_receipt"].get("generation"), int)
    }
    windows: list[dict[str, object]] = []
    active: dict[str, object] | None = None
    for row in loops:
        before_generation = int(row.get("reset_generation", -1))
        generation = int(
            row.get("reset_generation_after_ground_truth")
            if row.get("reset_generation_after_ground_truth") is not None
            else row.get("reset_generation", -1)
        )
        # A reset that changes generation inside this render loop is a fence,
        # not a command sample belonging to either adjacent window.
        if generation != before_generation:
            if active is not None:
                active["end_s"] = float(
                    row.get(
                        "simulation_time_before_app_s",
                        row.get("simulation_time_after_app_s"),
                    )
                )
                if abs(float(active["linear"])) > 0.0 or abs(float(active["angular"])) > 0.0:
                    windows.append(active)
                active = None
            continue
        assist = row.get("assist") or row.get("pre_app_assist") or {}
        target = assist.get("target") if isinstance(assist, dict) else None
        if not isinstance(target, list) or len(target) != 2:
            continue
        command = (float(target[0]), float(target[1]))
        stamp = float(row.get("simulation_time_after_app_s", row.get("simulation_time_before_app_s")))
        key = (generation, command)
        if active is None or active["key"] != key:
            if active is not None and (
                abs(float(active["linear"])) > 0.0 or abs(float(active["angular"])) > 0.0
            ):
                active["end_s"] = stamp
                windows.append(active)
            active = {
                "key": key,
                "generation": generation,
                "linear": command[0],
                "angular": command[1],
                "start_s": stamp,
                "end_s": stamp,
                "id": by_generation.get(generation, f"generation_{generation}"),
            }
        else:
            active["end_s"] = stamp
    if active is not None and (
        abs(float(active["linear"])) > 0.0 or abs(float(active["angular"])) > 0.0
    ):
        windows.append(active)

    # The required stationary reference is deliberately outside
    # MotionBenchmark. Recover one continuous >=10 s zero-target interval from
    # the same passive phase trace and keep it reset-generation bounded.
    stationary_runs: list[dict[str, object]] = []
    stationary: dict[str, object] | None = None
    for row in loops:
        before_generation = int(row.get("reset_generation", -1))
        after_generation = row.get("reset_generation_after_ground_truth")
        generation = (
            before_generation
            if after_generation is None
            else int(after_generation)
        )
        assist = row.get("assist") or row.get("pre_app_assist") or {}
        target = assist.get("target") if isinstance(assist, dict) else None
        stamp = float(
            row.get(
                "simulation_time_after_app_s",
                row.get("simulation_time_before_app_s", math.nan),
            )
        )
        is_zero = (
            generation == before_generation
            and isinstance(target, list)
            and len(target) == 2
            and float(target[0]) == 0.0
            and float(target[1]) == 0.0
            and math.isfinite(stamp)
        )
        if is_zero and (
            stationary is None or stationary["generation"] == generation
        ):
            if stationary is None:
                stationary = {
                    "generation": generation,
                    "linear": 0.0,
                    "angular": 0.0,
                    "start_s": stamp,
                    "end_s": stamp,
                    "id": "stationary_external_10s",
                }
            else:
                stationary["end_s"] = stamp
        else:
            if stationary is not None:
                stationary_runs.append(stationary)
            stationary = None
    if stationary is not None:
        stationary_runs.append(stationary)
    eligible_stationary = [
        item
        for item in stationary_runs
        if float(item["end_s"]) - float(item["start_s"]) >= 10.0
    ]
    if eligible_stationary:
        selected = eligible_stationary[0]
        selected["end_s"] = float(selected["start_s"]) + 10.0
        windows.insert(0, selected)
    return windows


def phase_window_metrics(
    phase: Sequence[dict[str, Any]], *, start_s: float, end_s: float
) -> dict[str, object]:
    rows = [
        row
        for row in phase
        if row.get("kind") == "loop"
        and start_s
        <= float(row.get("simulation_time_after_app_s", math.inf))
        <= end_s
    ]
    forward_deltas: list[float] = []
    yaw_rate_deltas: list[float] = []
    publish_stamps: list[float] = []
    sensor_publish_deltas: list[float] = []
    graph_errors = 0
    monotonic_order_violations = 0
    for row in rows:
        before = row.get("pre_assist_body", {})
        after = row.get("post_assist_body", {})
        if isinstance(before, dict) and isinstance(after, dict):
            if before.get("forward_speed_mps") is not None and after.get("forward_speed_mps") is not None:
                forward_deltas.append(
                    float(after["forward_speed_mps"])
                    - float(before["forward_speed_mps"])
                )
            if before.get("yaw_rate_radps") is not None and after.get("yaw_rate_radps") is not None:
                yaw_rate_deltas.append(
                    float(after["yaw_rate_radps"])
                    - float(before["yaw_rate_radps"])
                )
        graph = row.get("imu_graph_after_app", {})
        if isinstance(graph, dict):
            values: dict[str, float | None] = {}
            for key in (
                "read_imu_sensor_time_s",
                "publish_imu_timestamp_s",
            ):
                field = graph.get(key, {})
                if not isinstance(field, dict) or field.get("error") is not None:
                    graph_errors += 1
                    values[key] = None
                else:
                    value = field.get("value")
                    values[key] = None if value is None else float(value)
            sensor_stamp = values["read_imu_sensor_time_s"]
            publish_stamp = values["publish_imu_timestamp_s"]
            if publish_stamp is not None:
                publish_stamps.append(publish_stamp)
            if sensor_stamp is not None and publish_stamp is not None:
                sensor_publish_deltas.append(publish_stamp - sensor_stamp)
        boundaries = [
            row.get("before_app_monotonic_ns"),
            row.get("after_app_monotonic_ns"),
            row.get("after_assist_monotonic_ns"),
            row.get("before_ground_truth_monotonic_ns"),
            row.get("after_ground_truth_monotonic_ns"),
        ]
        if any(value is None for value in boundaries) or any(
            int(right) < int(left)
            for left, right in zip(boundaries, boundaries[1:])
            if left is not None and right is not None
        ):
            monotonic_order_violations += 1
    stamp_samples = [ScalarSample(value, 0.0) for value in publish_stamps]
    return {
        "loop_count": len(rows),
        "assist_forward_delta_mps_median": (
            statistics.median(forward_deltas) if forward_deltas else None
        ),
        "assist_forward_delta_mps_max_abs": (
            max(abs(value) for value in forward_deltas) if forward_deltas else None
        ),
        "assist_yaw_rate_delta_radps_median": (
            statistics.median(yaw_rate_deltas) if yaw_rate_deltas else None
        ),
        "assist_yaw_rate_delta_radps_max_abs": (
            max(abs(value) for value in yaw_rate_deltas) if yaw_rate_deltas else None
        ),
        "imu_graph_error_count": graph_errors,
        "sensor_publish_stamp_delta_s_max_abs": (
            max(abs(value) for value in sensor_publish_deltas)
            if sensor_publish_deltas
            else None
        ),
        "publish_imu_stamp_quality": _stamp_quality(stamp_samples),
        "monotonic_order_violation_count": monotonic_order_violations,
    }


def run_analysis(
    *,
    mcap: Path,
    phase_jsonl: Path,
    benchmark_report: Path,
    goal_evaluator: Path | None = None,
) -> dict[str, object]:
    streams = load_mcap(mcap)
    phase = _load_phase(phase_jsonl)
    report = _load_json(benchmark_report)
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
            phase, start_s=start_s, end_s=end_s
        )
        if (
            result["phase_metrics"]["imu_graph_error_count"]
            or result["phase_metrics"]["monotonic_order_violation_count"]
        ) and result.get("status") == "OK":
            result["status"] = "PHASE_TRACE_INVALID"
        results.append(result)
    goal = _load_json(goal_evaluator) if goal_evaluator is not None else None
    summary = summarize(results, goal=goal)
    summary["inputs"] = {
        "mcap": str(mcap),
        "phase_jsonl": str(phase_jsonl),
        "benchmark_report": str(benchmark_report),
        "goal_evaluator": None if goal_evaluator is None else str(goal_evaluator),
    }
    summary["phase_trace_loop_count"] = sum(
        row.get("kind") == "loop" for row in phase
    )
    summary["command_window_count"] = len(windows)
    return summary


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mcap", type=Path, required=True)
    parser.add_argument("--phase-jsonl", type=Path, required=True)
    parser.add_argument("--benchmark-report", type=Path, required=True)
    parser.add_argument("--goal-evaluator", type=Path)
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
    )
    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"verdict": result["verdict"], "output": str(output)}, sort_keys=True))
    return 0 if result["verdict"] not in {"FAIL"} else 2


if __name__ == "__main__":
    raise SystemExit(main())
