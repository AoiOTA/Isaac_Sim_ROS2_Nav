"""Strict offline aggregation for skid-steer contact-profile A/B reports.

The tool audits identities before aggregating measurements.  It never ranks
profiles or changes a robot configuration; its output is evidence for a later
engineering decision, not that decision itself.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

from .configuration import ConfigurationError
from .report import (
    ReportValidationError,
    _atomic_text_write,
    validate_runtime_provenance,
)


COMPLETE_MATRIX_ENVIRONMENTS = ("Warehouse", "SimplePlane")
COMPLETE_MATRIX_PROFILES = (
    "legacy_baseline",
    "threshold_corr_0p00025_offset_0p0004",
    "threshold_corr_0p00025_offset_0p04",
    "threshold_corr_0p025_offset_0p0004",
    "threshold_corr_0p025_offset_0p04",
    "explicit_material",
)
COMPLETE_MATRIX_ENVIRONMENT_TOPOLOGIES = (
    ("SimplePlane", "simple_plane_only1_v1"),
    ("Warehouse", "warehouse_combined32_v1"),
    ("Warehouse", "warehouse_plane_only1_v1"),
)
_SHIPPED_TOPOLOGY_ENVIRONMENTS = {
    topology: environment
    for environment, topology in COMPLETE_MATRIX_ENVIRONMENT_TOPOLOGIES
}
CANONICAL_WHEEL_RADIUS_M = 0.098

_MOTION_PROFILE_ID = "jackal_skid_steer_ab_v1"
_IDENTIFIER_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")
_WHEEL_KEYS = ("front_left", "front_right", "rear_left", "rear_right")
_LEFT_WHEELS = ("front_left", "rear_left")
_RIGHT_WHEELS = ("front_right", "rear_right")
_SEGMENT_SPECS = (
    ("rotate_left_360", "rotate_left", 0.0, 0.4, 2.0 * math.pi / 0.4),
    ("rotate_right_360", "rotate_right", 0.0, -0.4, 2.0 * math.pi / 0.4),
    ("forward_3m", "forward", 0.5, 0.0, 3.0 / 0.5),
    ("backward_2m", "backward", -0.3, 0.0, 2.0 / 0.3),
    ("arc_left_5s", "arc_left", 0.4, 0.4, 5.0),
    ("arc_right_5s", "arc_right", 0.4, -0.4, 5.0),
)
_SEGMENT_BY_ID = {specification[0]: specification for specification in _SEGMENT_SPECS}
_REPORT_KEYS = {
    "schema_version",
    "diagnostic",
    "profile_id",
    "environment_id",
    "odometry_mode",
    "config_file",
    "config_sha256",
    "output_file",
    "started_at_utc",
    "completed_at_utc",
    "configuration",
    "runtime_provenance",
    "segments",
    "timestamp_integrity",
    "safety",
    "result",
    "failure_reason",
    "failed_segments",
}
_CONFIGURATION_KEYS = {
    "schema_version",
    "profile_id",
    "topics",
    "reset",
    "sampling",
    "limits",
    "stop",
    "wheels",
    "segments",
}
_CONFIGURED_SEGMENT_KEYS = {
    "segment_id",
    "motion",
    "tier",
    "linear_x_mps",
    "angular_z_radps",
    "duration_sec",
}
_RUNTIME_PROVENANCE_KEYS = {
    "verified",
    "schema_version",
    "robot",
    "environment",
    "simulation",
    "contact",
    "git",
}
_RUNTIME_PROVENANCE_V5_KEYS = _RUNTIME_PROVENANCE_KEYS | {
    "ground_topology",
}
_RESULT_SEGMENT_KEYS = {
    "segment_id",
    "motion",
    "tier",
    "result",
    "command",
    "sample_counts",
    "pose",
    "yaw",
    "actual_velocity",
    "stopping",
    "wheels",
    "timestamp_integrity",
    "reset",
    "invalid_message_counts",
}
_COMMAND_KEYS = {
    "linear_x_mps",
    "angular_z_radps",
    "configured_duration_sec",
    "observed_duration_sec",
    "publish_count",
    "start_stamp_ns",
    "end_stamp_ns",
}
_POSE_KEYS = {
    "start",
    "end",
    "trajectory_length_m",
    "net_displacement_m",
    "longitudinal_displacement_m",
    "expected_longitudinal_displacement_m",
    "longitudinal_error_m",
    "lateral_displacement_m",
    "lateral_drift_m",
    "translation_drift_m",
}
_YAW_KEYS = {"change_rad", "expected_change_rad", "error_rad"}
_STOPPING_KEYS = {
    "stopped",
    "stationary_onset_after_command_sec",
    "confirmed_after_command_sec",
}
_WHEEL_REPORT_KEYS = {
    "direction",
    "expected_direction",
    "direction_matches",
    "speed_radps",
}
_ACTUAL_VELOCITY_KEYS = {
    "linear_x_mps",
    "linear_y_mps",
    "linear_speed_mps",
    "angular_z_radps",
}
_STEADY_STATE_WINDOW_KEYS = {
    "schema_version",
    "definition",
    "start_stamp_ns",
    "end_stamp_ns",
    "observed_duration_sec",
    "sample_count",
    "angular_z_radps",
}
_VELOCITY_DISTRIBUTION_KEYS = {
    "sample_count",
    "mean",
    "mean_abs",
    "minimum",
    "maximum",
    "peak_abs",
    "rmse",
}
_PHYSICAL_ACCEPTANCE_POLICY_ID = "skid_steer_plan_8_7_v1"
_PHYSICAL_ACCEPTANCE_THRESHOLDS = {
    "forward_abs_lateral_drift_max_m": 0.05,
    "backward_abs_lateral_drift_max_m": 0.08,
    "rotation_center_drift_max_m": 0.10,
    "rotation_center_drift_asymmetry_ratio_max": 0.20,
    "rotation_mean_yaw_rate_absolute_error_fraction_max": 0.10,
    "stop_stable_duration_min_sec": 0.5,
    "stop_linear_velocity_threshold_max_mps": 0.02,
    "stop_angular_velocity_threshold_max_radps": 0.05,
    "stop_wheel_velocity_threshold_max_radps": 0.20,
}
_PHYSICAL_ACCEPTANCE_APPLICABILITY = {
    "required_runtime_provenance_schema": 5,
    "required_environment_id": "SimplePlane",
    "required_ground_topology_id": "simple_plane_only1_v1",
    "required_odometry_mode": "ideal",
    "minimum_unique_repeats_per_group": 3,
}
_PHYSICAL_STEADY_STATE_MEASUREMENT_BASIS = (
    "actual_velocity.steady_state_window.angular_z_radps.mean over the "
    "final_half_of_command_interval window"
)
_PHYSICAL_STEADY_STATE_LEAF_BASIS = (
    "actual_velocity.steady_state_window.angular_z_radps.mean"
)
_PHYSICAL_MAXIMUM_CHECK_THRESHOLDS = {
    "forward_abs_lateral_drift_m": "forward_abs_lateral_drift_max_m",
    "backward_abs_lateral_drift_m": "backward_abs_lateral_drift_max_m",
    "rotate_left_center_drift_m": "rotation_center_drift_max_m",
    "rotate_right_center_drift_m": "rotation_center_drift_max_m",
    "rotation_center_drift_asymmetry_ratio": (
        "rotation_center_drift_asymmetry_ratio_max"
    ),
    "stop_config.linear_velocity_threshold_mps": (
        "stop_linear_velocity_threshold_max_mps"
    ),
    "stop_config.angular_velocity_threshold_radps": (
        "stop_angular_velocity_threshold_max_radps"
    ),
    "stop_config.wheel_velocity_threshold_radps": (
        "stop_wheel_velocity_threshold_max_radps"
    ),
}
_PHYSICAL_MINIMUM_CHECK_THRESHOLDS = {
    "stop_config.stable_duration_sec": "stop_stable_duration_min_sec",
}
_PHYSICAL_YAW_RATE_CHECK_SEGMENTS = {
    "rotate_left_mean_yaw_rate_absolute_error_fraction": "rotate_left_360",
    "rotate_right_mean_yaw_rate_absolute_error_fraction": "rotate_right_360",
}
_PHYSICAL_STOP_WINDOW_CHECK_SEGMENTS = {
    f"stop_window.{segment_id}": segment_id
    for segment_id, *_ in _SEGMENT_SPECS
}
_PHYSICAL_ACCEPTANCE_CHECK_IDS = frozenset(
    {
        *_PHYSICAL_MAXIMUM_CHECK_THRESHOLDS,
        *_PHYSICAL_MINIMUM_CHECK_THRESHOLDS,
        *_PHYSICAL_YAW_RATE_CHECK_SEGMENTS,
        *_PHYSICAL_STOP_WINDOW_CHECK_SEGMENTS,
        "wheel_direction_contract",
    }
)


def _physical_maximum_passed(observed: float, limit: float) -> bool:
    return observed <= limit or math.isclose(
        observed,
        limit,
        rel_tol=1e-12,
        abs_tol=1e-12,
    )


def _physical_minimum_passed(observed: float, limit: float) -> bool:
    return observed >= limit or math.isclose(
        observed,
        limit,
        rel_tol=1e-12,
        abs_tol=1e-12,
    )


@dataclass(frozen=True)
class InputRecord:
    """One fully validated report reduced to identity and metric records."""

    path: str
    sha256: str
    canonical_sha256: str
    report_schema_version: int
    runtime_provenance_schema: int
    environment_id: str
    odometry_mode: str
    ground_topology_id: str | None
    contact_profile_id: str
    global_lock: dict[str, Any]
    environment_lock: dict[str, Any]
    topology_lock: dict[str, Any] | None
    topology_ab_contact_lock: dict[str, Any] | None
    profile_lock: dict[str, Any]
    contact_lock: dict[str, Any]
    physical_stop_contract: dict[str, float] | None
    physical_yaw_rate_metrics: dict[str, dict[str, float]] | None
    segments: dict[str, dict[str, float]]


def _finite(value: Any, location: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigurationError(f"{location} must be a finite number")
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ConfigurationError(f"{location} must be a finite number")
    return parsed


def _positive(value: Any, location: str) -> float:
    parsed = _finite(value, location)
    if parsed <= 0.0:
        raise ConfigurationError(f"{location} must be positive")
    return parsed


def _nonnegative(value: Any, location: str) -> float:
    parsed = _finite(value, location)
    if parsed < 0.0:
        raise ConfigurationError(f"{location} must be non-negative")
    return parsed


def _mapping(value: Any, location: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ConfigurationError(f"{location} must be a mapping")
    return value


def _exact_keys(
    value: Mapping[str, Any], expected: set[str], location: str
) -> None:
    observed = set(value)
    if observed != expected:
        raise ConfigurationError(
            f"{location} keys must be exactly {sorted(expected)}; "
            f"observed {sorted(observed)}"
        )


def _exact_integer(value: Any, expected: int, location: str) -> None:
    if isinstance(value, bool) or not isinstance(value, int) or value != expected:
        raise ConfigurationError(f"{location} must be integer {expected}")


def _positive_integer_value(value: Any, location: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ConfigurationError(f"{location} must be a positive integer")
    return value


def _sequence(value: Any, location: str) -> Sequence[Any]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise ConfigurationError(f"{location} must be a sequence")
    return value


def _string(value: Any, location: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ConfigurationError(f"{location} must be a non-empty string")
    return value.strip()


def _sha256(value: Any, location: str) -> str:
    digest = _string(value, location)
    if len(digest) != 64:
        raise ConfigurationError(f"{location} must be a SHA256 hex digest")
    try:
        int(digest, 16)
    except ValueError as exc:
        raise ConfigurationError(
            f"{location} must be a SHA256 hex digest"
        ) from exc
    return digest


def _strict_json(content: bytes, location: str) -> Mapping[str, Any]:
    def reject_constant(token: str) -> None:
        raise ValueError(f"non-finite JSON constant {token}")

    def reject_duplicates(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"duplicate JSON key {key!r}")
            result[key] = value
        return result

    try:
        decoded = json.loads(
            content,
            parse_constant=reject_constant,
            object_pairs_hook=reject_duplicates,
        )
    except (UnicodeDecodeError, ValueError, json.JSONDecodeError) as exc:
        raise ConfigurationError(f"{location} must be strict JSON: {exc}") from exc
    return _mapping(decoded, location)


def _canonical(value: Any) -> str:
    try:
        return json.dumps(
            value,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        )
    except (TypeError, ValueError) as exc:
        raise ConfigurationError(f"identity contains an invalid JSON value: {exc}") from exc


def _distribution(values: Sequence[float], location: str) -> dict[str, float | int]:
    parsed = [_finite(value, location) for value in values]
    if not parsed:
        raise ConfigurationError(f"{location} requires at least one sample")
    ordered = sorted(parsed)
    p95_index = max(0, math.ceil(0.95 * len(ordered)) - 1)
    try:
        calculated = {
            "mean": statistics.fmean(ordered),
            "stddev_population": statistics.pstdev(ordered),
            "minimum": ordered[0],
            "median": statistics.median(ordered),
            "p95_nearest_rank": ordered[p95_index],
            "maximum": ordered[-1],
        }
    except (OverflowError, statistics.StatisticsError) as exc:
        raise ConfigurationError(f"{location} statistics overflowed") from exc
    try:
        finite_calculated = {
            name: _finite(value, f"{location}.statistics.{name}")
            for name, value in calculated.items()
        }
    except ConfigurationError as exc:
        raise ConfigurationError(
            f"{location} statistics must remain finite"
        ) from exc
    return {"count": len(ordered), **finite_calculated}


def _exclusion(code: str, detail: str) -> dict[str, str]:
    return {"code": code, "detail": detail}


def _identifier_values(values: Sequence[str], option: str) -> tuple[str, ...]:
    parsed = tuple(_string(value, option) for value in values)
    if any(not _IDENTIFIER_PATTERN.fullmatch(value) for value in parsed):
        raise ConfigurationError(f"{option} values must be path-safe identifiers")
    if len(set(parsed)) != len(parsed):
        raise ConfigurationError(f"{option} values must be unique")
    return parsed


def _exact_number(actual: Any, expected: float, location: str) -> None:
    parsed = _finite(actual, location)
    if parsed != expected:
        raise ConfigurationError(
            f"{location} must be exactly {expected!r}, observed {parsed!r}"
        )


def _validate_configured_segments(configuration: Mapping[str, Any], location: str) -> None:
    _exact_keys(configuration, _CONFIGURATION_KEYS, location)
    _exact_integer(
        configuration.get("schema_version"), 1, f"{location}.schema_version"
    )
    if configuration.get("profile_id") != _MOTION_PROFILE_ID:
        raise ConfigurationError(
            f"{location}.profile_id must be {_MOTION_PROFILE_ID!r}"
        )
    segments = _sequence(configuration.get("segments"), f"{location}.segments")
    if len(segments) != len(_SEGMENT_SPECS):
        raise ConfigurationError(f"{location}.segments must contain exactly 6 segments")
    for index, specification in enumerate(_SEGMENT_SPECS):
        segment_id, motion, linear, angular, duration = specification
        segment = _mapping(segments[index], f"{location}.segments[{index}]")
        _exact_keys(
            segment,
            _CONFIGURED_SEGMENT_KEYS,
            f"{location}.segments[{index}]",
        )
        if segment.get("segment_id") != segment_id or segment.get("motion") != motion:
            raise ConfigurationError(
                f"{location}.segments[{index}] must be {segment_id}/{motion}"
            )
        if segment.get("tier") != "ab":
            raise ConfigurationError(f"{location}.{segment_id}.tier must be 'ab'")
        _exact_number(
            segment.get("linear_x_mps"),
            linear,
            f"{location}.{segment_id}.linear_x_mps",
        )
        _exact_number(
            segment.get("angular_z_radps"),
            angular,
            f"{location}.{segment_id}.angular_z_radps",
        )
        _exact_number(
            segment.get("duration_sec"),
            duration,
            f"{location}.{segment_id}.duration_sec",
        )


def _wheel_means(
    segment: Mapping[str, Any],
    wheel_layout: Mapping[str, str],
    location: str,
) -> tuple[float, float]:
    wheels = _mapping(segment.get("wheels"), f"{location}.wheels")
    per_wheel = _mapping(wheels.get("per_wheel"), f"{location}.wheels.per_wheel")

    def mean(key: str) -> float:
        joint_name = wheel_layout[key]
        joint = _mapping(per_wheel.get(joint_name), f"{location}.wheels.{joint_name}")
        speeds = _mapping(joint.get("speed_radps"), f"{location}.wheels.{joint_name}.speed_radps")
        return _finite(speeds.get("mean"), f"{location}.wheels.{joint_name}.speed_radps.mean")

    try:
        left = statistics.fmean(mean(key) for key in _LEFT_WHEELS)
        right = statistics.fmean(mean(key) for key in _RIGHT_WHEELS)
    except OverflowError as exc:
        raise ConfigurationError(f"{location}.wheels mean speed overflowed") from exc
    left = _finite(left, f"{location}.wheels.left_mean_speed_radps")
    right = _finite(right, f"{location}.wheels.right_mean_speed_radps")
    return left, right


def _expected_wheel_directions(
    motion: str, wheel_layout: Mapping[str, str]
) -> dict[str, str]:
    if motion == "forward":
        return {name: "positive" for name in wheel_layout.values()}
    if motion == "backward":
        return {name: "negative" for name in wheel_layout.values()}
    if motion in {"arc_left", "arc_right"}:
        return {name: "positive" for name in wheel_layout.values()}
    left, right = (
        ("negative", "positive")
        if motion == "rotate_left"
        else ("positive", "negative")
    )
    return {
        **{wheel_layout[key]: left for key in _LEFT_WHEELS},
        **{wheel_layout[key]: right for key in _RIGHT_WHEELS},
    }


def _validate_wheel_directions(
    segment: Mapping[str, Any],
    wheel_layout: Mapping[str, str],
    motion: str,
    direction_deadband_radps: float,
    location: str,
) -> None:
    wheels = _mapping(segment.get("wheels"), f"{location}.wheels")
    _exact_keys(wheels, {"all_directions_match", "per_wheel"}, f"{location}.wheels")
    aggregate_matches = wheels.get("all_directions_match")
    if not isinstance(aggregate_matches, bool):
        raise ConfigurationError(
            f"{location}.wheels.all_directions_match must be a boolean"
        )
    per_wheel = _mapping(wheels.get("per_wheel"), f"{location}.wheels.per_wheel")
    expected = _expected_wheel_directions(motion, wheel_layout)
    if set(per_wheel) != set(expected):
        raise ConfigurationError(
            f"{location}.wheels.per_wheel must exactly match configured wheels"
        )
    per_wheel_matches: list[bool] = []
    for joint_name, expected_direction in expected.items():
        wheel = _mapping(
            per_wheel.get(joint_name), f"{location}.wheels.{joint_name}"
        )
        _exact_keys(
            wheel, _WHEEL_REPORT_KEYS, f"{location}.wheels.{joint_name}"
        )
        if wheel.get("expected_direction") != expected_direction:
            raise ConfigurationError(
                f"{location}.wheels.{joint_name}.expected_direction must be "
                f"{expected_direction!r}"
            )
        direction = wheel.get("direction")
        direction_matches = wheel.get("direction_matches")
        if not isinstance(direction, str):
            raise ConfigurationError(
                f"{location}.wheels.{joint_name}.direction must be a string"
            )
        if not isinstance(direction_matches, bool):
            raise ConfigurationError(
                f"{location}.wheels.{joint_name}.direction_matches must be a boolean"
            )
        computed_direction_matches = direction == expected_direction
        if direction_matches is not computed_direction_matches:
            raise ConfigurationError(
                f"{location}.wheels.{joint_name}.direction_matches is inconsistent "
                "with direction and expected_direction"
            )
        if direction not in {expected_direction, "mixed"}:
            raise ConfigurationError(
                f"{location}.wheels.{joint_name}.direction must be "
                f"{expected_direction!r}, or 'mixed' for a pure rotation"
            )
        if direction == "mixed" and motion not in {"rotate_left", "rotate_right"}:
            raise ConfigurationError(
                f"{location}.wheels.{joint_name}.direction 'mixed' is only valid "
                "for a pure rotation"
            )
        speeds = _mapping(
            wheel.get("speed_radps"),
            f"{location}.wheels.{joint_name}.speed_radps",
        )
        mean_speed = _finite(
            speeds.get("mean"),
            f"{location}.wheels.{joint_name}.speed_radps.mean",
        )
        if (
            expected_direction == "positive" and mean_speed <= 0.0
        ) or (
            expected_direction == "negative" and mean_speed >= 0.0
        ):
            raise ConfigurationError(
                f"{location}.wheels.{joint_name}.speed_radps.mean sign must "
                f"match {expected_direction!r} direction"
            )
        minimum = _finite(
            speeds.get("minimum"),
            f"{location}.wheels.{joint_name}.speed_radps.minimum",
        )
        maximum = _finite(
            speeds.get("maximum"),
            f"{location}.wheels.{joint_name}.speed_radps.maximum",
        )
        mean_below_minimum = mean_speed < minimum and not math.isclose(
            mean_speed, minimum, rel_tol=1e-12, abs_tol=1e-12
        )
        mean_above_maximum = mean_speed > maximum and not math.isclose(
            mean_speed, maximum, rel_tol=1e-12, abs_tol=1e-12
        )
        if minimum > maximum or mean_below_minimum or mean_above_maximum:
            raise ConfigurationError(
                f"{location}.wheels.{joint_name}.speed_radps must satisfy "
                "minimum <= mean <= maximum"
            )
        if direction == "mixed":
            if (
                minimum >= -direction_deadband_radps
                or maximum <= direction_deadband_radps
            ):
                raise ConfigurationError(
                    f"{location}.wheels.{joint_name}.speed_radps mixed range must "
                    "cross both sides of the direction deadband"
                )
        elif expected_direction == "positive":
            if (
                minimum < -direction_deadband_radps
                or maximum <= direction_deadband_radps
            ):
                raise ConfigurationError(
                    f"{location}.wheels.{joint_name}.speed_radps range contradicts "
                    "the reported positive direction classification"
                )
        elif (
            minimum >= -direction_deadband_radps
            or maximum > direction_deadband_radps
        ):
            raise ConfigurationError(
                f"{location}.wheels.{joint_name}.speed_radps range contradicts "
                "the reported negative direction classification"
            )
        per_wheel_matches.append(direction_matches)
    if aggregate_matches is not all(per_wheel_matches):
        raise ConfigurationError(
            f"{location}.wheels.all_directions_match is inconsistent with "
            "per-wheel direction_matches"
        )


def _segment_metrics(
    segment: Mapping[str, Any],
    specification: tuple[str, str, float, float, float],
    wheel_layout: Mapping[str, str],
    wheel_radius_m: float,
    direction_deadband_radps: float,
    report_schema_version: int,
    location: str,
) -> tuple[dict[str, float], float | None]:
    segment_id, motion, linear, angular, duration = specification
    _exact_keys(segment, _RESULT_SEGMENT_KEYS, location)
    if segment.get("segment_id") != segment_id or segment.get("motion") != motion:
        raise ConfigurationError(f"{location} must be {segment_id}/{motion}")
    if segment.get("tier") != "ab" or segment.get("result") != "complete":
        raise ConfigurationError(f"{location} must be a complete 'ab' segment")
    for name in (
        "sample_counts",
        "timestamp_integrity",
        "reset",
        "invalid_message_counts",
    ):
        _mapping(segment.get(name), f"{location}.{name}")
    command = _mapping(segment.get("command"), f"{location}.command")
    _exact_keys(command, _COMMAND_KEYS, f"{location}.command")
    _exact_number(command.get("linear_x_mps"), linear, f"{location}.command.linear_x_mps")
    _exact_number(command.get("angular_z_radps"), angular, f"{location}.command.angular_z_radps")
    _exact_number(
        command.get("configured_duration_sec"),
        duration,
        f"{location}.command.configured_duration_sec",
    )
    observed_duration = _positive(
        command.get("observed_duration_sec"),
        f"{location}.command.observed_duration_sec",
    )
    _positive_integer_value(
        command.get("publish_count"), f"{location}.command.publish_count"
    )
    start_stamp = _positive_integer_value(
        command.get("start_stamp_ns"), f"{location}.command.start_stamp_ns"
    )
    end_stamp = _positive_integer_value(
        command.get("end_stamp_ns"), f"{location}.command.end_stamp_ns"
    )
    if end_stamp <= start_stamp:
        raise ConfigurationError(f"{location}.command end stamp must follow start")
    try:
        stamp_duration = (end_stamp - start_stamp) / 1_000_000_000
    except OverflowError as exc:
        raise ConfigurationError(
            f"{location}.command timestamp interval is too large"
        ) from exc
    _exact_number(
        observed_duration,
        stamp_duration,
        f"{location}.command.observed_duration_sec",
    )
    pose = _mapping(segment.get("pose"), f"{location}.pose")
    _exact_keys(pose, _POSE_KEYS, f"{location}.pose")
    _mapping(pose.get("start"), f"{location}.pose.start")
    _mapping(pose.get("end"), f"{location}.pose.end")
    yaw = _mapping(segment.get("yaw"), f"{location}.yaw")
    _exact_keys(yaw, _YAW_KEYS, f"{location}.yaw")
    stopping = _mapping(segment.get("stopping"), f"{location}.stopping")
    _exact_keys(stopping, _STOPPING_KEYS, f"{location}.stopping")
    if stopping.get("stopped") is not True:
        raise ConfigurationError(f"{location}.stopping.stopped must be true")
    onset = _nonnegative(
        stopping.get("stationary_onset_after_command_sec"),
        f"{location}.stopping.stationary_onset_after_command_sec",
    )
    confirmed = _nonnegative(
        stopping.get("confirmed_after_command_sec"),
        f"{location}.stopping.confirmed_after_command_sec",
    )
    if confirmed < onset:
        raise ConfigurationError(f"{location} stop confirmation precedes onset")
    actual_velocity = _mapping(
        segment.get("actual_velocity"), f"{location}.actual_velocity"
    )
    _exact_keys(
        actual_velocity,
        (
            _ACTUAL_VELOCITY_KEYS | {"steady_state_window"}
            if report_schema_version == 2
            else _ACTUAL_VELOCITY_KEYS
        ),
        f"{location}.actual_velocity",
    )
    for velocity_name in _ACTUAL_VELOCITY_KEYS:
        distribution = _mapping(
            actual_velocity.get(velocity_name),
            f"{location}.actual_velocity.{velocity_name}",
        )
        _finite(
            distribution.get("mean"),
            f"{location}.actual_velocity.{velocity_name}.mean",
        )
    steady_state_mean_yaw_rate: float | None = None
    if report_schema_version == 2:
        steady_state_window = _mapping(
            actual_velocity.get("steady_state_window"),
            f"{location}.actual_velocity.steady_state_window",
        )
        steady_location = f"{location}.actual_velocity.steady_state_window"
        _exact_keys(
            steady_state_window,
            _STEADY_STATE_WINDOW_KEYS,
            steady_location,
        )
        _exact_integer(
            steady_state_window.get("schema_version"),
            1,
            f"{steady_location}.schema_version",
        )
        if (
            steady_state_window.get("definition")
            != "final_half_of_command_interval"
        ):
            raise ConfigurationError(
                f"{steady_location}.definition must be "
                "'final_half_of_command_interval'"
            )
        steady_start_stamp = start_stamp + (end_stamp - start_stamp) // 2
        _exact_integer(
            steady_state_window.get("start_stamp_ns"),
            steady_start_stamp,
            f"{steady_location}.start_stamp_ns",
        )
        _exact_integer(
            steady_state_window.get("end_stamp_ns"),
            end_stamp,
            f"{steady_location}.end_stamp_ns",
        )
        steady_duration = (end_stamp - steady_start_stamp) / 1_000_000_000
        _exact_number(
            steady_state_window.get("observed_duration_sec"),
            steady_duration,
            f"{steady_location}.observed_duration_sec",
        )
        steady_sample_count = _positive_integer_value(
            steady_state_window.get("sample_count"),
            f"{steady_location}.sample_count",
        )
        steady_angular = _mapping(
            steady_state_window.get("angular_z_radps"),
            f"{steady_location}.angular_z_radps",
        )
        _exact_keys(
            steady_angular,
            _VELOCITY_DISTRIBUTION_KEYS,
            f"{steady_location}.angular_z_radps",
        )
        distribution_sample_count = _positive_integer_value(
            steady_angular.get("sample_count"),
            f"{steady_location}.angular_z_radps.sample_count",
        )
        if distribution_sample_count != steady_sample_count:
            raise ConfigurationError(
                f"{steady_location}.angular_z_radps.sample_count must match "
                f"{steady_location}.sample_count"
            )
        for metric_name in _VELOCITY_DISTRIBUTION_KEYS - {"sample_count"}:
            _finite(
                steady_angular.get(metric_name),
                f"{steady_location}.angular_z_radps.{metric_name}",
            )
        steady_state_mean_yaw_rate = _finite(
            steady_angular.get("mean"),
            f"{steady_location}.angular_z_radps.mean",
        )
    _validate_wheel_directions(
        segment,
        wheel_layout,
        motion,
        direction_deadband_radps,
        location,
    )
    yaw_change = _finite(yaw.get("change_rad"), f"{location}.yaw.change_rad")
    expected_yaw = _finite(
        angular * observed_duration, f"{location}.yaw.expected_change_calculated"
    )
    _exact_number(
        yaw.get("expected_change_rad"),
        expected_yaw,
        f"{location}.yaw.expected_change_rad",
    )
    _exact_number(
        yaw.get("error_rad"),
        yaw_change - expected_yaw,
        f"{location}.yaw.error_rad",
    )
    longitudinal = _finite(
        pose.get("longitudinal_displacement_m"),
        f"{location}.pose.longitudinal_displacement_m",
    )
    expected_longitudinal = _finite(
        linear * observed_duration,
        f"{location}.pose.expected_longitudinal_displacement_calculated",
    )
    _exact_number(
        pose.get("expected_longitudinal_displacement_m"),
        expected_longitudinal,
        f"{location}.pose.expected_longitudinal_displacement_m",
    )
    _exact_number(
        pose.get("longitudinal_error_m"),
        longitudinal - expected_longitudinal,
        f"{location}.pose.longitudinal_error_m",
    )
    metrics = {
        "observed_duration_sec": observed_duration,
        "trajectory_length_m": _nonnegative(
            pose.get("trajectory_length_m"),
            f"{location}.pose.trajectory_length_m",
        ),
        "longitudinal_displacement_m": longitudinal,
        "distance_error_m": longitudinal - expected_longitudinal,
        "lateral_displacement_m": _finite(
            pose.get("lateral_displacement_m"),
            f"{location}.pose.lateral_displacement_m",
        ),
        "yaw_change_rad": yaw_change,
        "yaw_error_rad": yaw_change - expected_yaw,
        "stop_onset_sec": onset,
        "stop_confirmed_sec": confirmed,
    }
    if expected_yaw != 0.0:
        if yaw_change == 0.0 or (yaw_change > 0.0) != (expected_yaw > 0.0):
            raise ConfigurationError(f"{location} measured yaw sign is invalid")
        metrics["yaw_gain"] = _finite(
            yaw_change / expected_yaw, f"{location}.yaw_gain"
        )
    if motion in {"forward", "backward"}:
        lateral_drift = _finite(
            pose.get("lateral_drift_m"),
            f"{location}.pose.lateral_drift_m",
        )
        _exact_number(
            lateral_drift,
            metrics["lateral_displacement_m"],
            f"{location}.pose.lateral_drift_m",
        )
        metrics["lateral_drift_m"] = lateral_drift
    elif motion in {"arc_left", "arc_right"}:
        if pose.get("lateral_drift_m") is not None:
            raise ConfigurationError(
                f"{location}.pose.lateral_drift_m must be null for arcs"
            )
    else:
        metrics["center_drift_m"] = _nonnegative(
            pose.get("translation_drift_m"),
            f"{location}.pose.translation_drift_m",
        )
        left_rate, right_rate = _wheel_means(segment, wheel_layout, location)
        wheel_differential = _finite(
            wheel_radius_m * (right_rate - left_rate),
            f"{location}.wheel_differential_mps",
        )
        angular_velocity = _mapping(
            actual_velocity.get("angular_z_radps"),
            f"{location}.actual_velocity.angular_z_radps",
        )
        measured_yaw_rate = _finite(
            angular_velocity.get("mean"),
            f"{location}.actual_velocity.angular_z_radps.mean",
        )
        if measured_yaw_rate == 0.0 or wheel_differential == 0.0:
            raise ConfigurationError(f"{location} effective-track inputs must be non-zero")
        if (wheel_differential > 0.0) != (measured_yaw_rate > 0.0):
            raise ConfigurationError(f"{location} wheel/yaw signs disagree")
        metrics["effective_track_m"] = _positive(
            wheel_differential / measured_yaw_rate,
            f"{location}.effective_track_m",
        )
    return metrics, steady_state_mean_yaw_rate


def _identity_locks(
    report: Mapping[str, Any],
    provenance: Mapping[str, Any],
    configuration: Mapping[str, Any],
) -> tuple[
    dict[str, Any],
    dict[str, Any],
    dict[str, Any] | None,
    dict[str, Any] | None,
    dict[str, Any],
    dict[str, Any],
]:
    robot = _mapping(provenance.get("robot"), "runtime_provenance.robot")
    environment = _mapping(
        provenance.get("environment"), "runtime_provenance.environment"
    )
    simulation = _mapping(
        provenance.get("simulation"), "runtime_provenance.simulation"
    )
    git = _mapping(provenance.get("git"), "runtime_provenance.git")
    contact = _mapping(provenance.get("contact"), "runtime_provenance.contact")
    global_lock = {
        # Lock complete validated mappings so future nested schema additions
        # cannot become silent experimental variables.
        "robot": dict(robot),
        "simulation": dict(simulation),
        "git": dict(git),
        "motion_config_file": _string(
            report.get("config_file"), "report.config_file"
        ),
        "motion_config_sha256": _sha256(
            report.get("config_sha256"), "report.config_sha256"
        ),
        "motion_configuration": dict(configuration),
    }
    # The root-layer digest is captured after physics/runtime initialization.
    # Although topology/contact author their direct opinions in SessionLayer,
    # downstream initialization can author treatment-dependent derived
    # opinions into RootLayer.  Lock that digest inside the final treatment
    # group, while the explicit cross-treatment locks below continue to guard
    # every invariant input.  This is also the published v3/v4 scope.
    locked_environment = {
        key: environment[key]
        for key in sorted(environment)
        if key != "composed_root_layer_sha256"
    }
    environment_lock: dict[str, Any] = {
        "environment": locked_environment,
    }
    topology_lock: dict[str, Any] | None = None
    topology_ab_contact_lock: dict[str, Any] | None = None
    if provenance.get("schema_version") == 5:
        collider_contract = _mapping(
            contact.get("collider_contract"),
            "runtime_provenance.contact.collider_contract",
        )
        environment_lock.update(
            {
                "wheel_collider_contract": {
                    "wheel_joint_names": collider_contract[
                        "wheel_joint_names"
                    ],
                    "wheel_expected_count": collider_contract[
                        "wheel_expected_count"
                    ],
                },
                "wheel_colliders": contact["wheel_colliders"],
            }
        )
        topology = _mapping(
            provenance.get("ground_topology"),
            "runtime_provenance.ground_topology",
        )
        environment_lock["ground_source_contract"] = {
            "source_asset_path": topology["source_asset_path"],
            "source_asset_sha256": topology["source_asset_sha256"],
            "source_colliders": topology["source_colliders"],
            "source_collider_count": topology["source_collider_count"],
            "source_collider_paths_sha256": topology[
                "source_collider_paths_sha256"
            ],
        }
        topology_specific_keys = {
            "profile_path",
            "profile_sha256",
            "profile_id",
            "environment_id",
            "operation",
            "overlay_sha256",
            "target_colliders",
            "target_collider_count",
            "target_collider_paths_sha256",
            "disabled_colliders",
            "disabled_collider_count",
            "disabled_collider_paths_sha256",
            "stage_usd_readback_verified",
        }
        topology_lock = {
            "ground_topology": {
                key: topology[key]
                for key in sorted(topology_specific_keys)
            },
            "ground_collider_contract": {
                "ground_required_prim_paths": collider_contract[
                    "ground_required_prim_paths"
                ],
                "ground_semantic_classes": collider_contract[
                    "ground_semantic_classes"
                ],
                "ground_expected_enabled_count": collider_contract[
                    "ground_expected_enabled_count"
                ],
            },
            "ground_colliders": contact["ground_colliders"],
        }
    else:
        # Historical v3/v4 reports predate an explicit topology identity.  The
        # discovered ground contract therefore remains part of their
        # environment lock and cannot be varied inside one environment.
        environment_lock.update(
            {
                "collider_contract": contact["collider_contract"],
                "wheel_colliders": contact["wheel_colliders"],
                "ground_colliders": contact["ground_colliders"],
            }
        )
    profile_lock = {
        "profile_path": contact["profile_path"],
        "profile_sha256": contact["profile_sha256"],
        "profile_id": contact["profile_id"],
        "profile_mode": contact["profile_mode"],
        "explicit_materials": contact["explicit_materials"],
        "thresholds_authored": contact["thresholds_authored"],
    }
    if provenance.get("schema_version") == 5:
        # A topology A/B may change only the topology overlay and the exact
        # ground collider/binding path set.  Scene thresholds, wheel-side
        # evidence, and material values must remain identical for the same
        # environment/contact profile across every topology condition.
        topology_ab_contact_lock = {
            "profile": profile_lock,
            "scene": contact["scene"],
            "wheel_bindings": contact["wheel_bindings"],
            "wheel_material": contact["wheel_material"],
            "ground_material": contact["ground_material"],
            "stage_usd_readback_verified": contact[
                "stage_usd_readback_verified"
            ],
        }
    # The anonymous identifier embeds a process-specific address and is not an
    # identity.  Its canonical content hash is stable within the final
    # environment/topology/profile group and is deliberately locked along with
    # collider/binding evidence.
    contact_lock = {
        key: contact[key]
        for key in sorted(contact)
        if key != "overlay_identifier"
    }
    contact_lock["composed_root_layer_sha256"] = environment[
        "composed_root_layer_sha256"
    ]
    return (
        global_lock,
        environment_lock,
        topology_lock,
        topology_ab_contact_lock,
        profile_lock,
        contact_lock,
    )


def _validated_record(
    report: Mapping[str, Any],
    source_path: Path,
    source_sha256: str,
    canonical_sha256: str,
    wheel_radius_m: float,
) -> InputRecord:
    location = f"report {source_path}"
    _exact_keys(report, _REPORT_KEYS, location)
    report_schema_version = report.get("schema_version")
    if (
        isinstance(report_schema_version, bool)
        or not isinstance(report_schema_version, int)
        or report_schema_version not in {1, 2}
    ):
        raise ConfigurationError(
            f"{location} schema_version must be integer 1 or 2"
        )
    if report.get("result") != "success":
        raise ConfigurationError(f"{location}.result must be 'success'")
    if report.get("failure_reason") != "" or report.get("failed_segments") != []:
        raise ConfigurationError(f"{location} successful report has failure evidence")
    if report.get("diagnostic") != "four_wheel_chassis_motion_baseline":
        raise ConfigurationError(f"{location}.diagnostic is not a motion baseline")
    if report.get("profile_id") != _MOTION_PROFILE_ID:
        raise ConfigurationError(
            f"{location}.profile_id must be {_MOTION_PROFILE_ID!r}"
        )
    provenance = _mapping(report.get("runtime_provenance"), f"{location}.runtime_provenance")
    provenance_schema = provenance.get("schema_version")
    if (
        isinstance(provenance_schema, bool)
        or not isinstance(provenance_schema, int)
        or provenance_schema not in {3, 4, 5}
    ):
        raise ConfigurationError(
            f"{location}.runtime_provenance.schema_version must be integer "
            "3, 4, or 5"
        )
    _exact_keys(
        provenance,
        (
            _RUNTIME_PROVENANCE_V5_KEYS
            if provenance_schema == 5
            else _RUNTIME_PROVENANCE_KEYS
        ),
        f"{location}.runtime_provenance",
    )
    try:
        validate_runtime_provenance(provenance)
    except ReportValidationError as exc:
        raise ConfigurationError(f"{location} runtime provenance: {exc}") from exc
    if provenance_schema == 3:
        if wheel_radius_m != CANONICAL_WHEEL_RADIUS_M:
            raise ConfigurationError(
                f"{location} historical runtime provenance schema 3 requires "
                f"the canonical Jackal wheel radius "
                f"{CANONICAL_WHEEL_RADIUS_M}; selected {wheel_radius_m}"
            )
    else:
        robot = _mapping(
            provenance.get("robot"),
            f"{location}.runtime_provenance.robot",
        )
        kinematics = _mapping(
            robot.get("kinematics"),
            f"{location}.runtime_provenance.robot.kinematics",
        )
        provenance_wheel_radius = _positive(
            kinematics.get("wheel_radius_m"),
            f"{location}.runtime_provenance.robot.kinematics."
            "wheel_radius_m",
        )
        if provenance_wheel_radius != wheel_radius_m:
            raise ConfigurationError(
                f"{location} runtime provenance wheel_radius_m does not "
                f"match the selected robot: provenance="
                f"{provenance_wheel_radius}, selected={wheel_radius_m}"
            )
    git = _mapping(provenance.get("git"), f"{location}.runtime_provenance.git")
    if git.get("dirty") is not False:
        raise ConfigurationError(f"{location} Git worktree must be clean")
    environment = _mapping(
        provenance.get("environment"), f"{location}.runtime_provenance.environment"
    )
    environment_id = _string(
        environment.get("id"), f"{location}.runtime_provenance.environment.id"
    )
    if report.get("environment_id") != environment_id:
        raise ConfigurationError(f"{location} environment label/provenance mismatch")
    simulation = _mapping(
        provenance.get("simulation"), f"{location}.runtime_provenance.simulation"
    )
    odometry_mode = _string(
        simulation.get("odometry_mode"),
        f"{location}.runtime_provenance.simulation.odometry_mode",
    )
    if report.get("odometry_mode") != odometry_mode:
        raise ConfigurationError(f"{location} odometry label/provenance mismatch")
    configuration = _mapping(report.get("configuration"), f"{location}.configuration")
    _validate_configured_segments(configuration, f"{location}.configuration")
    wheels = _mapping(configuration.get("wheels"), f"{location}.configuration.wheels")
    if set(wheels) != set(_WHEEL_KEYS):
        raise ConfigurationError(f"{location}.configuration.wheels must name four roles")
    wheel_layout = {
        key: _string(wheels[key], f"{location}.configuration.wheels.{key}")
        for key in _WHEEL_KEYS
    }
    if len(set(wheel_layout.values())) != 4:
        raise ConfigurationError(f"{location}.configuration.wheels must be unique")
    stop_configuration = _mapping(
        configuration.get("stop"), f"{location}.configuration.stop"
    )
    direction_deadband_radps = _nonnegative(
        stop_configuration.get("wheel_velocity_threshold_radps"),
        f"{location}.configuration.stop.wheel_velocity_threshold_radps",
    )
    physical_stop_contract = None
    if provenance_schema == 5:
        physical_stop_contract = {
            "stable_duration_sec": _nonnegative(
                stop_configuration.get("stable_duration_sec"),
                f"{location}.configuration.stop.stable_duration_sec",
            ),
            "linear_velocity_threshold_mps": _nonnegative(
                stop_configuration.get("linear_velocity_threshold_mps"),
                f"{location}.configuration.stop."
                "linear_velocity_threshold_mps",
            ),
            "angular_velocity_threshold_radps": _nonnegative(
                stop_configuration.get("angular_velocity_threshold_radps"),
                f"{location}.configuration.stop."
                "angular_velocity_threshold_radps",
            ),
            "wheel_velocity_threshold_radps": direction_deadband_radps,
        }
    contact = _mapping(
        provenance.get("contact"), f"{location}.runtime_provenance.contact"
    )
    collider_contract = _mapping(
        contact.get("collider_contract"),
        f"{location}.runtime_provenance.contact.collider_contract",
    )
    contact_wheel_joints = {
        _string(
            name,
            f"{location}.runtime_provenance.contact.collider_contract."
            "wheel_joint_names",
        )
        for name in _sequence(
            collider_contract.get("wheel_joint_names"),
            f"{location}.runtime_provenance.contact.collider_contract."
            "wheel_joint_names",
        )
    }
    if contact_wheel_joints != set(wheel_layout.values()):
        raise ConfigurationError(
            f"{location} motion/configured wheels and contact collider "
            "wheel_joint_names must match"
        )
    raw_segments = _sequence(report.get("segments"), f"{location}.segments")
    if len(raw_segments) != len(_SEGMENT_SPECS):
        raise ConfigurationError(f"{location}.segments must contain exactly 6 segments")
    metrics: dict[str, dict[str, float]] = {}
    steady_state_mean_yaw_rates: dict[str, float] = {}
    for index, specification in enumerate(_SEGMENT_SPECS):
        segment_id = specification[0]
        segment_metrics, steady_state_mean_yaw_rate = _segment_metrics(
            _mapping(raw_segments[index], f"{location}.segments[{index}]"),
            specification,
            wheel_layout,
            wheel_radius_m,
            direction_deadband_radps,
            report_schema_version,
            f"{location}.segments[{index}]({segment_id})",
        )
        metrics[segment_id] = segment_metrics
        if steady_state_mean_yaw_rate is not None:
            steady_state_mean_yaw_rates[segment_id] = (
                steady_state_mean_yaw_rate
            )
    physical_yaw_rate_metrics = None
    if provenance_schema == 5 and report_schema_version == 2:
        physical_yaw_rate_metrics = {}
        for segment_id, motion, _linear, angular, _duration in _SEGMENT_SPECS:
            if motion not in {"rotate_left", "rotate_right"}:
                continue
            mean_yaw_rate = steady_state_mean_yaw_rates[segment_id]
            physical_yaw_rate_metrics[segment_id] = {
                "commanded_yaw_rate_radps": angular,
                "steady_state_mean_yaw_rate_radps": mean_yaw_rate,
                "absolute_error_fraction": abs(mean_yaw_rate - angular)
                / abs(angular),
            }
    (
        global_lock,
        environment_lock,
        topology_lock,
        topology_ab_contact_lock,
        profile_lock,
        contact_lock,
    ) = _identity_locks(report, provenance, configuration)
    ground_topology_id = None
    if provenance_schema == 5:
        topology = _mapping(
            provenance.get("ground_topology"),
            f"{location}.runtime_provenance.ground_topology",
        )
        ground_topology_id = _string(
            topology.get("profile_id"),
            f"{location}.runtime_provenance.ground_topology.profile_id",
        )
        shipped_environment = _SHIPPED_TOPOLOGY_ENVIRONMENTS.get(
            ground_topology_id
        )
        if (
            shipped_environment is not None
            and environment_id != shipped_environment
        ):
            raise ConfigurationError(
                f"{location} shipped ground topology/environment pair is "
                f"invalid: {environment_id}::{ground_topology_id}; expected "
                f"{shipped_environment}::{ground_topology_id}"
            )
    contact_profile_id = _string(
        contact.get("profile_id"),
        f"{location}.runtime_provenance.contact.profile_id",
    )
    return InputRecord(
        path=str(source_path),
        sha256=source_sha256,
        canonical_sha256=canonical_sha256,
        report_schema_version=report_schema_version,
        runtime_provenance_schema=provenance_schema,
        environment_id=environment_id,
        odometry_mode=odometry_mode,
        ground_topology_id=ground_topology_id,
        contact_profile_id=contact_profile_id,
        global_lock=global_lock,
        environment_lock=environment_lock,
        topology_lock=topology_lock,
        topology_ab_contact_lock=topology_ab_contact_lock,
        profile_lock=profile_lock,
        contact_lock=contact_lock,
        physical_stop_contract=physical_stop_contract,
        physical_yaw_rate_metrics=physical_yaw_rate_metrics,
        segments=metrics,
    )


def _reason_for_invalid(exc: ConfigurationError) -> dict[str, str]:
    detail = str(exc)
    if "Git worktree must be clean" in detail:
        return _exclusion("git_dirty", detail)
    if ".profile_id must" in detail:
        return _exclusion("invalid_motion_profile", detail)
    if (
        "runtime provenance" in detail
        or "ground topology/environment pair" in detail
    ):
        return _exclusion("invalid_runtime_provenance", detail)
    return _exclusion("invalid_motion_protocol", detail)


def _group_summary(records: Sequence[InputRecord]) -> dict[str, Any]:
    first = records[0]
    by_segment: dict[str, dict[str, Any]] = {}
    for segment_id, *_ in _SEGMENT_SPECS:
        metric_names = sorted(records[0].segments[segment_id])
        by_segment[segment_id] = {
            name: _distribution(
                [record.segments[segment_id][name] for record in records],
                f"{segment_id}.{name}",
            )
            for name in metric_names
        }

    def paired(left: str, right: str, metric: str) -> list[float]:
        return [
            record.segments[left][metric] - record.segments[right][metric]
            for record in records
        ]

    rotation_gain_delta = paired("rotate_left_360", "rotate_right_360", "yaw_gain")
    rotation_center_delta = paired(
        "rotate_left_360", "rotate_right_360", "center_drift_m"
    )
    rotation_track_delta = paired(
        "rotate_left_360", "rotate_right_360", "effective_track_m"
    )
    straight_lateral_delta = [
        abs(record.segments["forward_3m"]["lateral_drift_m"])
        - abs(record.segments["backward_2m"]["lateral_drift_m"])
        for record in records
    ]
    arc_gain_delta = paired("arc_left_5s", "arc_right_5s", "yaw_gain")
    arc_lateral_delta = [
        abs(record.segments["arc_left_5s"]["lateral_displacement_m"])
        - abs(record.segments["arc_right_5s"]["lateral_displacement_m"])
        for record in records
    ]
    effective_left = [
        record.segments["rotate_left_360"]["effective_track_m"]
        for record in records
    ]
    effective_right = [
        record.segments["rotate_right_360"]["effective_track_m"]
        for record in records
    ]
    summary = {
        "environment_id": first.environment_id,
        "contact_profile_id": first.contact_profile_id,
        "contact_contract": first.contact_lock,
        "repeat_count": len(records),
        "input_reports": [
            {
                "path": record.path,
                "sha256": record.sha256,
                "canonical_sha256": record.canonical_sha256,
                "report_schema_version": record.report_schema_version,
            }
            for record in records
        ],
        "segments": by_segment,
        "stop_latency": {
            "onset_sec": _distribution(
                [
                    record.segments[segment_id]["stop_onset_sec"]
                    for record in records
                    for segment_id, *_ in _SEGMENT_SPECS
                ],
                "stop_latency.onset_sec",
            ),
            "confirmed_sec": _distribution(
                [
                    record.segments[segment_id]["stop_confirmed_sec"]
                    for record in records
                    for segment_id, *_ in _SEGMENT_SPECS
                ],
                "stop_latency.confirmed_sec",
            ),
        },
        "rotation_symmetry": {
            "yaw_gain_signed_difference": _distribution(
                rotation_gain_delta, "rotation.yaw_gain_signed_difference"
            ),
            "yaw_gain_absolute_difference": _distribution(
                [abs(value) for value in rotation_gain_delta],
                "rotation.yaw_gain_absolute_difference",
            ),
            "center_drift_signed_difference_m": _distribution(
                rotation_center_delta, "rotation.center_drift_signed_difference_m"
            ),
            "effective_track_signed_difference_m": _distribution(
                rotation_track_delta, "rotation.effective_track_signed_difference_m"
            ),
        },
        "straight_symmetry": {
            "absolute_lateral_drift_difference_m": _distribution(
                straight_lateral_delta,
                "straight.absolute_lateral_drift_difference_m",
            )
        },
        "arc_symmetry": {
            "yaw_gain_signed_difference": _distribution(
                arc_gain_delta, "arc.yaw_gain_signed_difference"
            ),
            "absolute_lateral_displacement_difference_m": _distribution(
                arc_lateral_delta,
                "arc.absolute_lateral_displacement_difference_m",
            ),
        },
        "effective_track_m": {
            "overall": _distribution(
                [*effective_left, *effective_right], "effective_track.overall"
            ),
            "left": _distribution(effective_left, "effective_track.left"),
            "right": _distribution(effective_right, "effective_track.right"),
        },
    }
    if first.ground_topology_id is not None:
        summary["runtime_provenance_schema"] = (
            first.runtime_provenance_schema
        )
        summary["odometry_mode"] = first.odometry_mode
        summary["ground_topology_id"] = first.ground_topology_id
        summary["ground_topology_contract"] = first.topology_lock
    return summary


def _physical_repeat_result(
    record: InputRecord,
    repeat_index: int,
) -> dict[str, Any]:
    """Evaluate one schema-v5 report against every plan 8.7 hard gate."""

    stop = record.physical_stop_contract
    yaw_rate_metrics = record.physical_yaw_rate_metrics
    if stop is None or yaw_rate_metrics is None:
        raise ConfigurationError(
            f"schema-5 report {record.path} lacks a physical acceptance contract"
        )
    thresholds = _PHYSICAL_ACCEPTANCE_THRESHOLDS
    checks: dict[str, dict[str, Any]] = {}

    def maximum(check_id: str, observed: float, limit: float) -> None:
        checks[check_id] = {
            "observed": observed,
            "maximum": limit,
            "passed": _physical_maximum_passed(observed, limit),
        }

    def minimum(check_id: str, observed: float, limit: float) -> None:
        checks[check_id] = {
            "observed": observed,
            "minimum": limit,
            "passed": _physical_minimum_passed(observed, limit),
        }

    maximum(
        "forward_abs_lateral_drift_m",
        abs(record.segments["forward_3m"]["lateral_drift_m"]),
        thresholds["forward_abs_lateral_drift_max_m"],
    )
    maximum(
        "backward_abs_lateral_drift_m",
        abs(record.segments["backward_2m"]["lateral_drift_m"]),
        thresholds["backward_abs_lateral_drift_max_m"],
    )
    left_center = record.segments["rotate_left_360"]["center_drift_m"]
    right_center = record.segments["rotate_right_360"]["center_drift_m"]
    maximum(
        "rotate_left_center_drift_m",
        left_center,
        thresholds["rotation_center_drift_max_m"],
    )
    maximum(
        "rotate_right_center_drift_m",
        right_center,
        thresholds["rotation_center_drift_max_m"],
    )
    symmetry_denominator = max(left_center, right_center)
    symmetry_ratio = (
        0.0
        if symmetry_denominator == 0.0
        else abs(left_center - right_center) / symmetry_denominator
    )
    maximum(
        "rotation_center_drift_asymmetry_ratio",
        symmetry_ratio,
        thresholds["rotation_center_drift_asymmetry_ratio_max"],
    )
    for side, segment_id in (
        ("left", "rotate_left_360"),
        ("right", "rotate_right_360"),
    ):
        yaw_rate = yaw_rate_metrics[segment_id]
        check_id = f"rotate_{side}_mean_yaw_rate_absolute_error_fraction"
        maximum(
            check_id,
            yaw_rate["absolute_error_fraction"],
            thresholds[
                "rotation_mean_yaw_rate_absolute_error_fraction_max"
            ],
        )
        checks[check_id].update(
            {
                "commanded_yaw_rate_radps": yaw_rate[
                    "commanded_yaw_rate_radps"
                ],
                "steady_state_mean_yaw_rate_radps": yaw_rate[
                    "steady_state_mean_yaw_rate_radps"
                ],
                "steady_state_measurement_basis": (
                    _PHYSICAL_STEADY_STATE_LEAF_BASIS
                ),
            }
        )
    minimum(
        "stop_config.stable_duration_sec",
        stop["stable_duration_sec"],
        thresholds["stop_stable_duration_min_sec"],
    )
    maximum(
        "stop_config.linear_velocity_threshold_mps",
        stop["linear_velocity_threshold_mps"],
        thresholds["stop_linear_velocity_threshold_max_mps"],
    )
    maximum(
        "stop_config.angular_velocity_threshold_radps",
        stop["angular_velocity_threshold_radps"],
        thresholds["stop_angular_velocity_threshold_max_radps"],
    )
    maximum(
        "stop_config.wheel_velocity_threshold_radps",
        stop["wheel_velocity_threshold_radps"],
        thresholds["stop_wheel_velocity_threshold_max_radps"],
    )
    for segment_id, *_ in _SEGMENT_SPECS:
        onset = record.segments[segment_id]["stop_onset_sec"]
        confirmed = record.segments[segment_id]["stop_confirmed_sec"]
        observed_duration = confirmed - onset
        check_id = f"stop_window.{segment_id}"
        checks[check_id] = {
            # Reaching this evaluator proves _segment_metrics already required
            # the producer's stopped flag to be exactly true.
            "stopped": True,
            "stationary_onset_sec": onset,
            "confirmed_sec": confirmed,
            "observed_stable_duration_sec": observed_duration,
            "required_stable_duration_sec": stop["stable_duration_sec"],
            "passed": _physical_minimum_passed(
                observed_duration,
                stop["stable_duration_sec"],
            ),
        }
    checks["wheel_direction_contract"] = {
        "validated_segment_count": len(_SEGMENT_SPECS),
        "validated_by": "strict_motion_report_validator",
        "passed": True,
    }
    failed_checks = sorted(
        check_id
        for check_id, check in checks.items()
        if check["passed"] is not True
    )
    return {
        "repeat_index": repeat_index,
        "report_path": record.path,
        "report_sha256": record.sha256,
        "canonical_sha256": record.canonical_sha256,
        "passed": not failed_checks,
        "checks": checks,
        "failed_checks": failed_checks,
    }


def _physical_acceptance(
    grouped: Mapping[tuple[str, ...], Sequence[InputRecord]],
) -> dict[str, Any]:
    """Build a non-ranking, every-repeat schema-v1 plan 8.7 verdict."""

    applicability = dict(_PHYSICAL_ACCEPTANCE_APPLICABILITY)
    group_results: dict[str, dict[str, Any]] = {}
    applicable_groups: list[str] = []
    not_applicable_groups: list[str] = []
    passing_groups: list[str] = []
    failed_groups: list[str] = []
    for group_key, records in sorted(grouped.items()):
        group_id = "::".join(group_key)
        first = records[0]
        not_applicable_reasons = []
        if any(record.report_schema_version != 2 for record in records):
            not_applicable_reasons.append("motion_report_schema_not_2")
        if first.runtime_provenance_schema != 5:
            not_applicable_reasons.append(
                "runtime_provenance_schema_not_5"
            )
        if first.environment_id != "SimplePlane":
            not_applicable_reasons.append("environment_not_SimplePlane")
        if first.ground_topology_id != "simple_plane_only1_v1":
            not_applicable_reasons.append(
                "ground_topology_not_simple_plane_only1_v1"
            )
        if first.odometry_mode != "ideal":
            not_applicable_reasons.append("odometry_mode_not_ideal")
        if len(records) < 3:
            not_applicable_reasons.append(
                "fewer_than_3_unique_repeats"
            )
        if not_applicable_reasons:
            not_applicable_groups.append(group_id)
            group_results[group_id] = {
                "applicable": False,
                "passed": None,
                "not_applicable_reasons": not_applicable_reasons,
                "repeat_count": len(records),
                "checks": {},
                "failed_checks": [],
                "repeat_results": [],
            }
            continue
        applicable_groups.append(group_id)
        repeat_results = [
            _physical_repeat_result(record, repeat_index)
            for repeat_index, record in enumerate(
                sorted(records, key=lambda item: item.path),
                start=1,
            )
        ]
        check_ids = sorted(repeat_results[0]["checks"])
        checks = {
            check_id: {
                "passed_repeats": sum(
                    result["checks"][check_id]["passed"] is True
                    for result in repeat_results
                ),
                "failed_repeats": sum(
                    result["checks"][check_id]["passed"] is not True
                    for result in repeat_results
                ),
                "all_repeats_passed": all(
                    result["checks"][check_id]["passed"] is True
                    for result in repeat_results
                ),
            }
            for check_id in check_ids
        }
        failed_checks = sorted(
            check_id
            for check_id, check in checks.items()
            if check["all_repeats_passed"] is not True
        )
        passed = not failed_checks
        group_results[group_id] = {
            "applicable": True,
            "passed": passed,
            "not_applicable_reasons": [],
            "repeat_count": len(repeat_results),
            "checks": checks,
            "failed_checks": failed_checks,
            "repeat_results": repeat_results,
        }
        (passing_groups if passed else failed_groups).append(group_id)
    return {
        "schema_version": 1,
        "policy_id": _PHYSICAL_ACCEPTANCE_POLICY_ID,
        "evaluation_basis": "every_repeat",
        "ranking_policy": "none; pass/fail only",
        "applicability": applicability,
        "steady_state_measurement_basis": (
            _PHYSICAL_STEADY_STATE_MEASUREMENT_BASIS
        ),
        "thresholds": dict(_PHYSICAL_ACCEPTANCE_THRESHOLDS),
        "groups": group_results,
        "applicable_groups": applicable_groups,
        "not_applicable_groups": not_applicable_groups,
        "passing_groups": passing_groups,
        "failed_groups": failed_groups,
        "all_applicable_groups_passed": (
            None if not applicable_groups else not failed_groups
        ),
    }


def _validated_physical_repeat_check_outcomes(
    checks: Mapping[str, Any],
    location: str,
) -> dict[str, bool]:
    """Recompute every physical leaf from its immutable policy semantics."""

    for raw_check_id in checks:
        check_id = _string(raw_check_id, f"{location} check id")
        if check_id != raw_check_id:
            raise ConfigurationError(
                f"{location} check ids must not contain surrounding whitespace"
            )
    _exact_keys(checks, set(_PHYSICAL_ACCEPTANCE_CHECK_IDS), location)
    outcomes: dict[str, bool] = {}

    def exact_passed(
        check_id: str,
        check: Mapping[str, Any],
        expected: bool,
        check_location: str,
    ) -> None:
        actual = check.get("passed")
        if not isinstance(actual, bool):
            raise ConfigurationError(
                f"{check_location}.passed must be a boolean"
            )
        if actual is not expected:
            raise ConfigurationError(
                f"{check_location}.passed contradicts its observed value and limit"
            )
        outcomes[check_id] = actual

    for check_id, threshold_id in _PHYSICAL_MAXIMUM_CHECK_THRESHOLDS.items():
        check_location = f"{location}.{check_id}"
        check = _mapping(checks.get(check_id), check_location)
        _exact_keys(check, {"observed", "maximum", "passed"}, check_location)
        observed = _nonnegative(
            check.get("observed"), f"{check_location}.observed"
        )
        maximum = _nonnegative(
            check.get("maximum"), f"{check_location}.maximum"
        )
        _exact_number(
            maximum,
            _PHYSICAL_ACCEPTANCE_THRESHOLDS[threshold_id],
            f"{check_location}.maximum",
        )
        exact_passed(
            check_id,
            check,
            _physical_maximum_passed(observed, maximum),
            check_location,
        )

    for check_id, threshold_id in _PHYSICAL_MINIMUM_CHECK_THRESHOLDS.items():
        check_location = f"{location}.{check_id}"
        check = _mapping(checks.get(check_id), check_location)
        _exact_keys(check, {"observed", "minimum", "passed"}, check_location)
        observed = _nonnegative(
            check.get("observed"), f"{check_location}.observed"
        )
        minimum = _nonnegative(
            check.get("minimum"), f"{check_location}.minimum"
        )
        _exact_number(
            minimum,
            _PHYSICAL_ACCEPTANCE_THRESHOLDS[threshold_id],
            f"{check_location}.minimum",
        )
        exact_passed(
            check_id,
            check,
            _physical_minimum_passed(observed, minimum),
            check_location,
        )

    yaw_limit = _PHYSICAL_ACCEPTANCE_THRESHOLDS[
        "rotation_mean_yaw_rate_absolute_error_fraction_max"
    ]
    for check_id, segment_id in _PHYSICAL_YAW_RATE_CHECK_SEGMENTS.items():
        check_location = f"{location}.{check_id}"
        check = _mapping(checks.get(check_id), check_location)
        _exact_keys(
            check,
            {
                "observed",
                "maximum",
                "passed",
                "commanded_yaw_rate_radps",
                "steady_state_mean_yaw_rate_radps",
                "steady_state_measurement_basis",
            },
            check_location,
        )
        command = _finite(
            check.get("commanded_yaw_rate_radps"),
            f"{check_location}.commanded_yaw_rate_radps",
        )
        expected_command = _SEGMENT_BY_ID[segment_id][3]
        _exact_number(
            command,
            expected_command,
            f"{check_location}.commanded_yaw_rate_radps",
        )
        steady_mean = _finite(
            check.get("steady_state_mean_yaw_rate_radps"),
            f"{check_location}.steady_state_mean_yaw_rate_radps",
        )
        observed = _nonnegative(
            check.get("observed"), f"{check_location}.observed"
        )
        recomputed_error = abs(steady_mean - command) / abs(command)
        _exact_number(
            observed,
            recomputed_error,
            f"{check_location}.observed",
        )
        maximum = _nonnegative(
            check.get("maximum"), f"{check_location}.maximum"
        )
        _exact_number(maximum, yaw_limit, f"{check_location}.maximum")
        if (
            check.get("steady_state_measurement_basis")
            != _PHYSICAL_STEADY_STATE_LEAF_BASIS
        ):
            raise ConfigurationError(
                f"{check_location}.steady_state_measurement_basis must be "
                f"{_PHYSICAL_STEADY_STATE_LEAF_BASIS!r}"
            )
        exact_passed(
            check_id,
            check,
            _physical_maximum_passed(observed, maximum),
            check_location,
        )

    stable_check = _mapping(
        checks.get("stop_config.stable_duration_sec"),
        f"{location}.stop_config.stable_duration_sec",
    )
    stable_duration = _nonnegative(
        stable_check.get("observed"),
        f"{location}.stop_config.stable_duration_sec.observed",
    )
    for check_id in _PHYSICAL_STOP_WINDOW_CHECK_SEGMENTS:
        check_location = f"{location}.{check_id}"
        check = _mapping(checks.get(check_id), check_location)
        _exact_keys(
            check,
            {
                "stopped",
                "stationary_onset_sec",
                "confirmed_sec",
                "observed_stable_duration_sec",
                "required_stable_duration_sec",
                "passed",
            },
            check_location,
        )
        if check.get("stopped") is not True:
            raise ConfigurationError(f"{check_location}.stopped must be true")
        onset = _nonnegative(
            check.get("stationary_onset_sec"),
            f"{check_location}.stationary_onset_sec",
        )
        confirmed = _nonnegative(
            check.get("confirmed_sec"),
            f"{check_location}.confirmed_sec",
        )
        if confirmed < onset:
            raise ConfigurationError(
                f"{check_location}.confirmed_sec must not precede onset"
            )
        observed_duration = _nonnegative(
            check.get("observed_stable_duration_sec"),
            f"{check_location}.observed_stable_duration_sec",
        )
        _exact_number(
            observed_duration,
            confirmed - onset,
            f"{check_location}.observed_stable_duration_sec",
        )
        required_duration = _nonnegative(
            check.get("required_stable_duration_sec"),
            f"{check_location}.required_stable_duration_sec",
        )
        _exact_number(
            required_duration,
            stable_duration,
            f"{check_location}.required_stable_duration_sec",
        )
        exact_passed(
            check_id,
            check,
            _physical_minimum_passed(
                observed_duration,
                required_duration,
            ),
            check_location,
        )

    wheel_location = f"{location}.wheel_direction_contract"
    wheel_check = _mapping(
        checks.get("wheel_direction_contract"), wheel_location
    )
    _exact_keys(
        wheel_check,
        {"validated_segment_count", "validated_by", "passed"},
        wheel_location,
    )
    _exact_integer(
        wheel_check.get("validated_segment_count"),
        len(_SEGMENT_SPECS),
        f"{wheel_location}.validated_segment_count",
    )
    if wheel_check.get("validated_by") != "strict_motion_report_validator":
        raise ConfigurationError(
            f"{wheel_location}.validated_by must be "
            "'strict_motion_report_validator'"
        )
    if wheel_check.get("passed") is not True:
        raise ConfigurationError(f"{wheel_location}.passed must be true")
    outcomes["wheel_direction_contract"] = True
    return outcomes


def validate_physical_acceptance_accounting(
    analysis: Mapping[str, Any],
    expected_repeats: int,
) -> None:
    """Fail closed when a schema-v1 physical verdict loses repeat evidence."""

    if (
        isinstance(expected_repeats, bool)
        or not isinstance(expected_repeats, int)
        or expected_repeats <= 0
    ):
        raise ConfigurationError("expected_repeats must be a positive integer")
    analysis_mapping = _mapping(analysis, "analysis")
    _exact_integer(
        analysis_mapping.get("schema_version"),
        3,
        "analysis.schema_version",
    )
    selection_policy = _mapping(
        analysis_mapping.get("selection_policy"),
        "analysis.selection_policy",
    )
    selection_runtime_provenance_schema = selection_policy.get(
        "required_runtime_provenance_schema"
    )
    _exact_integer(
        selection_runtime_provenance_schema,
        5,
        "analysis.selection_policy.required_runtime_provenance_schema",
    )
    raw_expected_profiles = _sequence(
        selection_policy.get("expected_profiles"),
        "analysis.selection_policy.expected_profiles",
    )
    selected_profiles = [
        _string(
            profile_id,
            f"analysis.selection_policy.expected_profiles[{index}]",
        )
        for index, profile_id in enumerate(raw_expected_profiles)
    ]
    if len(selected_profiles) != len(set(selected_profiles)):
        raise ConfigurationError(
            "analysis.selection_policy.expected_profiles must be unique"
        )
    locked_inputs = _mapping(
        analysis_mapping.get("locked_inputs"),
        "analysis.locked_inputs",
    )
    locked_simulation = _mapping(
        locked_inputs.get("simulation"),
        "analysis.locked_inputs.simulation",
    )
    locked_odometry_mode = _string(
        locked_simulation.get("odometry_mode"),
        "analysis.locked_inputs.simulation.odometry_mode",
    )
    analysis_groups = _mapping(analysis_mapping.get("groups"), "analysis.groups")
    acceptance = _mapping(
        analysis_mapping.get("physical_acceptance"),
        "analysis.physical_acceptance",
    )
    _exact_keys(
        acceptance,
        {
            "schema_version",
            "policy_id",
            "evaluation_basis",
            "ranking_policy",
            "applicability",
            "steady_state_measurement_basis",
            "thresholds",
            "groups",
            "applicable_groups",
            "not_applicable_groups",
            "passing_groups",
            "failed_groups",
            "all_applicable_groups_passed",
        },
        "analysis.physical_acceptance",
    )
    _exact_integer(
        acceptance.get("schema_version"),
        1,
        "analysis.physical_acceptance.schema_version",
    )
    for field, expected in (
        ("policy_id", _PHYSICAL_ACCEPTANCE_POLICY_ID),
        ("evaluation_basis", "every_repeat"),
        ("ranking_policy", "none; pass/fail only"),
        (
            "steady_state_measurement_basis",
            _PHYSICAL_STEADY_STATE_MEASUREMENT_BASIS,
        ),
    ):
        if acceptance.get(field) != expected:
            raise ConfigurationError(
                f"analysis.physical_acceptance.{field} must be {expected!r}"
            )
    applicability_contract = _mapping(
        acceptance.get("applicability"),
        "analysis.physical_acceptance.applicability",
    )
    _exact_keys(
        applicability_contract,
        set(_PHYSICAL_ACCEPTANCE_APPLICABILITY),
        "analysis.physical_acceptance.applicability",
    )
    if _canonical(applicability_contract) != _canonical(
        _PHYSICAL_ACCEPTANCE_APPLICABILITY
    ):
        raise ConfigurationError(
            "analysis.physical_acceptance.applicability does not match the "
            "plan 8.7 policy"
        )
    thresholds_contract = _mapping(
        acceptance.get("thresholds"),
        "analysis.physical_acceptance.thresholds",
    )
    _exact_keys(
        thresholds_contract,
        set(_PHYSICAL_ACCEPTANCE_THRESHOLDS),
        "analysis.physical_acceptance.thresholds",
    )
    for threshold_id, expected in _PHYSICAL_ACCEPTANCE_THRESHOLDS.items():
        _exact_number(
            thresholds_contract.get(threshold_id),
            expected,
            f"analysis.physical_acceptance.thresholds.{threshold_id}",
        )
    acceptance_groups = _mapping(
        acceptance.get("groups"),
        "analysis.physical_acceptance.groups",
    )

    def group_names(groups: Mapping[Any, Any], location: str) -> set[str]:
        names: set[str] = set()
        for name in groups:
            parsed = _string(name, f"{location} group id")
            if parsed != name:
                raise ConfigurationError(
                    f"{location} group ids must not contain surrounding whitespace"
                )
            names.add(parsed)
        return names

    analysis_group_ids = group_names(analysis_groups, "analysis.groups")
    acceptance_group_ids = group_names(
        acceptance_groups,
        "analysis.physical_acceptance.groups",
    )
    if analysis_group_ids != acceptance_group_ids:
        raise ConfigurationError(
            "physical acceptance groups must exactly match analysis groups"
        )

    selection = _mapping(
        analysis_mapping.get("selection"),
        "analysis.selection",
    )
    _exact_keys(
        selection,
        {"included", "excluded"},
        "analysis.selection",
    )
    if selection.get("excluded") != []:
        raise ConfigurationError("analysis.selection.excluded must be an empty list")
    included = _sequence(
        selection.get("included"),
        "analysis.selection.included",
    )
    expected_included_count = len(analysis_group_ids) * expected_repeats
    if len(included) != expected_included_count:
        raise ConfigurationError(
            "analysis.selection.included count must equal analysis groups times "
            "expected_repeats"
        )
    selection_by_group: dict[
        str,
        set[tuple[str, str, str, int, str, str, str]],
    ] = {}
    selection_identities: set[
        tuple[str, str, str, int, str, str, str]
    ] = set()
    selection_paths: set[str] = set()
    selection_raw_hashes: set[str] = set()
    selection_canonical_hashes: set[str] = set()
    for index, raw_included in enumerate(included):
        included_location = f"analysis.selection.included[{index}]"
        item = _mapping(raw_included, included_location)
        _exact_keys(
            item,
            {
                "path",
                "sha256",
                "canonical_sha256",
                "report_schema_version",
                "environment_id",
                "ground_topology_id",
                "contact_profile_id",
            },
            included_location,
        )
        path = _string(item.get("path"), f"{included_location}.path")
        raw_hash = _sha256(
            item.get("sha256"), f"{included_location}.sha256"
        )
        canonical_hash = _sha256(
            item.get("canonical_sha256"),
            f"{included_location}.canonical_sha256",
        )
        report_schema_version = item.get("report_schema_version")
        if (
            isinstance(report_schema_version, bool)
            or not isinstance(report_schema_version, int)
            or report_schema_version not in {1, 2}
        ):
            raise ConfigurationError(
                f"{included_location}.report_schema_version must be integer 1 or 2"
            )
        environment_id = _string(
            item.get("environment_id"),
            f"{included_location}.environment_id",
        )
        ground_topology_id = _string(
            item.get("ground_topology_id"),
            f"{included_location}.ground_topology_id",
        )
        contact_profile_id = _string(
            item.get("contact_profile_id"),
            f"{included_location}.contact_profile_id",
        )
        group_id = "::".join(
            (environment_id, ground_topology_id, contact_profile_id)
        )
        if group_id not in analysis_group_ids:
            raise ConfigurationError(
                f"{included_location} resolves to unknown group {group_id!r}"
            )
        identity = (
            path,
            raw_hash,
            canonical_hash,
            report_schema_version,
            environment_id,
            ground_topology_id,
            contact_profile_id,
        )
        selection_identities.add(identity)
        selection_paths.add(path)
        selection_raw_hashes.add(raw_hash)
        selection_canonical_hashes.add(canonical_hash)
        selection_by_group.setdefault(group_id, set()).add(identity)
    if not all(
        len(values) == expected_included_count
        for values in (
            selection_identities,
            selection_paths,
            selection_raw_hashes,
            selection_canonical_hashes,
        )
    ):
        raise ConfigurationError(
            "analysis.selection.included must have unique identities, paths, "
            "raw SHA256 values, and canonical SHA256 values"
        )
    if set(selection_by_group) != analysis_group_ids or any(
        len(identities) != expected_repeats
        for identities in selection_by_group.values()
    ):
        raise ConfigurationError(
            "analysis.selection.included must exactly cover every analysis group"
        )
    observed_selection_profiles = {
        identity[6] for identity in selection_identities
    }
    if selected_profiles and set(selected_profiles) != observed_selection_profiles:
        raise ConfigurationError(
            "analysis.selection_policy.expected_profiles must exactly match "
            "selected contact profiles"
        )

    matrix = _mapping(analysis_mapping.get("matrix"), "analysis.matrix")
    _exact_keys(
        matrix,
        {"complete", "required_groups", "observed_groups", "missing_groups"},
        "analysis.matrix",
    )
    if matrix.get("complete") is not True:
        raise ConfigurationError("analysis.matrix.complete must be true")
    if matrix.get("missing_groups") != []:
        raise ConfigurationError("analysis.matrix.missing_groups must be empty")

    def matrix_group_list(field: str) -> list[str]:
        location = f"analysis.matrix.{field}"
        raw = _sequence(matrix.get(field), location)
        parsed = [
            _string(group_id, f"{location}[{index}]")
            for index, group_id in enumerate(raw)
        ]
        if len(parsed) != len(set(parsed)) or parsed != sorted(parsed):
            raise ConfigurationError(
                f"{location} must contain unique, sorted group ids"
            )
        return parsed

    observed_matrix_groups = matrix_group_list("observed_groups")
    if observed_matrix_groups != sorted(analysis_group_ids):
        raise ConfigurationError(
            "analysis.matrix.observed_groups must exactly match analysis groups"
        )
    required_matrix_groups = matrix_group_list("required_groups")
    expected_required_matrix_groups = (
        sorted(analysis_group_ids) if selected_profiles else []
    )
    if required_matrix_groups != expected_required_matrix_groups:
        raise ConfigurationError(
            "analysis.matrix.required_groups contradicts selection_policy and "
            "observed groups"
        )

    def unique_name_list(field: str) -> tuple[list[str], set[str]]:
        location = f"analysis.physical_acceptance.{field}"
        raw = _sequence(acceptance.get(field), location)
        parsed = [
            _string(value, f"{location}[{index}]")
            for index, value in enumerate(raw)
        ]
        if len(parsed) != len(set(parsed)):
            raise ConfigurationError(f"{location} must contain unique group ids")
        unknown = set(parsed) - acceptance_group_ids
        if unknown:
            raise ConfigurationError(
                f"{location} contains unknown groups: {sorted(unknown)}"
            )
        return parsed, set(parsed)

    applicable_list, applicable = unique_name_list("applicable_groups")
    not_applicable_list, not_applicable = unique_name_list(
        "not_applicable_groups"
    )
    passing_list, passing = unique_name_list("passing_groups")
    failed_list, failed = unique_name_list("failed_groups")
    if (
        applicable & not_applicable
        or applicable | not_applicable != acceptance_group_ids
    ):
        raise ConfigurationError(
            "applicable_groups and not_applicable_groups must uniquely and "
            "exactly partition physical acceptance groups"
        )
    if passing & failed or passing | failed != applicable:
        raise ConfigurationError(
            "passing_groups and failed_groups must uniquely and exactly "
            "partition applicable_groups"
        )

    def failed_check_list(value: Any, location: str) -> list[str]:
        raw = _sequence(value, location)
        parsed = [
            _string(check_id, f"{location}[{index}]")
            for index, check_id in enumerate(raw)
        ]
        if len(parsed) != len(set(parsed)):
            raise ConfigurationError(f"{location} must contain unique check ids")
        if parsed != sorted(parsed):
            raise ConfigurationError(f"{location} must be sorted")
        return parsed

    calculated_applicable: list[str] = []
    calculated_not_applicable: list[str] = []
    calculated_passing: list[str] = []
    calculated_failed: list[str] = []
    for group_id in sorted(analysis_group_ids):
        analysis_group = _mapping(
            analysis_groups.get(group_id),
            f"analysis.groups.{group_id}",
        )
        acceptance_group = _mapping(
            acceptance_groups.get(group_id),
            f"analysis.physical_acceptance.groups.{group_id}",
        )
        acceptance_location = (
            f"analysis.physical_acceptance.groups.{group_id}"
        )
        _exact_keys(
            acceptance_group,
            {
                "applicable",
                "passed",
                "not_applicable_reasons",
                "repeat_count",
                "checks",
                "failed_checks",
                "repeat_results",
            },
            acceptance_location,
        )
        _exact_integer(
            analysis_group.get("repeat_count"),
            expected_repeats,
            f"analysis.groups.{group_id}.repeat_count",
        )
        _exact_integer(
            acceptance_group.get("repeat_count"),
            expected_repeats,
            f"{acceptance_location}.repeat_count",
        )
        input_reports = _sequence(
            analysis_group.get("input_reports"),
            f"analysis.groups.{group_id}.input_reports",
        )
        if len(input_reports) != expected_repeats:
            raise ConfigurationError(
                f"analysis.groups.{group_id}.input_reports must contain "
                f"exactly {expected_repeats} reports"
            )
        expected_report_identities: set[tuple[str, str, str]] = set()
        expected_paths: set[str] = set()
        expected_raw_hashes: set[str] = set()
        expected_canonical_hashes: set[str] = set()
        group_report_identities: set[tuple[str, str, str, int]] = set()
        report_schema_versions: list[int] = []
        for index, raw_report in enumerate(input_reports):
            report_location = (
                f"analysis.groups.{group_id}.input_reports[{index}]"
            )
            report = _mapping(raw_report, report_location)
            _exact_keys(
                report,
                {
                    "path",
                    "sha256",
                    "canonical_sha256",
                    "report_schema_version",
                },
                report_location,
            )
            path = _string(report.get("path"), f"{report_location}.path")
            raw_hash = _sha256(
                report.get("sha256"), f"{report_location}.sha256"
            )
            canonical_hash = _sha256(
                report.get("canonical_sha256"),
                f"{report_location}.canonical_sha256",
            )
            report_schema_version = report.get("report_schema_version")
            if (
                isinstance(report_schema_version, bool)
                or not isinstance(report_schema_version, int)
                or report_schema_version not in {1, 2}
            ):
                raise ConfigurationError(
                    f"{report_location}.report_schema_version must be integer "
                    "1 or 2"
                )
            report_schema_versions.append(report_schema_version)
            group_report_identities.add(
                (path, raw_hash, canonical_hash, report_schema_version)
            )
            expected_report_identities.add((path, raw_hash, canonical_hash))
            expected_paths.add(path)
            expected_raw_hashes.add(raw_hash)
            expected_canonical_hashes.add(canonical_hash)
        if not all(
            len(values) == expected_repeats
            for values in (
                expected_report_identities,
                expected_paths,
                expected_raw_hashes,
                expected_canonical_hashes,
            )
        ):
            raise ConfigurationError(
                f"analysis.groups.{group_id}.input_reports must have unique "
                "paths, raw SHA256 values, and canonical SHA256 values"
            )
        if len(group_report_identities) != expected_repeats:
            raise ConfigurationError(
                f"analysis.groups.{group_id}.input_reports must have unique "
                "report schema-bound identities"
            )

        runtime_provenance_schema = analysis_group.get(
            "runtime_provenance_schema"
        )
        if (
            isinstance(runtime_provenance_schema, bool)
            or not isinstance(runtime_provenance_schema, int)
            or runtime_provenance_schema not in {3, 4, 5}
        ):
            raise ConfigurationError(
                f"analysis.groups.{group_id}.runtime_provenance_schema must "
                "be integer 3, 4, or 5"
            )
        if runtime_provenance_schema != selection_runtime_provenance_schema:
            raise ConfigurationError(
                f"analysis.groups.{group_id}.runtime_provenance_schema must "
                "match analysis.selection_policy"
            )
        environment_id = _string(
            analysis_group.get("environment_id"),
            f"analysis.groups.{group_id}.environment_id",
        )
        ground_topology_id = _string(
            analysis_group.get("ground_topology_id"),
            f"analysis.groups.{group_id}.ground_topology_id",
        )
        odometry_mode = _string(
            analysis_group.get("odometry_mode"),
            f"analysis.groups.{group_id}.odometry_mode",
        )
        if odometry_mode != locked_odometry_mode:
            raise ConfigurationError(
                f"analysis.groups.{group_id}.odometry_mode must match "
                "analysis.locked_inputs.simulation.odometry_mode"
            )
        contact_profile_id = _string(
            analysis_group.get("contact_profile_id"),
            f"analysis.groups.{group_id}.contact_profile_id",
        )
        expected_group_id = "::".join(
            (environment_id, ground_topology_id, contact_profile_id)
        )
        if group_id != expected_group_id:
            raise ConfigurationError(
                f"analysis group id {group_id!r} contradicts its environment, "
                "ground topology, and contact profile identity"
            )
        group_selection_identities = {
            (
                path,
                raw_hash,
                canonical_hash,
                report_schema_version,
                environment_id,
                ground_topology_id,
                contact_profile_id,
            )
            for (
                path,
                raw_hash,
                canonical_hash,
                report_schema_version,
            ) in group_report_identities
        }
        if group_selection_identities != selection_by_group[group_id]:
            raise ConfigurationError(
                f"analysis.groups.{group_id}.input_reports and identity fields "
                "must exactly match analysis.selection.included"
            )
        expected_reasons: list[str] = []
        if any(version != 2 for version in report_schema_versions):
            expected_reasons.append("motion_report_schema_not_2")
        if runtime_provenance_schema != 5:
            expected_reasons.append("runtime_provenance_schema_not_5")
        if environment_id != "SimplePlane":
            expected_reasons.append("environment_not_SimplePlane")
        if ground_topology_id != "simple_plane_only1_v1":
            expected_reasons.append(
                "ground_topology_not_simple_plane_only1_v1"
            )
        if odometry_mode != "ideal":
            expected_reasons.append("odometry_mode_not_ideal")
        if len(expected_report_identities) < 3:
            expected_reasons.append("fewer_than_3_unique_repeats")
        expected_applicable = not expected_reasons

        applicable_value = acceptance_group.get("applicable")
        if not isinstance(applicable_value, bool):
            raise ConfigurationError(
                f"{acceptance_location}.applicable must be a boolean"
            )
        if applicable_value is not expected_applicable:
            raise ConfigurationError(
                f"{acceptance_location}.applicable contradicts analysis group "
                "identity"
            )
        if applicable_value is not (group_id in applicable):
            raise ConfigurationError(
                f"{acceptance_location}.applicable disagrees with the top-level "
                "applicability partition"
            )
        reasons = _sequence(
            acceptance_group.get("not_applicable_reasons"),
            f"{acceptance_location}.not_applicable_reasons",
        )
        parsed_reasons = [
            _string(reason, f"{acceptance_location}.not_applicable_reasons")
            for reason in reasons
        ]
        if len(parsed_reasons) != len(set(parsed_reasons)):
            raise ConfigurationError(
                f"{acceptance_location}.not_applicable_reasons must be unique"
            )
        if parsed_reasons != expected_reasons:
            raise ConfigurationError(
                f"{acceptance_location}.not_applicable_reasons contradicts "
                "analysis group identity"
            )
        group_checks = _mapping(
            acceptance_group.get("checks"),
            f"{acceptance_location}.checks",
        )
        group_failed_checks = failed_check_list(
            acceptance_group.get("failed_checks"),
            f"{acceptance_location}.failed_checks",
        )
        repeat_results = _sequence(
            acceptance_group.get("repeat_results"),
            f"{acceptance_location}.repeat_results",
        )

        if not applicable_value:
            calculated_not_applicable.append(group_id)
            if acceptance_group.get("passed") is not None:
                raise ConfigurationError(
                    f"{acceptance_location}.passed must be null when not applicable"
                )
            if not parsed_reasons:
                raise ConfigurationError(
                    f"{acceptance_location}.not_applicable_reasons must be non-empty"
                )
            if group_checks or group_failed_checks or repeat_results:
                raise ConfigurationError(
                    f"{acceptance_location} not-applicable evidence must have "
                    "empty checks, failed_checks, and repeat_results"
                )
            continue

        calculated_applicable.append(group_id)
        if parsed_reasons:
            raise ConfigurationError(
                f"{acceptance_location}.not_applicable_reasons must be empty "
                "when applicable"
            )
        group_passed = acceptance_group.get("passed")
        if not isinstance(group_passed, bool):
            raise ConfigurationError(
                f"{acceptance_location}.passed must be a boolean when applicable"
            )
        if len(repeat_results) != expected_repeats:
            raise ConfigurationError(
                f"{acceptance_location}.repeat_results must contain exactly "
                f"{expected_repeats} results"
            )

        observed_report_identities: set[tuple[str, str, str]] = set()
        observed_paths: set[str] = set()
        observed_raw_hashes: set[str] = set()
        observed_canonical_hashes: set[str] = set()
        repeated_checks: list[dict[str, bool]] = []
        expected_check_ids = set(_PHYSICAL_ACCEPTANCE_CHECK_IDS)
        for position, raw_repeat in enumerate(repeat_results, start=1):
            repeat_location = f"{acceptance_location}.repeat_results[{position - 1}]"
            repeat = _mapping(raw_repeat, repeat_location)
            _exact_keys(
                repeat,
                {
                    "repeat_index",
                    "report_path",
                    "report_sha256",
                    "canonical_sha256",
                    "passed",
                    "checks",
                    "failed_checks",
                },
                repeat_location,
            )
            _exact_integer(
                repeat.get("repeat_index"),
                position,
                f"{repeat_location}.repeat_index",
            )
            path = _string(
                repeat.get("report_path"), f"{repeat_location}.report_path"
            )
            raw_hash = _sha256(
                repeat.get("report_sha256"),
                f"{repeat_location}.report_sha256",
            )
            canonical_hash = _sha256(
                repeat.get("canonical_sha256"),
                f"{repeat_location}.canonical_sha256",
            )
            observed_report_identities.add((path, raw_hash, canonical_hash))
            observed_paths.add(path)
            observed_raw_hashes.add(raw_hash)
            observed_canonical_hashes.add(canonical_hash)
            repeat_checks = _mapping(
                repeat.get("checks"), f"{repeat_location}.checks"
            )
            check_outcomes = _validated_physical_repeat_check_outcomes(
                repeat_checks,
                f"{repeat_location}.checks",
            )
            expected_repeat_failures = sorted(
                check_id
                for check_id, passed_value in check_outcomes.items()
                if not passed_value
            )
            repeat_failures = failed_check_list(
                repeat.get("failed_checks"),
                f"{repeat_location}.failed_checks",
            )
            if repeat_failures != expected_repeat_failures:
                raise ConfigurationError(
                    f"{repeat_location}.failed_checks disagrees with repeat checks"
                )
            repeat_passed = repeat.get("passed")
            if not isinstance(repeat_passed, bool):
                raise ConfigurationError(
                    f"{repeat_location}.passed must be a boolean"
                )
            if repeat_passed is not (not expected_repeat_failures):
                raise ConfigurationError(
                    f"{repeat_location}.passed disagrees with repeat checks"
                )
            repeated_checks.append(check_outcomes)

        if not all(
            len(values) == expected_repeats
            for values in (
                observed_report_identities,
                observed_paths,
                observed_raw_hashes,
                observed_canonical_hashes,
            )
        ):
            raise ConfigurationError(
                f"{acceptance_location}.repeat_results must have unique paths, "
                "raw SHA256 values, and canonical SHA256 values"
            )
        if observed_report_identities != expected_report_identities:
            raise ConfigurationError(
                f"{acceptance_location}.repeat_results must exactly cover "
                f"analysis.groups.{group_id}.input_reports"
            )

        if set(group_checks) != expected_check_ids:
            raise ConfigurationError(
                f"{acceptance_location}.checks must exactly cover repeat check ids"
            )
        recomputed_group_failures: list[str] = []
        for check_id in sorted(expected_check_ids):
            check_location = f"{acceptance_location}.checks.{check_id}"
            summary = _mapping(group_checks.get(check_id), check_location)
            _exact_keys(
                summary,
                {"passed_repeats", "failed_repeats", "all_repeats_passed"},
                check_location,
            )
            passed_repeats = sum(
                outcomes[check_id] for outcomes in repeated_checks
            )
            failed_repeats = expected_repeats - passed_repeats
            _exact_integer(
                summary.get("passed_repeats"),
                passed_repeats,
                f"{check_location}.passed_repeats",
            )
            _exact_integer(
                summary.get("failed_repeats"),
                failed_repeats,
                f"{check_location}.failed_repeats",
            )
            all_repeats_passed = summary.get("all_repeats_passed")
            if not isinstance(all_repeats_passed, bool):
                raise ConfigurationError(
                    f"{check_location}.all_repeats_passed must be a boolean"
                )
            expected_all_passed = failed_repeats == 0
            if all_repeats_passed is not expected_all_passed:
                raise ConfigurationError(
                    f"{check_location}.all_repeats_passed disagrees with repeat "
                    "checks"
                )
            if not expected_all_passed:
                recomputed_group_failures.append(check_id)
        if group_failed_checks != recomputed_group_failures:
            raise ConfigurationError(
                f"{acceptance_location}.failed_checks disagrees with group checks"
            )
        expected_group_passed = not recomputed_group_failures
        if group_passed is not expected_group_passed:
            raise ConfigurationError(
                f"{acceptance_location}.passed disagrees with group checks"
            )
        (
            calculated_passing
            if expected_group_passed
            else calculated_failed
        ).append(group_id)
        if (group_id in passing) is not expected_group_passed:
            raise ConfigurationError(
                f"{acceptance_location}.passed disagrees with passing_groups and "
                "failed_groups"
            )

    for field, observed, expected in (
        ("applicable_groups", applicable_list, calculated_applicable),
        (
            "not_applicable_groups",
            not_applicable_list,
            calculated_not_applicable,
        ),
        ("passing_groups", passing_list, calculated_passing),
        ("failed_groups", failed_list, calculated_failed),
    ):
        if observed != expected:
            raise ConfigurationError(
                f"analysis.physical_acceptance.{field} contradicts recomputed "
                "group semantics"
            )
    top_verdict = acceptance.get("all_applicable_groups_passed")
    expected_top_verdict: bool | None = (
        None if not calculated_applicable else not calculated_failed
    )
    if expected_top_verdict is None:
        if top_verdict is not None:
            raise ConfigurationError(
                "all_applicable_groups_passed must be null when no group applies"
            )
    elif not isinstance(top_verdict, bool) or top_verdict is not expected_top_verdict:
        raise ConfigurationError(
            "all_applicable_groups_passed disagrees with applicable group verdicts"
        )


def analyse_contact_ab(
    report_paths: Sequence[str | Path],
    wheel_radius_m: float,
    *,
    min_repeats: int = 3,
    expected_environments: Sequence[str] = (),
    expected_topologies: Sequence[str] = (),
    expected_profiles: Sequence[str] = (),
    require_complete_matrix: bool = False,
) -> dict[str, object]:
    """Validate and summarize one homogeneous schema-3, -4, or -5 batch."""
    radius = _positive(wheel_radius_m, "wheel_radius_m")
    if (
        isinstance(min_repeats, bool)
        or not isinstance(min_repeats, int)
        or min_repeats < 1
    ):
        raise ConfigurationError("min_repeats must be a positive integer")
    if not report_paths:
        raise ConfigurationError("at least one motion report is required")
    selected_environments = _identifier_values(
        expected_environments, "expected_environments"
    )
    selected_topologies = _identifier_values(
        expected_topologies, "expected_topologies"
    )
    explicit_topology_selection = bool(selected_topologies)
    selected_profiles = _identifier_values(expected_profiles, "expected_profiles")
    if require_complete_matrix:
        if selected_environments or selected_topologies or selected_profiles:
            raise ConfigurationError(
                "require_complete_matrix cannot be combined with expected selectors"
            )
        selected_environments = COMPLETE_MATRIX_ENVIRONMENTS
        selected_topologies = tuple(
            topology
            for _environment, topology
            in COMPLETE_MATRIX_ENVIRONMENT_TOPOLOGIES
        )
        selected_profiles = COMPLETE_MATRIX_PROFILES

    resolved = [Path(path).expanduser().resolve() for path in report_paths]
    if len(set(resolved)) != len(resolved):
        raise ConfigurationError("motion report paths must be unique")
    included: list[InputRecord] = []
    exclusions: list[dict[str, Any]] = []
    content_sources: dict[str, Path] = {}
    for source in resolved:
        if not source.is_file():
            raise FileNotFoundError(f"motion report does not exist: {source}")
        content = source.read_bytes()
        source_digest = hashlib.sha256(content).hexdigest()
        base: dict[str, Any] = {
            "path": str(source),
            "sha256": source_digest,
            "canonical_sha256": None,
        }
        try:
            document = _strict_json(content, f"report {source}")
            canonical_digest = hashlib.sha256(
                _canonical(document).encode("utf-8")
            ).hexdigest()
        except ConfigurationError as exc:
            exclusions.append({**base, "reasons": [_reason_for_invalid(exc)]})
            continue
        base["canonical_sha256"] = canonical_digest
        if canonical_digest in content_sources:
            exclusions.append(
                {
                    **base,
                    "reasons": [
                        _exclusion(
                            "duplicate_report_content",
                            f"duplicates {content_sources[canonical_digest]}",
                        )
                    ],
                }
            )
            continue
        content_sources[canonical_digest] = source
        try:
            record = _validated_record(
                document,
                source,
                source_digest,
                canonical_digest,
                radius,
            )
        except ConfigurationError as exc:
            exclusions.append({**base, "reasons": [_reason_for_invalid(exc)]})
            continue
        reasons = []
        if selected_environments and record.environment_id not in selected_environments:
            reasons.append(
                _exclusion("unexpected_environment", record.environment_id)
            )
        if selected_profiles and record.contact_profile_id not in selected_profiles:
            reasons.append(_exclusion("unexpected_profile", record.contact_profile_id))
        if (
            selected_topologies
            and record.ground_topology_id is not None
            and record.ground_topology_id not in selected_topologies
        ):
            reasons.append(
                _exclusion(
                    "unexpected_ground_topology",
                    record.ground_topology_id,
                )
            )
        if reasons:
            exclusions.append({**base, "reasons": reasons})
            continue
        included.append(record)
    if not included:
        message = "no valid contact A/B reports remain after selection"
        if exclusions and exclusions[0].get("reasons"):
            first_reason = exclusions[0]["reasons"][0]
            message += (
                f"; first exclusion [{first_reason.get('code', 'unknown')}]: "
                f"{first_reason.get('detail', 'no detail')}"
            )
        raise ConfigurationError(message)

    provenance_schemas = {
        record.runtime_provenance_schema for record in included
    }
    if len(provenance_schemas) != 1:
        raise ConfigurationError(
            "mixed runtime provenance schemas are forbidden in one contact "
            f"A/B batch: observed {sorted(provenance_schemas)}"
        )
    runtime_provenance_schema = next(iter(provenance_schemas))
    if runtime_provenance_schema != 5 and explicit_topology_selection:
        raise ConfigurationError(
            "expected_topologies requires runtime provenance schema 5; "
            f"observed schema {runtime_provenance_schema}"
        )

    reference_lock = included[0].global_lock
    for record in included[1:]:
        if _canonical(record.global_lock) != _canonical(reference_lock):
            raise ConfigurationError(
                "global input lock mismatch: "
                f"{record.path} differs from {included[0].path}"
            )
    environment_references: dict[str, InputRecord] = {}
    topology_references: dict[tuple[str, str], InputRecord] = {}
    topology_ab_contact_references: dict[tuple[str, str], InputRecord] = {}
    profile_references: dict[str, InputRecord] = {}
    for record in included:
        environment_reference = environment_references.setdefault(
            record.environment_id, record
        )
        if _canonical(record.environment_lock) != _canonical(
            environment_reference.environment_lock
        ):
            raise ConfigurationError(
                "environment contract mismatch for "
                f"{record.environment_id}: {record.path} differs from "
                f"{environment_reference.path}"
            )
        if runtime_provenance_schema == 5:
            if record.ground_topology_id is None or record.topology_lock is None:
                raise ConfigurationError(
                    f"schema-5 report {record.path} lacks a topology identity lock"
                )
            topology_key = (
                record.environment_id,
                record.ground_topology_id,
            )
            topology_reference = topology_references.setdefault(
                topology_key,
                record,
            )
            if _canonical(record.topology_lock) != _canonical(
                topology_reference.topology_lock
            ):
                raise ConfigurationError(
                    "ground topology contract mismatch for "
                    f"{topology_key[0]}::{topology_key[1]}: "
                    f"{record.path} differs from {topology_reference.path}"
                )
            if record.topology_ab_contact_lock is None:
                raise ConfigurationError(
                    f"schema-5 report {record.path} lacks a topology A/B "
                    "contact invariant lock"
                )
            contact_ab_key = (
                record.environment_id,
                record.contact_profile_id,
            )
            contact_ab_reference = topology_ab_contact_references.setdefault(
                contact_ab_key,
                record,
            )
            if _canonical(record.topology_ab_contact_lock) != _canonical(
                contact_ab_reference.topology_ab_contact_lock
            ):
                raise ConfigurationError(
                    "topology A/B contact invariant mismatch for "
                    f"{contact_ab_key[0]}::{contact_ab_key[1]}: "
                    f"{record.path} differs from {contact_ab_reference.path}"
                )
        profile_reference = profile_references.setdefault(
            record.contact_profile_id, record
        )
        if _canonical(record.profile_lock) != _canonical(
            profile_reference.profile_lock
        ):
            raise ConfigurationError(
                "profile contract mismatch for "
                f"{record.contact_profile_id}: {record.path} differs from "
                f"{profile_reference.path}"
            )
    grouped: dict[tuple[str, ...], list[InputRecord]] = {}
    for record in included:
        group_key = (
            (
                record.environment_id,
                record.ground_topology_id,
                record.contact_profile_id,
            )
            if runtime_provenance_schema == 5
            else (record.environment_id, record.contact_profile_id)
        )
        if any(value is None for value in group_key):
            raise ConfigurationError(
                f"schema-5 report {record.path} lacks a ground topology id"
            )
        grouped.setdefault(tuple(group_key), []).append(record)
    for group_key, records in grouped.items():
        reference_contact = records[0].contact_lock
        for record in records[1:]:
            if _canonical(record.contact_lock) != _canonical(reference_contact):
                raise ConfigurationError(
                    "contact contract mismatch within group "
                    + "::".join(group_key)
                )
        if len(records) < min_repeats:
            raise ConfigurationError(
                f"group {'::'.join(group_key)} requires at least "
                f"{min_repeats} unique report contents; observed {len(records)}"
            )

    observed_groups = set(grouped)
    required_groups: set[tuple[str, ...]] = set()
    if runtime_provenance_schema == 5:
        observed_environments = {group[0] for group in observed_groups}
        observed_topologies = {group[1] for group in observed_groups}
        observed_profiles = {group[2] for group in observed_groups}
        observed_pairs = {(group[0], group[1]) for group in observed_groups}
        required_pairs: set[tuple[str, str]]
        if selected_topologies:
            required_pairs = set()
            for topology in selected_topologies:
                environment = _SHIPPED_TOPOLOGY_ENVIRONMENTS.get(topology)
                if environment is None:
                    if len(selected_environments) != 1:
                        raise ConfigurationError(
                            "an unknown expected topology requires exactly one "
                            f"expected environment: {topology}"
                        )
                    environment = selected_environments[0]
                if (
                    selected_environments
                    and environment not in selected_environments
                ):
                    raise ConfigurationError(
                        "expected environment/topology selectors are "
                        f"incompatible: {environment}::{topology}"
                    )
                required_pairs.add((environment, topology))
            missing_pairs = required_pairs - observed_pairs
            if missing_pairs:
                raise ConfigurationError(
                    "expected environment/topology pairs are missing: "
                    + ", ".join(
                        f"{environment}::{topology}"
                        for environment, topology in sorted(missing_pairs)
                    )
                )
        else:
            required_pairs = {
                pair
                for pair in observed_pairs
                if not selected_environments
                or pair[0] in selected_environments
            }
        if selected_environments:
            missing_environments = (
                set(selected_environments) - observed_environments
            )
            if missing_environments:
                raise ConfigurationError(
                    "expected environments are missing: "
                    f"{sorted(missing_environments)}"
                )
        if selected_profiles:
            required_groups = {
                (environment, topology, profile)
                for environment, topology in required_pairs
                for profile in selected_profiles
            }
            missing_profiles = set(selected_profiles) - observed_profiles
            if missing_profiles:
                raise ConfigurationError(
                    f"expected profiles are missing: {sorted(missing_profiles)}"
                )
        if selected_topologies:
            missing_topologies = set(selected_topologies) - observed_topologies
            if missing_topologies:
                raise ConfigurationError(
                    "expected ground topologies are missing: "
                    f"{sorted(missing_topologies)}"
                )
    else:
        if selected_environments and selected_profiles:
            required_groups = {
                (environment, profile)
                for environment in selected_environments
                for profile in selected_profiles
            }
        elif selected_environments:
            missing_environments = set(selected_environments) - {
                environment for environment, _profile in observed_groups
            }
            if missing_environments:
                raise ConfigurationError(
                    "expected environments are missing: "
                    f"{sorted(missing_environments)}"
                )
        elif selected_profiles:
            missing_profiles = set(selected_profiles) - {
                profile for _environment, profile in observed_groups
            }
            if missing_profiles:
                raise ConfigurationError(
                    f"expected profiles are missing: {sorted(missing_profiles)}"
                )
    missing_groups = sorted(required_groups - observed_groups)
    if missing_groups:
        raise ConfigurationError(
            "required contact A/B groups are missing: "
            + ", ".join("::".join(group) for group in missing_groups)
        )
    matrix_complete = not missing_groups
    group_summaries = {
        "::".join(group): _group_summary(records)
        for group, records in sorted(grouped.items())
    }
    report = {
        "schema_version": 3 if runtime_provenance_schema == 5 else 1,
        "report_type": "contact_ab_analysis",
        "analysis_valid": not exclusions and matrix_complete,
        "method": {
            "motion_profile_id": _MOTION_PROFILE_ID,
            "wheel_radius_m": radius,
            "minimum_unique_repeats_per_group": min_repeats,
            "distribution": "population stddev; nearest-rank p95",
            "effective_track_definition": (
                "wheel_radius_m * (right_mean_rate - left_mean_rate) / "
                "measured_mean_yaw_rate"
            ),
            "ranking_policy": "none; this report never selects a best profile",
            "report_content_digest": (
                "SHA256 of compact, key-sorted strict JSON; source SHA256 is "
                "retained separately"
            ),
        },
        "locked_inputs": {"wheel_radius_m": radius, **reference_lock},
        "environment_contracts": {
            environment_id: reference.environment_lock
            for environment_id, reference in sorted(environment_references.items())
        },
        **(
            {
                "topology_contracts": {
                    f"{environment_id}::{topology_id}": (
                        reference.topology_lock
                    )
                    for (environment_id, topology_id), reference
                    in sorted(topology_references.items())
                }
            }
            if runtime_provenance_schema == 5
            else {}
        ),
        "profile_contracts": {
            profile_id: reference.profile_lock
            for profile_id, reference in sorted(profile_references.items())
        },
        "selection_policy": {
            "required_report_result": "success",
            "required_runtime_provenance_schema": (
                runtime_provenance_schema
            ),
            "required_git_dirty": False,
            "expected_environments": list(selected_environments),
            **(
                {"expected_topologies": list(selected_topologies)}
                if runtime_provenance_schema == 5
                else {}
            ),
            "expected_profiles": list(selected_profiles),
            "require_complete_matrix": require_complete_matrix,
        },
        "selection": {
            "included": [
                {
                    "path": record.path,
                    "sha256": record.sha256,
                    "canonical_sha256": record.canonical_sha256,
                    "report_schema_version": record.report_schema_version,
                    "environment_id": record.environment_id,
                    **(
                        {"ground_topology_id": record.ground_topology_id}
                        if runtime_provenance_schema == 5
                        else {}
                    ),
                    "contact_profile_id": record.contact_profile_id,
                }
                for record in included
            ],
            "excluded": exclusions,
        },
        "counts": {
            "input_reports": len(resolved),
            "included_reports": len(included),
            "excluded_reports": len(exclusions),
            "groups": len(grouped),
        },
        "matrix": {
            "complete": matrix_complete,
            "required_groups": [
                "::".join(group) for group in sorted(required_groups)
            ],
            "observed_groups": [
                "::".join(group) for group in sorted(observed_groups)
            ],
            "missing_groups": [
                "::".join(group) for group in missing_groups
            ],
        },
        **(
            {"physical_acceptance": _physical_acceptance(grouped)}
            if runtime_provenance_schema == 5
            else {}
        ),
        "groups": group_summaries,
    }
    _canonical(report)
    return report


def write_contact_ab_report(report: Mapping[str, Any], output_path: str | Path) -> Path:
    """Atomically write one strict contact A/B JSON report."""
    output = Path(output_path).expanduser().resolve()

    def writer(stream) -> None:
        json.dump(report, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")

    _atomic_text_write(output, writer)
    return output


def _positive_integer(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be a positive integer") from exc
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return parsed


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "reports",
        nargs="+",
        help="homogeneous schema-3, -4, or -5 motion report JSON files",
    )
    parser.add_argument(
        "--wheel-radius",
        required=True,
        type=float,
        help="wheel radius in metres",
    )
    parser.add_argument(
        "--output",
        required=True,
        help="strict atomic JSON output path",
    )
    parser.add_argument("--min-repeats", type=_positive_integer, default=3)
    parser.add_argument("--expected-environment", action="append", default=[])
    parser.add_argument("--expected-topology", action="append", default=[])
    parser.add_argument("--expected-profile", action="append", default=[])
    parser.add_argument(
        "--require-complete-matrix",
        action="store_true",
        help=(
            "require the shipped environment/contact matrix; schema 5 also "
            "requires all three legal environment/topology pairs"
        ),
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Run the installed offline CLI."""
    arguments = _parser().parse_args(argv)
    try:
        report = analyse_contact_ab(
            arguments.reports,
            arguments.wheel_radius,
            min_repeats=arguments.min_repeats,
            expected_environments=arguments.expected_environment,
            expected_topologies=arguments.expected_topology,
            expected_profiles=arguments.expected_profile,
            require_complete_matrix=arguments.require_complete_matrix,
        )
        output = write_contact_ab_report(report, arguments.output)
    except (ConfigurationError, OSError, ValueError) as exc:
        print(f"contact A/B analysis failed: {exc}", file=sys.stderr)
        return 1
    print(output)
    return 0 if report["analysis_valid"] is True else 2


if __name__ == "__main__":
    raise SystemExit(main())
