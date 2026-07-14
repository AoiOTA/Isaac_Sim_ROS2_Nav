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


@dataclass(frozen=True)
class InputRecord:
    """One fully validated report reduced to identity and metric records."""

    path: str
    sha256: str
    canonical_sha256: str
    environment_id: str
    contact_profile_id: str
    global_lock: dict[str, Any]
    environment_lock: dict[str, Any]
    profile_lock: dict[str, Any]
    contact_lock: dict[str, Any]
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
    location: str,
) -> dict[str, float]:
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
        actual_velocity, _ACTUAL_VELOCITY_KEYS, f"{location}.actual_velocity"
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
    return metrics


def _identity_locks(
    report: Mapping[str, Any],
    provenance: Mapping[str, Any],
    configuration: Mapping[str, Any],
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any], dict[str, Any]]:
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
    environment_lock = {
        "environment": {
            key: environment[key]
            for key in sorted(environment)
            if key != "composed_root_layer_sha256"
        },
        # These are project/environment discovery contracts, not contact
        # profile variables.  They must remain identical across profiles in a
        # given environment, while Warehouse and SimplePlane legitimately
        # have different ground topology.
        "collider_contract": contact["collider_contract"],
        "wheel_colliders": contact["wheel_colliders"],
        "ground_colliders": contact["ground_colliders"],
    }
    profile_lock = {
        "profile_path": contact["profile_path"],
        "profile_sha256": contact["profile_sha256"],
        "profile_id": contact["profile_id"],
        "profile_mode": contact["profile_mode"],
        "explicit_materials": contact["explicit_materials"],
        "thresholds_authored": contact["thresholds_authored"],
    }
    # The anonymous identifier embeds a process-specific address and is not an
    # identity.  Its canonical content hash is stable within an env/profile
    # group and is deliberately locked along with collider/binding evidence.
    contact_lock = {
        key: contact[key]
        for key in sorted(contact)
        if key != "overlay_identifier"
    }
    # The exported composed root reflects the exact environment/profile Stage
    # composition. Contact profiles may intentionally change it, while
    # repeats of the same environment/profile must remain byte-identical.
    contact_lock["composed_root_layer_sha256"] = environment[
        "composed_root_layer_sha256"
    ]
    return global_lock, environment_lock, profile_lock, contact_lock


def _validated_record(
    report: Mapping[str, Any],
    source_path: Path,
    source_sha256: str,
    canonical_sha256: str,
    wheel_radius_m: float,
) -> InputRecord:
    location = f"report {source_path}"
    _exact_keys(report, _REPORT_KEYS, location)
    _exact_integer(report.get("schema_version"), 1, f"{location} schema_version")
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
    _exact_keys(provenance, _RUNTIME_PROVENANCE_KEYS, f"{location}.runtime_provenance")
    _exact_integer(
        provenance.get("schema_version"),
        3,
        f"{location}.runtime_provenance.schema_version",
    )
    try:
        validate_runtime_provenance(provenance)
    except ReportValidationError as exc:
        raise ConfigurationError(f"{location} runtime provenance: {exc}") from exc
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
    if report.get("odometry_mode") != simulation.get("odometry_mode"):
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
    metrics = {
        specification[0]: _segment_metrics(
            _mapping(raw_segments[index], f"{location}.segments[{index}]"),
            specification,
            wheel_layout,
            wheel_radius_m,
            direction_deadband_radps,
            f"{location}.segments[{index}]({specification[0]})",
        )
        for index, specification in enumerate(_SEGMENT_SPECS)
    }
    global_lock, environment_lock, profile_lock, contact_lock = _identity_locks(
        report, provenance, configuration
    )
    contact_profile_id = _string(
        contact.get("profile_id"),
        f"{location}.runtime_provenance.contact.profile_id",
    )
    return InputRecord(
        path=str(source_path),
        sha256=source_sha256,
        canonical_sha256=canonical_sha256,
        environment_id=environment_id,
        contact_profile_id=contact_profile_id,
        global_lock=global_lock,
        environment_lock=environment_lock,
        profile_lock=profile_lock,
        contact_lock=contact_lock,
        segments=metrics,
    )


def _reason_for_invalid(exc: ConfigurationError) -> dict[str, str]:
    detail = str(exc)
    if "Git worktree must be clean" in detail:
        return _exclusion("git_dirty", detail)
    if ".profile_id must" in detail:
        return _exclusion("invalid_motion_profile", detail)
    if "runtime provenance" in detail:
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
    return {
        "environment_id": first.environment_id,
        "contact_profile_id": first.contact_profile_id,
        "contact_contract": first.contact_lock,
        "repeat_count": len(records),
        "input_reports": [
            {
                "path": record.path,
                "sha256": record.sha256,
                "canonical_sha256": record.canonical_sha256,
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


def analyse_contact_ab(
    report_paths: Sequence[str | Path],
    wheel_radius_m: float,
    *,
    min_repeats: int = 3,
    expected_environments: Sequence[str] = (),
    expected_profiles: Sequence[str] = (),
    require_complete_matrix: bool = False,
) -> dict[str, object]:
    """Validate, group, and summarize schema-3 motion reports."""
    radius = _positive(wheel_radius_m, "wheel_radius_m")
    if radius != CANONICAL_WHEEL_RADIUS_M:
        raise ConfigurationError(
            "wheel_radius_m must exactly match the canonical Jackal radius "
            f"{CANONICAL_WHEEL_RADIUS_M}"
        )
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
    selected_profiles = _identifier_values(expected_profiles, "expected_profiles")
    if require_complete_matrix:
        if selected_environments or selected_profiles:
            raise ConfigurationError(
                "require_complete_matrix cannot be combined with expected selectors"
            )
        selected_environments = COMPLETE_MATRIX_ENVIRONMENTS
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

    reference_lock = included[0].global_lock
    for record in included[1:]:
        if _canonical(record.global_lock) != _canonical(reference_lock):
            raise ConfigurationError(
                "global input lock mismatch: "
                f"{record.path} differs from {included[0].path}"
            )
    environment_references: dict[str, InputRecord] = {}
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
    grouped: dict[tuple[str, str], list[InputRecord]] = {}
    for record in included:
        grouped.setdefault(
            (record.environment_id, record.contact_profile_id), []
        ).append(record)
    for group_key, records in grouped.items():
        reference_contact = records[0].contact_lock
        for record in records[1:]:
            if _canonical(record.contact_lock) != _canonical(reference_contact):
                raise ConfigurationError(
                    "contact contract mismatch within group "
                    f"{group_key[0]}::{group_key[1]}"
                )
        if len(records) < min_repeats:
            raise ConfigurationError(
                f"group {group_key[0]}::{group_key[1]} requires at least "
                f"{min_repeats} unique report contents; observed {len(records)}"
            )

    observed_groups = set(grouped)
    required_groups: set[tuple[str, str]] = set()
    if selected_environments and selected_profiles:
        required_groups = {
            (environment, profile)
            for environment in selected_environments
            for profile in selected_profiles
        }
    elif selected_environments:
        missing_environments = set(selected_environments) - {
            environment for environment, _ in observed_groups
        }
        if missing_environments:
            raise ConfigurationError(
                f"expected environments are missing: {sorted(missing_environments)}"
            )
    elif selected_profiles:
        missing_profiles = set(selected_profiles) - {
            profile for _, profile in observed_groups
        }
        if missing_profiles:
            raise ConfigurationError(
                f"expected profiles are missing: {sorted(missing_profiles)}"
            )
    missing_groups = sorted(required_groups - observed_groups)
    if missing_groups:
        raise ConfigurationError(
            "required contact A/B groups are missing: "
            + ", ".join(f"{environment}::{profile}" for environment, profile in missing_groups)
        )
    matrix_complete = not missing_groups
    group_summaries = {
        f"{environment}::{profile}": _group_summary(records)
        for (environment, profile), records in sorted(grouped.items())
    }
    report = {
        "schema_version": 1,
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
        "profile_contracts": {
            profile_id: reference.profile_lock
            for profile_id, reference in sorted(profile_references.items())
        },
        "selection_policy": {
            "required_report_result": "success",
            "required_runtime_provenance_schema": 3,
            "required_git_dirty": False,
            "expected_environments": list(selected_environments),
            "expected_profiles": list(selected_profiles),
            "require_complete_matrix": require_complete_matrix,
        },
        "selection": {
            "included": [
                {
                    "path": record.path,
                    "sha256": record.sha256,
                    "canonical_sha256": record.canonical_sha256,
                    "environment_id": record.environment_id,
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
                f"{environment}::{profile}"
                for environment, profile in sorted(required_groups)
            ],
            "observed_groups": [
                f"{environment}::{profile}"
                for environment, profile in sorted(observed_groups)
            ],
            "missing_groups": [
                f"{environment}::{profile}" for environment, profile in missing_groups
            ],
        },
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
    parser.add_argument("reports", nargs="+", help="schema-3 motion report JSON files")
    parser.add_argument("--wheel-radius", required=True, type=float, help="wheel radius in metres")
    parser.add_argument("--output", required=True, help="strict atomic JSON output path")
    parser.add_argument("--min-repeats", type=_positive_integer, default=3)
    parser.add_argument("--expected-environment", action="append", default=[])
    parser.add_argument("--expected-profile", action="append", default=[])
    parser.add_argument(
        "--require-complete-matrix",
        action="store_true",
        help="require Warehouse/SimplePlane x all six shipped contact profiles",
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
