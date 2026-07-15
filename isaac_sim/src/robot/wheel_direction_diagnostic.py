"""Pure configuration, math, analysis, and reporting for wheel diagnostics.

The runtime adapter lives in :mod:`isaac_sim.apps.wheel_direction_diagnostic`.
Keeping this module free of Isaac/Omniverse imports makes the direction gates
cheap to unit test with the normal project test runner.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
import os
from pathlib import Path
import statistics
import tempfile
from typing import Any, Iterable, Mapping, Sequence

from isaac_sim.src.yaml_utils import (
    load_mapping,
    reject_unknown,
    require_keys,
    require_number,
)


class WheelDirectionDiagnosticError(RuntimeError):
    """Raised when a diagnostic contract or report is invalid."""


@dataclass(frozen=True)
class WheelDirectionProtocol:
    command_rad_s: float
    settle_steps: int
    baseline_steps: int
    drive_steps: int
    recovery_steps: int
    contact_ready_consecutive_steps: int
    contact_ready_timeout_steps: int
    max_contact_count: int


@dataclass(frozen=True)
class WheelDirectionThresholds:
    target_tolerance_rad_s: float
    active_rate_min_rad_s: float
    active_rate_sign_fraction_min: float
    inactive_rate_p95_advisory_max_rad_s: float
    contact_coverage_min: float
    normal_force_median_min_n: float
    spin_velocity_opposition_min_m_s: float
    friction_force_signed_median_min_n: float
    friction_impulse_signed_min_ns: float
    body_delta_velocity_signed_min_m_s: float
    body_displacement_signed_min_m: float
    normal_force_consistency_abs_tolerance_n: float
    normal_force_consistency_relative_tolerance: float
    normal_force_consistency_fraction_min: float
    symmetry_ratio_min: float
    symmetry_ratio_max: float


@dataclass(frozen=True)
class WheelDirectionConfig:
    environment_id: str
    ground_collision_prim: str
    protocol: WheelDirectionProtocol
    thresholds: WheelDirectionThresholds


@dataclass(frozen=True)
class TrialObservation:
    """One physics-step observation reduced to direction-relevant scalars."""

    phase: str
    step_index: int
    simulation_time_s: float
    joint_targets_rad_s: tuple[float, float, float, float]
    joint_rates_rad_s: tuple[float, float, float, float]
    active_contact_count: int
    active_normal_force_n: float
    active_spin_velocity_x_m_s: float | None
    active_surface_velocity_x_m_s: float | None
    active_friction_force_x_n: float
    normal_force_consistency_error_n: float | None
    base_position_x_m: float
    base_velocity_x_m_s: float
    base_acceleration_x_m_s2: float | None

    def to_json(self) -> dict[str, object]:
        return asdict(self)


def _positive_int(value: Any, context: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise WheelDirectionDiagnosticError(f"{context} must be a positive integer")
    return value


def _unit_interval(value: Any, context: str) -> float:
    result = require_number(value, context=context)
    if not 0.0 <= result <= 1.0:
        raise WheelDirectionDiagnosticError(f"{context} must be in [0, 1]")
    return result


def load_wheel_direction_config(path: str | Path) -> WheelDirectionConfig:
    """Load a strict, fail-closed wheel direction diagnostic configuration."""

    data = load_mapping(path)
    allowed = {
        "schema_version",
        "environment_id",
        "ground_collision_prim",
        "protocol",
        "thresholds",
    }
    reject_unknown(data, allowed, context="wheel direction diagnostic")
    require_keys(data, allowed, context="wheel direction diagnostic")
    if data["schema_version"] != 1:
        raise WheelDirectionDiagnosticError(
            "wheel direction diagnostic schema_version must be 1"
        )
    environment_id = data["environment_id"]
    if (
        not isinstance(environment_id, str)
        or not environment_id
        or any(character not in "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_.-" for character in environment_id)
        or not environment_id[0].isalnum()
    ):
        raise WheelDirectionDiagnosticError(
            "wheel direction diagnostic environment_id must be path-safe"
        )
    ground_collision_prim = data["ground_collision_prim"]
    if (
        not isinstance(ground_collision_prim, str)
        or not ground_collision_prim.startswith("/")
        or "//" in ground_collision_prim
    ):
        raise WheelDirectionDiagnosticError(
            "ground_collision_prim must be an absolute USD prim path"
        )

    raw_protocol = data["protocol"]
    if not isinstance(raw_protocol, dict):
        raise WheelDirectionDiagnosticError("protocol must be a mapping")
    protocol_fields = {
        "command_rad_s",
        "settle_steps",
        "baseline_steps",
        "drive_steps",
        "recovery_steps",
        "contact_ready_consecutive_steps",
        "contact_ready_timeout_steps",
        "max_contact_count",
    }
    reject_unknown(raw_protocol, protocol_fields, context="wheel direction protocol")
    require_keys(raw_protocol, protocol_fields, context="wheel direction protocol")
    protocol = WheelDirectionProtocol(
        command_rad_s=require_number(
            raw_protocol["command_rad_s"],
            context="protocol.command_rad_s",
            positive=True,
        ),
        settle_steps=_positive_int(raw_protocol["settle_steps"], "protocol.settle_steps"),
        baseline_steps=_positive_int(raw_protocol["baseline_steps"], "protocol.baseline_steps"),
        drive_steps=_positive_int(raw_protocol["drive_steps"], "protocol.drive_steps"),
        recovery_steps=_positive_int(raw_protocol["recovery_steps"], "protocol.recovery_steps"),
        contact_ready_consecutive_steps=_positive_int(
            raw_protocol["contact_ready_consecutive_steps"],
            "protocol.contact_ready_consecutive_steps",
        ),
        contact_ready_timeout_steps=_positive_int(
            raw_protocol["contact_ready_timeout_steps"],
            "protocol.contact_ready_timeout_steps",
        ),
        max_contact_count=_positive_int(
            raw_protocol["max_contact_count"], "protocol.max_contact_count"
        ),
    )
    if protocol.contact_ready_timeout_steps < protocol.contact_ready_consecutive_steps:
        raise WheelDirectionDiagnosticError(
            "contact_ready_timeout_steps must be at least contact_ready_consecutive_steps"
        )
    if protocol.max_contact_count < 4:
        raise WheelDirectionDiagnosticError("max_contact_count must be at least 4")

    raw_thresholds = data["thresholds"]
    if not isinstance(raw_thresholds, dict):
        raise WheelDirectionDiagnosticError("thresholds must be a mapping")
    threshold_fields = {
        "target_tolerance_rad_s",
        "active_rate_min_rad_s",
        "active_rate_sign_fraction_min",
        "inactive_rate_p95_advisory_max_rad_s",
        "contact_coverage_min",
        "normal_force_median_min_n",
        "spin_velocity_opposition_min_m_s",
        "friction_force_signed_median_min_n",
        "friction_impulse_signed_min_ns",
        "body_delta_velocity_signed_min_m_s",
        "body_displacement_signed_min_m",
        "normal_force_consistency_abs_tolerance_n",
        "normal_force_consistency_relative_tolerance",
        "normal_force_consistency_fraction_min",
        "symmetry_ratio_min",
        "symmetry_ratio_max",
    }
    reject_unknown(
        raw_thresholds, threshold_fields, context="wheel direction thresholds"
    )
    require_keys(
        raw_thresholds, threshold_fields, context="wheel direction thresholds"
    )

    def positive(name: str) -> float:
        return require_number(
            raw_thresholds[name], context=f"thresholds.{name}", positive=True
        )

    thresholds = WheelDirectionThresholds(
        target_tolerance_rad_s=positive("target_tolerance_rad_s"),
        active_rate_min_rad_s=positive("active_rate_min_rad_s"),
        active_rate_sign_fraction_min=_unit_interval(
            raw_thresholds["active_rate_sign_fraction_min"],
            "thresholds.active_rate_sign_fraction_min",
        ),
        inactive_rate_p95_advisory_max_rad_s=positive(
            "inactive_rate_p95_advisory_max_rad_s"
        ),
        contact_coverage_min=_unit_interval(
            raw_thresholds["contact_coverage_min"],
            "thresholds.contact_coverage_min",
        ),
        normal_force_median_min_n=positive("normal_force_median_min_n"),
        spin_velocity_opposition_min_m_s=positive(
            "spin_velocity_opposition_min_m_s"
        ),
        friction_force_signed_median_min_n=positive(
            "friction_force_signed_median_min_n"
        ),
        friction_impulse_signed_min_ns=positive(
            "friction_impulse_signed_min_ns"
        ),
        body_delta_velocity_signed_min_m_s=positive(
            "body_delta_velocity_signed_min_m_s"
        ),
        body_displacement_signed_min_m=positive(
            "body_displacement_signed_min_m"
        ),
        normal_force_consistency_abs_tolerance_n=positive(
            "normal_force_consistency_abs_tolerance_n"
        ),
        normal_force_consistency_relative_tolerance=positive(
            "normal_force_consistency_relative_tolerance"
        ),
        normal_force_consistency_fraction_min=_unit_interval(
            raw_thresholds["normal_force_consistency_fraction_min"],
            "thresholds.normal_force_consistency_fraction_min",
        ),
        symmetry_ratio_min=positive("symmetry_ratio_min"),
        symmetry_ratio_max=positive("symmetry_ratio_max"),
    )
    if thresholds.symmetry_ratio_min > thresholds.symmetry_ratio_max:
        raise WheelDirectionDiagnosticError(
            "thresholds.symmetry_ratio_min must not exceed symmetry_ratio_max"
        )
    return WheelDirectionConfig(
        environment_id=environment_id,
        ground_collision_prim=ground_collision_prim,
        protocol=protocol,
        thresholds=thresholds,
    )


Vector3 = tuple[float, float, float]
QuaternionWxyz = tuple[float, float, float, float]


def _vector3(value: Sequence[float], context: str) -> Vector3:
    if len(value) != 3:
        raise WheelDirectionDiagnosticError(f"{context} must have three elements")
    result = tuple(float(component) for component in value)
    if not all(math.isfinite(component) for component in result):
        raise WheelDirectionDiagnosticError(f"{context} must be finite")
    return result  # type: ignore[return-value]


def _quaternion(value: Sequence[float], context: str) -> QuaternionWxyz:
    if len(value) != 4:
        raise WheelDirectionDiagnosticError(f"{context} must have four elements")
    raw = tuple(float(component) for component in value)
    norm = math.sqrt(sum(component * component for component in raw))
    if not math.isfinite(norm) or norm <= 1e-12:
        raise WheelDirectionDiagnosticError(f"{context} must be a finite quaternion")
    return tuple(component / norm for component in raw)  # type: ignore[return-value]


def vector_add(left: Sequence[float], right: Sequence[float]) -> Vector3:
    a = _vector3(left, "left vector")
    b = _vector3(right, "right vector")
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def vector_subtract(left: Sequence[float], right: Sequence[float]) -> Vector3:
    a = _vector3(left, "left vector")
    b = _vector3(right, "right vector")
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def cross(left: Sequence[float], right: Sequence[float]) -> Vector3:
    a = _vector3(left, "left vector")
    b = _vector3(right, "right vector")
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def rotate_local_to_world(
    vector: Sequence[float], orientation_world_wxyz: Sequence[float]
) -> Vector3:
    """Rotate a vector using a normalized ``wxyz`` quaternion."""

    x, y, z = _vector3(vector, "vector")
    w, qx, qy, qz = _quaternion(
        orientation_world_wxyz, "orientation_world_wxyz"
    )
    # Equivalent to q * [0, v] * conjugate(q), expanded to avoid dependencies.
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + w * tx + (qy * tz - qz * ty),
        y + w * ty + (qz * tx - qx * tz),
        z + w * tz + (qx * ty - qy * tx),
    )


def rotate_world_to_local(
    vector: Sequence[float], orientation_world_wxyz: Sequence[float]
) -> Vector3:
    w, x, y, z = _quaternion(
        orientation_world_wxyz, "orientation_world_wxyz"
    )
    return rotate_local_to_world(vector, (w, -x, -y, -z))


def center_of_mass_world(
    prim_position_world: Sequence[float],
    prim_orientation_world_wxyz: Sequence[float],
    center_of_mass_position_prim: Sequence[float],
) -> Vector3:
    """Transform a prim-local PhysX center of mass into the world frame."""

    return vector_add(
        prim_position_world,
        rotate_local_to_world(
            center_of_mass_position_prim, prim_orientation_world_wxyz
        ),
    )


def contact_point_velocity_world(
    center_of_mass_linear_velocity_world: Sequence[float],
    angular_velocity_world_rad_s: Sequence[float],
    contact_point_world: Sequence[float],
    center_of_mass_position_world: Sequence[float],
) -> Vector3:
    """Return rigid-body surface velocity at one world-space contact point."""

    radius = vector_subtract(contact_point_world, center_of_mass_position_world)
    return vector_add(
        center_of_mass_linear_velocity_world,
        cross(angular_velocity_world_rad_s, radius),
    )


def spin_contact_velocity_world(
    wheel_angular_velocity_world_rad_s: Sequence[float],
    base_angular_velocity_world_rad_s: Sequence[float],
    contact_point_world: Sequence[float],
    wheel_center_of_mass_world: Sequence[float],
) -> Vector3:
    """Return only wheel spin's surface velocity, excluding chassis rotation."""

    relative_angular_velocity = vector_subtract(
        wheel_angular_velocity_world_rad_s, base_angular_velocity_world_rad_s
    )
    radius = vector_subtract(contact_point_world, wheel_center_of_mass_world)
    return cross(relative_angular_velocity, radius)


def _median(values: Iterable[float]) -> float:
    collected = [float(value) for value in values]
    if not collected or not all(math.isfinite(value) for value in collected):
        raise WheelDirectionDiagnosticError("metric requires finite observations")
    return float(statistics.median(collected))


def _p95(values: Iterable[float]) -> float:
    collected = sorted(float(value) for value in values)
    if not collected or not all(math.isfinite(value) for value in collected):
        raise WheelDirectionDiagnosticError("p95 metric requires finite observations")
    index = max(0, math.ceil(0.95 * len(collected)) - 1)
    return collected[index]


def summarize_trial(
    observations: Sequence[TrialObservation],
    *,
    wheel_name: str,
    wheel_index: int,
    command_rad_s: float,
    physics_dt_s: float,
    thresholds: WheelDirectionThresholds,
) -> dict[str, object]:
    """Reduce one +/- single-wheel trial and evaluate every hard gate."""

    if wheel_index not in range(4):
        raise WheelDirectionDiagnosticError("wheel_index must be in [0, 3]")
    if not math.isfinite(command_rad_s) or command_rad_s == 0.0:
        raise WheelDirectionDiagnosticError("command_rad_s must be finite and non-zero")
    if not math.isfinite(physics_dt_s) or physics_dt_s <= 0.0:
        raise WheelDirectionDiagnosticError("physics_dt_s must be positive")
    baseline = [sample for sample in observations if sample.phase == "baseline"]
    drive = [sample for sample in observations if sample.phase == "drive"]
    if not baseline or not drive:
        raise WheelDirectionDiagnosticError(
            "trial requires both baseline and drive observations"
        )
    if any(len(sample.joint_rates_rad_s) != 4 for sample in observations):
        raise WheelDirectionDiagnosticError("trial joint rate vectors must have four values")

    sign = 1.0 if command_rad_s > 0.0 else -1.0
    expected_targets = [0.0, 0.0, 0.0, 0.0]
    expected_targets[wheel_index] = command_rad_s
    target_max_error = max(
        abs(sample.joint_targets_rad_s[index] - expected_targets[index])
        for sample in drive
        for index in range(4)
    )
    active_signed_rates = [sign * sample.joint_rates_rad_s[wheel_index] for sample in drive]
    active_rate_median = _median(active_signed_rates)
    active_rate_sign_fraction = sum(value > 0.0 for value in active_signed_rates) / len(drive)
    inactive_rate_p95 = max(
        _p95(abs(sample.joint_rates_rad_s[index]) for sample in drive)
        for index in range(4)
        if index != wheel_index
    )
    inactive_to_active_rate_ratio = inactive_rate_p95 / max(
        abs(active_rate_median), 1e-12
    )
    contact_coverage = sum(sample.active_contact_count > 0 for sample in drive) / len(drive)
    normal_force_median = _median(sample.active_normal_force_n for sample in drive)
    spin_values = [
        sample.active_spin_velocity_x_m_s
        for sample in drive
        if sample.active_spin_velocity_x_m_s is not None
    ]
    spin_opposition_median = (
        _median(-sign * value for value in spin_values) if spin_values else 0.0
    )
    surface_values = [
        sample.active_surface_velocity_x_m_s
        for sample in drive
        if sample.active_surface_velocity_x_m_s is not None
    ]
    surface_velocity_median = _median(surface_values) if surface_values else None
    friction_signed_values = [
        sign * sample.active_friction_force_x_n for sample in drive
    ]
    friction_force_signed_median = _median(friction_signed_values)
    friction_impulse_signed = sum(friction_signed_values) * physics_dt_s
    baseline_velocity = _median(sample.base_velocity_x_m_s for sample in baseline)
    baseline_position = _median(sample.base_position_x_m for sample in baseline)
    tail_count = min(3, len(drive))
    final_velocity = _median(sample.base_velocity_x_m_s for sample in drive[-tail_count:])
    final_position = _median(sample.base_position_x_m for sample in drive[-tail_count:])
    body_delta_velocity_signed = sign * (final_velocity - baseline_velocity)
    body_displacement_signed = sign * (final_position - baseline_position)
    acceleration_values = [
        sign * sample.base_acceleration_x_m_s2
        for sample in drive
        if sample.base_acceleration_x_m_s2 is not None
    ]
    body_acceleration_signed_median = (
        _median(acceleration_values) if acceleration_values else None
    )
    consistency_samples = [
        sample
        for sample in drive
        if sample.normal_force_consistency_error_n is not None
    ]
    consistent_count = 0
    for sample in consistency_samples:
        tolerance = max(
            thresholds.normal_force_consistency_abs_tolerance_n,
            thresholds.normal_force_consistency_relative_tolerance
            * abs(sample.active_normal_force_n),
        )
        if (
            sample.normal_force_consistency_error_n is not None
            and sample.normal_force_consistency_error_n <= tolerance
        ):
            consistent_count += 1
    normal_force_consistency_fraction = (
        consistent_count / len(drive) if drive else 0.0
    )

    gates = {
        "target_readback": target_max_error <= thresholds.target_tolerance_rad_s,
        "active_rate": active_rate_median >= thresholds.active_rate_min_rad_s,
        "active_rate_sign": (
            active_rate_sign_fraction >= thresholds.active_rate_sign_fraction_min
        ),
        "ground_contact": contact_coverage >= thresholds.contact_coverage_min,
        "normal_force": normal_force_median >= thresholds.normal_force_median_min_n,
        "spin_opposes_forward": (
            spin_opposition_median
            >= thresholds.spin_velocity_opposition_min_m_s
        ),
        "friction_force_direction": (
            friction_force_signed_median
            >= thresholds.friction_force_signed_median_min_n
        ),
        "friction_impulse_direction": (
            friction_impulse_signed >= thresholds.friction_impulse_signed_min_ns
        ),
        "body_motion_direction": (
            body_delta_velocity_signed
            >= thresholds.body_delta_velocity_signed_min_m_s
            or body_displacement_signed
            >= thresholds.body_displacement_signed_min_m
        ),
        "normal_force_api_consistency": (
            normal_force_consistency_fraction
            >= thresholds.normal_force_consistency_fraction_min
        ),
    }
    inactive_within_advisory_limit = (
        inactive_rate_p95
        <= thresholds.inactive_rate_p95_advisory_max_rad_s
    )
    advisories: dict[str, object] = {
        "inactive_wheel_motion": {
            "within_advisory_limit": inactive_within_advisory_limit,
            "p95_max_rad_s": inactive_rate_p95,
            "threshold_rad_s": (
                thresholds.inactive_rate_p95_advisory_max_rad_s
            ),
            "ratio_to_active_rate_median": inactive_to_active_rate_ratio,
            "interpretation": (
                "zero target is the hard command-isolation contract; actual "
                "free-wheel motion can be induced by chassis/ground coupling"
            ),
        }
    }
    warnings = []
    if not inactive_within_advisory_limit:
        warnings.append(
            "inactive free-wheel p95 rate "
            f"{inactive_rate_p95:.6g} rad/s exceeds advisory limit "
            f"{thresholds.inactive_rate_p95_advisory_max_rad_s:.6g} rad/s "
            f"(ratio to active median={inactive_to_active_rate_ratio:.6g})"
        )
    metrics: dict[str, object] = {
        "target_max_error_rad_s": target_max_error,
        "active_rate_signed_median_rad_s": active_rate_median,
        "active_rate_sign_fraction": active_rate_sign_fraction,
        "inactive_rate_p95_max_rad_s": inactive_rate_p95,
        "inactive_to_active_rate_median_ratio": (
            inactive_to_active_rate_ratio
        ),
        "contact_coverage": contact_coverage,
        "normal_force_median_n": normal_force_median,
        "spin_velocity_opposition_median_m_s": spin_opposition_median,
        "surface_velocity_x_median_m_s": surface_velocity_median,
        "friction_force_signed_median_n": friction_force_signed_median,
        "friction_impulse_signed_ns": friction_impulse_signed,
        "body_delta_velocity_signed_m_s": body_delta_velocity_signed,
        "body_displacement_signed_m": body_displacement_signed,
        "body_acceleration_signed_median_m_s2": body_acceleration_signed_median,
        "normal_force_consistency_fraction": (
            normal_force_consistency_fraction
        ),
    }
    return {
        "wheel": wheel_name,
        "wheel_index": wheel_index,
        "command_rad_s": command_rad_s,
        "expected_body_direction": "+X" if command_rad_s > 0.0 else "-X",
        "metrics": metrics,
        "gates": gates,
        "advisories": advisories,
        "warnings": warnings,
        "passed": all(gates.values()),
    }


def evaluate_trial_set(
    summaries: Sequence[Mapping[str, object]],
    *,
    wheel_order: Sequence[str],
    thresholds: WheelDirectionThresholds,
) -> dict[str, object]:
    """Require exactly one positive and negative passing trial per wheel."""

    if len(wheel_order) != 4 or len(set(wheel_order)) != 4:
        raise WheelDirectionDiagnosticError("wheel_order must contain four unique names")
    indexed: dict[tuple[str, int], Mapping[str, object]] = {}
    for summary in summaries:
        wheel = summary.get("wheel")
        command = summary.get("command_rad_s")
        if not isinstance(wheel, str) or wheel not in wheel_order:
            raise WheelDirectionDiagnosticError(f"unknown trial wheel {wheel!r}")
        if isinstance(command, bool) or not isinstance(command, (int, float)) or command == 0:
            raise WheelDirectionDiagnosticError("trial command_rad_s must be non-zero")
        key = (wheel, 1 if command > 0 else -1)
        if key in indexed:
            raise WheelDirectionDiagnosticError(f"duplicate trial for {key}")
        indexed[key] = summary
    expected = {(wheel, sign) for wheel in wheel_order for sign in (-1, 1)}
    missing = sorted(expected - set(indexed))
    unknown = sorted(set(indexed) - expected)
    if missing or unknown:
        raise WheelDirectionDiagnosticError(
            f"trial matrix mismatch: missing={missing}, unknown={unknown}"
        )

    warnings: list[str] = []
    symmetry: dict[str, object] = {}
    for wheel in wheel_order:
        for sign, label in ((1, "+"), (-1, "-")):
            trial_warnings = indexed[(wheel, sign)].get("warnings", [])
            if not isinstance(trial_warnings, list) or not all(
                isinstance(value, str) for value in trial_warnings
            ):
                raise WheelDirectionDiagnosticError(
                    "trial warnings must be a list of strings"
                )
            warnings.extend(
                f"{wheel}:{label}: {value}" for value in trial_warnings
            )
        positive_metrics = indexed[(wheel, 1)].get("metrics")
        negative_metrics = indexed[(wheel, -1)].get("metrics")
        if not isinstance(positive_metrics, Mapping) or not isinstance(negative_metrics, Mapping):
            raise WheelDirectionDiagnosticError("trial metrics must be mappings")
        positive = float(positive_metrics["friction_force_signed_median_n"])
        negative = float(negative_metrics["friction_force_signed_median_n"])
        ratio = abs(positive) / max(abs(negative), 1e-12)
        in_range = thresholds.symmetry_ratio_min <= ratio <= thresholds.symmetry_ratio_max
        symmetry[wheel] = {
            "positive_to_negative_friction_magnitude_ratio": ratio,
            "within_advisory_range": in_range,
        }
        if not in_range:
            warnings.append(
                f"{wheel} +/- friction magnitude ratio {ratio:.6g} is outside "
                f"[{thresholds.symmetry_ratio_min}, {thresholds.symmetry_ratio_max}]"
            )
    failed_trials = [
        f"{wheel}:{'+' if sign > 0 else '-'}"
        for (wheel, sign), summary in indexed.items()
        if summary.get("passed") is not True
    ]
    return {
        "matrix_complete": True,
        "positive_commands_correspond_to_body_positive_x": not failed_trials,
        "left_right_front_rear_signs_consistent": not failed_trials,
        "failed_trials": sorted(failed_trials),
        "symmetry_advisory": symmetry,
        "warnings": warnings,
        "passed": not failed_trials,
    }


def _validate_json_value(value: Any, location: str = "report") -> None:
    if value is None or isinstance(value, (str, bool, int)):
        return
    if isinstance(value, float):
        if not math.isfinite(value):
            raise WheelDirectionDiagnosticError(
                f"{location} contains NaN or infinity"
            )
        return
    if isinstance(value, Mapping):
        for key, child in value.items():
            if not isinstance(key, str):
                raise WheelDirectionDiagnosticError(
                    f"{location} contains a non-string key"
                )
            _validate_json_value(child, f"{location}.{key}")
        return
    if isinstance(value, (list, tuple)):
        for index, child in enumerate(value):
            _validate_json_value(child, f"{location}[{index}]")
        return
    raise WheelDirectionDiagnosticError(
        f"{location} contains unsupported {type(value).__name__}"
    )


def write_json_atomic(path: str | Path, report: Mapping[str, object]) -> None:
    """Atomically publish strict JSON without ever exposing a partial report."""

    _validate_json_value(report)
    destination = Path(path).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{destination.name}.", suffix=".tmp", dir=destination.parent
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, destination)
    except Exception:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise
