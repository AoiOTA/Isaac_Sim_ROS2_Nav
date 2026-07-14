"""Fit skid-steer effective track width from motion-baseline JSON reports.

The fitter intentionally keeps the two ordinary least-squares directions
separate.  Fitting yaw rate as the response and inverting its slope is not the
same estimator as fitting wheel differential directly as ``B * yaw_rate``.
Both estimates, plus a through-origin total-least-squares estimate, are
reported so downstream calibration decisions cannot hide that distinction.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

from .configuration import ConfigurationError
from .report import _atomic_text_write


_WHEEL_KEYS = ("front_left", "front_right", "rear_left", "rear_right")
_LEFT_WHEEL_KEYS = ("front_left", "rear_left")
_RIGHT_WHEEL_KEYS = ("front_right", "rear_right")
_ROTATION_MOTIONS = {"rotate_left": "left", "rotate_right": "right"}
_PROVENANCE_COMPONENT = re.compile(r"^[A-Za-z0-9_-]+$")


@dataclass(frozen=True)
class ProvenanceRequirement:
    """One exact JSON-value requirement under ``runtime_provenance``."""

    path: tuple[str, ...]
    expected: Any

    @property
    def dotted_path(self) -> str:
        """Return the stable dotted representation used by the CLI report."""
        return ".".join(self.path)


@dataclass(frozen=True)
class RotationSample:
    """One report segment reduced to the two variables used by the fit."""

    source_path: str
    source_sha256: str
    segment_id: str
    side: str
    tier: str
    wheel_differential_linear_mps: float
    measured_yaw_rate_radps: float


def _finite(value: Any, location: str) -> float:
    if isinstance(value, bool):
        raise ConfigurationError(f"{location} must be a finite number")
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise ConfigurationError(f"{location} must be a finite number") from exc
    if not math.isfinite(parsed):
        raise ConfigurationError(f"{location} must be a finite number")
    return parsed


def _positive(value: Any, location: str) -> float:
    parsed = _finite(value, location)
    if parsed <= 0.0:
        raise ConfigurationError(f"{location} must be positive")
    return parsed


def _mapping(value: Any, location: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ConfigurationError(f"{location} must be a mapping")
    return value


def _sequence(value: Any, location: str) -> Sequence[Any]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise ConfigurationError(f"{location} must be a sequence")
    return value


def _nonempty_string(value: Any, location: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ConfigurationError(f"{location} must be a non-empty string")
    return value.strip()


def _sha256(content: bytes) -> str:
    return hashlib.sha256(content).hexdigest()


def parse_provenance_requirement(expression: str) -> ProvenanceRequirement:
    """Parse ``PATH=JSON_VALUE`` without assuming a provenance schema."""
    if not isinstance(expression, str) or "=" not in expression:
        raise ConfigurationError(
            "provenance requirement must use PATH=JSON_VALUE"
        )
    raw_path, raw_expected = expression.split("=", 1)
    components = tuple(raw_path.split("."))
    if not components or any(
        not component or not _PROVENANCE_COMPONENT.fullmatch(component)
        for component in components
    ):
        raise ConfigurationError(
            f"invalid provenance requirement path: {raw_path!r}"
        )
    if not raw_expected:
        raise ConfigurationError(
            f"provenance requirement {raw_path!r} has an empty expected value"
        )
    try:
        expected = json.loads(raw_expected)
    except json.JSONDecodeError:
        expected = raw_expected
    if isinstance(expected, (dict, list)):
        raise ConfigurationError(
            "provenance expected value must be a JSON scalar"
        )
    return ProvenanceRequirement(components, expected)


def _json_values_equal(actual: Any, expected: Any) -> bool:
    if isinstance(actual, bool) or isinstance(expected, bool):
        return type(actual) is type(expected) and actual == expected
    if isinstance(actual, (int, float)) and isinstance(expected, (int, float)):
        return actual == expected
    return type(actual) is type(expected) and actual == expected


def _provenance_exclusions(
    report: Mapping[str, Any],
    requirements: Sequence[ProvenanceRequirement],
) -> list[dict[str, str]]:
    if not requirements:
        return []
    provenance = report.get("runtime_provenance")
    if not isinstance(provenance, Mapping):
        return [{
            "code": "runtime_provenance_missing",
            "detail": "runtime_provenance is not a mapping",
        }]
    exclusions: list[dict[str, str]] = []
    for requirement in requirements:
        current: Any = provenance
        missing = False
        for component in requirement.path:
            if not isinstance(current, Mapping) or component not in current:
                missing = True
                break
            current = current[component]
        if missing:
            exclusions.append({
                "code": "provenance_path_missing",
                "detail": requirement.dotted_path,
            })
        elif not _json_values_equal(current, requirement.expected):
            exclusions.append({
                "code": "provenance_value_mismatch",
                "detail": (
                    f"{requirement.dotted_path}: expected "
                    f"{requirement.expected!r}, observed {current!r}"
                ),
            })
    return exclusions


def _wheel_layout(report: Mapping[str, Any], location: str) -> dict[str, str]:
    configuration = _mapping(report.get("configuration"), f"{location}.configuration")
    wheels = _mapping(configuration.get("wheels"), f"{location}.configuration.wheels")
    layout = {
        key: _nonempty_string(
            wheels.get(key), f"{location}.configuration.wheels.{key}"
        )
        for key in _WHEEL_KEYS
    }
    if len(set(layout.values())) != len(_WHEEL_KEYS):
        raise ConfigurationError(
            f"{location}.configuration.wheels must name four unique joints"
        )
    return layout


def _wheel_mean(
    segment: Mapping[str, Any],
    joint_name: str,
    location: str,
) -> float:
    wheels = _mapping(segment.get("wheels"), f"{location}.wheels")
    per_wheel = _mapping(wheels.get("per_wheel"), f"{location}.wheels.per_wheel")
    joint = _mapping(
        per_wheel.get(joint_name),
        f"{location}.wheels.per_wheel.{joint_name}",
    )
    speeds = _mapping(
        joint.get("speed_radps"),
        f"{location}.wheels.per_wheel.{joint_name}.speed_radps",
    )
    return _finite(
        speeds.get("mean"),
        f"{location}.wheels.per_wheel.{joint_name}.speed_radps.mean",
    )


def _rotation_samples(
    report: Mapping[str, Any],
    source_path: str,
    source_sha256: str,
    wheel_radius_m: float,
) -> tuple[list[RotationSample], list[dict[str, Any]]]:
    location = f"report {source_path}"
    layout = _wheel_layout(report, location)
    segments = _sequence(report.get("segments"), f"{location}.segments")
    samples: list[RotationSample] = []
    excluded_segments: list[dict[str, Any]] = []
    seen_segment_ids: set[str] = set()
    for index, raw_segment in enumerate(segments):
        segment_location = f"{location}.segments[{index}]"
        segment = _mapping(raw_segment, segment_location)
        segment_id = _nonempty_string(
            segment.get("segment_id"), f"{segment_location}.segment_id"
        )
        if segment_id in seen_segment_ids:
            raise ConfigurationError(
                f"{location} contains duplicate segment_id {segment_id!r}"
            )
        seen_segment_ids.add(segment_id)
        motion = segment.get("motion")
        if motion not in _ROTATION_MOTIONS:
            excluded_segments.append({
                "index": index,
                "segment_id": segment_id,
                "motion": motion,
                "reason": "not_pure_rotation",
            })
            continue
        if segment.get("result") != "complete":
            raise ConfigurationError(
                f"{segment_location}.result must be 'complete' for pure rotation"
            )
        command = _mapping(segment.get("command"), f"{segment_location}.command")
        linear = _finite(
            command.get("linear_x_mps"),
            f"{segment_location}.command.linear_x_mps",
        )
        angular = _finite(
            command.get("angular_z_radps"),
            f"{segment_location}.command.angular_z_radps",
        )
        side = _ROTATION_MOTIONS[motion]
        if linear != 0.0:
            raise ConfigurationError(
                f"{segment_location} pure rotation must command zero linear speed"
            )
        if angular == 0.0 or (side == "left") != (angular > 0.0):
            raise ConfigurationError(
                f"{segment_location} command sign does not match {motion}"
            )
        tier = _nonempty_string(segment.get("tier"), f"{segment_location}.tier")
        velocity = _mapping(
            segment.get("actual_velocity"),
            f"{segment_location}.actual_velocity",
        )
        yaw_stats = _mapping(
            velocity.get("angular_z_radps"),
            f"{segment_location}.actual_velocity.angular_z_radps",
        )
        yaw_rate = _finite(
            yaw_stats.get("mean"),
            f"{segment_location}.actual_velocity.angular_z_radps.mean",
        )
        left_rate = sum(
            _wheel_mean(segment, layout[key], segment_location)
            for key in _LEFT_WHEEL_KEYS
        ) / len(_LEFT_WHEEL_KEYS)
        right_rate = sum(
            _wheel_mean(segment, layout[key], segment_location)
            for key in _RIGHT_WHEEL_KEYS
        ) / len(_RIGHT_WHEEL_KEYS)
        wheel_differential = wheel_radius_m * (right_rate - left_rate)
        if yaw_rate == 0.0:
            raise ConfigurationError(
                f"{segment_location} measured yaw rate must be non-zero"
            )
        if wheel_differential == 0.0:
            raise ConfigurationError(
                f"{segment_location} wheel differential must be non-zero"
            )
        expected_positive = side == "left"
        if (yaw_rate > 0.0) != expected_positive:
            raise ConfigurationError(
                f"{segment_location} measured yaw rate sign does not match {motion}"
            )
        if (wheel_differential > 0.0) != expected_positive:
            raise ConfigurationError(
                f"{segment_location} wheel differential sign does not match {motion}"
            )
        samples.append(RotationSample(
            source_path=source_path,
            source_sha256=source_sha256,
            segment_id=segment_id,
            side=side,
            tier=tier,
            wheel_differential_linear_mps=wheel_differential,
            measured_yaw_rate_radps=yaw_rate,
        ))
    if not samples:
        raise ConfigurationError(f"{location} has no complete pure-rotation segments")
    sides = {sample.side for sample in samples}
    if sides != {"left", "right"}:
        raise ConfigurationError(
            f"{location} pure-rotation data must include left and right turns"
        )
    return samples, excluded_segments


def _origin_r_squared(observed: Sequence[float], predicted: Sequence[float]) -> float:
    denominator = sum(value * value for value in observed)
    if denominator <= 0.0:
        raise ConfigurationError("origin R-squared requires non-zero observations")
    residual_sum = sum(
        (actual - estimate) ** 2
        for actual, estimate in zip(observed, predicted)
    )
    return 1.0 - residual_sum / denominator


def _fit(samples: Sequence[RotationSample], location: str) -> dict[str, Any]:
    if not samples:
        raise ConfigurationError(f"{location} has no samples")
    wheel_values = [
        sample.wheel_differential_linear_mps for sample in samples
    ]
    yaw_values = [sample.measured_yaw_rate_radps for sample in samples]
    wheel_sum_squares = sum(value * value for value in wheel_values)
    yaw_sum_squares = sum(value * value for value in yaw_values)
    cross_sum = sum(
        wheel * yaw for wheel, yaw in zip(wheel_values, yaw_values)
    )
    if wheel_sum_squares <= 0.0 or yaw_sum_squares <= 0.0:
        raise ConfigurationError(f"{location} has a zero fit denominator")
    if cross_sum <= 0.0:
        raise ConfigurationError(
            f"{location} has a non-positive wheel/yaw cross product"
        )

    yaw_slope = cross_sum / wheel_sum_squares
    yaw_predicted = [yaw_slope * value for value in wheel_values]
    yaw_residual_sum = sum(
        (actual - estimate) ** 2
        for actual, estimate in zip(yaw_values, yaw_predicted)
    )

    direct_track = cross_sum / yaw_sum_squares
    wheel_predicted = [direct_track * value for value in yaw_values]
    direct_residual_sum = sum(
        (actual - estimate) ** 2
        for actual, estimate in zip(wheel_values, wheel_predicted)
    )

    tls_track = (
        wheel_sum_squares
        - yaw_sum_squares
        + math.hypot(
            wheel_sum_squares - yaw_sum_squares,
            2.0 * cross_sum,
        )
    ) / (2.0 * cross_sum)
    tls_residual_sum = sum(
        (wheel - tls_track * yaw) ** 2 / (1.0 + tls_track * tls_track)
        for wheel, yaw in zip(wheel_values, yaw_values)
    )

    return {
        "sample_count": len(samples),
        "data_sums": {
            "wheel_differential_squared": wheel_sum_squares,
            "yaw_rate_squared": yaw_sum_squares,
            "wheel_differential_times_yaw_rate": cross_sum,
        },
        "yaw_response_ols": {
            "model": "yaw_rate_radps = slope_per_m * wheel_differential_linear_mps",
            "slope_per_m": yaw_slope,
            "effective_track_width_m": 1.0 / yaw_slope,
            "origin_r_squared": _origin_r_squared(
                yaw_values, yaw_predicted
            ),
            "residual_sum_squares": yaw_residual_sum,
        },
        "direct_ols": {
            "model": "wheel_differential_linear_mps = effective_track_width_m * yaw_rate_radps",
            "effective_track_width_m": direct_track,
            "origin_r_squared": _origin_r_squared(
                wheel_values, wheel_predicted
            ),
            "residual_sum_squares": direct_residual_sum,
        },
        "origin_tls": {
            "model": "wheel_differential_linear_mps = effective_track_width_m * yaw_rate_radps",
            "effective_track_width_m": tls_track,
            "orthogonal_residual_sum_squares": tls_residual_sum,
        },
    }


def _exclusion(code: str, detail: str) -> dict[str, str]:
    return {"code": code, "detail": detail}


def analyse_effective_track(
    report_paths: Sequence[str | Path],
    wheel_radius_m: float,
    *,
    provenance_requirements: Sequence[ProvenanceRequirement] = (),
    minimum_included_reports: int = 1,
) -> dict[str, Any]:
    """Validate, select, and fit multiple motion-baseline reports."""
    radius = _positive(wheel_radius_m, "wheel_radius_m")
    if (
        isinstance(minimum_included_reports, bool)
        or not isinstance(minimum_included_reports, int)
        or minimum_included_reports <= 0
    ):
        raise ConfigurationError("minimum_included_reports must be positive")
    if not report_paths:
        raise ConfigurationError("at least one motion report is required")

    resolved_paths = [Path(path).expanduser().resolve() for path in report_paths]
    if len(set(resolved_paths)) != len(resolved_paths):
        raise ConfigurationError("motion report paths must be unique")

    included: list[dict[str, Any]] = []
    excluded: list[dict[str, Any]] = []
    all_samples: list[RotationSample] = []
    seen_hashes: dict[str, Path] = {}
    for source in resolved_paths:
        if not source.is_file():
            raise FileNotFoundError(f"motion report does not exist: {source}")
        content = source.read_bytes()
        digest = _sha256(content)
        if digest in seen_hashes:
            raise ConfigurationError(
                "motion report content must be unique; "
                f"{source} duplicates {seen_hashes[digest]}"
            )
        seen_hashes[digest] = source
        base_record: dict[str, Any] = {
            "path": str(source),
            "sha256": digest,
        }
        try:
            parsed = json.loads(content)
            report = _mapping(parsed, f"report {source}")
        except (UnicodeDecodeError, json.JSONDecodeError, ConfigurationError) as exc:
            excluded.append({
                **base_record,
                "reasons": [_exclusion("invalid_json_report", str(exc))],
            })
            continue
        if report.get("result") != "success":
            excluded.append({
                **base_record,
                "reasons": [_exclusion(
                    "report_result_not_success",
                    f"observed {report.get('result')!r}",
                )],
            })
            continue
        provenance_exclusions = _provenance_exclusions(
            report, provenance_requirements
        )
        if provenance_exclusions:
            excluded.append({
                **base_record,
                "reasons": provenance_exclusions,
            })
            continue
        try:
            report_samples, excluded_segments = _rotation_samples(
                report, str(source), digest, radius
            )
        except ConfigurationError as exc:
            excluded.append({
                **base_record,
                "reasons": [_exclusion("invalid_rotation_data", str(exc))],
            })
            continue
        all_samples.extend(report_samples)
        included.append({
            **base_record,
            "included_rotation_segments": [
                {
                    "segment_id": sample.segment_id,
                    "side": sample.side,
                    "tier": sample.tier,
                }
                for sample in report_samples
            ],
            "excluded_segments": excluded_segments,
        })

    if len(included) < minimum_included_reports:
        raise ConfigurationError(
            "included report count "
            f"{len(included)} is below required minimum "
            f"{minimum_included_reports}"
        )
    sides = {sample.side for sample in all_samples}
    if sides != {"left", "right"}:
        raise ConfigurationError(
            "aggregate pure-rotation data must include left and right turns"
        )

    tiers = sorted({sample.tier for sample in all_samples})
    report_fits = []
    for input_record in included:
        path = input_record["path"]
        samples = [sample for sample in all_samples if sample.source_path == path]
        report_fits.append({
            "path": path,
            "sha256": input_record["sha256"],
            "fit": _fit(samples, f"report {path}"),
        })

    return {
        "schema_version": 1,
        "report_type": "effective_track_width_analysis",
        "analysis_valid": True,
        "method": {
            "wheel_radius_m": radius,
            "side_wheel_rate_aggregation": "arithmetic mean of front and rear wheel means",
            "wheel_differential_definition": (
                "wheel_radius_m * (right_rate_radps - left_rate_radps)"
            ),
            "fit_intercept": 0.0,
            "yaw_response_note": (
                "yaw_response_ols fits yaw rate as the response and reports "
                "the reciprocal slope as effective track width"
            ),
        },
        "selection_policy": {
            "required_report_result": "success",
            "required_rotation_segment_result": "complete",
            "minimum_included_reports": minimum_included_reports,
            "provenance_requirements": [
                {
                    "path": requirement.dotted_path,
                    "expected": requirement.expected,
                }
                for requirement in provenance_requirements
            ],
        },
        "selection": {
            "included": included,
            "excluded": excluded,
        },
        "counts": {
            "input_reports": len(resolved_paths),
            "included_reports": len(included),
            "excluded_reports": len(excluded),
            "included_rotation_segments": len(all_samples),
            "excluded_non_rotation_segments": sum(
                len(record["excluded_segments"]) for record in included
            ),
        },
        "fits": {
            "overall": _fit(all_samples, "overall"),
            "by_side": {
                side: _fit(
                    [sample for sample in all_samples if sample.side == side],
                    f"side {side}",
                )
                for side in ("left", "right")
            },
            "by_tier": {
                tier: _fit(
                    [sample for sample in all_samples if sample.tier == tier],
                    f"tier {tier}",
                )
                for tier in tiers
            },
            "by_report": report_fits,
        },
    }


def write_effective_track_report(
    report: Mapping[str, Any], output_path: str | Path
) -> Path:
    """Atomically write one strict JSON effective-track report."""
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


def _argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "reports",
        nargs="+",
        help="motion-baseline JSON reports",
    )
    parser.add_argument(
        "--wheel-radius",
        required=True,
        type=float,
        help="wheel radius in metres used for every input report",
    )
    parser.add_argument("--output", required=True, help="strict JSON output path")
    parser.add_argument(
        "--require-provenance",
        action="append",
        default=[],
        metavar="PATH=JSON_VALUE",
        help=(
            "require an exact value below runtime_provenance; repeatable and "
            "schema-agnostic"
        ),
    )
    parser.add_argument(
        "--min-included-reports",
        type=_positive_integer,
        default=1,
        help="minimum valid reports required after selection",
    )
    parser.add_argument(
        "--fail-on-excluded",
        action="store_true",
        help="write the audit report but return status 2 if any input is excluded",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Run the installed offline CLI."""
    arguments = _argument_parser().parse_args(argv)
    try:
        requirements = [
            parse_provenance_requirement(value)
            for value in arguments.require_provenance
        ]
        report = analyse_effective_track(
            arguments.reports,
            arguments.wheel_radius,
            provenance_requirements=requirements,
            minimum_included_reports=arguments.min_included_reports,
        )
        output = write_effective_track_report(report, arguments.output)
    except (ConfigurationError, OSError, ValueError) as exc:
        print(f"effective track analysis failed: {exc}", file=sys.stderr)
        return 1
    print(output)
    if arguments.fail_on_excluded and report["counts"]["excluded_reports"]:
        print("effective track analysis excluded one or more inputs", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
