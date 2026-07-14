"""Tests for reproducible offline effective-track fitting."""

import hashlib
import json
import math
from pathlib import Path

import pytest

from robot_experiments.configuration import ConfigurationError
from robot_experiments.effective_track_analysis import (
    analyse_effective_track,
    main,
    parse_provenance_requirement,
)


RADIUS = 0.1
WHEEL_NAMES = {
    "front_left": "front_left_wheel_joint",
    "front_right": "front_right_wheel_joint",
    "rear_left": "rear_left_wheel_joint",
    "rear_right": "rear_right_wheel_joint",
}


def _rotation(segment_id, side, tier, wheel_differential, yaw_rate):
    half_rate = wheel_differential / (2.0 * RADIUS)
    left_rate = -half_rate
    right_rate = half_rate
    angular_command = 0.5 if side == "left" else -0.5
    return {
        "segment_id": segment_id,
        "motion": f"rotate_{side}",
        "tier": tier,
        "result": "complete",
        "command": {
            "linear_x_mps": 0.0,
            "angular_z_radps": angular_command,
        },
        "actual_velocity": {
            "angular_z_radps": {"mean": yaw_rate},
        },
        "wheels": {
            "per_wheel": {
                WHEEL_NAMES["front_left"]: {
                    "speed_radps": {"mean": left_rate}
                },
                WHEEL_NAMES["rear_left"]: {
                    "speed_radps": {"mean": left_rate}
                },
                WHEEL_NAMES["front_right"]: {
                    "speed_radps": {"mean": right_rate}
                },
                WHEEL_NAMES["rear_right"]: {
                    "speed_radps": {"mean": right_rate}
                },
            }
        },
    }


def _report(*, result="success", provenance=None, samples=None):
    if samples is None:
        samples = [
            ("left_low", "left", "low", 0.1, 0.08),
            ("right_low", "right", "low", -0.1, -0.09),
            ("left_nominal", "left", "nominal", 0.2, 0.19),
            ("right_nominal", "right", "nominal", -0.2, -0.18),
            ("left_high", "left", "high", 0.3, 0.31),
            ("right_high", "right", "high", -0.3, -0.28),
        ]
    return {
        "result": result,
        "configuration": {"wheels": WHEEL_NAMES},
        "runtime_provenance": provenance
        if provenance is not None
        else {
            "verified": True,
            "robot": {
                "solver": {
                    "position_iterations": 32,
                    "velocity_iterations": 4,
                }
            },
        },
        "segments": [
            {
                "segment_id": "forward_low",
                "motion": "forward",
                "result": "complete",
            },
            *(_rotation(*sample) for sample in samples),
        ],
    }


def _write_report(path: Path, document) -> Path:
    path.write_text(json.dumps(document, sort_keys=True), encoding="utf-8")
    return path


def _reverse_left_rotation_yaw(document):
    document["segments"][1]["actual_velocity"]["angular_z_radps"]["mean"] *= -1


def _reverse_left_rotation_wheel_differential(document):
    per_wheel = document["segments"][1]["wheels"]["per_wheel"]
    for joint in per_wheel.values():
        joint["speed_radps"]["mean"] *= -1


def test_three_origin_fits_and_all_groupings_are_reported(tmp_path):
    """All estimators and requested grouping axes are explicit."""
    source = _write_report(tmp_path / "motion.json", _report())
    report = analyse_effective_track([source], RADIUS)

    x_values = [0.1, -0.1, 0.2, -0.2, 0.3, -0.3]
    yaw_values = [0.08, -0.09, 0.19, -0.18, 0.31, -0.28]
    sxx = sum(value * value for value in x_values)
    syy = sum(value * value for value in yaw_values)
    sxy = sum(x * yaw for x, yaw in zip(x_values, yaw_values))
    yaw_response_track = sxx / sxy
    direct_track = sxy / syy
    tls_track = (
        sxx - syy + math.hypot(sxx - syy, 2.0 * sxy)
    ) / (2.0 * sxy)

    overall = report["fits"]["overall"]
    assert overall["sample_count"] == 6
    assert overall["yaw_response_ols"]["effective_track_width_m"] == pytest.approx(
        yaw_response_track
    )
    assert overall["direct_ols"]["effective_track_width_m"] == pytest.approx(
        direct_track
    )
    assert overall["origin_tls"]["effective_track_width_m"] == pytest.approx(
        tls_track
    )
    assert overall["yaw_response_ols"]["origin_r_squared"] < 1.0
    assert overall["direct_ols"]["origin_r_squared"] < 1.0
    assert set(report["fits"]["by_side"]) == {"left", "right"}
    assert set(report["fits"]["by_tier"]) == {"low", "nominal", "high"}
    assert len(report["fits"]["by_report"]) == 1
    assert report["counts"] == {
        "input_reports": 1,
        "included_reports": 1,
        "excluded_reports": 0,
        "included_rotation_segments": 6,
        "excluded_non_rotation_segments": 1,
    }
    selection = report["selection"]["included"][0]
    assert selection["path"] == str(source.resolve())
    assert selection["sha256"] == hashlib.sha256(source.read_bytes()).hexdigest()
    assert selection["excluded_segments"][0]["reason"] == "not_pure_rotation"


def test_explicit_provenance_rules_select_without_schema_assumptions(tmp_path):
    """Caller-defined dotted paths filter reports without inferred labels."""
    valid = _write_report(tmp_path / "valid.json", _report())
    wrong_solver = _write_report(
        tmp_path / "wrong_solver.json",
        _report(
            provenance={
                "verified": True,
                "robot": {
                    "solver": {
                        "position_iterations": 32,
                        "velocity_iterations": 16,
                    }
                },
            },
            samples=[
                ("left", "left", "nominal", 0.21, 0.2),
                ("right", "right", "nominal", -0.21, -0.2),
            ],
        ),
    )
    failed = _write_report(
        tmp_path / "failed.json",
        _report(result="failure"),
    )
    requirements = [
        parse_provenance_requirement("verified=true"),
        parse_provenance_requirement("robot.solver.velocity_iterations=4"),
    ]

    report = analyse_effective_track(
        [valid, wrong_solver, failed],
        RADIUS,
        provenance_requirements=requirements,
    )

    assert report["counts"]["included_reports"] == 1
    assert report["counts"]["excluded_reports"] == 2
    reasons = {
        Path(record["path"]).name: {reason["code"] for reason in record["reasons"]}
        for record in report["selection"]["excluded"]
    }
    assert reasons == {
        "wrong_solver.json": {"provenance_value_mismatch"},
        "failed.json": {"report_result_not_success"},
    }
    assert report["selection_policy"]["provenance_requirements"] == [
        {"path": "verified", "expected": True},
        {"path": "robot.solver.velocity_iterations", "expected": 4},
    ]


@pytest.mark.parametrize(
    "mutation,match",
    [
        (
            lambda document: document["segments"][1].update(result="failure"),
            "result must be 'complete'",
        ),
        (
            lambda document: document["segments"][1]["wheels"]["per_wheel"][
                WHEEL_NAMES["front_left"]
            ]["speed_radps"].pop("mean"),
            "speed_radps.mean must be a finite number",
        ),
        (
            lambda document: document["segments"][1]["actual_velocity"][
                "angular_z_radps"
            ].update(mean=0.0),
            "measured yaw rate must be non-zero",
        ),
        (
            _reverse_left_rotation_yaw,
            "measured yaw rate sign does not match rotate_left",
        ),
        (
            _reverse_left_rotation_wheel_differential,
            "wheel differential sign does not match rotate_left",
        ),
    ],
)
def test_invalid_pure_rotation_excludes_entire_report(
    tmp_path, mutation, match
):
    """One malformed rotation prevents every sample in that report entering."""
    valid = _write_report(tmp_path / "valid.json", _report())
    invalid_document = _report(
        samples=[
            ("left", "left", "nominal", 0.2, 0.2),
            ("right", "right", "nominal", -0.2, -0.2),
        ]
    )
    mutation(invalid_document)
    invalid = _write_report(tmp_path / "invalid.json", invalid_document)

    report = analyse_effective_track([valid, invalid], RADIUS)

    assert report["counts"]["included_reports"] == 1
    excluded = report["selection"]["excluded"][0]
    assert excluded["reasons"][0]["code"] == "invalid_rotation_data"
    assert match in excluded["reasons"][0]["detail"]
    assert all(
        sample["segment_id"] != "left"
        for sample in report["selection"]["included"][0][
            "included_rotation_segments"
        ]
    )


def test_cli_writes_strict_json_and_optionally_fails_on_exclusion(tmp_path):
    """The CLI preserves its audit artifact before returning strict status 2."""
    valid = _write_report(tmp_path / "valid.json", _report())
    failed = _write_report(tmp_path / "failed.json", _report(result="failure"))
    output = tmp_path / "analysis.json"

    status = main([
        str(valid),
        str(failed),
        "--wheel-radius",
        str(RADIUS),
        "--output",
        str(output),
        "--fail-on-excluded",
    ])

    assert status == 2
    report = json.loads(output.read_text(encoding="utf-8"))
    assert report["report_type"] == "effective_track_width_analysis"
    assert report["analysis_valid"] is True
    assert report["counts"]["excluded_reports"] == 1


def test_radius_minimum_and_duplicate_content_are_fail_closed(tmp_path):
    """Invalid scale, replication, and sample-count contracts are rejected."""
    source = _write_report(tmp_path / "motion.json", _report())
    duplicate = tmp_path / "duplicate.json"
    duplicate.write_bytes(source.read_bytes())

    with pytest.raises(ConfigurationError, match="wheel_radius_m must be positive"):
        analyse_effective_track([source], 0.0)
    with pytest.raises(ConfigurationError, match="content must be unique"):
        analyse_effective_track([source, duplicate], RADIUS)
    with pytest.raises(ConfigurationError, match="below required minimum"):
        analyse_effective_track(
            [source], RADIUS, minimum_included_reports=2
        )


@pytest.mark.parametrize(
    "expression",
    ["missing_equals", "=true", "robot..solver=4", "verified="],
)
def test_provenance_requirement_rejects_ambiguous_expressions(expression):
    """Malformed selection expressions never degrade into broad matches."""
    with pytest.raises(ConfigurationError):
        parse_provenance_requirement(expression)
