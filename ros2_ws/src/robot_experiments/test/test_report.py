import csv
from copy import deepcopy
import hashlib
import json
import math

import pytest

from robot_experiments.report import (
    REPRODUCIBILITY_FIELDS,
    ReportValidationError,
    configuration_sha256,
    validate_manifest,
    validate_runtime_provenance,
    write_run_report,
    write_strict_json_report,
)


def valid_runtime_provenance():
    return {
        "verified": True,
        "schema_version": 1,
        "robot": {
            "config": {"path": "/repo/jackal.yaml", "sha256": "a" * 64},
            "asset": {"path": "/repo/jackal_nav.usda", "sha256": "b" * 64},
            "solver": {
                "position_iterations": 32,
                "velocity_iterations": 4,
            },
        },
        "environment": {
            "project_stage": {
                "path": "/repo/navigation_scene.usda",
                "sha256": "c" * 64,
            },
            "source_asset": {
                "path": "/assets/warehouse.usd",
                "sha256": "d" * 64,
            },
            "asset_root": "/assets/Isaac/6.0",
            "asset_version": "6.0",
            "composed_root_layer_sha256": "e" * 64,
        },
        "simulation": {
            "navigation_mode": "mapping",
            "odometry_mode": "ideal",
            "physics_hz": 60.0,
        },
        "git": {
            "commit": "f" * 40,
            "branch": "codex/navigation-quality-fidelity",
            "dirty": True,
        },
    }


def valid_manifest():
    return {
        "scenario_id": "static_a",
        "random_seed": 42,
        "map_version": "map-v1",
        "posegraph_version": "graph-v1",
        "robot_config_hash": "a" * 64,
        "nav2_config_hash": "b" * 64,
        "dynamic_runtime_contract": {
            "verified": True,
            "enabled": False,
            "config_sha256": "c" * 64,
            "obstacle_ids": ["crossing_box", "cart_proxy_b"],
        },
        "spawn_pose_name": "mapping_start",
        "usd_start_pose": {"position": [1.0, 2.0, 0.1], "yaw_deg": 0.0},
        "map_start_pose": {"position": [0.0, 0.0], "yaw_deg": 0.0},
        "goal_pose": {"position": [4.0, 1.0], "yaw_deg": 0.0},
        "obstacle_trajectories": [],
        "physics_dt": 1.0 / 60.0,
        "rtf": 1.0,
        "result": "success",
        "failure_reason": "",
        "metrics": {"path_length_m": 4.2},
    }


def test_configuration_sha256_streams_exact_file(tmp_path):
    config = tmp_path / "config.yaml"
    config.write_bytes(b"alpha: 1\n")
    assert configuration_sha256(config) == hashlib.sha256(b"alpha: 1\n").hexdigest()


def test_validate_manifest_requires_every_reproducibility_field():
    manifest = valid_manifest()
    del manifest[REPRODUCIBILITY_FIELDS[0]]
    with pytest.raises(ReportValidationError, match="missing reproducibility"):
        validate_manifest(manifest)


@pytest.mark.parametrize("bad_hash", ["short", "g" * 64, 123])
def test_validate_manifest_rejects_invalid_hashes(bad_hash):
    manifest = valid_manifest()
    manifest["robot_config_hash"] = bad_hash
    with pytest.raises(ReportValidationError, match="SHA256"):
        validate_manifest(manifest)


def test_validate_manifest_requires_verified_dynamic_runtime_contract():
    manifest = valid_manifest()
    manifest["dynamic_runtime_contract"]["verified"] = False
    with pytest.raises(ReportValidationError, match="runtime-verified"):
        validate_manifest(manifest)


def test_validate_runtime_provenance_accepts_a_complete_startup_snapshot():
    validate_runtime_provenance(valid_runtime_provenance())


@pytest.mark.parametrize(
    ("path", "bad_value", "message"),
    [
        (("robot", "config", "sha256"), "g" * 64, "SHA256"),
        (("robot", "solver", "velocity_iterations"), True, "integer"),
        (("robot", "solver", "velocity_iterations"), 0, "integer"),
        (("robot", "solver", "velocity_iterations"), 256, "integer"),
        (("environment", "source_asset", "path"), "", "non-empty"),
        (("simulation", "physics_hz"), 0.0, "positive"),
        (("git", "commit"), "z" * 40, "Git object id"),
        (("git", "dirty"), "false", "boolean"),
    ],
)
def test_validate_runtime_provenance_rejects_bad_evidence(
    path, bad_value, message
):
    provenance = deepcopy(valid_runtime_provenance())
    cursor = provenance
    for key in path[:-1]:
        cursor = cursor[key]
    cursor[path[-1]] = bad_value
    with pytest.raises(ReportValidationError, match=message):
        validate_runtime_provenance(provenance)


def test_validate_runtime_provenance_requires_runtime_verification():
    provenance = valid_runtime_provenance()
    provenance["verified"] = False
    with pytest.raises(ReportValidationError, match="runtime-verified"):
        validate_runtime_provenance(provenance)


@pytest.mark.parametrize("bad_value", [math.nan, math.inf, -math.inf])
def test_nan_and_infinity_are_rejected_recursively(bad_value):
    manifest = valid_manifest()
    manifest["metrics"] = {"nested": [0.0, bad_value]}
    with pytest.raises(ReportValidationError, match="NaN or infinity"):
        validate_manifest(manifest)


def test_json_and_csv_are_written_with_complete_fields_and_no_temp_files(tmp_path):
    manifest = valid_manifest()
    json_path, csv_path = write_run_report(manifest, tmp_path, "run-0001")

    assert json.loads(json_path.read_text()) == manifest
    with csv_path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 1
    assert set(REPRODUCIBILITY_FIELDS) <= set(rows[0])
    assert json.loads(rows[0]["metrics"]) == manifest["metrics"]
    assert not list(tmp_path.glob("*.tmp"))


def test_report_atomically_replaces_existing_files(tmp_path):
    first = valid_manifest()
    write_run_report(first, tmp_path, "same-run")
    second = valid_manifest()
    second["result"] = "failure"
    second["failure_reason"] = "timed_out"
    write_run_report(second, tmp_path, "same-run")
    assert json.loads((tmp_path / "same-run.json").read_text())["result"] == "failure"


def test_generic_strict_json_report_is_atomic_and_rejects_nonfinite_values(tmp_path):
    destination = tmp_path / "motion.json"
    assert write_strict_json_report({"result": "success"}, destination) == destination
    assert json.loads(destination.read_text()) == {"result": "success"}
    with pytest.raises(ReportValidationError, match="NaN or infinity"):
        write_strict_json_report({"metric": math.nan}, destination)
    assert json.loads(destination.read_text()) == {"result": "success"}
    assert not list(tmp_path.glob("*.tmp"))


@pytest.mark.parametrize("stem", ["", ".", "..", "../escape", "nested/run"])
def test_report_stem_cannot_escape_output_directory(tmp_path, stem):
    with pytest.raises(ValueError, match="path-safe"):
        write_run_report(valid_manifest(), tmp_path, stem)
