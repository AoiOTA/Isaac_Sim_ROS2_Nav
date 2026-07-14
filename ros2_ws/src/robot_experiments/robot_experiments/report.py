"""Pure report validation, hashing, and atomic JSON/CSV writers."""

from __future__ import annotations

import csv
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any, Callable, Mapping, TextIO


REPRODUCIBILITY_FIELDS = (
    "scenario_id",
    "random_seed",
    "map_version",
    "posegraph_version",
    "robot_config_hash",
    "nav2_config_hash",
    "dynamic_runtime_contract",
    "spawn_pose_name",
    "usd_start_pose",
    "map_start_pose",
    "goal_pose",
    "obstacle_trajectories",
    "physics_dt",
    "rtf",
    "result",
    "failure_reason",
)


class ReportValidationError(ValueError):
    """Raised when a report is incomplete or not strict-JSON compatible."""


def configuration_sha256(path: str | os.PathLike[str]) -> str:
    source = Path(path)
    if not source.is_file():
        raise FileNotFoundError(f"configuration file does not exist: {source}")
    digest = hashlib.sha256()
    with source.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _validate_json_value(value: Any, location: str) -> None:
    if value is None or isinstance(value, (str, bool, int)):
        return
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ReportValidationError(f"{location} contains NaN or infinity")
        return
    if isinstance(value, Mapping):
        for key, child in value.items():
            if not isinstance(key, str):
                raise ReportValidationError(f"{location} contains a non-string key")
            _validate_json_value(child, f"{location}.{key}")
        return
    if isinstance(value, (list, tuple)):
        for index, child in enumerate(value):
            _validate_json_value(child, f"{location}[{index}]")
        return
    raise ReportValidationError(f"{location} has unsupported type {type(value).__name__}")


def validate_manifest(manifest: Mapping[str, Any]) -> None:
    missing = [field for field in REPRODUCIBILITY_FIELDS if field not in manifest]
    if missing:
        raise ReportValidationError(f"missing reproducibility fields: {', '.join(missing)}")
    _validate_json_value(manifest, "manifest")
    for hash_field in ("robot_config_hash", "nav2_config_hash"):
        value = manifest[hash_field]
        if not isinstance(value, str) or len(value) != 64:
            raise ReportValidationError(f"{hash_field} must be a SHA256 hex digest")
        try:
            int(value, 16)
        except ValueError as exc:
            raise ReportValidationError(f"{hash_field} must be a SHA256 hex digest") from exc
    dynamic_contract = manifest["dynamic_runtime_contract"]
    if not isinstance(dynamic_contract, Mapping):
        raise ReportValidationError(
            "dynamic_runtime_contract must be a mapping"
        )
    if dynamic_contract.get("verified") is not True:
        raise ReportValidationError(
            "dynamic_runtime_contract must be runtime-verified"
        )
    dynamic_hash = dynamic_contract.get("config_sha256")
    if not isinstance(dynamic_hash, str) or len(dynamic_hash) != 64:
        raise ReportValidationError(
            "dynamic_runtime_contract.config_sha256 must be a SHA256 hex digest"
        )
    try:
        int(dynamic_hash, 16)
    except ValueError as exc:
        raise ReportValidationError(
            "dynamic_runtime_contract.config_sha256 must be a SHA256 hex digest"
        ) from exc


def _required_mapping(
    value: Mapping[str, Any], key: str, location: str
) -> Mapping[str, Any]:
    child = value.get(key)
    if not isinstance(child, Mapping):
        raise ReportValidationError(f"{location}.{key} must be a mapping")
    return child


def _required_string(value: Any, location: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ReportValidationError(f"{location} must be a non-empty string")
    return value


def _validate_sha256(value: Any, location: str) -> None:
    if not isinstance(value, str) or len(value) != 64:
        raise ReportValidationError(f"{location} must be a SHA256 hex digest")
    try:
        int(value, 16)
    except ValueError as exc:
        raise ReportValidationError(
            f"{location} must be a SHA256 hex digest"
        ) from exc


def validate_runtime_provenance(provenance: Mapping[str, Any]) -> None:
    """Validate the Isaac-startup snapshot embedded in a diagnostic report."""

    if not isinstance(provenance, Mapping):
        raise ReportValidationError("runtime_provenance must be a mapping")
    _validate_json_value(provenance, "runtime_provenance")
    if provenance.get("verified") is not True:
        raise ReportValidationError("runtime_provenance must be runtime-verified")
    schema_version = provenance.get("schema_version")
    if isinstance(schema_version, bool) or schema_version != 1:
        raise ReportValidationError("runtime_provenance.schema_version must be 1")

    robot = _required_mapping(provenance, "robot", "runtime_provenance")
    for name in ("config", "asset"):
        input_file = _required_mapping(
            robot, name, "runtime_provenance.robot"
        )
        _required_string(
            input_file.get("path"),
            f"runtime_provenance.robot.{name}.path",
        )
        _validate_sha256(
            input_file.get("sha256"),
            f"runtime_provenance.robot.{name}.sha256",
        )
    solver = _required_mapping(
        robot, "solver", "runtime_provenance.robot"
    )
    for name in ("position_iterations", "velocity_iterations"):
        count = solver.get(name)
        if (
            isinstance(count, bool)
            or not isinstance(count, int)
            or not 1 <= count <= 255
        ):
            raise ReportValidationError(
                f"runtime_provenance.robot.solver.{name} must be an integer "
                "in [1, 255]"
            )

    environment = _required_mapping(
        provenance, "environment", "runtime_provenance"
    )
    for name in ("project_stage", "source_asset"):
        input_file = _required_mapping(
            environment, name, "runtime_provenance.environment"
        )
        _required_string(
            input_file.get("path"),
            f"runtime_provenance.environment.{name}.path",
        )
        _validate_sha256(
            input_file.get("sha256"),
            f"runtime_provenance.environment.{name}.sha256",
        )
    _required_string(
        environment.get("asset_root"),
        "runtime_provenance.environment.asset_root",
    )
    _required_string(
        environment.get("asset_version"),
        "runtime_provenance.environment.asset_version",
    )
    _validate_sha256(
        environment.get("composed_root_layer_sha256"),
        "runtime_provenance.environment.composed_root_layer_sha256",
    )

    simulation = _required_mapping(
        provenance, "simulation", "runtime_provenance"
    )
    for name in ("navigation_mode", "odometry_mode"):
        _required_string(
            simulation.get(name),
            f"runtime_provenance.simulation.{name}",
        )
    physics_hz = simulation.get("physics_hz")
    if (
        isinstance(physics_hz, bool)
        or not isinstance(physics_hz, (int, float))
        or not math.isfinite(float(physics_hz))
        or physics_hz <= 0
    ):
        raise ReportValidationError(
            "runtime_provenance.simulation.physics_hz must be positive"
        )

    git = _required_mapping(provenance, "git", "runtime_provenance")
    commit = git.get("commit")
    if not isinstance(commit, str) or len(commit) not in {40, 64}:
        raise ReportValidationError(
            "runtime_provenance.git.commit must be a Git object id"
        )
    try:
        int(commit, 16)
    except ValueError as exc:
        raise ReportValidationError(
            "runtime_provenance.git.commit must be a Git object id"
        ) from exc
    _required_string(git.get("branch"), "runtime_provenance.git.branch")
    if not isinstance(git.get("dirty"), bool):
        raise ReportValidationError(
            "runtime_provenance.git.dirty must be boolean"
        )


def _atomic_text_write(path: Path, writer: Callable[[TextIO], None]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        dir=path.parent,
        prefix=f".{path.name}.",
        suffix=".tmp",
        text=True,
    )
    temporary_path = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="") as stream:
            writer(stream)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, path)
    except BaseException:
        temporary_path.unlink(missing_ok=True)
        raise


def _csv_value(value: Any) -> str | int | float | bool:
    if value is None:
        return ""
    if isinstance(value, (str, int, float, bool)):
        return value
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def _validate_stem(stem: str) -> None:
    if not stem or stem in {".", ".."} or Path(stem).name != stem:
        raise ValueError("report stem must be one path-safe file name")


def write_run_report(
    manifest: Mapping[str, Any],
    output_directory: str | os.PathLike[str],
    stem: str,
) -> tuple[Path, Path]:
    """Atomically replace one strict JSON report and its single-row CSV peer."""
    validate_manifest(manifest)
    _validate_stem(stem)
    output = Path(output_directory)
    json_path = output / f"{stem}.json"
    csv_path = output / f"{stem}.csv"

    def write_json(stream: TextIO) -> None:
        json.dump(manifest, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")

    fieldnames = list(REPRODUCIBILITY_FIELDS)
    fieldnames.extend(sorted(set(manifest) - set(fieldnames)))

    def write_csv(stream: TextIO) -> None:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, extrasaction="raise")
        writer.writeheader()
        writer.writerow({key: _csv_value(manifest[key]) for key in fieldnames})

    _atomic_text_write(json_path, write_json)
    _atomic_text_write(csv_path, write_csv)
    return json_path, csv_path


def write_strict_json_report(
    document: Mapping[str, Any],
    output_path: str | os.PathLike[str],
) -> Path:
    """Atomically write any mapping after enforcing strict JSON values."""
    if not isinstance(document, Mapping):
        raise ReportValidationError("JSON report root must be a mapping")
    _validate_json_value(document, "report")
    destination = Path(output_path).expanduser()
    if not destination.name or destination.name in {".", ".."}:
        raise ValueError("output_path must name a JSON report file")

    def write_json(stream: TextIO) -> None:
        json.dump(document, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")

    _atomic_text_write(destination, write_json)
    return destination
