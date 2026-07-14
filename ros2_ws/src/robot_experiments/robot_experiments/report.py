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
