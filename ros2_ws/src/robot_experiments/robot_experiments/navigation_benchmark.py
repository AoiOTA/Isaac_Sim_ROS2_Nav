"""Aggregate navigation run manifests into strict acceptance statistics."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any, Iterable, Mapping

from .metrics import (
    PLAN_DYNAMIC_SUCCESS_MIN_PERCENT,
    PLAN_PATH_DEVIATION_MAX_PERCENT,
    PLAN_STATIC_SUCCESS_MIN_PERCENT,
    path_length_deviation_percent,
    percentile,
    success_rate_percent,
)
from .optimal_path import load_occupancy_grid_reference
from .report import validate_manifest


class NavigationBenchmarkError(ValueError):
    """Raised when a benchmark input set is incomplete or inconsistent."""


@dataclass(frozen=True)
class RateResult:
    successes: int
    total: int
    rate_percent: float
    required_percent: float
    minimum_total: int
    passed: bool


def _manifest_paths(directories: Iterable[str | Path]) -> list[Path]:
    paths: list[Path] = []
    for value in directories:
        directory = Path(value).expanduser().resolve()
        if not directory.is_dir():
            raise NavigationBenchmarkError(
                f"benchmark directory does not exist: {directory}"
            )
        paths.extend(sorted(directory.rglob("*.json")))
    if not paths:
        raise NavigationBenchmarkError("benchmark contains no JSON manifests")
    return paths


def _load_manifests(
    directories: Iterable[str | Path],
    scenario_type: str,
) -> list[Mapping[str, Any]]:
    manifests: list[Mapping[str, Any]] = []
    identities: set[tuple[str, int, int]] = set()
    for path in _manifest_paths(directories):
        value = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(value, dict):
            raise NavigationBenchmarkError(f"{path} is not a JSON object")
        validate_manifest(value)
        if value.get("scenario_type") != scenario_type:
            raise NavigationBenchmarkError(
                f"{path} is not a {scenario_type} scenario"
            )
        identity = (
            str(value["scenario_id"]),
            int(value["run_index"]),
            int(value["random_seed"]),
        )
        if identity in identities:
            raise NavigationBenchmarkError(
                f"duplicate run identity {identity}"
            )
        identities.add(identity)
        manifests.append(value)
    return manifests


def _rate(
    manifests: list[Mapping[str, Any]],
    required_percent: float,
    minimum_total: int,
) -> RateResult:
    total = len(manifests)
    successes = sum(value.get("result") == "success" for value in manifests)
    rate = success_rate_percent(successes, total)
    return RateResult(
        successes=successes,
        total=total,
        rate_percent=rate,
        required_percent=required_percent,
        minimum_total=minimum_total,
        passed=total >= minimum_total and rate >= required_percent,
    )


def summarize_navigation_benchmark(
    *,
    static_directories: Iterable[str | Path],
    dynamic_directories: Iterable[str | Path],
    map_file: str | Path,
    clearance_m: float,
    minimum_static_runs: int = 20,
    minimum_dynamic_runs: int = 20,
) -> dict[str, Any]:
    static = _load_manifests(static_directories, "static")
    dynamic = _load_manifests(dynamic_directories, "dynamic")
    reference = load_occupancy_grid_reference(
        map_file,
        clearance_m=clearance_m,
        allow_unknown=False,
    )

    path_runs: list[dict[str, Any]] = []
    for manifest in static:
        if manifest.get("result") != "success":
            continue
        metrics = manifest.get("metrics")
        if not isinstance(metrics, Mapping):
            raise NavigationBenchmarkError("manifest metrics must be a mapping")
        executed = float(metrics.get("ground_truth_path_length_m", math.nan))
        start = manifest.get("map_start_pose")
        goal = manifest.get("goal_pose")
        if not isinstance(start, Mapping) or not isinstance(goal, Mapping):
            raise NavigationBenchmarkError(
                "map_start_pose and goal_pose must be mappings"
            )
        optimal = reference.shortest_path_length(
            start.get("position", ()),
            goal.get("position", ()),
        )
        deviation = path_length_deviation_percent(executed, optimal)
        path_runs.append(
            {
                "scenario_id": manifest["scenario_id"],
                "run_index": manifest["run_index"],
                "random_seed": manifest["random_seed"],
                "executed_length_m": executed,
                "optimal_length_m": optimal,
                "deviation_percent": deviation,
                "passed": deviation <= PLAN_PATH_DEVIATION_MAX_PERCENT,
            }
        )

    deviations = [value["deviation_percent"] for value in path_runs]
    path_passed = bool(path_runs) and all(
        value["passed"] for value in path_runs
    )
    static_rate = _rate(
        static,
        PLAN_STATIC_SUCCESS_MIN_PERCENT,
        minimum_static_runs,
    )
    dynamic_rate = _rate(
        dynamic,
        PLAN_DYNAMIC_SUCCESS_MIN_PERCENT,
        minimum_dynamic_runs,
    )
    summary = {
        "schema_version": 1,
        "reference": {
            "method": "inflated_occupancy_grid_8_connected_astar",
            "map_file": str(Path(map_file).expanduser().resolve()),
            "clearance_m": float(clearance_m),
            "allow_unknown": False,
        },
        "static_avoidance": asdict(static_rate),
        "dynamic_avoidance": asdict(dynamic_rate),
        "path_optimality": {
            "evaluated_runs": len(path_runs),
            "maximum_deviation_percent": (
                max(deviations) if deviations else None
            ),
            "percentile_95_deviation_percent": (
                percentile(deviations, 95.0) if deviations else None
            ),
            "required_maximum_percent": PLAN_PATH_DEVIATION_MAX_PERCENT,
            "passed": path_passed,
            "runs": path_runs,
        },
    }
    summary["passed"] = (
        static_rate.passed
        and dynamic_rate.passed
        and path_passed
    )
    return summary


def _write_json_atomic(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        dir=path.parent,
        prefix=f".{path.name}.",
        suffix=".tmp",
        text=True,
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
        temporary.replace(path)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--static-directory", action="append", required=True)
    parser.add_argument("--dynamic-directory", action="append", required=True)
    parser.add_argument("--map-file", required=True)
    parser.add_argument("--clearance-m", type=float, default=0.34)
    parser.add_argument("--minimum-static-runs", type=int, default=20)
    parser.add_argument("--minimum-dynamic-runs", type=int, default=20)
    parser.add_argument("--output", required=True)
    arguments = parser.parse_args()
    summary = summarize_navigation_benchmark(
        static_directories=arguments.static_directory,
        dynamic_directories=arguments.dynamic_directory,
        map_file=arguments.map_file,
        clearance_m=arguments.clearance_m,
        minimum_static_runs=arguments.minimum_static_runs,
        minimum_dynamic_runs=arguments.minimum_dynamic_runs,
    )
    _write_json_atomic(Path(arguments.output).expanduser().resolve(), summary)
    print(json.dumps(summary, indent=2, sort_keys=True, allow_nan=False))
    raise SystemExit(0 if summary["passed"] else 2)


if __name__ == "__main__":
    main()
