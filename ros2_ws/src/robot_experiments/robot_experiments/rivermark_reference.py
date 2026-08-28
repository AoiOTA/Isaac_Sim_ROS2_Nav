"""Freeze the 0.05/0.025 m occupancy reference for Attempt31 Rivermark."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping

import yaml

from .optimal_path import load_occupancy_grid_reference


CLEARANCE_M = 0.34


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _mapping(path: Path) -> Mapping[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, Mapping):
        raise ValueError(f"expected a YAML mapping: {path}")
    return value


def build_rivermark_reference(
    scenario_file: str | Path,
    spawn_file: str | Path,
    map_file: str | Path,
) -> dict[str, Any]:
    """Compute every five-waypoint leg at native and refined resolution."""

    scenario_path = Path(scenario_file).expanduser().resolve()
    spawn_path = Path(spawn_file).expanduser().resolve()
    map_path = Path(map_file).expanduser().resolve()
    scenario = _mapping(scenario_path).get("scenario")
    if not isinstance(scenario, Mapping):
        raise ValueError("scenario file has no scenario mapping")
    spawn_name = scenario.get("spawn_pose_name")
    route = scenario.get("route")
    if not isinstance(spawn_name, str) or not isinstance(route, list) or not route:
        raise ValueError("scenario spawn/route is incomplete")
    spawn_poses = _mapping(spawn_path).get("spawn_poses")
    if not isinstance(spawn_poses, Mapping) or spawn_name not in spawn_poses:
        raise ValueError(f"spawn pose {spawn_name!r} is unavailable")
    spawn = spawn_poses[spawn_name]
    if not isinstance(spawn, Mapping) or not isinstance(spawn.get("map"), Mapping):
        raise ValueError("spawn map calibration is incomplete")
    start = spawn["map"].get("position")
    if not isinstance(start, list) or len(start) < 2:
        raise ValueError("spawn map position is invalid")
    targets: list[tuple[str, tuple[float, float]]] = []
    for index, item in enumerate(route):
        if not isinstance(item, Mapping) or not isinstance(item.get("position"), list):
            raise ValueError(f"route item {index} is invalid")
        position = item["position"]
        identifier = item.get("id")
        if not isinstance(identifier, str) or len(position) < 2:
            raise ValueError(f"route item {index} lacks id/position")
        targets.append((identifier, (float(position[0]), float(position[1]))))

    reference = load_occupancy_grid_reference(
        map_path, clearance_m=CLEARANCE_M, allow_unknown=False
    )
    if not math.isclose(reference.resolution, 0.05, abs_tol=1.0e-12):
        raise ValueError("Rivermark reference must use the frozen 0.05 m map")

    starts = [(float(start[0]), float(start[1]))] + [
        target for _, target in targets[:-1]
    ]

    def solve(solver) -> list[float]:
        return [
            solver.shortest_path_length(leg_start, target)
            for leg_start, (_, target) in zip(starts, targets)
        ]

    primary = solve(reference)
    refined = solve(reference.refined(2))
    primary_total = sum(primary)
    refined_total = sum(refined)
    convergence = abs(primary_total - refined_total) / refined_total * 100.0
    map_document = _mapping(map_path)
    image = map_document.get("image")
    if not isinstance(image, str):
        raise ValueError("map image is missing")
    image_path = (map_path.parent / image).resolve()
    return {
        "schema_version": 1,
        "campaign": "attempt31_rivermark_five_waypoint",
        "algorithm": {
            "name": "inflated_occupancy_grid_8_connected_astar",
            "clearance_m": CLEARANCE_M,
            "allow_unknown": False,
            "diagonal_corner_cutting": False,
            "reported_length": "translation_only",
            "algorithm_sha256": _sha256(Path(__file__).resolve()),
        },
        "map": {
            "yaml": map_path.name,
            "yaml_sha256": _sha256(map_path),
            "image": image_path.name,
            "image_sha256": _sha256(image_path),
            "native_resolution_m": reference.resolution,
        },
        "scenario": {
            "file": scenario_path.name,
            "sha256": _sha256(scenario_path),
            "spawn_file": spawn_path.name,
            "spawn_sha256": _sha256(spawn_path),
            "spawn_pose_name": spawn_name,
        },
        "legs": [
            {
                "id": identifier,
                "length_m_0_05": primary[index],
                "length_m_0_025": refined[index],
            }
            for index, (identifier, _) in enumerate(targets)
        ],
        "total_length_m_0_05": primary_total,
        "total_length_m_0_025": refined_total,
        "convergence_percent": convergence,
        "convergence_requirement_percent": 1.0,
        "converged": convergence <= 1.0,
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="build the frozen Attempt31 Rivermark optimal reference"
    )
    parser.add_argument("--scenario-file", required=True)
    parser.add_argument("--spawn-file", required=True)
    parser.add_argument("--map-file", required=True)
    parser.add_argument("--output", required=True)
    arguments = parser.parse_args()
    result = build_rivermark_reference(
        arguments.scenario_file, arguments.spawn_file, arguments.map_file
    )
    target = Path(arguments.output).expanduser().resolve()
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(
        json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({"output": str(target), "converged": result["converged"]}))
    raise SystemExit(0 if result["converged"] else 2)


if __name__ == "__main__":
    main()
