"""Freeze the footprint-aware optimal route used by the Kujiale campaign."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping

import yaml

from .kujiale_campaign import CampaignValidationError, load_campaign_definition
from .optimal_path import load_footprint_state_lattice_reference


FOOTPRINT = ((-0.230, -0.210), (0.255, -0.210), (0.255, 0.210), (-0.230, 0.210))
PADDING_M = 0.005


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _box_polygon(center: list[float], size: list[float]) -> list[list[float]]:
    if len(center) < 2 or len(size) < 2:
        raise CampaignValidationError("static obstacle center/size is invalid")
    x, y = float(center[0]), float(center[1])
    half_x, half_y = float(size[0]) / 2.0, float(size[1]) / 2.0
    return [[x - half_x, y - half_y], [x + half_x, y - half_y], [x + half_x, y + half_y], [x - half_x, y + half_y]]


def build_optimal_reference(
    campaign_file: str | Path,
    map_file: str | Path,
) -> dict[str, Any]:
    """Compute 0.05/0.025 m static-route references and freeze all inputs."""
    campaign_path = Path(campaign_file).expanduser().resolve()
    map_path = Path(map_file).expanduser().resolve()
    campaign = load_campaign_definition(campaign_path)
    acceptance = campaign["acceptance"]
    route = campaign["route"]
    static = campaign["static"]
    obstacles = static.get("obstacles")
    if not isinstance(obstacles, list):
        raise CampaignValidationError("campaign static obstacles must be a list")
    polygons = [_box_polygon(item["center"], item["size"]) for item in obstacles if isinstance(item, Mapping)]
    if len(polygons) != len(obstacles):
        raise CampaignValidationError("campaign static obstacle is malformed")

    start_pose = campaign["environment"].get("start_pose")
    if not isinstance(start_pose, list) or len(start_pose) != 3:
        raise CampaignValidationError("campaign environment.start_pose must be [x, y, yaw_deg]")
    waypoints = [(float(start_pose[0]), float(start_pose[1]), math.radians(float(start_pose[2])))] + [
        (float(item["pose"][0]), float(item["pose"][1]), math.radians(float(item["pose"][2])))
        for item in route
    ]
    position_tolerance = float(acceptance["position_tolerance_m"])
    # The report counts translation only.  In-place rotations are collision
    # checked by the lattice but carry zero length, therefore the terminating
    # state may use any collision-free heading inside the Nav2 position goal.
    yaw_tolerance = math.pi

    reference = load_footprint_state_lattice_reference(
        map_path, footprint=FOOTPRINT, padding_m=PADDING_M, heading_bins=16
    ).with_polygon_obstacles(polygons)

    def solve(solver) -> list[float]:
        return [
            solver.shortest_path_length(
                start, goal,
                goal_position_tolerance_m=position_tolerance,
                goal_orientation_tolerance_rad=yaw_tolerance,
            )
            for start, goal in zip(waypoints, waypoints[1:])
        ]

    primary = solve(reference)
    refined = solve(reference.refined(2))
    primary_total, refined_total = sum(primary), sum(refined)
    convergence_percent = abs(primary_total - refined_total) / refined_total * 100.0
    map_document = yaml.safe_load(map_path.read_text(encoding="utf-8"))
    if not isinstance(map_document, Mapping) or not isinstance(map_document.get("image"), str):
        raise CampaignValidationError("reference map YAML is invalid")
    image_path = (map_path.parent / map_document["image"]).resolve()
    return {
        "schema_version": 1,
        "campaign": "kujiale_long_route",
        "algorithm": {
            "name": "footprint_aware_se2_state_lattice_astar",
            "heading_bins": 16,
            "motion_primitives": ["in_place_rotation", "forward_swept_translation"],
            "occupied_and_unknown_blocked": True,
            "reported_length": "translation_only",
            "algorithm_sha256": _sha256(Path(__file__).resolve()),
        },
        "footprint": {"vertices": [list(point) for point in FOOTPRINT], "padding_m": PADDING_M},
        "goal_acceptance": {"position_tolerance_m": position_tolerance, "orientation_handling": "all collision-free terminal headings; rotations are checked but have zero reported translation length"},
        "map": {"yaml": str(map_path), "yaml_sha256": _sha256(map_path), "image": str(image_path), "image_sha256": _sha256(image_path)},
        "campaign_sha256": _sha256(campaign_path),
        "static_obstacle_polygons": polygons,
        "legs": [
            {"id": item["id"], "length_m_0_05": primary[index], "length_m_0_025": refined[index]}
            for index, item in enumerate(route)
        ],
        "total_length_m_0_05": primary_total,
        "total_length_m_0_025": refined_total,
        "convergence_percent": convergence_percent,
        "convergence_requirement_percent": 1.0,
        "converged": convergence_percent <= 1.0,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="build frozen Kujiale optimal_reference.json")
    parser.add_argument("--campaign-file", required=True)
    parser.add_argument("--map-file", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    result = build_optimal_reference(args.campaign_file, args.map_file)
    target = Path(args.output).expanduser().resolve()
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(target), "converged": result["converged"]}))
    raise SystemExit(0 if result["converged"] else 2)


if __name__ == "__main__":
    main()
