"""Freeze the footprint-aware optimal route used by the Kujiale campaign."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence

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


def _mapping(path: Path, label: str) -> Mapping[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, Mapping):
        raise CampaignValidationError(f"{label} must be a YAML mapping")
    return value


def _finite_pose(value: Any, label: str) -> tuple[float, float, float]:
    if not isinstance(value, list) or len(value) < 2:
        raise CampaignValidationError(f"{label} must contain x and y")
    yaw = value[2] if len(value) >= 3 else 0.0
    pose = (float(value[0]), float(value[1]), math.radians(float(yaw)))
    if not all(math.isfinite(item) for item in pose):
        raise CampaignValidationError(f"{label} must be finite")
    return pose


def _validate_lengths(primary: Sequence[float], refined: Sequence[float]) -> None:
    if len(primary) != len(refined) or not primary:
        raise CampaignValidationError("reference solver returned incomplete legs")
    if not all(math.isfinite(value) and value > 0.0 for value in (*primary, *refined)):
        raise CampaignValidationError("reference leg lengths must be finite and positive")


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


def build_v6_optimal_reference(
    scenario_file: str | Path,
    spawn_file: str | Path,
    obstacle_file: str | Path,
    map_file: str | Path,
) -> dict[str, Any]:
    """Compute the frozen V6 indoor static five-leg optimal reference."""

    scenario_path = Path(scenario_file).expanduser().resolve()
    spawn_path = Path(spawn_file).expanduser().resolve()
    obstacle_path = Path(obstacle_file).expanduser().resolve()
    map_path = Path(map_file).expanduser().resolve()

    scenario = _mapping(scenario_path, "scenario file").get("scenario")
    if not isinstance(scenario, Mapping):
        raise CampaignValidationError("scenario file has no scenario mapping")
    scenario_id = scenario.get("id")
    if not isinstance(scenario_id, str) or not scenario_id:
        raise CampaignValidationError("scenario.id is invalid")
    if scenario.get("type") != "static":
        raise CampaignValidationError("V6 optimal reference requires a static scenario")

    spawn_name = scenario.get("spawn_pose_name")
    if not isinstance(spawn_name, str) or not spawn_name:
        raise CampaignValidationError("scenario.spawn_pose_name is invalid")
    spawn_poses = _mapping(spawn_path, "spawn file").get("spawn_poses")
    if not isinstance(spawn_poses, Mapping) or spawn_name not in spawn_poses:
        raise CampaignValidationError(f"spawn pose {spawn_name!r} is unavailable")
    spawn = spawn_poses[spawn_name]
    if not isinstance(spawn, Mapping) or not isinstance(spawn.get("map"), Mapping):
        raise CampaignValidationError(f"spawn pose {spawn_name!r}.map is invalid")
    start_map = spawn["map"]
    start_position = start_map.get("position")
    start_yaw = start_map.get("yaw_deg")
    if not isinstance(start_position, list) or start_yaw is None:
        raise CampaignValidationError(f"spawn pose {spawn_name!r}.map is incomplete")
    start_pose = _finite_pose([*start_position[:2], start_yaw], f"spawn pose {spawn_name!r}.map")

    route = scenario.get("route")
    if not isinstance(route, list) or len(route) != 5:
        raise CampaignValidationError("V6 static route must contain exactly five legs")
    route_ids = tuple(item.get("id") if isinstance(item, Mapping) else None for item in route)
    if route_ids != ("G2", "G3", "G4", "G5", "G1"):
        raise CampaignValidationError("V6 static route must be G2,G3,G4,G5,G1")
    targets: list[tuple[str, tuple[float, float, float]]] = []
    for index, item in enumerate(route):
        assert isinstance(item, Mapping)
        position = item.get("position")
        yaw = item.get("yaw_deg")
        if not isinstance(position, list) or yaw is None:
            raise CampaignValidationError(f"scenario route item {index} is incomplete")
        targets.append(
            (str(item["id"]), _finite_pose([*position[:2], yaw], f"scenario route item {index}"))
        )

    obstacle_contract = scenario.get("obstacles")
    if not isinstance(obstacle_contract, Mapping):
        raise CampaignValidationError("scenario.obstacles is invalid")
    selected = obstacle_contract.get("static")
    if not isinstance(selected, list) or len(selected) != 1 or not isinstance(selected[0], Mapping):
        raise CampaignValidationError("V6 static scenario must select exactly one obstacle")
    obstacle_id = selected[0].get("id")
    if obstacle_id != "v6_low_box_solo":
        raise CampaignValidationError("V6 static obstacle must be v6_low_box_solo")

    obstacle_document = _mapping(obstacle_path, "obstacle file")
    physical_obstacles = obstacle_document.get("obstacles")
    if not isinstance(physical_obstacles, list) or len(physical_obstacles) != 1:
        raise CampaignValidationError("V6 obstacle file must contain exactly one obstacle")
    obstacle = physical_obstacles[0]
    if not isinstance(obstacle, Mapping) or obstacle.get("id") != obstacle_id:
        raise CampaignValidationError("V6 physical obstacle does not match the scenario")
    if obstacle.get("mode") != "stationary":
        raise CampaignValidationError("V6 optimal reference obstacle must be stationary")
    center = obstacle.get("start")
    size = obstacle.get("size")
    if not isinstance(center, list) or not isinstance(size, list):
        raise CampaignValidationError("V6 physical obstacle start/size is invalid")
    polygon = [
        [round(value, 12) for value in point]
        for point in _box_polygon(center, size)
    ]
    if not all(math.isfinite(value) for point in polygon for value in point):
        raise CampaignValidationError("V6 physical obstacle polygon must be finite")

    success = scenario.get("success")
    if not isinstance(success, Mapping):
        raise CampaignValidationError("scenario.success is invalid")
    position_tolerance = float(success.get("position_tolerance_m"))
    if not math.isfinite(position_tolerance) or position_tolerance < 0.0:
        raise CampaignValidationError("scenario.success.position_tolerance_m is invalid")
    yaw_tolerance = math.pi

    reference = load_footprint_state_lattice_reference(
        map_path, footprint=FOOTPRINT, padding_m=PADDING_M, heading_bins=16
    ).with_polygon_obstacles([polygon])
    if not math.isclose(reference.grid.resolution, 0.05, abs_tol=1.0e-12):
        raise CampaignValidationError("V6 optimal reference requires the frozen 0.05 m map")

    waypoints = [start_pose] + [pose for _, pose in targets]

    def solve(solver: Any) -> list[float]:
        return [
            solver.shortest_path_length(
                start,
                goal,
                goal_position_tolerance_m=position_tolerance,
                goal_orientation_tolerance_rad=yaw_tolerance,
            )
            for start, goal in zip(waypoints, waypoints[1:])
        ]

    primary = solve(reference)
    refined = solve(reference.refined(2))
    _validate_lengths(primary, refined)
    primary_total = sum(primary)
    refined_total = sum(refined)
    convergence_percent = abs(primary_total - refined_total) / refined_total * 100.0

    map_document = _mapping(map_path, "reference map YAML")
    image = map_document.get("image")
    if not isinstance(image, str) or not image:
        raise CampaignValidationError("reference map YAML image is invalid")
    image_path = (map_path.parent / image).resolve()
    generator_path = Path(__file__).resolve()
    solver_path = generator_path.with_name("optimal_path.py")
    map_id = scenario.get("map_version")
    if not isinstance(map_id, str) or not map_id:
        raise CampaignValidationError("scenario.map_version is invalid")

    return {
        "schema_version": 1,
        "campaign": scenario_id,
        "algorithm": {
            "name": "footprint_aware_se2_state_lattice_astar",
            "heading_bins": 16,
            "motion_primitives": ["in_place_rotation", "forward_swept_translation"],
            "occupied_and_unknown_blocked": True,
            "reported_length": "translation_only",
            "generator": "robot_experiments/kujiale_reference.py",
            "generator_sha256": _sha256(generator_path),
            "optimal_path": "robot_experiments/optimal_path.py",
            "optimal_path_sha256": _sha256(solver_path),
        },
        "footprint": {"vertices": [list(point) for point in FOOTPRINT], "padding_m": PADDING_M},
        "goal_acceptance": {
            "position_tolerance_m": position_tolerance,
            "orientation_handling": "all collision-free terminal headings; rotations are checked but have zero reported translation length",
        },
        "scenario": {
            "id": scenario_id,
            "file": scenario_path.name,
            "sha256": _sha256(scenario_path),
            "spawn_pose_name": spawn_name,
            "spawn_file": spawn_path.name,
            "spawn_sha256": _sha256(spawn_path),
            "obstacle_layout_id": obstacle_contract.get("layout_id"),
            "obstacle_id": obstacle_id,
            "obstacle_file": obstacle_path.name,
            "obstacle_sha256": _sha256(obstacle_path),
        },
        "map": {
            "id": map_id,
            "yaml": map_path.name,
            "yaml_sha256": _sha256(map_path),
            "image": image_path.name,
            "image_sha256": _sha256(image_path),
            "native_resolution_m": reference.grid.resolution,
        },
        "static_obstacle_polygons": [polygon],
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
        "convergence_percent": convergence_percent,
        "convergence_requirement_percent": 1.0,
        "converged": convergence_percent <= 1.0,
    }


def main(argv: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="build frozen Kujiale optimal_reference.json")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--campaign-file")
    source.add_argument("--scenario-file")
    parser.add_argument("--spawn-file")
    parser.add_argument("--obstacle-file")
    parser.add_argument("--map-file", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args(argv)
    if args.campaign_file is not None:
        if args.spawn_file is not None or args.obstacle_file is not None:
            parser.error("--spawn-file/--obstacle-file are valid only with --scenario-file")
        result = build_optimal_reference(args.campaign_file, args.map_file)
    else:
        if args.spawn_file is None or args.obstacle_file is None:
            parser.error("--scenario-file requires --spawn-file and --obstacle-file")
        result = build_v6_optimal_reference(
            args.scenario_file,
            args.spawn_file,
            args.obstacle_file,
            args.map_file,
        )
    target = Path(args.output).expanduser().resolve()
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(
        json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({"output": str(target), "converged": result["converged"]}))
    raise SystemExit(0 if result["converged"] else 2)


if __name__ == "__main__":
    main()
