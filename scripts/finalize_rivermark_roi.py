#!/usr/bin/env python3
"""Select a usable Rivermark candidate and create runnable demo configs."""

from __future__ import annotations

import argparse
import heapq
import json
import math
from pathlib import Path
import shutil

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
import yaml


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidate-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--region-stride-m", type=float, default=12.0)
    parser.add_argument(
        "--selected-candidate",
        choices=("A", "B", "auto"),
        default="A",
        help="A is the plaza Demo ROI; auto retains the structural score probe",
    )
    return parser


def _score(summary: dict, probe: dict) -> tuple:
    choice = summary["choice_space"]
    free_fraction = float(probe["free_fraction"])
    plausible = 0.05 <= free_fraction <= 0.95
    return (
        int(plausible),
        int(choice["start_goal_alternative_route_count"] > 0),
        int(choice["cycle_count"]),
        int(choice["physical_edge_count"]),
        -int(choice["component_count"]),
    )


def _feature_positions(geojson: dict) -> dict[int, tuple[float, float]]:
    return {
        int(feature["properties"]["id"]): tuple(
            float(value) for value in feature["geometry"]["coordinates"]
        )
        for feature in geojson["features"]
        if feature["geometry"]["type"] == "Point"
    }


def _shortest_support_path(
    geojson: dict,
    positions: dict[int, tuple[float, float]],
    start_id: int,
    goal_id: int,
) -> list[tuple[float, float]]:
    adjacency: dict[int, list[tuple[int, float]]] = {}
    for feature in geojson["features"]:
        if feature["geometry"]["type"] != "MultiLineString":
            continue
        properties = feature["properties"]
        start = int(properties["startid"])
        end = int(properties["endid"])
        length = float(properties["metadata"]["length_m"])
        adjacency.setdefault(start, []).append((end, length))
    distance = {start_id: 0.0}
    parent: dict[int, int] = {}
    queue = [(0.0, start_id)]
    while queue:
        current_distance, node = heapq.heappop(queue)
        if current_distance != distance.get(node):
            continue
        if node == goal_id:
            break
        for neighbor, length in adjacency.get(node, []):
            candidate = current_distance + length
            if candidate < distance.get(neighbor, math.inf):
                distance[neighbor] = candidate
                parent[neighbor] = node
                heapq.heappush(queue, (candidate, neighbor))
    if goal_id not in distance:
        raise RuntimeError("selected start and goal are disconnected in support graph")
    node_ids = [goal_id]
    while node_ids[-1] != start_id:
        node_ids.append(parent[node_ids[-1]])
    node_ids.reverse()
    return [positions[node_id] for node_id in node_ids]


def _support_adjacency(geojson: dict) -> dict[int, list[tuple[int, float]]]:
    adjacency: dict[int, list[tuple[int, float]]] = {}
    for feature in geojson["features"]:
        if feature["geometry"]["type"] != "MultiLineString":
            continue
        properties = feature["properties"]
        adjacency.setdefault(int(properties["startid"]), []).append(
            (
                int(properties["endid"]),
                float(properties["metadata"]["length_m"]),
            )
        )
    return adjacency


def _distances(
    adjacency: dict[int, list[tuple[int, float]]], start: int
) -> dict[int, float]:
    result = {int(start): 0.0}
    queue = [(0.0, int(start))]
    while queue:
        current, node = heapq.heappop(queue)
        if current != result.get(node):
            continue
        for neighbor, length in adjacency.get(node, []):
            candidate = current + length
            if candidate < result.get(neighbor, math.inf):
                result[neighbor] = candidate
                heapq.heappush(queue, (candidate, neighbor))
    return result


def _safe_demo_endpoints(
    *,
    positions: dict[int, tuple[float, float]],
    canonical_to_support: dict[int, int],
    geojson: dict,
    map_yaml: Path,
    boundary_inset_m: float = 5.0,
    minimum_clearance_m: float = 1.0,
) -> tuple[int, int, float]:
    metadata = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
    image_path = map_yaml.parent / str(metadata["image"])
    image = np.asarray(Image.open(image_path).convert("L"), dtype=np.uint8)
    free = image >= 250
    resolution = float(metadata["resolution"])
    origin_x, origin_y = (float(value) for value in metadata["origin"][:2])
    height, width = free.shape
    clearance = distance_transform_edt(free) * resolution
    safe = []
    for support_id in canonical_to_support.values():
        x, y = positions[support_id]
        column = int(math.floor((x - origin_x) / resolution))
        row_from_bottom = int(math.floor((y - origin_y) / resolution))
        row = height - 1 - row_from_bottom
        boundary = min(
            x - origin_x,
            origin_x + width * resolution - x,
            y - origin_y,
            origin_y + height * resolution - y,
        )
        if (
            0 <= row < height
            and 0 <= column < width
            and boundary >= boundary_inset_m
            and clearance[row, column] >= minimum_clearance_m
        ):
            safe.append(int(support_id))
    if len(safe) < 2:
        raise RuntimeError("fewer than two safe interior canonical endpoints")
    adjacency = _support_adjacency(geojson)
    first_distance = _distances(adjacency, min(safe))
    first = max(safe, key=lambda node: first_distance.get(node, -math.inf))
    second_distance = _distances(adjacency, first)
    second = max(safe, key=lambda node: second_distance.get(node, -math.inf))
    route_length = second_distance.get(second)
    if route_length is None or not math.isfinite(route_length):
        raise RuntimeError("safe demo endpoints are disconnected")
    return first, second, float(route_length)


def _polyline_midpoint(
    points: list[tuple[float, float]],
) -> tuple[tuple[float, float], tuple[float, float]]:
    segments = [math.dist(start, end) for start, end in zip(points, points[1:])]
    total = sum(segments)
    if total <= 0.0:
        raise RuntimeError("selected support path has zero length")
    target = total * 0.5
    traversed = 0.0
    for start, end, length in zip(points, points[1:], segments):
        if traversed + length >= target:
            fraction = (target - traversed) / length
            point = (
                start[0] + fraction * (end[0] - start[0]),
                start[1] + fraction * (end[1] - start[1]),
            )
            tangent = ((end[0] - start[0]) / length, (end[1] - start[1]) / length)
            return point, tangent
        traversed += length
    raise AssertionError("polyline midpoint traversal failed")


def _regions(
    positions: dict[int, tuple[float, float]],
    *,
    scene_id: str,
    stride_m: float,
) -> list[dict]:
    if stride_m <= 0.0 or stride_m > 15.0:
        raise ValueError("region stride must be in (0,15] m for the 16 m canvas")
    values = list(positions.values())
    minimum_x = math.floor(min(point[0] for point in values) / stride_m) * stride_m
    minimum_y = math.floor(min(point[1] for point in values) / stride_m) * stride_m
    occupied_cells: dict[tuple[int, int], list[tuple[float, float]]] = {}
    for point in values:
        column = int(math.floor((point[0] - minimum_x) / stride_m))
        row = int(math.floor((point[1] - minimum_y) / stride_m))
        occupied_cells.setdefault((row, column), []).append(point)
    result = []
    for index, ((row, column), points) in enumerate(sorted(occupied_cells.items())):
        x0 = minimum_x + column * stride_m
        y0 = minimum_y + row * stride_m
        center_x = x0 + 0.5 * stride_m
        center_y = y0 + 0.5 * stride_m
        result.append(
            {
                "id": f"{scene_id}:region_{index:02d}",
                "center_map_xy": [center_x, center_y],
                "yaw_deg": 0.0,
                "core_polygon_map": [
                    [x0, y0],
                    [x0 + stride_m, y0],
                    [x0 + stride_m, y0 + stride_m],
                    [x0, y0 + stride_m],
                ],
                "priority": 0,
            }
        )
    return result


def main() -> int:
    args = _parser().parse_args()
    source = args.candidate_dir.expanduser().resolve()
    output = args.output_dir.expanduser().resolve()
    probe = json.loads((source / "candidate_probe.json").read_text(encoding="utf-8"))
    candidates = {}
    for name in ("A", "B"):
        summary_path = source / f"candidate_{name}_graph_summary.json"
        geojson_path = source / f"candidate_{name}_gvg.geojson"
        mapping_path = source / f"candidate_{name}_gvg_mapping.json"
        if not all(path.is_file() for path in (summary_path, geojson_path, mapping_path)):
            continue
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        candidate_probe = probe["candidates"][name]
        candidates[name] = {
            "score": _score(summary, candidate_probe),
            "summary": summary,
            "probe": candidate_probe,
            "geojson": geojson_path,
            "mapping": mapping_path,
        }
    if not candidates:
        raise RuntimeError("no candidate has both occupancy and GVG outputs")
    if args.selected_candidate == "auto":
        selected_name, selected = max(
            candidates.items(), key=lambda item: (item[1]["score"], -ord(item[0]))
        )
        selection_reason = "largest usable 2.5D route choice space"
    else:
        selected_name = args.selected_candidate
        if selected_name not in candidates:
            raise RuntimeError(f"requested candidate {selected_name} has no usable outputs")
        selected = candidates[selected_name]
        selection_reason = (
            "explicit plaza ROI: dense curbs, buildings, street furniture, "
            "roundabout, and route alternatives"
        )
    if not selected["score"][0]:
        raise RuntimeError("candidate occupancy is implausibly empty or occupied")
    if not selected["score"][1]:
        raise RuntimeError("neither selected route probe nor its alternatives are usable")

    output.mkdir(parents=True, exist_ok=True)
    for suffix in ("pgm", "png"):
        shutil.copy2(
            source / f"candidate_{selected_name}_occupancy.{suffix}",
            output / f"rivermark_selected.{suffix}",
        )
    selected_yaml = yaml.safe_load(
        (source / f"candidate_{selected_name}_occupancy.yaml").read_text(
            encoding="utf-8"
        )
    )
    selected_yaml["image"] = "rivermark_selected.pgm"
    (output / "rivermark_selected.yaml").write_text(
        yaml.safe_dump(selected_yaml, sort_keys=False), encoding="utf-8"
    )
    shutil.copy2(selected["geojson"], output / "rivermark_selected.geojson")
    shutil.copy2(selected["mapping"], output / "rivermark_selected_mapping.json")
    preview = source / f"candidate_{selected_name}_gvg_preview"
    if preview.is_dir():
        if (output / "gvg_preview").exists():
            shutil.rmtree(output / "gvg_preview")
        shutil.copytree(preview, output / "gvg_preview")
    rgb = source / f"candidate_{selected_name}_topdown_rgb.png"
    if rgb.is_file():
        shutil.copy2(rgb, output / "rivermark_selected_topdown_rgb.png")
    diagnostic_assets = {
        "height.png": "rivermark_selected_height.png",
        "reachable.png": "rivermark_selected_reachable.png",
        "height_classes.png": "rivermark_selected_height_classes.png",
        "physx_omap.png": "rivermark_selected_physx_omap.png",
        "rgb_occupancy_overlay.png": "rivermark_map_validation.png",
        "edge_overlay.png": "rivermark_edge_alignment.png",
    }
    for candidate_suffix, destination_name in diagnostic_assets.items():
        source_path = source / f"candidate_{selected_name}_{candidate_suffix}"
        if source_path.is_file():
            shutil.copy2(source_path, output / destination_name)

    mapping = json.loads(selected["mapping"].read_text(encoding="utf-8"))
    geojson = json.loads(selected["geojson"].read_text(encoding="utf-8"))
    positions = _feature_positions(geojson)
    canonical_to_support = {
        int(key): int(value)
        for key, value in mapping["canonical_to_support_nodes"].items()
    }
    choice = selected["summary"]["choice_space"]
    start_support, goal_support, demo_route_length_m = _safe_demo_endpoints(
        positions=positions,
        canonical_to_support=canonical_to_support,
        geojson=geojson,
        map_yaml=source / f"candidate_{selected_name}_occupancy.yaml",
    )
    start = positions[start_support]
    goal = positions[goal_support]
    yaw_deg = math.degrees(math.atan2(goal[1] - start[1], goal[0] - start[0]))
    ground_z_m = float(selected["probe"]["ground_z_m"])

    scene_id = f"rivermark_{selected_name.lower()}"
    region_document = {
        "schema_version": 1,
        "scene_id": scene_id,
        "map_frame": "map",
        "regions": _regions(
            positions,
            scene_id=scene_id,
            stride_m=float(args.region_stride_m),
        ),
    }
    (output / "rivermark_regions.yaml").write_text(
        yaml.safe_dump(region_document, sort_keys=False), encoding="utf-8"
    )
    spawn_document = {
        "schema_version": 1,
        "spawn_poses": {
            "rivermark_start": {
                "usd": {
                    "position": [start[0], start[1], ground_z_m + 0.0635],
                    "yaw_deg": yaw_deg,
                },
                "map": {
                    "position": [start[0], start[1]],
                    "yaw_deg": yaw_deg,
                    "calibrated": False,
                },
            }
        },
    }
    (output / "rivermark.spawn.yaml").write_text(
        yaml.safe_dump(spawn_document, sort_keys=False), encoding="utf-8"
    )
    goals = {
        "frame_id": "map",
        "start": [start[0], start[1], yaw_deg],
        "goal": [goal[0], goal[1], yaw_deg],
    }
    (output / "rivermark_demo_goals.yaml").write_text(
        yaml.safe_dump(goals, sort_keys=False), encoding="utf-8"
    )
    support_path = _shortest_support_path(
        geojson,
        positions,
        start_support,
        goal_support,
    )
    midpoint, tangent = _polyline_midpoint(support_path)
    normal = (-tangent[1], tangent[0])
    crossing_half_width_m = 3.0
    dynamic = {
        "schema_version": 1,
        "seed": 20260813,
        "enabled": True,
        "obstacles": [
            {
                "id": "rivermark_crossing_cart",
                "shape": "cube",
                "size": [0.8, 0.6, 1.0],
                "mass": 30.0,
                "start": [
                    midpoint[0] - normal[0] * crossing_half_width_m,
                    midpoint[1] - normal[1] * crossing_half_width_m,
                    ground_z_m + 0.5,
                ],
                "end": [
                    midpoint[0] + normal[0] * crossing_half_width_m,
                    midpoint[1] + normal[1] * crossing_half_width_m,
                    ground_z_m + 0.5,
                ],
                "speed": 0.35,
                "phase_jitter": 0.0,
                "repeat": True,
            }
        ],
    }
    (output / "rivermark_dynamic.yaml").write_text(
        yaml.safe_dump(dynamic, sort_keys=False), encoding="utf-8"
    )

    comparison = {
        name: {
            "score": list(value["score"]),
            "free_fraction": value["probe"]["free_fraction"],
            "choice_space": value["summary"]["choice_space"],
        }
        for name, value in candidates.items()
    }
    selection = {
        "classification": "research_demo_engineering_output",
        "selected_candidate": selected_name,
        "selection_reason": selection_reason,
        "candidate_comparison": comparison,
        "map_yaml": str(output / "rivermark_selected.yaml"),
        "route_graph": str(output / "rivermark_selected.geojson"),
        "regions": str(output / "rivermark_regions.yaml"),
        "spawn_poses": str(output / "rivermark.spawn.yaml"),
        "demo_goals": str(output / "rivermark_demo_goals.yaml"),
        "dynamic_obstacles": str(output / "rivermark_dynamic.yaml"),
        "dynamic_crossing_map_xy": list(midpoint),
        "demo_endpoint_policy": "interior canonical nodes, boundary>=5m, clearance>=1m",
        "demo_route_length_m": demo_route_length_m,
    }
    (output / "selected_roi.yaml").write_text(
        yaml.safe_dump(selection, sort_keys=False), encoding="utf-8"
    )
    (source / "rivermark_candidate_region_comparison.json").write_text(
        json.dumps({"selected": selected_name, "candidates": comparison}, indent=2)
        + "\n",
        encoding="utf-8",
    )
    lines = [
        "# Rivermark candidate comparison",
        "",
        f"Selected: **Candidate {selected_name}**",
        "",
        "| Candidate | Free fraction | Physical edges | Cycles | Alternatives | Components |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for name, value in sorted(comparison.items()):
        choice = value["choice_space"]
        lines.append(
            f"| {name} | {value['free_fraction']:.3f} | {choice['physical_edge_count']} | "
            f"{choice['cycle_count']} | {choice['start_goal_alternative_route_count']} | "
            f"{choice['component_count']} |"
        )
    (source / "rivermark_candidate_region_comparison.md").write_text(
        "\n".join(lines) + "\n", encoding="utf-8"
    )
    print(json.dumps(selection, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
