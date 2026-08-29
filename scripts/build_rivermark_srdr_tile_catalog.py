#!/usr/bin/env python3
"""Build deterministic Module3 constraints for the selected Rivermark tiles."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import cv2
import numpy as np
import yaml


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "ros2_ws/src/robot_bringup"))
sys.path.insert(0, str(REPOSITORY_ROOT / "ros2_ws/src/robot_route_planner"))

from robot_bringup.map_manifest import (  # noqa: E402
    compute_occupancy_map_bundle_sha256,
)
from robot_route_planner.cognitive_constraints import (  # noqa: E402
    build_cognitive_constraints,
    occupancy_grid_version,
)
from robot_route_planner.map_io import load_occupancy_map  # noqa: E402
from robot_route_planner.regions import CognitiveRegion, load_region_config  # noqa: E402


SELECTION_SCHEMA = "bio_nav.v310.rivermark_srdr_tile_selection.v1"
CATALOG_SCHEMA = "bio_nav.v310.srdr_tile_catalog.v1"


def _load_mapping(path: Path, description: str) -> dict:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{description} must be a mapping")
    return value


def _require_int(value: object, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{field} must be an integer")
    return value


def _referenced_path(selection_path: Path, raw: object, field: str) -> Path:
    if not isinstance(raw, str) or not raw.strip():
        raise ValueError(f"references.{field} must be a non-empty path")
    return (selection_path.parent / raw).resolve()


def _assert_reference(actual: Path, expected: Path, field: str) -> None:
    if actual.resolve() != expected:
        raise ValueError(f"--{field} does not match selection reference")


def _runtime_map_version(map_yaml: Path) -> str:
    metadata = _load_mapping(map_yaml, "map YAML")
    if str(metadata.get("mode", "trinary")) != "trinary":
        raise ValueError("Rivermark catalog requires trinary map-server mode")
    image_path = Path(str(metadata["image"]))
    if not image_path.is_absolute():
        image_path = map_yaml.parent / image_path
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(image_path)
    occupancy = image.astype(np.float64) / 255.0
    if not int(metadata.get("negate", 0)):
        occupancy = 1.0 - occupancy
    data = np.full(image.shape, -1, dtype=np.int8)
    data[occupancy > float(metadata["occupied_thresh"])] = 100
    data[occupancy < float(metadata["free_thresh"])] = 0
    data = np.flipud(data)
    height, width = data.shape
    origin = metadata["origin"]
    return occupancy_grid_version(
        width=width,
        height=height,
        resolution=float(np.float32(metadata["resolution"])),
        origin_x=float(origin[0]),
        origin_y=float(origin[1]),
        data=data,
    )


def _bounds(region: CognitiveRegion) -> list[float]:
    polygon = np.asarray(region.core_polygon_map, dtype=np.float64)
    return [
        float(polygon[:, 0].min()),
        float(polygon[:, 1].min()),
        float(polygon[:, 0].max()),
        float(polygon[:, 1].max()),
    ]


def _canvas_bounds(region: CognitiveRegion) -> list[float]:
    corners = np.asarray(((-8.0, -8.0), (8.0, -8.0), (8.0, 8.0), (-8.0, 8.0)))
    inverse = np.linalg.inv(region.t_map_canvas)
    mapped = (inverse @ np.column_stack((corners, np.ones(4))).T).T
    xy = mapped[:, :2] / mapped[:, 2:3]
    return [
        float(xy[:, 0].min()),
        float(xy[:, 1].min()),
        float(xy[:, 0].max()),
        float(xy[:, 1].max()),
    ]


def _cardinal_neighbors(
    region: CognitiveRegion, selected: tuple[CognitiveRegion, ...]
) -> list[str]:
    x0, y0, x1, y1 = _bounds(region)
    neighbors = []
    for candidate in selected:
        if candidate.region_id == region.region_id:
            continue
        a0, b0, a1, b1 = _bounds(candidate)
        vertical = (np.isclose(x1, a0) or np.isclose(a1, x0)) and min(y1, b1) > max(y0, b0)
        horizontal = (np.isclose(y1, b0) or np.isclose(b1, y0)) and min(x1, a1) > max(x0, a0)
        if vertical or horizontal:
            neighbors.append(candidate.region_id)
    return sorted(neighbors)


def _region_for_point(regions: tuple[CognitiveRegion, ...], point: object) -> str:
    xy = tuple(float(value) for value in point[:2])
    matches = [region.region_id for region in regions if region.contains(xy)]
    if len(matches) != 1:
        raise ValueError(f"point {xy} belongs to {len(matches)} regions")
    return matches[0]


def _validate_scenario(
    selection_path: Path,
    selection: dict,
    regions: tuple[CognitiveRegion, ...],
    catalog_ids: set[str],
) -> dict:
    references = selection["references"]
    validation = selection["validation"]
    waypoint_document = _load_mapping(
        _referenced_path(selection_path, references["waypoint_config"], "waypoint_config"),
        "waypoint config",
    )
    actual_waypoints = {"start": _region_for_point(regions, waypoint_document["start"])}
    actual_waypoints.update(
        (str(row["id"]), _region_for_point(regions, row["position"]))
        for row in waypoint_document["route"]
    )
    expected_waypoints = validation["waypoint_region_ids"]
    if actual_waypoints != expected_waypoints:
        raise ValueError("G1..G5 waypoint region sequence differs from selection")

    static_document = _load_mapping(
        _referenced_path(
            selection_path, references["static_obstacles_config"], "static_obstacles_config"
        ),
        "static obstacles config",
    )
    static_rows = [
        row for row in static_document["obstacles"]
        if row["id"] == validation["static_obstacle_id"]
    ]
    if len(static_rows) != 1 or _region_for_point(regions, static_rows[0]["start"]) != validation["static_obstacle_region_id"]:
        raise ValueError("static arc12 region differs from selection")

    dynamic_document = _load_mapping(
        _referenced_path(selection_path, references["dynamic_config"], "dynamic_config"),
        "dynamic config",
    )
    dynamic = dynamic_document["cases"][validation["dynamic_case_id"]]["obstacle"]
    dynamic_regions = {_region_for_point(regions, point) for point in dynamic["waypoints"]}
    if dynamic_regions != {validation["dynamic_region_id"]}:
        raise ValueError("dynamic crossing region differs from selection")

    sequence = [str(value) for value in validation["nominal_route_region_sequence"]]
    outside = sorted(set(sequence).difference(catalog_ids))
    expected_outside = float(validation["expected_route_outside_catalog_m"])
    if outside or expected_outside != 0.0:
        raise ValueError("nominal route is not fully covered by the selected catalog")
    return {
        "waypoint_region_ids": actual_waypoints,
        "nominal_region_sequence": sequence,
        "outside_catalog_region_ids": outside,
        "outside_catalog_m": 0.0,
        "static_obstacle_region_id": validation["static_obstacle_region_id"],
        "dynamic_region_id": validation["dynamic_region_id"],
    }


def build_catalog(
    *, map_yaml: str | Path, graph_geojson: str | Path,
    regions_yaml: str | Path, selection_yaml: str | Path,
    output_root: str | Path,
) -> dict:
    map_path = Path(map_yaml).expanduser().resolve()
    graph_path = Path(graph_geojson).expanduser().resolve()
    regions_path = Path(regions_yaml).expanduser().resolve()
    selection_path = Path(selection_yaml).expanduser().resolve()
    destination = Path(output_root).expanduser().resolve()
    if destination.exists():
        raise FileExistsError(f"output root already exists: {destination}")

    selection = _load_mapping(selection_path, "selection config")
    if selection.get("schema") != SELECTION_SCHEMA:
        raise ValueError("unsupported selection schema")
    references = selection.get("references")
    if not isinstance(references, dict):
        raise ValueError("selection references must be a mapping")
    _assert_reference(map_path, _referenced_path(selection_path, references["map_yaml"], "map_yaml"), "map")
    _assert_reference(graph_path, _referenced_path(selection_path, references["graph_geojson"], "graph_geojson"), "graph")
    _assert_reference(regions_path, _referenced_path(selection_path, references["region_config"], "region_config"), "regions")

    map_version = _runtime_map_version(map_path)
    if map_version != selection["expected_map_version"]:
        raise ValueError("map bytes do not match expected_map_version")
    map_bundle_sha256 = compute_occupancy_map_bundle_sha256(map_path)
    if map_bundle_sha256 != selection["map_bundle_sha256"]:
        raise ValueError("map YAML/image bundle differs from selection")
    graph = json.loads(graph_path.read_text(encoding="utf-8"))
    if graph.get("type") != "FeatureCollection" or not isinstance(graph.get("features"), list):
        raise ValueError("graph must be a GeoJSON FeatureCollection")
    algorithm = str(graph.get("name", "")).partition(":")[2]
    if f"{map_path.stem}:{algorithm}" != selection["graph_id"]:
        raise ValueError("graph identity differs from selection")

    grid_width = _require_int(selection["grid_width"], "grid_width")
    grid_height = _require_int(selection["grid_height"], "grid_height")
    if (grid_width, grid_height, float(selection["resolution_m"])) != (16, 16, 1.0):
        raise ValueError("catalog grid must be 16x16 at 1.0 m")
    graph_revision = _require_int(selection["graph_revision"], "graph_revision")
    tile_revision = _require_int(selection["tile_revision"], "tile_revision")
    if graph_revision != 1 or tile_revision != 1:
        raise ValueError("Rivermark catalog revisions must both be 1")

    config = load_region_config(regions_path)
    if config.scene_id != selection["scene_id"]:
        raise ValueError("region scene_id differs from selection")
    region_by_id = {region.region_id: region for region in config.regions}
    route_ids = [str(value) for value in selection["route_region_ids"]]
    ring_ids = [str(value) for value in selection["ring_region_ids"]]
    if len(route_ids) != 16 or len(ring_ids) != 14 or set(route_ids) & set(ring_ids):
        raise ValueError("selection must contain disjoint route16 and ring14 IDs")
    selected_ids = route_ids + ring_ids
    if len(set(selected_ids)) != 30 or set(selected_ids).difference(region_by_id):
        raise ValueError("selection must name exactly 30 known regions")
    selected = tuple(region_by_id[identifier] for identifier in sorted(selected_ids))
    coverage = _validate_scenario(selection_path, selection, config.regions, set(selected_ids))

    occupancy = load_occupancy_map(
        map_path, unknown_is_occupied=bool(selection["unknown_is_occupied"])
    )
    entries = []
    constraint_payloads: list[tuple[Path, dict]] = []
    for region in selected:
        value = build_cognitive_constraints(
            occupancy,
            map_version=map_version,
            graph_revision=graph_revision,
            footprint_settings=selection["footprint"],
            t_map_canvas=region.t_map_canvas,
            cognitive_tile_id=region.region_id,
        )
        transitions = value.verified_transitions.astype(int).tolist()
        mask = [bool(item) for item in value.reachable_state_mask.tolist()]
        if len(mask) != 256 or len({tuple(edge) for edge in transitions}) != len(transitions):
            raise ValueError(f"invalid constraints for {region.region_id}")
        for source, target in transitions:
            source_row, source_column = divmod(source, 16)
            target_row, target_column = divmod(target, 16)
            if not mask[source] or not mask[target] or abs(source_row - target_row) + abs(source_column - target_column) != 1:
                raise ValueError(f"non-cardinal or invalid transition for {region.region_id}")
        suffix = region.region_id.rsplit("_", 1)[-1]
        constraints_relpath = Path("constraints") / f"region_{suffix}.json"
        snapshot_relpath = Path("snapshots") / f"region_{suffix}"
        constraints = {
            "grid_width": grid_width,
            "grid_height": grid_height,
            "resolution_m": 1.0,
            "map_version": map_version,
            "cognitive_tile_id": region.region_id,
            "tile_revision": tile_revision,
            "graph_revision": graph_revision,
            "model_id": selection["model_id"],
            "valid_state_count": int(value.reachable_state_mask.sum()),
            "verified_directed_transition_count": len(transitions),
            "T_map_canvas": value.t_map_canvas.tolist(),
            "valid_state_mask": mask,
            "verified_transitions": transitions,
        }
        constraint_payloads.append((constraints_relpath, constraints))
        entries.append({
            "region_id": region.region_id,
            "cognitive_tile_id": region.region_id,
            "role": "route" if region.region_id in route_ids else "ring",
            "map_version": map_version,
            "tile_revision": tile_revision,
            "graph_revision": graph_revision,
            "T_map_canvas": value.t_map_canvas.tolist(),
            "core_bounds_map": _bounds(region),
            "canvas_bounds_map": _canvas_bounds(region),
            "neighbor_tile_ids": _cardinal_neighbors(region, selected),
            "valid_state_count": constraints["valid_state_count"],
            "verified_directed_transition_count": len(transitions),
            "constraints_relpath": constraints_relpath.as_posix(),
            "snapshot_relpath": snapshot_relpath.as_posix(),
        })

    catalog = {
        "schema": CATALOG_SCHEMA,
        "scene_id": selection["scene_id"],
        "scenario_map_revision": selection["scenario_map_revision"],
        "map_bundle_sha256": selection["map_bundle_sha256"],
        "map_version": map_version,
        "graph_id": selection["graph_id"],
        "graph_revision": graph_revision,
        "tile_revision": tile_revision,
        "model_id": selection["model_id"],
        "grid_width": grid_width,
        "grid_height": grid_height,
        "resolution_m": 1.0,
        "region_count": len(entries),
        "route_region_ids": route_ids,
        "ring_region_ids": ring_ids,
        "route_coverage": coverage,
        "entries": entries,
    }
    destination.mkdir(parents=True)
    for relative, payload in constraint_payloads:
        target = destination / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (destination / "catalog.json").write_text(
        json.dumps(catalog, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return catalog


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", required=True)
    parser.add_argument("--graph", required=True)
    parser.add_argument("--regions", required=True)
    parser.add_argument("--selection", required=True)
    parser.add_argument("--output-root", required=True)
    args = parser.parse_args(argv)
    catalog = build_catalog(
        map_yaml=args.map,
        graph_geojson=args.graph,
        regions_yaml=args.regions,
        selection_yaml=args.selection,
        output_root=args.output_root,
    )
    print(json.dumps({"map_version": catalog["map_version"], "region_count": catalog["region_count"]}, sort_keys=True))


if __name__ == "__main__":
    main()
