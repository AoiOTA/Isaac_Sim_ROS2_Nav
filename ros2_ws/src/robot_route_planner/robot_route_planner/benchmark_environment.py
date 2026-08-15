"""Generate a collision/occupancy-consistent physical multi-route benchmark."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Mapping

import cv2
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np
import yaml

from .diagnostics import graph_diagnostics, physical_edges
from .feasibility import (
    apply_footprint_feasibility,
    retain_largest_feasible_component,
)
from .gvg import build_gvg, voronoi_layers
from .map_io import OccupancyMap
from .models import Graph, Traversability
from .multiroute import (
    MultiRouteQuery,
    analyze_multiroute_queries,
    k_shortest_loopless_routes,
    save_multiroute_analysis,
    select_stratified_queries,
)
from .route_support import export_route_support_graph, save_route_support


@dataclass(frozen=True)
class PhysicalBox:
    identifier: str
    role: str
    center_xyz: tuple[float, float, float]
    size_xyz: tuple[float, float, float]
    yaw_deg: float


@dataclass(frozen=True)
class BenchmarkLayout:
    name: str
    revision: int
    status: str
    bounds_xy: tuple[float, float, float, float]
    resolution_m: float
    base_asset_relative_path: str
    obstacles: tuple[PhysicalBox, ...]


def _numbers(value: object, size: int, context: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != size:
        raise ValueError(f"{context} must contain {size} numbers")
    parsed = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in parsed):
        raise ValueError(f"{context} must be finite")
    return parsed


def load_benchmark_layout(path: str | Path) -> BenchmarkLayout:
    source = Path(path).expanduser().resolve()
    data = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(data, Mapping) or set(data) != {
        "schema_version",
        "name",
        "revision",
        "status",
        "classification",
        "bounds_xy_m",
        "resolution_m",
        "base_environment",
        "obstacles",
    }:
        raise ValueError("benchmark layout has unexpected fields")
    if data["schema_version"] != 1:
        raise ValueError("benchmark layout schema_version must be 1")
    if data["classification"] != "engineering_benchmark_only":
        raise ValueError("benchmark layout classification is invalid")
    name = data["name"]
    if not isinstance(name, str) or re.fullmatch(r"[a-z0-9_]+", name) is None:
        raise ValueError("benchmark layout name is invalid")
    revision = data["revision"]
    if not isinstance(revision, int) or isinstance(revision, bool) or revision < 1:
        raise ValueError("benchmark layout revision is invalid")
    status = data["status"]
    if status not in {"rejected_baseline", "candidate", "frozen"}:
        raise ValueError("benchmark layout status is invalid")
    bounds = _numbers(data["bounds_xy_m"], 4, "bounds_xy_m")
    if bounds[0] >= bounds[2] or bounds[1] >= bounds[3]:
        raise ValueError("benchmark bounds are invalid")
    resolution = float(data["resolution_m"])
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError("benchmark resolution must be positive")
    width = (bounds[2] - bounds[0]) / resolution
    height = (bounds[3] - bounds[1]) / resolution
    if abs(width - round(width)) > 1.0e-9 or abs(height - round(height)) > 1.0e-9:
        raise ValueError("benchmark bounds must be an integer number of cells")
    base = data["base_environment"]
    if not isinstance(base, Mapping) or set(base) != {"asset_relative_path"}:
        raise ValueError("base_environment is invalid")
    relative = base["asset_relative_path"]
    if relative != "Isaac/Environments/Grid/default_environment.usd":
        raise ValueError("benchmark base must be NVIDIA Flat Grid default_environment")
    raw_obstacles = data["obstacles"]
    if not isinstance(raw_obstacles, list) or not raw_obstacles:
        raise ValueError("benchmark requires physical obstacles")
    obstacles = []
    identifiers = set()
    for raw in raw_obstacles:
        if not isinstance(raw, Mapping) or set(raw) != {
            "id", "role", "center", "size", "yaw_deg"
        }:
            raise ValueError("benchmark obstacle has unexpected fields")
        identifier = raw["id"]
        if (
            not isinstance(identifier, str)
            or re.fullmatch(r"[a-z][a-z0-9_]*", identifier) is None
            or identifier in identifiers
        ):
            raise ValueError(f"invalid or duplicate obstacle id {identifier!r}")
        identifiers.add(identifier)
        role = raw["role"]
        if role not in {
            "outer_boundary",
            "courtyard_block",
            "structural_wall",
            "bottleneck_wall",
            "dead_end_wall",
        }:
            raise ValueError(f"invalid obstacle role {role!r}")
        center = _numbers(raw["center"], 3, f"{identifier}.center")
        size = _numbers(raw["size"], 3, f"{identifier}.size")
        yaw_deg = float(raw["yaw_deg"])
        if not math.isfinite(yaw_deg):
            raise ValueError(f"{identifier}.yaw_deg must be finite")
        if min(size) <= 0.0 or center[2] - size[2] / 2.0 < -1.0e-9:
            raise ValueError(f"invalid physical extent for {identifier}")
        yaw = math.radians(yaw_deg)
        half_x, half_y = size[0] / 2.0, size[1] / 2.0
        corners = [
            (
                center[0] + math.cos(yaw) * x - math.sin(yaw) * y,
                center[1] + math.sin(yaw) * x + math.cos(yaw) * y,
            )
            for x, y in (
                (-half_x, -half_y),
                (half_x, -half_y),
                (half_x, half_y),
                (-half_x, half_y),
            )
        ]
        if (
            min(point[0] for point in corners) < bounds[0] - 1.0e-9
            or max(point[0] for point in corners) > bounds[2] + 1.0e-9
            or min(point[1] for point in corners) < bounds[1] - 1.0e-9
            or max(point[1] for point in corners) > bounds[3] + 1.0e-9
        ):
            raise ValueError(f"physical box {identifier} leaves the 16x16 m bounds")
        obstacles.append(PhysicalBox(identifier, role, center, size, yaw_deg))
    return BenchmarkLayout(
        name, revision, status, bounds, resolution, str(relative), tuple(obstacles)
    )


def rasterize_layout(layout: BenchmarkLayout, yaml_path: str | Path) -> OccupancyMap:
    left, bottom, right, top = layout.bounds_xy
    width = int(round((right - left) / layout.resolution_m))
    height = int(round((top - bottom) / layout.resolution_m))
    free = np.ones((height, width), dtype=bool)
    for obstacle in layout.obstacles:
        cx, cy, _ = obstacle.center_xyz
        sx, sy, _ = obstacle.size_xyz
        yaw = math.radians(obstacle.yaw_deg)
        points = []
        for local_x, local_y in (
            (-sx / 2.0, -sy / 2.0),
            (sx / 2.0, -sy / 2.0),
            (sx / 2.0, sy / 2.0),
            (-sx / 2.0, sy / 2.0),
        ):
            x = cx + math.cos(yaw) * local_x - math.sin(yaw) * local_y
            y = cy + math.sin(yaw) * local_x + math.cos(yaw) * local_y
            column = int(round((x - left) / layout.resolution_m))
            row = height - int(round((y - bottom) / layout.resolution_m))
            points.append((column, row))
        occupied = (~free).astype(np.uint8)
        cv2.fillConvexPoly(occupied, np.asarray(points, dtype=np.int32), 1)
        free = ~occupied.astype(bool)
    return OccupancyMap(
        free,
        layout.resolution_m,
        (left, bottom),
        layout.name,
        Path(yaml_path).expanduser().resolve(),
    )


def _write_map(occupancy: OccupancyMap, output_dir: Path) -> tuple[Path, Path]:
    pgm = output_dir / f"{occupancy.map_version}.pgm"
    yaml_path = output_dir / f"{occupancy.map_version}.yaml"
    if not cv2.imwrite(str(pgm), np.where(occupancy.free, 254, 0).astype(np.uint8)):
        raise RuntimeError(f"failed to write {pgm}")
    document = {
        "image": pgm.name,
        "mode": "trinary",
        "resolution": occupancy.resolution_m,
        "origin": [occupancy.origin_xy_m[0], occupancy.origin_xy_m[1], 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }
    yaml_path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    return pgm, yaml_path


def _write_usda(layout: BenchmarkLayout, flat_grid: Path, destination: Path) -> None:
    lines = [
        "#usda 1.0",
        "(",
        '    defaultPrim = "World"',
        "    metersPerUnit = 1",
        '    upAxis = "Z"',
        ")",
        "",
        f'def Xform "World" (prepend references = @{flat_grid.as_posix()}@</World>)',
        "{",
        '    def Xform "BenchmarkGeometry"',
        "    {",
    ]
    colors = {
        "outer_boundary": (0.12, 0.22, 0.36),
        "courtyard_block": (0.28, 0.48, 0.66),
        "structural_wall": (0.22, 0.42, 0.62),
        "bottleneck_wall": (0.58, 0.38, 0.18),
        "dead_end_wall": (0.42, 0.25, 0.55),
    }
    for item in layout.obstacles:
        cx, cy, cz = item.center_xyz
        sx, sy, sz = item.size_xyz
        r, g, b = colors[item.role]
        lines.extend(
            [
                f'        def Cube "{item.identifier}" (prepend apiSchemas = ["PhysicsCollisionAPI"])',
                "        {",
                "            double size = 1",
                "            bool physics:collisionEnabled = 1",
                f"            color3f[] primvars:displayColor = [({r}, {g}, {b})]",
                f"            double xformOp:rotateZ = {item.yaw_deg}",
                f"            double3 xformOp:scale = ({sx}, {sy}, {sz})",
                f"            double3 xformOp:translate = ({cx}, {cy}, {cz})",
                '            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateZ", "xformOp:scale"]',
                "        }",
            ]
        )
    lines.extend(["    }", "}", ""])
    destination.write_text("\n".join(lines), encoding="utf-8")


def _query_pose(node_lookup, query) -> tuple[float, float, float, float, float, float]:
    route = query.alternatives[0]
    start = node_lookup[route.node_ids[0]].position_xy
    start_next = node_lookup[route.node_ids[1]].position_xy
    goal_previous = node_lookup[route.node_ids[-2]].position_xy
    goal = node_lookup[route.node_ids[-1]].position_xy
    return (
        float(start[0]),
        float(start[1]),
        math.degrees(math.atan2(start_next[1] - start[1], start_next[0] - start[0])),
        float(goal[0]),
        float(goal[1]),
        math.degrees(math.atan2(goal[1] - goal_previous[1], goal[0] - goal_previous[0])),
    )


def _directed_execution_query(
    graph: Graph,
    start_node: int,
    goal_node: int,
    route_cost_settings: Mapping[str, float],
) -> MultiRouteQuery:
    """Build the same physical-route query contract for an explicit direction."""

    alternatives = k_shortest_loopless_routes(
        graph, start_node, goal_node, route_cost_settings, k=8
    )
    if len(alternatives) < 2:
        raise RuntimeError(
            f"focused query {start_node}->{goal_node} has fewer than two routes"
        )
    edge_lookup = graph.edge_by_id()

    def physical_keys(route) -> frozenset[tuple[int, int]]:
        return frozenset(
            tuple(
                sorted(
                    (
                        edge_lookup[edge_id].from_node,
                        edge_lookup[edge_id].to_node,
                    )
                )
            )
            for edge_id in route.edge_ids
        )

    shortest = alternatives[0]
    shortest_keys = physical_keys(shortest)
    meaningful = [shortest]
    second_overlap = None
    for alternative in alternatives[1:]:
        keys = physical_keys(alternative)
        overlap = len(keys & shortest_keys) / max(
            1, min(len(keys), len(shortest_keys))
        )
        if alternative.cost_m / shortest.cost_m <= 1.60 and overlap <= 0.75:
            meaningful.append(alternative)
            if second_overlap is None:
                second_overlap = overlap
    if len(meaningful) < 2 or second_overlap is None:
        raise RuntimeError(
            f"focused query {start_node}->{goal_node} has fewer than two meaningful routes"
        )
    nodes = graph.node_by_id()
    separation = math.dist(
        nodes[start_node].position_xy, nodes[goal_node].position_xy
    )
    second_ratio = meaningful[1].cost_m / shortest.cost_m
    route_priority = min(len(meaningful), 4)
    return MultiRouteQuery(
        query_id=f"Q{start_node:02d}_{goal_node:02d}",
        start_node=start_node,
        goal_node=goal_node,
        euclidean_separation_m=float(separation),
        shortest_cost_m=shortest.cost_m,
        second_cost_ratio=float(second_ratio),
        meaningful_route_count=len(meaningful),
        enumerated_route_count=len(alternatives),
        shortest_second_edge_overlap=float(second_overlap),
        score=float(
            1000.0 * route_priority
            + 25.0 * min(separation, 16.0)
            + 100.0 * (1.0 - second_overlap)
            - 100.0 * (second_ratio - 1.0)
        ),
        alternatives=tuple(meaningful),
    )


def _write_execution_candidates(
    layout: BenchmarkLayout,
    graph: Graph,
    queries,
    route_cost_settings: Mapping[str, float],
    bundle_sha256: str,
    json_path: Path,
    spawn_path: Path,
) -> list[dict[str, object]]:
    by_tier = {
        tier: [
            query
            for query in queries
            if min(query.meaningful_route_count, 4) == tier
        ]
        for tier in (2, 3, 4)
    }
    if any(len(by_tier[tier]) < 2 for tier in (3, 4)) or not by_tier[2]:
        raise RuntimeError("benchmark lacks the requested execution query tiers")
    representative = [
        by_tier[2][0],
        by_tier[3][0],
        by_tier[4][0],
        by_tier[3][1],
        by_tier[4][1],
    ]
    selected = []
    if layout.name == "attempt30_a21_multiroute_v4":
        by_pair = {
            (query.start_node, query.goal_node): query for query in queries
        }
        for pair in ((2, 58), (1, 50), (36, 4), (14, 45)):
            selected.append(
                by_pair.get(pair)
                or _directed_execution_query(
                    graph, pair[0], pair[1], route_cost_settings
                )
            )
    selected.extend(
        query
        for query in representative
        if all(query.query_id != existing.query_id for existing in selected)
    )
    node_lookup = graph.node_by_id()
    records = []
    spawn_poses = {}
    for query in selected:
        tier = min(query.meaningful_route_count, 4)
        start_x, start_y, start_yaw, goal_x, goal_y, goal_yaw = _query_pose(
            node_lookup, query
        )
        profile = f"{query.query_id.lower()}_start"
        spawn_poses[profile] = {
            "usd": {
                "position": [start_x, start_y, 0.0635],
                "yaw_deg": start_yaw,
            },
            "map": {
                "position": [start_x, start_y],
                "yaw_deg": start_yaw,
                "calibrated": True,
                "map_version": layout.name,
                "map_bundle_sha256": bundle_sha256,
                "position_stddev_m": 0.0,
                "yaw_stddev_deg": 0.0,
            },
        }
        records.append(
            {
                "query_id": query.query_id,
                "route_tier": tier,
                "meaningful_route_count": query.meaningful_route_count,
                "spawn_pose_name": profile,
                "start_node": query.start_node,
                "goal_node": query.goal_node,
                "start_xy_yaw_deg": [start_x, start_y, start_yaw],
                "goal_xy_yaw_deg": [goal_x, goal_y, goal_yaw],
                "shortest_cost_m": query.shortest_cost_m,
                "second_cost_ratio": query.second_cost_ratio,
                "shortest_second_edge_overlap": query.shortest_second_edge_overlap,
                "alternatives": [
                    {
                        "cost_m": route.cost_m,
                        "node_ids": list(route.node_ids),
                        "edge_ids": list(route.edge_ids),
                    }
                    for route in query.alternatives
                ],
            }
        )
    json_path.write_text(
        json.dumps(records, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    spawn_path.write_text(
        yaml.safe_dump(
            {"schema_version": 1, "spawn_poses": spawn_poses}, sort_keys=False
        ),
        encoding="utf-8",
    )
    return records


def _write_overview(
    layout: BenchmarkLayout,
    occupancy: OccupancyMap,
    graph,
    queries,
    defaults: Mapping[str, object],
    destination: Path,
) -> None:
    _, skeleton, clearance = voronoi_layers(
        occupancy,
        obstacle_site_min_separation_cells=float(defaults["graph"]["obstacle_site_min_separation_cells"]),
        obstacle_source_min_angle_deg=float(defaults["graph"]["obstacle_source_min_angle_deg"]),
        ridge_distance_difference_cells=float(defaults["graph"]["ridge_distance_difference_cells"]),
        minimum_free_component_area_m2=float(defaults["graph"]["minimum_free_component_area_m2"]),
        topology_clearance_m=float(defaults["footprint"]["padded_inscribed_radius_m"]),
    )
    extent = [
        layout.bounds_xy[0],
        layout.bounds_xy[2],
        layout.bounds_xy[1],
        layout.bounds_xy[3],
    ]
    figure, axes = plt.subplots(1, 3, figsize=(18, 6), constrained_layout=True)
    for axis in axes:
        axis.imshow(
            occupancy.free.astype(float),
            cmap="gray",
            origin="upper",
            extent=extent,
            vmin=0.0,
            vmax=1.0,
            interpolation="nearest",
        )
        axis.set_aspect("equal")
        axis.set_xlim(extent[0], extent[1])
        axis.set_ylim(extent[2], extent[3])
        axis.set_xlabel("map x [m]")
    axes[0].set_title("Physical geometry / occupancy")
    for item in layout.obstacles:
        cx, cy, _ = item.center_xyz
        sx, sy, _ = item.size_xyz
        yaw = math.radians(item.yaw_deg)
        corners = np.asarray([
            (
                cx + math.cos(yaw) * x - math.sin(yaw) * y,
                cy + math.sin(yaw) * x + math.cos(yaw) * y,
            )
            for x, y in (
                (-sx / 2.0, -sy / 2.0),
                (sx / 2.0, -sy / 2.0),
                (sx / 2.0, sy / 2.0),
                (-sx / 2.0, sy / 2.0),
            )
        ])
        axes[0].add_patch(Polygon(corners, fill=False, edgecolor="#ef4444", linewidth=1.0))
    rgba = np.zeros((*skeleton.shape, 4), dtype=float)
    rgba[skeleton, :] = (0.0, 0.8, 1.0, 0.85)
    axes[1].imshow(rgba, origin="upper", extent=extent, interpolation="nearest")
    axes[1].set_title("Footprint-qualified GVD/GVG skeleton")
    node_lookup = graph.node_by_id()
    for edge in physical_edges(graph):
        axes[1].plot(edge.polyline_xy[:, 0], edge.polyline_xy[:, 1], color="#f59e0b", linewidth=1.2)
    for node in graph.nodes:
        axes[1].scatter(*node.position_xy, s=24, c="#d946ef", zorder=5)
        axes[1].text(*node.position_xy, str(node.id), fontsize=7, ha="left", va="bottom")
    axes[2].set_title("Representative real multi-route queries")
    representatives = []
    for tier in (2, 3, 4):
        eligible = [q for q in queries if min(q.meaningful_route_count, 4) == tier]
        if eligible:
            representatives.append(eligible[0])
    colors = ("#22c55e", "#8b5cf6", "#f97316")
    edge_lookup = graph.edge_by_id()
    for color, query in zip(colors, representatives):
        for index, route in enumerate(query.alternatives[: min(query.meaningful_route_count, 4)]):
            points = []
            for edge_id in route.edge_ids:
                polyline = edge_lookup[edge_id].polyline_xy
                if points:
                    points.extend(polyline[1:])
                else:
                    points.extend(polyline)
            points = np.asarray(points)
            axes[2].plot(points[:, 0], points[:, 1], color=color, alpha=max(0.25, 0.95 - 0.18 * index), linewidth=2.0 - 0.2 * index)
        start = node_lookup[query.start_node].position_xy
        goal = node_lookup[query.goal_node].position_xy
        axes[2].scatter(*start, marker="o", s=45, c=color, zorder=6)
        axes[2].scatter(*goal, marker="*", s=75, c=color, zorder=6)
        axes[2].text(start[0], start[1], query.query_id, fontsize=7, color=color)
    figure.suptitle(
        f"Attempt30/A21 {layout.name} r{layout.revision} — physical geometry → occupancy → GVD/GVG",
        fontsize=13,
    )
    figure.savefig(destination, dpi=180)
    plt.close(figure)


def generate_benchmark_bundle(
    layout_path: str | Path,
    defaults_path: str | Path,
    asset_root: str | Path,
    output_dir: str | Path,
) -> dict[str, object]:
    layout_source = Path(layout_path).expanduser().resolve()
    layout = load_benchmark_layout(layout_source)
    defaults_source = Path(defaults_path).expanduser().resolve()
    defaults = yaml.safe_load(defaults_source.read_text(encoding="utf-8"))
    flat_grid = Path(asset_root).expanduser().resolve() / layout.base_asset_relative_path
    if not flat_grid.is_file():
        raise FileNotFoundError(flat_grid)
    output = Path(output_dir).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    occupancy = rasterize_layout(layout, output / f"{layout.name}.yaml")
    pgm_path, map_path = _write_map(occupancy, output)
    graph = build_gvg(
        occupancy,
        defaults["graph"],
        defaults["footprint"],
        defaults["route_cost"],
        revision=layout.revision,
    )
    apply_footprint_feasibility(graph, occupancy, defaults["footprint"])
    feasibility_pruning = retain_largest_feasible_component(graph)
    export = export_route_support_graph(
        graph, support_spacing_m=float(defaults["graph"]["route_support_spacing_m"])
    )
    geojson_path = output / f"{layout.name}.geojson"
    mapping_path = output / f"{layout.name}.mapping.json"
    save_route_support(export, geojson_path, mapping_path)
    queries = analyze_multiroute_queries(graph, defaults["route_cost"])
    query_json = output / f"{layout.name}_queries.json"
    query_csv = output / f"{layout.name}_queries.csv"
    save_multiroute_analysis(queries, query_json, query_csv)
    shortlist = select_stratified_queries(queries)
    shortlist_json = output / f"{layout.name}_top15.json"
    shortlist_csv = output / f"{layout.name}_top15.csv"
    save_multiroute_analysis(shortlist, shortlist_json, shortlist_csv)
    usd_path = output / f"{layout.name}.usda"
    _write_usda(layout, flat_grid, usd_path)
    bundle_sha256 = hashlib.sha256(
        layout_source.read_bytes() + pgm_path.read_bytes() + map_path.read_bytes()
    ).hexdigest()
    execution_json = output / f"{layout.name}_execution_candidates.json"
    spawn_path = output / f"{layout.name}.spawn.yaml"
    execution_candidates = _write_execution_candidates(
        layout,
        graph,
        queries,
        defaults["route_cost"],
        bundle_sha256,
        execution_json,
        spawn_path,
    )
    overview = output / f"{layout.name}_overview.png"
    _write_overview(layout, occupancy, graph, queries, defaults, overview)
    diagnostics = graph_diagnostics(graph)
    degree_three = sum(node.degree >= 3 for node in graph.nodes)
    degree_four = sum(node.degree >= 4 for node in graph.nodes)
    infeasible = sum(
        edge.static_traversability == Traversability.INFEASIBLE
        for edge in graph.edges
    )
    tier_counts = {
        str(tier): sum(min(query.meaningful_route_count, 4) == tier for query in queries)
        for tier in (2, 3, 4)
    }
    minimum_targets_met = bool(
        diagnostics["component_count"] == 1
        and degree_three >= 10
        and diagnostics["cycle_count"] >= 6
        and len(queries) >= 15
        and tier_counts["3"] >= 2
        and tier_counts["4"] >= 1
        and infeasible == 0
    )
    summary = {
        "classification": "engineering_benchmark_only",
        "layout_status": layout.status,
        "minimum_targets_met": minimum_targets_met,
        "environment_frozen": bool(layout.status == "frozen" and minimum_targets_met),
        "freeze_reason": (
            "user-approved final geometry and minimum targets met"
            if layout.status == "frozen" and minimum_targets_met
            else None
        ),
        "layout_name": layout.name,
        "layout_revision": layout.revision,
        "bounds_xy_m": list(layout.bounds_xy),
        "area_m2": (layout.bounds_xy[2] - layout.bounds_xy[0]) * (layout.bounds_xy[3] - layout.bounds_xy[1]),
        "resolution_m": layout.resolution_m,
        "base_environment": {
            "requested_uri": "/Isaac/Environments/Grid/default_environment.usd",
            "resolved_path": str(flat_grid),
            "sha256": hashlib.sha256(flat_grid.read_bytes()).hexdigest(),
        },
        "physical_obstacle_count": len(layout.obstacles),
        "physical_geometry_sha256": hashlib.sha256(layout_source.read_bytes()).hexdigest(),
        "map_sha256": hashlib.sha256(pgm_path.read_bytes()).hexdigest(),
        "graph": {
            **diagnostics,
            **feasibility_pruning,
            "degree_ge_3_count": degree_three,
            "degree_ge_4_count": degree_four,
            "infeasible_directed_edge_count": infeasible,
        },
        "multi_route": {
            "eligible_query_count": len(queries),
            "tier_counts": tier_counts,
            "top15_query_ids": [query.query_id for query in shortlist],
            "execution_query_ids": [
                item["query_id"] for item in execution_candidates
            ],
        },
        "artifacts": {
            "map_yaml": map_path.name,
            "map_pgm": pgm_path.name,
            "navigation_graph": geojson_path.name,
            "route_mapping": mapping_path.name,
            "queries_json": query_json.name,
            "queries_csv": query_csv.name,
            "top15_json": shortlist_json.name,
            "top15_csv": shortlist_csv.name,
            "execution_candidates": execution_json.name,
            "spawn_poses": spawn_path.name,
            "usd": usd_path.name,
            "overview_png": overview.name,
        },
    }
    summary_path = output / f"{layout.name}_summary.json"
    summary_path.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return summary


__all__ = [
    "BenchmarkLayout",
    "PhysicalBox",
    "generate_benchmark_bundle",
    "load_benchmark_layout",
    "rasterize_layout",
    "retain_largest_feasible_component",
]
