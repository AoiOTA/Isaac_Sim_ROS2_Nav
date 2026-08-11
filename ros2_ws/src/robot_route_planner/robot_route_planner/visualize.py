"""Export context-rich A21 engineering overlays for human visual review."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict, deque
import heapq
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np

from .defaults import load_engineering_defaults
from .diagnostics import graph_diagnostics
from .feasibility import apply_footprint_feasibility
from .gvg import build_gvg, voronoi_layers
from .map_io import load_occupancy_map
from .models import Traversability
from .tracking import RouteTracker


def _extent(occupancy):
    height, width = occupancy.free.shape
    return [
        occupancy.origin_xy_m[0],
        occupancy.origin_xy_m[0] + width * occupancy.resolution_m,
        occupancy.origin_xy_m[1],
        occupancy.origin_xy_m[1] + height * occupancy.resolution_m,
    ]


def _map_background(axis, occupancy) -> None:
    axis.imshow(
        occupancy.free.astype(float),
        cmap="gray",
        origin="upper",
        extent=_extent(occupancy),
        vmin=0.0,
        vmax=1.0,
        interpolation="nearest",
    )
    axis.set_aspect("equal")
    axis.set_xlabel("map x [m]")
    axis.set_ylabel("map y [m]")


def _pixel_overlay(axis, occupancy, mask, color, alpha=0.8) -> None:
    rgba = np.zeros((*mask.shape, 4), dtype=float)
    rgba[mask, :3] = color
    rgba[mask, 3] = alpha
    axis.imshow(
        rgba,
        origin="upper",
        extent=_extent(occupancy),
        interpolation="nearest",
    )


def _plot_graph(axis, graph, *, alpha=0.75, linewidth=0.65) -> None:
    for edge in graph.edges:
        if edge.from_node > edge.to_node:
            continue
        axis.plot(
            edge.polyline_xy[:, 0],
            edge.polyline_xy[:, 1],
            color="#00bcd4",
            linewidth=linewidth,
            alpha=alpha,
            zorder=3,
        )
    endpoint = np.asarray(
        [node.position_xy for node in graph.nodes if node.node_type.name == "ENDPOINT"]
    )
    junction = np.asarray(
        [node.position_xy for node in graph.nodes if node.node_type.name == "JUNCTION"]
    )
    loop = np.asarray(
        [node.position_xy for node in graph.nodes if node.node_type.name == "LOOP_ANCHOR"]
    )
    if len(endpoint):
        axis.scatter(endpoint[:, 0], endpoint[:, 1], s=8, c="#ffd54f", label="endpoint", zorder=5)
    if len(junction):
        axis.scatter(junction[:, 0], junction[:, 1], s=9, c="#e040fb", label="junction", zorder=5)
    if len(loop):
        axis.scatter(loop[:, 0], loop[:, 1], s=12, c="#ff7043", label="loop anchor", zorder=5)


def _route_from_geojson(geojson: dict, start: int, goal: int, closed: set[int]):
    outgoing = defaultdict(list)
    edge_feature = {}
    for feature in geojson["features"]:
        if feature["geometry"]["type"] != "MultiLineString":
            continue
        properties = feature["properties"]
        edge_id = int(properties["id"])
        edge_feature[edge_id] = feature
        length = float(properties.get("metadata", {}).get("length_m", 1.0))
        outgoing[int(properties["startid"])].append(
            (int(properties["endid"]), edge_id, length)
        )
    distance = {int(start): 0.0}
    previous = {}
    queue = [(0.0, int(start))]
    while queue:
        cost, node = heapq.heappop(queue)
        if cost != distance[node]:
            continue
        if node == int(goal):
            break
        for target, edge_id, length in outgoing[node]:
            if edge_id in closed:
                continue
            proposal = cost + length
            if proposal < distance.get(target, math.inf):
                distance[target] = proposal
                previous[target] = (node, edge_id)
                heapq.heappush(queue, (proposal, target))
    if int(goal) not in distance:
        raise RuntimeError("visual route has no graph path")
    edge_ids = []
    node = int(goal)
    while node != int(start):
        node, edge_id = previous[node]
        edge_ids.append(edge_id)
    edge_ids.reverse()
    return edge_ids, edge_feature, float(distance[int(goal)])


def _route_coordinates(edge_ids, features):
    coordinates = []
    for edge_id in edge_ids:
        segment = features[edge_id]["geometry"]["coordinates"][0]
        if not coordinates:
            coordinates.append(segment[0])
        coordinates.append(segment[-1])
    return np.asarray(coordinates, dtype=float)


def _select_route_endpoints(geojson: dict) -> tuple[int, int]:
    adjacency = defaultdict(set)
    for feature in geojson["features"]:
        if feature["geometry"]["type"] != "MultiLineString":
            continue
        properties = feature["properties"]
        start = int(properties["startid"])
        end = int(properties["endid"])
        adjacency[start].add(end)
        adjacency[end].add(start)
    unseen = set(adjacency)
    components = []
    while unseen:
        component = {unseen.pop()}
        queue = deque(component)
        while queue:
            node = queue.popleft()
            for other in adjacency[node]:
                if other in unseen:
                    unseen.remove(other)
                    component.add(other)
                    queue.append(other)
        components.append(component)
    largest = max(components, key=lambda value: (len(value), -min(value)))

    def farthest(start: int) -> int:
        distance = {start: 0}
        queue = deque([start])
        while queue:
            node = queue.popleft()
            for other in adjacency[node]:
                if other in largest and other not in distance:
                    distance[other] = distance[node] + 1
                    queue.append(other)
        return max(distance, key=lambda node: (distance[node], -node))

    first = farthest(min(largest))
    return first, farthest(first)


def _graph_components(graph) -> int:
    adjacency = defaultdict(set)
    for edge in graph.edges:
        adjacency[edge.from_node].add(edge.to_node)
        adjacency[edge.to_node].add(edge.from_node)
    unseen = {node.id for node in graph.nodes}
    count = 0
    while unseen:
        count += 1
        queue = deque([unseen.pop()])
        while queue:
            node = queue.popleft()
            for other in adjacency[node]:
                if other in unseen:
                    unseen.remove(other)
                    queue.append(other)
    return count


def _draw_footprint(axis, center, yaw, polygon, color) -> None:
    rotation = np.asarray(
        [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
    )
    points = np.asarray(polygon) @ rotation.T + np.asarray(center)
    points = np.vstack((points, points[0]))
    axis.plot(points[:, 0], points[:, 1], color=color, linewidth=1.4, zorder=7)


def export_visuals(
    map_path: Path,
    defaults_path: Path,
    geojson_path: Path,
    mapping_path: Path,
    output_dir: Path,
) -> dict:
    output_dir.mkdir(parents=True, exist_ok=True)
    defaults = load_engineering_defaults(defaults_path)
    occupancy = load_occupancy_map(
        map_path,
        unknown_is_occupied=bool(defaults["graph"]["unknown_is_occupied"]),
    )
    raw, skeleton, clearance = voronoi_layers(
        occupancy,
        obstacle_site_min_separation_cells=float(
            defaults["graph"]["obstacle_site_min_separation_cells"]
        ),
        obstacle_source_min_angle_deg=float(
            defaults["graph"]["obstacle_source_min_angle_deg"]
        ),
        ridge_distance_difference_cells=float(
            defaults["graph"]["ridge_distance_difference_cells"]
        ),
        minimum_free_component_area_m2=float(
            defaults["graph"]["minimum_free_component_area_m2"]
        ),
        topology_clearance_m=float(
            defaults["footprint"]["padded_inscribed_radius_m"]
        ),
    )
    graph = apply_footprint_feasibility(
        build_gvg(
            occupancy,
            defaults["graph"],
            defaults["footprint"],
            defaults["route_cost"],
        ),
        occupancy,
        defaults["footprint"],
    )
    unpruned_settings = dict(defaults["graph"])
    unpruned_settings["spur_max_length_m"] = 0.0
    unpruned_graph = build_gvg(
        occupancy,
        unpruned_settings,
        defaults["footprint"],
        defaults["route_cost"],
    )
    probe_start, probe_goal = (
        graph_diagnostics(graph)["start_node"],
        graph_diagnostics(graph)["goal_node"],
    )
    before_choice = graph_diagnostics(
        unpruned_graph, start_node=probe_start, goal_node=probe_goal
    )
    after_choice = graph_diagnostics(
        graph, start_node=probe_start, goal_node=probe_goal
    )

    figure, axes = plt.subplots(2, 2, figsize=(13, 16), constrained_layout=True)
    _map_background(axes[0, 0], occupancy)
    axes[0, 0].set_title("Occupancy / structural map")
    field = np.ma.masked_where(~occupancy.free, clearance)
    image = axes[0, 1].imshow(
        field,
        cmap="viridis",
        origin="upper",
        extent=_extent(occupancy),
        interpolation="nearest",
    )
    axes[0, 1].set_aspect("equal")
    axes[0, 1].set_title("Exact L2 clearance field [m]")
    figure.colorbar(image, ax=axes[0, 1], shrink=0.7)
    _map_background(axes[1, 0], occupancy)
    _pixel_overlay(axes[1, 0], occupancy, raw, (1.0, 0.2, 0.1), 0.75)
    axes[1, 0].set_title("Raw labeled-site GVD ridge candidates")
    _map_background(axes[1, 1], occupancy)
    _pixel_overlay(axes[1, 1], occupancy, skeleton, (0.0, 0.9, 1.0), 0.95)
    axes[1, 1].set_title("Guo-Hall thinned GVD skeleton")
    phase1 = output_dir / "phase1_gvd_overlay.png"
    figure.savefig(phase1, dpi=180)
    plt.close(figure)

    figure, axes = plt.subplots(1, 3, figsize=(24, 10), constrained_layout=True)
    _map_background(axes[0], occupancy)
    _pixel_overlay(axes[0], occupancy, raw, (1.0, 0.2, 0.1), 0.45)
    _pixel_overlay(axes[0], occupancy, skeleton, (0.0, 0.9, 1.0), 0.9)
    axes[0].set_title("GVD diagnostics: raw ridge + topology backbone")
    _map_background(axes[1], occupancy)
    _plot_graph(axes[1], unpruned_graph)
    axes[1].set_title(
        "Before endpoint-spur pruning: "
        f"{before_choice['node_count']} nodes / "
        f"{before_choice['physical_edge_count']} edges / "
        f"{before_choice['cycle_count']} cycle"
    )
    _map_background(axes[2], occupancy)
    _plot_graph(axes[2], graph)
    axes[2].set_title(
        "After pruning + degree-2 polyline simplification: "
        f"{after_choice['node_count']} nodes / "
        f"{after_choice['physical_edge_count']} edges / "
        f"{after_choice['cycle_count']} cycle\n"
        f"probe {probe_start}->{probe_goal}: "
        f"{after_choice['start_goal_route_count']} routes"
    )
    axes[2].legend(loc="upper right", fontsize=8)
    phase2 = output_dir / "phase2_gvg_before_after_pruning.png"
    figure.savefig(phase2, dpi=180)
    plt.close(figure)

    colors = {
        Traversability.FEASIBLE: "#00c853",
        Traversability.UNKNOWN: "#ffab00",
        Traversability.INFEASIBLE: "#d50000",
    }
    figure, axis = plt.subplots(figsize=(10, 14), constrained_layout=True)
    _map_background(axis, occupancy)
    representatives = {}
    for edge in graph.edges:
        if edge.from_node > edge.to_node:
            continue
        color = colors[edge.static_traversability]
        axis.plot(edge.polyline_xy[:, 0], edge.polyline_xy[:, 1], color=color, linewidth=1.0, alpha=0.9)
        current = representatives.get(edge.static_traversability)
        if current is None or edge.length_m > current.length_m:
            representatives[edge.static_traversability] = edge
    polygon = np.asarray(defaults["footprint"]["polygon_m"], dtype=float)
    for state, edge in representatives.items():
        index = max(0, len(edge.polyline_xy) // 2 - 1)
        center = edge.polyline_xy[index]
        following = edge.polyline_xy[min(index + 1, len(edge.polyline_xy) - 1)]
        yaw = math.atan2(*(following - center)[::-1])
        _draw_footprint(axis, center, yaw, polygon, colors[state])
    axis.set_title("Static footprint feasibility (sample footprints outlined)")
    axis.legend(
        handles=[Line2D([0], [0], color=color, label=state.name) for state, color in colors.items()],
        loc="upper right",
    )
    phase3 = output_dir / "phase3_footprint_feasibility.png"
    figure.savefig(phase3, dpi=180)
    plt.close(figure)

    geojson = json.loads(geojson_path.read_text(encoding="utf-8"))
    route_start, route_goal = _select_route_endpoints(geojson)
    base_ids, features, base_cost = _route_from_geojson(
        geojson, route_start, route_goal, set()
    )
    blocked_support_edge = None
    reroute_ids = None
    reroute_cost = math.inf
    for candidate in base_ids:
        try:
            alternate, _, alternate_cost = _route_from_geojson(
                geojson, route_start, route_goal, {candidate}
            )
        except RuntimeError:
            continue
        if candidate not in alternate:
            blocked_support_edge = candidate
            reroute_ids = alternate
            reroute_cost = alternate_cost
            break
    if blocked_support_edge is None or reroute_ids is None:
        raise RuntimeError("selected route has no alternate-capable support edge")
    base_xy = _route_coordinates(base_ids, features)
    reroute_xy = _route_coordinates(reroute_ids, features)
    figure, axis = plt.subplots(figsize=(10, 14), constrained_layout=True)
    _map_background(axis, occupancy)
    _plot_graph(axis, graph, alpha=0.18, linewidth=0.4)
    axis.plot(base_xy[:, 0], base_xy[:, 1], color="#2962ff", linewidth=2.2, label="base Route Server path")
    axis.plot(
        reroute_xy[:, 0],
        reroute_xy[:, 1],
        color="#ff6d00",
        linewidth=1.8,
        linestyle="--",
        label=f"edge {blocked_support_edge} blocked reroute",
    )
    axis.scatter(base_xy[0, 0], base_xy[0, 1], marker="o", s=65, c="#00c853", edgecolors="black", label="start")
    axis.scatter(base_xy[-1, 0], base_xy[-1, 1], marker="*", s=110, c="#d50000", edgecolors="black", label="goal")
    axis.legend(loc="upper right", fontsize=8)
    axis.set_title("Official Route Server equivalent macro-route and dynamic reroute")
    phase5 = output_dir / "phase5_selected_route.png"
    figure.savefig(phase5, dpi=180)
    plt.close(figure)

    mapping = json.loads(mapping_path.read_text(encoding="utf-8"))
    support_to_canonical = {
        int(key): int(value)
        for key, value in mapping["support_to_canonical_edge"].items()
    }
    canonical_ids = []
    for edge_id in base_ids:
        canonical = support_to_canonical[edge_id]
        if not canonical_ids or canonical_ids[-1] != canonical:
            canonical_ids.append(canonical)
    tracker = RouteTracker(graph, canonical_ids, defaults["route_tracking"])
    target_index = max(1, len(canonical_ids) // 3)
    edge_map = graph.edge_by_id()
    for edge_id in canonical_ids[:target_index]:
        tracker.update(tuple(edge_map[edge_id].polyline_xy[-1]))
    target = edge_map[canonical_ids[target_index]]
    segment = target.polyline_xy[min(1, len(target.polyline_xy) - 1)] - target.polyline_xy[0]
    unit = segment / max(np.linalg.norm(segment), np.finfo(float).eps)
    robot = target.polyline_xy[0] + 0.2 * unit + 0.12 * np.asarray([-unit[1], unit[0]])
    progress = tracker.update(tuple(robot))
    canonical_xy = np.vstack(
        [edge_map[edge_id].polyline_xy[:-1] for edge_id in canonical_ids]
        + [edge_map[canonical_ids[-1]].polyline_xy[-1:]]
    )
    figure, axes = plt.subplots(1, 2, figsize=(16, 9), constrained_layout=True)
    for axis in axes:
        _map_background(axis, occupancy)
        axis.plot(canonical_xy[:, 0], canonical_xy[:, 1], color="#2962ff", linewidth=2.0, label="canonical route")
        axis.scatter(robot[0], robot[1], c="black", s=55, label="robot")
        axis.scatter(progress.projected_xy[0], progress.projected_xy[1], c="#e040fb", s=60, label="projection")
        axis.scatter(progress.lookahead_xy[0], progress.lookahead_xy[1], c="#00c853", marker="*", s=110, label="lookahead")
        axis.plot(
            [robot[0], progress.projected_xy[0]],
            [robot[1], progress.projected_xy[1]],
            color="#e040fb",
            linestyle=":",
        )
    axes[0].set_title("Route projection / lookahead in full map context")
    margin = 1.5
    axes[1].set_xlim(robot[0] - margin, robot[0] + margin)
    axes[1].set_ylim(robot[1] - margin, robot[1] + margin)
    axes[1].set_title("Local detail: projection stays on current route and lookahead is ahead")
    axes[1].legend(loc="upper right", fontsize=8)
    phase6 = output_dir / "phase6_route_projection_lookahead.png"
    figure.savefig(phase6, dpi=180)
    plt.close(figure)

    graph_containment_failures = 0
    for edge in graph.edges:
        for x, y in edge.polyline_xy:
            row, column = occupancy.world_to_pixel(x, y)
            if not (
                0 <= row < occupancy.free.shape[0]
                and 0 <= column < occupancy.free.shape[1]
                and occupancy.free[row, column]
            ):
                graph_containment_failures += 1
    summary = {
        "classification": "engineering_visual_evidence",
        "images": [path.name for path in (phase1, phase2, phase3, phase5, phase6)],
        "skeleton_pixels": int(np.count_nonzero(skeleton)),
        "skeleton_obstacle_overlap_pixels": int(np.count_nonzero(skeleton & ~occupancy.free)),
        "skeleton_min_clearance_m": float(clearance[skeleton].min()),
        "raw_ridge_pixels": int(np.count_nonzero(raw)),
        "graph_nodes": len(graph.nodes),
        "graph_physical_edges": len(graph.edges) // 2,
        "graph_components": _graph_components(graph),
        "choice_space_before_pruning": before_choice,
        "choice_space_after_pruning": after_choice,
        "pruning_removed_nodes": before_choice["node_count"] - after_choice["node_count"],
        "pruning_removed_physical_edges": before_choice["physical_edge_count"]
        - after_choice["physical_edge_count"],
        "pruning_removed_cycles": before_choice["cycle_count"] - after_choice["cycle_count"],
        "isolated_nodes": sum(node.degree == 0 for node in graph.nodes),
        "graph_polyline_free_containment_failures": graph_containment_failures,
        "node_types": dict(Counter(node.node_type.name for node in graph.nodes)),
        "traversability": dict(Counter(edge.static_traversability.name for edge in graph.edges)),
        "base_route_support_edges": len(base_ids),
        "route_start_support_node": route_start,
        "route_goal_support_node": route_goal,
        "base_route_length_m": base_cost,
        "reroute_support_edges": len(reroute_ids),
        "reroute_length_m": reroute_cost,
        "blocked_support_edge": blocked_support_edge,
        "blocked_edge_used_in_reroute": blocked_support_edge in reroute_ids,
        "projection_lateral_error_m": progress.lateral_error_m,
        "lookahead_distance_ahead_m": math.dist(progress.projected_xy, progress.lookahead_xy),
    }
    (output_dir / "visual_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return summary


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", required=True, dest="map_path")
    parser.add_argument("--defaults", required=True, dest="defaults_path")
    parser.add_argument("--geojson", required=True)
    parser.add_argument("--mapping", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args(argv)
    summary = export_visuals(
        Path(args.map_path),
        Path(args.defaults_path),
        Path(args.geojson),
        Path(args.mapping),
        Path(args.output_dir),
    )
    print(json.dumps(summary, sort_keys=True))


if __name__ == "__main__":
    main()
