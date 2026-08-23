from pathlib import Path
import math

import cv2
import numpy as np

from robot_route_planner.feasibility import (
    _polygon_is_free,
    apply_footprint_feasibility,
    classify_edge,
)
from robot_route_planner.diagnostics import count_simple_routes, graph_diagnostics
from robot_route_planner.gvg import build_gvg
from robot_route_planner.map_io import OccupancyMap, load_occupancy_map
from robot_route_planner.models import Edge, Graph, Node, NodeType, Traversability
from robot_route_planner.ros_node import select_support_attachment
from robot_route_planner.route_support import export_route_support_graph


def _settings():
    return {
        "algorithm_version": "gvg_v1",
        "obstacle_site_min_separation_cells": 2,
        "obstacle_source_min_angle_deg": 90.0,
        "ridge_distance_difference_cells": 1.0,
        "minimum_free_component_area_m2": 0.25,
        "rdp_epsilon_m": 0.05,
        "spur_max_length_m": 0.30,
        "spur_clearance_ratio_of_inscribed": 1.25,
        "junction_merge_radius_cells": 1,
        "loop_anchor_min_separation_m": 0.50,
        "route_support_spacing_m": 0.20,
        "unknown_is_occupied": True,
    }


def _footprint_settings():
    return {
        "polygon_m": [
            [0.255, 0.21],
            [0.255, -0.21],
            [-0.23, -0.21],
            [-0.23, 0.21],
        ],
        "padding_m": 0.005,
        "padded_inscribed_radius_m": 0.215,
        "sweep_sample_spacing_m": 0.025,
    }


def _route_cost_settings():
    return {"preferred_clearance_m": 0.385}


def test_real_warehouse_map_builds_deterministic_directed_graph() -> None:
    repo = Path(__file__).resolve().parents[4]
    occupancy = load_occupancy_map(
        repo / "data/maps/occupancy/warehouse_new.yaml",
        unknown_is_occupied=True,
    )
    first = build_gvg(
        occupancy, _settings(), _footprint_settings(), _route_cost_settings()
    )
    second = build_gvg(
        occupancy, _settings(), _footprint_settings(), _route_cost_settings()
    )
    assert first.graph_id == "warehouse_new:gvg_v1"
    assert len(first.nodes) > 0
    assert len(first.edges) > 0
    assert [(node.id, node.position_xy) for node in first.nodes] == [
        (node.id, node.position_xy) for node in second.nodes
    ]
    assert [(edge.id, edge.from_node, edge.to_node) for edge in first.edges] == [
        (edge.id, edge.from_node, edge.to_node) for edge in second.edges
    ]
    reverse = {(edge.from_node, edge.to_node) for edge in first.edges}
    assert all((edge.to_node, edge.from_node) in reverse for edge in first.edges)


def test_v6_isaacgen_map_builds_connected_graph_with_clear_edges() -> None:
    repo = Path(__file__).resolve().parents[4]
    occupancy = load_occupancy_map(
        repo / "data/maps/occupancy/v6_kujiale_isaacgen_v1.yaml",
        unknown_is_occupied=True,
    )
    first = build_gvg(
        occupancy, _settings(), _footprint_settings(), _route_cost_settings()
    )
    second = build_gvg(
        occupancy, _settings(), _footprint_settings(), _route_cost_settings()
    )
    assert first.graph_id == "v6_kujiale_isaacgen_v1:gvg_v1"
    assert [(node.id, node.position_xy) for node in first.nodes] == [
        (node.id, node.position_xy) for node in second.nodes
    ]
    assert [(edge.id, edge.from_node, edge.to_node) for edge in first.edges] == [
        (edge.id, edge.from_node, edge.to_node) for edge in second.edges
    ]
    # The regenerated map keeps the whole graph in one component and every
    # edge at or above the padded inscribed clearance (0.215 + 0.005 m).
    assert graph_diagnostics(first)["component_count"] == 1
    assert min(edge.min_clearance_m for edge in first.edges) >= 0.22


def test_v6_clearance_map_builds_connected_five_leg_support_graph() -> None:
    repo = Path(__file__).resolve().parents[4]
    occupancy = load_occupancy_map(
        repo / "data/maps/occupancy/v6_kujiale_clearance_r1.yaml",
        unknown_is_occupied=True,
    )
    graph = build_gvg(
        occupancy, _settings(), _footprint_settings(), _route_cost_settings()
    )
    apply_footprint_feasibility(graph, occupancy, _footprint_settings())
    assert graph.graph_id == "v6_kujiale_clearance_r1:gvg_v1"
    assert graph_diagnostics(graph)["component_count"] == 1
    assert min(edge.min_clearance_m for edge in graph.edges) >= 0.22

    support = export_route_support_graph(
        graph, support_spacing_m=_settings()["route_support_spacing_m"]
    )
    positions = {
        int(feature["properties"]["id"]): tuple(
            feature["geometry"]["coordinates"]
        )
        for feature in support.geojson["features"]
        if feature["geometry"]["type"] == "Point"
    }
    adjacency: dict[int, set[int]] = {}
    for feature in support.geojson["features"]:
        if feature["geometry"]["type"] != "MultiLineString":
            continue
        properties = feature["properties"]
        adjacency.setdefault(int(properties["startid"]), set()).add(
            int(properties["endid"])
        )

    goals = [
        (0.45, -5.35),
        (0.80, 4.80),
        (-2.20, 3.25),
        (-3.00, -0.45),
        (-2.20, -2.95),
        (0.45, -5.35),
    ]
    attachments = [
        select_support_attachment(
            occupancy,
            positions,
            point,
            _footprint_settings(),
            departing=index < len(goals) - 1,
        )
        for index, point in enumerate(goals)
    ]
    for start, goal in zip(attachments, attachments[1:]):
        reachable = {start}
        frontier = [start]
        while frontier:
            node = frontier.pop()
            for following in adjacency.get(node, set()) - reachable:
                reachable.add(following)
                frontier.append(following)
        assert goal in reachable


def test_v6_clearance_r2_five_leg_routes_pass_directional_footprint_sweeps() -> None:
    repo = Path(__file__).resolve().parents[4]
    occupancy = load_occupancy_map(
        repo / "data/maps/occupancy/v6_kujiale_clearance_r2.yaml",
        unknown_is_occupied=True,
    )
    graph = build_gvg(
        occupancy, _settings(), _footprint_settings(), _route_cost_settings()
    )
    apply_footprint_feasibility(graph, occupancy, _footprint_settings())
    assert graph.graph_id == "v6_kujiale_clearance_r2:gvg_v1"
    assert graph_diagnostics(graph)["component_count"] == 1
    assert min(edge.min_clearance_m for edge in graph.edges) >= 0.22

    support = export_route_support_graph(
        graph, support_spacing_m=_settings()["route_support_spacing_m"]
    )
    positions = {
        int(feature["properties"]["id"]): tuple(
            feature["geometry"]["coordinates"]
        )
        for feature in support.geojson["features"]
        if feature["geometry"]["type"] == "Point"
    }
    segments = {
        int(feature["properties"]["id"]): feature
        for feature in support.geojson["features"]
        if feature["geometry"]["type"] == "MultiLineString"
    }
    adjacency: dict[int, list[tuple[int, int]]] = {}
    for edge_id, feature in segments.items():
        properties = feature["properties"]
        adjacency.setdefault(int(properties["startid"]), []).append(
            (int(properties["endid"]), edge_id)
        )

    goals = [
        (0.45, -5.35),
        (0.80, 4.80),
        (-2.20, 3.25),
        (-3.00, -0.45),
        (-2.20, -2.95),
        (0.45, -5.35),
    ]
    attachments = [
        select_support_attachment(
            occupancy,
            positions,
            point,
            _footprint_settings(),
            departing=index < len(goals) - 1,
        )
        for index, point in enumerate(goals)
    ]
    footprint = _footprint_settings()
    for start_xy, goal_xy, start, goal in zip(
        goals, goals[1:], attachments, attachments[1:]
    ):
        predecessor: dict[int, tuple[int, int]] = {}
        frontier = [start]
        while frontier and goal not in predecessor:
            node = frontier.pop(0)
            for following, edge_id in sorted(adjacency.get(node, [])):
                if following == start or following in predecessor:
                    continue
                predecessor[following] = (node, edge_id)
                frontier.append(following)
        assert goal in predecessor
        route_edges = []
        node = goal
        while node != start:
            node, edge_id = predecessor[node]
            route_edges.append(edge_id)
        route_edges.reverse()
        route_points = [start_xy]
        for edge_id in route_edges:
            start_point, end_point = segments[edge_id]["geometry"]["coordinates"][0]
            if math.dist(route_points[-1], start_point) > 1e-9:
                route_points.append(start_point)
            route_points.append(end_point)
        route_points.append(goal_xy)
        assert classify_edge(
            occupancy,
            np.asarray(route_points, dtype=np.float64),
            footprint_polygon_m=np.asarray(footprint["polygon_m"], dtype=np.float64),
            footprint_padding_m=footprint["padding_m"],
            padded_inscribed_radius_m=footprint["padded_inscribed_radius_m"],
            sweep_sample_spacing_m=footprint["sweep_sample_spacing_m"],
        ) == Traversability.FEASIBLE


def test_feasible_unknown_and_disconnected_are_distinct() -> None:
    free = np.ones((80, 80), dtype=bool)
    free[[0, -1], :] = False
    free[:, [0, -1]] = False
    occupancy = OccupancyMap(free, 0.05, (0.0, 0.0), "synthetic", Path("map.yaml"))
    footprint = np.asarray([[0.10, 0.08], [0.10, -0.08], [-0.10, -0.08], [-0.10, 0.08]])
    common = dict(
        footprint_polygon_m=footprint,
        footprint_padding_m=0.0,
        padded_inscribed_radius_m=0.08,
        sweep_sample_spacing_m=0.025,
    )
    feasible = classify_edge(
        occupancy, np.asarray([[0.5, 2.0], [3.0, 2.0]]), **common
    )
    assert feasible == Traversability.FEASIBLE

    local_obstacle = free.copy()
    local_obstacle[38:43, 38:43] = False
    around = OccupancyMap(local_obstacle, 0.05, (0.0, 0.0), "around", Path("map.yaml"))
    unknown = classify_edge(
        around, np.asarray([[0.5, 2.0], [3.0, 2.0]]), **common
    )
    assert unknown == Traversability.UNKNOWN

    wall = free.copy()
    wall[:, 39:42] = False
    split = OccupancyMap(wall, 0.05, (0.0, 0.0), "split", Path("map.yaml"))
    infeasible = classify_edge(
        split, np.asarray([[0.5, 2.0], [3.0, 2.0]]), **common
    )
    assert infeasible == Traversability.INFEASIBLE


def test_local_footprint_raster_matches_full_map_reference() -> None:
    free = np.ones((100, 120), dtype=bool)
    free[38:54, 55:70] = False
    occupancy = OccupancyMap(
        free, 0.05, (-3.0, -2.5), "local-mask", Path("map.yaml")
    )
    footprint = np.asarray(
        [[0.28, 0.20], [0.28, -0.20], [-0.28, -0.20], [-0.28, 0.20]],
        dtype=np.float64,
    )

    def full_map_reference(x: float, y: float, yaw: float) -> bool:
        rotation = np.asarray(
            [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
        )
        world = footprint @ rotation.T + np.asarray([x, y])
        pixels = np.asarray(
            [occupancy.world_to_pixel(point[0], point[1]) for point in world],
            dtype=np.int32,
        )
        polygon = np.column_stack((pixels[:, 1], pixels[:, 0])).astype(np.int32)
        mask = np.zeros_like(free, dtype=np.uint8)
        cv2.fillConvexPoly(mask, polygon, 1)
        return bool(np.all(free[mask.astype(bool)]))

    for x, y in ((-1.5, -1.0), (0.0, 0.0), (0.25, 0.25), (1.5, 1.0)):
        for yaw in (0.0, 0.4, 0.5 * math.pi, -0.8):
            assert _polygon_is_free(occupancy, x, y, yaw, footprint) is (
                full_map_reference(x, y, yaw)
            )


def test_real_map_pruning_preserves_cycles_and_probe_alternatives() -> None:
    repo = Path(__file__).resolve().parents[4]
    occupancy = load_occupancy_map(
        repo / "data/maps/occupancy/warehouse_new.yaml",
        unknown_is_occupied=True,
    )
    unpruned_settings = _settings()
    unpruned_settings["spur_max_length_m"] = 0.0
    unpruned = build_gvg(
        occupancy,
        unpruned_settings,
        _footprint_settings(),
        _route_cost_settings(),
    )
    pruned = build_gvg(
        occupancy,
        _settings(),
        _footprint_settings(),
        _route_cost_settings(),
    )
    after = graph_diagnostics(pruned)
    before = graph_diagnostics(
        unpruned,
        start_node=after["start_node"],
        goal_node=after["goal_node"],
    )
    assert before["cycle_count"] == after["cycle_count"] == 1
    assert before["start_goal_route_count"] == \
        after["start_goal_route_count"] == 2
    assert after["start_goal_alternative_route_count"] == 1
    assert after["physical_edge_count"] < before["physical_edge_count"]


def test_choice_space_diagnostic_counts_a_real_loop_not_density() -> None:
    nodes = [
        Node(index, (float(index), 0.0), 2, NodeType.JUNCTION, 1.0)
        for index in range(1, 5)
    ]
    directed = []
    edge_id = 1
    for source, target in ((1, 2), (2, 4), (1, 3), (3, 4)):
        for left, right in ((source, target), (target, source)):
            directed.append(
                Edge(
                    edge_id,
                    left,
                    right,
                    np.asarray([[float(left), 0.0], [float(right), 0.0]]),
                    1.0,
                    1.0,
                    1.0,
                    1.0,
                    2.0,
                    0.0,
                    False,
                )
            )
            edge_id += 1
    graph = Graph("loop", 1, "synthetic", 0.05, nodes, directed)
    routes, capped = count_simple_routes(graph, 1, 4)
    diagnostic = graph_diagnostics(graph, start_node=1, goal_node=4)
    assert routes == 2
    assert capped is False
    assert diagnostic["cycle_count"] == 1
    assert diagnostic["start_goal_alternative_route_count"] == 1
