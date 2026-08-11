from pathlib import Path

import numpy as np

from robot_route_planner.feasibility import classify_edge
from robot_route_planner.diagnostics import count_simple_routes, graph_diagnostics
from robot_route_planner.gvg import build_gvg
from robot_route_planner.map_io import OccupancyMap, load_occupancy_map
from robot_route_planner.models import Edge, Graph, Node, NodeType, Traversability


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
    }


def _footprint_settings():
    return {"padded_inscribed_radius_m": 0.215}


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
