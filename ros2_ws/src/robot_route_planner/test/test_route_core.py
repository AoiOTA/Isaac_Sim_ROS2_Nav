import numpy as np

from robot_route_planner.models import Edge, Graph, Node, NodeType, Traversability
from robot_route_planner.route_cost import shortest_route
from robot_route_planner.route_support import export_route_support_graph
from robot_route_planner.runtime_edges import RuntimeEdgeManager, RuntimeState
from robot_route_planner.stable_ids import stabilize_graph_ids
from robot_route_planner.structural_updates import StructuralChangeMonitor
from robot_route_planner.tracking import RouteTracker


def _edge(edge_id, source, target, points, clearance=0.5):
    polyline = np.asarray(points, dtype=float)
    length = float(np.linalg.norm(np.diff(polyline, axis=0), axis=1).sum())
    return Edge(
        edge_id,
        source,
        target,
        polyline,
        length,
        clearance,
        clearance,
        clearance,
        2.0 * clearance,
        0.0,
        clearance < 0.385,
        Traversability.FEASIBLE,
    )


def _graph():
    nodes = [
        Node(1, (0.0, 0.0), 2, NodeType.JUNCTION, 0.5),
        Node(2, (1.0, 0.0), 2, NodeType.JUNCTION, 0.5),
        Node(3, (0.0, 1.0), 2, NodeType.JUNCTION, 0.5),
        Node(4, (1.0, 1.0), 2, NodeType.JUNCTION, 0.5),
    ]
    edges = [
        _edge(1, 1, 2, [(0.0, 0.0), (0.5, 0.1), (1.0, 0.0)]),
        _edge(2, 2, 4, [(1.0, 0.0), (1.0, 1.0)]),
        _edge(3, 1, 3, [(0.0, 0.0), (0.0, 1.0)]),
        _edge(4, 3, 4, [(0.0, 1.0), (1.0, 1.0)]),
    ]
    return Graph("test:gvg_v1", 1, "test", 0.05, nodes, edges)


def _cost_settings():
    return {
        "minimum_clearance_m": 0.215,
        "preferred_clearance_m": 0.385,
        "numeric_epsilon": 1.0e-9,
        "clearance_penalty_weight": 0.5,
        "max_prior_cost_ratio_of_edge_length": 0.25,
    }


def test_support_export_splits_curves_and_maps_dynamic_edges() -> None:
    export = export_route_support_graph(_graph(), support_spacing_m=0.25)
    assert len(export.canonical_to_support_edges[1]) > 1
    assert all(
        export.support_to_canonical_edge[support] == 1
        for support in export.canonical_to_support_edges[1]
    )
    edge_features = [
        feature for feature in export.geojson["features"]
        if feature["geometry"]["type"] == "MultiLineString"
    ]
    assert all(len(item["geometry"]["coordinates"][0]) == 2 for item in edge_features)


def test_prior_changes_route_and_blocked_edge_reroutes() -> None:
    graph = _graph()
    _, base_edges, _ = shortest_route(graph, 1, 4, _cost_settings())
    assert base_edges == [3, 4]
    _, prior_edges, _ = shortest_route(
        graph, 1, 4, _cost_settings(), priors={3: (0.25, 1.0)}
    )
    assert prior_edges == [1, 2]
    _, rerouted, _ = shortest_route(
        graph, 1, 4, _cost_settings(), runtime={1: (0.0, True)}
    )
    assert rerouted == [3, 4]


def test_tracker_crosses_edge_boundary_and_switches_to_final_goal() -> None:
    tracker = RouteTracker(
        _graph(),
        [1, 2],
        {
            "projection_edge_window": 3,
            "max_backtrack_m": 0.2,
            "advance_hysteresis_m": 0.05,
            "lookahead_m": 0.6,
            "final_goal_switch_distance_m": 0.3,
        },
    )
    first = tracker.update((0.2, 0.04))
    assert first.edge_id == 1
    assert first.lookahead_xy[0] > 0.6
    second = tracker.update((1.0, 0.4))
    assert second.edge_id == 2
    final = tracker.update((1.0, 0.9))
    assert final.use_final_goal


def test_runtime_state_and_persistent_structural_change() -> None:
    manager = RuntimeEdgeManager(
        {
            "block_after_consecutive_failures": 3,
            "block_after_occupied_s": 3.0,
            "reopen_after_clear_s": 5.0,
            "unknown_after_unobserved_s": 30.0,
        },
        {
            "suspect_edge_penalty_m": 2.0,
            "unknown_edge_penalty_m": 1.0,
            "blocked_edge_penalty_m": 1000000.0,
        },
    )
    manager.observe_failure(7, 0.0, occupied_ahead=True)
    manager.observe_failure(7, 2.0, occupied_ahead=True)
    blocked = manager.observe_failure(7, 3.0, occupied_ahead=True)
    assert blocked.state == RuntimeState.BLOCKED
    manager.observe_clear(7, 4.0)
    opened = manager.observe_clear(7, 9.0)
    assert opened.state == RuntimeState.OPEN
    manager.tick(40.0)
    assert manager.state(7).state == RuntimeState.UNKNOWN

    baseline = np.ones((20, 20), dtype=bool)
    changed = baseline.copy()
    changed[:, 10] = False
    monitor = StructuralChangeMonitor(
        baseline,
        0.1,
        {"changed_area_m2": 0.1, "stable_snapshot_count": 3, "stable_for_s": 2.0},
    )
    assert not monitor.observe(changed, 0.0)
    assert not monitor.observe(changed, 1.0)
    assert monitor.observe(changed, 2.0)


def test_stable_ids_reuse_matches_and_allocate_new_values() -> None:
    old = _graph()
    new = _graph()
    for node in new.nodes:
        node.position_xy = (node.position_xy[0] + 0.01, node.position_xy[1])
    new.nodes.append(Node(5, (2.0, 2.0), 1, NodeType.ENDPOINT, 0.5))
    stable = stabilize_graph_ids(
        new,
        old,
        {
            "stable_node_match_radius_m": 0.25,
            "stable_edge_hausdorff_m": 0.2,
            "stable_edge_length_ratio": 0.25,
        },
    )
    assert stable.revision == 2
    assert {1, 2, 3, 4}.issubset({node.id for node in stable.nodes})
    assert max(node.id for node in stable.nodes) > 4
    assert {1, 2, 3, 4}.issubset({edge.id for edge in stable.edges})
