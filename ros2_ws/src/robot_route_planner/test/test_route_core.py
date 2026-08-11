import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from robot_route_planner.models import Edge, Graph, Node, NodeType, Traversability
from robot_route_planner.map_io import OccupancyMap
from robot_route_planner.ros_node import (
    CostmapSnapshot,
    RouteCoordinator,
    footprint_is_free,
    navigation_result_succeeded,
    populate_fresh_goal,
    select_live_feasible_lookahead,
    select_map_pose,
    select_support_attachment,
)
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


def test_support_export_shares_nodes_for_reverse_physical_edges() -> None:
    graph = _graph()
    graph.edges.append(
        _edge(5, 2, 1, [(1.0, 0.0), (0.5, 0.1), (0.0, 0.0)])
    )
    export = export_route_support_graph(graph, support_spacing_m=0.25)
    segments = {
        canonical: [
            (feature["properties"]["startid"], feature["properties"]["endid"])
            for feature in export.geojson["features"]
            if feature["geometry"]["type"] == "MultiLineString"
            and feature["properties"]["metadata"]["canonical_edge_id"]
            == canonical
        ]
        for canonical in (1, 5)
    }

    assert segments[1] == [
        (end, start) for start, end in reversed(segments[5])
    ]


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


def test_tracker_does_not_bypass_short_route_inside_lookahead_horizon() -> None:
    tracker = RouteTracker(
        _graph(),
        [1, 2],
        {
            "projection_edge_window": 3,
            "max_backtrack_m": 0.2,
            "advance_hysteresis_m": 0.05,
            "lookahead_m": 3.0,
            "final_goal_switch_distance_m": 0.3,
        },
    )
    progress = tracker.update((0.1, 0.0))
    assert progress.remaining_m > 0.3
    assert progress.remaining_m < 3.0
    assert not progress.use_final_goal


def test_tracker_uses_exact_trimmed_route_server_support_geometry() -> None:
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
        route_segments_xy=[
            np.asarray([[0.5, 0.1], [1.0, 0.0]]),
            np.asarray([[1.0, 0.0], [1.0, 0.4]]),
        ],
    )

    assert tracker.offsets[-1] < 1.0
    progress = tracker.update((0.55, 0.09))
    assert progress.lookahead_xy[1] > 0.0


def test_support_attachment_rejects_nearest_node_across_wall() -> None:
    free = np.ones((100, 100), dtype=bool)
    free[:, 50] = False
    occupancy = OccupancyMap(
        free, 0.1, (0.0, 0.0), "test", Path("test.yaml")
    )
    support = {1: (5.2, 5.0), 2: (4.0, 3.5)}
    settings = {
        "polygon_m": [[0.08, 0.06], [0.08, -0.06], [-0.08, -0.06], [-0.08, 0.06]],
        "padding_m": 0.0,
        "padded_inscribed_radius_m": 0.06,
        "sweep_sample_spacing_m": 0.05,
    }

    selected = select_support_attachment(
        occupancy, support, (4.0, 5.0), settings, departing=False
    )

    assert selected == 2


def test_live_costmap_advances_blocked_lookahead_on_same_route() -> None:
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
    current = (0.1, 0.0)
    progress = tracker.update(current)
    values = np.zeros((80, 80), dtype=np.uint8)
    snapshot = CostmapSnapshot(values, 0.05, (-1.0, -1.0), "map")
    footprint = np.asarray(
        [[0.08, 0.06], [0.08, -0.06], [-0.08, -0.06], [-0.08, 0.06]]
    )
    nominal = progress.lookahead_xy
    column = int((nominal[0] - snapshot.origin_xy[0]) / snapshot.resolution_m)
    row = int((nominal[1] - snapshot.origin_xy[1]) / snapshot.resolution_m)
    values[row - 2:row + 3, column - 2:column + 3] = 253

    assert not footprint_is_free(snapshot, nominal, 0.0, footprint)
    selected = select_live_feasible_lookahead(
        tracker,
        current,
        progress,
        snapshot,
        footprint,
        nominal_distance_m=0.6,
        sample_spacing_m=0.05,
    )

    assert selected.lookahead_xy != nominal
    assert selected.arc_length_m == progress.arc_length_m
    assert footprint_is_free(
        snapshot,
        selected.lookahead_xy,
        math.atan2(
            selected.lookahead_xy[1] - current[1],
            selected.lookahead_xy[0] - current[0],
        ),
        footprint,
    )


def test_map_frame_odometry_wins_over_transient_tf_and_odom_frame_uses_tf() -> None:
    assert select_map_pose("map", "map", (1.0, 2.0), (9.0, 8.0)) == (1.0, 2.0)
    assert select_map_pose("map", "odom", (1.0, 2.0), (9.0, 8.0)) == (9.0, 8.0)
    assert select_map_pose("map", None, None, None) is None


def test_final_goal_copy_refreshes_header_without_changing_pose() -> None:
    old_header = object()
    new_header = object()
    pose = object()
    source = SimpleNamespace(header=old_header, pose=pose)
    target = SimpleNamespace(header=None, pose=None)
    populate_fresh_goal(target, source, new_header)
    assert target.header is new_header
    assert target.pose is pose


def test_navigation_result_retires_old_leg_before_publishing_completion() -> None:
    events = []

    class Publisher:
        def publish(self, message) -> None:
            events.append((message.data, coordinator.pending_goal))

    coordinator = RouteCoordinator.__new__(RouteCoordinator)
    coordinator.route_active = True
    coordinator.pending_goal = object()
    coordinator.tracker = object()
    coordinator.navigation_goal_pending = True
    coordinator.navigation_goal_handle = object()
    coordinator.navigation_failed = True
    coordinator.pending_structural_map = None
    coordinator.goal_complete_pub = Publisher()
    coordinator.node = SimpleNamespace(
        get_logger=lambda: SimpleNamespace(info=lambda _message: None)
    )
    future = SimpleNamespace(result=lambda: SimpleNamespace(
        status=4,
        result=SimpleNamespace(error_code=205),
    ))

    coordinator._on_navigation_result(future)

    assert events == [(True, None)]
    assert not coordinator.route_active
    assert coordinator.tracker is None
    assert coordinator.navigation_goal_handle is None


def test_navigation_action_status_is_authoritative_over_error_detail() -> None:
    assert navigation_result_succeeded(SimpleNamespace(
        status=4,
        result=SimpleNamespace(error_code=205),
    ))
    assert not navigation_result_succeeded(SimpleNamespace(
        status=6,
        result=SimpleNamespace(error_code=0),
    ))


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
