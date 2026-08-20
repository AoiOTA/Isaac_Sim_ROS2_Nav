from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from robot_route_planner.cognitive_graph_adapter import (
    CognitiveGraphIdentity,
    build_hybrid_graph,
    validate_cognitive_graph_candidate,
)
from robot_route_planner.map_io import OccupancyMap
from robot_route_planner.models import Edge, Graph, Node, NodeType, Traversability
from robot_route_planner.ros_node import RouteCoordinator


FOOTPRINT = {
    'polygon_m': [[-0.1, -0.1], [-0.1, 0.1], [0.1, 0.1], [0.1, -0.1]],
    'padding_m': 0.0,
    'padded_inscribed_radius_m': 0.1,
    'sweep_sample_spacing_m': 0.05,
}


def _point(x, y):
    return SimpleNamespace(x=float(x), y=float(y), z=0.0)


def _map(*, wall=False):
    free = np.ones((80, 80), dtype=bool)
    if wall:
        free[:, 40] = False
    return OccupancyMap(free, 0.05, (-2.0, -2.0), 'map', Path('/tmp/map.yaml'))


def _identity():
    return CognitiveGraphIdentity(
        3, 'session', 'map', 'tile', 2, 'physical', 4, 'model')


def _candidate(*, stamp_ns=10_000_000_000, sequence=7, transform=None):
    edge = SimpleNamespace(
        DIRECTION_DIRECTED=0,
        DIRECTION_BIDIRECTIONAL=1,
        edge_id='e0',
        source_node_id='a',
        target_node_id='b',
        source_state_id=1,
        target_state_id=2,
        polyline_canvas=[_point(-0.8, 0.0), _point(0.8, 0.0)],
        transition_probability=1.0,
        evidence=3.0,
        directionality=1,
        changed=False,
        portal=False,
        success_count=3,
        failure_count=0,
    )
    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id='module2_canvas',
            stamp=SimpleNamespace(
                sec=stamp_ns // 1_000_000_000,
                nanosec=stamp_ns % 1_000_000_000,
            ),
        ),
        ttl=SimpleNamespace(sec=0, nanosec=500_000_000),
        schema_version='bio_nav_cognitive_place_graph_v1',
        source_sequence=sequence,
        recurrent_session_id='session',
        graph_id='cpg-' + 'a' * 24,
        topology_revision=5,
        value_sequence=1,
        map_version='map',
        reset_epoch=3,
        cognitive_tile_id='tile',
        tile_revision=2,
        source_physical_graph_id='physical',
        source_physical_graph_revision=4,
        t_map_canvas=(np.eye(3) if transform is None else transform).reshape(-1),
        model_id='model',
        module2_healthy=True,
        trusted_write=True,
        rejection_mask=0,
        nodes=[
            SimpleNamespace(node_id='a', canvas_position=_point(-0.8, 0.0)),
            SimpleNamespace(node_id='b', canvas_position=_point(0.8, 0.0)),
        ],
        edges=[edge],
    )


def _physical_graph():
    nodes = [
        Node(1, (-1.2, 0.5), 1, NodeType.ENDPOINT, 1.0),
        Node(2, (1.2, 0.5), 1, NodeType.ENDPOINT, 1.0),
    ]
    points = np.asarray([nodes[0].position_xy, nodes[1].position_xy])
    edge = Edge(
        1, 1, 2, points, 2.4, 1.0, 1.0, 1.0, 2.0, 0.0, False,
        Traversability.FEASIBLE,
    )
    reverse = Edge(
        2, 2, 1, points[::-1].copy(), 2.4, 1.0, 1.0, 1.0, 2.0, 0.0,
        False, Traversability.FEASIBLE,
    )
    return Graph('physical', 4, 'map', 0.05, nodes, [edge, reverse])


def test_candidate_inverse_transform_and_hybrid_keep_only_feasible_edges():
    transform = np.asarray([[1.0, 0.0, 0.2], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    validated = validate_cognitive_graph_candidate(
        _candidate(transform=transform),
        now_ns=10_100_000_000,
        expected=_identity(),
        last_source_sequence=6,
        occupancy=_map(),
        footprint=FOOTPRINT,
    )
    assert validated.graph.nodes[0].position_xy == pytest.approx((-1.0, 0.0))
    assert all(
        edge.static_traversability == Traversability.FEASIBLE
        for edge in validated.graph.edges
    )

    hybrid = build_hybrid_graph(
        _physical_graph(), validated, occupancy=_map(), footprint=FOOTPRINT)
    assert hybrid.graph_id.startswith('physical:hybrid:')
    assert any(edge.metadata.get('source') == 'module3_connector' for edge in hybrid.edges)
    assert all(edge.static_traversability == Traversability.FEASIBLE for edge in hybrid.edges)


def test_stale_malformed_and_wall_crossing_candidates_are_rejected():
    with pytest.raises(ValueError, match='stale'):
        validate_cognitive_graph_candidate(
            _candidate(stamp_ns=9_000_000_000),
            now_ns=10_100_000_000,
            expected=_identity(),
            last_source_sequence=6,
            occupancy=_map(),
            footprint=FOOTPRINT,
        )
    malformed = _candidate()
    malformed.nodes[1].node_id = 'a'
    with pytest.raises(ValueError, match='duplicate'):
        validate_cognitive_graph_candidate(
            malformed,
            now_ns=10_100_000_000,
            expected=_identity(),
            last_source_sequence=6,
            occupancy=_map(),
            footprint=FOOTPRINT,
        )
    with pytest.raises(ValueError, match='FEASIBLE'):
        validate_cognitive_graph_candidate(
            _candidate(),
            now_ns=10_100_000_000,
            expected=_identity(),
            last_source_sequence=6,
            occupancy=_map(wall=True),
            footprint=FOOTPRINT,
        )


def test_primary_fallback_is_single_and_whole_graph():
    coordinator = RouteCoordinator.__new__(RouteCoordinator)
    coordinator.primary_fallback_used = False
    coordinator.gvg_graph = _physical_graph()
    coordinator.graph = Graph('cognitive', 5, 'map', 0.05, [], [])
    coordinator.StructuralGraphStatus = SimpleNamespace(LAST_KNOWN_GOOD=2)
    switches = []
    coordinator._request_graph_switch = lambda graph, reason, fallback: switches.append(
        (graph.graph_id, reason, fallback)
    )
    coordinator._publish_structural_status = lambda *_args: None

    coordinator._fallback_to_gvg_once('route failed')
    coordinator._fallback_to_gvg_once('route failed again')

    assert switches == [('physical', 'route failed', True)]
