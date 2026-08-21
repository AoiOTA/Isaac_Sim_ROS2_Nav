from pathlib import Path
import threading
from types import SimpleNamespace

import numpy as np
import pytest
import robot_route_planner.ros_node as ros_node_module

from robot_route_planner.cognitive_graph_adapter import (
    CognitiveGraphFeedback,
    CognitiveGraphIdentity,
    build_hybrid_graph,
    validate_cognitive_graph_candidate,
)
from robot_route_planner.map_io import OccupancyMap
from robot_route_planner.models import Edge, Graph, Node, NodeType, Traversability
from robot_route_planner.ros_node import (
    GraphSwitchGeneration,
    RouteCoordinator,
    StructuralRebuildGeneration,
)


FOOTPRINT = {
    'polygon_m': [[-0.1, -0.1], [-0.1, 0.1], [0.1, 0.1], [0.1, -0.1]],
    'padding_m': 0.0,
    'padded_inscribed_radius_m': 0.1,
    'sweep_sample_spacing_m': 0.05,
}


class _FeedbackMessage:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None, frame_id='')


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _feedback(graph_id='cognitive', revision=5):
    return CognitiveGraphFeedback(
        'session', 3, 7, 'candidate', 5, 1, graph_id, revision,
        (('e0', '1'), ('e0', '2')),
    )


def _feedback_coordinator(*, graph_id='cognitive', revision=5):
    coordinator = RouteCoordinator.__new__(RouteCoordinator)
    coordinator.frame_id = 'map'
    coordinator.graph = SimpleNamespace(graph_id=graph_id, revision=revision)
    coordinator._now = lambda: SimpleNamespace(to_msg=lambda: object())
    coordinator.CognitiveGraphValidationAck = _FeedbackMessage
    coordinator.CognitiveEdgeOutcome = _FeedbackMessage
    coordinator.cognitive_graph_validation_pub = _Publisher()
    coordinator.cognitive_edge_outcome_pub = _Publisher()
    coordinator.cognitive_feedback_sequences = {}
    coordinator.cognitive_validation_terminal = set()
    coordinator.cognitive_outcome_terminal = set()
    coordinator.cognitive_reroute_revision = 0
    return coordinator


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


def _revalidated_candidate(
    *, source_ns=6_000_000_000, validation_ns=10_000_000_000,
    source_age_s=4.0, candidate_ttl_s=0.5,
):
    message = _candidate(stamp_ns=validation_ns)
    message.source_stamp = SimpleNamespace(
        sec=source_ns // 1_000_000_000,
        nanosec=source_ns % 1_000_000_000,
    )
    message.validation_stamp = SimpleNamespace(
        sec=validation_ns // 1_000_000_000,
        nanosec=validation_ns % 1_000_000_000,
    )
    message.source_age = SimpleNamespace(
        sec=int(source_age_s),
        nanosec=int(round((source_age_s - int(source_age_s)) * 1.0e9)),
    )
    message.candidate_ttl = SimpleNamespace(
        sec=int(candidate_ttl_s),
        nanosec=int(round((candidate_ttl_s - int(candidate_ttl_s)) * 1.0e9)),
    )
    message.validation_mode = 'identity_revalidated_static_graph'
    message.observation_valid = True
    return message


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
    neighbours = {node.id: set() for node in hybrid.nodes}
    for edge in hybrid.edges:
        neighbours[edge.from_node].add(edge.to_node)
        neighbours[edge.to_node].add(edge.from_node)
    assert all(node.degree == len(neighbours[node.id]) for node in hybrid.nodes)


def test_static_graph_revalidation_preserves_old_source_age() -> None:
    validated = validate_cognitive_graph_candidate(
        _revalidated_candidate(),
        now_ns=10_100_000_000,
        expected=_identity(),
        last_source_sequence=6,
        occupancy=_map(),
        footprint=FOOTPRINT,
    )
    assert validated.source_sequence == 7


@pytest.mark.parametrize(
    ('changes', 'reason'),
    (
        ({'source_ns': 4_900_000_000, 'source_age_s': 5.1}, 'provenance'),
        ({'validation_ns': 9_500_000_000, 'source_age_s': 3.5}, 'stale'),
        ({'source_ns': 10_100_000_000, 'source_age_s': -0.1}, 'provenance'),
        ({'source_age_s': 3.0}, 'provenance'),
        ({'candidate_ttl_s': 0.6}, 'provenance'),
    ),
)
def test_static_graph_revalidation_rejects_bad_provenance(changes, reason) -> None:
    with pytest.raises(ValueError, match=reason):
        validate_cognitive_graph_candidate(
            _revalidated_candidate(**changes),
            now_ns=10_100_000_000,
            expected=_identity(),
            last_source_sequence=6,
            occupancy=_map(),
            footprint=FOOTPRINT,
        )


@pytest.mark.parametrize(
    ('field', 'value'),
    (
        ('observation_valid', False),
        ('recurrent_session_id', 'other'),
        ('map_version', 'other-map'),
        ('cognitive_tile_id', 'other-tile'),
        ('source_physical_graph_id', 'other-physical'),
        ('source_physical_graph_revision', 99),
    ),
)
def test_static_graph_revalidation_rejects_health_and_context(field, value) -> None:
    message = _revalidated_candidate()
    setattr(message, field, value)
    with pytest.raises(ValueError):
        validate_cognitive_graph_candidate(
            message,
            now_ns=10_100_000_000,
            expected=_identity(),
            last_source_sequence=6,
            occupancy=_map(),
            footprint=FOOTPRINT,
        )


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


def test_edge_endpoints_and_directed_connectivity_are_enforced():
    disconnected = _candidate()
    disconnected.edges[0].polyline_canvas[0] = _point(-0.4, 0.0)
    with pytest.raises(ValueError, match='endpoints'):
        validate_cognitive_graph_candidate(
            disconnected,
            now_ns=10_100_000_000,
            expected=_identity(),
            last_source_sequence=6,
            occupancy=_map(),
            footprint=FOOTPRINT,
        )

    one_way = _candidate()
    one_way.edges[0].directionality = one_way.edges[0].DIRECTION_DIRECTED
    with pytest.raises(ValueError, match='strongly connected'):
        validate_cognitive_graph_candidate(
            one_way,
            now_ns=10_100_000_000,
            expected=_identity(),
            last_source_sequence=6,
            occupancy=_map(),
            footprint=FOOTPRINT,
        )

    reverse = SimpleNamespace(**vars(one_way.edges[0]))
    reverse.edge_id = 'e1'
    reverse.source_node_id = 'b'
    reverse.target_node_id = 'a'
    reverse.source_state_id = 2
    reverse.target_state_id = 1
    reverse.polyline_canvas = list(reversed(one_way.edges[0].polyline_canvas))
    one_way.edges.append(reverse)
    validated = validate_cognitive_graph_candidate(
        one_way,
        now_ns=10_100_000_000,
        expected=_identity(),
        last_source_sequence=6,
        occupancy=_map(),
        footprint=FOOTPRINT,
    )
    assert {(edge.from_node, edge.to_node) for edge in validated.graph.edges} == {
        (1, 2), (2, 1)
    }


def test_first_invalid_candidate_does_not_bind_generation_identity():
    coordinator = RouteCoordinator.__new__(RouteCoordinator)
    coordinator.pending_goal = None
    coordinator.primary_fallback_used = False
    coordinator.cognitive_graph_switch_pending = False
    coordinator.cognitive_graph_mode = 'shadow'
    coordinator.cognitive_graph_last_sequence = 0
    coordinator.cognitive_graph_identity = CognitiveGraphIdentity(
        3, '', 'map', '', 0, 'physical', 4, '')
    coordinator.graph = _physical_graph()
    coordinator.map = _map()
    coordinator.defaults = {'footprint': FOOTPRINT}
    coordinator.StructuralGraphStatus = SimpleNamespace(
        LAST_KNOWN_GOOD=2, READY=1)
    now = SimpleNamespace(
        nanoseconds=10_100_000_000, to_msg=lambda: object())
    coordinator._now = lambda: now
    coordinator._publish_structural_status = lambda *_args: None
    coordinator.frame_id = 'map'
    coordinator.CognitiveGraphValidationAck = _FeedbackMessage
    coordinator.cognitive_graph_validation_pub = _Publisher()
    invalid = _candidate()
    invalid.source_physical_graph_revision = 99

    coordinator._on_cognitive_graph(invalid)

    assert coordinator.cognitive_graph_identity == CognitiveGraphIdentity(
        3, '', 'map', '', 0, 'physical', 4, '')
    assert coordinator.cognitive_graph_last_sequence == 0
    ack = coordinator.cognitive_graph_validation_pub.messages[0]
    assert ack.accepted is False
    assert ack.validated_graph_id == 'physical'
    assert ack.validated_graph_revision == 4
    assert ack.validated_edge_id == ''


def test_reset_clears_candidate_generation_state():
    coordinator = RouteCoordinator.__new__(RouteCoordinator)
    coordinator.cognitive_graph_identity = _identity()
    coordinator.gvg_graph = _physical_graph()
    coordinator.graph = Graph('cognitive', 5, 'map', 0.05, [], [])
    coordinator.graph_generation = 4
    coordinator.graph_switch_generation = 7
    coordinator.cognitive_graph_switch_pending = True
    coordinator.cognitive_graph_last_sequence = 9
    coordinator.cognitive_graph_feedback_active = _feedback()
    coordinator.cognitive_graph_feedback_pending = _feedback()
    coordinator.cognitive_feedback_sequences = {(7, 'outcome', 'e0'): 2}
    coordinator.cognitive_validation_terminal = {(7, 'e0')}
    coordinator.cognitive_outcome_terminal = {(7, '1')}
    coordinator.pending_reroute_outcome = (_feedback(), '1', 'e0')
    coordinator.cognitive_reroute_revision = 2
    coordinator.pending_deadline_ns = 1
    coordinator.pending_prior_request_id = 2
    coordinator.pending_prior_graph_id = 'cognitive'
    coordinator.pending_prior_graph_revision = 5
    coordinator.pending_prior_started_ns = 1
    coordinator.pending_prior_model_id = 'model'
    coordinator.latest_priors = {1: (1.0, 1.0)}
    coordinator.latest_priors_stamp_ns = 1
    coordinator.latest_prior_model_id = 'model'
    coordinator.latest_priors_request_id = 2
    coordinator.latest_priors_graph_id = 'cognitive'
    coordinator.latest_priors_graph_revision = 5
    coordinator.cognitive_constraints_cache = SimpleNamespace(
        invalidate=lambda: None)
    reconciliations = []
    coordinator._publish_runtime_states = lambda **_kwargs: None
    coordinator._ensure_desired_graph = (
        lambda reason, **_kwargs: reconciliations.append(reason))

    coordinator._on_reset_event(None)

    identity = coordinator.cognitive_graph_identity
    assert identity.reset_epoch == 4
    assert identity.recurrent_session_id == ''
    assert identity.cognitive_tile_id == ''
    assert identity.tile_revision == 0
    assert identity.model_id == ''
    assert coordinator.cognitive_graph_last_sequence == 0
    assert coordinator.latest_priors == {}
    assert coordinator.graph_generation == 5
    assert coordinator.graph_switch_generation == 8
    assert coordinator.cognitive_graph_feedback_active is None
    assert coordinator.cognitive_feedback_sequences == {}
    assert coordinator.cognitive_validation_terminal == set()
    assert coordinator.cognitive_outcome_terminal == set()
    assert coordinator.pending_reroute_outcome is None
    assert coordinator.cognitive_reroute_revision == 0
    assert reconciliations == ['simulation reset requires Route Server GVG']


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


def test_set_route_graph_rejection_requests_one_whole_gvg_fallback():
    coordinator = RouteCoordinator.__new__(RouteCoordinator)
    coordinator.cognitive_graph_mode = 'primary'
    coordinator.primary_fallback_used = False
    coordinator.cognitive_graph_switch_pending = True
    coordinator.graph = Graph('cognitive', 5, 'map', 0.05, [], [])
    coordinator.gvg_graph = _physical_graph()
    coordinator.StructuralGraphStatus = SimpleNamespace(LAST_KNOWN_GOOD=2)
    coordinator._publish_structural_status = lambda *_args: None
    switches = []
    coordinator._request_graph_switch = lambda graph, reason, fallback: switches.append(
        (graph.graph_id, reason, fallback)
    )
    rejected = SimpleNamespace(
        result=lambda: SimpleNamespace(success=False))

    coordinator._finish_cognitive_graph_switch(
        rejected, coordinator.graph, None, 'selected rejected', False)

    assert switches == [('physical', 'selected rejected', True)]


def test_old_set_route_graph_callback_generation_is_discarded():
    coordinator = RouteCoordinator.__new__(RouteCoordinator)
    coordinator.request_id = 9
    coordinator.graph_generation = 4
    coordinator.graph_switch_generation = 7
    coordinator.cognitive_graph_switch_pending = True
    coordinator.StructuralGraphStatus = SimpleNamespace(LAST_KNOWN_GOOD=2)
    coordinator._publish_structural_status = lambda *_args: None
    coordinator._primary_fallback_available = lambda: False
    old = GraphSwitchGeneration(7, 8, 4)
    evaluated = []
    future = SimpleNamespace(result=lambda: evaluated.append(True))

    coordinator._finish_cognitive_graph_switch(
        future, _physical_graph(), None, 'old switch', False, old)

    assert evaluated == [True]
    assert coordinator.cognitive_graph_switch_pending is True


def _pending_graph_transaction(requested_graph):
    coordinator = RouteCoordinator.__new__(RouteCoordinator)
    physical = _physical_graph()
    physical_support = SimpleNamespace(geojson={'features': []})
    coordinator.graph = physical
    coordinator.support = physical_support
    coordinator.gvg_graph = physical
    coordinator.gvg_support = physical_support
    coordinator.map = _map()
    coordinator.request_id = 10
    coordinator.graph_generation = 4
    coordinator.graph_switch_generation = 7
    coordinator.reset_generation = 0
    coordinator.structural_generation = 3
    coordinator.desired_graph_generation = 1
    coordinator.desired_graph = requested_graph
    coordinator.desired_support = SimpleNamespace(geojson={'features': []})
    coordinator.graph_coherent = False
    transaction = GraphSwitchGeneration(
        7, None, 4, 0, 1, requested_graph.graph_id, requested_graph.revision)
    coordinator.graph_transaction_generation = transaction
    coordinator.cognitive_graph_switch_pending = True
    coordinator.cognitive_graph_identity = _identity()
    coordinator.cognitive_graph_last_sequence = 9
    coordinator.cognitive_graph_feedback_active = None
    coordinator.cognitive_graph_feedback_pending = None
    coordinator.cognitive_feedback_sequences = {}
    coordinator.cognitive_validation_terminal = set()
    coordinator.cognitive_outcome_terminal = set()
    coordinator.pending_reroute_outcome = None
    coordinator.cognitive_reroute_revision = 0
    coordinator.primary_fallback_used = False
    coordinator.route_active = False
    coordinator.pending_goal = None
    coordinator.navigation_goal_pending = False
    coordinator.navigation_goal_handle = None
    coordinator.navigation_goal_targets_final = False
    coordinator.navigation_failed = False
    coordinator.runtime = SimpleNamespace(edges={})
    coordinator.pending_structural_map = None
    coordinator.cognitive_constraints_cache = SimpleNamespace(invalidate=lambda: None)
    coordinator.region_selector = None
    coordinator.tf_buffer = None
    coordinator._publish_runtime_states = lambda **_kwargs: None
    coordinator._publish_structural_status = lambda *_args: None
    coordinator.node = SimpleNamespace(get_logger=lambda: SimpleNamespace(
        info=lambda *_args: None, warning=lambda *_args: None))
    coordinator.StructuralGraphStatus = SimpleNamespace(
        LAST_KNOWN_GOOD=2, READY=1)
    coordinator.module2_enabled = False
    coordinator.requests = []
    coordinator._request_graph_switch = lambda graph, reason, fallback: (
        coordinator.requests.append((graph.graph_id, reason, fallback)))
    return coordinator, transaction


def test_reset_stale_cognitive_success_requests_gvg_compensation():
    cognitive = Graph('cognitive', 5, 'map', 0.05, [], [])
    coordinator, transaction = _pending_graph_transaction(cognitive)
    support = SimpleNamespace(geojson={'features': []})
    result_calls = []

    coordinator._on_reset_event(None)
    coordinator._finish_cognitive_graph_switch(
        SimpleNamespace(result=lambda: (
            result_calls.append(True) or SimpleNamespace(success=True))),
        cognitive, support, 'late cognitive', False, transaction,
    )

    assert result_calls == [True]
    assert coordinator.graph.graph_id == 'physical'
    assert coordinator.graph_transaction_generation is None
    assert coordinator.graph_coherent is False
    coordinator.graph_retry_due_steady_s = 0.0
    coordinator._graph_reconciliation_tick()
    assert coordinator.requests == [
        ('physical', 'stale SetRouteGraph success', True)]


def test_rebuild_reset_fresh_goal_late_success_does_not_commit_old_graph():
    rebuilt = Graph('rebuilt', 5, 'map:structural', 0.05, [], [])
    coordinator, transaction = _pending_graph_transaction(rebuilt)
    old_map = coordinator.map
    rebuilt_map = _map(wall=True)
    support = SimpleNamespace(geojson={'features': []})
    generation = StructuralRebuildGeneration(
        10, 0, 3, 1, rebuilt.graph_id, rebuilt.revision,
        4, 1, id(rebuilt_map))
    goal = SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=1.0, y=2.0)))

    coordinator._on_reset_event(None)
    coordinator._on_goal(goal)
    coordinator._finish_rebuild(
        SimpleNamespace(result=lambda: SimpleNamespace(success=True)),
        rebuilt, rebuilt_map, support, generation, transaction,
    )

    assert coordinator.graph.graph_id == 'physical'
    assert coordinator.gvg_graph.graph_id == 'physical'
    assert coordinator.map is old_map
    assert coordinator.pending_goal is goal
    coordinator.graph_retry_due_steady_s = 0.0
    coordinator._graph_reconciliation_tick()
    assert coordinator.requests == [
        ('physical', 'stale structural rebuild success', True)]


def test_cognitive_validation_crossing_reset_is_discarded(monkeypatch):
    coordinator = _feedback_coordinator(graph_id='physical', revision=4)
    coordinator.pending_goal = None
    coordinator.request_id = 8
    coordinator.graph_generation = 4
    coordinator.reset_generation = 0
    coordinator.primary_fallback_used = False
    coordinator.cognitive_graph_switch_pending = False
    coordinator.cognitive_graph_mode = 'primary'
    coordinator.cognitive_graph_last_sequence = 0
    coordinator.cognitive_graph_identity = _identity()
    coordinator.map = _map()
    coordinator.gvg_graph = coordinator.graph
    coordinator.defaults = {'footprint': FOOTPRINT}
    coordinator.StructuralGraphStatus = SimpleNamespace(
        LAST_KNOWN_GOOD=2, READY=1)
    coordinator._now = lambda: SimpleNamespace(
        nanoseconds=10_100_000_000, to_msg=lambda: object())
    coordinator._publish_structural_status = lambda *_args: None
    switches = []
    coordinator._request_graph_switch = (
        lambda graph, detail, **kwargs: switches.append((graph, detail, kwargs))
    )
    validation_ready = threading.Event()
    release_validation = threading.Event()
    original_validate = ros_node_module.validate_cognitive_graph_candidate

    def blocked_validate(*args, **kwargs):
        result = original_validate(*args, **kwargs)
        validation_ready.set()
        assert release_validation.wait(timeout=2.0)
        return result

    monkeypatch.setattr(
        ros_node_module, 'validate_cognitive_graph_candidate', blocked_validate)
    worker = threading.Thread(
        target=coordinator._on_cognitive_graph,
        args=(_candidate(sequence=8),),
    )
    worker.start()
    assert validation_ready.wait(timeout=2.0)
    with coordinator._route_state_lock():
        coordinator.request_id = 9
        coordinator.reset_generation = 1
        coordinator.cognitive_graph_identity = CognitiveGraphIdentity(
            4, '', 'map', '', 0, 'physical', 4, '')
    release_validation.set()
    worker.join(timeout=2.0)

    assert not worker.is_alive()
    assert switches == []
    assert coordinator.cognitive_graph_last_sequence == 0
    assert coordinator.cognitive_graph_identity.reset_epoch == 4
    assert coordinator.cognitive_graph_validation_pub.messages == []


def test_route_graph_transaction_paths_are_unique_and_do_not_cross_write(
    tmp_path, monkeypatch,
):
    monkeypatch.setattr(
        ros_node_module.tempfile, 'gettempdir', lambda: str(tmp_path))
    first = GraphSwitchGeneration(7, 10, 4, 1, 3, 'graph-a', 5)
    second = GraphSwitchGeneration(8, 11, 4, 1, 4, 'graph-b', 6)

    first_graph, first_map = RouteCoordinator._graph_transaction_paths(
        'selected', first)
    second_graph, second_map = RouteCoordinator._graph_transaction_paths(
        'selected', second)
    first_graph.write_text('first-graph', encoding='utf-8')
    first_map.write_text('first-map', encoding='utf-8')
    second_graph.write_text('second-graph', encoding='utf-8')
    second_map.write_text('second-map', encoding='utf-8')

    assert first_graph != second_graph
    assert first_map != second_map
    assert 'switch_7_selected_' in first_graph.parent.name
    assert 'switch_8_selected_' in second_graph.parent.name
    assert first_graph.read_text(encoding='utf-8') == 'first-graph'
    assert first_map.read_text(encoding='utf-8') == 'first-map'
    assert second_graph.read_text(encoding='utf-8') == 'second-graph'
    assert second_map.read_text(encoding='utf-8') == 'second-map'


def test_reassert_transaction_is_reserved_before_export(monkeypatch):
    graph = _physical_graph()
    coordinator = RouteCoordinator.__new__(RouteCoordinator)
    coordinator.graph = graph
    coordinator.support = SimpleNamespace(geojson={'features': []})
    coordinator.gvg_graph = graph
    coordinator.gvg_support = coordinator.support
    coordinator.desired_graph = graph
    coordinator.desired_support = coordinator.support
    coordinator.desired_graph_generation = 2
    coordinator.graph_generation = 4
    coordinator.graph_switch_generation = 6
    coordinator.reset_generation = 1
    coordinator.graph_transaction_generation = None
    coordinator.graph_coherent = False
    coordinator.graph_reassert_required = True
    coordinator.cognitive_graph_switch_pending = False
    coordinator.cognitive_graph_feedback_pending = None
    coordinator.pending_goal = object()
    coordinator.request_id = 10
    coordinator.defaults = {'graph': {'route_support_spacing_m': 0.20}}
    coordinator.SetRouteGraph = SimpleNamespace(
        Request=lambda: SimpleNamespace(graph_filepath=''))
    submitted_paths = []
    coordinator.set_graph_client = SimpleNamespace(
        service_is_ready=lambda: True,
        call_async=lambda request: (
            submitted_paths.append(request.graph_filepath)
            or SimpleNamespace(add_done_callback=lambda _callback: None)
        ),
    )
    export_ready = threading.Event()
    release_export = threading.Event()
    original_export = ros_node_module.export_route_support_graph

    def blocked_export(*args, **kwargs):
        result = original_export(*args, **kwargs)
        export_ready.set()
        assert release_export.wait(timeout=2.0)
        return result

    monkeypatch.setattr(
        ros_node_module, 'export_route_support_graph', blocked_export)
    worker = threading.Thread(
        target=coordinator._request_graph_switch,
        args=(graph, 'reset reassert'),
        kwargs={'fallback': True},
    )
    worker.start()
    assert export_ready.wait(timeout=2.0)
    with coordinator._route_state_lock():
        assert coordinator.graph_transaction_generation is not None
        assert coordinator.graph_coherent is False
        assert coordinator.graph_reassert_required is True
        assert coordinator._desired_graph_is_coherent_locked() is False
    release_export.set()
    worker.join(timeout=2.0)

    assert not worker.is_alive()
    assert len(submitted_paths) == 1
    assert 'switch_7_gvg_fallback_' in submitted_paths[0]


class _DeferredGraphFuture:
    def __init__(self):
        self.callback = None
        self.response = None
        self.error = None

    def add_done_callback(self, callback):
        self.callback = callback

    def result(self):
        if self.error is not None:
            raise self.error
        return self.response

    def finish(self, *, success=None, error=None):
        self.error = error
        self.response = (
            None if success is None else SimpleNamespace(success=bool(success))
        )
        assert self.callback is not None
        self.callback(self)


class _SetGraphClient:
    def __init__(self):
        self.ready = True
        self.raise_call = False
        self.calls = []
        self.futures = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.calls.append(request.graph_filepath)
        if self.raise_call:
            raise RuntimeError('call failed')
        future = _DeferredGraphFuture()
        self.futures.append(future)
        return future


def _reassert_liveness_coordinator(monkeypatch):
    coordinator = RouteCoordinator.__new__(RouteCoordinator)
    graph = _physical_graph()
    support = SimpleNamespace(geojson={'features': []})
    coordinator.graph = graph
    coordinator.support = support
    coordinator.gvg_graph = graph
    coordinator.gvg_support = support
    coordinator.desired_graph = graph
    coordinator.desired_support = support
    coordinator.desired_graph_generation = 2
    coordinator.graph_generation = 4
    coordinator.graph_switch_generation = 6
    coordinator.reset_generation = 1
    coordinator.graph_coherent = False
    coordinator.graph_reassert_required = True
    coordinator.graph_transaction_generation = None
    coordinator.graph_transaction_future = None
    coordinator.graph_transaction_deadline_steady_s = None
    coordinator.graph_transaction_kind = None
    coordinator.graph_retry_key = None
    coordinator.graph_retry_attempt = 0
    coordinator.graph_retry_due_steady_s = None
    coordinator.graph_retry_reason = ''
    coordinator.graph_retry_kind = 'switch'
    coordinator.cognitive_graph_switch_pending = False
    coordinator.cognitive_graph_feedback_pending = None
    coordinator.cognitive_graph_feedback_active = None
    coordinator.pending_reroute_outcome = None
    coordinator.cognitive_constraints_cache = SimpleNamespace(invalidate=lambda: None)
    coordinator.pending_goal = object()
    coordinator.route_active = True
    coordinator.request_id = 10
    coordinator.navigation_goal_handle = None
    coordinator.navigation_goal_pending = False
    coordinator.navigation_goal_targets_final = False
    coordinator.navigation_failed = False
    coordinator.module2_enabled = False
    coordinator.defaults = {'graph': {'route_support_spacing_m': 0.20}}
    coordinator.SetRouteGraph = SimpleNamespace(
        Request=lambda: SimpleNamespace(graph_filepath=''))
    coordinator.set_graph_client = _SetGraphClient()
    coordinator.node = SimpleNamespace(get_logger=lambda: SimpleNamespace(
        info=lambda *_args: None, warning=lambda *_args: None))
    coordinator.StructuralGraphStatus = SimpleNamespace(
        LAST_KNOWN_GOOD=2, READY=1)
    coordinator._publish_structural_status = lambda *_args: None
    coordinator._publish_graph = lambda: None
    coordinator._publish_cognitive_constraints = lambda: None
    coordinator._clear_latest_priors = lambda: None
    coordinator._clear_pending_prior_request = lambda: None
    coordinator._publish_route_context = lambda: None
    coordinator.prepared = []
    coordinator._prepare_route = lambda priors: coordinator.prepared.append(priors)
    coordinator.steady_s = 10.0
    coordinator._steady_now = lambda: coordinator.steady_s
    monkeypatch.setattr(ros_node_module, 'save_route_support', lambda *_args: None)
    return coordinator


def _install_recording_structural_intent(coordinator):
    candidate = object()
    coordinator.pending_structural_map = candidate
    coordinator.structural_candidate_generation = 0
    coordinator.pending_structural_intent = None
    with coordinator._route_state_lock():
        coordinator._refresh_structural_intent_locked()
    coordinator.structural_submits = []

    def submit_latest():
        if (
            coordinator.graph_transaction_generation is not None
            or coordinator.route_active
            or coordinator.pending_goal is not None
            or coordinator.graph_retry_due_steady_s is not None
        ):
            return
        coordinator.structural_submits.append(
            coordinator.pending_structural_intent.candidate_generation)
        coordinator.pending_structural_map = None
        coordinator.pending_structural_intent = None

    coordinator._rebuild_structural_graph = submit_latest
    return candidate


def _structural_liveness_coordinator(monkeypatch):
    coordinator = _reassert_liveness_coordinator(monkeypatch)
    coordinator.StructuralGraphStatus.REBUILDING = 3
    coordinator.defaults.update({'footprint': {}, 'route_cost': {}})
    coordinator.map = _map()
    coordinator.pending_goal = None
    coordinator.route_active = False
    coordinator.structural_generation = 0
    coordinator.structural_candidate_generation = 0
    coordinator.pending_structural_intent = None
    coordinator.feasible_only_largest_component = False
    coordinator.structural_monitor = SimpleNamespace(accept_rebuild=lambda: None)
    coordinator.cognitive_graph_identity = _identity()
    coordinator.cognitive_graph_last_sequence = 0
    coordinator.support_node_positions = {}
    coordinator._publish_graph = lambda: None
    coordinator._publish_cognitive_constraints = lambda: None
    rebuilt = Graph('rebuilt', 5, 'map:structural', 0.05, [], [])
    support = SimpleNamespace(geojson={'features': []})
    monkeypatch.setattr(ros_node_module, 'build_gvg', lambda *_args, **_kwargs: rebuilt)
    monkeypatch.setattr(
        ros_node_module, 'apply_footprint_feasibility',
        lambda graph, *_args, **_kwargs: graph)
    monkeypatch.setattr(
        ros_node_module, 'stabilize_graph_ids',
        lambda graph, *_args, **_kwargs: graph)
    monkeypatch.setattr(
        ros_node_module, 'export_route_support_graph',
        lambda *_args, **_kwargs: support)
    return coordinator, rebuilt


def test_reassert_service_unavailable_rejection_backoff_and_no_storm(monkeypatch):
    coordinator = _reassert_liveness_coordinator(monkeypatch)
    client = coordinator.set_graph_client
    client.ready = False

    coordinator._ensure_desired_graph('reset GVG')
    assert client.calls == []
    assert coordinator.graph_retry_due_steady_s == pytest.approx(10.25)
    coordinator._ensure_desired_graph('duplicate reset GVG')
    assert client.calls == []
    assert coordinator.graph_retry_due_steady_s == pytest.approx(10.25)
    coordinator.steady_s = 10.24
    coordinator._graph_reconciliation_tick()
    assert client.calls == []

    client.ready = True
    coordinator.steady_s = 10.25
    coordinator._graph_reconciliation_tick()
    assert len(client.calls) == 1
    for expected_delay in (0.5, 1.0, 2.0, 2.0):
        client.futures[-1].finish(success=False)
        due = coordinator.graph_retry_due_steady_s
        assert due - coordinator.steady_s == pytest.approx(expected_delay)
        coordinator._graph_reconciliation_tick()
        assert len(client.calls) == len(client.futures)
        coordinator.steady_s = due
        coordinator._graph_reconciliation_tick()
        assert len(client.calls) == len(client.futures)
    assert len(client.calls) == 5
    assert coordinator.graph_coherent is False
    assert coordinator.graph_reassert_required is True
    assert coordinator.prepared == []


@pytest.mark.parametrize('first_outcome', ('rejected', 'exception'))
def test_deferred_structural_intent_survives_switch_failure_until_recovery(
    monkeypatch, first_outcome,
):
    coordinator = _reassert_liveness_coordinator(monkeypatch)
    coordinator.pending_goal = None
    coordinator.route_active = False
    _install_recording_structural_intent(coordinator)

    coordinator._ensure_desired_graph('idle graph switch')
    first = coordinator.set_graph_client.futures[0]
    if first_outcome == 'exception':
        first.finish(error=RuntimeError('switch failed'))
    else:
        first.finish(success=False)
    assert coordinator.structural_submits == []

    coordinator.steady_s = coordinator.graph_retry_due_steady_s
    coordinator._graph_reconciliation_tick()
    coordinator.set_graph_client.futures[1].finish(success=True)

    assert coordinator.structural_submits == [1]


def test_deferred_structural_intent_wakes_after_switch_success_once(monkeypatch):
    coordinator = _reassert_liveness_coordinator(monkeypatch)
    coordinator.pending_goal = None
    coordinator.route_active = False
    _install_recording_structural_intent(coordinator)

    coordinator._ensure_desired_graph('idle graph switch')
    coordinator.set_graph_client.futures[0].finish(success=True)
    coordinator._graph_reconciliation_tick()

    assert coordinator.structural_submits == [1]


def test_hung_switch_recovery_wakes_structural_and_late_success_does_not_repeat(
    monkeypatch,
):
    coordinator = _reassert_liveness_coordinator(monkeypatch)
    coordinator.pending_goal = None
    coordinator.route_active = False
    _install_recording_structural_intent(coordinator)

    coordinator._ensure_desired_graph('idle graph switch')
    first = coordinator.set_graph_client.futures[0]
    coordinator.steady_s = 12.0
    coordinator._graph_reconciliation_tick()
    assert coordinator.structural_submits == []
    coordinator.steady_s = coordinator.graph_retry_due_steady_s
    coordinator._graph_reconciliation_tick()
    coordinator.set_graph_client.futures[1].finish(success=True)
    first.finish(success=True)

    assert coordinator.structural_submits == [1]


def test_structural_service_unavailable_retains_latest_with_bounded_retry(
    monkeypatch,
):
    coordinator, _rebuilt = _structural_liveness_coordinator(monkeypatch)
    candidate = _map(wall=True)
    coordinator.pending_structural_map = candidate
    with coordinator._route_state_lock():
        intent = coordinator._refresh_structural_intent_locked()
    coordinator.set_graph_client.ready = False

    coordinator._try_deferred_structural_rebuild()
    first_due = coordinator.graph_retry_due_steady_s
    coordinator._try_deferred_structural_rebuild()

    assert coordinator.pending_structural_map is candidate
    assert coordinator.pending_structural_intent == intent
    assert coordinator.set_graph_client.calls == []
    assert first_due == pytest.approx(10.25)
    assert coordinator.graph_retry_due_steady_s == first_due

    coordinator.set_graph_client.ready = True
    coordinator.steady_s = first_due
    coordinator._graph_reconciliation_tick()
    assert len(coordinator.set_graph_client.calls) == 1
    coordinator.set_graph_client.futures[0].finish(success=True)
    assert coordinator.pending_structural_map is None


def test_structural_candidate_coalesces_while_transaction_is_in_flight(
    monkeypatch,
):
    coordinator, _rebuilt = _structural_liveness_coordinator(monkeypatch)
    first_map = _map(wall=True)
    coordinator.pending_structural_map = first_map
    with coordinator._route_state_lock():
        first_intent = coordinator._refresh_structural_intent_locked()
    coordinator._try_deferred_structural_rebuild()
    first_future = coordinator.set_graph_client.futures[0]

    latest_map = _map(wall=False)
    coordinator.pending_structural_map = latest_map
    with coordinator._route_state_lock():
        latest_intent = coordinator._refresh_structural_intent_locked()
    coordinator._try_deferred_structural_rebuild()
    coordinator._graph_reconciliation_tick()
    assert len(coordinator.set_graph_client.calls) == 1
    assert latest_intent.candidate_generation == first_intent.candidate_generation + 1

    first_future.finish(success=True)
    coordinator.steady_s = coordinator.graph_retry_due_steady_s
    coordinator._graph_reconciliation_tick()
    coordinator.set_graph_client.futures[1].finish(success=True)
    assert len(coordinator.set_graph_client.calls) == 3
    coordinator.set_graph_client.futures[2].finish(success=True)

    assert coordinator.pending_structural_map is None
    assert coordinator.map is latest_map


@pytest.mark.parametrize('first_outcome', ('rejected', 'exception'))
def test_structural_transaction_failure_retains_candidate_and_retries(
    monkeypatch, first_outcome,
):
    coordinator, _rebuilt = _structural_liveness_coordinator(monkeypatch)
    candidate = _map(wall=True)
    coordinator.pending_structural_map = candidate
    with coordinator._route_state_lock():
        coordinator._refresh_structural_intent_locked()
    coordinator._try_deferred_structural_rebuild()
    first = coordinator.set_graph_client.futures[0]

    if first_outcome == 'exception':
        first.finish(error=RuntimeError('structural transaction failed'))
    else:
        first.finish(success=False)
    assert coordinator.pending_structural_map is candidate
    assert coordinator.graph_retry_kind == 'structural'

    coordinator.steady_s = coordinator.graph_retry_due_steady_s
    coordinator._graph_reconciliation_tick()
    assert len(coordinator.set_graph_client.calls) == 2
    coordinator.set_graph_client.futures[1].finish(success=True)
    assert coordinator.pending_structural_map is None


def test_hung_structural_transaction_recovers_and_late_success_cannot_recommit(
    monkeypatch,
):
    coordinator, _rebuilt = _structural_liveness_coordinator(monkeypatch)
    old_map = coordinator.map
    candidate = _map(wall=True)
    coordinator.pending_structural_map = candidate
    with coordinator._route_state_lock():
        coordinator._refresh_structural_intent_locked()
    coordinator._try_deferred_structural_rebuild()
    first = coordinator.set_graph_client.futures[0]

    coordinator.steady_s = 12.0
    coordinator._graph_reconciliation_tick()
    coordinator.steady_s = coordinator.graph_retry_due_steady_s
    coordinator._graph_reconciliation_tick()
    coordinator.set_graph_client.futures[1].finish(success=True)
    committed_generation = coordinator.graph_generation
    assert coordinator.map is candidate

    first.finish(success=True)
    assert coordinator.map is candidate
    assert coordinator.map is not old_map
    assert coordinator.graph_generation == committed_generation
    assert coordinator.pending_structural_map is None


def test_reset_clears_in_flight_structural_intent_and_late_success_is_stale(
    monkeypatch,
):
    coordinator, _rebuilt = _structural_liveness_coordinator(monkeypatch)
    old_map = coordinator.map
    coordinator.pending_structural_map = _map(wall=True)
    with coordinator._route_state_lock():
        coordinator._refresh_structural_intent_locked()
    coordinator._try_deferred_structural_rebuild()
    first = coordinator.set_graph_client.futures[0]

    coordinator._on_reset_event(None)
    assert coordinator.pending_structural_map is None
    assert coordinator.pending_structural_intent is None
    first.finish(success=True)

    assert coordinator.map is old_map
    assert coordinator.pending_structural_map is None


def test_navigation_terminal_defers_structural_until_graph_transaction_retires(
    monkeypatch,
):
    coordinator = _reassert_liveness_coordinator(monkeypatch)
    coordinator.cognitive_graph_mode = 'gvg'
    coordinator.navigation_goal_pending = True
    coordinator.goal_complete_pub = _Publisher()
    coordinator.goal_result_pub = _Publisher()
    coordinator.cognitive_graph_identity = _identity()
    generation = coordinator._route_callback_generation()
    _install_recording_structural_intent(coordinator)
    coordinator._ensure_desired_graph('active route graph transaction')
    first = coordinator.set_graph_client.futures[0]

    rejected = SimpleNamespace(result=lambda: SimpleNamespace(accepted=False))
    coordinator._on_navigation_goal_handle(rejected, generation)
    coordinator._on_navigation_goal_handle(rejected, generation)
    assert coordinator.structural_submits == []
    assert len(coordinator.goal_complete_pub.messages) == 1
    assert len(coordinator.goal_result_pub.messages) == 1

    first.finish(success=True)
    coordinator.steady_s = coordinator.graph_retry_due_steady_s
    coordinator._graph_reconciliation_tick()
    coordinator.set_graph_client.futures[1].finish(success=True)

    assert coordinator.structural_submits == [1]


def test_reassert_call_exception_retries_on_steady_clock(monkeypatch):
    coordinator = _reassert_liveness_coordinator(monkeypatch)
    coordinator.set_graph_client.raise_call = True
    coordinator._now = lambda: SimpleNamespace(nanoseconds=123)  # frozen ROS clock

    coordinator._ensure_desired_graph('reset GVG')
    assert len(coordinator.set_graph_client.calls) == 1
    assert coordinator.graph_retry_due_steady_s == pytest.approx(10.25)
    coordinator.steady_s = 10.25
    coordinator._graph_reconciliation_tick()
    assert len(coordinator.set_graph_client.calls) == 2
    assert coordinator.graph_retry_due_steady_s == pytest.approx(10.75)


def test_cognitive_retry_preserves_validation_context(monkeypatch):
    coordinator = _reassert_liveness_coordinator(monkeypatch)
    coordinator.set_graph_client.raise_call = True
    coordinator.cognitive_graph_identity = CognitiveGraphIdentity(
        3, '', 'map', '', 0, 'physical', 4, '')
    coordinator.cognitive_graph_last_sequence = 0
    validations = []
    coordinator._publish_graph_validation = (
        lambda feedback, accepted, reason: validations.append(
            (feedback, accepted, reason)))
    cognitive = Graph('candidate:primary', 5, 'map', 0.05, [], [])
    feedback = _feedback()
    candidate = SimpleNamespace(identity=_identity(), source_sequence=7)

    coordinator._request_graph_switch(
        cognitive,
        'candidate switch',
        fallback=False,
        feedback=feedback,
        candidate=candidate,
    )
    assert coordinator.graph_retry_switch_context[2:4] == (feedback, candidate)
    coordinator.set_graph_client.raise_call = False
    coordinator.steady_s = coordinator.graph_retry_due_steady_s
    coordinator._graph_reconciliation_tick()
    assert coordinator.graph_transaction_switch_context[2:4] == (
        feedback, candidate)
    coordinator.set_graph_client.futures[0].finish(success=True)

    assert coordinator.cognitive_graph_identity == candidate.identity
    assert validations == [(feedback, True, 'set_route_graph_accepted')]


def test_hung_timeout_late_failure_is_harmless_after_recovery(monkeypatch):
    coordinator = _reassert_liveness_coordinator(monkeypatch)
    coordinator._ensure_desired_graph('reset GVG')
    first = coordinator.set_graph_client.futures[0]
    assert coordinator.graph_transaction_deadline_steady_s == pytest.approx(12.0)

    coordinator.steady_s = 12.0
    coordinator._graph_reconciliation_tick()
    assert coordinator.graph_transaction_generation is None
    assert coordinator.graph_retry_due_steady_s == pytest.approx(12.25)
    coordinator.steady_s = 12.25
    coordinator._graph_reconciliation_tick()
    second = coordinator.set_graph_client.futures[1]
    second.finish(success=True)
    assert coordinator.graph_coherent is True
    assert coordinator.prepared == [{}]

    first.finish(success=False)
    assert coordinator.graph_coherent is True
    assert coordinator.graph_retry_due_steady_s is None


def test_hung_timeout_late_success_fails_closed_and_compensates(monkeypatch):
    coordinator = _reassert_liveness_coordinator(monkeypatch)
    coordinator._ensure_desired_graph('reset GVG')
    first = coordinator.set_graph_client.futures[0]
    coordinator.steady_s = 12.0
    coordinator._graph_reconciliation_tick()
    coordinator.steady_s = 12.25
    coordinator._graph_reconciliation_tick()
    coordinator.set_graph_client.futures[1].finish(success=True)
    assert coordinator.graph_coherent is True

    first.finish(success=True)
    assert coordinator.graph_coherent is False
    assert coordinator.graph_reassert_required is True
    due = coordinator.graph_retry_due_steady_s
    coordinator.steady_s = due
    coordinator._graph_reconciliation_tick()
    assert len(coordinator.set_graph_client.calls) == 3


@pytest.mark.parametrize('kind', ('switch', 'structural'))
def test_hung_graph_transaction_crossing_reset_eventually_requests_fresh_gvg(
    kind,
):
    cognitive = Graph('cognitive', 5, 'map', 0.05, [], [])
    coordinator, transaction = _pending_graph_transaction(cognitive)
    coordinator.graph_transaction_future = _DeferredGraphFuture()
    coordinator.graph_transaction_deadline_steady_s = 12.0
    coordinator.graph_transaction_kind = kind
    coordinator.steady_s = 10.0
    coordinator._steady_now = lambda: coordinator.steady_s

    coordinator._on_reset_event(None)
    assert coordinator.requests == []
    coordinator.steady_s = 12.0
    coordinator._graph_reconciliation_tick()

    assert coordinator.requests == [
        ('physical', 'simulation reset requires Route Server GVG', True)]
    assert coordinator.graph_coherent is False


def test_pending_goal_waits_for_reassert_success_before_prepare(monkeypatch):
    coordinator = _reassert_liveness_coordinator(monkeypatch)
    coordinator._ensure_desired_graph('new goal GVG')
    assert coordinator.prepared == []
    coordinator.set_graph_client.futures[0].finish(success=True)
    assert coordinator.prepared == [{}]


def test_goal_arriving_during_unbound_reassert_requires_request_bound_retry(
    monkeypatch,
):
    coordinator = _reassert_liveness_coordinator(monkeypatch)
    coordinator.pending_goal = None
    coordinator.route_active = False
    coordinator._ensure_desired_graph('reset GVG without goal')
    first = coordinator.set_graph_client.futures[0]

    coordinator.request_id = 11
    coordinator.pending_goal = object()
    coordinator.route_active = True
    with coordinator._route_state_lock():
        coordinator._set_desired_graph_locked(
            coordinator.gvg_graph,
            coordinator.gvg_support,
            require_reassert=True,
        )
    coordinator._ensure_desired_graph('new goal requires request-bound GVG')
    first.finish(success=True)
    assert coordinator.graph_coherent is False
    assert coordinator.prepared == []

    coordinator.graph_retry_due_steady_s = coordinator.steady_s
    coordinator._graph_reconciliation_tick()
    second = coordinator.set_graph_client.futures[1]
    assert coordinator.graph_transaction_generation.route_request_id == 11
    second.finish(success=True)
    assert coordinator.prepared == [{}]


def test_shadow_validation_ack_is_not_an_execution_outcome():
    coordinator = _feedback_coordinator(graph_id='physical', revision=4)
    coordinator.pending_goal = None
    coordinator.primary_fallback_used = False
    coordinator.cognitive_graph_switch_pending = False
    coordinator.cognitive_graph_mode = 'shadow'
    coordinator.cognitive_graph_last_sequence = 0
    coordinator.cognitive_graph_identity = CognitiveGraphIdentity(
        3, '', 'map', '', 0, 'physical', 4, '')
    coordinator.map = _map()
    coordinator.defaults = {'footprint': FOOTPRINT}
    coordinator.StructuralGraphStatus = SimpleNamespace(
        LAST_KNOWN_GOOD=2, READY=1)
    now = SimpleNamespace(nanoseconds=10_100_000_000, to_msg=lambda: object())
    coordinator._now = lambda: now
    coordinator._publish_structural_status = lambda *_args: None

    coordinator._on_cognitive_graph(_candidate())

    ack = coordinator.cognitive_graph_validation_pub.messages[0]
    assert ack.accepted is False
    assert ack.reason == 'physically_validated_shadow_not_selected'
    assert ack.validated_graph_id == 'physical'
    assert ack.validated_graph_revision == 4
    assert ack.validated_edge_id == ''
    assert coordinator.cognitive_edge_outcome_pub.messages == []


@pytest.mark.parametrize('shape', ('one_node', 'zero_edge', 'no_mapping'))
def test_immature_primary_candidate_retains_gvg_and_later_matures(shape):
    coordinator = _feedback_coordinator(graph_id='physical', revision=4)
    coordinator.pending_goal = None
    coordinator.primary_fallback_used = False
    coordinator.cognitive_graph_switch_pending = False
    coordinator.cognitive_graph_mode = 'primary'
    coordinator.cognitive_graph_last_sequence = 0
    coordinator.cognitive_graph_identity = _identity()
    coordinator.map = _map()
    coordinator.defaults = {'footprint': FOOTPRINT}
    coordinator.StructuralGraphStatus = SimpleNamespace(
        LAST_KNOWN_GOOD=2, READY=1)
    coordinator._now = lambda: SimpleNamespace(
        nanoseconds=10_100_000_000, to_msg=lambda: object())
    statuses = []
    coordinator._publish_structural_status = (
        lambda status, detail: statuses.append((status, detail))
    )
    switches = []
    coordinator._request_graph_switch = (
        lambda graph, detail, **kwargs: switches.append((graph, detail, kwargs))
    )
    immature = _candidate()
    if shape == 'one_node':
        immature.nodes = immature.nodes[:1]
        immature.edges = []
    elif shape == 'zero_edge':
        immature.edges = []
    else:
        immature.edges[0].edge_id = ''

    coordinator._on_cognitive_graph(immature)

    assert coordinator.graph.graph_id == 'physical'
    assert coordinator.primary_fallback_used is False
    assert coordinator.cognitive_graph_validation_pub.messages == []
    assert switches == []
    assert statuses[-1][1] == 'cognitive_graph_immature_gvg_bootstrap'

    coordinator._on_cognitive_graph(_candidate(sequence=8))
    assert len(switches) == 1
    assert switches[0][2]['fallback'] is False


def test_immature_cognitive_status_crossing_hold_is_silent(monkeypatch):
    coordinator = _feedback_coordinator(graph_id='physical', revision=4)
    coordinator.cognitive_graph_mode = 'primary'
    coordinator.StructuralGraphStatus = SimpleNamespace(
        LAST_KNOWN_GOOD=2, READY=1)
    statuses = []
    coordinator._publish_structural_status = (
        lambda state, detail: statuses.append((state, detail)))
    entered = threading.Barrier(2)
    release = threading.Event()

    def blocked_maturity(_message):
        entered.wait(timeout=2.0)
        assert release.wait(timeout=2.0)
        return False

    monkeypatch.setattr(
        ros_node_module,
        'cognitive_graph_candidate_is_mature',
        blocked_maturity,
    )
    callback = threading.Thread(
        target=coordinator._on_cognitive_graph,
        args=(_candidate(),),
    )
    callback.start()
    entered.wait(timeout=2.0)
    with coordinator._route_state_lock():
        coordinator.reset_hold_barrier = True
        coordinator.reset_intent_generation = 2
        coordinator.reset_generation = 1
        coordinator.request_id = 1
    release.set()
    callback.join(timeout=2.0)

    assert not callback.is_alive()
    assert statuses == []
    assert coordinator.cognitive_graph_validation_pub.messages == []


def test_set_route_graph_accepts_only_after_success_and_reject_does_not_bind():
    graph = Graph('candidate:primary', 5, 'map', 0.05, [], [])
    feedback = CognitiveGraphFeedback(
        'session', 3, 7, 'candidate', 5, 1, graph.graph_id, graph.revision,
        (('e0', '1'),),
    )
    candidate = SimpleNamespace(identity=_identity(), source_sequence=7)

    def configured():
        coordinator = _feedback_coordinator(
            graph_id='physical', revision=4)
        coordinator.cognitive_graph_identity = CognitiveGraphIdentity(
            3, '', 'map', '', 0, 'physical', 4, '')
        coordinator.cognitive_graph_last_sequence = 0
        coordinator.cognitive_graph_switch_pending = True
        coordinator.graph_generation = 0
        coordinator.pending_goal = None
        coordinator.support_node_positions = {}
        coordinator.cognitive_constraints_cache = SimpleNamespace(
            invalidate=lambda: None)
        coordinator._clear_latest_priors = lambda: None
        coordinator._publish_graph = lambda: None
        coordinator._publish_cognitive_constraints = lambda: None
        coordinator._publish_structural_status = lambda *_args: None
        coordinator.StructuralGraphStatus = SimpleNamespace(
            LAST_KNOWN_GOOD=2, READY=1)
        coordinator._primary_fallback_available = lambda: False
        return coordinator

    rejected = configured()
    rejected._finish_cognitive_graph_switch(
        SimpleNamespace(result=lambda: SimpleNamespace(success=False)),
        graph, None, 'rejected', False, None, feedback, candidate,
    )
    assert rejected.cognitive_graph_identity.recurrent_session_id == ''
    assert rejected.cognitive_graph_validation_pub.messages[0].accepted is False

    accepted = configured()
    support = SimpleNamespace(geojson={'features': []})
    accepted._finish_cognitive_graph_switch(
        SimpleNamespace(result=lambda: SimpleNamespace(success=True)),
        graph, support, 'accepted', False, None, feedback, candidate,
    )
    assert accepted.cognitive_graph_identity == _identity()
    assert accepted.cognitive_graph_last_sequence == 7
    assert accepted.cognitive_graph_validation_pub.messages[0].accepted is True


def test_edge_crossing_and_terminal_outcomes_are_published_once():
    coordinator = _feedback_coordinator()
    coordinator.cognitive_graph_feedback_active = _feedback()
    coordinator.tracker = SimpleNamespace(
        edge_index=1,
        edges=[SimpleNamespace(id=1), SimpleNamespace(id=2)],
    )

    coordinator._publish_crossed_edge_outcomes(0, 1)
    coordinator._publish_crossed_edge_outcomes(0, 1)
    coordinator._publish_crossed_edge_outcomes(1, 1)

    outcomes = coordinator.cognitive_edge_outcome_pub.messages
    assert len(outcomes) == 1
    assert outcomes[0].success is True
    assert outcomes[0].reason == 'route_tracker_edge_crossed'


def test_navigate_failure_is_once_and_no_edge_has_no_outcome():
    coordinator = _feedback_coordinator()
    coordinator.cognitive_graph_mode = 'shadow'
    coordinator.primary_fallback_used = False
    coordinator.cognitive_graph_feedback_active = _feedback()
    coordinator.tracker = SimpleNamespace(
        edge_index=0, edges=[SimpleNamespace(id=1)])

    coordinator._publish_navigation_edge_failure('navigate_failed')
    coordinator._publish_navigation_edge_failure('navigate_failed')
    assert len(coordinator.cognitive_edge_outcome_pub.messages) == 1
    assert coordinator.cognitive_edge_outcome_pub.messages[0].failure is True

    coordinator.tracker = None
    coordinator._publish_navigation_edge_failure('dynamic_or_compute_without_route')
    assert len(coordinator.cognitive_edge_outcome_pub.messages) == 1


def test_lookahead_success_has_no_outcome_but_final_confirmed_does():
    def configured(targets_final):
        coordinator = _feedback_coordinator()
        coordinator.cognitive_graph_mode = 'shadow'
        coordinator.primary_fallback_used = False
        coordinator.cognitive_graph_feedback_active = _feedback()
        coordinator.tracker = SimpleNamespace(
            edge_index=0, edges=[SimpleNamespace(id=1)])
        coordinator.pending_goal = SimpleNamespace(
            pose=SimpleNamespace(position=SimpleNamespace(x=1.0, y=2.0)))
        coordinator.route_goal_completion_tolerance_m = 0.25
        coordinator.navigation_goal_targets_final = targets_final
        coordinator.navigation_goal_pending = True
        coordinator.navigation_goal_handle = object()
        coordinator.navigation_failed = False
        coordinator._current_xy = lambda: (1.0, 2.0)
        coordinator._finish_active_route = lambda: None
        coordinator.goal_complete_pub = _Publisher()
        coordinator.node = SimpleNamespace(get_logger=lambda: SimpleNamespace(
            info=lambda *_args: None, warning=lambda *_args: None))
        return coordinator

    result = SimpleNamespace(result=lambda: SimpleNamespace(
        status=4, result=SimpleNamespace(error_code=0)))
    lookahead = configured(False)
    lookahead._on_navigation_result(result)
    assert lookahead.cognitive_edge_outcome_pub.messages == []

    final = configured(True)
    final._on_navigation_result(result)
    outcomes = final.cognitive_edge_outcome_pub.messages
    assert len(outcomes) == 1
    assert outcomes[0].success is True
    assert outcomes[0].reason == 'final_goal_distance_confirmed'


def test_fallback_requested_and_applied_are_separate_monotonic_events():
    coordinator = _feedback_coordinator()
    coordinator.cognitive_graph_mode = 'primary'
    coordinator.primary_fallback_used = False
    coordinator.cognitive_graph_feedback_active = _feedback()
    coordinator.tracker = SimpleNamespace(
        edge_index=0, edges=[SimpleNamespace(id=1)])
    coordinator._publish_navigation_edge_failure('navigate_failed')
    first = coordinator.cognitive_edge_outcome_pub.messages[0]
    assert first.reroute_applied is False

    coordinator.cognitive_graph_switch_pending = True
    coordinator.graph_generation = 0
    coordinator.pending_goal = None
    coordinator.cognitive_constraints_cache = SimpleNamespace(
        invalidate=lambda: None)
    coordinator._clear_latest_priors = lambda: None
    coordinator._publish_graph = lambda: None
    coordinator._publish_cognitive_constraints = lambda: None
    coordinator._publish_structural_status = lambda *_args: None
    coordinator.StructuralGraphStatus = SimpleNamespace(READY=1)
    fallback_graph = Graph('physical', 4, 'map', 0.05, [], [])
    support = SimpleNamespace(geojson={'features': []})
    coordinator._finish_cognitive_graph_switch(
        SimpleNamespace(result=lambda: SimpleNamespace(success=True)),
        fallback_graph, support, 'fallback', True,
    )

    second = coordinator.cognitive_edge_outcome_pub.messages[1]
    assert second.reroute_applied is True
    assert second.reroute_revision == 1
    assert second.event_sequence > first.event_sequence
