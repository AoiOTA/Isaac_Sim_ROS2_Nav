from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

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
    coordinator._ensure_desired_graph = lambda reason: reconciliations.append(reason)

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
    assert coordinator.requests == [
        ('physical', 'stale SetRouteGraph success', True)]


def test_rebuild_reset_fresh_goal_late_success_does_not_commit_old_graph():
    rebuilt = Graph('rebuilt', 5, 'map:structural', 0.05, [], [])
    coordinator, transaction = _pending_graph_transaction(rebuilt)
    old_map = coordinator.map
    rebuilt_map = _map(wall=True)
    support = SimpleNamespace(geojson={'features': []})
    generation = StructuralRebuildGeneration(
        10, 0, 3, 1, rebuilt.graph_id, rebuilt.revision)
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
    assert coordinator.requests == [
        ('physical', 'stale structural rebuild success', True)]


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
