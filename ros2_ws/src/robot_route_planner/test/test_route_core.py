import json
import math
from pathlib import Path
import threading
from types import SimpleNamespace

import numpy as np
import pytest
import robot_route_planner.ros_node as ros_node_module

from robot_route_planner.cognitive_graph_adapter import CognitiveGraphIdentity
from robot_route_planner.models import Edge, Graph, Node, NodeType, Traversability
from robot_route_planner.map_io import OccupancyMap
from robot_route_planner.ros_node import (
    CostmapSnapshot,
    DEFAULT_ROUTE_ODOMETRY_TOPIC,
    RouteCoordinator,
    edge_prior_is_usable,
    footprint_is_free,
    navigation_result_succeeded,
    parse_reset_stop_gate_status,
    populate_fresh_goal,
    select_live_feasible_lookahead,
    select_map_pose,
    select_support_attachment,
    validate_route_odometry_topic,
)
from robot_route_planner.route_cost import edge_cost_breakdown, shortest_route
from robot_route_planner.route_support import export_route_support_graph
from robot_route_planner.runtime_edges import RuntimeEdgeManager, RuntimeState
from robot_route_planner.stable_ids import stabilize_graph_ids
from robot_route_planner.structural_updates import StructuralChangeMonitor
from robot_route_planner.tracking import RouteTracker


def _edge_prior_message(
    *, request_id: int, stamp_ns: int, model_id: str = 'srdr-v310'
):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(
            sec=stamp_ns // 1_000_000_000,
            nanosec=stamp_ns % 1_000_000_000,
        )),
        request_id=request_id,
        graph_id='test:gvg_v1',
        graph_revision=3,
        model_id=model_id,
        healthy=True,
        priors=[SimpleNamespace(
            edge_id=7,
            cost_delta_m=1.0,
            learned_risk=0.4,
            confidence=0.8,
        )],
    )


def test_edge_prior_requires_fresh_health_model_and_finite_bounds() -> None:
    valid = dict(
        healthy=True,
        model_id="srdr-v310",
        stamp_ns=9_500_000_000,
        now_ns=10_000_000_000,
        max_age_s=2.0,
        priors=[(7, 1.5, 0.4, 0.8)],
    )
    assert edge_prior_is_usable(**valid) == (True, "fresh and healthy")

    for replacement, reason in (
        ({"healthy": False}, "unhealthy"),
        ({"model_id": ""}, "model_id"),
        ({"stamp_ns": 0}, "timestamp"),
        ({"stamp_ns": 10_500_000_000}, "future"),
        ({"stamp_ns": 7_000_000_000}, "stale"),
        ({"priors": [(7, float("nan"), 0.4, 0.8)]}, "non-finite"),
        ({"priors": [(7, -0.1, 0.4, 0.8)]}, "negative"),
        ({"priors": [(7, 1.0, 1.1, 0.8)]}, "learned_risk"),
        ({"priors": [(7, 1.0, 0.4, 1.1)]}, "confidence"),
    ):
        candidate = {**valid, **replacement}
        usable, detail = edge_prior_is_usable(**candidate)
        assert usable is False
        assert reason in detail


def test_prior_timeout_and_ttl_restore_geometry_only_routing() -> None:
    replans = []
    warnings = []
    coordinator = RouteCoordinator.__new__(RouteCoordinator)
    coordinator.pending_goal = object()
    coordinator.pending_deadline_ns = 2_000_000_000
    coordinator.pending_prior_request_id = 4
    coordinator.pending_prior_graph_id = 'test:gvg_v1'
    coordinator.pending_prior_graph_revision = 3
    coordinator.pending_prior_started_ns = 1_000_000_000
    coordinator.pending_prior_model_id = None
    coordinator.request_id = 4
    coordinator.graph = SimpleNamespace(
        graph_id='test:gvg_v1',
        revision=3,
        edges=[SimpleNamespace(id=7)],
    )
    coordinator.latest_priors = {7: (1.0, 0.8)}
    coordinator.latest_priors_stamp_ns = 1_000_000_000
    coordinator.latest_prior_model_id = 'srdr-v310'
    coordinator.module2_prior_ttl_s = 2.0
    coordinator.module2_enabled = False
    coordinator.last_context_publish_ns = 0
    coordinator.defaults = {
        'module2_edge_prior': {
            'active_refresh_period_s': 5.0,
            'response_timeout_s': 0.5,
        },
    }
    coordinator.runtime = SimpleNamespace(tick=lambda _now: False)
    now = SimpleNamespace(nanoseconds=3_100_000_000)
    coordinator._now = lambda: now
    coordinator._prepare_route = lambda priors: replans.append(priors)
    coordinator.node = SimpleNamespace(get_logger=lambda: SimpleNamespace(
        info=lambda message: warnings.append(message),
        warning=lambda message: warnings.append(message),
    ))

    coordinator._check_prior_timeout()
    assert coordinator.pending_deadline_ns is None
    assert coordinator.latest_priors == {}
    assert coordinator.latest_priors_stamp_ns is None
    assert replans == [{}]

    coordinator._on_priors(_edge_prior_message(
        request_id=4, stamp_ns=3_000_000_000))
    assert coordinator.latest_priors == {}
    assert coordinator.latest_priors_stamp_ns is None
    assert replans == [{}]

    coordinator.latest_priors = {7: (1.0, 0.8)}
    coordinator.latest_priors_stamp_ns = 1_000_000_000
    coordinator._runtime_tick()
    assert coordinator.latest_priors == {}
    assert coordinator.latest_priors_stamp_ns is None
    assert replans == [{}, {}]
    assert any('geometry-only' in message for message in warnings)


def _prior_wait_coordinator(*, mode='primary', timeout_s=0.25):
    coordinator = RouteCoordinator.__new__(RouteCoordinator)
    coordinator.cognitive_graph_mode = mode
    coordinator.cognitive_goal_prior_wait_s = 4.0
    coordinator.module2_response_timeout_s = 0.0
    coordinator.module2_prior_ttl_s = 2.0
    coordinator.defaults = {
        'module2_edge_prior': {'response_timeout_s': timeout_s},
    }
    coordinator.request_id = 9
    coordinator.graph = SimpleNamespace(
        graph_id='test:gvg_v1', revision=3,
        edges=[SimpleNamespace(id=7)],
    )
    coordinator.pending_goal = object()
    coordinator.latest_prior_model_id = None
    coordinator.latest_priors = {}
    coordinator.latest_priors_stamp_ns = None
    coordinator.latest_priors_request_id = None
    coordinator.latest_priors_graph_id = None
    coordinator.latest_priors_graph_revision = None
    coordinator.node = SimpleNamespace(get_logger=lambda: SimpleNamespace(
        info=lambda _message: None, warning=lambda _message: None,
    ))
    coordinator._now_ns = 1_000_000_000
    coordinator._now = lambda: SimpleNamespace(
        nanoseconds=coordinator._now_ns)
    coordinator.prepared = []
    coordinator._prepare_route = lambda priors: coordinator.prepared.append(priors)
    coordinator._arm_prior_request(coordinator._now_ns)
    return coordinator


def test_primary_wait_accepts_2p8s_prior_and_identity_zero_falls_back() -> None:
    coordinator = _prior_wait_coordinator()
    assert coordinator.pending_deadline_ns == 5_000_000_000
    coordinator._now_ns = 3_800_000_000
    coordinator._on_priors(_edge_prior_message(
        request_id=9, stamp_ns=3_800_000_000))
    assert coordinator.prepared == [{7: (1.0, 0.8)}]

    zero = _prior_wait_coordinator()
    zero._now_ns = 2_000_000_000
    message = _edge_prior_message(request_id=9, stamp_ns=2_000_000_000)
    message.priors = []
    zero._on_priors(message)
    assert zero.prepared == [{}]


def test_primary_timeout_and_late_prior_are_generation_safe() -> None:
    coordinator = _prior_wait_coordinator()
    coordinator._now_ns = 5_000_000_000
    coordinator._check_prior_timeout()
    assert coordinator.prepared == [{}]
    coordinator._now_ns = 5_100_000_000
    coordinator._on_priors(_edge_prior_message(
        request_id=9, stamp_ns=5_100_000_000))
    assert coordinator.prepared == [{}]


def test_legacy_gvg_prior_timeout_remains_unchanged() -> None:
    coordinator = _prior_wait_coordinator(mode='gvg', timeout_s=0.25)
    assert coordinator.pending_deadline_ns == 1_250_000_000


def test_prior_from_old_refresh_generation_is_rejected() -> None:
    replans = []
    coordinator = RouteCoordinator.__new__(RouteCoordinator)
    coordinator.pending_goal = object()
    coordinator.pending_deadline_ns = 6_500_000_000
    coordinator.pending_prior_request_id = 8
    coordinator.pending_prior_graph_id = 'test:gvg_v1'
    coordinator.pending_prior_graph_revision = 3
    coordinator.pending_prior_started_ns = 5_500_000_000
    coordinator.pending_prior_model_id = 'srdr-v310'
    coordinator.request_id = 8
    coordinator.graph = SimpleNamespace(
        graph_id='test:gvg_v1',
        revision=3,
        edges=[SimpleNamespace(id=7)],
    )
    coordinator.latest_priors = {}
    coordinator.latest_priors_stamp_ns = None
    coordinator.latest_prior_model_id = 'srdr-v310'
    coordinator.module2_prior_ttl_s = 2.0
    coordinator._now = lambda: SimpleNamespace(nanoseconds=6_000_000_000)
    coordinator._prepare_route = lambda priors: replans.append(priors)

    coordinator._on_priors(_edge_prior_message(
        request_id=8, stamp_ns=5_400_000_000))

    assert coordinator.pending_deadline_ns == 6_500_000_000
    assert coordinator.latest_priors == {}
    assert replans == []


def test_matching_prior_for_new_request_is_accepted() -> None:
    replans = []
    coordinator = RouteCoordinator.__new__(RouteCoordinator)
    coordinator.pending_goal = object()
    coordinator.pending_deadline_ns = 6_500_000_000
    coordinator.pending_prior_request_id = 9
    coordinator.pending_prior_graph_id = 'test:gvg_v1'
    coordinator.pending_prior_graph_revision = 3
    coordinator.pending_prior_started_ns = 5_500_000_000
    coordinator.pending_prior_model_id = None
    coordinator.request_id = 9
    coordinator.graph = SimpleNamespace(
        graph_id='test:gvg_v1',
        revision=3,
        edges=[SimpleNamespace(id=7)],
    )
    coordinator.latest_priors = {}
    coordinator.latest_priors_stamp_ns = None
    coordinator.latest_prior_model_id = None
    coordinator.module2_prior_ttl_s = 2.0
    coordinator._now = lambda: SimpleNamespace(nanoseconds=6_000_000_000)
    coordinator._prepare_route = lambda priors: replans.append(priors)
    coordinator.node = SimpleNamespace(get_logger=lambda: SimpleNamespace(
        warning=lambda _message: None,
    ))

    coordinator._on_priors(_edge_prior_message(
        request_id=9, stamp_ns=5_800_000_000))

    assert coordinator.pending_deadline_ns is None
    assert coordinator.latest_priors == {7: (1.0, 0.8)}
    assert coordinator.latest_priors_stamp_ns == 5_800_000_000
    assert coordinator.latest_prior_model_id == 'srdr-v310'
    assert replans == [{7: (1.0, 0.8)}]


def test_active_prior_refresh_arms_a_bounded_response_deadline() -> None:
    coordinator = RouteCoordinator.__new__(RouteCoordinator)
    coordinator.pending_goal = object()
    coordinator.pending_deadline_ns = None
    coordinator.pending_prior_request_id = None
    coordinator.pending_prior_graph_id = None
    coordinator.pending_prior_graph_revision = None
    coordinator.pending_prior_started_ns = None
    coordinator.pending_prior_model_id = None
    coordinator.request_id = 12
    coordinator.graph = SimpleNamespace(graph_id='test:gvg_v1', revision=3)
    coordinator.latest_priors = {}
    coordinator.latest_priors_stamp_ns = None
    coordinator.latest_prior_model_id = None
    coordinator.module2_prior_ttl_s = 2.0
    coordinator.module2_enabled = True
    coordinator.module2_response_timeout_s = 0.0
    coordinator.last_context_publish_ns = 0
    coordinator.defaults = {
        'module2_edge_prior': {
            'active_refresh_period_s': 5.0,
            'response_timeout_s': 0.5,
        },
    }
    coordinator.runtime = SimpleNamespace(tick=lambda _now: False)
    now = SimpleNamespace(nanoseconds=6_000_000_000)
    coordinator._now = lambda: now
    published = []

    def publish_context():
        published.append(True)
        coordinator.last_context_publish_ns = now.nanoseconds

    coordinator._publish_route_context = publish_context
    coordinator._runtime_tick()

    assert published == [True]
    assert coordinator.pending_deadline_ns == 6_500_000_000
    assert coordinator.pending_prior_request_id == 12
    assert coordinator.pending_prior_started_ns == 6_000_000_000


def test_stale_prior_is_removed_at_consumption_between_timer_ticks() -> None:
    warnings = []
    coordinator = RouteCoordinator.__new__(RouteCoordinator)
    coordinator.request_id = 5
    coordinator.graph = SimpleNamespace(graph_id='test:gvg_v1', revision=3)
    coordinator.module2_prior_ttl_s = 2.0
    coordinator.latest_priors = {7: (1.0, 0.8)}
    coordinator.latest_priors_stamp_ns = 1_000_000_000
    coordinator.latest_prior_model_id = 'srdr-v310'
    coordinator.latest_priors_request_id = 5
    coordinator.latest_priors_graph_id = 'test:gvg_v1'
    coordinator.latest_priors_graph_revision = 3
    coordinator._now = lambda: SimpleNamespace(nanoseconds=3_100_000_000)
    coordinator.node = SimpleNamespace(get_logger=lambda: SimpleNamespace(
        warning=lambda message: warnings.append(message),
    ))

    assert coordinator._priors_for_consumption(
        coordinator.latest_priors
    ) == {}
    assert coordinator.latest_priors == {}
    assert any('geometry-only' in message for message in warnings)


def test_old_dynamic_edges_callback_generation_is_discarded() -> None:
    coordinator = RouteCoordinator.__new__(RouteCoordinator)
    coordinator.pending_goal = object()
    coordinator.request_id = 4
    coordinator.graph_generation = 2
    coordinator.graph = SimpleNamespace(graph_id='graph-a', revision=3)
    generation = coordinator._route_callback_generation()
    coordinator.graph_generation = 3
    evaluated = []
    future = SimpleNamespace(result=lambda: evaluated.append(True))

    coordinator._after_edge_update(future, 1, 2, generation)

    assert evaluated == [True]


@pytest.mark.parametrize('failure_kind', ('dynamic', 'route', 'navigation'))
def test_primary_async_failures_request_exactly_one_fallback(failure_kind) -> None:
    coordinator = RouteCoordinator.__new__(RouteCoordinator)
    coordinator.pending_goal = object()
    coordinator.request_id = 6
    coordinator.graph_generation = 2
    coordinator.graph = SimpleNamespace(graph_id='cognitive', revision=5)
    coordinator.cognitive_graph_mode = 'primary'
    coordinator.primary_fallback_used = False
    coordinator.navigation_goal_pending = True
    coordinator.navigation_failed = False
    coordinator.tracker = object()
    coordinator.goal_complete_pub = SimpleNamespace(publish=lambda _message: None)
    coordinator.node = SimpleNamespace(get_logger=lambda: SimpleNamespace(
        warning=lambda _message: None,
    ))
    reasons = []

    def fallback(reason, *_args, **_kwargs):
        coordinator.primary_fallback_used = True
        reasons.append(reason)

    coordinator._fallback_to_gvg_once = fallback
    rejected = SimpleNamespace(result=lambda: SimpleNamespace(
        success=False, accepted=False,
    ))
    for _ in range(2):
        if failure_kind == 'dynamic':
            coordinator._after_edge_update(rejected, 1, 2)
        elif failure_kind == 'route':
            coordinator._on_route_goal_handle(rejected)
        else:
            coordinator._on_navigation_goal_handle(rejected)

    assert len(reasons) == 1


class _CapturePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _AcceptedHandle:
    accepted = True

    def __init__(self):
        self.cancel_calls = 0

    def cancel_goal_async(self):
        self.cancel_calls += 1

    def get_result_async(self):
        return SimpleNamespace(add_done_callback=lambda _callback: None)


def _reset_route_coordinator(*, active=True, handle=None):
    coordinator = RouteCoordinator.__new__(RouteCoordinator)
    coordinator.route_active = active
    coordinator.pending_goal = object() if active else None
    coordinator.request_id = 41
    coordinator.graph_generation = 3
    coordinator.graph_switch_generation = 7
    coordinator.reset_status_generation = 1
    coordinator.reset_status_snapshot = parse_reset_stop_gate_status(
        '{"eligible_generation":null,"generation":1,"held":false,'
        '"reason":"released:activation_gate"}'
    )
    coordinator.reset_intent_generation = None
    coordinator.reset_event_completed_generation = None
    # The event-driven tests below exercise the gate-less legacy reset path;
    # the status handler flips this flag on the first strict status anyway.
    coordinator.reset_status_authority_seen = False
    coordinator.reset_hold_barrier = False
    coordinator.reset_ready_pending = False
    coordinator.graph = SimpleNamespace(graph_id='physical', revision=4)
    coordinator.gvg_graph = coordinator.graph
    coordinator.cognitive_graph_identity = CognitiveGraphIdentity(
        8, 'old-session', 'map-v1', 'old-tile', 3, 'physical', 4, 'model')
    coordinator.cognitive_graph_switch_pending = True
    coordinator.cognitive_graph_last_sequence = 9
    coordinator.cognitive_graph_feedback_active = object()
    coordinator.cognitive_graph_feedback_pending = object()
    coordinator.cognitive_feedback_sequences = {(4, 'outcome', 'edge'): 2}
    coordinator.cognitive_validation_terminal = {(4, 'edge')}
    coordinator.cognitive_outcome_terminal = {(4, '1')}
    coordinator.pending_reroute_outcome = object()
    coordinator.cognitive_reroute_revision = 2
    coordinator.primary_fallback_used = True
    coordinator.tracker = object()
    coordinator.navigation_goal_pending = handle is None and active
    coordinator.navigation_goal_handle = handle
    coordinator.navigation_goal_targets_final = True
    coordinator.navigation_failed = True
    coordinator.pending_deadline_ns = 2
    coordinator.pending_prior_request_id = 41
    coordinator.pending_prior_graph_id = 'physical'
    coordinator.pending_prior_graph_revision = 4
    coordinator.pending_prior_started_ns = 1
    coordinator.pending_prior_model_id = 'model'
    coordinator.latest_priors = {1: (1.0, 0.5)}
    coordinator.latest_priors_stamp_ns = 1
    coordinator.latest_prior_model_id = 'model'
    coordinator.latest_priors_request_id = 41
    coordinator.latest_priors_graph_id = 'physical'
    coordinator.latest_priors_graph_revision = 4
    coordinator.latest_pose_xy = (1.0, 2.0)
    coordinator.latest_pose_frame_id = 'map'
    coordinator.latest_pose_stamp_ns = 1
    coordinator.latest_global_costmap = object()
    coordinator.last_context_publish_ns = 1
    coordinator.runtime = SimpleNamespace(edges={1: object()})
    coordinator.pending_structural_map = object()
    coordinator.structural_monitor = SimpleNamespace(
        last_candidate=object(), first_stable_s=1.0, stable_count=3)
    coordinator.cognitive_constraints_cache = SimpleNamespace(
        invalidations=0)

    def invalidate():
        coordinator.cognitive_constraints_cache.invalidations += 1

    coordinator.cognitive_constraints_cache.invalidate = invalidate
    coordinator.region_selector = SimpleNamespace(
        current=object(), last_switch_s=1.0)
    coordinator.tf_buffer = SimpleNamespace(clear_calls=0)

    def clear_tf():
        coordinator.tf_buffer.clear_calls += 1

    coordinator.tf_buffer.clear = clear_tf
    coordinator.goal_complete_pub = _CapturePublisher()
    coordinator.goal_result_pub = _CapturePublisher()
    coordinator.context_pub = _CapturePublisher()
    coordinator.progress_pub = _CapturePublisher()
    coordinator.lookahead_pub = _CapturePublisher()
    coordinator.goal_update_pub = _CapturePublisher()
    coordinator.route_pub = _CapturePublisher()
    coordinator.runtime_snapshots = []
    coordinator._publish_runtime_states = lambda *, graph=None: (
        coordinator.runtime_snapshots.append((
            graph.graph_id, graph.revision, list(coordinator.runtime.edges)
        ))
    )
    coordinator.graph_reconciliations = []
    coordinator._ensure_desired_graph = lambda reason, **_kwargs: (
        coordinator.graph_reconciliations.append(reason)
    )
    coordinator.node = SimpleNamespace(get_logger=lambda: SimpleNamespace(
        info=lambda _message: None,
        warning=lambda _message: None,
        error=lambda _message: None,
    ))
    coordinator.StructuralGraphStatus = SimpleNamespace(
        LAST_KNOWN_GOOD=2, READY=1)
    coordinator.structural_statuses = []
    coordinator._publish_structural_status = lambda state, detail: (
        coordinator.structural_statuses.append((state, detail))
    )
    coordinator._fallback_to_gvg_once = lambda _reason: pytest.fail(
        'physical graph reset must not request fallback')
    return coordinator


def _gate_status(generation, held, reason, eligible=None):
    return SimpleNamespace(data=json.dumps({
        'generation': generation,
        'held': held,
        'eligible_generation': eligible,
        'reason': reason,
    }))


def _startup_reset_route_coordinator():
    coordinator = _reset_route_coordinator(active=False)
    coordinator.request_id = 0
    coordinator.graph_generation = 0
    coordinator.graph_switch_generation = 0
    coordinator.reset_generation = 0
    coordinator.structural_generation = 0
    coordinator.desired_graph_generation = 0
    coordinator.reset_status_generation = None
    coordinator.reset_status_snapshot = None
    coordinator.reset_intent_generation = None
    coordinator.reset_event_completed_generation = None
    coordinator.reset_status_authority_seen = False
    coordinator.reset_hold_barrier = True
    coordinator.reset_ready_pending = False
    coordinator.tracker = None
    coordinator.navigation_goal_pending = False
    coordinator.navigation_goal_handle = None
    coordinator.navigation_goal_targets_final = False
    coordinator.navigation_failed = False
    coordinator.pending_deadline_ns = None
    coordinator.pending_prior_request_id = None
    coordinator.latest_priors = {}
    coordinator.latest_priors_stamp_ns = None
    coordinator.latest_priors_request_id = None
    coordinator.cognitive_graph_last_sequence = 0
    coordinator.cognitive_graph_feedback_active = None
    coordinator.cognitive_graph_feedback_pending = None
    coordinator.pending_reroute_outcome = None
    coordinator.cognitive_graph_switch_pending = False
    coordinator.graph_transaction_generation = None
    coordinator.graph_transaction_future = None
    coordinator.graph_transaction_deadline_steady_s = None
    coordinator.graph_transaction_kind = None
    coordinator.graph_retry_key = None
    coordinator.graph_retry_due_steady_s = None
    coordinator.pending_structural_map = None
    coordinator.pending_structural_intent = None
    coordinator.structural_candidate_generation = 0
    coordinator.structural_observation_generation = 0
    coordinator.runtime.edges = {}
    coordinator.latest_pose_xy = None
    coordinator.latest_global_costmap = None
    coordinator.desired_graph = coordinator.gvg_graph
    coordinator.desired_support = None
    coordinator.graph_coherent = True
    coordinator.graph_reassert_required = False
    return coordinator


def test_reset_stop_gate_status_parser_rejects_malformed_or_incoherent_state() -> None:
    valid = parse_reset_stop_gate_status(json.dumps({
        'generation': 4,
        'held': True,
        'eligible_generation': None,
        'reason': 'hold',
    }))
    assert valid.generation == 4
    assert valid.held is True

    for payload in (
        'not-json',
        '[]',
        '{"generation":true,"held":true,"reason":"hold"}',
        '{"generation":4,"held":false,"reason":"hold"}',
        '{"generation":4,"held":true,"eligible_generation":3,'
        '"reason":"reset_complete"}',
        '{"generation":4,"held":false,"eligible_generation":4,'
        '"reason":"released:activation_gate"}',
    ):
        with pytest.raises(ValueError):
            parse_reset_stop_gate_status(payload)


def test_hold_retires_active_route_before_event_and_same_generation_is_idempotent() -> None:
    handle = _AcceptedHandle()
    coordinator = _reset_route_coordinator(handle=handle)
    hold = _gate_status(2, True, 'hold')

    coordinator._on_reset_stop_gate_status(hold)

    assert coordinator.reset_hold_barrier is True
    assert coordinator.reset_intent_generation == 2
    assert coordinator.request_id == 42
    assert coordinator.cognitive_graph_identity.reset_epoch == 9
    assert handle.cancel_calls == 1
    assert [message.data for message in coordinator.goal_complete_pub.messages] == [False]
    assert json.loads(coordinator.goal_result_pub.messages[0].data) == {
        'request_id': 41,
        'status': 'aborted',
        'reason': 'simulation_reset',
        'reset_epoch': 9,
    }
    assert coordinator.runtime_snapshots == []
    assert coordinator.graph_reconciliations == []

    coordinator._on_reset_stop_gate_status(hold)
    coordinator._on_reset_event(None)
    coordinator._on_reset_event(None)
    coordinator._on_reset_stop_gate_status(
        _gate_status(2, True, 'reset_complete', eligible=2))
    assert coordinator.reset_hold_barrier is True
    assert coordinator.request_id == 42
    assert len(coordinator.goal_result_pub.messages) == 1
    assert coordinator.runtime_snapshots == [('physical', 4, [])]
    assert coordinator.graph_reconciliations == [
        'simulation reset requires Route Server GVG']

    coordinator._on_reset_stop_gate_status(
        _gate_status(2, False, 'released:activation_gate'))
    coordinator._on_reset_stop_gate_status(
        _gate_status(2, False, 'released:activation_gate'))
    assert coordinator.reset_hold_barrier is False
    assert len(coordinator.goal_result_pub.messages) == 1

    coordinator._on_reset_event(None)
    assert coordinator.request_id == 42
    assert coordinator.runtime_snapshots == [('physical', 4, [])]

    coordinator.graph_coherent = True
    coordinator.graph_reassert_required = False
    coordinator.module2_enabled = False
    prepared = []
    coordinator._prepare_route = lambda priors: prepared.append(priors)
    coordinator._publish_route_context = lambda: None
    fresh_goal = SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(
        x=5.0, y=6.0)))
    coordinator._on_goal(fresh_goal)
    assert coordinator.pending_goal is fresh_goal
    assert prepared == [{}]


def test_release_publishes_deferred_ready_once_after_reassert_commit_in_hold() -> None:
    coordinator = _reset_route_coordinator(active=False)
    coordinator._on_reset_stop_gate_status(_gate_status(2, True, 'hold'))
    coordinator._on_reset_stop_gate_status(
        _gate_status(2, True, 'reset_complete', eligible=2))

    # The completion-owned GVG reassert committed while HOLD fenced outputs.
    coordinator.graph_coherent = True
    coordinator.graph_reassert_required = False
    coordinator.reset_ready_pending = True

    released = _gate_status(2, False, 'released:activation_gate')
    coordinator._on_reset_stop_gate_status(released)
    coordinator._on_reset_stop_gate_status(released)

    assert coordinator.reset_hold_barrier is False
    assert coordinator.structural_statuses == [
        (coordinator.StructuralGraphStatus.READY, 'reset GVG reconciled')
    ]


def test_release_without_pending_reassert_commit_publishes_no_ready() -> None:
    coordinator = _reset_route_coordinator(active=False)
    coordinator._on_reset_stop_gate_status(_gate_status(2, True, 'hold'))
    coordinator._on_reset_stop_gate_status(
        _gate_status(2, True, 'reset_complete', eligible=2))
    coordinator._on_reset_stop_gate_status(
        _gate_status(2, False, 'released:activation_gate'))

    assert coordinator.reset_hold_barrier is False
    assert coordinator.structural_statuses == []


def _two_by_two_map_and_grid():
    coordinator_map = SimpleNamespace(
        free=np.zeros((2, 2), dtype=np.uint8),
        resolution_m=1.0,
        origin_xy_m=(0.0, 0.0),
    )
    grid = SimpleNamespace(
        info=SimpleNamespace(
            width=2,
            height=2,
            resolution=1.0,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0)),
        ),
        data=[0, 0, 0, 0],
    )
    return coordinator_map, grid


def test_release_replays_map_binding_deferred_during_hold() -> None:
    coordinator = _reset_route_coordinator(active=False)
    coordinator.map, grid = _two_by_two_map_and_grid()
    coordinator.live_map_version = None
    constraints_publications = []
    coordinator._publish_cognitive_constraints = lambda: (
        constraints_publications.append('constraints')
    )

    coordinator._on_reset_stop_gate_status(_gate_status(2, True, 'hold'))
    coordinator._on_occupancy_map(grid)
    assert coordinator.live_map_version is None
    assert constraints_publications == []

    coordinator._on_reset_stop_gate_status(
        _gate_status(2, True, 'reset_complete', eligible=2))
    coordinator._on_reset_stop_gate_status(
        _gate_status(2, False, 'released:activation_gate'))

    assert coordinator.reset_hold_barrier is False
    assert coordinator.live_map_version is not None
    assert constraints_publications == ['constraints']


def test_release_re_publishes_constraints_fenced_during_hold() -> None:
    coordinator = _reset_route_coordinator(active=False)
    constraints_publications = []
    coordinator._publish_cognitive_constraints = lambda: (
        constraints_publications.append('constraints')
    )

    coordinator._on_reset_stop_gate_status(_gate_status(2, True, 'hold'))
    coordinator._on_reset_stop_gate_status(
        _gate_status(2, True, 'reset_complete', eligible=2))
    assert constraints_publications == []

    coordinator._on_reset_stop_gate_status(
        _gate_status(2, False, 'released:activation_gate'))
    assert coordinator.reset_hold_barrier is False
    assert constraints_publications == ['constraints']

    # An exact duplicate release stays idempotent.
    coordinator._on_reset_stop_gate_status(
        coordinator.reset_status_snapshot)
    assert constraints_publications == ['constraints']


def test_startup_released_baseline_also_publishes_constraints() -> None:
    coordinator = _startup_reset_route_coordinator()
    constraints_publications = []
    coordinator._publish_cognitive_constraints = lambda: (
        constraints_publications.append('constraints')
    )

    coordinator._on_reset_stop_gate_status(
        _gate_status(7, False, 'released:activation_gate'))

    assert coordinator.reset_hold_barrier is False
    assert constraints_publications == ['constraints']


def test_gateless_reset_event_open_also_publishes_constraints() -> None:
    coordinator = _reset_route_coordinator(active=False)
    constraints_publications = []
    coordinator._publish_cognitive_constraints = lambda: (
        constraints_publications.append('constraints')
    )

    coordinator._on_reset_event(SimpleNamespace(data=''))

    assert coordinator.reset_hold_barrier is False
    assert constraints_publications == ['constraints']


def test_startup_released_baseline_synchronizes_without_fake_terminal() -> None:
    coordinator = _reset_route_coordinator(active=False)
    coordinator.reset_status_generation = None
    coordinator.reset_status_snapshot = None
    coordinator.reset_hold_barrier = True

    coordinator._on_reset_stop_gate_status(
        _gate_status(7, False, 'released:activation_gate'))

    assert coordinator.reset_status_generation == 7
    assert coordinator.reset_hold_barrier is False
    assert coordinator.goal_complete_pub.messages == []
    assert coordinator.goal_result_pub.messages == []
    assert coordinator.request_id == 41


def test_startup_reset_complete_baseline_reconciles_then_release_accepts_goal() -> None:
    coordinator = _startup_reset_route_coordinator()

    coordinator._on_reset_stop_gate_status(
        _gate_status(7, True, 'reset_complete', eligible=7))

    assert coordinator.reset_status_generation == 7
    assert coordinator.reset_intent_generation == 7
    assert coordinator.reset_event_completed_generation == 7
    assert coordinator.reset_hold_barrier is True
    assert coordinator.reset_generation == 1
    assert coordinator.request_id == 1
    assert coordinator.runtime_snapshots == [('physical', 4, [])]
    assert coordinator.graph_reconciliations == [
        'simulation reset requires Route Server GVG']
    assert coordinator.goal_complete_pub.messages == []
    assert coordinator.goal_result_pub.messages == []

    coordinator._on_reset_stop_gate_status(
        _gate_status(7, False, 'released:activation_gate'))
    assert coordinator.reset_hold_barrier is False

    coordinator.graph_coherent = True
    coordinator.graph_reassert_required = False
    coordinator.module2_enabled = False
    prepared = []
    coordinator._prepare_route = lambda priors: prepared.append(priors)
    coordinator._publish_route_context = lambda: None
    fresh_goal = SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(
        x=5.0, y=6.0)))
    coordinator._on_goal(fresh_goal)

    assert coordinator.pending_goal is fresh_goal
    assert prepared == [{}]
    assert coordinator.goal_complete_pub.messages == []
    assert coordinator.goal_result_pub.messages == []


def test_startup_reset_complete_duplicate_is_idempotent_and_bad_followups_hold() -> None:
    coordinator = _startup_reset_route_coordinator()
    complete = _gate_status(7, True, 'reset_complete', eligible=7)

    coordinator._on_reset_stop_gate_status(complete)
    coordinator._on_reset_stop_gate_status(complete)

    assert coordinator.reset_generation == 1
    assert coordinator.request_id == 1
    assert coordinator.runtime_snapshots == [('physical', 4, [])]
    assert coordinator.goal_complete_pub.messages == []
    assert coordinator.goal_result_pub.messages == []

    coordinator._on_reset_stop_gate_status(
        _gate_status(6, False, 'released:activation_gate'))
    coordinator._on_reset_stop_gate_status(_gate_status(7, True, 'hold'))

    assert coordinator.reset_status_generation == 7
    assert coordinator.reset_hold_barrier is True
    assert coordinator.reset_generation == 1
    assert coordinator.goal_complete_pub.messages == []
    assert coordinator.goal_result_pub.messages == []


def test_first_reset_complete_with_active_route_retires_once_and_stays_held() -> None:
    handle = _AcceptedHandle()
    coordinator = _reset_route_coordinator(active=True, handle=handle)
    coordinator.reset_status_generation = None
    coordinator.reset_status_snapshot = None
    coordinator.reset_hold_barrier = True

    coordinator._on_reset_stop_gate_status(
        _gate_status(7, True, 'reset_complete', eligible=7))

    # The completed generation is authoritative reset evidence: the
    # interrupted route is retired with exactly one abort terminal and the
    # barrier stays held until the strict release arrives.
    assert coordinator.reset_status_generation == 7
    assert coordinator.reset_intent_generation == 7
    assert coordinator.reset_event_completed_generation == 7
    assert coordinator.reset_hold_barrier is True
    assert coordinator.request_id == 42
    assert handle.cancel_calls == 1
    assert [
        message.data for message in coordinator.goal_complete_pub.messages
    ] == [False]
    assert json.loads(coordinator.goal_result_pub.messages[0].data) == {
        'request_id': 41,
        'status': 'aborted',
        'reason': 'simulation_reset',
        'reset_epoch': 9,
    }
    assert coordinator.runtime_snapshots == [('physical', 4, [])]
    assert coordinator.graph_reconciliations == [
        'simulation reset requires Route Server GVG']

    coordinator._on_reset_event(None)  # duplicate of the completed generation
    assert len(coordinator.goal_result_pub.messages) == 1

    coordinator._on_reset_stop_gate_status(
        _gate_status(7, False, 'released:activation_gate'))
    assert coordinator.reset_hold_barrier is False
    assert len(coordinator.goal_result_pub.messages) == 1


def test_new_generation_completion_without_hold_retires_and_stays_held() -> None:
    coordinator = _startup_reset_route_coordinator()
    coordinator._on_reset_stop_gate_status(
        _gate_status(7, False, 'released:activation_gate'))

    coordinator._on_reset_stop_gate_status(
        _gate_status(8, True, 'reset_complete', eligible=8))

    # A newer completed generation is authoritative even when its HOLD was
    # missed: retire once, publish completion, and keep the barrier held.
    assert coordinator.reset_status_generation == 8
    assert coordinator.reset_intent_generation == 8
    assert coordinator.reset_event_completed_generation == 8
    assert coordinator.reset_hold_barrier is True
    assert coordinator.reset_generation == 1
    assert coordinator.runtime_snapshots == [('physical', 4, [])]
    assert coordinator.goal_complete_pub.messages == []

    coordinator._on_reset_stop_gate_status(
        _gate_status(8, False, 'released:activation_gate'))
    assert coordinator.reset_hold_barrier is False


def test_startup_completion_reconciliation_precedes_concurrent_release() -> None:
    coordinator = _startup_reset_route_coordinator()
    reconciliation_entered = threading.Event()
    allow_reconciliation = threading.Event()
    release_returned = threading.Event()

    def reconcile(_reason, **_kwargs):
        reconciliation_entered.set()
        assert allow_reconciliation.wait(timeout=2.0)

    coordinator._ensure_desired_graph = reconcile
    completion_thread = threading.Thread(
        target=coordinator._on_reset_stop_gate_status,
        args=(_gate_status(7, True, 'reset_complete', eligible=7),),
    )
    release_thread = threading.Thread(
        target=lambda: (
            coordinator._on_reset_stop_gate_status(
                _gate_status(7, False, 'released:activation_gate')),
            release_returned.set(),
        ),
    )

    completion_thread.start()
    assert reconciliation_entered.wait(timeout=2.0)
    release_thread.start()
    assert not release_returned.wait(timeout=0.05)
    allow_reconciliation.set()
    completion_thread.join(timeout=2.0)
    release_thread.join(timeout=2.0)

    assert not completion_thread.is_alive()
    assert not release_thread.is_alive()
    assert coordinator.reset_event_completed_generation == 7
    assert coordinator.reset_hold_barrier is False
    assert coordinator.goal_complete_pub.messages == []
    assert coordinator.goal_result_pub.messages == []


def test_hold_completion_release_does_not_depend_on_empty_event() -> None:
    coordinator = _reset_route_coordinator(active=False)

    coordinator._on_reset_stop_gate_status(_gate_status(2, True, 'hold'))
    coordinator._on_reset_stop_gate_status(
        _gate_status(2, True, 'reset_complete', eligible=2))
    coordinator._on_reset_stop_gate_status(
        _gate_status(2, False, 'released:activation_gate'))

    assert coordinator.reset_intent_generation == 2
    assert coordinator.reset_event_completed_generation == 2
    assert coordinator.reset_hold_barrier is False
    assert coordinator.reset_generation == 1
    assert coordinator.request_id == 42
    assert coordinator.runtime_snapshots == [('physical', 4, [])]
    assert coordinator.graph_reconciliations == [
        'simulation reset requires Route Server GVG']


def test_hold_event_completion_release_publishes_baseline_once() -> None:
    coordinator = _reset_route_coordinator(active=False)

    coordinator._on_reset_stop_gate_status(_gate_status(2, True, 'hold'))
    coordinator._on_reset_event(None)
    coordinator._on_reset_stop_gate_status(
        _gate_status(2, True, 'reset_complete', eligible=2))
    coordinator._on_reset_stop_gate_status(
        _gate_status(2, False, 'released:activation_gate'))

    assert coordinator.reset_hold_barrier is False
    assert coordinator.reset_generation == 1
    assert coordinator.runtime_snapshots == [('physical', 4, [])]
    assert coordinator.graph_reconciliations == [
        'simulation reset requires Route Server GVG']


@pytest.mark.parametrize('final_reason', ('reset_complete', 'released'))
def test_event_before_first_status_re_fences_on_late_status(final_reason) -> None:
    coordinator = _reset_route_coordinator(active=False)
    coordinator.reset_status_generation = None
    coordinator.reset_status_snapshot = None
    coordinator.reset_hold_barrier = True
    status = (
        _gate_status(7, True, 'reset_complete', eligible=7)
        if final_reason == 'reset_complete'
        else _gate_status(7, False, 'released:activation_gate')
    )

    coordinator._on_reset_event(None)

    # No gate authority was ever seen: the Empty alone fences and completes
    # the epoch (gate-less legacy path), and the barrier re-opens.
    assert coordinator.reset_generation == 1
    assert coordinator.request_id == 42
    assert coordinator.reset_hold_barrier is False
    assert coordinator.runtime_snapshots == [('physical', 4, [])]
    assert coordinator.goal_complete_pub.messages == []

    coordinator._on_reset_stop_gate_status(status)

    if final_reason == 'reset_complete':
        # The late strict completion re-fences and retires once more; the
        # second retire has no route left and publishes no terminal.
        assert coordinator.reset_intent_generation == 7
        assert coordinator.reset_event_completed_generation == 7
        assert coordinator.reset_hold_barrier is True
        assert coordinator.reset_generation == 2
        assert coordinator.request_id == 43
    else:
        # A released baseline after the legacy event leaves the barrier open.
        assert coordinator.reset_status_generation == 7
        assert coordinator.reset_hold_barrier is False
        assert coordinator.reset_generation == 1
    assert coordinator.goal_result_pub.messages == []


def test_first_completion_then_event_and_release_synchronize_one_generation() -> None:
    coordinator = _reset_route_coordinator(active=False)
    coordinator.reset_status_generation = None
    coordinator.reset_status_snapshot = None
    coordinator.reset_hold_barrier = True

    coordinator._on_reset_stop_gate_status(
        _gate_status(7, True, 'reset_complete', eligible=7))
    coordinator._on_reset_event(None)
    coordinator._on_reset_stop_gate_status(
        _gate_status(7, False, 'released:activation_gate'))

    assert coordinator.reset_intent_generation == 7
    assert coordinator.reset_event_completed_generation == 7
    assert coordinator.reset_generation == 1
    assert coordinator.request_id == 42
    assert coordinator.reset_hold_barrier is False
    assert coordinator.runtime_snapshots == [('physical', 4, [])]
    assert coordinator.goal_result_pub.messages == []


def test_hold_without_event_stays_fail_closed_and_rejects_new_goal() -> None:
    coordinator = _reset_route_coordinator()
    coordinator._on_reset_stop_gate_status(_gate_status(2, True, 'hold'))
    request_id = coordinator.request_id
    goal = SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(
        x=5.0, y=6.0)))

    coordinator._on_goal(goal)
    coordinator._on_reset_stop_gate_status(
        _gate_status(2, False, 'released:activation_gate'))

    assert coordinator.reset_hold_barrier is True
    assert coordinator.request_id == request_id
    assert coordinator.pending_goal is None
    assert len(coordinator.goal_result_pub.messages) == 1


def test_bad_or_backward_gate_status_cannot_open_newer_hold() -> None:
    coordinator = _reset_route_coordinator(active=False)
    coordinator._on_reset_stop_gate_status(_gate_status(2, True, 'hold'))
    coordinator._on_reset_stop_gate_status(SimpleNamespace(data='not-json'))
    coordinator._on_reset_stop_gate_status(
        _gate_status(1, False, 'released:activation_gate'))

    assert coordinator.reset_status_generation == 2
    assert coordinator.reset_hold_barrier is True
    assert coordinator.goal_complete_pub.messages == []
    assert coordinator.goal_result_pub.messages == []


def test_hold_fences_late_acceptance_and_navigation_result() -> None:
    coordinator = _reset_route_coordinator()
    generation = coordinator._route_callback_generation()
    coordinator._on_reset_stop_gate_status(_gate_status(2, True, 'hold'))
    late_handle = _AcceptedHandle()

    coordinator._on_navigation_goal_handle(
        SimpleNamespace(result=lambda: late_handle), generation)
    coordinator._on_navigation_result(
        SimpleNamespace(result=lambda: SimpleNamespace(
            status=5, result=SimpleNamespace(error_code=207))),
        generation,
    )

    assert late_handle.cancel_calls == 1
    assert [message.data for message in coordinator.goal_complete_pub.messages] == [False]
    assert len(coordinator.goal_result_pub.messages) == 1


def test_navigation_terminal_before_hold_is_not_followed_by_reset_abort() -> None:
    coordinator = _reset_route_coordinator(handle=_AcceptedHandle())
    coordinator.pending_structural_map = None
    coordinator.cognitive_graph_feedback_active = None
    coordinator.cognitive_graph_mode = 'gvg'
    generation = coordinator._route_callback_generation()

    coordinator._on_navigation_result(
        SimpleNamespace(result=lambda: SimpleNamespace(
            status=5, result=SimpleNamespace(error_code=207))),
        generation,
    )
    coordinator._on_reset_stop_gate_status(_gate_status(2, True, 'hold'))

    assert [message.data for message in coordinator.goal_complete_pub.messages] == [False]
    assert [json.loads(message.data) for message in coordinator.goal_result_pub.messages] == [{
        'request_id': 41,
        'status': 'failed',
        'reason': 'navigate_to_pose_failed_error_207',
        'reset_epoch': 8,
    }]


def test_active_reset_retires_state_cancels_once_and_publishes_one_terminal() -> None:
    handle = _AcceptedHandle()
    coordinator = _reset_route_coordinator(handle=handle)

    coordinator._on_reset_event(None)

    assert coordinator.request_id == 42
    assert coordinator.graph_generation == 4
    assert coordinator.graph_switch_generation == 8
    assert coordinator.route_active is False
    assert coordinator.pending_goal is None
    assert coordinator.tracker is None
    assert coordinator.navigation_goal_pending is False
    assert coordinator.navigation_goal_handle is None
    assert coordinator.navigation_goal_targets_final is False
    assert coordinator.navigation_failed is False
    assert coordinator.pending_deadline_ns is None
    assert coordinator.latest_priors == {}
    assert coordinator.pending_reroute_outcome is None
    assert coordinator.runtime.edges == {}
    assert coordinator.latest_pose_xy is None
    assert coordinator.latest_pose_frame_id is None
    assert coordinator.latest_pose_stamp_ns is None
    assert coordinator.latest_global_costmap is None
    assert coordinator.pending_structural_map is None
    assert coordinator.structural_monitor.last_candidate is None
    assert coordinator.cognitive_constraints_cache.invalidations == 1
    assert coordinator.region_selector.current is None
    assert coordinator.tf_buffer.clear_calls == 1
    assert coordinator.runtime_snapshots == [('physical', 4, [])]
    assert coordinator.graph_reconciliations == [
        'simulation reset requires Route Server GVG']
    assert handle.cancel_calls == 1
    assert [message.data for message in coordinator.goal_complete_pub.messages] == [False]
    assert len(coordinator.goal_result_pub.messages) == 1
    assert json.loads(coordinator.goal_result_pub.messages[0].data) == {
        'request_id': 41,
        'status': 'aborted',
        'reason': 'simulation_reset',
        'reset_epoch': 9,
    }

    coordinator._on_reset_event(None)
    assert handle.cancel_calls == 1
    assert len(coordinator.goal_complete_pub.messages) == 1
    assert len(coordinator.goal_result_pub.messages) == 1
    assert coordinator.runtime_snapshots == [
        ('physical', 4, []), ('physical', 4, [])]


def test_reset_pending_action_late_accept_is_cancelled_and_result_is_ignored() -> None:
    coordinator = _reset_route_coordinator()
    generation = coordinator._route_callback_generation()
    coordinator._on_reset_event(None)
    late_handle = _AcceptedHandle()
    handle_result_calls = []
    coordinator._on_navigation_goal_handle(
        SimpleNamespace(result=lambda: handle_result_calls.append(True) or late_handle),
        generation,
    )

    assert handle_result_calls == [True]
    assert late_handle.cancel_calls == 1
    assert coordinator.navigation_goal_handle is None
    terminal_count = len(coordinator.goal_complete_pub.messages)
    result_calls = []
    coordinator._on_navigation_result(
        SimpleNamespace(result=lambda: result_calls.append(True)), generation)
    assert result_calls == [True]
    assert len(coordinator.goal_complete_pub.messages) == terminal_count
    assert len(coordinator.goal_result_pub.messages) == 1


def test_reset_waits_for_navigation_handle_check_and_cancels_committed_handle() -> None:
    coordinator = _reset_route_coordinator()
    generation = coordinator._route_callback_generation()
    handle = _AcceptedHandle()
    check_entered = threading.Barrier(2)
    release_check = threading.Event()
    original_check = coordinator._route_callback_is_current

    def blocked_check(token):
        check_entered.wait(timeout=2.0)
        assert release_check.wait(timeout=2.0)
        return original_check(token)

    coordinator._route_callback_is_current = blocked_check
    callback_thread = threading.Thread(
        target=coordinator._on_navigation_goal_handle,
        args=(SimpleNamespace(result=lambda: handle), generation),
    )
    callback_thread.start()
    check_entered.wait(timeout=2.0)
    reset_thread = threading.Thread(target=coordinator._on_reset_event, args=(None,))
    reset_thread.start()
    reset_thread.join(timeout=0.05)
    assert reset_thread.is_alive()
    release_check.set()
    callback_thread.join(timeout=2.0)
    reset_thread.join(timeout=2.0)

    assert not callback_thread.is_alive()
    assert not reset_thread.is_alive()
    assert handle.cancel_calls == 1
    assert coordinator.navigation_goal_handle is None
    assert [message.data for message in coordinator.goal_complete_pub.messages] == [False]


def test_reset_terminal_cannot_be_overtaken_by_old_navigation_rejection() -> None:
    coordinator = _reset_route_coordinator()
    coordinator.pending_structural_map = None
    coordinator.cognitive_graph_feedback_active = None
    generation = coordinator._route_callback_generation()
    events = []
    old_publish_entered = threading.Event()
    release_old_publish = threading.Event()
    reset_state_retired = threading.Event()

    class OrderedCompletePublisher:
        def publish(self, _message):
            name = threading.current_thread().name
            events.append((name, 'complete'))
            if name == 'old-navigation':
                old_publish_entered.set()
                assert release_old_publish.wait(timeout=2.0)

    class OrderedResultPublisher:
        def publish(self, _message):
            events.append((threading.current_thread().name, 'result'))

    coordinator.goal_complete_pub = OrderedCompletePublisher()
    coordinator.goal_result_pub = OrderedResultPublisher()
    original_retire = coordinator._retire_active_route_for_reset

    def observed_retire():
        result = original_retire()
        reset_state_retired.set()
        return result

    coordinator._retire_active_route_for_reset = observed_retire
    callback_thread = threading.Thread(
        name='old-navigation',
        target=coordinator._on_navigation_goal_handle,
        args=(SimpleNamespace(result=lambda: SimpleNamespace(accepted=False)), generation),
    )
    callback_thread.start()
    assert old_publish_entered.wait(timeout=2.0)
    reset_thread = threading.Thread(
        name='reset',
        target=coordinator._on_reset_stop_gate_status,
        args=(_gate_status(2, True, 'hold'),),
    )
    reset_thread.start()
    reset_thread.join(timeout=0.05)
    assert reset_thread.is_alive()
    release_old_publish.set()
    callback_thread.join(timeout=2.0)
    reset_thread.join(timeout=2.0)

    assert not callback_thread.is_alive()
    assert not reset_thread.is_alive()
    assert events == [
        ('old-navigation', 'complete'),
        ('old-navigation', 'result'),
    ]
    assert reset_state_retired.is_set()


def test_reset_wins_terminal_race_and_late_rejection_is_silent() -> None:
    coordinator = _reset_route_coordinator()
    coordinator.cognitive_graph_feedback_active = None
    generation = coordinator._route_callback_generation()
    events = []
    reset_retired = threading.Event()
    release_reset = threading.Event()
    original_retire = coordinator._retire_active_route_for_reset

    class OrderedPublisher:
        def __init__(self, kind):
            self.kind = kind

        def publish(self, _message):
            events.append((threading.current_thread().name, self.kind))

    coordinator.goal_complete_pub = OrderedPublisher('complete')
    coordinator.goal_result_pub = OrderedPublisher('result')

    def blocked_retire():
        result = original_retire()
        reset_retired.set()
        assert release_reset.wait(timeout=2.0)
        return result

    coordinator._retire_active_route_for_reset = blocked_retire
    reset_thread = threading.Thread(
        name='reset',
        target=coordinator._on_reset_stop_gate_status,
        args=(_gate_status(2, True, 'hold'),),
    )
    reset_thread.start()
    assert reset_retired.wait(timeout=2.0)
    callback_thread = threading.Thread(
        name='old-navigation',
        target=coordinator._on_navigation_goal_handle,
        args=(SimpleNamespace(result=lambda: SimpleNamespace(accepted=False)), generation),
    )
    callback_thread.start()
    callback_thread.join(timeout=0.05)
    assert callback_thread.is_alive()
    release_reset.set()
    reset_thread.join(timeout=2.0)
    callback_thread.join(timeout=2.0)

    assert not reset_thread.is_alive()
    assert not callback_thread.is_alive()
    assert events == [('reset', 'complete'), ('reset', 'result')]


def test_navigation_rejection_retires_once_and_duplicate_callback_is_silent() -> None:
    coordinator = _reset_route_coordinator()
    coordinator.pending_structural_map = None
    coordinator.cognitive_graph_feedback_active = None
    coordinator.cognitive_graph_mode = 'gvg'
    generation = coordinator._route_callback_generation()

    rejected = SimpleNamespace(result=lambda: SimpleNamespace(accepted=False))
    coordinator._on_navigation_goal_handle(rejected, generation)
    coordinator._on_navigation_goal_handle(rejected, generation)

    assert coordinator.route_active is False
    assert coordinator.pending_goal is None
    assert [message.data for message in coordinator.goal_complete_pub.messages] == [False]
    assert [json.loads(message.data) for message in coordinator.goal_result_pub.messages] == [{
        'request_id': 41,
        'status': 'failed',
        'reason': 'navigate_to_pose_rejected',
        'reset_epoch': 8,
    }]


def test_old_compute_rejection_cannot_mark_fresh_goal_fallback() -> None:
    coordinator = RouteCoordinator.__new__(RouteCoordinator)
    coordinator.pending_goal = object()
    coordinator.request_id = 6
    coordinator.graph_generation = 2
    coordinator.graph = SimpleNamespace(graph_id='cognitive', revision=5)
    coordinator.gvg_graph = SimpleNamespace(graph_id='physical', revision=4)
    coordinator.gvg_support = object()
    coordinator.support = object()
    coordinator.desired_graph = coordinator.graph
    coordinator.desired_support = coordinator.support
    coordinator.desired_graph_generation = 1
    coordinator.graph_transaction_generation = None
    coordinator.graph_coherent = True
    coordinator.graph_reassert_required = False
    coordinator.cognitive_graph_mode = 'primary'
    coordinator.primary_fallback_used = False
    coordinator.node = SimpleNamespace(get_logger=lambda: SimpleNamespace(
        warning=lambda _message: None))
    reconciliations = []
    coordinator._ensure_desired_graph = (
        lambda reason, **_kwargs: reconciliations.append(reason))
    generation = coordinator._route_callback_generation()
    fallback_entered = threading.Event()
    release_fallback = threading.Event()
    original_fallback = coordinator._fallback_to_gvg_once

    def blocked_fallback(reason, token=None):
        fallback_entered.set()
        assert release_fallback.wait(timeout=2.0)
        return original_fallback(reason, token)

    coordinator._fallback_to_gvg_once = blocked_fallback
    callback_thread = threading.Thread(
        target=coordinator._on_route_goal_handle,
        args=(SimpleNamespace(result=lambda: SimpleNamespace(accepted=False)), generation),
    )
    callback_thread.start()
    assert fallback_entered.wait(timeout=2.0)
    with coordinator._route_state_lock():
        coordinator.request_id = 7
        coordinator.pending_goal = object()
        coordinator.primary_fallback_used = False
    release_fallback.set()
    callback_thread.join(timeout=2.0)

    assert not callback_thread.is_alive()
    assert coordinator.primary_fallback_used is False
    assert coordinator.desired_graph.graph_id == 'cognitive'
    assert reconciliations == []


def test_runtime_empty_snapshot_uses_requested_graph_identity() -> None:
    class RuntimeArray:
        def __init__(self):
            self.header = SimpleNamespace(stamp=None, frame_id='')
            self.graph_id = ''
            self.graph_revision = 0
            self.states = []

    coordinator = RouteCoordinator.__new__(RouteCoordinator)
    coordinator.RuntimeEdgeStateArray = RuntimeArray
    coordinator.frame_id = 'map'
    coordinator.graph = SimpleNamespace(graph_id='stale-cognitive', revision=9)
    coordinator.runtime = SimpleNamespace(edges={})
    coordinator._now = lambda: SimpleNamespace(to_msg=lambda: 'stamp')
    coordinator.runtime_pub = _CapturePublisher()
    gvg = SimpleNamespace(graph_id='physical', revision=4)

    coordinator._publish_runtime_states(graph=gvg)

    message = coordinator.runtime_pub.messages[0]
    assert message.graph_id == 'physical'
    assert message.graph_revision == 4
    assert message.states == []


def test_post_reset_timers_and_route_callback_cannot_publish_old_request() -> None:
    coordinator = _reset_route_coordinator()
    generation = coordinator._route_callback_generation()
    coordinator._on_reset_stop_gate_status(_gate_status(2, True, 'hold'))
    coordinator._current_xy = lambda: pytest.fail('retired tracker was evaluated')
    coordinator._publish_progress()
    coordinator._publish_route_context()
    route_result_calls = []
    coordinator._on_route_result(
        SimpleNamespace(result=lambda: route_result_calls.append(True)), generation)

    assert route_result_calls == [True]
    assert coordinator.context_pub.messages == []
    assert coordinator.progress_pub.messages == []
    assert coordinator.lookahead_pub.messages == []
    assert coordinator.goal_update_pub.messages == []
    assert coordinator.route_pub.messages == []


def test_reset_without_active_route_does_not_publish_fake_terminal() -> None:
    coordinator = _reset_route_coordinator(active=False)

    coordinator._on_reset_event(None)

    assert coordinator.goal_complete_pub.messages == []
    assert coordinator.goal_result_pub.messages == []


def test_new_goal_preemption_clears_old_tracker_before_context_and_can_restart() -> None:
    handle = _AcceptedHandle()
    coordinator = _reset_route_coordinator(handle=handle)
    coordinator.module2_enabled = True
    coordinator._now = lambda: SimpleNamespace(nanoseconds=10)
    coordinator._arm_prior_request = lambda _now: None
    observed = []

    def publish_context():
        observed.append((coordinator.request_id, coordinator.tracker, coordinator.pending_goal))

    coordinator._publish_route_context = publish_context
    goal = SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(
        x=3.0, y=4.0)))
    coordinator._on_goal(goal)

    assert handle.cancel_calls == 1
    assert observed == []
    assert coordinator.graph_reconciliations[-1] == (
        'new goal requires Route Server GVG')
    coordinator.graph_coherent = True
    coordinator.graph_reassert_required = False
    coordinator._resume_pending_goal_after_graph_coherent()
    assert observed == [(42, None, goal)]
    assert coordinator.route_active is True
    assert coordinator.navigation_goal_pending is False

    coordinator._on_reset_event(None)
    coordinator.module2_enabled = False
    prepared = []
    coordinator._prepare_route = lambda priors: prepared.append(priors)
    coordinator._publish_route_context = lambda: None
    fresh_goal = SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(
        x=5.0, y=6.0)))
    coordinator._on_goal(fresh_goal)
    assert coordinator.pending_goal is fresh_goal
    assert coordinator.request_id == 44
    assert prepared == []
    coordinator.graph_coherent = True
    coordinator.graph_reassert_required = False
    coordinator._resume_pending_goal_after_graph_coherent()
    assert prepared == [{}]


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


def test_module2_delta_is_capped_and_confidence_applied_exactly_once() -> None:
    edge = _graph().edge_by_id()[2]
    value = edge_cost_breakdown(
        edge,
        _cost_settings(),
        prior_cost_delta_m=10.0,
        prior_confidence=0.4,
        runtime_penalty_m=0.3,
    )
    assert value.requested_module2_delta_m == 10.0
    assert value.applied_module2_delta_m == 0.25 * edge.length_m * 0.4
    assert value.final_cost_m == (
        value.structural_cost_m
        + value.applied_module2_delta_m
        + value.runtime_penalty_m
    )


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


def test_route_odometry_topic_defaults_to_odom_and_allows_explicit_estimates() -> None:
    assert DEFAULT_ROUTE_ODOMETRY_TOPIC == "/odom"
    assert validate_route_odometry_topic("/wheel/odom") == "/wheel/odom"


@pytest.mark.parametrize(
    "topic",
    [
        "/ground_truth/odom",
        "/isaac/ground-truth/pose",
        "/sim/groundtruth/odom",
    ],
)
def test_route_odometry_topic_rejects_ground_truth(topic: str) -> None:
    with pytest.raises(ValueError, match="must not use ground-truth"):
        validate_route_odometry_topic(topic)


def test_map_pose_prefers_tf_over_fresh_map_frame_odometry() -> None:
    assert select_map_pose(
        "map", "map", (1.0, 2.0), 0.1, 0.5, (9.0, 8.0)
    ) == (9.0, 8.0)


def test_map_pose_fallback_requires_fresh_map_frame_odometry() -> None:
    assert select_map_pose(
        "map", "map", (1.0, 2.0), 0.1, 0.5, None
    ) == (1.0, 2.0)
    assert select_map_pose(
        "map", "odom", (1.0, 2.0), 0.1, 0.5, None
    ) is None
    assert select_map_pose(
        "map", "map", (1.0, 2.0), 0.6, 0.5, None
    ) is None
    assert select_map_pose(
        "map", "map", (1.0, 2.0), -0.1, 0.5, None
    ) is None


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
    coordinator.pending_goal = SimpleNamespace(
        pose=SimpleNamespace(position=SimpleNamespace(x=1.0, y=2.0))
    )
    coordinator.route_goal_completion_tolerance_m = 0.25
    coordinator.navigation_goal_targets_final = True
    coordinator._current_xy = lambda: (1.0, 2.0)
    coordinator.tracker = object()
    coordinator.navigation_goal_pending = True
    coordinator.navigation_goal_handle = object()
    coordinator.navigation_failed = True
    coordinator.pending_structural_map = None
    coordinator.goal_complete_pub = Publisher()
    coordinator.goal_result_pub = _CapturePublisher()
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
    assert json.loads(coordinator.goal_result_pub.messages[0].data) == {
        'request_id': 0,
        'status': 'succeeded',
        'reason': 'final_goal_distance_confirmed',
        'reset_epoch': 0,
    }


def test_intermediate_lookahead_success_continues_same_leg() -> None:
    events = []

    coordinator = RouteCoordinator.__new__(RouteCoordinator)
    coordinator.route_active = True
    coordinator.pending_goal = SimpleNamespace(
        pose=SimpleNamespace(position=SimpleNamespace(x=10.0, y=0.0))
    )
    coordinator.route_goal_completion_tolerance_m = 0.20
    coordinator.navigation_goal_targets_final = False
    coordinator._current_xy = lambda: (2.8, 0.0)
    coordinator.tracker = object()
    coordinator.navigation_goal_pending = False
    coordinator.navigation_goal_handle = object()
    coordinator.navigation_failed = False
    coordinator.goal_complete_pub = SimpleNamespace(
        publish=lambda message: events.append(message.data)
    )
    coordinator.goal_result_pub = _CapturePublisher()
    coordinator.node = SimpleNamespace(
        get_logger=lambda: SimpleNamespace(info=lambda _message: None)
    )
    future = SimpleNamespace(
        result=lambda: SimpleNamespace(
            status=4,
            result=SimpleNamespace(error_code=0),
        )
    )

    coordinator._on_navigation_result(future)

    assert events == []
    assert coordinator.route_active
    assert coordinator.pending_goal is not None
    assert coordinator.tracker is not None
    assert coordinator.navigation_goal_handle is None


def test_final_goal_success_accepts_map_pose_inside_campaign_gate() -> None:
    events = []
    coordinator = RouteCoordinator.__new__(RouteCoordinator)
    coordinator.route_active = True
    coordinator.pending_goal = SimpleNamespace(
        pose=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0))
    )
    coordinator.route_goal_completion_tolerance_m = 0.25
    coordinator.navigation_goal_targets_final = True
    coordinator._current_xy = lambda: (0.223, 0.0)
    coordinator.tracker = object()
    coordinator.navigation_goal_pending = False
    coordinator.navigation_goal_handle = object()
    coordinator.navigation_failed = False
    coordinator.pending_structural_map = None
    coordinator.goal_complete_pub = SimpleNamespace(
        publish=lambda message: events.append(message.data)
    )
    coordinator.goal_result_pub = _CapturePublisher()
    coordinator.node = SimpleNamespace(
        get_logger=lambda: SimpleNamespace(info=lambda _message: None)
    )
    future = SimpleNamespace(result=lambda: SimpleNamespace(
        status=4, result=SimpleNamespace(error_code=0),
    ))

    coordinator._on_navigation_result(future)

    assert events == [True]
    assert coordinator.pending_goal is None
    assert json.loads(coordinator.goal_result_pub.messages[0].data)['status'] == 'succeeded'


def test_final_navigation_failure_publishes_one_bool_json_pair() -> None:
    coordinator = _reset_route_coordinator()
    coordinator.pending_structural_map = None
    coordinator.cognitive_graph_feedback_active = None
    coordinator.cognitive_graph_mode = 'gvg'
    coordinator.navigation_goal_pending = False
    coordinator.navigation_goal_handle = object()
    generation = coordinator._route_callback_generation()
    future = SimpleNamespace(result=lambda: SimpleNamespace(
        status=6, result=SimpleNamespace(error_code=207)))

    coordinator._on_navigation_result(future, generation)
    coordinator._on_navigation_result(future, generation)

    assert [message.data for message in coordinator.goal_complete_pub.messages] == [False]
    assert [json.loads(message.data) for message in coordinator.goal_result_pub.messages] == [{
        'request_id': 41,
        'status': 'failed',
        'reason': 'navigate_to_pose_failed_error_207',
        'reset_epoch': 8,
    }]


def _runtime_manager_for_hold_race() -> RuntimeEdgeManager:
    return RuntimeEdgeManager(
        {
            'block_after_occupied_s': 0.0,
            'block_after_consecutive_failures': 1,
            'reopen_after_clear_s': 0.5,
            'unknown_after_unobserved_s': 0.5,
        },
        {
            'suspect_edge_penalty_m': 1.0,
            'blocked_edge_penalty_m': 10.0,
            'unknown_edge_penalty_m': 2.0,
        },
    )


def test_runtime_edge_77_observation_crossing_hold_is_discarded_for_100_rounds() -> None:
    message = SimpleNamespace(
        edge_id=77,
        observed_clear=False,
        planning_failed=True,
        occupied_ahead=True,
    )
    for _round in range(100):
        coordinator = _reset_route_coordinator(active=False)
        coordinator.runtime = _runtime_manager_for_hold_race()
        entered = threading.Barrier(2)
        release = threading.Event()

        def blocked_now():
            entered.wait(timeout=2.0)
            assert release.wait(timeout=2.0)
            return SimpleNamespace(nanoseconds=1_000_000_000)

        coordinator._now = blocked_now
        callback = threading.Thread(
            target=coordinator._on_runtime_observation,
            args=(message,),
        )
        callback.start()
        entered.wait(timeout=2.0)
        coordinator._on_reset_stop_gate_status(_gate_status(2, True, 'hold'))
        release.set()
        callback.join(timeout=2.0)

        assert not callback.is_alive()
        assert 77 not in coordinator.runtime.edges
        assert coordinator.runtime_snapshots == []


def test_runtime_tick_crossing_hold_cannot_restore_cleared_edge() -> None:
    coordinator = _reset_route_coordinator(active=False)
    coordinator.runtime = _runtime_manager_for_hold_race()
    edge = coordinator.runtime.state(77)
    edge.last_observed_s = 0.0
    coordinator.defaults = {
        'module2_edge_prior': {'active_refresh_period_s': 5.0},
    }
    coordinator.module2_enabled = False
    coordinator.module2_prior_ttl_s = 2.0
    coordinator.latest_priors_stamp_ns = None
    entered = threading.Barrier(2)
    release = threading.Event()

    def blocked_now():
        entered.wait(timeout=2.0)
        assert release.wait(timeout=2.0)
        return SimpleNamespace(nanoseconds=1_000_000_000)

    coordinator._now = blocked_now
    callback = threading.Thread(target=coordinator._runtime_tick)
    callback.start()
    entered.wait(timeout=2.0)
    coordinator._on_reset_stop_gate_status(_gate_status(2, True, 'hold'))
    release.set()
    callback.join(timeout=2.0)

    assert not callback.is_alive()
    assert coordinator.runtime.edges == {}
    assert coordinator.runtime_snapshots == []


def test_prior_validation_crossing_hold_cannot_commit_latest_or_feedback(
    monkeypatch,
) -> None:
    coordinator = _reset_route_coordinator(active=True)
    graph = SimpleNamespace(
        graph_id='test:gvg_v1', revision=3,
        edges=[SimpleNamespace(id=7)],
    )
    coordinator.graph = graph
    coordinator.gvg_graph = graph
    coordinator.desired_graph = graph
    coordinator.pending_deadline_ns = 6_500_000_000
    coordinator.pending_prior_request_id = 41
    coordinator.pending_prior_graph_id = graph.graph_id
    coordinator.pending_prior_graph_revision = graph.revision
    coordinator.pending_prior_started_ns = 5_500_000_000
    coordinator.pending_prior_model_id = None
    coordinator.latest_priors = {}
    coordinator.latest_priors_stamp_ns = None
    coordinator.latest_prior_model_id = None
    coordinator.module2_prior_ttl_s = 2.0
    coordinator._now = lambda: SimpleNamespace(nanoseconds=6_000_000_000)
    prepared = []
    coordinator._prepare_route = lambda priors: prepared.append(priors)
    entered = threading.Barrier(2)
    release = threading.Event()
    original = ros_node_module.edge_prior_is_usable

    def blocked_validation(**kwargs):
        entered.wait(timeout=2.0)
        assert release.wait(timeout=2.0)
        return original(**kwargs)

    monkeypatch.setattr(
        ros_node_module, 'edge_prior_is_usable', blocked_validation)
    callback = threading.Thread(
        target=coordinator._on_priors,
        args=(_edge_prior_message(request_id=41, stamp_ns=5_800_000_000),),
    )
    callback.start()
    entered.wait(timeout=2.0)
    coordinator._on_reset_stop_gate_status(_gate_status(2, True, 'hold'))
    release.set()
    callback.join(timeout=2.0)

    assert not callback.is_alive()
    assert coordinator.latest_priors == {}
    assert coordinator.pending_deadline_ns is None
    assert prepared == []


def test_structural_observation_crossing_hold_cannot_queue_or_rebuild(
    monkeypatch,
) -> None:
    coordinator = _reset_route_coordinator(active=False)
    structural_map = OccupancyMap(
        np.ones((8, 8), dtype=bool),
        0.05,
        (0.0, 0.0),
        'map-v1',
        Path('/tmp/map.yaml'),
    )
    settings = {
        'ros_free_max_occupancy': 20,
        'changed_area_m2': 0.0,
        'stable_snapshot_count': 1,
        'stable_for_s': 0.0,
    }
    coordinator.map = structural_map
    coordinator.defaults = {'structural_updates': settings}
    coordinator.structural_monitor = StructuralChangeMonitor(
        structural_map.free, structural_map.resolution_m, settings)
    coordinator.pending_structural_map = None
    coordinator.pending_structural_intent = None
    coordinator.structural_observation_generation = 0
    coordinator._now = lambda: SimpleNamespace(nanoseconds=1_000_000_000)
    rebuilds = []
    coordinator._try_deferred_structural_rebuild = (
        lambda *args, **kwargs: rebuilds.append((args, kwargs)))
    entered = threading.Barrier(2)
    release = threading.Event()

    def blocked_observe(self, free, now_s):
        entered.wait(timeout=2.0)
        assert release.wait(timeout=2.0)
        self.last_candidate = np.asarray(free, dtype=bool).copy()
        self.first_stable_s = float(now_s)
        self.stable_count = 1
        return True

    monkeypatch.setattr(StructuralChangeMonitor, 'observe', blocked_observe)
    message = SimpleNamespace(
        data=np.zeros(64, dtype=np.int8),
        info=SimpleNamespace(height=8, width=8),
    )
    callback = threading.Thread(
        target=coordinator._on_structural_map,
        args=(message,),
    )
    callback.start()
    entered.wait(timeout=2.0)
    coordinator._on_reset_stop_gate_status(_gate_status(2, True, 'hold'))
    release.set()
    callback.join(timeout=2.0)

    assert not callback.is_alive()
    assert coordinator.pending_structural_map is None
    assert coordinator.pending_structural_intent is None
    assert rebuilds == []


def test_region_tick_crossing_hold_cannot_select_or_publish() -> None:
    coordinator = _reset_route_coordinator(active=False)
    entered = threading.Barrier(2)
    release = threading.Event()
    select_calls = []
    constraint_calls = []

    class Selector:
        current = SimpleNamespace(region_id='old')
        last_switch_s = 0.0

        def select(self, xy, now_s):
            select_calls.append((xy, now_s))
            return SimpleNamespace(region_id='new')

    coordinator.region_selector = Selector()

    def blocked_xy():
        entered.wait(timeout=2.0)
        assert release.wait(timeout=2.0)
        return (1.0, 2.0)

    coordinator._current_xy = blocked_xy
    coordinator._now = lambda: SimpleNamespace(nanoseconds=1_000_000_000)
    coordinator._publish_cognitive_constraints = (
        lambda: constraint_calls.append(True))
    callback = threading.Thread(target=coordinator._region_tick)
    callback.start()
    entered.wait(timeout=2.0)
    coordinator._on_reset_stop_gate_status(_gate_status(2, True, 'hold'))
    release.set()
    callback.join(timeout=2.0)

    assert not callback.is_alive()
    assert select_calls == []
    assert constraint_calls == []


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
