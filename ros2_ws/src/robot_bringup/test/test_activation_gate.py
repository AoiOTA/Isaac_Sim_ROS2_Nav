from pathlib import Path
import threading
from types import SimpleNamespace

from nav2_msgs.srv import ManageLifecycleNodes
import pytest

from robot_bringup.activation_gate import Nav2ActivationGate
from robot_bringup.lifecycle_policy import RetryPolicy
from robot_bringup.readiness import ReadinessConfig, ReadinessTracker


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


class _Logger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(('info', message))

    def warning(self, message):
        self.messages.append(('warning', message))

    def error(self, message):
        self.messages.append(('error', message))


class _Future:
    def __init__(self, response=None, exception=None):
        self._response = response
        self._exception = exception

    def result(self):
        if self._exception is not None:
            raise self._exception
        return self._response


def _pose_message(stamp_s, *, frame_id='map'):
    sec = int(stamp_s)
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(
                sec=sec,
                nanosec=int(round((stamp_s - sec) * 1.0e9)),
            ),
            frame_id=frame_id,
        ),
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                orientation=SimpleNamespace(
                    x=0.0, y=0.0, z=0.0, w=1.0),
            ),
            covariance=[0.0] * 36,
        ),
    )


def _amcl_gate(*, clock_stamp_s=10.0, clock_floor_s=9.5):
    gate = SimpleNamespace(
        _localization_backend='amcl',
        _clock_stamp_s=clock_stamp_s,
        _freshness_timeout=0.5,
        _amcl_epoch_clock_floor_s=clock_floor_s,
        _amcl_initialpose_stamp_s=None,
        _amcl_initialpose_received_at=None,
        _amcl_pose_stamp_s=None,
        _amcl_pose_received_at=None,
        _amcl_tf_stamp_s=None,
        _amcl_tf_received_at=None,
        _amcl_tf_anchor=None,
        _amcl_tf_stable_since=None,
        _amcl_tf_stable_duration=0.2,
        _amcl_tf_translation_tolerance=0.05,
        _amcl_tf_yaw_tolerance=0.05,
        _state_query_lock=threading.RLock(),
    )
    gate._pose_message_is_finite = Nav2ActivationGate._pose_message_is_finite
    gate._sample_is_current_epoch = lambda stamp_s: \
        Nav2ActivationGate._sample_is_current_epoch(gate, stamp_s)
    gate._amcl_pose_is_fresh = lambda now: \
        Nav2ActivationGate._amcl_pose_is_fresh(gate, now)
    gate._amcl_transform_is_stable = lambda now: \
        Nav2ActivationGate._amcl_transform_is_stable(gate, now)
    gate._clear_amcl_transform = lambda: \
        Nav2ActivationGate._clear_amcl_transform(gate)
    return gate


def _gate_harness():
    failures = []
    stages = []
    logger = _Logger()
    token = object()
    gate = SimpleNamespace(
        _generation=4,
        _request_in_flight=True,
        _manager_request_token=token,
        _state_query_lock=threading.RLock(),
        _managed_nodes=['controller_server', 'planner_server'],
        _activation_verifying=False,
        _recovery_pause_verifying=False,
        _recovery_resume_verifying=False,
        _next_attempt_at=0.0,
        _record_failure=lambda reason, now, **kwargs: failures.append(
            (reason, kwargs)
        ),
        _set_recovery_stage=stages.append,
        _set_fatal=lambda reason: failures.append((reason, {})),
        _command_name=Nav2ActivationGate._command_name,
        get_logger=lambda: logger,
    )
    return gate, failures, stages, logger, token


def test_manager_false_response_is_diagnostic_and_retryable():
    gate, failures, _, _, token = _gate_harness()
    response = SimpleNamespace(success=False)
    Nav2ActivationGate._manager_command_done(
        gate,
        _Future(response=response),
        context='activation',
        command=ManageLifecycleNodes.Request.STARTUP,
        generation=4,
        token=token,
    )
    assert not gate._request_in_flight
    assert len(failures) == 1
    assert 'returned success=False' in failures[0][0]
    assert 'controller_server' in failures[0][0]
    assert failures[0][1]['attempt_already_counted'] is True


def test_manager_future_exception_is_captured_for_finite_retry():
    gate, failures, _, _, token = _gate_harness()
    Nav2ActivationGate._manager_command_done(
        gate,
        _Future(exception=RuntimeError('service disappeared')),
        context='activation',
        command=ManageLifecycleNodes.Request.STARTUP,
        generation=4,
        token=token,
    )
    assert len(failures) == 1
    assert 'service disappeared' in failures[0][0]
    assert failures[0][1]['attempt_already_counted'] is True


def test_manager_success_requires_a_followup_state_verification():
    gate, failures, stages, logger, token = _gate_harness()
    Nav2ActivationGate._manager_command_done(
        gate,
        _Future(response=SimpleNamespace(success=True)),
        context='activation',
        command=ManageLifecycleNodes.Request.STARTUP,
        generation=4,
        token=token,
    )
    assert failures == []
    assert stages == []
    assert gate._activation_verifying
    assert any('STARTUP accepted' in message for _, message in logger.messages)


def test_stale_async_generation_cannot_mutate_new_epoch():
    gate, failures, stages, logger, token = _gate_harness()
    Nav2ActivationGate._manager_command_done(
        gate,
        _Future(response=SimpleNamespace(success=False)),
        context='activation',
        command=ManageLifecycleNodes.Request.STARTUP,
        generation=3,
        token=token,
    )
    assert gate._request_in_flight
    assert failures == []
    assert stages == []
    assert logger.messages == []


class _PendingFuture:
    def __init__(self):
        self.callback = None

    def add_done_callback(self, callback):
        self.callback = callback


class _CountingClient:
    def __init__(self):
        self.calls = 0
        self.future = _PendingFuture()

    def call_async(self, request):
        del request
        self.calls += 1
        return self.future


def test_snapshot_to_manager_reservation_allows_only_one_startup():
    """Two completed-snapshot handlers cannot reserve duplicate STARTUPs."""
    client = _CountingClient()
    logger = _Logger()
    gate = SimpleNamespace(
        _state_query_lock=threading.RLock(),
        _snapshot_in_flight=True,
        _snapshot_generation=8,
        _generation=8,
        _request_in_flight=False,
        _manager_request_token=None,
        _attempts=0,
        _retry_policy=RetryPolicy(),
        _managed_nodes=['controller_server'],
        _lifecycle_client=client,
        _command_name=Nav2ActivationGate._command_name,
        _snapshot_is_current=lambda generation: (
            gate._snapshot_in_flight
            and generation == gate._generation
            and generation == gate._snapshot_generation
        ),
        _consume_snapshot=lambda generation: _consume_snapshot(
            gate, generation),
        _manager_command_done=lambda *args, **kwargs: None,
        _record_failure=lambda *args, **kwargs: None,
        get_logger=lambda: logger,
    )
    barrier = threading.Barrier(3)
    results = []

    def reserve():
        barrier.wait()
        results.append(Nav2ActivationGate._send_manager_command(
            gate,
            ManageLifecycleNodes.Request.STARTUP,
            'activation',
            snapshot_generation=8,
        ))

    workers = [threading.Thread(target=reserve) for _ in range(2)]
    for worker in workers:
        worker.start()
    barrier.wait()
    for worker in workers:
        worker.join(timeout=1.0)

    assert not any(worker.is_alive() for worker in workers)
    assert sorted(results) == [False, True]
    assert client.calls == 1
    assert gate._attempts == 1


def _consume_snapshot(gate, generation):
    if not gate._snapshot_is_current(generation):
        return False
    gate._snapshot_in_flight = False
    gate._snapshot_generation = None
    return True


class _BlockingFuture:
    def __init__(self, response):
        self._response = response
        self.entered = threading.Event()
        self.release = threading.Event()

    def result(self):
        self.entered.set()
        assert self.release.wait(timeout=1.0)
        return self._response


def test_manager_completion_racing_epoch_cannot_clear_new_request():
    gate, failures, stages, logger, old_token = _gate_harness()
    future = _BlockingFuture(SimpleNamespace(success=False))
    worker = threading.Thread(
        target=Nav2ActivationGate._manager_command_done,
        args=(gate, future),
        kwargs={
            'context': 'activation',
            'command': ManageLifecycleNodes.Request.STARTUP,
            'generation': 4,
            'token': old_token,
        },
    )
    worker.start()
    assert future.entered.wait(timeout=1.0)
    new_token = object()
    with gate._state_query_lock:
        gate._generation = 5
        gate._manager_request_token = new_token
        gate._request_in_flight = True
    future.release.set()
    worker.join(timeout=1.0)

    assert not worker.is_alive()
    assert gate._generation == 5
    assert gate._manager_request_token is new_token
    assert gate._request_in_flight
    assert failures == []
    assert stages == []
    assert logger.messages == []


def test_recovery_completion_racing_epoch_cannot_advance_new_stage():
    logger = _Logger()
    old_token = object()
    old_operation = {'token': old_token}
    failures = []
    gate = SimpleNamespace(
        _generation=2,
        _state_query_lock=threading.RLock(),
        _recovery_service_in_flight=True,
        _recovery_service_operation=old_operation,
        _set_recovery_stage=lambda stage: failures.append(stage),
        _handle_recovery_service_failure=lambda *args: failures.append(args),
        get_logger=lambda: logger,
    )
    future = _BlockingFuture(SimpleNamespace(success=False, message='old'))
    worker = threading.Thread(
        target=Nav2ActivationGate._recovery_service_done,
        args=(gate, future),
        kwargs={
            'label': 'clear global costmap',
            'next_stage': 'clear_local',
            'required': True,
            'generation': 2,
            'token': old_token,
        },
    )
    worker.start()
    assert future.entered.wait(timeout=1.0)
    new_token = object()
    new_operation = {'token': new_token}
    with gate._state_query_lock:
        gate._generation = 3
        gate._recovery_service_operation = new_operation
        gate._recovery_service_in_flight = True
    future.release.set()
    worker.join(timeout=1.0)

    assert not worker.is_alive()
    assert gate._recovery_service_operation is new_operation
    assert gate._recovery_service_in_flight
    assert failures == []


def test_required_recovery_failure_retries_then_fails_without_advancing():
    logger = _Logger()
    fatals = []
    gate = SimpleNamespace(
        _last_failure='',
        _recovery_stage_attempts=0,
        _retry_policy=RetryPolicy(
            max_attempts=3,
            initial_backoff=0.1,
            maximum_backoff=0.2,
        ),
        _next_attempt_at=0.0,
        _recovery_stage_started_at=0.0,
        _set_fatal=fatals.append,
        _set_recovery_stage=lambda stage: (_ for _ in ()).throw(
            AssertionError(f'unexpected stage advance: {stage}')),
        get_logger=lambda: logger,
    )

    for attempt in range(1, 4):
        Nav2ActivationGate._handle_recovery_service_failure(
            gate,
            'clear global costmap',
            'service rejected request',
            'clear_local',
            True,
            float(attempt),
        )
        assert gate._recovery_stage_attempts == attempt

    assert len(fatals) == 1
    assert 'Required recovery step failed after 3 attempts' in fatals[0]
    assert 'clear global costmap' in fatals[0]


def test_localization_wait_policy_keeps_gate_nonfatal_after_startup_timeout():
    logger = _Logger()
    fatals = []
    gate = SimpleNamespace(
        _started_at=10.0,
        _startup_timeout=5.0,
        _startup_timeout_policy='wait_for_localization',
        _startup_timeout_reported=False,
        _missing_readiness_requirements=lambda now: ['stable map->odom'],
        _last_failure='',
        _managed_nodes=['controller_server'],
        _set_fatal=fatals.append,
        get_logger=lambda: logger,
    )

    assert not Nav2ActivationGate._handle_startup_timeout(gate, 15.1)
    assert not Nav2ActivationGate._handle_startup_timeout(gate, 20.0)
    assert fatals == []
    assert logger.messages == [(
        'warning',
        'Nav2 activation gate timed out after 5.1s; '
        'missing=stable map->odom; last_failure=none; '
        "managed_nodes=['controller_server']; "
        'startup_timeout_policy=wait_for_localization, continuing diagnostics '
        'with Nav2 inactive',
    )]


def test_autonomy_policy_fails_closed_after_startup_timeout():
    fatals = []
    gate = SimpleNamespace(
        _started_at=10.0,
        _startup_timeout=5.0,
        _startup_timeout_policy='fail_closed',
        _startup_timeout_reported=False,
        _missing_readiness_requirements=lambda now: [
            'grid localization generation'],
        _last_failure='',
        _managed_nodes=['controller_server'],
        _set_fatal=fatals.append,
    )

    assert Nav2ActivationGate._handle_startup_timeout(gate, 15.1)
    assert len(fatals) == 1
    assert 'missing=grid localization generation' in fatals[0]


def test_gate_source_keeps_wall_timer_and_explicit_recovery_sequence():
    source = (
        PACKAGE_ROOT / 'robot_bringup' / 'activation_gate.py'
    ).read_text()
    assert 'ClockType.STEADY_TIME' in source
    assert 'self._timer.cancel()' not in source
    assert "self._set_recovery_stage('cancel_goal')" in source
    for stage in (
        'pause_query',
        'clear_global',
        'clear_local',
        'waiting_localization',
        'resume_query',
    ):
        assert stage in source
    assert "CancelGoal, '/navigate_to_pose/_action/cancel_goal'" in source
    assert "'/global_costmap/clear_entirely_global_costmap'" in source
    assert "'/local_costmap/clear_entirely_local_costmap'" in source
    assert "'/bio_nav/localization/status'" in source
    assert "'/initial_pose/reseed'" not in source
    assert "self.declare_parameter('immutable_map_node', 'map_server')" in source
    assert 'Repairing immutable map lifecycle' in source
    assert 'except (KeyboardInterrupt, ExternalShutdownException):' in source
    assert 'if node is not None:\n            node.destroy_node()' in source


def _localization_status(
        generation, state, accepted, *, level=1, correction=(0.0, 0.0, 0.0)):
    values = [
        SimpleNamespace(key='generation', value=str(generation)),
        SimpleNamespace(key='state', value=state),
        SimpleNamespace(key='accepted', value=str(accepted).lower()),
        SimpleNamespace(key='correction_x_m', value=str(correction[0])),
        SimpleNamespace(key='correction_y_m', value=str(correction[1])),
        SimpleNamespace(key='correction_yaw_rad', value=str(correction[2])),
    ]
    return SimpleNamespace(status=[SimpleNamespace(
        name='grid_localization', level=level, values=values)])


def test_warn_waiting_status_is_healthy_but_not_ready_until_accepted():
    logger = _Logger()
    gate = SimpleNamespace(
        _state_query_lock=threading.RLock(),
        _localization_generation=0,
        _localization_state='',
        _localization_accepted_generation=0,
        _localization_generation_floor=0,
        _localization_requires_active_generation=False,
        _localization_active_generation=None,
        _localization_accepted_correction=None,
        _map_to_odom_correction=(0.0, 0.0, 0.0),
        _localization_tf_translation_tolerance=0.01,
        _localization_tf_yaw_tolerance=0.01,
        _tracker=SimpleNamespace(missing_requirements=lambda now: []),
        get_logger=lambda: logger,
    )
    gate._localization_correction_matches_transform = lambda: \
        Nav2ActivationGate._localization_correction_matches_transform(gate)

    Nav2ActivationGate._localization_status_callback(
        gate, _localization_status(1, 'WAITING_FOR_SCAN', False))
    assert gate._localization_state == 'WAITING_FOR_SCAN'
    assert Nav2ActivationGate._missing_readiness_requirements(gate, 0.0)
    assert logger.messages == []

    Nav2ActivationGate._localization_status_callback(
        gate, _localization_status(1, 'WAITING_FOR_RESULT', False))
    assert gate._localization_state == 'WAITING_FOR_RESULT'
    Nav2ActivationGate._localization_status_callback(
        gate, _localization_status(1, 'ACCEPTED', True, level=0))
    assert Nav2ActivationGate._missing_readiness_requirements(gate, 0.0) == []


def test_reset_floor_requires_a_new_accepted_localization_generation():
    gate = SimpleNamespace(
        _state_query_lock=threading.RLock(),
        _localization_generation=4,
        _localization_state='ACCEPTED',
        _localization_accepted_generation=4,
        _localization_generation_floor=4,
        _localization_requires_active_generation=True,
        _localization_active_generation=None,
        _localization_accepted_correction=None,
        _map_to_odom_correction=(0.0, 0.0, 0.0),
        _localization_tf_translation_tolerance=0.01,
        _localization_tf_yaw_tolerance=0.01,
        _tracker=SimpleNamespace(missing_requirements=lambda now: []),
        get_logger=lambda: _Logger(),
    )
    gate._localization_correction_matches_transform = lambda: \
        Nav2ActivationGate._localization_correction_matches_transform(gate)

    Nav2ActivationGate._localization_status_callback(
        gate, _localization_status(4, 'ACCEPTED', True))
    assert Nav2ActivationGate._missing_readiness_requirements(gate, 0.0)
    Nav2ActivationGate._localization_status_callback(
        gate, _localization_status(5, 'ACCEPTED', True))
    assert Nav2ActivationGate._missing_readiness_requirements(gate, 0.0)
    Nav2ActivationGate._localization_status_callback(
        gate, _localization_status(5, 'WAITING_FOR_RESULT', False))
    assert Nav2ActivationGate._missing_readiness_requirements(gate, 0.0)
    Nav2ActivationGate._localization_status_callback(
        gate, _localization_status(5, 'ACCEPTED', True))
    assert Nav2ActivationGate._missing_readiness_requirements(gate, 0.0) == []


def _reset_localization_gate(*, observed_correction=None):
    gate = SimpleNamespace(
        _state_query_lock=threading.RLock(),
        _localization_generation=4,
        _localization_state='ACCEPTED',
        _localization_accepted_generation=4,
        _localization_generation_floor=4,
        _localization_requires_active_generation=True,
        _localization_active_generation=None,
        _localization_accepted_correction=None,
        _map_to_odom_correction=observed_correction,
        _localization_tf_translation_tolerance=0.01,
        _localization_tf_yaw_tolerance=0.01,
        _tracker=SimpleNamespace(missing_requirements=lambda now: []),
        get_logger=lambda: _Logger(),
    )
    gate._localization_correction_matches_transform = lambda: \
        Nav2ActivationGate._localization_correction_matches_transform(gate)
    return gate


def test_accepted_correction_must_match_actual_map_to_odom():
    gate = _reset_localization_gate(observed_correction=(9.0, 0.0, 0.0))
    Nav2ActivationGate._localization_status_callback(
        gate, _localization_status(5, 'WAITING_FOR_SCAN', False))
    Nav2ActivationGate._localization_status_callback(
        gate,
        _localization_status(
            5, 'ACCEPTED', True, correction=(1.0, 0.0, 0.0)),
    )
    assert Nav2ActivationGate._missing_readiness_requirements(gate, 0.0) == [
        'map->odom matching accepted localization correction']

    gate._map_to_odom_correction = (1.0, 0.0, 0.0)
    assert Nav2ActivationGate._missing_readiness_requirements(gate, 0.0) == []


@pytest.mark.parametrize('status_first', [False, True])
def test_matching_status_and_transform_callback_ordering_is_safe(status_first):
    correction = (1.0, -0.5, 0.2)
    gate = _reset_localization_gate(
        observed_correction=None if status_first else correction)
    Nav2ActivationGate._localization_status_callback(
        gate, _localization_status(5, 'WAITING_FOR_RESULT', False))
    Nav2ActivationGate._localization_status_callback(
        gate, _localization_status(
            5, 'ACCEPTED', True, correction=correction))
    if status_first:
        assert Nav2ActivationGate._missing_readiness_requirements(
            gate, 0.0)
        gate._map_to_odom_correction = correction
    assert Nav2ActivationGate._missing_readiness_requirements(gate, 0.0) == []


def test_same_correction_value_after_reset_is_safe():
    correction = (1.25, -0.25, -0.1)
    gate = _reset_localization_gate(observed_correction=correction)
    Nav2ActivationGate._localization_status_callback(
        gate, _localization_status(5, 'WAITING_FOR_SCAN', False))
    Nav2ActivationGate._localization_status_callback(
        gate, _localization_status(
            5, 'ACCEPTED', True, correction=correction))
    assert Nav2ActivationGate._missing_readiness_requirements(gate, 0.0) == []


def test_rejected_active_generation_cannot_accept_without_new_waiting():
    gate = _reset_localization_gate(observed_correction=(0.0, 0.0, 0.0))
    Nav2ActivationGate._localization_status_callback(
        gate, _localization_status(5, 'WAITING_FOR_RESULT', False))
    Nav2ActivationGate._localization_status_callback(
        gate, _localization_status(5, 'REJECTED', False))
    Nav2ActivationGate._localization_status_callback(
        gate, _localization_status(5, 'ACCEPTED', True))
    assert Nav2ActivationGate._missing_readiness_requirements(gate, 0.0)


def test_amcl_rejects_stale_and_out_of_order_pose_samples():
    gate = _amcl_gate()

    Nav2ActivationGate._initialpose_callback(gate, _pose_message(9.0))
    assert gate._amcl_initialpose_stamp_s is None

    Nav2ActivationGate._initialpose_callback(gate, _pose_message(10.0))
    Nav2ActivationGate._amcl_pose_callback(gate, _pose_message(9.9))
    assert gate._amcl_initialpose_stamp_s == 10.0
    assert gate._amcl_pose_stamp_s is None

    Nav2ActivationGate._amcl_pose_callback(gate, _pose_message(10.0))
    assert gate._amcl_pose_stamp_s == 10.0


def test_amcl_reset_clears_old_pose_and_tf_readiness():
    gate = _amcl_gate()
    Nav2ActivationGate._initialpose_callback(gate, _pose_message(10.0))
    Nav2ActivationGate._amcl_pose_callback(gate, _pose_message(10.0))
    Nav2ActivationGate._observe_amcl_transform(
        gate, 0.0, 0.0, 0.0, 10.0, 1.0)

    Nav2ActivationGate._reset_amcl_readiness(gate, 10.0)

    assert gate._amcl_initialpose_stamp_s is None
    assert gate._amcl_pose_stamp_s is None
    assert gate._amcl_tf_stamp_s is None


def test_amcl_requires_ordered_pose_and_post_pose_stable_tf(monkeypatch):
    now = 100.0
    monkeypatch.setattr(
        'robot_bringup.activation_gate.time.monotonic', lambda: now)
    tracker = ReadinessTracker(ReadinessConfig(
        freshness_timeout=0.5,
        tf_stable_duration=0.2,
        tf_translation_tolerance=0.05,
        tf_yaw_tolerance=0.05,
        clock_jump_tolerance=5.0,
    ))
    tracker.mark_clock(10.0, now)
    tracker.mark_scan(10.0, now)
    tracker.mark_odom(10.0, now)
    tracker.mark_map()
    gate = _amcl_gate()
    gate._tracker = tracker

    Nav2ActivationGate._initialpose_callback(gate, _pose_message(10.0))
    Nav2ActivationGate._amcl_pose_callback(gate, _pose_message(10.0))
    assert 'stable map->odom after current-epoch /amcl_pose' in \
        Nav2ActivationGate._missing_readiness_requirements(gate, now)

    tracker.observe_transform(0.0, 0.0, 0.0, 10.0, now)
    Nav2ActivationGate._observe_amcl_transform(
        gate, 0.0, 0.0, 0.0, 10.0, now)
    assert 'stable map->odom transform' in \
        Nav2ActivationGate._missing_readiness_requirements(gate, now)

    now += 0.21
    tracker.mark_clock(10.21, now)
    tracker.mark_scan(10.21, now)
    tracker.mark_odom(10.21, now)
    tracker.observe_transform(0.0, 0.0, 0.0, 10.21, now)
    gate._clock_stamp_s = 10.21
    gate._amcl_pose_stamp_s = 10.21
    gate._amcl_pose_received_at = now
    Nav2ActivationGate._observe_amcl_transform(
        gate, 0.0, 0.0, 0.0, 10.21, now)
    assert Nav2ActivationGate._missing_readiness_requirements(gate, now) == []

    now += 0.01
    tracker.mark_clock(10.22, now)
    tracker.mark_scan(10.22, now)
    tracker.mark_odom(10.22, now)
    tracker.observe_transform(1.0, 0.0, 0.0, 10.22, now)
    gate._clock_stamp_s = 10.22
    gate._amcl_pose_stamp_s = 10.22
    gate._amcl_pose_received_at = now
    Nav2ActivationGate._observe_amcl_transform(
        gate, 1.0, 0.0, 0.0, 10.22, now)
    assert 'stable map->odom transform' in \
        Nav2ActivationGate._missing_readiness_requirements(gate, now)


def test_reset_stop_gate_status_tracks_current_release_generation():
    fatals = []
    gate = SimpleNamespace(
        _state_query_lock=threading.RLock(),
        _stop_gate_generation=None,
        _stop_gate_eligible_generation=None,
        _stop_gate_held=True,
        _set_fatal=fatals.append,
    )
    Nav2ActivationGate._stop_gate_status_callback(
        gate,
        SimpleNamespace(data=(
            '{"eligible_generation":4,"generation":4,'
            '"held":true,"reason":"reset_complete"}'
        )),
    )
    assert fatals == []
    assert gate._stop_gate_generation == 4
    assert gate._stop_gate_eligible_generation == 4
    assert gate._stop_gate_held is True


def test_stale_stop_gate_release_completion_cannot_release_new_epoch():
    old_token = object()
    new_token = object()
    finalized = []
    gate = SimpleNamespace(
        _state_query_lock=threading.RLock(),
        _stop_gate_generation=5,
        _stop_gate_release_token=new_token,
        _stop_gate_release_in_flight=True,
        _stop_gate_held=True,
        _stop_gate_eligible_generation=5,
        _set_fatal=lambda reason: (_ for _ in ()).throw(
            AssertionError(reason)),
        _finalize_active=lambda **kwargs: finalized.append(kwargs),
    )
    response = SimpleNamespace(results=[SimpleNamespace(successful=True)])
    Nav2ActivationGate._stop_gate_release_done(
        gate,
        _Future(response=response),
        generation=4,
        recovered=True,
        token=old_token,
    )
    assert gate._stop_gate_held is True
    assert gate._stop_gate_release_token is new_token
    assert finalized == []


def test_current_stop_gate_release_failure_is_fatal_and_remains_held():
    token = object()
    fatals = []
    gate = SimpleNamespace(
        _state_query_lock=threading.RLock(),
        _stop_gate_generation=6,
        _stop_gate_release_token=token,
        _stop_gate_release_in_flight=True,
        _stop_gate_held=True,
        _stop_gate_eligible_generation=6,
        _set_fatal=fatals.append,
        _finalize_active=lambda **kwargs: (_ for _ in ()).throw(
            AssertionError(kwargs)),
    )
    response = SimpleNamespace(
        results=[SimpleNamespace(successful=False, reason='mismatch')]
    )
    Nav2ActivationGate._stop_gate_release_done(
        gate,
        _Future(response=response),
        generation=6,
        recovered=True,
        token=token,
    )
    assert gate._stop_gate_held is True
    assert fatals and 'release failed' in fatals[0]
