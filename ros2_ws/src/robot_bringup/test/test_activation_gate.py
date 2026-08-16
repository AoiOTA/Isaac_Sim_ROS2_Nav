from pathlib import Path
import threading
from types import SimpleNamespace

from nav2_msgs.srv import ManageLifecycleNodes
import yaml

from robot_bringup.activation_gate import Nav2ActivationGate
from robot_bringup.lifecycle_policy import RetryPolicy


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
        'reseed',
        'waiting_readiness',
        'resume_query',
    ):
        assert stage in source
    assert "CancelGoal, '/navigate_to_pose/_action/cancel_goal'" in source
    assert "'/global_costmap/clear_entirely_global_costmap'" in source
    assert "'/local_costmap/clear_entirely_local_costmap'" in source
    assert "self.declare_parameter('immutable_map_node', 'map_server')" in source
    assert 'Repairing immutable map lifecycle' in source
    assert 'except (KeyboardInterrupt, ExternalShutdownException):' in source
    assert 'if node is not None:\n            node.destroy_node()' in source


class _LocalizationFuture:
    def __init__(self, response=None, exception=None):
        self._response = response
        self._exception = exception
        self._callbacks = []

    def result(self):
        if self._exception is not None:
            raise self._exception
        return self._response

    def add_done_callback(self, callback):
        self._callbacks.append(callback)

    def fire(self):
        for callback in self._callbacks:
            callback(self)


class _LocalizationStateClient:
    def __init__(self, label='active', ready=True):
        self.label = label
        self.ready = ready
        self.calls = 0
        self.futures = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        del request
        self.calls += 1
        future = _LocalizationFuture(response=SimpleNamespace(
            current_state=SimpleNamespace(label=self.label)))
        self.futures.append(future)
        return future


def _localization_gate(backend='amcl', **client_kwargs):
    logger = _Logger()
    client = _LocalizationStateClient(**client_kwargs)
    gate = SimpleNamespace(
        _localization_backend=backend,
        _localization_node='amcl',
        _localization_active=False,
        _localization_query_in_flight=False,
        _localization_query_token=None,
        _localization_state_client=client,
        _state_query_lock=threading.RLock(),
        _generation=1,
        _last_failure='',
        get_logger=lambda: logger,
    )
    gate._localization_state_done = lambda *args, **kwargs: (
        Nav2ActivationGate._localization_state_done(
            gate, *args, **kwargs))
    return gate, client, logger


def test_non_amcl_backends_skip_the_localization_lifecycle_check():
    for backend in ('', 'ideal', 'slam_toolbox'):
        gate, client, _ = _localization_gate(backend=backend)
        assert Nav2ActivationGate._localization_readiness(gate) == []
        assert client.calls == 0


def test_amcl_backend_waits_for_the_get_state_service():
    gate, client, _ = _localization_gate(ready=False)
    assert Nav2ActivationGate._localization_readiness(gate) == \
        ['/amcl/get_state']
    assert client.calls == 0


def test_amcl_active_response_latches_localization_readiness():
    gate, client, logger = _localization_gate(label='active')
    assert Nav2ActivationGate._localization_readiness(gate) == \
        ['AMCL lifecycle query']
    # A second poll while the query is in flight does not re-issue it.
    assert Nav2ActivationGate._localization_readiness(gate) == \
        ['AMCL lifecycle query']
    assert client.calls == 1

    client.futures[0].fire()
    assert gate._localization_active
    assert Nav2ActivationGate._localization_readiness(gate) == []
    assert any(
        'lifecycle is ACTIVE' in message for _, message in logger.messages)


def test_amcl_inactive_response_keeps_waiting_and_requeries():
    gate, client, _ = _localization_gate(label='inactive')
    Nav2ActivationGate._localization_readiness(gate)
    client.futures[0].fire()
    assert not gate._localization_active
    assert "'inactive'" in gate._last_failure

    # The next poll issues a fresh query instead of latching the failure.
    client.label = 'active'
    assert Nav2ActivationGate._localization_readiness(gate) == \
        ['AMCL lifecycle query']
    assert client.calls == 2
    client.futures[1].fire()
    assert gate._localization_active


def test_amcl_get_state_exception_is_recorded_without_latching():
    gate, client, logger = _localization_gate()
    Nav2ActivationGate._localization_readiness(gate)
    token = gate._localization_query_token
    Nav2ActivationGate._localization_state_done(
        gate,
        _LocalizationFuture(exception=RuntimeError('service disappeared')),
        generation=1,
        token=token,
    )
    assert not gate._localization_active
    assert not gate._localization_query_in_flight
    assert 'service disappeared' in gate._last_failure
    assert any(
        'service disappeared' in message
        for level, message in logger.messages if level == 'warning')


def test_stale_localization_query_cannot_latch_a_new_epoch():
    gate, client, _ = _localization_gate(label='active')
    Nav2ActivationGate._localization_readiness(gate)
    gate._generation = 2  # epoch invalidation while the query is in flight
    client.futures[0].fire()
    assert not gate._localization_active


def test_gate_declares_the_localization_backend_contract():
    source = (
        PACKAGE_ROOT / 'robot_bringup' / 'activation_gate.py'
    ).read_text()
    assert "self.declare_parameter('localization_backend', '')" in source
    assert "self.declare_parameter('localization_node', 'amcl')" in source
    assert "f'/{self._localization_node}/get_state'" in source
    parameters = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'activation_gate.yaml').read_text())
    gate_parameters = parameters['nav2_activation_gate']['ros__parameters']
    assert gate_parameters['localization_backend'] == ''
    assert gate_parameters['localization_node'] == 'amcl'
