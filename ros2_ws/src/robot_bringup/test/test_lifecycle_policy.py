import pytest

from robot_bringup.lifecycle_policy import duplicate_names
from robot_bringup.lifecycle_policy import lifecycle_decision
from robot_bringup.lifecycle_policy import LifecycleAction
from robot_bringup.lifecycle_policy import RetryPolicy


@pytest.mark.parametrize(
    ('state', 'action'),
    [
        ('unconfigured', LifecycleAction.STARTUP),
        ('inactive', LifecycleAction.RESUME),
        ('active', LifecycleAction.ALREADY_ACTIVE),
    ],
)
def test_uniform_stable_states_have_idempotent_manager_actions(state, action):
    decision = lifecycle_decision({
        'controller_server': state,
        'planner_server': state,
        'bt_navigator': state,
    })
    assert decision.action is action


@pytest.mark.parametrize(
    'state',
    ['configuring', 'activating', 'deactivating', 'cleaningup', 'shuttingdown'],
)
def test_transitional_states_wait_without_requesting_another_transition(state):
    decision = lifecycle_decision({
        'controller_server': state,
        'planner_server': 'inactive',
    })
    assert decision.action is LifecycleAction.WAIT
    assert 'transition in progress' in decision.reason


@pytest.mark.parametrize('state', ['finalized', 'errorprocessing', 'unknown', ''])
def test_terminal_or_unknown_states_fail_with_node_diagnostics(state):
    decision = lifecycle_decision({
        'controller_server': state,
        'planner_server': 'inactive',
    })
    assert decision.action is LifecycleAction.FAIL
    assert f'controller_server={state}' in decision.reason


def test_mixed_stable_states_are_not_blindly_retransitioned():
    decision = lifecycle_decision({
        'controller_server': 'active',
        'planner_server': 'inactive',
    })
    assert decision.action is LifecycleAction.FAIL
    assert 'unsafe mixed' in decision.reason


def test_empty_snapshot_fails_and_duplicate_fqns_are_reported():
    assert lifecycle_decision({}).action is LifecycleAction.FAIL
    assert duplicate_names(['/a', '/b', '/a', '/b', '/c']) == ['/a', '/b']


def test_retry_policy_is_finite_and_exponentially_bounded():
    policy = RetryPolicy(
        max_attempts=3,
        initial_backoff=0.5,
        maximum_backoff=1.0,
    )
    assert [policy.delay_after_failure(attempt) for attempt in (1, 2, 3)] == [
        0.5,
        1.0,
        1.0,
    ]
    assert policy.can_retry(0)
    assert policy.can_retry(2)
    assert not policy.can_retry(3)


@pytest.mark.parametrize(
    'kwargs',
    [
        {'max_attempts': 0},
        {'initial_backoff': 0.0},
        {'initial_backoff': 2.0, 'maximum_backoff': 1.0},
    ],
)
def test_invalid_retry_policy_is_rejected(kwargs):
    with pytest.raises(ValueError):
        RetryPolicy(**kwargs)
