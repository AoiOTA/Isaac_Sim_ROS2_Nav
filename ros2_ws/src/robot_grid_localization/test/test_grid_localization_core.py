import math

import pytest

from robot_grid_localization.core import (
    LocalizationGate,
    map_to_odom,
    RigidTransform,
    STATUS_KEYS,
    status_values,
)


def _yaw_transform(x, y, yaw):
    return RigidTransform(
        x, y, 0.0, 0.0, 0.0,
        math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def test_map_to_odom_formula_preserves_map_to_base_chain():
    map_base = _yaw_transform(5.0, 2.0, math.pi / 2.0)
    odom_base = _yaw_transform(1.0, 0.0, math.pi / 2.0)
    correction = map_to_odom(map_base, odom_base)
    assert correction.x == pytest.approx(4.0)
    assert correction.y == pytest.approx(2.0)
    assert correction.z == pytest.approx(0.0)
    assert correction.qz == pytest.approx(0.0, abs=1.0e-12)
    assert correction.qw == pytest.approx(1.0)


def test_trigger_generation_is_monotonic_and_only_one_is_pending():
    gate = LocalizationGate()
    first = gate.begin_trigger(100)
    duplicate = gate.begin_trigger(101)
    assert first.accepted is True
    assert first.generation == 1
    assert duplicate.accepted is False
    assert duplicate.reason == 'request_already_pending'
    assert duplicate.generation == 1
    assert gate.pending_generation == 1
    assert gate.trigger_stamp_ns == 100

    gate.classify_result(200, finite=True, has_same_stamp_tf=True)
    second = gate.begin_trigger(300)
    assert second.accepted is True
    assert second.generation == 2


def test_result_before_trigger_is_rejected_without_pending_generation():
    decision = LocalizationGate().classify_result(
        100, finite=True, has_same_stamp_tf=True)
    assert decision.accepted is False
    assert decision.reason == 'no_pending_generation'


@pytest.mark.parametrize(
    ('stamp', 'finite', 'has_tf', 'reason'),
    [
        (0, True, True, 'invalid_result_stamp'),
        (100, False, True, 'non_finite_result'),
        (100, True, False, 'missing_same_stamp_odom_to_base'),
    ],
)
def test_invalid_result_is_rejected_and_consumes_pending_generation(
        stamp, finite, has_tf, reason):
    gate = LocalizationGate()
    gate.begin_trigger(10)
    decision = gate.classify_result(stamp, finite, has_tf)
    assert decision.accepted is False
    assert decision.reason == reason
    assert gate.pending_generation is None


def test_result_older_than_current_trigger_is_rejected():
    gate = LocalizationGate()
    gate.begin_trigger(10)
    accepted = gate.classify_result(100, finite=True, has_same_stamp_tf=True)
    assert accepted.accepted is True
    gate.begin_trigger(200)
    stale = gate.classify_result(100, finite=True, has_same_stamp_tf=True)
    assert stale.accepted is False
    assert stale.reason == 'result_before_current_trigger'


def test_late_result_after_proxy_failure_cannot_enter_next_generation():
    gate = LocalizationGate()
    gate.begin_trigger(100)
    gate.reject_pending('grid_trigger_proxy_error')
    retry = gate.begin_trigger(300)
    assert retry.generation == 2

    late_first = gate.classify_result(
        200, finite=True, has_same_stamp_tf=True)
    assert late_first.accepted is False
    assert late_first.reason == 'result_before_current_trigger'
    assert gate.pending_generation is None


def test_pending_timeout_is_terminal_and_allows_a_fresh_trigger():
    gate = LocalizationGate()
    gate.begin_trigger(100)
    assert gate.expire_pending(1_099, 1_000) is None
    timed_out = gate.expire_pending(1_100, 1_000)
    assert timed_out is not None
    assert timed_out.accepted is False
    assert timed_out.reason == 'localization_timeout'
    assert gate.pending_generation is None

    retry = gate.begin_trigger(1_200)
    assert retry.accepted is True
    assert retry.generation == 2
    late_first = gate.classify_result(
        1_150, finite=True, has_same_stamp_tf=True)
    assert late_first.accepted is False
    assert late_first.reason == 'result_before_current_trigger'


def test_accepted_status_has_the_frozen_keys_and_correction():
    gate = LocalizationGate()
    gate.begin_trigger(1_000_000_000)
    decision = gate.classify_result(
        1_500_000_000, finite=True, has_same_stamp_tf=True)
    correction = _yaw_transform(0.25, -0.5, 0.1)
    values = dict(status_values(decision, 'ACCEPTED', correction, 0.5))
    assert tuple(values) == STATUS_KEYS
    assert values['generation'] == '1'
    assert values['state'] == 'ACCEPTED'
    assert values['accepted'] == 'true'
    assert values['reason'] == 'accepted'
    assert float(values['correction_x_m']) == pytest.approx(0.25)
    assert float(values['correction_y_m']) == pytest.approx(-0.5)
    assert float(values['correction_yaw_rad']) == pytest.approx(0.1)
    assert float(values['latency_s']) == pytest.approx(0.5)
