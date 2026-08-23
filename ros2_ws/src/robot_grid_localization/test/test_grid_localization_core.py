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

    selected = gate.observe_scan(200)
    assert selected.expected_result_stamp_ns == 200
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
        (100, False, True, 'non_finite_result'),
        (100, True, False, 'missing_same_stamp_odom_to_base'),
    ],
)
def test_invalid_result_is_rejected_and_consumes_pending_generation(
        stamp, finite, has_tf, reason):
    gate = LocalizationGate()
    gate.begin_trigger(10)
    gate.observe_scan(stamp)
    decision = gate.classify_result(stamp, finite, has_tf)
    assert decision.accepted is False
    assert decision.reason == reason
    assert gate.pending_generation is None


def test_unexpected_result_does_not_consume_pending_then_exact_is_accepted():
    gate = LocalizationGate()
    gate.begin_trigger(10)
    gate.observe_scan(100)
    unexpected = gate.classify_result(
        99, finite=True, has_same_stamp_tf=True)
    assert unexpected.accepted is False
    assert unexpected.reason == 'unexpected_result_stamp'
    assert unexpected.expected_result_stamp_ns == 100
    assert gate.pending_generation == 1

    accepted = gate.classify_result(
        100, finite=True, has_same_stamp_tf=True)
    assert accepted.accepted is True


def test_buffered_pretrigger_scan_is_not_selected():
    gate = LocalizationGate()
    assert gate.observe_scan(100) is None
    gate.begin_trigger(200)
    assert gate.observe_scan(0) is None
    assert gate.observe_scan(100) is None
    assert gate.observe_scan(99) is None
    selected = gate.observe_scan(101)
    assert selected is not None
    assert selected.expected_result_stamp_ns == 101


def test_each_generation_selects_and_forwards_only_one_scan_stamp():
    gate = LocalizationGate()
    gate.begin_trigger(100)
    first = gate.observe_scan(200)
    second = gate.observe_scan(201)
    assert first is not None
    assert second is None
    assert gate.expected_result_stamp_ns == 200


def test_result_before_scan_selection_does_not_consume_pending():
    gate = LocalizationGate()
    gate.begin_trigger(100)
    result = gate.classify_result(
        200, finite=True, has_same_stamp_tf=True)
    assert result.accepted is False
    assert result.reason == 'waiting_for_selected_scan'
    assert gate.pending_generation == 1


def test_no_scan_timeout_is_terminal_and_allows_a_fresh_trigger():
    gate = LocalizationGate()
    gate.begin_trigger(100)
    assert gate.expire_pending(1_099, 1_000) is None
    timed_out = gate.expire_pending(1_100, 1_000)
    assert timed_out is not None
    assert timed_out.accepted is False
    assert timed_out.reason == 'scan_timeout'
    assert gate.pending_generation is None

    retry = gate.begin_trigger(1_200)
    assert retry.accepted is True
    assert retry.generation == 2


def test_no_result_timeout_clears_selected_stamp_and_allows_retry():
    gate = LocalizationGate()
    gate.begin_trigger(100)
    gate.observe_scan(200)
    timed_out = gate.expire_pending(1_100, 1_000)
    assert timed_out.reason == 'result_timeout'
    assert timed_out.expected_result_stamp_ns == 200
    assert gate.pending_generation is None
    assert gate.expected_result_stamp_ns == 0
    assert gate.begin_trigger(1_200).generation == 2


def test_accepted_status_has_the_frozen_keys_and_correction():
    gate = LocalizationGate()
    gate.begin_trigger(1_000_000_000)
    gate.observe_scan(1_500_000_000)
    decision = gate.classify_result(
        1_500_000_000, finite=True, has_same_stamp_tf=True)
    correction = _yaw_transform(0.25, -0.5, 0.1)
    values = dict(status_values(decision, 'ACCEPTED', correction, 0.5))
    assert tuple(values) == STATUS_KEYS
    assert values['generation'] == '1'
    assert values['state'] == 'ACCEPTED'
    assert values['accepted'] == 'true'
    assert values['reason'] == 'accepted'
    assert values['expected_result_stamp_ns'] == '1500000000'
    assert float(values['correction_x_m']) == pytest.approx(0.25)
    assert float(values['correction_y_m']) == pytest.approx(-0.5)
    assert float(values['correction_yaw_rad']) == pytest.approx(0.1)
    assert float(values['latency_s']) == pytest.approx(0.5)
