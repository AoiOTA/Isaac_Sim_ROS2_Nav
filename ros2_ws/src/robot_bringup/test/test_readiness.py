import pytest

from robot_bringup.readiness import ReadinessConfig, ReadinessTracker


def _tracker():
    return ReadinessTracker(ReadinessConfig(
        freshness_timeout=0.5,
        tf_stable_duration=1.0,
        tf_translation_tolerance=0.05,
        tf_yaw_tolerance=0.05,
    ))


def _mark_inputs(tracker, now, stamp_s=10.0):
    tracker.mark_clock(stamp_s, now)
    tracker.mark_scan(now)
    tracker.mark_odom(now)
    tracker.mark_map()


def test_gate_requires_nonzero_fresh_inputs_and_stable_transform():
    tracker = _tracker()
    _mark_inputs(tracker, 1.0)
    tracker.observe_transform(0.0, 0.0, 0.0, 10.0, 1.0)
    assert not tracker.ready(1.9)

    _mark_inputs(tracker, 2.0, 10.1)
    tracker.observe_transform(0.01, 0.0, 0.01, 10.1, 2.0)
    assert tracker.ready(2.0)


def test_gate_requires_a_latched_map_but_not_map_freshness():
    tracker = _tracker()
    tracker.mark_clock(10.0, 1.0)
    tracker.mark_scan(1.0)
    tracker.mark_odom(1.0)
    tracker.observe_transform(0.0, 0.0, 0.0, 10.0, 0.0)
    assert 'latched /map' in tracker.missing_requirements(1.0)

    tracker.mark_map()
    _mark_inputs(tracker, 2.0, 10.1)
    tracker.observe_transform(0.0, 0.0, 0.0, 10.1, 2.0)
    assert tracker.ready(2.0)


def test_zero_clock_and_stale_sensor_data_block_activation():
    tracker = _tracker()
    tracker.mark_clock(0.0, 1.0)
    tracker.mark_scan(1.0)
    tracker.mark_odom(1.0)
    tracker.observe_transform(0.0, 0.0, 0.0, 0.1, 0.0)
    assert 'non-zero /clock' in tracker.missing_requirements(1.0)
    assert 'fresh /scan' in tracker.missing_requirements(2.0)
    assert 'fresh /odom' in tracker.missing_requirements(2.0)


def test_transform_jump_restarts_stability_window():
    tracker = _tracker()
    _mark_inputs(tracker, 0.0)
    tracker.observe_transform(0.0, 0.0, 0.0, 10.0, 0.0)
    tracker.mark_clock(10.1, 0.8)
    tracker.observe_transform(0.20, 0.0, 0.0, 10.1, 0.8)
    _mark_inputs(tracker, 1.7, 10.2)
    tracker.observe_transform(0.20, 0.0, 0.0, 10.2, 1.7)
    assert not tracker.ready(1.7)
    _mark_inputs(tracker, 1.81, 10.3)
    tracker.observe_transform(0.20, 0.0, 0.0, 10.3, 1.81)
    assert tracker.ready(1.81)


def test_cached_or_sim_time_stale_transform_cannot_refresh_gate():
    tracker = _tracker()
    tracker.mark_clock(20.0, 1.0)
    tracker.mark_scan(1.0)
    tracker.mark_odom(1.0)
    tracker.observe_transform(0.0, 0.0, 0.0, 20.0, 1.0)

    tracker.mark_clock(21.0, 2.0)
    tracker.mark_scan(2.0)
    tracker.mark_odom(2.0)
    tracker.observe_transform(0.0, 0.0, 0.0, 20.0, 2.0)
    assert 'map->odom transform' in tracker.missing_requirements(2.0)


def test_bounded_future_dated_localization_transform_is_fresh():
    tracker = _tracker()
    tracker.observe_transform(0.0, 0.0, 0.0, 20.1, 0.0)
    _mark_inputs(tracker, 1.0, 20.0)
    tracker.observe_transform(0.0, 0.0, 0.0, 20.2, 1.0)
    assert tracker.ready(1.0)

    tracker.observe_transform(0.0, 0.0, 0.0, 21.0, 1.0)
    assert 'map->odom transform' in tracker.missing_requirements(1.0)


def test_sim_time_reset_requires_a_new_transform_epoch():
    tracker = _tracker()
    tracker.mark_clock(30.0, 1.0)
    tracker.observe_transform(0.0, 0.0, 0.0, 30.0, 1.0)
    tracker.mark_clock(0.1, 2.0)
    assert 'map->odom transform' in tracker.missing_requirements(2.0)
    tracker.observe_transform(0.0, 0.0, 0.0, 0.1, 2.0)
    assert tracker._tf_stamp_s == pytest.approx(0.1)


def test_invalid_readiness_config_is_rejected():
    with pytest.raises(ValueError):
        ReadinessConfig(freshness_timeout=0.0)
