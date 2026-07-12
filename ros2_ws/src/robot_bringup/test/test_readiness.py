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
    tracker.mark_scan(stamp_s, now)
    tracker.mark_odom(stamp_s, now)
    tracker.mark_map()


def test_gate_requires_nonzero_fresh_inputs_and_stable_transform():
    tracker = _tracker()
    _mark_inputs(tracker, 1.0)
    tracker.observe_transform(0.0, 0.0, 0.0, 10.0, 1.0)
    assert not tracker.ready(1.9)

    _mark_inputs(tracker, 2.0, 10.1)
    tracker.observe_transform(0.01, 0.0, 0.01, 10.1, 2.0)
    assert not tracker.ready(2.0)
    _mark_inputs(tracker, 2.5, 10.2)
    tracker.observe_transform(0.01, 0.0, 0.01, 10.2, 2.5)
    _mark_inputs(tracker, 3.0, 10.3)
    tracker.observe_transform(0.01, 0.0, 0.01, 10.3, 3.0)
    assert tracker.ready(3.0)


def test_gate_requires_a_latched_map_but_not_map_freshness():
    tracker = _tracker()
    tracker.mark_clock(10.0, 1.0)
    tracker.mark_scan(10.0, 1.0)
    tracker.mark_odom(10.0, 1.0)
    tracker.observe_transform(0.0, 0.0, 0.0, 10.0, 0.0)
    assert 'latched /map' in tracker.missing_requirements(1.0)

    tracker.mark_map()
    _mark_inputs(tracker, 2.0, 10.1)
    tracker.observe_transform(0.0, 0.0, 0.0, 10.1, 2.0)
    assert not tracker.ready(2.0)
    _mark_inputs(tracker, 2.5, 10.2)
    tracker.observe_transform(0.0, 0.0, 0.0, 10.2, 2.5)
    _mark_inputs(tracker, 3.0, 10.3)
    tracker.observe_transform(0.0, 0.0, 0.0, 10.3, 3.0)
    assert tracker.ready(3.0)


def test_zero_clock_and_stale_sensor_data_block_activation():
    tracker = _tracker()
    tracker.mark_clock(0.0, 1.0)
    assert not tracker.mark_scan(0.0, 1.0)
    assert not tracker.mark_odom(0.0, 1.0)
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
    tracker.mark_scan(20.0, 1.0)
    tracker.mark_odom(20.0, 1.0)
    tracker.observe_transform(0.0, 0.0, 0.0, 20.0, 1.0)

    tracker.mark_clock(21.0, 2.0)
    tracker.mark_scan(21.0, 2.0)
    tracker.mark_odom(21.0, 2.0)
    tracker.observe_transform(0.0, 0.0, 0.0, 20.0, 2.0)
    assert 'map->odom transform' in tracker.missing_requirements(2.0)


def test_transform_freshness_break_restarts_stability_window():
    tracker = _tracker()
    _mark_inputs(tracker, 0.0, 10.0)
    tracker.observe_transform(0.0, 0.0, 0.0, 10.0, 0.0)

    assert 'map->odom transform' in tracker.missing_requirements(0.6)
    assert tracker._tf_stable_since is None

    for now in (0.7, 1.1, 1.5):
        stamp = 10.0 + now
        _mark_inputs(tracker, now, stamp)
        tracker.observe_transform(0.0, 0.0, 0.0, stamp, now)
    assert not tracker.ready(1.5)

    _mark_inputs(tracker, 1.71, 11.71)
    tracker.observe_transform(0.0, 0.0, 0.0, 11.71, 1.71)
    assert tracker.ready(1.71)


def test_bounded_future_dated_localization_transform_is_fresh():
    tracker = _tracker()
    _mark_inputs(tracker, 0.0, 20.0)
    tracker.observe_transform(0.0, 0.0, 0.0, 20.1, 0.0)
    _mark_inputs(tracker, 1.0, 20.0)
    tracker.observe_transform(0.0, 0.0, 0.0, 20.2, 1.0)
    assert tracker.ready(1.0)

    tracker.observe_transform(0.0, 0.0, 0.0, 21.0, 1.0)
    assert 'map->odom transform' in tracker.missing_requirements(1.0)


def test_sim_time_reset_requires_a_new_transform_epoch():
    tracker = _tracker()
    _mark_inputs(tracker, 1.0, 30.0)
    tracker.observe_transform(0.0, 0.0, 0.0, 30.0, 1.0)
    event = tracker.mark_clock(0.1, 2.0)
    assert event.kind == 'rollback'
    assert event.epoch == 1
    missing = tracker.missing_requirements(2.0)
    assert 'fresh /scan' in missing
    assert 'fresh /odom' in missing
    assert 'map->odom transform' in tracker.missing_requirements(2.0)
    assert not tracker.mark_scan(30.1, 2.0)
    assert not tracker.mark_odom(30.1, 2.0)
    assert tracker.mark_scan(0.1, 2.0)
    assert tracker.mark_odom(0.1, 2.0)
    tracker.observe_transform(0.0, 0.0, 0.0, 0.1, 2.0)
    assert tracker._tf_stamp_s == pytest.approx(0.1)


def test_large_forward_clock_jump_starts_a_new_epoch():
    tracker = _tracker()
    _mark_inputs(tracker, 1.0, 10.0)
    tracker.observe_transform(0.0, 0.0, 0.0, 10.0, 1.0)

    event = tracker.mark_clock(20.0, 2.0)
    assert event.kind == 'forward_jump'
    assert event.previous_stamp_s == pytest.approx(10.0)
    assert event.stamp_s == pytest.approx(20.0)
    assert tracker.epoch == 1
    assert {'fresh /scan', 'fresh /odom', 'map->odom transform'} \
        <= set(tracker.missing_requirements(2.0))


def test_dropped_clock_samples_do_not_look_like_a_forward_jump():
    tracker = _tracker()
    tracker.mark_clock(10.0, 1.0)

    event = tracker.mark_clock(20.0, 11.0)

    assert event is None
    assert tracker.epoch == 0


def test_reset_event_starts_new_epoch_even_when_clock_is_monotonic():
    tracker = _tracker()
    _mark_inputs(tracker, 1.0, 10.0)
    tracker.observe_transform(0.0, 0.0, 0.0, 10.0, 1.0)

    event = tracker.mark_reset()
    assert event.kind == 'reset'
    assert event.previous_stamp_s == pytest.approx(10.0)
    assert event.stamp_s == pytest.approx(10.0)
    assert tracker.epoch == 1
    missing = set(tracker.missing_requirements(1.0))
    assert {'fresh /scan', 'fresh /odom', 'map->odom transform'} <= missing
    assert 'latched /map' not in missing


def test_invalid_readiness_config_is_rejected():
    with pytest.raises(ValueError):
        ReadinessConfig(freshness_timeout=0.0)
