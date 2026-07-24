import pytest

from robot_experiments.experiment_runner import (
    CommandSample,
    ExperimentRunner,
    OdometrySample,
)


def test_motion_quality_measures_reverse_curves_and_turn_reversals():
    samples = [
        CommandSample(0.30, 0.60, 0.0),
        CommandSample(0.30, 0.60, 0.1),
        CommandSample(-0.20, -0.50, 0.2),
        CommandSample(-0.20, -0.50, 0.3),
    ]
    metrics = ExperimentRunner._motion_quality_metrics(samples)
    assert metrics["translated_distance_m"] == pytest.approx(0.08)
    assert metrics["reverse_distance_m"] == pytest.approx(0.02)
    assert metrics["reverse_distance_fraction"] == pytest.approx(0.25)
    assert metrics["curved_distance_fraction"] == pytest.approx(1.0)
    assert metrics["angular_direction_changes"] == 1
    assert metrics["stopped_time_fraction"] == pytest.approx(0.0)


def test_motion_quality_ignores_large_timestamp_gaps():
    samples = [
        CommandSample(0.50, 1.00, 0.0),
        CommandSample(-0.50, -1.00, 1.0),
    ]
    metrics = ExperimentRunner._motion_quality_metrics(samples)
    assert metrics["observed_duration_sec"] == 0.0
    assert metrics["translated_distance_m"] == 0.0
    assert metrics["maximum_linear_acceleration_mps2"] == 0.0


def test_same_direction_overtake_requires_lateral_bypass_and_passing():
    ground_truth = [
        OdometrySample(-0.80, -1.00, 0.0, 0.5, 0.0, 1.00, 0.0),
        OdometrySample(-0.82, -0.20, 0.0, 0.5, 0.0, 1.10, 0.0),
        OdometrySample(-0.45, 0.70, 0.0, 0.5, 0.0, 1.20, 0.0),
    ]
    actor = [
        {"id": "same_direction_slow_actor", "state": "moving", "stamp_s": 1.00, "position": [-0.45, -0.60, 0.5]},
        {"id": "same_direction_slow_actor", "state": "moving", "stamp_s": 1.10, "position": [-0.45, -0.10, 0.5]},
        {"id": "same_direction_slow_actor", "state": "moving", "stamp_s": 1.20, "position": [-0.45, 0.20, 0.5]},
    ]

    metrics = ExperimentRunner._same_direction_overtake_metrics(
        ground_truth, actor, "same_direction_slow_actor"
    )

    assert metrics["lateral_bypass_seen"]
    assert metrics["passed_while_moving"]
    assert metrics["passed_before_actor_yielded_right"]
    assert metrics["complete"]


def test_same_direction_waiting_is_not_an_overtake():
    ground_truth = [
        OdometrySample(-0.45, -1.10, 0.0, 0.0, 0.0, 1.00, 0.0),
        OdometrySample(-0.45, -1.10, 0.0, 0.0, 0.0, 1.10, 0.0),
    ]
    actor = [
        {"id": "same_direction_slow_actor", "state": "moving", "stamp_s": 1.00, "position": [-0.45, -0.60, 0.5]},
        {"id": "same_direction_slow_actor", "state": "moving", "stamp_s": 1.10, "position": [-0.45, -0.58, 0.5]},
    ]

    metrics = ExperimentRunner._same_direction_overtake_metrics(
        ground_truth, actor, "same_direction_slow_actor"
    )

    assert not metrics["lateral_bypass_seen"]
    assert not metrics["passed_while_moving"]
    assert not metrics["passed_before_actor_yielded_right"]
    assert not metrics["complete"]


def test_local_bypass_requires_passing_to_the_actor_right():
    ground_truth = [
        OdometrySample(0.20, 0.35, 0.0, 0.4, 0.0, 1.00, 0.0),
        OdometrySample(0.25, 0.95, 0.0, 0.4, 0.0, 1.10, 0.0),
    ]
    actor = [
        {"id": "local_bypass_actor", "state": "moving", "stamp_s": 1.00, "position": [-0.20, 0.45, 0.5]},
        {"id": "local_bypass_actor", "state": "moving", "stamp_s": 1.10, "position": [-0.10, 0.45, 0.5]},
    ]

    metrics = ExperimentRunner._local_right_bypass_metrics(
        ground_truth, actor, "local_bypass_actor"
    )

    assert metrics["right_side_bypass_seen"]
    assert metrics["passed_while_moving"]
    assert metrics["complete"]


def test_local_bypass_accepts_a_pass_after_the_planned_park():
    ground_truth = [
        OdometrySample(0.30, 0.40, 0.0, 0.3, 0.0, 1.00, 0.0),
        OdometrySample(0.32, 0.92, 0.0, 0.3, 0.0, 1.10, 0.0),
    ]
    actor = [
        {"id": "local_bypass_actor", "state": "moving", "stamp_s": 1.00, "position": [-0.20, 0.45, 0.5]},
        {"id": "local_bypass_actor", "state": "parked", "stamp_s": 1.10, "position": [-0.10, 0.45, 0.5]},
    ]

    metrics = ExperimentRunner._local_right_bypass_metrics(
        ground_truth, actor, "local_bypass_actor"
    )

    assert metrics["planned_park_seen"]
    assert metrics["right_side_bypass_seen"]
    assert not metrics["passed_while_moving"]
    assert metrics["passed_after_planned_park"]
    assert metrics["complete"]


def test_g2_g3_exit_requires_following_then_left_exit_turn():
    ground_truth = [
        OdometrySample(-0.42, 1.60, 0.0, 0.4, 0.0, 1.00, 0.0),
        OdometrySample(-0.85, -0.55, 0.0, 0.4, 0.0, 1.10, 0.0),
    ]
    actor = [
        {"id": "g2_g3_exit_actor", "state": "moving", "stamp_s": 1.00, "position": [-0.40, 1.00, 0.5]},
        {"id": "g2_g3_exit_actor", "state": "parked", "stamp_s": 1.10, "position": [-0.40, -0.70, 0.5]},
    ]

    metrics = ExperimentRunner._g2_g3_exit_metrics(
        ground_truth, actor, "g2_g3_exit_actor"
    )

    assert metrics["continuous_follow_seen"]
    assert metrics["outlet_left_turn_seen"]
    assert metrics["complete"]


def test_g5_g1_crossing_requires_left_side_pass_while_actor_exists():
    ground_truth = [
        OdometrySample(-0.85, -1.42, 0.0, 0.4, 0.0, 1.00, 0.0),
        OdometrySample(-0.86, -1.90, 0.0, 0.4, 0.0, 1.10, 0.0),
    ]
    actor = [
        {"id": "g5_g1_crossing_actor", "state": "moving", "stamp_s": 1.00, "position": [-0.45, -1.45, 0.5]},
        {"id": "g5_g1_crossing_actor", "state": "parked", "stamp_s": 1.10, "position": [-0.20, -1.45, 0.5]},
    ]

    metrics = ExperimentRunner._g5_g1_left_bypass_metrics(
        ground_truth, actor, "g5_g1_crossing_actor"
    )

    assert metrics["left_side_bypass_seen"]
    assert metrics["passed_while_present"]
    assert metrics["complete"]
