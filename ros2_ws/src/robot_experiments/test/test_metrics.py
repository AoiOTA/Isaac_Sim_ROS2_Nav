import math

import pytest

from robot_experiments.metrics import (
    PLAN_DYNAMIC_SUCCESS_MIN_PERCENT,
    PLAN_INCREMENTAL_IMPROVEMENT_MIN_PERCENT,
    PLAN_NAVIGATION_SUCCESS_MIN_PERCENT,
    PLAN_PATH_DEVIATION_MAX_PERCENT,
    PLAN_STATIC_SUCCESS_MIN_PERCENT,
    SingleRunObservation,
    angular_error_statistics,
    dynamic_avoidance_rate,
    error_statistics,
    evaluate_single_run,
    incremental_time_improvement_percent,
    navigation_success_rate,
    path_length,
    path_length_deviation_percent,
    percentile,
    static_avoidance_rate,
    success_rate_percent,
    threshold_summary,
    translation_error_statistics,
    wrap_angle,
)


def successful_observation(**overrides):
    values = {
        "nav2_succeeded": True,
        "ground_truth_available": True,
        "ground_truth_position_error_m": 0.25,
        "ground_truth_orientation_error_rad": math.radians(10.0),
        "orientation_required": True,
        "collision_detected": False,
        "localization_lost": False,
        "tf_interrupted": False,
        "timed_out": False,
        "collision_monitor_locked": False,
        "final_linear_speed_mps": 0.02,
        "final_angular_speed_radps": 0.05,
        "safety_observability_complete": True,
    }
    values.update(overrides)
    return SingleRunObservation(**values)


@pytest.mark.parametrize(
    ("angle", "expected"),
    [
        (0.0, 0.0),
        (math.pi, -math.pi),
        (-math.pi, -math.pi),
        (3.0 * math.pi, -math.pi),
        (-3.0 * math.pi, -math.pi),
    ],
)
def test_wrap_angle_half_open_interval(angle, expected):
    assert wrap_angle(angle) == pytest.approx(expected)


def test_wrap_angle_rejects_non_finite():
    with pytest.raises(ValueError, match="finite"):
        wrap_angle(math.inf)


def test_path_length_uses_ground_truth_polyline_geometry():
    assert path_length([(0.0, 0.0), (3.0, 4.0), (6.0, 4.0)]) == pytest.approx(8.0)
    assert path_length([]) == 0.0


def test_translation_error_statistics_include_rmse_max_and_p95():
    stats = translation_error_statistics(
        [(0.0, 0.0), (4.0, 0.0), (0.0, 3.0)],
        [(0.0, 0.0), (1.0, 0.0), (0.0, 0.0)],
    )
    assert stats.rmse == pytest.approx(math.sqrt(6.0))
    assert stats.maximum == pytest.approx(3.0)
    assert stats.percentile_95 == pytest.approx(3.0)
    assert stats.sample_count == 3


def test_angular_error_statistics_wrap_at_pi():
    stats = angular_error_statistics(
        [math.radians(-179.0), math.radians(179.0)],
        [math.radians(179.0), math.radians(-179.0)],
    )
    assert stats.rmse == pytest.approx(math.radians(2.0))
    assert stats.maximum == pytest.approx(math.radians(2.0))


def test_error_statistics_and_percentile_interpolate():
    assert percentile([0.0, 10.0], 95.0) == pytest.approx(9.5)
    assert error_statistics([-1.0, 1.0]).rmse == pytest.approx(1.0)
    with pytest.raises(ValueError, match="at least one"):
        error_statistics([])


def test_path_length_deviation_and_incremental_improvement():
    assert path_length_deviation_percent(12.0, 10.0) == pytest.approx(20.0)
    assert path_length_deviation_percent(8.0, 10.0) == pytest.approx(20.0)
    assert incremental_time_improvement_percent(100.0, 70.0) == pytest.approx(30.0)
    with pytest.raises(ValueError, match="positive"):
        path_length_deviation_percent(1.0, 0.0)
    with pytest.raises(ValueError, match="positive"):
        incremental_time_improvement_percent(0.0, 0.0)


def test_success_rate_wrappers_return_percent():
    assert success_rate_percent(9, 10) == pytest.approx(90.0)
    assert navigation_success_rate(9, 10) == pytest.approx(90.0)
    assert static_avoidance_rate(19, 20) == pytest.approx(95.0)
    assert dynamic_avoidance_rate(9, 10) == pytest.approx(90.0)


@pytest.mark.parametrize(
    ("successes", "total", "error"),
    [(-1, 1, ValueError), (2, 1, ValueError), (0, 0, ValueError), (True, 1, TypeError)],
)
def test_success_rate_rejects_invalid_counts(successes, total, error):
    with pytest.raises(error):
        success_rate_percent(successes, total)


def test_plan_threshold_summary_is_exact():
    assert threshold_summary() == {
        "static_success_min_percent": PLAN_STATIC_SUCCESS_MIN_PERCENT,
        "dynamic_success_min_percent": PLAN_DYNAMIC_SUCCESS_MIN_PERCENT,
        "navigation_success_min_percent": PLAN_NAVIGATION_SUCCESS_MIN_PERCENT,
        "path_deviation_max_percent": PLAN_PATH_DEVIATION_MAX_PERCENT,
        "incremental_improvement_min_percent": PLAN_INCREMENTAL_IMPROVEMENT_MIN_PERCENT,
    }


def test_single_run_accepts_values_at_plan_boundaries():
    evaluation = evaluate_single_run(successful_observation())
    assert evaluation.success
    assert evaluation.failure_reasons == ()


def test_contact_sensor_collision_remains_an_authoritative_failure():
    evaluation = evaluate_single_run(
        successful_observation(collision_detected=True)
    )

    assert not evaluation.success
    assert evaluation.failure_reasons == ("collision_detected",)


@pytest.mark.parametrize(
    ("override", "reason"),
    [
        ({"nav2_succeeded": False}, "nav2_action_failed"),
        ({"ground_truth_available": False}, "ground_truth_unavailable"),
        ({"ground_truth_position_error_m": 0.251}, "goal_position_error"),
        (
            {"ground_truth_orientation_error_rad": math.radians(10.1)},
            "goal_orientation_error",
        ),
        ({"ground_truth_orientation_error_rad": None}, "goal_orientation_unavailable"),
        ({"collision_detected": True}, "collision_detected"),
        ({"localization_lost": True}, "localization_lost"),
        ({"tf_interrupted": True}, "tf_interrupted"),
        ({"timed_out": True}, "timed_out"),
        ({"collision_monitor_locked": True}, "collision_monitor_locked"),
        ({"final_linear_speed_mps": 0.021}, "final_velocity_not_zero"),
        ({"final_angular_speed_radps": -0.051}, "final_velocity_not_zero"),
        ({"safety_observability_complete": False}, "safety_status_unavailable"),
    ],
)
def test_single_run_reports_each_failure_condition(override, reason):
    evaluation = evaluate_single_run(successful_observation(**override))
    assert not evaluation.success
    assert reason in evaluation.failure_reasons


def test_orientation_is_ignored_when_goal_does_not_require_it():
    evaluation = evaluate_single_run(
        successful_observation(
            orientation_required=False,
            ground_truth_orientation_error_rad=None,
        )
    )
    assert evaluation.success
