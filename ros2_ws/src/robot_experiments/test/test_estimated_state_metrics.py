import math
from pathlib import Path

import pytest

from robot_experiments.estimated_state_metrics import evaluate_trajectory
from robot_experiments.estimated_state_metrics import PoseSample
from robot_experiments.estimated_state_metrics import stream_diagnostics


def _sample(stamp_s, x, y, yaw, covariance=None):
    return PoseSample(
        stamp_ns=int(round(stamp_s * 1.0e9)),
        x=x,
        y=y,
        yaw=yaw,
        covariance=covariance,
    )


def test_straight_trajectory_is_first_frame_se2_aligned():
    truth = [_sample(index + 1, index, 0.0, 0.0) for index in range(4)]
    estimate = [
        _sample(index + 1, 10.0, index, math.pi / 2.0)
        for index in range(4)
    ]

    result = evaluate_trajectory(estimate, truth, max_time_delta_ns=0)

    assert result.summary['association']['matched_count'] == 4
    assert result.summary['relative_ate']['xy_m']['rmse'] == pytest.approx(0.0)
    assert result.summary['relative_ate']['yaw_rad']['rmse'] == pytest.approx(0.0)
    assert result.summary['rpe']['xy_m']['rmse'] == pytest.approx(0.0)


def test_turn_trajectory_reports_relative_xy_and_yaw_error():
    truth = [
        _sample(1.0, 0.0, 0.0, 0.0),
        _sample(2.0, 1.0, 0.0, math.pi / 2.0),
        _sample(3.0, 1.0, 1.0, math.pi),
    ]
    estimate = [
        _sample(1.0, 0.0, 0.0, 0.0),
        _sample(2.0, 1.1, 0.0, math.pi / 2.0 + 0.1),
        _sample(3.0, 1.1, 0.9, -math.pi + 0.1),
    ]

    result = evaluate_trajectory(estimate, truth, max_time_delta_ns=0)

    assert result.summary['relative_ate']['xy_m']['rmse'] > 0.0
    assert result.summary['relative_ate']['yaw_rad']['rmse'] > 0.0
    assert result.summary['rpe']['xy_m']['count'] == 2
    assert result.summary['rpe']['yaw_rad']['count'] == 2
    assert all(abs(row['ate_yaw_rad']) <= math.pi for row in result.rows)


def test_nearest_timestamp_association_enforces_upper_bound():
    truth = [
        _sample(1.0, 0.0, 0.0, 0.0),
        _sample(2.0, 1.0, 0.0, 0.0),
        _sample(3.0, 2.0, 0.0, 0.0),
    ]
    estimate = [
        _sample(1.05, 0.0, 0.0, 0.0),
        _sample(2.20, 1.0, 0.0, 0.0),
        _sample(3.05, 2.0, 0.0, 0.0),
    ]

    result = evaluate_trajectory(
        estimate, truth, max_time_delta_ns=100_000_000)

    assert result.summary['association']['matched_count'] == 2
    assert result.summary['association']['max_abs_time_delta_ms'] == pytest.approx(50.0)
    assert [row['estimate_stamp_ns'] for row in result.rows] == [
        1_050_000_000,
        3_050_000_000,
    ]


def test_nan_pose_is_excluded_and_nan_covariance_is_reported():
    finite_covariance = tuple(0.0 for _ in range(36))
    nan_covariance = tuple(
        math.nan if index == 0 else 0.0 for index in range(36))
    estimate = [
        _sample(1.0, 0.0, 0.0, 0.0, finite_covariance),
        _sample(2.0, math.nan, 0.0, 0.0, finite_covariance),
        _sample(2.0, 1.0, 0.0, 0.0, nan_covariance),
        _sample(1.5, 0.5, 0.0, 0.0, None),
    ]
    truth = [
        _sample(1.0, 0.0, 0.0, 0.0),
        _sample(1.5, 0.5, 0.0, 0.0),
        _sample(2.0, 1.0, 0.0, 0.0),
    ]

    result = evaluate_trajectory(estimate, truth, max_time_delta_ns=0)
    diagnostics = stream_diagnostics(estimate)

    assert result.summary['valid_unique_samples'] == 3
    assert result.summary['association']['matched_count'] == 3
    assert diagnostics['duplicate_stamps'] == 1
    assert diagnostics['backward_stamps'] == 1
    assert diagnostics['covariance']['coverage_fraction'] == pytest.approx(0.75)
    assert diagnostics['covariance']['finite_fraction'] == pytest.approx(2.0 / 3.0)


def test_ros_adapter_is_evaluator_only_and_requires_explicit_output_dir():
    package_root = Path(__file__).parents[1]
    source = (
        package_root / 'robot_experiments' / 'estimated_state_evaluator.py'
    ).read_text(encoding='utf-8')
    setup_source = (package_root / 'setup.py').read_text(encoding='utf-8')

    assert "Odometry, '/odom'" in source
    assert "Odometry, '/wheel/odom'" in source
    assert "Odometry, '/lidar/odom'" in source
    assert "'lidar_odom': evaluate_trajectory(" in source
    assert "Imu, '/imu/data'" in source
    assert "'imu_data': evaluate_trajectory(" in source
    assert "'/amcl_pose'" in source
    assert "'/ground_truth/odom'" in source
    assert "declare_parameter('output_dir', '')" in source
    assert 'create_publisher' not in source
    assert 'TransformBroadcaster' not in source
    assert 'estimated_state_evaluator =' in setup_source
