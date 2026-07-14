import copy
import json
import math
from pathlib import Path

import pytest
import yaml

from robot_experiments.configuration import ConfigurationError
from robot_experiments.motion_baseline import (
    JointSample,
    MotionSegment,
    OdomSample,
    StopSettings,
    TimestampTracker,
    WheelLayout,
    analyse_motion_segment,
    detect_stopping,
    expected_wheel_directions,
    load_motion_baseline_config,
)


PACKAGE_ROOT = Path(__file__).parents[1]
CONFIG_PATH = PACKAGE_ROOT / "config" / "motion_baseline.yaml"
SKID_STEER_AB_CONFIG_PATH = (
    PACKAGE_ROOT / "config" / "motion_skid_steer_ab.yaml"
)
WHEELS = WheelLayout(
    front_left="front_left_wheel_joint",
    front_right="front_right_wheel_joint",
    rear_left="rear_left_wheel_joint",
    rear_right="rear_right_wheel_joint",
)
STOP = StopSettings(
    linear_velocity_threshold_mps=0.02,
    angular_velocity_threshold_radps=0.05,
    wheel_velocity_threshold_radps=0.2,
    stable_duration_sec=0.5,
    timeout_sec=5.0,
)


def _write_config(tmp_path, mutate):
    document = yaml.safe_load(CONFIG_PATH.read_text())
    mutate(document)
    path = tmp_path / "motion.yaml"
    path.write_text(yaml.safe_dump(document, sort_keys=False))
    return path


def _joint(stamp_ns, values):
    return JointSample.from_mapping(
        stamp_ns,
        dict(zip(WHEELS.ordered_names, values)),
    )


def test_installed_profile_is_strict_three_tier_and_arc_ab_matrix():
    config = load_motion_baseline_config(CONFIG_PATH)
    assert config.schema_version == 1
    assert len(config.segments) == 14
    assert {segment.motion for segment in config.segments} == {
        "forward",
        "backward",
        "rotate_left",
        "rotate_right",
        "arc_left",
        "arc_right",
    }
    assert {segment.tier for segment in config.segments} == {
        "low",
        "nominal",
        "high",
        "ab",
    }
    arcs = {segment.motion: segment for segment in config.segments if "arc" in segment.motion}
    assert set(arcs) == {"arc_left", "arc_right"}
    assert arcs["arc_left"].linear_x_mps == pytest.approx(0.4)
    assert arcs["arc_left"].angular_z_radps == pytest.approx(0.4)
    assert arcs["arc_right"].angular_z_radps == pytest.approx(-0.4)
    assert {segment.duration_sec for segment in arcs.values()} == {5.0}
    assert len(set(config.wheels.ordered_names)) == 4
    assert config.topics.cmd_vel == "/cmd_vel"
    assert config.reset.service == "/simulation/reset"


def test_skid_steer_ab_profile_matches_the_plan_commands_exactly():
    config = load_motion_baseline_config(SKID_STEER_AB_CONFIG_PATH)

    assert config.profile_id == "jackal_skid_steer_ab_v1"
    assert [segment.segment_id for segment in config.segments] == [
        "rotate_left_360",
        "rotate_right_360",
        "forward_3m",
        "backward_2m",
        "arc_left_5s",
        "arc_right_5s",
    ]
    commands = [
        (segment.linear_x_mps, segment.angular_z_radps, segment.duration_sec)
        for segment in config.segments
    ]
    assert commands == pytest.approx(
        [
            (0.0, 0.4, 2.0 * math.pi / 0.4),
            (0.0, -0.4, 2.0 * math.pi / 0.4),
            (0.5, 0.0, 3.0 / 0.5),
            (-0.3, 0.0, 2.0 / 0.3),
            (0.4, 0.4, 5.0),
            (0.4, -0.4, 5.0),
        ]
    )


@pytest.mark.parametrize(
    "mutate,match",
    [
        (lambda document: document.update(extra=True), "unknown keys"),
        (
            lambda document: document["segments"][0].update(linear_x_mps=-0.1),
            "command signs",
        ),
        (
            lambda document: document.update(
                segments=[
                    segment
                    for segment in document["segments"]
                    if segment["motion"] != "rotate_right"
                ]
            ),
            "must cover",
        ),
        (
            lambda document: document["sampling"].update(
                command_wall_timeout_sec=1.0
            ),
            "must exceed",
        ),
        (
            lambda document: document["segments"][0].update(linear_x_mps=1.1),
            "linear speed exceeds limits",
        ),
        (
            lambda document: document["sampling"].update(publish_rate_hz=5.0),
            "within \\[10, 100\\]",
        ),
        (
            lambda document: document["wheels"].update(
                rear_right=document["wheels"]["front_right"]
            ),
            "four unique",
        ),
    ],
)
def test_configuration_rejects_ambiguous_or_unsafe_profiles(tmp_path, mutate, match):
    path = _write_config(tmp_path, mutate)
    with pytest.raises(ConfigurationError, match=match):
        load_motion_baseline_config(path)


def test_timestamp_tracker_counts_receipt_order_regressions_and_duplicates():
    tracker = TimestampTracker()
    for stamp in (10, 11, 11, 9, 10):
        tracker.observe(stamp)
    assert tracker.as_dict() == {
        "sample_count": 5,
        "first_stamp_ns": 10,
        "last_stamp_ns": 10,
        "regression_count": 1,
        "duplicate_count": 1,
        "monotonic_unique": False,
    }
    with pytest.raises(ValueError, match="non-negative integer"):
        tracker.observe(-1)


def test_expected_wheel_directions_cover_forward_backward_and_pure_rotation():
    assert set(expected_wheel_directions("forward", WHEELS).values()) == {
        "positive"
    }
    assert set(expected_wheel_directions("backward", WHEELS).values()) == {
        "negative"
    }
    left = expected_wheel_directions("rotate_left", WHEELS)
    assert [left[name] for name in WHEELS.left_names] == ["negative", "negative"]
    assert [left[name] for name in WHEELS.right_names] == ["positive", "positive"]
    right = expected_wheel_directions("rotate_right", WHEELS)
    assert [right[name] for name in WHEELS.left_names] == ["positive", "positive"]
    assert [right[name] for name in WHEELS.right_names] == ["negative", "negative"]
    assert set(expected_wheel_directions("arc_left", WHEELS).values()) == {
        "positive"
    }
    assert set(expected_wheel_directions("arc_right", WHEELS).values()) == {
        "positive"
    }


def test_stop_detection_requires_chassis_and_all_four_wheels_to_stay_still():
    end = 3_000_000_000
    odom = [
        OdomSample(end + 100_000_000, 0, 0, 0, 0.1, 0, 0),
        OdomSample(end + 200_000_000, 0, 0, 0, 0.0, 0, 0),
        OdomSample(end + 700_000_000, 0, 0, 0, 0.0, 0, 0),
    ]
    joints = [
        _joint(end + 100_000_000, (1, 1, 1, 1)),
        _joint(end + 200_000_000, (0, 0, 0, 0)),
        _joint(end + 700_000_000, (0, 0, 0, 0)),
    ]
    result = detect_stopping(end, odom, joints, WHEELS, STOP)
    assert result.stopped is True
    assert result.stationary_onset_after_command_sec == pytest.approx(0.2)
    assert result.confirmed_after_command_sec == pytest.approx(0.7)

    not_stopped = copy.copy(joints)
    not_stopped[-1] = _joint(end + 700_000_000, (0, 0, 0, 0.3))
    assert detect_stopping(end, odom, not_stopped, WHEELS, STOP).stopped is False


def test_forward_analysis_records_pose_drift_velocity_wheels_and_stop_time():
    start = 1_000_000_000
    end = 3_000_000_000
    odom = [
        OdomSample(start, 0.0, 0.0, math.pi / 2, 0.5, 0.0, 0.0),
        OdomSample(2_000_000_000, 0.05, 0.5, math.pi / 2, 0.5, 0.0, 0.0),
        OdomSample(end, 0.1, 1.0, math.pi / 2, 0.5, 0.0, 0.0),
        OdomSample(end + 200_000_000, 0.1, 1.0, math.pi / 2, 0, 0, 0),
        OdomSample(end + 700_000_000, 0.1, 1.0, math.pi / 2, 0, 0, 0),
    ]
    joints = [
        _joint(start, (5.0, 5.1, 4.9, 5.0)),
        _joint(2_000_000_000, (5.0, 5.1, 4.9, 5.0)),
        _joint(end, (5.0, 5.1, 4.9, 5.0)),
        _joint(end + 200_000_000, (0, 0, 0, 0)),
        _joint(end + 700_000_000, (0, 0, 0, 0)),
    ]
    segment = MotionSegment("forward_nominal", "forward", "nominal", 0.5, 0, 2)
    result = analyse_motion_segment(
        segment,
        start,
        end,
        odom,
        joints,
        WHEELS,
        STOP,
        command_publish_count=41,
        timestamp_integrity={
            "clock": {"regression_count": 0, "duplicate_count": 0},
            "odom": {"regression_count": 0, "duplicate_count": 0},
            "joint_states": {"regression_count": 0, "duplicate_count": 0},
        },
    )
    assert result["result"] == "complete"
    assert result["command"]["configured_duration_sec"] == 2
    assert result["command"]["observed_duration_sec"] == pytest.approx(2)
    assert result["pose"]["longitudinal_displacement_m"] == pytest.approx(1.0)
    assert result["pose"]["lateral_drift_m"] == pytest.approx(-0.1)
    assert result["pose"]["trajectory_length_m"] == pytest.approx(
        2 * math.hypot(0.05, 0.5)
    )
    assert result["yaw"]["change_rad"] == pytest.approx(0)
    assert result["actual_velocity"]["linear_x_mps"]["mean"] == pytest.approx(0.5)
    assert result["stopping"]["stationary_onset_after_command_sec"] == pytest.approx(0.2)
    assert result["wheels"]["all_directions_match"] is True
    json.dumps(result, allow_nan=False)


def test_rotation_analysis_unwraps_yaw_and_flags_wrong_wheel_direction():
    start = 1_000_000_000
    end = 2_000_000_000
    odom = [
        OdomSample(start, 0, 0, 3.0, 0, 0, 0.5),
        OdomSample(1_500_000_000, 0.01, 0, -3.0, 0, 0, 0.5),
        OdomSample(end, 0.02, 0, -2.8, 0, 0, 0.5),
        OdomSample(end + 100_000_000, 0.02, 0, -2.8, 0, 0, 0),
        OdomSample(end + 600_000_000, 0.02, 0, -2.8, 0, 0, 0),
    ]
    joints = [
        _joint(start, (3, 3, 3, 3)),
        _joint(1_500_000_000, (3, 3, 3, 3)),
        _joint(end, (3, 3, 3, 3)),
        _joint(end + 100_000_000, (0, 0, 0, 0)),
        _joint(end + 600_000_000, (0, 0, 0, 0)),
    ]
    segment = MotionSegment("left", "rotate_left", "test", 0, 0.5, 1)
    result = analyse_motion_segment(
        segment,
        start,
        end,
        odom,
        joints,
        WHEELS,
        STOP,
        command_publish_count=20,
        timestamp_integrity={},
    )
    assert result["yaw"]["change_rad"] == pytest.approx(
        (2 * math.pi - 6.0) + 0.2
    )
    assert result["pose"]["translation_drift_m"] == pytest.approx(0.02)
    assert result["wheels"]["all_directions_match"] is False
    assert (
        result["wheels"]["per_wheel"][WHEELS.front_left]["expected_direction"]
        == "negative"
    )


def test_arc_analysis_keeps_intended_lateral_displacement_distinct_from_drift():
    start = 1_000_000_000
    end = 6_000_000_000
    odom = [
        OdomSample(start, 0, 0, 0, 0.4, 0, 0.4),
        OdomSample(end, 0.9, 1.4, 2.0, 0.4, 0, 0.4),
        OdomSample(end + 100_000_000, 0.9, 1.4, 2.0, 0, 0, 0),
        OdomSample(end + 600_000_000, 0.9, 1.4, 2.0, 0, 0, 0),
    ]
    joints = [
        _joint(start, (3, 5, 3, 5)),
        _joint(end, (3, 5, 3, 5)),
        _joint(end + 100_000_000, (0, 0, 0, 0)),
        _joint(end + 600_000_000, (0, 0, 0, 0)),
    ]
    segment = MotionSegment("arc_left_ab", "arc_left", "ab", 0.4, 0.4, 5)
    result = analyse_motion_segment(
        segment,
        start,
        end,
        odom,
        joints,
        WHEELS,
        STOP,
        command_publish_count=100,
        timestamp_integrity={},
    )
    assert result["pose"]["lateral_displacement_m"] == pytest.approx(1.4)
    assert result["pose"]["lateral_drift_m"] is None
    assert result["pose"]["translation_drift_m"] is None
    assert result["yaw"]["expected_change_rad"] == pytest.approx(2.0)
    assert result["wheels"]["all_directions_match"] is True


def test_analysis_rejects_missing_command_samples_instead_of_inventing_metrics():
    segment = MotionSegment("forward", "forward", "test", 0.1, 0, 1)
    with pytest.raises(ValueError, match="no odometry"):
        analyse_motion_segment(
            segment,
            1,
            2,
            [],
            [],
            WHEELS,
            STOP,
            command_publish_count=1,
            timestamp_integrity={},
        )
