import copy
import hashlib
import json
import math
import time
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

import robot_experiments.motion_baseline_runner as motion_baseline_runner
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
from robot_experiments.motion_baseline_runner import (
    MotionBaselineRunner,
    _coherent_group_ready,
    _decode_hashed_ground_topology_snapshot,
    _parse_reset_response_metadata,
    _post_reset_observation_ns,
    _require_live_runtime_provenance_schema,
    _timestamp_regression_topics,
    _update_stationary_window,
)
from robot_experiments.report import ReportValidationError


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


def test_ground_topology_parameter_pair_decodes_canonical_verified_json():
    topology = {
        "operation": "disable_selected_colliders",
        "target": {"collider_count": 1},
    }
    payload = json.dumps(
        topology,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    )
    digest = hashlib.sha256(payload.encode("utf-8")).hexdigest()

    assert _decode_hashed_ground_topology_snapshot(payload, digest) == topology


@pytest.mark.parametrize(
    ("payload", "message"),
    [
        ('{"a":1,"a":2}', "duplicate JSON key"),
        ('{"b":2, "a":1}', "canonical strict JSON"),
        ('{"value":NaN}', "non-finite JSON constant"),
        ('{"value":1e400}', "strict JSON"),
        ("[]", "root must be a mapping"),
    ],
)
def test_ground_topology_parameter_pair_rejects_invalid_json(payload, message):
    digest = hashlib.sha256(payload.encode("utf-8")).hexdigest()

    with pytest.raises(ReportValidationError, match=message):
        _decode_hashed_ground_topology_snapshot(payload, digest)


def test_ground_topology_parameter_pair_rejects_hash_mismatch():
    payload = '{"operation":"keep_all"}'

    with pytest.raises(ReportValidationError, match="JSON SHA256 mismatch"):
        _decode_hashed_ground_topology_snapshot(payload, "0" * 64)


@pytest.mark.parametrize("payload", [None, 123, {}, []])
def test_ground_topology_parameter_pair_requires_string_payload(payload):
    with pytest.raises(ReportValidationError, match="json must be a string"):
        _decode_hashed_ground_topology_snapshot(payload, "0" * 64)


@pytest.mark.parametrize(
    "digest",
    [None, 123, "short", "g" * 64, "a" * 63, "a" * 65],
)
def test_ground_topology_parameter_pair_requires_sha256_digest(digest):
    with pytest.raises(ReportValidationError, match="SHA256 hex digest"):
        _decode_hashed_ground_topology_snapshot("{}", digest)


@pytest.mark.parametrize("schema_version", [5, 6.0, True, None])
def test_live_motion_report_requires_exact_runtime_schema_v6(schema_version):
    with pytest.raises(RuntimeError, match="schema must be integer 6"):
        _require_live_runtime_provenance_schema(schema_version)


def test_live_motion_report_accepts_runtime_schema_v6():
    assert _require_live_runtime_provenance_schema(6) == 6


def test_base_motion_report_uses_schema_v4_while_configuration_stays_v1():
    runner = object.__new__(MotionBaselineRunner)
    runner._config = load_motion_baseline_config(CONFIG_PATH)
    runner._environment_id = "Warehouse"
    runner._odometry_mode = "ideal"
    runner._config_path = CONFIG_PATH
    runner._config_hash = "0" * 64
    runner._output_path = Path("/tmp/motion-report.json")
    runner._started_at = SimpleNamespace(
        isoformat=lambda: "2026-07-15T00:00:00+00:00"
    )
    runner._runtime_provenance = {"verified": False}
    runner._segments = []
    runner._session_timestamps = {
        "clock": TimestampTracker(),
        "odom": TimestampTracker(),
        "joint_states": TimestampTracker(),
    }
    runner._authorized_reset_publishers = set()
    runner._publisher = None
    runner._safe_stop_attempted = False

    report = runner._base_report()

    assert report["schema_version"] == 4
    assert report["configuration"]["schema_version"] == 1


def test_motion_runner_reads_and_validates_reset_strategy_and_topology(
    monkeypatch,
):
    names = motion_baseline_runner._RUNTIME_PROVENANCE_PARAMETER_NAMES
    topology = {"operation": "keep_all", "target": {"collider_count": 32}}
    topology_payload = json.dumps(
        topology,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    )
    reset_strategy = {
        "schema_version": 1,
        "id": "pose_restore_v1",
        "lift_distance_m": 0.0,
        "separation_step_count": 0,
        "recontact_step_count": 1,
        "contact_probe": {
            "schema_version": 1,
            "enabled": True,
            "wheel_bindings": [
                {
                    "joint_name": joint_name,
                    "wheel_link_path": f"/World/Robot/{joint_name}",
                }
                for joint_name in ("fl", "fr", "rl", "rr")
            ],
            "wheel_count": 4,
            "ground_filter_paths": ["/World/Ground"],
            "ground_filter_count": 1,
            "max_contact_count": 128,
            "report_threshold_n": 0.0,
            "stage_usd_readback_verified": True,
        },
    }
    reset_strategy_payload = json.dumps(
        reset_strategy,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    )
    values = {name: None for name in names}
    values.update(
        {
            "runtime_provenance.schema_version": 6,
            "runtime_provenance.environment.id": "Warehouse",
            "runtime_provenance.simulation.odometry_mode": "ground_truth",
            "runtime_provenance.simulation.reset_strategy.json": (
                reset_strategy_payload
            ),
            "runtime_provenance.simulation.reset_strategy.sha256": (
                hashlib.sha256(
                    reset_strategy_payload.encode("utf-8")
                ).hexdigest()
            ),
            "runtime_provenance.contact.json": "{}",
            "runtime_provenance.contact.sha256": hashlib.sha256(
                b"{}"
            ).hexdigest(),
            "runtime_provenance.ground_topology.json": topology_payload,
            "runtime_provenance.ground_topology.sha256": hashlib.sha256(
                topology_payload.encode("utf-8")
            ).hexdigest(),
        }
    )

    class ParameterClient:
        requested_names = None

        def wait_for_services(self, *, timeout_sec):
            assert timeout_sec == pytest.approx(1.0)
            return True

        def get_parameters(self, requested_names):
            self.requested_names = tuple(requested_names)
            return SimpleNamespace(
                result=lambda: SimpleNamespace(
                    values=[values[name] for name in requested_names]
                )
            )

    validated = []
    client = ParameterClient()
    runner = SimpleNamespace(
        _config=SimpleNamespace(
            reset=SimpleNamespace(service_timeout_sec=1.0)
        ),
        _isaac_parameter_client=client,
        _wait_future=lambda future, deadline: True,
        _raise_if_shutdown=lambda: None,
        _runtime_provenance={"verified": False},
        _odometry_mode="ground_truth",
        _environment_id="Warehouse",
    )
    monkeypatch.setattr(
        motion_baseline_runner,
        "parameter_value_to_python",
        lambda value: value,
    )
    monkeypatch.setattr(
        motion_baseline_runner,
        "validate_runtime_provenance",
        validated.append,
    )

    MotionBaselineRunner._read_runtime_provenance(runner)

    assert client.requested_names == names
    assert runner._runtime_provenance["schema_version"] == 6
    assert (
        runner._runtime_provenance["simulation"]["reset_strategy"]
        == reset_strategy
    )
    assert runner._runtime_provenance["ground_topology"] == topology
    assert validated == [runner._runtime_provenance]


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
    assert config.sampling.command_wall_timeout_sec == pytest.approx(90.0)
    assert config.sampling.max_future_skew_sec == pytest.approx(0.02)
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
            lambda document: document["sampling"].update(
                max_future_skew_sec=0.051
            ),
            "max_future_skew_sec must not exceed",
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


def test_configuration_allows_strict_zero_future_skew(tmp_path):
    path = _write_config(
        tmp_path,
        lambda document: document["sampling"].update(max_future_skew_sec=0.0),
    )
    assert load_motion_baseline_config(path).sampling.max_future_skew_sec == 0.0


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
    result = detect_stopping(end, odom, joints, WHEELS, STOP, 0.5)
    assert result.stopped is True
    assert result.stationary_onset_after_command_sec == pytest.approx(0.2)
    assert result.confirmed_after_command_sec == pytest.approx(0.7)

    not_stopped = copy.copy(joints)
    not_stopped[-1] = _joint(end + 700_000_000, (0, 0, 0, 0.3))
    assert (
        detect_stopping(end, odom, not_stopped, WHEELS, STOP, 0.5).stopped
        is False
    )


@pytest.mark.parametrize("starved_source", ("odom", "joint_states"))
def test_stop_detection_rejects_a_single_stale_stream(starved_source):
    end = 3_000_000_000
    odom = [
        OdomSample(end + 100_000_000, 0, 0, 0, 0, 0, 0),
        OdomSample(end + 700_000_000, 0, 0, 0, 0, 0, 0),
    ]
    joints = [
        _joint(end + 100_000_000, (0, 0, 0, 0)),
        _joint(end + 700_000_000, (0, 0, 0, 0)),
    ]
    if starved_source == "odom":
        odom = odom[:1]
    else:
        joints = joints[:1]

    result = detect_stopping(end, odom, joints, WHEELS, STOP, 0.5)

    assert result.stopped is False
    assert result.stationary_evidence is None


def test_stop_detection_records_dual_stream_stationary_evidence():
    end = 3_000_000_000
    odom = [
        OdomSample(end + offset, 0, 0, 0, 0, 0, 0)
        for offset in (100_000_000, 350_000_000, 700_000_000)
    ]
    joints = [
        _joint(end + offset, (0, 0, 0, 0))
        for offset in (100_000_000, 400_000_000, 700_000_000)
    ]

    result = detect_stopping(end, odom, joints, WHEELS, STOP, 0.5)

    assert result.stopped is True
    assert result.stationary_evidence == {
        "schema_version": 1,
        "definition": "dual_stream_continuously_stationary_after_zero_command",
        "boundary_semantics": "closed_interval",
        "start_stamp_ns": end + 100_000_000,
        "end_stamp_ns": end + 700_000_000,
        "observed_duration_sec": pytest.approx(0.6),
        "max_sample_age_sec": pytest.approx(0.5),
        "streams": {
            "odom": {
                "sample_count": 3,
                "first_sample_stamp_ns": end + 100_000_000,
                "last_sample_stamp_ns": end + 700_000_000,
                "maximum_inter_sample_gap_sec": pytest.approx(0.35),
            },
            "joint_states": {
                "sample_count": 3,
                "first_sample_stamp_ns": end + 100_000_000,
                "last_sample_stamp_ns": end + 700_000_000,
                "maximum_inter_sample_gap_sec": pytest.approx(0.3),
            },
        },
    }


def test_stop_detection_restarts_after_a_gap_larger_than_sample_age():
    end = 3_000_000_000
    odom = [
        OdomSample(end + offset, 0, 0, 0, 0, 0, 0)
        for offset in (100_000_000, 700_000_000)
    ]
    joints = [
        _joint(end + offset, (0, 0, 0, 0))
        for offset in (100_000_000, 700_000_000)
    ]

    result = detect_stopping(end, odom, joints, WHEELS, STOP, 0.5)

    assert result.stopped is False


def test_stop_detection_supports_regular_asynchronous_streams():
    end = 3_000_000_000
    odom = [
        OdomSample(end + offset, 0, 0, 0, 0, 0, 0)
        for offset in (100_000_000, 300_000_000, 500_000_000, 700_000_000)
    ]
    joints = [
        _joint(end + offset, (0, 0, 0, 0))
        for offset in (200_000_000, 400_000_000, 600_000_000, 800_000_000)
    ]

    result = detect_stopping(end, odom, joints, WHEELS, STOP, 0.5)

    assert result.stopped is True
    evidence = result.stationary_evidence
    assert evidence is not None
    assert evidence["start_stamp_ns"] == end + 200_000_000
    assert evidence["end_stamp_ns"] == end + 700_000_000
    streams = evidence["streams"]
    assert streams["odom"]["first_sample_stamp_ns"] == end + 300_000_000
    assert streams["joint_states"]["first_sample_stamp_ns"] == end + 200_000_000


def test_stop_detection_rejects_boolean_sample_age():
    with pytest.raises(ValueError, match="finite number"):
        detect_stopping(0, [], [], WHEELS, STOP, True)


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
        max_sample_age_sec=1.0,
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
    assert result["pose"][
        "max_radial_displacement_from_start_m"
    ] == pytest.approx(math.hypot(0.1, 1.0))
    assert result["yaw"]["change_rad"] == pytest.approx(0)
    assert result["actual_velocity"]["linear_x_mps"]["mean"] == pytest.approx(0.5)
    assert result["stopping"]["stationary_onset_after_command_sec"] == pytest.approx(0.2)
    assert result["wheels"]["all_directions_match"] is True
    json.dumps(result, allow_nan=False)


def test_motion_analysis_uses_only_final_half_for_steady_state_yaw_rate():
    start = 1_000_000_000
    end = 4_000_000_001
    boundary = start + (end - start) // 2
    odom = [
        OdomSample(start, 0, 0, 0, 0, 0, 9.0),
        OdomSample(boundary - 1, 0, 0, 0, 0, 0, 8.0),
        OdomSample(boundary, 0, 0, 0, 0, 0, 0.4),
        OdomSample(end, 0, 0, 0, 0, 0, 0.6),
        OdomSample(end + 100_000_000, 0, 0, 0, 0, 0, 0),
        OdomSample(end + 600_000_000, 0, 0, 0, 0, 0, 0),
    ]
    moving_wheels = (-3.0, 3.0, -3.0, 3.0)
    joints = [
        _joint(start, moving_wheels),
        _joint(boundary - 1, moving_wheels),
        _joint(boundary, moving_wheels),
        _joint(end, moving_wheels),
        _joint(end + 100_000_000, (0, 0, 0, 0)),
        _joint(end + 600_000_000, (0, 0, 0, 0)),
    ]
    segment = MotionSegment("left", "rotate_left", "test", 0, 0.5, 3)

    result = analyse_motion_segment(
        segment,
        start,
        end,
        odom,
        joints,
        WHEELS,
        STOP,
        command_publish_count=60,
        max_sample_age_sec=2.0,
        timestamp_integrity={},
    )

    window = result["actual_velocity"]["steady_state_window"]
    assert window == {
        "schema_version": 1,
        "definition": "final_half_of_command_interval",
        "boundary_semantics": "closed_interval",
        "start_stamp_ns": boundary,
        "end_stamp_ns": end,
        "observed_duration_sec": pytest.approx(
            (end - boundary) / 1_000_000_000
        ),
        "sample_count": 2,
        "first_sample_stamp_ns": boundary,
        "last_sample_stamp_ns": end,
        "maximum_inter_sample_gap_sec": pytest.approx(
            (end - boundary) / 1_000_000_000
        ),
        "angular_z_radps": {
            "sample_count": 2,
            "mean": pytest.approx(0.5),
            "mean_abs": pytest.approx(0.5),
            "minimum": pytest.approx(0.4),
            "maximum": pytest.approx(0.6),
            "peak_abs": pytest.approx(0.6),
            "rmse": pytest.approx(math.sqrt((0.4**2 + 0.6**2) / 2)),
        },
    }
    assert result["actual_velocity"]["angular_z_radps"]["mean"] != pytest.approx(
        window["angular_z_radps"]["mean"]
    )
    wheel_window = result["wheels"]["steady_state_window"]
    assert set(wheel_window) == {
        "schema_version",
        "definition",
        "boundary_semantics",
        "start_stamp_ns",
        "end_stamp_ns",
        "observed_duration_sec",
        "sample_count",
        "first_sample_stamp_ns",
        "last_sample_stamp_ns",
        "maximum_inter_sample_gap_sec",
        "classification_deadband_radps",
        "all_directions_match",
        "per_wheel",
    }
    assert wheel_window["schema_version"] == 1
    assert wheel_window["definition"] == "final_half_of_command_interval"
    assert wheel_window["boundary_semantics"] == "closed_interval"
    assert wheel_window["start_stamp_ns"] == boundary
    assert wheel_window["end_stamp_ns"] == end
    assert wheel_window["observed_duration_sec"] == pytest.approx(
        (end - boundary) / 1_000_000_000
    )
    assert wheel_window["sample_count"] == 2
    assert wheel_window["first_sample_stamp_ns"] == boundary
    assert wheel_window["last_sample_stamp_ns"] == end
    assert wheel_window["maximum_inter_sample_gap_sec"] == pytest.approx(
        (end - boundary) / 1_000_000_000
    )
    assert wheel_window["classification_deadband_radps"] == pytest.approx(
        STOP.wheel_velocity_threshold_radps
    )
    assert wheel_window["all_directions_match"] is True
    front_left = wheel_window["per_wheel"][WHEELS.front_left]
    assert set(front_left) == {
        "direction",
        "expected_direction",
        "direction_matches",
        "speed_radps",
        "direction_sample_counts",
    }
    assert front_left["direction"] == "negative"
    assert front_left["expected_direction"] == "negative"
    assert front_left["direction_matches"] is True
    assert front_left["direction_sample_counts"] == {
        "positive_above_deadband": 0,
        "negative_below_deadband": 2,
        "within_deadband": 0,
    }
    assert front_left["speed_radps"] == {
        "sample_count": 2,
        "mean": pytest.approx(-3.0),
        "mean_abs": pytest.approx(3.0),
        "minimum": pytest.approx(-3.0),
        "maximum": pytest.approx(-3.0),
        "peak_abs": pytest.approx(3.0),
        "rmse": pytest.approx(3.0),
    }


def test_motion_analysis_rejects_empty_final_half_odom_steady_state_window():
    start = 1_000_000_000
    end = 3_000_000_000
    boundary = start + (end - start) // 2
    odom = [
        OdomSample(start, 0, 0, 0, 0.5, 0, 0),
        OdomSample(boundary - 1, 0.4, 0, 0, 0.5, 0, 0),
        OdomSample(end + 100_000_000, 0.4, 0, 0, 0, 0, 0),
        OdomSample(end + 600_000_000, 0.4, 0, 0, 0, 0, 0),
    ]
    joints = [
        _joint(start, (5, 5, 5, 5)),
        _joint(end, (5, 5, 5, 5)),
        _joint(end + 100_000_000, (0, 0, 0, 0)),
        _joint(end + 600_000_000, (0, 0, 0, 0)),
    ]
    segment = MotionSegment("forward", "forward", "test", 0.5, 0, 2)

    with pytest.raises(ValueError, match="steady-state window"):
        analyse_motion_segment(
            segment,
            start,
            end,
            odom,
            joints,
            WHEELS,
            STOP,
            command_publish_count=40,
            max_sample_age_sec=1.0,
            timestamp_integrity={},
        )


def test_motion_analysis_rejects_empty_final_half_joint_steady_state_window():
    start = 1_000_000_000
    end = 3_000_000_000
    boundary = start + (end - start) // 2
    odom = [
        OdomSample(start, 0, 0, 0, 0.5, 0, 0),
        OdomSample(boundary, 0.5, 0, 0, 0.5, 0, 0),
        OdomSample(end, 1.0, 0, 0, 0.5, 0, 0),
        OdomSample(end + 100_000_000, 1.0, 0, 0, 0, 0, 0),
        OdomSample(end + 600_000_000, 1.0, 0, 0, 0, 0, 0),
    ]
    joints = [
        _joint(start, (5, 5, 5, 5)),
        _joint(boundary - 1, (5, 5, 5, 5)),
        _joint(end + 100_000_000, (0, 0, 0, 0)),
        _joint(end + 600_000_000, (0, 0, 0, 0)),
    ]

    with pytest.raises(
        ValueError,
        match="no joint-state samples overlap the steady-state window",
    ):
        analyse_motion_segment(
            MotionSegment("forward", "forward", "test", 0.5, 0, 2),
            start,
            end,
            odom,
            joints,
            WHEELS,
            STOP,
            command_publish_count=40,
            max_sample_age_sec=1.0,
            timestamp_integrity={},
        )


@pytest.mark.parametrize(
    ("steady_stamps", "message"),
    (
        ((2_000_000_000,), "at least two samples"),
        (
            (2_600_000_000, 3_000_000_000),
            "first odometry steady-state sample",
        ),
        (
            (2_000_000_000, 2_400_000_000),
            "last odometry steady-state sample",
        ),
        (
            (2_000_000_000, 2_800_000_000, 3_000_000_000),
            "steady-state sample gap",
        ),
        (
            (2_000_000_000, 2_000_000_000, 3_000_000_000),
            "timestamps must be strictly increasing",
        ),
        (
            (2_000_000_000, 2_700_000_000, 2_500_000_000, 3_000_000_000),
            "timestamps must be strictly increasing",
        ),
    ),
)
def test_odom_steady_state_window_rejects_sparse_or_nonmonotonic_evidence(
    steady_stamps,
    message,
):
    start = 1_000_000_000
    end = 3_000_000_000
    odom = [
        OdomSample(start, 0, 0, 0, 0, 0, 0),
        *(
            OdomSample(stamp, 0, 0, 0, 0, 0, 0.4)
            for stamp in steady_stamps
        ),
        OdomSample(end + 100_000_000, 0, 0, 0, 0, 0, 0),
        OdomSample(end + 600_000_000, 0, 0, 0, 0, 0, 0),
    ]
    joints = [
        _joint(start, (-3, 3, -3, 3)),
        _joint(2_000_000_000, (-3, 3, -3, 3)),
        _joint(2_500_000_000, (-3, 3, -3, 3)),
        _joint(end, (-3, 3, -3, 3)),
        _joint(end + 100_000_000, (0, 0, 0, 0)),
        _joint(end + 600_000_000, (0, 0, 0, 0)),
    ]

    with pytest.raises(ValueError, match=message):
        analyse_motion_segment(
            MotionSegment("left", "rotate_left", "test", 0, 0.4, 2),
            start,
            end,
            odom,
            joints,
            WHEELS,
            STOP,
            command_publish_count=40,
            max_sample_age_sec=0.5,
            timestamp_integrity={},
        )


@pytest.mark.parametrize(
    ("steady_stamps", "message"),
    (
        ((2_000_000_000,), "at least two samples"),
        (
            (2_600_000_000, 3_000_000_000),
            "first joint-state steady-state sample",
        ),
        (
            (2_000_000_000, 2_400_000_000),
            "last joint-state steady-state sample",
        ),
        (
            (2_000_000_000, 2_800_000_000, 3_000_000_000),
            "steady-state sample gap",
        ),
    ),
)
def test_wheel_steady_state_window_rejects_sparse_evidence(
    steady_stamps,
    message,
):
    start = 1_000_000_000
    end = 3_000_000_000
    odom = [
        OdomSample(start, 0, 0, 0, 0.5, 0, 0),
        OdomSample(2_000_000_000, 0.5, 0, 0, 0.5, 0, 0),
        OdomSample(2_500_000_000, 0.75, 0, 0, 0.5, 0, 0),
        OdomSample(end, 1.0, 0, 0, 0.5, 0, 0),
        OdomSample(end + 100_000_000, 1.0, 0, 0, 0, 0, 0),
        OdomSample(end + 600_000_000, 1.0, 0, 0, 0, 0, 0),
    ]
    joints = [
        _joint(start, (5, 5, 5, 5)),
        *(_joint(stamp, (5, 5, 5, 5)) for stamp in steady_stamps),
        _joint(end + 100_000_000, (0, 0, 0, 0)),
        _joint(end + 600_000_000, (0, 0, 0, 0)),
    ]

    with pytest.raises(ValueError, match=message):
        analyse_motion_segment(
            MotionSegment("forward", "forward", "test", 0.5, 0, 2),
            start,
            end,
            odom,
            joints,
            WHEELS,
            STOP,
            command_publish_count=40,
            max_sample_age_sec=0.5,
            timestamp_integrity={},
        )


@pytest.mark.parametrize(
    ("max_sample_age_sec", "message"),
    (
        (True, "finite number"),
        (False, "finite number"),
        ("0.5", "finite number"),
        (float("nan"), "finite"),
        (float("inf"), "finite"),
        (-float("inf"), "finite"),
        (0, "positive"),
        (-0.1, "positive"),
    ),
)
def test_motion_analysis_rejects_invalid_max_sample_age(
    max_sample_age_sec,
    message,
):
    start = 1_000_000_000
    boundary = 2_000_000_000
    end = 3_000_000_000
    odom = [
        OdomSample(start, 0, 0, 0, 0.5, 0, 0),
        OdomSample(boundary, 0.5, 0, 0, 0.5, 0, 0),
        OdomSample(end, 1.0, 0, 0, 0.5, 0, 0),
        OdomSample(end + 100_000_000, 1.0, 0, 0, 0, 0, 0),
        OdomSample(end + 600_000_000, 1.0, 0, 0, 0, 0, 0),
    ]
    joints = [
        _joint(start, (5, 5, 5, 5)),
        _joint(boundary, (5, 5, 5, 5)),
        _joint(end, (5, 5, 5, 5)),
        _joint(end + 100_000_000, (0, 0, 0, 0)),
        _joint(end + 600_000_000, (0, 0, 0, 0)),
    ]

    with pytest.raises(ValueError, match=message):
        analyse_motion_segment(
            MotionSegment("forward", "forward", "test", 0.5, 0, 2),
            start,
            end,
            odom,
            joints,
            WHEELS,
            STOP,
            command_publish_count=40,
            max_sample_age_sec=max_sample_age_sec,
            timestamp_integrity={},
        )


def test_wheel_steady_state_window_excludes_first_half_mixed_transient():
    start = 1_000_000_000
    end = 3_000_000_000
    boundary = start + (end - start) // 2
    odom = [
        OdomSample(start, 0, 0, 0, 0.4, 0, 0.4),
        OdomSample(boundary, 0.4, 0, 0.4, 0.4, 0, 0.4),
        OdomSample(end, 0.8, 0, 0.8, 0.4, 0, 0.4),
        OdomSample(end + 100_000_000, 0.8, 0, 0.8, 0, 0, 0),
        OdomSample(end + 600_000_000, 0.8, 0, 0.8, 0, 0, 0),
    ]
    joints = [
        _joint(start, (-0.5, 5.0, 2.0, 5.0)),
        _joint(boundary - 1, (-0.4, 5.0, 2.0, 5.0)),
        _joint(
            boundary,
            (STOP.wheel_velocity_threshold_radps, 5.0, 2.0, 5.0),
        ),
        _joint(end, (1.7, 5.0, 2.0, 5.0)),
        _joint(end + 100_000_000, (0, 0, 0, 0)),
        _joint(end + 600_000_000, (0, 0, 0, 0)),
    ]

    result = analyse_motion_segment(
        MotionSegment("arc", "arc_left", "test", 0.4, 0.4, 2),
        start,
        end,
        odom,
        joints,
        WHEELS,
        STOP,
        command_publish_count=40,
        max_sample_age_sec=1.0,
        timestamp_integrity={},
    )

    whole = result["wheels"]
    assert whole["per_wheel"][WHEELS.front_left]["direction"] == "mixed"
    assert whole["all_directions_match"] is False
    steady = whole["steady_state_window"]
    assert steady["sample_count"] == 2
    assert steady["per_wheel"][WHEELS.front_left]["direction"] == "positive"
    assert steady["per_wheel"][WHEELS.front_left]["direction_matches"] is True
    assert steady["per_wheel"][WHEELS.front_left][
        "direction_sample_counts"
    ] == {
        "positive_above_deadband": 1,
        "negative_below_deadband": 0,
        "within_deadband": 1,
    }
    assert steady["all_directions_match"] is True


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
        max_sample_age_sec=0.5,
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


def test_rotation_analysis_keeps_maximum_excursion_distinct_from_endpoint_residual():
    start = 1_000_000_000
    middle = 1_500_000_000
    end = 2_000_000_000
    odom = [
        OdomSample(start, 0, 0, 0, 0, 0, 0.5),
        OdomSample(middle, 0.15, 0, 0.25, 0, 0, 0.5),
        OdomSample(end, 0, 0, 0.5, 0, 0, 0.5),
        # A post-command parking sample must not inflate the command metric.
        OdomSample(end + 100_000_000, 1.0, 0, 0.5, 0, 0, 0),
        OdomSample(end + 600_000_000, 1.0, 0, 0.5, 0, 0, 0),
    ]
    moving_wheels = (-3.0, 3.0, -3.0, 3.0)
    joints = [
        _joint(start, moving_wheels),
        _joint(middle, moving_wheels),
        _joint(end, moving_wheels),
        _joint(end + 100_000_000, (0, 0, 0, 0)),
        _joint(end + 600_000_000, (0, 0, 0, 0)),
    ]

    result = analyse_motion_segment(
        MotionSegment("left", "rotate_left", "test", 0, 0.5, 1),
        start,
        end,
        odom,
        joints,
        WHEELS,
        STOP,
        command_publish_count=20,
        max_sample_age_sec=0.5,
        timestamp_integrity={},
    )

    assert result["pose"]["translation_drift_m"] == pytest.approx(0.0)
    assert result["pose"][
        "max_radial_displacement_from_start_m"
    ] == pytest.approx(0.15)


@pytest.mark.parametrize(
    ("motion", "linear_x_mps", "angular_z_radps", "moving_wheels"),
    [
        ("forward", 0.2, 0.0, (2.0, 2.0, 2.0, 2.0)),
        ("backward", -0.2, 0.0, (-2.0, -2.0, -2.0, -2.0)),
        ("rotate_left", 0.0, 0.2, (-2.0, 2.0, -2.0, 2.0)),
        ("rotate_right", 0.0, -0.2, (2.0, -2.0, 2.0, -2.0)),
        ("arc_left", 0.2, 0.2, (1.0, 2.0, 1.0, 2.0)),
        ("arc_right", 0.2, -0.2, (2.0, 1.0, 2.0, 1.0)),
    ],
)
def test_every_motion_kind_reports_finite_non_negative_maximum_excursion(
    motion, linear_x_mps, angular_z_radps, moving_wheels
):
    start = 1_000_000_000
    middle = 1_500_000_000
    end = 2_000_000_000
    odom = [
        OdomSample(start, -0.2, 0.3, 0, linear_x_mps, 0, angular_z_radps),
        OdomSample(middle, -0.1, 0.4, 0, linear_x_mps, 0, angular_z_radps),
        OdomSample(end, 0.0, 0.5, 0, linear_x_mps, 0, angular_z_radps),
        OdomSample(end + 100_000_000, 0.0, 0.5, 0, 0, 0, 0),
        OdomSample(end + 600_000_000, 0.0, 0.5, 0, 0, 0, 0),
    ]
    joints = [
        _joint(start, moving_wheels),
        _joint(middle, moving_wheels),
        _joint(end, moving_wheels),
        _joint(end + 100_000_000, (0, 0, 0, 0)),
        _joint(end + 600_000_000, (0, 0, 0, 0)),
    ]

    result = analyse_motion_segment(
        MotionSegment(
            motion,
            motion,
            "test",
            linear_x_mps,
            angular_z_radps,
            1,
        ),
        start,
        end,
        odom,
        joints,
        WHEELS,
        STOP,
        command_publish_count=20,
        max_sample_age_sec=0.5,
        timestamp_integrity={},
    )

    maximum_excursion = result["pose"][
        "max_radial_displacement_from_start_m"
    ]
    assert math.isfinite(maximum_excursion)
    assert maximum_excursion >= 0.0


@pytest.mark.parametrize(
    ("reverse_transient", "expected_direction", "expected_match"),
    (
        (0.31, "mixed", False),
        (0.19, "negative", True),
    ),
)
def test_rotation_direction_classification_preserves_deadband_transients(
    reverse_transient, expected_direction, expected_match
):
    """Producer retains above-deadband reversals without misclassifying noise."""
    start = 1_000_000_000
    middle = 1_500_000_000
    end = 2_000_000_000
    odom = [
        OdomSample(start, 0, 0, 0, 0, 0, 0.5),
        OdomSample(middle, 0, 0, 0.25, 0, 0, 0.5),
        OdomSample(end, 0, 0, 0.5, 0, 0, 0.5),
        OdomSample(end + 100_000_000, 0, 0, 0.5, 0, 0, 0),
        OdomSample(end + 600_000_000, 0, 0, 0.5, 0, 0, 0),
    ]
    joints = [
        _joint(start, (-1, 1, -1, 1)),
        _joint(middle, (reverse_transient, 1, -1, 1)),
        _joint(end, (-1, 1, -1, 1)),
        _joint(end + 100_000_000, (0, 0, 0, 0)),
        _joint(end + 600_000_000, (0, 0, 0, 0)),
    ]
    result = analyse_motion_segment(
        MotionSegment("left", "rotate_left", "test", 0, 0.5, 1),
        start,
        end,
        odom,
        joints,
        WHEELS,
        STOP,
        command_publish_count=20,
        max_sample_age_sec=0.5,
        timestamp_integrity={},
    )

    front_left = result["wheels"]["per_wheel"][WHEELS.front_left]
    assert front_left["direction"] == expected_direction
    assert front_left["direction_matches"] is expected_match
    assert result["wheels"]["all_directions_match"] is expected_match
    assert front_left["speed_radps"]["mean"] < -STOP.wheel_velocity_threshold_radps


def _reset_success_message(
    *, generation: int = 7, boundary_clock_ns: int = 1_000_000_000
) -> str:
    metadata = json.dumps(
        {
            "boundary_clock_ns": boundary_clock_ns,
            "generation": generation,
            "schema_version": 1,
        },
        sort_keys=True,
        separators=(",", ":"),
    )
    return (
        "simulation reset transaction complete: pose=mapping_start; "
        "reset_event emitted after all queued ROS reset calls completed; "
        f"reset_metadata_v1={metadata}"
    )


def test_reset_response_metadata_requires_anchored_versioned_trailer():
    message = (
        "pose=generation=999,boundary_clock_ns=0; "
        + _reset_success_message(generation=7, boundary_clock_ns=1_250_000_000)
    )
    assert _parse_reset_response_metadata(message) == (7, 1_250_000_000)
    assert _parse_reset_response_metadata(
        _reset_success_message(generation=1, boundary_clock_ns=0)
    ) == (1, 0)

    old_injectable_message = (
        "simulation reset transaction complete: "
        "pose=boundary_clock_ns=0, generation=12; reset_event emitted"
    )
    with pytest.raises(RuntimeError, match="versioned metadata trailer"):
        _parse_reset_response_metadata(old_injectable_message)


@pytest.mark.parametrize(
    "payload,match",
    [
        ("not-json", "valid JSON"),
        (json.dumps({"schema_version": 1}), "invalid schema"),
        (
            json.dumps(
                {
                    "schema_version": 2,
                    "generation": 1,
                    "boundary_clock_ns": 0,
                }
            ),
            "unsupported",
        ),
        (
            json.dumps(
                {
                    "schema_version": True,
                    "generation": 1,
                    "boundary_clock_ns": 0,
                }
            ),
            "schema_version must be an integer",
        ),
        (
            json.dumps(
                {
                    "schema_version": 1.0,
                    "generation": 1,
                    "boundary_clock_ns": 0,
                }
            ),
            "schema_version must be an integer",
        ),
        (
            '{"schema_version":2,"schema_version":1,'
            '"generation":1,"boundary_clock_ns":0}',
            "valid JSON",
        ),
        (
            json.dumps(
                {
                    "schema_version": 1,
                    "generation": 0,
                    "boundary_clock_ns": 0,
                }
            ),
            "must be positive",
        ),
        (
            json.dumps(
                {
                    "schema_version": 1,
                    "generation": True,
                    "boundary_clock_ns": 0,
                }
            ),
            "must be integers",
        ),
        (
            json.dumps(
                {
                    "schema_version": 1,
                    "generation": 1,
                    "boundary_clock_ns": -1,
                }
            ),
            "must be non-negative",
        ),
        (
            json.dumps(
                {
                    "schema_version": 1,
                    "generation": 1,
                    "boundary_clock_ns": 1.5,
                }
            ),
            "must be integers",
        ),
    ],
)
def test_reset_response_metadata_fails_closed(payload, match):
    with pytest.raises(RuntimeError, match=match):
        _parse_reset_response_metadata(
            f"reset complete; reset_metadata_v1={payload}"
        )


def test_coherent_group_requires_new_evidence_from_all_three_streams():
    credited = (10, 20, 30)
    assert _coherent_group_ready((11, 20, 30), credited) is False
    assert _coherent_group_ready((11, 21, 30), credited) is False
    assert _coherent_group_ready((99, 20, 30), credited) is False
    assert _coherent_group_ready((11, 21, 31), credited) is True

    credited = (11, 21, 31)
    assert _coherent_group_ready((99, 21, 31), credited) is False
    assert _coherent_group_ready((99, 22, 31), credited) is False
    assert _coherent_group_ready((99, 22, 32), credited) is True
    with pytest.raises(ValueError):
        _coherent_group_ready((1, 2), (0, 0, 0))


def test_post_reset_observation_requires_every_stamp_after_boundary():
    boundary = 1_000_000_000
    assert _post_reset_observation_ns(
        boundary_clock_ns=boundary,
        clock_ns=boundary + 3,
        odom_stamp_ns=boundary + 2,
        joint_stamp_ns=boundary + 1,
    ) == boundary + 1
    for stamps in (
        (None, boundary + 1, boundary + 1),
        (boundary, boundary + 1, boundary + 1),
        (boundary + 1, boundary - 1, boundary + 1),
        (boundary + 1, boundary + 1, boundary),
    ):
        assert _post_reset_observation_ns(
            boundary_clock_ns=boundary,
            clock_ns=stamps[0],
            odom_stamp_ns=stamps[1],
            joint_stamp_ns=stamps[2],
        ) is None


def test_per_topic_timestamp_regression_cannot_hide_behind_aggregate_minimum():
    high_watermarks = (1_100_000_000, 1_110_000_000, 1_100_000_000)
    current = (1_120_000_000, 1_105_000_000, 1_120_000_000)
    assert _timestamp_regression_topics(current, high_watermarks) == ("odom",)
    assert _timestamp_regression_topics(
        (1_120_000_000, 1_120_000_000, 1_120_000_000),
        high_watermarks,
    ) == ()


def test_stream_freshness_accepts_one_tick_callback_phase_only():
    """A bounded sensor-first DDS callback is coherent, not a stale stream."""
    runner = object.__new__(MotionBaselineRunner)
    runner._clock_ns = 2_000_000_000
    runner._latest_odom = OdomSample(
        2_016_666_667,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )
    runner._latest_joint = JointSample.from_mapping(
        2_000_000_000,
        {name: 0.0 for name in WHEELS.ordered_names},
    )
    now = time.monotonic()
    runner._last_clock_received_wall = now
    runner._last_odom_received_wall = now
    runner._last_joint_received_wall = now
    runner._config = SimpleNamespace(
        sampling=SimpleNamespace(
            max_sample_age_sec=0.5,
            max_future_skew_sec=0.02,
        ),
        stop=STOP,
        wheels=WHEELS,
    )

    assert runner._stream_sim_ages_ns()["odom"] == -16_666_667
    assert runner._streams_fresh() is True
    assert runner._latest_stationary() is True

    runner._latest_odom = OdomSample(
        2_020_000_001,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )
    assert runner._streams_fresh() is False
    assert runner._stream_freshness_gate_status()[
        "odom_not_too_far_ahead"
    ] is False

    runner._latest_odom = OdomSample(
        2_000_000_000,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )
    runner._latest_joint = JointSample.from_mapping(
        1_499_999_999,
        {name: 0.0 for name in WHEELS.ordered_names},
    )
    assert runner._streams_fresh() is False
    assert runner._stream_freshness_gate_status()[
        "joint_states_not_stale"
    ] is False


def test_stationary_window_advances_on_clock_and_motion_resets_it():
    """Coherent groups advance a window; a real motion gate failure does not."""
    stationary, last_clock, settled, status = _update_stationary_window(
        observation_ns=1_100_000_000,
        last_observation_ns=1_000_000_000,
        stationary_since_ns=None,
        gates_passed=True,
    )
    assert (stationary, last_clock, settled, status) == (
        1_100_000_000,
        1_100_000_000,
        0,
        "advanced",
    )

    stationary, last_clock, settled, status = _update_stationary_window(
        observation_ns=1_100_000_000,
        last_observation_ns=last_clock,
        stationary_since_ns=stationary,
        gates_passed=True,
    )
    assert stationary == 1_100_000_000
    assert settled == 0
    assert status == "waiting_for_observation"

    stationary, last_clock, settled, status = _update_stationary_window(
        observation_ns=1_100_000_000,
        last_observation_ns=last_clock,
        stationary_since_ns=stationary,
        gates_passed=False,
    )
    assert stationary is None
    assert settled == 0
    assert status == "blocked"

    stationary, last_clock, settled, status = _update_stationary_window(
        observation_ns=1_200_000_000,
        last_observation_ns=last_clock,
        stationary_since_ns=stationary,
        gates_passed=True,
    )
    assert stationary == 1_200_000_000
    assert status == "advanced"
    stationary, last_clock, settled, status = _update_stationary_window(
        observation_ns=1_700_000_000,
        last_observation_ns=last_clock,
        stationary_since_ns=stationary,
        gates_passed=True,
    )
    assert settled == 500_000_000
    assert status == "advanced"


def test_stationary_window_never_recredits_time_across_motion_or_regression():
    stationary, high_watermark, _, status = _update_stationary_window(
        observation_ns=1_000_000_000,
        last_observation_ns=900_000_000,
        stationary_since_ns=None,
        gates_passed=True,
    )
    assert status == "advanced"

    stationary, high_watermark, _, status = _update_stationary_window(
        observation_ns=1_400_000_000,
        last_observation_ns=high_watermark,
        stationary_since_ns=stationary,
        gates_passed=False,
    )
    assert (stationary, high_watermark, status) == (
        None,
        1_400_000_000,
        "blocked",
    )

    stationary, high_watermark, settled, status = _update_stationary_window(
        observation_ns=1_100_000_000,
        last_observation_ns=high_watermark,
        stationary_since_ns=stationary,
        gates_passed=True,
    )
    assert (stationary, high_watermark, settled, status) == (
        None,
        1_400_000_000,
        0,
        "observation_regression",
    )

    stationary, high_watermark, settled, status = _update_stationary_window(
        observation_ns=1_600_000_000,
        last_observation_ns=high_watermark,
        stationary_since_ns=stationary,
        gates_passed=True,
    )
    assert (stationary, high_watermark, settled, status) == (
        1_600_000_000,
        1_600_000_000,
        0,
        "advanced",
    )


def test_reset_recovery_uses_post_boundary_interleaved_coherent_groups():
    boundary = 1_000_000_000
    tick_ns = 16_666_667
    events: list[tuple[str, int]] = [
        # These callbacks were queued before the service response.  Sequence
        # numbers are new, but the service boundary must reject the samples.
        ("odom", boundary),
        ("clock", boundary),
        ("joint", boundary),
    ]
    # Odom reaches the response-window high while Clock/Joint lag by one tick.
    # Per-topic watermarks pass, but min(stamps) is before known movement and
    # must be rejected by the conservative observation floor.
    events.extend(
        [
            ("odom", boundary + 10 * tick_ns),
            ("clock", boundary + 9 * tick_ns),
            ("joint", boundary + 9 * tick_ns),
        ]
    )
    first_tick = boundary + tick_ns
    events.extend(
        [("odom", first_tick), ("clock", first_tick), ("joint", first_tick)]
    )
    # Clock alone advances beyond max_sample_age_sec.  It must neither credit
    # the same Odom/Joint samples nor let the later catch-up bridge the gap.
    for tick in range(2, 33):
        events.append(("clock", boundary + tick * tick_ns))
    catch_up_tick = boundary + 32 * tick_ns
    events.extend([("odom", catch_up_tick), ("joint", catch_up_tick)])
    orders = (
        ("joint", "odom", "clock"),
        ("clock", "joint", "odom"),
        ("odom", "clock", "joint"),
    )
    for tick in range(33, 82):
        stamp_ns = boundary + tick * tick_ns
        if tick == 40:
            # A short Odom regression is overwritten before Clock/Joint make
            # the group coherent.  The receive-level latch must still break
            # the settle window.
            events.extend(
                [
                    ("odom", boundary + 38 * tick_ns),
                    ("odom", stamp_ns),
                    ("clock", stamp_ns),
                    ("joint", stamp_ns),
                ]
            )
        elif tick == 50:
            # A later moving sample raises Odom's receive high-watermark.
            # A regressed stationary sample then completes the coherent group;
            # that group must be consumed and rejected rather than credited.
            events.extend(
                [
                    ("odom_moving", stamp_ns),
                    ("clock", stamp_ns),
                    ("odom", boundary + 49 * tick_ns + 1),
                    ("joint", stamp_ns),
                ]
            )
        else:
            events.extend((topic, stamp_ns) for topic in orders[tick % 3])

    response = SimpleNamespace(
        success=True,
        message=_reset_success_message(
            generation=7, boundary_clock_ns=boundary
        ),
    )

    class FakeFuture:
        def result(self):
            return response

    class FakeResetClient:
        @staticmethod
        def wait_for_service(timeout_sec):
            assert timeout_sec == pytest.approx(1.0)
            return True

        @staticmethod
        def call_async(request):
            del request
            return FakeFuture()

    runner = object.__new__(MotionBaselineRunner)
    runner._config = SimpleNamespace(
        reset=SimpleNamespace(
            service="/simulation/reset",
            service_timeout_sec=1.0,
            recovery_timeout_sec=1.0,
            settle_duration_sec=0.5,
        ),
        sampling=SimpleNamespace(
            publish_rate_hz=20.0,
            max_sample_age_sec=0.5,
            max_future_skew_sec=0.02,
        ),
        stop=STOP,
        wheels=WHEELS,
    )
    runner._reset_client = FakeResetClient()
    runner._clock_sequence = 0
    runner._odom_sequence = 0
    runner._joint_sequence = 0
    runner._clock_ns = boundary - tick_ns
    runner._latest_odom = OdomSample(
        boundary - tick_ns, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    )
    runner._latest_joint = JointSample.from_mapping(
        boundary - tick_ns,
        {name: 0.0 for name in WHEELS.ordered_names},
    )
    received_wall = time.monotonic()
    runner._last_clock_received_wall = received_wall
    runner._last_odom_received_wall = received_wall
    runner._last_joint_received_wall = received_wall
    runner._assert_command_channel_uncontended = lambda: None
    runner.safe_stop = lambda: None
    runner._publish = lambda linear, angular: None

    def wait_future(future, deadline):
        del future
        assert deadline > time.monotonic()
        # The service event is already committed, but its response has not yet
        # reached the client.  A moving post-boundary Odom callback handled in
        # this window belongs to the sequence barrier and must still seed the
        # receive timestamp high-watermark.
        stamp_ns = boundary + 10 * tick_ns
        runner._observe_reset_wait_timestamp("odom", stamp_ns)
        runner._latest_odom = OdomSample(
            stamp_ns, 0.0, 0.0, 0.0, 0.03, 0.0, 0.0
        )
        runner._odom_sequence += 1
        runner._last_odom_received_wall = time.monotonic()
        # The latest callback at response time is older and stationary.  The
        # scoped Reset-wait maximum must retain the overwritten moving stamp.
        regressed_stamp_ns = boundary + tick_ns
        runner._observe_reset_wait_timestamp("odom", regressed_stamp_ns)
        runner._latest_odom = OdomSample(
            regressed_stamp_ns, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        )
        runner._odom_sequence += 1
        runner._last_odom_received_wall = time.monotonic()
        return True

    runner._wait_future = wait_future

    event_index = 0

    def spin_once(timeout_sec):
        nonlocal event_index
        assert timeout_sec >= 0.0
        assert event_index < len(events), "Reset recovered without enough evidence"
        topic, stamp_ns = events[event_index]
        event_index += 1
        now = time.monotonic()
        if topic == "clock":
            runner._clock_ns = stamp_ns
            runner._clock_sequence += 1
            runner._last_clock_received_wall = now
        elif topic.startswith("odom"):
            runner._latest_odom = OdomSample(
                stamp_ns,
                0.0,
                0.0,
                0.0,
                0.03 if topic == "odom_moving" else 0.0,
                0.0,
                0.0,
            )
            runner._odom_sequence += 1
            runner._last_odom_received_wall = now
        else:
            runner._latest_joint = JointSample.from_mapping(
                stamp_ns,
                {name: 0.0 for name in WHEELS.ordered_names},
            )
            runner._joint_sequence += 1
            runner._last_joint_received_wall = now

    runner._spin_once = spin_once

    report = runner._reset_and_wait()

    assert event_index == len(events)
    assert report["reset_generation"] == 7
    assert report["reset_boundary_clock_ns"] == boundary
    assert report["fresh_clock_received"] is True
    assert report["fresh_odom_received"] is True
    assert report["fresh_joint_states_received"] is True
    assert report["recovery_observation_counts"]["pre_boundary_group"] == 1
    assert report["recovery_observation_counts"]["observation_regression"] == 1
    assert report["recovery_observation_counts"]["stationary"] == 49
    assert (
        report["recovery_observation_counts"]["receive_timestamp_regression"]
        == 13
    )
    assert (
        report["recovery_violation_counts"][
            "receive_timestamp_regression:odom"
        ]
        == 4
    )
    assert (
        report["recovery_violation_counts"][
            "coherent_timestamp_regression:odom"
        ]
        == 2
    )
    assert report["recovery_violation_counts"]["odom_linear_speed"] > 0
    assert report["recovery_violation_counts"]["stream:odom_not_stale"] > 0
    assert (
        report["recovery_violation_counts"]["stream:joint_states_not_stale"]
        > 0
    )
    assert report["longest_stationary_duration_sec"] == pytest.approx(0.5)


def test_reset_recovery_diagnostic_identifies_freshness_and_motion_blockers():
    """A Reset timeout retains the exact stream ages and velocity gates."""
    runner = object.__new__(MotionBaselineRunner)
    runner._clock_sequence = 8
    runner._odom_sequence = 9
    runner._joint_sequence = 10
    runner._clock_ns = 2_000_000_000
    runner._latest_odom = OdomSample(
        1_400_000_000,
        0.0,
        0.0,
        0.0,
        0.03,
        0.04,
        0.06,
    )
    runner._latest_joint = JointSample.from_mapping(
        1_900_000_000,
        {
            WHEELS.front_left: 0.3,
            WHEELS.front_right: 0.0,
            WHEELS.rear_left: 0.0,
            WHEELS.rear_right: 0.0,
        },
    )
    now = time.monotonic()
    runner._last_clock_received_wall = now - 0.1
    runner._last_odom_received_wall = now - 0.1
    runner._last_joint_received_wall = now - 0.1
    runner._config = SimpleNamespace(
        sampling=SimpleNamespace(
            max_sample_age_sec=0.5,
            max_future_skew_sec=0.02,
        ),
        stop=STOP,
        wheels=WHEELS,
    )

    diagnostic = runner._reset_recovery_diagnostic(
        (7, 8, 9),
        (8, 9, 10),
        (2_000_000_000, 1_400_000_000, 1_900_000_000),
        (2_000_000_000, 1_400_000_000, 1_900_000_000),
        3,
        1_000_000_000,
        {"sequence_not_fresh": 1, "not_stationary": 2, "stationary": 3},
        {
            "streams_fresh": 2,
            "odom_linear_speed": 2,
            "odom_angular_speed": 1,
            f"wheel:{WHEELS.front_left}": 2,
        },
        {
            "odom_linear_speed_mps": 0.1,
            "odom_angular_speed_radps": 0.2,
            "wheel_abs_speed_radps": {WHEELS.front_left: 0.4},
        },
        250_000_000,
    )

    assert diagnostic["fresh_sequences"] == {
        "clock": True,
        "odom": True,
        "joint_states": True,
    }
    assert diagnostic["sim_age_sec"] == {"odom": 0.6, "joint_states": 0.1}
    assert diagnostic["streams_fresh"] is False
    assert diagnostic["wall_streams_fresh"] is True
    assert diagnostic["stationary_now"] is False
    assert diagnostic["terminal_blockers"] == [
        "odom_angular_speed",
        "odom_linear_speed",
        "stream:odom_not_stale",
        "streams_fresh",
        f"wheel:{WHEELS.front_left}",
    ]
    assert diagnostic["sim_timestamp_span_sec"] == pytest.approx(0.6)
    assert diagnostic["violation_counts"]["odom_linear_speed"] == 2
    assert diagnostic["peak_observed"]["odom_linear_speed_mps"] == 0.1
    assert diagnostic["longest_stationary_duration_sec"] == 0.25
    assert diagnostic["odom"]["linear_speed_mps"] == pytest.approx(0.05)
    assert diagnostic["joint_states"]["wheel_abs_speed_radps"][
        WHEELS.front_left
    ] == pytest.approx(0.3)


def test_reset_recovery_diagnostic_remains_strict_json_after_float_overflow():
    """Finite message fields may overflow a derived hypot without hiding timeout."""
    runner = object.__new__(MotionBaselineRunner)
    runner._clock_sequence = 2
    runner._odom_sequence = 2
    runner._joint_sequence = 2
    runner._clock_ns = 2_000_000_000
    runner._latest_odom = OdomSample(
        2_000_000_000,
        0.0,
        0.0,
        0.0,
        float.fromhex("0x1.fffffffffffffp+1023"),
        float.fromhex("0x1.fffffffffffffp+1023"),
        0.0,
    )
    runner._latest_joint = JointSample.from_mapping(
        2_000_000_000,
        {name: 0.0 for name in WHEELS.ordered_names},
    )
    now = time.monotonic()
    runner._last_clock_received_wall = now
    runner._last_odom_received_wall = now
    runner._last_joint_received_wall = now
    runner._config = SimpleNamespace(
        sampling=SimpleNamespace(
            max_sample_age_sec=0.5,
            max_future_skew_sec=0.02,
        ),
        stop=STOP,
        wheels=WHEELS,
    )

    diagnostic = runner._reset_recovery_diagnostic(
        (1, 1, 1),
        (2, 2, 2),
        (2_000_000_000, 2_000_000_000, 2_000_000_000),
        (2_000_000_000, 2_000_000_000, 2_000_000_000),
        4,
        1_000_000_000,
        {"sequence_not_fresh": 0, "not_stationary": 1, "stationary": 0},
        {"odom_linear_speed": 1},
        {"odom_linear_speed_mps": math.inf},
        0,
    )

    assert diagnostic["odom"]["linear_speed_mps"] == "non_finite:+inf"
    assert diagnostic["peak_observed"]["odom_linear_speed_mps"] == (
        "non_finite:+inf"
    )
    json.dumps(diagnostic, allow_nan=False)


def test_arc_analysis_keeps_intended_lateral_displacement_distinct_from_drift():
    start = 1_000_000_000
    end = 6_000_000_000
    odom = [
        OdomSample(start, 0, 0, 0, 0.4, 0, 0.4),
        OdomSample(3_500_000_000, 0.45, 0.7, 1.0, 0.4, 0, 0.4),
        OdomSample(end, 0.9, 1.4, 2.0, 0.4, 0, 0.4),
        OdomSample(end + 100_000_000, 0.9, 1.4, 2.0, 0, 0, 0),
        OdomSample(end + 600_000_000, 0.9, 1.4, 2.0, 0, 0, 0),
    ]
    joints = [
        _joint(start, (3, 5, 3, 5)),
        _joint(3_500_000_000, (3, 5, 3, 5)),
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
        max_sample_age_sec=2.5,
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
            max_sample_age_sec=1.0,
            timestamp_integrity={},
        )
