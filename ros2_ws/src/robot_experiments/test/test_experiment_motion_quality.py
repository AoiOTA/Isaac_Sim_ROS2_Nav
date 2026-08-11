import hashlib
import json
from types import SimpleNamespace

import pytest

from robot_experiments.experiment_runner import (
    _edge_prior_statistics,
    _record_tracked_route_length,
    _strict_success_from_leg_count,
    CommandSample,
    ExperimentRunner,
    OdometrySample,
    _dynamic_interaction_acceptance,
)


def test_tracked_route_length_replaces_untrimmed_canonical_edge_sum():
    routes = [{"request_id": 7, "planned_length_m": 14.65}]

    _record_tracked_route_length(routes, 7, 0.07, 11.87)
    _record_tracked_route_length(routes, 7, 11.88, 0.07)

    assert routes[0]["canonical_full_edge_length_m"] == pytest.approx(14.65)
    assert routes[0]["tracked_route_length_m"] == pytest.approx(11.95)
    assert routes[0]["planned_length_m"] == pytest.approx(11.95)


def test_edge_prior_statistics_preserve_nonzero_learned_cost_evidence():
    priors = [
        SimpleNamespace(cost_delta_m=0.0, learned_risk=0.0),
        SimpleNamespace(cost_delta_m=0.55, learned_risk=1.0),
        SimpleNamespace(cost_delta_m=0.07, learned_risk=0.58),
    ]

    assert _edge_prior_statistics(priors) == {
        "prior_count": 3,
        "positive_cost_count": 2,
        "total_cost_delta_m": pytest.approx(0.62),
        "maximum_cost_delta_m": pytest.approx(0.55),
        "maximum_learned_risk": pytest.approx(1.0),
    }


def test_strict_success_counts_single_goal_when_route_is_omitted():
    assert _strict_success_from_leg_count("success", 1, 0)
    assert _strict_success_from_leg_count("success", 5, 5)
    assert not _strict_success_from_leg_count("success", 0, 0)
    assert not _strict_success_from_leg_count("failure", 1, 0)


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


def test_g2_g3_exit_accepts_calibrated_one_point_four_metre_following_gap():
    ground_truth = [
        OdometrySample(-0.42, 2.40, 0.0, 0.4, 0.0, 1.00, 0.0),
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
    assert metrics["complete"]


def test_g2_g3_exit_rejects_a_following_gap_beyond_the_calibrated_window():
    ground_truth = [
        OdometrySample(-0.42, 2.41, 0.0, 0.4, 0.0, 1.00, 0.0),
        OdometrySample(-0.85, -0.55, 0.0, 0.4, 0.0, 1.10, 0.0),
    ]
    actor = [
        {"id": "g2_g3_exit_actor", "state": "moving", "stamp_s": 1.00, "position": [-0.40, 1.00, 0.5]},
        {"id": "g2_g3_exit_actor", "state": "parked", "stamp_s": 1.10, "position": [-0.40, -0.70, 0.5]},
    ]

    metrics = ExperimentRunner._g2_g3_exit_metrics(
        ground_truth, actor, "g2_g3_exit_actor"
    )

    assert not metrics["continuous_follow_seen"]
    assert not metrics["complete"]


def test_failed_pilot_evidence_is_retried_only_when_successful_resume_is_required(tmp_path):
    root = tmp_path / "run-0002-seed-7301"
    root.mkdir()
    manifest = {
        "random_seed": 7301,
        "run_index": 2,
        "condition_id": "dynamic_appearance",
        "appearance": {"profile_id": "dim_warm"},
        "dynamic_selection": {"case_id": "full_route_three_stage", "variant_id": "v1"},
        "result": "failure",
    }
    summary = {"data_complete": True, "checksums_verified": True}
    (root / "run_manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
    (root / "run_summary.json").write_text(json.dumps(summary), encoding="utf-8")
    checksums = [
        f"{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.name}"
        for path in sorted(root.iterdir())
    ]
    (root / "checksums.sha256").write_text("\n".join(checksums) + "\n", encoding="utf-8")
    selection = SimpleNamespace(
        seed=7301,
        condition_id="dynamic_appearance",
        appearance_profile_id="dim_warm",
        case_id="full_route_three_stage",
        variant_id="v1",
    )
    runner = object.__new__(ExperimentRunner)

    runner._require_successful_resume = False
    assert runner._completed_resume_manifest(root, 2, selection) == manifest
    runner._require_successful_resume = True
    assert runner._completed_resume_manifest(root, 2, selection) is None


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


def test_focused_dynamic_case_skips_unselected_intermediate_goal_groups():
    runner = object.__new__(ExperimentRunner)
    runner._scenario = SimpleNamespace(
        scenario_type="dynamic",
        obstacle_trajectories=(
            {"id": "local", "motion": "local_bypass", "trigger_group": "G2"},
            {"id": "exit", "motion": "g2_g3_exit", "trigger_group": "G3"},
            {"id": "door", "motion": "g5_g1_crossing", "trigger_group": "G1"},
        ),
    )
    runner._active_selection = SimpleNamespace(case_id="g2_g3_exit")

    assert runner._selected_dynamic_groups_for_goal("G2") == []
    assert runner._selected_dynamic_groups_for_goal("G3") == ["G3"]

    runner._active_selection = SimpleNamespace(case_id="full_route_three_stage")
    assert runner._selected_dynamic_groups_for_goal("G2") == ["G2"]
    assert runner._selected_dynamic_groups_for_goal("G3") == ["G3"]
    assert runner._selected_dynamic_groups_for_goal("G1") == ["G1"]


def test_runner_has_no_actor_lifecycle_costmap_clear_workaround():
    assert not hasattr(
        ExperimentRunner, "_request_pending_dynamic_trail_clears"
    )


def test_collision_free_policy_keeps_low_clearance_as_warning():
    actor_ids = {"local", "exit", "door"}
    status = _dynamic_interaction_acceptance(
        scenario_type="dynamic",
        expected_ids=actor_ids,
        triggered_ids=actor_ids,
        completed_ids=actor_ids,
        retired_ids=actor_ids,
        clearance_by_actor={"local": 0.0, "exit": 0.69, "door": 0.23},
        evidence_complete=True,
    )

    assert status["complete"] is True
    assert status["minimum_clearance_complete"] is True
    assert status["clearance_warning_below_0_10m"] is True
    assert status["minimum_clearance_requirement_m"] == 0.0
    assert status["acceptance_policy"] == "physical_collision_free"


@pytest.mark.parametrize(
    ("triggered", "clearance", "evidence_complete"),
    [
        ({"local", "exit"}, {"local": 0.2, "exit": 0.2, "door": 0.2}, True),
        ({"local", "exit", "door"}, {"local": 0.2, "exit": 0.2}, True),
        ({"local", "exit", "door"}, {"local": 0.2, "exit": 0.2, "door": 0.2}, False),
    ],
)
def test_collision_free_policy_still_requires_complete_interaction_evidence(
    triggered, clearance, evidence_complete
):
    actor_ids = {"local", "exit", "door"}
    status = _dynamic_interaction_acceptance(
        scenario_type="dynamic",
        expected_ids=actor_ids,
        triggered_ids=triggered,
        completed_ids=actor_ids,
        retired_ids=actor_ids,
        clearance_by_actor=clearance,
        evidence_complete=evidence_complete,
    )

    assert status["complete"] is False
