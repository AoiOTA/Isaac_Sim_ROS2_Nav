from dataclasses import replace
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
from nav2_msgs.msg import CollisionMonitorState

import robot_experiments.experiment_runner as experiment_runner_module
from robot_experiments.attempt31_rivermark_qualification import _rate_group
from robot_experiments.experiment_runner import (
    _edge_prior_statistics,
    _parse_obstacle_completion,
    _record_tracked_route_length,
    _result_with_terminal_zero,
    _strict_success_from_leg_count,
    CommandSample,
    ExperimentRunner,
    OdometrySample,
    _dynamic_interaction_acceptance,
    _reset_dynamic_selection,
)
from robot_experiments.configuration import ConfigurationError
from robot_experiments.scenario import RunSelection, load_scenario


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


def test_obstacle_completion_requires_the_exact_selected_actor_set():
    payload = json.dumps({"group": "G2", "retired": ["dynamic_box"]})
    assert _parse_obstacle_completion(
        payload,
        expected_group="G2",
        expected_ids={"dynamic_box"},
    ) == ("dynamic_box",)

    with pytest.raises(RuntimeError, match="retired IDs mismatch"):
        _parse_obstacle_completion(
            json.dumps({"group": "G2", "retired": []}),
            expected_group="G2",
            expected_ids={"dynamic_box"},
        )


def test_dynamic_retirement_clearance_requires_fresh_empty_source_and_two_zero_consumers():
    def stamp(value):
        return SimpleNamespace(sec=int(value), nanosec=0)

    runner = object.__new__(ExperimentRunner)
    runner._obstacle_state_stamp_s = 11.0
    runner._obstacle_state = {
        "obstacles": [{"id": "dynamic_box", "state": "retired"}],
        "events": [],
    }
    runner._latest_cognitive_obstacles = SimpleNamespace(
        sequence=9,
        reset_epoch=2,
        recurrent_session_id="session-2",
        map_version="map-v1",
        header=SimpleNamespace(stamp=stamp(11.0)),
        validation_stamp=stamp(11.0),
        obstacles=[],
    )
    zero_status = lambda consumer: SimpleNamespace(
        consumer=consumer,
        mode="active",
        applied=False,
        active_cell_count=0,
        fallback_reason="rejection_reason=no_costmap_cells",
        maximum_cost=0,
        raised_cell_count=0,
        maximum_cost_increase=0,
        source_sequence=9,
        reset_epoch=2,
        recurrent_session_id="session-2",
        map_version="map-v1",
        stamp=stamp(11.0),
    )
    runner._latest_cognitive_layer_statuses = {
        "/global_costmap/global_costmap:cognitive_obstacle_layer": zero_status(
            "global"
        ),
        "/local_costmap/local_costmap:cognitive_obstacle_layer": zero_status(
            "local"
        ),
    }

    assert runner._dynamic_retirement_clearance_observed(
        {"dynamic_box"}, 10.0, 8, {"global": 8, "local": 8}
    )
    runner._latest_cognitive_obstacles.obstacles = [object()]
    assert not runner._dynamic_retirement_clearance_observed(
        {"dynamic_box"}, 10.0, 8, {"global": 8, "local": 8}
    )


def test_dynamic_retirement_clearance_rejects_stale_or_mismatched_evidence():
    def stamp(value):
        return SimpleNamespace(sec=int(value), nanosec=0)

    source = SimpleNamespace(
        sequence=9,
        reset_epoch=2,
        recurrent_session_id="session-2",
        map_version="map-v1",
        header=SimpleNamespace(stamp=stamp(11.0)),
        validation_stamp=stamp(11.0),
        obstacles=[],
    )
    status = SimpleNamespace(
        mode="active",
        applied=False,
        active_cell_count=0,
        fallback_reason="rejection_reason=no_costmap_cells",
        maximum_cost=0,
        raised_cell_count=0,
        maximum_cost_increase=0,
        source_sequence=9,
        reset_epoch=2,
        recurrent_session_id="session-2",
        map_version="map-v1",
        stamp=stamp(11.0),
    )
    runner = object.__new__(ExperimentRunner)
    runner._obstacle_state_stamp_s = 11.0
    runner._obstacle_state = {
        "obstacles": [{"id": "dynamic_box", "state": "retired"}],
        "events": [],
    }
    runner._latest_cognitive_obstacles = source
    runner._latest_cognitive_layer_statuses = {
        "/global_costmap/global_costmap:cognitive_obstacle_layer": status,
    }

    assert not runner._dynamic_retirement_clearance_observed(
        {"dynamic_box"}, 10.0, 8, {"global": 8, "local": 8}
    )
    runner._latest_cognitive_layer_statuses[
        "/local_costmap/local_costmap:cognitive_obstacle_layer"
    ] = SimpleNamespace(**vars(status))
    source.sequence = 8
    assert not runner._dynamic_retirement_clearance_observed(
        {"dynamic_box"}, 10.0, 8, {"global": 8, "local": 8}
    )
    source.sequence = 9
    runner._latest_cognitive_layer_statuses[
        "/local_costmap/local_costmap:cognitive_obstacle_layer"
    ].reset_epoch = 1
    assert not runner._dynamic_retirement_clearance_observed(
        {"dynamic_box"}, 10.0, 8, {"global": 8, "local": 8}
    )
    runner._latest_cognitive_layer_statuses[
        "/local_costmap/local_costmap:cognitive_obstacle_layer"
    ].reset_epoch = 2
    runner._latest_cognitive_layer_statuses[
        "/local_costmap/local_costmap:cognitive_obstacle_layer"
    ].fallback_reason = "rejection_reason=offered"
    assert not runner._dynamic_retirement_clearance_observed(
        {"dynamic_box"}, 10.0, 8, {"global": 8, "local": 8}
    )


def test_completion_clears_both_costmaps_before_returning():
    class Future:
        @staticmethod
        def result():
            return SimpleNamespace(
                success=True,
                message=json.dumps(
                    {
                        "group": "G2",
                        "retired": ["v6_dynamic_g2_crossing_box"],
                    }
                ),
            )

    class Client:
        @staticmethod
        def wait_for_service(*, timeout_sec):
            return timeout_sec > 0.0

        @staticmethod
        def call_async(_request):
            return Future()

    runner = object.__new__(ExperimentRunner)
    runner._scenario = SimpleNamespace(
        scenario_type="dynamic",
        map_version="v6_kujiale_isaacgen_v1",
    )
    runner._nav2_profile = "v6_low_obstacle_isolation"
    runner._selected_dynamic_trajectories = lambda: (
        {"id": "v6_dynamic_g2_crossing_box", "trigger_group": "G2"},
    )
    runner._selected_dynamic_groups_for_goal = lambda _goal_id: ["G2"]
    runner._obstacle_complete_clients = {"G2": Client()}
    runner._service_timeout_sec = 1.0
    runner._wait_future = lambda _future, _deadline: True
    runner._latest_cognitive_obstacles = SimpleNamespace(sequence=8)
    runner._latest_cognitive_layer_statuses = {}
    runner._clock_seconds = lambda: 10.0
    cleared = []
    runner._clear_navigation_costmaps = lambda: cleared.append(True)
    runner._dynamic_retirement_clearance_observed = (
        lambda retired, _barrier, source_cursor, _status_cursors: (
            retired == {"v6_dynamic_g2_crossing_box"} and source_cursor == 8
        )
    )
    runner._wait_until = lambda predicate, _timeout: predicate()

    assert runner._complete_obstacle_group("G2") == (
        "v6_dynamic_g2_crossing_box",
    )
    assert cleared == [True]

    runner._wait_until = lambda _predicate, _timeout: False
    with pytest.raises(RuntimeError, match="did not clear"):
        runner._complete_obstacle_group("G2")


def test_noncanonical_dynamic_completion_does_not_enter_cognitive_clearance_gate():
    runner = object.__new__(ExperimentRunner)
    runner._nav2_profile = "dynamic_avoidance"
    runner._scenario = SimpleNamespace(
        scenario_type="dynamic",
        map_version="other-map",
    )
    assert not runner._requires_dynamic_retirement_clearance({"dynamic_box"})


def test_dynamic_manifest_requires_successful_completion_return(tmp_path):
    runner, _manifest, _summary, _root = _static_sat_evidence_run(
        tmp_path,
        obstacle_position=None,
        contact_sensor_collision=False,
    )
    scenario = load_scenario(
        Path(__file__).parents[1] / "config" / "dynamic.yaml"
    )
    runner._scenario = replace(
        scenario,
        obstacle_trajectories=({"id": "crossing_box", "motion": "crossing"},),
        success=replace(
            scenario.success,
            minimum_ground_truth_path_length_m=0.0,
            minimum_reverse_distance_m=0.0,
            maximum_reverse_distance_fraction=1.0,
            minimum_curved_distance_fraction=0.0,
            maximum_stopped_time_fraction=1.0,
        ),
    )
    runner._active_selection = RunSelection(7301, condition_id="dynamic")
    runner._depth_frame = {}
    runner._scan_frame = {}
    runner._local_costmap = object()
    runner._obstacle_samples = [{"id": "crossing_box", "min_clearance_m": 0.2}]
    runner._obstacle_events = [
        {"event": "armed", "obstacle_id": "crossing_box"},
        {"event": "motion_complete", "obstacle_id": "crossing_box"},
        {"event": "park", "obstacle_id": "crossing_box"},
    ]
    runner._leg_results = [{
        "id": "G1",
        "nav2_status": experiment_runner_module.GoalStatus.STATUS_ABORTED,
    }]

    failed = runner._build_manifest(
        run_index=3,
        seed=7301,
        nav2_succeeded=False,
        timed_out=False,
        nav2_status=experiment_runner_module.GoalStatus.STATUS_ABORTED,
        final_still=True,
        runner_error=None,
    )

    assert failed["dynamic_interaction"]["completed_ids"] == []
    assert failed["dynamic_interaction"]["retired_ids"] == []
    assert not failed["dynamic_interaction"]["complete"]
    assert "dynamic_obstacle_interaction_incomplete" in failed["failure_reason"]

    runner._completed_dynamic_obstacle_ids.update(("crossing_box",))
    runner._obstacle_events.append(
        {"event": "goal_reached_retire", "obstacle_id": "crossing_box"}
    )
    completed = runner._build_manifest(
        run_index=3,
        seed=7301,
        nav2_succeeded=True,
        timed_out=False,
        nav2_status=experiment_runner_module.GoalStatus.STATUS_SUCCEEDED,
        final_still=True,
        runner_error=None,
    )

    assert completed["dynamic_interaction"]["completed_ids"] == ["crossing_box"]
    assert completed["dynamic_interaction"]["retired_ids"] == ["crossing_box"]
    assert completed["dynamic_interaction"]["complete"]

    runner._clear_run_state()
    assert runner._completed_dynamic_obstacle_ids == set()


def test_strict_success_counts_single_goal_when_route_is_omitted():
    assert _strict_success_from_leg_count(
        "success", 1, 0, terminal_zero_confirmed=True
    )
    assert _strict_success_from_leg_count(
        "success", 5, 5, terminal_zero_confirmed=True
    )
    assert not _strict_success_from_leg_count(
        "success", 0, 0, terminal_zero_confirmed=True
    )
    assert not _strict_success_from_leg_count(
        "failure", 1, 0, terminal_zero_confirmed=True
    )
    assert not _strict_success_from_leg_count(
        "success", 5, 5, terminal_zero_confirmed=False
    )


def _twist(nonzero=False):
    return SimpleNamespace(
        linear=SimpleNamespace(x=0.2 if nonzero else 0.0, y=0.0),
        angular=SimpleNamespace(z=0.1 if nonzero else 0.0),
    )


def _terminal_zero_runner(
    monkeypatch,
    events,
    *,
    timeout_sec=0.65,
    odom_speed_at=None,
    barrier_source="route_goal_complete",
    barrier_at=0.0,
):
    clock = SimpleNamespace(now=10.0)
    monkeypatch.setattr(
        experiment_runner_module.time, "monotonic", lambda: clock.now
    )
    runner = object.__new__(ExperimentRunner)
    runner._clear_run_state()
    runner._scenario = SimpleNamespace(
        success=SimpleNamespace(
            final_still_timeout_sec=timeout_sec,
            final_still_duration_sec=0.15,
            final_linear_speed_mps=0.02,
            final_angular_speed_radps=0.05,
        )
    )
    runner._odom_max_age_sec = 0.5
    runner._raise_if_shutdown = lambda: None
    pending = list(events)

    def spin_once(timeout_sec):
        clock.now += timeout_sec
        if (
            barrier_source is not None
            and runner._terminal_zero_barrier_monotonic is None
            and clock.now >= 10.0 + barrier_at
        ):
            runner._mark_terminal_zero_barrier(barrier_source)
        speed = (
            odom_speed_at(clock.now - 10.0)
            if odom_speed_at is not None
            else 0.0
        )
        runner._odom_samples.append(
            OdometrySample(
                0.0, 0.0, 0.0, speed, 0.0, clock.now, clock.now
            )
        )
        while pending and clock.now >= 10.0 + pending[0][0]:
            _offset, nonzero = pending.pop(0)
            runner._actuator_command_callback(_twist(nonzero))

    runner._spin_once = spin_once
    runner._test_clock = clock
    runner._start_terminal_zero_observation()
    if barrier_source is not None and barrier_at == 0.0:
        runner._mark_terminal_zero_barrier(barrier_source)
    return runner


def test_terminal_zero_is_part_of_manifest_result_without_erasing_other_failures():
    assert _result_with_terminal_zero([], True) == ("success", [])
    assert _result_with_terminal_zero([], False) == (
        "failure",
        ["terminal_zero_not_confirmed"],
    )
    assert _result_with_terminal_zero(
        ["collision_detected", "timed_out"], False
    ) == (
        "failure",
        ["collision_detected", "timed_out", "terminal_zero_not_confirmed"],
    )


def test_terminal_zero_immediate_repeated_quiet_window_passes(monkeypatch):
    runner = _terminal_zero_runner(
        monkeypatch,
        ((0.05, False), (0.22, False)),
    )

    assert runner._wait_for_final_stillness()
    assert runner._terminal_zero_confirmed
    assert runner._terminal_zero_reason == "terminal_zero_confirmed"
    timing = runner._terminal_zero_timing()
    assert timing["barrier_source"] == "route_goal_complete"
    assert timing["first_zero_after_terminal_sec"] <= 0.10
    assert timing["last_zero_after_terminal_sec"] >= 0.20
    assert timing["confirming_zero_sample_count"] == 2


def test_terminal_zero_ignores_zeros_before_terminal_barrier(monkeypatch):
    runner = _terminal_zero_runner(monkeypatch, (), barrier_source=None)
    runner._test_clock.now = 10.02
    runner._actuator_command_callback(_twist())
    runner._test_clock.now = 10.18
    runner._actuator_command_callback(_twist())
    runner._test_clock.now = 10.20
    runner._mark_terminal_zero_barrier("route_goal_complete")

    assert not runner._terminal_zero_observation_complete(10.20, 0.15)
    assert runner._terminal_zero_reason == "terminal_zero_not_observed"
    assert runner._terminal_zero_timing()["observed_zero_sample_count"] == 0


def _expect_route_completion(runner, *, next_epoch, leg_id, final_leg):
    runner._route_goal_complete_epoch = next_epoch - 1
    runner._terminal_zero_expected_route_completion_epoch = next_epoch
    runner._terminal_zero_expected_route_leg_id = leg_id
    runner._terminal_zero_expected_route_leg_is_final = final_leg


def test_intermediate_success_clears_epoch_without_terminal_barrier(monkeypatch):
    runner = _terminal_zero_runner(monkeypatch, (), barrier_source=None)
    _expect_route_completion(
        runner, next_epoch=5, leg_id="G3", final_leg=False
    )
    runner._test_clock.now = 10.20

    runner._route_goal_complete_callback(SimpleNamespace(data=True))

    assert runner._route_goal_complete_epoch == 5
    assert runner._latest_route_goal_complete
    assert runner._terminal_zero_barrier_monotonic is None
    assert runner._terminal_zero_expected_route_completion_epoch is None
    assert runner._terminal_zero_expected_route_leg_id is None


def test_intermediate_false_records_episode_terminal_barrier(monkeypatch):
    runner = _terminal_zero_runner(monkeypatch, (), barrier_source=None)
    _expect_route_completion(
        runner, next_epoch=5, leg_id="G3", final_leg=False
    )
    runner._test_clock.now = 10.20

    runner._route_goal_complete_callback(SimpleNamespace(data=False))

    assert runner._terminal_zero_barrier_monotonic == pytest.approx(10.20)
    assert runner._terminal_zero_barrier_source == "route_goal_complete"
    assert runner._terminal_zero_barrier_leg_id == "G3"


def test_timeout_cancel_fresh_false_records_matching_leg_barrier(monkeypatch):
    runner = _terminal_zero_runner(monkeypatch, (), barrier_source=None)
    _expect_route_completion(
        runner, next_epoch=8, leg_id="G4", final_leg=False
    )
    runner._test_clock.now = 10.40

    runner._route_goal_complete_callback(SimpleNamespace(data=False))

    assert runner._route_goal_complete_epoch == 8
    assert runner._terminal_zero_barrier_monotonic == pytest.approx(10.40)
    assert runner._terminal_zero_barrier_leg_id == "G4"


def test_stale_duplicate_completion_is_ignored_after_epoch_consumed(monkeypatch):
    runner = _terminal_zero_runner(monkeypatch, (), barrier_source=None)
    _expect_route_completion(
        runner, next_epoch=5, leg_id="G3", final_leg=False
    )
    runner._route_goal_complete_callback(SimpleNamespace(data=True))
    runner._test_clock.now = 10.30

    runner._route_goal_complete_callback(SimpleNamespace(data=False))

    assert runner._route_goal_complete_epoch == 6
    assert runner._terminal_zero_barrier_monotonic is None


def test_fresh_final_success_records_terminal_barrier(monkeypatch):
    runner = _terminal_zero_runner(monkeypatch, (), barrier_source=None)
    _expect_route_completion(
        runner, next_epoch=5, leg_id="G1", final_leg=True
    )
    runner._test_clock.now = 10.20

    runner._route_goal_complete_callback(SimpleNamespace(data=True))

    assert runner._terminal_zero_barrier_monotonic == pytest.approx(10.20)
    assert runner._terminal_zero_barrier_source == "route_goal_complete"
    assert runner._terminal_zero_barrier_leg_id == "G1"


def test_terminal_zero_rejects_first_zero_later_than_100ms(monkeypatch):
    runner = _terminal_zero_runner(monkeypatch, ())
    runner._test_clock.now = 10.11
    runner._actuator_command_callback(_twist())
    runner._test_clock.now = 10.30
    runner._actuator_command_callback(_twist())

    assert not runner._terminal_zero_observation_complete(10.30, 0.15)
    assert runner._terminal_zero_reason == "terminal_first_zero_late"


def test_terminal_zero_rejects_any_nonzero_tail_after_barrier(monkeypatch):
    runner = _terminal_zero_runner(monkeypatch, ())
    for offset, nonzero in (
        (0.02, False),
        (0.08, True),
        (0.09, False),
        (0.30, False),
    ):
        runner._test_clock.now = 10.0 + offset
        runner._actuator_command_callback(_twist(nonzero))

    assert not runner._terminal_zero_observation_complete(10.30, 0.15)
    assert runner._terminal_zero_reason == "terminal_nonzero_after_barrier"


def test_terminal_zero_single_zero_then_timeout_does_not_erase_final_stillness(
    monkeypatch,
):
    runner = _terminal_zero_runner(
        monkeypatch, ((0.05, False),), timeout_sec=0.35
    )

    assert runner._wait_for_final_stillness()
    assert not runner._terminal_zero_confirmed
    assert runner._terminal_zero_reason == "terminal_zero_timeout"
    assert runner._terminal_zero_timing()["observed_zero_sample_count"] == 1


def test_terminal_zero_wait_restarts_full_odom_window_after_late_motion(
    monkeypatch,
):
    runner = _terminal_zero_runner(
        monkeypatch,
        ((0.30, False), (0.50, False), (0.60, False), (0.75, False)),
        timeout_sec=0.90,
        odom_speed_at=lambda offset: 0.2 if 0.25 <= offset < 0.50 else 0.0,
        barrier_at=0.25,
    )

    assert runner._wait_for_final_stillness()
    assert runner._test_clock.now >= 10.65
    assert runner._terminal_zero_confirmed


def test_direct_backend_action_return_barrier_uses_same_contract(monkeypatch):
    runner = _terminal_zero_runner(
        monkeypatch,
        ((0.05, False), (0.22, False)),
        barrier_source="navigate_action_return",
    )

    assert runner._wait_for_final_stillness()
    assert runner._terminal_zero_timing()["barrier_source"] == (
        "navigate_action_return"
    )


def test_cross_reset_completion_callback_cannot_mark_terminal_barrier(monkeypatch):
    runner = _terminal_zero_runner(monkeypatch, (), barrier_source=None)
    _expect_route_completion(
        runner, next_epoch=5, leg_id="G3", final_leg=False
    )

    runner._clear_run_state()
    runner._route_goal_complete_callback(SimpleNamespace(data=False))

    assert runner._route_goal_complete_epoch == 5
    assert runner._terminal_zero_barrier_monotonic is None
    assert runner._terminal_zero_expected_route_completion_epoch is None


def test_clear_run_state_resets_terminal_zero_observation_fields():
    runner = object.__new__(ExperimentRunner)
    runner._clear_run_state()
    runner._terminal_zero_observation_started_monotonic = 1.0
    runner._terminal_zero_barrier_monotonic = 1.05
    runner._terminal_zero_barrier_source = "route_goal_complete"
    runner._terminal_zero_barrier_leg_id = "G3"
    runner._terminal_zero_expected_route_completion_epoch = 9
    runner._terminal_zero_expected_route_leg_id = "G4"
    runner._terminal_zero_expected_route_leg_is_final = True
    runner._terminal_zero_confirmed_monotonic = 2.0
    runner._terminal_zero_first_zero_monotonic = 1.1
    runner._terminal_zero_last_zero_monotonic = 1.9
    runner._terminal_zero_confirming_sample_count = 4
    runner._terminal_zero_confirmed = True
    runner._terminal_zero_reason = "terminal_zero_confirmed"
    runner._cmd_vel_sim_last_receive_monotonic = 2.0
    runner._cmd_vel_sim_last_nonzero_monotonic = 1.2
    runner._cmd_vel_sim_zero_stamps = [1.3, 1.9]

    runner._clear_run_state()

    assert runner._terminal_zero_observation_started_monotonic is None
    assert runner._terminal_zero_barrier_monotonic is None
    assert runner._terminal_zero_barrier_source == "not_observed"
    assert runner._terminal_zero_barrier_leg_id is None
    assert runner._terminal_zero_expected_route_completion_epoch is None
    assert runner._terminal_zero_expected_route_leg_id is None
    assert not runner._terminal_zero_expected_route_leg_is_final
    assert runner._terminal_zero_confirmed_monotonic is None
    assert runner._terminal_zero_first_zero_monotonic is None
    assert runner._terminal_zero_last_zero_monotonic is None
    assert runner._terminal_zero_confirming_sample_count == 0
    assert not runner._terminal_zero_confirmed
    assert runner._terminal_zero_reason == "not_checked"
    assert runner._cmd_vel_sim_last_receive_monotonic is None
    assert runner._cmd_vel_sim_last_nonzero_monotonic is None
    assert runner._cmd_vel_sim_zero_stamps == []


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


def test_terminal_zero_failure_is_retried_when_successful_resume_is_required(tmp_path):
    root = tmp_path / "run-0002-seed-7301"
    root.mkdir()
    manifest = {
        "random_seed": 7301,
        "run_index": 2,
        "condition_id": "dynamic_appearance",
        "appearance": {"profile_id": "dim_warm"},
        "dynamic_selection": {"case_id": "full_route_three_stage", "variant_id": "v1"},
        "result": "success",
        "terminal_zero_confirmed": False,
        "terminal_zero_reason": "terminal_zero_timeout",
    }
    summary = {
        "data_complete": True,
        "checksums_verified": True,
        "strict_success": False,
        "terminal_zero_confirmed": False,
    }
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


def _static_sat_evidence_run(
    tmp_path: Path,
    *,
    obstacle_position: tuple[float, float] | None,
    contact_sensor_collision: bool,
    nav2_succeeded: bool = True,
    collision_monitor_stop: bool = False,
):
    scenario = load_scenario(
        Path(__file__).parents[1] / "config" / "static.yaml"
    )
    scenario = replace(
        scenario,
        obstacles={
            "layout_id": "sat_diagnostic_fixture",
            "static": [{"id": "low_box"}],
            "trajectories": [],
        },
        success=replace(
            scenario.success,
            minimum_ground_truth_path_length_m=0.0,
            minimum_reverse_distance_m=0.0,
            maximum_reverse_distance_fraction=1.0,
            minimum_curved_distance_fraction=0.0,
            maximum_stopped_time_fraction=1.0,
        ),
    )
    runner = object.__new__(ExperimentRunner)
    runner._clear_run_state()
    runner._scenario = scenario
    runner._active_selection = RunSelection(7301, condition_id="static")
    runner._spawn_pose = SimpleNamespace(
        name="mapping_start",
        usd=SimpleNamespace(as_dict=lambda: {}),
        map=SimpleNamespace(as_dict=lambda: {}),
    )
    runner._ground_truth_samples = [
        OdometrySample(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0)
    ]
    runner._odom_samples = [
        OdometrySample(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0)
    ]
    runner._leg_results = [{
        "id": "G1",
        "nav2_status": experiment_runner_module.GoalStatus.STATUS_SUCCEEDED,
    }]
    runner._robot_footprint = (
        (0.255, 0.210),
        (0.255, -0.210),
        (-0.230, -0.210),
        (-0.230, 0.210),
    )
    if obstacle_position is not None:
        runner._obstacle_state = {
            "obstacles": [{
                "id": "low_box",
                "position": [*obstacle_position, 0.08],
                "position_frame": "map",
                "size": [0.30, 0.30, 0.16],
                "retired": False,
            }],
            "events": [],
        }
    runner._collision_seen = False
    runner._collision_callback(SimpleNamespace(data=contact_sensor_collision))
    runner._collision_monitor_active = True
    runner._localization_seen = True
    runner._tf_ever_available = True
    runner._terminal_zero_confirmed = True
    runner._terminal_zero_reason = "terminal_zero_confirmed"
    runner._navigation_start_stamp_s = 0.5
    runner._navigation_end_stamp_s = 1.5
    runner._dynamic_runtime_contract = {"verified": True}
    runner._appearance_runtime_contract = {"verified": True}
    runner._appearance_state = None
    runner._appearance_config_hash = None
    runner._optimal_reference = None
    runner._optimal_reference_hash = None
    runner._navigation_graph = None
    runner._minimum_safety_scan_range_m = None
    runner._provenance = {}
    runner._robot_config_hash = "robot"
    runner._nav2_config_hash = "nav2"
    runner._nav2_profile = "stable"
    runner._clear_slam_localization_buffer = True
    runner._reset_map_base_translation_tolerance_m = 0.05
    runner._experiment_arm = ""
    runner._navigation_execution_backend = "navigate_to_pose"
    runner._reset_receipt = {}
    runner._record_bag = False
    runner._fail_stop_metric_contract = None
    if collision_monitor_stop:
        runner._collision_lock_timeout_sec = 0.0
        runner._lookup_fresh_map_to_odom = lambda: None
        runner._collision_lock_callback(
            SimpleNamespace(action_type=CollisionMonitorState.STOP)
        )
        runner._update_health()

    nav2_status = (
        experiment_runner_module.GoalStatus.STATUS_SUCCEEDED
        if nav2_succeeded
        else experiment_runner_module.GoalStatus.STATUS_ABORTED
    )
    manifest = runner._build_manifest(
        run_index=2,
        seed=7301,
        nav2_succeeded=nav2_succeeded,
        timed_out=False,
        nav2_status=nav2_status,
        final_still=True,
        runner_error=None,
    )
    manifest["dynamic_selection"] = {"case_id": None, "variant_id": None}

    root = tmp_path / "run-0002-seed-7301"
    root.mkdir()
    (root / "TRIAL_DISPATCHED.json").write_text("{}\n", encoding="utf-8")

    def write_files(names):
        for name in names:
            (root / name).write_text("fixture\n", encoding="utf-8")
        return True

    runner._write_depth_snapshot = lambda _root: write_files(
        ("depth_frame.pgm", "depth_frame.json")
    )
    runner._write_costmap_snapshot = lambda _root, name, _grid: write_files(
        (f"{name}.pgm", f"{name}.json")
    )
    runner._write_scan_snapshot = lambda _root, stem, _frame: write_files(
        (f"{stem}.csv", f"{stem}.json")
    )
    summary = runner._write_run_evidence(
        manifest, 7301, 2, root, bag_complete=False
    )
    return runner, manifest, summary, root


def test_sat_overlap_is_diagnostic_when_contact_sensor_is_clear(tmp_path):
    runner, manifest, summary, _root = _static_sat_evidence_run(
        tmp_path,
        obstacle_position=(1.25, 0.0),
        contact_sensor_collision=False,
    )

    geometric = manifest["static_geometric_contact"]
    assert geometric["observed"]
    assert geometric["maximum_sat_overlap_m"] > 0.001
    assert geometric["exceeds_acceptance_overlap"]
    assert geometric["diagnostic_only"]
    assert not runner._collision_detected
    assert manifest["result"] == "success"
    assert manifest["failure_reason"] == ""
    assert summary["strict_success"]
    assert summary["physical_collision_free"]
    assert not summary["isaac_contact_sensor_collision_detected"]


def test_contact_sensor_collision_fails_even_when_sat_overlap_is_zero(tmp_path):
    runner, manifest, summary, _root = _static_sat_evidence_run(
        tmp_path,
        obstacle_position=(5.0, 0.0),
        contact_sensor_collision=True,
    )

    geometric = manifest["static_geometric_contact"]
    assert geometric["observed"]
    assert geometric["maximum_sat_overlap_m"] == 0.0
    assert runner._isaac_contact_sensor_collision_detected
    assert manifest["result"] == "failure"
    assert manifest["failure_reason"] == "collision_detected"
    assert not summary["strict_success"]
    assert not summary["physical_collision_free"]
    assert summary["isaac_contact_sensor_collision_detected"]


def test_collision_monitor_stop_is_navigation_failure_not_physical_collision(
    tmp_path,
):
    runner, manifest, summary, _root = _static_sat_evidence_run(
        tmp_path,
        obstacle_position=(5.0, 0.0),
        contact_sensor_collision=False,
        nav2_succeeded=False,
        collision_monitor_stop=True,
    )

    assert runner._collision_monitor_locked
    assert manifest["result"] == "failure"
    assert "nav2_action_failed" in manifest["failure_reason"]
    assert "collision_monitor_locked" in manifest["failure_reason"]
    assert "collision_detected" not in manifest["failure_reason"]
    assert not summary["strict_success"]
    assert summary["physical_collision_free"]
    assert not summary["isaac_contact_sensor_collision_detected"]


@pytest.mark.parametrize(
    ("obstacle_position", "warning_expected"),
    [
        (None, False),
        ((1.25, 0.0), True),
    ],
)
def test_sat_diagnostic_presence_does_not_change_complete_resume_eligibility(
    tmp_path, obstacle_position, warning_expected
):
    runner, manifest, summary, root = _static_sat_evidence_run(
        tmp_path,
        obstacle_position=obstacle_position,
        contact_sensor_collision=False,
    )
    geometric = manifest["static_geometric_contact"]
    assert geometric["observed"] is warning_expected
    assert bool(manifest["warning_reason"]) is warning_expected
    assert summary["warning_reason"] == manifest["warning_reason"]
    assert summary["data_complete"]
    assert summary["checksums_verified"]
    assert summary["strict_success"]
    assert summary["physical_collision_free"]
    selection = SimpleNamespace(
        seed=7301,
        condition_id="static",
        appearance_profile_id=None,
        case_id=None,
        variant_id=None,
    )
    runner._require_successful_resume = True

    assert runner._completed_resume_manifest(root, 2, selection) == manifest


def test_sat_diagnostics_do_not_change_collision_rate_statistics():
    records = []
    for seed in range(20):
        summary = {
            "strict_success": True,
            "physical_collision_free": True,
            "data_complete": True,
            "checksums_verified": True,
            "path_deviation_percent": 0.0,
            "legs": [{"id": f"G{index}"} for index in range(1, 6)],
        }
        if seed % 2:
            summary["static_geometric_contact"] = {
                "observed": True,
                "contact_detected": True,
                "maximum_sat_overlap_m": 0.12,
                "diagnostic_only": True,
            }
        records.append({"summary": summary, "manifest": {}})

    result = _rate_group(
        records,
        name="static",
        required_rate_percent=95.0,
        require_path_deviation=True,
    )

    assert result["strict_successes"] == 20
    assert result["collision_free_runs"] == 20
    assert result["collision_free_rate_percent"] == 100.0
    assert result["complete_evidence_runs"] == 20
    assert result["passed"]


def test_checksum_finalization_updates_summary_and_covers_final_bytes(tmp_path):
    root = tmp_path / "run-0001-seed-19301"
    root.mkdir()
    summary = {"checksums_verified": False, "strict_success": True}
    (root / "run_summary.json").write_text(json.dumps(summary), encoding="utf-8")
    (root / "evidence.json").write_text('{"complete": true}\n', encoding="utf-8")

    ExperimentRunner._finalize_checksums(root, summary)

    stored_summary = json.loads((root / "run_summary.json").read_text(encoding="utf-8"))
    assert summary["checksums_verified"] is True
    assert stored_summary["checksums_verified"] is True
    assert ExperimentRunner._checksums_are_verified(root)


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

    runner._scenario.obstacle_trajectories = (
        {"id": "oncoming", "motion": "oncoming", "trigger_group": "G2"},
        {"id": "crossing", "motion": "crossing", "trigger_group": "G3"},
        {"id": "following", "motion": "same_direction_slow", "trigger_group": "G4"},
        {"id": "block", "motion": "temporary_block", "trigger_group": "G5"},
    )
    runner._active_selection = SimpleNamespace(case_id="full_route_four_stage")
    assert runner._selected_dynamic_groups_for_goal("G2") == ["G2"]
    assert runner._selected_dynamic_groups_for_goal("G3") == ["G3"]
    assert runner._selected_dynamic_groups_for_goal("G4") == ["G4"]
    assert runner._selected_dynamic_groups_for_goal("G5") == ["G5"]


def test_single_dynamic_g2_crossing_maps_crossing_to_its_trigger_group():
    runner = object.__new__(ExperimentRunner)
    runner._scenario = SimpleNamespace(
        scenario_type="dynamic",
        obstacle_trajectories=(
            {"id": "low_box", "motion": "crossing", "trigger_group": "G2"},
        ),
    )
    runner._active_selection = SimpleNamespace(case_id="single_dynamic_g2_crossing")

    assert runner._selected_dynamic_groups_for_goal("G2") == ["G2"]


def test_seeds_only_dynamic_scenario_selects_all_trajectories_and_passes_guard():
    runner = object.__new__(ExperimentRunner)
    runner._scenario = SimpleNamespace(
        scenario_type="dynamic",
        obstacle_trajectories=(
            {"id": "first", "motion": "crossing", "trigger_group": "G2"},
            {"id": "second", "motion": "oncoming", "trigger_group": "G3"},
        ),
    )
    runner._active_selection = RunSelection(1)

    assert runner._selected_dynamic_groups_for_goal("G2") == ["G2"]
    assert runner._selected_dynamic_groups_for_goal("G3") == ["G3"]
    runner._validate_dynamic_episode_selection()


@pytest.mark.parametrize(
    ("case_id", "trajectories"),
    [
        (
            "unknown_dynamic_case",
            ({"id": "low_box", "motion": "crossing", "trigger_group": "G2"},),
        ),
        ("single_dynamic_g2_crossing", ()),
    ],
)
def test_dynamic_episode_rejects_empty_selection_before_reset(case_id, trajectories):
    runner = object.__new__(ExperimentRunner)
    runner._clock_ready = True
    runner._clock_timeout_sec = 1.0
    runner._wait_until = lambda predicate, _timeout: predicate()
    runner._verify_dynamic_runtime_contract = lambda: None
    runner._verify_appearance_runtime_contract = lambda: None
    runner._verify_collision_monitor_active = lambda: None
    runner._authorization_only = False
    runner._scenario = SimpleNamespace(
        scenario_id="dynamic_selection_test",
        scenario_type="dynamic",
        obstacle_trajectories=trajectories,
        run_matrix=(RunSelection(1, case_id, "v1"),),
        seeds=(),
    )
    runner._run_indices = None
    runner._require_pregoal_authorization = False
    reset_calls = []
    runner._reset_simulation = lambda *args: reset_calls.append(args)

    with pytest.raises(ConfigurationError, match="no trigger groups or expected actor IDs"):
        runner.run_all()

    assert reset_calls == []


def test_runner_has_no_actor_lifecycle_costmap_clear_workaround():
    assert not hasattr(
        ExperimentRunner, "_request_pending_dynamic_trail_clears"
    )


def test_global_costmap_readiness_rejects_default_window_and_covers_all_goals():
    runner = object.__new__(ExperimentRunner)
    runner._spawn_pose = SimpleNamespace(map=SimpleNamespace(position=(21.2, 120.0)))
    runner._scenario = SimpleNamespace(
        route=(
            SimpleNamespace(position=(1.5, 131.8)),
            SimpleNamespace(position=(-42.6, 180.6)),
        ),
        goal=SimpleNamespace(position=(-42.6, 180.6)),
    )
    metadata = SimpleNamespace(
        resolution=0.05,
        size_x=100,
        size_y=100,
        origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0)),
    )
    runner._global_costmap = SimpleNamespace(
        header=SimpleNamespace(frame_id="map"), metadata=metadata
    )
    assert not runner._global_costmap_covers_mission()

    metadata.size_x = metadata.size_y = 1600
    metadata.origin.position.x = -52.0182
    metadata.origin.position.y = 111.603
    assert runner._global_costmap_covers_mission()


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


def test_dynamic_acceptance_rejects_empty_expected_actor_ids():
    status = _dynamic_interaction_acceptance(
        scenario_type="dynamic",
        expected_ids=set(),
        triggered_ids=set(),
        completed_ids=set(),
        retired_ids=set(),
        clearance_by_actor={},
        evidence_complete=True,
    )

    assert status["complete"] is False
    assert status["minimum_clearance_complete"] is False
    assert status["reason"] == "expected_dynamic_actor_ids_empty"


def test_static_appearance_profile_does_not_select_dynamic_obstacle_case():
    appearance = RunSelection(
        9201, "static", "v1", "dim_warm", "rivermark_appearance"
    )
    dynamic = RunSelection(9101, "full_route_four_stage", "v1")

    assert _reset_dynamic_selection("static", appearance) == (None, None)
    assert _reset_dynamic_selection("dynamic", dynamic) == (
        "full_route_four_stage", "v1"
    )


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
