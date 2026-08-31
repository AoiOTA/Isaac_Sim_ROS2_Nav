import math
from types import MethodType, SimpleNamespace

import pytest

from robot_experiments import experiment_runner as experiment_runner_module
from robot_experiments.configuration import ConfigurationError
from robot_experiments.experiment_runner import (
    ExperimentRunner,
    OdometrySample,
    _positive_finite_float,
)
from robot_experiments.motion_benchmark import ResetStopGateStatus


class _UnavailableClient:
    def __init__(self) -> None:
        self.wait_count = 0
        self.call_count = 0

    def wait_for_service(self, *, timeout_sec: float) -> bool:
        self.wait_count += 1
        return False

    def call_async(self, request):
        self.call_count += 1
        raise AssertionError("unavailable service must not be called")


def test_disabled_slam_buffer_clear_does_not_touch_client_and_reset_continues():
    calls = []
    runner = SimpleNamespace(
        _clear_slam_localization_buffer=False,
        _localization_seed_epoch=7,
        _cancel_stale_navigation_goal=lambda: calls.append("cancel"),
    )

    def clear_buffer() -> None:
        calls.append("clear")
        ExperimentRunner._clear_localization_buffer(runner)

    class ResetContinued(RuntimeError):
        pass

    def set_reset_seed(*args) -> None:
        calls.append("set_reset_seed")
        raise ResetContinued

    runner._clear_localization_buffer = clear_buffer
    runner._set_reset_seed = set_reset_seed

    with pytest.raises(ResetContinued):
        ExperimentRunner._reset_simulation(runner, 41)

    assert calls == ["cancel", "clear", "set_reset_seed"]
    assert not hasattr(runner, "_localization_buffer_client")


def test_default_enabled_slam_buffer_clear_still_fails_closed_when_service_missing():
    client = _UnavailableClient()
    runner = SimpleNamespace(
        _clear_slam_localization_buffer=True,
        _localization_buffer_client=client,
        _service_timeout_sec=0.25,
        _raise_if_shutdown=lambda: None,
    )

    with pytest.raises(
        RuntimeError,
        match="SLAM Toolbox localization buffer service is unavailable",
    ):
        ExperimentRunner._clear_localization_buffer(runner)

    assert client.wait_count == 1
    assert client.call_count == 0


def _planning_prior(*, stamp_sec: int, healthy: bool):
    return SimpleNamespace(
        stamp=SimpleNamespace(sec=stamp_sec, nanosec=0),
        sequence=stamp_sec,
        reset_epoch=2,
        recurrent_session_id="session-2",
        map_version="map-v1",
        module2_healthy=healthy,
        input_healthy=healthy,
        observation_valid=healthy,
        trusted_write=healthy,
        schema_version="bio_nav_planning_prior_v4",
        place_entropy_normalized=0.58,
        context_uncertainty=0.58,
        place_belief=[],
        dynamic_cost=[],
    )


def test_planning_readiness_uses_consecutive_fresh_healthy_priors_only():
    runner = SimpleNamespace(
        _latest_planning_prior_readiness=None,
        _planning_prior_ready_streak=0,
        _navigation_active=False,
    )

    for stamp_sec in range(1, 6):
        ExperimentRunner._planning_prior_callback(
            runner, _planning_prior(stamp_sec=stamp_sec, healthy=True)
        )

    assert runner._planning_prior_ready_streak == 5
    assert runner._latest_planning_prior_readiness == {
        "stamp_s": 5.0,
        "sequence": 5,
        "reset_epoch": 2,
        "recurrent_session_id": "session-2",
        "map_version": "map-v1",
        "module2_healthy": True,
        "input_healthy": True,
        "observation_valid": True,
        "trusted_write": True,
        "schema_version": "bio_nav_planning_prior_v4",
        "accepted": True,
        "place_entropy_normalized": 0.58,
        "context_uncertainty": 0.58,
    }

    ExperimentRunner._planning_prior_callback(
        runner, _planning_prior(stamp_sec=6, healthy=False)
    )
    assert runner._planning_prior_ready_streak == 0

    untrusted = _planning_prior(stamp_sec=7, healthy=True)
    untrusted.trusted_write = False
    ExperimentRunner._planning_prior_callback(runner, untrusted)
    assert runner._planning_prior_ready_streak == 0
    assert runner._latest_planning_prior_readiness["accepted"] is False


def _transform(x: float, y: float, yaw_rad: float, stamp_s: float):
    return SimpleNamespace(
        stamp_s=stamp_s,
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=x, y=y),
            rotation=SimpleNamespace(
                x=0.0,
                y=0.0,
                z=math.sin(yaw_rad / 2.0),
                w=math.cos(yaw_rad / 2.0),
            ),
        ),
    )


def _reset_recovery_runner(
    *,
    odom_x: float = 0.0,
    map_base_x: float = 1.0,
    map_base_yaw_rad: float = 0.0,
    map_base_stamp_s: float = 10.0,
    map_base_tolerance_m: float = 0.15,
):
    map_to_odom = _transform(1.0, 2.0, 0.0, 10.0)
    map_to_base = _transform(
        map_base_x, 2.0, map_base_yaw_rad, map_base_stamp_s
    )
    runner = SimpleNamespace(
        _reset_recovery_timeout_sec=0.5,
        _reset_tf_stability_sec=0.015,
        _odom_max_age_sec=1.0,
        _tf_gap_tolerance_sec=1.0,
        _reset_tf_translation_tolerance_m=0.05,
        _reset_map_base_translation_tolerance_m=map_base_tolerance_m,
        _reset_tf_yaw_tolerance_rad=math.radians(3.0),
        _odom_samples=[OdometrySample(odom_x, 0.0, 0.0, 0.0, 0.0, 10.0, 0.0)],
        _ground_truth_samples=[
            OdometrySample(1.0, 2.0, 0.0, 0.0, 0.0, 10.0, 0.0)
        ],
        _spawn_pose=SimpleNamespace(
            map=SimpleNamespace(position=(1.0, 2.0), yaw_deg=0.0)
        ),
        _spin_once=lambda _timeout: None,
        _lookup_fresh_map_to_odom=lambda: (map_to_odom, 10.0),
        _tf_buffer=SimpleNamespace(
            lookup_transform=lambda _target, _source, _time: map_to_base
        ),
        _transform_stamp_seconds=lambda transform: transform.stamp_s,
        _clock_seconds=lambda: 10.0,
    )
    return runner


def _monotonic(values):
    iterator = iter(values)
    last = [0.0]

    def read():
        try:
            last[0] = next(iterator)
        except StopIteration:
            pass
        return last[0]

    return read


def test_map_base_six_centimetres_passes_with_independent_fifteen_centimetre_tolerance(
    monkeypatch,
):
    runner = _reset_recovery_runner(map_base_x=1.06, map_base_tolerance_m=0.15)
    monkeypatch.setattr(experiment_runner_module.rclpy, "ok", lambda: True)
    monkeypatch.setattr(
        experiment_runner_module.time,
        "monotonic",
        _monotonic([0.0, 0.01, 0.02, 0.03, 0.04]),
    )

    ExperimentRunner._wait_for_reset_recovery(runner, 9.0, 9.0)


def test_relaxed_map_base_tolerance_does_not_relax_odom_reset_tolerance(monkeypatch):
    runner = _reset_recovery_runner(odom_x=0.06, map_base_tolerance_m=0.15)
    monkeypatch.setattr(experiment_runner_module.rclpy, "ok", lambda: True)
    monkeypatch.setattr(
        experiment_runner_module.time,
        "monotonic",
        _monotonic([0.0, 0.1, 0.2, 0.6]),
    )

    with pytest.raises(TimeoutError, match="odom_ready=False"):
        ExperimentRunner._wait_for_reset_recovery(runner, 9.0, 9.0)


@pytest.mark.parametrize(
    ("runner", "status"),
    [
        (
            _reset_recovery_runner(map_base_x=1.06, map_base_tolerance_m=0.05),
            "map_base_ready=False",
        ),
        (_reset_recovery_runner(map_base_stamp_s=8.0), "map_base_ready=False"),
        (_reset_recovery_runner(map_base_yaw_rad=math.radians(3.1)), "map_base_ready=False"),
    ],
    ids=("default-five-centimetres", "stale-stamp", "yaw-outside-three-degrees"),
)
def test_map_base_translation_override_preserves_other_reset_gates(
    monkeypatch, runner, status
):
    monkeypatch.setattr(experiment_runner_module.rclpy, "ok", lambda: True)
    monkeypatch.setattr(
        experiment_runner_module.time,
        "monotonic",
        _monotonic([0.0, 0.1, 0.2, 0.6]),
    )

    with pytest.raises(TimeoutError, match=status):
        ExperimentRunner._wait_for_reset_recovery(runner, 9.0, 9.0)


@pytest.mark.parametrize("value", (0.0, -0.1, math.nan, math.inf, "invalid"))
def test_map_base_translation_tolerance_rejects_invalid_values(value):
    with pytest.raises(
        ConfigurationError,
        match="reset_map_base_translation_tolerance_m must be positive and finite",
    ):
        _positive_finite_float(value, "reset_map_base_translation_tolerance_m")


class _RouteGoalPublisher:
    def __init__(self, runner):
        self.runner = runner
        self.messages = []

    def get_subscription_count(self):
        return 1

    def publish(self, message):
        self.messages.append(message)
        self.runner._canonical_route_epoch += 1
        self.runner._canonical_routes.append({
            "request_id": len(self.messages),
            "planned_length_m": 1.0,
            "edge_ids": [len(self.messages)],
        })
        self.runner._route_goal_complete_epoch += 1
        self.runner._latest_route_goal_complete = True


def test_route_guided_arms_only_after_four_legs_and_before_final_goal():
    specifications = tuple(
        SimpleNamespace(goal_id=f"G{index}") for index in range(1, 6)
    )
    runner = SimpleNamespace(
        _navigation_graph=object(),
        _service_timeout_sec=1.0,
        _scenario=SimpleNamespace(
            route=specifications,
            goal=specifications[-1],
            timeout_sec=30.0,
            leg_timeout_sec=5.0,
        ),
        _navigation_active=False,
        _navigation_start_stamp_s=None,
        _navigation_end_stamp_s=None,
        _minimum_poses_remaining=None,
        _canonical_route_epoch=0,
        _route_goal_complete_epoch=0,
        _latest_route_goal_complete=False,
        _canonical_routes=[],
        _ground_truth_samples=[],
        _leg_results=[],
        _completed_dynamic_obstacle_ids=set(),
        _goal_dispatch_recorded=True,
        _dynamic_guard_aborted=False,
        _wait_until=lambda predicate, _timeout: predicate(),
        _clock_seconds=lambda: 1.0,
        _pose_message=lambda specification: specification,
        _trigger_obstacle_group=lambda _goal_id: None,
        _complete_obstacle_group=lambda goal_id: (f"actor-{goal_id}",),
        _spin_once=lambda _timeout: None,
        _wait_for_reset_stop_gate_release=lambda: None,
    )
    runner._route_goal_publisher = _RouteGoalPublisher(runner)
    arm_publication_counts = []
    runner._arm_next_terminal_fence = lambda: arm_publication_counts.append(
        len(runner._route_goal_publisher.messages)
    )

    result = ExperimentRunner._navigate_route_guided(runner)

    assert result == (
        True,
        False,
        experiment_runner_module.GoalStatus.STATUS_SUCCEEDED,
    )
    assert len(runner._route_goal_publisher.messages) == 5
    assert arm_publication_counts == [4]
    assert runner._completed_dynamic_obstacle_ids == {
        "actor-G1",
        "actor-G2",
        "actor-G3",
        "actor-G4",
        "actor-G5",
    }


def test_route_goal_complete_bool_does_not_arm_terminal_fence():
    client = SimpleNamespace(call_count=0)
    runner = SimpleNamespace(
        _route_goal_complete_epoch=0,
        _latest_route_goal_complete=False,
        _terminal_fence_arm_client=client,
        _terminal_zero_expected_route_completion_epoch=None,
        _terminal_zero_expected_route_leg_id=None,
        _terminal_zero_expected_route_leg_is_final=False,
    )

    ExperimentRunner._route_goal_complete_callback(
        runner, SimpleNamespace(data=True))

    assert runner._route_goal_complete_epoch == 1
    assert runner._latest_route_goal_complete is True
    assert client.call_count == 0


class _PublisherCount:
    def __init__(self, count: int) -> None:
        self.count = count

    def get_publisher_count(self) -> int:
        return self.count


def _gate_status(generation: int, held: bool, received_at: float):
    return ResetStopGateStatus(
        generation=generation,
        held=held,
        eligible_generation=generation if held else None,
        received_at=received_at,
    )


def _route_guided_gate_runner(
    *,
    status,
    status_received: bool = True,
    publisher_count: int = 1,
    status_updates=(),
):
    events = []
    specification = SimpleNamespace(goal_id="G1")
    runner = SimpleNamespace(
        _navigation_graph=object(),
        _service_timeout_sec=1.0,
        _reset_recovery_timeout_sec=0.25,
        _scenario=SimpleNamespace(
            route=(specification,),
            goal=specification,
            timeout_sec=30.0,
            leg_timeout_sec=5.0,
        ),
        _navigation_active=False,
        _navigation_start_stamp_s=None,
        _navigation_end_stamp_s=None,
        _minimum_poses_remaining=None,
        _canonical_route_epoch=0,
        _route_goal_complete_epoch=0,
        _latest_route_goal_complete=False,
        _canonical_routes=[],
        _ground_truth_samples=[],
        _leg_results=[],
        _completed_dynamic_obstacle_ids=set(),
        _goal_dispatch_recorded=False,
        _dynamic_guard_aborted=False,
        _reset_receipt={"generation": 4},
        _reset_call_barrier_monotonic=1.0,
        _reset_stop_gate_status=status,
        _reset_stop_gate_status_error=None,
        _reset_stop_gate_status_received=status_received,
        _reset_stop_gate_status_subscription=_PublisherCount(publisher_count),
        _clock_seconds=lambda: 1.0,
        _pose_message=lambda value: value,
        _trigger_obstacle_group=lambda _goal_id: None,
        _complete_obstacle_group=lambda _goal_id: ("dynamic_box",),
        _arm_next_terminal_fence=lambda: None,
    )

    class OrderedRouteGoalPublisher(_RouteGoalPublisher):
        def publish(self, message):
            events.append("publish")
            super().publish(message)

    updates = list(status_updates)

    def spin_once(_timeout):
        if updates:
            label, payload = updates.pop(0)
            events.append(label)
            if isinstance(payload, str):
                ExperimentRunner._reset_stop_gate_status_callback(
                    runner, SimpleNamespace(data=payload)
                )
            else:
                runner._reset_stop_gate_status = payload
                runner._reset_stop_gate_status_received = True

    def wait_until(predicate, _timeout):
        for _ in range(3):
            if predicate():
                return True
            spin_once(0.0)
        return False

    def record_dispatch():
        events.append("dispatch")
        runner._goal_dispatch_recorded = True

    runner._spin_once = spin_once
    runner._wait_until = wait_until
    runner._record_trial_dispatched = record_dispatch
    runner._route_goal_publisher = OrderedRouteGoalPublisher(runner)
    runner._wait_for_reset_stop_gate_release = MethodType(
        ExperimentRunner._wait_for_reset_stop_gate_release, runner
    )
    return runner, events


def test_route_dispatch_waits_for_delayed_same_generation_release():
    runner, events = _route_guided_gate_runner(
        status=_gate_status(4, True, 1.1),
        status_updates=((
            "release",
            '{"generation":4,"held":false,"eligible_generation":null}',
        ),),
    )

    result = ExperimentRunner._navigate_route_guided(runner)

    assert result == (
        True,
        False,
        experiment_runner_module.GoalStatus.STATUS_SUCCEEDED,
    )
    assert events == ["release", "dispatch", "publish"]
    assert runner._reset_stop_gate_status.received_at > 1.0
    assert runner._completed_dynamic_obstacle_ids == {"dynamic_box"}


def test_route_dispatch_waits_on_stale_generation_without_writing_or_publishing():
    runner, events = _route_guided_gate_runner(
        status=_gate_status(3, False, 1.1)
    )

    with pytest.raises(TimeoutError, match="generation=4"):
        ExperimentRunner._navigate_route_guided(runner)

    assert events == []


def test_route_dispatch_current_generation_hold_uses_reset_recovery_timeout():
    runner, events = _route_guided_gate_runner(
        status=_gate_status(4, True, 1.1)
    )
    observed_timeouts = []

    def wait_until(predicate, timeout):
        if predicate():
            return True
        observed_timeouts.append(timeout)
        return False

    runner._wait_until = wait_until

    with pytest.raises(TimeoutError, match="release timed out"):
        ExperimentRunner._navigate_route_guided(runner)

    assert observed_timeouts == [runner._reset_recovery_timeout_sec]
    assert events == []


def test_route_dispatch_future_generation_fails_before_write_or_publish():
    runner, events = _route_guided_gate_runner(
        status=_gate_status(5, False, 1.1)
    )

    with pytest.raises(RuntimeError, match="generation advanced"):
        ExperimentRunner._navigate_route_guided(runner)

    assert events == []


def test_route_dispatch_malformed_status_fails_before_write_or_publish():
    runner, events = _route_guided_gate_runner(
        status=None,
        status_updates=(("malformed", "not-json"),),
    )

    with pytest.raises(RuntimeError, match="invalid reset stop gate status"):
        ExperimentRunner._navigate_route_guided(runner)

    assert events == ["malformed"]


def test_route_dispatch_missing_status_blocks_even_when_no_publisher():
    runner, events = _route_guided_gate_runner(
        status=None,
        status_received=False,
        publisher_count=0,
    )

    with pytest.raises(TimeoutError, match="release timed out"):
        ExperimentRunner._navigate_route_guided(runner)

    assert events == []
    assert runner._completed_dynamic_obstacle_ids == set()


class _ImmediateFuture:
    def __init__(self, result) -> None:
        self._result = result

    def result(self):
        return self._result


class _SuccessfulGoalHandle:
    accepted = True

    def get_result_async(self):
        return _ImmediateFuture(SimpleNamespace(
            status=experiment_runner_module.GoalStatus.STATUS_SUCCEEDED
        ))


class _SuccessfulNavigateClient:
    def wait_for_server(self, *, timeout_sec):
        return timeout_sec > 0.0

    def send_goal_async(self, _goal, *, feedback_callback):
        assert callable(feedback_callback)
        return _ImmediateFuture(_SuccessfulGoalHandle())


def _successful_direct_runner(complete_obstacle_group):
    specification = SimpleNamespace(goal_id="G2")
    return SimpleNamespace(
        _navigate_client=_SuccessfulNavigateClient(),
        _service_timeout_sec=1.0,
        _action_name="/navigate_to_pose",
        _scenario=SimpleNamespace(
            route=(specification,),
            goal=specification,
            timeout_sec=30.0,
            leg_timeout_sec=5.0,
        ),
        _navigation_active=False,
        _navigation_start_stamp_s=None,
        _navigation_end_stamp_s=None,
        _ground_truth_samples=[],
        _minimum_poses_remaining=None,
        _goal_dispatch_recorded=True,
        _goal_message=lambda value: value,
        _navigation_feedback_callback=lambda _feedback: None,
        _wait_future=lambda _future, _deadline, **_kwargs: True,
        _trigger_obstacle_group=lambda _goal_id: None,
        _complete_obstacle_group=complete_obstacle_group,
        _completed_dynamic_obstacle_ids=set(),
        _leg_results=[],
        _dynamic_guard_aborted=False,
        _clock_seconds=lambda: 1.0,
    )


def test_direct_success_registers_completion_returned_actor_ids():
    runner = _successful_direct_runner(
        lambda _goal_id: ("dynamic_box",)
    )

    result = ExperimentRunner._navigate_direct(runner)

    assert result == (
        True,
        False,
        experiment_runner_module.GoalStatus.STATUS_SUCCEEDED,
    )
    assert runner._completed_dynamic_obstacle_ids == {"dynamic_box"}


def test_direct_completion_failure_does_not_register_actor_ids():
    def completion_failure(_goal_id):
        raise RuntimeError("completion failed")

    runner = _successful_direct_runner(completion_failure)

    with pytest.raises(RuntimeError, match="completion failed"):
        ExperimentRunner._navigate_direct(runner)

    assert runner._completed_dynamic_obstacle_ids == set()


def test_direct_backend_does_not_consult_reset_stop_gate():
    runner = SimpleNamespace(
        _navigation_execution_backend="navigate_to_pose",
        _navigate_direct=lambda: (True, False, 4),
        _navigate_route_guided=lambda: pytest.fail("route gate must not run"),
        _reset_stop_gate_status_error="malformed",
    )

    assert ExperimentRunner._navigate(runner) == (True, False, 4)


def test_terminal_fence_arm_service_unavailable_fails_before_request():
    client = _UnavailableClient()
    runner = SimpleNamespace(
        _terminal_fence_arm_client=client,
        _service_timeout_sec=0.25,
        _raise_if_shutdown=lambda: None,
    )

    with pytest.raises(RuntimeError, match="arm service is unavailable"):
        ExperimentRunner._arm_next_terminal_fence(runner)

    assert client.wait_count == 1
    assert client.call_count == 0
