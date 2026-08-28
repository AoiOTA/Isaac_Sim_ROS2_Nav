import math
from types import SimpleNamespace

import pytest

from robot_experiments import experiment_runner as experiment_runner_module
from robot_experiments.configuration import ConfigurationError
from robot_experiments.experiment_runner import (
    ExperimentRunner,
    OdometrySample,
    _positive_finite_float,
)


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
        module2_healthy=healthy,
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
        "module2_healthy": True,
        "place_entropy_normalized": 0.58,
        "context_uncertainty": 0.58,
    }

    ExperimentRunner._planning_prior_callback(
        runner, _planning_prior(stamp_sec=6, healthy=False)
    )
    assert runner._planning_prior_ready_streak == 0


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
        _goal_dispatch_recorded=True,
        _dynamic_guard_aborted=False,
        _wait_until=lambda predicate, _timeout: predicate(),
        _clock_seconds=lambda: 1.0,
        _pose_message=lambda specification: specification,
        _trigger_obstacle_group=lambda _goal_id: None,
        _complete_obstacle_group=lambda _goal_id: None,
        _spin_once=lambda _timeout: None,
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


def test_route_goal_complete_bool_does_not_arm_terminal_fence():
    client = SimpleNamespace(call_count=0)
    runner = SimpleNamespace(
        _route_goal_complete_epoch=0,
        _latest_route_goal_complete=False,
        _terminal_fence_arm_client=client,
    )

    ExperimentRunner._route_goal_complete_callback(
        runner, SimpleNamespace(data=True))

    assert runner._route_goal_complete_epoch == 1
    assert runner._latest_route_goal_complete is True
    assert client.call_count == 0


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
