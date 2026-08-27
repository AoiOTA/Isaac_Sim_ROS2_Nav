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
