from types import SimpleNamespace

import pytest

from robot_experiments.experiment_runner import ExperimentRunner


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
