from pathlib import Path
from types import SimpleNamespace

import pytest

from robot_bringup.ordered_shutdown import _call_step
from robot_bringup.ordered_shutdown import ShutdownStep
from robot_bringup.ordered_shutdown import shutdown_steps


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_navigation_stops_navigation_before_localization():
    steps = shutdown_steps("navigation")
    assert [step.service for step in steps] == [
        "/lifecycle_manager_navigation/manage_nodes",
        "/lifecycle_manager_localization/manage_nodes",
    ]
    assert [step.command for step in steps] == [4, 4]


def test_mapping_deactivates_cleans_and_shuts_down_slam():
    for operation in ("mapping", "incremental_mapping"):
        steps = shutdown_steps(operation)
        assert [step.command for step in steps] == [4, 2, 5]
        assert {step.service for step in steps} == {"/slam_toolbox/change_state"}


def test_localization_uses_only_localization_manager():
    steps = shutdown_steps("localization")
    assert len(steps) == 1
    assert steps[0].service == "/lifecycle_manager_localization/manage_nodes"


def test_unknown_operation_is_rejected():
    with pytest.raises(ValueError, match="unsupported operation"):
        shutdown_steps("invalid")


def test_console_entry_point_is_installed():
    setup = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")
    assert "ordered_shutdown = robot_bringup.ordered_shutdown:main" in setup


def test_helper_spins_with_its_private_rclpy_context():
    source = (
        PACKAGE_ROOT / "robot_bringup" / "ordered_shutdown.py"
    ).read_text(encoding="utf-8")
    assert "SingleThreadedExecutor(context=context)" in source
    assert "executor=executor" in source
    assert "deadline = time.monotonic() + timeout_s" in source
    assert "remaining_s = deadline - started" in source


def test_service_discovery_uses_the_full_shutdown_handshake_timeout():
    observed_timeouts = []

    class UnavailableClient:
        def wait_for_service(self, *, timeout_sec):
            observed_timeouts.append(timeout_sec)
            return False

    client = UnavailableClient()
    destroyed = []
    node = SimpleNamespace(
        create_client=lambda _service_type, _service: client,
        destroy_client=destroyed.append,
    )
    step = ShutdownStep(
        "navigation lifecycle manager",
        "/lifecycle_manager_navigation/manage_nodes",
        "manager",
        4,
    )

    success, detail = _call_step(node, None, step, timeout_s=8.0)

    assert not success
    assert detail == "service unavailable after 8.0s"
    assert observed_timeouts == [8.0]
    assert destroyed == [client]
