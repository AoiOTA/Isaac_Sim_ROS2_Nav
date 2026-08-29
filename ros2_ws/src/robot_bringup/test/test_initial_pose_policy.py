from pathlib import Path

import pytest

from robot_bringup.initial_pose_policy import normalize_initial_pose_source


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize(
    ("raw", "expected"),
    [("auto", "auto"), (" RVIZ ", "rviz"), (" ISAAC ", "isaac")],
)
def test_normalize_initial_pose_source(raw, expected):
    assert normalize_initial_pose_source(raw) == expected


@pytest.mark.parametrize("raw", ["", "manual", None])
def test_normalize_initial_pose_source_rejects_unknown_values(raw):
    with pytest.raises(ValueError, match="auto, rviz, or isaac"):
        normalize_initial_pose_source(raw)


def test_main_treats_external_context_shutdown_as_clean_exit():
    source = (
        PACKAGE_ROOT / "robot_bringup" / "initial_pose_policy.py"
    ).read_text(encoding="utf-8")
    assert "except (KeyboardInterrupt, ExternalShutdownException):" in source
    assert source.index("node.destroy_node()") < source.index("rclpy.shutdown()")
