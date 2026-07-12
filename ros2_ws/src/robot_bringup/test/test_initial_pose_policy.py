import pytest

from robot_bringup.initial_pose_policy import normalize_initial_pose_source


@pytest.mark.parametrize(
    ("raw", "expected"),
    [("auto", "auto"), (" RVIZ ", "rviz")],
)
def test_normalize_initial_pose_source(raw, expected):
    assert normalize_initial_pose_source(raw) == expected


@pytest.mark.parametrize("raw", ["", "manual", None])
def test_normalize_initial_pose_source_rejects_unknown_values(raw):
    with pytest.raises(ValueError, match="auto or rviz"):
        normalize_initial_pose_source(raw)
