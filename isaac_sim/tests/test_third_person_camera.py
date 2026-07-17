from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import pytest

from isaac_sim.src.config import load_project_config
from isaac_sim.src.visualization.third_person_camera import (
    relative_camera_pose,
)


ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "isaac_sim/configs/project.yaml"
ASSET_ROOT = Path("/home/lyb/isaacsim_assets/Assets/Isaac/6.0")


def _camera_config():
    project = load_project_config(
        CONFIG,
        {
            "PROJECT_ROOT": str(ROOT),
            "ISAAC_ASSET_ROOT": str(ASSET_ROOT),
        },
    )
    return project.third_person_camera


def test_camera_is_behind_and_above_base_link_and_looks_forward():
    config = _camera_config()
    pose = relative_camera_pose(config)

    assert pose.eye == pytest.approx((-3.2, 0.0, 2.2))
    assert pose.aim == pytest.approx((1.0, 0.0, 0.25))
    assert pose.eye[0] < 0.0 < pose.aim[0]
    assert pose.eye[2] > pose.aim[2]


def test_relative_pose_uses_configured_distance_and_height():
    config = replace(
        _camera_config(),
        distance_m=4.5,
        height_m=5.5,
        look_ahead_m=1.0,
    )
    pose = relative_camera_pose(config)

    assert pose.eye == pytest.approx((-4.5, 0.0, 5.5))
    assert pose.aim[0] == pytest.approx(1.0)
