from __future__ import annotations

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]


def test_pointcloud_projection_matches_2d_navigation_baseline():
    document = yaml.safe_load(
        (ROOT / "ros2_ws/src/robot_perception/config/pointcloud_to_laserscan.yaml").read_text()
    )
    parameters = document["pointcloud_to_laserscan"]["ros__parameters"]
    assert parameters["use_sim_time"] is True
    assert parameters["target_frame"] == "base_link"
    assert 0.0 <= parameters["min_height"] < parameters["max_height"]
    assert parameters["angle_min"] < -3.14
    assert parameters["angle_max"] > 3.14
    assert 0.0 < parameters["range_min"] < parameters["range_max"]
    assert parameters["use_inf"] is True
