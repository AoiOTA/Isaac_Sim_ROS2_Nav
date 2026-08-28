from __future__ import annotations

import math
from pathlib import Path

import pytest
import yaml

from isaac_sim.src.sensors.sensor_factory import (
    _lidar_omni_attributes,
    _load_lidar,
)


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
    robot = yaml.safe_load(
        (ROOT / "isaac_sim/configs/robots/jackal.yaml").read_text()
    )
    circumscribed_radius = max(
        math.hypot(float(x), float(y)) for x, y in robot["footprint"]
    )
    assert parameters["range_min"] >= circumscribed_radius + 0.05
    assert parameters["use_inf"] is True


def test_lidar_points_are_authored_in_the_published_sensor_frame():
    lidar = _load_lidar(ROOT / "isaac_sim/configs/sensors/lidar_3d.yaml")

    assert lidar["config"] == "RPLIDAR_S2E"
    assert lidar["scan_plane_rotation_wxyz"] == pytest.approx(
        [math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0]
    )
    w, x, y, z = lidar["scan_plane_rotation_wxyz"]
    rotated_sensor_z = (
        2.0 * (x * z + w * y),
        2.0 * (y * z - w * x),
        1.0 - 2.0 * (x * x + y * y),
    )
    assert rotated_sensor_z == pytest.approx([0.0, -1.0, 0.0], abs=1e-12)
    assert lidar["frame_id"] == "rtx_lidar"
    assert lidar["output_frame"] == "SENSOR"
    assert lidar["motion_compensation"] == "COMPENSATED"
    assert _lidar_omni_attributes(lidar) == {
        "omni:sensor:Core:outputFrameOfReference": "SENSOR",
        "omni:sensor:Core:outputMotionCompensationState": "COMPENSATED",
    }


def test_navigation_app_enables_motion_bvh_before_kit_startup():
    source = (ROOT / "isaac_sim/apps/navigation_sim.py").read_text()

    assert '"--/renderer/raytracingMotion/enabled=true"' in source
    assert '"--/renderer/raytracingMotion/enableHydraEngineMasking=true"' in source
    assert '"--/rtx/rendering/perSensorTickTlas=true"' in source


def test_physical_lidar_tf_remains_the_measured_mount_pose():
    robot = yaml.safe_load(
        (ROOT / "isaac_sim/configs/robots/jackal.yaml").read_text()
    )
    lidar_tf = next(
        transform
        for transform in robot["static_transforms"]
        if transform["child"] == "lidar_link"
    )

    assert lidar_tf["translation"] == [0.120, 0.000, 0.333]
    assert lidar_tf["rotation_xyzw"] == [0.0, 0.0, 0.0, 1.0]
    sensor_tf = next(
        transform
        for transform in robot["static_transforms"]
        if transform["child"] == "rtx_lidar"
    )
    assert sensor_tf["parent"] == "lidar_link"
    assert sensor_tf["translation"] == [0.0, 0.0, 0.0]
    assert sensor_tf["rotation_xyzw"] == pytest.approx(
        [math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)]
    )

    xacro = (
        ROOT / "ros2_ws/src/robot_description/urdf/jackal_sensors.xacro"
    ).read_text()
    assert 'xyz="0.120 0.000 0.333"/>' in xacro
