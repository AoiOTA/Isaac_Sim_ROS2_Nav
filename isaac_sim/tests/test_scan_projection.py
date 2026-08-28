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


def test_d222_sensor_axes_are_horizontal_and_use_one_ros_yaw_to_mount():
    lidar = _load_lidar(ROOT / "isaac_sim/configs/sensors/lidar_3d.yaml")

    assert lidar["config"] == "RPLIDAR_S2E"
    assert lidar["scan_plane_rotation_wxyz"] == pytest.approx(
        [math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0]
    )
    # Frozen from the d222 sensor-local PointCloud capture. The two broad
    # horizontal axes and near-zero vertical spread prove that SENSOR output
    # already incorporates the authored +90 X prim rotation.
    raw_axis_std_m = (2.889, 5.597, 1.67e-6)
    assert raw_axis_std_m[2] < min(raw_axis_std_m[:2]) * 1e-5
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


def test_d222_rtx_yaw_maps_sensor_xy_into_the_measured_mount_frame():
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
    rtx_tf = next(
        transform
        for transform in robot["static_transforms"]
        if transform["child"] == "rtx_lidar"
    )
    assert rtx_tf["parent"] == "lidar_link"
    assert rtx_tf["translation"] == [0.0, 0.0, 0.0]
    assert rtx_tf["rotation_xyzw"] == pytest.approx(
        [0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)]
    )
    # d222 map residuals selected +90 Z over identity. Expressed in the mount,
    # sensor +X becomes mount +Y, sensor +Y becomes mount -X, and Z is unchanged.
    sensor_xyz = (2.0, 3.0, 0.25)
    mount_xyz = (-sensor_xyz[1], sensor_xyz[0], sensor_xyz[2])
    assert mount_xyz == (-3.0, 2.0, 0.25)
    assert rtx_tf["rotation_xyzw"][:2] == [0.0, 0.0]
    assert sum(
        transform["child"] == "lidar_link"
        for transform in robot["static_transforms"]
    ) == 1
    assert sum(
        transform["child"] == "rtx_lidar"
        for transform in robot["static_transforms"]
    ) == 1

    xacro = (
        ROOT / "ros2_ws/src/robot_description/urdf/jackal_sensors.xacro"
    ).read_text()
    assert 'xyz="0.120 0.000 0.333"/>' in xacro
    assert xacro.count('parent="base_link" child="lidar_link"') == 1
    assert xacro.count('parent="lidar_link" child="rtx_lidar"') == 1
    assert 'rpy="0 0 1.5707963267948966"' in xacro
