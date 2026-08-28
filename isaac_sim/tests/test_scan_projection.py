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


def test_sensor_local_raw_axes_are_already_horizontal_in_the_mount_frame():
    lidar = _load_lidar(ROOT / "isaac_sim/configs/sensors/lidar_3d.yaml")

    assert lidar["config"] == "RPLIDAR_S2E"
    assert lidar["scan_plane_rotation_wxyz"] == pytest.approx(
        [math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0]
    )
    # Frozen from the d221 sensor-local PointCloud capture. The two broad
    # horizontal axes and near-zero vertical spread prove that SENSOR output
    # already incorporates the authored prim rotation.
    raw_axis_std_m = (2.889, 5.597, 1.67e-6)
    assert raw_axis_std_m[2] < min(raw_axis_std_m[:2]) * 1e-5
    assert lidar["frame_id"] == "lidar_link"
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
    assert not any(
        transform
        for transform in robot["static_transforms"]
        if transform["parent"] == "lidar_link"
    )

    xacro = (
        ROOT / "ros2_ws/src/robot_description/urdf/jackal_sensors.xacro"
    ).read_text()
    assert 'xyz="0.120 0.000 0.333"/>' in xacro
    assert xacro.count('parent="base_link" child="lidar_link"') == 1
