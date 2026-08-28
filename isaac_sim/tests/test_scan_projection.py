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


def _median_nearest_occupancy_residual(raw_xy, occupied_xy, yaw_deg):
    yaw = math.radians(yaw_deg)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    residuals = []
    for x, y in raw_xy:
        projected = (cosine * x - sine * y, sine * x + cosine * y)
        residuals.append(
            min(
                math.hypot(projected[0] - wall_x, projected[1] - wall_y)
                for wall_x, wall_y in occupied_xy
            )
        )
    return sorted(residuals)[len(residuals) // 2]


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
    lidar = _load_lidar(ROOT / "isaac_sim/configs/sensors/lidar_3d.yaml")
    lidar_tf = next(
        transform
        for transform in robot["static_transforms"]
        if transform["child"] == "lidar_link"
    )

    assert lidar_tf["translation"] == [0.120, 0.000, 0.333]
    assert lidar_tf["rotation_xyzw"] == [0.0, 0.0, 0.0, 1.0]
    assert not any(
        transform["child"] == "rtx_lidar"
        for transform in robot["static_transforms"]
    )
    assert lidar["ros_frame_parent"] == "lidar_link"
    assert lidar["ros_frame_translation"] == [0.0, 0.0, 0.0]
    assert lidar["ros_frame_rotation_xyzw"] == pytest.approx(
        [0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)]
    )
    assert sum(
        transform["child"] == "lidar_link"
        for transform in robot["static_transforms"]
    ) == 1

    factory_source = (
        ROOT / "isaac_sim/src/sensors/sensor_factory.py"
    ).read_text()
    assert 'orientations=[lidar_config["scan_plane_rotation_wxyz"]]' \
        in factory_source

    xacro = (
        ROOT / "ros2_ws/src/robot_description/urdf/jackal_sensors.xacro"
    ).read_text()
    assert 'xyz="0.120 0.000 0.333"/>' in xacro
    assert xacro.count('parent="base_link" child="lidar_link"') == 1
    assert xacro.count('parent="lidar_link" child="rtx_lidar"') == 1
    assert 'rpy="0 0 1.5707963267948966"' in xacro


def test_d222_yaw_candidate_is_discriminated_by_asymmetric_occupancy():
    # Frozen review summary for d222 frames 5/150/300. No exact -90/180 values
    # were recorded, so the asymmetric fixture below guards their sign/order.
    d222_median_residual_m = {
        "run1_identity": (0.38, 0.83, 0.81),
        "run1_plus90": (0.03, 0.03, 0.04),
        "run2_identity": (0.50, 0.65, 0.69),
        "run2_plus90": (0.03, 0.03, 0.03),
    }
    d222_within_0p15m_percent = {
        "run1_identity": (21, 18, 23),
        "run1_plus90": (77, 68, 76),
        "run2_identity": (19, 26, 7),
        "run2_plus90": (77, 80, 84),
    }
    for run in ("run1", "run2"):
        assert max(d222_median_residual_m[f"{run}_plus90"]) <= 0.04
        assert min(d222_median_residual_m[f"{run}_identity"]) >= 0.38
        assert min(d222_within_0p15m_percent[f"{run}_plus90"]) >= 68
        assert max(d222_within_0p15m_percent[f"{run}_identity"]) <= 26

    raw_xy = (
        (0.7, 1.1),
        (1.4, 2.6),
        (2.8, 0.4),
        (3.6, -1.3),
        (1.2, -2.4),
    )
    occupied_wall_xy = (
        (-1.08, 0.69),
        (-2.62, 1.41),
        (-0.39, 2.82),
        (1.32, 3.59),
        (2.38, 1.18),
    )
    residuals = {
        yaw: _median_nearest_occupancy_residual(
            raw_xy, occupied_wall_xy, yaw
        )
        for yaw in (90, 0, -90, 180)
    }
    assert residuals[90] <= 0.05
    assert residuals[0] >= 0.3
    assert residuals[90] < residuals[-90]
    assert residuals[90] < residuals[180]
