from __future__ import annotations

import ast
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _read(relative: str) -> str:
    return (PACKAGE_ROOT / relative).read_text(encoding="utf-8")


def test_launch_is_default_off_and_starts_only_the_shadow_node():
    launch_source = _read("launch/gicp_shadow.launch.py")
    tree = ast.parse(launch_source)
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)]
    declarations = [
        call
        for call in calls
        if getattr(call.func, "id", None) == "DeclareLaunchArgument"
    ]
    assert len(declarations) == 1
    assert declarations[0].args[0].value == "enabled"
    defaults = {
        keyword.arg: keyword.value.value
        for keyword in declarations[0].keywords
        if keyword.arg == "default_value"
    }
    assert defaults["default_value"] == "false"
    assert "IfCondition(enabled)" in launch_source
    assert launch_source.count("Node(") == 1


def test_config_has_exact_isolated_topics_frames_and_fixed_gicp_parameters():
    config = yaml.safe_load(_read("config/gicp_shadow.yaml"))["gicp_shadow_node"][
        "ros__parameters"
    ]
    assert config["input_topic"] == "/lio/points_raw"
    assert config["odom_topic"] == "/local_odom/gicp_shadow"
    assert config["status_topic"] == "/local_odom/gicp_status"
    assert config["lidar_frame"] == "lio_lidar_link"
    assert config["base_frame"] == "base_link"
    assert config["odom_frame"] == "gicp_odom_shadow"
    assert config["voxel_leaf_size"] > 0.0
    assert config["min_points"] > 3
    assert config["max_correspondence_distance"] > 0.0
    assert config["max_iterations"] > 0
    assert config["max_fitness_score"] >= 0.0
    assert len(config["pose_covariance_diagonal"]) == 6
    assert config["twist_covariance_diagonal"] == [1_000_000.0] * 6


def test_source_contract_has_sensor_qos_no_tf_broadcaster_and_no_other_inputs():
    implementation = "\n".join(
        [
            _read("src/gicp_shadow_node.cpp"),
            _read("src/gicp_odometry.cpp"),
            _read("include/pointcloud_local_odometry/gicp_odometry.hpp"),
            _read("CMakeLists.txt"),
            _read("package.xml"),
            _read("config/gicp_shadow.yaml"),
            _read("launch/gicp_shadow.launch.py"),
        ]
    )
    assert "rclcpp::SensorDataQoS()" in implementation
    assert implementation.count("create_subscription<") == 1
    assert "lookupTransform(" in implementation
    assert "TransformBroadcaster" not in implementation
    assert "StaticTransformBroadcaster" not in implementation
    forbidden = [
        "/" + "wheel",
        "/" + "imu",
        "/" + "map",
        "/ground_" + "truth",
    ]
    for token in forbidden:
        assert token not in implementation
