from __future__ import annotations

import ast
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _read(relative: str) -> str:
    return (PACKAGE_ROOT / relative).read_text(encoding="utf-8")


def test_launch_is_default_off_and_starts_only_the_ndt_shadow_node():
    launch_source = _read("launch/ndt_shadow.launch.py")
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
    assert 'executable="ndt_local_odometry_node"' in launch_source


def test_config_has_exact_isolated_topics_frames_and_fixed_ndt_parameters():
    config = yaml.safe_load(_read("config/ndt_shadow.yaml"))[
        "ndt_local_odometry_node"
    ]["ros__parameters"]
    assert config["input_topic"] == "/lio/points_raw"
    assert config["odom_topic"] == "/local_odom/ndt_shadow"
    assert config["status_topic"] == "/local_odom/ndt_status"
    assert config["lidar_frame"] == "lio_lidar_link"
    assert config["base_frame"] == "base_link"
    assert config["odom_frame"] == "ndt_odom_shadow"
    assert config["voxel_leaf_size"] == 0.15
    assert config["min_points"] == 100
    assert config["resolution"] == 0.5
    assert config["step_size"] == 0.1
    assert config["max_iterations"] == 40
    assert config["transformation_epsilon"] == 0.001
    assert config["max_fitness_score"] == 0.25
    assert len(config["pose_covariance_diagonal"]) == 6
    assert config["twist_covariance_diagonal"] == [1_000_000.0] * 6


def test_product_contract_has_no_forbidden_inputs_tf_output_or_old_backend():
    product_files = [
        "src/ndt_local_odometry_node.cpp",
        "src/ndt_odometry.cpp",
        "include/pointcloud_local_odometry/ndt_odometry.hpp",
        "CMakeLists.txt",
        "package.xml",
        "config/ndt_shadow.yaml",
        "launch/ndt_shadow.launch.py",
    ]
    implementation = "\n".join(_read(path) for path in product_files)
    assert "rclcpp::SensorDataQoS()" in implementation
    assert "pcl::NormalDistributionsTransform<Point, Point>" in implementation
    assert "pcl::PointXYZI" in implementation
    assert implementation.count("create_subscription<") == 1
    assert "lookupTransform(" in implementation
    assert "TransformBroadcaster" not in implementation
    assert "StaticTransformBroadcaster" not in implementation
    for token in ["/wheel", "/imu", "/map", "/ground_truth"]:
        assert token not in implementation
    assert "gicp" not in implementation.lower()
    assert not (PACKAGE_ROOT / "config/gicp_shadow.yaml").exists()
    assert not (PACKAGE_ROOT / "launch/gicp_shadow.launch.py").exists()
    assert not (PACKAGE_ROOT / "src/gicp_shadow_node.cpp").exists()
    assert not (PACKAGE_ROOT / "src/gicp_odometry.cpp").exists()
    assert not (
        PACKAGE_ROOT / "include/pointcloud_local_odometry/gicp_odometry.hpp"
    ).exists()
