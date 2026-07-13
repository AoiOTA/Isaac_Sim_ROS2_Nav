import ast
from pathlib import Path
import xml.etree.ElementTree as ET

import jsonschema
import pytest
import yaml


PACKAGE_ROOT = Path(__file__).parents[1]


@pytest.mark.parametrize(
    "filename",
    [
        "initial_pose.launch.py",
        "experiment.launch.py",
        "scan_fault_bridge.launch.py",
    ],
)
def test_launch_files_define_generate_launch_description(filename):
    source = (PACKAGE_ROOT / "launch" / filename).read_text()
    tree = ast.parse(source)
    functions = {node.name for node in tree.body if isinstance(node, ast.FunctionDef)}
    assert "generate_launch_description" in functions
    assert "LaunchDescription" in source
    assert "Node(" in source


def test_package_metadata_declares_runtime_contract():
    root = ET.parse(PACKAGE_ROOT / "package.xml").getroot()
    assert root.findtext("name") == "robot_experiments"
    dependencies = {element.text for element in root.findall("exec_depend")}
    assert {
        "action_msgs",
        "geometry_msgs",
        "nav2_msgs",
        "nav_msgs",
        "rclpy",
        "rosgraph_msgs",
        "std_srvs",
        "tf2_ros",
    } <= dependencies
    assert root.find("export/build_type").text == "ament_python"


@pytest.mark.parametrize(
    "filename",
    [
        "static.yaml",
        "static_long_range.yaml",
        "dynamic.yaml",
        "incremental_mapping.yaml",
    ],
)
def test_examples_validate_against_installed_schema(filename):
    schema = yaml.safe_load((PACKAGE_ROOT / "config" / "scenario.schema.yaml").read_text())
    instance = yaml.safe_load((PACKAGE_ROOT / "config" / filename).read_text())
    jsonschema.Draft202012Validator(schema).validate(instance)


def test_package_does_not_install_a_second_spawn_pose_truth_source():
    assert not (PACKAGE_ROOT / "config" / "spawn_poses.yaml").exists()
    launch_source = (PACKAGE_ROOT / "launch" / "initial_pose.launch.py").read_text()
    assert "ISAAC_NAV_SPAWN_POSES" in launch_source


def test_incremental_map_comparison_has_an_installed_cli():
    setup_source = (PACKAGE_ROOT / "setup.py").read_text()
    assert (
        "incremental_map_compare = "
        "robot_experiments.incremental_map_compare:main"
    ) in setup_source


def test_scan_fault_bridge_has_an_installed_cli_and_opt_in_output():
    setup_source = (PACKAGE_ROOT / "setup.py").read_text()
    assert (
        "scan_fault_bridge = robot_experiments.scan_fault_bridge:main"
        in setup_source
    )
    launch_source = (PACKAGE_ROOT / "launch" / "scan_fault_bridge.launch.py").read_text()
    assert 'default_value="/scan"' in launch_source
    assert 'default_value="/scan_fault"' in launch_source
    assert 'default_value="/simulation/reset_event"' in launch_source


def test_runner_has_no_publishers_or_control_and_localization_topics():
    source = (PACKAGE_ROOT / "robot_experiments" / "experiment_runner.py").read_text()
    tree = ast.parse(source)
    attribute_calls = {
        node.func.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    }
    assert "create_publisher" not in attribute_calls
    assert "/cmd_vel" not in source
    assert "/initialpose" not in source
    assert "self._scenario.goal" in source
    assert "_verify_dynamic_runtime_contract" in source
    assert "dynamic_obstacles_config_sha256" in source
    assert '"/simulation/localization_seeded"' in source
    assert "stamp_s > tf_stamp_barrier_s" in source
    assert "ExternalShutdownException" in source
    assert "ExperimentIsolationError" in source
    assert "odom.stamp_s > sample_stamp_barrier_s" in source


def test_initial_pose_contract_waits_for_clock_and_uses_reliable_qos():
    source = (PACKAGE_ROOT / "robot_experiments" / "initial_pose_publisher.py").read_text()
    assert '"/clock"' in source
    assert "ReliabilityPolicy.RELIABLE" in source
    assert "require_calibrated=True" in source
    assert "wait_for_odom_to_base_tf" in source
    assert "lookup_transform(" in source
    assert "self._odom_frame, self._base_frame, Time()" in source
    assert "wait_for_map_to_odom_tf" not in source
    assert "publish_count" in source
    assert "Buffer(node=self)" in source
    assert '"/initial_pose/reseed"' in source
    assert "_initial_pose_callback" in source
    assert "external /initialpose accepted" in source
    assert "simulation clock rollback" in source
    assert "stay_alive_for_reseed" in source
    assert '"/scan"' in source
    assert '"/simulation/reset_event"' in source
    assert '"/initial_pose/status"' in source
    assert "PostResetScanBarrier" in source
    assert "manual RViz initial pose remains authoritative" in source
