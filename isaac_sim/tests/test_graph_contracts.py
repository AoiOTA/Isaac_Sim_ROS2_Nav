from __future__ import annotations

from pathlib import Path

import pytest

from isaac_sim.graphs.control_graph import control_graph_spec
from isaac_sim.graphs.odometry_graph import ideal_odometry_graph_spec
from isaac_sim.graphs.ros_contract import load_qos_profiles, load_topics
from isaac_sim.graphs.sensor_graph import core_sensor_graph_spec, lidar_graph_spec
from isaac_sim.graphs.tf_graph import structure_tf_graph_spec
from isaac_sim.src.bridge.tf_ownership import (
    TfOwnershipError,
    expected_tf_owners,
    validate_tf_publishers,
)
from isaac_sim.src.config import load_project_config


ROOT = Path(__file__).resolve().parents[2]


def _config(mode: str = "ideal"):
    return load_project_config(
        ROOT / "isaac_sim/configs/project.yaml",
        {
            "PROJECT_ROOT": str(ROOT),
            "ISAAC_ASSET_ROOT": "/home/lyb/isaacsim_assets/Assets/Isaac/6.0",
            "ISAAC_NAV__SIMULATION__ODOMETRY_MODE": mode,
        },
    )


def test_control_sensor_and_ideal_odometry_specs_validate():
    config = _config()
    specs = (
        control_graph_spec(config),
        core_sensor_graph_spec(config, "/World/Robots/Jackal/base_link/imu_link/imu_sensor"),
        lidar_graph_spec(config, "/Render/Product"),
        ideal_odometry_graph_spec(config),
    )
    for spec in specs:
        spec.validate()
        assert "ground_truth" not in repr(spec).lower()
    control = specs[0]
    assert control.on_demand is True
    node_types = dict(control.nodes)
    assert node_types["OnPhysicsStep"] \
        == "isaacsim.core.nodes.OnPhysicsStep"
    assert "OnPlaybackTick" not in node_types
    assert [
        name
        for name, node_type in control.nodes
        if node_type.endswith("IsaacArticulationController")
    ] == ["WheelController"]
    assert (
        "OnPhysicsStep.outputs:deltaSimulationTime",
        "DifferentialController.inputs:dt",
    ) in control.connections
    assert not any(
        name == "DifferentialController.inputs:dt"
        for name, _ in control.values
    )
    assert (
        "SubscribeTwist.outputs:execOut",
        "DifferentialController.inputs:execIn",
    ) not in control.connections
    assert (
        "FourWheelCommand.outputs:array",
        "WheelController.inputs:velocityCommand",
    ) in control.connections
    assert {
        (
            "LeftWheelCommand.outputs:value",
            "FourWheelCommand.inputs:input0",
        ),
        (
            "RightWheelCommand.outputs:value",
            "FourWheelCommand.inputs:input1",
        ),
        (
            "LeftWheelCommand.outputs:value",
            "FourWheelCommand.inputs:input2",
        ),
        (
            "RightWheelCommand.outputs:value",
            "FourWheelCommand.inputs:input3",
        ),
    }.issubset(set(control.connections))
    assert dict(control.values)["FourWheelCommand.inputs:arraySize"] == 4
    assert dict(control.values)["WheelController.inputs:jointNames"] == [
        config.robot.front_wheel_joints[0],
        config.robot.front_wheel_joints[1],
        config.robot.rear_wheel_joints[0],
        config.robot.rear_wheel_joints[1],
    ]
    control_values = dict(specs[0].values)
    assert control_values[
        "DifferentialController.inputs:wheelDistance"
    ] == pytest.approx(0.800)
    assert control_values[
        "DifferentialController.inputs:maxAngularAcceleration"
    ] == pytest.approx(6.0)


def test_static_sensor_frames_use_raw_tf_and_no_world_frame():
    spec = structure_tf_graph_spec(_config())
    spec.validate()
    node_types = dict(spec.nodes)
    assert node_types["WheelTF"].endswith("ROS2PublishTransformTree")
    for node, node_type in node_types.items():
        if node.endswith("TF") and node != "WheelTF":
            assert node_type.endswith("ROS2PublishRawTransformTree")
    frame_values = [
        value
        for attribute, value in spec.values
        if attribute.endswith(("parentFrameId", "childFrameId"))
    ]
    assert "world" not in frame_values


def test_tf_ownership_requires_exactly_one_publisher():
    expected = expected_tf_owners("realistic")
    validate_tf_publishers("realistic", {key: [owner] for key, owner in expected.items()})
    duplicated = {key: [owner] for key, owner in expected.items()}
    duplicated["odom->base_link"].append("isaac")
    with pytest.raises(TfOwnershipError, match="sole owner"):
        validate_tf_publishers("realistic", duplicated)

    rsp_expected = expected_tf_owners("realistic", "rsp")
    assert rsp_expected["base_link->sensor_links"] == "rsp"
    validate_tf_publishers(
        "realistic",
        {key: [owner] for key, owner in rsp_expected.items()},
        "rsp",
    )
    with pytest.raises(TfOwnershipError, match="ideal odometry requires"):
        expected_tf_owners("ideal", "rsp")


def test_topic_and_qos_contracts_are_absolute_and_encoded():
    topics = load_topics(ROOT / "isaac_sim/configs/ros2_bridge/topics.yaml")
    qos = load_qos_profiles(ROOT / "isaac_sim/configs/ros2_bridge/qos.yaml")
    assert topics["pointcloud"] == "/lidar/points_raw"
    assert topics["frames"]["base"] == "base_link"
    assert '"reliability":"bestEffort"' in qos["sensor_data"]
    assert '"durability":"transientLocal"' in qos["static_tf"]
