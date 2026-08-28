from __future__ import annotations

import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest

from isaac_sim.graphs.control_graph import control_graph_spec
from isaac_sim.graphs import odometry_graph
from isaac_sim.graphs.odometry_graph import ideal_odometry_graph_spec
from isaac_sim.graphs.ros_contract import load_qos_profiles, load_topics
from isaac_sim.graphs.sensor_graph import core_sensor_graph_spec, lidar_graph_spec
from isaac_sim.graphs.spec import materialize_graph
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
    lidar = specs[2]
    odometry = specs[-1]
    assert control.on_demand is True
    assert dict(control.values)["SubscribeTwist.inputs:topicName"] \
        == "/cmd_vel_sim"
    node_types = dict(control.nodes)
    assert node_types["OnPhysicsStep"] \
        == "isaacsim.core.nodes.OnPhysicsStep"
    assert "OnPlaybackTick" not in node_types
    assert odometry.on_demand is True
    odometry_nodes = dict(odometry.nodes)
    assert odometry_nodes["OnImpulseEvent"] == "omni.graph.action.OnImpulseEvent"
    assert "OnPlaybackTick" not in odometry_nodes
    assert (
        "OnImpulseEvent.outputs:execOut",
        "PublishOdometry.inputs:execIn",
    ) in odometry.connections
    assert (
        "OnImpulseEvent.outputs:execOut",
        "PublishOdomTF.inputs:execIn",
    ) in odometry.connections
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
    ] == pytest.approx(6.5)
    assert dict(lidar.values)["PointCloudPublisher.inputs:frameId"] \
        == "lidar_link"


def test_core_sensors_publish_once_per_physics_step():
    spec = core_sensor_graph_spec(
        _config(),
        "/World/Robots/Jackal/base_link/imu_link/imu_sensor",
    )
    spec.validate()
    assert spec.on_demand is True
    node_types = dict(spec.nodes)
    assert node_types["OnPhysicsStep"] == "isaacsim.core.nodes.OnPhysicsStep"
    assert list(node_types.values()).count("isaacsim.core.nodes.OnPhysicsStep") == 1
    assert "OnPlaybackTick" not in node_types

    connections = set(spec.connections)
    assert len(connections) == len(spec.connections)
    expected_exec_connections = {
        ("OnPhysicsStep.outputs:step", "PublishClock.inputs:execIn"),
        ("OnPhysicsStep.outputs:step", "PublishJointState.inputs:execIn"),
        ("OnPhysicsStep.outputs:step", "ReadIMU.inputs:execIn"),
        ("ReadIMU.outputs:execOut", "PublishIMU.inputs:execIn"),
    }
    exec_connections = {
        connection
        for connection in connections
        if connection[1].endswith("inputs:execIn")
    }
    assert exec_connections == expected_exec_connections
    for target in ("PublishClock", "PublishJointState", "ReadIMU", "PublishIMU"):
        assert sum(
            destination == f"{target}.inputs:execIn"
            for _, destination in connections
        ) == 1
    assert dict(spec.values)["PublishIMU.inputs:topicName"] == "/imu/data_raw"


def test_core_sensors_materialize_on_demand(monkeypatch):
    spec = core_sensor_graph_spec(
        _config(),
        "/World/Robots/Jackal/base_link/imu_link/imu_sensor",
    )
    captured = {}
    on_demand_stage = object()

    class Controller:
        Keys = SimpleNamespace(
            CREATE_NODES="create_nodes",
            CONNECT="connect",
            SET_VALUES="set_values",
            CREATE_ATTRIBUTES="create_attributes",
        )

        @staticmethod
        def edit(graph_description, edit):
            captured["graph_description"] = graph_description
            captured["edit"] = edit
            return "graph", "nodes", None, None

    class Prim:
        @staticmethod
        def IsValid():
            return False

    stage = SimpleNamespace(GetPrimAtPath=lambda _path: Prim())
    core = ModuleType("omni.graph.core")
    core.Controller = Controller
    core.GraphPipelineStage = SimpleNamespace(
        GRAPH_PIPELINE_STAGE_ONDEMAND=on_demand_stage,
    )
    graph = ModuleType("omni.graph")
    graph.core = core
    usd = ModuleType("omni.usd")
    usd.get_context = lambda: SimpleNamespace(get_stage=lambda: stage)
    omni = ModuleType("omni")
    omni.__path__ = []
    omni.graph = graph
    omni.usd = usd
    usdrt = ModuleType("usdrt")
    usdrt.Sdf = SimpleNamespace(Path=lambda path: path)
    for name, module in {
        "omni": omni,
        "omni.graph": graph,
        "omni.graph.core": core,
        "omni.usd": usd,
        "usdrt": usdrt,
    }.items():
        monkeypatch.setitem(sys.modules, name, module)

    graph_result, nodes_result = materialize_graph(spec)

    assert (graph_result, nodes_result) == ("graph", "nodes")
    assert captured["graph_description"] == {
        "graph_path": "/World/Graphs/Sensors",
        "pipeline_stage": on_demand_stage,
    }
    assert "evaluator_name" not in captured["graph_description"]


def test_isaac_static_tf_uses_the_physical_lidar_mount_without_extra_frame():
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
    raw_tf_nodes = [
        node
        for node, node_type in spec.nodes
        if node_type.endswith("ROS2PublishRawTransformTree")
    ]
    assert len(raw_tf_nodes) == 9
    values = dict(spec.values)
    edges = {
        (
            values[f"{node}.inputs:parentFrameId"],
            values[f"{node}.inputs:childFrameId"],
        )
        for node in raw_tf_nodes
    }
    assert ("base_link", "lidar_link") in edges
    assert sum(child == "lidar_link" for _, child in edges) == 1
    assert not any(parent == "lidar_link" for parent, _ in edges)


def test_tf_ownership_requires_exactly_one_publisher():
    expected = expected_tf_owners("realistic")
    assert expected["map->odom"] == "amcl"
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
    assert expected_tf_owners("ideal")["map->odom"] == "ideal_localization_tf"
    assert expected_tf_owners("estimated")["odom->base_link"] == (
        "robot_localization"
    )
    mixed = expected_tf_owners("mixed")
    assert mixed["odom->base_link"] == "isaac_compute_odometry"
    assert mixed["map->odom"] == "amcl"
    validate_tf_publishers(
        "mixed", {key: [owner] for key, owner in mixed.items()}
    )
    with pytest.raises(TfOwnershipError, match="mixed odometry requires"):
        expected_tf_owners("mixed", "rsp")


def test_mixed_builds_the_compute_odometry_graph(monkeypatch):
    mixed = _config("mixed")
    spec = ideal_odometry_graph_spec(mixed)
    spec.validate()
    assert dict(spec.values)["PublishOdometry.inputs:topicName"] == "/odom"
    assert dict(spec.values)["PublishOdomTF.inputs:parentFrameId"] == "odom"

    built = object()
    monkeypatch.setattr(
        odometry_graph.IdealOdomPublisher,
        "create",
        classmethod(
            lambda _cls, config, epoch=0: (
                built, config.simulation.odometry_mode, epoch
            )
        ),
    )
    assert odometry_graph.build_odometry_graph(mixed, epoch=4) == (
        built, "mixed", 4
    )
    assert odometry_graph.build_odometry_graph(_config("realistic")) is None


def test_topic_and_qos_contracts_are_absolute_and_encoded():
    topics = load_topics(ROOT / "isaac_sim/configs/ros2_bridge/topics.yaml")
    qos = load_qos_profiles(ROOT / "isaac_sim/configs/ros2_bridge/qos.yaml")
    assert topics["pointcloud"] == "/lidar/points_raw"
    assert topics["frames"]["base"] == "base_link"
    assert topics["frames"]["lidar"] == "lidar_link"
    assert '"reliability":"bestEffort"' in qos["sensor_data"]
    assert '"depth":2' in qos["camera_sensor_data"]
    assert '"durability":"transientLocal"' in qos["static_tf"]
