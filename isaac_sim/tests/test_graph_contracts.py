from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import pytest

from isaac_sim.apps.navigation_sim import _parser, _rebuild_control_graph
from isaac_sim.graphs.control_graph import control_graph_spec
from isaac_sim.graphs.odometry_graph import ideal_odometry_graph_spec
from isaac_sim.graphs.ros_contract import load_qos_profiles, load_topics
from isaac_sim.graphs.sensor_graph import core_sensor_graph_spec, lidar_graph_spec
from isaac_sim.graphs.spec import graph_pipeline_kind
from isaac_sim.graphs.tf_graph import structure_tf_graph_spec
from isaac_sim.src.bridge import ros_graph_builder as ros_graph_builder_module
from isaac_sim.src.bridge.ros_graph_builder import RosGraphBuilder
from isaac_sim.src.bridge.tf_ownership import (
    TfOwnershipError,
    expected_tf_owners,
    validate_tf_publishers,
)
from isaac_sim.src.config import load_project_config
from isaac_sim.src.yaml_utils import YamlConfigError


ROOT = Path(__file__).resolve().parents[2]
SPLIT_AXLE_V1 = "split_axle_v1"
SINGLE_FOUR_WHEEL_WRITE_V1 = "single_four_wheel_write_v1"


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
    for spec in (specs[0], specs[1], specs[3]):
        assert "omni.graph.action.OnPlaybackTick" not in dict(spec.nodes).values()
        assert "isaacsim.core.nodes.OnPhysicsStep" in dict(spec.nodes).values()
        assert graph_pipeline_kind(spec) == "on_demand"
    assert graph_pipeline_kind(specs[2]) == "execution"
    dt_values = [value for name, value in specs[0].values if name.endswith("inputs:dt")]
    assert dt_values == [pytest.approx(1.0 / 60.0)]
    sensor_nodes = dict(specs[1].nodes)
    sensor_values = dict(specs[1].values)
    lidar_values = dict(specs[2].values)
    assert sensor_nodes["ReadJointState"].endswith("IsaacReadJointState")
    assert "PublishJointState.inputs:targetPrim" not in sensor_values
    assert (
        lidar_values[
            "PointCloudPublisher.inputs:resetSimulationTimeOnStop"
        ]
        is True
    )
    assert (
        "ReadJointState.outputs:jointNames",
        "PublishJointState.inputs:jointNames",
    ) in specs[1].connections


def test_control_graph_uses_top_level_radius_and_effective_track_width():
    spec = control_graph_spec(_config())
    nodes = dict(spec.nodes)
    values = dict(spec.values)

    assert "AppendWheelCommands" not in nodes
    assert nodes["FrontController"].endswith("IsaacArticulationController")
    assert nodes["RearController"].endswith("IsaacArticulationController")
    assert values["DifferentialController.inputs:wheelRadius"] == 0.098
    assert values["DifferentialController.inputs:wheelDistance"] == 0.37559
    assert values["FrontController.inputs:jointNames"] == [
        "front_left_wheel_joint",
        "front_right_wheel_joint",
    ]
    assert values["RearController.inputs:jointNames"] == [
        "rear_left_wheel_joint",
        "rear_right_wheel_joint",
    ]


def test_control_graph_single_write_duplicates_command_for_ordered_four_wheels():
    spec = control_graph_spec(
        _config(),
        wheel_command_application=SINGLE_FOUR_WHEEL_WRITE_V1,
    )
    spec.validate()
    nodes = dict(spec.nodes)
    values = dict(spec.values)

    assert "FrontController" not in nodes
    assert "RearController" not in nodes
    assert nodes["AppendWheelCommands"] == "omni.graph.nodes.AppendArray"
    assert nodes["WheelController"].endswith("IsaacArticulationController")
    assert (
        "DifferentialController.outputs:velocityCommand",
        "AppendWheelCommands.inputs:input0",
    ) in spec.connections
    assert (
        "DifferentialController.outputs:velocityCommand",
        "AppendWheelCommands.inputs:input1",
    ) in spec.connections
    assert (
        "AppendWheelCommands.outputs:array",
        "WheelController.inputs:velocityCommand",
    ) in spec.connections
    assert values["WheelController.inputs:jointNames"] == [
        "front_left_wheel_joint",
        "front_right_wheel_joint",
        "rear_left_wheel_joint",
        "rear_right_wheel_joint",
    ]


def test_control_graph_rejects_unknown_wheel_command_application():
    with pytest.raises(ValueError, match="wheel command application"):
        control_graph_spec(
            _config(),
            wheel_command_application="last_writer_wins_v1",
        )


def test_ros_graph_builder_keeps_wheel_command_application_immutable_for_rebuild(
    monkeypatch,
):
    calls = []

    def fake_build_control_graph(config, wheel_command_application):
        calls.append((config, wheel_command_application))
        return "control-graph"

    monkeypatch.setattr(
        ros_graph_builder_module,
        "build_control_graph",
        fake_build_control_graph,
    )
    config = _config()
    builder = RosGraphBuilder(
        config,
        object(),
        wheel_command_application=SINGLE_FOUR_WHEEL_WRITE_V1,
    )

    assert builder.build_control() == "control-graph"
    assert calls == [(config, SINGLE_FOUR_WHEEL_WRITE_V1)]
    with pytest.raises(AttributeError):
        builder.wheel_command_application = SPLIT_AXLE_V1


def test_navigation_cli_selects_only_versioned_wheel_command_applications():
    parser = _parser()

    assert parser.parse_args([]).wheel_command_application == SPLIT_AXLE_V1
    assert (
        parser.parse_args(
            ["--wheel-command-application", SINGLE_FOUR_WHEEL_WRITE_V1]
        ).wheel_command_application
        == SINGLE_FOUR_WHEEL_WRITE_V1
    )
    with pytest.raises(SystemExit):
        parser.parse_args(
            ["--wheel-command-application", "last_writer_wins_v1"]
        )


def test_navigation_reset_executes_control_graph_rebuild_through_selected_builder():
    events = []

    class IdleBrake:
        def reset(self):
            events.append("idle_brake.reset")

    class GraphBuilder:
        wheel_command_application = SINGLE_FOUR_WHEEL_WRITE_V1

        def build_control(self):
            events.append(
                f"build:{self.wheel_command_application}"
            )
            return {"mode": self.wheel_command_application}

    references = {"control": {"mode": SPLIT_AXLE_V1}}

    rebuilt = _rebuild_control_graph(
        IdleBrake(),
        GraphBuilder(),
        references,
    )

    assert events == [
        "idle_brake.reset",
        f"build:{SINGLE_FOUR_WHEEL_WRITE_V1}",
    ]
    assert rebuilt == {"mode": SINGLE_FOUR_WHEEL_WRITE_V1}
    assert references["control"] is rebuilt


def test_control_graph_rejects_project_joint_targets_that_diverge_from_robot_yaml():
    config = _config()
    project_robot = replace(
        config.robot,
        wheel_joints=("project_fl", "project_fr", "project_rl", "project_rr"),
        front_wheel_joints=("project_fl", "project_fr"),
        rear_wheel_joints=("project_rl", "project_rr"),
    )

    with pytest.raises(YamlConfigError, match="ProjectConfig robot joint targets"):
        control_graph_spec(replace(config, robot=project_robot))


def test_static_sensor_frames_use_raw_tf_and_no_world_frame():
    spec = structure_tf_graph_spec(_config())
    spec.validate()
    node_types = dict(spec.nodes)
    values = dict(spec.values)
    assert node_types["ComputeWheelTF"].endswith("IsaacComputeTransformTree")
    assert node_types["WheelTF"].endswith("ROS2PublishTransformTree")
    for node, node_type in node_types.items():
        if node.startswith("StaticTF"):
            assert node_type.endswith("ROS2PublishRawTransformTree")
    frame_values = [
        value
        for attribute, value in spec.values
        if attribute.endswith(("parentFrameId", "childFrameId"))
    ]
    assert "world" not in frame_values
    assert "WheelTF.inputs:targetPrims" not in values
    assert (
        "ComputeWheelTF.outputs:parentFrames",
        "WheelTF.inputs:parentFrames",
    ) in spec.connections
    assert graph_pipeline_kind(spec) == "on_demand"


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
    assert topics["camera_front_image"] == "/camera/front/image_raw"
    assert topics["camera_front_info"] == "/camera/front/camera_info"
    assert topics["frames"]["camera_front_optical"] == (
        "camera_front_optical_frame"
    )
    assert '"reliability":"bestEffort"' in qos["sensor_data"]
    assert '"depth":2' in qos["camera_sensor_data"]
    assert '"durability":"transientLocal"' in qos["static_tf"]
