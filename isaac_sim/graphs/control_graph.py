"""`/cmd_vel` to four wheel-joint velocity targets."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from isaac_sim.graphs.spec import GraphSpec, TargetPaths, materialize_graph
from isaac_sim.graphs.ros_contract import load_qos_profiles, load_topics
from isaac_sim.src.config import ProjectConfig
from isaac_sim.src.robot.kinematics_config import (
    RobotConfigContract,
    load_robot_config_contract,
)
from isaac_sim.src.yaml_utils import YamlConfigError


SPLIT_AXLE_V1 = "split_axle_v1"
SINGLE_FOUR_WHEEL_WRITE_V1 = "single_four_wheel_write_v1"
WHEEL_COMMAND_APPLICATIONS = (
    SPLIT_AXLE_V1,
    SINGLE_FOUR_WHEEL_WRITE_V1,
)


def require_wheel_command_application(value: str) -> str:
    if value not in WHEEL_COMMAND_APPLICATIONS:
        raise ValueError(
            "wheel command application must be one of "
            f"{WHEEL_COMMAND_APPLICATIONS!r}, got {value!r}"
        )
    return value


def _controller_values(contract: RobotConfigContract) -> dict[str, float]:
    kinematics = contract.kinematics
    controller = contract.controller
    return {
        "wheel_radius": kinematics.wheel_radius,
        "effective_track_width": kinematics.effective_track_width,
        "max_linear_speed": controller.max_linear_speed,
        "max_angular_speed": controller.max_angular_speed,
        "max_wheel_speed": controller.max_wheel_speed,
        "max_acceleration": controller.max_acceleration,
        "max_deceleration": controller.max_deceleration,
        "max_angular_acceleration": controller.max_angular_acceleration,
    }


def load_controller_config(path: str | Path) -> dict[str, float]:
    return _controller_values(load_robot_config_contract(path))


def control_graph_spec(
    config: ProjectConfig,
    wheel_command_application: str = SPLIT_AXLE_V1,
) -> GraphSpec:
    wheel_command_application = require_wheel_command_application(
        wheel_command_application
    )
    contract = load_robot_config_contract(config.files.robot)
    joints = contract.wheel_joints
    project_targets = (
        tuple(config.robot.wheel_joints),
        tuple(config.robot.front_wheel_joints),
        tuple(config.robot.rear_wheel_joints),
    )
    contract_targets = (joints.ordered, joints.front, joints.rear)
    if project_targets != contract_targets:
        raise YamlConfigError(
            "ProjectConfig robot joint targets must exactly match "
            "robot YAML wheel_joints: "
            f"project={project_targets!r}, robot_yaml={contract_targets!r}"
        )
    controller = _controller_values(contract)
    topics = load_topics(config.files.topics)
    qos = load_qos_profiles(config.files.qos)
    nodes = [
        ("OnPhysicsStep", "isaacsim.core.nodes.OnPhysicsStep"),
        ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
        ("BreakLinear", "omni.graph.nodes.BreakVector3"),
        ("BreakAngular", "omni.graph.nodes.BreakVector3"),
        ("DifferentialController", "isaacsim.robot.wheeled_robots.DifferentialController"),
    ]
    connections = [
        ("OnPhysicsStep.outputs:step", "SubscribeTwist.inputs:execIn"),
        ("SubscribeTwist.outputs:execOut", "DifferentialController.inputs:execIn"),
        ("SubscribeTwist.outputs:linearVelocity", "BreakLinear.inputs:tuple"),
        ("SubscribeTwist.outputs:angularVelocity", "BreakAngular.inputs:tuple"),
        ("BreakLinear.outputs:x", "DifferentialController.inputs:linearVelocity"),
        ("BreakAngular.outputs:z", "DifferentialController.inputs:angularVelocity"),
    ]
    values: list[tuple[str, Any]] = [
        ("SubscribeTwist.inputs:topicName", topics["cmd_vel"]),
        ("SubscribeTwist.inputs:nodeNamespace", config.ros2.namespace),
        ("SubscribeTwist.inputs:queueSize", 1),
        ("SubscribeTwist.inputs:qosProfile", qos["command"]),
        ("DifferentialController.inputs:wheelRadius", controller["wheel_radius"]),
        (
            "DifferentialController.inputs:wheelDistance",
            controller["effective_track_width"],
        ),
        ("DifferentialController.inputs:dt", 1.0 / config.simulation.physics_hz),
        ("DifferentialController.inputs:maxLinearSpeed", controller["max_linear_speed"]),
        ("DifferentialController.inputs:maxAngularSpeed", controller["max_angular_speed"]),
        ("DifferentialController.inputs:maxWheelSpeed", controller["max_wheel_speed"]),
        ("DifferentialController.inputs:maxAcceleration", controller["max_acceleration"]),
        ("DifferentialController.inputs:maxDeceleration", controller["max_deceleration"]),
        ("DifferentialController.inputs:maxAngularAcceleration", controller["max_angular_acceleration"]),
    ]
    if wheel_command_application == SPLIT_AXLE_V1:
        nodes.extend(
            (
                (
                    "FrontController",
                    "isaacsim.core.nodes.IsaacArticulationController",
                ),
                (
                    "RearController",
                    "isaacsim.core.nodes.IsaacArticulationController",
                ),
            )
        )
        connections.extend(
            (
                ("OnPhysicsStep.outputs:step", "FrontController.inputs:execIn"),
                ("OnPhysicsStep.outputs:step", "RearController.inputs:execIn"),
                (
                    "DifferentialController.outputs:velocityCommand",
                    "FrontController.inputs:velocityCommand",
                ),
                (
                    "DifferentialController.outputs:velocityCommand",
                    "RearController.inputs:velocityCommand",
                ),
            )
        )
        values.extend(
            (
                (
                    "FrontController.inputs:targetPrim",
                    TargetPaths((config.robot.articulation_root,)),
                ),
                ("FrontController.inputs:jointNames", list(joints.front)),
                (
                    "RearController.inputs:targetPrim",
                    TargetPaths((config.robot.articulation_root,)),
                ),
                ("RearController.inputs:jointNames", list(joints.rear)),
            )
        )
    else:
        nodes.extend(
            (
                ("AppendWheelCommands", "omni.graph.nodes.AppendArray"),
                (
                    "WheelController",
                    "isaacsim.core.nodes.IsaacArticulationController",
                ),
            )
        )
        connections.extend(
            (
                (
                    "DifferentialController.outputs:velocityCommand",
                    "AppendWheelCommands.inputs:input0",
                ),
                (
                    "DifferentialController.outputs:velocityCommand",
                    "AppendWheelCommands.inputs:input1",
                ),
                (
                    "OnPhysicsStep.outputs:step",
                    "WheelController.inputs:execIn",
                ),
                (
                    "AppendWheelCommands.outputs:array",
                    "WheelController.inputs:velocityCommand",
                ),
            )
        )
        values.extend(
            (
                (
                    "WheelController.inputs:targetPrim",
                    TargetPaths((config.robot.articulation_root,)),
                ),
                ("WheelController.inputs:jointNames", list(joints.ordered)),
            )
        )
    return GraphSpec(
        "/World/Graphs/Control",
        tuple(nodes),
        tuple(connections),
        tuple(values),
    )


def build_control_graph(
    config: ProjectConfig,
    wheel_command_application: str = SPLIT_AXLE_V1,
):
    return materialize_graph(
        control_graph_spec(config, wheel_command_application)
    )
