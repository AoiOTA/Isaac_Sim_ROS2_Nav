"""`/cmd_vel` to four wheel-joint velocity targets."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from isaac_sim.graphs.spec import GraphSpec, TargetPaths, materialize_graph
from isaac_sim.graphs.ros_contract import load_qos_profiles, load_topics
from isaac_sim.src.config import ProjectConfig
from isaac_sim.src.yaml_utils import load_mapping, reject_unknown, require_keys, require_number


def load_controller_config(path: str | Path) -> dict[str, float]:
    data = load_mapping(path)
    reject_unknown(
        data,
        {
            "schema_version",
            "name",
            "wheel_radius",
            "wheel_width",
            "geometric_track_width",
            "wheelbase",
            "base_mass",
            "wheel_mass",
            "nominal_total_mass",
            "physics",
            "wheel_joints",
            "controller",
            "frames",
            "footprint",
            "static_transforms",
        },
        context="robot config",
    )
    require_keys(data, {"schema_version", "wheel_joints", "controller"}, context="robot config")
    if data["schema_version"] != 1 or not isinstance(data["controller"], dict):
        raise ValueError("invalid robot/controller config")
    controller = data["controller"]
    fields = {
        "wheel_radius",
        "wheel_distance",
        "max_linear_speed",
        "max_angular_speed",
        "max_wheel_speed",
        "max_acceleration",
        "max_deceleration",
        "max_angular_acceleration",
    }
    reject_unknown(controller, fields, context="robot.controller")
    require_keys(controller, fields, context="robot.controller")
    return {key: require_number(controller[key], context=f"robot.controller.{key}", positive=True) for key in fields}


def control_graph_spec(config: ProjectConfig) -> GraphSpec:
    controller = load_controller_config(config.files.robot)
    topics = load_topics(config.files.topics)
    qos = load_qos_profiles(config.files.qos)
    nodes = (
        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
        ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
        ("BreakLinear", "omni.graph.nodes.BreakVector3"),
        ("BreakAngular", "omni.graph.nodes.BreakVector3"),
        ("DifferentialController", "isaacsim.robot.wheeled_robots.DifferentialController"),
        ("FrontController", "isaacsim.core.nodes.IsaacArticulationController"),
        ("RearController", "isaacsim.core.nodes.IsaacArticulationController"),
    )
    connections = (
        ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
        ("SubscribeTwist.outputs:execOut", "DifferentialController.inputs:execIn"),
        ("SubscribeTwist.outputs:linearVelocity", "BreakLinear.inputs:tuple"),
        ("SubscribeTwist.outputs:angularVelocity", "BreakAngular.inputs:tuple"),
        ("BreakLinear.outputs:x", "DifferentialController.inputs:linearVelocity"),
        ("BreakAngular.outputs:z", "DifferentialController.inputs:angularVelocity"),
        ("OnPlaybackTick.outputs:tick", "FrontController.inputs:execIn"),
        ("OnPlaybackTick.outputs:tick", "RearController.inputs:execIn"),
        ("DifferentialController.outputs:velocityCommand", "FrontController.inputs:velocityCommand"),
        ("DifferentialController.outputs:velocityCommand", "RearController.inputs:velocityCommand"),
    )
    values: list[tuple[str, Any]] = [
        ("SubscribeTwist.inputs:topicName", topics["cmd_vel"]),
        ("SubscribeTwist.inputs:nodeNamespace", config.ros2.namespace),
        ("SubscribeTwist.inputs:queueSize", 1),
        ("SubscribeTwist.inputs:qosProfile", qos["command"]),
        ("DifferentialController.inputs:wheelRadius", controller["wheel_radius"]),
        ("DifferentialController.inputs:wheelDistance", controller["wheel_distance"]),
        ("DifferentialController.inputs:dt", 1.0 / config.simulation.physics_hz),
        ("DifferentialController.inputs:maxLinearSpeed", controller["max_linear_speed"]),
        ("DifferentialController.inputs:maxAngularSpeed", controller["max_angular_speed"]),
        ("DifferentialController.inputs:maxWheelSpeed", controller["max_wheel_speed"]),
        ("DifferentialController.inputs:maxAcceleration", controller["max_acceleration"]),
        ("DifferentialController.inputs:maxDeceleration", controller["max_deceleration"]),
        ("DifferentialController.inputs:maxAngularAcceleration", controller["max_angular_acceleration"]),
        ("FrontController.inputs:targetPrim", TargetPaths((config.robot.articulation_root,))),
        ("FrontController.inputs:jointNames", list(config.robot.front_wheel_joints)),
        ("RearController.inputs:targetPrim", TargetPaths((config.robot.articulation_root,))),
        ("RearController.inputs:jointNames", list(config.robot.rear_wheel_joints)),
    ]
    return GraphSpec("/World/Graphs/Control", nodes, connections, tuple(values))


def build_control_graph(config: ProjectConfig):
    return materialize_graph(control_graph_spec(config))
