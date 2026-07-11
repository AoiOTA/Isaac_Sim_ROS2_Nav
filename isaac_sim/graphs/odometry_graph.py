"""Ideal Isaac odometry graph; intentionally absent in realistic mode."""

from __future__ import annotations

from isaac_sim.graphs.spec import GraphSpec, TargetPaths, materialize_graph
from isaac_sim.graphs.ros_contract import load_qos_profiles, load_topics
from isaac_sim.src.config import ProjectConfig


def ideal_odometry_graph_spec(config: ProjectConfig) -> GraphSpec:
    if config.simulation.odometry_mode != "ideal":
        raise ValueError("Isaac ideal odometry graph is forbidden in realistic mode")
    topics = load_topics(config.files.topics)
    qos = load_qos_profiles(config.files.qos)
    nodes = (
        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
        ("ComputeOdometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
        ("PublishOdometry", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
        ("PublishOdomTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
    )
    connections = (
        ("OnPlaybackTick.outputs:tick", "ComputeOdometry.inputs:execIn"),
        ("OnPlaybackTick.outputs:tick", "PublishOdometry.inputs:execIn"),
        ("OnPlaybackTick.outputs:tick", "PublishOdomTF.inputs:execIn"),
        ("ReadSimTime.outputs:simulationTime", "PublishOdometry.inputs:timeStamp"),
        ("ReadSimTime.outputs:simulationTime", "PublishOdomTF.inputs:timeStamp"),
        ("ComputeOdometry.outputs:position", "PublishOdometry.inputs:position"),
        ("ComputeOdometry.outputs:orientation", "PublishOdometry.inputs:orientation"),
        ("ComputeOdometry.outputs:linearVelocity", "PublishOdometry.inputs:linearVelocity"),
        ("ComputeOdometry.outputs:angularVelocity", "PublishOdometry.inputs:angularVelocity"),
        ("ComputeOdometry.outputs:position", "PublishOdomTF.inputs:translation"),
        ("ComputeOdometry.outputs:orientation", "PublishOdomTF.inputs:rotation"),
    )
    values = (
        ("ComputeOdometry.inputs:chassisPrim", TargetPaths((config.robot.base_link_prim,))),
        ("PublishOdometry.inputs:topicName", topics["odom"]),
        ("PublishOdometry.inputs:odomFrameId", topics["frames"]["odom"]),
        ("PublishOdometry.inputs:chassisFrameId", topics["frames"]["base"]),
        ("PublishOdometry.inputs:nodeNamespace", config.ros2.namespace),
        ("PublishOdometry.inputs:queueSize", 10),
        ("PublishOdometry.inputs:qosProfile", qos["state"]),
        ("PublishOdomTF.inputs:parentFrameId", topics["frames"]["odom"]),
        ("PublishOdomTF.inputs:childFrameId", topics["frames"]["base"]),
        ("PublishOdomTF.inputs:topicName", topics["tf"]),
        ("PublishOdomTF.inputs:nodeNamespace", config.ros2.namespace),
        ("PublishOdomTF.inputs:qosProfile", qos["tf"]),
    )
    return GraphSpec("/World/Graphs/IdealOdometry", nodes, connections, values)


def build_odometry_graph(config: ProjectConfig):
    if config.simulation.odometry_mode == "realistic":
        return None
    return materialize_graph(ideal_odometry_graph_spec(config))
