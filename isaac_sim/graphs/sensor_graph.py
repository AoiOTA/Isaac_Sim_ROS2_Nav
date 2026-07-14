"""Clock, JointState, IMU, and RTX PointCloud ROS graphs."""

from __future__ import annotations

from isaac_sim.graphs.spec import GraphSpec, TargetPaths, materialize_graph
from isaac_sim.graphs.ros_contract import load_qos_profiles, load_topics
from isaac_sim.src.config import ProjectConfig


def core_sensor_graph_spec(config: ProjectConfig, imu_prim: str) -> GraphSpec:
    topics = load_topics(config.files.topics)
    qos = load_qos_profiles(config.files.qos)
    nodes = (
        ("OnPhysicsStep", "isaacsim.core.nodes.OnPhysicsStep"),
        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
        ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ("ReadJointState", "isaacsim.sensors.physics.IsaacReadJointState"),
        ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
        ("ReadIMU", "isaacsim.sensors.physics.IsaacReadIMU"),
        ("PublishIMU", "isaacsim.ros2.bridge.ROS2PublishImu"),
    )
    connections = (
        ("OnPhysicsStep.outputs:step", "PublishClock.inputs:execIn"),
        ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
        ("OnPhysicsStep.outputs:step", "ReadJointState.inputs:execIn"),
        ("ReadJointState.outputs:execOut", "PublishJointState.inputs:execIn"),
        ("ReadJointState.outputs:jointNames", "PublishJointState.inputs:jointNames"),
        ("ReadJointState.outputs:jointPositions", "PublishJointState.inputs:jointPositions"),
        ("ReadJointState.outputs:jointVelocities", "PublishJointState.inputs:jointVelocities"),
        ("ReadJointState.outputs:jointEfforts", "PublishJointState.inputs:jointEfforts"),
        ("ReadJointState.outputs:jointDofTypes", "PublishJointState.inputs:jointDofTypes"),
        (
            "ReadJointState.outputs:stageMetersPerUnit",
            "PublishJointState.inputs:stageMetersPerUnit",
        ),
        ("ReadJointState.outputs:sensorTime", "PublishJointState.inputs:sensorTime"),
        ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
        ("OnPhysicsStep.outputs:step", "ReadIMU.inputs:execIn"),
        ("OnPhysicsStep.outputs:step", "PublishIMU.inputs:execIn"),
        ("ReadIMU.outputs:linAcc", "PublishIMU.inputs:linearAcceleration"),
        ("ReadIMU.outputs:angVel", "PublishIMU.inputs:angularVelocity"),
        ("ReadIMU.outputs:orientation", "PublishIMU.inputs:orientation"),
        ("ReadIMU.outputs:sensorTime", "PublishIMU.inputs:timeStamp"),
    )
    values = (
        ("PublishClock.inputs:topicName", topics["clock"]),
        ("PublishClock.inputs:queueSize", 1),
        ("PublishClock.inputs:qosProfile", qos["clock"]),
        ("ReadJointState.inputs:prim", TargetPaths((config.robot.articulation_root,))),
        ("PublishJointState.inputs:topicName", topics["joint_states"]),
        ("PublishJointState.inputs:nodeNamespace", config.ros2.namespace),
        ("PublishJointState.inputs:queueSize", 10),
        ("PublishJointState.inputs:qosProfile", qos["state"]),
        ("ReadIMU.inputs:imuPrim", TargetPaths((imu_prim,))),
        ("ReadIMU.inputs:readGravity", True),
        ("ReadIMU.inputs:useLatestData", False),
        ("PublishIMU.inputs:frameId", topics["frames"]["imu"]),
        ("PublishIMU.inputs:topicName", topics["imu"]),
        ("PublishIMU.inputs:nodeNamespace", config.ros2.namespace),
        ("PublishIMU.inputs:queueSize", 5),
        ("PublishIMU.inputs:qosProfile", qos["sensor_data"]),
    )
    return GraphSpec("/World/Graphs/Sensors", nodes, connections, values)


def lidar_graph_spec(config: ProjectConfig, render_product_path: str) -> GraphSpec:
    topics = load_topics(config.files.topics)
    qos = load_qos_profiles(config.files.qos)
    nodes = (
        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
        ("PointCloudConfig", "isaacsim.ros2.bridge.ROS2RtxLidarPointCloudConfig"),
        ("PointCloudPublisher", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
    )
    connections = (
        ("OnPlaybackTick.outputs:tick", "PointCloudPublisher.inputs:execIn"),
        ("PointCloudConfig.outputs:selectedMetadata", "PointCloudPublisher.inputs:selectedMetadata"),
    )
    values = (
        # Navigation consumes XYZ only. Per-point intensity/timestamp metadata
        # nearly doubles the local DDS payload and is not used by the
        # PointCloud2-to-LaserScan chain; the PointCloud2 header still carries
        # the simulation timestamp.
        ("PointCloudConfig.inputs:outputIntensity", False),
        ("PointCloudConfig.inputs:outputTimestamp", False),
        ("PointCloudPublisher.inputs:renderProductPath", render_product_path),
        ("PointCloudPublisher.inputs:type", "point_cloud"),
        ("PointCloudPublisher.inputs:topicName", topics["pointcloud"]),
        ("PointCloudPublisher.inputs:frameId", topics["frames"]["lidar"]),
        ("PointCloudPublisher.inputs:nodeNamespace", config.ros2.namespace),
        ("PointCloudPublisher.inputs:queueSize", 5),
        ("PointCloudPublisher.inputs:qosProfile", qos["sensor_data"]),
        ("PointCloudPublisher.inputs:useSystemTime", False),
        ("PointCloudPublisher.inputs:resetSimulationTimeOnStop", False),
        # Isaac Sim 6.0.1 ships the deprecated fullScan input with a True
        # default and warns at runtime even when projects never author it.
        # False is the vendor implementation's explicit compatibility path;
        # accumulateOutputs on the RTX sensor still owns complete scans.
        ("PointCloudPublisher.inputs:fullScan", False),
    )
    return GraphSpec("/World/Graphs/Lidar", nodes, connections, values)


def build_sensor_graphs(config: ProjectConfig, imu_prim: str, render_product_path: str):
    return (
        materialize_graph(core_sensor_graph_spec(config, imu_prim)),
        materialize_graph(lidar_graph_spec(config, render_product_path)),
    )
