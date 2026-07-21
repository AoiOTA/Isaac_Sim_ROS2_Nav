"""Structure-only TF graphs with no USD World frame publication."""

from __future__ import annotations

import math
from pathlib import Path

from isaac_sim.graphs.spec import GraphSpec, TargetPaths, materialize_graph
from isaac_sim.graphs.ros_contract import load_qos_profiles, load_topics
from isaac_sim.src.config import ProjectConfig
from isaac_sim.src.robot.spawn_pose_manager import load_spawn_poses
from isaac_sim.src.yaml_utils import load_mapping, require_vector


_STRUCTURE_EDGES = (
    ("base_link", "lidar_link"),
    ("base_link", "imu_link"),
    ("base_link", "camera_link"),
    ("camera_link", "camera_front_link"),
    ("camera_front_link", "camera_front_optical_frame"),
    ("camera_link", "camera_left_link"),
    ("camera_link", "camera_right_link"),
    ("camera_left_link", "camera_left_optical_frame"),
    ("camera_right_link", "camera_right_optical_frame"),
)


def load_static_transforms(path: str | Path):
    """Load the stable ROS structure tree from the selected robot profile."""
    transforms = load_mapping(path).get("static_transforms")
    if not isinstance(transforms, list) or len(transforms) != len(
            _STRUCTURE_EDGES):
        raise ValueError(
            "robot.static_transforms must contain exactly the documented "
            f"{len(_STRUCTURE_EDGES)} entries")
    result = []
    observed = []
    for index, transform in enumerate(transforms):
        if not isinstance(transform, dict) or set(transform) != {
                "parent", "child", "translation", "rotation_xyzw"}:
            raise ValueError(
                f"robot.static_transforms[{index}] has an invalid schema")
        edge = (transform["parent"], transform["child"])
        observed.append(edge)
        translation = require_vector(
            transform["translation"], 3,
            context=f"robot.static_transforms[{index}].translation")
        rotation = require_vector(
            transform["rotation_xyzw"], 4,
            context=f"robot.static_transforms[{index}].rotation_xyzw")
        norm = math.sqrt(sum(value * value for value in rotation))
        if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1e-6):
            raise ValueError(
                f"robot.static_transforms[{index}] quaternion must be normalized")
        node_name = "StaticTF" + str(index)
        result.append((node_name, *edge, list(translation), list(rotation)))
    if tuple(observed) != _STRUCTURE_EDGES:
        raise ValueError(
            "robot.static_transforms must preserve the documented TF tree; "
            f"expected={_STRUCTURE_EDGES}, got={tuple(observed)}")
    return tuple(result)


def rtx_world_transform(config: ProjectConfig):
    """Return the fixed odom -> USD-world data-frame transform.

    RTX PointCloud coordinates are absolute USD-world positions.  Isaac ideal
    odometry is zeroed at the selected spawn, so this inverse spawn transform
    attaches that fixed coordinate system below odom without adding a second
    parent to odom.
    """
    poses = load_spawn_poses(config.spawn.poses_file)
    pose = poses[config.spawn.selected].usd
    x, y, z = pose.position
    yaw = math.radians(pose.yaw_deg)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    translation = [
        -(cosine * x + sine * y),
        -(-sine * x + cosine * y),
        -z,
    ]
    half_inverse_yaw = -0.5 * yaw
    rotation = [
        0.0,
        0.0,
        math.sin(half_inverse_yaw),
        math.cos(half_inverse_yaw),
    ]
    return (
        "RtxWorldTF",
        "odom",
        "rtx_world",
        translation,
        rotation,
    )


def structure_tf_graph_spec(config: ProjectConfig) -> GraphSpec:
    topics = load_topics(config.files.topics)
    qos = load_qos_profiles(config.files.qos)
    base = config.robot.base_link_prim
    static_transforms = load_static_transforms(config.files.robot)
    published_transforms = static_transforms + (rtx_world_transform(config),)
    nodes = (
        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
        ("WheelTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
    ) + tuple(
        (node, "isaacsim.ros2.bridge.ROS2PublishRawTransformTree")
        for node, _, _, _, _ in published_transforms
    )
    connections: list[tuple[str, str]] = []
    for node in ("WheelTF",) + tuple(item[0] for item in published_transforms):
        connections.append(("OnPlaybackTick.outputs:tick", f"{node}.inputs:execIn"))
        connections.append(("ReadSimTime.outputs:simulationTime", f"{node}.inputs:timeStamp"))
    values: tuple[tuple[str, object], ...] = (
        ("WheelTF.inputs:parentPrim", TargetPaths((base,))),
        (
            "WheelTF.inputs:targetPrims",
            TargetPaths(
                tuple(
                    f"{config.robot.runtime_prim_path}/{name.replace('_joint', '_link')}"
                    for name in config.robot.wheel_joints
                )
            ),
        ),
        ("WheelTF.inputs:topicName", topics["tf"]),
        ("WheelTF.inputs:qosProfile", qos["tf"]),
        ("WheelTF.inputs:staticPublisher", False),
    )
    static_values: list[tuple[str, object]] = []
    for node, parent, child, translation, rotation in published_transforms:
        static_values.extend(
            (
                (f"{node}.inputs:parentFrameId", parent),
                (f"{node}.inputs:childFrameId", child),
                (f"{node}.inputs:translation", translation),
                (f"{node}.inputs:rotation", rotation),
                (f"{node}.inputs:topicName", topics["tf_static"]),
                (f"{node}.inputs:qosProfile", qos["static_tf"]),
                (f"{node}.inputs:staticPublisher", True),
            )
        )
    namespace_values = tuple(
        (f"{node}.inputs:nodeNamespace", config.ros2.namespace)
        for node in ("WheelTF",) + tuple(item[0] for item in published_transforms)
    )
    return GraphSpec(
        "/World/Graphs/StructureTF",
        nodes,
        tuple(connections),
        values + tuple(static_values) + namespace_values,
    )


def build_tf_graph(config: ProjectConfig):
    return materialize_graph(structure_tf_graph_spec(config))
