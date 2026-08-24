"""Front RGB-D and opt-in stereo Camera ROS 2 graph contracts."""

from __future__ import annotations

from dataclasses import dataclass
import json

from isaac_sim.graphs.ros_contract import load_qos_profiles, load_topics
from isaac_sim.graphs.spec import GraphSpec, materialize_graph
from isaac_sim.src.config import ProjectConfig
from isaac_sim.src.sensors.sensor_factory import CameraConfig, CameraRuntime


@dataclass(frozen=True)
class CameraRosContract:
    image_topic: str
    camera_info_topic: str
    depth_topic: str | None
    depth_points_topic: str | None
    optical_frame: str
    qos_profile: str


def _qualified_topic(namespace: str, topic_name: str) -> str:
    return f"{namespace.rstrip('/')}/{topic_name.lstrip('/')}"


def validate_camera_ros_contract(
    config: ProjectConfig,
    camera_config: CameraConfig,
    camera_name: str | None = None,
) -> CameraRosContract:
    """Cross-check Camera YAML against the centralized ROS Topic/QoS contract."""

    topics = load_topics(config.files.topics)
    qos_profiles = load_qos_profiles(config.files.qos)
    name = camera_name or camera_config.primary_camera
    camera = camera_config.cameras[name]
    image_topic = _qualified_topic(camera.node_namespace, camera.rgb.topic_name)
    camera_info_topic = _qualified_topic(
        camera.node_namespace, camera.camera_info.topic_name
    )
    depth_topic = (
        _qualified_topic(camera.node_namespace, camera.depth.topic_name)
        if camera.depth.enabled else None
    )
    depth_points_topic = (
        _qualified_topic(camera.node_namespace, camera.depth_points.topic_name)
        if camera.depth_points.enabled else None
    )
    topic_keys = {
        "front": (
            "camera_front_image", "camera_front_info", "camera_front_depth",
            "camera_front_depth_points",
        ),
        "left": (
            "camera_left_image", "camera_left_info", "camera_left_depth", None,
        ),
        "right": (
            "camera_right_image", "camera_right_info", None, None,
        ),
    }[name]
    if image_topic != topics[topic_keys[0]]:
        raise ValueError(
            f"Camera Image topic mismatch: config={image_topic}, "
            f"contract={topics[topic_keys[0]]}"
        )
    if camera_info_topic != topics[topic_keys[1]]:
        raise ValueError(
            f"CameraInfo topic mismatch: config={camera_info_topic}, "
            f"contract={topics[topic_keys[1]]}"
        )
    if topic_keys[2] is not None and depth_topic != topics[topic_keys[2]]:
        raise ValueError(
            f"Camera depth topic mismatch: config={depth_topic}, "
            f"contract={topics[topic_keys[2]]}"
        )
    if topic_keys[3] is not None and depth_points_topic != topics[topic_keys[3]]:
        raise ValueError(
            f"Camera depth points topic mismatch: config={depth_points_topic}, "
            f"contract={topics[topic_keys[3]]}"
        )
    if camera.link_frame != topics["frames"][f"camera_{name}"]:
        raise ValueError("Camera mechanical frame does not match ROS frame contract")
    if camera.optical_frame != topics["frames"][f"camera_{name}_optical"]:
        raise ValueError("Camera optical frame does not match ROS frame contract")
    if len({camera.rgb.qos_profile, camera.camera_info.qos_profile,
            camera.depth.qos_profile, camera.depth_points.qos_profile}) != 1:
        raise ValueError("Camera stream QoS profiles must match")
    try:
        encoded_qos = qos_profiles[camera.rgb.qos_profile]
    except KeyError as exc:
        raise ValueError(
            f"Camera QoS profile is missing: {camera.rgb.qos_profile}"
        ) from exc
    decoded_qos = json.loads(encoded_qos)
    expected = {
        "history": "keepLast",
        "depth": 2,
        "reliability": "bestEffort",
        "durability": "volatile",
    }
    if any(decoded_qos.get(key) != value for key, value in expected.items()):
        raise ValueError(
            "Camera QoS must be Keep Last / depth 2 / Best Effort / Volatile"
        )
    return CameraRosContract(
        image_topic=image_topic,
        camera_info_topic=camera_info_topic,
        depth_topic=depth_topic,
        depth_points_topic=depth_points_topic,
        optical_frame=camera.optical_frame,
        qos_profile=encoded_qos,
    )


def camera_graph_spec(config: ProjectConfig, camera: CameraRuntime) -> GraphSpec:
    """Build RGB/CameraInfo and optional depth publishers from one Render Product."""

    camera_config = __import__(
        "isaac_sim.src.sensors.sensor_factory", fromlist=["_load_camera"]
    )._load_camera(config.files.camera)
    contract = validate_camera_ros_contract(config, camera_config, camera.name)
    if camera.optical_frame != contract.optical_frame:
        raise ValueError(
            f"Camera runtime frame mismatch: {camera.optical_frame} != "
            f"{contract.optical_frame}"
        )
    if len({camera.rgb.qos_profile, camera.camera_info.qos_profile,
            camera.depth.qos_profile, camera.depth_points.qos_profile}) != 1:
        raise ValueError("Camera runtime streams must share one QoS profile")
    nodes = [
        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
        ("PublishRGB", "isaacsim.ros2.bridge.ROS2CameraHelper"),
        ("PublishCameraInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
        ("PublishDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
    ]
    connections = [
        ("OnPlaybackTick.outputs:tick", "PublishRGB.inputs:execIn"),
        ("OnPlaybackTick.outputs:tick", "PublishCameraInfo.inputs:execIn"),
        ("OnPlaybackTick.outputs:tick", "PublishDepth.inputs:execIn"),
    ]
    values = [
        ("PublishRGB.inputs:enabled", True),
        ("PublishRGB.inputs:renderProductPath", camera.render_product_path),
        ("PublishRGB.inputs:type", "rgb"),
        ("PublishRGB.inputs:frameId", camera.optical_frame),
        ("PublishRGB.inputs:nodeNamespace", camera.node_namespace),
        ("PublishRGB.inputs:topicName", camera.rgb.topic_name),
        ("PublishRGB.inputs:queueSize", camera.rgb.queue_size),
        ("PublishRGB.inputs:qosProfile", contract.qos_profile),
        ("PublishRGB.inputs:useSystemTime", False),
        ("PublishRGB.inputs:resetSimulationTimeOnStop", False),
        ("PublishCameraInfo.inputs:enabled", True),
        (
            "PublishCameraInfo.inputs:renderProductPath",
            camera.render_product_path,
        ),
        ("PublishCameraInfo.inputs:frameId", camera.optical_frame),
        ("PublishCameraInfo.inputs:nodeNamespace", camera.node_namespace),
        ("PublishCameraInfo.inputs:topicName", camera.camera_info.topic_name),
        ("PublishCameraInfo.inputs:queueSize", camera.camera_info.queue_size),
        ("PublishCameraInfo.inputs:qosProfile", contract.qos_profile),
        ("PublishCameraInfo.inputs:useSystemTime", False),
        ("PublishCameraInfo.inputs:resetSimulationTimeOnStop", False),
        ("PublishDepth.inputs:enabled", True),
        ("PublishDepth.inputs:renderProductPath", camera.render_product_path),
        ("PublishDepth.inputs:type", "depth"),
        ("PublishDepth.inputs:frameId", camera.optical_frame),
        ("PublishDepth.inputs:nodeNamespace", camera.node_namespace),
        ("PublishDepth.inputs:topicName", camera.depth.topic_name),
        ("PublishDepth.inputs:queueSize", camera.depth.queue_size),
        ("PublishDepth.inputs:qosProfile", contract.qos_profile),
        ("PublishDepth.inputs:useSystemTime", False),
        ("PublishDepth.inputs:resetSimulationTimeOnStop", False),
    ]
    if camera.depth_points_enabled:
        nodes.append(("PublishDepthPoints", "isaacsim.ros2.bridge.ROS2CameraHelper"))
        connections.append(("OnPlaybackTick.outputs:tick", "PublishDepthPoints.inputs:execIn"))
        values.extend((
            ("PublishDepthPoints.inputs:enabled", True),
            ("PublishDepthPoints.inputs:renderProductPath", camera.render_product_path),
            ("PublishDepthPoints.inputs:type", "depth_pcl"),
            ("PublishDepthPoints.inputs:frameId", camera.optical_frame),
            ("PublishDepthPoints.inputs:nodeNamespace", camera.node_namespace),
            ("PublishDepthPoints.inputs:topicName", camera.depth_points.topic_name),
            ("PublishDepthPoints.inputs:queueSize", camera.depth_points.queue_size),
            ("PublishDepthPoints.inputs:qosProfile", contract.qos_profile),
            ("PublishDepthPoints.inputs:useSystemTime", False),
            ("PublishDepthPoints.inputs:resetSimulationTimeOnStop", False),
        ))
    return GraphSpec(camera.graph_path, tuple(nodes), tuple(connections), tuple(values))


def stereo_camera_graph_spec(
    config: ProjectConfig,
    cameras: tuple[CameraRuntime, ...],
) -> GraphSpec:
    """Publish one stereo pair and left aligned depth from one shared tick."""

    if tuple(camera.name for camera in cameras) != ("left", "right"):
        raise ValueError("stereo Camera runtime order must be left/right")
    left, right = cameras
    if any(camera.profile_name != "stereo_vio" for camera in cameras):
        raise ValueError("stereo Camera graph requires the stereo_vio profile")
    if left.graph_path != right.graph_path:
        raise ValueError("stereo Camera runtimes must share one graph path")
    if left.render_product_path == right.render_product_path:
        raise ValueError("stereo Cameras require independent Render Products")

    camera_config = __import__(
        "isaac_sim.src.sensors.sensor_factory", fromlist=["_load_camera"]
    )._load_camera(config.files.camera)
    left_contract = validate_camera_ros_contract(config, camera_config, "left")
    right_contract = validate_camera_ros_contract(config, camera_config, "right")
    if not left.depth.enabled or right.depth.enabled:
        raise ValueError("stereo Camera depth must be left-only")
    if left.depth_points_enabled or right.depth_points_enabled:
        raise ValueError("stereo Camera profile must not publish depth points")
    if len({left_contract.qos_profile, right_contract.qos_profile}) != 1:
        raise ValueError("stereo Camera streams must share one QoS profile")

    nodes = (
        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
        ("PublishLeftRGB", "isaacsim.ros2.bridge.ROS2CameraHelper"),
        ("PublishRightRGB", "isaacsim.ros2.bridge.ROS2CameraHelper"),
        ("PublishLeftDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
        (
            "PublishStereoCameraInfo",
            "isaacsim.ros2.bridge.ROS2CameraInfoHelper",
        ),
    )
    connections = tuple(
        ("OnPlaybackTick.outputs:tick", f"{node}.inputs:execIn")
        for node in (
            "PublishLeftRGB", "PublishRightRGB", "PublishLeftDepth",
            "PublishStereoCameraInfo",
        )
    )
    values = (
        ("PublishLeftRGB.inputs:enabled", True),
        ("PublishLeftRGB.inputs:renderProductPath", left.render_product_path),
        ("PublishLeftRGB.inputs:type", "rgb"),
        ("PublishLeftRGB.inputs:frameId", left.optical_frame),
        ("PublishLeftRGB.inputs:nodeNamespace", left.node_namespace),
        ("PublishLeftRGB.inputs:topicName", left.rgb.topic_name),
        ("PublishLeftRGB.inputs:queueSize", left.rgb.queue_size),
        ("PublishLeftRGB.inputs:qosProfile", left_contract.qos_profile),
        ("PublishLeftRGB.inputs:useSystemTime", False),
        ("PublishLeftRGB.inputs:resetSimulationTimeOnStop", False),
        ("PublishRightRGB.inputs:enabled", True),
        ("PublishRightRGB.inputs:renderProductPath", right.render_product_path),
        ("PublishRightRGB.inputs:type", "rgb"),
        ("PublishRightRGB.inputs:frameId", right.optical_frame),
        ("PublishRightRGB.inputs:nodeNamespace", right.node_namespace),
        ("PublishRightRGB.inputs:topicName", right.rgb.topic_name),
        ("PublishRightRGB.inputs:queueSize", right.rgb.queue_size),
        ("PublishRightRGB.inputs:qosProfile", right_contract.qos_profile),
        ("PublishRightRGB.inputs:useSystemTime", False),
        ("PublishRightRGB.inputs:resetSimulationTimeOnStop", False),
        ("PublishLeftDepth.inputs:enabled", True),
        ("PublishLeftDepth.inputs:renderProductPath", left.render_product_path),
        ("PublishLeftDepth.inputs:type", "depth"),
        ("PublishLeftDepth.inputs:frameId", left.optical_frame),
        ("PublishLeftDepth.inputs:nodeNamespace", left.node_namespace),
        ("PublishLeftDepth.inputs:topicName", left.depth.topic_name),
        ("PublishLeftDepth.inputs:queueSize", left.depth.queue_size),
        ("PublishLeftDepth.inputs:qosProfile", left_contract.qos_profile),
        ("PublishLeftDepth.inputs:useSystemTime", False),
        ("PublishLeftDepth.inputs:resetSimulationTimeOnStop", False),
        ("PublishStereoCameraInfo.inputs:enabled", True),
        (
            "PublishStereoCameraInfo.inputs:renderProductPath",
            left.render_product_path,
        ),
        (
            "PublishStereoCameraInfo.inputs:renderProductPathRight",
            right.render_product_path,
        ),
        ("PublishStereoCameraInfo.inputs:frameId", left.optical_frame),
        ("PublishStereoCameraInfo.inputs:frameIdRight", right.optical_frame),
        ("PublishStereoCameraInfo.inputs:topicName", left_contract.camera_info_topic),
        (
            "PublishStereoCameraInfo.inputs:topicNameRight",
            right_contract.camera_info_topic,
        ),
        ("PublishStereoCameraInfo.inputs:queueSize", left.camera_info.queue_size),
        ("PublishStereoCameraInfo.inputs:qosProfile", left_contract.qos_profile),
        ("PublishStereoCameraInfo.inputs:useSystemTime", False),
        ("PublishStereoCameraInfo.inputs:resetSimulationTimeOnStop", False),
    )
    return GraphSpec(left.graph_path, nodes, connections, values)


def build_camera_graphs(config: ProjectConfig, cameras: tuple[CameraRuntime, ...]):
    if not cameras:
        return ()
    if cameras[0].profile_name == "stereo_vio":
        return (materialize_graph(stereo_camera_graph_spec(config, cameras)),)
    return tuple(materialize_graph(camera_graph_spec(config, camera)) for camera in cameras)


def destroy_camera_graphs(graph_paths: tuple[str, ...]) -> None:
    """Destroy helper nodes before their Render Products are released."""

    if not graph_paths:
        return
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return
    for path in dict.fromkeys(graph_paths):
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid():
            stage.RemovePrim(path)
