"""Front RGB-D Camera ROS 2 graph contracts."""

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
    depth_points_topic: str
    optical_frame: str
    qos_profile: str


def _qualified_topic(namespace: str, topic_name: str) -> str:
    return f"{namespace.rstrip('/')}/{topic_name.lstrip('/')}"


def validate_camera_ros_contract(
    config: ProjectConfig,
    camera_config: CameraConfig,
) -> CameraRosContract:
    """Cross-check Camera YAML against the centralized ROS Topic/QoS contract."""

    topics = load_topics(config.files.topics)
    qos_profiles = load_qos_profiles(config.files.qos)
    camera = camera_config.cameras[camera_config.primary_camera]
    image_topic = _qualified_topic(camera.node_namespace, camera.rgb.topic_name)
    camera_info_topic = _qualified_topic(
        camera.node_namespace, camera.camera_info.topic_name
    )
    depth_points_topic = _qualified_topic(
        camera.node_namespace, camera.depth_points.topic_name
    )
    if image_topic != topics["camera_front_image"]:
        raise ValueError(
            f"Camera Image topic mismatch: config={image_topic}, "
            f"contract={topics['camera_front_image']}"
        )
    if camera_info_topic != topics["camera_front_info"]:
        raise ValueError(
            f"CameraInfo topic mismatch: config={camera_info_topic}, "
            f"contract={topics['camera_front_info']}"
        )
    if depth_points_topic != topics["camera_front_depth_points"]:
        raise ValueError(
            f"Camera depth points topic mismatch: config={depth_points_topic}, "
            f"contract={topics['camera_front_depth_points']}"
        )
    if camera.link_frame != topics["frames"]["camera_front"]:
        raise ValueError("Camera mechanical frame does not match ROS frame contract")
    if camera.optical_frame != topics["frames"]["camera_front_optical"]:
        raise ValueError("Camera optical frame does not match ROS frame contract")
    if len({camera.rgb.qos_profile, camera.camera_info.qos_profile,
            camera.depth_points.qos_profile}) != 1:
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
        depth_points_topic=depth_points_topic,
        optical_frame=camera.optical_frame,
        qos_profile=encoded_qos,
    )


def camera_graph_spec(config: ProjectConfig, camera: CameraRuntime) -> GraphSpec:
    """Build RGB/CameraInfo and optional depth publishers from one Render Product."""

    camera_config = __import__(
        "isaac_sim.src.sensors.sensor_factory", fromlist=["_load_camera"]
    )._load_camera(config.files.camera)
    contract = validate_camera_ros_contract(config, camera_config)
    if camera.optical_frame != contract.optical_frame:
        raise ValueError(
            f"Camera runtime frame mismatch: {camera.optical_frame} != "
            f"{contract.optical_frame}"
        )
    if len({camera.rgb.qos_profile, camera.camera_info.qos_profile,
            camera.depth_points.qos_profile}) != 1:
        raise ValueError("Camera runtime streams must share one QoS profile")
    nodes = [
        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
        ("PublishRGB", "isaacsim.ros2.bridge.ROS2CameraHelper"),
        ("PublishCameraInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
    ]
    connections = [
        ("OnPlaybackTick.outputs:tick", "PublishRGB.inputs:execIn"),
        ("OnPlaybackTick.outputs:tick", "PublishCameraInfo.inputs:execIn"),
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


def build_camera_graphs(config: ProjectConfig, cameras: tuple[CameraRuntime, ...]):
    return tuple(materialize_graph(camera_graph_spec(config, camera)) for camera in cameras)


def destroy_camera_graphs(graph_paths: tuple[str, ...]) -> None:
    """Destroy helper nodes before their Render Products are released."""

    if not graph_paths:
        return
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return
    for path in graph_paths:
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid():
            stage.RemovePrim(path)
