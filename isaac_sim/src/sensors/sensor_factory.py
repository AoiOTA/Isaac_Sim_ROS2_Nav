"""Isaac Sim 6.0.1 sensor authoring with no module-level Isaac imports."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import PurePosixPath
from typing import Any

from isaac_sim.src.config import ProjectConfig
from isaac_sim.src.yaml_utils import (
    load_mapping,
    reject_unknown,
    require_keys,
    require_number,
    require_vector,
)


class SensorConfigError(RuntimeError):
    pass


CAMERA_PROFILE_NAMES = (
    "off", "monitoring", "standard", "high_quality", "rgbd_navigation"
)
_CAMERA_PROFILE_CONTRACT = {
    "off": (False, 0, 0, 0.0, False),
    "monitoring": (True, 640, 360, 15.0, False),
    "standard": (True, 640, 480, 20.0, False),
    "high_quality": (True, 1280, 720, 30.0, False),
    "rgbd_navigation": (True, 320, 180, 10.0, True),
}


@dataclass(frozen=True)
class CameraProfile:
    name: str
    enabled: bool
    width: int
    height: int
    publish_rate_hz: float
    depth_points_enabled: bool


@dataclass(frozen=True)
class CameraStream:
    enabled: bool
    topic_name: str
    qos_profile: str
    queue_size: int
    encoding: str | None = None


@dataclass(frozen=True)
class CameraDefinition:
    name: str
    enabled: bool
    sensor_prim: str
    link_frame: str
    optical_frame: str
    node_namespace: str
    rgb: CameraStream
    camera_info: CameraStream
    depth: CameraStream
    depth_points: CameraStream
    clipping_range_m: tuple[float, float]
    projection: str
    focal_length_mm: float
    horizontal_aperture_mm: float
    vertical_aperture_mode: str
    focus_distance_m: float
    exposure_enabled: bool
    exposure_time_s: float
    exposure_responsivity: float
    exposure_f_stop: float
    rviz_enabled: bool
    rviz_transport: str
    rviz_reliability: str
    rviz_queue_size: int


@dataclass(frozen=True)
class CameraConfig:
    enabled: bool
    default_profile: str
    primary_camera: str
    profiles: dict[str, CameraProfile]
    cameras: dict[str, CameraDefinition]


@dataclass(frozen=True)
class CameraSelection:
    profile: CameraProfile
    camera: CameraDefinition | None


@dataclass
class CameraRuntime:
    name: str
    profile_name: str
    sensor: object
    camera_prim_path: str
    render_product: object
    render_product_path: str
    graph_path: str
    optical_frame: str
    node_namespace: str
    rgb: CameraStream
    camera_info: CameraStream
    depth: CameraStream
    depth_points: CameraStream
    depth_points_enabled: bool
    width: int
    height: int
    publish_rate_hz: float
    _render_product_released: bool = field(default=False, init=False, repr=False)

    def release_render_product(self) -> None:
        """Destroy the owned Hydra render product exactly once."""

        if self._render_product_released:
            return
        destroy = getattr(self.render_product, "destroy", None)
        if not callable(destroy):
            raise SensorConfigError(
                f"camera {self.name} render product has no destroy() owner API"
            )
        destroy()
        self._render_product_released = True


@dataclass
class SensorBundle:
    lidar: object
    lidar_prim_path: str
    lidar_render_product: object
    lidar_render_product_path: str
    imu: object
    imu_prim_path: str
    cameras: tuple[CameraRuntime, ...]

    def close_camera_resources(self) -> None:
        """Release render products, then remove dynamically authored Camera prims."""

        errors: list[str] = []
        for camera in self.cameras:
            try:
                camera.release_render_product()
            except Exception as exc:
                errors.append(f"{camera.name} render product: {exc}")

        if self.cameras:
            try:
                import omni.usd

                stage = omni.usd.get_context().get_stage()
                if stage is not None:
                    for camera in self.cameras:
                        prim = stage.GetPrimAtPath(camera.camera_prim_path)
                        if prim.IsValid():
                            stage.RemovePrim(camera.camera_prim_path)
            except Exception as exc:
                errors.append(f"camera prim cleanup: {exc}")

        if errors:
            raise SensorConfigError("; ".join(errors))


def _require_mapping(value: Any, keys: set[str], *, context: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise SensorConfigError(f"{context} must be a mapping")
    reject_unknown(value, keys, context=context)
    require_keys(value, keys, context=context)
    return value


def _require_bool(value: Any, *, context: str) -> bool:
    if not isinstance(value, bool):
        raise SensorConfigError(f"{context} must be boolean")
    return value


def _require_positive_int(value: Any, *, context: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise SensorConfigError(f"{context} must be a positive integer")
    return value


def _require_string(value: Any, *, context: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise SensorConfigError(f"{context} must be a non-empty string")
    return value.strip()


def _load_camera_stream(
    value: Any,
    *,
    context: str,
    encoding: bool,
) -> CameraStream:
    keys = {"enabled", "topic_name", "qos_profile", "queue_size"}
    if encoding:
        keys.add("encoding")
    data = _require_mapping(value, keys, context=context)
    topic_name = _require_string(data["topic_name"], context=f"{context}.topic_name")
    if topic_name.startswith("/"):
        raise SensorConfigError(f"{context}.topic_name must be relative to node_namespace")
    stream_encoding = None
    if encoding:
        stream_encoding = _require_string(data["encoding"], context=f"{context}.encoding")
    return CameraStream(
        enabled=_require_bool(data["enabled"], context=f"{context}.enabled"),
        topic_name=topic_name,
        qos_profile=_require_string(
            data["qos_profile"], context=f"{context}.qos_profile"
        ),
        queue_size=_require_positive_int(
            data["queue_size"], context=f"{context}.queue_size"
        ),
        encoding=stream_encoding,
    )


def _load_camera_profile(name: str, value: Any) -> CameraProfile:
    expected = _CAMERA_PROFILE_CONTRACT[name]
    keys = {"enabled", "depth_points_enabled"} if name == "off" else {
        "enabled", "width", "height", "publish_rate_hz", "depth_points_enabled"
    }
    data = _require_mapping(value, keys, context=f"camera.profiles.{name}")
    enabled = _require_bool(data["enabled"], context=f"camera.profiles.{name}.enabled")
    if name == "off":
        profile = CameraProfile(
            name, enabled, 0, 0, 0.0,
            _require_bool(data["depth_points_enabled"], context=f"camera.profiles.{name}.depth_points_enabled"),
        )
    else:
        profile = CameraProfile(
            name=name,
            enabled=enabled,
            width=_require_positive_int(
                data["width"], context=f"camera.profiles.{name}.width"
            ),
            height=_require_positive_int(
                data["height"], context=f"camera.profiles.{name}.height"
            ),
            publish_rate_hz=require_number(
                data["publish_rate_hz"],
                context=f"camera.profiles.{name}.publish_rate_hz",
                positive=True,
            ),
            depth_points_enabled=_require_bool(
                data["depth_points_enabled"],
                context=f"camera.profiles.{name}.depth_points_enabled",
            ),
        )
    observed = (
        profile.enabled,
        profile.width,
        profile.height,
        profile.publish_rate_hz,
        profile.depth_points_enabled,
    )
    if observed != expected:
        raise SensorConfigError(
            f"camera profile {name!r} must preserve the verified contract "
            f"enabled/width/height/rate={expected}, got {observed}"
        )
    return profile


def _load_camera_definition(name: str, value: Any) -> CameraDefinition:
    keys = {
        "enabled",
        "sensor_prim",
        "link_frame",
        "optical_frame",
        "node_namespace",
        "rgb",
        "camera_info",
        "depth",
        "depth_points",
        "clipping_range_m",
        "optics",
        "exposure",
        "rviz",
    }
    data = _require_mapping(value, keys, context=f"camera.cameras.{name}")
    sensor_prim = _require_string(
        data["sensor_prim"], context=f"camera.cameras.{name}.sensor_prim"
    )
    if not sensor_prim.startswith("/") or ".." in PurePosixPath(sensor_prim).parts:
        raise SensorConfigError(f"camera.cameras.{name}.sensor_prim must be absolute")
    link_frame = _require_string(
        data["link_frame"], context=f"camera.cameras.{name}.link_frame"
    )
    optical_frame = _require_string(
        data["optical_frame"], context=f"camera.cameras.{name}.optical_frame"
    )
    if any("/" in frame for frame in (link_frame, optical_frame)):
        raise SensorConfigError("camera ROS frame IDs must be relative names")
    namespace = _require_string(
        data["node_namespace"], context=f"camera.cameras.{name}.node_namespace"
    )
    if not namespace.startswith("/") or namespace.endswith("/"):
        raise SensorConfigError("camera node_namespace must be absolute without a trailing slash")

    rgb = _load_camera_stream(
        data["rgb"], context=f"camera.cameras.{name}.rgb", encoding=True
    )
    camera_info = _load_camera_stream(
        data["camera_info"],
        context=f"camera.cameras.{name}.camera_info",
        encoding=False,
    )
    depth = _load_camera_stream(
        data["depth"], context=f"camera.cameras.{name}.depth", encoding=False
    )
    depth_points = _load_camera_stream(
        data["depth_points"],
        context=f"camera.cameras.{name}.depth_points",
        encoding=False,
    )
    if not rgb.enabled or rgb.encoding != "rgb8":
        raise SensorConfigError("front Camera RGB must be enabled with rgb8 encoding")
    if not camera_info.enabled or not depth.enabled or not depth_points.enabled:
        raise SensorConfigError(
            "CameraInfo, raw depth, and depth points must be enabled"
        )
    if len({rgb.qos_profile, camera_info.qos_profile, depth.qos_profile, depth_points.qos_profile}) != 1:
        raise SensorConfigError("Camera streams must use the same QoS profile")
    if (rgb.queue_size, camera_info.queue_size, depth.queue_size, depth_points.queue_size) != (2, 2, 2, 2):
        raise SensorConfigError("Camera RGB, CameraInfo, raw depth, and depth points queue_size must be 2")

    clipping = _require_mapping(
        data["clipping_range_m"], {"near", "far"},
        context=f"camera.cameras.{name}.clipping_range_m"
    )
    near = require_number(
        clipping["near"], context=f"camera.cameras.{name}.clipping_range_m.near",
        positive=True,
    )
    far = require_number(
        clipping["far"], context=f"camera.cameras.{name}.clipping_range_m.far",
        positive=True,
    )
    if near >= far:
        raise SensorConfigError("camera clipping near must be smaller than far")

    optics = _require_mapping(
        data["optics"],
        {
            "projection",
            "focal_length_mm",
            "horizontal_aperture_mm",
            "vertical_aperture_mode",
            "focus_distance_m",
        },
        context=f"camera.cameras.{name}.optics",
    )
    projection = _require_string(
        optics["projection"], context=f"camera.cameras.{name}.optics.projection"
    )
    if projection != "perspective":
        raise SensorConfigError("front Camera projection must be perspective")
    vertical_aperture_mode = _require_string(
        optics["vertical_aperture_mode"],
        context=f"camera.cameras.{name}.optics.vertical_aperture_mode",
    )
    if vertical_aperture_mode != "match_profile_aspect_ratio":
        raise SensorConfigError(
            "front Camera vertical aperture must match the selected profile "
            "aspect ratio"
        )

    exposure = _require_mapping(
        data["exposure"], {"enabled", "time_s", "responsivity", "f_stop"},
        context=f"camera.cameras.{name}.exposure"
    )
    rviz = _require_mapping(
        data["rviz"], {"enabled", "transport", "reliability", "queue_size"},
        context=f"camera.cameras.{name}.rviz"
    )
    rviz_transport = _require_string(
        rviz["transport"], context=f"camera.cameras.{name}.rviz.transport"
    )
    rviz_reliability = _require_string(
        rviz["reliability"], context=f"camera.cameras.{name}.rviz.reliability"
    )
    if rviz_transport != "raw" or rviz_reliability != "best_effort":
        raise SensorConfigError("Camera RViz contract must use raw Best Effort")

    return CameraDefinition(
        name=name,
        enabled=_require_bool(data["enabled"], context=f"camera.cameras.{name}.enabled"),
        sensor_prim=sensor_prim,
        link_frame=link_frame,
        optical_frame=optical_frame,
        node_namespace=namespace,
        rgb=rgb,
        camera_info=camera_info,
        depth=depth,
        depth_points=depth_points,
        clipping_range_m=(near, far),
        projection=projection,
        focal_length_mm=require_number(
            optics["focal_length_mm"],
            context=f"camera.cameras.{name}.optics.focal_length_mm",
            positive=True,
        ),
        horizontal_aperture_mm=require_number(
            optics["horizontal_aperture_mm"],
            context=f"camera.cameras.{name}.optics.horizontal_aperture_mm",
            positive=True,
        ),
        vertical_aperture_mode=vertical_aperture_mode,
        focus_distance_m=require_number(
            optics["focus_distance_m"],
            context=f"camera.cameras.{name}.optics.focus_distance_m",
            positive=True,
        ),
        exposure_enabled=_require_bool(
            exposure["enabled"], context=f"camera.cameras.{name}.exposure.enabled"
        ),
        exposure_time_s=require_number(
            exposure["time_s"], context=f"camera.cameras.{name}.exposure.time_s",
            positive=True,
        ),
        exposure_responsivity=require_number(
            exposure["responsivity"],
            context=f"camera.cameras.{name}.exposure.responsivity",
            positive=True,
        ),
        exposure_f_stop=require_number(
            exposure["f_stop"], context=f"camera.cameras.{name}.exposure.f_stop",
            positive=True,
        ),
        rviz_enabled=_require_bool(
            rviz["enabled"], context=f"camera.cameras.{name}.rviz.enabled"
        ),
        rviz_transport=rviz_transport,
        rviz_reliability=rviz_reliability,
        rviz_queue_size=_require_positive_int(
            rviz["queue_size"], context=f"camera.cameras.{name}.rviz.queue_size"
        ),
    )


def _load_lidar(path) -> dict[str, Any]:
    data = load_mapping(path)
    allowed = {
        "schema_version",
        "enabled",
        "sensor_prim",
        "config",
        "variant",
        "scan_plane_rotation_wxyz",
        "tick_rate",
        "accumulate_outputs",
        "render_product_resolution",
        "topic_name",
        "frame_id",
        "output_frame",
        "motion_compensation",
        "type",
        "qos_profile",
    }
    reject_unknown(data, allowed, context="lidar config")
    require_keys(data, allowed, context="lidar config")
    if data["schema_version"] != 1 or data["enabled"] is not True:
        raise SensorConfigError("navigation LiDAR must be enabled with schema_version 1")
    if data["config"] != "RPLIDAR_S2E":
        raise SensorConfigError(
            "navigation mapping requires the single-channel RPLIDAR_S2E RTX config"
        )
    if data["type"] != "point_cloud" or data["topic_name"] != "/lidar/points_raw":
        raise SensorConfigError("LiDAR must publish point_cloud on /lidar/points_raw")
    if data["frame_id"] != "rtx_world":
        raise SensorConfigError("RTX PointCloud frame_id must be rtx_world")
    if data["output_frame"] != "WORLD":
        raise SensorConfigError(
            "RTX PointCloud output_frame must be WORLD when frame_id is rtx_world"
        )
    if data["motion_compensation"] != "COMPENSATED":
        raise SensorConfigError(
            "moving navigation LiDAR requires COMPENSATED full-scan output"
        )
    rotation = require_vector(
        data["scan_plane_rotation_wxyz"],
        4,
        context="lidar.scan_plane_rotation_wxyz",
    )
    norm_squared = sum(float(value) ** 2 for value in rotation)
    if abs(norm_squared - 1.0) > 1e-6:
        raise SensorConfigError("LiDAR scan-plane rotation must be a unit quaternion")
    data["tick_rate"] = require_number(data["tick_rate"], context="lidar.tick_rate", positive=True)
    data["render_product_resolution"] = require_vector(
        data["render_product_resolution"], 2, context="lidar.render_product_resolution"
    )
    if tuple(data["render_product_resolution"]) != (1.0, 1.0):
        raise SensorConfigError(
            "RTX LiDAR render_product_resolution must be [1, 1] under "
            "the Isaac Sim 6.0 multi-tick contract"
        )
    return data


def _load_imu(path) -> dict[str, Any]:
    data = load_mapping(path)
    allowed = {
        "schema_version",
        "enabled",
        "sensor_prim",
        "frame_id",
        "topic_name",
        "publish_rate",
        "read_gravity",
        "use_latest_data",
        "filter_widths",
        "qos_profile",
    }
    reject_unknown(data, allowed, context="imu config")
    require_keys(data, allowed, context="imu config")
    if data["schema_version"] != 1 or data["enabled"] is not True:
        raise SensorConfigError("IMU must be enabled with schema_version 1")
    if data["frame_id"] != "imu_link" or data["topic_name"] != "/imu/data_raw":
        raise SensorConfigError(
            "IMU frame/topic contract is imu_link and /imu/data_raw"
        )
    filters = data["filter_widths"]
    if not isinstance(filters, dict) or set(filters) != {"linear_acceleration", "angular_velocity", "orientation"}:
        raise SensorConfigError("IMU filter_widths must define all three filter sizes")
    if any(isinstance(value, bool) or not isinstance(value, int) or value < 1 for value in filters.values()):
        raise SensorConfigError("IMU filter widths must be positive integers")
    return data


def _load_camera(path) -> CameraConfig:
    data = load_mapping(path)
    allowed = {
        "schema_version",
        "enabled",
        "default_profile",
        "primary_camera",
        "profiles",
        "cameras",
    }
    reject_unknown(data, allowed, context="camera config")
    require_keys(data, allowed, context="camera config")
    if data["schema_version"] != 3:
        raise SensorConfigError("camera schema_version must be 3")

    raw_profiles = _require_mapping(
        data["profiles"], set(CAMERA_PROFILE_NAMES), context="camera.profiles"
    )
    profiles = {
        name: _load_camera_profile(name, raw_profiles[name])
        for name in CAMERA_PROFILE_NAMES
    }
    raw_cameras = _require_mapping(
        data["cameras"], {"front"}, context="camera.cameras"
    )
    cameras = {"front": _load_camera_definition("front", raw_cameras["front"])}
    default_profile = _require_string(
        data["default_profile"], context="camera.default_profile"
    )
    primary_camera = _require_string(
        data["primary_camera"], context="camera.primary_camera"
    )
    if default_profile not in profiles or default_profile == "off":
        raise SensorConfigError("camera.default_profile must name an enabled profile")
    if primary_camera != "front":
        raise SensorConfigError("camera.primary_camera must be canonical front")
    front = cameras[primary_camera]
    if (
        front.link_frame != "camera_front_link"
        or front.optical_frame != "camera_front_optical_frame"
        or front.node_namespace != "/camera/front"
    ):
        raise SensorConfigError(
            "canonical front Camera must use camera_front_link, "
            "camera_front_optical_frame, and /camera/front"
        )
    if not front.enabled:
        raise SensorConfigError("primary front Camera must be enabled")
    return CameraConfig(
        enabled=_require_bool(data["enabled"], context="camera.enabled"),
        default_profile=default_profile,
        primary_camera=primary_camera,
        profiles=profiles,
        cameras=cameras,
    )


def resolve_camera_selection(
    config: CameraConfig,
    requested_profile: str | None,
    *,
    headless: bool,
) -> CameraSelection:
    """Resolve explicit CLI choice or GUI/headless defaults before Kit starts."""

    name = requested_profile
    if name is None:
        name = "off" if headless else config.default_profile
    if name not in config.profiles:
        raise SensorConfigError(
            f"unknown camera profile {name!r}; available={list(CAMERA_PROFILE_NAMES)}"
        )
    profile = config.profiles[name]
    if profile.enabled and not config.enabled:
        raise SensorConfigError(
            f"camera profile {name!r} requested while camera.enabled=false"
        )
    camera = config.cameras[config.primary_camera] if profile.enabled else None
    return CameraSelection(profile=profile, camera=camera)


class SensorFactory:
    def __init__(self, config: ProjectConfig, camera_profile: str | None = None):
        self.config = config
        self.camera_profile = camera_profile

    def create_all(self) -> SensorBundle:
        """Create sensors after SimulationApp and required extensions are active."""

        lidar_config = _load_lidar(self.config.files.lidar)
        imu_config = _load_imu(self.config.files.imu)
        camera_config = _load_camera(self.config.files.camera)
        camera_selection = resolve_camera_selection(
            camera_config,
            self.camera_profile,
            headless=self.config.simulation.headless,
        )

        import carb.settings

        carb.settings.get_settings().set(
            "/persistent/isaac/asset_root/default", str(self.config.asset_root)
        )

        from isaacsim.sensors.experimental.rtx import Lidar
        import omni.replicator.core as rep

        lidar = Lidar.create(
            path=lidar_config["sensor_prim"],
            config=lidar_config["config"],
            variant=lidar_config["variant"],
            tick_rate=lidar_config["tick_rate"],
            accumulate_outputs=bool(lidar_config["accumulate_outputs"]),
            attributes={
                "omni:sensor:Core:outputFrameOfReference": lidar_config[
                    "output_frame"
                ],
                "omni:sensor:Core:outputMotionCompensationState": lidar_config[
                    "motion_compensation"
                ],
            },
        )
        if len(lidar.paths) != 1:
            raise SensorConfigError(
                f"expected one RTX LiDAR prim, got {list(lidar.paths)}"
            )
        # Rotary assets are authored in a vertical X-Z plane. Keep the physical
        # lidar_link TF unchanged and rotate only the internal ray generator
        # into the horizontal X-Y navigation plane.
        lidar.set_local_poses(
            orientations=[lidar_config["scan_plane_rotation_wxyz"]]
        )
        # Sensor assets may reference an Xform containing the actual OmniLidar
        # child. A render product must target that resolved sensor prim, not the
        # configured mount root.
        lidar_prim_path = str(lidar.paths[0])
        lidar_prim = lidar.prims[0]
        output_frame = lidar_prim.GetAttribute(
            "omni:sensor:Core:outputFrameOfReference"
        ).Get()
        if str(output_frame) != lidar_config["output_frame"]:
            raise SensorConfigError(
                f"LiDAR {lidar_prim_path} output frame is {output_frame!r}, "
                f"expected {lidar_config['output_frame']!r}"
            )
        motion_compensation = lidar_prim.GetAttribute(
            "omni:sensor:Core:outputMotionCompensationState"
        ).Get()
        if str(motion_compensation) != lidar_config["motion_compensation"]:
            raise SensorConfigError(
                f"LiDAR {lidar_prim_path} motion compensation is "
                f"{motion_compensation!r}, expected "
                f"{lidar_config['motion_compensation']!r}"
            )
        resolution = tuple(int(value) for value in lidar_config["render_product_resolution"])
        render_product = rep.create.render_product(lidar_prim_path, resolution=resolution)
        render_product_path = str(render_product.path)
        if not render_product_path:
            raise SensorConfigError("RTX LiDAR render product creation returned an empty path")

        from isaacsim.sensors.experimental.physics import IMU

        filters = imu_config["filter_widths"]
        imu = IMU.create(
            imu_config["sensor_prim"],
            linear_acceleration_filter_size=filters["linear_acceleration"],
            angular_velocity_filter_size=filters["angular_velocity"],
            orientation_filter_size=filters["orientation"],
        )

        cameras: tuple[CameraRuntime, ...] = ()
        if camera_selection.camera is not None:
            cameras = (self._create_camera(camera_selection),)

        return SensorBundle(
            lidar=lidar,
            lidar_prim_path=lidar_prim_path,
            lidar_render_product=render_product,
            lidar_render_product_path=render_product_path,
            imu=imu,
            imu_prim_path=imu_config["sensor_prim"],
            cameras=cameras,
        )

    def _create_camera(self, selection: CameraSelection) -> CameraRuntime:
        """Author one RTX Camera and one owned Render Product for both ROS streams."""

        if selection.camera is None or not selection.profile.enabled:
            raise SensorConfigError("cannot create a disabled Camera selection")
        definition = selection.camera
        profile = selection.profile
        expected_prefix = f"{self.config.robot.runtime_prim_path}/"
        if not definition.sensor_prim.startswith(expected_prefix):
            raise SensorConfigError(
                f"camera sensor prim must be inside {self.config.robot.runtime_prim_path}: "
                f"{definition.sensor_prim}"
            )
        expected_parent = (
            f"{self.config.robot.base_link_prim}/camera_link/"
            f"{definition.link_frame}/{definition.optical_frame}"
        )
        if str(PurePosixPath(definition.sensor_prim).parent) != expected_parent:
            raise SensorConfigError(
                "camera sensor prim must be a child of the canonical optical frame; "
                f"expected parent={expected_parent}, got={definition.sensor_prim}"
            )

        import omni.replicator.core as rep
        import omni.usd
        from isaacsim.sensors.experimental.rtx import RtxCamera
        from pxr import Gf, UsdGeom
        from isaac_sim.src.stage.scene_composer import author_configured_static_frames

        stage = omni.usd.get_context().get_stage()
        # Reassert the project-layer specs immediately before Camera creation:
        # Kit may complete the referenced Jackal payload after stage validation.
        author_configured_static_frames(
            stage, self.config.robot.base_link_prim, self.config.files.robot
        )
        for frame_path in (
            f"{self.config.robot.base_link_prim}/camera_link/{definition.link_frame}",
            expected_parent,
        ):
            frame = stage.GetPrimAtPath(frame_path)
            if not frame.IsValid() or frame.GetTypeName() != "Xform":
                raise SensorConfigError(f"canonical Camera frame is missing: {frame_path}")
        if stage.GetPrimAtPath(definition.sensor_prim).IsValid():
            raise SensorConfigError(
                f"camera sensor prim already exists before creation: {definition.sensor_prim}"
            )

        render_product = None
        try:
            sensor = RtxCamera.create(
                definition.sensor_prim,
                tick_rate=profile.publish_rate_hz,
                reset_xform_op_properties=False,
            )
            if tuple(str(path) for path in sensor.paths) != (definition.sensor_prim,):
                raise SensorConfigError(
                    f"expected one Camera prim {definition.sensor_prim}, got {sensor.paths}"
                )
            camera_prim = stage.GetPrimAtPath(definition.sensor_prim)
            if not camera_prim.IsValid() or camera_prim.GetTypeName() != "Camera":
                raise SensorConfigError(
                    f"RtxCamera did not author a Camera prim: {definition.sensor_prim}"
                )

            # USD cameras look along local -Z with +Y up. The ROS optical frame
            # looks along +Z with +Y down, so Rx(pi) is the fixed convention
            # adapter. The installation pose remains owned by the parent frames.
            camera_xform = UsdGeom.Xformable(camera_prim)
            camera_xform.ClearXformOpOrder()
            camera_xform.AddOrientOp().Set(
                Gf.Quatf(0.0, Gf.Vec3f(1.0, 0.0, 0.0))
            )

            camera_schema = UsdGeom.Camera(camera_prim)
            camera_schema.CreateProjectionAttr().Set(definition.projection)
            camera_schema.CreateFocalLengthAttr().Set(definition.focal_length_mm)
            camera_schema.CreateHorizontalApertureAttr().Set(
                definition.horizontal_aperture_mm
            )
            camera_schema.CreateVerticalApertureAttr().Set(
                definition.horizontal_aperture_mm
                * profile.height
                / profile.width
            )
            camera_schema.CreateFocusDistanceAttr().Set(definition.focus_distance_m)
            camera_schema.CreateClippingRangeAttr().Set(
                Gf.Vec2f(*definition.clipping_range_m)
            )
            camera_schema.CreateFStopAttr().Set(definition.exposure_f_stop)
            if definition.exposure_enabled:
                camera_schema.CreateExposureTimeAttr().Set(
                    definition.exposure_time_s
                )
                camera_schema.CreateExposureResponsivityAttr().Set(
                    definition.exposure_responsivity
                )
                camera_schema.CreateExposureFStopAttr().Set(
                    definition.exposure_f_stop
                )

            render_product = rep.create.render_product(
                definition.sensor_prim,
                resolution=(profile.width, profile.height),
                force_new=True,
                name="CameraFront",
                render_vars=["LdrColor"],
            )
            render_product_path = str(render_product.path)
            if not render_product_path or not stage.GetPrimAtPath(
                render_product_path
            ).IsValid():
                raise SensorConfigError(
                    "front Camera render product creation returned an invalid path"
                )
        except Exception:
            if render_product is not None:
                destroy = getattr(render_product, "destroy", None)
                if callable(destroy):
                    destroy()
            if stage.GetPrimAtPath(definition.sensor_prim).IsValid():
                stage.RemovePrim(definition.sensor_prim)
            raise

        return CameraRuntime(
            name=definition.name,
            profile_name=profile.name,
            sensor=sensor,
            camera_prim_path=definition.sensor_prim,
            render_product=render_product,
            render_product_path=render_product_path,
            graph_path="/World/Graphs/ROS2CameraFront",
            optical_frame=definition.optical_frame,
            node_namespace=definition.node_namespace,
            rgb=definition.rgb,
            camera_info=definition.camera_info,
            depth=definition.depth,
            depth_points=definition.depth_points,
            depth_points_enabled=profile.depth_points_enabled,
            width=profile.width,
            height=profile.height,
            publish_rate_hz=profile.publish_rate_hz,
        )
