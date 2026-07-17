"""Isaac Sim 6.0.1 sensor authoring with no module-level Isaac imports."""

from __future__ import annotations

from dataclasses import dataclass
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


@dataclass
class SensorBundle:
    lidar: object
    lidar_prim_path: str
    lidar_render_product: object
    lidar_render_product_path: str
    imu: object
    imu_prim_path: str
    cameras: tuple[object, ...]


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
    if data["frame_id"] != "imu_link" or data["topic_name"] != "/imu/data":
        raise SensorConfigError("IMU frame/topic contract is imu_link and /imu/data")
    filters = data["filter_widths"]
    if not isinstance(filters, dict) or set(filters) != {"linear_acceleration", "angular_velocity", "orientation"}:
        raise SensorConfigError("IMU filter_widths must define all three filter sizes")
    if any(isinstance(value, bool) or not isinstance(value, int) or value < 1 for value in filters.values()):
        raise SensorConfigError("IMU filter widths must be positive integers")
    return data


def _load_camera(path) -> dict[str, Any]:
    data = load_mapping(path)
    allowed = {"schema_version", "enabled", "parent_prim", "publish_rate", "stereo_baseline", "left", "right"}
    reject_unknown(data, allowed, context="camera config")
    require_keys(data, allowed, context="camera config")
    if data["schema_version"] != 1 or not isinstance(data["enabled"], bool):
        raise SensorConfigError("invalid camera schema/enabled value")
    return data


class SensorFactory:
    def __init__(self, config: ProjectConfig):
        self.config = config

    def create_all(self) -> SensorBundle:
        """Create sensors after SimulationApp and required extensions are active."""

        lidar_config = _load_lidar(self.config.files.lidar)
        imu_config = _load_imu(self.config.files.imu)
        camera_config = _load_camera(self.config.files.camera)

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

        cameras: tuple[object, ...] = ()
        if camera_config["enabled"]:
            from pxr import UsdGeom
            import omni.usd

            stage = omni.usd.get_context().get_stage()
            camera_prims = []
            for side in ("left", "right"):
                side_config = camera_config[side]
                if not isinstance(side_config, dict) or "sensor_prim" not in side_config:
                    raise SensorConfigError(f"camera.{side}.sensor_prim is required")
                camera_prims.append(UsdGeom.Camera.Define(stage, side_config["sensor_prim"]))
            cameras = tuple(camera_prims)

        return SensorBundle(
            lidar=lidar,
            lidar_prim_path=lidar_prim_path,
            lidar_render_product=render_product,
            lidar_render_product_path=render_product_path,
            imu=imu,
            imu_prim_path=imu_config["sensor_prim"],
            cameras=cameras,
        )
