"""Build all ROS-facing OmniGraphs according to strict mode ownership."""

from __future__ import annotations

from dataclasses import dataclass

from isaac_sim.graphs.camera_graph import build_camera_graphs
from isaac_sim.graphs.control_graph import build_control_graph
from isaac_sim.graphs.odometry_graph import build_odometry_graph
from isaac_sim.graphs.sensor_graph import build_sensor_graphs
from isaac_sim.graphs.tf_graph import build_tf_graph
from isaac_sim.src.config import ProjectConfig


@dataclass(frozen=True)
class RosGraphHandles:
    control: object
    sensors: tuple[object, object]
    tf: object | None
    odometry: object | None
    cameras: tuple[object, ...]


class RosGraphBuilder:
    def __init__(self, config: ProjectConfig, sensors):
        self.config = config
        self.sensors = sensors

    def build(self) -> RosGraphHandles:
        if self.config.simulation.odometry_mode == "realistic":
            # Deliberately do not instantiate IsaacComputeOdometry or its TF publisher.
            odometry = None
        else:
            odometry = build_odometry_graph(self.config)
        return RosGraphHandles(
            control=build_control_graph(self.config),
            sensors=build_sensor_graphs(
                self.config,
                self.sensors.imu_prim_path,
                self.sensors.lidar_render_product_path,
            ),
            tf=(
                build_tf_graph(self.config)
                if self.config.simulation.structure_tf_source == "isaac"
                else None
            ),
            odometry=odometry,
            cameras=build_camera_graphs(self.config, self.sensors.cameras),
        )
