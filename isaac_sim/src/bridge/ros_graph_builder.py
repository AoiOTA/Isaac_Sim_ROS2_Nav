"""Build all ROS-facing OmniGraphs according to strict mode ownership."""

from __future__ import annotations

from dataclasses import dataclass

from isaac_sim.graphs.camera_graph import build_camera_graphs
from isaac_sim.graphs.control_graph import (
    SPLIT_AXLE_V1,
    build_control_graph,
    build_control_graph_with_snapshot,
    capture_materialized_control_graph_snapshot,
    control_graph_spec,
    require_wheel_command_application,
)
from isaac_sim.graphs.odometry_graph import build_odometry_graph
from isaac_sim.graphs.sensor_graph import build_sensor_graphs
from isaac_sim.graphs.tf_graph import build_tf_graph
from isaac_sim.src.config import ProjectConfig


@dataclass(frozen=True)
class RosGraphHandles:
    control: object
    control_snapshot: dict[str, object]
    sensors: tuple[object, object]
    tf: object | None
    odometry: object | None
    cameras: tuple[object, ...]


@dataclass(frozen=True)
class RosGraphBuilder:
    config: ProjectConfig
    sensors: object
    wheel_command_application: str = SPLIT_AXLE_V1

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "wheel_command_application",
            require_wheel_command_application(self.wheel_command_application),
        )

    def build_control(self) -> object:
        return build_control_graph(
            self.config,
            self.wheel_command_application,
        )

    def control_spec(self):
        return control_graph_spec(
            self.config,
            self.wheel_command_application,
        )

    def capture_control_snapshot(
        self,
        materialized_graph: object,
    ) -> dict[str, object]:
        """Recapture a rebuilt graph for reset-time SHA comparison."""

        return capture_materialized_control_graph_snapshot(
            self.control_spec(),
            materialized_graph,
            self.wheel_command_application,
        )

    def build_control_with_snapshot(
        self,
    ) -> tuple[object, dict[str, object]]:
        return build_control_graph_with_snapshot(
            self.config,
            self.wheel_command_application,
        )

    def build(self) -> RosGraphHandles:
        control, control_snapshot = self.build_control_with_snapshot()
        if self.config.simulation.odometry_mode == "realistic":
            # Deliberately do not instantiate IsaacComputeOdometry or its TF publisher.
            odometry = None
        else:
            odometry = build_odometry_graph(self.config)
        return RosGraphHandles(
            control=control,
            control_snapshot=control_snapshot,
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
