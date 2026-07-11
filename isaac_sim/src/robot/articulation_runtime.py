"""Thin Jackal runtime adapter over Isaac Sim 6.0 experimental Articulation."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

from isaac_sim.src.yaml_utils import (
    load_mapping,
    reject_unknown,
    require_keys,
    require_number,
)


class ArticulationRuntimeError(RuntimeError):
    pass


@dataclass(frozen=True)
class ArticulationPhysicsConfig:
    sleep_threshold: float
    stabilization_threshold: float
    wheel_static_friction_effort: float
    wheel_dynamic_friction_effort: float
    wheel_viscous_friction_coefficient: float
    idle_brake_command_timeout_sec: float
    idle_brake_command_deadband: float


def load_articulation_physics_config(
    path: str | Path,
) -> ArticulationPhysicsConfig:
    data = load_mapping(path)
    physics = data.get("physics")
    if not isinstance(physics, dict):
        raise ArticulationRuntimeError("robot.physics must be a mapping")
    fields = {
        "sleep_threshold",
        "stabilization_threshold",
        "wheel_static_friction_effort",
        "wheel_dynamic_friction_effort",
        "wheel_viscous_friction_coefficient",
        "idle_brake_command_timeout_sec",
        "idle_brake_command_deadband",
    }
    reject_unknown(physics, fields, context="robot.physics")
    require_keys(physics, fields, context="robot.physics")
    static_friction = require_number(
        physics["wheel_static_friction_effort"],
        context="robot.physics.wheel_static_friction_effort",
    )
    dynamic_friction = require_number(
        physics["wheel_dynamic_friction_effort"],
        context="robot.physics.wheel_dynamic_friction_effort",
    )
    viscous_friction = require_number(
        physics["wheel_viscous_friction_coefficient"],
        context="robot.physics.wheel_viscous_friction_coefficient",
    )
    if min(static_friction, dynamic_friction, viscous_friction) < 0.0:
        raise ArticulationRuntimeError(
            "robot.physics wheel friction values must be non-negative"
        )
    if static_friction < dynamic_friction:
        raise ArticulationRuntimeError(
            "wheel static friction effort must be at least dynamic friction effort"
        )
    return ArticulationPhysicsConfig(
        sleep_threshold=require_number(
            physics["sleep_threshold"],
            context="robot.physics.sleep_threshold",
            positive=True,
        ),
        stabilization_threshold=require_number(
            physics["stabilization_threshold"],
            context="robot.physics.stabilization_threshold",
            positive=True,
        ),
        wheel_static_friction_effort=static_friction,
        wheel_dynamic_friction_effort=dynamic_friction,
        wheel_viscous_friction_coefficient=viscous_friction,
        idle_brake_command_timeout_sec=require_number(
            physics["idle_brake_command_timeout_sec"],
            context="robot.physics.idle_brake_command_timeout_sec",
            positive=True,
        ),
        idle_brake_command_deadband=require_number(
            physics["idle_brake_command_deadband"],
            context="robot.physics.idle_brake_command_deadband",
            positive=True,
        ),
    )


class ArticulationRuntime:
    def __init__(self, prim_path: str, base_link_prim_path: str, app):
        self.prim_path = prim_path
        self.base_link_prim_path = base_link_prim_path
        self.app = app
        self._articulation = None

    def initialize(self) -> None:
        from isaacsim.core.experimental.prims import Articulation
        from isaacsim.core.simulation_manager import SimulationManager

        self._articulation = Articulation(self.prim_path)
        self.app.update()
        if not self._articulation.is_physics_tensor_entity_valid():
            SimulationManager.initialize_physics()
            self.app.update()
        if not self._articulation.is_physics_tensor_entity_valid():
            # Rebind after physics warmup in case the first wrapper preceded view creation.
            self._articulation = Articulation(self.prim_path)
            self.app.update()
        if not self._articulation.is_physics_tensor_entity_valid():
            raise ArticulationRuntimeError(f"physics articulation view is invalid for {self.prim_path}")

    @property
    def articulation(self):
        if self._articulation is None:
            raise ArticulationRuntimeError("articulation is not initialized")
        return self._articulation

    @property
    def num_dof(self) -> int:
        return int(self.articulation.num_dofs)

    def get_dof_names(self) -> tuple[str, ...]:
        return tuple(self.articulation.dof_names)

    def configure_stability(
        self, settings: ArticulationPhysicsConfig
    ) -> None:
        self.articulation.set_sleep_thresholds([settings.sleep_threshold])
        self.articulation.set_stabilization_thresholds(
            [settings.stabilization_threshold]
        )
        self.articulation.set_dof_friction_properties(
            static_frictions=[settings.wheel_static_friction_effort],
            dynamic_frictions=[settings.wheel_dynamic_friction_effort],
            viscous_frictions=[settings.wheel_viscous_friction_coefficient],
        )

    def set_world_pose(self, position: Sequence[float], orientation_wxyz: Sequence[float]) -> None:
        self.articulation.set_world_poses(positions=[list(position)], orientations=[list(orientation_wxyz)])

    def get_world_pose(self) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
        positions, orientations = self.articulation.get_world_poses()
        position = tuple(float(v) for v in positions.numpy()[0])
        orientation = tuple(float(v) for v in orientations.numpy()[0])
        return position, orientation  # type: ignore[return-value]

    def set_base_velocities(self, linear: Sequence[float], angular: Sequence[float]) -> None:
        self.articulation.set_velocities(linear_velocities=[list(linear)], angular_velocities=[list(angular)])

    def set_joint_velocities(self, values: Sequence[float]) -> None:
        if len(values) != self.num_dof:
            raise ArticulationRuntimeError(f"expected {self.num_dof} joint velocities, got {len(values)}")
        self.articulation.set_dof_velocities([list(values)])

    def set_joint_velocity_targets(self, values: Sequence[float]) -> None:
        if len(values) != self.num_dof:
            raise ArticulationRuntimeError(f"expected {self.num_dof} joint targets, got {len(values)}")
        self.articulation.set_dof_velocity_targets([list(values)])

    def zero_all_velocities(self) -> None:
        zeros = [0.0] * self.num_dof
        self.set_base_velocities([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        self.set_joint_velocities(zeros)
        self.set_joint_velocity_targets(zeros)

    def _physx_body_handle(self):
        import omni.usd
        from omni.physx.bindings._physx import (
            acquire_physx_simulation_interface,
        )
        from pxr.PhysicsSchemaTools import sdfPathToInt

        stage_id = omni.usd.get_context().get_stage_id()
        return (
            acquire_physx_simulation_interface(),
            stage_id,
            sdfPathToInt(self.base_link_prim_path),
        )

    def put_to_sleep(self) -> None:
        interface, stage_id, body_path = self._physx_body_handle()
        interface.put_to_sleep(stage_id, body_path)

    def wake_up(self) -> None:
        interface, stage_id, body_path = self._physx_body_handle()
        interface.wake_up(stage_id, body_path)
