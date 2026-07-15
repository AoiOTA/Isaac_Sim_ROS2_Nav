"""Thin Jackal runtime adapter over Isaac Sim 6.0 experimental Articulation."""

from __future__ import annotations

from dataclasses import dataclass
import math
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
    solver_position_iterations: int
    solver_velocity_iterations: int
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
        "solver_position_iterations",
        "solver_velocity_iterations",
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

    def positive_integer(name: str) -> int:
        value = physics[name]
        if (
            isinstance(value, bool)
            or not isinstance(value, int)
            or not 1 <= value <= 255
        ):
            raise ArticulationRuntimeError(
                f"robot.physics.{name} must be an integer in [1, 255]"
            )
        return value

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
        solver_position_iterations=positive_integer(
            "solver_position_iterations"
        ),
        solver_velocity_iterations=positive_integer(
            "solver_velocity_iterations"
        ),
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
        self._initial_dof_positions: tuple[float, ...] | None = None

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
        self._initial_dof_positions = self._read_dof_positions(
            context="initial articulation state"
        )

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
    ) -> tuple[int, int]:
        self.articulation.set_solver_iteration_counts(
            [settings.solver_position_iterations],
            [settings.solver_velocity_iterations],
        )
        self.articulation.set_sleep_thresholds([settings.sleep_threshold])
        self.articulation.set_stabilization_thresholds(
            [settings.stabilization_threshold]
        )
        self.articulation.set_dof_friction_properties(
            static_frictions=[settings.wheel_static_friction_effort],
            dynamic_frictions=[settings.wheel_dynamic_friction_effort],
            viscous_frictions=[settings.wheel_viscous_friction_coefficient],
        )
        actual = self.get_solver_iteration_usd_values()
        expected = (
            settings.solver_position_iterations,
            settings.solver_velocity_iterations,
        )
        if actual != expected:
            raise ArticulationRuntimeError(
                "articulation USD solver readback does not match configuration: "
                f"expected={expected}, actual={actual}"
            )
        return actual

    def get_solver_iteration_usd_values(self) -> tuple[int, int]:
        """Read composed USD values through the initialized Articulation wrapper."""

        position, velocity = self.articulation.get_solver_iteration_counts()

        def single_count(values: object, name: str) -> int:
            flattened = values.numpy().reshape(-1)
            if len(flattened) != 1:
                raise ArticulationRuntimeError(
                    f"expected one {name} solver count, got {len(flattened)}"
                )
            value = int(flattened[0])
            if not 1 <= value <= 255:
                raise ArticulationRuntimeError(
                    f"articulation USD {name} solver count must be in [1, 255], "
                    f"got {value}"
                )
            return value

        return (
            single_count(position, "position"),
            single_count(velocity, "velocity"),
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

    def _read_dof_values(
        self,
        getter_name: str,
        *,
        context: str,
    ) -> tuple[float, ...]:
        values = getattr(self.articulation, getter_name)().numpy()
        if len(values) != 1:
            raise ArticulationRuntimeError(
                f"{context} expected one articulation, got {len(values)}"
            )
        row = values[0]
        if len(row) != self.num_dof:
            raise ArticulationRuntimeError(
                f"{context} expected {self.num_dof} DOF values, got {len(row)}"
            )
        result = tuple(float(value) for value in row)
        if not all(math.isfinite(value) for value in result):
            raise ArticulationRuntimeError(
                f"{context} contains a non-finite DOF value"
            )
        return result

    def _read_dof_positions(self, *, context: str) -> tuple[float, ...]:
        return self._read_dof_values(
            "get_dof_positions",
            context=context,
        )

    def restore_initial_joint_state(self) -> None:
        """Restore the initialized DOF pose and a zero dynamic state.

        Resetting every DOF to a numeric zero would corrupt custom robots whose
        authored rest pose is non-zero.  The initialization snapshot instead
        removes accumulated DOF pose as an uncontrolled reset variable while
        preserving the asset's complete articulation pose.
        """

        expected = self._initial_dof_positions
        if expected is None:
            raise ArticulationRuntimeError(
                "initial DOF positions are unavailable before initialization"
            )
        zeros = [0.0] * self.num_dof
        self.articulation.set_dof_positions([list(expected)])
        self.articulation.set_dof_velocities([zeros])
        self.articulation.set_dof_velocity_targets([zeros])
        self.articulation.set_dof_efforts([zeros])
        actual = self._read_dof_positions(context="restored articulation state")
        mismatches = [
            (index, expected_value, actual_value)
            for index, (expected_value, actual_value) in enumerate(
                zip(expected, actual, strict=True)
            )
            if not math.isclose(
                actual_value,
                expected_value,
                rel_tol=0.0,
                abs_tol=1e-6,
            )
        ]
        if mismatches:
            raise ArticulationRuntimeError(
                "restored DOF position readback does not match the initialized "
                f"state: {mismatches}"
            )
        for getter_name, label in (
            ("get_dof_velocities", "velocity"),
            ("get_dof_velocity_targets", "velocity target"),
            ("get_dof_efforts", "effort"),
        ):
            values = self._read_dof_values(
                getter_name,
                context=f"restored articulation {label}",
            )
            if any(abs(value) > 1e-6 for value in values):
                raise ArticulationRuntimeError(
                    f"restored DOF {label} readback is not zero: {values}"
                )

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


def author_articulation_solver_iterations(
    stage: object,
    articulation_root: str,
    settings: ArticulationPhysicsConfig,
) -> None:
    """Author configured solver USD attributes before PhysX parses the Stage."""

    from pxr import Sdf, UsdPhysics

    prim = stage.GetPrimAtPath(articulation_root)
    if not prim or not prim.IsValid() or not prim.HasAPI(
        UsdPhysics.ArticulationRootAPI
    ):
        raise ArticulationRuntimeError(
            "articulation root is invalid or lacks ArticulationRootAPI: "
            f"{articulation_root}"
        )
    values = (
        (
            "physxArticulation:solverPositionIterationCount",
            settings.solver_position_iterations,
        ),
        (
            "physxArticulation:solverVelocityIterationCount",
            settings.solver_velocity_iterations,
        ),
    )
    for name, value in values:
        attribute = prim.GetAttribute(name)
        if not attribute:
            attribute = prim.CreateAttribute(name, Sdf.ValueTypeNames.Int)
        elif attribute.GetTypeName() != Sdf.ValueTypeNames.Int:
            raise ArticulationRuntimeError(
                f"{name} on {articulation_root} must use USD int, got "
                f"{attribute.GetTypeName()}"
            )
        if not attribute.Set(value):
            raise ArticulationRuntimeError(
                f"failed to author {name}={value} on {articulation_root}"
            )
