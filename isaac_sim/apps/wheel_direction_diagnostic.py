#!/usr/bin/env python3
"""Run an exclusive, non-ROS four-wheel direction diagnostic in Isaac Sim.

Each wheel receives +1 and -1 rad/s while every other DOF target remains zero.
The application owns no ROS publisher and creates no OmniGraph controller.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, replace
from datetime import datetime, timezone
import math
import os
from pathlib import Path
import statistics
import sys
from typing import Any, Sequence


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from isaac_sim.apps.navigation_sim import (
    _simulation_app_config,
    validate_composed_stage,
)
from isaac_sim.src.config import (
    ProjectConfig,
    configure_process_environment,
    load_project_config,
)
from isaac_sim.src.robot.articulation_runtime import (
    ArticulationRuntime,
    load_articulation_physics_config,
)
from isaac_sim.src.robot.joint_validator import JointGroups, JointValidator
from isaac_sim.src.robot.spawn_pose_manager import SpawnPoseManager, load_spawn_poses
from isaac_sim.src.robot.wheel_direction_diagnostic import (
    TrialObservation,
    WheelDirectionConfig,
    WheelDirectionDiagnosticError,
    center_of_mass_world,
    contact_point_velocity_world,
    evaluate_trial_set,
    load_wheel_direction_config,
    rotate_world_to_local,
    spin_contact_velocity_world,
    summarize_trial,
    vector_subtract,
    write_json_atomic,
)
from isaac_sim.src.runtime_provenance import (
    capture_runtime_provenance,
    file_sha256,
)
from isaac_sim.src.stage.scene_composer import SceneComposer
from isaac_sim.src.stage.contact_setup import resolve_ground_colliders


DEFAULT_PROJECT_CONFIG = PROJECT_ROOT / "isaac_sim/configs/project.yaml"
DEFAULT_DIAGNOSTIC_CONFIG = (
    PROJECT_ROOT / "isaac_sim/configs/diagnostics/wheel_direction.yaml"
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the exclusive Isaac four-wheel +/- direction diagnostic"
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_PROJECT_CONFIG,
        help="project YAML loaded into Isaac Sim",
    )
    parser.add_argument(
        "--diagnostic-config",
        type=Path,
        default=DEFAULT_DIAGNOSTIC_CONFIG,
        help="strict direction protocol and threshold YAML",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="atomic JSON output path (default: timestamped data/reports/physics file)",
    )
    parser.add_argument(
        "--headless",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="run headless by default; use --no-headless for visual inspection",
    )
    parser.add_argument(
        "--pacing-mode",
        choices=("realtime", "unbounded"),
        default="unbounded",
        help="unbounded is safe because physics dt remains fixed",
    )
    return parser


def _output_path(requested: Path | None, environment_id: str) -> Path:
    if requested is not None:
        return requested if requested.is_absolute() else PROJECT_ROOT / requested
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return (
        PROJECT_ROOT
        / "data/reports/physics"
        / f"wheel_direction_{environment_id}_{stamp}.json"
    )


def _enable_runtime_extension(app: object) -> None:
    """Enable only the current public prim wrapper needed by this diagnostic."""

    import omni.kit.app

    extension_id = "isaacsim.core.experimental.prims"
    manager = omni.kit.app.get_app().get_extension_manager()
    manager.set_extension_enabled_immediate(extension_id, True)
    if not manager.is_extension_enabled(extension_id):
        raise WheelDirectionDiagnosticError(
            f"required Isaac extension could not be enabled: {extension_id}"
        )
    app.update()


def _wheel_stage_bindings(
    stage: object,
    config: ProjectConfig,
) -> list[dict[str, object]]:
    """Resolve configured wheel joints and rigid links from USD alone.

    This deliberately runs before PhysX is initialized so contact reporting can
    be authored on the rigid wheel links before tensor views parse the Stage.
    """

    from pxr import Usd, UsdPhysics

    root = stage.GetPrimAtPath(config.robot.articulation_root)
    if not root or not root.IsValid():
        raise WheelDirectionDiagnosticError("articulation root is invalid")
    bindings: list[dict[str, object]] = []
    for joint_name in config.robot.wheel_joints:
        matches = [
            prim
            for prim in Usd.PrimRange(root)
            if prim.GetName() == joint_name
            and prim.IsA(UsdPhysics.RevoluteJoint)
        ]
        if len(matches) != 1:
            raise WheelDirectionDiagnosticError(
                f"wheel joint {joint_name!r} did not resolve uniquely"
            )
        joint = UsdPhysics.Joint(matches[0])
        body0 = tuple(joint.GetBody0Rel().GetTargets())
        body1 = tuple(joint.GetBody1Rel().GetTargets())
        if len(body0) != 1 or len(body1) != 1:
            raise WheelDirectionDiagnosticError(
                f"wheel joint {joint_name!r} must have one Body0 and Body1"
            )
        base_path = config.robot.base_link_prim
        candidates = [path for path in body0 + body1 if str(path) != base_path]
        if len(candidates) != 1:
            raise WheelDirectionDiagnosticError(
                f"wheel joint {joint_name!r} does not connect one wheel to base_link"
            )
        bindings.append(
            {
                "joint_name": joint_name,
                "dof_path": str(matches[0].GetPath()),
                "wheel_link_path": str(candidates[0]),
            }
        )
    wheel_paths = [str(binding["wheel_link_path"]) for binding in bindings]
    if len(bindings) != 4 or len(set(wheel_paths)) != 4:
        raise WheelDirectionDiagnosticError(
            "direction diagnostic requires four unique wheel rigid links"
        )
    return bindings


def _wheel_bindings(
    stage_bindings: Sequence[dict[str, object]],
    config: ProjectConfig,
    robot: ArticulationRuntime,
) -> tuple[tuple[int, int, int, int], list[dict[str, object]]]:
    """Cross-check USD wheel bindings against live tensor DOFs."""

    dof_indices = JointValidator(
        config.robot.wheel_joints,
        JointGroups(
            config.robot.front_wheel_joints,
            config.robot.rear_wheel_joints,
        ),
    ).validate(robot.get_dof_names())
    runtime_dof_paths = robot.articulation.dof_paths[0]
    bindings: list[dict[str, object]] = []
    for stage_binding, dof_index in zip(stage_bindings, dof_indices):
        joint_name = str(stage_binding["joint_name"])
        dof_path = str(runtime_dof_paths[dof_index])
        if dof_path != str(stage_binding["dof_path"]):
            raise WheelDirectionDiagnosticError(
                f"runtime DOF path mismatch for {joint_name}: "
                f"tensor={dof_path}, stage={stage_binding['dof_path']}"
            )
        binding = dict(stage_binding)
        binding["dof_index"] = dof_index
        bindings.append(binding)
    return dof_indices, bindings


def _author_wheel_contact_reports(
    stage: object,
    bindings: Sequence[dict[str, object]],
) -> list[dict[str, object]]:
    """Enable zero-threshold PhysX reports in the anonymous Session Layer."""

    from pxr import PhysxSchema, UsdPhysics

    original_target = stage.GetEditTarget()
    snapshots: list[dict[str, object]] = []
    try:
        # Never dirty the committed robot USD or generated project Stage.
        stage.SetEditTarget(stage.GetSessionLayer())
        for binding in bindings:
            path = str(binding["wheel_link_path"])
            prim = stage.GetPrimAtPath(path)
            if (
                not prim
                or not prim.IsValid()
                or not prim.IsActive()
                or not prim.HasAPI(UsdPhysics.RigidBodyAPI)
            ):
                raise WheelDirectionDiagnosticError(
                    f"wheel contact-report prim is not an active rigid body: {path}"
                )
            report_api = PhysxSchema.PhysxContactReportAPI.Apply(prim)
            if not report_api:
                raise WheelDirectionDiagnosticError(
                    f"failed to apply PhysxContactReportAPI: {path}"
                )
            threshold = report_api.CreateThresholdAttr()
            if not threshold.Set(0.0):
                raise WheelDirectionDiagnosticError(
                    f"failed to author zero contact-report threshold: {path}"
                )
            snapshots.append(
                {
                    "wheel_link_path": path,
                    "physx_contact_report_api": prim.HasAPI(
                        PhysxSchema.PhysxContactReportAPI
                    ),
                    "threshold_n": float(threshold.Get()),
                    "authored_in_anonymous_session_layer": True,
                }
            )
    finally:
        stage.SetEditTarget(original_target)
    if len(snapshots) != 4 or not all(
        snapshot["physx_contact_report_api"] is True
        and snapshot["threshold_n"] == 0.0
        for snapshot in snapshots
    ):
        raise WheelDirectionDiagnosticError(
            "four wheel contact-report APIs did not read back exactly"
        )
    return snapshots


def _validate_ground_collision(stage: object, path: str) -> dict[str, object]:
    from pxr import UsdPhysics

    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid() or not prim.IsActive():
        raise WheelDirectionDiagnosticError(
            f"configured ground collision prim is invalid or inactive: {path}"
        )
    if not prim.HasAPI(UsdPhysics.CollisionAPI):
        raise WheelDirectionDiagnosticError(
            f"configured ground prim lacks CollisionAPI: {path}"
        )
    enabled = UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
    if enabled is False:
        raise WheelDirectionDiagnosticError(
            f"configured ground collision is disabled: {path}"
        )
    return {
        "path": path,
        "type": prim.GetTypeName(),
        "collision_enabled": enabled is not False,
    }


def _physics_material_snapshot(stage: object, prim_path: str) -> dict[str, object]:
    """Read the effective bound USD PhysicsMaterial without changing the Stage."""

    from pxr import UsdPhysics, UsdShade

    prim = stage.GetPrimAtPath(prim_path)
    binding_api = UsdShade.MaterialBindingAPI(prim)
    # Physics-purpose bindings must win over an inherited/allPurpose visual or
    # legacy material.  Assets that only carry an allPurpose PhysicsMaterial
    # remain supported as an explicit fallback.
    material, relationship = binding_api.ComputeBoundMaterial("physics")
    if not material or not material.GetPrim().IsValid():
        material, relationship = binding_api.ComputeBoundMaterial()
    if not material:
        return {
            "bound": False,
            "path": None,
            "binding_relationship": None,
            "binding_purpose": None,
            "has_physics_material_api": False,
            "properties": {},
        }
    material_prim = material.GetPrim()
    relationship_path = (
        str(relationship.GetPath())
        if relationship and relationship.IsValid()
        else None
    )
    binding_purpose = (
        "physics"
        if relationship_path and relationship_path.endswith(":physics")
        else "allPurpose"
    )
    properties: dict[str, object] = {}
    for name in (
        "physics:staticFriction",
        "physics:dynamicFriction",
        "physics:restitution",
        "physxMaterial:frictionCombineMode",
        "physxMaterial:restitutionCombineMode",
    ):
        attribute = material_prim.GetAttribute(name)
        if attribute and attribute.IsValid() and attribute.HasAuthoredValueOpinion():
            value = attribute.Get()
            properties[name] = str(value) if not isinstance(value, (bool, int, float, str)) else value
    return {
        "bound": True,
        "path": str(material.GetPath()),
        "binding_relationship": relationship_path,
        "binding_purpose": binding_purpose,
        "has_physics_material_api": material_prim.HasAPI(
            UsdPhysics.MaterialAPI
        ),
        "properties": properties,
    }


def _wheel_collider_snapshot(
    stage: object, bindings: Sequence[dict[str, object]]
) -> list[dict[str, object]]:
    from pxr import Usd, UsdPhysics

    snapshots: list[dict[str, object]] = []
    for binding in bindings:
        wheel_path = str(binding["wheel_link_path"])
        wheel = stage.GetPrimAtPath(wheel_path)
        colliders = []
        for prim in Usd.PrimRange(wheel):
            if not prim.IsActive() or not prim.HasAPI(UsdPhysics.CollisionAPI):
                continue
            collision_enabled = UsdPhysics.CollisionAPI(
                prim
            ).GetCollisionEnabledAttr().Get()
            if collision_enabled is False:
                continue
            colliders.append(prim)
        if len(colliders) != 1:
            raise WheelDirectionDiagnosticError(
                f"wheel {wheel_path} must have exactly one active collider; got "
                f"{[str(prim.GetPath()) for prim in colliders]}"
            )
        collider = colliders[0]
        snapshots.append(
            {
                "joint_name": binding["joint_name"],
                "wheel_link_path": wheel_path,
                "collider_path": str(collider.GetPath()),
                "collider_type": collider.GetTypeName(),
                "physics_material": _physics_material_snapshot(
                    stage, str(collider.GetPath())
                ),
            }
        )
    return snapshots


def _to_float_tuple(values: Any, size: int) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != size or not all(math.isfinite(value) for value in result):
        raise WheelDirectionDiagnosticError(
            f"runtime tensor must contain {size} finite values"
        )
    return result


def _vector_norm(vector: Sequence[float]) -> float:
    return math.sqrt(sum(float(value) ** 2 for value in vector))


def _vector_sum(vectors: Sequence[Sequence[float]]) -> tuple[float, float, float]:
    return tuple(sum(float(vector[index]) for vector in vectors) for index in range(3))  # type: ignore[return-value]


def _contact_slice(
    values: Any,
    counts: Any,
    starts: Any,
    wheel_index: int,
) -> Any:
    """Concatenate one sensor's contact rows across every ground filter."""

    import numpy as np

    if counts.ndim != 2 or starts.shape != counts.shape:
        raise WheelDirectionDiagnosticError(
            "contact counts and starts must have matching sensor/filter shape"
        )
    parts = []
    total = 0
    for filter_index in range(counts.shape[1]):
        count = int(counts[wheel_index, filter_index])
        start = int(starts[wheel_index, filter_index])
        if count < 0 or start < 0 or start + count > len(values):
            raise WheelDirectionDiagnosticError(
                "contact tensor range is outside the detailed-data buffer"
            )
        if count:
            parts.append(values[start : start + count])
            total += count
    if not parts:
        return values[0:0], 0
    return np.concatenate(parts, axis=0), total


def _root_pose_and_velocity(robot: ArticulationRuntime) -> tuple[Any, Any, Any, Any]:
    positions, orientations = robot.articulation.get_world_poses()
    linear, angular = robot.articulation.get_velocities()
    return (
        positions.numpy()[0].copy(),
        orientations.numpy()[0].copy(),
        linear.numpy()[0].copy(),
        angular.numpy()[0].copy(),
    )


def _sample(
    *,
    robot: ArticulationRuntime,
    wheel_view: object,
    wheel_dof_indices: Sequence[int],
    bindings: Sequence[dict[str, object]],
    active_wheel_index: int,
    phase: str,
    step_index: int,
    physics_dt_s: float,
    trial_origin_world: Sequence[float],
    trial_orientation_world_wxyz: Sequence[float],
    previous_linear_velocity_world: Sequence[float] | None,
    previous_simulation_time_s: float | None,
    total_mass_kg: float,
) -> tuple[TrialObservation, dict[str, object], tuple[float, float, float], float]:
    """Read one coherent post-step tensor/contact snapshot."""

    import numpy as np
    from isaacsim.core.simulation_manager import SimulationManager

    articulation = robot.articulation
    simulation_time_s = float(SimulationManager.get_simulation_time())
    target_values = articulation.get_dof_velocity_targets(
        dof_indices=list(wheel_dof_indices)
    ).numpy()[0]
    rate_values = articulation.get_dof_velocities(
        dof_indices=list(wheel_dof_indices)
    ).numpy()[0]
    effort_values = articulation.get_dof_efforts(
        dof_indices=list(wheel_dof_indices)
    ).numpy()[0]
    joint_targets = _to_float_tuple(target_values, 4)
    joint_rates = _to_float_tuple(rate_values, 4)
    joint_efforts = _to_float_tuple(effort_values, 4)

    root_position, root_orientation, root_linear, root_angular = _root_pose_and_velocity(robot)
    root_position_tuple = _to_float_tuple(root_position, 3)
    root_orientation_tuple = _to_float_tuple(root_orientation, 4)
    root_linear_tuple = _to_float_tuple(root_linear, 3)
    root_angular_tuple = _to_float_tuple(root_angular, 3)
    base_position_x = rotate_world_to_local(
        vector_subtract(root_position_tuple, trial_origin_world),
        trial_orientation_world_wxyz,
    )[0]
    base_velocity_trial = rotate_world_to_local(
        root_linear_tuple, trial_orientation_world_wxyz
    )
    base_acceleration_trial: tuple[float, float, float] | None = None
    if previous_linear_velocity_world is not None and previous_simulation_time_s is not None:
        elapsed = simulation_time_s - previous_simulation_time_s
        if elapsed <= 0.0 or abs(elapsed - physics_dt_s) > max(1e-6, physics_dt_s * 0.10):
            raise WheelDirectionDiagnosticError(
                f"non-fixed physics sample interval: expected={physics_dt_s}, actual={elapsed}"
            )
        acceleration_world = tuple(
            (root_linear_tuple[index] - float(previous_linear_velocity_world[index]))
            / elapsed
            for index in range(3)
        )
        base_acceleration_trial = rotate_world_to_local(
            acceleration_world, trial_orientation_world_wxyz
        )

    wheel_positions, wheel_orientations = wheel_view.get_world_poses()
    wheel_com_positions, _ = wheel_view.get_coms()
    wheel_linear_velocities, wheel_angular_velocities = wheel_view.get_velocities()
    wheel_positions_np = wheel_positions.numpy().copy()
    wheel_orientations_np = wheel_orientations.numpy().copy()
    wheel_com_positions_np = wheel_com_positions.numpy().copy()
    wheel_linear_np = wheel_linear_velocities.numpy().copy()
    wheel_angular_np = wheel_angular_velocities.numpy().copy()

    normal_magnitudes, normal_points, normal_vectors, separations, normal_counts, normal_starts = (
        wheel_view.get_contact_force_data(dt=physics_dt_s)
    )
    friction_forces, friction_points, friction_counts, friction_starts = (
        wheel_view.get_friction_data(dt=physics_dt_s)
    )
    ground_contact_force_matrix = wheel_view.get_contact_force_matrix(
        dt=physics_dt_s
    )
    normal_magnitudes_np = normal_magnitudes.numpy().copy().reshape(-1)
    normal_points_np = normal_points.numpy().copy()
    normal_vectors_np = normal_vectors.numpy().copy()
    separations_np = separations.numpy().copy().reshape(-1)
    normal_counts_np = normal_counts.numpy().copy()
    normal_starts_np = normal_starts.numpy().copy()
    friction_forces_np = friction_forces.numpy().copy()
    friction_points_np = friction_points.numpy().copy()
    friction_counts_np = friction_counts.numpy().copy()
    friction_starts_np = friction_starts.numpy().copy()
    matrix_normal_force_np = (
        ground_contact_force_matrix.numpy().copy().sum(axis=1)
    )

    contact_reports: list[dict[str, object]] = []
    active_contact_count = 0
    active_normal_force_n = 0.0
    active_spin_values: list[float] = []
    active_surface_values: list[float] = []
    active_friction_x = 0.0
    active_normal_force_consistency_error: float | None = None
    body_normal_vectors_trial: list[tuple[float, float, float]] = []
    body_friction_vectors_trial: list[tuple[float, float, float]] = []

    for wheel_index, binding in enumerate(bindings):
        normal_slice, normal_count = _contact_slice(
            normal_magnitudes_np, normal_counts_np, normal_starts_np, wheel_index
        )
        normal_vector_slice, _ = _contact_slice(
            normal_vectors_np, normal_counts_np, normal_starts_np, wheel_index
        )
        normal_point_slice, _ = _contact_slice(
            normal_points_np, normal_counts_np, normal_starts_np, wheel_index
        )
        separation_slice, _ = _contact_slice(
            separations_np, normal_counts_np, normal_starts_np, wheel_index
        )
        normal_force_vectors_world = [
            tuple(float(normal_slice[index]) * float(value) for value in normal_vector_slice[index])
            for index in range(normal_count)
        ]
        normal_force_world = (
            _vector_sum(normal_force_vectors_world)
            if normal_force_vectors_world
            else (0.0, 0.0, 0.0)
        )
        normal_force_trial = rotate_world_to_local(
            normal_force_world, trial_orientation_world_wxyz
        )

        friction_force_slice, friction_count = _contact_slice(
            friction_forces_np, friction_counts_np, friction_starts_np, wheel_index
        )
        friction_point_slice, _ = _contact_slice(
            friction_points_np, friction_counts_np, friction_starts_np, wheel_index
        )
        friction_force_world = (
            _vector_sum(friction_force_slice)
            if friction_count
            else (0.0, 0.0, 0.0)
        )
        friction_force_trial = rotate_world_to_local(
            friction_force_world, trial_orientation_world_wxyz
        )
        matrix_normal_force_world = _to_float_tuple(
            matrix_normal_force_np[wheel_index], 3
        )
        normal_force_consistency_error = _vector_norm(
            vector_subtract(
                normal_force_world, matrix_normal_force_world
            )
        )

        wheel_position = _to_float_tuple(wheel_positions_np[wheel_index], 3)
        wheel_orientation = _to_float_tuple(wheel_orientations_np[wheel_index], 4)
        wheel_com_local = _to_float_tuple(wheel_com_positions_np[wheel_index], 3)
        wheel_com_world = center_of_mass_world(
            wheel_position, wheel_orientation, wheel_com_local
        )
        wheel_linear_world = _to_float_tuple(wheel_linear_np[wheel_index], 3)
        wheel_angular_world = _to_float_tuple(wheel_angular_np[wheel_index], 3)
        friction_contacts: list[dict[str, object]] = []
        spin_values: list[float] = []
        surface_values: list[float] = []
        for contact_index in range(friction_count):
            point_world = _to_float_tuple(friction_point_slice[contact_index], 3)
            force_world = _to_float_tuple(friction_force_slice[contact_index], 3)
            surface_world = contact_point_velocity_world(
                wheel_linear_world,
                wheel_angular_world,
                point_world,
                wheel_com_world,
            )
            spin_world = spin_contact_velocity_world(
                wheel_angular_world,
                root_angular_tuple,
                point_world,
                wheel_com_world,
            )
            surface_trial = rotate_world_to_local(
                surface_world, trial_orientation_world_wxyz
            )
            spin_trial = rotate_world_to_local(
                spin_world, trial_orientation_world_wxyz
            )
            force_trial = rotate_world_to_local(
                force_world, trial_orientation_world_wxyz
            )
            surface_values.append(surface_trial[0])
            spin_values.append(spin_trial[0])
            friction_contacts.append(
                {
                    "point_world_m": list(point_world),
                    "force_trial_frame_n": list(force_trial),
                    "surface_velocity_trial_frame_m_s": list(surface_trial),
                    "spin_velocity_trial_frame_m_s": list(spin_trial),
                }
            )
        contact_reports.append(
            {
                "joint_name": binding["joint_name"],
                "wheel_link_path": binding["wheel_link_path"],
                "normal_contact_count": normal_count,
                "normal_force_trial_frame_n": list(normal_force_trial),
                "normal_points_world_m": [
                    [float(value) for value in point]
                    for point in normal_point_slice
                ],
                "normal_separations_m": [float(value) for value in separation_slice],
                "friction_contact_count": friction_count,
                "friction_force_trial_frame_n": list(friction_force_trial),
                "normal_force_matrix_trial_frame_n": list(
                    rotate_world_to_local(
                        matrix_normal_force_world,
                        trial_orientation_world_wxyz,
                    )
                ),
                "normal_force_reconstruction_error_n": (
                    normal_force_consistency_error
                ),
                "friction_contacts": friction_contacts,
            }
        )
        body_normal_vectors_trial.append(normal_force_trial)
        body_friction_vectors_trial.append(friction_force_trial)
        if wheel_index == active_wheel_index:
            active_contact_count = normal_count
            active_normal_force_n = _vector_norm(normal_force_world)
            active_spin_values = spin_values
            active_surface_values = surface_values
            active_friction_x = friction_force_trial[0]
            active_normal_force_consistency_error = (
                normal_force_consistency_error
            )

    active_spin_x = (
        float(statistics.median(active_spin_values)) if active_spin_values else None
    )
    active_surface_x = (
        float(statistics.median(active_surface_values))
        if active_surface_values
        else None
    )
    observation = TrialObservation(
        phase=phase,
        step_index=step_index,
        simulation_time_s=simulation_time_s,
        joint_targets_rad_s=joint_targets,  # type: ignore[arg-type]
        joint_rates_rad_s=joint_rates,  # type: ignore[arg-type]
        active_contact_count=active_contact_count,
        active_normal_force_n=active_normal_force_n,
        active_spin_velocity_x_m_s=active_spin_x,
        active_surface_velocity_x_m_s=active_surface_x,
        active_friction_force_x_n=active_friction_x,
        normal_force_consistency_error_n=(
            active_normal_force_consistency_error
        ),
        base_position_x_m=base_position_x,
        base_velocity_x_m_s=base_velocity_trial[0],
        base_acceleration_x_m_s2=(
            base_acceleration_trial[0]
            if base_acceleration_trial is not None
            else None
        ),
    )
    body_normal = _vector_sum(body_normal_vectors_trial)
    body_friction = _vector_sum(body_friction_vectors_trial)
    sample_report: dict[str, object] = {
        "phase": phase,
        "step_index": step_index,
        "simulation_time_s": simulation_time_s,
        "joint_targets_rad_s": list(joint_targets),
        "joint_rates_rad_s": list(joint_rates),
        "joint_actuation_efforts_nm": list(joint_efforts),
        "base": {
            "position_world_m": list(root_position_tuple),
            "orientation_world_wxyz": list(root_orientation_tuple),
            "position_x_trial_frame_m": base_position_x,
            "linear_velocity_trial_frame_m_s": list(base_velocity_trial),
            "angular_velocity_world_rad_s": list(root_angular_tuple),
            "linear_acceleration_trial_frame_m_s2": (
                list(base_acceleration_trial)
                if base_acceleration_trial is not None
                else None
            ),
            "mass_times_acceleration_trial_frame_n": (
                [total_mass_kg * value for value in base_acceleration_trial]
                if base_acceleration_trial is not None
                else None
            ),
        },
        "body_ground_normal_force_trial_frame_n": list(body_normal),
        "body_ground_friction_force_trial_frame_n": list(body_friction),
        "wheel_contacts": contact_reports,
        "direction_metrics": observation.to_json(),
    }
    return observation, sample_report, root_linear_tuple, simulation_time_s


def _set_single_wheel_target(
    robot: ArticulationRuntime,
    *,
    active_dof_index: int | None,
    command_rad_s: float,
) -> None:
    targets = [0.0] * robot.num_dof
    if active_dof_index is not None:
        targets[active_dof_index] = command_rad_s
    robot.set_joint_velocity_targets(targets)


def _reset_to_spawn(
    runtime: object,
    spawn_manager: SpawnPoseManager,
    pose_name: str,
) -> None:
    runtime.pause()
    spawn_manager.apply_usd_pose(pose_name)
    runtime.step(render=False)
    runtime.play()


def _wait_for_all_wheel_contacts(
    *,
    runtime: object,
    wheel_view: object,
    physics_dt_s: float,
    consecutive_required: int,
    timeout_steps: int,
) -> int:
    consecutive = 0
    for step in range(1, timeout_steps + 1):
        runtime.update()
        _, _, _, _, counts, _ = wheel_view.get_contact_force_data(
            dt=physics_dt_s
        )
        count_values = counts.numpy()
        all_contacting = all(int(count_values[index].sum()) > 0 for index in range(4))
        consecutive = consecutive + 1 if all_contacting else 0
        if consecutive >= consecutive_required:
            return step
    raise WheelDirectionDiagnosticError(
        "four-wheel ground contact did not become continuously ready within "
        f"{timeout_steps} physics steps"
    )


def _contact_partner_paths(wheel_view: object, physics_dt_s: float) -> list[str]:
    """Resolve raw other-actor IDs through the public contact-view API."""

    import numpy as np
    import warp as wp

    _, _, _, _, counts, starts, actor_ids = wheel_view.get_raw_contact_data(
        dt=physics_dt_s
    )
    counts_np = counts.numpy().copy()
    starts_np = starts.numpy().copy()
    actor_ids_np = actor_ids.numpy().copy()
    used_ids: list[int] = []
    for wheel_index in range(4):
        start = int(starts_np[wheel_index])
        count = int(counts_np[wheel_index])
        used_ids.extend(int(value) for value in actor_ids_np[start : start + count])
    if not used_ids:
        return []
    ids_cpu = wp.array(np.asarray(used_ids, dtype=np.uint64), dtype=wp.uint64, device="cpu")
    return sorted({path for path in wheel_view.get_actor_paths_from_ids(ids_cpu) if path})


def _run_trial(
    *,
    config: ProjectConfig,
    diagnostic: WheelDirectionConfig,
    runtime: object,
    robot: ArticulationRuntime,
    spawn_manager: SpawnPoseManager,
    wheel_view: object,
    wheel_dof_indices: Sequence[int],
    bindings: Sequence[dict[str, object]],
    active_wheel_index: int,
    command_rad_s: float,
    physics_dt_s: float,
    total_mass_kg: float,
) -> dict[str, object]:
    protocol = diagnostic.protocol
    _reset_to_spawn(runtime, spawn_manager, config.spawn.selected)
    _set_single_wheel_target(robot, active_dof_index=None, command_rad_s=0.0)
    for _ in range(protocol.settle_steps):
        runtime.update()
    origin_position, origin_orientation, _, _ = _root_pose_and_velocity(robot)
    trial_origin = _to_float_tuple(origin_position, 3)
    trial_orientation = _to_float_tuple(origin_orientation, 4)
    observations: list[TrialObservation] = []
    sample_reports: list[dict[str, object]] = []
    previous_velocity: tuple[float, float, float] | None = None
    previous_time: float | None = None
    step_index = 0

    def collect(phase: str, count: int) -> None:
        nonlocal previous_velocity, previous_time, step_index
        for _ in range(count):
            runtime.update()
            observation, sample_report, previous_velocity, previous_time = _sample(
                robot=robot,
                wheel_view=wheel_view,
                wheel_dof_indices=wheel_dof_indices,
                bindings=bindings,
                active_wheel_index=active_wheel_index,
                phase=phase,
                step_index=step_index,
                physics_dt_s=physics_dt_s,
                trial_origin_world=trial_origin,
                trial_orientation_world_wxyz=trial_orientation,
                previous_linear_velocity_world=previous_velocity,
                previous_simulation_time_s=previous_time,
                total_mass_kg=total_mass_kg,
            )
            observations.append(observation)
            sample_reports.append(sample_report)
            step_index += 1

    collect("baseline", protocol.baseline_steps)
    _set_single_wheel_target(
        robot,
        active_dof_index=wheel_dof_indices[active_wheel_index],
        command_rad_s=command_rad_s,
    )
    collect("drive", protocol.drive_steps)
    _set_single_wheel_target(robot, active_dof_index=None, command_rad_s=0.0)
    collect("recovery", protocol.recovery_steps)
    summary = summarize_trial(
        observations,
        wheel_name=str(bindings[active_wheel_index]["joint_name"]),
        wheel_index=active_wheel_index,
        command_rad_s=command_rad_s,
        physics_dt_s=physics_dt_s,
        thresholds=diagnostic.thresholds,
    )
    return {
        "trial_id": (
            f"{bindings[active_wheel_index]['joint_name']}_"
            f"{'positive' if command_rad_s > 0.0 else 'negative'}"
        ),
        "wheel": bindings[active_wheel_index]["joint_name"],
        "wheel_index": active_wheel_index,
        "command_rad_s": command_rad_s,
        "trial_start_world": {
            "position_m": list(trial_origin),
            "orientation_wxyz": list(trial_orientation),
        },
        "samples": sample_reports,
        "summary": summary,
    }


def run(
    config: ProjectConfig,
    diagnostic: WheelDirectionConfig,
    *,
    project_config_path: Path,
    diagnostic_config_path: Path,
    output_path: Path,
) -> dict[str, object]:
    """Run Isaac and always atomically publish a strict success/failure report."""

    report: dict[str, object] = {
        "schema_version": 1,
        "diagnostic": "single_wheel_direction",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "result": "failure",
        "failure": None,
        "inputs": {
            "project_config": {
                "path": str(project_config_path.resolve()),
                "sha256": file_sha256(project_config_path),
            },
            "diagnostic_config": {
                "path": str(diagnostic_config_path.resolve()),
                "sha256": file_sha256(diagnostic_config_path),
            },
        },
        "protocol": asdict(diagnostic.protocol),
        "thresholds": asdict(diagnostic.thresholds),
        "frame_conventions": {
            "world": "USD world, metres, Z-up",
            "trial_frame": "robot base axes frozen at each trial start",
            "positive_x_contract": (
                "positive wheel command has negative bottom-surface spin X, "
                "positive ground-friction X, and positive chassis motion X"
            ),
            "contact_force": "force acting on the wheel sensor prim",
        },
    }
    app = None
    runtime = None
    try:
        configure_process_environment(config)
        from isaacsim import SimulationApp

        original_argv = sys.argv[:]
        try:
            sys.argv = [sys.argv[0]]
            app = SimulationApp(_simulation_app_config(config))
        finally:
            sys.argv = original_argv
        _enable_runtime_extension(app)

        from isaac_sim.src.stage.physics_setup import PhysicsSetup, prepare_pacing
        from isaacsim.core.simulation_manager import SimulationManager
        from isaacsim.core.experimental.prims import RigidPrim

        prepare_pacing(config.simulation)
        composer = SceneComposer(config)
        stage = composer.compose(save=False)
        stage_bindings = _wheel_stage_bindings(stage, config)
        ground_paths = resolve_ground_colliders(stage, config)
        if diagnostic.ground_collision_prim not in ground_paths:
            raise WheelDirectionDiagnosticError(
                "diagnostic ground anchor is absent from the effective ground "
                f"collider set: anchor={diagnostic.ground_collision_prim}, "
                f"resolved={list(ground_paths)}"
            )
        ground_snapshots = [
            _validate_ground_collision(stage, path) for path in ground_paths
        ]
        contact_report_snapshots = _author_wheel_contact_reports(
            stage, stage_bindings
        )
        runtime = PhysicsSetup(config.simulation).apply(stage, app)
        app.update()
        validate_composed_stage(config, stage)
        settings = load_articulation_physics_config(config.files.robot)
        runtime.reset()

        robot = ArticulationRuntime(
            config.robot.articulation_root,
            config.robot.base_link_prim,
            app,
        )
        robot.initialize()
        solver_readback = robot.configure_stability(settings)
        wheel_dof_indices, bindings = _wheel_bindings(
            stage_bindings, config, robot
        )
        report["runtime_provenance"] = capture_runtime_provenance(
            config,
            stage,
            articulation_usd_solver_iterations=solver_readback,
            repository_root=PROJECT_ROOT,
        )
        report["bindings"] = {
            "articulation_root": config.robot.articulation_root,
            "base_link_prim": config.robot.base_link_prim,
            "wheel_order": list(config.robot.wheel_joints),
            "wheels": bindings,
            "ground_collision_anchor": diagnostic.ground_collision_prim,
            "ground_collisions": ground_snapshots,
            "ground_physics_material": _physics_material_snapshot(
                stage, diagnostic.ground_collision_prim
            ),
            "wheel_colliders": _wheel_collider_snapshot(stage, bindings),
            "wheel_contact_reports": contact_report_snapshots,
            "contact_profile": (
                composer.contact_snapshot.to_dict()
                if composer.contact_snapshot is not None
                else None
            ),
        }

        wheel_paths = [str(binding["wheel_link_path"]) for binding in bindings]
        wheel_view = RigidPrim(
            wheel_paths,
            contact_filter_paths=list(ground_paths),
            max_contact_count=diagnostic.protocol.max_contact_count,
        )
        app.update()
        if not wheel_view.is_physics_tensor_entity_valid():
            raise WheelDirectionDiagnosticError(
                "RigidPrim wheel tensor view is invalid after physics warmup"
            )
        if wheel_view.num_contact_filters != len(ground_paths):
            raise WheelDirectionDiagnosticError(
                "wheel contact view filter count mismatch: "
                f"expected={len(ground_paths)}, "
                f"actual={wheel_view.num_contact_filters}"
            )
        physics_dt_s = float(SimulationManager.get_physics_dt())
        expected_dt = 1.0 / config.simulation.physics_hz
        if abs(physics_dt_s - expected_dt) > 1e-9:
            raise WheelDirectionDiagnosticError(
                f"physics dt mismatch: expected={expected_dt}, actual={physics_dt_s}"
            )
        link_masses = robot.articulation.get_link_masses().numpy()[0]
        total_mass_kg = float(link_masses.sum())
        if not math.isfinite(total_mass_kg) or total_mass_kg <= 0.0:
            raise WheelDirectionDiagnosticError(
                f"articulation total mass is invalid: {total_mass_kg}"
            )
        report["runtime_measurement"] = {
            "physics_dt_s": physics_dt_s,
            "physics_hz": config.simulation.physics_hz,
            "articulation_total_mass_kg": total_mass_kg,
            "contact_force_dt_argument_s": physics_dt_s,
            "contact_force_units": "N (not default N*s impulse)",
            "ground_contact_filter_count": len(ground_paths),
        }

        poses = load_spawn_poses(config.spawn.poses_file)
        if config.spawn.selected not in poses:
            raise WheelDirectionDiagnosticError(
                f"selected spawn pose is absent: {config.spawn.selected}"
            )
        spawn_manager = SpawnPoseManager(robot, poses)
        _reset_to_spawn(runtime, spawn_manager, config.spawn.selected)
        contact_ready_steps = _wait_for_all_wheel_contacts(
            runtime=runtime,
            wheel_view=wheel_view,
            physics_dt_s=physics_dt_s,
            consecutive_required=(
                diagnostic.protocol.contact_ready_consecutive_steps
            ),
            timeout_steps=diagnostic.protocol.contact_ready_timeout_steps,
        )
        contact_partners = _contact_partner_paths(wheel_view, physics_dt_s)
        ground_actor_found = any(
            partner == ground.rstrip("/")
            or partner.startswith(f"{ground.rstrip('/')}/")
            or ground.rstrip("/").startswith(f"{partner.rstrip('/')}/")
            for partner in contact_partners
            for ground in ground_paths
        )
        if not ground_actor_found:
            raise WheelDirectionDiagnosticError(
                "raw contact actor IDs do not include any resolved ground: "
                f"resolved={list(ground_paths)}, actual={contact_partners}"
            )
        report["contact_readiness"] = {
            "steps": contact_ready_steps,
            "required_consecutive_steps": (
                diagnostic.protocol.contact_ready_consecutive_steps
            ),
            "ground_filter_paths": list(ground_paths),
            "raw_contact_partner_paths": contact_partners,
        }

        trials: list[dict[str, object]] = []
        command_magnitude = diagnostic.protocol.command_rad_s
        for wheel_index in range(4):
            for command in (command_magnitude, -command_magnitude):
                trials.append(
                    _run_trial(
                        config=config,
                        diagnostic=diagnostic,
                        runtime=runtime,
                        robot=robot,
                        spawn_manager=spawn_manager,
                        wheel_view=wheel_view,
                        wheel_dof_indices=wheel_dof_indices,
                        bindings=bindings,
                        active_wheel_index=wheel_index,
                        command_rad_s=command,
                        physics_dt_s=physics_dt_s,
                        total_mass_kg=total_mass_kg,
                    )
                )
        summaries = [trial["summary"] for trial in trials]
        cross_trial = evaluate_trial_set(
            summaries,  # type: ignore[arg-type]
            wheel_order=config.robot.wheel_joints,
            thresholds=diagnostic.thresholds,
        )
        report["trials"] = trials
        report["cross_trial"] = cross_trial
        if cross_trial["passed"] is True:
            report["result"] = "success"
        else:
            report["failure"] = {
                "type": "DirectionGateFailure",
                "message": f"failed trials: {cross_trial['failed_trials']}",
            }
        runtime.pause()
        robot.zero_all_velocities()
    except Exception as exc:
        report["result"] = "failure"
        report["failure"] = {
            "type": type(exc).__name__,
            "message": str(exc),
        }
    finally:
        if runtime is not None:
            try:
                runtime.stop()
            except Exception as exc:
                report["cleanup_warning"] = str(exc)
        # SimulationApp defaults to fast shutdown and can terminate the Python
        # process inside close().  The report and console handoff therefore
        # must be durable before close(), and close() must receive the real
        # result as its process exit status.
        write_json_atomic(output_path, report)
        print(
            f"wheel direction diagnostic: {report['result']}; "
            f"report={output_path}",
            flush=True,
        )
        if app is not None:
            try:
                app.close(
                    exit_code=0 if report["result"] == "success" else 1
                )
            except Exception as exc:
                report["app_close_warning"] = str(exc)
                write_json_atomic(output_path, report)
    return report


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    project_config_path = args.config.expanduser().resolve()
    diagnostic_config_path = args.diagnostic_config.expanduser().resolve()
    diagnostic = load_wheel_direction_config(diagnostic_config_path)
    config = load_project_config(project_config_path)
    config.require_runtime_paths()
    if config.environment.identifier != diagnostic.environment_id:
        raise WheelDirectionDiagnosticError(
            "diagnostic environment does not match project environment: "
            f"diagnostic={diagnostic.environment_id}, "
            f"project={config.environment.identifier}"
        )
    config = replace(
        config,
        simulation=replace(
            config.simulation,
            headless=bool(args.headless),
            pacing_mode=args.pacing_mode,
            max_frames=0,
        ),
    )
    output_path = _output_path(args.output, diagnostic.environment_id)
    report = run(
        config,
        diagnostic,
        project_config_path=project_config_path,
        diagnostic_config_path=diagnostic_config_path,
        output_path=output_path,
    )
    return 0 if report["result"] == "success" else 1


if __name__ == "__main__":
    raise SystemExit(main())
