"""Compose the project Stage using an environment Sublayer and robot Reference."""

from __future__ import annotations

from pathlib import Path

from isaac_sim.src.config import ProjectConfig
from isaac_sim.src.robot.articulation_runtime import (
    author_articulation_solver_iterations,
    load_articulation_physics_config,
)
from isaac_sim.src.robot.mass_collision_runtime import (
    apply_mass_collision_profile,
)
from isaac_sim.src.robot.spawn_pose_manager import (
    author_initial_articulation_pose,
    load_spawn_poses,
)
from isaac_sim.src.robot.wheel_velocity_drive import (
    apply_wheel_velocity_drive,
)
from isaac_sim.src.stage.asset_validator import dependency_report, validate_default_prim, validate_prim
from isaac_sim.src.stage.contact_setup import apply_contact_profile
from isaac_sim.src.stage.ground_topology import apply_ground_topology
from isaac_sim.src.stage.stage_loader import (
    create_or_open_project_stage,
    ensure_reference,
    ensure_sublayer,
    ensure_xform,
    save_stage,
)


def _require_exclusive_environment_sublayer(root_layer, source_asset) -> None:
    """Reject stale environment layers instead of composing contaminated A/Bs."""

    root_path = Path(root_layer.realPath or root_layer.identifier).resolve()
    actual = []
    for identifier in root_layer.subLayerPaths:
        path = Path(identifier)
        if not path.is_absolute():
            path = root_path.parent / path
        actual.append(path.resolve())
    expected = Path(source_asset).resolve()
    if actual != [expected]:
        raise RuntimeError(
            "project Stage must contain exactly the selected environment "
            f"Sublayer: expected={[str(expected)]}, "
            f"actual={[str(path) for path in actual]}"
        )


class SceneComposer:
    def __init__(self, config: ProjectConfig):
        self.config = config
        self.ground_topology_snapshot = None
        self.contact_snapshot = None
        self.mass_collision_snapshot = None
        self.wheel_velocity_drive_snapshot = None

    def compose(self, *, save: bool = False):
        config = self.config
        if config.environment.composition != "sublayer":
            raise ValueError("complete environment stages must use Sublayer composition")
        validate_default_prim(
            config.robot.asset_path,
            expected_name=config.robot.default_prim,
        )
        robot_dependencies = dependency_report(config.robot.asset_path)
        if robot_dependencies.unresolved:
            raise RuntimeError(f"robot asset has unresolved dependencies: {robot_dependencies.unresolved}")

        stage = create_or_open_project_stage(config.environment.project_stage)
        root_layer = stage.GetRootLayer()
        ensure_sublayer(root_layer, config.environment.source_asset)
        _require_exclusive_environment_sublayer(
            root_layer,
            config.environment.source_asset,
        )
        ensure_xform(stage, "/World")
        ensure_xform(stage, "/World/Robots")
        ensure_xform(stage, "/World/Graphs")
        ensure_xform(stage, "/World/DynamicObstacles")
        ensure_xform(stage, "/World/ExperimentMarkers")
        robot_prim = ensure_xform(stage, config.robot.runtime_prim_path)
        ensure_reference(robot_prim, config.robot.asset_path)
        poses = load_spawn_poses(config.spawn.poses_file)
        try:
            selected_pose = poses[config.spawn.selected]
        except KeyError as exc:
            raise ValueError(
                f"unknown selected spawn pose {config.spawn.selected!r}; "
                f"available={sorted(poses)}"
            ) from exc
        author_initial_articulation_pose(
            stage,
            config.robot.runtime_prim_path,
            selected_pose,
        )
        author_articulation_solver_iterations(
            stage,
            config.robot.articulation_root,
            load_articulation_physics_config(config.files.robot),
        )
        # These anonymous overlays must exist before PhysicsSetup performs the
        # first update that lets PhysX parse the composed articulation.
        self.mass_collision_snapshot = apply_mass_collision_profile(
            stage,
            config,
        )
        self.wheel_velocity_drive_snapshot = apply_wheel_velocity_drive(
            stage,
            config,
        )
        validate_prim(stage, config.robot.base_link_prim, "Xform")
        self.ground_topology_snapshot = apply_ground_topology(stage, config)
        self.contact_snapshot = apply_contact_profile(stage, config)
        if save:
            save_stage(stage)
        return stage
