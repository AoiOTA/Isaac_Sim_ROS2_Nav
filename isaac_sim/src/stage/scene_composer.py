"""Compose the project Stage using an environment Sublayer and robot Reference."""

from __future__ import annotations

from isaac_sim.src.config import ProjectConfig
from isaac_sim.src.stage.asset_validator import dependency_report, validate_default_prim, validate_prim
from isaac_sim.src.stage.physics_setup import ensure_physics_scene
from isaac_sim.src.stage.stage_loader import (
    create_or_open_project_stage,
    ensure_reference,
    make_environment_meshes_double_sided,
    repair_malformed_asset_paths,
    ensure_sublayer,
    ensure_xform,
    save_stage,
)


class SceneComposer:
    def __init__(self, config: ProjectConfig):
        self.config = config

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
        from pxr import UsdGeom

        # A newly created root layer otherwise falls back to USD's centimeter
        # default even when the selected environment sublayer is authored in meters.
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        root_layer = stage.GetRootLayer()
        ensure_sublayer(root_layer, config.environment.source_asset)
        repair_malformed_asset_paths(
            stage,
            config.environment.source_asset.parent,
        )
        try:
            config.environment.source_asset.resolve().relative_to(
                config.asset_root.resolve()
            )
        except ValueError:
            # Local/exported rooms are not guaranteed to have inward-facing
            # structural normals. Official Isaac assets keep their authored
            # renderer contract unchanged.
            make_environment_meshes_double_sided(stage)
        world_prim = ensure_xform(stage, "/World")
        stage.SetDefaultPrim(world_prim)
        ensure_xform(stage, "/World/Robots")
        ensure_xform(stage, "/World/Graphs")
        ensure_xform(stage, "/World/DynamicObstacles")
        ensure_xform(stage, "/World/ExperimentMarkers")
        robot_prim = ensure_xform(stage, config.robot.runtime_prim_path)
        ensure_reference(robot_prim, config.robot.asset_path)
        ensure_physics_scene(stage, config.simulation.expected_physics_scene)
        validate_prim(stage, config.robot.base_link_prim, "Xform")
        if save:
            save_stage(stage)
        return stage
