"""Compose the project Stage using an environment Sublayer and robot Reference."""

from __future__ import annotations

from isaac_sim.src.config import ProjectConfig
from isaac_sim.src.robot.articulation_runtime import (
    author_articulation_solver_iterations,
    load_articulation_physics_config,
)
from isaac_sim.src.stage.asset_validator import dependency_report, validate_default_prim, validate_prim
from isaac_sim.src.stage.stage_loader import (
    create_or_open_project_stage,
    ensure_reference,
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
        root_layer = stage.GetRootLayer()
        ensure_sublayer(root_layer, config.environment.source_asset)
        ensure_xform(stage, "/World")
        ensure_xform(stage, "/World/Robots")
        ensure_xform(stage, "/World/Graphs")
        ensure_xform(stage, "/World/DynamicObstacles")
        ensure_xform(stage, "/World/ExperimentMarkers")
        robot_prim = ensure_xform(stage, config.robot.runtime_prim_path)
        ensure_reference(robot_prim, config.robot.asset_path)
        author_articulation_solver_iterations(
            stage,
            config.robot.articulation_root,
            load_articulation_physics_config(config.files.robot),
        )
        validate_prim(stage, config.robot.base_link_prim, "Xform")
        if save:
            save_stage(stage)
        return stage
