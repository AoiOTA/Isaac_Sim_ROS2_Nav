from __future__ import annotations

import hashlib
from dataclasses import replace
from pathlib import Path

import pytest


try:
    import pxr  # noqa: F401
except ImportError:
    HAS_PXR = False
else:
    HAS_PXR = True
pytestmark = [
    pytest.mark.isaac,
    pytest.mark.skipif(not HAS_PXR, reason="Isaac/USD pxr bindings are unavailable"),
]

from isaac_sim.src.config import load_project_config  # noqa: E402
from isaac_sim.src.experiment.dynamic_obstacles import DynamicObstacleManager  # noqa: E402
from isaac_sim.src.experiment.scenario import load_dynamic_scenario  # noqa: E402
from isaac_sim.src.stage.asset_validator import validate_robot_articulation  # noqa: E402
from isaac_sim.src.stage.asset_validator import validate_sensor_frames  # noqa: E402
from isaac_sim.src.stage.physics_setup import find_all_physics_scenes  # noqa: E402
from isaac_sim.src.stage.scene_composer import SceneComposer  # noqa: E402


ROOT = Path(__file__).resolve().parents[2]


def _config():
    return load_project_config(
        ROOT / "isaac_sim/configs/project.yaml",
        {
            "PROJECT_ROOT": str(ROOT),
            "ISAAC_ASSET_ROOT": "/home/lyb/isaacsim_assets/Assets/Isaac/6.0",
        },
    )


def test_environment_is_sublayer_robot_is_reference_and_stage_is_not_saved():
    config = _config()
    before = hashlib.sha256(config.environment.project_stage.read_bytes()).hexdigest()
    stage = SceneComposer(config).compose(save=False)
    after = hashlib.sha256(config.environment.project_stage.read_bytes()).hexdigest()
    assert before == after
    sublayers = [str(Path(path).resolve()) for path in stage.GetRootLayer().subLayerPaths]
    assert str(config.environment.source_asset) in sublayers
    references = stage.GetPrimAtPath(config.robot.runtime_prim_path).GetMetadata("references")
    authored_references = references.GetAddedOrExplicitItems()
    assert len(authored_references) == 1
    authored_path = Path(authored_references[0].assetPath)
    if not authored_path.is_absolute():
        authored_path = config.environment.project_stage.parent / authored_path
    assert authored_path.resolve() == config.robot.asset_path
    validate_robot_articulation(
        stage,
        config.robot.articulation_root,
        config.robot.base_link_prim,
        config.robot.wheel_joints,
    )
    validate_sensor_frames(stage, config.robot.base_link_prim)


def test_composed_stage_has_exactly_one_expected_physics_scene():
    config = _config()
    stage = SceneComposer(config).compose(save=False)
    scenes = find_all_physics_scenes(stage)
    assert [str(scene.GetPath()) for scene in scenes] == [config.simulation.expected_physics_scene]


def test_dynamic_obstacle_reset_restarts_scenario_time():
    from pxr import Usd, UsdGeom, UsdPhysics

    scenario = replace(
        load_dynamic_scenario(ROOT / "isaac_sim/configs/experiments/dynamic.yaml"),
        enabled=True,
    )
    stage = Usd.Stage.CreateInMemory()
    manager = DynamicObstacleManager(stage, scenario)

    crossing = stage.GetPrimAtPath('/World/DynamicObstacles/crossing_box')
    assert crossing.HasAPI(UsdPhysics.CollisionAPI)
    world_transform = UsdGeom.Xformable(
        crossing).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    assert tuple(world_transform.ExtractTranslation()) == pytest.approx(
        scenario.obstacles[0].start)

    manager.reset(42)
    manager.update(10.0)
    initial = tuple(manager._runtime["crossing_box"].translate_op.Get())
    manager.update(11.0)
    advanced = tuple(manager._runtime["crossing_box"].translate_op.Get())
    assert advanced != initial

    manager.reset(42)
    manager.update(100.0)
    restarted = tuple(manager._runtime["crossing_box"].translate_op.Get())
    assert restarted == pytest.approx(initial)
