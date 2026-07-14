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


def test_project_overlay_uses_supported_symmetric_wheel_colliders_and_tgs_counts():
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    config = _config()
    stage = SceneComposer(config).compose(save=False)
    root = stage.GetPrimAtPath(config.robot.articulation_root)
    assert root.GetAttribute(
        "physxArticulation:solverPositionIterationCount"
    ).Get() == 32
    assert root.GetAttribute(
        "physxArticulation:solverVelocityIterationCount"
    ).Get() == 4

    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    wheel_positions = {}
    material_paths = []
    for joint_name in config.robot.wheel_joints:
        joints = [
            prim
            for prim in stage.TraverseAll()
            if prim.GetName() == joint_name
            and prim.IsA(UsdPhysics.RevoluteJoint)
        ]
        assert len(joints) == 1
        joint = UsdPhysics.RevoluteJoint(joints[0])
        bodies = tuple(joint.GetBody0Rel().GetTargets()) + tuple(
            joint.GetBody1Rel().GetTargets()
        )
        wheel_paths = [
            path for path in bodies if str(path) != config.robot.base_link_prim
        ]
        assert len(wheel_paths) == 1
        wheel = stage.GetPrimAtPath(wheel_paths[0])
        legacy = wheel.GetChild("collisions")
        assert legacy.IsValid()
        assert not legacy.IsActive()
        colliders = [
            prim
            for prim in Usd.PrimRange(wheel)
            if prim.HasAPI(UsdPhysics.CollisionAPI)
        ]
        assert len(colliders) == 1
        collider = colliders[0]
        assert collider.GetTypeName() == "Cylinder"
        assert collider.GetName() == "collisions_v2"
        assert not collider.HasAttribute("physxCollisionCustomGeometry")
        cylinder = UsdGeom.Cylinder(collider)
        assert cylinder
        assert cylinder.GetAxisAttr().Get() == UsdGeom.Tokens.z
        assert cylinder.GetRadiusAttr().Get() == pytest.approx(0.098)
        assert cylinder.GetHeightAttr().Get() == pytest.approx(0.04)
        extent = cylinder.GetExtentAttr().Get()
        assert tuple(extent[0]) == pytest.approx((-0.098, -0.098, -0.02))
        assert tuple(extent[1]) == pytest.approx((0.098, 0.098, 0.02))
        assert (
            UsdPhysics.CollisionAPI(collider)
            .GetCollisionEnabledAttr()
            .Get()
            is True
        )
        local_transform = UsdGeom.Xformable(collider).GetLocalTransformation()
        axis_in_wheel = local_transform.TransformDir(
            Gf.Vec3d(0.0, 0.0, 1.0)
        ).GetNormalized()
        assert tuple(axis_in_wheel) == pytest.approx(
            (0.0, -1.0, 0.0), abs=1e-7
        )
        material_targets = collider.GetRelationship(
            "material:binding"
        ).GetTargets()
        assert len(material_targets) == 1
        material_paths.append(str(material_targets[0]))
        relative, resets_xform_stack = cache.ComputeRelativeTransform(
            wheel, root
        )
        assert not resets_xform_stack
        wheel_positions[joint_name] = tuple(relative.ExtractTranslation())

    assert set(material_paths) == {
        f"{config.robot.runtime_prim_path}/PhysicsMaterials/wheels"
    }
    material = stage.GetPrimAtPath(material_paths[0])
    assert material.HasAPI(UsdPhysics.MaterialAPI)
    material_api = UsdPhysics.MaterialAPI(material)
    assert material_api.GetStaticFrictionAttr().Get() == pytest.approx(0.2)
    assert material_api.GetDynamicFrictionAttr().Get() == pytest.approx(0.2)
    assert material_api.GetRestitutionAttr().Get() == pytest.approx(0.0)

    front_left = wheel_positions["front_left_wheel_joint"]
    front_right = wheel_positions["front_right_wheel_joint"]
    rear_left = wheel_positions["rear_left_wheel_joint"]
    rear_right = wheel_positions["rear_right_wheel_joint"]
    assert front_left[0] == pytest.approx(-rear_left[0])
    assert front_right[0] == pytest.approx(-rear_right[0])
    assert front_left[1] == pytest.approx(-front_right[1])
    assert rear_left[1] == pytest.approx(-rear_right[1])
    assert front_left[2] == pytest.approx(rear_right[2])
    assert front_left[0] - rear_left[0] == pytest.approx(0.262)
    assert front_left[1] - front_right[1] == pytest.approx(0.37559)


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
