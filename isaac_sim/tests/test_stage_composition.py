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
from isaac_sim.src.stage.physics_setup import (  # noqa: E402
    ensure_physics_scene,
    find_all_physics_scenes,
)
from isaac_sim.src.stage.scene_composer import (  # noqa: E402
    SceneComposer,
    author_configured_static_frames,
    ensure_kujiale_g5_doorway_floor_fill,
)
from isaac_sim.src.stage.stage_loader import (  # noqa: E402
    make_environment_meshes_double_sided,
    repair_malformed_asset_paths,
)
from isaac_sim.src.visualization.third_person_camera import (  # noqa: E402
    ThirdPersonCamera,
)


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
    from pxr import UsdGeom

    config = _config()
    stage = SceneComposer(config).compose(save=False)
    scenes = find_all_physics_scenes(stage)
    assert [str(scene.GetPath()) for scene in scenes] == [config.simulation.expected_physics_scene]
    assert UsdGeom.GetStageMetersPerUnit(stage) == pytest.approx(1.0)
    assert UsdGeom.GetStageUpAxis(stage) == UsdGeom.Tokens.z


def test_configured_static_frames_are_root_authored_and_idempotent():
    config = _config()
    stage = SceneComposer(config).compose(save=False)
    first = author_configured_static_frames(
        stage, config.robot.base_link_prim, config.files.robot
    )
    second = author_configured_static_frames(
        stage, config.robot.base_link_prim, config.files.robot
    )
    assert first == second
    assert (
        f"{config.robot.base_link_prim}/camera_link/camera_front_link/"
        "camera_front_optical_frame"
    ) in second
    validate_sensor_frames(stage, config.robot.base_link_prim)


def test_missing_physics_scene_is_created_once():
    from pxr import Usd

    stage = Usd.Stage.CreateInMemory()
    first = ensure_physics_scene(stage, "/PhysicsScene")
    second = ensure_physics_scene(stage, "/PhysicsScene")

    assert first == second
    assert [str(scene.GetPath()) for scene in find_all_physics_scenes(stage)] == [
        "/PhysicsScene"
    ]


def test_kujiale_g5_doorway_floor_fill_is_flush_static_collision_only():
    from pxr import Usd, UsdGeom, UsdPhysics

    stage = Usd.Stage.CreateInMemory()
    repaired = ensure_kujiale_g5_doorway_floor_fill(
        stage,
        Path("kujiale_0026_A_to_B_door_open.usd"),
    )
    prim = stage.GetPrimAtPath(
        "/World/EnvironmentRepairs/kujiale_g5_doorway_floor_fill"
    )
    assert repaired is True
    assert prim.IsA(UsdGeom.Cube)
    assert prim.HasAPI(UsdPhysics.CollisionAPI)
    assert not prim.HasAPI(UsdPhysics.RigidBodyAPI)
    bounds = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(), [UsdGeom.Tokens.default_]
    ).ComputeWorldBound(prim).ComputeAlignedRange()
    assert bounds.GetMin()[2] == pytest.approx(-0.10)
    assert bounds.GetMax()[2] == pytest.approx(0.0)
    assert ensure_kujiale_g5_doorway_floor_fill(
        stage,
        Path("unrelated_room.usd"),
    ) is False


def test_malformed_asset_path_is_repaired_in_overlay_only(tmp_path: Path):
    from pxr import Sdf, Usd, UsdShade

    texture = tmp_path / "Materials" / "Textures" / "albedo.png"
    texture.parent.mkdir(parents=True)
    texture.write_bytes(b"texture")
    source_path = tmp_path / "source.usda"
    source = Usd.Stage.CreateNew(str(source_path))
    shader = UsdShade.Shader.Define(source, "/Root/Shader")
    shader.CreateInput("texture", Sdf.ValueTypeNames.Asset).Set(
        Sdf.AssetPath(".../Materials/Textures/albedo.png")
    )
    source.GetRootLayer().Save()

    overlay = Usd.Stage.CreateInMemory()
    overlay.GetRootLayer().subLayerPaths.append(str(source_path))
    repaired = repair_malformed_asset_paths(overlay, tmp_path)

    attribute = overlay.GetPrimAtPath("/Root/Shader").GetAttribute("inputs:texture")
    assert repaired == ("/Root/Shader.inputs:texture",)
    assert attribute.Get().path == str(texture.resolve())
    assert (
        Usd.Stage.Open(str(source_path))
        .GetPrimAtPath("/Root/Shader")
        .GetAttribute("inputs:texture")
        .Get()
        .path
        == ".../Materials/Textures/albedo.png"
    )


def test_single_sided_room_mesh_is_repaired_in_overlay_only(tmp_path: Path):
    from pxr import Usd, UsdGeom

    source_path = tmp_path / "room.usda"
    source = Usd.Stage.CreateNew(str(source_path))
    mesh = UsdGeom.Mesh.Define(source, "/Root/wall")
    mesh.CreateDoubleSidedAttr(False)
    source.GetRootLayer().Save()

    overlay = Usd.Stage.CreateInMemory()
    overlay.GetRootLayer().subLayerPaths.append(str(source_path))
    repaired = make_environment_meshes_double_sided(overlay)

    assert repaired == ("/Root/wall",)
    assert UsdGeom.Mesh(overlay.GetPrimAtPath("/Root/wall")) \
        .GetDoubleSidedAttr().Get() is True
    assert UsdGeom.Mesh(Usd.Stage.Open(str(source_path)).GetPrimAtPath(
        "/Root/wall")).GetDoubleSidedAttr().Get() is False


def test_third_person_camera_is_authored_below_base_link():
    from pxr import UsdGeom

    config = _config()
    stage = SceneComposer(config).compose(save=False)
    camera = ThirdPersonCamera(
        stage,
        config.robot.base_link_prim,
        config.third_person_camera,
        activate_viewport=False,
    )

    expected_path = (
        f"{config.robot.base_link_prim}/"
        f"{config.third_person_camera.prim_name}"
    )
    prim = stage.GetPrimAtPath(expected_path)
    assert camera.camera_path == expected_path
    assert prim.IsA(UsdGeom.Camera)
    assert str(prim.GetParent().GetPath()) == config.robot.base_link_prim
    assert UsdGeom.Camera(prim).GetFocalLengthAttr().Get() \
        == pytest.approx(config.third_person_camera.focal_length_mm)


def test_composed_stage_uses_supported_wheel_colliders():
    from pxr import Usd, UsdGeom, UsdPhysics

    config = _config()
    stage = SceneComposer(config).compose(save=False)
    root = stage.GetPrimAtPath(config.robot.articulation_root)
    assert root.GetAttribute(
        "physxArticulation:solverPositionIterationCount"
    ).Get() == 32
    assert root.GetAttribute(
        "physxArticulation:solverVelocityIterationCount"
    ).Get() == 4

    for joint_name in config.robot.wheel_joints:
        joint_prim = next(
            prim
            for prim in Usd.PrimRange(root)
            if prim.GetName() == joint_name
        )
        joint = UsdPhysics.RevoluteJoint(joint_prim)
        bodies = (
            tuple(joint.GetBody0Rel().GetTargets())
            + tuple(joint.GetBody1Rel().GetTargets())
        )
        wheel_path = next(
            path
            for path in bodies
            if str(path) != config.robot.base_link_prim
        )
        wheel = stage.GetPrimAtPath(wheel_path)
        assert not wheel.GetChild("collisions").IsActive()
        collider = wheel.GetChild("collisions_v2")
        assert collider.IsA(UsdGeom.Cylinder)
        assert collider.HasAPI(UsdPhysics.CollisionAPI)
        assert not collider.HasAttribute("physxCollisionCustomGeometry")
        assert UsdGeom.Cylinder(collider).GetRadiusAttr().Get() \
            == pytest.approx(0.098)
        assert UsdGeom.Cylinder(collider).GetHeightAttr().Get() \
            == pytest.approx(0.04)


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


def test_stationary_obstacle_keeps_an_isaac_gui_translate_edit_until_reset():
    from pxr import Gf, Usd

    scenario = replace(
        load_dynamic_scenario(
            ROOT / "isaac_sim/configs/experiments/kujiale_long_range_static.yaml"),
        enabled=True,
    )
    stage = Usd.Stage.CreateInMemory()
    manager = DynamicObstacleManager(stage, scenario, map_to_usd=lambda position: position)
    runtime = manager._runtime["rgbd_low_box_center"]
    edited = (0.34, -0.12, 0.08)
    runtime.translate_op.Set(Gf.Vec3d(*edited))

    manager.update(5.0)
    assert tuple(runtime.translate_op.Get()) == pytest.approx(edited)

    manager.reset(7201)
    assert tuple(runtime.translate_op.Get()) == pytest.approx(
        runtime.spec.start)
