"""Compose the project Stage using an environment Sublayer and robot Reference."""

from __future__ import annotations

from pathlib import Path

import yaml

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


_KUJIALE_DOORWAY_ASSET = "kujiale_0026_A_to_B_door_open.usd"
_KUJIALE_DOORWAY_FILL_PATH = "/World/EnvironmentRepairs/kujiale_g5_doorway_floor_fill"


def author_configured_static_frames(stage, base_link_prim: str, robot_config: Path) -> tuple[str, ...]:
    """Author the robot's configured fixed-frame tree in the project layer.

    The NVIDIA Jackal reference can finish payload composition after the first
    Kit updates.  Root-layer frame specs keep camera/LiDAR/IMU ownership stable
    without editing the imported binary asset or duplicating sensor geometry.
    """

    from pxr import Gf, UsdGeom, UsdPhysics

    document = yaml.safe_load(Path(robot_config).read_text(encoding="utf-8"))
    transforms = document.get("static_transforms") if isinstance(document, dict) else None
    if not isinstance(transforms, list):
        raise ValueError("robot static_transforms must be a list")
    paths = {"base_link": str(base_link_prim)}
    authored: list[str] = []
    for index, item in enumerate(transforms):
        if not isinstance(item, dict):
            raise ValueError(f"static_transforms[{index}] must be an object")
        parent, child = item.get("parent"), item.get("child")
        translation, rotation = item.get("translation"), item.get("rotation_xyzw")
        if parent not in paths or not isinstance(child, str) or not child:
            raise ValueError(f"static_transforms[{index}] has an unresolved frame")
        if (
            not isinstance(translation, list)
            or len(translation) != 3
            or not isinstance(rotation, list)
            or len(rotation) != 4
        ):
            raise ValueError(f"static_transforms[{index}] pose is invalid")
        path = f"{paths[parent]}/{child}"
        existing = stage.GetPrimAtPath(path)
        if existing.IsValid() and existing.GetTypeName() not in {"", "Xform"}:
            raise ValueError(f"configured static frame has incompatible type: {path}")
        frame = UsdGeom.Xform.Define(stage, path).GetPrim()
        if frame.HasAPI(UsdPhysics.RigidBodyAPI):
            raise ValueError(f"configured static frame must not be a rigid body: {path}")
        xform = UsdGeom.Xformable(frame)
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(*(float(value) for value in translation)))
        x, y, z, w = (float(value) for value in rotation)
        xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(
            Gf.Quatf(w, x, y, z)
        )
        paths[child] = path
        authored.append(path)
    return tuple(authored)


def ensure_kujiale_g5_doorway_floor_fill(stage, source_asset) -> bool:
    """Bridge the exported 10 cm doorway recess with a flush static collider.

    The selected Kujiale USD contains a collision-only floor cube at the G5
    doorway whose top is 0.10 m below the adjacent floor.  The navigation map
    correctly treats the doorway as traversable, but a wheel can enter that
    recess and become stuck.  Author the repair in the runtime project layer,
    never in the supplied environment USD.
    """

    from pxr import Gf, UsdGeom, UsdPhysics

    if source_asset.name != _KUJIALE_DOORWAY_ASSET:
        return False

    ensure_xform(stage, "/World/EnvironmentRepairs")
    cube = UsdGeom.Cube.Define(stage, _KUJIALE_DOORWAY_FILL_PATH)
    cube.CreateSizeAttr(1.0)
    xform = UsdGeom.Xformable(cube.GetPrim())
    xform.ClearXformOpOrder()
    # Exact footprint of the recessed collider plus a 20 mm shell.  The
    # centre/height yield a top face at z=0, flush with both floor tiles.
    xform.AddTranslateOp().Set(Gf.Vec3d(3.458, 1.448, -0.05))
    xform.AddScaleOp().Set(Gf.Vec3d(1.44, 0.29, 0.10))
    collision = UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    collision.CreateCollisionEnabledAttr(True)
    cube.GetPrim().SetCustomDataByKey(
        "isaac_nav:purpose", "flush fill for the G5 doorway collision recess"
    )
    return True


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
        ensure_kujiale_g5_doorway_floor_fill(stage, config.environment.source_asset)
        robot_prim = ensure_xform(stage, config.robot.runtime_prim_path)
        ensure_reference(robot_prim, config.robot.asset_path)
        author_configured_static_frames(
            stage, config.robot.base_link_prim, config.files.robot
        )
        ensure_physics_scene(stage, config.simulation.expected_physics_scene)
        validate_prim(stage, config.robot.base_link_prim, "Xform")
        if save:
            save_stage(stage)
        return stage
