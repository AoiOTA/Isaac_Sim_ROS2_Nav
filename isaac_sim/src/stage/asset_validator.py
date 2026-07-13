"""Static USD contract checks used before physics or ROS starts."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


class AssetValidationError(RuntimeError):
    pass


@dataclass(frozen=True)
class DependencyReport:
    layers: tuple[str, ...]
    assets: tuple[str, ...]
    unresolved: tuple[str, ...]


def validate_prim(stage, prim_path: str, expected_type: str | None = None):
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid() or not prim.IsActive():
        raise AssetValidationError(f"required active prim is missing: {prim_path}")
    if expected_type is not None and prim.GetTypeName() != expected_type:
        raise AssetValidationError(
            f"prim {prim_path} has type {prim.GetTypeName()!r}, expected {expected_type!r}"
        )
    return prim


def validate_default_prim(asset_path: str | Path, expected_name: str | None = None) -> str:
    from pxr import Usd

    stage = Usd.Stage.Open(str(Path(asset_path).resolve()), load=Usd.Stage.LoadNone)
    if stage is None:
        raise AssetValidationError(f"could not open asset {asset_path}")
    prim = stage.GetDefaultPrim()
    if not prim.IsValid():
        raise AssetValidationError(f"asset has no valid defaultPrim: {asset_path}")
    if expected_name is not None and prim.GetName() != expected_name:
        raise AssetValidationError(f"defaultPrim is {prim.GetName()!r}, expected {expected_name!r}")
    return str(prim.GetPath())


def dependency_report(asset_path: str | Path, *, module_allowlist: Iterable[str] = ("OmniPBR.mdl",)) -> DependencyReport:
    from pxr import UsdUtils

    layers, assets, unresolved = UsdUtils.ComputeAllDependencies(str(Path(asset_path).resolve()))
    allowed = set(module_allowlist)
    missing = tuple(sorted(str(item) for item in unresolved if str(item) not in allowed))
    return DependencyReport(
        layers=tuple(sorted(layer.identifier for layer in layers)),
        assets=tuple(sorted(str(item) for item in assets)),
        unresolved=missing,
    )


def validate_sensor_frames(stage, base_link_prim: str) -> None:
    from pxr import UsdPhysics

    required = (
        f"{base_link_prim}/lidar_link",
        f"{base_link_prim}/imu_link",
        f"{base_link_prim}/camera_link",
        f"{base_link_prim}/camera_link/camera_front_link/camera_front_optical_frame",
        f"{base_link_prim}/camera_link/camera_left_link/camera_left_optical_frame",
        f"{base_link_prim}/camera_link/camera_right_link/camera_right_optical_frame",
    )
    for path in required:
        prim = validate_prim(stage, path, "Xform")
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            raise AssetValidationError(f"fixed sensor frame must not be an independent rigid body: {path}")


def validate_robot_articulation(
    stage,
    articulation_root: str,
    base_link_prim: str,
    wheel_joints: Iterable[str],
) -> None:
    """Validate the asset topology shared by Jackal and migration robots."""
    from pxr import Usd, UsdPhysics

    root = validate_prim(stage, articulation_root, "Xform")
    roots = [
        prim for prim in Usd.PrimRange(root)
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    if roots != [root]:
        raise AssetValidationError(
            "robot must have exactly one ArticulationRootAPI at "
            f"{articulation_root}; got {[str(prim.GetPath()) for prim in roots]}")
    base = validate_prim(stage, base_link_prim, "Xform")
    if not base.HasAPI(UsdPhysics.RigidBodyAPI):
        raise AssetValidationError(
            f"base_link must be a rigid body: {base_link_prim}")

    descendants = tuple(Usd.PrimRange(root))
    for joint_name in wheel_joints:
        matches = [prim for prim in descendants if prim.GetName() == joint_name]
        if len(matches) != 1 or not matches[0].IsA(UsdPhysics.RevoluteJoint):
            raise AssetValidationError(
                f"wheel joint {joint_name!r} must identify one RevoluteJoint")
        joint = UsdPhysics.RevoluteJoint(matches[0])
        body0 = list(joint.GetBody0Rel().GetTargets())
        body1 = list(joint.GetBody1Rel().GetTargets())
        if len(body0) != 1 or len(body1) != 1:
            raise AssetValidationError(
                f"wheel joint {joint_name!r} must have one Body0 and one Body1")
        if base.GetPath() not in body0 + body1:
            raise AssetValidationError(
                f"wheel joint {joint_name!r} must connect to {base_link_prim}")
        wheel_path = body1[0] if body0[0] == base.GetPath() else body0[0]
        wheel = validate_prim(stage, str(wheel_path), "Xform")
        if not wheel.HasAPI(UsdPhysics.RigidBodyAPI):
            raise AssetValidationError(
                f"wheel body must be rigid for joint {joint_name!r}: {wheel_path}")
        if not any(
            prim.HasAPI(UsdPhysics.CollisionAPI)
            for prim in Usd.PrimRange(wheel)
        ):
            raise AssetValidationError(
                f"wheel body has no collision geometry: {wheel_path}")


def validate_no_navigation_ground_truth_connections(connections: Iterable[tuple[str, str]]) -> None:
    """Reject graph-level GT connections into navigation-related endpoints."""

    forbidden = ("slam", "ekf", "nav2", "controller", "wheelodom", "wheel_odom")
    violations = [(source, target) for source, target in connections if "ground_truth" in source.lower() and any(
        token in target.lower() for token in forbidden
    )]
    if violations:
        raise AssetValidationError(f"ground truth must not feed navigation: {violations}")
