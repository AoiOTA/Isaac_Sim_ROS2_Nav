"""Robot-relative third-person camera for the interactive Isaac viewport.

The camera is authored below ``base_link``. USD transform inheritance makes it
follow every robot translation, turn, and Reset without a per-frame pose copy.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING


if TYPE_CHECKING:
    from isaac_sim.src.config import ThirdPersonCameraConfig


class ThirdPersonCameraError(RuntimeError):
    """Raised when the configured third-person camera cannot be constructed."""


@dataclass(frozen=True)
class RelativeCameraPose:
    eye: tuple[float, float, float]
    aim: tuple[float, float, float]


def relative_camera_pose(
    config: ThirdPersonCameraConfig,
) -> RelativeCameraPose:
    """Return camera points in the Jackal ``base_link`` coordinate system."""

    return RelativeCameraPose(
        eye=(-config.distance_m, 0.0, config.height_m),
        aim=(config.look_ahead_m, 0.0, config.look_at_height_m),
    )


class ThirdPersonCamera:
    """Create a camera below ``base_link`` and bind the active GUI viewport."""

    def __init__(
        self,
        stage: object,
        base_link_prim: str,
        config: ThirdPersonCameraConfig,
        *,
        activate_viewport: bool = True,
    ) -> None:
        from pxr import Gf, UsdGeom

        base = stage.GetPrimAtPath(base_link_prim)
        if not base.IsValid():
            raise ThirdPersonCameraError(
                f"base_link prim does not exist: {base_link_prim}"
            )
        camera_path = f"{base_link_prim}/{config.prim_name}"
        camera = UsdGeom.Camera.Define(stage, camera_path)
        if not camera.GetPrim().IsValid():
            raise ThirdPersonCameraError(
                f"failed to define camera at {camera_path}"
            )
        camera.CreateFocalLengthAttr().Set(config.focal_length_mm)
        camera.CreateClippingRangeAttr().Set(Gf.Vec2f(0.05, 1000.0))
        xformable = UsdGeom.Xformable(camera.GetPrim())
        xformable.ClearXformOpOrder()
        pose = relative_camera_pose(config)
        transform = Gf.Matrix4d(1.0).SetLookAt(
            Gf.Vec3d(*pose.eye),
            Gf.Vec3d(*pose.aim),
            Gf.Vec3d(0.0, 0.0, 1.0),
        ).GetInverse().GetOrthonormalized()
        xformable.MakeMatrixXform().Set(transform)
        self._camera_path = camera.GetPath()
        self._viewport_bound = False
        self._activate_viewport = activate_viewport
        if activate_viewport:
            self.bind_viewport()

    @property
    def viewport_bound(self) -> bool:
        return self._viewport_bound

    @property
    def camera_path(self) -> str:
        return str(self._camera_path)

    def bind_viewport(self) -> bool:
        """Bind once the main viewport exists; startup may require a retry."""

        if self._viewport_bound:
            return True
        if not self._activate_viewport:
            return False
        from omni.kit.viewport.utility import get_active_viewport

        viewport = get_active_viewport()
        if viewport is None:
            return False
        viewport.camera_path = self._camera_path
        viewport.updates_enabled = True
        self._viewport_bound = True
        return True
