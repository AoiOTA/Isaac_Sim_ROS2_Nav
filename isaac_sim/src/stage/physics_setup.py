"""PhysicsScene validation and Isaac Sim 6.0.1 lifecycle adapter."""

from __future__ import annotations

from dataclasses import dataclass

from isaac_sim.src.config import SimulationConfig


class PhysicsSetupError(RuntimeError):
    pass


def find_all_physics_scenes(stage):
    from pxr import UsdPhysics

    return [prim for prim in stage.TraverseAll() if prim.IsA(UsdPhysics.Scene)]


def validate_stage_units(stage, expected_meters: float = 1.0) -> None:
    from pxr import UsdGeom

    actual = float(UsdGeom.GetStageMetersPerUnit(stage))
    if abs(actual - expected_meters) > 1e-9:
        raise PhysicsSetupError(f"stage metersPerUnit={actual}, expected {expected_meters}")


def validate_up_axis(stage, expected: str = "Z") -> None:
    from pxr import UsdGeom

    actual = str(UsdGeom.GetStageUpAxis(stage)).upper()
    if actual != expected.upper():
        raise PhysicsSetupError(f"stage upAxis={actual}, expected {expected}")


def _create_physics_scene(stage, scene_path: str):
    from pxr import UsdPhysics

    return UsdPhysics.Scene.Define(stage, scene_path).GetPrim()


def _configure_scene(prim, physics_hz: float) -> None:
    from pxr import Gf, Sdf, UsdPhysics

    scene = UsdPhysics.Scene(prim)
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)
    prim.CreateAttribute("physxScene:timeStepsPerSecond", Sdf.ValueTypeNames.UInt).Set(int(round(physics_hz)))
    prim.CreateAttribute("physxScene:solverType", Sdf.ValueTypeNames.Token).Set("TGS")
    prim.CreateAttribute("physxScene:enableCCD", Sdf.ValueTypeNames.Bool).Set(True)
    prim.CreateAttribute("physxScene:enableStabilization", Sdf.ValueTypeNames.Bool).Set(True)


@dataclass
class IsaacSimulationRuntime:
    """Concrete lifecycle port around SimulationManager and omni.timeline."""

    app: object

    def __post_init__(self) -> None:
        import omni.timeline

        self._timeline = omni.timeline.get_timeline_interface()

    def reset(self) -> None:
        """Reset first, warm physics, then pause for articulation/spawn setup."""

        self._timeline.stop()
        self.app.update()
        self._timeline.play()
        self.app.update()
        self.app.update()
        self._timeline.pause()
        self.app.update()

    def pause(self) -> None:
        self._timeline.pause()
        self.app.update()

    def play(self) -> None:
        self._timeline.play()
        self.app.update()

    def stop(self) -> None:
        self._timeline.stop()
        self.app.update()

    def step(self, *, render: bool) -> None:
        from isaacsim.core.simulation_manager import SimulationManager

        SimulationManager.step(steps=1, update_fabric=False)
        if render:
            self.app.update()

    def update(self) -> None:
        self.app.update()


class PhysicsSetup:
    def __init__(self, config: SimulationConfig):
        self.config = config

    def apply(self, stage, app) -> IsaacSimulationRuntime:
        scenes = find_all_physics_scenes(stage)
        if len(scenes) == 0:
            scene_prim = _create_physics_scene(stage, self.config.expected_physics_scene)
        elif len(scenes) == 1:
            scene_prim = scenes[0]
        else:
            paths = [str(prim.GetPath()) for prim in scenes]
            raise PhysicsSetupError(f"multiple PhysicsScene prims detected: {paths}")
        scene_path = str(scene_prim.GetPath())
        if scene_path != self.config.expected_physics_scene:
            raise PhysicsSetupError(
                f"PhysicsScene is {scene_path}, expected {self.config.expected_physics_scene}; refusing to create a second"
            )
        validate_stage_units(stage, 1.0)
        validate_up_axis(stage, "Z")
        _configure_scene(scene_prim, self.config.physics_hz)

        from isaacsim.core.simulation_manager import SimulationManager

        SimulationManager.set_physics_dt(1.0 / self.config.physics_hz, physics_scene=scene_path)
        import omni.timeline

        timeline = omni.timeline.get_timeline_interface()
        timeline.set_target_framerate(self.config.rendering_hz)
        return IsaacSimulationRuntime(app)
