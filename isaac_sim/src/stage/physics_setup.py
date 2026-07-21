"""PhysicsScene validation and Isaac Sim 6.0.1 lifecycle adapter."""

from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction

from isaac_sim.src.config import SimulationConfig


class PhysicsSetupError(RuntimeError):
    pass


@dataclass(frozen=True)
class PacingPlan:
    """Separate fixed simulation dt from the wall-clock loop rate limit."""

    mode: str
    timeline_hz: float
    wall_loop_hz: float | None
    target_realtime_factor: float


def pacing_plan(config: SimulationConfig) -> PacingPlan:
    if config.pacing_mode == "realtime":
        return PacingPlan(
            mode="realtime",
            timeline_hz=config.rendering_hz,
            wall_loop_hz=(
                config.rendering_hz * config.target_realtime_factor
            ),
            target_realtime_factor=config.target_realtime_factor,
        )
    if config.pacing_mode == "unbounded":
        return PacingPlan(
            mode="unbounded",
            timeline_hz=config.rendering_hz,
            wall_loop_hz=None,
            target_realtime_factor=config.target_realtime_factor,
        )
    raise PhysicsSetupError(
        f"unsupported pacing mode after configuration validation: "
        f"{config.pacing_mode}"
    )


def prepare_pacing(config: SimulationConfig) -> PacingPlan:
    """Seed Kit/Fabric timing before opening the project Stage."""

    import carb.settings
    import omni.timeline

    plan = pacing_plan(config)
    settings = carb.settings.get_settings()
    settings.set_bool("/app/player/useFixedTimeStepping", True)
    settings.set_bool(
        "/app/runLoops/main/rateLimitEnabled",
        plan.wall_loop_hz is not None,
    )
    if plan.wall_loop_hz is not None:
        settings.set_float(
            "/app/runLoops/main/rateLimitFrequency",
            plan.wall_loop_hz,
        )
    period = Fraction(1.0 / plan.timeline_hz).limit_denominator(1_000_000)
    settings.set_int(
        "/app/settings/fabricDefaultSimPeriodNumerator",
        period.numerator,
    )
    settings.set_int(
        "/app/settings/fabricDefaultSimPeriodDenominator",
        period.denominator,
    )
    omni.timeline.get_timeline_interface().set_target_framerate(
        plan.wall_loop_hz or plan.timeline_hz
    )
    return plan


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


def ensure_physics_scene(stage, expected_path: str):
    """Return the sole expected PhysicsScene, creating it when absent."""

    scenes = find_all_physics_scenes(stage)
    if len(scenes) == 0:
        return _create_physics_scene(stage, expected_path)
    if len(scenes) > 1:
        paths = [str(prim.GetPath()) for prim in scenes]
        raise PhysicsSetupError(f"multiple PhysicsScene prims detected: {paths}")
    scene_prim = scenes[0]
    scene_path = str(scene_prim.GetPath())
    if scene_path != expected_path:
        raise PhysicsSetupError(
            f"PhysicsScene is {scene_path}, expected {expected_path}; "
            "refusing to create a second"
        )
    return scene_prim


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
        scene_prim = ensure_physics_scene(
            stage,
            self.config.expected_physics_scene,
        )
        scene_path = str(scene_prim.GetPath())
        validate_stage_units(stage, 1.0)
        validate_up_axis(stage, "Z")
        _configure_scene(scene_prim, self.config.physics_hz)

        from isaacsim.core.rendering_manager import RenderingManager
        from isaacsim.core.simulation_manager import SimulationManager

        SimulationManager.setup_simulation(dt=1.0 / self.config.physics_hz)
        import carb.settings
        import omni.timeline

        timeline = omni.timeline.get_timeline_interface()
        settings = carb.settings.get_settings()
        settings.set_bool("/app/player/useFixedTimeStepping", True)
        settings.set_bool(
            "/app/runLoops/main/rateLimitEnabled",
            plan.wall_loop_hz is not None,
        )
        # Isaac Sim 6.0 couples RunLoop, Timeline time codes, and its manual
        # loop runner here. Setting only targetFramerate leaves the Stage at
        # its previous timeCodesPerSecond and can advance simulation at ~2x.
        RenderingManager.set_dt(1.0 / plan.timeline_hz)
        timeline.set_play_every_frame(
            plan.mode == "unbounded" or plan.target_realtime_factor > 1.0
        )
        if plan.wall_loop_hz is not None:
            settings.set_float(
                "/app/runLoops/main/rateLimitFrequency",
                plan.wall_loop_hz,
            )
            timeline.set_target_framerate(plan.wall_loop_hz)
        return IsaacSimulationRuntime(app)
