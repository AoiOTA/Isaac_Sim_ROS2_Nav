"""Repeatable kinematic obstacle authoring and playback."""

from __future__ import annotations

from dataclasses import dataclass
import math

from isaac_sim.src.experiment.scenario import DynamicScenario, ObstacleSpec


@dataclass
class _ObstacleRuntime:
    spec: ObstacleSpec
    translate_op: object
    phase: float


class DynamicObstacleManager:
    def __init__(self, stage, scenario: DynamicScenario, root_path: str = "/World/DynamicObstacles"):
        self.stage = stage
        self.scenario = scenario
        self.root_path = root_path
        self._runtime: dict[str, _ObstacleRuntime] = {}
        self._reset_time: float | None = None
        self._author()
        self.reset(scenario.seed)

    def _author(self) -> None:
        from pxr import Gf, UsdGeom, UsdPhysics

        UsdGeom.Xform.Define(self.stage, self.root_path)
        if not self.scenario.enabled:
            return
        for obstacle in self.scenario.obstacles:
            path = f"{self.root_path}/{obstacle.obstacle_id}"
            cube = UsdGeom.Cube.Define(self.stage, path)
            cube.CreateSizeAttr(1.0)
            xform = UsdGeom.Xformable(cube.GetPrim())
            xform.ClearXformOpOrder()
            translate = xform.AddTranslateOp()
            translate.Set(Gf.Vec3d(*obstacle.start))
            scale = xform.AddScaleOp()
            scale.Set(Gf.Vec3d(*obstacle.size))
            collision = UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
            collision.CreateCollisionEnabledAttr(True)
            rigid = UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
            rigid.CreateRigidBodyEnabledAttr(True)
            rigid.CreateKinematicEnabledAttr(True)
            mass = UsdPhysics.MassAPI.Apply(cube.GetPrim())
            mass.CreateMassAttr(obstacle.mass)
            self._runtime[obstacle.obstacle_id] = _ObstacleRuntime(obstacle, translate, 0.0)

    def reset(self, seed: int) -> None:
        from pxr import Gf

        phases = self.scenario.sampled_phases(seed)
        self._reset_time = None
        for identifier, runtime in self._runtime.items():
            runtime.phase = phases[identifier]
            runtime.translate_op.Set(Gf.Vec3d(*runtime.spec.start))

    def update(self, simulation_time: float) -> None:
        if not self.scenario.enabled:
            return
        if self._reset_time is None:
            self._reset_time = simulation_time
        elapsed = max(0.0, simulation_time - self._reset_time)
        from pxr import Gf

        for runtime in self._runtime.values():
            spec = runtime.spec
            delta = tuple(end - start for start, end in zip(spec.start, spec.end))
            distance = math.sqrt(sum(value * value for value in delta))
            if distance <= 1e-9:
                raise RuntimeError(f"dynamic obstacle {spec.obstacle_id} has a zero-length trajectory")
            duration = distance / spec.speed
            progress = (elapsed + runtime.phase) / duration
            if spec.repeat:
                normalized = progress % 2.0
                fraction = (
                    normalized if normalized <= 1.0 else 2.0 - normalized
                )
            else:
                fraction = min(1.0, max(0.0, progress))
            position = tuple(start + fraction * value for start, value in zip(spec.start, delta))
            runtime.translate_op.Set(Gf.Vec3d(*position))
