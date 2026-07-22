"""Repeatable physical obstacle authoring, triggering, and evidence states."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from typing import Callable

from isaac_sim.src.experiment.scenario import DynamicScenario, ObstacleSpec


@dataclass
class _ObstacleRuntime:
    spec: ObstacleSpec
    translate_op: object
    collision_attr: object
    visibility_attr: object
    active_at: float | None = None
    retired: bool = False
    phase: float = 0.0
    motion_started: bool = False
    completion_recorded: bool = False
    progress: float = 0.0


class DynamicObstacleManager:
    """Owns obstacle state; one group is triggered only after its Nav2 goal is accepted."""

    def __init__(
        self,
        stage,
        scenario: DynamicScenario,
        root_path: str = "/World/DynamicObstacles",
        map_to_usd: Callable[[tuple[float, float, float]], tuple[float, float, float]] | None = None,
    ):
        self.stage = stage
        self.scenario = scenario
        self.root_path = root_path
        self._map_to_usd = map_to_usd
        self._runtime: dict[str, _ObstacleRuntime] = {}
        self._events: list[dict[str, object]] = []
        self._reset_time: float | None = None
        self._author()
        self.reset(scenario.seed)

    def _world_position(self, position: tuple[float, float, float]) -> tuple[float, float, float]:
        if self.scenario.coordinate_frame == "map":
            if self._map_to_usd is None:
                raise RuntimeError("map-coordinate obstacles require a calibrated map_to_usd transform")
            return self._map_to_usd(position)
        return position

    def _event(self, kind: str, simulation_time: float, **detail: object) -> None:
        self._events.append({"event": kind, "simulation_time": simulation_time, **detail})

    def _set_enabled(self, runtime: _ObstacleRuntime, enabled: bool) -> None:
        from pxr import UsdGeom
        runtime.collision_attr.Set(enabled)
        runtime.visibility_attr.Set(UsdGeom.Tokens.inherited if enabled else UsdGeom.Tokens.invisible)

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
            translate.Set(Gf.Vec3d(*self._world_position(obstacle.start)))
            scale = xform.AddScaleOp()
            scale.Set(Gf.Vec3d(*obstacle.size))
            collision = UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
            collision_attr = collision.CreateCollisionEnabledAttr(True)
            rigid = UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
            rigid.CreateRigidBodyEnabledAttr(True)
            rigid.CreateKinematicEnabledAttr(True)
            UsdPhysics.MassAPI.Apply(cube.GetPrim()).CreateMassAttr(obstacle.mass)
            visibility = UsdGeom.Imageable(cube.GetPrim()).CreateVisibilityAttr()
            self._runtime[obstacle.obstacle_id] = _ObstacleRuntime(
                obstacle, translate, collision_attr, visibility
            )

    def reset(self, seed: int) -> None:
        from pxr import Gf

        phases = self.scenario.sampled_phases(seed)
        self._reset_time = None
        self._events.clear()
        for identifier, runtime in self._runtime.items():
            runtime.phase = phases[identifier]
            runtime.active_at = None if runtime.spec.trigger_group else 0.0
            runtime.retired = False
            runtime.motion_started = False
            runtime.completion_recorded = False
            runtime.progress = 0.0
            runtime.translate_op.Set(Gf.Vec3d(*self._world_position(runtime.spec.start)))
            self._set_enabled(runtime, runtime.active_at is not None)
        self._event("reset", 0.0, seed=seed)

    def trigger(self, group: str, simulation_time: float) -> tuple[str, ...]:
        activated: list[str] = []
        for identifier, runtime in self._runtime.items():
            if runtime.spec.trigger_group == group and runtime.active_at is None and not runtime.retired:
                runtime.active_at = simulation_time
                self._set_enabled(runtime, True)
                activated.append(identifier)
                self._event("trigger", simulation_time, obstacle_id=identifier, group=group)
        return tuple(activated)

    def state(self) -> dict[str, object]:
        return {
            "obstacles": [
                {
                    "id": identifier,
                    "trigger_group": runtime.spec.trigger_group,
                    "state": "retired" if runtime.retired else ("active" if runtime.active_at is not None else "waiting"),
                    "progress": runtime.progress,
                }
                for identifier, runtime in sorted(self._runtime.items())
            ],
            "events": list(self._events),
        }

    def bind_ros(self, node, simulation_time: Callable[[], float]) -> None:
        """Expose the documented trigger/reset/state endpoints from the Isaac process."""
        from std_msgs.msg import String
        from std_srvs.srv import Trigger

        self._state_publisher = node.create_publisher(String, "/experiment/obstacles/state", 10)
        self._services = []
        for group in sorted({item.trigger_group for item in self.scenario.obstacles if item.trigger_group}):
            def trigger_callback(request, response, group=group):
                activated = self.trigger(group, simulation_time())
                response.success = bool(activated)
                response.message = json.dumps({"group": group, "activated": activated})
                return response
            self._services.append(node.create_service(Trigger, f"/experiment/obstacles/{group}/trigger", trigger_callback))
        def reset_callback(request, response):
            self.reset(self.scenario.seed)
            response.success = True
            response.message = "obstacles reset"
            return response
        self._services.append(node.create_service(Trigger, "/experiment/obstacles/reset", reset_callback))

    def _publish_state(self) -> None:
        if not hasattr(self, "_state_publisher"):
            return
        from std_msgs.msg import String
        message = String()
        message.data = json.dumps(self.state(), separators=(",", ":"))
        self._state_publisher.publish(message)

    def update(self, simulation_time: float) -> None:
        if not self.scenario.enabled:
            return
        if self._reset_time is None:
            self._reset_time = simulation_time
        from pxr import Gf
        for identifier, runtime in self._runtime.items():
            spec = runtime.spec
            if runtime.active_at is None or runtime.retired:
                continue
            elapsed = max(0.0, simulation_time - runtime.active_at - spec.delay_sec)
            if not runtime.motion_started and simulation_time >= runtime.active_at + spec.delay_sec:
                runtime.motion_started = True
                self._event("motion_start", simulation_time, obstacle_id=identifier)
            delta = tuple(end - start for start, end in zip(spec.start, spec.end))
            distance = math.sqrt(sum(value * value for value in delta))
            duration = distance / spec.speed if spec.mode == "linear" else math.inf
            progress = (elapsed + runtime.phase) / duration if math.isfinite(duration) else 0.0
            if spec.repeat:
                normalized = progress % 2.0
                fraction = normalized if normalized <= 1.0 else 2.0 - normalized
            else:
                fraction = min(1.0, max(0.0, progress))
            runtime.progress = fraction
            position = tuple(start + fraction * value for start, value in zip(spec.start, delta))
            runtime.translate_op.Set(Gf.Vec3d(*self._world_position(position)))
            if not spec.repeat and progress >= 1.0 and spec.post_motion == "retire":
                if not runtime.completion_recorded:
                    runtime.completion_recorded = True
                    self._event("motion_complete", simulation_time, obstacle_id=identifier, progress=fraction)
                runtime.retired = True
                self._set_enabled(runtime, False)
                self._event("retire", simulation_time, obstacle_id=identifier)
        self._publish_state()
