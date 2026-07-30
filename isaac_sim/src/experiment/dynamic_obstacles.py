"""Safe, observable dynamic-obstacle authoring and state machine."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from typing import Callable

from isaac_sim.src.experiment.scenario import DynamicCase, DynamicScenario, DynamicVariant, ObstacleSpec


@dataclass
class _ObstacleRuntime:
    spec: ObstacleSpec
    translate_op: object
    collision_attr: object
    visibility_attr: object
    state: str = "waiting"
    armed_at: float | None = None
    gate_at: float | None = None
    motion_at: float | None = None
    yield_at: float | None = None
    retired: bool = False
    phase: float = 0.0
    progress: float = 0.0
    position_map: tuple[float, float, float] = (0.0, 0.0, 0.0)
    velocity_mps: float = 0.0
    min_clearance_m: float = math.inf


def _distance_point_to_segment(point, start, end) -> float:
    dx, dy = end[0] - start[0], end[1] - start[1]
    length2 = dx * dx + dy * dy
    if length2 == 0: return math.hypot(point[0] - start[0], point[1] - start[1])
    t = max(0.0, min(1.0, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length2))
    return math.hypot(point[0] - start[0] - t * dx, point[1] - start[1] - t * dy)


class DynamicObstacleManager:
    """Own physical actors and prevent kinematic actors from pushing the robot.

    For schema v3/v4, ``trigger`` only transitions to ``armed``.  A monotonic
    spatial gate plus a robot-speed threshold starts the actor exactly once.
    A pre-contact safety yield freezes a risky kinematic actor while keeping
    it visible and collidable, then resumes only once the robot has passed.
    """
    def __init__(self, stage, scenario: DynamicScenario, root_path: str = "/World/DynamicObstacles",
                 map_to_usd: Callable[[tuple[float, float, float]], tuple[float, float, float]] | None = None,
                 usd_to_map: Callable[[tuple[float, float, float]], tuple[float, float, float]] | None = None):
        self.stage, self.scenario, self.root_path = stage, scenario, root_path
        self._map_to_usd, self._usd_to_map = map_to_usd, usd_to_map
        self._runtime: dict[str, _ObstacleRuntime] = {}
        self._events: list[dict[str, object]] = []
        self._selected_cases: tuple[DynamicCase, ...] = ()
        self._selected_variants: dict[str, DynamicVariant] = {}
        self._case_by_obstacle_id: dict[str, DynamicCase] = {}
        self._last_publish_time = -math.inf
        self._last_robot: dict[str, float] | None = None
        self._author(); self.reset(scenario.seed)

    def _world_position(self, position):
        if self.scenario.coordinate_frame == "map":
            if self._map_to_usd is None: raise RuntimeError("map-coordinate obstacles require calibrated map_to_usd")
            return self._map_to_usd(position)
        return position

    def _event(self, kind: str, simulation_time: float, **detail) -> None:
        self._events.append({"event": kind, "simulation_time": round(simulation_time, 6), **detail})

    def _set_enabled(self, runtime: _ObstacleRuntime, enabled: bool) -> None:
        from pxr import UsdGeom
        runtime.collision_attr.Set(enabled)
        runtime.visibility_attr.Set(UsdGeom.Tokens.inherited if enabled else UsdGeom.Tokens.invisible)

    def _author(self) -> None:
        from pxr import Gf, UsdGeom, UsdPhysics
        UsdGeom.Xform.Define(self.stage, self.root_path)
        if not self.scenario.enabled: return
        for spec in self.scenario.obstacles:
            cube = UsdGeom.Cube.Define(self.stage, f"{self.root_path}/{spec.obstacle_id}")
            cube.CreateSizeAttr(1.0); xform = UsdGeom.Xformable(cube.GetPrim()); xform.ClearXformOpOrder()
            translate = xform.AddTranslateOp(); translate.Set(Gf.Vec3d(*self._world_position(spec.start)))
            xform.AddScaleOp().Set(Gf.Vec3d(*spec.size))
            collision = UsdPhysics.CollisionAPI.Apply(cube.GetPrim()).CreateCollisionEnabledAttr(True)
            rigid = UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim()); rigid.CreateRigidBodyEnabledAttr(True); rigid.CreateKinematicEnabledAttr(True)
            UsdPhysics.MassAPI.Apply(cube.GetPrim()).CreateMassAttr(spec.mass)
            visibility = UsdGeom.Imageable(cube.GetPrim()).CreateVisibilityAttr()
            self._runtime[spec.obstacle_id] = _ObstacleRuntime(spec, translate, collision, visibility, position_map=spec.start)

    def reset(self, seed: int, case_id: str | None = None, variant_id: str | int | None = None) -> None:
        from pxr import Gf
        self._events.clear()
        self._selected_cases = self.scenario.selected_cases(case_id) if self.scenario.is_case_matrix else ()
        self._selected_variants = {
            item.case_id: item.variant(variant_id) for item in self._selected_cases
        }
        self._case_by_obstacle_id = {
            item.obstacle.obstacle_id: item for item in self._selected_cases
        }
        phases = self.scenario.sampled_phases(seed)
        for identifier, runtime in self._runtime.items():
            active = not self._selected_cases or identifier in self._case_by_obstacle_id
            runtime.state, runtime.armed_at, runtime.gate_at, runtime.motion_at, runtime.yield_at, runtime.retired = "waiting", None, None, None, None, not active
            runtime.phase, runtime.progress, runtime.velocity_mps, runtime.min_clearance_m = phases[identifier], 0.0, 0.0, math.inf
            runtime.position_map = runtime.spec.start; runtime.translate_op.Set(Gf.Vec3d(*self._world_position(runtime.spec.start)))
            self._set_enabled(runtime, active and runtime.spec.trigger_group is None)
        self._event(
            "reset", 0.0, seed=seed,
            case_id=case_id,
            case_ids=[item.case_id for item in self._selected_cases],
            variant_id=str(variant_id) if variant_id is not None else None,
        )

    def trigger(self, group: str, simulation_time: float) -> tuple[str, ...]:
        activated = []
        for identifier, runtime in self._runtime.items():
            if (
                runtime.spec.trigger_group == group
                and runtime.state == "waiting"
                and not runtime.retired
                and (not self._selected_cases or identifier in self._case_by_obstacle_id)
            ):
                runtime.state, runtime.armed_at, runtime.gate_at = "armed", simulation_time, None
                # Schema v4 deliberately keeps an armed actor hidden until the
                # spatial gate.  Otherwise it becomes a static global-costmap
                # obstacle for several seconds before the interaction.
                if not self.scenario.is_case_matrix:
                    self._set_enabled(runtime, True)
                activated.append(identifier)
                self._event("armed", simulation_time, obstacle_id=identifier, group=group)
        return tuple(activated)

    def complete(self, group: str, simulation_time: float) -> tuple[str, ...]:
        """Retire actors only after their corresponding Nav2 goal succeeds.

        The operation is intentionally idempotent so a runner retry cannot
        re-enable, move, or otherwise alter an already retired actor.
        """
        retired = []
        for identifier, runtime in self._runtime.items():
            if (
                runtime.spec.trigger_group != group
                or runtime.retired
                or runtime.state == "waiting"
                or (self._selected_cases and identifier not in self._case_by_obstacle_id)
            ):
                continue
            previous_state = runtime.state
            runtime.state, runtime.retired, runtime.velocity_mps = "retired", True, 0.0
            self._set_enabled(runtime, False)
            retired.append(identifier)
            self._event(
                "goal_reached_retire", simulation_time,
                obstacle_id=identifier, group=group, previous_state=previous_state,
            )
        return tuple(retired)

    def _gate_passed(self, case: DynamicCase, robot: dict[str, float]) -> bool:
        axis_value = robot[case.gate.axis]
        if case.gate.direction == "positive" and axis_value < case.gate.threshold: return False
        if case.gate.direction == "negative" and axis_value > case.gate.threshold: return False
        if case.gate.x_range and not case.gate.x_range[0] <= robot["x"] <= case.gate.x_range[1]: return False
        if case.gate.max_distance_to_obstacle_start_m is not None:
            distance = math.dist(
                (robot["x"], robot["y"]), case.waypoints[0][:2]
            )
            if distance > case.gate.max_distance_to_obstacle_start_m:
                return False
        # north/south direction is explicit because accepting G2 alone is not evidence of approach.
        heading = robot.get("vy", 0.0)
        return robot.get("speed", 0.0) >= case.gate.min_speed_mps and (heading > 0.0 if case.gate.direction == "positive" else heading < 0.0)

    @staticmethod
    def _profile(distance: float, vmax: float, accel: float, elapsed: float) -> tuple[float, float, float]:
        """Return a bounded cosine-eased rest-to-rest segment.

        A trapezoidal velocity profile changes acceleration discontinuously at
        its accelerate/cruise/brake boundaries.  That is visually apparent on
        a kinematic PhysX actor as a short twitch.  The cosine easing below
        has continuous position and velocity, while selecting its duration to
        respect both configured velocity and acceleration limits.
        """
        duration = max(
            math.pi * distance / (2.0 * vmax),
            math.sqrt(math.pi * math.pi * distance / (2.0 * accel)),
        )
        progress = min(1.0, max(0.0, elapsed / duration))
        angle = math.pi * progress
        travelled = 0.5 * distance * (1.0 - math.cos(angle))
        velocity = 0.5 * math.pi * distance / duration * math.sin(angle)
        return travelled, velocity, duration

    def _trajectory(self, case: DynamicCase, variant: DynamicVariant | None, elapsed: float) -> tuple[tuple[float, float, float], float, float, str]:
        points = case.waypoints; total_length = sum(math.dist(a, b) for a, b in zip(points, points[1:])); travelled = 0.0
        for index, (start, end) in enumerate(zip(points, points[1:])):
            length = math.dist(start, end); distance, velocity, duration = self._profile(length, case.obstacle.speed, case.max_acceleration, elapsed)
            if elapsed <= duration:
                ratio = distance / length; pos = tuple(a + ratio * (b - a) for a, b in zip(start, end))
                return pos, velocity, min(1.0, (travelled + distance) / total_length), "moving"
            elapsed -= duration; travelled += length
            if index < len(points) - 2 and variant and variant.dwell_sec:
                if elapsed <= variant.dwell_sec: return end, 0.0, travelled / total_length, "dwell"
                elapsed -= variant.dwell_sec
        return points[-1], 0.0, 1.0, "clearing"

    def _guard_clearance(self, runtime: _ObstacleRuntime, robot: dict[str, float]) -> float:
        # Conservative footprint bound: robot centre-to-box distance minus its circular bound.
        # It is independent of the PhysX contact sensor, which remains evidence only.
        half_x, half_y = runtime.spec.size[0] / 2, runtime.spec.size[1] / 2
        dx = max(abs(robot["x"] - runtime.position_map[0]) - half_x, 0.0)
        dy = max(abs(robot["y"] - runtime.position_map[1]) - half_y, 0.0)
        # The configured rectangular robot's circumscribed radius is 0.33 m.
        # Never accept a smaller, optimistic runtime value here.
        return max(0.0, math.hypot(dx, dy) - max(0.33, float(robot.get("footprint_radius", 0.33))))

    def _abort_guard(self, runtime: _ObstacleRuntime, simulation_time: float, clearance: float) -> None:
        runtime.state, runtime.retired, runtime.velocity_mps = "guard_aborted", True, 0.0; self._set_enabled(runtime, False)
        self._event("near_contact_abort", simulation_time, obstacle_id=runtime.spec.obstacle_id, clearance_m=round(clearance, 4))

    def state(self) -> dict[str, object]:
        return {"schema_version": 4, "case_ids": [item.case_id for item in self._selected_cases],
                "obstacles": [{"id": key, "trigger_group": item.spec.trigger_group,
                                "retire_group": item.spec.trigger_group, "state": item.state,
                                "progress": round(item.progress, 5), "position": list(self._reported_position(item)), "position_frame": "map" if self.scenario.coordinate_frame == "map" else "usd",
                                "velocity_mps": round(item.velocity_mps, 4),
                                "size": list(item.spec.size),
                                "min_clearance_m": None if math.isinf(item.min_clearance_m) else round(item.min_clearance_m, 4)}
                               for key, item in sorted(self._runtime.items())], "events": list(self._events)}

    def _reported_position(self, runtime):
        if self.scenario.coordinate_frame == "map": return runtime.position_map
        value = runtime.translate_op.Get(); return float(value[0]), float(value[1]), float(value[2])

    def bind_ros(self, node, simulation_time: Callable[[], float]) -> None:
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        self._state_publisher = node.create_publisher(String, "/experiment/obstacles/state", 10)
        try:
            from visualization_msgs.msg import MarkerArray
            self._marker_type = MarkerArray; self._marker_publisher = node.create_publisher(MarkerArray, "/experiment/dynamic_obstacles/markers", 10)
        except ImportError: self._marker_publisher = None
        self._services = []
        for group in sorted({item.trigger_group for item in self.scenario.obstacles if item.trigger_group}):
            def callback(request, response, group=group):
                activated = self.trigger(group, simulation_time()); response.success = bool(activated); response.message = json.dumps({"group": group, "activated": activated}); return response
            self._services.append(node.create_service(Trigger, f"/experiment/obstacles/{group}/trigger", callback))
            def complete_callback(request, response, group=group):
                retired = self.complete(group, simulation_time())
                # Goal completion is safe to retry: an already-retired actor
                # simply yields an empty list instead of failing the runner.
                response.success = True
                response.message = json.dumps({"group": group, "retired": retired})
                return response
            self._services.append(node.create_service(Trigger, f"/experiment/obstacles/{group}/complete", complete_callback))
        def reset_callback(request, response): self.reset(self.scenario.seed); response.success = True; response.message = "obstacles reset"; return response
        self._services.append(node.create_service(Trigger, "/experiment/obstacles/reset", reset_callback))

    def _publish(self, simulation_time: float) -> None:
        if simulation_time - self._last_publish_time < .05: return
        self._last_publish_time = simulation_time
        if hasattr(self, "_state_publisher"):
            from std_msgs.msg import String
            msg = String(); msg.data = json.dumps(self.state(), separators=(",", ":")); self._state_publisher.publish(msg)
        if getattr(self, "_marker_publisher", None) is not None:
            from visualization_msgs.msg import Marker
            markers = self._marker_type(); colors = {"waiting": (.5,.5,.5), "armed": (1.,.75,0.), "moving": (0.,.8,1.), "dwell": (1.,.45,0.), "safety_yield": (1.,.1,.75), "clearing": (.2,1.,.2), "parked": (.35,.35,.35), "retired": (.2,.2,.2), "guard_aborted": (1.,0.,0.)}
            for index, item in enumerate(self._runtime.values()):
                header = {"frame_id": "map", "stamp_sec": int(simulation_time), "stamp_nanosec": int((simulation_time % 1) * 1e9)}
                if item.retired or item.state == "waiting":
                    for namespace, marker_id in (("dynamic_obstacles", index), ("dynamic_obstacle_status", 100 + index)):
                        marker = Marker(); marker.header.frame_id=header["frame_id"]; marker.header.stamp.sec=header["stamp_sec"]; marker.header.stamp.nanosec=header["stamp_nanosec"]; marker.ns=namespace; marker.id=marker_id; marker.action=Marker.DELETE; markers.markers.append(marker)
                    continue
                marker = Marker(); marker.header.frame_id=header["frame_id"]; marker.header.stamp.sec=header["stamp_sec"]; marker.header.stamp.nanosec=header["stamp_nanosec"]; marker.ns="dynamic_obstacles"; marker.id=index; marker.type=Marker.CUBE; marker.action=Marker.ADD
                marker.pose.position.x, marker.pose.position.y, marker.pose.position.z=item.position_map; marker.pose.orientation.w=1.; marker.scale.x, marker.scale.y, marker.scale.z=item.spec.size
                marker.color.r, marker.color.g, marker.color.b=colors[item.state]; marker.color.a=.82; markers.markers.append(marker)
                label = Marker(); label.header=marker.header; label.ns="dynamic_obstacle_status"; label.id=100+index; label.type=Marker.TEXT_VIEW_FACING; label.action=Marker.ADD; label.pose.position.x, label.pose.position.y, label.pose.position.z=item.position_map[0],item.position_map[1],item.position_map[2]+.65; label.pose.orientation.w=1.; label.scale.z=.18; label.color.r=label.color.g=label.color.b=1.; label.color.a=1.; label.text=f"{item.spec.obstacle_id}: {item.state} v={item.velocity_mps:.2f}"; markers.markers.append(label)
            if self._selected_cases:
                from geometry_msgs.msg import Point
                for index, case in enumerate(self._selected_cases):
                    runtime = self._runtime[case.obstacle.obstacle_id]
                    path = Marker(); path.header.frame_id="map"; path.header.stamp.sec=int(simulation_time); path.header.stamp.nanosec=int((simulation_time%1)*1e9); path.ns="dynamic_future_trajectory"; path.id=300 + index
                    if runtime.retired:
                        path.action=Marker.DELETE; markers.markers.append(path); continue
                    path.type=Marker.LINE_STRIP; path.action=Marker.ADD; path.scale.x=.035; path.color.r=1.; path.color.g=.25; path.color.b=.85; path.color.a=.9
                    for waypoint in case.waypoints:
                        point=Point(); point.x,point.y,point.z=waypoint; path.points.append(point)
                    markers.markers.append(path)
            self._marker_publisher.publish(markers)

    def update(self, simulation_time: float, robot: dict[str, float] | None = None) -> None:
        if not self.scenario.enabled: return
        self._last_robot = robot
        from pxr import Gf
        for runtime in self._runtime.values():
            if runtime.retired or runtime.state == "waiting": continue
            case = self._case_by_obstacle_id.get(runtime.spec.obstacle_id)
            variant = self._selected_variants.get(case.case_id) if case is not None else None
            if runtime.state == "safety_yield":
                if robot is None:
                    continue
                clearance = self._guard_clearance(runtime, robot)
                runtime.min_clearance_m = min(runtime.min_clearance_m, clearance)
                # Keep a generous separation before resuming the kinematic
                # actor; otherwise it can repeatedly catch the robot during
                # the same overtaking manoeuvre.
                if clearance < self.scenario.guard_clearance_m + 0.20:
                    continue
                assert runtime.yield_at is not None and runtime.motion_at is not None
                runtime.motion_at += simulation_time - runtime.yield_at
                runtime.yield_at = None
                runtime.state = "moving"
                self._event("safety_resume", simulation_time, obstacle_id=runtime.spec.obstacle_id, clearance_m=round(clearance, 4))
            if runtime.state == "armed":
                if case is None:
                    runtime.state, runtime.motion_at = "moving", simulation_time
                    self._event("motion_start", simulation_time, obstacle_id=runtime.spec.obstacle_id)
                else:
                    if (
                        runtime.gate_at is None
                        and robot is not None
                        and self._gate_passed(case, robot)
                    ):
                        runtime.gate_at = simulation_time
                        self._set_enabled(runtime, True)
                        self._event("gate_enter", simulation_time, obstacle_id=runtime.spec.obstacle_id)
                    if (
                        runtime.gate_at is not None
                        and simulation_time >= runtime.gate_at + (variant.start_delay_sec if variant else 0.0)
                    ):
                        runtime.state, runtime.motion_at = "moving", simulation_time
                        self._event("motion_start", simulation_time, obstacle_id=runtime.spec.obstacle_id)
                    else:
                        continue
            if runtime.state in {"moving", "dwell", "clearing"}:
                if case:
                    position, velocity, progress, state = self._trajectory(case, variant, max(0., simulation_time - runtime.motion_at))
                    runtime.position_map, runtime.velocity_mps, runtime.progress, runtime.state = position, velocity, progress, state
                    runtime.translate_op.Set(Gf.Vec3d(*self._world_position(position)))
                    if robot is not None:
                        clearance = self._guard_clearance(runtime, robot); runtime.min_clearance_m = min(runtime.min_clearance_m, clearance)
                        if clearance <= self.scenario.guard_clearance_m:
                            # Do not make the obstacle disappear.  A kinematic
                            # actor must yield before contact rather than push
                            # the robot or be removed from the scene.
                            runtime.state, runtime.velocity_mps, runtime.yield_at = "safety_yield", 0.0, simulation_time
                            self._event("safety_yield", simulation_time, obstacle_id=runtime.spec.obstacle_id, clearance_m=round(clearance, 4))
                            continue
                    if state == "clearing":
                        self._event("motion_complete", simulation_time, obstacle_id=runtime.spec.obstacle_id)
                        if runtime.spec.post_motion == "park":
                            runtime.state = "parked"
                            self._event("park", simulation_time, obstacle_id=runtime.spec.obstacle_id)
                        else:
                            runtime.retired = True
                            self._set_enabled(runtime, False)
                            self._event("retire", simulation_time, obstacle_id=runtime.spec.obstacle_id)
                else:
                    elapsed=max(0., simulation_time-runtime.armed_at-runtime.spec.delay_sec); delta=tuple(b-a for a,b in zip(runtime.spec.start,runtime.spec.end)); duration=math.dist(runtime.spec.start,runtime.spec.end)/runtime.spec.speed; fraction=min(1.,elapsed/duration); runtime.progress=fraction; runtime.position_map=tuple(a+fraction*d for a,d in zip(runtime.spec.start,delta)); runtime.translate_op.Set(Gf.Vec3d(*self._world_position(runtime.position_map)))
        self._publish(simulation_time)
