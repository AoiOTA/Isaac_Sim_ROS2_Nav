"""Default-off Stage 2.2-R2C1 free-space odometry probe support.

Nothing in this module participates in navigation.  It owns the small,
repeatable Isaac-only script and the append-only trace used to audit the
causal chain from the motion-assist write to the delivered ROS odometry.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import threading
import time
from typing import Any


SCHEMA = "bio_nav_stage2_2_r2c1_free_space_probe_trace_v3"
REQUIRED_CLEARANCE_M = 0.20


def stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def yaw_from_wxyz(value: tuple[float, float, float, float]) -> float:
    w, x, y, z = value
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


@dataclass(frozen=True)
class ProbeSegment:
    segment_id: str
    linear_x: float
    angular_z: float


class SegmentedFreeSpaceScript:
    """Six short, reset-separated movements at the calibrated open spawn."""

    segments = (
        ProbeSegment("idle", 0.0, 0.0),
        ProbeSegment("straight", 0.2, 0.0),
        ProbeSegment("spin_left", 0.0, 0.5),
        ProbeSegment("spin_right", 0.0, -0.5),
        ProbeSegment("arc_left", 0.2, 0.5),
        ProbeSegment("arc_right", 0.2, -0.5),
    )
    settle_s = 2.0
    action_s = 3.0
    idle_s = 1.0

    @classmethod
    def segment_duration_s(cls) -> float:
        return cls.settle_s + cls.action_s + cls.idle_s

    @classmethod
    def required_end_timecode(cls, rendering_hz: float) -> int:
        duration = len(cls.segments) * cls.segment_duration_s() + 2.0
        return int(math.ceil(duration * float(rendering_hz)))

    @classmethod
    def phase(
        cls, elapsed_s: float, segment: ProbeSegment
    ) -> tuple[float, float, str]:
        if elapsed_s < cls.settle_s:
            return 0.0, 0.0, "settle"
        if elapsed_s < cls.settle_s + cls.action_s:
            return segment.linear_x, segment.angular_z, "action"
        return 0.0, 0.0, "idle"


def payload_from_robot(robot: Any) -> dict[str, object]:
    """Read one direct articulation state without depending on Kit types."""

    position, orientation = robot.get_world_pose()
    linear, angular = robot.get_base_velocities()
    return {
        "position": [float(item) for item in position],
        "yaw_rad": yaw_from_wxyz(tuple(float(item) for item in orientation)),
        "linear_xyz": [float(item) for item in linear],
        "angular_xyz": [float(item) for item in angular],
    }


def wheel_velocities(robot: Any) -> list[float] | None:
    """Best-effort diagnostic read; unavailable adapters must remain usable."""

    articulation = getattr(robot, "articulation", None)
    getter = getattr(articulation, "get_dof_velocities", None)
    if not callable(getter):
        return None


def swept_xy_points(
    segment: ProbeSegment, *, yaw_rad: float, samples: int = 61
) -> list[tuple[float, float]]:
    """Sample the commanded 3 s body sweep in the local horizontal plane."""

    if samples < 2:
        raise ValueError("swept clearance requires at least two samples")
    points: list[tuple[float, float]] = []
    for index in range(samples):
        elapsed = SegmentedFreeSpaceScript.action_s * index / (samples - 1)
        if abs(segment.angular_z) < 1.0e-12:
            local_x, local_y = segment.linear_x * elapsed, 0.0
        else:
            radius = segment.linear_x / segment.angular_z
            local_x = radius * math.sin(segment.angular_z * elapsed)
            local_y = radius * (1.0 - math.cos(segment.angular_z * elapsed))
        points.append((
            math.cos(yaw_rad) * local_x - math.sin(yaw_rad) * local_y,
            math.sin(yaw_rad) * local_x + math.cos(yaw_rad) * local_y,
        ))
    return points


def minimum_xy_clearance(
    *, start_xy: tuple[float, float], yaw_rad: float, segment: ProbeSegment,
    obstacle_bounds_xy: list[tuple[float, float, float, float]],
    footprint_radius_m: float = 0.33,
) -> float:
    """Return conservative point-to-AABB clearance for one commanded sweep."""

    if footprint_radius_m <= 0.0 or not math.isfinite(footprint_radius_m):
        raise ValueError("footprint_radius_m must be finite and positive")
    if not obstacle_bounds_xy:
        raise ValueError("free-space preflight found no static collision bounds")
    minimum = math.inf
    for local_x, local_y in swept_xy_points(segment, yaw_rad=yaw_rad):
        point_x, point_y = start_xy[0] + local_x, start_xy[1] + local_y
        for min_x, min_y, max_x, max_y in obstacle_bounds_xy:
            dx = max(min_x - point_x, 0.0, point_x - max_x)
            dy = max(min_y - point_y, 0.0, point_y - max_y)
            minimum = min(minimum, math.hypot(dx, dy) - footprint_radius_m)
    return minimum
    values = getter()
    numpy = getattr(values, "numpy", None)
    values = numpy() if callable(numpy) else values
    try:
        return [float(item) for item in values[0]]
    except (IndexError, TypeError):
        return None


class R2C1Trace:
    """Thread-safe append-only trace with header-stamp callback association."""

    def __init__(self, path: Path, *, manifest: dict[str, object]) -> None:
        self.path = Path(path).expanduser().resolve()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._output = self.path.open("x", encoding="utf-8")
        self._lock = threading.Lock()
        self._trigger_by_stamp: dict[int, dict[str, object]] = {}
        self._same_stamp_ordinals: dict[int, int] = {}
        self._latest_collision = False
        self._collision_count = 0
        self._write({"schema": SCHEMA, "kind": "manifest", **manifest})

    def _write(self, value: dict[str, object]) -> None:
        with self._lock:
            self._output.write(json.dumps(value, sort_keys=True) + "\n")
            self._output.flush()

    @property
    def collision_count(self) -> int:
        with self._lock:
            return self._collision_count

    @property
    def latest_collision(self) -> bool:
        with self._lock:
            return self._latest_collision

    def record_segment_reset(
        self, *, segment_index: int, segment_id: str, reset_epoch: int,
        simulation_time_s: float, status: str,
    ) -> None:
        with self._lock:
            self._latest_collision = False
            self._collision_count = 0
        self._write({
            "schema": SCHEMA, "kind": "segment_reset", "segment_index": int(segment_index),
            "segment_id": str(segment_id), "reset_epoch": int(reset_epoch),
            "simulation_time_s": float(simulation_time_s), "status": str(status),
        })

    def record_preflight(
        self, *, segment_index: int, segment_id: str, clearance_m: float,
        valid: bool,
    ) -> None:
        self._write({
            "schema": SCHEMA, "kind": "segment_preflight", "segment_index": int(segment_index),
            "segment_id": str(segment_id), "clearance_m": float(clearance_m),
            "required_clearance_m": REQUIRED_CLEARANCE_M, "valid": bool(valid),
        })

    def snapshot(
        self, *, phase: str, loop_sequence: int, reset_epoch: int,
        segment_index: int, segment_id: str, segment_phase: str,
        simulation_time_s: float, robot: Any, motion_assist: Any,
    ) -> dict[str, object]:
        payload = payload_from_robot(robot)
        state = motion_assist.state
        row: dict[str, object] = {
            "schema": SCHEMA, "kind": "snapshot", "phase": str(phase),
            "loop_sequence": int(loop_sequence), "reset_epoch": int(reset_epoch),
            "segment_index": int(segment_index), "segment_id": str(segment_id),
            "segment_phase": str(segment_phase), "simulation_time_s": float(simulation_time_s),
            "recorded_monotonic_ns": time.monotonic_ns(),
            "realized_payload": payload, "wheel_velocities": wheel_velocities(robot),
            "motion_assist_target": [float(state.target_linear_speed), float(state.target_yaw_rate)],
            "motion_assist_applied": None if state.last_applied_linear_speed is None or state.last_applied_yaw_rate is None else [float(state.last_applied_linear_speed), float(state.last_applied_yaw_rate)],
            "collision_detected": self.latest_collision,
        }
        self._write(row)
        return payload

    def record_trigger(
        self, receipt: dict[str, object], *, simulation_time_s: float,
        loop_sequence: int, reset_epoch: int, segment_index: int, segment_id: str,
        post_assist_payload: dict[str, object],
    ) -> None:
        header_stamp_ns = int(round(float(simulation_time_s) * 1_000_000_000.0))
        registered = {
            "loop_sequence": int(loop_sequence), "reset_epoch": int(reset_epoch),
            "segment_index": int(segment_index), "segment_id": str(segment_id),
            "header_stamp_ns": header_stamp_ns,
        }
        self._trigger_by_stamp[header_stamp_ns] = registered
        self._write({
            "schema": SCHEMA, "kind": "ideal_odom_trigger", **registered,
            "recorded_sequence": int(loop_sequence), "sim_time_ns": header_stamp_ns,
            "loop_publish_count": int(receipt["loop_publish_count"]),
            "post_assist_payload": post_assist_payload,
            "source_payload": receipt.get("source_payload"),
            "compute_payload": receipt.get("source_payload"),
            "compute_odometry_payload": receipt.get("source_payload"),
            "publisher_payload": receipt.get("publisher_payload"),
            "realized_next_payload": None,
        })

    def record_realized_next(
        self, *, trigger_loop_sequence: int, reset_epoch: int,
        simulation_time_s: float, payload: dict[str, object],
    ) -> None:
        self._write({
            "schema": SCHEMA, "kind": "realized_next_state",
            "trigger_loop_sequence": int(trigger_loop_sequence), "reset_epoch": int(reset_epoch),
            "simulation_time_s": float(simulation_time_s), "realized_next_payload": payload,
        })

    def _trigger(self, header_stamp_ns: int) -> dict[str, object]:
        exact = self._trigger_by_stamp.get(header_stamp_ns)
        if exact is not None:
            return exact
        candidates = [
            value for stamp, value in self._trigger_by_stamp.items()
            if abs(stamp - header_stamp_ns) <= 1
        ]
        return candidates[0] if len(candidates) == 1 else {}

    def record_odom(self, message: Any, *, arrival_loop_sequence: int) -> None:
        pose, twist = message.pose.pose, message.twist.twist
        header_stamp_ns = stamp_ns(message.header.stamp)
        trigger = self._trigger(header_stamp_ns)
        ordinal = self._same_stamp_ordinals.get(header_stamp_ns, 0)
        self._same_stamp_ordinals[header_stamp_ns] = ordinal + 1
        received_ns = time.monotonic_ns()
        trigger_loop = trigger.get("loop_sequence")
        latency_ticks = (
            None if not isinstance(trigger_loop, int)
            else int(arrival_loop_sequence) - trigger_loop
        )
        self._write({
            "schema": SCHEMA, "kind": "odom_receive", **trigger,
            "header_stamp_ns": header_stamp_ns, "same_stamp_ordinal": ordinal,
            "arrival_loop_sequence": int(arrival_loop_sequence),
            "callback_monotonic_ns": received_ns,
            "callback_latency_ns": None, "callback_latency_ticks": latency_ticks,
            "publisher_payload": {
                "position": [float(pose.position.x), float(pose.position.y), float(pose.position.z)],
                "yaw_rad": yaw_from_wxyz((float(pose.orientation.w), float(pose.orientation.x), float(pose.orientation.y), float(pose.orientation.z))),
                "linear_xyz": [float(twist.linear.x), float(twist.linear.y), float(twist.linear.z)],
                "angular_xyz": [float(twist.angular.x), float(twist.angular.y), float(twist.angular.z)],
            },
        })

    def record_tf(self, message: Any, *, arrival_loop_sequence: int) -> None:
        for transform in message.transforms:
            if transform.header.frame_id != "odom" or transform.child_frame_id != "base_link":
                continue
            header_stamp_ns = stamp_ns(transform.header.stamp)
            trigger = self._trigger(header_stamp_ns)
            translation, rotation = transform.transform.translation, transform.transform.rotation
            self._write({
                "schema": SCHEMA, "kind": "tf_receive", **trigger,
                "header_stamp_ns": header_stamp_ns,
                "arrival_loop_sequence": int(arrival_loop_sequence),
                "callback_monotonic_ns": time.monotonic_ns(), "callback_latency_ns": None,
                "callback_latency_ticks": (
                    None if not isinstance(trigger.get("loop_sequence"), int)
                    else int(arrival_loop_sequence) - int(trigger["loop_sequence"])
                ),
                "publisher_payload": {
                    "position": [float(translation.x), float(translation.y), float(translation.z)],
                    "yaw_rad": yaw_from_wxyz((float(rotation.w), float(rotation.x), float(rotation.y), float(rotation.z))),
                },
            })

    def record_collision(self, message: Any, *, reset_epoch: int) -> None:
        value = bool(message.data)
        with self._lock:
            self._latest_collision = value
            if value:
                self._collision_count += 1
        self._write({
            "schema": SCHEMA, "kind": "collision_receive", "reset_epoch": int(reset_epoch),
            "collision_detected": value, "callback_monotonic_ns": time.monotonic_ns(),
        })

    def record_segment_end(
        self, *, segment_index: int, segment_id: str, reset_epoch: int,
        clearance_m: float,
    ) -> None:
        self._write({
            "schema": SCHEMA, "kind": "segment_end", "segment_index": int(segment_index),
            "segment_id": str(segment_id), "reset_epoch": int(reset_epoch),
            "collision_count": self.collision_count, "clearance_m": float(clearance_m),
        })

    def close(self) -> None:
        with self._lock:
            if not self._output.closed:
                self._output.close()
