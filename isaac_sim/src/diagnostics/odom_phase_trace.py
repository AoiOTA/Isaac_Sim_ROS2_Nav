"""Default-off trace support for the Stage 2.2-R2A3 odometry phase probe."""

from __future__ import annotations

import json
import math
from pathlib import Path
import time
from typing import Any


SCHEMA = "bio_nav_stage2_2_r2a3_odom_phase_trace_v1"


def stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def yaw_from_wxyz(value: tuple[float, float, float, float]) -> float:
    w, x, y, z = value
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class OdomPhaseScript:
    """Fixed, simulation-time command schedule used only by the probe."""

    segments = (
        (5.0, 0.0, 0.0, "idle"),
        (10.0, 0.5, 0.0, "straight"),
        (10.0, 0.0, 1.0, "spin_left"),
        (10.0, 0.0, -1.0, "spin_right"),
        (10.0, 0.5, 0.5, "arc_left"),
        (10.0, 0.5, -0.5, "arc_right"),
    )
    settle_s = 1.0

    def __init__(self) -> None:
        self._started_at: float | None = None

    def command(self, simulation_time: float) -> tuple[float, float, str] | None:
        if self._started_at is None:
            self._started_at = float(simulation_time)
        elapsed = float(simulation_time) - self._started_at
        boundary = 0.0
        for duration, linear_x, angular_z, name in self.segments:
            boundary += duration
            if elapsed < boundary:
                return linear_x, angular_z, name
        return None

    def complete(self, simulation_time: float) -> bool:
        if self._started_at is None:
            return False
        return float(simulation_time) - self._started_at >= sum(item[0] for item in self.segments) + self.settle_s

    @classmethod
    def required_end_timecode(cls, rendering_hz: float) -> int:
        """Return the Stage end code needed for the fixed probe plus margin."""
        return int(math.ceil((sum(item[0] for item in cls.segments) + cls.settle_s + 2.0) * float(rendering_hz)))


class OdomPhaseTrace:
    """Append-only JSONL trace; callers own the Isaac and ROS lifecycle."""

    def __init__(self, path: Path, *, publish_raw_velocities: bool = False, stage_end_timecode: int | None = None) -> None:
        self.path = Path(path).expanduser().resolve()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._output = self.path.open("x", encoding="utf-8")
        self._ordinal_by_stamp: dict[int, int] = {}
        manifest: dict[str, object] = {
            "schema": SCHEMA,
            "kind": "manifest",
            "publish_raw_velocities": bool(publish_raw_velocities),
        }
        if stage_end_timecode is not None:
            manifest["stage_end_timecode"] = int(stage_end_timecode)
        self._write(manifest)

    def _write(self, value: dict[str, object]) -> None:
        self._output.write(json.dumps(value, sort_keys=True) + "\n")
        self._output.flush()

    def snapshot(
        self,
        *,
        phase: str,
        loop_sequence: int,
        simulation_time: float,
        robot: Any,
        motion_assist: Any,
        command: tuple[float, float, str] | None,
    ) -> None:
        position, orientation = robot.get_world_pose()
        linear, angular = robot.get_base_velocities()
        state = motion_assist.state
        self._write({
            "schema": SCHEMA,
            "kind": "snapshot",
            "phase": str(phase),
            "loop_sequence": int(loop_sequence),
            "simulation_time_s": float(simulation_time),
            "recorded_monotonic_ns": time.monotonic_ns(),
            "position": [float(item) for item in position],
            "orientation_wxyz": [float(item) for item in orientation],
            "yaw_rad": yaw_from_wxyz(tuple(float(item) for item in orientation)),
            "linear_xyz": [float(item) for item in linear],
            "angular_xyz": [float(item) for item in angular],
            "motion_assist_target": [float(state.target_linear_speed), float(state.target_yaw_rate)],
            "motion_assist_applied": (
                None
                if state.last_applied_linear_speed is None
                or state.last_applied_yaw_rate is None
                else [
                    float(state.last_applied_linear_speed),
                    float(state.last_applied_yaw_rate),
                ]
            ),
            "motion_assist_last_command_at_s": state.last_command_at,
            "script_command": None if command is None else [float(command[0]), float(command[1]), command[2]],
        })

    def record_odom(self, message: Any, *, loop_sequence: int) -> None:
        pose = message.pose.pose
        twist = message.twist.twist
        stamp = stamp_ns(message.header.stamp)
        ordinal = self._ordinal_by_stamp.get(stamp, 0)
        self._ordinal_by_stamp[stamp] = ordinal + 1
        self._write({
            "schema": SCHEMA,
            "kind": "odom_receive",
            "loop_sequence": int(loop_sequence),
            "header_stamp_ns": stamp,
            "same_stamp_ordinal": ordinal,
            "recorded_monotonic_ns": time.monotonic_ns(),
            "position": [float(pose.position.x), float(pose.position.y), float(pose.position.z)],
            "orientation_xyzw": [float(pose.orientation.x), float(pose.orientation.y), float(pose.orientation.z), float(pose.orientation.w)],
            "linear_xyz": [float(twist.linear.x), float(twist.linear.y), float(twist.linear.z)],
            "angular_xyz": [float(twist.angular.x), float(twist.angular.y), float(twist.angular.z)],
        })

    def close(self) -> None:
        if not self._output.closed:
            self._output.close()
