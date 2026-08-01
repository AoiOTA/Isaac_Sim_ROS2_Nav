"""Default-off trace support for the Stage 2.2-R2B odometry phase probe."""

from __future__ import annotations

import json
import math
from pathlib import Path
import time
from typing import Any


SCHEMA = "bio_nav_stage2_2_r2b_odom_phase_trace_v2"


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
        # Bridge callbacks can arrive after the next app update.  Associate
        # them with the graph trigger by authoritative header stamp, never by
        # callback arrival order.
        self._trigger_by_stamp: dict[int, dict[str, object]] = {}
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
        odom_publish: dict[str, object] | None = None,
    ) -> None:
        position, orientation = robot.get_world_pose()
        linear, angular = robot.get_base_velocities()
        state = motion_assist.state
        source_payload = {
            "position": [float(item) for item in position],
            "yaw_rad": yaw_from_wxyz(tuple(float(item) for item in orientation)),
            "linear_xyz": [float(item) for item in linear],
            "angular_xyz": [float(item) for item in angular],
        }
        row: dict[str, object] = {
            "schema": SCHEMA,
            "kind": "snapshot",
            "phase": str(phase),
            "loop_sequence": int(loop_sequence),
            "simulation_time_s": float(simulation_time),
            "recorded_monotonic_ns": time.monotonic_ns(),
            "position": source_payload["position"],
            "orientation_wxyz": [float(item) for item in orientation],
            "yaw_rad": source_payload["yaw_rad"],
            "linear_xyz": source_payload["linear_xyz"],
            "angular_xyz": source_payload["angular_xyz"],
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
            "ideal_odom_publish": odom_publish,
        }
        if phase == "after_odom_trigger":
            row["graph_epoch"] = None if odom_publish is None else odom_publish["graph_epoch"]
            row["loop_publish_count"] = None if odom_publish is None else odom_publish["loop_publish_count"]
            row["trigger_status"] = None if odom_publish is None else odom_publish["trigger_status"]
            row["evaluate_status"] = None if odom_publish is None else odom_publish["evaluate_status"]
            row["source_payload"] = source_payload
            if odom_publish is not None:
                # These values are read directly from ComputeOdometry outputs
                # and ROS2PublishOdometry inputs after evaluate_sync.  They
                # are in odom coordinates, unlike robot world-base pose.
                row["source_payload"] = odom_publish.get("source_payload", source_payload)
                row["publisher_payload"] = odom_publish.get("publisher_payload")
        self._write(row)

    def record_odom_trigger(
        self, receipt: dict[str, object], *, simulation_time: float
    ) -> None:
        """Record the synchronous graph trigger before ROS delivery occurs."""

        stamp = int(round(float(simulation_time) * 1_000_000_000.0))
        trigger = {**receipt, "expected_header_stamp_ns": stamp}
        self._trigger_by_stamp[stamp] = trigger
        self._write({
            "schema": SCHEMA,
            "kind": "ideal_odom_trigger",
            **trigger,
        })

    def _trigger_for_stamp(self, stamp: int) -> dict[str, object]:
        exact = self._trigger_by_stamp.get(stamp)
        if exact is not None:
            return exact
        # Simulation time is a binary float while ROS stamps are integer ns.
        # Accept only the one-nanosecond round-trip difference, never a
        # callback-arrival-based or broad temporal match.
        candidates = [
            value
            for candidate_stamp, value in self._trigger_by_stamp.items()
            if abs(candidate_stamp - stamp) <= 1
        ]
        return candidates[0] if len(candidates) == 1 else {}

    def record_odom(self, message: Any) -> None:
        pose = message.pose.pose
        twist = message.twist.twist
        stamp = stamp_ns(message.header.stamp)
        trigger = self._trigger_for_stamp(stamp)
        ordinal = self._ordinal_by_stamp.get(stamp, 0)
        self._ordinal_by_stamp[stamp] = ordinal + 1
        self._write({
            "schema": SCHEMA,
            "kind": "odom_receive",
            "loop_sequence": int(trigger.get("loop_sequence", -1)),
            "header_stamp_ns": stamp,
            "same_stamp_ordinal": ordinal,
            "recorded_monotonic_ns": time.monotonic_ns(),
            "position": [float(pose.position.x), float(pose.position.y), float(pose.position.z)],
            "orientation_xyzw": [float(pose.orientation.x), float(pose.orientation.y), float(pose.orientation.z), float(pose.orientation.w)],
            "linear_xyz": [float(twist.linear.x), float(twist.linear.y), float(twist.linear.z)],
            "angular_xyz": [float(twist.angular.x), float(twist.angular.y), float(twist.angular.z)],
            # This row is the observed publisher payload, deliberately kept
            # separate from the source-state snapshot above.
            "publisher_payload_observed": True,
            "publisher_payload": {
                "position": [float(pose.position.x), float(pose.position.y), float(pose.position.z)],
                "yaw_rad": yaw_from_wxyz((
                    float(pose.orientation.w), float(pose.orientation.x),
                    float(pose.orientation.y), float(pose.orientation.z),
                )),
                "linear_xyz": [float(twist.linear.x), float(twist.linear.y), float(twist.linear.z)],
                "angular_xyz": [float(twist.angular.x), float(twist.angular.y), float(twist.angular.z)],
            },
        })

    def record_tf(self, message: Any) -> None:
        """Record only the ideal odom->base transform for per-loop parity."""

        for transform in message.transforms:
            if transform.header.frame_id != "odom" or transform.child_frame_id != "base_link":
                continue
            stamp = stamp_ns(transform.header.stamp)
            trigger = self._trigger_for_stamp(stamp)
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            self._write({
                "schema": SCHEMA,
                "kind": "tf_receive",
                "loop_sequence": int(trigger.get("loop_sequence", -1)),
                "header_stamp_ns": stamp,
                "recorded_monotonic_ns": time.monotonic_ns(),
                "publisher_payload": {
                    "position": [float(translation.x), float(translation.y), float(translation.z)],
                    "yaw_rad": yaw_from_wxyz((
                        float(rotation.w), float(rotation.x), float(rotation.y), float(rotation.z),
                    )),
                },
            })

    def close(self) -> None:
        if not self._output.closed:
            self._output.close()
