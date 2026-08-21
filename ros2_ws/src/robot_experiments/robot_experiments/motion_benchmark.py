"""Execute repeatable skid-steer primitives using estimated-state readiness."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import tempfile
import time
from typing import Any, Mapping

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


class MotionBenchmarkError(ValueError):
    """Raised when a motion benchmark configuration or run is invalid."""


@dataclass(frozen=True)
class MotionSegment:
    duration_sec: float
    linear_x: float
    angular_z: float


@dataclass(frozen=True)
class MotionPrimitive:
    identifier: str
    segments: tuple[MotionSegment, ...]


@dataclass(frozen=True)
class MotionThresholds:
    linear_mae_mps: float
    angular_mae_radps: float
    radius_relative_error_percent: float
    tracking_fraction: float
    transition_latency_sec: float
    overshoot_ratio: float
    wrong_direction_fraction: float


@dataclass(frozen=True)
class MotionConfig:
    spawn_pose_name: str
    reset_seed: int
    command_rate_hz: float
    reset_settle_sec: float
    final_settle_sec: float
    steady_window_sec: float
    thresholds: MotionThresholds
    primitives: tuple[MotionPrimitive, ...]


@dataclass(frozen=True)
class MotionSample:
    received_at: float
    stamp_s: float
    x: float
    y: float
    yaw: float
    linear_speed: float
    angular_speed: float
    segment_index: int
    segment_elapsed: float
    command_linear: float
    command_angular: float


def _finite_number(value: Any, name: str, *, positive: bool = False) -> float:
    if isinstance(value, bool):
        raise MotionBenchmarkError(f"{name} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise MotionBenchmarkError(f"{name} must be finite")
    if positive and result <= 0.0:
        raise MotionBenchmarkError(f"{name} must be positive")
    return result


def load_motion_config(path: str | Path) -> MotionConfig:
    """Load the strict motion primitive benchmark YAML."""

    source = Path(path).expanduser().resolve()
    document = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(document, dict) or document.get("schema_version") != 1:
        raise MotionBenchmarkError("motion benchmark schema_version must be 1")
    allowed = {
        "schema_version",
        "spawn_pose_name",
        "reset_seed",
        "command_rate_hz",
        "reset_settle_sec",
        "final_settle_sec",
        "steady_window_sec",
        "thresholds",
        "primitives",
    }
    unknown = set(document) - allowed
    if unknown:
        raise MotionBenchmarkError(
            f"unknown motion benchmark fields: {sorted(unknown)}"
        )
    spawn_pose_name = document.get("spawn_pose_name")
    reset_seed = document.get("reset_seed")
    if not isinstance(spawn_pose_name, str) or not spawn_pose_name:
        raise MotionBenchmarkError("spawn_pose_name must be a non-empty string")
    if (
        isinstance(reset_seed, bool)
        or not isinstance(reset_seed, int)
        or reset_seed < 0
    ):
        raise MotionBenchmarkError("reset_seed must be a non-negative integer")

    raw_thresholds = document.get("thresholds")
    if not isinstance(raw_thresholds, dict):
        raise MotionBenchmarkError("thresholds must be a mapping")
    threshold_fields = {
        "linear_mae_mps",
        "angular_mae_radps",
        "radius_relative_error_percent",
        "tracking_fraction",
        "transition_latency_sec",
        "overshoot_ratio",
        "wrong_direction_fraction",
    }
    if set(raw_thresholds) != threshold_fields:
        raise MotionBenchmarkError(
            "thresholds must contain exactly "
            + ", ".join(sorted(threshold_fields))
        )
    thresholds = MotionThresholds(
        **{
            name: _finite_number(
                raw_thresholds[name],
                f"thresholds.{name}",
                positive=True,
            )
            for name in sorted(threshold_fields)
        }
    )
    if not 0.0 < thresholds.tracking_fraction <= 1.0:
        raise MotionBenchmarkError("tracking_fraction must be in (0, 1]")
    if not 0.0 < thresholds.wrong_direction_fraction <= 1.0:
        raise MotionBenchmarkError(
            "wrong_direction_fraction must be in (0, 1]"
        )
    if thresholds.overshoot_ratio < 1.0:
        raise MotionBenchmarkError("overshoot_ratio must be at least 1")

    raw_primitives = document.get("primitives")
    if not isinstance(raw_primitives, list) or not raw_primitives:
        raise MotionBenchmarkError("primitives must be a non-empty list")
    primitives: list[MotionPrimitive] = []
    identifiers: set[str] = set()
    for primitive_index, raw_primitive in enumerate(raw_primitives):
        if not isinstance(raw_primitive, dict) or set(raw_primitive) != {
            "id",
            "segments",
        }:
            raise MotionBenchmarkError(
                f"primitives[{primitive_index}] requires id and segments"
            )
        identifier = raw_primitive["id"]
        if (
            not isinstance(identifier, str)
            or not identifier
            or identifier in identifiers
        ):
            raise MotionBenchmarkError(
                f"invalid or duplicate primitive id {identifier!r}"
            )
        identifiers.add(identifier)
        raw_segments = raw_primitive["segments"]
        if not isinstance(raw_segments, list) or not raw_segments:
            raise MotionBenchmarkError(
                f"primitive {identifier!r} requires segments"
            )
        segments: list[MotionSegment] = []
        for segment_index, raw_segment in enumerate(raw_segments):
            if not isinstance(raw_segment, dict) or set(raw_segment) != {
                "duration_sec",
                "linear_x",
                "angular_z",
            }:
                raise MotionBenchmarkError(
                    f"{identifier}.segments[{segment_index}] is invalid"
                )
            segment = MotionSegment(
                duration_sec=_finite_number(
                    raw_segment["duration_sec"],
                    f"{identifier}.segments[{segment_index}].duration_sec",
                    positive=True,
                ),
                linear_x=_finite_number(
                    raw_segment["linear_x"],
                    f"{identifier}.segments[{segment_index}].linear_x",
                ),
                angular_z=_finite_number(
                    raw_segment["angular_z"],
                    f"{identifier}.segments[{segment_index}].angular_z",
                ),
            )
            if (
                abs(segment.linear_x) < 1.0e-9
                and abs(segment.angular_z) < 1.0e-9
            ):
                raise MotionBenchmarkError(
                    f"{identifier}.segments[{segment_index}] cannot be zero"
                )
            segments.append(segment)
        primitives.append(MotionPrimitive(identifier, tuple(segments)))

    return MotionConfig(
        spawn_pose_name=spawn_pose_name,
        reset_seed=reset_seed,
        command_rate_hz=_finite_number(
            document.get("command_rate_hz"), "command_rate_hz", positive=True
        ),
        reset_settle_sec=_finite_number(
            document.get("reset_settle_sec"), "reset_settle_sec", positive=True
        ),
        final_settle_sec=_finite_number(
            document.get("final_settle_sec"), "final_settle_sec", positive=True
        ),
        steady_window_sec=_finite_number(
            document.get("steady_window_sec"), "steady_window_sec", positive=True
        ),
        thresholds=thresholds,
        primitives=tuple(primitives),
    )


def _mean(values: list[float]) -> float | None:
    return sum(values) / len(values) if values else None


def _maximum(values: list[float]) -> float | None:
    return max(values) if values else None


def _unwrap_delta(yaws: list[float]) -> float:
    return sum(
        math.atan2(math.sin(current - previous), math.cos(current - previous))
        for previous, current in zip(yaws, yaws[1:])
    )


def evaluate_motion_primitive(
    primitive: MotionPrimitive,
    samples: list[MotionSample],
    collision_detected: bool,
    thresholds: MotionThresholds,
    steady_window_sec: float,
) -> dict[str, Any]:
    """Calculate tracking, curvature, direction, and transition metrics."""

    if not samples:
        raise MotionBenchmarkError(
            f"primitive {primitive.identifier!r} has no estimated odometry samples"
        )
    steady: list[MotionSample] = []
    segment_metrics: list[dict[str, Any]] = []
    transition_latencies: list[float | None] = []
    radius_errors: list[float] = []
    wrong_direction = 0
    direction_evaluated = 0
    tracking_hits = 0
    tracking_evaluated = 0
    linear_overshoots: list[float] = []
    angular_overshoots: list[float] = []

    for index, segment in enumerate(primitive.segments):
        window = min(steady_window_sec, segment.duration_sec * 0.4)
        selected = [
            sample
            for sample in samples
            if sample.segment_index == index
            and window <= sample.segment_elapsed
            <= max(window, segment.duration_sec - 0.10)
        ]
        steady.extend(selected)
        linear_values = [sample.linear_speed for sample in selected]
        angular_values = [sample.angular_speed for sample in selected]
        mean_linear = _mean(linear_values)
        mean_angular = _mean(angular_values)
        segment_value: dict[str, Any] = {
            "segment_index": index,
            "command_linear_mps": segment.linear_x,
            "command_angular_radps": segment.angular_z,
            "steady_sample_count": len(selected),
            "mean_linear_mps": mean_linear,
            "mean_angular_radps": mean_angular,
        }
        if (
            mean_linear is not None
            and mean_angular is not None
            and abs(segment.linear_x) > 1.0e-6
            and abs(segment.angular_z) > 1.0e-6
            and abs(mean_angular) > 1.0e-6
        ):
            requested_radius = abs(segment.linear_x / segment.angular_z)
            actual_radius = abs(mean_linear / mean_angular)
            radius_error = (
                abs(actual_radius - requested_radius)
                / requested_radius
                * 100.0
            )
            radius_errors.append(radius_error)
            segment_value.update(
                {
                    "requested_radius_m": requested_radius,
                    "actual_radius_m": actual_radius,
                    "radius_relative_error_percent": radius_error,
                }
            )
        segment_metrics.append(segment_value)

        for sample in selected:
            if abs(segment.linear_x) > 1.0e-6:
                tracking_evaluated += 1
                if (
                    sample.linear_speed * segment.linear_x > 0.0
                    and abs(sample.linear_speed)
                    >= 0.70 * abs(segment.linear_x)
                ):
                    tracking_hits += 1
                direction_evaluated += 1
                if sample.linear_speed * segment.linear_x < -0.01:
                    wrong_direction += 1
                linear_overshoots.append(
                    abs(sample.linear_speed) / abs(segment.linear_x)
                )
            if abs(segment.angular_z) > 1.0e-6:
                direction_evaluated += 1
                if sample.angular_speed * segment.angular_z < -0.03:
                    wrong_direction += 1
                angular_overshoots.append(
                    abs(sample.angular_speed) / abs(segment.angular_z)
                )

        if index and primitive.segments[index - 1].angular_z * segment.angular_z < 0:
            candidates = [
                sample
                for sample in samples
                if sample.segment_index == index
                and sample.angular_speed * segment.angular_z > 0.0
                and abs(sample.angular_speed) >= 0.50 * abs(segment.angular_z)
            ]
            transition_latencies.append(
                min(sample.segment_elapsed for sample in candidates)
                if candidates
                else None
            )

    linear_errors = [
        abs(sample.linear_speed - sample.command_linear)
        for sample in steady
        if abs(sample.command_linear) > 1.0e-6
    ]
    angular_errors = [
        abs(sample.angular_speed - sample.command_angular)
        for sample in steady
        if abs(sample.command_angular) > 1.0e-6
    ]
    valid_latencies = [
        value for value in transition_latencies if value is not None
    ]
    tracking_fraction = (
        tracking_hits / tracking_evaluated if tracking_evaluated else 1.0
    )
    wrong_direction_fraction = (
        wrong_direction / direction_evaluated
        if direction_evaluated
        else 0.0
    )
    linear_mae = _mean(linear_errors)
    angular_mae = _mean(angular_errors)
    maximum_radius_error = _maximum(radius_errors)
    maximum_latency = _maximum(valid_latencies)
    maximum_overshoot = _maximum(linear_overshoots + angular_overshoots)
    path_length = sum(
        math.hypot(current.x - previous.x, current.y - previous.y)
        for previous, current in zip(samples, samples[1:])
    )
    expected_yaw = sum(
        segment.angular_z * segment.duration_sec
        for segment in primitive.segments
    )
    actual_yaw = _unwrap_delta([sample.yaw for sample in samples])

    failures: list[str] = []
    if collision_detected:
        failures.append("collision_detected")
    if linear_mae is not None and linear_mae > thresholds.linear_mae_mps:
        failures.append("linear_tracking_error")
    if angular_mae is not None and angular_mae > thresholds.angular_mae_radps:
        failures.append("angular_tracking_error")
    if (
        maximum_radius_error is not None
        and maximum_radius_error > thresholds.radius_relative_error_percent
    ):
        failures.append("curvature_tracking_error")
    if tracking_fraction < thresholds.tracking_fraction:
        failures.append("translation_tracking_fraction")
    if (
        transition_latencies
        and (
            len(valid_latencies) != len(transition_latencies)
            or maximum_latency is None
            or maximum_latency > thresholds.transition_latency_sec
        )
    ):
        failures.append("turn_reversal_latency")
    if (
        maximum_overshoot is not None
        and maximum_overshoot > thresholds.overshoot_ratio
    ):
        failures.append("velocity_overshoot")
    if wrong_direction_fraction > thresholds.wrong_direction_fraction:
        failures.append("wrong_direction")

    return {
        "id": primitive.identifier,
        "passed": not failures,
        "failure_reasons": failures,
        "collision_detected": collision_detected,
        "sample_count": len(samples),
        "steady_sample_count": len(steady),
        "path_length_m": path_length,
        "net_displacement_m": math.hypot(
            samples[-1].x - samples[0].x,
            samples[-1].y - samples[0].y,
        ),
        "expected_yaw_change_rad": expected_yaw,
        "actual_yaw_change_rad": actual_yaw,
        "linear_mae_mps": linear_mae,
        "angular_mae_radps": angular_mae,
        "maximum_radius_relative_error_percent": maximum_radius_error,
        "translation_tracking_fraction": tracking_fraction,
        "turn_transition_latencies_sec": transition_latencies,
        "maximum_turn_transition_latency_sec": maximum_latency,
        "maximum_steady_overshoot_ratio": maximum_overshoot,
        "wrong_direction_fraction": wrong_direction_fraction,
        "segments": segment_metrics,
    }


class MotionBenchmarkNode(Node):
    """ROS adapter for deterministic primitive playback and estimated capture."""

    def __init__(self, config: MotionConfig) -> None:
        super().__init__("motion_benchmark")
        self._config = config
        reliable = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(Twist, "/cmd_vel", reliable)
        clock_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._clock_subscription = self.create_subscription(
            Clock,
            "/clock",
            self._clock_callback,
            clock_qos,
        )
        self._odom_subscription = self.create_subscription(
            Odometry,
            "/odom",
            self._odom_callback,
            reliable,
        )
        self._collision_subscription = self.create_subscription(
            Bool,
            "/simulation/collision",
            self._collision_callback,
            reliable,
        )
        self._reset_client = self.create_client(Trigger, "/simulation/reset")
        self._isaac_parameters = AsyncParameterClient(
            self, "/isaac_navigation_sim"
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer, self, spin_thread=False
        )
        self._latest_odometry: MotionSample | None = None
        self._clock_s: float | None = None
        self._samples: list[MotionSample] = []
        self._recording = False
        self._collision_detected = False
        self._segment_index = -1
        self._segment_started_at = 0.0
        self._command_linear = 0.0
        self._command_angular = 0.0

    def _clock_callback(self, message: Clock) -> None:
        self._clock_s = (
            message.clock.sec + message.clock.nanosec * 1.0e-9
        )

    def _odom_callback(self, message: Odometry) -> None:
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0
            - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )
        now = time.monotonic()
        stamp_s = (
            message.header.stamp.sec
            + message.header.stamp.nanosec * 1.0e-9
        )
        previous = self._latest_odometry
        linear_speed = 0.0
        angular_speed = 0.0
        if previous is not None:
            dt = stamp_s - previous.stamp_s
            if 0.005 <= dt <= 0.25:
                yaw_delta = math.atan2(
                    math.sin(yaw - previous.yaw),
                    math.cos(yaw - previous.yaw),
                )
                midpoint_yaw = previous.yaw + 0.5 * yaw_delta
                delta_x = float(message.pose.pose.position.x) - previous.x
                delta_y = float(message.pose.pose.position.y) - previous.y
                linear_speed = (
                    delta_x * math.cos(midpoint_yaw)
                    + delta_y * math.sin(midpoint_yaw)
                ) / dt
                angular_speed = yaw_delta / dt
        sample = MotionSample(
            received_at=now,
            stamp_s=stamp_s,
            x=float(message.pose.pose.position.x),
            y=float(message.pose.pose.position.y),
            yaw=yaw,
            # Derive body-frame velocity from consecutive estimated poses so
            # the dispatcher never needs simulator ground truth.
            linear_speed=linear_speed,
            angular_speed=angular_speed,
            segment_index=self._segment_index,
            segment_elapsed=max(0.0, stamp_s - self._segment_started_at),
            command_linear=self._command_linear,
            command_angular=self._command_angular,
        )
        self._latest_odometry = sample
        if self._recording:
            self._samples.append(sample)

    def _collision_callback(self, message: Bool) -> None:
        if self._recording:
            self._collision_detected = (
                self._collision_detected or bool(message.data)
            )

    def _spin_once(self, timeout: float = 0.05) -> None:
        if not rclpy.ok():
            raise ExternalShutdownException()
        rclpy.spin_once(self, timeout_sec=timeout)

    def _wait_future(self, future, timeout: float) -> Any:
        deadline = time.monotonic() + timeout
        while not future.done():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise TimeoutError("ROS request timed out")
            self._spin_once(min(0.1, remaining))
        return future.result()

    def _publish(self, linear: float, angular: float) -> None:
        message = Twist()
        message.linear.x = float(linear)
        message.angular.z = float(angular)
        self._publisher.publish(message)

    def _reset(self, seed: int) -> None:
        self._publish(0.0, 0.0)
        if not self._isaac_parameters.wait_for_services(timeout_sec=10.0):
            raise RuntimeError("Isaac parameter services are unavailable")
        response = self._wait_future(
            self._isaac_parameters.set_parameters(
                [
                    Parameter("reset_seed", value=seed),
                    Parameter(
                        "reset_pose_name",
                        value=self._config.spawn_pose_name,
                    ),
                ]
            ),
            10.0,
        )
        if response is None or any(
            not result.successful for result in response.results
        ):
            raise RuntimeError("Isaac rejected motion benchmark reset parameters")
        if not self._reset_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("/simulation/reset is unavailable")
        barrier = time.monotonic()
        barrier_clock = self._clock_s
        reset_response = self._wait_future(
            self._reset_client.call_async(Trigger.Request()),
            30.0,
        )
        if reset_response is None or not reset_response.success:
            raise RuntimeError(
                "simulation reset failed: "
                + (
                    "no response"
                    if reset_response is None
                    else reset_response.message
                )
            )
        stable_since: float | None = None
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            self._spin_once(0.05)
            sample = self._latest_odometry
            if (
                sample is not None
                and sample.received_at > barrier
                and self._clock_s is not None
                and (barrier_clock is None or self._clock_s > barrier_clock)
                and abs(sample.linear_speed) <= 0.02
                and abs(sample.angular_speed) <= 0.05
                and self._estimated_tf_ready()
            ):
                if stable_since is None:
                    stable_since = time.monotonic()
                elif (
                    time.monotonic() - stable_since
                    >= self._config.reset_settle_sec
                ):
                    return
            else:
                stable_since = None
        raise TimeoutError(
            "estimated odometry/clock/odom->base_link TF did not settle after reset"
        )

    def _estimated_tf_ready(self) -> bool:
        try:
            self._tf_buffer.lookup_transform("odom", "base_link", Time())
        except TransformException:
            return False
        return True

    def _play_segment(self, index: int, segment: MotionSegment) -> None:
        period = 1.0 / self._config.command_rate_hz
        self._segment_index = index
        while self._clock_s is None:
            self._spin_once(0.05)
        self._segment_started_at = self._clock_s
        self._command_linear = segment.linear_x
        self._command_angular = segment.angular_z
        next_publish = time.monotonic()
        deadline = self._segment_started_at + segment.duration_sec
        while self._clock_s is None or self._clock_s < deadline:
            now = time.monotonic()
            if now >= next_publish:
                self._publish(segment.linear_x, segment.angular_z)
                next_publish += period
            self._spin_once(0.01)

    def _settle(self) -> None:
        period = 1.0 / self._config.command_rate_hz
        self._segment_index = -1
        while self._clock_s is None:
            self._spin_once(0.05)
        self._segment_started_at = self._clock_s
        self._command_linear = 0.0
        self._command_angular = 0.0
        next_publish = time.monotonic()
        deadline = self._segment_started_at + self._config.final_settle_sec
        while self._clock_s is None or self._clock_s < deadline:
            now = time.monotonic()
            if now >= next_publish:
                self._publish(0.0, 0.0)
                next_publish += period
            self._spin_once(0.01)

    def run(self) -> dict[str, Any]:
        results: list[dict[str, Any]] = []
        for index, primitive in enumerate(self._config.primitives):
            self.get_logger().info(
                f"running motion primitive {primitive.identifier}"
            )
            self._reset(self._config.reset_seed + index)
            self._samples = []
            self._collision_detected = False
            self._recording = True
            for segment_index, segment in enumerate(primitive.segments):
                self._play_segment(segment_index, segment)
            self._settle()
            self._recording = False
            result = evaluate_motion_primitive(
                primitive,
                self._samples,
                self._collision_detected,
                self._config.thresholds,
                self._config.steady_window_sec,
            )
            results.append(result)
            self.get_logger().info(
                f"completed {primitive.identifier}: "
                f"{'pass' if result['passed'] else 'failure'}"
            )
        self._publish(0.0, 0.0)
        return {
            "schema_version": 1,
            "recorded_at_utc": datetime.now(timezone.utc).isoformat(),
            "spawn_pose_name": self._config.spawn_pose_name,
            "command_rate_hz": self._config.command_rate_hz,
            "thresholds": self._config.thresholds.__dict__,
            "passed": all(result["passed"] for result in results),
            "primitive_count": len(results),
            "passed_primitive_count": sum(
                bool(result["passed"]) for result in results
            ),
            "primitives": results,
        }


def _write_json_atomic(path: Path, report: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        dir=path.parent,
        prefix=f".{path.name}.",
        suffix=".tmp",
        text=True,
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
        temporary.replace(path)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--output", required=True)
    arguments, ros_arguments = parser.parse_known_args()
    config = load_motion_config(arguments.config)
    rclpy.init(args=ros_arguments)
    node = MotionBenchmarkNode(config)
    try:
        report = node.run()
        output = Path(arguments.output).expanduser().resolve()
        _write_json_atomic(output, report)
        print(json.dumps(report, indent=2, sort_keys=True, allow_nan=False))
        raise SystemExit(0 if report["passed"] else 2)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
