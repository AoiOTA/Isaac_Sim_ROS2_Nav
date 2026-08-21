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
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from robot_experiments.reset_receipt import parse_reset_receipt


class MotionBenchmarkError(ValueError):
    """Raised when a motion benchmark configuration or run is invalid."""


class MotionSafetyStop(RuntimeError):
    """Raised after a fail-closed zero command ends a benchmark run."""


DEFAULT_STATE_FRESHNESS_SEC = 0.25
DEFAULT_STAMP_COHERENCE_SEC = 0.50
DEFAULT_SIM_CLOCK_STALL_TIMEOUT_SEC = 0.50
MOTION_DISPATCH_TIMEOUT_SEC = 15.0
COLLISION_MONITOR_QUERY_TIMEOUT_SEC = 2.0


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
class StationaryReference:
    identifier: str
    duration_sec: float
    reset_seed: int


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
    state_freshness_sec: float
    stamp_coherence_sec: float
    sim_clock_stall_timeout_sec: float
    thresholds: MotionThresholds
    primitives: tuple[MotionPrimitive, ...]
    stationary_reference: StationaryReference | None = None


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


@dataclass(frozen=True)
class ResetStopGateStatus:
    generation: int
    held: bool
    eligible_generation: int | None
    received_at: float


@dataclass(frozen=True)
class StampedObservation:
    """One ROS stream's latest arrival, stamp, and forward progress time."""

    received_at: float
    stamp_s: float
    progressed_at: float


def estimated_state_is_fresh(
    *,
    clock: StampedObservation | None,
    odometry: StampedObservation | None,
    transform: StampedObservation | None,
    reset_started_at: float,
    now: float,
    max_age_sec: float,
    stamp_coherence_sec: float,
) -> bool:
    """Require continuously advancing, mutually coherent estimated state."""

    observations = (clock, odometry, transform)
    if any(observation is None for observation in observations):
        return False
    assert clock is not None and odometry is not None and transform is not None
    for observation in observations:
        assert observation is not None
        if (
            observation.received_at <= reset_started_at
            or observation.progressed_at <= reset_started_at
            or now - observation.received_at > max_age_sec
            or now - observation.progressed_at > max_age_sec
            or not math.isfinite(observation.stamp_s)
            or observation.stamp_s < 0.0
        ):
            return False
    return (
        abs(odometry.stamp_s - clock.stamp_s) <= stamp_coherence_sec
        and abs(transform.stamp_s - clock.stamp_s) <= stamp_coherence_sec
    )


def validate_motion_dispatch(
    *,
    generation: int,
    reset_started_at: float,
    gate_status: ResetStopGateStatus | None,
    collision_monitor_active: bool,
    estimated_state_ready: bool,
) -> None:
    """Final instantaneous authority check immediately before nonzero output."""

    if gate_status is None or gate_status.received_at <= reset_started_at:
        raise MotionSafetyStop("reset_stop_gate_status_missing_at_dispatch")
    if gate_status.generation != generation:
        raise MotionSafetyStop("reset_stop_gate_generation_changed_at_dispatch")
    if gate_status.held:
        raise MotionSafetyStop("reset_stop_gate_held_at_dispatch")
    if not collision_monitor_active:
        raise MotionSafetyStop("collision_monitor_inactive_at_dispatch")
    if not estimated_state_ready:
        raise MotionSafetyStop("estimated_state_stale_at_dispatch")


def parse_reset_stop_gate_status(
    payload: str, *, received_at: float
) -> ResetStopGateStatus:
    """Parse the existing ResetStopGate authority status strictly."""

    try:
        document = json.loads(payload)
    except (TypeError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"invalid reset stop gate status: {exc}") from exc
    if not isinstance(document, dict):
        raise RuntimeError("invalid reset stop gate status: expected object")
    generation = document.get("generation")
    held = document.get("held")
    eligible = document.get("eligible_generation")
    if (
        isinstance(generation, bool)
        or not isinstance(generation, int)
        or generation < 0
        or not isinstance(held, bool)
        or (
            eligible is not None
            and (
                isinstance(eligible, bool)
                or not isinstance(eligible, int)
                or eligible != generation
            )
        )
        or (not held and eligible is not None)
    ):
        raise RuntimeError("invalid reset stop gate status fields")
    return ResetStopGateStatus(
        generation=generation,
        held=held,
        eligible_generation=eligible,
        received_at=float(received_at),
    )


@dataclass
class MotionDispatchBarrier:
    """Generation-fenced readiness required before any nonzero command."""

    generation: int
    reset_started_at: float
    settle_sec: float
    stable_since: float | None = None
    gate_seen: bool = False
    gate_released: bool = False
    collision_monitor_active: bool = False
    estimated_state_ready: bool = False

    def observe(
        self,
        *,
        gate_status: ResetStopGateStatus | None,
        collision_monitor_active: bool,
        estimated_state_ready: bool,
        now: float,
    ) -> bool:
        if (
            gate_status is not None
            and gate_status.received_at > self.reset_started_at
        ):
            self.gate_seen = True
            if gate_status.generation != self.generation:
                raise RuntimeError(
                    "reset stop gate generation mismatch: "
                    f"receipt={self.generation}, status={gate_status.generation}"
                )
            if self.gate_released and gate_status.held:
                raise RuntimeError(
                    "reset stop gate returned to HOLD during dispatch settle"
                )
            self.gate_released = not gate_status.held
        self.collision_monitor_active = bool(collision_monitor_active)
        self.estimated_state_ready = bool(estimated_state_ready)
        ready_now = (
            self.gate_seen
            and self.gate_released
            and self.collision_monitor_active
            and self.estimated_state_ready
        )
        if not ready_now:
            self.stable_since = None
            return False
        if self.stable_since is None:
            self.stable_since = now
            return self.settle_sec == 0.0
        return now - self.stable_since >= self.settle_sec

    def timeout_detail(self) -> str:
        return (
            "motion dispatch barrier timed out: "
            f"generation={self.generation}, gate_seen={self.gate_seen}, "
            f"gate_released={self.gate_released}, "
            f"collision_monitor_active={self.collision_monitor_active}, "
            f"estimated_state_ready={self.estimated_state_ready}"
        )


def wait_for_motion_dispatch_barrier(
    barrier: MotionDispatchBarrier,
    *,
    spin_once,
    snapshot,
    timeout_sec: float,
    monotonic=time.monotonic,
) -> None:
    deadline = monotonic() + timeout_sec
    while monotonic() < deadline:
        spin_once(0.05)
        gate_status, collision_active, estimated_ready = snapshot()
        if barrier.observe(
            gate_status=gate_status,
            collision_monitor_active=collision_active,
            estimated_state_ready=estimated_ready,
            now=monotonic(),
        ):
            return
    raise TimeoutError(barrier.timeout_detail())


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
        "state_freshness_sec",
        "stamp_coherence_sec",
        "sim_clock_stall_timeout_sec",
        "thresholds",
        "primitives",
        "stationary_reference",
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

    stationary_reference = None
    raw_stationary = document.get("stationary_reference")
    if raw_stationary is not None:
        if not isinstance(raw_stationary, dict) or set(raw_stationary) != {
            "id",
            "duration_sec",
            "reset_seed",
        }:
            raise MotionBenchmarkError(
                "stationary_reference requires exactly id, duration_sec, reset_seed"
            )
        identifier = raw_stationary["id"]
        reset_seed_value = raw_stationary["reset_seed"]
        if not isinstance(identifier, str) or not identifier:
            raise MotionBenchmarkError("stationary_reference.id must be non-empty")
        if (
            isinstance(reset_seed_value, bool)
            or not isinstance(reset_seed_value, int)
            or reset_seed_value < 0
        ):
            raise MotionBenchmarkError(
                "stationary_reference.reset_seed must be a non-negative integer"
            )
        stationary_reference = StationaryReference(
            identifier=identifier,
            duration_sec=_finite_number(
                raw_stationary["duration_sec"],
                "stationary_reference.duration_sec",
                positive=True,
            ),
            reset_seed=reset_seed_value,
        )
        if identifier in identifiers:
            raise MotionBenchmarkError(
                "stationary_reference.id must not duplicate a primitive id"
            )

    config = MotionConfig(
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
        state_freshness_sec=_finite_number(
            document.get(
                "state_freshness_sec", DEFAULT_STATE_FRESHNESS_SEC
            ),
            "state_freshness_sec",
            positive=True,
        ),
        stamp_coherence_sec=_finite_number(
            document.get(
                "stamp_coherence_sec", DEFAULT_STAMP_COHERENCE_SEC
            ),
            "stamp_coherence_sec",
            positive=True,
        ),
        sim_clock_stall_timeout_sec=_finite_number(
            document.get(
                "sim_clock_stall_timeout_sec",
                DEFAULT_SIM_CLOCK_STALL_TIMEOUT_SEC,
            ),
            "sim_clock_stall_timeout_sec",
            positive=True,
        ),
        thresholds=thresholds,
        primitives=tuple(primitives),
        stationary_reference=stationary_reference,
    )
    if config.state_freshness_sec >= config.reset_settle_sec:
        raise MotionBenchmarkError(
            "state_freshness_sec must be smaller than reset_settle_sec"
        )
    return config


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
            # Evidence-only contract field: the analyzer independently checks
            # this against the same installed diagnostic YAML and the phase
            # command-window duration. It does not change playback timing.
            "duration_sec": segment.duration_sec,
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
        # This calibration driver is upstream of velocity smoothing and
        # Collision Monitor; it never becomes a second final /cmd_vel owner.
        self._publisher = self.create_publisher(
            Twist, "/cmd_vel_nav", reliable
        )
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
        gate_status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._gate_status_subscription = self.create_subscription(
            String,
            "/simulation/reset_stop_gate/status",
            self._gate_status_callback,
            gate_status_qos,
        )
        self._reset_client = self.create_client(Trigger, "/simulation/reset")
        self._collision_state_client = self.create_client(
            GetState, "/collision_monitor/get_state"
        )
        self._isaac_parameters = AsyncParameterClient(
            self, "/isaac_navigation_sim"
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer, self, spin_thread=False
        )
        self._latest_odometry: MotionSample | None = None
        self._odom_progressed_at: float | None = None
        self._clock_s: float | None = None
        self._clock_observation: StampedObservation | None = None
        self._tf_observation: StampedObservation | None = None
        self._samples: list[MotionSample] = []
        self._recording = False
        self._collision_detected = False
        self._gate_status: ResetStopGateStatus | None = None
        self._gate_status_error: str | None = None
        self._collision_state_future = None
        self._collision_monitor_active = False
        self._collision_state_received_at: float | None = None
        self._segment_index = -1
        self._segment_started_at = 0.0
        self._command_linear = 0.0
        self._command_angular = 0.0
        self._reset_receipts: list[dict[str, Any]] = []
        self._current_reset_receipt: dict[str, Any] | None = None

    def _clock_callback(self, message: Clock) -> None:
        stamp_s = (
            message.clock.sec + message.clock.nanosec * 1.0e-9
        )
        now = time.monotonic()
        previous = self._clock_observation
        progressed_at = (
            now
            if previous is None or stamp_s > previous.stamp_s
            else previous.progressed_at
        )
        self._clock_s = stamp_s
        self._clock_observation = StampedObservation(
            received_at=now,
            stamp_s=stamp_s,
            progressed_at=progressed_at,
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
        if previous is None or stamp_s > previous.stamp_s:
            self._odom_progressed_at = now
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

    def _gate_status_callback(self, message: String) -> None:
        try:
            self._gate_status = parse_reset_stop_gate_status(
                message.data, received_at=time.monotonic()
            )
            self._gate_status_error = None
        except RuntimeError as exc:
            self._gate_status_error = str(exc)

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

    def _poll_collision_monitor_active(self) -> bool:
        now = time.monotonic()
        future = self._collision_state_future
        if future is not None and future.done():
            self._collision_state_future = None
            try:
                response = future.result()
            except Exception as exc:
                raise RuntimeError(
                    "CollisionMonitor lifecycle query failed: "
                    f"{type(exc).__name__}: {exc}"
                ) from exc
            if response is None:
                raise RuntimeError("CollisionMonitor lifecycle query returned no response")
            self._collision_monitor_active = (
                response.current_state.id == State.PRIMARY_STATE_ACTIVE
            )
            self._collision_state_received_at = now
        if (
            self._collision_state_future is None
            and self._collision_state_client.service_is_ready()
        ):
            self._collision_state_future = self._collision_state_client.call_async(
                GetState.Request()
            )
        return (
            self._collision_monitor_active
            and self._collision_state_received_at is not None
            and now - self._collision_state_received_at
            <= self._config.state_freshness_sec
        )

    def _query_collision_monitor_active(self) -> bool:
        pending = self._collision_state_future
        if pending is not None and not pending.done():
            cancel = getattr(pending, "cancel", None)
            if callable(cancel):
                cancel()
        self._collision_state_future = None
        if not self._collision_state_client.wait_for_service(
            timeout_sec=COLLISION_MONITOR_QUERY_TIMEOUT_SEC
        ):
            return False
        response = self._wait_future(
            self._collision_state_client.call_async(GetState.Request()),
            COLLISION_MONITOR_QUERY_TIMEOUT_SEC,
        )
        if response is None:
            return False
        self._collision_monitor_active = (
            response.current_state.id == State.PRIMARY_STATE_ACTIVE
        )
        self._collision_state_received_at = time.monotonic()
        return self._collision_monitor_active

    def _observe_estimated_tf(self) -> StampedObservation | None:
        try:
            transform = self._tf_buffer.lookup_transform(
                "odom", "base_link", Time()
            )
        except TransformException:
            return None
        now = time.monotonic()
        stamp_s = (
            transform.header.stamp.sec
            + transform.header.stamp.nanosec * 1.0e-9
        )
        previous = self._tf_observation
        progressed_at = (
            now
            if previous is None or stamp_s > previous.stamp_s
            else previous.progressed_at
        )
        self._tf_observation = StampedObservation(
            received_at=now,
            stamp_s=stamp_s,
            progressed_at=progressed_at,
        )
        return self._tf_observation

    def _estimated_state_ready(self, *, reset_started_at: float) -> bool:
        sample = self._latest_odometry
        odometry = None
        if sample is not None and self._odom_progressed_at is not None:
            odometry = StampedObservation(
                received_at=sample.received_at,
                stamp_s=sample.stamp_s,
                progressed_at=self._odom_progressed_at,
            )
        ready = estimated_state_is_fresh(
            clock=self._clock_observation,
            odometry=odometry,
            transform=self._observe_estimated_tf(),
            reset_started_at=reset_started_at,
            now=time.monotonic(),
            max_age_sec=self._config.state_freshness_sec,
            stamp_coherence_sec=self._config.stamp_coherence_sec,
        )
        return bool(
            ready
            and sample is not None
            and abs(sample.linear_speed) <= 0.02
            and abs(sample.angular_speed) <= 0.05
        )

    def _stop(self, reason: str) -> None:
        self._publish(0.0, 0.0)
        raise MotionSafetyStop(reason)

    def _reset(self, seed: int) -> dict[str, Any]:
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
        self._gate_status_error = None
        self._collision_monitor_active = False
        self._collision_state_received_at = None
        pending_state = self._collision_state_future
        if pending_state is not None and not pending_state.done():
            cancel = getattr(pending_state, "cancel", None)
            if callable(cancel):
                cancel()
        self._collision_state_future = None
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
        receipt = parse_reset_receipt(
            reset_response.message,
            requested_seed=seed,
        )
        self._current_reset_receipt = receipt
        self._reset_receipts.append(receipt)
        dispatch_barrier = MotionDispatchBarrier(
            generation=receipt["generation"],
            reset_started_at=barrier,
            settle_sec=self._config.reset_settle_sec,
        )

        def snapshot():
            if self._gate_status_error is not None:
                raise RuntimeError(self._gate_status_error)
            return (
                self._gate_status,
                self._poll_collision_monitor_active(),
                self._estimated_state_ready(reset_started_at=barrier),
            )

        try:
            wait_for_motion_dispatch_barrier(
                dispatch_barrier,
                spin_once=self._spin_once,
                snapshot=snapshot,
                timeout_sec=MOTION_DISPATCH_TIMEOUT_SEC,
            )
            # The settle result is not cached authority.  Query CollisionMonitor
            # again and resample gate/clock/odom/TF at the dispatch instant.
            collision_active = self._query_collision_monitor_active()
            validate_motion_dispatch(
                generation=receipt["generation"],
                reset_started_at=barrier,
                gate_status=self._gate_status,
                collision_monitor_active=collision_active,
                estimated_state_ready=self._estimated_state_ready(
                    reset_started_at=barrier
                ),
            )
        except (RuntimeError, TimeoutError) as exc:
            self._stop(f"motion_dispatch_stop:{exc}")
        return receipt

    def _assert_sim_clock_live(self) -> None:
        observation = self._clock_observation
        now = time.monotonic()
        if (
            observation is None
            or now - observation.received_at
            > self._config.sim_clock_stall_timeout_sec
            or now - observation.progressed_at
            > self._config.sim_clock_stall_timeout_sec
        ):
            self._stop("sim_clock_stalled_during_motion")

    def _play_segment(self, index: int, segment: MotionSegment) -> None:
        period = 1.0 / self._config.command_rate_hz
        self._segment_index = index
        waiting_since = time.monotonic()
        while self._clock_s is None:
            if (
                time.monotonic() - waiting_since
                > self._config.sim_clock_stall_timeout_sec
            ):
                self._stop("sim_clock_missing_during_motion")
            self._spin_once(0.05)
        self._assert_sim_clock_live()
        self._segment_started_at = self._clock_s
        self._command_linear = segment.linear_x
        self._command_angular = segment.angular_z
        next_publish = time.monotonic()
        deadline = self._segment_started_at + segment.duration_sec
        while self._clock_s is None or self._clock_s < deadline:
            self._assert_sim_clock_live()
            now = time.monotonic()
            if now >= next_publish:
                self._publish(segment.linear_x, segment.angular_z)
                next_publish += period
            self._spin_once(0.01)

    def _settle(self) -> None:
        period = 1.0 / self._config.command_rate_hz
        self._segment_index = -1
        waiting_since = time.monotonic()
        while self._clock_s is None:
            if (
                time.monotonic() - waiting_since
                > self._config.sim_clock_stall_timeout_sec
            ):
                self._stop("sim_clock_missing_during_settle")
            self._spin_once(0.05)
        self._assert_sim_clock_live()
        self._segment_started_at = self._clock_s
        self._command_linear = 0.0
        self._command_angular = 0.0
        next_publish = time.monotonic()
        deadline = self._segment_started_at + self._config.final_settle_sec
        while self._clock_s is None or self._clock_s < deadline:
            self._assert_sim_clock_live()
            now = time.monotonic()
            if now >= next_publish:
                self._publish(0.0, 0.0)
                next_publish += period
            self._spin_once(0.01)

    @staticmethod
    def _segment_contract(primitive: MotionPrimitive) -> list[dict[str, float]]:
        return [
            {
                "duration_sec": segment.duration_sec,
                "linear_x": segment.linear_x,
                "angular_z": segment.angular_z,
            }
            for segment in primitive.segments
        ]

    def _stationary_reference(self) -> dict[str, Any] | None:
        reference = self._config.stationary_reference
        if reference is None:
            return None
        self._samples = []
        self._collision_detected = False
        self._recording = False
        self._segment_index = -1
        self._segment_started_at = 0.0
        self._command_linear = 0.0
        self._command_angular = 0.0
        self._current_reset_receipt = None
        try:
            self._reset(reference.reset_seed)
            self._recording = True
            if self._clock_s is None:
                self._stop("stationary_reference_clock_missing")
            assert self._clock_s is not None
            start_s = self._clock_s
            self._segment_started_at = start_s
            deadline = start_s + reference.duration_sec
            period = 1.0 / self._config.command_rate_hz
            next_publish = time.monotonic()
            zero_command_count = 0
            while self._clock_s is None or self._clock_s < deadline:
                self._assert_sim_clock_live()
                now = time.monotonic()
                if now >= next_publish:
                    self._publish(0.0, 0.0)
                    zero_command_count += 1
                    next_publish += period
                self._spin_once(0.01)
            self._publish(0.0, 0.0)
            zero_command_count += 1
        except MotionSafetyStop as exc:
            self._recording = False
            self._publish(0.0, 0.0)
            stopped: dict[str, Any] = {
                "id": reference.identifier,
                "passed": False,
                "stopped": True,
                "outcome": "STOP",
                "failure_reasons": [str(exc)],
                "collision_detected": self._collision_detected,
                "sample_count": len(self._samples),
                "segments": [],
                "requested_duration_sec": reference.duration_sec,
                "measured_duration_sec": None,
                "zero_command_count": 1,
                "final_zero_published": True,
                "reset_seed": reference.reset_seed,
            }
            if self._current_reset_receipt is not None:
                stopped["reset_receipt"] = self._current_reset_receipt
            return stopped
        finally:
            self._recording = False

        measured_duration = (
            self._samples[-1].stamp_s - self._samples[0].stamp_s
            if len(self._samples) >= 2
            else 0.0
        )
        displacements = (
            [
                math.hypot(sample.x - self._samples[0].x, sample.y - self._samples[0].y)
                for sample in self._samples
            ]
            if self._samples else []
        )
        max_displacement = (
            max(displacements)
            if displacements and all(math.isfinite(value) for value in displacements)
            else None
        )
        failures: list[str] = []
        if not self._samples:
            failures.append("no_estimated_odometry_samples")
        if measured_duration + 0.05 < reference.duration_sec:
            failures.append("stationary_duration_short")
        if not math.isfinite(measured_duration):
            failures.append("stationary_duration_nonfinite")
        if self._collision_detected:
            failures.append("collision_detected")
        if displacements and max_displacement is None:
            failures.append("stationary_odometry_nonfinite")
        elif max_displacement is not None and max_displacement > 0.02:
            failures.append("stationary_odometry_displacement")
        return {
            "id": reference.identifier,
            "passed": not failures,
            "stopped": False,
            "outcome": "COMPLETED",
            "failure_reasons": failures,
            "collision_detected": self._collision_detected,
            "sample_count": len(self._samples),
            "segments": [],
            "requested_duration_sec": reference.duration_sec,
            "measured_duration_sec": measured_duration,
            "max_odometry_displacement_m": max_displacement,
            "zero_command_count": zero_command_count,
            "final_zero_published": True,
            "reset_seed": reference.reset_seed,
            "reset_receipt": self._current_reset_receipt,
        }

    def run(self) -> dict[str, Any]:
        results: list[dict[str, Any]] = []
        stationary = (
            MotionBenchmarkNode._stationary_reference(self)
            if getattr(self._config, "stationary_reference", None) is not None
            else None
        )
        stationary_blocked = bool(
            stationary is not None
            and (not stationary.get("passed", False) or stationary.get("stopped", False))
        )
        for index, primitive in enumerate(self._config.primitives):
            if stationary_blocked:
                break
            # Clear all result-bearing primitive state before logging, reset,
            # or any other operation that can fail.  A STOP must describe only
            # the primitive whose reset/dispatch/playback was attempted.
            self._samples = []
            self._collision_detected = False
            self._recording = False
            self._segment_index = -1
            self._segment_started_at = 0.0
            self._command_linear = 0.0
            self._command_angular = 0.0
            self._current_reset_receipt = None
            self.get_logger().info(
                f"running motion primitive {primitive.identifier}"
            )
            try:
                self._reset(self._config.reset_seed + index)
                self._recording = True
                for segment_index, segment in enumerate(primitive.segments):
                    self._play_segment(segment_index, segment)
                self._settle()
            except MotionSafetyStop as exc:
                self._recording = False
                self._publish(0.0, 0.0)
                stopped: dict[str, Any] = {
                    "id": primitive.identifier,
                    "passed": False,
                    "stopped": True,
                    "outcome": "STOP",
                    "failure_reasons": [str(exc)],
                    "collision_detected": self._collision_detected,
                    "sample_count": len(self._samples),
                    "segments": MotionBenchmarkNode._segment_contract(primitive),
                    "reset_seed": self._config.reset_seed + index,
                    "final_zero_published": True,
                }
                if self._current_reset_receipt is not None:
                    stopped["reset_receipt"] = self._current_reset_receipt
                results.append(stopped)
                self.get_logger().error(
                    f"stopped {primitive.identifier}: {exc}"
                )
                break
            finally:
                self._recording = False
            result = evaluate_motion_primitive(
                primitive,
                self._samples,
                self._collision_detected,
                self._config.thresholds,
                self._config.steady_window_sec,
            )
            result["reset_receipt"] = self._current_reset_receipt
            result["reset_seed"] = self._config.reset_seed + index
            result["stopped"] = False
            result["outcome"] = "COMPLETED"
            result["final_zero_published"] = True
            results.append(result)
            self.get_logger().info(
                f"completed {primitive.identifier}: "
                f"{'pass' if result['passed'] else 'failure'}"
            )
        self._publish(0.0, 0.0)
        collision_detected = bool(
            (stationary or {}).get("collision_detected", False)
            or any(result.get("collision_detected", False) for result in results)
        )
        stopped = bool(
            (stationary or {}).get("stopped", False)
            or any(result.get("stopped", False) for result in results)
        )
        complete = len(results) == len(self._config.primitives)
        passed = bool(
            complete
            and all(result["passed"] for result in results)
            and (stationary is None or stationary.get("passed", False))
        )
        return {
            "schema_version": 1,
            "recorded_at_utc": datetime.now(timezone.utc).isoformat(),
            "spawn_pose_name": self._config.spawn_pose_name,
            "command_rate_hz": self._config.command_rate_hz,
            "reset_settle_sec": self._config.reset_settle_sec,
            "state_freshness_sec": self._config.state_freshness_sec,
            "stamp_coherence_sec": self._config.stamp_coherence_sec,
            "sim_clock_stall_timeout_sec": (
                self._config.sim_clock_stall_timeout_sec
            ),
            "dispatch_barrier_timeout_sec": MOTION_DISPATCH_TIMEOUT_SEC,
            "collision_monitor_state_freshness_sec": (
                self._config.state_freshness_sec
            ),
            "collision_monitor_query_timeout_sec": (
                COLLISION_MONITOR_QUERY_TIMEOUT_SEC
            ),
            "collision_monitor_required_state": "active",
            "reset_stop_gate_generation_match_required": True,
            "thresholds": self._config.thresholds.__dict__,
            "passed": passed,
            "stopped": stopped,
            "collision_detected": collision_detected,
            "sample_count": sum(
                int(result.get("sample_count", 0)) for result in results
            ) + int((stationary or {}).get("sample_count", 0)),
            "segment_count": sum(len(result.get("segments", [])) for result in results),
            "final_zero_published": True,
            "primitive_count": len(results),
            "passed_primitive_count": sum(
                bool(result["passed"]) for result in results
            ),
            "reset_receipts": list(self._reset_receipts),
            "stationary_reference": stationary,
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
