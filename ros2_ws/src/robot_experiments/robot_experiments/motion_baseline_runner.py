"""ROS 2 adapter for deterministic four-wheel chassis motion diagnostics."""

from __future__ import annotations

import json
import math
import signal
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from .configuration import ConfigurationError
from .motion_baseline import (
    JointSample,
    MotionBaselineConfig,
    MotionSegment,
    NANOSECONDS_PER_SECOND,
    OdomSample,
    TimestampTracker,
    analyse_motion_segment,
    detect_stopping,
    load_motion_baseline_config,
)
from .report import (
    configuration_sha256,
    decode_hashed_contact_snapshot,
    validate_runtime_provenance,
    write_strict_json_report,
)


_RUNTIME_PROVENANCE_PARAMETER_NAMES = (
    "runtime_provenance.schema_version",
    "runtime_provenance.robot.config.path",
    "runtime_provenance.robot.config.sha256",
    "runtime_provenance.robot.asset.path",
    "runtime_provenance.robot.asset.sha256",
    "runtime_provenance.robot.solver.position_iterations",
    "runtime_provenance.robot.solver.velocity_iterations",
    "runtime_provenance.robot.solver."
    "stage_articulation_usd_readback_verified",
    "runtime_provenance.environment.id",
    "runtime_provenance.environment.project_stage.path",
    "runtime_provenance.environment.project_stage.sha256",
    "runtime_provenance.environment.source_asset.path",
    "runtime_provenance.environment.source_asset.sha256",
    "runtime_provenance.environment.asset_root",
    "runtime_provenance.environment.asset_version",
    "runtime_provenance.environment.composed_root_layer_sha256",
    "runtime_provenance.simulation.navigation_mode",
    "runtime_provenance.simulation.odometry_mode",
    "runtime_provenance.simulation.physics_hz",
    "runtime_provenance.contact.json",
    "runtime_provenance.contact.sha256",
    "runtime_provenance.git.commit",
    "runtime_provenance.git.branch",
    "runtime_provenance.git.dirty",
)


def _stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * NANOSECONDS_PER_SECOND + int(stamp.nanosec)


def _diagnostic_json_safe(value: Any) -> Any:
    """Keep derived overflow evidence without emitting invalid JSON numbers."""
    if isinstance(value, float) and not math.isfinite(value):
        if math.isnan(value):
            return "non_finite:nan"
        return "non_finite:+inf" if value > 0.0 else "non_finite:-inf"
    if isinstance(value, dict):
        return {
            str(key): _diagnostic_json_safe(item)
            for key, item in value.items()
        }
    if isinstance(value, (list, tuple)):
        return [_diagnostic_json_safe(item) for item in value]
    return value


def _yaw_from_quaternion(quaternion: Any) -> float:
    values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("odometry quaternion contains a non-finite value")
    norm = math.sqrt(sum(float(value) ** 2 for value in values))
    if norm <= 1.0e-12:
        raise ValueError("odometry quaternion has zero norm")
    x, y, z, w = (float(value) / norm for value in values)
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


class MotionBaselineRunner(Node):
    """Reset before each segment and own ``/cmd_vel`` for a bounded interval."""

    def __init__(self) -> None:
        super().__init__("motion_baseline_runner")
        config_file = str(self.declare_parameter("config_file", "").value).strip()
        output_file = str(self.declare_parameter("output_file", "").value).strip()
        self._environment_id = str(
            self.declare_parameter("environment_id", "").value
        ).strip()
        self._odometry_mode = str(
            self.declare_parameter("odometry_mode", "").value
        ).strip()
        if not config_file:
            raise ConfigurationError("config_file is required")
        if not output_file:
            raise ConfigurationError("output_file is required")
        if not self._environment_id:
            raise ConfigurationError("environment_id is required")
        if self._odometry_mode not in {"ideal", "realistic"}:
            raise ConfigurationError("odometry_mode must be ideal or realistic")
        self._config_path = Path(config_file).expanduser().resolve()
        self._output_path = Path(output_file).expanduser().resolve()
        self._config: MotionBaselineConfig = load_motion_baseline_config(
            self._config_path
        )
        self._config_hash = configuration_sha256(self._config_path)

        sensor_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        clock_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        command_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._command_qos = command_qos
        self._publisher = None
        self._clock_subscription = self.create_subscription(
            Clock,
            self._config.topics.clock,
            self._clock_callback,
            clock_qos,
        )
        self._odom_subscription = self.create_subscription(
            Odometry,
            self._config.topics.odom,
            self._odom_callback,
            sensor_qos,
        )
        self._joint_subscription = self.create_subscription(
            JointState,
            self._config.topics.joint_states,
            self._joint_callback,
            sensor_qos,
        )
        self._reset_client = self.create_client(
            Trigger, self._config.reset.service
        )
        self._isaac_parameter_client = AsyncParameterClient(
            self,
            str(
                self.declare_parameter(
                    "isaac_node_name", "/isaac_navigation_sim"
                ).value
            ),
        )

        self._clock_ns: int | None = None
        self._clock_sequence = 0
        self._odom_sequence = 0
        self._joint_sequence = 0
        self._latest_odom: OdomSample | None = None
        self._latest_joint: JointSample | None = None
        self._last_clock_received_wall: float | None = None
        self._last_odom_received_wall: float | None = None
        self._last_joint_received_wall: float | None = None
        self._session_timestamps = {
            "clock": TimestampTracker(),
            "odom": TimestampTracker(),
            "joint_states": TimestampTracker(),
        }
        self._active_timestamps: dict[str, TimestampTracker] | None = None
        self._active_odom: list[OdomSample] | None = None
        self._active_joints: list[JointSample] | None = None
        self._active_invalid = {"odom": 0, "joint_states": 0}
        self._segments: list[dict[str, object]] = []
        self._started_at = datetime.now(timezone.utc)
        self._safe_stop_attempted = False
        self._authorized_reset_publishers: set[str] = set()
        self._runtime_provenance: dict[str, object] = {"verified": False}

    def _observe_timestamp(self, topic: str, stamp_ns: int) -> None:
        self._session_timestamps[topic].observe(stamp_ns)
        if self._active_timestamps is not None:
            self._active_timestamps[topic].observe(stamp_ns)

    def _clock_callback(self, message: Clock) -> None:
        stamp = _stamp_ns(message.clock)
        self._observe_timestamp("clock", stamp)
        self._clock_ns = stamp
        self._clock_sequence += 1
        self._last_clock_received_wall = time.monotonic()

    def _odom_callback(self, message: Odometry) -> None:
        stamp = _stamp_ns(message.header.stamp)
        self._observe_timestamp("odom", stamp)
        try:
            sample = OdomSample(
                stamp_ns=stamp,
                x_m=float(message.pose.pose.position.x),
                y_m=float(message.pose.pose.position.y),
                yaw_rad=_yaw_from_quaternion(message.pose.pose.orientation),
                linear_x_mps=float(message.twist.twist.linear.x),
                linear_y_mps=float(message.twist.twist.linear.y),
                angular_z_radps=float(message.twist.twist.angular.z),
            )
        except (TypeError, ValueError):
            if self._active_odom is not None:
                self._active_invalid["odom"] += 1
            return
        self._latest_odom = sample
        self._odom_sequence += 1
        self._last_odom_received_wall = time.monotonic()
        if self._active_odom is not None:
            self._active_odom.append(sample)

    def _joint_callback(self, message: JointState) -> None:
        stamp = _stamp_ns(message.header.stamp)
        self._observe_timestamp("joint_states", stamp)
        try:
            if len(message.name) != len(message.velocity):
                raise ValueError("joint name and velocity lengths differ")
            if len(set(message.name)) != len(message.name):
                raise ValueError("joint names are duplicated")
            velocity_map = {
                str(name): float(velocity)
                for name, velocity in zip(message.name, message.velocity)
            }
            missing = set(self._config.wheels.ordered_names) - set(velocity_map)
            if missing:
                raise ValueError("required wheel joint is missing")
            sample = JointSample.from_mapping(
                stamp,
                {
                    name: velocity_map[name]
                    for name in self._config.wheels.ordered_names
                },
            )
        except (TypeError, ValueError):
            if self._active_joints is not None:
                self._active_invalid["joint_states"] += 1
            return
        self._latest_joint = sample
        self._joint_sequence += 1
        self._last_joint_received_wall = time.monotonic()
        if self._active_joints is not None:
            self._active_joints.append(sample)

    def _raise_if_shutdown(self) -> None:
        if not rclpy.ok(context=self.context):
            raise ExternalShutdownException()

    def _spin_once(self, timeout_sec: float) -> None:
        self._raise_if_shutdown()
        rclpy.spin_once(self, timeout_sec=max(0.0, timeout_sec))

    def _wait_future(self, future: Any, deadline: float) -> bool:
        while time.monotonic() < deadline:
            self._spin_once(min(0.05, deadline - time.monotonic()))
            if future.done():
                return True
        return future.done()

    def _wait_until(self, predicate: Any, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            self._spin_once(min(0.05, deadline - time.monotonic()))
            if predicate():
                return True
        return bool(predicate())

    def _read_runtime_provenance(self) -> None:
        timeout_sec = self._config.reset.service_timeout_sec
        if not self._isaac_parameter_client.wait_for_services(
            timeout_sec=timeout_sec
        ):
            self._raise_if_shutdown()
            raise RuntimeError("Isaac runtime provenance services are unavailable")
        future = self._isaac_parameter_client.get_parameters(
            list(_RUNTIME_PROVENANCE_PARAMETER_NAMES)
        )
        if not self._wait_future(
            future, time.monotonic() + timeout_sec
        ):
            raise TimeoutError("reading Isaac runtime provenance timed out")
        response = future.result()
        if response is None or len(response.values) != len(
            _RUNTIME_PROVENANCE_PARAMETER_NAMES
        ):
            raise RuntimeError("Isaac returned incomplete runtime provenance")
        values = {
            name: parameter_value_to_python(value)
            for name, value in zip(
                _RUNTIME_PROVENANCE_PARAMETER_NAMES, response.values
            )
        }

        def value(suffix: str) -> object:
            return values[f"runtime_provenance.{suffix}"]

        provenance = {
            "verified": True,
            "schema_version": value("schema_version"),
            "robot": {
                "config": {
                    "path": value("robot.config.path"),
                    "sha256": value("robot.config.sha256"),
                },
                "asset": {
                    "path": value("robot.asset.path"),
                    "sha256": value("robot.asset.sha256"),
                },
                "solver": {
                    "position_iterations": value(
                        "robot.solver.position_iterations"
                    ),
                    "velocity_iterations": value(
                        "robot.solver.velocity_iterations"
                    ),
                    "stage_articulation_usd_readback_verified": value(
                        "robot.solver."
                        "stage_articulation_usd_readback_verified"
                    ),
                },
            },
            "environment": {
                "id": value("environment.id"),
                "project_stage": {
                    "path": value("environment.project_stage.path"),
                    "sha256": value("environment.project_stage.sha256"),
                },
                "source_asset": {
                    "path": value("environment.source_asset.path"),
                    "sha256": value("environment.source_asset.sha256"),
                },
                "asset_root": value("environment.asset_root"),
                "asset_version": value("environment.asset_version"),
                "composed_root_layer_sha256": value(
                    "environment.composed_root_layer_sha256"
                ),
            },
            "simulation": {
                "navigation_mode": value("simulation.navigation_mode"),
                "odometry_mode": value("simulation.odometry_mode"),
                "physics_hz": value("simulation.physics_hz"),
            },
            "contact": decode_hashed_contact_snapshot(
                value("contact.json"),
                value("contact.sha256"),
            ),
            "git": {
                "commit": value("git.commit"),
                "branch": value("git.branch"),
                "dirty": value("git.dirty"),
            },
        }
        validate_runtime_provenance(provenance)
        # Preserve a structurally verified Isaac snapshot in failure reports
        # even when a caller-supplied grouping label does not match it.
        self._runtime_provenance = provenance
        runtime_odometry = provenance["simulation"]["odometry_mode"]
        if runtime_odometry != self._odometry_mode:
            raise RuntimeError(
                "odometry label does not match Isaac runtime provenance: "
                f"requested={self._odometry_mode}, runtime={runtime_odometry}"
            )
        runtime_environment = provenance["environment"]["id"]
        if runtime_environment != self._environment_id:
            raise RuntimeError(
                "environment label does not match Isaac runtime provenance: "
                f"requested={self._environment_id}, runtime={runtime_environment}"
            )

    def _begin_segment_capture(self) -> None:
        self._active_timestamps = {
            "clock": TimestampTracker(),
            "odom": TimestampTracker(),
            "joint_states": TimestampTracker(),
        }
        self._active_odom = []
        self._active_joints = []
        self._active_invalid = {"odom": 0, "joint_states": 0}

    def _timestamp_report(self) -> dict[str, dict[str, object]]:
        if self._active_timestamps is None:
            return {}
        return {
            name: tracker.as_dict()
            for name, tracker in self._active_timestamps.items()
        }

    @staticmethod
    def _node_fqn(node_name: str, node_namespace: str) -> str:
        return f"{node_namespace.rstrip('/')}/{node_name}".replace("//", "/")

    def _owns_reset_service(self, endpoint: Any) -> bool:
        try:
            services = self.get_service_names_and_types_by_node(
                endpoint.node_name, endpoint.node_namespace
            )
        except RuntimeError:
            return False
        return any(
            service_name == self._config.reset.service
            and "std_srvs/srv/Trigger" in service_types
            for service_name, service_types in services
        )

    def _foreign_command_publishers(self) -> list[str]:
        publishers = self.get_publishers_info_by_topic(self._config.topics.cmd_vel)
        own_name = self.get_name()
        own_namespace = self.get_namespace()
        foreign: set[str] = set()
        reset_publishers: set[str] = set()
        for endpoint in publishers:
            if (
                endpoint.node_name == own_name
                and endpoint.node_namespace == own_namespace
            ):
                continue
            identity = self._node_fqn(
                endpoint.node_name, endpoint.node_namespace
            )
            if self._owns_reset_service(endpoint):
                reset_publishers.add(identity)
            else:
                foreign.add(identity)

        # Reset deliberately publishes a zero Twist before restoring the robot.
        # Authenticate that safety endpoint by service ownership instead of a
        # hard-coded node name. Multiple Reset owners are ambiguous and unsafe.
        if len(reset_publishers) == 1:
            self._authorized_reset_publishers = reset_publishers
        else:
            self._authorized_reset_publishers = set()
            foreign.update(reset_publishers)
        return sorted(foreign)

    def _assert_command_channel_uncontended(self) -> None:
        foreign = self._foreign_command_publishers()
        if foreign:
            raise RuntimeError(
                f"{self._config.topics.cmd_vel} already has foreign publishers: "
                + ", ".join(foreign)
            )

    def _create_command_publisher(self) -> None:
        # Give DDS graph discovery a bounded opportunity to reveal an existing owner.
        discovery_deadline = time.monotonic() + 0.5
        while time.monotonic() < discovery_deadline:
            self._spin_once(min(0.05, discovery_deadline - time.monotonic()))
        self._assert_command_channel_uncontended()
        self._publisher = self.create_publisher(
            Twist, self._config.topics.cmd_vel, self._command_qos
        )
        if not self._wait_until(
            lambda: self._publisher is not None
            and self._publisher.get_subscription_count() > 0,
            self._config.reset.service_timeout_sec,
        ):
            raise RuntimeError(
                f"{self._config.topics.cmd_vel} has no command subscriber"
            )

    def _publish(self, linear_x: float, angular_z: float) -> None:
        if self._publisher is None:
            raise RuntimeError("command publisher has not been created")
        message = Twist()
        message.linear.x = float(linear_x)
        message.angular.z = float(angular_z)
        self._publisher.publish(message)

    def safe_stop(self) -> None:
        """Publish a bounded zero burst; safe to call repeatedly from ``finally``."""
        self._safe_stop_attempted = True
        if self._publisher is None:
            return
        for index in range(self._config.sampling.zero_publish_count):
            try:
                self._publish(0.0, 0.0)
                if (
                    index + 1 < self._config.sampling.zero_publish_count
                    and self._config.sampling.zero_publish_interval_sec > 0.0
                    and rclpy.ok(context=self.context)
                ):
                    rclpy.spin_once(
                        self,
                        timeout_sec=self._config.sampling.zero_publish_interval_sec,
                    )
            except Exception:
                # Shutdown can invalidate the context between the check and publish.
                break

    def _streams_fresh(self) -> bool:
        if (
            self._clock_ns is None
            or self._latest_odom is None
            or self._latest_joint is None
        ):
            return False
        max_age_ns = round(
            self._config.sampling.max_sample_age_sec * NANOSECONDS_PER_SECOND
        )
        odom_age = self._clock_ns - self._latest_odom.stamp_ns
        joint_age = self._clock_ns - self._latest_joint.stamp_ns
        return 0 <= odom_age <= max_age_ns and 0 <= joint_age <= max_age_ns

    def _wall_streams_fresh(self) -> bool:
        now = time.monotonic()
        stamps = (
            self._last_clock_received_wall,
            self._last_odom_received_wall,
            self._last_joint_received_wall,
        )
        return all(
            stamp is not None
            and now - stamp <= self._config.sampling.max_sample_age_sec
            for stamp in stamps
        )

    def _stationary_gate_status(self) -> dict[str, bool]:
        stop = self._config.stop
        gates = {"streams_fresh": self._streams_fresh()}
        odom = self._latest_odom
        joint = self._latest_joint
        gates["odom_linear_speed"] = (
            odom is not None
            and math.hypot(odom.linear_x_mps, odom.linear_y_mps)
            <= stop.linear_velocity_threshold_mps
        )
        gates["odom_angular_speed"] = (
            odom is not None
            and abs(odom.angular_z_radps)
            <= stop.angular_velocity_threshold_radps
        )
        velocities = {} if joint is None else joint.velocity_map()
        for name in self._config.wheels.ordered_names:
            gates[f"wheel:{name}"] = (
                name in velocities
                and abs(velocities[name]) <= stop.wheel_velocity_threshold_radps
            )
        return gates

    def _latest_stationary(self) -> bool:
        return all(self._stationary_gate_status().values())

    def _reset_recovery_diagnostic(
        self,
        barriers: tuple[int, int, int],
        observation_counts: dict[str, int],
        violation_counts: dict[str, int],
        peak_observed: dict[str, object],
        longest_stationary_duration_ns: int,
    ) -> dict[str, object]:
        """Describe the exact freshness or motion gate blocking Reset recovery."""
        now = time.monotonic()
        clock_ns = self._clock_ns
        odom = self._latest_odom
        joint = self._latest_joint
        wall_received = {
            "clock": self._last_clock_received_wall,
            "odom": self._last_odom_received_wall,
            "joint_states": self._last_joint_received_wall,
        }
        diagnostic: dict[str, object] = {
            "sequence_barriers": {
                "clock": barriers[0],
                "odom": barriers[1],
                "joint_states": barriers[2],
            },
            "sequence_current": {
                "clock": self._clock_sequence,
                "odom": self._odom_sequence,
                "joint_states": self._joint_sequence,
            },
            "fresh_sequences": {
                "clock": self._clock_sequence > barriers[0],
                "odom": self._odom_sequence > barriers[1],
                "joint_states": self._joint_sequence > barriers[2],
            },
            "wall_age_sec": {
                topic: None if received is None else max(0.0, now - received)
                for topic, received in wall_received.items()
            },
            "clock_ns": clock_ns,
            "sim_age_sec": {
                "odom": (
                    None
                    if clock_ns is None or odom is None
                    else (clock_ns - odom.stamp_ns) / NANOSECONDS_PER_SECOND
                ),
                "joint_states": (
                    None
                    if clock_ns is None or joint is None
                    else (clock_ns - joint.stamp_ns) / NANOSECONDS_PER_SECOND
                ),
            },
            "streams_fresh": self._streams_fresh(),
            "wall_streams_fresh": self._wall_streams_fresh(),
            "observation_counts": dict(observation_counts),
            "violation_counts": dict(violation_counts),
            "peak_observed": peak_observed,
            "longest_stationary_duration_sec": (
                longest_stationary_duration_ns / NANOSECONDS_PER_SECOND
            ),
            "thresholds": {
                "linear_speed_mps": self._config.stop.linear_velocity_threshold_mps,
                "angular_speed_radps": (
                    self._config.stop.angular_velocity_threshold_radps
                ),
                "wheel_speed_radps": (
                    self._config.stop.wheel_velocity_threshold_radps
                ),
            },
        }
        if odom is not None:
            diagnostic["odom"] = {
                "stamp_ns": odom.stamp_ns,
                "linear_speed_mps": math.hypot(
                    odom.linear_x_mps, odom.linear_y_mps
                ),
                "angular_speed_radps": abs(odom.angular_z_radps),
            }
        if joint is not None:
            velocities = joint.velocity_map()
            diagnostic["joint_states"] = {
                "stamp_ns": joint.stamp_ns,
                "wheel_abs_speed_radps": {
                    name: abs(velocities[name])
                    for name in self._config.wheels.ordered_names
                },
            }
        terminal_gates = self._stationary_gate_status()
        diagnostic["terminal_gates"] = terminal_gates
        diagnostic["terminal_blockers"] = sorted(
            gate for gate, passed in terminal_gates.items() if not passed
        )
        diagnostic["stationary_now"] = all(terminal_gates.values())
        safe_diagnostic = _diagnostic_json_safe(diagnostic)
        assert isinstance(safe_diagnostic, dict)
        return safe_diagnostic

    def _reset_and_wait(self) -> dict[str, object]:
        self._assert_command_channel_uncontended()
        self.safe_stop()
        self._assert_command_channel_uncontended()
        before_clock = self._clock_ns
        barriers = (
            self._clock_sequence,
            self._odom_sequence,
            self._joint_sequence,
        )
        started = time.monotonic()
        reset = self._config.reset
        if not self._reset_client.wait_for_service(
            timeout_sec=reset.service_timeout_sec
        ):
            self._raise_if_shutdown()
            raise RuntimeError(f"reset service unavailable: {reset.service}")
        future = self._reset_client.call_async(Trigger.Request())
        if not self._wait_future(
            future, time.monotonic() + reset.service_timeout_sec
        ):
            raise TimeoutError("simulation Reset response timed out")
        response = future.result()
        response_wall = time.monotonic()
        if response is None or not response.success:
            message = "no response" if response is None else response.message
            raise RuntimeError(f"simulation Reset failed: {message}")

        recovery_deadline = time.monotonic() + reset.recovery_timeout_sec
        stationary_since_ns: int | None = None
        longest_stationary_duration_ns = 0
        observation_counts = {
            "sequence_not_fresh": 0,
            "not_stationary": 0,
            "stationary": 0,
        }
        gate_names = (
            "streams_fresh",
            "odom_linear_speed",
            "odom_angular_speed",
            *(f"wheel:{name}" for name in self._config.wheels.ordered_names),
        )
        violation_counts = {name: 0 for name in gate_names}
        peak_observed: dict[str, object] = {
            "odom_linear_speed_mps": 0.0,
            "odom_angular_speed_radps": 0.0,
            "wheel_abs_speed_radps": {
                name: 0.0 for name in self._config.wheels.ordered_names
            },
        }
        publish_period = 1.0 / self._config.sampling.publish_rate_hz
        next_publish = 0.0
        while time.monotonic() < recovery_deadline:
            now = time.monotonic()
            if now >= next_publish:
                self._publish(0.0, 0.0)
                next_publish = now + publish_period
            self._spin_once(min(0.05, recovery_deadline - now))
            fresh_sequences = (
                self._clock_sequence > barriers[0]
                and self._odom_sequence > barriers[1]
                and self._joint_sequence > barriers[2]
            )
            if not fresh_sequences:
                observation_counts["sequence_not_fresh"] += 1
                stationary_since_ns = None
                continue
            if self._latest_odom is not None:
                peak_observed["odom_linear_speed_mps"] = max(
                    float(peak_observed["odom_linear_speed_mps"]),
                    math.hypot(
                        self._latest_odom.linear_x_mps,
                        self._latest_odom.linear_y_mps,
                    ),
                )
                peak_observed["odom_angular_speed_radps"] = max(
                    float(peak_observed["odom_angular_speed_radps"]),
                    abs(self._latest_odom.angular_z_radps),
                )
            if self._latest_joint is not None:
                velocities = self._latest_joint.velocity_map()
                wheel_peaks = peak_observed["wheel_abs_speed_radps"]
                assert isinstance(wheel_peaks, dict)
                for name in self._config.wheels.ordered_names:
                    wheel_peaks[name] = max(
                        float(wheel_peaks[name]),
                        abs(velocities[name]),
                    )
            gate_status = self._stationary_gate_status()
            if not all(gate_status.values()):
                observation_counts["not_stationary"] += 1
                for gate, passed in gate_status.items():
                    if not passed:
                        violation_counts[gate] += 1
                if stationary_since_ns is not None and self._clock_ns is not None:
                    longest_stationary_duration_ns = max(
                        longest_stationary_duration_ns,
                        self._clock_ns - stationary_since_ns,
                    )
                stationary_since_ns = None
                continue
            observation_counts["stationary"] += 1
            assert self._clock_ns is not None
            if stationary_since_ns is None:
                stationary_since_ns = self._clock_ns
            if self._clock_ns < stationary_since_ns:
                stationary_since_ns = self._clock_ns
            settled_ns = self._clock_ns - stationary_since_ns
            if settled_ns >= round(
                reset.settle_duration_sec * NANOSECONDS_PER_SECOND
            ):
                after_clock = self._clock_ns
                return {
                    "service": reset.service,
                    "response_message": str(response.message),
                    "service_latency_wall_sec": response_wall - started,
                    "recovery_latency_wall_sec": time.monotonic() - response_wall,
                    "clock_before_ns": before_clock,
                    "clock_after_ns": after_clock,
                    "clock_rollback_observed": (
                        before_clock is not None and after_clock < before_clock
                    ),
                    "fresh_clock_received": self._clock_sequence > barriers[0],
                    "fresh_odom_received": self._odom_sequence > barriers[1],
                    "fresh_joint_states_received": self._joint_sequence > barriers[2],
                    "stationary_settle_duration_sec": reset.settle_duration_sec,
                }
        if stationary_since_ns is not None and self._clock_ns is not None:
            longest_stationary_duration_ns = max(
                longest_stationary_duration_ns,
                self._clock_ns - stationary_since_ns,
            )
        diagnostic = self._reset_recovery_diagnostic(
            barriers,
            observation_counts,
            violation_counts,
            peak_observed,
            longest_stationary_duration_ns,
        )
        raise TimeoutError(
            "Reset recovery timed out waiting for fresh /clock, /odom, "
            "/joint_states and a stationary chassis; diagnostic="
            + json.dumps(
                diagnostic,
                sort_keys=True,
                separators=(",", ":"),
                allow_nan=False,
            )
        )

    def _execute_command(self, segment: MotionSegment) -> tuple[int, int, int]:
        self._assert_command_channel_uncontended()
        if self._publisher is None or self._publisher.get_subscription_count() <= 0:
            raise RuntimeError(
                f"{self._config.topics.cmd_vel} command subscriber disappeared"
            )
        if self._clock_ns is None or not self._wall_streams_fresh():
            raise RuntimeError("motion baseline input streams are not fresh")
        command_start_ns = self._clock_ns
        duration_ns = round(segment.duration_sec * NANOSECONDS_PER_SECOND)
        deadline = time.monotonic() + self._config.sampling.command_wall_timeout_sec
        publish_period = 1.0 / self._config.sampling.publish_rate_hz
        next_publish = 0.0
        publish_count = 0
        while time.monotonic() < deadline:
            self._raise_if_shutdown()
            if not self._wall_streams_fresh():
                raise RuntimeError("/clock, /odom, or /joint_states became stale")
            assert self._clock_ns is not None
            if self._clock_ns < command_start_ns:
                raise RuntimeError("simulation clock regressed during a command segment")
            if self._clock_ns - command_start_ns >= duration_ns:
                self._publish(0.0, 0.0)
                return command_start_ns, self._clock_ns, publish_count
            now = time.monotonic()
            if now >= next_publish:
                self._publish(segment.linear_x_mps, segment.angular_z_radps)
                publish_count += 1
                next_publish = now + publish_period
            self._spin_once(min(0.02, deadline - now))
        self._publish(0.0, 0.0)
        raise TimeoutError(
            f"command {segment.segment_id} did not reach its simulated duration "
            "before the wall-clock timeout"
        )

    def _wait_for_stop(self, command_end_ns: int) -> None:
        if self._active_odom is None or self._active_joints is None:
            raise RuntimeError("segment capture is not active")
        deadline = time.monotonic() + self._config.stop.timeout_sec
        publish_period = 1.0 / self._config.sampling.publish_rate_hz
        next_publish = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_publish:
                self._publish(0.0, 0.0)
                next_publish = now + publish_period
            self._spin_once(min(0.02, deadline - now))
            detected = detect_stopping(
                command_end_ns,
                self._active_odom,
                self._active_joints,
                self._config.wheels,
                self._config.stop,
            )
            if detected.stopped:
                return

    def _run_segment(self, segment: MotionSegment) -> dict[str, object]:
        # Reset is an intentional epoch boundary.  Keep its clock rollback in
        # the reset report, then start integrity/drift sampling in the fresh
        # epoch so a valid Reset is not misreported as a sensor regression.
        self._active_timestamps = None
        self._active_odom = None
        self._active_joints = None
        self._active_invalid = {"odom": 0, "joint_states": 0}
        reset_report = self._reset_and_wait()
        self._begin_segment_capture()
        command_start_ns, command_end_ns, publish_count = self._execute_command(
            segment
        )
        self._wait_for_stop(command_end_ns)
        self.safe_stop()
        assert self._active_odom is not None
        assert self._active_joints is not None
        result = analyse_motion_segment(
            segment,
            command_start_ns,
            command_end_ns,
            self._active_odom,
            self._active_joints,
            self._config.wheels,
            self._config.stop,
            command_publish_count=publish_count,
            timestamp_integrity=self._timestamp_report(),
        )
        result["reset"] = reset_report
        result["invalid_message_counts"] = dict(self._active_invalid)
        return result

    def _base_report(self) -> dict[str, object]:
        return {
            "schema_version": 1,
            "diagnostic": "four_wheel_chassis_motion_baseline",
            "profile_id": self._config.profile_id,
            "environment_id": self._environment_id,
            "odometry_mode": self._odometry_mode,
            "config_file": str(self._config_path),
            "config_sha256": self._config_hash,
            "output_file": str(self._output_path),
            "started_at_utc": self._started_at.isoformat(),
            "completed_at_utc": datetime.now(timezone.utc).isoformat(),
            "configuration": self._config.as_dict(),
            "runtime_provenance": self._runtime_provenance,
            "segments": list(self._segments),
            "timestamp_integrity": {
                name: tracker.as_dict()
                for name, tracker in self._session_timestamps.items()
            },
            "safety": {
                "exclusive_non_reset_cmd_vel_owner_enforced": True,
                "authorized_reset_safety_publishers": sorted(
                    self._authorized_reset_publishers
                ),
                "cmd_vel_subscription_count": (
                    0
                    if self._publisher is None
                    else self._publisher.get_subscription_count()
                ),
                "safe_zero_burst_attempted": self._safe_stop_attempted,
                "zero_publish_count": self._config.sampling.zero_publish_count,
            },
        }

    def run_all(self) -> dict[str, object]:
        fatal_error: Exception | None = None
        try:
            self._read_runtime_provenance()
            self._create_command_publisher()
            if not self._wait_until(
                lambda: self._clock_ns is not None
                and self._clock_ns > 0
                and self._latest_odom is not None
                and self._latest_joint is not None,
                self._config.reset.recovery_timeout_sec,
            ):
                raise TimeoutError(
                    "timed out waiting for non-zero /clock, /odom, and /joint_states"
                )
            for segment in self._config.segments:
                self.get_logger().info(
                    f"starting motion segment {segment.segment_id}"
                )
                try:
                    result = self._run_segment(segment)
                    self._segments.append(result)
                    self.get_logger().info(
                        f"completed {segment.segment_id}: {result['result']}"
                    )
                except Exception as exc:
                    fatal_error = exc
                    self._segments.append(
                        {
                            "segment_id": segment.segment_id,
                            "motion": segment.motion,
                            "tier": segment.tier,
                            "result": "error",
                            "failure_reason": f"{type(exc).__name__}:{exc}",
                            "invalid_message_counts": dict(self._active_invalid),
                            "timestamp_integrity": self._timestamp_report(),
                        }
                    )
                    break
        except Exception as exc:
            fatal_error = exc
        finally:
            self.safe_stop()

        report = self._base_report()
        incomplete = len(self._segments) != len(self._config.segments)
        segment_failures = [
            str(segment["segment_id"])
            for segment in self._segments
            if segment.get("result") != "complete"
        ]
        report["result"] = (
            "success"
            if fatal_error is None and not incomplete and not segment_failures
            else "failure"
        )
        report["failure_reason"] = (
            ""
            if fatal_error is None
            else f"{type(fatal_error).__name__}:{fatal_error}"
        )
        report["failed_segments"] = segment_failures
        write_strict_json_report(report, self._output_path)
        self.get_logger().info(f"motion baseline report: {self._output_path}")
        if fatal_error is not None:
            raise fatal_error
        return report


def _raise_keyboard_interrupt(signum, frame) -> None:
    del signum, frame
    raise KeyboardInterrupt


def main(args=None) -> None:
    rclpy.init(args=args)
    node: MotionBaselineRunner | None = None
    previous_handlers = {}
    try:
        node = MotionBaselineRunner()
        # Override rclpy's context-shutdown handlers while commands may be
        # non-zero.  Keeping the context alive lets ``finally`` publish the
        # explicit bounded zero burst on INT, TERM, or terminal hangup.
        for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            previous_handlers[signum] = signal.signal(
                signum, _raise_keyboard_interrupt
            )
        node.run_all()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.safe_stop()
            node.destroy_node()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
        if rclpy.ok():
            rclpy.shutdown()
