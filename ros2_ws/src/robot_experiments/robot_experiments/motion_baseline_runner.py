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
    "runtime_provenance.robot.kinematics.profile_id",
    "runtime_provenance.robot.kinematics.lifecycle",
    "runtime_provenance.robot.kinematics.wheel_radius_m",
    "runtime_provenance.robot.kinematics.wheel_width_m",
    "runtime_provenance.robot.kinematics.geometric_track_width_m",
    "runtime_provenance.robot.kinematics.effective_track_width_m",
    "runtime_provenance.robot.kinematics.controller_contract_verified",
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


def _json_object_without_duplicate_keys(
    pairs: list[tuple[str, Any]],
) -> dict[str, Any]:
    value: dict[str, Any] = {}
    for key, item in pairs:
        if key in value:
            raise ValueError(f"duplicate JSON key: {key}")
        value[key] = item
    return value


def _parse_reset_response_metadata(message: object) -> tuple[int, int]:
    text = str(message)
    marker = "; reset_metadata_v1="
    _, separator, payload = text.rpartition(marker)
    if not separator:
        raise RuntimeError(
            "simulation Reset response lacks the versioned metadata trailer"
        )
    try:
        metadata = json.loads(
            payload, object_pairs_hook=_json_object_without_duplicate_keys
        )
    except (TypeError, ValueError) as exc:
        raise RuntimeError(
            "simulation Reset metadata trailer is not valid JSON"
        ) from exc
    expected_keys = {"schema_version", "generation", "boundary_clock_ns"}
    if not isinstance(metadata, dict) or set(metadata) != expected_keys:
        raise RuntimeError(
            "simulation Reset metadata trailer has an invalid schema"
        )
    schema_version = metadata["schema_version"]
    if isinstance(schema_version, bool) or not isinstance(schema_version, int):
        raise RuntimeError(
            "simulation Reset metadata schema_version must be an integer"
        )
    if schema_version != 1:
        raise RuntimeError("unsupported simulation Reset metadata schema")
    generation = metadata["generation"]
    boundary_clock_ns = metadata["boundary_clock_ns"]
    if (
        isinstance(generation, bool)
        or not isinstance(generation, int)
        or isinstance(boundary_clock_ns, bool)
        or not isinstance(boundary_clock_ns, int)
    ):
        raise RuntimeError(
            "simulation Reset generation/boundary_clock_ns must be integers"
        )
    if generation <= 0:
        raise RuntimeError("simulation Reset generation must be positive")
    if boundary_clock_ns < 0:
        raise RuntimeError(
            "simulation Reset boundary_clock_ns must be non-negative"
        )
    return generation, boundary_clock_ns


def _coherent_group_ready(
    current_sequences: tuple[int, int, int],
    credited_sequences: tuple[int, int, int],
) -> bool:
    return all(
        current > credited
        for current, credited in zip(
            current_sequences, credited_sequences, strict=True
        )
    )


def _post_reset_observation_ns(
    *,
    boundary_clock_ns: int,
    clock_ns: int | None,
    odom_stamp_ns: int | None,
    joint_stamp_ns: int | None,
) -> int | None:
    stamps = (clock_ns, odom_stamp_ns, joint_stamp_ns)
    if any(stamp is None for stamp in stamps):
        return None
    concrete = tuple(int(stamp) for stamp in stamps if stamp is not None)
    if any(stamp <= boundary_clock_ns for stamp in concrete):
        return None
    return min(concrete)


def _timestamp_regression_topics(
    current_stamps_ns: tuple[int, int, int],
    high_watermarks_ns: tuple[int, int, int],
) -> tuple[str, ...]:
    names = ("clock", "odom", "joint_states")
    return tuple(
        name
        for name, current, high_watermark in zip(
            names,
            current_stamps_ns,
            high_watermarks_ns,
            strict=True,
        )
        if current < high_watermark
    )


def _update_stationary_window(
    *,
    observation_ns: int | None,
    last_observation_ns: int | None,
    stationary_since_ns: int | None,
    gates_passed: bool,
) -> tuple[int | None, int | None, int, str]:
    """Advance a settle window only for a newer coherent stream group."""
    if observation_ns is None:
        if not gates_passed:
            return None, last_observation_ns, 0, "blocked"
        return (
            stationary_since_ns,
            last_observation_ns,
            0,
            "waiting_for_observation",
        )
    if last_observation_ns is not None:
        if observation_ns < last_observation_ns:
            return (
                None,
                last_observation_ns,
                0,
                "observation_regression",
            )
        if observation_ns == last_observation_ns:
            if not gates_passed:
                return None, last_observation_ns, 0, "blocked"
            return (
                stationary_since_ns,
                last_observation_ns,
                0,
                "waiting_for_observation",
            )
    if not gates_passed:
        # A moving coherent group clears the settle window, but its timestamp
        # still becomes the monotonic high-watermark.  Otherwise a later
        # out-of-order stationary group could be credited across known motion.
        return None, observation_ns, 0, "blocked"
    if stationary_since_ns is None or observation_ns < stationary_since_ns:
        stationary_since_ns = observation_ns
    return (
        stationary_since_ns,
        observation_ns,
        observation_ns - stationary_since_ns,
        "advanced",
    )


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
        self._reset_wait_stamp_high_watermarks_ns: dict[
            str, int | None
        ] | None = None
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

    def _observe_reset_wait_timestamp(self, topic: str, stamp_ns: int) -> None:
        """Retain callback maxima while a Reset Trigger response is pending."""
        high_watermarks = self._reset_wait_stamp_high_watermarks_ns
        if high_watermarks is None:
            return
        high_watermark = high_watermarks[topic]
        high_watermarks[topic] = (
            stamp_ns
            if high_watermark is None
            else max(high_watermark, stamp_ns)
        )

    def _clock_callback(self, message: Clock) -> None:
        stamp = _stamp_ns(message.clock)
        self._observe_timestamp("clock", stamp)
        self._observe_reset_wait_timestamp("clock", stamp)
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
        self._observe_reset_wait_timestamp("odom", stamp)
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
        self._observe_reset_wait_timestamp("joint_states", stamp)
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

        schema_version = value("schema_version")
        if (
            isinstance(schema_version, bool)
            or not isinstance(schema_version, int)
            or schema_version != 4
        ):
            raise RuntimeError(
                "Isaac runtime provenance schema must be integer 4 for new "
                "motion reports"
            )
        provenance = {
            "verified": True,
            "schema_version": schema_version,
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
                "kinematics": {
                    "profile_id": value("robot.kinematics.profile_id"),
                    "lifecycle": value("robot.kinematics.lifecycle"),
                    "wheel_radius_m": value(
                        "robot.kinematics.wheel_radius_m"
                    ),
                    "wheel_width_m": value(
                        "robot.kinematics.wheel_width_m"
                    ),
                    "geometric_track_width_m": value(
                        "robot.kinematics.geometric_track_width_m"
                    ),
                    "effective_track_width_m": value(
                        "robot.kinematics.effective_track_width_m"
                    ),
                    "controller_contract_verified": value(
                        "robot.kinematics.controller_contract_verified"
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

    def _stream_sim_ages_ns(self) -> dict[str, int | None]:
        clock_ns = self._clock_ns
        return {
            "odom": (
                None
                if clock_ns is None or self._latest_odom is None
                else clock_ns - self._latest_odom.stamp_ns
            ),
            "joint_states": (
                None
                if clock_ns is None or self._latest_joint is None
                else clock_ns - self._latest_joint.stamp_ns
            ),
        }

    def _stream_freshness_gate_status(self) -> dict[str, bool]:
        ages_ns = self._stream_sim_ages_ns()
        max_age_ns = round(
            self._config.sampling.max_sample_age_sec * NANOSECONDS_PER_SECOND
        )
        max_future_skew_ns = round(
            self._config.sampling.max_future_skew_sec * NANOSECONDS_PER_SECOND
        )
        gates: dict[str, bool] = {}
        for topic, age_ns in ages_ns.items():
            gates[f"{topic}_not_stale"] = (
                age_ns is not None and age_ns <= max_age_ns
            )
            gates[f"{topic}_not_too_far_ahead"] = (
                age_ns is not None and age_ns >= -max_future_skew_ns
            )
        return gates

    def _streams_fresh(self) -> bool:
        # The three DDS subscriptions are not an atomic snapshot. Odom or
        # JointState for the current physics tick may be handled before the
        # matching /clock callback, so accept only the explicitly bounded
        # callback-phase lead while retaining the independent stale bound.
        return all(self._stream_freshness_gate_status().values())

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
        freshness_gates = self._stream_freshness_gate_status()
        gates = {
            "streams_fresh": all(freshness_gates.values()),
            **{
                f"stream:{name}": passed
                for name, passed in freshness_gates.items()
            },
            "wall_streams_fresh": self._wall_streams_fresh(),
        }
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
        credited_sequences: tuple[int, int, int],
        credited_stamp_high_watermarks_ns: tuple[int, int, int],
        received_stamp_high_watermarks_ns: tuple[int, int, int],
        reset_generation: int,
        boundary_clock_ns: int,
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
        sim_ages_ns = self._stream_sim_ages_ns()
        sim_stamps_ns = [
            stamp
            for stamp in (
                clock_ns,
                None if odom is None else odom.stamp_ns,
                None if joint is None else joint.stamp_ns,
            )
            if stamp is not None
        ]
        diagnostic: dict[str, object] = {
            "reset_epoch": {
                "generation": reset_generation,
                "boundary_clock_ns": boundary_clock_ns,
                "credited_sequences": {
                    "clock": credited_sequences[0],
                    "odom": credited_sequences[1],
                    "joint_states": credited_sequences[2],
                },
                "credited_stamp_high_watermarks_ns": {
                    "clock": credited_stamp_high_watermarks_ns[0],
                    "odom": credited_stamp_high_watermarks_ns[1],
                    "joint_states": credited_stamp_high_watermarks_ns[2],
                },
                "received_stamp_high_watermarks_ns": {
                    "clock": received_stamp_high_watermarks_ns[0],
                    "odom": received_stamp_high_watermarks_ns[1],
                    "joint_states": received_stamp_high_watermarks_ns[2],
                },
            },
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
                topic: (
                    None
                    if age_ns is None
                    else age_ns / NANOSECONDS_PER_SECOND
                )
                for topic, age_ns in sim_ages_ns.items()
            },
            "sim_timestamp_span_sec": (
                None
                if len(sim_stamps_ns) < 3
                else (max(sim_stamps_ns) - min(sim_stamps_ns))
                / NANOSECONDS_PER_SECOND
            ),
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
                "max_sample_age_sec": (
                    self._config.sampling.max_sample_age_sec
                ),
                "max_future_skew_sec": (
                    self._config.sampling.max_future_skew_sec
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
        started = time.monotonic()
        reset = self._config.reset
        if not self._reset_client.wait_for_service(
            timeout_sec=reset.service_timeout_sec
        ):
            self._raise_if_shutdown()
            raise RuntimeError(f"reset service unavailable: {reset.service}")
        self._reset_wait_stamp_high_watermarks_ns = {
            "clock": None,
            "odom": None,
            "joint_states": None,
        }
        try:
            future = self._reset_client.call_async(Trigger.Request())
            if not self._wait_future(
                future, time.monotonic() + reset.service_timeout_sec
            ):
                raise TimeoutError("simulation Reset response timed out")
            response = future.result()
            response_wall = time.monotonic()
            reset_wait_stamp_high_watermarks_ns = tuple(
                self._reset_wait_stamp_high_watermarks_ns[topic]
                for topic in ("clock", "odom", "joint_states")
            )
        finally:
            self._reset_wait_stamp_high_watermarks_ns = None
        if response is None or not response.success:
            message = "no response" if response is None else response.message
            raise RuntimeError(f"simulation Reset failed: {message}")

        reset_generation, boundary_clock_ns = _parse_reset_response_metadata(
            response.message
        )
        # Sequence watermarks prevent sample reuse. The service-provided
        # simulation-time boundary separately rejects messages that entered a
        # DDS queue before the completed reset epoch but were handled later.
        barriers = (
            self._clock_sequence,
            self._odom_sequence,
            self._joint_sequence,
        )
        credited_sequences = barriers
        barrier_stamps_ns = (
            self._clock_ns,
            (
                None
                if self._latest_odom is None
                else self._latest_odom.stamp_ns
            ),
            (
                None
                if self._latest_joint is None
                else self._latest_joint.stamp_ns
            ),
        )
        barrier_stamp_high_watermarks_ns = tuple(
            max(
                boundary_clock_ns,
                boundary_clock_ns if stamp is None else stamp,
                (
                    boundary_clock_ns
                    if wait_stamp is None
                    else wait_stamp
                ),
            )
            for stamp, wait_stamp in zip(
                barrier_stamps_ns,
                reset_wait_stamp_high_watermarks_ns,
                strict=True,
            )
        )
        credited_stamp_high_watermarks_ns = barrier_stamp_high_watermarks_ns
        received_sequences = barriers
        received_stamp_high_watermarks_ns = barrier_stamp_high_watermarks_ns
        recovery_deadline = time.monotonic() + reset.recovery_timeout_sec
        stationary_since_ns: int | None = None
        # No settle interval may begin before the latest evidence already
        # processed while the Trigger response was in flight.  The streams can
        # differ by one callback phase, so use the maximum barrier watermark
        # as the conservative observation floor.
        last_observation_ns: int | None = max(
            barrier_stamp_high_watermarks_ns
        )
        longest_stationary_duration_ns = 0
        observation_counts = {
            "coherent_group_not_ready": 0,
            "pre_boundary_group": 0,
            "not_stationary": 0,
            "stationary": 0,
            "coherent_without_time_progress": 0,
            "observation_regression": 0,
            "receive_timestamp_regression": 0,
            "coherent_timestamp_regression": 0,
        }
        gate_names = tuple(self._stationary_gate_status())
        immediate_gate_names = (
            "wall_streams_fresh",
            *(
                name for name in gate_names if name.startswith("stream:")
            ),
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
            "sim_age_sec": {
                "odom": {"minimum": None, "maximum": None},
                "joint_states": {"minimum": None, "maximum": None},
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
            current_sequences = (
                self._clock_sequence,
                self._odom_sequence,
                self._joint_sequence,
            )
            current_receive_stamps_ns = (
                self._clock_ns,
                (
                    None
                    if self._latest_odom is None
                    else self._latest_odom.stamp_ns
                ),
                (
                    None
                    if self._latest_joint is None
                    else self._latest_joint.stamp_ns
                ),
            )
            receive_regression_topics: list[str] = []
            next_received_high_watermarks = list(
                received_stamp_high_watermarks_ns
            )
            for index, topic in enumerate(
                ("clock", "odom", "joint_states")
            ):
                if current_sequences[index] <= received_sequences[index]:
                    continue
                stamp_ns = current_receive_stamps_ns[index]
                assert stamp_ns is not None
                if stamp_ns < received_stamp_high_watermarks_ns[index]:
                    observation_counts["receive_timestamp_regression"] += 1
                    key = f"receive_timestamp_regression:{topic}"
                    violation_counts[key] = violation_counts.get(key, 0) + 1
                    receive_regression_topics.append(topic)
                else:
                    next_received_high_watermarks[index] = stamp_ns
            received_sequences = current_sequences
            received_stamp_high_watermarks_ns = tuple(
                next_received_high_watermarks
            )
            if receive_regression_topics:
                if (
                    stationary_since_ns is not None
                    and last_observation_ns is not None
                ):
                    longest_stationary_duration_ns = max(
                        longest_stationary_duration_ns,
                        max(0, last_observation_ns - stationary_since_ns),
                    )
                stationary_since_ns = None
            sim_age_extrema = peak_observed["sim_age_sec"]
            assert isinstance(sim_age_extrema, dict)
            for topic, age_ns in self._stream_sim_ages_ns().items():
                if age_ns is None:
                    continue
                age_sec = age_ns / NANOSECONDS_PER_SECOND
                topic_extrema = sim_age_extrema[topic]
                assert isinstance(topic_extrema, dict)
                minimum = topic_extrema["minimum"]
                maximum = topic_extrema["maximum"]
                topic_extrema["minimum"] = (
                    age_sec if minimum is None else min(float(minimum), age_sec)
                )
                topic_extrema["maximum"] = (
                    age_sec if maximum is None else max(float(maximum), age_sec)
                )
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
            if not _coherent_group_ready(
                current_sequences, credited_sequences
            ):
                observation_counts["coherent_group_not_ready"] += 1
                immediate_blockers = [
                    name
                    for name in immediate_gate_names
                    if not gate_status[name]
                ]
                if immediate_blockers:
                    observation_counts["not_stationary"] += 1
                    for name in immediate_blockers:
                        violation_counts[name] += 1
                    if (
                        stationary_since_ns is not None
                        and last_observation_ns is not None
                    ):
                        longest_stationary_duration_ns = max(
                            longest_stationary_duration_ns,
                            max(
                                0,
                                last_observation_ns - stationary_since_ns,
                            ),
                        )
                    stationary_since_ns = None
                continue

            # Consume every stream watermark exactly once, including invalid
            # pre-boundary or incoherent groups, so queued data cannot be
            # recombined indefinitely with a later callback.
            credited_sequences = current_sequences
            odom_stamp_ns = (
                None if self._latest_odom is None else self._latest_odom.stamp_ns
            )
            joint_stamp_ns = (
                None if self._latest_joint is None else self._latest_joint.stamp_ns
            )
            observation_ns = _post_reset_observation_ns(
                boundary_clock_ns=boundary_clock_ns,
                clock_ns=self._clock_ns,
                odom_stamp_ns=odom_stamp_ns,
                joint_stamp_ns=joint_stamp_ns,
            )
            if observation_ns is None:
                observation_counts["pre_boundary_group"] += 1
                if (
                    stationary_since_ns is not None
                    and last_observation_ns is not None
                ):
                    longest_stationary_duration_ns = max(
                        longest_stationary_duration_ns,
                        max(0, last_observation_ns - stationary_since_ns),
                    )
                stationary_since_ns = None
                continue

            assert self._clock_ns is not None
            assert odom_stamp_ns is not None
            assert joint_stamp_ns is not None
            current_stamps_ns = (
                self._clock_ns,
                odom_stamp_ns,
                joint_stamp_ns,
            )
            required_stamp_high_watermarks_ns = tuple(
                max(credited, received)
                for credited, received in zip(
                    credited_stamp_high_watermarks_ns,
                    received_stamp_high_watermarks_ns,
                    strict=True,
                )
            )
            regression_topics = _timestamp_regression_topics(
                current_stamps_ns,
                required_stamp_high_watermarks_ns,
            )
            if regression_topics:
                observation_counts["coherent_timestamp_regression"] += 1
                for topic in regression_topics:
                    key = f"coherent_timestamp_regression:{topic}"
                    violation_counts[key] = violation_counts.get(key, 0) + 1
                if (
                    stationary_since_ns is not None
                    and last_observation_ns is not None
                ):
                    longest_stationary_duration_ns = max(
                        longest_stationary_duration_ns,
                        max(0, last_observation_ns - stationary_since_ns),
                    )
                stationary_since_ns = None
                credited_stamp_high_watermarks_ns = (
                    required_stamp_high_watermarks_ns
                )
                continue
            credited_stamp_high_watermarks_ns = current_stamps_ns

            previous_stationary_since_ns = stationary_since_ns
            previous_observation_ns = last_observation_ns
            (
                stationary_since_ns,
                last_observation_ns,
                settled_ns,
                window_status,
            ) = _update_stationary_window(
                observation_ns=observation_ns,
                last_observation_ns=last_observation_ns,
                stationary_since_ns=stationary_since_ns,
                gates_passed=all(gate_status.values()),
            )
            if window_status == "blocked":
                observation_counts["not_stationary"] += 1
                for gate, passed in gate_status.items():
                    if not passed:
                        violation_counts[gate] += 1
                if previous_stationary_since_ns is not None:
                    longest_stationary_duration_ns = max(
                        longest_stationary_duration_ns,
                        max(
                            0,
                            (
                                previous_observation_ns
                                - previous_stationary_since_ns
                                if previous_observation_ns is not None
                                else 0
                            ),
                        ),
                    )
                continue
            if window_status == "waiting_for_observation":
                observation_counts["coherent_without_time_progress"] += 1
                continue
            if window_status == "observation_regression":
                observation_counts["observation_regression"] += 1
                if (
                    previous_stationary_since_ns is not None
                    and previous_observation_ns is not None
                ):
                    longest_stationary_duration_ns = max(
                        longest_stationary_duration_ns,
                        max(
                            0,
                            previous_observation_ns
                            - previous_stationary_since_ns,
                        ),
                    )
                continue
            observation_counts["stationary"] += 1
            if settled_ns >= round(
                reset.settle_duration_sec * NANOSECONDS_PER_SECOND
            ):
                assert self._clock_ns is not None
                after_clock = self._clock_ns
                return {
                    "service": reset.service,
                    "response_message": str(response.message),
                    "reset_generation": reset_generation,
                    "reset_boundary_clock_ns": boundary_clock_ns,
                    "service_latency_wall_sec": response_wall - started,
                    "recovery_latency_wall_sec": time.monotonic() - response_wall,
                    "clock_before_ns": before_clock,
                    "clock_after_ns": after_clock,
                    "clock_rollback_observed": (
                        before_clock is not None and after_clock < before_clock
                    ),
                    "fresh_clock_received": credited_sequences[0] > barriers[0],
                    "fresh_odom_received": credited_sequences[1] > barriers[1],
                    "fresh_joint_states_received": (
                        credited_sequences[2] > barriers[2]
                    ),
                    "stationary_settle_duration_sec": reset.settle_duration_sec,
                    "recovery_observation_counts": dict(observation_counts),
                    "recovery_violation_counts": dict(violation_counts),
                    "recovery_peak_observed": _diagnostic_json_safe(
                        peak_observed
                    ),
                    "credited_stamp_high_watermarks_ns": {
                        "clock": credited_stamp_high_watermarks_ns[0],
                        "odom": credited_stamp_high_watermarks_ns[1],
                        "joint_states": (
                            credited_stamp_high_watermarks_ns[2]
                        ),
                    },
                    "received_stamp_high_watermarks_ns": {
                        "clock": received_stamp_high_watermarks_ns[0],
                        "odom": received_stamp_high_watermarks_ns[1],
                        "joint_states": received_stamp_high_watermarks_ns[2],
                    },
                    "longest_stationary_duration_sec": max(
                        longest_stationary_duration_ns, settled_ns
                    )
                    / NANOSECONDS_PER_SECOND,
                }
        if stationary_since_ns is not None and last_observation_ns is not None:
            longest_stationary_duration_ns = max(
                longest_stationary_duration_ns,
                max(0, last_observation_ns - stationary_since_ns),
            )
        diagnostic = self._reset_recovery_diagnostic(
            barriers,
            credited_sequences,
            credited_stamp_high_watermarks_ns,
            received_stamp_high_watermarks_ns,
            reset_generation,
            boundary_clock_ns,
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
