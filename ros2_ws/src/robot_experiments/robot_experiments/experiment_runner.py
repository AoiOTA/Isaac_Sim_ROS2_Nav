"""Run deterministic NavigateToPose trials and write reproducible manifests."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import math
import os
from pathlib import Path
import time
from typing import Any

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, Empty as EmptyMessage
from std_srvs.srv import Empty, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .configuration import ConfigurationError
from .metrics import (
    SingleRunObservation,
    SingleRunThresholds,
    evaluate_single_run,
    path_length,
    wrap_angle,
)
from .report import configuration_sha256, write_run_report
from .scenario import (
    Scenario,
    load_scenario,
    validate_dynamic_physical_contract,
    validate_dynamic_runtime_contract,
    validate_navigation_runner_scenario,
)
from .spawn_poses import SpawnPose, load_spawn_pose


@dataclass(frozen=True)
class OdometrySample:
    x: float
    y: float
    yaw_rad: float
    linear_speed_mps: float
    angular_speed_radps: float
    stamp_s: float
    received_at: float


@dataclass(frozen=True)
class CommandSample:
    linear_speed_mps: float
    angular_speed_radps: float
    stamp_s: float


class ExperimentIsolationError(RuntimeError):
    """Raised when an old action may still contaminate the next trial."""


def _yaw_from_quaternion(quaternion: Any) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def _sample_from_odometry(message: Odometry) -> OdometrySample | None:
    stamp_s = message.header.stamp.sec + message.header.stamp.nanosec * 1.0e-9
    values = (
        message.pose.pose.position.x,
        message.pose.pose.position.y,
        _yaw_from_quaternion(message.pose.pose.orientation),
        message.twist.twist.linear.x,
        message.twist.twist.angular.z,
        stamp_s,
    )
    if not all(math.isfinite(value) for value in values):
        return None
    return OdometrySample(*values, received_at=time.monotonic())


class ExperimentRunner(Node):
    """Sequential runner; ground truth is sampled but never republished or controlled from."""

    def __init__(self) -> None:
        super().__init__("experiment_runner")
        scenario_file = str(self.declare_parameter("scenario_file", "").value).strip()
        if not scenario_file:
            raise ConfigurationError("scenario_file is required")
        self._scenario: Scenario = load_scenario(scenario_file)
        validate_navigation_runner_scenario(self._scenario)

        configured_spawn_file = str(
            self.declare_parameter("spawn_poses_file", "").value
        ).strip()
        spawn_file = configured_spawn_file or os.environ.get(
            "ISAAC_NAV_SPAWN_POSES", ""
        ).strip()
        if not spawn_file:
            raise ConfigurationError(
                "spawn_poses_file is required (or set ISAAC_NAV_SPAWN_POSES)"
            )
        self._spawn_pose: SpawnPose = load_spawn_pose(
            spawn_file,
            self._scenario.spawn_pose_name,
            require_calibrated=True,
        )

        robot_override = str(self.declare_parameter("robot_config_file", "").value).strip()
        nav2_override = str(self.declare_parameter("nav2_config_file", "").value).strip()
        robot_config = (
            Path(robot_override).expanduser().resolve()
            if robot_override
            else self._scenario.resolve_path(self._scenario.robot_config_file)
        )
        nav2_config = (
            Path(nav2_override).expanduser().resolve()
            if nav2_override
            else self._scenario.resolve_path(self._scenario.nav2_config_file)
        )
        self._robot_config_hash = configuration_sha256(robot_config)
        self._nav2_config_hash = configuration_sha256(nav2_config)
        self._dynamic_config_hash = None
        if self._scenario.dynamic_config_file is not None:
            dynamic_config = self._scenario.resolve_path(
                self._scenario.dynamic_config_file
            )
            validate_dynamic_physical_contract(
                self._scenario, self._spawn_pose, dynamic_config
            )
            self._dynamic_config_hash = configuration_sha256(dynamic_config)

        self._output_directory = Path(
            str(self.declare_parameter("output_directory", "data/experiment_runs").value)
        ).expanduser()
        self._reset_service_name = str(
            self.declare_parameter("reset_service", "/simulation/reset").value
        )
        self._action_name = str(
            self.declare_parameter("navigate_action", "/navigate_to_pose").value
        )
        self._service_timeout_sec = float(
            self.declare_parameter("service_timeout_sec", 30.0).value
        )
        self._clock_timeout_sec = float(
            self.declare_parameter("clock_timeout_sec", 30.0).value
        )
        self._odom_max_age_sec = float(
            self.declare_parameter("odom_max_age_sec", 0.5).value
        )
        self._tf_gap_tolerance_sec = float(
            self.declare_parameter("tf_gap_tolerance_sec", 1.0).value
        )
        self._collision_lock_timeout_sec = float(
            self.declare_parameter("collision_lock_timeout_sec", 5.0).value
        )
        self._reset_recovery_timeout_sec = float(
            self.declare_parameter("reset_recovery_timeout_sec", 30.0).value
        )
        self._reset_tf_stability_sec = float(
            self.declare_parameter("reset_tf_stability_sec", 0.5).value
        )
        self._reset_tf_translation_tolerance_m = float(
            self.declare_parameter(
                "reset_tf_translation_tolerance_m", 0.05
            ).value
        )
        self._reset_tf_yaw_tolerance_rad = float(
            self.declare_parameter(
                "reset_tf_yaw_tolerance_rad", math.radians(3.0)
            ).value
        )
        if min(
            self._service_timeout_sec,
            self._clock_timeout_sec,
            self._odom_max_age_sec,
            self._tf_gap_tolerance_sec,
            self._collision_lock_timeout_sec,
            self._reset_recovery_timeout_sec,
            self._reset_tf_stability_sec,
            self._reset_tf_translation_tolerance_m,
            self._reset_tf_yaw_tolerance_rad,
        ) <= 0.0:
            raise ConfigurationError("runner timeouts must be positive")

        reliable = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        clock_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._clock_ready = False
        self._clock_stamp = None
        self._clock_subscription = self.create_subscription(
            Clock, "/clock", self._clock_callback, clock_qos
        )
        self._gt_subscription = self.create_subscription(
            Odometry,
            str(self.declare_parameter("ground_truth_topic", "/ground_truth/odom").value),
            self._ground_truth_callback,
            reliable,
        )
        self._odom_subscription = self.create_subscription(
            Odometry,
            str(self.declare_parameter("odom_topic", "/odom").value),
            self._odom_callback,
            reliable,
        )
        self._command_subscription = self.create_subscription(
            Twist,
            str(self.declare_parameter("command_topic", "/cmd_vel").value),
            self._command_callback,
            reliable,
        )
        self._status_subscriptions = [
            self.create_subscription(
                Bool,
                str(self.declare_parameter("collision_topic", "/simulation/collision").value),
                self._collision_callback,
                reliable,
            ),
            self.create_subscription(
                CollisionMonitorState,
                str(
                    self.declare_parameter(
                        "collision_monitor_state_topic", "/collision_monitor_state"
                    ).value
                ),
                self._collision_lock_callback,
                reliable,
            ),
            self.create_subscription(
                EmptyMessage,
                str(
                    self.declare_parameter(
                        "localization_seeded_topic",
                        "/simulation/localization_seeded",
                    ).value
                ),
                self._localization_seeded_callback,
                reliable,
            ),
        ]
        self._reset_client = self.create_client(Trigger, self._reset_service_name)
        self._localization_buffer_client = self.create_client(
            Empty, "/slam_toolbox/clear_localization_buffer"
        )
        self._isaac_parameter_client = AsyncParameterClient(
            self,
            str(
                self.declare_parameter(
                    "isaac_node_name", "/isaac_navigation_sim"
                ).value
            ),
        )
        self._navigate_client = ActionClient(self, NavigateToPose, self._action_name)
        self._collision_monitor_state_client = self.create_client(
            GetState,
            str(
                self.declare_parameter(
                    "collision_monitor_state_service",
                    "/collision_monitor/get_state",
                ).value
            ),
        )
        self._nav2_managed_state_clients = tuple(
            (
                node_name,
                self.create_client(GetState, f"/{node_name}/get_state"),
            )
            for node_name in (
                "controller_server",
                "planner_server",
                "behavior_server",
                "velocity_smoother",
                "collision_monitor",
                "bt_navigator",
            )
        )
        self._costmap_clear_clients = (
            (
                "global costmap",
                self.create_client(
                    ClearEntireCostmap,
                    "/global_costmap/clear_entirely_global_costmap",
                ),
            ),
            (
                "local costmap",
                self.create_client(
                    ClearEntireCostmap,
                    "/local_costmap/clear_entirely_local_costmap",
                ),
            ),
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self._dynamic_runtime_contract: dict[str, Any] = {
            "verified": False,
        }
        self._localization_seed_epoch = 0
        self._collision_monitor_active = False
        self._clear_run_state()

    def _clear_run_state(self) -> None:
        self._ground_truth_samples: list[OdometrySample] = []
        self._odom_samples: list[OdometrySample] = []
        self._command_samples: list[CommandSample] = []
        self._navigation_active = False
        self._navigation_start_stamp_s: float | None = None
        self._navigation_end_stamp_s: float | None = None
        self._route_feedback_count = 0
        self._minimum_poses_remaining: int | None = None
        self._maximum_route_recoveries = 0
        self._collision_seen = False
        self._collision_detected = False
        self._localization_seen = False
        self._localization_lost = False
        self._lock_status_seen = False
        self._lock_started_at: float | None = None
        self._collision_monitor_locked = False
        self._tf_ever_available = False
        self._last_tf_stamp_s: float | None = None
        self._last_tf_ok_at: float | None = None
        self._tf_interrupted = False

    def _clock_callback(self, message: Clock) -> None:
        if message.clock.sec != 0 or message.clock.nanosec != 0:
            self._clock_ready = True
            self._clock_stamp = message.clock

    def _ground_truth_callback(self, message: Odometry) -> None:
        sample = _sample_from_odometry(message)
        if sample is not None:
            self._ground_truth_samples.append(sample)

    def _odom_callback(self, message: Odometry) -> None:
        sample = _sample_from_odometry(message)
        if sample is not None:
            self._odom_samples.append(sample)

    def _command_callback(self, message: Twist) -> None:
        stamp_s = self._clock_seconds()
        values = (
            message.linear.x,
            message.angular.z,
            stamp_s,
        )
        if (
            self._navigation_active
            and stamp_s is not None
            and all(math.isfinite(value) for value in values)
        ):
            self._command_samples.append(
                CommandSample(
                    linear_speed_mps=float(message.linear.x),
                    angular_speed_radps=float(message.angular.z),
                    stamp_s=stamp_s,
                )
            )

    def _collision_callback(self, message: Bool) -> None:
        self._collision_seen = True
        self._collision_detected = self._collision_detected or bool(message.data)

    def _collision_lock_callback(self, message: CollisionMonitorState) -> None:
        self._lock_status_seen = True
        stopped = message.action_type == CollisionMonitorState.STOP
        if stopped and self._lock_started_at is None:
            self._lock_started_at = time.monotonic()
        elif not stopped:
            self._lock_started_at = None

    def _localization_seeded_callback(self, message: EmptyMessage) -> None:
        del message
        self._localization_seed_epoch += 1

    @staticmethod
    def _raise_if_shutdown() -> None:
        if not rclpy.ok():
            raise ExternalShutdownException()

    def _clock_seconds(self) -> float | None:
        if self._clock_stamp is None:
            return None
        return self._clock_stamp.sec + self._clock_stamp.nanosec * 1.0e-9

    @staticmethod
    def _transform_stamp_seconds(transform: Any) -> float:
        stamp = transform.header.stamp
        return stamp.sec + stamp.nanosec * 1.0e-9

    def _lookup_fresh_map_to_odom(self) -> tuple[Any, float] | None:
        try:
            transform = self._tf_buffer.lookup_transform("map", "odom", Time())
        except TransformException:
            return None
        clock_s = self._clock_seconds()
        stamp_s = self._transform_stamp_seconds(transform)
        if clock_s is None or stamp_s <= 0.0:
            return None
        age_s = clock_s - stamp_s
        # SLAM Toolbox intentionally future-dates map->odom by
        # transform_timeout; bound the absolute gap instead of rejecting that
        # valid publication policy.
        if abs(age_s) > self._tf_gap_tolerance_sec:
            return None
        return transform, stamp_s

    def _latest_map_to_odom_stamp(self) -> float | None:
        try:
            transform = self._tf_buffer.lookup_transform("map", "odom", Time())
        except TransformException:
            return None
        stamp_s = self._transform_stamp_seconds(transform)
        return stamp_s if math.isfinite(stamp_s) and stamp_s > 0.0 else None

    def _update_health(self) -> None:
        now = time.monotonic()
        if self._lock_started_at is not None:
            if now - self._lock_started_at >= self._collision_lock_timeout_sec:
                self._collision_monitor_locked = True
        fresh_transform = self._lookup_fresh_map_to_odom()
        if fresh_transform is not None:
            _, stamp_s = fresh_transform
            self._localization_seen = True
            self._tf_ever_available = True
            if self._last_tf_stamp_s != stamp_s:
                self._last_tf_stamp_s = stamp_s
                self._last_tf_ok_at = now
        else:
            if self._tf_ever_available and self._last_tf_ok_at is not None:
                if now - self._last_tf_ok_at >= self._tf_gap_tolerance_sec:
                    self._tf_interrupted = True
                    self._localization_lost = True

    def _spin_once(self, timeout_sec: float = 0.05) -> None:
        rclpy.spin_once(self, timeout_sec=timeout_sec)
        self._update_health()

    def _wait_until(self, predicate, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while not predicate():
            if not rclpy.ok():
                raise ExternalShutdownException()
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False
            self._spin_once(min(0.1, remaining))
        return True

    def _wait_future(self, future, deadline: float) -> bool:
        while not future.done():
            if not rclpy.ok():
                raise ExternalShutdownException()
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False
            self._spin_once(min(0.1, remaining))
        return True

    def _set_reset_seed(self, seed: int) -> None:
        if not self._isaac_parameter_client.wait_for_services(
            timeout_sec=self._service_timeout_sec
        ):
            self._raise_if_shutdown()
            raise RuntimeError("Isaac reset parameter services are unavailable")
        future = self._isaac_parameter_client.set_parameters(
            [
                Parameter("reset_seed", value=seed),
                Parameter("reset_pose_name", value=self._scenario.spawn_pose_name),
            ]
        )
        deadline = time.monotonic() + self._service_timeout_sec
        if not self._wait_future(future, deadline):
            raise TimeoutError("setting deterministic reset parameters timed out")
        response = future.result()
        if response is None:
            raise RuntimeError("setting deterministic reset parameters returned no response")
        failures = [result.reason for result in response.results if not result.successful]
        if failures:
            raise RuntimeError(f"Isaac rejected reset parameters: {failures}")

    def _verify_dynamic_runtime_contract(self) -> None:
        if not self._isaac_parameter_client.wait_for_services(
            timeout_sec=self._service_timeout_sec
        ):
            self._raise_if_shutdown()
            raise RuntimeError("Isaac parameter services are unavailable")
        names = [
            "dynamic_obstacles_enabled",
            "dynamic_obstacles_config_sha256",
            "dynamic_obstacle_ids",
        ]
        future = self._isaac_parameter_client.get_parameters(names)
        deadline = time.monotonic() + self._service_timeout_sec
        if not self._wait_future(future, deadline):
            raise TimeoutError(
                "reading the Isaac dynamic obstacle contract timed out"
            )
        response = future.result()
        if response is None or len(response.values) != len(names):
            raise RuntimeError(
                "Isaac returned an incomplete dynamic obstacle contract"
            )
        enabled, config_hash, obstacle_ids = (
            parameter_value_to_python(value) for value in response.values
        )
        if not isinstance(enabled, bool):
            raise RuntimeError("Isaac dynamic_obstacles_enabled is not boolean")
        if not isinstance(config_hash, str) or not config_hash:
            raise RuntimeError(
                "Isaac dynamic_obstacles_config_sha256 is invalid"
            )
        if (
            not isinstance(obstacle_ids, list)
            or not all(isinstance(value, str) for value in obstacle_ids)
        ):
            raise RuntimeError("Isaac dynamic_obstacle_ids is invalid")
        runtime_ids = tuple(obstacle_ids)
        validate_dynamic_runtime_contract(
            self._scenario,
            runtime_enabled=enabled,
            runtime_config_hash=config_hash,
            runtime_obstacle_ids=runtime_ids,
            expected_config_hash=self._dynamic_config_hash,
        )
        self._dynamic_runtime_contract = {
            "verified": True,
            "enabled": enabled,
            "config_sha256": config_hash,
            "obstacle_ids": list(runtime_ids),
        }

    def _verify_collision_monitor_active(self) -> None:
        if not self._collision_monitor_state_client.wait_for_service(
            timeout_sec=self._service_timeout_sec
        ):
            self._raise_if_shutdown()
            raise RuntimeError(
                "Collision Monitor lifecycle state service is unavailable"
            )
        future = self._collision_monitor_state_client.call_async(
            GetState.Request()
        )
        if not self._wait_future(
            future, time.monotonic() + self._service_timeout_sec
        ):
            raise TimeoutError(
                "reading Collision Monitor lifecycle state timed out"
            )
        response = future.result()
        if (
            response is None
            or response.current_state.id != State.PRIMARY_STATE_ACTIVE
        ):
            label = (
                "no response"
                if response is None
                else response.current_state.label
            )
            raise RuntimeError(
                f"Collision Monitor is not active: {label}"
            )
        self._collision_monitor_active = True

    def _wait_for_nav2_managed_nodes_active(self) -> None:
        """Do not dispatch a goal while the reset recovery gate is resuming Nav2."""

        deadline = time.monotonic() + self._reset_recovery_timeout_sec
        latest_states: dict[str, str] = {}
        while rclpy.ok():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                state_summary = ", ".join(
                    f"{name}={latest_states.get(name, 'unavailable')}"
                    for name, _ in self._nav2_managed_state_clients
                )
                raise TimeoutError(
                    "simulation reset recovery timed out waiting for all Nav2 "
                    f"managed nodes to become active: {state_summary}"
                )

            services_ready = True
            for node_name, client in self._nav2_managed_state_clients:
                if not client.wait_for_service(timeout_sec=min(0.1, remaining)):
                    services_ready = False
                    latest_states[node_name] = "service_unavailable"
            if not services_ready:
                self._spin_once(min(0.1, max(0.0, deadline - time.monotonic())))
                continue

            futures = {
                node_name: client.call_async(GetState.Request())
                for node_name, client in self._nav2_managed_state_clients
            }
            query_deadline = min(deadline, time.monotonic() + 1.0)
            query_complete = True
            for future in futures.values():
                if not self._wait_future(future, query_deadline):
                    query_complete = False
            if query_complete:
                all_active = True
                for node_name, future in futures.items():
                    response = future.result()
                    if response is None:
                        latest_states[node_name] = "no_response"
                        all_active = False
                        continue
                    latest_states[node_name] = response.current_state.label
                    all_active = (
                        all_active
                        and response.current_state.id
                        == State.PRIMARY_STATE_ACTIVE
                    )
                if all_active:
                    self._collision_monitor_active = True
                    return
            self._spin_once(min(0.1, max(0.0, deadline - time.monotonic())))
        raise ExternalShutdownException()

    def _clear_navigation_costmaps(self) -> None:
        for label, client in self._costmap_clear_clients:
            if not client.wait_for_service(
                timeout_sec=self._service_timeout_sec
            ):
                self._raise_if_shutdown()
                raise RuntimeError(f"{label} clear service is unavailable")
            future = client.call_async(ClearEntireCostmap.Request())
            if not self._wait_future(
                future, time.monotonic() + self._service_timeout_sec
            ):
                raise TimeoutError(f"clearing {label} timed out")
            if future.result() is None:
                raise RuntimeError(f"clearing {label} returned no response")

    def _clear_localization_buffer(self) -> None:
        if not self._localization_buffer_client.wait_for_service(
            timeout_sec=self._service_timeout_sec
        ):
            self._raise_if_shutdown()
            raise RuntimeError(
                "SLAM Toolbox localization buffer service is unavailable"
            )
        future = self._localization_buffer_client.call_async(Empty.Request())
        if not self._wait_future(
            future, time.monotonic() + self._service_timeout_sec
        ):
            raise TimeoutError("clearing the localization buffer timed out")
        if future.result() is None:
            raise RuntimeError(
                "clearing the localization buffer returned no response"
            )

    def _wait_for_reset_recovery(
        self,
        tf_stamp_barrier_s: float | None,
        sample_stamp_barrier_s: float | None,
    ) -> None:
        deadline = time.monotonic() + self._reset_recovery_timeout_sec
        stable_since: float | None = None
        stable_anchor: tuple[float, float, float] | None = None
        while rclpy.ok():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise TimeoutError(
                    "simulation reset recovery timed out waiting for the "
                    "spawn-aligned Ground Truth/odom/map->base state and "
                    "stable map->odom"
                )
            self._spin_once(min(0.1, remaining))
            now = time.monotonic()
            odom_ready = bool(
                self._odom_samples
                and now - self._odom_samples[-1].received_at <= self._odom_max_age_sec
            )
            ground_truth_ready = bool(
                self._ground_truth_samples
                and now - self._ground_truth_samples[-1].received_at
                <= self._odom_max_age_sec
            )
            if odom_ready:
                odom = self._odom_samples[-1]
                odom_ready = (
                    (
                        sample_stamp_barrier_s is None
                        or odom.stamp_s > sample_stamp_barrier_s
                    )
                    and
                    math.hypot(odom.x, odom.y)
                    <= self._reset_tf_translation_tolerance_m
                    and abs(wrap_angle(odom.yaw_rad))
                    <= self._reset_tf_yaw_tolerance_rad
                )
            if ground_truth_ready:
                ground_truth = self._ground_truth_samples[-1]
                expected_x, expected_y = self._spawn_pose.map.position
                expected_yaw = math.radians(
                    self._spawn_pose.map.yaw_deg
                )
                ground_truth_ready = (
                    (
                        sample_stamp_barrier_s is None
                        or ground_truth.stamp_s > sample_stamp_barrier_s
                    )
                    and math.hypot(
                        ground_truth.x - expected_x,
                        ground_truth.y - expected_y,
                    )
                    <= self._reset_tf_translation_tolerance_m
                    and abs(wrap_angle(
                        ground_truth.yaw_rad - expected_yaw
                    )) <= self._reset_tf_yaw_tolerance_rad
                )
            fresh_transform = self._lookup_fresh_map_to_odom()
            tf_ready = False
            transform_values: tuple[float, float, float] | None = None
            if fresh_transform is not None:
                transform, stamp_s = fresh_transform
                rotation = transform.transform.rotation
                yaw = math.atan2(
                    2.0 * (
                        rotation.w * rotation.z
                        + rotation.x * rotation.y
                    ),
                    1.0
                    - 2.0 * (
                        rotation.y * rotation.y
                        + rotation.z * rotation.z
                    ),
                )
                translation = transform.transform.translation
                transform_values = (translation.x, translation.y, yaw)
                tf_ready = (
                    tf_stamp_barrier_s is None
                    or stamp_s > tf_stamp_barrier_s
                )
            map_base_ready = False
            try:
                map_base = self._tf_buffer.lookup_transform(
                    "map", "base_link", Time()
                )
            except TransformException:
                map_base = None
            if map_base is not None:
                map_base_stamp = self._transform_stamp_seconds(map_base)
                clock_s = self._clock_seconds()
                rotation = map_base.transform.rotation
                map_base_yaw = math.atan2(
                    2.0 * (
                        rotation.w * rotation.z
                        + rotation.x * rotation.y
                    ),
                    1.0
                    - 2.0 * (
                        rotation.y * rotation.y
                        + rotation.z * rotation.z
                    ),
                )
                translation = map_base.transform.translation
                expected_x, expected_y = self._spawn_pose.map.position
                expected_yaw = math.radians(
                    self._spawn_pose.map.yaw_deg
                )
                map_base_ready = (
                    clock_s is not None
                    and map_base_stamp > 0.0
                    and abs(clock_s - map_base_stamp)
                    <= self._tf_gap_tolerance_sec
                    and math.hypot(
                        translation.x - expected_x,
                        translation.y - expected_y,
                    ) <= self._reset_tf_translation_tolerance_m
                    and abs(wrap_angle(
                        map_base_yaw - expected_yaw
                    )) <= self._reset_tf_yaw_tolerance_rad
                )
            tf_ready = tf_ready and map_base_ready
            if odom_ready and ground_truth_ready and tf_ready:
                if stable_anchor is None:
                    stable_anchor = transform_values
                    stable_since = now
                else:
                    assert transform_values is not None
                    translation_delta = math.hypot(
                        transform_values[0] - stable_anchor[0],
                        transform_values[1] - stable_anchor[1],
                    )
                    yaw_delta = abs(
                        wrap_angle(transform_values[2] - stable_anchor[2])
                    )
                    if (
                        translation_delta
                        > self._reset_tf_translation_tolerance_m
                        or yaw_delta > self._reset_tf_yaw_tolerance_rad
                    ):
                        stable_anchor = transform_values
                        stable_since = now
                if now - stable_since >= self._reset_tf_stability_sec:
                    return
            else:
                stable_since = None
                stable_anchor = None
        raise ExternalShutdownException()

    def _reset_simulation(self, seed: int) -> None:
        previous_seed_epoch = self._localization_seed_epoch
        self._clear_localization_buffer()
        self._set_reset_seed(seed)
        if not self._reset_client.wait_for_service(timeout_sec=self._service_timeout_sec):
            self._raise_if_shutdown()
            raise RuntimeError(f"reset service unavailable: {self._reset_service_name}")
        future = self._reset_client.call_async(Trigger.Request())
        deadline = time.monotonic() + self._service_timeout_sec
        if not self._wait_future(future, deadline):
            raise TimeoutError("simulation reset timed out")
        response = future.result()
        if response is None or not response.success:
            message = "no response" if response is None else response.message
            raise RuntimeError(f"simulation reset failed: {message}")
        self._clear_navigation_costmaps()
        self._clear_run_state()
        if not self._wait_until(
            lambda: self._localization_seed_epoch > previous_seed_epoch,
            self._reset_recovery_timeout_sec,
        ):
            raise TimeoutError(
                "simulation reset recovery timed out waiting for a fresh-scan "
                "localization seed event"
            )
        # Snapshot after Isaac has published the initial-pose message.  A subsequent,
        # strictly newer map->odom publication is then required before the
        # pose/stability gate can pass.
        tf_stamp_barrier_s = self._latest_map_to_odom_stamp()
        sample_stamp_barrier_s = self._clock_seconds()
        self._wait_for_reset_recovery(
            tf_stamp_barrier_s,
            sample_stamp_barrier_s,
        )
        self._wait_for_nav2_managed_nodes_active()

    def _pose_message(self, specification) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = specification.frame_id
        if self._clock_stamp is not None:
            pose.header.stamp = self._clock_stamp
        pose.pose.position.x = specification.position[0]
        pose.pose.position.y = specification.position[1]
        yaw = math.radians(specification.yaw_deg)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _goal_message(self, specification=None) -> NavigateToPose.Goal:
        goal = NavigateToPose.Goal()
        goal.pose = self._pose_message(
            specification or self._scenario.goal
        )
        return goal

    def _navigation_feedback_callback(self, message) -> None:
        feedback = message.feedback
        self._route_feedback_count += 1
        self._maximum_route_recoveries = max(
            self._maximum_route_recoveries,
            int(feedback.number_of_recoveries),
        )

    def _navigate(self) -> tuple[bool, bool, int]:
        if not self._navigate_client.wait_for_server(
            timeout_sec=self._service_timeout_sec
        ):
            self._raise_if_shutdown()
            raise RuntimeError(
                f"Nav2 action unavailable: {self._action_name}"
            )
        specifications = (
            self._scenario.route
            if self._scenario.route
            else (self._scenario.goal,)
        )
        overall_deadline = time.monotonic() + self._scenario.timeout_sec
        self._navigation_active = True
        self._navigation_start_stamp_s = self._clock_seconds()
        try:
            for index, specification in enumerate(specifications):
                poses_remaining = len(specifications) - index
                self._minimum_poses_remaining = (
                    poses_remaining
                    if self._minimum_poses_remaining is None
                    else min(
                        self._minimum_poses_remaining,
                        poses_remaining,
                    )
                )
                send_future = self._navigate_client.send_goal_async(
                    self._goal_message(specification),
                    feedback_callback=self._navigation_feedback_callback,
                )
                send_deadline = min(
                    overall_deadline,
                    time.monotonic() + self._service_timeout_sec,
                )
                if not self._wait_future(send_future, send_deadline):
                    raise TimeoutError(
                        "Nav2 goal acknowledgement timed out"
                    )
                goal_handle = send_future.result()
                if goal_handle is None or not goal_handle.accepted:
                    return False, False, GoalStatus.STATUS_ABORTED

                result_future = goal_handle.get_result_async()
                if not self._wait_future(result_future, overall_deadline):
                    cancel_future = goal_handle.cancel_goal_async()
                    if not self._wait_future(
                        cancel_future,
                        time.monotonic() + self._service_timeout_sec,
                    ):
                        raise ExperimentIsolationError(
                            "Nav2 goal timed out and cancellation "
                            "acknowledgement was not received"
                        )
                    try:
                        cancel_response = cancel_future.result()
                    except Exception as exc:
                        raise ExperimentIsolationError(
                            f"Nav2 cancellation request failed: {exc}"
                        ) from exc
                    if cancel_response is None:
                        raise ExperimentIsolationError(
                            "Nav2 cancellation returned no response"
                        )
                    if not self._wait_future(
                        result_future,
                        time.monotonic() + self._service_timeout_sec,
                    ):
                        raise ExperimentIsolationError(
                            "Nav2 goal did not reach a terminal state "
                            "after cancellation"
                        )
                    wrapped_result = result_future.result()
                    if wrapped_result is None:
                        raise ExperimentIsolationError(
                            "Nav2 goal returned no terminal result "
                            "after cancellation"
                        )
                    return False, True, int(wrapped_result.status)
                wrapped_result = result_future.result()
                if wrapped_result is None:
                    return False, False, GoalStatus.STATUS_UNKNOWN
                if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
                    return False, False, int(wrapped_result.status)
                self._minimum_poses_remaining = len(specifications) - index - 1
            return True, False, GoalStatus.STATUS_SUCCEEDED
        finally:
            self._navigation_end_stamp_s = self._clock_seconds()
            self._navigation_active = False

    @staticmethod
    def _motion_quality_metrics(
        samples: list[CommandSample] | list[OdometrySample],
    ) -> dict[str, Any]:
        translated_distance = 0.0
        reverse_distance = 0.0
        curved_distance = 0.0
        moving_time = 0.0
        observed_time = 0.0
        maximum_linear_acceleration = 0.0
        maximum_angular_acceleration = 0.0
        angular_direction_changes = 0
        previous_turn_sign = 0
        for previous, current in zip(samples, samples[1:]):
            dt = current.stamp_s - previous.stamp_s
            if not 0.005 <= dt <= 0.25:
                continue
            linear = previous.linear_speed_mps
            angular = previous.angular_speed_radps
            distance = abs(linear) * dt
            translated_distance += distance
            reverse_distance += max(-linear, 0.0) * dt
            if abs(linear) >= 0.05 and abs(angular) >= 0.15:
                curved_distance += distance
            if abs(linear) >= 0.03 or abs(angular) >= 0.10:
                moving_time += dt
            observed_time += dt
            maximum_linear_acceleration = max(
                maximum_linear_acceleration,
                abs(current.linear_speed_mps - linear) / dt,
            )
            maximum_angular_acceleration = max(
                maximum_angular_acceleration,
                abs(current.angular_speed_radps - angular) / dt,
            )
            if abs(angular) >= 0.25:
                turn_sign = 1 if angular > 0.0 else -1
                if previous_turn_sign and turn_sign != previous_turn_sign:
                    angular_direction_changes += 1
                previous_turn_sign = turn_sign
        return {
            "sample_count": len(samples),
            "observed_duration_sec": observed_time,
            "translated_distance_m": translated_distance,
            "reverse_distance_m": reverse_distance,
            "reverse_distance_fraction": (
                reverse_distance / translated_distance
                if translated_distance > 1.0e-6
                else 0.0
            ),
            "curved_distance_m": curved_distance,
            "curved_distance_fraction": (
                curved_distance / translated_distance
                if translated_distance > 1.0e-6
                else 0.0
            ),
            "stopped_time_fraction": (
                1.0 - moving_time / observed_time
                if observed_time > 1.0e-6
                else 1.0
            ),
            "maximum_linear_acceleration_mps2": maximum_linear_acceleration,
            "maximum_angular_acceleration_radps2": maximum_angular_acceleration,
            "angular_direction_changes": angular_direction_changes,
        }

    def _wait_for_final_stillness(self) -> bool:
        settings = self._scenario.success
        deadline = time.monotonic() + settings.final_still_timeout_sec
        stationary_since: float | None = None
        while time.monotonic() < deadline:
            self._raise_if_shutdown()
            self._spin_once(0.05)
            now = time.monotonic()
            if not self._odom_samples:
                stationary_since = None
                continue
            sample = self._odom_samples[-1]
            fresh = now - sample.received_at <= self._odom_max_age_sec
            stationary = (
                abs(sample.linear_speed_mps) <= settings.final_linear_speed_mps
                and abs(sample.angular_speed_radps) <= settings.final_angular_speed_radps
            )
            if fresh and stationary:
                stationary_since = stationary_since or now
                if now - stationary_since >= settings.final_still_duration_sec:
                    return True
            else:
                stationary_since = None
        return False

    def _build_manifest(
        self,
        *,
        run_index: int,
        seed: int,
        nav2_succeeded: bool,
        timed_out: bool,
        nav2_status: int,
        final_still: bool,
        runner_error: str | None,
    ) -> dict[str, Any]:
        gt = self._ground_truth_samples[-1] if self._ground_truth_samples else None
        odom = self._odom_samples[-1] if self._odom_samples else None
        goal_x, goal_y = self._scenario.goal.position
        position_error = math.hypot(gt.x - goal_x, gt.y - goal_y) if gt else 0.0
        goal_yaw = math.radians(self._scenario.goal.yaw_deg)
        orientation_error = wrap_angle(gt.yaw_rad - goal_yaw) if gt else None
        safety_complete = (
            self._collision_seen
            and self._localization_seen
            and (
                self._lock_status_seen
                or self._collision_monitor_active
            )
        ) or not self._scenario.success.require_safety_observations
        thresholds = SingleRunThresholds(
            position_tolerance_m=self._scenario.success.position_tolerance_m,
            orientation_tolerance_rad=math.radians(
                self._scenario.success.orientation_tolerance_deg
            ),
            final_linear_speed_tolerance_mps=self._scenario.success.final_linear_speed_mps,
            final_angular_speed_tolerance_radps=self._scenario.success.final_angular_speed_radps,
        )
        observation = SingleRunObservation(
            nav2_succeeded=nav2_succeeded,
            ground_truth_available=gt is not None,
            ground_truth_position_error_m=position_error,
            ground_truth_orientation_error_rad=orientation_error,
            orientation_required=self._scenario.goal.require_orientation,
            collision_detected=self._collision_detected,
            localization_lost=self._localization_lost,
            tf_interrupted=self._tf_interrupted or not self._tf_ever_available,
            timed_out=timed_out,
            collision_monitor_locked=self._collision_monitor_locked,
            final_linear_speed_mps=odom.linear_speed_mps if odom else 0.0,
            final_angular_speed_radps=odom.angular_speed_radps if odom else 0.0,
            safety_observability_complete=safety_complete,
        )
        evaluation = evaluate_single_run(observation, thresholds)
        navigation_odom = [
            sample
            for sample in self._odom_samples
            if (
                self._navigation_start_stamp_s is not None
                and self._navigation_end_stamp_s is not None
                and self._navigation_start_stamp_s
                <= sample.stamp_s
                <= self._navigation_end_stamp_s
            )
        ]
        ground_truth_path_length = path_length(
            [(sample.x, sample.y) for sample in self._ground_truth_samples]
        )
        command_quality = self._motion_quality_metrics(
            self._command_samples
        )
        measured_quality = self._motion_quality_metrics(navigation_odom)
        quality_thresholds = self._scenario.success
        reasons = list(evaluation.failure_reasons)
        if odom is None:
            reasons.append("odom_unavailable")
        if not final_still:
            reasons.append("final_still_duration_not_met")
        if (
            ground_truth_path_length
            < quality_thresholds.minimum_ground_truth_path_length_m
        ):
            reasons.append("ground_truth_path_too_short")
        if (
            measured_quality["reverse_distance_m"]
            < quality_thresholds.minimum_reverse_distance_m
        ):
            reasons.append("insufficient_reverse_motion")
        if (
            measured_quality["reverse_distance_fraction"]
            > quality_thresholds.maximum_reverse_distance_fraction
        ):
            reasons.append("excessive_reverse_motion")
        if (
            measured_quality["curved_distance_fraction"]
            < quality_thresholds.minimum_curved_distance_fraction
        ):
            reasons.append("insufficient_curved_motion")
        if (
            measured_quality["stopped_time_fraction"]
            > quality_thresholds.maximum_stopped_time_fraction
        ):
            reasons.append("excessive_stopped_time")
        if runner_error:
            reasons.append(f"runner_error:{runner_error}")
        reasons = list(dict.fromkeys(reasons))
        result = "success" if not reasons else "failure"
        return {
            "scenario_id": self._scenario.scenario_id,
            "random_seed": seed,
            "map_version": self._scenario.map_version,
            "posegraph_version": self._scenario.posegraph_version,
            "robot_config_hash": self._robot_config_hash,
            "nav2_config_hash": self._nav2_config_hash,
            "dynamic_runtime_contract": dict(
                self._dynamic_runtime_contract
            ),
            "spawn_pose_name": self._spawn_pose.name,
            "usd_start_pose": self._spawn_pose.usd.as_dict(),
            "map_start_pose": self._spawn_pose.map.as_dict(),
            "goal_pose": self._scenario.goal.as_dict(),
            "route_poses": [
                specification.as_dict()
                for specification in self._scenario.route
            ],
            "obstacle_trajectories": list(self._scenario.obstacle_trajectories),
            "physics_dt": self._scenario.physics_dt,
            "rtf": self._scenario.rtf,
            "result": result,
            "failure_reason": ";".join(reasons),
            "run_index": run_index,
            "scenario_type": self._scenario.scenario_type,
            "recorded_at_utc": datetime.now(timezone.utc).isoformat(),
            "nav2_status": nav2_status,
            "motion_acceptance": {
                "minimum_ground_truth_path_length_m": (
                    quality_thresholds.minimum_ground_truth_path_length_m
                ),
                "minimum_reverse_distance_m": (
                    quality_thresholds.minimum_reverse_distance_m
                ),
                "maximum_reverse_distance_fraction": (
                    quality_thresholds.maximum_reverse_distance_fraction
                ),
                "minimum_curved_distance_fraction": (
                    quality_thresholds.minimum_curved_distance_fraction
                ),
                "maximum_stopped_time_fraction": (
                    quality_thresholds.maximum_stopped_time_fraction
                ),
            },
            "sample_counts": {
                "ground_truth": len(self._ground_truth_samples),
                "odom": len(self._odom_samples),
                "navigation_odom": len(navigation_odom),
                "navigation_commands": len(self._command_samples),
            },
            "metrics": {
                "ground_truth_position_error_m": position_error,
                "ground_truth_orientation_error_rad": orientation_error,
                "ground_truth_path_length_m": ground_truth_path_length,
                "odom_path_length_m": path_length(
                    [(sample.x, sample.y) for sample in self._odom_samples]
                ),
                "final_linear_speed_mps": odom.linear_speed_mps if odom else 0.0,
                "final_angular_speed_radps": odom.angular_speed_radps if odom else 0.0,
                "final_still_duration_met": final_still,
                "command_motion_quality": command_quality,
                "measured_motion_quality": measured_quality,
                "route_feedback_count": self._route_feedback_count,
                "minimum_poses_remaining": self._minimum_poses_remaining,
                "maximum_route_recoveries": self._maximum_route_recoveries,
            },
            "observability": {
                "collision_status_seen": self._collision_seen,
                "localization_status_seen": self._localization_seen,
                "collision_monitor_status_seen": (
                    self._lock_status_seen
                    or self._collision_monitor_active
                ),
                "collision_monitor_state_message_seen": (
                    self._lock_status_seen
                ),
                "collision_monitor_lifecycle_active": (
                    self._collision_monitor_active
                ),
                "map_to_odom_seen": self._tf_ever_available,
            },
        }

    def run_all(self) -> list[dict[str, Any]]:
        if not self._wait_until(lambda: self._clock_ready, self._clock_timeout_sec):
            raise TimeoutError("timed out waiting for a non-zero /clock")
        self._verify_dynamic_runtime_contract()
        self._verify_collision_monitor_active()
        manifests: list[dict[str, Any]] = []
        for run_index, seed in enumerate(self._scenario.seeds, start=1):
            nav2_succeeded = False
            timed_out = False
            nav2_status = GoalStatus.STATUS_UNKNOWN
            final_still = False
            runner_error: str | None = None
            isolation_error: ExperimentIsolationError | None = None
            try:
                self._reset_simulation(seed)
                nav2_succeeded, timed_out, nav2_status = self._navigate()
                final_still = self._wait_for_final_stillness()
            except Exception as exc:  # Preserve a manifest for every attempted run.
                if not rclpy.ok():
                    raise KeyboardInterrupt from exc
                if isinstance(exc, ExperimentIsolationError):
                    isolation_error = exc
                runner_error = f"{type(exc).__name__}:{exc}"
                self.get_logger().error(runner_error)
            manifest = self._build_manifest(
                run_index=run_index,
                seed=seed,
                nav2_succeeded=nav2_succeeded,
                timed_out=timed_out,
                nav2_status=nav2_status,
                final_still=final_still,
                runner_error=runner_error,
            )
            timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
            stem = f"{self._scenario.scenario_id}-run-{run_index:04d}-seed-{seed}-{timestamp}"
            write_run_report(manifest, self._output_directory, stem)
            manifests.append(manifest)
            self.get_logger().info(f"completed {stem}: {manifest['result']}")
            if isolation_error is not None:
                raise isolation_error
        return manifests


def main(args=None) -> None:
    rclpy.init(args=args)
    node: ExperimentRunner | None = None
    try:
        node = ExperimentRunner()
        node.run_all()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
