"""ROS-facing, deterministic simulation reset bridge.

The bridge is deliberately composed around an existing :class:`rclpy.node.Node`
instead of inheriting from it.  This keeps module import free of ROS/Isaac
side-effects and makes the reset callback straightforward to unit test.
"""

from __future__ import annotations

import math
from typing import Any, Callable

from isaac_sim.src.robot.reset import ResetManager, ResetRequest
from isaac_sim.src.robot.spawn_pose_manager import SpawnPoseManager


class ResetServiceError(RuntimeError):
    """Raised when the ROS reset bridge is configured incorrectly."""


class InitialPoseRepublisher:
    """Publish reset poses only after a post-reset laser scan arrives.

    SLAM Toolbox localizes an ``/initialpose`` request against its latest scan.
    Publishing from the synchronous reset callback can therefore pair the new
    robot pose with a scan captured before the teleport.  This scheduler gates
    publication on scan header stamps newer than the reset-time barrier.
    """

    def __init__(
        self,
        publish: Callable[[str], None],
        *,
        publish_count: int = 1,
    ) -> None:
        if not callable(publish):
            raise ResetServiceError("initial pose publish callback must be callable")
        if isinstance(publish_count, bool) or not isinstance(publish_count, int):
            raise ResetServiceError("initial pose publish_count must be an integer")
        if publish_count <= 0:
            raise ResetServiceError("initial pose publish_count must be positive")
        self._publish = publish
        self._publish_count = publish_count
        self._pose_name: str | None = None
        self._remaining = 0
        self._after_stamp_s: float | None = None
        self._last_scan_stamp_s: float | None = None

    @property
    def pending(self) -> bool:
        return self._remaining > 0

    def schedule(self, pose_name: str, *, after_stamp_s: float) -> None:
        if not isinstance(pose_name, str) or not pose_name:
            raise ResetServiceError("initial pose name must be non-empty")
        if not math.isfinite(after_stamp_s) or after_stamp_s < 0.0:
            raise ResetServiceError("initial pose scan barrier must be non-negative")
        self._pose_name = pose_name
        self._remaining = self._publish_count
        self._after_stamp_s = after_stamp_s
        self._last_scan_stamp_s = None

    def observe_scan(self, scan_stamp_s: float) -> None:
        if not math.isfinite(scan_stamp_s) or scan_stamp_s < 0.0:
            raise ResetServiceError("scan stamp must be finite and non-negative")
        if not self.pending:
            return
        assert self._after_stamp_s is not None
        if scan_stamp_s <= self._after_stamp_s:
            return
        if (
            self._last_scan_stamp_s is not None
            and scan_stamp_s <= self._last_scan_stamp_s
        ):
            return
        assert self._pose_name is not None
        self._publish(self._pose_name)
        self._last_scan_stamp_s = scan_stamp_s
        self._remaining -= 1
        if self._remaining == 0:
            self._pose_name = None
            self._after_stamp_s = None


class ResetServiceBridge:
    """Expose ``/simulation/reset`` and the ROS-side reset hooks.

    The service callback is expected to be dispatched by ``rclpy.spin_once``
    from the simulation loop.  Consequently, the transactional reset always
    runs on Kit's main thread rather than from a background executor.
    """

    def __init__(
        self,
        node: Any,
        spawn_manager: SpawnPoseManager,
        *,
        default_pose_name: str,
        navigation_mode: str,
        odometry_mode: str,
        simulation_time: Callable[[], float],
        service_name: str = "/simulation/reset",
    ) -> None:
        from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
        from nav2_msgs.srv import ClearEntireCostmap
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
            qos_profile_sensor_data,
        )
        from robot_localization.srv import SetPose
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import Empty as EmptyMessage
        from std_srvs.srv import Empty, Trigger

        if navigation_mode not in {"mapping", "localization"}:
            raise ResetServiceError("navigation_mode must be mapping or localization")
        if odometry_mode not in {"ideal", "realistic"}:
            raise ResetServiceError("odometry_mode must be ideal or realistic")
        if not default_pose_name:
            raise ResetServiceError("default_pose_name must be non-empty")
        if not callable(simulation_time):
            raise ResetServiceError("simulation_time must be callable")

        self.node = node
        self.spawn_manager = spawn_manager
        self._manager: ResetManager | None = None
        self._pending_futures: set[Any] = set()
        self._unavailable_warnings: set[str] = set()
        self._simulation_time = simulation_time

        self._PoseWithCovarianceStamped = PoseWithCovarianceStamped
        self._Twist = Twist
        self._EmptyMessage = EmptyMessage
        self._EmptyService = Empty
        self._SetPose = SetPose
        self._ClearEntireCostmap = ClearEntireCostmap

        self._declare_parameter("reset_seed", 0)
        self._declare_parameter("reset_pose_name", default_pose_name)
        self._declare_parameter("navigation_mode", navigation_mode)
        self._declare_parameter("odometry_mode", odometry_mode)

        reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        initial_pose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._cmd_vel_publisher = node.create_publisher(Twist, "/cmd_vel", reliable)
        self._reset_event_publisher = node.create_publisher(
            EmptyMessage, "/simulation/reset_event", reliable
        )
        self._localization_seeded_publisher = node.create_publisher(
            EmptyMessage, "/simulation/localization_seeded", reliable
        )
        self._initial_pose_publisher = node.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", initial_pose_qos
        )
        self._initial_pose_republisher = InitialPoseRepublisher(
            self._publish_map_initial_pose_once
        )
        self._scan_subscription = node.create_subscription(
            LaserScan,
            "/scan",
            self._scan_callback,
            qos_profile_sensor_data,
        )

        self._wheel_reset_client = node.create_client(Empty, "/wheel_odometry/reset")
        self._ekf_set_pose_client = node.create_client(SetPose, "/set_pose")
        self._costmap_clients = (
            (
                "global costmap",
                node.create_client(
                    ClearEntireCostmap,
                    "/global_costmap/clear_entirely_global_costmap",
                ),
            ),
            (
                "local costmap",
                node.create_client(
                    ClearEntireCostmap,
                    "/local_costmap/clear_entirely_local_costmap",
                ),
            ),
        )
        self._service = node.create_service(Trigger, service_name, self._reset_callback)

    def _declare_parameter(self, name: str, default: Any) -> None:
        if not self.node.has_parameter(name):
            self.node.declare_parameter(name, default)

    def bind(self, manager: ResetManager) -> None:
        if self._manager is not None:
            raise ResetServiceError("a ResetManager is already bound")
        self._manager = manager

    def _read_request(self) -> ResetRequest:
        seed = self.node.get_parameter("reset_seed").value
        pose_name = self.node.get_parameter("reset_pose_name").value
        navigation_mode = self.node.get_parameter("navigation_mode").value
        odometry_mode = self.node.get_parameter("odometry_mode").value
        if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
            raise ResetServiceError("reset_seed must be a non-negative integer")
        if not isinstance(pose_name, str) or not pose_name:
            raise ResetServiceError("reset_pose_name must be non-empty")
        if not isinstance(navigation_mode, str) or not isinstance(odometry_mode, str):
            raise ResetServiceError("navigation_mode and odometry_mode must be strings")
        return ResetRequest(pose_name, navigation_mode, odometry_mode, seed)

    def _reset_callback(self, request: Any, response: Any) -> Any:
        del request
        if self._manager is None:
            response.success = False
            response.message = "ResetManager is not bound"
            return response
        try:
            reset_request = self._read_request()
            self._manager.reset(reset_request)
        except Exception as exc:
            response.success = False
            response.message = f"{type(exc).__name__}: {exc}"
            self.node.get_logger().error(f"Simulation reset failed: {response.message}")
            return response
        response.success = True
        response.message = (
            f"reset complete: pose={reset_request.pose_name}, "
            f"seed={reset_request.random_seed}, odometry={reset_request.odometry_mode}"
        )
        self.node.get_logger().info(response.message)
        return response

    def send_zero_velocity(self) -> None:
        self._cmd_vel_publisher.publish(self._Twist())

    def _queue_service_call(self, client: Any, request: Any, label: str) -> bool:
        if not client.service_is_ready():
            if label not in self._unavailable_warnings:
                self.node.get_logger().warning(
                    f"{label} reset service is unavailable; continuing with reset event/recovery gate"
                )
                self._unavailable_warnings.add(label)
            return False
        try:
            future = client.call_async(request)
        except Exception as exc:
            self.node.get_logger().error(f"failed to queue {label} reset: {exc}")
            return False
        self._pending_futures.add(future)

        def completed(done: Any) -> None:
            self._pending_futures.discard(done)
            try:
                done.result()
            except Exception as exc:
                self.node.get_logger().error(f"{label} reset failed: {exc}")

        future.add_done_callback(completed)
        return True

    def reset_ros_odometry(self, odometry_mode: str) -> None:
        """Notify local estimators and reset realistic-mode filter state."""

        self._reset_event_publisher.publish(self._EmptyMessage())
        if odometry_mode == "ideal":
            return
        if odometry_mode != "realistic":
            raise ResetServiceError(f"unknown odometry mode {odometry_mode!r}")
        self._queue_service_call(
            self._wheel_reset_client,
            self._EmptyService.Request(),
            "wheel odometry",
        )
        request = self._SetPose.Request()
        request.pose.header.stamp = self.node.get_clock().now().to_msg()
        request.pose.header.frame_id = "odom"
        request.pose.pose.pose.orientation.w = 1.0
        request.pose.pose.covariance[0] = 0.05**2
        request.pose.pose.covariance[7] = 0.05**2
        request.pose.pose.covariance[35] = math.radians(5.0) ** 2
        self._queue_service_call(self._ekf_set_pose_client, request, "EKF")

    def clear_costmaps(self) -> None:
        for label, client in self._costmap_clients:
            self._queue_service_call(
                client,
                self._ClearEntireCostmap.Request(),
                label,
            )

    def publish_map_initial_pose(self, pose_name: str) -> None:
        """Schedule a calibrated pose after the first fresh laser scan."""

        # ResetManager invokes this hook synchronously while Kit is paused.
        # Deferring publication prevents SLAM Toolbox from matching the new
        # pose against a scan retained from before the teleport.
        barrier = float(self._simulation_time())
        self._initial_pose_republisher.schedule(
            pose_name,
            after_stamp_s=barrier,
        )

    def _scan_callback(self, message: Any) -> None:
        stamp = message.header.stamp
        stamp_s = stamp.sec + stamp.nanosec * 1.0e-9
        self._initial_pose_republisher.observe_scan(stamp_s)

    def _publish_map_initial_pose_once(self, pose_name: str) -> None:
        map_pose = self.spawn_manager.get_map_pose(
            pose_name, purpose="localization reset initial pose"
        )
        message = self._PoseWithCovarianceStamped()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.pose.position.x = map_pose.position[0]
        message.pose.pose.position.y = map_pose.position[1]
        yaw = math.radians(map_pose.yaw_deg)
        message.pose.pose.orientation.z = math.sin(yaw * 0.5)
        message.pose.pose.orientation.w = math.cos(yaw * 0.5)
        message.pose.covariance[0] = map_pose.position_stddev_m**2
        message.pose.covariance[7] = map_pose.position_stddev_m**2
        message.pose.covariance[35] = math.radians(map_pose.yaw_stddev_deg) ** 2
        self._initial_pose_publisher.publish(message)
        self._localization_seeded_publisher.publish(self._EmptyMessage())
