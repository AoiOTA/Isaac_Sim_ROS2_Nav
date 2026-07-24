"""ROS-facing, deterministic simulation reset bridge.

The bridge is deliberately composed around an existing :class:`rclpy.node.Node`
instead of inheriting from it.  This keeps module import free of ROS/Isaac
side-effects and makes the reset callback straightforward to unit test.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Callable

from isaac_sim.src.robot.reset import ResetManager, ResetRequest
from isaac_sim.src.robot.spawn_pose_manager import SpawnPoseManager


class ResetServiceError(RuntimeError):
    """Raised when the ROS reset bridge is configured incorrectly."""


@dataclass
class _ResetTransaction:
    """Track every asynchronous ROS reset hook as one finite transaction."""

    generation: int
    completion: Any
    on_finished: Callable[["_ResetTransaction"], None]
    calls: list[tuple[str, Any]] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)
    skipped: list[str] = field(default_factory=list)
    initial_pose_name: str | None = None
    sealed: bool = False
    finished: bool = False
    timed_out: bool = False
    timeout_timer: Any | None = None
    _pending: int = 0

    def add_call(self, label: str, future: Any) -> None:
        if self.sealed:
            raise ResetServiceError("cannot add a service call to a sealed reset")
        self.calls.append((label, future))
        self._pending += 1

        def completed(done: Any) -> None:
            if self.finished:
                return
            try:
                done.result()
            except Exception as exc:
                self.errors.append(f"{label}: {type(exc).__name__}: {exc}")
            self._pending -= 1
            self._finish_if_ready()

        future.add_done_callback(completed)

    def record_error(self, label: str, exc: Exception | str) -> None:
        detail = str(exc)
        if isinstance(exc, Exception):
            detail = f"{type(exc).__name__}: {exc}"
        self.errors.append(f"{label}: {detail}")

    def seal(self) -> None:
        self.sealed = True
        self._finish_if_ready()

    def timeout(self, timeout_sec: float) -> None:
        if self.finished:
            return
        pending = [label for label, future in self.calls if not future.done()]
        self.timed_out = True
        self.errors.append(
            f"transaction timed out after {timeout_sec:.3f}s"
            + (f"; pending={pending}" if pending else "")
        )
        self._finish(force=True)

    def _finish_if_ready(self) -> None:
        if self.sealed and self._pending == 0:
            self._finish()

    def _finish(self, *, force: bool = False) -> None:
        if self.finished or (not force and (not self.sealed or self._pending != 0)):
            return
        self.finished = True
        try:
            self.on_finished(self)
        except Exception as exc:
            self.errors.append(
                f"transaction finalization: {type(exc).__name__}: {exc}"
            )
        finally:
            if not self.completion.done():
                self.completion.set_result(self)


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
        clock_scan_tolerance_s: float = 0.5,
        clock_rollback_tolerance_s: float = 1.0e-6,
    ) -> None:
        if not callable(publish):
            raise ResetServiceError("initial pose publish callback must be callable")
        if isinstance(publish_count, bool) or not isinstance(publish_count, int):
            raise ResetServiceError("initial pose publish_count must be an integer")
        if publish_count <= 0:
            raise ResetServiceError("initial pose publish_count must be positive")
        if not math.isfinite(clock_scan_tolerance_s) or clock_scan_tolerance_s <= 0.0:
            raise ResetServiceError(
                "initial pose clock_scan_tolerance_s must be finite and positive"
            )
        if (
            not math.isfinite(clock_rollback_tolerance_s)
            or clock_rollback_tolerance_s < 0.0
        ):
            raise ResetServiceError(
                "initial pose clock_rollback_tolerance_s must be finite and non-negative"
            )
        self._publish = publish
        self._publish_count = publish_count
        self._clock_scan_tolerance_s = clock_scan_tolerance_s
        self._clock_rollback_tolerance_s = clock_rollback_tolerance_s
        self._pose_name: str | None = None
        self._remaining = 0
        self._after_stamp_s: float | None = None
        self._clock_stamp_s: float | None = None
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
        self._clock_stamp_s = after_stamp_s
        self._last_scan_stamp_s = None

    def cancel(self) -> None:
        self._pose_name = None
        self._remaining = 0
        self._after_stamp_s = None
        self._clock_stamp_s = None
        self._last_scan_stamp_s = None

    def observe_clock(self, clock_stamp_s: float) -> None:
        """Observe authoritative simulation time and detect a new clock epoch."""

        if not math.isfinite(clock_stamp_s) or clock_stamp_s < 0.0:
            raise ResetServiceError("simulation clock stamp must be finite and non-negative")
        if not self.pending:
            return
        assert self._after_stamp_s is not None
        if clock_stamp_s + self._clock_rollback_tolerance_s < self._after_stamp_s:
            self._after_stamp_s = clock_stamp_s
            self._last_scan_stamp_s = None
        self._clock_stamp_s = clock_stamp_s

    def observe_scan(
        self,
        scan_stamp_s: float,
        *,
        clock_stamp_s: float | None = None,
    ) -> None:
        if not math.isfinite(scan_stamp_s) or scan_stamp_s < 0.0:
            raise ResetServiceError("scan stamp must be finite and non-negative")
        if not self.pending:
            return
        if clock_stamp_s is not None:
            self.observe_clock(clock_stamp_s)
        assert self._after_stamp_s is not None
        assert self._clock_stamp_s is not None
        if scan_stamp_s <= self._after_stamp_s:
            return
        if abs(scan_stamp_s - self._clock_stamp_s) > self._clock_scan_tolerance_s:
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
            self.cancel()


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
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.clock import Clock
        from rclpy.clock_type import ClockType
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
            qos_profile_sensor_data,
        )
        from rclpy.task import Future
        from robot_localization.srv import SetPose
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import Empty as EmptyMessage, String
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
        self._configured_navigation_mode = navigation_mode
        self._configured_odometry_mode = odometry_mode
        self._default_pose_name = default_pose_name
        self._callback_group = ReentrantCallbackGroup()
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._Future = Future
        self._transaction_generation = 0
        self._active_transaction: _ResetTransaction | None = None
        self._closed = False
        self._initial_pose_source: str | None = None
        self._deferred_initial_pose_name: str | None = None

        self._PoseWithCovarianceStamped = PoseWithCovarianceStamped
        self._Twist = Twist
        self._EmptyMessage = EmptyMessage
        self._EmptyService = Empty
        self._SetPose = SetPose
        self._ClearEntireCostmap = ClearEntireCostmap
        self._String = String

        self._declare_parameter("reset_seed", 0)
        # Selection is intentionally reset-scoped: a campaign can choose one
        # physical case/variant without mutating the immutable simulation YAML.
        self._declare_parameter("dynamic_case_id", "")
        self._declare_parameter("dynamic_variant_id", "")
        self._declare_parameter("reset_pose_name", default_pose_name)
        self._declare_parameter("navigation_mode", navigation_mode)
        self._declare_parameter("odometry_mode", odometry_mode)
        self._declare_parameter("reset_transaction_timeout_sec", 10.0)
        timeout = self.node.get_parameter("reset_transaction_timeout_sec").value
        if (
            isinstance(timeout, bool)
            or not isinstance(timeout, (int, float))
            or not math.isfinite(float(timeout))
            or float(timeout) <= 0.0
        ):
            raise ResetServiceError(
                "reset_transaction_timeout_sec must be finite and positive"
            )
        self._transaction_timeout_sec = float(timeout)

        reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        initial_pose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        policy_qos = QoSProfile(
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
            callback_group=self._callback_group,
        )
        self._initial_pose_source_subscription = node.create_subscription(
            String,
            "/simulation/initial_pose_source",
            self._initial_pose_source_callback,
            policy_qos,
            callback_group=self._callback_group,
        )

        self._wheel_reset_client = node.create_client(
            Empty,
            "/wheel_odometry/reset",
            callback_group=self._callback_group,
        )
        self._ekf_set_pose_client = node.create_client(
            SetPose,
            "/set_pose",
            callback_group=self._callback_group,
        )
        self._costmap_clients = (
            (
                "global costmap",
                node.create_client(
                    ClearEntireCostmap,
                    "/global_costmap/clear_entirely_global_costmap",
                    callback_group=self._callback_group,
                ),
            ),
            (
                "local costmap",
                node.create_client(
                    ClearEntireCostmap,
                    "/local_costmap/clear_entirely_local_costmap",
                    callback_group=self._callback_group,
                ),
            ),
        )
        self._service = node.create_service(
            Trigger,
            service_name,
            self._reset_callback,
            callback_group=self._callback_group,
        )

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
        if navigation_mode != self._configured_navigation_mode:
            raise ResetServiceError(
                "navigation_mode is immutable at runtime: "
                f"configured={self._configured_navigation_mode!r}, "
                f"requested={navigation_mode!r}"
            )
        if odometry_mode != self._configured_odometry_mode:
            raise ResetServiceError(
                "odometry_mode is immutable at runtime: "
                f"configured={self._configured_odometry_mode!r}, "
                f"requested={odometry_mode!r}"
            )
        if (
            self._configured_navigation_mode == "localization"
            and pose_name != self._default_pose_name
        ):
            raise ResetServiceError(
                "reset_pose_name is bound to the map manifest at startup and "
                "is immutable in localization mode: "
                f"configured={self._default_pose_name!r}, "
                f"requested={pose_name!r}"
            )
        return ResetRequest(pose_name, navigation_mode, odometry_mode, seed)

    def start_reset(self, reset_request: ResetRequest) -> _ResetTransaction:
        """Start one reset without blocking Kit's main thread.

        The returned transaction is completed by normal executor progress.  It
        is also used for the startup reset, where no service callback exists to
        await the ROS client futures.
        """

        if getattr(self, "_closed", False):
            raise ResetServiceError("ResetServiceBridge is closed")
        if self._manager is None:
            raise ResetServiceError("ResetManager is not bound")
        if self._active_transaction is not None:
            raise ResetServiceError(
                "another reset transaction is still active: "
                f"generation={self._active_transaction.generation}"
            )
        if self._pending_futures:
            raise ResetServiceError(
                "previous timed-out reset calls are still resolving; "
                f"pending={len(self._pending_futures)}"
            )
        self._initial_pose_republisher.cancel()
        self._deferred_initial_pose_name = None
        self._transaction_generation += 1
        completion = self._Future(executor=self.node.executor)
        transaction = _ResetTransaction(
            generation=self._transaction_generation,
            completion=completion,
            on_finished=self._finish_transaction,
        )
        self._active_transaction = transaction
        transaction.timeout_timer = self.node.create_timer(
            self._transaction_timeout_sec,
            lambda: transaction.timeout(self._transaction_timeout_sec),
            callback_group=self._callback_group,
            clock=self._steady_clock,
        )
        try:
            self._manager.reset(reset_request)
        except Exception as exc:
            transaction.record_error("simulation reset", exc)
            transaction.seal()
        else:
            transaction.seal()
        return transaction

    def close(self) -> None:
        """Cancel reset work before the owning ROS node/context is destroyed."""

        if self._closed:
            return
        self._closed = True
        self._initial_pose_republisher.cancel()
        self._deferred_initial_pose_name = None

        transaction = self._active_transaction
        if transaction is not None and not transaction.finished:
            transaction.record_error(
                "shutdown", "reset cancelled during process shutdown"
            )
            for _, future in transaction.calls:
                if future.done():
                    continue
                cancel = getattr(future, "cancel", None)
                if callable(cancel):
                    cancel()
            transaction._finish(force=True)

        for future in tuple(self._pending_futures):
            cancel = getattr(future, "cancel", None)
            if callable(cancel) and not future.done():
                cancel()
        self._pending_futures.clear()

    async def _reset_callback(self, request: Any, response: Any) -> Any:
        del request
        if self._manager is None:
            response.success = False
            response.message = "ResetManager is not bound"
            return response
        try:
            reset_request = self._read_request()
            transaction = self.start_reset(reset_request)
        except Exception as exc:
            response.success = False
            response.message = f"{type(exc).__name__}: {exc}"
            self.node.get_logger().error(f"Simulation reset failed: {response.message}")
            return response
        await transaction.completion
        response.success = not transaction.errors
        summary = (
            f"pose={reset_request.pose_name}, seed={reset_request.random_seed}, "
            f"odometry={reset_request.odometry_mode}, "
            f"generation={transaction.generation}"
        )
        if transaction.errors:
            response.message = (
                f"simulation reset transaction failed: {summary}; "
                f"errors={transaction.errors}"
            )
            self.node.get_logger().error(response.message)
        else:
            skipped = (
                f"; unavailable optional services={transaction.skipped}"
                if transaction.skipped
                else ""
            )
            response.message = (
                f"simulation reset transaction complete: {summary}{skipped}; "
                "reset_event emitted after all queued ROS reset calls completed"
            )
            self.node.get_logger().info(response.message)
        return response

    def _finish_transaction(self, transaction: _ResetTransaction) -> None:
        if transaction.timeout_timer is not None:
            transaction.timeout_timer.cancel()
        if self._active_transaction is transaction:
            self._active_transaction = None

        if transaction.errors:
            # A failed/expired transaction is not a valid recovery epoch.  In
            # particular, do not arm an initial pose while stale reset calls
            # may still mutate wheel/EKF/costmap state.
            self._initial_pose_republisher.cancel()
            self._deferred_initial_pose_name = None
            return

        # This event is the recovery epoch boundary.  It must be emitted only
        # after every queued wheel/EKF/costmap reset future has resolved.
        self._reset_event_publisher.publish(self._EmptyMessage())
        if transaction.initial_pose_name is not None:
            self._deferred_initial_pose_name = transaction.initial_pose_name
            self._apply_initial_pose_policy()

    def send_zero_velocity(self) -> None:
        self._cmd_vel_publisher.publish(self._Twist())

    def _queue_service_call(self, client: Any, request: Any, label: str) -> bool:
        transaction = self._active_transaction
        if transaction is None:
            raise ResetServiceError(
                f"{label} reset requested outside ResetServiceBridge.start_reset()"
            )
        if not client.service_is_ready():
            if label not in self._unavailable_warnings:
                self.node.get_logger().warning(
                    f"{label} reset service is unavailable; continuing with reset event/recovery gate"
                )
                self._unavailable_warnings.add(label)
            transaction.skipped.append(label)
            return False
        try:
            future = client.call_async(request)
        except Exception as exc:
            self.node.get_logger().error(f"failed to queue {label} reset: {exc}")
            transaction.record_error(f"queue {label}", exc)
            return False
        self._pending_futures.add(future)

        def completed(done: Any) -> None:
            self._pending_futures.discard(done)

        future.add_done_callback(completed)
        transaction.add_call(label, future)
        return True

    def reset_ros_odometry(self, odometry_mode: str) -> None:
        """Notify local estimators and reset realistic-mode filter state."""

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
        """Defer a calibrated pose until the reset transaction is complete."""

        transaction = self._active_transaction
        if transaction is None:
            raise ResetServiceError(
                "initial pose requested outside ResetServiceBridge.start_reset()"
            )
        transaction.initial_pose_name = pose_name

    def _initial_pose_source_callback(self, message: Any) -> None:
        source = str(message.data).strip().lower()
        if source not in {"auto", "rviz"}:
            self.node.get_logger().error(
                "ignoring invalid /simulation/initial_pose_source value "
                f"{message.data!r}; expected auto or rviz"
            )
            return
        changed = source != self._initial_pose_source
        self._initial_pose_source = source
        if changed:
            self.node.get_logger().info(
                f"initial pose source policy received: {source}"
            )
        self._apply_initial_pose_policy()

    def _apply_initial_pose_policy(self) -> None:
        if self._initial_pose_source == "rviz":
            self._initial_pose_republisher.cancel()
            self._deferred_initial_pose_name = None
            return
        if (
            self._initial_pose_source != "auto"
            or self._deferred_initial_pose_name is None
        ):
            return
        pose_name = self._deferred_initial_pose_name
        self._deferred_initial_pose_name = None
        barrier = float(self._simulation_time())
        self._initial_pose_republisher.schedule(
            pose_name,
            after_stamp_s=barrier,
        )

    def _scan_callback(self, message: Any) -> None:
        stamp = message.header.stamp
        stamp_s = stamp.sec + stamp.nanosec * 1.0e-9
        clock_stamp_s = float(self._simulation_time())
        self._initial_pose_republisher.observe_scan(
            stamp_s,
            clock_stamp_s=clock_stamp_s,
        )

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
