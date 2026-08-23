"""ROS-facing, deterministic simulation reset bridge.

The bridge is deliberately composed around an existing :class:`rclpy.node.Node`
instead of inheriting from it.  This keeps module import free of ROS/Isaac
side-effects and makes the reset callback straightforward to unit test.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
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
    stop_generation: int | None = None
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
    """Legacy scan-gated seed scheduler retained only for unit compatibility.

    V6-GRID does not construct this helper or publish a global localization
    seed from the reset bridge.
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
        default_reset_seed: int,
        simulation_time: Callable[[], float],
        reset_stop_gate: Any | None = None,
        external_recovery_release_required: bool = True,
        service_name: str = "/simulation/reset",
    ) -> None:
        from nav2_msgs.srv import ClearEntireCostmap
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.clock import Clock
        from rclpy.clock_type import ClockType
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from rclpy.task import Future
        from robot_localization.srv import SetPose
        from std_msgs.msg import Empty as EmptyMessage
        from std_srvs.srv import Empty, Trigger

        if navigation_mode not in {"mapping", "localization"}:
            raise ResetServiceError("navigation_mode must be mapping or localization")
        if odometry_mode not in {"ideal", "realistic"}:
            raise ResetServiceError("odometry_mode must be ideal or realistic")
        if not default_pose_name:
            raise ResetServiceError("default_pose_name must be non-empty")
        if (
            isinstance(default_reset_seed, bool)
            or not isinstance(default_reset_seed, int)
            or default_reset_seed < 0
        ):
            raise ResetServiceError("default_reset_seed must be a non-negative integer")
        if not callable(simulation_time):
            raise ResetServiceError("simulation_time must be callable")
        if not isinstance(external_recovery_release_required, bool):
            raise ResetServiceError(
                "external_recovery_release_required must be bool"
            )

        self.node = node
        self.spawn_manager = spawn_manager
        self._manager: ResetManager | None = None
        self._pending_futures: set[Any] = set()
        self._unavailable_warnings: set[str] = set()
        self._simulation_time = simulation_time
        self._configured_navigation_mode = navigation_mode
        self._configured_odometry_mode = odometry_mode
        self._default_pose_name = default_pose_name
        self._reset_stop_gate = reset_stop_gate
        self._external_recovery_release_required = (
            external_recovery_release_required
        )
        self._callback_group = ReentrantCallbackGroup()
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._Future = Future
        self._transaction_generation = 0
        self._active_transaction: _ResetTransaction | None = None
        self._closed = False
        self._EmptyMessage = EmptyMessage
        self._EmptyService = Empty
        self._SetPose = SetPose
        self._ClearEntireCostmap = ClearEntireCostmap

        self._declare_parameter("reset_seed", default_reset_seed)
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
        self._reset_event_publisher = node.create_publisher(
            EmptyMessage, "/simulation/reset_event", reliable
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
        case_id = self.node.get_parameter("dynamic_case_id").value
        variant_id = self.node.get_parameter("dynamic_variant_id").value
        if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
            raise ResetServiceError("reset_seed must be a non-negative integer")
        if not isinstance(pose_name, str) or not pose_name:
            raise ResetServiceError("reset_pose_name must be non-empty")
        if not isinstance(navigation_mode, str) or not isinstance(odometry_mode, str):
            raise ResetServiceError("navigation_mode and odometry_mode must be strings")
        if not isinstance(case_id, str) or not isinstance(variant_id, str):
            raise ResetServiceError("dynamic case and variant IDs must be strings")
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
        return ResetRequest(
            pose_name,
            navigation_mode,
            odometry_mode,
            seed,
            dynamic_case_id=case_id,
            dynamic_variant_id=variant_id,
        )

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
            reset_stop_gate = getattr(self, "_reset_stop_gate", None)
            if reset_stop_gate is not None:
                transaction.stop_generation = reset_stop_gate.hold()
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
        receipt = json.dumps(
            {
                "pose": reset_request.pose_name,
                "seed": reset_request.random_seed,
                "odometry": reset_request.odometry_mode,
                "generation": transaction.generation,
                "case_id": reset_request.dynamic_case_id,
                "variant_id": reset_request.dynamic_variant_id,
            },
            separators=(",", ":"),
            sort_keys=True,
        )
        if transaction.errors:
            response.message = (
                f"simulation reset transaction failed: {summary}; "
                f"reset_receipt={receipt}; errors={transaction.errors}"
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
                f"reset_receipt={receipt}; "
                "reset_event emitted after all queued ROS reset calls completed"
            )
            self.node.get_logger().info(response.message)
        return response

    def _finish_transaction(self, transaction: _ResetTransaction) -> None:
        if transaction.timeout_timer is not None:
            transaction.timeout_timer.cancel()
        try:
            if transaction.errors:
                return

            # This event is the recovery epoch boundary.  It must be emitted
            # only after every queued ROS reset future has resolved.
            self._reset_event_publisher.publish(self._EmptyMessage())
            # Keep the final command gate held until every safety-critical
            # finalization action above has succeeded.  Logging and the reset
            # service response are intentionally outside this boundary: they
            # describe the result but cannot make the reset safe to release.
            if (
                getattr(self, "_reset_stop_gate", None) is not None
                and transaction.stop_generation is not None
            ):
                self._reset_stop_gate.mark_reset_complete(
                    transaction.stop_generation
                )
                if not getattr(
                    self, "_external_recovery_release_required", True
                ):
                    # Mapping, teleop, and diagnostics have no ActivationGate.
                    # Release only this successful transaction's generation.
                    self._reset_stop_gate.release(
                        transaction.stop_generation,
                        source="reset_transaction_complete",
                    )
        finally:
            # Keep the transaction exclusive through generation completion and
            # optional same-generation release.  An early Trigger cannot begin
            # a newer HOLD between mark_reset_complete() and release().
            if self._active_transaction is transaction:
                self._active_transaction = None

    def send_zero_velocity(self) -> None:
        if getattr(self, "_reset_stop_gate", None) is None:
            raise ResetServiceError("reset stop gate is unavailable")
        self._reset_stop_gate.publish_zero()

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
        """Ignore the retired global-pose hook in V6-GRID reset mode."""

        del pose_name
