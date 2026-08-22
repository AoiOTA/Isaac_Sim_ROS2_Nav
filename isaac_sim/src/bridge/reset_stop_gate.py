"""Fail-closed final velocity gate for simulation reset epochs."""

from __future__ import annotations

import json
import math
import threading
import time
from dataclasses import dataclass, replace
from typing import Any


class ResetStopGateError(RuntimeError):
    pass


@dataclass
class ResetStopGateState:
    """Pure generation fence; commands are never cached across HOLD."""

    generation: int = 0
    held: bool = True
    eligible_generation: int | None = None

    def hold(self) -> int:
        self.generation += 1
        self.held = True
        self.eligible_generation = None
        return self.generation

    def mark_reset_complete(self, generation: int) -> None:
        if generation != self.generation or not self.held:
            raise ResetStopGateError(
                "reset completion generation mismatch: "
                f"requested={generation}, current={self.generation}, held={self.held}"
            )
        self.eligible_generation = generation

    def release(self, generation: int) -> None:
        if (
            not self.held
            or generation != self.generation
            or generation != self.eligible_generation
        ):
            raise ResetStopGateError(
                "reset stop release rejected: "
                f"requested={generation}, current={self.generation}, "
                f"eligible={self.eligible_generation}, held={self.held}"
            )
        self.held = False
        self.eligible_generation = None


class ResetStopGate:
    """Relay final ``/cmd_vel`` to Isaac's private ``/cmd_vel_sim`` topic.

    While held, only zero Twist messages are published.  Releasing clears the
    epoch fence but does not replay any pre-reset command; only a subsequently
    received Twist can move the articulation.
    """

    RELEASE_PARAMETER = "reset_stop_gate_release_generation"

    def __init__(
        self,
        node: Any,
        *,
        input_topic: str = "/cmd_vel",
        output_topic: str = "/cmd_vel_sim",
        zero_rate_hz: float = 20.0,
    ) -> None:
        from geometry_msgs.msg import Twist
        from rcl_interfaces.msg import SetParametersResult
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from std_msgs.msg import String

        if not input_topic or not output_topic or input_topic == output_topic:
            raise ResetStopGateError("reset stop gate topics must be distinct")
        if not math.isfinite(float(zero_rate_hz)) or float(zero_rate_hz) <= 0.0:
            raise ResetStopGateError("zero_rate_hz must be finite and positive")

        self.node = node
        self.state = ResetStopGateState()
        self._lock = threading.RLock()
        self._publish_lock = threading.RLock()
        self._heartbeat_stop = threading.Event()
        self._heartbeat_failure: str | None = None
        self._closed = False
        self._heartbeat_period_s = 1.0 / float(zero_rate_hz)
        self._Twist = Twist
        self._String = String
        self._SetParametersResult = SetParametersResult
        reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        # Isaac advances rclpy only once per render frame.  A deeper reader
        # cache can therefore replay CollisionMonitor commands that were
        # superseded by a terminal zero before this callback is serviced.
        # The final actuator gate needs the latest command, never a backlog.
        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = node.create_publisher(Twist, output_topic, reliable)
        self._status_publisher = node.create_publisher(
            String, "/simulation/reset_stop_gate/status", status_qos
        )
        self._subscription = node.create_subscription(
            Twist, input_topic, self._command_callback, command_qos
        )
        if not node.has_parameter(self.RELEASE_PARAMETER):
            node.declare_parameter(self.RELEASE_PARAMETER, -1)
        self._parameter_callback = self._set_parameters_callback
        node.add_on_set_parameters_callback(self._parameter_callback)
        self._heartbeat_thread = threading.Thread(
            target=self._zero_heartbeat,
            name="reset-stop-gate-zero-heartbeat",
            daemon=True,
        )
        with self._publish_lock:
            with self._lock:
                self.publish_zero()
                self._publish_status("initialized")
        self._heartbeat_thread.start()

    @property
    def generation(self) -> int:
        with self._lock:
            return self.state.generation

    def hold(self) -> int:
        with self._publish_lock:
            with self._lock:
                if self._closed:
                    raise ResetStopGateError("reset stop gate is closed")
                generation = self.state.hold()
                self.publish_zero()
                self._publish_status("hold")
                return generation

    def mark_reset_complete(self, generation: int) -> None:
        with self._publish_lock:
            with self._lock:
                if self._closed:
                    raise ResetStopGateError("reset stop gate is closed")
                staged = replace(self.state)
                staged.mark_reset_complete(generation)
                self.publish_zero()
                self._publish_status("reset_complete", state=staged)
                self.state.eligible_generation = staged.eligible_generation

    def release(self, generation: int, *, source: str) -> None:
        # The publication lock excludes the independent heartbeat through the
        # final zero/status/commit boundary.  Once this method returns, no
        # heartbeat zero can follow a fresh relayed command.
        with self._publish_lock:
            with self._lock:
                if self._closed:
                    raise ResetStopGateError("reset stop gate is closed")
                # Publish the prospective released status while command
                # handling is still locked and the live state is still HOLD.
                # A publisher failure therefore cannot leave the gate open.
                staged = replace(self.state)
                staged.release(generation)
                # Leave one explicit zero at the epoch boundary.  No cached
                # input exists, so movement requires a fresh command.
                self.publish_zero()
                self._publish_status(f"released:{source}", state=staged)
                self.state.held = staged.held
                self.state.eligible_generation = staged.eligible_generation

    def publish_zero(self) -> None:
        self._publisher.publish(self._Twist())

    def _command_callback(self, message: Any) -> None:
        with self._publish_lock:
            with self._lock:
                try:
                    if self._closed or self.state.held:
                        self.publish_zero()
                        return
                    self._publisher.publish(message)
                except Exception as error:
                    self._record_publish_failure_locked(
                        error, source="command"
                    )
                    raise

    def _record_publish_failure_locked(
        self, error: Exception, *, source: str
    ) -> None:
        """Keep HOLD and expose a best-effort status after publish failure."""

        self.state.held = True
        self.state.eligible_generation = None
        self._heartbeat_failure = (
            f"{source}:{type(error).__name__}: {error}"
        )
        try:
            self._publish_status(
                f"{source}_publish_error:{type(error).__name__}:{error}"
            )
        except Exception:
            # The retained in-process failure string remains inspectable even
            # if the DDS status publisher is the failing resource.
            pass

    def _zero_heartbeat(self) -> None:
        """Publish HOLD zeros from wall time, independent of ROS executor spin."""

        deadline = time.monotonic() + self._heartbeat_period_s
        while not self._heartbeat_stop.wait(
            max(0.0, deadline - time.monotonic())
        ):
            now = time.monotonic()
            with self._publish_lock:
                with self._lock:
                    if self._closed:
                        return
                    if self.state.held:
                        try:
                            self.publish_zero()
                        except Exception as error:
                            self._record_publish_failure_locked(
                                error, source="heartbeat"
                            )
            deadline = max(
                deadline + self._heartbeat_period_s,
                now + self._heartbeat_period_s,
            )

    def _set_parameters_callback(self, parameters: list[Any]) -> Any:
        requested = [
            parameter for parameter in parameters
            if parameter.name == self.RELEASE_PARAMETER
        ]
        if not requested:
            return self._SetParametersResult(successful=True)
        if len(requested) != 1:
            return self._SetParametersResult(
                successful=False, reason="duplicate reset stop release generation"
            )
        value = requested[0].value
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            return self._SetParametersResult(
                successful=False,
                reason="reset stop release generation must be non-negative int",
            )
        try:
            self.release(value, source="activation_gate")
        except ResetStopGateError as exc:
            return self._SetParametersResult(successful=False, reason=str(exc))
        return self._SetParametersResult(successful=True)

    def _publish_status(
        self, reason: str, *, state: ResetStopGateState | None = None
    ) -> None:
        status = self.state if state is None else state
        message = self._String()
        message.data = json.dumps(
            {
                "generation": status.generation,
                "held": status.held,
                "eligible_generation": status.eligible_generation,
                "reason": reason,
            },
            sort_keys=True,
        )
        self._status_publisher.publish(message)

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self.state.held = True
            self.state.eligible_generation = None
            self._closed = True
            self._heartbeat_stop.set()

        # Stop and join before destroying any ROS resource the daemon uses.
        self._heartbeat_thread.join(timeout=max(1.0, 4.0 * self._heartbeat_period_s))
        heartbeat_alive = self._heartbeat_thread.is_alive()
        with self._publish_lock:
            with self._lock:
                self.publish_zero()
                self._publish_status(
                    "closed" if not heartbeat_alive else "closed:heartbeat_join_timeout"
                )

        remove_callback = getattr(
            self.node, "remove_on_set_parameters_callback", None
        )
        if remove_callback is not None:
            remove_callback(self._parameter_callback)
        self.node.destroy_subscription(self._subscription)
        self.node.destroy_publisher(self._status_publisher)
        self.node.destroy_publisher(self._publisher)
        if heartbeat_alive:
            raise ResetStopGateError("zero heartbeat did not stop before close timeout")
