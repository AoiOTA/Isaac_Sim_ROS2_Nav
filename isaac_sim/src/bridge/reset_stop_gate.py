"""Fail-closed final velocity gate for simulation reset epochs."""

from __future__ import annotations

from dataclasses import dataclass, replace
import json
import math
import threading
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
        from rclpy.clock import Clock
        from rclpy.clock_type import ClockType
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
        self._Twist = Twist
        self._String = String
        self._SetParametersResult = SetParametersResult
        reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
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
            Twist, input_topic, self._command_callback, reliable
        )
        if not node.has_parameter(self.RELEASE_PARAMETER):
            node.declare_parameter(self.RELEASE_PARAMETER, -1)
        self._parameter_callback = node.add_on_set_parameters_callback(
            self._set_parameters_callback
        )
        steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = node.create_timer(
            1.0 / float(zero_rate_hz), self._zero_timer_callback, clock=steady_clock
        )
        self.publish_zero()
        self._publish_status("initialized")

    @property
    def generation(self) -> int:
        with self._lock:
            return self.state.generation

    def hold(self) -> int:
        with self._lock:
            generation = self.state.hold()
            self.publish_zero()
            self._publish_status("hold")
            return generation

    def mark_reset_complete(self, generation: int) -> None:
        with self._lock:
            staged = replace(self.state)
            staged.mark_reset_complete(generation)
            self.publish_zero()
            self._publish_status("reset_complete", state=staged)
            self.state.eligible_generation = staged.eligible_generation

    def release(self, generation: int, *, source: str) -> None:
        with self._lock:
            # Publish the prospective released status while command handling
            # is still locked and the live state is still HOLD.  A publisher
            # failure therefore cannot leave the articulation gate open.
            staged = replace(self.state)
            staged.release(generation)
            # Leave one explicit zero at the epoch boundary.  No cached input
            # exists, so movement requires a fresh post-release command.
            self.publish_zero()
            self._publish_status(f"released:{source}", state=staged)
            self.state.held = staged.held
            self.state.eligible_generation = staged.eligible_generation

    def publish_zero(self) -> None:
        self._publisher.publish(self._Twist())

    def _command_callback(self, message: Any) -> None:
        with self._lock:
            if self.state.held:
                self.publish_zero()
                return
            self._publisher.publish(message)

    def _zero_timer_callback(self) -> None:
        with self._lock:
            if self.state.held:
                self.publish_zero()

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
            self.state.held = True
            self.state.eligible_generation = None
            self.publish_zero()
            self._publish_status("closed")
