"""ROS 2 LaserScan fault bridge used only by safety verification runs."""

from __future__ import annotations

import copy
import json
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, String

from .scan_fault import ScanFaultCommandError, ScanFaultController


class ScanFaultBridge(Node):
    """Forward ``/scan`` to ``/scan_fault`` while applying explicit test faults."""

    def __init__(
        self,
        *,
        node_name: str = "scan_fault_bridge",
        parameter_overrides=None,
    ) -> None:
        super().__init__(node_name, parameter_overrides=parameter_overrides)
        input_topic = str(self.declare_parameter("input_topic", "/scan").value).strip()
        output_topic = str(
            self.declare_parameter("output_topic", "/scan_fault").value
        ).strip()
        control_topic = str(
            self.declare_parameter("control_topic", "/scan_fault/control").value
        ).strip()
        status_topic = str(
            self.declare_parameter("status_topic", "/scan_fault/status").value
        ).strip()
        reset_event_topic = str(
            self.declare_parameter(
                "reset_event_topic", "/simulation/reset_event"
            ).value
        ).strip()
        status_period_s = float(
            self.declare_parameter("status_period_s", 1.0).value
        )
        rollback_tolerance_s = float(
            self.declare_parameter("reset_rollback_tolerance_s", 1.0e-6).value
        )

        topics = {
            "input_topic": input_topic,
            "output_topic": output_topic,
            "control_topic": control_topic,
            "status_topic": status_topic,
            "reset_event_topic": reset_event_topic,
        }
        empty = [name for name, value in topics.items() if not value]
        if empty:
            raise ValueError(f"Topic parameter(s) must not be empty: {', '.join(empty)}")
        if input_topic == output_topic:
            raise ValueError("input_topic and output_topic must differ")
        if not math.isfinite(status_period_s) or status_period_s <= 0.0:
            raise ValueError("status_period_s must be finite and positive")
        if not math.isfinite(rollback_tolerance_s) or rollback_tolerance_s < 0.0:
            raise ValueError(
                "reset_rollback_tolerance_s must be finite and non-negative"
            )

        rollback_tolerance_ns = int(round(rollback_tolerance_s * 1_000_000_000))
        self._controller = ScanFaultController(
            rollback_tolerance_ns=rollback_tolerance_ns
        )

        reliable = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        transient = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._scan_publisher = self.create_publisher(
            LaserScan, output_topic, qos_profile_sensor_data
        )
        self._status_publisher = self.create_publisher(String, status_topic, transient)
        self._scan_subscription = self.create_subscription(
            LaserScan, input_topic, self._scan_callback, qos_profile_sensor_data
        )
        self._control_subscription = self.create_subscription(
            String, control_topic, self._control_callback, reliable
        )
        self._reset_subscription = self.create_subscription(
            Empty, reset_event_topic, self._reset_callback, reliable
        )
        self._status_timer = self.create_timer(status_period_s, self._status_timer_callback)

        self.get_logger().info(
            "Scan fault bridge ready: "
            f"{input_topic} -> {output_topic}; control={control_topic}; "
            f"status={status_topic}; reset={reset_event_topic}"
        )
        self._publish_status(event="startup", ok=True)

    @staticmethod
    def _monotonic_s() -> float:
        return time.monotonic()

    def _publish_status(self, *, event: str, ok: bool, error: str = "") -> None:
        now_s = self._monotonic_s()
        payload = {
            "node": self.get_name(),
            "event": event,
            "ok": ok,
            "error": error,
            "state": self._controller.status(now_s=now_s),
        }
        message = String()
        message.data = json.dumps(payload, sort_keys=True, separators=(",", ":"))
        self._status_publisher.publish(message)

    def _scan_callback(self, message: LaserScan) -> None:
        stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        decision = self._controller.process(
            stamp_ns=stamp_ns, now_s=self._monotonic_s()
        )
        if decision.forward:
            output = message
            if decision.frame_id_override is not None:
                output = copy.deepcopy(message)
                output.header.frame_id = decision.frame_id_override
            self._scan_publisher.publish(output)
        if decision.epoch_changed:
            self.get_logger().warning(
                f"Scan timestamp rollback opened epoch {decision.epoch}; active fault cleared"
            )
            self._publish_status(event="scan_stamp_rollback", ok=True)
        elif decision.state_changed:
            self._publish_status(event="fault_completed", ok=True)

    def _control_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ScanFaultCommandError("command must be a JSON object")
            self._controller.apply_command(payload, now_s=self._monotonic_s())
        except (json.JSONDecodeError, ScanFaultCommandError, ValueError) as exc:
            error = str(exc)
            self.get_logger().error(f"Rejected scan fault command: {error}")
            self._publish_status(event="command_rejected", ok=False, error=error)
            return
        self.get_logger().info(
            f"Applied scan fault command: {payload['command']} "
            f"(epoch={self._controller.epoch})"
        )
        self._publish_status(event="command_applied", ok=True)

    def _reset_callback(self, _message: Empty) -> None:
        epoch = self._controller.begin_new_epoch("reset_event")
        self.get_logger().warning(
            f"Simulation Reset opened scan fault epoch {epoch}; active fault cleared"
        )
        self._publish_status(event="reset_event", ok=True)

    def _status_timer_callback(self) -> None:
        self._publish_status(event="periodic", ok=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ScanFaultBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
