#!/usr/bin/env python3
"""V6 IMU regime Attempt4 session safety monitor.

Passive observer for the locked flat20/goal capture sessions.  It records
collision, command-chain, reset-gate, and clock events to JSONL and writes a
JSON summary on exit.  It publishes nothing and never interferes with the
command chain; a collision-true observation flips the summary to fail and
exits nonzero so the session driver can STOP.
"""

import argparse
import json
import math
import signal
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


def _stamp_now(node):
    return node.get_clock().now().nanoseconds * 1.0e-9


class Attempt4Monitor(Node):
    def __init__(self, output_path):
        super().__init__("attempt4_safety_monitor")
        self._output = open(output_path, "a", encoding="utf-8")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._collision_true = 0
        self._collision_false = 0
        self._clock_backward = 0
        self._last_clock = None
        self._gate_events = 0
        self._reset_events = 0
        self._last_commands = {}
        self._last_sim_s = None
        from geometry_msgs.msg import Twist
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import Bool
        from std_msgs.msg import Empty
        from std_msgs.msg import String

        self.create_subscription(Clock, "/clock", self._on_clock, qos)
        self.create_subscription(
            Bool, "/simulation/collision", self._on_collision, qos)
        self.create_subscription(
            Empty, "/simulation/reset_event", self._on_reset, qos)
        self.create_subscription(
            String, "/simulation/reset_stop_gate/status", self._on_gate, qos)
        for topic in (
            "/cmd_vel_nav",
            "/cmd_vel_smoothed",
            "/cmd_vel",
            "/cmd_vel_sim",
        ):
            self.create_subscription(
                Twist, topic, self._twist_cb(topic), qos)

    def _write(self, kind, **fields):
        record = {
            "kind": kind,
            "ros_time_s": _stamp_now(self),
            **fields,
        }
        self._output.write(json.dumps(record, sort_keys=True) + "\n")
        self._output.flush()

    def _on_clock(self, message):
        value = message.clock.sec + message.clock.nanosec * 1.0e-9
        if self._last_clock is not None and value < self._last_clock - 1.0e-9:
            self._clock_backward += 1
            self._write("clock_backward", sim_s=value)
        self._last_clock = value
        self._last_sim_s = value

    def _on_collision(self, message):
        if bool(message.data):
            self._collision_true += 1
            self._write("collision", data=True)
        else:
            self._collision_false += 1

    def _on_reset(self, _message):
        self._reset_events += 1
        self._write("reset_event", count=self._reset_events)

    def _on_gate(self, message):
        self._gate_events += 1
        self._write("gate_status", payload=message.data[:512])

    def _twist_cb(self, topic):
        def _cb(message):
            values = {
                "linear_x": float(message.linear.x),
                "linear_y": float(message.linear.y),
                "linear_z": float(message.linear.z),
                "angular_x": float(message.angular.x),
                "angular_y": float(message.angular.y),
                "angular_z": float(message.angular.z),
            }
            max_abs = max(abs(value) for value in values.values())
            self._last_commands[topic.lstrip("/")] = {
                "max_abs": max_abs,
                "payload": values,
                "ros_time_s": _stamp_now(self),
                "sim_s": self._last_sim_s,
            }
        return _cb

    def summary(self):
        final_zero = True
        for name, record in self._last_commands.items():
            if not math.isfinite(record["max_abs"]):
                final_zero = False
        last = self._last_commands.get("cmd_vel_sim")
        final_zero_pass = bool(
            last is not None and last["max_abs"] == 0.0
        )
        passed = (
            self._collision_true == 0
            and self._clock_backward == 0
            and final_zero
        )
        return {
            "schema_version": 1,
            "pass": passed,
            "collision_true_count": self._collision_true,
            "collision_false_count": self._collision_false,
            "sim_time_backward_count": self._clock_backward,
            "reset_event_count": self._reset_events,
            "gate_event_count": self._gate_events,
            "final_zero_pass": final_zero_pass,
            "last_commands": self._last_commands,
            "last_sim_s": self._last_sim_s,
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--summary", required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Attempt4Monitor(args.output)
    stopped = False

    def _stop(_signum, _frame):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    while rclpy.ok() and not stopped and node._collision_true == 0:
        rclpy.spin_once(node, timeout_sec=0.5)
    summary = node.summary()
    with open(args.summary, "w", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2, sort_keys=True)
        stream.write("\n")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if summary["pass"] else 3


if __name__ == "__main__":
    sys.exit(main())
