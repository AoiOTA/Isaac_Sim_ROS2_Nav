#!/usr/bin/env python3
"""Bounded pre-goal health gate for the optional cuVSLAM shadow."""

import argparse
import json
import math
from pathlib import Path
import time

from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


def odometry_is_fresh_and_finite(message):
    """Accept a newly received stamped odometry sample with finite fields."""
    stamp_ns = (
        int(message.header.stamp.sec) * 1_000_000_000
        + int(message.header.stamp.nanosec)
    )
    values = [
        message.pose.pose.position.x,
        message.pose.pose.position.y,
        message.pose.pose.position.z,
        message.pose.pose.orientation.x,
        message.pose.pose.orientation.y,
        message.pose.pose.orientation.z,
        message.pose.pose.orientation.w,
        message.twist.twist.linear.x,
        message.twist.twist.linear.y,
        message.twist.twist.linear.z,
        message.twist.twist.angular.x,
        message.twist.twist.angular.y,
        message.twist.twist.angular.z,
        *message.pose.covariance,
        *message.twist.covariance,
    ]
    return stamp_ns > 0 and all(math.isfinite(value) for value in values)


def status_classification(message):
    """Classify the installed official status as healthy, pending, or fatal."""
    timing = [
        message.node_callback_execution_time,
        message.track_execution_time,
        message.track_execution_time_mean,
        message.track_execution_time_max,
    ]
    timing_is_valid = all(
        math.isfinite(value) and value >= 0.0 for value in timing)
    if message.vo_state == 2:
        return 'fatal'
    if message.vo_state == 1 and timing_is_valid:
        return 'healthy'
    return 'pending'


def gate_exit_code(*, valid_odometry, healthy_status, fatal_status, timed_out):
    """Return the bounded gate verdict without hiding a fatal status."""
    if fatal_status:
        return 2
    if valid_odometry and healthy_status:
        return 0
    if timed_out:
        return 2
    return None


class VisualShadowGate(Node):
    """Observe only the two official shadow outputs needed by the gate."""

    def __init__(self):
        super().__init__('v6_visual_shadow_pregoal_gate')
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.valid_odometry = False
        self.healthy_status = False
        self.fatal_status = False
        self.odometry_samples = 0
        self.status_samples = 0
        self.create_subscription(
            Odometry, '/visual/odom_shadow', self._odometry_callback, qos)
        self.create_subscription(
            VisualSlamStatus, '/visual/status', self._status_callback, qos)

    def _odometry_callback(self, message):
        self.odometry_samples += 1
        self.valid_odometry = (
            self.valid_odometry or odometry_is_fresh_and_finite(message))

    def _status_callback(self, message):
        self.status_samples += 1
        classification = status_classification(message)
        self.healthy_status = self.healthy_status or classification == 'healthy'
        self.fatal_status = self.fatal_status or classification == 'fatal'


def _parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout-sec', type=float, default=45.0)
    parser.add_argument('--output-json', type=Path, required=True)
    return parser.parse_args()


def main():
    """Wait for a fresh finite odometry sample and healthy official status."""
    args = _parse_args()
    if not math.isfinite(args.timeout_sec) or args.timeout_sec <= 0.0:
        raise SystemExit('--timeout-sec must be finite and positive')
    rclpy.init()
    node = VisualShadowGate()
    deadline = time.monotonic() + args.timeout_sec
    exit_code = None
    while exit_code is None:
        rclpy.spin_once(node, timeout_sec=0.1)
        exit_code = gate_exit_code(
            valid_odometry=node.valid_odometry,
            healthy_status=node.healthy_status,
            fatal_status=node.fatal_status,
            timed_out=time.monotonic() >= deadline,
        )
    result = {
        'fatal_status': node.fatal_status,
        'healthy_status': node.healthy_status,
        'odometry_samples': node.odometry_samples,
        'pass': exit_code == 0,
        'status_samples': node.status_samples,
        'timeout_sec': args.timeout_sec,
        'valid_odometry': node.valid_odometry,
    }
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(
        json.dumps(result, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
