#!/usr/bin/env python3
"""Publish deterministic room scans and summarize RF2O topic behavior."""

import argparse
import json
import math
import sys
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


def planar_motion_detected(messages, threshold=1.0e-3):
    """Detect planar motion relative to the first sample or from twist."""
    if len(messages) < 2:
        return False
    first = messages[0]
    first_yaw = 2.0 * math.atan2(
        first.pose.pose.orientation.z, first.pose.pose.orientation.w)
    for message in messages[1:]:
        displacement = math.hypot(
            message.pose.pose.position.x - first.pose.pose.position.x,
            message.pose.pose.position.y - first.pose.pose.position.y,
        )
        yaw = 2.0 * math.atan2(
            message.pose.pose.orientation.z,
            message.pose.pose.orientation.w,
        )
        yaw_delta = math.atan2(
            math.sin(yaw - first_yaw), math.cos(yaw - first_yaw))
        if displacement > threshold or abs(yaw_delta) > threshold:
            return True
        if (abs(message.twist.twist.linear.x) > threshold
                or abs(message.twist.twist.angular.z) > threshold):
            return True
    return False


class Rf2oSyntheticSmoke(Node):
    """Drive RF2O with finite translated scans and record its outputs."""

    def __init__(self, stationary=False):
        super().__init__('rf2o_synthetic_smoke')
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.create_subscription(Odometry, '/lidar/odom', self._odom, 10)
        self.create_subscription(TFMessage, '/tf', self._tf, 10)
        self.odometry = []
        self.tf_count = 0
        self.stationary = stationary

    def _odom(self, message):
        self.odometry.append(message)

    def _tf(self, message):
        self.tf_count += len(message.transforms)

    @staticmethod
    def _room_range(robot_x, robot_y, angle):
        direction_x = math.cos(angle)
        direction_y = math.sin(angle)
        candidates = []
        if abs(direction_x) > 1.0e-9:
            for wall_x in (-4.0, 4.0):
                distance = (wall_x - robot_x) / direction_x
                wall_y = robot_y + distance * direction_y
                if distance > 0.0 and -3.0 <= wall_y <= 3.0:
                    candidates.append(distance)
        if abs(direction_y) > 1.0e-9:
            for wall_y in (-3.0, 3.0):
                distance = (wall_y - robot_y) / direction_y
                wall_x = robot_x + distance * direction_x
                if distance > 0.0 and -4.0 <= wall_x <= 4.0:
                    candidates.append(distance)
        return min(candidates)

    def publish_scan(self, index):
        stamp_ns = 1_000_000_000 + index * 50_000_000
        clock = Clock()
        clock.clock.sec = stamp_ns // 1_000_000_000
        clock.clock.nanosec = stamp_ns % 1_000_000_000
        self.clock_pub.publish(clock)

        scan = LaserScan()
        scan.header.stamp = clock.clock
        scan.header.frame_id = 'laser'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2.0 * math.pi / 719.0
        scan.time_increment = 0.05 / 719.0
        scan.scan_time = 0.05
        scan.range_min = 0.05
        scan.range_max = 20.0
        robot_x = 0.0 if self.stationary else 0.008 * index
        robot_y = (
            0.0 if self.stationary else 0.10 * math.sin(index * 0.04))
        scan.ranges = [
            self._room_range(
                robot_x,
                robot_y,
                scan.angle_min + beam * scan.angle_increment,
            )
            for beam in range(720)
        ]
        self.scan_pub.publish(scan)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--stationary', action='store_true',
        help='repeat an unchanged room scan and require no detected motion')
    args = parser.parse_args()
    rclpy.init()
    node = Rf2oSyntheticSmoke(stationary=args.stationary)
    try:
        for index in range(100):
            node.publish_scan(index)
            deadline = time.monotonic() + 0.05
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.005)
        for _ in range(100):
            rclpy.spin_once(node, timeout_sec=0.01)

        stamps = [
            message.header.stamp.sec * 1_000_000_000
            + message.header.stamp.nanosec
            for message in node.odometry
        ]
        values = [
            value
            for message in node.odometry
            for value in (
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                message.pose.pose.orientation.z,
                message.pose.pose.orientation.w,
                message.twist.twist.linear.x,
                message.twist.twist.angular.z,
            )
        ]
        finite = bool(values) and all(math.isfinite(value) for value in values)
        motion_detected = planar_motion_detected(node.odometry)
        monotonic = all(left < right for left, right in zip(stamps, stamps[1:]))
        covariance_nonzero = bool(node.odometry) and all(
            all(message.pose.covariance[index * 6 + index] > 0.0
                for index in range(6))
            and all(message.twist.covariance[index * 6 + index] > 0.0
                    for index in range(6))
            for message in node.odometry
        )
        frames_valid = bool(node.odometry) and all(
            message.header.frame_id == 'odom'
            and message.child_frame_id == 'base_link'
            for message in node.odometry
        )
        minimum_odometry_count = 1 if args.stationary else 5
        stable = (
            len(node.odometry) >= minimum_odometry_count
            and finite
            and monotonic
            and covariance_nonzero
            and frames_valid
            and node.tf_count == 0
        )
        motion_matches = motion_detected is not args.stationary
        result = {
            'classification': (
                'PASS' if stable and motion_matches else 'AMBIGUOUS'),
            'odom_count': len(node.odometry),
            'finite': finite,
            'motion_detected': motion_detected,
            'expected_motion': not args.stationary,
            'stamp_monotonic': monotonic,
            'covariance_nonzero': covariance_nonzero,
            'frames_valid': frames_valid,
            'dynamic_tf_count': node.tf_count,
        }
        print(json.dumps(result, sort_keys=True))
        return 0 if stable and motion_matches else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
