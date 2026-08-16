"""
Open-loop calibration driver comparing estimated odometry to ground truth.

The node drives the robot through a fixed pattern (straight, rotate CCW,
rotate CW, straight) with open-loop /cmd_vel, samples /ground_truth/odom,
/odom (EKF) and /wheel/odom at every segment boundary, and writes a JSON
summary.  Ground truth is consumed offline here only; it never feeds the
online odometry chain.
"""

import json
import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile

from robot_odometry.calibration_metrics import PlanarPose
from robot_odometry.calibration_metrics import segment_motion
from robot_odometry.calibration_metrics import wrap_angle
from robot_odometry.calibration_metrics import yaw_from_quaternion


PHASE_STRAIGHT_OUT = 'straight_out'
PHASE_ROTATE_CCW = 'rotate_ccw'
PHASE_ROTATE_CW = 'rotate_cw'
PHASE_STRAIGHT_BACK = 'straight_back'
PHASE_SETTLE = 'settle'


def _odom_to_planar(msg: Odometry) -> PlanarPose:
    pose = msg.pose.pose
    return PlanarPose(
        x=pose.position.x,
        y=pose.position.y,
        yaw=yaw_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ),
    )


class OdomCalibrationDriver(Node):
    """Drive the calibration pattern and summarize odom against ground truth."""

    def __init__(self):
        super().__init__('odom_calibration_driver')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        self.declare_parameter('command_topic', '/cmd_vel')
        self.declare_parameter('ground_truth_topic', '/ground_truth/odom')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('wheel_odom_topic', '/wheel/odom')
        self.declare_parameter('output_file', '')
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('straight_time_s', 16.67)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('rotate_time_s', 2.0 * math.pi / 0.5)
        self.declare_parameter('settle_time_s', 2.0)
        self.declare_parameter('start_timeout_s', 60.0)

        linear_speed = float(self.get_parameter('linear_speed').value)
        straight_time = float(self.get_parameter('straight_time_s').value)
        angular_speed = float(self.get_parameter('angular_speed').value)
        rotate_time = float(self.get_parameter('rotate_time_s').value)
        settle_time = float(self.get_parameter('settle_time_s').value)
        for name, value in (
            ('linear_speed', linear_speed),
            ('straight_time_s', straight_time),
            ('angular_speed', angular_speed),
            ('rotate_time_s', rotate_time),
            ('settle_time_s', settle_time),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')

        self._program = [
            (PHASE_STRAIGHT_OUT, linear_speed, 0.0, straight_time),
            (PHASE_SETTLE, 0.0, 0.0, settle_time),
            (PHASE_ROTATE_CCW, 0.0, angular_speed, rotate_time),
            (PHASE_SETTLE, 0.0, 0.0, settle_time),
            (PHASE_ROTATE_CW, 0.0, -angular_speed, rotate_time),
            (PHASE_SETTLE, 0.0, 0.0, settle_time),
            (PHASE_STRAIGHT_BACK, linear_speed, 0.0, straight_time),
            (PHASE_SETTLE, 0.0, 0.0, settle_time),
        ]

        self._latest = {}
        self._sample_counts = {}
        self._last_yaw = {}
        self._yaw_accum = {}
        self._boundaries = []
        self._phase_index = -1
        self._phase_start_stamp = None
        self.finished = False

        command_topic = self.get_parameter('command_topic').value
        self._command_pub = self.create_publisher(
            Twist, command_topic, QoSProfile(depth=10))
        for key, topic in (
            ('ground_truth', self.get_parameter('ground_truth_topic').value),
            ('odom', self.get_parameter('odom_topic').value),
            ('wheel_odom', self.get_parameter('wheel_odom_topic').value),
        ):
            self.create_subscription(
                Odometry,
                topic,
                lambda msg, key=key: self._on_odom(key, msg),
                qos_profile_sensor_data,
            )

        self._wait_deadline = time.monotonic() + float(
            self.get_parameter('start_timeout_s').value)
        self._command_timer = self.create_timer(0.05, self._publish_command)
        self._state_timer = self.create_timer(0.1, self._advance_state)
        self.get_logger().info(
            'odom calibration driver waiting for ground truth and odom feeds')

    def _on_odom(self, key, msg: Odometry) -> None:
        pose = _odom_to_planar(msg)
        self._latest[key] = pose
        self._sample_counts[key] = self._sample_counts.get(key, 0) + 1
        # Accumulate wrapped per-sample yaw deltas so segment rotations stay
        # valid beyond one half turn (a boundary-only difference wraps to
        # (-pi, pi] and misreads >180 deg turns).
        last_yaw = self._last_yaw.get(key)
        if last_yaw is not None:
            self._yaw_accum[key] = self._yaw_accum.get(key, 0.0) + wrap_angle(
                pose.yaw - last_yaw)
        self._last_yaw[key] = pose.yaw

    def _publish_command(self) -> None:
        command = Twist()
        if 0 <= self._phase_index < len(self._program):
            _, vx, wz, _ = self._program[self._phase_index]
            command.linear.x = vx
            command.angular.z = wz
        self._command_pub.publish(command)

    def _advance_state(self) -> None:
        if self.finished:
            return
        if self._phase_index < 0:
            missing = [
                key for key in ('ground_truth', 'odom', 'wheel_odom')
                if key not in self._latest
            ]
            if missing:
                if time.monotonic() > self._wait_deadline:
                    self.get_logger().error(
                        'timed out waiting for odometry feeds: '
                        + ', '.join(missing))
                    self.finished = True
                return
            self._enter_phase(0)
            return
        _, _, _, duration = self._program[self._phase_index]
        elapsed = (
            self.get_clock().now().nanoseconds - self._phase_start_stamp
        ) * 1e-9
        if elapsed < duration:
            return
        if self._phase_index + 1 < len(self._program):
            self._enter_phase(self._phase_index + 1)
            return
        self._finish()

    def _enter_phase(self, index: int) -> None:
        self._phase_index = index
        self._phase_start_stamp = self.get_clock().now().nanoseconds
        name, vx, wz, duration = self._program[index]
        self._boundaries.append(
            (name, dict(self._latest), dict(self._yaw_accum)))
        if name != PHASE_SETTLE:
            self.get_logger().info(
                f'segment {name}: vx={vx:.3f} wz={wz:.3f} for {duration:.1f}s')

    def _finish(self) -> None:
        segments = []
        boundary_pairs = zip(self._boundaries, self._boundaries[1:])
        for (name, start, start_accum), (_, end, end_accum) in boundary_pairs:
            if name == PHASE_SETTLE:
                continue
            _, vx, wz, duration = next(
                item for item in self._program if item[0] == name)
            segment = {
                'segment': name,
                'commanded': {'vx': vx, 'wz': wz, 'duration_s': duration},
                'samples': {
                    key: self._sample_counts.get(key, 0)
                    for key in ('ground_truth', 'odom', 'wheel_odom')
                },
            }
            for key in ('ground_truth', 'odom', 'wheel_odom'):
                if key in start and key in end:
                    forward, lateral, dyaw = segment_motion(
                        start[key], end[key])
                    if key in start_accum and key in end_accum:
                        dyaw = end_accum[key] - start_accum[key]
                    segment[key] = {
                        'forward_m': forward,
                        'lateral_m': lateral,
                        'dyaw_rad': dyaw,
                    }
            segments.append(segment)

        output_file = self.get_parameter('output_file').value
        if not output_file:
            stamp = time.strftime('%Y%m%d-%H%M%S')
            output_file = f'/tmp/odom_calibration_{stamp}.json'
        payload = {'output_file': output_file, 'segments': segments}
        with open(output_file, 'w', encoding='utf-8') as handle:
            json.dump(payload, handle, indent=2)

        for segment in segments:
            gt = segment.get('ground_truth')
            est = segment.get('odom')
            wheel = segment.get('wheel_odom')
            if gt and est:
                self.get_logger().info(
                    '{segment}: gt(f={gf:.3f}, l={gl:.3f}, yaw={gy:.3f}) '
                    'ekf(f={ef:.3f}, l={el:.3f}, yaw={ey:.3f})'.format(
                        segment=segment['segment'],
                        gf=gt['forward_m'], gl=gt['lateral_m'],
                        gy=gt['dyaw_rad'],
                        ef=est['forward_m'], el=est['lateral_m'],
                        ey=est['dyaw_rad'],
                    ))
            if gt and wheel and abs(wheel['forward_m']) > 1e-6:
                self.get_logger().info(
                    '{segment}: gt/wheel forward ratio={ratio:.4f}'.format(
                        segment=segment['segment'],
                        ratio=gt['forward_m'] / wheel['forward_m'],
                    ))
        self.get_logger().info(f'calibration summary written to {output_file}')
        self.finished = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomCalibrationDriver()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
