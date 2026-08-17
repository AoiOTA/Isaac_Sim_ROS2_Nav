"""ROS adapter for the pure wheel odometry integrator."""

import math

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import ReliabilityPolicy
from robot_odometry.kinematics import covariance_from_diagonal
from robot_odometry.kinematics import DEFAULT_LEFT_JOINTS
from robot_odometry.kinematics import DEFAULT_RIGHT_JOINTS
from robot_odometry.kinematics import WheelOdometry
from robot_odometry.kinematics import WheelOdometryConfig
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty as EmptyMessage
from std_srvs.srv import Empty


YAW_COVARIANCE_INDEX = 5 * 6 + 5
SLIP_COVARIANCE_SCALE = 10.0
STALE_COVARIANCE_SCALE = 100.0
DIAGNOSTICS_PERIOD_S = 1.0


class WheelOdometryNode(Node):
    """Publish wheel odometry while intentionally never publishing TF."""

    def __init__(self):
        super().__init__('wheel_odometry')
        self.declare_parameter('wheel_radius', 0.098)
        self.declare_parameter('track_width', 0.800)
        self.declare_parameter('left_joint_names', list(DEFAULT_LEFT_JOINTS))
        self.declare_parameter('right_joint_names', list(DEFAULT_RIGHT_JOINTS))
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('max_integration_step', 0.25)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter(
            'pose_covariance_diagonal',
            [0.0025, 0.0025, 1000000.0, 1000000.0, 1000000.0, 0.01],
        )
        self.declare_parameter(
            'twist_covariance_diagonal',
            [0.0025, 1000000.0, 1000000.0, 1000000.0, 1000000.0, 0.01],
        )
        self.declare_parameter('joint_state_timeout_s', 0.1)
        self.declare_parameter('same_side_diff_threshold_radps', 3.0)
        self.declare_parameter('turn_diff_threshold_radps', 1.0)
        self.declare_parameter('turn_yaw_covariance_scale', 2.0)

        publish_rate = float(self.get_parameter('publish_rate').value)
        if not math.isfinite(publish_rate) or publish_rate <= 0.0:
            raise ValueError('publish_rate must be finite and positive')

        self._joint_state_timeout_s = float(
            self.get_parameter('joint_state_timeout_s').value)
        if (not math.isfinite(self._joint_state_timeout_s)
                or self._joint_state_timeout_s <= 0.0):
            raise ValueError(
                'joint_state_timeout_s must be finite and positive')
        self._same_side_diff_threshold = float(
            self.get_parameter('same_side_diff_threshold_radps').value)
        if (not math.isfinite(self._same_side_diff_threshold)
                or self._same_side_diff_threshold < 0.0):
            raise ValueError(
                'same_side_diff_threshold_radps must be finite, non-negative')
        self._turn_diff_threshold = float(
            self.get_parameter('turn_diff_threshold_radps').value)
        if (not math.isfinite(self._turn_diff_threshold)
                or self._turn_diff_threshold < 0.0):
            raise ValueError(
                'turn_diff_threshold_radps must be finite, non-negative')
        self._turn_yaw_covariance_scale = float(
            self.get_parameter('turn_yaw_covariance_scale').value)
        if (not math.isfinite(self._turn_yaw_covariance_scale)
                or self._turn_yaw_covariance_scale < 1.0):
            raise ValueError(
                'turn_yaw_covariance_scale must be finite and >= 1.0')

        config = WheelOdometryConfig(
            wheel_radius=float(self.get_parameter('wheel_radius').value),
            track_width=float(self.get_parameter('track_width').value),
            left_joint_names=tuple(
                self.get_parameter('left_joint_names').value),
            right_joint_names=tuple(
                self.get_parameter('right_joint_names').value),
            max_integration_step=float(
                self.get_parameter('max_integration_step').value),
        )
        self._integrator = WheelOdometry(config)
        self._left_joint_names = config.left_joint_names
        self._right_joint_names = config.right_joint_names
        self._odom_frame = str(self.get_parameter('odom_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._pose_covariance = covariance_from_diagonal(
            self.get_parameter('pose_covariance_diagonal').value)
        self._twist_covariance = covariance_from_diagonal(
            self.get_parameter('twist_covariance_diagonal').value)
        self._last_received_stamp_s = None
        self._last_integrated_sample = None
        self._last_integrated_stamp = None
        self._last_rejection = None
        self._non_monotonic_samples = 0
        self._slip_detected = False
        self._turn_detected = False
        self._last_diagnostics_key = None
        self._last_diagnostics_stamp_s = None

        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._joint_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            reliable_qos,
        )
        self._odom_publisher = self.create_publisher(
            Odometry, '/wheel/odom', reliable_qos)
        self._diagnostics_publisher = self.create_publisher(
            DiagnosticArray, '/diagnostics', reliable_qos)
        self._reset_service = self.create_service(
            Empty, '~/reset', self._reset_callback)
        self._reset_event_subscription = self.create_subscription(
            EmptyMessage,
            '/simulation/reset_event',
            self._reset_event_callback,
            reliable_qos,
        )
        self._timer = self.create_timer(1.0 / publish_rate, self._timer_callback)

    def _joint_state_callback(self, message):
        stamp_s = (message.header.stamp.sec
                   + message.header.stamp.nanosec * 1.0e-9)
        if (self._last_received_stamp_s is not None
                and stamp_s <= self._last_received_stamp_s):
            self._non_monotonic_samples += 1
            self.get_logger().warning(
                f'Skipping non-monotonic joint stamp {stamp_s:.6f}'
                f' (last {self._last_received_stamp_s:.6f})',
                throttle_duration_sec=1.0)
            return
        self._last_received_stamp_s = stamp_s

        result = self._integrator.update(
            list(message.name), list(message.velocity), stamp_s)
        if not result.accepted:
            if result.reason != self._last_rejection:
                self.get_logger().warning(
                    f'Wheel odometry sample rejected: {result.reason}')
                self._last_rejection = result.reason
            return

        self._last_rejection = None
        self._last_integrated_sample = result.sample
        self._last_integrated_stamp = message.header.stamp
        self._slip_detected, self._turn_detected = \
            self._analyze_wheel_quality(
                list(message.name), list(message.velocity))

    def _analyze_wheel_quality(self, names, velocities):
        by_name = dict(zip(names, velocities))
        left = [by_name[name] for name in self._left_joint_names
                if name in by_name]
        right = [by_name[name] for name in self._right_joint_names
                 if name in by_name]
        slip = any(
            (max(side) - min(side)) > self._same_side_diff_threshold
            for side in (left, right) if side)
        turn = False
        if left and right:
            left_mean = sum(left) / len(left)
            right_mean = sum(right) / len(right)
            turn = abs(left_mean - right_mean) > self._turn_diff_threshold
        return slip, turn

    def _timer_callback(self):
        if self._last_integrated_sample is None:
            return
        now = self.get_clock().now()
        now_s = now.nanoseconds * 1.0e-9
        stale = (now_s - self._last_received_stamp_s
                 > self._joint_state_timeout_s)

        scale = 1.0
        if self._slip_detected:
            scale = SLIP_COVARIANCE_SCALE
        if stale:
            scale = STALE_COVARIANCE_SCALE
        yaw_scale = 1.0
        if self._turn_detected and not stale:
            yaw_scale = self._turn_yaw_covariance_scale

        self._odom_publisher.publish(self._to_message(
            self._last_integrated_sample,
            self._last_integrated_stamp,
            _scale_covariance(self._pose_covariance, scale, yaw_scale),
            _scale_covariance(self._twist_covariance, scale, yaw_scale),
        ))
        self._publish_diagnostics(now, now_s, stale, scale, yaw_scale)

    def _publish_diagnostics(self, now, now_s, stale, scale, yaw_scale):
        key = (stale, self._slip_detected, self._turn_detected, scale,
               yaw_scale, self._non_monotonic_samples)
        if (key == self._last_diagnostics_key
                and self._last_diagnostics_stamp_s is not None
                and now_s - self._last_diagnostics_stamp_s
                < DIAGNOSTICS_PERIOD_S):
            return
        self._last_diagnostics_key = key
        self._last_diagnostics_stamp_s = now_s

        status = DiagnosticStatus()
        status.name = 'wheel_odometry'
        status.hardware_id = 'wheel_encoders'
        if stale:
            status.level = DiagnosticStatus.STALE
            status.message = 'joint_states timeout: pose held, covariance x100'
        elif self._slip_detected:
            status.level = DiagnosticStatus.WARN
            status.message = 'same-side wheel speed spread suggests slip'
        elif self._turn_detected:
            status.level = DiagnosticStatus.OK
            status.message = 'turning: yaw covariance inflated'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'ok'
        status.values = [
            KeyValue(key='stale', value=str(stale).lower()),
            KeyValue(key='slip_detected',
                     value=str(self._slip_detected).lower()),
            KeyValue(key='turn_detected',
                     value=str(self._turn_detected).lower()),
            KeyValue(key='covariance_scale', value=f'{scale:g}'),
            KeyValue(key='yaw_covariance_scale', value=f'{yaw_scale:g}'),
            KeyValue(key='non_monotonic_samples',
                     value=str(self._non_monotonic_samples)),
            KeyValue(key='last_rejection', value=self._last_rejection or ''),
        ]
        array = DiagnosticArray()
        array.header.stamp = now.to_msg()
        array.status.append(status)
        self._diagnostics_publisher.publish(array)

    def _to_message(self, sample, stamp, pose_covariance, twist_covariance):
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = self._odom_frame
        message.child_frame_id = self._base_frame
        message.pose.pose.position.x = sample.x
        message.pose.pose.position.y = sample.y
        message.pose.pose.orientation.z = math.sin(0.5 * sample.yaw)
        message.pose.pose.orientation.w = math.cos(0.5 * sample.yaw)
        message.pose.covariance = pose_covariance
        message.twist.twist.linear.x = sample.linear_velocity
        message.twist.twist.angular.z = sample.angular_velocity
        message.twist.covariance = twist_covariance
        return message

    def _reset_callback(self, request, response):
        del request
        self._reset_state()
        self.get_logger().info('Wheel odometry reset safely')
        return response

    def _reset_event_callback(self, message):
        del message
        self._reset_state()
        self.get_logger().info('Wheel odometry reset from simulation event')

    def _reset_state(self):
        self._integrator.reset()
        self._last_received_stamp_s = None
        self._last_integrated_sample = None
        self._last_integrated_stamp = None
        self._last_rejection = None
        self._non_monotonic_samples = 0
        self._slip_detected = False
        self._turn_detected = False
        self._last_diagnostics_key = None
        self._last_diagnostics_stamp_s = None


def _scale_covariance(covariance, scale, yaw_scale):
    """Scale a 6x6 covariance, inflating the yaw diagonal further."""
    return [
        value * (scale * yaw_scale if index == YAW_COVARIANCE_INDEX else scale)
        for index, value in enumerate(covariance)
    ]


def main(args=None):
    """Run the wheel odometry node."""
    rclpy.init(args=args)
    node = WheelOdometryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
