"""ROS adapter for the pure wheel odometry integrator."""

import math

from nav_msgs.msg import Odometry
import rclpy
from rclpy._rclpy_pybind11 import RCLError
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
        self.declare_parameter('stamp_diagnostic_interval', 1000)
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
        self._odom_frame = str(self.get_parameter('odom_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._pose_covariance = covariance_from_diagonal(
            self.get_parameter('pose_covariance_diagonal').value)
        self._twist_covariance = covariance_from_diagonal(
            self.get_parameter('twist_covariance_diagonal').value)
        self._last_joint_stamp_ns = None
        self._stamp_diagnostic_interval = max(
            1, int(self.get_parameter('stamp_diagnostic_interval').value))
        self._stamp_counters = {
            'accepted': 0,
            'duplicate': 0,
            'backward': 0,
        }
        self._warned_rejections = set()
        self._shutting_down = False
        self._shutdown_suppressed_callbacks = 0

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
        self._reset_service = self.create_service(
            Empty, '~/reset', self._reset_callback)
        self._reset_event_subscription = self.create_subscription(
            EmptyMessage,
            '/simulation/reset_event',
            self._reset_event_callback,
            reliable_qos,
        )
    def _joint_state_callback(self, message):
        if self._shutting_down or not self.context.ok():
            self._shutdown_suppressed_callbacks += 1
            return

        stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        if stamp_ns <= 0:
            reason = 'invalid_or_zero_stamp'
        elif (
            self._last_joint_stamp_ns is not None
            and stamp_ns <= self._last_joint_stamp_ns
        ):
            reason = (
                'duplicate_stamp'
                if stamp_ns == self._last_joint_stamp_ns
                else 'time_regression'
            )
        else:
            reason = None

        if reason is not None:
            if reason == 'duplicate_stamp':
                self._record_stamp_event('duplicate', warning=True)
            elif reason == 'time_regression':
                self._record_stamp_event('backward', warning=True)
            else:
                self._warn_rejection(reason)
            return

        self._last_joint_stamp_ns = stamp_ns
        result = self._integrator.update(
            list(message.name),
            list(message.velocity),
            stamp_ns * 1.0e-9,
        )
        if not result.accepted:
            self._warn_rejection(result.reason)
            return

        self._record_stamp_event('accepted')
        try:
            self._odom_publisher.publish(
                self._to_message(result.sample, message.header.stamp))
        except RCLError:
            if self._shutting_down or not self.context.ok():
                self._shutdown_suppressed_callbacks += 1
                return
            raise

    def _record_stamp_event(self, event, warning=False):
        self._stamp_counters[event] += 1
        count = self._stamp_counters[event]
        first_warning = warning and count == 1
        periodic = count % self._stamp_diagnostic_interval == 0
        if not first_warning and not periodic:
            return

        counters = ', '.join(
            f'{name}={value}' for name, value in self._stamp_counters.items())
        message = f'Wheel odometry stamp diagnostics: {counters}'
        if warning:
            self.get_logger().warning(message)
        else:
            self.get_logger().info(message)

    def _warn_rejection(self, reason):
        if reason in self._warned_rejections:
            return
        self._warned_rejections.add(reason)
        self.get_logger().warning(
            f'Wheel odometry sample rejected: {reason}')

    def _to_message(self, sample, stamp):
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = self._odom_frame
        message.child_frame_id = self._base_frame
        message.pose.pose.position.x = sample.x
        message.pose.pose.position.y = sample.y
        message.pose.pose.orientation.z = math.sin(0.5 * sample.yaw)
        message.pose.pose.orientation.w = math.cos(0.5 * sample.yaw)
        message.pose.covariance = self._pose_covariance
        message.twist.twist.linear.x = sample.linear_velocity
        message.twist.twist.angular.z = sample.angular_velocity
        message.twist.covariance = self._twist_covariance
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
        self._last_joint_stamp_ns = None

    def destroy_node(self):
        self._shutting_down = True
        return super().destroy_node()


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
