"""ROS adapter for the pure wheel odometry integrator."""

import math
import time

from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.parameter_service import ParameterService
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import ReliabilityPolicy
from robot_odometry.kinematics import covariance_from_diagonal
from robot_odometry.kinematics import WheelOdometry
from robot_odometry.kinematics import WheelOdometryConfig
from robot_odometry.robot_profile import ISAAC_KINEMATICS_PARAMETER_NAMES
from robot_odometry.robot_profile import load_robot_profile
from robot_odometry.robot_profile import validate_isaac_kinematics
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty as EmptyMessage
from std_srvs.srv import Empty


class WheelOdometryNode(Node):
    """Publish wheel odometry while intentionally never publishing TF."""

    def __init__(self):
        super().__init__(
            'wheel_odometry',
            start_parameter_services=False,
            enable_rosout=False,
        )
        try:
            self._declare_runtime_parameters()
            profile = load_robot_profile(
                self._required_string_parameter('robot_config_file'))
            timeout_sec = self._positive_parameter(
                'kinematics_handshake_timeout_sec')
            self._establish_kinematics_contract(profile, timeout_sec)
            self._activate_runtime(profile)
            self.get_logger().info(
                'Wheel odometry kinematics verified against Isaac: '
                f'profile={profile.profile_id}, lifecycle={profile.lifecycle}, '
                f'sha256={profile.sha256}, '
                f'radius={profile.wheel_radius_m:g}m, '
                f'effective_track={profile.effective_track_width_m:g}m')
        except BaseException:
            self.destroy_node()
            raise

    def _declare_runtime_parameters(self):
        read_only = ParameterDescriptor(read_only=True)
        parameters = (
            ('robot_config_file', ''),
            ('isaac_node_name', '/isaac_navigation_sim'),
            ('kinematics_handshake_timeout_sec', 10.0),
            ('publish_rate', 50.0),
            ('max_integration_step', 0.25),
            ('odom_frame', 'odom'),
            ('base_frame', 'base_link'),
            (
                'pose_covariance_diagonal',
                [
                    0.0025, 0.0025, 1000000.0,
                    1000000.0, 1000000.0, 0.01,
                ],
            ),
            (
                'twist_covariance_diagonal',
                [
                    0.0025, 1000000.0, 1000000.0,
                    1000000.0, 1000000.0, 0.01,
                ],
            ),
        )
        for name, default in parameters:
            self.declare_parameter(name, default, read_only)

    def _required_string_parameter(self, name):
        value = self.get_parameter(name).value
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f'{name} must be a non-empty string')
        return value.strip()

    def _positive_parameter(self, name):
        value = self.get_parameter(name).value
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f'{name} must be a finite positive number')
        parsed = float(value)
        if not math.isfinite(parsed) or parsed <= 0.0:
            raise ValueError(f'{name} must be a finite positive number')
        return parsed

    def _establish_kinematics_contract(self, profile, timeout_sec):
        isaac_node_name = self._required_string_parameter('isaac_node_name')
        client = AsyncParameterClient(self, isaac_node_name)
        self._isaac_parameter_client = client
        deadline = time.monotonic() + timeout_sec
        if not client.wait_for_services(timeout_sec=timeout_sec):
            raise TimeoutError(
                'Isaac kinematics parameter services are unavailable: '
                f'node={isaac_node_name}, timeout={timeout_sec:g}s')

        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise TimeoutError(
                'Isaac kinematics handshake timed out before readback')
        future = client.get_parameters(
            list(ISAAC_KINEMATICS_PARAMETER_NAMES))
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=remaining)
        if not future.done():
            future.cancel()
            raise TimeoutError('Isaac kinematics parameter readback timed out')
        if future.exception() is not None:
            raise RuntimeError(
                'Isaac kinematics parameter readback failed') \
                from future.exception()
        response = future.result()
        if (response is None
                or len(response.values)
                != len(ISAAC_KINEMATICS_PARAMETER_NAMES)):
            raise RuntimeError(
                'Isaac returned an incomplete kinematics parameter set')
        values = {
            name: parameter_value_to_python(value)
            for name, value in zip(
                ISAAC_KINEMATICS_PARAMETER_NAMES, response.values)
        }
        return validate_isaac_kinematics(profile, values)

    def _activate_runtime(self, profile):
        publish_rate = self._positive_parameter('publish_rate')
        config = WheelOdometryConfig(
            wheel_radius=profile.wheel_radius_m,
            track_width=profile.effective_track_width_m,
            left_joint_names=profile.left_joint_names,
            right_joint_names=profile.right_joint_names,
            max_integration_step=self._positive_parameter(
                'max_integration_step'),
        )
        self._integrator = WheelOdometry(config)
        self._odom_frame = self._required_string_parameter('odom_frame')
        self._base_frame = self._required_string_parameter('base_frame')
        self._pose_covariance = covariance_from_diagonal(
            self.get_parameter('pose_covariance_diagonal').value)
        self._twist_covariance = covariance_from_diagonal(
            self.get_parameter('twist_covariance_diagonal').value)
        self._latest_joint_sample = None
        self._last_rejection = None

        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._parameter_service = ParameterService(self)
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
        self._timer = self.create_timer(
            1.0 / publish_rate, self._timer_callback)

    def _joint_state_callback(self, message):
        self._latest_joint_sample = (list(message.name), list(message.velocity))

    def _timer_callback(self):
        if self._latest_joint_sample is None:
            return
        now = self.get_clock().now()
        names, velocities = self._latest_joint_sample
        result = self._integrator.update(
            names, velocities, now.nanoseconds * 1.0e-9)
        if not result.accepted:
            if result.reason != self._last_rejection:
                self.get_logger().warning(
                    f'Wheel odometry sample rejected: {result.reason}')
                self._last_rejection = result.reason
            return

        self._last_rejection = None
        self._odom_publisher.publish(self._to_message(result.sample, now.to_msg()))

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
        self._latest_joint_sample = None
        self._last_rejection = None


def main(args=None):
    """Run the wheel odometry node."""
    rclpy.init(args=args)
    node = None
    try:
        node = WheelOdometryNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
