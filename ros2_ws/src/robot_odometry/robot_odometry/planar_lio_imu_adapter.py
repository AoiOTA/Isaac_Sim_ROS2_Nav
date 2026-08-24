"""Default-off planar IMU adapter for the isolated FAST-LIO shadow only."""

import math
from copy import deepcopy

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Imu


def make_planar_lio_imu(message: Imu) -> Imu:
    """Copy a valid IMU sample and zero only roll/pitch angular velocity."""
    stamp = message.header.stamp
    if stamp.sec < 0 or stamp.nanosec < 0 or stamp.nanosec >= 1_000_000_000:
        raise ValueError('IMU stamp is invalid')
    if stamp.sec == 0 and stamp.nanosec == 0:
        raise ValueError('IMU stamp must be nonzero')

    values = (
        message.orientation.x,
        message.orientation.y,
        message.orientation.z,
        message.orientation.w,
        message.angular_velocity.x,
        message.angular_velocity.y,
        message.angular_velocity.z,
        message.linear_acceleration.x,
        message.linear_acceleration.y,
        message.linear_acceleration.z,
        *message.orientation_covariance,
        *message.angular_velocity_covariance,
        *message.linear_acceleration_covariance,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError('IMU sample contains a non-finite value')

    output = deepcopy(message)
    output.angular_velocity.x = 0.0
    output.angular_velocity.y = 0.0
    return output


class PlanarLioImuAdapter(Node):
    """Publish a planar copy without changing the canonical IMU topic."""

    def __init__(self):
        """Create the fixed-topic, stateless adapter."""
        super().__init__('planar_lio_imu_adapter')
        self.declare_parameter('input_topic', '/imu/data')
        self.declare_parameter('output_topic', '/imu/lio')
        self.declare_parameter('diagnostic_interval', 1000)
        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self._diagnostic_interval = int(
            self.get_parameter('diagnostic_interval').value
        )
        if not input_topic or not output_topic:
            raise ValueError('input_topic and output_topic must be nonempty')
        if input_topic == output_topic:
            raise ValueError('input_topic and output_topic must differ')
        if self._diagnostic_interval <= 0:
            raise ValueError('diagnostic_interval must be positive')

        self._accepted = 0
        self._invalid = 0
        self._publisher = self.create_publisher(
            Imu, output_topic, qos_profile_sensor_data
        )
        self._subscription = self.create_subscription(
            Imu, input_topic, self._callback, qos_profile_sensor_data
        )

    def _callback(self, message: Imu) -> None:
        try:
            output = make_planar_lio_imu(message)
        except ValueError as error:
            self._invalid += 1
            periodic = self._invalid % self._diagnostic_interval == 0
            if self._invalid == 1 or periodic:
                self.get_logger().warning(
                    'Dropped invalid LIO IMU sample: '
                    f'reason={error}, accepted={self._accepted}, '
                    f'invalid={self._invalid}'
                )
            return

        self._accepted += 1
        if self._accepted % self._diagnostic_interval == 0:
            self.get_logger().info(
                'Planar LIO IMU diagnostics: '
                f'accepted={self._accepted}, invalid={self._invalid}'
            )
        self._publisher.publish(output)


def main(args=None):
    """Run the isolated FAST-LIO IMU adapter."""
    rclpy.init(args=args)
    node = PlanarLioImuAdapter()
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
