"""ROS adapter preserving raw IMU evidence while correcting EKF yaw rate."""

from copy import deepcopy

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robot_odometry.imu_calibration import ImuYawCalibration
from robot_odometry.imu_calibration import ImuYawCalibrationConfig
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty


class ImuCalibrationNode(Node):
    """Publish one corrected `/imu/data` sample per accepted raw sample."""

    def __init__(self):
        super().__init__('imu_yaw_calibrator')
        self.declare_parameter('input_topic', '/imu/data_raw')
        self.declare_parameter('output_topic', '/imu/data')
        self.declare_parameter('yaw_scale', 0.9294)
        self.declare_parameter('yaw_bias_rad_s', 0.0)
        self.declare_parameter('yaw_variance', 1.0e-4)
        self.declare_parameter('diagnostic_interval', 1000)

        config = ImuYawCalibrationConfig(
            yaw_scale=float(self.get_parameter('yaw_scale').value),
            yaw_bias_rad_s=float(
                self.get_parameter('yaw_bias_rad_s').value),
            yaw_variance=float(self.get_parameter('yaw_variance').value),
        )
        self._calibration = ImuYawCalibration(config)
        self._diagnostic_interval = int(
            self.get_parameter('diagnostic_interval').value)
        if self._diagnostic_interval <= 0:
            raise ValueError('diagnostic_interval must be positive')

        self._publisher = self.create_publisher(
            Imu,
            str(self.get_parameter('output_topic').value),
            qos_profile_sensor_data,
        )
        self._subscription = self.create_subscription(
            Imu,
            str(self.get_parameter('input_topic').value),
            self._raw_callback,
            qos_profile_sensor_data,
        )
        self._reset_subscription = self.create_subscription(
            Empty,
            '/simulation/reset_event',
            self._reset_callback,
            10,
        )

    def _raw_callback(self, message):
        stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        result = self._calibration.calibrate(
            stamp_ns, message.angular_velocity.z)
        self._report_diagnostics(result.reason)
        if not result.accepted:
            return

        corrected = deepcopy(message)
        corrected.angular_velocity.z = result.angular_velocity_z
        corrected.angular_velocity_covariance[8] = (
            self._calibration.config.yaw_variance)
        self._publisher.publish(corrected)

    def _reset_callback(self, message):
        del message
        self._calibration.reset_stamp()

    def _report_diagnostics(self, reason):
        count = self._calibration.counters[reason]
        first_rejection = reason != 'accepted' and count == 1
        periodic = count % self._diagnostic_interval == 0
        if not first_rejection and not periodic:
            return
        counters = ', '.join(
            f'{name}={value}'
            for name, value in self._calibration.counters.items()
        )
        message = f'IMU yaw calibration diagnostics: {counters}'
        if reason == 'accepted':
            self.get_logger().info(message)
        else:
            self.get_logger().warning(message)


def main(args=None):
    """Run the raw-to-corrected IMU adapter."""
    rclpy.init(args=args)
    node = ImuCalibrationNode()
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
