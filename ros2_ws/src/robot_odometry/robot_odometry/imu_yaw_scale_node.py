"""
Calibrated IMU yaw-rate scaling relay.

The Kujiale IMU's yaw rate over-reads by a systematic +8.4%/+8.6% gain
(mean x1.085) against ground truth (validation doc section 0.2,
2026-08-16).  The EKF therefore fused a biased yaw rate and IMU fusion
was disabled in favor of wheel-only yaw.  This relay rescales the yaw
rate offline-calibrated to unity gain so the EKF can fuse both wheel and
IMU yaw again.  The bias is a fixed gain constant baked into config; no
ground truth enters the online chain.

Only angular_velocity.z is touched; every other field passes through
unchanged.  The yaw-rate covariance is scaled by scale^2, or replaced by
yaw_rate_covariance when the simulator publishes a zero/unspecified
covariance (robot_localization rejects zero covariance).
"""

from copy import deepcopy

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class ImuYawScaleNode(Node):
    """Republish /imu/data with a calibrated yaw-rate gain."""

    def __init__(self):
        super().__init__('imu_yaw_scale')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        self.declare_parameter('input_topic', '/imu/data')
        self.declare_parameter('output_topic', '/imu/data_scaled')
        # Ground-truth-calibrated yaw-rate gain correction (1/1.085).
        self.declare_parameter('yaw_rate_scale', 0.922)
        self.declare_parameter('yaw_rate_bias_radps', 0.0)
        self.declare_parameter('yaw_rate_covariance', 0.005)

        self._scale = float(self.get_parameter('yaw_rate_scale').value)
        self._bias = float(self.get_parameter('yaw_rate_bias_radps').value)
        self._covariance = float(
            self.get_parameter('yaw_rate_covariance').value)
        if not 0.0 < self._scale <= 2.0:
            raise ValueError('yaw_rate_scale must be in (0, 2]')
        if self._covariance <= 0.0:
            raise ValueError('yaw_rate_covariance must be positive')

        self._publisher = self.create_publisher(
            Imu,
            str(self.get_parameter('output_topic').value),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter('input_topic').value),
            self._on_imu,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            'IMU yaw-rate calibration relay: '
            f'scale={self._scale:.4f}, bias={self._bias:.4f} rad/s, '
            f'covariance={self._covariance:.4f}'
        )

    def _on_imu(self, message: Imu) -> None:
        scaled = deepcopy(message)
        raw_rate = message.angular_velocity.z
        scaled.angular_velocity.z = (raw_rate - self._bias) * self._scale
        covariance = message.angular_velocity_covariance[8]
        if covariance > 0.0:
            scaled.angular_velocity_covariance[8] = (
                covariance * self._scale * self._scale
            )
        else:
            scaled.angular_velocity_covariance[8] = self._covariance
        self._publisher.publish(scaled)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuYawScaleNode()
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
