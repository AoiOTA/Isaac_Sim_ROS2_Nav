"""Activate Nav2 only after simulation data and localization are ready."""

import math
import time

from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.time import Time
from robot_bringup.readiness import ReadinessConfig, ReadinessTracker
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


class Nav2ActivationGate(Node):
    """Call lifecycle STARTUP after all navigation inputs are ready."""

    def __init__(self):
        super().__init__('nav2_activation_gate')
        self.declare_parameter('startup_timeout', 30.0)
        self.declare_parameter('check_period', 0.10)
        self.declare_parameter('freshness_timeout', 0.50)
        self.declare_parameter('tf_stable_duration', 1.00)
        self.declare_parameter('tf_translation_tolerance', 0.05)
        self.declare_parameter('tf_yaw_tolerance', 0.0523598776)
        self.declare_parameter(
            'lifecycle_service',
            '/lifecycle_manager_navigation/manage_nodes',
        )

        self._startup_timeout = self._positive_parameter('startup_timeout')
        check_period = self._positive_parameter('check_period')
        readiness_config = ReadinessConfig(
            freshness_timeout=self._positive_parameter('freshness_timeout'),
            tf_stable_duration=self._positive_parameter(
                'tf_stable_duration'),
            tf_translation_tolerance=self._positive_parameter(
                'tf_translation_tolerance'),
            tf_yaw_tolerance=self._positive_parameter('tf_yaw_tolerance'),
        )
        self._tracker = ReadinessTracker(readiness_config)
        self._started_at = time.monotonic()
        self._last_status_at = self._started_at
        self._request_in_flight = False
        self._activated = False

        best_effort = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        transient_local = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._clock_subscription = self.create_subscription(
            Clock, '/clock', self._clock_callback, best_effort)
        self._scan_subscription = self.create_subscription(
            LaserScan, '/scan', self._scan_callback, best_effort)
        self._odom_subscription = self.create_subscription(
            Odometry, '/odom', self._odom_callback, reliable)
        self._map_subscription = self.create_subscription(
            OccupancyGrid, '/map', self._map_callback, transient_local)

        self._tf_buffer = Buffer(node=self)
        self._tf_listener = TransformListener(self._tf_buffer, self)
        service_name = str(self.get_parameter('lifecycle_service').value)
        self._lifecycle_client = self.create_client(
            ManageLifecycleNodes, service_name)
        self._timer = self.create_timer(check_period, self._check_readiness)

    def _positive_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be finite and positive')
        return value

    def _clock_callback(self, message):
        stamp_s = message.clock.sec + message.clock.nanosec * 1.0e-9
        self._tracker.mark_clock(stamp_s, time.monotonic())

    def _scan_callback(self, message):
        del message
        self._tracker.mark_scan(time.monotonic())

    def _odom_callback(self, message):
        del message
        self._tracker.mark_odom(time.monotonic())

    def _map_callback(self, message):
        del message
        self._tracker.mark_map()

    def _check_readiness(self):
        now = time.monotonic()
        elapsed = now - self._started_at
        if elapsed >= self._startup_timeout:
            missing = ', '.join(self._tracker.missing_requirements(now))
            raise RuntimeError(
                f'Nav2 activation gate timed out after {elapsed:.1f}s; '
                f'missing: {missing}')

        self._observe_map_to_odom(now)
        if self._activated or self._request_in_flight:
            return

        missing = self._tracker.missing_requirements(now)
        service_ready = self._lifecycle_client.service_is_ready()
        if missing or not service_ready:
            if now - self._last_status_at >= 2.0:
                if not service_ready:
                    missing.append('Nav2 lifecycle service')
                self.get_logger().info(
                    'Waiting to activate Nav2: ' + ', '.join(missing))
                self._last_status_at = now
            return

        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        self._request_in_flight = True
        self.get_logger().info(
            'Readiness gate satisfied; requesting Nav2 lifecycle STARTUP')
        future = self._lifecycle_client.call_async(request)
        future.add_done_callback(self._startup_done)

    def _observe_map_to_odom(self, now):
        try:
            transform = self._tf_buffer.lookup_transform(
                'map', 'odom', Time())
        except TransformException:
            return
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        translation = transform.transform.translation
        stamp = transform.header.stamp
        stamp_s = stamp.sec + stamp.nanosec * 1.0e-9
        self._tracker.observe_transform(
            translation.x, translation.y, yaw, stamp_s, now)

    def _startup_done(self, future):
        self._request_in_flight = False
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError('Nav2 lifecycle STARTUP request failed')
        self._activated = True
        self._timer.cancel()
        self.get_logger().info('Nav2 lifecycle activation completed')


def main(args=None):
    """Run the Nav2 activation gate."""
    rclpy.init(args=args)
    node = Nav2ActivationGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
