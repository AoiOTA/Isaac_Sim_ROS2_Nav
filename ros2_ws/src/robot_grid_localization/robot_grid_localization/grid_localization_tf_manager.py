"""Accept triggered Grid Localizer poses and own the map->odom transform."""

import math
from threading import Lock
from typing import Optional

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from isaac_ros_pointcloud_interfaces.msg import FlatScan
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from robot_grid_localization.core import (
    all_finite,
    GateDecision,
    LocalizationGate,
    map_to_odom,
    RigidTransform,
    status_values,
)
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


class GridLocalizationTFManager(Node):
    """Generation gate and sole V6-GRID publisher of ``map->odom``."""

    def __init__(self) -> None:
        super().__init__('grid_localization_tf_manager')
        self.declare_parameter('tf_lookup_timeout_s', 0.1)
        self.declare_parameter('tf_broadcast_rate_hz', 20.0)
        self.declare_parameter('pending_timeout_s', 10.0)

        self._gate = LocalizationGate()
        self._gate_lock = Lock()
        self._latest_correction: Optional[RigidTransform] = None
        self._tf_timeout = Duration(
            seconds=float(self.get_parameter('tf_lookup_timeout_s').value))
        tf_broadcast_rate_hz = float(
            self.get_parameter('tf_broadcast_rate_hz').value)
        pending_timeout_s = float(
            self.get_parameter('pending_timeout_s').value)
        if tf_broadcast_rate_hz <= 0.0:
            raise ValueError('tf_broadcast_rate_hz must be positive')
        if pending_timeout_s <= 0.0:
            raise ValueError('pending_timeout_s must be positive')
        self._pending_timeout_ns = int(pending_timeout_s * 1.0e9)

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/bio_nav/localization_pose',
            latched_qos,
        )
        self._status_publisher = self.create_publisher(
            DiagnosticArray,
            '/bio_nav/localization/status',
            latched_qos,
        )
        self._tf_broadcaster = TransformBroadcaster(self)
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._result_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/localization_result',
            self._on_localization_result,
            10,
        )
        vendor_qos = QoSProfile(depth=10)
        self._flat_scan_subscription = self.create_subscription(
            FlatScan,
            '/flatscan',
            self._on_flat_scan,
            vendor_qos,
        )
        self._flat_scan_trigger_publisher = self.create_publisher(
            FlatScan,
            '/flatscan_localization',
            vendor_qos,
        )
        self._relocalize_service = self.create_service(
            Trigger, '/bio_nav/relocalize', self._on_relocalize)
        self._maintenance_timer = self.create_timer(
            1.0 / tf_broadcast_rate_hz, self._on_maintenance_timer)

    def _on_relocalize(self, _request, response):
        with self._gate_lock:
            if self._gate.pending_generation is not None:
                response.success = False
                response.message = (
                    f'localization request already pending; generation='
                    f'{self._gate.pending_generation}')
                return response
            now = self.get_clock().now()
            decision = self._gate.begin_trigger(now.nanoseconds)
            self._publish_status(
                decision, 'WAITING_FOR_SCAN', None, now)

        response.success = True
        response.message = (
            f'waiting for next FlatScan; generation={decision.generation}')
        return response

    def _on_flat_scan(self, message: FlatScan) -> None:
        scan_stamp_ns = (
            message.header.stamp.sec * 1_000_000_000
            + message.header.stamp.nanosec)
        now = self.get_clock().now()
        with self._gate_lock:
            decision = self._gate.observe_scan(scan_stamp_ns)
            if decision is None:
                return
            self._publish_status(
                decision, 'WAITING_FOR_RESULT', None, now)

        try:
            self._flat_scan_trigger_publisher.publish(message)
        except Exception as exc:
            with self._gate_lock:
                if self._gate.pending_generation != decision.generation:
                    return
                failed = self._gate.reject_pending('scan_forward_error')
            self.get_logger().error(
                f'FlatScan forward failed for generation='
                f'{decision.generation}: {exc}')
            self._publish_status(
                failed, 'REJECTED', None, self.get_clock().now())

    def _on_maintenance_timer(self) -> None:
        now = self.get_clock().now()
        with self._gate_lock:
            timeout_decision = self._gate.expire_pending(
                now.nanoseconds, self._pending_timeout_ns)
        if timeout_decision is not None:
            self._publish_status(
                timeout_decision, 'REJECTED', None, now)
        if self._latest_correction is not None:
            self._broadcast_correction(self._latest_correction, now)

    @staticmethod
    def _pose_is_finite(message: PoseWithCovarianceStamped) -> bool:
        pose = message.pose.pose
        values = (
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
            *message.pose.covariance,
        )
        quaternion_norm = math.sqrt(
            pose.orientation.x ** 2 + pose.orientation.y ** 2
            + pose.orientation.z ** 2 + pose.orientation.w ** 2)
        return all_finite(values) and quaternion_norm > 1.0e-12

    @staticmethod
    def _pose_transform(message: PoseWithCovarianceStamped) -> RigidTransform:
        pose = message.pose.pose
        return RigidTransform(
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
        )

    @staticmethod
    def _tf_transform(message: TransformStamped) -> RigidTransform:
        transform = message.transform
        return RigidTransform(
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        )

    def _on_localization_result(
            self, message: PoseWithCovarianceStamped) -> None:
        result_stamp_ns = (
            message.header.stamp.sec * 1_000_000_000
            + message.header.stamp.nanosec)

        with self._gate_lock:
            stamp_decision = self._gate.result_stamp_decision(result_stamp_ns)
            if stamp_decision is not None:
                still_pending = self._gate.pending_generation is not None
                expected_stamp_ns = self._gate.expected_result_stamp_ns
        if stamp_decision is not None:
            if still_pending:
                self.get_logger().debug(
                    f'Ignoring localization result stamp={result_stamp_ns}; '
                    f'expected={expected_stamp_ns}')
            else:
                self._publish_status(
                    stamp_decision, 'REJECTED', None,
                    self.get_clock().now())
            return

        finite = self._pose_is_finite(message)
        odom_to_base = None
        if finite and result_stamp_ns > 0 and message.header.frame_id == 'map':
            try:
                odom_to_base = self._tf_buffer.lookup_transform(
                    'odom', 'base_link',
                    Time.from_msg(message.header.stamp),
                    timeout=self._tf_timeout,
                )
                finite = self._transform_is_finite(odom_to_base)
            except TransformException:
                odom_to_base = None

        with self._gate_lock:
            if (message.header.frame_id != 'map'
                    and self._gate.pending_generation is not None):
                decision = self._gate.reject_pending(
                    'invalid_result_frame', result_stamp_ns)
            else:
                decision = self._gate.classify_result(
                    result_stamp_ns, finite, odom_to_base is not None)

        received_at = self.get_clock().now()
        if not decision.accepted:
            self._publish_status(
                decision, 'REJECTED', None, received_at)
            return

        correction = map_to_odom(
            self._pose_transform(message), self._tf_transform(odom_to_base))
        self._latest_correction = correction
        self._broadcast_correction(
            correction, Time.from_msg(message.header.stamp))
        self._broadcast_correction(correction, received_at)
        self._pose_publisher.publish(message)
        self._publish_status(
            decision, 'ACCEPTED', correction, received_at)

    def _broadcast_correction(
            self, correction: RigidTransform, stamp: Time) -> None:
        transform = TransformStamped()
        transform.header.stamp = stamp.to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'
        transform.transform.translation.x = correction.x
        transform.transform.translation.y = correction.y
        transform.transform.translation.z = correction.z
        transform.transform.rotation.x = correction.qx
        transform.transform.rotation.y = correction.qy
        transform.transform.rotation.z = correction.qz
        transform.transform.rotation.w = correction.qw
        self._tf_broadcaster.sendTransform(transform)

    @staticmethod
    def _transform_is_finite(message: TransformStamped) -> bool:
        transform = message.transform
        values = (
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        )
        quaternion_norm = math.sqrt(
            transform.rotation.x ** 2 + transform.rotation.y ** 2
            + transform.rotation.z ** 2 + transform.rotation.w ** 2)
        return all_finite(values) and quaternion_norm > 1.0e-12

    def _publish_status(
            self,
            decision: GateDecision,
            state: str,
            correction: Optional[RigidTransform],
            stamp) -> None:
        latency_s = 0.0
        if decision.trigger_stamp_ns > 0:
            latency_s = (
                stamp.nanoseconds - decision.trigger_stamp_ns) / 1.0e9
        status = DiagnosticStatus()
        status.level = (
            DiagnosticStatus.OK
            if state in {'WAITING', 'ACCEPTED'}
            else DiagnosticStatus.WARN)
        status.name = 'grid_localization'
        status.hardware_id = 'isaac_ros_occupancy_grid_localizer'
        status.message = decision.reason
        status.values = [
            KeyValue(key=key, value=value)
            for key, value in status_values(
                decision, state, correction, latency_s)
        ]
        array = DiagnosticArray()
        array.header.stamp = stamp.to_msg()
        array.status = [status]
        self._status_publisher.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GridLocalizationTFManager()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
