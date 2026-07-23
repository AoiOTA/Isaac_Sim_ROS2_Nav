from geometry_msgs.msg import TransformStamped
import math
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster


def identity_map_to_odom(stamp) -> TransformStamped:
    return spawn_aligned_map_to_odom(stamp, x=0.0, y=0.0, yaw_deg=0.0)


def spawn_aligned_map_to_odom(stamp, *, x: float, y: float, yaw_deg: float) -> TransformStamped:
    """Place the selected ideal-odometry origin at its calibrated Map pose."""
    if not all(math.isfinite(value) for value in (x, y, yaw_deg)):
        raise ValueError("map-to-odom spawn pose must contain finite values")
    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id = 'map'
    transform.child_frame_id = 'odom'
    transform.transform.translation.x = x
    transform.transform.translation.y = y
    half_yaw = math.radians(yaw_deg) * 0.5
    transform.transform.rotation.z = math.sin(half_yaw)
    transform.transform.rotation.w = math.cos(half_yaw)
    return transform


class IdealLocalizationTransform(Node):
    """Publish the calibrated selected-spawn map->odom transform."""

    def __init__(self) -> None:
        super().__init__('ideal_localization_tf')
        self._broadcaster = TransformBroadcaster(self)
        self._map_to_odom_x = float(
            self.declare_parameter('map_to_odom_x', 0.0).value)
        self._map_to_odom_y = float(
            self.declare_parameter('map_to_odom_y', 0.0).value)
        self._map_to_odom_yaw_deg = float(
            self.declare_parameter('map_to_odom_yaw_deg', 0.0).value)
        # Validate before publishing so a malformed launch parameter cannot
        # silently place RViz at the old map origin.
        spawn_aligned_map_to_odom(
            self.get_clock().now().to_msg(),
            x=self._map_to_odom_x,
            y=self._map_to_odom_y,
            yaw_deg=self._map_to_odom_yaw_deg,
        )
        self.get_logger().info(
            'Ideal map->odom aligned to selected spawn: '
            f'x={self._map_to_odom_x:.3f}, y={self._map_to_odom_y:.3f}, '
            f'yaw_deg={self._map_to_odom_yaw_deg:.1f}')
        # Preserve the reset contract used by the experiment runner.  Ideal
        # localization has no scan buffer, so clearing it is a no-op.
        self._clear_service = self.create_service(
            Empty,
            '/slam_toolbox/clear_localization_buffer',
            self._clear_localization_buffer,
        )
        self._timer = self.create_timer(0.05, self._publish)

    @staticmethod
    def _clear_localization_buffer(request, response):
        del request
        return response

    def _publish(self) -> None:
        self._broadcaster.sendTransform(
            spawn_aligned_map_to_odom(
                self.get_clock().now().to_msg(),
                x=self._map_to_odom_x,
                y=self._map_to_odom_y,
                yaw_deg=self._map_to_odom_yaw_deg,
            ))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IdealLocalizationTransform()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
