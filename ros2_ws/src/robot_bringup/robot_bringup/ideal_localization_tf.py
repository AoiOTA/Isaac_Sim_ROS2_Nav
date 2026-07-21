from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster


def identity_map_to_odom(stamp) -> TransformStamped:
    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id = 'map'
    transform.child_frame_id = 'odom'
    transform.transform.rotation.w = 1.0
    return transform


class IdealLocalizationTransform(Node):
    """Publish a fresh identity map->odom transform for ideal odometry."""

    def __init__(self) -> None:
        super().__init__('ideal_localization_tf')
        self._broadcaster = TransformBroadcaster(self)
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
            identity_map_to_odom(self.get_clock().now().to_msg()))


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
