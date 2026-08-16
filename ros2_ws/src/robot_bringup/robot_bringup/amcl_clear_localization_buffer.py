import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Empty


class AmclClearLocalizationBuffer(Node):
    """Preserve the reset contract used by the experiment runner.

    The experiment runner clears /slam_toolbox/clear_localization_buffer on
    every reset.  SLAM Toolbox owns a scan buffer behind that service and
    ideal_localization_tf stubs it in ideal mode.  AMCL has no scan buffer
    to clear: after a simulation reset the initial pose publisher re-seeds
    /initialpose and AMCL re-initializes from it, so clearing is a no-op.
    """

    def __init__(self) -> None:
        super().__init__('amcl_clear_localization_buffer')
        self._clear_service = self.create_service(
            Empty,
            '/slam_toolbox/clear_localization_buffer',
            self._clear_localization_buffer,
        )
        self.get_logger().info(
            'AMCL clear-localization-buffer shim ready; resets re-seed '
            '/initialpose instead')

    @staticmethod
    def _clear_localization_buffer(request, response):
        del request
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AmclClearLocalizationBuffer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
