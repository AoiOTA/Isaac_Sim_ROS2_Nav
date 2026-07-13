#!/usr/bin/env python3
"""Publish robot_description without claiming any TF ownership."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class RobotDescriptionPublisher(Node):
    """Keep one transient-local URDF message available to RViz consumers."""

    def __init__(self):
        super().__init__('robot_description_publisher')
        self.declare_parameter('robot_description', '')
        description = self.get_parameter('robot_description').value
        if not description:
            raise ValueError('robot_description parameter must not be empty')
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(
            String, '/robot_description', qos)
        self._publisher.publish(String(data=description))


def main(args=None):
    """Run the description-only publisher."""
    rclpy.init(args=args)
    node = None
    try:
        node = RobotDescriptionPublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as error:
        # A launch-wide SIGINT may invalidate the context between the
        # executor readiness check and WaitSet construction. Treat only that
        # shutdown race as a normal exit; preserve live-context failures.
        if rclpy.ok() or 'context is not valid' not in str(error):
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
