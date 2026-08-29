"""Publish the selected initial-pose owner as a durable runtime contract."""

from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def normalize_initial_pose_source(value: object) -> str:
    source = str(value).strip().lower()
    if source not in {"auto", "rviz", "isaac"}:
        raise ValueError("initial_pose_source must be auto, rviz, or isaac")
    return source


class InitialPosePolicyPublisher(Node):
    """Keep one transient-local source policy available to Isaac Sim."""

    def __init__(self) -> None:
        super().__init__("initial_pose_policy")
        source = normalize_initial_pose_source(
            self.declare_parameter("initial_pose_source", "auto").value
        )
        topic = str(
            self.declare_parameter(
                "policy_topic", "/simulation/initial_pose_source"
            ).value
        ).strip()
        if not topic:
            raise ValueError("policy_topic must be non-empty")
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(String, topic, qos)
        message = String()
        message.data = source
        self._publisher.publish(message)
        self.get_logger().info(
            f"initial pose policy published: source={source}, topic={topic}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node: InitialPosePolicyPublisher | None = None
    try:
        node = InitialPosePolicyPublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
