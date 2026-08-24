"""Convert aligned 32FC1 metre depth into cuVSLAM 16UC1 millimetres."""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image


_INPUT_TOPIC = '/camera/front/depth/image_raw'
_OUTPUT_TOPIC = '/camera/front/depth/image_uint16'


def convert_depth_image(message):
    """Return a header-preserving contiguous 16UC1 millimetre image."""
    if message.encoding != '32FC1':
        raise ValueError(f'expected 32FC1, got {message.encoding!r}')
    if message.is_bigendian:
        raise ValueError('big-endian depth is unsupported')
    expected_step = message.width * np.dtype('<f4').itemsize
    if message.step != expected_step:
        raise ValueError(
            f'expected contiguous step {expected_step}, got {message.step}')
    expected_size = message.height * message.step
    if len(message.data) != expected_size:
        raise ValueError(
            f'expected {expected_size} depth bytes, got {len(message.data)}')

    metres = np.frombuffer(message.data, dtype='<f4')
    millimetres = np.zeros(metres.shape, dtype='<u2')
    valid = np.isfinite(metres) & (metres > 0.0)
    scaled = np.rint(metres[valid].astype(np.float64) * 1000.0)
    millimetres[valid] = np.clip(scaled, 0.0, 65535.0).astype('<u2')

    output = Image()
    output.header = message.header
    output.height = message.height
    output.width = message.width
    output.encoding = '16UC1'
    output.is_bigendian = 0
    output.step = message.width * np.dtype('<u2').itemsize
    output.data = millimetres.tobytes()
    return output


class DepthFloatToUint16Node(Node):
    """Publish the one exact depth representation needed by cuVSLAM."""

    def __init__(self):
        super().__init__('depth_float_to_uint16')
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(Image, _OUTPUT_TOPIC, qos)
        self._subscription = self.create_subscription(
            Image, _INPUT_TOPIC, self._convert_and_publish, qos)

    def _convert_and_publish(self, message):
        try:
            output = convert_depth_image(message)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        self._publisher.publish(output)


def main(args=None):
    """Run the depth conversion node."""
    rclpy.init(args=args)
    node = DepthFloatToUint16Node()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
