"""Strict Isaac RTX Ouster PointCloud2 to FAST-LIO2 schema adapter."""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2, PointField


INPUT_TOPIC = "/lio/points_raw_isaac"
OUTPUT_TOPIC = "/lio/points_raw"
EXPECTED_FRAME = "lio_lidar_link"
MAX_SCAN_DURATION_NS = 120_000_000
DEFAULT_MAX_RING = 31
MAX_UINT8_RING = 255


class PointCloudContractError(ValueError):
    """The raw cloud cannot be converted without inventing required data."""


_REQUIRED_FIELDS = {
    "x": (PointField.FLOAT32, 1, "<f4"),
    "y": (PointField.FLOAT32, 1, "<f4"),
    "z": (PointField.FLOAT32, 1, "<f4"),
    "intensity": (PointField.FLOAT32, 1, "<f4"),
    "channel_id": (PointField.UINT32, 1, "<u4"),
    "timestamp": (PointField.UINT32, 2, ("<u4", (2,))),
}


def _validate_max_ring(max_ring: int) -> int:
    if isinstance(max_ring, bool) or not isinstance(
        max_ring, (int, np.integer)
    ):
        raise PointCloudContractError(
            "max_ring must be an integer in [0, 255]"
        )
    value = int(max_ring)
    if value < 0 or value > MAX_UINT8_RING:
        raise PointCloudContractError(
            "max_ring must be an integer in [0, 255]"
        )
    return value


def _raw_dtype(message: PointCloud2) -> np.dtype:
    if message.is_bigendian:
        raise PointCloudContractError(
            "big-endian Isaac PointCloud2 is unsupported"
        )
    if message.point_step <= 0:
        raise PointCloudContractError("point_step must be positive")

    by_name: dict[str, PointField] = {}
    for field in message.fields:
        if field.name in by_name:
            raise PointCloudContractError(
                f"duplicate PointCloud2 field {field.name!r}"
            )
        by_name[field.name] = field

    names: list[str] = []
    formats: list[object] = []
    offsets: list[int] = []
    occupied: list[tuple[int, int, str]] = []
    for name, (datatype, count, numpy_format) in _REQUIRED_FIELDS.items():
        field = by_name.get(name)
        if field is None:
            raise PointCloudContractError(
                f"required PointCloud2 field {name!r} is absent"
            )
        if field.datatype != datatype or field.count != count:
            raise PointCloudContractError(
                f"field {name!r} must have datatype={datatype}, "
                f"count={count}; "
                f"got datatype={field.datatype}, count={field.count}"
            )
        byte_count = 4 * count
        start = int(field.offset)
        end = start + byte_count
        if start < 0 or end > message.point_step:
            raise PointCloudContractError(
                f"field {name!r} exceeds point_step={message.point_step}"
            )
        for other_start, other_end, other_name in occupied:
            if start < other_end and other_start < end:
                raise PointCloudContractError(
                    f"fields {name!r} and {other_name!r} overlap"
                )
        occupied.append((start, end, name))
        names.append(name)
        formats.append(numpy_format)
        offsets.append(start)

    return np.dtype(
        {
            "names": names,
            "formats": formats,
            "offsets": offsets,
            "itemsize": int(message.point_step),
        }
    )


def convert_isaac_ouster_cloud(
    message: PointCloud2,
    *,
    expected_frame: str = EXPECTED_FRAME,
    max_scan_duration_ns: int = MAX_SCAN_DURATION_NS,
    max_ring: int = DEFAULT_MAX_RING,
) -> PointCloud2:
    """Convert one ordered raw scan without TF, sorting, or deskew."""

    max_ring = _validate_max_ring(max_ring)
    if not expected_frame or message.header.frame_id != expected_frame:
        raise PointCloudContractError(
            f"raw frame_id must be {expected_frame!r}; "
            f"got {message.header.frame_id!r}"
        )
    if message.width <= 0 or message.height <= 0:
        raise PointCloudContractError(
            "raw PointCloud2 must contain at least one point"
        )
    minimum_row_step = int(message.point_step) * int(message.width)
    if message.row_step < minimum_row_step:
        raise PointCloudContractError(
            f"row_step={message.row_step} is smaller than {minimum_row_step}"
        )
    expected_size = int(message.row_step) * int(message.height)
    if len(message.data) != expected_size:
        raise PointCloudContractError(
            f"data length={len(message.data)} does not match "
            f"row_step*height={expected_size}"
        )
    if max_scan_duration_ns <= 0 or max_scan_duration_ns > 0xFFFFFFFF:
        raise PointCloudContractError(
            "max_scan_duration_ns must fit uint32 and be positive"
        )

    dtype = _raw_dtype(message)
    points = np.ndarray(
        shape=(int(message.height), int(message.width)),
        dtype=dtype,
        buffer=memoryview(message.data),
        strides=(int(message.row_step), int(message.point_step)),
    ).reshape(-1)

    for name in ("x", "y", "z"):
        if not np.isfinite(points[name]).all():
            raise PointCloudContractError(
                f"field {name!r} contains non-finite values"
            )

    channels = points["channel_id"].astype(np.uint64, copy=False)
    if np.any(channels > max_ring):
        observed_min = int(channels.min())
        observed_max = int(channels.max())
        raise PointCloudContractError(
            "channel_id must be directly representable as ring in "
            f"[0, {max_ring}]; "
            f"observed [{observed_min}, {observed_max}]"
        )

    timestamp_words = points["timestamp"].astype(np.uint64, copy=False)
    timestamps = timestamp_words[:, 0] | (
        timestamp_words[:, 1] << np.uint64(32)
    )
    first_timestamp = int(timestamps[0])
    if np.any(timestamps < timestamps[0]):
        raise PointCloudContractError(
            "a point timestamp precedes the first point"
        )
    if np.any(timestamps[1:] < timestamps[:-1]):
        raise PointCloudContractError(
            "point timestamps are not acquisition-order monotonic; "
            "points are not sorted"
        )
    relative_ns = timestamps - timestamps[0]
    duration_ns = int(relative_ns.max())
    if duration_ns > max_scan_duration_ns:
        raise PointCloudContractError(
            f"scan duration {duration_ns} ns exceeds {max_scan_duration_ns} ns"
        )

    output_dtype = np.dtype(
        {
            "names": ("x", "y", "z", "intensity", "ring", "t"),
            "formats": ("<f4", "<f4", "<f4", "<f4", "u1", "<u4"),
            "offsets": (0, 4, 8, 12, 16, 17),
            "itemsize": 21,
        }
    )
    output_points = np.empty(points.size, dtype=output_dtype)
    for name in ("x", "y", "z", "intensity"):
        output_points[name] = points[name]
    output_points["ring"] = channels.astype(np.uint8)
    output_points["t"] = relative_ns.astype(np.uint32)

    output = PointCloud2()
    output.header.frame_id = expected_frame
    output.header.stamp.sec = first_timestamp // 1_000_000_000
    output.header.stamp.nanosec = first_timestamp % 1_000_000_000
    output.height = int(message.height)
    output.width = int(message.width)
    output.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(
            name="intensity",
            offset=12,
            datatype=PointField.FLOAT32,
            count=1,
        ),
        PointField(name="ring", offset=16, datatype=PointField.UINT8, count=1),
        PointField(name="t", offset=17, datatype=PointField.UINT32, count=1),
    ]
    output.is_bigendian = False
    output.point_step = output_dtype.itemsize
    output.row_step = output.point_step * output.width
    output.data = output_points.tobytes(order="C")
    output.is_dense = True
    return output


def sensor_data_qos(depth: int = 5) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class OusterPointCloudAdapter(Node):
    def __init__(self) -> None:
        super().__init__("ouster_pointcloud_adapter")
        self.declare_parameter("input_topic", INPUT_TOPIC)
        self.declare_parameter("output_topic", OUTPUT_TOPIC)
        self.declare_parameter("expected_frame", EXPECTED_FRAME)
        self.declare_parameter("max_scan_duration_ns", MAX_SCAN_DURATION_NS)
        self.declare_parameter("max_ring", DEFAULT_MAX_RING)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self._expected_frame = str(self.get_parameter("expected_frame").value)
        self._max_scan_duration_ns = int(
            self.get_parameter("max_scan_duration_ns").value
        )
        self._max_ring = _validate_max_ring(
            self.get_parameter("max_ring").value
        )
        qos = sensor_data_qos()
        self._publisher = self.create_publisher(PointCloud2, output_topic, qos)
        self._subscription = self.create_subscription(
            PointCloud2, input_topic, self._adapt, qos
        )

    def _adapt(self, message: PointCloud2) -> None:
        try:
            output = convert_isaac_ouster_cloud(
                message,
                expected_frame=self._expected_frame,
                max_scan_duration_ns=self._max_scan_duration_ns,
                max_ring=self._max_ring,
            )
        except PointCloudContractError as exc:
            self.get_logger().error(f"rejecting raw Isaac Ouster cloud: {exc}")
            return
        self._publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OusterPointCloudAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
