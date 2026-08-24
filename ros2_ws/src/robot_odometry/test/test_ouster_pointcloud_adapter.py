from __future__ import annotations

import math
import struct
from pathlib import Path

import pytest
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField

from robot_odometry.ouster_pointcloud_adapter import (
    PointCloudContractError,
    convert_isaac_ouster_cloud,
    sensor_data_qos,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _raw_cloud(
    *,
    channels=(0, 31, 7),
    timestamps=(4_294_967_290, 4_294_967_310, 4_295_067_290),
    frame="lio_lidar_link",
) -> PointCloud2:
    message = PointCloud2()
    message.header.frame_id = frame
    message.height = 1
    message.width = len(channels)
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(
            name="intensity", offset=12,
            datatype=PointField.FLOAT32, count=1,
        ),
        PointField(
            name="channel_id", offset=16,
            datatype=PointField.UINT32, count=1,
        ),
        PointField(
            name="timestamp", offset=20,
            datatype=PointField.UINT32, count=2,
        ),
    ]
    message.is_bigendian = False
    message.point_step = 28
    message.row_step = message.point_step * message.width
    data = bytearray(message.row_step)
    for index, (channel, timestamp) in enumerate(zip(channels, timestamps)):
        struct.pack_into(
            "<ffffIII",
            data,
            index * message.point_step,
            float(index + 1),
            float(index + 2),
            float(index + 3),
            float(10 + index),
            channel,
            timestamp & 0xFFFFFFFF,
            timestamp >> 32,
        )
    message.data = data
    message.is_dense = True
    return message


def _unpack_output(message: PointCloud2):
    return [
        struct.unpack_from("<ffffBI", message.data, index * message.point_step)
        for index in range(message.width * message.height)
    ]


def test_exact_raw_schema_converts_fields_order_ring_and_relative_time():
    output = convert_isaac_ouster_cloud(_raw_cloud())

    observed_fields = [
        (field.name, field.datatype, field.count)
        for field in output.fields
    ]
    assert observed_fields == [
        ("x", PointField.FLOAT32, 1),
        ("y", PointField.FLOAT32, 1),
        ("z", PointField.FLOAT32, 1),
        ("intensity", PointField.FLOAT32, 1),
        ("ring", PointField.UINT8, 1),
        ("t", PointField.UINT32, 1),
    ]
    assert output.point_step == 21
    assert output.header.frame_id == "lio_lidar_link"
    assert output.header.stamp.sec == 4
    assert output.header.stamp.nanosec == 294_967_290
    points = _unpack_output(output)
    assert [point[:3] for point in points] == [
        (1.0, 2.0, 3.0),
        (2.0, 3.0, 4.0),
        (3.0, 4.0, 5.0),
    ]
    assert [point[4] for point in points] == [0, 31, 7]
    assert [point[5] for point in points] == [0, 20, 100_000]


def test_uint32_low_word_scan_wrap_preserves_absolute_nsec_arithmetic():
    first = (19 << 32) + 0xFFFFFFF0
    output = convert_isaac_ouster_cloud(
        _raw_cloud(
            channels=(2, 2),
            timestamps=(first, first + 64),
        )
    )

    assert output.header.stamp.sec == first // 1_000_000_000
    assert output.header.stamp.nanosec == first % 1_000_000_000
    assert [point[5] for point in _unpack_output(output)] == [0, 64]


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (
            lambda cloud: cloud.fields.pop(),
            "required PointCloud2 field 'timestamp'",
        ),
        (
            lambda cloud: setattr(cloud.fields[-1], "count", 1),
            "field 'timestamp' must have",
        ),
        (
            lambda cloud: setattr(
                cloud.fields[4], "datatype", PointField.UINT16
            ),
            "field 'channel_id' must have",
        ),
        (lambda cloud: setattr(cloud, "is_bigendian", True), "big-endian"),
        (lambda cloud: setattr(cloud, "point_step", 24), "exceeds point_step"),
        (lambda cloud: setattr(cloud, "row_step", 1), "row_step"),
    ],
)
def test_missing_or_bad_raw_contract_fails_loudly(mutation, message):
    cloud = _raw_cloud()
    mutation(cloud)
    with pytest.raises(PointCloudContractError, match=message):
        convert_isaac_ouster_cloud(cloud)


def test_ring_bounds_are_not_shifted_or_fabricated():
    with pytest.raises(PointCloudContractError, match=r"ring in \[0, 31\]"):
        convert_isaac_ouster_cloud(
            _raw_cloud(channels=(1, 32), timestamps=(100, 200))
        )


def test_frame_finite_xyz_timestamp_order_and_duration_are_strict():
    with pytest.raises(PointCloudContractError, match="raw frame_id"):
        convert_isaac_ouster_cloud(_raw_cloud(frame="rtx_world"))

    nonfinite = _raw_cloud()
    struct.pack_into("<f", nonfinite.data, 0, math.nan)
    with pytest.raises(PointCloudContractError, match="non-finite"):
        convert_isaac_ouster_cloud(nonfinite)

    with pytest.raises(
        PointCloudContractError, match="acquisition-order monotonic"
    ):
        convert_isaac_ouster_cloud(
            _raw_cloud(channels=(1, 2, 3), timestamps=(100, 300, 200))
        )

    with pytest.raises(PointCloudContractError, match="scan duration"):
        convert_isaac_ouster_cloud(
            _raw_cloud(channels=(1, 2), timestamps=(100, 120_000_101))
        )


def test_adapter_uses_sensor_data_qos_and_has_no_tf_path():
    qos = sensor_data_qos()
    assert qos.depth == 5
    assert qos.reliability == ReliabilityPolicy.BEST_EFFORT
    assert qos.durability == DurabilityPolicy.VOLATILE

    source = (
        PACKAGE_ROOT / "robot_odometry/ouster_pointcloud_adapter.py"
    ).read_text(encoding="utf-8")
    assert 'for name in ("x", "y", "z", "intensity")' in source
    assert "output_points[name] = points[name]" in source
    assert "tf2" not in source
    assert "Transform" not in source


def test_adapter_launch_is_default_off_with_exact_topics():
    source = (
        PACKAGE_ROOT / "launch/ouster_pointcloud_adapter.launch.py"
    ).read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("enabled", default_value="false")' in source
    assert 'default_value="/lio/points_raw_isaac"' in source
    assert 'default_value="/lio/points_raw"' in source
    assert "IfCondition(enabled)" in source
