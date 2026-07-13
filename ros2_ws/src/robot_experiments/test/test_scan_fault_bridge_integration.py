"""DDS-level integration checks for the opt-in scan fault bridge."""

import json
import time
import uuid

import pytest


rclpy = pytest.importorskip("rclpy")
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import (  # noqa: E402
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan  # noqa: E402
from std_msgs.msg import Empty, String  # noqa: E402

from robot_experiments.scan_fault_bridge import ScanFaultBridge  # noqa: E402


def _spin_until(executor, predicate, timeout_s=3.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
        if predicate():
            return True
    return predicate()


def _scan(stamp_ns, frame_id="lidar_link"):
    message = LaserScan()
    message.header.stamp.sec = stamp_ns // 1_000_000_000
    message.header.stamp.nanosec = stamp_ns % 1_000_000_000
    message.header.frame_id = frame_id
    message.angle_min = -1.0
    message.angle_max = 1.0
    message.angle_increment = 1.0
    message.range_min = 0.1
    message.range_max = 10.0
    message.ranges = [1.0, 2.0, 3.0]
    return message


def test_ros_topics_apply_faults_publish_status_and_isolate_reset_epoch():
    owns_context = not rclpy.ok()
    if owns_context:
        rclpy.init()
    suffix = uuid.uuid4().hex
    prefix = f"/scan_fault_test_{suffix}"
    topics = {
        "input_topic": f"{prefix}/input",
        "output_topic": f"{prefix}/output",
        "control_topic": f"{prefix}/control",
        "status_topic": f"{prefix}/status",
        "reset_event_topic": f"{prefix}/reset",
    }
    overrides = [
        Parameter(name=name, value=value) for name, value in topics.items()
    ] + [Parameter(name="status_period_s", value=0.05)]
    bridge = ScanFaultBridge(
        node_name=f"scan_fault_bridge_{suffix}", parameter_overrides=overrides
    )
    client = Node(f"scan_fault_test_client_{suffix}")
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(client)

    reliable = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    transient = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    outputs = []
    statuses = []
    scan_publisher = client.create_publisher(
        LaserScan, topics["input_topic"], qos_profile_sensor_data
    )
    control_publisher = client.create_publisher(
        String, topics["control_topic"], reliable
    )
    reset_publisher = client.create_publisher(
        Empty, topics["reset_event_topic"], reliable
    )
    client.create_subscription(
        LaserScan,
        topics["output_topic"],
        outputs.append,
        qos_profile_sensor_data,
    )
    client.create_subscription(
        String,
        topics["status_topic"],
        lambda message: statuses.append(json.loads(message.data)),
        transient,
    )

    def send_command(payload, expected_event="command_applied"):
        previous = len(statuses)
        message = String()
        message.data = json.dumps(payload)
        control_publisher.publish(message)
        assert _spin_until(
            executor,
            lambda: any(
                status["event"] == expected_event
                for status in statuses[previous:]
            ),
        )

    try:
        assert _spin_until(
            executor,
            lambda: client.count_subscribers(topics["input_topic"]) == 1
            and client.count_subscribers(topics["control_topic"]) == 1,
        )

        scan_publisher.publish(_scan(1_000_000_000))
        assert _spin_until(executor, lambda: len(outputs) == 1)
        assert outputs[-1].header.frame_id == "lidar_link"

        send_command({"command": "drop_next", "count": 1, "epoch": 0})
        scan_publisher.publish(_scan(1_100_000_000))
        _spin_until(executor, lambda: False, timeout_s=0.15)
        assert len(outputs) == 1
        scan_publisher.publish(_scan(1_200_000_000))
        assert _spin_until(executor, lambda: len(outputs) == 2)

        send_command(
            {
                "command": "replace_frame_id",
                "frame_id": "fault_missing_lidar",
                "epoch": 0,
            }
        )
        scan_publisher.publish(_scan(1_300_000_000))
        assert _spin_until(executor, lambda: len(outputs) == 3)
        assert outputs[-1].header.frame_id == "fault_missing_lidar"
        assert outputs[-1].ranges == pytest.approx([1.0, 2.0, 3.0])

        send_command({"command": "drop_all", "epoch": 0})
        reset_publisher.publish(Empty())
        assert _spin_until(
            executor,
            lambda: any(
                status["event"] == "reset_event"
                and status["state"]["epoch"] == 1
                for status in statuses
            ),
        )
        scan_publisher.publish(_scan(100_000_000))
        assert _spin_until(executor, lambda: len(outputs) == 4)
        assert outputs[-1].header.frame_id == "lidar_link"

        send_command(
            {"command": "drop_all", "epoch": 0},
            expected_event="command_rejected",
        )
        scan_publisher.publish(_scan(200_000_000))
        assert _spin_until(executor, lambda: len(outputs) == 5)
        rejected = [
            status for status in statuses if status["event"] == "command_rejected"
        ]
        assert rejected[-1]["ok"] is False
        assert "stale epoch 0" in rejected[-1]["error"]
    finally:
        executor.remove_node(client)
        executor.remove_node(bridge)
        client.destroy_node()
        bridge.destroy_node()
        executor.shutdown()
        if owns_context and rclpy.ok():
            rclpy.shutdown()
