import math
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time
from rclpy.node import Node

from robot_experiments.outdoor_initial_pose import OutdoorInitialPose


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(message)


def _fake_node(*, std_x_m=0.10, std_y_m=0.10, std_yaw_deg=5.0, publish_count=5):
    publisher = FakePublisher()
    node = SimpleNamespace(
        x=1.0,
        y=2.0,
        yaw_deg=30.0,
        std_x_m=std_x_m,
        std_y_m=std_y_m,
        std_yaw_deg=std_yaw_deg,
        publish_count=publish_count,
        clock_ready=True,
        scan_ready=True,
        published=0,
        publisher=publisher,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            to_msg=lambda: Time(sec=1)
        )),
        get_logger=lambda: FakeLogger(),
    )
    return node, publisher


def test_declared_std_parameter_defaults(monkeypatch):
    declared = {}

    def fake_declare_parameter(self, name, default):
        declared[name] = default
        return SimpleNamespace(value=default)

    monkeypatch.setattr(Node, "__init__", lambda self, *args, **kwargs: None)
    monkeypatch.setattr(Node, "declare_parameter", fake_declare_parameter)
    monkeypatch.setattr(
        OutdoorInitialPose, "create_publisher", lambda self, *args: SimpleNamespace()
    )
    monkeypatch.setattr(
        OutdoorInitialPose, "create_subscription", lambda self, *args: SimpleNamespace()
    )
    monkeypatch.setattr(
        OutdoorInitialPose, "create_timer", lambda self, *args: SimpleNamespace()
    )

    node = OutdoorInitialPose()

    assert declared["initial_pose_std_x_m"] == 0.10
    assert declared["initial_pose_std_y_m"] == 0.10
    assert declared["initial_pose_std_yaw_deg"] == 5.0
    assert node.std_x_m == 0.10
    assert node.std_y_m == 0.10
    assert node.std_yaw_deg == 5.0


def test_covariance_diagonal_derives_from_std_parameters():
    node, publisher = _fake_node(std_x_m=0.10, std_y_m=0.25, std_yaw_deg=5.0)

    OutdoorInitialPose._tick(node)

    assert len(publisher.messages) == 1
    message = publisher.messages[0]
    assert message.header.frame_id == "map"
    assert message.pose.pose.position.x == pytest.approx(1.0)
    assert message.pose.pose.position.y == pytest.approx(2.0)
    half = 0.5 * math.radians(30.0)
    assert message.pose.pose.orientation.z == pytest.approx(math.sin(half))
    assert message.pose.pose.orientation.w == pytest.approx(math.cos(half))
    assert message.pose.covariance[0] == pytest.approx(0.10**2)
    assert message.pose.covariance[7] == pytest.approx(0.25**2)
    assert message.pose.covariance[35] == pytest.approx(math.radians(5.0) ** 2)
    off_diagonal = [
        value
        for index, value in enumerate(message.pose.covariance)
        if index not in (0, 7, 35)
    ]
    assert off_diagonal == [0.0] * 33


def test_default_std_reproduces_amcl_seed_covariance():
    node, publisher = _fake_node()

    OutdoorInitialPose._tick(node)

    covariance = publisher.messages[0].pose.covariance
    assert covariance[0] == pytest.approx(0.01)
    assert covariance[7] == pytest.approx(0.01)
    assert covariance[35] == pytest.approx(math.radians(5.0) ** 2)


def test_tick_waits_for_clock_and_scan_and_stops_after_publish_count():
    node, publisher = _fake_node(publish_count=2)
    node.clock_ready = False
    node.scan_ready = False

    OutdoorInitialPose._tick(node)
    assert publisher.messages == []

    node.clock_ready = True
    OutdoorInitialPose._tick(node)
    assert publisher.messages == []

    node.scan_ready = True
    OutdoorInitialPose._tick(node)
    OutdoorInitialPose._tick(node)
    OutdoorInitialPose._tick(node)
    assert len(publisher.messages) == 2
    assert node.published == 2


def test_reset_rearms_a_limited_republish():
    node, publisher = _fake_node(publish_count=1)

    OutdoorInitialPose._tick(node)
    assert len(publisher.messages) == 1

    OutdoorInitialPose._on_reset(node, SimpleNamespace())
    assert not node.scan_ready
    assert node.published == 0

    OutdoorInitialPose._tick(node)
    assert len(publisher.messages) == 1

    node.scan_ready = True
    OutdoorInitialPose._tick(node)
    assert len(publisher.messages) == 2
    assert node.published == 1
