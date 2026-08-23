from pathlib import Path
from threading import Lock
from types import SimpleNamespace

from geometry_msgs.msg import PoseWithCovarianceStamped
from isaac_ros_pointcloud_interfaces.msg import FlatScan
from rclpy.time import Time
from robot_grid_localization.core import LocalizationGate, RigidTransform
from robot_grid_localization.grid_localization_tf_manager import (
    GridLocalizationTFManager,
)
from tf2_ros import Buffer


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_manager_freezes_standard_ros_interfaces_and_tf_ownership():
    source = (
        PACKAGE_ROOT / 'robot_grid_localization'
        / 'grid_localization_tf_manager.py').read_text()
    assert "'/localization_result'" in source
    assert "'/bio_nav/localization_pose'" in source
    assert "'/bio_nav/localization/status'" in source
    assert "'/bio_nav/relocalize'" in source
    assert "'/flatscan'" in source
    assert "'/flatscan_localization'" in source
    assert "'/trigger_grid_search_localization'" not in source
    assert 'from std_srvs.srv import Trigger' in source
    assert 'Empty' not in source
    assert 'vendor_qos = QoSProfile(depth=10)' in source
    assert "transform.child_frame_id = 'odom'" in source
    assert source.count('TransformBroadcaster(self)') == 1
    assert 'StaticTransformBroadcaster' not in source
    assert 'sendTransform(transform)' in source
    assert "declare_parameter('tf_broadcast_rate_hz', 20.0)" in source
    assert 'self._latest_correction = correction' in source
    assert 'correction, Time.from_msg(message.header.stamp)' in source
    assert 'self._broadcast_correction(correction, received_at)' in source
    assert 'self._broadcast_correction(self._latest_correction, now)' in source


def test_manager_uses_exact_result_stamp_for_odom_to_base_lookup():
    source = (
        PACKAGE_ROOT / 'robot_grid_localization'
        / 'grid_localization_tf_manager.py').read_text()
    assert "'odom', 'base_link'" in source
    assert 'Time.from_msg(message.header.stamp)' in source
    assert 'Time()' not in source
    assert 'MultiThreadedExecutor(num_threads=2)' in source


def test_pending_timeout_is_parameterized_and_publishes_terminal_state():
    source = (
        PACKAGE_ROOT / 'robot_grid_localization'
        / 'grid_localization_tf_manager.py').read_text()
    assert "declare_parameter('pending_timeout_s', 10.0)" in source
    assert 'expire_pending(' in source
    assert "timeout_decision, 'REJECTED', None, now" in source


def test_public_trigger_waits_for_scan_without_vendor_service_proxy():
    manager = object.__new__(GridLocalizationTFManager)
    manager._gate = LocalizationGate()
    manager._gate_lock = Lock()
    manager.get_clock = lambda: SimpleNamespace(
        now=lambda: Time(nanoseconds=100))
    statuses = []
    manager._publish_status = lambda *args: statuses.append(args)
    response = SimpleNamespace(success=None, message='')

    returned = manager._on_relocalize(None, response)

    assert returned is response
    assert response.success is True
    assert 'generation=1' in response.message
    assert manager._gate.pending_generation == 1
    assert statuses[0][1] == 'WAITING_FOR_SCAN'


def test_manager_forwards_selected_scan_exactly_once_without_header_change():
    class RecordingPublisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    manager = object.__new__(GridLocalizationTFManager)
    manager._gate = LocalizationGate()
    manager._gate.begin_trigger(100)
    manager._gate_lock = Lock()
    manager.get_clock = lambda: SimpleNamespace(
        now=lambda: Time(nanoseconds=150))
    manager._publish_status = lambda *_args: None
    manager._flat_scan_trigger_publisher = RecordingPublisher()
    first = FlatScan()
    first.header.stamp.nanosec = 200
    first.header.frame_id = 'lidar'
    second = FlatScan()
    second.header.stamp.nanosec = 201

    manager._on_flat_scan(first)
    manager._on_flat_scan(second)

    assert manager._flat_scan_trigger_publisher.messages == [first]
    assert first.header.stamp.nanosec == 200
    assert first.header.frame_id == 'lidar'
    assert manager._gate.expected_result_stamp_ns == 200


def test_manager_ignores_unexpected_result_without_relabelling_generation():
    manager = object.__new__(GridLocalizationTFManager)
    manager._gate = LocalizationGate()
    manager._gate.begin_trigger(100)
    manager._gate.observe_scan(200)
    manager._gate_lock = Lock()
    statuses = []
    manager._publish_status = lambda *args: statuses.append(args)
    manager.get_logger = lambda: SimpleNamespace(debug=lambda *_args: None)
    message = PoseWithCovarianceStamped()
    message.header.stamp.nanosec = 199

    manager._on_localization_result(message)

    assert statuses == []
    assert manager._gate.pending_generation == 1
    assert manager._gate.expected_result_stamp_ns == 200


def test_duplicate_trigger_preserves_the_active_generation_and_state():
    manager = object.__new__(GridLocalizationTFManager)
    manager._gate = LocalizationGate()
    manager._gate.begin_trigger(100)
    manager._gate_lock = Lock()
    response = SimpleNamespace(success=None, message='')

    returned = manager._on_relocalize(None, response)

    assert returned is response
    assert response.success is False
    assert 'generation=1' in response.message
    assert manager._gate.pending_generation == 1
    assert manager._gate.trigger_stamp_ns == 100


def test_periodic_current_stamp_tf_supports_one_ms_future_lookup():
    buffer = Buffer()

    class BufferingBroadcaster:
        def sendTransform(self, transform):
            buffer.set_transform(transform, 'manager_regression_test')

    manager = object.__new__(GridLocalizationTFManager)
    manager._tf_broadcaster = BufferingBroadcaster()
    correction = RigidTransform(
        1.25, -0.5, 0.0, 0.0, 0.0, 0.0, 1.0)
    result_stamp_ns = 1_000_000_000
    accepted_at_ns = result_stamp_ns + 50_000_000

    manager._broadcast_correction(
        correction, Time(nanoseconds=result_stamp_ns))
    manager._broadcast_correction(
        correction, Time(nanoseconds=accepted_at_ns))

    future = buffer.lookup_transform(
        'map', 'odom', Time(nanoseconds=result_stamp_ns + 1_000_000))
    assert future.transform.translation.x == 1.25
    assert future.transform.translation.y == -0.5
    assert future.header.stamp.nanosec == 1_000_000
