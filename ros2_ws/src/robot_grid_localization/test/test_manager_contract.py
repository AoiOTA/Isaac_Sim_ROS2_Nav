from pathlib import Path
from threading import Lock
from types import SimpleNamespace

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
    assert "'/trigger_grid_search_localization'" in source
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
