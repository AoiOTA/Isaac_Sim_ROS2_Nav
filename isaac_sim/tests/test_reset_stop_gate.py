from __future__ import annotations

import os
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import pytest

from isaac_sim.src.bridge.reset_stop_gate import (
    ResetStopGate,
    ResetStopGateError,
    ResetStopGateState,
)


ROOT = Path(__file__).resolve().parents[2]


def test_hold_release_requires_current_completed_generation():
    state = ResetStopGateState()
    first = state.hold()
    assert state.held
    with pytest.raises(ResetStopGateError, match="release rejected"):
        state.release(first)

    state.mark_reset_complete(first)
    state.release(first)
    assert not state.held


def test_new_reset_rejects_stale_release_and_stays_held_on_failure():
    state = ResetStopGateState()
    first = state.hold()
    state.mark_reset_complete(first)
    second = state.hold()

    with pytest.raises(ResetStopGateError, match="release rejected"):
        state.release(first)
    assert state.held
    assert state.eligible_generation is None

    state.mark_reset_complete(second)
    state.release(second)
    assert not state.held


def test_release_never_carries_command_state_between_epochs():
    state = ResetStopGateState()
    generation = state.hold()
    assert vars(state) == {
        "generation": generation,
        "held": True,
        "eligible_generation": None,
    }
    state.mark_reset_complete(generation)
    state.release(generation)
    assert vars(state) == {
        "generation": generation,
        "held": False,
        "eligible_generation": None,
    }


def test_release_status_publication_failure_leaves_live_gate_held():
    state = ResetStopGateState()
    generation = state.hold()
    state.mark_reset_complete(generation)
    published_zero = []

    def fail_status(reason, *, state=None):
        del reason, state
        raise RuntimeError("status publisher failed")

    gate = SimpleNamespace(
        state=state,
        _lock=threading.RLock(),
        _publish_lock=threading.RLock(),
        _closed=False,
        publish_zero=lambda: published_zero.append(True),
        _publish_status=fail_status,
    )
    with pytest.raises(RuntimeError, match="status publisher failed"):
        ResetStopGate.release(gate, generation, source="test")

    assert state.held
    assert state.eligible_generation == generation
    assert published_zero == [True]


def test_completion_status_publication_failure_does_not_make_gate_eligible():
    state = ResetStopGateState()
    generation = state.hold()
    gate = SimpleNamespace(
        state=state,
        _lock=threading.RLock(),
        _publish_lock=threading.RLock(),
        _closed=False,
        publish_zero=lambda: None,
        _publish_status=lambda reason, state=None: (_ for _ in ()).throw(
            RuntimeError("completion status publisher failed")
        ),
    )
    with pytest.raises(RuntimeError, match="completion status publisher failed"):
        ResetStopGate.mark_reset_complete(gate, generation)

    assert state.held
    assert state.eligible_generation is None


class _HeartbeatPublisher:
    def __init__(self):
        self.messages = []
        self.lock = threading.Lock()

    def publish(self, message):
        with self.lock:
            self.messages.append((time.monotonic(), message))

    def count(self):
        with self.lock:
            return len(self.messages)

    def times(self):
        with self.lock:
            return [stamp for stamp, _message in self.messages]


def _heartbeat_gate(*, period_s=0.02):
    publisher = _HeartbeatPublisher()
    destroyed = []
    node = SimpleNamespace(
        remove_on_set_parameters_callback=lambda callback: destroyed.append(
            ("callback", callback)
        ),
        destroy_subscription=lambda subscription: destroyed.append(
            ("subscription", subscription)
        ),
        destroy_publisher=lambda item: destroyed.append(("publisher", item)),
    )
    gate = ResetStopGate.__new__(ResetStopGate)
    gate.node = node
    gate.state = ResetStopGateState(generation=2, held=True)
    gate._lock = threading.RLock()
    gate._publish_lock = threading.RLock()
    gate._heartbeat_stop = threading.Event()
    gate._heartbeat_failure = None
    gate._closed = False
    gate._heartbeat_period_s = period_s
    gate._Twist = SimpleNamespace
    gate._publisher = publisher
    gate._status_publisher = _HeartbeatPublisher()
    gate._subscription = object()
    gate._parameter_callback = object()
    gate._String = lambda: SimpleNamespace(data="")
    gate._heartbeat_thread = threading.Thread(
        target=gate._zero_heartbeat, daemon=True
    )
    gate._heartbeat_thread.start()
    return gate, publisher, destroyed


def test_wall_heartbeat_runs_without_executor_spin_and_release_stops_zeros():
    gate, publisher, _destroyed = _heartbeat_gate()
    deadline = time.monotonic() + 0.25
    while publisher.count() < 4 and time.monotonic() < deadline:
        time.sleep(0.005)

    times = publisher.times()
    assert len(times) >= 4
    assert max(b - a for a, b in zip(times, times[1:])) <= 0.08

    gate.mark_reset_complete(2)
    gate.release(2, source="test")
    released_count = publisher.count()
    time.sleep(0.08)
    assert publisher.count() == released_count

    message = SimpleNamespace(linear=SimpleNamespace(x=1.0))
    gate._command_callback(message)
    assert publisher.count() == released_count + 1
    assert publisher.messages[-1][1] is message
    gate.close()


def test_heartbeat_stale_generation_close_and_resource_ordering():
    gate, publisher, destroyed = _heartbeat_gate()
    with pytest.raises(ResetStopGateError, match="release rejected"):
        gate.release(1, source="stale")
    assert gate.state.held is True

    gate.close()
    assert not gate._heartbeat_thread.is_alive()
    closed_count = publisher.count()
    time.sleep(0.05)
    assert publisher.count() == closed_count
    assert [kind for kind, _item in destroyed] == [
        "callback", "subscription", "publisher", "publisher"
    ]
    gate.close()


def test_relay_publish_exception_returns_gate_to_observable_hold():
    gate, publisher, _destroyed = _heartbeat_gate()
    gate.mark_reset_complete(2)
    gate.release(2, source="test")
    publish = publisher.publish

    def fail_publish(_message):
        raise RuntimeError("relay publish injected")

    publisher.publish = fail_publish
    with pytest.raises(RuntimeError, match="relay publish injected"):
        gate._command_callback(SimpleNamespace())

    assert gate.state.held is True
    assert gate.state.eligible_generation is None
    assert gate._heartbeat_failure.startswith("command:RuntimeError:")
    publisher.publish = publish
    gate.close()


def test_command_reader_keep_last_one_overwrites_unspun_stale_train():
    """A terminal zero replaces queued stale commands before Isaac spins."""
    rclpy = pytest.importorskip("rclpy")
    from geometry_msgs.msg import Twist
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSCompatibility,
        QoSProfile,
        ReliabilityPolicy,
        qos_check_compatible,
    )

    context = Context()
    rclpy.init(context=context)
    suffix = f"{os.getpid()}_{time.monotonic_ns()}"
    input_topic = f"/test_reset_stop_gate_{suffix}/input"
    output_topic = f"/test_reset_stop_gate_{suffix}/output"
    gate_node = rclpy.create_node(f"reset_stop_gate_{suffix}", context=context)
    source_node = rclpy.create_node(f"reset_stop_source_{suffix}", context=context)
    sink_node = rclpy.create_node(f"reset_stop_sink_{suffix}", context=context)
    offered = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    observed = []
    gate = None
    gate_executor = SingleThreadedExecutor(context=context)
    sink_executor = SingleThreadedExecutor(context=context)
    gate_executor.add_node(gate_node)
    sink_executor.add_node(sink_node)
    try:
        gate = ResetStopGate(
            gate_node,
            input_topic=input_topic,
            output_topic=output_topic,
            zero_rate_hz=20.0,
        )
        source = source_node.create_publisher(Twist, input_topic, offered)
        sink_node.create_subscription(
            Twist,
            output_topic,
            lambda message: observed.append(
                (float(message.linear.x), float(message.angular.z))
            ),
            offered,
        )
        deadline = time.monotonic() + 2.0
        while (
            source_node.count_subscribers(input_topic) < 1
            or gate_node.count_publishers(input_topic) < 1
            or gate_node.count_subscribers(output_topic) < 1
        ) and time.monotonic() < deadline:
            time.sleep(0.01)
        assert source_node.count_subscribers(input_topic) == 1
        command_qos = gate._subscription.qos_profile
        assert command_qos.history == HistoryPolicy.KEEP_LAST
        assert command_qos.depth == 1
        assert command_qos.reliability == ReliabilityPolicy.RELIABLE
        assert command_qos.durability == DurabilityPolicy.VOLATILE
        compatibility, _reason = qos_check_compatible(
            offered, command_qos
        )
        assert compatibility != QoSCompatibility.ERROR

        gate.mark_reset_complete(0)
        gate.release(0, source="test")
        for _index in range(8):
            sink_executor.spin_once(timeout_sec=0.01)
        observed.clear()

        nonzero = Twist()
        nonzero.linear.x = 0.25
        nonzero.angular.z = 0.4
        for _index in range(32):
            source.publish(nonzero)
        source.publish(Twist())
        # DDS receives while the app deliberately does not spin gate_node.
        time.sleep(0.15)
        for _index in range(8):
            gate_executor.spin_once(timeout_sec=0.01)
        for _index in range(8):
            sink_executor.spin_once(timeout_sec=0.01)

        assert observed
        assert all(linear == 0.0 and angular == 0.0 for linear, angular in observed)
    finally:
        if gate is not None:
            gate.close()
        gate_executor.shutdown()
        sink_executor.shutdown()
        source_node.destroy_node()
        sink_node.destroy_node()
        gate_node.destroy_node()
        rclpy.shutdown(context=context)


def test_navigation_profile_has_one_final_external_command_authority():
    reset_source = (
        ROOT / "isaac_sim/src/bridge/reset_service.py"
    ).read_text(encoding="utf-8")
    gate_source = (
        ROOT / "isaac_sim/src/bridge/reset_stop_gate.py"
    ).read_text(encoding="utf-8")
    topics = (
        ROOT / "isaac_sim/configs/ros2_bridge/topics.yaml"
    ).read_text(encoding="utf-8")

    assert 'create_publisher(Twist, "/cmd_vel"' not in reset_source
    assert 'input_topic: str = "/cmd_vel"' in gate_source
    assert 'output_topic: str = "/cmd_vel_sim"' in gate_source
    assert "cmd_vel: /cmd_vel_sim" in topics
    navigation_source = (
        ROOT / "isaac_sim/apps/navigation_sim.py"
    ).read_text(encoding="utf-8")
    assert 'Twist, "/cmd_vel_diagnostic", 1' in navigation_source
    assert 'create_publisher(Twist, "/cmd_vel", 1)' not in navigation_source


def test_startup_and_trigger_resets_share_one_release_owner_contract():
    navigation_source = (
        ROOT / "isaac_sim/apps/navigation_sim.py"
    ).read_text(encoding="utf-8")
    bridge_source = (
        ROOT / "isaac_sim/src/bridge/reset_service.py"
    ).read_text(encoding="utf-8")

    assert "startup_stop_released" not in navigation_source
    assert "startup_reset_complete" not in navigation_source
    assert "external_recovery_release_required=(" in navigation_source
    assert (
        'config.simulation.navigation_mode == "localization"'
        in navigation_source
    )
    assert "and not diagnostic_command_mode" in navigation_source
    assert 'source="reset_transaction_complete"' in bridge_source
