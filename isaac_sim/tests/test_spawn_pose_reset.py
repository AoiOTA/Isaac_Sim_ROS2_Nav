from __future__ import annotations

import asyncio
from dataclasses import dataclass, field, replace
from pathlib import Path
from types import SimpleNamespace

import pytest

from isaac_sim.src.bridge.reset_service import (
    InitialPoseRepublisher,
    ResetServiceBridge,
    ResetServiceError,
    _ResetTransaction,
)
from isaac_sim.src.robot.reset import ResetError, ResetHooks, ResetManager, ResetRequest
from isaac_sim.src.robot.spawn_pose_manager import (
    SpawnPoseError,
    SpawnPoseManager,
    load_spawn_poses,
)


ROOT = Path(__file__).resolve().parents[2]


@dataclass
class FakeRobot:
    num_dof: int = 6
    calls: list[tuple[str, object]] = field(default_factory=list)

    def set_world_pose(self, position, orientation_wxyz):
        self.calls.append(("pose", (tuple(position), tuple(orientation_wxyz))))

    def set_base_velocities(self, linear, angular):
        self.calls.append(("base_velocity", (tuple(linear), tuple(angular))))

    def set_joint_velocities(self, values):
        self.calls.append(("joint_velocity", tuple(values)))

    def set_joint_velocity_targets(self, values):
        self.calls.append(("joint_target", tuple(values)))


@dataclass
class FakeSimulation:
    calls: list[str] = field(default_factory=list)

    def pause(self):
        self.calls.append("pause")

    def step(self, *, render):
        self.calls.append(f"step:{render}")

    def play(self):
        self.calls.append("play")


def test_spawn_reset_zeros_all_robot_state_and_gates_map_pose():
    robot = FakeRobot()
    poses = load_spawn_poses(ROOT / "isaac_sim/configs/spawn_poses.yaml")
    manager = SpawnPoseManager(robot, poses)
    manager.apply_usd_pose("mapping_start")
    assert robot.calls[0][0] == "pose"
    assert robot.calls[-2:] == [
        ("joint_velocity", (0.0,) * 6),
        ("joint_target", (0.0,) * 6),
    ]
    map_pose = manager.get_map_pose("mapping_start", purpose="test localization")
    assert map_pose.position_stddev_m == pytest.approx(0.05)
    assert map_pose.yaw_stddev_deg == pytest.approx(5.0)

    uncalibrated = replace(
        poses["mapping_start"],
        map=replace(poses["mapping_start"].map, calibrated=False),
    )
    gated_manager = SpawnPoseManager(robot, {"mapping_start": uncalibrated})
    with pytest.raises(SpawnPoseError, match="calibrated=false"):
        gated_manager.get_map_pose("mapping_start", purpose="test localization")


def test_reset_manager_executes_every_hook_in_fixed_order():
    robot = FakeRobot()
    spawn = SpawnPoseManager(
        robot, load_spawn_poses(ROOT / "isaac_sim/configs/spawn_poses.yaml")
    )
    simulation = FakeSimulation()
    calls: list[str] = []
    hooks = ResetHooks(
        send_zero_velocity=lambda: calls.append("zero"),
        clear_controller_state=lambda: calls.append("controller"),
        reset_odometry=lambda mode: calls.append(f"odom:{mode}"),
        reset_ground_truth_path=lambda: calls.append("gt"),
        reset_dynamic_obstacles=lambda seed: calls.append(f"dynamic:{seed}"),
        clear_costmaps=lambda: calls.append("costmaps"),
        publish_map_initial_pose=lambda pose: calls.append(f"initial:{pose}"),
    )
    ResetManager(simulation, spawn, hooks).reset(
        ResetRequest("mapping_start", "mapping", "ideal", 123)
    )
    assert simulation.calls == ["pause", "step:False", "play"]
    assert calls == ["zero", "controller", "odom:ideal", "gt", "dynamic:123", "costmaps"]


def test_invalid_reset_request_is_rejected():
    with pytest.raises(ResetError, match="non-negative"):
        ResetRequest("mapping_start", "mapping", "ideal", -1)


def test_reset_manager_resumes_timeline_when_a_hook_fails():
    robot = FakeRobot()
    spawn = SpawnPoseManager(
        robot, load_spawn_poses(ROOT / "isaac_sim/configs/spawn_poses.yaml")
    )
    simulation = FakeSimulation()
    hooks = ResetHooks(
        send_zero_velocity=lambda: (_ for _ in ()).throw(RuntimeError("boom")),
        clear_controller_state=lambda: None,
        reset_odometry=lambda mode: None,
        reset_ground_truth_path=lambda: None,
        reset_dynamic_obstacles=lambda seed: None,
        clear_costmaps=lambda: None,
        publish_map_initial_pose=lambda pose: None,
    )

    with pytest.raises(ResetError, match="boom"):
        ResetManager(simulation, spawn, hooks).reset(
            ResetRequest("mapping_start", "mapping", "ideal", 0)
        )

    assert simulation.calls == ["pause", "play"]
    assert robot.calls == []


def test_localization_pose_is_validated_before_timeline_or_robot_changes():
    robot = FakeRobot()
    poses = load_spawn_poses(ROOT / "isaac_sim/configs/spawn_poses.yaml")
    uncalibrated = replace(
        poses["mapping_start"],
        map=replace(poses["mapping_start"].map, calibrated=False),
    )
    spawn = SpawnPoseManager(robot, {"mapping_start": uncalibrated})
    simulation = FakeSimulation()
    hooks = ResetHooks(
        send_zero_velocity=lambda: None,
        clear_controller_state=lambda: None,
        reset_odometry=lambda mode: None,
        reset_ground_truth_path=lambda: None,
        reset_dynamic_obstacles=lambda seed: None,
        clear_costmaps=lambda: None,
        publish_map_initial_pose=lambda pose: None,
    )

    with pytest.raises(SpawnPoseError, match="calibrated=false"):
        ResetManager(simulation, spawn, hooks).reset(
            ResetRequest("mapping_start", "localization", "ideal", 0)
        )

    assert simulation.calls == []
    assert robot.calls == []


def test_initial_pose_republisher_waits_for_post_reset_scans():
    published: list[str] = []
    republisher = InitialPoseRepublisher(
        published.append,
        publish_count=3,
    )

    republisher.schedule("mapping_start", after_stamp_s=10.0)
    assert republisher.pending
    republisher.observe_scan(9.9)
    republisher.observe_scan(10.0)
    assert published == []

    republisher.observe_scan(10.1)
    republisher.observe_scan(10.1)
    republisher.observe_scan(10.2)
    republisher.observe_scan(10.3)
    assert published == ["mapping_start"] * 3
    assert not republisher.pending


def test_initial_pose_republisher_replaces_pending_reset_and_validates_inputs():
    published: list[str] = []
    republisher = InitialPoseRepublisher(
        published.append,
        publish_count=1,
    )
    republisher.schedule("old_pose", after_stamp_s=1.0)
    republisher.schedule("mapping_start", after_stamp_s=2.0)
    republisher.observe_scan(2.1)
    assert published == ["mapping_start"]

    with pytest.raises(ResetServiceError, match="pose name"):
        republisher.schedule("", after_stamp_s=1.0)
    with pytest.raises(ResetServiceError, match="scan barrier"):
        republisher.schedule("mapping_start", after_stamp_s=float("nan"))
    with pytest.raises(ResetServiceError, match="scan stamp"):
        republisher.observe_scan(float("nan"))
    with pytest.raises(ResetServiceError, match="simulation clock stamp"):
        republisher.observe_clock(float("nan"))
    with pytest.raises(ResetServiceError, match="publish_count"):
        InitialPoseRepublisher(published.append, publish_count=0)
    with pytest.raises(ResetServiceError, match="clock_scan_tolerance"):
        InitialPoseRepublisher(
            published.append,
            clock_scan_tolerance_s=0.0,
        )


def test_initial_pose_republisher_uses_clock_evidence_across_epoch_rollback():
    published: list[str] = []
    republisher = InitialPoseRepublisher(
        published.append,
        publish_count=1,
        clock_scan_tolerance_s=0.5,
    )

    republisher.schedule("mapping_start", after_stamp_s=30.0)
    # Delayed scans alone are not evidence that simulation time rolled back.
    republisher.observe_scan(28.0, clock_stamp_s=30.0)
    republisher.observe_scan(28.1, clock_stamp_s=30.1)
    assert published == []
    assert republisher.pending

    # The authoritative simulation clock establishes the new epoch.  A stale
    # high-stamp scan is still rejected after that rollback.
    republisher.observe_clock(0.1)
    republisher.observe_scan(28.2, clock_stamp_s=0.1)
    republisher.observe_scan(0.1, clock_stamp_s=0.1)
    assert published == []

    republisher.observe_scan(0.2, clock_stamp_s=0.2)
    assert published == ["mapping_start"]
    assert not republisher.pending


class FakeFuture:
    def __init__(self):
        self._done = False
        self._error = None
        self._callbacks = []

    def add_done_callback(self, callback):
        self._callbacks.append(callback)

    def done(self):
        return self._done

    def result(self):
        if self._error is not None:
            raise self._error
        return object()

    def complete(self, error=None):
        self._done = True
        self._error = error
        for callback in list(self._callbacks):
            callback(self)


class FakeCompletion(FakeFuture):
    def __init__(self, *, executor=None):
        super().__init__()
        del executor
        self.value = None

    def set_result(self, value):
        self.value = value
        self.complete()


def test_reset_transaction_finishes_only_after_every_async_call():
    order = []
    completion = FakeCompletion()
    transaction = _ResetTransaction(
        generation=7,
        completion=completion,
        on_finished=lambda tx: order.append("reset_event"),
    )
    wheel = FakeFuture()
    costmap = FakeFuture()
    transaction.add_call("wheel", wheel)
    transaction.add_call("costmap", costmap)
    transaction.seal()

    assert not transaction.finished
    assert not completion.done()
    assert order == []
    wheel.complete()
    assert not transaction.finished
    costmap.complete()

    assert transaction.finished
    assert completion.done()
    assert order == ["reset_event"]
    assert transaction.errors == []


def test_reset_transaction_reports_failure_after_calls_resolve():
    order = []
    transaction = _ResetTransaction(
        generation=8,
        completion=FakeCompletion(),
        on_finished=lambda tx: order.append("reset_event"),
    )
    ekf = FakeFuture()
    transaction.add_call("EKF", ekf)
    transaction.seal()
    ekf.complete(RuntimeError("set pose rejected"))

    assert transaction.finished
    assert order == ["reset_event"]
    assert transaction.errors == ["EKF: RuntimeError: set pose rejected"]


class FakeTimer:
    def __init__(self):
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


class FakePublisher:
    def __init__(self, events):
        self.events = events

    def publish(self, message):
        del message
        self.events.append("reset_event")


def test_timeout_failure_does_not_emit_reset_event_or_arm_initial_pose():
    events = []
    cancelled = []
    bridge = SimpleNamespace(
        _active_transaction=None,
        _reset_event_publisher=FakePublisher(events),
        _EmptyMessage=lambda: object(),
        _initial_pose_republisher=SimpleNamespace(
            cancel=lambda: cancelled.append("cancel")
        ),
        _deferred_initial_pose_name="old",
        _apply_initial_pose_policy=lambda: events.append("arm"),
    )
    transaction = _ResetTransaction(
        generation=9,
        completion=FakeCompletion(),
        on_finished=lambda tx: ResetServiceBridge._finish_transaction(bridge, tx),
        initial_pose_name="mapping_start",
    )
    bridge._active_transaction = transaction
    transaction.timeout_timer = FakeTimer()
    pending = FakeFuture()
    transaction.add_call("wheel", pending)
    transaction.seal()
    transaction.timeout(0.25)

    assert transaction.finished
    assert transaction.timed_out
    assert events == []
    assert cancelled == ["cancel"]
    assert bridge._deferred_initial_pose_name is None
    assert transaction.timeout_timer.cancelled


def test_successful_transaction_emits_event_before_initial_pose_policy():
    events = []
    bridge = SimpleNamespace(
        _active_transaction=None,
        _reset_event_publisher=FakePublisher(events),
        _EmptyMessage=lambda: object(),
        _initial_pose_republisher=SimpleNamespace(cancel=lambda: None),
        _deferred_initial_pose_name=None,
        _apply_initial_pose_policy=lambda: events.append("arm"),
    )
    transaction = _ResetTransaction(
        generation=10,
        completion=FakeCompletion(),
        on_finished=lambda tx: ResetServiceBridge._finish_transaction(bridge, tx),
        initial_pose_name="mapping_start",
    )
    bridge._active_transaction = transaction
    transaction.timeout_timer = FakeTimer()
    transaction.seal()

    assert events == ["reset_event", "arm"]
    assert bridge._deferred_initial_pose_name == "mapping_start"


def test_startup_reset_returns_pending_transaction_without_blocking():
    pending = FakeFuture()
    holder = {}

    class FakeManager:
        def reset(self, request):
            del request
            holder["bridge"]._active_transaction.add_call("wheel", pending)

    bridge = SimpleNamespace(
        _manager=FakeManager(),
        _active_transaction=None,
        _pending_futures=set(),
        _initial_pose_republisher=SimpleNamespace(cancel=lambda: None),
        _deferred_initial_pose_name=None,
        _transaction_generation=0,
        _Future=FakeCompletion,
        node=SimpleNamespace(
            executor=None,
            create_timer=lambda *args, **kwargs: FakeTimer(),
        ),
        _transaction_timeout_sec=1.0,
        _callback_group=object(),
        _steady_clock=object(),
        _finish_transaction=lambda tx: None,
    )
    holder["bridge"] = bridge

    transaction = ResetServiceBridge.start_reset(
        bridge,
        ResetRequest("mapping_start", "mapping", "ideal", 0),
    )

    assert transaction.sealed
    assert not transaction.finished
    assert not transaction.completion.done()
    pending.complete()
    assert transaction.finished


def test_repeated_reset_is_rejected_while_generation_is_active():
    bridge = SimpleNamespace(
        _manager=object(),
        _active_transaction=SimpleNamespace(generation=11),
    )
    with pytest.raises(ResetServiceError, match="generation=11"):
        ResetServiceBridge.start_reset(
            bridge,
            ResetRequest("mapping_start", "mapping", "ideal", 0),
        )


class AwaitMarker:
    def __init__(self, events):
        self.events = events

    def __await__(self):
        async def wait():
            self.events.append("futures_complete")
            return None

        return wait().__await__()


class FakeLogger:
    def __init__(self, events):
        self.events = events

    def info(self, message):
        del message
        self.events.append("response_success")

    def error(self, message):
        del message
        self.events.append("response_failure")


@pytest.mark.parametrize("errors", [[], ["EKF: failed"]])
def test_reset_service_response_waits_for_transaction_completion(errors):
    events = []
    request_value = ResetRequest("mapping_start", "mapping", "ideal", 0)
    transaction = SimpleNamespace(
        completion=AwaitMarker(events),
        errors=errors,
        skipped=[],
        generation=12,
    )
    bridge = SimpleNamespace(
        _manager=object(),
        _read_request=lambda: request_value,
        start_reset=lambda request: transaction,
        node=SimpleNamespace(get_logger=lambda: FakeLogger(events)),
    )
    response = SimpleNamespace(success=None, message="")

    result = asyncio.run(
        ResetServiceBridge._reset_callback(bridge, object(), response)
    )

    assert result is response
    assert events[0] == "futures_complete"
    assert response.success is (not errors)
    assert events[1] == (
        "response_failure" if errors else "response_success"
    )
