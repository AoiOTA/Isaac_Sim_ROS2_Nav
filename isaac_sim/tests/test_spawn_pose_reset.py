from __future__ import annotations

import asyncio
from dataclasses import dataclass, field, replace
from hashlib import sha256
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


def test_v6_clearance_r2_spawn_identity_keeps_calibrated_map_frame():
    poses = load_spawn_poses(
        ROOT
        / "isaac_sim/configs/environments/"
        "kujiale_0026_A_to_B_door_open.v6_clearance_r2.spawn.yaml"
    )
    assert set(poses) == {
        "mapping_start",
        "long_route_start_g1",
        "long_route_start_g2",
        "long_route_start_g5",
    }
    assert all(
        pose.map.map_version == "v6_kujiale_clearance_r2"
        and pose.map.calibrated
        for pose in poses.values()
    )
    assert poses["long_route_start_g1"].map.position == pytest.approx(
        (0.45, -5.35)
    )
    digest = sha256()
    for suffix in ("pgm", "yaml"):
        digest.update(
            (ROOT / f"data/maps/occupancy/v6_kujiale_clearance_r2.{suffix}")
            .read_bytes()
        )
    assert all(
        pose.map.map_bundle_sha256 == digest.hexdigest()
        for pose in poses.values()
    )


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
    assert map_pose.map_version == "warehouse_v1"
    assert map_pose.map_bundle_sha256 == (
        "88b91be7fb0afe4364851c59dc3466f560017df5acc5405f3ab590729ded9bac"
    )

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
    assert robot.calls[0][0] == "pose"


def test_invalid_reset_request_is_rejected():
    assert ResetRequest(
        "mapping_start", "localization", "mixed", 0
    ).odometry_mode == "mixed"
    with pytest.raises(ResetError, match="non-negative"):
        ResetRequest("mapping_start", "mapping", "ideal", -1)


def test_mixed_reset_queues_wheel_and_module1_ekf_before_epoch_completion():
    queued = []

    class EmptyService:
        class Request:
            pass

    class SetPose:
        class Request:
            def __init__(self):
                pose = SimpleNamespace(
                    pose=SimpleNamespace(
                        pose=SimpleNamespace(
                            orientation=SimpleNamespace(w=0.0)
                        ),
                        covariance=[0.0] * 36,
                    ),
                    header=SimpleNamespace(stamp=None, frame_id=""),
                )
                self.pose = pose

    bridge = SimpleNamespace(
        _wheel_reset_client=object(),
        _ekf_set_pose_client=object(),
        _EmptyService=EmptyService,
        _SetPose=SetPose,
        node=SimpleNamespace(
            get_clock=lambda: SimpleNamespace(
                now=lambda: SimpleNamespace(to_msg=lambda: "stamp")
            )
        ),
        _queue_service_call=lambda client, request, label: queued.append(
            (client, request, label)
        ),
    )

    ResetServiceBridge.reset_ros_odometry(bridge, "mixed")

    assert [label for _, _, label in queued] == ["wheel odometry", "EKF"]
    set_pose = queued[1][1]
    assert set_pose.pose.header.frame_id == "odom"
    assert set_pose.pose.pose.pose.orientation.w == 1.0

    app_source = (
        ROOT / "isaac_sim/apps/navigation_sim.py"
    ).read_text(encoding="utf-8")
    reset_block = app_source[
        app_source.index("        def reset_odometry(mode: str)"):
        app_source.index("        def reset_ground_truth_path()")
    ]
    assert reset_block.index("reset_bridge.reset_ros_odometry(mode)") < (
        reset_block.index("previous.retire()")
    )
    assert reset_block.index("previous.retire()") < reset_block.index(
        "build_odometry_graph("
    )


def test_localization_reset_pose_is_immutable_manifest_bound_profile():
    parameters = {
        "reset_seed": SimpleNamespace(value=0),
        "reset_pose_name": SimpleNamespace(value="other_map_pose"),
        "navigation_mode": SimpleNamespace(value="localization"),
        "odometry_mode": SimpleNamespace(value="ideal"),
        "dynamic_case_id": SimpleNamespace(value=""),
        "dynamic_variant_id": SimpleNamespace(value=""),
    }
    bridge = SimpleNamespace(
        node=SimpleNamespace(get_parameter=parameters.__getitem__),
        _configured_navigation_mode="localization",
        _configured_odometry_mode="ideal",
        _default_pose_name="mapping_start",
    )

    with pytest.raises(ResetServiceError, match="bound to the map manifest"):
        ResetServiceBridge._read_request(bridge)

    parameters["reset_pose_name"].value = "mapping_start"
    request = ResetServiceBridge._read_request(bridge)
    assert request.pose_name == "mapping_start"


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


class CancellableFakeFuture(FakeFuture):
    def __init__(self):
        super().__init__()
        self.cancelled = False

    def cancel(self):
        self.cancelled = True
        self.complete()
        return True


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


class FailingPublisher:
    def publish(self, message):
        del message
        raise RuntimeError("publisher rejected reset event")


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


class RecordingStopGate:
    def __init__(self, events):
        self.events = events
        self.generation = 0
        self.held = True
        self.eligible = None
        self.on_release = None

    def hold(self):
        self.generation += 1
        self.held = True
        self.eligible = None
        self.events.append(("hold", self.generation))
        return self.generation

    def mark_reset_complete(self, generation):
        assert generation == self.generation
        assert self.held
        self.eligible = generation
        self.events.append(("complete", generation))

    def release(self, generation, *, source):
        assert generation == self.generation == self.eligible
        assert self.held
        self.held = False
        self.eligible = None
        self.events.append(("release", generation, source))
        if self.on_release is not None:
            self.on_release()


def _finalization_bridge(events, gate, *, external_release):
    return SimpleNamespace(
        _active_transaction=None,
        _manager=object(),
        _reset_stop_gate=gate,
        _external_recovery_release_required=external_release,
        _reset_event_publisher=FakePublisher(events),
        _EmptyMessage=lambda: object(),
        _initial_pose_republisher=SimpleNamespace(cancel=lambda: None),
        _deferred_initial_pose_name=None,
        _apply_initial_pose_policy=lambda: None,
    )


def test_non_navigation_success_auto_releases_same_generation():
    events = []
    gate = RecordingStopGate(events)
    generation = gate.hold()
    bridge = _finalization_bridge(events, gate, external_release=False)

    def assert_new_reset_is_still_excluded():
        with pytest.raises(ResetServiceError, match="another reset transaction"):
            ResetServiceBridge.start_reset(
                bridge,
                ResetRequest("mapping_start", "mapping", "ideal", 1),
            )

    gate.on_release = assert_new_reset_is_still_excluded
    transaction = _ResetTransaction(
        generation=1,
        completion=FakeCompletion(),
        on_finished=lambda tx: ResetServiceBridge._finish_transaction(bridge, tx),
        stop_generation=generation,
    )
    bridge._active_transaction = transaction
    transaction.timeout_timer = FakeTimer()
    transaction.seal()

    assert not gate.held
    assert events == [
        ("hold", generation),
        "reset_event",
        ("complete", generation),
        ("release", generation, "reset_transaction_complete"),
    ]


def test_navigation_success_remains_held_for_external_generation_release():
    events = []
    gate = RecordingStopGate(events)
    generation = gate.hold()
    bridge = _finalization_bridge(events, gate, external_release=True)
    transaction = _ResetTransaction(
        generation=2,
        completion=FakeCompletion(),
        on_finished=lambda tx: ResetServiceBridge._finish_transaction(bridge, tx),
        stop_generation=generation,
    )
    bridge._active_transaction = transaction
    transaction.timeout_timer = FakeTimer()
    transaction.seal()

    assert gate.held
    assert gate.eligible == generation
    assert events == [
        ("hold", generation),
        "reset_event",
        ("complete", generation),
    ]


@pytest.mark.parametrize("failure", ["reset_event", "initial_pose"])
def test_critical_finalization_failure_keeps_non_navigation_gate_held(failure):
    events = []
    gate = RecordingStopGate(events)
    generation = gate.hold()
    bridge = _finalization_bridge(events, gate, external_release=False)
    if failure == "reset_event":
        bridge._reset_event_publisher = FailingPublisher()
    else:
        bridge._apply_initial_pose_policy = lambda: (_ for _ in ()).throw(
            RuntimeError("initial pose policy failed")
        )
    transaction = _ResetTransaction(
        generation=12,
        completion=FakeCompletion(),
        on_finished=lambda tx: ResetServiceBridge._finish_transaction(bridge, tx),
        initial_pose_name=(
            "mapping_start" if failure == "initial_pose" else None
        ),
        stop_generation=generation,
    )
    bridge._active_transaction = transaction
    transaction.timeout_timer = FakeTimer()
    transaction.seal()

    assert transaction.errors
    assert "transaction finalization" in transaction.errors[0]
    assert gate.held
    assert gate.eligible is None
    assert not any(
        isinstance(event, tuple) and event[0] == "release" for event in events
    )


def test_release_failure_is_transaction_failure_and_keeps_gate_held():
    events = []

    class FailingReleaseStopGate(RecordingStopGate):
        def release(self, generation, *, source):
            del generation, source
            raise RuntimeError("release status publication failed")

    gate = FailingReleaseStopGate(events)
    generation = gate.hold()
    bridge = _finalization_bridge(events, gate, external_release=False)
    transaction = _ResetTransaction(
        generation=13,
        completion=FakeCompletion(),
        on_finished=lambda tx: ResetServiceBridge._finish_transaction(bridge, tx),
        stop_generation=generation,
    )
    bridge._active_transaction = transaction
    transaction.timeout_timer = FakeTimer()
    transaction.seal()

    assert transaction.errors == [
        "transaction finalization: RuntimeError: release status publication failed"
    ]
    assert gate.held
    assert gate.eligible == generation
    assert events == [
        ("hold", generation),
        "reset_event",
        ("complete", generation),
    ]


def test_failed_transaction_never_marks_complete_or_auto_releases():
    events = []
    gate = RecordingStopGate(events)
    generation = gate.hold()
    bridge = _finalization_bridge(events, gate, external_release=False)
    transaction = _ResetTransaction(
        generation=3,
        completion=FakeCompletion(),
        on_finished=lambda tx: ResetServiceBridge._finish_transaction(bridge, tx),
        stop_generation=generation,
    )
    bridge._active_transaction = transaction
    transaction.timeout_timer = FakeTimer()
    transaction.record_error("reset", "failed")
    transaction.seal()

    assert gate.held
    assert gate.eligible is None
    assert events == [("hold", generation)]


def test_close_cancels_active_reset_without_emitting_epoch_event():
    events = []
    pending = CancellableFakeFuture()
    bridge = SimpleNamespace(
        _closed=False,
        _active_transaction=None,
        _pending_futures={pending},
        _initial_pose_republisher=SimpleNamespace(
            cancel=lambda: events.append("pose_cancel")
        ),
        _deferred_initial_pose_name="mapping_start",
        _reset_event_publisher=FakePublisher(events),
        _EmptyMessage=lambda: object(),
        _apply_initial_pose_policy=lambda: events.append("arm"),
    )
    transaction = _ResetTransaction(
        generation=11,
        completion=FakeCompletion(),
        on_finished=lambda tx: ResetServiceBridge._finish_transaction(
            bridge, tx
        ),
    )
    transaction.timeout_timer = FakeTimer()
    transaction.add_call("costmap", pending)
    transaction.seal()
    bridge._active_transaction = transaction

    ResetServiceBridge.close(bridge)
    ResetServiceBridge.close(bridge)

    assert bridge._closed
    assert pending.cancelled
    assert transaction.finished
    assert transaction.timeout_timer.cancelled
    assert any("shutdown" in error for error in transaction.errors)
    assert events == ["pose_cancel", "pose_cancel"]
    assert bridge._pending_futures == set()


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


def test_reset_stop_hold_is_synchronous_before_reset_manager_pause_path():
    events = []

    class Gate:
        def hold(self):
            events.append("hold")
            return 3

    class Manager:
        def reset(self, request):
            del request
            events.append("manager_reset")

    bridge = SimpleNamespace(
        _manager=Manager(),
        _reset_stop_gate=Gate(),
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
    transaction = ResetServiceBridge.start_reset(
        bridge,
        ResetRequest("mapping_start", "mapping", "ideal", 8601),
    )
    assert events == ["hold", "manager_reset"]
    assert transaction.stop_generation == 3


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
    assert 'reset_receipt={"case_id":"","generation":12' in response.message
    assert '"seed":0,"variant_id":""}' in response.message
