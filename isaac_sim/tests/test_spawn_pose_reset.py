from __future__ import annotations

import asyncio
from dataclasses import dataclass, field, replace
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from isaac_sim.src.config import ResetStrategyConfig
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
    position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    orientation: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    linear_velocity: tuple[float, float, float] = (1.0, 2.0, 3.0)
    angular_velocity: tuple[float, float, float] = (4.0, 5.0, 6.0)

    def set_world_pose(self, position, orientation_wxyz):
        self.position = tuple(position)
        self.orientation = tuple(orientation_wxyz)
        self.calls.append(("pose", (tuple(position), tuple(orientation_wxyz))))

    def get_world_pose(self):
        # Quaternion sign is physically equivalent and must not fail readback.
        return self.position, tuple(-value for value in self.orientation)

    def set_base_velocities(self, linear, angular):
        self.linear_velocity = tuple(linear)
        self.angular_velocity = tuple(angular)
        self.calls.append(("base_velocity", (tuple(linear), tuple(angular))))

    def get_base_velocities(self):
        return self.linear_velocity, self.angular_velocity

    def restore_initial_joint_state(self):
        self.calls.append(("joint_state", "initial_pose_and_zero_dynamics"))


@dataclass
class FakeSimulation:
    calls: list[str] = field(default_factory=list)
    events: list[str] | None = None

    def pause(self):
        self.calls.append("pause")
        if self.events is not None:
            self.events.append("pause")

    def step(self, *, render):
        self.calls.append(f"step:{render}")
        if self.events is not None:
            self.events.append(f"step:{render}")

    def play(self):
        self.calls.append("play")
        if self.events is not None:
            self.events.append("play")


@dataclass
class FakeContactProbe:
    calls: list[float] = field(default_factory=list)
    events: list[str] | None = None
    error: Exception | None = None

    def assert_all_wheels_separated(self, *, physics_dt_s):
        self.calls.append(physics_dt_s)
        if self.events is not None:
            self.events.append("assert_separated")
        if self.error is not None:
            raise self.error


def make_reset_manager(
    simulation,
    spawn,
    hooks,
    *,
    strategy_id="pose_restore_v1",
    contact_probe=None,
):
    return ResetManager(
        simulation,
        spawn,
        hooks,
        reset_strategy=ResetStrategyConfig(
            schema_version=1,
            identifier=strategy_id,
        ),
        contact_probe=contact_probe or FakeContactProbe(),
        physics_dt_s=1.0 / 60.0,
    )


def test_spawn_reset_zeros_all_robot_state_and_gates_map_pose():
    robot = FakeRobot()
    poses = load_spawn_poses(ROOT / "isaac_sim/configs/spawn_poses.yaml")
    manager = SpawnPoseManager(robot, poses)
    manager.apply_usd_pose("mapping_start")
    assert robot.calls == [
        ("joint_state", "initial_pose_and_zero_dynamics"),
        (
            "base_velocity",
            ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        ),
        (
            "pose",
            (
                poses["mapping_start"].usd.position,
                (1.0, 0.0, 0.0, 0.0),
            ),
        ),
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


def test_spawn_reset_supports_fixed_z_lift_and_rejects_pose_readback_mismatch():
    poses = load_spawn_poses(ROOT / "isaac_sim/configs/spawn_poses.yaml")
    robot = FakeRobot()
    manager = SpawnPoseManager(robot, poses)

    manager.apply_usd_pose("mapping_start", z_offset_m=0.2)

    assert robot.position == (
        poses["mapping_start"].usd.position[0],
        poses["mapping_start"].usd.position[1],
        poses["mapping_start"].usd.position[2] + 0.2,
    )
    assert robot.linear_velocity == (0.0, 0.0, 0.0)
    assert robot.angular_velocity == (0.0, 0.0, 0.0)

    robot.get_world_pose = lambda: ((99.0, 0.0, 0.0), robot.orientation)
    with pytest.raises(SpawnPoseError, match="root pose readback"):
        manager.apply_usd_pose("mapping_start")

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
    make_reset_manager(simulation, spawn, hooks).reset(
        ResetRequest("mapping_start", "mapping", "ideal", 123)
    )
    assert simulation.calls == ["pause", "step:False", "play"]
    assert calls == ["zero", "controller", "odom:ideal", "gt", "dynamic:123"]


def test_localization_reset_clears_costmaps_before_initial_pose():
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

    make_reset_manager(simulation, spawn, hooks).reset(
        ResetRequest("mapping_start", "localization", "ideal", 123)
    )

    assert calls == [
        "zero",
        "controller",
        "odom:ideal",
        "gt",
        "dynamic:123",
        "costmaps",
        "initial:mapping_start",
    ]


def test_invalid_reset_request_is_rejected():
    with pytest.raises(ResetError, match="non-negative"):
        ResetRequest("mapping_start", "mapping", "ideal", -1)


def test_localization_reset_pose_is_immutable_manifest_bound_profile():
    parameters = {
        "reset_seed": SimpleNamespace(value=0),
        "reset_pose_name": SimpleNamespace(value="other_map_pose"),
        "navigation_mode": SimpleNamespace(value="localization"),
        "odometry_mode": SimpleNamespace(value="ideal"),
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


def test_reset_manager_recovers_exact_pose_and_stays_paused_when_a_hook_fails():
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
        make_reset_manager(simulation, spawn, hooks).reset(
            ResetRequest("mapping_start", "mapping", "ideal", 0)
        )

    assert simulation.calls == ["pause", "pause"]
    assert robot.position == load_spawn_poses(
        ROOT / "isaac_sim/configs/spawn_poses.yaml"
    )["mapping_start"].usd.position
    assert "play" not in simulation.calls


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
        make_reset_manager(simulation, spawn, hooks).reset(
            ResetRequest("mapping_start", "localization", "ideal", 0)
        )

    assert simulation.calls == []
    assert robot.calls == []


def test_separate_recontact_strategy_has_exact_two_step_order():
    events: list[str] = []
    robot = FakeRobot()
    delegate = SpawnPoseManager(
        robot, load_spawn_poses(ROOT / "isaac_sim/configs/spawn_poses.yaml")
    )

    class RecordingSpawn:
        def get(self, pose_name):
            return delegate.get(pose_name)

        def get_map_pose(self, pose_name, *, purpose):
            return delegate.get_map_pose(pose_name, purpose=purpose)

        def apply_usd_pose(self, pose_name, *, z_offset_m=0.0):
            events.append(f"pose:{z_offset_m:.1f}")
            return delegate.apply_usd_pose(pose_name, z_offset_m=z_offset_m)

    simulation = FakeSimulation(events=events)
    probe = FakeContactProbe(events=events)
    hooks = ResetHooks(
        send_zero_velocity=lambda: events.append("zero"),
        clear_controller_state=lambda: events.append("controller"),
        reset_odometry=lambda mode: events.append(f"odom:{mode}"),
        reset_ground_truth_path=lambda: events.append("gt"),
        reset_dynamic_obstacles=lambda seed: events.append(f"dynamic:{seed}"),
        clear_costmaps=lambda: events.append("costmaps"),
        publish_map_initial_pose=lambda pose: events.append(f"initial:{pose}"),
    )

    make_reset_manager(
        simulation,
        RecordingSpawn(),
        hooks,
        strategy_id="separate_recontact_0p20m_1step_v1",
        contact_probe=probe,
    ).reset(ResetRequest("mapping_start", "mapping", "ideal", 123))

    assert events == [
        "pause",
        "zero",
        "controller",
        "pose:0.2",
        "step:False",
        "assert_separated",
        "pose:0.0",
        "odom:ideal",
        "gt",
        "dynamic:123",
        "step:False",
        "play",
    ]
    assert probe.calls == [pytest.approx(1.0 / 60.0)]


def test_separation_failure_recovers_pose_without_hooks_step_or_play():
    events: list[str] = []
    robot = FakeRobot()
    delegate = SpawnPoseManager(
        robot, load_spawn_poses(ROOT / "isaac_sim/configs/spawn_poses.yaml")
    )

    class RecordingSpawn:
        def get(self, pose_name):
            return delegate.get(pose_name)

        def apply_usd_pose(self, pose_name, *, z_offset_m=0.0):
            events.append(f"pose:{z_offset_m:.1f}")
            return delegate.apply_usd_pose(pose_name, z_offset_m=z_offset_m)

    simulation = FakeSimulation(events=events)
    probe = FakeContactProbe(
        events=events,
        error=RuntimeError("wheel contact remains"),
    )
    hooks = ResetHooks(
        send_zero_velocity=lambda: events.append("zero"),
        clear_controller_state=lambda: events.append("controller"),
        reset_odometry=lambda mode: events.append(f"odom:{mode}"),
        reset_ground_truth_path=lambda: events.append("gt"),
        reset_dynamic_obstacles=lambda seed: events.append(f"dynamic:{seed}"),
        clear_costmaps=lambda: events.append("costmaps"),
        publish_map_initial_pose=lambda pose: events.append(f"initial:{pose}"),
    )

    with pytest.raises(ResetError, match="wheel contact remains"):
        make_reset_manager(
            simulation,
            RecordingSpawn(),
            hooks,
            strategy_id="separate_recontact_0p20m_1step_v1",
            contact_probe=probe,
        ).reset(ResetRequest("mapping_start", "mapping", "ideal", 5))

    assert events == [
        "pause",
        "zero",
        "controller",
        "pose:0.2",
        "step:False",
        "assert_separated",
        "pause",
        "pose:0.0",
    ]
    assert robot.position == delegate.get("mapping_start").usd.position


def test_reset_error_reports_failed_best_effort_recovery_and_never_plays():
    events: list[str] = []
    delegate = SpawnPoseManager(
        FakeRobot(),
        load_spawn_poses(ROOT / "isaac_sim/configs/spawn_poses.yaml"),
    )

    class FailingRecoverySpawn:
        def get(self, pose_name):
            return delegate.get(pose_name)

        def apply_usd_pose(self, pose_name, *, z_offset_m=0.0):
            events.append(f"pose:{z_offset_m:.1f}")
            if z_offset_m == 0.0:
                raise RuntimeError("recovery pose rejected")
            return delegate.apply_usd_pose(pose_name, z_offset_m=z_offset_m)

    simulation = FakeSimulation(events=events)
    probe = FakeContactProbe(
        events=events,
        error=RuntimeError("contact query failed"),
    )
    hooks = ResetHooks(
        send_zero_velocity=lambda: events.append("zero"),
        clear_controller_state=lambda: events.append("controller"),
        reset_odometry=lambda mode: events.append(f"odom:{mode}"),
        reset_ground_truth_path=lambda: events.append("gt"),
        reset_dynamic_obstacles=lambda seed: events.append(f"dynamic:{seed}"),
        clear_costmaps=lambda: events.append("costmaps"),
        publish_map_initial_pose=lambda pose: events.append(f"initial:{pose}"),
    )

    with pytest.raises(
        ResetError,
        match="contact query failed.*recovery pose rejected",
    ):
        make_reset_manager(
            simulation,
            FailingRecoverySpawn(),
            hooks,
            strategy_id="separate_recontact_0p20m_1step_v1",
            contact_probe=probe,
        ).reset(ResetRequest("mapping_start", "mapping", "ideal", 5))

    assert events[-2:] == ["pause", "pose:0.0"]
    assert "play" not in events


def test_initial_pause_failure_is_retried_before_best_effort_pose_recovery():
    class FailsFirstPause(FakeSimulation):
        def pause(self):
            super().pause()
            if self.calls.count("pause") == 1:
                raise RuntimeError("timeline pause rejected")

    robot = FakeRobot()
    spawn = SpawnPoseManager(
        robot, load_spawn_poses(ROOT / "isaac_sim/configs/spawn_poses.yaml")
    )
    simulation = FailsFirstPause()
    hooks = ResetHooks(
        send_zero_velocity=lambda: None,
        clear_controller_state=lambda: None,
        reset_odometry=lambda mode: None,
        reset_ground_truth_path=lambda: None,
        reset_dynamic_obstacles=lambda seed: None,
        clear_costmaps=lambda: None,
        publish_map_initial_pose=lambda pose: None,
    )

    with pytest.raises(ResetError, match="timeline pause rejected"):
        make_reset_manager(simulation, spawn, hooks).reset(
            ResetRequest("mapping_start", "mapping", "ideal", 0)
        )

    assert simulation.calls == ["pause", "pause"]
    assert robot.position == spawn.get("mapping_start").usd.position


def test_recovery_pause_failure_never_teleports_or_claims_paused_state():
    class AlwaysFailsPause(FakeSimulation):
        def pause(self):
            super().pause()
            raise RuntimeError("timeline pause unavailable")

    robot = FakeRobot()
    spawn = SpawnPoseManager(
        robot, load_spawn_poses(ROOT / "isaac_sim/configs/spawn_poses.yaml")
    )
    simulation = AlwaysFailsPause()
    hooks = ResetHooks(
        send_zero_velocity=lambda: None,
        clear_controller_state=lambda: None,
        reset_odometry=lambda mode: None,
        reset_ground_truth_path=lambda: None,
        reset_dynamic_obstacles=lambda seed: None,
        clear_costmaps=lambda: None,
        publish_map_initial_pose=lambda pose: None,
    )

    with pytest.raises(
        ResetError,
        match=(
            "pause could not be verified and no recovery teleport was "
            "attempted.*timeline pause unavailable"
        ),
    ):
        make_reset_manager(simulation, spawn, hooks).reset(
            ResetRequest("mapping_start", "mapping", "ideal", 0)
        )

    assert simulation.calls == ["pause", "pause"]
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


def test_timeout_failure_does_not_emit_reset_event_or_arm_initial_pose():
    events = []
    cancelled = []
    bridge = SimpleNamespace(
        _active_transaction=None,
        _simulation_time=lambda: 12.5,
        _reset_event_publisher=FakePublisher(events),
        _EmptyMessage=lambda: object(),
        _initial_pose_republisher=SimpleNamespace(
            cancel=lambda: cancelled.append("cancel")
        ),
        _deferred_initial_pose_name="old",
        _apply_initial_pose_policy=lambda **_: events.append("arm"),
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


def test_successful_transaction_prepares_policy_before_epoch_event_commit():
    events = []

    class RecordingRepublisher:
        @staticmethod
        def schedule(pose_name, *, after_stamp_s):
            assert pose_name == "mapping_start"
            assert after_stamp_s == pytest.approx(12.5)
            events.append("schedule")

        @staticmethod
        def cancel():
            events.append("cancel")

    bridge = SimpleNamespace(
        _active_transaction=None,
        _simulation_time=lambda: 12.5,
        _reset_event_publisher=FakePublisher(events),
        _EmptyMessage=lambda: object(),
        _initial_pose_republisher=RecordingRepublisher(),
        _initial_pose_source="auto",
        _deferred_initial_pose_name=None,
    )
    bridge._apply_initial_pose_policy = (
        lambda **kwargs: ResetServiceBridge._apply_initial_pose_policy(
            bridge, **kwargs
        )
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

    assert events == ["schedule", "reset_event"]
    assert bridge._deferred_initial_pose_name is None
    assert transaction.boundary_clock_ns == 12_500_000_000


@pytest.mark.parametrize("boundary_time_s", [math.nan, math.inf, -0.1])
def test_invalid_reset_boundary_never_emits_epoch_event(boundary_time_s):
    events = []
    bridge = SimpleNamespace(
        _active_transaction=None,
        _simulation_time=lambda: boundary_time_s,
        _reset_event_publisher=FakePublisher(events),
        _EmptyMessage=lambda: object(),
        _initial_pose_republisher=SimpleNamespace(cancel=lambda: None),
        _deferred_initial_pose_name=None,
        _apply_initial_pose_policy=lambda **_: events.append("arm"),
    )
    transaction = _ResetTransaction(
        generation=12,
        completion=FakeCompletion(),
        on_finished=lambda tx: ResetServiceBridge._finish_transaction(
            bridge, tx
        ),
    )
    bridge._active_transaction = transaction

    transaction.seal()

    assert events == []
    assert transaction.boundary_clock_ns is None
    assert any("boundary simulation time" in error for error in transaction.errors)


def test_initial_pose_policy_failure_happens_before_epoch_event_commit():
    events = []

    class PartiallyFailingRepublisher:
        @staticmethod
        def schedule(pose_name, *, after_stamp_s):
            assert pose_name == "mapping_start"
            assert after_stamp_s == pytest.approx(12.5)
            events.append("schedule_attempt")
            raise RuntimeError("cannot arm initial pose")

        @staticmethod
        def cancel():
            events.append("cancel")

    bridge = SimpleNamespace(
        _active_transaction=None,
        _simulation_time=lambda: 12.5,
        _reset_event_publisher=FakePublisher(events),
        _EmptyMessage=lambda: object(),
        _initial_pose_republisher=PartiallyFailingRepublisher(),
        _initial_pose_source="auto",
        _deferred_initial_pose_name=None,
    )
    bridge._apply_initial_pose_policy = (
        lambda **kwargs: ResetServiceBridge._apply_initial_pose_policy(
            bridge, **kwargs
        )
    )
    transaction = _ResetTransaction(
        generation=13,
        completion=FakeCompletion(),
        on_finished=lambda tx: ResetServiceBridge._finish_transaction(
            bridge, tx
        ),
        initial_pose_name="mapping_start",
    )
    bridge._active_transaction = transaction

    transaction.seal()

    assert events == ["schedule_attempt", "cancel"]
    assert transaction.boundary_clock_ns is None
    assert bridge._deferred_initial_pose_name is None
    assert any("cannot arm initial pose" in error for error in transaction.errors)


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
        _apply_initial_pose_policy=lambda **_: events.append("arm"),
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
        boundary_clock_ns=None if errors else 12_500_000_000,
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
    assert (
        "boundary_clock_ns=None" if errors else "boundary_clock_ns=12500000000"
    ) in response.message
    if errors:
        assert "; reset_metadata_v1=" not in response.message
    else:
        marker = "; reset_metadata_v1="
        metadata = json.loads(response.message.rpartition(marker)[2])
        assert metadata == {
            "boundary_clock_ns": 12_500_000_000,
            "generation": 12,
            "schema_version": 1,
        }
    assert events[1] == (
        "response_failure" if errors else "response_success"
    )
