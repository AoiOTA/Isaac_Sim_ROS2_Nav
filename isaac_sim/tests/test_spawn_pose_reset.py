from __future__ import annotations

import asyncio
from dataclasses import dataclass, field, replace
from hashlib import sha256
from pathlib import Path
from types import SimpleNamespace

import pytest

from isaac_sim.apps.navigation_sim import _prime_isaac_ros_clock
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


def test_startup_primes_isaac_clock_under_zero_hold_before_reset():
    state = {"simulation": 0.0, "ros": 0.0}
    events = ["held", "zero"]

    def play_first_update():
        events.append("play_update")
        state["simulation"] = 0.01

    def app_update():
        events.append("app_update")
        state["simulation"] += 0.01

    def spin_once():
        events.append("spin")
        if state["simulation"] >= 0.02:
            state["ros"] = state["simulation"]

    result = _prime_isaac_ros_clock(
        play_first_update=play_first_update,
        app_update=app_update,
        spin_once=spin_once,
        simulation_time=lambda: state["simulation"],
        ros_time=lambda: state["ros"],
        max_frame_lag_seconds=0.01,
        max_updates=3,
    )

    assert result == pytest.approx((0.0, 0.0, 0.02, 0.02))
    assert events == [
        "held",
        "zero",
        "play_update",
        "spin",
        "app_update",
        "spin",
    ]
    source = (ROOT / "isaac_sim/apps/navigation_sim.py").read_text(
        encoding="utf-8"
    )
    bind = source.index("reset_bridge.bind(reset_manager)")
    zero = source.index("reset_stop_gate.publish_zero()", bind)
    prime = source.index("_prime_isaac_ros_clock(", zero)
    startup = source.index("startup_reset = reset_bridge.start_reset(", prime)
    assert bind < zero < prime < startup


def test_startup_clock_priming_failure_is_bounded_and_fail_stop():
    state = {"simulation": 0.0}
    updates = []

    def advance(kind):
        updates.append(kind)
        state["simulation"] += 0.01

    with pytest.raises(
        RuntimeError,
        match=r"Isaac ROS clock did not start.*updates=3",
    ):
        _prime_isaac_ros_clock(
            play_first_update=lambda: advance("play_update"),
            app_update=lambda: advance("app_update"),
            spin_once=lambda: None,
            simulation_time=lambda: state["simulation"],
            ros_time=lambda: 0.0,
            max_frame_lag_seconds=0.01,
            max_updates=3,
        )

    assert updates == ["play_update", "app_update", "app_update"]


def test_startup_clock_priming_rejects_stale_external_epoch():
    state = {"simulation": 0.0, "ros": 0.0}

    def update():
        state["simulation"] += 0.01

    def spin_once():
        state["ros"] = 123.0

    with pytest.raises(
        RuntimeError,
        match=(
            r"clock did not advance within one publish frame.*"
            r"initial_sim_time=0\.000000000.*initial_ros_time=0\.000000000.*"
            r"simulation_time=0\.050000000.*ros_time=123\.000000000.*updates=5"
        ),
    ):
        _prime_isaac_ros_clock(
            play_first_update=update,
            app_update=update,
            spin_once=spin_once,
            simulation_time=lambda: state["simulation"],
            ros_time=lambda: state["ros"],
            max_frame_lag_seconds=0.01,
        )


def test_startup_clock_priming_rejects_initial_external_clock():
    updates = []

    with pytest.raises(
        RuntimeError,
        match=r"initial ROS clock is outside the Isaac epoch.*updates=0",
    ):
        _prime_isaac_ros_clock(
            play_first_update=lambda: updates.append("play_update"),
            app_update=lambda: updates.append("app_update"),
            spin_once=lambda: None,
            simulation_time=lambda: 0.0,
            ros_time=lambda: 123.0,
            max_frame_lag_seconds=0.01,
        )

    assert updates == []


def test_startup_clock_priming_rejects_ros_clock_rollback():
    state = {"simulation": 1.0, "ros": 1.0}

    def update():
        state["simulation"] += 0.01

    def spin_once():
        state["ros"] = 0.5

    with pytest.raises(
        RuntimeError,
        match=r"ROS clock moved backwards.*updates=1",
    ):
        _prime_isaac_ros_clock(
            play_first_update=update,
            app_update=update,
            spin_once=spin_once,
            simulation_time=lambda: state["simulation"],
            ros_time=lambda: state["ros"],
            max_frame_lag_seconds=0.01,
        )


def test_startup_clock_priming_accepts_one_publish_frame_lag():
    state = {"simulation": 0.0, "ros": 0.0}

    def update():
        state["simulation"] += 0.01

    def spin_once():
        state["ros"] = max(0.0, state["simulation"] - 0.01)

    result = _prime_isaac_ros_clock(
        play_first_update=update,
        app_update=update,
        spin_once=spin_once,
        simulation_time=lambda: state["simulation"],
        ros_time=lambda: state["ros"],
        max_frame_lag_seconds=0.01,
    )

    assert result == pytest.approx((0.0, 0.0, 0.02, 0.01))


def test_startup_clock_priming_accepts_float_quantized_equal_times():
    state = {"simulation": 0.0, "ros": 0.0}

    def update():
        state["simulation"] = 0.15

    def spin_once():
        state["ros"] = 0.15000000000000002

    result = _prime_isaac_ros_clock(
        play_first_update=update,
        app_update=update,
        spin_once=spin_once,
        simulation_time=lambda: state["simulation"],
        ros_time=lambda: state["ros"],
        max_frame_lag_seconds=0.01,
        max_updates=1,
    )

    assert result == (0.0, 0.0, 0.15, 0.15000000000000002)


def test_startup_clock_priming_rejects_ros_ahead_beyond_float_epsilon():
    state = {"simulation": 0.0, "ros": 0.0}

    def update():
        state["simulation"] = 0.15

    def spin_once():
        state["ros"] = 0.150000002

    with pytest.raises(
        RuntimeError,
        match=r"clock did not advance within one publish frame.*updates=1",
    ):
        _prime_isaac_ros_clock(
            play_first_update=update,
            app_update=update,
            spin_once=spin_once,
            simulation_time=lambda: state["simulation"],
            ros_time=lambda: state["ros"],
            max_frame_lag_seconds=0.01,
            max_updates=1,
        )


def test_startup_clock_priming_accepts_float_quantized_upper_bound():
    state = {"simulation": 0.0, "ros": 0.0}

    def update():
        state["simulation"] = 0.15

    def spin_once():
        state["ros"] = 0.13999999999999999

    result = _prime_isaac_ros_clock(
        play_first_update=update,
        app_update=update,
        spin_once=spin_once,
        simulation_time=lambda: state["simulation"],
        ros_time=lambda: state["ros"],
        max_frame_lag_seconds=0.01,
        max_updates=1,
    )

    assert result == (0.0, 0.0, 0.15, 0.13999999999999999)


def test_startup_clock_priming_rejects_more_than_one_publish_frame_lag():
    state = {"simulation": 0.0, "ros": 0.0}
    updates = []

    def update(kind):
        updates.append(kind)
        state["simulation"] += 0.01

    def spin_once():
        state["ros"] += 0.001

    with pytest.raises(
        RuntimeError,
        match=r"clock did not advance within one publish frame.*updates=5",
    ):
        _prime_isaac_ros_clock(
            play_first_update=lambda: update("play_update"),
            app_update=lambda: update("app_update"),
            spin_once=spin_once,
            simulation_time=lambda: state["simulation"],
            ros_time=lambda: state["ros"],
            max_frame_lag_seconds=0.005,
        )

    assert updates == ["play_update"] + ["app_update"] * 4


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
        _queue_service_call=lambda client, request, label, **kwargs: queued.append(
            (client, request, label, kwargs)
        ),
    )

    ResetServiceBridge.reset_ros_odometry(bridge, "mixed")

    assert [label for _, _, label, _ in queued] == ["wheel odometry", "EKF"]
    assert [kwargs for _, _, _, kwargs in queued] == [
        {"required": True}, {"required": True}
    ]
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
    def __init__(self, result=None):
        self._done = False
        self._error = None
        self._result = object() if result is None else result
        self._callbacks = []

    def add_done_callback(self, callback):
        self._callbacks.append(callback)

    def done(self):
        return self._done

    def result(self):
        if self._error is not None:
            raise self._error
        return self._result

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


class FakeResetClient:
    def __init__(
        self,
        *,
        ready=True,
        future=None,
        queue_error=None,
        discovered_on_wait=False,
        wait_error=None,
    ):
        self.ready = ready
        self.future = future or FakeFuture()
        self.queue_error = queue_error
        self.discovered_on_wait = discovered_on_wait
        self.wait_error = wait_error
        self.wait_calls = []
        self.queue_count = 0

    def service_is_ready(self):
        return self.ready

    def wait_for_service(self, *, timeout_sec):
        self.wait_calls.append(timeout_sec)
        if self.wait_error is not None:
            raise self.wait_error
        if self.discovered_on_wait:
            self.ready = True
        return self.ready

    def call_async(self, request):
        del request
        self.queue_count += 1
        if self.queue_error is not None:
            raise self.queue_error
        return self.future


class ResetTestLogger:
    def __init__(self):
        self.messages = []

    def warning(self, message):
        self.messages.append(("warning", message))

    def error(self, message):
        self.messages.append(("error", message))


class ResetTestEmptyService:
    class Request:
        pass


class ResetTestSetPose:
    class Request:
        def __init__(self):
            self.pose = SimpleNamespace(
                header=SimpleNamespace(stamp=None, frame_id=""),
                pose=SimpleNamespace(
                    pose=SimpleNamespace(
                        orientation=SimpleNamespace(w=0.0)
                    ),
                    covariance=[0.0] * 36,
                ),
            )


def _odometry_transaction_bridge(events, wheel_client, ekf_client):
    gate = RecordingStopGate(events)
    generation = gate.hold()
    bridge = object.__new__(ResetServiceBridge)
    bridge._wheel_reset_client = wheel_client
    bridge._ekf_set_pose_client = ekf_client
    bridge._EmptyService = ResetTestEmptyService
    bridge._SetPose = ResetTestSetPose
    bridge._pending_futures = set()
    bridge._unavailable_warnings = set()
    bridge._required_service_discovery_pending = True
    bridge._transaction_timeout_sec = 1.5
    bridge._reset_stop_gate = gate
    bridge._external_recovery_release_required = False
    bridge._reset_event_publisher = FakePublisher(events)
    bridge._EmptyMessage = lambda: object()
    bridge._initial_pose_republisher = SimpleNamespace(cancel=lambda: None)
    bridge._deferred_initial_pose_name = None
    bridge._apply_initial_pose_policy = lambda: None
    bridge.node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: "stamp")
        ),
        get_logger=lambda: ResetTestLogger(),
    )
    transaction = _ResetTransaction(
        generation=1,
        completion=FakeCompletion(),
        on_finished=lambda tx: ResetServiceBridge._finish_transaction(
            bridge, tx
        ),
        stop_generation=generation,
    )
    transaction.timeout_timer = FakeTimer()
    bridge._active_transaction = transaction
    return bridge, transaction, gate


def test_mixed_startup_waits_once_for_delayed_required_service_discovery():
    events = []
    wheel = FakeResetClient(ready=False, discovered_on_wait=True)
    ekf = FakeResetClient(ready=False, discovered_on_wait=True)
    bridge, transaction, gate = _odometry_transaction_bridge(
        events, wheel, ekf
    )

    bridge.reset_ros_odometry("mixed")
    transaction.seal()
    wheel.future.complete()
    ekf.future.complete()

    assert wheel.wait_calls == [1.5]
    assert ekf.wait_calls == [1.5]
    assert wheel.queue_count == 1
    assert ekf.queue_count == 1
    assert transaction.errors == []
    assert not gate.held


@pytest.mark.parametrize("missing", ["wheel odometry", "EKF"])
def test_mixed_startup_discovery_timeout_keeps_generation_held(missing):
    events = []
    wheel = FakeResetClient(ready=missing != "wheel odometry")
    ekf = FakeResetClient(ready=missing != "EKF")
    bridge, transaction, gate = _odometry_transaction_bridge(
        events, wheel, ekf
    )

    bridge.reset_ros_odometry("mixed")
    transaction.seal()
    for client in (wheel, ekf):
        if client.ready:
            client.future.complete()

    missing_client = wheel if missing == "wheel odometry" else ekf
    assert missing_client.wait_calls == [1.5]
    assert missing_client.queue_count == 0
    assert transaction.errors == [
        f"{missing}: required reset service is unavailable"
    ]
    assert gate.held
    assert events == [("hold", 1)]


def test_mixed_startup_discovery_wait_exception_keeps_generation_held():
    events = []
    wheel = FakeResetClient(
        ready=False,
        wait_error=RuntimeError("DDS discovery failed"),
    )
    ekf = FakeResetClient()
    bridge, transaction, gate = _odometry_transaction_bridge(
        events, wheel, ekf
    )

    bridge.reset_ros_odometry("mixed")
    transaction.seal()
    ekf.future.complete()

    assert wheel.wait_calls == [1.5]
    assert wheel.queue_count == 0
    assert ekf.queue_count == 1
    assert transaction.errors == [
        "wheel odometry service discovery: RuntimeError: DDS discovery failed"
    ]
    assert gate.held
    assert events == [("hold", 1)]


def test_mixed_startup_ready_services_do_not_wait_and_queue_once():
    events = []
    wheel = FakeResetClient()
    ekf = FakeResetClient()
    bridge, transaction, _ = _odometry_transaction_bridge(events, wheel, ekf)

    bridge.reset_ros_odometry("mixed")
    transaction.seal()
    wheel.future.complete()
    ekf.future.complete()

    assert wheel.wait_calls == []
    assert ekf.wait_calls == []
    assert wheel.queue_count == 1
    assert ekf.queue_count == 1


def test_mixed_followup_reset_keeps_immediate_required_service_check():
    events = []
    wheel = FakeResetClient(ready=False, discovered_on_wait=True)
    ekf = FakeResetClient()
    bridge, transaction, gate = _odometry_transaction_bridge(
        events, wheel, ekf
    )
    bridge._required_service_discovery_pending = False

    bridge.reset_ros_odometry("mixed")
    transaction.seal()
    ekf.future.complete()

    assert wheel.wait_calls == []
    assert wheel.queue_count == 0
    assert transaction.errors == [
        "wheel odometry: required reset service is unavailable"
    ]
    assert gate.held


@pytest.mark.parametrize("missing", ["wheel odometry", "EKF"])
def test_mixed_reset_requires_both_odometry_services(missing):
    events = []
    wheel = FakeResetClient(ready=missing != "wheel odometry")
    ekf = FakeResetClient(ready=missing != "EKF")
    bridge, transaction, gate = _odometry_transaction_bridge(
        events, wheel, ekf
    )

    bridge.reset_ros_odometry("mixed")
    transaction.seal()
    for client in (wheel, ekf):
        if client.ready:
            client.future.complete()

    assert transaction.errors == [
        f"{missing}: required reset service is unavailable"
    ]
    assert gate.held
    assert gate.eligible is None
    assert events == [("hold", 1)]


@pytest.mark.parametrize("failure", ["queue", "future", "negative"])
def test_mixed_reset_service_failure_keeps_generation_held(failure):
    events = []
    wheel = FakeResetClient()
    if failure == "queue":
        ekf = FakeResetClient(queue_error=RuntimeError("queue rejected"))
    elif failure == "future":
        ekf = FakeResetClient()
    else:
        ekf = FakeResetClient(
            future=FakeFuture(
                SimpleNamespace(success=False, message="set pose rejected")
            )
        )
    bridge, transaction, gate = _odometry_transaction_bridge(
        events, wheel, ekf
    )

    bridge.reset_ros_odometry("mixed")
    transaction.seal()
    wheel.future.complete()
    if failure == "future":
        ekf.future.complete(RuntimeError("set pose failed"))
    elif failure == "negative":
        ekf.future.complete()

    assert transaction.errors
    assert gate.held
    assert gate.eligible is None
    assert events == [("hold", 1)]


def test_mixed_reset_completes_only_after_wheel_and_ekf_succeed():
    events = []
    wheel = FakeResetClient()
    ekf = FakeResetClient()
    bridge, transaction, gate = _odometry_transaction_bridge(
        events, wheel, ekf
    )

    bridge.reset_ros_odometry("mixed")
    transaction.seal()
    wheel.future.complete()

    assert not transaction.finished
    assert events == [("hold", 1)]

    ekf.future.complete()

    assert transaction.finished
    assert transaction.errors == []
    assert not gate.held
    assert events == [
        ("hold", 1),
        "reset_event",
        ("complete", 1),
        ("release", 1, "reset_transaction_complete"),
    ]


def test_realistic_reset_keeps_unavailable_odometry_services_optional():
    events = []
    wheel = FakeResetClient(ready=False, discovered_on_wait=True)
    ekf = FakeResetClient(ready=False, discovered_on_wait=True)
    bridge, transaction, gate = _odometry_transaction_bridge(
        events,
        wheel,
        ekf,
    )

    bridge.reset_ros_odometry("realistic")
    transaction.seal()

    assert wheel.wait_calls == []
    assert ekf.wait_calls == []
    assert transaction.errors == []
    assert transaction.skipped == ["wheel odometry", "EKF"]
    assert not gate.held
    assert events == [
        ("hold", 1),
        "reset_event",
        ("complete", 1),
        ("release", 1, "reset_transaction_complete"),
    ]


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


def test_startup_mapping_reset_is_allowed_and_returns_pending_transaction():
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


def test_external_mapping_reset_is_rejected_without_epoch_mutation():
    events = []
    bridge = SimpleNamespace(
        _configured_navigation_mode="mapping",
        _transaction_generation=7,
        start_reset=lambda request: events.append(("reset", request)),
    )

    with pytest.raises(
        ResetServiceError,
        match="end the bag and SLAM processes.*new mapping episode",
    ):
        ResetServiceBridge.start_external_reset(
            bridge,
            ResetRequest("mapping_start", "mapping", "mixed", 0),
        )

    assert bridge._transaction_generation == 7
    assert events == []


def test_external_mapping_reset_service_returns_explicit_failure():
    events = []
    logger = ResetTestLogger()
    request = ResetRequest("mapping_start", "mapping", "mixed", 0)
    bridge = SimpleNamespace(
        _manager=object(),
        _configured_navigation_mode="mapping",
        _transaction_generation=9,
        _read_request=lambda: request,
        start_reset=lambda value: events.append(("reset", value)),
        node=SimpleNamespace(get_logger=lambda: logger),
    )
    bridge.start_external_reset = lambda value: (
        ResetServiceBridge.start_external_reset(bridge, value)
    )
    response = SimpleNamespace(success=None, message="")

    result = asyncio.run(
        ResetServiceBridge._reset_callback(bridge, object(), response)
    )

    assert result is response
    assert response.success is False
    assert "end the bag and SLAM processes" in response.message
    assert "new mapping episode" in response.message
    assert bridge._transaction_generation == 9
    assert events == []
    assert logger.messages[-1][0] == "error"


def test_external_localization_reset_keeps_existing_transaction_path():
    request = ResetRequest(
        "long_route_start_g1", "localization", "mixed", 0
    )
    transaction = object()
    calls = []
    bridge = SimpleNamespace(
        _configured_navigation_mode="localization",
        start_reset=lambda value: calls.append(value) or transaction,
    )

    assert (
        ResetServiceBridge.start_external_reset(bridge, request)
        is transaction
    )
    assert calls == [request]


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
    request_value = ResetRequest(
        "long_route_start_g1", "localization", "ideal", 0
    )
    transaction = SimpleNamespace(
        completion=AwaitMarker(events),
        errors=errors,
        skipped=[],
        generation=12,
    )
    bridge = SimpleNamespace(
        _manager=object(),
        _read_request=lambda: request_value,
        start_external_reset=lambda request: transaction,
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
