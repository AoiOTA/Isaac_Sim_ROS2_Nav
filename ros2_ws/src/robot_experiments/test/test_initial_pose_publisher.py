from types import SimpleNamespace

import pytest
from rclpy.executors import ExternalShutdownException

import robot_experiments.initial_pose_publisher as initial_pose_module
from robot_experiments.initial_pose_publisher import (
    InitialPosePublisher,
    PostResetScanBarrier,
)


def test_post_reset_scan_requires_new_stamp_close_to_current_clock():
    barrier = PostResetScanBarrier(clock_tolerance_sec=0.5)
    barrier.arm(30.0)

    assert not barrier.observe_scan(28.0)
    assert not barrier.observe_scan(30.0)
    assert not barrier.ready
    barrier.observe_clock(30.2)
    assert barrier.observe_scan(30.1)
    assert barrier.ready


def test_clock_rollback_does_not_accept_stale_high_epoch_scans():
    barrier = PostResetScanBarrier(clock_tolerance_sec=0.5)
    barrier.arm(30.0)
    assert barrier.observe_clock(0.1)
    barrier.arm(0.1)

    assert not barrier.observe_scan(28.0)
    assert not barrier.observe_scan(28.1)
    assert not barrier.observe_scan(0.1)
    barrier.observe_clock(0.2)
    assert barrier.observe_scan(0.2)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"clock_tolerance_sec": 0.0},
        {"clock_tolerance_sec": 0.5, "rollback_tolerance_sec": -1.0},
    ],
)
def test_post_reset_scan_barrier_rejects_invalid_configuration(kwargs):
    with pytest.raises(ValueError):
        PostResetScanBarrier(**kwargs)


class FakeTimer:
    def __init__(self):
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


class FakeLogger:
    def __init__(self):
        self.messages = []
        self.warnings = []

    def info(self, message):
        self.messages.append(message)

    def warning(self, message):
        self.warnings.append(message)


def test_ordinary_reseed_preserves_runtime_manual_override():
    timer = FakeTimer()
    logger = FakeLogger()
    statuses = []
    node = SimpleNamespace(
        _manual_override=True,
        complete=False,
        failure="old failure",
        _timer=timer,
        _set_status=statuses.append,
        get_logger=lambda: logger,
    )

    result = InitialPosePublisher._rearm(
        node,
        "simulation clock rollback",
        force_calibrated=False,
        barrier_stamp_s=0.1,
    )

    assert not result
    assert node._manual_override
    assert node.complete
    assert node.failure is None
    assert timer.cancelled
    assert statuses == ["manual_override"]


def test_physical_reset_is_the_only_rearm_that_clears_manual_override():
    timer = SimpleNamespace(reset=lambda: None)
    tf_buffer = SimpleNamespace(clear=lambda: None)
    scan_barrier = PostResetScanBarrier(clock_tolerance_sec=0.5)
    statuses = []
    node = SimpleNamespace(
        _manual_override=True,
        _published=4,
        complete=True,
        failure="old failure",
        _last_clock=SimpleNamespace(sec=2, nanosec=0),
        _clock_ready_at=None,
        _scan_ready_at=1.0,
        _started_at=0.0,
        _scan_barrier=scan_barrier,
        _tf_buffer=tf_buffer,
        _timer=timer,
        _set_status=statuses.append,
        get_logger=lambda: FakeLogger(),
    )

    result = InitialPosePublisher._rearm(
        node,
        "physical simulation reset event",
        force_calibrated=True,
        barrier_stamp_s=0.2,
    )

    assert result
    assert not node._manual_override
    assert not node.complete
    assert node.failure is None
    assert not scan_barrier.ready
    assert scan_barrier.barrier_stamp_s == pytest.approx(0.2)
    assert statuses == ["waiting_scan"]


def _pose_message(*, frame="map", x=1.0, y=2.0, z=0.0, qz=0.0, qw=1.0):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame),
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=y, z=z),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=qz, w=qw),
            )
        ),
    )


@pytest.mark.parametrize(
    ("message", "expected"),
    [
        (_pose_message(frame="odom"), "frame_id must be 'map'"),
        (_pose_message(x=float("nan")), "must be finite"),
        (_pose_message(qz=0.0, qw=0.5), "must be normalized"),
    ],
)
def test_invalid_manual_pose_never_claims_initial_pose_ownership(
    message, expected
):
    logger = FakeLogger()
    node = SimpleNamespace(
        _map_frame="map",
        _manual_override=False,
        _matches_calibrated_pose=lambda _message: False,
        _manual_pose_error=lambda candidate: (
            InitialPosePublisher._manual_pose_error(node, candidate)
        ),
        get_logger=lambda: logger,
    )

    InitialPosePublisher._initial_pose_callback(node, message)

    assert not node._manual_override
    assert expected in logger.warnings[0]


def test_valid_map_pose_can_claim_manual_ownership():
    logger = FakeLogger()
    statuses = []
    timer = FakeTimer()
    node = SimpleNamespace(
        _map_frame="map",
        _manual_override=False,
        complete=False,
        failure="old",
        _timer=timer,
        _matches_calibrated_pose=lambda _message: False,
        _manual_pose_error=lambda candidate: (
            InitialPosePublisher._manual_pose_error(node, candidate)
        ),
        _set_status=statuses.append,
        get_logger=lambda: logger,
    )

    InitialPosePublisher._initial_pose_callback(node, _pose_message())

    assert node._manual_override
    assert node.complete
    assert node.failure is None
    assert timer.cancelled
    assert statuses == ["manual_override"]


def test_publish_count_one_completes_and_cancels_after_first_tick(monkeypatch):
    timer = FakeTimer()
    statuses = []
    published = []
    logger = FakeLogger()
    node = SimpleNamespace(
        _manual_override=False,
        complete=False,
        _last_clock=SimpleNamespace(sec=1, nanosec=0),
        _started_at=0.0,
        _clock_ready_at=0.0,
        _scan_ready_at=0.0,
        _scan_barrier=SimpleNamespace(ready=True),
        _tf_ready=lambda: True,
        _clear_recoverable_failure=lambda: None,
        _set_status=statuses.append,
        _map_frame="map",
        _pose=SimpleNamespace(
            name="rivermark_start",
            map=SimpleNamespace(position=(1.0, 2.0), yaw_deg=0.0),
            position_stddev_m=0.5,
            yaw_stddev_deg=10.0,
        ),
        _publisher=SimpleNamespace(publish=published.append),
        _published=0,
        _publish_count=1,
        _timer=timer,
        get_logger=lambda: logger,
    )
    monkeypatch.setattr(initial_pose_module.time, "monotonic", lambda: 0.0)

    InitialPosePublisher._tick(node)

    assert len(published) == 1
    assert node._published == 1
    assert node.complete
    assert timer.cancelled
    assert statuses == ["publishing", "complete"]


@pytest.mark.parametrize(
    ("exception", "should_raise"),
    [
        (ExternalShutdownException(), False),
        (RuntimeError("unexpected publisher failure"), True),
    ],
)
def test_main_only_swallows_expected_external_shutdown(
    monkeypatch, exception, should_raise
):
    events = []
    node = SimpleNamespace(
        _stay_alive=True,
        destroy_node=lambda: events.append("destroy"),
    )
    monkeypatch.setattr(
        initial_pose_module, "InitialPosePublisher", lambda: node
    )
    monkeypatch.setattr(
        initial_pose_module.rclpy, "init", lambda args=None: events.append("init")
    )
    monkeypatch.setattr(
        initial_pose_module.rclpy,
        "spin",
        lambda _node: (_ for _ in ()).throw(exception),
    )
    monkeypatch.setattr(initial_pose_module.rclpy, "ok", lambda: True)
    monkeypatch.setattr(
        initial_pose_module.rclpy, "shutdown", lambda: events.append("shutdown")
    )

    if should_raise:
        with pytest.raises(RuntimeError, match="unexpected publisher failure"):
            initial_pose_module.main()
    else:
        initial_pose_module.main()

    assert events == ["init", "destroy", "shutdown"]
