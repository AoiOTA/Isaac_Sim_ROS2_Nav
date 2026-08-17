"""Node-level tests for the continuity guard ROS adapter.

The node is exercised through a SimpleNamespace harness (same pattern as
test_activation_gate.py) so no rclpy context or running middleware is
required; only real message/time types are used.
"""

import math
from collections import deque
from types import MethodType, SimpleNamespace

from builtin_interfaces.msg import Time as TimeMessage
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import TransformException

from robot_bringup.localization_continuity_guard import (
    LocalizationContinuityGuard,
)
from robot_bringup.localization_guard_filter import ContinuityGuard
from robot_bringup.localization_guard_filter import GuardConfig
from robot_bringup.localization_guard_filter import STATE_HOLDING
from robot_bringup.localization_guard_filter import STATE_TRACKING


class _Logger:
    def __init__(self):
        self.messages = []

    def info(self, message, **kwargs):
        self.messages.append(('info', message))

    def warn(self, message, **kwargs):
        self.messages.append(('warn', message))

    def error(self, message):
        self.messages.append(('error', message))


class _Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Broadcaster:
    def __init__(self):
        self.transforms = []

    def sendTransform(self, transform):
        self.transforms.append(transform)


class _FakeClock:
    def __init__(self):
        self.t = 0.0

    def now(self):
        return Time(nanoseconds=int(round(self.t * 1.0e9)))


class _FakeTfBuffer:
    def __init__(self):
        self.transform = None
        self.exception = None

    def lookup_transform(self, target, source, stamp, timeout=None):
        del target, source, timeout
        if self.exception is not None:
            raise self.exception
        return self.transform


def _stamp(value):
    seconds = int(value)
    return TimeMessage(
        sec=seconds,
        nanosec=int(round((value - seconds) * 1.0e9)),
    )


def _odom_tf(stamp, x=0.0, y=0.0, yaw=0.0):
    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.transform.translation.x = x
    transform.transform.translation.y = y
    transform.transform.rotation.z = math.sin(yaw * 0.5)
    transform.transform.rotation.w = math.cos(yaw * 0.5)
    return transform


def _amcl_msg(stamp, x, y, yaw, cov_xx=0.01, cov_yy=0.01, cov_yaw=0.001):
    message = PoseWithCovarianceStamped()
    message.header.stamp = stamp
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.orientation.z = math.sin(yaw * 0.5)
    message.pose.pose.orientation.w = math.cos(yaw * 0.5)
    covariance = [0.0] * 36
    covariance[0] = cov_xx
    covariance[7] = cov_yy
    covariance[35] = cov_yaw
    message.pose.covariance = covariance
    return message


def _config(**overrides):
    values = {
        'accept_translation_m': 0.08,
        'accept_yaw_deg': 3.0,
        'far_translation_m': 0.25,
        'far_yaw_deg': 10.0,
        'cluster_trans_m': 0.05,
        'cluster_yaw_deg': 2.0,
        'stable_window_s': 1.0,
        'resume_samples': 2,
        'blend_rate': 0.5,
    }
    values.update(overrides)
    return GuardConfig(**values)


_BOUND_METHODS = (
    '_current_status',
    '_publish_status',
    '_lookup_odom_to_base',
    '_covariance_within_limits',
    '_on_candidate',
    '_begin_relocalization',
    '_accept_pending_relocalization',
    '_robot_recently_stationary',
    '_sample_odom_pose',
    '_update_watchdog',
    '_on_reset',
    '_publish',
)


def _node_harness(**overrides):
    logger = _Logger()
    clock = _FakeClock()
    node = SimpleNamespace(
        _guard=ContinuityGuard(_config()),
        _map_frame='map',
        _odom_frame='odom',
        _base_frame='base_link',
        _future_dating=Duration(seconds=0.2),
        _max_sigma_xy_m=0.15,
        _max_sigma_yaw_deg=5.0,
        _tf_max_deviation_ms=50.0,
        _amcl_timeout=Duration(seconds=0.5),
        _watchdog_stationary_translation=0.08,
        _watchdog_stationary_yaw=math.radians(3.0),
        _watchdog_stationary_window=1.0,
        _relocalize_settle=Duration(seconds=1.0),
        _allow_autonomous_rebase=True,
        _last_candidate_at=None,
        _recent_odom=deque(),
        _stale=False,
        _pending_relocalization=None,
        _rebase_withheld=False,
        _status=None,
        _tf_buffer=_FakeTfBuffer(),
        _broadcaster=_Broadcaster(),
        _status_publisher=_Recorder(),
        _relocalization_event_publisher=_Recorder(),
        get_logger=lambda: logger,
        get_clock=lambda: clock,
    )
    for name, value in overrides.items():
        setattr(node, name, value)
    for name in _BOUND_METHODS:
        setattr(node, name, MethodType(
            getattr(LocalizationContinuityGuard, name), node))
    return node, logger, clock


def _feed(node, clock, t, x, y, yaw, **covariance):
    clock.t = t
    node._tf_buffer.transform = _odom_tf(_stamp(t))
    node._on_candidate(_amcl_msg(_stamp(t), x, y, yaw, **covariance))


def test_covariance_breach_holds_and_publishes_holding():
    node, logger, clock = _node_harness()
    _feed(node, clock, 0.0, 0.0, 0.0, 0.0)
    assert node._status == 'TRACKING'

    _feed(node, clock, 0.1, 0.0, 0.0, 0.0, cov_xx=0.04)

    assert node._guard.state == STATE_HOLDING
    assert node._status == 'HOLDING'
    assert node._status_publisher.messages[-1].data == 'HOLDING'
    assert any('covariance too large' in message
               for level, message in logger.messages if level == 'warn')
    # The frozen estimate was not touched by the rejected candidate.
    assert math.isclose(node._guard.estimate.x, 0.0)


def test_covariance_yaw_breach_also_holds():
    node, _, clock = _node_harness()
    _feed(node, clock, 0.0, 0.0, 0.0, 0.0)

    _feed(node, clock, 0.1, 0.0, 0.0, 0.0,
          cov_yaw=math.radians(10.0) ** 2)

    assert node._guard.state == STATE_HOLDING
    assert node._status == 'HOLDING'


def test_tf_lookup_failure_keeps_frozen_estimate():
    node, _, clock = _node_harness()
    _feed(node, clock, 0.0, 0.0, 0.0, 0.0)

    node._tf_buffer.exception = TransformException('no transform')
    clock.t = 0.1
    node._on_candidate(_amcl_msg(_stamp(0.1), 1.0, 0.0, 0.0))

    assert node._guard.state == STATE_TRACKING
    assert math.isclose(node._guard.estimate.x, 0.0)
    assert node._status == 'TRACKING'


def test_tf_time_deviation_beyond_limit_rejects_candidate():
    node, _, clock = _node_harness()
    _feed(node, clock, 0.0, 0.0, 0.0, 0.0)

    # The returned transform lags the AMCL stamp by 100 ms (> 50 ms limit).
    node._tf_buffer.transform = _odom_tf(_stamp(0.0))
    clock.t = 0.1
    node._on_candidate(_amcl_msg(_stamp(0.1), 1.0, 0.0, 0.0))

    assert node._guard.state == STATE_TRACKING
    assert math.isclose(node._guard.estimate.x, 0.0)
    assert node._status == 'TRACKING'


def test_stale_watchdog_stops_republication_and_recovers_via_resume():
    node, _, clock = _node_harness()
    _feed(node, clock, 0.0, 0.0, 0.0, 0.0)
    node._publish()
    assert len(node._broadcaster.transforms) == 1

    # No candidate for 1.0 s (> 0.5 s watchdog) while the odom pose moved:
    # STALE and no republish.
    node._tf_buffer.transform = _odom_tf(_stamp(1.0), x=0.5)
    clock.t = 1.0
    node._publish()
    assert node._stale
    assert node._guard.state == STATE_HOLDING
    assert node._status == 'STALE'
    assert len(node._broadcaster.transforms) == 1

    # Fresh candidates clear STALE but must climb the resume streak.
    _feed(node, clock, 1.0, 0.0, 0.0, 0.0)
    assert not node._stale
    assert node._status == 'HOLDING'
    node._publish()
    assert len(node._broadcaster.transforms) == 2

    _feed(node, clock, 1.05, 0.0, 0.0, 0.0)
    assert node._guard.state == STATE_TRACKING
    assert node._status == 'TRACKING'


def test_relocalization_pending_settles_then_rebases():
    node, _, clock = _node_harness()
    _feed(node, clock, 0.0, 0.0, 0.0, 0.0)

    _feed(node, clock, 0.1, 1.0, 0.0, 0.0)
    assert node._pending_relocalization is None
    _feed(node, clock, 0.6, 1.0, 0.0, 0.0)
    _feed(node, clock, 1.2, 1.0, 0.0, 0.0)

    assert node._pending_relocalization is not None
    assert node._status == 'RELOCALIZATION_PENDING'
    events = node._relocalization_event_publisher.messages
    assert len(events) == 1
    assert events[0].header.frame_id == 'map'
    assert math.isclose(events[0].pose.position.x, 1.0)
    assert math.isclose(events[0].pose.position.y, 0.0)
    # The estimate stays frozen during the settle hold.
    assert math.isclose(node._guard.estimate.x, 0.0)

    # Candidates arriving during the settle hold are ignored.
    _feed(node, clock, 1.5, 1.0, 0.0, 0.0)
    assert len(node._relocalization_event_publisher.messages) == 1
    clock.t = 1.5
    node._publish()
    assert math.isclose(
        node._broadcaster.transforms[-1].transform.translation.x, 0.0)

    # The stream stays alive (candidate ignored) while the hold elapses.
    _feed(node, clock, 1.9, 1.0, 0.0, 0.0)

    # After relocalize_settle_s the pending candidate is applied.
    clock.t = 2.3
    node._publish()
    assert node._pending_relocalization is None
    assert node._guard.state == STATE_TRACKING
    assert math.isclose(node._guard.estimate.x, 1.0)
    assert node._status == 'TRACKING'
    assert math.isclose(
        node._broadcaster.transforms[-1].transform.translation.x, 1.0)


def test_pending_relocalization_aborted_when_amcl_goes_stale():
    node, _, clock = _node_harness()
    _feed(node, clock, 0.0, 0.0, 0.0, 0.0)
    _feed(node, clock, 0.1, 1.0, 0.0, 0.0)
    _feed(node, clock, 0.6, 1.0, 0.0, 0.0)
    _feed(node, clock, 1.2, 1.0, 0.0, 0.0)
    assert node._status == 'RELOCALIZATION_PENDING'

    # The stream dies while the robot keeps moving: watchdog fires.
    node._tf_buffer.transform = _odom_tf(_stamp(2.0), x=0.5)
    clock.t = 2.0
    node._publish()
    assert node._stale
    assert node._pending_relocalization is None
    assert node._status == 'STALE'

    # Past the original settle deadline the aborted rebase never applies.
    clock.t = 2.5
    node._publish()
    assert math.isclose(node._guard.estimate.x, 0.0)


def test_stale_watchdog_suppressed_while_robot_stationary():
    node, _, clock = _node_harness()
    _feed(node, clock, 0.0, 0.0, 0.0, 0.0)
    node._publish()
    assert len(node._broadcaster.transforms) == 1

    # No fresh candidates but the odom pose has not moved: AMCL legitimately
    # stops publishing for a stationary robot, so the watchdog stays quiet
    # and the frozen estimate keeps being republished.
    clock.t = 1.0
    node._publish()
    assert not node._stale
    assert node._status == 'TRACKING'
    assert len(node._broadcaster.transforms) == 2

    # Once the odom pose moves without fresh candidates the watchdog fires.
    node._tf_buffer.transform = _odom_tf(_stamp(1.1), x=0.5)
    clock.t = 1.1
    node._publish()
    assert node._stale
    assert node._status == 'STALE'
    assert len(node._broadcaster.transforms) == 2

    # Parking again while STALE resumes republication of the frozen
    # estimate without waiting for fresh AMCL candidates, once the recent
    # odom window contains only parked samples.
    for t in (1.2, 1.5, 2.2):
        node._tf_buffer.transform = _odom_tf(_stamp(t), x=0.5)
        clock.t = t
        node._publish()
    assert not node._stale
    assert len(node._broadcaster.transforms) == 3


def test_relocalization_withheld_when_autonomous_rebase_disabled():
    node, _, clock = _node_harness(_allow_autonomous_rebase=False)
    _feed(node, clock, 0.0, 0.0, 0.0, 0.0)

    _feed(node, clock, 0.1, 1.0, 0.0, 0.0)
    _feed(node, clock, 0.6, 1.0, 0.0, 0.0)
    _feed(node, clock, 1.2, 1.0, 0.0, 0.0)
    assert node._status == 'RELOCALIZATION_PENDING'
    assert len(node._relocalization_event_publisher.messages) == 1

    # Past the settle deadline the estimate stays frozen and no rebase
    # applies; the status leaves PENDING without moving the estimate.
    clock.t = 2.3
    node._publish()
    assert node._pending_relocalization is None
    assert math.isclose(node._guard.estimate.x, 0.0)


def test_reset_authorizes_immediate_reseed():
    node, _, clock = _node_harness()
    _feed(node, clock, 0.0, 0.0, 0.0, 0.0)

    node._on_reset(None)
    assert node._status == 'HOLDING'
    assert node._guard.estimate is None

    # The first post-reset candidate re-initializes without any settle.
    _feed(node, clock, 0.5, 3.0, 2.0, 0.5)
    assert node._guard.state == STATE_TRACKING
    assert node._status == 'TRACKING'
    assert math.isclose(node._guard.estimate.x, 3.0)
    assert node._relocalization_event_publisher.messages == []
