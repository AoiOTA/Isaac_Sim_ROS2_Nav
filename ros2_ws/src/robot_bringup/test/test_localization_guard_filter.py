import math

import pytest

from robot_bringup.localization_guard_filter import ContinuityGuard
from robot_bringup.localization_guard_filter import GuardConfig
from robot_bringup.localization_guard_filter import PlanarPose
from robot_bringup.localization_guard_filter import STATE_HOLDING
from robot_bringup.localization_guard_filter import STATE_INIT
from robot_bringup.localization_guard_filter import STATE_TRACKING


def _config(**overrides):
    values = {
        'accept_translation_m': 0.08,
        'accept_yaw_deg': 3.0,
        'far_translation_m': 0.25,
        'far_yaw_deg': 10.0,
        'far_accept_samples': 3,
        'resume_samples': 2,
        'blend_rate': 0.5,
    }
    values.update(overrides)
    return GuardConfig(**values)


def test_first_candidate_initializes():
    guard = ContinuityGuard(_config())

    assert guard.state == STATE_INIT
    assert guard.observe(PlanarPose(1.0, 2.0, 0.5)) == 'initialize'
    assert guard.state == STATE_TRACKING
    assert guard.estimate == PlanarPose(1.0, 2.0, 0.5)


def test_small_correction_is_blended_toward_candidate():
    guard = ContinuityGuard(_config(blend_rate=0.5))
    guard.observe(PlanarPose(0.0, 0.0, 0.0))

    assert guard.observe(PlanarPose(0.04, -0.02, 0.01)) == 'accept'
    estimate = guard.estimate
    assert math.isclose(estimate.x, 0.02)
    assert math.isclose(estimate.y, -0.01)
    assert math.isclose(estimate.yaw, 0.005)
    assert guard.state == STATE_TRACKING


def test_mid_jump_holds_without_moving_estimate():
    guard = ContinuityGuard(_config())
    guard.observe(PlanarPose(0.0, 0.0, 0.0))

    # 0.15 m is above accept (0.08) but below far (0.25).
    assert guard.observe(PlanarPose(0.15, 0.0, 0.0)) == 'hold_mid'
    assert guard.state == STATE_HOLDING
    assert guard.estimate == PlanarPose(0.0, 0.0, 0.0)


def test_far_jump_holds_then_rebases_after_stable_streak():
    guard = ContinuityGuard(_config(far_accept_samples=3))
    guard.observe(PlanarPose(0.0, 0.0, 0.0))
    far_candidate = PlanarPose(1.0, 0.0, 0.0)

    assert guard.observe(far_candidate) == 'hold_far'
    assert guard.estimate == PlanarPose(0.0, 0.0, 0.0)
    assert guard.observe(far_candidate) == 'hold_far'
    assert guard.observe(far_candidate) == 'rebase'
    assert guard.estimate == far_candidate
    assert guard.state == STATE_TRACKING


def test_far_streak_broken_by_mid_jump():
    guard = ContinuityGuard(_config(far_accept_samples=3))
    guard.observe(PlanarPose(0.0, 0.0, 0.0))

    assert guard.observe(PlanarPose(1.0, 0.0, 0.0)) == 'hold_far'
    assert guard.observe(PlanarPose(0.15, 0.0, 0.0)) == 'hold_mid'
    assert guard.observe(PlanarPose(1.0, 0.0, 0.0)) == 'hold_far'
    assert guard.state == STATE_HOLDING


def test_resume_after_near_streak():
    guard = ContinuityGuard(_config(resume_samples=2))
    guard.observe(PlanarPose(0.0, 0.0, 0.0))
    guard.observe(PlanarPose(0.15, 0.0, 0.0))
    assert guard.state == STATE_HOLDING

    assert guard.observe(PlanarPose(0.15, 0.0, 0.0)) == 'hold_mid'
    assert guard.observe(PlanarPose(0.03, 0.0, 0.0)) == 'accept'
    assert guard.observe(PlanarPose(0.03, 0.0, 0.0)) == 'resume'
    assert guard.state == STATE_TRACKING


def test_reset_returns_to_init():
    guard = ContinuityGuard(_config())
    guard.observe(PlanarPose(1.0, 2.0, 0.5))

    guard.reset()

    assert guard.state == STATE_INIT
    assert guard.estimate is None
    assert guard.observe(PlanarPose(5.0, 5.0, 0.0)) == 'initialize'


def test_config_validation():
    with pytest.raises(ValueError):
        GuardConfig(accept_translation_m=0.3, far_translation_m=0.2)
    with pytest.raises(ValueError):
        GuardConfig(accept_yaw_deg=20.0, far_yaw_deg=10.0)
    with pytest.raises(ValueError):
        GuardConfig(blend_rate=0.0)
    with pytest.raises(ValueError):
        GuardConfig(blend_rate=1.5)
    with pytest.raises(ValueError):
        GuardConfig(far_accept_samples=0)
