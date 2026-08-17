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
        'cluster_trans_m': 0.05,
        'cluster_yaw_deg': 2.0,
        'stable_window_s': 1.0,
        'resume_samples': 2,
        'blend_rate': 0.5,
    }
    values.update(overrides)
    return GuardConfig(**values)


def test_first_candidate_initializes():
    guard = ContinuityGuard(_config())

    assert guard.state == STATE_INIT
    assert guard.observe(PlanarPose(1.0, 2.0, 0.5), 0.0) == 'initialize'
    assert guard.state == STATE_TRACKING
    assert guard.estimate == PlanarPose(1.0, 2.0, 0.5)


def test_small_correction_is_blended_toward_candidate():
    guard = ContinuityGuard(_config(blend_rate=0.5))
    guard.observe(PlanarPose(0.0, 0.0, 0.0), 0.0)

    assert guard.observe(PlanarPose(0.04, -0.02, 0.01), 0.1) == 'accept'
    estimate = guard.estimate
    assert math.isclose(estimate.x, 0.02)
    assert math.isclose(estimate.y, -0.01)
    assert math.isclose(estimate.yaw, 0.005)
    assert guard.state == STATE_TRACKING


def test_mid_jump_holds_without_moving_estimate():
    guard = ContinuityGuard(_config())
    guard.observe(PlanarPose(0.0, 0.0, 0.0), 0.0)

    # 0.15 m is above accept (0.08) but below far (0.25).
    assert guard.observe(PlanarPose(0.15, 0.0, 0.0), 0.1) == 'hold_mid'
    assert guard.state == STATE_HOLDING
    assert guard.estimate == PlanarPose(0.0, 0.0, 0.0)


def test_far_jump_rebases_only_after_full_stable_window():
    guard = ContinuityGuard(_config(stable_window_s=1.0))
    guard.observe(PlanarPose(0.0, 0.0, 0.0), 0.0)
    far_candidate = PlanarPose(1.0, 0.0, 0.0)

    assert guard.observe(far_candidate, 0.1) == 'hold_far'
    assert guard.estimate == PlanarPose(0.0, 0.0, 0.0)
    assert guard.observe(far_candidate, 0.5) == 'hold_far'
    # 1.1 s after the window opened the cluster is stable.
    assert guard.observe(far_candidate, 1.2) == 'rebase'
    # The decision is advisory: the estimate moves only via apply_rebase.
    assert guard.estimate == PlanarPose(0.0, 0.0, 0.0)
    assert guard.state == STATE_HOLDING

    guard.apply_rebase(guard.cluster_mean)
    assert guard.state == STATE_TRACKING
    assert math.isclose(guard.estimate.x, 1.0)
    assert math.isclose(guard.estimate.y, 0.0)
    assert guard.cluster_mean is None


def test_alternating_far_modes_never_rebase():
    """Two alternating wrong modes keep restarting the cluster window."""
    guard = ContinuityGuard(_config(stable_window_s=1.0))
    guard.observe(PlanarPose(0.0, 0.0, 0.0), 0.0)
    mode_a = PlanarPose(1.0, 0.0, 0.0)
    mode_b = PlanarPose(-1.0, 0.0, 0.0)

    for index in range(12):
        candidate = mode_a if index % 2 == 0 else mode_b
        assert guard.observe(candidate, 0.1 + 0.4 * index) == 'hold_far'

    assert guard.estimate == PlanarPose(0.0, 0.0, 0.0)
    assert guard.state == STATE_HOLDING


def test_off_cluster_candidate_restarts_window():
    guard = ContinuityGuard(_config(stable_window_s=1.0))
    guard.observe(PlanarPose(0.0, 0.0, 0.0), 0.0)

    assert guard.observe(PlanarPose(1.0, 0.0, 0.0), 0.1) == 'hold_far'
    assert guard.observe(PlanarPose(1.02, 0.01, 0.0), 0.6) == 'hold_far'
    # 2.0 m away from the cluster mean: the window restarts here.
    assert guard.observe(PlanarPose(2.0, 0.0, 0.0), 0.8) == 'hold_far'
    # 0.9 s after the restart the new window is not yet complete.
    assert guard.observe(PlanarPose(2.01, 0.0, 0.01), 1.5) == 'hold_far'
    # 1.1 s after the restart the new cluster is stable.
    assert guard.observe(PlanarPose(2.01, 0.0, 0.01), 1.9) == 'rebase'
    assert guard.estimate == PlanarPose(0.0, 0.0, 0.0)
    assert guard.cluster_mean.x == pytest.approx((2.0 + 2.01 + 2.01) / 3.0)


def test_far_window_broken_by_mid_jump():
    guard = ContinuityGuard(_config(stable_window_s=1.0))
    guard.observe(PlanarPose(0.0, 0.0, 0.0), 0.0)

    assert guard.observe(PlanarPose(1.0, 0.0, 0.0), 0.1) == 'hold_far'
    assert guard.observe(PlanarPose(0.15, 0.0, 0.0), 0.5) == 'hold_mid'
    # The mid jump cleared the cluster; this far candidate restarts it.
    assert guard.observe(PlanarPose(1.0, 0.0, 0.0), 0.9) == 'hold_far'
    assert guard.state == STATE_HOLDING
    assert guard.observe(PlanarPose(1.0, 0.0, 0.0), 2.0) == 'rebase'


def test_resume_after_near_streak():
    guard = ContinuityGuard(_config(resume_samples=2))
    guard.observe(PlanarPose(0.0, 0.0, 0.0), 0.0)
    guard.observe(PlanarPose(0.15, 0.0, 0.0), 0.1)
    assert guard.state == STATE_HOLDING

    assert guard.observe(PlanarPose(0.15, 0.0, 0.0), 0.2) == 'hold_mid'
    assert guard.observe(PlanarPose(0.03, 0.0, 0.0), 0.3) == 'accept'
    assert guard.observe(PlanarPose(0.03, 0.0, 0.0), 0.4) == 'resume'
    assert guard.state == STATE_TRACKING


def test_hold_freezes_until_resume_streak():
    guard = ContinuityGuard(_config(resume_samples=2))
    guard.observe(PlanarPose(0.0, 0.0, 0.0), 0.0)

    guard.hold()
    assert guard.state == STATE_HOLDING
    assert guard.estimate == PlanarPose(0.0, 0.0, 0.0)

    assert guard.observe(PlanarPose(0.03, 0.0, 0.0), 0.1) == 'accept'
    assert guard.state == STATE_HOLDING
    assert guard.observe(PlanarPose(0.03, 0.0, 0.0), 0.2) == 'resume'
    assert guard.state == STATE_TRACKING


def test_hold_in_init_stays_uninitialized():
    guard = ContinuityGuard(_config())
    guard.hold()
    assert guard.state == STATE_INIT
    assert guard.observe(PlanarPose(1.0, 0.0, 0.0), 0.0) == 'initialize'


def test_reset_returns_to_init():
    guard = ContinuityGuard(_config())
    guard.observe(PlanarPose(1.0, 2.0, 0.5), 0.0)

    guard.reset()

    assert guard.state == STATE_INIT
    assert guard.estimate is None
    assert guard.cluster_mean is None
    assert guard.observe(PlanarPose(5.0, 5.0, 0.0), 0.1) == 'initialize'


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
        GuardConfig(cluster_trans_m=-0.1)
    with pytest.raises(ValueError):
        GuardConfig(cluster_yaw_deg=0.0)
    with pytest.raises(ValueError):
        GuardConfig(stable_window_s=0.0)
    with pytest.raises(ValueError):
        GuardConfig(resume_samples=0)
