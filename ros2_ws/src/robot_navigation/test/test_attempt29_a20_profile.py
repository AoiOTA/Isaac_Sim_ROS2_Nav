import math
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _profile(name):
    return yaml.safe_load(
        (PACKAGE_ROOT / 'config' / f'nav2_{name}.yaml').read_text())


def test_attempt29_a20_d1_replaces_directional_critics_with_tube_progress():
    candidate = _profile('attempt29_a20_d1')
    follow_path = candidate['controller_server']['ros__parameters'][
        'FollowPath']

    assert follow_path['critics'] == [
        'ConstraintCritic', 'CostCritic', 'GoalCritic', 'GoalAngleCritic',
        'PathAlignCritic', 'PathFollowCritic', 'TubeProgressCritic']
    assert 'PreferForwardCritic' not in follow_path['critics']
    assert 'PathAngleCritic' not in follow_path['critics']
    # No parameter blocks survive for the removed critics either.
    assert 'PreferForwardCritic' not in follow_path
    assert 'PathAngleCritic' not in follow_path

    tube = follow_path['TubeProgressCritic']
    assert tube['enabled'] is True
    assert tube['cost_weight'] == 6.0
    # The progress goal stays active to the end of the path: the critic must
    # not reintroduce the A19 terminal dead band.
    assert 'threshold_to_consider' not in tube


def test_attempt29_a20_d1_uses_box_goal_checker_for_crossings():
    candidate = _profile('attempt29_a20_d1')
    parameters = candidate['controller_server']['ros__parameters']

    assert parameters['goal_checker_plugins'] == [
        'goal_checker', 'a20_crossing_goal_checker']
    checker = parameters['a20_crossing_goal_checker']
    assert checker['plugin'] == 'bio_nav_a20_nav2_plugins::BoxGoalChecker'
    assert checker['stateful'] is False
    assert checker['xy_goal_tolerance_longitudinal'] == 0.05
    assert checker['xy_goal_tolerance_lateral'] == 0.15
    assert math.isclose(checker['yaw_goal_tolerance'], math.pi)


def test_attempt29_a20_d1_keeps_the_validated_a19_d1_envelope():
    candidate = _profile('attempt29_a20_d1')
    a19_d1 = _profile('attempt28_a19_d1')

    local = candidate['/local_costmap/local_costmap']['ros__parameters']
    assert local['width'] == 5
    assert local['height'] == 5
    assert local == a19_d1['/local_costmap/local_costmap']['ros__parameters']
    assert candidate['planner_server']['ros__parameters']['GridBased'][
        'tolerance'] == 0.0

    parameters = candidate['controller_server']['ros__parameters']
    assert parameters['failure_tolerance'] == 0.6
    assert parameters['publish_zero_velocity'] is True
    assert parameters['controller_plugins'] == ['FollowPath']
    assert parameters['progress_checker_plugins'] == ['progress_checker']

    follow_path = parameters['FollowPath']
    a19_follow_path = a19_d1['controller_server']['ros__parameters'][
        'FollowPath']
    assert follow_path['visualize'] is False
    assert follow_path['max_robot_pose_search_dist'] == 0.21
    for key in (
            'vx_std', 'wz_std', 'vx_max', 'vx_min', 'wz_max',
            'ax_max', 'ax_min', 'az_max', 'gamma'):
        assert follow_path[key] == a19_follow_path[key]
    assert follow_path['CostCritic'] == a19_follow_path['CostCritic']
