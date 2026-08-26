from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_c4_profile_routes_cognition_through_plugins_not_raw_depth():
    document = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'nav2_v6_low_obstacle_isolation.yaml')
        .read_text(encoding='utf-8')
    )
    controller = document['controller_server']['ros__parameters']['FollowPath']
    assert controller['critics'][-1] == 'CognitiveRiskCritic'
    # Phase 1: cognitive chain neutralized to shadow while the module2-side
    # root cause of the run#7 G1 exit deadlock is fixed; restore 'active'
    # in Phase 2.  Evidence: v6_g1_exit_rootcause_20260823.
    assert controller['CognitiveRiskCritic']['mode'] == 'shadow'
    assert controller['CognitiveRiskCritic']['obstacle_weight'] == 4.0
    assert controller['CognitiveRiskCritic']['direction_weight'] == 0.0
    assert controller['CognitiveRiskCritic']['novelty_weight'] == 0.0
    assert controller['CognitiveRiskCritic']['uncertainty_weight'] == 0.0
    for costmap_name in ('local_costmap', 'global_costmap'):
        params = document[costmap_name][costmap_name]['ros__parameters']
        assert 'cognitive_obstacle_layer' in params['plugins']
        assert params['cognitive_obstacle_layer']['mode'] == 'shadow'
        assert params['cognitive_obstacle_layer'][
            'maximum_ood_probability'] == 0.2
        assert params['depth_voxel_layer']['enabled'] is False
        assert all('stvl' not in plugin.lower() for plugin in params['plugins'])


def test_m0_m3_modes_preserve_shadow_and_control_ownership():
    modes = yaml.safe_load(
        (PACKAGE_ROOT.parent / 'robot_bringup' / 'config' / 'modes.yaml')
        .read_text(encoding='utf-8')
    )['cognitive_profiles']
    assert modes['M0'] == {
        'obstacle_layer_mode': 'off',
        'risk_critic_mode': 'off',
        'module2_enabled': False,
    }
    assert modes['M1']['obstacle_layer_mode'] == 'shadow'
    assert modes['M1']['risk_critic_mode'] == 'shadow'
    assert modes['M1']['module2_enabled'] is True
    assert modes['M2'] == {
        'obstacle_layer_mode': 'active',
        'risk_critic_mode': 'off',
        'module2_enabled': True,
    }
    assert modes['M3'] == {
        'obstacle_layer_mode': 'active',
        'risk_critic_mode': 'active',
        'module2_enabled': True,
    }
