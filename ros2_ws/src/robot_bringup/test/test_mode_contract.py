from pathlib import Path

import pytest
from robot_bringup.mode_contract import cognitive_nav2_parameters
from robot_bringup.mode_contract import posegraph_prefix
from robot_bringup.mode_contract import resolve_ekf_profile
from robot_bringup.mode_contract import resolve_route_prior_enabled
from robot_bringup.mode_contract import validate_cognitive_graph_mode
from robot_bringup.mode_contract import validate_cognitive_profile
from robot_bringup.mode_contract import validate_mode
from robot_bringup.mode_contract import validate_nav2_profile
from robot_bringup.mode_contract import validate_nav2_profile_params_file
from robot_bringup.mode_contract import validate_robot_runtime_files
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_nav2_profiles_are_bounded_and_normalized():
    assert validate_nav2_profile(' Stable ') == 'stable'
    assert validate_nav2_profile('PERFORMANCE') == 'performance'
    assert validate_nav2_profile('dynamic_avoidance') == 'dynamic_avoidance'
    assert validate_nav2_profile('v6_low_obstacle_isolation') == \
        'v6_low_obstacle_isolation'
    assert validate_nav2_profile('bio_nav_planning_only') == \
        'bio_nav_planning_only'
    assert validate_nav2_profile('bio_nav_risk_only') == \
        'bio_nav_risk_only'
    assert validate_nav2_profile('bio_nav_tiebreak_risk') == \
        'bio_nav_tiebreak_risk'
    assert validate_nav2_profile('attempt21_static_collection') == \
        'attempt21_static_collection'
    assert validate_nav2_profile('attempt22_reachability_shadow') == \
        'attempt22_reachability_shadow'
    assert validate_nav2_profile('bio_nav_rgbd_risk_shadow') == \
        'bio_nav_rgbd_risk_shadow'
    assert validate_nav2_profile('bio_nav_rgbd_risk_ab') == \
        'bio_nav_rgbd_risk_ab'
    assert validate_nav2_profile('bio_nav_rgbd_risk_static_opt_in') == \
        'bio_nav_rgbd_risk_static_opt_in'
    with pytest.raises(ValueError, match='nav2_profile'):
        validate_nav2_profile('benchmark-custom')


def test_v6_low_obstacle_profile_keeps_valid_mppi_timing():
    profile = validate_nav2_profile_params_file(
        PACKAGE_ROOT.parent
        / 'robot_navigation/config/nav2_v6_low_obstacle_isolation.yaml'
    )
    assert profile.controller_frequency == 10.0
    assert profile.model_dt == 0.10
    assert profile.time_steps == 20
    assert profile.batch_size == 700


def test_m0_m3_contract_drives_final_nav2_modes_and_critic_list():
    modes_file = PACKAGE_ROOT / 'config' / 'modes.yaml'
    expected = {
        'M0': ('off', 'off', False),
        'M1': ('shadow', 'shadow', True),
        'M2': ('active', 'off', True),
        'M3': ('active', 'active', True),
    }
    for name, values in expected.items():
        profile = validate_cognitive_profile(name, modes_file)
        assert (
            profile.obstacle_layer_mode,
            profile.risk_critic_mode,
            profile.module2_enabled,
        ) == values
        final = cognitive_nav2_parameters(profile)
        follow_path = final['controller_server']['ros__parameters'][
            'FollowPath']
        assert follow_path['critics'][-1] == 'CognitiveRiskCritic'
        assert follow_path['CognitiveRiskCritic']['mode'] == values[1]
        for costmap in ('local_costmap', 'global_costmap'):
            assert final[costmap][costmap]['ros__parameters'][
                'cognitive_obstacle_layer']['mode'] == values[0]
            assert final[costmap][costmap]['ros__parameters'][
                'cognitive_obstacle_layer'][
                    'static_track_coalescing_enabled'] is False


def test_static_track_coalescing_is_an_exact_dual_costmap_boolean():
    profile = validate_cognitive_profile(
        'M3', PACKAGE_ROOT / 'config' / 'modes.yaml')
    enabled = cognitive_nav2_parameters(profile, True)
    for costmap in ('local_costmap', 'global_costmap'):
        assert enabled[costmap][costmap]['ros__parameters'][
            'cognitive_obstacle_layer'][
                'static_track_coalescing_enabled'] is True
    with pytest.raises(ValueError, match='must be boolean'):
        cognitive_nav2_parameters(profile, 'true')


def test_graph_mode_is_an_independent_validated_experiment_axis():
    for mode in ('gvg', 'shadow', 'hybrid', 'primary'):
        assert validate_cognitive_graph_mode(mode.upper()) == mode
    with pytest.raises(ValueError, match='cognitive_graph_mode'):
        validate_cognitive_graph_mode('M3')


def test_route_prior_auto_preserves_legacy_module2_coupling():
    assert resolve_route_prior_enabled('auto', True) is True
    assert resolve_route_prior_enabled('AUTO', 'false') is False
    assert resolve_route_prior_enabled('true', False) is True
    assert resolve_route_prior_enabled('false', True) is False
    with pytest.raises(ValueError, match='route_prior_enabled'):
        resolve_route_prior_enabled('shadow', True)
    with pytest.raises(ValueError, match='module2_enabled'):
        resolve_route_prior_enabled('auto', 'sometimes')


def test_final_cognitive_overlay_replaces_the_later_a21_critic_list():
    a21_follow_path = {
        'critics': ['ConstraintCritic', 'VelocityDeadbandCritic'],
        'VelocityDeadbandCritic': {'enabled': True},
    }
    profile = validate_cognitive_profile(
        'M3', PACKAGE_ROOT / 'config' / 'modes.yaml')
    final_follow_path = cognitive_nav2_parameters(profile)[
        'controller_server']['ros__parameters']['FollowPath']
    merged = {**a21_follow_path, **final_follow_path}
    assert merged['VelocityDeadbandCritic']['enabled'] is True
    assert merged['critics'][-2:] == [
        'VelocityDeadbandCritic', 'CognitiveRiskCritic']


def test_estimated_and_legacy_realistic_tf_ownership_modes_are_accepted():
    ideal = validate_mode(
        'mapping', 'ideal', 'isaac', check_posegraph_files=False)
    realistic_isaac = validate_mode(
        'mapping', 'realistic', 'isaac', check_posegraph_files=False)
    realistic_rsp = validate_mode(
        'mapping', 'realistic', 'rsp', check_posegraph_files=False)
    estimated_rsp = validate_mode(
        'mapping', 'estimated', 'rsp', check_posegraph_files=False)
    mixed = validate_mode(
        'mapping', 'mixed', 'isaac', check_posegraph_files=False)
    assert ideal.odometry_mode == 'ideal'
    assert realistic_isaac.structure_tf_source == 'isaac'
    assert realistic_rsp.structure_tf_source == 'rsp'
    assert estimated_rsp.odometry_mode == 'estimated'
    assert mixed.odometry_mode == 'mixed'


def test_ekf_profile_selection_is_bounded_by_all_odometry_modes():
    for mode in ('ideal', 'realistic', 'estimated'):
        assert resolve_ekf_profile(
            mode, 'wheel_imu', 'off', False
        ) == 'wheel_imu'
        assert resolve_ekf_profile(
            mode, 'wheel_imu_lidar', 'rf2o', True
        ) == 'wheel_imu_lidar'
        with pytest.raises(ValueError, match='requires mixed odometry mode'):
            resolve_ekf_profile(
                mode, 'module1_wheel_imu', 'off', False)

    with pytest.raises(ValueError, match='mixed odometry is an Isaac-owned'):
        validate_mode(
            'mapping', 'mixed', 'rsp', check_posegraph_files=False)
    assert resolve_ekf_profile(
        'mixed', 'wheel_imu', 'off', False
    ) == 'module1_wheel_imu'
    assert resolve_ekf_profile(
        'mixed', 'module1_wheel_imu', 'off', False
    ) == 'module1_wheel_imu'
    for profile, backend, validated in (
            ('wheel_imu_lidar', 'off', False),
            ('wheel_imu', 'rf2o', False),
            ('wheel_imu', 'off', True)):
        with pytest.raises(ValueError, match='forbids LiDAR'):
            resolve_ekf_profile(
                'mixed', profile, backend, validated)
    with pytest.raises(ValueError, match='fixes ekf_params_file'):
        resolve_ekf_profile(
            'mixed', 'wheel_imu', 'off', False, '/tmp/custom.yaml')


def test_invalid_choices_and_ideal_rsp_fail_fast():
    with pytest.raises(ValueError, match='operation'):
        validate_mode('invalid', 'ideal', 'isaac')
    with pytest.raises(ValueError, match='odometry_mode'):
        validate_mode('mapping', 'unknown', 'isaac')
    with pytest.raises(ValueError, match='structure_tf_source'):
        validate_mode('mapping', 'ideal', 'both')
    with pytest.raises(ValueError, match='ideal odometry'):
        validate_mode('mapping', 'ideal', 'rsp')


def test_mapping_rejects_posegraph_and_saved_map_modes_require_one():
    with pytest.raises(ValueError, match='must be empty'):
        validate_mode('mapping', 'ideal', 'isaac', '/tmp/map')
    with pytest.raises(ValueError, match='required'):
        validate_mode('localization', 'realistic', 'isaac', '')
    with pytest.raises(ValueError, match='required'):
        validate_mode('navigation', 'realistic', 'rsp', '')
    with pytest.raises(ValueError, match='required'):
        validate_mode('incremental_mapping', 'ideal', 'isaac', '')


def test_posegraph_pair_is_checked_and_extension_is_normalized(tmp_path):
    prefix = tmp_path / 'warehouse_v001'
    prefix.with_suffix('.posegraph').write_bytes(b'posegraph')
    with pytest.raises(ValueError, match='incomplete'):
        validate_mode('localization', 'realistic', 'isaac', str(prefix))

    prefix.with_suffix('.data').write_bytes(b'data')
    occupancy_map = tmp_path / 'warehouse_v001.yaml'
    occupancy_map.write_text('image: warehouse_v001.pgm\n')
    selection = validate_mode(
        'navigation',
        'realistic',
        'rsp',
        str(prefix) + '.posegraph',
        str(occupancy_map),
        check_posegraph_files=False,
    )
    assert selection.posegraph_prefix == str(prefix)
    assert selection.occupancy_map_file == str(occupancy_map)
    incremental = validate_mode(
        'incremental_mapping', 'ideal', 'isaac', str(prefix),
        check_posegraph_files=False)
    assert incremental.posegraph_prefix == str(prefix)
    assert posegraph_prefix(str(prefix) + '.data') == str(prefix)


def test_localization_requires_existing_occupancy_map(tmp_path):
    prefix = tmp_path / 'warehouse_v001'
    prefix.with_suffix('.posegraph').write_bytes(b'posegraph')
    prefix.with_suffix('.data').write_bytes(b'data')
    with pytest.raises(ValueError, match='map_file is required'):
        validate_mode('localization', 'ideal', 'isaac', str(prefix))
    with pytest.raises(ValueError, match='does not exist'):
        validate_mode(
            'localization',
            'ideal',
            'isaac',
            str(prefix),
            str(tmp_path / 'missing.yaml'),
        )


@pytest.mark.parametrize(
    ('odometry_mode', 'localization_owner', 'initial_pose_source',
     'expected_owner'),
    (
        ('mixed', 'auto', 'auto', 'amcl'),
        ('mixed', 'amcl', 'rviz', 'amcl'),
        ('mixed', 'ideal', 'isaac', 'ideal'),
        ('ideal', 'auto', 'auto', 'ideal'),
        ('ideal', 'ideal', 'rviz', 'ideal'),
        ('estimated', 'auto', 'auto', 'amcl'),
        ('realistic', 'amcl', 'isaac', 'amcl'),
    ),
)
def test_occupancy_only_accepts_supported_localization_owner_matrix(
        tmp_path, odometry_mode, localization_owner, initial_pose_source,
        expected_owner):
    occupancy_map = tmp_path / 'rivermark_selected.yaml'
    occupancy_map.write_text('image: rivermark_selected.pgm\n')
    route_graph = tmp_path / 'rivermark_selected.geojson'
    route_graph.write_text('{"type": "FeatureCollection", "features": []}\n')

    selection = validate_mode(
        'navigation',
        odometry_mode,
        'isaac',
        posegraph_file='',
        map_file=str(occupancy_map),
        localization_map_contract='occupancy_only',
        localization_owner=localization_owner,
        initial_pose_source=initial_pose_source,
        route_graph_file=str(route_graph),
    )

    assert selection.posegraph_prefix == ''
    assert selection.occupancy_map_file == str(occupancy_map)
    assert selection.route_graph_file == str(route_graph)
    assert selection.localization_map_contract == 'occupancy_only'
    assert selection.localization_owner == expected_owner
    assert selection.map_manifest_file == ''


@pytest.mark.parametrize('initial_pose_source', ('auto', 'rviz'))
def test_mixed_ideal_requires_isaac_reset_source(
        tmp_path, initial_pose_source):
    occupancy_map = tmp_path / 'rivermark_selected.yaml'
    occupancy_map.write_text('image: rivermark_selected.pgm\n')
    route_graph = tmp_path / 'rivermark_selected.geojson'
    route_graph.write_text('{"type": "FeatureCollection", "features": []}\n')

    with pytest.raises(ValueError, match='initial_pose_source=isaac'):
        validate_mode(
            'navigation',
            'mixed',
            'isaac',
            map_file=str(occupancy_map),
            localization_map_contract='occupancy_only',
            localization_owner='ideal',
            initial_pose_source=initial_pose_source,
            route_graph_file=str(route_graph),
        )


def test_mixed_ideal_rejects_posegraph_bundle_even_with_isaac_reset():
    with pytest.raises(
            ValueError,
            match='localization_map_contract=occupancy_only'):
        validate_mode(
            'navigation',
            'mixed',
            'isaac',
            posegraph_file='/tmp/kujiale',
            map_file='/tmp/kujiale.yaml',
            check_posegraph_files=False,
            localization_map_contract='posegraph_bundle',
            localization_owner='ideal',
            initial_pose_source='isaac',
        )


def test_occupancy_only_rejects_missing_assets_and_wrong_owner(tmp_path):
    occupancy_map = tmp_path / 'rivermark_selected.yaml'
    occupancy_map.write_text('image: rivermark_selected.pgm\n')
    route_graph = tmp_path / 'rivermark_selected.geojson'
    route_graph.write_text('{"type": "FeatureCollection", "features": []}\n')

    with pytest.raises(ValueError, match='occupancy map YAML does not exist'):
        validate_mode(
            'navigation', 'estimated', 'isaac',
            map_file=str(tmp_path / 'missing.yaml'),
            localization_map_contract='occupancy_only',
            localization_owner='amcl',
            route_graph_file=str(route_graph),
        )
    with pytest.raises(ValueError, match='route graph does not exist'):
        validate_mode(
            'navigation', 'estimated', 'isaac',
            map_file=str(occupancy_map),
            localization_map_contract='occupancy_only',
            localization_owner='amcl',
            route_graph_file=str(tmp_path / 'missing.geojson'),
        )
    with pytest.raises(ValueError, match='conflicts with odometry_mode'):
        validate_mode(
            'navigation', 'ideal', 'isaac',
            map_file=str(occupancy_map),
            localization_map_contract='occupancy_only',
            localization_owner='amcl',
            route_graph_file=str(route_graph),
        )
    with pytest.raises(ValueError, match='conflicts with odometry_mode'):
        validate_mode(
            'navigation', 'estimated', 'isaac',
            map_file=str(occupancy_map),
            localization_map_contract='occupancy_only',
            localization_owner='ideal',
            route_graph_file=str(route_graph),
        )


def test_legacy_navigation_defaults_to_posegraph_bundle():
    with pytest.raises(ValueError, match='posegraph_file is required'):
        validate_mode(
            'navigation',
            'estimated',
            'isaac',
            posegraph_file='',
            map_file='/tmp/warehouse_new.yaml',
            route_graph_file='/tmp/warehouse_new.geojson',
        )


def test_documented_mode_matrix_has_no_duplicate_tf_owners():
    document = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'modes.yaml').read_text())
    assert set(document['modes']) == {
        'ideal_isaac', 'realistic_isaac', 'realistic_rsp',
        'estimated_isaac', 'estimated_rsp', 'compute_amcl_dual_odom'}
    mixed = document['modes']['compute_amcl_dual_odom']
    assert mixed == {
        'odometry_mode': 'mixed',
        'structure_tf_source': 'isaac',
        'odom_to_base_publisher': 'Isaac Compute Odometry',
        'map_to_odom_publisher': 'AMCL',
        'fixed_localization': {
            'localization_owner': 'ideal',
            'map_to_odom_publisher': 'ideal_localization_tf',
            'localization_map_contract': 'occupancy_only',
            'initial_pose_source': 'isaac',
        },
        'structure_tf_publisher': 'Isaac Sim',
        'module1_odometry_topic': '/bio_nav/module1/odom',
        'module1_odometry_publisher': 'wheel_imu_ekf',
        'module1_publish_tf': False,
        'lidar_odometry_backend': 'off',
    }
    assert document['operations']['mapping']['publishes_initialpose'] is False
    assert document['operations']['incremental_mapping'][
        'posegraph_required'] is True
    assert document['operations']['localization']['posegraph_required'] is True
    assert document['operations']['localization'][
        'occupancy_map_required'] is True
    assert document['operations']['navigation'][
        'occupancy_map_required'] is True
    assert document['localization_map_contracts']['posegraph_bundle'][
        'posegraph_required'] is True
    assert document['localization_map_contracts']['occupancy_only'] == {
        'localization_owners': ['amcl', 'ideal'],
        'mixed_ideal_initial_pose_source': 'isaac',
        'posegraph_required': False,
        'occupancy_map_required': True,
        'route_graph_required': True,
    }
    assert document['operations']['navigation']['starts_nav2'] is True
    for operation in ('localization', 'navigation'):
        assert document['operations'][operation][
            'localization_backends'] == ['amcl', 'ideal']
        assert document['operations'][operation][
            'localization_backend_default'] == 'amcl'


def test_stable_operation_launch_entries_delegate_to_core_contract():
    launch_dir = PACKAGE_ROOT / 'launch'
    for operation in (
            'mapping', 'incremental_mapping', 'localization', 'navigation'):
        source = (
            launch_dir / f'{operation}_bringup.launch.py').read_text()
        assert "'ros_stack.launch.py'" in source
        assert f"'operation': '{operation}'" in source
        for argument in (
                'robot_description_file',
                'wheel_odometry_params_file',
                'nav2_params_file',
                'interactive',
                'use_rviz',
                'rviz_config',
                'use_teleop',
                'project_root'):
            assert argument in source


def test_robot_runtime_files_are_explicit_and_checked(tmp_path):
    description = tmp_path / 'custom.urdf.xacro'
    wheel_params = tmp_path / 'wheel_odometry.yaml'
    nav2_params = tmp_path / 'nav2.yaml'
    for path in (description, wheel_params, nav2_params):
        path.write_text('placeholder')

    selection = validate_robot_runtime_files(
        str(description), str(wheel_params), str(nav2_params))
    assert selection.description_file == str(description)
    assert selection.wheel_odometry_params_file == str(wheel_params)
    assert selection.nav2_params_file == str(nav2_params)

    with pytest.raises(ValueError, match='does not exist'):
        validate_robot_runtime_files(
            str(tmp_path / 'missing.xacro'),
            str(wheel_params),
            str(nav2_params),
        )
    with pytest.raises(ValueError, match='must be a YAML file'):
        validate_robot_runtime_files(
            str(description), str(description), str(nav2_params))


def test_navigation_uses_activation_gate_instead_of_autostart():
    core_source = (
        PACKAGE_ROOT / 'launch' / 'ros_stack.launch.py').read_text()
    bringup_source = (
        PACKAGE_ROOT / 'launch' / 'navigation_bringup.launch.py').read_text()
    nav_source = (
        PACKAGE_ROOT.parent
        / 'robot_navigation'
        / 'launch'
        / 'navigation.launch.py'
    ).read_text()
    assert "'autostart': 'false'" in core_source
    assert "executable='nav2_activation_gate'" in core_source
    assert "DeclareLaunchArgument('autostart', default_value='false')" \
        in nav_source
    assert "'cognitive_profile_params_file'" in nav_source
    assert nav_source.index('str(a21_overlay),') < nav_source.index(
        'cognitive_profile_params_file,')
    assert "DeclareLaunchArgument('nav2_profile', default_value='stable')" \
        in (PACKAGE_ROOT / 'launch' / 'navigation_bringup.launch.py').read_text()
    assert "DeclareLaunchArgument('nav2_profile_params_file', default_value='')" \
        in (PACKAGE_ROOT / 'launch' / 'navigation_bringup.launch.py').read_text()
    assert 'validate_nav2_profile_params_file(' in core_source
    assert 'invalid nav2_profile_params_file:' in core_source
    assert 'gate_runtime_overlay = ' \
        '_write_activation_gate_runtime_overlay(' in core_source
    assert 'str(gate_runtime_overlay),' in core_source
    assert "'activation_startup_policy', default_value='fail_closed'" \
        in core_source
    assert "'activation_startup_timeout': LaunchConfiguration(" \
        in bringup_source
    assert "'activation_startup_policy': LaunchConfiguration(" \
        in bringup_source


def test_occupancy_only_launch_uses_map_server_amcl_and_no_fake_posegraph():
    core_source = (
        PACKAGE_ROOT / 'launch' / 'ros_stack.launch.py').read_text()
    mapping_source = (
        PACKAGE_ROOT.parent / 'robot_mapping' / 'launch'
        / 'localization.launch.py').read_text()
    navigation_source = (
        PACKAGE_ROOT / 'launch' / 'navigation_bringup.launch.py').read_text()

    assert "'localization_map_contract', default_value='posegraph_bundle'" \
        in core_source
    assert "'localization_backend': selection.localization_owner" \
        in core_source
    assert "'route_graph_file': selection.route_graph_file" in core_source
    assert "selection.odometry_mode in {'realistic', 'estimated', 'mixed'}" \
        in core_source
    assert "executable='map_server'" in mapping_source
    assert "if backend == 'amcl':" in mapping_source
    assert "executable='amcl'" in mapping_source
    assert 'fake_posegraph' not in core_source
    assert "'localization_map_contract': LaunchConfiguration(" \
        in navigation_source


def test_v6_local_arms_do_not_select_the_cognitive_graph_mode():
    core_source = (
        PACKAGE_ROOT / 'launch' / 'ros_stack.launch.py').read_text()
    assert "'cognitive_graph_mode': cognitive_graph_mode" in core_source
    assert 'cognitive_profile.cognitive_graph_mode' not in core_source
    assert "'true' if cognitive_profile.module2_enabled else 'false'" \
        in core_source
    assert "'module2_enabled': module2_enabled" in core_source


def test_only_navigation_enables_the_parallel_nearfield_safety_scan():
    core_source = (
        PACKAGE_ROOT / 'launch' / 'ros_stack.launch.py').read_text()

    assert "'enable_safety_scan': (" in core_source
    assert "'true' if selection.operation == 'navigation' else 'false'" \
        in core_source
    assert "'use_self_filter': use_self_filter" in core_source


def test_ideal_mapping_anchors_map_to_ground_truth_odometry():
    core_source = (
        PACKAGE_ROOT / 'launch' / 'ros_stack.launch.py').read_text()

    assert "'use_scan_matching': (" in core_source
    assert "'do_loop_closing': (" in core_source
    assert "if selection.odometry_mode == 'ideal'" in core_source


def test_three_source_lidar_fusion_requires_explicit_validation():
    launch_dir = PACKAGE_ROOT / 'launch'
    core_source = (launch_dir / 'ros_stack.launch.py').read_text()

    assert "'lidar_odometry_validated', default_value='false'" in core_source
    assert 'validate_lidar_gate(' in core_source
    assert "'ekf_params_file': str(ekf_params_file)" in core_source
    assert 'if ekf_uses_lidar and lidar_odometry_backend' in core_source
    for wrapper in (
            'navigation_bringup.launch.py',
            'localization_bringup.launch.py'):
        source = (launch_dir / wrapper).read_text()
        assert "'lidar_odometry_validated', default_value='false'" in source
        assert "'lidar_odometry_validated': LaunchConfiguration(" in source


def test_estimated_stack_has_one_raw_imu_calibrator_before_unchanged_ekf_input():
    launch_dir = PACKAGE_ROOT / 'launch'
    core_source = (launch_dir / 'ros_stack.launch.py').read_text()
    ekf_config = (
        PACKAGE_ROOT.parent / 'robot_localization_config' / 'config'
        / 'ekf_wheel_imu.yaml').read_text()

    assert core_source.count("executable='imu_yaw_calibrator'") == 1
    assert core_source.index("executable='imu_yaw_calibrator'") < (
        core_source.index("'robot_localization_config',\n            'ekf.launch.py'"))
    assert "'imu_calibration_params_file'," in core_source
    assert "imu0: /imu/data" in ekf_config
    assert "/imu/data_raw" not in ekf_config
    for wrapper_name in ('navigation_bringup', 'localization_bringup'):
        wrapper = (launch_dir / f'{wrapper_name}.launch.py').read_text()
        assert "DeclareLaunchArgument(\n            'imu_calibration_params_file'" in wrapper
        assert "'imu_calibration_params_file': LaunchConfiguration(" in wrapper


def test_mixed_stack_selects_amcl_and_dedicated_module1_ekf_without_lidar():
    core_source = (
        PACKAGE_ROOT / 'launch' / 'ros_stack.launch.py').read_text()
    mapping_source = (
        PACKAGE_ROOT.parent / 'robot_mapping' / 'launch'
        / 'localization.launch.py').read_text()
    activation_source = (
        PACKAGE_ROOT / 'robot_bringup' / 'activation_gate.py').read_text()
    route_source = (
        PACKAGE_ROOT.parent / 'robot_route_planner'
        / 'robot_route_planner' / 'ros_node.py').read_text()
    navigation_root = PACKAGE_ROOT.parent / 'robot_navigation'
    navigation_source = '\n'.join(
        path.read_text(encoding='utf-8')
        for path in navigation_root.rglob('*')
        if path.is_file() and path.suffix in {'.py', '.yaml', '.xml'}
    )

    assert 'ekf_profile = resolve_ekf_profile(' in core_source
    assert "if selection.odometry_mode != 'mixed':" in core_source
    assert "'localization_backend': selection.localization_owner" in core_source
    assert "if backend == 'amcl':" in mapping_source
    assert "Odometry, '/odom'" in activation_source
    assert 'DEFAULT_ROUTE_ODOMETRY_TOPIC = "/odom"' in route_source
    assert '/bio_nav/module1/odom' not in navigation_source
    assert '/ground_truth/' not in activation_source
    assert 'must not use ground-truth data' in route_source


def test_navigation_forwards_region_config_to_route_coordinator_in_mixed_mode():
    launch_dir = PACKAGE_ROOT / 'launch'
    wrapper_source = (
        launch_dir / 'navigation_bringup.launch.py').read_text()
    core_source = (launch_dir / 'ros_stack.launch.py').read_text()
    navigation_source = (
        PACKAGE_ROOT.parent / 'robot_navigation' / 'launch'
        / 'navigation.launch.py').read_text()

    assert "DeclareLaunchArgument('region_config_file', default_value='')" \
        in wrapper_source
    assert "'region_config_file': LaunchConfiguration(" in wrapper_source
    assert "DeclareLaunchArgument('region_config_file', default_value='')" \
        in core_source
    assert "'region_config_file': LaunchConfiguration(" in core_source
    assert "'region_config_file': region_config_file" in navigation_source
    assert "'odometry_topic': '/odom'" in navigation_source


def test_incremental_and_localization_modes_include_initial_pose():
    core_source = (
        PACKAGE_ROOT / 'launch' / 'ros_stack.launch.py').read_text()
    mapping_start = core_source.index(
        "if selection.operation in {'mapping', 'incremental_mapping'}:")
    incremental_start = core_source.index(
        "if (selection.operation == 'incremental_mapping'", mapping_start)
    localization_start = core_source.index('    else:', incremental_start)
    initial_pose = core_source.index("'initial_pose.launch.py'")
    assert incremental_start < initial_pose < localization_start
    assert "'spawn_poses_file'" in core_source
    assert "'spawn_pose_name'" in core_source


def test_posegraph_calibration_is_explicitly_retired_from_localization():
    launch_dir = PACKAGE_ROOT / 'launch'
    core_source = (launch_dir / 'ros_stack.launch.py').read_text()
    localization_source = (
        launch_dir / 'localization_bringup.launch.py').read_text()

    assert 'posegraph_calibration must be true or false' in core_source
    assert 'posegraph_calibration is retired from localization bringup' \
        in core_source
    assert "'posegraph_calibration'" in localization_source


def test_initial_pose_source_is_forwarded_and_rviz_disables_auto_publisher():
    launch_dir = PACKAGE_ROOT / 'launch'
    core_source = (launch_dir / 'ros_stack.launch.py').read_text()
    assert "initial_pose_source not in {'auto', 'rviz', 'isaac'}" in core_source
    assert "if initial_pose_source == 'auto':" in core_source
    assert core_source.count("'initial_pose.launch.py'") == 2
    assert "'initial_pose_source': initial_pose_source" in core_source
    assert "executable='initial_pose_policy'" in core_source
    assert "DeclareLaunchArgument(\n            'map_manifest_file'" \
        in core_source
    assert "initial_pose_source == 'isaac'" not in core_source
    assert 'initial_pose_publish_count' not in core_source
    setup_source = (PACKAGE_ROOT / 'setup.py').read_text()
    assert 'initial_pose_policy = robot_bringup.initial_pose_policy:main' \
        in setup_source
    for operation in (
            'incremental_mapping', 'localization', 'navigation'):
        source = (
            launch_dir / f'{operation}_bringup.launch.py').read_text()
        assert "DeclareLaunchArgument('initial_pose_source'" in source
        assert "'initial_pose_source': LaunchConfiguration(" in source
        assert "DeclareLaunchArgument('map_manifest_file'" in source
        assert "'map_manifest_file': LaunchConfiguration(" in source


def test_core_launch_manages_rviz_and_mapping_only_teleop():
    source = (
        PACKAGE_ROOT / 'launch' / 'ros_stack.launch.py'
    ).read_text(encoding='utf-8')

    assert 'resolve_interactive_selection' in source
    assert "project_root / 'scripts' / 'run_rviz.sh'" in source
    assert "'ISAAC_NAV_DEDICATED_PROCESS_GROUP': '0'" in source
    assert "project_root / 'scripts' / 'run_teleop.sh'" in source
    assert 'teleop_terminal_command' in source
    assert "DeclareLaunchArgument(\n            'interactive'" in source
    assert "DeclareLaunchArgument(\n            'use_rviz'" in source
    assert "DeclareLaunchArgument(\n            'rviz_config'" in source
    assert "DeclareLaunchArgument(\n            'use_teleop'" in source


def test_all_bringup_wrappers_forward_configurable_ceres_threads():
    launch_dir = PACKAGE_ROOT / 'launch'
    core = (launch_dir / 'ros_stack.launch.py').read_text(encoding='utf-8')
    assert "DeclareLaunchArgument('ceres_num_threads', default_value='12')" \
        in core
    for operation in (
            'mapping', 'incremental_mapping', 'localization', 'navigation'):
        wrapper = (
            launch_dir / f'{operation}_bringup.launch.py'
        ).read_text(encoding='utf-8')
        assert "DeclareLaunchArgument('ceres_num_threads', default_value='12')" \
            in wrapper
        assert "'ceres_num_threads': LaunchConfiguration(" in wrapper


def test_mapping_launches_forward_all_runtime_teleop_speed_arguments():
    launch_dir = PACKAGE_ROOT / 'launch'
    argument_defaults = {
        'teleop_linear_speed': '0.50',
        'teleop_angular_speed': '0.80',
        'teleop_linear_speed_step': '0.05',
        'teleop_angular_speed_step': '0.10',
        'teleop_min_linear_speed': '0.10',
        'teleop_min_angular_speed': '0.20',
        'teleop_max_linear_speed': '1.00',
        'teleop_max_angular_speed': '1.50',
    }
    core = (launch_dir / 'ros_stack.launch.py').read_text(encoding='utf-8')
    for name, default in argument_defaults.items():
        assert name in core
        assert f"'{name}', default_value='{default}'" in core
        for operation in ('mapping', 'incremental_mapping'):
            wrapper = (
                launch_dir / f'{operation}_bringup.launch.py'
            ).read_text(encoding='utf-8')
            assert f"'{name}', default_value='{default}'" in wrapper
            assert f"'{name}': LaunchConfiguration(" in wrapper

    assert 'Mapping Teleop is running in a separate terminal.' in core
    assert 'Click the window titled "Isaac Nav Mapping Teleop"' in core
    assert 'before pressing W/A/S/D or the arrow keys.' in core


def test_stack_does_not_try_to_order_shutdown_after_sigint_broadcast():
    source = (
        PACKAGE_ROOT / 'launch' / 'ros_stack.launch.py'
    ).read_text(encoding='utf-8')
    assert 'OnShutdown' not in source
    assert "'robot_bringup.ordered_shutdown'" not in source
