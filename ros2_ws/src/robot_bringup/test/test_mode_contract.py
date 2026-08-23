import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
from robot_bringup.mode_contract import cognitive_nav2_parameters
from robot_bringup.mode_contract import posegraph_prefix
from robot_bringup.mode_contract import validate_cognitive_graph_mode
from robot_bringup.mode_contract import validate_cognitive_profile
from robot_bringup.mode_contract import validate_mode
from robot_bringup.mode_contract import validate_nav2_profile
from robot_bringup.mode_contract import validate_nav2_profile_params_file
from robot_bringup.mode_contract import validate_robot_runtime_files
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _load_core_launch():
    launch_file = PACKAGE_ROOT / 'launch' / 'ros_stack.launch.py'
    spec = importlib.util.spec_from_file_location(
        'robot_bringup_ros_stack_launch_for_modes', launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


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
        expected_risk = 'shadow' if values[1] == 'active' else values[1]
        expected_obstacle = 'shadow' if values[0] == 'active' else values[0]
        assert follow_path['CognitiveRiskCritic']['mode'] == expected_risk
        assert 'GoalAngleCritic' not in follow_path['critics']
        for costmap in ('local_costmap', 'global_costmap'):
            assert final[costmap][costmap]['ros__parameters'][
                'cognitive_obstacle_layer']['mode'] == expected_obstacle


@pytest.mark.parametrize('nav2_profile', [
    'stable',
    'v6_low_obstacle_isolation',
])
def test_launch_setup_m0_always_disables_module2(nav2_profile):
    launch_module = _load_core_launch()
    profile = SimpleNamespace(name='M0', module2_enabled=False)
    assert launch_module._resolve_module2_enabled(
        nav2_profile=nav2_profile,
        cognitive_profile=profile,
        requested_value='true',
    ) == 'false'


def test_launch_setup_preserves_m1_m3_module2_resolution():
    launch_module = _load_core_launch()
    for name in ('M1', 'M2', 'M3'):
        profile = SimpleNamespace(name=name, module2_enabled=True)
        assert launch_module._resolve_module2_enabled(
            nav2_profile='stable',
            cognitive_profile=profile,
            requested_value='true',
        ) == 'true'
        assert launch_module._resolve_module2_enabled(
            nav2_profile='stable',
            cognitive_profile=profile,
            requested_value='false',
        ) == 'false'
        assert launch_module._resolve_module2_enabled(
            nav2_profile='v6_low_obstacle_isolation',
            cognitive_profile=profile,
            requested_value='false',
        ) == 'true'


def test_phase1_default_m0_keeps_gvg_route_backend():
    source = (PACKAGE_ROOT / 'launch' / 'ros_stack.launch.py').read_text()
    assert "'cognitive_profile', default_value='M0'" in source
    assert "'cognitive_graph_mode', default_value='gvg'" in source


def test_graph_mode_is_an_independent_validated_experiment_axis():
    for mode in ('gvg', 'shadow', 'hybrid', 'primary'):
        assert validate_cognitive_graph_mode(mode.upper()) == mode
    with pytest.raises(ValueError, match='cognitive_graph_mode'):
        validate_cognitive_graph_mode('M3')


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
    assert ideal.odometry_mode == 'ideal'
    assert realistic_isaac.structure_tf_source == 'isaac'
    assert realistic_rsp.structure_tf_source == 'rsp'
    assert estimated_rsp.odometry_mode == 'estimated'


def test_invalid_choices_and_ideal_rsp_fail_fast():
    with pytest.raises(ValueError, match='operation'):
        validate_mode('invalid', 'ideal', 'isaac')
    with pytest.raises(ValueError, match='odometry_mode'):
        validate_mode('mapping', 'unknown', 'isaac')
    with pytest.raises(ValueError, match='structure_tf_source'):
        validate_mode('mapping', 'ideal', 'both')
    with pytest.raises(ValueError, match='ideal odometry'):
        validate_mode('mapping', 'ideal', 'rsp')


def test_mapping_rejects_posegraph_and_grid_modes_require_occupancy_contract():
    with pytest.raises(ValueError, match='must be empty'):
        validate_mode('mapping', 'ideal', 'isaac', '/tmp/map')
    with pytest.raises(ValueError, match='requires.*occupancy_only'):
        validate_mode('localization', 'realistic', 'isaac', '')
    with pytest.raises(ValueError, match='requires.*occupancy_only'):
        validate_mode('navigation', 'realistic', 'rsp', '')
    with pytest.raises(ValueError, match='required'):
        validate_mode('incremental_mapping', 'ideal', 'isaac', '')


def test_legacy_ideal_posegraph_pair_and_extension_are_normalized(tmp_path):
    prefix = tmp_path / 'warehouse_v001'
    prefix.with_suffix('.posegraph').write_bytes(b'posegraph')
    with pytest.raises(ValueError, match='incomplete'):
        validate_mode('localization', 'ideal', 'isaac', str(prefix))

    prefix.with_suffix('.data').write_bytes(b'data')
    occupancy_map = tmp_path / 'warehouse_v001.yaml'
    occupancy_map.write_text('image: warehouse_v001.pgm\n')
    selection = validate_mode(
        'navigation',
        'ideal',
        'isaac',
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


def test_occupancy_only_accepts_grid_without_posegraph(tmp_path):
    occupancy_map = tmp_path / 'rivermark_selected.yaml'
    occupancy_map.write_text('image: rivermark_selected.pgm\n')
    route_graph = tmp_path / 'rivermark_selected.geojson'
    route_graph.write_text('{"type": "FeatureCollection", "features": []}\n')

    selection = validate_mode(
        'navigation',
        'estimated',
        'isaac',
        posegraph_file='',
        map_file=str(occupancy_map),
        localization_map_contract='occupancy_only',
        localization_owner='grid',
        route_graph_file=str(route_graph),
    )

    assert selection.posegraph_prefix == ''
    assert selection.occupancy_map_file == str(occupancy_map)
    assert selection.route_graph_file == str(route_graph)
    assert selection.localization_map_contract == 'occupancy_only'
    assert selection.localization_owner == 'grid'
    assert selection.map_manifest_file == ''


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
            localization_owner='grid',
            route_graph_file=str(route_graph),
        )
    with pytest.raises(ValueError, match='route graph does not exist'):
        validate_mode(
            'navigation', 'estimated', 'isaac',
            map_file=str(occupancy_map),
            localization_map_contract='occupancy_only',
            localization_owner='grid',
            route_graph_file=str(tmp_path / 'missing.geojson'),
        )
    with pytest.raises(ValueError, match='requires localization_owner=grid'):
        validate_mode(
            'navigation', 'ideal', 'isaac',
            map_file=str(occupancy_map),
            localization_map_contract='occupancy_only',
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
    with pytest.raises(ValueError, match='requires.*occupancy_only'):
        validate_mode(
            'navigation',
            'estimated',
            'isaac',
            posegraph_file='',
            map_file='/tmp/warehouse_new.yaml',
            route_graph_file='/tmp/warehouse_new.geojson',
        )


@pytest.mark.parametrize('retired_owner', ['amcl', 'odom_static'])
def test_retired_localization_owners_are_rejected(retired_owner):
    with pytest.raises(ValueError, match='localization_owner must be one of'):
        validate_mode(
            'navigation',
            'estimated',
            'isaac',
            map_file='/tmp/v6_map.yaml',
            check_posegraph_files=False,
            localization_map_contract='occupancy_only',
            localization_owner=retired_owner,
            route_graph_file='/tmp/v6.geojson',
        )


def test_documented_mode_matrix_has_no_duplicate_tf_owners():
    document = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'modes.yaml').read_text())
    assert set(document['modes']) == {
        'ideal_isaac', 'realistic_isaac', 'realistic_rsp',
        'estimated_isaac', 'estimated_rsp'}
    assert document['operations']['mapping']['publishes_initialpose'] is False
    assert document['operations']['incremental_mapping'][
        'posegraph_required'] is True
    assert document['operations']['localization']['posegraph_required'] is False
    assert document['operations']['localization'][
        'occupancy_map_required'] is True
    assert document['operations']['navigation'][
        'occupancy_map_required'] is True
    assert document['localization_map_contracts']['posegraph_bundle'][
        'posegraph_required'] is True
    assert document['localization_map_contracts']['occupancy_only'] == {
        'localization_owner': 'grid',
        'posegraph_required': False,
        'occupancy_map_required': True,
        'route_graph_required': True,
    }
    assert document['operations']['navigation']['starts_nav2'] is True
    assert document['operations']['localization'][
        'localization_backend'] == 'grid'


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


def test_normal_estimated_launch_uses_grid_backend_and_no_initial_pose():
    core_source = (
        PACKAGE_ROOT / 'launch' / 'ros_stack.launch.py').read_text()
    mapping_source = (
        PACKAGE_ROOT.parent / 'robot_mapping' / 'launch'
        / 'localization.launch.py').read_text()
    navigation_source = (
        PACKAGE_ROOT / 'launch' / 'navigation_bringup.launch.py').read_text()

    assert "'localization_map_contract', default_value='occupancy_only'" \
        in navigation_source
    assert "DeclareLaunchArgument('odometry_mode', default_value='estimated')" \
        in navigation_source
    assert "'localization_backend': selection.localization_owner" \
        in core_source
    assert "'route_graph_file': selection.route_graph_file" in core_source
    assert "selection.odometry_mode in {'realistic', 'estimated'}" \
        in core_source
    assert "executable='map_server'" in mapping_source
    assert "if backend == 'grid':" in mapping_source
    assert "executable='grid_localization_tf_manager'" in mapping_source
    assert 'occupancy_grid_localizer' in mapping_source
    assert 'amcl_params_file' not in core_source
    assert core_source.count("'initial_pose.launch.py'") == 1
    assert core_source.index("'initial_pose.launch.py'") < core_source.index(
        "'robot_mapping',\n                'localization.launch.py'")
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
        core_source.index("'robot_localization_config',\n                'ekf.launch.py'"))
    assert "'imu_calibration_params_file'," in core_source
    assert "imu0: /imu/data" in ekf_config
    assert "/imu/data_raw" not in ekf_config
    for wrapper_name in ('navigation_bringup', 'localization_bringup'):
        wrapper = (launch_dir / f'{wrapper_name}.launch.py').read_text()
        assert "DeclareLaunchArgument(\n            'imu_calibration_params_file'" in wrapper
        assert "'imu_calibration_params_file': LaunchConfiguration(" in wrapper


def test_only_incremental_mapping_includes_initial_pose():
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


def test_initial_pose_source_is_mapping_only():
    launch_dir = PACKAGE_ROOT / 'launch'
    core_source = (launch_dir / 'ros_stack.launch.py').read_text()
    assert "initial_pose_source not in {'auto', 'rviz'}" in core_source
    assert "and initial_pose_source == 'auto'" in core_source
    assert "'initial_pose_source': initial_pose_source" in core_source
    assert "executable='initial_pose_policy'" in core_source
    assert "DeclareLaunchArgument(\n            'map_manifest_file'" \
        in core_source
    setup_source = (PACKAGE_ROOT / 'setup.py').read_text()
    assert 'initial_pose_policy = robot_bringup.initial_pose_policy:main' \
        in setup_source
    for operation in ('incremental_mapping',):
        source = (
            launch_dir / f'{operation}_bringup.launch.py').read_text()
        assert "DeclareLaunchArgument('initial_pose_source'" in source
        assert "'initial_pose_source': LaunchConfiguration(" in source
        assert "DeclareLaunchArgument('map_manifest_file'" in source
        assert "'map_manifest_file': LaunchConfiguration(" in source
    for operation in ('localization', 'navigation'):
        source = (
            launch_dir / f'{operation}_bringup.launch.py').read_text()
        assert "DeclareLaunchArgument('initial_pose_source'" not in source
        assert "'initial_pose_source': LaunchConfiguration(" not in source


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
