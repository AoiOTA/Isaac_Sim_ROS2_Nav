from pathlib import Path

import pytest
from robot_bringup.mode_contract import posegraph_prefix
from robot_bringup.mode_contract import validate_mode
from robot_bringup.mode_contract import validate_nav2_profile
from robot_bringup.mode_contract import validate_robot_runtime_files
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_nav2_profiles_are_bounded_and_normalized():
    assert validate_nav2_profile(' Stable ') == 'stable'
    assert validate_nav2_profile('PERFORMANCE') == 'performance'
    assert validate_nav2_profile('dynamic_avoidance') == 'dynamic_avoidance'
    assert validate_nav2_profile('bio_nav_planning_only') == \
        'bio_nav_planning_only'
    assert validate_nav2_profile('bio_nav_risk_only') == \
        'bio_nav_risk_only'
    assert validate_nav2_profile('bio_nav_tiebreak_risk') == \
        'bio_nav_tiebreak_risk'
    with pytest.raises(ValueError, match='nav2_profile'):
        validate_nav2_profile('benchmark-custom')


def test_three_tf_ownership_modes_are_accepted():
    ideal = validate_mode(
        'mapping', 'ideal', 'isaac', check_posegraph_files=False)
    realistic_isaac = validate_mode(
        'mapping', 'realistic', 'isaac', check_posegraph_files=False)
    realistic_rsp = validate_mode(
        'mapping', 'realistic', 'rsp', check_posegraph_files=False)
    assert ideal.odometry_mode == 'ideal'
    assert realistic_isaac.structure_tf_source == 'isaac'
    assert realistic_rsp.structure_tf_source == 'rsp'


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


def test_documented_mode_matrix_has_no_duplicate_tf_owners():
    document = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'modes.yaml').read_text())
    assert set(document['modes']) == {
        'ideal_isaac', 'realistic_isaac', 'realistic_rsp'}
    assert document['operations']['mapping']['publishes_initialpose'] is False
    assert document['operations']['incremental_mapping'][
        'posegraph_required'] is True
    assert document['operations']['localization']['posegraph_required'] is True
    assert document['operations']['localization'][
        'occupancy_map_required'] is True
    assert document['operations']['navigation'][
        'occupancy_map_required'] is True
    assert document['operations']['navigation']['starts_nav2'] is True


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
    assert "parameters=[params_file, profile_params_file]" in nav_source
    assert "DeclareLaunchArgument('nav2_profile', default_value='stable')" \
        in (PACKAGE_ROOT / 'launch' / 'navigation_bringup.launch.py').read_text()
    assert "DeclareLaunchArgument('nav2_profile_params_file', default_value='')" \
        in (PACKAGE_ROOT / 'launch' / 'navigation_bringup.launch.py').read_text()
    assert 'validate_nav2_profile_params_file(' in core_source
    assert 'invalid nav2_profile_params_file:' in core_source


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


def test_ideal_posegraph_calibration_is_explicit_and_localization_only():
    launch_dir = PACKAGE_ROOT / 'launch'
    core_source = (launch_dir / 'ros_stack.launch.py').read_text()
    localization_source = (
        launch_dir / 'localization_bringup.launch.py').read_text()

    assert 'posegraph_calibration must be true or false' in core_source
    assert 'posegraph_calibration is only valid for Ideal localization' \
        in core_source
    assert 'or posegraph_calibration' in core_source
    assert "'posegraph_calibration'" in localization_source


def test_initial_pose_source_is_forwarded_and_rviz_disables_auto_publisher():
    launch_dir = PACKAGE_ROOT / 'launch'
    core_source = (launch_dir / 'ros_stack.launch.py').read_text()
    assert "initial_pose_source not in {'auto', 'rviz'}" in core_source
    assert "if initial_pose_source == 'auto':" in core_source
    assert "'initial_pose_source': initial_pose_source" in core_source
    assert "executable='initial_pose_policy'" in core_source
    assert "DeclareLaunchArgument(\n            'map_manifest_file'" \
        in core_source
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
