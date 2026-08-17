import hashlib
import json
from pathlib import Path

import pytest
from robot_bringup.map_manifest import compute_bundle_sha256
from robot_bringup.mode_contract import posegraph_prefix
from robot_bringup.mode_contract import validate_mode
from robot_bringup.mode_contract import validate_nav2_profile
from robot_bringup.mode_contract import validate_robot_runtime_files
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = Path(__file__).resolve().parents[4]


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
    assert validate_nav2_profile('estimated_static') == 'estimated_static'
    assert validate_nav2_profile('estimated_dynamic') == 'estimated_dynamic'
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


def test_localization_backend_matrix(tmp_path):
    prefix = tmp_path / 'warehouse_v001'
    prefix.with_suffix('.posegraph').write_bytes(b'posegraph')
    prefix.with_suffix('.data').write_bytes(b'data')
    occupancy_map = tmp_path / 'warehouse_v001.yaml'
    occupancy_map.write_text('image: warehouse_v001.pgm\n')

    # ideal + ideal PASS, explicitly and through the empty-backend default.
    ideal = validate_mode(
        'localization', 'ideal', 'isaac', str(prefix), str(occupancy_map),
        check_posegraph_files=False, localization_backend='ideal')
    assert ideal.localization_backend == 'ideal'
    derived_ideal = validate_mode(
        'localization', 'ideal', 'isaac', str(prefix), str(occupancy_map),
        check_posegraph_files=False)
    assert derived_ideal.localization_backend == 'ideal'

    # realistic + amcl PASS: the occupancy map is required but the serialized
    # pose graph is not, even with file checks enabled.
    amcl = validate_mode(
        'navigation', 'realistic', 'isaac', '', str(occupancy_map),
        localization_backend='amcl')
    assert amcl.localization_backend == 'amcl'
    assert amcl.posegraph_prefix == ''
    assert amcl.occupancy_map_file == str(occupancy_map)

    # realistic + slam_toolbox PASS, explicitly and as the legacy default.
    slam = validate_mode(
        'navigation', 'realistic', 'isaac', str(prefix), str(occupancy_map),
        check_posegraph_files=False, localization_backend='slam_toolbox')
    assert slam.localization_backend == 'slam_toolbox'
    derived_slam = validate_mode(
        'navigation', 'realistic', 'isaac', str(prefix), str(occupancy_map),
        check_posegraph_files=False)
    assert derived_slam.localization_backend == 'slam_toolbox'

    # ideal + amcl PASS: Isaac Compute Odometry owns odom->base_link and the
    # continuity guard owns map->odom from AMCL output.
    ideal_amcl = validate_mode(
        'navigation', 'ideal', 'isaac', '', str(occupancy_map),
        check_posegraph_files=False, localization_backend='amcl')
    assert ideal_amcl.localization_backend == 'amcl'
    assert ideal_amcl.odometry_mode == 'ideal'
    modes_doc = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'modes.yaml').read_text())
    assert modes_doc['modes']['ideal_isaac']['odom_to_base_publisher'] == \
        'Isaac Sim'
    assert modes_doc['localization_backends']['amcl'][
        'map_to_odom_publisher'] == 'localization_continuity_guard'
    assert modes_doc['localization_backends']['amcl'][
        'allowed_odometry_modes'] == ['ideal', 'realistic']

    # ideal + slam_toolbox still FAILs.
    with pytest.raises(ValueError, match='requires realistic odometry'):
        validate_mode(
            'localization', 'ideal', 'isaac', str(prefix),
            str(occupancy_map), check_posegraph_files=False,
            localization_backend='slam_toolbox')

    # amcl without an occupancy map FAILs.
    with pytest.raises(ValueError, match='map_file is required'):
        validate_mode(
            'navigation', 'realistic', 'isaac', '', '',
            localization_backend='amcl')
    with pytest.raises(ValueError, match='does not exist'):
        validate_mode(
            'localization', 'realistic', 'isaac', '',
            str(tmp_path / 'missing.yaml'), localization_backend='amcl')

    # slam_toolbox without a pose graph FAILs.
    with pytest.raises(ValueError, match='posegraph_file is required'):
        validate_mode(
            'navigation', 'realistic', 'isaac', '', str(occupancy_map),
            localization_backend='slam_toolbox')

    # Unknown backend values FAIL.
    with pytest.raises(ValueError, match='localization_backend'):
        validate_mode(
            'mapping', 'ideal', 'isaac', check_posegraph_files=False,
            localization_backend='cartographer')

    # AMCL rejects a SLAM Toolbox pose-graph input.
    with pytest.raises(ValueError, match='must be empty'):
        validate_mode(
            'localization', 'realistic', 'isaac', str(prefix),
            str(occupancy_map), check_posegraph_files=False,
            localization_backend='amcl')


def test_localization_backend_ownership_is_documented():
    document = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'modes.yaml').read_text())
    backends = document['localization_backends']
    assert backends['ideal']['map_to_odom_publisher'] == \
        'ideal_localization_tf'
    assert backends['amcl']['map_to_odom_publisher'] == \
        'localization_continuity_guard'
    assert backends['slam_toolbox']['map_to_odom_publisher'] == \
        'slam_toolbox'
    assert backends['amcl']['requires_posegraph'] is False
    assert backends['amcl']['requires_occupancy_map'] is True
    assert backends['slam_toolbox']['requires_posegraph'] is True


def test_core_launch_plumbs_localization_backend_everywhere():
    launch_dir = PACKAGE_ROOT / 'launch'
    core = (launch_dir / 'ros_stack.launch.py').read_text()
    assert "DeclareLaunchArgument(\n            'localization_backend'," \
        in core
    assert 'localization_backend=LaunchConfiguration(' in core
    # localization.launch.py, navigation.launch.py and nav2_activation_gate
    # all receive the resolved backend, and the mode banner logs it.
    assert core.count("'localization_backend': localization_backend") == 3
    assert 'localization={localization_backend}' in core
    for operation in (
            'mapping', 'incremental_mapping', 'localization', 'navigation'):
        wrapper = (
            launch_dir / f'{operation}_bringup.launch.py').read_text()
        assert "DeclareLaunchArgument('localization_backend', " \
            "default_value='')" in wrapper
        assert "'localization_backend': LaunchConfiguration(" in wrapper


def test_rivermark_launch_supports_the_estimated_localization_chain():
    source = (
        PACKAGE_ROOT / 'launch' / 'rivermark_navigation.launch.py'
    ).read_text()
    assert 'DeclareLaunchArgument("odometry_mode", default_value="ideal")' \
        in source
    assert 'DeclareLaunchArgument("localization_backend", ' \
        'default_value="ideal")' in source
    for name in (
            'wheel_odometry_params_file',
            'ekf_params_file',
            'amcl_params_file'):
        assert f'DeclareLaunchArgument(\n                "{name}",' in source
    assert 'odometry_share / "config" / "wheel_odometry.yaml"' in source
    assert 'localization_share / "config" / "ekf.yaml"' in source
    assert 'mapping_share / "config" / "amcl.yaml"' in source
    assert 'if odometry_mode == "realistic":' in source
    assert '"wheel_odometry.launch.py"' in source
    assert '"ekf.launch.py"' in source
    assert '"localization_backend": localization_backend' in source
    assert 'use_posegraph_localization' not in source
    # Nav2 keeps autostart with the delayed-start mechanism.
    assert '"autostart": "true"' in source
    assert 'TimerAction(\n            period=2.0,' in source


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
    # The calibration diagnostic swaps the map->odom owner to SLAM Toolbox
    # through the backend argument instead of the legacy boolean.
    assert 'if posegraph_calibration' in core_source
    assert "'localization_backend': localization_backend" in core_source
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



def _write_route_graph_bundle(root, version='warehouse_amcl'):
    """Stage an occupancy-only manifest bundle bound to a GVG trio."""
    occupancy = root / 'data/maps/occupancy'
    manifests = root / 'data/maps/manifests'
    graphs = root / 'graphs'
    for directory in (occupancy, manifests, graphs):
        directory.mkdir(parents=True)
    (occupancy / f'{version}.pgm').write_bytes(b'P5\n1 1\n255\n\x00')
    (occupancy / f'{version}.yaml').write_text(
        f'image: {version}.pgm\n'
        'resolution: 0.05\n'
        'origin: [0.0, 0.0, 0.0]\n')
    trio = {
        'geojson': graphs / f'{version}_gvg_v1.geojson',
        'support_map': graphs / f'{version}_gvg_v1_support_map.json',
        'summary': graphs / f'{version}_gvg_v1_summary.json',
    }
    trio['geojson'].write_text(
        '{"type": "FeatureCollection", "features": []}')
    trio['support_map'].write_text('{"nodes": []}')
    trio['summary'].write_text(json.dumps({'map_version': version}))
    occupancy_files = []
    bundle_entries = []
    for role, name in (
            ('yaml', f'{version}.yaml'), ('image', f'{version}.pgm')):
        payload = (occupancy / name).read_bytes()
        occupancy_files.append({
            'role': role,
            'path': f'data/maps/occupancy/{name}',
            'bytes': len(payload),
            'sha256': hashlib.sha256(payload).hexdigest(),
        })
        bundle_entries.append((
            role,
            f'data/maps/occupancy/{name}',
            len(payload),
            occupancy_files[-1]['sha256'],
        ))
    manifest = {
        'schema_version': 1,
        'map_version': version,
        'bundle_sha256': compute_bundle_sha256(bundle_entries),
        'occupancy_grid': {'files': occupancy_files},
        'route_graph': {
            role: {
                'path': path.relative_to(root).as_posix(),
                'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
            }
            for role, path in trio.items()
        },
        'calibration': {
            'calibrated': False,
            'spawn_pose_profile': None,
            'bundle_sha256': None,
            'calibrated_at': None,
            'calibration_method': None,
        },
    }
    manifest_path = manifests / f'{version}.yaml'
    manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False))
    return manifest_path, trio


def _refresh_route_graph_entry(manifest_path, path, role):
    document = yaml.safe_load(manifest_path.read_text())
    document['route_graph'][role]['sha256'] = hashlib.sha256(
        path.read_bytes()).hexdigest()
    manifest_path.write_text(yaml.safe_dump(document, sort_keys=False))


def test_amcl_binds_route_graph_to_the_map_manifest(tmp_path):
    manifest_path, trio = _write_route_graph_bundle(tmp_path)
    map_file = tmp_path / 'data/maps/occupancy/warehouse_amcl.yaml'

    for odometry_mode in ('ideal', 'realistic'):
        selection = validate_mode(
            'navigation', odometry_mode, 'isaac', '', str(map_file),
            localization_backend='amcl',
            route_graph_file=str(trio['geojson']))
        assert selection.map_version == 'warehouse_amcl'
        assert selection.map_bundle_sha256 != ''
        assert selection.map_manifest_file == str(manifest_path)
        assert selection.occupancy_map_file == str(map_file)
        # An empty route_graph_file falls back to the launch default, which
        # is the manifested trio; the trio is still SHA256-verified at load.
        defaulted = validate_mode(
            'navigation', odometry_mode, 'isaac', '', str(map_file),
            localization_backend='amcl')
        assert defaulted.map_version == 'warehouse_amcl'


def test_amcl_rejects_map_graph_mismatch(tmp_path):
    # Forged geojson content breaks the trio SHA256 contract.
    root = tmp_path / 'forged'
    _, trio = _write_route_graph_bundle(root)
    map_file = root / 'data/maps/occupancy/warehouse_amcl.yaml'
    trio['geojson'].write_text(
        '{"type": "FeatureCollection", "features": [1]}')
    with pytest.raises(ValueError, match='SHA256 mismatch'):
        validate_mode(
            'navigation', 'realistic', 'isaac', '', str(map_file),
            localization_backend='amcl',
            route_graph_file=str(trio['geojson']))

    # A summary bound to another map version is rejected.
    root = tmp_path / 'skewed'
    manifest_path, trio = _write_route_graph_bundle(root)
    map_file = root / 'data/maps/occupancy/warehouse_amcl.yaml'
    trio['summary'].write_text(json.dumps({'map_version': 'other_map'}))
    _refresh_route_graph_entry(manifest_path, trio['summary'], 'summary')
    with pytest.raises(ValueError, match='map_version does not match'):
        validate_mode(
            'navigation', 'ideal', 'isaac', '', str(map_file),
            localization_backend='amcl',
            route_graph_file=str(trio['geojson']))

    # A route graph path other than the manifested geojson is rejected.
    root = tmp_path / 'alias'
    _, trio = _write_route_graph_bundle(root)
    map_file = root / 'data/maps/occupancy/warehouse_amcl.yaml'
    elsewhere = root / 'graphs' / 'elsewhere.geojson'
    elsewhere.write_text('{}')
    with pytest.raises(ValueError, match='route_graph_file does not match'):
        validate_mode(
            'navigation', 'realistic', 'isaac', '', str(map_file),
            localization_backend='amcl',
            route_graph_file=str(elsewhere))

    # A hit manifest without a route_graph section cannot bind a graph.
    root = tmp_path / 'bare'
    manifest_path, trio = _write_route_graph_bundle(root)
    map_file = root / 'data/maps/occupancy/warehouse_amcl.yaml'
    document = yaml.safe_load(manifest_path.read_text())
    del document['route_graph']
    manifest_path.write_text(yaml.safe_dump(document, sort_keys=False))
    with pytest.raises(ValueError, match='declares no route_graph'):
        validate_mode(
            'navigation', 'realistic', 'isaac', '', str(map_file),
            localization_backend='amcl',
            route_graph_file=str(trio['geojson']))


def test_repository_warehouse_manifests_bind_their_gvg_trios():
    for version in ('warehouse_new', 'warehouse_new_realistic'):
        map_file = (
            REPOSITORY_ROOT / 'data/maps/occupancy' / f'{version}.yaml')
        graph = (
            REPOSITORY_ROOT / 'ros2_ws/src/robot_route_planner/config'
            / f'{version}_gvg_v1.geojson')
        for odometry_mode in ('ideal', 'realistic'):
            selection = validate_mode(
                'navigation', odometry_mode, 'isaac', '', str(map_file),
                localization_backend='amcl', route_graph_file=str(graph))
            assert selection.map_version == version
        other = 'warehouse_new_realistic' if version == 'warehouse_new' \
            else 'warehouse_new'
        wrong_graph = (
            REPOSITORY_ROOT / 'ros2_ws/src/robot_route_planner/config'
            / f'{other}_gvg_v1.geojson')
        with pytest.raises(ValueError, match='does not match map manifest'):
            validate_mode(
                'navigation', 'realistic', 'isaac', '', str(map_file),
                localization_backend='amcl',
                route_graph_file=str(wrong_graph))
