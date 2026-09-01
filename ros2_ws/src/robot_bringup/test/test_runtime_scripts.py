from __future__ import annotations

import os
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import time

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[2]
COMMON = REPOSITORY_ROOT / 'scripts' / 'lib' / 'common.sh'
BUILD_ROS2 = REPOSITORY_ROOT / 'scripts' / 'build_ros2.sh'
CLEAN_RUNTIME = REPOSITORY_ROOT / 'scripts' / 'clean_runtime.sh'
PERFORMANCE_MODE = REPOSITORY_ROOT / 'scripts' / 'performance_mode.sh'
RUN_RVIZ = REPOSITORY_ROOT / 'scripts' / 'run_rviz.sh'
RUN_ISAAC = REPOSITORY_ROOT / 'scripts' / 'run_isaac.sh'
RUN_TELEOP = REPOSITORY_ROOT / 'scripts' / 'run_teleop.sh'
RUN_ROS = REPOSITORY_ROOT / 'scripts' / 'run_ros.sh'
RUN_EXPERIMENT = REPOSITORY_ROOT / 'scripts' / 'run_experiment.sh'
RUN_V6_LOW_OBSTACLES = (
    REPOSITORY_ROOT / 'scripts' / 'run_v6_kujiale_low_obstacles.sh')
RUN_V6_RIVERMARK = REPOSITORY_ROOT / 'scripts' / 'run_v6_rivermark.sh'
RUN_RIVERMARK_VISUAL = REPOSITORY_ROOT / 'scripts' / 'run_rivermark_visual.sh'
SAVE_MAP = REPOSITORY_ROOT / 'scripts' / 'save_map.sh'
SETUP_ROS_ENV = REPOSITORY_ROOT / 'scripts' / 'setup_ros_env.sh'
V6_DYNAMIC_STARTUP = (
    REPOSITORY_ROOT / 'scripts' / 'lib' / 'v6_dynamic_startup.sh')


def _environment(**overrides: str) -> dict[str, str]:
    environment = os.environ.copy()
    environment.pop('ROS_DOMAIN_ID', None)
    environment.pop('RMW_IMPLEMENTATION', None)
    environment.update(overrides)
    return environment


def _run_bash(command: str, *, cwd: Path, environment=None):
    return subprocess.run(
        ['bash', '-c', command],
        cwd=cwd,
        env=environment or _environment(),
        text=True,
        capture_output=True,
        check=False,
    )


def test_setup_ros_env_must_be_sourced():
    result = subprocess.run(
        [str(SETUP_ROS_ENV)],
        cwd=REPOSITORY_ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode != 0
    assert 'must be sourced' in result.stderr


def test_setup_ros_env_sets_project_domain_rmw_and_is_idempotent(tmp_path):
    environment = _environment()
    result = _run_bash(
        f'''
        source "{SETUP_ROS_ENV}" >/tmp/setup_ros_env_first.txt
        first_path="$PATH"
        first_ament="$AMENT_PREFIX_PATH"
        source "{SETUP_ROS_ENV}" >/tmp/setup_ros_env_second.txt
        [[ "$PATH" == "$first_path" ]]
        [[ "$AMENT_PREFIX_PATH" == "$first_ament" ]]
        printf '%s|%s|%s|%s' \
          "$PROJECT_ROOT" "$ROS_DISTRO" "$ROS_DOMAIN_ID" \
          "$RMW_IMPLEMENTATION"
        ''',
        cwd=tmp_path,
        environment=environment,
    )
    assert result.returncode == 0, result.stderr
    assert result.stdout == (
        f'{REPOSITORY_ROOT}|jazzy|42|rmw_fastrtps_cpp')


@pytest.mark.parametrize(
    ('name', 'value', 'expected'),
    [
        ('ROS_DOMAIN_ID', '7', 'ROS_DOMAIN_ID must be 42'),
        ('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp',
         'RMW_IMPLEMENTATION must be rmw_fastrtps_cpp'),
    ],
)
def test_setup_ros_env_rejects_conflicts_without_exiting_shell(
        tmp_path, name, value, expected):
    result = _run_bash(
        f'''
        status=0
        source "{SETUP_ROS_ENV}" || status=$?
        printf 'alive:%s' "$status"
        ''',
        cwd=tmp_path,
        environment=_environment(**{name: value}),
    )
    assert result.returncode == 0
    assert result.stdout == 'alive:1'
    assert expected in result.stderr


def test_setup_ros_env_rejects_unbuilt_workspace(tmp_path):
    missing = tmp_path / 'missing-install' / 'setup.bash'
    result = _run_bash(
        f'source "{SETUP_ROS_ENV}"',
        cwd=tmp_path,
        environment=_environment(
            ISAAC_NAV_WORKSPACE_SETUP=str(missing)),
    )
    assert result.returncode != 0
    assert 'ROS workspace is not built' in result.stderr


def test_setup_ros_env_explicit_daemon_restart_uses_selected_cli(tmp_path):
    fake = tmp_path / 'fake_ros2'
    fake.write_text(
        '#!/usr/bin/env bash\nprintf "%s\\n" "$*" >>"$DAEMON_LOG"\n',
        encoding='utf-8',
    )
    fake.chmod(0o755)
    daemon_log = tmp_path / 'daemon.log'
    result = _run_bash(
        f'source "{SETUP_ROS_ENV}" --restart-daemon >/dev/null',
        cwd=tmp_path,
        environment=_environment(
            ISAAC_NAV_ROS2_CLI=str(fake),
            DAEMON_LOG=str(daemon_log),
        ),
    )
    assert result.returncode == 0, result.stderr
    assert daemon_log.read_text(encoding='utf-8').splitlines() == [
        'daemon stop', 'daemon start']


@pytest.mark.parametrize('working_directory', [REPOSITORY_ROOT, Path.home()])
def test_common_resolves_project_root_from_any_directory(working_directory):
    result = _run_bash(
        f'source "{COMMON}"; printf "%s" "$PROJECT_ROOT"',
        cwd=working_directory,
    )
    assert result.returncode == 0, result.stderr
    assert result.stdout == str(REPOSITORY_ROOT)


def test_common_resolves_project_root_from_unrelated_temporary_directory(tmp_path):
    result = _run_bash(
        f'source "{COMMON}"; printf "%s" "$PROJECT_ROOT"',
        cwd=tmp_path,
    )
    assert result.returncode == 0, result.stderr
    assert result.stdout == str(REPOSITORY_ROOT)


def test_common_defaults_integration_root_to_module3_worktree_sibling():
    environment = _environment()
    environment.pop('BIO_NAV_INTEGRATION_ROOT', None)
    environment.pop('BIO_NAV_INTEGRATION_SETUP', None)
    result = _run_bash(
        f'''source "{COMMON}"
        printf '%s|%s' "$BIO_NAV_INTEGRATION_ROOT" \
          "$BIO_NAV_INTEGRATION_SETUP"
        ''',
        cwd=REPOSITORY_ROOT,
        environment=environment,
    )
    integration_root = REPOSITORY_ROOT.parent / 'bio_nav_integration'
    assert result.returncode == 0, result.stderr
    assert result.stdout == (
        f'{integration_root}|'
        f'{integration_root}/ros2_ws/install/local_setup.bash')


def _fake_v6_integration_underlay(tmp_path: Path):
    integration_root = tmp_path / 'cleanup-v6-integration'
    install = integration_root / 'ros2_ws' / 'install'
    setup = install / 'local_setup.bash'
    setup.parent.mkdir(parents=True)
    setup.write_text('export EXPLICIT_INTEGRATION_SETUP=1\n', encoding='utf-8')

    bridge_prefix = install / 'bio_nav_ros_bridge'
    (bridge_prefix / 'share' / 'bio_nav_ros_bridge' / 'config').mkdir(
        parents=True)
    (bridge_prefix / 'share' / 'bio_nav_ros_bridge' / 'config'
     / 'engineering_defaults.yaml').touch()

    interfaces_prefix = install / 'bio_nav_interfaces'
    include = (interfaces_prefix / 'include' / 'bio_nav_interfaces'
               / 'bio_nav_interfaces' / 'msg')
    (include / 'detail').mkdir(parents=True)
    (include / 'local_risk_grid.hpp').touch()
    (include / 'detail' / 'cognitive_obstacle_array__struct.hpp').write_text(
        'bool observation_valid;\n', encoding='utf-8')
    (include / 'detail' / 'planning_prior__struct.hpp').write_text(
        'int local_direction_schema_version;\n', encoding='utf-8')

    fake_bin = tmp_path / 'bin'
    fake_bin.mkdir()
    fake_ros2 = fake_bin / 'ros2'
    fake_ros2.write_text(
        '''#!/usr/bin/env bash
case "$3" in
  bio_nav_ros_bridge) printf '%s\\n' "$FAKE_BRIDGE_PREFIX" ;;
  bio_nav_interfaces) printf '%s\\n' "$FAKE_INTERFACES_PREFIX" ;;
  *) exit 1 ;;
esac
''',
        encoding='utf-8',
    )
    fake_ros2.chmod(0o755)
    environment = _environment(
        BIO_NAV_INTEGRATION_ROOT=str(integration_root),
        BIO_NAV_INTEGRATION_SETUP=str(setup),
        FAKE_BRIDGE_PREFIX=str(bridge_prefix),
        FAKE_INTERFACES_PREFIX=str(interfaces_prefix),
        PATH=f'{fake_bin}:{os.environ["PATH"]}',
    )
    return integration_root, setup, fake_bin, environment


def test_common_accepts_explicit_integration_root_and_setup(tmp_path):
    integration_root, setup, _, environment = _fake_v6_integration_underlay(
        tmp_path)
    result = _run_bash(
        f'''source "{COMMON}"
        source_v6_integration_underlay >/dev/null
        printf '%s|%s|%s' "$BIO_NAV_INTEGRATION_ROOT" \
          "$BIO_NAV_INTEGRATION_SETUP" "$EXPLICIT_INTEGRATION_SETUP"
        ''',
        cwd=tmp_path,
        environment=environment,
    )
    assert result.returncode == 0, result.stderr
    assert result.stdout == f'{integration_root}|{setup}|1'


def test_common_rejects_setup_outside_chosen_integration_root(tmp_path):
    integration_root = tmp_path / 'chosen-integration'
    integration_root.mkdir()
    setup = tmp_path / 'outside' / 'setup.bash'
    setup.parent.mkdir()
    setup.touch()
    result = _run_bash(
        f'source "{COMMON}"; validate_v6_integration_underlay',
        cwd=tmp_path,
        environment=_environment(
            BIO_NAV_INTEGRATION_ROOT=str(integration_root),
            BIO_NAV_INTEGRATION_SETUP=str(setup),
        ),
    )
    assert result.returncode != 0
    assert 'must resolve inside' in result.stderr
    assert str(integration_root) in result.stderr


def test_build_ros2_sources_explicit_integration_root(tmp_path):
    integration_root, _, fake_bin, environment = (
        _fake_v6_integration_underlay(tmp_path))
    ros_setup = tmp_path / 'ros_setup.bash'
    ros_setup.write_text('export ROS_DISTRO=jazzy\n', encoding='utf-8')
    fake_colcon = fake_bin / 'colcon'
    fake_colcon.write_text(
        '#!/usr/bin/env bash\nprintf "%s|%s" '
        '"$EXPLICIT_INTEGRATION_SETUP" "$BIO_NAV_INTEGRATION_ROOT"\n',
        encoding='utf-8',
    )
    fake_colcon.chmod(0o755)
    environment.update({
        'ROS_SETUP': str(ros_setup),
        'ISAAC_NAV_FASTDDS_PROFILE': str(
            REPOSITORY_ROOT
            / 'isaac_sim/configs/ros2_bridge/fastdds_udp_only.xml'),
    })
    result = subprocess.run(
        [str(BUILD_ROS2)],
        cwd=tmp_path,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    assert result.stdout.endswith(f'1|{integration_root}')


def test_common_requires_v6_integration_underlay_interfaces():
    source = COMMON.read_text(encoding='utf-8')
    assert '/worktrees/v6-compute-amcl-dual-odom/bio_nav_integration' not in source
    assert '/repos/' not in source
    assert 'validate_v6_integration_underlay' in source
    assert 'engineering_defaults.yaml' in source
    assert 'ros2 pkg prefix bio_nav_ros_bridge' in source
    assert 'bio_nav_interfaces/msg/local_risk_grid.hpp' in source
    assert 'source_ros --require-integration-underlay' in BUILD_ROS2.read_text(
        encoding='utf-8')


def test_experiment_runner_pins_integration_then_module3_local_overlay(
        tmp_path):
    source = RUN_EXPERIMENT.read_text(encoding='utf-8')
    source_call = (
        'source_ros --require-workspace --require-integration-underlay')
    assert source_call in source
    assert source.index(source_call) < source.index(
        'exec ros2 launch robot_experiments experiment.launch.py')

    integration_root, integration_setup, fake_bin, environment = (
        _fake_v6_integration_underlay(tmp_path))
    source_log = tmp_path / 'source.log'
    ros_setup = tmp_path / 'ros_setup.bash'
    ros_setup.write_text(
        'export ROS_DISTRO=jazzy\n'
        'printf "ros\\n" >>"$SOURCE_LOG"\n',
        encoding='utf-8',
    )
    integration_setup.write_text(
        'printf "integration\\n" >>"$SOURCE_LOG"\n'
        'export AMENT_PREFIX_PATH="$BIO_NAV_INTEGRATION_ROOT/ros2_ws/install"\n'
        'export PYTHONPATH="integration-python"\n',
        encoding='utf-8',
    )
    module3_install = tmp_path / 'module3-install'
    module3_install.mkdir()
    (module3_install / 'setup.bash').write_text(
        'printf "stale-setup\\n" >>"$SOURCE_LOG"\n'
        'export STALE_MODULE3_SETUP=1\n',
        encoding='utf-8',
    )
    (module3_install / 'local_setup.bash').write_text(
        'printf "module3-local\\n" >>"$SOURCE_LOG"\n'
        'export AMENT_PREFIX_PATH="module3:$AMENT_PREFIX_PATH"\n'
        'export PYTHONPATH="module3-python:$PYTHONPATH"\n',
        encoding='utf-8',
    )
    launch_log = tmp_path / 'launch.log'
    fake_ros2 = fake_bin / 'ros2'
    fake_ros2.write_text(
        '''#!/usr/bin/env bash
if [[ "$1" == pkg && "$2" == prefix ]]; then
  case "$3" in
    bio_nav_ros_bridge) printf '%s\n' "$FAKE_BRIDGE_PREFIX" ;;
    bio_nav_interfaces) printf '%s\n' "$FAKE_INTERFACES_PREFIX" ;;
    *) exit 1 ;;
  esac
  exit 0
fi
printf 'AMENT=%s\nPYTHON=%s\nSTALE=%s\n' \
  "${AMENT_PREFIX_PATH:-}" "${PYTHONPATH:-}" \
  "${STALE_MODULE3_SETUP:-}" >"$LAUNCH_LOG"
printf '%s\n' "$@" >>"$LAUNCH_LOG"
''',
        encoding='utf-8',
    )
    fake_ros2.chmod(0o755)
    scenario = tmp_path / 'scenario.yaml'
    scenario.touch()
    spawn = tmp_path / 'spawn.yaml'
    spawn.touch()
    output = tmp_path / 'output'
    environment.update({
        'ROS_SETUP': str(ros_setup),
        'BIO_NAV_INTEGRATION_ROOT': str(integration_root),
        'BIO_NAV_INTEGRATION_SETUP': str(integration_setup),
        'BIO_NAV_MODULE3_INSTALL': str(module3_install),
        'ISAAC_NAV_SPAWN_POSES': str(spawn),
        'ISAAC_NAV_FASTDDS_PROFILE': str(
            REPOSITORY_ROOT
            / 'isaac_sim/configs/ros2_bridge/fastdds_udp_only.xml'),
        'SOURCE_LOG': str(source_log),
        'LAUNCH_LOG': str(launch_log),
        'AMENT_PREFIX_PATH': 'poison-ament',
        'CMAKE_PREFIX_PATH': 'poison-cmake',
        'PYTHONPATH': 'poison-python',
    })

    result = subprocess.run(
        [str(RUN_EXPERIMENT), str(scenario), str(output)],
        cwd=tmp_path,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert source_log.read_text(encoding='utf-8').splitlines() == [
        'ros', 'integration', 'module3-local']
    launch_lines = launch_log.read_text(encoding='utf-8').splitlines()
    assert launch_lines[:3] == [
        (f'AMENT=module3:'
         f'{integration_root}/ros2_ws/install'),
        'PYTHON=module3-python:integration-python',
        'STALE=',
    ]
    assert launch_lines[3:] == [
        'launch',
        'robot_experiments',
        'experiment.launch.py',
        f'scenario_file:={scenario}',
        f'spawn_poses_file:={spawn}',
        f'output_directory:={output}',
    ]


def test_v6_wrapper_separates_local_c_arms_from_explicit_d_graph_modes():
    source = RUN_V6_LOW_OBSTACLES.read_text(encoding='utf-8')
    assert 'run_ros_profile gvg fail_closed auto M3 mixed final' in source
    assert ('run_ros_profile gvg wait_for_seed rviz M1 estimated rf2o-shadow'
            in source)
    assert 'C/shadow entrypoints fix cognitive_graph_mode=gvg' in source
    assert '^(shadow|hybrid|primary)$' in source
    assert ('run_ros_profile "${graph_mode}" fail_closed auto M3 mixed final'
            in source)
    assert 'cognitive_graph_mode:="${graph_mode}"' in source


def test_v6_runner_argv_passes_effective_base_overlay_profile_identity(tmp_path):
    scripts = tmp_path / 'scripts'
    (scripts / 'lib').mkdir(parents=True)
    shutil.copy2(RUN_V6_LOW_OBSTACLES, scripts / RUN_V6_LOW_OBSTACLES.name)
    (scripts / 'lib' / 'common.sh').write_text(
        f'''PROJECT_ROOT="{tmp_path}"
require_file() {{ [[ -f "$1" ]]; }}
die() {{ printf '%s\n' "$*" >&2; return 1; }}
source_ros() {{ :; }}
''',
        encoding='utf-8',
    )
    scenario = (tmp_path / 'ros2_ws/src/robot_experiments/config'
                / 'v6_final_kujiale_static.yaml')
    overlay = (tmp_path / 'ros2_ws/src/robot_navigation/config'
               / 'nav2_v6_low_obstacle_isolation.yaml')
    spawn = (tmp_path / 'isaac_sim/configs/environments'
             / 'kujiale_0026_A_to_B_door_open.v6_isaacgen_v1.spawn.yaml')
    for path in (scenario, overlay, spawn):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.touch()
    fake_ros2 = tmp_path / 'ros2'
    fake_ros2.write_text(
        '#!/usr/bin/env bash\nprintf "%s\\n" "$@"\n',
        encoding='utf-8',
    )
    fake_ros2.chmod(0o755)
    output = tmp_path / 'runs'

    result = subprocess.run(
        [str(scripts / RUN_V6_LOW_OBSTACLES.name), 'runner', str(output)],
        cwd=tmp_path,
        env=_environment(PATH=f'{tmp_path}:{os.environ["PATH"]}'),
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines() == [
        'launch',
        'robot_experiments',
        'experiment.launch.py',
        f'scenario_file:={scenario}',
        f'spawn_poses_file:={spawn}',
        f'output_directory:={output}',
        'nav2_profile:=v6_low_obstacle_isolation',
        f'nav2_config_file:={overlay}',
    ]


def _v6_wrapper_argv(tmp_path: Path, *arguments: str) -> list[str]:
    scripts = tmp_path / 'scripts'
    (scripts / 'lib').mkdir(parents=True)
    (tmp_path / 'ros2_ws' / 'src' / 'robot_experiments' / 'config').mkdir(
        parents=True)
    shutil.copy2(RUN_V6_LOW_OBSTACLES, scripts / RUN_V6_LOW_OBSTACLES.name)
    (scripts / 'lib' / 'common.sh').write_text(
        f'''PROJECT_ROOT="{tmp_path}"
require_file() {{ [[ -f "$1" ]]; }}
require_directory() {{ [[ -d "$1" ]]; }}
die() {{ printf '%s\\n' "$*" >&2; return 1; }}
''',
        encoding='utf-8',
    )
    (tmp_path / 'ros2_ws' / 'src' / 'robot_experiments' / 'config'
     / 'v6_kujiale_low_obstacles_static.yaml').touch()
    module2_root = tmp_path / 'module2'
    (module2_root / 'configs').mkdir(parents=True)
    (module2_root / 'configs'
     / 'kujiale_0026_module1_visual_shadow_v310.yaml').touch()
    fake_run_ros = scripts / 'run_ros.sh'
    fake_run_ros.write_text(
        '#!/usr/bin/env bash\nprintf "%s\\n" "$@"\n',
        encoding='utf-8',
    )
    fake_run_ros.chmod(0o755)

    result = subprocess.run(
        [str(scripts / RUN_V6_LOW_OBSTACLES.name), *arguments],
        cwd=tmp_path,
        env=_environment(BIO_NAV_MODULE2_V310_ROOT=str(module2_root)),
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    return result.stdout.splitlines()


def test_v6_shadow_argv_defaults_to_topic_only_rf2o_with_wheel_imu(tmp_path):
    arguments = _v6_wrapper_argv(tmp_path, 'shadow')

    assert 'ekf_profile:=wheel_imu' in arguments
    assert 'lidar_odometry_backend:=rf2o' in arguments
    assert 'lidar_odometry_validated:=false' in arguments
    assert 'odometry_mode:=estimated' in arguments
    assert 'odometry_mode:=mixed' not in arguments


def test_v6_shadow_trailing_odometry_overrides_remain_last(tmp_path):
    arguments = _v6_wrapper_argv(
        tmp_path,
        'shadow',
        'M2',
        'ekf_profile:=wheel_imu_lidar',
        'lidar_odometry_validated:=true',
    )

    assert 'cognitive_profile:=M2' in arguments
    assert arguments.index('ekf_profile:=wheel_imu') < arguments.index(
        'ekf_profile:=wheel_imu_lidar')
    assert arguments.index('lidar_odometry_validated:=false') < (
        arguments.index('lidar_odometry_validated:=true'))


def test_v6_nonshadow_ros_argv_fixes_final_estimated_policy(tmp_path):
    arguments = _v6_wrapper_argv(tmp_path, 'ros', 'M0')

    assert 'cognitive_profile:=M0' in arguments
    assert 'odometry_mode:=mixed' in arguments
    assert 'odometry_mode:=estimated' not in arguments
    assert 'ekf_profile:=wheel_imu' in arguments
    assert 'lidar_odometry_backend:=off' in arguments
    assert 'lidar_odometry_validated:=false' in arguments
    assert any(argument.startswith('imu_calibration_params_file:=')
               and argument.endswith('/robot_odometry/config/imu_calibration.yaml')
               for argument in arguments)


def test_v6_primary_argv_fixes_final_estimated_policy(tmp_path):
    arguments = _v6_wrapper_argv(tmp_path, 'ros-d', 'primary')

    assert 'cognitive_graph_mode:=primary' in arguments
    assert 'odometry_mode:=mixed' in arguments
    assert 'odometry_mode:=estimated' not in arguments
    assert 'ekf_profile:=wheel_imu' in arguments
    assert 'lidar_odometry_backend:=off' in arguments
    assert 'lidar_odometry_validated:=false' in arguments


def _v6_rivermark_argv(
        tmp_path: Path, *arguments: str) -> tuple[list[str], dict[str, str]]:
    scripts = tmp_path / 'scripts'
    (scripts / 'lib').mkdir(parents=True)
    shutil.copy2(RUN_V6_RIVERMARK, scripts / RUN_V6_RIVERMARK.name)
    (scripts / 'lib' / 'common.sh').write_text(
        f'''PROJECT_ROOT="{tmp_path}"
require_file() {{ [[ -f "$1" ]]; }}
die() {{ printf '%s\\n' "$*" >&2; return 1; }}
''',
        encoding='utf-8',
    )
    demo = tmp_path / 'data' / 'rivermark_demo'
    demo.mkdir(parents=True)
    for name in (
        'rivermark.spawn.yaml',
        'rivermark_selected.yaml',
        'rivermark_selected.geojson',
        'rivermark_regions.yaml',
        'rivermark_demo_goals.yaml',
        'final_rivermark_static_obstacles.yaml',
        'final_rivermark_dynamic.yaml',
        'rivermark_appearance_profiles.yaml',
    ):
        (demo / name).touch()
    environment_usd = tmp_path / 'rivermark.usd'
    environment_usd.touch()
    call_log = tmp_path / 'rivermark_calls.log'
    importer = scripts / 'import_assets.sh'
    importer.write_text(
        '#!/usr/bin/env bash\n'
        'printf "import_assets:%s:asset_root=%s\\n" "$*" '
        '"${ISAAC_ASSET_ROOT:-}" >>"$RIVERMARK_CALL_LOG"\n'
        '[[ "${FAKE_IMPORT_ASSETS_FAIL:-0}" != 1 ]]\n',
        encoding='utf-8',
    )
    importer.chmod(0o755)
    for executable in ('run_isaac.sh', 'run_ros.sh'):
        target = scripts / executable
        target.write_text(
            '#!/usr/bin/env bash\n'
            f'printf "{executable.removesuffix(".sh")}\\n" '
            '>>"$RIVERMARK_CALL_LOG"\n'
            'printf "GROUND_TRUTH=%s\\n" '
            '"${ISAAC_NAV__GROUND_TRUTH__ENABLED:-}"\n'
            'printf "SCENARIO=%s\\n" "${V6_RIVERMARK_SCENARIO:-}"\n'
            'printf "GOALS=%s\\n" "${V6_RIVERMARK_GOALS_FILE:-}"\n'
            'printf "%s\\n" "$@"\n',
            encoding='utf-8',
        )
        target.chmod(0o755)

    result = subprocess.run(
        [str(scripts / RUN_V6_RIVERMARK.name), *arguments],
        cwd=tmp_path,
        env=_environment(
            RIVERMARK_USD=str(environment_usd),
            RIVERMARK_CALL_LOG=str(call_log),
            ISAAC_ASSET_ROOT=str(tmp_path / 'explicit assets'),
        ),
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    lines = result.stdout.splitlines()
    metadata = dict(line.split('=', 1) for line in lines[:3])
    return lines[3:], metadata


def test_v6_rivermark_ros_argv_is_full_mixed_m3_gvg_chain(tmp_path):
    arguments, metadata = _v6_rivermark_argv(
        tmp_path, 'ros', 'static')

    assert arguments[0] == 'navigation'
    assert 'odometry_mode:=mixed' in arguments
    assert 'structure_tf_source:=isaac' in arguments
    assert 'localization_map_contract:=occupancy_only' in arguments
    assert 'localization_owner:=ideal' in arguments
    assert 'localization_profile:=rivermark' in arguments
    assert 'ekf_profile:=wheel_imu' in arguments
    assert 'lidar_odometry_backend:=off' in arguments
    assert 'lidar_odometry_validated:=false' in arguments
    assert any(argument.startswith('imu_calibration_params_file:=')
               and argument.endswith('/robot_odometry/config/imu_calibration.yaml')
               for argument in arguments)
    assert 'nav2_profile:=v6_low_obstacle_isolation' in arguments
    assert not any(argument.startswith('nav2_profile_params_file:=')
                   or argument.startswith('nav2_params_file:=')
                   for argument in arguments)
    assert 'cognitive_profile:=M3' in arguments
    assert 'cognitive_graph_mode:=gvg' in arguments
    assert 'route_prior_enabled:=true' in arguments
    assert 'module2_enabled:=true' in arguments
    assert any(argument.startswith('region_config_file:=')
               and argument.endswith('/rivermark_regions.yaml')
               for argument in arguments)
    assert 'initial_pose_source:=isaac' in arguments
    assert arguments.count('activation_startup_timeout:=240.0') == 1
    assert not any(argument.startswith('activation_startup_policy:=')
                   for argument in arguments)
    assert 'use_rviz:=false' in arguments
    assert not any(argument.startswith('posegraph_file:=')
                   for argument in arguments)
    assert not any('depth' in argument.lower() or 'stvl' in argument.lower()
                   for argument in arguments)
    assert any(argument.endswith('/rivermark_selected.yaml')
               for argument in arguments)
    assert any(argument.endswith('/rivermark_selected.geojson')
               for argument in arguments)
    assert metadata['SCENARIO'] == 'static'
    assert metadata['GOALS'].endswith('/rivermark_demo_goals.yaml')
    assert (tmp_path / 'rivermark_calls.log').read_text(
        encoding='utf-8').splitlines() == ['run_ros']


def test_v6_rivermark_requires_explicit_readable_usd(tmp_path):
    scripts = tmp_path / 'scripts'
    (scripts / 'lib').mkdir(parents=True)
    shutil.copy2(RUN_V6_RIVERMARK, scripts / RUN_V6_RIVERMARK.name)
    (scripts / 'lib' / 'common.sh').write_text(
        f'''PROJECT_ROOT="{tmp_path}"
require_file() {{ [[ -f "$1" ]]; }}
die() {{ printf '%s\\n' "$*" >&2; return 1; }}
''',
        encoding='utf-8',
    )
    environment = _environment()
    environment.pop('RIVERMARK_USD', None)
    result = subprocess.run(
        [str(scripts / RUN_V6_RIVERMARK.name), 'ros', 'static'],
        cwd=tmp_path,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode != 0
    assert 'RIVERMARK_USD must name the frozen Rivermark USD' in result.stderr

    missing = tmp_path / 'missing-rivermark.usd'
    result = subprocess.run(
        [str(scripts / RUN_V6_RIVERMARK.name), 'ros', 'static'],
        cwd=tmp_path,
        env=_environment(RIVERMARK_USD=str(missing)),
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode != 0
    assert f'RIVERMARK_USD is not readable: {missing}' in result.stderr

    source = RUN_V6_RIVERMARK.read_text(encoding='utf-8')
    assert '/home/lyb/Rivermark' not in source


@pytest.mark.parametrize(
    'override',
    ('cognitive_profile:=M3', 'cognitive_graph_mode:=primary',
     'route_prior_enabled:=true',
     'module2_enabled:=true',
     'region_config_file:=/tmp/regions.yaml',
     'initial_pose_source:=auto',
     'activation_startup_timeout:=30.0',
     'activation_startup_policy:=wait_for_seed',
     'nav2_profile_params_file:=/tmp/caller-nav2.yaml',
     'nav2_params_file:=/tmp/caller-nav2.yaml'),
)
def test_v6_rivermark_rejects_final_navigation_overrides(tmp_path, override):
    scripts = tmp_path / 'scripts'
    (scripts / 'lib').mkdir(parents=True)
    shutil.copy2(RUN_V6_RIVERMARK, scripts / RUN_V6_RIVERMARK.name)
    (scripts / 'lib' / 'common.sh').write_text(
        f'''PROJECT_ROOT="{tmp_path}"
require_file() {{ [[ -f "$1" ]]; }}
die() {{ printf '%s\\n' "$*" >&2; return 1; }}
''',
        encoding='utf-8',
    )
    demo = tmp_path / 'data' / 'rivermark_demo'
    demo.mkdir(parents=True)
    for name in (
        'rivermark.spawn.yaml', 'rivermark_selected.yaml',
        'rivermark_selected.geojson', 'rivermark_demo_goals.yaml',
        'rivermark_regions.yaml',
        'final_rivermark_static_obstacles.yaml',
        'final_rivermark_dynamic.yaml', 'rivermark_appearance_profiles.yaml',
    ):
        (demo / name).touch()
    environment_usd = tmp_path / 'rivermark.usd'
    environment_usd.touch()
    result = subprocess.run(
        [str(scripts / RUN_V6_RIVERMARK.name), 'ros', 'static', override],
        cwd=tmp_path,
        env=_environment(RIVERMARK_USD=str(environment_usd)),
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode != 0
    assert 'rejected override' in result.stderr


def test_v6_rivermark_isaac_argv_covers_three_pilot_scenes(tmp_path):
    static, static_metadata = _v6_rivermark_argv(
        tmp_path / 'static', 'isaac', 'static')
    dynamic, dynamic_metadata = _v6_rivermark_argv(
        tmp_path / 'dynamic', 'isaac', 'dynamic')
    appearance, appearance_metadata = _v6_rivermark_argv(
        tmp_path / 'appearance', 'isaac', 'appearance', 'dim_cool')

    for arguments, metadata in (
            (static, static_metadata),
            (dynamic, dynamic_metadata),
            (appearance, appearance_metadata)):
        assert '--environment-usd' in arguments
        assert '--spawn-pose' in arguments
        assert 'rivermark_start' in arguments
        assert '--mode' in arguments
        assert 'mixed' in arguments
        assert '--structure-tf-source' in arguments
        assert 'isaac' in arguments
        assert '--camera-profile' in arguments
        assert 'rgbd_navigation' in arguments
        assert arguments.count('--disable-dlss') == 1
        assert '--no-disable-dlss' not in arguments
        assert metadata['GROUND_TRUTH'] == 'true'

    for run_root in (
            tmp_path / 'static',
            tmp_path / 'dynamic',
            tmp_path / 'appearance'):
        asset_root = run_root / 'explicit assets'
        assert (run_root / 'rivermark_calls.log').read_text(
            encoding='utf-8').splitlines() == [
                f'import_assets::asset_root={asset_root}',
                f'import_assets:--check:asset_root={asset_root}',
                'run_isaac',
            ]

    assert 'final_rivermark_static_obstacles.yaml' in ' '.join(static)
    assert '--dynamic-obstacles' in static
    assert 'final_rivermark_dynamic.yaml' in ' '.join(dynamic)
    assert 'crossing' in dynamic
    assert 'v3' in dynamic
    assert 'final_rivermark_static_obstacles.yaml' in ' '.join(appearance)
    assert '--dynamic-obstacles' in appearance
    assert 'dim_cool' in appearance
    static_root = tmp_path / 'static'
    call_log = static_root / 'rivermark_calls.log'
    call_log.unlink()
    environment = _environment(
        RIVERMARK_USD=str(static_root / 'rivermark.usd'),
        RIVERMARK_CALL_LOG=str(call_log),
        ISAAC_ASSET_ROOT=str(static_root / 'explicit assets'),
        FAKE_IMPORT_ASSETS_FAIL='1',
    )
    failed = subprocess.run(
        [str(static_root / 'scripts' / RUN_V6_RIVERMARK.name),
         'isaac', 'static'],
        cwd=static_root,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert failed.returncode != 0
    assert call_log.read_text(encoding='utf-8').splitlines() == [
        f'import_assets::asset_root={static_root / "explicit assets"}',
    ]

    for override in ('--disable-dlss', '--no-disable-dlss'):
        rejected = subprocess.run(
            [str(static_root / 'scripts' / RUN_V6_RIVERMARK.name),
             'isaac', 'static', override],
            cwd=static_root,
            env=_environment(
                RIVERMARK_USD=str(static_root / 'rivermark.usd'),
                RIVERMARK_CALL_LOG=str(call_log),
                ISAAC_ASSET_ROOT=str(static_root / 'explicit assets'),
            ),
            text=True,
            capture_output=True,
            check=False,
        )
        assert rejected.returncode != 0
        assert f'rejected override: {override}' in rejected.stderr


@pytest.mark.parametrize(
    ('scenario_args', 'expected_call'),
    [
        (('static',), 'module2 static baseline'),
        (('dynamic',), 'module2 dynamic baseline'),
        (('appearance', 'dim_cool'), 'module2 appearance dim_cool'),
    ],
)
def test_attempt31_rivermark_visual_preserves_legacy_demo_defaults(
        tmp_path, scenario_args, expected_call):
    scripts = tmp_path / 'scripts'
    scripts.mkdir()
    shutil.copy2(RUN_RIVERMARK_VISUAL, scripts / RUN_RIVERMARK_VISUAL.name)
    fake_demo = scripts / 'run_rivermark_demo.sh'
    fake_demo.write_text(
        '''#!/usr/bin/env bash
printf 'CALL=%s\n' "$*"
printf 'CONFIG=%s\n' "${RIVERMARK_OBSTACLE_CONFIG-unset}"
printf 'PHYSICAL=%s\n' "${RIVERMARK_PHYSICAL_OBSTACLES-unset}"
printf 'CASE=%s\n' "$RIVERMARK_DYNAMIC_CASE"
printf 'VARIANT=%s\n' "$RIVERMARK_DYNAMIC_VARIANT"
''',
        encoding='utf-8',
    )
    fake_demo.chmod(0o755)

    result = subprocess.run(
        [str(scripts / RUN_RIVERMARK_VISUAL.name), *scenario_args],
        cwd=tmp_path,
        env=_environment(RIVERMARK_VISUAL_REVISION='attempt31'),
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert f'CALL={expected_call}' in result.stdout
    assert 'CONFIG=unset' in result.stdout
    assert 'PHYSICAL=unset' in result.stdout
    assert 'CASE=full_route_four_stage' in result.stdout
    assert 'VARIANT=v3' in result.stdout

    demo_source = (
        REPOSITORY_ROOT / 'scripts' / 'run_rivermark_demo.sh'
    ).read_text(encoding='utf-8')
    assert 'RIVERMARK_OBSTACLE_CONFIG:-${demo_dir}/rivermark_dynamic.yaml' in (
        demo_source
    )
    assert 'physical_obstacles="0"' in demo_source
    assert '[[ "${scenario}" == "dynamic" ]] && physical_obstacles="1"' in (
        demo_source
    )
    assert 'RIVERMARK_DYNAMIC_CASE:-full_route_four_stage' in demo_source


@pytest.mark.parametrize(
    ('name', 'value', 'expected'),
    [
        ('ROS_DOMAIN_ID', '7', 'ROS_DOMAIN_ID must be 42'),
        (
            'RMW_IMPLEMENTATION',
            'rmw_cyclonedds_cpp',
            'RMW_IMPLEMENTATION must be rmw_fastrtps_cpp',
        ),
    ],
)
def test_common_rejects_cross_domain_or_cross_rmw_environment(
    tmp_path, name, value, expected
):
    result = _run_bash(
        f'source "{COMMON}"',
        cwd=tmp_path,
        environment=_environment(**{name: value}),
    )
    assert result.returncode != 0
    assert expected in result.stderr


def test_single_instance_lock_rejects_second_holder_and_releases(tmp_path):
    runtime = tmp_path / 'runtime'
    environment = _environment(ISAAC_NAV_RUNTIME_DIR=str(runtime))
    holder = subprocess.Popen(
        [
            'bash',
            '-c',
            f'source "{COMMON}"; acquire_instance_lock ros "test ROS"; sleep 30',
        ],
        cwd=tmp_path,
        env=environment,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        start_new_session=True,
    )
    try:
        deadline = time.monotonic() + 3.0
        while not (runtime / 'ros.pid').exists() and time.monotonic() < deadline:
            time.sleep(0.02)
        assert (runtime / 'ros.pid').exists()

        duplicate = _run_bash(
            f'source "{COMMON}"; acquire_instance_lock ros "test ROS"',
            cwd=tmp_path,
            environment=environment,
        )
        assert duplicate.returncode != 0
        assert 'test ROS is already running' in duplicate.stderr
    finally:
        os.killpg(holder.pid, signal.SIGTERM)
        holder.wait(timeout=3.0)

    restarted = _run_bash(
        f'source "{COMMON}"; acquire_instance_lock ros "test ROS"',
        cwd=tmp_path,
        environment=environment,
    )
    assert restarted.returncode == 0, restarted.stderr


def _registered_dummy_process(runtime: Path, *, spawn_child: bool = False):
    marker = f'ros2 launch robot_bringup {REPOSITORY_ROOT}'
    if spawn_child:
        child_file = runtime.parent / 'child.pid'
        code = (
            'import pathlib,signal,subprocess,sys,time; '
            f'child=subprocess.Popen([sys.executable,"-c",'
            f'"import time; time.sleep(30)","{marker}"]); '
            f'pathlib.Path("{child_file}").write_text(str(child.pid)); '
            'signal.signal(signal.SIGINT, lambda *_: sys.exit(0)); '
            'signal.signal(signal.SIGTERM, lambda *_: sys.exit(0)); '
            'time.sleep(30)'
        )
    else:
        child_file = None
        code = (
            'import signal,sys,time; '
            'signal.signal(signal.SIGINT, lambda *_: sys.exit(0)); '
            'signal.signal(signal.SIGTERM, lambda *_: sys.exit(0)); '
            'time.sleep(30)'
        )
    process_environment = os.environ.copy()
    process_environment['PROJECT_ROOT'] = str(REPOSITORY_ROOT)
    process = subprocess.Popen(
        [sys.executable, '-c', code, marker],
        start_new_session=True,
        env=process_environment,
    )
    runtime.mkdir(mode=0o700)
    start_ticks = Path(f'/proc/{process.pid}/stat').read_text(
        encoding='utf-8').split()[21]
    boot_id = Path('/proc/sys/kernel/random/boot_id').read_text(
        encoding='utf-8').strip()
    (runtime / 'ros.pid').write_text(
        '\n'.join(
            [
                f'pid={process.pid}',
                f'process_group={os.getpgid(process.pid)}',
                f'leader_start_ticks={start_ticks}',
                f'boot_id={boot_id}',
                'component=ros',
                f'project_root={REPOSITORY_ROOT}',
                'started_at=test',
                '',
            ]
        ),
        encoding='utf-8',
    )
    return process, child_file


def test_clean_runtime_dry_run_then_stops_only_registered_process(tmp_path):
    runtime = tmp_path / 'runtime'
    shm_root = tmp_path / 'shm'
    proc_root = tmp_path / 'proc'
    shm_root.mkdir()
    proc_root.mkdir()
    process, _ = _registered_dummy_process(runtime)
    environment = _environment(
        ISAAC_NAV_RUNTIME_DIR=str(runtime),
        ISAAC_NAV_SHM_ROOT=str(shm_root),
        ISAAC_NAV_DDS_PROC_ROOT=str(proc_root),
    )
    try:
        dry_run = subprocess.run(
            [str(CLEAN_RUNTIME), '--dry-run'],
            cwd=tmp_path,
            env=environment,
            text=True,
            capture_output=True,
            check=False,
        )
        assert dry_run.returncode == 0, dry_run.stderr
        assert 'would stop ros' in dry_run.stdout
        assert process.poll() is None

        cleanup = subprocess.run(
            [str(CLEAN_RUNTIME)],
            cwd=tmp_path,
            env=environment,
            text=True,
            capture_output=True,
            check=False,
            timeout=8.0,
        )
        assert cleanup.returncode == 0, cleanup.stderr
        process.wait(timeout=3.0)
        assert not (runtime / 'ros.pid').exists()
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=3.0)


def test_clean_runtime_stops_registered_process_group_descendants(tmp_path):
    runtime = tmp_path / 'runtime'
    shm_root = tmp_path / 'shm'
    proc_root = tmp_path / 'proc'
    shm_root.mkdir()
    proc_root.mkdir()
    process, child_file = _registered_dummy_process(
        runtime, spawn_child=True)
    environment = _environment(
        ISAAC_NAV_RUNTIME_DIR=str(runtime),
        ISAAC_NAV_SHM_ROOT=str(shm_root),
        ISAAC_NAV_DDS_PROC_ROOT=str(proc_root),
    )
    try:
        deadline = time.monotonic() + 3.0
        while not child_file.exists() and time.monotonic() < deadline:
            time.sleep(0.02)
        assert child_file.exists()
        child_pid = int(child_file.read_text(encoding='utf-8'))

        cleanup = subprocess.run(
            [str(CLEAN_RUNTIME)],
            cwd=tmp_path,
            env=environment,
            text=True,
            capture_output=True,
            check=False,
            timeout=8.0,
        )

        assert cleanup.returncode == 0, cleanup.stderr
        process.wait(timeout=3.0)
        deadline = time.monotonic() + 3.0
        while Path(f'/proc/{child_pid}').exists() and time.monotonic() < deadline:
            time.sleep(0.02)
        assert not Path(f'/proc/{child_pid}').exists()
        assert not (runtime / 'ros.pid').exists()
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=3.0)


def test_clean_runtime_dds_shm_is_dry_run_capable_and_test_root_bounded(tmp_path):
    runtime = tmp_path / 'runtime'
    shm_root = tmp_path / 'shm'
    proc_root = tmp_path / 'proc'
    shm_root.mkdir()
    proc_root.mkdir()
    candidate = shm_root / 'fastrtps_port_test'
    candidate.write_text('stale', encoding='utf-8')
    untouched = shm_root / 'unrelated_file'
    untouched.write_text('keep', encoding='utf-8')
    environment = _environment(
        ISAAC_NAV_RUNTIME_DIR=str(runtime),
        ISAAC_NAV_SHM_ROOT=str(shm_root),
        ISAAC_NAV_DDS_PROC_ROOT=str(proc_root),
    )

    dry_run = subprocess.run(
        [str(CLEAN_RUNTIME), '--dry-run', '--dds-shm'],
        cwd=tmp_path,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert dry_run.returncode == 0, dry_run.stderr
    assert candidate.exists()
    assert 'would remove Fast DDS SHM artifact' in dry_run.stdout

    cleanup = subprocess.run(
        [str(CLEAN_RUNTIME), '--dds-shm'],
        cwd=tmp_path,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert cleanup.returncode == 0, cleanup.stderr
    assert not candidate.exists()
    assert untouched.read_text(encoding='utf-8') == 'keep'


def test_runtime_scripts_use_strict_shell_and_diagnose_is_read_only():
    scripts = [
        *sorted((REPOSITORY_ROOT / 'scripts').glob('*.sh')),
        *sorted((REPOSITORY_ROOT / 'scripts' / 'lib').glob('*.sh')),
    ]
    for script in scripts:
        source = script.read_text(encoding='utf-8')
        if script == SETUP_ROS_ENV:
            # A sourced environment helper must preserve the caller's shell
            # option state instead of globally enabling strict mode.
            assert 'intentionally has no `set -euo pipefail`' in source
        elif script == V6_DYNAMIC_STARTUP:
            # This sourced function library inherits strict mode from each
            # standalone caller instead of changing the caller's options.
            for caller_name in (
                    'run_v6_low_obstacle_phase_f_stack.sh',
                    'run_v6_single_dynamic_low_obstacle.sh'):
                caller = (REPOSITORY_ROOT / 'scripts' / caller_name).read_text(
                    encoding='utf-8')
                assert 'set -Eeuo pipefail' in caller
                assert 'source "${script_dir}/lib/v6_dynamic_startup.sh"' in caller
        else:
            assert 'set -Eeuo pipefail' in source, script
        result = subprocess.run(
            ['bash', '-n', str(script)],
            text=True,
            capture_output=True,
            check=False,
        )
        assert result.returncode == 0, f'{script}: {result.stderr}'

    diagnose = (REPOSITORY_ROOT / 'scripts' / 'diagnose.sh').read_text(
        encoding='utf-8'
    )
    assert 'timeout' in diagnose
    assert 'ros2 lifecycle set' not in diagnose
    assert 'rm -' not in diagnose
    assert 'kill -INT' not in diagnose
    assert 'kill -TERM' not in diagnose
    assert (
        'for candidate in incremental_mapping mapping localization navigation'
        in diagnose
    )
    assert 'Isaac/ROS pair is incomplete' in diagnose
    assert 'nav2_profile="unavailable"' in diagnose
    cleanup = CLEAN_RUNTIME.read_text(encoding='utf-8')
    assert 'for component in teleop rviz ros isaac' in cleanup


def test_performance_mode_enable_is_transactional_and_restore_is_exact(
        tmp_path):
    cpu_root = tmp_path / 'cpu'
    state_dir = tmp_path / 'state'
    policies = {
        'cpu0': ('powersave', 'balance_performance'),
        'cpu1': ('schedutil', 'power'),
    }
    for cpu, (governor, epp) in policies.items():
        cpufreq = cpu_root / cpu / 'cpufreq'
        cpufreq.mkdir(parents=True)
        (cpufreq / 'scaling_governor').write_text(
            governor, encoding='utf-8')
        (cpufreq / 'energy_performance_preference').write_text(
            epp, encoding='utf-8')
        (cpufreq / 'scaling_driver').write_text(
            'test_driver', encoding='utf-8')
    environment = _environment(
        ISAAC_NAV_CPU_SYSFS_ROOT=str(cpu_root),
        ISAAC_NAV_PERFORMANCE_STATE_DIR=str(state_dir),
        ISAAC_NAV_PERFORMANCE_BACKEND='sysfs',
    )

    enabled = subprocess.run(
        [str(PERFORMANCE_MODE), 'enable'],
        cwd=tmp_path,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert enabled.returncode == 0, enabled.stderr
    assert (state_dir / 'performance_mode.state').is_file()
    for cpu in policies:
        cpufreq = cpu_root / cpu / 'cpufreq'
        assert (cpufreq / 'scaling_governor').read_text() == 'performance'
        assert (
            cpufreq / 'energy_performance_preference'
        ).read_text() == 'performance'

    repeated = subprocess.run(
        [str(PERFORMANCE_MODE), 'enable'],
        cwd=tmp_path,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert repeated.returncode != 0
    assert 'run restore before enable' in repeated.stderr

    restored = subprocess.run(
        [str(PERFORMANCE_MODE), 'restore'],
        cwd=tmp_path,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert restored.returncode == 0, restored.stderr
    assert not (state_dir / 'performance_mode.state').exists()
    for cpu, (governor, epp) in policies.items():
        cpufreq = cpu_root / cpu / 'cpufreq'
        assert (cpufreq / 'scaling_governor').read_text() == governor
        assert (
            cpufreq / 'energy_performance_preference'
        ).read_text() == epp


@pytest.mark.parametrize(
    ('script', 'expected'),
    [
        (RUN_RVIZ, 'mapping|incremental_mapping|localization|navigation'),
        (RUN_TELEOP, 'deadman-protected W/A/S/D'),
    ],
)
def test_interactive_script_help_works_from_unrelated_directory(
        tmp_path, script, expected):
    result = subprocess.run(
        [str(script), '--help'],
        cwd=tmp_path,
        env=_environment(),
        text=True,
        capture_output=True,
        check=False,
    )
    assert os.access(script, os.X_OK)
    assert result.returncode == 0, result.stderr
    assert expected in result.stdout


def test_teleop_refuses_noninteractive_stdin_before_joining_ros_graph(tmp_path):
    result = subprocess.run(
        [str(RUN_TELEOP)],
        cwd=tmp_path,
        env=_environment(),
        stdin=subprocess.DEVNULL,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode != 0
    assert 'requires an interactive TTY' in result.stderr


def test_ros_launcher_blocks_mapping_teleop_in_navigation_modes():
    source = (REPOSITORY_ROOT / 'scripts' / 'run_ros.sh').read_text(
        encoding='utf-8')
    assert 'runtime_lock_is_held teleop' in source
    assert 'stop the Mapping teleop before starting' in source


def test_ros_launcher_does_not_leak_its_lock_to_managed_rviz():
    source = (REPOSITORY_ROOT / 'scripts' / 'run_ros.sh').read_text(
        encoding='utf-8')
    assert 'ros_lock_fd="${ISAAC_NAV_LOCK_FDS[-1]}"' in source
    assert 'setsid -- ros2 launch robot_bringup \\' in source
    assert ('"${operation}_bringup.launch.py" "${launch_args[@]}" '
            '{ros_lock_fd}>&- &') in source


def test_ros_launcher_closes_the_identity_checked_managed_rviz_before_lifecycle_shutdown():
    source = (REPOSITORY_ROOT / 'scripts' / 'run_ros.sh').read_text(
        encoding='utf-8')
    assert 'stop_managed_rviz()' in source
    assert 'registered identity no longer matches' in source
    assert 'kill -INT "${rviz_pid}"' in source
    assert source.index('stop_managed_rviz') < source.index(
        'requesting ordered ${operation} lifecycle shutdown')


def test_isaac_launcher_does_not_leak_its_lock_to_kit_children():
    source = RUN_ISAAC.read_text(encoding='utf-8')
    assert 'isaac_lock_fd="${ISAAC_NAV_LOCK_FDS[-1]}"' in source
    assert '"$@" {isaac_lock_fd}>&- &' in source
    assert 'wait "${isaac_pid}"' in source


def test_ros_launcher_defaults_navigation_to_warehouse_new_bundle():
    source = (REPOSITORY_ROOT / 'scripts' / 'run_ros.sh').read_text(
        encoding='utf-8')
    assert 'default_map_version="warehouse_new"' in source
    assert 'posegraph_file:=${posegraph_file}' in source
    assert 'map_file:=${map_file}' in source
    assert 'AMCL for estimated localization' in source
    assert 'ideal|realistic|estimated|mixed' in source
    assert 'mixed mode requires structure_tf_source=isaac' in source
    assert 'mixed mode forbids LiDAR odometry and LiDAR EKF fusion' in source
    assert 'mixed mode fixes ekf_params_file' in source
    assert ('localization_map_contract=occupancy_only requires '
            'localization_owner=amcl') not in source
    assert ('localization_map_contract=occupancy_only requires '
            'AMCL odometry ownership') not in source
