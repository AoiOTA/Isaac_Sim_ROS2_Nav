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
RUN_V6_LOW_OBSTACLES = (
    REPOSITORY_ROOT / 'scripts' / 'run_v6_kujiale_low_obstacles.sh')
RUN_KUJIALE_ISAAC = REPOSITORY_ROOT / 'scripts' / 'run_kujiale_4x20_isaac.sh'
RUN_V6_R5_SESSION = (
    REPOSITORY_ROOT / 'scripts' / 'v6_reset_cold_boundary_r5_session.sh')
RUN_V6_RIVERMARK = REPOSITORY_ROOT / 'scripts' / 'run_v6_rivermark.sh'
SAVE_MAP = REPOSITORY_ROOT / 'scripts' / 'save_map.sh'
SETUP_ROS_ENV = REPOSITORY_ROOT / 'scripts' / 'setup_ros_env.sh'


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


def test_common_requires_only_the_allowed_v6_integration_underlay():
    source = COMMON.read_text(encoding='utf-8')
    assert '/worktrees/cognitive-navigation/bio_nav_intergration' in source
    assert '/repos/' not in source
    assert 'validate_v6_integration_underlay' in source
    assert 'engineering_defaults.yaml' in source
    assert 'ros2 pkg prefix bio_nav_ros_bridge' in source
    assert 'bio_nav_interfaces/msg/local_risk_grid.hpp' in source
    assert 'source_ros --require-integration-underlay' in BUILD_ROS2.read_text(
        encoding='utf-8')


def test_v6_wrapper_is_canonical_phase1_grid_entry():
    source = RUN_V6_LOW_OBSTACLES.read_text(encoding='utf-8')
    assert 'localization_owner:=grid' in source
    assert 'nav2_profile:=stable' in source
    assert 'cognitive_profile:=M0' in source
    assert 'module2_enabled:=false' in source
    assert 'cognitive_graph_mode:=gvg' in source
    assert 'v6-phase1-empty-room' in source


def _v6_wrapper_argv(tmp_path: Path, *arguments: str) -> list[str]:
    scripts = tmp_path / 'scripts'
    (scripts / 'lib').mkdir(parents=True)
    (tmp_path / 'ros2_ws' / 'src' / 'robot_experiments' / 'config').mkdir(
        parents=True)
    shutil.copy2(RUN_V6_LOW_OBSTACLES, scripts / RUN_V6_LOW_OBSTACLES.name)
    (scripts / 'lib' / 'common.sh').write_text(
        f'''PROJECT_ROOT="{tmp_path}"
require_file() {{ [[ -f "$1" ]]; }}
die() {{ printf '%s\\n' "$*" >&2; return 1; }}
source_ros() {{ :; }}
''',
        encoding='utf-8',
    )
    (tmp_path / 'ros2_ws' / 'src' / 'robot_experiments' / 'config'
     / 'v6_final_kujiale_static.yaml').touch()
    fake_bin = tmp_path / 'bin'
    fake_bin.mkdir()
    fake_ros2 = fake_bin / 'ros2'
    fake_ros2.write_text(
        '#!/usr/bin/env bash\nprintf "%s\\n" "$@"\n',
        encoding='utf-8',
    )
    fake_ros2.chmod(0o755)

    result = subprocess.run(
        [str(scripts / RUN_V6_LOW_OBSTACLES.name), *arguments],
        cwd=tmp_path,
        env=_environment(PATH=f'{fake_bin}:{os.environ["PATH"]}'),
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    return result.stdout.splitlines()


def test_v6_ros_argv_expands_phase1_grid_stable_m0_empty_room(tmp_path):
    arguments = _v6_wrapper_argv(tmp_path, 'ros')

    assert arguments[:3] == ['launch', 'robot_bringup', 'ros_stack.launch.py']
    assert 'localization_owner:=grid' in arguments
    assert 'nav2_profile:=stable' in arguments
    assert 'cognitive_profile:=M0' in arguments
    assert 'module2_enabled:=false' in arguments
    assert 'cognitive_graph_mode:=gvg' in arguments
    assert 'ekf_profile:=wheel_imu' in arguments
    assert 'lidar_odometry_backend:=off' in arguments
    assert 'lidar_odometry_validated:=false' in arguments
    assert any(argument.startswith('imu_calibration_params_file:=')
               and argument.endswith('/robot_odometry/config/imu_calibration.yaml')
               for argument in arguments)


def test_v6_production_scripts_have_no_retired_localization_tokens():
    for script in (RUN_V6_LOW_OBSTACLES, RUN_V6_R5_SESSION):
        source = script.read_text(encoding='utf-8')
        for token in ('/amcl_pose', '/initialpose', 'odom_static'):
            assert token not in source


def test_v6_r5_session_pins_phase1_and_records_grid_topics():
    source = RUN_V6_R5_SESSION.read_text(encoding='utf-8')
    for contract in (
        'V6_LOCALIZATION_BACKEND:-grid',
        'V6_NAV2_PROFILE:-stable',
        'V6_COGNITIVE_PROFILE:-M0',
        'V6_MODULE2_ENABLED:-false',
        'V6_COGNITIVE_GRAPH_MODE:-gvg',
        'V6_LOW_OBSTACLES_ENABLED:-false',
        'V6_DYNAMIC_ACTORS_ENABLED:-false',
        'mission=G1->G2->G3->G4->G5->G1',
        '/flatscan /localization_result /bio_nav/localization/status',
    ):
        assert contract in source
    assert 'start_bg module2' not in source


def test_v6_phase1_isaac_disables_dynamic_actors():
    source = RUN_KUJIALE_ISAAC.read_text(encoding='utf-8')
    assert 'v6-phase1-empty-room)' in source
    assert 'dynamic_arguments=(--no-dynamic-obstacles)' in source
    assert 'kujiale_0026_A_to_B_door_open.v6_isaacgen_v1.spawn.yaml' in source


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
        'rivermark_demo_goals.yaml',
        'final_rivermark_static_obstacles.yaml',
        'final_rivermark_dynamic.yaml',
        'rivermark_appearance_profiles.yaml',
    ):
        (demo / name).touch()
    environment_usd = tmp_path / 'rivermark.usd'
    environment_usd.touch()
    for executable in ('run_isaac.sh', 'run_ros.sh'):
        target = scripts / executable
        target.write_text(
            '#!/usr/bin/env bash\n'
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
        env=_environment(RIVERMARK_USD=str(environment_usd)),
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    lines = result.stdout.splitlines()
    metadata = dict(line.split('=', 1) for line in lines[:3])
    return lines[3:], metadata


def test_v6_rivermark_ros_argv_is_estimated_occupancy_only_primary(tmp_path):
    arguments, metadata = _v6_rivermark_argv(
        tmp_path, 'ros', 'static')

    assert arguments[0] == 'navigation'
    assert 'odometry_mode:=estimated' in arguments
    assert 'structure_tf_source:=isaac' in arguments
    assert 'localization_map_contract:=occupancy_only' in arguments
    assert 'localization_owner:=amcl' in arguments
    assert 'localization_profile:=rivermark' in arguments
    assert 'ekf_profile:=wheel_imu' in arguments
    assert 'lidar_odometry_backend:=off' in arguments
    assert 'lidar_odometry_validated:=false' in arguments
    assert any(argument.startswith('imu_calibration_params_file:=')
               and argument.endswith('/robot_odometry/config/imu_calibration.yaml')
               for argument in arguments)
    assert 'nav2_profile:=v6_low_obstacle_isolation' in arguments
    assert 'cognitive_profile:=M3' in arguments
    assert 'cognitive_graph_mode:=primary' in arguments
    assert 'use_rviz:=false' in arguments
    assert not any(argument.startswith('posegraph_file:=')
                   for argument in arguments)
    assert any(argument.endswith('/rivermark_selected.yaml')
               for argument in arguments)
    assert any(argument.endswith('/rivermark_selected.geojson')
               for argument in arguments)
    assert metadata['SCENARIO'] == 'static'
    assert metadata['GOALS'].endswith('/rivermark_demo_goals.yaml')


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
        assert 'realistic' in arguments
        assert '--structure-tf-source' in arguments
        assert 'isaac' in arguments
        assert '--camera-profile' in arguments
        assert 'rgbd_navigation' in arguments
        assert metadata['GROUND_TRUTH'] == 'true'

    assert 'final_rivermark_static_obstacles.yaml' in ' '.join(static)
    assert '--dynamic-obstacles' in static
    assert 'final_rivermark_dynamic.yaml' in ' '.join(dynamic)
    assert 'full_route_four_stage' in dynamic
    assert 'v3' in dynamic
    assert '--no-dynamic-obstacles' in appearance
    assert 'dim_cool' in appearance


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
