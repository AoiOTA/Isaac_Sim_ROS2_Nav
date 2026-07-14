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
CLEAN_RUNTIME = REPOSITORY_ROOT / 'scripts' / 'clean_runtime.sh'
PERFORMANCE_MODE = REPOSITORY_ROOT / 'scripts' / 'performance_mode.sh'
RUN_RVIZ = REPOSITORY_ROOT / 'scripts' / 'run_rviz.sh'
RUN_TELEOP = REPOSITORY_ROOT / 'scripts' / 'run_teleop.sh'
RUN_TELEOP_TERMINAL = (
    REPOSITORY_ROOT / 'scripts' / 'run_teleop_terminal.sh')
RUN_ROS = REPOSITORY_ROOT / 'scripts' / 'run_ros.sh'
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


def _process_is_running(pid: int) -> bool:
    stat = Path(f'/proc/{pid}/stat')
    if not stat.is_file():
        return False
    return stat.read_text(encoding='utf-8').split()[2] != 'Z'


def _wait_until(predicate, *, timeout: float = 4.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return predicate()


def _runtime_metadata(path: Path) -> dict[str, str]:
    return dict(
        line.split('=', 1)
        for line in path.read_text(encoding='utf-8').splitlines()
        if '=' in line
    )


def _lock_is_available(path: Path) -> bool:
    return subprocess.run(
        ['flock', '-n', str(path), 'true'],
        text=True,
        capture_output=True,
        check=False,
    ).returncode == 0


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
    assert 'for component in motion_baseline teleop rviz ros isaac' in cleanup
    assert '/lib/robot_experiments/motion_baseline_runner' in cleanup
    assert '/bin/ros2 run robot_experiments motion_baseline_runner' in cleanup
    assert '${PROJECT_ROOT}/scripts/run_rviz.sh' in cleanup


def test_motion_baseline_identity_accepts_ros2_run_leader_only():
    """The authenticated leader is ros2 while the installed node is its child."""
    command = (
        '/usr/bin/python3 /opt/ros/jazzy/bin/ros2 run robot_experiments '
        'motion_baseline_runner --ros-args'
    )
    wrong_package = command.replace('robot_experiments', 'other_package')
    result = subprocess.run(
        [
            'bash',
            '-c',
            'source scripts/lib/common.sh\n'
            f'runtime_component_command_matches motion_baseline {command!r}\n'
            'if runtime_component_command_matches motion_baseline '
            f'{wrong_package!r}; then exit 91; fi\n',
        ],
        cwd=REPOSITORY_ROOT,
        text=True,
        capture_output=True,
        check=False,
        timeout=10,
    )
    assert result.returncode == 0, result.stderr


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


def test_ros_launcher_supervises_ordered_shutdown_before_sigint():
    source = RUN_ROS.read_text(encoding='utf-8')
    helper = 'python3 -m robot_bringup.ordered_shutdown'
    launch = 'setsid -- ros2 launch robot_bringup'
    ordered_stop = source[
        source.index('ordered_stop() {'):source.index('cleanup_supervisor() {')]

    assert helper in source
    assert launch in source
    assert 'shutdown_owned_process_groups' in ordered_stop
    assert ordered_stop.index(helper) < ordered_stop.index(
        'shutdown_owned_process_groups')
    assert "trap 'ordered_stop INT' INT" in source
    assert "trap 'ordered_stop TERM' TERM" in source
    assert "trap 'force_stop TERM' TERM" in source
    assert 'wait_for_owned_groups_exit "${shutdown_int_checks}"' in source
    assert 'signal_all_owned_groups KILL' in source
    assert 'shutdown_timeout_seconds="${ISAAC_NAV_SHUTDOWN_TIMEOUT_SECONDS:-20}"' \
        in source
    assert 'runtime_registered_process_group' in source
    assert 'managed_process_group_starts' in source
    assert 'runtime_process_start_ticks "${process_group}"' in source
    assert "trap '' INT TERM HUP\n  log_info" not in source


def _prepare_fake_rviz_project(tmp_path: Path, rviz_source: str):
    project = tmp_path / 'project'
    scripts = project / 'scripts'
    scripts_lib = scripts / 'lib'
    scripts_lib.mkdir(parents=True)
    shutil.copy2(RUN_RVIZ, scripts / 'run_rviz.sh')
    shutil.copy2(CLEAN_RUNTIME, scripts / 'clean_runtime.sh')
    shutil.copy2(COMMON, scripts_lib / 'common.sh')

    workspace_setup = project / 'ros2_ws/install/setup.bash'
    workspace_setup.parent.mkdir(parents=True)
    workspace_setup.write_text(
        'export ROS_DISTRO=jazzy\n', encoding='utf-8')
    ros_setup = tmp_path / 'ros_setup.bash'
    ros_setup.write_text('export ROS_DISTRO=jazzy\n', encoding='utf-8')
    rviz_config = tmp_path / 'navigation.rviz'
    rviz_config.write_text('Panels: []\n', encoding='utf-8')

    fake_bin = tmp_path / 'bin'
    fake_bin.mkdir()
    fake_ros2 = fake_bin / 'ros2'
    fake_ros2.write_text(
        '#!/usr/bin/env bash\nexit 0\n', encoding='utf-8')
    fake_ros2.chmod(0o755)
    fake_rviz = fake_bin / 'rviz2'
    fake_rviz.write_text(rviz_source, encoding='utf-8')
    fake_rviz.chmod(0o755)

    runtime = tmp_path / 'runtime'
    shm_root = tmp_path / 'shm'
    dds_proc_root = tmp_path / 'proc'
    shm_root.mkdir()
    dds_proc_root.mkdir()
    environment = _environment(
        PATH=f'{fake_bin}:{os.environ["PATH"]}',
        ROS_SETUP=str(ros_setup),
        ISAAC_NAV_RUNTIME_DIR=str(runtime),
        ISAAC_NAV_SHM_ROOT=str(shm_root),
        ISAAC_NAV_DDS_PROC_ROOT=str(dds_proc_root),
        ISAAC_NAV_RVIZ_INT_CHECKS='2',
        ISAAC_NAV_RVIZ_TERM_CHECKS='2',
        ISAAC_NAV_RVIZ_KILL_CHECKS='20',
        ISAAC_NAV_CLEAN_INT_CHECKS='2',
        ISAAC_NAV_CLEAN_TERM_CHECKS='2',
        ISAAC_NAV_CLEAN_KILL_CHECKS='20',
        FAKE_RVIZ_CHILD_PID_FILE=str(tmp_path / 'rviz-child.pid'),
        FAKE_RVIZ_FD_FILE=str(tmp_path / 'rviz-lock-fd.txt'),
        FAKE_RVIZ_READY_FILE=str(tmp_path / 'rviz-ready'),
    )
    return project, runtime, rviz_config, environment


def test_rviz_wrapper_kills_stubborn_descendant_before_metadata_cleanup(
        tmp_path):
    project, runtime, rviz_config, environment = _prepare_fake_rviz_project(
        tmp_path,
        '''#!/usr/bin/env bash
set -Eeuo pipefail
/usr/bin/python3 -c '
import os
from pathlib import Path
import signal
import time

signal.signal(signal.SIGINT, signal.SIG_IGN)
signal.signal(signal.SIGTERM, signal.SIG_IGN)
lock_path = os.path.join(os.environ["ISAAC_NAV_RUNTIME_DIR"], "rviz.lock")
inherited = False
for descriptor in os.listdir("/proc/self/fd"):
    try:
        inherited = inherited or os.readlink(
            f"/proc/self/fd/{descriptor}") == lock_path
    except OSError:
        pass
Path(os.environ["FAKE_RVIZ_FD_FILE"]).write_text(
    "held" if inherited else "closed", encoding="utf-8")
Path(os.environ["FAKE_RVIZ_CHILD_PID_FILE"]).write_text(
    str(os.getpid()), encoding="utf-8")
Path(os.environ["FAKE_RVIZ_READY_FILE"]).touch()
time.sleep(30)
' &
for _ in {1..200}; do
  [[ -e "${FAKE_RVIZ_READY_FILE}" ]] && exit 0
  sleep 0.01
done
exit 2
''')
    process = subprocess.Popen(
        [str(project / 'scripts/run_rviz.sh'), 'navigation',
         str(rviz_config)],
        cwd=project,
        env=environment,
        start_new_session=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    child_pid = None
    try:
        stdout, stderr = process.communicate(timeout=6.0)
        child_pid = int(
            (tmp_path / 'rviz-child.pid').read_text(encoding='utf-8'))

        assert process.returncode == 0, stdout + stderr
        assert not _process_is_running(child_pid)
        assert not (runtime / 'rviz.pid').exists()
        assert (tmp_path / 'rviz-lock-fd.txt').read_text(
            encoding='utf-8') == 'closed'
        assert _lock_is_available(runtime / 'rviz.lock')
        assert 'live process-group descendants; sending SIGINT' in stderr
        assert 'ignored SIGINT; sending SIGTERM' in stderr
        assert 'ignored SIGTERM; sending SIGKILL' in stderr
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3.0)
        if child_pid is not None and _process_is_running(child_pid):
            os.kill(child_pid, signal.SIGKILL)


def test_clean_runtime_authenticates_and_kills_standalone_rviz_group(
        tmp_path):
    project, runtime, rviz_config, environment = _prepare_fake_rviz_project(
        tmp_path,
        '''#!/usr/bin/env bash
set -Eeuo pipefail
trap '' INT TERM HUP
printf '%s\n' "$BASHPID" >"${FAKE_RVIZ_CHILD_PID_FILE}"
touch "${FAKE_RVIZ_READY_FILE}"
while true; do sleep 1; done
''')
    environment.update(
        ISAAC_NAV_RVIZ_INT_CHECKS='100',
        ISAAC_NAV_RVIZ_TERM_CHECKS='100',
    )
    process = subprocess.Popen(
        [str(project / 'scripts/run_rviz.sh'), 'navigation',
         str(rviz_config)],
        cwd=project,
        env=environment,
        start_new_session=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    child_pid = None
    try:
        assert _wait_until(
            lambda: (
                (runtime / 'rviz.pid').is_file()
                and (tmp_path / 'rviz-ready').is_file()
            )
        )
        metadata = _runtime_metadata(runtime / 'rviz.pid')
        child_pid = int(
            (tmp_path / 'rviz-child.pid').read_text(encoding='utf-8'))
        environment_entries = Path(
            f'/proc/{process.pid}/environ').read_bytes().split(b'\0')
        assert f'PROJECT_ROOT={project}'.encode() in environment_entries
        assert (
            f'ISAAC_NAV_SESSION_ID={metadata["session_id"]}'.encode()
            in environment_entries
        )
        assert int(metadata['pid']) == process.pid
        assert int(metadata['process_group']) == process.pid

        cleanup = subprocess.run(
            [str(project / 'scripts/clean_runtime.sh')],
            cwd=project,
            env=environment,
            text=True,
            capture_output=True,
            timeout=6.0,
            check=False,
        )
        process.wait(timeout=3.0)

        assert cleanup.returncode == 0, cleanup.stdout + cleanup.stderr
        assert not _process_is_running(process.pid)
        assert not _process_is_running(child_pid)
        assert not (runtime / 'rviz.pid').exists()
        assert _lock_is_available(runtime / 'rviz.lock')
        assert 'ignored SIGTERM; sending SIGKILL' in cleanup.stderr
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3.0)
        if child_pid is not None and _process_is_running(child_pid):
            try:
                os.killpg(os.getpgid(child_pid), signal.SIGKILL)
            except ProcessLookupError:
                pass


def test_teleop_terminal_kills_stubborn_registered_group_before_cleanup(
        tmp_path):
    project = tmp_path / 'project'
    scripts = project / 'scripts'
    scripts_lib = scripts / 'lib'
    scripts_lib.mkdir(parents=True)
    shutil.copy2(RUN_TELEOP_TERMINAL, scripts / 'run_teleop_terminal.sh')
    shutil.copy2(COMMON, scripts_lib / 'common.sh')
    fake_run_teleop = scripts / 'run_teleop.sh'
    fake_run_teleop.write_text(
        '''#!/usr/bin/env bash
set -Eeuo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/common.sh"
original_args=("$@")
ensure_dedicated_process_group "${original_args[@]}"
acquire_instance_lock teleop "fake Mapping teleop"
exec "${FAKE_TELEOP_EXECUTABLE}"
''',
        encoding='utf-8',
    )
    fake_run_teleop.chmod(0o755)
    fake_bin = tmp_path / 'bin'
    fake_bin.mkdir()
    fake_teleop = fake_bin / 'robot_teleop_keyboard_teleop'
    fake_teleop.write_text(
        '''#!/usr/bin/env bash
set -Eeuo pipefail
trap '' INT TERM HUP
printf '%s\n' "$BASHPID" >"${FAKE_TELEOP_PID_FILE}"
touch "${FAKE_TELEOP_READY_FILE}"
while true; do sleep 1; done
''',
        encoding='utf-8',
    )
    fake_teleop.chmod(0o755)
    fake_terminal = fake_bin / 'fake-terminal'
    fake_terminal.write_text(
        '''#!/usr/bin/env bash
set -Eeuo pipefail
"${FAKE_RUN_TELEOP}" &
wait "$!"
''',
        encoding='utf-8',
    )
    fake_terminal.chmod(0o755)

    runtime = tmp_path / 'runtime'
    environment = _environment(
        PATH=f'{fake_bin}:{os.environ["PATH"]}',
        ISAAC_NAV_RUNTIME_DIR=str(runtime),
        ISAAC_NAV_SESSION_ID='teleop-terminal-test-session',
        ISAAC_NAV_TELEOP_INT_CHECKS='3',
        ISAAC_NAV_TELEOP_TERM_CHECKS='3',
        ISAAC_NAV_TELEOP_KILL_CHECKS='20',
        ISAAC_NAV_TERMINAL_TERM_CHECKS='3',
        ISAAC_NAV_TERMINAL_KILL_CHECKS='20',
        FAKE_RUN_TELEOP=str(fake_run_teleop),
        FAKE_TELEOP_EXECUTABLE=str(fake_teleop),
        FAKE_TELEOP_PID_FILE=str(tmp_path / 'teleop-process.pid'),
        FAKE_TELEOP_READY_FILE=str(tmp_path / 'teleop-ready'),
    )
    process = subprocess.Popen(
        [str(scripts / 'run_teleop_terminal.sh'), str(fake_terminal)],
        cwd=project,
        env=environment,
        start_new_session=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    teleop_pid = None
    try:
        assert _wait_until(
            lambda: (
                (runtime / 'teleop.pid').is_file()
                and (tmp_path / 'teleop-ready').is_file()
            )
        )
        teleop_pid = int(
            (tmp_path / 'teleop-process.pid').read_text(encoding='utf-8'))
        teleop_group = int(
            _runtime_metadata(runtime / 'teleop.pid')['process_group'])
        assert os.getpgid(teleop_pid) == teleop_group

        os.kill(process.pid, signal.SIGTERM)
        time.sleep(0.12)
        assert _process_is_running(teleop_pid)
        assert (runtime / 'teleop.pid').is_file()
        stdout, stderr = process.communicate(timeout=6.0)

        assert process.returncode == 143, stdout + stderr
        assert not _process_is_running(teleop_pid)
        assert not (runtime / 'teleop.pid').exists()
        assert _lock_is_available(runtime / 'teleop.lock')
        assert 'did not exit after SIGINT; sending SIGTERM' in stderr
        assert 'did not exit after SIGTERM; sending SIGKILL' in stderr
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3.0)
        if teleop_pid is not None and _process_is_running(teleop_pid):
            try:
                os.killpg(os.getpgid(teleop_pid), signal.SIGKILL)
            except ProcessLookupError:
                pass


def _prepare_fake_managed_ros_project(
    tmp_path: Path, *, start_rviz: bool, start_teleop: bool = False
):
    project = tmp_path / 'project'
    scripts = project / 'scripts'
    scripts_lib = scripts / 'lib'
    scripts_lib.mkdir(parents=True)
    shutil.copy2(RUN_ROS, scripts / 'run_ros.sh')
    shutil.copy2(RUN_RVIZ, scripts / 'run_rviz.sh')
    shutil.copy2(COMMON, scripts_lib / 'common.sh')
    fake_run_teleop = scripts / 'run_teleop.sh'
    fake_run_teleop.write_text(
        """#!/usr/bin/env bash
set -Eeuo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/common.sh"
original_args=("$@")
ensure_dedicated_process_group "${original_args[@]}"
acquire_instance_lock teleop "fake Mapping teleop"
exec "${FAKE_TELEOP_EXECUTABLE}"
""",
        encoding='utf-8',
    )
    fake_run_teleop.chmod(0o755)
    workspace_setup = project / 'ros2_ws/install/setup.bash'
    workspace_setup.parent.mkdir(parents=True)
    workspace_setup.write_text(
        'export ROS_DISTRO=jazzy\n', encoding='utf-8')
    ros_setup = tmp_path / 'ros_setup.bash'
    ros_setup.write_text('export ROS_DISTRO=jazzy\n', encoding='utf-8')
    rviz_config = tmp_path / 'navigation.rviz'
    rviz_config.write_text('Panels: []\n', encoding='utf-8')

    fake_bin = tmp_path / 'bin'
    fake_bin.mkdir()
    fake_ros2 = fake_bin / 'ros2'
    fake_ros2.write_text(
        """#!/usr/bin/env bash
set -Eeuo pipefail
if [[ "$1" == launch ]]; then
  printf '%s\n' "$$" >"${FAKE_LAUNCH_PID_FILE}"
  trap 'exit 130' INT
  trap 'exit 143' TERM HUP
  if [[ "${FAKE_START_RVIZ}" == 1 ]]; then
    ISAAC_NAV_DEDICATED_PROCESS_GROUP=0 \
      "${FAKE_RUN_RVIZ}" navigation "${FAKE_RVIZ_CONFIG}" &
  fi
  if [[ "${FAKE_START_TELEOP}" == 1 ]]; then
    ISAAC_NAV_DEDICATED_PROCESS_GROUP=0 "${FAKE_RUN_TELEOP}" &
  fi
  while true; do sleep 1; done
fi
exit 99
""",
        encoding='utf-8',
    )
    fake_ros2.chmod(0o755)
    fake_rviz = fake_bin / 'rviz2'
    fake_rviz.write_text(
        """#!/usr/bin/env bash
set -Eeuo pipefail
printf '%s\n' "$$" >"${FAKE_RVIZ_PID_FILE}"
trap '' INT TERM HUP
while true; do sleep 1; done
""",
        encoding='utf-8',
    )
    fake_rviz.chmod(0o755)
    fake_teleop = fake_bin / 'robot_teleop_keyboard_teleop'
    fake_teleop.write_text(
        """#!/usr/bin/env bash
set -Eeuo pipefail
printf '%s\n' "$$" >"${FAKE_TELEOP_PID_FILE}"
trap '' INT TERM HUP
while true; do sleep 1; done
""",
        encoding='utf-8',
    )
    fake_teleop.chmod(0o755)
    fake_python = fake_bin / 'python3'
    fake_python.write_text(
        """#!/usr/bin/env bash
set -Eeuo pipefail
if [[ "${1:-}" == -m \
      && "${2:-}" == robot_bringup.ordered_shutdown ]]; then
  printf 'ordered-helper\n' >>"${FAKE_EVENT_LOG}"
  exit 0
fi
exec /usr/bin/python3 "$@"
""",
        encoding='utf-8',
    )
    fake_python.chmod(0o755)

    runtime = tmp_path / 'runtime'
    environment = _environment(
        PATH=f'{fake_bin}:{os.environ["PATH"]}',
        ROS_SETUP=str(ros_setup),
        ISAAC_NAV_RUNTIME_DIR=str(runtime),
        ISAAC_NAV_SHUTDOWN_TIMEOUT_SECONDS='4',
        ISAAC_NAV_LIFECYCLE_SHUTDOWN_SECONDS='1',
        ISAAC_NAV_SHUTDOWN_INT_CHECKS='3',
        ISAAC_NAV_SHUTDOWN_TERM_CHECKS='3',
        ISAAC_NAV_SHUTDOWN_KILL_CHECKS='20',
        FAKE_START_RVIZ='1' if start_rviz else '0',
        FAKE_START_TELEOP='1' if start_teleop else '0',
        FAKE_RUN_RVIZ=str(scripts / 'run_rviz.sh'),
        FAKE_RUN_TELEOP=str(fake_run_teleop),
        FAKE_RVIZ_CONFIG=str(rviz_config),
        FAKE_TELEOP_EXECUTABLE=str(fake_teleop),
        FAKE_LAUNCH_PID_FILE=str(tmp_path / 'launch.pid'),
        FAKE_RVIZ_PID_FILE=str(tmp_path / 'rviz-process.pid'),
        FAKE_TELEOP_PID_FILE=str(tmp_path / 'teleop-process.pid'),
        FAKE_EVENT_LOG=str(tmp_path / 'events.log'),
    )
    return project, runtime, environment


def test_ros_supervisor_stops_nested_rviz_group_and_cleans_metadata(tmp_path):
    project, runtime, environment = _prepare_fake_managed_ros_project(
        tmp_path, start_rviz=True, start_teleop=True)
    process = subprocess.Popen(
        [str(project / 'scripts/run_ros.sh'), 'mapping'],
        cwd=project,
        env=environment,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    rviz_process_pid = None
    teleop_process_pid = None
    launch_pid = None
    try:
        assert _wait_until(
            lambda: (
                (runtime / 'ros.pid').is_file()
                and (runtime / 'rviz.pid').is_file()
                and (runtime / 'teleop.pid').is_file()
                and (tmp_path / 'launch.pid').is_file()
                and (tmp_path / 'rviz-process.pid').is_file()
                and (tmp_path / 'teleop-process.pid').is_file()
                and os.getpgid(process.pid) == process.pid
            )
        )
        launch_pid = int((tmp_path / 'launch.pid').read_text())
        rviz_process_pid = int(
            (tmp_path / 'rviz-process.pid').read_text())
        teleop_process_pid = int(
            (tmp_path / 'teleop-process.pid').read_text())
        rviz_metadata = _runtime_metadata(runtime / 'rviz.pid')
        teleop_metadata = _runtime_metadata(runtime / 'teleop.pid')
        rviz_group = int(rviz_metadata['process_group'])
        assert rviz_group == int(rviz_metadata['pid'])
        assert rviz_group != process.pid
        assert rviz_group != launch_pid
        assert os.getpgid(rviz_process_pid) == rviz_group
        assert os.getpgid(teleop_process_pid) == int(
            teleop_metadata['process_group'])
        assert rviz_metadata['session_id'] == _runtime_metadata(
            runtime / 'ros.pid')['session_id']

        shutdown_started = time.monotonic()
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=6.0)
        stdout, stderr = process.communicate(timeout=1.0)

        assert time.monotonic() - shutdown_started < 4.5
        assert not _process_is_running(launch_pid)
        assert not _process_is_running(rviz_process_pid)
        assert not _process_is_running(teleop_process_pid)
        assert not (runtime / 'rviz.pid').exists()
        assert not (runtime / 'teleop.pid').exists()
        assert not (runtime / 'ros.pid').exists()
        assert (tmp_path / 'events.log').read_text().splitlines() == [
            'ordered-helper']
        assert 'stopping managed rviz process group' in stdout
        assert 'stopping managed teleop process group' in stdout
        assert 'with SIGINT' in stdout
        assert 'with SIGTERM' in stdout
        assert 'with SIGKILL' in stdout
        assert 'escalating to SIGKILL' in stderr
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3.0)
        for pid in (rviz_process_pid, teleop_process_pid, launch_pid):
            if pid is not None and _process_is_running(pid):
                try:
                    os.killpg(os.getpgid(pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass


def test_ros_supervisor_refuses_foreign_project_rviz_metadata(tmp_path):
    project, runtime, environment = _prepare_fake_managed_ros_project(
        tmp_path, start_rviz=False)
    foreign_root = tmp_path / 'foreign-project'
    foreign_root.mkdir()
    foreign_environment = os.environ.copy()
    foreign_environment.update(
        PROJECT_ROOT=str(foreign_root),
        ISAAC_NAV_SESSION_ID='foreign-session',
    )
    foreign = subprocess.Popen(
        [sys.executable, '-c', 'import time; time.sleep(30)', 'rviz2'],
        start_new_session=True,
        env=foreign_environment,
    )
    runtime.mkdir(mode=0o700)
    foreign_start = Path(f'/proc/{foreign.pid}/stat').read_text().split()[21]
    boot_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    foreign_metadata = runtime / 'rviz.pid'
    foreign_metadata.write_text(
        '\n'.join([
            f'pid={foreign.pid}',
            f'process_group={os.getpgid(foreign.pid)}',
            f'leader_start_ticks={foreign_start}',
            f'boot_id={boot_id}',
            'component=rviz',
            f'project_root={foreign_root}',
            'session_id=foreign-session',
            'started_at=test',
            '',
        ]),
        encoding='utf-8',
    )
    process = subprocess.Popen(
        [str(project / 'scripts/run_ros.sh'), 'mapping'],
        cwd=project,
        env=environment,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    launch_pid = None
    try:
        assert _wait_until(
            lambda: (
                (tmp_path / 'launch.pid').is_file()
                and (runtime / 'ros.pid').is_file()
                and os.getpgid(process.pid) == process.pid
            )
        )
        launch_pid = int((tmp_path / 'launch.pid').read_text())
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=6.0)

        assert _process_is_running(foreign.pid)
        assert foreign_metadata.is_file()
        assert _runtime_metadata(foreign_metadata)['project_root'] \
            == str(foreign_root)
        assert not (runtime / 'ros.pid').exists()
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3.0)
        if launch_pid is not None and _process_is_running(launch_pid):
            os.killpg(launch_pid, signal.SIGKILL)
        if foreign.poll() is None:
            os.killpg(foreign.pid, signal.SIGKILL)
            foreign.wait(timeout=3.0)


def test_ros_supervisor_second_signal_forces_stubborn_launch_group(tmp_path):
    project = tmp_path / 'project'
    scripts = project / 'scripts'
    scripts_lib = scripts / 'lib'
    scripts_lib.mkdir(parents=True)
    shutil.copy2(RUN_ROS, scripts / 'run_ros.sh')
    shutil.copy2(COMMON, scripts_lib / 'common.sh')
    workspace_setup = project / 'ros2_ws/install/setup.bash'
    workspace_setup.parent.mkdir(parents=True)
    workspace_setup.write_text(
        'export ROS_DISTRO=jazzy\n', encoding='utf-8')
    ros_setup = tmp_path / 'ros_setup.bash'
    ros_setup.write_text('export ROS_DISTRO=jazzy\n', encoding='utf-8')
    fake_bin = tmp_path / 'bin'
    fake_bin.mkdir()
    fake_ros2 = fake_bin / 'ros2'
    fake_ros2.write_text(
        '''#!/usr/bin/env bash
set -Eeuo pipefail
if [[ "$1" == launch ]]; then
  printf '%s\n' "$$" >"${FAKE_LAUNCH_PID_FILE}"
  trap '' INT TERM HUP
  while true; do sleep 1; done
fi
exit 99
''',
        encoding='utf-8',
    )
    fake_ros2.chmod(0o755)
    fake_python = fake_bin / 'python3'
    fake_python.write_text(
        '''#!/usr/bin/env bash
set -Eeuo pipefail
printf '%s\n' "$$" >"${FAKE_HELPER_PID_FILE}"
sleep 30
''',
        encoding='utf-8',
    )
    fake_python.chmod(0o755)
    launch_pid_file = tmp_path / 'launch.pid'
    helper_pid_file = tmp_path / 'helper.pid'
    environment = _environment(
        PATH=f'{fake_bin}:{os.environ["PATH"]}',
        ROS_SETUP=str(ros_setup),
        ISAAC_NAV_RUNTIME_DIR=str(tmp_path / 'runtime'),
        ISAAC_NAV_SHUTDOWN_INT_CHECKS='3',
        ISAAC_NAV_SHUTDOWN_TERM_CHECKS='3',
        FAKE_LAUNCH_PID_FILE=str(launch_pid_file),
        FAKE_HELPER_PID_FILE=str(helper_pid_file),
    )
    process = subprocess.Popen(
        [str(scripts / 'run_ros.sh'), 'mapping'],
        cwd=project,
        env=environment,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    launch_pid = None
    try:
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            if launch_pid_file.exists() and os.getpgid(process.pid) == process.pid:
                break
            time.sleep(0.02)
        assert launch_pid_file.exists()
        launch_pid = int(launch_pid_file.read_text(encoding='utf-8'))

        os.killpg(process.pid, signal.SIGINT)
        deadline = time.monotonic() + 3.0
        while not helper_pid_file.exists() and time.monotonic() < deadline:
            time.sleep(0.02)
        assert helper_pid_file.exists()

        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=4.0)
        with pytest.raises(ProcessLookupError):
            os.kill(launch_pid, 0)
    finally:
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=3.0)
        if launch_pid is not None and _process_is_running(launch_pid):
            os.killpg(launch_pid, signal.SIGKILL)


def test_saved_map_launcher_derives_and_requires_version_manifest():
    source = RUN_ROS.read_text(encoding='utf-8')
    assert 'if [[ "${operation}" != "mapping" ]]' in source
    assert 'map_manifest_file:=*' in source
    assert '/data/maps/manifests/$(basename "${posegraph_prefix}").yaml' \
        in source
    assert 'launch_args+=("map_manifest_file:=${map_manifest_file}")' \
        in source


def test_save_map_publishes_verified_manifest_after_all_four_artifacts():
    source = SAVE_MAP.read_text(encoding='utf-8')
    assert 'data/maps/.staging/${version}.XXXXXX' in source
    assert 'ros2 run robot_bringup map_manifest create' in source
    assert 'ros2 run robot_bringup map_manifest verify' in source
    first_artifact_publish = source.index(
        'publish_no_clobber "${staged_occupancy}.yaml" "${occupancy}.yaml"')
    last_artifact_publish = source.index(
        'publish_no_clobber "${staged_posegraph}.data" "${posegraph}.data"')
    bundle_verify = source.index(
        'ros2 run robot_bringup map_manifest verify')
    manifest_publish = source.index(
        'publish_no_clobber "${staged_manifest}" "${manifest}"')
    assert first_artifact_publish < last_artifact_publish
    assert last_artifact_publish < bundle_verify < manifest_publish
    assert '"calibrated": False' not in source  # owned by the strict generator


def _prepare_fake_map_save_project(tmp_path):
    project = tmp_path / 'project'
    scripts = project / 'scripts'
    scripts_lib = scripts / 'lib'
    scripts_lib.mkdir(parents=True)
    shutil.copy2(SAVE_MAP, scripts / 'save_map.sh')
    shutil.copy2(COMMON, scripts_lib / 'common.sh')
    workspace_setup = project / 'ros2_ws/install/setup.bash'
    workspace_setup.parent.mkdir(parents=True)
    workspace_setup.write_text(
        'export ROS_DISTRO=jazzy\n', encoding='utf-8')
    ros_setup = tmp_path / 'ros_setup.bash'
    ros_setup.write_text('export ROS_DISTRO=jazzy\n', encoding='utf-8')
    fake_bin = tmp_path / 'bin'
    fake_bin.mkdir()
    fake_ros2 = fake_bin / 'ros2'
    fake_ros2.write_text(
        '''#!/usr/bin/env bash
set -Eeuo pipefail
if [[ "$1 $2 $3" == "run nav2_map_server map_saver_cli" ]]; then
  prefix="$5"
  printf 'image: %s.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n' \
    "$(basename "${prefix}")" >"${prefix}.yaml"
  printf 'P5\n1 1\n255\n\\0' >"${prefix}.pgm"
  printf 'occupancy_saved\n' >>"${FAKE_EVENT_LOG}"
  exit 0
fi
if [[ "$1 $2" == "service call" ]]; then
  service_request="${5}"
  prefix="${service_request#*filename: \\\'}"
  prefix="${prefix%%\\\'*}"
  printf posegraph >"${prefix}.posegraph"
  printf data >"${prefix}.data"
  printf 'posegraph_serialized\n' >>"${FAKE_EVENT_LOG}"
  printf 'response: result=%s\n' "${FAKE_SERIALIZE_RESULT:-0}"
  exit 0
fi
if [[ "$1 $2 $3" == "run robot_bringup map_manifest" ]]; then
  shift 3
  if [[ "$1" == verify ]]; then
    version="${FAKE_MAP_VERSION}"
    root="${FAKE_PROJECT_ROOT}"
    test -s "${root}/data/maps/occupancy/${version}.yaml"
    test -s "${root}/data/maps/occupancy/${version}.pgm"
    test -s "${root}/data/maps/posegraphs/${version}.posegraph"
    test -s "${root}/data/maps/posegraphs/${version}.data"
    test ! -e "${root}/data/maps/manifests/${version}.yaml"
    printf 'bundle_verified_before_manifest\n' >>"${FAKE_EVENT_LOG}"
	else
	  if [[ -n "${FAKE_CONCURRENT_TARGET:-}" ]]; then
	    mkdir -p "$(dirname "${FAKE_CONCURRENT_TARGET}")"
	    printf external >"${FAKE_CONCURRENT_TARGET}"
	  fi
	  printf 'manifest_created_in_staging\n' >>"${FAKE_EVENT_LOG}"
	fi
  exec python3 -m robot_bringup.map_manifest "$@"
fi
printf 'unexpected fake ros2 command: %s\n' "$*" >&2
exit 99
''',
        encoding='utf-8',
    )
    fake_ros2.chmod(0o755)
    fake_ln = fake_bin / 'ln'
    fake_ln.write_text(
        '''#!/usr/bin/env bash
set -Eeuo pipefail
/usr/bin/ln "$@"
target="${@: -1}"
if [[ -n "${FAKE_SIGNAL_AFTER_LINK:-}" \
      && "${target}" == "${FAKE_SIGNAL_AFTER_LINK}" ]]; then
  kill -TERM "${PPID}"
fi
''',
        encoding='utf-8',
    )
    fake_ln.chmod(0o755)
    environment = _environment(
        PATH=f'{fake_bin}:{os.environ["PATH"]}',
        PYTHONPATH=(
            f'{PACKAGE_ROOT}:{os.environ.get("PYTHONPATH", "")}'),
        ROS_SETUP=str(ros_setup),
        ISAAC_NAV_RUNTIME_DIR=str(tmp_path / 'runtime'),
        FAKE_EVENT_LOG=str(tmp_path / 'events.log'),
        FAKE_PROJECT_ROOT=str(project),
        FAKE_MAP_VERSION='contract_v2',
    )
    return project, environment


def test_save_map_transaction_manifest_last_and_failure_cleanup(tmp_path):
    project, environment = _prepare_fake_map_save_project(tmp_path)
    script = project / 'scripts/save_map.sh'
    result = subprocess.run(
        [str(script), 'contract_v2'],
        cwd=tmp_path,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    manifest_path = project / 'data/maps/manifests/contract_v2.yaml'
    document = yaml.safe_load(manifest_path.read_text(encoding='utf-8'))
    assert document['calibration']['calibrated'] is False
    assert document['calibration']['spawn_pose_profile'] is None
    assert document['calibration']['bundle_sha256'] is None
    assert (tmp_path / 'events.log').read_text(encoding='utf-8').splitlines() == [
        'occupancy_saved',
        'posegraph_serialized',
        'manifest_created_in_staging',
        'bundle_verified_before_manifest',
    ]
    assert not any((project / 'data/maps/.staging').iterdir())

    failed_tmp = tmp_path / 'failed-fixture'
    failed_tmp.mkdir()
    project, failed_environment = _prepare_fake_map_save_project(failed_tmp)
    failed_environment['FAKE_SERIALIZE_RESULT'] = '1'
    failed = subprocess.run(
        [str(project / 'scripts/save_map.sh'), 'contract_v2'],
        cwd=failed_tmp,
        env=failed_environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert failed.returncode != 0
    assert 'pose-graph serialization failure' in failed.stderr
    assert not list((project / 'data/maps/occupancy').glob('contract_v2.*'))
    assert not list((project / 'data/maps/posegraphs').glob('contract_v2.*'))
    assert not (project / 'data/maps/manifests/contract_v2.yaml').exists()
    assert not any((project / 'data/maps/.staging').iterdir())


def test_save_map_preserves_artifact_created_during_serialization(tmp_path):
    project, environment = _prepare_fake_map_save_project(tmp_path)
    concurrent = project / 'data/maps/occupancy/contract_v2.pgm'
    environment['FAKE_CONCURRENT_TARGET'] = str(concurrent)

    result = subprocess.run(
        [str(project / 'scripts/save_map.sh'), 'contract_v2'],
        cwd=tmp_path,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode != 0
    assert 'refusing to overwrite concurrently-created' in result.stderr
    assert concurrent.read_text(encoding='utf-8') == 'external'
    assert not (project / 'data/maps/occupancy/contract_v2.yaml').exists()
    assert not list((project / 'data/maps/posegraphs').glob('contract_v2.*'))
    assert not (project / 'data/maps/manifests/contract_v2.yaml').exists()
    assert not any((project / 'data/maps/.staging').iterdir())


def test_save_map_rolls_back_manifest_if_signal_arrives_after_link(tmp_path):
    project, environment = _prepare_fake_map_save_project(tmp_path)
    manifest = project / 'data/maps/manifests/contract_v2.yaml'
    environment['FAKE_SIGNAL_AFTER_LINK'] = str(manifest)

    result = subprocess.run(
        [str(project / 'scripts/save_map.sh'), 'contract_v2'],
        cwd=tmp_path,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 130
    assert not list((project / 'data/maps/occupancy').glob('contract_v2.*'))
    assert not list((project / 'data/maps/posegraphs').glob('contract_v2.*'))
    assert not manifest.exists()
    assert not any((project / 'data/maps/.staging').iterdir())


def test_save_map_rejects_symlinked_storage_directory(tmp_path):
    project, environment = _prepare_fake_map_save_project(tmp_path)
    maps = project / 'data/maps'
    maps.mkdir(parents=True)
    external = tmp_path / 'external_manifests'
    external.mkdir()
    (maps / 'manifests').symlink_to(external, target_is_directory=True)

    result = subprocess.run(
        [str(project / 'scripts/save_map.sh'), 'contract_v2'],
        cwd=tmp_path,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode != 0
    assert 'traverses a symbolic link' in result.stderr
    assert not any(external.iterdir())
