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


def test_ros_launcher_supervises_ordered_shutdown_before_sigint():
    source = RUN_ROS.read_text(encoding='utf-8')
    helper = 'python3 -m robot_bringup.ordered_shutdown'
    launch = 'setsid -- ros2 launch robot_bringup'
    relay = 'signal_launch_group INT'

    assert helper in source
    assert launch in source
    assert relay in source
    assert source.index(helper) < source.index(relay)
    assert "trap 'ordered_stop INT' INT" in source
    assert "trap 'ordered_stop TERM' TERM" in source
    assert "trap 'force_stop TERM' TERM" in source
    assert 'wait_for_launch_group_exit "${shutdown_int_checks}"' in source
    assert 'signal_launch_group KILL' in source
    assert "trap '' INT TERM HUP\n  log_info" not in source


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
