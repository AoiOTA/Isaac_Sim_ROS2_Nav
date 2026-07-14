from __future__ import annotations

import hashlib
import json
import os
import shutil
import subprocess
from pathlib import Path

import pytest


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
SCRIPT = REPOSITORY_ROOT / 'scripts' / 'run_contact_ab_matrix.sh'


def _run(*arguments: str) -> subprocess.CompletedProcess[str]:
    environment = os.environ.copy()
    environment.pop('ROS_DOMAIN_ID', None)
    environment.pop('RMW_IMPLEMENTATION', None)
    return subprocess.run(
        ['bash', str(SCRIPT), *arguments],
        cwd=REPOSITORY_ROOT,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
        timeout=10,
    )


def _shell_function_source(name: str) -> str:
    source = SCRIPT.read_text(encoding='utf-8')
    start = source.index(f'{name}() {{')
    offset = start
    in_python_heredoc = False
    for line in source[start:].splitlines(keepends=True):
        offset += len(line)
        if in_python_heredoc:
            if line.rstrip('\n') == 'PY':
                in_python_heredoc = False
            continue
        if "<<'PY'" in line:
            in_python_heredoc = True
            continue
        if line.rstrip('\n') == '}':
            return source[start:offset].rstrip('\n')
    raise AssertionError(f'unterminated shell function: {name}')


def _bash_harness(
    source: str,
    *,
    timeout: float = 10.0,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ['bash', '-c', source],
        cwd=REPOSITORY_ROOT,
        text=True,
        capture_output=True,
        check=False,
        timeout=timeout,
    )


def test_contact_ab_matrix_has_valid_shell_and_help_contract():
    assert SCRIPT.is_file()
    assert os.access(SCRIPT, os.X_OK)
    syntax = subprocess.run(
        ['bash', '-n', str(SCRIPT)],
        text=True,
        capture_output=True,
        check=False,
        timeout=10,
    )
    assert syntax.returncode == 0, syntax.stderr

    help_result = _run('--help')
    assert help_result.returncode == 0, help_result.stderr
    assert '--environment Warehouse|SimplePlane|all' in help_result.stdout
    assert '--repeats N' in help_result.stdout
    assert '--output-dir DIR' in help_result.stdout
    assert '36' in help_result.stdout


@pytest.mark.parametrize(
    ('arguments', 'message'),
    [
        (('--environment', 'warehouse', '--output-dir', '/tmp/out'),
         '--environment must be Warehouse, SimplePlane, or all'),
        (('--repeats', '0', '--output-dir', '/tmp/out'),
         '--repeats must be a positive integer'),
        (('--repeats', '101', '--output-dir', '/tmp/out'),
         '--repeats must be an integer in [1, 100]'),
        (('--repeats', '999999999999999999999999999999',
          '--output-dir', '/tmp/out'),
         '--repeats must be an integer in [1, 100]'),
        (('--repeats', '1'), '--output-dir is required'),
        (('--unknown',), 'unknown contact A/B argument'),
    ],
)
def test_contact_ab_matrix_rejects_ambiguous_arguments(arguments, message):
    result = _run(*arguments)
    assert result.returncode != 0
    assert message in result.stderr


def test_contact_ab_matrix_locks_the_ordered_inputs_and_runtime_modes():
    source = SCRIPT.read_text(encoding='utf-8')
    profiles = [
        'legacy_baseline',
        'threshold_corr_0p00025_offset_0p0004',
        'threshold_corr_0p025_offset_0p0004',
        'threshold_corr_0p00025_offset_0p04',
        'threshold_corr_0p025_offset_0p04',
        'explicit_material',
    ]
    positions = [source.index(f'  {profile}\n') for profile in profiles]
    assert positions == sorted(positions)
    assert 'ros2_ws/src/robot_experiments/config/motion_skid_steer_ab.yaml' \
        in source
    assert 'isaac_sim/configs/simple_plane.project.yaml' in source
    assert 'isaac_sim/configs/project.yaml' in source
    assert 'for environment_id in "${environments[@]}"' in source
    assert 'for profile_index in "${!profile_ids[@]}"' in source
    assert 'for ((repeat = 1; repeat <= repeats; repeat++))' in source
    assert '"${SCRIPT_DIR}/run_isaac.sh"' in source
    assert '--headless --pacing-mode unbounded' in source
    assert (
        '--navigation-mode mapping --mode ideal --camera-profile off' in source
    )
    assert '"${SCRIPT_DIR}/run_motion_baseline.sh"' in source
    assert '--odometry-mode ideal' in source


def test_contact_ab_matrix_is_fail_closed_on_git_readiness_and_reports():
    source = SCRIPT.read_text(encoding='utf-8')
    assert 'status --porcelain --untracked-files=normal' in source
    assert 'ls-files --error-unmatch' in source
    assert 'runtime_provenance.schema_version' in source
    assert 'runtime_provenance.environment.id' in source
    assert 'runtime_provenance.contact.json' in source
    assert 'runtime_provenance.contact.sha256' in source
    assert 'profile_sha256' in source
    assert 'stage_usd_readback_verified' in source
    assert 'motion_skid_steer_ab.yaml' in source
    assert 'manifest.tsv' in source
    assert 'report.get("result") != "success"' in source
    assert 'runtime_provenance.git.dirty' in source
    assert 'runtime_provenance.git.commit' in source
    assert 'runtime_provenance.git.branch' in source
    assert 'runtime_provenance.robot.config.path' in source
    assert 'runtime_provenance.robot.config.sha256' in source
    assert 'runtime_provenance.robot.asset.path' in source
    assert 'runtime_provenance.robot.asset.sha256' in source
    assert 'runtime_provenance.robot.solver.position_iterations' in source
    assert 'runtime_provenance.robot.solver.velocity_iterations' in source
    assert 'runtime_provenance.environment.project_stage.path' in source
    assert 'runtime_provenance.environment.project_stage.sha256' in source
    assert 'runtime_provenance.environment.source_asset.path' in source
    assert 'runtime_provenance.environment.source_asset.sha256' in source
    assert 'runtime_provenance.simulation.physics_hz' in source
    assert 'runtime_provenance.simulation.navigation_mode' in source
    assert 'runtime_provenance.simulation.odometry_mode' in source
    assert 'analyse_contact_ab(' in source
    assert 'min_repeats=1' in source
    assert 'workspace source deliberately' in source


def test_contact_ab_matrix_clears_untrusted_nested_overrides_dynamically():
    function = _shell_function_source('clear_inherited_config_overrides')
    result = _bash_harness(
        'set -Eeuo pipefail\n'
        f'{function}\n'
        'export ISAAC_NAV__SIMULATION__PHYSICS_HZ=999\n'
        'export ISAAC_NAV__FILES__CONTACT_PROFILE=/tmp/evil.yaml\n'
        'export ISAAC_NAV_SESSION_ID=kept-session\n'
        'export PROJECT_ROOT=/kept/root\n'
        'clear_inherited_config_overrides\n'
        '[[ -z "${ISAAC_NAV__SIMULATION__PHYSICS_HZ+x}" ]]\n'
        '[[ -z "${ISAAC_NAV__FILES__CONTACT_PROFILE+x}" ]]\n'
        '[[ "${ISAAC_NAV_SESSION_ID}" == kept-session ]]\n'
        '[[ "${PROJECT_ROOT}" == /kept/root ]]\n'
    )
    assert result.returncode == 0, result.stderr


def test_contact_ab_matrix_reinstates_only_explicit_isaac_overrides():
    source = SCRIPT.read_text(encoding='utf-8')
    launch = source[
        source.index('launch_isaac() {'):
        source.index('launch_motion_runner() {')
    ]
    assert launch.index('clear_inherited_config_overrides') \
        < launch.index('export ISAAC_NAV_PROJECT_CONFIG=')
    assert launch.index('clear_inherited_config_overrides') \
        < launch.index('export ISAAC_NAV__FILES__CONTACT_PROFILE=')
    assert 'ISAAC_NAV__SIMULATION__PHYSICS_HZ' not in launch
    boundary = (
        'does not\n    # currently expose headless, pacing, or camera '
        'selection'
    )
    assert boundary in launch


def test_contact_ab_matrix_locks_batch_identity_and_hashed_evidence():
    source = SCRIPT.read_text(encoding='utf-8')
    assert "rev-parse --verify 'HEAD^{commit}'" in source
    assert 'symbolic-ref --quiet --short HEAD' in source
    assert 'locked_input_paths=(' in source
    assert 'batch_profile_hashes_json' in source
    run_one = source[
        source.index('run_one_condition() {'):
        source.index('reject_symlink_path_components() {')
    ]
    assert 'verify_batch_identity "before ${current_run_id}"' in run_one
    assert 'verify_batch_identity "after ${current_run_id}"' in run_one
    assert run_one.index('stop_owned_component isaac') \
        < run_one.index('append_current_manifest success')
    assert 'report_sha256' in source
    assert 'isaac_log_sha256' in source
    assert 'runner_log_sha256' in source
    assert 'warehouse_project_config_sha256' in source
    assert 'simple_plane_project_config_sha256' in source
    assert 'append_manifest_line_atomically' in source


def test_manifest_append_is_complete_and_atomic_dynamically(tmp_path):
    function = _shell_function_source('append_manifest_line_atomically')
    manifest = tmp_path / 'manifest.tsv'
    manifest.write_text('header\n', encoding='utf-8')
    result = _bash_harness(
        'set -Eeuo pipefail\n'
        f'{function}\n'
        f'manifest={str(manifest)!r}\n'
        "append_manifest_line_atomically $'one\\ttwo'\n"
    )
    assert result.returncode == 0, result.stderr
    assert manifest.read_text(encoding='utf-8') == 'header\none\ttwo\n'
    assert not list(tmp_path.glob('manifest.tsv.tmp.*'))


def test_success_manifest_row_has_all_locked_inputs_and_final_hashes(tmp_path):
    functions = '\n'.join(
        _shell_function_source(name)
        for name in (
            'tsv_safe',
            'final_evidence_sha256',
            'append_manifest_line_atomically',
            'append_current_manifest',
        )
    )
    manifest = tmp_path / 'manifest.tsv'
    report = tmp_path / 'report.json'
    isaac_log = tmp_path / 'isaac.log'
    runner_log = tmp_path / 'runner.log'
    manifest.write_text(
        '\t'.join(f'column_{index}' for index in range(31)) + '\n'
    )
    report.write_text('{"result":"success"}\n', encoding='utf-8')
    isaac_log.write_text('isaac stopped\n', encoding='utf-8')
    runner_log.write_text('runner stopped\n', encoding='utf-8')
    assignments = {
        'manifest': manifest,
        'current_run_id': '001_simple_plane_legacy_baseline_r01',
        'current_environment': 'SimplePlane',
        'current_profile_id': 'legacy_baseline',
        'current_profile_mode': 'legacy_baseline',
        'current_repeat': '1',
        'current_report': report,
        'current_isaac_log': isaac_log,
        'current_runner_log': runner_log,
        'batch_git_commit': 'a' * 40,
        'batch_git_branch': 'codex/test',
        'motion_config': '/repo/motion.yaml',
        'batch_motion_sha256': 'b' * 64,
        'warehouse_config': '/repo/warehouse.yaml',
        'batch_warehouse_project_sha256': 'c' * 64,
        'simple_plane_config': '/repo/simple.yaml',
        'batch_simple_plane_project_sha256': 'd' * 64,
        'current_project_config': '/repo/simple.yaml',
        'current_project_sha256': 'd' * 64,
        'current_profile_path': '/repo/legacy.yaml',
        'current_profile_sha256': 'e' * 64,
        'batch_profile_hashes_json': '{"legacy_baseline":"digest"}',
        'current_project_stage': '/repo/simple-stage.usda',
        'current_project_stage_sha256': 'f' * 64,
        'current_source_asset': '/repo/simple.usda',
        'current_source_asset_sha256': '1' * 64,
        'current_started': '2026-07-14T00:00:00Z',
    }
    shell_assignments = '\n'.join(
        f'{name}={str(value)!r}' for name, value in assignments.items()
    )
    result = _bash_harness(
        'set -Eeuo pipefail\n'
        'declare -Ag owned_pids=()\n'
        'runtime_process_is_running() { return 1; }\n'
        'sha256_file() { sha256sum "$1" | awk "{print \\$1}"; }\n'
        f'{functions}\n'
        f'{shell_assignments}\n'
        'current_recorded=false\n'
        'append_current_manifest success complete\n'
        '[[ "${current_recorded}" == true ]]\n'
    )
    assert result.returncode == 0, result.stderr
    lines = manifest.read_text(encoding='utf-8').splitlines()
    assert len(lines) == 2
    fields = lines[1].split('\t')
    assert len(fields) == 31
    assert fields[8] == hashlib.sha256(report.read_bytes()).hexdigest()
    assert fields[10] == hashlib.sha256(isaac_log.read_bytes()).hexdigest()
    assert fields[12] == hashlib.sha256(runner_log.read_bytes()).hexdigest()
    assert fields[13] == 'a' * 40
    assert fields[16] == 'b' * 64
    assert fields[18] == 'c' * 64
    assert fields[20] == 'd' * 64


def test_output_path_symlink_rejection_runs_before_realpath_dynamically(
    tmp_path,
):
    function = _shell_function_source('reject_symlink_path_components')
    target = tmp_path / 'target'
    target.mkdir()
    link = tmp_path / 'linked'
    link.symlink_to(target, target_is_directory=True)
    result = _bash_harness(
        'set -Eeuo pipefail\n'
        f'{function}\n'
        f'reject_symlink_path_components {str(link / "evidence")!r}\n'
    )
    assert result.returncode != 0
    assert 'path contains a symlink' in result.stderr
    source = SCRIPT.read_text(encoding='utf-8')
    prepare = source[
        source.index('prepare_output_directory() {'):
        source.index('initialize_manifest() {')
    ]
    assert (
        prepare.index('reject_symlink_path_components "${raw_output_dir}"')
        < prepare.index('realpath -m "${raw_output_dir}"')
    )
    assert 'check-ignore -q' in prepare
    source = SCRIPT.read_text(encoding='utf-8')
    process_group_call = (
        'ensure_dedicated_process_group "${original_args[@]}"'
    )
    assert source.index(process_group_call) \
        < source.index('\nprepare_output_directory\n')


def test_contact_ab_matrix_only_signals_authenticated_child_groups():
    source = SCRIPT.read_text(encoding='utf-8')
    identity_and_stop = source[
        source.index('component_identity_is_current() {'):
        source.index('cleanup_batch() {')
    ]
    stop_start = identity_and_stop.index('stop_owned_component() {')
    stop = identity_and_stop[stop_start:]
    assert 'runtime_registered_process_group' in stop
    assert 'runtime_process_group_is_owned_by_session' in identity_and_stop
    assert (
        stop.index('runtime_registered_process_group')
        < stop.index('kill -INT')
    )
    assert (
        stop.index('kill -INT')
        < stop.index('kill -TERM')
        < stop.index('kill -KILL')
    )
    assert 'pkill' not in source
    assert 'killall' not in source
    assert 'close_instance_lock_fds_for_child' in source


def test_unregistered_live_child_is_not_signalled_or_waited_dynamically():
    bounded = _shell_function_source('bounded_positive_integer')
    function = _shell_function_source('stop_owned_component')
    harness = (
        'set -Eeuo pipefail\n'
        'declare -Ag owned_pids=() owned_groups=() owned_start_ticks=()\n'
        'ISAAC_NAV_SESSION_ID=test-session\n'
        'wait_called=false\n'
        'signal_called=false\n'
        'runtime_registered_process_group() { return 1; }\n'
        'runtime_process_is_running() { return 0; }\n'
        'runtime_process_group_is_running() { return 1; }\n'
        'runtime_pid_file() { printf "/tmp/never"; }\n'
        'runtime_metadata_value() { return 1; }\n'
        'log_warn() { printf "%s\\n" "$*" >&2; }\n'
        'wait() { wait_called=true; return 0; }\n'
        'kill() { signal_called=true; return 0; }\n'
        f'{bounded}\n'
        f'{function}\n'
        'owned_pids[isaac]=424242\n'
        'if stop_owned_component isaac "Isaac sentinel"; then exit 91; fi\n'
        '[[ "${wait_called}" == false ]]\n'
        '[[ "${signal_called}" == false ]]\n'
    )
    result = _bash_harness(harness, timeout=5)
    assert result.returncode == 0, result.stderr
    assert 'never registered an authenticated process group' in result.stderr


def test_contact_ab_matrix_has_bounded_cardinality_and_final_assertions():
    source = SCRIPT.read_text(encoding='utf-8')
    assert '10#${repeats} > 100' in source
    assert 'expected_conditions=' in source
    assert 'successful_rows=' in source
    assert 'manifest_rows=' in source
    assert 'completion count mismatch' in source
    assert 'bounded_positive_integer "${timeout_seconds}" 120' in source
    assert 'bounded_positive_integer "${timeout_seconds}" 900' in source
    assert 'bounded_positive_integer "${int_checks}" 600' in source


def test_contact_ab_matrix_runs_strict_final_aggregate_before_summary():
    source = SCRIPT.read_text(encoding='utf-8')
    finalizer = _shell_function_source('finalize_contact_analysis')
    assert 'from robot_experiments.contact_ab_analysis import (' in finalizer
    assert '"min_repeats": repeats' in finalizer
    assert 'arguments["require_complete_matrix"] = True' in finalizer
    assert 'arguments["expected_environments"] = (environment_selection,)' \
        in finalizer
    assert 'arguments["expected_profiles"] = COMPLETE_MATRIX_PROFILES' \
        in finalizer
    assert 'analysis.get("analysis_valid") is not True' in finalizer
    assert 'counts.get("excluded_reports") != 0' in finalizer
    assert 'counts.get("included_reports") != expected_runs' in finalizer
    assert 'counts.get("groups") != expected_groups' in finalizer
    assert 'write_contact_ab_report(analysis, output_path)' in finalizer

    loop_complete = source.rindex('\nexpected_conditions=')
    freeze_call = source.rindex('\nfreeze_manifest \\\n')
    analysis_call = source.rindex('\nfinalize_contact_analysis \\\n')
    analysis_hash = source.rindex('\nanalysis_sha256="$(')
    summary_call = source.rindex('\nwrite_batch_summary ')
    assert (
        loop_complete
        < freeze_call
        < analysis_call
        < analysis_hash
        < summary_call
    )


def test_batch_summary_atomically_records_frozen_evidence_hashes(tmp_path):
    function = _shell_function_source('write_batch_summary')
    manifest = tmp_path / 'manifest.tsv'
    analysis = tmp_path / 'analysis.json'
    manifest.write_text('header\nrow\n', encoding='utf-8')
    analysis.write_text(
        '{"counts":{"excluded_reports":0,"groups":6,'
        '"included_reports":12}}\n',
        encoding='utf-8',
    )
    manifest_sha256 = hashlib.sha256(manifest.read_bytes()).hexdigest()
    analysis_sha256 = hashlib.sha256(analysis.read_bytes()).hexdigest()
    profile_ids = (
        'legacy_baseline',
        'threshold_corr_0p00025_offset_0p0004',
        'threshold_corr_0p025_offset_0p0004',
        'threshold_corr_0p00025_offset_0p04',
        'threshold_corr_0p025_offset_0p04',
        'explicit_material',
    )
    profile_hashes = {
        profile_id: str(index) * 64
        for index, profile_id in enumerate(profile_ids, start=1)
    }
    summary = tmp_path / 'batch_summary.json'
    assignments = {
        'output_dir': tmp_path,
        'environment_selection': 'SimplePlane',
        'repeats': '2',
        'expected_conditions': '12',
        'expected_groups': '6',
        'batch_git_commit': 'a' * 40,
        'batch_git_branch': 'codex/test',
        'motion_config': '/repo/motion.yaml',
        'batch_motion_sha256': 'b' * 64,
        'warehouse_config': '/repo/warehouse.yaml',
        'batch_warehouse_project_sha256': 'c' * 64,
        'simple_plane_config': '/repo/simple.yaml',
        'batch_simple_plane_project_sha256': 'd' * 64,
        'physics_dir': '/repo/physics',
        'batch_profile_hashes_json': json.dumps(
            profile_hashes, sort_keys=True, separators=(',', ':')
        ),
        'manifest': manifest,
        'frozen_manifest_sha256': manifest_sha256,
        'analysis_path': analysis,
        'analysis_sha256': analysis_sha256,
    }
    shell_assignments = '\n'.join(
        f'{name}={str(value)!r}' for name, value in assignments.items()
    )
    result = _bash_harness(
        'set -Eeuo pipefail\n'
        'log_warn() { printf "%s\\n" "$*" >&2; }\n'
        f'{function}\n'
        f'{shell_assignments}\n'
        'write_batch_summary 12 12\n'
        f'[[ "${{batch_summary_path}}" == {str(summary)!r} ]]\n'
    )
    assert result.returncode == 0, result.stderr
    document = json.loads(summary.read_text(encoding='utf-8'))
    assert document['result'] == 'success'
    assert document['actual_counts']['analysis_included_reports'] == 12
    assert document['evidence']['manifest'] == {
        'path': str(manifest),
        'sha256': manifest_sha256,
    }
    assert document['evidence']['analysis'] == {
        'path': str(analysis),
        'sha256': analysis_sha256,
    }
    assert set(document['evidence']) == {'manifest', 'analysis'}
    assert not list(tmp_path.glob('.batch_summary.json.*.tmp'))


def test_shellcheck_when_available():
    shellcheck = shutil.which('shellcheck')
    if shellcheck is None:
        pytest.skip('shellcheck is not installed')
    result = subprocess.run(
        [shellcheck, str(SCRIPT)],
        cwd=REPOSITORY_ROOT,
        text=True,
        capture_output=True,
        check=False,
        timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr
