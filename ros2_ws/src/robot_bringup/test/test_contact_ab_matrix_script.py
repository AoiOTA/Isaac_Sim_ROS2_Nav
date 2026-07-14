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
    assert '--ground-topology baseline|all|ID' in help_result.stdout
    assert '--repeats N' in help_result.stdout
    assert '--robot-config FILE' in help_result.stdout
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
        (('--robot-config',), '--robot-config requires a value'),
        (('--ground-topology',), '--ground-topology requires a value'),
        (
            ('--ground-topology', 'unknown', '--output-dir', '/tmp/out'),
            '--ground-topology must be baseline, all',
        ),
        (
            (
                '--environment',
                'SimplePlane',
                '--ground-topology',
                'warehouse_combined32_v1',
                '--output-dir',
                '/tmp/out',
            ),
            '--ground-topology ID must match the selected --environment',
        ),
        (
            (
                '--ground-topology',
                'simple_plane_only1_v1',
                '--output-dir',
                '/tmp/out',
            ),
            '--ground-topology ID must match the selected --environment',
        ),
        (
            ('--robot-config', '', '--output-dir', '/tmp/out'),
            '--robot-config requires a non-empty value',
        ),
        (
            ('--output-dir', '/tmp/out\rhidden'),
            '--output-dir must not contain tabs, carriage returns, or '
            'newlines',
        ),
        (
            (
                '--robot-config',
                '/tmp/robot.yaml\rhidden',
                '--output-dir',
                '/tmp/out',
            ),
            '--robot-config must not contain tabs, carriage returns, or '
            'newlines',
        ),
        (('--unknown',), 'unknown contact A/B argument'),
    ],
)
def test_contact_ab_matrix_rejects_ambiguous_arguments(arguments, message):
    result = _run(*arguments)
    assert result.returncode != 0
    assert message in result.stderr


@pytest.mark.parametrize(
    (
        'environment_selection',
        'topology_selection',
        'expected_pairs',
        'expected_default_runs',
        'expected_default_groups',
    ),
    [
        (
            'all',
            'baseline',
            [
                ('SimplePlane', 'simple_plane_only1_v1'),
                ('Warehouse', 'warehouse_combined32_v1'),
            ],
            36,
            12,
        ),
        (
            'all',
            'all',
            [
                ('SimplePlane', 'simple_plane_only1_v1'),
                ('Warehouse', 'warehouse_combined32_v1'),
                ('Warehouse', 'warehouse_plane_only1_v1'),
            ],
            54,
            18,
        ),
        (
            'Warehouse',
            'all',
            [
                ('Warehouse', 'warehouse_combined32_v1'),
                ('Warehouse', 'warehouse_plane_only1_v1'),
            ],
            36,
            12,
        ),
        (
            'SimplePlane',
            'all',
            [('SimplePlane', 'simple_plane_only1_v1')],
            18,
            6,
        ),
        (
            'Warehouse',
            'warehouse_plane_only1_v1',
            [('Warehouse', 'warehouse_plane_only1_v1')],
            18,
            6,
        ),
    ],
)
def test_ground_topology_selection_produces_only_legal_pairs(
    environment_selection,
    topology_selection,
    expected_pairs,
    expected_default_runs,
    expected_default_groups,
):
    function = _shell_function_source('select_ground_topology_pairs')
    environments = (
        ['SimplePlane', 'Warehouse']
        if environment_selection == 'all'
        else [environment_selection]
    )
    environment_words = ' '.join(environments)
    result = _bash_harness(
        'set -Eeuo pipefail\n'
        f'environment_selection={environment_selection!r}\n'
        f'ground_topology_selection={topology_selection!r}\n'
        f'environments=({environment_words})\n'
        'declare -ag matrix_environment_ids=()\n'
        'declare -ag matrix_ground_topology_ids=()\n'
        f'{function}\n'
        'select_ground_topology_pairs\n'
        'for index in "${!matrix_environment_ids[@]}"; do\n'
        '  printf "%s\\t%s\\n" "${matrix_environment_ids[index]}" '
        '"${matrix_ground_topology_ids[index]}"\n'
        'done\n'
    )
    assert result.returncode == 0, result.stderr
    observed_pairs = [
        tuple(line.split('\t')) for line in result.stdout.splitlines()
    ]
    assert observed_pairs == expected_pairs
    assert len(observed_pairs) * 6 * 3 == expected_default_runs
    assert len(observed_pairs) * 6 == expected_default_groups


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
    assert 'for pair_index in "${!matrix_environment_ids[@]}"' in source
    assert 'for profile_index in "${!profile_ids[@]}"' in source
    assert 'for ((repeat = 1; repeat <= repeats; repeat++))' in source
    assert '"${SCRIPT_DIR}/run_isaac.sh"' in source
    assert '--headless --pacing-mode unbounded' in source
    assert (
        '--navigation-mode mapping --mode ideal --camera-profile off' in source
    )
    assert '"${SCRIPT_DIR}/run_motion_baseline.sh"' in source
    assert '--odometry-mode ideal' in source


def test_explicit_robot_config_must_be_canonical_absolute_regular_file(
    tmp_path,
):
    function = _shell_function_source('validate_robot_config_path')
    robot = tmp_path / 'robot.yaml'
    robot.write_text('schema_version: 2\n', encoding='utf-8')
    linked = tmp_path / 'robot-link.yaml'
    linked.symlink_to(robot)
    directory = tmp_path / 'directory'
    directory.mkdir()
    result = _bash_harness(
        'set -Eeuo pipefail\n'
        'log_warn() { printf "%s\\n" "$*" >&2; }\n'
        f'{function}\n'
        f'validate_robot_config_path {str(robot)!r}\n'
        f'if validate_robot_config_path {str(linked)!r}; then exit 91; fi\n'
        f'if validate_robot_config_path {str(directory)!r}; then exit 92; fi\n'
        'if validate_robot_config_path relative.yaml; then exit 93; fi\n'
    )
    assert result.returncode == 0, result.stderr
    assert 'canonical absolute regular file' in result.stderr


@pytest.mark.parametrize(
    'index_flag',
    ('--skip-worktree', '--assume-unchanged'),
)
def test_tracked_input_matches_head_even_when_index_hides_change(
    tmp_path,
    index_flag,
):
    repository = tmp_path / 'repository'
    repository.mkdir()
    protocol = repository / 'protocol.yaml'
    subprocess.run(
        ['git', 'init', '-q'],
        cwd=repository,
        check=True,
    )
    protocol.write_text('committed: true\n', encoding='utf-8')
    subprocess.run(['git', 'add', 'protocol.yaml'], cwd=repository, check=True)
    subprocess.run(
        [
            'git',
            '-c',
            'user.name=Test',
            '-c',
            'user.email=test@example.invalid',
            'commit',
            '-qm',
            'fixture',
        ],
        cwd=repository,
        check=True,
    )
    function = _shell_function_source('require_tracked_input')
    assert 'git hash-object --no-filters -- "${path}"' in function
    unchanged = _bash_harness(
        'set -Eeuo pipefail\n'
        f'PROJECT_ROOT={str(repository)!r}\n'
        'die() { printf "%s\\n" "$*" >&2; exit 64; }\n'
        f'{function}\n'
        f'require_tracked_input {str(protocol)!r}\n'
    )
    assert unchanged.returncode == 0, unchanged.stderr

    subprocess.run(
        ['git', 'update-index', index_flag, 'protocol.yaml'],
        cwd=repository,
        check=True,
    )
    protocol.write_text('committed: false\n', encoding='utf-8')
    hidden_status = subprocess.run(
        ['git', 'status', '--porcelain'],
        cwd=repository,
        check=True,
        text=True,
        capture_output=True,
    )
    assert hidden_status.stdout == ''
    changed = _bash_harness(
        'set -Eeuo pipefail\n'
        f'PROJECT_ROOT={str(repository)!r}\n'
        'die() { printf "%s\\n" "$*" >&2; exit 64; }\n'
        f'{function}\n'
        f'require_tracked_input {str(protocol)!r}\n'
    )
    assert changed.returncode != 0
    assert 'does not match the committed HEAD blob' in changed.stderr


def test_tracked_input_rejects_symlink_type_from_head(tmp_path):
    repository = tmp_path / 'repository'
    repository.mkdir()
    target = repository / 'target.yaml'
    target.write_text('target: true\n', encoding='utf-8')
    protocol = repository / 'protocol.yaml'
    protocol.symlink_to(target.name)
    subprocess.run(['git', 'init', '-q'], cwd=repository, check=True)
    subprocess.run(['git', 'add', '.'], cwd=repository, check=True)
    subprocess.run(
        [
            'git',
            '-c',
            'user.name=Test',
            '-c',
            'user.email=test@example.invalid',
            'commit',
            '-qm',
            'fixture',
        ],
        cwd=repository,
        check=True,
    )
    function = _shell_function_source('require_tracked_input')
    result = _bash_harness(
        'set -Eeuo pipefail\n'
        f'PROJECT_ROOT={str(repository)!r}\n'
        'die() { printf "%s\\n" "$*" >&2; exit 64; }\n'
        f'{function}\n'
        f'require_tracked_input {str(protocol)!r}\n'
    )
    assert result.returncode != 0
    assert 'committed regular file' in result.stderr


def test_only_explicit_matrix_inputs_are_reinstated_as_runtime_overrides():
    source = SCRIPT.read_text(encoding='utf-8')
    contract = _shell_function_source('project_runtime_contract')
    assert 'ISAAC_NAV__FILES__ROBOT' in contract
    launch = source[
        source.index('launch_isaac() {'):
        source.index('launch_motion_runner() {')
    ]
    clear_position = launch.index('clear_inherited_config_overrides')
    project_position = launch.index('export ISAAC_NAV_PROJECT_CONFIG=')
    contact_position = launch.index(
        'export ISAAC_NAV__FILES__CONTACT_PROFILE='
    )
    robot_position = launch.index('export ISAAC_NAV__FILES__ROBOT=')
    topology_position = launch.index(
        'export ISAAC_NAV__FILES__GROUND_TOPOLOGY_PROFILE='
    )
    assert (
        clear_position
        < project_position
        < contact_position
        < topology_position
        < robot_position
    )
    nested_exports = [
        line.strip()
        for line in launch.splitlines()
        if line.strip().startswith('export ISAAC_NAV__')
    ]
    assert nested_exports == [
        'export ISAAC_NAV__FILES__CONTACT_PROFILE="${profile_path}"',
        'export ISAAC_NAV__FILES__GROUND_TOPOLOGY_PROFILE="${ground_topology_path}"',
        'export ISAAC_NAV__FILES__ROBOT="${robot_config}"',
    ]


def test_contact_ab_matrix_is_fail_closed_on_git_readiness_and_reports():
    source = SCRIPT.read_text(encoding='utf-8')
    assert 'status --porcelain --untracked-files=normal' in source
    assert 'ls-files --error-unmatch' in source
    assert 'runtime_provenance.schema_version' in source
    assert '"${schema}" != 5' in source
    assert 'runtime_provenance.environment.id' in source
    assert 'runtime_provenance.contact.json' in source
    assert 'runtime_provenance.contact.sha256' in source
    assert 'runtime_provenance.ground_topology.json' in source
    assert 'runtime_provenance.ground_topology.sha256' in source
    assert 'ground_topology_readiness_matches' in source
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
    assert 'runtime_provenance.robot.kinematics.profile_id' in source
    assert 'runtime_provenance.robot.kinematics.lifecycle' in source
    assert 'runtime_provenance.robot.kinematics.wheel_radius_m' in source
    assert 'runtime_provenance.robot.kinematics.wheel_width_m' in source
    assert (
        'runtime_provenance.robot.kinematics.geometric_track_width_m'
        in source
    )
    assert (
        'runtime_provenance.robot.kinematics.effective_track_width_m'
        in source
    )
    assert (
        'runtime_provenance.robot.kinematics.controller_contract_verified'
        in source
    )
    assert 'runtime_provenance.environment.project_stage.path' in source
    assert 'runtime_provenance.environment.project_stage.sha256' in source
    assert 'runtime_provenance.environment.source_asset.path' in source
    assert 'runtime_provenance.environment.source_asset.sha256' in source
    assert 'runtime_provenance.simulation.physics_hz' in source
    assert 'runtime_provenance.simulation.navigation_mode' in source
    assert 'runtime_provenance.simulation.odometry_mode' in source
    assert 'analyse_contact_ab(' in source
    assert 'expected_topologies=(ground_topology_id,)' in source
    assert '[path],\n        wheel_radius,' in source
    assert 'analyse_contact_ab(report_paths, wheel_radius,' in source
    assert 'analyse_contact_ab(report_paths, 0.098' not in source
    assert 'min_repeats=1' in source
    assert 'workspace source deliberately' in source


def test_ros_parameter_boolean_contract_matches_jazzy_cli_output():
    """Jazzy --hide-type prints bool values as capitalized True/False."""
    function = _shell_function_source('ros_parameter_boolean_matches')
    result = _bash_harness(
        'set -Eeuo pipefail\n'
        f'{function}\n'
        'ros_parameter_boolean_matches True true\n'
        'ros_parameter_boolean_matches False false\n'
        'if ros_parameter_boolean_matches true true; then exit 91; fi\n'
        'if ros_parameter_boolean_matches false false; then exit 92; fi\n'
        'if ros_parameter_boolean_matches True false; then exit 93; fi\n'
    )
    assert result.returncode == 0, result.stderr


def test_ground_topology_readiness_requires_canonical_hashed_profile_contract():
    function = _shell_function_source('ground_topology_readiness_matches')
    profile_path = (
        REPOSITORY_ROOT
        / 'isaac_sim/configs/ground_topologies/simple_plane_only1_v1.yaml'
    )
    profile_sha256 = hashlib.sha256(profile_path.read_bytes()).hexdigest()
    source_asset_sha256 = (
        '63aa9b6b2ed4025aecc373e1c6442fa460f3ed8c670c7dbf0e9de2b41da322f3'
    )
    collider = '/Root/GroundPlane/CollisionPlane'
    collider_sha256 = (
        '093b0b40e3e87c6102b5e60ab009a27b36b45428ac4e61f424ea89d054448e3f'
    )
    topology = {
        'profile_path': str(profile_path),
        'profile_sha256': profile_sha256,
        'profile_id': 'simple_plane_only1_v1',
        'environment_id': 'SimplePlane',
        'operation': 'preserve_source_colliders',
        'source_asset_path': '/repo/simple.usda',
        'source_asset_sha256': source_asset_sha256,
        'overlay_identifier': 'anon:ground-topology',
        'overlay_sha256': 'a' * 64,
        'source_colliders': [collider],
        'source_collider_count': 1,
        'source_collider_paths_sha256': collider_sha256,
        'target_colliders': [collider],
        'target_collider_count': 1,
        'target_collider_paths_sha256': collider_sha256,
        'disabled_colliders': [],
        'disabled_collider_count': 0,
        'disabled_collider_paths_sha256': (
            '4f53cda18c2baa0c0354bb5f9a3ecbe5ed12ab4d8e11ba873c2f11161202b945'
        ),
        'stage_usd_readback_verified': True,
    }
    payload = json.dumps(
        topology, sort_keys=True, separators=(',', ':'), allow_nan=False
    )
    payload_sha256 = hashlib.sha256(payload.encode()).hexdigest()
    contact_payload = json.dumps(
        {'ground_colliders': [collider]},
        sort_keys=True,
        separators=(',', ':'),
    )
    arguments = [
        payload,
        payload_sha256,
        contact_payload,
        'simple_plane_only1_v1',
        str(profile_path),
        profile_sha256,
        'SimplePlane',
        '/repo/simple.usda',
        source_asset_sha256,
    ]
    quoted = ' '.join(repr(value) for value in arguments)
    bad_arguments = list(arguments)
    bad_arguments[1] = '0' * 64
    bad_quoted = ' '.join(repr(value) for value in bad_arguments)
    result = _bash_harness(
        'set -Eeuo pipefail\n'
        f'PROJECT_ROOT={str(REPOSITORY_ROOT)!r}\n'
        f'{function}\n'
        f'ground_topology_readiness_matches {quoted}\n'
        f'if ground_topology_readiness_matches {bad_quoted}; then exit 91; fi\n'
    )
    assert result.returncode == 0, result.stderr


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


def test_tsv_safe_strips_all_record_separators_dynamically():
    function = _shell_function_source('tsv_safe')
    result = _bash_harness(
        'set -Eeuo pipefail\n'
        f'{function}\n'
        "[[ \"$(tsv_safe $'one\\rtwo\\tthree\\nfour')\" "
        '== "one two three four" ]]\n'
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
    lock = _shell_function_source('lock_batch_identity')
    assert "rev-parse --verify 'HEAD^{commit}'" in source
    assert 'symbolic-ref --quiet --short HEAD' in source
    assert 'locked_input_paths=(' in source
    assert '"${robot_config}"' in lock
    assert 'batch_robot_config_sha256' in lock
    assert 'batch_profile_hashes_json' in source
    assert 'batch_ground_topology_hashes_json' in source
    assert 'ground_topology_path "${topology_id}"' in lock
    assert 'locked_input_paths+=("${topology_path}")' in lock
    assert 'profile_hash_arguments' in lock
    assert 'topology_hash_arguments' in lock
    assert 'locked_input_hashes[${profile_path}]' in lock
    assert 'locked_input_hashes[${topology_path}]' in lock
    assert '.read_bytes()' not in lock
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


def test_locked_topology_hash_map_never_rereads_mutated_file(tmp_path):
    function = _shell_function_source('lock_batch_identity')
    physics_dir = tmp_path / 'physics'
    topology_dir = tmp_path / 'topologies'
    physics_dir.mkdir()
    topology_dir.mkdir()
    profile_ids = (
        'legacy_baseline',
        'threshold_corr_0p00025_offset_0p0004',
        'threshold_corr_0p025_offset_0p0004',
        'threshold_corr_0p00025_offset_0p04',
        'threshold_corr_0p025_offset_0p04',
        'explicit_material',
    )
    for profile_id in profile_ids:
        (physics_dir / f'{profile_id}.yaml').write_text(
            f'{profile_id}\n', encoding='utf-8'
        )
    topology_id = 'simple_plane_only1_v1'
    topology_path = topology_dir / f'{topology_id}.yaml'
    topology_path.write_text('topology-a\n', encoding='utf-8')
    expected_digest = hashlib.sha256(topology_path.read_bytes()).hexdigest()
    fixed_inputs = {
        name: tmp_path / f'{name}.yaml'
        for name in ('motion', 'warehouse', 'simple_plane', 'robot')
    }
    for name, path in fixed_inputs.items():
        path.write_text(f'{name}\n', encoding='utf-8')
    robot_digest = hashlib.sha256(
        fixed_inputs['robot'].read_bytes()
    ).hexdigest()
    profile_array = ' '.join(repr(profile_id) for profile_id in profile_ids)
    harness = (
        'set -Eeuo pipefail\n'
        f'{function}\n'
        f'physics_dir={str(physics_dir)!r}\n'
        f'ground_topology_dir={str(topology_dir)!r}\n'
        f'topology_path={str(topology_path)!r}\n'
        f'motion_config={str(fixed_inputs["motion"])!r}\n'
        f'warehouse_config={str(fixed_inputs["warehouse"])!r}\n'
        f'simple_plane_config={str(fixed_inputs["simple_plane"])!r}\n'
        f'robot_config={str(fixed_inputs["robot"])!r}\n'
        f'robot_config_sha256={robot_digest!r}\n'
        f'profile_ids=({profile_array})\n'
        f'matrix_ground_topology_ids=({topology_id!r})\n'
        "matrix_environment_ids=('SimplePlane')\n"
        'declare -a locked_input_paths=()\n'
        'declare -A locked_input_hashes=()\n'
        "git_commit() { printf '%040d\\n' 1; }\n"
        "git_branch() { printf 'codex/test\\n'; }\n"
        'require_tracked_input() { return 0; }\n'
        'ground_topology_path() { '
        'printf "%s/%s.yaml\\n" "${ground_topology_dir}" "$1"; }\n'
        'die() { printf "%s\\n" "$*" >&2; return 1; }\n'
        'sha256_file() {\n'
        '  local digest\n'
        '  digest="$(sha256sum "$1" | awk \'{print $1}\')"\n'
        '  if [[ "$1" == "${topology_path}" ]]; then\n'
        "    printf 'topology-b\\n' >\"$1\"\n"
        '  fi\n'
        '  printf "%s\\n" "${digest}"\n'
        '}\n'
        'lock_batch_identity\n'
        'actual="$(python3 -c \'import json,sys; '
        'print(json.loads(sys.argv[1])["simple_plane_only1_v1"])\' '
        '"${batch_ground_topology_hashes_json}")"\n'
        f'[[ "${{actual}}" == {expected_digest!r} ]]\n'
        f'[[ "$(sha256sum "${{topology_path}}" | awk \'{{print $1}}\')" '
        f'!= {expected_digest!r} ]]\n'
    )
    result = _bash_harness(harness)
    assert result.returncode == 0, result.stderr


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
        '\t'.join(f'column_{index}' for index in range(43)) + '\n'
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
        'robot_config_selection': 'explicit_cli',
        'robot_config': '/repo/robot.yaml',
        'batch_robot_config_sha256': '9' * 64,
        'robot_kinematics_profile_id': 'jackal_candidate_v1',
        'robot_kinematics_lifecycle': 'experimental',
        'robot_wheel_radius': '0.098',
        'robot_wheel_width': '0.08',
        'robot_geometric_track_width': '0.37559',
        'robot_effective_track_width': '1.012',
        'current_project_config': '/repo/simple.yaml',
        'current_project_sha256': 'd' * 64,
        'current_profile_path': '/repo/legacy.yaml',
        'current_profile_sha256': 'e' * 64,
        'current_ground_topology_id': 'simple_plane_only1_v1',
        'current_ground_topology_path': '/repo/simple_plane_only1_v1.yaml',
        'current_ground_topology_sha256': '8' * 64,
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
    assert len(fields) == 43
    assert fields[8] == hashlib.sha256(report.read_bytes()).hexdigest()
    assert fields[10] == hashlib.sha256(isaac_log.read_bytes()).hexdigest()
    assert fields[12] == hashlib.sha256(runner_log.read_bytes()).hexdigest()
    assert fields[13] == 'a' * 40
    assert fields[16] == 'b' * 64
    assert fields[18] == 'c' * 64
    assert fields[20] == 'd' * 64
    assert fields[21] == 'explicit_cli'
    assert fields[22] == '/repo/robot.yaml'
    assert fields[23] == '9' * 64
    assert fields[24] == 'jackal_candidate_v1'
    assert fields[25] == 'experimental'
    assert fields[26:30] == ['0.098', '0.08', '0.37559', '1.012']
    assert fields[34:37] == [
        'simple_plane_only1_v1',
        '/repo/simple_plane_only1_v1.yaml',
        '8' * 64,
    ]


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
    assert 'plan 8.7 physical acceptance=PASS' in source
    assert 'plan 8.7 physical acceptance=FAIL' in source
    assert 'plan 8.7 physical acceptance=NOT_APPLICABLE' in source
    assert 'contact A/B evidence matrix complete' in source


def test_contact_ab_matrix_runs_strict_final_aggregate_before_summary():
    source = SCRIPT.read_text(encoding='utf-8')
    finalizer = _shell_function_source('finalize_contact_analysis')
    assert 'from robot_experiments.contact_ab_analysis import (' in finalizer
    assert '"min_repeats": repeats' in finalizer
    assert '"expected_environments": selected_environments' in finalizer
    assert '"expected_topologies": selected_topologies' in finalizer
    assert '"expected_profiles": COMPLETE_MATRIX_PROFILES' in finalizer
    assert 'row.get("ground_topology_id")' in finalizer
    assert 'analysis.get("analysis_valid") is not True' in finalizer
    assert 'analysis.get("schema_version") != 3' in finalizer
    assert 'counts.get("excluded_reports") != 0' in finalizer
    assert 'counts.get("included_reports") != expected_runs' in finalizer
    assert 'counts.get("groups") != expected_groups' in finalizer
    assert 'physical_acceptance.get("policy_id")' in finalizer
    assert 'aggregate physical acceptance group accounting is invalid' in finalizer
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
    manifest_header = (
        'environment',
        'ground_topology_id',
        'ground_topology_profile_path',
        'ground_topology_profile_sha256',
    )
    manifest_row = (
        'SimplePlane',
        'simple_plane_only1_v1',
        '/repo/ground_topologies/simple_plane_only1_v1.yaml',
        '7' * 64,
    )
    manifest.write_text(
        '\t'.join(manifest_header)
        + '\n'
        + ('\t'.join(manifest_row) + '\n') * 18,
        encoding='utf-8',
    )
    profile_ids = (
        'legacy_baseline',
        'threshold_corr_0p00025_offset_0p0004',
        'threshold_corr_0p025_offset_0p0004',
        'threshold_corr_0p00025_offset_0p04',
        'threshold_corr_0p025_offset_0p04',
        'explicit_material',
    )
    group_ids = sorted(
        f'SimplePlane::simple_plane_only1_v1::{profile_id}'
        for profile_id in profile_ids
    )
    physical_thresholds = {
        'forward_abs_lateral_drift_max_m': 0.05,
        'backward_abs_lateral_drift_max_m': 0.08,
        'rotation_center_drift_max_m': 0.10,
        'rotation_center_drift_asymmetry_ratio_max': 0.20,
        'rotation_mean_yaw_rate_absolute_error_fraction_max': 0.10,
        'stop_stable_duration_min_sec': 0.5,
        'stop_linear_velocity_threshold_max_mps': 0.02,
        'stop_angular_velocity_threshold_max_radps': 0.05,
        'stop_wheel_velocity_threshold_max_radps': 0.20,
    }

    def physical_checks(group_passed):
        maximum_checks = {
            'forward_abs_lateral_drift_m': (
                0.0 if group_passed else 0.051,
                0.05,
            ),
            'backward_abs_lateral_drift_m': (0.0, 0.08),
            'rotate_left_center_drift_m': (0.0, 0.10),
            'rotate_right_center_drift_m': (0.0, 0.10),
            'rotation_center_drift_asymmetry_ratio': (0.0, 0.20),
            'stop_config.linear_velocity_threshold_mps': (0.0, 0.02),
            'stop_config.angular_velocity_threshold_radps': (0.0, 0.05),
            'stop_config.wheel_velocity_threshold_radps': (0.0, 0.20),
        }
        checks = {
            check_id: {
                'observed': observed,
                'maximum': maximum,
                'passed': (
                    group_passed
                    if check_id == 'forward_abs_lateral_drift_m'
                    else True
                ),
            }
            for check_id, (observed, maximum) in maximum_checks.items()
        }
        checks['stop_config.stable_duration_sec'] = {
            'observed': 0.5,
            'minimum': 0.5,
            'passed': True,
        }
        for side, command in (('left', 0.4), ('right', -0.4)):
            checks[
                f'rotate_{side}_mean_yaw_rate_absolute_error_fraction'
            ] = {
                'observed': 0.0,
                'maximum': 0.10,
                'passed': True,
                'commanded_yaw_rate_radps': command,
                'steady_state_mean_yaw_rate_radps': command,
                'steady_state_measurement_basis': (
                    'actual_velocity.steady_state_window.'
                    'angular_z_radps.mean'
                ),
            }
        for segment_id in (
            'rotate_left_360',
            'rotate_right_360',
            'forward_3m',
            'backward_2m',
            'arc_left_5s',
            'arc_right_5s',
        ):
            checks[f'stop_window.{segment_id}'] = {
                'stopped': True,
                'stationary_onset_sec': 0.0,
                'confirmed_sec': 0.5,
                'observed_stable_duration_sec': 0.5,
                'required_stable_duration_sec': 0.5,
                'passed': True,
            }
        checks['wheel_direction_contract'] = {
            'validated_segment_count': 6,
            'validated_by': 'strict_motion_report_validator',
            'passed': True,
        }
        return checks

    analysis_groups = {}
    acceptance_groups = {}
    selection_included = []
    for group_index, group_id in enumerate(group_ids):
        contact_profile_id = group_id.rsplit('::', 1)[1]
        group_passed = group_index < 2
        input_reports = []
        repeat_results = []
        for repeat_index in range(1, 4):
            label = f'{group_id}-repeat-{repeat_index}'
            report_path = f'/repo/reports/{label}.json'
            report_sha256 = hashlib.sha256(label.encode()).hexdigest()
            canonical_sha256 = hashlib.sha256(
                f'canonical-{label}'.encode()
            ).hexdigest()
            input_reports.append(
                {
                    'path': report_path,
                    'sha256': report_sha256,
                    'canonical_sha256': canonical_sha256,
                    'report_schema_version': 2,
                }
            )
            selection_included.append(
                {
                    'path': report_path,
                    'sha256': report_sha256,
                    'canonical_sha256': canonical_sha256,
                    'report_schema_version': 2,
                    'environment_id': 'SimplePlane',
                    'ground_topology_id': 'simple_plane_only1_v1',
                    'contact_profile_id': contact_profile_id,
                }
            )
            checks = physical_checks(group_passed)
            repeat_results.append(
                {
                    'repeat_index': repeat_index,
                    'report_path': report_path,
                    'report_sha256': report_sha256,
                    'canonical_sha256': canonical_sha256,
                    'passed': group_passed,
                    'checks': checks,
                    'failed_checks': (
                        []
                        if group_passed
                        else ['forward_abs_lateral_drift_m']
                    ),
                }
            )
        analysis_groups[group_id] = {
            'runtime_provenance_schema': 5,
            'environment_id': 'SimplePlane',
            'ground_topology_id': 'simple_plane_only1_v1',
            'odometry_mode': 'ideal',
            'contact_profile_id': contact_profile_id,
            'repeat_count': 3,
            'input_reports': input_reports,
        }
        check_ids = sorted(repeat_results[0]['checks'])
        acceptance_groups[group_id] = {
            'applicable': True,
            'passed': group_passed,
            'not_applicable_reasons': [],
            'repeat_count': 3,
            'checks': {
                check_id: {
                    'passed_repeats': sum(
                        repeat['checks'][check_id]['passed']
                        for repeat in repeat_results
                    ),
                    'failed_repeats': sum(
                        not repeat['checks'][check_id]['passed']
                        for repeat in repeat_results
                    ),
                    'all_repeats_passed': all(
                        repeat['checks'][check_id]['passed']
                        for repeat in repeat_results
                    ),
                }
                for check_id in check_ids
            },
            'failed_checks': (
                [] if group_passed else ['forward_abs_lateral_drift_m']
            ),
            'repeat_results': repeat_results,
        }
    analysis_document = {
        'schema_version': 3,
        'counts': {
            'input_reports': 18,
            'excluded_reports': 0,
            'groups': 6,
            'included_reports': 18,
        },
        'selection': {
            'included': selection_included,
            'excluded': [],
        },
        'matrix': {
            'complete': True,
            'required_groups': group_ids,
            'observed_groups': group_ids,
            'missing_groups': [],
        },
        'selection_policy': {
            'required_runtime_provenance_schema': 5,
            'expected_profiles': list(profile_ids),
        },
        'locked_inputs': {
            'simulation': {'odometry_mode': 'ideal'},
        },
        'groups': analysis_groups,
        'physical_acceptance': {
            'schema_version': 1,
            'policy_id': 'skid_steer_plan_8_7_v1',
            'evaluation_basis': 'every_repeat',
            'ranking_policy': 'none; pass/fail only',
            'steady_state_measurement_basis': (
                'actual_velocity.steady_state_window.angular_z_radps.mean '
                'over the final_half_of_command_interval window'
            ),
            'thresholds': physical_thresholds,
            'applicability': {
                'required_runtime_provenance_schema': 5,
                'required_environment_id': 'SimplePlane',
                'required_ground_topology_id': 'simple_plane_only1_v1',
                'required_odometry_mode': 'ideal',
                'minimum_unique_repeats_per_group': 3,
            },
            'groups': acceptance_groups,
            'applicable_groups': group_ids,
            'not_applicable_groups': [],
            'passing_groups': group_ids[:2],
            'failed_groups': group_ids[2:],
            'all_applicable_groups_passed': False,
        },
    }
    analysis.write_text(
        json.dumps(analysis_document, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    manifest_sha256 = hashlib.sha256(manifest.read_bytes()).hexdigest()
    analysis_sha256 = hashlib.sha256(analysis.read_bytes()).hexdigest()
    profile_hashes = {
        profile_id: str(index) * 64
        for index, profile_id in enumerate(profile_ids, start=1)
    }
    summary = tmp_path / 'batch_summary.json'
    assignments = {
        'PROJECT_ROOT': REPOSITORY_ROOT,
        'output_dir': tmp_path,
        'environment_selection': 'SimplePlane',
        'ground_topology_selection': 'baseline',
        'batch_environment_topology_pairs_json': json.dumps(
            [
                {
                    'environment_id': 'SimplePlane',
                    'ground_topology_id': 'simple_plane_only1_v1',
                }
            ],
            sort_keys=True,
            separators=(',', ':'),
        ),
        'repeats': '3',
        'expected_conditions': '18',
        'expected_groups': '6',
        'batch_git_commit': 'a' * 40,
        'batch_git_branch': 'codex/test',
        'motion_config': '/repo/motion.yaml',
        'batch_motion_sha256': 'b' * 64,
        'warehouse_config': '/repo/warehouse.yaml',
        'batch_warehouse_project_sha256': 'c' * 64,
        'simple_plane_config': '/repo/simple.yaml',
        'batch_simple_plane_project_sha256': 'd' * 64,
        'robot_config_selection': 'explicit_cli',
        'robot_config': '/repo/robot.yaml',
        'batch_robot_config_sha256': '9' * 64,
        'robot_kinematics_profile_id': 'jackal_candidate_v1',
        'robot_kinematics_lifecycle': 'experimental',
        'robot_wheel_radius': '0.098',
        'robot_wheel_width': '0.08',
        'robot_geometric_track_width': '0.37559',
        'robot_effective_track_width': '1.012',
        'physics_dir': '/repo/physics',
        'batch_profile_hashes_json': json.dumps(
            profile_hashes, sort_keys=True, separators=(',', ':')
        ),
        'ground_topology_dir': '/repo/ground_topologies',
        'batch_ground_topology_hashes_json': json.dumps(
            {'simple_plane_only1_v1': '7' * 64},
            sort_keys=True,
            separators=(',', ':'),
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
        'write_batch_summary 18 18\n'
        f'[[ "${{batch_summary_path}}" == {str(summary)!r} ]]\n'
    )
    assert result.returncode == 0, result.stderr
    document = json.loads(summary.read_text(encoding='utf-8'))
    assert document['result'] == 'success'
    assert document['schema_version'] == 4
    assert document['ground_topology_selection'] == 'baseline'
    assert document['environment_topology_pairs'] == [
        {
            'environment_id': 'SimplePlane',
            'ground_topology_id': 'simple_plane_only1_v1',
        }
    ]
    assert document['actual_counts']['analysis_included_reports'] == 18
    assert document['actual_counts']['acceptance_applicable_groups'] == 6
    assert document['actual_counts']['acceptance_not_applicable_groups'] == 0
    assert document['actual_counts']['acceptance_passing_groups'] == 2
    assert document['actual_counts']['acceptance_failed_groups'] == 4
    assert document['physical_acceptance'] == {
        'schema_version': 1,
        'policy_id': 'skid_steer_plan_8_7_v1',
        'evaluation_basis': 'every_repeat',
        'ranking_policy': 'none; pass/fail only',
        'thresholds': physical_thresholds,
        'applicability': {
            'required_runtime_provenance_schema': 5,
            'required_environment_id': 'SimplePlane',
            'required_ground_topology_id': 'simple_plane_only1_v1',
            'required_odometry_mode': 'ideal',
            'minimum_unique_repeats_per_group': 3,
        },
        'all_applicable_groups_passed': False,
        'applicable_groups': group_ids,
        'not_applicable_groups': [],
        'passing_groups': group_ids[:2],
        'failed_groups': group_ids[2:],
    }
    assert document['evidence']['manifest'] == {
        'path': str(manifest),
        'sha256': manifest_sha256,
    }
    assert document['evidence']['analysis'] == {
        'path': str(analysis),
        'sha256': analysis_sha256,
    }
    assert set(document['evidence']) == {'manifest', 'analysis'}
    assert document['locked_protocol_inputs']['robot_config'] == {
        'selection': 'explicit_cli',
        'path': '/repo/robot.yaml',
        'sha256': '9' * 64,
        'kinematics': {
            'profile_id': 'jackal_candidate_v1',
            'lifecycle': 'experimental',
            'wheel_radius_m': 0.098,
            'wheel_width_m': 0.08,
            'geometric_track_width_m': 0.37559,
            'effective_track_width_m': 1.012,
        },
    }
    assert document['locked_protocol_inputs']['ground_topology_profiles'] == {
        'simple_plane_only1_v1': {
            'environment_id': 'SimplePlane',
            'path': '/repo/ground_topologies/simple_plane_only1_v1.yaml',
            'sha256': '7' * 64,
        }
    }
    assert not list(tmp_path.glob('.batch_summary.json.*.tmp'))

    # A top-level pass/fail list is not enough to claim every-repeat evidence.
    # Removing one repeat must fail before an atomic summary is published.
    summary.unlink()
    tampered_analysis = json.loads(json.dumps(analysis_document))
    tampered_analysis['physical_acceptance']['groups'][group_ids[0]][
        'repeat_results'
    ].pop()
    analysis.write_text(
        json.dumps(tampered_analysis, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    assignments['analysis_sha256'] = hashlib.sha256(
        analysis.read_bytes()
    ).hexdigest()
    tampered_assignments = '\n'.join(
        f'{name}={str(value)!r}' for name, value in assignments.items()
    )
    tampered_result = _bash_harness(
        'set -Eeuo pipefail\n'
        'log_warn() { printf "%s\\n" "$*" >&2; }\n'
        f'{function}\n'
        f'{tampered_assignments}\n'
        'write_batch_summary 18 18\n'
    )
    assert tampered_result.returncode != 0
    assert 'every-repeat physical acceptance evidence is invalid' \
        in tampered_result.stderr
    assert not summary.exists()

    analysis.write_text(
        json.dumps(analysis_document, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    assignments['analysis_sha256'] = hashlib.sha256(
        analysis.read_bytes()
    ).hexdigest()

    # The frozen file digest alone is insufficient: the summary must bind
    # every row's topology identity to the originally locked hash map.
    mismatched_row = (*manifest_row[:-1], '6' * 64)
    manifest.write_text(
        '\t'.join(manifest_header)
        + '\n'
        + ('\t'.join(mismatched_row) + '\n') * 18,
        encoding='utf-8',
    )
    assignments['frozen_manifest_sha256'] = hashlib.sha256(
        manifest.read_bytes()
    ).hexdigest()
    mismatched_assignments = '\n'.join(
        f'{name}={str(value)!r}' for name, value in assignments.items()
    )
    mismatch_result = _bash_harness(
        'set -Eeuo pipefail\n'
        'log_warn() { printf "%s\\n" "$*" >&2; }\n'
        f'{function}\n'
        f'{mismatched_assignments}\n'
        'write_batch_summary 18 18\n'
    )
    assert mismatch_result.returncode != 0
    assert 'manifest row 1 ground-topology identity mismatch' \
        in mismatch_result.stderr
    assert not summary.exists()


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
