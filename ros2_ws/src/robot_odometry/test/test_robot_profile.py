from copy import deepcopy
import hashlib

import pytest
import yaml


TOP_LEVEL = {
    'schema_version': 3,
    'name': 'jackal',
    'kinematics_profile_id': 'jackal_legacy_geometric_v1',
    'lifecycle': 'stable_baseline',
    'wheel_radius': 0.098,
    'wheel_width': 0.040,
    'geometric_track_width': 0.37559,
    'effective_track_width': 0.37559,
    'wheelbase': 0.262,
    'base_mass': 17.0,
    'wheel_mass': 0.477,
    'nominal_total_mass': 18.908,
    'mass_collision_profile': (
        '../robot_mass_profiles/legacy_default_sensor_density_v1.yaml'),
    'wheel_velocity_drive': {
        'schema_version': 1,
        'profile_id': 'jackal_drive_test_v1',
        'drive_type': 'force',
        'stiffness_n_m_per_rad': 0.0,
        'damping_n_m_s_per_rad': 100.0,
        'max_effort_n_m': 20.0,
        'max_joint_velocity_rad_s': 25.0,
    },
    'physics': {},
    'wheel_joints': {
        'front_left': 'front_left_wheel_joint',
        'front_right': 'front_right_wheel_joint',
        'rear_left': 'rear_left_wheel_joint',
        'rear_right': 'rear_right_wheel_joint',
    },
    'controller': {
        'max_linear_speed': 1.0,
        'max_angular_speed': 1.5,
        'max_wheel_speed': 15.0,
        'max_acceleration': 0.75,
        'max_deceleration': 1.0,
        'max_angular_acceleration': 2.0,
    },
    'frames': {},
    'footprint': [],
    'static_transforms': [],
}


def _write_profile(tmp_path, document=None):
    source = tmp_path / 'robot.yaml'
    source.write_text(
        yaml.safe_dump(TOP_LEVEL if document is None else document),
        encoding='utf-8',
    )
    return source


def _document():
    return deepcopy(TOP_LEVEL)


def test_load_robot_profile_captures_single_source_kinematics(tmp_path):
    from robot_odometry.robot_profile import load_robot_profile

    source = _write_profile(tmp_path)
    profile = load_robot_profile(source)

    assert profile.source == source.resolve()
    assert profile.sha256 == hashlib.sha256(source.read_bytes()).hexdigest()
    assert profile.profile_id == 'jackal_legacy_geometric_v1'
    assert profile.lifecycle == 'stable_baseline'
    assert profile.wheel_radius_m == 0.098
    assert profile.wheel_width_m == 0.040
    assert profile.geometric_track_width_m == 0.37559
    assert profile.effective_track_width_m == 0.37559
    assert profile.wheelbase_m == 0.262
    assert profile.base_mass_kg == 17.0
    assert profile.wheel_mass_kg == 0.477
    assert profile.nominal_total_mass_kg == 18.908
    assert profile.left_joint_names == (
        'front_left_wheel_joint', 'rear_left_wheel_joint')
    assert profile.right_joint_names == (
        'front_right_wheel_joint', 'rear_right_wheel_joint')


@pytest.mark.parametrize('schema_version', [2, 3.0, True, None])
def test_robot_profile_requires_integer_schema_v3(tmp_path, schema_version):
    from robot_odometry.robot_profile import load_robot_profile

    document = _document()
    document['schema_version'] = schema_version
    with pytest.raises(ValueError, match='schema_version must be integer 3'):
        load_robot_profile(_write_profile(tmp_path, document))


@pytest.mark.parametrize('mutation', ['unknown', 'missing'])
def test_robot_profile_requires_exact_top_level_keys(tmp_path, mutation):
    from robot_odometry.robot_profile import load_robot_profile

    document = _document()
    if mutation == 'unknown':
        document['wheel_radius_copy'] = document['wheel_radius']
    else:
        del document['wheel_width']
    with pytest.raises(ValueError, match='robot config keys'):
        load_robot_profile(_write_profile(tmp_path, document))


@pytest.mark.parametrize(
    ('field', 'value', 'message'),
    [
        ('kinematics_profile_id', 'spaces are invalid', 'profile_id'),
        ('kinematics_profile_id', '', 'profile_id'),
        ('lifecycle', 'candidate', 'lifecycle'),
        ('wheel_radius', True, 'wheel_radius'),
        ('wheel_radius', 0.0, 'wheel_radius'),
        ('geometric_track_width', None, 'geometric_track_width'),
        ('effective_track_width', float('inf'), 'effective_track_width'),
        ('wheelbase', True, 'wheelbase'),
        ('base_mass', 0.0, 'base_mass'),
        ('wheel_mass', float('nan'), 'wheel_mass'),
        ('nominal_total_mass', -1.0, 'nominal_total_mass'),
    ],
)
def test_robot_profile_rejects_invalid_kinematics(
        tmp_path, field, value, message):
    from robot_odometry.robot_profile import load_robot_profile

    document = _document()
    document[field] = value
    with pytest.raises(ValueError, match=message):
        load_robot_profile(_write_profile(tmp_path, document))


@pytest.mark.parametrize(
    'mutation', ['unknown', 'missing', 'duplicate', 'unsafe'])
def test_robot_profile_requires_exact_unique_wheel_joints(tmp_path, mutation):
    from robot_odometry.robot_profile import load_robot_profile

    document = _document()
    if mutation == 'unknown':
        document['wheel_joints']['middle_left'] = 'middle_left_wheel_joint'
    elif mutation == 'missing':
        del document['wheel_joints']['rear_right']
    elif mutation == 'duplicate':
        document['wheel_joints']['rear_right'] = \
            document['wheel_joints']['front_right']
    else:
        document['wheel_joints']['rear_right'] = 'rear/right'
    with pytest.raises(ValueError, match='wheel_joints'):
        load_robot_profile(_write_profile(tmp_path, document))


def test_robot_profile_requires_consistent_nominal_total_mass(tmp_path):
    from robot_odometry.robot_profile import load_robot_profile

    document = _document()
    document['nominal_total_mass'] = 18.0
    with pytest.raises(ValueError, match='nominal_total_mass'):
        load_robot_profile(_write_profile(tmp_path, document))


@pytest.mark.parametrize('mutation', ['legacy_duplicate', 'missing'])
def test_robot_profile_requires_schema_v3_controller_contract(
        tmp_path, mutation):
    from robot_odometry.robot_profile import load_robot_profile

    document = _document()
    if mutation == 'legacy_duplicate':
        document['controller']['wheel_radius'] = document['wheel_radius']
    else:
        del document['controller']['max_wheel_speed']
    with pytest.raises(ValueError, match='controller keys'):
        load_robot_profile(_write_profile(tmp_path, document))


@pytest.mark.parametrize(
    ('mutation', 'message'),
    [
        ('missing_mass_profile', 'mass_collision_profile'),
        ('unknown_drive_key', 'wheel_velocity_drive keys'),
        ('wrong_drive_schema', 'wheel_velocity_drive.schema_version'),
        ('wrong_drive_type', 'wheel_velocity_drive.drive_type'),
        ('nonzero_stiffness', 'stiffness_n_m_per_rad'),
        ('nonpositive_limit', 'max_effort_n_m'),
        ('controller_exceeds_drive', 'max_wheel_speed'),
    ],
)
def test_robot_profile_requires_schema_v3_physics_identity(
        tmp_path, mutation, message):
    from robot_odometry.robot_profile import load_robot_profile

    document = _document()
    if mutation == 'missing_mass_profile':
        document['mass_collision_profile'] = ''
    elif mutation == 'unknown_drive_key':
        document['wheel_velocity_drive']['typo'] = 1
    elif mutation == 'wrong_drive_schema':
        document['wheel_velocity_drive']['schema_version'] = 2
    elif mutation == 'wrong_drive_type':
        document['wheel_velocity_drive']['drive_type'] = 'acceleration'
    elif mutation == 'nonzero_stiffness':
        document['wheel_velocity_drive']['stiffness_n_m_per_rad'] = 1.0
    elif mutation == 'nonpositive_limit':
        document['wheel_velocity_drive']['max_effort_n_m'] = 0.0
    else:
        document['controller']['max_wheel_speed'] = 30.0
    with pytest.raises(ValueError, match=message):
        load_robot_profile(_write_profile(tmp_path, document))


def test_robot_profile_rejects_duplicate_yaml_keys(tmp_path):
    from robot_odometry.robot_profile import load_robot_profile

    source = _write_profile(tmp_path)
    source.write_text(
        source.read_text(encoding='utf-8') + '\nwheel_radius: 0.5\n',
        encoding='utf-8',
    )
    with pytest.raises(ValueError, match='duplicate YAML key: wheel_radius'):
        load_robot_profile(source)


def _isaac_parameters(profile):
    return {
        'runtime_provenance.schema_version': 7,
        'runtime_provenance.robot.config.schema_version': 3,
        'runtime_provenance.robot.config.path': str(profile.source),
        'runtime_provenance.robot.config.sha256': profile.sha256,
        'runtime_provenance.robot.kinematics.profile_id': profile.profile_id,
        'runtime_provenance.robot.kinematics.lifecycle': profile.lifecycle,
        'runtime_provenance.robot.kinematics.wheel_radius_m': (
            profile.wheel_radius_m),
        'runtime_provenance.robot.kinematics.wheel_width_m': (
            profile.wheel_width_m),
        'runtime_provenance.robot.kinematics.geometric_track_width_m': (
            profile.geometric_track_width_m),
        'runtime_provenance.robot.kinematics.effective_track_width_m': (
            profile.effective_track_width_m),
        'runtime_provenance.robot.kinematics.controller_contract_verified': (
            True),
    }


def test_matching_isaac_v7_kinematics_is_accepted(tmp_path):
    from robot_odometry.robot_profile import load_robot_profile
    from robot_odometry.robot_profile import validate_isaac_kinematics

    profile = load_robot_profile(_write_profile(tmp_path))
    parameters = _isaac_parameters(profile)
    parameters['runtime_provenance.robot.config.path'] = str(
        profile.source.parent / 'unused' / '..' / profile.source.name)

    snapshot = validate_isaac_kinematics(profile, parameters)

    assert snapshot.schema_version == 7
    assert snapshot.config_schema_version == 3
    assert snapshot.config_path == profile.source
    assert snapshot.config_sha256 == profile.sha256
    assert snapshot.controller_contract_verified is True


def test_historical_v6_kinematics_is_rejected_by_live_handshake(tmp_path):
    from robot_odometry.robot_profile import load_robot_profile
    from robot_odometry.robot_profile import validate_isaac_kinematics

    profile = load_robot_profile(_write_profile(tmp_path))
    parameters = _isaac_parameters(profile)
    parameters['runtime_provenance.schema_version'] = 6

    with pytest.raises(ValueError, match='schema_version must be integer 7'):
        validate_isaac_kinematics(profile, parameters)


@pytest.mark.parametrize(
    ('parameter_name', 'replacement', 'message'),
    [
        ('runtime_provenance.schema_version', 7.0, 'schema_version'),
        ('runtime_provenance.schema_version', True, 'schema_version'),
        ('runtime_provenance.robot.config.schema_version', 2,
         'config.schema_version'),
        ('runtime_provenance.robot.config.schema_version', 3.0,
         'config.schema_version'),
        ('runtime_provenance.robot.config.path', '/tmp/other.yaml',
         'config.path'),
        ('runtime_provenance.robot.config.path', 12, 'config.path'),
        ('runtime_provenance.robot.config.sha256', '0' * 64,
         'config.sha256'),
        ('runtime_provenance.robot.config.sha256', 'not-a-digest',
         'config.sha256'),
        ('runtime_provenance.robot.kinematics.profile_id', 'other_profile',
         'profile_id'),
        ('runtime_provenance.robot.kinematics.lifecycle',
         'experimental_candidate', 'lifecycle'),
        ('runtime_provenance.robot.kinematics.wheel_radius_m', 0.099,
         'wheel_radius_m'),
        ('runtime_provenance.robot.kinematics.wheel_width_m', 0.05,
         'wheel_width_m'),
        ('runtime_provenance.robot.kinematics.geometric_track_width_m',
         0.4, 'geometric_track_width_m'),
        ('runtime_provenance.robot.kinematics.effective_track_width_m',
         0.4, 'effective_track_width_m'),
        ('runtime_provenance.robot.kinematics.controller_contract_verified',
         False, 'controller_contract_verified'),
        ('runtime_provenance.robot.kinematics.controller_contract_verified',
         1, 'controller_contract_verified'),
    ],
)
def test_isaac_kinematics_mismatch_fails_closed(
        tmp_path, parameter_name, replacement, message):
    from robot_odometry.robot_profile import load_robot_profile
    from robot_odometry.robot_profile import validate_isaac_kinematics

    profile = load_robot_profile(_write_profile(tmp_path))
    parameters = _isaac_parameters(profile)
    parameters[parameter_name] = replacement

    with pytest.raises(ValueError, match=message):
        validate_isaac_kinematics(profile, parameters)


@pytest.mark.parametrize('mutation', ['missing', 'unknown'])
def test_isaac_kinematics_requires_exact_parameter_set(tmp_path, mutation):
    from robot_odometry.robot_profile import load_robot_profile
    from robot_odometry.robot_profile import validate_isaac_kinematics

    profile = load_robot_profile(_write_profile(tmp_path))
    parameters = _isaac_parameters(profile)
    if mutation == 'missing':
        del parameters['runtime_provenance.robot.kinematics.lifecycle']
    else:
        parameters['runtime_provenance.robot.kinematics.unverified'] = True

    with pytest.raises(ValueError, match='Isaac kinematics parameter keys'):
        validate_isaac_kinematics(profile, parameters)


def test_isaac_handshake_rejects_robot_config_changed_after_loading(tmp_path):
    from robot_odometry.robot_profile import load_robot_profile
    from robot_odometry.robot_profile import validate_isaac_kinematics

    profile = load_robot_profile(_write_profile(tmp_path))
    parameters = _isaac_parameters(profile)
    profile.source.write_text(
        profile.source.read_text(encoding='utf-8') + '\n# changed\n',
        encoding='utf-8',
    )

    with pytest.raises(ValueError, match='changed after local loading'):
        validate_isaac_kinematics(profile, parameters)
