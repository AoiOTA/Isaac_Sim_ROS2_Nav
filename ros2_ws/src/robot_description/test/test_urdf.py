import ast
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import xacro
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[2]
XACRO_FILE = PACKAGE_ROOT / 'urdf' / 'jackal.urdf.xacro'
ROBOT_CONFIG_FILE = (
    REPOSITORY_ROOT / 'isaac_sim' / 'configs' / 'robots' / 'jackal.yaml')
PHYSICAL_GEOMETRY_KEYS = (
    'wheel_radius',
    'wheel_width',
    'geometric_track_width',
    'wheelbase',
    'base_mass',
    'wheel_mass',
)
WHEEL_JOINT_KEYS = (
    'front_left',
    'front_right',
    'rear_left',
    'rear_right',
)
ROBOT_CONFIG_FIELDS = {
    'schema_version',
    'name',
    'kinematics_profile_id',
    'lifecycle',
    'wheel_radius',
    'wheel_width',
    'geometric_track_width',
    'effective_track_width',
    'wheelbase',
    'base_mass',
    'wheel_mass',
    'nominal_total_mass',
    'physics',
    'wheel_joints',
    'controller',
    'frames',
    'footprint',
    'static_transforms',
}


def _description_launch_module():
    launch_file = PACKAGE_ROOT / 'launch' / 'description.launch.py'
    spec = importlib.util.spec_from_file_location(
        'robot_description_launch_under_test', launch_file)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


DESCRIPTION_LAUNCH = _description_launch_module()


def _robot_config(path=ROBOT_CONFIG_FILE):
    with Path(path).open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def _robot_root(robot_config_file=ROBOT_CONFIG_FILE, prefix=''):
    geometry = DESCRIPTION_LAUNCH._load_robot_geometry(robot_config_file)
    config = _robot_config(robot_config_file)
    mappings = {
        key: format(geometry[key], '.17g')
        for key in PHYSICAL_GEOMETRY_KEYS
    }
    mappings['prefix'] = prefix
    mappings.update({
        f'{key}_joint_name': config['wheel_joints'][key]
        for key in WHEEL_JOINT_KEYS
    })
    document = xacro.process_file(
        str(XACRO_FILE),
        mappings=mappings,
    )
    return ET.fromstring(document.toxml())


def _joint_xyz(root, name):
    joint = next(
        element for element in root.findall('joint')
        if element.attrib['name'] == name)
    return tuple(float(value) for value in joint.find('origin').attrib['xyz'].split())


def _wheel_link(root, name):
    return next(
        element for element in root.findall('link')
        if element.attrib['name'] == f'{name}_wheel_link')


def _write_robot_config(path, config):
    path.write_text(yaml.safe_dump(config, sort_keys=False), encoding='utf-8')


def test_repository_robot_config_drives_physical_geometry_and_mass():
    config = _robot_config()
    root = _robot_root()

    front_left = _joint_xyz(root, 'front_left_wheel_joint')
    front_right = _joint_xyz(root, 'front_right_wheel_joint')
    rear_left = _joint_xyz(root, 'rear_left_wheel_joint')
    rear_right = _joint_xyz(root, 'rear_right_wheel_joint')
    expected_x = config['wheelbase'] / 2.0
    expected_y = config['geometric_track_width'] / 2.0
    assert front_left[:2] == pytest.approx((expected_x, expected_y))
    assert front_right[:2] == pytest.approx((expected_x, -expected_y))
    assert rear_left[:2] == pytest.approx((-expected_x, expected_y))
    assert rear_right[:2] == pytest.approx((-expected_x, -expected_y))
    assert front_left[1] - front_right[1] == pytest.approx(
        config['geometric_track_width'])
    assert front_left[0] - rear_left[0] == pytest.approx(config['wheelbase'])

    base_link = next(
        element for element in root.findall('link')
        if element.attrib['name'] == 'base_link')
    assert float(base_link.find('inertial/mass').attrib['value']) \
        == pytest.approx(config['base_mass'])

    for wheel_name in ('front_left', 'front_right', 'rear_left', 'rear_right'):
        wheel_link = _wheel_link(root, wheel_name)
        assert float(wheel_link.find('inertial/mass').attrib['value']) \
            == pytest.approx(config['wheel_mass'])
        for geometry_path in ('visual/geometry/cylinder',
                              'collision/geometry/cylinder'):
            cylinder = wheel_link.find(geometry_path)
            assert float(cylinder.attrib['radius']) \
                == pytest.approx(config['wheel_radius'])
            assert float(cylinder.attrib['length']) \
                == pytest.approx(config['wheel_width'])


def test_xacro_uses_geometric_values_but_not_effective_track_width(tmp_path):
    config = _robot_config()
    config.update({
        'geometric_track_width': 0.42,
        'effective_track_width': 9.99,
        'wheelbase': 0.31,
    })
    config_file = tmp_path / 'robot.yaml'
    _write_robot_config(config_file, config)

    root = _robot_root(config_file)
    front_left = _joint_xyz(root, 'front_left_wheel_joint')
    front_right = _joint_xyz(root, 'front_right_wheel_joint')
    rear_left = _joint_xyz(root, 'rear_left_wheel_joint')
    assert front_left[1] - front_right[1] == pytest.approx(0.42)
    assert front_left[0] - rear_left[0] == pytest.approx(0.31)
    assert abs(front_left[1]) != pytest.approx(9.99 / 2.0)

    wheel_link = _wheel_link(root, 'front_left')
    cylinder = wheel_link.find('visual/geometry/cylinder')
    assert float(cylinder.attrib['radius']) == pytest.approx(
        config['wheel_radius'])
    assert float(cylinder.attrib['length']) == pytest.approx(
        config['wheel_width'])
    assert float(wheel_link.find('inertial/mass').attrib['value']) \
        == pytest.approx(config['wheel_mass'])

    base_link = next(
        element for element in root.findall('link')
        if element.attrib['name'] == 'base_link')
    assert float(base_link.find('inertial/mass').attrib['value']) \
        == pytest.approx(config['base_mass'])


def test_effective_track_width_does_not_change_rendered_urdf(tmp_path):
    baseline = _robot_config()
    candidate = dict(baseline)
    candidate['effective_track_width'] = 9.99
    baseline_file = tmp_path / 'baseline.yaml'
    candidate_file = tmp_path / 'candidate.yaml'
    _write_robot_config(baseline_file, baseline)
    _write_robot_config(candidate_file, candidate)

    baseline_xml = ET.tostring(_robot_root(baseline_file))
    candidate_xml = ET.tostring(_robot_root(candidate_file))
    assert candidate_xml == baseline_xml


def test_robot_yaml_wheel_joint_names_are_rendered_verbatim(tmp_path):
    config = _robot_config()
    config['wheel_joints'] = {
        'front_left': 'fl_drive_joint',
        'front_right': 'fr_drive_joint',
        'rear_left': 'rl_drive_joint',
        'rear_right': 'rr_drive_joint',
    }
    config_file = tmp_path / 'robot.yaml'
    _write_robot_config(config_file, config)

    root = _robot_root(config_file)
    joint_names = {joint.attrib['name'] for joint in root.findall('joint')}
    assert set(config['wheel_joints'].values()) <= joint_names
    assert 'front_left_wheel_joint' not in joint_names


def test_visual_link_prefix_does_not_rename_yaml_wheel_joints():
    config = _robot_config()
    root = _robot_root(prefix='robot_1_')

    joint_names = {joint.attrib['name'] for joint in root.findall('joint')}
    assert set(config['wheel_joints'].values()) <= joint_names
    assert not {
        f'robot_1_{joint_name}'
        for joint_name in config['wheel_joints'].values()
    } & joint_names
    assert root.find("link[@name='robot_1_base_link']") is not None


@pytest.mark.parametrize('mutation', ['unknown', 'missing', 'duplicate', 'unsafe'])
def test_robot_geometry_loader_rejects_invalid_wheel_joint_contract(
        tmp_path, mutation):
    config = _robot_config()
    joints = config['wheel_joints']
    if mutation == 'unknown':
        joints['middle_left'] = 'middle_left_wheel_joint'
    elif mutation == 'missing':
        joints.pop('rear_right')
    elif mutation == 'duplicate':
        joints['rear_right'] = joints['front_right']
    else:
        joints['rear_right'] = 'rear right wheel joint'
    config_file = tmp_path / 'robot.yaml'
    _write_robot_config(config_file, config)

    with pytest.raises(RuntimeError, match='wheel_joints'):
        DESCRIPTION_LAUNCH._load_robot_geometry(config_file)


def test_robot_geometry_loader_requires_consistent_nominal_total_mass(tmp_path):
    config = _robot_config()
    config['nominal_total_mass'] = 18.0
    config_file = tmp_path / 'robot.yaml'
    _write_robot_config(config_file, config)

    with pytest.raises(RuntimeError, match='nominal_total_mass'):
        DESCRIPTION_LAUNCH._load_robot_geometry(config_file)


@pytest.mark.parametrize(
    ('field', 'value'),
    (
        ('wheel_radius', 0.11),
        ('wheel_width', 0.05),
        ('base_mass', 18.0),
        ('wheel_mass', 0.6),
    ),
)
def test_fixed_jackal_inertia_rejects_incompatible_inputs(
        tmp_path, field, value):
    config = _robot_config()
    config[field] = value
    config['nominal_total_mass'] = (
        config['base_mass'] + 4.0 * config['wheel_mass'])
    config_file = tmp_path / 'robot.yaml'
    _write_robot_config(config_file, config)

    with pytest.raises(RuntimeError, match='fixed Jackal inertia'):
        DESCRIPTION_LAUNCH._load_robot_geometry(config_file)


@pytest.mark.parametrize(
    ('mutation', 'message'),
    (
        (lambda config: config.update(schema_version=1), 'schema_version'),
        (lambda config: config.pop('wheel_radius'), 'wheel_radius'),
        (lambda config: config.update(wheel_width=True), 'wheel_width'),
        (lambda config: config.update(geometric_track_width='0.4'),
         'geometric_track_width'),
        (lambda config: config.update(wheelbase=0.0), 'wheelbase'),
        (lambda config: config.update(base_mass=float('inf')), 'base_mass'),
        (lambda config: config.update(wheel_mass=-0.1), 'wheel_mass'),
    ),
)
def test_robot_geometry_loader_rejects_invalid_schema_v2_values(
        tmp_path, mutation, message):
    config = _robot_config()
    mutation(config)
    config_file = tmp_path / 'robot.yaml'
    _write_robot_config(config_file, config)

    with pytest.raises(RuntimeError, match=message):
        DESCRIPTION_LAUNCH._load_robot_geometry(config_file)


@pytest.mark.parametrize(
    ('field', 'value', 'message'),
    (
        ('kinematics_profile_id', 'path/profile', 'kinematics_profile_id'),
        ('kinematics_profile_id', '', 'kinematics_profile_id'),
        ('lifecycle', 'stable', 'lifecycle'),
        ('lifecycle', True, 'lifecycle'),
        ('effective_track_width', True, 'effective_track_width'),
        ('effective_track_width', float('inf'), 'effective_track_width'),
        ('effective_track_width', 0.0, 'effective_track_width'),
    ),
)
def test_robot_geometry_loader_rejects_invalid_shared_kinematics_metadata(
        tmp_path, field, value, message):
    config = _robot_config()
    config[field] = value
    config_file = tmp_path / 'robot.yaml'
    _write_robot_config(config_file, config)

    with pytest.raises(RuntimeError, match=message):
        DESCRIPTION_LAUNCH._load_robot_geometry(config_file)


def test_robot_geometry_loader_rejects_missing_or_non_mapping_yaml(tmp_path):
    with pytest.raises(RuntimeError, match='does not exist'):
        DESCRIPTION_LAUNCH._load_robot_geometry(tmp_path / 'missing.yaml')

    config_file = tmp_path / 'robot.yaml'
    config_file.write_text('- not\n- a\n- mapping\n', encoding='utf-8')
    with pytest.raises(RuntimeError, match='mapping'):
        DESCRIPTION_LAUNCH._load_robot_geometry(config_file)


def test_robot_geometry_loader_rejects_non_utf8_yaml(tmp_path):
    config_file = tmp_path / 'robot.yaml'
    config_file.write_bytes(b'\xff\xfe\x00\x01')

    with pytest.raises(RuntimeError, match='failed to read'):
        DESCRIPTION_LAUNCH._load_robot_geometry(config_file)


def test_robot_geometry_loader_rejects_unknown_top_level_keys(tmp_path):
    config = _robot_config()
    config['wheelbase_typo'] = config['wheelbase']
    config_file = tmp_path / 'robot.yaml'
    _write_robot_config(config_file, config)

    with pytest.raises(RuntimeError, match='unknown.*wheelbase_typo'):
        DESCRIPTION_LAUNCH._load_robot_geometry(config_file)


def test_robot_geometry_loader_rejects_duplicate_top_level_keys(tmp_path):
    source = ROBOT_CONFIG_FILE.read_text(encoding='utf-8')
    source = source.replace(
        'wheel_radius: 0.098\n',
        'wheel_radius: 0.098\nwheel_radius: 0.098\n',
        1,
    )
    config_file = tmp_path / 'robot.yaml'
    config_file.write_text(source, encoding='utf-8')

    with pytest.raises(RuntimeError, match='duplicate.*wheel_radius'):
        DESCRIPTION_LAUNCH._load_robot_geometry(config_file)


@pytest.mark.parametrize('field', sorted(ROBOT_CONFIG_FIELDS))
def test_robot_geometry_loader_requires_every_schema_v2_top_level_key(
        tmp_path, field):
    config = _robot_config()
    config.pop(field)
    config_file = tmp_path / 'robot.yaml'
    _write_robot_config(config_file, config)

    with pytest.raises(RuntimeError, match=f'missing.*{field}'):
        DESCRIPTION_LAUNCH._load_robot_geometry(config_file)


def test_robot_config_file_is_a_required_launch_argument():
    launch_file = PACKAGE_ROOT / 'launch' / 'description.launch.py'
    tree = ast.parse(launch_file.read_text(encoding='utf-8'))
    declarations = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == 'DeclareLaunchArgument'
        and node.args
        and isinstance(node.args[0], ast.Constant)
        and node.args[0].value == 'robot_config_file'
    ]
    assert len(declarations) == 1
    assert all(
        keyword.arg != 'default_value'
        for keyword in declarations[0].keywords)


def test_required_links_and_joints_are_present():
    root = _robot_root()
    links = {element.attrib['name'] for element in root.findall('link')}
    joints = {element.attrib['name'] for element in root.findall('joint')}

    assert {
        'base_link',
        'front_left_wheel_link',
        'front_right_wheel_link',
        'rear_left_wheel_link',
        'rear_right_wheel_link',
        'lidar_link',
        'imu_link',
        'camera_link',
        'camera_front_link',
        'camera_front_optical_frame',
        'camera_left_link',
        'camera_right_link',
        'camera_left_optical_frame',
        'camera_right_optical_frame',
    } <= links
    assert {
        'front_left_wheel_joint',
        'front_right_wheel_joint',
        'rear_left_wheel_joint',
        'rear_right_wheel_joint',
    } <= joints

    for name in (
        'front_left_wheel_joint',
        'front_right_wheel_joint',
        'rear_left_wheel_joint',
        'rear_right_wheel_joint',
    ):
        joint = next(
            element for element in root.findall('joint')
            if element.attrib['name'] == name)
        assert joint.find('origin').attrib['xyz'].endswith(' 0.0345')
        # USD axis X rotated +90 degrees around Z is base-frame +Y.
        assert joint.find('axis').attrib['xyz'] == '0 1 0'


def test_description_does_not_claim_navigation_or_truth_frames():
    root = _robot_root()
    links = {element.attrib['name'] for element in root.findall('link')}
    assert links.isdisjoint({'world', 'map', 'odom', 'ground_truth_base_link'})


def test_sensor_and_optical_joints_are_fixed():
    root = _robot_root()
    joints = {element.attrib['name']: element for element in root.findall('joint')}
    expected = {
        'lidar_link_joint',
        'imu_link_joint',
        'camera_link_joint',
        'camera_front_link_joint',
        'camera_front_optical_frame_joint',
        'camera_left_link_joint',
        'camera_right_link_joint',
        'camera_left_optical_frame_joint',
        'camera_right_optical_frame_joint',
    }
    assert all(joints[name].attrib['type'] == 'fixed' for name in expected)

    for name in ('camera_front_optical_frame_joint',
                 'camera_left_optical_frame_joint',
                 'camera_right_optical_frame_joint'):
        origin = joints[name].find('origin')
        assert origin is not None
        assert origin.attrib['rpy'] == '-1.57079632679 0 -1.57079632679'


def test_description_only_mode_does_not_use_a_tf_broadcaster():
    publisher_source = (
        PACKAGE_ROOT / 'scripts' / 'robot_description_publisher.py').read_text()
    assert 'TransformBroadcaster' not in publisher_source
    assert "'/robot_description'" in publisher_source
    launch_source = (
        PACKAGE_ROOT / 'launch' / 'description.launch.py').read_text()
    assert "DeclareLaunchArgument(\n            'publish_tf'" in launch_source
    assert "'xacro_file'" in launch_source
    assert "xacro_file = LaunchConfiguration('xacro_file')" in launch_source


def test_description_publisher_treats_external_shutdown_as_clean_exit():
    publisher_source = (
        PACKAGE_ROOT / 'scripts' / 'robot_description_publisher.py').read_text()
    assert 'except (KeyboardInterrupt, ExternalShutdownException):' \
        in publisher_source
    assert 'except RuntimeError as error:' in publisher_source
    assert "'context is not valid' not in str(error)" in publisher_source
    assert 'raise' in publisher_source
    assert publisher_source.index('node.destroy_node()') \
        < publisher_source.index('rclpy.shutdown()')
