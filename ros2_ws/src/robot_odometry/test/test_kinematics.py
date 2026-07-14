import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from robot_odometry.kinematics import covariance_from_diagonal
from robot_odometry.kinematics import WheelOdometry
from robot_odometry.kinematics import WheelOdometryConfig
from robot_odometry.robot_profile import load_robot_profile
import yaml


NAMES = [
    'front_left_wheel_joint',
    'front_right_wheel_joint',
    'rear_left_wheel_joint',
    'rear_right_wheel_joint',
]
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[2]
EXPERIMENTAL_ROBOT_DIR = (
    REPOSITORY_ROOT / 'isaac_sim' / 'configs' / 'robots' / 'experimental')


def test_integrator_requires_an_explicit_robot_kinematics_profile():
    with pytest.raises(TypeError):
        WheelOdometryConfig(max_integration_step=0.25)


@pytest.mark.parametrize(
    ('filename', 'profile_id', 'effective_track_width'),
    (
        ('jackal_etw_0p989_v1.yaml', 'jackal_etw_0p989_v1', 0.989),
        ('jackal_etw_1p012_v1.yaml', 'jackal_etw_1p012_v1', 1.012),
    ),
)
def test_experimental_effective_track_profiles_are_valid_wheel_odom_inputs(
        filename, profile_id, effective_track_width):
    profile = load_robot_profile(EXPERIMENTAL_ROBOT_DIR / filename)

    assert profile.profile_id == profile_id
    assert profile.lifecycle == 'experimental_candidate'
    assert profile.wheel_radius_m == pytest.approx(0.098)
    assert profile.geometric_track_width_m == pytest.approx(0.37559)
    assert profile.effective_track_width_m == pytest.approx(
        effective_track_width)
    assert profile.left_joint_names == (
        'front_left_wheel_joint', 'rear_left_wheel_joint')
    assert profile.right_joint_names == (
        'front_right_wheel_joint', 'rear_right_wheel_joint')


def _integrator(max_step=0.25):
    return WheelOdometry(WheelOdometryConfig(
        wheel_radius=0.098,
        track_width=0.37559,
        left_joint_names=(
            'front_left_wheel_joint', 'rear_left_wheel_joint'),
        right_joint_names=(
            'front_right_wheel_joint', 'rear_right_wheel_joint'),
        max_integration_step=max_step,
    ))


def test_straight_motion_uses_both_wheels_on_each_side():
    odometry = _integrator()
    odometry.update(NAMES, [1.0, 1.0, 3.0, 3.0], 1.0)
    result = odometry.update(NAMES, [1.0, 1.0, 3.0, 3.0], 1.1)

    assert result.accepted
    assert result.sample.linear_velocity == pytest.approx(0.196)
    assert result.sample.angular_velocity == pytest.approx(0.0)
    assert result.sample.x == pytest.approx(0.0196)
    assert result.sample.y == pytest.approx(0.0)


def test_opposed_wheel_velocities_rotate_in_place():
    odometry = _integrator()
    velocities = [-1.0, 1.0, -1.0, 1.0]
    odometry.update(NAMES, velocities, 5.0)
    result = odometry.update(NAMES, velocities, 5.1)

    expected_wz = 2.0 * 0.098 / 0.37559
    assert result.sample.linear_velocity == pytest.approx(0.0)
    assert result.sample.angular_velocity == pytest.approx(expected_wz)
    assert result.sample.yaw == pytest.approx(expected_wz * 0.1)


def test_missing_joint_consumes_time_but_never_integrates_gap():
    odometry = _integrator()
    odometry.update(NAMES, [1.0] * 4, 1.0)
    missing = odometry.update(NAMES[:-1], [1.0] * 3, 1.1)
    recovered = odometry.update(NAMES, [1.0] * 4, 1.2)

    assert not missing.accepted
    assert missing.reason == 'missing_required_joint'
    assert recovered.sample.dt == pytest.approx(0.1)
    assert recovered.sample.x == pytest.approx(0.0098)


def test_time_regression_resets_pose_safely():
    odometry = _integrator()
    odometry.update(NAMES, [1.0] * 4, 10.0)
    odometry.update(NAMES, [1.0] * 4, 10.1)
    assert odometry.pose[0] > 0.0

    regressed = odometry.update(NAMES, [1.0] * 4, 2.0)
    assert not regressed.accepted
    assert regressed.reason == 'time_regression_reset'
    assert odometry.pose == pytest.approx((0.0, 0.0, 0.0))

    resumed = odometry.update(NAMES, [1.0] * 4, 2.1)
    assert resumed.accepted
    assert resumed.sample.x == pytest.approx(0.0098)


def test_duplicate_and_large_steps_are_skipped():
    odometry = _integrator(max_step=0.2)
    odometry.update(NAMES, [1.0] * 4, 1.0)
    assert odometry.update(NAMES, [1.0] * 4, 1.0).reason == 'duplicate_stamp'
    gap = odometry.update(NAMES, [1.0] * 4, 2.0)
    assert gap.reason == 'integration_gap_skipped'
    assert odometry.pose == pytest.approx((0.0, 0.0, 0.0))


def test_explicit_reset_clears_pose_and_timestamp():
    odometry = _integrator()
    odometry.update(NAMES, [1.0] * 4, 1.0)
    odometry.update(NAMES, [1.0] * 4, 1.1)
    odometry.reset()
    assert odometry.pose == pytest.approx((0.0, 0.0, 0.0))
    assert odometry.last_stamp_s is None


def test_covariance_diagonal_expansion_and_validation():
    covariance = covariance_from_diagonal([1, 2, 3, 4, 5, 6])
    assert [covariance[index * 6 + index] for index in range(6)] \
        == [1, 2, 3, 4, 5, 6]
    assert math.fsum(covariance) == 21
    with pytest.raises(ValueError):
        covariance_from_diagonal([1, 2])


def test_ros_adapter_relies_on_rclpy_builtin_sim_time_parameter():
    source = (
        __file__.replace('test/test_kinematics.py', '')
        + 'robot_odometry/wheel_odometry_node.py'
    )
    with open(source, encoding='utf-8') as source_file:
        assert "declare_parameter('use_sim_time'" not in source_file.read()


def test_ros_adapter_treats_external_shutdown_as_clean_exit():
    source = (
        __file__.replace('test/test_kinematics.py', '')
        + 'robot_odometry/wheel_odometry_node.py'
    )
    with open(source, encoding='utf-8') as source_file:
        source_text = source_file.read()
    assert 'except (KeyboardInterrupt, ExternalShutdownException):' \
        in source_text
    assert source_text.index('node.destroy_node()') \
        < source_text.index('rclpy.shutdown()')


def test_runtime_yaml_contains_no_duplicate_robot_kinematics():
    parameters = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'wheel_odometry.yaml').read_text(
            encoding='utf-8'))['wheel_odometry']['ros__parameters']
    assert not {
        'wheel_radius',
        'track_width',
        'left_joint_names',
        'right_joint_names',
    } & set(parameters)


def test_ros_adapter_gates_all_runtime_endpoints_on_isaac_identity():
    source_text = (
        PACKAGE_ROOT / 'robot_odometry' / 'wheel_odometry_node.py'
    ).read_text(encoding='utf-8')
    constructor = source_text.split('    def __init__(self):', 1)[1].split(
        '    def _', 1)[0]

    assert 'start_parameter_services=False' in constructor
    assert 'enable_rosout=False' in constructor
    assert "declare_parameter('wheel_radius'" not in source_text
    assert "declare_parameter('track_width'" not in source_text
    assert "declare_parameter('left_joint_names'" not in source_text
    assert "declare_parameter('right_joint_names'" not in source_text
    assert constructor.index('load_robot_profile(') \
        < constructor.index('self._establish_kinematics_contract(') \
        < constructor.index('self._activate_runtime(')
    gated = source_text.split('    def _activate_runtime(', 1)[1]
    for endpoint in (
            'create_subscription(', 'create_publisher(', 'create_service(',
            'create_timer('):
        assert endpoint in gated
        assert endpoint not in constructor
    assert 'AsyncParameterClient' in source_text
    assert 'ISAAC_KINEMATICS_PARAMETER_NAMES' in source_text
    assert 'parameter_value_to_python' in source_text
    assert 'validate_isaac_kinematics' in source_text


def test_main_cleans_rclpy_when_node_construction_fails():
    source_text = (
        PACKAGE_ROOT / 'robot_odometry' / 'wheel_odometry_node.py'
    ).read_text(encoding='utf-8')
    main_source = source_text.split('def main(args=None):', 1)[1]
    assert 'node = None' in main_source
    assert main_source.index('node = None') < main_source.index('try:')
    assert 'if node is not None:' in main_source
    assert main_source.index('if node is not None:') \
        < main_source.index('if rclpy.ok():') \
        < main_source.index('rclpy.shutdown()')


def test_odometry_launch_forwards_the_explicit_handshake_inputs():
    source_text = (
        PACKAGE_ROOT / 'launch' / 'wheel_odometry.launch.py'
    ).read_text(encoding='utf-8')
    for argument in (
            'robot_config_file', 'isaac_node_name',
            'kinematics_handshake_timeout_sec'):
        assert f"'{argument}'" in source_text
        assert f"'{argument}': LaunchConfiguration(" in source_text


def test_odometry_launch_is_fail_closed_when_the_node_exits():
    source_text = (
        PACKAGE_ROOT / 'launch' / 'wheel_odometry.launch.py'
    ).read_text(encoding='utf-8')

    assert 'RegisterEventHandler(OnProcessExit(' in source_text
    assert 'target_action=wheel_odometry' in source_text
    assert 'function=_shutdown_if_wheel_odometry_exited' in source_text
    assert 'if context.is_shutdown:' in source_text
    assert "Shutdown(\n        reason='Wheel odometry exited" in source_text


def test_package_declares_profile_and_handshake_runtime_dependencies():
    root = ET.parse(PACKAGE_ROOT / 'package.xml').getroot()
    dependencies = {
        element.text for element in root.findall('exec_depend')
    }
    assert {'python3-yaml', 'rcl_interfaces', 'std_msgs'} <= dependencies
