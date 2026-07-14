from pathlib import Path
from threading import Thread
import uuid

import pytest
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from robot_odometry.robot_profile import ISAAC_KINEMATICS_PARAMETER_NAMES
from robot_odometry.robot_profile import load_robot_profile
from robot_odometry.wheel_odometry_node import main
from robot_odometry.wheel_odometry_node import WheelOdometryNode


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
ROBOT_CONFIG = (
    REPOSITORY_ROOT / 'isaac_sim' / 'configs' / 'robots' / 'jackal.yaml')


def _runtime_parameters(profile):
    return {
        'runtime_provenance.schema_version': 5,
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


def _start_parameter_server(name, parameters):
    node = Node(name, use_global_arguments=False)
    read_only = ParameterDescriptor(read_only=True)
    assert set(parameters) == set(ISAAC_KINEMATICS_PARAMETER_NAMES)
    for parameter_name, value in parameters.items():
        node.declare_parameter(parameter_name, value, read_only)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = Thread(target=executor.spin, daemon=True)
    thread.start()
    return node, executor, thread


def _init_context(robot_config, isaac_node_name, timeout_sec=2.0):
    rclpy.init(args=[
        '--ros-args',
        '-p', f'robot_config_file:={robot_config}',
        '-p', f'isaac_node_name:={isaac_node_name}',
        '-p', f'kinematics_handshake_timeout_sec:={timeout_sec}',
    ])


def _shutdown(node, server, executor, thread):
    if node is not None:
        node.destroy_node()
    executor.shutdown()
    thread.join(timeout=2.0)
    server.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def test_matching_read_only_isaac_contract_activates_wheel_odometry():
    profile = load_robot_profile(ROBOT_CONFIG)
    remote_name = server = executor = thread = None
    wheel_node = None
    try:
        server_name = f'test_isaac_{uuid.uuid4().hex}'
        remote_name = f'/{server_name}'
        _init_context(ROBOT_CONFIG, remote_name)
        server, executor, thread = _start_parameter_server(
            server_name, _runtime_parameters(profile))
        wheel_node = WheelOdometryNode()

        assert wheel_node._integrator.config.wheel_radius \
            == profile.wheel_radius_m
        assert wheel_node._integrator.config.track_width \
            == profile.effective_track_width_m
        assert wheel_node.count_publishers('/wheel/odom') == 1
    finally:
        if server is not None:
            _shutdown(wheel_node, server, executor, thread)


def test_sha_mismatch_creates_no_runtime_endpoint(monkeypatch):
    profile = load_robot_profile(ROBOT_CONFIG)
    parameters = _runtime_parameters(profile)
    parameters['runtime_provenance.robot.config.sha256'] = '0' * 64
    remote_name = server = executor = thread = None

    create_publisher = WheelOdometryNode.create_publisher

    def guarded_publisher(node, message_type, topic, *args, **kwargs):
        if topic == '/wheel/odom':
            pytest.fail(
                'wheel odometry publisher was created before kinematics '
                'matched')
        return create_publisher(
            node, message_type, topic, *args, **kwargs)

    def forbidden_endpoint(*args, **kwargs):
        del args, kwargs
        pytest.fail('runtime endpoint was created before kinematics matched')

    monkeypatch.setattr(
        WheelOdometryNode, 'create_publisher', guarded_publisher)
    for method_name in (
            'create_subscription', 'create_service', 'create_timer'):
        monkeypatch.setattr(
            WheelOdometryNode, method_name, forbidden_endpoint)

    try:
        server_name = f'test_isaac_{uuid.uuid4().hex}'
        remote_name = f'/{server_name}'
        _init_context(ROBOT_CONFIG, remote_name)
        server, executor, thread = _start_parameter_server(
            server_name, parameters)
        with pytest.raises(ValueError, match='config.sha256'):
            WheelOdometryNode()
    finally:
        if server is not None:
            _shutdown(None, server, executor, thread)


def test_missing_isaac_service_times_out_and_main_cleans_rclpy():
    missing_node = f'/missing_isaac_{uuid.uuid4().hex}'
    with pytest.raises(TimeoutError, match='services are unavailable'):
        main(args=[
            '--ros-args',
            '-p', f'robot_config_file:={ROBOT_CONFIG}',
            '-p', f'isaac_node_name:={missing_node}',
            '-p', 'kinematics_handshake_timeout_sec:=0.05',
        ])
    assert not rclpy.ok()
