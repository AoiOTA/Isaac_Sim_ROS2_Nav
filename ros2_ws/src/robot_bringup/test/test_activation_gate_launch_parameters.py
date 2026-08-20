import importlib.util
from pathlib import Path

import pytest
import rclpy
from rclpy.node import Node


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_ROOT / 'launch' / 'ros_stack.launch.py'
DEFAULT_PARAMETERS = PACKAGE_ROOT / 'config' / 'activation_gate.yaml'


def _load_launch_module():
    spec = importlib.util.spec_from_file_location(
        'robot_bringup_ros_stack_launch', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class _ActivationGateParameterProbe(Node):
    def __init__(self):
        super().__init__('nav2_activation_gate')
        self.declare_parameter('initial_pose_source', 'auto')
        self.declare_parameter('startup_timeout', 30.0)
        self.declare_parameter('startup_timeout_policy', 'fail_closed')


@pytest.mark.parametrize(
    ('use_sim_time', 'initial_pose_source', 'startup_timeout',
     'startup_timeout_policy'),
    [
        ('false', 'auto', '41.0', 'fail_closed'),
        ('false', 'rviz', '7.5', 'wait_for_seed'),
    ],
)
def test_exact_node_runtime_overlay_wins_during_rclpy_parameter_parsing(
        use_sim_time, initial_pose_source, startup_timeout,
        startup_timeout_policy):
    launch_module = _load_launch_module()
    overlay = launch_module._write_activation_gate_runtime_overlay(
        use_sim_time=use_sim_time,
        initial_pose_source=initial_pose_source,
        startup_timeout=startup_timeout,
        startup_timeout_policy=startup_timeout_policy,
    )
    node = None
    try:
        rclpy.init(args=[
            '--ros-args',
            '--params-file', str(DEFAULT_PARAMETERS),
            '--params-file', str(overlay),
        ])
        node = _ActivationGateParameterProbe()
        assert node.get_parameter('use_sim_time').value is (
            use_sim_time == 'true')
        assert node.get_parameter('initial_pose_source').value == \
            initial_pose_source
        assert node.get_parameter('startup_timeout').value == \
            float(startup_timeout)
        assert node.get_parameter('startup_timeout_policy').value == \
            startup_timeout_policy
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        overlay.unlink(missing_ok=True)
