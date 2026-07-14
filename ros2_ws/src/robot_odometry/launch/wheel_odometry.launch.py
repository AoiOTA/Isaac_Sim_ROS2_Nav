from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _shutdown_if_wheel_odometry_exited(context):
    """Fail the current Realistic launch when its odometry source exits."""
    if context.is_shutdown:
        return []
    return [EmitEvent(event=Shutdown(
        reason='Wheel odometry exited before the Realistic stack'))]


def generate_launch_description():
    package_share = Path(get_package_share_directory('robot_odometry'))
    default_config = package_share / 'config' / 'wheel_odometry.yaml'
    wheel_odometry = Node(
        package='robot_odometry',
        executable='wheel_odometry_node',
        name='wheel_odometry',
        output='screen',
        parameters=[
            LaunchConfiguration('wheel_odometry_params_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_config_file': LaunchConfiguration(
                    'robot_config_file'),
                'isaac_node_name': LaunchConfiguration(
                    'isaac_node_name'),
                'kinematics_handshake_timeout_sec': LaunchConfiguration(
                    'kinematics_handshake_timeout_sec'),
            },
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'robot_config_file',
            description='Absolute path to the schema v2 robot YAML'),
        DeclareLaunchArgument(
            'isaac_node_name', default_value='/isaac_navigation_sim'),
        DeclareLaunchArgument(
            'kinematics_handshake_timeout_sec', default_value='10.0'),
        DeclareLaunchArgument(
            'wheel_odometry_params_file', default_value=str(default_config)),
        wheel_odometry,
        RegisterEventHandler(OnProcessExit(
            target_action=wheel_odometry,
            on_exit=[OpaqueFunction(
                function=_shutdown_if_wheel_odometry_exited)],
        )),
    ])
