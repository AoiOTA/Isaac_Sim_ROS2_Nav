from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_setup(context, robot_description):
    publish_tf = LaunchConfiguration('publish_tf').perform(context).lower()
    if publish_tf not in {'true', 'false'}:
        raise RuntimeError('publish_tf must be true or false')
    if publish_tf == 'true':
        return [Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        )]
    return [Node(
        package='robot_description',
        executable='robot_description_publisher',
        name='robot_description_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )]


def generate_launch_description():
    description_share = Path(get_package_share_directory('robot_description'))
    default_xacro_file = description_share / 'urdf' / 'jackal.urdf.xacro'

    prefix = LaunchConfiguration('prefix')
    xacro_file = LaunchConfiguration('xacro_file')
    robot_description = ParameterValue(
        Command(['xacro', ' ', xacro_file, ' prefix:=', prefix]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('prefix', default_value=''),
        DeclareLaunchArgument(
            'xacro_file', default_value=str(default_xacro_file)),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='true',
            description=(
                'Use robot_state_publisher when true; publish only the '
                'description topic when false')),
        OpaqueFunction(
            function=_launch_setup,
            args=[robot_description],
        ),
    ])
