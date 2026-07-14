from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def generate_launch_description():
    core_launch = (
        Path(get_package_share_directory('robot_bringup'))
        / 'launch'
        / 'ros_stack.launch.py'
    )
    return LaunchDescription([
        DeclareLaunchArgument('odometry_mode', default_value='ideal'),
        DeclareLaunchArgument('structure_tf_source', default_value='isaac'),
        DeclareLaunchArgument('posegraph_file', default_value=''),
        DeclareLaunchArgument('map_manifest_file', default_value=''),
        DeclareLaunchArgument('ceres_num_threads', default_value='12'),
        DeclareLaunchArgument('robot_description_file', default_value=''),
        DeclareLaunchArgument('robot_config_file', default_value=''),
        DeclareLaunchArgument(
            'isaac_node_name', default_value='/isaac_navigation_sim'),
        DeclareLaunchArgument(
            'kinematics_handshake_timeout_sec', default_value='10.0'),
        DeclareLaunchArgument(
            'wheel_odometry_params_file', default_value=''),
        DeclareLaunchArgument('nav2_params_file', default_value=''),
        DeclareLaunchArgument(
            'spawn_poses_file',
            default_value=EnvironmentVariable(
                'ISAAC_NAV_SPAWN_POSES', default_value='')),
        DeclareLaunchArgument(
            'spawn_pose_name', default_value='mapping_start'),
        DeclareLaunchArgument('initial_pose_source', default_value='auto'),
        DeclareLaunchArgument('interactive', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value='auto'),
        DeclareLaunchArgument('use_teleop', default_value='auto'),
        DeclareLaunchArgument('teleop_linear_speed', default_value='0.50'),
        DeclareLaunchArgument('teleop_angular_speed', default_value='0.80'),
        DeclareLaunchArgument(
            'teleop_linear_speed_step', default_value='0.05'),
        DeclareLaunchArgument(
            'teleop_angular_speed_step', default_value='0.10'),
        DeclareLaunchArgument(
            'teleop_min_linear_speed', default_value='0.10'),
        DeclareLaunchArgument(
            'teleop_min_angular_speed', default_value='0.20'),
        DeclareLaunchArgument(
            'teleop_max_linear_speed', default_value='1.00'),
        DeclareLaunchArgument(
            'teleop_max_angular_speed', default_value='1.50'),
        DeclareLaunchArgument(
            'project_root',
            default_value=EnvironmentVariable(
                'PROJECT_ROOT', default_value='')),
        DeclareLaunchArgument('use_self_filter', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(core_launch)),
            launch_arguments={
                'operation': 'incremental_mapping',
                'odometry_mode': LaunchConfiguration('odometry_mode'),
                'structure_tf_source': LaunchConfiguration(
                    'structure_tf_source'),
                'posegraph_file': LaunchConfiguration('posegraph_file'),
                'map_manifest_file': LaunchConfiguration(
                    'map_manifest_file'),
                'ceres_num_threads': LaunchConfiguration(
                    'ceres_num_threads'),
                'robot_description_file': LaunchConfiguration(
                    'robot_description_file'),
                'robot_config_file': LaunchConfiguration(
                    'robot_config_file'),
                'isaac_node_name': LaunchConfiguration('isaac_node_name'),
                'kinematics_handshake_timeout_sec': LaunchConfiguration(
                    'kinematics_handshake_timeout_sec'),
                'wheel_odometry_params_file': LaunchConfiguration(
                    'wheel_odometry_params_file'),
                'nav2_params_file': LaunchConfiguration('nav2_params_file'),
                'spawn_poses_file': LaunchConfiguration('spawn_poses_file'),
                'spawn_pose_name': LaunchConfiguration('spawn_pose_name'),
                'initial_pose_source': LaunchConfiguration(
                    'initial_pose_source'),
                'interactive': LaunchConfiguration('interactive'),
                'use_rviz': LaunchConfiguration('use_rviz'),
                'rviz_config': LaunchConfiguration('rviz_config'),
                'use_teleop': LaunchConfiguration('use_teleop'),
                'teleop_linear_speed': LaunchConfiguration(
                    'teleop_linear_speed'),
                'teleop_angular_speed': LaunchConfiguration(
                    'teleop_angular_speed'),
                'teleop_linear_speed_step': LaunchConfiguration(
                    'teleop_linear_speed_step'),
                'teleop_angular_speed_step': LaunchConfiguration(
                    'teleop_angular_speed_step'),
                'teleop_min_linear_speed': LaunchConfiguration(
                    'teleop_min_linear_speed'),
                'teleop_min_angular_speed': LaunchConfiguration(
                    'teleop_min_angular_speed'),
                'teleop_max_linear_speed': LaunchConfiguration(
                    'teleop_max_linear_speed'),
                'teleop_max_angular_speed': LaunchConfiguration(
                    'teleop_max_angular_speed'),
                'project_root': LaunchConfiguration('project_root'),
                'use_self_filter': LaunchConfiguration('use_self_filter'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),
    ])
