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
    default_route_graph = (
        Path(get_package_share_directory('robot_route_planner'))
        / 'config'
        / 'warehouse_new_gvg_v1.geojson'
    )
    return LaunchDescription([
        DeclareLaunchArgument('odometry_mode', default_value='ideal'),
        DeclareLaunchArgument('structure_tf_source', default_value='isaac'),
        DeclareLaunchArgument('posegraph_file', default_value=''),
        DeclareLaunchArgument('ceres_num_threads', default_value='12'),
        DeclareLaunchArgument('map_file', default_value=''),
        DeclareLaunchArgument('map_manifest_file', default_value=''),
        DeclareLaunchArgument('robot_description_file', default_value=''),
        DeclareLaunchArgument(
            'wheel_odometry_params_file', default_value=''),
        DeclareLaunchArgument('ekf_profile', default_value='wheel_imu'),
        DeclareLaunchArgument(
            'lidar_odometry_validated', default_value='false'),
        DeclareLaunchArgument('ekf_params_file', default_value=''),
        DeclareLaunchArgument(
            'lidar_odometry_backend', default_value='off'),
        DeclareLaunchArgument(
            'lidar_odometry_params_file', default_value=''),
        DeclareLaunchArgument(
            'localization_profile', default_value='kujiale'),
        DeclareLaunchArgument('amcl_params_file', default_value=''),
        DeclareLaunchArgument('nav2_params_file', default_value=''),
        DeclareLaunchArgument('nav2_profile', default_value='stable'),
        DeclareLaunchArgument('nav2_profile_params_file', default_value=''),
        DeclareLaunchArgument('cognitive_profile', default_value='M0'),
        DeclareLaunchArgument('module2_enabled', default_value='true'),
        DeclareLaunchArgument(
            'route_graph_file', default_value=str(default_route_graph)),
        DeclareLaunchArgument(
            'feasible_only_largest_component', default_value='false'),
        DeclareLaunchArgument('module2_prior_ttl_s', default_value='2.0'),
        DeclareLaunchArgument(
            'cognitive_graph_mode', default_value='gvg'),
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
        DeclareLaunchArgument(
            'project_root',
            default_value=EnvironmentVariable(
                'PROJECT_ROOT', default_value='')),
        DeclareLaunchArgument('use_self_filter', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(core_launch)),
            launch_arguments={
                'operation': 'navigation',
                'odometry_mode': LaunchConfiguration('odometry_mode'),
                'structure_tf_source': LaunchConfiguration(
                    'structure_tf_source'),
                'posegraph_file': LaunchConfiguration('posegraph_file'),
                'ceres_num_threads': LaunchConfiguration(
                    'ceres_num_threads'),
                'map_file': LaunchConfiguration('map_file'),
                'map_manifest_file': LaunchConfiguration(
                    'map_manifest_file'),
                'robot_description_file': LaunchConfiguration(
                    'robot_description_file'),
                'wheel_odometry_params_file': LaunchConfiguration(
                    'wheel_odometry_params_file'),
                'ekf_profile': LaunchConfiguration('ekf_profile'),
                'lidar_odometry_validated': LaunchConfiguration(
                    'lidar_odometry_validated'),
                'ekf_params_file': LaunchConfiguration('ekf_params_file'),
                'lidar_odometry_backend': LaunchConfiguration(
                    'lidar_odometry_backend'),
                'lidar_odometry_params_file': LaunchConfiguration(
                    'lidar_odometry_params_file'),
                'localization_profile': LaunchConfiguration(
                    'localization_profile'),
                'amcl_params_file': LaunchConfiguration('amcl_params_file'),
                'nav2_params_file': LaunchConfiguration('nav2_params_file'),
                'nav2_profile': LaunchConfiguration('nav2_profile'),
                'nav2_profile_params_file': LaunchConfiguration(
                    'nav2_profile_params_file'),
                'cognitive_profile': LaunchConfiguration(
                    'cognitive_profile'),
                'module2_enabled': LaunchConfiguration('module2_enabled'),
                'route_graph_file': LaunchConfiguration('route_graph_file'),
                'feasible_only_largest_component': LaunchConfiguration(
                    'feasible_only_largest_component'),
                'module2_prior_ttl_s': LaunchConfiguration(
                    'module2_prior_ttl_s'),
                'cognitive_graph_mode': LaunchConfiguration(
                    'cognitive_graph_mode'),
                'spawn_poses_file': LaunchConfiguration('spawn_poses_file'),
                'spawn_pose_name': LaunchConfiguration('spawn_pose_name'),
                'initial_pose_source': LaunchConfiguration(
                    'initial_pose_source'),
                'interactive': LaunchConfiguration('interactive'),
                'use_rviz': LaunchConfiguration('use_rviz'),
                'rviz_config': LaunchConfiguration('rviz_config'),
                'use_teleop': LaunchConfiguration('use_teleop'),
                'project_root': LaunchConfiguration('project_root'),
                'use_self_filter': LaunchConfiguration('use_self_filter'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),
    ])
