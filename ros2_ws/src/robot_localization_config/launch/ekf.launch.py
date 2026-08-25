from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from robot_localization_config.ekf_input_policy import validate_lidar_gate


def _launch_setup(context):
    package_share = Path(
        get_package_share_directory('robot_localization_config'))
    profile = LaunchConfiguration('ekf_profile').perform(context).strip()
    if profile not in {
            'wheel_imu', 'wheel_imu_lidar', 'module1_wheel_imu'}:
        raise RuntimeError(
            'ekf_profile must be wheel_imu, wheel_imu_lidar, or '
            'module1_wheel_imu')
    lidar_validated_value = LaunchConfiguration(
        'lidar_odometry_validated').perform(context).strip().lower()
    if lidar_validated_value not in {'true', 'false'}:
        raise RuntimeError('lidar_odometry_validated must be true or false')
    requested = LaunchConfiguration('ekf_params_file').perform(context).strip()
    params_file = (
        Path(requested).expanduser()
        if requested
        else package_share / 'config' / f'ekf_{profile}.yaml'
    )
    if not params_file.is_file():
        raise RuntimeError(f'EKF params file does not exist: {params_file}')
    try:
        validate_lidar_gate(
            params_file, lidar_validated_value == 'true')
    except ValueError as exc:
        raise RuntimeError(f'invalid EKF input policy: {exc}') from exc
    output_topic = (
        '/bio_nav/module1/odom'
        if profile == 'module1_wheel_imu'
        else '/odom'
    )
    return [Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            str(params_file),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[('odometry/filtered', output_topic)],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('ekf_profile', default_value='wheel_imu'),
        DeclareLaunchArgument(
            'lidar_odometry_validated', default_value='false'),
        DeclareLaunchArgument(
            'ekf_params_file', default_value='',
            description='Optional explicit EKF YAML; profile selects default'),
        OpaqueFunction(function=_launch_setup),
    ])
