from pathlib import Path

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    backend = LaunchConfiguration(
        'lidar_odometry_backend').perform(context).strip().lower()
    if backend == 'off':
        return [LogInfo(msg='2D LiDAR odometry disabled')]
    if backend != 'rf2o':
        raise RuntimeError('lidar_odometry_backend must be off or rf2o')
    try:
        get_package_prefix('rf2o_laser_odometry')
    except Exception as exc:
        raise RuntimeError(
            'lidar_odometry_backend=rf2o requested, but ROS package '
            'rf2o_laser_odometry is not installed. Install/build a Jazzy '
            'compatible RF2O package before selecting the three-source EKF.'
        ) from exc
    package_share = Path(get_package_share_directory('robot_odometry'))
    requested = LaunchConfiguration(
        'lidar_odometry_params_file').perform(context).strip()
    params_file = (
        Path(requested).expanduser()
        if requested else package_share / 'config' / 'rf2o.yaml'
    )
    if not params_file.is_file():
        raise RuntimeError(
            f'LiDAR odometry params file does not exist: {params_file}')
    return [Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry_node',
        output='screen',
        parameters=[
            str(params_file),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/lidar/odom'),
        ],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'lidar_odometry_backend', default_value='off'),
        DeclareLaunchArgument(
            'lidar_odometry_params_file', default_value=''),
        OpaqueFunction(function=_launch_setup),
    ])
