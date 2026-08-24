from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    params_file = (
        Path(get_package_share_directory('robot_odometry'))
        / 'config'
        / 'cuvslam_stereo_imu_shadow.yaml'
    )
    visual_slam = ComposableNode(
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        name='visual_slam_stereo_imu_shadow',
        parameters=[str(params_file)],
        remappings=[
            ('visual_slam/image_0', '/camera/left/image_raw'),
            ('visual_slam/camera_info_0', '/camera/left/camera_info'),
            ('visual_slam/image_1', '/camera/right/image_raw'),
            ('visual_slam/camera_info_1', '/camera/right/camera_info'),
            ('visual_slam/imu', '/imu/vio'),
            ('visual_slam/tracking/odometry', '/visual/odom_shadow'),
            ('visual_slam/status', '/visual/status'),
        ],
    )
    container = ComposableNodeContainer(
        name='visual_odometry_stereo_imu_shadow_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[visual_slam],
        output='screen',
    )
    return LaunchDescription([container])
