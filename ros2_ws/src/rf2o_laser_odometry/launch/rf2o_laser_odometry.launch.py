from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry_node',
            output='screen',
            parameters=[{
                'laser_scan_topic': '/scan',
                'odom_topic': '/lidar/odom',
                'publish_tf': False,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'init_pose_from_topic': '',
                'freq': 20.0,
                'pose_covariance_diagonal': [
                    0.05, 0.05, 1.0e6, 1.0e6, 1.0e6, 0.10],
                'twist_covariance_diagonal': [
                    0.10, 0.10, 1.0e6, 1.0e6, 1.0e6, 0.20],
            }],
        ),
    ])
