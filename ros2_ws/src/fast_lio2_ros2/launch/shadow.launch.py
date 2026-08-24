"""Launch the optional FAST-LIO2 PointCloud2 odometry shadow."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arguments = (
        ("enabled", "false", "Start the isolated FAST-LIO2 shadow"),
        (
            "planar_imu_enabled",
            "false",
            "Use the LIO-only planar IMU adapter",
        ),
        ("use_sim_time", "true", "Use the ROS simulation clock"),
        ("input_cloud_topic", "/lio/points_raw", "Ouster PointCloud2 input"),
        ("input_imu_topic", "/imu/data", "IMU input"),
        ("output_odom_topic", "/lio/odom_shadow", "Shadow odometry output"),
        ("map_frame", "lio_map_shadow", "Private FAST-LIO local-map frame"),
        ("body_frame", "base_link", "Odometry child/body frame"),
        ("publish_tf", "false", "Publish map-to-body TF (unsafe for default shadow)"),
        ("publish_path", "false", "Publish the optional shadow path"),
        ("publish_registered_cloud", "false", "Publish registered world cloud"),
        ("publish_body_cloud", "false", "Publish undistorted body-frame cloud"),
        ("publish_effect_cloud", "false", "Publish effective feature cloud"),
        ("publish_map", "false", "Publish accumulated local map cloud"),
    )

    declarations = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description in arguments
    ]
    config = PathJoinSubstitution(
        [FindPackageShare("fast_lio2_ros2"), "config", "ouster_shadow.yaml"]
    )
    # The SENSOR-to-IMU axis conversion lives only in ouster_shadow.yaml.
    # Keep launch free of extrinsic or point-coordinate rotation overrides.
    planar_imu_config = PathJoinSubstitution(
        [FindPackageShare("robot_odometry"), "config", "planar_lio_imu.yaml"]
    )
    planar_imu = Node(
        package="robot_odometry",
        executable="planar_lio_imu_adapter",
        name="planar_lio_imu_adapter",
        output="screen",
        condition=IfCondition(LaunchConfiguration("planar_imu_enabled")),
        parameters=[planar_imu_config],
    )
    selected_imu_topic = PythonExpression([
        "'/imu/lio' if '",
        LaunchConfiguration("planar_imu_enabled"),
        "'.lower() == 'true' else '",
        LaunchConfiguration("input_imu_topic"),
        "'",
    ])
    node = Node(
        package="fast_lio2_ros2",
        executable="fastlio_mapping",
        name="fast_lio2_shadow",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enabled")),
        parameters=[
            config,
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "common.lid_topic": LaunchConfiguration("input_cloud_topic"),
                "common.imu_topic": selected_imu_topic,
                "common.odom_topic": LaunchConfiguration("output_odom_topic"),
                "common.map_frame": LaunchConfiguration("map_frame"),
                "common.body_frame": LaunchConfiguration("body_frame"),
                "publish.publish_tf": LaunchConfiguration("publish_tf"),
                "publish.path_en": LaunchConfiguration("publish_path"),
                "publish.scan_publish_en": LaunchConfiguration(
                    "publish_registered_cloud"
                ),
                "publish.scan_bodyframe_pub_en": LaunchConfiguration(
                    "publish_body_cloud"
                ),
                "publish.effect_map_en": LaunchConfiguration(
                    "publish_effect_cloud"
                ),
                "publish.map_en": LaunchConfiguration("publish_map"),
            },
        ],
    )
    return LaunchDescription([*declarations, planar_imu, node])
