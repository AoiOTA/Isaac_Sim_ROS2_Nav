from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    package_share = Path(get_package_share_directory('robot_navigation'))
    default_config = package_share / 'config' / 'nav2_params.yaml'
    default_profile = package_share / 'config' / 'nav2_stable.yaml'
    default_nav_to_pose_bt = package_share / 'behavior_trees' / (
        'navigate_to_pose_with_dead_end_recovery.xml')
    default_nav_through_poses_bt = package_share / 'behavior_trees' / (
        'navigate_through_poses_with_dead_end_recovery.xml')
    params_file = LaunchConfiguration('nav2_params_file')
    profile_params_file = LaunchConfiguration('nav2_profile_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'velocity_smoother',
        'collision_monitor',
        'bt_navigator',
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='false'),
        DeclareLaunchArgument(
            'nav2_params_file', default_value=str(default_config)),
        DeclareLaunchArgument(
            'nav2_profile_params_file', default_value=str(default_profile)),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        SetParameter('use_sim_time', use_sim_time),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            sigterm_timeout='15.0',
            parameters=[params_file, profile_params_file],
            remappings=[('cmd_vel', '/cmd_vel_nav')],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            sigterm_timeout='15.0',
            parameters=[params_file, profile_params_file],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            sigterm_timeout='15.0',
            parameters=[params_file, profile_params_file],
            remappings=[('cmd_vel', '/cmd_vel_nav')],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[
                params_file,
                {
                    'default_nav_to_pose_bt_xml': str(
                        default_nav_to_pose_bt),
                    'default_nav_through_poses_bt_xml': str(
                        default_nav_through_poses_bt),
                },
            ],
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            sigterm_timeout='15.0',
            parameters=[params_file, profile_params_file],
            remappings=[('cmd_vel', '/cmd_vel_nav')],
        ),
        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            sigterm_timeout='15.0',
            parameters=[params_file, profile_params_file],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            sigterm_timeout='15.0',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': autostart},
                {'node_names': lifecycle_nodes},
                # A simulation reset deliberately pauses every managed node
                # while TF and costmaps are re-seeded.  On a loaded Isaac
                # process that transition can take longer than Nav2's
                # default four-second bond timeout, which otherwise causes a
                # freshly resumed controller to be declared dead before its
                # first heartbeat.  Keep the timeout bounded, but long enough
                # to cover the documented reset transaction.
                {'bond_timeout': 10.0},
            ],
        ),
    ])
