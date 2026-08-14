"""Isolated Ideal-odometry navigation for a generated physical benchmark."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(package: str, launch_file: str, arguments: dict[str, str]):
    source = (
        Path(get_package_share_directory(package)) / "launch" / launch_file
    )
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(source)),
        launch_arguments=arguments.items(),
    )


def _setup(context):
    required = {}
    for name in ("map_file", "route_graph_file", "spawn_poses_file"):
        value = Path(LaunchConfiguration(name).perform(context)).expanduser().resolve()
        if not value.is_file():
            raise RuntimeError(f"{name} does not exist: {value}")
        required[name] = value
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    return [
        _include(
            "robot_description",
            "description.launch.py",
            {"use_sim_time": use_sim_time, "publish_tf": "false"},
        ),
        _include(
            "robot_perception",
            "lidar_processing.launch.py",
            {
                "use_sim_time": use_sim_time,
                "use_self_filter": "true",
                "enable_safety_scan": "true",
            },
        ),
        _include(
            "robot_mapping",
            "localization.launch.py",
            {
                "use_sim_time": use_sim_time,
                "autostart": "true",
                "use_posegraph_localization": "false",
                "map_file": str(required["map_file"]),
                "map_to_odom_x": LaunchConfiguration("map_to_odom_x").perform(context),
                "map_to_odom_y": LaunchConfiguration("map_to_odom_y").perform(context),
                "map_to_odom_yaw_deg": LaunchConfiguration("map_to_odom_yaw_deg").perform(context),
            },
        ),
        _include(
            "robot_experiments",
            "initial_pose.launch.py",
            {
                "spawn_poses_file": str(required["spawn_poses_file"]),
                "spawn_pose_name": LaunchConfiguration("spawn_pose_name").perform(context),
                "wait_for_odom_to_base_tf": "true",
                "stay_alive_for_reseed": "true",
            },
        ),
        _include(
            "robot_navigation",
            "navigation.launch.py",
            {
                "use_sim_time": use_sim_time,
                "autostart": "true",
                "nav2_profile_params_file": LaunchConfiguration(
                    "nav2_profile_params_file"
                ).perform(context),
                "structural_map_file": str(required["map_file"]),
                "route_graph_file": str(required["route_graph_file"]),
                "feasible_only_largest_component": "true",
                "module2_enabled": LaunchConfiguration("module2_enabled").perform(context),
                "execute_route_navigation": "true",
                "voxel_grid_topic": "stvl_voxel_grid",
                "region_config_file": LaunchConfiguration(
                    "region_config_file"
                ).perform(context),
                "region_switch_min_dwell_s": LaunchConfiguration(
                    "region_switch_min_dwell_s"
                ).perform(context),
            },
        ),
    ]


def generate_launch_description():
    navigation_share = Path(get_package_share_directory("robot_navigation"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("map_file"),
            DeclareLaunchArgument("route_graph_file"),
            DeclareLaunchArgument("spawn_poses_file"),
            DeclareLaunchArgument("spawn_pose_name"),
            DeclareLaunchArgument("map_to_odom_x"),
            DeclareLaunchArgument("map_to_odom_y"),
            DeclareLaunchArgument("map_to_odom_yaw_deg"),
            DeclareLaunchArgument("module2_enabled", default_value="false"),
            DeclareLaunchArgument("region_config_file", default_value=""),
            DeclareLaunchArgument(
                "region_switch_min_dwell_s", default_value="0.5"
            ),
            DeclareLaunchArgument(
                "nav2_profile_params_file",
                default_value=str(
                    navigation_share / "config" / "nav2_bio_nav_planning_only.yaml"
                ),
            ),
            OpaqueFunction(function=_setup),
        ]
    )
