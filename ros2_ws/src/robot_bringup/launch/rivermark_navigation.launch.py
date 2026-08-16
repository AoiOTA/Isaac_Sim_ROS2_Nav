"""Rivermark Nav2 stack with selectable ideal or estimated localization.

The default ideal odometry mode reproduces the qualified Attempt31 demo
(Isaac ideal odometry plus ideal_localization_tf).  The realistic mode runs
wheel odometry and the EKF instead, and localization_backend=amcl lets AMCL
own the map->odom transform for the Attempt31 Rivermark demo.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _include(package: str, launch_file: str, arguments: dict[str, str]):
    source = Path(get_package_share_directory(package)) / "launch" / launch_file
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(source)),
        launch_arguments=arguments.items(),
    )


def _setup(context):
    files = {}
    for name in (
        "map_file",
        "route_graph_file",
        "region_config_file",
        "waypoint_config_file",
    ):
        value = Path(LaunchConfiguration(name).perform(context)).expanduser().resolve()
        if not value.is_file():
            raise RuntimeError(f"{name} does not exist: {value}")
        files[name] = value
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    odometry_mode = (
        LaunchConfiguration("odometry_mode").perform(context).strip().lower()
    )
    if odometry_mode not in {"ideal", "realistic"}:
        raise RuntimeError("odometry_mode must be ideal or realistic")
    localization_backend = (
        LaunchConfiguration("localization_backend")
        .perform(context)
        .strip()
        .lower()
    )
    if localization_backend not in {"ideal", "amcl", "slam_toolbox"}:
        raise RuntimeError(
            "localization_backend must be ideal, amcl, or slam_toolbox"
        )
    if odometry_mode == "ideal" and localization_backend != "ideal":
        raise RuntimeError(
            f"localization_backend={localization_backend} requires realistic "
            "odometry; ideal odometry owns map->odom through "
            "ideal_localization_tf"
        )
    description_share = Path(get_package_share_directory("robot_description"))
    actions = [
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
    ]
    if odometry_mode == "realistic":
        # Estimated odometry chain: wheel velocities + IMU yaw rate fused by
        # the EKF into /odom and the odom->base_link TF.
        actions.extend(
            [
                _include(
                    "robot_odometry",
                    "wheel_odometry.launch.py",
                    {
                        "use_sim_time": use_sim_time,
                        "wheel_odometry_params_file": LaunchConfiguration(
                            "wheel_odometry_params_file"
                        ).perform(context),
                    },
                ),
                _include(
                    "robot_localization_config",
                    "ekf.launch.py",
                    {
                        "use_sim_time": use_sim_time,
                        "ekf_params_file": LaunchConfiguration(
                            "ekf_params_file"
                        ).perform(context),
                    },
                ),
            ]
        )
    actions.extend(
        [
        _include(
            "robot_mapping",
            "localization.launch.py",
            {
                "use_sim_time": use_sim_time,
                "autostart": "true",
                # The backend selects the map->odom owner; with amcl the
                # ideal_localization_tf node is not started.
                "localization_backend": localization_backend,
                "amcl_params_file": LaunchConfiguration(
                    "amcl_params_file"
                ).perform(context),
                "map_file": str(files["map_file"]),
                # Isaac's ideal odometry is reset to zero at the selected
                # spawn. Align that local origin with the Rivermark map pose.
                "map_to_odom_x": LaunchConfiguration("start_x").perform(context),
                "map_to_odom_y": LaunchConfiguration("start_y").perform(context),
                "map_to_odom_yaw_deg": LaunchConfiguration(
                    "start_yaw_deg"
                ).perform(context),
            },
        ),
        # Preserve the same reset/localization ownership contract as the
        # qualified stack.  Isaac waits for this durable policy before it
        # republishes the calibrated pose and emits localization_seeded.
        Node(
            package="robot_bringup",
            executable="initial_pose_policy",
            name="initial_pose_policy",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "initial_pose_source": "auto",
                }
            ],
        ),
        Node(
            package="robot_experiments",
            executable="outdoor_initial_pose",
            name="outdoor_initial_pose",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "x": ParameterValue(LaunchConfiguration("start_x"), value_type=float),
                    "y": ParameterValue(LaunchConfiguration("start_y"), value_type=float),
                    "yaw_deg": ParameterValue(
                        LaunchConfiguration("start_yaw_deg"), value_type=float
                    ),
                }
            ],
        ),
        Node(
            package="bio_nav_ros_bridge",
            executable="v310_visualizer",
            name="bio_nav_attempt31_outdoor_visualizer",
            output="screen",
            parameters=[
                {
                    "use_sim_time": ParameterValue(
                        LaunchConfiguration("use_sim_time"), value_type=bool
                    ),
                    "region_config_file": str(files["region_config_file"]),
                    "waypoint_config_file": str(files["waypoint_config_file"]),
                }
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rivermark_rviz",
            output="screen",
            arguments=[
                "-d",
                str(description_share / "rviz" / "rivermark.rviz"),
            ],
            parameters=[
                {
                    "use_sim_time": ParameterValue(
                        LaunchConfiguration("use_sim_time"), value_type=bool
                    )
                }
            ],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
        # The Nav2 StaticLayer is transient-local, but if it configures in the
        # narrow interval before this launch's map_server becomes active it can
        # retain its 12 m default window for the whole mission.  Give the map
        # lifecycle transaction a bounded head start; the campaign and runner
        # still fail closed on the actual 1600x1600 global-costmap extent.
        TimerAction(
            period=2.0,
            actions=[
                _include(
                    "robot_navigation",
                    "navigation.launch.py",
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": "true",
                        "nav2_profile_params_file": LaunchConfiguration(
                            "nav2_profile_params_file"
                        ).perform(context),
                        "structural_map_file": str(files["map_file"]),
                        "route_graph_file": str(files["route_graph_file"]),
                        "region_config_file": str(files["region_config_file"]),
                        # The frozen Route Server asset contains every feasible
                        # component. The coordinator must export the identical
                        # support-node ID space; pruning only its private copy
                        # would make the same uint16 ID refer to a different
                        # map position.
                        "feasible_only_largest_component": "false",
                        "module2_enabled": LaunchConfiguration(
                            "module2_enabled"
                        ).perform(context),
                        "controller_max_linear_velocity_mps": LaunchConfiguration(
                            "controller_max_linear_velocity_mps"
                        ).perform(context),
                        "controller_linear_velocity_std_mps": LaunchConfiguration(
                            "controller_linear_velocity_std_mps"
                        ).perform(context),
                        "execute_route_navigation": "true",
                        "voxel_grid_topic": "stvl_voxel_grid",
                    },
                )
            ],
        ),
    ])
    return actions


def generate_launch_description():
    navigation_share = Path(get_package_share_directory("robot_navigation"))
    odometry_share = Path(get_package_share_directory("robot_odometry"))
    localization_share = Path(
        get_package_share_directory("robot_localization_config")
    )
    mapping_share = Path(get_package_share_directory("robot_mapping"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            # The ideal defaults preserve the qualified Attempt31 behavior;
            # odometry_mode=realistic with localization_backend=amcl runs the
            # wheel-odometry + EKF + AMCL estimated localization chain.
            DeclareLaunchArgument("odometry_mode", default_value="ideal"),
            DeclareLaunchArgument("localization_backend", default_value="ideal"),
            DeclareLaunchArgument(
                "wheel_odometry_params_file",
                default_value=str(
                    odometry_share / "config" / "wheel_odometry.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "ekf_params_file",
                default_value=str(localization_share / "config" / "ekf.yaml"),
            ),
            DeclareLaunchArgument(
                "amcl_params_file",
                default_value=str(mapping_share / "config" / "amcl.yaml"),
            ),
            DeclareLaunchArgument("map_file"),
            DeclareLaunchArgument("route_graph_file"),
            DeclareLaunchArgument("region_config_file"),
            DeclareLaunchArgument("waypoint_config_file"),
            DeclareLaunchArgument("start_x"),
            DeclareLaunchArgument("start_y"),
            DeclareLaunchArgument("start_yaw_deg", default_value="0.0"),
            DeclareLaunchArgument("module2_enabled", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            # Long-range outdoor execution uses the historically validated
            # Jackal envelope, not A21's conservative indoor 0.35 m/s cap.
            DeclareLaunchArgument(
                "controller_max_linear_velocity_mps", default_value="0.75"
            ),
            DeclareLaunchArgument(
                "controller_linear_velocity_std_mps", default_value="0.35"
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
