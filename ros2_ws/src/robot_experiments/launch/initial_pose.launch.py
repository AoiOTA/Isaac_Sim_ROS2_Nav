from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    spawn_poses_file = LaunchConfiguration("spawn_poses_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "spawn_poses_file",
                default_value=EnvironmentVariable(
                    "ISAAC_NAV_SPAWN_POSES",
                    default_value="",
                ),
                description=(
                    "Required path to the single project spawn_poses.yaml truth source; "
                    "the node refuses an empty path or an uncalibrated map pose"
                ),
            ),
            DeclareLaunchArgument("spawn_pose_name", default_value="mapping_start"),
            DeclareLaunchArgument("wait_for_odom_to_base_tf", default_value="true"),
            DeclareLaunchArgument("publish_count", default_value="5"),
            DeclareLaunchArgument("publish_period_sec", default_value="0.5"),
            DeclareLaunchArgument("stay_alive_for_reseed", default_value="true"),
            Node(
                package="robot_experiments",
                executable="initial_pose_publisher",
                name="initial_pose_publisher",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "spawn_poses_file": spawn_poses_file,
                        "spawn_pose_name": LaunchConfiguration("spawn_pose_name"),
                        "wait_for_odom_to_base_tf": ParameterValue(
                            LaunchConfiguration("wait_for_odom_to_base_tf"),
                            value_type=bool,
                        ),
                        "publish_count": ParameterValue(
                            LaunchConfiguration("publish_count"), value_type=int
                        ),
                        "publish_period_sec": ParameterValue(
                            LaunchConfiguration("publish_period_sec"), value_type=float
                        ),
                        "stay_alive_for_reseed": ParameterValue(
                            LaunchConfiguration("stay_alive_for_reseed"),
                            value_type=bool,
                        ),
                    }
                ],
            ),
        ]
    )
