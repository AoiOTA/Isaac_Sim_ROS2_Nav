"""Map-aligned Ground Truth ROS publishers. This module never publishes TF."""

from __future__ import annotations

import math
from dataclasses import dataclass

from isaac_sim.src.config import GroundTruthConfig
from isaac_sim.src.ground_truth.transforms import Pose2D, compute_map_t_usd, usd_pose_to_map
from isaac_sim.src.robot.spawn_pose_manager import SpawnPose, require_map_calibration


class GroundTruthError(RuntimeError):
    pass


@dataclass(frozen=True)
class GroundTruthReceipt:
    """Read-only facts for the Ground Truth sample published by ``update``."""

    stamp_ns: int
    simulation_time_s: float
    position_xyz: tuple[float, float, float]
    yaw_rad: float
    post_assist_yaw_rate_radps: float


def _yaw_from_wxyz(q: tuple[float, float, float, float]) -> float:
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _stamp_from_seconds(seconds: float):
    from builtin_interfaces.msg import Time

    sec = int(math.floor(seconds))
    nanosec = int(round((seconds - sec) * 1_000_000_000))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    return Time(sec=sec, nanosec=nanosec)


class GroundTruthRecorder:
    def __init__(self, config: GroundTruthConfig, robot, node, spawn_pose: SpawnPose):
        if not config.enabled:
            raise GroundTruthError("GroundTruthRecorder must not be constructed while ground_truth.enabled=false")
        require_map_calibration(spawn_pose, "map-frame Ground Truth publication")
        self.config = config
        self.robot = robot
        self.node = node
        usd_start = Pose2D.from_degrees(
            spawn_pose.usd.position[0], spawn_pose.usd.position[1], spawn_pose.usd.yaw_deg
        )
        map_start = Pose2D.from_degrees(
            spawn_pose.map.position[0], spawn_pose.map.position[1], spawn_pose.map.yaw_deg
        )
        self.map_t_usd = compute_map_t_usd(usd_start, map_start)

        from nav_msgs.msg import Odometry, Path

        self._Odometry = Odometry
        self._Path = Path
        self._odom_publisher = node.create_publisher(Odometry, config.odom_topic, 10)
        self._path_publisher = node.create_publisher(Path, config.path_topic, 10)
        self._path = Path()
        self._path.header.frame_id = config.frame_id
        self._last_odom_time = -math.inf
        self._last_path_time = -math.inf

    def reset_path(self) -> None:
        self._path = self._Path()
        self._path.header.frame_id = self.config.frame_id
        self._last_path_time = -math.inf

    def update(self, simulation_time: float) -> GroundTruthReceipt | None:
        if simulation_time - self._last_odom_time < 1.0 / self.config.odom_hz:
            return None
        position, orientation_wxyz = self.robot.get_world_pose()
        _linear_velocity, angular_velocity = self.robot.get_base_velocities()
        usd_pose = Pose2D(position[0], position[1], _yaw_from_wxyz(orientation_wxyz))
        map_pose = usd_pose_to_map(self.map_t_usd, usd_pose)
        stamp = _stamp_from_seconds(simulation_time)
        qz = math.sin(map_pose.yaw * 0.5)
        qw = math.cos(map_pose.yaw * 0.5)

        message = self._Odometry()
        message.header.stamp = stamp
        message.header.frame_id = self.config.frame_id
        message.child_frame_id = self.config.child_frame_id
        message.pose.pose.position.x = map_pose.x
        message.pose.pose.position.y = map_pose.y
        message.pose.pose.position.z = position[2]
        message.pose.pose.orientation.z = qz
        message.pose.pose.orientation.w = qw
        # Ground truth is evaluation-only. It deliberately contains no TF broadcaster.
        self._odom_publisher.publish(message)
        self._last_odom_time = simulation_time

        receipt = GroundTruthReceipt(
            stamp_ns=int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec),
            simulation_time_s=float(simulation_time),
            position_xyz=(float(map_pose.x), float(map_pose.y), float(position[2])),
            yaw_rad=float(map_pose.yaw),
            post_assist_yaw_rate_radps=float(angular_velocity[2]),
        )

        if simulation_time - self._last_path_time >= 1.0 / self.config.path_hz:
            from geometry_msgs.msg import PoseStamped

            pose = PoseStamped()
            pose.header = message.header
            pose.pose = message.pose.pose
            self._path.header.stamp = stamp
            self._path.poses.append(pose)
            self._path_publisher.publish(self._path)
            self._last_path_time = simulation_time
        return receipt
