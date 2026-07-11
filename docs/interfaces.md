# Runtime interfaces and ownership contracts

This document is the runtime contract for the current Isaac Sim standalone and
ROS 2 bringup. It describes what the implementation publishes today, not every
interface proposed in `plan.md`.

## Mode pairing

| ROS operation | Isaac `--navigation-mode` | SLAM executable | Pose Graph requirement | Occupancy map requirement | Nav2 |
| --- | --- | --- | --- | --- | --- |
| `mapping` | `mapping` | `async_slam_toolbox_node` | must be empty | must be empty | off |
| `incremental_mapping` | `mapping` | `async_slam_toolbox_node` | `<prefix>.posegraph` and `<prefix>.data` | must be empty | off |
| `localization` | `localization` | `localization_slam_toolbox_node` | `<prefix>.posegraph` and `<prefix>.data` | saved `.yaml` plus referenced image | off |
| `navigation` | `localization` | `localization_slam_toolbox_node` | `<prefix>.posegraph` and `<prefix>.data` | saved `.yaml` plus referenced image | readiness-gated |

`incremental_mapping`, `localization`, and `navigation` also require a measured
Map Pose with `map.calibrated: true`. Baseline mapping is the only operation
that intentionally permits an uncalibrated Map Pose. `localization` and
`navigation` additionally require `map_file`; they reject a missing or
nonexistent OccupancyGrid YAML before launch.

The ROS `posegraph_file` argument is normalized to a prefix, so all of the
following refer to the same serialized map:

```text
data/maps/posegraphs/warehouse_v1
data/maps/posegraphs/warehouse_v1.posegraph
data/maps/posegraphs/warehouse_v1.data
```

Both files must exist. Baseline `mapping` rejects a non-empty Pose Graph instead
of silently turning into an incremental run. For `localization` and
`navigation`, `scripts/run_ros.sh` derives
`data/maps/occupancy/<posegraph-basename>.yaml` when `map_file` is omitted. Pass
`map_file:=/path/to/map.yaml` explicitly when the basenames differ. The inferred
or explicit file must exist; inference never falls back to a live SLAM map.

Structure TF ownership is also a two-process contract:

```bash
# Default: Isaac owns structure TF
./scripts/run_isaac.sh --mode realistic --structure-tf-source isaac
./scripts/run_ros.sh mapping \
  odometry_mode:=realistic structure_tf_source:=isaac

# Optional: RSP owns structure TF; valid only in Realistic mode
./scripts/run_isaac.sh --mode realistic --structure-tf-source rsp
./scripts/run_ros.sh mapping \
  odometry_mode:=realistic structure_tf_source:=rsp
```

Omitting the option selects `isaac` on both sides. The combination
`--mode ideal --structure-tf-source rsp` fails configuration validation.

The four top-level ROS bringups also expose the Stage-13 robot replacement
seams `robot_description_file`, `wheel_odometry_params_file`, and
`nav2_params_file`. Defaults select the Jackal files; explicit replacements are
checked for existence and file type before any node starts. These parameters
replace robot geometry/kinematics/tuning only—they do not permit renaming the
stable topics, frames, or Nav2 plugin topology. Isaac selects the matching
project/robot profile through `ISAAC_NAV_PROJECT_CONFIG`; see the
[custom-robot contract](../isaac_sim/assets/robots/custom_robot/README.md) for
the fail-fast calibration workflow.

## Topic and service contract

| Name | Type | Producer/owner | Consumer or purpose | Expected frame/rate |
| --- | --- | --- | --- | --- |
| `/clock` | `rosgraph_msgs/msg/Clock` | Isaac | every simulated-time ROS node | about 60 Hz |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Nav2 Collision Monitor or manual mapping teleop | Isaac wheel controller | `base_link` convention |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | Nav2 controller/behaviors | Velocity Smoother | Navigation only |
| `/cmd_vel_smoothed` | `geometry_msgs/msg/Twist` | Velocity Smoother | Collision Monitor | Navigation only |
| `/joint_states` | `sensor_msgs/msg/JointState` | Isaac | Wheel Odom and RobotModel | simulator tick |
| `/imu/data` | `sensor_msgs/msg/Imu` | Isaac | EKF in Realistic mode | `imu_link`, configured 60 Hz |
| `/lidar/points_raw` | `sensor_msgs/msg/PointCloud2` | Isaac RTX LiDAR | pointcloud-to-laserscan | `lidar_link`, nominal 10 Hz |
| `/scan` | `sensor_msgs/msg/LaserScan` | `pointcloud_to_laserscan` | SLAM Toolbox, costmaps, Collision Monitor | `base_link`, nominal 10 Hz, 720 bins |
| `/map` | `nav_msgs/msg/OccupancyGrid` | SLAM Toolbox in Mapping; `nav2_map_server` in Localization/Navigation | map inspection in Mapping; activation gate and global costmap in Navigation | `map`; reliable, transient local; exactly one mode-appropriate publisher |
| `/slam_toolbox/map` | `nav_msgs/msg/OccupancyGrid` | SLAM Toolbox in Localization/Navigation | diagnostics only; never a Nav2 static-map input | `map`; scan-rasterized localization view |
| `/wheel/odom` | `nav_msgs/msg/Odometry` | `wheel_odometry` | EKF | Realistic only; `odom`/`base_link` |
| `/odom` | `nav_msgs/msg/Odometry` | Isaac in Ideal; EKF in Realistic | SLAM, Nav2, experiments | `odom`/`base_link`; exactly one publisher |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | calibrated initial-pose node or Isaac Reset | SLAM Toolbox localization | `map`; never publish an uncalibrated pose |
| `/ground_truth/odom` | `nav_msgs/msg/Odometry` | optional Isaac GT recorder | metrics only | `map`/`ground_truth_base_link`, configured 60 Hz |
| `/ground_truth/path` | `nav_msgs/msg/Path` | optional Isaac GT recorder | metrics/visualization only | `map`, configured 10 Hz |
| `/simulation/collision` | `std_msgs/msg/Bool` | Isaac chassis contact sensor | experiment safety metric | about 20 Hz; instantaneous contact state |
| `/collision_monitor_state` | `nav2_msgs/msg/CollisionMonitorState` | Nav2 Collision Monitor | experiment lock/stop metric | Navigation only |
| `/simulation/reset_event` | `std_msgs/msg/Empty` | Isaac Reset bridge | Wheel Odom and reset observers | one event per Reset |
| `/simulation/localization_seeded` | `std_msgs/msg/Empty` | Isaac Reset bridge | experiment reset gate | emitted after a post-reset `/scan` triggers `/initialpose` |
| `/simulation/reset` | `std_srvs/srv/Trigger` | Isaac Reset bridge | operator/experiment runner | deterministic reset request |

The PointCloud-to-LaserScan projection uses `base_link` as its target frame,
height `[0.05, 0.50] m`, range `[0.30, 25.0] m`, a full `[-pi, pi]` field of
view, and a `0.5°` angular increment. The optional self filter is disabled by
default and changes the projection input only when explicitly enabled.

Ground Truth is evaluation-only. It publishes no TF and must not be remapped
into SLAM Toolbox, robot_localization, Nav2, Wheel Odom, or the controller.

In Localization/Navigation, `nav2_map_server` is the sole `/map` publisher and
serves the immutable saved OccupancyGrid. SLAM Toolbox still owns localization
and `map -> odom`, but its scan-rasterized map output is remapped to
`/slam_toolbox/map`. This separation prevents a moving obstacle observed during
localization from being persisted as a static-map ghost in Nav2. Mapping and
incremental mapping retain SLAM Toolbox ownership of `/map` because their
purpose is to change the map.

## Isaac-side QoS profiles

| Profile | Reliability | Durability | History/depth | Used by |
| --- | --- | --- | --- | --- |
| `clock` | best effort | volatile | keep last / 1 | `/clock` |
| `sensor_data` | best effort | volatile | keep last / 5 | point cloud, IMU |
| `command` | reliable | volatile | keep last / 1 | `/cmd_vel` subscription |
| `state` | reliable | volatile | keep last / 10 | JointState, Ideal `/odom` |
| `tf` | reliable | volatile | keep last / 100 | Isaac dynamic TF |
| `static_tf` | reliable | transient local | keep last / 1 | Isaac static TF |

These are the explicit Isaac graph profiles. ROS package publishers and
subscriptions retain their own node-specific QoS. Topic discovery alone is not
proof of compatibility; use `ros2 topic info --verbose <topic>` when debugging.

## TF ownership

The only navigation tree is:

```text
map -> odom -> base_link
                  |-> front_left_wheel_link
                  |-> front_right_wheel_link
                  |-> rear_left_wheel_link
                  |-> rear_right_wheel_link
                  |-> lidar_link
                  |-> imu_link
                  `-> camera_link -> camera_left/right_link -> optical frames
```

There is no ROS `world` frame. USD `/World` is a Stage prim path and is not a TF
frame.

| Edge or subtree | Ideal | Realistic | Constraint |
| --- | --- | --- | --- |
| `map -> odom` | SLAM Toolbox | SLAM Toolbox | exactly one Mapping or Localization instance |
| `odom -> base_link` | Isaac | `robot_localization` EKF | exactly one owner; Wheel Odom does not publish TF |
| wheel-link dynamic TF | Isaac | Isaac or RSP, selected explicitly | derived from articulation joints/JointState |
| sensor/camera static TF | Isaac | Isaac or RSP, selected explicitly | seven static pairs |

Structure ownership must match on both processes. The default is Isaac. In
Realistic mode, selecting RSP disables the Isaac StructureTF graph and enables
Robot State Publisher TF; `ideal + rsp` is rejected because Ideal is an
Isaac-owned odometry/structure mode. Never start Isaac and RSP structure
publishers for the same run. There is no cross-process negotiation, so a
mismatch remains an operator/configuration error and must be caught with the
ownership checks below.

Useful ownership checks:

```bash
ros2 topic info --verbose /odom
ros2 topic info --verbose /map
ros2 topic info --verbose /slam_toolbox/map
ros2 topic info --verbose /tf
ros2 topic info --verbose /tf_static
ros2 lifecycle get /map_server
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_tools view_frames
```

## Reset contract

The Isaac node declares these runtime parameters:

| Parameter | Meaning |
| --- | --- |
| `reset_seed` | non-negative dynamic-obstacle random seed |
| `reset_pose_name` | key in `spawn_poses.yaml` |
| `navigation_mode` | running Isaac mode: `mapping` or `localization` |
| `odometry_mode` | running mode: `ideal` or `realistic` |

A Reset executes in this fixed order:

1. Pause physics and publish zero velocity.
2. Recreate/clear controller state.
3. Apply the selected USD Pose and zero chassis/joint velocity and targets.
4. Reset Ideal odometry, or publish the reset event and queue Wheel Odom plus
   EKF resets in Realistic mode.
5. Clear Ground Truth path and collision latch/state.
6. Restore deterministic dynamic-obstacle phases from `reset_seed`.
7. Queue available global/local Costmap clear services.
8. Step physics once; in `localization` mode, record the current simulation-time
   barrier and schedule the calibrated Map Pose, then resume.
9. Ignore cached scans at or before the barrier. On the first `/scan` stamped
   strictly after it, publish `/initialpose` and then
   `/simulation/localization_seeded`.

Unavailable ROS-side reset services produce warnings and are best-effort. A
successful Trigger response therefore means the synchronous simulation reset
completed and available downstream requests were queued; clients must still
wait for `/simulation/localization_seeded`, fresh `/odom`, and a newly stamped,
stable `map -> odom`. The experiment runner snapshots the TF timestamp after the
seed event and requires a strictly newer transform, fresh post-event odometry
and Ground Truth samples, and spawn-aligned `map -> base_link` before sending a
goal.

## Navigation activation contract

The top-level `navigation` bringup starts Nav2 lifecycle nodes with autostart
disabled. It activates them only after all of these are true:

- non-zero, fresh `/clock`;
- fresh `/scan` and `/odom`;
- receipt of the reliable, transient-local `/map` (the static map is latched, so
  wall-time freshness is intentionally not required);
- a `map -> odom` transform with a fresh simulation timestamp, stable for at
  least 1 second;
- the Nav2 lifecycle management service is available.

Current tolerances are 0.5 seconds of input freshness, `0.05 m` translation,
and `3°` yaw over the TF stability window, with a 30-second startup timeout.
Re-reading the same cached TF does not renew its freshness. A simulation-time
rollback clears the accumulated TF stability state.

After a successful lifecycle `STARTUP`, the activation gate remains alive and
cancels only its readiness timer. The launch shutdown handler therefore cannot
race normal activation by interpreting successful gate completion as a process
failure; an actual gate process exit still shuts down the composed stack.

## Experiment scenario contract

The committed static scenario is a fixed
`warehouse_multiple_shelves_v1` smoke with an empty authored `static` obstacle
list. Its four seeds are deterministic repetitions of that same world, not four
random layouts. Non-empty authored static-obstacle lists are rejected because
physical static obstacle authoring is not implemented.

Before any static or dynamic run, the experiment runner reads the Isaac runtime
contract. Static runs require dynamic obstacles to be disabled. Dynamic runs
require the enabled flag, dynamic configuration SHA256, and sorted obstacle ID
set to match the scenario exactly; mismatches fail before Reset or goal dispatch.

The physical Isaac configuration and ROS scenario must also agree on every
dynamic obstacle's ID, supported shape, XY dimensions, Map-frame start/end
points, trajectory duration, and boolean `repeat`. `repeat: false` clamps the
obstacle at its endpoint after one traversal; `repeat: true` makes it traverse
back and forth. Both documents must state the value explicitly. The committed
dynamic baseline currently uses `repeat: false` for both obstacles.

`incremental_mapping.yaml` is a mapping-workflow descriptor. The
`NavigateToPose` runner rejects it deliberately; execute it through
`incremental_mapping` bringup, save a versioned map/Pose Graph pair, and compare
the resulting artifacts and elapsed mapping time explicitly.
