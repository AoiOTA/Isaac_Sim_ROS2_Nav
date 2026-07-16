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

## Interactive workflow contract

All four top-level bringups expose the same interaction arguments:

| Argument | Default | Contract |
| --- | --- | --- |
| `interactive` | `true` | `false` disables both RViz and Teleop regardless of their individual values. |
| `use_rviz` | `true` | Launch exactly one managed RViz process with the operation-specific config. |
| `rviz_config` | `auto` | Select `mapping.rviz`, `localization.rviz`, or `navigation.rviz`; an explicit path must exist. |
| `use_teleop` | `auto` | Enable only for `mapping` and `incremental_mapping`; explicit true in Localization/Navigation is rejected. |
| `initial_pose_source` | `auto` | `auto` owns calibrated reseeding; `rviz` gives ownership to valid `/initialpose` input. Incremental Mapping requires `auto`. |

The three RViz configurations are mode contracts, not cosmetic presets:

- Mapping shows the live SLAM `/map` and enables the SLAM Toolbox panel.
- Localization shows the fixed `/map`, keeps `/slam_toolbox/map` as a disabled diagnostic overlay, and provides `SetInitialPose`.
- Navigation additionally loads the official `nav2_rviz_plugins/Navigation 2` panel and `GoalTool`, dual costmaps, paths, footprints, and Collision Monitor zones. It sends Nav2 actions directly; there is no project `/goal_pose` bridge.

Managed RViz/Teleop processes use the same environment and PID registry as the main scripts. Mapping Teleop publishes Reliable/Volatile `/cmd_vel` at 20 Hz, stops after 0.18 seconds without a key event using a monotonic wall clock, clamps commands to 1.0 m/s and 1.5 rad/s, and publishes a final zero on every normal/signal/EOF exit. Navigation's command chain remains the sole owner in its mode.

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
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | calibrated initial-pose node/Isaac Reset in `auto`, or RViz in `rviz` | SLAM Toolbox localization | `map`; Reliable + Volatile; invalid frame/non-finite/non-normalized manual poses are ignored |
| `/initial_pose/status` | `std_msgs/msg/String` | calibrated initial-pose node | operator and recovery diagnostics | transient-local state such as waiting clock/scan/TF, complete, or manual override |
| `/simulation/initial_pose_source` | `std_msgs/msg/String` | `initial_pose_policy` | Isaac Reset and ROS recovery contract | transient-local `auto` or `rviz` |
| `/ground_truth/odom` | `nav_msgs/msg/Odometry` | optional Isaac GT recorder | metrics only | `map`/`ground_truth_base_link`, configured 60 Hz |
| `/ground_truth/path` | `nav_msgs/msg/Path` | optional Isaac GT recorder | metrics/visualization only | `map`, configured 10 Hz |
| `/simulation/collision` | `std_msgs/msg/Bool` | Isaac chassis contact sensor | experiment safety metric | about 20 Hz; instantaneous contact state |
| `/collision_monitor_state` | `nav2_msgs/msg/CollisionMonitorState` | Nav2 Collision Monitor | experiment lock/stop metric | Navigation only |
| `/simulation/reset_event` | `std_msgs/msg/Empty` | Isaac Reset bridge | Wheel Odom and reset observers | one event per Reset |
| `/simulation/localization_seeded` | `std_msgs/msg/Empty` | Isaac Reset bridge | experiment reset gate | emitted after a post-reset `/scan` triggers `/initialpose` |
| `/simulation/reset` | `std_srvs/srv/Trigger` | Isaac Reset bridge | operator/experiment runner | deterministic reset request |
| `/initial_pose/reseed` | `std_srvs/srv/Trigger` | calibrated initial-pose node | Activation Gate reset recovery | arm calibrated pose after a post-request scan; preserves valid manual ownership |

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

The committed RViz configs explicitly bind map-like topics (`/map`, both costmaps) as Reliable + Transient Local. Sensor streams (`/scan`, `/lidar/points_raw`) are Best Effort + Volatile. This distinction is regression-tested. The top-level interactive launch starts RViz before delaying perception by 1.5 seconds so the saved sensor QoS is applied before the `/scan` publisher appears; this avoids a misleading one-shot constructor warning without changing the final graph.

## Isaac wheel-control execution contract

The control graph is an on-demand graph driven by
`isaacsim.core.nodes.OnPhysicsStep`, not by a render/playback tick and not by
`ROS2SubscribeTwist.execOut`.

On every physics step it:

1. polls `/cmd_vel`, retaining the subscriber's most recently received Twist
   when no new message is available;
2. executes `DifferentialController`;
3. supplies `OnPhysicsStep.deltaSimulationTime` directly to the controller's
   `dt` input;
4. expands `[left, right]` to `[front_left, front_right, rear_left,
   rear_right]`;
5. writes all four velocity targets through one
   `IsaacArticulationController` call.

Consequently the acceleration limits are integrated at the physics-step rate
and remain independent of whether Teleop/Nav2 publishes at 10 Hz, 20 Hz, or
another supported command rate. The single four-wheel write also prevents a
front/rear axle command from straddling two graph executions.

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

A Reset is a non-blocking ROS service transaction with one active generation at a time. Overlapping requests are rejected. It executes in this fixed order:

1. Validate the named spawn and required Map calibration before pausing.
2. Pause physics and publish zero velocity.
3. Recreate/clear controller state, apply the USD Pose, and zero chassis/joint velocity and targets.
4. Reset Ideal odometry, or submit Wheel Odom and EKF reset requests in Realistic mode.
5. Clear Ground Truth path, collision state, and deterministic dynamic-obstacle phase.
6. Submit available global/local Costmap clear requests.
7. Step physics once and always resume the timeline, including exception paths.
8. Await every submitted ROS future under steady-wall-time deadlines. A submitted call that fails or times out fails the Trigger; a service unavailable before submission is reported as skipped so the continuously running recovery gate can handle its layer. Stale callbacks carry a generation and cannot complete a newer transaction.
9. Only after the transaction succeeds, publish `/simulation/reset_event` and return `success: true`.
10. In Localization with `initial_pose_source=auto`, arm a simulation-clock evidence barrier, ignore old/high-epoch cached scans, and publish the calibrated `/initialpose` plus `/simulation/localization_seeded` on the first valid post-reset scan. With source `rviz`, automatic publication remains disabled.

A successful Trigger response therefore means the physical reset and all submitted downstream reset/clear calls completed, not merely that they were queued. It still does not prove localization readiness. Clients must wait for the appropriate automatic seed or new RViz pose, fresh `/odom`, and a newly stamped, stable `map -> odom`. The experiment runner additionally requires fresh post-event Ground Truth, spawn-aligned `map -> base_link`, and all six Nav2 managed nodes in `active` state before dispatching a goal.

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
and `3°` yaw over the TF stability window, with 30-second startup/recovery
timeouts. Re-reading the same cached TF does not renew its freshness. A
simulation-time rollback, excessive forward jump, or explicit reset event
starts a new readiness epoch, clears the TF buffer/stability evidence, and
invalidates callbacks from the old generation.

Before every transition the gate atomically snapshots all six managed Nav2
states and rejects duplicate node names. It is the sole lifecycle owner. Calls
carry a generation/token pair, are guarded by one lock, and use at most three
attempts with bounded exponential backoff; an old future cannot advance the new
epoch. If a manager command completes only partially, the gate normalizes the
mixed stable state in dependency order: activate/configure forward and
deactivate in reverse, then verifies the full snapshot again.

After a successful lifecycle `STARTUP`, the activation gate remains alive.
On reset it cancels active navigation, pauses managed nodes, clears global and
local costmaps, calls calibrated reseed in `auto` mode or waits for a new RViz
pose in `rviz` mode, rebuilds readiness evidence, and resumes the stack. A
recovery service failure blocks resume rather than silently activating an
inconsistent stack. An actual gate process exit shuts down the composed stack.

## Navigation control performance contract

The committed MPPI timing keeps the measured Isaac Sim 6.0.1 headless Ideal
baseline. The batch reduction also passed two consecutive Realistic curved
goals, including a reverse-turning goal:

| Parameter | Value | Reason |
| --- | --- | --- |
| `controller_frequency` | 10 Hz | Preserves the two-second horizon and remained close to its target under the measured Realistic load. |
| `FollowPath.time_steps` | 20 | Combined with `model_dt` keeps the prediction horizon at two seconds. |
| `FollowPath.model_dt` | 0.10 s | Matches the measured controller period. |
| `FollowPath.batch_size` | 500 | Avoids localization contention while preserving the two-second horizon. |
| Localization `throttle_scans` | 2 | Removes SLAM contention; Collision Monitor still consumes the full `/scan` stream. |

The 20 Hz/40×0.05/1500 baseline and tested 20 Hz reduced batches repeatedly
missed their deadline on this workstation. Velocity Smoother remains at 20 Hz;
the controller's prediction cadence and the final smoothed command cadence are
separate contracts. Re-tune only with comparable runtime evidence.

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

For calibrated Ideal Localization/Navigation, Isaac already publishes the
authoritative `odom -> base_link`. ROS therefore serves the immutable map and
publishes a freshly stamped identity `map -> odom` instead of applying a second
SLAM Toolbox localization correction. Realistic mode continues to use the
serialized Pose Graph and SLAM Toolbox for `map -> odom`.

`incremental_mapping.yaml` is a mapping-workflow descriptor. The
`NavigateToPose` runner rejects it deliberately; execute it through
`incremental_mapping` bringup, save a versioned map/Pose Graph pair, and compare
the resulting artifacts and elapsed mapping time explicitly.
