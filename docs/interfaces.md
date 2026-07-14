# Runtime interfaces and ownership contracts

This document is the runtime contract for the current Isaac Sim standalone and
ROS 2 bringup. It describes what the implementation publishes today, not every
interface proposed in `plan.md`.

## Mode pairing

| ROS operation | Isaac `--navigation-mode` | SLAM executable | Saved-map input | Nav2 |
| --- | --- | --- | --- | --- |
| `mapping` | `mapping` | `async_slam_toolbox_node` | none; Pose Graph and OccupancyGrid inputs are rejected, and no manifest is consumed | off |
| `incremental_mapping` | `mapping` | `async_slam_toolbox_node` | one verified four-artifact manifest bundle; OccupancyGrid is not an input | off |
| `localization` | `localization` | `localization_slam_toolbox_node` | one verified four-artifact manifest bundle | off |
| `navigation` | `localization` | `localization_slam_toolbox_node` | one verified four-artifact manifest bundle | readiness-gated |

Every saved-map operation requires one strict manifest at
`data/maps/manifests/<map_version>.yaml`. The manifest binds the OccupancyGrid
YAML/PGM and Pose Graph `.posegraph`/`.data` as one indivisible bundle.
`localization` and `navigation` additionally require `map_file`; they reject a
missing or nonexistent OccupancyGrid YAML before launch. Baseline `mapping`
does not consume an existing map and therefore has no manifest input.

Automatic initial pose is a stronger contract than merely having four files.
For `initial_pose_source=auto`, the manifest must be calibrated, its
`calibration.spawn_pose_profile` must equal `spawn_pose_name`, and the selected
entry in `spawn_poses.yaml` must repeat the exact `map_version` and
`map_bundle_sha256`. `incremental_mapping` requires `auto`, so it also requires
this calibration. `localization` and `navigation` may use an uncalibrated new
bundle only with `initial_pose_source=rviz`, in which case the operator owns the
pose and must provide a new valid **2D Pose Estimate** after every Reset.

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
The script likewise derives
`data/maps/manifests/<posegraph-basename>.yaml` when `map_manifest_file` is
omitted. An explicit Pose Graph or OccupancyGrid path must resolve to the exact
artifact named by that manifest; cross-version mixing fails before nodes start.

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

- Mapping shows the live SLAM `/map`. It intentionally does not load the SLAM
  Toolbox lifecycle panel; lifecycle ownership stays outside RViz.
- Localization shows the fixed `/map`, keeps `/slam_toolbox/map` as a disabled diagnostic overlay, and provides `SetInitialPose`.
- Navigation loads the project-owned
  `robot_rviz_plugins/Navigation 2 Safe` panel and the official Nav2
  `GoalTool`, plus dual costmaps, paths, footprints, and Collision Monitor
  zones. The safe panel preserves the official action interface while owning
  and joining its QThread/QtConcurrent work during close. It sends Nav2 actions
  directly; there is no project `/goal_pose` bridge.
- Mapping, Localization, and Navigation include an Image display for
  `/camera/front/image_raw` using raw transport and Best Effort/Volatile depth
  2. A Camera `off` profile therefore produces a deliberate `No Image` state,
  not a graph error. `camera_view.rviz` is the camera-only layout.

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
| `/scan` | `sensor_msgs/msg/LaserScan` | `pointcloud_to_laserscan` | SLAM Toolbox, costmaps, production Collision Monitor; optional scan-fault bridge input | `base_link`, nominal 10 Hz, 720 bins; Best Effort + Volatile |
| `/scan_fault` | `sensor_msgs/msg/LaserScan` | opt-in `scan_fault_bridge` | Collision Monitor only when a safety-test parameter overlay explicitly selects it | unchanged scan or test-only frame override; Best Effort + Volatile |
| `/scan_fault/control` | `std_msgs/msg/String` | safety-test operator/runner | `scan_fault_bridge` JSON command input | Reliable + Volatile; every command must carry the current epoch |
| `/scan_fault/status` | `std_msgs/msg/String` | opt-in `scan_fault_bridge` | safety-test evidence and automation | transient-local JSON snapshot, plus periodic/event updates |
| `/map` | `nav_msgs/msg/OccupancyGrid` | SLAM Toolbox in Mapping; `nav2_map_server` in Localization/Navigation | map inspection in Mapping; activation gate and global costmap in Navigation | `map`; reliable, transient local; exactly one mode-appropriate publisher |
| `/slam_toolbox/map` | `nav_msgs/msg/OccupancyGrid` | SLAM Toolbox in Localization/Navigation | diagnostics only; never a Nav2 static-map input | `map`; scan-rasterized localization view |
| `/wheel/odom` | `nav_msgs/msg/Odometry` | `wheel_odometry` | EKF | Realistic only; `odom`/`base_link` |
| `/odom` | `nav_msgs/msg/Odometry` | Isaac in Ideal; EKF in Realistic | SLAM, Nav2, experiments | `odom`/`base_link`; exactly one publisher |
| `/plan` | `nav_msgs/msg/Path` | Nav2 Planner Server | RViz and profiler global-plan evidence | `map`; Navigation only |
| `/optimal_trajectory` | `nav_msgs/msg/Path` | MPPI Controller Server | RViz **Local Plan** and runtime profiler | selected local trajectory in `odom`; Navigation only, nominal controller rate; Reliable + Volatile |
| `/transformed_global_plan` | `nav_msgs/msg/Path` | MPPI Controller Server | optional transformed-reference diagnostic | reference path in the controller frame, not the local plan; Reliable + Volatile |
| `/trajectories` | `visualization_msgs/msg/MarkerArray` | MPPI trajectory visualizer | opt-in candidate-sample visualization | Reliable + Volatile, expensive/lazy; committed RViz display is disabled and creates no subscription |
| `/camera/front/image_raw` | `sensor_msgs/msg/Image` | Isaac front Camera graph | RViz or external perception/recording only | `camera_front_optical_frame`, `rgb8`; profile rate/resolution; Best Effort + Volatile, depth 2 |
| `/camera/front/camera_info` | `sensor_msgs/msg/CameraInfo` | same Isaac Render Product as Image | camera calibration consumers and profiler pairing | same frame, simulated stamp and QoS as Image; Best Effort + Volatile, depth 2 |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | calibrated initial-pose node/Isaac Reset in `auto`, or RViz in `rviz` | SLAM Toolbox localization | `map`; Reliable + Volatile; invalid frame/non-finite/non-normalized manual poses are ignored |
| `/initial_pose/status` | `std_msgs/msg/String` | calibrated initial-pose node | operator and recovery diagnostics | transient-local state such as waiting clock/scan/TF, complete, or manual override |
| `/simulation/initial_pose_source` | `std_msgs/msg/String` | `initial_pose_policy` | Isaac Reset and ROS recovery contract | transient-local `auto` or `rviz` |
| `/ground_truth/odom` | `nav_msgs/msg/Odometry` | optional Isaac GT recorder | metrics only | `map`/`ground_truth_base_link`, configured 60 Hz |
| `/ground_truth/path` | `nav_msgs/msg/Path` | optional Isaac GT recorder | metrics/visualization only | `map`, configured 10 Hz |
| `/simulation/collision` | `std_msgs/msg/Bool` | Isaac chassis contact sensor | experiment safety metric | about 20 Hz; instantaneous contact state |
| `/collision_monitor_state` | `nav2_msgs/msg/CollisionMonitorState` | Nav2 Collision Monitor | experiment lock/stop metric | Navigation only |
| `/simulation/reset_event` | `std_msgs/msg/Empty` | Isaac Reset bridge | Wheel Odom, Activation Gate, scan-fault bridge and reset observers | Reliable + Volatile; one event after each successful transaction, and the recovery-epoch boundary |
| `/simulation/localization_seeded` | `std_msgs/msg/Empty` | Isaac Reset bridge | experiment reset gate | Reliable + Volatile; emitted after a post-reset `/scan` triggers `/initialpose` |
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

## Map manifest and calibration identity

A saved map is committed only as this fixed layout:

```text
data/maps/manifests/<version>.yaml
data/maps/occupancy/<version>.yaml
data/maps/occupancy/<version>.pgm
data/maps/posegraphs/<version>.posegraph
data/maps/posegraphs/<version>.data
```

The manifest records the byte count and lowercase SHA256 of all four artifacts,
then hashes their ordered role/path/size/content identities into one
`bundle_sha256`. Validation rejects unknown schema fields, unsafe or overlong
versions（包括纯点保留名）、absolute/cross-version paths、任意父级 symlink 和 path escapes、unhydrated
Git LFS pointers, size/hash/bundle mismatches, and incomplete bundles. It also
proves that the OccupancyGrid YAML names exactly the manifested PGM and that the
YAML 的正数 resolution/origin and PGM dimensions match the manifest metadata. A valid
individual file is therefore not sufficient evidence for a valid map.

`scripts/save_map.sh <version>` is transactional and refuses every overwrite.
It saves all four artifacts in `data/maps/.staging`, creates an uncalibrated
manifest there, uses same-filesystem no-clobber hard links to publish and verify
the four artifacts, and atomically publishes the manifest last. The manifest is
the commit marker: an interrupted or failed transaction removes only inodes
owned by that transaction, preserving any same-name file created concurrently
instead of exposing a half-written version or deleting another writer's data. A
newly saved bundle deliberately has
`calibration.calibrated: false`; saving a map is not calibration.

Calibration binds two documents to the same identity:

- the manifest names `spawn_pose_profile`, repeats its own `bundle_sha256`, and
  records the calibration time/method and measured poses;
- `spawn_poses.yaml` marks that profile calibrated and repeats the identical
  `map_version` and `map_bundle_sha256` beside its Map Pose;
- both documents must also match the exact USD position/yaw、Map position/yaw
  and position/yaw standard deviations. Keeping an old hash while editing a
  pose is rejected.

Changing any of the four artifacts creates a different bundle. Do not edit a
hash to make a modified bundle look calibrated, copy a Map Pose across versions,
or reuse an old manifest. Generate a new version, verify it, measure its
calibration, and update both sides of the binding.

## Camera stream contract

The front Camera is an optional observation stream. It is not consumed by SLAM,
EKF, Nav2, Collision Monitor, Reset, or the activation gate. `--camera-profile`
selects one of these strict profiles before Kit starts:

| Profile | Resolution | Configured target rate | Contract |
| --- | ---: | ---: | --- |
| `off` | none | 0 Hz | create no Camera/Render Product/ROS publishers |
| `monitoring` | 640×360 | 15 Hz | GUI default and normal navigation observation |
| `standard` | 640×480 | 20 Hz | intermediate observation/recording load |
| `high_quality` | 1280×720 | 30 Hz | visual-quality run; not the navigation performance baseline |

When the CLI option is omitted, GUI mode resolves to `monitoring` and headless
mode resolves to `off`. Rates in the table are configured targets, not wall-time
guarantees: GPU load and RTF determine the observed rate.

One SensorFactory-owned Render Product feeds both
`/camera/front/image_raw` (`rgb8`) and `/camera/front/camera_info`. Both helpers
use simulation time, `camera_front_optical_frame`, raw sensor semantics, and the
same Keep Last / depth 2 / Best Effort / Volatile QoS. Best Effort delivery means
consumer-side counts can differ; synchronize Image and CameraInfo by header
stamp and record mismatches instead of assuming every arrival is paired. Camera
graph destruction precedes Render Product release, including shutdown and
profile teardown.

Camera schema v3 separates the USD optical aperture from manual exposure:
`optics.f_stop=0.0` disables optical depth of field for the machine-vision
camera, while `exposure.f_stop` remains an independent exposure value. Each
enabled CameraFront RenderProduct applies
`OmniRtxPostDebugSettingsAPI_1` (`omni:rtx:post:aa:op=rtxaa`, token),
`OmniRtxPostMotionBlurAPI_1`
(`omni:rtx:post:motionblur:enabled=false`, bool), and
`OmniRtxPostDofAPI_1` (`omni:rtx:post:dof:enabled=false`, bool). The Camera
prim applies `OmniRtxCameraAutoExposureAPI_1`
(`omni:rtx:autoExposure:enabled=false`, bool). These are per-prim contracts;
the sensor factory does not mutate the interactive viewport's global renderer
settings. They are implementation evidence, not a substitute for real rendered
image inspection.

## Local-plan visualization contract

MPPI `FollowPath.visualize` is enabled so the controller publishes its selected
trajectory as `/optimal_trajectory` (`nav_msgs/msg/Path`). This is the only
Topic named **Local Plan** in the committed RViz and profiler contracts. A real
navigation run produces it in `odom` at approximately the controller rate.

`/transformed_global_plan` is the global reference path transformed into the
controller frame; it is useful for comparison but is not the chosen local
trajectory. `/trajectories` is the heavyweight MarkerArray of candidate MPPI
samples. Its publisher is lazy, the committed RViz display is disabled, and the
normal graph must have no `/trajectories` subscriber. Enable it only for a
short, explicit controller-debug session.

## Collision freshness and scan-fault test interface

The production Collision Monitor consumes `/scan` directly. Its command chain
is `/cmd_vel_nav -> /cmd_vel_smoothed -> /cmd_vel`, and only its output owns the
final Navigation `/cmd_vel`. The committed freshness boundary is
`source_timeout: 0.40 s`, with `transform_tolerance: 0.20 s`: a sustained scan
outage or a scan whose frame cannot transform to `base_link` is invalid input
and must stop the robot. One or two missing nominal 10 Hz samples remain inside
the timeout and are not by themselves proof of a safety fault.

`scan_fault_bridge` is an opt-in verification adapter, never a production
dependency. A complete fault test must both launch `/scan -> /scan_fault` and
provide a temporary Nav2 parameter overlay that selects `/scan_fault` for the
Collision Monitor. Starting the bridge alone leaves production Navigation on
`/scan`.

The Reliable/Volatile `/scan_fault/control` payload is one JSON object. Supported
commands are:

```json
{"command":"drop_next","count":2,"epoch":0}
{"command":"pause_for","seconds":0.6,"epoch":0}
{"command":"drop_all","epoch":0}
{"command":"replace_frame_id","frame_id":"missing_scan_frame","epoch":0}
{"command":"resume","epoch":0}
```

`resume` is accepted as an alias for canonical mode `normal`. Only one fault
mode is active at a time. `/scan_fault/status` is Reliable/Transient Local and reports event/result,
mode, current epoch, command sequence, active fields, and total/per-epoch
received/forwarded/dropped counters. A successful `/simulation/reset_event` or
a scan timestamp rollback opens a new epoch, clears the active fault and
per-epoch counters, and rejects a delayed command that names the old epoch.

## Isaac-side QoS profiles

| Profile | Reliability | Durability | History/depth | Used by |
| --- | --- | --- | --- | --- |
| `clock` | best effort | volatile | keep last / 1 | `/clock` |
| `sensor_data` | best effort | volatile | keep last / 5 | point cloud, IMU |
| `camera_sensor_data` | best effort | volatile | keep last / 2 | front RGB Image and CameraInfo |
| `command` | reliable | volatile | keep last / 1 | `/cmd_vel` subscription |
| `state` | reliable | volatile | keep last / 10 | JointState, Ideal `/odom` |
| `tf` | reliable | volatile | keep last / 100 | Isaac dynamic TF |
| `static_tf` | reliable | transient local | keep last / 1 | Isaac static TF |

These are the explicit Isaac graph profiles. ROS package publishers and
subscriptions retain their own node-specific QoS. Topic discovery alone is not
proof of compatibility; use `ros2 topic info --verbose <topic>` when debugging.

The committed RViz configs explicitly bind map-like topics (`/map`, both costmaps) as Reliable + Transient Local. Sensor streams (`/scan`, `/lidar/points_raw`) are Best Effort + Volatile. This distinction is regression-tested. The top-level interactive launch starts RViz before delaying perception by 1.5 seconds so the saved sensor QoS is applied before the `/scan` publisher appears; this avoids a misleading one-shot constructor warning without changing the final graph.

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
| `reset_pose_name` | key in `spawn_poses.yaml`; immutable in Localization because it is bound to the startup Manifest profile |
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

A Reset has two related identities. The Isaac bridge increments an internal
transaction `generation` when it accepts a request; async callbacks from another
generation cannot complete it, overlapping transactions are rejected, and a
failed/timed-out generation emits no reset event. The reliable, volatile
`/simulation/reset_event` emitted by a successful transaction is the public
recovery-epoch boundary. Activation Gate clears TF/readiness and invalidates old
async tokens, Wheel Odom resets its integrator, the scan-fault bridge clears its
fault and increments its epoch, and profilers split time-series evidence at a
clock rollback. Because the event is Volatile, observers must subscribe before
calling Reset; late subscribers cannot infer that an event occurred.

A successful Trigger response therefore means the physical reset and all submitted downstream reset/clear calls completed, not merely that they were queued. It still does not prove localization readiness. Clients must wait for the appropriate automatic seed or new RViz pose, fresh `/odom`, and a newly stamped, stable `map -> odom`. The experiment runner additionally requires fresh post-event Ground Truth and spawn-aligned `map -> base_link` before dispatching a goal.

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
epoch.

After a successful lifecycle `STARTUP`, the activation gate remains alive.
On reset it cancels active navigation, pauses managed nodes, clears global and
local costmaps, calls calibrated reseed in `auto` mode or waits for a new RViz
pose in `rviz` mode, rebuilds readiness evidence, and resumes the stack. A
recovery service failure blocks resume rather than silently activating an
inconsistent stack. An actual gate process exit shuts down the composed stack.

## Ordered shutdown and process ownership

`scripts/run_ros.sh` is the ROS process-group supervisor, not a transparent
alias for `ros2 launch`. The launch child, integrated RViz, Mapping Teleop, and
ordered-lifecycle helper use separate process groups. Their metadata and live
members must match the project root and `ISAAC_NAV_SESSION_ID` created by this
supervisor. Terminal `SIGINT`, `SIGTERM`, or `SIGHUP` therefore reaches the
supervisor while Lifecycle services and executors are still alive. It invokes
`robot_bringup.ordered_shutdown` with a private ROS Context and
SingleThreadedExecutor. One global shutdown deadline defaults to 20 seconds;
the helper may use at most 10 seconds of the remaining budget and reserves the
last second for process-group escalation and metadata cleanup:

| Operation | Ordered Lifecycle responsibility |
| --- | --- |
| Navigation | shut down Navigation manager first, then Localization manager |
| Localization | shut down Localization manager |
| Mapping / Incremental Mapping | deactivate, clean up, then shut down SLAM Toolbox |

After those requests complete—or are reported as explicit warnings—the
supervisor sends `SIGINT` to every authenticated launch/RViz/Teleop/helper group
and waits. Remaining groups receive `SIGTERM` and then `SIGKILL`, with identity
revalidation before every phase. A second terminal stop request skips to TERM;
a third requests KILL. Metadata deletion refuses to remove a still-live group,
and cached PGIDs are pinned to the original leader start ticks while that
leader exists. The Navigation RViz config uses the project-owned observer-only
safe panel so its ROS thread and Qt futures are cooperatively interrupted and
joined; integrated RViz also has its own registered group and closes inherited
instance-lock file descriptors in the child.

The ordered contract applies when the supervisor receives the signal. Directly
killing a launch child, lifecycle manager, or RViz bypasses some or all of the
ordering. `clean_runtime.sh` is the cross-session recovery entrypoint for a lost
terminal: for each registered Isaac/ROS/RViz/Teleop/motion-baseline component it
validates PID/PGID, boot ID, leader start ticks, UID, project root, command, and
every live group member. It then revalidates before its own bounded
INT→TERM→KILL phases and retains metadata if a group remains visible. This
recovery path does not require the current shell to possess the old session ID.
Neither path may be replaced with manual broad `pkill` or unverified `kill`
commands.

## Navigation control performance contract

The two committed MPPI overlays share the measured control timing and differ
only in sample count:

| Parameter | `stable` | `performance` | Contract |
| --- | ---: | ---: | --- |
| `controller_frequency` | 10 Hz | 10 Hz | control period is exactly `model_dt` |
| `FollowPath.time_steps` | 20 | 20 | two-second prediction horizon |
| `FollowPath.model_dt` | 0.10 s | 0.10 s | preserves MPPI control-sequence shifting |
| `FollowPath.batch_size` | 750 | 1000 | stable keeps more compute headroom; performance evaluates more samples |
| Localization `throttle_scans` | 2 | 2 | SLAM processes every second scan; Collision Monitor still receives every `/scan` |

Profile validation runs before Nav2 nodes start. Values must be finite and
positive, steps/batch must be positive integers, and controller period
`1 / controller_frequency` must not exceed `model_dt`. A configuration such as
8 Hz with `model_dt=0.10 s` fails fast instead of starting a controller that
cannot satisfy the MPPI timing invariant. Velocity Smoother remains at 20 Hz;
controller prediction cadence and final smoothed-command cadence are separate
contracts. Re-tune only with comparable runtime evidence.

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
