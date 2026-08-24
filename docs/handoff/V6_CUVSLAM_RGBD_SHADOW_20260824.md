# V6 cuVSLAM RGB-D visual-odometry Shadow handoff (2026-08-24)

## Result and boundary

- Implemented a **default-OFF** Isaac ROS cuVSLAM 4.5 RGB-D diagnostic
  shadow. OFF creates no component container, subscriptions, or GPU work.
- The shadow publishes only `/visual/odom_shadow` and `/visual/status` for
  recording. It is not an EKF input, publishes no TF, and has no Grid, map,
  Costmap, Nav2, safety, or control influence.
- Static tests/build validate the launch contract only. Isaac Sim, the live
  ROS graph, camera/component startup, visual odometry, navigation, and formal
  qualification were not run or claimed by this amendment.

## Installed component and exact configuration

- Installed package: `isaac_ros_visual_slam 4.5.0`; registered component:
  `nvidia::isaac_ros::visual_slam::VisualSlamNode` from
  `lib/libvisual_slam_node.so`.
- Launch: `robot_odometry/visual_odometry.launch.py`; parameters:
  `robot_odometry/config/cuvslam_rgbd_shadow.yaml`.
- Inputs: `visual_slam/image_0 -> /camera/front/image_raw`,
  `visual_slam/camera_info_0 -> /camera/front/camera_info`, and
  `visual_slam/depth_0 -> /camera/front/depth/image_raw`. There is no IMU
  remap or subscription in the RGB-D launch contract.
- Outputs: `visual_slam/tracking/odometry -> /visual/odom_shadow` and the
  installed official `visual_slam/status -> /visual/status`.
- Parameters: `use_sim_time=true`, `tracking_mode=2`, `num_cameras=1`,
  `min_num_images=1`, `depth_camera_id=0`, `depth_scale_factor=1.0`,
  `camera_optical_frames=[camera_front_optical_frame]`, `base_frame=base_link`,
  `odom_frame=visual_odom_shadow`, `map_frame=visual_map_shadow`,
  `enable_localization_n_mapping=false`, visualization/views false,
  `rectified_images=false`, `image_qos=SENSOR_DATA`, sync threshold `10 ms`,
  jitter threshold `20 ms`, both TF publication flags false, and
  `override_publishing_stamp=false`.
- The existing canonical Isaac camera remains unchanged: one aligned RGB-D
  Render Product, RGB8, shared `camera_front_optical_frame`, Best Effort /
  Volatile QoS, and the `rgbd_navigation` profile at 320x180 / 10 Hz. The
  actual live depth encoding and scale still require the bounded preflight;
  `depth_scale_factor=1.0` follows the Isaac ROS Bridge metre-depth contract,
  not a new live measurement.

## Default-OFF runner and recording

- `ros_stack.launch.py` has one boolean argument,
  `visual_odometry_shadow_enabled`, default `false`. The component launch is
  included only when it is `true`.
- The canonical wrapper/session uses
  `V6_VISUAL_ODOMETRY_SHADOW_ENABLED=false` by default and passes that one
  value to `ros_stack`. The next diagnostic sets it explicitly to `true`.
- When enabled, the existing R5 recorder additionally records RGB, raw depth,
  CameraInfo, `/visual/odom_shadow`, and `/visual/status`; its existing
  `/clock`, GT, wheel, EKF, corrected/raw IMU, `/tf`, and `/tf_static` topics
  remain. `/camera/front/depth/points` is deliberately not duplicated.

## Validation

- Focused visual/runner tests: **9 passed, 35 deselected**.
- Complete `robot_odometry` plus runtime-script suite: **97 passed**.
- Isolated `/opt/ros/jazzy`-only build under `/tmp/v6_cuvslam_shadow_*`:
  **2 packages finished** (`robot_odometry`, `robot_bringup`); the new launch
  and YAML were present in the install tree.
- Both modified shell scripts passed `bash -n`; both modified launch files
  passed Python compilation with bytecode redirected under `/tmp`.
- `ament_flake8` reported no problems for the new launch/contract test and
  modified `ros_stack`; `git diff --check` passed.
- No EKF YAML, wheel/IMU/guard, camera configuration/generation, scene, R2,
  map, GVG, Grid, Costmap, Nav2, Integration, or Module2 file was changed.

## Exact next R2 diagnostic command

After a fresh strict combined snapshot containing this commit is built and
domain 229 is confirmed empty, run one canonical R2 episode with the shadow
explicitly enabled:

```bash
cd /home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3
RUN_DIR="/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_grid_phase1_clearance_r2_cuvslam_shadow_$(date -u +%Y%m%dT%H%M%SZ)"
SNAPSHOT_ROOT="/absolute/path/to/fresh_combined_phase1_snapshot"
ISAAC_ASSET_ROOT=/home/lyb/isaacsim_assets/Assets/Isaac/6.0 \
V6_VISUAL_ODOMETRY_SHADOW_ENABLED=true \
R5_DOMAIN_ID=229 R5_EPISODE_INDICES=0 R5_EPISODE_SEEDS=7201 \
  ./scripts/run_v6_kujiale_low_obstacles.sh session "${RUN_DIR}" "${SNAPSHOT_ROOT}"
```

Before treating the shadow as ready, the reviewer must record the actual RGB
and depth encodings, dimensions/rates, RGB-depth-CameraInfo stamp deltas and
QoS, confirm bounded `/visual/odom_shadow` and `/visual/status` output, and
confirm the visual node publishes no `/tf` or `/tf_static`. A component/NITROS
failure, unsupported depth encoding/scale, or missing bounded output is a
STOP for the shadow only; it does not justify fusion or promotion.
