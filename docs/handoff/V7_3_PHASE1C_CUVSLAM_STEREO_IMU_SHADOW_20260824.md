# V7.3 Phase 1C cuVSLAM stereo + IMU shadow handoff (2026-08-24)

## Result and boundary

- Added an opt-in, independently direct-launchable Isaac ROS Visual SLAM 4.5.0
  stereo + IMU shadow configuration. It does not modify or replace the legacy
  RGB-D shadow launch.
- The component consumes the Phase1A left/right image and CameraInfo topics and
  the Phase1B `/imu/vio` stream. It publishes only `/visual/odom_shadow` and
  `/visual/status`; both cuVSLAM TF publishers are disabled.
- This amendment does not connect `ros_stack`, a runner, reset orchestration,
  `VioOdomAdapter`, canonical `/odom`, Integration, or Module2.
- Validation is code/static/build only. Isaac, ROS live, cuVSLAM tracking, IMU
  fusion, output cadence/gaps, status behavior, and reset behavior were not run
  or verified. The Phase1B observed RTF `0.64053` remains a warning to measure
  with the actual cuVSLAM consumer.

## Direct shadow contract

```text
/camera/left/image_raw + /camera/left/camera_info
/camera/right/image_raw + /camera/right/camera_info
/imu/vio
  -> nvidia::isaac_ros::visual_slam::VisualSlamNode (tracking_mode=1)
  -> /visual/odom_shadow + /visual/status
```

The node name is `visual_slam_stereo_imu_shadow`; the container name is
`visual_odometry_stereo_imu_shadow_container`. `tracking_mode: 1` is the
installed 4.5.0 VIO mode. That release does not expose an
`enable_imu_fusion` parameter, so none is configured. No depth input or
converter is present.

The installed official reset endpoint is `/visual_slam/reset` with type
`isaac_ros_visual_slam_interfaces/srv/Reset`. Static interface discovery is
covered, but service presence and successful reset are live-unverified.

## Validation and next live review

Focused contract tests, Python compilation, an isolated `robot_odometry`
build under `/tmp`, and `git diff --check` are the only validations for this
amendment. No live process was started.

- Source-first/no-cache focused contract test: `10 passed`.
- Python compilation: passed.
- Isolated package build: one `robot_odometry` package passed at
  `/tmp/v73_phase1c_robot_odometry.EQkgZ7`; the installed share contains the
  new YAML and launch file. Colcon emitted the existing underlay override
  warning; this Python package installs no headers and the build completed.
- Diff check: passed.

For the first bounded live review, use a freshly confirmed empty ROS domain no
higher than 232 (for example 231) and apply the committed 4 MiB UDP-only Fast
DDS profile to both producer and consumer:

```bash
export ROS_DOMAIN_ID=231
export ISAAC_NAV_FASTDDS_PROFILE="$PWD/isaac_sim/configs/ros2_bridge/fastdds_udp_only.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="$ISAAC_NAV_FASTDDS_PROFILE"
export FASTDDS_DEFAULT_PROFILES_FILE="$ISAAC_NAV_FASTDDS_PROFILE"
ros2 launch robot_odometry visual_odometry_stereo_imu_shadow.launch.py
```

The live reviewer must establish tracker initialization, actual IMU fusion,
odom/status rate and gaps, absence of VIO TF, RTF/GPU load, and successful
reset through the official service before any Phase1C live PASS claim.

## First bounded engineering live review

- Run: Module3 `6b950b3d85a2eb24e52d2ec690cd1d19922da1fd`, domain
  231, realistic Kujiale `stereo_vio`, 120/60 Hz physics/render, the 4 MiB
  UDP-only Fast DDS profile, both IMU calibrators, wheel odometry, EKF, and
  only the direct cuVSLAM shadow launch. Evidence is under
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v73_phase1c_cuvslam_20260824T125409Z`.
  Nav2, Integration, Module2, route goals, bags, and canonical-odom cutover
  were absent. The isolated launch/YAML matched this HEAD before startup.
- Tracker/IMU: the live log reported `Tracking mode: VIO (IMU fusion)`,
  `Enable IMU Fusion: true`, and successful tracker initialization both at
  initial startup and after reset. There was no parameter, image-format,
  CUDA, fatal, NaN, or initialization error.
- Outputs: the retained 180.000 s observer received 2284 odometry and 2284
  status samples across 114.15 s simulation time. Both streams were exactly
  20.0 Hz with strictly monotonic unique stamps; p95/max simulation gaps were
  `50.0/50.000998 ms`, with zero gaps at or above 150 ms. Wall-receive p95/max
  gaps were about `88.3/204.2 ms`, with no gap above 0.5 s. Every status was
  `vo_state=1`; failure and nonfinite counts were zero. Maximum single-frame
  planar/yaw increments were `0.006402 m` and `0.013189 rad`, with no
  discontinuity-sized jump.
- Reset: the actual endpoint/type was exactly `/visual_slam/reset` and
  `isaac_ros_visual_slam_interfaces/srv/Reset`. The empty request returned
  `success=True` in 717 ms. The last observer samples before the call had
  odom/status stamp `231.75 s`; after the response, the first samples with
  strictly newer stamp were odom/status at `232.25 s`, with status healthy,
  received `11.4/11.8 ms` after the response. No pre-reset stamp was counted
  as recovery.
- Inputs: all four stereo image/CameraInfo streams supplied the same 2290
  stamps with zero missing slots at exactly 20 Hz. `/imu/vio` supplied 13824
  unique monotonic samples at `120.00002 Hz`; each camera interval contained
  mean/median `6.0004/6` IMU samples. Resolved publisher and cuVSLAM subscriber
  QoS was BEST_EFFORT/VOLATILE for all camera inputs and `/imu/vio`;
  `/visual/odom_shadow` and `/visual/status` publishers were
  RELIABLE/VOLATILE.
- Isolation/ownership: `/odom` retained one publisher, `ekf_filter_node`.
  cuVSLAM instantiated `/tf` and `/tf_static` publisher endpoints even with
  both TF flags false, but all three observer windows contained only the
  existing wheel, `odom->base_link`, robot sensor, and RTX transforms. No
  `visual_odom_shadow` or `visual_map_shadow` transform was emitted.
- Motion evidence integrity: a first BEST_EFFORT commander was incompatible
  with Isaac's RELIABLE `/cmd_vel_sim` subscriptions and is excluded as an
  operator-invalid sample. One RELIABLE replacement published all requested
  commands with collision false. The retained observer fully covered both
  yaw commands and saw visual yaw deltas `+0.02237/-0.13039 rad`, consistent
  in sign with wheel `+0.05129/-0.31303` and EKF
  `+0.03360/-0.14214`. Scheduling left only 1.89 wall seconds (1.15 s
  simulation time) of the 3 s straight command inside that observer. Wheel
  and EKF forward displacement was positive (`0.01627/0.01606 m`), while the
  visual forward projection was `-0.01509 m` amid `0.03052 m` planar visual
  motion. Therefore no straight-direction or accuracy claim is made. A
  separate authorized 10 s cleanup observer received 100 explicit zeros,
  zero nonzero commands, 103 false collision samples, all 129 visual statuses
  healthy, and only `0.000002/0.001247/0.001244 m` visual/wheel/EKF planar
  displacement.
- Load/transport: overall RTF was `0.640646`. This remains below the plan's
  0.8 recommendation but did not produce tracker loss or native output gaps.
  The 190-s GPU probe measured 45.68% mean/56% max utilization and
  6017/6098 MiB mean/max memory. Observer-bracket host used memory increased
  by 62 MiB. Host-wide UDP `InErrors` and `RcvbufErrors` each increased by 21,
  although camera pairing and output-gap checks showed no payload loss;
  attribution to this domain is not established. The 202 log warnings were
  numerical `50.000001-50.000998 ms` comparisons against a 50 ms threshold at
  the exact 20 Hz cadence.
- One later full odometry sample had nonzero but extremely small covariance
  (roughly `1e-15` to `1e-14` scale). Treat covariance credibility as a route
  pilot question rather than as demonstrated uncertainty quality here.
- Cleanup: all owned process groups stopped, domain 231 was empty, both Isaac
  locks were free, and preserved domain-141 PID 3600069 remained alive.
- Verdict: **PHASE 1C INITIAL ENGINEERING PASS WITH WARNINGS; NOT FORMAL
  QUALIFICATION AND NOT VIO PROMOTION**. Tracker, IMU fusion, reset, native
  cadence/gaps, finite output, continuity, collision safety, and shadow
  isolation passed. Warnings are RTF, host-wide UDP errors, the excluded
  commander, partial straight overlap with unmatched visual forward sign,
  separate final-zero evidence, and tiny covariance. Next run is the planned
  same-route G1-to-G2 shadow pilot for pivot translation and route error; do
  not cut over `/odom` or TF yet.
