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
