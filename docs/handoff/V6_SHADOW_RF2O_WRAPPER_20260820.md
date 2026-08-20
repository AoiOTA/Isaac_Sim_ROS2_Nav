# V6 shadow wrapper RF2O default

- Goal: make the V6 `shadow` wrapper start RF2O as topic-only shadow while
  retaining wheel+IMU as the default EKF input set.
- Branch/worktree/start: `cognitive-navigation`;
  `/home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3`;
  `1438caca4362205650d54a58022924f6073ecc48`.
- Change: `shadow` now supplies `ekf_profile:=wheel_imu`,
  `lidar_odometry_backend:=rf2o`, and
  `lidar_odometry_validated:=false` before caller arguments. Explicit trailing
  overrides remain last; the existing loaded-EKF fail-closed gate remains the
  authority for any LiDAR-consuming EKF configuration.
- Isolation: `ros`, M0--M3 selection, `ros-d`, and legacy wrappers retain their
  prior implicit odometry defaults.
- Validation: `bash -n scripts/run_v6_kujiale_low_obstacles.sh` PASS; focused
  argv contracts `4 passed`; full `test_runtime_scripts.py` `30 passed`.
- Verdict: **PASS (shell syntax and isolated argv contracts only)**.
- Unrun/risk: no ROS, Isaac, TF/topic observation, navigation, metrics, or
  formal qualification was run. RF2O remains unvalidated for EKF fusion.

