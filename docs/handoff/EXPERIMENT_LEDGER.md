# Experiment ledger

## 2026-08-20 — V6 Stage 1 Module3 estimated substrate

- Hypothesis: a minimal Wheel+IMU EKF and AMCL chain can be made buildable now, while the unavailable LiDAR odometry backend remains an explicit fail-fast option and Module2 priors fail open on stale/unhealthy refresh.
- Branch/worktree/commit: `cognitive-navigation`; `/home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3`; implementation `f4bc4eda5ffe059daef52cdd481bf45a41635b56`.
- Main files: `robot_bringup/launch/ros_stack.launch.py`, `robot_mapping/launch/localization.launch.py`, scene AMCL YAML, EKF A/B YAML/launch, RF2O YAML/launch, `robot_route_planner/ros_node.py`, and focused tests.
- Configuration: `odometry_mode:=estimated`, default `ekf_profile:=wheel_imu`, default `lidar_odometry_backend:=off`, `localization_profile:=kujiale|rivermark`, prior TTL default 2.0 s.
- Commands: pure Jazzy selective `colcon build`; selective `colcon test`; `colcon test-result --verbose`; `bash -n scripts/run_ros.sh`; focused bringup/TF pytest.
- Core result: independent build PASS for 5 packages; `95 tests, 0 failures, 1 skipped`; focused bringup/TF `206 passed`.
- Evidence: local `ros2_ws/build/*/test_results` and `ros2_ws/log`; generated build artifacts are not committed.
- Verdict: **PASS (implementation/build/unit only)**.
- Blocker: full Module3 build requires `bio_nav_interfaces` from the allowed Integration worktree install, which was absent. RF2O is absent from `/opt/ros/jazzy`.
- Unrun: ROS runtime, TF/topic observation, Isaac runtime, AMCL convergence, Nav2 navigation, odometry metrics, scene calibration, and formal qualification.
- Next: rebuild with the allowed Integration overlay when ready; then no-Isaac launch smoke, followed by actual RF2O installation and runtime calibration.

## 2026-08-20 — V6 Stage 1 prior lifecycle and legacy Ideal wrappers

- Hypothesis: consuming the pending prior window on timeout and binding each
  response to the live request/graph/refresh/model identity prevents late
  learned costs from re-entering geometry-only routing; explicit Ideal launch
  arguments prevent the new AMCL default from changing legacy wrappers.
- Branch/worktree/commit: `cognitive-navigation`;
  `/home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3`;
  commit recorded by the task handoff after submission.
- Commands: focused source pytest; pure Jazzy plus allowed Integration
  `local_setup.bash` `colcon build --packages-up-to robot_route_planner
  robot_bringup`; focused colcon route/bringup tests; new launch-test flake8;
  `git diff --check`.
- Core result: `72 passed, 1 skipped` source tests; 13-package build PASS;
  focused colcon tests `27 passed` route plus `21 passed` bringup.
- Verdict: **PASS (implementation/build/unit and offline launch expansion)**.
- Unrun: ROS/Isaac/Nav2 runtime, 8-second no-Isaac launch smoke, and formal
  qualification.
- Warning: the unrelated full-suite frozen-JSON absolute-path assertion is
  checkout-sensitive and was deliberately left untouched.
- Next: reviewer may run the bounded no-Isaac launch smoke from the built
  overlay, then proceed to runtime localization/navigation validation.

## 2026-08-20 — V6 A3 fixed-revision RF2O shadow runtime

- Hypothesis: vendoring a fixed official RF2O revision with a true single-node
  Jazzy wrapper can provide usable topic-only LiDAR odometry without changing
  TF ownership or prematurely enabling three-source fusion.
- Branch/worktree/commit: `cognitive-navigation`;
  `/home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3`;
  commit containing `docs/handoff/V6_A3_RF2O_VENDOR.md`.
- Upstream/configuration: MAPIRlab RF2O commit `b38c68e46387b98845ecbfeb6660292f967a00d3`;
  `/lidar/odom`; `odom -> base_link`; `publish_tf:=false`; conservative
  parameterized covariance; `lidar_odometry_validated:=false` by default.
- Commands: fixed-SHA temporary clone verification; focused `rosdep
  --ignore-src`; Jazzy + allowed Integration selective colcon build/test;
  direct validation-gate negative launch; isolated-domain static-TF plus
  deterministic LaserScan/clock smoke; flake8/xmllint/diff check.
- Core result: build PASS; tests `18 + 14 + 204` passed; gate negative test
  exited 1 as intended; synthetic smoke produced 44 finite, nonzero,
  stamp-monotonic messages with positive covariance and correct frames; exactly
  one RF2O node, no hidden listener/algorithm node, and zero `/tf` publishers.
- Evidence/reproducer: `docs/handoff/V6_A3_RF2O_VENDOR.md` and
  `ros2_ws/src/robot_odometry/test/rf2o_synthetic_smoke.py`; generated build
  artifacts remain under `ros2_ws/build`, `install`, and `log` and are not
  committed.
- Verdict: **PASS (vendor/build/unit/synthetic ROS smoke)**.
- Deferred: bag/Isaac ATE/RPE, covariance calibration, real-sensor robustness,
  and formal qualification. No usable bag exists in this allowed worktree, so
  three-source fusion remains fail-closed shadow.

## 2026-08-20 — V6 A3 reviewer gate and motion amendment

- Goal: bind LiDAR validation to the final loaded EKF YAML rather than the
  profile label, and prevent constant quaternion w from satisfying synthetic
  motion evidence.
- Changes: shared fail-closed EKF input classifier used by direct EKF and core
  bringup; relative x/y/yaw plus planar-twist `motion_detected`; explicit
  stationary and moving smoke modes.
- Build/test: pure Jazzy + allowed Integration build PASS (14 packages);
  `robot_localization_config` 18 total/0 failed, `robot_odometry` 19/19, and
  `robot_bringup` 204/204.
- Actual launch matrix: profile `wheel_imu` plus custom three-source YAML and
  validation false rejected RC=1; the same YAML with true started; profile
  `wheel_imu_lidar` plus actual canonical wheel-only YAML and false started.
- Synthetic smoke: stationary repeated scans PASS with
  `motion_detected=false`, one valid odometry sample before RF2O rejected the
  degenerate identical pairs; translated room scans PASS with 68 valid samples
  and `motion_detected=true`. Both retained finite data, positive covariance,
  correct frames, monotonic stamps and zero dynamic TF publication.
- Verdict: **PASS (reviewer amendment build/unit/launch/synthetic smoke)**.
- Deferred unchanged: Isaac/bag ATE/RPE, covariance calibration, and formal
  qualification.

## 2026-08-20 — V6 C4 Kujiale frozen low-obstacle layout

- Goal: freeze a separate six-obstacle Kujiale layout without editing the USD,
  map, existing draft layout, or existing static/dynamic campaign defaults.
- Branch/worktree/start: `cognitive-navigation`;
  `/home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3`;
  `f81c690f2064a5711ecfe8ea68edcabc3e41915e`.
- Layout: `kujiale_v6_low_obstacles_frozen_r1_20260820`; five original centers
  retained; east bar x minimally adjusted to 1.300000 m to raise the nearest
  pair clearance above the 0.585 m robot-plus-margin contract.
- Profile: explicit `run_v6_kujiale_low_obstacles.sh` and additive
  `v6-low-obstacles` Isaac mode; default-off physical YAML; dedicated
  experiment scenario; dedicated Nav2 isolation overlay.
- Sensor contract: RGB, depth image, and depth points remain published under
  `rgbd_navigation`; raw depth points are absent from the V6 Costmap overlay.
- Commands: wrapper `bash -n`; focused pytest covering new layout, obstacle
  parser regression, scenario parsing, and Nav2 profile timing; `git diff
  --check`.
- Result: `103 passed`; no static geometry/contract failure.
- Verdict: **PASS (frozen configuration and static tests only)**.
- Unrun: Isaac/ROS/Nav2, camera visibility, contact, bypass behavior, estimated
  localization, causal experiment, and formal qualification.
- Handoff: `docs/handoff/V6_C4_LOW_OBSTACLE_LAYOUT.md`.
