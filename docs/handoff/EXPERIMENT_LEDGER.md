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
