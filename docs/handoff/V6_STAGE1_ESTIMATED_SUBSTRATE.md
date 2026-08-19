# V6 Stage 1 Module3 estimated substrate handoff

Date: 2026-08-20

## Scope and result

- Goal: establish the first formal estimated-state substrate without adding CognitiveObstacleLayer or CognitiveRiskCritic.
- Branch/worktree: `cognitive-navigation` at `/home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3`.
- Implementation commit: `f4bc4eda5ffe059daef52cdd481bf45a41635b56`.
- Result: **PASS for implementation and independent package build/unit tests; ROS runtime and Isaac runtime remain unrun.**

## Implemented contracts

- Wheel odometry remains topic-only on `/wheel/odom`, uses the `JointState` message stamp, and does not publish TF.
- `estimated` is a supported ROS odometry mode; legacy `realistic` follows the same estimator/localization topology.
- EKF A/B profiles are `wheel_imu` and `wheel_imu_lidar`. Both publish the sole `/odom` and `odom->base_link`; the three-source profile adds differential LiDAR x/y/yaw from `/lidar/odom`.
- RF2O wiring is present with `publish_tf: false`. Selecting RF2O fails fast when `rf2o_laser_odometry` is unavailable; `off` remains the current usable setting.
- Localization/navigation use immutable `map_server` + AMCL + `lifecycle_manager_localization`. AMCL is the sole estimated `map->odom` owner. SLAM Toolbox remains only in mapping.
- Kujiale and Rivermark AMCL values are separate initial tuning profiles, not calibration claims.
- RouteCoordinator rejects unhealthy, empty-model, stale/future, non-finite, negative-cost, or out-of-range priors. Refresh requests have response deadlines; timeout/TTL expiry clears learned priors and re-applies geometry-only physical routing.

## Validation actually run

Clean commands used `env -i`, then sourced only `/opt/ros/jazzy/setup.bash`.

```bash
colcon build --packages-select robot_slam_solver robot_mapping robot_localization_config robot_odometry robot_route_planner --symlink-install
colcon test --packages-select robot_slam_solver robot_mapping robot_localization_config robot_odometry robot_route_planner
colcon test-result --verbose
```

Result: 5 packages built; `95 tests, 0 errors, 0 failures, 1 skipped`. The skip is the existing optional `pxr` benchmark test.

```bash
bash -n scripts/run_ros.sh
python3 -m pytest -q ros2_ws/src/robot_bringup/test isaac_sim/tests/test_graph_contracts.py
```

Result: `206 passed` using a task-local `ROS_LOG_DIR` under the same pure Jazzy environment.

## Explicit non-results and blockers

- A pure Jazzy full-workspace build stopped at `bio_nav_fusion`: `bio_nav_interfacesConfig.cmake` was unavailable. The allowed Integration worktree install did not yet exist, so no forbidden checkout install was sourced.
- `/opt/ros/jazzy` contains `robot_localization`, `nav2_amcl`, `nav2_map_server`, and `nav2_lifecycle_manager`, but no RF2O, laser_scan_matcher, MOLA LiDAR odometry, or mp2p ICP package. Therefore `/lidar/odom` has not run.
- No ROS nodes were launched. No TF/topic ownership was observed at runtime.
- Isaac Sim, Nav2 navigation, AMCL convergence, odometry accuracy, and scene calibration were not run or claimed.

## Next action

After the allowed Integration worktree exports `bio_nav_interfaces` and `bio_nav_ros_bridge`, source only that worktree install on top of Jazzy, rebuild the affected Module3 packages, then run a no-Isaac launch smoke. Install/build a Jazzy-compatible RF2O package before selecting `ekf_profile:=wheel_imu_lidar lidar_odometry_backend:=rf2o`.
