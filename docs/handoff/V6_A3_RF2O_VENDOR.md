# V6 A3 — fixed-revision RF2O vendor and shadow runtime

- Goal/hypothesis: replace the empty RF2O configuration contract with a
  locally built, fixed-revision Jazzy package while keeping LiDAR fusion in
  shadow until real-data accuracy is validated.
- Branch/worktree: `cognitive-navigation`;
  `/home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3`;
  parent `28a8a7a1d174bd71c8d8fe1b84ce03b5ad12ce02`; commit is the commit
  containing this handoff.
- Upstream: `https://github.com/MAPIRlab/rf2o_laser_odometry` at
  `b38c68e46387b98845ecbfeb6660292f967a00d3`, verified in a temporary clone
  before copying without `.git`; GPLv3 license, README, copyright headers and
  `UPSTREAM.md` retained.
- Main files: `ros2_ws/src/rf2o_laser_odometry/**`, RF2O YAML/contract/smoke
  under `robot_odometry`, validation gates in `robot_localization_config` and
  `robot_bringup`.
- Runtime contract: one ROS node; wait for valid `base_link <- scan` TF before
  initialization; publish only `/lidar/odom`, stamped from strictly increasing
  scans, with frames `odom -> base_link`; never publish `/tf`; finite positive
  parameterized pose/twist covariance.
- Shadow gate: `wheel_imu_lidar` fails closed unless
  `lidar_odometry_validated:=true` is explicitly supplied. Default remains
  false. The direct EKF negative launch test exited 1 with the intended gate
  message.
- Dependency/build commands: Jazzy plus the allowed Integration
  `install/local_setup.bash`; `rosdep install --from-paths
  src/rf2o_laser_odometry --ignore-src -r -y`; `colcon build
  --symlink-install --packages-up-to rf2o_laser_odometry robot_odometry
  robot_localization_config robot_bringup`.
- Build result: PASS, 14 dependency packages built; the final focused RF2O and
  robot_odometry rebuild also passed.
- Test result: PASS — `robot_odometry 18/18`,
  `robot_localization_config 14/14`, `robot_bringup 204/204`; RF2O launch and
  synthetic publisher flake8 PASS; RF2O package XML valid; `git diff --check`
  PASS.
- Synthetic ROS smoke: PASS on isolated `ROS_DOMAIN_ID=183`, no Isaac. Static
  `base_link -> laser` plus deterministic translated room scans and `/clock`
  produced 44 `/lidar/odom` messages: finite, nonzero, strictly monotonic
  stamps, positive covariance, correct frames. `ros2 node list -a` showed one
  `rf2o_laser_odometry_node` and no `transform_listener_impl`/algorithm node.
  `/tf` reported publisher count 0; smoke observed `dynamic_tf_count=0`.
  Reproducer: `ros2_ws/src/robot_odometry/test/rf2o_synthetic_smoke.py`.
- Available bags: none; the allowed worktree's `data/bags` contains only
  `.gitkeep`. No other worktree was searched.
- Verdict: **PASS (vendor/build/unit/synthetic ROS smoke)**.
- Remaining risk: no bag/Isaac ATE or RPE, no covariance calibration, and no
  real-sensor robustness evidence. This is not formal qualification; keep the
  validation gate false until a later bag/Isaac calibration task passes.
