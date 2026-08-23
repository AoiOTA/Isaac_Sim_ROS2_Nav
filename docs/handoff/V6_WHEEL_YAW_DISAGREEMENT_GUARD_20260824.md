# V6 wheel yaw-disagreement guard candidate C (2026-08-24)

## Verdict and boundary

Candidate C is implemented and reproduced by exact MCAP replay, but remains
**default OFF**. This is code, unit, build, and replay evidence only. It is not
a canonical Phase-1 default, an Isaac/Nav2/control live run, navigation
success, recurrence evidence, or formal qualification.

## Bounded implementation

- Inputs when enabled: raw wheel-derived `vx/wz` plus corrected `/imu/data`
  `angular_velocity.z` and header stamp. Ground Truth and command topics are
  not inputs.
- Entry: wheel and IMU yaw rates have opposite signs and both magnitudes are
  at least `0.10 rad/s` for three consecutive stamped joint samples.
- Active: clamp signed wheel `|vx|` to `0.05 m/s` for integration and
  `/wheel/odom` linear velocity; wheel angular velocity is unchanged.
- Exit: three consecutive sign-agree, unusable-IMU, or
  `min(|wheel wz|, |IMU wz|) <= 0.02 rad/s` samples. Missing, stale, future, or
  non-finite IMU is immediately fail-open for that sample. Reset clears the
  detector and IMU cache.
- The only adjustable parameters are
  `yaw_disagreement_guard_enabled=false`,
  `yaw_disagreement_entry_threshold=0.10`, and
  `yaw_disagreement_imu_timeout=0.05`. Confirmation/clear count `3`, exit
  ratio `0.2`, and clamp `0.05 m/s` are fixed.
- With the guard OFF, no `/imu/data` subscription is created. Existing pose,
  twist, covariance, stamps, reset behavior, and no-TF ownership remain the
  baseline path.

## Validation

- Focused robot-odometry + EKF + launch tests: **60 passed** with pytest cache
  and bytecode disabled. The EKF regression still selects only wheel `vx`
  and corrected IMU `wz`.
- Modified Python lint: **7 files checked, no problems**.
- Isolated `/tmp/yaw_guard_build.FG5yGF` build: `rf2o_laser_odometry` dependency
  plus `robot_odometry`, **2 packages finished**. RF2O was built only as the
  package dependency and was not launched.
- Default-OFF installed-node probe on empty domain 226 showed subscribers only
  for `/clock`, `/joint_states`, and `/simulation/reset_event`; no `/imu/data`
  or `/tf` endpoint existed.

Exact replay used the failed bag
`/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_grid_phase1_20260823T174635Z/rosbag/r5_session/r5_session_0.mcap`
on initially empty domain 225. Only `/clock`, `/joint_states`, corrected
`/imu/data`, and `/simulation/reset_event` were replayed at 1x. The enabled
actual node output was remapped to `/yaw_guard/wheel_odom` and recorded at:

```text
/tmp/yaw_guard_replay_exact_20260824T0437/guard_output/guard_output_0.mcap
/tmp/yaw_guard_replay_exact_20260824T0437/replay_metrics.json
```

Replay result:

- `4626/4626` outputs, all unique and in exact JointState-header-stamp order;
  zero missing or extra stamps.
- One 83-sample state episode, `56.666666666–58.033333333 s`; zero full-bag
  output mismatches versus offline candidate C and no unintended route change.
- Pivot absolute forward integral `0.14653831432307307 m`, matching offline C
  `0.1465383143230731 m`; baseline was `0.5206281441032308 m`.
- Straight absolute-`vx` scale versus GT `0.9982402680931219`, exactly matching
  offline C. GT is scoring only.
- Maximum angular-output difference versus raw wheel yaw rate `0`; on 4,542
  overlapping recorded baseline samples, angular velocity, off-guard linear
  velocity, and both covariance arrays matched exactly.

## Opt-in short diagnostic

Use a dedicated, confirmed-empty ROS domain and do not run another wheel
odometry publisher in that domain:

```bash
source /opt/ros/jazzy/setup.bash
source /absolute/path/to/fresh_module3_install/setup.bash
ROS_DOMAIN_ID=225 ROS_LOCALHOST_ONLY=1 \
  ros2 launch robot_odometry wheel_odometry.launch.py \
  use_sim_time:=true yaw_disagreement_guard_enabled:=true
```

For an existing `robot_bringup/ros_stack.launch.py` diagnostic invocation, the
single equivalent override is `yaw_disagreement_guard_enabled:=true`. No
canonical runner or Phase-1 default was changed.
