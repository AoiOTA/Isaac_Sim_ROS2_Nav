# V6 Estimated State calibration runner/evaluator handoff

Date: 2026-08-21

## Scope and result

- Worktree/branch: permitted Module3 `cognitive-navigation` worktree, starting
  from `055bdeafd95d7335f7f8a07683ab759376174c7f`.
- Result: **PASS (implementation/build/unit only)**.
- Live state: **CALIBRATION_NOT_RUN**. No ROS, Isaac, Nav2, 45-row matrix, or
  RF2O promotion was run; no tuning or qualification result is claimed.

## Implemented contract

- Exact 45 episodes, grouped by arm to allow a single Isaac session:
  `off` 15, `shadow` 15, `fused` 15.
- Per arm: 3 repeats each of 3 m straight (`0.30 m/s`, `10 s`), CCW/CW 360
  (`+/-0.50 rad/s`, `12.566 s`), the three-segment S route, and Rivermark
  static start to G1 (`1.521014, 131.813786, 135 deg`) through the current
  occupancy-only estimated Nav2 wrapper.
- Each episode specifies exactly one reset and zero retries. The dispatcher
  topics exclude `/ground_truth/*`; only the passive evaluator subscribes to
  `/ground_truth/odom`.
- Arm arguments are frozen as `wheel_imu/off/false`,
  `wheel_imu/rf2o/false`, and `wheel_imu_lidar/rf2o/true`.
- Fused dispatch fails closed unless all 15 shadow reports pass timestamp,
  jump, frequency, bounded offset, scale, finite/symmetric/PSD covariance
  checks and the operator also supplies the explicit promotion flag. Setting
  `lidar_odom_validated:=true` is recorded as configuration, not evidence.
- The present `run` command has no runtime adapter and writes `NOT_RUN`; this
  prevents a generated manifest from being mistaken for calibration evidence.

## Metrics and outputs

- Both absolute (no first-frame alignment) and first-frame SE(2)-aligned ATE.
- Fixed 1 s and 1 m RPE, endpoint longitudinal/lateral/position/yaw error,
  linear/yaw scale, signed yaw bias and CW/CCW asymmetry.
- Bounded best estimate-to-GT time offset, frequency, duplicate/backward
  timestamps and pose/yaw jumps.
- Finite/symmetric/PSD planar covariance diagnostics, per-axis 2-sigma
  coverage, and planar NEES computed from GT error and reported covariance.
  NIS is explicitly `NOT_AVAILABLE` because innovations are not recorded.
- JSON/CSV matrix and evaluation outputs plus plot-input manifest; aggregates
  are grouped by arm and scenario/run.
- Plan reference thresholds and engineering recommendations are distinct;
  covariance coverage is diagnostic only. Fused requires a clear improvement
  and no regression against off/shadow before it can be accepted.

## Entry points

```bash
scripts/run_v6_estimated_calibration.sh manifest --output-dir /tmp/v6_calib
scripts/run_v6_estimated_calibration.sh plan --output-dir /tmp/v6_calib
scripts/run_v6_estimated_calibration.sh evaluate \
  --results-root /path/to/episode_outputs --output-dir /tmp/v6_calib
```

## Validation

- Python compilation: PASS for the four changed/new Python modules.
- Focused metrics/motion/package/calibration tests: `59 passed`.
- Exact manifest smoke: `45` rows and arm grouping PASS.
- `robot_experiments` isolated build: PASS at
  `/tmp/v6_estimated_calibration_build.qNYxfL`.
- `git diff --check`: PASS.
- Full `robot_experiments/test` collection was not interpretable in the active
  shell because an external stale Integration overlay lacks
  `bio_nav_interfaces.msg.CanonicalRoute`; the task did not modify that
  concurrent Integration worktree.

## Next runtime step

Implement or connect the real episode adapter, run the off and shadow groups,
calibrate wheel geometry/IMU/EKF from measured scale/bias/timing, then evaluate
the shadow gate. Only after the gate passes should an explicit fused promotion
permit the fused group. Rivermark follows through Nav2, not a cmd_vel primitive.
