# V6 FAST-LIO2 planar IMU discrimination replay (2026-08-24)

## Verdict

**PORT_ALGORITHM_BUG / KEEP_OFF.** Zeroing corrected IMU angular velocity x/y
did not eliminate or materially delay FAST-LIO2's catastrophic motion
divergence. This result rejects the single planar-IMU hypothesis; it does not
identify the specific port/core defect.

FAST-LIO2 remains default OFF, shadow-only, TF false, and excluded from EKF,
Grid, Nav2, safety, and control. No fusion, navigation, Isaac, or qualification
run occurred.

## Minimal implementation

- `robot_odometry` provides a default-off SensorData-QoS adapter from
  `/imu/data` to `/imu/lio`. It validates a nonzero valid stamp and finite IMU
  payload, deep-copies the message, and sets only angular velocity x/y to exact
  zero. Header/frame, orientation, acceleration xyz, corrected gyro z, and all
  covariances are preserved. Invalid messages are dropped with first/periodic
  bounded counters.
- `fast_lio2_ros2 shadow.launch.py` uses `/imu/lio` only when
  `planar_imu_enabled:=true`. Its default remains false and the existing
  `/imu/data` behavior remains unchanged. The canonical EKF configs still use
  `/imu/data`.
- `h_share_model` retains all existing thresholds and behavior, but replaces
  unbounded `stderr` spam with one-second aggregate warnings containing scan
  begin/end stamps and nearest-neighbor, plane-fit, residual, and unclassified
  rejection counts.

## Exact isolated replay

- Input MCAP:
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_fastlio2_ouster_g2_retry_20260824T035347Z/bag/fastlio_shadow`
- Fresh ROS domain: 228.
- Start offset: 399.0 s. The original bag's first recorded
  `/lio/odom_shadow` was at offset 399.970925135 s, so this reproduces the same
  pre-first-output initialization segment rather than starting mid-motion.
- Published from the MCAP only: `/lio/points_raw`, `/imu/data`, `/clock`.
- Actual launch: `enabled:=true planar_imu_enabled:=true
  output_odom_topic:=/lio/odom_planar_shadow`.
- Recorded only `/lio/odom_planar_shadow`; no Nav2, Isaac, control, TF, or
  other config change. Temporary output was
  `/tmp/fastlio_planar_replay.VrNbNG`.
- Adapter diagnostics reached at least 13,000 accepted and zero invalid input
  samples.

Header-stamp comparison used motion start 468.112098237 s and collision
494.533333333 s:

| Metric | Baseline | Planar replay |
|---|---:|---:|
| 0.5 m XY crossing | 1.472 s | 1.572 s |
| 1 m XY crossing | 2.472 s | 2.572 s |
| 5 m XY crossing | 10.772 s | 10.672 s |
| 100 m XY crossing | 16.672 s | 16.572 s |
| collision-window final XY error vs EKF | about 510.7 m | 487.5 m |
| collision-window final 3D error vs EKF | not reported | 506.4 m |

The nearest pre-collision planar sample had relative xyz
`[-434.05, 205.34, -137.08] m`; matched EKF was
`[8.47, 0.75, 0.00] m`. Route RMSE was 184.62 m XY and 191.10 m 3D.
Wheel comparison was equivalently catastrophic: 485.84 m endpoint XY error.

Output diagnostics:

- 1,885 odometry messages, header stamps 301.084176620--522.584176620 s;
- median 10 Hz; route 10 Hz, 11 gaps over 0.2 s, maximum 0.4 s;
- route 3D jump P50/P95/max 1.66/5.73/10.68 m, 132 jumps over 1 m;
- stationary 167.0 s endpoint drift 0.070 m XY and +0.194 m Z;
- first timestamped `No Effective Points` was scan begin 514.316213989 s,
  after the collision stamp; eight emitted aggregate buckets contained 166
  invalid-update callbacks, overwhelmingly nearest-neighbor rejection. The
  baseline's 315 unbounded lines had no scan stamps, so the totals are not a
  like-for-like scan count. The late timing shows rejection is an escalation
  symptom here, not the cause of the early crossing.

## Validation and next action

- Isolated `/tmp` build: rf2o dependency, `robot_odometry`, and
  `fast_lio2_ros2` finished.
- Installed package tests: 87 tests, zero failures/errors/skips (80
  `robot_odometry`, 7 `fast_lio2_ros2`).
- Focused adapter/remap/EKF suite: 17 passed.
- New Python files pass flake8; Python compilation and `git diff --check`
  pass. Existing PCL/Boost build warnings are unchanged and non-blocking.

Next action: keep the adapter as default-off experimental tooling and inspect
the port/core state/update path. Do not tune time, extrinsic, voxel, LiDAR,
IMU calibration, or navigation from this result, and do not promote LIO.
