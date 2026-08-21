# V6 IMU raw/corrected calibration amendment

> Later evidence boundary (2026-08-21): `0.9294` was subsequently validated on
> three CW plus three CCW flat20 pure rotations: corrected IMU and EKF yaw
> scales were approximately `0.999--1.002`, with closure below `0.63 deg`.
> A later Kujiale mixed-motion route nevertheless failed IMU non-degradation.
> This is regime dependence, not a retraction of the rotation result. A global
> `0.9814` replacement is rejected; see
> `V6_IMU_REGIME_DEPENDENCE_20260821.md`.

## Scope

- Worktree/branch: permitted Module3 `cognitive-navigation` worktree.
- Parent: `af5c3d4b618edfb97936e85b00a7d489d45a98bb`.
- Goal: preserve raw IMU audit evidence while applying the flat-arena yaw-rate
  scale before the EKF. This task did not change motion assist, routing,
  PRIMARY authority, formal runners, Integration, or Module2.

## Implemented contract

- Isaac publishes the unchanged sensor sample on `/imu/data_raw` with its
  original simulation stamp and `imu_link` frame.
- Exactly one estimated/realistic bringup node subscribes with sensor-data QoS
  and publishes `/imu/data`. The corrected yaw rate is
  `(raw_z - yaw_bias_rad_s) * yaw_scale`.
- The V6 default is `yaw_scale=0.9294`, `yaw_bias_rad_s=0.0`, and conservative
  `angular_velocity_covariance[8]=1.0e-4`. All other header, orientation,
  angular-velocity, linear-acceleration, and covariance fields are copied.
- Non-finite, duplicate, backward, and zero-stamp samples are rejected. An
  explicit `/simulation/reset_event` clears only the stamp boundary.
- Scale is bounded to `[0.5, 1.5]`; bias must be finite; variance must be finite
  and positive. Missing raw input produces no corrected output.
- `imu_calibration_identity.yaml` provides an explicit `yaw_scale=1.0`
  rollback/non-Isaac profile while retaining the auditable topic split.
- The EKF input remains `/imu/data`. The passive evaluator now records and
  integrates `/imu/data_raw` and `/imu/data` separately, including separate
  yaw scale/bias results and angular-z covariance diagnostics.

## Validation

- Changed Python compilation: PASS.
- Focused source contracts: `24 passed`.
- Related source regression after the authorized shell strict-mode repair:
  `314 passed`.
- New odometry files `ament_flake8`: PASS, no problems.
- Isolated build at `/tmp/v6_imu_calibration_build.6H1KpP`: `16 packages`
  finished; installed calibrator executable and both YAML profiles confirmed.
- Isolated package tests: `robot_odometry 37/37`, `robot_bringup 224/224`, and
  `robot_experiments 416/417`. The sole experiments failure is the pre-existing
  checkout-sensitive frozen JSON absolute-path comparison; all Estimated/IMU
  tests passed.
- `bash -n scripts/run_v6_estimated_calibration.sh`: PASS.
- `git diff --check`: PASS.

## Verdict and remaining runtime work

**PASS (implementation/build/unit only).** No ROS, Isaac, Nav2, calibration,
PRIMARY, evidence, or qualification campaign was launched. A fresh live flat
run must still confirm one raw publisher, one corrected publisher, zero
duplicate corrected stamps, the expected 0.9294 multiplier, covariance, and
improved EKF yaw scale before accepting the tuning result or rerunning affected
PRIMARY navigation evidence.
