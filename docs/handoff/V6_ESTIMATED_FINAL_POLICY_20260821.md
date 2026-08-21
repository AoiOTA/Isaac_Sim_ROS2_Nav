# V6 Estimated final policy freeze

Date: 2026-08-21

> Later evidence boundary (2026-08-21): the flat20 result remains valid for
> three CW plus three CCW pure rotations (`0.9294` corrected IMU/EKF scale near
> `1.0`, closure below `0.63 deg`), and S-route met the reference in two of
> three runs. A later Kujiale mixed-motion route failed IMU non-degradation.
> A global `0.9814` replacement is rejected because it conflicts with the
> rotation-valid range. The wheel+IMU architecture and RF2O-off decision remain
> unchanged; see `V6_IMU_REGIME_DEPENDENCE_20260821.md`.

## Result

- Worktree/branch/start: permitted Module3 `cognitive-navigation` worktree at
  `4b57893bfbf430aead98e5c19d1c445149c54d55`.
- Verdict: **PASS (implementation/build/unit only)**.
- No ROS, Isaac, Nav2, navigation, evidence, or qualification campaign was
  launched by this amendment.

## Frozen policy

Calibration evidence `/tmp/v6_imu_calibration_live.VAf50R` supports the V6
final Estimated policy `wheel + calibrated IMU`. Straight and S-route wheel
scale were approximately `1.130--1.137` and `1.122--1.133`; corrected rotation
was acceptable. RF2O shadow was about 10 Hz, below the 15 Hz promotion floor,
and produced no fused rows. RF2O promotion therefore remains **BLOCKED**.

All six V6 final manifests now require:

- `ekf_profile: wheel_imu`;
- calibrated Isaac V6 IMU profile;
- `lidar_odometry_backend: off`;
- `lidar_odometry_validated: false`;
- `rf2o_decision: not_validated_off`.

Formal preflight rejects a final manifest that selects RF2O shadow or fused
operation. A pilot may use the explicit engineering override, but remains
`NOT_QUALIFIED`. The Rivermark and Kujiale PRIMARY wrappers pass the frozen
policy explicitly. The existing Kujiale `shadow` entrypoint remains available
for topic-only RF2O engineering diagnosis and does not fuse RF2O.

## Shutdown repair

`wheel_odometry_node` now stops accepting callbacks before ROS entities are
destroyed and checks context validity before publishing. Only a shutdown-time
`RCLError` is suppressed and counted; an `RCLError` while the context is live
is still raised. Normal integration and duplicate/backward stamp counters are
unchanged.

## Validation

- Changed Python compilation, shell syntax, and `git diff --check`: PASS.
- Focused manifest/policy, wrapper-argv, and wheel tests: `98 passed`.
- Isolated build: `robot_odometry`, `robot_experiments`, and `robot_bringup`
  PASS at `/dev/shm/v6_final_policy_build.kx9nXV`; warnings only reported
  unchanged packages supplied by the existing Module3 underlay.

## Remaining risk

This freezes the selected policy but does not create new runtime or formal
evidence. Rivermark/PRIMARY and later qualification runs must use this policy;
RF2O must remain off unless a new engineering calibration satisfies its
promotion contract.
