# V6 motion-assist pure-yaw consistency amendment

Date: 2026-08-21

## Scope and evidence

- Worktree/branch/start: permitted Module3 `cognitive-navigation` worktree at
  `74270f90e310bff0acfaa2b16e970c85928ca713`.
- Calibration evidence supplied to this amendment measured raw IMU/GT yaw
  scale mean `1.081260`; `1 / 0.925 = 1.081081`, a `0.0166%` difference.
- The simulation loop publishes the current physics-step sensor samples before
  applying skid-steer motion assist. The previous pure-yaw `0.925` multiplier
  could therefore create the measured post-sensor angular-rate disagreement.

## Change

- `_yaw_command_scale(0.0)` and `_yaw_command_scale(-0.0)` now return exactly
  `1.0`.
- Existing nonzero arc scaling is unchanged: `0.10 m/s -> 0.9625`, and speeds
  at or above `0.20 m/s -> 1.0`.
- Linear correction, acceleration bounds, timeout behavior, command handling,
  arc assist, configuration, wheel geometry, EKF, covariance and sensor graphs
  are unchanged.
- A static contract test keeps `motion_assist.update()` after the physics-step
  `app.update()` call so future ordering changes cannot silently invalidate the
  calibration interpretation.

## Validation

- `python3 -m py_compile isaac_sim/src/robot/skid_steer_motion_assist.py isaac_sim/tests/test_skid_steer_motion_assist.py`: PASS.
- Focused motion-assist and graph-contract tests: `11 passed`.
- Full `isaac_sim/tests`: `185 passed, 11 skipped`; all skips require optional
  Isaac/USD `pxr` bindings unavailable in this unit environment.
- No Isaac, ROS, Nav2, calibration, evidence or qualification run was started.

## Verdict and remaining work

**PASS (implementation/static-test only).** Live behavior is not yet claimed.
Rerun the six flat20 rotation episodes and confirm raw IMU/GT yaw scale and
closure. In the same run, confirm there are no duplicate equal-stamp core
sensor samples. Retain separate off/shadow RF2O results and rerun affected
Rivermark/PRIMARY evidence only after the accepted Estimated State parameters
are frozen.
