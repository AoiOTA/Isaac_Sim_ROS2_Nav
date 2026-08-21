# V6 IMU regime evidence-contract amendment — 2026-08-22

## Scope

- Worktree/branch: permitted Module3 `cognitive-navigation` worktree.
- Start HEAD: `7b6b01f48fafdeb9985463e0cda6f49ababb2e71`.
- Fixed `main` pins were rechecked before editing and matched the project
  boundary. Integration and Module2 were not modified.
- The frozen `yaw_scale=0.9294`, RF2O-off policy, route/reset-gate/critic logic,
  and navigation loop call order were not changed.

## Amendment

- The diagnostic-only MotionBenchmark config now has an explicit independent
  `stationary_reference` reset (`seed=8609`, 10 s continuous zero). Its report
  includes reset receipt/generation, duration, collision, odometry motion,
  sample count, zero-command count, and final-zero status. Ordinary benchmark
  configs retain `stationary_reference=None` and perform no extra reset.
- The analyzer accepts only the ordered nine-primitive contract, seeds
  `8610..8618`, matching positive increasing generations/receipts, nonzero
  samples, expected segment commands, and one-to-one top receipts. Explicit
  failure/STOP/collision, extra/duplicate/order/seed/receipt mismatch is
  `FAIL`; missing/truncated/insufficient evidence is `AMBIGUOUS`.
- MCAP input requires exact `sensor_msgs/msg/Imu` and
  `nav_msgs/msg/Odometry` types, finite vectors/quaternions/yaw, normalized GT
  quaternion, strictly increasing header stamps, and explicit topic counts.
  Goal yaw evidence requires schema/source/reset/outcome/collision provenance
  and finite arrays. Failures still produce a structured report.
- Every analysis window uses the common overlap
  `t0=max(starts)..t1=min(ends)`, no extrapolation, one sorted union grid,
  linear rate/GT interpolation, and shared-grid trapezoidal integration. The S
  route requires three command windows; all other moving primitives require
  one.
- Phase gating validates all four ReadIMU/PublishIMU value/time attributes,
  their shape/finite/error/null/valid counts, strict stamps, monotonic loop
  boundaries, and reset generation. A malformed value is `FAIL`; absent or
  empty evidence is `AMBIGUOUS`.
- Added unique `flat20_start` while preserving `mapping_start`. The locked
  runner `scripts/run_v6_imu_regime_diagnostic_isaac.sh` selects the Isaac 6
  Grid USD, flat20 profile, realistic odometry, GT, and dynamics off. Trace
  mode fails closed on any provenance mismatch, preventing a Kujiale
  `mapping_start` trace from being accepted.
- Added runtime dependencies `rosbag2_py` and `rosidl_runtime_py`. The existing
  loop-order regression now uses Python AST call ordering and still requires
  `app.update -> motion_assist.update -> ground_truth.update`.

## Validation actually run

- Source-first/no-cache related regression: **110 passed**.
- Python bytecode compilation, diagnostic YAML parsing/loader, runner
  `bash -n`, and `git diff --check`: PASS.
- Fresh isolated `robot_experiments` build: PASS.
  - build: `/tmp/v6_imu_evidence_build.yizdjX`
  - install: `/tmp/v6_imu_evidence_install.l2zy4u`
  - log: `/tmp/v6_imu_evidence_log.POQ9cz`
- Source-first post-build import resolved to this worktree; strict config,
  stationary seed, primitive order, and 1201-point offline scale grid: PASS.

## Verdict and next action

**PASS (CODE / BUILD / UNIT ONLY).** No Isaac, ROS graph, MCAP capture,
MotionBenchmark runtime, navigation goal, calibration decision, or formal
qualification was run. Live closure remains pending. Keep `0.9294` during the
capture and run the locked flat20 session: stationary, all nine primitives,
then the separately provenance-bearing Kujiale Estimated goal. Only the full
contract can produce `PASS_CANDIDATE` or `CONFIRMED_NO_GLOBAL_CONSTANT`.
