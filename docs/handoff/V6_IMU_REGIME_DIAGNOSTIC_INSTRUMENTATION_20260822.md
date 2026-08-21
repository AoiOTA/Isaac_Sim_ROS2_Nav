# V6 IMU regime diagnostic instrumentation — 2026-08-22

## Scope and provenance

- Branch/worktree: `cognitive-navigation` in the permitted Module3 worktree.
- Start HEAD: `7df6be9b95de31f1c6b41aa9c5f7d8135f799134`.
- Fixed local `main` pins checked before editing:
  - Integration `f23a7eccc542e602ec641daf7a20b14c2371dca9`
  - Module3 `22d66470c4b903349b2467dc876490bbebfc0083`
  - Module2 `c8297a590ba61bcf712ad4a339437fb2c44a027e`
- Scope is instrumentation/config/offline analysis only. No yaw-scale value,
  RF2O policy, reset gate, MotionBenchmark behavior, route logic, Integration,
  or Module2 code was changed.

## Implemented

- Added default-off `--imu-regime-phase-trace PATH` to `navigation_sim.py`.
  It is deliberately excluded from `diagnostic_command_mode`: it does not
  change ResetStopGate input, create a publisher, send a command, or add an
  `app.update`, graph evaluation, sleep, or state setter.
- Added buffered, getter-only `ImuRegimePhaseTrace` JSONL capture. Each loop
  records reset generation before/after the loop; simulation time around
  `app.update`; monotonic boundaries around app, MotionAssist, and GT; body
  forward/yaw rate before and after assist; assist enabled/target/applied;
  `ReadIMU` angVel/sensorTime; `PublishIMU` angularVelocity/timestamp; and the
  GT publication receipt. Missing graph/runtime values carry an explicit
  per-field error and `null`, rather than a fabricated value. Buffers flush
  every 60 loops and at observed reset-generation transitions.
- `GroundTruthRecorder.update()` now returns a frozen receipt only when its
  existing odometry publication occurs, otherwise `None`. Publication rate,
  topics, message contents, order, and all existing ignore-the-return callers
  remain compatible.
- Added strict current-MotionBenchmark-schema
  `v6_imu_regime_diagnostic.yaml`: external passive 10 s stationary window,
  then CW360, CCW360, v=0.05/0.10/0.25 bilateral 4 s arcs, and the frozen
  2.5/5/2.5 s S route. The existing benchmark executes one reset per
  primitive with seeds 8610 through 8618, no retry path, and final zero.
- Added `imu_regime_analysis` offline entry point. It reads MCAP through the
  installed `rosbag2_py` MCAP backend plus phase JSONL, MotionBenchmark report,
  and optional goal-evaluator yaw series. It partitions by reset/command
  windows and reports GT unwrap, raw/corrected trapezoidal integration,
  endpoint/aligned RMSE/P95, steady ratio, `k*`, direction/speed bins,
  pre/post-assist delta, stamp quality, each segment's <=5 degree scale
  interval, optional goal identity-nondegrade interval, global intersection,
  and one of `CONFIRMED_NO_GLOBAL_CONSTANT`, `PASS_CANDIDATE`, `AMBIGUOUS`, or
  `FAIL`. Missing or invalid evidence does not produce a PASS claim.

## Validation actually run

- Source-first, no-cache focused/regression pytest:
  `82 passed` for the new phase/analysis tests plus existing ground-truth,
  navigation camera-contract, MotionBenchmark, and package-contract tests.
- New-only focused pytest after final analysis additions: `12 passed`.
- `python3 -m py_compile` for all changed Python modules: PASS.
- strict YAML parse plus `load_motion_config(...)`: PASS; 9 primitives and
  incremental seeds confirmed.
- Fresh isolated `colcon build --packages-select robot_experiments
  --symlink-install`: PASS at:
  - build `/tmp/v6_imu_regime_final_build.trCpFF`
  - install `/tmp/v6_imu_regime_final_install.NxCfJS`
  - log `/tmp/v6_imu_regime_final_log.UMrBl3`
- Source-first post-build import resolved to this worktree and the 0.90..1.02
  scan contains 1201 points: PASS.
- `git diff --check`: PASS.

## Verdict and remaining risk

**PASS (CODE / BUILD / UNIT ONLY).** No Isaac, ROS graph, MCAP recording,
MotionBenchmark run, stationary window, calibration sequence, Kujiale goal,
visual inspection, navigation campaign, or formal qualification was run.
Therefore this commit neither confirms regime dependence nor chooses a new
scale. The currently frozen `yaw_scale=0.9294` and RF2O-off policy are
unchanged. The next authorized runtime must first verify that Isaac 6 exposes
the named OmniGraph attributes in the expected form; missing attributes will
be visible and should make the offline result `AMBIGUOUS`, not crash Isaac or
silently fabricate evidence.

## Recommended next action

Run the bounded same-session diagnostic sequence with the phase flag and MCAP
recording: external stationary 10 s, the nine MotionBenchmark primitives, then
the existing Kujiale Estimated goal evaluator. Analyze only after all reset
receipts, final zeros, collision status, and topic stamp-quality checks are
present. Keep 0.9294 during capture; candidate scales are scanned offline only.
