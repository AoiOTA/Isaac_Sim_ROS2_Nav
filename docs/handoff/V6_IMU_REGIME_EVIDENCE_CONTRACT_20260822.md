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

## Final duration, goal-MCAP, and installed-resource closure

- Follow-up start HEAD:
  `238be5522f74b6f25af76e5baa3d0980ea7c7be6`.
- The analyzer now resolves the diagnostic YAML and flat20 spawn YAML through
  the installed `robot_experiments` package-share manifest by default. A
  source-first invocation must pass both absolute `--config` and
  `--spawn-poses-file` paths. The trace manifest records both resolved paths;
  `__file__.parents` is no longer used for evidence identity.
- The locked Isaac runner resolves those same installed resources with
  `ros2 pkg prefix robot_experiments` and passes the diagnostic config to the
  passive trace CLI. The installed spawn/config content must match the bounded
  source contract. A stale underlay without the manifest exits before Isaac.
- Report thresholds, command rate, every requested segment duration, command
  and order are checked against the resolved config. Phase command duration
  and the raw/corrected/GT common-window duration must cover the configured
  `12.566 s` rotations, `4 s` arcs, and `2.5/5/2.5 s` S segments within
  `max(two phase periods, 2%)`, capped at `0.25 s`. Short `0.6/1 s` windows
  cannot become candidates.
- Stationary plus nine motion reset receipts require exact Python `int`
  seeds/generations (booleans and floats rejected), identical top/primitive
  receipts, and ten consecutive generations. Extra phase generations at or
  after the stationary generation are hidden-reset `FAIL`.
- Goal evidence now requires `--goal-mcap`. The analyzer derives the motion
  window, common-grid raw-IMU integral, and GT-relative yaw directly from the
  MCAP. Required topics are `/imu/data_raw`, `/ground_truth/odom`, `/cmd_vel`,
  `/simulation/reset_event`, `/simulation/collision`,
  `/bio_nav/route_goal_complete`, and `/rosout`. It requires one reset event,
  one parseable reset receipt log, one fresh successful route completion,
  collision-free samples, finite typed data, strict fresh-epoch stamps, and
  nonzero-command coverage. Pre-reset samples are excluded by record order.
- `--goal-evaluator` is metadata only, with
  `source=goal_mcap_outcome_metadata`; its resolved `source_mcap`, requested
  and actual seed, generation, pose, outcome, and collision fields must match
  the bag. Any arrays in that JSON are ignored. Omitting `--goal-mcap` is
  `AMBIGUOUS`; path/type/nonfinite/stamp/reset/outcome/collision mismatch is
  fail closed.

Installed entrypoints for the next authorized capture are:

```bash
PREFIX="$(ros2 pkg prefix robot_experiments)"
ros2 run robot_experiments motion_benchmark \
  --config "$PREFIX/share/robot_experiments/config/v6_imu_regime_diagnostic.yaml" \
  --output /absolute/path/motion_report.json

ros2 run robot_experiments imu_regime_analysis \
  --mcap /absolute/path/flat20_mcap \
  --phase-jsonl /absolute/path/phase.jsonl \
  --benchmark-report /absolute/path/motion_report.json \
  --goal-mcap /absolute/path/goal_mcap \
  --goal-evaluator /absolute/path/goal_metadata.json \
  --output /absolute/path/imu_regime_analysis.json
```

Final validation for this follow-up is code/build/unit only:

- focused source-first/no-cache IMU, ordinary MotionBenchmark, phase, graph,
  and reset-gate tests: **80 passed**;
- full `robot_experiments` plus related Isaac selection: **506 passed, 1
  unrelated pre-existing frozen-reference absolute-path failure**;
- fresh isolated build/install:
  `/tmp/v6_imu_contract_build.BJXc1r`,
  `/tmp/v6_imu_contract_install.KHMxhH`, and
  `/tmp/v6_imu_contract_log.45afly`;
- installed-package focused tests: **37 passed**; installed `ros2 run
  robot_experiments imu_regime_analysis --help`, package-share manifest,
  config/spawn lookup, and import identity: PASS;
- `py_compile`, YAML/resource loading, `bash -n`, and `git diff --check`: PASS.

No Isaac, ROS graph, MotionBenchmark live run, MCAP capture, navigation goal,
calibration choice, or formal qualification was performed. The frozen
`yaw_scale=0.9294` and RF2O-off policy remain unchanged; live closure is still
pending.
