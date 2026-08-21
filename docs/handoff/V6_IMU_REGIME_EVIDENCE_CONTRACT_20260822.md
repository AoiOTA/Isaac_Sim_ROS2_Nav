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

## Single-attempt goal binding and dropout closure

- Follow-up start HEAD:
  `1d717a87b0079d540678e5f48156e801ece3b570`.
- A post-reset goal bag now needs exactly one fresh
  `/bio_nav/route_goal_complete` terminal and that terminal must be `true`.
  A `false` terminal, `false -> true`, multiple `true` terminals, multiple
  recorded route requests, nonzero command before a recorded request, or
  nonzero command at/after the terminal cannot be merged into one successful
  attempt.
- When `/bio_nav/route_goal` is recorded, its `PoseStamped` record time,
  header time/frame, position, and orientation must identify exactly one
  request between reset and terminal and exactly match the evaluator metadata
  `route_goal_request`. Without that optional topic, the bounded provenance is
  explicitly `reset_terminal_single_command_attempt`; a greater-than-1-second
  split in nonzero commands remains `AMBIGUOUS` rather than being bridged.
- The result records terminal count/values/timestamps, route-request identity,
  binding source, selected command window, and MCAP topic counts. Raw IMU and
  GT, plus corrected IMU when `/imu/data` is present, must each cover the
  command window with a maximum sample gap no greater than the same `0.25 s`
  diagnostic limit. Larger or edge-truncated coverage is `AMBIGUOUS`; linear
  interpolation is performed only after this gate. The result exposes every
  stream's maximum gap and common coverage fraction.
- Focused source-first tests: **43 passed**. They include a 0.1-second positive
  fixture, one successful terminal, `false -> true`, multiple successful
  terminals, a greater-than-0.5-second raw/corrected gap, request metadata
  mismatch, and post-terminal motion.
- Fresh isolated `robot_experiments` build: PASS at
  `/tmp/v6_imu_goal_build.TYHcAO`, install
  `/tmp/v6_imu_goal_install.2tt8zi`, log
  `/tmp/v6_imu_goal_log.c9Pypu`. Installed-package focused tests: **43 passed**;
  installed import identity and `ros2 run robot_experiments
  imu_regime_analysis --help`: PASS. `py_compile` and `git diff --check`: PASS.

Verdict remains **PASS (code/build/unit only)**. No Isaac, ROS graph, MCAP
capture, navigation, scale selection, or qualification was run. Live capture
must include the new terminal/coverage provenance before any yaw-scale
decision; `yaw_scale=0.9294` and RF2O-off remain unchanged.

## Reset/request timestamp boundary closure

- Follow-up start HEAD:
  `1d3a2d8d990c85daa24c25dd282e818a36dc5329`.
- When a `PoseStamped` route request is recorded, the pre-request nonzero
  command fence is now exactly `reset_s <= t < request_s`. A stale nonzero
  command stamped exactly at reset therefore fails with
  `goal_command_before_request`; a command strictly before reset is excluded,
  while the request timestamp itself remains the valid lower bound.
- When no route request topic is recorded, the conservative
  `reset_terminal_single_command_attempt` path is explicit: reset is the
  attempt boundary, so a legitimate first nonzero command at exactly reset is
  retained instead of being misclassified as pre-request motion.
- Source-first and fresh-installed focused tests each report **47 passed**.
  Adversarial coverage includes reset-equal rejection, strict pre/post-reset
  behavior, request-boundary acceptance, and a valid no-request first command;
  terminal multiplicity, raw/corrected gap, command-duration, and installed
  package-share contracts remain passing.
- Fresh isolated package build/install: PASS at
  `/tmp/v6_imu_reset_boundary_build.lrnEgy` and
  `/tmp/v6_imu_reset_boundary_install.Vw4VgR`, with log at
  `/tmp/v6_imu_reset_boundary_log.okL1lC`. The installed import resolves to
  that prefix and installed `ros2 run robot_experiments imu_regime_analysis
  --help` passes.

Verdict remains **PASS (code/build/unit only)**. No Isaac, ROS graph, MCAP,
navigation, calibration selection, or formal qualification was run; the live
regime capture is still pending. Frozen `yaw_scale=0.9294` and RF2O-off are
unchanged.

## Corrected IMU header-stamp authority closure

- Follow-up start HEAD:
  `6e3deb2c777977e144e0c001bbaa2a504cbeafaf`.
- `/imu/data_raw`, `/imu/data`, and `/ground_truth/odom` now always use their
  message header stamps in goal-MCAP analysis. Corrected IMU can no longer
  fall back to bag record time. Missing, zero, duplicate, and backward header
  stamps fail closed.
- Bag record time is not used for yaw-stream ordering, attempt-window
  selection, maximum-gap checks, interpolation, or integration. A corrected
  header stream that is ordered but does not cover the command-derived common
  window remains `AMBIGUOUS` under the existing `0.25 s` coverage contract;
  no record/header skew threshold was added.
- Adversarial tests cover corrected zero, duplicate, backward, and shifted
  stale headers, plus a valid raw/corrected/GT case whose sensor bag times are
  shifted by `100 s` and jittered while authoritative headers remain valid.
  Existing route-request, reset-equal boundary, command-duration, terminal,
  gap, and installed-resource-share tests remain in the same focused suite.
- Source-first focused tests: **52 passed**. Fresh build/install PASS at
  `/tmp/v6_imu_header_stamp.hHu18p`; fresh-installed focused tests:
  **52 passed**. Installed import resolves inside that prefix and the installed
  `imu_regime_analysis --help`, changed-file `py_compile`, and
  `git diff --check` pass.

Verdict remains **PASS (code/build/unit only)**. No Isaac, ROS graph, MCAP
capture, navigation, calibration selection, or formal qualification was run.
The live regime capture remains pending; `yaw_scale=0.9294` and RF2O-off are
unchanged.

## Explicit MCAP file-order authority closure

- Follow-up start HEAD:
  `4aa311783582c4df75d48217b761090fec2326ca`.
- Both diagnostic and goal MCAP readers now require an explicit
  `rosbag2_py.ReadOrder(sort_by=ReadOrderSortBy.File, reverse=False)` storage
  acknowledgement before consuming evidence. Missing API support, an
  exception, or a non-true storage response is structured
  `AMBIGUOUS/mcap_file_order_unavailable`; the reader never silently accepts
  the rosbag2 default received-timestamp order.
- Raw IMU, corrected IMU, and ground-truth header stamps remain in MCAP file
  publish order for duplicate/backward validation. Their header domain alone
  drives yaw windowing, maximum gaps, interpolation, and integration; no sort
  can hide a file-order header regression.
- Reset, command, collision, route request, route terminal, and receipt-log
  event times use bag received timestamps and are explicitly sorted after
  collection. Received-time jitter therefore cannot change yaw-stream header
  order, while event boundaries remain deterministic in their own clock
  domain. Output provenance records both time bases and the file read order.
- Real `rosbag2_py.SequentialWriter` MCAP tests cover a positive bag with
  adjacent received timestamps reversed while file/header order increases,
  plus file-order raw-header duplicate and corrected-header backward failures.
  Existing single-attempt, reset/request boundary, terminal, and 0.25-second
  stream-gap regressions remain in the same focused suite.
- Source-first focused tests: **56 passed**. Fresh isolated package build PASS
  at `/tmp/v6_imu_file_order_build.qBwj0i`, with install
  `/tmp/v6_imu_file_order_install.ukM3HM` and log
  `/tmp/v6_imu_file_order_log.3K0ILK`; fresh-installed focused tests:
  **56 passed**. Installed import identity, installed `imu_regime_analysis
  --help`, changed-file `py_compile`, and `git diff --check` pass.

Verdict remains **PASS (code/build/unit only)**. No Isaac, ROS graph,
navigation, live MCAP capture, calibration selection, or formal qualification
was run. The live regime capture is still pending; `yaw_scale=0.9294` and
RF2O-off remain unchanged.

## Flat20 LiDAR-feature readiness amendment

- Amendment start HEAD:
  `aa9d2d73d2f2dc843f29a086fbfb71db5b06f4d2`.
- Attempt 1 evidence is
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_imu_regime_session_a_20260821T210119Z`.
  Verdict is **STOP / NO ENGINEERING CAPTURE / NOT FORMAL**. Isaac stopped at
  7.587 s in `SceneComposer` because the archive lacked the external
  `jackal_original.usd` binary. ROS, stationary seed 8609, all primitives, and
  MCAP never started. Authority is `STOP.md` plus `logs/isaac.log`.
- Attempt 2 evidence is
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_imu_regime_session_a_attempt2_20260821T210842Z`.
  Verdict is **STOP / NO ENGINEERING CAPTURE / NOT FORMAL**. Asset import/check,
  scene validation, Integration plus 14 Module3 packages, domain 100, IMU,
  odometry, ground truth, and authority checks passed. The final 10 s readiness
  window nevertheless observed zero messages on `/lidar/points_raw`,
  `/lidar/points_scan`, `/scan`, and `/scan_safety`; seed 8609, the nine
  primitives, MCAP, and the analyzer did not start. Authority is `STOP.md`,
  `summary.json`, and `provenance/pre_benchmark_contract_final.json`.
- Root cause was the locked runner's explicit `--no-dynamic-obstacles`. The
  plain Grid USD has no vertical LiDAR geometry. The already versioned
  `v6_calibration_grid_features.yaml` contains four boundary walls and three
  asymmetric full-LiDAR-height features, but the failed runner never authored
  them.
- The runner now resolves that YAML from the installed `robot_experiments`
  package share and passes `--dynamic-obstacle-config ... --dynamic-obstacles`.
  The legacy CLI name only enables authoring: the locked config has seed
  20260821, seven stationary objects, fixed start=end, speed zero, and therefore
  **zero moving objects**. CollisionMonitor and `/cmd_vel` authority are not
  changed or bypassed.
- Passive phase provenance now fails closed unless obstacle authoring is true,
  the resolved feature file has exact bounded content, seed 20260821, seven
  objects, and moving-object count zero. The offline analyzer resolves the same
  installed feature resource and rejects missing/mismatched path, content, or
  provenance. Source-first use must pass the complete `--config`,
  `--spawn-poses-file`, and `--obstacle-config` resource set.
- Added the read-only installed entry point `v6_imu_lidar_preflight`. Before
  seed 8609 is dispatched, it requires at least two live messages on raw cloud,
  filtered cloud, mapping scan, and safety scan; header stamps must be strictly
  increasing, newest age must be below 0.4 s, and every stream must expose a
  finite return. It publishes no command and returns STOP on timeout or any
  contract failure:

```bash
ros2 run robot_experiments v6_imu_lidar_preflight \
  --output /absolute/attempt3/provenance/lidar_readiness.json
```

- Source-first/no-cache related tests: **173 passed**. A smaller new-contract
  set reported **72 passed**. `py_compile`, runner `bash -n`, YAML/resource
  loading, installed analyzer/preflight `--help`, installed resource resolution,
  and `git diff --check` passed. Fresh isolated `robot_experiments` build passed
  at `/tmp/v6_imu_lidar_build.0BdhgV`, install
  `/tmp/v6_imu_lidar_install.W9631i`, log
  `/tmp/v6_imu_lidar_log.n99ktJ`.

Verdict is **PASS (CODE / BUILD / UNIT ONLY)**. No Isaac, ROS graph, LiDAR
readiness probe, stationary window, primitive, MCAP, navigation goal, scale
selection, or formal qualification was run for this amendment. Attempt 3 is
**PENDING**. `yaw_scale=0.9294` and RF2O-off remain unchanged.

## Sim-time and exact flat20 geometry amendment

- Amendment start HEAD:
  `78d49a1f9d3cc9ece74400682ff5ab55d7efabc0`.
- `v6_imu_lidar_preflight` now creates its node with a locked
  `use_sim_time=true` override, rejects attempts to disable that parameter,
  disables global ROS arguments, and verifies that its node clock is
  `ROS_TIME`. The entry point consumes only its two documented application
  options and explicitly rejects `--ros-args`; it therefore cannot silently
  reinterpret `--output` or accept a caller-provided system-time override.
- A zero or non-finite simulation clock is STOP and does not produce an age.
  Once `/clock` is positive, freshness is computed only from the node's ROS
  simulation time and each message's header stamp. A system-epoch `now`
  against simulation stamps is a tested stale STOP counterexample.
- The analyzer no longer accepts any seven stationary objects with the right
  seed. It performs a complete, strong-type structured comparison of the
  installed `v6_calibration_grid_features.yaml`: exact ordered IDs, cube
  dimensions/height/scale, positions, masses, start/end, stationary/parked
  policy, zero velocity, delay/jitter, coordinate frame, seed, and count.
  Renamed, 9 x 9, displaced, mass-changed, moving, nonzero-velocity,
  reseeded, and missing-object fixtures all fail with
  `obstacle_config_mismatch`. Installed-resource resolution is covered by the
  same comparison rather than a hard-coded identity-only report. Both
  `flat20_start` and its `mapping_start` authoring alias must retain the exact
  map/USD origin and orientation; the legacy bundle digest is not used as
  geometry authority.
- Validation: source-first focused tests **72 passed**; fresh isolated
  `robot_experiments` build PASS at
  `/tmp/v6_imu_simtime_build.woJmcR`, install
  `/tmp/v6_imu_simtime_install.b6pBGp`, log
  `/tmp/v6_imu_simtime_log.3NGRES`; fresh-installed import plus the same tests
  **72 passed**; installed analyzer/preflight `--help`, entry-node ROS_TIME/
  zero-clock test, changed-file `py_compile`, and `git diff --check` PASS.

Verdict remains **PASS (code/build/unit only)**. No Isaac, live ROS graph,
LiDAR readiness capture, stationary/primitive session, MCAP, calibration
selection, or formal qualification was run. Attempt 3 remains **PENDING**;
`yaw_scale=0.9294`, RF2O-off, CollisionMonitor, route/reset/benchmark, and
critic semantics are unchanged.

## Smoother-aware schedule and Attempt 3 retroactive analysis

- Amendment start HEAD:
  `6af4fde9b6b6eaa095f4c0f25ff34a561766ecc8`.
- MotionBenchmark report schema 2 adds an immutable per-segment simulation
  schedule receipt: index, start/end simulation time, expected duration,
  upstream `/cmd_vel_nav` linear/angular intent, publish count, and
  completed/truncated status. The stationary reference records the same
  schedule shape. Final-zero booleans remain only for schema-1 compatibility;
  schema 2 records zero publish time/count receipts and the analyzer derives
  authority from downstream MCAP evidence. Command values, playback timing,
  thresholds, reset/gate logic, and ordinary benchmark evaluation are
  unchanged.
- The Session A analyzer now requires exact MCAP types and finite samples for
  `/clock`, `/simulation/reset_event`, `/cmd_vel_nav`,
  `/cmd_vel_smoothed`, `/cmd_vel`, and `/cmd_vel_sim` in addition to the yaw
  streams. Reset events, consecutive report generations, the report schedule,
  and upstream intent bind each primitive. Smoother/CollisionMonitor/gate
  ramps are coverage evidence only: CW/CCW/arcs remain one segment and the S
  route is exactly `(+0.45,2.5 s) -> (-0.45,5.0 s) -> (+0.45,2.5 s)`.
  Generation-crossing phase rows are excluded/fail closed.
- Final-stop evidence requires a final upstream zero, bounded downstream-zero
  latency, at least 0.8 s continuous `/cmd_vel_sim` zero coverage, no later
  nonzero command in the reset epoch, and command-chain gap/coverage checks.
  Hidden resets, wrong/missing/truncated intent, HOLD leaks, non-finite data,
  and short/gapped/missing final-zero coverage cannot authorize a scale.
- `capture_contract_status` is independent from `performance_status`.
  Performance failure does not erase segment evidence: k-star, endpoint error,
  aligned RMSE/P95, and <=5 degree intervals remain in the output. Overall is
  still FAIL and `scale_selection_authorized=false`; neither
  `PASS_CANDIDATE` nor `CONFIRMED_NO_GLOBAL_CONSTANT` is promoted through a
  failed benchmark. A no-goal run remains non-PASS.
- Attempt 3 was re-analyzed without altering its original artifacts:
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_imu_regime_session_a_attempt3_20260821T220048Z/analysis/smoother_schedule_v2/imu_regime_analysis.json`.
  Twelve windows were recovered. Verdict is **FAIL / NOT FORMAL** with
  `performance_status=FAIL`, `capture_contract_status=AMBIGUOUS`, and
  `scale_selection_authorized=false`. The capture ambiguity is explicit:
  stationary plus the eight single-segment primitives lack the required 0.8 s
  final `/cmd_vel_sim` zero coverage in this old capture; the S-route final
  zero tail is long enough. This does not invalidate the computed diagnostic
  values: pure-spin k-star is 0.928739 CW and 0.929535 CCW; arc values span
  0.914476--0.981557; S segments are 0.908586, 0.918064, and 0.925207. The
  segment <=5 degree intersection is `[0.9162, 0.9417]`, but it is not a scale
  selection because performance failed and goal evidence is absent.
- Validation: source-first and fresh-installed focused suites each report
  **97 passed**. Fresh isolated build/install PASS at
  `/tmp/v6_imu_schedule_final3_build.YDJFR2` and
  `/tmp/v6_imu_schedule_final3_install.usJHFc`, log
  `/tmp/v6_imu_schedule_final3_log.G2E8gL`. Installed import identity, installed
  `imu_regime_analysis --help`, changed-file `py_compile`, and
  `git diff --check` PASS.

Verdict for this amendment is **PASS (code/build/unit plus retroactive offline
analysis)**. It is not a new live capture, navigation goal, calibration
selection, or formal qualification. Attempt 4 must use schema-2 schedules and
the full 0.8 s downstream zero receipt; the Kujiale goal run remains pending.
`yaw_scale=0.9294` and RF2O-off are unchanged.

The wider package regression collected 530 tests and reported **529 passed, 1
unrelated path-sensitive failure** in `test_rivermark_reference`: its frozen
reference embeds a different historical absolute worktree path. No runtime or
numeric value differed, and this amendment did not inspect or change that
out-of-scope reference. The focused source-first and fresh-installed suites
above remain the acceptance result.
