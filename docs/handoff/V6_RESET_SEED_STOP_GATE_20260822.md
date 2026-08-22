# V6 reset-seed receipt and ResetStopGate handoff

Date: 2026-08-22

## Scope and result

- Worktree/branch/start: permitted Module3 worktree
  `/home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3`,
  `cognitive-navigation`, `9671be168266cb639e04e4c1baf46e1d353c0720`.
- Fixed main ancestry checked: `22d66470c4b903349b2467dc876490bbebfc0083`.
- Implementation commit: `20bc5df` (`Close reset seed and velocity stop
  contracts`). No Integration or Module2 file was changed.
- Verdict: **PASS (code/build/unit contracts only)**; **AMBIGUOUS for live
  active-reset closure** because Isaac, ROS, Nav2, navigation, visual evidence,
  and formal qualification were not run.
- The committed IMU `yaw_scale=0.9294` baseline and critic semantics were not
  changed.

### Reviewer-blocker amendment

- Amendment start: `b28bdf56ccb7edf61dc176bfe3d8c49a0e3b03cc` on the same
  permitted worktree/branch. This handoff's current commit closes the review
  blockers at code/build/unit level only.
- Reset release ownership is now explicit. Navigation/localization mode keeps
  the current generation held for ActivationGate; mapping/teleop and explicit
  R2C diagnostic command modes auto-release only after their complete reset
  transaction succeeds. A failed transaction remains held, with no timeout
  release.
- Startup and Trigger resets now use the same `ResetServiceBridge` finalizer.
  The active transaction remains exclusive through `mark_reset_complete()`
  and optional same-generation release, so an early Trigger cannot create a
  newer HOLD in that interval. `navigation_sim` no longer performs an
  unconditional startup release.
- MotionBenchmark now waits for a post-request ResetStopGate status matching
  the receipt generation with HOLD cleared, an active CollisionMonitor
  lifecycle state, and continuously settled new-epoch clock/odometry/TF before
  returning from reset and dispatching any nonzero `/cmd_vel_nav`. Missing,
  never-released, malformed, or wrong-generation status times out or STOPs.
- Reset receipt extraction now uses `json.JSONDecoder.raw_decode` after the
  explicit marker. Legal JSON strings may contain `};` and escaped quotes;
  directly appended trailing junk and wrong field types are rejected.

## Seed and receipt contract

- `effective_reset_seed` is the explicit CLI `--dynamic-seed` when supplied,
  otherwise the scenario seed. The same value is passed to startup
  `ResetRequest` and to `ResetServiceBridge(default_reset_seed=...)`, so the
  first Trigger no longer silently falls back to zero.
- Per-episode parameter changes remain supported. The reset response now
  contains a flat JSON `reset_receipt` with actual seed, reset transaction
  generation, pose, odometry mode, case ID, and variant ID.
- `ExperimentRunner`, `V6FormalNode`, and `MotionBenchmarkNode` use the shared
  parser, retain the complete response, and fail closed if requested
  seed/case/variant differs from the receipt. Tests cover CLI/scenario fallback,
  `8601 -> 8602`, complete provenance, and mismatch STOP.

## Final command authority

The normal Navigation chain is now:

`controller/behavior -> /cmd_vel_nav -> velocity_smoother ->
/cmd_vel_smoothed -> CollisionMonitor -> /cmd_vel -> ResetStopGate ->
/cmd_vel_sim -> Isaac control graph/IdleBrake/MotionAssist`.

- `ResetServiceBridge` no longer creates a final `/cmd_vel` publisher.
- Reset start synchronously calls `ResetStopGate.hold()` before
  `ResetManager.reset()` can pause Timeline. HOLD clears release eligibility
  and a steady-wall timer continuously publishes zero on `/cmd_vel_sim`.
- A failed, timed-out, missing-service, repeated, or exceptional reset remains
  held. Successful transaction completion makes only that generation eligible.
- ActivationGate submits the current generation through Isaac's atomic
  parameter service only after fresh-epoch recovery and the managed-node state
  snapshot (including `collision_monitor`) are active. A stale generation is
  rejected and cannot release a newer HOLD.
- Startup and subsequent Trigger resets share the same release policy. In
  Navigation/localization mode, ActivationGate releases the eligible current
  generation after lifecycle readiness. Mapping/teleop and explicit diagnostic
  command modes have no ActivationGate, so the successful transaction
  finalizer auto-releases its captured generation. Release publishes one final
  zero and never replays a cached pre-reset command; motion requires a fresh
  subsequent message.
- Command-generating Isaac diagnostics use `/cmd_vel_diagnostic` only when an
  explicit diagnostic CLI mode selects that gate input. Motion benchmark is
  upstream on `/cmd_vel_nav`. In the default Navigation profile, Collision
  Monitor remains the only external `/cmd_vel` publisher and ResetStopGate the
  only `/cmd_vel_sim` publisher.

## Existing dynamic medium FAIL kept separate

This amendment was motivated by, but did not rerun or reinterpret, the
existing mixed-motion evidence at
`/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_estimated_dynamic_smoke_20260821T150407Z`.
That run navigated successfully in `51.93 s`, ended `0.195 m` from goal, and
had no physical/footprint collision, but remained **ENGINEERING FAIL** because
the `0.9294` correction worsened aligned yaw RMSE (`0.08527 -> 0.12342 rad`)
and endpoint errors. Its separate runtime debts were requested seed `8601`
but receipt seed `0`, plus simultaneous Collision Monitor and Isaac reset-zero
publishers on `/cmd_vel`. This commit addresses those two software contracts;
it does not repair, invalidate, or requalify the IMU finding.

## Validation actually run

- `python3 -m py_compile` on all changed Python runtime modules: PASS.
- Source-first, no-cache focused pytest across Isaac reset/control graph,
  ResetStopGate, ActivationGate/launch parameters, runner adapters, receipt,
  motion benchmark, and V6 formal tests: **124 passed**.
- Fresh isolated `robot_experiments` build: PASS at
  `/tmp/bio_nav_reset_gate_build.oB9j5S` and
  `/tmp/bio_nav_reset_gate_install.yRv32y`.
- Fresh sanitized isolated `robot_bringup` build: PASS at
  `/tmp/bio_nav_gate_clean_build.ct1qQi` and
  `/tmp/bio_nav_gate_clean_install.Bx5J1F`.
- Installed-package import of `Nav2ActivationGate` and
  `parse_reset_receipt`: PASS.
- YAML load, `xmllint --noout`, `git diff --check`: PASS.
- `ament_flake8` was available but the repository-wide quote/docstyle policy
  reports thousands of pre-existing issues in the touched legacy modules; it
  was not treated as a clean regression signal. `py_compile` and focused tests
  are clean. No shell script changed, so bash syntax validation was not
  applicable.
- `colcon test` for the isolated bringup package did not start tests because
  colcon required unbuilt workspace dependency `package.sh` files. The same
  source-first bringup tests are included in the 124-pass focused run.

Reviewer-blocker amendment validation:

- Source-first/no-cache focused reset, control-graph, ActivationGate/bringup,
  receipt, MotionBenchmark, and experiments contract pytest: **141 passed**.
  Added cases cover auto-release, external release, failure HOLD, release-time
  reset exclusion, delayed/never/wrong-generation dispatch gates,
  CollisionMonitor/estimated-state readiness, and `valid};case`/escaped quote
  receipt strings.
- Fresh isolated `robot_experiments` + `robot_bringup` build and installed
  imports: PASS at `/tmp/bionav_reset_gate_build.OvPN5Z`.
- Changed-file `py_compile`, relevant YAML/XML parse, and `git diff --check`:
  PASS.
- No Isaac, ROS graph, Nav2, navigation, visual evidence, engineering campaign,
  or formal qualification was run. Live closure remains **UNVERIFIED**.

## Final-review freshness and release-finalization amendment

- Branch/worktree/start: `cognitive-navigation` in the permitted Module3
  worktree, starting at `3e607ccbee920ac3b7df9c0a43f4a897cf60909e`;
  fixed Module3 main `22d66470c4b903349b2467dc876490bbebfc0083`
  remained an ancestor. The amendment commit is the commit containing this
  handoff and is reported in the master handoff.
- Reset transaction hypothesis: non-navigation auto-release is safe only
  after the reset futures, `/simulation/reset_event`, initial-pose policy, and
  ResetStopGate status transitions all succeed. The finalizer now performs
  those critical actions before eligibility/release. ResetStopGate stages
  completion/release status while the live command gate remains HOLD, so a
  zero/status publication exception cannot leave it open. Receipt construction
  and logging remain noncritical description outside the release boundary.
- Motion dispatch hypothesis: a one-time clock/odom/TF snapshot is not a safe
  motion barrier. Clock, odom, and `odom -> base_link` TF now carry wall receipt,
  ROS stamp, and forward-progress times. They must keep advancing within
  `state_freshness_sec: 0.25`, remain within
  `stamp_coherence_sec: 0.50` of simulation clock, and stay valid throughout
  the `0.60 s` settle. CollisionMonitor is polled during settle and queried
  again immediately before dispatch. Gate generation/HOLD changes, stale or
  incoherent state, or an inactive final CollisionMonitor query produce STOP
  before any nonzero command.
- Motion watchdog: a non-advancing `/clock` for
  `sim_clock_stall_timeout_sec: 0.50` during playback immediately publishes
  zero, records the primitive as `FAIL/STOP`, and ends the benchmark report.
- Changed runtime/config/tests: `isaac_sim/src/bridge/{reset_service.py,
  reset_stop_gate.py}`, `robot_experiments/{motion_benchmark.py,
  config/motion_benchmark.yaml}`, and their focused tests. RouteCoordinator,
  receipt/seed parsing, ActivationGate, R2C/non-navigation auto-release,
  Navigation external release, and early-Trigger exclusion were regression
  tested without source changes.
- Validation actually run: source-first/no-cache focused pytest **51 passed**;
  sanitized broader reset/control/ActivationGate/receipt/MotionBenchmark/V6
  adapter plus RouteCoordinator pytest **249 passed**. The first broader run
  inherited a stale overlay and failed collection; after removing inherited
  overlay paths and putting current allowed-worktree sources first, it passed.
- Fresh build: `robot_experiments` isolated build passed, then a sanitized
  `--packages-up-to robot_bringup` build passed for **14 packages** at
  `/tmp/bionav_gate_fresh.pXhLQh/install_up_to2`. Installed MotionBenchmark and
  ActivationGate imports resolved from that fresh install. Changed-file
  `py_compile`, motion YAML load/relationship check, relevant package XML,
  and `git diff --check` passed.
- Verdict: **PASS (code/build/unit only)**. No Isaac, live ROS graph, Nav2,
  navigation, visual evidence, engineering campaign, or formal qualification
  was run. Live behavior remains **UNVERIFIED** and requires the already-listed
  authorized reviewer closure.

## Reviewer-only live closure still required

Run the authorized Navigation reviewer with an active goal and Trigger reset.
Required checks remain:

1. `ros2 topic info -v /cmd_vel` shows Collision Monitor as the sole publisher;
   `/cmd_vel_sim` shows ResetStopGate as the sole publisher.
2. From synchronous HOLD through Timeline play, asynchronous reset futures,
   reset event, goal cancel, PAUSE, reseed/readiness, RESUME, and generation
   release, every `/cmd_vel_sim` sample is zero.
3. No old-goal nonzero command crosses release; only a fresh new-goal command
   moves the robot.
4. Ground Truth and estimated odometry show no post-reset slip and no collision.
5. Repeat reset, service-unavailable, timeout, and stale-release cases remain
   held. Any nonzero internal command, second final publisher, stale release,
   slip, or collision is **FAIL / STOP**.

## MotionBenchmark STOP evidence amendment

- Branch/worktree/start: `cognitive-navigation` in the permitted Module3
  worktree, starting at `9cd4b81963f7ddc06cdeb648cf49d2ce5a790331`;
  fixed Module3 main `22d66470c4b903349b2467dc876490bbebfc0083`
  remained an ancestor and the tracked worktree was clean. The amendment
  commit is the commit containing this handoff and is reported to master.
- Evidence isolation: before each primitive can log, reset, enter the dispatch
  barrier, or play motion, MotionBenchmark clears its samples, collision flag,
  recording/segment/command state, and current receipt. A second-primitive
  dispatch STOP therefore records `sample_count: 0` and
  `collision_detected: false` without changing the completed first row.
- Receipt retention: as soon as a successful Trigger response is parsed, its
  seed, generation, case/variant identifiers, pose/odometry, and full response
  are retained in both current-primitive and top-level receipt state. A later
  dispatch-barrier STOP includes that receipt instead of losing it.
- Report reproducibility: the JSON now records the decision inputs
  `reset_settle_sec`, `state_freshness_sec`, `stamp_coherence_sec`,
  `sim_clock_stall_timeout_sec`, `dispatch_barrier_timeout_sec`,
  CollisionMonitor state/query freshness thresholds and required active state,
  ResetStopGate generation-match requirement, plus the existing motion
  thresholds.
- Contract tests: the format-sensitive TF source-string check is now an AST
  assertion for a semantic `lookup_transform(odom, base_link, Time())` call.
  The focused MotionBenchmark/V6 tests passed **39 tests**; the source-first,
  no-cache benchmark/reset/estimated-state/ResetStopGate/ActivationGate suite
  passed **106 tests**. The suite covers retained second-reset receipts,
  isolated STOP rows, and the existing clock-stall zero-command/STOP behavior.
- Fresh isolated `robot_experiments` build and installed-module import: PASS at
  `/tmp/bionav_motion_evidence.u42NYu`; the imported
  `motion_benchmark.py` resolved from that fresh install. Changed-file
  `py_compile` and final `git diff --check` are recorded with the commit handoff.
- Verdict: **PASS (code/build/unit evidence closure only)**. No Isaac, ROS
  graph, Nav2, navigation, visual evidence, engineering campaign, or formal
  qualification was run. Live reviewer closure remains **PENDING** under the
  unchanged reviewer-only checks above.

## Attempt5 wall-heartbeat amendment

- Start HEAD: `5dc3b91e9922a12b45e63b135342350e5e847a33`; fixed Module3
  main remained `22d66470c4b903349b2467dc876490bbebfc0083`.
- Triggering evidence:
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_active_reset_live_attempt5_20260822T001729Z`.
  Attempt5 remains **ENGINEERING FAIL / STOP / NOT FORMAL**.  Its finalized
  bag had 41 all-zero `/cmd_vel_sim` HOLD samples but a 0.378262 s maximum
  receive gap, above the 0.25 s coverage contract.
- ResetStopGate no longer depends on a ROS executor timer for HOLD coverage.
  One daemon wall-time thread publishes zero at the configured 20 Hz through
  the existing ResetStopGate publisher, so node identity and publisher GID do
  not multiply.  HOLD and held input callbacks publish zero immediately.
- A publication lock serializes heartbeat, command relay, HOLD/completion, and
  release.  Release leaves one final zero and stages status while live state is
  still held, then atomically disables heartbeat publication before a later
  input may relay.  No command is cached across the boundary.
- Heartbeat or command publication exceptions retain HOLD, clear eligibility,
  and expose a best-effort error status plus an in-process failure string.
  `close()` marks HOLD, stops and bounded-joins the daemon, then publishes the
  close boundary and destroys callback/subscription/publishers in order.
- Deterministic tests block executor progress by never spinning it and still
  observe continuous zero cadence; they cover release silence, fresh relay,
  stale generation, command publish failure, close/join/resource order, and
  idempotent close.
- Validation is recorded in `V6_ACTIVE_RESET_PROBE_20260822.md`.  Verdict:
  **PASS (code/build/unit only)**; fresh Attempt6 runtime remains **PENDING**.

## Attempt9 stale-reader backlog amendment

- Start HEAD: `22c7f5c75b46ad1cc0db69bd8c1d61f68b595c39`; fixed Module3
  main remained `22d66470c4b903349b2467dc876490bbebfc0083`.
- Immutable input:
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_active_reset_live_attempt9_20260822T023957Z`.
  Attempt9 remains **ENGINEERING FAIL / STOP / NOT FORMAL**.  Nav2 had already
  reported success and upstream `/cmd_vel_nav`, `/cmd_vel_smoothed`, and
  `/cmd_vel` reached zero by -0.0406/+0.0352/+0.0455 s around the route result,
  but `/cmd_vel_sim` still relayed nonzero commands through +0.4006 s and did
  not first reach zero until +0.5732 s.  The stale actuator vectors match
  approximately 0.44--0.52 s older `/cmd_vel` inputs.  Ground-truth yaw also
  continued by about 0.156 rad after terminal success.
- Root cause: Isaac services rclpy once per render frame.  ResetStopGate's
  reliable `KEEP_LAST depth=10` `/cmd_vel` reader could therefore drain an
  obsolete nonzero train after CollisionMonitor had already published its
  terminal zero.  RouteCoordinator/Nav2 terminal ordering was not the cause.
- Product fix: ResetStopGate's input reader is now explicit reliable/volatile
  `KEEP_LAST depth=1`; its `/cmd_vel_sim` publisher remains reliable depth 10.
  An unprocessed command is overwritten by the latest value, while the unique
  publisher, HOLD wall heartbeat, generation fence, and no-cache release
  semantics remain unchanged.  No executor-drain loop or settled-ack service
  was added.  The Jazzy parameter-callback handle is retained explicitly so
  `close()` removes the registered callback rather than `None`.
- Deterministic in-process rclpy coverage creates 32 nonzero commands and a
  final zero without spinning the gate node, then spins it repeatedly.  The
  requested QoS is asserted as compatible reliable/volatile `KEEP_LAST
  depth=1`, and the relay emits zero only; no stale train is observed.
- Source validation: ResetStopGate plus reset-service tests **36 passed**;
  active-reset probe plus retained package contracts **104 passed**;
  ActivationGate source-first tests **14 passed**.  Fresh isolated
  `robot_experiments` build/install passed at
  `/tmp/v6_attempt10_fix_build.vfIE1f`,
  `/tmp/v6_attempt10_fix_install.dCXUjX`, and
  `/tmp/v6_attempt10_fix_log.uE0yIV`; installed probe/package tests also
  passed **104 tests**, installed import resolved from that root, and the
  installed entry-point help passed.
- Verdict: **PASS (code/build/unit only)**.  No ROS graph, Isaac, Nav2,
  navigation, reset, evidence campaign, engineering run, or formal
  qualification was launched.  Attempt10 is **PENDING** and must verify the
  first `/cmd_vel_sim` zero within 0.25 s, no later nonzero, and bounded
  post-terminal ground-truth XY/yaw motion.
