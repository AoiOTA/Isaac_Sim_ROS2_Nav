# V6 RouteCoordinator reset retirement handoff

Date: 2026-08-22

## Scope and verdict

- Worktree: permitted Module3 worktree
  `/home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3`.
- Branch/start: `cognitive-navigation` at
  `baa05f96061bb8084c71d258216fe0aa105568cc` for this review-blocker
  amendment (the original retirement implementation started at
  `0a0fdf8adccaf95bfdf4d933993fa926ba6f3be4`).
- Fixed main ancestry: `22d66470c4b903349b2467dc876490bbebfc0083`.
- Goal: retire every RouteCoordinator-owned old-epoch route/action intent when
  `/simulation/reset_event` arrives, and prevent normal goal preemption from
  exposing an old tracker under a new request ID; close the multithreaded
  callback TOCTOU, Route Server graph-authority, stale rebuild, and transient
  runtime-state review blockers.
- Verdict: **PASS (code/build/unit only)**. No ROS graph, Isaac, Nav2,
  navigation, visual evidence, engineering campaign, or formal qualification
  was run.

## Implementation

- Added a synchronous reset retirement path. It captures the old request,
  goal, and accepted action handle; advances request and graph callback fences;
  then clears the active route, pending goal/prior, tracker, navigation flags,
  reroute state, transient runtime edges, old pose/costmap/TF/region cache, and
  pending structural candidate without triggering a structural rebuild.
- Cancellation is best effort and occurs only after coordinator state is
  cleared. A pending `NavigateToPose` future that accepts after its generation
  becomes stale is still consumed and its returned handle is immediately
  cancelled rather than retained.
- An active route reset publishes exactly one
  `/bio_nav/route_goal_complete=false` and one String JSON terminal on
  `/bio_nav/route_goal_result` with `request_id`, `status=aborted`,
  `reason=simulation_reset`, and the new `reset_epoch`. A reset with no active
  route publishes no fake terminal.
- New goals advance the request fence and synchronously clear the previous
  tracker/navigation state before the new pending goal or route context is
  visible. Existing preemption semantics remain non-terminal.
- Generation checks continue to discard late route and navigation results, so
  they cannot restore state or publish a second terminal after reset.
- A shared `RLock` now makes each route/action callback generation check and
  local state commit atomic with reset, preemption, and the progress timer.
  Completed futures are consumed outside the lock; publish, cancel, action,
  and service operations are also outside the state lock. Stale accepted
  `ComputeRoute` and `NavigateToPose` handles are cancelled.
- `desired_graph` is now the local authority for one serialized
  `SetRouteGraph` transaction. Reset and preemption force desired GVG even if
  the local graph is already GVG or an older cognitive/rebuild request is in
  flight. Every stale response is read; a stale success never commits local
  graph state and triggers a generation-fenced GVG compensation. A failed or
  incoherent graph blocks fresh route preparation and publishes a
  `LAST_KNOWN_GOOD` status.
- Structural rebuild requests capture request/reset/structural/desired tokens.
  A reset or fresh active goal makes a late rebuild stale, so its graph, map,
  support, and GVG state cannot overwrite the fresh epoch; a stale success is
  reconciled to the current desired graph.
- Reset publishes an explicit empty transient-local
  `/bio_nav/runtime_edge_states` snapshot using the desired GVG identity after
  clearing the internal runtime-edge map. Publication failure is visible in
  logs and structural status.

## Validation actually run

- Source-first/no-cache focused route/reset tests:
  `test_route_core.py test_cognitive_graph_adapter.py`: **73 passed**.
- Source-first/no-cache complete `robot_route_planner/test`: **97 passed,
  1 skipped**. The skip is the existing Isaac benchmark import because `pxr`
  is unavailable; Isaac was outside this task.
- Source-first/no-cache pure ActivationGate and mode-contract tests:
  **44 passed**. No ROS launch/integration test was run.
- Changed-file `python3 -m py_compile`: PASS.
- Fresh isolated package build: PASS at
  `/tmp/bionav_route_concurrency.j12692`.
- Fresh isolated `colcon test`: **98 tests, 0 errors, 0 failures, 1 skipped**
  (`97 passed`).
- `git diff --check`: PASS.

Tests cover active reset state retirement and one cancellation, single Bool and
JSON terminals, pending-action late acceptance cancellation, ignored late
results, no old-request timer/context/progress/lookahead/goal-update/canonical
publication, no fake no-active terminal, transient edge/cache cleanup, normal
preemption tracker retirement, and a fresh post-reset goal generation. New
deterministic coverage uses a real `threading.Barrier` to interleave reset with
an accepted-handle callback, and covers stale cognitive success compensation,
rebuild -> reset -> fresh goal -> late success, explicit empty runtime
snapshot identity, and consumption of stale futures without old-request
publications.

## Remaining live risk and next step

The cited review blockers are **closed in code/build/unit evidence only**. This
amendment does not prove executor/action-server timing, DDS publication
ordering, ActivationGate cancel-all interaction, or command behavior across a
real active reset. StopGate live closure remains **pending authorized live
review** with ROS/Isaac/Nav2 authority. That review must verify an
active goal reset produces one abort terminal, old request IDs never reappear,
late accepted handles are cancelled, no stale NavigateToPose or nonzero command
crosses StopGate release, and a fresh post-reset goal is the only route that can
resume motion.

## Final terminal and Route Server reassert-liveness amendment

- Start HEAD: `ea6f532554177f8256c194f67449dae622b009a8`; the reset gate,
  benchmark STOP evidence, route-generation fences, and passive IMU diagnostics
  already present at that HEAD are retained unchanged.
- Non-fallback `NavigateToPose` rejection now checks the route generation and
  synchronously retires the current route while holding the same output/state
  lock order used by reset. Rejection, reset, final success, and final failure
  each emit exactly one paired Bool plus JSON result. JSON includes
  `request_id`, `reset_epoch`, `status`, and `reason`; rejection uses
  `status=failed`, `reason=navigate_to_pose_rejected`. Fallback, intermediate
  lookahead success, preemption, and duplicate late callbacks remain
  non-terminal. A pending structural rebuild is triggered once after terminal
  retirement using the existing semantics.
- Route Server graph reconciliation now runs from a dedicated 0.1 s steady
  clock. Each transaction records its future, kind, and a 2.0 s steady deadline.
  Retry identity binds reset generation, desired-graph generation, optional
  route request ID, graph ID, and revision. Failures use bounded backoff
  0.25/0.5/1.0/2.0 s; one tick dispatches at most one transaction, and duplicate
  triggers cannot bypass an armed retry.
- Service unavailability, export/call exceptions, current rejection/exception,
  and timeout remain fail closed (`graph_coherent=false`, reassert required)
  and schedule retry without recursive submission. Timed-out futures are not
  cancelled. Every late response is consumed: a stale failure cannot damage a
  recovered coherent graph, while stale success fails closed and schedules a
  generation-fenced compensation. Cognitive and structural transactions both
  carry deadlines; a reset crossing either kind preserves the fresh desired-GVG
  retry. Service readiness, export, call, publish, cancel, and route preparation
  remain outside the route state lock.
- Deterministic coverage includes both reset/rejection barrier orders, duplicate
  rejection/result callbacks, final success/failure pairs, service unavailable,
  call exception, rejection backoff/no-storm, hung timeout, late failure after
  recovery, late-success compensation, cognitive/structural timeout crossing
  reset, frozen ROS time with advancing steady time, and pending-goal preparation
  only after successful reassert.
- Validation: focused route/graph **90 passed**; complete source-first/no-cache
  `robot_route_planner/test` **114 passed, 1 skipped** (existing `pxr` import is
  unavailable); associated reset-seed/ResetStopGate/IMU/MotionBenchmark/reset
  receipt regression **65 passed** after sourcing `/opt/ros/jazzy/setup.bash`;
  `py_compile` and `git diff --check` PASS. Fresh isolated package build and
  constrained `colcon test` PASS at
  `/tmp/v6_route_terminal_final_build.NRRMgN`, install
  `/tmp/v6_route_terminal_final_install.R7UZTF`, log
  `/tmp/v6_route_terminal_final_log.ujW3QY`; result: **115 tests, 0 errors,
  0 failures, 1 skipped**. An initial `colcon test` invocation from the broad
  workspace root stopped at duplicate package-name discovery and ran 0 tests;
  the cited rerun was constrained to the allowed Module3 `ros2_ws`.
- Verdict: **PASS (code/build/unit only)**. No ROS graph, Isaac, Nav2,
  navigation, evidence campaign, or formal qualification was run. Executor,
  action-server, DDS ordering, and live active-reset/reassert behavior remain
  unverified and require the already planned active-reset live review.

## Second concurrency-review blocker amendment

- Start HEAD: `5d4de361c5d1f7a9f7f6ea9e33b6ef36c04d18dd`; the MotionBenchmark STOP
  evidence repair at that HEAD is retained unchanged.
- Route terminals now use a dedicated output lock. A reset can retire route
  state concurrently, but its abort terminal cannot be followed by an older
  `NavigateToPose` rejection/result terminal. Async DynamicEdges,
  ComputeRoute, and NavigateToPose fallbacks carry the original route
  generation and cannot set `primary_fallback_used` for a fresh request.
- `graph_reassert_required` is independent of local graph identity. Reset,
  preemption, and stale successful graph callbacks keep routing fail closed
  until the desired GVG has received a successful Route Server transaction;
  the transaction is reserved before graph export, eliminating the previous
  coherent/no-transaction window.
- Cognitive candidate validation captures request, graph, reset generation,
  and reset epoch before physical validation. The same token is atomically
  rechecked before desired-graph/transaction reservation and again before
  `SetRouteGraph`; a validation crossing reset is discarded without changing
  fresh state.
- Every cognitive/fallback/structural graph export uses a new immutable temp
  directory whose prefix binds reset, desired, and switch generations. Files
  are complete before their unique GeoJSON path is passed to Route Server, so
  concurrent or stale exports cannot overwrite a submitted request path.
- Deterministic tests cover reset ordering against old navigation rejection,
  old ComputeRoute rejection against a fresh request, candidate validation
  crossing reset, fail-closed reassert during blocked export, and distinct
  non-cross-writing transaction paths.
- Validation: focused source-first/no-cache route/graph **78 passed**; complete
  `robot_route_planner` **102 passed, 1 skipped** (`pxr` unavailable); focused
  MotionBenchmark/reset-receipt/ResetStopGate/ActivationGate regression set
  **42 passed**; fresh isolated package build PASS and `colcon test` **103
  tests, 0 errors, 0 failures, 1 skipped** using
  `/tmp/v6_route_second_build.Xl1fS6` and
  `/tmp/v6_route_second_install.qGmRab`; `py_compile` and `git diff --check`
  PASS.
- Verdict: **PASS (code/build/unit only)**. No ROS graph, Isaac, Nav2,
  navigation, visual evidence, engineering campaign, or formal qualification
  was run. The active-reset live review described above remains pending.

## Deferred structural-rebuild intent amendment

- Start HEAD: `deb275252190a6eacbc9f08a0c9ad76773a172f4`; reset StopGate,
  MotionBenchmark evidence, IMU diagnostics, and prior route fixes at that HEAD
  are retained unchanged.
- A persistent structural-map candidate is now represented by one coalescing
  intent fenced by candidate generation/object identity, request ID, physical
  graph generation, and reset generation. Reset clears the intent; a new goal
  keeps the latest candidate pending and prevents structural submission until
  that route retires.
- Terminal route retirement still precedes its exactly-once Bool/JSON output.
  If a cognitive or structural `SetRouteGraph` transaction owns the server at
  that instant, the candidate is retained instead of being lost. Transaction
  success, rejection, exception, the 2 s steady timeout, late completion, and
  service recovery all flow through the existing serialized reconciliation and
  bounded 0.25/0.5/1.0/2.0 s retry path. No service, file, cancellation, or
  publication call was moved under the route-state lock.
- Latest-candidate coalescing prevents duplicate timer/callback wakeups from
  producing a request storm. A stale structural success cannot commit an old
  map; Route Server authority is first reconciled, then exactly one current
  idle candidate is submitted. Permanently unresolved timed-out futures remain
  an observed non-blocking resource risk because ROS futures are not cancelled.
- Deterministic tests cover cognitive switch success/rejection/exception,
  structural service unavailable/rejection/exception, both transaction timeout
  and late-success paths, reset clearing, active-goal delay followed by terminal
  wakeup, duplicate terminal/tick suppression, and latest-candidate coalescing.
- Validation: focused route/graph **101 passed**; complete source-first/no-cache
  `robot_route_planner/test` **125 passed, 1 skipped** (existing optional `pxr`
  import unavailable); associated ResetStopGate/MotionBenchmark/reset receipt,
  IMU/localization and EKF regression **87 passed** after sourcing Jazzy.
  `py_compile` and `git diff --check` PASS. Fresh isolated build and `colcon
  test` PASS at `/tmp/bio_nav_route_deferred.jMe0m0`: **126 tests, 0 errors,
  0 failures, 1 skipped**.
- Verdict: **PASS (code/build/unit only)**. No ROS graph, Isaac, Nav2,
  navigation, visual evidence, engineering campaign, or formal qualification
  was run. Active reset plus live structural-change timing remains pending the
  planned runtime review.

## Active-reset HOLD-intent amendment after live FAIL

- Start HEAD: `b2523a812c1fad832b9aa87622e30b60b8681015`; all prior
  reset-seed, ResetStopGate, benchmark STOP evidence, route concurrency, and
  deferred-structural-rebuild work is retained.
- Triggering evidence:
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_active_reset_live_20260821T190834Z`.
  The run is **ENGINEERING FAIL / NOT FORMAL**: StopGate entered generation-2
  HOLD first; wheel reset followed by about 59 ms, but the old Nav2 action
  terminal was not observed until about 186 ms and RouteCoordinator did not
  finish its event-only retirement until about 602 ms. This amendment does not
  reinterpret or promote that failed run.
- RouteCoordinator now subscribes to the existing reliable/transient-local
  `/simulation/reset_stop_gate/status` JSON. A strictly valid higher-generation
  `held=true, reason=hold` is the reset-begin authority: it fences every old
  route/action callback generation, retires tracker/prior/runtime/structural
  and graph intent, cancels the captured action handle, and emits at most one
  active-route Bool/JSON abort terminal with `reason=simulation_reset` and the
  new reset epoch. A released startup snapshot only synchronizes baseline
  generation and emits no terminal.
- The same-generation `/simulation/reset_event` is completion, not a second
  reset begin: it publishes the empty runtime snapshot and starts the existing
  desired-GVG reconciliation without a second terminal or epoch bump. A goal
  remains rejected while HOLD is active; same-generation release opens the
  barrier only after completion. Missing completion stays fail closed. Exact
  duplicates are idempotent; malformed, incoherent, backward, or conflicting
  status remains held and is logged. The legacy event-only path is retained for
  a coordinator fixture/process with no status-backed reset intent.
- Route context/cost, DynamicEdges/ComputeRoute dispatch, canonical route,
  progress/lookahead/goal-update, and NavigateToPose dispatch share the terminal
  output lock at their final generation check. Therefore HOLD cannot retire a
  request between its last check and an old-request publication or dispatch.
  Runtime/structural/cognitive route inputs and timers are ignored while held.
- Mapping, diagnostic, and R2C launch topology is unchanged; this patch neither
  changes ResetServiceBridge event order nor adds a reset acknowledgement.
  Reliable/transient-local status is the configured interface, not a claim of
  formal DDS delivery or executor-order proof.

Validation actually run:

- Source-first/no-cache complete `robot_route_planner/test`: **132 passed,
  1 skipped** (existing `pxr` import unavailable).
- Associated source-first ResetStopGate, ActivationGate, mode/Nav profile, and
  MotionBenchmark regression: **98 passed**; combined rerun: **230 passed,
  1 skipped**.
- Changed-file `py_compile` and `git diff --check`: PASS.
- Fresh isolated `robot_route_planner` build: PASS at
  `/tmp/v6_route_hold_build.HMD46a`, install
  `/tmp/v6_route_hold_install.PohSre`, log
  `/tmp/v6_route_hold_log.74aLob`. Fresh isolated `colcon test`: **133 tests,
  0 errors, 0 failures, 1 skipped**, log
  `/tmp/v6_route_hold_test_log.95Ce9m`.
- Unit coverage includes startup released baseline, immediate active HOLD abort,
  status/event/release idempotence, release-before-event fail-closed behavior,
  no-event permanent HOLD, rejected goals, late accept cancellation, late result
  silence, both HOLD/Nav-terminal lock orders, malformed/backward status, no old
  timer/output, legacy event fallback, and a fresh post-release goal.

Verdict: **PASS (code/build/unit only)**. No ROS graph, Isaac, Nav2,
navigation, visual evidence, engineering campaign, or formal qualification was
run for this amendment. The required next step is a fresh active-reset live
rerun from the resulting commit; executor timing, cancellation completion,
observed status/event order, HOLD displacement, and fresh-goal recovery remain
runtime-pending.

## HOLD-crossing external-input fence amendment

- Start HEAD: `600981e28616709e1937046e90e715f1a36f4d21`; the prior route-retirement,
  reset-gate, benchmark evidence, and IMU evidence-contract commits are retained.
- Added one immutable RouteCoordinator input token binding reset intent,
  completion, route request, and physical-graph identity. Runtime observations,
  runtime expiry, structural-map observation/rebuild, Module2 priors, cognitive
  candidates, region selection, odometry/costmap context, and their timers now
  capture this token under the shared state lock and revalidate it before any
  reset-owned state commit. Output and dispatch paths keep the established
  output-lock then state-lock final check.
- Runtime edge mutation and its transient snapshot are atomic. DynamicEdges
  cost construction snapshots `route_cost_view` under the state lock. A HOLD
  crossing cannot recreate edge 77, publish a stale runtime snapshot, retain a
  stale prior, queue/rebuild a structural candidate, select a region, or emit an
  immature cognitive-graph status. Structural observation runs on a private
  monitor snapshot and commits only if the observation generation and route
  token are still current; no TF, publish, service, action, export, or graph
  build runs under the state lock.
- Same-generation completion/release, desired-GVG reassertion, deferred
  structural coalescing, and exactly-once terminal behavior are unchanged.
  Deterministic barriers cover the six crossing paths above; the runtime edge
  race is repeated for 40 rounds.

Validation actually run:

- Deterministic HOLD-crossing subset: **6 passed**.
- Complete source-first/no-cache `robot_route_planner/test`: **138 passed,
  1 skipped**; the existing optional `pxr` import remains unavailable.
- Risk-related ResetStopGate/ActivationGate/mode/MotionBenchmark/reset-receipt/
  IMU/localization/EKF regression set: **174 passed**.
- Changed-file `py_compile` and `git diff --check`: PASS.
- Fresh isolated `robot_route_planner` build: PASS at
  `/tmp/v6_route_hold_fence_build.bZ2EMi`, install
  `/tmp/v6_route_hold_fence_install.mkbJi2`, log
  `/tmp/v6_route_hold_fence_log.9otDEl`. Fresh `colcon test-result`: **139
  tests, 0 errors, 0 failures, 1 skipped**.

Verdict: **PASS (code/build/unit only)**. No ROS graph, Isaac, Nav2,
navigation, visual evidence, engineering campaign, or formal qualification was
run. A fresh active-reset live rerun remains required to verify executor/DDS
ordering, zero old-epoch output and command through HOLD, and fresh-goal recovery.

## HOLD graph-output and Route Server dispatch amendment

- Start HEAD: `dc09e9eebb0c73cc23d4c72eddf0e6bdc05a9944`; all earlier
  route-retirement, deferred-rebuild, reset-stop, benchmark, and IMU work is
  retained unchanged.
- Cognitive graph export errors, service-unavailable results, request-call
  exceptions, validation acknowledgements, fallback outcomes, graph/runtime
  publications, and structural status now use the established output-lock then
  state-lock final fence. The captured route-input identity and cognitive
  validation identity must still be current and HOLD must remain open before
  an old callback can emit an output.
- Cognitive and structural `SetRouteGraph` final submission now holds the
  output lock from its last state/token/transaction check through request
  construction, `call_async`, future registration, and callback registration.
  No service call is made while the state lock is held. A HOLD that wins before
  this critical section yields zero request; once submission owns the output
  lock, reset retirement cannot interpose between the final check and dispatch.
- Reset completion carries an explicit status-generation token. Only that
  completion-owned, still-current GVG reassert may submit while HOLD remains
  active; ordinary cognitive, fallback, retry, and structural submissions stay
  fenced. The token survives bounded retry and cannot authorize a later reset.
- Deterministic tests cover export error, service unavailable, request
  exception, cognitive/structural final dispatch, fallback-success outputs,
  old fallback request/status, held reset-GVG reassert success, and fresh
  same-generation release dispatch.
- Validation: focused graph adapter **64 passed**; complete source-first
  `robot_route_planner` **147 passed, 1 skipped** (`pxr` unavailable); the
  associated reset/gate/benchmark/IMU/localization/EKF static regression set
  passed **200 tests**. Fresh isolated package build passed at
  `/tmp/v6_route_dispatch_build.Qy6KPR` with install
  `/tmp/v6_route_dispatch_install.Hf0ucn`; final `colcon test-result` was **148
  tests, 0 errors, 0 failures, 1 skipped**, log
  `/tmp/v6_route_dispatch_final_test_log.DNvdJi`. Changed-file `py_compile`
  and `git diff --check` passed.
- Verdict: **PASS (code/build/unit only)**. No ROS graph, Isaac, Nav2,
  navigation, visual evidence, engineering campaign, or formal qualification
  was run. The fresh active-reset live rerun remains required.

## Reset-completion graph-retry authority amendment

- Start HEAD: `819e519484a1b837ca2817465edabe6cda6d9b41`; the prior HOLD dispatch,
  route-retirement, reset-stop, benchmark, and IMU commits remain retained.
- A retry switch context now binds its completion token to the captured
  `RouteInputGeneration`, coordinator reset generation, and desired retry key.
  When reset completion encounters an in-flight cognitive/structural graph
  transaction or an existing same-key retry, that current completion-owned
  context is merged instead of discarded. It outranks ordinary or stale
  callback contexts for the same generation/key; a newer HOLD/reset replaces
  it. Backoff and same-key duplicates remain no-storm.
- A successful held GVG reassert retains a refreshed completion context so a
  later old success that changes Route Server authority can schedule exactly
  one compensating GVG reassert. Old success, rejection, exception, timeout,
  and late settlement cannot clear the current completion context. Retry
  dispatch reuses the captured route-input fence and token.
- Graph-switch failure and stale-success status output now shares a final
  route-input generation check. An old fallback rejection/exception is silent
  after HOLD, and old stale success remains silent after release. Only an
  explicitly current completion-owned GVG context may authorize failure output
  during HOLD.
- Deterministic coverage includes cognitive and structural in-flight success/
  rejection, timeout plus late success, existing same-key retry, newer reset
  invalidation, no-storm dispatch, old fallback rejection/exception, current
  reset-completion failure, and old success after release. The runtime-edge
  HOLD crossing race now repeats for 100 rounds.
- Validation: focused graph adapter **76 passed**; complete source-first
  `robot_route_planner` and fresh installed colcon test each **159 passed, 1
  skipped** (existing optional `pxr` unavailable); associated reset/gate/mode/
  benchmark/reset-receipt/IMU/localization/EKF set **203 passed**. Fresh
  isolated package build/test used `/tmp/v6_route_retry_build.4tnZkN`, install
  `/tmp/v6_route_retry_install.tTUkd7`, and log
  `/tmp/v6_route_retry_log.8PfwFz`; `colcon test-result` reported **160 tests,
  0 errors, 0 failures, 1 skipped**. Changed-file `py_compile` and
  `git diff --check` passed.
- Verdict: **PASS (code/build/unit only)**. No ROS graph, Isaac, Nav2,
  navigation, visual evidence, engineering campaign, or formal qualification
  was run. The planned fresh active-reset live rerun remains required to
  verify executor/DDS ordering, observed GVG reconciliation, zero old-epoch
  output/command through HOLD, and fresh-goal recovery.

## Late-join startup completion baseline amendment

- Start HEAD: `ba941acddadc785fd0938be24e28ef13fd65f8a5`; the prior route-reset,
  ResetStopGate, benchmark, IMU, and LiDAR-readiness amendments remain retained.
- Live Attempt 2 evidence:
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_active_reset_live_attempt2_20260821T212047Z/`.
  Verdict: **ENGINEERING FAIL / STOP / NOT FORMAL**, before any goal or Trigger
  request. At `1787347473.815673754`, RouteCoordinator joined after Isaac's
  startup reset and rejected transient-local generation 1 `reset_complete` as
  `higher generation did not begin with HOLD`. At `1787347479.391320030`, it
  rejected generation 1 `released:activation_gate` as
  `same-generation transition without reset HOLD`; route goals remained held.
  Primary evidence is `summary.md`, `metrics/status_sequence.json`, and
  `setup_bag_analysis.json` in that directory.
- Root cause: the reliable/transient-local ResetStopGate status intentionally
  has depth 1. A late-joining RouteCoordinator can therefore receive the
  current `reset_complete` snapshot without the earlier HOLD event. The prior
  state machine treated this expected startup snapshot as an invalid runtime
  transition and could never accept the same-generation release.
- Change: one strict startup-only completion baseline is accepted only when no
  reset status/intent exists, all route and input generations are pristine,
  there is no active/pending navigation or route, and no graph/retry/structural
  transaction exists. It synchronizes the status intent and completed
  generation, advances the coordinator reset epoch, emits the empty runtime
  snapshot, and requests the completion-owned GVG reassert while retaining the
  HOLD barrier. It emits no route terminal, performs no navigation cancel, and
  does not fabricate a HOLD. Only a legal same-generation release opens the
  barrier. Duplicate completion is idempotent; backward/conflicting state,
  active/old state, and later runtime completion without HOLD remain fail
  closed. The existing first-released startup baseline and legacy Empty-event
  path are unchanged.
- Deterministic tests cover startup completion/release/fresh-goal preparation,
  no fake terminal, duplicate/backward/conflict, old-state rejection, later
  completion without HOLD, the existing first-released baseline, and
  completion reconciliation ordering against a concurrent release callback.
- Validation: focused RouteCoordinator file **64 passed**; complete source-first
  route package **164 passed, 1 skipped** (existing optional `pxr` unavailable).
  Fresh isolated build/test passed with build
  `/tmp/v6_route_startup_build.6AfGxd`, install
  `/tmp/v6_route_startup_install.sqc5Ow`, and log
  `/tmp/v6_route_startup_log.LPvRP3`; `colcon test-result` reported **165 tests,
  0 errors, 0 failures, 1 skipped**.
- Verdict: **PASS (code/build/unit only)**. Attempt 2 remains STOP evidence and
  is not promoted. Active-reset live Attempt 3 is **PENDING** and must confirm
  actual DDS late-join ordering, successful startup release and fresh goal,
  then one active reset with zero old-epoch output/command and fresh-goal
  recovery. No Isaac, ROS graph, navigation, visual review, engineering PASS,
  or formal qualification was run by this amendment.

## Cross-topic late-join reset merge amendment

- Start HEAD: `aab15690c64017775ec025acd51fe03f0cf5c210`; the prior
  route/reset, benchmark, IMU evidence, and LiDAR-readiness commits remain
  retained. Triggering evidence remains live Attempt 2 at
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_active_reset_live_attempt2_20260821T212047Z/`;
  it remains **ENGINEERING FAIL / STOP / NOT FORMAL**. Attempt 3 is pending.
- Root cause extension: `/simulation/reset_event` is volatile while
  `/simulation/reset_stop_gate/status` is reliable/transient-local depth 1.
  Their callbacks therefore have no cross-topic total order. The prior
  startup-completion amendment covered completion-first only; event-first was
  irreversibly handled as legacy, and a HOLD followed by `reset_complete` still
  depended on receiving the volatile Empty event.
- Change: a pristine first Empty now advances the coordinator reset state once
  but remains an unbound completion hint for 0.5 steady seconds. A first strict
  HOLD, `reset_complete`, or released snapshot binds that already-completed
  physical reset to its generation without a second request/epoch advance.
  Completion/release reconciliation publishes one empty runtime snapshot and
  one completion-owned GVG reassert before release opens the route barrier. A
  same-generation `reset_complete` after HOLD now performs the same completion
  work even when Empty was missed; a later Empty is idempotent. Event-only
  legacy deployments resume after the bounded grace. Non-pristine missing-HOLD
  status sequences, late status after legacy resolution, malformed, backward,
  and conflicting transitions remain fail closed.
- Deterministic tests cover event/completion/release, completion/event/release,
  event/HOLD/completion/release, HOLD/completion/release without Empty,
  HOLD/event/completion/release, event/release with completion missed, bounded
  event-only fallback, non-pristine missing-HOLD orders, and 500 concurrent
  event/completion rounds. Existing active HOLD retirement/cancel/terminal,
  callback/output fences, graph transaction/retry, and deferred rebuild tests
  remain passing.
- Validation: focused RouteCoordinator **75 passed**; complete source-first
  route package **175 passed, 1 skipped** (optional `pxr` unavailable). Fresh
  isolated build/test passed with build
  `/tmp/v6_route_cross_topic_build.OkpE4E`, install
  `/tmp/v6_route_cross_topic_install.OXFzsb`, and log
  `/tmp/v6_route_cross_topic_log.GLwmz6`; `colcon test-result` reported **176
  tests, 0 errors, 0 failures, 1 skipped**. Changed-file `py_compile` and
  `git diff --check` passed. The first build command placed global
  `--log-base` after the `build` subcommand and exited during argument parsing;
  it performed no build, and the corrected fresh invocation above passed.
- Verdict: **PASS (code/build/unit only)**. No ROS graph, Isaac, Nav2,
  navigation, visual evidence, engineering campaign, or formal qualification
  was run. Active-reset live Attempt 3 remains required to validate real DDS
  ordering, startup release, one active reset, zero old-epoch output/command,
  and fresh-goal recovery.

## Startup hint authority/deadline safety amendment

- Start HEAD: `bccbd3e00cf32686434b4f0f6dc21f7cc19b9c74`; scope is limited to
  RouteCoordinator reset-status handling, its deterministic tests, and this
  handoff/ledger. IMU, ResetStopGate producer, ActivationGate, critic, command,
  and event contracts are unchanged.
- A malformed, backward, or conflicting status now atomically marks strict
  status authority as observed, retires the unbound startup Empty and its
  steady timer, and keeps the route barrier held. That timer can no longer
  publish completion or reopen the route after the hint is invalidated.
- Normal `initialized`/`closed` snapshots also establish status authority. An
  Empty arriving before or after that baseline remains held with legacy fallback
  disabled; a later HOLD binds the same already-completed physical reset, then
  completion/release performs exactly one request/epoch advance, empty runtime
  snapshot, and GVG reconciliation. Both callback orders and both baseline
  reasons are covered.
- Hint binding checks the steady deadline while holding the same output/state
  locks used by the timer and status callback. An already-expired hint is
  deterministically retired before a HOLD can bind it; a later strict reset is
  a new reset rather than a merge. Event-only operation with no observed status
  authority retains the bounded legacy fallback.
- Deterministic coverage includes malformed/conflicting status plus timer,
  `initialized`/`closed` before and after Empty, deadline expiry before timer
  callback, late baseline after expiry, both timer/status orders, a new strict
  reset after legacy expiry, and 500 concurrent baseline/Empty rounds with no
  double request/epoch/completion. Existing active-reset, output, graph retry,
  and callback fences remain in the complete package regression.
- Validation: focused RouteCoordinator **86 passed**; complete source-first
  `robot_route_planner` **186 passed, 1 skipped** (existing optional `pxr`
  unavailable); associated reset/gate/mode/benchmark/IMU/localization/EKF
  regression **232 passed**. Fresh isolated package build/test passed at
  `/tmp/v6_route_hint_build.NVrz8b`, install
  `/tmp/v6_route_hint_install.bTHSo4`, and log
  `/tmp/v6_route_hint_log.FK8WjM`; `colcon test-result` reported **187 tests,
  0 errors, 0 failures, 1 skipped**. Changed-file `py_compile` and
  `git diff --check` passed. The first associated-test command stopped during
  collection on the stale repository overlay's pre-V6 `bio_nav_interfaces`;
  the cited rerun sourced the allowed Integration worktree generated interface
  first and passed.
- Verdict: **PASS (code/build/unit only)**. No ROS graph, Isaac, Nav2,
  navigation, visual evidence, engineering campaign, or formal qualification
  was run. Active-reset live Attempt 3 remains **PENDING**.

## Attempt5 reset-GVG READY amendment

- Start HEAD: `5dc3b91e9922a12b45e63b135342350e5e847a33`; triggering evidence is
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_active_reset_live_attempt5_20260822T001729Z`.
  Attempt5 remains **ENGINEERING FAIL / STOP / NOT FORMAL** and is not rerun.
- The finalized bag contained the reset graph-set request but no reset-era
  GVG `READY`.  A successful completion-owned reassert could commit while the
  reset HOLD barrier was active; its callback correctly suppressed output, but
  the later release had no path to publish the now-coherent state.
- RouteCoordinator now tracks the generation whose reset GVG READY was
  published.  On a legal same-generation release it publishes exactly one
  `StructuralGraphStatus.READY` with detail `reset GVG reconciled` only when
  desired graph, local graph, and GVG identities match, graph coherence is
  true, and no graph transaction, retry deadline, reassert, or cognitive switch
  is pending.
- If reassert commits after release, the generation-fenced completion callback
  publishes the same READY.  A newer reset invalidates the token.  Startup
  released baselines, stale generations, service unavailable/retry state, and
  incoherent graphs cannot fabricate READY.
- Tests cover success committed during HOLD then release, duplicate release,
  pending/unavailable reassert, and success completing after release.  The
  wider reset/retry/concurrency suite remains passing as recorded in
  `V6_ACTIVE_RESET_PROBE_20260822.md`.
- Verdict: **PASS (code/build/unit only)**.  Fresh Attempt6 runtime remains
  **PENDING**.
