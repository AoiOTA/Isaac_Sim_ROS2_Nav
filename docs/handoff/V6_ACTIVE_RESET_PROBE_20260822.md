# V6 active-reset exactly-once probe

## Scope and status

This amendment adds a single-process `active_reset_probe` runner for the next
fresh active-reset attempt.  It replaces the split Attempt4 harness timing
with one monotonic state machine and an atomically refreshed JSON receipt.

This is **PASS (code/build/unit only)**.  Attempts 5 and 6 are retained as
**ENGINEERING FAIL / STOP / NOT FORMAL**; this amendment does not rerun them.
No ROS graph, Isaac, Nav2, navigation, reset, bag, visual review, engineering
campaign, or formal qualification was run by this task.  Attempt7 remains
**PENDING**.

## Attempt4 input

The bounded prior evidence is:

`/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_active_reset_live_attempt4_20260821T224036Z`

Attempt4 remains **ENGINEERING FAIL / STOP / NOT FORMAL**.  Its exactly-once
old G2 request was consumed and moved the robot, but the external harness did
not surface the active boundary before the route later failed.  No Trigger
reset or fresh goal was sent.  This amendment does not reinterpret or modify
that evidence.

## Runner contract

The entry point `active_reset_probe` executes:

`WAIT_ENDPOINTS -> PUBLISH_OLD_ONCE -> WAIT_ACTIVE_READY -> CALL_RESET_ONCE -> OBSERVE_HOLD_ABORT -> WAIT_RELEASE -> QUIET -> PUBLISH_FRESH_ONCE -> WAIT_SUCCESS -> POSTZERO -> PROVISIONAL_COMPLETE`

It fails closed to `STOP` and exits nonzero on the first contract violation.
Wall deadlines use `time.monotonic()`; ROS time is used only for published
message stamps.  The route-goal publisher is reliable/volatile and remains
alive for the whole process.

Key checks are (including the review-blocker amendment at Module3 start
`24136355b632b3b8e9abc30c790ef939e4c9438b`):

- use endpoint-info APIs, not scalar counts, to require exactly one
  `/cmd_vel` publisher `/collision_monitor`, exactly one `/cmd_vel_sim`
  publisher `/isaac_navigation_sim` (ResetStopGate authority), exactly one
  probe publisher, and the exact configured route-goal subscribers. Attempt5
  defaults are `/bio_nav_route_coordinator` plus `/rosbag2_recorder`;
- record node names, namespaces, GIDs and counts, then require the exact graph
  to remain unchanged at `prepublish`, `pre_reset`, `post_release`, and
  `pre_fresh` before any associated side effect;
- wait for the Trigger service, all observed topic publishers, finite
  `/ground_truth/odom` in normalized frame `map`, finite official `/odom` in
  normalized frame `odom`, and exactly one retained generation-1 release.
  Position, quaternion, source stamp, receive time, and every six-component
  Twist must be finite; NaN/Inf is an immediate STOP, never a zero sample;
- publish old G2 `(0.8, 4.8, -2.792526803)` exactly once;
- within six seconds observe one new canonical request, matching progress,
  lookahead and goal-update, at least five nonzero `/cmd_vel_sim` messages,
  at least 0.10 m GT displacement, no collision, and no terminal;
- synchronously recheck topology, then sample the actual dispatch monotonic
  time immediately before `call_async`; call `/simulation/reset` exactly once
  only when that time is within 0.5 s of `active_ready`;
- strictly validate seed 8601, generation 2, pose `long_route_start_g1`, and
  odometry `realistic` from the reset receipt;
- reject any pre-dispatch generation above the sole generation-1 released
  baseline, then observe the exact same-topic generation-2
  `hold -> reset_complete -> released:*` receive sequence. HOLD-to-release
  `/cmd_vel_sim` and collision, plus stable-to-release GT and EKF odometry,
  each require at least two samples, both edges within 0.25 s and no gap over
  0.25 s. Commands must all be zero, collisions all false, and GT/odom spans
  no greater than 0.02 m. GT landing is checked against map `(0.45,-5.35)`;
  realistic EKF odometry landing is separately checked against odom `(0,0)`;
- require one old Bool `false` plus exact JSON
  `aborted/simulation_reset/reset_epoch=2` for the old request, one reset
  event, and an exact reset receipt;
- from Trigger dispatch until the received generation-2 HOLD, old route output
  is recorded as `pre_hold_inflight_outputs` but is not misclassified as a
  barrier violation.  Dispatch-to-HOLD receive latency must be no greater than
  0.5 s or the probe STOPs.  From received HOLD until the exact old terminal
  pair completes, canonical route, progress, lookahead, and goal-update/
  Navigate intent are recorded as `post_hold_pre_retirement_inflight` with
  counts, receive times, and available request IDs.  This bounded interval is
  at most 0.25 s; a mismatched request ID, late output, late/missing terminal,
  or wrong terminal pair STOPs;
- establish the coordinator retirement fence only when the probe has received
  both old Bool `false` and exact JSON
  `aborted/simulation_reset/reset_epoch=2`.  Any old output after that fence
  immediately STOPs.  Require one second of quiet from that received-pair
  fence, then publish fresh goal
  `(0.685, -3.975535, 1.570796327)` exactly once;
- require a strictly newer request ID, the exact configured canonical edge
  list (default `[51,52]`, with no support-equivalent escape), Bool `true`,
  exact JSON `succeeded/final_goal_distance_confirmed/reset_epoch=2`, GT error
  no greater than 0.30 m, and no collision within 30 seconds;
- for the complete final 1.0 s require at least two fresh samples on each of
  the four command-chain topics, both edges within 0.25 s, no gap over 0.25 s,
  and every sample zero.

Generation-2 duplicates, extras, regression, advance, or conflicts remain
fail-stop after release and through a final 0.25 s provisional monitoring
window. A nonzero command in that window also STOPs.

The output JSON is rewritten via file and parent-directory `fsync` plus
`os.replace` at a bounded 20 Hz
and forced at every state transition.  Trigger dispatch happens before the
state-transition write, so NAS latency is outside the 0.5 s boundary. It
records phase/verdict/STOP reason, counts, exact request
IDs and routes, reset receipt and gate sequence, monotonic boundary times,
`active_ready`-to-reset delay, GT displacement/drift/error, collision, and
post-stop command evidence. Endpoint, publish, service dispatch, done-callback,
JSON-write, spin and teardown exceptions fail to an atomically retried terminal
STOP document; publish-attempt counters are advanced before the side effect so
an exception cannot cause a retry. The state machine can only reach
`PROVISIONAL_COMPLETE`. Only after `destroy_node`, `rclpy.shutdown`, and a final
atomic/fsynced JSON succeed does the process return zero with verdict
`PASS_REQUIRES_BAG`. Persistence or teardown failure returns nonzero and makes
best-effort writes to both the final STOP path and a distinct emergency STOP
path.

Callback monotonic times do **not** prove cross-topic source order, and the
terminal-pair fence does not claim a DDS total order. In-process
completion is `PROVISIONAL_PASS_REQUIRES_BAG_ORDER`; after successful teardown
and final persistence the verdict is `PASS_REQUIRES_BAG`. Both explicitly keep
`engineering_pass=false`. Only an independent finalized-bag ordering analysis
may promote the fresh Attempt7 episode to engineering PASS.

Example operator invocation after a fresh archive build:

```bash
ros2 run robot_experiments active_reset_probe \
  --output "$evidence_root/probe/active_reset_probe.json"
```

Do not reuse any Attempt4/5/6 runtime state.  The next reviewer must start
a fresh isolated episode, bag the contracted topics independently, and compare
the probe JSON with the finalized bag before making an engineering-runtime
claim.

## Attempt5 failure input and closure amendment

The immutable input is:

`/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_active_reset_live_attempt5_20260822T001729Z`

Attempt5 remains **ENGINEERING FAIL / STOP / NOT FORMAL**.  It dispatched the
single Trigger 0.000621 s after `ACTIVE_READY`.  One old progress/lookahead/
goal-update triplet arrived about 0.260 s after dispatch and about 0.012 s
before the received generation-2 HOLD.  The old probe stopped at dispatch,
but finalized-bag review found no route output at or after HOLD.  The probe now
records that bounded pre-HOLD interval instead and makes the actual received
HOLD the old-output fence, with a separate 0.5 s dispatch-to-HOLD deadline.

Attempt5 also exposed two independent blockers now covered by source changes:

- `/cmd_vel_sim` was all zero during HOLD, but its maximum receive gap was
  0.378262 s.  ResetStopGate now uses a daemon wall-time zero heartbeat rather
  than an executor timer, retaining the same node and publisher GID.  HOLD is
  immediate, release excludes the heartbeat before relaying, and close stops
  and joins the thread before destroying ROS resources.
- the bag contained no reset-era GVG `READY`.  RouteCoordinator now publishes
  exactly one `READY` with detail `reset GVG reconciled` after the same
  generation is released and the desired/local/GVG graph is coherent with no
  transaction, retry, or reassert pending.  A reassert completing after release
  publishes from its callback; unavailable or pending service state cannot
  claim READY.

The Attempt5 exact seed 8601 receipt, one reset event, same-topic
`hold -> reset_complete -> released:activation_gate`, old abort terminal pair,
zero collision, and stable GT/EKF landing remain useful bounded evidence, but
do not promote the stopped episode.  Attempt6 must be fresh.

## Attempt6 failure input and retirement-fence amendment

The immutable input is:

`/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_active_reset_live_attempt6_20260822T010744Z`

Attempt6 remains **ENGINEERING FAIL / STOP / NOT FORMAL**.  The probe called
Trigger exactly once and received generation-2 HOLD 0.175794 s later.  The
finalized bag then recorded the old request-2 progress/lookahead/goal-update
triplet at about 0.0076 s after HOLD, followed by the matching Bool `false`
and exact `aborted/simulation_reset/reset_epoch=2` JSON at about 0.0089 s.
No old route output was recorded after that terminal pair.  The stopped run
did not publish or test the fresh route.

The triplet and HOLD came from different DDS writers.  Source locking and the
bag are consistent with already-published cross-topic in-flight delivery, but
cannot prove a total order.  The probe therefore uses the coordinator-owned
old terminal pair as the retirement fence: the HOLD-to-pair interval is
observable and bounded to 0.25 s, while any old output after pair completion
is fail-stop.  This is an engineering observation contract, not a
reinterpretation or promotion of Attempt6.

Attempt7 must use a fresh isolated runtime and finalized bag.  It remains
**PENDING**.

## Deterministic validation

Pure state-machine coverage includes exact endpoint identities and graph-change
STOP, endpoint/subscriber wait, startup
retained-event exclusion, exactly-once publication/service calls, active
timeout, terminal-before-reset, reset-call delay, receipt mismatch, gate order,
duplicates/races, service failure, HOLD/GT/odom/collision coverage and leakage,
reset teleport versus post-landing drift/error, old-output silence, strict
request/epoch/reason/edge identity, fresh success/failure, the four-chain full
postzero contract, and atomic entrypoint STOP output.

The Attempt5 closure amendment validated source-first **214 passed** across
ResetStopGate, active-reset probe, RouteCoordinator core, and cognitive graph
adapter tests.  The same **214 passed** against the fresh install.  A wider
source-first suite produced **819 passed, 1 skipped**, plus one unrelated
path-sensitive Rivermark frozen-reference failure whose stored absolute path
names an older worktree.  Package-configured probe flake8, `py_compile`, and
`git diff --check` passed.  Fresh isolated build/install passed for
`robot_route_planner` and `robot_experiments` at
`/tmp/v6_attempt5_closure_build.g7Sp4d`, install
`/tmp/v6_attempt5_closure_install.a1hBg5`, and log
`/tmp/v6_attempt5_closure_log.A2CV4w`.  Installed import paths, `ros2 run ...
active_reset_probe --help`, and the direct installed entry point passed.  No
ROS graph, Isaac, Nav2, navigation, reset, or evidence episode was launched by
this amendment.

The Attempt6 retirement-fence amendment validated source-first **82 passed**
(44 probe-state plus 38 retained package-contract tests) and the same
**82 passed** against the fresh install.  Package-configured flake8,
`py_compile`, and `git diff --check` passed.  A clean-environment isolated
build/install passed for `robot_experiments` at
`/tmp/v6_probe_retirement_final_build.iQHvBF`, install
`/tmp/v6_probe_retirement_final_install.m5R4AB`, and log
`/tmp/v6_probe_retirement_final_log.RRYMMs`.  The installed import path,
`ros2 run ... active_reset_probe --help`, and direct installed tests passed.
No ROS graph, Isaac, Nav2, navigation, reset, evidence episode, engineering
campaign, or formal qualification was launched by this amendment.

## Attempt7 reset-owned subscriber GID-rotation amendment

The immutable input is:

`/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_active_reset_live_attempt7_20260822T013726Z`

Attempt7 remains **ENGINEERING FAIL / STOP / NOT FORMAL**.  Its `prepublish`
and `pre_reset` topology snapshots were exactly stable.  At `post_release`,
the sole `/cmd_vel_sim` subscription owned by
`/_World_Graphs_Control_SubscribeTwist` retained its node name, namespace,
topic type, endpoint type and count, but its GID changed from
`010fa6bfc4c415930100000000000604` to
`010fa6bfc4c415930100000000001604`.  The old probe treated that reset-owned
replacement as an arbitrary topology change and stopped before publishing the
fresh route.

The probe now admits at most one such replacement after reset release.  Every
semantic endpoint and GID remains strictly identical from `prepublish` through
`pre_reset`.  At `post_release` or `pre_fresh`, the only allowed delta is one
old GID disappearing and one new GID appearing for the exact reset-owned
`geometry_msgs/msg/Twist` subscription; endpoint overlap, count changes,
identity/type changes, publisher changes, unrelated GID changes, reversion, or
a second rotation remain fail-stop.  Accepted changes are written to
`topology_gid_rotations` with topic, direction, node, old/new GIDs and the
checkpoint.  The `pre_fresh` snapshot must preserve the accepted replacement.

This is a discrete graph-snapshot contract.  It does not prove that no
instantaneous DDS endpoint overlap occurred between checkpoints.
`/cmd_vel_sim` callback coverage and zero/nonzero behavior remain an independent
functional authority check and are not inferred from topology identity.

Source-first focused tests passed **54 tests**; the same **54 tests** passed
against the fresh install.  Package-configured flake8, `py_compile`,
`git diff --check`, installed import-path assertion and installed entry-point
help passed.  Fresh isolated `robot_experiments` build/install passed at
`/tmp/v6_probe_gid_rotation_build.tQuspc`,
`/tmp/v6_probe_gid_rotation_install.NC1gLQ`, and
`/tmp/v6_probe_gid_rotation_log.pnCbdh`.

Verdict: **PASS (code/build/unit only)**.  No ROS graph, Isaac, Nav2,
navigation, reset, evidence episode, engineering campaign, or formal
qualification was launched.  Attempt8 remains **PENDING** and must use a fresh
isolated runtime plus finalized bag.

### Late-first GID-rotation checkpoint tightening

The reset-owned replacement is now admissible only when first observed at
`post_release`.  If `post_release` is still exactly the baseline snapshot,
`pre_fresh` must also remain exactly baseline; a replacement first seen at
`pre_fresh` is fail-stop.  If `post_release` contains the one admitted
replacement, `pre_fresh` must remain exactly that rotated snapshot.  This
removes the discrete-checkpoint gap without changing the one reset-owned
endpoint exception itself.

Source-first and clean fresh-installed focused tests each passed **55 tests**,
including unchanged, post-release replacement, late-first replacement, and
second-replacement cases.  Package-configured flake8, `py_compile`, and
`git diff --check` passed.  The isolated `robot_experiments` build/install is
at `/tmp/v6_probe_gid_checkpoint_clean.CnAEae`.  No ROS graph, Isaac, Nav2,
navigation, reset, evidence episode, engineering campaign, or formal
qualification was launched.  Attempt8 remains **PENDING**.

## Attempt8 cross-writer receive-order amendment

The immutable input is:

`/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_active_reset_live_attempt8_20260822T021512Z`

Attempt8 remains **ENGINEERING FAIL / STOP / NOT FORMAL**.  The probe reader
received the exact old Bool `false` terminal before it processed generation-2
HOLD and stopped with `old_terminal_before_hold:bool`.  The finalized MCAP
reader observed HOLD 0.000870 s before Bool and 0.000926 s before the exact
abort JSON, with no old route output at or after HOLD or pair completion.  The
two readers do not establish a cross-writer DDS total order.  Fresh navigation,
post-release topology persistence, and full postzero therefore remain untested
in that stopped episode.

The probe now buffers and validates either side first.  It establishes the
coordinator retirement fence only after all three observations exist: coherent
generation-2 HOLD, Bool `false`, and exact
`aborted/simulation_reset/reset_epoch=2` JSON for the old request.  The
dispatch-to-HOLD and dispatch-to-pair-completion intervals must each be at most
0.5 s, and the absolute HOLD/pair-completion receive skew must be at most
0.25 s.  The fence and quiet-window start are the later of HOLD and pair
completion.  Old outputs before that common fence are recorded as bounded
cross-writer in-flight observations; any old output at or after the fence is
fail-stop.  Wrong identity/epoch/reason, duplicates, a missing side, or any
deadline violation remain fail-stop.

This callback-order admission does not bypass the gate sequence or coverage
contracts.  A terminal-first episode must still receive the exact
`hold -> reset_complete -> released:*` sequence, complete zero
`/cmd_vel_sim`, collision, GT and odometry coverage, and pass all later fresh
and postzero checks.  Finalized-bag source/record order remains an external
requirement before an engineering PASS can be assigned.

Source-first and fresh-installed focused tests each passed **60 tests**.  They
cover the Attempt6 HOLD-first triplet, Attempt8 terminal-first order, both
missing-side timeouts, both receive-skew directions, dispatch-to-pair timing,
outputs before and after the common fence, wrong contracts, and duplicates.
Package-configured flake8, `py_compile`, `git diff --check`, installed import
path and entry-point help passed.  Fresh isolated `robot_experiments`
build/install passed at `/tmp/v6_probe_attempt9_build.hHQLhc`,
`/tmp/v6_probe_attempt9_install.wN2fwa`, and
`/tmp/v6_probe_attempt9_log.EOokYP`.

Verdict: **PASS (code/build/unit only)**.  No ROS graph, Isaac, Nav2,
navigation, reset, evidence episode, engineering campaign, or formal
qualification was launched by this amendment.  Attempt9 remains **PENDING**
and must use a fresh isolated runtime plus finalized bag.

## Attempt9 event-driven terminal-zero and actuator-backlog amendment

The immutable input is:

`/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_active_reset_live_attempt9_20260822T023957Z`

Attempt9 remains **ENGINEERING FAIL / STOP / NOT FORMAL**.  Its upstream
event-driven streams did stop correctly: `/cmd_vel_nav` emitted its final zero
0.0406 s before the route result, while `/cmd_vel_smoothed` and `/cmd_vel`
first emitted zero 0.0352 s and 0.0455 s after it.  Requiring all four topics
to publish a continuous one-second zero heartbeat was therefore the wrong
observation model.  However, this does not promote Attempt9: `/cmd_vel_sim`
relayed stale nonzero values through +0.4006 s, reached its first zero only at
+0.5732 s, and ground-truth yaw moved about 0.156 rad after terminal success.

The revised fail-stop contract is event driven but bounded:

- each command topic retains its rolling last finite sample for the fresh
  route only;
- every topic must have a finite zero in
  `[result - 0.25 s, result + 0.25 s]`;
- nonzero is allowed only before that topic's first settled zero and within
  the same 0.25 s deadline; any nonzero after settled zero is immediate STOP;
- after all four topics settle, the probe observes for at least 1.0 s from the
  latest first-zero time.  Event topics may be `ZERO_THEN_SILENT`; their last
  observed command must remain finite zero.  `/cmd_vel_sim` must supply at
  least two zero samples with a positive gap no larger than 0.25 s;
- ground truth must cover that full observation with callback gaps no larger
  than 0.25 s, XY span no larger than 0.02 m, and unwrapped yaw span no larger
  than 0.02 rad.  Collision remains false;
- a final `postzero_end` graph snapshot must exactly equal `pre_fresh`,
  including the already-admitted reset-owned GID state.

JSON now records per topic last nonzero, first settled zero, settle latency,
zero count, last observed sample, silence horizon, and
`ZERO_THEN_SILENT`/`ZERO_CONTINUING` classification, plus ground-truth XY/yaw
coverage and the final topology checkpoint.  Tests cover no zero, a late zero,
stale pre-result zero, nonzero after settled zero, Attempt9's +0.4006 s stale
actuator train, missing actuator zero cadence, topology change, XY drift, and
the 0.156 rad yaw drift.

Source-first probe plus package-contract tests passed **104 tests**; the same
**104 tests** passed against the fresh install.  ResetStopGate/reset-service
tests passed **36 tests**, ActivationGate tests passed **14 tests**, probe
flake8/`py_compile`/`git diff --check` passed, and the fresh build/install roots
are `/tmp/v6_attempt10_fix_build.vfIE1f`,
`/tmp/v6_attempt10_fix_install.dCXUjX`, and
`/tmp/v6_attempt10_fix_log.uE0yIV`.

Verdict: **PASS (code/build/unit only)**.  Attempt10 remains **PENDING**; it
requires a fresh isolated runtime and finalized bag.  If depth-one latest-value
delivery still misses the 0.25 s actuator deadline, escalate to an explicit
settled acknowledgment rather than widening this probe bound.

## Attempt10 pairwise XY-span amendment

The probe's shared position-coverage summary now defines `span_m` as the exact
maximum pairwise Euclidean separation between samples in the observation
window.  The previous first-sample anchor could report only 0.019 m for a
`+0.019 m -> -0.019 m` oscillation whose true peak-to-peak span is 0.038 m.
The shared helper feeds reset ground-truth, reset odometry, and fresh-route
postzero checks, so all three now use the same pairwise span semantics.  First
and last reset-landing errors retain their existing meanings.

Focused tests cover static samples, monotonic straight motion, the exact
0.038 m pairwise result, and state-machine STOP for the oscillation.  Source
probe plus package-contract tests passed **108 tests** (focused probe: **70
tests**); the same **108 tests** passed against the fresh install.  Changed-file
flake8, `py_compile`, `git diff --check`, installed import-path assertion,
installed entry-point help, and a fresh isolated `robot_experiments` build
passed.  Build/install/log roots are
`/tmp/v6_probe_pairwise_build.j0Hfvn`,
`/tmp/v6_probe_pairwise_install.mO2nop`, and
`/tmp/v6_probe_pairwise_log.MWsXJn`.

Verdict: **PASS (code/build/unit only)**.  No ROS graph, Isaac, Nav2,
navigation, reset, evidence episode, engineering campaign, or formal
qualification was launched.  Attempt10 remains **PENDING** and requires a
fresh isolated runtime plus finalized bag.
