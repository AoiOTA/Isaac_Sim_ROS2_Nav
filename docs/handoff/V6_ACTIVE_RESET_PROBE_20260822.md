# V6 active-reset exactly-once probe

## Scope and status

This amendment adds a single-process `active_reset_probe` runner for the next
fresh active-reset attempt.  It replaces the split Attempt4 harness timing
with one monotonic state machine and an atomically refreshed JSON receipt.

This is **PASS (code/build/unit only)**.  Attempt5 is retained as
**ENGINEERING FAIL / STOP / NOT FORMAL**; this amendment does not rerun it.
No ROS graph, Isaac, Nav2, navigation, reset, bag, visual review, engineering
campaign, or formal qualification was run by this task.  Attempt6 remains
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
  0.5 s or the probe STOPs.  From received HOLD until fresh publication, any
  canonical route, progress, lookahead, goal-update/Navigate intent, or other
  old route output is recorded with receive time/type/request identity and
  immediately STOPs; this does not wait for the old terminal pair;
- require one second without old route outputs, then publish fresh goal
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

Callback monotonic times do **not** prove cross-topic source order. In-process
completion is `PROVISIONAL_PASS_REQUIRES_BAG_ORDER`; after successful teardown
and final persistence the verdict is `PASS_REQUIRES_BAG`. Both explicitly keep
`engineering_pass=false`. Only an independent finalized-bag ordering analysis
may promote the fresh Attempt5 episode to engineering PASS.

Example operator invocation after a fresh archive build:

```bash
ros2 run robot_experiments active_reset_probe \
  --output "$evidence_root/probe/active_reset_probe.json"
```

Do not reuse Attempt4 or Attempt5 runtime state.  The next reviewer must start
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
