# V6 active-reset exactly-once probe

## Scope and status

This amendment adds a single-process `active_reset_probe` runner for the next
fresh active-reset attempt.  It replaces the split Attempt4 harness timing
with one monotonic state machine and an atomically refreshed JSON receipt.

This is **PASS (code/build/unit only)**.  No ROS graph, Isaac, Nav2,
navigation, reset, bag, visual review, engineering campaign, or formal
qualification was run by this task.  Attempt5 remains **PENDING**.

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

`WAIT_ENDPOINTS -> PUBLISH_OLD_ONCE -> WAIT_ACTIVE_READY -> CALL_RESET_ONCE -> OBSERVE_HOLD_ABORT -> WAIT_RELEASE -> QUIET -> PUBLISH_FRESH_ONCE -> WAIT_SUCCESS -> POSTZERO`

It fails closed to `STOP` and exits nonzero on the first contract violation.
Wall deadlines use `time.monotonic()`; ROS time is used only for published
message stamps.  The route-goal publisher is reliable/volatile and remains
alive for the whole process.

Key checks are (including the review-blocker amendment at Module3 start
`8825e606245df83d8bd755e84dff0730c9d11aa1`):

- use endpoint-info APIs, not scalar counts, to require exactly one
  `/cmd_vel` publisher `/collision_monitor`, exactly one `/cmd_vel_sim`
  publisher `/isaac_navigation_sim` (ResetStopGate authority), exactly one
  probe publisher, and the exact configured route-goal subscribers. Attempt5
  defaults are `/bio_nav_route_coordinator` plus `/rosbag2_recorder`;
- record node names, namespaces, GIDs and counts, then require the exact graph
  to remain unchanged at `prepublish`, `pre_reset`, `post_release`, and
  `pre_fresh` before any associated side effect;
- wait for the Trigger service, all observed topic publishers, one GT and one
  estimated-odometry sample, and retained generation-1 release;
- publish old G2 `(0.8, 4.8, -2.792526803)` exactly once;
- within six seconds observe one new canonical request, matching progress,
  lookahead and goal-update, at least five nonzero `/cmd_vel_sim` messages,
  at least 0.10 m GT displacement, no collision, and no terminal;
- call `/simulation/reset` exactly once within 0.5 s of `active_ready`;
- strictly validate seed 8601, generation 2, pose `long_route_start_g1`, and
  odometry `realistic` from the reset receipt;
- observe the same-topic, strictly increasing generation-2
  `hold -> reset_complete -> released:*` receive sequence. HOLD-to-release
  `/cmd_vel_sim` and collision, plus stable-to-release GT and EKF odometry,
  each require at least two samples, both edges within 0.25 s and no gap over
  0.25 s. Commands must all be zero, collisions all false, and GT/odom spans
  no greater than 0.02 m. GT landing is checked against map `(0.45,-5.35)`;
  realistic EKF odometry landing is separately checked against odom `(0,0)`;
- require one old Bool `false` plus exact JSON
  `aborted/simulation_reset/reset_epoch=2` for the old request, one reset
  event, and an exact reset receipt;
- require one second without old route outputs, then publish fresh goal
  `(0.685, -3.975535, 1.570796327)` exactly once;
- require a strictly newer request ID, the exact configured canonical edge
  list (default `[51,52]`, with no support-equivalent escape), Bool `true`,
  exact JSON `succeeded/final_goal_distance_confirmed/reset_epoch=2`, GT error
  no greater than 0.30 m, and no collision within 30 seconds;
- for the complete final 1.0 s require at least two fresh samples on each of
  the four command-chain topics, both edges within 0.25 s, no gap over 0.25 s,
  and every sample zero.

The output JSON is rewritten via `fsync` plus `os.replace` at a bounded 20 Hz
and forced at every state transition.  Trigger dispatch happens before the
state-transition write, so NAS latency is outside the 0.5 s boundary. It
records phase/verdict/STOP reason, counts, exact request
IDs and routes, reset receipt and gate sequence, monotonic boundary times,
`active_ready`-to-reset delay, GT displacement/drift/error, collision, and
post-stop command evidence. Endpoint, publish, service dispatch, done-callback,
JSON-write, spin and teardown exceptions fail to an atomically retried terminal
STOP document; publish-attempt counters are advanced before the side effect so
an exception cannot cause a retry.

Callback monotonic times do **not** prove cross-topic source order. Even when
all in-process contracts pass, the JSON verdict is therefore
`PROVISIONAL_PASS_REQUIRES_BAG_ORDER`; it explicitly sets
`engineering_pass=false`. Only an independent finalized-bag ordering analysis
may promote the fresh Attempt5 episode to engineering PASS.

Example operator invocation after a fresh archive build:

```bash
ros2 run robot_experiments active_reset_probe \
  --output "$evidence_root/probe/active_reset_probe.json"
```

Do not reuse Attempt4 runtime state.  The next reviewer must start a fresh
isolated episode, bag the contracted topics independently, and compare the
probe JSON with the finalized bag before making an engineering-runtime claim.

## Deterministic validation

Pure state-machine coverage includes exact endpoint identities and graph-change
STOP, endpoint/subscriber wait, startup
retained-event exclusion, exactly-once publication/service calls, active
timeout, terminal-before-reset, reset-call delay, receipt mismatch, gate order,
duplicates/races, service failure, HOLD/GT/odom/collision coverage and leakage,
reset teleport versus post-landing drift/error, old-output silence, strict
request/epoch/reason/edge identity, fresh success/failure, the four-chain full
postzero contract, and atomic entrypoint STOP output.

The review-blocker amendment validated source-first **58 passed** (20 probe
tests plus 38 retained package-contract tests), package-configured flake8,
`py_compile`, and `git diff --check`. A fresh ordinary isolated build/install
passed at `/tmp/v6_active_reset_probe_fix_build.D38WEx`, install
`/tmp/v6_active_reset_probe_fix_install.51b4zR`, and log
`/tmp/v6_active_reset_probe_fix_log.ozbzdC`. The same 58 tests imported the
installed module (path asserted under that install), and both installed
`ros2 run ... --help` and the direct console entry point passed. No ROS graph,
Isaac, Nav2, navigation, reset or evidence episode was launched.
