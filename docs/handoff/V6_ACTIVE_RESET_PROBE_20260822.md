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

Key checks are:

- wait for the configured route-goal subscriber count, Trigger service, all
  observed topic publishers, one GT sample, and retained generation-1 release;
- publish old G2 `(0.8, 4.8, -2.792526803)` exactly once;
- within six seconds observe one new canonical request, matching progress,
  lookahead and goal-update, at least five nonzero `/cmd_vel_sim` messages,
  at least 0.10 m GT displacement, no collision, and no terminal;
- call `/simulation/reset` exactly once within 0.5 s of `active_ready`;
- strictly validate seed 8601, generation 2, pose `long_route_start_g1`, and
  odometry `realistic` from the reset receipt;
- observe generation-2 `hold -> reset_complete -> released:*`, zero nonzero
  `/cmd_vel_sim` during HOLD, one old Bool `false` plus JSON
  `aborted/simulation_reset` terminal for the old request, one reset event,
  and post-landing GT drift no greater than 0.02 m;
- require one second without old route outputs, then publish fresh goal
  `(0.685, -3.975535, 1.570796327)` exactly once;
- require the new request, default canonical `[51, 52]` (or a CLI-explicit
  recorded support-equivalent allowance), Bool `true`, JSON `succeeded`, GT
  error no greater than 0.30 m, and no collision within 30 seconds;
- for the final second require samples on all four command-chain topics, zero
  nonzero samples, and a trailing zero on each.

The output JSON is rewritten via `fsync` plus `os.replace` at a bounded 20 Hz
and forced at every state transition.  Trigger dispatch happens before the
state-transition write, so NAS latency is outside the 0.5 s boundary. It
records phase/verdict/STOP reason, counts, exact request
IDs and routes, reset receipt and gate sequence, monotonic boundary times,
`active_ready`-to-reset delay, GT displacement/drift/error, collision, and
post-stop command evidence.

Example operator invocation after a fresh archive build:

```bash
ros2 run robot_experiments active_reset_probe \
  --output "$evidence_root/probe/active_reset_probe.json" \
  --expected-route-subscribers 2
```

Do not reuse Attempt4 runtime state.  The next reviewer must start a fresh
isolated episode, bag the contracted topics independently, and compare the
probe JSON with the finalized bag before making an engineering-runtime claim.

## Deterministic validation

Pure state-machine coverage includes endpoint/subscriber wait, startup
retained-event exclusion, exactly-once publication/service calls, active
timeout, terminal-before-reset, reset-call delay, receipt mismatch, gate order,
HOLD nonzero output, reset teleport versus post-landing drift, old-output
silence, strict/allowed fresh route identity, fresh success/failure, and the
four-chain postzero contract.

Validation was source-first focused **52 passed** (14 probe-state tests plus
38 retained package-contract tests). Changed-file flake8, `py_compile`, and
`git diff --check` passed. A fresh ordinary isolated install build passed at
`/tmp/v6_active_reset_probe_commit_build.vDaiVx`, install
`/tmp/v6_active_reset_probe_commit_install.lmarja`, log
`/tmp/v6_active_reset_probe_commit_log.FD2IDl`; the installed `ros2 run ... --help`
and console entry point passed. An earlier isolated `--symlink-install`
attempt failed before packaging because the existing `setup.py` external
resource paths escape a `/tmp` build base; the non-symlink isolated build is
the cited result. The final commit is recorded in the experiment ledger.
