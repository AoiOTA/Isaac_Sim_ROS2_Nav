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
- Startup has no Nav2 recovery owner yet. After the startup transaction and all
  queued reset futures finish, `navigation_sim` releases the same-process
  generation directly. Release publishes one final zero and never replays a
  cached pre-reset command; motion requires a subsequent fresh message.
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
