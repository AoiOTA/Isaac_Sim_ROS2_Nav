# V6 L0--L3 localization causal runner handoff — 2026-08-21

## Scope and status

- Worktree: `/home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3`
- Parent HEAD: `9a9725729972d58d40c1614038fc720cbd8dad1f`
- Commit: this handoff's single `feat: add V6 localization causal runner` commit.
- Result: **PASS (code-level contract only)**.
- Campaign state: **ENGINEERING_CAUSAL_NOT_RUN**. No Isaac, ROS navigation,
  60-row campaign, causal result, or formal qualification was run.

## Implemented contract

- `v6_localization_causal.yaml` expands to the frozen core 60 rows:
  `L0--L3 × 5 seeds × S0/S3/W0`. S1 and S2 remain two single-seed
  engineering preflights and do not count toward the core 60.
- Every arm fixes estimated odometry, `M0`, GVG, no direct RGB-D Costmap,
  `use_rviz=false`, Isaac structure TF, G2, and no Module2 planning influence.
  Only the localization intervention changes:
  - L0: no Integration; conventional coarse startup seed.
  - L1: estimated shadow; Integration shadow writes zero initial poses.
  - L2: estimated shadow; startup writes exactly one cognitive seed.
  - L3: estimated autonomy; one startup seed and only an explicit manual
    rescue after S3 lost detection. Automatic rescue remains disabled.
- S0 binds seeds `8701..8705` and the fixed `(+1.0 m, -0.5 m, +20 deg)`
  coarse offset for L0/L1. S3 binds `8731..8735` and a physical G2-to-G5
  teleport without resetting odometry, EKF, or AMCL. W0 binds `8741..8745`
  and allows no initial-pose write or manual rescue.
- The dispatcher topic list contains no Ground Truth. The passive evaluator
  alone consumes `/ground_truth/odom` and computes absolute map-frame position
  and yaw error without first-frame or SE(2) alignment. It also evaluates 2 s
  convergence hold, 1 s lost hold, recovery, wrong reseed, P95 error, pause
  commands, collision, and publisher ownership.
- Aggregate criteria encode L2 `>=4/5` convergence within 5 s and median
  improvement `>=30%` versus L0; L3 `>=4/5` recovery within 8 s and improvement
  `>=30%` or `>=2 s` versus L2; wrong reseed, pause motion, and collision must
  remain zero.
- `run` deliberately returns `NOT_RUN` because no live adapter is installed.
  `manifest`, `plan`, and `evaluate` are available through the installed entry
  point and `scripts/run_v6_localization_causal.sh`.

## True-kidnap service

- `KidnapServiceBridge` exposes `/simulation/kidnap` as `std_srvs/Trigger`.
  It is rejected unless odometry mode is `realistic`, `kidnap_armed=true`, the
  named spawn pose exists, and fresh `/cmd_vel` has remained zero for the fixed
  hold interval.
- The runner plan cancels the goal, verifies no active goal, holds zero command,
  pauses, arms `long_route_start_g5`, calls Trigger once with no retry on an
  unknown response, unarms, and resumes. The service itself provides the
  independent fresh-zero guard; route-idle verification remains explicitly
  runner-owned.
- The controller zeros base, joint velocities, and joint targets before and
  after `set_world_pose`. It is one-shot per arming cycle and creates no ROS
  publisher, reset event, initial pose, odometry reset, or epoch update.

## Validation

- Fixed `main` refs matched the project boundary before writing.
- `python3 -m py_compile` for both runner/evaluator modules, kidnap service, and
  `navigation_sim.py`: PASS.
- Focused new plus existing V6 formal/causal/reset tests:
  `66 passed in 0.23s`.
- `bash -n scripts/run_v6_localization_causal.sh`: PASS.
- `git diff --check`: PASS.
- Isolated `robot_experiments` build: PASS at
  `/tmp/v6_localization_causal_build.61UoeJ`.
- Installed entry checks: manifest 60 rows; plan 60 rows; `run` returned the
  expected nonzero `NOT_RUN` state.

## Remaining work / risks

- The Trigger bridge has pure/mock coverage only. Its ROS callback, Kit
  main-thread behavior, articulation teleport, and parameter transitions have
  not been exercised live.
- No active-goal status is available inside the Isaac bridge; a real adapter
  must enforce the frozen cancel/no-active-goal step before arming.
- The 60 rows and two preflights remain unrun. Do not interpret synthetic unit
  fixtures or evaluator `PASS_CRITERIA` capability as a causal result.
