# V7.3 Phase 1C G1-to-G2 pilot runner handoff (2026-08-24)

## Result and boundary

- The canonical V6-GRID runner now has one opt-in visual-shadow profile enum:
  `rgbd|stereo_imu`, default `rgbd`. The existing enable boolean remains
  default false. Disabled and enabled RGB-D paths retain their prior launch
  selections; `stereo_imu` derives the Isaac `stereo_vio` camera, calibrated
  `/imu/vio`, and the existing stereo+IMU cuVSLAM shadow launch.
- Only the stereo+IMU profile adds the physical-reset boundary: after a
  successful `/simulation/reset` receipt and before Grid/Nav2/goal waits, the
  dispatcher captures the last visual odom/status stamps, calls the official
  `/visual_slam/reset` `isaac_ros_visual_slam_interfaces/srv/Reset`, requires
  `success=true`, and waits on wall monotonic time for finite odom and healthy
  status with stamps strictly newer than both captured floors.
- Missing/timed-out/rejected visual reset, equal/older stamps, nonfinite data,
  `vo_state=2`, or invalid status timings stop the episode with zero goals.
  The only new barrier event names are
  `visual_shadow_pre_reset_barrier`, `visual_shadow_reset_response`, and
  `visual_shadow_post_reset_ready`.
- The manifest and formal/qualification contract remain five legs. An explicit
  engineering `--pilot --dispatch-pilot` may set `--max-legs 1..5`; one leg
  ends successfully at G2 and cannot publish G3. The session default is
  `R5_MAX_LEGS=5`; this Phase 1C pilot uses `1`.
- The stereo recorder branch adds left/right image and CameraInfo,
  `/imu/vio_raw`, `/imu/vio`, `/visual/odom_shadow`, and `/visual/status` while
  retaining GT, wheel/EKF odom, commands, collision, plans, and raw costmaps.
  Camera/IMU overrides are BEST_EFFORT/VOLATILE and visual outputs are
  RELIABLE/VOLATILE. Left depth is intentionally not recorded.
- This amendment is code/static/build only. It did not start Isaac, ROS, Nav2,
  a route goal, a bag, or a qualification campaign. It does not change
  canonical `/odom`, cuVSLAM TF/config, Integration, Module2, or the estimated
  state evaluator.

## Validation

- Source-first/no-cache focused pytest:
  - `robot_experiments/test/test_v6_formal.py`: **32 passed**;
  - `robot_bringup/test/test_runtime_scripts.py`: **53 passed**;
  - `robot_odometry/test/test_visual_odometry_contract.py`: **10 passed**.
- `bash -n` passed for the two changed orchestration scripts and the existing
  formal wrapper. Python compilation passed for `ros_stack.launch.py` and
  `v6_formal.py`.
- Clean isolated root `/tmp/v73_phase1c_pilot_isolated.nkP2XY` built
  `robot_experiments`, `robot_grid_localization`, `robot_route_planner`, and
  `rf2o_laser_odometry`; the installed validation-only CLI retained the exact
  five-leg default. `robot_bringup` could not be built alone because ten
  existing same-worktree runtime packages were not included in that isolated
  install. No unrelated dependency closure was built or repaired.

## First live command

After producing a fresh combined snapshot containing this commit, confirm an
unused domain no higher than 232 and run exactly one engineering leg:

```bash
cd /home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3
RUN_DIR="/mnt/nas_home/Bio_Nav_Data/experiments/runs/v73_phase1c_g1_g2_$(date -u +%Y%m%dT%H%M%SZ)"
SNAPSHOT_ROOT="/absolute/path/to/fresh_v73_phase1c_combined_snapshot"
ISAAC_ASSET_ROOT=/home/lyb/isaacsim_assets/Assets/Isaac/6.0 \
V6_VISUAL_ODOMETRY_SHADOW_ENABLED=true \
V6_VISUAL_ODOMETRY_SHADOW_PROFILE=stereo_imu \
R5_DOMAIN_ID=230 R5_EPISODE_INDICES=0 R5_EPISODE_SEEDS=7201 \
R5_MAX_LEGS=1 \
  ./scripts/run_v6_kujiale_low_obstacles.sh session "${RUN_DIR}" "${SNAPSHOT_ROOT}"
```

## Remaining live risks

- The combined snapshot/full-overlay argument chain and actual recorder
  subscriptions remain live-unverified.
- The physical-reset to visual-reset ordering, official reset response, strict
  post-stamp barrier, Grid/Nav2 release, and G1-to-G2 terminal behavior are
  code-tested but have not been observed together.
- Phase 1C route accuracy, pivot translation, collision outcome, RTF/GPU load,
  transport loss, and covariance quality remain unverified. This commit is not
  VIO promotion or formal qualification evidence.

## Seed-7201 live pilot result

- Evidence root:
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v73_phase1c_g1_g2_20260824T135835Z`.
  The one-leg engineering pilot published G2 only, never published G3, ended
  with action/runner state `SUCCEEDED`, and recorded zero collisions. Verdict:
  **ENGINEERING ROUTE COMPLETION PASS / VIO PROMOTION FAIL**. This is not
  formal qualification; do not cut over VIO or enter Phase 1D from this run.
- After the seed-7201 physical reset, the official visual reset returned
  `success=true` in `14.245 ms`. The odom/status floor was `21.25 s`; both
  first accepted strictly-new stamps were `21.30 s`, ready in `15.655 ms`
  with `vo_state=1`. Through completion, VIO was 20 Hz, max native gap was
  `50.000998 ms`, there were zero gaps `>=150 ms`, and all 717 status
  samples were state 1.
- The bag-time comparison window was
  `1787579991033417850 -> 1787580047482525307 ns`. Although the action
  succeeded, evaluator-only GT ended `0.331244 m` from G2, outside 0.25 m.
  Start-aligned SE(2) VIO endpoint/max XY error was
  `0.393677/0.445978 m`, versus active EKF `0.214637/0.214637 m`; VIO
  endpoint yaw error was `0.009563 rad`. Frames were GT
  `map -> ground_truth_base_link`, wheel/EKF `odom -> base_link`, and VIO
  `visual_odom_shadow -> base_link`. Route-window RTF `0.583269` remains a
  performance warning.
- Terminal settling was bounded but not instantaneous: one small nonzero
  `/cmd_vel_sim` appeared completion +`161.082 ms`; the first explicit zero
  was +`313.349 ms`; the following four commands and final latched command
  were zero. GT moved `0.023999 m` after the first zero.
- NAS indices: `conclusion.md`, `review/phase1c_g1_g2_metrics.json`, and
  `figures/phase1c_g1_g2_trajectories.png`. The 3.09 GB MCAP contains 151,133
  readable messages, but the runner force-killed rosbag after its graceful
  timeout, leaving no metadata/index. This is an evidence warning, not a
  product failure. Owned resources were cleaned, domain 230 was empty, and
  preserved domain-141 PID 3600069 remained alive.
- Next: keep the shadow isolated and perform a bounded accuracy RCA at the
  calibration/extrinsic/estimator-scale first-error layer before considering
  any single-variable rerun.

## Stereo-only STOP RCA and terminal-zero amendment (2026-08-25)

- The initial stereo+IMU seed-7201 route remains an engineering route success
  but a VIO promotion failure. Both later stereo-only routes stopped on true
  collision: `v73_phase1c_stereo_only_20260824T150824Z` at the dining-room
  chair and `v73_phase1c_stereo_only_jointdiag_20260824T164920Z` at the sofa.
  Neither run promotes VIO or authorizes Phase 1D.
- The corrected joint/MCAP probe supersedes the first stereo-only observer's
  apparent collapsed-wheel diagnosis. In the dispatch-to-collision window,
  joint-equivalent, wheel-odom, and GT paths were `8.941021`, `8.931083`, and
  `8.813655 m`; the joint and wheel chain therefore passed this targeted
  route check. The remaining product blocker was terminal stop behavior.
- The targeted MCAP recorded no downstream `/cmd_vel_sim` zero after
  collision: its last nonzero was collision +`299.246 ms`, the final command
  was nonzero, and GT drifted `0.056858 m` after collision. `/cmd_vel_sim`
  retained its expected sole publisher, `/isaac_navigation_sim`.
- This amendment makes the episode node publish only upstream
  `/cmd_vel_nav` reliable/volatile zeros at 20 Hz after an active-goal
  failure. It continues spinning until cancel completes or the action is
  terminal, observes a terminal-start-newer downstream `/cmd_vel_sim` zero,
  and then sees a short window without a newer nonzero. The wait is bounded;
  timeout remains STOP, records `terminal_zero_timeout`, and preserves the
  collision/root reason. It does not publish `/cmd_vel` or `/cmd_vel_sim`,
  and pre-goal STOP and success behavior do not acquire this wait.
- Validation is code/static/build only: source-first/no-cache focused tests
  passed `38`; an isolated `/tmp/v73_terminal_zero.Htj4jW` build of
  `robot_experiments` passed, and its installed artifact passed the same 38
  focused tests. Full package `colcon test` was not usable because unrelated
  collection imported an external stale `bio_nav_interfaces` install without
  `CanonicalRoute`; no overlay or Integration repair was made.
- Authoritative NAS index:
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v73_phase1c_stereo_only_jointdiag_20260824T164920Z/mcap_targeted_conclusion.md`
  and `review/mcap_targeted_metrics.json` below that run root. No MCAP bag
  payload was read by this implementation amendment.
- Verdict: **TERMINAL-ZERO CODE/FOCUSED-TEST PASS; LIVE UNVERIFIED**. Next is
  one bounded human-triggered cancel live check proving downstream
  `/cmd_vel_sim` zero and post-zero stability before teardown; do not claim
  collision closure, VIO promotion, or Phase 1D yet.

## Terminal-zero live engineering closure (2026-08-25)

- The first one-shot live on `9fac3b855620a18e9e81eda0009d59f9ae4fb1d6`
  **FAILED / INCOMPLETE**: the production node did not register `Twist`, so
  terminal settling raised `KeyError: 'Twist'` before publishing its zero
  burst. The focused-test fixture had injected `Twist` directly and therefore
  missed the real constructor path. NAS index:
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v73_phase1c_terminal_zero_cancel_20260824T174858Z/conclusion.md`.
- `8ac12dbcb363e48589a628ecb5aed47e050d7c59` registered the production type
  and added constructor-backed coverage; focused tests passed `39`, and the
  isolated build passed.
- The fresh one-shot live
  `v73_phase1c_terminal_zero_cancel_20260824T181235Z` sent exactly one
  `/navigate_to_pose` cancel for UUID
  `5a6f52481a0f470280f65eedf8cd9521`, with `all_goals=false`; the response
  returned code `0`, and collision remained false. Stable downstream
  `/cmd_vel_sim` zero arrived terminal +`450.854 ms`; the following quiet
  interval was `307.636 ms`. `terminal_zero_confirmed` preceded
  `episode_result` by `1.765 ms` and Nav2 teardown by `182.821 ms`.
  `/cmd_vel_nav`, `/cmd_vel_smoothed`, `/cmd_vel`, and `/cmd_vel_sim` all ended
  at zero. Post-zero GT displacement was `2.278 mm` over `6.717 s`, and
  `/cmd_vel_sim` retained its sole owner `/isaac_navigation_sim` when present.
- NAS index:
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v73_phase1c_terminal_zero_cancel_20260824T181235Z/conclusion.md`.
- Verdict: **TERMINAL-ZERO ENGINEERING PASS ONLY**. This is not formal
  qualification, VIO promotion, collision closure, or Phase 1D authorization.
  Return to the remaining Phase 1C VIO accuracy decision.

## Final mapping_start paired diagnostic and Phase 1C decision (2026-08-25)

- Authoritative NAS root:
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v73_phase1c_paired_motion_mapping_start_20260824T202321Z`;
  primary indices are `conclusion.md` and `paired_summary.json`.
- The sequential mode-1 (stereo+IMU) and mode-0 (stereo-only) arms both ended
  `profile_complete`, collision-free, and with a complete final-zero phase.
  Realized GT motion was comparable: path `1.383402/1.384178 m`, ratio
  `1.000561`.
- Full-profile visual endpoint/max/p95 XY errors were
  `0.036598/0.090372/0.071388 m` for mode 1 and
  `0.117591/0.129877/0.117591 m` for mode 0. The frozen full-profile score was
  therefore worse for mode 0 by `0.046202 m` (`1.647x`), but did not cross the
  `0.05 m` significance floor. Mode 1 has a net numerical benefit, while the
  pair identifies no credible single-variable repair.
- Both arms retained 20 Hz visual odom/status, 321 state-1 samples, and a
  `50.001 ms` maximum native gap. Profile RTF remained a warning
  (`0.593420/0.590081` for mode 1/mode 0), together with the shared scene
  material/texture and mesh-collision fallback diagnostics.
- This paired open-space fixed-motion result is an **engineering diagnostic**,
  not route evidence and not formal qualification. The earlier route/collision
  STOP evidence remains valid and is not superseded by this pair.
- Decision: **PHASE 1C VIO DEFERRED / NOT PROMOTED**. Phase 1D and Phase 2 are
  not authorized. Keep VIO isolated from canonical `/odom` and TF; next move
  to the canonical independent local-odom alternative lane rather than another
  VIO parameter iteration.
