# Experiment ledger

## 2026-08-20 — V6 Stage 1 Module3 estimated substrate

- Hypothesis: a minimal Wheel+IMU EKF and AMCL chain can be made buildable now, while the unavailable LiDAR odometry backend remains an explicit fail-fast option and Module2 priors fail open on stale/unhealthy refresh.
- Branch/worktree/commit: `cognitive-navigation`; `/home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3`; implementation `f4bc4eda5ffe059daef52cdd481bf45a41635b56`.
- Main files: `robot_bringup/launch/ros_stack.launch.py`, `robot_mapping/launch/localization.launch.py`, scene AMCL YAML, EKF A/B YAML/launch, RF2O YAML/launch, `robot_route_planner/ros_node.py`, and focused tests.
- Configuration: `odometry_mode:=estimated`, default `ekf_profile:=wheel_imu`, default `lidar_odometry_backend:=off`, `localization_profile:=kujiale|rivermark`, prior TTL default 2.0 s.
- Commands: pure Jazzy selective `colcon build`; selective `colcon test`; `colcon test-result --verbose`; `bash -n scripts/run_ros.sh`; focused bringup/TF pytest.
- Core result: independent build PASS for 5 packages; `95 tests, 0 failures, 1 skipped`; focused bringup/TF `206 passed`.
- Evidence: local `ros2_ws/build/*/test_results` and `ros2_ws/log`; generated build artifacts are not committed.
- Verdict: **PASS (implementation/build/unit only)**.
- Blocker: full Module3 build requires `bio_nav_interfaces` from the allowed Integration worktree install, which was absent. RF2O is absent from `/opt/ros/jazzy`.
- Unrun: ROS runtime, TF/topic observation, Isaac runtime, AMCL convergence, Nav2 navigation, odometry metrics, scene calibration, and formal qualification.
- Next: rebuild with the allowed Integration overlay when ready; then no-Isaac launch smoke, followed by actual RF2O installation and runtime calibration.

## 2026-08-20 — V6 Stage 1 prior lifecycle and legacy Ideal wrappers

- Hypothesis: consuming the pending prior window on timeout and binding each
  response to the live request/graph/refresh/model identity prevents late
  learned costs from re-entering geometry-only routing; explicit Ideal launch
  arguments prevent the new AMCL default from changing legacy wrappers.
- Branch/worktree/commit: `cognitive-navigation`;
  `/home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3`;
  commit recorded by the task handoff after submission.
- Commands: focused source pytest; pure Jazzy plus allowed Integration
  `local_setup.bash` `colcon build --packages-up-to robot_route_planner
  robot_bringup`; focused colcon route/bringup tests; new launch-test flake8;
  `git diff --check`.
- Core result: `72 passed, 1 skipped` source tests; 13-package build PASS;
  focused colcon tests `27 passed` route plus `21 passed` bringup.
- Verdict: **PASS (implementation/build/unit and offline launch expansion)**.
- Unrun: ROS/Isaac/Nav2 runtime, 8-second no-Isaac launch smoke, and formal
  qualification.
- Warning: the unrelated full-suite frozen-JSON absolute-path assertion is
  checkout-sensitive and was deliberately left untouched.
- Next: reviewer may run the bounded no-Isaac launch smoke from the built
  overlay, then proceed to runtime localization/navigation validation.

## 2026-08-20 — V6 C2/C3/D3 Module3 cognitive consumers

- Hypothesis: typed Module2 V6 candidates can influence Costmap, MPPI and the
  one Route Server without acquiring physical legality, TF, control or safety
  ownership.
- Branch/worktree: `cognitive-navigation` at the permitted Module3 worktree;
  implementation commit recorded after submission.
- Inputs: Integration interfaces `e16929d2369c0d5fce7cbaa5e07dbc0465a901f0`;
  Module2 producer `15e4c2bb9257345b43404864e751874f03bfcb82`.
- Commands: temporary exact-source interface archive/build; pure Jazzy plus
  temporary-interface `colcon build --packages-up-to` for fusion, route,
  navigation and bringup; `bio_nav_fusion` colcon tests; focused and full Route
  consumer/profile pytest; Python compilation; `git diff --check`.
- Core result: temporary interface build PASS; 14-package consumer build PASS;
  C++/plugin-loader `15 tests, 0 failures`; Python `77 passed, 1 skipped` due
  unavailable optional `pxr`.
- Verdict: **PASS (implementation/build/unit and fixture validation only)**.
- Unrun: Isaac/ROS/Nav2 runtime, live Costmap/MPPI/Route Server behavior,
  navigation closure, metrics, visual evidence and formal qualification.
- Next: fresh reviewer should first verify live off/shadow byte identity and
  status sequencing, then active Costmap/critic ranking and atomic graph
  switch/GVG fallback before any end-to-end claim.

## 2026-08-20 — V6 A3 fixed-revision RF2O shadow runtime

- Hypothesis: vendoring a fixed official RF2O revision with a true single-node
  Jazzy wrapper can provide usable topic-only LiDAR odometry without changing
  TF ownership or prematurely enabling three-source fusion.
- Branch/worktree/commit: `cognitive-navigation`;
  `/home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3`;
  commit containing `docs/handoff/V6_A3_RF2O_VENDOR.md`.
- Upstream/configuration: MAPIRlab RF2O commit `b38c68e46387b98845ecbfeb6660292f967a00d3`;
  `/lidar/odom`; `odom -> base_link`; `publish_tf:=false`; conservative
  parameterized covariance; `lidar_odometry_validated:=false` by default.
- Commands: fixed-SHA temporary clone verification; focused `rosdep
  --ignore-src`; Jazzy + allowed Integration selective colcon build/test;
  direct validation-gate negative launch; isolated-domain static-TF plus
  deterministic LaserScan/clock smoke; flake8/xmllint/diff check.
- Core result: build PASS; tests `18 + 14 + 204` passed; gate negative test
  exited 1 as intended; synthetic smoke produced 44 finite, nonzero,
  stamp-monotonic messages with positive covariance and correct frames; exactly
  one RF2O node, no hidden listener/algorithm node, and zero `/tf` publishers.
- Evidence/reproducer: `docs/handoff/V6_A3_RF2O_VENDOR.md` and
  `ros2_ws/src/robot_odometry/test/rf2o_synthetic_smoke.py`; generated build
  artifacts remain under `ros2_ws/build`, `install`, and `log` and are not
  committed.
- Verdict: **PASS (vendor/build/unit/synthetic ROS smoke)**.
- Deferred: bag/Isaac ATE/RPE, covariance calibration, real-sensor robustness,
  and formal qualification. No usable bag exists in this allowed worktree, so
  three-source fusion remains fail-closed shadow.

## 2026-08-20 — V6 A3 reviewer gate and motion amendment

- Goal: bind LiDAR validation to the final loaded EKF YAML rather than the
  profile label, and prevent constant quaternion w from satisfying synthetic
  motion evidence.
- Changes: shared fail-closed EKF input classifier used by direct EKF and core
  bringup; relative x/y/yaw plus planar-twist `motion_detected`; explicit
  stationary and moving smoke modes.
- Build/test: pure Jazzy + allowed Integration build PASS (14 packages);
  `robot_localization_config` 18 total/0 failed, `robot_odometry` 19/19, and
  `robot_bringup` 204/204.
- Actual launch matrix: profile `wheel_imu` plus custom three-source YAML and
  validation false rejected RC=1; the same YAML with true started; profile
  `wheel_imu_lidar` plus actual canonical wheel-only YAML and false started.
- Synthetic smoke: stationary repeated scans PASS with
  `motion_detected=false`, one valid odometry sample before RF2O rejected the
  degenerate identical pairs; translated room scans PASS with 68 valid samples
  and `motion_detected=true`. Both retained finite data, positive covariance,
  correct frames, monotonic stamps and zero dynamic TF publication.
- Verdict: **PASS (reviewer amendment build/unit/launch/synthetic smoke)**.
- Deferred unchanged: Isaac/bag ATE/RPE, covariance calibration, and formal
  qualification.

## 2026-08-20 — V6 C4 Kujiale frozen low-obstacle layout

- Goal: freeze a separate six-obstacle Kujiale layout without editing the USD,
  map, existing draft layout, or existing static/dynamic campaign defaults.
- Branch/worktree/start: `cognitive-navigation`;
  `/home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3`;
  `f81c690f2064a5711ecfe8ea68edcabc3e41915e`.
- Layout: `kujiale_v6_low_obstacles_frozen_r1_20260820`; five original centers
  retained; east bar x minimally adjusted to 1.300000 m to raise the nearest
  pair clearance above the 0.585 m robot-plus-margin contract.
- Profile: explicit `run_v6_kujiale_low_obstacles.sh` and additive
  `v6-low-obstacles` Isaac mode; default-off physical YAML; dedicated
  experiment scenario; dedicated Nav2 isolation overlay.
- Sensor contract: RGB, depth image, and depth points remain published under
  `rgbd_navigation`; raw depth points are absent from the V6 Costmap overlay.
- Commands: wrapper `bash -n`; focused pytest covering new layout, obstacle
  parser regression, scenario parsing, and Nav2 profile timing; `git diff
  --check`.
- Result: `103 passed`; no static geometry/contract failure.
- Verdict: **PASS (frozen configuration and static tests only)**.
- Unrun: Isaac/ROS/Nav2, camera visibility, contact, bypass behavior, estimated
  localization, causal experiment, and formal qualification.
- Handoff: `docs/handoff/V6_C4_LOW_OBSTACLE_LAYOUT.md`.

## 2026-08-20 — V6 estimated/Isaac odometry-mode alignment

- Goal: prevent the V6 estimated chain from also enabling Isaac ideal `/odom`
  and `odom -> base_link`, without changing legacy static/dynamic defaults.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `1b175aa8d3c1e23305ae9e7e924cb4d1b6c4309c`.
- Change: `v6-low-obstacles` now defaults Isaac to `realistic`; static/dynamic
  retain `ideal`; explicit trailing caller options override profile defaults.
- Validation: wrapper/V6 entry `bash -n` PASS; new focused script contract
  pytest `1 passed`. A combined sibling selection was `1 passed, 1 failed`
  because the existing Costmap-overlay assertion expects an obsolete one-line
  plugin list; that tracked overlay was not changed by this amendment.
- Verdict: **PASS (shell syntax and focused static contract only)**.
- Unrun: live ROS/Isaac publisher counts, TF ownership and navigation closure.

## 2026-08-20 — V6 activation-gate exact override amendment

- Goal: make launch-time gate policy/source/timeout/sim-time overrides use the
  same exact `nav2_activation_gate` key as the default parameter file.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `cb50199f3a7fd8ae61c6060c8addd938a2679ea7`.
- Change: replace the launch-generated wildcard parameter dictionary with a
  later exact-node runtime YAML; keep autonomy defaults `fail_closed/auto`.
- Validation: changed Python `py_compile` PASS; focused pytest `67 passed`;
  full `robot_bringup/test` pytest `214 passed`; clean pure-Jazzy
  `robot_bringup` build PASS at `/tmp/bionav_gate_exact_build.HtXiti`.
- Real parameter evidence: Jazzy `rclpy` loaded default plus runtime files into
  a same-name Node for both `fail_closed/auto` and `wait_for_seed/rviz`.
- Verdict: **PASS (implementation, parameter parsing, unit tests, clean build)**.
- Unrun: ROS graph, Isaac, Nav2, live 120-second/reseed behavior, navigation,
  evidence, and formal qualification. Handoff:
  `docs/handoff/V6_GATE_EXACT_OVERRIDE_20260820.md`.

## 2026-08-20 — V6 shadow wrapper RF2O default

- Goal: start topic-only RF2O from the V6 `shadow` entry while keeping the
  default EKF on wheel+IMU and preserving non-shadow defaults.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `1438caca4362205650d54a58022924f6073ecc48`.
- Change: shadow argv now defaults to `ekf_profile:=wheel_imu`,
  `lidar_odometry_backend:=rf2o`, and
  `lidar_odometry_validated:=false`; caller overrides remain trailing.
- Validation: wrapper `bash -n` PASS; focused argv contracts `4 passed`; full
  runtime-script pytest `30 passed`.
- Verdict: **PASS (shell syntax and isolated argv contracts only)**.
- Unrun: ROS/Isaac, live topic and TF ownership, navigation, metrics, and
  qualification. Handoff: `docs/handoff/V6_SHADOW_RF2O_WRAPPER_20260820.md`.

## 2026-08-21 — V6 M1 shadow live engineering observation

- Goal: observe M1 cognitive-obstacle telemetry while confirming that shadow
  mode cannot affect control; evidence directory:
  `/tmp/v6_live_m1_shadow.UDx2Bz`.
- Revisions: Integration `430e977884202d9800235b73eab320dd68e5f325`,
  Module2 `163dbdc4a469c37aced5a1a7c673b84b2765efe4`, Module3
  `faddfbd3b840450dfa49c1e6b87771a3613cd907` on `cognitive-navigation`.
- 30.01 s result: `14/14` obstacle messages nonempty, 18 objects throughout;
  observation/input/Module2 health true, trusted write false, TTL at most 0.5 s,
  stable identity, and sequence `3022 -> 3118`.
- Isolation: layer and critic `applied=false`, raised cells 0; `/cmd_vel`
  messages/nonzero `0/0`, `/initialpose=0`, `/goal_pose=0`. AMCL/Nav2 were
  active. RGB-D was enabled while the direct classic obstacle layer was used;
  voxel/STVL were absent from the runtime Costmap plugin lists.
- Limitations: layer samples fell back stale at about 1.7 s; scan invisibility
  was unverified; after Module2 stop the core Nav2 lifecycle remained active but
  no fresh layer status was captured.
- Verdict: **PASS for M1 telemetry/control isolation; PARTIAL overall**. This is
  not M2 active/causal navigation/formal qualification evidence.
- Cleanup: all run-owned launch and monitor processes were stopped; logs and
  PID records were retained under the evidence directory. No standalone exact
  launch-command transcript exists in that directory.
- Handoff: `docs/handoff/V6_M1_SHADOW_ENGINEERING_20260821.md`.

## 2026-08-21 — V6 Module3 obstacle validation consumer

- Goal: consume only fresh or depth-revalidated confirmed-static cognitive
  obstacles while preserving fail-open, max-only Costmap authority.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `b71fdf31644bd8a89dea70a91929e3bc537f8657`.
- Interface underlay: permitted Integration worktree at
  `4f294be1b9f2f0eff3fc27199082cb22b9ab9cdb`.
- Changes: exact dual-timeline/source-age and odometry gates; 0.5 s validation
  TTL and 50 ms future tolerance; strict fresh/static-depth modes; validation
  timestamp TF; private-layer clear on rejection; structured existing status
  fields; max-only active and zero-write shadow contracts.
- Validation: fresh interface build PASS; fresh `bio_nav_fusion` clean build
  PASS; colcon result 21 tests/0 failures; focused profile pytest 7 passed;
  `git diff --check` PASS. Temporary root:
  `/tmp/bionav_m3_obstacle_final.3stwVY`.
- Verdict: **PASS (implementation and code-level tests only)**.
- Unrun: live ROS/TF/Costmap, Isaac, Nav2 navigation, evidence campaign, and
  formal qualification.
- Handoff: `docs/handoff/V6_M3_OBSTACLE_VALIDATION_CONSUMER_20260821.md`.

## 2026-08-21 — V6 Isaac cold stage-readiness timeout repair

- Goal: tolerate multi-minute first-run RTX shader/PSO/cache compilation while
  retaining a bounded Kit context-stage readiness failure.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `ff7ab2724aba4095a0f334f7c50ae79690aeaee8`.
- Changes: required positive timeout config/env/CLI with 420-second default;
  warm-ready immediate return; post-update readiness check before deadline;
  30-second/phase-change cold-cache progress; actionable timeout diagnostics;
  real SimulationApp path now reuses the single-GPU launch contract.
- Validation: changed Python `py_compile` PASS; focused fake-clock/config/app
  pytest `29 passed`; `git diff --check` PASS.
- Verdict: **PASS (implementation and code-level tests only)**.
- Unrun: Isaac, ROS, Nav2, navigation, evidence, and formal qualification.
  Runtime cold-start confirmation remains for the next authorized retry.
- Handoff: `docs/handoff/V6_ISAAC_COLD_STAGE_READINESS_20260821.md`.

## 2026-08-21 — V6 Module3 typed cognitive graph feedback

- Goal: return physically grounded graph validation and concrete edge outcomes
  to Integration without conflating `SetRouteGraph` acceptance, lookahead
  completion, edge traversal, or fallback request/application.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `e328e27b4c4dcedc4748b59d39ec36bf38535152`.
- Interface underlay: permitted Integration revision
  `c1e411b6c579e4f07f72b9e768760ff1f13c2bb0`.
- Changes: typed validation/outcome publishers; candidate-to-concrete edge
  provenance; post-SetRouteGraph acceptance; RouteTracker crossing and final
  distance-gated success; once-only failure; requested/applied GVG fallback;
  reset and stale-callback cleanup.
- Validation: owned pytest `46 passed`; isolated `robot_route_planner` build
  PASS; package tests `71 tests, 0 errors, 0 failures, 1 skipped` (optional
  `pxr`); `git diff --check` PASS. Temporary root:
  `/tmp/v6_module3_graph_feedback.H2jtx5`.
- Verdict: **PASS (implementation/build/unit only)**.
- Unrun: live typed-event flow, ROS/Isaac/Nav2 navigation, causal Module2
  update, evidence campaign, and formal qualification.
- Handoff: `docs/handoff/V6_MODULE3_GRAPH_FEEDBACK_20260821.md`.

## 2026-08-21 — V6 A7 estimated navigation reviewer smoke

- Baseline: Module3 `e328e27b4c4dcedc4748b59d39ec36bf38535152`;
  Integration `c1e411b6c579e4f07f72b9e768760ff1f13c2bb0`.
- Evidence: `/tmp/v6_a7_estimated_isaac.Ikapdb`.
- Result: G2 `34.8 s`; recovery `0`; collision `0`; remaining `0.071 m`;
  EKF xy ATE RMSE `0.0689 m`; AMCL xy ATE RMSE `0.0629 m`.
- Verdict: **PASS (engineering smoke only; not qualification)**.
- Handoff: `docs/handoff/V6_A7_ESTIMATED_NAV_SMOKE_20260821.md`.

## 2026-08-21 — V6 formal single-episode runner contract

- Goal: add the minimal Module3 runtime-side contract for a future V6 pilot or
  formal episode without permitting draft manifests to dispatch.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `5d40626dbf3c8c29dfa577a7fb0b5c31ba43b61f`.
- Changes: independent zero-GT dispatcher; RouteCoordinator PRIMARY goal;
  exactly-once reset with endpoint/readiness, bridge epoch +1, and B5 seed
  gates; causal/control topic capture schema; six 20-row draft manifests;
  default-disabled wrapper and `NOT_QUALIFIED` pilot inspection.
- Draft matrix: Kujiale `7201..7220`, `7301..7320`, `7201..7220` and Rivermark
  `19301..19320`, `19401..19420`, `19501..19520`; dynamic `v1..v5` four each;
  four appearance profiles five each. All are explicitly unfrozen with missing
  asset/reset/route placeholders and formal fail-closed.
- Validation: changed Python `py_compile` PASS; focused pytest `26 passed`;
  wrapper `bash -n` PASS; `git diff --check` PASS; isolated
  `robot_experiments` build PASS at `/tmp/v6_formal_build.D1EGxV`; installed
  pilot inspection returned `NOT_QUALIFIED` without output creation.
- Verdict: **PASS (implementation/build/unit only)**.
- Unrun: ROS/Isaac/Nav2, live reset/route/capture, pilot episode, evidence,
  scene-contract freeze, and formal qualification.
- Handoff:
  `docs/handoff/V6_FORMAL_SINGLE_EPISODE_RUNNER_20260821.md`.

## 2026-08-21 — V6 low-obstacle M0--M3 causal runner/evaluator skeleton

- Goal: freeze the 12-row counterbalanced M0--M3 engineering matrix and add a
  pure offline causal evaluator without running or fabricating experiments.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `734330df30b7d0838ecf6b9b6b761892976fb706`.
- Frozen identity: Kujiale low-obstacle layout r1, seed `8601`, G1 to G2,
  PRIMARY/GVG, direct RGB-D Costmap off, 180 seconds; exact order
  `M0 M1 M2 M3 / M3 M2 M1 M0 / M1 M3 M0 M2`.
- Contracts: M0 disables Module2 UDS/bridge socket while preserving the same
  Integration estimated-autonomy localization; dispatcher zero-GT; passive GT
  recorder separate; exactly-once reset state plan; stale typed obstacles
  stop/fail open.
- Evaluator: synchronized scan invisibility and typed spatial matching;
  path Hausdorff/length; passive clearance/collision/success; M1 isolation;
  M2/M3 clearance/collision/direction checks; M3 offline critic attribution
  remains `AMBIGUOUS` without trajectory separation; visualization-input JSON.
- Validation: new Python `py_compile` PASS; wrapper `bash -n` PASS; focused
  causal + V6 formal pytest `38 passed`; `git diff --check` PASS; isolated
  `robot_experiments` build PASS at
  `/tmp/v6_low_obstacle_causal_build.S2ysce`; installed manifest inspection
  returned `ENGINEERING_CAUSAL_NOT_RUN 12`.
- Verdict: **PASS (implementation/build/unit only)**;
  **ENGINEERING_CAUSAL_NOT_RUN** for live evidence. No ROS/Isaac/Nav2,
  12-row campaign, causal result, visual result, or formal qualification ran.
- Handoff: `docs/handoff/V6_LOW_OBSTACLE_CAUSAL_RUNNER_20260821.md`.

## 2026-08-21 — V6 L0--L3 localization causal runner and true-kidnap skeleton

- Goal: freeze the 60-row localization causal contract, add an absolute-map
  passive evaluator, and provide a fail-closed physical kidnap primitive without
  fabricating live or causal evidence.
- Worktree / parent: Module3 `cognitive-navigation` at
  `9a9725729972d58d40c1614038fc720cbd8dad1f`; delivered in the single commit
  containing this ledger row.
- Changes: new L0--L3 manifest/plan/evaluator/entrypoint/tests; S0/S3/W0 seeds
  and counterbalanced order; S1/S2 preflight definitions; GT-free dispatcher;
  absolute position/yaw error, convergence/lost/recovery/P95/wrong-reseed and
  safety criteria; realistic-only explicitly armed one-shot
  `/simulation/kidnap` Trigger with fresh stable zero-command guard and
  pre/post articulation zeroing; minimal Isaac app wiring.
- Validation: pycompile PASS; focused new + V6 formal/causal/reset pytest
  `66 passed`; wrapper `bash -n` PASS; `git diff --check` PASS; isolated
  `robot_experiments` build and installed 60-row manifest/plan/NOT_RUN checks
  PASS at `/tmp/v6_localization_causal_build.61UoeJ`.
- Result: **PASS (code-level contract only)**. Core 60 and preflights are
  **ENGINEERING_CAUSAL_NOT_RUN**; no ROS/Isaac navigation, causal result, or
  qualification was claimed.
- Remaining: live Trigger/Kit behavior, route-idle adapter enforcement, actual
  S1/S2 preflights, core 60 execution, passive evidence capture, and review.
- Handoff: `docs/handoff/V6_LOCALIZATION_CAUSAL_RUNNER_20260821.md`.

## 2026-08-21 — V6 formal multi-leg engineering-pilot amendment

- Goal: replace the final-goal-only formal draft with complete five-leg
  candidate scenes and an explicitly non-qualified live engineering adapter.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `5b6e7b8854151c385790801cd96b645ad8eaad68`.
- Changes: filled six unfrozen candidate manifests with exact assets, calibrated
  missions, dynamic cases/variants and appearance profiles; sequential PRIMARY
  route-goal dispatch; exact-once dynamic trigger/complete with fail-stop;
  obstacle/appearance/causal/control JSONL capture; explicit
  `--pilot --dispatch-pilot` labelled `ENGINEERING_PILOT/NOT_QUALIFIED`.
- Validation: Python compile and shell syntax PASS; focused formal pytest
  `42 passed`; formal/causal/package regression `98 passed`; isolated
  `robot_experiments` build PASS at
  `/tmp/v6_formal_multileg_build.a5pQGC`; `git diff --check` PASS.
- Verdict: **PASS (implementation/build/unit only)**;
  **ENGINEERING_PILOT_NOT_RUN**. No ROS/Isaac/Nav2 pilot, scene freeze, evidence
  campaign, or formal qualification was performed.
- Handoff:
  `docs/handoff/V6_FORMAL_MULTILEG_ENGINEERING_PILOT_20260821.md`.

## 2026-08-21 — V6 Rivermark occupancy-only estimated bringup

- Goal: make the six Rivermark engineering-pilot scene/stack pairings runnable
  without fabricating a missing SLAM Toolbox posegraph.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `e7d0a1d3ac1cbd3598bcc670817ea3ea59b00079`.
- Changes: explicit fail-closed `occupancy_only` localization map contract;
  AMCL/occupancy/route-GeoJSON validation; legacy posegraph-bundle default
  retained; paired `isaac|ros` static/dynamic/appearance wrapper fixed to
  realistic Isaac sensors, RGB-D, passive GT, estimated EKF+AMCL, M3, PRIMARY
  and no RViz.
- Validation: shell syntax and changed Python compile PASS; contract plus new
  wrapper tests `32 passed`; isolated `robot_bringup` build PASS at
  `/tmp/v6_rivermark_bringup.iJSG4n`; `git diff --check` PASS. The broader
  runtime-script file had `60 passed` plus one unrelated pre-existing strict
  shell-style failure outside this task's ownership.
- Verdict: **PASS (implementation/build/unit only)**;
  **ENGINEERING_PILOT_NOT_RUN**. No ROS/Isaac/Nav2, scene freeze, evidence
  capture, or qualification was run.
- Handoff:
  `docs/handoff/V6_RIVERMARK_OCCUPANCY_ONLY_BRINGUP_20260821.md`.

## 2026-08-21 — V6 Estimated State calibration runner/evaluator

- Goal: freeze a measurable 3 m/360/S/Rivermark RF2O off-shadow-fused matrix
  and extend the passive evaluator without claiming that calibration ran.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `055bdeafd95d7335f7f8a07683ab759376174c7f`.
- Matrix: exact 45 arm-grouped episodes (36 indoor primitives plus 9
  Rivermark static-start-to-G1 routes), three repeats, one reset/zero retry.
- Contracts: dispatcher has no GT subscription; reset readiness uses Trigger
  success plus fresh `/clock`, estimated `/odom`, and `odom->base_link` TF;
  passive evaluator alone reads `/ground_truth/odom`; fused is fail-closed on
  15 shadow reports plus an explicit promotion flag.
- Metrics: absolute/aligned ATE, fixed 1 s/1 m RPE, endpoints, scale, CW/CCW
  bias, bounded time offset, stream health, covariance finite/symmetric/PSD,
  diagnostic 2-sigma coverage and planar NEES; NIS explicitly unavailable.
- Validation: changed Python compile PASS; focused calibration/metrics/motion/
  package tests `59 passed`; exact 45-row manifest smoke PASS; isolated
  `robot_experiments` build PASS at
  `/tmp/v6_estimated_calibration_build.qNYxfL`; `git diff --check` PASS.
  Full package collection was blocked by an unrelated stale external
  Integration overlay missing `CanonicalRoute` and was not treated as a code
  result.
- Verdict: **PASS (implementation/build/unit only)**;
  **CALIBRATION_NOT_RUN**. No ROS/Isaac/Nav2, tuning result, RF2O promotion, or
  qualification is claimed.
- Handoff:
  `docs/handoff/V6_ESTIMATED_CALIBRATION_RUNNER_20260821.md`.

## 2026-08-21 — V6 Module3 PRIMARY authority consumer amendment

- Goal: consume Integration's typed static-graph revalidation provenance,
  retain GVG during immature cognitive-graph bootstrap, and wait long enough
  for the bounded V6 goal-prior retry.
- Worktree / parent / dependency: permitted Module3 `cognitive-navigation` at
  `9c515892aeb76a2eef0363e91f6d870e99ecdd10`; Integration interfaces from the
  permitted worktree at `9373d9d9155198f8c84ac2f888e2c76b7b9ebc03`.
- Changes: legacy direct freshness preserved; static revalidation checks exact
  source/validation age, bounded validation/arrival TTL, health, identity,
  physical graph and numeric topology; immature PRIMARY/hybrid candidates keep
  GVG without SetRouteGraph/edge ack/fallback; shadow is observational;
  PRIMARY/hybrid prior wait is 4.0 s while legacy modes remain unchanged.
- Validation: changed Python compile PASS; focused `64 passed`; full package
  `88 passed, 1 skipped`; isolated Integration-interface build PASS and colcon
  test `89 tests, 0 errors, 0 failures, 1 skipped` at
  `/tmp/v6_m3_primary_cleanbuild.BErizZ`; `git diff --check` PASS. The skip is the
  optional `pxr` benchmark in the unit environment.
- Verdict: **PASS (implementation/build/unit only)**. No ROS/Isaac/Nav2/live
  PRIMARY/evidence/qualification campaign was run.
- Handoff:
  `docs/handoff/V6_MODULE3_PRIMARY_AUTHORITY_CONSUMER_20260821.md`.

## 2026-08-21 — V6 flat Estimated State calibration amendment

- Goal: move 3 m/360/S calibration out of the narrow indoor scene, record
  wheel/IMU/RF2O separately, and make collision/dispatcher/promotion evaluation
  fail closed without running calibration.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `df0b985a0a13fa4aa48dd396f8f5d37f417c4ae1`.
- Environment: official Grid `default_environment.usd` collision plane; runtime
  stationary config adds four boundary walls plus three asymmetric features;
  calibrated identity spawn; deterministic 404 x 404, 0.05 m flat20 map.
- Contracts: supplemental revision/exclusion mapping; six passive streams;
  RF2O-only shadow promotion; meaningful-motion scale denominators;
  primitive-specific gates; dispatcher/collision invalidation; matched
  canonical-route plus route-progress acceptance before Rivermark timeout.
- Validation: changed Python compile PASS; focused regression `67 passed`;
  related Isaac static `52 passed`; shell/diff checks PASS; isolated
  `robot_experiments` build PASS at
  `/tmp/v6_flat20_robot_experiments.YT6lXC`; 45-row manifest smoke PASS at
  `/tmp/v6_flat20_manifest.xuLCJH`.
- Verdict: **PASS (implementation/build/unit only)**;
  **CALIBRATION_NOT_RUN**. No Isaac/ROS/Nav2, tuning, RF2O promotion, PRIMARY
  replay, evidence campaign, or qualification was run.
- Handoff:
  `docs/handoff/V6_FLAT_ESTIMATED_CALIBRATION_AMENDMENT_20260821.md`.

## 2026-08-21 — V6 formal B5 cognitive bootstrap readiness

- Goal: replace the formal runner's incorrect pre-reset AMCL/prior and
  post-reset Isaac localization-seeded requirements with the active B5
  cognitive bootstrap contract.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `6071afc9727aa5a397bd918e66a442e05ffb2ee9`.
- Changes: all six final manifests select `b5_cognitive`; epoch-zero negative
  readiness; exactly-once physical reset; Bridge epoch 1 -> B5 consensus,
  initialpose, new AMCL and confirmation -> Bridge epoch 2/new session;
  same-generation trusted PlanningPrior plus Nav2/TF before GOAL_READY; no
  dependency on `/simulation/localization_seeded`.
- Validation: changed Python compile PASS; focused formal `53 passed`;
  formal/causal/calibration regression `90 passed`; isolated
  `robot_experiments` build PASS at `/tmp/v6_formal_b5_build.5JYXDP`;
  `git diff --check` PASS.
- Verdict: **PASS (implementation/build/unit only)**. No ROS/Isaac/Nav2,
  engineering pilot, evidence, or qualification campaign was run.
- Handoff:
  `docs/handoff/V6_FORMAL_B5_READINESS_20260821.md`.

## 2026-08-21 — V6 core sensor single physics-step publication

- Goal: stop Clock/JointState/IMU from receiving two bit-identical executions
  for one simulation stamp before the EKF can double-fuse equal-stamp samples.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `d5cd8d9f89b06465e4080b5dee3378d25b29daf1`.
- Changes: core sensor execution moved from `OnPlaybackTick` to exactly one
  `OnPhysicsStep`; Clock and JointState execute directly once, while IMU uses
  the ordered `physics step -> ReadIMU -> PublishIMU` chain. Topics, frames,
  QoS, publisher counts, and independent RTX LiDAR playback semantics remain
  unchanged.
- Validation: changed Python compile PASS; focused graph contracts `6 passed`;
  all static Isaac tests `184 passed, 11 skipped` (existing `pxr`-unavailable
  stage-composition skips); `git diff --check` PASS.
- Verdict: **PASS (implementation/static-test only)**. No Isaac/ROS/Nav2 was
  launched, so live single-stamp publication remains to be confirmed. IMU
  covariance is still zero/unspecified pending Estimated State calibration;
  no covariance value was guessed here.
- Handoff:
  `docs/handoff/V6_CORE_SENSOR_PHYSICS_STEP_20260821.md`.

## 2026-08-21 — V6 motion-assist pure-yaw consistency amendment

- Goal: remove the flat-calibration IMU/GT pure-yaw scale inconsistency without
  disabling skid-steer assist or changing its nonzero arc behavior.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `74270f90e310bff0acfaa2b16e970c85928ca713`.
- Input evidence: raw IMU/GT yaw scale mean `1.081260` versus
  `1 / 0.925 = 1.081081` (`0.0166%` difference); assist is applied after the
  current physics-step sensor publication.
- Change: pure in-place yaw scale is exactly `1.0`; nonzero arc interpolation,
  linear correction, acceleration/timeout behavior and all configuration are
  unchanged. Static coverage locks the post-physics-step execution order.
- Validation: changed Python compile PASS; focused motion/graph contracts
  `11 passed`; all Isaac static tests `185 passed, 11 skipped` (optional `pxr`
  unavailable); no Isaac/ROS/Nav2 runtime was launched.
- Verdict: **PASS (implementation/static-test only)**. Live retest remains:
  six flat20 rotations plus duplicate equal-stamp sensor checks, followed by
  Estimated parameter acceptance before any affected Rivermark/PRIMARY rerun.
- Handoff:
  `docs/handoff/V6_MOTION_ASSIST_YAW_CONSISTENCY_20260821.md`.

## 2026-08-21 — V6 core sensor on-demand execution

- Goal: remove the Isaac materialization fatal caused by placing
  `OnPhysicsStep` in a core Sensors graph using the default execution
  evaluator.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `c96f434811b5c698b1d5157d91e5af1a23500eed`.
- Change: `/World/Graphs/Sensors` now materializes with
  `GRAPH_PIPELINE_STAGE_ONDEMAND`, matching the Control graph contract; its
  nodes, physics-step edges, topics, frames, QoS, target prims, and timestamps
  are unchanged, and the separate RTX LiDAR graph is untouched.
- Validation: changed Python compile PASS; focused graph contracts `7 passed`;
  all static Isaac tests `186 passed, 11 skipped` (existing optional
  `pxr`-unavailable stage-composition skips); `git diff --check` PASS.
- Verdict: **PASS (implementation/static-test only)**. No Isaac/ROS/Nav2 was
  launched. A fresh live retry must confirm graph materialization and
  once-per-physics-step Clock/JointState/IMU publication.
- Handoff:
  `docs/handoff/V6_CORE_SENSOR_ONDEMAND_20260821.md`.

## 2026-08-21 — V6 IMU raw/corrected calibration seam

- Goal: retain raw IMU audit evidence while applying the flat-arena yaw-rate
  scale before the EKF without changing GT, TF, control, or route ownership.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `af5c3d4b618edfb97936e85b00a7d489d45a98bb`.
- Contract: Isaac `/imu/data_raw`; one bounded calibrator publishes EKF-bound
  `/imu/data`; default `(raw_z - 0.0) * 0.9294`; yaw variance `1.0e-4`; exact
  stamp/frame/orientation/linear acceleration/other covariance preservation;
  duplicate/backward/non-finite fail closed; explicit identity rollback YAML.
- Evaluator: raw and corrected IMU integrations, yaw scale/bias, and angular-z
  covariance diagnostics remain separate; GT stays evaluator-only.
- Validation: Python compile and focused `24 passed`; related source regression
  `314 passed`; new-file flake8 PASS; isolated 16-package build PASS at
  `/tmp/v6_imu_calibration_build.6H1KpP`; isolated tests odometry `37/37`,
  bringup `224/224`, experiments `416/417`. The one experiments failure is an
  existing checkout-sensitive frozen absolute-path comparison, not this seam.
- Verdict: **PASS (implementation/build/unit only)**. No ROS/Isaac/Nav2 live
  retest or calibration/PRIMARY/qualification run was performed.
- Handoff:
  `docs/handoff/V6_IMU_RAW_CORRECTED_CALIBRATION_20260821.md`.

## 2026-08-21 — V6 Estimated final policy freeze

- Goal: freeze V6 final Estimated state to wheel + calibrated IMU with RF2O
  off, and prevent the wheel publisher shutdown race.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `4b57893bfbf430aead98e5c19d1c445149c54d55`.
- Input evidence: `/tmp/v6_imu_calibration_live.VAf50R`; RF2O shadow about
  10 Hz versus the 15 Hz floor, with no fused rows, so promotion remains
  blocked.
- Changes: all six V6 final manifests and final/PRIMARY wrappers select
  `wheel_imu`, calibrated IMU, RF2O `off`, and unvalidated; formal preflight
  rejects shadow/fused policy drift; an explicit pilot override remains
  `NOT_QUALIFIED`; wheel callbacks/publish now fail quietly only during
  confirmed shutdown while live-context `RCLError` remains visible.
- Validation: changed Python compile, shell syntax, and diff check PASS;
  focused `98 passed`; isolated 3-package build PASS at
  `/dev/shm/v6_final_policy_build.kx9nXV`.
- Verdict: **PASS (implementation/build/unit only)**. No ROS/Isaac/Nav2 or
  qualification run was launched. Rivermark/PRIMARY runtime evidence remains
  to be collected under the frozen policy.
- Handoff: `docs/handoff/V6_ESTIMATED_FINAL_POLICY_20260821.md`.

## 2026-08-21 — V6 obstacle FRESH/continuous-validation consumer amendment

- Goal: align Module3 obstacle admission with FRESH zero-odom semantics and
  permit bounded continuous depth revalidation of one source observation.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `3481749d459f545714a0bea3c89be60a76691809`.
- Changes: FRESH accepts zero/zero odometry stamps and requires a zero sensor
  mask; present FRESH odometry stamps must match. Static revalidation keeps its
  strict positive dual timeline. A compound source/validation cursor permits
  only strictly newer same-source static refreshes and rejects duplicate,
  backward, identity-drift, and new-source time-regression inputs. Reset clears
  the cursor; accepted refreshes replace the prior payload and validation TTL.
- Validation: isolated `bio_nav_fusion` build PASS; gtests PASS, 24 tests / 0
  errors / 0 failures / 0 skipped; `git diff --check` PASS. Build root:
  `/tmp/v6_obstacle_consumer_refresh.YAfw9B`.
- Verdict: **PASS (implementation/build/unit only)**. No ROS/Isaac/Nav2,
  engineering evidence, or qualification campaign was run; live costmap refresh
  and stale-cell clearing remain runtime validation work.
- Handoff:
  `docs/handoff/V6_M3_OBSTACLE_VALIDATION_CONSUMER_20260821.md`.

## 2026-08-21 — V6 obstacle status consumer identity

- Goal: remove global/local CognitiveObstacleLayer ambiguity on the shared
  absolute status topic while leaving Costmap behavior unchanged.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `dff45d0434638b9282bf6ea61811f09f638bb0cf`.
- Changes: optional plugin-scoped `consumer_id`; otherwise deterministic
  `<fully-qualified costmap node>:<layer name>` identity; every status uses the
  resolved identity. Empty inputs have a stable explicit fallback.
- Validation: isolated `bio_nav_fusion` build PASS; package result `26 tests, 0
  errors, 0 failures, 0 skipped`; `git diff --check` PASS. Build root:
  `/tmp/v6_obstacle_consumer_identity.mdIXgn`.
- Verdict: **PASS (implementation/build/unit only)**. No ROS/Nav2/Isaac or live
  evidence was run; actual lifecycle node names and both consumer streams remain
  to be observed in the next authorized live campaign.
- Handoff:
  `docs/handoff/V6_OBSTACLE_CONSUMER_IDENTITY_20260821.md`.
