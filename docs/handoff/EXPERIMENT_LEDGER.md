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

## 2026-08-21 — V6 IMU regime dependence; global candidate rejected

- Goal: reconcile the accepted flat20 rotation calibration with the later
  mixed-motion failure without changing the wheel+IMU/RF2O-off architecture.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `f63fd23ab9dfe9b19d9d5f2e7f44c7d8eff90ca0`.
- Mixed-route runtime evidence:
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_estimated_dynamic_smoke_20260821T150407Z`;
  exact snapshot Integration `2dd3aa937ae470d497cd97722302281efcc2e3f0`,
  Module3 `3dc2830c1da5b5f441191217220bc120058bd4b2`, Module2
  `2925f806c88b1551d1c48ca89d1c1c5adf2ba748`.
- Result: navigation succeeded (`51.93 s`, final goal error `0.195 m`, zero
  actual/physical collisions), but `0.9294`
  worsened raw-to-corrected aligned yaw RMSE from `0.08527` to `0.12342 rad`,
  full endpoint absolute error from `0.08447` to `0.21168 rad`, and goal-window
  endpoint absolute error from `0.13643` to `0.18516 rad`. Verdict:
  **ENGINEERING FAIL**; formal qualification was not run.
- Tradeoff: old rotation closure `<=5 deg` permits
  `k=[0.917435, 0.940927]`; mixed-route full-window identity non-degradation
  requires `k=[0.962746, 1.0]`. The empty intersection rejects one global
  constant. Offline route fits `0.9814` (full) and `0.9700` (goal) remain
  diagnostic candidates only; the dirty `0.9814` code/config WIP was rejected
  and `0.9294` remains the committed rotation-valid baseline.
- Separate debts: requested dynamic seed `8601` became reset seed `0`; strict
  `/cmd_vel` unique-publisher evidence also failed because collision monitor
  and Isaac reset-zero publishing coexisted. Neither debt is folded into the
  IMU scale conclusion.
- Verdict: **ENGINEERING FAIL (mixed route); rotation baseline remains valid**.
  No ROS/Isaac/Nav2/evidence/qualification run was launched by this amendment.
- Handoff: `docs/handoff/V6_IMU_REGIME_DEPENDENCE_20260821.md`.

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

## 2026-08-21 — V6 obstacle static source-age contract alignment

- Goal: align Module3's static-depth-revalidated source-age cap with the
  Integration producer's five-second static retention contract.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `b1e37922a1191c3634881138e5d304620ab3abc6`.
- Changes: static revalidation accepts exact nonnegative source ages through
  `5.0 s` and rejects `5.01 s` as `source_age`; FRESH remains exact-zero and
  rejects `2.2 s` as `fresh_mismatch`. TTL/future/odom/depth/sequence/TF gates
  are unchanged.
- Validation: isolated `bio_nav_fusion` build PASS; package result `26 tests,
  0 errors, 0 failures, 0 skipped`; `git diff --check` PASS. Build root:
  `/tmp/v6_obstacle_static_source_age.RmMSTu`.
- Verdict: **PASS (implementation/build/unit only)**. No ROS/Nav2/Isaac or live
  evidence was run; both live Costmap consumers still require observation.
- Handoff:
  `docs/handoff/V6_OBSTACLE_STATIC_SOURCE_AGE_20260821.md`.

## 2026-08-21 — V6 critic static-revalidation freshness alignment

- Goal: prevent MPPI's `CognitiveRiskCritic` from rejecting a static obstacle
  that the Costmap layer accepts on a fresh depth-revalidation timeline.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `e55ccc13fbd01479cd1ad20aefca800f9e218d73`.
- Changes: critic obstacle admission now reuses
  `CognitiveObstacleLayer::validateMessage()` with the planning-prior identity,
  so dual-timeline freshness, static confirmation, identity, trust, OOD, and
  malformed-data gates have one verdict. Planning-prior freshness remains a
  separate 0.5-second fail-open gate; critic authority and scoring are
  unchanged.
- Validation: fresh isolated `bio_nav_interfaces` plus `bio_nav_fusion` build
  PASS at `/tmp/bio_nav_module3_critic.JLGsAi`; focused fusion gtest PASS,
  28/28; plugin loader isolation PASS, 1/1; `git diff --check` PASS. The first
  build at `/tmp/bio_nav_module3_critic.mUTBid` stopped before compiling this
  change because a pre-existing interface overlay lacked generated headers;
  the successful run rebuilt the interface from the allowed source.
- Verdict: **PASS (implementation/build/unit only)**. No active MPPI, ROS,
  Nav2, Isaac, navigation, engineering evidence, or qualification campaign was
  run.
- Next: in an authorized live run, publish one source observation older than
  0.5 seconds with a fresh static-depth validation and confirm both Costmap and
  critic status report applied, then allow the validation TTL to expire and
  confirm zero cognitive cost.
- Handoff:
  `docs/handoff/V6_CRITIC_STATIC_REVALIDATION_FRESHNESS_20260821.md`.

## 2026-08-22 — V6 route terminal and graph reassert-liveness repair

- Goal: close the final RouteCoordinator review blockers: exactly-once paired
  route terminals under reset/rejection concurrency, and bounded steady-clock
  Route Server reassertion after unavailable, rejected, exceptional, or hung
  `SetRouteGraph` transactions.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `ea6f532554177f8256c194f67449dae622b009a8`.
- Changes: rejection/reset/final outcomes share an output/state lock order,
  synchronously retire current route state, and emit one Bool+JSON pair;
  duplicate late callbacks, fallback, intermediate success, and preemption are
  silent. Added a 0.1 s steady reconciliation timer, 2.0 s transaction deadline,
  generation/route/graph-bound retry key, 0.25-to-2.0 s capped backoff, no-storm
  dispatch, uncancelled late-future consumption, stale-failure isolation, and
  stale-success fail-closed compensation for cognitive and structural requests.
- Validation: focused route/graph **90 passed**; full route package **114
  passed, 1 skipped** (`pxr` unavailable); associated reset/gate/IMU/benchmark
  regression **65 passed**; isolated `robot_route_planner` colcon build/test
  PASS at `/tmp/v6_route_terminal_final_build.NRRMgN` with install
  `/tmp/v6_route_terminal_final_install.R7UZTF`; colcon result **115 tests,
  0 errors, 0 failures, 1 skipped**; `py_compile` and `git diff --check` PASS.
  The first associated-test invocation lacked the ROS environment and had
  three collection errors; the source-first rerun after
  `source /opt/ros/jazzy/setup.bash` is the cited passing result. An initial
  colcon-test invocation from the workspace root stopped during duplicate-name
  discovery and ran 0 tests; the passing rerun was constrained to the allowed
  Module3 `ros2_ws`.
- Result: **PASS (code/build/unit only)**. No ROS/Isaac/Nav2/navigation runtime,
  visual evidence, engineering campaign, or formal qualification was run.
- Remaining risk/next step: executor/action-server/DDS timing and actual active
  reset/reassert behavior are still unverified; run the planned bounded live
  active-reset review before making an engineering-runtime claim.
- Handoff: `docs/handoff/V6_ROUTE_RESET_RETIREMENT_20260822.md`.

## 2026-08-22 — V6 IMU smoother-aware schedule evidence and Attempt 3 retroactive analysis

- Goal: bind Session A segments to reset generation plus upstream intent
  without treating velocity-smoother/CollisionMonitor/gate plateaus as segment
  boundaries, and retain useful IMU metrics when the benchmark performance
  gate fails.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `6af4fde9b6b6eaa095f4c0f25ff34a561766ecc8`.
- Changes: MotionBenchmark schema 2 records immutable sim-time segment and
  stationary schedules plus zero publish receipts; playback commands/timing
  and thresholds are unchanged. The analyzer consumes exact `/clock`, reset,
  upstream intent, smoother, final command, and gate-output MCAP streams;
  verifies generation/schedule/intent binding, HOLD and stream coverage, and
  downstream final-zero continuity; and separates capture from performance
  verdicts. A failed performance gate keeps all segment k-star, endpoint,
  RMSE/P95, and interval metrics but forces overall FAIL and
  `scale_selection_authorized=false`.
- Retroactive input:
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_imu_regime_session_a_attempt3_20260821T220048Z`.
  New derived output only:
  `analysis/smoother_schedule_v2/imu_regime_analysis.json`; originals were not
  changed. Result: 12 windows; **FAIL / NOT FORMAL**;
  performance FAIL, capture AMBIGUOUS, no scale authorization. The old capture
  lacks 0.8 s final `/cmd_vel_sim` zero coverage for stationary plus eight
  single primitives. Pure-spin k-star is 0.928739/0.929535; all segment
  <=5-degree intervals intersect at `[0.9162,0.9417]`, but goal evidence and
  an acceptable performance run are absent.
- Validation: source-first/fresh-installed focused suites **97 passed**;
  isolated build/install PASS at `/tmp/v6_imu_schedule_final3_build.YDJFR2`
  and `/tmp/v6_imu_schedule_final3_install.usJHFc`, log
  `/tmp/v6_imu_schedule_final3_log.G2E8gL`; installed entrypoint/import,
  `py_compile`, and `git diff --check` PASS.
- Verdict: **PASS (code/build/unit plus retroactive offline analysis only)**.
  Attempt 4 and the goal run remain pending. `yaw_scale=0.9294`, RF2O-off,
  route/reset/gate policy, command values, and thresholds are unchanged.
- Wider package regression: **529 passed, 1 unrelated path-sensitive
  failure** in the frozen Rivermark-reference absolute-path comparison; no
  runtime/numeric mismatch and no out-of-scope fix attempted.

## 2026-08-22 — V6 active-reset HOLD-intent repair

- Goal/hypothesis: close the active-reset live failure where RouteCoordinator
  retained the old request until the later Empty reset event; the existing
  StopGate HOLD status should fence and retire route intent first.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `b2523a812c1fad832b9aa87622e30b60b8681015`.
- Triggering evidence/result:
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_active_reset_live_20260821T190834Z`;
  generation-2 HOLD preceded wheel reset by about 59 ms, old Nav terminal by
  about 186 ms, and event-only RouteCoordinator completion by about 602 ms.
  Original run remains **ENGINEERING FAIL / NOT FORMAL**.
- Changes: RouteCoordinator consumes strict reliable/transient-local StopGate
  status; higher HOLD performs one reset begin/active abort/cancellation and
  fences old callback/output/dispatch state; same-generation Empty event only
  completes runtime/GVG reset work; same-generation release opens the goal
  barrier only after completion. Missing event, malformed/backward/conflicting
  status, and early release stay fail closed. Startup released baseline and
  legacy event-only behavior remain non-terminal/compatible respectively.
- Commands/results: source-first complete route pytest **132 passed, 1
  skipped** (`pxr` unavailable); associated gate/activation/mode/profile/
  benchmark pytest **98 passed**; combined rerun **230 passed, 1 skipped**;
  changed-file `py_compile` and `git diff --check` PASS. Fresh isolated package
  build PASS at `/tmp/v6_route_hold_build.HMD46a` with install
  `/tmp/v6_route_hold_install.PohSre` and log
  `/tmp/v6_route_hold_log.74aLob`; isolated `colcon test` **133 tests, 0
  errors, 0 failures, 1 skipped**, log
  `/tmp/v6_route_hold_test_log.95Ce9m`.
- Verdict: **PASS (code/build/unit only)**. No ROS/Isaac/Nav2/navigation or
  qualification run was launched. No DDS/executor/cancellation timing guarantee
  is claimed from unit evidence.
- Blocker/next: active-reset live rerun remains required to prove one abort at
  HOLD, zero old-request output/motion through HOLD, same-generation completion
  and release, and a successful fresh post-release goal.
- Handoff: `docs/handoff/V6_ROUTE_RESET_RETIREMENT_20260822.md`.

## 2026-08-22 — V6 IMU reset/request timestamp boundary closure

- Goal: prevent a reset-equal stale nonzero command from entering a recorded
  route request's successful goal window without rejecting a legitimate
  no-request attempt whose first command is stamped at reset.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `1d3a2d8d990c85daa24c25dd282e818a36dc5329`.
- Changes: the recorded-request pre-motion fence is now
  `reset_s <= t < request_s`; the no-request conservative path explicitly
  retains reset as its valid attempt boundary. Added reset-equal, strict
  pre/post-reset, request-boundary, and no-request positive adversarial tests.
- Validation: source-first **47 passed**; fresh isolated build/install PASS at
  `/tmp/v6_imu_reset_boundary_build.lrnEgy` and
  `/tmp/v6_imu_reset_boundary_install.Vw4VgR` (log
  `/tmp/v6_imu_reset_boundary_log.okL1lC`); installed **47 passed**; installed
  import identity and `imu_regime_analysis --help` PASS. Terminal, stream-gap,
  duration, and installed-share regressions remain passing.
- Verdict: **PASS (code/build/unit only)**. No Isaac/ROS/MCAP/navigation live
  run or formal qualification; live regime closure remains pending.
- Handoff:
  `docs/handoff/V6_IMU_REGIME_EVIDENCE_CONTRACT_20260822.md`.

## 2026-08-22 — V6 IMU preflight sim-time and flat20 geometry closure

- Goal: close the remaining preflight clock-domain and installed-feature
  geometry review blockers without changing `yaw_scale=0.9294`, RF2O-off,
  command authority, route/reset/benchmark, or critic behavior.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `78d49a1f9d3cc9ece74400682ff5ab55d7efabc0`.
- Changes: the read-only LiDAR gate locks `use_sim_time=true`, verifies a
  `ROS_TIME` clock, rejects ROS CLI overrides, and treats zero/non-finite clock
  as STOP. Freshness uses ROS simulation `now` against message header stamps.
  The offline analyzer now strongly compares the complete installed flat20
  feature YAML, including exact seven IDs, cube size/height/scale, pose, mass,
  stationary/parked/zero-velocity policy, seed, and count. Geometry, motion,
  seed, and count mutation fixtures fail closed; both spawn aliases bind the
  exact map/USD origin without treating the legacy digest as authority.
- Validation: source-first focused **72 passed**; fresh isolated package build
  PASS at `/tmp/v6_imu_simtime_build.woJmcR`, install
  `/tmp/v6_imu_simtime_install.b6pBGp`, log
  `/tmp/v6_imu_simtime_log.3NGRES`; fresh-installed import and focused tests
  **72 passed**; installed analyzer/preflight `--help`, node clock test,
  `py_compile`, and `git diff --check` PASS.
- Verdict: **PASS (code/build/unit only)**. No Isaac, live readiness, ROS/Nav2
  navigation, stationary/primitive/MCAP run, scale selection, or formal
  qualification was performed. Attempt 3 remains **PENDING**.
- Handoff:
  `docs/handoff/V6_IMU_REGIME_EVIDENCE_CONTRACT_20260822.md`.

## 2026-08-22 — V6 RouteCoordinator late-join startup reset baseline

- Goal: recover the expected transient-local depth-1 startup sequence where a
  late RouteCoordinator first sees `reset_complete`, without weakening the
  missing-HOLD fail-closed rule after route-era state exists.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `ba941acddadc785fd0938be24e28ef13fd65f8a5`.
- Attempt 2 evidence:
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_active_reset_live_attempt2_20260821T212047Z/`;
  **ENGINEERING FAIL / STOP / NOT FORMAL** before goal/Trigger. At
  `1787347473.815673754`, generation 1 `reset_complete` was rejected as
  `higher generation did not begin with HOLD`; at `1787347479.391320030`, the
  same-generation release was rejected as
  `same-generation transition without reset HOLD`. Evidence files:
  `summary.md`, `metrics/status_sequence.json`, `setup_bag_analysis.json`.
- Change: accept first strict `reset_complete` only for a provably pristine
  coordinator; synchronize intent/completion, reset/GVG baseline, and keep the
  barrier held until same-generation release. No fake HOLD, route terminal, or
  navigation cancellation. Duplicate is idempotent; backward/conflict,
  active/old state, and later no-HOLD completion remain fail closed; first
  released and legacy Empty-event behavior remain.
- Validation: RouteCoordinator file **64 passed**; complete source-first route
  package **164 passed, 1 skipped** (`pxr` unavailable); fresh isolated
  build/test PASS at `/tmp/v6_route_startup_build.6AfGxd`, install
  `/tmp/v6_route_startup_install.sqc5Ow`, log
  `/tmp/v6_route_startup_log.LPvRP3`; colcon result **165 tests, 0 errors, 0
  failures, 1 skipped**.
- Verdict: **PASS (code/build/unit only)**. Attempt 2 remains STOP. Attempt 3
  is **PENDING**; no Isaac/ROS/Nav2/navigation/live engineering or formal
  qualification was run by this amendment.
- Handoff: `docs/handoff/V6_ROUTE_RESET_RETIREMENT_20260822.md`.

## 2026-08-22 — V6 RouteCoordinator HOLD graph-output/dispatch fence

- Goal: close the two remaining HOLD races between cognitive/structural final
  validation and `SetRouteGraph` dispatch, and prevent old cognitive/fallback
  outputs after reset retirement.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `dc09e9eebb0c73cc23d4c72eddf0e6bdc05a9944`.
- Changes: output-lock then state-lock revalidation for cognitive graph error,
  validation and fallback outputs; output-serialized cognitive/structural
  request construction, `call_async`, future and callback registration; an
  explicit current reset-completion token for the necessary GVG reassert while
  HOLD remains closed.
- Deterministic coverage: export error, unavailable service, request exception,
  cognitive/structural dispatch, fallback success/request outputs, reset-GVG
  positive path, and same-generation release positive path.
- Validation: focused graph adapter **64 passed**; full route package **147
  passed, 1 skipped** (`pxr` unavailable); associated reset/gate/mode/
  benchmark/reset-receipt/IMU/localization/EKF tests **200 passed**; fresh
  isolated build PASS at `/tmp/v6_route_dispatch_build.Qy6KPR` with final
  `colcon test-result` **148 tests, 0 errors, 0 failures, 1 skipped**; changed-
  file `py_compile` and `git diff --check` PASS.
- Verdict: **PASS (code/build/unit only)**. No ROS/Isaac/Nav2/navigation,
  evidence campaign, or formal qualification was run.
- Remaining risk/next: the planned fresh active-reset live rerun must still
  verify DDS/executor timing, zero old output/command through HOLD, and fresh
  goal recovery.
- Handoff: `docs/handoff/V6_ROUTE_RESET_RETIREMENT_20260822.md`.

## 2026-08-22 — V6 IMU duration, goal-MCAP, and installed-resource closure

- Goal: close the remaining evidence-contract paths that could promote a
  short primitive, hand-authored goal yaw array, stale source-tree resource,
  non-integer seed/generation, or hidden reset.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `238be5522f74b6f25af76e5baa3d0980ea7c7be6`.
- Changes: installed package-share manifest for the diagnostic config and
  flat20 spawn resource; installed-path runner/provenance; config/report/phase/
  common-grid duration and threshold checks; exact integer and consecutive
  reset receipts; direct `--goal-mcap` derivation from raw IMU, GT, final
  command, reset event/receipt log, collision, and route completion. Goal JSON
  is metadata-only and manual arrays are ignored. MotionBenchmark gained only
  an evidence report `duration_sec` field; playback behavior is unchanged.
- Validation: focused source-first/no-cache **80 passed**; full package plus
  related Isaac **506 passed** with one unrelated frozen-reference absolute-
  path mismatch; fresh isolated build/install PASS at
  `/tmp/v6_imu_contract_build.BJXc1r`,
  `/tmp/v6_imu_contract_install.KHMxhH`,
  `/tmp/v6_imu_contract_log.45afly`; installed focused **37 passed** and real
  `ros2 run`/package-share lookup PASS; pycompile, `bash -n`, and diff check
  PASS.
- Verdict: **PASS (code/build/unit only)**. No Isaac/ROS/MCAP/motion/goal
  runtime or formal qualification was run. `yaw_scale=0.9294` and RF2O off are
  unchanged; live regime capture remains pending.
- Handoff: `docs/handoff/V6_IMU_REGIME_EVIDENCE_CONTRACT_20260822.md`.

## 2026-08-22 — V6 IMU regime diagnostic instrumentation

- Goal/hypothesis: add passive loop-phase evidence and a reproducible offline
  scale-intersection analysis so the pure-rotation versus mixed-route conflict
  can be tested without changing the frozen 0.9294 scale during capture.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `7df6be9b95de31f1c6b41aa9c5f7d8135f799134`.
- Changes: default-off getter-only IMU/MotionAssist/GT phase JSONL; frozen GT
  publication receipt; strict nine-primitive MotionBenchmark diagnostic YAML;
  rosbag2_py MCAP offline analyzer and console entry point. The trace is not a
  command diagnostic mode and adds no publisher, control setter, graph
  evaluation, app update, sleep, or command. Missing attributes remain
  explicit errors/nulls. No yaw-scale, RF2O, reset, route, Integration, or
  Module2 changes.
- Validation: source-first/no-cache focused plus regression pytest **82
  passed**; final new-only tests **12 passed**; py_compile, strict YAML and
  `load_motion_config`, source-first import, and `git diff --check` PASS; fresh
  isolated `robot_experiments` build PASS at
  `/tmp/v6_imu_regime_final_build.trCpFF` with install
  `/tmp/v6_imu_regime_final_install.NxCfJS` and log
  `/tmp/v6_imu_regime_final_log.UMrBl3`.
- Verdict: **PASS (CODE / BUILD / UNIT ONLY)**. No Isaac, ROS graph, MCAP,
  benchmark primitive, stationary window, navigation goal, visual evidence,
  engineering campaign, or formal qualification was run. This is not new
  calibration evidence and does not change `yaw_scale=0.9294` or RF2O-off.
- Next: bounded same-session stationary + CW/CCW + bilateral low/normal-speed
  arcs + S route + Kujiale goal capture, then offline scan. Missing required
  streams/attributes or stamp anomalies must remain FAIL/AMBIGUOUS.
- Handoff:
  `docs/handoff/V6_IMU_REGIME_DIAGNOSTIC_INSTRUMENTATION_20260822.md`.

## 2026-08-22 — MotionBenchmark STOP evidence isolation and receipt retention

- Goal/hypothesis: ensure a reset/dispatch STOP is attributable only to its
  current primitive and remains reproducible from the emitted report.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `9cd4b81963f7ddc06cdeb648cf49d2ce5a790331`; fixed Module3 main
  `22d66470c4b903349b2467dc876490bbebfc0083` remained an ancestor. Final commit
  is the commit containing this ledger row and is reported to master.
- Changes: primitive samples/collision/recording/segment/command/current-receipt
  state is cleared before every fallible primitive step. Parsed reset receipts
  are immediately retained before the dispatch barrier, so STOP rows and the
  top-level list include seed/generation/case/variant/full response. Reports
  now include reset settle, state freshness, stamp coherence, clock stall,
  dispatch timeout, CollisionMonitor, ResetStopGate, and motion thresholds.
  The estimated-calibration TF contract uses AST semantics instead of an exact
  source-format string.
- Validation: source-first/no-cache focused MotionBenchmark/V6 pytest **39
  passed**; expanded benchmark/reset/estimated-state/ResetStopGate/
  ActivationGate contract pytest **106 passed**; fresh isolated
  `robot_experiments` build and installed import PASS at
  `/tmp/bionav_motion_evidence.u42NYu`; changed-file `py_compile` and final
  `git diff --check` are recorded in the commit handoff.
- Result: **PASS (code/build/unit evidence closure only)**. The second primitive
  dispatch-STOP test records zero samples, no inherited collision, an intact
  complete receipt, and an unchanged first row. Clock-stall zero output/STOP
  remains covered.
- Limits/next: no Isaac, ROS graph, Nav2, navigation, visual evidence,
  engineering campaign, or formal qualification was run. Live reviewer
  closure remains **PENDING**; route handoff and route code are unchanged.
- Handoff: `docs/handoff/V6_RESET_SEED_STOP_GATE_20260822.md`.

## 2026-08-22 — V6 RouteCoordinator reset retirement

- Goal/hypothesis: synchronously retire every old-epoch route/action intent on
  simulation reset so StopGate release cannot resume an old tracker or dispatch
  a stale NavigateToPose goal; also clear an old tracker before normal goal
  preemption exposes the new request.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `0a0fdf8adccaf95bfdf4d933993fa926ba6f3be4`, fixed-main ancestor
  `22d66470c4b903349b2467dc876490bbebfc0083`.
- Changes: added request/graph generation fencing and synchronous route state
  retirement; best-effort accepted-handle cancellation after clearing state;
  stale pending-action acceptance cancellation; runtime edge and epoch-bound
  pose/costmap/TF/region/structural-candidate cleanup; preemption tracker
  retirement; and one active-reset Bool false plus JSON abort terminal on
  `/bio_nav/route_goal_result`. No active route produces no terminal.
- Validation: focused source-first/no-cache route/reset tests **69 passed**;
  complete route package **93 passed, 1 skipped** (`pxr` unavailable); changed
  Python `py_compile` and `git diff --check` PASS; fresh isolated build PASS and
  `colcon test` **94 tests, 0 errors, 0 failures, 1 skipped** at
  `/tmp/bionav_route_reset.0m4Bm7`.
- Verdict: **PASS (code/build/unit only)**. No ROS, Isaac, Nav2, navigation,
  visual evidence, campaign, or formal qualification was run.
- Remaining blocker: StopGate active-reset live closure remains pending the
  combined read-only reviewer; verify exactly one abort terminal, no old route
  publications/action dispatch or post-release command, idempotent cancel-all,
  and successful fresh-goal restart.
- Handoff: `docs/handoff/V6_ROUTE_RESET_RETIREMENT_20260822.md`.

## 2026-08-22 — V6 RouteCoordinator concurrency and graph-authority amendment

- Goal/hypothesis: close the reset review blockers caused by multithreaded
  generation-check/state-commit TOCTOU, stale successful `SetRouteGraph`
  responses, stale structural rebuilds, and a missing empty transient runtime
  snapshot, while retaining the original single-abort/no-fake-terminal route
  retirement semantics.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `baa05f96061bb8084c71d258216fe0aa105568cc`, fixed-main ancestor
  `22d66470c4b903349b2467dc876490bbebfc0083`.
- Changes: shared `RLock` around route/action generation checks plus local
  commits; stale accepted action-handle cancellation; desired-GVG authority on
  reset/preemption; one serialized graph transaction with stale-response
  consumption and compensation; request/reset/structural/desired tokens for
  rebuild commit; fail-closed route preparation while graph state is
  incoherent; explicit empty transient-local runtime-edge snapshot after reset.
- Tests: deterministic `threading.Barrier` reset/callback interleaving; stale
  cognitive success -> GVG compensation; rebuild -> reset -> fresh goal ->
  late success; empty runtime snapshot identity; existing old-request,
  terminal, preemption, and fresh-goal coverage.
- Validation: source-first focused route/graph **73 passed**; complete route
  package **97 passed, 1 skipped** (`pxr` unavailable); pure ActivationGate and
  mode-contract **44 passed**; changed-file `py_compile` and `git diff --check`
  PASS; fresh isolated build PASS and `colcon test` **98 tests, 0 errors,
  0 failures, 1 skipped** at `/tmp/bionav_route_concurrency.j12692`.
- Verdict: **PASS (review blockers closed in code/build/unit only)**. No ROS
  graph, Isaac, Nav2, navigation, visual evidence, engineering campaign, or
  formal qualification was run.
- Remaining risk/next: live reset/action-server/DDS ordering, ActivationGate
  cancel-all interaction, StopGate command behavior, and fresh-goal restart
  remain pending an explicitly authorized live review.
- Handoff: `docs/handoff/V6_ROUTE_RESET_RETIREMENT_20260822.md`.

## 2026-08-22 — V6 RouteCoordinator second concurrency-review amendment

- Goal/hypothesis: close the remaining reset interleavings after a callback's
  current-generation check, keep desired-GVG compensation fail closed through
  Route Server confirmation, discard cognitive validation crossing reset, and
  remove shared graph-export filenames.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `5d4de361c5d1f7a9f7f6ea9e33b6ef36c04d18dd`; fixed-main ancestor
  `22d66470c4b903349b2467dc876490bbebfc0083`.
- Changes: dedicated terminal-order lock; generation-fenced async fallback;
  explicit Route Server reassert requirement; transaction reservation before
  export; request/graph/reset/reset-epoch cognitive validation token; unique
  immutable reset/desired/switch-bound graph export directories; submission
  revalidation and stale compensation for cognitive and structural graphs.
- Tests: deterministic old-navigation-terminal/reset ordering, old
  ComputeRoute rejection versus fresh request, cognitive validation crossing
  reset, blocked-export reassert gate, and distinct file/content isolation.
  Existing route retirement, stale graph/rebuild, empty transient edge,
  MotionBenchmark receipt/STOP, ResetStopGate, and ActivationGate tests remain
  passing.
- Validation: focused route/graph **78 passed**; complete route package **102
  passed, 1 skipped** (`pxr` unavailable); related regression set **42
  passed**; fresh isolated build PASS and `colcon test` **103 tests, 0 errors,
  0 failures, 1 skipped** at `/tmp/v6_route_second_build.Xl1fS6`; changed
  `py_compile` and `git diff --check` PASS.
- Verdict: **PASS (code/build/unit only)**. No ROS graph, Isaac, Nav2,
  navigation, visual evidence, engineering campaign, or formal qualification
  was run. Active-reset live closure remains pending.
- Handoff: `docs/handoff/V6_ROUTE_RESET_RETIREMENT_20260822.md`.

## 2026-08-22 — V6 reset seed receipt and final command ResetStopGate

- Goal: close the CLI/startup/Trigger reset-seed divergence and remove Isaac's
  second final `/cmd_vel` publisher without changing IMU or critic semantics.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `9671be168266cb639e04e4c1baf46e1d353c0720`; implementation `20bc5df`.
- Seed result: one effective CLI-or-scenario seed initializes startup and the
  reset parameter. Trigger responses carry actual seed/generation/pose/
  odometry/case/variant; ExperimentRunner, V6Formal, and MotionBenchmark retain
  full responses and STOP on mismatch. Tests cover `8601`, fallback, the
  `8601 -> 8602` change, and mismatch rejection.
- Authority result: Collision Monitor owns external Navigation `/cmd_vel`;
  same-process ResetStopGate alone publishes `/cmd_vel_sim` to the control
  graph, IdleBrake, and MotionAssist. HOLD begins before Timeline pause,
  continuously zeros on steady wall time, spans asynchronous recovery, and is
  released only by the current eligible generation after ActivationGate
  confirms managed nodes including Collision Monitor active. Startup release
  is same-process after startup transaction completion and never replays cache.
- Existing evidence retained, not rerun:
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_estimated_dynamic_smoke_20260821T150407Z`.
  It remains mixed-route **ENGINEERING FAIL** for IMU non-degradation despite
  successful/collision-free navigation; seed `8601 -> 0` and dual `/cmd_vel`
  ownership were separate debts addressed only at code-contract level here.
- Validation: source-first/no-cache focused pytest **124 passed**; py_compile,
  YAML, XML, installed imports, and diff check PASS; fresh isolated
  `robot_experiments` and sanitized `robot_bringup` builds PASS under `/tmp`.
  Isolated `colcon test` did not start because unbuilt workspace dependency
  hooks were absent; the same bringup tests passed source-first.
- Verdict: **PASS (implementation/build/unit only); AMBIGUOUS live closure**.
  No Isaac, ROS, Nav2, navigation, visual evidence, engineering campaign, or
  formal qualification was run. Reviewer must verify unique topic publishers,
  zero HOLD through active-goal reset/recovery, no stale command, no slip, and
  no collision; any violation is FAIL/STOP.
- Handoff: `docs/handoff/V6_RESET_SEED_STOP_GATE_20260822.md`.

## 2026-08-22 — ResetStopGate reviewer-blocker amendment

- Goal/hypothesis: prevent mapping/teleop/diagnostic resets from remaining
  permanently held, preserve generation-fenced ActivationGate release for
  Navigation, prevent MotionBenchmark from dispatching before the final gate
  and CollisionMonitor are ready, and parse complete reset receipt JSON.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `b28bdf56ccb7edf61dc176bfe3d8c49a0e3b03cc`; fixed Module3 main
  `22d66470c4b903349b2467dc876490bbebfc0083` remained an ancestor.
- Implementation: `ResetServiceBridge` has an explicit external-release
  contract. Successful non-navigation/diagnostic transactions auto-release
  only their captured generation; Navigation waits for ActivationGate;
  exceptions/timeouts remain HOLD. Startup uses this same finalizer, whose
  exclusivity now spans completion/release to close the early-Trigger race.
- MotionBenchmark consumes the existing ResetStopGate status authority and
  `/collision_monitor/get_state`; same receipt generation released,
  CollisionMonitor active, and stable post-reset clock/odom/TF are mandatory
  before nonzero command dispatch. Receipt extraction uses marker plus
  `JSONDecoder.raw_decode` and strict types.
- Validation actually run: source-first/no-cache focused pytest **141 passed**;
  changed-file `py_compile`, relevant YAML/XML parse, and diff check PASS;
  fresh isolated `robot_experiments` + `robot_bringup` build and installed
  imports PASS at `/tmp/bionav_reset_gate_build.OvPN5Z`.
- Verdict: **PASS (code/build/unit only)**; reviewer blockers are closed at
  code level. **Live behavior remains UNVERIFIED** because no Isaac, ROS graph,
  Nav2, navigation, visual evidence, engineering run, or formal qualification
  was launched.
- Remaining risk/next step: an authorized live reviewer must confirm mapping,
  Navigation recovery, R2C segment resets, MotionBenchmark delayed release,
  unique `/cmd_vel_sim` authority, and zero output throughout HOLD.
- Handoff: `docs/handoff/V6_RESET_SEED_STOP_GATE_20260822.md`.

## 2026-08-22 — ResetStopGate final-review freshness amendment

- Goal/hypothesis: keep non-navigation resets HOLD through every critical
  finalization action, and prevent MotionBenchmark nonzero dispatch from a
  one-shot or frozen clock/odom/TF snapshot or stale CollisionMonitor result.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `3e607ccbee920ac3b7df9c0a43f4a897cf60909e`; fixed Module3 main
  `22d66470c4b903349b2467dc876490bbebfc0083` remained an ancestor. Final commit
  is the commit containing this ledger row and is reported in the master
  handoff.
- Changes: reset event and initial-pose policy now precede gate eligibility;
  staged gate status publication cannot open the live state on exception.
  MotionBenchmark requires continuously fresh/coherent clock, odom, and TF,
  performs a final CollisionMonitor lifecycle query, fail-stops generation or
  gate changes, and records clock-stall zero output as `FAIL/STOP`.
- Config: `state_freshness_sec=0.25`, `stamp_coherence_sec=0.50`,
  `sim_clock_stall_timeout_sec=0.50`; motion settle remains `0.60 s` and config
  validation requires freshness shorter than settle.
- Validation actually run: source-first/no-cache focused pytest **51 passed**;
  sanitized broader contract and RouteCoordinator pytest **249 passed**;
  fresh sanitized `--packages-up-to robot_bringup` build **14 packages PASS**
  at `/tmp/bionav_gate_fresh.pXhLQh/install_up_to2`; installed imports,
  changed-file `py_compile`, YAML, XML, and diff checks PASS. An inherited stale
  overlay caused an initial collection failure and was removed before the
  passing run.
- Verdict: **PASS (code/build/unit only)**. No Isaac/ROS/Nav2/navigation run or
  formal qualification was performed; live closure remains **UNVERIFIED**.
- Handoff: `docs/handoff/V6_RESET_SEED_STOP_GATE_20260822.md`.

## 2026-08-21 — V6 critic revalidation TF/cursor blocker rework

- Goal: fix reviewer blockers where static-revalidated points used an old
  source-time TF and critic admission had no persistent ordering/identity
  cursor.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `6a64e4a67cd866759e07ca3db2dfed77efa8153b`.
- Changes: newly offered snapshots use the shared layer validator with a
  mutex-protected compound cursor and bound identity; accepted snapshots remain
  scoreable until their freshness/trust gates fail; TF lookup and point headers
  use the accepted validation/effective stamp. LIVE validation remains exactly
  source-time by contract. Physical safety and Costmap ownership are unchanged.
- Validation: fresh allowed Integration interface + Module3 fusion build PASS
  at `/tmp/bio_nav_module3_critic_rework_final.TIm4by`; focused package result 32
  tests / 0 errors / 0 failures / 0 skipped; plugin loader 1/1 PASS; `git diff
  --check` PASS. Score-level TF tests cover moving-frame static placement/yaw,
  LIVE source/effective time, expired/missing/bad/OOD/untrusted zero cost,
  repeat scoring, duplicate/backward validation, identity rejection, and
  reset/rebind.
- Verdict: **PASS (implementation/build/unit only)**. No active MPPI,
  ROS/Nav2/Isaac, navigation, engineering evidence, or qualification run was
  launched; live callback/TF timing remains unverified.
- Handoff:
  `docs/handoff/V6_CRITIC_STATIC_REVALIDATION_FRESHNESS_20260821.md`.

## 2026-08-21 — V6 critic callback admission/reset blocker repair

- Goal: align critic callback cursor advancement with the Costmap layer and
  recover one live critic instance after Integration advances `reset_epoch` and
  changes recurrent session.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `1e77edce4fec62ad0387f32c59e9925b0a898096`.
- Changes: obstacle callback now validates and advances the shared compound
  cursor immediately and preserves the last accepted snapshot on rejection;
  score only revalidates that snapshot against the current prior. Because MPPI
  exposes no per-critic reset hook, rebind requires a strictly higher epoch,
  changed session, fully matching fresh/trusted obstacle+prior pair, and
  unchanged map/tile/graph/model identity plus prior schema/route context before
  atomically replacing the old state. Old epoch replay, arbitrary identity
  drift, and untrusted pairs remain rejected.
- Validation: fresh allowed Integration interface + Module3 fusion build PASS
  at `/tmp/bio_nav_module3_critic_admission.Bzf9rD`; focused equal-cost result
  33/33 plus plugin loader 1/1, total 34 focused gtest cases (corrected from the
  earlier erroneous value of 35); `git diff --check` PASS. Callback tests cover
  newer-obstacle/lower-prior interleaving,
  duplicate/backward/source regression, first binding, same-instance trusted
  reset, old replay, changed map/route, and untrusted reset. Existing
  validation-time TF, LIVE, repeat-score, moving/rotating, and fail-open cases
  remain covered.
- Verdict: **PASS (implementation/build/unit only)**. No active MPPI,
  ROS/Nav2/Isaac, navigation, engineering evidence, or qualification run was
  launched. Live stream ordering across reset remains unverified; obstacle-first
  reset delivery safely waits for a later obstacle publication after the new
  prior is present.
- Handoff:
  `docs/handoff/V6_CRITIC_STATIC_REVALIDATION_FRESHNESS_20260821.md`.

## 2026-08-21 — V6 critic V3.10 component-trust contract repair

- Goal/hypothesis: allow a trusted, matching static-revalidated obstacle pair
  from the fixed Integration V3.10 runtime to add nonnegative MPPI cost while
  keeping its intentionally diagnostic context and periodic local direction at
  zero influence.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `1d977d7c822ef81d6139d082cdade373769bdb35`.
- Changes: split basic pair admission from context and direction component
  gates; `context_trusted=false` suppresses novelty/uncertainty without making
  the pair unhealthy; only the existing strict direction validator enables
  direction. Basic accepted pairs now support callback admission and monotonic
  reset rebind. Applied status names `obstacle_applied` and every suppressed
  component. No Integration/Module2 changes and no `GoalPlanningPrior`
  subscription were added.
- Validation: fresh allowed Integration interface + Module3 fusion build PASS
  at `/tmp/bio_nav_module3_critic_component.gIA7MD`; focused equal-cost 35/35
  and plugin loader 1/1 PASS, total 36 focused gtest cases; `git diff --check`
  PASS. Production-shaped V3.10 tests cover context/direction suppression,
  synthetic trusted-context enablement, stale/untrusted/identity mismatch,
  both callback orders, monotonic reset rebind, and old-epoch replay rejection.
- Verdict: **PASS (implementation/build/unit only)**. No ROS graph, active
  MPPI, Nav2, Isaac, navigation, visual evidence, engineering campaign, or
  formal qualification was run. Goal-conditioned direction is still not
  connected to the critic; live status and active-MPPI influence remain
  unverified.
- Handoff:
  `docs/handoff/V6_CRITIC_STATIC_REVALIDATION_FRESHNESS_20260821.md`.

## 2026-08-21 — V6 critic real-timestamp obstacle-independence amendment

- Goal/hypothesis: align the critic with the real Integration V3.10 contract:
  static depth refresh republishes the cached obstacle with a fresh validation
  timeline but preserves the original source sequence/stamp, and does not
  republish a same-sequence fresh `PlanningPrior`.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `fea521b5b849c381d29191659ea97b291e7a0aeb`.
- Changes: trusted obstacle admission, cursor, freshness, identity, and
  validation-time TF are independent of planning-prior availability. A prior
  now gates only context/novelty/uncertainty/local-direction components and is
  suppressed when missing, stale, untrusted, OOD, identity/session
  incompatible, or sequence mismatched. Trusted higher-epoch/new-session
  obstacle reset rebind no longer depends on an accepted prior. Status names
  `obstacle_applied` and every prior component outcome. Static test helpers no
  longer retime priors, and the moving validation-TF test uses no prior.
- Validation: fresh allowed Integration interface + Module3 fusion build PASS
  at `/tmp/bio_nav_module3_critic_independent.pbOLuY`; focused
  `test_equal_cost_search` 35/35 and plugin loader 1/1 PASS; `git diff --check`
  PASS. Tests cover real old-prior/static-refresh timestamps, fresh
  different-sequence and missing priors, legal fresh pairs, obstacle fail-open
  gates, ordinary LIVE, validation-time TF, callback interleaving, and
  no-prior monotonic reset/replay rejection.
- Verdict: **PASS (implementation/build/unit only)**. No ROS graph, active
  MPPI, Nav2, Isaac, navigation, visual evidence, engineering campaign, or
  formal qualification was run. Live callback/status behavior and active-MPPI
  influence remain unverified.
- Handoff:
  `docs/handoff/V6_CRITIC_STATIC_REVALIDATION_FRESHNESS_20260821.md`.

## 2026-08-21 — V6 official stationary fresh runtime

- Goal: record the bounded fresh stationary runtime result without promoting
  it to current-HEAD critic, moving-navigation, or formal-qualification proof.
- Branch/worktree/base: `cognitive-navigation`; permitted Module3 worktree;
  documentation preflight HEAD `25b97a12ff8c77f0e882cdce259209a7eaeb7374`.
- Runtime provenance: immutable Module3 snapshot
  `1d977d7c822ef81d6139d082cdade373769bdb35`, not the current authoring HEAD;
  evidence root
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_stationary_static_authority_fresh_20260821T140713Z`.
- Configuration/result: fresh 14-package Module3 build and asset check PASS;
  `165.502708967 s` post-reset observation; unique reset/epoch rollover and
  depth conservation had no regressions or violations; 518 trusted static
  obstacle messages; global/local Costmap applied counts 284/1,431 with
  raised/masked cells. Transient continuity gaps and rejections recovered.
- Stationary safety/TF: nonzero command count 0, ground-truth displacement
  `0.0 m`, collision count 0, maximum odometry displacement
  `3.924656304988303e-9 m`; `map -> odom` 10 Hz and `odom -> base_link` 60 Hz
  with zero stamp regressions; the GT firewall had no navigation subscriber.
- Verdict: **ENGINEERING PASS (narrow stationary only)**;
  **NOT FORMAL QUALIFICATION**.
- Limits: zero goals/runners; critic applied count 0, so no active-critic claim;
  `cognitive_graph_mode=gvg` was producer-only, with no graph application or
  `PRIMARY`; the older runtime snapshot does not validate the current critic.
  Two Integration helper children warned/exited 1 during commanded teardown
  after the observation window.
- Next: fresh current-HEAD, at-most-180-second, low-speed single-goal active-M3
  pilot; any moving authority gap measured in seconds is **FAIL / STOP**.
- Handoff: `docs/handoff/V6_OFFICIAL_STATIONARY_FRESH_20260821.md`.

## 2026-08-21 — V6 critic route-context rebind and applied-truth repair

- Goal: prevent route-ambiguous same-instance reset rebind and prevent admitted
  but zero-delta critic cycles from being reported as online-applied.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `3dc2830c1da5b5f441191217220bc120058bd4b2`.
- Changes: higher-epoch rebind now waits for a health/trust-compatible
  `PlanningPrior` carrying the exact new session/reset and unchanged planning
  schema, local schema, route graph, physical graph/revision, topology, and
  stable map/tile/graph/model identity. Stale or sequence-mismatched prior data
  can prove route identity but remains suppressed from scoring. Actual finite
  positive cost-vector deltas determine overall/component applied flags;
  empty/zero/far contributions report `zero_cost_delta`. Distinct callback
  rejection status preserves rejected offer identity while old accepted scoring
  names its own source. The causal evaluator requires the new positive-delta
  marker before reporting `online_applied`.
- Validation: fresh allowed Integration interfaces + Module3 fusion build PASS
  (2 packages) at `ros2_ws/build_critic_fix_kOEv7h` and
  `ros2_ws/install_critic_fix_8raZSf`; focused equal-cost and plugin-loader
  CTests PASS (2/2 executables); source-first causal/localization/mode pytest
  **52 passed**; `git diff --check` PASS after docs.
- Verdict: **PASS (implementation/build/unit only)**. No ROS graph, active
  MPPI, Nav2, Isaac, navigation, visual evidence, engineering campaign, or
  formal qualification was run. `GoalPlanningPrior` remains unconnected; the
  stationary handoff and its bounded conclusion are unchanged.
- Handoff:
  `docs/handoff/V6_CRITIC_STATIC_REVALIDATION_FRESHNESS_20260821.md`.

## 2026-08-22 — V6 IMU regime evidence-contract amendment

- Goal: prevent partial, misidentified, malformed, or misaligned diagnostic
  evidence from selecting a new global IMU yaw scale.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `7b6b01f48fafdeb9985463e0cda6f49ababb2e71`.
- Changes: explicit diagnostic-only 10 s stationary reset/zero report; strict
  nine-primitive/seed/generation/receipt/segment contract; exact MCAP types and
  finite/strict-stamp checks; provenance-bearing goal input; common-overlap
  union-grid integration; four-attribute phase gate; unique `flat20_start` and
  locked Grid/dynamics-off runner/provenance; required ROS bag/runtime deps;
  AST loop-order test. `yaw_scale=0.9294` and RF2O off are unchanged.
- Validation: source-first/no-cache **110 passed**; pycompile, YAML/loader,
  `bash -n`, source-first post-build import, and `git diff --check` PASS; fresh
  isolated `robot_experiments` build PASS at
  `/tmp/v6_imu_evidence_build.yizdjX` with install
  `/tmp/v6_imu_evidence_install.l2zy4u` and log
  `/tmp/v6_imu_evidence_log.POQ9cz`.
- Verdict: **PASS (code/build/unit only)**. No Isaac/ROS/MCAP/motion/goal
  runtime or formal qualification was run; live regime decision remains
  pending.
- Handoff: `docs/handoff/V6_IMU_REGIME_EVIDENCE_CONTRACT_20260822.md`.

## 2026-08-22 — V6 deferred structural rebuild after route terminal

- Goal: retain the latest persistent structural-map candidate when a route
  terminal races an existing Route Server graph transaction, without duplicate
  terminal output or `SetRouteGraph` request storms.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `deb275252190a6eacbc9f08a0c9ad76773a172f4`.
- Changes: one candidate/request/graph/reset-fenced structural intent; reset
  clearing; active-goal deferral; completion/timeout/late-callback wakeup through
  the existing steady reconciliation and bounded retry path; stale candidates
  cannot commit map/GVG state.
- Validation: focused route/graph **101 passed**; complete route package **125
  passed, 1 skipped** (`pxr` unavailable); associated reset/gate/benchmark/IMU/
  localization/EKF **87 passed**; fresh isolated build and `colcon test` at
  `/tmp/bio_nav_route_deferred.jMe0m0` **126 tests, 0 failures, 1 skipped**;
  `py_compile` and `git diff --check` PASS.
- Verdict: **PASS (code/build/unit only)**. No ROS/Isaac/Nav2/navigation,
  evidence campaign, or formal qualification was run.
- Remaining risk/next: permanently unresolved timed-out futures can accumulate;
  active-reset plus live structural-change behavior remains for the planned
  runtime review.
- Handoff: `docs/handoff/V6_ROUTE_RESET_RETIREMENT_20260822.md`.

## 2026-08-22 — V6 IMU single-attempt and stream-gap authority closure

- Goal: prevent failed/retried goal commands and long IMU/GT dropouts from
  being merged or interpolated into a successful yaw-scale candidate.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `1d717a87b0079d540678e5f48156e801ece3b570`.
- Changes: require exactly one fresh successful route terminal; reject false or
  multiple terminals and abnormal pre-request/post-terminal motion; bind an
  optional single `PoseStamped` route request to evaluator metadata; record
  terminal/request/window provenance; apply the diagnostic `0.25 s` maximum
  gap and coverage gate to raw/GT and optional corrected IMU before common-grid
  interpolation.
- Validation: source-first focused **43 passed**; fresh isolated package build
  PASS at `/tmp/v6_imu_goal_build.TYHcAO`; installed focused **43 passed**;
  installed import/entrypoint, `py_compile`, and `git diff --check` PASS.
- Verdict: **PASS (code/build/unit only)**. No Isaac, ROS graph, goal MCAP,
  navigation, calibration selection, or formal qualification was run. Live
  closure remains pending; frozen `yaw_scale=0.9294` and RF2O-off unchanged.
- Handoff:
  `docs/handoff/V6_IMU_REGIME_EVIDENCE_CONTRACT_20260822.md`.

## 2026-08-22 — V6 RouteCoordinator HOLD-crossing input fences

- Goal: prevent a runtime, structural, prior, cognitive, region, costmap, or
  timer callback admitted before reset HOLD from committing or publishing old-
  epoch state after HOLD retirement or same-generation release.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `600981e28616709e1937046e90e715f1a36f4d21`.
- Changes: immutable reset/route/graph input token; state-lock commit
  revalidation; output-lock then state-lock final publication/dispatch check;
  atomic runtime mutation/snapshot and locked runtime cost view; private
  structural-monitor observation with generation-fenced commit; HOLD rejection
  in deferred/rebuild/cognitive/prior/region/context paths. Existing reset
  completion/release, GVG reassertion, deferred coalescing, and exactly-once
  route terminal semantics remain.
- Deterministic tests: runtime edge 77 crossing repeated for 40 rounds;
  structural pending/rebuild, prior latest/pending, cognitive immature status,
  runtime tick, and region tick crossings. Result: **6 passed**.
- Validation: complete source-first route package **138 passed, 1 skipped**
  (`pxr` unavailable); risk-related gate/mode/MotionBenchmark/reset/IMU/
  localization/EKF set **174 passed**; changed-file `py_compile` and
  `git diff --check` PASS; fresh isolated build PASS and `colcon test-result`
  **139 tests, 0 errors, 0 failures, 1 skipped** at
  `/tmp/v6_route_hold_fence_build.bZ2EMi`.
- Verdict: **PASS (code/build/unit only)**. No ROS/Isaac/Nav2/navigation,
  visual evidence, engineering campaign, or formal qualification was run.
- Remaining risk/next: run the planned fresh active-reset live review to verify
  executor/DDS ordering, old-request silence and zero command through HOLD, and
  successful fresh-goal recovery.
- Handoff: `docs/handoff/V6_ROUTE_RESET_RETIREMENT_20260822.md`.

## 2026-08-22 — V6 corrected IMU header-stamp authority

- Goal: prevent corrected IMU bag-record time from silently replacing the
  sensor header time used to derive the goal yaw-scale candidate.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `6e3deb2c777977e144e0c001bbaa2a504cbeafaf`.
- Changes: raw IMU, corrected IMU, and GT now require positive message header
  stamps for goal-MCAP evidence; corrected IMU no longer falls back to bag
  time. Header order, attempt-window coverage, maximum gap, common grid, and
  integration all use the header domain. No record/header skew threshold was
  introduced.
- Adversarial coverage: corrected zero/duplicate/backward/shifted-stale
  headers and a valid `100 s` shifted, jittered bag-time case; existing
  goal-attempt, reset-boundary, duration, terminal, gap, and installed-share
  regressions remain passing.
- Validation: source-first focused **52 passed**; fresh build/install PASS at
  `/tmp/v6_imu_header_stamp.hHu18p`; fresh-installed focused **52 passed**;
  installed import/entrypoint, changed-file `py_compile`, and
  `git diff --check` PASS.
- Verdict: **PASS (code/build/unit only)**. No Isaac, ROS graph, MCAP capture,
  navigation, calibration choice, or formal qualification was run. Live
  regime closure remains pending; `yaw_scale=0.9294` and RF2O-off are
  unchanged.
- Handoff:
  `docs/handoff/V6_IMU_REGIME_EVIDENCE_CONTRACT_20260822.md`.

## 2026-08-22 — V6 IMU MCAP file-order authority

- Goal: prevent rosbag received-time sorting or jitter from reordering yaw
  headers and hiding file/publish-order duplicate or backward stamps.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `4aa311783582c4df75d48217b761090fec2326ca`.
- Changes: both MCAP readers require confirmed `ReadOrderSortBy.File`; missing
  support is structured `AMBIGUOUS`. Raw/corrected/GT retain file order and use
  header time for order/gap/window/integration. Reset/command/terminal/request
  and other event streams use received time and are sorted only after
  collection. Result provenance exposes both domains.
- Adversarial coverage: real rosbag2 MCAP with adjacent received-time inversion
  and increasing file/header order passes; real file-order raw duplicate and
  corrected backward headers fail. Existing single-attempt, reset/request
  boundary, terminal, and gap regressions remain passing.
- Validation: source-first **56 passed**; fresh build/install PASS at
  `/tmp/v6_imu_file_order_build.qBwj0i` and
  `/tmp/v6_imu_file_order_install.ukM3HM`, log
  `/tmp/v6_imu_file_order_log.3K0ILK`; fresh-installed **56 passed**; installed
  import/entrypoint, changed-file `py_compile`, and `git diff --check` PASS.
- Verdict: **PASS (code/build/unit only)**. No Isaac, ROS graph, navigation,
  live MCAP, calibration choice, or formal qualification was run. Live regime
  closure remains pending; `yaw_scale=0.9294` and RF2O-off are unchanged.
- Handoff:
  `docs/handoff/V6_IMU_REGIME_EVIDENCE_CONTRACT_20260822.md`.

## 2026-08-22 — V6 reset-completion graph-retry authority

- Goal: preserve the current completion-owned GVG reassert across an old
  cognitive/structural graph transaction, same-key retry, timeout, or late
  callback, while preventing old graph callbacks from emitting status into a
  held or released fresh reset epoch.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `819e519484a1b837ca2817465edabe6cda6d9b41`.
- Changes: retry context now binds completion token, route-input generation,
  coordinator reset generation, and desired retry key; current completion
  authority wins same-key merges and newer reset generations replace it;
  successful held reassert retains refreshed authority for one late-success
  compensation; failure/stale-success output uses the same final route-epoch
  fence, with explicit current reset completion as the only HOLD exception.
- Deterministic coverage: cognitive/structural old success and rejection,
  timeout plus late success, existing same-key retry, newer reset invalidation,
  duplicate no-storm, old fallback rejection/exception during HOLD, current
  completion failure positive, stale success after release, and 100 repeated
  runtime-edge HOLD crossings.
- Validation: focused graph adapter **76 passed**; complete route package **159
  passed, 1 skipped** (`pxr` unavailable); associated reset/gate/mode/
  benchmark/reset-receipt/IMU/localization/EKF **203 passed**; fresh isolated
  build/test PASS at `/tmp/v6_route_retry_build.4tnZkN` with install
  `/tmp/v6_route_retry_install.tTUkd7` and log
  `/tmp/v6_route_retry_log.8PfwFz`; colcon result **160 tests, 0 errors, 0
  failures, 1 skipped**; changed-file `py_compile` and `git diff --check` PASS.
- Verdict: **PASS (code/build/unit only)**. No ROS/Isaac/Nav2/navigation,
  visual evidence, engineering campaign, or formal qualification was run.
- Remaining risk/next: run the planned fresh active-reset live review to verify
  actual executor/DDS ordering, exactly-once GVG compensation, zero old output
  and command during HOLD, and fresh-goal recovery.
- Handoff: `docs/handoff/V6_ROUTE_RESET_RETIREMENT_20260822.md`.

## 2026-08-22 — V6 flat20 IMU LiDAR-feature readiness closure

- Goal: remove the identified flat20 LiDAR-readiness blocker without changing
  the frozen IMU scale, RF2O policy, CollisionMonitor, route/reset/critic, or
  command authority.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `aa9d2d73d2f2dc843f29a086fbfb71db5b06f4d2`.
- Attempt 1:
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_imu_regime_session_a_20260821T210119Z`;
  **STOP / NO ENGINEERING CAPTURE / NOT FORMAL**, missing archived external
  `jackal_original.usd`, no ROS/stationary/primitive/MCAP. Attempt 2:
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_imu_regime_session_a_attempt2_20260821T210842Z`;
  same verdict; asset/scene/build/domain/IMU/odom/GT/authority checks passed but
  all four LiDAR readiness streams remained zero for the final 10 s, so no
  benchmark/MCAP/analyzer ran.
- Root cause/change: the runner had disabled obstacle authoring on a bare Grid
  USD. It now authors the installed `v6_calibration_grid_features.yaml`: seed
  20260821, four walls plus three asymmetric LiDAR-height features, all seven
  stationary, zero moving objects. Trace and analyzer provenance bind the same
  installed resource and fail closed on missing/mismatch.
- Added read-only `v6_imu_lidar_preflight`: raw/filtered cloud plus mapping/
  safety scan each need two live messages, strict stamps, age below 0.4 s, and
  finite returns before seed 8609. It publishes no command and does not bypass
  CollisionMonitor.
- Validation: related source-first/no-cache **173 passed**; new-contract subset
  **72 passed**; fresh isolated package build/install PASS at
  `/tmp/v6_imu_lidar_build.0BdhgV` and
  `/tmp/v6_imu_lidar_install.W9631i`, log
  `/tmp/v6_imu_lidar_log.n99ktJ`; installed resource/entrypoint checks,
  `py_compile`, YAML, runner `bash -n`, and `git diff --check` PASS.
- Verdict: **PASS (code/build/unit only)**. Attempt 3 is **PENDING**; no Isaac,
  ROS graph, live readiness, stationary, primitive, MCAP, navigation,
  calibration decision, or formal qualification was run. `yaw_scale=0.9294`
  and RF2O-off remain unchanged.
- Handoff:
  `docs/handoff/V6_IMU_REGIME_EVIDENCE_CONTRACT_20260822.md`.

## 2026-08-22 — V6 RouteCoordinator cross-topic late-join reset merge

- Goal: make volatile Empty reset events and transient-local gate status
  generation-idempotent under either callback order, including missed Empty or
  missed completion, without weakening runtime missing-HOLD fail-closed rules.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `aab15690c64017775ec025acd51fe03f0cf5c210`.
- Triggering evidence:
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_active_reset_live_attempt2_20260821T212047Z/`;
  retained as **ENGINEERING FAIL / STOP / NOT FORMAL**. Attempt 3 remains
  pending.
- Changes: pristine event-first uses a bounded 0.5 s unbound completion hint;
  strict HOLD/completion/release can bind it to one physical reset without a
  second request/epoch advance. Same-generation `reset_complete` now owns the
  empty runtime/GVG completion when Empty is missed. Release opens only after
  completion reconciliation. Event-only legacy operation resumes after the
  grace; non-pristine missing-HOLD and invalid transitions stay held.
- Deterministic coverage: event/completion/release,
  completion/event/release, event/HOLD/completion/release, HOLD/completion with
  no event, HOLD/event/completion, event/release with completion missed,
  event-only timeout, non-pristine negative orders, 500 concurrent
  event/completion rounds, and retained active-reset/output/graph retry tests.
- Validation: focused **75 passed**; full source-first route package **175
  passed, 1 skipped** (`pxr` unavailable); fresh isolated build/test PASS at
  `/tmp/v6_route_cross_topic_build.OkpE4E` with install
  `/tmp/v6_route_cross_topic_install.OXFzsb` and log
  `/tmp/v6_route_cross_topic_log.GLwmz6`; colcon result **176 tests, 0 errors,
  0 failures, 1 skipped**; changed-file `py_compile` and `git diff --check`
  PASS. One earlier build invocation had global `--log-base` in the wrong CLI
  position and exited during argument parsing before any build; the corrected
  fresh invocation is the cited result.
- Verdict: **PASS (code/build/unit only)**. No ROS/Isaac/Nav2/navigation,
  visual evidence, engineering campaign, or formal qualification was run.
- Handoff: `docs/handoff/V6_ROUTE_RESET_RETIREMENT_20260822.md`.

## 2026-08-22 — V6 startup reset hint authority/deadline safety

- Goal: close the two late-join reviewer HIGH blockers without changing IMU,
  ResetStopGate producer, ActivationGate, critic, command, or event contracts.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `bccbd3e00cf32686434b4f0f6dc21f7cc19b9c74`.
- Changes: malformed/backward/conflicting status atomically invalidates the
  startup Empty hint/timer and holds the barrier; `initialized`/`closed`
  establishes strict status authority and disables legacy auto-open while
  retaining a same-physical-reset HOLD binding in either callback order;
  bind/timer/status deadline decisions are steady-time and lock-linearized;
  expired hints cannot bind a new generation, while status-free event-only
  legacy fallback remains available.
- Deterministic coverage: malformed/conflicting status then timer, both normal
  baseline reasons and callback orders, expiry before timer/HOLD, late baseline
  after expiry, both timer/status orders, new reset after expiry, and 500
  concurrent baseline/Empty rounds with one request/epoch/completion.
- Validation: focused **86 passed**; full route package **186 passed, 1
  skipped** (`pxr` unavailable); associated reset/gate/mode/benchmark/IMU/
  localization/EKF **232 passed**; fresh isolated build/test PASS at
  `/tmp/v6_route_hint_build.NVrz8b`, install
  `/tmp/v6_route_hint_install.bTHSo4`, log
  `/tmp/v6_route_hint_log.FK8WjM`; colcon result **187 tests, 0 errors, 0
  failures, 1 skipped**; `py_compile` and `git diff --check` PASS. One initial
  associated-test collection used a stale repository overlay and failed on
  missing `CanonicalRoute`; the source-first rerun used the allowed Integration
  worktree generated interface and passed.
- Verdict: **PASS (code/build/unit only)**. Attempt 3 remains **PENDING**; no
  ROS/Isaac/Nav2/navigation, visual evidence, engineering campaign, or formal
  qualification was run.
- Handoff: `docs/handoff/V6_ROUTE_RESET_RETIREMENT_20260822.md`.

## 2026-08-22 — V6 IMU schema-2 capture-boundary amendment

- Goal: close the schema-2 reviewer blockers in MotionBenchmark evidence and
  the offline analyzer without changing motion playback, thresholds,
  `yaw_scale=0.9294`, RF2O, route/reset/gate, or critic behavior.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `503cceb15a6517f3a0436a09817b594f009dfcf8`.
- Changes: stationary intent count is limited to the main 10 s schedule and
  final settle has an independent receipt; all command-chain schedules require
  finite ordered start/end/internal coverage; stationary, reset HOLD, and final
  settle require full-window all-zero evidence. MCAP `/clock` is strict in file
  order; command duplicates are explicit while backward stamps fail. Schema-1
  remains readable but is capture-ambiguous and can never authorize scale.
  Malformed phase simulation time becomes structured evidence.
- Deterministic coverage: schema-2 positive report/MCAP binding; moving start
  and end dropout; HOLD dropout; nonzero injection at each of four stationary
  command stages; schema-1 retained metrics/no-auth; clock and command order;
  malformed/non-finite phase time; short final-settle receipt.
- Validation: source-first focused **114 passed**; fresh-installed focused
  **114 passed**; isolated build/install PASS at
  `/tmp/v6_imu_schema2_final_build.d34e4i` and
  `/tmp/v6_imu_schema2_final_install.UciJmK`, log
  `/tmp/v6_imu_schema2_final_log.kNJ411`; installed analyzer import/help,
  `py_compile`, and `git diff --check` PASS. Wider package result **546 passed,
  1 unrelated path-sensitive frozen-reference failure**.
- Verdict: **PASS (code/build/unit only)**. Existing Attempt 3 remains
  FAIL/AMBIGUOUS/no-auth with metrics retained; it was not rerun or modified.
  Attempt 4 remains **PENDING**. No ROS/Isaac/navigation/evidence/formal run.
- Handoff:
  `docs/handoff/V6_IMU_REGIME_EVIDENCE_CONTRACT_20260822.md`.

## 2026-08-22 — V6 active-reset exactly-once probe

- Goal: replace Attempt4's split reviewer timing with one continuously
  observable, exactly-once, fail-stop active-reset runner.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `89c052fb078e96d97af891c8fc8410ec36c2d753`.
- Attempt4 input:
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_active_reset_live_attempt4_20260821T224036Z`;
  retained as **ENGINEERING FAIL / STOP / NOT FORMAL** because the old route
  terminated before Trigger and the external harness missed the earlier active
  boundary. Attempt5 remains pending and must use a fresh episode.
- Changes: new `active_reset_probe` entry point with a pure monotonic state
  machine and ROS adapter; reliable/volatile long-lived route publisher;
  exactly-once old/fresh goals and Trigger; strict active, receipt, generation-2
  gate, old abort/silence, reset landing drift, fresh success/GT error, collision,
  and four-command-chain postzero checks; atomically refreshed JSON evidence.
- Deterministic coverage: subscriber/endpoints wait, retained startup exclusion,
  exactly-once actions, active timeout, pre-reset terminal, 0.5 s reset delay,
  receipt mismatch, gate order/HOLD leakage, teleport versus drift, old silence,
  fresh route/success/failure, and postzero pass/fail.
- Validation: focused source-first **52 passed** (14 probe-state plus 38
  retained package-contract tests); changed-file flake8, `py_compile`, and
  `git diff --check` PASS. Fresh ordinary isolated build/install PASS at
  `/tmp/v6_active_reset_probe_commit_build.vDaiVx`, install
  `/tmp/v6_active_reset_probe_commit_install.lmarja`, log
  `/tmp/v6_active_reset_probe_commit_log.FD2IDl`; installed help and console entry
  point PASS. An initial `--symlink-install` attempt failed before packaging
  because existing external-resource paths escape a `/tmp` build base; the
  corrected non-symlink isolated build is the cited result.
- Verdict: **PASS (code/build/unit only)**. No ROS/Isaac/Nav2,
  navigation, reset, evidence collection, engineering campaign, or formal
  qualification was run.
- Handoff: `docs/handoff/V6_ACTIVE_RESET_PROBE_20260822.md`.

## 2026-08-22 — V6 active-reset probe final-contract amendment

- Goal: close every independent review blocker on the `24136355` probe before
  Attempt5, without changing route/reset/IMU/control behavior.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `24136355b632b3b8e9abc30c790ef939e4c9438b`. Final change is the
  ledger-containing commit.
- Changed: `active_reset_probe.py`, its focused tests, this ledger, and
  `V6_ACTIVE_RESET_PROBE_20260822.md` only.
- Data contract: the estimated stream is official `/odom`; GT and odometry
  require normalized `map`/`odom` frames respectively. Position, quaternion,
  source stamp, callback time, and all six Twist components are finite or
  STOP. GT and odom reset landings remain map `(0.45,-5.35)` and odom `(0,0)`.
- Reset/route contract: the retained baseline is exactly generation-1
  released with no hidden higher generation. Generation 2 is exactly
  `hold -> reset_complete -> released:*`; duplicates/extras/regression/
  advance/conflict STOP through end-of-run monitoring. From actual Trigger
  dispatch or first HOLD, any old canonical/progress/lookahead/Navigate intent
  immediately STOPs and records receive time/type/request identity.
- Dispatch/lifecycle: the 0.5 s delay uses a fresh steady timestamp after the
  synchronous topology query and immediately before `call_async`, so a slow
  query dispatches nothing. The machine ends at `PROVISIONAL_COMPLETE`; only
  successful node destruction, ROS shutdown, and atomic file plus directory
  fsync produce zero exit and final `PASS_REQUIRES_BAG`. Persistence/teardown
  errors remain nonzero STOP and attempt a distinct emergency STOP receipt.
- Deterministic coverage: NaN/Inf pose/Twist/time, wrong frame/topic constants,
  immediate old outputs, hidden/duplicate/backward/extra gate generations,
  slow dispatch/no call, provisional-to-final flow, write failure, and destroy/
  shutdown failures, plus all retained active-reset and package contracts.
- Validation: source-first **76 passed** (38 probe plus 38 retained package
  contract); package-configured flake8, `py_compile`, and `git diff --check`
  PASS. Fresh isolated build/install PASS at
  `/tmp/v6_active_reset_final2_build.0JWgYE`, install
  `/tmp/v6_active_reset_final2_install.1vIoZW`, log
  `/tmp/v6_active_reset_final2_log.kF2OrQ`; installed-module path assertion and
  the same **76 passed**. Installed `ros2 run ... --help` and direct console
  entry point PASS.
- Verdict: **PASS (code/build/unit only)**. Attempt5 remains **PENDING**. No ROS
  graph, Isaac, Nav2, navigation, reset, bag/evidence episode, engineering
  campaign, or formal qualification was run.
- Handoff: `docs/handoff/V6_ACTIVE_RESET_PROBE_20260822.md`.

## 2026-08-22 — V6 IMU schema-1 retrospective continuity and schema-2 HOLD tail

- Goal/hypothesis: retain metrics from immutable schema-1 Attempt 3 when only
  the newer strict command-boundary proof is unavailable, while closing the
  schema-2 reset-HOLD gap through the first schedule start.
- Worktree/branch/start: permitted Module3 `cognitive-navigation` worktree,
  `cc7debb38ad5fcf6676540ff1a828a9aa16f0f6e`.
- Changed: `imu_regime_analysis.py`, its focused tests, this ledger, and
  `V6_IMU_REGIME_EVIDENCE_CONTRACT_20260822.md`. No scale/config, reset/route,
  MotionBenchmark playback, Integration, or Module2 file changed.
- Result: schema-1 boundary/gap insufficiency is explicit capture ambiguity
  and never authorizes scale; schema-2 four-stage HOLD-tail leak/dropout is
  FAIL through an end-exclusive first-schedule boundary.
- Attempt 3 retrospective evidence:
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_imu_regime_session_a_attempt3_20260821T220048Z/analysis/schema1_retro_hold_gap_20260822/imu_regime_analysis.json`.
  Result **FAIL / NOT FORMAL**: 12 windows, stored k-star/segment identity
  preserved, performance FAIL, capture AMBIGUOUS, scale authorization false.
- Validation: source-first **102 passed**; fresh isolated package build PASS at
  `ros2_ws/build_imu_retro_hold.3bjyYU` / install
  `ros2_ws/install_imu_retro_hold.hS83p9` / log
  `ros2_ws/log_imu_retro_hold.Qtjinp`; fresh-installed analyzer `--help` and
  **102 passed**. The first `/tmp` build-base attempt failed on the package's
  pre-existing workspace-relative symlink-data layout; the correctly located
  isolated build passed.
- Verdict: **PASS (code/build/unit plus retrospective offline analysis)**.
  Attempt 4 prospective schema-2 live capture remains **PENDING**;
  `yaw_scale=0.9294` and RF2O-off are unchanged.

## 2026-08-22 — V6 active-reset probe evidence-contract amendment

- Goal: close the independent cc7 probe-review blockers before any Attempt5
  live episode, without changing route/reset/IMU/control implementations.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `8825e606245df83d8bd755e84dff0730c9d11aa1`. Final change is the
  ledger-containing commit.
- Changed: `active_reset_probe.py`, its focused tests, this ledger, and
  `V6_ACTIVE_RESET_PROBE_20260822.md` only.
- Contract: exact endpoint-info identities/GIDs/counts at four invariant graph
  checkpoints; exact newer request and epoch/status/reason/edge terminals;
  0.25 s edge/gap and two-sample minimum coverage for HOLD command/collision,
  stable GT/odom, and all four 1.0 s postzero streams; map and odom landing
  errors are evaluated in their respective frames. All support-equivalent
  routing was removed.
- Claim boundary: callback times are explicitly receive order, not cross-topic
  source order. An in-process success reports
  `PROVISIONAL_PASS_REQUIRES_BAG_ORDER` with `engineering_pass=false`; only a
  finalized-bag ordering analysis may promote Attempt5.
- Failure behavior: endpoint, goal publish, Trigger dispatch, future callback,
  JSON persistence, spin and teardown exceptions produce STOP; side-effect
  attempt counters prevent exception-driven retries.
- Validation: source-first focused **58 passed** (20 probe plus 38 retained
  package-contract tests); package-configured flake8, `py_compile`, and
  `git diff --check` PASS. Fresh isolated build/install PASS at
  `/tmp/v6_active_reset_probe_fix_build.D38WEx`, install
  `/tmp/v6_active_reset_probe_fix_install.51b4zR`, log
  `/tmp/v6_active_reset_probe_fix_log.ozbzdC`. Installed-module path assertion,
  the same **58 passed**, `ros2 run ... --help`, and the direct installed console
  entry point PASS.
- Verdict: **PASS (code/build/unit only)**. Attempt5 remains **PENDING**. No ROS
  graph, Isaac, Nav2, navigation, reset, bag/evidence episode, engineering
  campaign, or formal qualification was run.
- Handoff: `docs/handoff/V6_ACTIVE_RESET_PROBE_20260822.md`.

## 2026-08-22 — Attempt5 runtime-closure amendment

- Goal: close three Attempt5 blockers before a fresh Attempt6:
  executor-independent ResetStopGate HOLD coverage, reset-GVG READY liveness,
  and the active-reset probe's true received-HOLD output boundary.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `5dc3b91e9922a12b45e63b135342350e5e847a33`. Fixed Module3 main remained
  `22d66470c4b903349b2467dc876490bbebfc0083`.
- Immutable input:
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_active_reset_live_attempt5_20260822T001729Z`.
  Attempt5 remains **ENGINEERING FAIL / STOP / NOT FORMAL**: output after
  Trigger but before HOLD, no output at/after HOLD, 41 all-zero HOLD commands
  with a 0.378262 s max gap, and no reset-era GVG READY.
- ResetStopGate now uses one daemon wall-time 20 Hz zero heartbeat on the same
  node/publisher. Publication locking excludes heartbeat from release/relay;
  publish failure returns to observable HOLD; close bounded-joins before ROS
  resource destruction.
- RouteCoordinator publishes exactly one `READY/reset GVG reconciled` only
  after same-generation release and coherent identical desired/local/GVG
  authority with no transaction/retry/reassert pending. Late reassert success
  owns the same publication; a new reset invalidates the token.
- Probe records dispatch-to-HOLD output as `pre_hold_inflight_outputs`, fences
  old output at received generation-2 HOLD, records dispatch-to-HOLD latency,
  and STOPs if HOLD is missing or later than 0.5 s.
- Validation: source-first focused **214 passed**; fresh-installed focused
  **214 passed**. Wider result: **819 passed, 1 skipped**, plus one unrelated
  frozen Rivermark absolute-path failure. Probe package-configured flake8,
  changed-source `py_compile`, and `git diff --check` passed.
- Fresh two-package build/install PASS:
  `/tmp/v6_attempt5_closure_build.g7Sp4d`,
  `/tmp/v6_attempt5_closure_install.a1hBg5`, and
  `/tmp/v6_attempt5_closure_log.A2CV4w`; installed import and both entrypoint
  help checks passed.
- Verdict: **PASS (code/build/unit only)**. No live runtime was launched;
  Attempt6 remains **PENDING**.
- Handoffs: `V6_ACTIVE_RESET_PROBE_20260822.md`,
  `V6_RESET_SEED_STOP_GATE_20260822.md`, and
  `V6_ROUTE_RESET_RETIREMENT_20260822.md`.

## 2026-08-22 — Attempt6 coordinator-retirement-fence amendment

- Goal: admit only the bounded cross-topic old-route delivery observed between
  received HOLD and the matching coordinator abort pair, while preserving a
  strict old-output fence before fresh navigation.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `4d356a5abfe6d0f6a4c7e77018300ccb75056383`. Fixed Module3 main remained
  `22d66470c4b903349b2467dc876490bbebfc0083`.
- Immutable input:
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_active_reset_live_attempt6_20260822T010744Z`.
  Attempt6 remains **ENGINEERING FAIL / STOP / NOT FORMAL**.  Its finalized bag
  recorded one old request-2 progress/lookahead/goal-update triplet about
  0.0076 s after HOLD, the exact abort pair about 0.0089 s after HOLD, and no
  old output after that pair.  Fresh navigation was not exercised.
- Changed only the active-reset probe, focused tests, this ledger, and
  `V6_ACTIVE_RESET_PROBE_20260822.md`.  No route, gate, IMU, control,
  Integration, or Module2 implementation changed.
- Contract: dispatch-to-HOLD remains at most 0.5 s.  HOLD-to-old Bool `false`
  plus exact `aborted/simulation_reset/reset_epoch=2` JSON is at most 0.25 s.
  Pre-pair route callbacks are recorded with type/time/available request ID;
  wrong ID, deadline overrun, missing/wrong/duplicate terminal, or any output
  after pair completion is fail-stop.  Quiet remains one second from the
  pair-completion fence.  Callback order does not claim a cross-topic DDS
  total order, and final success remains provisional plus finalized-bag review.
- Validation: source-first and fresh-installed probe plus retained package
  contract each **82 passed**; package-configured flake8, `py_compile`, and
  `git diff --check` passed.  Clean isolated package build/install passed at
  `/tmp/v6_probe_retirement_final_build.iQHvBF`,
  `/tmp/v6_probe_retirement_final_install.m5R4AB`, and
  `/tmp/v6_probe_retirement_final_log.RRYMMs`; installed import and help passed.
- Verdict: **PASS (code/build/unit only)**.  No ROS/Isaac/Nav2/navigation/reset
  or evidence campaign was run.  Attempt7 remains **PENDING**.
- Handoff: `docs/handoff/V6_ACTIVE_RESET_PROBE_20260822.md`.

## 2026-08-22 — Attempt7 reset-owned subscriber GID-rotation amendment

- Goal: admit only the reset-owned Isaac control-subscription GID replacement
  observed in Attempt7, without weakening any other safety-critical topology
  identity or functional command-coverage check.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `12b74ba84f76b921695c92b8fd4d31caded32284`.  Fixed Module3 main remained
  `22d66470c4b903349b2467dc876490bbebfc0083`.
- Immutable input:
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_active_reset_live_attempt7_20260822T013726Z`.
  Attempt7 remains **ENGINEERING FAIL / STOP / NOT FORMAL**: the probe stopped
  at `post_release` after the exact
  `/_World_Graphs_Control_SubscribeTwist` subscription GID changed from
  `010fa6bfc4c415930100000000000604` to
  `010fa6bfc4c415930100000000001604`; fresh navigation was not exercised.
- Changed only `active_reset_probe.py`, focused tests, this ledger and
  `V6_ACTIVE_RESET_PROBE_20260822.md`.  No route, reset-gate, IMU, control,
  Integration, or Module2 implementation changed.
- Contract: all semantic endpoints/GIDs are strict through `pre_reset`.
  `post_release`/`pre_fresh` may contain only one exact reset-owned
  `/cmd_vel_sim` subscription replacement, with stable node/name/namespace/
  type/count, old GID absent, new GID unique, and no publisher or unrelated GID
  delta.  The accepted replacement persists through `pre_fresh` and is recorded
  under `topology_gid_rotations`.  Discrete snapshots do not prove absence of
  instantaneous overlap; callback coverage remains an independent authority
  check.
- Validation: source-first focused **54 passed** and fresh-installed focused
  **54 passed**.  Package-configured flake8, `py_compile`, `git diff --check`,
  installed import path and entry-point help passed.  Fresh isolated package
  build/install passed at `/tmp/v6_probe_gid_rotation_build.tQuspc`,
  `/tmp/v6_probe_gid_rotation_install.NC1gLQ`, and
  `/tmp/v6_probe_gid_rotation_log.pnCbdh`.
- Verdict: **PASS (code/build/unit only)**.  No live runtime was launched;
  Attempt8 remains **PENDING**.
- Handoff: `docs/handoff/V6_ACTIVE_RESET_PROBE_20260822.md`.

## 2026-08-22 — Active-reset late-first GID-rotation checkpoint tightening

- Goal: close the remaining discrete-checkpoint gap in the reset-owned
  subscriber exception before Attempt8.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `41c6a02a90e2e2fa1ff8e1c040f078334388834c`.  Fixed Module3 main remained
  `22d66470c4b903349b2467dc876490bbebfc0083`.
- Changed only `active_reset_probe.py`, its focused tests, this ledger, and
  `V6_ACTIVE_RESET_PROBE_20260822.md`; no product, route, reset-gate, control,
  obstacle, or IMU implementation changed.
- Contract: the first reset-owned GID replacement is allowed only at
  `post_release`.  Baseline at `post_release` requires exact baseline at
  `pre_fresh`; an admitted post-release replacement requires exact persistence
  of that rotated snapshot at `pre_fresh`.
- Validation: source-first focused **55 passed** and clean fresh-installed
  focused **55 passed**, including positive persistence and negative late-first
  coverage.  Package-configured flake8, `py_compile`, `git diff --check`, clean
  isolated build/install, installed import-path assertion and installed entry
  point help passed.  Build root:
  `/tmp/v6_probe_gid_checkpoint_clean.CnAEae`.
- Verdict: **PASS (code/build/unit only)**.  No ROS/Isaac/Nav2/navigation/reset
  or evidence campaign was run.  Attempt8 remains **PENDING**.
- Handoff: `docs/handoff/V6_ACTIVE_RESET_PROBE_20260822.md`.

## 2026-08-22 — Attempt8 cross-writer receive-order amendment

- Goal: admit either legal reader receive order between generation-2 HOLD and
  the exact old terminal pair without weakening the common retirement fence.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `a9373ce0a3829d128efc89e0a40122f16cd741b5`.  Fixed Module3 main remained
  `22d66470c4b903349b2467dc876490bbebfc0083`.
- Immutable input:
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_active_reset_live_attempt8_20260822T021512Z`.
  Attempt8 remains **ENGINEERING FAIL / STOP / NOT FORMAL**.  The probe reader
  received Bool `false` before HOLD and stopped; the finalized bag reader
  recorded HOLD 0.000870 s before Bool and 0.000926 s before the exact abort
  JSON, with no old output at or after HOLD or pair completion.  Fresh route,
  post-release topology and postzero were not tested.
- Changed only `active_reset_probe.py`, its focused tests, this ledger and
  `V6_ACTIVE_RESET_PROBE_20260822.md`; no product, route, reset-gate, control,
  obstacle, or IMU implementation changed.
- Contract: HOLD-first and exact-terminal-first are both buffered.  The fence
  exists only after HOLD, Bool `false`, and exact old
  `aborted/simulation_reset/reset_epoch=2` JSON all exist, with both
  dispatch-to-HOLD and dispatch-to-pair at most 0.5 s and absolute receive skew
  at most 0.25 s.  Fence/quiet time is `max(HOLD,pair completion)`.  Earlier
  old output is recorded as cross-writer in-flight; output after the fence,
  wrong identity/epoch/reason, duplicates, missing sides and timeouts STOP.
  Gate order/coverage remains mandatory, and finalized-bag order remains the
  external verdict boundary.
- Validation: source-first focused **60 passed** and fresh-installed focused
  **60 passed**; package-configured flake8, `py_compile`, `git diff --check`,
  installed import path and entry-point help passed.  Fresh isolated build,
  install and log roots are `/tmp/v6_probe_attempt9_build.hHQLhc`,
  `/tmp/v6_probe_attempt9_install.wN2fwa`, and
  `/tmp/v6_probe_attempt9_log.EOokYP`.
- Verdict: **PASS (code/build/unit only)**.  No ROS/Isaac/Nav2/navigation/reset
  or evidence campaign was run.  Attempt9 remains **PENDING**.
- Handoff: `docs/handoff/V6_ACTIVE_RESET_PROBE_20260822.md`.

## 2026-08-22 — Attempt9 latest-value actuator relay and postzero amendment

- Goal: fix the stale `/cmd_vel` reader backlog proven by Attempt9 and replace
  a continuous-heartbeat postzero assumption with a bounded event-driven
  terminal-zero contract, without weakening actuator or physical-stop safety.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `22c7f5c75b46ad1cc0db69bd8c1d61f68b595c39`.  Fixed Module3 main remained
  `22d66470c4b903349b2467dc876490bbebfc0083`.
- Immutable input:
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_active_reset_live_attempt9_20260822T023957Z`.
  Attempt9 remains **ENGINEERING FAIL / STOP / NOT FORMAL**: upstream zeros
  arrived by result +0.0455 s, but stale gate output remained nonzero through
  +0.4006 s, first actuator zero was +0.5732 s, and post-terminal GT yaw moved
  about 0.156 rad.
- Product change: ResetStopGate's final `/cmd_vel` reader is reliable/volatile
  `KEEP_LAST depth=1`, so the latest CollisionMonitor zero overwrites an
  unprocessed stale train before Isaac's once-per-frame rclpy spin.  Output
  publisher, wall HOLD heartbeat, generation fence, and unique command
  authority are unchanged.  Callback teardown now retains the actual Jazzy
  callback instead of the API's `None` return.
- Probe change: each four-chain topic must produce finite zero in result +/-
  0.25 s; a later nonzero STOPs.  One second of event-driven silence is legal
  after zero, while `/cmd_vel_sim` still needs two reasonably spaced zeros,
  GT needs continuous coverage with XY <=0.02 m and unwrapped yaw <=0.02 rad,
  collision remains false, and end topology must be exactly stable.  JSON
  records last-NZ/first-zero/settle/zero-count/silence/classification.
- Validation: source ResetStopGate/reset-service **36 passed**; source probe
  plus package contracts **104 passed**; source-first ActivationGate **14
  passed**.  Fresh isolated `robot_experiments` build/install passed at
  `/tmp/v6_attempt10_fix_build.vfIE1f`,
  `/tmp/v6_attempt10_fix_install.dCXUjX`, and
  `/tmp/v6_attempt10_fix_log.uE0yIV`; installed probe/package tests passed
  **104 tests**, import-path and installed help checks passed.  Probe flake8,
  changed-file `py_compile`, and `git diff --check` passed.
- Verdict: **PASS (code/build/unit only)**.  No ROS/Isaac/Nav2/navigation/reset
  or evidence campaign was run.  Attempt10 remains **PENDING** and must use a
  fresh isolated runtime plus finalized bag.
- Handoffs: `V6_RESET_SEED_STOP_GATE_20260822.md` and
  `V6_ACTIVE_RESET_PROBE_20260822.md`.

## 2026-08-22 — Attempt10 pairwise XY-span amendment

- Goal: close the active-reset probe's remaining XY-span blind spot without
  changing reset, control, route, obstacle, or IMU product behavior.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `932662911beff74b434e54a8f3d5064f6c4e49ca`.  Fixed Module3 main remained
  `22d66470c4b903349b2467dc876490bbebfc0083`.
- Changed only `active_reset_probe.py`, focused tests, this ledger, and
  `V6_ACTIVE_RESET_PROBE_20260822.md`.  Position `span_m` is now the exact
  maximum pairwise Euclidean distance in the window.  A `+0.019/-0.019 m`
  oscillation therefore records 0.038 m and STOPs against the 0.02 m bound;
  static and monotonic straight-motion cases retain the expected span.
- Scope: the shared calculation is used consistently for reset ground-truth,
  reset odometry, and fresh postzero coverage.  Landing-error calculations and
  all thresholds are unchanged.
- Validation: source probe plus package contracts **108 passed** (focused probe
  **70 passed**); fresh-installed probe plus package contracts **108 passed**.
  Changed-file flake8, `py_compile`, `git diff --check`, installed import path,
  installed entry-point help, and fresh isolated `robot_experiments` build
  passed.  Build/install/log roots:
  `/tmp/v6_probe_pairwise_build.j0Hfvn`,
  `/tmp/v6_probe_pairwise_install.mO2nop`, and
  `/tmp/v6_probe_pairwise_log.MWsXJn`.
- Verdict: **PASS (code/build/unit only)**.  No ROS/Isaac/Nav2/navigation/reset
  or evidence campaign was run.  Attempt10 remains **PENDING** and must use a
  fresh isolated runtime plus finalized bag.
- Handoff: `docs/handoff/V6_ACTIVE_RESET_PROBE_20260822.md`.

## 2026-08-22 — Reset simplification work package R (code part R1–R4)

- Goal: replace the active-reset state machine and probe with a cold episode
  boundary protocol, minimal generation isolation, the ResetStopGate as the
  physical stop authority, and hot (in-place, Option A) episode re-arm.
- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `96549133f45e8f0abd80f48f1e5bdee012b81a5c`.  Fixed Module3 main remained
  `22d66470c4b903349b2467dc876490bbebfc0083`.
- Commits: `6484e73` (R3 coordinator), `a639270` (R1/R2/R4 runner + probe
  retirement); docs commit follows.  Changed only
  `robot_route_planner/robot_route_planner/ros_node.py`,
  `robot_route_planner/test/test_route_core.py`,
  `robot_route_planner/test/test_cognitive_graph_adapter.py`,
  `robot_experiments/robot_experiments/v6_formal.py`,
  `robot_experiments/robot_experiments/reset_receipt.py`,
  `robot_experiments/robot_experiments/active_reset_probe.py` (deleted),
  `robot_experiments/test/test_active_reset_probe.py` (deleted),
  `robot_experiments/test/test_v6_formal.py`,
  `robot_experiments/test/test_reset_receipt.py`,
  `robot_experiments/setup.py`, this ledger, and
  `V6_RESET_SIMPLIFICATION_20260822.md`.
- R1: readiness now enforces the cold episode boundary — no pre-reset route
  traffic, zero `/cmd_vel`/`/cmd_vel_sim` and bounded odom span in the 1.0 s
  quiet window, sole-publisher ownership; violations fail-stop with explicit
  blocker labels.  R2: the six minimal invariants (receipt match incl. new
  pose check, post-reset odom landing/span, zero post-reset commands, no
  stale route, GT firewall, sole publishers) are PASS/FAIL in the runner;
  the 2364-line probe and its 1369-line test file are deleted.  R3: the
  coordinator keeps input generation filtering, exactly-once retire with one
  precise abort terminal, and fail-closed hold/release; the pristine binding
  machine, 12-condition READY recheck, and release/GVG-ready ledgers are
  removed (one `reset_ready_pending` flag defers READY to the release);
  HOLD-fenced switch retries stay due and dispatch after release.  R4:
  `arm_reset` accepts the current bridge epoch as baseline and requires
  baseline+1/+2 epoch rollover; B5 readiness accepts a baseline-generation
  seeded string; per-generation exactly-once semantics unchanged.
- Validation: source-first at `5f0e088` — robot_route_planner **172 passed,
  1 skipped** (optional `pxr`), robot_bringup **228 passed**,
  robot_experiments **567 passed, 1 pre-existing worktree-path failure**
  (`test_rivermark_reference_..._converges`, fails identically without these
  changes).  Fresh-installed from isolated base identical.  Isolated colcon
  build PASS: `ros2_ws/build_reset_simpl_QYUBBF`,
  `install_reset_simpl_QYUBBF`, `log_reset_simpl_QYUBBF` (robot_bringup
  against the worktree underlay).  Changed-file flake8
  (`--max-line-length=99 --extend-ignore=A003,Q000`) adds no findings
  (ros_node 51→46, v6_formal 56→55, reset_receipt 2→2); `py_compile` and
  `git diff --check` PASS.  Metrics: probe removal -3733 lines; ros_node.py
  4969→4634 (status handler 230→174, event handler 80→59, completion+READY
  recheck 86→38, pristine machine 190→0, startup tick 35→0).
- Verdict: **PASS (code/build/unit only)**.  No ROS/Isaac/Nav2/navigation or
  live reset was run.  R5 (live verification) is pending; watch items:
  warm-stack residual prior/candidate traffic vs the cumulative negative
  window, B5 baseline string equality, 0.10 m odom landing threshold, and
  the READY detail-string timing note — all listed in the handoff.
- Handoff: `docs/handoff/V6_RESET_SIMPLIFICATION_20260822.md`.

## 2026-08-22 — V6 IMU regime Attempt4: first live schema-2 capture

- Goal: produce the first live schema-2 IMU-regime capture (locked flat20
  runner + LiDAR preflight + stationary seed 8609 + primitives 8610..8618 +
  0.8 s downstream zero receipts + MCAP/phase JSONL) plus the separately
  provenance-bearing Kujiale Estimated goal, then the analysis verdict.
- Branch/worktree: `cognitive-navigation`; permitted Module3 worktree.
  Baselines verified: Integration `f23a7ec`, Module3 `22d6647`, Module2
  `c8297a5`. Runtime from `git archive` snapshot
  `/tmp/v6_imu_session_a_attempt4.IPZuzv`, isolated `*_attempt4` colcon trees
  (Integration 2 + Module3 14 packages PASS), ROS_DOMAIN_ID 171, RViz off,
  capture scale frozen `0.9294`, RF2O off, M0.
- Code changes (committed): session driver/monitor/goal-metadata/audit
  scripts under `scripts/`; `motion_benchmark.py` HOLD-window zero
  publishing, fresh-clock-tick schedule starts, 0.5 s pre-settle drain, and a
  stationary-duration float-epsilon fix; diagnostic-only collision monitor
  `stop_pub_timeout:=30.0` overlay inside the driver (no shared nav config
  changed). Focused suites 126 passed; snapshot full suite 567 passed + 1
  known path-sensitive failure.
- flat20 evidence:
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_imu_regime_session_a_attempt4_20260822T065252Z`.
  **capture_contract_status=PASS** (first live schema-2 capture; 12/12
  windows OK, 10 consecutive reset receipts, zero collision, monitor pass).
  **performance_status=FAIL** (arc_v005_ccw, arc_v010_cw/ccw, arc_v025_cw,
  s_route below thresholds — same chain under-tracking as Attempt3).
  Segment k*: spins 0.9292/0.9309, v0.05 0.9231/0.9496, v0.10
  **0.9836/0.9749**, v0.25 0.9229/0.9182, S 0.9140/0.9209/0.9139; 12-window
  <=5 deg intersection **[0.9207, 0.9420]** (contains frozen 0.9294).
- Kujiale goal evidence:
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_imu_regime_session_a_attempt4_goal_20260822T071314Z`.
  Route G1→G2 completed (44.92 s, 0.202 m, no collision); reset receipt seed
  8619 == requested (Evidence-B seed debt fixed). Goal MCAP fails two
  single-attempt binding fences deterministically: `goal_request_count`
  (probe republishes the goal at 1 Hz until ack — 4 recorded requests) and
  `goal_command_after_terminal` (one nonzero /cmd_vel at terminal+45 ms from
  the smoother decel tail; the historical Evidence B bag shows the same +58 ms
  tail, so this fence was never live-satisfiable).
- Official analyzer outputs: `imu_regime_analysis_flat20_only.json`
  (verdict FAIL via performance; capture PASS; regime AMBIGUOUS only due to
  missing goal) and `imu_regime_analysis_full_contract.json` (verdict FAIL at
  `goal_request_count`) in the flat20 run's `analysis/`.
- Verdict: **FAIL / NOT FORMAL** — schema-2 capture pipeline now live-capable
  and the flat20 regime intersection is measured, but no
  PASS_CANDIDATE/CONFIRMED_NO_GLOBAL_CONSTANT promotion is possible through a
  failed benchmark and an unbound goal attempt. Master decisions pending:
  goal-binding contract vs probe/coordinator behavior, and arc/S tracking
  performance vs thresholds. `yaw_scale=0.9294` and RF2O-off unchanged.
- Handoff: `docs/handoff/V6_IMU_REGIME_ATTEMPT4_20260822.md`.

## 2026-08-22 — V6 cold-boundary R5 live validation attempts 1–7 (stopped by user)

- Goal: live-validate the simplified cold-episode-boundary reset machinery (R1–R4)
  with 2–3 consecutive same-stack episodes; 6 minimal invariants per boundary.
- Branch/worktree: `cognitive-navigation`, permitted Module3 worktree. R5 commits
  `c1d4cde`, `a6b2cae`, `90219af`, `e49b74c`, `7f39ddf`, `6757daa`, `67ee113`,
  `d5ac812`, `7e9417b` on top of the R-package commits. Fixed Module3 main pin
  `22d66470c4b903349b2467dc876490bbebfc0083` verified unchanged.
- Arc: attempts 1–6 each exposed one distinct bootstrap-path defect class
  (HOLD-window map binding, enrollment reseed burst, AMCL straggler ordering,
  untrusted-prior gating, B5 gate witness) and were minimally fixed; attempt 7
  passed the reset/bootstrap gauntlet (receipt seed 7201 == requested, generation
  2, exactly-once reset, GT firewall PASS, sole-publisher ownership PASS) and
  reached actual navigation, ending `STOP/collision` with zero completed legs.
- Result: **reset machinery engineering-usable; remaining blocker is
  navigation-level (collision), owned by Phase 2**. Multi-episode consecutive
  re-arm remains unproven live. Stopped by user decision after attempt 7.
- User decision recorded: IMU regime investigation closed; `yaw_scale=0.9294`
  frozen (pure-spin calibration; no single global constant exists across regimes
  per schema-2 Attempt4 evidence); estimated-substrate non-degradation gate
  lifted; project returns to the estimated-odom + localization + navigation main
  line.
- Evidence: `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_reset_cold_boundary_r5_20260822T095144Z`
  through `..._20260822T115942Z` (7 sessions).
- Handoff: `docs/handoff/V6_RESET_COLD_BOUNDARY_LIVE_20260822.md` (written by
  master from NAS evidence and commit history after the R5 agent was stopped).

## 2026-08-22 — V6 Kujiale low-obstacle layout 6 → 1 (v6_low_box_solo)

- Goal: user decision 2026-08-22 — keep exactly one stationary low obstacle so
  the estimated-odometry closed loop can pass; the box keeps the causal
  condition (0.16 m tall: below the 0.333 m LiDAR plane and 0.218 m RGB-D
  origin, overlapping the wheel/chassis band [0, 0.196] m) for later Module2
  causal runs. Seed 7201 leg G1→G2 had collided with `v6_low_bar_north`.
- Branch/worktree: `cognitive-navigation`, permitted Module3 worktree; commit
  recorded by the task handoff after submission.
- Change: `v6_kujiale_low_obstacles_frozen.yaml` and its manifest now hold a
  single box `v6_low_box_solo` (0.30×0.30×0.16 m, 5 kg, stationary) at
  map (-1.150, -0.350, 0.08) / usd (4.050, 0.150, 0.08), central open hall.
  Runner scenario `v6_kujiale_low_obstacles_static.yaml` static list updated.
  `layout_id`/`revision`/`frozen_date` deliberately unchanged to avoid
  re-touching the causal identity chain.
- Geometry contract: `maximum_route_proximity_m` replaced by
  `minimum_route_edge_clearance_m: 1.0` (measured 1.60–3.57 m on all five
  segments); `minimum_open_bypass_side_m` relaxed 1.9 → 1.2 (measured best
  side 3.41 m); pairwise-clearance field removed (single obstacle).
  Edge distance to every goal ≥ 1.70 m. RGB-D forward visibility verified by
  sampling: 15 route samples with free line of sight (G1→G2 ×10, G3→G4 ×5;
  cos≥0.5, d≤3.5 m).
- Commands: source pytest `isaac_sim/tests/test_v6_low_obstacle_layout.py`
  (7 passed), `test_configuration.py` (55 passed), `test_v6_formal.py`
  (64 passed, unchanged — it holds no obstacle-count assertions),
  `robot_bringup/test_runtime_scripts.py` (33 passed); suite minus 8
  ROS-env-blocked modules: 581 passed, 2 pre-existing failures
  (rivermark frozen-JSON absolute paths from an old checkout; one rclpy
  import) unrelated to this change.
- Evidence: overlay + notes at
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_layout_single_obstacle_20260822T144924Z/`.
- Verdict: **PASS (static contract only)**. Isaac/ROS/Nav2 runtime not started;
  closed-loop passability under estimated odometry must be shown by a live run.

## 2026-08-22 — V6 Phase-1 R5 live #1: leg G1→G2 passes; runner state regression stops episode

- Goal: first live run of the single-obstacle layout (`v6_low_box_solo`) under
  the R5 session driver; verify the Phase-0 passability goal.
- Snapshot `/tmp/v6_r5_phase1.OXoJdO2W`: module3 `f8782e5` (layout 6→1),
  integration `9c94c82`, module2 `2925f80`; isolated `install_r5` builds PASS.
- Run: `scripts/v6_reset_cold_boundary_r5_session.sh RUN_DIR SNAPSHOT_ROOT`,
  ROS_DOMAIN_ID 173, seed 7201 (only episode; driver stops on first failure).
- Result: leg G1→G2 physically succeeded, no collision — final GT error
  0.161 m (tol 0.25), min box-center distance 0.75 m. Episode FAIL:
  `route_completion_timeout:G2` — runner `_maybe_goal_ready()` regresses
  NAVIGATING → GOAL_READY on the next prior/AMCL callback, so 176 route_progress
  and the completion signal were captured but never counted; leg loop waited the
  full 300 s. Boundary six invariants PASS (manual replication,
  `analysis_boundary_invariants_manual.json`). Tooling bug found:
  `scripts/v6_reset_boundary_check.py` crashes on bags with a recorded reset
  event (float-unpack TypeError at :240). Module2 shadow: no discrete
  `v6_low_box_solo` detection during the box pass; layer status 274×
  applied=false.
- Evidence:
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_reset_cold_boundary_r5_phase1_20260822T152936Z`
  (`REVIEWER_NOTE.md`, bag `rosbag/r5_session`, `analysis_bag_phase1.json`,
  `analysis_boundary_invariants_manual.json`, `analysis_cognitive_obstacles.json`,
  `trajectory_overlay_phase1.png`).
- Verdict: **PHASE 1 FAIL (runner contract defect)** — Phase-0 objective itself
  validated (collision-free G1→G2 under estimated odometry).
- Next: minimal `_maybe_goal_ready` guard + checker float fix, then rerun the
  same session driver.

## 2026-08-22 — V6 Phase-1 R5 live #2 (rerun): runner/checker fixes confirmed; G2→G3 collision

- Goal: live-validate the runner state-machine fix and boundary-checker float
  fix (module3 `6f4efef`).
- Snapshot `/tmp/v6_r5_phase1_rerun.yL3KuU8p`: module3 `6f4efef`, integration
  `9c94c82`, module2 `2925f80`; same R5 session driver, seed 7201, domain 173.
- Result: runner fix confirmed — leg G2 completed and counted
  (`completed_leg_ids=["G2"]`, progress 213 / completion 1 / results 2 vs 0/0/0
  in live #1) and leg G3 dispatched; fixed checker ran end-to-end, six
  invariants PASS (overall pass=false driven only by the real collision).
  New blocker: leg G2→G3 collided with static `livingroom_595/cabinet_0003` at
  map (-0.60, 3.17) (doorway corner, in static map) — AMCL–GT error 0.20–0.25 m
  at contact plus insufficient clearance margin; Nav2 aborted error_104 after
  ~3.5 s pushing. Module2 shadow baseline reproduced: no discrete
  `v6_low_box_solo` detection during the box pass.
- Evidence:
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_reset_cold_boundary_r5_phase1_rerun_20260822T173340Z`
  (`REVIEWER_NOTE.md`, `analysis/boundary_checks.json`,
  `analysis_module2_health_baseline.json`, `trajectory_overlay_phase1_rerun.png`).
- Verdict: **PHASE 1 FAIL (navigation clearance margin under estimated
  odometry)** — not the runner, not Phase-0 layout, not boundary invariants.
- Next: widen v6 margin — committed as `fd01d0ac` (footprint_padding 0.03,
  inflation_radius 0.55, v6 overlay only). Multi-episode same-stack re-arm
  still unproven live.

## 2026-08-22 — V6 Phase-1 R5 live #3 (all-fix): F1/F2 confirmed active; stationary bootstrap deadlock

- Goal: live-verify the full fix stack — module3 `fd01d0ac` (layout + runner +
  v6 margin), integration `c72ce867` (F1 remove 50× motion amplification + F2
  adapter injection), module2 `4e38152a` (confidence decoupling).
- Snapshot `/tmp/v6_r5_phase1_allfix.uVC8KVby`; same R5 session driver, seed
  7201, domain 173.
- Result: episode STOP `post_reset_readiness_timeout`, no goal published, robot
  never moved (GT span 0.0 m, no collision). F1/F2 effects confirmed: post-reset
  module1/module2/model_output unhealthy 152→4 vs previous run;
  `cognitive_obstacle_layer/status` applied=true 2764 (epoch 2) — F2+4e38152a
  chain now writes obstacles into the costmap. New blocker (root cause): with F1
  the parked robot produces zero motion freshness (`stale_input` 832/1078), and
  the stationary startup revalidation window is closed by the first post-reset
  initialpose, so 1073/1078 planning priors age past the B4 candidate TTL → all
  localization candidates rejected → readiness never satisfied (bootstrap
  deadlock; readiness chain had implicitly depended on the 50× noise
  amplification). Checker: six invariants PASS; the two false flags are
  zero-goal window artifacts. Evidence gap recorded: the bag did not record
  `/bio_nav/module2/cognitive_obstacles` and the runner status capture lacked
  `raised_cell_count`, so the 2764 writes cannot be classified true vs false
  positive — gap closed by `ffbd7ba` (session rosbag topic + runner capture
  field) for the next live run.
- Evidence:
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_reset_cold_boundary_r5_phase1_allfix_20260822T182350Z`
  (`REVIEWER_NOTE.md`, `analysis/boundary_checks.json`,
  `analysis_readiness_allfix.json`, `analysis_shadow_f1f2.json`,
  `trajectory_overlay_phase1_allfix.png`).
- Verdict: **PHASE 1 FAIL (F1 × B4 bootstrap interaction defect, integration
  side)** — not the runner, not margin, not module2, not boundary invariants.
- Next: integration fix in progress (keep the stationary revalidation window
  open across the first initialpose, or a motion-independent B4 freshness
  criterion); then rerun this driver — multi-episode re-arm still unproven.

## 2026-08-23 — V6 Kujiale AMCL 地图重生成（Isaac omap）：diff 干净、五腿全通；采用暂停于 GVG 强耦合

- Goal: 用 Isaac Occupancy Map Generator 从 Kujiale USD 实际碰撞几何重生成 AMCL
  静态地图，消除 warehouse_new 幻影唇行（run#8 GT 穿图、run#9 AMCL 蠕行死锁）。
- 工具: `isaac_sim/tools/rivermark_occupancy_generate.py` 最小参数化（显式 USD
  窗口/整格裁剪/flip 输出帧/小场景种子）；修正 Generator buffer 列序认知
  （列 = −x_usd 降序，地标 56 物体客观判定），修正后再生成与离线校验逐字节一致。
- 参数: 0.05 m/cell；高度带 z∈[0.30,0.37] m（对齐 LiDAR 0.333 m 扫描面）；
  输出 154×248 格、origin (-5.14,-6.52)，与 warehouse_new 逐格同构。
- 结果: 唇行幻影消失（(-0.3,1.96) 全 free；残留占用为 table_0000 真实边缘）；
  门宽一致（1.15–1.20 vs 旧 1.20–1.30 m）；cabinet_0003 对齐；6 历史低矮障碍区+solo
  全新图 free；G1–G5 free；run#8/#9 GT 379 采样在新图 0 占用 0 未知；五腿 maximin
  全连通、瓶颈 0.30–0.40 m（lethal 0.22）。
- 耦合: posegraph 弱耦合（navigation 不加载，仅 manifest 契约绑定，可字节拷贝复用）。
  **GVG geojson 强耦合**：nav2 route_server 运行时消费冻结图（/compute_route 在 v6
  活路径）；22/78 canonical 边在新图跌破 0.22 m（edge 4/6/55/56 为 0，穿越真实
  占用，edge 6 紧邻 G5）。按任务条款停止采用，未再生物件、未切换引用。
- Evidence: `/mnt/nas_home/Bio_Nav_Data/experiments/analysis/v6_kujiale_map_regen_20260823/`
  （REPORT.md、diff_analysis.json、orientation_landmarks.json、validated/ 候选图、
  figures/ 顶视叠加+GT 轨迹+净空热力+diff overlay）。
- Verdict: **地图本身 PASS（可采用）；链路切换 BLOCKED——待 master 决策 GVG 再生
  （`robot_route_planner build_graph` CLI 现成）与 graph_id 迁移（涉 module2 先验）**。
- Next: master 决策后按 REPORT.md §切换路径执行（文件/manifest/spawn/引用/测试清单
  已勘察齐全）。

## 2026-08-23 — V6 新地图整体采用 + GVG 再生（master 决策执行）

- Decision: master 裁定采用 `v6_kujiale_isaacgen_v1`（diff 干净、GT 一致、五腿连通）；
  module2 graph_id 先验重登记明确不在本次范围（Phase 2/3；M1 shadow 下 edge prior
  本就全部超时回退 geometry-only，graph_id 不匹配走既有 fail-open 丢弃路径，无功能回退）。
- Bundle: `data/maps/occupancy/v6_kujiale_isaacgen_v1.{pgm,yaml}`（origin [-5.14,-6.52],
  0.05 m, 154x248）、`data/maps/posegraphs/v6_kujiale_isaacgen_v1.{posegraph,data}`
  （warehouse_new 字节拷贝——navigation 模式不反序列化，仅契约绑定）、
  `data/maps/manifests/v6_kujiale_isaacgen_v1.yaml`（bundle ce1dfd19…，calibration 同
  warehouse_new）、新 spawn `kujiale_0026_A_to_B_door_open.v6_isaacgen_v1.spawn.yaml`
  （几何不变，版本/bundle 引用换新；warehouse_new spawn 原样保留）。
  `load_map_manifest` + 4 个 pose 的 initial-pose 契约全部通过。
- GVG: `robot_route_planner.cli`（build_graph）在新图上确定性再生 →
  `ros2_ws/src/robot_route_planner/config/v6_kujiale_isaacgen_v1_gvg_v1.{geojson,
  support_map.json,summary.json}`；graph_id `v6_kujiale_isaacgen_v1:gvg_v1` rev 1，
  24 节点 / 48 有向边 / 310 support 边（FEASIBLE 30 / UNKNOWN 16 / INFEASIBLE 2）。
  验证（gvg_regen_validation.json）：全部边净空 ≥0.224 m（lethal 0.22，无穿越真实占用）；
  五腿 G1→G2→G3→G4→G5→G1 逐段 route 均存在（腿内最小边净空 0.25–0.40 m）；
  五目标点 support 挂接全 FEASIBLE（G3 贴床兜位，首个可行挂接 1.896 m）。
- 引用切换: `scripts/v6_reset_cold_boundary_r5_session.sh`、`scripts/v6_imu_regime_attempt4_session.sh`
  （含 probe --map）、`v6_final_kujiale_{static,dynamic,appearance}.yaml`（occupancy_map/
  posegraph_file/spawn_manifest/route_graph）、`v6_kujiale_low_obstacles_static.yaml`
  （map/posegraph_version）、`v6_kujiale_low_obstacles_frozen_manifest.yaml`（occupancy_map）、
  `run_v6_kujiale_low_obstacles.sh`（显式四参）。`run_ros.sh` 默认值与 launch 默认 graph
  保持 warehouse_new（历史流不动）。
- Tests: test_map_manifest（+新 bundle 用例）、test_v6_formal、test_v6_low_obstacle_layout、
  test_gvg_and_feasibility（+新图连通/净空用例）；suites 257 passed。
- Evidence: `/mnt/nas_home/Bio_Nav_Data/experiments/analysis/v6_kujiale_map_regen_20260823/`
  （REPORT.md、diff_analysis.json、gvg_regen_validation.json、orientation_landmarks.json、
  figures/）。
- Verdict: **ADOPTED（工程采用）**。下一次 live run（Phase 1 第十次）以新地图+新 GVG+
  AMCL round-2+M1 shadow 全量验证。
- Next: live run 验证柜-桌走廊（0.63 m 净宽、瓶颈 0.30 m）AMCL round-2 表现；
  Phase 2/3 再做 module2 侧新 graph_id 先验登记。

## 2026-08-23 — V6 Phase 1 定位 A/B 定案：主线冻结 odom_static（AMCL 0/3 vs odom_static 3/3）

- Goal: G1→G2 走廊定位后端 A/B（arm A=amcl、arm B=odom_static，同一 r5 session
  driver 经 `V6_LOCALIZATION_BACKEND` 切换，各 3 次 live run）裁定 Phase 1 定位主线。
- Result: AMCL 0/3（A1/A2/A3 全部 SAFETY_STOP_DEADLOCK，corridor 样本 n=0/7/8，
  有样本处 corridor p50 0.204–0.304 m）；odom_static 走廊段 3/3（B1/B2/B3 corridor
  n=34/35/35，p50 0.0062/0.0565/0.0873 m），leg1（G1→G2 全腿）2/3 PASS
  （B2/B3；B1 过走廊后 SAFETY_STOP_DEADLOCK @(-0.261,3.672)）。
- Decision: Phase 1 定位主线冻结为 `odom_static + Wheel/IMU/EKF`；
  `scripts/v6_reset_cold_boundary_r5_session.sh` 默认
  `V6_LOCALIZATION_BACKEND=odom_static`，`=amcl` 保留为回滚/对照入口。
  后端状态：AMCL 退为对照/回滚臂；RF2O 维持 off（未安装）；GridLocalizer 未引入本仓。
- Warning（记录不修）: odom_static anchor jitter —— 每次 enrollment seed 重锚定
  map->odom 的瞬时跳变，幅度小、不污染 corridor 指标。
- Blocker: G2→G3 doorway —— B2 planner abort（`navigate_to_pose_failed_error_105`，
  request_id=3）、B3 碰撞 @(-0.564,3.119)；两者均已完成 G2 腿，失败发生在 G2→G3 段，
  与定位臂选择无关。
- Evidence: `/mnt/nas_home/Bio_Nav_Data/experiments/analysis/v6_ab_g1g2_20260823/`
  （`ab_summary.json` + A1–A3/B1–B3 六次 run 目录）。
- Verdict: **A/B DECIDED — odom_static 为 Phase 1 定位主线**。
- Next: G2→G3 doorway 第一错误层离线诊断（B2 planner abort vs B3 碰撞）。

## 2026-08-23 — V6-GRID localization core and public interface

- Goal: replace the production Module3 localization launch with the installed
  Isaac ROS 4.5.0 LaserScan→FlatScan→OccupancyGridLocalizer chain and add the
  thin, generation-gated sole `map->odom` manager.
- Branch/worktree: `cognitive-navigation`, permitted Module3 worktree; base
  `ccbd54d1f800fcf6db073f22b52377a24b67a900`; result is the single commit
  containing `docs/handoff/V6_GRID_LOCALIZATION_CORE_20260823.md`.
- Change: new `robot_grid_localization` package; `robot_mapping` production
  backend/default is now `grid`, passes one validated map YAML to both map
  server and localizer, preserves `/localization_result`, and does not start or
  select AMCL/odom_static. Manager accepts standard
  `PoseWithCovarianceStamped`, proxies `/bio_nav/relocalize` to the installed
  `std_srvs/Empty` service, gates one pending generation, requires finite data
  and exact-stamp `odom->base_link`, then publishes accepted pose/status and
  `T_map_odom=T_map_base*inverse(T_odom_base)`.
- Frozen outputs: `/bio_nav/localization_pose`
  (`geometry_msgs/PoseWithCovarianceStamped`) and
  `/bio_nav/localization/status` (`diagnostic_msgs/DiagnosticArray`) are
  reliable/transient-local keep-last 1. Fixed status keys and downstream
  semantics are recorded in the handoff.
- Validation: focused pytest **14 passed**; clean `/opt/ros/jazzy` isolated
  build **2 packages passed**; new-package pytest **10 passed** and
  flake8/pep257/xmllint PASS; mapping CTest **5/5 passed**. Generic mapping
  `colcon test` wrapper was blocked before CTest by the pre-existing unselected
  `robot_slam_solver` runtime hook, so direct CTest was used without expanding
  scope.
- Verdict: **PASS (code/test/build only)**. No ROS graph, Isaac Sim, NITROS,
  Nav2, TF ownership, or live localization run was started; live result is
  **UNVERIFIED**.
- Next: fresh Integration/bringup coder consumes the three frozen public
  interfaces, removes old readiness/seed paths, waits for matching-generation
  `ACCEPTED`, and then a reviewer performs the first actual grid smoke.

## 2026-08-23 — V6-GRID localization core blocker repair

- Goal: close the post-review dynamic-TF, result-generation, timeout,
  duplicate-trigger, and exact-stamp TF callback blockers without changing the
  frozen ROS message/service interfaces.
- Branch/worktree: `cognitive-navigation`, permitted Module3 worktree; repair
  base `1097f2ca0b15ae17d80d625b8c67d321ae69759b`; result is the single commit
  containing this ledger entry.
- Changed: only `robot_grid_localization`, this core handoff, and this ledger.
  The latest accepted dynamic `map->odom` correction is sent immediately and
  refreshed at current ROS time (default 20 Hz); exact-stamp TF handling uses a
  two-thread executor. Pending generations require a result stamp at or after
  their trigger, time out terminally after a configurable default 10 s, and can
  then be retriggered. Duplicate Trigger calls fail without changing the active
  generation's latched `WAITING` status.
- Validation: four review probes **4/4 PASS**; source-focused suite **19
  passed**; isolated `/opt/ros/jazzy` build/install/import PASS at
  `/tmp/v6_grid_core_repair_final.jpybT7`; installed-package pytest **15
  passed**; flake8/pep257/xmllint PASS.
- Verdict: **PASS (code/test/build only)**. No ROS graph, Isaac Sim, vendor
  Grid Localizer, Nav2, or live TF run was started; live behavior remains
  **UNVERIFIED**.
- Remaining risk: the vendor result carries no generation identifier, so the
  fail-closed correlation is source-time based; a semantically old result with
  a stamp at or after a newer trigger cannot be distinguished until the vendor
  exposes a stronger correlation token. Live component/result timing and TF
  ownership still require the authorized smoke stage.

## 2026-08-23 — V6-GRID FlatScan source-stamp correlation repair

- Goal: replace the service-trigger-time lower bound after confirming Isaac ROS
  Occupancy Grid Localizer 4.5.0 localizes its cached `flatscan` immediately on
  the Empty service, while `flatscan_localization` directly localizes its input
  and the result retains that FlatScan header.
- Branch/worktree: `cognitive-navigation`, permitted Module3 worktree; base
  `48ed78d7982d6bde61e7f3c8afb69f2918ff5071`; result is the single commit
  containing this entry.
- Changed: only `robot_grid_localization`, the Grid core handoff, and this
  ledger. `/bio_nav/relocalize` now opens `WAITING_FOR_SCAN`; the next valid
  source stamp newer than the pre-trigger observed baseline is forwarded once,
  unchanged, from `/flatscan` to `/flatscan_localization`. That exact stamp is
  recorded as `expected_result_stamp_ns`; only an exact result can be accepted.
  Other stamps neither consume pending nor publish a current-generation status.
  The vendor Empty service is no longer called by the production manager.
- Preserved: public Trigger/pose/status types and names, 20 Hz dynamic TF,
  concurrent exact-stamp TF lookup, sole `map->odom` ownership, correction
  formula, duplicate behavior, and retry after timeout. Timeouts now distinguish
  `scan_timeout` from `result_timeout`.
- QoS/type check: installed `isaac_ros_pointcloud_interfaces/msg/FlatScan`
  Python type is available with a standard header; manager input/output use the
  vendor v4.5 `DEFAULT` contract (reliable/volatile keep-last 10).
- Validation: focused source suite **24 passed** (20 package + 4 unchanged
  mapping checks); flake8/pep257/xmllint PASS; isolated build/install/import
  PASS at `/tmp/v6_grid_stamp_repair.HMLzaL`; installed colcon tests **20
  passed, 0 failures**.
- Verdict: **PASS (code/test/build only)**. No full ROS graph, Isaac Sim,
  vendor localizer, Nav2, or live TF run was started.
- Remaining risk: live NITROS/QoS negotiation, delivery order, vendor result
  stamp echo, exact-stamp TF availability, and TF ownership are unverified.

## 2026-08-23 — V6-GRID bringup and Nav2 production cutover

- Goal: connect normal estimated localization/navigation bringup to the
  existing Grid backend, remove Bringup reseed ownership, and freeze the
  Phase-1 LiDAR-only XY-goal Nav2 profile.
- Branch/worktree: `cognitive-navigation`, permitted Module3 worktree; base
  `749ee7c3128c87d65560b40f5401231413f50bcc`; result is the single commit
  containing `docs/handoff/V6_GRID_BRINGUP_NAV2_CUTOVER_20260823.md`.
- Changed: non-Ideal `auto` localization owner is `grid`; normal wrappers use
  `estimated + occupancy_only`; production launch no longer passes AMCL or
  includes initial pose. ActivationGate only observes fixed-key Grid status
  and TF, requires a newer accepted generation after reset, and never calls a
  reseed/relocalize service. Nav2 uses PositionGoalChecker, no GoalAngleCritic,
  LiDAR-only formal Costmaps, and OFF/shadow cognitive writes (M2/M3 active is
  clamped to shadow during Phase 1).
- Validation: source suite **252 passed, 10 skipped** (retired Attempt21 direct
  depth contract); installed launch args show `estimated + occupancy_only +
  auto`; isolated `/opt/ros/jazzy` build root
  `/tmp/v6_grid_bringup_narrow.EXEMuz` built both owned packages; final colcon
  result **270 tests, 0 errors, 0 failures, 10 skipped**; focused
  flake8/pep257 and `git diff --check` PASS.
- Verdict: **PASS (code/test/build only)**. No ROS graph, Isaac Sim, vendor
  component, Nav2 motion, visual evidence, or qualification run was started.
- Remaining risk/next: full-overlay launch and Phase 1B live smoke must verify
  NITROS/status ordering, one Integration relocalize call per reset, one
  `map->odom` owner, full TF, then the empty-house five-leg loop.

## 2026-08-23 — Phase-1 reset gate and M0 launch blocker repair

- Goal: close the two reviewer-reproduced Phase-1 blockers without changing
  localization interfaces, relocalize ownership, Nav2 configuration, or
  Module2 M1-M3 contracts.
- Branch/worktree: `cognitive-navigation`, permitted Module3 worktree; base
  `0475a680746039f74dbeb15d4c5f06c52530d79e`; result is the single commit
  containing this entry.
- Changed: reset epochs require WAITING observation before same-generation
  ACCEPTED; accepted correction fields must match actual planar `map->odom`
  within configurable 0.01 m/rad defaults. M0 launch setup always passes
  `module2_enabled=false` for stable and low-obstacle profiles; default route
  backend remains `gvg`; M1-M3 behavior is unchanged.
- Validation: focused suite **59 passed**; ROS gate fixture **3 passed**;
  relevant source regression **261 passed, 10 skipped**; clean isolated
  `robot_bringup` build/test at `/tmp/v6_gate_m0_repair_clean.ipjnBD` **239
  passed, 10 skipped, 0 failures**. No cache/bytecode was used for the final
  isolated test.
- Verdict: **PASS (code/test/synthetic only)**. No Isaac Sim, navigation,
  evidence campaign, qualification, or live ROS graph was started.
- Remaining risk: actual transient status/TF timing and one-owner runtime
  behavior remain unverified. Conditional `bio_nav_fusion` buildability is
  explicitly unresolved; Phase 1 must use a profile with buildable plugins.

## 2026-08-23 — V6-GRID Phase-1 canonical empty-room runner

- Goal: replace the remaining B5/retired-localization/low-obstacle runner path
  with the real Grid + stable + M0 full-house entry, without adding a wrapper
  framework or starting a live campaign.
- Branch/worktree: `cognitive-navigation`, permitted Module3 worktree; base
  `ced458cda229464c2d0f11b06c432a108eab4592`; result is the single commit
  containing `docs/handoff/V6_GRID_PHASE1_CANONICAL_RUNNER_20260823.md`.
- Changed: `v6_formal` now gates G2 on a newer WAITING→ACCEPTED Grid generation,
  matching ResetStopGate release, Nav2, and full TF. The only mission is reset
  G1 then XY-only `G2→G3→G4→G5→G1`, with identity orientation as a protocol
  placeholder. Static is Phase-1 enabled; dynamic/appearance retain later
  intent but are dispatch-disabled. R5/Isaac/ROS entries fix Grid, stable, M0,
  Module2 false, gvg, RF2O off, low obstacles off, dynamic actors off, and NAS
  recording. The reset bridge no longer publishes a global localization seed.
- Validation: focused runner/reset **45 passed**; V6 runtime scripts **8
  passed, 25 deselected**; shell syntax, production retired-token grep,
  E/F/I-focused lint, and `git diff --check` PASS. Isolated
  `robot_experiments` build/install PASS at
  `/tmp/v6_phase1_runner_build.SDLQp0`; installed CLI dry probe printed the
  expected runtime and five legs.
- Verdict: **PASS (code/test/build/dry-probe only)**. No Isaac Sim, ROS graph,
  NITROS, Nav2 motion, visual evidence, or qualification was run.
- Remaining: build fresh combined overlays; verify Integration launch
  arguments, process/domain isolation, Grid status ordering/relocalize count,
  one `map→odom` owner, and the actual empty-room five-leg loop. Stable remains
  deliberate until current-IDL `bio_nav_fusion` buildability is proven.

## 2026-08-23 — V6-GRID actual-launch map/QoS blocker repair

- Goal: repair only the reproduced vendor map-constructor and `/scan` QoS
  blockers, plus align the canonical session with Integration's agreed M0
  runtime profile.
- Branch/worktree: `cognitive-navigation`, permitted Module3 worktree; base
  `990f3e2b1055d514f3662485c1a4009934c76e58`; result is the single commit
  containing this entry.
- Changed: Grid localizer parameters now begin with the resolved map YAML
  parameter source and retain only minimal overrides; the LaserScan converter
  uses official `input_qos=SENSOR_DATA`; the R5 Integration launch argument is
  now `runtime_profile:=estimated_m0`. No Grid manager, Nav2, Integration, or
  Module2 implementation changed.
- Launch smoke: isolated installed vendor components on domains 186/187 loaded
  `v6_kujiale_isaacgen_v1.pgm` at `0.05 m` with origin
  `[-5.14, -6.52, 0.0]` and no constructor image failure. `/scan` publisher
  and converter subscriber were both best-effort/volatile. One synthetic scan
  produced a matching five-ray `/flatscan`; its publisher and Grid subscribers
  were reliable/volatile. Logs: `/tmp/v6_map_qos_smoke.wpWho2`.
- Validation: mapping tests **4 passed**; runner tests **4 passed**; `bash -n`
  and `git diff --check` passed; clean `/opt/ros/jazzy` build/install at
  `/tmp/v6_map_qos_repair_final.p23nr1`; installed mapping tests/linters
  **15 passed, 0 failures**.
- Verdict: **PASS (code/test/build/isolated launch smoke only)**. No Isaac Sim,
  Nav2, full-overlay localization result/TF, five-leg navigation, evidence, or
  qualification was run. A controlled-SIGINT-only existing manager
  double-shutdown traceback was recorded but did not affect the probes.
- Remaining: build the fresh combined Integration/Module3 overlay after the
  concurrent `estimated_m0` implementation lands, then verify the complete
  Grid/status/TF/navigation chain.

## 2026-08-24 — V6-GRID first canonical live attempt stopped at Isaac startup

- Goal: run Phase 1B on an actual empty Kujiale stack and, only after it
  passed, one engineering-pilot `G1 reset → G2 → G3 → G4 → G5 → G1` loop.
- Pins: Module3 `42a222bb088b3184d2a99399979bd1a6e3678db7`, Integration
  `2578366c350fee741ca2e97cd846d5741b48eb68`, Module2 metadata-only
  `c18bd9ea7c69b4cc44e4226a7e37d6e1b803de30`; all fixed mains and expected
  tracked-clean/untracked counts passed before launch.
- Runtime: canonical session, snapshot
  `/tmp/v6_phase1_combined_repreflight.sOYIbk`, dedicated empty domain `209`,
  one requested episode only (index 0 / seed 7201). The unrelated
  `odom_static` remained alive on domain 141.
- Result: **FAIL at Isaac startup**. Before Kit started, the snapshot Isaac
  entrypoint required the live Integration worktree's
  `ros2_ws/install/.../cognitive_obstacle_array__struct.hpp`, which does not
  exist. The same header is present in the supplied snapshot
  `i_src/ros2_ws/install_r5`, and the prescribed source order resolves the
  interfaces package there.
- First error layer: the session sources the immutable snapshot overlays, but
  the canonical V6 wrapper exports a live-worktree `BIO_NAV_INTEGRATION_SETUP`
  and the Isaac underlay validator accepts only that live-worktree root.
- Evidence: `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_grid_phase1_20260823T163229Z/`
  (`logs/isaac.log`, `conclusion.md`, `STOP.md`, and `provenance/`).
- Boundary: Isaac sensor topics, Phase 1B interface checks, reset, recorder,
  and every route goal were **not run**. Phase 1C was therefore not authorized.
  This is neither engineering success nor formal qualification.
- Cleanup: only owned process groups were stopped; domain 209 was empty after
  cleanup and the domain-141 process was preserved.
- Next: a fresh coder must bind the canonical Isaac path to the supplied
  snapshot Integration install while retaining fail-closed allowed-root
  validation, then build a fresh combined snapshot and rerun Phase 1B before
  any goal.

## 2026-08-24 — V6-GRID snapshot Integration underlay repair

- Goal: repair only the canonical pre-Kit underlay failure recorded in
  `v6_grid_phase1_20260823T163229Z`; no runner refactor or ROS package change.
- Branch/worktree: `cognitive-navigation`, permitted Module3 worktree; base
  `38884481b12d58ec6c01b3adc5161d951fce2e06`; result is the single commit
  containing this entry.
- Changed: the R5 session resolves and exports snapshot Integration
  root/install/setup plus snapshot Module3 local setup. `common.sh` uses those
  selected roots for package/header validation, clears inherited overlay
  variables, and sources `/opt -> snapshot Integration -> snapshot Module3`.
  Missing snapshot setup/header data fails before Kit without live fallback.
- Tests: deterministic valid-snapshot/stale-live and missing-snapshot/valid-live
  cases **2 passed**; complete runtime-script file **35 passed**; five involved
  scripts passed `bash -n`.
- Preserved-snapshot probe: current `common.sh` plus
  `/tmp/v6_phase1_combined_repreflight.sOYIbk` resolved Integration bridge and
  interfaces to snapshot `i_src/install_r5`; inspected AMENT/CMAKE/LD/PYTHON
  paths contained only snapshot Module3, snapshot Integration, and `/opt`.
- Verdict: **PASS (code/test/pre-Kit shell probe only)**. Isaac Sim, ROS graph,
  Phase 1B, reset, route dispatch, navigation, and qualification were not run.
- Next: build a fresh combined snapshot containing this commit, confirm an
  empty ROS domain, then rerun one episode (`R5_EPISODE_INDICES=0`,
  `R5_EPISODE_SEEDS=7201`). Phase 1B must pass before any five-leg dispatch.

## 2026-08-24 — V6-GRID live retry stopped at ignored Jackal asset dependency

- Goal: after the snapshot-underlay repair, run actual Phase 1B and only on its
  PASS dispatch one engineering-pilot `G1 reset -> G2 -> G3 -> G4 -> G5 -> G1`
  loop.
- Pins: Module3 `a1232a3cc9f25e9a7ece5dcf64a3a4aa9456fcda`, Integration
  `2578366c350fee741ca2e97cd846d5741b48eb68`, Module2 metadata-only
  `c18bd9ea7c69b4cc44e4226a7e37d6e1b803de30`; fixed mains and expected
  tracked-clean/untracked counts passed.
- Snapshot: `/tmp/v6_phase1_combined_retry.5gqVsI`, made only with
  `git archive HEAD` from the permitted Integration and Module3 worktrees.
  Integration's 2 packages and Module3's explicit 13-package stable/M0 closure
  built successfully; fusion and RF2O were excluded. Pre-Kit `common.sh`
  validation resolved only `/opt`, snapshot Integration, and snapshot Module3.
- Runtime: canonical session on confirmed-empty domain `210`, episode index
  `0`, seed `7201`; NAS evidence:
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_grid_phase1_20260823T165127Z/`.
- Result: **FAIL at Isaac scene composition**. Kit completed startup, then
  `SceneComposer` rejected unresolved dependency
  `isaac_sim/assets/robots/jackal/source/jackal_original.usd` in the strict
  snapshot. The file exists in the allowed live worktree but is untracked and
  ignored by `source/.gitignore:2:*.usd`, so it is absent from `git archive`.
  The reviewer did not copy it into the declared strict snapshot or retry.
- Boundary: Phase 1B did not start; no sensor, EKF, FlatScan, Grid acceptance,
  TF/Nav2, frequency/stamp/frame/QoS, correction/latency, or initial GT-error
  evidence exists. Phase 1C was not authorized; no reset/goal, rosbag, motion
  metric, or visual exists. This is an engineering live startup failure, not
  navigation success or formal qualification.
- Cleanup: only owned groups were stopped; domain 210 was empty afterward and
  the unrelated domain-141 `odom_static` was preserved.
- Next: make the required Jackal USD reproducibly available from an authorized
  committed/snapshot source, then create another strict snapshot and rerun
  Phase 1B before any route goal.

## 2026-08-24 — V6-GRID strict-snapshot Jackal asset materialization repair

- Goal: repair only the ignored Jackal runtime dependency reproduced in
  `v6_grid_phase1_20260823T165127Z`, without tracking NVIDIA binaries or adding
  another importer.
- Branch/worktree: `cognitive-navigation`, permitted Module3 worktree; base
  `c5d7cb8d8989e9e8928fc5fd8f4ffb4b9b1c21`; result is the single commit
  containing this entry.
- Changed: the canonical session requires an explicit absolute operator-selected
  `ISAAC_ASSET_ROOT`. Once per snapshot/session and before Isaac, it invokes the
  existing archived `scripts/import_assets.sh`, then its existing `--check`
  mode. Import and check failures stop before Kit. Asset root/status are
  recorded in `run.yaml`, `logs/asset_materialization.log`, STOP, and the run
  contract with `git_contains_runtime_asset_binaries=false`.
- Focused validation: complete asset-path/import and runtime-script tests **43
  passed**; `bash -n` passed. Fake temporary archived-importer tests cover the
  full three-destination closure, missing root/source rejection, snapshot-only
  writes, and idempotent rerun.
- Actual-root pre-Kit probe: strict index-tree archive
  `/tmp/v6_asset_prekit.LU45Nm` initially lacked the ignored files, then the
  existing importer/check materialized both Jackal source layers and the third
  configuration destination from
  `/home/lyb/isaacsim_assets/Assets/Isaac/6.0`.
  `dependency_report(jackal_nav.usda).unresolved` was empty; no Kit started.
- Verdict: **PASS (code/test/pre-Kit asset inspection only)**. No Isaac Sim,
  ROS graph, Phase 1B/1C, reset, navigation, visual evidence, engineering
  success, or qualification was run. The previous live FAIL remains the latest
  runtime result until a fresh combined snapshot is rerun.

## 2026-08-24 — V6-GRID asset-materialized live retry stopped before reset

- Goal: use the repaired canonical asset path for actual Phase 1B and dispatch
  one `G1 reset -> G2 -> G3 -> G4 -> G5 -> G1` engineering loop only after
  Phase 1B passed.
- Pins: Module3 `0df8f131b6226c622f8acbea2f214bfd4a2e75e3`, Integration
  `2578366c350fee741ca2e97cd846d5741b48eb68`, Module2 metadata-only
  `c18bd9ea7c69b4cc44e4226a7e37d6e1b803de30`; fixed mains, branch, tracked
  cleanliness, and expected untracked top-level counts passed.
- Snapshot/build: `/tmp/v6_phase1_combined_asset_live.w0TLP3`, made only from
  the permitted two `git archive HEAD` sources. Integration 2 packages and
  Module3's stable/M0 13-package closure built; fusion/RF2O were excluded.
  Pre-Kit Integration IDL/package prefixes and all inspected overlay paths were
  snapshot-only plus `/opt`.
- Asset/Kit: canonical import and `--check` passed from explicit root
  `/home/lyb/isaacsim_assets/Assets/Isaac/6.0`; all three manifest
  destinations were verified inside the snapshot. Kit and the Kujiale scene
  started, with real sensor, FlatScan, wheel/EKF odometry, and TF traffic.
- Runtime: empty domain `211`, episode index `0`, seed `7201`; NAS evidence:
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_grid_phase1_20260823T172000Z/`.
- Result: **FAIL in Phase 1B before reset**. The Grid manager publishes no
  startup status; its first status follows relocalize. The episode's pre-reset
  readiness requires `localization_status_seen` before calling reset, while
  reset is what causes Integration to call relocalize. The Nav2 activation
  gate then reached its 120-second fail-closed deadline with
  `latest=0 state=none` and shut down navigation.
- Evidence boundary: 256.461-second/112,748-message MCAP; 4,353
  `odom->base_link` transforms; zero Grid status, NVIDIA result, `map->odom`,
  reset event, route goal, route progress, or true collision. Phase 1C was not
  authorized and no spatial/path/costmap visual was meaningful. See
  `logs/navigation.log`, `conclusion.md`, `STOP.md`, and
  `review/phase1b_bag_metrics.json`.
- Cleanup/claim: only owned groups stopped; domain 211 was empty afterward and
  domain 141 was preserved. This is an engineering live interface failure, not
  code-test success, navigation success, or formal qualification.
- Next: minimally break the pre-reset status/reset dependency while retaining
  post-reset WAITING/ACCEPTED, full TF, active Nav2, and reset-gate release
  before G2; then rebuild a fresh strict snapshot and rerun Phase 1B.

## 2026-08-24 — V6-GRID pre-reset status/reset cycle repair

- Goal: repair only the actual startup-order blocker from
  `v6_grid_phase1_20260823T172000Z`; no Grid manager, ActivationGate, Nav2
  config, Integration, Module2, asset, map, runner-shell, or timeout change.
- Branch/worktree: `cognitive-navigation`, permitted Module3 worktree; base
  `5c7b90ed270c333e89168bad77d8ab08429e0128`; result is the single commit
  containing this entry.
- Changed: pre-reset readiness now admits reset from real sensor/estimated-odom
  and reset/Grid endpoints without requiring a Grid status sample, Nav2, TF,
  navigation graph, or route subscriber. The dispatcher checks but never calls
  `/bio_nav/relocalize`; Integration remains its sole caller. Reset remains
  exactly once. Post-reset G2 authorization still requires a newer WAITING ->
  same-generation ACCEPTED with matching finite correction, a fresh
  post-accept full-TF/Nav2/route/publisher epoch, and matching ResetStopGate
  release. Stale ACCEPTED/TF state is rejected.
- Validation: focused V6 formal/runtime tests **33 passed, 28 deselected**;
  `/opt/ros/jazzy`-only isolated `robot_experiments` build at
  `/tmp/v6_reset_order_isolated.rwsJIm` **1 package finished**; installed CLI
  dry probe and installed-package one-reset/five-XY-leg synthetic sequence
  passed. No cache or bytecode was used.
- Verdict: **PASS (code/test/build/synthetic only)**. No Isaac Sim, live ROS
  graph, relocalization, Phase 1B/1C, route goal, navigation, visual evidence,
  engineering success, or qualification was run.
- Next: build a fresh strict combined snapshot, confirm domain 212 is empty,
  and rerun only episode 0 / seed 7201 with the canonical asset root. STOP
  unless the single reset and complete new-generation
  Grid/correction/TF/ActivationGate/Nav2 chain precede G2.

## 2026-08-24 — Default-OFF wheel yaw-disagreement guard candidate C

- Goal: implement only the single-bag counterfactual candidate C for wheel
  forward-speed bounding during confirmed wheel/corrected-IMU yaw-sign
  disagreement, without promoting it into the canonical Phase-1 default.
- Change: `robot_odometry` now has a default-OFF guard. Enabled entry is three
  consecutive opposite-sign `|wz|>=0.10` samples with corrected IMU age
  `0..0.05 s`; active signed `|vx|` is capped at `0.05 m/s`, raw wheel `wz`
  remains unchanged, and three sign-agree/unusable/below-`0.02` samples clear.
  Unusable IMU is fail-open per sample; reset clears detector/cache. The OFF
  path creates no IMU subscription. A single bool launch override is threaded
  through wheel odometry and `ros_stack`; canonical runners remain unchanged.
- Validation: focused robot-odometry/EKF/launch tests **60 passed**; modified
  Python lint **7 files clean**; isolated `/tmp` build **2 packages finished**.
  Default-OFF installed-node inspection showed no IMU or TF endpoint.
- Exact replay: initially empty domain 225, 1x replay of only clock, joint,
  corrected IMU, and reset topics from exact failed MCAP. Actual enabled node
  produced `4626/4626` unique ordered stamps and zero full-bag output mismatch
  versus offline C. One 83-sample episode ran at
  `56.666666666–58.033333333 s`; pivot absolute forward integral was
  `0.14653831432307307 m` versus baseline `0.5206281441032308 m`; straight
  scale was unchanged at `0.9982402680931219`; raw angular velocity and
  baseline covariances were unchanged.
- Evidence: `/tmp/yaw_guard_replay_exact_20260824T0437/replay_metrics.json`,
  recorded output MCAP in the sibling `guard_output/`, and
  `docs/handoff/V6_WHEEL_YAW_DISAGREEMENT_GUARD_20260824.md`.
- Verdict: **PASS (code/unit/build/exact replay only), DEFAULT OFF**. No Isaac,
  Nav2, controller, new navigation run, recurrence, engineering navigation
  success, or formal qualification is claimed. A separate short live
  diagnostic is still required before any promotion decision.
