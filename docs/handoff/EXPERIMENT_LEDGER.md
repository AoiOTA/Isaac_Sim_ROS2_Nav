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
