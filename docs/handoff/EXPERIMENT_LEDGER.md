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
