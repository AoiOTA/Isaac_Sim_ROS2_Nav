# V6-GRID bringup and Nav2 cutover handoff (2026-08-23)

## Result

- Verdict: **PASS for code, focused tests, launch-description probes, and
  isolated build/test**.
- Branch/worktree: `cognitive-navigation` in the permitted Module3 worktree.
- Base HEAD: `749ee7c3128c87d65560b40f5401231413f50bcc`.
- Result commit: the single commit containing this handoff.
- Live ROS graph, Isaac Sim, vendor components, Nav2 motion, TF ownership, and
  qualification: **not run and not verified**.

## Cutover

- Normal localization/navigation wrappers now default to `estimated` plus the
  `occupancy_only` contract. `auto` resolves non-Ideal localization ownership
  to `grid`; `amcl` and `odom_static` are rejected by the formal mode contract.
- `ros_stack.launch.py` passes only the occupancy map and resolved `grid`
  backend to `robot_mapping/localization.launch.py`. Production
  localization/navigation no longer include an initial-pose publisher or pass
  AMCL parameters. Mapping-only legacy initial-pose support remains isolated.
- ActivationGate has no relocalization/reseed client. Integration remains the
  sole caller of `/bio_nav/relocalize`; Bringup observes
  `/bio_nav/localization/status` and `map->odom`. It parses the fixed
  `generation`, `state`, and `accepted` keys, treats WARN-level
  `WAITING_FOR_SCAN`/`WAITING_FOR_RESULT` as normal waiting, and requires an
  `ACCEPTED` generation newer than the pre-reset accepted generation before
  Nav2 activation/resume.
- The formal Nav2 base uses `PositionGoalChecker`, no yaw tolerance,
  no `GoalAngleCritic`, and `use_final_approach_orientation: false`. Because
  the controller is MPPI rather than RotationShim, no invalid
  `rotate_to_goal_heading` parameter was added.
- Formal local/global Costmaps contain only 2D LiDAR obstacle + inflation, and
  static + 2D LiDAR obstacle + inflation respectively. The base and V6 overlay
  contain no depth voxel/STVL or RGB-D PointCloud Costmap source.
- The V6 Phase-1 cognitive overlay remains OFF/shadow. Runtime M2/M3 active
  selections are clamped to shadow so they cannot restore writes during the
  base-loop phase. Ten Attempt21 tests that require direct RGB-D Costmap depth
  were explicitly retired/skipped; their historical generator/profile files
  were not changed.

## Validation

- Source-focused/full suite: **252 passed, 10 skipped** (retired Attempt21
  depth contract), including the ROS ActivationGate reset fixture.
- Mapping localization description tests: included in that suite; the grid
  backend expands to the map server, Isaac ROS component container,
  `grid_localization_tf_manager`, and lifecycle manager, with no AMCL or
  odom-static executable.
- Installed launch argument probe:
  `ros2 launch robot_bringup localization_bringup.launch.py --show-args`;
  defaults reported `estimated`, `occupancy_only`, and `auto` owner.
- Isolated build/test root: `/tmp/v6_grid_bringup_narrow.EXEMuz`.
  Narrow `/opt/ros/jazzy` build: **2 packages passed**. Final colcon result:
  **270 tests, 0 errors, 0 failures, 10 skipped**.
- Modified implementation/test lint: `ament_flake8` and `ament_pep257` PASS;
  `git diff --check` PASS.

An earlier broad `--packages-up-to` build stopped in the unrelated
`bio_nav_fusion` dependency because its current underlay lacked
`bio_nav_interfaces/msg/cognitive_obstacle_array.hpp`. No out-of-scope source
was changed; the authorized packages were subsequently built and tested from
the narrow isolated base paths above.

## Remaining live work

- Verify actual wrapper expansion against the full installed Module3 overlay,
  Isaac ROS component loading/NITROS delivery, status ordering, exact-stamp TF,
  one `map->odom` publisher, and one Integration relocalize call per reset.
- Run the authorized Phase 1B interface smoke, then the empty-house
  G1→G2→G3→G4→G5→G1 loop. This commit is not runtime or qualification evidence.

## Phase-1 reviewer blocker amendment

- Amendment base: `0475a680746039f74dbeb15d4c5f06c52530d79e`.
- After every reset epoch, ActivationGate now requires an observed
  `WAITING_FOR_SCAN` or `WAITING_FOR_RESULT` for a generation newer than the
  accepted floor before `ACCEPTED` for that exact generation can arm the
  gate. Late pre-reset, skipped-generation, stale, rejected, and malformed
  statuses remain blocked. Integration remains the sole relocalize caller.
- The matching `ACCEPTED` status must provide finite `correction_x_m`,
  `correction_y_m`, and `correction_yaw_rad`. Gate compares them with the
  actual planar `map->odom` transform before release. Translation and yaw
  tolerances are independently configurable and default to `0.01` m/rad.
  Status-before-TF and TF-before-status ordering both remain safe; an unchanged
  correction value is allowed when the observed transform numerically matches.
- Launch setup now treats M0 as authoritative: `module2_enabled=false` for
  both `stable` and `v6_low_obstacle_isolation`, even if the launch argument is
  `true`. M1/M2/M3 retain their existing stable-profile argument behavior and
  low-obstacle profile contract. Phase-1 defaults remain `M0` plus `gvg`.
- Validation: focused pure/launch tests **59 passed**; deterministic ROS gate
  fixture **3 passed**; relevant source regression **261 passed, 10 skipped**;
  clean narrow build/install/test at
  `/tmp/v6_gate_m0_repair_clean.ipjnBD` built one package and produced **239
  passed, 10 skipped, 0 failures**. `git diff --check` and implementation
  pep257 passed. These are code/test/synthetic results only.
- Remaining risk: no live graph verified the real transient-local status/TF
  delivery timing. The conditional `bio_nav_fusion` build was not changed or
  resolved; the actual Phase-1 runner must select a profile whose requested
  plugins are buildable before the live smoke.
