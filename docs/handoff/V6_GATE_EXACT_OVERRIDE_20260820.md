# V6 activation-gate exact override amendment

Date: 2026-08-20

## Scope and result

- Branch/worktree: `cognitive-navigation`;
  `worktrees/cognitive-navigation/bio_nav_module3`.
- Starting HEAD: `cb50199f3a7fd8ae61c6060c8addd938a2679ea7`.
- Goal: ensure launch-time shadow enrollment values override the exact-node
  defaults in `activation_gate.yaml` without changing normal autonomy defaults.
- Result: **PASS for implementation, real rclpy parameter parsing, pytest, and
  clean package build only**. No ROS graph, Isaac, Nav2, navigation, evidence,
  or qualification campaign was started.

## Amendment

- `ros_stack.launch.py` now writes its four runtime gate values under the exact
  `nav2_activation_gate: ros__parameters:` key and loads that file after the
  default exact-node YAML. This prevents an internally generated `/**`
  wildcard from losing to the default exact-node block on Jazzy.
- Normal autonomy remains `startup_timeout_policy=fail_closed` and
  `initial_pose_source=auto`; shadow enrollment can now apply
  `wait_for_seed` and `rviz` as requested.
- A real Jazzy `rclpy` test loads the default and runtime parameter files into
  a Node named `nav2_activation_gate`. It covers both `fail_closed/auto` and
  `wait_for_seed/rviz`, including distinct timeout and sim-time overrides.

## Validation

- `python3 -m py_compile` for the changed launch and test Python: PASS.
- Focused activation/launch/runtime/V6 pytest: `67 passed`.
- Full `robot_bringup/test` pytest: `214 passed`.
- Clean pure-Jazzy `robot_bringup` build: `1 package finished`, artifacts at
  `/tmp/bionav_gate_exact_build.HtXiti`.
- The first probe run failed because the test probe redeclared Jazzy's
  automatically declared `use_sim_time`; matching the real gate fixed the
  fixture and the rerun passed. The first clean build attempt selected a
  package without ignoring unbuilt workspace exec dependencies; the isolated
  retry ignored all unselected packages and passed.

## Remaining risk

- This proves Jazzy parameter-file parsing and package build, not the live
  120-second shadow behavior, late RViz reseed activation, or navigation.
- The separate shadow-wrapper RF2O default issue was reported to master and is
  outside this amendment's assigned files.
