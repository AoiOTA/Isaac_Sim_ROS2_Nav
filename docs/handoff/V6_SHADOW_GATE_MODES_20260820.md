# V6 shadow activation gate and causal modes handoff

Date: 2026-08-20

## Scope and result

- Worktree: `worktrees/cognitive-navigation/bio_nav_module3`
- Branch: `cognitive-navigation`
- Starting HEAD: `faabaa00c2a1a5dbe7d088768c49211143c4f405`
- Goal: keep zero-seed shadow enrollment alive without activating Nav2, and
  separate the C local-planning arms from the D cognitive-graph experiment.
- Result: **PASS for implementation, focused tests, and package build only**.
- ROS, DDS, Nav2, Isaac Sim, navigation, visual evidence, and qualification
  were not started.

## Implemented contracts

- `nav2_activation_gate` now accepts
  `startup_timeout_policy=fail_closed|wait_for_seed`. The default remains
  bounded `fail_closed` with a 120 s launch/config deadline. `wait_for_seed`
  reports the deadline once, continues normal missing-requirement diagnostics,
  and leaves Nav2 inactive until the unchanged readiness contract succeeds.
  Duplicate managed nodes, recovery failures, and other gate errors remain
  fatal.
- `navigation_bringup.launch.py` and `ros_stack.launch.py` expose and forward
  `activation_startup_timeout` and `activation_startup_policy`.
- `scripts/run_v6_kujiale_low_obstacles.sh shadow [M0|M1|M2|M3]` is the
  reproducible enrollment entry. It defaults to local arm M1, fixes the graph
  to GVG, selects RViz initial-pose input, and uses `wait_for_seed` without
  bypassing the localization/Nav2 stack.
- M0--M3 now control only the Module2 local-planning ablation:
  M0=`layer off, critic off, Module2 disabled`; M1=`shadow, shadow, enabled`;
  M2=`active, off, enabled`; M3=`active, active, enabled`.
- `cognitive_graph_mode` is validated independently. The C entry
  `... ros [M0|M1|M2|M3]` fixes it to `gvg` and rejects graph overrides. The D
  entry `... ros-d shadow|hybrid|primary [M0|M1|M2|M3]` passes the requested
  graph arm explicitly.
- The existing last-precedence exact-node Nav2 overlay is unchanged: M0--M3
  still replace only the cognitive layer/critic modes and preserve the A21
  critic list. Non-V6 profiles retain the caller-provided legacy
  `module2_enabled` behavior.

## Changed files

- `ros2_ws/src/robot_bringup/robot_bringup/activation_gate.py`
- `ros2_ws/src/robot_bringup/robot_bringup/mode_contract.py`
- `ros2_ws/src/robot_bringup/config/activation_gate.yaml`
- `ros2_ws/src/robot_bringup/config/modes.yaml`
- `ros2_ws/src/robot_bringup/launch/ros_stack.launch.py`
- `ros2_ws/src/robot_bringup/launch/navigation_bringup.launch.py`
- `ros2_ws/src/robot_bringup/test/test_activation_gate.py`
- `ros2_ws/src/robot_bringup/test/test_mode_contract.py`
- `ros2_ws/src/robot_bringup/test/test_runtime_scripts.py`
- `ros2_ws/src/robot_navigation/test/test_v6_cognitive_profile.py`
- `scripts/run_v6_kujiale_low_obstacles.sh`

## Validation

- `bash -n scripts/run_v6_kujiale_low_obstacles.sh`: PASS.
- `python3 -m py_compile` on the changed gate, mode-contract, and launch Python
  files: PASS.
- Focused pytest for activation gate, mode/launch contract, runtime wrapper,
  V6 cognitive profile, and Nav2 config: `80 passed`.
- Clean pure-Jazzy temporary build, selecting only `robot_bringup` and ignoring
  unselected workspace exec dependencies: `1 package finished` at
  `/tmp/bionav_v6_shadow_clean_build.5K1BKG`.
- `git diff --check`: PASS.
- An earlier build invocation failed because `--log-base` was placed after the
  `build` verb. A corrected retry exposed inherited stale overlay paths and
  unbuilt selected-prefix dependencies. No result from those attempts was
  used; the reported build result is from the clean `env -i` run above.

## Remaining live risk

- Static tests do not prove that a real zero-seed AMCL process continues
  diagnostics beyond 120 s, that RViz reseeding later activates Nav2, or that
  the process-exit handler never fires in this path.
- M0--M3 Module2 enable state, final live Costmap/MPPI parameters, and D graph
  propagation still require an authorized ROS/Nav2 runtime check.
- No causal campaign or formal qualification conclusion is claimed.
