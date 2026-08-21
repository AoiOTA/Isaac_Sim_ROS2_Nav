# V6 Rivermark occupancy-only estimated bringup

Date: 2026-08-21

## Result

**PASS (implementation/build/unit only).** Rivermark navigation can now select
an explicit `localization_map_contract:=occupancy_only` without inventing a
SLAM Toolbox posegraph. The exception is fail-closed: it is valid only for
localization/navigation with AMCL ownership, an existing occupancy YAML, and
an existing independent route GeoJSON. The default remains
`posegraph_bundle`, so Kujiale and legacy saved-map entrypoints still require
the serialized `.posegraph`/`.data` pair and manifest.

The six paired engineering-pilot entrypoints are:

```text
scripts/run_v6_rivermark.sh isaac static
scripts/run_v6_rivermark.sh ros static
scripts/run_v6_rivermark.sh isaac dynamic
scripts/run_v6_rivermark.sh ros dynamic
scripts/run_v6_rivermark.sh isaac appearance [profile]
scripts/run_v6_rivermark.sh ros appearance [profile]
```

They bind the Rivermark USD, `rivermark_start`, selected occupancy map,
selected route GeoJSON, goal YAML, final static/dynamic configurations and
appearance profiles. Isaac is realistic-sensor mode with Isaac structural TF,
RGB-D and passive ground truth enabled. ROS is estimated-state navigation with
wheel/IMU/RF2O EKF input, AMCL as the sole `map->odom` owner, M3, PRIMARY and
RViz disabled. The route GeoJSON is passed to the existing Nav2 Route Server;
no synthetic posegraph or replacement graph is created.

## Files

- `scripts/run_ros.sh`
- `scripts/run_v6_rivermark.sh`
- `ros2_ws/src/robot_bringup/robot_bringup/mode_contract.py`
- `ros2_ws/src/robot_bringup/launch/ros_stack.launch.py`
- `ros2_ws/src/robot_bringup/launch/navigation_bringup.launch.py`
- `ros2_ws/src/robot_bringup/launch/localization_bringup.launch.py`
- `ros2_ws/src/robot_bringup/config/modes.yaml`
- focused `robot_bringup` tests

## Validation

- `bash -n scripts/run_ros.sh scripts/run_v6_rivermark.sh`: PASS.
- Changed Python `py_compile`: PASS.
- Complete `test_mode_contract.py` plus new runtime-wrapper tests: `32 passed`.
- Full `test_mode_contract.py` + `test_runtime_scripts.py`: `60 passed`, one
  unrelated pre-existing failure because `scripts/run_v6_formal_episode.sh`
  uses `set -euo pipefail` while the global script-style test expects
  `set -Eeuo pipefail`; this file is outside this task's write ownership.
- Isolated `robot_bringup` build: PASS at
  `/tmp/v6_rivermark_bringup.iJSG4n`.
- `git diff --check`: PASS before commit.

## Not run / remaining risk

No ROS, Isaac Sim, Nav2, route goal, engineering pilot, evidence capture, scene
freeze, or qualification campaign was run. The first live step must start the
matching Isaac entrypoint, wait through the Rivermark cold load, then start the
matching ROS entrypoint and verify the real AMCL/EKF/Route/PRIMARY chain before
dispatching the engineering goal. This change makes that run possible; it does
not claim that the run has passed.
