# V6-GRID Phase-1 canonical runner handoff (2026-08-23)

## Result

- Verdict: **PASS for code, focused tests, shell argument expansion, and an
  isolated `robot_experiments` build/install**.
- Live ROS graph, Isaac Sim, NITROS, Nav2 motion, TF ownership, and
  qualification: **not run and not verified**.
- Branch/worktree: `cognitive-navigation`, permitted Module3 worktree; base
  `ced458cda229464c2d0f11b06c432a108eab4592`; result is the single commit
  containing this handoff.

## Canonical Phase-1 contract

- Physical reset starts at G1 and may retain its calibrated physical yaw.
  Route goals are exactly `G2, G3, G4, G5, G1`; every goal contains only
  `id/frame_id/x/y`, and the ROS message uses identity orientation as a valid
  protocol placeholder. Orientation is not a completion condition.
- Runtime is fixed to `estimated + grid + stable + M0 + module2=false + gvg`,
  with RF2O OFF, low obstacles OFF, and dynamic actors OFF. `stable` is
  deliberate: it has the Phase-1 PositionGoalChecker/LiDAR-only behavior and
  avoids the unresolved current-IDL `bio_nav_fusion` build dependency.
- The existing `v6_kujiale_isaacgen_v1` occupancy map, spawn manifest, and GVG
  are reused; no map or GVG regeneration is performed.
- Before G2, the dispatcher requires a Grid generation newer than the
  pre-reset accepted floor to emit WAITING and then same-generation ACCEPTED,
  plus Nav2/full TF readiness and the matching ResetStopGate release.
- The Isaac reset bridge no longer constructs a global-pose publisher or seed
  event publisher. Its legacy ResetManager hook is a no-op in the V6-GRID
  active path; reset generation/event and normal estimator/costmap reset hooks
  remain intact.
- Static is the only enabled Phase-1 manifest. Dynamic and appearance
  manifests retain later-pilot intent/episode metadata but cannot dispatch
  Phase 1.
- The R5 session recorder writes directly under the NAS run root and includes
  FlatScan, Grid result/status, odom/TF, route, planner/controller/safety,
  collision, and evaluator-only Ground Truth inputs. Retired localization
  topics are absent.

## Canonical live command

Build the combined Integration and Module3 snapshot overlays first, then run:

```bash
cd /home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3
RUN_DIR="/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_grid_phase1_$(date -u +%Y%m%dT%H%M%SZ)"
SNAPSHOT_ROOT="/absolute/path/to/combined_phase1_snapshot"
./scripts/run_v6_kujiale_low_obstacles.sh session "${RUN_DIR}" "${SNAPSHOT_ROOT}"
```

The snapshot root must contain `m3_src/ros2_ws/install_r5` and
`i_src/ros2_ws/install_r5`. The session owns and stops only the process groups
it starts, on its dedicated `ROS_DOMAIN_ID` (default 173).

## Validation

- Focused `v6_formal` + reset suites: **45 passed**.
- Focused V6 runtime-script suite: **8 passed, 25 deselected**.
- Four owned shell scripts: `bash -n` **PASS**.
- Production-path retired-token grep and `git diff --check`: **PASS**.
- E/F/I-focused Python lint under the package config: **PASS**.
- Clean isolated build/install: `/tmp/v6_phase1_runner_build.SDLQp0`, final
  `robot_experiments` build **1 package finished**. Installed CLI dry probe
  reported Grid, stable, M0, Module2 false, empty room, and the five legs.

## Remaining pre-live dependencies

- Build a fresh combined Integration/Module3 overlay and ensure the
  Integration `estimated_shadow` launch accepts the Grid/M0 arguments used by
  the R5 driver; this amendment did not inspect or modify Integration.
- Verify the full-overlay launch, process/domain isolation, vendor component
  loading, FlatScan/result/status ordering, one Integration relocalize request
  per reset, one `map->odom` publisher, and actual five-leg navigation.
- Do not select `v6_low_obstacle_isolation` until `bio_nav_fusion` builds
  against the current IDL. Dynamic/appearance pilots remain disabled here.
