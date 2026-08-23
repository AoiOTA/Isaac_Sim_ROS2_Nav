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

## Actual-launch blocker repair amendment (2026-08-23)

- The R5 session now passes `runtime_profile:=estimated_m0` to the Integration
  V6 launch, replacing only the former cosmetic Integration-side
  `module2_enabled:=false` argument. Module3's own Phase-1 M0/Module2-false
  launch contract is unchanged.
- Direct runtime-script tests **4 passed** and `bash -n` passed. The matching
  Integration profile implementation is concurrent and was not inspected or
  launched here.
- The same amendment also repaired the Module3 vendor map parameter source and
  `/scan` subscriber QoS. Isolated vendor component loading, endpoint QoS, and
  synthetic LaserScan-to-FlatScan delivery passed; see the Grid core handoff.
- Remaining dependency: build a fresh combined Integration/Module3 overlay and
  verify that `estimated_m0` is accepted together with Grid status ordering,
  relocalize count, sole `map->odom` ownership, full TF, and five-leg motion.

## Snapshot Integration underlay repair amendment (2026-08-24)

- Repaired only the reproduced pre-Kit failure from
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_grid_phase1_20260823T163229Z`.
  A canonical session now resolves `SNAPSHOT_ROOT` once and exports that
  snapshot's Integration source root, `install_r5`, setup file, and the
  snapshot Module3 `local_setup.bash` to the Isaac entrypoint.
- `common.sh` validates package prefixes and required IDL headers strictly
  inside the selected Integration install. It clears inherited ROS, library,
  Python, and C/C++ include overlay variables before sourcing
  `/opt/ros/jazzy -> snapshot Integration -> snapshot Module3`; a snapshot
  invocation cannot fall back to or mix in the live Integration worktree.
  The live-worktree default remains for non-snapshot legacy invocations.
- Deterministic fake-overlay tests cover a missing/stale live root with a valid
  snapshot and the inverse case. They also assert exact source order, emitted
  AMENT/CMAKE/LD/PYTHON paths, and that the fake live setup was not read.
- Validation: focused new tests **2 passed**; complete runtime-script file
  **35 passed**; five directly involved shell scripts passed `bash -n`.
  A no-Kit probe using the preserved snapshot resolved both Integration
  packages and every inspected overlay variable under snapshot `i_src`,
  snapshot `m3_src`, and `/opt` only.
- Boundary: this was code/test/pre-Kit shell validation only. No Isaac Sim,
  ROS graph, Phase 1B interface smoke, reset, route goal, navigation evidence,
  engineering success, or formal qualification was run or claimed.

After building a fresh combined snapshot containing this commit, rerun exactly
one episode with a confirmed-empty domain:

```bash
cd /home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3
RUN_DIR="/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_grid_phase1_$(date -u +%Y%m%dT%H%M%SZ)"
SNAPSHOT_ROOT="/absolute/path/to/fresh_combined_phase1_snapshot"
R5_DOMAIN_ID=209 R5_EPISODE_INDICES=0 R5_EPISODE_SEEDS=7201 \
  ./scripts/run_v6_kujiale_low_obstacles.sh session "${RUN_DIR}" "${SNAPSHOT_ROOT}"
```

Phase 1B must pass before the canonical dispatcher may send the five route
goals; the repair itself does not satisfy that live dependency.

## Strict-snapshot Jackal asset materialization amendment (2026-08-24)

- Repaired only the next reproduced startup blocker from
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_grid_phase1_20260823T165127Z`.
  A canonical session now requires the operator to set an explicit absolute
  `ISAAC_ASSET_ROOT`; there is no live-worktree fallback.
- Before starting Isaac once per session, the driver calls the existing
  archived `m3_src/scripts/import_assets.sh` with the archived Jackal manifest,
  then calls its existing `--check` mode. Destinations therefore land only in
  the selected snapshot. Repeating the session-side import is idempotent.
- Missing/non-absolute roots, unavailable manifest sources, import failures,
  and check failures stop before Kit. `run.yaml`, the materialization log,
  STOP input, and the run contract record the selected root and status while
  stating that the runtime binaries are not contained in Git.
- Focused validation: asset-import plus complete runtime-script tests **43
  passed**; involved shell syntax passed. A bounded strict index-tree archive
  at `/tmp/v6_asset_prekit.LU45Nm` initially omitted both ignored source
  layers, imported and checked all three manifest destinations from
  `/home/lyb/isaacsim_assets/Assets/Isaac/6.0`, and reported
  `dependency_report.unresolved=[]` for `jackal_nav.usda` (the allowed
  `OmniPBR.mdl` diagnostic remained informational).
- Boundary: code/test/pre-Kit asset inspection only. No Kit, Isaac Sim, ROS
  graph, Phase 1B, reset, goal, navigation, visual evidence, engineering
  success, or qualification was run or claimed.

After building a fresh combined snapshot containing this amendment, the exact
one-episode rerun is:

```bash
cd /home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3
RUN_DIR="/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_grid_phase1_$(date -u +%Y%m%dT%H%M%SZ)"
SNAPSHOT_ROOT="/absolute/path/to/fresh_combined_phase1_snapshot"
ISAAC_ASSET_ROOT=/home/lyb/isaacsim_assets/Assets/Isaac/6.0 \
R5_DOMAIN_ID=211 R5_EPISODE_INDICES=0 R5_EPISODE_SEEDS=7201 \
  ./scripts/run_v6_kujiale_low_obstacles.sh session "${RUN_DIR}" "${SNAPSHOT_ROOT}"
```

The operator must confirm that the selected domain is empty. Phase 1B still
must pass before any route goal is authorized.
