# V6 Kujiale clearance layout R2 handoff (2026-08-24)

## Result and boundary

- Verdict: **PASS for offline USD composition, collision OMap generation,
  deterministic GVG generation, five-leg footprint feasibility, canonical
  Phase-1 binding, focused tests, changed-package build, CLI dry probe, and
  visual inspection**.
- Live Isaac/ROS/Nav2 navigation and formal qualification: **not run and not
  claimed**.
- Identity: `v6_kujiale_clearance_r2`. The original scene, original generated
  baseline, and `v6_kujiale_clearance_r1` artifacts remain unchanged.

## Scene and collision map

- Overlay:
  `isaac_sim/assets/environments/v6_kujiale_clearance_r2/kujiale_0026_A_to_B_door_open.usd`.
  It sublayers the same required read-only original and moves exactly these ten
  top-level prims by `-0.48 m` USD x, with y/z/rotation/scale inherited:
  `table_0000=2.0041237336013427`,
  `tablecloth_0004=2.0198628175599715`,
  `unknown_0001=1.9756577602812007`,
  `ornament_0029=1.9255963096535003`,
  `vase_0001=1.9319162291121375`,
  `flower_0001=1.9311050112131274`,
  `menorah_0000=2.0761237563256993`,
  `menorah_0001=1.9403277101840360`,
  `book_0000=2.0755727424629202`, and
  `book_0001=2.0745784912153167`.
  No chandelier, chair, or other prim is overridden.
- Direct PXR composition found 12,283 prims, the exact ten translations, 19
  preserved internal assembly bbox overlaps, and zero 3D bbox overlap with
  the nearby cabinet/sofas/chandelier/television/wall set. The external source
  USD retained the same size, mtime, and inode before/after generation.
- Collision-derived occupancy:
  `data/maps/occupancy/v6_kujiale_clearance_r2.{pgm,yaml}`; resolution
  `0.05 m`, shape `154x248`, origin `[-5.14,-6.52]`. Counts are occupied
  `11090`, free `17134`, unknown `9968`.
- OMap slice remains `z=0.30..0.37 m`. Table/tablecloth and borderline
  `unknown_0001` affect the collision map; the seven higher tabletop items do
  not. This expected 2D omission is not treated as physical removal: all ten
  collision-enabled assembly prims move in the composed USD.

## Graph, route, and recorded-contact regression

- GVG:
  `ros2_ws/src/robot_route_planner/config/v6_kujiale_clearance_r2_gvg_v1.{geojson,support_map.json,summary.json}`;
  graph id `v6_kujiale_clearance_r2:gvg_v1`, revision 1, 25 canonical nodes,
  48 directed canonical edges, 181 support nodes, 358 support edges, one
  connected component.
- All five directed support legs are connected and pass the exact directional
  footprint sweep: G1→G2 `12.4856 m`, G2→G3 `10.2606 m`, G3→G4 `3.0512 m`,
  G4→G5 `4.1631 m`, and G5→G1 `9.0271 m`.
- Actual G1→G2 throat route is at `x=-0.165 m`, `y=2.005..2.605 m`; center
  clearance is minimum `0.5295349359512329 m`, median/maximum
  `0.550000011920929 m`.
- Recorded R1 contact reconstruction uses the published estimated pose plus
  observed error vector. Its conservative 3D robot AABB overlaps stale R1
  `unknown_0001`; after the full R2 move, clearances are `0.1673803572 m` to
  the full assembly, `0.1892881092 m` to the moved table, and
  `0.4366810166 m` to the cabinet. This is an offline regression, not a new
  collision or navigation run.

## Canonical Phase-1 binding

- The enabled static manifest and direct runner/session paths now select the
  R2 overlay, map, spawn manifest, and GVG. Dynamic and appearance remain
  disabled later-pilot metadata aligned to R2.
- Runtime remains `estimated + grid + stable + M0 + Module2 off + GVG`, with
  RF2O off, dynamic/appearance/low-obstacle actors off, XY-only route goals,
  unchanged footprint/inflation/Collision Monitor, and yaw-disagreement guard
  default OFF.

## Generation commands

```bash
cd /home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3
ISAAC_PYTHON=/home/lyb/miniconda3/envs/isaacsim/bin/python \
MAPPING_OUTPUT_ROOT=/tmp/v6_clearance_r2_map \
/home/lyb/miniconda3/envs/isaacsim/bin/python \
  isaac_sim/tools/rivermark_occupancy_generate.py \
  --headless --output-root /tmp/v6_clearance_r2_map \
  --map-version v6_kujiale_clearance_r2 \
  --map-stem v6_kujiale_clearance_r2 \
  --usd isaac_sim/assets/environments/v6_kujiale_clearance_r2/kujiale_0026_A_to_B_door_open.usd \
  --bounds-min-x -0.66 --bounds-min-y -7.08 \
  --bounds-max-x 9.04 --bounds-max-y 7.32 \
  --crop-cells 20 20 20 20 \
  --flip-origin-x 2.9 --flip-origin-y -0.2 \
  --seed-x 2.9 --seed-y -0.2 --seed-ground-z 0.0 \
  --mapping-height-m 0.30 \
  --minimum-z-offset-m 0.0 --maximum-z-offset-m 0.07 \
  --seed-offsets-m 0.3 0.0 -0.3 0.0 0.0 0.3 0.0 -0.3

PYTHONPATH=ros2_ws/src/robot_route_planner \
PYTHONDONTWRITEBYTECODE=1 \
python3 -m robot_route_planner.cli \
  --map data/maps/occupancy/v6_kujiale_clearance_r2.yaml \
  --defaults /tmp/v6_clearance_engineering_defaults.yaml \
  --geojson /tmp/v6_kujiale_clearance_r2_gvg_v1.geojson \
  --mapping /tmp/v6_kujiale_clearance_r2_gvg_v1_support_map.json \
  --summary /tmp/v6_kujiale_clearance_r2_gvg_v1_summary.json
```

## Validation and visual

- Focused Python suite: **95 passed, 15 skipped** only because system Python
  lacks PXR. The four R1/R2 composition and recorded-contact tests passed
  separately under Isaac Python: **4 passed**.
- Clean `/opt/ros/jazzy`-only build at
  `/tmp/v6_clearance_r2_isolated.IM4tiE`: **2 changed packages finished**
  (`robot_experiments`, `robot_route_planner`). Installed static-manifest CLI
  dry probe returned dispatch false, Grid/stable/M0/Module2-off/GVG/RF2O-off,
  and `NOT_QUALIFIED` as intended.
- Direct GVG regeneration matched all three tracked artifacts byte-for-byte;
  generated PGM/YAML matched the tracked map byte-for-byte. Three changed
  shell scripts passed `bash -n`.
- Inspected normal map preview:
  `/tmp/v6_clearance_r2_map/v6_kujiale_clearance_r2/v6_kujiale_clearance_r2.png`.
  Inspected combined map/GVG/assembly visual:
  `/tmp/v6_kujiale_clearance_r2_map_gvg_assembly.png`.

## Exact next live command

After a fresh strict combined Integration/Module3 snapshot containing this
commit is built and domain 229 is confirmed empty, run exactly one engineering
episode:

```bash
cd /home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3
RUN_DIR="/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_grid_phase1_clearance_r2_$(date -u +%Y%m%dT%H%M%SZ)"
SNAPSHOT_ROOT="/absolute/path/to/fresh_combined_phase1_snapshot"
ISAAC_ASSET_ROOT=/home/lyb/isaacsim_assets/Assets/Isaac/6.0 \
R5_DOMAIN_ID=229 R5_EPISODE_INDICES=0 R5_EPISODE_SEEDS=7201 \
  ./scripts/run_v6_kujiale_low_obstacles.sh session "${RUN_DIR}" "${SNAPSHOT_ROOT}"
```

Phase 1B and the existing fail-closed Grid/reset/TF/Nav2 chain must pass before
G2. This amendment makes no navigation or formal-qualification claim.
