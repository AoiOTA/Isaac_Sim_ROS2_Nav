# V6 Kujiale clearance layout R1 handoff (2026-08-24)

## Result

- Verdict: **PASS for offline USD composition, collision-derived occupancy,
  deterministic GVG generation, five-leg route/footprint validation, focused
  tests, and runner identity binding**.
- Live Isaac/ROS/Nav2 navigation and formal qualification: **not run and not
  claimed**.
- Identity: `v6_kujiale_clearance_r1`; old
  `v6_kujiale_isaacgen_v1` scene/map/GVG artifacts remain unchanged and
  selectable as rollback inputs.

## Scene and generated artifacts

- Overlay:
  `isaac_sim/assets/environments/v6_kujiale_clearance_r1/kujiale_0026_A_to_B_door_open.usd`.
  It sublayers the required read-only original
  `/home/lyb/kujiale_usd_rooms_20260717/kujiale_0026/kujiale_0026_A_to_B_door_open.usd`
  and overrides only:
  - `table_0000` x: `2.4841237336013426 -> 2.0041237336013426`;
  - `tablecloth_0004` x: `2.4998628175599715 -> 2.0198628175599715`.
- A root-relative sublayer plus `PXR_AR_DEFAULT_SEARCH_PATH` passed plain PXR
  composition but the Kit OMap process saw no collision geometry. The final
  overlay therefore uses the exact existing external path above; the same
  generator then completed normally. No source USD was edited.
- Occupancy: `data/maps/occupancy/v6_kujiale_clearance_r1.{pgm,yaml}`;
  `0.05 m`, `154x248`, origin `[-5.14,-6.52]`, collision-derived by the
  existing OMap tool. Counts: occupied `11119`, free `17105`, unknown `9968`.
- GVG:
  `ros2_ws/src/robot_route_planner/config/v6_kujiale_clearance_r1_gvg_v1.{geojson,support_map.json,summary.json}`;
  graph id `v6_kujiale_clearance_r1:gvg_v1`, revision 1, 25 canonical nodes,
  48 directed canonical edges, 181 support nodes, 358 support edges, one
  connected component.
- The new spawn profile retains the unchanged calibrated map/USD transform and
  spawn coordinates while binding the new occupancy-only identity. Static is
  the only Phase-1-enabled manifest; dynamic and appearance remain disabled
  later-pilot metadata aligned to the same layout/map/GVG.

## Generation commands

```bash
cd /home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3
ISAAC_PYTHON=/home/lyb/miniconda3/envs/isaacsim/bin/python \
MAPPING_OUTPUT_ROOT=/tmp/v6_clearance_map \
/home/lyb/miniconda3/envs/isaacsim/bin/python \
  isaac_sim/tools/rivermark_occupancy_generate.py \
  --headless --output-root /tmp/v6_clearance_map \
  --map-version v6_kujiale_clearance_r1 \
  --map-stem v6_kujiale_clearance_r1 \
  --usd isaac_sim/assets/environments/v6_kujiale_clearance_r1/kujiale_0026_A_to_B_door_open.usd \
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
  --map /tmp/v6_clearance_map/v6_kujiale_clearance_r1/v6_kujiale_clearance_r1.yaml \
  --defaults /tmp/v6_clearance_engineering_defaults.yaml \
  --geojson /tmp/v6_kujiale_clearance_r1_gvg_v1.geojson \
  --mapping /tmp/v6_kujiale_clearance_r1_gvg_v1_support_map.json \
  --summary /tmp/v6_kujiale_clearance_r1_gvg_v1_summary.json
```

The temporary defaults reproduced the current deterministic `gvg_v1`
settings: unknown occupied, 0.215 m padded inscribed radius, exact Jackal
polygon plus 0.005 m padding, 0.025 m sweep spacing, 0.20 m support spacing,
and 0.385 m preferred clearance.

## Offline geometry and route validation

- Exact composed bbox shift: both table prims moved `-0.48 m` in USD x only.
- Full table assembly to cabinet gap: `1.1102488319837813 m`; to sofa:
  `0.05339693274651003 m`.
- No new 3D bbox overlap was introduced; the assembly does not overlap the
  cabinet or sofa. Existing tabletop-clutter overlaps remain pre-existing and
  were not moved because the authorized override is exactly two prims.
- G1->G2 cabinet-table throat along the generated route (`x=-0.165`,
  `y=2.005..2.605 m`): center clearance minimum `0.5295349359512329 m`,
  median/maximum `0.550000011920929 m` versus the old analyzed `0.30 m`.
- All five support routes exist, have no occupied center sample/wall crossing,
  and pass the exact directional footprint sweep:
  - G1->G2: 12.4950 m, route-edge minimum center clearance 0.3202 m;
  - G2->G3: 10.2700 m, 0.3202 m;
  - G3->G4: 3.0512 m, 0.2693 m;
  - G4->G5: 4.1631 m, 0.2500 m;
  - G5->G1: 9.0271 m, 0.2500 m.
- Visual inspected:
  `/tmp/v6_kujiale_clearance_r1_validation_overlay.png`.
  Machine-readable validation:
  `/tmp/v6_kujiale_clearance_r1_validation.json`.

## Focused validation

- Direct GVG regeneration compared byte-for-byte with the tracked GeoJSON,
  support map, and summary: **PASS**.
- Runner/static-manifest/GVG/spawn focused tests: **93 passed**.
- Direct Isaac-Python USD composition assertion: **PASS**, 12,283 composed
  prims. The Isaac Python environment lacks `pytest`, so this exact assertion
  was run as a direct Python probe; the matching pytest test is tracked.
- Three changed shell scripts: `bash -n` **PASS**.
- `git diff --check`: **PASS**.

## Exact next live command

Build a fresh strict combined Integration/Module3 snapshot containing this
commit, confirm domain 213 is empty, then run exactly one engineering episode:

```bash
cd /home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3
RUN_DIR="/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_grid_phase1_clearance_r1_$(date -u +%Y%m%dT%H%M%SZ)"
SNAPSHOT_ROOT="/absolute/path/to/fresh_combined_phase1_snapshot"
ISAAC_ASSET_ROOT=/home/lyb/isaacsim_assets/Assets/Isaac/6.0 \
R5_DOMAIN_ID=213 R5_EPISODE_INDICES=0 R5_EPISODE_SEEDS=7201 \
  ./scripts/run_v6_kujiale_low_obstacles.sh session "${RUN_DIR}" "${SNAPSHOT_ROOT}"
```

Phase 1B and the existing fail-closed Grid/reset/TF/Nav2 chain must still pass
before G2. Offline geometry is not live navigation or qualification evidence.
