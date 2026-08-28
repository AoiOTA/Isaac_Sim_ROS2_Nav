# Current V6 state

Date: 2026-08-29

## Current cleanup starting heads

The documentation cleanup started from this three-repository combination:

| Repository | Commit |
| --- | --- |
| Integration | `5ba37eaa6a81d37193bc9ff4232cb24f87bbcf2d` |
| Module3 | `53cbc3fb3eba10fdd8d4675f9bd38bb95d7b9b74` |
| Module2 | `9e0731c760b6b15a9c0b0e5fbc20efa0eea423ad` |

These are cleanup starting heads, not pins for the 5/5 run below. This cleanup
and its subsequent documentation-only commits have not been live-revalidated.
Do not substitute a similarly named historical branch, stale install, or an
unrecorded three-repository combination for a live result.

## Current implementation and Pilot boundary

The current cleanup Pilot used this implementation tuple:

| Repository | Commit |
| --- | --- |
| Integration | `ea157871` |
| Module3 | `ec3c8326` |
| Module2 | `a6b1b216` |

`ec3c8326` includes the Jackal asset hydration fix needed before Isaac startup;
this documentation-only commit is its descendant. Across one static smoke, a
two-episode static pair, and a two-episode dynamic pair, all five episodes
completed the full route: `25/25` legs were strict-success and collision-free.
The compact evidence roots are:

- static smoke:
  `/mnt/nas_home/Bio_Nav_Data/experiments/pilots/v6_cleanup_static_cold_master_20260828T182058Z_d212`;
- static pair:
  `/mnt/nas_home/Bio_Nav_Data/experiments/pilots/v6_cleanup_static_pair_master_20260828T183756Z_d213`;
- dynamic pair:
  `/mnt/nas_home/Bio_Nav_Data/experiments/pilots/v6_cleanup_dynamic_pair_master_20260828T185156Z_d214`.

This is current engineering Pilot evidence, not formal qualification. The
authoritative cross-repository state and detailed metrics remain in Integration
`docs/CURRENT_STATE.md` and `docs/handoff/EXPERIMENT_LEDGER.md`; this page only
indexes the Module3 boundary.

## Current runtime

- Active exact-scene chain:
  `scripts/run_v6_r5_phase_b_kujiale.sh` -> `scripts/run_ros.sh` and
  `scripts/run_isaac.sh`.
- Phase F chain: `scripts/run_v6_kujiale_low_obstacles.sh` plus
  `scripts/run_v6_low_obstacle_phase_f_stack.sh`.
- Environment source: paired Integration
  `env/v6_pilot_setup.sh`; source it once in each clean terminal.
- Localization substrate: Isaac Compute Odometry owns `/odom` and
  `odom -> base_link`; AMCL owns `/amcl_pose` and `map -> odom`; Module1 EKF
  publishes `/bio_nav/module1/odom` with no TF.
- Scene identity: original `kujiale_0026_A_to_B_door_open.usd` with the
  `v6_kujiale_isaacgen_v1` occupancy map, spawn calibration, and GVG.
- Nav2: `stable` for the Phase B baseline and
  `v6_low_obstacle_isolation` for Phase F.
- Reset service owner: Isaac `ResetServiceBridge` owns `/simulation/reset`.
  Phase B uses the `v6_formal_episode` caller through
  `run_v6_formal_episode.sh`; Phase F uses the `ExperimentRunner` caller. Each
  run has only one orchestrating episode caller.
- Live output root: `/mnt/nas_home/Bio_Nav_Data/experiments/runs/`.

## Evidence achieved

Phase 0 static-cold completed 5/5 as engineering evidence on these actual run
pins:

| Repository | Commit |
| --- | --- |
| Integration | `07815fa2f1d8a12f14c87ac92cdb3b3dbd4e16a5` |
| Module3 | `dcbe1b2b030732cf09b2511aa5844682a60409b0` |
| Module2 | `d71b2820036f8caf8387f0651191947bffee5bf6` |

The run root is
`/mnt/nas_home/Bio_Nav_Data/experiments/pilots/v6_phase0_static_cold_single_source_20260828T112000Z_d171`;
the summary is at
`runner/v6_pilot_kujiale_static_hotreset/run-0001-seed-8601/run_summary.json`
under that root. This is narrow engineering evidence, not a formal
qualification. It does not promote historical Rivermark results into the
current V6 line or close the P2 items below. Detailed cross-repository metrics
remain in the Integration current-state record rather than being duplicated
here.

## Generated-report storage

The generated `docs/reports/` tree from source HEAD
`09c3ae80a5766ccf37fd244421e4c5f50afe7e91` is stored at
`/mnt/nas_home/Bio_Nav_Data/experiments/visualizations/module3_repo_generated_09c3ae80a5766ccf37fd244421e4c5f50afe7e91/docs/reports/`:
9 files, 512198 bytes, with per-file `cmp` PASS. This storage move does not
promote any evidence. `docs/report_assets/`, `docs/videos/`, and `docs/figures/`
remain tracked because repository callers still reference them.

## Known P2 work

- The five-episode cleanup Pilot closes only its static/dynamic engineering
  boundary; it does not replace the planned formal campaign.
- Precise terminal command-to-actuator zero latency was not closed in this
  documentation amendment. Terminal safety behavior must not be described as
  qualified from code inspection or historical timing alone.
- Outdoor and appearance candidate assets/configuration remain unfrozen. A
  historical Rivermark candidate referenced the host-local
  `/home/lyb/Rivermark/rivermark.usd`; it is not a canonical current asset.
  Any future use must freeze an explicit external-asset configuration and NAS
  path before a live run.
- Outdoor navigation and appearance coverage are not complete on the current
  V6 baseline.
- Formal qualification is not complete. No Pilot result above may be labeled
  `FORMAL_QUALIFICATION_PASS`.

## Boundary

This page describes current runtime and evidence classification only. The
historical ledger, Rivermark/Attempt reports, 4x20 artifacts, and report assets
are retained for traceability. No V6 micro-handoff files remain; retained
history does not override the pins or open items above.
