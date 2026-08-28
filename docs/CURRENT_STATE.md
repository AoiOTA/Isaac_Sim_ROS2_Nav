# Current V6 state

Date: 2026-08-28

## Pinned baseline

The current convergence baseline is the canonical
`v6-compute-amcl-dual-odom` three-repository combination:

| Repository | Commit |
| --- | --- |
| Module3 | `4e9030f3413214c8a4cc0cf0f5e1a16b3785ee91` |
| Integration | `14594f38` |
| Module2 | `7f4fbae` |

Do not substitute a similarly named historical branch, stale install, or a
different three-repository combination.

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
- Reset owner: `ExperimentRunner` calls `/simulation/reset`; consumers observe
  the resulting reset generation/event rather than initiating another reset.
- Live output root: `/mnt/nas_home/Bio_Nav_Data/experiments/runs/`.

## Evidence achieved

Phase 0 static-cold completed 5/5 as engineering evidence on the pinned
combination. This is a narrow current-run engineering result. It is not a
formal qualification, does not promote historical Rivermark results into the
current V6 line, and does not close the P2 items below. Detailed cross-repository
metrics remain in the Integration current-state record rather than being
duplicated here.

## Known P2 work

- Hot-reset and dynamic evidence is mixed across prior runs and has not been
  closed as one current pinned campaign.
- Precise terminal command-to-actuator zero latency was not closed in this
  documentation amendment. Terminal safety behavior must not be described as
  qualified from code inspection or historical timing alone.
- Outdoor navigation is not complete on the current V6 baseline.
- Appearance coverage is not complete on the current V6 baseline.
- Formal qualification is not complete. No current result may be labeled
  `FORMAL_QUALIFICATION_PASS`.

## Boundary

This page describes current runtime and evidence classification only. Dated
handoffs, Rivermark/Attempt reports, 4x20 artifacts, and report assets are
retained for historical traceability. They do not override the pins or open
items above.
