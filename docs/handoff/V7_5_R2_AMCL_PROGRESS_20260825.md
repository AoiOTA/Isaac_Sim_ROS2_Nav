# V7.5-R2 AMCL progress and stop boundary

Date: 2026-08-25

V7.5-R2 is **incomplete** and is not formal qualification. Work stopped at the
user's request after bounded Phase 3 diagnostics. Phase 4--7 and full-house
have not started. Integration and Module2 remained frozen with no V7.5 runtime.

## Repository stop state

- Module3 implementation HEAD `96f490803dc53a0feafa8d25a57c8615b22014e8`;
  pinned main `22d66470c4b903349b2467dc876490bbebfc0083`.
- Integration HEAD `d0f7fab7a9126f456377e096359c3854181bbab1`;
  pinned main `f23a7eccc542e602ec641daf7a20b14c2371dca9`.
- Module2 HEAD `98b3ffb4526a55acd318cebdf1462de82939ec05`;
  pinned main `c8297a590ba61bcf712ad4a339437fb2c44a027e`.

All three tracked worktrees were clean at stop. Run-owned processes were
stopped and locks released. Existing untracked build/install/log outputs were
preserved.

## Module3 implementation and static validation

- `0641d65`: align RPLIDAR S2E tick rate with its 10 Hz scan rate.
- `3ad8d91`: explicit occupancy-only AMCL localization owner.
- `b1222b7`: backend-aware localization readiness.
- `24e48fe`: rearm AMCL TF stability after a stale gap.
- `fc2524f`: latch sparse current-epoch AMCL pose readiness.
- `96f4908`: structured-grid AMCL motion-pilot configuration.

Recorded focused static groups passed `16`, `98`, `122`, and `30` tests; the
broad source-first suite passed `269`. Phase 1 isolated build and launch/review
checks passed. These are static/build results only.

Retained overlays:
`/tmp/bio_nav_v75_integration_bridge_overlay.JKDjgM/install/local_setup.bash`,
`/tmp/bio_nav_v75_phase2_runtime_overlay.b0oI7A/install/local_setup.bash`, and
`/tmp/v75-grid-motion-build.VXLy9K/install/local_setup.bash`.

## Phase results and evidence

### Phase 0 -- engineering GO

Combined retry4 sensor/TF/LiDAR preflight and retry5 reset-zero evidence:

- `/mnt/nas_home/Bio_Nav_Data/runs/development/v75_phase0_full_smoke_retry4_20260825T072055Z`
- `/mnt/nas_home/Bio_Nav_Data/runs/development/v75_phase0_reset_zero_retry5_20260825T073404Z`

The result is engineering GO, not qualification. Direct authored RTX LiDAR
prim readback remained unverified in that run.

### Phase 1 -- static/build/review PASS

AMCL ownership/readiness and the motion configuration passed recorded static,
build, and review checks. Live AMCL quality was not established by Phase 1.

### Phase 2 -- live engineering PASS

Evidence:
`/mnt/nas_home/Bio_Nav_Data/runs/development/v75_r2_amcl_reset_idle_retry2_20260825T092219Z`.

AMCL was the sole `map -> odom` owner and EKF the sole `odom -> base_link`
owner. Startup and one operator reset reached current-epoch readiness and
released the gate. Post-reset there were five initial poses, five AMCL poses,
and 27 stable `map -> odom` samples over 2.775 s; commands were zero and no
collision occurred. Verdict: **PHASE 2 ENGINEERING PASS, NOT QUALIFICATION**.

### Phase 3A -- bare Grid PASS, `SCENE_UNOBSERVABLE`

Evidence:
`/mnt/nas_home/Bio_Nav_Data/experiments/runs/v75_phase3a_bare_grid_20260825T094040Z`.

The basic-chain/reset smoke passed, but bare Grid had no geometry intersecting
the LiDAR plane and produced no scan. This is not AMCL or LiDAR-cadence proof.

### Phase 3B -- Attempt30 V3 stationary PASS

Bundle:
`/mnt/nas_home/Bio_Nav_Data/runs/development/v75_attempt30_v3_bundle_20260825T093839Z`.

Evidence:
`/mnt/nas_home/Bio_Nav_Data/runs/development/v75_phase3b_attempt30_stationary_20260825T095011Z`.

Candidate `Q35_50` with 27 static obstacles passed the engineering stationary
smoke. Over 141.43 simulated seconds, 720-beam scans ran at 10 Hz with
98.33% minimum and 98.75% median finite coverage. Last AMCL error was
`1.9986e-05 m` XY and `8.2448e-07 rad` yaw; commands stayed zero and no
collision occurred. Motion recovery and navigation remain unproven.

### Phase 3B -- motion engineering FAIL

Evidence:
`/mnt/nas_home/Bio_Nav_Data/runs/development/v75_phase3b_attempt30_motion_20260825T100737Z`.

Only short straight passed: **1/4 ENGINEERING PASS / MOTION ENGINEERING FAIL**.
Requested spin command integral was approximately 99%, but GT completed only
approximately 18% of a revolution (`1.1495/1.1397 rad` left/right versus
`2*pi`). AMCL ended the spins with approximately `0.734/0.728 rad` yaw error;
the circle's last XY error was `0.895 m`. No collision occurred and final
commands were zero.

Canonical flat20 evidence at
`/mnt/nas_home/Bio_Nav_Data/runs/development/v75_phase3_flat20_yaw_phase_trace_20260825T103340Z`
delivered 98.751% of expected two-second yaw. This excludes common
actuator/assist loss, but does not distinguish scene physics from localization.

### Final one-factor attempts -- harness STOP only

- `.../v75_phase3_v3_mapping_yaw_20260825T104928Z`: recorder bag directory
  was pre-created.
- `.../v75_phase3_v3_mapping_yaw_20260825T105513Z`: reset released, but a late
  probe missed volatile gate status and stopped before yaw publication.

Both roots are under `/mnt/nas_home/Bio_Nav_Data/runs/development/`. Neither
attempt exercised motion: **HARNESS STOP / NO PRODUCT VERDICT**. STOP files,
partial bag/logs, and cleanup records remain on NAS. Latest cleanup found no
domain-155 nodes or run-owned process, released locks, and no injector output.

## Blocker and next experiment

The unresolved discriminator is scene-specific motion versus localization,
not a demonstrated AMCL/EKF parameter, IMU scale, or common actuation fault.
If resumed, run one fresh Attempt30 V3 + mapping-only two-second yaw with a
transient-local gate subscriber started before reset and no pre-created bag
directory. Keep scene/spawn/realistic odometry/command/actuator path fixed and
do not tune AMCL, EKF, or IMU scale first. Compare GT yaw with canonical
flat20. V7.5-R2 remains incomplete.
