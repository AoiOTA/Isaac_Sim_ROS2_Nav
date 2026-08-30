# Module3 current runtime handoff

Date: 2026-08-30

The authoritative cross-repository handoff is Integration
`docs/CURRENT_STATE.md` on branch `v6-compute-amcl-dual-odom`. This file records
only the Module3 boundary needed to avoid launching a stale runtime.

## Runtime implementation

- canonical worktree:
  `/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/bio_nav_module3`
- branch: `v6-compute-amcl-dual-odom`
- reviewed runtime commit:
  `e3c4385d8a50d78d1fb41f5a1ceca7bfd0f6c83d`

This documentation update descends that runtime implementation, which adds
the cognitive-obstacle-layer `track_ttl_s` ghost purge on top of `7ad02f8`.
Before any run, verify local HEAD, upstream, and the remote branch agree and
tracked files are clean.

## Active ownership

Indoor Kujiale:

- Compute Odometry owns `/odom` and `odom -> base_link`;
- AMCL owns `map -> odom`;
- Module1 wheel+IMU EKF publishes `/bio_nav/module1/odom` without TF.

Outdoor Rivermark:

- Compute Odometry owns `/odom` and `odom -> base_link`;
- AMCL is absent;
- `ideal_localization_tf` alone publishes calibrated fixed `map -> odom`;
- the original `rivermark_selected` map and current 30-tile catalog remain in
  use; no regenerated map was adopted;
- Rivermark alone passes `--disable-dlss`, which maps to SimulationApp
  `anti_aliasing=0`. RGB-D remains `320x180 @ 10 Hz`.

Both scenes keep Module1, Module2 obstacle output, GVG, SR/DR RoutePrior,
cognitive obstacle layers, and `CognitiveRiskCritic` in the M3 arm. The active
low-obstacle profile does not use the raw RGB-D voxel writer.

## Current evidence boundary

- fixed outdoor TF and Module2/cognitive readiness were observed live on d232,
  but its T3 runner came from a stale overlay; d232 is invalid, not a Pilot;
- d211 proved T2 cannot start first because its startup reset needs the ROS
  wheel/EKF reset services;
- d210 and d208 failed before READY with Isaac/RTX GPU faults;
- the DLSS-disabled candidate ran live: d218 held 30 min stable (GPU question
  passed), but the identical d219 cold start crashed 71 s into startup with
  Xid 109 / CTX_SWITCH_TIMEOUT after foliage point-instancer warnings — DLSS
  is not the root cause; outdoor escalation (load-halving A/B) is chosen and
  parked until the user resumes outdoor work;
- indoor d215 formal 3x20 on `7ad02f8`: static 19/20, dynamic 16/20,
  appearance 20/20 (55/60);
- the dynamic ghost family (costmap LETHAL persisting after the box retires)
  is fixed by `e3c4385` (`track_ttl_s`) and validated live on d220: dynamic
  rep01-03 all pass, layer status shows zero applied cells from the retire
  event onward, and both costmap instances log the TTL expiry;
- sufficient Pilot: `0/43`;
- formal campaign: `0/120`;
- the previous indoor static20 stress run is excluded from both counts.

## Resume point

Finish the per-condition pilot rerun on the fixed runtime (3 reps each of
static, dynamic, appearance; one fresh cold stack per condition; fresh
domain/root/socket; headless Isaac; per-rep dispatch with the canonical
runner arguments `run_indices:=N resume:=false
clear_slam_localization_buffer:=false
reset_map_base_translation_tolerance_m:=0.1
navigation_execution_backend:=route_guided`). Confirm static and appearance
are unaffected by the `track_ttl_s` change and that dynamic no longer shows
the ghost family, then report for a counted-rerun decision.

Do not relaunch Rivermark until the user resumes outdoor work. When resumed,
run the load-halving cold-start A/B per Integration `docs/CURRENT_STATE.md`.
See that file for exact commits, asset paths, invalid roots, Pilot matrix,
and the rest of the plan.
