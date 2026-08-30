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
- **regression found in the pilot rerun on `e3c4385`**: static 3/3, dynamic
  3/3, but appearance 1/3 — the 5.0 s TTL is shorter than the static box's
  worst sighting gap (24.1 s on the G3 approach), so rep03 ended in a real
  contact (Isaac contact sensor fired) and rep01 in a collision-monitor stop
  with no contact. Contact judgments use the Isaac contact sensor only; SAT
  overlap is diagnostic-only. The revised design (not yet implemented): TTL
  default 90 s as leak backstop + runner clears both costmaps right after a
  dynamic group retires at leg success (the layer's `reset()` already calls
  `clearStaticTracks()`);
- sufficient Pilot: `0/43`;
- formal campaign: `0/120`;
- the previous indoor static20 stress run is excluded from both counts.

## Resume point

Implement the TTL revision first (see the evidence bullet above): in
`bio_nav_fusion` raise the `track_ttl_s` default to 90.0 (leak backstop) and
keep the layer tests pinned to an explicit TTL; in `robot_experiments` make
`_complete_obstacle_group` report actual retirements and, on dynamic leg
success with at least one retirement, clear both costmaps via the existing
`_costmap_clear_clients` before the next leg. Rebuild both packages in the
scrubbed environment of Integration `docs/RUNBOOK.md` section 1 (never with an
inherited polluted shell — the prefix-chain pollution incident is recorded in
Integration `docs/CURRENT_STATE.md`), run both packages' tests, then rerun the
per-condition pilot (static/dynamic/appearance x3, fresh cold stack per
condition, canonical runner arguments `run_indices:=N resume:=false
clear_slam_localization_buffer:=false
reset_map_base_translation_tolerance_m:=0.1
navigation_execution_backend:=route_guided`). Appearance must reach 3/3 with
zero contact-sensor events before any counted rerun; then report for a
counted-rerun decision.

Do not relaunch Rivermark until the user resumes outdoor work. When resumed,
run the load-halving cold-start A/B per Integration `docs/CURRENT_STATE.md`.
See that file for exact commits, asset paths, invalid roots, Pilot matrix,
and the rest of the plan.
