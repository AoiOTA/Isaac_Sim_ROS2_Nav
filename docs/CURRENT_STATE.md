# Module3 current runtime handoff

Date: 2026-08-29

The authoritative cross-repository handoff is Integration
`docs/CURRENT_STATE.md` on branch `v6-compute-amcl-dual-odom`. This file records
only the Module3 boundary needed to avoid launching a stale runtime.

## Runtime implementation

- canonical worktree:
  `/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/bio_nav_module3`
- branch: `v6-compute-amcl-dual-odom`
- reviewed runtime commit:
  `c3f7fb3d0cd6eb6ec8cd8f0cdb453baa49fa5e6f`

This documentation commit descends that runtime implementation. Before any
run, verify local HEAD, upstream, and the remote branch agree and tracked files
are clean.

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
- the DLSS-disabled candidate is reviewed and pushed but has not been run;
- sufficient Pilot: `0/43`;
- formal campaign: `0/120`;
- the previous indoor static20 stress run is excluded from both counts.

## Resume point

Do not launch a Pilot first. On a fresh domain/root/socket, start T1 then T2
with the current wrappers and do not start T3. The startup-only discriminator
must prove:

- no DLSS internal-upscale warning;
- no GPU page fault, Xid 109, or device lost;
- Isaac and Nav2 READY;
- RGB, depth, CameraInfo, scan, ComputeOdom, GT, fixed TF, Module2, and catalog
  are current and correctly identified.

If it passes, clean up and use another fresh root for outdoor static rep1--3.
If it fails, preserve the logs and stop the unchanged launch. See Integration
`docs/CURRENT_STATE.md` for exact commits, asset paths, invalid roots, Pilot
matrix, and the rest of the plan.
