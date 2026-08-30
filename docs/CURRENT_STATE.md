# Module3 current runtime handoff

Date: 2026-08-30

The authoritative cross-repository handoff is the
[Integration current state](/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/bio_nav_integration/docs/CURRENT_STATE.md).
This file records only the Module3 boundary needed to avoid launching stale or
rejected runtime paths.

## Runtime implementation

- canonical worktree:
  `/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/bio_nav_module3`
- branch: `v6-compute-amcl-dual-odom`
- current runtime implementation commit:
  `10aaf6803c42c6c7025fbfbd1213cd4a6385e5bf`
- indoor live-evidence commit: `350105e`

Current `10aaf68` is the post-run commit that removes the rejected Rivermark
PointInstancer filter. It must not be attributed to live runs executed at
`350105e`. Before any run, verify that local HEAD, upstream, and remote agree
and that tracked files are clean.

## Active ownership and scene contracts

Indoor Kujiale:

- Compute Odometry owns `/odom` and `odom -> base_link`;
- AMCL owns `map -> odom`;
- Module1 wheel+IMU EKF publishes `/bio_nav/module1/odom` without TF;
- static and appearance use the low obstacle;
- dynamic uses the LiDAR-visible G2 crossing actor and therefore does not prove
  sub-LiDAR dynamic-obstacle perception.

Outdoor Rivermark:

- Compute Odometry owns `/odom` and `odom -> base_link`;
- AMCL is absent;
- `ideal_localization_tf` alone publishes calibrated fixed `map -> odom`;
- the original `rivermark_selected` map and 30-tile catalog remain active;
- RGB-D remains `320x180 @ 10 Hz` and Rivermark alone uses DLSS-off.

Both scenes keep Module1, Module2 obstacle output, GVG, SR/DR RoutePrior,
cognitive obstacle layers, and `CognitiveRiskCritic` active in the M3 arm. The
raw RGB-D voxel writer is not active.

## Current behavior and indoor evidence

- SAT overlap is permanently diagnostic-only. Only Isaac ContactSensor
  `/simulation/collision` determines physical collision; Collision Monitor
  stops remain independent navigation/safety failures.
- `CognitiveObstacleLayer.track_ttl_s` defaults to 90 s so static/appearance
  tracks survive the observed 24.1 s sighting gap.
- G2 completion reports actual actor retirements, requires the actor to be
  retired/invisible/collision-disabled, clears both global and local costmaps,
  and waits for fresh empty Module2 and layer status before G3.
- At `350105e`, focused LiDAR-visible retirement passed `2/2`; dynamic and
  appearance engineering closure passed `3/3` each. Static has only a fresh
  `1/1` smoke; its earlier `3/3` evidence ran an older commit and cannot be
  attributed to the current baseline, whose planned static `3/3` remains
  unmet. These are engineering runs, not sufficient Pilot or formal
  qualification.

Exact roots and evidence boundaries are in the
[Integration experiment ledger](/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/bio_nav_integration/docs/handoff/EXPERIMENT_LEDGER.md#2026-08-30--v6-dynamic-retirement-indoor-closure-and-rivermark-candidate-verdict).

## Outdoor blocker and resume point

The official `350105e` startup campaign observed A1 filter OFF for 602 s with
zero GPU faults. B1 filter ON produced the exact expected inspection counts,
then encountered Xid 109 / device-lost / SIGSEGV. The PointInstancer candidate
failed its gate; this does not prove that the filter caused the fault. T3 was
not started, and the filter was removed in `10aaf68`.

Do not rerun the rejected candidate and do not start outdoor static3. Resume
only after the Integration current state records a new discriminative
Rivermark hypothesis with a fresh startup gate. The
[Module3 runbook](RUNBOOK.md) retains component commands as blocked reference,
not authorization.
