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
  `bb4e78daa2e97ca85d48a2bdd2591828eb826f99`
- current cross-repository runtime tuple: Integration `493a65be`, Module2
  `8928cd8f`, Module3 `bb4e78d`
- indoor live-evidence commit: `350105e`

The indoor runs at `350105e` must not be attributed to the current runtime
tuple. Rivermark A controls ran `33136fa2`; B ran `d62f482`, whose source tree is
identical to integrated descriptor candidate `7ba6816` in current `bb4e78d`.
The documentation-only commits that record this handoff are not runtime
implementation commits. Before any run, verify that local HEAD, upstream, and
remote agree and that tracked files are clean.

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
- RGB-D remains `320x180 @ 10 Hz`, Rivermark alone uses DLSS-off, and the sole
  Rivermark wrapper fixes `--rtx-descriptor-sets 20000`.

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
- Current `bb4e78d` implements condition-stack attestation, immutable
  per-episode `stack_contract.json` snapshots, and the six-root sufficient-Pilot
  aggregate generator/freezer input path. This tooling has code/focused-test
  evidence only and has not produced Pilot or formal results.
- At `350105e`, focused LiDAR-visible retirement passed `2/2`; dynamic and
  appearance engineering closure passed `3/3` each. Static has only a fresh
  `1/1` smoke; its earlier `3/3` evidence ran an older commit and cannot be
  attributed to the current baseline, whose planned static `3/3` remains
  unmet. These are engineering runs, not sufficient Pilot or formal
  qualification.

Exact roots and evidence boundaries are in the
[Integration experiment ledger](/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/bio_nav_integration/docs/handoff/EXPERIMENT_LEDGER.md#2026-08-30--v6-dynamic-retirement-indoor-closure-and-rivermark-candidate-verdict).

## Outdoor startup verdict and resume point

The authoritative descriptor campaign root is
`/mnt/nas_home/Bio_Nav_Data/experiments/pilots/v6_rivermark_descriptor_ab_33136fa2_20260830T105641Z`.
All valid A1/A2/A3 control runs page-faulted at the same location before READY,
so A passed `0/3`; A ran `33136fa2`. B1/B2/B3 ran `d62f482` and each reported
`RTX_DESCRIPTOR_SETS requested=20000 applied=20000`, reached READY with all
required topics, fixed TF, and no AMCL, then held for `603/604/603 s` with zero
kernel faults. B passed `3/3`; its source tree is identical to integrated
`7ba6816`, which current `bb4e78d` contains. Descriptor sets `20000` is promoted
as the only Rivermark startup setting.

This is startup engineering evidence, not a driver or lower-level root-cause
claim. The rejected PointInstancer filter remains retired. Outdoor static
engineering `3/3` is the next eligible engineering stage, but the current user
stop boundary is immediately after publishing Module3: do not run it now. The
sufficient Pilot remains `0/18`, formal remains `0/120`, and formal execution
remains `NOT_AUTHORIZED`. The [Module3 runbook](RUNBOOK.md) records the future
procedure without authorizing execution.
