# V7.3 Alt-1 local-odometry shadow handoff (2026-08-25)

## GICP terminal replay decision

The original default-OFF GICP shadow reached 2,031/2,031 input/output delivery
in the exact finalized-bag replay at
`/mnt/nas_home/Bio_Nav_Data/experiments/runs/v73_alt1_gicp_replay_20260824T211335Z`.
Processing p50/p95/max was 6.48/11.87/54.36 ms, but XY error relative to the
same-run active EKF `/odom` crossed 0.5/1/5 m after only
1.20/2.50/11.40 s of motion and reached 11.529 m before the recorded source
collision. This is **ENGINEERING STOP**, despite delivery and load success.

Offline relative-increment planar projection did not rescue GICP: projected
error crossed 0.5/1/5 m after 1.20/2.30/10.00 s and ended 12.409 m away with
a 93.145 degree endpoint-direction error. The GICP implementation is therefore
removed rather than retained behind a backend selector.

## NDT replacement

Implementation commit: `1117bd6971edcd8fdffec7f8c1b61acad0e55dff`.

`pointcloud_local_odometry` now has one default-OFF PCL 1.14 NDT shadow:

```text
/lio/points_raw  PointCloud2, SensorDataQoS, lio_lidar_link
  -> finite xyz check -> one VoxelGrid -> PCL NDT current(source)-to-previous(target)
  -> /local_odom/ndt_shadow  Odometry
       frame ndt_odom_shadow, child base_link, input stamp
  -> /local_odom/ndt_status  DiagnosticArray
```

The only executable/config/launch products are `ndt_local_odometry_node`,
`config/ndt_shadow.yaml`, and `launch/ndt_shadow.launch.py`. The launch
`enabled` argument defaults to false. There is one cloud subscription, a
direct static `base_link <- lio_lidar_link` lookup, no TF broadcaster, and no
wheel, IMU, map, GT, producer, adapter, bringup, profile, EKF, Grid, Nav2,
Integration, or Module2 connection.

The first valid scan publishes identity with `initializing`. Accepted NDT
increments are conjugated into the base frame and accumulated in SE(3).
Invalid/nonfinite/insufficient input, missing TF, non-convergence, nonfinite
result, or fitness above the fixed threshold emits `degraded` without
replacing the previous successful scan. Fixed initial shadow parameters are
voxel 0.15 m, minimum 100 filtered points, NDT resolution 0.5 m, step 0.1,
40 iterations, transformation epsilon 0.001, and maximum fitness 0.25.

## NDT replay decision

Clean `/opt/ros/jazzy` isolated build/test at
`/tmp/v73_alt1_ndt_final.7q1fdS` reported `13 tests, 0 errors, 0 failures,
0 skipped`. Source-first no-cache pytest, launch Python
compilation/show-args, and diff checks passed. Eight synthetic GTests cover
identity, known xyz/rpy direction, two-step accumulation,
base/LiDAR conjugation, insufficient/nonfinite input, fitness rejection,
non-convergence, and rejected-scan retention. The only build stderr is PCL's
non-blocking FLANN `CMP0144` developer warning.

The targeted finalized-MCAP replay at
`/mnt/nas_home/Bio_Nav_Data/experiments/runs/v73_alt1_ndt_replay_20260824T220508Z`
then reached 1,838/1,838 post-initialization input/status/odom delivery with
processing p50/p95/max `3.715/7.262/23.030 ms`, but failed geometry against the
same-run active EKF sensitivity reference. XY difference crossed `0.5/1/5 m`
after `2.00/4.10/19.40 s`; at the recorded source collision it was `6.584 m`,
with `50.65 deg` endpoint-direction error, path scale `0.682`, and relative z
`-0.815 m`. Verdict: **NDT32 ENGINEERING STOP**. Keep NDT default OFF; this is
not GT, Isaac/Nav2 live evidence, promotion, Phase 1D, or qualification.

## OS1-128 sensor extension

The existing installed Isaac `OS1` registry contains the exact variant
`OS1_REV6_128ch10hz512res`. Module3 now exposes that variant only as a second
explicit `--lio-lidar-profile` choice. The safe producer default remains
`off`, and the existing explicit OS1-32 choice remains first and unchanged.
Both selections use the same physical mount, `lio_lidar_link`, raw topic
`/lio/points_raw_isaac`, SensorData QoS, auxiliary PointCloud2 outputs, and
strict `/lio/points_raw` adapter; only the registry variant and declared
channel count differ. No producer thread, adapter, NDT parameter/code,
canonical `/odom`/TF, Nav2, Integration, or Module2 change was made.

Source-first/no-cache focused tests passed `31`; the unchanged adapter tests
passed `12`; Python compilation and diff checks passed. The no-Kit
`--validate-only --lio-lidar-profile OS1_REV6_128ch10hz512res` path returned
`validation: PASS`. Verdict: **OS1-128 SENSOR EXTENSION STATIC READY ONLY**.
No Kit, ROS/Isaac live, fixed-motion, navigation, promotion, Phase 1D, or
qualification was run. The next step is a fresh reviewer short Isaac
fixed-motion producer/adapter smoke. It must observe the actual 128-channel
`channel_id` range because the unchanged adapter's established OS1-32 ring
bound has not been runtime-validated with this variant.

## OS1-128 adapter ring-bound amendment

The subsequent passive smoke at
`/mnt/nas_home/Bio_Nav_Data/experiments/runs/v73_alt1_ndt_os1_128_smoke_20260824T225456Z`
proved the OS1-128 producer contract in live Isaac: 80 raw frames were
observed, and each of the first five complete frames had finite XYZ, the
required schema, monotonic timestamps, and exactly 128 unique
`channel_id` values spanning `0..127`. The unchanged adapter rejected all
frames at its fixed `[0, 31]` bound, so adapted frames, NDT status, and NDT
odometry all remained zero. Verdict: **PRODUCER ENGINEERING PASS / ADAPTER
ENGINEERING STOP**. No NDT128 result exists.

This amendment makes the adapter's strict upper bound an integer `max_ring`
parameter. Its default remains `31`, preserving the complete OS1-32 contract.
The adapter launch exposes the same default and forwards an explicitly typed
integer; an OS1-128 passive smoke must opt in independently with
`max_ring:=127`. Startup rejects values outside `0..255`, matching the output
`UINT8 ring` representation. Accepted `channel_id` is copied unchanged to
`ring`; there is no auto-detection, profile binding, truncation, remapping,
filtering, fallback, or producer/NDT change.

Source-first/no-cache adapter plus LiDAR-profile focused tests passed `33`.
They cover default acceptance of every ring `0..31`, default rejection of 32
and 127, explicit acceptance and unchanged schema/time for every ring
`0..127`, invalid bounds, launch default/forwarding, and a cross-component
32-channel-to-31 / 128-channel-to-127 contract read from `lidar_3d.yaml`.
The isolated `/opt/ros/jazzy` build/test at
`/tmp/v73_alt1_adapter_ring128.L3OepV` passed `91 tests, 0 errors, 0 failures,
0 skipped`; Python compilation, launch `--show-args`, and diff checks passed.
Verdict: **ADAPTER FIX STATIC PASS ONLY**. No ROS/Isaac/live run was performed
for the amendment.

Next is one fresh passive producer/adapter/NDT preflight using the explicit
OS1-128 profile and `max_ring:=127`. Do not reset or move until adapted output
has the full unchanged `ring 0..127` range and NDT emits identity/status
without degradation. NDT128 geometry, cadence, and accuracy remain unassessed;
this amendment is not promotion, Phase 1D authorization, or qualification.

## NDT128 terminal decision

The subsequent bounded OS1-128 capture at
`/mnt/nas_home/Bio_Nav_Data/experiments/runs/v73_alt1_ndt_os1_128_capture_first_20260825T002726Z`
completed its 16 s engineering window with 160 raw, adapted, NDT status, and
NDT odometry samples. The full `ring 0..127` contract remained valid, NDT
reported tracking throughout the measured window, and processing p50/p95/max
was `7.456/15.531/24.931 ms`.

Geometry nevertheless failed the local-odometry objective. Against the
start-aligned GT sensitivity reference, NDT XY error crossed `0.5/1.0 m` after
`3.750/7.150 s`, reached `2.067 m`, and ended at `2.060 m`; endpoint-direction
error was `-102.60 deg` and path scale was `1.866`. The same-window active EKF
ended at `0.028 m` XY error. The run-level `ENGINEERING_SMOKE_COMPLETE` field
therefore records capture completion only, not localization acceptance.
Verdict: **NDT128 ENGINEERING STOP**. NDT is removed from the product surface;
this is not promotion, Phase 1D, navigation evidence, or qualification. The
specified run root contained `metrics/analysis.json` but no conclusion file
when this amendment was written.

## KISS-ICP offline vendor replacement

Implementation commit: `d16d2a8b8315cad446cbbc294aa46c629ab0911b`.

Module3 vendors official KISS-ICP `v1.3.0` at
`b16835283aee62f7d5e2bdf6c1c3bb2930de74ff`. Its tag-pinned official
dependencies are Sophus `1.24.6` at
`d0b7315a0d90fc6143defa54596a3a95d9fa10ec` and robin-map `v1.4.0` at
`4ec1bf19c6a96125ea22062f38c2cf5b958e448e`. Full required C++/ROS source and
the two dependency archives are retained with their licenses and no Git
metadata. The only upstream differences are three offline CMake edits: force
the vendored header dependencies and replace their FetchContent URLs with
local `SOURCE_DIR` paths. No algorithm or ROS node implementation changed.

`pointcloud_local_odometry` is now only the default-OFF launch/config owner.
It starts official `kiss_icp_node` with `/lio/points_raw` remapped to
`pointcloud_topic` and `kiss/odometry` remapped to
`/local_odom/kiss_shadow`. Fixed shadow integration is `base_link`,
`kiss_odom_shadow`, `publish_odom_tf=false`, `publish_debug_clouds=false`, and
`data.deskew=true`; all remaining KISS algorithm parameters retain upstream
defaults. There is no backend selector, PCL/NDT/GICP implementation, health
adapter, canonical odom/TF/Nav2 consumer, or wheel/IMU/map/GT subscription.
Process restart is the reset boundary; runner-owned external TF readiness must
be proven before replay input.

Clean `/opt/ros/jazzy` build at `/tmp/v73_kiss_offline_build.R3x3Bb` passed for
`kiss_icp` and `pointcloud_local_odometry` with
`FETCHCONTENT_FULLY_DISCONNECTED=ON`. The upstream packages declare no tests
in this build (`0 tests, 0 errors, 0 failures, 0 skipped`). Pin/license/source
diff/static-product checks, Python compilation, single-package discovery, and
launch `--show-args` passed. The only warning was the harmless unused
FetchContent variable in the launch-only owner package.

Verdict: **KISS VENDOR STATIC PASS ONLY**. No ROS/Isaac process, replay, live
navigation, promotion, Phase 1D, or qualification was run. Next is one fresh
reviewer-owned bounded OS1-32 finalized-MCAP replay using exact source
`/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_fastlio2_ouster_g2_retry_20260824T035347Z/bag/fastlio_shadow`,
rate `1.0`, offset `399.0 s`, and duration `250.0 s`, after external TF
readiness. Do not promote unless its measured geometry provides new evidence.

## Common LiDAR axis StructureTF static amendment

Offline evidence at
`/mnt/nas_home/Bio_Nav_Data/experiments/runs/v73_kiss_icp_replay_20260825T010237Z`
supports one common `base_link -> lio_lidar_link` `Rz(+90 deg)` candidate. It
reduced start-SE3 endpoint-direction error from `97.52/93.56/94.14 deg` to
`7.50/3.59/4.20 deg` for KISS/GICP/NDT32 and delayed each backend's first
0.5 m active-EKF-relative crossing. This was an offline counterfactual against
estimated `/odom`, not a corrected replay, ground truth, live evidence, or a
product PASS. Residual KISS scale/z/pitch, GICP non-planar, and NDT32
scale/z/pitch blockers remain.

The minimal implementation gives only the Isaac StructureTF publisher an
optional `tf_rotation_override_xyzw`. The `lio_lidar_link` entry publishes
xyzw `[0,0,0.7071067812,0.7071067812]` while retaining translation
`[0.120,0,0.333]`. The physical `rotation_xyzw` remains identity, and
`scene_composer` continues to author the existing OS1 prim pose from that
physical field. A missing override retains the previous TF behavior. No
adapter, sensor prim/profile, USDA, URDF/RSP source, KISS config/algorithm,
canonical odometry/TF, EKF, Nav2, Integration, or Module2 change was made.
Both OS1-32 and OS1-128 inherit the same `lio_lidar_link` StructureTF, and the
adapter continues to copy raw SENSOR XYZ unchanged.

FAST-LIO remains rejected/default OFF. Its unchanged
`config/ouster_shadow.yaml` still owns an internal `Rz(+90 deg)` conversion;
it must not be enabled with this StructureTF correction until a separate task
chooses one owner and removes the duplicate from the other path. This
amendment does not modify that rejected algorithm configuration. The RSP/URDF
structure source also remains identity and is outside this Isaac-StructureTF
candidate; do not use RSP for the corrected replay.

Focused source tests passed `25` for LiDAR/StructureTF and `22` for the raw
adapter/FAST-LIO contracts. No-Kit `mapping_start` validate-only checks passed
for both `OS1_REV6_32ch10hz512res` and `OS1_REV6_128ch10hz512res`; the existing
unresolved `OmniPBR.mdl` warnings remained non-blocking. Python compilation
and final diff checks are recorded in the ledger.

Verdict: **COMMON-AXIS STATIC FIX ONLY**. No replay, ROS graph, Isaac/Kit,
navigation, promotion, Phase 1D, safety evidence, or qualification was run.
Next is the same finalized-bag KISS replay with the exact corrected static TF,
adapter XYZ raw, and FAST-LIO off; recompute direction/crossing/scale/z/rp
metrics before any live run or acceptance decision.
