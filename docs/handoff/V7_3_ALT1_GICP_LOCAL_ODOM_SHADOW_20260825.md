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
