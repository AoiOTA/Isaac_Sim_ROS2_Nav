# V7.3 Alt-1 GICP local-odometry shadow handoff (2026-08-25)

## Result and boundary

- Implementation commit: `9f959fb0a01d3fcc7f03d5906b69d1e8a8cd0aaf`.
- Added only the independent C++ package `pointcloud_local_odometry`. Its
  launch argument `enabled` defaults to `false`; it is not referenced by
  bringup, canonical profiles, EKF, Grid, Nav2, Integration, or Module2.
- This is an **IMPLEMENTED / STATIC PASS** result. No Isaac, ROS live, bag
  replay, navigation, promotion, Phase 1D, or qualification run occurred.

## Interface and algorithm

```text
/lio/points_raw  PointCloud2, SensorDataQoS, lio_lidar_link
  -> finite check -> one fixed VoxelGrid -> PCL 1.14 GICP
  -> /local_odom/gicp_shadow  Odometry
       frame gicp_odom_shadow, child base_link, input stamp
  -> /local_odom/gicp_status  DiagnosticArray
```

GICP aligns the current source scan to the previous successful target scan and
returns `T_prevL_currL`. The core converts it with the direct static
`T_B_L` lookup as `T_B_L * T_prevL_currL * inverse(T_B_L)`, then accumulates
from identity. The first valid scan publishes identity with `initializing`.
Twist is unavailable and remains zero with `1e6` diagonal covariance; pose
uses the conservative fixed diagonal in `config/gicp_shadow.yaml`. The node
constructs no TF broadcaster and never subscribes to wheel, IMU, global-map,
or evaluator-only pose inputs.

Frame/stamp rollback, missing direct `base_link <- lio_lidar_link` lookup,
insufficient/nonfinite points, GICP non-convergence, nonfinite output, and
fitness above the fixed threshold produce `degraded` status without a new
pose. Rejected scans do not replace the previous successful scan. Reset is
only process restart; no service, retry, fallback, or additional state machine
was added.

## Fixed configuration

- voxel leaf `0.15 m`, minimum filtered points `100`;
- maximum correspondence `1.0 m`, iterations `40`;
- transformation/Euclidean-fitness epsilon `1e-4`;
- maximum accepted fitness `0.25`;
- exact topics and frames are in `config/gicp_shadow.yaml`.

These are initial shadow values, not live-calibrated or promotion thresholds.

## Static validation

- Clean `/opt/ros/jazzy`-only isolated build at
  `/tmp/v73_alt1_gicp_final.3GcMgH`: one package finished.
- Isolated `colcon test-result`: `12 tests, 0 errors, 0 failures, 0 skipped`.
  Seven GTests cover identity, known xyz/rpy direction, two-step accumulation,
  base/LiDAR conjugation, insufficient/nonfinite input, fitness rejection, and
  retention of the previous successful scan. Three Python contract cases
  cover default OFF, exact topics/frames/config, SensorDataQoS, one input,
  forbidden data dependencies, and absence of a TF broadcaster.
- Source-first/no-cache contract pytest, launch Python compilation, and scoped
  `git diff --check` passed. The only build stderr was PCL's non-blocking CMake
  `CMP0144/FLANN_ROOT` developer warning.

## Next bounded review and remaining risk

A fresh reviewer should first use a short sensor replay or live shadow with
the already validated OS1 adapter, after physical reset, and measure point
count, convergence/fitness, processing time versus the 10 Hz input, direction,
drift, jumps, covariance usefulness, and restart behavior. It must also
observe that no GICP TF is emitted. Until then, this package remains default
OFF and supplies no evidence for canonical `/odom`, Phase 1D, navigation, or
formal qualification.
