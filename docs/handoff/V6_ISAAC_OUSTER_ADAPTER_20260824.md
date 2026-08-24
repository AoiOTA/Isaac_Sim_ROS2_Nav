# V6 Isaac Ouster producer and FAST-LIO2 adapter handoff (2026-08-24)

## Result and boundary

- Added one optional Isaac RTX LiDAR profile,
  `OS1_REV6_32ch10hz512res`, and a strict PointCloud2 adapter. Both are
  default OFF and are not wired into the canonical Phase-1 runner.
- The existing `RPLIDAR_S2E` producer is unchanged: it still publishes
  `/lidar/points_raw` in `rtx_world` using WORLD/COMPENSATED output. Its
  PointCloud2-to-LaserScan/Nav2 `/scan` path is unchanged.
- No FAST-LIO2 process, LIO output, EKF fusion, Nav2, route goal, navigation
  episode, evidence campaign, or qualification was run.
- A bounded live Isaac/ROS preflight was not run by this task agent because
  its execution instructions prohibited launching Isaac or ROS. Therefore
  all actual scan metrics below remain explicitly unverified.

## Sensor-factory startup amendment

The first sensor-only review stopped before topic creation. Exact log
`/tmp/v6_ouster_sensor_review.pqKSbr/isaac_domain230.log` showed the producer
calling the installed Isaac 6.0.1 API with the profile name as `config` and a
null `variant`:

```text
ValueError: Lidar config 'OS1_REV6_32ch10hz512res' not found.
```

The stable user selection remains `OS1_REV6_32ch10hz512res` and remains
default OFF. Its Isaac loader fields are now separate and exact:

```text
config   OS1
variant  OS1_REV6_32ch10hz512res
```

Before the optional sensor is created, the factory resolves those two names
against the installed `SUPPORTED_LIDAR_CONFIGS` registry. Missing config or
variant values fail with the requested names and available names; the call is
not allowed to fall through to USD loading with a null or unknown variant.
The existing RPLIDAR creation path and arguments are unchanged.

## Producer contract

The single producer selection is `--lio-lidar-profile`:

```text
default                           off
opt-in                            OS1_REV6_32ch10hz512res
Isaac config / variant            OS1 / OS1_REV6_32ch10hz512res
channels / scan / horizontal      32 / 10 Hz / 512
range                             0.3-120 m
raw topic / frame                 /lio/points_raw_isaac / lio_lidar_link
coordinates / motion             SENSOR / NONCOMPENSATED
metadata                          intensity, timestamp, channel ID
QoS                               SensorData, BestEffort, depth 5
```

When the profile is `off`, `SensorFactory` does not call `Lidar.create` for
the OS1, does not create its Render Product, and `RosGraphBuilder` does not
materialize `/World/Graphs/LioLidar`. The baseline RPLIDAR still runs normally.

Installed Isaac 6.0.1 source inspection confirms that
`ROS2RtxLidarPointCloudConfig` supports the three selected metadata fields.
The installed publisher test defines raw timestamp as one PointCloud2 field
named `timestamp`, `UINT32 count=2`, with low/high words reconstructing the
absolute nanoseconds `GMO.timestampNs + GMO.timeOffsetNs`; `channel_id` is
`UINT32` and is copied from GMO. This is API/source evidence, not a live scan.

Important live STOP risk: the installed OS1 USDA authors emitter channel IDs
`1..32`. Since the installed publisher test checks that `channel_id` matches
GMO directly, the actual cloud may contain ring 32. The adapter intentionally
requires direct `channel_id` values `0..31` and does not subtract, clamp, or
fabricate a ring. If live preflight observes `1..32`, it must STOP for an
explicit producer/consumer convention decision.

## Adapter contract

`ros2 launch robot_odometry ouster_pointcloud_adapter.launch.py` starts no
process by default. Explicit opt-in is `enabled:=true`.

Input requirements are little-endian, finite XYZ, valid `point_step` and
`row_step`, frame `lio_lidar_link`, and exactly typed required fields:

```text
x/y/z/intensity  FLOAT32 count 1
channel_id       UINT32  count 1, each value 0..31
timestamp        UINT32  count 2, absolute ns low/high words
```

The adapter preserves input point order and geometry. It performs no TF,
coordinate transform, deskew, sorting, timestamp synthesis, or ring synthesis.
It rejects timestamps earlier than the first point, acquisition-order
regressions, and scan spans over 120 ms.

Output `/lio/points_raw` is SensorData QoS and has the committed FAST-LIO2
schema `x/y/z/intensity FLOAT32`, `ring UINT8`, and `t UINT32`. `t` is
nanoseconds relative to the first point; the header stamp is that first
point's absolute sensor timestamp, and the frame remains `lio_lidar_link`.

## Frames and starting extrinsic

All rotations are identity:

```text
base_link -> lio_lidar_link  [0.120,  0.000, 0.333] m
base_link -> imu_link        [0.012,  0.002, 0.067] m
imu_link  -> lio_lidar_link  [0.108, -0.002, 0.266] m
```

The last translation matches the current FAST-LIO2 starting extrinsic. Axis
orientation and time alignment are not calibration evidence and must be
verified with live stationary/motion data before FAST-LIO2 use.

## Validation completed

- Focused adapter/Isaac/static suite: **28 passed**.
- Isolated `/tmp` build: `rf2o_laser_odometry` and `robot_odometry` finished.
- Installed `robot_odometry` test result: **73 tests, 0 failures/errors/skips**.
- Python compilation and scoped fatal-error lint passed; `git diff --check`
  is part of the final commit check.
- Startup amendment: the focused config/factory/registry regression suite is
  **27 passed**. It executes the installed Isaac 6.0.1 registry module without
  starting Kit, resolves the exact OS1 entry, checks the exact `Lidar.create`
  keyword arguments with a faithful factory double, and checks named failures
  for an invalid config and variant. Python compilation and `git diff --check`
  passed. No Isaac or ROS process was started by the amendment coder.
- A broad non-Isaac/non-ROS Isaac test pass found and repaired one temporary
  `run()` signature regression. Two current-HEAD low-obstacle script/test
  mismatches remain unrelated and were not changed by this task.

These results do not prove actual OS1 profile loading, fields, point count,
ring range, timestamps, frequency, orientation, or non-empty GPU output.

## Required sensor-only preflight and STOP conditions

Use a fresh ROS domain with no Nav2, FAST-LIO2, reset, or goal process. Start
the default-off adapter explicitly, then run only Isaac with the OS1 profile,
front/third-person cameras off, dynamic obstacles off, and a bounded step
count. Capture several consecutive raw and adapted scans in `/tmp`.

The preflight must record:

1. exact raw field names, offsets, datatypes, counts, point/row step;
2. nonzero points per scan, 10 Hz cadence, finite range distribution;
3. raw `channel_id` min/max/unique count and absolute timestamp low/high
   reconstruction, order, span, and header relation;
4. adapted `ring` min/max/unique count, `t` min/max/order, first-point header,
   frame, field schema, point count, and SensorData QoS;
5. stationary SENSOR-frame orientation sanity and continued unchanged `/scan`
   contract if its existing conversion process is present.

Reviewer rerun boundary: use a fresh domain and an isolated install sourced
from this commit. Start only the default-off adapter opt-in and the bounded
Isaac producer (two terminals, same `review_domain`):

```bash
review_domain=231
ROS_DOMAIN_ID="${review_domain}" ros2 launch robot_odometry \
  ouster_pointcloud_adapter.launch.py enabled:=true

review_domain=231
ROS_DOMAIN_ID="${review_domain}" ./scripts/run_isaac.sh \
  --headless \
  --max-steps 1800 \
  --camera-profile off \
  --no-third-person-camera \
  --no-dynamic-obstacles \
  --lio-lidar-profile OS1_REV6_32ch10hz512res
```

The rerun ends after recording the raw/adapted sensor contract listed above.
Do not start FAST-LIO2, Nav2, reset, a route goal, fusion, navigation, or any
evidence/qualification campaign. The previous pre-topic config fatal must be
absent and `/lio/points_raw_isaac` must exist before inspecting data fields.

STOP without starting FAST-LIO2 if the asset/profile cannot load, the cloud is
empty, required auxiliary fields are missing/wrongly typed, channel IDs are
not directly `0..31`, timestamps cannot support the stated absolute/relative
contract, or the cloud is WORLD/COMPENSATED. Stop only the owned processes.

After this preflight passes, the next separate packet may start FAST-LIO2 as a
TF-disabled odometry shadow after physical reset, using `/lio/points_raw` and
`/imu/data`. It must first verify axes/extrinsic, time alignment, initialization,
deskew, output continuity/covariance, compute load, drift, and restart per
episode; no EKF or navigation promotion is implied.
