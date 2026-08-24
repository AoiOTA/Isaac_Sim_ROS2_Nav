# V6 FAST-LIO2 Jazzy PointCloud2 shadow handoff (2026-08-24)

## Axis-contract amendment (2026-08-24)

The initial identity rotation below has been superseded by the measured Ouster
SENSOR-to-IMU axis contract. `config/ouster_shadow.yaml` now contains the sole
algorithm conversion, fixed row-major LiDAR-to-IMU rotation
`[0,-1,0; 1,0,0; 0,0,1]` (+90 degree yaw). Translation remains
`[0.108,-0.002,0.266]` and online estimation remains false.

The Ouster adapter still copies raw SENSOR XYZ unchanged, and the published
`lio_lidar_link` plus physical static TF remain unchanged. Do not rotate the
adapter output, launch parameters, URDF, or static TF as well; that would
double-apply this internal FAST-LIO basis conversion. The exact full replay
confirmed the early axis correction but later diverged, so FAST-LIO remains
default OFF, TF false, and excluded from EKF/Nav2/control. See
`V6_FAST_LIO2_AXIS_FIX_REPLAY_20260824.md`.

## Result and boundary

- Added one independent `fast_lio2_ros2` package from audited Ericsii
  FAST_LIO_ROS2 commit `2fffc570a25d0df172720bac034fbdb6a13d2162` and
  ikd-Tree submodule commit `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`.
- The upstream GPL-2.0 `LICENSE` is verbatim and package metadata declares
  `GPL-2.0`; `ORIGIN.md` records the exact pins and port delta.
- The port builds only standard `sensor_msgs/msg/PointCloud2`; it has no
  Livox message/SDK or `pcl_ros` dependency. The FAST-LIO2/IKFoM/ikd-Tree
  estimation core is otherwise retained.
- This package is optional and default OFF. It is not referenced by the
  canonical runner, `robot_bringup`, EKF, Grid, Nav2, safety, or control.
  No Isaac sensor, adapter, fusion, navigation, or qualification was added.

## Exact launch contract

The default launch expands but starts no process. Explicit shadow opt-in is:

```bash
source /opt/ros/jazzy/setup.bash
source /absolute/path/to/fast_lio2_install/setup.bash
ros2 launch fast_lio2_ros2 shadow.launch.py enabled:=true
```

Defaults:

```text
PointCloud2 input  /lio/points_raw
IMU input          /imu/data
Odometry output    /lio/odom_shadow
header.frame_id    lio_map_shadow
child_frame_id     base_link
publish_tf         false
path/cloud/map/PCD false
use_sim_time       true
```

Launch arguments parameterize both inputs, odometry output, map/body frames,
TF, and each optional output class. All concrete output topic names are ROS
parameters under `common.*` in `config/ouster_shadow.yaml`.

With `publish.publish_tf=false`, the node does not construct a TF broadcaster;
it never creates a static broadcaster. Odometry pose and finite-sanitized pose
covariance are filled before publication. Twist remains explicitly zero because
FAST-LIO2 does not expose it through this port, with `1e6` diagonal twist
covariance to mark it unavailable rather than precise.

## Initial Ouster/IMU configuration

- Expected PointCloud2 fields are float32 `x/y/z/intensity`, uint8 `ring`, and
  uint32 `t`; `t` is interpreted as nanoseconds. Scan rate is 10 Hz and blind
  range is 0.3 m.
- LiDAR-to-body translation is `[0.108, -0.002, 0.266]`; the fixed internal
  SENSOR-to-IMU rotation is +90 degree yaw, and online extrinsic estimation is
  disabled. The early motion direction is replay-verified, but full-route
  stability is not established.
- The existing 60 Hz corrected IMU is the initial shadow candidate. Promotion
  may require a 120 or 200 Hz IMU after measured initialization/deskew tests.

## Reset/teleport boundary

No speculative reset API was added. Start FAST-LIO2 only after the physical
episode reset, or restart the shadow process for every episode. Never carry its
local map across an Isaac teleport. The audited candidate did not provide a
safe, directly testable reset API suitable for exposure here.

## Validation

- Clean `/opt/ros/jazzy` build: one package finished under
  `/tmp/fast_lio2_jazzy_verified.24hWGJ`; no project overlay, apt, or pip was used.
- `colcon test-result`: 6 tests, 0 errors/failures/skips. This includes a direct
  C++ synthetic Ouster PointCloud2 preprocessing test and three static/launch
  tests for pins/license, removed dependencies, defaults, topics/frames,
  TF construction, odometry/covariance ordering, and launch expansion.
- Installed `ros2 launch ... --show-args` expanded all default-OFF arguments.
- A bounded initialization on an initially empty domain 67 started and
  subscribed without sensor data. The graph listed only `/clock`, both inputs,
  `/lio/odom_shadow`, and ROS internal topics; `/tf`, `/tf_static`, and optional
  path/cloud/map topics were absent. The owned process was stopped afterward.
- Non-blocking build notes are the upstream Boost bind placeholder notice and
  PCL's missing optional pcap feature/dev policy warning.

## Remaining blockers before useful shadow data

1. There is no Isaac Ouster producer or adapter in this change. A later task
   must provide `/lio/points_raw` and prove the exact field types, timestamp
   unit/order, QoS, frame, scan lines, and 10 Hz cadence.
2. The +90 degree axis basis is discrimination-replay verified, but the later
   476--479 s geometry collapse remains unresolved. Translation, time
   alignment, and full-route stability are not promoted calibration evidence.
3. No valid LIO odometry was produced without sensors. Continuity, covariance,
   deskew, map reset, compute load, and drift remain live-unverified.
4. Any EKF promotion is a separate A/B decision. This package must remain
   shadow-only with TF false until real output is measured and approved.

Verdict: **PASS for source pin/license, Jazzy build, unit/static/launch, and
bounded no-sensor initialization only; DEFAULT OFF.** No live navigation,
fusion, odometry accuracy, or formal qualification claim is made.
