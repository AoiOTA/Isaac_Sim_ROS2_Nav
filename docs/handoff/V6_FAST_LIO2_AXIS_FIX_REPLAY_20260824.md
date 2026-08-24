# V6 FAST-LIO2 Ouster axis-contract fix and full replay (2026-08-24)

## Verdict

**AXIS_FIX_CONFIRMED_AND_KEEP_OFF.** The Ouster SENSOR-to-IMU +90 degree yaw
fix corrects the first motion direction and materially reduces the exact early
EKF/wheel error. It does not make the full route stable: the estimate degrades
around 476--479 s and later diverges catastrophically. FAST-LIO remains
default OFF, TF false, shadow-only, and excluded from EKF, Grid, Nav2, safety,
and control.

## Single authoritative conversion

`fast_lio2_ros2/config/ouster_shadow.yaml` is the only algorithm conversion:

```text
mapping.extrinsic_R  [0,-1,0; 1,0,0; 0,0,1]  (+90 degree yaw)
mapping.extrinsic_T  [0.108,-0.002,0.266]
extrinsic_est_en     false
```

The values are YAML doubles because the ROS parameter is a `double_array`.
The Ouster adapter continues to copy raw SENSOR x/y/z exactly; it does no TF,
coordinate rotation, or deskew. `lio_lidar_link` and the physical static TF
remain the published ROS frame contract. Launch adds no extrinsic override and
defaults the whole shadow, planar IMU, and TF publisher OFF. Do not add the
same yaw to the adapter, launch, URDF, sensor frame, or static TF.

## Exact isolated full replay

- Input MCAP:
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_fastlio2_ouster_g2_retry_20260824T035347Z/bag/fastlio_shadow`
- Fresh ROS domain: 220; start offset: 399.0 s; playback rate: 1.
- Published from the MCAP only: `/lio/points_raw`, canonical non-planar
  `/imu/data`, and `/clock`.
- New config was loaded from the clean isolated install. The only runtime
  override was `common.odom_topic:=/lio/odom_axis_fixed_shadow`.
- Recorded only `/lio/odom_axis_fixed_shadow`; no planar IMU adapter, Isaac,
  Nav2, fusion, control, TF, or qualification process was started.
- Artifacts: `/tmp/fastlio_axis_fix_replay.qrEChE/full_replay`; finalized MCAP
  contains 1,662 messages at header stamps
  `300.383810424--522.584176620` s. Median rate is 9.998 Hz, with 208 gaps over
  0.2 s and a 1.399 s maximum gap.

The comparison subtracts each stream's interpolated pose at motion start
`468.112098237` s. At the exact early checkpoints:

| Stamp | axis-fixed vs EKF XY | identity vs EKF XY | axis-fixed vs wheel XY |
|---:|---:|---:|---:|
| 468.816213989 | 0.0300 m | 0.1581 m | 0.0371 m |
| 470.016213989 | 0.0544 m | 0.7738 m | 0.0709 m |

Axis-fixed relative xyz is `[0.1238,-0.0086,-0.0247]` m and
`[0.5117,0.0105,0.0766]` m at those checkpoints, compared with EKF
`[0.1504,0.0052,0]` m and `[0.5646,0.0235,0]` m. This confirms the input-axis
root cause even though replay scheduling causes small run-to-run differences
from the shorter RCA variant.

## Full-motion failure

- Versus EKF, XY error crosses 0.5/1/5/100 m at
  3.872/4.572/10.172/16.772 s after motion start. Versus wheel the crossings
  are 3.872/4.472/10.272/16.772 s.
- Motion-window 3D jumps have P50/P95/max
  `1.579/5.891/11.632` m; 133 jumps exceed 1 m.
- During 476--479 s, EKF-relative XY error has 2.825 m median and reaches
  9.636 m at 479 s; wheel-relative error reaches 9.077 m. This is the bounded
  geometry/observability collapse that follows the corrected early motion.
- At collision stamp `494.533333333`, error versus EKF is 520.82 m XY and
  549.18 m 3D; motion RMSE is 194.19 m XY and 204.49 m 3D. Identity baseline
  motion RMSE was 189.08 m XY and 197.72 m 3D, so the axis fix is not a
  full-route improvement or promotion candidate.
- Over 167.73 s before motion, endpoint drift is 0.1429 m XY with
  `+0.1287` m z change.
- First timestamped `No Effective Points` is scan begin `495.616213989` s,
  after collision; later aggregate warnings begin at `496.716213989` and
  `502.116213989`. These late rejections are escalation evidence, not the
  cause of the corrected early behavior or the first 476--479 s collapse.

## Build and focused validation

- Clean isolated build root: `/tmp/fastlio_axis_fix_replay.qrEChE`; packages
  `rf2o_laser_odometry`, `robot_odometry`, and `fast_lio2_ros2` finished.
- Installed FAST-LIO tests: 7 tests, zero failures/errors/skips.
- Focused Ouster adapter tests: 12 passed; combined source contract check before
  the clean build: 16 passed.
- Tests assert the exact double-array R/T, orthonormality, determinant +1,
  +90 degree yaw, identity source fallback, unchanged raw adapter XYZ, no
  launch duplicate rotation, unchanged zero-rotation physical frame, and
  default-off TF/shadow/planar behavior. `git diff --check` passed.
- Non-blocking warnings are the existing PCL optional pcap warning and Boost
  bind placeholder notice.

## Next bounded action

Keep this committed input-contract correction but keep FAST-LIO OFF. The next
separate task may inspect only the 476--479 s geometry/sensor-density and
observability collapse using the corrected axis basis. Do not also enable the
planar IMU adapter, undo the raw adapter contract, double-rotate TF/URDF, or
tune noise, voxel, timing, navigation, or control in this axis-fix task.
