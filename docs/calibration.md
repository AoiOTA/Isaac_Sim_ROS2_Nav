# Map Pose calibration

`isaac_sim/configs/spawn_poses.yaml` stores two different poses for the same
physical starting point:

- `usd`: the robot pose in the Isaac Stage, used to spawn/reset physics;
- `map`: the pose of `base_link` in the saved ROS map, used for Localization,
  Ground Truth alignment, repeatable experiments, and incremental mapping.

## Current calibration record

The current source records this pair for repository baseline `warehouse_v1`:

```text
USD base pose: [4.0, 0.0, 0.0635], yaw 0°
Map base pose: [0.0, 0.0], yaw 0°
Initial-pose standard deviation: 0.05 m, 5°
Calibration date: 2026-07-10
```

`0.0635 m` is the measured resting `base_link` height on the warehouse floor.
The earlier `0.10 m` value left the chassis above its resting contact pose and
introduced a gravity/contact settling transient, so it must not be reused as
the reset height.

At the reset Mapping pose, the observed `map -> odom` transform was
`[0, 0, 0]`; Ideal `odom -> base_link` was reset to identity, yielding the
recorded Map base pose. The `warehouse_v1` OccupancyGrid and serialized Pose
Graph were generated and inspected together. This curated bundle is distributed
with the repository; the large `.posegraph` uses Git LFS. Run `git lfs pull`
after cloning. `preflight.sh` verifies all four files against the committed byte
sizes and SHA256 digests before runtime use.

Multiple Ideal sessions plus one Realistic session have since loaded the bundle,
and the final multi-reset navigation batches succeeded. The stricter requirement
for three independent cold starts in each odometry mode, quantified pose spread,
and broad statistical navigation acceptance remains separate in
`docs/verification.md`.

The rest of this document is the procedure for reproducing or replacing that
calibration. If the warehouse, map origin, Pose Graph, or USD spawn changes,
first set the affected Map Pose back to `calibrated: false`.

## 1. Produce the baseline artifacts

Start from the fixed USD pose in Ideal mode:

```bash
./scripts/run_isaac.sh --navigation-mode mapping --mode ideal
./scripts/run_ros.sh mapping odometry_mode:=ideal
```

Collect the full route slowly, include rotations and loop closure, inspect the
result for tearing or duplicated walls, then save both representations with one
version:

```bash
./scripts/save_map.sh warehouse_v1
```

The following four files are one logical version and must be retained together:

```text
data/maps/occupancy/warehouse_v1.yaml
data/maps/occupancy/warehouse_v1.pgm
data/maps/posegraphs/warehouse_v1.posegraph
data/maps/posegraphs/warehouse_v1.data
```

Do not calibrate against an OccupancyGrid from one run and a Pose Graph from
another.

## 2. Bootstrap localization without weakening the source gate

When recalibrating, Localization intentionally refuses the tracked file while
`map.calibrated: false`. Make a temporary copy outside the repository, enter an
initial estimate in that copy, and set only the temporary copy to `true`:

```bash
cp isaac_sim/configs/spawn_poses.yaml /tmp/spawn_poses_calibration.yaml
```

Edit `/tmp/spawn_poses_calibration.yaml`; do not change the tracked file yet.
For a replacement map, even the old measured `[0.0, 0.0, 0.0°]` value is only
a bootstrap estimate until remeasured against that new Pose Graph. Keep Ground
Truth disabled during this procedure so a wrong bootstrap transform cannot be
recorded as truth.

Point both processes at the same temporary source:

```bash
export ISAAC_NAV__SPAWN__POSES_FILE=/tmp/spawn_poses_calibration.yaml
export ISAAC_NAV_SPAWN_POSES=/tmp/spawn_poses_calibration.yaml

# terminal A
./scripts/run_isaac.sh --navigation-mode localization --mode ideal

# terminal B
./scripts/run_ros.sh localization \
  odometry_mode:=ideal \
  posegraph_file:="$PWD/data/maps/posegraphs/warehouse_v1"
```

Localization needs both representations from step 1. With the matching
`warehouse_v1` basename shown above, `run_ros.sh` infers
`data/maps/occupancy/warehouse_v1.yaml`; use an explicit
`map_file:=/path/to/map.yaml` argument if the OccupancyGrid and Pose Graph names
differ. The Map Server publishes that saved grid on `/map`, while SLAM Toolbox
loads the Pose Graph to publish `map -> odom`; its diagnostic grid is isolated
on `/slam_toolbox/map`.

Use RViz scan/map overlap and `2D Pose Estimate` if the bootstrap estimate does
not converge. Keep the physical robot at the exact `mapping_start.usd` pose;
after localization has settled, record the transform:

```bash
ros2 run tf2_ros tf2_echo map base_link
```

Record translation X/Y and yaw in degrees. This is the measured
`mapping_start.map` pose. It is not the USD X/Y, and `map -> odom` by itself is
not the requested pose unless `odom -> base_link` is exactly identity.

## 3. Verify repeatability

Repeat a full stop/start and localization at least three times, always restoring
the same USD pose. For each run verify:

- `/scan` visually overlaps the saved map;
- `map -> odom` remains available without jumps;
- `map -> base_link` settles near the same X/Y/yaw;
- `/odom` and `odom -> base_link` each have a single owner.

As the project calibration gate, require the cold-start results to agree within
`0.05 m` translation and `3°` yaw, matching the current Nav2 activation TF
stability tolerances. If they do not, fix the map, pose estimate, or TF
ownership; do not average an unstable result and mark it calibrated.

## 4. Promote the measured pose

Only after the repeatability check, update the tracked entry:

```yaml
spawn_poses:
  mapping_start:
    usd:
      position: [4.0, 0.0, 0.0635]
      yaw_deg: 0.0
    map:
      position: [MEASURED_X, MEASURED_Y]
      yaw_deg: MEASURED_YAW_DEG
      calibrated: true
      position_stddev_m: MEASURED_OR_CONSERVATIVE_STDDEV
      yaw_stddev_deg: MEASURED_OR_CONSERVATIVE_STDDEV_DEG
```

Clear the temporary overrides and validate both Localization and Ground Truth
gates against the real source:

```bash
unset ISAAC_NAV__SPAWN__POSES_FILE ISAAC_NAV_SPAWN_POSES

./scripts/run_isaac.sh \
  --navigation-mode localization \
  --mode ideal \
  --validate-only

ISAAC_NAV__GROUND_TRUTH__ENABLED=true \
  ./scripts/run_isaac.sh \
  --navigation-mode localization \
  --mode ideal \
  --validate-only
```

Then rerun Localization from a cold start using the tracked file before
attempting Navigation or experiments.

## 5. Realign dynamic-obstacle coordinates

The committed Isaac dynamic coordinates use the current calibrated Map Pose
`[0, 0, 0°]` at USD base pose `[4, 0, 0.0635, 0°]`. A different measured Map
Pose changes the Map/USD transform. For planar poses the implementation uses:

```text
map_T_usd = map_T_base_start * inverse(usd_T_base_start)
map_T_obstacle = map_T_usd * usd_T_obstacle
```

After calibration, update `isaac_sim/configs/experiments/dynamic.yaml` so every
physical obstacle trajectory maps to the corresponding trajectory in
`ros2_ws/src/robot_experiments/config/dynamic.yaml`. Verify the crossing and
oncoming endpoints in RViz or recorded Ground Truth before collecting dynamic
success statistics. The experiment contract also requires matching obstacle
IDs, shapes, XY dimensions, durations, and explicit boolean `repeat` values;
changing a trajectory from one-shot (`false`) to back-and-forth (`true`) on only
one side is a configuration error.

## 6. Version and evidence discipline

The calibration commit or handoff record must identify:

- map and Pose Graph version;
- unchanged USD start pose;
- measured Map X/Y/yaw;
- number and spread of cold-start trials;
- TF ownership check;
- whether dynamic-obstacle coordinates were re-aligned.

Changing the warehouse layer, map origin, spawn USD pose, robot base frame, or
serialized Pose Graph invalidates the calibration and requires the Map Pose to
return to `calibrated: false` until remeasured.
