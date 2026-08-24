# V6 cuVSLAM RGB-D visual-odometry Shadow handoff (2026-08-24)

## Current result and boundary

- The original canonical R2 pre-goal run stopped with cuVSLAM15 reporting
  persistent CUDA `invalid argument(1)` for valid aligned 320x180 RGB8 plus
  32FC1 metre depth. It produced zero `/visual/odom_shadow` and zero
  `/visual/status`; no reset or route goal was dispatched.
- Official-equivalent RGB-D parameters and a mono8-only variant reproduced the
  same failure. The next isolated factor was therefore depth representation.
- The shadow now converts only 32FC1 metres to 16UC1 millimetres before
  cuVSLAM and sets `depth_scale_factor=1000.0`. Exact failed-bag replay passes:
  tracker initialized once, CUDA invalid-argument count was zero, and both
  official visual outputs contained 215 finite, strictly monotonic samples.
- The feature remains default OFF and still has no EKF, TF, Grid, map,
  Costmap, Nav2, safety, or control influence. Replay is engineering
  discrimination, not Isaac live navigation or formal qualification.

## Minimal implementation

- Installed `isaac_ros_depth_image_proc::ConvertMetricNode` was inspected
  first; it converts in the opposite direction (16UC1 raw depth to 32FC1
  metres), so it cannot satisfy this input contract.
- `robot_odometry/depth_float_to_uint16_node.py` subscribes only to
  `/camera/front/depth/image_raw` with SensorData BestEffort/Volatile
  KEEP_LAST depth 10. It requires 32FC1, little-endian, contiguous data.
  Finite positive metre samples are rounded and clamped to uint16
  millimetres; non-finite and non-positive values become zero.
- It preserves the input header, frame, and stamp and publishes contiguous
  16UC1 on `/camera/front/depth/image_uint16` with the same SensorData QoS.
- `visual_odometry.launch.py` starts this converter only inside the existing
  default-OFF visual launch. cuVSLAM depth input is remapped to the converted
  topic. Mode 2, one camera, RGB8 input, `rectified_images=false`, both visual
  TF flags false, stamps, synchronization, and all other settings are
  unchanged.

## Canonical pre-goal and recorder behavior

- When `V6_VISUAL_ODOMETRY_SHADOW_ENABLED=true`, the canonical session now
  blocks before the episode dispatcher until the bounded visual gate receives
  a newly delivered, stamped, finite odometry message plus official status
  `vo_state=1` with finite nonnegative timings. Official `vo_state=2` or
  timeout stops the run before physical reset or any route goal.
- The disabled path is unchanged and does not run the gate or either visual
  node.
- Camera recorder QoS queues are BestEffort/Volatile KEEP_LAST depth 100 and
  the current rosbag CLI cache is 512 MiB. Both raw and converted depth are
  retained when enabled. The exact rosbag2 transport-loss warning (or its
  absence) is copied to `provenance/rosbag_transport_loss.log` after recorder
  shutdown.
- The failed live bag's apparent 8.445 Hz raw-depth rate was recorder loss,
  not publisher rate: RGB and CameraInfo retained 837 messages at 10 Hz,
  depth retained 707, and rosbag2 reported exactly 130 transport-layer losses.
  The camera graph remains configured at 10 Hz; resolution, rate, and render
  graph were not changed.

## Exact replay result

- Input:
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_grid_phase1_clearance_r2_cuvslam_20260824T001245Z/preflight_session/rosbag/r5_session`
- Fresh empty domain: 231; replayed only `/clock`, RGB, raw depth,
  CameraInfo, `/tf`, and `/tf_static` for 30 seconds.
- Output/evidence: `/tmp/v6_cuvslam_uint16_replay.rQgJAD`.
- Counts: converted depth 215, visual odometry 215, visual status 215,
  healthy status 215, fatal status 0.
- Tracker initialization 1; `Failed to track` 0; CUDA invalid argument 0;
  converter errors 0.
- Converted images were 16UC1/little-endian/contiguous. Converted depth,
  odometry, and status stamps were all strictly monotonic; all odometry fields
  were finite; no visual-frame transform appeared on `/tf` or `/tf_static`.
- Non-blocking unchanged warning: cuVSLAM reports the 100 ms input period as
  above the configured 20 ms image-jitter threshold. It still tracked and
  published every paired replay sample; this task did not change sync/jitter
  settings.

## Validation

- Full `robot_odometry` plus runtime-script suite: 108 passed (the focused
  conversion/launch/gate/recorder selection was 16 passed).
- Isolated `/opt/ros/jazzy` build: 2 packages finished (`robot_odometry`,
  `robot_bringup`) under `/tmp/v6_cuvslam_uint16_{build,install,log}.*`.
- New/modified Python files passed flake8 and pydocstyle; both affected shell
  entrypoints passed `bash -n`; launch/node scripts compiled; `git diff
  --check` passed. The installed converter and launch were used by the replay.

## Exact next R2 live command

Create a fresh strict combined snapshot containing this commit, confirm the
chosen ROS domain is empty, then run one shadow-enabled canonical R2 episode:

```bash
cd /home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3
RUN_DIR="/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_grid_phase1_clearance_r2_cuvslam_uint16_$(date -u +%Y%m%dT%H%M%SZ)"
SNAPSHOT_ROOT="/absolute/path/to/fresh_combined_phase1_snapshot"
ISAAC_ASSET_ROOT=/home/lyb/isaacsim_assets/Assets/Isaac/6.0 \
V6_VISUAL_ODOMETRY_SHADOW_ENABLED=true \
V6_VISUAL_ODOMETRY_WARMUP_SEC=45 \
R5_DOMAIN_ID=229 R5_EPISODE_INDICES=0 R5_EPISODE_SEEDS=7201 \
  ./scripts/run_v6_kujiale_low_obstacles.sh session "${RUN_DIR}" "${SNAPSHOT_ROOT}"
```

The live run must retain the gate JSON, exact transport-loss log, raw and
converted camera counts/rates, finite monotonic visual outputs, and absence of
visual TF. A gate failure remains a shadow-only STOP; no fusion or navigation
success may be claimed from replay alone.
