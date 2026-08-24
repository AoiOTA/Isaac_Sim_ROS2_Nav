# V7.3 Phase 1A stereo producer handoff (2026-08-24)

## Result and boundary

- Added the explicit `stereo_vio` Camera profile. It is opt-in only: the GUI
  default remains `monitoring`, the headless default remains `off`, and every
  older profile still selects only the legacy `front` Camera.
- Explicit `--camera-profile stereo_vio` selects only `left` and `right`.
  Each owns a distinct RTX Camera prim and Render Product at 640x360 with a
  configured 20 Hz tick rate and identical optics/exposure. The existing
  front, stereo frame poses, IMU, odometry, TF ownership, Nav2, Grid,
  Integration, and Module2 were not changed.
- This coder task performed code/static validation only. It did not start
  Isaac, ROS, cuVSLAM, Nav2, or a recorder and is not live or formal evidence.

## Producer graph and ROS schema

One `/World/Graphs/ROS2StereoCamera` graph contains a single
`OnPlaybackTick` feeding independent left/right RGB branches, left depth, and
one paired `ROS2CameraInfoHelper`:

| Output | Topic | Frame | Source |
| --- | --- | --- | --- |
| left RGB | `/camera/left/image_raw` | `camera_left_optical_frame` | left RP |
| left aligned depth | `/camera/left/depth/image_raw` | `camera_left_optical_frame` | same left RP |
| left CameraInfo | `/camera/left/camera_info` | `camera_left_optical_frame` | paired helper, left RP |
| right RGB | `/camera/right/image_raw` | `camera_right_optical_frame` | right RP |
| right CameraInfo | `/camera/right/camera_info` | `camera_right_optical_frame` | paired helper, right RP |

There is no right depth or stereo depth-point publisher. The paired helper
receives `renderProductPathRight`, `frameIdRight`, and `topicNameRight`; both
CameraInfo topics are absolute because the helper exposes one common namespace.
Graph teardown deduplicates the shared graph path, while each Camera runtime
releases its own Render Product and prim once.

The unchanged robot static transforms place `camera_left_link` at `+0.060 m`
and `camera_right_link` at `-0.060 m`, giving the configured 0.120 m baseline.
The installed paired helper is expected to derive `P_left[3] = 0` and
`P_right[3] = -fx * 0.12` from those actual USD poses. Static tests verify the
pose/profile/helper wiring and formula; the emitted matrices are not yet live
verified.

## Static validation

From this worktree, with the current source first and pytest cache disabled:

```bash
python3 -m py_compile \
  isaac_sim/src/sensors/sensor_factory.py \
  isaac_sim/graphs/camera_graph.py \
  isaac_sim/graphs/ros_contract.py \
  isaac_sim/tests/test_camera_contracts.py \
  isaac_sim/tests/test_graph_contracts.py
PYTHONPATH="$PWD" python3 -m pytest -p no:cacheprovider \
  isaac_sim/tests/test_camera_contracts.py \
  isaac_sim/tests/test_graph_contracts.py -q
git diff --check
```

Result: `21 passed`; compilation and diff check passed. Coverage includes
default/legacy selection, distinct Camera prim/RP paths, one shared tick,
left RGB/depth RP identity, no right depth, paired right-side CameraInfo
fields, topics/frames, 0.120 m static baseline/P contract, and exact-once
Render Product/shared-graph cleanup.

## Required 30 s live review

Use an empty ROS domain. Start only the existing Isaac application (no ROS
navigation, VIO, or cuVSLAM) with the explicit stereo profile:

```bash
cd /home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3
ROS_DOMAIN_ID=243 ./scripts/run_isaac.sh \
  --headless \
  --environment-usd kujiale_0026_A_to_B_door_open.usd \
  --navigation-mode localization \
  --mode ideal \
  --spawn-pose long_route_start_g1 \
  --camera-profile stereo_vio \
  --no-dynamic-obstacles \
  --no-third-person-camera
```

After the ready line, record the bounded camera-only evidence in another
terminal:

```bash
cd /home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3
source scripts/setup_ros_env.sh
export ROS_DOMAIN_ID=243
LIVE_DIR="/tmp/v7_3_phase1a_stereo_$(date -u +%Y%m%dT%H%M%SZ)"
timeout --signal=INT 30s ros2 bag record -o "$LIVE_DIR" \
  /camera/left/image_raw /camera/left/depth/image_raw \
  /camera/left/camera_info /camera/right/image_raw \
  /camera/right/camera_info /clock /tf_static
```

Reviewer must measure actual rates, pairwise stamps, drop/transport-loss
counts, depth encoding/alignment, frames, K/R/P matrices, baseline agreement,
GPU load, and RTF. Do not claim actual 20 Hz, exact stamp equality,
rectification, or `P_right[3]` until that run passes.
