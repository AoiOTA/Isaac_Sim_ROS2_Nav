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

## Live FAIL and UDP receive-buffer amendment

The bounded live run at
`/mnt/nas_home/Bio_Nav_Data/experiments/runs/v73_phase1a_stereo_20260824T100039Z`
is an **ENGINEERING FAIL**: CameraInfo ran at the configured 20 Hz, while
left RGB, right RGB, and left depth measured 18.68, 19.34, and 13.53 Hz;
depth gaps reached 0.4 s. RTF was 0.9145 and GPU load was 34.5%. Payload
stamps were subsets of the CameraInfo time slots. The same deficit reproduced
for NAS, memory, and per-topic capture, while frame, K/R/P, TF, and depth
content checks passed.

As one bounded hypothesis, the UDP-only Fast DDS transport now requests a
4 MiB receive buffer. The camera graph, factory, profile, QoS, tick,
resolution, and publisher thread are unchanged. Cumulative
`UdpRcvbufErrors` were elevated, but they were not run-scoped, so this code
change does **not** establish loopback UDP loss as the root cause and has no
live PASS evidence.

The next live review must export all three variables below for both the
producer and subscriber processes, then record `/proc/net/snmp`
`UdpRcvbufErrors` immediately before and after the bounded run:

```bash
export ISAAC_NAV_FASTDDS_PROFILE="$PWD/isaac_sim/configs/ros2_bridge/fastdds_udp_only.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="$ISAAC_NAV_FASTDDS_PROFILE"
export FASTDDS_DEFAULT_PROFILES_FILE="$ISAAC_NAV_FASTDDS_PROFILE"
```

Only the run-scoped counter delta together with the repeated topic rates and
gaps can support or reject this transport-loss hypothesis.

## UDP receive-buffer 30 s live recheck

The bounded camera-only A/B at
`/mnt/nas_home/Bio_Nav_Data/experiments/runs/v73_phase1a_udp_buffer_20260824T103230Z`
is an **ENGINEERING LIVE PASS** on Module3
`3e1d63ec92ba9722f25a0c2b7f27acc8fb1592cc`. All three fixed `main` refs
matched the project boundary, the worktree was tracked-clean, domain 230 was
empty, and Linux `net.core.rmem_max` was 4194304. Producer and subscriber both
used the same absolute UDP-only profile through
`ISAAC_NAV_FASTDDS_PROFILE`, `FASTRTPS_DEFAULT_PROFILES_FILE`, and
`FASTDDS_DEFAULT_PROFILES_FILE`. No VIO, Nav2, Integration, Module2, or bag
recorder ran.

The valid 30.000 s direct probe used one rclpy process, BEST_EFFORT KEEP_LAST
depth 100, and an eight-thread `MultiThreadedExecutor`; callbacks recorded only
header stamp, monotonic receive time, and first-message schema. Every one of
the five topics delivered 506 unique, strictly monotonic stamps over the same
25.25 s simulation interval. All five stamp sets were exactly equal, so left
RGB/right RGB and left RGB/depth/info were exact-stamp paired with zero missing
slots. Every topic measured 20.0 Hz in simulation time. Maximum stamp gap was
50.000998 ms: nine intervals per topic were numerically above 50 ms by no more
than 0.998 us, but there were no 100 ms missing-frame gaps. Maximum wall
receive gaps were 81.44 ms left RGB, 88.97 ms depth, 79.02 ms left info,
90.59 ms right RGB, and 79.01 ms right info; none exceeded 100 ms. RTF was
0.84587, above the 0.8 plan threshold. GPU utilization across 32 one-hertz
samples was 40.75% mean and 44% maximum.

Immediate `/proc/net/snmp` reads bracketed the valid probe. `RcvbufErrors`,
`InErrors`, `NoPorts`, `SndbufErrors`, and `MemErrors` all had delta zero;
`InDatagrams` increased by 33499 and `OutDatagrams` by 46698. `IgnoredMulti`
increased by 31; it is a system-wide multicast counter, not evidence of camera
payload loss. Raw snapshots and `udp_counter_delta.json` are in the run
directory.

Evidence integrity note: the first launch was rejected before Isaac by the
existing non-default-domain guard and was corrected with
`ISAAC_NAV_EXPECTED_DOMAIN_ID=230`. A later probe process exited before
creating any subscription because its evidence-helper attribute conflicted
with rclpy; it produced no experimental sample. After a producer-off 1 s
subscriber self-test passed, master explicitly authorized the single valid
replacement capture reported above. The invalid logs remain in the same run
directory and are not included in the metrics.

Compared with the earlier direct result of 510/369/547/528/547 messages and
only 338 common stamps, the 4 MiB state removed the observed RGB/depth stamp
loss in this bounded run and therefore supports keeping the receive-buffer
change. It does not formally prove UDP receive-buffer exhaustion as the root
cause: the earlier run lacked a run-scoped UDP counter and its ad-hoc direct
subscriber implementation was not preserved. This is engineering live
evidence, not formal qualification. Keep the 4 MiB setting and proceed to the
next planned stereo/VIO consumer smoke; do not change
`publish_with_queue_thread` based on this passing A/B.
