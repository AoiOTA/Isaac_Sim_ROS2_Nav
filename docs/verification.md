# Verification status

This is an evidence ledger, not a claim that every acceptance item in
`plan.md` is complete. Results below were observed on 2026-07-10 through 2026-07-12
with Isaac Sim 6.0.1.0, ROS 2 Jazzy, Fast DDS, Nav2 1.3.12, and an RTX 4090.
Generated Kit logs and raw experiment captures remain outside normal Git
history. The curated `warehouse_v1` map bundle is versioned; its large
`.posegraph` is stored through Git LFS.

## Runtime evidence obtained

| Area | Observed evidence | Status and limit |
| --- | --- | --- |
| Stage/runtime | Project Stage composed the official warehouse and Jackal overlay, found one `/PhysicsScene`, and reached the standalone ready loop without `Error`, `Traceback`, or unattached-render-product messages in the checked run | smoke verified; not a long-duration soak |
| RTX LiDAR | 91 `/lidar/points_raw` messages in 10 wall seconds: `9.10 Hz`; each observed cloud had 41,363 points, about 993 KiB, frame `lidar_link` | nominal 10 Hz rate verified |
| LaserScan projection | 55 `/scan` messages in 6 wall seconds: `9.08 Hz`; 720 beams, frame `base_link`, all ranges finite in that sample | rate/frame/shape verified; full projection-quality metrics not measured |
| Ideal drive | A `+0.2 m/s` command for 3 seconds moved Ideal odometry X by approximately `+0.517 m` | forward sign and motion path verified; full control characterization not run |
| Low-speed control | After removing extra wheel-joint friction, a `+0.02 m/s` command for 5 seconds traveled `0.09981 m` | low-speed wake/motion path verified; one manual sample only |
| Idle stability | With no effective non-zero command, the command watchdog put the articulation to sleep; stationary tests showed exact rest, zero wheel-joint velocities, and no Ground Truth drift for more than 10 seconds after the navigation goal | idle watchdog regression verified; not a long-duration soak |
| Deterministic Reset | Final experiment batches completed 13 reset/reseed cycles across Ideal static, Ideal dynamic, Ideal long-range, and Realistic static runs. Recovery required a post-reset scan, `/simulation/localization_seeded`, strictly newer `map -> odom`, fresh stamped GT/odom, spawn alignment, and a stable base pose | multi-run isolation gate verified; destructive/cancel-failure injection was not run |
| Ideal Mapping | SLAM produced `/map` at `0.05 m` resolution, observed live size `395 x 604`, with `map -> odom` and a complete base/sensor TF chain | mapping/save smoke verified; full-route map-quality acceptance not run |
| Curated map artifacts | `warehouse_v1` OccupancyGrid (`.yaml`/`.pgm`/preview) and matching serialized Pose Graph (`.posegraph`/`.data`) are distributed as one repository baseline; the 27 MB `.posegraph` uses Git LFS. The manifest records sizes, SHA256 digests, `398 x 606` dimensions, origin, and calibration; `preflight.sh` verified all four runtime artifacts | clone-reproducible baseline verified; other generated maps remain ignored until deliberately curated |
| Map Pose calibration | At the resting reset USD pose `[4, 0, 0.0635, 0°]`, Mapping reported `map -> odom = [0, 0, 0]` and Ideal odometry was at reset identity; `mapping_start.map` is recorded as `[0, 0, 0°]`, `calibrated: true` | coordinate pair recorded for `warehouse_v1`; three-start repeatability remains pending |
| Localization + Ground Truth | The saved Pose Graph loaded in multiple Ideal sessions and one Realistic session. All 13 final runs passed the runner's post-reset map/base alignment and GT freshness gates | Ideal/Realistic runtime alignment exercised; three independent cold starts per mode with quantified pose spread remain pending |
| Immutable navigation map | In Localization/Navigation, `map_server` was active and the only `/map` publisher at `398 x 606`, while SLAM Toolbox published its changing diagnostic grid only on `/slam_toolbox/map`; Nav2 consumed the saved transient-local map | publisher isolation verified and used by the final dynamic batch |
| Nav2 activation | Activation waited for latched `/map`, fresh non-zero `/clock`, fresh `/scan` and `/odom`, and stable, freshly stamped `map -> odom`; lifecycle activation completed. The gate remains alive after `STARTUP`, eliminating the prior process-exit handler race | readiness/lifecycle smoke verified |
| Transactional Reset recovery | A live Navigation reset returned success only after its queued reset/clear futures resolved and emitted one recovery epoch. The gate logged one ordered cancel/pause/clear/reseed/wait/resume sequence; all six Nav2 managed nodes returned active with no duplicate transition. Unit tests also inject old futures, timeout/service failures, overlapping resets, and stale scan epochs | live auto-reseed recovery verified; destructive service-failure runtime injection remains pending |
| RViz workflows | `mapping.rviz`, `localization.rviz`, and `navigation.rviz` each loaded under Xvfb for eight seconds with no plugin/config error. The integrated Mapping launch opened managed RViz, loaded final Best-Effort sensor QoS before `/scan`, and shut down without orphan RViz processes | all three configs load-tested; extended manual GUI sessions across every display toggle are not a soak |
| Mapping Teleop | A real PTY run produced `+0.30 m/s` on W, returned to zero after the 0.18-second deadman, and Q logged a final zero. The integrated Mapping launch started the separate terminal and its wrapper stopped the identity-checked node on top-level shutdown; no Teleop/ROS/RViz process remained | input/deadman/shutdown smoke verified; not a human full-map driving acceptance |
| RViz navigation interaction | Navigation config contains the official Navigation 2 panel and GoalTool, SetInitialPose, dual costmaps, paths, footprints, and collision zones; config tests reject a project `/goal_pose` bridge and lock topic/QoS values. Live Nav2 goals completed through the same action server used by the panel | plugin/config and action path verified; final click-by-click human acceptance was bounded to config/load and navigation smoke |
| Ideal NavigateToPose | Final static batch: 4/4 at `[1, 0, 0°]`, GT errors `0.178–0.188 m`; long-range run: 1/1 at `[3, 0, 0°]`, GT path `2.807 m`, error `0.193 m`. Every run returned Nav2 status 4 and met the final-still gate | deterministic smoke/recovery evidence; not the plan's multi-goal statistical matrix |
| Realistic odometry | `/wheel/odom` and EKF `/odom` both observed at about `45 Hz`; `/odom` had one publisher (`ekf_filter_node`) | Wheel Odom + IMU + EKF ownership smoke verified |
| Realistic Navigation + Reset | Four Realistic static runs succeeded after repeated Wheel Odom/EKF resets; GT errors were `0.175–0.187 m`, odom path lengths `0.818–0.831 m`, final linear speeds about `-0.0015 m/s`, and every run met the still gate. `/odom` had exactly one publisher, `ekf_filter_node` | 4/4 deterministic smoke verified; broader drift and varied-goal statistics not run |
| TF structure | All seven configured static sensor/camera pairs and dynamic wheel links were observed; Ideal and Realistic `odom -> base_link` ownership matched the selected mode | checked runtime snapshots; always recheck after mode changes |
| Dynamic navigation | Two physical one-shot obstacles (crossing and oncoming) were verified against the ROS scenario by enabled state, SHA256, IDs, shape, dimensions, transformed endpoints, duration, and `repeat`. The final 4-seed batch succeeded 4/4 with GT errors `0.168–0.186 m`, final still state, and collision/localization/TF observations | one deterministic two-obstacle baseline verified; not the required diverse dynamic matrix or a true-contact test |
| Experiment contracts | Static, dynamic, and Realistic final batches wrote strict CSV/JSON reports; static uses the fixed warehouse with `static: []`, dynamic verifies its physical contract, and the navigation runner rejects incremental workflow descriptors | baseline batches executed; full statistical matrix remains pending |
| Runtime hygiene | `preflight.sh` passed with Jazzy, Domain 42, Fast DDS, map hashes/LFS, assets, GPU, three RViz files and Teleop package. Safe cleanup removed stale registered metadata and current-user Fast DDS SHM only after stopping the CLI daemon and proving no Fast DDS mapping remained | normal and dry-run paths tested; the diagnostic process list may also show the IDE/Codex process because its working directory is the repository, but it is never a managed kill target |

The measured RTX and ROS rates are wall-time observations from one run. They are
useful regression baselines, not confidence intervals or performance guarantees
for another GPU, renderer, or workload.

## 2026-07-12 MPPI comparison

The same headless Isaac Ideal Localization session, `warehouse_v1`, and map goal `[3.0, 0.0]` were used while the host CPU governor was `powersave`. Durations below are comparable navigation log spans, not universal wall-time guarantees.

| Controller / MPPI | Localization processing | Outcome | Missed-control evidence |
| --- | --- | --- | --- |
| 20 Hz, 40×0.05 s, batch 1500 | every scan, 0.10 s minimum | succeeded, about 15.14 s | 9 warnings; worst observed achieved rate 4.615 Hz |
| 20 Hz, 40×0.05 s, batch 1000 | every scan, 0.10 s minimum | succeeded, about 16.32 s | 5 warnings; worst 4.285 Hz |
| 20 Hz, 40×0.05 s, batch 750 | every scan, 0.10 s minimum | succeeded, about 15.72 s | 12 warnings; worst 4.285 Hz |
| 10 Hz, 20×0.10 s, batch 1000 | every scan, 0.10 s minimum | succeeded, about 5.86 s | 3 warnings |
| **10 Hz, 20×0.10 s, batch 1000** | **every second scan, 0.20 s minimum** | **succeeded, about 5.75 s** | **0 warnings; `/cmd_vel_nav` 10.000 Hz, maximum observed interval about 0.182 s** |
| 20 Hz, 40×0.05 s, batch 1000 | every second scan, 0.20 s minimum | succeeded, about 18.23 s | many warnings (approximately 18), maximum command interval about 0.323 s, and stale-scan safety stops |

The selected row preserves a two-second MPPI horizon while reducing per-cycle work and SLAM contention. Collision Monitor still subscribes to the original nominal 10 Hz `/scan`; `throttle_scans` changes SLAM Toolbox processing, not the safety sensor stream. The 20 Hz retest with reduced localization load was still worse, so the final default was chosen from measured behavior rather than batch size alone.

## Nav2 1.3.12 Smac inflation diagnostic

During planner configuration, Nav2 1.3.12 emits the generic
`Inflation layer either not found or inflation is not set sufficiently` ERROR.
For `SmacPlanner2D` in this version, this is an upstream false diagnostic rather
than evidence of this project's invalid inflation radius: the upstream
[`SmacPlanner2D` source](https://github.com/ros-navigation/navigation2/blob/1.3.12/nav2_smac_planner/src/smac_planner_2d.cpp#L113-L118)
intentionally calls the shared collision checker with radius mode enabled and
`possible_collision_cost=0.0`, while the shared
[`GridCollisionChecker` source](https://github.com/ros-navigation/navigation2/blob/1.3.12/nav2_smac_planner/src/collision_checker.cpp#L45-L68)
logs the ERROR for any non-positive value before returning for radius mode.

Both project costmaps do contain `nav2_costmap_2d::InflationLayer` with an
inflation radius of `0.55 m`. The padded rectangular Jackal footprint has a
circumscribed radius of approximately `0.34 m`, so the configured radius is
larger than the footprint requirement. The subsequent 1 m Ideal navigation
goal planned and completed. Keep treating this message as a version-specific
known diagnostic only while those footprint, plugin, radius, and Nav2 version
facts remain unchanged.

## Automated test evidence

The final delivery verification on 2026-07-12 recorded:

- preflight: PASS, including `warehouse_v1` size/SHA256 and Git LFS hydration;
- ROS build: 9 packages completed;
- root/pure suite: 289 passed, 5 Isaac/ROS-marked tests deselected;
- ROS colcon suite: 292 tests, 0 errors, 0 failures, 0 skipped;
- Isaac/USD marker suite: 3 passed, 45 non-Isaac tests deselected;
- the two real-rclpy activation-gate integration cases passed in 20 consecutive loops (40 case executions total);
- all three RViz configs passed seven structural/QoS assertions and independent Xvfb load smokes;
- repository index set comparison covered all 227 deliverable files with zero missing or extra entries;
- `git diff --check` passed.

Re-run the same gates after any change; terminal output is authoritative if
counts change as tests are added:

```bash
./scripts/preflight.sh
./scripts/build_ros2.sh
./scripts/test.sh --with-isaac
```

For isolated diagnosis:

```bash
python3 -m pytest -m "not isaac and not ros"

cd ros2_ws
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

The unified script handles the stock Isaac environment's missing-pytest case as
described in `docs/development.md`.

## Runtime reproduction checks

With an Ideal Mapping pair running:

```bash
# terminal A
./scripts/run_isaac.sh \
  --navigation-mode mapping \
  --mode ideal \
  --headless

# terminal B
./scripts/run_ros.sh mapping odometry_mode:=ideal interactive:=false
```

Capture timing, frames, and ownership:

```bash
ros2 topic hz /clock
ros2 topic hz /lidar/points_raw
ros2 topic hz /scan
ros2 topic hz /odom

ros2 topic info --verbose /odom
ros2 topic info --verbose /tf
ros2 topic info --verbose /tf_static
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_tools view_frames
```

For Realistic Mapping, restart both processes with the same odometry selection:

```bash
./scripts/run_isaac.sh \
  --navigation-mode mapping \
  --mode realistic \
  --headless

./scripts/run_ros.sh mapping odometry_mode:=realistic interactive:=false
```

Then require one `/odom` publisher (EKF), observe both odometry streams, and
exercise Reset:

```bash
ros2 topic hz /wheel/odom
ros2 topic hz /odom
ros2 topic info --verbose /odom

ros2 param set /isaac_navigation_sim reset_seed 4242
ros2 param set /isaac_navigation_sim reset_pose_name mapping_start
ros2 service call /simulation/reset std_srvs/srv/Trigger '{}'
```

After the service returns, wait for fresh `/odom` and verify the TF tree again;
do not infer readiness solely from `success: true`.

For a manual reproduction of the final Ideal static baseline, start the
localization-mode Isaac/ROS pair shown in `README.md`, wait for the activation
gate to report completion, and use the official RViz Nav2 GoalTool to drag a
goal near `[1.0, 0.0, 0°]`. For a scriptable equivalent:

```bash
ros2 action send_goal /navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback
```

Action success at the configured tolerance is the smoke criterion. Record
Ground Truth and `map -> base_link` separately; do not expect the final X value
to equal exactly `1.0 m` when the position tolerance is `0.25 m`.

## Explicitly not yet accepted

| Plan capability | Current state/blocker |
| --- | --- |
| Localization cold restart | multiple Ideal sessions and one Realistic session loaded and navigated, but the required three independent cold starts per mode and quantified Map-pose spread have not been accepted |
| Map-frame Ground Truth | all final batches passed runtime alignment/freshness gates; three-cold-start spread and longer statistical calibration evidence are not complete |
| Nav2 end-to-end navigation | Ideal/Realistic 1 m batches and one Ideal 3 m goal succeeded; broader start/goal pairs, recovery fault injection, stop/slowdown characterization, and statistical acceptance remain untested |
| Static obstacle statistics | Ideal 4/4 and Realistic 4/4 smoke batches succeeded, but the current YAML is one fixed warehouse with no authored static obstacles, not the required 10 start/goal pairs x 5 layouts x 4 seeds = 200 trials |
| Dynamic obstacle statistics | the two-obstacle baseline succeeded 4/4, but the required at-least-90% claim across crossing, oncoming, following, blocking, size, and speed variants has not been measured; a true-contact failure sample is also absent |
| General navigation statistics | not run; required target is at least 90%, with path-length deviation at most 20% |
| Automated experiment matrix | runner, reset recovery gate, runtime identity checks, scenario schema, metrics, and reports are exercised by the three final 4-seed batches; this is still a baseline, not the complete plan matrix |
| Incremental-map benefit | bringup and a strict three-map offline comparator exist, but no real changed-region trial has produced baseline/full/incremental maps and same-definition timings to prove at least 30% improvement, changed-cell recovery, or old-region preservation; updated-map Localization/Nav2 is also pending |
| Long-duration stability | no soak-duration result is recorded |
| Custom robot migration | project/defaultPrim, static TF, Xacro, Wheel Odom, and Nav2 inputs are parameterized with a fail-fast template; no real custom USD, measured calibration, or full-chain evidence is available |

The final deterministic batches are valid smoke, Reset-isolation, and baseline
avoidance evidence, but they are not the plan's broad statistical population.
Do not convert 4/4 into a general 100% static/dynamic success-rate claim. The
current dynamic Map/USD coordinates and runtime identity contract match the
calibrated transform only for `warehouse_v1` and `mapping_start`.

## Final acceptance sequence

1. Repeat cold-start Localization at least three times in Ideal and Realistic
   modes; verify single TF/odom ownership and quantify map-pose spread.
2. Repeat Ground Truth alignment across those starts, retain the no-drift check,
   and confirm that no navigation node subscribes to Ground Truth.
3. Expand Navigation beyond the successful Ideal/Realistic baselines; cover
   varied start/goal pairs, recovery fault injection, velocity smoothing,
   stop/slowdown behavior, and intentional physical collision observability.
4. Expand and execute the static and dynamic statistical matrices, preserve raw
   reports under `data/`, and publish only summaries backed by those reports.
5. Execute the changed-region incremental workflow, run the three-map comparator,
   then validate Localization and Nav2 with the accepted updated map.
6. Supply and calibrate the real custom robot asset, then execute its Ideal,
   Realistic, Ground Truth, Reset, Localization, and Nav2 acceptance sequence.
