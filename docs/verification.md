# Verification status

This is an evidence ledger, not a claim that every acceptance item in
`plan.md` is complete. Results below were observed on 2026-07-10 through 2026-07-17
with Isaac Sim 6.0.1.0, ROS 2 Jazzy, Fast DDS, Nav2 1.3.12, and an RTX 4090.
Generated Kit logs and raw experiment captures remain outside normal Git
history. The full-coverage `warehouse_v2` bundle is the current default;
the incomplete `warehouse_v1` bundle remains versioned for historical-result
reproduction. Both large `.posegraph` files are stored through Git LFS.

## Runtime evidence obtained

| Area | Observed evidence | Status and limit |
| --- | --- | --- |
| Stage/runtime | Project Stage composed the official warehouse and Jackal overlay, found one `/PhysicsScene`, and reached the standalone ready loop without `Error`, `Traceback`, or unattached-render-product messages in the checked run | smoke verified; not a long-duration soak |
| RTX LiDAR | 91 `/lidar/points_raw` messages in 10 wall seconds: `9.10 Hz`; each observed cloud had 41,363 points, about 993 KiB, frame `lidar_link` | nominal 10 Hz rate verified |
| LaserScan projection | 55 `/scan` messages in 6 wall seconds: `9.08 Hz`; 720 beams, frame `base_link`, all ranges finite in that sample | rate/frame/shape verified; full projection-quality metrics not measured |
| Ideal drive | A `+0.2 m/s` command for 3 seconds moved Ideal odometry X by approximately `+0.517 m` | forward sign and motion path verified; full control characterization not run |
| Low-speed control | After removing extra wheel-joint friction, a `+0.02 m/s` command for 5 seconds traveled `0.09981 m` | low-speed wake/motion path verified; one manual sample only |
| Idle stability | With no effective non-zero command, the command watchdog put the articulation to sleep; stationary tests showed exact rest, zero wheel-joint velocities, and no Ground Truth drift for more than 10 seconds after the navigation goal | idle watchdog regression verified; not a long-duration soak |
| Deterministic Reset | The final 20-run static and 20-run dynamic Realistic batches completed all reset/reseed cycles. Recovery required a post-reset scan, `/simulation/localization_seeded`, strictly newer `map -> odom`, fresh stamped GT/odom, spawn alignment, stable TF, and all six Nav2 managed nodes active before goal dispatch | 40-run isolation gate verified; destructive service-failure runtime injection was not run |
| Ideal Mapping | SLAM produced `/map` at `0.05 m` resolution, observed live size `395 x 604`, with `map -> odom` and a complete base/sensor TF chain | mapping/save smoke verified; full-route map-quality acceptance not run |
| Curated map artifacts | Full-coverage `warehouse_v2` OccupancyGrid and matching serialized Pose Graph are distributed as one indivisible default bundle; the manifest pins all four byte sizes/SHA256 values, `406 x 611 @ 0.05 m`, origin `[-14.692, -12.294, 0°]`, source and calibration. Its 39.8 MB `.posegraph` uses Git LFS. The incomplete v1 bundle remains available but is no longer the default | clone-reproducible v2 baseline verified by manifest/preflight; other generated maps remain ignored until deliberately curated |
| `warehouse_v2` Map Pose calibration | Three independent Isaac + ROS cold starts explicitly enabled Ideal Pose Graph localization, loaded both v2 map representations and measured `map -> base_link = [0.000, -0.000, 0.000°]` at USD `[4, 0, 0.0635, 0°]`. Each run served one `406 x 611` `/map`; `/map` and `/odom` each had one owner | `3/3` repeatability; no spread at `0.001 m / 0.001°` reported precision, while the source retains conservative `0.05 m / 5°` covariance |
| `warehouse_v2` default navigation | An Ideal Navigation cold start passed no map arguments to `run_ros.sh`; Map Server loaded `warehouse_v2.yaml` at `406 x 611`, all Nav2 managed nodes activated, and `/map` had one publisher. A goal at `[-5.267, 0.331, 0°]` was deliberately chosen in a cell that is unknown in v1 but free in v2. NavigateToPose succeeded in about 21 s with 0 recoveries; final TF position was `[-5.269, 0.222]` (about `0.109 m` position error) | default-version selection and navigation into the v2-only mapped area verified in one cold-start smoke; not a repeated complex-route batch |
| Localization + Ground Truth | The saved Pose Graph loaded in multiple Ideal sessions and one Realistic session. All 13 final runs passed the runner's post-reset map/base alignment and GT freshness gates | Ideal/Realistic runtime alignment exercised; three independent cold starts per mode with quantified pose spread remain pending |
| Immutable navigation map | In Localization/Navigation, `map_server` was active and the only `/map` publisher at `398 x 606`, while SLAM Toolbox published its changing diagnostic grid only on `/slam_toolbox/map`; Nav2 consumed the saved transient-local map | publisher isolation verified and used by the final dynamic batch |
| Nav2 activation | Activation waited for latched `/map`, fresh non-zero `/clock`, fresh `/scan` and `/odom`, and stable, freshly stamped `map -> odom`; lifecycle activation completed. The gate remains alive after `STARTUP`, eliminating the prior process-exit handler race | readiness/lifecycle smoke verified |
| Transactional Reset recovery | The gate performs cancel/pause/clear/reseed/readiness/resume and repairs partial Lifecycle Manager transitions by ordered per-node normalization. A dedicated integration test reproduces Controller/Planner active with the other four nodes inactive; the final dynamic batch then passed resets 18–20 without terminating the ROS stack | live long-batch recovery and mixed-state repair verified; destructive service-failure runtime injection remains pending |
| RViz workflows | `mapping.rviz`, `localization.rviz`, and `navigation.rviz` each loaded under Xvfb for eight seconds with no plugin/config error. The integrated Mapping launch opened managed RViz, loaded final Best-Effort sensor QoS before `/scan`, and shut down without orphan RViz processes | all three configs load-tested; extended manual GUI sessions across every display toggle are not a soak |
| Mapping Teleop | A real PTY run produced `+0.30 m/s` on W, returned to zero after the 0.18-second deadman, and Q logged a final zero. The integrated Mapping launch started the separate terminal and its wrapper stopped the identity-checked node on top-level shutdown; no Teleop/ROS/RViz process remained | input/deadman/shutdown smoke verified; not a human full-map driving acceptance |
| RViz navigation interaction | Navigation config contains the official Navigation 2 panel and GoalTool, SetInitialPose, dual costmaps, paths, footprints, and collision zones; config tests reject a project `/goal_pose` bridge and lock topic/QoS values. Live Nav2 goals completed through the same action server used by the panel | plugin/config and action path verified; final click-by-click human acceptance was bounded to config/load and navigation smoke |
| Ideal NavigateToPose | Final static batch: 4/4 at `[1, 0, 0°]`, GT errors `0.178–0.188 m`; long-range run: 1/1 at `[3, 0, 0°]`, GT path `2.807 m`, error `0.193 m`. Every run returned Nav2 status 4 and met the final-still gate | deterministic smoke/recovery evidence; not the plan's multi-goal statistical matrix |
| Realistic odometry | `/wheel/odom` and EKF `/odom` both observed at about `45 Hz`; `/odom` had one publisher (`ekf_filter_node`) | Wheel Odom + IMU + EKF ownership smoke verified |
| Realistic Navigation + Reset | Four Realistic static runs succeeded after repeated Wheel Odom/EKF resets; GT errors were `0.175–0.187 m`, odom path lengths `0.818–0.831 m`, final linear speeds about `-0.0015 m/s`, and every run met the still gate. `/odom` had exactly one publisher, `ekf_filter_node` | 4/4 deterministic smoke verified; broader drift and varied-goal statistics not run |
| TF structure | All seven configured static sensor/camera pairs and dynamic wheel links were observed; Ideal and Realistic `odom -> base_link` ownership matched the selected mode | checked runtime snapshots; always recheck after mode changes |
| Static navigation benchmark | Realistic fixed-warehouse far goal `[2, 5, 90°]`: `20/20` successes, `0` collisions, mean GT path `5.649 m`, mean/max goal error `0.171/0.191 m` | static avoidance rate `100%`, exceeding the `95%` requirement for this same-environment/same-goal benchmark |
| Dynamic navigation benchmark | Two physical one-shot obstacles clear the route after crossing/oncoming interaction. The Realistic far-goal batch achieved `19/20` successes and `0` physical collisions; successful runs had mean GT path `5.856 m` and mean/max goal error `0.126/0.178 m`. The sole failure was an early Nav2 action abort, not contact | dynamic avoidance rate `95%`, exceeding the `90%` requirement for this scenario |
| Ideal complex static route | Three forward-only runs enforced all 6 sequential poses over mean GT path `50.132 m`; mean/max final position error `0.132/0.143 m`, mean measured curved-distance fraction `30.8%`, mean stopped fraction `2.81%`, `0` recoveries and `0` commanded reverse distance | `3/3` accepted; fixed warehouse and one route, not multi-layout generalization |
| Ideal complex dynamic route | Four physical one-shot moving obstacles crossed the long route. Three runs enforced all 6 poses over mean GT path `50.108 m`; mean/max final position error `0.137/0.152 m`, mean measured curved-distance fraction `29.4%`, mean stopped fraction `4.16%`, `0` recoveries, `0` collisions and `0` commanded reverse distance | `3/3` accepted under Ideal odometry; obstacles are non-reciprocal scripted actors, so speeds are bounded to leave a valid yielding window |
| Path optimality | Inflated OccupancyGrid 8-connected A* with `0.34 m` clearance produced a `5.828 m` reference. Across all 20 successful static runs, maximum deviation was `4.31%` and P95 was `3.96%` | passes the `≤20%` requirement; reference is grid/clearance dependent |
| Experiment contracts | Static and dynamic 20-run batches wrote strict CSV/JSON reports; the aggregate checker rejects duplicate identities, insufficient samples, wrong scenario types, rate failures, and path-deviation failures | same-environment statistical benchmark accepted; multi-map/multi-layout generalization remains pending |
| Runtime hygiene | `preflight.sh` passed with Jazzy, Domain 42, Fast DDS, map hashes/LFS, assets, GPU, three RViz files and Teleop package. Safe cleanup removed stale registered metadata and current-user Fast DDS SHM only after stopping the CLI daemon and proving no Fast DDS mapping remained | normal and dry-run paths tested; the diagnostic process list may also show the IDE/Codex process because its working directory is the repository, but it is never a managed kill target |

The measured RTX and ROS rates are wall-time observations from one run. They are
useful regression baselines, not confidence intervals or performance guarantees
for another GPU, renderer, or workload.

## 2026-07-16 Realistic skid-steer control verification

The Jackal wheel collision shapes, drive limits, effective track calibration,
Wheel Odometry, command smoothing, and MPPI configuration were verified together
in a headless Realistic Navigation session. A bounded motion-assist layer was
enabled because PhysX isotropic tire contact reproduced wheel-joint target
velocities but converted too little of the left/right speed difference into body
yaw, especially while translating.

Before the correction, a commanded `0.30 m/s, 0.80 rad/s` forward arc produced
about `0.235 m/s, 0.088 rad/s`, a turn radius of approximately `2.68 m` instead
of the requested `0.375 m`. The corresponding reverse arc produced a radius of
approximately `2.08 m`. With the final configuration:

| Command | Observed steady response | Result |
| --- | --- | --- |
| rotate `0.00 m/s, +0.80 rad/s` | about `+0.819 rad/s` | yaw-rate error about 2.4% |
| forward arc `+0.30 m/s, +0.80 rad/s` | about `+0.287 m/s, +0.806 rad/s`; radius `0.356 m` | tight forward arc follows the requested curvature |
| reverse arc `-0.25 m/s, +0.80 rad/s` | about `-0.239 m/s, +0.798 rad/s`; radius `0.299 m` | reverse turning follows the requested curvature |
| reverse straight `-0.30 m/s` | about `-0.300 m/s` | reverse speed is no longer artificially suppressed |

The low-level graph was subsequently verified after replacing the
message-triggered controller execution with an on-demand `OnPhysicsStep` graph.
The physics node's measured simulation delta drives `DifferentialController.dt`,
and one articulation controller writes all four wheel targets:

| Step command | 10 Hz `/cmd_vel` | 20 Hz `/cmd_vel` | Difference |
| --- | --- | --- | --- |
| `+0.60 m/s` wheel-speed 10–90% rise | `0.181 s` | `0.164 s` | `0.017 s` |
| `+1.20 rad/s` wheel-speed 10–90% rise | `0.132 s` | `0.148 s` | `0.016 s` |

The old message-triggered topology would have scaled the configured
acceleration by approximately `command_rate / 60`; that behavior is absent in
these measurements. During the forward step, contact dynamics caused brief
front/rear physical-wheel differences, but after `0.30 s` the maximum same-side
difference was about `0.0089 rad/s` and the mean was about `0.0018 rad/s`.
Command targets themselves are written atomically in one four-joint call.

After the timing/topology change, a fresh end-to-end Realistic Navigation run
to `[1.0, 1.0, +90°]` returned Nav2 status 4. All 98 observed non-zero
`/cmd_vel` samples contained curvature, the maximum observed `|wz|` was about
`0.617 rad/s`, and no missed-controller-rate warning was emitted during the
goal.

The final MPPI default uses a 10 Hz controller, a two-second horizon
(`20 x 0.10 s`), and batch size 500. Two consecutive curved navigation goals
were then executed from one reset:

| Goal | Outcome | Command-path evidence |
| --- | --- | --- |
| `[1.0, 1.0, +90°]` | Nav2 succeeded | 53 command samples; 52 contained curvature; maximum observed `|wz|` about `0.595 rad/s` |
| `[0.2, -0.8, -90°]` | Nav2 succeeded | 70 command samples; 63 contained curvature; 52 used reverse motion; maximum observed `|wz|` reached `1.20 rad/s` |

One controller-rate warning was observed on the first goal (`8.57 Hz` achieved
against the 10 Hz target) while the CPU governor was `powersave`; the second
goal completed without a missed-rate warning. This is direct evidence for
forward curves, reverse curves, and Nav2 selecting reverse trajectories, but it
does not replace a broad obstacle/layout statistical matrix.

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

The bold row was the 2026-07-12 baseline. The 2026-07-16 Realistic curved-goal
verification above subsequently reduced the batch to 500. Both configurations
preserve a two-second MPPI horizon while reducing per-cycle work and SLAM
contention. Collision Monitor still subscribes to the original nominal 10 Hz
`/scan`; `throttle_scans` changes SLAM Toolbox processing, not the safety sensor
stream. The 20 Hz retest with reduced localization load was still worse.

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

The 2026-07-16 skid-steer increment additionally recorded:

- preflight and headless Isaac configuration/Stage validation: PASS;
- root non-Isaac/non-ROS suite: 295 passed, 6 deselected;
- focused control/Nav2 suite: 17 passed;
- focused Wheel Odometry suite: 9 passed;
- Isaac/USD Stage composition suite: 4 passed;
- rebuilt `robot_navigation` and `robot_odometry`; selected-package tests passed,
  while the workspace result ledger reported 978 tests, 0 errors, 0 failures,
  and 1 existing skip;
- `git diff --check`: PASS.

The 2026-07-17 final Ideal complex-navigation and `warehouse_v2` promotion
additionally recorded:

- preflight: PASS against the v2 manifest; full ROS build: 9 packages completed;
- root/pure suite: 319 passed, 7 runtime-marked tests deselected;
- ROS colcon suite: 328 tests, 0 errors, 0 failures, 0 skipped;
- Isaac/USD Stage composition suite: 4 passed, 49 non-Isaac tests
  deselected;
- v2 Map Pose calibration: 3/3 independent cold starts at identity, with no
  spread at `0.001 m / 0.001°` reported precision;
- default-argument v2 NavigateToPose smoke: succeeded in the v2-only mapped
  area in about 21 seconds, with 0 recoveries and about `0.109 m` final
  position error;
- automated chassis motion benchmark: `10/10` primitives accepted with zero
  collision, including circles, slaloms and rapid turn-direction reversals;
- complex static route: `3/3`, all 6 poses, 0 recoveries and 0 collisions;
- complex dynamic route: `3/3`, four physical moving obstacles, all 6 poses,
  0 recoveries and 0 collisions;
- both complex batches commanded no reverse motion; reverse recovery was
  intentionally outside this test stage;
- Ideal Localization/Navigation used a freshly stamped identity `map -> odom`
  rather than a second SLAM localization correction.

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
| Nav2 end-to-end navigation | The fixed-warehouse far-goal statistical benchmark is accepted; broader start/goal pairs, recovery fault injection, and multi-map generalization remain untested |
| Static obstacle statistics | The requested same-environment 20-run static rate passed at 100%; a larger authored multi-layout matrix is still not available |
| Dynamic obstacle statistics | The requested two-obstacle 20-run rate passed at 95%; following/blocking/size/speed families and intentional-contact observability remain separate future coverage |
| Automated experiment matrix | 20 static + 20 dynamic runs and A* aggregate acceptance are automated; multi-layout generation and unattended cold-start orchestration remain pending |
| Incremental-map benefit | bringup and a strict three-map offline comparator exist, but no real changed-region trial has produced baseline/full/incremental maps and same-definition timings to prove at least 30% improvement, changed-cell recovery, or old-region preservation; updated-map Localization/Nav2 is also pending |
| Long-duration stability | no soak-duration result is recorded |
| Custom robot migration | project/defaultPrim, static TF, Xacro, Wheel Odom, and Nav2 inputs are parameterized with a fail-fast template; no real custom USD, measured calibration, or full-chain evidence is available |

The accepted `100%` static and `95%` dynamic rates apply only to the recorded
`warehouse_v1`, `mapping_start`, goal `[2, 5, 90°]`, Realistic-mode benchmark.
They must not be generalized to unrelated maps, layouts, obstacle families, or
robots without new runs.

## Final acceptance sequence

1. Repeat cold-start Localization at least three times in Ideal and Realistic
   modes; verify single TF/odom ownership and quantify map-pose spread.
2. Repeat Ground Truth alignment across those starts, retain the no-drift check,
   and confirm that no navigation node subscribes to Ground Truth.
3. Expand Navigation beyond the successful Ideal/Realistic baselines; cover
   varied start/goal pairs, recovery fault injection, velocity smoothing,
   stop/slowdown behavior, and intentional physical collision observability.
4. Expand the accepted same-environment static/dynamic benchmark into
   multi-layout and multi-obstacle-family matrices, preserving raw reports
   under `data/`.
5. Execute the changed-region incremental workflow, run the three-map comparator,
   then validate Localization and Nav2 with the accepted updated map.
6. Supply and calibrate the real custom robot asset, then execute its Ideal,
   Realistic, Ground Truth, Reset, Localization, and Nav2 acceptance sequence.
