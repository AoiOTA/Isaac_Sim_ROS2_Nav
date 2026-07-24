# 最终验证台账

This is an evidence ledger, not a claim that every acceptance item in
`plan.md` is complete. It contains historical observations from 2026-07-10
through 2026-07-17, the historical 2026-07-22 Kujiale campaign, and the current
2026-07-23 static candidate batch, run with Isaac
Sim 6.0.1.0, ROS 2 Jazzy, Fast DDS, Nav2 1.3.12, and an RTX 4090. Generated Kit
logs and raw experiment captures remain outside normal Git history. On this
branch the calibrated `warehouse_new` bundle is the only distributed map;
`warehouse_v1` and `warehouse_v2` appear below only as historical evidence.
The remaining `warehouse_new.posegraph` is stored through Git LFS.

## 当前静态候选结果（2026-07-23；不等同完整 20+20 验收）

当前六障碍候选布局的本地报告批次为
`kujiale_long_route_static_20260723-194416`。它使用 `long_route_start_g1`、G1–G5
闭环路线、`warehouse_new`、Ideal Odometry 和 `rgbd_navigation`；静态严格成功与物理
无碰撞均为 `20/20 (100%)`，最大路径偏差为 `10.4614%`，低于 `20%` 门槛。动态 20 轮
尚未执行，布局也仍允许继续手调；因此该条目只能说明当前静态候选通过，不能作为动态或完整
20+20 验收声明。原始报告是被忽略的本地生成物。

## 历史正式验收（2026-07-22；不适用于当前重设计）

该历史酷家乐批次使用旧 `mapping_start`、G1–G8、旧障碍布局、`warehouse_new`、Ideal
Odometry 与 `rgbd_navigation`，包含静态 20 次、动态 20 次。当前分支的 S/G1、G2–G5、
中心区四个可手调静态方块、两个可手调静态长条和两组动态障碍为新布局，尚未验收；生成报告存放在被忽略的 `data/reports/`。

| 项目 | 结果 | 范围与边界 |
| --- | --- | --- |
| 静态严格成功 | `20/20 (100%)` | 通过 `19/20 (95%)` 门槛。 |
| 动态严格成功 | `18/20 (90%)` | 达到 `18/20 (90%)` 门槛。 |
| 物理无碰撞 | 静态 `20/20 (100%)`、动态 `19/20 (95%)` | 分别通过 `19/20`、`18/20` 门槛。 |
| 静态路径最优性 | 最大偏差 `19.2868%` | 通过 `<=20%` 门槛。 |
| RGB-D 证据 | 按批次记录深度点云、VoxelLayer、全局/局部 Costmap | Collision Monitor 仍只使用 `/scan`。 |

历史路线、报告一致性检查与精确结论以
[`kujiale_long_range_navigation_test_plan.md`](kujiale_long_range_navigation_test_plan.md) 的历史章节为准。动态
失败种子为 `7305`（G2 物理碰撞）和 `7312`（G1 超时）；两轮都计入旧布局分母并保留证据。

## 历史能力台账

以下条目是 2026-07-10 至 2026-07-17 的 Warehouse、旧 benchmark 和早期实现记录。
它们不描述当前默认入口、当前地图，也不能覆盖本页上方的 Kujiale 正式批次结论；需要
运行当前系统时请使用 `user_manual.md`、`interfaces.md` 和测试方案。

本文严格区分三类数据：

- **配置目标**：YAML、Launch 或 CLI 请求的值；
- **实际测量**：运行时报告、Topic/TF 快照或终端日志中的观测值；
- **测试夹具**：自动测试创建的临时地图、伪服务或进程，不等同于真实
  Warehouse 运行。

本轮性能报告来自 `.tmp_runtime/reports/*.json`。这些原始报告是本机临时
证据，不进入正常 Git 历史；关键值已回填到本文。报告测量窗口约 12 秒，
Isaac 使用 headless + realtime pacing，目标 RTF 为 `1.0`。报告元数据把 CPU
模式记为 `powersave`；实际内核为 `intel_pstate`、governor 标签为
`powersave`，同时 EPP 为 `performance`，因此不能把结果描述成纯粹的节能
或纯粹的 performance-governor 对比。

## 验收摘要

| 范围 | 当前结论 | 证据边界 |
| --- | --- | --- |
| Stage/runtime | Project Stage composed the official warehouse and Jackal overlay, found one `/PhysicsScene`, and reached the standalone ready loop without `Error`, `Traceback`, or unattached-render-product messages in the checked run | smoke verified; not a long-duration soak |
| Custom Kujiale mapping Stage | `kujiale_0026_A_to_B_door_open.usd` resolved by filename from `/home/lyb`, composed with the Jackal overlay, meter/Z metadata and one auto-created `/PhysicsScene`. The promoted `warehouse_new` map is `154 x 248 @ 0.05 m`, with 18,015 free and 3,294 occupied cells | full saved OccupancyGrid promoted for Ideal navigation |
| `warehouse_new` calibration | Three independent Isaac + ROS cold starts held `map -> base_link` at identity. Occupancy-grid scan correlation used 703–704 live endpoints per final trials; each trial placed 100% within `0.075 m` of occupied cells, with mean raster distances `0.0028 / 0.0032 / 0.0028 m` | identity Map Pose accepted at the map's 0.05 m resolution; conservative uncertainty `0.05 m / 1°` |
| `warehouse_new` Ideal navigation | With no explicit ROS map arguments, Map Server loaded `154 x 248` `warehouse_new`, all six Nav2 managed nodes became Active, and a goal at `[1.435, -0.395, 0°]` succeeded in about `4.7 s` simulated time with 0 recoveries and about `0.15 m` final position error. After the dead-end recovery change, the same goal succeeded again in about `3.43 s` simulated time with 0 recoveries | calibrated short-route smoke and post-recovery regression passed; complex multi-room route not yet run |
| `warehouse_new` long-route smoothness profile | Current configuration retains the physical `0.485 x 0.420 m` footprint, `0.005 m` padding and `0.40 m / 9.0` inflation; StopZone remains `0.535 x 0.460 m`. The current five-goal route deliberately skips the former narrow-passage stop. The tuning uses a `0.660 x 0.464 m` SlowdownZone at 90%, requires 6 scan points, disables predictive ApproachZone, and uses a 700-sample MPPI bank with fresh-noise internal retries (limit 3) before BT recovery. | The earlier `49.09 s → 27.87 s` result was obtained with the previous `0.770 x 0.470 m`/85%/ApproachZone profile and remains historical evidence only. This configuration has passed static checks but needs a new GUI navigation run before any performance claim. |
| Dead-end reverse recovery | Historical run: both navigator plugins ordered costmap clear, `BackUp`, `Spin`, then a one-second wait. The then-forward-only MPPI setting used `vx_min=0`; the current configuration permits bounded `vx_min=-0.15 m/s` terminal adjustment and the Velocity Smoother permits recovery down to `-0.25 m/s`. The recorded cul-de-sac repeat reversed `0.350 m` in about `3.22 s` and returned `SUCCEEDED` with the lidar Collision Monitor retained. | Historical root-cause evidence; current long-route acceptance is recorded above. |
| RTX LiDAR | Single-channel `RPLIDAR_S2E` clouds used frame `rtx_world`, contained 3,058–3,098 points (median 3,079), and were observed at about `13 Hz` wall time in the checked fast-simulation run | horizontal scan and processing-rate baseline verified on the custom room |
| LaserScan projection | 720 beams in `base_link`; stationary valid-return ratio median `97.57%`, and during 109 rotating frames median/minimum were `97.92% / 96.94%` | rate/frame/shape and moving projection density verified |
| Ideal drive | A `+0.2 m/s` command for 3 seconds moved Ideal odometry X by approximately `+0.517 m` | forward sign and motion path verified; full control characterization not run |
| Low-speed control | After removing extra wheel-joint friction, a `+0.02 m/s` command for 5 seconds traveled `0.09981 m` | low-speed wake/motion path verified; one manual sample only |
| Idle stability | With no effective non-zero command, the command watchdog put the articulation to sleep; stationary tests showed exact rest, zero wheel-joint velocities, and no Ground Truth drift for more than 10 seconds after the navigation goal | idle watchdog regression verified; not a long-duration soak |
| Deterministic Reset | The final 20-run static and 20-run dynamic Realistic batches completed all reset/reseed cycles. Recovery required a post-reset scan, `/simulation/localization_seeded`, strictly newer `map -> odom`, fresh stamped GT/odom, spawn alignment, stable TF, and all six Nav2 managed nodes active before goal dispatch | 40-run isolation gate verified; destructive service-failure runtime injection was not run |
| Ideal Mapping | SLAM produced `/map` at `0.05 m` resolution, observed live size `395 x 604`, with `map -> odom` and a complete base/sensor TF chain | mapping/save smoke verified; full-route map-quality acceptance not run |
| Curated map artifacts | 历史 `warehouse_v1/v2` OccupancyGrid、Manifest 与 Pose Graph 已从当前仓库移除；当前唯一分发的可执行地图 bundle 是 `warehouse_new`。下方的 v1/v2 数值仅保留审计背景。 | 历史 Warehouse 结果不能在当前 checkout 直接复现；使用当前酷家乐流程时必须以 `warehouse_new` Manifest 和 preflight 为准。 |
| `warehouse_v2` Map Pose calibration | Three independent Isaac + ROS cold starts explicitly enabled Ideal Pose Graph localization, loaded both v2 map representations and measured `map -> base_link = [0.000, -0.000, 0.000°]` at USD `[4, 0, 0.0635, 0°]`. Each run served one `406 x 611` `/map`; `/map` and `/odom` each had one owner | `3/3` repeatability; no spread at `0.001 m / 0.001°` reported precision, while the source retains conservative `0.05 m / 5°` covariance |
| `warehouse_v2` default navigation | An Ideal Navigation cold start passed no map arguments to `run_ros.sh`; Map Server loaded `warehouse_v2.yaml` at `406 x 611`, all Nav2 managed nodes activated, and `/map` had one publisher. A goal at `[-5.267, 0.331, 0°]` was deliberately chosen in a cell that is unknown in v1 but free in v2. NavigateToPose succeeded in about 21 s with 0 recoveries; final TF position was `[-5.269, 0.222]` (about `0.109 m` position error) | default-version selection and navigation into the v2-only mapped area verified in one cold-start smoke; not a repeated complex-route batch |
| Localization + Ground Truth | The saved Pose Graph loaded in multiple Ideal sessions and one Realistic session. All 13 final runs passed the runner's post-reset map/base alignment and GT freshness gates | Ideal/Realistic runtime alignment exercised; three independent cold starts per mode with quantified pose spread remain pending |
| Immutable navigation map | In Localization/Navigation, `map_server` was active and the only `/map` publisher at `398 x 606`, while SLAM Toolbox published its changing diagnostic grid only on `/slam_toolbox/map`; Nav2 consumed the saved transient-local map | publisher isolation verified and used by the final dynamic batch |
| Nav2 activation | Activation waited for latched `/map`, fresh non-zero `/clock`, fresh `/scan` and `/odom`, and stable, freshly stamped `map -> odom`; lifecycle activation completed. The gate remains alive after `STARTUP`, eliminating the prior process-exit handler race | readiness/lifecycle smoke verified |
| Transactional Reset recovery | The gate performs cancel/pause/clear/reseed/readiness/resume and repairs partial Lifecycle Manager transitions by ordered per-node normalization. A dedicated integration test reproduces Controller/Planner active with the other four nodes inactive; the final dynamic batch then passed resets 18–20 without terminating the ROS stack | live long-batch recovery and mixed-state repair verified; destructive service-failure runtime injection remains pending |
| RViz workflows | `mapping.rviz`, `localization.rviz`, and `navigation.rviz` each loaded under Xvfb for eight seconds with no plugin/config error. The integrated Mapping launch opened managed RViz, loaded final Best-Effort sensor QoS before `/scan`, and shut down without orphan RViz processes | all three configs load-tested; extended manual GUI sessions across every display toggle are not a soak |
| Mapping Teleop | A real PTY run produced `+0.30 m/s` on W, returned to zero after the 0.18-second deadman, and Q logged a final zero. The integrated Mapping launch started the separate terminal and its wrapper stopped the identity-checked node on top-level shutdown; no Teleop/ROS/RViz process remained | input/deadman/shutdown smoke verified; not a human full-map driving acceptance |
| RViz navigation interaction | Navigation config contains the project-owned Navigation 2 Safe panel, standard 2D Goal Pose, SetInitialPose, dual costmaps, paths, footprints, collision zones, and RGB-D Fusion. The custom Voxel Grid display loads and subscribes to `nav2_msgs/msg/VoxelGrid`; config tests lock its type/topic/QoS and reject a project goal bridge | plugin/config load and synthetic VoxelGrid decode verified; low-obstacle end-to-end navigation acceptance remains pending |
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

## Map Manifest 与标定

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

实际执行：

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 run robot_bringup map_manifest verify \
  --project-root "$PWD" \
  --manifest data/maps/manifests/warehouse_v1.yaml
```

The bold row was the 2026-07-12 baseline. The 2026-07-16 Realistic curved-goal
verification above subsequently reduced the batch to 500. Both configurations
preserve a two-second MPPI horizon while reducing per-cycle work and SLAM
contention. Collision Monitor still subscribes to the original nominal 10 Hz
`/scan`; `throttle_scans` changes SLAM Toolbox processing, not the safety sensor
stream. The 20 Hz retest with reduced localization load was still worse.

```text
map manifest verified: warehouse_v1 bundle=88b91be7fb0afe4364851c59dc3466f560017df5acc5405f3ab590729ded9bac
```

Manifest 中的四个不可分割工件为：

Both project costmaps do contain `nav2_costmap_2d::InflationLayer` with an
inflation radius of `0.40 m`. The padded rectangular Jackal footprint has a
circumscribed radius of approximately `0.337 m`, so the configured radius is
larger than the footprint requirement. The subsequent 1 m Ideal navigation
goal planned and completed. Keep treating this message as a version-specific
known diagnostic only while those footprint, plugin, radius, and Nav2 version
facts remain unchanged.

OccupancyGrid 声明为 `398 x 606`、`0.05 m/cell`、origin
`[-14.360, -12.247, 0.0]`。Manifest 的 calibrated bundle、
`spawn_poses.yaml` 的 `mapping_start`、地图版本和 bundle SHA256 必须完全一致，
`initial_pose_source=auto` 才允许启动。

已覆盖的失败条件包括：未 hydration 的 Git LFS pointer、文件大小或 SHA256
不匹配、bundle SHA256 不匹配、越界/符号链接路径、YAML 指向错误 PGM、
PGM 尺寸以及 resolution/origin 不一致。`save_map.sh` 的自动测试夹具还验证了
“staging -> 四工件发布 -> bundle 校验 -> Manifest 最后发布”的事务顺序，
并验证 Pose Graph 序列化失败时不留下半成品。

测试夹具创建了一个未标定的 `warehouse_v2`：`initial_pose_source=auto` 会在
节点启动前失败，而 `initial_pose_source=rviz` 允许进入人工初始位姿流程。
这证明的是契约实现，**不是**真实 Warehouse v2 已建图、已标定或已导航。

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

The 2026-07-17 robot-relative third-person camera increment additionally
recorded:

- GUI runtime created and bound
  `/World/Robots/Jackal/base_link/third_person_camera` to the active viewport;
- the current indoor-safe `3.2 m` rear / `2.2 m` high / `16 mm` wide-angle
  framing stays below a normal ceiling while retaining Jackal and the next
  doorway or bend in view;
- root/pure suite: 322 passed, 8 runtime-marked tests deselected;
- ROS colcon suite: 328 tests, 0 errors, 0 failures, 0 skipped;
- Isaac/USD Stage composition suite: 5 passed, 52 non-Isaac tests
  deselected;
- `git diff --check`: PASS.

The 2026-07-17 custom Kujiale Mapping increment additionally recorded:

- filename-only resolution selected
  `kujiale_0026_A_to_B_door_open.usd` and its uncalibrated scene-specific
  Mapping spawn profile;
- Isaac Sim 6.0.1 RTX PointCloud endpoints were verified to be absolute USD
  world coordinates, so `/lidar/points_raw` is labeled `rtx_world` and joined
  to `odom` by the inverse selected spawn pose;
- the original 32-line sensor projected only about 10% valid navigation-height
  bins because most indoor returns hit the ceiling. The horizontal
  single-channel RPLIDAR produced about 3,080 points per cloud and more than
  97% valid 2D bins without the prior processing overload;
- ROS Mapping short integration activated SLAM Toolbox, subscribed to the
  projected LaserScan and registered `Custom Described Lidar`;
- a collision-free Ideal S-curve traveled `0.532 m`; a subsequent near-full
  rotation held the moving scan above `96.94%` valid bins. Maps saved before
  and after that rotation were byte-identical, demonstrating that walls no
  longer rotate or duplicate with the chassis;
- the promoted `warehouse_new` OccupancyGrid passed three independent Ideal
  cold-start scan correlations at identity; all sampled endpoints were within
  `0.075 m` of saved obstacles and the worst mean raster distance was
  `0.0032 m`;
- the default ROS navigation entry loaded `warehouse_new` without explicit map
  arguments, activated all six Nav2 managed nodes, and completed a 1.49 m
  NavigateToPose smoke with status `SUCCEEDED`, 0 recoveries and about 0.15 m
  final position error;
- root non-Isaac/non-ROS suite: 333 passed, 11 runtime-marked tests deselected;
- ROS colcon suite: 329 tests, 0 errors, 0 failures, 0 skipped;
- Isaac/USD Stage composition suite: 8 passed, 62 non-Isaac tests deselected;
- repository index covered all 257 deliverable files, and `git diff --check`
  passed.

Re-run the same gates after any change; terminal output is authoritative if
counts change as tests are added:

```bash
./scripts/preflight.sh
./scripts/build_ros2.sh
./scripts/test.sh --with-isaac
```

若单独诊断 ROS 测试，必须先 source ROS 和工作空间；直接从未 source 的 Shell
调用 pytest 会因找不到工作空间包而产生环境性 `ModuleNotFoundError`，这不等同
于代码测试失败。

## 仍未验收或不得外推的事项

| 能力 | 当前边界 |
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
