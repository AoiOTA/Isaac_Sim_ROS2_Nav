# 最终验证台账

本文件记录仓库当前实现的可复核证据。截至 2026-07-14，本轮运行环境为
Ubuntu 24.04、Isaac Sim 6.0.1.0、ROS 2 Jazzy、Fast DDS、Nav2 1.3.12、
RTX 4090，ROS Domain 为 `42`。

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
| Map Manifest | `warehouse_v1` 与 `warehouse_v2` 四工件的逐文件/bundle 哈希均通过真实仓库校验 | v2 来自遗留本地工件恢复，来源日志缺失、运行时对齐未验证且未标定；`rviz` 路径允许人工播种，但按证据政策只用于对齐检查，不能计入正式统计 |
| 物理步与传感器时间 | OnPhysicsStep 发布的 8 秒短窗中 Clock/IMU/Joint/Odom 为 56.40 Hz、点云 9.51 Hz，所有观测 Topic 均无重复/回退/future stamp | RTF 为 0.940；`getSimulationTimeMonotonicAtTime` 仍偶发，Reset 前后、15 分钟和完整 A/B 尚未完成 |
| 底盘运动基线 | Warehouse + Ideal 改动前基线及标准 Cylinder 下 32/4、32/16 隔离 A/B 均完成 14/14；报告已绑定实际文件、solver、组合 Stage 与 Git 运行态指纹 | 32/4 已冻结并消除项目轮 collider/TGS 两类警告；低速左右转向仍不对称，SimplePlane、Realistic、接触材料与有效轮距 A/B 尚未完成 |
| Collision Monitor / `scan_fault` | 单帧/双帧丢失不停机，持续断流和 TF 缺失停车，恢复及 Reset 清故障均通过实时测试 | 是显式启用的安全测试桥，不是常驻数据通路 |
| Local Plan | `/optimal_trajectory` 为真实 MPPI 局部轨迹，10/15 Hz 均有实测 | `/transformed_global_plan` 是参考全局计划，不是 Local Plan；候选 `/trajectories` 默认不订阅 |
| MPPI | 10/15 Hz 共 12 个可行组合全部完成 3 m 目标且 missed=0；8 Hz 的 6 个组合被硬约束拒绝 | 8 Hz 没有性能数据；它们在 ROS 节点创建前即为无效配置 |
| Ceres | 8/12/16/20 线程均完成 3 m 目标且控制 missed=0；保留 12 线程默认值 | 20 线程的 RTF、Scan 和 TF 延迟更差，不能据线程数推断性能更高 |
| Camera / RViz | Camera schema v3 的严格配置、per-Prim USD authoring 与 headless 属性读回已完成；旧 schema 下 Off/Monitoring/HQ 有运行报告 | v3 尚未重跑真实静止/运动图像、RTF/GPU；`standard` 未运行，旧 HQ 约 15 Hz 不能冒充新配置验收 |
| Realistic odometry | `/wheel/odom`、IMU、EKF 唯一 `/odom` 所有权和 10 Hz 控制在实时报告中成立 | 本轮 12 秒报告结束时目标仍 active，没有记录该目标最终结果 |
| Reset | 性能矩阵逐次 Reset、Camera stamp 恢复和 `scan_fault` epoch 隔离均有实时证据 | 不能用 Trigger 成功替代后续定位/TF readiness 检查 |
| Ordered shutdown | 当前监督器对本会话认证的 launch/RViz/Teleop/helper 组执行 Lifecycle 后 INT→TERM→KILL；34 个 runtime 脚本测试、176 个 bringup 测试及 3 个顽固组用例连续 5 轮通过 | 既有真实干净退出来自前一版监督器；当前实现尚未完成真实 RViz/active-goal 连续 10 轮 N19，不能混用两代证据 |
| 自动测试 | 2026-07-14 当前工作树已执行完整门：root 508 passed/6 deselected、ROS 480/480、Isaac marker 4 passed，`test.sh --with-isaac` exit 0 | 这只证明当前代码回归门通过；第三至第十三阶段仍在实施，后续代码冻结和最终正式统计前必须再次重跑 |

## Map Manifest 与标定

### `warehouse_v1` 仓库基线

实际执行：

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 run robot_bringup map_manifest verify \
  --project-root "$PWD" \
  --manifest data/maps/manifests/warehouse_v1.yaml
```

输出：

```text
map manifest verified: warehouse_v1 bundle=88b91be7fb0afe4364851c59dc3466f560017df5acc5405f3ab590729ded9bac
```

Manifest 中的四个不可分割工件为：

| 角色 | 路径 | 字节数 | SHA256 |
| --- | --- | ---: | --- |
| Occupancy YAML | `data/maps/occupancy/warehouse_v1.yaml` | 136 | `891fdaec103073711e88cf675c9149e7684eb3d92f49913e6cf406adddf6cb6a` |
| Occupancy PGM | `data/maps/occupancy/warehouse_v1.pgm` | 241203 | `24d4a19c9a8e7f2bfdd548cbdf778e8d5c66711a71ceedb2ec5c12626cf509c5` |
| Pose Graph | `data/maps/posegraphs/warehouse_v1.posegraph` | 27834150 | `ad6a995790553d1a1a9b9ba9298d2d9d6ff7321f802f0c97e1c1e02cf19e2092` |
| Pose Graph data | `data/maps/posegraphs/warehouse_v1.data` | 204812 | `de9f482ce3d871177a2dd65d05f99585d3febb46fa397d3073f7cca9ca4f1422` |

OccupancyGrid 声明为 `398 x 606`、`0.05 m/cell`、origin
`[-14.360, -12.247, 0.0]`。Manifest 的 calibrated bundle、
`spawn_poses.yaml` 的 `mapping_start`、地图版本和 bundle SHA256 必须完全一致，
`initial_pose_source=auto` 才允许启动。

已覆盖的失败条件包括：未 hydration 的 Git LFS pointer、文件大小或 SHA256
不匹配、bundle SHA256 不匹配、越界/符号链接路径、YAML 指向错误 PGM、
PGM 尺寸以及 resolution/origin 不一致。`save_map.sh` 的自动测试夹具还验证了
“staging -> 四工件发布 -> bundle 校验 -> Manifest 最后发布”的事务顺序，
并验证 Pose Graph 序列化失败时不留下半成品。

### `warehouse_v2` 待标定候选 bundle

2026-07-14 从本机被忽略的遗留工件恢复并登记了真实四工件 bundle。实际执行同一
`map_manifest verify` 命令得到：

```text
map manifest verified: warehouse_v2 bundle=75d7df63a9feddeeb4d38053ed18d4b04603bdc6045ea191e7c689b0b98168d4
```

| 角色 | 路径 | 字节数 | SHA256 |
| --- | --- | ---: | --- |
| Occupancy YAML | `data/maps/occupancy/warehouse_v2.yaml` | 136 | `32d19866a29efd1d880ee046b934695d5999aa1ac9f0219924857b1272680067` |
| Occupancy PGM | `data/maps/occupancy/warehouse_v2.pgm` | 242815 | `2dbaadc6e4534767bcf939f9342024c84f11db1b4efd4591b86f0c6ebe7f6ce7` |
| Pose Graph | `data/maps/posegraphs/warehouse_v2.posegraph` | 37008280 | `b8799ad00fb4db997bef4c8aaf83a2efb5e47c1bee706f5aebdd9348012da264` |
| Pose Graph data | `data/maps/posegraphs/warehouse_v2.data` | 4865034 | `80fde603db5b06755d468184d3f897632c7ee165e5cd417e6ab1aa422ce3b347` |

其中只有大型 `.posegraph` 由 Git LFS 管理；preview PNG 不属于四工件 bundle。
Manifest 明确记录 `provenance: recovered_ignored_local_artifacts`、
`runtime_alignment_verified: false` 和 `calibrated: false`。因此校验通过只证明文件
没有缺失、混版或篡改，**不证明**它已和当前 Warehouse Stage、出生点或障碍结构
对齐。`initial_pose_source=auto` 会在启动前拒绝该 bundle；`rviz` 路径允许人工
播种，但按当前证据政策只用于对齐检查，不能计入正式统计。尚未完成重复冷启动
标定、长距离路线或任何 v2 正式导航统计。

## 物理步同步发布与时间警告（2026-07-14）

Isaac ROS 图已从渲染/播放 tick 改为物理步触发，realtime 主循环使用 `FramePacer`
限制墙钟节奏且每个 app frame 只推进一个固定物理步。真实 headless Warehouse、
Mapping、Ideal、Camera Off 的 profiler 短窗为 8 秒（warmup 1 秒）；原始本机报告
`/tmp/timing_physics_step_2.json` 被忽略，SHA256：
`dc01911c4844dbbffc0b239b5e92f7717c20a0d862c8df5705494d158ef7aff9`。报告元数据
记录基线提交 `c445136` 且工作区 dirty；被测物理步实现随后提交为 `2a33c57`。

| Topic/TF | Samples | Wall Hz / Stamp period | 时间完整性 |
| --- | ---: | --- | --- |
| `/clock` | 452 | `56.404 Hz` / `16.6667 ms` | duplicate/rollback/future 均 0 |
| `/imu/data` | 452 | `56.409 Hz` / `16.6667 ms` | duplicate/rollback/future 均 0 |
| `/joint_states` | 452 | `56.405 Hz` / `16.6667 ms` | duplicate/rollback/future 均 0 |
| `/odom` | 452 | `56.399 Hz` / `16.6667 ms` | duplicate/rollback/future 均 0 |
| `/lidar/points_raw` | 77 | `9.506 Hz` / `100.0000 ms` | duplicate/rollback/future 均 0 |
| `odom -> base_link` | 161 lookups | P99 lag `16.667 ms` | lookup failure/future 均 0 |

测得 RTF `0.9401`。这证明短窗内不再把同一物理状态重复发布为多条 ROS 消息，
但不等于阶段 1 完成：默认 LiDAR `accumulate_outputs=true` 的两个约 140/190 秒
日志仍分别出现 11/23 次 `getSimulationTimeMonotonicAtTime`。临时把
`accumulate_outputs=false` 后，90 秒内反而升至 2860 次，因此保留默认 true。
另一个临时候选 `resetSimulationTimeOnStop=true` 在 90 秒无 Reset 的窗口中为 0 次，
但尚未验证 Reset 前后 PointCloud/Clock epoch 与消息年龄，不能接受；候选已撤回。
仍需完成 Reset 前后、GUI/headless、realtime/unbounded、60/120 Hz 和至少 15 分钟
A/B，才能决定最终修复并宣称警告消除。

## 底盘运动基线（2026-07-14）

真实 Warehouse、Ideal Odom、Camera Off 下执行：

```bash
./scripts/run_motion_baseline.sh \
  --environment Warehouse \
  --odometry-mode ideal \
  --output data/reports/motion/baseline_warehouse_before.json
```

报告使用 `jackal_three_tier_motion_v1`，配置 SHA256 为
`be1c26709c6839e38147318f5fb41bdc234d510141505f12a7bf34b37712c7e3`；原始 JSON
按数据策略被 Git 忽略，本机文件 SHA256 为
`07f83f0232de021d1d71817f6452bf33633bbc689eb29e763248ada8ac4cccb2`。
14/14 段均为 `complete`，最终 `result=success`；这只表示采集完整，不是物理验收
PASS。

| 命令组 | 实际平均响应 | 平移/横向漂移 |
| --- | --- | --- |
| 直行 `±0.15 m/s` | `+0.140 / -0.141 m/s` | 横向漂移绝对值小于 `0.001 mm` |
| 直行 `±0.35 m/s` | `+0.304 / -0.305 m/s` | 横向漂移绝对值小于 `0.002 mm` |
| 直行 `±0.60 m/s` | `+0.472 / -0.472 m/s` | 横向漂移绝对值小于 `0.004 mm` |
| 原地转向 `±0.30 rad/s` | `+0.114 / -0.076 rad/s` | `0.0267 / 0.0367 m` |
| 原地转向 `±0.65 rad/s` | `+0.194 / -0.216 rad/s` | `0.0540 / 0.0551 m` |
| 原地转向 `±1.00 rad/s` | `+0.338 / -0.273 rad/s` | `0.0664 / 0.0831 m` |
| 圆弧 `0.40 m/s, ±0.40 rad/s` | 线速度 `0.370 / 0.375 m/s`，角速度仅 `+0.033 / -0.034 rad/s` | 轮向检查不通过 |

Clock、Odom、JointState 的 session 统计均为 `duplicate_count=0`、
`regression_count=0`；停止 onset 为 `0.0167–0.0500 s`，连续静止确认在
`0.5167–0.5500 s`。所有原地转向段及两段圆弧的四轮方向检查均出现
`mixed`/不匹配。这些结果把“转向慢、左右不对称、旋转漂移”从主观现象变成了
可复现基线，也说明底层物理问题尚未解决。SimplePlane 与 Realistic 对照、轮胎/
Collider/Joint/有效轮距修复后的同配置复跑仍是阶段 3 的阻塞验收项。

### 标准轮胎 Collider 与 TGS 32/4、32/16 隔离 A/B

官方 2022 Jackal 层的四个轮胎 Cylinder 都带已移除的
`physxCollisionCustomGeometry=true`。项目没有修改 NVIDIA 源资产，而是在
`jackal_nav.usda` 中停用四个旧 `collisions` prim，并在相同 wheel link 下定义
`collisions_v2`：半径 `0.098 m`、高度 `0.040 m`、局部 Z 轴、Rx90 后轮轴沿
wheel-link `-Y`，四轮共享 `/World/Robots/Jackal/PhysicsMaterials/wheels`。Stage
回归同时锁定前后轮距 `0.262 m`、左右轮距 `0.37559 m`、材质摩擦和旧 prim
inactive 状态。

真实 180 步 headless Warehouse + Ideal 启动中，32/4 日志
`kit_20260714_141359.log` 对 `customGeometry|more than 4 velocity` 无匹配；只把
velocity iterations 改为 16 的 `kit_20260714_144438.log` 仍无
`customGeometry`，但精确复现一次 TGS `more than 4 velocity iterations`
警告。由此把 collider 迁移与 solver 警告分开归因。

两次完整 14 段报告都由 Isaac 启动时发布并由 runner fail-closed 校验
`runtime_provenance`，包含机器人 YAML/USD、项目 Stage、Warehouse 源资产、组合
根 Layer 的 SHA256、32/4 或 32/16、运行模式，以及 Git commit/branch/dirty。
本机忽略报告及 SHA256 为：

- 32/4：`candidate_supported_cylinder_tgs4_provenance_retry_warehouse_ideal.json`，
  `ce40c395f30950f0b82055ea4698084099d2ac412c6b1152ab4e2fa57fbe4bb7`；
- 32/16：`candidate_supported_cylinder_tgs16_provenance_warehouse_ideal.json`，
  `1d81278aa21675f9ac5075364c509475e471ceb536c61f66a6268b9aabbb4e3f`。
- 恢复冻结配置并完成文档化注释后的最终调参复跑：
  `frozen_supported_cylinder_tgs4_warehouse_ideal.json`，报告 SHA256
  `b832ab30cbb21da26d420fe08a88eaf825fe2cce73709661e5d07a8cfc3cdde6`；
  14/14 成功，报告内 robot config SHA256 为
  `270ba5db751895f7faa5cb099a3385ff16d01c64525016d7a906a87f51423932`、
  overlay SHA256 为
  `bf870a06c9b974eea2607dd7f33bb536eb930f2a7795ed07f25def792b150a8a`，
  且 `customGeometry|more than 4 velocity` 在对应
  `kit_20260714_145327.log` 中均为 0 次。

| 指标 | 32/4 | 32/16 | 结论 |
| --- | ---: | ---: | --- |
| 三档前进均值 | `0.140 / 0.304 / 0.472 m/s` | `0.140 / 0.304 / 0.472 m/s` | 等价 |
| 三档倒车绝对均值 | `0.141 / 0.305 / 0.472 m/s` | `0.141 / 0.305 / 0.472 m/s` | 等价 |
| nominal 左/右旋转 | `+0.240 / -0.180 rad/s` | `+0.220 / -0.190 rad/s` | 各有一侧较高，仍不对称 |
| high 左/右旋转 | `+0.350 / -0.345 rad/s` | `+0.330 / -0.323 rad/s` | 32/4 两侧均更高 |
| 停止确认 | `0.533–0.550 s` | `0.517–0.550 s` | 等价 |
| Reset recovery 最大值 | `1.85 s` | `9.07 s` | 32/16 有明显离群点 |
| TGS 警告 | 0 | 1 | 只有 32/4 通过警告门 |

因此冻结 position/velocity iterations 为 `32/4`。这项结论只覆盖标准 collider
下的 Warehouse + Ideal solver 选择；低档左/右实际角速度仍约
`+0.035/-0.076 rad/s`，不能据此声称 skid-steer 动力学已完成。一次长时间空闲后
首段 Reset recovery 超时被保留为失败报告；全新 Isaac 进程立即复跑 14/14
成功，当前不放宽 30 秒或静止阈值，后续 soak/Reset 矩阵继续观察。

## Nav2 1.3.12 Smac inflation diagnostic

`SmacPlanner2D` 配置阶段会打印通用的
`Inflation layer either not found or inflation is not set sufficiently` ERROR。对本仓库
固定的 Nav2 1.3.12 `SmacPlanner2D` 2D radius 路径，这是上游诊断误报：该版本的
[`SmacPlanner2D` 源码](https://github.com/ros-navigation/navigation2/blob/1.3.12/nav2_smac_planner/src/smac_planner_2d.cpp#L113-L118)
以 radius mode 和 `possible_collision_cost=0.0` 调用共享 collision checker，而
[`GridCollisionChecker` 源码](https://github.com/ros-navigation/navigation2/blob/1.3.12/nav2_smac_planner/src/collision_checker.cpp#L45-L68)
会先对非正值打印 ERROR，再从 radius mode 分支返回。

本项目 Local/Global Costmap 都包含 `nav2_costmap_2d::InflationLayer`，配置半径
`0.55 m`，大于带 padding 的 Jackal footprint 约 `0.34 m` 外接半径；其后的实际
1 m Ideal 目标也完成了规划与执行。只有在 Nav2 版本、Planner、Footprint、插件和
inflation 参数均未改变时，才能把这条消息分类为已知误诊；任一条件变化都必须
重新调查，不能把所有 inflation ERROR 一概忽略。

## Collision Monitor 与 `scan_fault`

安全测试时显式启用 `scan_fault_bridge`，路径为：

```text
/scan -> scan_fault_bridge -> /scan_fault -> Collision Monitor
```

其他 SLAM/Costmap 消费者继续使用原始 `/scan`。桥接输出与 Collision Monitor
使用 Best Effort/Volatile Sensor Data QoS；`scan.topic` 的实时参数读回为
`/scan_fault`，`source_timeout` 为 `0.40 s`。

| 故障动作 | 实际观察 | 结论 |
| --- | --- | --- |
| 正常转发 | `/scan_fault` 唯一 Publisher 为 bridge，唯一外部 Consumer 为 Collision Monitor | 测试链路所有权正确 |
| `drop_next: 1` | 后续 `/cmd_vel` 仍为非零：linear `0.0132 m/s`、angular `0.0582 rad/s` | 单帧丢失不误停车 |
| `drop_next: 2` | 后续 `/cmd_vel` 仍为非零：linear `0.0736 m/s`、angular `0.0613 rad/s` | 双帧丢失不误停车 |
| `drop_all` | 超过 `0.40 s` 后出现 invalid-source 日志并执行安全停车 | 完全断流会停车 |
| `resume` | 日志恢复 `Robot to continue normal operation`，扫描继续转发 | 故障可恢复 |
| `replace_frame_id: missing_scan_frame` | TF 查询失败，随后 invalid-source 停车 | TF 缺失会停车 |
| 故障中 Reset | epoch 从 1 变为 2；mode 恢复 `normal`，remaining=0，replacement=null | Reset 不继承旧故障 |

Reset 后状态快照记录 `received=2658`、`forwarded=2273`、`dropped=385`，
`last_epoch_reason=reset_event`。Reset 后正常 `/scan_fault` 约 `9.35 Hz`。
测试结束时 Navigation 和 Localization Lifecycle manager 均成功完成有序
Shutdown；该故障目标在断流期间触发 progress/recovery 属预期安全行为，不能
把它当作一次普通导航成功样本。

## RViz Local Plan

实时图验证的 Local Plan 契约为：

| 项目 | 实际值 |
| --- | --- |
| Topic | `/optimal_trajectory` |
| 类型 | `nav_msgs/msg/Path` |
| Publisher | `controller_server`，1 个 |
| Frame | `odom` |
| 单条轨迹 | 20 poses（stable 20-step 配置） |
| 10 Hz stable 实测 | `10.001 Hz`，到达间隔 P99 `101.079 ms` |
| 15 Hz 矩阵实测范围 | `15.000–15.001 Hz`，到达间隔 P99 `67.332–67.784 ms` |

`/transformed_global_plan` 是 Controller 使用的参考全局计划，不能标为局部
轨迹。MPPI 候选轨迹 `/trajectories` 保持默认不显示，RViz 没有订阅，实测
external subscriber count 为 0，避免候选批次序列化和渲染开销。

## MPPI 10/15 Hz 性能矩阵

所有可行行使用同一 `warehouse_v1`、Ideal Localization、Camera Off、RViz Off、
`model_dt=0.10 s`、Ceres 12 线程。每行都是新 ROS 启动、Reset、3 m
NavigateToPose、约 12 秒 profiler 和有序退出。表中为实际测量值，显示时做了
有限小数舍入；原始精度保留在对应 JSON 中。

| Hz | Batch / Steps / Horizon | RTF | Controller 实际 Hz | `/cmd_vel_nav` 间隔 P99 | Scan Hz / Age P99 | Local Plan Hz | Host CPU | ROS CPU（单核口径） | Goal / 时长 | Missed |
| ---: | --- | ---: | ---: | ---: | --- | ---: | ---: | ---: | --- | ---: |
| 10 | 500 / 15 / 1.5 s | 0.9553 | 10.0008 | 102.297 ms | 9.571 / 16.667 ms | 10.0004 | 27.86% | 89.50% | succeeded / 6.261 s | 0 |
| 10 | 500 / 20 / 2.0 s | 0.9585 | 10.0006 | 100.840 ms | 9.578 / 16.667 ms | 10.0006 | 27.56% | 87.83% | succeeded / 6.451 s | 0 |
| 10 | 750 / 15 / 1.5 s | 0.9608 | 9.9982 | 101.881 ms | 9.602 / 16.667 ms | 9.9983 | 27.59% | 86.00% | succeeded / 6.260 s | 0 |
| **10** | **750 / 20 / 2.0 s** | **0.9547** | **10.0011** | **101.079 ms** | **9.542 / 31.333 ms** | **10.0011** | **28.75%** | **88.33%** | **succeeded / 6.550 s** | **0** |
| 10 | 1000 / 15 / 1.5 s | 0.9529 | 9.9999 | 100.848 ms | 9.474 / 50.000 ms | 9.9999 | 27.59% | 87.91% | succeeded / 6.151 s | 0 |
| **10** | **1000 / 20 / 2.0 s** | **0.9543** | **10.0008** | **100.843 ms** | **9.478 / 132.167 ms** | **10.0006** | **27.90%** | **91.75%** | **succeeded / 6.451 s** | **0** |
| 15 | 500 / 15 / 1.5 s | 0.9508 | 15.0004 | 67.380 ms | 9.511 / 31.000 ms | 15.0005 | 28.43% | 90.00% | succeeded / 6.711 s | 0 |
| 15 | 500 / 20 / 2.0 s | 0.9644 | 15.0008 | 67.784 ms | 9.626 / 16.667 ms | 15.0008 | 27.91% | 87.92% | succeeded / 6.980 s | 0 |
| 15 | 750 / 15 / 1.5 s | 0.9555 | 15.0003 | 67.332 ms | 9.571 / 16.667 ms | 15.0001 | 28.55% | 91.33% | succeeded / 6.511 s | 0 |
| 15 | 750 / 20 / 2.0 s | 0.9631 | 15.0002 | 67.734 ms | 9.633 / 16.667 ms | 15.0002 | 28.86% | 90.25% | succeeded / 6.851 s | 0 |
| 15 | 1000 / 15 / 1.5 s | 0.9626 | 15.0003 | 67.558 ms | 9.668 / 33.333 ms | 15.0001 | 28.71% | 90.42% | succeeded / 6.581 s | 0 |
| 15 | 1000 / 20 / 2.0 s | 0.9590 | 15.0013 | 67.748 ms | 9.634 / 16.667 ms | 15.0012 | 27.79% | 88.75% | succeeded / 6.711 s | 0 |

两个报告在结束采样时 Controller 参数服务暂时不可用：
`mppi_c10_b0500_t20.json` 和 `mppi_c15_b1000_t20.json`。这两行的配置值来自
各自被 Launch 读取的 overlay 文件和报告 label；Topic 实际频率、目标结果和
其他测量值来自运行时报告，不能把 overlay 值误称为该次参数服务读回。

### 8 Hz 硬约束拒绝

8 Hz 的所有 `batch={500,750,1000} x steps={15,20}` 组合共享：

```text
controller period = 1 / 8 Hz = 0.125 s
FollowPath.model_dt = 0.100 s
```

Nav2 1.3.12 MPPI 要求 Controller period 不大于 `model_dt`。Launch 在构造
任何 ROS Node action 前执行严格校验，因此这 6 行是 **invalid/rejected**，
不是“性能较差”。代表性真实启动约 0.7 秒内失败，明确提示至少使用 10 Hz，
且没有遗留 ROS 节点。其余五个组合由相同频率/DT 硬约束分类，不存在 RTF、
Goal 或 Missed 等测量值。

### 最终 Profile 选择

- `stable`：10 Hz、750 batch、20 steps、0.10 s DT；
- `performance`：10 Hz、1000 batch、20 steps、0.10 s DT；
- 15 Hz 保留为有效 Benchmark 点，不作为交付默认值；交付 Profile 保持
  Controller period 与 `model_dt` 相等；
- performance 行的 Scan Age P99 为 `132.167 ms`，明显高于 stable 行的
  `31.333 ms`，所以“performance”是显式实验 Profile，不表示每个指标都更优。

## Ceres 8/12/16/20 线程矩阵

以下行固定使用 MPPI stable（10 Hz、750、20、0.10 s）、Camera Off、RViz Off，
每行新启动、Reset、3 m Goal 和约 12 秒采样。

| Ceres 线程 | RTF | Controller Hz | Scan Hz / Age P99 | `map->base_link` Lag P99 | Host CPU | ROS CPU（单核口径） | Goal / 时长 | Missed |
| ---: | ---: | ---: | --- | ---: | ---: | ---: | --- | ---: |
| 8 | 0.9604 | 10.0006 | 9.625 / 33.167 ms | 0.000 ms | 27.81% | 88.25% | succeeded / 6.451 s | 0 |
| **12** | **0.9565** | **10.0002** | **9.595 / 33.333 ms** | **16.667 ms** | **28.38%** | **90.50%** | **succeeded / 6.450 s** | **0** |
| 16 | 0.9607 | 10.0014 | 9.605 / 16.667 ms | 16.667 ms | 29.13% | 93.42% | succeeded / 6.450 s | 0 |
| 20 | 0.9342 | 10.0002 | 9.106 / 32.000 ms | 382.500 ms | 27.82% | 90.00% | succeeded / 6.750 s | 0 |

8 线程报告的 Ceres 参数服务读回为 `service_unavailable`，线程数来自该次
CLI/metadata（`ceres_num_threads="8"`）；其余性能值来自实时 Topic/TF 报告。
20 线程降低 RTF 和 Scan rate，并把 TF P99 拉高到 `382.5 ms`。12 线程因此
保留为保守默认值；本轮没有声称 Ceres、SLAM 或 MPPI 获得 CUDA 加速。

## Camera、RViz 与导航组合

> 2026-07-14 起 Camera 使用 schema v3（分离光学/曝光 f-stop，并在
> CameraFront RenderProduct 局部启用 RTXAA、关闭 Motion Blur/DoF，同时关闭
> Camera Auto Exposure）。下表是该改动之前的历史运行数据，只保留为性能基线，
> 不能作为新清晰度配置的真实图像验收证据。

### Camera schema v3/API authoring 证据（2026-07-14）

- 本机 Isaac Sim 6.0.1 的 `omni.usd.schema.render_settings.rtx 1.0.2`
  `generatedSchema.usda`/`plugInfo.json` 确认：Auto Exposure API 只能应用到
  `Camera`；AA、Motion Blur、DoF API 只能应用到 `RenderProduct`；对应属性类型
  分别是 `bool`、`token`、`bool`、`bool`。
- `pytest -q isaac_sim/tests/test_camera_contracts.py`：`15 passed`；严格 schema
  漂移、光学/曝光拆分、per-Prim API/属性名和类型均有定向测试。
- Headless SimulationApp 冒烟检查成功加载 RTX schema 扩展，并在内存 Camera/
  RenderProduct Prim 上读回
  `auto_exposure=false, aa=rtxaa, motion_blur=false, dof=false`；进程正常退出且无
  Isaac/Kit 残留。

这组证据只证明配置解析和 USD authoring 闭环。它没有创建并观察真实仓库画面，
也没有重新测量 Image Hz、RTF、GPU 或静止/运动清晰度，因此 Camera 视觉验收
仍为待办。

Profile 的配置目标是：Monitoring `640x360 @ 15 Hz`，Standard
`640x480 @ 20 Hz`，High Quality `1280x720 @ 30 Hz`。下面把目标与测量值
分开列出。`Image sim Hz` 由图像 Header stamp period P50 换算；`Image wall Hz`
是 profiler 的墙钟到达率，两者不能混用。

| 组合 | 配置目标 | Image sim / wall Hz | CameraInfo wall Hz | Image Age P99 | RTF | Controller Hz / Missed | Scan Hz / Age P99 | Goal 结果 |
| --- | --- | --- | ---: | ---: | ---: | --- | --- | --- |
| Off + RViz On | 无 Publisher | — / — | — | — | 0.8895 | 10.0012 / 0 | 8.844 / 33.333 ms | 报告未记录 Goal |
| Monitoring + RViz Off | 640x360 @ 15 | 15.000 / 12.431 | 13.684 | 25.167 ms | 0.9118 | 10.0005 / 0 | 9.121 / 33.333 ms | 报告未记录 Goal |
| Monitoring + RViz On | 640x360 @ 15 | 15.000 / 8.995 | 12.830 | 16.667 ms | 0.8556 | 10.0003 / 0 | 8.460 / 33.333 ms | 1 succeeded / 7.261 s |
| High Quality + RViz On | 1280x720 @ 30 | **15.000** / 10.800 | 25.717 | 33.333 ms | 0.8564 | 9.9989 / 0 | 8.542 / 33.333 ms | 1 succeeded / 7.261 s |
| **Standard + RViz On** | **640x480 @ 20** | **未运行** | **未运行** | **未运行** | **未运行** | **未运行** | **未运行** | **未运行** |

运行时消息契约：

- Monitoring 图像为 `640x360 rgb8`，High Quality 为 `1280x720 rgb8`；
- Image 和 CameraInfo 均使用 `camera_front_optical_frame`，每个 Topic 各 1 个
  Publisher；
- RViz On 时 Image 有 1 个 RViz 外部订阅者，RViz Off 时为 0；
- Off 时 Image/CameraInfo Publisher 都是 0，即使 RViz Image Display 正在等待
  Topic 也没有崩溃；
- 每个已收到的 RGB frame 都找到 exact-stamp CameraInfo，ratio 为 `1.0`，
  但 CameraInfo 还分别多出 15、48、179 个无对应 RGB 的样本；不能将其描述成
  两个 Topic 完全一一同频；
- HQ RGB 没达到 30 Hz 配置目标，而是约 15 Hz 仿真时间频率，因此 HQ 频率目标
  尚未验收；Monitoring 在仿真时间口径接近 15 Hz，但墙钟率随 RTF/RViz 负载
  下降；
- HQ 报告中 Camera 静态 TF lookup failure 为 0；Monitoring + RViz On 报告记录
  2 次 Camera 静态 TF lookup failure，不能宣称所有组合的 TF 采样均零失败；
- 两张本机 RGB 抓帧已做图像检查，画面朝前、正立、未镜像且转向后内容改变；
  这只是抓帧检查，不等同于用户完成了 RViz 面板、交互和视觉体验的人工作业。

### GPU 指标的重要限制

Camera 和 Realistic 报告采样时，GPU 上同时存在另一个用户的 Isaac Sim 进程
（PID 825090），另有 Sunshine 进程。因此 `nvidia-smi` 的 device utilization、
总显存和功耗是共享设备快照，**不能**作为本项目 Camera On/Off 的独占增量。

| 组合 | GPU Util | 总显存 | 功耗 | 本项目 Isaac 显存 | 并发用户 Isaac 显存 |
| --- | ---: | ---: | ---: | ---: | ---: |
| Off + RViz On | 46% | 7077 MiB | 113.72 W | 2927 MiB | 2690 MiB |
| Monitoring + RViz Off | 45% | 7746 MiB | 111.66 W | 3086 MiB | 3286 MiB |
| Monitoring + RViz On | 50% | 7837 MiB | 116.05 W | 3085 MiB | 3158 MiB |
| High Quality + RViz On | 56% | 7983 MiB | 130.50 W | 3228 MiB | 3158 MiB |
| Realistic Monitoring + RViz Off | 54% | 7631 MiB | 121.48 W | 3086 MiB | 3222 MiB |

这些数字只用于说明采样当时的资源状态。若要得到可信 Camera GPU 增量，必须在
独占 GPU、停止 PID 825090 和其他图形负载后重跑同一矩阵。

## Realistic odometry

`realistic_monitoring_nav.json` 使用 Realistic、Monitoring、RViz Off、stable
Profile。12 秒窗口中的实际结果：

| 指标 | 实际值 |
| --- | --- |
| `/wheel/odom` | `45.893 Hz` wall，1 个 Publisher：`wheel_odometry` |
| `/odom` | `44.806 Hz` wall，1 个 Publisher：`ekf_filter_node` |
| `/imu/data` | `109.456 Hz` wall；Header stamp P50 `16.67 ms`（约 60 Hz 仿真时间），存在重复 stamp 到达 |
| `odom -> base_link` | EKF 所有，lookup failure=0，future stamp=0，Lag P99 `233.333 ms` |
| Controller | 目标 10 Hz，实际 `9.9982 Hz`，missed=0 |
| Scan | `9.127 Hz`，Age P99 `33.333 ms` |
| Camera | `640x360 rgb8`，Image sim/wall 约 `15.000/11.602 Hz`，exact match ratio=1.0 |
| RTF / Host CPU / ROS CPU | `0.9205` / `38.26%` / `100.00%`（ROS 为单核口径） |
| Navigation snapshot | `active_goal_count=1`，没有 succeeded/failed result |

该报告验证的是 Wheel Odom + IMU + EKF 的所有权、数据流和目标执行期间的控制
稳定性，**不能**声称这个特定目标已完成。此前 2026-07-12 的 1 m Realistic
基线有 4/4 成功记录（GT 终点误差 `0.175–0.187 m`），它是独立的历史 smoke
证据，不替代本轮报告，也不是广泛 start/goal 统计结论。

## Reset 证据

本轮可复核的 Reset 证据包括：

- 12 个可行 MPPI 组合和 4 个 Ceres 组合在各自新进程中先完成 Reset/readiness，
  再发送 3 m Goal；16/16 Goal 均 succeeded；
- 一次实时 Navigation Reset 只在 cancel/pause/clear/reseed/wait/resume 的异步
  Future 完成后返回，六个受管 Nav2 节点恢复 active；
- `scan_fault` 在 fault active 时收到 `/simulation/reset_event`，新 epoch 自动恢复
  `normal`，旧 epoch 命令被拒绝；
- Camera Reset 前后 Image stamp 从 `143.333333666` 前进到 `145.0`，没有回退，
  Image/CameraInfo Publisher 数保持各 1 个；
- 自动测试还覆盖旧 Future、超时/服务失败、重叠 Reset、旧 scan epoch 和 stamp
  rollback，旧 generation callback 不能完成新事务；
- 更早的最终实验批次累计完成 13 次 Ideal/Realistic reset/reseed，均通过 post-reset
  scan、`/simulation/localization_seeded`、新 `map->odom`、新 GT/odom 和稳定 Pose
  门控。

Reset Trigger 的 `success: true` 只表示已提交的物理重置和下游服务调用完成。
调用方仍必须等待新的 seed/manual initial pose、fresh `/odom` 和稳定且新 stamp 的
`map -> odom`，不能单凭 Trigger 返回值开始导航。

## Ordered shutdown 与 RViz 安全退出

当前 `scripts/run_ros.sh` 是顶层监督器。`ros2 launch`、集成 RViz、Mapping
Teleop 和 ordered-lifecycle helper 分别运行在独立进程组；监督器只接受 PID/
PGID、leader start ticks、项目根和 `ISAAC_NAV_SESSION_ID` 均匹配的本会话组。
收到 INT/TERM/HUP 后，它先运行私有 rclpy Context/Executor 的
`robot_bringup.ordered_shutdown`，随后对全部已认证组执行有界
`SIGINT -> SIGTERM -> SIGKILL`，没有全局 `pkill`。

顺序契约为：

- Navigation：先 `/lifecycle_manager_navigation/manage_nodes` Shutdown，再
  `/lifecycle_manager_localization/manage_nodes` Shutdown；
- Localization：只关闭 Localization manager；
- Mapping/Incremental Mapping：依次向 SLAM Toolbox 发送 deactivate、cleanup、
  shutdown transition；
- 一次关闭的默认**总 deadline** 是 20 秒；Lifecycle helper 最多使用其中 10 秒，
  并为进程组升级和元数据清理保留最后 1 秒，不是每个握手各有 20 秒；
- 第二次停止请求直接对仍认证的组请求 TERM，第三次才请求 KILL；元数据删除函数
  在组仍可见时会拒绝删除，RViz 子进程也不会继承实例锁 FD；
- RViz 使用仓库内只观察 Lifecycle 的安全 Nav2 Panel，避免与 Activation Gate
  争夺所有权，并协作清理 callback/Future/Context。

历史真实运行证据（早于本次 session-auth/顽固后代加固）包括：

- 12 个 MPPI 与 4 个 Ceres 新进程运行均以该监督器有序退出，没有 Controller
  `-6`、未等待协程或残留 ROS/RViz 进程；
- Collision 故障测试的两步 manager Shutdown 实测分别约 `3.239 s` 和
  `3.351 s`，全栈干净退出；
- 新监督器下至少一次 Mapping 实跑完成 SLAM 生命周期有序退出；
- High Quality + RViz 首轮退出暴露了 `robot_description` 的 rclpy Context
  竞态，修复后用同一组合复跑明确 clean；
- Realistic 首轮退出暴露了 `wheel_odometry` 未处理的
  `ExternalShutdownException`，修复后复跑明确 clean；
- 安全 RViz Panel 在监督器改造前另做了 Navigation、Mapping、Localization、
  Camera View 各 5 次（20/20）启动/退出测试；该数据验证 Panel 本身，但不能
  冒充当前监督器的跨模式矩阵。

当前实现的自动证据为：

- `robot_bringup` runtime 脚本定向测试 `34 passed`，包级测试 `176 passed`；
- 独立 PGID RViz 可被 `clean_runtime` 认证、RViz leader 退出但顽固后代仍持锁、
  顽固 Teleop wrapper 三个对抗用例连续执行 5 轮全部通过；
- 测试覆盖 leader 已是 PGID 时仍重执行环境/默认信号、关闭 RViz 子进程锁 FD、
  PGID+start-ticks 防复用、逐阶段元数据/成员复核，以及只有组消失才删元数据；
- 五个改动脚本 `bash -n`、`git diff --check` 均通过，测试结束无假进程残留。

当前代码还没有真实执行 N19 要求的 RViz 连续 10 轮，也没有覆盖当前实现下的
每个模式、Camera profile、active goal 和 Render Product 泄漏矩阵。因此结论只能
是“进程组安全契约和对抗自动测试通过”；不能把旧真实 smoke、旧 Panel 20/20 或
新假进程测试拼成当前真实退出验收。

## 发布前审查回归

2026-07-13 当轮发布前的三路只读审查没有发现 P0，但发现了地图发布、进程退出和
输入边界阻塞项；当时代码冻结后已完成针对性与全量重跑。其后新增的 Camera v3、
物理步发布、底盘诊断、Warehouse V2 和 session-auth 退出加固当前只有定向证据，
仍须在最终冻结后重跑完整门。历史新增回归包括：

- `save_map.sh` no-clobber hard-link 发布、并发同名工件保留、Manifest 链接后的
  信号窗口回滚、父目录 symlink 和纯点版本拒绝；
- Manifest 正数 resolution、父级 symlink/路径身份、USD/Map pose、yaw、两项
  标准差逐值绑定，以及 Localization Reset profile 不可跨 bundle 切换；
- `run_ros.sh` helper 不继承忽略信号、第二次停止强制 TERM、顽固 launch 组的
  有界升级，以及 Ordered Shutdown 的 20 秒全局 deadline；
- Safe RViz Panel 的空 GoalStatus 和非法/超范围循环文本；
- `/scan_fault` 每条命令强制 epoch，Profiler 识别 supervisor operation，并在
  PID+start-time 成员集合变化时拒绝输出误导 CPU 差分；
- `robot_rviz_plugins` 在 `BUILD_TESTING=OFF` 且不查找测试依赖时独立配置、编译和
  安装成功。

## 自动测试证据与当前重跑边界

2026-07-14 当前工作树已在标准 Cylinder、TGS 32/4、运行时 provenance 和本文档
更新后执行完整三条门。构建、预检和 `test.sh --with-isaac` 均 exit 0；预检仍如实
报告 265 个 Fast DDS SHM 遗留工件和 CPU governor 非 performance 的环境警告，
资产、地图、GPU 与其余门通过。下表是本批冻结点的真实证据；第三至第十三阶段
继续修改后，最终正式统计前仍必须重新执行，不能把本批通过外推为整个 Goal 完成。

| Gate | 最近证据 |
| --- | --- |
| `./scripts/preflight.sh` | 2026-07-14 PASS；资产/地图/GPU 通过，另有 265 个 Fast DDS SHM 遗留工件和 CPU governor 非 performance 的非阻塞环境警告 |
| `./scripts/build_ros2.sh` | 2026-07-14：11 packages build completed，exit 0 |
| `./scripts/test.sh --with-isaac` 的 pure/root suite | 2026-07-14：514 collected，508 passed，6 deselected |
| ROS `colcon test` | 2026-07-14：480 tests，0 errors，0 failures，0 skipped |
| Isaac/USD marker suite | 2026-07-14：4 passed，77 deselected |
| RViz config/load smoke | 结构测试包含在 454 个 root tests；安全 Panel 20/20 历史循环及本轮 Off/Monitoring/HQ 实跑组合见上文 |
| `robot_rviz_plugins` production-only build | 独立 `-DBUILD_TESTING=OFF` configure/build/install PASS |
| 2026-07-14 Camera 定向测试 | Camera contracts 15 passed；Camera/config 定向集合 27 passed；`isaac_sim/tests` 69 passed、3 skipped |
| 2026-07-14 底盘诊断定向测试 | 48 passed；真实 Warehouse + Ideal 14/14 段完整采集 |
| 2026-07-14 退出加固定向测试 | Runtime 脚本 34 passed；`robot_bringup` 176 passed；3 个顽固进程组用例连续 5 轮通过 |
| Map bundle 校验 | `warehouse_v1`、`warehouse_v2` 的真实 Manifest verify 均 PASS |
| Repository index set comparison | 当前拟提交的 282 个路径对 282 个索引路径，集合差分无输出 |
| Markdown 相对链接 | README 与 10 个 `docs/*.md` 当前缺失链接为 0 |
| `git diff --check` | 当前 PASS；代码冻结和提交前再执行一次最终审计 |

推荐最终命令：

```bash
./scripts/build_ros2.sh
./scripts/preflight.sh
./scripts/test.sh --with-isaac
```

若单独诊断 ROS 测试，必须先 source ROS 和工作空间；直接从未 source 的 Shell
调用 pytest 会因找不到工作空间包而产生环境性 `ModuleNotFoundError`，这不等同
于代码测试失败。

## 仍未验收或不得外推的事项

| 能力 | 当前边界 |
| --- | --- |
| 真实 `warehouse_v2` | 四工件与 Manifest 已登记并通过完整性校验；来源日志、Stage 对齐、出生点标定、长距离路线和正式导航均未完成 |
| 底盘物理 A/B | 改动前与标准 Cylinder 下 Warehouse + Ideal 32/4、32/16 已完成；SimplePlane、Realistic、接触材料、Joint Axis、有效轮距和低速转向不对称尚未完成 |
| Camera Standard | schema v3 的 `640x480 @ 20 Hz` 组合未运行；不存在可外推的新配置性能数据 |
| Camera High Quality 频率 | 约 15 Hz 是 schema v3 前历史基线；v3 配置目标 30 Hz 尚未实测，不能写成已达到或已失败 |
| Camera 人工 GUI 验收 | schema v3 前抓帧做过方向检查；v3 静止/运动清晰度与用户 click-by-click、面板布局、视觉体验均未验收 |
| GPU Camera 增量 | 并发另一个用户 Isaac Sim，当前共享 GPU 快照不能做独占增量结论 |
| Realistic 本轮目标结果 | 报告结束时目标 active；不得写成该报告 succeeded |
| Localization 冷启动统计 | 未完成 Ideal/Realistic 各 3 次独立冷启动及 Map Pose 离散度量化 |
| 一般导航成功率 | MPPI/Ceres 是固定 3 m 调参矩阵，不是多 start/goal 的总体成功率统计 |
| Static/Dynamic 大矩阵 | 已有固定场景 smoke，但没有完成计划要求的广泛布局、速度、尺寸和种子统计 |
| Incremental-map 收益 | 尚无真实 changed-region 三地图对照及至少 30% 收益证据 |
| Long-duration soak | 未记录长时间稳定性结果 |
| Custom robot | 只有参数化迁移契约和模板，没有真实自定义 USD、标定和全链路验收 |
| 完整 Shutdown 矩阵 | 当前 session-auth 监督器尚未完成 N19 真实 RViz 连续 10 轮；所有模式/Profile/active-goal 各连续 5 次矩阵也未归档 |

以上固定目标运行可以作为回归、Reset 隔离、控制稳定性和资源竞争基线，但不能
把 16/16 调参 Goal、历史 4/4 Realistic 或 4/4 动态 smoke 外推成一般性的
100% 导航成功率。
