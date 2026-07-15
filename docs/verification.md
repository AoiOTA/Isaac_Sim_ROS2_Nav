# 最终验证台账

本文件记录仓库当前实现的可复核证据。截至 2026-07-15，本轮运行环境为
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
| 物理步与传感器时间 | OnPhysicsStep 的 8 秒短窗保持 56.40 Hz 状态 Topic 与 9.51 Hz 点云；`resetSimulationTimeOnStop=false` 的 30 分钟基线为 `93 / 0 / 93` 次时间样本警告，采用供应商默认 `true` 后的两个 Camera 短窗与 15 分钟 headless soak 均为 `0 / 0 / 0` | 15 分钟报告中 `/clock` 和点云均无重复/回退，RTF 为 0.947；真正 Timeline Stop→Play 以及 GUI/headless × realtime/unbounded × 60/120 Hz 完整矩阵仍未完成 |
| 底盘运动基线 | Warehouse + Ideal 改动前基线及标准 Cylinder 下 32/4、32/16 隔离 A/B 均完成 14/14；clean commit `0500f9e` 上的 SimplePlane/Warehouse × 六 Profile × 三重复也完成 36/36 运行、216/216 段 | 32/4 已冻结并消除项目轮 collider/TGS 两类警告；历史矩阵证明证据链和 Reset 合同可靠，且描述性中心漂移 `0.297–0.350 m`、旧整段角速度误差 `60.1%–69.0%` 暴露严重欠转，但这些 schema-1 报告没有当前稳态窗口，计划 8.7 verdict 为 N/A；Realistic 和候选有效轮距 A/B 仍未完成 |
| 阶段 3 物理诊断工具 | 可逆 contact Profile、三个版本化 ground-topology Profile、两个版本化 Reset strategy、独立 SimplePlane、单轮方向诊断、有效轮距离线拟合及 runtime provenance schema 6 均已实现；当前 motion/analysis/physical/summary 为 schema 3/5/3/6，Manifest v2 固定 47 列，最终分组身份为 environment/topology/reset/contact 四元组 | clean `65ae923` 的固定输入 Reset A/B 正式批次完成 20/20 run、20 included、0 excluded、两组各 10 repeat。两组都只因 `rotation_center_drift_asymmetry_ratio <= 0.20` 的 every-repeat 门失败：A 失败 5/10，B 失败 6/10；其余 17 类检查均为 10/10。B 没有改善，不能晋级，项目继续保留 A 默认。批次 `result=success` 只表示证据链闭合，不是物理 PASS |
| Collision Monitor / `scan_fault` | 单帧/双帧丢失不停机，持续断流和 TF 缺失停车，恢复及 Reset 清故障均通过实时测试 | 是显式启用的安全测试桥，不是常驻数据通路 |
| Local Plan | `/optimal_trajectory` 为真实 MPPI 局部轨迹，10/15 Hz 均有实测 | `/transformed_global_plan` 是参考全局计划，不是 Local Plan；候选 `/trajectories` 默认不订阅 |
| MPPI | 10/15 Hz 共 12 个可行组合全部完成 3 m 目标且 missed=0；8 Hz 的 6 个组合被硬约束拒绝 | 8 Hz 没有性能数据；它们在 ROS 节点创建前即为无效配置 |
| Ceres | 8/12/16/20 线程均完成 3 m 目标且控制 missed=0；保留 12 线程默认值 | 20 线程的 RTF、Scan 和 TF 延迟更差，不能据线程数推断性能更高 |
| Camera / RViz | Camera schema v3 的严格配置、per-Prim USD authoring 与 headless 属性读回已完成；旧 schema 下 Off/Monitoring/HQ 有运行报告 | v3 尚未重跑真实静止/运动图像、RTF/GPU；`standard` 未运行，旧 HQ 约 15 Hz 不能冒充新配置验收 |
| Realistic odometry | `/wheel/odom`、IMU、EKF 唯一 `/odom` 所有权和 10 Hz 控制在历史实时报告中成立；新契约用真实 rclpy 覆盖 schema-v6 匹配、SHA 错配和 Isaac 服务超时，并在 Wheel Odom 退出时关闭整套 Realistic launch | 新 schema-v6 握手尚未完成一轮冻结候选的真实 Isaac+ROS Realistic 导航；本轮 12 秒历史报告结束时目标仍 active，没有记录该目标最终结果 |
| Reset | A=`pose_restore_v1` 与 B=`separate_recontact_0p20m_1step_v1` 已由 schema-1 定义、contact probe、runtime provenance v6、motion report、47 列 Manifest 和四元分组精确绑定；根 Pose 最后通过 USD backend 写入并 flush/同步 physics articulation | clean `65ae923` 正式 A/B 证明两策略都能 10/10 完成，但物理门分别有 5/10 与 6/10 repeat 因旋转中心漂移不对称失败；B 不得替换默认 A，且 Trigger 成功仍不能替代后续 readiness |
| Ordered shutdown | 当前监督器对本会话认证的 launch/RViz/Teleop/helper 组执行 Lifecycle 后 INT→TERM→KILL；34 个 runtime 脚本测试、176 个 bringup 测试及 3 个顽固组用例连续 5 轮通过 | 既有真实干净退出来自前一版监督器；当前实现尚未完成真实 RViz/active-goal 连续 10 轮 N19，不能混用两代证据 |
| 自动测试 | clean `65ae923` 的当前代码执行 `pytest -q` 为 `1270 passed / 34 skipped`；`./scripts/test.sh --with-isaac` 为 root `1268 passed / 1 skipped / 35 deselected`、ROS `1035 tests / 0 errors / 0 failures / 1 skipped`、Isaac marker `32 passed / 1 skipped / 283 deselected` | shellcheck 与 Isaac/USD binding 的两个环境性 skip 已明确记录；单元/契约门不等于物理 PASS、Realistic 导航或 Warehouse V2 正式统计，第三至第十三阶段仍须继续 |

## 当前 Reset A/B v3 正式证据（2026-07-15）

### 合同与受控输入

提交 `ae06c1f` 引入两个不可原地改义的 schema-1 Reset strategy：

- A：`pose_restore_v1`，恢复完整 pose/DOF 状态；
- B：`separate_recontact_0p20m_1step_v1`，先抬升 `0.20 m`，推进一步并通过
  四轮到目标地面的接触探针验证分离，再恢复完整状态并推进一步重接触。

提交 `d228f25` 让根 Pose 在完整关节状态和零 base velocity 之后最后写入，并通过
USD backend、`flush_changes()` 与 physics articulation kinematic update 保证 teleport
跨物理步持久。提交 `572595f` 让 `--reset-strategy all` 在奇数 repeat 使用 A→B、
偶数 repeat 使用 B→A；提交 `01aeb2c` 稳定进程身份采样；提交 `65ae923` 在 headless
模式关闭 Isaac Sim 默认 viewport 更新，保留独立 RTX RenderProduct，避免连续冷启动的
descriptor 压力。当前证据链为 runtime provenance 6、motion report 3、Manifest
contract 2/47 列、analysis 5、physical acceptance 3、policy
`skid_steer_plan_8_7_v3`、batch summary 6；group key 为
`environment::topology::reset-v1-ID::contact-profile`。

固定输入为 `SimplePlane`、`simple_plane_only1_v1`、
`threshold_corr_0p00025_offset_0p04`、`jackal_etw_0p989_v1`、Ideal、Mapping、
60 Hz、TGS `32/4`、Camera Off。正式命令为：

```bash
CANDIDATE="$(realpath -e -- \
  "$PWD/isaac_sim/configs/robots/experimental/jackal_etw_0p989_v1.yaml")"
./scripts/run_contact_ab_matrix.sh \
  --environment SimplePlane \
  --ground-topology simple_plane_only1_v1 \
  --contact-profile threshold_corr_0p00025_offset_0p04 \
  --reset-strategy all \
  --repeats 10 \
  --robot-config "$CANDIDATE" \
  --output-dir data/reports/contact_ab/reset_strategy_ab_65ae923
```

### 资源故障、修复与烟测

clean `01aeb2c` 的第一次正式尝试在 r01-A 完成后，于 r01-B 日志出现
`[Fatal] [omni.rtx] Out of resource descriptors!`；readiness 超时并失败关闭，没有生成
analysis/summary。该失败工件保留在本机
`data/reports/contact_ab/reset_strategy_ab_01aeb2c/`，不得计入统计。

修复后的 clean `65ae923` 先完成 A/B 各一次烟测：2/2 run、2 included、0 excluded、
2 group、47 列 Manifest，summary `result=success`；因每组仅一次，physical 为 N/A。
工件 SHA256：

| 工件 | SHA256 |
| --- | --- |
| smoke `manifest.tsv` | `0e8fe1c57447688cf1c7acd1e1e6452565f6239b08b262be9009b829767b24f1` |
| smoke `analysis.json` | `f17006d62d22723e76d7067a853f607d89e1498a1e09e7e5b453ebaa1141cf2a` |
| smoke `batch_summary.json` | `8a6fe8c3e1e9dcc1b919462535546a68b19f6879367171588c797b6c5d902c2b` |

为避免把 contact-only SimplePlane 当作传感器场景，另在 Warehouse、headless、Camera Off
上运行 8 秒 profiler：日志出现 `Viewport updates disabled`，点云为 `77` 样本、
`9.6028 Hz`、`38,264,444` bytes，Camera Image/Info publisher 均为 0。报告
`data/reports/runtime/viewport_warehouse_65ae923.json` 的 SHA256 为
`80a317aa0d9dbbd781727df8e69e51cd0d66302ec4c695ae7226582f09ab1689`。
SimplePlane 的唯一碰撞平面是 `purpose="guide"`，RTX 默认不可见；其 raw pointcloud
publisher 存在但零命中时不发送空消息，这是夹具边界，不是 viewport 修复回归。

### 20-run 正式结论

正式批次从 `2026-07-15T03:49:45Z` 运行到 `04:34:44Z`。结果为 20/20 run success、
20 included、0 excluded、2 groups、每策略 10 个唯一 repeat；20 行均锁定 motion/runtime/
reset schema `3/6/1`，matrix complete。工件 SHA256：

| 工件 | SHA256 |
| --- | --- |
| `manifest.tsv` | `da7faad19cbcb247287ae241626bd392562e6737dca6d0fa91f82eac236121e5` |
| `analysis.json` | `9e595e51dd625718d45bf1a4d8611248eb8ee13daf720ab0d8764ed814b1deba` |
| `batch_summary.json` | `1b7778236594fab4867f5d04956dcb1f7955144f4e868f3835c63911930f72d7` |

`batch_summary.result="success"` 只证明采集、身份、矩阵和聚合闭合；physical
`all_applicable_groups_passed=false`，两组均 FAIL。唯一失败检查是
`rotation_center_drift_asymmetry_ratio <= 0.20`：

| 策略 | Repeat 物理 PASS | 不对称失败 | 失败观测范围 | 其余 17 类检查 |
| --- | ---: | ---: | --- | --- |
| A `pose_restore_v1` | 5/10 | 5/10 | `0.225606–0.226383` | 全部 10/10 |
| B `separate_recontact_0p20m_1step_v1` | 4/10 | 6/10 | `0.225606–0.226383` | 全部 10/10 |

通过分支的不对称值为 `0.053836–0.054785`；左右中心漂移本身均小于 `0.10 m`，
稳态 yaw-rate、六段轮向和全部停止窗也逐重复通过。B 比 A 多一次失败，且 policy
明确不提供排名；因此严谨结论是“两者都未通过，B 没有解决离散旋转不对称”，项目保留
A 作为默认，不晋级 B，也不能进入最终物理参数冻结。

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

测得 RTF `0.9401`。这证明该短窗内不再把同一物理状态重复发布为多条 ROS
消息，但不能单独证明 RTX helper 的时间警告已经消除。

### RTX helper 时间策略与 A/B

默认 `accumulate_outputs=true` 继续保留。此前临时改成 false 后，约 90 秒内
`getSimulationTimeMonotonicAtTime` 从短窗偶发升至 2860 次，因此该参数不是修复
方向。当前安装的 Isaac Sim 6.0.1 `isaacsim.ros2.nodes` 版本为 `1.18.13`；其
changelog 记录 `1.5.3` 已把 Camera、CameraInfo 和 RTX LiDAR helper 的
`resetSimulationTimeOnStop` 默认值改成 true，安装版 OGN 定义也仍以 true 为
默认。本项目此前显式覆盖为 false，现已让三个 RTX publisher 与供应商默认以及
直接 `/clock`/Odom 时间节点使用同一 epoch 策略。

本机 Kit 日志中，三项计数依次表示
`getSimulationTimeMonotonicAtTime`、`getSimulationTimeAtTime` 和
`No adjacent samples found`：

| `resetSimulationTimeOnStop` / 运行条件 | 可复核活跃窗口 | 三类计数 | 证据 |
| --- | ---: | ---: | --- |
| `false`；GUI、realtime、60 Hz、Camera Monitoring | 至少 `30m14.728s` | `93 / 0 / 93` | `kit_20260714_115620.log`，SHA256 `c0ffac901c13ccf163ad1db4660382b15d84e394296617cd3677c66320744947` |
| `true`；headless、realtime、60 Hz、Camera Off，含事务 Reset | `2m17.761s` | `0 / 0 / 0` | `kit_20260714_155906.log`，SHA256 `95c695b5e9283ef1584df37be8e69df35d2f9d61122af008ca45b451de5b91d1` |
| `true`；headless、realtime、60 Hz、Camera Monitoring | `2m08.637s` | `0 / 0 / 0` | `kit_20260714_160142.log`，SHA256 `28c42283c5250e2556a9e178e2dea64247964e8fc45eddc38d5c2056be07cca3` |
| `true`；headless、realtime、60 Hz、Camera Off | profiler `15m00.000s`；Kit `16m51.385s` | `0 / 0 / 0` | `kit_20260714_160458.log`，SHA256 `3e7f4a4169552b48ba6112f9f9140b887d8dfa5060e3eb3d03f8e1e1423f4558` |

false 基线从第二次 `onResume` 到最后一条活动传感器记录，日志没有正常退出记录，
所以 `30m14.728s` 只是可证明的运行下界。三组短窗和长跑都来自 dirty 工作树；
开关标签由当时工作树与命令记录重建，而非日志内的属性快照。因此它们是可信的
工程 A/B，但不会冒充 clean-commit 的完整受控矩阵。

false 路径中的 monotonic 查询与 `No adjacent samples found` 93 次一一成对，而
所有 true 运行均为 0。结合供应商默认值和 helper 行为，当前根因判断是：false
要求跨 Timeline Stop 使用单调时间，RTX writer 对有限 Fabric 时间样本历史的查询
会偶发越界。SimulationManager 对应实现是发行版二进制，仓库无法做完整源码证明，
所以该根因按“日志与供应商接口共同支持的推断”记录，而不是伪装成源码定论。

### 15 分钟时间完整性与 Reset

长跑报告 `/tmp/timing_vendor_true_15min_off.json` 被 Git 忽略，SHA256 为
`160dcc5d32b09348725cbea98ffa5f8d281192d0aea73bcf93790501e7cd9aa2`；报告绑定
分支 `codex/navigation-quality-fidelity`、提交 `343b0227dc56753b47394bdb75b8fcf638673865`
并明确标记 dirty。900.000185 秒窗口测得 RTF `0.947079`：

| Topic | Samples / Wall Hz | Duplicate / Regression / future | 补充 |
| --- | --- | --- | --- |
| `/clock` | `51130 / 56.8103 Hz` | `0 / 0 / 0` | 51130 个 RTF 样本全部唯一，epoch rollback 0 |
| `/lidar/points_raw` | `8523 / 9.4695 Hz` | `0 / 0 / 0` | P99 age `16.667 ms`，max `100 ms` |
| `/odom` | `51129 / 56.8092 Hz` | `0 / 0 / 1` | 单个未解释 lead 为 `233.333 ms`，聚合报告不能定位其阶段或原因 |
| `/imu/data` | `51130 / 56.8103 Hz` | `0 / 0 / 20822` | “future”按最新收到的 `/clock` 回调计算，最大 lead 仅 `28.483 us` |
| `/joint_states` | `51130 / 56.8103 Hz` | `0 / 0 / 4354` | 同一回调顺序指标，最大 lead `28.483 us` |

这里如实保留 profiler 的 future 计数：IMU/Joint 的最大 lead 只有 28.483 微秒，
反映同一物理步内 ROS 回调到达顺序，并不是 stamp 回退；Odom 有一个 233.333 ms
离群点，仍需在最终时间矩阵中复查，不能被 `0 / 0 / 0` Kit 警告计数掩盖。

`data/reports/motion/reset_time_true_warehouse_ideal.json` 另记录 Warehouse、Ideal、
60 Hz 下 14/14 段成功运行，SHA256
`fbee8661880ee704f2f654cc8a57d494d7ab0c47502faeeafa67d219ba8de49d`。
14 次 `/simulation/reset` 后均收到新 `/clock`、`/joint_states` 和 `/odom`；三者
样本数分别为 3299、3303、3289，duplicate/regression 全为 0。项目 Reset 是
pause → 单步 → play，不执行 Timeline Stop，因此 true 不会在该事务中创建新 epoch。

据此已采用 `resetSimulationTimeOnStop=true` 作为项目修复：它与当前供应商默认值
一致，并在已测 headless/Camera Off、headless/Camera Monitoring 和 15 分钟窗口内
消除了三类 Kit 时间样本警告。第三阶段仍未退出：真正 Timeline Stop→Play 的点云
header/消息年龄/旧 DDS 样本隔离，以及 LiDAR 开关、`update_fabric`、GUI/headless、
realtime/unbounded、60/120 Hz 的完整矩阵尚未完成，不能外推成所有组合均已验收。

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
`0.5167–0.5500 s`。所有原地转向段及两段圆弧的四轮方向检查均出现历史报告所称的
`mixed`/不匹配：`mixed` 只表示轮速样本最小值和最大值越过 deadband 两侧，即存在
双向瞬态，不等于主导平均轮速必然反向。当前严格 A/B 分析器只允许纯旋转段保留
这种瞬态，并独立核对平均符号、min/max、逐轮标志和总标志。这些结果把“转向慢、
左右不对称、旋转漂移”从主观现象变成了可复现基线，也说明底层物理问题尚未解决。
SimplePlane 正式重复、Realistic 对照及轮胎/Collider/Joint/有效轮距修复后的同配置
复跑仍是阶段 3 的阻塞验收项。

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

最初两次完整 14 段 A/B 报告已包含机器人 YAML/USD、项目 Stage、Warehouse 源
资产、组合根 Layer、运行模式和 Git 指纹；但发布前只复制 robot YAML 中的 solver
值，且 runner 只核对 odometry，没有执行初始化后 Articulation wrapper 的第二次
USD 读取，也允许调用方把 Warehouse 误标为 SimplePlane。其运动数据和日志仍可
用于 solver A/B，因为 Stage 属性、唯一改变量与警告均有独立证据；但这些 schema
v1 本机忽略报告只保留为历史 A/B 证据，不满足随后引入的 schema v2 runner 契约。
报告及 SHA256 为：

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

审查后已把门禁强化为：项目配置必须提供 path-safe `environment.id`；solver 属性
缺失时必须以 USD `int` 创建；Isaac 初始化 Articulation wrapper 后通过其 USD 后端
读回 solver，并要求它与有效 Stage 的两个属性完全一致，之后才创建 ROS 节点和
发布只读参数。该非弃用 API 证明的是组合 USD 输入一致，不是 PhysX 引擎内部状态
直接读回；32/4 与 32/16 的行为差异仍以隔离 A/B、运动数据和警告日志为证。
runner 同时强制 `--environment`、`--odometry-mode` 与运行态匹配，并在创建运动
`/cmd_vel` publisher 前完成所有校验。

schema v1 加固后同一工作树的真实 Warehouse + Ideal 复跑
`effective_readback_warehouse_ideal.json` 为 14/14 成功，报告 SHA256
`1aa6cb187a977473b7078fe4e18ced0a7109a1d2428169a25dab58dd954d273c`；其中
`environment.id=Warehouse`、solver `32/4`、
`stage_runtime_readback_verified=true`（schema v1 旧字段），Clock/Odom/JointState 分别记录
5512/5504/5521 个样本且 duplicate/regression 均为 0。对应
`kit_20260714_152558.log` 的 `customGeometry` 与 TGS `more than 4 velocity`
均为 0；`getSimulationTimeMonotonicAtTime` 仍有 24 次，所以仿真时间问题没有被
掩盖或宣称解决。

schema v1 的负向实跑把同一 Isaac 故意标为 `SimplePlane`：
`provenance_environment_mismatch_verified.json` 以 exit 1 失败，报告 SHA256
`2929f500de389ec309df81907d646e24ca69ca202d80e9b22d3b11594a998c36`，并保留
已验证的实际 `Warehouse` 与 Stage/Articulation USD 32/4。失败前后 `/cmd_vel` publisher
count 都只有 Isaac 自己的 Reset 安全 publisher（1），报告中的 runner
`cmd_vel_subscription_count=0`，证明错误标签没有获得运动命令所有权。

随后 schema v2 已用中性文件名重新实跑。正确 Warehouse + Ideal 报告
`usd_provenance_v2_warehouse_ideal.json` 为 14/14 成功，SHA256 为
`7411bafef1ed644f74a0418ee3390e7c2f39f65675ac36c9a89bb6dd1dbc8560`；报告记录
`schema_version=2`、`environment.id=Warehouse`、solver `32/4` 和
`stage_articulation_usd_readback_verified=true`。Clock/Odom/JointState 分别为
3536/3527/3540 个样本，duplicate/regression 均为 0；robot config、overlay 和
组合根 Layer SHA256 分别为
`270ba5db751895f7faa5cb099a3385ff16d01c64525016d7a906a87f51423932`、
`bf870a06c9b974eea2607dd7f33bb536eb930f2a7795ed07f25def792b150a8a` 和
`1a4f79b5d893f60a165b4d711435162966d0dba3d2334e05b6be593be23dc0fb`。

同一进程的 schema v2 负向报告
`usd_provenance_v2_environment_mismatch.json` 以 exit 1 失败，SHA256 为
`4d26487606577919879a107122bb6710370596198a68c0a3d86e22f1b7c68a49`；失败报告
保留 verified Warehouse、USD 32/4 和新字段，runner 的
`cmd_vel_subscription_count=0`，退出后系统仍只有 Isaac Reset 安全 publisher。
对应 `kit_20260714_154544.log` 在干净停止前的 `customGeometry` 与 TGS
`more than 4 velocity` 均为 0；`getSimulationTimeMonotonicAtTime` 仍有 23 次，
所以 schema 加固没有掩盖尚未解决的仿真时间问题。

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

### Robot kinematics 单一来源零行为迁移（2026-07-15）

提交 `dd58c63e305eee2257f3afeb4d53ccbd9e2ff3ec` 完成运动学契约迁移。稳定
`jackal.yaml` 从 schema v1 升为 v2，新增
`kinematics_profile_id=jackal_legacy_geometric_v1`、
`lifecycle=stable_baseline` 和独立 `effective_track_width`。稳定值仍为：

| 字段 | 迁移前实际输入 | 迁移后稳定输入 | 行为结论 |
| --- | ---: | ---: | --- |
| wheel radius | `0.098 m` | `0.098 m` | 不变 |
| geometric track width | `0.37559 m` | `0.37559 m` | USD/URDF 物理轮心不变 |
| controller wheel distance | `0.37559 m` | `effective_track_width=0.37559 m` | Control Graph 数值不变 |
| Wheel Odom track width | `0.37559 m` 的独立 YAML/Python 副本 | 同一 robot YAML 的 `effective_track_width=0.37559 m` | 积分数值不变，副本删除 |

迁移后的失败关闭链如下：

- robot loader 拒绝重复/未知/缺失键、旧 schema、非法 profile/lifecycle、非唯一或
  非法 joint、非正/非有限几何与质量，以及不满足
  `nominal_total_mass = base_mass + 4 × wheel_mass` 的文件；
- Isaac DifferentialController 使用有效轮距，四个 Controller jointNames 从 robot
  YAML 派生，并与 project config 中的三组镜像 exact 比较；
- Robot Description 只用几何轮距布置轮心，从同一 YAML 传入实际 joint 名；非空
  visual link prefix 也不会改写 JointState 名。当前惯量张量仍是 Jackal 基线，因此
  不兼容的轮径、轮宽或质量会被明确拒绝，而不是冒充通用自定义机器人模型；
- `dd58c63` 提交当时 runtime provenance 升为 v4，发布 robot path/原始字节 SHA256、
  profile、lifecycle、轮径、轮宽、几何/有效轮距及 controller 合同标志；当时的
  motion runner 只产 v4，历史 v3 只能离线单批复核；后续 live 链先统一升级到 v5，
  当前已进一步升级到 v6；
- `dd58c63` 提交当时 Realistic Wheel Odom 在创建 `/wheel/odom`、JointState/Reset
  subscription、Reset service 或 timer 前读取并逐项比对 Isaac v4 参数；超时或错配时非零退出，
  OnProcessExit 随后关闭当前 Realistic launch；
- Contact A/B 的 v4 有效轮距计算使用每份 provenance 与所选 robot 一致的真实轮径，
  不再硬编码 `0.098 m`；历史 v3 仍锁定 canonical 半径，v3/v4 不得混批，输出记录
  本批真实 schema。

实际验证命令和结果：

```text
./scripts/build_ros2.sh
  11 packages finished

./scripts/preflight.sh
  preflight: PASS

./scripts/test.sh --with-isaac
  root: 907 passed, 1 skipped, 23 deselected
  ROS: 725 tests, 0 errors, 0 failures, 1 skipped
  Isaac/USD: 21 passed, 232 deselected
```

`shellcheck` 未安装是唯一 skip。preflight 另报告 300 个 Fast DDS SHM 工件和
20 个非 performance governor 核心；这些是后续性能实验必须记录/清理的主机状态，
不是本轮契约测试失败。此迁移没有把拟合的约 `1.012 m` 写入 stable，也没有产生新
物理通过结论；多速度候选、SimplePlane/Warehouse、Realistic 和接触拓扑 A/B 仍是
第三阶段下一步。

### 版本化有效轮距实验候选（2026-07-15）

提交 `ab909b4` 在不修改 stable 的前提下新增两个完整、自包含、可按原始字节哈希的
schema-v2 候选：

| Profile | `effective_track_width` | 来源 | 文件 SHA256 |
| --- | ---: | --- | --- |
| `jackal_etw_0p989_v1` | `0.989 m` | clean `0500f9e` 接触矩阵 Warehouse `threshold_corr_0p025_offset_0p0004` mean `0.989336019045897 m` 的三位舍入 | `2b8860141964be5a7e40cbee830d2e24c27acc7db82599b47e41186519781e3e` |
| `jackal_etw_1p012_v1` | `1.012 m` | 同一矩阵 `threshold_corr_0p025_offset_0p04` 的 SimplePlane/Warehouse mean `1.0140121078344426/1.0100307720240023 m` 等权均值 `1.0120214399292224 m` 的三位舍入；亦接近历史 yaw-response OLS `1.0124019295 m` | `5fc56206f06797dc206a68e0966094fb1652b2d8aeba76f3e7dc4c698f3d1a7c` |

两文件均标记 `experimental_candidate`；相对 stable 的 YAML 语义差异集合严格等于
`{kinematics_profile_id, lifecycle, effective_track_width}`。轮径、轮宽、质量、
几何轮距、joint、solver `32/4`、控制限幅和外参完全不变；Isaac Control Graph 与
Wheel Odom parser 分别读到 `0.989/1.012 m`，而两份候选渲染出的 URDF 与 stable
字节等价。v1 文件不得原地修改，后续值必须版本递增。

clean `ab909b4` 的 11 包 build、preflight 和全门结果为上表最新计数。这里的 PASS
只证明候选身份、解析链、固定惯量边界和零几何差异；尚未执行候选的两环境、多速度、
接触拓扑或 Realistic 物理 A/B，不能据此把任一值升级为 stable。

提交 `4b55f90` 已给正式接触矩阵增加 `--robot-config FILE`：显式输入必须是仓库内
canonical absolute regular file、被 Git 跟踪、在 `HEAD` 中是普通 blob，且工作树原始
字节与该 blob 一致；默认和显式 robot 都进入批次哈希锁。Isaac 子进程清除继承的
`ISAAC_NAV__*` 后只恢复 project/contact/robot 三项，40 列 manifest 与 schema-v2
batch summary 都记录 robot 选择来源、路径、SHA256、profile/lifecycle 和完整运动学。
该提交的 contact matrix 静态/契约定向验证为 `31 passed, 1 skipped`，唯一 skip 是
本机未安装 `shellcheck`。这只验收参数、信任边界、override、manifest/summary 和失败
关闭合同；尚未用任一候选执行真实物理矩阵，不能把静态 PASS 当作候选标定证据。

### 可逆接触 Profile 与 SimplePlane 隔离基线（2026-07-14）

提交 `dcb5ca2` 增加了三个严格模式：不 author 新值的 `legacy_baseline`、只 author
scene threshold 的 `threshold_only`，以及同时 author wheel/ground 材质与
physics-purpose binding 的 `explicit_material`。四个 threshold 文件覆盖
correlation distance `0.00025/0.025 m` 与 offset threshold `0.0004/0.04 m` 的
2×2 矩阵；显式材质固定 wheel `0.2/0.2/0.0`、ground `0.5/0.5/0.0`，两个
combine mode 均为 `average`。

所有修改只进入 SessionLayer 下的独立匿名 sublayer；应用后重新解析和读回有效 Stage，失败会移除
临时层。定向 USD 验证确认：

- Warehouse 精确解析 4 个启用 wheel collider、32 个 ground collider，配置的
  `floor_decal` semantic class 与必需 `/Root/GroundPlane/CollisionPlane` 均有匹配；
- SimplePlane 使用独立源环境和组合根，只解析唯一
  `/Root/GroundPlane/CollisionPlane`，`SceneComposer` 拒绝额外环境 sublayer；
- default legacy 不 author scene/material 值；当前 Warehouse 有效读回保持 wheel
  material `static=0.2/dynamic=0.2`、ground 无 physics material、scene threshold
  `0.00025/0.0004 m`；
- explicit profile 的 direct/effective binding、三项材质数值和两个 combine mode
  都与 YAML 一致；读回失败和非法 semantic class 均为 fail-closed；
- 默认 Warehouse 与 SimplePlane 的 `run_isaac.sh --validate-only` 都通过；纯 USD
  定向集合 `test_contact_setup.py + test_stage_composition.py -m isaac` 为
  `21 passed`。

这些证据证明配置选择、非持久化 authoring、精确 collider 集合和读回门可用，
不证明某个 Profile 改善了运动。当前已完成每个 Profile 一次的 SimplePlane 严格
批处理烟测；尚未完成每个 threshold 点/显式材质至少三次独立进程的
SimplePlane/Warehouse 同输入 motion A/B，也没有 Realistic 对照。
提交 `84c397c` 已将 motion runtime provenance 升级为 schema v3：Isaac 对 profile
路径/hash、匿名 overlay、scene、collider contract、binding、材质与 mode flags 做
Stage 读回校验，再以 canonical JSON + SHA256 只读参数发布；runner 在创建运动命令
publisher 前重新验 hash、解码并执行同等级结构校验。定向 provenance/report/package
集合为 69 passed。随后在 clean commit
`088bfda7812eae9c73da62ca3f6e6eae010e40c6` 上运行 Warehouse + Ideal +
`legacy_baseline`，14/14 motion 段全部 `complete`：

```text
report: /tmp/motion_v3_clean_warehouse_legacy_01.json
report SHA256: 8532187c6e4a3dc4667412320c05241863a3280959d88ab55eee3cf0ac9a0f23
Kit log: kit_20260714_175441.log
Kit log SHA256: c0cefb0b603a9fd6bd5916a26c8592b46bc8ee4271b5297d16ad2618a2502ca6
```

报告为 `schema_version=3`、`git.dirty=false`、solver `32/4`；contact 读回为
`legacy_baseline`、4 个 wheel collider、32 个 ground collider、匿名 overlay SHA256
`1a6561d7db0df2086521ad4744c1acdaa70d5c4442e3786f8f40e9e70228d95e`，且
`stage_usd_readback_verified=true`。Clock/Odom/JointState 分别记录
`3529/3518/3533` 个样本，三者 duplicate/regression 均为 0。Kit 日志 28 条 warning、
`[Error]` 为 0，`getSimulationTimeMonotonicAtTime`、TGS、obsolete
`customGeometry` 和 contact filter/API mismatch 均为 0。该实跑证明 v3 发布/传输/
解码链可用；它仍只是一个 legacy 条件，不能代替后续接触矩阵。

### Ground collider topology 与 provenance v5（2026-07-15）

提交 `6897712` 新增三个 schema-v1 topology profile：

| Profile | 环境 | Source / Target / Disabled | 操作 |
| --- | --- | ---: | --- |
| `simple_plane_only1_v1` | SimplePlane | `1 / 1 / 0` | 保留源 collider |
| `warehouse_combined32_v1` | Warehouse | `32 / 32 / 0` | 保留源 collider |
| `warehouse_plane_only1_v1` | Warehouse | `32 / 1 / 31` | 只禁用非目标 collider |

Profile 锁定源资产原始字节 SHA256、三组精确路径数量和 canonical 路径集合 hash。
实现只在 SessionLayer 下的 topology 专用匿名 sublayer 中 author plane-only 所需的 31 条
`physics:collisionEnabled=false`；combined 与 SimplePlane 不 author collider 属性。
应用后重新读取 Stage，检查 overlay 没有额外 Prim、metadata、attribute 或其他 opinion，
并验证 source 是 target 与 disabled 的无交集精确并集。切换 profile 会先移除旧层，
源 Warehouse/SimplePlane USD 不会被修改。该提交定向验证为纯 Python `240 passed`、
Isaac/USD `30 passed`，其中包括匿名层内容 hash 稳定、31 个精确 disable opinion、
combined→plane-only→combined 可逆，以及额外 Prim/metadata fail-closed。

随后 `3d1f891`、`899690b`、`835afb2`、`dc31164` 把启动链升级为 runtime provenance
schema v5。生产者发布独立的 `ground_topology.json/.sha256`，并在发布前无条件从
当前 Stage 重新 capture topology/contact；传入的 SceneComposer 快照只能与 fresh
readback canonical 全量一致。离线 validator 验证 exact key、有限值、三个集合的
数量/hash/partition/operation 以及 contact ground target。motion runner 完整校验
canonical topology/contact JSON 与 SHA；Realistic Wheel Odom 只握手 schema、robot
config path/SHA 和七个 kinematics/controller 字段，两者的 live 握手都只接受整数
schema 5。A/B analyzer 的 v5 分组键为
`environment::topology::contact_profile`：同环境锁除 runtime-derived RootLayer SHA
外的完整 environment、wheel collider 和 source collider；另按 contact profile ID
跨环境/topology 锁 profile path/SHA/id/mode/flags；同 environment+topology 锁 profile/operation/overlay SHA/
target/disabled/readback 与 contact ground selector/list；同一 environment+contact
跨 topology 锁 profile、scene、wheel bindings、wheel/ground material 和 readback，
允许 treatment 所需的 ground bindings/path 改变；三元组内锁 runtime 初始化后的
RootLayer SHA，以及除进程地址型 `overlay_identifier` 外的完整 contact。topology/contact
的直接 opinion 位于 SessionLayer 下的两个独立匿名 sublayer，但初始化
可形成 treatment-dependent 的 RootLayer 派生 opinion，`a85828f` 真实机制烟测因此证明
不能把该摘要误锁为跨 treatment 常量。历史 v3/v4
保留已发布的旧锁层并可分别离线
审计，但与 v5 互相禁止混批。

定向证据为 runtime provenance `20 passed / 2 skipped`（无 PXR 的普通 Python 环境）、
真实 Isaac/PXR producer 集成用例 `2 passed / 20 deselected`、report/motion/package
`167 passed`、contact analyzer `59 passed`。提交 `6897712` 的 `30 passed` 只证明
ground-topology overlay 核心及其可逆性；新增的两个 producer fresh-readback 用例由
该 PXR marker 定向运行独立复验。这里的 PASS 证明配置、overlay、读回、传输和统计隔离
合同成立。后续 clean `a85828f` 机制烟测已产生 Warehouse combined32 与 plane-only1
各六个 contact profile 的 12 份真实 schema-v5 成功报告；但每条件只有一次，且当次
聚合因错误地把 runtime-derived RootLayer SHA 当作跨 treatment 环境常量而失败关闭，
不能声称 plane-only 改善转向或有效轮距。完整失败证据和修复见下文。

PXR 定向命令为：

```bash
source scripts/lib/common.sh
ISAAC_SITE_PACKAGES="$("$ISAAC_PYTHON" -c \
  'import site; print(site.getsitepackages()[0])')"
PYTHONPATH="$ISAAC_SITE_PACKAGES:${PYTHONPATH:-}" \
  python3 -m pytest -q isaac_sim/tests/test_runtime_provenance.py -m isaac
```

提交 `a1056c3` 将严格批处理入口扩展为 topology 维度。默认 `baseline` 保留
SimplePlane/only1 + Warehouse/combined32 的 `36 runs / 12 groups`；显式 `all` 只展开
三个合法 environment/topology pair，为 `54 runs / 18 groups`，不会生成非法笛卡尔积。
脚本把 topology profile 加入 clean-HEAD blob/hash 锁，子进程只恢复
project/topology/contact/robot 四个受信覆盖，readiness 与单轮报告都验证 schema v5、
canonical topology JSON/SHA、源资产/三组 collider/readback 和 contact target。当时输出升级为
43 列 manifest、analysis schema 2 与 batch-summary schema 3；这也是下文 `d5840ed`
历史工件的版本。它随后被 v2 的 44 列、`3/4/2/5` 链接替；当前生产合同则是本文
开头所列 Manifest v2/47 列与 `3/5/3/6`，三代不得混写。该阶段定向脚本验证为
`42 passed / 1 skipped`，唯一 skip 是本机未安装 `shellcheck`。后续加固还让 profile/
topology 摘要直接序列化第一次 HEAD/hash 锁定值，并逐行核对冻结 Manifest 的 topology
ID/path/SHA，阻断瞬态二次读盘造成的证据分叉；这仍只是批处理合同验证，
54 次正式、每组三重复的真实 Isaac topology 矩阵尚未执行。

### Warehouse 32-vs-1 首次机制烟测失败与 RootLayer 锁修复（`a85828f`，2026-07-15）

在 clean commit `a85828f` 上执行：

```bash
./scripts/run_contact_ab_matrix.sh \
  --environment Warehouse \
  --ground-topology all \
  --repeats 1 \
  --output-dir data/reports/contact_ab/ground_topology_smoke_a85828f
```

脚本实际启动 12 个独立 Isaac 进程：`warehouse_combined32_v1` 与
`warehouse_plane_only1_v1` 各配六个 contact profile。12 份报告均为
`result=success`、provenance schema 5、`verified=true`、`git.dirty=false`；72 个运动段
全部为 `result=complete`。43 列冻结 Manifest 共 12 行且全部 `success`，SHA256 为
`1fc6d8a56979334471274a052865c510d15543c4828a800ba6a37fea8f886345`。逐行回算报告、
Isaac log、runner log 与九类锁定输入共 144 次 SHA256，结果为 `0 missing / 0 mismatch`。
combined32 六份报告均读回 `32 source / 32 target / 0 disabled`，plane-only1 六份均为
`32 / 1 / 31`，USD readback 全部通过。

旧 analyzer 在 12 个进程完成后按设计失败关闭，错误为
`environment contract mismatch for Warehouse: report 007 differs from 001`，因此失败目录
没有生成 `analysis.json` 或 `batch_summary.json`，也没有事后回填或改写。递归比较证明
两个报告唯一不同的环境字段是 `composed_root_layer_sha256`。实际观察到三种值：

- combined32 六个 profile：`1a4f79b5d893f60a165b4d711435162966d0dba3d2334e05b6be593be23dc0fb`；
- plane-only1 的其余四个 profile：`b96c794f348ef63109b1aeba190471317e3435112532e81142f667fe2d7531a9`；
- plane-only1 的 correlation `0.025` profile：`b1b475d80ad7345452f90857a65206b331343fe6294e6bdb06eb38c5a5d40ac8`。

根因是该摘要在 PhysicsSetup、传感器、Reset 与 Articulation 初始化之后捕获；即使
topology/contact 的直接 opinion 只位于 SessionLayer 下两个匿名 sublayer，后续 runtime
仍可能在 RootLayer 形成 treatment-dependent 派生 opinion。修复把 RootLayer SHA 从
跨 treatment 的 environment lock 移到最终 environment/topology/contact 组内；global、
environment、跨环境 profile、environment+topology、environment+contact 的显式输入锁全部保留。定向
analyzer 回归为 `59 passed`；对原始 12 份报告只读离线重聚合得到
`analysis_valid=true`、`12 input / 12 included / 0 excluded / 12 groups`，所选 12 组子矩阵完整。

这批每格只有一次，不能估计方差、排名 profile 或作 topology 因果结论。描述性均值中，
combined32→plane-only1 的左/右旋转中心漂移为 `0.21350→0.22344 m` /
`0.33907→0.33888 m`，yaw gain 为 `0.31165→0.30508` / `0.26082→0.26113`，
有效轮距为 `0.93421→0.95026 m` / `1.06220→1.05974 m`；方向混合，且两种 topology
的 yaw gain 都远低于 `1`。因此该批只证明真实机制、证据链与修复方向；正式结论必须
来自 clean 冻结提交上的每组三重复 54-run/18-group 全 topology 矩阵。

### RootLayer 修复后 `d5840ed` 成功机制烟测（2026-07-15）

在 clean commit `d5840ed60334a487a88538dd2cbd7b4cc7b53482`、分支
`codex/navigation-quality-fidelity` 上重新执行同一 Warehouse 32-vs-1 机制批次：

```bash
./scripts/run_contact_ab_matrix.sh \
  --environment Warehouse \
  --ground-topology all \
  --repeats 1 \
  --output-dir data/reports/contact_ab/ground_topology_smoke_d5840ed
```

本次冻结根证据为：

| 文件 | SHA256 |
| --- | --- |
| `manifest.tsv` | `a31acc23fa5c4c029ba09b938d417a1ce08d053d4928e271631dcc816634bdcf` |
| `analysis.json` | `3c39c1593dcdd5cf19d3678410bd734531ebf662ab5600bb588ec2d0301babc9` |
| `batch_summary.json` | `92e136a8cb968859b72976977a4c129d19813ba897c67807661d695543011852` |

43 列 Manifest 为只读、12 行全部 `success`；12/12 报告为成功，72/72 六段运动均为
`complete`。逐行审计的 144 个路径/hash 对由每轮 report、Isaac log、runner log 与
九类冻结协议输入组成，即 `12 × (3 + 9)`；结果为 `0 missing / 0 mismatch`。12 份
报告全部绑定上述 commit、同一分支且 `git.dirty=false`，topology/contact USD readback
和 contact ground target 交叉门均为真。聚合纳入 12、排除 0、形成 12 个完整选中组；
summary 的预期值为 12 run、12 group、2 topology、6 contact profile，实际
successful/manifest/included run 与 group 均为 12，top-level 身份列表也分别包含
2 个 topology 和 6 个 contact profile。

runtime 初始化后的 `environment.composed_root_layer_sha256` 不再被误锁成跨 treatment
常量，实际 `6/4/2` 分布完整保留：

| 组别 | 报告数 | Root SHA256 |
| --- | ---: | --- |
| Warehouse combined32，六个 contact profile | 6 | `1a4f79b5d893f60a165b4d711435162966d0dba3d2334e05b6be593be23dc0fb` |
| Warehouse plane-only1：legacy、explicit-material、两个 correlation `0.00025` profile | 4 | `b96c794f348ef63109b1aeba190471317e3435112532e81142f667fe2d7531a9` |
| Warehouse plane-only1：两个 correlation `0.025` profile | 2 | `b1b475d80ad7345452f90857a65206b331343fe6294e6bdb06eb38c5a5d40ac8` |

日志边界必须按来源和级别分别阅读：12/12 Isaac 日志出现 ready，72/72 Reset 出现
complete，72/72 runner segment 出现 complete，12/12 报告出现写出记录；runner 日志的
warning/error/failure 计数为 0。Kit 格式的 `[Error]` 为 0，Fatal、Exception、Traceback
也为 0，且 `PxShape getMaterial internal face`、`getSimulationTimeMonotonicAtTime`、
`customGeometry`、ContactReportAPI/filter mismatch、TGS、CUDA、segfault/core、assertion/
runtime traceback 等高风险模式均为 0。与此同时，12 份 Isaac 日志各有一条非致命
absl `E0000 descriptor_database.cc:633 File already exists in database:
grpc/health/v1/health.proto`，并各有一条对应的重复注册 `W0000`；另有 214 条 Kit
`[Warning]`、12 条 absl bootstrap `WARNING`。所以这批只能准确写成“Kit `[Error]=0`、
高风险模式为 0，但存在 12 条非致命 absl `E0000`”，不得写成“零错误”。日志还说明
USD diagnostics 被静音，底层 USD warning 细节不可见；CPU governor 标记为 powersave，
因此该机制批次也不是性能基准。

该目录是在下面的新机器硬门合同出现之前生成的历史工件：`analysis.json` 为 schema 2，
`batch_summary.json` 为 schema 3，并不含 `physical_acceptance`。它证明 RootLayer 修复后
的 repeat=1 证据链和严格聚合可以闭合，但仍不能估计重复方差、排名 profile、给出
topology 因果结论或冒充正式每组三重复的 54-run/18-group 全 topology 矩阵。其输入
同时是 Warehouse、repeat=1、motion report schema 1；按当前适用性合同，12 个 group
都应是 `applicable=false`、`passed=null` 的 N/A，不能改写成 `0/12 fail`。

### 历史计划 8.7 v2 机器硬门合同（已由当前 v3 接替）

下述内容记录 v2 合同及其当时证据，版本号和哈希均按历史保留；当前生产合同与正式
结论见本文件开头的“当前 Reset A/B v3 正式证据”。当时 motion runner 生成顶层
schema 3 报告，内嵌
`configuration.schema_version=1` 保持不变。每段 Odom 与 JointState 都保存 schema-1
`final_half_of_command_interval` closed window；边界固定为
`command_start + (command_end-command_start)//2` 到 command end。两窗至少有两个严格
递增样本，首尾延迟和最大相邻间隔必须不超过 canonical profile 的
`max_sample_age_sec=0.5`，且 reported max gap 必须落在由首尾时间和样本数决定的
`average gap..span` 可行区间。Odom 窗保存实际角速度分布；JointState 窗保存 deadband、
三类方向样本计数及四轮分布，且稳态分布必须能作为整段命令分布的真实样本子集成立。
停止确认要求 Odom 与 JointState 两路都持续、新鲜且同时静止，报告分别保存两路样本、
跨度和 freshness。Reset generation、三路接收水位、Reset 后首个时间戳、命令与停止
样本记账、非法消息计数、安全零速退出、起终姿态、路程、纵横向位移和模 `2π` 航向
也都必须能相互重算。整段轮向只作描述，稳态 mixed/stationary/opposite 是 valid evidence，
但会让方向物理门失败；不可能的计数、矩、极值、时间或几何关系则属于 invalid evidence。

v5 离线输出为 analysis schema 4，内嵌 `physical_acceptance` schema 2、policy
`skid_steer_plan_8_7_v2`、`evaluation_basis="every_repeat"`；44 列 manifest 增加并冻结
`report_schema_version=3`。其 header 必须精确且有序，每行交叉绑定规范化、JSON 类型
严格的 motion 配置、report/selection、robot config/asset/kinematics/controller/solver、
Mapping/Ideal/60 Hz、Git、environment/topology/contact，以及三类互异 canonical regular
证据的 path/hash；报告完成时刻必须落在 manifest 同一 UTC 秒区间。batch summary 为
schema 5，并冻结同一 robot asset、solver、simulation 和运动配置。物理 gate 只适用于同时满足以下
六项的 group：runtime provenance schema 5、环境 `SimplePlane`、topology
`simple_plane_only1_v1`、Ideal odometry、实际至少 3 个唯一 repeat、所有 motion report
schema 3。schema 1/2 仍可作历史输入，但固定以 `motion_report_schema_not_3` 记 N/A。
对适用 group，每个 repeat 都计算全部检查，任一 repeat 任一检查失败即判定整个 group
失败；报告不排名，也不自动选择 profile。

| 检查 | 固定机器边界 |
| --- | ---: |
| 前进横向漂移绝对值 | `≤ 0.05 m` |
| 后退横向漂移绝对值 | `≤ 0.08 m` |
| 左/右旋转中心漂移 | 每侧 `≤ 0.10 m` |
| 左右旋转漂移不对称 | `abs(left-right)/max(left,right) ≤ 0.20`；两者均为零时为零 |
| 左/右稳态角速度误差 | 后半命令窗口实际 `angular_z_radps.mean` 相对目标角速度的 `abs(actual-commanded)/abs(commanded) ≤ 0.10` |
| 六段确认静止窗 | 配置的稳定时长 `≥ 0.5 s`，每段确认窗 `≥` 该配置值 |
| 停止配置上限 | 线速 `≤ 0.02 m/s`、角速 `≤ 0.05 rad/s`、轮速 `≤ 0.20 rad/s` |
| 四轮速度方向 | 六段均符合严格报告方向合同 |

analysis 顶层 `applicable_groups/not_applicable_groups` 精确划分所有 group，
`passing_groups/failed_groups` 精确划分适用 group；没有适用 group 时
`all_applicable_groups_passed=null`，否则它才表示全部适用 group 是否通过。
`batch_summary.json` schema 5 的顶层 `result="success"` 只表达证据采集、身份、矩阵与
聚合闭合；物理结果另复制 `all_applicable_groups_passed` 和上述四个列表，两者不得
合并成一个“成功”结论。

最终记账仍固定 18 个检查 ID；方向叶保存六段 × 四轮共 24 个稳态观察。公共 validator
除重算每个 leaf、group 和顶层 verdict 外，还会重新读取 `selection.included.path` 的
原始 report，验证 canonical regular path、raw/canonical SHA、全局 Git/robot/asset/
kinematics/solver/simulation/motion identity，并重新核对 Reset/时间/停止/样本记账、
姿态/航向几何和分布子集后再重算整个 physical acceptance。协调把 schema 3 改成
schema 2/N/A、把真实 wheel FAIL 改成 PASS、篡改 source/hash/全局身份、删除段/轮或
改写 selection/matrix 都会失败关闭。

当前定向结果为 contact analyzer `217 passed`、motion baseline `92 passed`、matrix
script `45 passed / 1 skipped`，合并为 `354 passed / 1 skipped`；唯一 skip 是缺少
`shellcheck`。当时的 v2 合同已在 clean
`0484b72741bfb5cd8a0866ca2631b15b2d2909fc` 完成三条正式门：build 11 packages、
preflight PASS，`./scripts/test.sh --with-isaac` exit 0；root
`1206 passed / 1 skipped / 34 deselected`，ROS 11 packages / 1006 tests / 0 errors /
0 failures / 1 skipped，Isaac `32 passed / 250 deselected`。preflight 如实记录 422 个
Fast DDS SHM 工件和 20 个非 performance governor 的非阻塞警告。clean `190f357` 的历史
schema-2 smoke、clean `22a7746` 的 schema-3 smoke 和 clean `8973728` 的正式
SimplePlane 三重复失败结论见下文；完整 54-run/18-group topology 矩阵仍未执行。

### 历史 v1/schema-2 SimplePlane 真实机制烟测（2026-07-15）

在 clean `190f357e0785383cdd273d3a53728f683dc14dbd` 上执行：

```bash
./scripts/run_contact_ab_matrix.sh \
  --environment SimplePlane \
  --ground-topology simple_plane_only1_v1 \
  --repeats 1 \
  --output-dir data/reports/contact_ab/simple_plane_schema4_smoke_190f357
```

结果为 6/6 run、36/36 segment complete、analysis 6 included / 0 excluded /
6 groups、matrix complete，脚本 exit 0。六份 motion report 均为 schema 2；runtime
provenance 均为 schema 5、`verified=true`、Git dirty false、commit 精确匹配
`190f357`；36 个 segment 都包含 schema-1 `final_half_of_command_interval` 稳态窗口，
最少 151 个 Odom 样本。`analysis.json` 为 schema 3，SHA256
`9c9b423eac26db8e940d08bbd9996f3f8e09fc206902f086e1826654d0fbb37c`；
`batch_summary.json` 为 schema 4、`result=success`。43 列 manifest 为 mode `0444`，有 6 个数据行，
SHA256 `bc443124528af896a31f85239a8788a59eb3614bb3adad8e62c39905953d36cd`；
12 类 path/hash 配对 × 6 行共 72 对（144 个 path/hash 叶检查）复核为
0 missing / 0 mismatch，summary 中冻结的两份 evidence SHA 也逐字匹配。严格
accounting validator 以 `expected_repeats=1` 复核通过。

物理结论按设计为 0 applicable / 6 N/A / 0 passing / 0 failed，
`all_applicable_groups_passed=null`；六组唯一原因都是
`fewer_than_3_unique_repeats`。这证明适用性和 schema 4 记账，不是物理 PASS/FAIL，
也不能替代正式每组三重复。描述性单次指标显示：前进横漂绝对值
`0.000006–0.001517 m`、后退 `0.000045–0.001281 m`；左右旋转漂移不对称
`0.1112–0.1405`。但左旋中心漂移 `0.2975–0.3047 m`、右旋
`0.3429–0.3493 m`，稳态 yaw-rate 绝对误差比例 `0.6323–0.6493`，都明显超过
计划 8.7 的 `0.10 m/0.10` 门，因此在投入 54-run 前先继续有效轮距/动力学单变量 A/B。

六份 Isaac 日志的 Kit `[Error]` 为 0，高风险 Fatal/Traceback/Segfault 模式为 0；
仍各有一条非致命 gRPC protobuf 重复注册 `E0000` 和 `W0000`，合计各 6 条，并有
108 条 Kit warning。runner 日志没有 error/failure 模式。故本批不能表述成“零错误”，
上游 protobuf 重复注册噪声继续作为已知边界保留。

### 有效轮距候选的 schema-2 失败筛选（2026-07-15）

为避免把明显欠转的 stable `0.37559 m` 直接扩成 54-run，先只改变 robot YAML 的
`effective_track_width`，固定 SimplePlane、`simple_plane_only1_v1`、Ideal、solver
`32/4`、60 Hz 和 legacy contact。两次批处理都在首个 report 后由旧 schema-2 严格
验证器失败关闭，manifest 状态为 `failure / motion_report_verification_failed`，没有
生成 analysis 或 summary，也不能计作正式 repeat：

| 候选 | clean commit | report raw / canonical SHA256 | manifest SHA256 |
| --- | --- | --- | --- |
| `1.012 m` | `05fdba7da7b0f8f1f6b0208a653b84d0e7f20e72` | `9318a70f...6c226` / `1303cebb...58c3` | `a70145e2...cc50` |
| `0.989 m` | `8d1c5f4e03ad29030a258404662c7510ce48180a` | `913ab6d7...409eb` / `516835dc...557` | `d429d91a...7b0` |

失败根因不是 Isaac 崩溃。两份 report 都是 6/6 segment complete，runtime provenance
schema 5、`verified=true`、Git dirty false；Kit `[Error]`、Fatal/Traceback/Segfault 均为
0，但每份日志仍各有一组非致命 gRPC protobuf `E0000/W0000`。旧 report 只保存整段
JointState 的样本数、minimum/maximum、mean、mean-abs、peak-abs、RMSE 与方向分类，
没有可认证的 JointState 稳态窗口；候选轮距降低了圆弧内侧目标轮速，少量越过 `±0.2 rad/s`
deadband 的反向值就把整段分类成 `mixed`。旧 analyzer 又只允许纯旋转出现 mixed，
因此把真实观察误作 invalid protocol；它既不能证明反向只发生在启动期，也不能形成
正常的 physical direction FAIL。这一证据缺口直接触发了上节的 schema-3 closed
JointState window 与 physical schema-2 方向叶设计，旧报告不得事后补造稳态数据。

描述性物理指标仍可用于选择下一次重跑顺序，但不是机器 verdict：

| 候选 | 左/右稳态 yaw 误差 | 左/右中心漂移 | 漂移不对称 | 解释 |
| --- | ---: | ---: | ---: | --- |
| stable `0.37559 m` 六 profile 范围 | `63.23–64.34% / 64.06–64.93%` | `0.2975–0.3047 / 0.3429–0.3493 m` | `11.12–14.05%` | 对称严重欠转 |
| `1.012 m` legacy | `8.61% / 0.34%` | `0.03247 / 0.01829 m` | `43.67%` | yaw/绝对漂移进入门，但不对称失败 |
| `0.989 m` legacy | `2.95% / 0.02%` | `0.05302 / 0.05871 m` | `9.70%` | 除未认证稳态轮向外，其余 17 项单次推演均在门内 |

`0.989 m` 因而被选作下节 schema-3 clean smoke；首轮六 profile 已重新采集，但仍必须
执行每组三个唯一 repeat。其 arc 稳态 yaw 仍只有约 `+0.096/-0.111 rad/s`（命令
`±0.4`），不属于现有 18 项硬门但属于尚未解决的运动逼真度边界；即使正式 8.7 PASS，
也不能写成圆弧动态已经完全标定。

### schema-3 SimplePlane 0.989 m 六 Profile 真实烟测（2026-07-15）

在 clean `22a77465b1a2c8c9c685683cfe829d46bc08ac48` 上执行：

```bash
./scripts/run_contact_ab_matrix.sh \
  --environment SimplePlane \
  --ground-topology simple_plane_only1_v1 \
  --repeats 1 \
  --robot-config \
    isaac_sim/configs/robots/experimental/jackal_etw_0p989_v1.yaml \
  --output-dir \
    data/reports/contact_ab/simple_plane_etw_0p989_schema5_screen_22a7746
```

这是当时 v2 合同的首个真实 schema-3 批次。结果为 6/6 run、36/36 segment complete、
analysis 6 included / 0 excluded / 6 groups、matrix complete，脚本 exit 0。六份 report
均为 schema 3，runtime provenance 均为 schema 5、`verified=true`、Git dirty false；
`analysis.json` 为 schema 4，`physical_acceptance` 为 schema 2 / policy
`skid_steer_plan_8_7_v2`，`batch_summary.json` 为 schema 5、`result=success`。44 列
manifest 有 6 行且 mode `0444`，逐行 status `success`、report schema `3`。严格 accounting
以 `expected_repeats=1` 重读原报告并通过。

证据 SHA256：

| 工件 | SHA256 |
| --- | --- |
| `manifest.tsv` | `2468d024780034dda9251928a8d881d2db5828c7036f710de509a41c8d7d447a` |
| `analysis.json` | `ebe612d7dfc9799fbb8f6bdc0dbe49cf8745b02411827b5ec4bd22aedd311881` |
| `batch_summary.json` | `fa2042dc8eee87d9f7a9fd8c75244a7906ac78c3aa97c13e394ec8fbb2761297` |

物理结论严格为 0 applicable / 6 N/A / 0 passing / 0 failed，
`all_applicable_groups_passed=null`；六组唯一原因都是
`fewer_than_3_unique_repeats`。这不是物理 PASS。把每份单次观察仅作下一轮排序投影时，
结果如下；“投影通过”表示这一次落在全部 18 个 v2 叶边界内，不改变正式 N/A：

| Profile | 左/右中心漂移 | 漂移不对称 | 左/右稳态 yaw 误差 | 稳态轮向 | 单次投影 |
| --- | ---: | ---: | ---: | --- | --- |
| `legacy_baseline` | `0.03513 / 0.07092 m` | `50.46%` | `6.39% / 1.26%` | 6 段全匹配 | FAIL：不对称 |
| `threshold_corr_0p00025_offset_0p0004` | `0.03513 / 0.11679 m` | `69.92%` | `6.39% / 0.68%` | 6 段全匹配 | FAIL：右漂移、不对称 |
| `threshold_corr_0p025_offset_0p0004` | `0.02511 / 0.08822 m` | `71.53%` | `6.54% / 0.49%` | 6 段全匹配 | FAIL：不对称 |
| `threshold_corr_0p00025_offset_0p04` | `0.05302 / 0.05871 m` | `9.70%` | `2.95% / 0.02%` | 6 段全匹配 | 投影通过 |
| `threshold_corr_0p025_offset_0p04` | `0.02511 / 0.04467 m` | `43.78%` | `6.54% / 0.53%` | 6 段全匹配 | FAIL：不对称 |
| `explicit_material` | `0.05302 / 0.08723 m` | `39.22%` | `2.95% / 1.07%` | 6 段全匹配 | FAIL：不对称 |

六份报告的前进横漂绝对值为 `0.000005–0.001577 m`，后退为
`0.000045–0.001176 m`；36 段双流静止证据时长为 `0.500000–0.516667 s`。Clock、Odom、
JointState 的 regression/duplicate 和 Odom/JointState invalid message 总数都为 0。
36 个稳态轮向窗口全部匹配；整段描述窗口则如实保留 18 个启动期 mismatch，证明新合同
没有通过删除瞬态来伪造整段结果。六组 arc 稳态 yaw 仍约为
`+0.0961/-0.1109..-0.1110 rad/s`，远低于命令 `±0.4 rad/s`，仍是未进入 18 项硬门的
运动逼真度缺口。

六份 Isaac 日志的 Kit `[Error]`、Fatal/Traceback/Segfault 为 0，runner 的
error/failure 模式为 0；Kit warning 共 108 行。每份 Isaac 日志仍各有一组非致命 gRPC
protobuf `E0000/W0000`，合计各 6 条，不能写成“零错误”。同一 `legacy_baseline` 的单次
不对称从旧筛选的 `9.70%` 变为本批 `50.46%`，进一步说明不能用 repeat=1 冻结参数；
下一步必须执行 SimplePlane/only1 六 profile × 三个唯一 repeat。

### schema-3 SimplePlane 0.989 m 六 Profile 正式三重复（2026-07-15）

在上述单次 smoke 之后，于 clean
`897372866baa634d31d067a5b3ed5add74783acb`、同一分支和同一冻结输入上执行：

```bash
./scripts/run_contact_ab_matrix.sh \
  --environment SimplePlane \
  --ground-topology simple_plane_only1_v1 \
  --repeats 3 \
  --robot-config \
    isaac_sim/configs/robots/experimental/jackal_etw_0p989_v1.yaml \
  --output-dir \
    data/reports/contact_ab/simple_plane_etw_0p989_schema5_repeat3_8973728
```

批次机制完整成功：18/18 run、108/108 segment complete；44 列只读 manifest
有 18 行且全部 `success`，18 份 report 均为 schema 3；analysis 为 schema 4，纳入
18、排除 0、形成 6 个完整组；physical acceptance 为 schema 2，六组全部适用；summary
为 schema 5、`result=success`。公共 accounting 重读全部原报告并复算身份、双 SHA、
Reset/时间/停止/姿态/分布和每个物理叶后通过。冻结根证据为：

| 工件 | SHA256 |
| --- | --- |
| `manifest.tsv` | `622ee56fb33f3a679e2db0d97c9920e0d07cba23c5a6fa8de2b85ccad725fe4c` |
| `analysis.json` | `82bbd0fb286972df9c5e00469745e2afda97565fe83730f981a7d33de09cfcfb` |
| `batch_summary.json` | `f879db530c2b2d9439b3ee4c0ad23f6db86b153827acd2226593496cdee6e5a0` |

物理结论与机制结论必须分开：`all_applicable_groups_passed=false`，6/6 applicable
group 全部 failed，0 passing、0 N/A。18 个 repeat 中有 8 个逐项通过、10 个失败；
每个失败都只来自
`rotation_center_drift_asymmetry_ratio <= 0.20`，其余 17 项逐重复检查全部通过。
因为策略是 `evaluation_basis="every_repeat"`，2/3 通过也不能把 group 判为 PASS：

| Profile | 通过重复 | 左中心漂移 m（r1/r2/r3） | 右中心漂移 m（r1/r2/r3） | 不对称率（r1/r2/r3） |
| --- | ---: | --- | --- | --- |
| `explicit_material` | 1/3 | `0.053018 / 0.035133 / 0.035133` | `0.066732 / 0.071537 / 0.032957` | `0.205502 / 0.508889 / 0.061926` |
| `legacy_baseline` | 2/3 | `0.053018 / 0.053018 / 0.053018` | `0.087229 / 0.058710 / 0.058710` | `0.392192 / 0.096950 / 0.096950` |
| `threshold_corr_0p00025_offset_0p0004` | 2/3 | `0.035133 / 0.053018 / 0.035133` | `0.071537 / 0.058710 / 0.039660` | `0.508889 / 0.096950 / 0.114146` |
| `threshold_corr_0p00025_offset_0p04` | 2/3 | `0.053018 / 0.053018 / 0.053018` | `0.043602 / 0.070054 / 0.043602` | `0.177598 / 0.243182 / 0.177598` |
| `threshold_corr_0p025_offset_0p0004` | 0/3 | `0.025113 / 0.032775 / 0.032775` | `0.088220 / 0.041256 / 0.024031` | `0.715335 / 0.205585 / 0.266771` |
| `threshold_corr_0p025_offset_0p04` | 1/3 | `0.032775 / 0.025113 / 0.025113` | `0.032401 / 0.088220 / 0.044670` | `0.011389 / 0.715335 / 0.437808` |

`threshold_corr_0p00025_offset_0p04` 是下一轮诊断的优先输入，不是自动选出的合格
Profile：它 2/3 次通过，唯一失败值 `0.243181692` 只比固定上限高
`0.043181692`；正式报告仍明确写 `ranking_policy="none; pass/fail only"`。
其他硬门保留了明显余量：全批最大前进/后退横漂分别为 `0.001911/0.001226 m`，
最大单侧旋转中心漂移为 `0.088220 m`，左右稳态 yaw-rate 最大相对误差分别为
`6.542%/5.264%`；108/108 停止窗、432/432 个稳态轮向观察和全部时间戳/Reset/
样本记账均通过，invalid Odom/JointState 消息为 0。

失败形态呈离散、可重复的中心漂移水平，而不是轮向错误或持续 yaw-rate 欠标定。
被测 clean `8973728` 的每段 Reset 会恢复根 pose、根速度、DOF 速度和 velocity target，
但没有显式恢复
DOF position；连续轮 joint 和 PhysX 接触暖状态因此成为下一步必须隔离的变量。这个
判断是由结果产生的根因假设，不是已经证明的修复结论；修复后必须在新 clean commit
上重新运行同一 18-run 批次，旧失败样本不得删除或混入新批。

日志审计中 Isaac Kit `[Error]`、Fatal、Exception、Traceback、segfault/core、runner
error/failure 均为 0；324 条 Kit warning 在 18 个进程中分布一致。每份 Isaac 日志仍
各有一条非致命 protobuf `E0000` 和一条 `W0000`，合计各 18 条；18/18 日志还记录 CPU
governor 为 powersave。因此这批既不能写成“零错误”，也不能外推成性能基准。

### 完整关节状态 Reset 后的正式三重复复测（2026-07-15）

clean `55418fe2eee507e9d3b690eb84584862c350b2db` 修复了上一批暴露的 Reset
隐藏状态：初始化后保存全部 DOF position，Reset 时恢复该快照，并把 DOF velocity、
velocity target 和 effort 清零；四类状态都通过 tensor 读回验证。随后保持机器人、
环境、topology、六个 contact profile、Mapping/Ideal/60 Hz、TGS 32/4、运动脚本与
三重复政策不变，重新执行同一正式子矩阵：

```bash
./scripts/run_contact_ab_matrix.sh \
  --environment SimplePlane \
  --ground-topology simple_plane_only1_v1 \
  --repeats 3 \
  --robot-config \
    /home/lyb/Workspace/Isaac_Sim_ROS2_Nav/isaac_sim/configs/robots/experimental/jackal_etw_0p989_v1.yaml \
  --output-dir \
    data/reports/contact_ab/simple_plane_etw_0p989_schema5_repeat3_resetstate_55418fe
```

证据机制再次完整成功：18/18 run、108/108 segment complete，18 份 motion report
均为 schema 3、runtime provenance 均为 schema 5；analysis 纳入 18、排除 0、形成
6/6 完整组；44 列 manifest 有 18 行、mode `0444`。summary schema 5 的
`result="success"` 只表示运行、身份、矩阵和聚合成功。严格复验覆盖 216 个 manifest
path/SHA 配对（0 missing、0 mismatch、0 越出仓库）、18/18 规范 `output_file`、公共
physical-accounting validator 和原始 18 报告离线重聚合；重聚合文件与冻结
`analysis.json` 字节完全一致。冻结根证据为：

| 工件 | SHA256 |
| --- | --- |
| `manifest.tsv` | `2dc3aba651ff0eb253687c12c32fe2156a827af9444ad7c2dc39c76ed5a03866` |
| `analysis.json` | `51903b770da88030c5d56418771408e54d02fcd195d176407be1b9be7773cc10` |
| `batch_summary.json` | `b85dc9188bcb0d783570993850fc173966803eb7b5eb25c9911f9fd5c0b6c2f1` |

物理结论仍是 FAIL，但修复产生了可量化改善：passing group 从旧批的 0/6 增加到
3/6，逐 repeat 全门通过从 8/18 增加到 12/18。`explicit_material`、
`legacy_baseline`、`threshold_corr_0p00025_offset_0p0004` 三组正式通过；其余三组
仍按 `every_repeat` 失败：

| Profile | 通过重复 | 左中心漂移 m（r1/r2/r3） | 右中心漂移 m（r1/r2/r3） | 不对称率（r1/r2/r3） |
| --- | ---: | --- | --- | --- |
| `explicit_material` | 3/3 | `0.064496 / 0.064496 / 0.064496` | `0.060962 / 0.060962 / 0.061023` | `0.05479 / 0.05479 / 0.05384` |
| `legacy_baseline` | 3/3 | `0.064496 / 0.064496 / 0.064496` | `0.061023 / 0.060962 / 0.060962` | `0.05384 / 0.05479 / 0.05479` |
| `threshold_corr_0p00025_offset_0p0004` | 3/3 | `0.064496 / 0.064496 / 0.064496` | `0.060962 / 0.060962 / 0.061023` | `0.05479 / 0.05479 / 0.05384` |
| `threshold_corr_0p00025_offset_0p04` | 1/3 | `0.064496 / 0.047209 / 0.047209` | `0.061023 / 0.060962 / 0.060962` | `0.05384 / 0.22561 / 0.22561` |
| `threshold_corr_0p025_offset_0p0004` | 2/3 | `0.044670 / 0.044670 / 0.044670` | `0.053916 / 0.118818 / 0.053916` | `0.17149 / 0.62405 / 0.17149` |
| `threshold_corr_0p025_offset_0p04` | 0/3 | `0.040198 / 0.040198 / 0.044670` | `0.053916 / 0.053916 / 0.118818` | `0.25444 / 0.25444 / 0.62405` |

18/18 左、右稳态 yaw-rate 检查、432/432 稳态轮向观察、108/108 停止窗和
72/72 stop-config 叶全部通过。失败叶只有 6 次
`rotation_center_drift_asymmetry_ratio` 与其中 2 次同时发生的
`rotate_right_center_drift_m`。这证明完整 DOF 状态恢复是有效修复，但不能证明已经
消除接触形成/solver 数值路径的离散分支；每个 repeat 已经是独立 Isaac 冷进程，不能
把现象解释为上一进程的缓存直接存活，也不能以 12/18 或组均值冒充物理通过。

18 份 Isaac 日志的 Kit `[Error]`、Fatal/Exception/Traceback/segfault 和 runner
error/failure 均为 0；非致命 protobuf 重复注册 `E0000/W0000` 各 18 条。Kit warning
共 327 条：常规 324 条加 run 14 的 3 条 `UsdNoticeHandler` attribute-type warning，
未造成运行失败；18/18 仍记录 powersave governor。因此该批也不是性能基准。

下一步不直接改 contact 数值或有效轮距。必须先把版本化 `reset_strategy` 写入 runtime
provenance、motion report、版本化 manifest（必要时升版）、matrix readiness 和 group identity，再用同一 clean
commit 做 A/B：A 保留 `pose_restore_v1`；B 在 SimplePlane 上固定执行
`separate_recontact_0p20m_1step_v1`（抬升 0.20 m、固定一步确认脱离接触、精确回写出生
Pose、再执行原有一步）。首轮只锁
`threshold_corr_0p00025_offset_0p04`、ETW `0.989 m`、60 Hz、TGS 32/4，两臂按
A/B、B/A 交替各至少 10 个冷进程。旧 `8973728` 早于 DOF 修复，不能充当该 A/B 的
control；这个方案目前只是待验证假设，不是已完成或已证明的修复。

### SimplePlane 六 Profile 严格批处理烟测（2026-07-14）

在 clean commit `a8863d2822aeb8f5f1134be534f32c81e2670d78` 上执行：

```bash
./scripts/run_contact_ab_matrix.sh \
  --environment SimplePlane \
  --repeats 1 \
  --output-dir data/reports/contact_ab/simple_plane_smoke_a8863d2_diagnostic
```

六个 Profile 各自使用独立 Isaac 进程，全部 motion 报告为 `result=success`，且
每份 6 个 segment 均为 `complete`、`failed_segments=[]`、runtime provenance
schema v3 `verified=true`；最终 `batch_summary.json` 为 `result=success`，
实际/预期运行数、manifest 行数均为
`6/6`。严格聚合结果为 `analysis_valid=true`、完整矩阵、6 个 group、纳入 6 份、
排除 0 份，缺失 group 为 0。所有报告都记录
`environment.id=SimplePlane`、分支 `codex/navigation-quality-fidelity`、上述 commit
和 `git.dirty=false`。36 次 Reset recovery 全部成功，wall latency 为
`0.5097–3.7595 s`，均值 `1.1929 s`。

独立只读审计还复算了 manifest 中报告/日志/配置/Stage 声明哈希、分析与 summary
中的 path/hash 引用，以及六份报告的 canonical SHA256，均与磁盘内容一致；本地、
origin tracking 和 GitHub 远程分支在烟测与审计时也都指向同一被测 commit。

关键本机证据及 SHA256：

| 文件 | SHA256 |
| --- | --- |
| `manifest.tsv` | `024c5cffe5bb99c2fd3dd1c9f3e6a3b215e8651804fbcf3365d8ed526a7f1551` |
| `analysis.json` | `535716f271d07954357a963f9a29fa4e6acc1d761729bd763c1b1cc35032d773` |
| `batch_summary.json` | `6d2e4c81b04eae188b4bea4b25854b7505c93464915cd61d00ce28de4c79d2ef` |

烟测前的失败证据没有删除，也不能用最终成功覆盖解释：

- `simple_plane_smoke_b8bff52` 的首份 motion 报告实际完成，但当时分析器把纯旋转
  的真实 `mixed` 瞬态错误当作必然失败；报告 SHA256 为
  `98ced680553e723b4d733364041d8bc7e13f6dd1e92366e823feb678990c4f2d`。
- `simple_plane_smoke_02327e3` 六份报告都完成，旧聚合器却把随 contact Profile
  合理变化的 composed root Layer SHA 错锁成环境全局常量。修正锁作用域后对原始
  六报告离线复验为 `analysis_valid=true`、纳入 6/排除 0、矩阵完整；复验 JSON
  SHA256 为 `fea97b404f8f18ed2a8a142d067ee75591b57b1b66f3d50d9bac3b2be114814f`。
- `simple_plane_smoke_75de37a` 与 `_retry1` 均在首段 Reset recovery 30 秒超时，
  失败报告 SHA256 分别为
  `29b2fadba5869dc09f86397ac6aee684eb25fa8e75d7471ca9c5594d8bd42113`、
  `693922352426d0786259d0c480ffaa5295b0cf2965f79dd1340b7c74b944efbd`。
  旧报告只证明 fresh streams 与 stationary chassis 的联合门超时，不能事后伪造
  某个具体速度门根因。提交 `a8863d2` 已增加逐门违规计数、峰值、末端 blocker 和
  最长连续静止窗；本轮未再复现，因此仍保留为待 soak 定位的间歇问题，未放宽
  30 秒或速度阈值。

#### Reset epoch/coherent recovery 修复后复验（2026-07-14）

后续正式矩阵 `data/reports/contact_ab/skid_steer_v1_bcfc201` 在第 8 轮
`008_simple_plane_threshold_corr_0p025_offset_0p0004_r02` 失败关闭。失败报告
SHA256 为 `38452edbc39263b234fecbc6cc8e9f534a242d84461673e958641ddabf775fdc`，
manifest SHA256 为 `4c516937d585b50cffbcf9b317bc7ecc3dd9fb5c46d3739e6ce5e513b47607be`。
当时所有速度门均通过，但 `/clock`、Odom、JointState 的非原子 callback 相位反复清空
静止窗；该目录和失败样本保留，不续跑、不删除，也不混入新的正式矩阵。

提交 `d7710b7f07039c18a38dcabc68e3958bbec13bf8` 为 Reset Trigger 增加版本化
generation/boundary trailer，并让 runner 以服务端 boundary、Trigger 等待窗口逐 Topic
历史最大 timestamp、response barrier 当前 stamp、三路 receive/credited sequence 和
timestamp 水位共同消费相干组。Clock 单独推进、旧队列、覆盖式回退、断流和已知运动
均不能跨窗；同一 60 Hz tick 的有界 callback 相位仍可恢复。该提交通过：

- Reset/运动聚焦测试 `81 passed`；
- `robot_experiments` 根目录与标准隔离 `colcon test` 均为 `271 passed`；
- Isaac 测试 `121 passed, 21 skipped`；
- contact batch/runtime 脚本测试 `56 passed, 1 skipped`，skip 原因为本机未安装
  `shellcheck`；
- fatal/F401 flake8、`compileall` 与 `git diff --check` 均通过；三轮独立状态机复核
  均未发现 P0/P1。

在该 clean commit 上重新执行：

```bash
./scripts/run_contact_ab_matrix.sh \
  --environment SimplePlane \
  --repeats 1 \
  --output-dir data/reports/contact_ab/simple_plane_smoke_d7710b7
```

六个独立 Isaac 进程全部完成，6 份报告均为 `result=success`、每份 6 个 segment
均为 `complete`；summary 为 `result=success`，实际/预期运行数、manifest 行数均为
`6/6`。严格聚合为 `analysis_valid=true`、矩阵完整、6 个 group、纳入 6、排除 0。
36 次 Reset 的 versioned trailer、generation/boundary、三路 fresh 标志、静止时长和
stamp 下界检查全部通过，recovery wall latency 为 `0.5287–0.5579 s`，均值
`0.5401 s`；service latency 为 `0.1553–0.1831 s`，均值 `0.1657 s`。

全窗审计记录 15 个 pre-boundary group 和 13 个 JointState receive timestamp
regression；非零 violation 只有
`receive_timestamp_regression:joint_states=13`，这些旧队列证据均被拒绝。coherent/
observation regression、not-stationary、速度、wall freshness、stale/future-skew 和四轮
速度 violation 全为 0。恢复期 Odom 线速度峰值 `0.000325 m/s`、角速度峰值
`0.000227 rad/s`，远低于 `0.02 m/s` 与 `0.05 rad/s` 门；JointState sim age 为
`-0.0000036–0.0333369 s`，Odom 为 `0–0.0166667 s`。

独立 `sha256sum --check --strict` 对 manifest 中 6 份报告、6 份 Isaac 日志和 6 份
runner 日志共 18 个文件全部返回 `OK`。根证据哈希为：

| 文件 | SHA256 |
| --- | --- |
| `manifest.tsv` | `f908728da1fb90ed6f087869cf18502881ee5315939a60fd460edf0a92d33163` |
| `analysis.json` | `27fc2681e8c1321c8a7e02021044e67c7bce22993667aa1e5393372f0ff1ae1f` |
| `batch_summary.json` | `0f4fba89eb76daa21eba4eaeb9eb3f771a866552934d7a4cfac0075e3dc77730` |

这仍是每组 1 次的机制 smoke，只证明 `d7710b7` 的真实 Reset/批处理链路恢复，不能
替代两环境 × 六 Profile × 三重复的新正式矩阵，也不能用于自动选择 contact Profile。

这次 `--repeats 1` 只验收六 Profile 串行启动、运行态 provenance、报告结构、
聚合身份锁和失败保留机制。每组样本数为 1，不能估计方差、排名或选择材质；计划
当时要求的 SimplePlane/Warehouse 每组至少 3 次正式矩阵仍未完成；随后已在下一节
从新目录执行，不能把本节 smoke 样本混入该正式矩阵。

### SimplePlane/Warehouse 六 Profile × 三重复正式接触矩阵（2026-07-14）

在 clean commit `0500f9eebd71836977e466484cad828715bb8228` 上从空目录执行：

```bash
./scripts/run_contact_ab_matrix.sh \
  --environment all \
  --repeats 3 \
  --output-dir data/reports/contact_ab/skid_steer_v1_0500f9e
```

矩阵按 SimplePlane→Warehouse、六 Profile、repeat 1/2/3 的固定顺序启动 36 个独立
Isaac 进程。36/36 run、216/216 motion segment、216/216 Reset 均完成；12 个
环境×Profile group 各有三个唯一样本，严格聚合 `analysis_valid=true`、matrix
complete、纳入 36、排除 0。36 份报告、36 份 Isaac 日志和 36 份 runner 日志共
108 个 Manifest checksum 经独立 `sha256sum --check --strict` 全部通过，离线重新聚合
与 `analysis.json` 完全相等。根证据哈希为：

| 文件 | SHA256 |
| --- | --- |
| `manifest.tsv` | `f38bdbdfcd2a7a9b1fe53fe8dcdf3bb6af07993f45a1f75859a158ea372eb7df` |
| `analysis.json` | `e8095612eb792a9751c0233c55aca0d18c4cb3bd1f4fb631bd2b4b4da75791ea` |
| `batch_summary.json` | `f22c0079b1a4597163e9e6ab383473e0a320405928e53153f4b1e7f7165c3185` |

Reset metadata/generation/boundary、水位和三路 freshness 为 216/216；每个进程的
generation 为 2–7。服务 latency 为 `0.1547–0.1917 s`、均值 `0.1694 s`；恢复
latency 为 `0.5278–0.6071 s`、均值 `0.5427 s`。恢复期最大 Odom 线/角速度和轮速
分别为 `0.000432 m/s`、`0.000847 rad/s`、`0.0089 rad/s`，均低于
`0.02/0.05/0.2` 门。119 个 pre-boundary group 与 105 个 JointState receive
timestamp regression 均被拒绝；coherent/observation regression、stale、future、
wall freshness 和物理速度 violation 为 0，运动段内三路 timestamp 也全部单调唯一。

上面的结果只说明证据链、Reset 恢复、停止和批处理合同通过。下表是当时按整段均值
做的探索性阈值比较，不是当前计划 8.7 的机器 verdict：这些历史 motion report 都是
schema 1，没有命令后半段 `steady_state_window`，因此当前合同下连 SimplePlane + Ideal
组也为 N/A；Warehouse 还额外不满足环境/topology 适用性。

| 历史探索阈值 | 结果 | 当前解释 |
| --- | --- | --- |
| 直行 3 m 横向偏差 `≤0.05 m` | 最坏 `0.001911 m` | 历史描述值在阈值内；当前 verdict N/A |
| 倒车 2 m 横向偏差 `≤0.08 m` | 最坏 `0.001144 m` | 历史描述值在阈值内；当前 verdict N/A |
| 原地旋转中心漂移 `≤0.10 m` | `0.297486–0.350392 m` | 历史描述值超阈值；当前 verdict N/A |
| 左右旋转漂移差 `≤20%` | SimplePlane 六组按 `abs(L-R) / max(L,R)` 为 `11.22%–14.19%` | 历史 SimplePlane 描述值在阈值内；当前 verdict N/A；Warehouse 不适用 |
| 整段平均角速度误差 `≤10%` | `60.10%–69.05%` | 不是后半命令稳态窗口，不能代替当前角速度门 |
| 零命令后 0.5 s 接近零 | 216/216 均形成 0.5 s 连续静止窗；确认 `0.5167–0.5667 s` | 历史停止描述值；当前整体 verdict 仍为 N/A |
| 四轮速度方向符合契约 | 非旋转 144/144 段全匹配；纯旋转 72/72 段有短暂 `mixed`，主导平均符号正确 | 仍需独立 8-trial 单轮硬门，不能无条件签字 |

历史描述性 yaw gain 只有 `0.2406–0.3461`；该值不是当前机器 yaw-rate 门。threshold 和当前显式材质对直线、
圆弧以及欠转的影响都很小，不能把某个 Profile 写成赢家。Profile
`threshold_corr_0p025_offset_0p04`（correlation `0.025`、offset `0.04`）的跨环境总体有效轮距最接近
`1.012 m`，但 SimplePlane 左右方向仍相差约 20%，只能作为下一轮探索初值。
SimplePlane 六组左右漂移差通过，Warehouse 六组为 `28.93%–37.62%`，全部失败；
Warehouse 的旋转不稳定集中在左转支路，应先隔离 31 个 floor-decal collider 与
GroundPlane、补两环境单轮诊断，再做 60/120 Hz、CCD、stabilization 的单变量筛选。
有效轮距必须用 `w=±0.2/±0.4/±0.8 rad/s`、两环境、每方向至少三次重新拟合，之后
才能同步控制器与 Wheel Odom 并进入 Realistic A/B。

72 个纯旋转段的每轮主导平均轮速符号正确，但都含短暂 `mixed`，因此
`all_directions_match=false`；分析器只在最小/最大跨 deadband、逐轮布尔一致且平均
符号正确时纳入。该例外不能替代独立 8-trial 单轮方向硬门。Isaac 日志无
Traceback、Error、Reset timeout、BVH、simulation-time、customGeometry 或 TGS
命中，但仍有 645 条 Hydra/Semantics/Timeline/USD/`frictionType` 等 warning 噪声，
后续需单独清理或建立有来源的 allowlist。

### 单轮正/负方向真实诊断（2026-07-14）

提交 `cf27605` 增加 standalone 诊断。它在四个 wheel joint 上分别执行
`+1/-1 rad/s` 共 8 个 trial，只给 active DOF 非零目标，并以固定
`dt=1/60 s` 读取精确 ground-filter contact、法向力、摩擦、轮底速度和底盘响应。
真实 Warehouse + legacy Profile 报告
`/tmp/wheel_direction_diagnostic_smoke4.json` 为 `result=success`，SHA256：

```text
85ffe058245561b875e59d36b875f1ffbd31ecac233e8502d2e6c7336f652cd9
```

报告实测 32 个 ground filter、四个启用 ContactReportAPI 的 wheel rigid body，
15 个连续物理步达到接触就绪。8/8 trial 的以下硬门全部为 `true`：目标读回、active
轮速及符号、ground contact、法向力、轮底 spin 反向、摩擦力方向、摩擦冲量方向、
底盘运动方向，以及 detailed contact 与 normal-force matrix 的一致性；
`cross_trial.failed_trials=[]`。

零目标自由轮的实际 p95 轮速为 `0.120288–0.268468 rad/s`，全部超过配置的
`0.10 rad/s` advisory，但 active-rate 比值只有 `0.115621–0.270699`；由于目标
读回仍为零，这些值按设计记录为底盘/地面耦合证据，而不是放宽命令隔离硬门。
四轮正负摩擦幅值比为 `1.31962 / 1.26280 / 0.97486 / 0.85030`，均处于 advisory
范围 `[0.5, 2.0]`。

对应 Kit 日志
`kit_20260714_173253.log` 的 SHA256 为
`bc1a39e14f7e1039be2aeda788052276489f910fb9007e452a907073cf2a9df4`；
`[Error]`、缺失 ContactReportAPI 和 filter-count mismatch 都为 0。日志仍有 15 条
warning，其中 6 条是 PhysX
`PxShape::getMaterialFromInternalFaceIndex ... 0xFFFFFFFF`，因此不能写成“零警告”。
当前纯 Python 定向测试 `test_wheel_direction_diagnostic.py` 为 `22 passed`。

这份实跑报告使用 runtime provenance schema v2，Git 快照记录 commit
`dcb5ca2b3d712b8dfdedc20e11fad014e8e3174e` 且 `dirty=true`，因为当时 wheel
diagnostic 文件尚未提交；后来代码才以 `cf27605` 固化。它可以证明这次真实物理
方向行为和报告门，但不是 clean-commit 最终冻结证据。正式验收必须在最终代码与
provenance 契约冻结后用 clean Git 重新运行；单轮成功也不等于低速左右对称、接触
材料或有效轮距已经解决。

schema v3 合入并提交文档后，已在上述 clean commit `088bfda` 重新执行同一诊断。
新报告 `/tmp/wheel_direction_diagnostic_final_v3.json` 仍为 8/8、所有硬门为真，且
contact 快照为 `legacy_baseline`、4 wheel/32 ground、Stage readback true、Git
dirty false：

```text
report SHA256: f63ec096d382b74952ccc15d13c59f24093e62b83cd725f525d0eb73d43da9de
Kit log: kit_20260714_175309.log
Kit log SHA256: 48c46980606c055f28de6feb6da6cec099cf0f74a392d23717f50095ecbaa9ae
```

被动车轮 advisory 与四轮正负摩擦对称比和上一轮数值一致。Kit 日志仍是 15 条
warning（含 6 条 `getMaterialFromInternalFaceIndex ... 0xFFFFFFFF`），`[Error]`、
ContactReportAPI 缺失和 filter mismatch 均为 0；因此 clean v3 证据解决了报告身份
边界，但没有消除或掩盖上游 PhysX material-face warning。

### 有效轮距离线拟合（2026-07-14）

提交 `121eafd` 增加 `effective_track_analysis`。工具按输入内容 SHA256 去重，只从
`result=success` 报告的 complete 左/右纯旋转段读取四轮均值与 yaw rate；命令非零
线速度、yaw/轮差为零、测量符号错误、NaN/Infinity 或 provenance 不一致都会拒绝
或进入明确 exclude 记录。输出同时保留 yaw-response OLS、direct OLS、过原点 TLS
及 side/tier/report 分组，不会自动写回配置。

现有五份本机历史 Warehouse + Ideal、solver 32/4、同 robot asset 报告的探索性
分析输出 `/tmp/effective_track_5.json`，SHA256：

```text
1a5c369c51cfc7a33c8029ea4d8ce9d51f603056f64adc75b88f15fad0ecc0af
```

选择门要求 `runtime_provenance.verified=true`、solver `32/4` 和 robot asset
SHA256
`bf870a06c9b974eea2607dd7f33bb536eb930f2a7795ed07f25def792b150a8a`。
5/5 报告纳入、0 个报告排除，共 30 个旋转样本；40 个直行/圆弧非旋转段只进入
审计 exclusion。overall 结果为：

| 拟合 | 有效轮距 |
| --- | ---: |
| yaw-response OLS | `1.0124019295 m` |
| direct OLS | `0.9956613217 m` |
| origin TLS | `1.0040302614 m` |

yaw-response OLS 的过原点 `R²=0.983464`，但 left/right 已分成
`0.978724/1.049072 m`；low/nominal/high 更分成
`1.338142/1.050950/0.975149 m`。这种速度依赖和左右差异说明单一等效轮距仍在吸收
接触/滑移效应。五份输入中三份是历史 provenance schema v1、两份是 schema v2，
选择策略也没有绑定 contact Profile；因此 `1.0124 m` 只作为后续候选，不是冻结
配置。先完成接触矩阵、同一新 provenance 的独立重复，再对候选轮距做运动与
Realistic Odom A/B。当前 fitter + 安装契约定向测试为 `30 passed`。

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

2026-07-15 在 clean 提交 `c210150` 上重新执行完整三条门。该提交包含
per-treatment-group RootLayer 锁修复及完整锁层文档；构建、预检和
`test.sh --with-isaac` 均 exit 0。预检仍如实报告 342 个 Fast DDS SHM 遗留工件和
20 个 CPU core governor 非 performance 的环境警告，资产、地图、GPU 与其余门通过。
这次全门覆盖 schema v5、三个 topology profile、严格 topology 矩阵和修正后的
RootLayer 锁作用域；保留报告只读离线重聚合仍为
`12 included / 0 excluded / 12 groups`。单元门也不能替代正式 54-run topology、
Realistic 导航或 Warehouse V2 正式统计。

同日，先在未提交工作树上验证新 motion report schema 2、analysis schema 3、
physical acceptance schema 1 和 summary schema 4：完整 contact analyzer 测试文件
`116 passed`，motion baseline
`66 passed`，matrix script `42 passed / 1 skipped`；唯一 skip 是本机缺少
`shellcheck`。随后执行 `./scripts/test.sh`，exit 0；root pytest 为
`1061 passed / 1 skipped / 34 deselected`，ROS 为 11 packages、861 tests、0 errors、
0 failures、1 skipped。随后在 clean `2cd0788` 重新执行三条正式门：build 11 packages、
preflight PASS；`./scripts/test.sh --with-isaac` 的 root suite 为
`1076 passed / 1 skipped / 34 deselected`，ROS 为 11 packages、876 tests、0 errors、
0 failures、1 skipped，Isaac 为 `32 passed / 250 deselected`。唯一 skip 仍是本机缺少
`shellcheck`；preflight 如实报告 396 个 Fast DDS SHM 工件和 20 个非 performance
governor 的非阻塞环境警告。历史 schema-2 smoke 见上文“历史 v1/schema-2 SimplePlane
真实机制烟测”；SimplePlane 正式 18-run 子矩阵已在 clean `8973728` 执行并物理失败，
完整 54-run/18-group topology 矩阵仍待执行。

随后本轮 v2 方向合同完成 motion/analysis/physical/summary `3/4/2/5` 与 44 列
manifest 升级。三份定向文件合并为 `354 passed / 1 skipped`；在 clean `0484b72` 上，
build、preflight、`./scripts/test.sh --with-isaac` 均 exit 0，root 为
`1206 passed / 1 skipped / 34 deselected`，ROS 为 11 packages、1006 tests、0 errors、
0 failures、1 skipped，Isaac 为 `32 passed / 250 deselected`。新增负测已复现并关闭 impossible gap、
稀疏/单流陈旧停止证据、Reset 水位与命令次序、样本记账、姿态/航向几何、方向
counts/extrema/矩/稳态子集伪造、协调全局身份或 schema→N/A、wheel FAIL→PASS、
partial manifest、配置 JSON 类型混淆、asset/solver/simulation/UTC 时间篡改，以及早期
双日志删除/hash/symlink 篡改。clean `22a7746` 的 schema-3 repeat=1 smoke 已闭合；
clean `8973728` 的正式 SimplePlane 三重复证据链也已闭合，但 6/6 group 均因旋转中心
漂移不对称失败；完整 topology 矩阵仍待执行。

完整 DOF 状态 Reset 修复提交到 clean `55418fe` 后再次执行三条门：
`./scripts/build_ros2.sh` 完成 11 packages，`./scripts/preflight.sh` PASS，
`./scripts/test.sh --with-isaac` exit 0。root 收集 1246 项，结果为
`1211 passed / 1 skipped / 34 deselected`；ROS 为 11 packages、1006 tests、0 errors、
0 failures、1 skipped；Isaac marker 又独立复核为 `32 passed / 255 deselected`，完整
`isaac_sim/tests` 为 `255 passed / 32 skipped`。唯一测试 skip 仍是未安装 `shellcheck`；
preflight 另如实报告 518 个 Fast DDS SHM 工件和 20 个非 performance governor 的
非阻塞环境警告。随后同一 clean commit 的正式 18-run 复测从旧批 0/6 group 改善到
3/6 group，但整体物理门仍失败；详见上文完整关节状态 Reset 小节。

| Gate | 最近证据 |
| --- | --- |
| clean `65ae923` `pytest -q` | `1270 passed / 0 failed / 34 skipped` |
| clean `65ae923` `./scripts/test.sh --with-isaac` | exit 0；root `1268 passed / 1 skipped / 35 deselected`；ROS 11 packages、1035 tests、0 errors、0 failures、1 skipped；Isaac marker `32 passed / 1 skipped / 283 deselected` |
| clean `65ae923` Reset A/B formal | 20/20 run、20 included、0 excluded、两组各 10 repeat；physical 0 passing / 2 failed，A/B 分别有 5/10、6/10 repeat 因旋转中心漂移不对称失败 |
| clean `65ae923` Warehouse RTX profile | headless 默认 viewport disabled；8 秒点云 77 samples / 9.6028 Hz，Camera Off 的 Image/Info publisher 为 0 |
| clean `55418fe` `./scripts/test.sh --with-isaac` | exit 0；root `1246 collected / 1211 passed / 1 skipped / 34 deselected`；ROS 11 packages、1006 tests、0 errors、0 failures、1 skipped；Isaac marker `32 passed / 255 deselected`；唯一 skip 为缺少 `shellcheck` |
| 历史 v2 schema 定向测试 | contact analyzer `217 passed`；motion baseline `92 passed`；matrix script `45 passed / 1 skipped`；合并 `354 passed / 1 skipped`（缺少 `shellcheck`） |
| `./scripts/preflight.sh` | clean `55418fe`，2026-07-15 PASS；资产/地图/GPU 通过，另有 518 个 Fast DDS SHM 遗留工件和 20 个 CPU core governor 非 performance 的非阻塞环境警告 |
| `./scripts/build_ros2.sh` | clean `55418fe`，2026-07-15：11 packages build completed，exit 0 |
| `./scripts/test.sh --with-isaac` 的 pure/root suite | clean `55418fe`：1246 collected，1211 passed，1 skipped，34 deselected |
| ROS `colcon test` | clean `55418fe`：1006 tests，0 errors，0 failures，1 skipped |
| Isaac/USD marker suite | clean `55418fe`：287 collected，32 passed，255 deselected |
| RViz config/load smoke | 结构测试包含在当前 pure/root suite；安全 Panel 20/20 历史循环及本轮 Off/Monitoring/HQ 实跑组合见上文 |
| `robot_rviz_plugins` production-only build | 独立 `-DBUILD_TESTING=OFF` configure/build/install PASS |
| 2026-07-14 Camera 定向测试 | Camera contracts 15 passed；Camera/config 定向集合 27 passed；`isaac_sim/tests` 69 passed、3 skipped |
| 2026-07-14 既有底盘诊断定向测试 | 48 passed；真实 Warehouse + Ideal 14/14 段完整采集 |
| Contact Profile/Stage 定向测试 | `test_contact_setup.py + test_stage_composition.py -m isaac`：21 passed |
| Wheel direction 定向测试 | `test_wheel_direction_diagnostic.py`：22 passed；真实 Warehouse 8/8 trial 硬门通过，但报告 Git dirty |
| Clean schema v3 wheel direction | Warehouse + legacy：8/8 trial、全部硬门通过，Git dirty false；报告 SHA256 `f63ec096...a9de` |
| Clean schema v3 motion provenance | Warehouse + Ideal + legacy：14/14 complete，三路时间戳无重复/回退，Git dirty false；报告 SHA256 `8532187c...0f23` |
| Effective-track 定向测试 | fitter + package contract：30 passed；五报告探索拟合完成，但 contact/provenance 身份不足以冻结参数 |
| Contact A/B 聚合与 Reset 诊断 | 历史证据按原 schema 保留；当前 v3 合同在 clean `65ae923` 完成 Reset A/B 20/20 run、两组各 10 repeat。A/B 都只因旋转中心漂移不对称失败，分别 5/10 与 6/10 repeat 失败；B 不晋级，完整 54-run topology 矩阵仍待物理阻塞闭合后执行 |
| 2026-07-14 退出加固定向测试 | Runtime 脚本 34 passed；`robot_bringup` 176 passed；3 个顽固进程组用例连续 5 轮通过 |
| Map bundle 校验 | `warehouse_v1`、`warehouse_v2` 的真实 Manifest verify 均 PASS |
| Repository index set comparison | 当前 320 个 Git 跟踪路径对 320 个唯一索引路径；缺失、陈旧、重复均为 0 |
| Markdown 相对链接 | README、`plan.md` 与 10 个 `docs/*.md` 共 84 个本地链接，当前缺失为 0 |
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
| 底盘物理 A/B | 改动前与标准 Cylinder 下 Warehouse + Ideal 32/4、32/16 已完成；clean `8973728` 的候选 ETW 正式子矩阵 0/6 group，通过完整 DOF Reset 修复后的 clean `55418fe` 改善到 3/6 group。版本化 Reset A/B 已在 clean `65ae923` 完成 20/20，但 A/B 均因旋转不对称失败且 B 更差一例；Realistic、候选有效轮距、多角速度、接触拓扑和 solver 后续单变量 A/B 尚未完成 |
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
