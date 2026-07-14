# Isaac Sim 6.0.1 + ROS 2 Jazzy Jackal 二维 SLAM 与二维导航完整方案（修订版）

项目根目录：

```text
/home/lyb/Workspace/Isaac_Sim_ROS2_Nav
```

后文记为：

```bash
PROJECT_ROOT=/home/lyb/Workspace/Isaac_Sim_ROS2_Nav
```

该路径及原始系统目标来自现有方案。

## 0. 修订说明与当前状态（2026-07-15）

本文最初是项目从零搭建时的完整设计方案，后续章节仍保留当时的目标、SOP 和最终统计验收标准，便于回溯为什么采用当前架构。它不是“所有目标均已验收”的声明；第一次使用仓库请先看 [`docs/user_manual.md`](docs/user_manual.md)，逐文件理解请看 [`docs/repository_index.md`](docs/repository_index.md)，当前实测证据和明确边界以 [`docs/verification.md`](docs/verification.md) 为准。

当前实现相对原方案又完成了以下可靠性升级：

- 四个地图工件由严格 Manifest 和 bundle hash 绑定，`save_map.sh` 事务式保存、Manifest 最后提交；
- 前置 RGB Camera 支持 `off/monitoring/standard/high_quality` profile、Image/CameraInfo、TF、Camera-only 与集成 RViz；
- MPPI 真正局部轨迹使用 `/optimal_trajectory`，控制器参数在创建 ROS 节点前校验时间离散约束；
- `/scan_fault` 提供丢包、暂停和错误 Frame 注入，Reset 会恢复正常模式并隔离旧代次命令；
- Runtime Profiler 统计受管 ROS 进程树以及 RTF、Topic Age、TF Lag、CPU/GPU；
- ROS 监督脚本按 Lifecycle 顺序关闭 Nav2/定位节点，RViz 使用安全退出面板。
- Jackal robot YAML 已升级为 schema v2 单一运动学真源，显式区分几何/有效轮距并
  统一 wheel joint；runtime provenance v5 与 Realistic Wheel Odom 启动握手会锁定
  文件路径、原始字节 SHA256、profile/lifecycle 和运动学数值，稳定 profile 保持原
  `0.37559 m` 控制行为，尚不代表有效轮距已经标定。
- SimplePlane 1-collider、Warehouse 32-collider 与 Warehouse plane-only 1-collider
  已成为版本化、可逆的 ground-topology profile；schema v5 还锁定源资产、匿名
  overlay、source/target/disabled 精确集合并要求 contact ground target 一致。USD
  应用/读回和离线分组合同已验证；严格矩阵入口可按三个合法 pair 生成 54-run/18-group
  全拓扑批次。RootLayer 锁作用域修复后的 clean `d5840ed` 已完成 Warehouse 32-vs-1、
  六 contact profile、每格一次的 12-run 机制烟测：12/12 run、72/72 段和 144/144
  路径/hash 对均闭合，Root SHA 分布为 combined32 六份、plane-only legacy/explicit/
  两个 `0.00025` profile 四份、plane-only 两个 `0.025` profile 两份。该历史批次保存
  analysis schema 2、batch-summary schema 3；Kit `[Error]` 为 0，但 12 份 Isaac 日志
  各有一条非致命 absl `E0000`，因此不能写成“零错误”。正式每组三重复的
  54-run/18-group 全拓扑批次仍未执行。
- 导航质量升级计划 8.7 的 v2 机器硬门已实现：新 motion report 顶层为 schema 3，
  `configuration.schema_version` 仍为 1。Odom 与 JointState 都保存命令后半段 closed
  window，至少两个严格递增样本，并对首尾覆盖、最大间隔、deadband、方向样本计数和
  分布执行 fail-closed 复核；稳态角速度分布还必须能由整段 Odom 分布的真实样本子集
  实现。每段停车证据要求 Odom/JointState 在同一命令后区间内持续静止、样本新鲜且总
  count 足以同时覆盖命令窗和停车窗；Reset recovery watermark 必须先于命令和段内首
  样本。起止 pose 会重算净位移、纵横向位移、轨迹下界和旋转漂移，端点 yaw 与累计
  yaw 也须按 `2*pi` 自洽。整段轮向只作描述，稳态 mixed/stationary/opposite 会作为
  valid evidence 纳入并使物理方向门失败。v5 analysis/physical/summary 分别为 schema
  4/2/5，policy 为 `skid_steer_plan_8_7_v2`，manifest 为 44 列并锁定逐轮 report schema。
  机器判定只适用于 runtime provenance 5 + `SimplePlane` +
  `simple_plane_only1_v1` + Ideal + 每组至少 3 个唯一 repeat + motion report schema 3；
  旧 schema 1/2 以 `motion_report_schema_not_3` 记 N/A。公共 accounting 会重新读取
  selection 原报告、复核 raw/canonical SHA、全局身份和每个 physical 叶并重算完整
  acceptance。批次 summary 还会逐行语义核对 44 列、报告路径/时间、规范化 motion YAML
  （类型严格）、robot asset/kinematics/solver、Mapping/Ideal/60 Hz、环境、topology、
  contact、Git 和三类证据 hash，不能靠协调改写 schema、N/A、方向叶或运行身份伪造
  verdict。当前定向套件为 analyzer `217 passed`、motion baseline `92 passed`、matrix
  `45 passed / 1 skipped`，合并 `354 passed / 1 skipped`（唯一 skip 为缺少
  `shellcheck`）。当前 v2 合同已在 clean `0484b72` 上完成三条全门：build 11 packages，
  preflight PASS，`./scripts/test.sh --with-isaac` exit 0；root
  `1206 passed / 1 skipped / 34 deselected`，ROS 11 packages / 1006 tests / 0 errors /
  0 failures / 1 skipped，Isaac `32 passed / 250 deselected`；预检另有 422 个 Fast DDS
  SHM 工件和 20 个非 performance governor 的非阻塞环境警告。schema-3 真实 smoke
  仍待执行。此前在 clean `190f357` 完成
  SimplePlane/only1 × 六 profile × 一次的历史 schema-2 smoke：6/6 run、36/36 段、
  72/72 Manifest path/hash 配对（144 个叶检查）闭合；motion/analysis/summary 分别为
  schema 2/3/4，六组都只因少于 3 个 repeat 而 N/A。正式 54-run/18-group 实跑仍未执行。
- `0.989/1.012 m` 已分别保存为不可原地修改的 `experimental_candidate` v1 文件，并在
  clean `8d1c5f4` / `05fdba7` 各采集过一份 SimplePlane + legacy 原始筛选。旧 schema 2
  因圆弧整段 `mixed` 被验证器排除，不能形成正式 verdict；这正是上面 schema-3 稳态
  方向合同的触发证据。描述性指标中 `0.989 m` 的左右稳态 yaw 误差为
  `2.95%/0.02%`、中心漂移 `0.0530/0.0587 m`、不对称 `9.70%`，明显优于 `1.012 m`
  的不对称 `43.67%`，因此是 schema-3 重跑首选。两者都没有覆盖 stable，也尚未完成
  三重复、两环境、多速度、全拓扑或 Realistic 物理 A/B。

原方案十三个阶段的当前状态如下。“已实现”表示代码和契约存在，“实机/仿真证据”只写本仓库已经实际运行的范围；计划中的广义统计门槛仍须独立完成。

| 原阶段 | 当前实现状态 | 已有实机/仿真证据 | 仍未覆盖的边界 |
| --- | --- | --- | --- |
| 1 Jackal 物理底盘 | 已实现 | 官方 Warehouse + 项目 Jackal 可启动，唯一 PhysicsScene、固定出生点与 Reset 已运行 | 更换场景后的全资产复验 |
| 2 `/cmd_vel` 控制 | 已实现 | Teleop、Nav2、停车与 Reset 路径已运行 | 不同地面材料和载荷的系统辨识 |
| 3 `/clock` | 已实现，RTX helper 时间策略已对齐供应商默认 | 15 分钟 headless soak 中 `/clock` 51130 样本无重复/回退，三类 Kit 时间样本警告均为 0；Camera Monitoring 短窗同样为 0 | 真正 Timeline Stop→Play 与 GUI/headless × realtime/unbounded × 60/120 Hz 完整矩阵 |
| 4 TF 与 Ideal Odom | 已实现 | Ideal 导航、Reset、TF freshness 已运行 | 计划中的全部长时统计矩阵 |
| 5 LiDAR 与 `/scan` | 已实现 | 正常扫描及丢包/暂停/错误 Frame 故障矩阵已运行 | 更广传感器噪声与遮挡矩阵 |
| 6 SLAM/Localization | 已实现 | `warehouse_v1` 建图工件、Localization、Manifest 校验可用 | 真实变化场景的 `warehouse_v2` 尚未制作 |
| 7 Nav2 | 已实现 | 1 m/3 m smoke、MPPI 10/15 Hz 参数矩阵和真实局部轨迹已运行 | 多终点、多布局的完整统计 |
| 8 Ground Truth | 已实现 | smoke 报告已记录终点误差与 GT 路径 | 200 次统计验收 |
| 9 Realistic Odom | 已实现；运动学配置已收敛到 schema-v2 robot YAML，启动前做 provenance v5 失败关闭握手 | 已有 Realistic 静态 smoke，`/odom` 唯一发布者已检查；握手匹配、SHA 错配和服务超时由真实 rclpy 集成测试覆盖 | 新契约仍需在冻结候选上做完整 Realistic 物理复验及更复杂滑移/噪声矩阵 |
| 10 动态避障 | 已实现基线 | 当前固定世界的 4-seed 基线为 4/4 | 不能外推为多类障碍 90% 广义避障率 |
| 11 自动实验 | 已实现框架 | Reset、场景契约和 smoke 批次可重复运行 | 完整 200 次矩阵未执行 |
| 12 增量地图 | 工作流与比较器已实现 | Manifest、未标定 `auto` 拒绝及 `rviz` 路径由临时夹具验证 | 没有真实 `warehouse_v2`，未证明 changed-region 时间改善 ≥30% |
| 13 自定义机器人 | 仅迁移模板 | 配置入口和 fail-fast 契约有自动测试 | 没有真实自定义 USD，不能声称完成迁移 |

Camera 的 `monitoring` 与 `high_quality` 已完成 headless 发布采样；前向画面、非镜像/非倒置、无大面积自遮挡和转弯后的视角变化已通过实际截图目视确认，集成 RViz 已实际运行。`standard` profile 尚无独立实机性能报告，完整 GUI 人因/布局验收也没有用一次启动替代。详细矩阵、数字和限制见 [`docs/runtime_reliability_and_performance_upgrade_plan.md`](docs/runtime_reliability_and_performance_upgrade_plan.md) 与 [`docs/verification.md`](docs/verification.md)。

---

# 第一部分：需求理解和关键约束

## 1.1 系统目标

在 Isaac Sim 6.0.1 中，以 Clearpath Jackal 四轮滑移转向机器人为原型，搭建一套 ROS 2 Jazzy 室内移动机器人导航仿真系统，支持：

1. 使用 Isaac Sim 官方仓库环境；
2. 使用项目自有 Jackal USD；
3. 使用 3D RTX LiDAR 生成二维 `/scan`；
4. 使用 SLAM Toolbox 完成二维建图；
5. 使用 SLAM Toolbox Localization 完成已知地图定位；
6. 使用 Nav2 完成二维全局规划、局部控制和避障；
7. 支持理想里程计和真实感里程计两种模式；
8. 使用 Wheel Odometry 与 IMU 进行 EKF 融合；
9. 独立发布 Ground Truth，用于误差评价；
10. 固定并可重复设置机器人出生位置；
11. 支持自动重复实验和统计；
12. 后续迁移到自定义四轮滑移转向机器人；
13. 保留向真实机器人迁移的 ROS 接口和 TF 架构。

## 1.2 本次修订后的核心技术选择

| 模块                  | 最终选择                                   |
| ------------------- | -------------------------------------- |
| 仿真平台                | Isaac Sim 6.0.1                        |
| ROS 2               | Jazzy                                  |
| DDS                 | Fast DDS                               |
| 主场景                 | `warehouse_multiple_shelves.usd`       |
| 场景组合方式              | 官方完整场景作为 Sublayer                      |
| 机器人组合方式             | 项目 Jackal 主资产作为 Reference              |
| 项目主 Stage           | `navigation_scene.usda`                |
| PhysicsScene        | 复用官方场景中的唯一 `/PhysicsScene`             |
| 仿真启动                | standalone Python，GUI 模式               |
| 主传感器                | 3D RTX LiDAR                           |
| 当前导航维度              | 二维 SLAM、二维 Nav2                        |
| 点云处理                | 直接 `PointCloud2 → LaserScan`           |
| 自车过滤                | 按实际点云结果决定，默认关闭                         |
| Range/Height Filter | 不单独创建，由投影节点完成                          |
| VoxelGrid           | 当前不使用                                  |
| Nav2 Voxel Layer    | 当前不使用，作为后续扩展                           |
| 建图                  | SLAM Toolbox Async Mapping             |
| 定位                  | SLAM Toolbox Localization              |
| 全局规划                | SmacPlanner2D                          |
| 局部控制                | MPPI                                   |
| 全局代价地图              | Static + Obstacle(`/scan`) + Inflation |
| 局部代价地图              | Obstacle(`/scan`) + Inflation          |
| 命令平滑                | Nav2 Velocity Smoother                 |
| 安全停车                | Collision Monitor                      |
| 自定义 OGN Watchdog    | 第一版删除                                  |
| 真值                  | 独立 Ground Truth Topic                  |
| 出生点                 | 同时保存 USD Pose 与 Map Pose               |

## 1.3 关键约束

### USD 是仿真物理主模型

USD/PhysX 负责：

* 刚体；
* Joint；
* Articulation；
* 碰撞；
* 质量与惯量；
* 轮地接触；
* 传感器安装位置；
* 场景结构；
* 仿真真值。

URDF/Xacro 负责：

* ROS 机器人描述；
* RViz RobotModel；
* `robot_state_publisher`；
* 真机迁移。

不得同时让 USD 和 URDF 驱动 Isaac Sim 中同一套物理结构。

### 场景与机器人采用不同组合方式

```text
官方完整环境 Stage → Sublayer
独立机器人资产     → Reference
```

原因：

* 环境文件中除了 `/Root`，还有同级的 `/PhysicsScene`、`/NavMesh`；
* Sublayer 可以保留整个环境 Layer；
* Jackal 是独立可实例化资产，适合挂载到指定 Prim；
* `configuration/*.usd` 是配置层，不是完整机器人入口，不得直接作为 Reference。

### 整个 Stage 只能有一个 PhysicsScene

当前仓库环境已经包含：

```text
/PhysicsScene
```

因此：

* 不创建 `/World/physicsScene`；
* standalone Python 只检查、复用和按需校验已有 PhysicsScene；
* 若没有 PhysicsScene，再创建；
* 若发现多个 PhysicsScene，立即报错。

### 不手工维护最终 OmniGraph

GUI 用于：

* 检查 Prim；
* 验证传感器方向；
* 调试节点；
* 选择出生点；
* 检查 PhysicsScene。

正式工程由 standalone Python 创建和配置 OmniGraph。

### ROS Frame 与 USD Prim Path 不等价

```text
USD Prim:
/World/Robots/Jackal/base_link

ROS Frame:
base_link
```

名称相同不代表自动建立对应关系。

### ROS 主 TF 树

```text
map → odom → base_link
```

不发布：

```text
world → map
world → base_link
```

USD 的 `/World` 只是 Stage Prim 路径，不是 ROS Frame。

### Ground Truth 不进入导航

Ground Truth 只能用于：

* 指标评价；
* 轨迹对比；
* RViz 显示；
* rosbag；
* CSV/JSON 报告。

不得输入：

* SLAM Toolbox；
* EKF；
* Nav2；
* Wheel Odometry；
* 控制器。

## 1.4 容易混淆的概念

| 概念                        | 正确理解                               |
| ------------------------- | ---------------------------------- |
| `.usda`                   | 可读文本格式的 USD，便于开发和 Git Diff，不是强制格式  |
| Layer                     | 保存 USD 描述的一个文件或数据层                 |
| Sublayer                  | 将另一 Layer 的完整根命名空间加入当前 Layer Stack |
| Reference                 | 将某个资产 Prim 子树挂载到当前指定 Prim          |
| Stage                     | 多个 Layer 组合后的最终场景                  |
| `defaultPrim`             | Reference 未指定源 Prim 时使用的默认入口       |
| PhysicsScene              | PhysX 的场景级仿真参数，一个 Stage 原则上只保留一个   |
| standalone Python         | 仿真生命周期和配置编排层                       |
| OmniGraph                 | Isaac Sim 内部实时数据流                  |
| ROS 2 Bridge              | Isaac 数据与 ROS 消息之间的转换层             |
| 3D LiDAR                  | 发布 `PointCloud2`，不等于 `/scan`       |
| `pointcloud_to_laserscan` | 从指定高度范围的 3D 点生成二维 LaserScan        |
| VoxelGrid                 | 三维点云降采样滤波器                         |
| Voxel Layer               | 内部维护三维体素、最终写入二维 Nav2 Costmap       |
| Obstacle Layer            | 使用 LaserScan 或 PointCloud 标记二维障碍   |
| `/odom`                   | 里程计 Topic                          |
| `odom → base_link`        | 连续局部位姿 TF                          |
| `map → odom`              | 由建图或定位节点发布的全局修正 TF                 |
| USD Pose                  | 机器人在 Isaac Stage 中的物理位置            |
| Map Pose                  | 同一地点在 ROS 已保存地图中的位置                |
| Ground Truth              | 仿真真值，不是导航状态估计                      |

---

# 第二部分：推荐总体架构

## 2.1 USD Layer 与 Prim 组合结构

```text
Layer Stack
────────────────────────────────────────────

navigation_scene.usda                项目根 Layer，当前 Edit Target
└── Sublayer
    └── warehouse_multiple_shelves.usd
        ├── /Root
        ├── /PhysicsScene
        └── /NavMesh
```

项目根 Layer 中新增：

```text
/
├── Root                              来自环境 Sublayer
├── PhysicsScene                      来自环境 Sublayer
├── NavMesh                           来自环境 Sublayer
└── World                             来自项目根 Layer
    ├── Robots
    │   └── Jackal                    Reference → jackal_nav.usda
    ├── Graphs
    ├── DynamicObstacles
    └── ExperimentMarkers
```

## 2.2 分层架构图

```text
┌──────────────────────────────────────────────────────────────┐
│                     用户与实验管理层                          │
│ RViz2 │ Nav2 Goal │ Experiment Manager │ Metrics │ rosbag    │
└──────────────────────────────▲───────────────────────────────┘
                               │
┌──────────────────────────────┴───────────────────────────────┐
│                     ROS 2 导航算法层                          │
│ SLAM Toolbox │ robot_localization │ SmacPlanner2D │ MPPI     │
│ Velocity Smoother │ Collision Monitor                        │
└──────────────────────────────▲───────────────────────────────┘
                               │
┌──────────────────────────────┴───────────────────────────────┐
│                     ROS 2 感知处理层                          │
│ pointcloud_to_laserscan                                      │
│ 可选 CropBox Self Filter                                     │
└──────────────────────────────▲───────────────────────────────┘
                               │ ROS Topic / TF / DDS
┌──────────────────────────────┴───────────────────────────────┐
│                     Fast DDS 通信层                           │
│ ROS_DOMAIN_ID │ Discovery │ QoS │ Serialization               │
└──────────────────────────────▲───────────────────────────────┘
                               │
┌──────────────────────────────┴───────────────────────────────┐
│                 Isaac Sim ROS 2 Bridge 层                     │
│ Clock │ PointCloud │ IMU │ JointState │ Odom │ TF │ Twist    │
└──────────────────────────────▲───────────────────────────────┘
                               │ OmniGraph
┌──────────────────────────────┴───────────────────────────────┐
│                   standalone Python 编排层                   │
│ StageComposer │ PhysicsSetup │ SpawnPoseManager               │
│ SensorFactory │ RosGraphBuilder │ ResetManager                │
│ GroundTruthRecorder │ ExperimentManager                       │
└──────────────────────────────▲───────────────────────────────┘
                               │ USD / PhysX
┌──────────────────────────────┴───────────────────────────────┐
│                     USD + PhysX 物理层                        │
│ Warehouse │ PhysicsScene │ Jackal │ Wheels │ RTX LiDAR        │
│ IMU │ Camera │ Dynamic Obstacles                              │
└──────────────────────────────────────────────────────────────┘
```

## 2.3 控制数据流

```text
Nav2 MPPI
   │
   ▼
/cmd_vel_nav
   │
   ▼
Velocity Smoother
   │
   ▼
/cmd_vel_smoothed
   │
   ▼
Collision Monitor
   │
   ▼
/cmd_vel
   │
   ▼
ROS2SubscribeTwist
   │
   ├── linear.x
   └── angular.z
   │
   ▼
DifferentialController
   │
   ├── ω_left
   └── ω_right
   │
   ├───────────────────────────┐
   ▼                           ▼
Front Controller         Rear Controller
   │                           │
前左、前右                 后左、后右
   └────────── PhysX ──────────┘
```

第一版不插入自定义 OGN Watchdog。

`/cmd_vel` 超时由：

```text
Velocity Smoother.velocity_timeout
Collision Monitor.source_timeout
```

处理。低层 Watchdog 作为真机或高安全等级扩展保留。

## 2.4 LiDAR 数据流

当前二维导航基线：

```text
RTX LiDAR
   │
   ▼
/lidar/points_raw
   │
   ├── 无自车点：直接输入
   │
   └── 有自车点：可选 CropBox Self Filter
   │
   ▼
pointcloud_to_laserscan
   │
   ▼
/scan
   ├── SLAM Toolbox
   ├── Global Costmap Obstacle Layer
   ├── Local Costmap Obstacle Layer
   └── Collision Monitor
```

不再建立：

```text
/lidar/points_navigation
/lidar/points_projection
VoxelGrid
Voxel Layer
```

## 2.5 Mapping 数据流

```text
固定 USD 出生点
   │
   ├── 重置机器人状态
   └── 重置 odom
         │
         ▼
/scan + /odom + TF
         │
         ▼
SLAM Toolbox Mapping
         │
         ├── /map
         ├── map → odom
         ├── OccupancyGrid
         └── Serialized Pose Graph
```

Mapping 模式不发布 `/initialpose`。

## 2.6 Localization 数据流

```text
固定 USD 出生点
        │
        ├── Isaac 使用 usd_pose
        └── ROS 发布对应 map_pose
                     │
                     ▼
                /initialpose
                     │
Serialized Pose Graph + /scan + /odom + TF
                     │
                     ▼
           SLAM Toolbox Localization
                     │
                     └── map → odom
```

## 2.7 理想里程计

```text
Isaac Sim 机器人真值状态
          │
          ▼
Isaac Compute Odometry
          │
          ├── /odom
          └── odom → base_link
```

## 2.8 真实感里程计

```text
四轮 Joint State
        │
        ▼
Wheel Odometry
        │
        └── /wheel/odom ────────┐
                                │
/imu/data ──────────────────────┤
                                ▼
                     robot_localization EKF
                                │
                                ├── /odom
                                └── odom → base_link
```

## 2.9 Ground Truth

```text
USD 世界中的真实位姿
        │
        ▼
map_T_usd 坐标对齐
        │
        ▼
GroundTruthRecorder
        │
        ├── /ground_truth/odom
        ├── /ground_truth/path
        ├── CSV/JSON
        └── Metrics
```

---

# 第三部分：Isaac Sim、standalone Python、OmniGraph、ROS 2 Bridge、ROS 2 的职责边界

## 3.1 Isaac Sim USD/PhysX

负责：

* 环境和机器人几何；
* Articulation；
* Joint；
* 碰撞；
* 质量和惯量；
* 摩擦；
* 轮地接触与滑移；
* 传感器安装；
* 动态障碍物；
* Ground Truth。

不负责：

* SLAM；
* 定位算法；
* Nav2；
* 地图保存；
* 全局规划；
* `/initialpose` 语义。

## 3.2 standalone Python

负责：

* 启动 `SimulationApp`；
* 创建或打开项目主 Stage；
* 将官方环境加入 Sublayer；
* 将项目 Jackal 加入 Reference；
* 校验唯一 PhysicsScene；
* 设置仿真 DT；
* 设置固定出生点；
* 创建传感器；
* 创建 OmniGraph；
* 按模式启停 Odom/TF；
* 实验 Reset；
* Ground Truth；
* 动态障碍物；
* 日志和实验生命周期。

不负责：

* 重新实现 DDS；
* 替代 ROS 2 节点；
* 替代 SLAM Toolbox；
* 通过不断修改 Pose 实现导航。

## 3.3 OmniGraph

负责：

* Playback Tick；
* ROS 发布和订阅执行流；
* `/cmd_vel` 到轮 Joint 目标；
* `/clock`；
* PointCloud；
* IMU；
* JointState；
* 理想 Odom；
* TF。

不负责：

* 点云高级算法；
* SLAM；
* 路径规划；
* 实验统计。

## 3.4 ROS 2 Bridge

负责：

* Isaac 数据与 ROS 消息转换；
* Publisher/Subscriber；
* DDS；
* Domain ID；
* QoS；
* ROS 时间戳。

## 3.5 ROS 2 感知处理

当前只负责：

* `PointCloud2 → LaserScan`；
* 高度范围筛选；
* 距离范围筛选；
* 可选自车 CropBox；
* TF 转换；
* QoS 适配。

不再默认负责：

* VoxelGrid；
* 两路点云分支；
* Nav2 Voxel Layer 输入。

## 3.6 SLAM Toolbox

负责：

* 二维扫描匹配；
* Mapping；
* Localization；
* Pose Graph；
* `/map`；
* `map → odom`；
* 地图序列化；
* 增量更新。

不负责：

* `odom → base_link`；
* Wheel Odometry；
* IMU 融合。

## 3.7 robot_localization

负责：

* Wheel Odom 与 IMU 融合；
* `/odom`；
* `odom → base_link`；
* 协方差传播；
* 平面运动约束。

## 3.8 Nav2

负责：

* Global Costmap；
* Local Costmap；
* 全局规划；
* 局部轨迹控制；
* 静态和反应式动态避障；
* 命令平滑；
* Collision Monitor；
* Behavior Tree；
* Goal 管理。

## 3.9 职责边界表

| 层                 | 输入                 | 输出                    | 主要职责    |
| ----------------- | ------------------ | --------------------- | ------- |
| USD/PhysX         | Joint Target、Stage | 物理状态                  | 仿真物理    |
| standalone Python | YAML、USD           | 仿真应用                  | 生命周期与编排 |
| OmniGraph         | Tick、Isaac/ROS 数据  | 实时数据流                 | 控制与传感器图 |
| ROS 2 Bridge      | Isaac 数据           | ROS Topic/TF          | 消息转换    |
| Perception        | PointCloud2        | LaserScan             | 二维投影    |
| SLAM Toolbox      | Scan、Odom、TF       | Map、`map→odom`        | 建图与定位   |
| EKF               | Wheel Odom、IMU     | Odom、`odom→base_link` | 局部状态估计  |
| Nav2              | Map、Scan、TF        | cmd_vel               | 二维导航    |
| Ground Truth      | USD 真值             | GT Topic、Metrics      | 评价      |

---

# 第四部分：机器人 USD 设计

## 4.1 官方资产与项目资产

不得直接修改：

```text
/home/lyb/isaacsim_assets/Assets/Isaac/6.0/Isaac/Robots/Clearpath/Jackal/
```

项目资产：

```text
$PROJECT_ROOT/isaac_sim/assets/robots/jackal/
├── source/
│   └── jackal_original.usda
├── configuration/
│   └── jackal_robot_schema.usd
├── jackal_nav.usda
└── README.md
```

应复制官方 Jackal 目录中必要的完整依赖，不能只复制主文件。

`configuration/*.usd` 由 Jackal 主资产内部 Sublayer/Reference 使用，不能直接拖入环境 Stage。直接引用无 `defaultPrim` 的 Schema 文件会失败。

## 4.2 项目主场景与机器人关系

```text
navigation_scene.usda
├── Sublayer → warehouse_multiple_shelves.usd
└── /World/Robots/Jackal
      Reference → jackal_nav.usda:/jackal
```

运行时路径：

```yaml
robot_prim: /World/Robots/Jackal
articulation_root: /World/Robots/Jackal
base_link_prim: /World/Robots/Jackal/base_link
```

## 4.3 Jackal 已确认参数

```yaml
wheel_radius: 0.098
wheel_width: 0.040
geometric_track_width: 0.37559
wheelbase: 0.262

base_mass: 17.0
wheel_mass: 0.477
nominal_total_mass: 18.908
```

四轮 Joint：

```text
front_left_wheel_joint
front_right_wheel_joint
rear_left_wheel_joint
rear_right_wheel_joint
```

这些参数与原方案从 Jackal USDA 中确认的内容一致。

## 4.4 当前 Jackal 修改方案

### LiDAR

原结构：

```text
base_link
└── sick_lms1xx_lidar_frame
    └── Lidar
```

目标：

```text
base_link
└── lidar_link
    └── rtx_lidar
```

操作：

1. 删除旧 Sensor Prim；
2. 保留固定安装 Xform；
3. 重命名为 `lidar_link`；
4. 保留安装位姿；
5. 由 Python 创建 RTX LiDAR；
6. Sensor Prim 不设置为独立刚体；
7. 外壳碰撞可保留。

原始安装位置为：

```yaml
translation: [0.120, 0.000, 0.333]
```

### IMU

原结构：

```text
base_link
└── com_frame
    └── imu_sensor
```

目标：

```text
base_link
└── imu_link
    └── imu_sensor
```

原始安装位置：

```yaml
translation: [0.012, 0.002, 0.067]
```

以上传感器安装参数来自当前 Jackal 资产分析。

### 相机

历史设计以双目 Frame 为主；当前交付把 `front` 定义为唯一默认发布的机器人第一视角，同时保留未启用的左右 Frame 供后续双目扩展。实际结构为：

```text
camera_link
├── camera_front_link
│   └── camera_front_optical_frame
│       └── camera_front_sensor   # 当前唯一默认启用的 Camera
├── camera_left_link
│   └── camera_left_optical_frame
└── camera_right_link
    └── camera_right_optical_frame
```

ROS 普通 Frame：

```text
x 前
y 左
z 上
```

Optical Frame：

```text
x 右
y 下
z 前
```

原双目 Frame 的基线仍为 `0.120 m`，但当前没有把左右 Camera 当作已实现的 Stereo/Depth 发布链。前置 Camera 的运行时契约是 `/camera/front/image_raw`、`/camera/front/camera_info` 和 `camera_front_optical_frame`。

## 4.5 修改后的机器人结构

```text
/World/Robots/Jackal
├── base_link
│   ├── visuals
│   ├── collisions
│   ├── lidar_link
│   │   └── rtx_lidar
│   ├── imu_link
│   │   └── imu_sensor
│   └── camera_link
│       ├── camera_front_link
│       │   └── camera_front_optical_frame
│       │       └── camera_front_sensor
│       ├── camera_left_link
│       │   └── camera_left_optical_frame
│       └── camera_right_link
│           └── camera_right_optical_frame
├── front_left_wheel_link
├── front_right_wheel_link
├── rear_left_wheel_link
├── rear_right_wheel_link
├── front_left_wheel_joint
├── front_right_wheel_joint
├── rear_left_wheel_joint
└── rear_right_wheel_joint
```

## 4.6 轮子方向验证

逐轮输入：

```text
+1 rad/s
```

目标：

```text
所有轮 Joint 正速度均使接触点产生机器人 +X 前进趋势
```

若方向不一致，优先修正 Joint Frame，不在 ROS 层长期添加符号补偿。

## 4.7 固定出生点

机器人场景位姿设置在：

```text
/World/Robots/Jackal
```

而不是内部 `base_link`。

配置：

```yaml
spawn_poses:
  mapping_start:
    usd:
      position: [X_USD, Y_USD, Z_USD]
      yaw_deg: YAW_USD

    map:
      position: [X_MAP, Y_MAP]
      yaw_deg: YAW_MAP
```

Mapping 时：

* 使用 `usd`；
* 重置 Odom；
* 不发布 `/initialpose`。

Localization 时：

* 使用同一 `usd`；
* 发布对应 `map` 到 `/initialpose`。

## 4.8 Footprint

初始保守值：

```yaml
footprint:
  - [ 0.255,  0.210]
  - [ 0.255, -0.210]
  - [-0.230, -0.210]
  - [-0.230,  0.210]
```

该 Footprint 来自当前 Jackal 底盘及外壳范围估计。

## 4.9 最终自定义机器人规范

必须具备：

* 唯一 Articulation Root；
* 米制单位；
* Z-up；
* 正确刚体层级；
* 四个轮 Link；
* 四个 Revolute Joint；
* 正确 Body0/Body1；
* 统一 Joint 正方向；
* 质量、质心和惯量；
* Collision；
* 轮胎 Physics Material；
* 有限 Drive 参数；
* `base_link`；
* `lidar_link`；
* `imu_link`；
* 相机机械 Frame 与 Optical Frame；
* 固定传感器不成为独立刚体；
* 明确的 `defaultPrim`。

---

# 第五部分：传感器方案比较与最终推荐

## 5.1 方案比较

| 方案 | 配置                               | 优点                      | 缺点              | 适用阶段     |
| -- | -------------------------------- | ----------------------- | --------------- | -------- |
| A  | 2D LiDAR + IMU + Wheel Odom      | 简单、计算量低                 | 后续扩展能力有限        | 最快跑通     |
| B  | 3D LiDAR 投影二维 + IMU + Wheel Odom | 保留 3D 硬件接口，当前仍可使用成熟二维导航 | 点云带宽高于 2D LiDAR | 推荐       |
| C  | 3D LiDAR + Voxel Layer + IMU     | 可利用高度信息                 | 参数和调试复杂         | 后续三维感知扩展 |
| D  | 3D LiDAR + RGB-D/双目 + IMU        | 感知丰富                    | 标定、同步、算力开销高     | 视觉研究阶段   |

## 5.2 最终推荐

采用方案 B：

```text
3D RTX LiDAR
+ PointCloud to LaserScan
+ IMU
+ Joint State
+ Wheel Odometry
```

但当前只进行：

```text
二维 SLAM
二维定位
二维 Costmap
二维规划与控制
```

## 5.3 点云处理链

基线：

```text
/lidar/points_raw
→ pointcloud_to_laserscan
→ /scan
```

可选自车过滤：

```text
/lidar/points_raw
→ CropBox Self Filter
→ /lidar/points_scan
→ pointcloud_to_laserscan
→ /scan
```

不单独添加：

```text
Range Filter
Height Filter
VoxelGrid
```

原因：

* Range 由 `range_min/range_max` 完成；
* Height 由 `min_height/max_height` 完成；
* VoxelGrid 对当前 10 Hz 二维投影不是必要条件；
* 避免过早删除最近障碍点。

## 5.4 投影高度范围

推荐：

```yaml
target_frame: base_link
min_height: 0.05
max_height: 0.50   # 初始值，需根据完整碰撞包络实测
```

`max_height` 应满足：

```text
机器人最高碰撞点 + 安全余量
```

不能直接使用 `1.80 m`，否则高处货架横梁可能被错误投影成地面障碍。

## 5.5 何时重新加入 Voxel Layer

出现以下需求时再加入：

* 需要区分低矮和悬空障碍；
* 需要在桌板或货架横梁下通行；
* 二维投影造成严重虚假封路；
* 真机局部避障必须直接消费 3D 点云；
* 需要评价不同高度障碍检测性能。

---

# 第六部分：Topic、Message、Frame、TF、QoS 和频率表

## 6.1 Topic 表

| Topic                | Message                     | Frame                            | 发布者               | 订阅者                            |     初始频率 |
| -------------------- | --------------------------- | -------------------------------- | ----------------- | ------------------------------ | -------: |
| `/clock`             | `rosgraph_msgs/Clock`       | —                                | Isaac Sim         | 所有仿真时间节点                       |    60 Hz |
| `/lidar/points_raw`  | `sensor_msgs/PointCloud2`   | `lidar_link`                     | Isaac Sim         | 投影节点/可选过滤器                     |    10 Hz |
| `/lidar/points_scan` | `PointCloud2`               | `base_link`或`lidar_link`         | 可选 Self Filter    | 投影节点                           |    10 Hz |
| `/scan`              | `sensor_msgs/LaserScan`     | `base_link`或`lidar_link`         | 投影节点              | SLAM、Costmap、Collision Monitor |    10 Hz |
| `/imu/data`          | `sensor_msgs/Imu`           | `imu_link`                       | Isaac Sim         | EKF、Recorder                   |    60 Hz |
| `/joint_states`      | `sensor_msgs/JointState`    | —                                | Isaac Sim         | Wheel Odom、RSP                 |    60 Hz |
| `/wheel/odom`        | `nav_msgs/Odometry`         | `odom` / `base_link`             | Wheel Odom        | EKF                            |    50 Hz |
| `/odom`              | `nav_msgs/Odometry`         | `odom` / `base_link`             | Isaac 或 EKF       | SLAM、Nav2                      |    50 Hz |
| `/tf`                | `tf2_msgs/TFMessage`        | —                                | 按所有权发布            | 全部 TF 用户                       | 30–60 Hz |
| `/tf_static`         | `tf2_msgs/TFMessage`        | —                                | Isaac 或 RSP       | 全部 TF 用户                       |      启动时 |
| `/map`               | `nav_msgs/OccupancyGrid`    | `map`                            | SLAM Toolbox      | Nav2、RViz                      |    地图更新时 |
| `/initialpose`       | `PoseWithCovarianceStamped` | `map`                            | RViz/实验节点         | Localization                   |       按需 |
| `/cmd_vel_nav`       | `geometry_msgs/Twist`       | —                                | Nav2 Controller   | Velocity Smoother              |    20 Hz |
| `/cmd_vel_smoothed`  | `Twist`                     | —                                | Velocity Smoother | Collision Monitor              | 20–50 Hz |
| `/cmd_vel`           | `Twist`                     | —                                | Collision Monitor | Isaac Sim                      | 20–50 Hz |
| `/ground_truth/odom` | `nav_msgs/Odometry`         | `map` / `ground_truth_base_link` | Isaac GT          | Metrics                        | 50–60 Hz |
| `/ground_truth/path` | `nav_msgs/Path`             | `map`                            | Isaac GT          | RViz、Metrics                   |  5–10 Hz |

## 6.2 Frame 表

| Frame                    | 父 Frame      | 类型       | 含义             |
| ------------------------ | ------------ | -------- | -------------- |
| `map`                    | 无            | 全局       | SLAM 地图        |
| `odom`                   | `map`        | 动态       | 连续局部坐标         |
| `base_link`              | `odom`       | 动态       | 机器人主体          |
| wheel links              | `base_link`  | Joint 动态 | 四个车轮           |
| `lidar_link`             | `base_link`  | 固定       | LiDAR 安装坐标     |
| `imu_link`               | `base_link`  | 固定       | IMU 安装坐标       |
| `camera_link`            | `base_link`  | 固定       | 相机安装坐标         |
| camera optical frames    | camera links | 固定       | ROS 光学坐标       |
| `ground_truth_base_link` | 不发布 TF       | 消息字段     | 真值 Child Frame |

## 6.3 TF 表

| Transform                   | 类型 | 唯一发布者                               |
| --------------------------- | -- | ----------------------------------- |
| `map → odom`                | 动态 | SLAM Toolbox Mapping 或 Localization |
| `odom → base_link`          | 动态 | Isaac 或 EKF，互斥                      |
| `base_link → wheel_link`    | 动态 | Isaac 或 RSP，互斥                      |
| `base_link → lidar_link`    | 静态 | Isaac 或 RSP                         |
| `base_link → imu_link`      | 静态 | Isaac 或 RSP                         |
| `base_link → camera_link`   | 静态 | Isaac 或 RSP                         |
| camera link → optical frame | 静态 | Isaac 或 RSP                         |

## 6.4 QoS 表

| 数据             | Reliability | Durability      | Depth |
| -------------- | ----------- | --------------- | ----: |
| `/clock`       | Best Effort | Volatile        |     1 |
| PointCloud2    | Best Effort | Volatile        |     5 |
| LaserScan      | Best Effort | Volatile        |     5 |
| IMU            | Best Effort | Volatile        |     5 |
| JointState     | Reliable    | Volatile        |    10 |
| Odometry       | Reliable    | Volatile        |    10 |
| `/cmd_vel`     | Reliable    | Volatile        |     1 |
| `/tf`          | Reliable    | Volatile        |   100 |
| `/tf_static`   | Reliable    | Transient Local |     1 |
| `/map`         | Reliable    | Transient Local |     1 |
| `/initialpose` | Reliable    | Volatile        |    10 |

## 6.5 频率表

| 模块                |       频率 |
| ----------------- | -------: |
| Physics           |    60 Hz |
| Rendering         |    60 Hz |
| `/clock`          |    60 Hz |
| LiDAR             |    10 Hz |
| `/scan`           |    10 Hz |
| IMU               |    60 Hz |
| JointState        |    60 Hz |
| Wheel Odom        |    50 Hz |
| EKF               |    50 Hz |
| Nav2 Controller   |    20 Hz |
| Ground Truth Odom | 50–60 Hz |

---

# 第七部分：TF 所有权和三阶段迁移方案

## 7.1 第一阶段：理想仿真模式

| TF                 | 发布者          |
| ------------------ | ------------ |
| `map → odom`       | SLAM Toolbox |
| `odom → base_link` | Isaac Sim    |
| wheel TF           | Isaac Sim    |
| sensor TF          | Isaac Sim    |
| optical TF         | Isaac Sim    |

关闭：

```text
robot_localization publish_tf
robot_state_publisher TF
Wheel Odom TF
```

## 7.2 第二阶段：真实感里程计模式

| TF                 | 发布者                |
| ------------------ | ------------------ |
| `map → odom`       | SLAM Toolbox       |
| `odom → base_link` | robot_localization |
| wheel TF           | Isaac Sim          |
| sensor TF          | Isaac Sim          |

关闭：

```text
Isaac 理想 /odom
Isaac odom → base_link
```

## 7.3 第三阶段：标准 ROS/真机迁移模式

| TF                 | 发布者                   |
| ------------------ | --------------------- |
| `map → odom`       | SLAM Toolbox          |
| `odom → base_link` | robot_localization    |
| wheel TF           | robot_state_publisher |
| sensor TF          | robot_state_publisher |
| optical TF         | robot_state_publisher |

Isaac 只发布：

```text
/clock
/joint_states
/lidar/points_raw
/imu/data
/camera/*
/ground_truth/*
```

Isaac 只订阅：

```text
/cmd_vel
```

三阶段 TF 所有权继承原方案的“每段 TF 只能有一个发布者”原则。

## 7.4 Mapping 与 Localization

两者都发布：

```text
map → odom
```

因此严格互斥。

## 7.5 固定出生点与 TF

Mapping：

```text
设置 USD Pose
→ 重置 Odom
→ 启动 Mapping
→ 不发布 /initialpose
```

Localization：

```text
设置同一 USD Pose
→ 重置 Odom
→ 启动 Localization
→ 发布对应 Map Pose
```

## 7.6 Map 与 USD 对齐

保存：

[
{}^{map}T_{usd}
===============

{}^{map}T_{base,start}
\left({}^{usd}T_{base,start}\right)^{-1}
]

Ground Truth：

[
{}^{map}T_{base}^{gt}
=====================

{}^{map}T_{usd}
{}^{usd}T_{base}^{gt}
]

---

# 第八部分：传统导航技术选型

## 8.1 感知

输入：

```text
3D PointCloud2
```

处理：

```text
pointcloud_to_laserscan
```

可选：

```text
CropBox Self Filter
```

输出：

```text
/scan
```

## 8.2 建图

选择：

```text
SLAM Toolbox Async Mapping
```

输入：

```text
/scan
/odom
odom → base_link
base_link → lidar_link
```

输出：

```text
/map
map → odom
OccupancyGrid
Serialized Pose Graph
```

必须同时保存 OccupancyGrid 和 Pose Graph。

## 8.3 定位

选择：

```text
SLAM Toolbox Localization
```

输入：

```text
Serialized Pose Graph
/scan
/odom
TF
/initialpose
```

固定出生点定位时，自动发布已标定的 Map Pose。

## 8.4 状态估计

理想模式：

```text
Isaac True Odom
```

真实模式：

```text
Wheel Odom + IMU → robot_localization
```

EKF：

```yaml
frequency: 50.0
two_d_mode: true
map_frame: map
odom_frame: odom
base_link_frame: base_link
world_frame: odom
publish_tf: true
```

## 8.5 全局规划

```text
SmacPlanner2D
```

适用于：

* 差速/滑移转向；
* 可近似原地旋转；
* 二维室内栅格；
* 无汽车式最小转弯半径约束。

## 8.6 Global Costmap

```text
Static Layer
Obstacle Layer：/scan
Inflation Layer
Keepout Filter：按需
Speed Filter：按需
```

## 8.7 Local Costmap

```text
Obstacle Layer：/scan
Inflation Layer
```

不启用 Voxel Layer。

Local Costmap 使用 rolling window。

## 8.8 局部控制

```text
MPPI Controller
```

初始参数：

```yaml
controller_frequency: 20.0
model_dt: 0.05
time_steps: 40
batch_size: 1500

vx_max: 1.0
vx_min: -0.20
wz_max: 1.5
```

模型选择 differential drive。

## 8.9 速度平滑

使用 Nav2 Velocity Smoother，负责：

* 最大线速度；
* 最大角速度；
* 加速度；
* 减速度；
* 命令插值；
* `velocity_timeout` 超时停车。

## 8.10 底盘控制

DifferentialController：

```yaml
wheel_radius: 0.098
wheel_distance: 0.37559
max_linear_speed: 1.0
max_angular_speed: 1.5
max_wheel_speed: 15.0
max_acceleration: 0.75
max_deceleration: 1.00
max_angular_acceleration: 2.00
```

输出：

```text
[ω_left, ω_right]
```

两个 ArticulationController：

```text
Front:
  front_left
  front_right

Rear:
  rear_left
  rear_right
```

## 8.11 Collision Monitor

数据源：

```text
/scan
```

配置：

* Stop Zone；
* Slowdown Zone；
* Approach Zone；
* `source_timeout`；
* 合理的 footprint/source frame。

## 8.12 动态避障

当前为二维反应式动态避障：

```text
动态障碍进入投影高度范围
→ /scan 实时更新
→ Local Obstacle Layer
→ MPPI 重新选择局部轨迹
→ Collision Monitor 最终保护
```

当前不能宣称：

* 三维路径规划；
* 悬空障碍高度理解；
* 不同高度可通行性推理。

## 8.13 增量地图更新

```text
加载旧 Pose Graph
→ 启动 Mapping Mode
→ 访问变化区域
→ 更新 Pose Graph 和 OccupancyGrid
→ 保存新版本
→ 关闭 Mapping
→ 启动 Localization
→ 导航验证
```

---

# 第九部分：完整工作空间结构

```text
$PROJECT_ROOT/
├── isaac_sim/
│   ├── apps/
│   │   └── navigation_sim.py
│   │
│   ├── assets/
│   │   ├── environments/
│   │   │   └── navigation_scene.usda
│   │   └── robots/
│   │       ├── jackal/
│   │       │   ├── source/
│   │       │   │   └── jackal_original.usda
│   │       │   ├── configuration/
│   │       │   ├── jackal_nav.usda
│   │       │   └── README.md
│   │       └── custom_robot/
│   │
│   ├── configs/
│   │   ├── project.yaml
│   │   ├── environments/
│   │   │   └── warehouse_multiple_shelves.yaml
│   │   ├── robots/
│   │   │   ├── jackal.yaml
│   │   │   └── custom_robot.yaml
│   │   ├── sensors/
│   │   │   ├── lidar_3d.yaml
│   │   │   ├── imu.yaml
│   │   │   └── camera.yaml
│   │   ├── simulation/
│   │   │   ├── ideal.yaml
│   │   │   └── realistic.yaml
│   │   ├── spawn_poses.yaml
│   │   ├── ros2_bridge/
│   │   │   ├── topics.yaml
│   │   │   └── qos.yaml
│   │   └── experiments/
│   │       ├── static.yaml
│   │       ├── dynamic.yaml
│   │       └── incremental_mapping.yaml
│   │
│   ├── graphs/
│   │   ├── control_graph.py
│   │   ├── sensor_graph.py
│   │   ├── odometry_graph.py
│   │   └── tf_graph.py
│   │
│   ├── src/
│   │   ├── stage/
│   │   │   ├── scene_composer.py
│   │   │   ├── stage_loader.py
│   │   │   ├── physics_setup.py
│   │   │   └── asset_validator.py
│   │   ├── robot/
│   │   │   ├── articulation_runtime.py
│   │   │   ├── joint_validator.py
│   │   │   ├── spawn_pose_manager.py
│   │   │   └── reset.py
│   │   ├── sensors/
│   │   ├── bridge/
│   │   ├── ground_truth/
│   │   └── experiment/
│   │
│   └── tests/
│       ├── test_stage_composition.py
│       ├── test_single_physics_scene.py
│       ├── test_asset_paths.py
│       ├── test_joint_mapping.py
│       ├── test_wheel_direction.py
│       ├── test_spawn_pose_reset.py
│       ├── test_control_graph.py
│       ├── test_tf_ownership.py
│       └── test_scan_projection.py
│
├── ros2_ws/
│   └── src/
│       ├── robot_description/
│       ├── robot_bringup/
│       ├── robot_perception/
│       │   ├── config/
│       │   │   ├── pointcloud_to_laserscan.yaml
│       │   │   └── self_filter_optional.yaml
│       │   └── launch/
│       │       └── lidar_processing.launch.py
│       ├── robot_mapping/
│       ├── robot_localization_config/
│       ├── robot_navigation/
│       ├── robot_experiments/
│       │   ├── experiment_runner.py
│       │   ├── initial_pose_publisher.py
│       │   ├── metrics.py
│       │   └── report.py
│       └── robot_interfaces/
│
├── data/
│   ├── maps/
│   │   ├── occupancy/
│   │   └── posegraphs/
│   ├── bags/
│   ├── trajectories/
│   ├── metrics/
│   ├── reports/
│   └── experiment_runs/
│
├── scripts/
├── docs/
├── .vscode/
├── .gitignore
├── pyproject.toml
└── README.md
```

删除原结构中的：

```text
graphs/nodes/cmd_vel_watchdog/
test_watchdog.py
pointcloud_filter.yaml
```

除非后续重新启用低层安全 Watchdog 或完整点云预处理。

---

# 第十部分：standalone Python 和 ROS 2 关键代码骨架

## 10.1 项目配置

```yaml
environment:
  project_stage:
    $PROJECT_ROOT/isaac_sim/assets/environments/navigation_scene.usda

  source_asset:
    /home/lyb/isaacsim_assets/Assets/Isaac/6.0/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd

  composition: sublayer

robot:
  asset_path:
    $PROJECT_ROOT/isaac_sim/assets/robots/jackal/jackal_nav.usda

  runtime_prim_path: /World/Robots/Jackal
  articulation_root: /World/Robots/Jackal
  base_link_prim: /World/Robots/Jackal/base_link

simulation:
  physics_hz: 60
  rendering_hz: 60
  expected_physics_scene: /PhysicsScene

spawn:
  selected: mapping_start
```

## 10.2 SceneComposer

```python
class SceneComposer:
    def __init__(self, config):
        self.config = config

    def compose(self):
        stage = create_or_open_project_stage(
            self.config.environment.project_stage
        )

        root_layer = stage.GetRootLayer()

        ensure_sublayer(
            root_layer=root_layer,
            layer_path=self.config.environment.source_asset,
        )

        ensure_xform(stage, "/World")
        ensure_xform(stage, "/World/Robots")
        ensure_xform(stage, "/World/Graphs")
        ensure_xform(stage, "/World/DynamicObstacles")

        jackal_prim = ensure_xform(
            stage,
            self.config.robot.runtime_prim_path,
        )

        ensure_reference(
            prim=jackal_prim,
            asset_path=self.config.robot.asset_path,
        )

        validate_prim(
            stage,
            self.config.robot.runtime_prim_path,
        )

        return stage
```

## 10.3 PhysicsSetup

```python
class PhysicsSetup:
    def __init__(self, config):
        self.physics_dt = 1.0 / config.physics_hz
        self.rendering_dt = 1.0 / config.rendering_hz

    def apply(self, stage):
        physics_scenes = find_all_physics_scenes(stage)

        if len(physics_scenes) == 0:
            scene_path = "/PhysicsScene"
            create_physics_scene(stage, scene_path)

        elif len(physics_scenes) == 1:
            scene_path = physics_scenes[0].GetPath().pathString

        else:
            paths = [str(p.GetPath()) for p in physics_scenes]
            raise RuntimeError(
                f"Multiple PhysicsScene prims detected: {paths}"
            )

        validate_stage_units(stage, expected_meters=1.0)
        validate_up_axis(stage, expected="Z")

        set_or_validate_physics_parameters(
            stage=stage,
            scene_path=scene_path,
            physics_hz=1.0 / self.physics_dt,
            gravity=9.81,
            solver="TGS",
            enable_ccd=True,
            enable_stabilization=True,
        )

        simulation_context = create_simulation_context(
            stage=stage,
            physics_scene_path=scene_path,
            physics_dt=self.physics_dt,
            rendering_dt=self.rendering_dt,
            set_defaults=False,
        )

        return simulation_context
```

重点：

* 不无条件创建 `/World/physicsScene`；
* 不同时创建两个不同路径的 PhysicsScene；
* `stage` 参数必须真正使用；
* 返回并持有 SimulationContext。

## 10.4 SpawnPoseManager

```python
class SpawnPoseManager:
    def __init__(self, robot, pose_config):
        self.robot = robot
        self.pose_config = pose_config

    def apply_usd_pose(self, pose_name):
        pose = self.pose_config[pose_name]["usd"]

        position = pose["position"]
        orientation = quaternion_from_yaw_deg(
            pose["yaw_deg"]
        )

        self.robot.set_world_pose(
            position=position,
            orientation=orientation,
        )

        self.robot.set_linear_velocity([0.0, 0.0, 0.0])
        self.robot.set_angular_velocity([0.0, 0.0, 0.0])
        self.robot.set_joint_velocities(
            [0.0] * self.robot.num_dof
        )

    def get_map_pose(self, pose_name):
        return self.pose_config[pose_name]["map"]
```

## 10.5 主程序

```python
from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": False,
    "renderer": "RaytracedLighting",
})


def main():
    config = load_project_config()

    enable_required_extensions(config)

    stage = SceneComposer(config).compose()
    simulation = PhysicsSetup(config.simulation).apply(stage)

    robot = ArticulationRuntime(
        prim_path=config.robot.runtime_prim_path,
    )
    robot.initialize()

    JointValidator(config.robot).validate(
        robot.get_dof_names()
    )

    spawn_manager = SpawnPoseManager(
        robot=robot,
        pose_config=load_spawn_poses(),
    )
    spawn_manager.apply_usd_pose(config.spawn.selected)

    sensors = SensorFactory(stage, config).create_all()

    RosGraphBuilder(
        config=config,
        robot=robot,
        sensors=sensors,
    ).build()

    ground_truth = GroundTruthRecorder(config, robot)

    reset_manager = ResetManager(
        robot=robot,
        spawn_manager=spawn_manager,
        simulation=simulation,
    )

    simulation.reset()
    simulation.play()

    try:
        while simulation_app.is_running():
            simulation_app.update()
            ground_truth.update()
    finally:
        simulation.stop()
        simulation_app.close()
```

## 10.6 控制 Graph

```text
OnPlaybackTick
    │
    ▼
ROS2SubscribeTwist
    │
    ├── linearVelocity → BreakVector3.x
    └── angularVelocity → BreakVector3.z
    │
    ▼
DifferentialController
    │
    ├── velocityCommand → FrontArticulationController
    └── velocityCommand → RearArticulationController
```

不创建：

```text
CmdVelWatchdog
ReadSimulationTime for Watchdog
```

## 10.7 RTX LiDAR

```python
def create_rtx_lidar(config):
    lidar = create_lidar_prim(
        prim_path=config.sensor_prim,
        config_name=config.rtx_config_name,
        frequency=config.publish_rate,
    )

    attach_ros2_pointcloud_writer(
        lidar=lidar,
        topic_name="/lidar/points_raw",
        frame_id="lidar_link",
    )

    return lidar
```

RTX Config 名称必须在本机 Isaac Sim 6.0.1 中验证。

## 10.8 pointcloud_to_laserscan

```yaml
pointcloud_to_laserscan:
  ros__parameters:
    use_sim_time: true

    target_frame: base_link
    transform_tolerance: 0.05

    min_height: 0.05
    max_height: 0.50

    angle_min: -3.14159265
    angle_max: 3.14159265
    angle_increment: 0.008726646

    scan_time: 0.10

    range_min: 0.30
    range_max: 25.0
    use_inf: true
```

Launch：

```python
Node(
    package="pointcloud_to_laserscan",
    executable="pointcloud_to_laserscan_node",
    parameters=[
        projection_config,
        {"use_sim_time": True},
    ],
    remappings=[
        ("cloud_in", "/lidar/points_raw"),
        ("scan", "/scan"),
    ],
)
```

若需要 Self Filter，仅把 `cloud_in` 改为：

```text
/lidar/points_scan
```

## 10.9 Mapping Launch

```python
Node(
    package="slam_toolbox",
    executable="async_slam_toolbox_node",
    name="slam_toolbox",
    parameters=[
        mapping_config,
        {"use_sim_time": True},
    ],
    remappings=[
        ("scan", "/scan"),
    ],
)
```

## 10.10 Localization Launch

```python
Node(
    package="slam_toolbox",
    executable="localization_slam_toolbox_node",
    name="slam_toolbox",
    parameters=[
        localization_config,
        {
            "use_sim_time": True,
            "map_file_name": serialized_posegraph,
        },
    ],
    remappings=[
        ("scan", "/scan"),
    ],
)
```

## 10.11 固定初始位姿发布

```python
def publish_initial_pose(publisher, map_pose, stamp):
    msg = PoseWithCovarianceStamped()

    msg.header.frame_id = "map"
    msg.header.stamp = stamp

    msg.pose.pose.position.x = map_pose["position"][0]
    msg.pose.pose.position.y = map_pose["position"][1]

    msg.pose.pose.orientation = quaternion_from_yaw_deg(
        map_pose["yaw_deg"]
    )

    msg.pose.covariance[0] = 0.05 ** 2
    msg.pose.covariance[7] = 0.05 ** 2
    msg.pose.covariance[35] = radians(5.0) ** 2

    publisher.publish(msg)
```

发布时机：

1. `/clock` 有效；
2. Localization 节点已启动；
3. TF 可查询；
4. 必要时连续发布数次；
5. 确认 `map → odom` 稳定后激活 Nav2。

## 10.12 ResetManager

```python
class ResetManager:
    def reset(self, pose_name, mode):
        self.simulation.pause()

        self.send_zero_velocity()
        self.clear_controller_state()

        self.spawn_manager.apply_usd_pose(pose_name)

        self.reset_ideal_odom_or_ekf(mode)
        self.reset_ground_truth_path()
        self.reset_dynamic_obstacles()
        self.clear_costmaps()

        self.simulation.step(render=False)
        self.simulation.play()

        if mode == "localization":
            self.publish_map_initial_pose(pose_name)
```

---

# 第十一部分：完整实施 SOP

## 11.1 Stage 与资产

1. 建立项目目录；
2. 创建 `navigation_scene.usda`；
3. 在 Layer 面板将其设置为 Edit Target；
4. 将官方 `warehouse_multiple_shelves.usd` 加为 Sublayer；
5. 确认出现 `/Root`、`/PhysicsScene`、`/NavMesh`；
6. 确认 Stage 只有一个 PhysicsScene；
7. 创建 `/World`；
8. 创建 `/World/Robots`；
9. 创建 `/World/Graphs`；
10. 复制并整理项目 Jackal 资产；
11. 确认 `jackal_nav.usda` 有有效 `defaultPrim`；
12. 将 `jackal_nav.usda` 作为 Reference 挂到 `/World/Robots/Jackal`；
13. 不引用 `configuration/*.usd`；
14. 运行 Asset Validator；
15. 保存项目 Stage。

## 11.2 Jackal 修改

16. 删除旧 LiDAR Sensor Prim；
17. 保留安装 Frame；
18. 重命名为 `lidar_link`；
19. 删除旧 IMU Sensor；
20. 将 `com_frame` 重命名为 `imu_link`；
21. 整理相机 Frame；
22. 验证固定传感器不是独立刚体；
23. 检查 Articulation Root；
24. 打印四个 Joint 名称；
25. 单轮验证正方向；
26. 检查质量、碰撞和轮胎材料。

## 11.3 固定出生点

27. 在 GUI 中选择开阔、平整、有足够几何特征的位置；
28. 设置 `/World/Robots/Jackal` 的 X、Y、Z、Yaw；
29. 保持 Roll、Pitch 为零；
30. 保持 Scale 为一；
31. 播放确认机器人正常落地；
32. 记录 USD Pose 到 `spawn_poses.yaml`；
33. 每次启动和 Reset 使用该 Pose；
34. 建图完成后记录该地点对应 Map Pose。

## 11.4 控制与时间

35. 创建控制 Graph；
36. 测试前进、后退、原地旋转；
37. 配置 Velocity Smoother 超时；
38. 配置 Collision Monitor；
39. 发布 `/clock`；
40. 所有 ROS 算法节点设置 `use_sim_time=true`。

## 11.5 TF 与 Odom

41. 发布结构 TF；
42. 发布理想 `/odom`；
43. 验证 `odom → base_link`；
44. 确认没有 ROS `world` Frame；
45. 确认无重复 TF。

## 11.6 LiDAR 与 `/scan`

46. 创建 RTX LiDAR；
47. 发布 `/lidar/points_raw`；
48. 在 RViz 检查距离和方向；
49. 启动 `pointcloud_to_laserscan`；
50. 调整高度范围；
51. 检查是否看到自车点；
52. 只有存在自车点时添加 CropBox；
53. 检查 `/scan` 与点云最近障碍一致。

## 11.7 建图与定位

54. 固定到 `mapping_start`；
55. 重置 Odom；
56. 启动 Mapping；
57. 缓慢原地旋转检查地图；
58. 完成完整路线；
59. 进行闭环；
60. 保存 OccupancyGrid；
61. 保存 Pose Graph；
62. 关闭 Mapping；
63. 重新放回固定 USD Pose；
64. 启动 Localization；
65. 发布固定 Map Pose；
66. 检查 `map → odom` 稳定；
67. 启动 Nav2。

## 11.8 Nav2 与实验

68. 配置 Global Costmap；
69. 配置 Local Costmap；
70. 配置 SmacPlanner2D；
71. 配置 MPPI；
72. 配置 Velocity Smoother；
73. 配置 Collision Monitor；
74. 进行静态避障实验；
75. 添加动态障碍；
76. 进行反应式动态避障实验；
77. 发布 Ground Truth；
78. 实现 Wheel Odom；
79. 配置 EKF；
80. 自动化 Reset；
81. 保存 rosbag、CSV、JSON；
82. 进行增量地图更新；
83. 迁移到自定义机器人。

---

# 第十二部分：实验、指标和统计方法

## 12.1 单次实验成功条件

同时满足：

1. Nav2 Action 返回成功；
2. Ground Truth 到达目标区域；
3. 位置误差不超过 `0.25 m`；
4. 要求朝向时，角度误差不超过 `10°`；
5. 无碰撞；
6. 未定位丢失；
7. 无 TF 中断；
8. 未超时；
9. Collision Monitor 未长期锁死；
10. 最终速度接近零。

## 12.2 静态避障率

[
R_{static}
==========

\frac{N_{static_success}}
{N_{static_total}}
\times 100%
]

要求：

[
R_{static}\ge95%
]

建议：

```text
10 组起止点
× 5 组障碍布局
× 4 个随机种子
= 200 次
```

## 12.3 动态避障率

[
R_{dynamic}
===========

\frac{N_{dynamic_success}}
{N_{dynamic_total}}
\times 100%
]

要求：

[
R_{dynamic}\ge90%
]

动态模式：

* 横穿；
* 对向；
* 同向慢速；
* 临时封路；
* 双动态障碍；
* 不同尺寸；
* 不同速度。

当前二维基线只评价进入投影高度范围的障碍。

## 12.4 导航成功率

[
R_{navigation}
==============

\frac{N_{goal_reached}}
{N_{total}}
\times100%
]

要求：

[
R_{navigation}\ge90%
]

## 12.5 路径长度偏差

[
E_L
===

\frac{|L_{executed}-L_{optimal}|}
{L_{optimal}}
\times100%
]

要求：

[
E_L\le20%
]

Ground Truth 路径：

[
L_{executed}
============

\sum_{i=1}^{N-1}
\left|
p_{i+1}^{gt}-p_i^{gt}
\right|
]

理论最优路径必须考虑：

* Footprint；
* Inflation；
* Keepout；
* 不可通行栅格。

## 12.6 定位误差

[
e_t(i)
======

\left|
p_i^{estimate}-p_i^{gt}
\right|
]

[
RMSE_t
======

\sqrt{
\frac{1}{N}
\sum_{i=1}^{N}e_t(i)^2
}
]

角度误差：

[
e_\theta(i)
===========

wrap(\theta_i^{estimate}-\theta_i^{gt})
]

记录：

* 平移 RMSE；
* 角度 RMSE；
* 最大误差；
* 95% 分位误差；
* 重定位时间；
* 定位丢失次数；
* `map → odom` 跳变量。

## 12.7 里程计误差

比较：

```text
Ideal Odom
Wheel Odom
EKF Odom
Ground Truth
```

指标：

* 每米位置漂移；
* 每 360° 角度漂移；
* 直线距离误差；
* 原地旋转误差；
* 闭环漂移。

## 12.8 控制性能

[
RMSE_v
======

\sqrt{
\frac{1}{N}\sum(v_{gt}-v_{cmd})^2
}
]

[
RMSE_\omega
===========

\sqrt{
\frac{1}{N}\sum(\omega_{gt}-\omega_{cmd})^2
}
]

记录：

* 加速时间；
* 制动时间；
* 最大超调；
* 命令超时停车延迟；
* 前后轮速度一致性；
* 直行偏航；
* 原地旋转半径。

## 12.9 `/scan` 投影质量

记录：

* 点云与 LaserScan 最近距离误差；
* 地面误检率；
* 自车残留率；
* 高处结构虚假投影率；
* 小障碍漏检率；
* 投影处理延迟；
* `/scan` 丢帧率。

## 12.10 增量地图更新

[
I_T
===

\frac{T_{full}-T_{incremental}}
{T_{full}}
\times100%
]

要求：

[
I_T\ge30%
]

## 12.11 可重复性

每轮保存：

```yaml
scenario_id:
random_seed:
map_version:
posegraph_version:
robot_config_hash:
nav2_config_hash:
spawn_pose_name:
usd_start_pose:
map_start_pose:
goal_pose:
obstacle_trajectories:
physics_dt:
rtf:
result:
failure_reason:
```

原方案中的避障率、导航成功率和路径偏差目标保持不变。

---

# 第十三部分：常见错误和调试清单

| 问题                          | 典型现象                    | 检查与修复                                         |
| --------------------------- | ----------------------- | --------------------------------------------- |
| ROS_DOMAIN_ID 不一致           | Topic 互不可见              | 两端检查 `echo $ROS_DOMAIN_ID`                    |
| Fast DDS 不一致                | Discovery 异常            | 统一 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`      |
| `use_sim_time` 未开启          | SLAM/Nav2 时间错误          | 所有算法节点设为 true                                 |
| `/clock` 缺失                 | ROS 时间为 0               | 检查 Clock Graph 和 Timeline                     |
| `/clock` 回退                 | TF 报旧数据                 | Reset 时保持时间单调或重启相关节点                          |
| TF 重复发布                     | RViz 跳动                 | 检查每段 TF 唯一所有权                                 |
| Frame ID 不一致                | Message Filter 丢数据      | 检查 `base_link/lidar_link/imu_link`            |
| optical frame 错误            | 图像方向异常                  | 验证 x右、y下、z前                                   |
| Joint 顺序错误                  | 左右轮交叉                   | 控制数组严格 `[left,right]`                         |
| 轮子方向错误                      | 前进变转向                   | 单轮 `+1 rad/s` 测试                              |
| Articulation Root 错误        | 找不到 DOF                 | 保证唯一有效 Root                                   |
| Ground Truth 误接入 Nav2       | 结果虚假地好                  | GT 只能进入 Metrics                               |
| QoS 不匹配                     | Topic 存在但无数据            | 检查 Reliability/Durability                     |
| Camera Render Product 未执行   | Camera 无图像              | 创建并驱动 Render Product                          |
| LiDAR 频率不正确                 | 点云 Hz 波动                | 检查 Sensor Tick 和 RTF                          |
| PointCloud 坐标错误             | 点云旋转、漂移                 | 检查 Frame 与安装变换                                |
| `/cmd_vel` 超时               | ROS 停止后仍运动              | 配置 Velocity Smoother timeout；必要时增加低层 Watchdog |
| USD `/World` 与 ROS `map` 混淆 | TF 中出现 world            | 不发布 USD Prim 名为 ROS Frame                     |
| 直接修改官方资产                    | 更新后丢失修改                 | 官方资产只读，使用项目 Layer                             |
| ROS 与 Isaac Python 冲突       | `rclpy` 或库加载失败          | 使用不同终端和解释器                                    |
| 多个 PhysicsScene             | 重力或求解异常                 | 遍历 Stage，确保唯一                                 |
| 固定创建 `/World/physicsScene`  | 出现第二个 PhysicsScene      | 复用官方 `/PhysicsScene`                          |
| 环境错误使用 Reference            | 丢失 PhysicsScene/NavMesh | 完整环境使用 Sublayer                               |
| Schema 文件作为 Reference       | `no default prim`       | 引用 Jackal 主入口文件                               |
| Sublayer 路径失效               | 场景缺失                    | 校验本地资产路径                                      |
| Edit Target 错误              | 修改写入官方资产                | Edit Target 设为项目根 Layer                       |
| Mapping 与 Localization 同时运行 | 两个 `map→odom`           | Launch 严格互斥                                   |
| Isaac 与 EKF 同发 Odom TF      | 位姿跳动                    | Realistic 模式关闭 Isaac Odom                     |
| Isaac 与 RSP 同发结构 TF         | 重复 TF                   | ROS Standard 模式关闭 Isaac TF                    |
| `/scan` 两个发布者               | 数据交替                    | 只保留一个投影节点                                     |
| 投影高度过高                      | 高处横梁封路                  | 按机器人碰撞高度设置 `max_height`                       |
| 投影高度过低                      | 高障碍漏检                   | 逐步增加 `max_height`                             |
| 地面点进入 `/scan`               | 周围近距离障碍                 | 提高 `min_height`                               |
| 自车点进入 `/scan`               | 机器人周围固定障碍               | 添加 CropBox                                    |
| VoxelGrid 与 Voxel Layer 混淆  | 配置职责错误                  | 前者降采样，后者 Costmap 插件                           |
| USD Pose 与 Map Pose 混淆      | 定位初始位置错误                | 同一出生点保存两组坐标                                   |
| Reset 后轮速残留                 | 重置后突然运动                 | 清零速度和 Joint Target                            |
| `wheelDistance` 不准确         | 原地旋转误差大                 | 标定有效轮距                                        |
| RTF 过低                      | 导航超时                    | 降低 LiDAR 分辨率或控制计算量                            |

---

# 第十四部分：分阶段开发计划和验收标准

以下内容保留原始阶段目标和严格验收门槛，不应仅凭代码存在就勾选完成。每阶段截至 2026-07-15 的“实现/实测/边界”结论见本文开头的 **0. 修订说明与当前状态**；尤其阶段 10 的 90% 广义动态避障率、阶段 11 的完整统计矩阵、阶段 12 的真实 changed-region 30% 改善和阶段 13 的真实机器人迁移仍未验收。

## 阶段 1：Jackal 物理底盘验证

工作：

* 创建项目主 Layer；
* Sublayer 官方仓库；
* Reference 项目 Jackal；
* 校验唯一 PhysicsScene；
* 设置固定出生点；
* 检查碰撞、质量、Articulation、Joint。

验收：

* 无 unresolved assets；
* Stage 只有一个 PhysicsScene；
* Jackal Reference 正常展开；
* 机器人静置稳定；
* 不穿透地面；
* 四轮 Joint 可读取；
* 单轮方向正确；
* 每次重启都回到同一出生点。

## 阶段 2：`/cmd_vel` 控制

工作：

* ROS2SubscribeTwist；
* DifferentialController；
* 两个 ArticulationController；
* 限速和加速度限制。

验收：

* 正向直行；
* 倒车；
* 正向原地旋转；
* 前后同侧轮速一致；
* 不直接修改机器人 Pose；
* Velocity Smoother 超时后能停车。

## 阶段 3：`/clock`

验收：

* `/clock` 约 60 Hz；
* Pause 时 ROS 时间暂停；
* Play 后继续；
* ROS 消息使用仿真时间。

## 阶段 4：TF 和理想 `/odom`

验收：

* TF 树为 `map→odom→base_link`；
* 无 `world`；
* 无重复 TF；
* `/odom` 与 TF 一致；
* RViz RobotModel 稳定。

## 阶段 5：单一 LiDAR

工作：

* RTX LiDAR；
* `/lidar/points_raw`；
* 直接生成 `/scan`；
* 可选自车过滤。

验收：

* 点云稳定 10 Hz；
* Frame 正确；
* 障碍方向和距离正确；
* `/scan` 频率稳定；
* 点云和 `/scan` 最近距离一致；
* 地面和高处结构无明显误投影。

## 阶段 6：SLAM

工作：

* 固定 Mapping 起点；
* Mapping；
* 闭环；
* 保存 OccupancyGrid；
* 保存 Pose Graph；
* 标定固定起点 Map Pose；
* Localization。

验收：

* 地图无明显撕裂；
* 闭环后轨迹一致；
* 地图可重启加载；
* Localization 可从固定起点稳定定位；
* Mapping 与 Localization 不同时运行。

## 阶段 7：Nav2

工作：

* Global Costmap；
* Local Costmap；
* SmacPlanner2D；
* MPPI；
* Velocity Smoother；
* Collision Monitor。

验收：

* 多组目标成功；
* 路径不穿越膨胀障碍；
* 局部控制无持续振荡；
* 接近障碍减速；
* Stop Zone 能停车；
* 不使用 Voxel Layer 仍可完成二维导航。

## 阶段 8：Ground Truth

验收：

* GT 与视觉位置一致；
* `map_T_usd` 对齐正确；
* GT 不发布导航 TF；
* GT 不被 SLAM、EKF、Nav2 订阅；
* 可计算路径和定位误差。

## 阶段 9：真实感里程计

验收：

* `/wheel/odom` 与 JointState 一致；
* EKF 唯一发布 `/odom` 和 Odom TF；
* Isaac 理想 Odom 已关闭；
* 直线和旋转误差可统计；
* 合理滑移下 Localization 稳定。

## 阶段 10：动态避障

工作：

* 添加进入二维投影高度范围的动态障碍；
* Local Obstacle Layer；
* MPPI；
* Collision Monitor。

验收：

* 横穿和对向障碍可避让；
* 动态障碍及时进入和离开 Costmap；
* 动态避障率达到 90%；
* 无障碍时不长期急停；
* 不把二维结果描述成三维避障。

## 阶段 11：自动实验

验收：

* 单次 Isaac Sim 会话完成多轮实验；
* 每轮恢复固定 USD 起点；
* Localization 自动发布对应 Map Pose；
* 速度、轮速、Costmap、路径全部重置；
* 随机种子可复现；
* CSV/JSON 字段完整。

## 阶段 12：增量地图更新

验收：

* 更新时间改善不少于 30%；
* 变化区域正确更新；
* 旧区域无明显退化；
* 更新后 Localization 和 Nav2 正常。

## 阶段 13：自定义机器人迁移

工作：

* 替换自定义 USD；
* 更新机器人 YAML；
* 更新 URDF/Xacro；
* 标定轮径、轮距和 Footprint；
* 保持 ROS 接口不变。

验收：

* Topic 名称不变；
* TF 主树不变；
* Nav2 配置结构不变；
* 只替换机器人参数和资产；
* 理想模式、真实模式、Ground Truth 和固定出生点全部可用。
