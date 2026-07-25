# Isaac Sim 6.0.1 + ROS 2 Jazzy：酷家乐 RGB-D 导航

https://github.com/user-attachments/assets/4b83405f-be85-41e3-9f3e-a8224a30b06d

本分支提供 Clearpath Jackal 在酷家乐室内场景中的 Isaac Sim 导航闭环：二维
LiDAR、前向 RGB-D、Nav2、RViz、确定性 Reset 与长距离实验。README、使用手册和
正式长距离场景使用的标准酷家乐组合是：

```text
场景:      kujiale_0026_A_to_B_door_open.usd
地图:      warehouse_new
长距离出生点: long_route_start_g1（Map `[0.45, -5.35, 90°]`）
定位:      Ideal Odometry / 已标定、按出生点对齐的 map -> odom
导航感知:  /scan + /camera/front/depth/points
```

`warehouse_new` 仅用于普通 Ideal Localization/Navigation；用 `realistic` 或
`posegraph_calibration:=true` 启动该地图会被 `run_ros.sh` 拒绝。Warehouse 的旧
地图工件不再随仓库分发；旧实验和历史调参不属于本分支的运行入口，见
[`docs/documentation_status.md`](docs/documentation_status.md)。

## 长距离重设计状态

当前长距离静态/动态配置从 `long_route_start_g1` 出生，依次运行
`G1 → G2 → G3 → G4 → G5 → G1`。原狭窄通道航点已移除，原左侧厕所和左下房间航点依次重命名为 G4、G5；中心区使用四个可在
Isaac GUI 中诊断性拖动的 RGB-D 低矮方块和两个低矮长条，或三个按空间 gate 接力运动并停车的动态方块（G1→G2、G2→G3、G5→G1）。六个
静态障碍的当前坐标来自 `2026-07-23 13:37:02 +08:00` 的完整 GUI 捕获：四个方块为 `0.30 × 0.30 × 0.16 m`，
两个长条为 `0.60 × 0.30 × 0.16 m`；4×20正式运行使用版本化 YAML 和哈希，任何 GUI 手调都必须先导出、更新配置并重新运行四组证据。

## 当前 4×20 外观鲁棒性实验

当前正式复跑入口是四组各 20 轮的 4×20 campaign：静态基准、静态＋外观变化、动态基准、
动态＋外观变化。它使用同一条 G1–G5 闭环路线；外观变化只在匿名 USD Session Layer 中
覆盖灯光和材质颜色，不写回源 USD，也不改变几何、碰撞、地图或动态障碍运动学。

当前版本已实现调度、断点续跑、预检、每轮 RGB 快照和可视化报告，但**尚未执行这 80 轮**，
因此不能宣称任一 4×20 组已达到 90% 成功率。完整可执行命令、四组矩阵和验收口径以
[`docs/kujiale_4x20_appearance_benchmark_plan.md`](docs/kujiale_4x20_appearance_benchmark_plan.md) 为准。

| 项目 | 当前结果 |
| --- | --- |
| 4×20 实现与离线测试 | 已完成；80 轮运行证据尚未产生 |
| 静态基准 / 静态＋外观 | 各要求严格成功且无碰撞 `≥19/20` |
| 动态基准 / 动态＋外观 | 各要求严格成功且无碰撞 `≥18/20` |
| 静态路径偏差 | 每个成功轮次 `≤20%` |

`kujiale_long_route_static_20260723-194416` 的静态 20/20 和更早的旧路线结果均为历史证据，
不替代本轮 4×20 结果；边界见 [`docs/verification.md`](docs/verification.md)。

完整路线、验收口径、失败边界和报告目录结构见
[`docs/kujiale_4x20_appearance_benchmark_plan.md`](docs/kujiale_4x20_appearance_benchmark_plan.md)；
静态/动态地图和航点见
[`docs/kujiale_long_route_map.md`](docs/kujiale_long_route_map.md)。

## 首次准备

```bash
cd /your/path/Isaac_Sim_ROS2_Nav
export PROJECT_ROOT="$PWD"
git lfs install
git lfs pull
./scripts/import_assets.sh
./scripts/build_ros2.sh
./scripts/preflight.sh
```

脚本默认使用 ROS 2 Jazzy、`ROS_DOMAIN_ID=42` 和 `rmw_fastrtps_cpp`。如果需要
在终端中直接使用 `ros2`，先执行：

```bash
source ./scripts/setup_ros_env.sh
```

`preflight.sh` 成功时会显示 `map manifest verified: warehouse_new bundle=<SHA256>` 和
`preflight: PASS`。它会校验地图 bundle（含 Git LFS 工件）、GPU、Isaac、ROS 与已构建
工作区。

## 手动导航：两个终端

开始前不要已有第二套 Isaac 或 ROS 栈。若脚本提示锁被占用，先运行
`./scripts/diagnose.sh`；复用已有的同配置会话，或按下面的停止顺序正常关闭，
不要用 `pkill`。

终端 A 启动 Isaac GUI、Ideal Odom 与 RGB-D：

```bash
cd /你的实际路径/Isaac_Sim_ROS2_Nav
./scripts/run_isaac.sh \
  --environment-usd kujiale_0026_A_to_B_door_open.usd \
  --navigation-mode localization \
  --mode ideal \
  --spawn-pose long_route_start_g1 \
  --camera-profile rgbd_navigation
```

Isaac 控制台必须出现 `spawn=long_route_start_g1`；这对应 USD
`[2.45, 5.15, 0.0635]`、Map `[0.45, -5.35, 90°]`。若仍显示
`spawn=mapping_start`，先停止该旧进程并用以上完整命令重启，不能在已启动的 Isaac
进程中热切换出生点。

终端 B 启动 Navigation、Map Server 与受管 RViz：

```bash
cd /你的实际路径/Isaac_Sim_ROS2_Nav
./scripts/run_ros.sh navigation odometry_mode:=ideal spawn_pose_name:=long_route_start_g1
```

等待 `Nav2 lifecycle activation completed`。随后在 RViz：

1. 确认 Fixed Frame 为 `map`，**Navigation 2 Safe** 面板为激活状态；
2. 选择工具栏 **2D Goal Pose**；
3. 在可通行区域拖出目标位置和朝向；
4. 观察全局/局部路径、MPPI 轨迹、Costmap 和 Collision Monitor；
5. 确认面板显示成功且机器人停止。

目标由 Nav2 标准 `goal_pose` 接口处理；没有项目自定义目标桥，也不需要第三个
终端。完整的人工回归目标、RGB-D 可视化、Reset 与排障步骤见
[`docs/user_manual.md`](docs/user_manual.md)。

静态/动态全屋长距离测试的 `warehouse_new` 地图、S/G1 与 G2–G5 航点、静态方块和三阶段
动态障碍路线见 [`docs/kujiale_long_route_map.md`](docs/kujiale_long_route_map.md)。

若要一边拖动四个静态方块和两个静态长条、一边在 RViz 手动发送 Goal 观察效果，请按
[`docs/user_manual.md`](docs/user_manual.md#82-静态可视化单轮) 的“交互式布局与手动导航”流程启动；
不要运行会 Reset 方块位置的自动 `run_visual_route.sh static`。

## RGB-D 感知边界

`--camera-profile rgbd_navigation` 会发布：

```text
/camera/front/image_raw
/camera/front/camera_info
/camera/front/depth/points
```

深度点云的 Costmap 消费者由 `nav2_profile` 决定：`stable` 在 Local 和 Global
Costmap 都使用 `depth_voxel_layer`，保持低矮静态障碍；`dynamic_avoidance` 在
Local 使用时空 STVL、Global 只使用静态图和 `/scan`，避免移动 actor 留下全局残影。
RViz 的 **RGB-D Fusion** 分组分别显示标准 `/local_costmap/voxel_grid` 和动态
`/local_costmap/stvl_voxel_grid`。Collision Monitor 仍只使用二维 `/scan`，RGB-D
不进入 SLAM、EKF 或 Collision Monitor。

## 其他当前操作

| 目标 | 入口 |
| --- | --- |
| 仅检查定位与 TF | `./scripts/run_ros.sh localization odometry_mode:=ideal` |
| 从零建图 | Isaac 使用 `--navigation-mode mapping --mode ideal`，ROS 使用 `./scripts/run_ros.sh mapping odometry_mode:=ideal` |
| 保存新地图 | `./scripts/save_map.sh <新版本名>`；新地图先是未标定状态，必须完成标定后才能用 `initial_pose_source:=auto`。 |
| 无头自动运行 | Isaac 增加 `--headless`；ROS 增加 `interactive:=false`。 |
| 查看当前进程/锁 | `./scripts/diagnose.sh` |
| 受管清理 | `./scripts/clean_runtime.sh --dry-run`，确认目标后才按输出执行。 |

Mapping Teleop 只属于 Mapping，不能与 Localization 或
Navigation 同时发布 `/cmd_vel`。

## 实验与报告

### 当前正式入口：4×20 光照/颜色鲁棒性 campaign

推荐入口会自动构建工作区、启动静态 Isaac/Nav2、运行静态 pilot＋40轮并立即保留静态 `2×20` 报告、有序关闭两套栈、
启动动态栈、运行动态 pilot＋40轮并保留动态 `2×20` 报告，最后才生成同一批次总4×20报告：

```bash
./scripts/run_kujiale_4x20_all.sh
# 或指定可追踪批次 ID：
# ./scripts/run_kujiale_4x20_all.sh 20260725-120000
```

静态/动态切换由脚本管理，不需要打开多个终端。中断后的同一 ID 可用
`./scripts/run_kujiale_4x20_all.sh "$CAMPAIGN_ID" --resume` 继续；`--skip-build` 可跳过已经完成的构建。
预检项目和报告退出码见
[`docs/kujiale_4x20_appearance_benchmark_plan.md`](docs/kujiale_4x20_appearance_benchmark_plan.md)。

报告输出位于 `data/reports/kujiale_4x20_<campaign_id>/`：`static_2x20/` 和 `dynamic_2x20/` 是各阶段完成后
立即保留的独立报告，根目录为同一批次80轮完成后的总报告。若静态已通过而动态需修复复测，运行
`./scripts/run_kujiale_4x20_all.sh --dynamic-only --skip-build`；它不重跑静态、也不自动把不同批次合并成总4×20结论。
已完成静态但尚未产生子报告的批次可用 `./scripts/run_kujiale_4x20.sh static-report <CAMPAIGN_ID>` 补报。各报告均包含
PDF、Markdown、PNG、CSV、JSON 和证据索引；即使验收失败也生成报告，此时命令返回 `2`。

### 历史候选与可视化入口

正式静态、动态场景配置位于：

```text
ros2_ws/src/robot_experiments/config/kujiale_static_long_range.yaml
ros2_ws/src/robot_experiments/config/kujiale_dynamic_long_range.yaml
ros2_ws/src/robot_experiments/config/kujiale_long_range_campaign.yaml
```

静态 GUI + RViz 单轮回归使用 `kujiale_static_visual.yaml` 和
`./scripts/run_visual_route.sh static`。动态可视化使用 schema-v4 三阶段 actor
与 `./scripts/run_kujiale_three_stage_visual.sh {g1-g2|g2-g3|g5-g1|full}`：整圈会自动发送
`G1 → G2 → G3 → G4 → G5 → G1`，在 G1→G2、G2→G3、G5→G1 分别触发一个 actor。两类
可视化都不构成正式 20+20 验收；动态 `--record` 只保留本轮观测证据。

静态和动态的 GUI 自动全路线测试已在
[`docs/user_manual.md`](docs/user_manual.md#8-可视化单轮全屋长距离测试isaac-gui--rviz) 汇总；两种自动
模式都会发送 `G2 → G3 → G4 → G5 → G1`。

### 静态可视化一轮（Isaac GUI + RViz）

用途是人工观察六个低矮静态障碍、RGB-D 点云/VoxelGrid、Costmap 和 MPPI 行为。开始前先确保
没有另一套 Isaac 或 ROS 会话；依次在三个终端运行，终端 B 出现
`Nav2 lifecycle activation completed` 后才启动终端 C：

```bash
# 终端 A：Isaac GUI + 六个静态障碍
cd "$PROJECT_ROOT"
ISAAC_NAV__GROUND_TRUTH__ENABLED=true ./scripts/run_isaac.sh \
  --environment-usd kujiale_0026_A_to_B_door_open.usd \
  --navigation-mode localization \
  --mode ideal \
  --spawn-pose long_route_start_g1 \
  --camera-profile rgbd_navigation \
  --dynamic-obstacle-config isaac_sim/configs/experiments/kujiale_long_range_static.yaml \
  --dynamic-obstacles

# 终端 B：Navigation + RViz
cd "$PROJECT_ROOT"
./scripts/run_ros.sh navigation odometry_mode:=ideal spawn_pose_name:=long_route_start_g1 nav2_profile:=stable

# 终端 C：唯一静态 seed 7201；自动执行 G2 → G3 → G4 → G5 → G1
cd "$PROJECT_ROOT"
./scripts/run_visual_route.sh static
```

此模式不写入 MCAP、JSON、CSV 或报告，不能作为验收结果。运行时只观察，不要在 RViz 再手动发送
Goal；若要拖动障碍后手动导航，保持 A/B 运行且不要执行终端 C，因为 visual runner 会 Reset 障碍到 YAML 基线。

### 静态 20 轮自动候选测试（无头 Isaac）

该测试顺序运行种子 `7201`–`7220`，输出可复核的静态候选报告；运行期间不要关闭终端 A/B，也不要打开
RViz 或 Teleop。先停止可视化会话，再按下列顺序启动：

```bash
# 终端 A：无头 Isaac + 六个静态障碍
cd "$PROJECT_ROOT"
ISAAC_NAV__GROUND_TRUTH__ENABLED=true ./scripts/run_isaac.sh \
  --headless \
  --environment-usd kujiale_0026_A_to_B_door_open.usd \
  --navigation-mode localization \
  --mode ideal \
  --spawn-pose long_route_start_g1 \
  --camera-profile rgbd_navigation \
  --dynamic-obstacle-config isaac_sim/configs/experiments/kujiale_long_range_static.yaml \
  --dynamic-obstacles

# 终端 B：无交互 Navigation；等待 Nav2 lifecycle activation completed
cd "$PROJECT_ROOT"
./scripts/run_ros.sh navigation \
  odometry_mode:=ideal \
  spawn_pose_name:=long_route_start_g1 \
  interactive:=false \
  use_rviz:=false

# 终端 C：执行并汇总静态 20 轮；可选固定批次 ID
cd "$PROJECT_ROOT"
./scripts/run_kujiale_static_20.sh
# ./scripts/run_kujiale_static_20.sh 20260723-120000
```

完成后打开 `data/reports/kujiale_long_route_static_<campaign_id>/index.html`；该目录还包含
`report.pdf`、`report.md`、`benchmark.json`、`benchmark.csv` 和每轮证据。退出码 `0` 表示静态候选
通过，`2` 表示报告已生成但未满足静态门槛；二者都不代表动态或完整 20+20 结论。

运行证据与报告写入 `data/experiment_runs/` 和 `data/reports/`。这些目录中的
HTML、PDF、PNG、CSV、JSON、MCAP 和图像是本地生成物，默认不推送到 Git；受版本
控制的是生成器、场景、校验规则和文档。

自动启动静态 20 轮、动态 20 轮、汇总并核验自包含报告的完整命令见
[`docs/user_manual.md`](docs/user_manual.md)。

## 文档入口

第一次使用建议按“使用手册 → 路线/测试方案 → 接口与排障”的顺序阅读。下面的文档都以
当前 `warehouse_new`、`long_route_start_g1` 和 Ideal Odometry 入口为准；历史结论与草案的
适用范围请先看“文档状态与事实来源”。

| 你要做什么 | 阅读文档 | 可以获得什么 |
| --- | --- | --- |
| 首次启动 Isaac、ROS、RViz 或运行 4×20 | [`docs/user_manual.md`](docs/user_manual.md) | 从环境准备到手动 Goal、4×20 三终端运行、报告查看与有序停止的可复制命令。 |
| 核对酷家乐路线、航点和障碍位置 | [`docs/kujiale_long_route_map.md`](docs/kujiale_long_route_map.md) | `long_route_start_g1`、G1–G5 闭环路线、静态六障碍与动态障碍的地图坐标和语义。 |
| 了解验收口径或准备复跑完整长距离实验 | [`docs/kujiale_4x20_appearance_benchmark_plan.md`](docs/kujiale_4x20_appearance_benchmark_plan.md) | 四组各20轮、外观配置、成功/无碰撞/路径偏差门槛、证据和自包含报告要求。 |
| 调整地图坐标、出生点或定位基线 | [`docs/calibration.md`](docs/calibration.md) | `map -> odom` 标定步骤、地图/posegraph 匹配规则和初始位姿来源。 |
| 修改节点、话题、TF 或 Odometry 配置 | [`docs/interfaces.md`](docs/interfaces.md) | ROS Topic、TF、QoS、模式配对和发布所有权契约，避免重复 `/odom` 或 TF 发布者。 |
| 处理启动失败、锁、RViz、地图或 Reset 问题 | [`docs/troubleshooting.md`](docs/troubleshooting.md) | 分步骤诊断命令、常见故障表现和安全清理流程。 |
| 查看当前结果能否作为结论或引用历史记录 | [`docs/verification.md`](docs/verification.md) | 当前验收状态、运行证据的边界和历史结果不能替代当前复验的条件。 |
| 修改代码、添加测试或理解工程结构 | [`docs/development.md`](docs/development.md) | 本地开发、构建、测试、配置变更和提交前检查约定。 |
| 判断文档是否仍适用于当前工作树 | [`docs/documentation_status.md`](docs/documentation_status.md) | 当前操作文档、历史记录、计划草案和事实来源之间的职责边界。 |
| 快速定位实现与配置文件 | [`docs/repository_index.md`](docs/repository_index.md) | 目录职责、主要脚本、ROS 包和推荐的代码修改入口。 |

## 验证

```bash
./scripts/test.sh
./scripts/test.sh --with-isaac
```

运行 ROS 集成测试时需停止同一 Domain 42 中的 Isaac 仿真；否则真实 `/clock` 会与
测试夹具时钟冲突。代码改动后至少运行 `git diff --check` 和相应测试。
