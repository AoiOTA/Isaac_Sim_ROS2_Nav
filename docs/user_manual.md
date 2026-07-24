# 酷家乐 RGB-D 导航使用手册

> 当前可执行手册，适用分支：`main`。
>
> 本手册只描述当前默认的酷家乐 `warehouse_new` + Ideal Odometry 流程。旧
> Warehouse、Realistic、增量建图和历史 benchmark 记录不作为本分支的日常入口；
> 它们的角色见 [`documentation_status.md`](documentation_status.md)。

## 1. 运行前提

当前导航组合固定为：

| 项目 | 当前值 |
| --- | --- |
| 场景 | `kujiale_0026_A_to_B_door_open.usd` |
| 地图 | `warehouse_new`，`154 x 248 @ 0.05 m` |
| 出生点 | `long_route_start_g1`（Map `[0.45, -5.35, 90°]`） |
| USD 出生位姿 | `[2.45, 5.15, 0.0635]`，yaw `270°` |
| Map 初始位姿 | `[0.45, -5.35]`，yaw `90°` |
| 里程计 | Isaac Ideal `/odom` 和 `odom -> base_link` |
| Map TF | 已标定、按出生点对齐的 `map -> odom` |
| ROS | Jazzy、Domain `42`、`rmw_fastrtps_cpp` |

`warehouse_new` 的 Pose Graph 是建图来源记录，不用于 Realistic 或显式 Pose
Graph 定位。`scripts/run_ros.sh` 会拒绝这两个组合，避免误用未批准的定位链。

## 2. 首次安装与验证

```bash
cd /你的实际路径/Isaac_Sim_ROS2_Nav
export PROJECT_ROOT="$PWD"
git lfs install
git lfs pull
./scripts/import_assets.sh
./scripts/build_ros2.sh
./scripts/preflight.sh
```

预检成功时应包含（`bundle` 值会随地图工件变更）：

```text
map manifest verified: warehouse_new bundle=<SHA256>
preflight: PASS
```

需要直接使用 ROS CLI 时：

```bash
source ./scripts/setup_ros_env.sh
```

不要在不同终端混用 ROS Domain 或 RMW。当前项目要求：

```text
ROS_DOMAIN_ID=42
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

## 3. 手动导航

### 3.1 启动前检查

每次启动前执行：

```bash
./scripts/diagnose.sh
```

如果有受管 Isaac、ROS、RViz 或 Teleop 会话，不要重新启动第二套。应复用相同参数
的会话，或先按第 3.6 节正常关闭。`clean_runtime.sh` 只用于已确认会话异常退出后：

```bash
./scripts/clean_runtime.sh --dry-run
```

不要使用 `pkill`、`kill -9` 或 `rm -rf /dev/shm/*`。

### 3.2 终端 A：启动 Isaac

GUI + RGB-D 手动测试：

```bash
cd "$PROJECT_ROOT"
./scripts/run_isaac.sh \
  --environment-usd kujiale_0026_A_to_B_door_open.usd \
  --navigation-mode localization \
  --mode ideal \
  --spawn-pose long_route_start_g1 \
  --camera-profile rgbd_navigation
```

等待：

```text
Isaac navigation simulation ready
```

同一行还必须包含 `spawn=long_route_start_g1`。它的物理 USD 位姿是
`[2.45, 5.15, 0.0635] / 270°`，对应 Map `[0.45, -5.35] / 90°`；若日志仍是
`spawn=mapping_start`，说明运行的是旧进程或遗漏了 `--spawn-pose`，应停止 Isaac 后以
上述完整命令重启。出生点不能在运行中热切换。

终端 B 启动后，`ideal_localization_tf` 日志必须显示
`Ideal map->odom aligned to selected spawn: x=0.450, y=-5.350, yaw_deg=90.0`；此时
RViz 中的机器人也应落在地图最下方的 G1，而不是旧的 `[0, 0]` 原点。

GUI 会使用 Jackal 的第三人称相机。无头自动运行才使用 `--headless`；手动点击
目标需要 RViz，因此日常人工测试使用 GUI Isaac + RViz 更直接。

### 3.3 终端 B：启动 Navigation 与 RViz

```bash
cd "$PROJECT_ROOT"
./scripts/run_ros.sh navigation odometry_mode:=ideal spawn_pose_name:=long_route_start_g1
```

不传地图参数时，脚本自动选择：

```text
data/maps/posegraphs/warehouse_new
data/maps/occupancy/warehouse_new.yaml
data/maps/manifests/warehouse_new.yaml
```

等待：

```text
Nav2 lifecycle activation completed
```

此命令受管启动 `navigation.rviz`，不要再手工执行 `rviz2` 或 `run_rviz.sh`。

### 3.4 在 RViz 发送目标

1. 确认 Fixed Frame 为 `map`；
2. 确认 **Navigation 2 Safe** 面板显示 Nav2 已激活；
3. 点击 **2D Goal Pose**；
4. 在空闲栅格中按住左键，从目标位置向期望朝向拖动后松开；
5. 观察黄色全局路径、洋红色局部路径、橙色 MPPI 轨迹与 Costmap；
6. 等待面板显示成功，且机器人停止。

当前流程使用 RViz 标准 `SetGoal` 与 Nav2 `goal_pose`。没有私有
`/goal_pose` 转换节点，也不需要第三个终端。

建议人工回归最少完成：

- 同一房间的近目标；
- 穿过门洞的远目标；
- 返回空旷区域的目标。

每个目标记录是否成功、是否出现 StopZone/碰撞、是否有持续振荡，以及完成后是否
静止。Nav2 目标检查的 XY 容差为 `0.20 m`；正式实验的 Ground Truth 成功门槛为
`0.25 m`，二者不是同一个概念。Nav2 的动作收敛半径比报告门槛更严格 `0.05 m`，因此
报告余量不会再成为提前结束的理由。RGB-D VoxelLayer 仅把实际深度观测标记为低矮障碍；相机前向视野外
的全未知体素列不会覆盖静态地图的空闲格，避免到点阶段出现虚假的 Costmap 阻塞。

### 3.5 RGB-D 观察

`rgbd_navigation` 在 Isaac 启动时创建：

```text
/camera/front/image_raw
/camera/front/camera_info
/camera/front/depth/points
```

在 `navigation.rviz` 展开 **RGB-D Fusion**：

- **Robot Front Camera**：前视 RGB；
- **Depth PointCloud2**：青色深度点云；
- **Marked Voxels (3D)**：局部 Costmap 已标记的浅绿色体素。

深度点云仅进入滚动 Local Costmap 的 `depth_voxel_layer`；Global Costmap 只使用
静态地图和实时 `/scan`，避免动态物体在全局代价地图留下视觉残影。默认 RViz 仅渲染
`/local_costmap/voxel_grid`。Collision Monitor 仍只以 `/scan` 作为急停传感器。
RGB-D 不进入 SLAM、EKF 或 odometry。

快速检查：

```bash
source ./scripts/setup_ros_env.sh
ros2 topic info /camera/front/depth/points --verbose
ros2 topic hz /camera/front/depth/points
ros2 topic echo /local_costmap/voxel_grid --once
```

只看到深度点但没有体素时，先检查相机 profile、TF、深度点云和局部 Costmap；不要
把 `/local_costmap/voxel_grid` 当作普通 PointCloud2 显示。

### 3.6 正常停止

先在终端 B 按一次 Ctrl+C。受管 RViz 会先收到独立关闭请求，随后 Navigation 执行有序
Lifecycle 关闭；再在终端 A 按一次 Ctrl+C 停止 Isaac。正常情况下不要连续按键。若下一次启动仍提示实例占用：

```bash
./scripts/diagnose.sh
./scripts/clean_runtime.sh --dry-run
```

确认诊断输出后再使用清理脚本的建议操作。

## 4. 初始位姿与 Reset

默认 `initial_pose_source:=auto` 会根据 `warehouse_new` 的 Manifest 与
`long_route_start_g1` 的派生标定自动发布初始位姿。每次 Reset 后，系统会等待新鲜的 `/clock`、
`/scan`、`/odom` 与稳定 `map -> odom`，然后恢复 Nav2。

若你在开发新地图且尚未完成标定，可显式选择人工位姿：

```bash
./scripts/run_ros.sh navigation \
  odometry_mode:=ideal \
  initial_pose_source:=rviz
```

随后在 RViz 使用 **2D Pose Estimate**。此模式下 Reset 后必须再次人工播种位姿。

不要对当前 `warehouse_new` 正常导航使用 `posegraph_calibration:=true`。

## 5. 仅看定位与相机

不启动 Nav2、只查看静态地图、TF 和初始位姿：

```bash
# 终端 A 仍使用第 3.2 节的 Isaac 命令
./scripts/run_ros.sh localization odometry_mode:=ideal
```

仅查看 Camera 时让 ROS 栈不启动 RViz，再开专用视图：

```bash
# 终端 B
./scripts/run_ros.sh localization \
  odometry_mode:=ideal \
  use_rviz:=false

# 终端 C
./scripts/run_camera_view.sh
```

`run_rviz.sh` 仅用于没有被受管启动 RViz 的会话；已有 RViz 时会因单实例锁拒绝
重复启动。

## 6. 建图新版本

只有修改场景、传感器、机器人或确实需要新地图时才重新建图。建图不能与
Localization/Navigation 并行。

```bash
# 终端 A
./scripts/run_isaac.sh \
  --environment-usd kujiale_0026_A_to_B_door_open.usd \
  --navigation-mode mapping \
  --mode ideal

# 终端 B；会启动 Mapping RViz 与受管键盘 Teleop
./scripts/run_ros.sh mapping odometry_mode:=ideal
```

Teleop 只在 Mapping 中可用，使用 W/A/S/D 或方向键；松键后 `0.18 s` 自动停车，
Space 立即停车，Q 退出。保存地图：

```bash
./scripts/save_map.sh <新地图版本>
```

它生成 OccupancyGrid、Pose Graph 与 Manifest 四件套，但新 Manifest 初始为未标定。
在将新地图用于 `initial_pose_source:=auto` 或正式实验前，必须完成 Map/USD 标定、
Manifest 更新和冷启动复核。具体流程见 [`calibration.md`](calibration.md)。

## 7. 自动化全屋长距离测试与报告

本节运行当前正式的 40 轮全屋长路线批次：静态种子 `7201–7220`、动态种子
`7301–7320`，每轮依次自动执行 `G2 → G3 → G4 → G5 → G1`。实验 runner 会在每轮开始时设置
seed、调用 `/simulation/reset`、等待 Nav2 恢复、发送五个 Nav2 Goal，并记录 GT、
Scan、深度图/点云、Costmap、碰撞、安全状态和 MCAP；它不负责启动 Isaac 或 Nav2。

正式配置是冻结输入，不要编辑后继续沿用正式结论：

```text
ros2_ws/src/robot_experiments/config/kujiale_static_long_range.yaml
ros2_ws/src/robot_experiments/config/kujiale_dynamic_long_range.yaml
ros2_ws/src/robot_experiments/config/kujiale_long_range_campaign.yaml
```

当前重设计的静态批次含中心区四个 RGB-D 低矮方块和两个低矮长条；动态批次含两组在 G2 受理后横穿
G1→G2 通道并停住的实体障碍。两批都使用 `warehouse_new`、`long_route_start_g1`、Ideal Odom 与
`rgbd_navigation` Camera。旧 `mapping_start` / G1–G8 的正式报告是历史证据，不能用于本布局。
由于 runner 会以 `/ground_truth/odom` 核验 Reset 和统计路线，下面所有长距离命令都
显式设置 `ISAAC_NAV__GROUND_TRUTH__ENABLED=true`；它仅发布评测数据，不参与导航 TF
或控制。
详细验收口径见
[`kujiale_long_range_navigation_test_plan.md`](kujiale_long_range_navigation_test_plan.md)。
静态/动态地图、S/G1、G2–G5、障碍位置和动态触发路线见
[`kujiale_long_route_map.md`](kujiale_long_route_map.md)。

### 7.1 正式批次前检查

正式结果会记录当前 Git 提交、工作区状态、地图/配置 SHA256 和每轮证据。先停止所有
旧 Isaac、ROS、RViz 与实验 runner，并确认工作区没有待提交修改：

```bash
cd "$PROJECT_ROOT"
./scripts/preflight.sh
./scripts/diagnose.sh
git status --short
```

最后一条应没有输出。执行 `git status --short` 时如果仍有修改，先提交或另建实验分支；
不要把混合配置的证据当作正式批次。完整批次的理论最大导航时间是静态 `20 × 600 s` 加
动态 `20 × 720 s`，即约 7 小时 20 分钟，另加 Reset、初始化和 MCAP 写盘时间。确保
磁盘空间充足，并在无人值守前关闭睡眠/自动锁屏。

为静态、动态和最终报告创建同一个正式批次 ID。该 ID 必须取批次开始时的本地时间：

```bash
export CAMPAIGN_ID="$(date +%Y%m%d-%H%M%S)"
export RUN_ROOT="$PROJECT_ROOT/data/experiment_runs/kujiale_long_route_${CAMPAIGN_ID}"
export REPORT_ROOT="$PROJECT_ROOT/data/reports/kujiale_long_route_${CAMPAIGN_ID}"
mkdir -p "$RUN_ROOT/static" "$RUN_ROOT/dynamic"
printf 'campaign_id=%s\n' "$CAMPAIGN_ID"
```

### 7.2 自动静态 20 轮候选批次（当前六障碍参数）

终端 A 启动无头 Isaac。`--dynamic-obstacles` 对静态批次同样是必须的：它启用冻结的
四个 `rgbd_low_box_*` 方块和两个 `rgbd_low_bar_*` 长条物理障碍，而不是启用动态轨迹。当前脚本使用
`kujiale_rgbd_low_obstacles_v6_draft_20260723_133702` 可编辑基线；它对应 `2026-07-23 13:37:02 +08:00` 捕获的四个方块和两个长条。结果是**静态候选**，不替代布局冻结
后的正式 20+20 验收。

```bash
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
```

终端 B 启动无交互 Navigation。等待 `Nav2 lifecycle activation completed` 后再启动
runner；不要同时打开 RViz 或 Teleop。

```bash
cd "$PROJECT_ROOT"
./scripts/run_ros.sh navigation \
  odometry_mode:=ideal \
  spawn_pose_name:=long_route_start_g1 \
  interactive:=false \
  use_rviz:=false
```

终端 C 启动 20 个静态 seed。该命令结束前不要关闭 A 或 B；可选参数是本次启动时的
`YYYYMMDD-HHMMSS` 批次 ID，省略时脚本自动生成。runner 完成后脚本会自动汇总静态证据并生成
离线中文报告：

```bash
cd "$PROJECT_ROOT"
./scripts/run_kujiale_static_20.sh
# 或固定本次批次 ID：
# ./scripts/run_kujiale_static_20.sh 20260723-120000
```

runner 会顺序运行，不会并发启动机器人。每轮证据位于
`data/experiment_runs/kujiale_long_route_static_<campaign_id>/kujiale_static_long_range/run-<序号>-seed-<seed>/`；
其 `run_summary.json`、`run_manifest.json`、`checksums.sha256` 和 `telemetry/*.mcap` 是报告输入。
完成后自动输出 `data/reports/kujiale_long_route_static_<campaign_id>/`，其中 `index.html` 可直接双击打开，
不需要 Web 服务器。全屋轨迹地图提供 **种子**（7201–7220）和**结果**筛选器：选择一个种子会只显示该轮
Ground Truth 轨迹，同时联动隐藏其他轮的明细行；悬停轨迹可查看该轮原始里程与通过/失败状态，点击热力图单元格或
“查看详情”可进入该轮证据页。报告同时生成 `report.pdf`、`report.md`、`benchmark.json`、`benchmark.csv`、
`data_dictionary.md`、`figures/`、`runs/` 和 `checksums.sha256`。

静态 runner 的某轮导航失败会写成该轮结果并继续；静态门槛不通过时，脚本仍会生成完整报告，
并以退出码 `2` 结束（表示“有结论但未通过”，不是报告生成失败）。该报告明确标注“动态 20 轮未运行”，
不能用于动态或完整 20+20 验收。启动前契约不匹配、Reset 隔离错误或中断则应停止批次，保存终端日志并
从新的 `CAMPAIGN_ID` 重跑。

静态 runner 正常结束后，先在终端 B 按 Ctrl+C（受管 RViz 会先关闭）等待有序关闭，再在终端 A 按 Ctrl+C。
不要直接切换 Isaac 障碍配置后复用旧进程。

### 7.3 自动动态批次（待 Pilot 后执行）

动态批次必须用新的 Isaac 进程，加载两组 G2 后触发、横穿 G1→G2 通道并 hold 的物理障碍配置。重新打开终端 A：

```bash
cd "$PROJECT_ROOT"
ISAAC_NAV__GROUND_TRUTH__ENABLED=true ./scripts/run_isaac.sh \
  --headless \
  --environment-usd kujiale_0026_A_to_B_door_open.usd \
  --navigation-mode localization \
  --mode ideal \
  --spawn-pose long_route_start_g1 \
  --camera-profile rgbd_navigation \
  --dynamic-obstacle-config isaac_sim/configs/experiments/kujiale_long_range_dynamic.yaml \
  --dynamic-obstacles
```

终端 B 使用与静态批次相同的无交互 Navigation：

```bash
cd "$PROJECT_ROOT"
./scripts/run_ros.sh navigation \
  odometry_mode:=ideal \
  spawn_pose_name:=long_route_start_g1 \
  interactive:=false \
  use_rviz:=false
```

等 Nav2 激活后，在终端 C 运行动态 seed：

```bash
cd "$PROJECT_ROOT"
./scripts/run_experiment.sh \
  ros2_ws/src/robot_experiments/config/kujiale_dynamic_long_range.yaml \
  "$RUN_ROOT/dynamic"
```

动态 runner 会核验 Isaac 已启用障碍、物理配置 SHA256 和障碍 ID；若不匹配会在发出
第一个 Goal 前失败，而不是把错误环境记录成测试数据。结束后按静态批次相同顺序关闭
终端 B、终端 A。

### 7.4 生成并核验自包含报告

两个 runner 都正常完成后，在新终端汇总全部 40 轮证据：

```bash
cd "$PROJECT_ROOT"
source ./scripts/setup_ros_env.sh
ros2 run robot_experiments kujiale_campaign \
  --run-directory "$RUN_ROOT/static" \
  --run-directory "$RUN_ROOT/dynamic" \
  --output-directory "$REPORT_ROOT"
```

汇总器要求静态和动态各恰好 20 个冻结 seed；它会验证每轮数据完整性、校验和、成功率
门槛和静态路径偏差，并根据证据自动写出结论。即使验收未通过，报告也会生成；此时命令
以退出码 `2` 结束，表示“测试结论未通过”，不是允许手工修改结论的错误。

输出目录为：

```text
data/reports/kujiale_long_route_<campaign_id>/
├── index.html
├── report.pdf
├── report.md
├── benchmark.json
├── benchmark.csv
├── data_dictionary.md
├── figures/
├── runs/
└── checksums.sha256
```

用浏览器直接打开 `index.html`；它不需要 Web 服务器。`benchmark.json` 是唯一机器可读
KPI 来源，HTML/PDF/Markdown/CSV 都由它和每轮证据生成。不要手工编辑报告、`runs/` 或
校验和来改变结论；若需要重生成同一批报告，只对原始 `$RUN_ROOT` 重新执行汇总命令。

运行证据与报告在 `data/experiment_runs/`、`data/reports/` 下，默认被 Git 忽略：HTML、
PDF、PNG、CSV、JSON、MCAP 和图像都不推送；应提交的是代码、冻结场景/校验规则和对
应文档。

## 8. 可视化单轮全屋长距离测试（Isaac GUI + RViz）

本节用于可视化回归：Isaac 保持 GUI、Navigation 保持 RViz，但由 experiment runner
自动按 `G2 → G3 → G4 → G5 → G1` 发送五个 Nav2 Goal（G1 是出生点也是最终回归点）。你不需要点击 **2D Goal Pose**、手动
触发动态障碍或运行 20 轮。静态和动态各只运行固定的一个 seed，适合观察 RGB-D 融合、
Costmap、MPPI、Collision Monitor 与实体障碍行为。

可视化单轮不写 MCAP、JSON、CSV、报告或项目结果目录；它只向正在运行的 ROS 图发送
自动路线。它不是正式 20+20 验收，不能替代正式结论。每次只启动一套 Isaac 和 ROS；
同一套会话中不要并行启动正式 `run_experiment.sh`。

### 8.1 启动约定

先正常停止旧的 Isaac/ROS 会话：

```bash
cd "$PROJECT_ROOT"
./scripts/diagnose.sh
```

终端 A 始终运行 Isaac GUI；终端 B 启动受管 `navigation.rviz`；终端 C 运行单轮
visual runner。C 启动后先自行 Reset 到 `long_route_start_g1`，再顺序发送 G2、G3、G4、G5、G1；在 RViz
只观察，不要再手动发送 Goal。`run_visual_route.sh` 不创建 `data/experiment_runs/` 或
`data/reports/` 下的任何文件。visual runner 同样需读取 Ground Truth 完成 Reset
一致性检查，因此终端 A 的命令也显式启用它；这不会产生评测证据或报告。

终端 A 的启动日志同样必须显示 `spawn=long_route_start_g1`。若显示
`mapping_start`，不要启动 B/C；先停止旧 Isaac，再复制本节完整命令重新启动。

### 8.2 静态可视化单轮

终端 A 启动带 RGB-D 低矮方块的 Isaac GUI。`--dynamic-obstacles` 对静态场景同样必须
启用，因为它负责实例化中心区四个 RGB-D 低矮方块和两个低矮长条；该配置没有运动轨迹：

```bash
cd "$PROJECT_ROOT"
ISAAC_NAV__GROUND_TRUTH__ENABLED=true ./scripts/run_isaac.sh \
  --environment-usd kujiale_0026_A_to_B_door_open.usd \
  --navigation-mode localization \
  --mode ideal \
  --spawn-pose long_route_start_g1 \
  --camera-profile rgbd_navigation \
  --dynamic-obstacle-config isaac_sim/configs/experiments/kujiale_long_range_static.yaml \
  --dynamic-obstacles
```

终端 B 启动 Navigation 和 RViz，等待 `Nav2 lifecycle activation completed`：

```bash
cd "$PROJECT_ROOT"
./scripts/run_ros.sh navigation odometry_mode:=ideal spawn_pose_name:=long_route_start_g1
```

#### 8.2.1 交互式布局与手动导航（可反复调整，不写实验输出）

六个静态障碍现在是用于布局确认的可编辑实体。保持上面的终端 A 和 B 运行，**不要启动**
`./scripts/run_visual_route.sh static`：自动 visual runner 会执行 Reset，故会把障碍恢复为
YAML 中的临时种子位置。

在 Isaac GUI 的 Stage 树展开 `/World/DynamicObstacles`，依次选择：

- `rgbd_low_box_west`
- `rgbd_low_box_center`
- `rgbd_low_box_east`
- `rgbd_low_box_north`
- `rgbd_low_bar_east`
- `rgbd_low_bar_north`

使用 Move 工具（`W`）只修改每个 Prim 的 **Translate X/Y**；不要旋转、缩放，也不要改变
`Z=0.08 m`。四个方块是 `0.30 × 0.30 × 0.16 m`，两个长条是 `0.60 × 0.30 × 0.16 m`；普通仿真 tick 不会覆盖 GUI 拖动，因此可在
机器人停止后再次拖动、再次测试任意次数。每次修改后在 RViz 选择 **2D Goal Pose**，按顺序发送
G2 `[0.80, 4.80, -160°]`、G3 `[-2.20, 3.25, -105°]`、G4 `[-3.00, -0.45, -68°]`、
G5 `[-2.20, -2.95, -42°]`、G1 `[0.45, -5.35, 90°]`。Nav2 仅在距离目标不超过 `0.20 m`
时完成 Action；GoalAngleCritic 同样在最后 `0.20 m` 才执行下一段朝向的预对准，避免尚未到点就背向来路。报告仍会用独立 Ground Truth `0.25 m` 门槛复核。
原狭窄通道停靠点已删除；G4 是原左侧厕所航点，只作为普通目标显示。观察深度点云、VoxelGrid、Local
Costmap、MPPI 路径和实际绕行。此交互式过程不创建 MCAP、JSON、CSV、报告或正式实验目录。

需要随时读取当前的精确 Map 坐标时，在终端 C 执行：

```bash
cd "$PROJECT_ROOT"
source ./scripts/setup_ros_env.sh
ros2 service call /experiment/obstacles/capture_layout std_srvs/srv/Trigger '{}'
```

返回 JSON 中的六个 `position` 已经是 `warehouse_new` 的 **Map** 坐标，而不是 Isaac USD 坐标。
每次保存会同步可编辑的 Isaac 基线、候选 campaign、地图示意和候选 `optimal_reference.json`，并保留一份带时间戳的草案快照；只有明确确认
“冻结布局”后，才最后一次重生成参考并更新正式测试文档。点击 Isaac Reset、调用
`/experiment/obstacles/reset`、或运行自动 visual runner 均会恢复最近保存的 YAML 基线；这是避免试验中的
手调位置被误当作已冻结参数的保护措施。

完整六障碍的最后 GUI 捕获是 `isaac_sim/configs/experiments/kujiale_static_layout_draft_20260723-133702.yaml`；它与
`kujiale_long_range_static.yaml`、静态 Pilot/visual/20 轮场景和候选理论参考同步。下一次静态 GUI 启动会加载这六个障碍；
它明确不是正式验收冻结。继续微调后再次调用 capture 服务，即可导出包含六个障碍的下一份快照。

终端 C 启动唯一静态 visual seed `7201`。runner 自动发送完整 G2、G3、G4、G5、G1 闭环路线，且不写
项目输出：

```bash
./scripts/run_visual_route.sh static
```

在 GUI 中观察低矮方块与机器人绕行；在 RViz 展开 **RGB-D Fusion**，按需启用
**Robot Front Camera**、**Depth PointCloud2** 和 **Marked Voxels (3D)**。同时观察全局/
局部路径、MPPI 最优轨迹和 Collision Monitor。仅 LiDAR Scan 看不到低矮方块并不表示
RGB-D 失败；应以深度点云和 VoxelGrid 为准。

### 8.3 动态可视化单轮

静态单轮结束后，先在终端 B 按 Ctrl+C（受管 RViz 会先关闭）等待有序关闭，再在终端 A 按 Ctrl+C 停止
Isaac。动态物理配置在 Isaac 启动时冻结，必须重新启动 GUI：

```bash
cd "$PROJECT_ROOT"
ISAAC_NAV__GROUND_TRUTH__ENABLED=true ./scripts/run_isaac.sh \
  --environment-usd kujiale_0026_A_to_B_door_open.usd \
  --navigation-mode localization \
  --mode ideal \
  --spawn-pose long_route_start_g1 \
  --camera-profile rgbd_navigation \
  --dynamic-obstacle-config isaac_sim/configs/experiments/kujiale_long_range_dynamic.yaml \
  --dynamic-obstacles
```

重新在终端 B 启动 Navigation/RViz：

```bash
cd "$PROJECT_ROOT"
./scripts/run_ros.sh navigation odometry_mode:=ideal spawn_pose_name:=long_route_start_g1
```

等待 Nav2 激活后，在终端 C 运行唯一动态 visual seed `7301`，同样不写项目输出：

```bash
./scripts/run_visual_route.sh dynamic
```

runner 在 G2 的 Goal 被 Nav2 接受后自动调用两组横穿 G1→G2 通道障碍的触发服务；不需要手工输入
服务命令。若希望额外查看状态，可另开终端 D：

```bash
source "$PROJECT_ROOT/scripts/setup_ros_env.sh"
ros2 topic echo /experiment/obstacles/state
```

在 GUI 中应看到两组实体从通道侧方进入 G1→G2 行进线、延迟触发并 hold；在 RViz 观察路径重规划、减速/等待、
MPPI 最优轨迹、Costmap 和 Collision Monitor。一次运行结束后，终端 C 直接退出，不会
留下项目证据目录。

### 8.4 重跑、截图与停止

要重跑任一 visual 场景，直接再次执行对应的终端 C 命令；runner 会为这一轮自动 Reset，
不需要发送 2D Goal Pose、手工 Reset 或准备输出目录。

每个关键航段可按需保存 Isaac GUI（机器人与实体障碍）和 RViz（路径、Costmap、RGB-D
Fusion/Collision Monitor）截图；是否保存截图完全由操作者决定。若保存，使用单独的
本地调试目录，不能与正式 `kujiale_long_route_<campaign_id>` 目录混用。

完成后在终端 B 按一次 Ctrl+C：脚本会先关闭受管 RViz，再执行 Nav2 的有序 lifecycle
关闭；随后停止终端 A。若异常退出，先运行 `./scripts/diagnose.sh`，再按
输出使用 `clean_runtime.sh --dry-run`。

## 9. 常用诊断

| 症状 | 首先执行 |
| --- | --- |
| 启动前失败、地图/Manifest 错误 | `./scripts/preflight.sh`，再看 [`troubleshooting.md`](troubleshooting.md) |
| 已有实例或锁 | `./scripts/diagnose.sh`，然后 `clean_runtime.sh --dry-run` |
| 看不到 Isaac Topic | `source ./scripts/setup_ros_env.sh`，检查 `/clock` 与 `/lidar/points_raw` |
| Nav2 不激活 | 等待 `/map`、`/clock`、`/scan`、`/odom` 和 `map -> odom`；不要提前发 Goal |
| RViz 无深度/体素 | 确认 `--camera-profile rgbd_navigation`、点云 Topic 与相机 TF |
| 地图或机器人位置明显不对 | 不要修改现有标定；用 `initial_pose_source:=rviz` 检查，随后按标定流程处理 |

完整排障命令和安全边界见 [`troubleshooting.md`](troubleshooting.md)。Topic、TF、QoS
和唯一发布者要求见 [`interfaces.md`](interfaces.md)。

## 10. 验证与提交

代码或配置改动后：

```bash
git diff --check
./scripts/test.sh
```

包含 Isaac/USD 行为的改动再运行：

```bash
./scripts/test.sh --with-isaac
```

ROS 集成测试使用 Domain 42；运行测试前停止真实 Isaac 会话，否则真实 `/clock` 会
与测试时钟冲突。不要提交 `data/reports/`、`data/experiment_runs/`、rosbag、MCAP
或本地 Isaac 资产。
