# 酷家乐 RGB-D 导航使用手册

> 当前可执行手册，适用分支：`codex/kujiale-long-range-navigation-test`。
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
| 出生点 | `mapping_start` |
| USD 出生位姿 | `[2.9, -0.2, 0.0635]`，yaw `180°` |
| Map 初始位姿 | `[0, 0]`，yaw `0°` |
| 里程计 | Isaac Ideal `/odom` 和 `odom -> base_link` |
| Map TF | 已标定 identity `map -> odom` |
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

预检成功时应包含：

```text
map baseline: warehouse_new (integrity verified)
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
  --camera-profile rgbd_navigation
```

等待：

```text
Isaac navigation simulation ready
```

GUI 会使用 Jackal 的第三人称相机。无头自动运行才使用 `--headless`；手动点击
目标需要 RViz，因此日常人工测试使用 GUI Isaac + RViz 更直接。

### 3.3 终端 B：启动 Navigation 与 RViz

```bash
cd "$PROJECT_ROOT"
./scripts/run_ros.sh navigation odometry_mode:=ideal
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
`0.25 m`，二者不是同一个概念。

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

深度点云同时进入全局和局部 `depth_voxel_layer`；默认 RViz 仅渲染
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

先在终端 B 按一次 Ctrl+C，等待 Navigation 的有序 Lifecycle 关闭完成；再在终端 A
按一次 Ctrl+C 停止 Isaac。正常情况下不要连续按键。若下一次启动仍提示实例占用：

```bash
./scripts/diagnose.sh
./scripts/clean_runtime.sh --dry-run
```

确认诊断输出后再使用清理脚本的建议操作。

## 4. 初始位姿与 Reset

默认 `initial_pose_source:=auto` 会根据 `warehouse_new` 的 Manifest 与
`mapping_start` 标定自动发布初始位姿。每次 Reset 后，系统会等待新鲜的 `/clock`、
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
`7301–7320`，每轮依次自动执行 `G1` 到 `G8`。实验 runner 会在每轮开始时设置
seed、调用 `/simulation/reset`、等待 Nav2 恢复、发送八个 Nav2 Goal，并记录 GT、
Scan、深度图/点云、Costmap、碰撞、安全状态和 MCAP；它不负责启动 Isaac 或 Nav2。

正式配置是冻结输入，不要编辑后继续沿用正式结论：

```text
ros2_ws/src/robot_experiments/config/kujiale_static_long_range.yaml
ros2_ws/src/robot_experiments/config/kujiale_dynamic_long_range.yaml
ros2_ws/src/robot_experiments/config/kujiale_long_range_campaign.yaml
```

静态批次含 RGB-D 低矮方块；动态批次含三个由 G1、G2、G6 触发的穿行障碍。两批都
使用 `warehouse_new`、`mapping_start`、Ideal Odom 与 `rgbd_navigation` Camera。
详细验收口径见
[`kujiale_long_range_navigation_test_plan.md`](kujiale_long_range_navigation_test_plan.md)。

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

### 7.2 自动静态批次

终端 A 启动无头 Isaac。`--dynamic-obstacles` 对静态批次同样是必须的：它启用冻结的
`rgbd_low_box` 物理障碍，而不是启用动态轨迹。

```bash
cd "$PROJECT_ROOT"
./scripts/run_isaac.sh \
  --headless \
  --environment-usd kujiale_0026_A_to_B_door_open.usd \
  --navigation-mode localization \
  --mode ideal \
  --spawn-pose mapping_start \
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
  interactive:=false \
  use_rviz:=false
```

终端 C 启动 20 个静态 seed。该命令结束前不要关闭 A 或 B：

```bash
cd "$PROJECT_ROOT"
./scripts/run_experiment.sh \
  ros2_ws/src/robot_experiments/config/kujiale_static_long_range.yaml \
  "$RUN_ROOT/static"
```

runner 会顺序运行，不会并发启动机器人。每轮证据位于
`$RUN_ROOT/static/kujiale_static_long_range/run-<序号>-seed-<seed>/`；其
`run_summary.json`、`run_manifest.json`、`checksums.sha256` 和 `telemetry/*.mcap`
是后续报告的输入。runner 的某轮导航失败会写成该轮结果并继续；启动前契约不匹配、
Reset 隔离错误或中断则应停止批次，保存终端日志并排障后从新的 `CAMPAIGN_ID` 重跑。

静态 runner 正常结束后，先在终端 B 按 Ctrl+C 等待有序关闭，再在终端 A 按 Ctrl+C。
不要直接切换 Isaac 障碍配置后复用旧进程。

### 7.3 自动动态批次

动态批次必须用新的 Isaac 进程，加载三个触发式动态障碍的物理配置。重新打开终端 A：

```bash
cd "$PROJECT_ROOT"
./scripts/run_isaac.sh \
  --headless \
  --environment-usd kujiale_0026_A_to_B_door_open.usd \
  --navigation-mode localization \
  --mode ideal \
  --spawn-pose mapping_start \
  --camera-profile rgbd_navigation \
  --dynamic-obstacle-config isaac_sim/configs/experiments/kujiale_long_range_dynamic.yaml \
  --dynamic-obstacles
```

终端 B 使用与静态批次相同的无交互 Navigation：

```bash
cd "$PROJECT_ROOT"
./scripts/run_ros.sh navigation \
  odometry_mode:=ideal \
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

## 8. 人工可视化全屋长距离测试（Isaac GUI + RViz）

本节用于人工观察真实 GUI、RViz 路径、RGB-D 融合、Costmap 和避障动作。它与第 7 节
的正式自动化验收互补，但不能替代后者：人工测试不会自动逐轮 Reset、记录完整 MCAP
证据或生成可验收的 `benchmark.json`。不要把人工截图或手工记录混入正式
`$RUN_ROOT` 后再运行汇总器。

每次只启动一套 Isaac 和 ROS。使用本节时不要启动 `run_experiment.sh`，它会抢占
`/simulation/reset` 并自行发送 Nav2 Goal。

### 8.1 固定人工路线

从 `mapping_start`（S）依次通过下表的 G1 至 G8。使用 RViz 的 **2D Goal Pose**：在
Map 显示中放置目标，拖动箭头使其与表中 yaw 一致。若地图缩放后难以精确点击，可先在
RViz 状态栏确认鼠标 Map 坐标；目标容差为 `0.25 m`、朝向容差为 `10°`。

| 顺序 | Map 坐标 `[x, y, yaw]` | 单段人工观察重点 |
| --- | --- | --- |
| S | `[0.00, 0.00, 0°]` | 起点与 `map -> odom -> base_link` 对齐。 |
| G1 | `[0.80, 4.80, -135°]` | 首段门洞与静态低矮方块绕行。 |
| G2 | `[-3.45, 3.90, 0°]` | 顶部走廊与动态 D2 穿行。 |
| G3 | `[-4.05, 1.15, 0°]` | 左侧房间进入/退出。 |
| G4 | `[-3.25, -0.45, 0°]` | 左侧房间航点；按普通房间目标处理。 |
| G5 | `[-2.50, -3.35, 90°]` | 下方转角与恢复动作。 |
| G6 | `[0.65, -4.25, -90°]` | 底部通道与动态 D3 穿行。 |
| G7 | `[0.45, -5.35, 90°]` | 最下方房间朝向收敛。 |
| G8 | `[0.00, 0.00, 0°]` | 返回起点、终点静止。 |

每个目标完成后，等待机器人静止至少 1 秒，再发送下一目标。若 Nav2 失败、发生碰撞、
Collision Monitor 长时间 Stop 或路径持续振荡，记录发生的航段、现象和截图；不要通过
手动推车、Teleop 或连续重发同一目标掩盖失败。

### 8.2 人工静态长距离测试

终端 A 以 GUI 启动静态低矮方块。这里仍需 `--dynamic-obstacles`，因为它负责把
`rgbd_low_box` 实体放入场景；该配置没有运动轨迹。

```bash
cd "$PROJECT_ROOT"
./scripts/run_isaac.sh \
  --environment-usd kujiale_0026_A_to_B_door_open.usd \
  --navigation-mode localization \
  --mode ideal \
  --spawn-pose mapping_start \
  --camera-profile rgbd_navigation \
  --dynamic-obstacle-config isaac_sim/configs/experiments/kujiale_long_range_static.yaml \
  --dynamic-obstacles
```

终端 B 启动 Navigation 和受管 RViz：

```bash
cd "$PROJECT_ROOT"
./scripts/run_ros.sh navigation odometry_mode:=ideal
```

等待 `Nav2 lifecycle activation completed`。在 Isaac GUI 中确认低矮方块位于 S 到 G1
区域；在 RViz 展开 **RGB-D Fusion**，启用 **Robot Front Camera**、
**Depth PointCloud2** 和 **Marked Voxels (3D)**（按需），再按第 8.1 节依次手动发送
G1–G8。

人工静态通过的可视化观察应同时满足：低矮方块可在深度点云/体素中出现、全局或局部
路径绕开其占地、机器人没有碰撞、每个目标由 Nav2 成功完成。仅 LiDAR Scan 看不到该
方块并不表示 RGB-D 失败；应检查 Depth PointCloud2 和 VoxelGrid。

### 8.3 人工动态长距离测试

先正常关闭静态的终端 B，再关闭终端 A。动态配置在 Isaac 启动时冻结，不能在原
Isaac 进程中切换。随后使用 GUI 启动三个触发式障碍：

```bash
cd "$PROJECT_ROOT"
./scripts/run_isaac.sh \
  --environment-usd kujiale_0026_A_to_B_door_open.usd \
  --navigation-mode localization \
  --mode ideal \
  --spawn-pose mapping_start \
  --camera-profile rgbd_navigation \
  --dynamic-obstacle-config isaac_sim/configs/experiments/kujiale_long_range_dynamic.yaml \
  --dynamic-obstacles
```

终端 B 启动可视化 Navigation：

```bash
cd "$PROJECT_ROOT"
./scripts/run_ros.sh navigation odometry_mode:=ideal
```

终端 C 用于确认触发服务，并在人工目标被 Nav2 接受后触发对应障碍：

```bash
cd "$PROJECT_ROOT"
source ./scripts/setup_ros_env.sh
ros2 service list | rg '^/experiment/obstacles/(G1|G2|G6)/trigger$'
```

若要持续观察 `waiting`、`active`、`retired` 状态，可另开终端 D：

```bash
source "$PROJECT_ROOT/scripts/setup_ros_env.sh"
ros2 topic echo /experiment/obstacles/state
```

在 RViz 发送 G1 后，立刻在终端 C 运行以下命令；随后等待 G1 成功。对 G2 和
G6 重复同样操作。服务只允许每个触发组在一次 Reset 后成功一次，响应中的
`activated` 应列出对应障碍 ID。

```bash
# G1 已被 Nav2 接受后：D1（central_crossing）在 S -> G1 路段穿行
ros2 service call /experiment/obstacles/G1/trigger std_srvs/srv/Trigger '{}'

# G2 已被 Nav2 接受后：D2（north_crossing）在 G1 -> G2 路段穿行
ros2 service call /experiment/obstacles/G2/trigger std_srvs/srv/Trigger '{}'

# G6 已被 Nav2 接受后：D3（south_crossing）在 G5 -> G6 路段穿行
ros2 service call /experiment/obstacles/G6/trigger std_srvs/srv/Trigger '{}'
```

完整人工顺序为：发送 G1 并触发 G1 服务，等待完成；发送 G2 并触发 G2 服务，等待
完成；依次发送 G3、G4、G5；发送 G6 并触发 G6 服务；最后发送 G7、G8。观察
`/experiment/obstacles/state` 从 `waiting` 到 `active`，并在运动完成后到 `retired`。
在 GUI 中应能看到实体移动/消失；在 RViz 中同时观察全局/局部路径、MPPI 最优轨迹、
Collision Monitor 和机器人轨迹，确认减速、等待、绕行或恢复行为合理且无物理碰撞。

### 8.4 手工重测、截图与停止

同一静态或动态场景需要从 S 重测时，先取消当前 Nav2 Goal；然后在终端 C 设置确定性
seed 并请求事务式 Reset：

```bash
ros2 param set /isaac_navigation_sim reset_seed 7301
ros2 service call /simulation/reset std_srvs/srv/Trigger '{}'
ros2 topic echo /simulation/localization_seeded --once
```

静态重测可使用 `7201`，动态重测使用 `7301`；也可以选择同一正式种子范围内的其他值。
Reset 后等待新的定位完成和 Navigation 2 Safe 面板恢复激活，再从 G1 开始。动态障碍的
服务触发状态会随 Reset 清除，因此必须重新调用 G1/G2/G6 的服务。

每个关键航段建议保存两张截图：Isaac GUI（机器人与实体障碍）和 RViz（路径、Costmap、
RGB-D Fusion/Collision Monitor）。这些人工证据可放入一个单独的本地目录，例如
`data/reports/manual_<时间戳>/`，用于调试记录；它不应与正式自动批次报告混用。

停止顺序仍是先在终端 B 按 Ctrl+C，等待 Nav2 有序关闭，再在终端 A 按 Ctrl+C 停止
Isaac；最后停止终端 C 的 `ros2 topic echo`。若异常退出，先执行
`./scripts/diagnose.sh`，再按其输出使用 `clean_runtime.sh --dry-run`。

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
