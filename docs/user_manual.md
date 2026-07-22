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

## 7. 自动实验与报告

当前正式长路线配置：

```text
ros2_ws/src/robot_experiments/config/kujiale_static_long_range.yaml
ros2_ws/src/robot_experiments/config/kujiale_dynamic_long_range.yaml
ros2_ws/src/robot_experiments/config/kujiale_long_range_campaign.yaml
```

静态、动态实验使用固定的 G1–G8 路线、`warehouse_new`、Ideal Odom 与 RGB-D。
正式批次的完整要求和结果见
[`kujiale_long_range_navigation_test_plan.md`](kujiale_long_range_navigation_test_plan.md)。

单个场景的 runner 入口格式为：

```bash
./scripts/run_experiment.sh <场景 YAML> <输出目录>
```

运行前必须由 Isaac 启动匹配场景、出生点、相机 profile 和障碍配置；不要把历史
Warehouse 场景文件与当前酷家乐地图混用。报告生成物位于 `data/experiment_runs/` 与
`data/reports/`，默认被 Git 忽略：HTML、PDF、PNG、CSV、JSON、MCAP 均不推送，
但报告生成器和场景配置会提交。

## 8. 常用诊断

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

## 9. 验证与提交

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
