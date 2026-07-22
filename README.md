# Isaac Sim 6.0.1 + ROS 2 Jazzy：酷家乐 RGB-D 导航

本分支提供 Clearpath Jackal 在酷家乐室内场景中的 Isaac Sim 导航闭环：二维
LiDAR、前向 RGB-D、Nav2、RViz、确定性 Reset 与长距离实验。README、使用手册和
正式长距离场景使用的标准酷家乐组合是：

```text
场景:      kujiale_0026_A_to_B_door_open.usd
地图:      warehouse_new
出生点:    mapping_start
定位:      Ideal Odometry / 已标定 identity map -> odom
导航感知:  /scan + /camera/front/depth/points
```

`warehouse_new` 仅用于普通 Ideal Localization/Navigation；用 `realistic` 或
`posegraph_calibration:=true` 启动该地图会被 `run_ros.sh` 拒绝。Warehouse 的旧
地图、旧实验和历史调参不属于本分支的运行入口，见
[`docs/documentation_status.md`](docs/documentation_status.md)。

## 已记录的正式批次结果

下表是 2026-07-22 正式全屋批次 `kujiale_long_route_20260722-171828` 的自动汇总
结果，不是对当前工作树或后续参数改动的重新验收声明。原始报告是本地忽略工件；当前
可执行的验收规格、阈值与复跑步骤以使用手册和测试方案为准。

| 项目 | 结果 |
| --- | --- |
| 静态严格成功 | `20/20 (100%)` |
| 动态严格成功 | `18/20 (90%)` |
| 物理无碰撞 | 静态 `20/20 (100%)`、动态 `19/20 (95%)` |
| 静态最大路径偏差 | `19.2868%`，低于 `20%` 门槛 |

完整路线、验收口径、失败边界和报告目录结构见
[`docs/kujiale_long_range_navigation_test_plan.md`](docs/kujiale_long_range_navigation_test_plan.md)；
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

`preflight.sh` 必须显示 `map baseline: warehouse_new (integrity verified)` 和
`preflight: PASS`。它也会检查 Git LFS、地图 Manifest、GPU、Isaac、ROS 与已构建
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
  --camera-profile rgbd_navigation
```

终端 B 启动 Navigation、Map Server 与受管 RViz：

```bash
cd /你的实际路径/Isaac_Sim_ROS2_Nav
./scripts/run_ros.sh navigation odometry_mode:=ideal
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

静态/动态全屋长距离测试的 `warehouse_new` 地图、S 与 G1–G8 航点、静态方块和三条
动态障碍触发路线见 [`docs/kujiale_long_route_map.md`](docs/kujiale_long_route_map.md)。

## RGB-D 感知边界

`--camera-profile rgbd_navigation` 会发布：

```text
/camera/front/image_raw
/camera/front/camera_info
/camera/front/depth/points
```

深度点云被全局和局部 Costmap 的独立 `depth_voxel_layer` 使用；RViz 的
**RGB-D Fusion** 分组显示局部 `/local_costmap/voxel_grid`。Collision Monitor
仍只使用二维 `/scan`，RGB-D 不进入 SLAM、EKF 或 Collision Monitor。

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

正式静态、动态场景配置位于：

```text
ros2_ws/src/robot_experiments/config/kujiale_static_long_range.yaml
ros2_ws/src/robot_experiments/config/kujiale_dynamic_long_range.yaml
ros2_ws/src/robot_experiments/config/kujiale_long_range_campaign.yaml
```

GUI + RViz 的单轮可视化回归分别使用
`kujiale_static_visual.yaml` 与 `kujiale_dynamic_visual.yaml`：runner 自动发送完整
G1–G8 路线，动态场景也自动触发障碍。使用 `./scripts/run_visual_route.sh static|dynamic`
启动；它不生成项目实验输出，二者均不计入正式 20+20 结果。

运行证据与报告写入 `data/experiment_runs/` 和 `data/reports/`。这些目录中的
HTML、PDF、PNG、CSV、JSON、MCAP 和图像是本地生成物，默认不推送到 Git；受版本
控制的是生成器、场景、校验规则和文档。

自动启动静态 20 轮、动态 20 轮、汇总并核验自包含报告的完整命令见
[`docs/user_manual.md`](docs/user_manual.md)。

需要 GUI + RViz 的可视化回归时，同一手册提供静态/动态各一轮的自动 G1–G8 路线；
无需手动点选目标或手动触发障碍。

## 文档入口

| 文档 | 用途 |
| --- | --- |
| [`docs/user_manual.md`](docs/user_manual.md) | 当前可执行使用手册。 |
| [`docs/interfaces.md`](docs/interfaces.md) | Topic、TF、模式配对和所有权契约。 |
| [`docs/troubleshooting.md`](docs/troubleshooting.md) | 启动、地图、RViz、QoS、Reset 和清理排障。 |
| [`docs/verification.md`](docs/verification.md) | 当前验收和历史证据边界。 |
| [`docs/documentation_status.md`](docs/documentation_status.md) | 当前文档与历史文档的职责边界。 |
| [`docs/repository_index.md`](docs/repository_index.md) | 文件职责和代码修改入口。 |

## 验证

```bash
./scripts/test.sh
./scripts/test.sh --with-isaac
```

运行 ROS 集成测试时需停止同一 Domain 42 中的 Isaac 仿真；否则真实 `/clock` 会与
测试夹具时钟冲突。代码改动后至少运行 `git diff --check` 和相应测试。
