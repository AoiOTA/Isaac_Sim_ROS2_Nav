# Isaac Sim 6.0.1 + ROS 2 Jazzy：酷家乐 RGB-D 导航

## 演示视频（4×速，直接预览）

### 动态避障演示
<video controls width="960" preload="metadata">
  <source src="docs/videos/动态避障演示_4x_10MB.mp4" type="video/mp4" />
  你的浏览器不支持内嵌视频，请点击下方链接查看。
</video>

### 静态避障演示
<video controls width="960" preload="metadata">
  <source src="docs/videos/静态避障演示_4x_10MB.mp4" type="video/mp4" />
  你的浏览器不支持内嵌视频，请点击下方链接查看。
</video>

---

本仓库提供 Clearpath Jackal 在酷家乐室内场景中的 Isaac Sim 导航闭环：2D LiDAR、前向 RGB-D、Nav2、RViz、确定性 Reset，以及可复核的全屋 4×20 鲁棒性实验。

当前默认运行组合：

```text
场景:      kujiale_0026_A_to_B_door_open.usd
地图:      warehouse_new
出生点:    long_route_start_g1（map: [0.45, -5.35, 90°]）
定位:      Ideal Odometry；按出生点对齐 map -> odom
导航输入:  /scan + /camera/front/depth/points
```

`warehouse_new` 只支持 Ideal Localization/Navigation；`realistic` 和 `posegraph_calibration:=true` 会被启动契约拒绝。

## 当前正式结果：4×20 外观鲁棒性实验

当前正式实验包含四组、每组20轮：静态基准、静态＋外观变化、动态基准、动态＋外观变化。外观扰动通过匿名 USD Session Layer 改变光照和材质颜色，不修改原始 USD、几何、碰撞、地图或动态障碍运动学。

正式批次 `20260725-210035` 已完成，根报告为 `complete=true`、`passed=true`、`issues=[]`：

| 组别 | 严格成功 | 物理无碰撞 | 结论 |
| --- | ---: | ---: | --- |
| 静态基准 | 20/20 | 20/20 | 最大路径偏差 10.1687% |
| 静态＋外观 | 20/20 | 20/20 | 最大路径偏差 10.1442% |
| 动态基准 | 19/20 | 20/20 | 通过 18/20 门槛 |
| 动态＋外观 | 19/20 | 20/20 | 通过 18/20 门槛 |

结果仅适用于当前冻结的地图、场景、Nav2 profile、外观矩阵、actor 配置和验收规则。完整证据边界见 [验证台账](docs/verification.md)，执行问题与恢复规则见 [4×20 执行复盘](docs/kujiale_4x20_execution_lessons.md)。

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

脚本默认使用 ROS 2 Jazzy、`ROS_DOMAIN_ID=42` 和 `rmw_fastrtps_cpp`。需要直接执行 `ros2` 命令时，先运行：

```bash
source "$PROJECT_ROOT/scripts/setup_ros_env.sh"
```

## 手动导航（Isaac GUI + RViz）

先确认没有另一套项目 Isaac 或 ROS 会话。若提示锁被占用，先执行 `./scripts/diagnose.sh`；不要使用宽泛的 `pkill` 或手动删除锁文件。

终端 A 启动 Isaac GUI：

```bash
cd "$PROJECT_ROOT"
./scripts/run_isaac.sh \
  --environment-usd kujiale_0026_A_to_B_door_open.usd \
  --navigation-mode localization \
  --mode ideal \
  --spawn-pose long_route_start_g1 \
  --camera-profile rgbd_navigation
```

终端 B 启动 Navigation 和 RViz：

```bash
cd "$PROJECT_ROOT"
./scripts/run_ros.sh navigation \
  odometry_mode:=ideal \
  spawn_pose_name:=long_route_start_g1 \
  nav2_profile:=stable
```

等待 `Nav2 lifecycle activation completed` 后，在 RViz 中使用 **2D Goal Pose** 发布目标。完整的 Reset、RGB-D 显示和人工导航步骤见 [用户手册](docs/user_manual.md)。

## 正式 4×20 批量运行

一条命令会构建工作区，依次运行静态 pilot＋40轮、生成静态 `2×20` 报告、受控停止静态栈、运行动态 pilot＋40轮、生成动态 `2×20` 报告，最后生成同一批次的总 `4×20` 报告：

```bash
cd "$PROJECT_ROOT"
./scripts/run_kujiale_4x20_all.sh
```

省略批次 ID 时会自动生成 `YYYYMMDD-HHMMSS`。指定 ID 或在相同代码、配置下断点续跑：

```bash
./scripts/run_kujiale_4x20_all.sh 20260726-120000
./scripts/run_kujiale_4x20_all.sh 20260726-120000 --resume --skip-build
```

若只需要在同一配置下续跑动态阶段：

```bash
./scripts/run_kujiale_4x20_all.sh <CAMPAIGN_ID> --dynamic-only --resume --skip-build
```

如果修改了动态代码、Nav2 参数、actor 配置或验收规则，不能对旧动态证据直接使用 `--resume`；应使用新批次 ID 运行 `--dynamic-only`，或严格按 [执行复盘](docs/kujiale_4x20_execution_lessons.md) 替换同一 ID 下的全部动态证据。

报告目录为 `data/reports/kujiale_4x20_<campaign_id>/`。每份报告包含 HTML、PDF、Markdown、PNG、CSV、JSON 和证据索引；报告即使未通过也会生成，退出码 `2` 表示批次完成但未通过门槛或证据不完整。HTML 可按实验组、seed、外观配置、动态变体和结果筛选每轮路径：静态图叠加六个静态障碍，动态图叠加本轮实际触发 actor 的轨迹、起终点和方向。

完整实验矩阵、外观定义、报告与门槛见 [4×20 运行手册](docs/kujiale_4x20_appearance_benchmark_plan.md)。

## 全屋单轮可视化（不计入正式证据）

两种单轮模式都会自动发送 `G2 → G3 → G4 → G5 → G1`。运行前确保没有其他项目栈；从静态切换到动态时，先按 Ctrl+C 停止静态 ROS，再停止静态 Isaac。

### 静态单轮

```bash
# 终端 A：Isaac GUI + 六个静态 RGB-D 障碍
cd "$PROJECT_ROOT"
./scripts/run_kujiale_4x20_isaac.sh static

# 终端 B：Navigation + RViz
cd "$PROJECT_ROOT"
./scripts/run_ros.sh navigation \
  odometry_mode:=ideal \
  spawn_pose_name:=long_route_start_g1 \
  nav2_profile:=stable

# 终端 C：自动发送全屋航点
cd "$PROJECT_ROOT"
./scripts/run_visual_route.sh static
```

### 动态单轮

```bash
# 终端 A：Isaac GUI + 三阶段动态 actor
cd "$PROJECT_ROOT"
./scripts/run_kujiale_dynamic_isaac.sh

# 终端 B：Navigation + RViz
cd "$PROJECT_ROOT"
./scripts/run_ros.sh navigation \
  odometry_mode:=ideal \
  spawn_pose_name:=long_route_start_g1 \
  nav2_profile:=dynamic_avoidance

# 终端 C：自动发送全屋航点并触发三阶段 actor
cd "$PROJECT_ROOT"
./scripts/run_kujiale_three_stage_visual.sh full --variant 1 --seed 7501
```

动态单轮默认不写正式证据；仅在需要调试记录时，在终端 C 末尾加 `--record`。更详细的观察项、聚焦单段和停止顺序见 [用户手册第8节](docs/user_manual.md#8-可视化单轮全屋长距离测试isaac-gui--rviz)。

## RGB-D 感知边界

`rgbd_navigation` 发布：

```text
/camera/front/image_raw
/camera/front/camera_info
/camera/front/depth/points
```

`stable` profile 在 Local 和 Global Costmap 使用 `depth_voxel_layer`，用于低矮静态障碍；`dynamic_avoidance` 在 Local Costmap 使用时空 STVL，Global Costmap 仅使用静态图和 `/scan`，避免移动 actor 留下全局残影。Collision Monitor 始终只订阅 `/scan`；RGB-D 不参与 SLAM、EKF 或 Collision Monitor。

## 常用操作

| 目标 | 命令 |
| --- | --- |
| 仅启动定位与 TF | `./scripts/run_ros.sh localization odometry_mode:=ideal` |
| 从零建图 | Isaac：`--navigation-mode mapping --mode ideal`；ROS：`./scripts/run_ros.sh mapping odometry_mode:=ideal` |
| 保存新地图 | `./scripts/save_map.sh <新版本名>`，完成标定后才能使用 `initial_pose_source:=auto` |
| 查看受管进程和锁 | `./scripts/diagnose.sh` |
| 安全预览清理 | `./scripts/clean_runtime.sh --dry-run` |

## 文档入口

| 目标 | 文档 |
| --- | --- |
| 执行正式实验、理解外观矩阵与报告 | [4×20 运行手册](docs/kujiale_4x20_appearance_benchmark_plan.md) |
| 排查 pilot、supervisor、续跑或动态复测 | [4×20 执行复盘](docs/kujiale_4x20_execution_lessons.md) |
| 查看地图、航点、静态障碍和动态 actor 路线 | [全屋路线地图](docs/kujiale_long_route_map.md) |
| 查看运行契约、Topic、TF、QoS 和所有权 | [接口文档](docs/interfaces.md) |
| 诊断启动、锁、DDS、RViz、Reset 或 Nav2 问题 | [排障手册](docs/troubleshooting.md) |
| 查看正式结果与历史结果边界 | [验证台账](docs/verification.md) |
| 查看文档的当前/历史适用范围 | [文档状态](docs/documentation_status.md) |

## 验证

```bash
./scripts/test.sh
./scripts/test.sh --with-isaac
git diff --check
```

运行 ROS 集成测试前，停止同一 ROS Domain 42 中的 Isaac 仿真，避免真实 `/clock` 与测试夹具冲突。
