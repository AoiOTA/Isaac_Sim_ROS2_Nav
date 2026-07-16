# Isaac Sim 6.0.1 + ROS 2 Jazzy Navigation

这是一个面向 Clearpath Jackal 的二维 SLAM、定位、Nav2 导航与可重复实验工程。仿真运行在 Isaac Sim 6.0.1，ROS 侧使用 Jazzy、SLAM Toolbox、robot_localization、SmacPlanner2D、MPPI、Velocity Smoother 和 Collision Monitor。

完整设计和分阶段验收标准见 [`plan.md`](plan.md)。本 README 只保留可执行入口、运行约束和交付状态。

> 当前交付状态：Stage、传感器、Ideal/Realistic 里程计、SLAM、事务式 Reset/Lifecycle 恢复、三套 RViz、Mapping 安全 Teleop、动态障碍、Nav2 和实验框架均已实现。2026-07-17 的 Realistic 同环境远距离验收达到静态 20/20、动态 19/20，且两批均无物理碰撞；同日 Ideal 前进优先复杂长路线验收达到静态 3/3、动态 3/3，每轮约 50 m、6/6 航点、0 恢复、0 碰撞且导航指令无倒车。`warehouse_v1` 完整地图基线随仓库发布，其中大 Pose Graph 使用 Git LFS。多地图/多布局泛化矩阵和真实 changed-region 的增量建图 30% 改善基准仍未执行。详细证据与边界见 [`docs/verification.md`](docs/verification.md)。

## 文档导航

第一次使用建议先看前两项：

| 文档 | 适合什么时候看 |
| --- | --- |
| [`docs/user_manual.md`](docs/user_manual.md) | 不熟悉项目时从这里开始；按步骤完成安装、导航、建图、实验和排障。 |
| [`docs/repository_index.md`](docs/repository_index.md) | 想理解代码结构或准备修改文件时；逐项解释全部 Git 跟踪文件。 |
| [`docs/interfaces.md`](docs/interfaces.md) | 排查 Topic、QoS、TF、Reset、模式配对或 Nav2 激活问题时。 |
| [`docs/troubleshooting.md`](docs/troubleshooting.md) | 运行异常时按症状执行安全诊断和恢复，不盲目杀进程或删除 SHM。 |
| [`docs/calibration.md`](docs/calibration.md) | 修改地图、出生点、传感器外参或动态障碍坐标时。 |
| [`docs/verification.md`](docs/verification.md) | 判断某项能力是否真正验证，以及了解当前未验收边界时。 |
| [`docs/development.md`](docs/development.md) | 开发、测试、调试和准备 Git 提交时。 |
| [`docs/rviz_workflow_upgrade_plan.md`](docs/rviz_workflow_upgrade_plan.md) | 回溯本轮 RViz/Teleop/Lifecycle/性能升级的冻结设计和完成状态时。 |
| [`plan.md`](plan.md) | 需要完整设计背景、技术选型、指标公式和最终验收目标时。 |

## 系统契约

- USD/PhysX 是唯一物理模型；URDF/Xacro 只用于 ROS RobotModel、结构描述和真机迁移。
- 主 TF 树固定为 `map → odom → base_link`，不发布 ROS `world` frame。
- Mapping 与 Localization 严格互斥，因为二者都拥有 `map → odom`。
- Localization/Navigation 中由 `nav2_map_server` 唯一发布保存的静态 `/map`；
  SLAM Toolbox 只负责定位和 `map → odom`，其扫描栅格图重映射到
  `/slam_toolbox/map`，不得作为 Nav2 静态地图。
- Ideal 模式由 Isaac 唯一发布 `/odom` 和 `odom → base_link`；Realistic 模式关闭 Isaac odom，由 Wheel Odom + IMU + EKF 唯一发布。
- Ground Truth 只进入记录和指标模块，不进入 SLAM、EKF、Nav2 或控制器。
- 感知基线为 `/lidar/points_raw → pointcloud_to_laserscan → /scan`；默认不启用 Self Filter、VoxelGrid 或 Nav2 Voxel Layer。
- 当前动态避障是基于二维 `/scan` 的反应式避障，不表示三维路径规划或高度可通行性推理。

完整 Topic、QoS、TF 所有权和 Reset 契约见 [`docs/interfaces.md`](docs/interfaces.md)。

## 已验证环境

当前工作站的基线为：

- Isaac Sim `6.0.1.0`：`/home/lyb/miniconda3/envs/isaacsim`；
- ROS 2 Jazzy：`/opt/ros/jazzy`；
- Fast DDS：`rmw_fastrtps_cpp`；
- NVIDIA GeForce RTX 4090；
- 官方资产根：`/home/lyb/isaacsim_assets/Assets/Isaac/6.0`。

所有路径都可以通过环境变量覆盖，脚本不会修改官方 Warehouse 或 Jackal 文件。

## 首次设置

```bash
git lfs install
git lfs pull
./scripts/preflight.sh
./scripts/import_assets.sh
./scripts/build_ros2.sh
```

资产导入只在本地复制 Jackal 的最小运行依赖，并校验来源和 SHA256。NVIDIA 二进制资产被 Git 忽略；仓库只管理项目自有的 USD overlay、manifest 和导入工具。

默认使用：

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

两个终端必须使用相同的值。

## 运行模式

Isaac 的 `--navigation-mode` 描述仿真 Reset 行为；ROS 的第一个参数选择实际算法栈。两端必须按下表配对：

| ROS 操作 | Isaac `--navigation-mode` | Pose Graph | OccupancyGrid YAML | Map Pose 标定 |
| --- | --- | --- | --- | --- |
| `mapping` | `mapping` | 禁止传入 | 禁止传入 | 不要求 |
| `incremental_mapping` | `mapping` | 必须存在 `.posegraph` 和 `.data` | 禁止传入 | 必须 |
| `localization` | `localization` | 必须存在 `.posegraph` 和 `.data` | 必须存在 | 必须 |
| `navigation` | `localization` | 必须存在 `.posegraph` 和 `.data` | 必须存在 | 必须 |

里程计模式在两端都使用 `ideal` 或 `realistic`。结构 TF 默认由 Isaac 发布；Realistic 模式也支持改由 Robot State Publisher 发布，但必须在两端同时显式选择 `structure_tf_source:=rsp`。Ideal + RSP 会被拒绝，Isaac 与 RSP 不能同时拥有结构 TF。

```bash
# 可选的 Realistic + RSP 结构 TF 组合；两个参数必须成对出现
./scripts/run_isaac.sh \
  --navigation-mode mapping \
  --mode realistic \
  --structure-tf-source rsp

./scripts/run_ros.sh mapping \
  odometry_mode:=realistic \
  structure_tf_source:=rsp
```

## 基线建图

终端 A 启动 Ideal 模式仿真：

```bash
./scripts/run_isaac.sh --navigation-mode mapping --mode ideal
```

终端 B 启动点云投影与异步 SLAM：

```bash
./scripts/run_ros.sh mapping odometry_mode:=ideal
```

命令会自动打开 Mapping RViz 和独立安全 Teleop 终端。使用 `W/A/S/D` 或方向键缓慢完成旋转、走廊覆盖和闭环；超过 0.18 秒无按键会自动停车，`Space` 立即停车，`Q` 安全退出。随后同时保存 OccupancyGrid 与序列化 Pose Graph：

```bash
./scripts/save_map.sh warehouse_v1
```

当前 `mapping_start.map` 已依据 `warehouse_v1` 建图结果标定为 `[0.0, 0.0, 0.0°]`，并记录了初始位姿不确定度。该精选基线的 OccupancyGrid、`.data` 和 Git LFS Pose Graph 均纳入仓库；`preflight.sh` 会拒绝未执行 `git lfs pull` 的指针文件、缺失工件以及大小或 SHA256 不一致。启动 Localization、Navigation 或增量建图前仍须把四个工件视为不可混用的同一版本。

标定步骤和动态障碍坐标重对齐要求见 [`docs/calibration.md`](docs/calibration.md)。

## 增量建图

完成基线地图和 Map Pose 标定后，可以加载旧 Pose Graph 继续建图：

```bash
# 终端 A
./scripts/run_isaac.sh --navigation-mode mapping --mode ideal

# 终端 B；参数可以是前缀，也可以带 .posegraph/.data 后缀
./scripts/run_ros.sh incremental_mapping \
  odometry_mode:=ideal \
  posegraph_file:="$PWD/data/maps/posegraphs/warehouse_v1"
```

该模式会启动 async SLAM Toolbox、加载旧 Pose Graph，并在 `/clock` 与 `odom → base_link` 就绪后发布已标定 `/initialpose`。完成变化区域采集后用新的版本名保存，禁止覆盖基线：

```bash
./scripts/save_map.sh warehouse_v2
```

提交的 `incremental_mapping.yaml` 是建图工作流描述符，不是导航试验；`NavigateToPose` runner 会显式拒绝它。增量试验必须使用上述 bringup、保存新地图，再显式对比工件。当前只验证了模式校验和启动编排，尚未用真实变化区域证明“耗时改善不少于 30%”。

保存基线、完整重建和增量更新三张地图后，复制并填写严格比较模板，再生成 JSON 证据报告：

```bash
cp ros2_ws/src/robot_experiments/config/incremental_comparison.example.yaml \
  data/reports/incremental_comparison.yaml
# 填写三张地图、同口径耗时和真实 Map 坐标矩形范围后：
ros2 run robot_experiments incremental_map_compare \
  --spec "$PWD/data/reports/incremental_comparison.yaml" \
  --output "$PWD/data/reports/incremental_comparison.json"
```

比较器按世界坐标处理不同原点、尺寸和 yaw，检查 30% 时间改善、三图覆盖率、变化单元恢复率、变化区域一致率与旧区域退化率。返回码 `0` 表示保存地图比较通过，`2` 表示阈值未通过，`1` 表示输入无效；即使通过也不能替代更新后 Localization/Nav2 的独立运行验收。

## 定位与导航

先启动与 ROS 模式一致的仿真：

```bash
# Ideal odometry（仅在 Map Pose 标定后可启动）
./scripts/run_isaac.sh --navigation-mode localization --mode ideal

# 或 Wheel Odom + IMU + EKF
./scripts/run_isaac.sh --navigation-mode localization --mode realistic
```

另一个终端启动 Localization 或完整导航：

```bash
./scripts/run_ros.sh localization \
  odometry_mode:=ideal \
  posegraph_file:="$PWD/data/maps/posegraphs/warehouse_v1"

./scripts/run_ros.sh navigation \
  odometry_mode:=ideal \
  posegraph_file:="$PWD/data/maps/posegraphs/warehouse_v1"
```

`run_ros.sh` 默认自动启动模式专用 RViz。Navigation 使用官方 Nav2 Navigation 2 面板和 GoalTool：等待 `Nav2 lifecycle activation completed` 后，在 RViz 地图中拖出目标位置和朝向即可；日常操作不需要 CLI 发布 `/goal_pose` 或另写桥接节点。Localization/Navigation 默认从已标定出生点自动播种，需人工定位时传 `initial_pose_source:=rviz` 并使用 **2D Pose Estimate**。`interactive:=false` 可同时关闭 RViz/Teleop用于无头实验。

Realistic 模式把 `odometry_mode` 改为 `realistic`。两端都会拒绝各自已知的不合法组合，但进程之间没有自动握手；操作者仍须保证 `odometry_mode` 和 `structure_tf_source` 成对一致，并用 Topic/TF introspection 确认唯一所有权。

Localization 和 Navigation 同时使用两种同版本地图工件：SLAM Toolbox 从
`posegraph_file` 加载 `.posegraph`/`.data` 以定位并发布 `map → odom`，
`nav2_map_server` 从 `map_file` 加载 `.yaml`/`.pgm` 并独占发布静态 `/map`。
若命令已传 `posegraph_file` 而省略 `map_file`，`run_ros.sh` 会按 Pose Graph
基名推导 `data/maps/occupancy/<basename>.yaml`，文件不存在时立即失败；工件名不一致时须显式传入 `map_file:=...`。SLAM Toolbox 的实时扫描栅格图只发布在
`/slam_toolbox/map`，用于诊断而不进入 Nav2，从而避免移动物体固化为静态地图残影。

Navigation 不会立即激活 Nav2。Activation Gate 会先等待 Map Server 的 transient-local `/map`、非零且新鲜的 `/clock`、新鲜的 `/scan` 和 `/odom`，以及连续稳定且时间戳新鲜的 `map → odom`，然后才请求 Nav2 lifecycle `STARTUP`。Gate 是唯一 Lifecycle 管理者并持续存活；Reset 后按暂停、清 Costmap、重新播种/等待 RViz 位姿、readiness、恢复的顺序执行，旧异步回调由代次令牌隔离。

最终回归在固定 `/map` 架构下完成：Ideal 1 m 静态 4/4（GT 误差 `0.178–0.188 m`），Ideal 3 m 长距离 1/1（GT 路径 `2.807 m`、误差 `0.193 m`），Realistic 1 m 静态 4/4（GT 误差 `0.175–0.187 m`）。所有轮次 Nav2 状态为成功、最终静止门满足；Realistic `/odom` 运行时只有 `ekf_filter_node` 一个发布者。这些是确定性 smoke/recovery 证据，不是计划中多起终点与多布局的统计验收。

Nav2 1.3.12 在 `SmacPlanner2D` 初始化时会打印一条 inflation `ERROR`；对该版本的 2D planner 路径，这是上游通用 collision checker 的误报。当前双 costmap 均已配置 `InflationLayer`，`0.55 m` 半径大于约 `0.34 m` 的带 padding 外接半径。原因、上游源码链接和限定条件见 [`docs/verification.md`](docs/verification.md#nav2-1312-smac-inflation-diagnostic)。

## 动态障碍与实验

动态障碍由 Isaac CLI 显式打开；默认关闭：

```bash
# Map Pose 标定、Pose Graph 可用后
ISAAC_NAV__GROUND_TRUTH__ENABLED=true \
  ./scripts/run_isaac.sh \
  --navigation-mode localization \
  --mode ideal \
  --dynamic-obstacles

./scripts/run_ros.sh navigation \
  odometry_mode:=ideal \
  posegraph_file:="$PWD/data/maps/posegraphs/warehouse_v1"
```

待 Localization、Ground Truth 与 Nav2 readiness 均通过后，可在第三个终端启动实验 runner。包装脚本会自动统一 Domain 42 和 Fast DDS 环境：

```bash
./scripts/run_experiment.sh \
  ros2_ws/src/robot_experiments/config/dynamic.yaml \
  data/experiment_runs/dynamic_smoke
```

Dynamic runner 在运行前会从 Isaac 读取并核对动态障碍 enabled flag、配置 SHA256 和 obstacle ID 集合，并严格比对物理配置与 ROS 场景中的 ID、形状、平面尺寸、Map 坐标端点、运动时长和 `repeat`，不匹配时 fail fast。

2026-07-17 的 Realistic 同环境、同起止点远距离验收结果为：静态 `20/20`、动态 `19/20`，两批均为 `0` 次物理碰撞；动态唯一失败是一次早期 Nav2 action abort。静态成功路线相对带 `0.34 m` 净空的 OccupancyGrid A* 理论最短路，最大偏差 `4.31%`、P95 `3.96%`。机器可读汇总写在 `data/reports/navigation_benchmark_20260717.json`；这证明当前固定仓库远距离基准通过，不等同于多地图、多布局的泛化结论。

`/simulation/collision` 来自底盘物理接触传感器；`/collision_monitor_state` 来自 Nav2 Collision Monitor。Ground Truth 只在显式启用且 Map Pose 已标定时发布，不发布 TF，也不进入控制链。

当前 Isaac 动态障碍坐标按已标定 Map Pose `[0, 0, 0°]` 编写，并与 ROS 场景端点使用同一变换。4-run 基线不等于计划中的广义动态避障率验收；若地图、出生点或 Map Pose 改变，必须按 [`docs/calibration.md`](docs/calibration.md) 重新计算障碍 USD 坐标。

## 自定义机器人迁移模板

阶段 13 已参数化 `robot.default_prim`、项目配置、Isaac 静态 TF，以及 ROS 侧的 Xacro、Wheel Odom 与 Nav2 参数入口。模板故意保留 `null` 测量项并 fail fast，不会伪造一个“已迁移”机器人。真实资产到位后设置 `ISAAC_NAV_PROJECT_CONFIG=isaac_sim/configs/custom_robot.project.yaml` 并按 [`isaac_sim/assets/robots/custom_robot/README.md`](isaac_sim/assets/robots/custom_robot/README.md) 完成 USD、质量/惯量、轮向、传感器外参、Footprint、出生点和全链路验收。

## 确定性 Reset

运行中的 Isaac 节点提供 Trigger 服务。先设置种子与出生点，再调用 Reset：

```bash
ros2 param set /isaac_navigation_sim reset_seed 4242
ros2 param set /isaac_navigation_sim reset_pose_name mapping_start
ros2 service call /simulation/reset std_srvs/srv/Trigger '{}'
```

Reset 会按固定顺序停车、清控制器、恢复 USD Pose、重置里程计/GT 路径/碰撞状态/动态障碍，并等待已排队的 Wheel/EKF/Costmap 请求。Trigger 只在事务完成后返回成功，失败不会伪造 reset event；重叠请求会被拒绝。Localization 的自动初始位姿只接受 Reset 后的新鲜 `/scan`，随后发布 `/simulation/localization_seeded`；RViz 初始位姿模式则等待新的人工输入。Navigation Gate 还会等待严格更新且稳定的 `map → odom` 和新鲜 `/odom` 后恢复 Lifecycle，因此服务返回不能等同于系统已经可接收目标。无有效非零命令时，Isaac 侧 idle watchdog 会把底盘保持在物理 sleep 状态；实测休眠时静止无漂移，有效低速命令仍能唤醒车体。

## 测试

纯 Python、ROS package 和可选 Isaac 测试统一由以下入口执行：

```bash
./scripts/test.sh
./scripts/test.sh --with-isaac
```

针对运行中系统的验收检查包括：

```bash
ros2 topic hz /clock
ros2 topic hz /lidar/points_raw
ros2 topic hz /scan
ros2 topic info --verbose /odom
ros2 topic info --verbose /map
ros2 topic info --verbose /slam_toolbox/map
ros2 topic info --verbose /tf
ros2 lifecycle get /map_server
ros2 run tf2_tools view_frames
```

统计阈值、场景数量和失败判定以 [`plan.md`](plan.md) 第十二、十四部分为准。单元测试或一次 smoke run 不能替代 200 次静态障碍实验等统计验收。

完整的已验证结果、复现命令和剩余验收项见 [`docs/verification.md`](docs/verification.md)。

运行异常时先执行只读诊断；确认旧会话已停止后再做受控清理：

```bash
./scripts/diagnose.sh
./scripts/clean_runtime.sh --dry-run
./scripts/clean_runtime.sh --dds-shm
```

清理器只处理 PID 元数据和命令身份均匹配的本项目进程。DDS SHM 清理还会先证明没有进程仍在使用 Fast DDS；详细判断见 [`docs/troubleshooting.md`](docs/troubleshooting.md)。

## 目录

```text
isaac_sim/   Stage、项目 USD overlay、传感器、OmniGraph、Reset、GT 和场景编排
ros2_ws/     描述、感知、Wheel Odom、EKF、SLAM、Nav2、bringup 和实验节点
data/        Git LFS 地图基线，以及 bag、轨迹、指标和报告的本地输出边界
scripts/     预检、资产导入、构建、测试、启动和地图保存入口
docs/        使用手册、逐文件索引、排障、开发、接口、标定、升级方案和验收文档
```

## Git 管理

提交遵循 Conventional Commits，并按可独立验证、可独立回滚的能力拆分。代码、参数、测试和必要文档应进入同一 commit；ROS 构建产物、Isaac 日志、rosbag 和批量实验输出不得进入普通 Git 历史。

```bash
git log --oneline --decorate --graph
git show --stat <commit>
```

具体约定见 [`CONTRIBUTING.md`](CONTRIBUTING.md)、[`docs/development.md`](docs/development.md) 和 [`data/README.md`](data/README.md)。
