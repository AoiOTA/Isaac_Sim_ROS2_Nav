# Isaac Sim 6.0.1 + ROS 2 Jazzy Navigation

这是一个面向 Clearpath Jackal 的二维 SLAM、定位、Nav2 导航与可重复实验工程。仿真运行在 Isaac Sim 6.0.1，ROS 侧使用 Jazzy、SLAM Toolbox、robot_localization、SmacPlanner2D、MPPI、Velocity Smoother 和 Collision Monitor。

完整设计和分阶段验收标准见 [`plan.md`](plan.md)。本 README 只保留可执行入口、运行约束和交付状态。

> 当前交付状态（2026-07-15）：Stage、LiDAR/IMU、前置 RGB Camera、Ideal/Realistic 里程计、SLAM、事务式 Reset/Lifecycle 恢复、四套 RViz、Mapping 安全 Teleop、动态障碍、Nav2 和实验框架均已实现。最新升级还加入了地图四工件 Manifest 绑定、`/scan_fault` 可控故障桥、真实 MPPI `/optimal_trajectory` 显示、Nav2 参数硬约束、进程组级 Runtime Profiler、物理步同步 ROS 发布、顺序 Lifecycle Shutdown、只观察 Lifecycle 的安全 Navigation 2 面板，以及独占 `/cmd_vel` 的底盘运动诊断。Jackal 项目 Overlay 已停用四个带 obsolete `customGeometry` 的只读轮胎 collider，改用对称标准 Cylinder；TGS 32/4 与 32/16 的 Warehouse + Ideal 隔离 A/B 支持冻结 32/4。地面碰撞拓扑现已拆成三个版本化、可逆 Profile：SimplePlane 1-collider、Warehouse 原始 32-collider 和 Warehouse plane-only 1-collider；源资产、匿名 overlay、source/target/disabled 集合与 contact target 由 runtime provenance v5 绑定，但真实 32-vs-1 运动 A/B 尚未完成。运动报告还会绑定实际加载的机器人、规范环境 ID、仿真模式与 Git 指纹，solver 必须通过 Stage 属性和初始化后 Articulation wrapper 的 USD 后端读回交叉校验；这不是 PhysX 引擎内部状态的直接读回，实际行为由隔离 A/B 和警告日志另行验证。机器人配置已升级为 schema v2：几何轮距与控制/Wheel Odom 使用的有效轮距是两个显式字段，Isaac、ROS Wheel Odom 和 Xacro 由同一 robot YAML 驱动；Realistic Wheel Odom 在精确 schema v5、robot config path/SHA 和七个 kinematics/controller 字段全部匹配前不会创建 `/wheel/odom`。稳定基线的两种轮距目前仍同为 `0.37559 m`，这次迁移不改变控制行为，也不表示有效轮距已标定。环境不匹配的 topology profile 会在 Isaac 组合 Stage 时失败；错误的运动报告环境标签会在 motion runner 创建命令 publisher 前失败。低速左右转向不对称、Realistic 对照及接触拓扑/材料/有效轮距仍未验收。旧 Camera 配置下的 `monitoring`/`high_quality` 已有 headless 性能与截图基线；2026-07-14 的 Camera schema v3 已完成严格配置、USD API 写入和 headless 属性读回，但新配置的真实静止/运动画质、RTF 和 GPU 仍待复测，不能沿用旧截图冒充验收。`warehouse_v1` 是可自动播种的发布基线；`warehouse_v2` 四工件已由 Manifest 登记，其中大型 `.posegraph` 由 Git LFS 管理，但来源日志缺失、运行对齐未验证且尚未标定，不能当作已验收地图。完整静态/动态统计、Warehouse V2 标定与路线矩阵及真实自定义机器人迁移仍未完成。详细证据与边界见 [`docs/verification.md`](docs/verification.md)。

> 阶段 3 另提供 `0.989/1.012 m` 两个版本化 `experimental_candidate`，只用于后续
> 两环境、多速度、拓扑和 Realistic A/B；它们尚未验收，不会覆盖稳定配置。clean
> `8973728` 已完成 `0.989 m`、SimplePlane/only1、六 contact profile × 三重复的正式
> 18-run 批次：证据采集 18/18 成功，但 6/6 物理组都只因左右旋转中心漂移不对称失败；
> 详见 [`docs/verification.md`](docs/verification.md)。

## 文档导航

第一次使用建议先看前两项：

| 文档 | 适合什么时候看 |
| --- | --- |
| [`docs/user_manual.md`](docs/user_manual.md) | 不熟悉项目时从这里开始；按步骤完成安装、Camera/RViz、导航、建图、Reset、底盘诊断、性能采样、排障和 Git 回溯。 |
| [`docs/repository_index.md`](docs/repository_index.md) | 想理解代码结构或准备修改文件时；逐项解释全部 Git 跟踪文件。 |
| [`docs/interfaces.md`](docs/interfaces.md) | 排查 Topic、QoS、TF、Reset、模式配对或 Nav2 激活问题时。 |
| [`docs/troubleshooting.md`](docs/troubleshooting.md) | 运行异常时按症状执行安全诊断和恢复，不盲目杀进程或删除 SHM。 |
| [`docs/calibration.md`](docs/calibration.md) | 修改地图、出生点、传感器外参或动态障碍坐标时。 |
| [`docs/verification.md`](docs/verification.md) | 判断某项能力是否真正验证，以及了解当前未验收边界时。 |
| [`docs/development.md`](docs/development.md) | 开发、测试、调试和准备 Git 提交时。 |
| [`docs/rviz_workflow_upgrade_plan.md`](docs/rviz_workflow_upgrade_plan.md) | 回溯本轮 RViz/Teleop/Lifecycle/性能升级的冻结设计和完成状态时。 |
| [`docs/runtime_reliability_and_performance_upgrade_plan.md`](docs/runtime_reliability_and_performance_upgrade_plan.md) | 回溯地图 Manifest、Camera、MPPI/Ceres、故障注入、Profiler 与有序退出的设计、实测结果和未验收边界时。 |
| [`docs/navigation_quality_and_simulation_fidelity_upgrade_plan.md`](docs/navigation_quality_and_simulation_fidelity_upgrade_plan.md) | 查看当前导航质量、物理/时间/传感器保真度、Warehouse V2 和最终静态/动态统计的执行顺序与验收门。 |
| [`plan.md`](plan.md) | 需要完整设计背景、技术选型、指标公式和最终验收目标时。 |

## 系统契约

- USD/PhysX 是唯一物理模型；URDF/Xacro 只用于 ROS RobotModel、结构描述和真机迁移。
- 环境 ground collider 必须由与环境匹配的版本化 topology profile 选择；任何禁用
  只进入匿名 session overlay，不修改 NVIDIA 或项目源 USD。
- 主 TF 树固定为 `map → odom → base_link`，不发布 ROS `world` frame。
- Mapping 与 Localization 严格互斥，因为二者都拥有 `map → odom`。
- Localization/Navigation 中由 `nav2_map_server` 唯一发布保存的静态 `/map`；
  SLAM Toolbox 只负责定位和 `map → odom`，其扫描栅格图重映射到
  `/slam_toolbox/map`，不得作为 Nav2 静态地图。
- Ideal 模式由 Isaac 唯一发布 `/odom` 和 `odom → base_link`；Realistic 模式关闭 Isaac odom，由 Wheel Odom + IMU + EKF 唯一发布。
- Ground Truth 只进入记录和指标模块，不进入 SLAM、EKF、Nav2 或控制器。
- 感知基线为 `/lidar/points_raw → pointcloud_to_laserscan → /scan`；默认不启用 Self Filter、VoxelGrid 或 Nav2 Voxel Layer。
- 当前动态避障是基于二维 `/scan` 的反应式避障，不表示三维路径规划或高度可通行性推理。
- 保存地图的 `.yaml`、`.pgm`、`.posegraph`、`.data` 必须由同一个 Manifest bundle 绑定；`auto` 初始位姿只接受与出生点标定版本一致的 bundle，未标定的新地图只能使用 RViz 人工播种。

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
./scripts/import_assets.sh
./scripts/build_ros2.sh
./scripts/preflight.sh
```

先按 [`docs/user_manual.md`](docs/user_manual.md) 安装 ROS/Python 依赖，再执行上述顺序；干净 clone 在资产导入和 ROS 构建之前运行 `preflight.sh` 会按设计失败。资产导入只在本地复制 Jackal 的最小运行依赖，并校验来源和 SHA256。NVIDIA 二进制资产被 Git 忽略；仓库只管理项目自有的 USD overlay、manifest 和导入工具。

默认使用：

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

两个终端必须使用相同的值。

## 五分钟快速启动

完成首次设置后，可先用仓库自带的 `warehouse_v1` 跑通最小导航闭环。GUI 默认启用 `monitoring` Camera；这里显式写出 profile，便于日志和性能报告回溯。

```bash
# 终端 A：Isaac、Ideal Odom、前向 Camera
cd /你的实际路径/Isaac_Sim_ROS2_Nav
./scripts/run_isaac.sh \
  --navigation-mode localization \
  --mode ideal \
  --camera-profile monitoring

# 终端 B：地图定位、Nav2、集成 RViz
cd /你的实际路径/Isaac_Sim_ROS2_Nav
./scripts/run_ros.sh navigation \
  odometry_mode:=ideal \
  posegraph_file:="$PWD/data/maps/posegraphs/warehouse_v1"
```

等待终端 B 出现 `Nav2 lifecycle activation completed`，再在 RViz 用 Navigation 2 Goal 工具拖出目标。右侧 **Robot Front Camera** 应显示机器人前向画面；若只做无头性能基线，在终端 A 加 `--headless --camera-profile off`，并在终端 B 加 `interactive:=false`。停止时在启动 ROS 的终端按一次 Ctrl+C；监督脚本会先按 Navigation/Localization 顺序关闭 Lifecycle，再终止其余 ROS 子进程。

如果这一步失败，先运行 `./scripts/preflight.sh` 和 `./scripts/diagnose.sh`，然后按 [`docs/user_manual.md`](docs/user_manual.md) 的逐步流程排查。Camera profile、地图保存、Reset、故障注入和性能采样的完整命令也都在使用手册中。

## 运行模式

Isaac 的 `--navigation-mode` 描述仿真 Reset 行为；ROS 的第一个参数选择实际算法栈。两端必须按下表配对：

| ROS 操作 | Isaac `--navigation-mode` | Pose Graph | OccupancyGrid YAML | Map Pose 标定 |
| --- | --- | --- | --- | --- |
| `mapping` | `mapping` | 禁止传入 | 禁止传入 | 不要求 |
| `incremental_mapping` | `mapping` | 必须存在 `.posegraph` 和 `.data` | 禁止传入 | 仅允许 `auto`；exact bundle 与出生点必须已标定，未标定地图不得进入增量建图 |
| `localization` | `localization` | 必须存在 `.posegraph` 和 `.data` | 必须存在 | 正式统计必须使用已标定 `auto`；`rviz` 仅用于人工对齐/诊断 |
| `navigation` | `localization` | 必须存在 `.posegraph` 和 `.data` | 必须存在 | 正式统计必须使用已标定 `auto`；`rviz` 仅用于人工对齐/诊断 |

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
MAP_VERSION="warehouse_mapping_$(date -u +%Y%m%dT%H%M%SZ)"
./scripts/save_map.sh "$MAP_VERSION"
```

`save_map.sh` 先在暂存目录生成 OccupancyGrid 与序列化 Pose Graph，逐项验证后再安装四个工件，最后原子发布 Manifest；任一步失败都会回滚，不留下“半套新地图”。脚本严格禁止覆盖，仓库已有的 `warehouse_v1`/`warehouse_v2` 不能再作为保存目标；请保留上面的新 `MAP_VERSION`，供后续校验和标定使用。当前 `mapping_start.map` 已依据 `warehouse_v1` 建图结果标定为 `[0.0, 0.0, 0.0°]`，并绑定对应 Manifest bundle。该精选基线的 OccupancyGrid、`.data` 和 Git LFS Pose Graph 均纳入仓库；`preflight.sh` 会拒绝未执行 `git lfs pull` 的指针文件、缺失工件、路径逃逸以及大小或 SHA256 不一致。启动 Localization、Navigation 或增量建图前必须把四个工件和 Manifest 视为不可混用的同一版本。

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
INCREMENTAL_VERSION="warehouse_incremental_$(date -u +%Y%m%dT%H%M%SZ)"
./scripts/save_map.sh "$INCREMENTAL_VERSION"
```

提交的 `incremental_mapping.yaml` 是建图工作流描述符，不是导航试验；`NavigateToPose` runner 会显式拒绝它。增量试验必须使用上述 bringup、保存新地图，再显式对比工件。当前只验证了模式校验和启动编排，尚未用真实变化区域证明“耗时改善不少于 30%”。

保存基线、完整重建和增量更新三张地图后，复制并填写严格比较模板，再生成 JSON 证据报告：

```bash
source ./scripts/setup_ros_env.sh
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

`run_ros.sh` 默认自动启动模式专用 RViz。Navigation 使用官方 GoalTool 和仓库内安全关闭版 Navigation 2 面板：等待 `Nav2 lifecycle activation completed` 后，在 RViz 地图中拖出目标位置和朝向即可；日常操作不需要 CLI 发布 `/goal_pose` 或另写桥接节点。局部轨迹显示读取 MPPI 真正输出的 `/optimal_trajectory`，`/transformed_global_plan` 是控制器参考路径，默认不订阅体量更大的候选集 `/trajectories`。Localization/Navigation 默认从已标定出生点自动播种；Manifest 或出生点 bundle 未标定时，`auto` 会 fail fast，需传 `initial_pose_source:=rviz` 并使用 **2D Pose Estimate**。`interactive:=false` 可同时关闭 RViz/Teleop，用于无头实验。

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

待 Localization、Ground Truth 与 Nav2 readiness 均通过后，可在第三个终端启动当前 4-seed 动态基线 runner：

```bash
source /opt/ros/jazzy/setup.bash
source "$PWD/ros2_ws/install/setup.bash"
ros2 launch robot_experiments experiment.launch.py \
  scenario_file:="$PWD/ros2_ws/src/robot_experiments/config/dynamic.yaml" \
  output_directory:="$PWD/data/experiment_runs/dynamic_smoke"
```

Dynamic runner 在运行前会从 Isaac 读取并核对动态障碍 enabled flag、配置 SHA256 和 obstacle ID 集合，并严格比对物理配置与 ROS 场景中的 ID、形状、平面尺寸、Map 坐标端点、运动时长和 `repeat`，不匹配时 fail fast。`repeat: false` 表示单程到达终点后保持，`repeat: true` 表示沿同一路径往返；两侧都必须显式填写且一致。当前横穿与对向两个单程障碍的 4-seed 基线已 4/4 成功，GT 终点误差为 `0.168–0.186 m`，每轮均看到 Collision Monitor、碰撞状态、定位状态和 `map → odom`，且最终静止。静态 smoke 仍只使用固定仓库、显式 `static: []`；这些 4 个 seed 是同一世界的确定性重复，不是多布局统计，也不得计入 N20/N21。参数冻结后仍须在已对齐、已标定的 `warehouse_v2` 上分别完成静态不少于 100 次、动态不少于 100 次正式统计。

`/simulation/collision` 来自底盘物理接触传感器；`/collision_monitor_state` 来自 Nav2 Collision Monitor。Ground Truth 只在显式启用且 Map Pose 已标定时发布，不发布 TF，也不进入控制链。

当前 Isaac 动态障碍坐标按已标定 Map Pose `[0, 0, 0°]` 编写，并与 ROS 场景端点使用同一变换。4-run 基线不等于计划中的广义动态避障率验收；若地图、出生点或 Map Pose 改变，必须按 [`docs/calibration.md`](docs/calibration.md) 重新计算障碍 USD 坐标。

## 自定义机器人迁移模板

阶段 13 已参数化 `robot.default_prim`、项目配置、Isaac 静态 TF，以及 ROS 侧的 Xacro、Wheel Odom 与 Nav2 参数入口。模板故意保留 `null` 测量项并 fail fast，不会伪造一个“已迁移”机器人。真实资产到位后设置 `ISAAC_NAV_PROJECT_CONFIG=isaac_sim/configs/custom_robot.project.yaml` 并按 [`isaac_sim/assets/robots/custom_robot/README.md`](isaac_sim/assets/robots/custom_robot/README.md) 完成 USD、质量/惯量、轮向、传感器外参、Footprint、出生点和全链路验收。

## 确定性 Reset

运行中的 Isaac 节点提供 Trigger 服务。先设置种子与出生点，再调用 Reset：

```bash
source ./scripts/setup_ros_env.sh
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
source ./scripts/setup_ros_env.sh
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

统计阈值、场景数量和失败判定以
[`docs/navigation_quality_and_simulation_fidelity_upgrade_plan.md`](docs/navigation_quality_and_simulation_fidelity_upgrade_plan.md)
为准。单元测试或一次 smoke run 不能替代参数冻结后的 `warehouse_v2` 静态
不少于 100 次（N20）和动态不少于 100 次（N21）正式统计。

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
data/        地图 bundle（精选 `.posegraph` 走 Git LFS），以及 bag、轨迹、指标和报告的本地输出边界
scripts/     预检、资产导入、构建、测试、启动和地图保存入口
docs/        使用手册、逐文件索引、排障、开发、接口、标定、升级方案和验收文档
```

## Git 管理

提交遵循 Conventional Commits，并按可独立验证、可独立回滚的能力拆分。代码、参数、测试和必要文档应进入同一 commit；ROS 构建产物、Isaac 日志、rosbag 和批量实验输出不得进入普通 Git 历史。

```bash
git log --oneline --decorate --graph
COMMIT_SHA="$(git rev-parse HEAD)"
git show --stat "$COMMIT_SHA"
```

具体约定见 [`CONTRIBUTING.md`](CONTRIBUTING.md)、[`docs/development.md`](docs/development.md) 和 [`data/README.md`](data/README.md)。
