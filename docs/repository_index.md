# 仓库文件索引

本文列出当前交付中的全部 265 个 Git 文件，并逐个解释职责。索引已与 `git ls-files --cached --others --exclude-standard` 做集合比对，当前没有遗漏。构建产物、运行日志、批量实验结果和本地导入的 NVIDIA 资产受 `.gitignore` 管理，不属于源码索引。

使用项目请先阅读 [`user_manual.md`](user_manual.md)；修改文件前再用本索引确认它属于 Isaac 物理层、ROS 算法层、配置层还是验证层。

快速查找某个路径可在本文中搜索完整文件名。例如看到 `activation_gate.py` 时，应同时查看同节的纯策略、参数和测试，而不是只改 ROS adapter。

## 0. 新用户先看哪里

| 需求 | 首选入口 | 接下来查看 |
| --- | --- | --- |
| 第一次安装并跑起来 | `README.md`、`docs/user_manual.md` | `scripts/preflight.sh`、`scripts/build_ros2.sh`、`scripts/run_isaac.sh`、`scripts/run_ros.sh` |
| 了解系统边界 | `docs/interfaces.md` | `isaac_sim/configs/project.yaml`、`ros2_ws/src/robot_bringup/launch/ros_stack.launch.py` |
| 修改参数 | 本索引对应配置行 | 同包的 `test/`、`docs/development.md` |
| 保存或切换地图 | `scripts/save_map.sh` | `robot_bringup/map_manifest.py`、`data/maps/manifests/`、`docs/calibration.md` |
| 做导航/性能实验 | `scripts/profile_runtime.sh` | `robot_experiments/runtime_profiler.py`、`docs/verification.md` |
| 遇到启动、TF、QoS 或退出问题 | `scripts/diagnose.sh` | `docs/troubleshooting.md`、`scripts/clean_runtime.sh` |

## 1. 顶层文件

| 文件 | 用途 |
| --- | --- |
| `.gitattributes` | 定义 Git LFS 规则；当前把 SLAM Toolbox `.posegraph` 作为 LFS 大文件管理。 |
| `.gitignore` | 排除构建目录、缓存、日志、官方资产、rosbag、批量结果和普通生成地图，同时放行当前 `warehouse_v2` 与历史 v1 精选地图。 |
| `CONTRIBUTING.md` | 代码贡献、分支、提交消息、验证和数据管理约定。 |
| `LICENSE` | 项目源码的 Apache-2.0 许可证文本。 |
| `README.md` | GitHub 首页：项目能力、运行入口、验证状态以及主要文档导航。 |
| `THIRD_PARTY_NOTICES.md` | 仓库级第三方代码声明；指出可配置 Ceres solver 源自 Slam Toolbox、采用 LGPL-2.1-only，并链接包内溯源和许可证。 |
| `plan.md` | 原始完整设计方案、技术选型、实施 SOP、实验指标和分阶段验收标准；用于理解“为什么这样设计”。 |
| `pyproject.toml` | Python 3.12 项目元数据、PyYAML/开发依赖及 pytest 搜索路径和 marker 配置。 |

## 2. 文档

| 文件 | 用途 |
| --- | --- |
| `docs/user_manual.md` | 面向使用者的中文操作手册，从 clone、构建到导航、建图、实验和排障。 |
| `docs/repository_index.md` | 本文件；逐项解释所有 Git 跟踪文件。 |
| `docs/skid_steer_navigation_solution.md` | Jackal 直行正常但导航转弯困难的专项复盘：原始症状、证据化根因、分层修复、Ideal 复杂路线结果和适用边界。 |
| `docs/kujiale_usd_navigation_postmortem_20260717.md` | 2026-07-17 酷家乐 USD 导航复盘：对比官方 Warehouse，记录材质、Stage、RTX/TF、建图标定、窄空间 MPPI、RViz 和剩余边界。 |
| `docs/interfaces.md` | 运行时权威契约：模式配对、Topic、Message、QoS、TF 所有权、Reset 和 Nav2 激活门。 |
| `docs/calibration.md` | USD Pose 与 Map Pose 的标定、三次冷启动复测、v2 实测记录、版本化和动态障碍坐标重对齐流程。 |
| `docs/verification.md` | 证据台账：已通过的运行/测试结果、已知 Nav2 诊断以及尚未验收的范围。 |
| `docs/development.md` | 开发环境、调试命令、测试方式、运行探针和提交/数据纪律。 |
| `docs/troubleshooting.md` | 按症状组织的运行排障手册，覆盖环境、Fast DDS SHM、QoS、TF、Lifecycle、Reset、RViz、Teleop 和 MPPI。 |
| `docs/rviz_workflow_upgrade_plan.md` | RViz 一体化升级的冻结设计、问题分析、实施步骤、测试矩阵和完成状态；用于回溯本轮架构决策。 |
| `docs/runtime_reliability_and_performance_upgrade_plan.md` | 运行时可靠性、性能、地图生命周期、相机和退出清理升级的实施计划、测试矩阵与证据回填台账。 |

## 3. 数据目录

| 文件 | 用途 |
| --- | --- |
| `data/README.md` | 说明地图、bag、轨迹、指标、报告和实验输出的存放及版本管理策略。 |
| `data/bags/.gitkeep` | 保留本地 rosbag 输出目录；实际 bag 默认不提交。 |
| `data/experiment_runs/.gitkeep` | 保留自动实验 CSV/JSON 输出目录；实际批次默认不提交。 |
| `data/metrics/.gitkeep` | 保留聚合指标输出目录。 |
| `data/reports/.gitkeep` | 保留分析报告、比较结果和图表输出目录。 |
| `data/trajectories/.gitkeep` | 保留估计轨迹与 Ground Truth 轨迹输出目录。 |
| `data/maps/manifests/warehouse_v1.yaml` | 历史不完整 v1 的来源、尺寸、SHA256、坐标原点和旧标定记录。 |
| `data/maps/manifests/warehouse_v2.yaml` | 当前完整仓库导航基线的来源、四件套大小/SHA256、`406×611` 栅格、坐标原点及三次冷启动标定证据；preflight 的权威清单。 |
| `data/maps/manifests/warehouse_new.yaml` | 酷家乐分支默认地图的四件套完整性、`154×248` 栅格、坐标原点和三次 Ideal 扫描配准标定证据。 |
| `data/maps/occupancy/.gitkeep` | 保留 OccupancyGrid 目录。 |
| `data/maps/occupancy/warehouse_v1.yaml` | 历史不完整 v1 的 ROS Map Server 元数据。 |
| `data/maps/occupancy/warehouse_v1.pgm` | 历史不完整 v1 的二值/三值占据栅格图，用于旧结果复现。 |
| `data/maps/occupancy/warehouse_v1_preview.png` | 方便人工浏览地图的预览图，不参与导航。 |
| `data/maps/occupancy/warehouse_v2.yaml` | 当前默认导航 OccupancyGrid 元数据：分辨率、原点、占据阈值及 v2 PGM 文件名。 |
| `data/maps/occupancy/warehouse_v2.pgm` | 覆盖完整仓库的 `406×611` 二值/三值栅格，由 Nav2 Map Server 发布。 |
| `data/maps/occupancy/warehouse_new.yaml` | 酷家乐默认 OccupancyGrid 元数据。 |
| `data/maps/occupancy/warehouse_new.pgm` | 酷家乐房间 `154×248 @ 0.05 m` 占据栅格。 |
| `data/maps/posegraphs/.gitkeep` | 保留 SLAM Toolbox 序列化地图目录。 |
| `data/maps/posegraphs/warehouse_v1.posegraph` | Git LFS 管理的历史 v1 SLAM Toolbox Pose Graph。 |
| `data/maps/posegraphs/warehouse_v1.data` | 与历史 v1 `.posegraph` 配套的序列化数据。 |
| `data/maps/posegraphs/warehouse_v2.posegraph` | Git LFS 管理的完整仓库 v2 Pose Graph；Realistic 定位和显式 Ideal 标定加载它。 |
| `data/maps/posegraphs/warehouse_v2.data` | 与 v2 `.posegraph` 不可拆分的序列化传感器/数据文件。 |
| `data/maps/posegraphs/warehouse_new.posegraph` | Git LFS 管理的酷家乐建图序列化工件；当前只作为 Ideal 建图来源记录。 |
| `data/maps/posegraphs/warehouse_new.data` | 与 `warehouse_new.posegraph` 配套的扫描数据。 |

## 4. 操作脚本

| 文件 | 用途 |
| --- | --- |
| `scripts/lib/common.sh` | 所有脚本共享的项目根、Isaac/ROS 路径、DDS 默认值、依赖检查和 `source_ros` 函数。 |
| `scripts/preflight.sh` | 启动前检查 Isaac 版本、官方资产、全部项目 ROS 包、安全 RViz 插件、GPU，并调用 `map_manifest verify` 校验 Git LFS 地图 bundle。 |
| `scripts/import_assets.sh` | 调用 Isaac Python，把官方 Jackal 的最小依赖复制到本地项目资产目录并校验 hash。 |
| `scripts/build_ros2.sh` | source ROS 环境并执行 `colcon build --symlink-install`。 |
| `scripts/test.sh` | 统一运行纯 Python、ROS colcon 和可选 Isaac/USD 测试。 |
| `scripts/run_isaac.sh` | 选择项目配置并监督 Isaac Python standalone 仿真；支持 custom profile，并只让监督器持有 Isaac 单实例锁，防止遗留 Omniverse Hub 锁住下一次启动。 |
| `scripts/run_ros.sh` | 启动四种顶层 ROS 操作；该酷家乐分支的 Localization/Navigation 默认 `warehouse_new` 与对应出生点，显式传图时仍按 basename 配对。监督器持有 ROS 单实例锁，并在启动 launch 子进程前关闭其继承副本，避免孤立 RViz 锁住下一次启动。 |
| `scripts/run_experiment.sh` | 在统一 Domain/RMW 环境中启动场景 runner，避免独立终端因 DDS 环境未对齐而看不到 `/clock`。 |
| `scripts/run_rviz.sh` | 按操作选择已安装的 Mapping/Localization/Navigation RViz 配置，统一 ROS 环境并阻止重复 RViz。 |
| `scripts/run_teleop.sh` | 只在 Mapping 场景启动 deadman 键盘节点；执行 TTY、冲突节点、参数和单实例检查。 |
| `scripts/run_teleop_terminal.sh` | 顶层 launch 的前台 Teleop 终端托管器；转发停止信号、校验 PID 身份并等待真实节点退出。 |
| `scripts/save_map.sh` | 在 `data/maps/.staging/` 事务保存四工件，逐级拒绝存储目录 symlink，以 no-clobber hard link 发布并复验未标定 manifest；最后发布提交标记，失败时只回滚本事务 inode，不覆盖或删除并发创建的数据。 |
| `scripts/diagnose.sh` | 只读收集环境、受管进程、ROS 图、Lifecycle、QoS、TF、`/optimal_trajectory`、仿真时间、DDS SHM、CPU governor 和近期错误。 |
| `scripts/clean_runtime.sh` | 依据 PID、UID、项目根、命令和工作目录认证受管进程组（含 ROS 监督器）后安全停止；可在无 Fast DDS 使用者时清理当前用户的残留 SHM。 |
| `scripts/setup_ros_env.sh` | 给新终端 source 的统一 ROS 环境入口；校验项目、Jazzy、Domain 42、Fast DDS 和工作区安装，可选安全重启 ROS daemon。 |
| `scripts/performance_mode.sh` | 可逆的主机性能策略工具；记录原 governor/EPP 状态，提供 status、enable、restore，并输出温度/GPU 功耗提醒。 |
| `scripts/profile_runtime.sh` | 运行 `runtime_profiler` 的命令行包装器；统一持续时间、预热、标签和原子 JSON 报告路径。 |
| `scripts/run_camera_view.sh` | 单独启动前视 RGB 相机 RViz 界面；复用项目 ROS 环境、受管进程组和 RViz 单实例锁。 |

## 5. Isaac Sim 包入口与主程序

| 文件 | 用途 |
| --- | --- |
| `isaac_sim/__init__.py` | Isaac 项目 Python 包标记及顶层说明。 |
| `isaac_sim/apps/__init__.py` | standalone 应用子包标记。 |
| `isaac_sim/apps/navigation_sim.py` | Isaac 主入口：解析模式与相机 profile CLI、加载配置、组合 Stage、创建传感器/图、启动 ROS 节点并运行仿真循环；退出时按 Camera graph、Reset bridge、ROS node/context 的依赖顺序释放资源。 |

## 6. Isaac USD 与资产

| 文件 | 用途 |
| --- | --- |
| `isaac_sim/assets/environments/navigation_scene.usda` | 项目场景 overlay；定义 `/World` 下的机器人和项目 Prim。官方 Warehouse 由 `SceneComposer` 按环境配置在运行时作为 subLayer 注入。 |
| `isaac_sim/assets/robots/jackal/jackal_nav.usda` | 项目 Jackal 入口 USD；通过 subLayer 引入本地导入的官方 Jackal，并覆盖/补充导航传感器 frame。 |
| `isaac_sim/assets/robots/jackal/asset_manifest.json` | 官方 Jackal 源文件到本地目标文件的清单与 SHA256。 |
| `isaac_sim/assets/robots/jackal/README.md` | Jackal 资产组合方式、导入边界和禁止直接提交官方文件的说明。 |
| `isaac_sim/assets/robots/jackal/source/.gitignore` | 忽略导入到 `source/` 的官方 Jackal 二进制/文本资产。 |
| `isaac_sim/assets/robots/jackal/configuration/.gitignore` | 忽略导入到 `configuration/` 的官方 schema/config USD。 |
| `isaac_sim/assets/robots/jackal/configuration/README.md` | 解释 configuration 文件只作为依赖，不是可 Reference 的机器人主入口。 |
| `isaac_sim/assets/robots/custom_robot/README.md` | Stage-13 自定义四轮滑移机器人迁移契约、必测参数、稳定接口和验收边界。 |

## 7. Isaac 顶层配置

| 文件 | 用途 |
| --- | --- |
| `isaac_sim/configs/project.yaml` | 默认 Jackal 项目总配置；串联环境、机器人、仿真模式、出生点、ROS、GT、第三人称相机和所有子配置文件。 |
| `isaac_sim/configs/custom_robot.project.yaml` | 自定义机器人项目模板；要求显式环境变量提供真实 USD、defaultPrim 和传感器配置，并把第三人称相机挂到自定义 `base_link`。 |
| `isaac_sim/configs/spawn_poses.yaml` | 出生点唯一真源；同时保存物理 USD Pose、已标定 Map Pose 及不确定度。 |
| `isaac_sim/configs/environments/kujiale_0026_A_to_B_door_open.spawn.yaml` | 酷家乐 0026 开门场景的 Mapping 出生点；Map Pose 在新地图标定前保持未标定。 |
| `isaac_sim/configs/environments/warehouse_multiple_shelves.yaml` | 官方 Warehouse 资产路径、组合方式和预期关键 Prim 的小型描述。 |

## 8. Isaac 机器人与仿真配置

| 文件 | 用途 |
| --- | --- |
| `isaac_sim/configs/robots/jackal.yaml` | Jackal 轮径/轮距、质量、物理参数、控制限幅、Footprint、joint/frame 和七条静态 TF。 |
| `isaac_sim/configs/robots/custom_robot.yaml` | 与运行时同 schema 的 fail-fast 模板；`null` 表示真实机器人尚未测量，不能伪造默认值。 |
| `isaac_sim/configs/simulation/ideal.yaml` | Ideal 模式与 TF 发布所有权的配置快照；当前运行时不读取此文件。 |
| `isaac_sim/configs/simulation/realistic.yaml` | Realistic 模式与 TF 发布所有权的配置快照；当前运行时不读取此文件。 |

## 9. Isaac 传感器、ROS 和场景配置

| 文件 | 用途 |
| --- | --- |
| `isaac_sim/configs/sensors/lidar_3d.yaml` | RTX LiDAR Prim、型号/频率和点云发布配置。 |
| `isaac_sim/configs/sensors/imu.yaml` | IMU Prim、频率、Topic 和 frame 配置。 |
| `isaac_sim/configs/sensors/camera.yaml` | 前视 RGB 相机的严格 schema：`off`、`monitoring`、`standard`、`high_quality` profile，Render/光学参数、RGB/CameraInfo Topic、QoS 与 RViz 传输配置。 |
| `isaac_sim/configs/ros2_bridge/topics.yaml` | Isaac OmniGraph 使用的 ROS Topic 和 frame 名称集中表。 |
| `isaac_sim/configs/ros2_bridge/qos.yaml` | Clock、sensor、command、state、TF、static TF 的显式 QoS profiles。 |
| `isaac_sim/configs/experiments/static.yaml` | 静态实验 seed、场景 ID 和目标列表的占位清单；当前 Isaac 运行时不读取此文件。 |
| `isaac_sim/configs/experiments/dynamic.yaml` | 原始 4-seed 动态 smoke 的横穿/对向物理障碍配置。 |
| `isaac_sim/configs/experiments/dynamic_benchmark.yaml` | 远距离 20-run 动态验收物理配置；两障碍在交互后继续离开路线，避免单程终点永久堵塞通道。 |
| `isaac_sim/configs/experiments/dynamic_complex_route.yaml` | Ideal 复杂长路线动态验收的四个低速一次性横穿/对向物理障碍。 |
| `isaac_sim/configs/experiments/dynamic.yaml` | Isaac 物理动态障碍物 ID、形状、尺寸、USD 轨迹、速度和 repeat。 |
| `isaac_sim/configs/experiments/incremental_mapping.yaml` | Isaac 增量地图变化占位配置；真实 changed-region 资产尚需另行制作。 |

## 10. Isaac OmniGraph 定义

| 文件 | 用途 |
| --- | --- |
| `isaac_sim/graphs/__init__.py` | OmniGraph 构建子包标记。 |
| `isaac_sim/graphs/spec.py` | 与 Isaac API 解耦的 GraphSpec/TargetPaths 数据结构和 materialize 工具；支持动态属性及 OnPhysicsStep 所需的 on-demand pipeline。 |
| `isaac_sim/graphs/ros_contract.py` | 严格读取 Topic/QoS 配置并提供图构建所需的 ROS 合同。 |
| `isaac_sim/graphs/control_graph.py` | `OnPhysicsStep` 每个物理步读取最新 `/cmd_vel`，使用真实仿真 `dt` 执行 DifferentialController，并通过单个 ArticulationController 原子写入四轮目标。 |
| `isaac_sim/graphs/odometry_graph.py` | Ideal 模式的 `/odom` 和 `odom → base_link` 发布图。 |
| `isaac_sim/graphs/sensor_graph.py` | `/clock`、JointState、IMU、RTX LiDAR PointCloud2 等传感器发布图。 |
| `isaac_sim/graphs/tf_graph.py` | 轮子动态 TF 与传感器/相机静态 TF 图；从所选 robot YAML 读取外参。 |
| `isaac_sim/graphs/camera_graph.py` | 校验集中式 Camera Topic/QoS/frame 契约，用同一 Render Product 构建 RGB 与 CameraInfo 发布图，并负责先销毁图后释放渲染资源。 |

## 11. Isaac 配置和通用工具

| 文件 | 用途 |
| --- | --- |
| `isaac_sim/src/__init__.py` | Isaac 核心源码包标记。 |
| `isaac_sim/src/config.py` | 严格解析总配置、环境变量替换、嵌套 override、模式组合和必需路径。 |
| `isaac_sim/src/environment_selection.py` | 解析自定义 USD 的绝对/相对/唯一文件名，选择同名出生点配置并生成按资产隔离的运行时 Stage 路径。 |
| `isaac_sim/src/yaml_utils.py` | 通用 YAML mapping、字段、数值、向量和未知 key 校验函数。 |

## 12. Isaac Stage 管理

| 文件 | 用途 |
| --- | --- |
| `isaac_sim/src/stage/__init__.py` | Stage 子包标记。 |
| `isaac_sim/src/stage/stage_loader.py` | 创建/打开项目 Stage，管理 Sublayer、Reference、Xform、保存，并在 overlay 中修复可解析的 `.../` 资产路径笔误。 |
| `isaac_sim/src/stage/scene_composer.py` | 按“环境 Sublayer + 机器人 Reference”组合最终 Stage，统一米/Z 元数据并保持项目层为 edit target。 |
| `isaac_sim/src/stage/physics_setup.py` | 查找并校验唯一 PhysicsScene；自定义环境缺失时创建预期场景，再设置重力、时间步和 PhysX 参数。 |
| `isaac_sim/src/stage/asset_validator.py` | 检查 defaultPrim、依赖、关键 Prim、传感器 frame、Articulation、wheel joint/碰撞和 GT 隔离。 |

## 13. Isaac 机器人运行时

| 文件 | 用途 |
| --- | --- |
| `isaac_sim/src/robot/__init__.py` | 机器人运行时子包标记。 |
| `isaac_sim/src/robot/articulation_runtime.py` | 获取 Jackal articulation、应用物理参数、控制/读取关节并处理 sleep/wake。 |
| `isaac_sim/src/robot/joint_validator.py` | 验证四个 wheel joint 的存在、分组、方向映射和 DOF 顺序。 |
| `isaac_sim/src/robot/spawn_pose_manager.py` | 读取/校验命名 USD/Map Pose；已标定 Pose 必须携带合法 map version 与 bundle SHA256，并向 Reset 提供 Pose 查询。 |
| `isaac_sim/src/robot/reset.py` | ResetManager 物理事务：预校验出生点/标定，暂停、停控、恢复 Pose、清速度、重置子系统，并在失败路径也恢复 timeline。 |
| `isaac_sim/src/robot/idle_brake.py` | 低层命令超时/死区 watchdog；无有效命令时让底盘可靠静止但允许低速唤醒。 |
| `isaac_sim/src/robot/skid_steer_motion_assist.py` | 对 PhysX 四轮滑移转向的曲率欠响应进行超时保护和加速度受限的平面速度补偿，使前进/倒车弧线与 `/cmd_vel` 一致。 |
| `isaac_sim/src/visualization/__init__.py` | Isaac GUI 可视化工具子包标记。 |
| `isaac_sim/src/visualization/third_person_camera.py` | 在机器人 `base_link` 下创建固定相对位姿的第三人称 USD Camera，并自动绑定 Isaac 主视口。 |

## 14. Isaac 传感器与 ROS Bridge

| 文件 | 用途 |
| --- | --- |
| `isaac_sim/src/sensors/__init__.py` | 传感器子包标记。 |
| `isaac_sim/src/sensors/sensor_factory.py` | 根据配置创建/校验 RTX LiDAR、IMU、相机 render product 和传感器 Prim。 |
| `isaac_sim/src/bridge/__init__.py` | ROS Bridge 子包标记。 |
| `isaac_sim/src/bridge/ros_graph_builder.py` | 按选定模式组合控制、传感器、里程计和结构 TF Graph。 |
| `isaac_sim/src/bridge/tf_ownership.py` | 计算并拒绝 Ideal/Realistic、Isaac/RSP 之间的重复 TF 所有权组合。 |
| `isaac_sim/src/bridge/reset_service.py` | 非阻塞 `/simulation/reset` 事务：防重入、等待 Wheel/EKF/Costmap futures、发布代次 reset event，并按初始位姿策略处理 fresh-scan 自动播种；Localization 拒绝运行时切换 Manifest 未授权 pose，退出时取消未完成 future 与重播器。 |

## 15. Isaac 实验与 Ground Truth

| 文件 | 用途 |
| --- | --- |
| `isaac_sim/src/experiment/__init__.py` | Isaac 实验场景子包标记。 |
| `isaac_sim/src/experiment/scenario.py` | 严格解析 Isaac `dynamic.yaml`，校验动态 box 的几何、轨迹、速度、repeat 并按 seed 采样相位。 |
| `isaac_sim/src/experiment/dynamic_obstacles.py` | 按配置中的 USD 坐标直接创建运动学动态 box，并按仿真时间推进或 Reset；不执行 USD→Map 变换。 |
| `isaac_sim/src/experiment/collision_monitor.py` | 读取底盘物理接触并发布 `/simulation/collision`。 |
| `isaac_sim/src/ground_truth/__init__.py` | Ground Truth 子包标记。 |
| `isaac_sim/src/ground_truth/transforms.py` | 纯数学的二维 Pose、`map_T_usd` 和 USD→Map 变换。 |
| `isaac_sim/src/ground_truth/recorder.py` | 独立发布 `/ground_truth/odom`、`/ground_truth/path` 并在 Reset 时清路径；不发布导航 TF。 |

## 16. Isaac 资产工具

| 文件 | 用途 |
| --- | --- |
| `isaac_sim/tools/__init__.py` | Isaac 工具子包标记。 |
| `isaac_sim/tools/import_assets.py` | 按 manifest 从官方资产根复制 Jackal 最小依赖，校验源/目标 SHA256 并避免修改官方文件。 |

## 17. Isaac 测试

| 文件 | 用途 |
| --- | --- |
| `isaac_sim/tests/conftest.py` | 把项目根加入测试导入路径。 |
| `isaac_sim/tests/test_config.py` | 测试项目/机器人配置 schema、环境变量 override、模式组合和 custom fail-fast。 |
| `isaac_sim/tests/test_environment_selection.py` | 测试自定义 USD 文件名解析、歧义拒绝、出生点配置优先级和隔离的运行时 Stage 路径。 |
| `isaac_sim/tests/test_third_person_camera.py` | 测试第三人称相机在 `base_link` 坐标系中的后上方位置、前向注视点和配置覆盖。 |
| `isaac_sim/tests/test_asset_paths.py` | 测试资产 manifest、项目 overlay 只引用本地导入以及路径可复现性。 |
| `isaac_sim/tests/test_stage_composition.py` | Isaac/USD marker 测试：Stage 组合、唯一 PhysicsScene、defaultPrim 和 articulation 结构。 |
| `isaac_sim/tests/test_graph_contracts.py` | 测试 Topic/QoS、控制/传感器/TF Graph 节点连接和 GT 隔离。 |
| `isaac_sim/tests/test_joint_mapping.py` | 测试四轮 joint 分组和控制顺序。 |
| `isaac_sim/tests/test_scan_projection.py` | 检查 Isaac LiDAR 与 ROS LaserScan 投影参数的 frame/range/角度契约。 |
| `isaac_sim/tests/test_ground_truth_transforms.py` | 测试 `map_T_usd` 与 Pose 变换数学。 |
| `isaac_sim/tests/test_idle_brake.py` | 测试命令死区、超时停车、低速唤醒和 sim-time 行为。 |
| `isaac_sim/tests/test_skid_steer_motion_assist.py` | 测试滑移转向补偿的前进/倒车跟踪、加速度边界、命令超时、Reset 和原地旋转标定。 |
| `isaac_sim/tests/test_spawn_pose_reset.py` | 测试出生点标定门、Reset 顺序、fresh-scan initial pose 和重复 seed。 |
| `isaac_sim/tests/test_dynamic_obstacles.py` | 测试动态障碍解析、有限数、repeat、轨迹推进和 Reset。 |
| `isaac_sim/tests/test_camera_contracts.py` | 测试相机 profile 默认值、严格 schema、RGB/CameraInfo 同源时间戳/QoS、CLI 选择和 Render Product 幂等释放。 |

## 18. ROS 工作区总览

`ros2_ws/src/` 中的项目包如下；不要依赖写死的包数量，新增包时应同时补本表、preflight 与构建验证：

| 包 | 职责 |
| --- | --- |
| **robot_description** | Xacro、RViz RobotModel 和可选 Robot State Publisher。 |
| **robot_perception** | PointCloud2→LaserScan 和可选 self filter。 |
| **robot_odometry** | 四轮编码器运动学和 `/wheel/odom`。 |
| **robot_localization_config** | IMU+Wheel Odom 的 EKF。 |
| **robot_mapping** | SLAM Toolbox Mapping/Localization 和 Map Server。 |
| **robot_slam_solver** | 独立的可配置线程 Ceres ScanSolver 插件；不修改系统 `/opt/ros`，包内 LGPL-2.1-only。 |
| **robot_navigation** | Nav2 planner/controller/costmap/safety。 |
| **robot_experiments** | 初始位姿、Reset runner、场景、指标和报告。 |
| **robot_bringup** | 模式验证、组合 launch 和 Nav2 readiness gate。 |
| **robot_rviz_plugins** | 项目自有、退出安全的 Nav2 RViz 面板，以及 Nav2 `VoxelGrid` 三维显示插件。 |
| **robot_teleop** | 仅 Mapping 可用、带稳态时钟 deadman 和速度上限的 W/A/S/D 键盘控制。 |

## 19. `robot_description`

| 文件 | 用途 |
| --- | --- |
| `ros2_ws/src/robot_description/CMakeLists.txt` | 安装 Xacro、launch、脚本、RViz 并注册 ament 测试。 |
| `ros2_ws/src/robot_description/package.xml` | 包元数据及 xacro、RSP、launch、测试依赖。 |
| `ros2_ws/src/robot_description/launch/description.launch.py` | 处理自定义 Xacro；按 `publish_tf` 选择 RSP 或只发布 `robot_description`。 |
| `ros2_ws/src/robot_description/scripts/robot_description_publisher.py` | Isaac 拥有结构 TF 时，仅发布 transient-local `/robot_description`，避免 RSP 重复 TF。 |
| `ros2_ws/src/robot_description/urdf/jackal.urdf.xacro` | Jackal ROS 描述主入口，组合 base 与 sensors。 |
| `ros2_ws/src/robot_description/urdf/jackal_base.xacro` | base_link、四轮 link/joint、visual/collision/inertial 的 ROS 模型。 |
| `ros2_ws/src/robot_description/urdf/jackal_sensors.xacro` | lidar、imu、camera、双目和 optical frame 的固定关节。 |
| `ros2_ws/src/robot_description/rviz/mapping.rviz` | Mapping/Incremental Mapping 专用界面：实时地图、LaserScan、RobotModel、TF 与 Odom；不加载会在退出阶段残留后台线程的 SLAM Toolbox 面板。 |
| `ros2_ws/src/robot_description/rviz/localization.rviz` | Localization 专用界面：固定 `/map`、可选 `/slam_toolbox/map` 诊断层、扫描、里程计和 2D Pose Estimate。 |
| `ros2_ws/src/robot_description/rviz/navigation.rviz` | Navigation 完整界面：项目安全 Nav2 面板、标准 2D Goal Pose、双 Costmap、全局路径、真实 MPPI 局部轨迹 `/optimal_trajectory`、Footprint、Collision Monitor 和 RGB-D Fusion（青色深度点云、已标记体素盒子）。 |
| `ros2_ws/src/robot_description/rviz/camera_view.rviz` | 独立的前视 Camera RViz 布局；显示 RGB 图像及必要的机器人/TF 上下文，供单独观察相机链路。 |
| `ros2_ws/src/robot_description/test/test_urdf.py` | 展开 Xacro，检查必需 link/joint、轮轴、传感器固定关节、禁用导航/GT frame 和 description-only TF 所有权。 |
| `ros2_ws/src/robot_description/test/test_rviz_configs.py` | 解析四套 RViz YAML，锁定模式 Topic、安全面板、真实局部轨迹、显示开关以及 Map/Sensor QoS。 |

## 20. `robot_perception`

| 文件 | 用途 |
| --- | --- |
| `ros2_ws/src/robot_perception/CMakeLists.txt` | 安装配置/launch并注册 lint/pytest。 |
| `ros2_ws/src/robot_perception/package.xml` | pointcloud_to_laserscan、launch 和测试依赖。 |
| `ros2_ws/src/robot_perception/config/pointcloud_to_laserscan.yaml` | 投影 target frame、TF 容差、高度、角度、scan time、range 和 use_inf 参数。 |
| `ros2_ws/src/robot_perception/config/self_filter_optional.yaml` | 可选 CropBox 自车过滤参数；默认链路不启用。 |
| `ros2_ws/src/robot_perception/launch/lidar_processing.launch.py` | 启动直接投影链；启用 self-filter 路由时等待外部过滤节点发布过滤后点云。 |
| `ros2_ws/src/robot_perception/test/test_projection_config.py` | 检查投影 frame、高度/range/scan time 契约，并确认 self-filter 必须显式启用。 |

## 21. `robot_odometry`

| 文件 | 用途 |
| --- | --- |
| `ros2_ws/src/robot_odometry/package.xml` | Python ROS 包元数据和 JointState/Odometry/reset 依赖。 |
| `ros2_ws/src/robot_odometry/setup.py` | 安装 `robot_odometry` 包、配置文件和 `wheel_odometry_node` console script。 |
| `ros2_ws/src/robot_odometry/setup.cfg` | ROS Python 可执行文件安装到 libexec 的规则。 |
| `ros2_ws/src/robot_odometry/resource/robot_odometry` | ament resource index 标记。 |
| `ros2_ws/src/robot_odometry/robot_odometry/__init__.py` | 包入口，重新导出核心里程计 dataclass 和计算类。 |
| `ros2_ws/src/robot_odometry/robot_odometry/kinematics.py` | 纯 Python 四轮滑移转向积分、joint 映射、时间间隔和协方差计算。 |
| `ros2_ws/src/robot_odometry/robot_odometry/wheel_odometry_node.py` | 订阅 `/joint_states`、发布 `/wheel/odom`、响应 reset；不发布 TF。 |
| `ros2_ws/src/robot_odometry/config/wheel_odometry.yaml` | 轮径、有效轮距、joint 名、频率、协方差和积分间隔。 |
| `ros2_ws/src/robot_odometry/launch/wheel_odometry.launch.py` | 启动节点并允许替换自定义参数 YAML。 |
| `ros2_ws/src/robot_odometry/test/test_kinematics.py` | 测试直行、旋转、角度 wrap、时间异常、Reset 和 joint 输入边界。 |

## 22. `robot_localization_config`

| 文件 | 用途 |
| --- | --- |
| `ros2_ws/src/robot_localization_config/CMakeLists.txt` | 安装 EKF 配置/launch并注册测试。 |
| `ros2_ws/src/robot_localization_config/package.xml` | robot_localization、消息和 launch 依赖。 |
| `ros2_ws/src/robot_localization_config/config/ekf.yaml` | 二维 EKF 输入选择、frame、频率、sensor timeout 和 TF 发布配置。 |
| `ros2_ws/src/robot_localization_config/launch/ekf.launch.py` | 启动 `ekf_node` 并使用 sim time。 |
| `ros2_ws/src/robot_localization_config/test/test_ekf_config.py` | 检查 EKF 是 Realistic `/odom`/TF 唯一 owner，且不使用 Ground Truth。 |

## 23. `robot_mapping`

| 文件 | 用途 |
| --- | --- |
| `ros2_ws/src/robot_mapping/CMakeLists.txt` | 安装 SLAM 配置/launch并注册测试。 |
| `ros2_ws/src/robot_mapping/package.xml` | SLAM Toolbox、Map Server、lifecycle 和 launch 依赖。 |
| `ros2_ws/src/robot_mapping/config/slam_mapping.yaml` | Async Mapping 的 solver、scan matcher、回环、地图分辨率和 frame 参数。 |
| `ros2_ws/src/robot_mapping/config/slam_localization.yaml` | Localization 的 scan matching、buffer、TF 和已保存 Pose Graph 参数；实测每两帧处理一次以降低 MPPI 竞争。 |
| `ros2_ws/src/robot_mapping/launch/mapping.launch.py` | 启动 async SLAM；基线模式新建图，增量模式加载 `.posegraph/.data`，并给 Lifecycle 清理保留退出宽限。 |
| `ros2_ws/src/robot_mapping/launch/localization.launch.py` | 启动 Map Server 和 Localization SLAM；`/map` 固定、SLAM 诊断图重映射，并给 Lifecycle 清理保留退出宽限。 |
| `ros2_ws/src/robot_mapping/test/test_mapping_modes.py` | 检查两个模式的 executable、参数互斥、Map Server、话题隔离和退出 timeout 契约。 |

## 24. `robot_slam_solver`

| 文件 | 用途 |
| --- | --- |
| `ros2_ws/src/robot_slam_solver/CMakeLists.txt` | 构建并安装 `configurable_ceres_solver_plugin`、头文件、许可证和 NOTICE，向 Slam Toolbox 导出 pluginlib 描述并注册契约测试。 |
| `ros2_ws/src/robot_slam_solver/package.xml` | solver 包元数据、LGPL-2.1-only 声明，以及 Ceres、Eigen、Boost、pluginlib、rclcpp 和 Slam Toolbox 依赖。 |
| `ros2_ws/src/robot_slam_solver/solver_plugins.xml` | pluginlib 类注册表；把 `robot_slam_solver::ConfigurableCeresSolver` 声明为 `karto::ScanSolver` 实现。 |
| `ros2_ws/src/robot_slam_solver/LICENSE` | 该独立 solver 包及其派生源码适用的完整 GNU LGPL 2.1 许可证文本。 |
| `ros2_ws/src/robot_slam_solver/NOTICE.md` | 记录上游 Slam Toolbox 2.8.5 仓库、精确 commit、原文件 SHA256 以及本地命名和线程参数修改。 |
| `ros2_ws/src/robot_slam_solver/include/robot_slam_solver/ceres_solver.hpp` | 可配置 Ceres solver 类声明；实现 Karto `ScanSolver` 的图节点、约束、求解、修正和 lifecycle 配置接口。 |
| `ros2_ws/src/robot_slam_solver/include/robot_slam_solver/ceres_utils.hpp` | Ceres 二维位姿图优化辅助实现：角度 manifold、旋转矩阵、残差项和约束哈希。 |
| `ros2_ws/src/robot_slam_solver/src/ceres_solver.cpp` | solver 插件实现；声明并校验 `ceres_num_threads`，维护 Karto 图约束并调用 Ceres 求解，取代上游硬编码 50 线程。 |
| `ros2_ws/src/robot_slam_solver/test/test_solver_contract.py` | 锁定可配置线程、硬编码移除、pluginlib 类、硬件并发上限、上游 commit 和 LGPL 文件存在性。 |

## 25. `robot_navigation`

| 文件 | 用途 |
| --- | --- |
| `ros2_ws/src/robot_navigation/CMakeLists.txt` | 安装 Nav2 配置、行为树和 launch，并注册测试。 |
| `ros2_ws/src/robot_navigation/package.xml` | Nav2 planner/controller/behavior/smoother/collision monitor/lifecycle 依赖。 |
| `ros2_ws/src/robot_navigation/behavior_trees/navigate_to_pose_with_dead_end_recovery.xml` | 单目标前进优先导航行为树；常规 MPPI 不倒车，系统恢复时在原地旋转前执行 0.55 m 激光安全监控倒车。 |
| `ros2_ws/src/robot_navigation/behavior_trees/navigate_through_poses_with_dead_end_recovery.xml` | 多目标版本的死胡同恢复行为树，保持与单目标相同的清图、倒车、旋转和等待顺序。 |
| `ros2_ws/src/robot_navigation/config/nav2_params.yaml` | 全局/局部 Costmap、SmacPlanner2D、MPPI、BT、Velocity Smoother、Collision Monitor 的统一参数；MPPI 使用 10 Hz、2 秒窗和 500 批次，常规路径前进优先，恢复链允许有限倒车。 |
| `ros2_ws/src/robot_navigation/launch/navigation.launch.py` | 创建 Nav2 lifecycle 节点并注入项目死胡同恢复行为树；默认不 autostart，等待外部 readiness gate。 |
| `ros2_ws/src/robot_navigation/test/test_nav2_config.py` | 检查插件、Footprint、inflation、话题链、2D obstacle layer 和安全参数。 |

## 26. `robot_experiments` 包装文件与配置

| 文件 | 用途 |
| --- | --- |
| `ros2_ws/src/robot_experiments/package.xml` | 实验 ROS 包元数据及 Nav2 action、TF、lifecycle、YAML 等依赖。 |
| `ros2_ws/src/robot_experiments/setup.py` | 安装配置/launch并注册 experiment runner、initial pose、incremental comparator、runtime profiler 与 opt-in scan fault bridge CLI。 |
| `ros2_ws/src/robot_experiments/setup.cfg` | Python ROS 可执行文件安装规则和 flake8 配置。 |
| `ros2_ws/src/robot_experiments/resource/robot_experiments` | ament resource index 标记。 |
| `ros2_ws/src/robot_experiments/config/scenario.schema.yaml` | Static/Dynamic/Incremental 场景 YAML 的结构、类型和必需字段定义。 |
| `ros2_ws/src/robot_experiments/config/static.yaml` | 1 m Ideal/Realistic 静态 smoke：4 seeds、目标、阈值和配置引用。 |
| `ros2_ws/src/robot_experiments/config/static_long_range.yaml` | 3 m 长距离 smoke：单 seed 和更长 timeout。 |
| `ros2_ws/src/robot_experiments/config/dynamic.yaml` | ROS 动态基线：目标、4 seeds、两条 Map-frame 障碍轨迹和 repeat 契约。 |
| `ros2_ws/src/robot_experiments/config/static_benchmark.yaml` | Realistic 远距离静态验收：同一仓库、同起止点、20 seeds 和严格终点/静止/安全观测门。 |
| `ros2_ws/src/robot_experiments/config/dynamic_benchmark.yaml` | Realistic 远距离动态验收：同起止点、20 seeds，并与 Isaac 横穿/对向实体障碍配置严格对齐。 |
| `ros2_ws/src/robot_experiments/config/static_complex_route.yaml` | Ideal 前进优先复杂静态验收：6 个强制航点、约 50 m 路线、3 seeds 和运动质量门。 |
| `ros2_ws/src/robot_experiments/config/dynamic_complex_route.yaml` | Ideal 复杂动态验收：与四个物理移动障碍严格对齐的 6 航点、3-seed 长路线。 |
| `ros2_ws/src/robot_experiments/config/motion_benchmark.yaml` | 底盘直线、原地旋转、正反圆弧、蛇形和快速反向的自动运动基准。 |
| `ros2_ws/src/robot_experiments/config/incremental_mapping.yaml` | 增量建图工作流描述符；NavigateToPose runner 会明确拒绝它。 |
| `ros2_ws/src/robot_experiments/config/incremental_comparison.example.yaml` | 三地图离线比较模板；路径、耗时、真实变化矩形为必填占位。 |

## 27. `robot_experiments` 运行代码

| 文件 | 用途 |
| --- | --- |
| `ros2_ws/src/robot_experiments/robot_experiments/__init__.py` | 实验 Python 包标记。 |
| `ros2_ws/src/robot_experiments/robot_experiments/configuration.py` | 通用配置加载、相对路径解析和严格字段工具。 |
| `ros2_ws/src/robot_experiments/robot_experiments/spawn_poses.py` | ROS 侧读取命名 USD/Map Pose并要求已标定。 |
| `ros2_ws/src/robot_experiments/robot_experiments/scenario.py` | 场景 dataclass/loader、导航场景门、Isaac 动态运行时和物理几何契约校验。 |
| `ros2_ws/src/robot_experiments/robot_experiments/metrics.py` | 单轮成功判定、路径长度、角度 wrap、避障率和增量时间改善等纯指标函数。 |
| `ros2_ws/src/robot_experiments/robot_experiments/optimal_path.py` | 读取 OccupancyGrid YAML/PGM、按机器人净空膨胀障碍，并用禁止斜穿墙角的 8 邻域 A* 计算理论最短路。 |
| `ros2_ws/src/robot_experiments/robot_experiments/navigation_benchmark.py` | 汇总静态/动态 manifest，验收 95%/90% 成功率和成功静态路线相对理论最短路不超过 20% 的偏差。 |
| `ros2_ws/src/robot_experiments/robot_experiments/report.py` | manifest schema 校验、配置 SHA256，以及单轮 CSV/JSON 报告的原子写入。 |
| `ros2_ws/src/robot_experiments/robot_experiments/initial_pose_publisher.py` | 冷启动与 Reset 后等待新 `/clock`、`/scan`、TF 再发布标定 `/initialpose`；支持 reseed 服务、状态 Topic 和合法 RViz 人工位姿优先权。 |
| `ros2_ws/src/robot_experiments/robot_experiments/experiment_runner.py` | 自动多轮 Reset、恢复门、NavigateToPose、cancel 隔离、观测/指标和报告主节点。 |
| `ros2_ws/src/robot_experiments/robot_experiments/motion_benchmark.py` | 使用 Ground Truth 自动执行并验收底盘运动原语、曲率、误差和转向反向延迟。 |
| `ros2_ws/src/robot_experiments/robot_experiments/incremental_map_compare.py` | 离线加载三张 Map Server YAML/PGM，按世界坐标比较耗时、变化恢复和旧区退化。 |
| `ros2_ws/src/robot_experiments/robot_experiments/runtime_profiler.py` | 运行时观测器：采集 RTF、Topic/TF/Camera/Nav2 指标并识别 supervisor operation；按 PID+start-time 聚合进程树 CPU/RSS/GPU，成员集合变化时将 CPU 标为无效而非错误差分，原子输出 JSON。 |
| `ros2_ws/src/robot_experiments/robot_experiments/scan_fault.py` | 无 ROS 依赖的 LaserScan 故障状态机；实现丢包/暂停/frame 替换/恢复/计数，并强制每条命令携带当前 epoch，彻底隔离 Reset 前排队的旧命令。 |
| `ros2_ws/src/robot_experiments/robot_experiments/scan_fault_bridge.py` | ROS adapter：把 `/scan` 按显式 JSON 命令转发到 `/scan_fault`，发布 transient-local 状态，并在 reset event 或时间戳回退时清除旧故障。仅用于 Collision Monitor 安全验证。 |
| `ros2_ws/src/robot_experiments/launch/initial_pose.launch.py` | 把 spawn pose、持续监听和扫描/TF 恢复参数传给 initial pose publisher。 |
| `ros2_ws/src/robot_experiments/launch/experiment.launch.py` | 启动 experiment runner并传入场景、出生点、输出目录和可选配置 override。 |
| `ros2_ws/src/robot_experiments/launch/scan_fault_bridge.launch.py` | opt-in 故障桥 launch；暴露输入、输出、控制、状态、Reset Topic 与状态周期参数，生产导航默认不启动。 |

## 28. `robot_experiments` 测试

| 文件 | 用途 |
| --- | --- |
| `ros2_ws/src/robot_experiments/test/fixtures/spawn_poses_calibrated.yaml` | 已标定出生点的最小测试 fixture。 |
| `ros2_ws/src/robot_experiments/test/fixtures/spawn_poses_uncalibrated.yaml` | 未标定出生点 fixture，用于验证 fail-fast。 |
| `ros2_ws/src/robot_experiments/test/test_configuration.py` | 测试三种场景解析、schema、相对路径、静/动态契约和真实物理配置一致性。 |
| `ros2_ws/src/robot_experiments/test/test_metrics.py` | 测试成功/失败原因、路径、角度、率和时间改善指标。 |
| `ros2_ws/src/robot_experiments/test/test_optimal_path.py` | 用合成地图与 20+20 份 manifest 测试 A* 绕行、阻塞判定、成功率和路径偏差验收。 |
| `ros2_ws/src/robot_experiments/test/test_report.py` | 测试 manifest 必需字段、有限数/hash、CSV/JSON 原子替换和输出路径安全。 |
| `ros2_ws/src/robot_experiments/test/test_incremental_map_compare.py` | 用合成 PGM 测试变化恢复、旧区退化、阈值 override 和 CLI 返回码。 |
| `ros2_ws/src/robot_experiments/test/test_package_contract.py` | 检查安装入口、配置/launch 文件和 reset/dynamic contract 关键代码存在。 |
| `ros2_ws/src/robot_experiments/test/test_ros_adapters.py` | 在 ROS 环境中测试消息→内部 sample 的 adapter 和时间戳保存。 |
| `ros2_ws/src/robot_experiments/test/test_initial_pose_publisher.py` | 测试回钟/Reset 后扫描屏障、reseed、人工位姿合法性与人工所有权不被自动位姿覆盖。 |
| `ros2_ws/src/robot_experiments/test/test_experiment_motion_quality.py` | 测试命令/实测前进、倒车、弧线、停止和转向换向指标。 |
| `ros2_ws/src/robot_experiments/test/test_motion_benchmark.py` | 测试运动基准配置、判定和报告逻辑。 |

## 29. `robot_bringup` 配置与入口

| 文件 | 用途 |
| --- | --- |
| `ros2_ws/src/robot_bringup/package.xml` | 顶层 bringup 包元数据，声明组合 launch、地图契约、Lifecycle gate、顺序关停及项目 RViz 插件所需依赖。 |
| `ros2_ws/src/robot_bringup/setup.py` | 安装配置/launch，并注册 Lifecycle Gate、初始位姿策略、`map_manifest` 与 `ordered_shutdown` CLI。 |
| `ros2_ws/src/robot_bringup/setup.cfg` | Python ROS 可执行文件安装规则。 |
| `ros2_ws/src/robot_bringup/resource/robot_bringup` | ament resource index 标记。 |
| `ros2_ws/src/robot_bringup/config/modes.yaml` | 人类可读的四种 operation、里程计和 TF 所有权矩阵。 |
| `ros2_ws/src/robot_bringup/config/activation_gate.yaml` | Nav2 startup/reset recovery 的 freshness、TF 稳定窗口、服务超时、有限重试和退避参数。 |
| `ros2_ws/src/robot_bringup/launch/mapping_bringup.launch.py` | Baseline Mapping 顶层入口。 |
| `ros2_ws/src/robot_bringup/launch/incremental_mapping_bringup.launch.py` | 加载旧 Pose Graph 继续 Mapping 的顶层入口。 |
| `ros2_ws/src/robot_bringup/launch/localization_bringup.launch.py` | Localization 顶层入口；转发公共参数及只用于地图标定的 `posegraph_calibration`。 |
| `ros2_ws/src/robot_bringup/launch/navigation_bringup.launch.py` | Localization + Nav2 + readiness gate 顶层入口。 |
| `ros2_ws/src/robot_bringup/launch/ros_stack.launch.py` | 四种入口共享组合器；校验模式/文件，包含描述、感知、odom、SLAM/Nav2、显式 Ideal Pose Graph 标定，并受管启动模式专用 RViz 与 Mapping Teleop。 |

## 30. `robot_bringup` 代码与测试

| 文件 | 用途 |
| --- | --- |
| `ros2_ws/src/robot_bringup/robot_bringup/__init__.py` | bringup Python 包标记。 |
| `ros2_ws/src/robot_bringup/robot_bringup/mode_contract.py` | 纯函数校验 operation、Ideal/Realistic、Isaac/RSP、自定义文件、地图 manifest/工件/初始位姿绑定，以及 `controller_period <= MPPI model_dt` 的启动前时序约束。 |
| `ros2_ws/src/robot_bringup/robot_bringup/map_manifest.py` | ROS 无关地图 bundle 契约与 CLI；拒绝保留版本/父级 symlink/逃逸/LFS pointer，校验四工件、正数栅格元数据，并把 version/bundle/profile 与 USD/Map pose、yaw、标准差双向逐值绑定。 |
| `ros2_ws/src/robot_bringup/robot_bringup/ordered_shutdown.py` | 用私有 rclpy Context/Executor 和一个全局 deadline 调 Lifecycle：Navigation 先停导航再停定位，Localization 停定位，Mapping 依次 deactivate/cleanup/shutdown SLAM Toolbox。 |
| `ros2_ws/src/robot_bringup/robot_bringup/interactive_policy.py` | 纯函数解析 interactive/RViz/Teleop 选项、按模式选配置并寻找可受管的终端模拟器。 |
| `ros2_ws/src/robot_bringup/robot_bringup/initial_pose_policy.py` | 以 transient-local Topic 发布 `auto|rviz` 初始位姿所有权，供 Isaac Reset 与 ROS Gate 共用。 |
| `ros2_ws/src/robot_bringup/robot_bringup/ideal_localization_tf.py` | Ideal 定位发布新鲜 identity `map→odom`，并提供兼容实验 Reset 的空缓冲清理服务。 |
| `ros2_ws/src/robot_bringup/robot_bringup/lifecycle_policy.py` | Lifecycle STARTUP/PAUSE/RESUME、混合稳定状态的有序归一化和有限指数退避纯策略。 |
| `ros2_ws/src/robot_bringup/robot_bringup/readiness.py` | 按仿真代次判断 Clock/scan/odom/map 与 map→odom 的新鲜、稳定和时间跳变。 |
| `ros2_ws/src/robot_bringup/robot_bringup/activation_gate.py` | Nav2 Lifecycle 唯一管理者；原子检查状态，Reset 时暂停/清图/重播/恢复，并修复 Manager 部分转换留下的 Active/Inactive 混合状态。 |
| `ros2_ws/src/robot_bringup/test/test_mode_contract.py` | 覆盖合法/非法模式、缺失地图、后缀归一化和自定义文件入口。 |
| `ros2_ws/src/robot_bringup/test/test_readiness.py` | 覆盖 freshness、TF 抖动、重复时间戳、时钟回退、稳定窗口和 timeout。 |
| `ros2_ws/src/robot_bringup/test/test_lifecycle_policy.py` | 覆盖 Lifecycle 状态决策、代次隔离、重试次数和退避上限。 |
| `ros2_ws/src/robot_bringup/test/test_ideal_localization_tf.py` | 验证 Ideal `map→odom` 的 frame、时间戳和单位变换。 |
| `ros2_ws/src/robot_bringup/test/test_activation_gate.py` | 使用 ROS adapter 替身测试 Gate 的状态查询、服务结果、Reset 恢复顺序和旧 future 隔离。 |
| `ros2_ws/src/robot_bringup/test/test_activation_gate_integration.py` | 在真实 rclpy executor 中回归异步服务、代次竞态和部分 RESUME 后的有序归一化。 |
| `ros2_ws/src/robot_bringup/test/test_initial_pose_policy.py` | 测试 `auto|rviz` 规范化和非法策略拒绝。 |
| `ros2_ws/src/robot_bringup/test/test_interactive_policy.py` | 覆盖模式专用 RViz 选择、headless 行为、Teleop 模式禁令和终端命令。 |
| `ros2_ws/src/robot_bringup/test/test_runtime_scripts.py` | 用隔离运行目录测试统一环境、单实例锁、安全清理、ROS supervisor 顺序退出、地图事务/manifest，以及 RViz/Teleop 启动脚本。 |

## 31. `robot_rviz_plugins`

| 文件 | 用途 |
| --- | --- |
| `ros2_ws/src/robot_rviz_plugins/CMakeLists.txt` | 以 C++17/Qt5 构建并安装项目 Nav2 Panel 共享库，导出 pluginlib 描述、头文件、许可证与 pytest 契约。 |
| `ros2_ws/src/robot_rviz_plugins/package.xml` | ament_cmake 包元数据；声明 Nav2、RViz、rclcpp/action、TF、Qt、YAML 与测试依赖。 |
| `ros2_ws/src/robot_rviz_plugins/plugins_description.xml` | 注册安全 Navigation 2 Panel 与 `robot_rviz_plugins/Voxel Grid` 两个 RViz class。 |
| `ros2_ws/src/robot_rviz_plugins/include/robot_rviz_plugins/nav2_panel.hpp` | 基于 Nav2 Panel 的项目命名空间头文件；增加可中断初始化线程、受管异步任务与退出状态成员。 |
| `ros2_ws/src/robot_rviz_plugins/src/nav2_panel.cpp` | 安全面板实现；除协作式停止 timer/QThread/QFuture 外，还防御空 GoalStatus 与非法循环输入，避免正常关闭或异常消息造成 RViz 崩溃。 |
| `ros2_ws/src/robot_rviz_plugins/include/robot_rviz_plugins/voxel_grid_display.hpp` | `nav2_msgs/msg/VoxelGrid` 的 RViz Display 声明。 |
| `ros2_ws/src/robot_rviz_plugins/src/voxel_grid_display.cpp` | 解码 Nav2 高 16 位 `MARKED` 体素为 PointCloud2，并复用 RViz 点云渲染器显示 3D 方盒。 |
| `ros2_ws/src/robot_rviz_plugins/LICENSE` | 此派生面板使用的完整 Apache License 2.0。 |
| `ros2_ws/src/robot_rviz_plugins/NOTICE.md` | 记录 Navigation2 `nav2_rviz_plugins` 1.3.12 来源、上游 SHA256 和本地退出安全修改。 |
| `ros2_ws/src/robot_rviz_plugins/test/test_safe_panel_contract.py` | 静态契约测试插件类名、协作式 QThread 退出、QFuture 所有权、ROS context guard 与许可证溯源。 |

## 32. `robot_teleop`

| 文件 | 用途 |
| --- | --- |
| `ros2_ws/src/robot_teleop/package.xml` | Python ROS 包元数据以及 geometry_msgs/rclpy/test 依赖。 |
| `ros2_ws/src/robot_teleop/setup.py` | 安装包、配置和 `keyboard_teleop` console script。 |
| `ros2_ws/src/robot_teleop/setup.cfg` | ROS Python 可执行文件安装到 libexec 的规则。 |
| `ros2_ws/src/robot_teleop/resource/robot_teleop` | ament resource index 标记。 |
| `ros2_ws/src/robot_teleop/config/teleop.yaml` | `/cmd_vel`、20 Hz 发布、0.18 秒 deadman、线/角速度和硬限幅。 |
| `ros2_ws/src/robot_teleop/robot_teleop/__init__.py` | Teleop Python 包标记。 |
| `ros2_ws/src/robot_teleop/robot_teleop/safety.py` | 与 ROS 解耦的按键映射、稳态墙钟 deadman、速度夹紧和最终零速度策略。 |
| `ros2_ws/src/robot_teleop/robot_teleop/keyboard_teleop.py` | 原始 TTY W/A/S/D/方向键适配器和 `/cmd_vel` ROS 节点；所有退出路径保证最终零速度。 |
| `ros2_ws/src/robot_teleop/test/test_safety.py` | 覆盖键位、边界值、超时停车、时间回退和关闭幂等性。 |
| `ros2_ws/src/robot_teleop/test/test_teleop_package_contract.py` | 检查配置、安装入口、终端按键解码和安全默认值；唯一模块名保证全工作区 pytest 可收集。 |

## 33. 修改文件时的依赖关系

常见改动不是只改一个文件：

| 改动 | 必须同步检查 |
| --- | --- |
| 轮径/轮距/joint | Isaac robot YAML、Wheel Odom YAML、Xacro、Nav2 Footprint和 joint 测试 |
| 传感器外参 | Isaac robot YAML static TF、Xacro sensors、投影高度、Map Pose/地图 |
| 出生点 | `spawn_poses.yaml`、Map Pose 标定、GT 变换、动态障碍 USD↔Map 坐标 |
| 动态障碍 | 对应的 Isaac physical `dynamic*.yaml` 与 ROS scenario `dynamic*.yaml` |
| Nav2 footprint/速度 | `nav2_params.yaml`、Collision Monitor polygons和验证场景 |
| Nav2 控制时序/负载 | `nav2_stable.yaml`、`nav2_performance.yaml`、MPPI `model_dt`、`mode_contract.py` 启动前约束和 profiler 实测 |
| Collision Monitor scan 源 | `nav2_params.yaml`；仅验证故障时再同步 `scan_fault_bridge.launch.py` 的 `/scan_fault` overlay，不要改生产默认源 |
| 地图版本 | 四个地图工件、manifest bundle、spawn Map Pose 的 version/hash 和所有 scenario version 字段 |
| RViz Nav2 面板 | `robot_rviz_plugins` 源码/plugin XML、`navigation.rviz`、preflight、退出测试和第三方 NOTICE |
| 自定义机器人 | custom project/robot YAML、USD、Xacro、Wheel Odom、Nav2、传感器 YAML、出生点和地图 |

更详细的修改流程见 [`user_manual.md`](user_manual.md#19-修改配置时应该改哪里) 和 [`development.md`](development.md)。

## 34. 如何确认索引没有漏文件

维护者在提交前可从仓库根目录执行下列双向集合差分。它把“已跟踪文件 + 尚未 `git add` 的交付源码”作为左集合，排除明确非交付的 `.tmp_runtime/`，再与本文文件表第一列的完整路径比较；命令无输出才表示既没有漏项，也没有指向已删除文件的陈旧条目。

```bash
comm -3 \
  <(git ls-files --cached --others --exclude-standard \
      | rg -v '^\.tmp_runtime/' | sort -u) \
  <(rg -o '^\| \x60[^\x60]+\x60' docs/repository_index.md \
      | sed 's/^| //' | tr -d '\140' | sort -u)
```

若将来引入新的生成目录，应先在本节解释为何它不是交付物，再加入左集合的排除规则；普通源码、配置、文档、测试、许可证和 fixture 不应靠目录通配说明代替逐文件索引。
