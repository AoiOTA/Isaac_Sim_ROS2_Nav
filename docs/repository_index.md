# 仓库文件索引

本文列出仓库中的全部 Git 跟踪文件，并解释它们的职责。构建产物、运行日志、批量实验结果和本地导入的 NVIDIA 资产受 `.gitignore` 管理，不属于源码索引。

使用项目请先阅读 [`user_manual.md`](user_manual.md)；修改文件前再用本索引确认它属于 Isaac 物理层、ROS 算法层、配置层还是验证层。

## 1. 顶层文件

| 文件 | 用途 |
| --- | --- |
| `.gitattributes` | 定义 Git LFS 规则；当前把 SLAM Toolbox `.posegraph` 作为 LFS 大文件管理。 |
| `.gitignore` | 排除构建目录、缓存、日志、官方资产、rosbag、批量结果和普通生成地图，同时放行精选 `warehouse_v1` 基线。 |
| `CONTRIBUTING.md` | 代码贡献、分支、提交消息、验证和数据管理约定。 |
| `LICENSE` | 项目源码的 Apache-2.0 许可证文本。 |
| `README.md` | GitHub 首页：项目能力、运行入口、验证状态以及主要文档导航。 |
| `plan.md` | 原始完整设计方案、技术选型、实施 SOP、实验指标和分阶段验收标准；用于理解“为什么这样设计”。 |
| `pyproject.toml` | Python 3.12 项目元数据、PyYAML/开发依赖及 pytest 搜索路径和 marker 配置。 |

## 2. 文档

| 文件 | 用途 |
| --- | --- |
| `docs/user_manual.md` | 面向使用者的中文操作手册，从 clone、构建到导航、建图、实验和排障。 |
| `docs/repository_index.md` | 本文件；逐项解释所有 Git 跟踪文件。 |
| `docs/interfaces.md` | 运行时权威契约：模式配对、Topic、Message、QoS、TF 所有权、Reset 和 Nav2 激活门。 |
| `docs/calibration.md` | USD Pose 与 Map Pose 的标定、复测、版本化和动态障碍坐标重对齐流程。 |
| `docs/verification.md` | 证据台账：已通过的运行/测试结果、已知 Nav2 诊断以及尚未验收的范围。 |
| `docs/development.md` | 开发环境、调试命令、测试方式、运行探针和提交/数据纪律。 |

## 3. 数据目录

| 文件 | 用途 |
| --- | --- |
| `data/README.md` | 说明地图、bag、轨迹、指标、报告和实验输出的存放及版本管理策略。 |
| `data/bags/.gitkeep` | 保留本地 rosbag 输出目录；实际 bag 默认不提交。 |
| `data/experiment_runs/.gitkeep` | 保留自动实验 CSV/JSON 输出目录；实际批次默认不提交。 |
| `data/metrics/.gitkeep` | 保留聚合指标输出目录。 |
| `data/reports/.gitkeep` | 保留分析报告、比较结果和图表输出目录。 |
| `data/trajectories/.gitkeep` | 保留估计轨迹与 Ground Truth 轨迹输出目录。 |
| `data/maps/manifests/warehouse_v1.yaml` | `warehouse_v1` 地图版本的来源、尺寸、SHA256、坐标原点和标定记录；preflight 以此校验工件。 |
| `data/maps/occupancy/.gitkeep` | 保留 OccupancyGrid 目录。 |
| `data/maps/occupancy/warehouse_v1.yaml` | ROS Map Server 元数据：PGM 文件、分辨率、原点和占据阈值。 |
| `data/maps/occupancy/warehouse_v1.pgm` | `warehouse_v1` 二值/三值占据栅格图；Nav2 的静态地图来源。 |
| `data/maps/occupancy/warehouse_v1_preview.png` | 方便人工浏览地图的预览图，不参与导航。 |
| `data/maps/posegraphs/.gitkeep` | 保留 SLAM Toolbox 序列化地图目录。 |
| `data/maps/posegraphs/warehouse_v1.posegraph` | Git LFS 管理的 SLAM Toolbox Pose Graph 主文件；Localization 用它做扫描匹配。 |
| `data/maps/posegraphs/warehouse_v1.data` | 与 `.posegraph` 配套的序列化传感器/数据文件；两者不能混用不同版本。 |

## 4. 操作脚本

| 文件 | 用途 |
| --- | --- |
| `scripts/lib/common.sh` | 所有脚本共享的项目根、Isaac/ROS 路径、DDS 默认值、依赖检查和 `source_ros` 函数。 |
| `scripts/preflight.sh` | 启动前检查 Isaac 版本、官方资产、ROS 包、GPU、Git LFS 地图和 manifest SHA256。 |
| `scripts/import_assets.sh` | 调用 Isaac Python，把官方 Jackal 的最小依赖复制到本地项目资产目录并校验 hash。 |
| `scripts/build_ros2.sh` | source ROS 环境并执行 `colcon build --symlink-install`。 |
| `scripts/test.sh` | 统一运行纯 Python、ROS colcon 和可选 Isaac/USD 测试。 |
| `scripts/run_isaac.sh` | 选择项目配置并用 Isaac Python 启动 standalone 仿真；支持 custom profile。 |
| `scripts/run_ros.sh` | 启动四种顶层 ROS 操作，并为 Localization/Navigation 自动推导同名 OccupancyGrid。 |
| `scripts/save_map.sh` | 原子保存 OccupancyGrid 和 Pose Graph，拒绝覆盖同名版本并清理失败的半成品。 |

## 5. Isaac Sim 包入口与主程序

| 文件 | 用途 |
| --- | --- |
| `isaac_sim/__init__.py` | Isaac 项目 Python 包标记及顶层说明。 |
| `isaac_sim/apps/__init__.py` | standalone 应用子包标记。 |
| `isaac_sim/apps/navigation_sim.py` | Isaac 主入口：解析 CLI、加载配置、组合 Stage、创建传感器/图、启动 ROS 节点并运行仿真循环。 |

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
| `isaac_sim/configs/project.yaml` | 默认 Jackal 项目总配置；串联环境、机器人、仿真模式、出生点、ROS、GT 和所有子配置文件。 |
| `isaac_sim/configs/custom_robot.project.yaml` | 自定义机器人项目模板；要求显式环境变量提供真实 USD、defaultPrim 和传感器配置。 |
| `isaac_sim/configs/spawn_poses.yaml` | 出生点唯一真源；同时保存物理 USD Pose、已标定 Map Pose 及不确定度。 |
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
| `isaac_sim/configs/sensors/camera.yaml` | 双目/相机 frame 与可选发布配置；当前导航不使用图像。 |
| `isaac_sim/configs/ros2_bridge/topics.yaml` | Isaac OmniGraph 使用的 ROS Topic 和 frame 名称集中表。 |
| `isaac_sim/configs/ros2_bridge/qos.yaml` | Clock、sensor、command、state、TF、static TF 的显式 QoS profiles。 |
| `isaac_sim/configs/experiments/static.yaml` | 静态实验 seed、场景 ID 和目标列表的占位清单；当前 Isaac 运行时不读取此文件。 |
| `isaac_sim/configs/experiments/dynamic.yaml` | Isaac 物理动态障碍物 ID、形状、尺寸、USD 轨迹、速度和 repeat。 |
| `isaac_sim/configs/experiments/incremental_mapping.yaml` | Isaac 增量地图变化占位配置；真实 changed-region 资产尚需另行制作。 |

## 10. Isaac OmniGraph 定义

| 文件 | 用途 |
| --- | --- |
| `isaac_sim/graphs/__init__.py` | OmniGraph 构建子包标记。 |
| `isaac_sim/graphs/spec.py` | 与 Isaac API 解耦的 GraphSpec/TargetPaths 数据结构和 materialize 工具，便于纯 Python 测试图契约。 |
| `isaac_sim/graphs/ros_contract.py` | 严格读取 Topic/QoS 配置并提供图构建所需的 ROS 合同。 |
| `isaac_sim/graphs/control_graph.py` | `/cmd_vel`→DifferentialController→前后轮 ArticulationController 数据流。 |
| `isaac_sim/graphs/odometry_graph.py` | Ideal 模式的 `/odom` 和 `odom → base_link` 发布图。 |
| `isaac_sim/graphs/sensor_graph.py` | `/clock`、JointState、IMU、RTX LiDAR PointCloud2 等传感器发布图。 |
| `isaac_sim/graphs/tf_graph.py` | 轮子动态 TF 与传感器/相机静态 TF 图；从所选 robot YAML 读取外参。 |

## 11. Isaac 配置和通用工具

| 文件 | 用途 |
| --- | --- |
| `isaac_sim/src/__init__.py` | Isaac 核心源码包标记。 |
| `isaac_sim/src/config.py` | 严格解析总配置、环境变量替换、嵌套 override、模式组合和必需路径。 |
| `isaac_sim/src/yaml_utils.py` | 通用 YAML mapping、字段、数值、向量和未知 key 校验函数。 |

## 12. Isaac Stage 管理

| 文件 | 用途 |
| --- | --- |
| `isaac_sim/src/stage/__init__.py` | Stage 子包标记。 |
| `isaac_sim/src/stage/stage_loader.py` | 创建/打开项目 Stage，管理 Sublayer、Reference、Xform 和保存。 |
| `isaac_sim/src/stage/scene_composer.py` | 按“环境 Sublayer + 机器人 Reference”组合最终 Stage，并保持项目层为 edit target。 |
| `isaac_sim/src/stage/physics_setup.py` | 查找并校验唯一 PhysicsScene，设置重力、时间步和 PhysX 参数。 |
| `isaac_sim/src/stage/asset_validator.py` | 检查 defaultPrim、依赖、关键 Prim、传感器 frame、Articulation、wheel joint/碰撞和 GT 隔离。 |

## 13. Isaac 机器人运行时

| 文件 | 用途 |
| --- | --- |
| `isaac_sim/src/robot/__init__.py` | 机器人运行时子包标记。 |
| `isaac_sim/src/robot/articulation_runtime.py` | 获取 Jackal articulation、应用物理参数、控制/读取关节并处理 sleep/wake。 |
| `isaac_sim/src/robot/joint_validator.py` | 验证四个 wheel joint 的存在、分组、方向映射和 DOF 顺序。 |
| `isaac_sim/src/robot/spawn_pose_manager.py` | 读取/校验命名 USD/Map Pose，提供标定门和 Pose 查询。 |
| `isaac_sim/src/robot/reset.py` | ResetManager 事务：停控、恢复 Pose、清速度、重置子系统和恢复 timeline。 |
| `isaac_sim/src/robot/idle_brake.py` | 低层命令超时/死区 watchdog；无有效命令时让底盘可靠静止但允许低速唤醒。 |

## 14. Isaac 传感器与 ROS Bridge

| 文件 | 用途 |
| --- | --- |
| `isaac_sim/src/sensors/__init__.py` | 传感器子包标记。 |
| `isaac_sim/src/sensors/sensor_factory.py` | 根据配置创建/校验 RTX LiDAR、IMU、相机 render product 和传感器 Prim。 |
| `isaac_sim/src/bridge/__init__.py` | ROS Bridge 子包标记。 |
| `isaac_sim/src/bridge/ros_graph_builder.py` | 按选定模式组合控制、传感器、里程计和结构 TF Graph。 |
| `isaac_sim/src/bridge/tf_ownership.py` | 计算并拒绝 Ideal/Realistic、Isaac/RSP 之间的重复 TF 所有权组合。 |
| `isaac_sim/src/bridge/reset_service.py` | `/simulation/reset` 服务、Wheel/EKF/Costmap reset、fresh-scan `/initialpose` 和 localization-seeded 事件。 |

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
| `isaac_sim/tests/test_asset_paths.py` | 测试资产 manifest、项目 overlay 只引用本地导入以及路径可复现性。 |
| `isaac_sim/tests/test_stage_composition.py` | Isaac/USD marker 测试：Stage 组合、唯一 PhysicsScene、defaultPrim 和 articulation 结构。 |
| `isaac_sim/tests/test_graph_contracts.py` | 测试 Topic/QoS、控制/传感器/TF Graph 节点连接和 GT 隔离。 |
| `isaac_sim/tests/test_joint_mapping.py` | 测试四轮 joint 分组和控制顺序。 |
| `isaac_sim/tests/test_scan_projection.py` | 检查 Isaac LiDAR 与 ROS LaserScan 投影参数的 frame/range/角度契约。 |
| `isaac_sim/tests/test_ground_truth_transforms.py` | 测试 `map_T_usd` 与 Pose 变换数学。 |
| `isaac_sim/tests/test_idle_brake.py` | 测试命令死区、超时停车、低速唤醒和 sim-time 行为。 |
| `isaac_sim/tests/test_spawn_pose_reset.py` | 测试出生点标定门、Reset 顺序、fresh-scan initial pose 和重复 seed。 |
| `isaac_sim/tests/test_dynamic_obstacles.py` | 测试动态障碍解析、有限数、repeat、轨迹推进和 Reset。 |

## 18. ROS 工作区总览

`ros2_ws/src/` 包含 8 个包：

| 包 | 职责 |
| --- | --- |
| **robot_description** | Xacro、RViz RobotModel 和可选 Robot State Publisher。 |
| **robot_perception** | PointCloud2→LaserScan 和可选 self filter。 |
| **robot_odometry** | 四轮编码器运动学和 `/wheel/odom`。 |
| **robot_localization_config** | IMU+Wheel Odom 的 EKF。 |
| **robot_mapping** | SLAM Toolbox Mapping/Localization 和 Map Server。 |
| **robot_navigation** | Nav2 planner/controller/costmap/safety。 |
| **robot_experiments** | 初始位姿、Reset runner、场景、指标和报告。 |
| **robot_bringup** | 模式验证、组合 launch 和 Nav2 readiness gate。 |

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
| `ros2_ws/src/robot_description/rviz/navigation.rviz` | 预置 RobotModel、LaserScan 和 Map 三类 RViz 显示。 |
| `ros2_ws/src/robot_description/test/test_urdf.py` | 展开 Xacro，检查必需 link/joint、轮轴、传感器固定关节、禁用导航/GT frame 和 description-only TF 所有权。 |

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
| `ros2_ws/src/robot_mapping/config/slam_localization.yaml` | Localization 的 scan matching、buffer、TF 和已保存 Pose Graph 参数。 |
| `ros2_ws/src/robot_mapping/launch/mapping.launch.py` | 启动 async SLAM；基线模式新建图，增量模式加载 `.posegraph/.data`。 |
| `ros2_ws/src/robot_mapping/launch/localization.launch.py` | 启动 Map Server 和 Localization SLAM；`/map` 固定，SLAM 诊断图重映射。 |
| `ros2_ws/src/robot_mapping/test/test_mapping_modes.py` | 检查两个模式的 executable、参数互斥、Map Server 和话题隔离。 |

## 24. `robot_navigation`

| 文件 | 用途 |
| --- | --- |
| `ros2_ws/src/robot_navigation/CMakeLists.txt` | 安装 Nav2 配置/launch并注册测试。 |
| `ros2_ws/src/robot_navigation/package.xml` | Nav2 planner/controller/behavior/smoother/collision monitor/lifecycle 依赖。 |
| `ros2_ws/src/robot_navigation/config/nav2_params.yaml` | 全局/局部 Costmap、SmacPlanner2D、MPPI、BT、Velocity Smoother、Collision Monitor 的统一参数。 |
| `ros2_ws/src/robot_navigation/launch/navigation.launch.py` | 创建 Nav2 lifecycle 节点；默认不 autostart，等待外部 readiness gate。 |
| `ros2_ws/src/robot_navigation/test/test_nav2_config.py` | 检查插件、Footprint、inflation、话题链、2D obstacle layer 和安全参数。 |

## 25. `robot_experiments` 包装文件与配置

| 文件 | 用途 |
| --- | --- |
| `ros2_ws/src/robot_experiments/package.xml` | 实验 ROS 包元数据及 Nav2 action、TF、lifecycle、YAML 等依赖。 |
| `ros2_ws/src/robot_experiments/setup.py` | 安装配置/launch并注册 experiment runner、initial pose、incremental comparator 三个 CLI。 |
| `ros2_ws/src/robot_experiments/setup.cfg` | Python ROS 可执行文件安装规则和 flake8 配置。 |
| `ros2_ws/src/robot_experiments/resource/robot_experiments` | ament resource index 标记。 |
| `ros2_ws/src/robot_experiments/config/scenario.schema.yaml` | Static/Dynamic/Incremental 场景 YAML 的结构、类型和必需字段定义。 |
| `ros2_ws/src/robot_experiments/config/static.yaml` | 1 m Ideal/Realistic 静态 smoke：4 seeds、目标、阈值和配置引用。 |
| `ros2_ws/src/robot_experiments/config/static_long_range.yaml` | 3 m 长距离 smoke：单 seed 和更长 timeout。 |
| `ros2_ws/src/robot_experiments/config/dynamic.yaml` | ROS 动态基线：目标、4 seeds、两条 Map-frame 障碍轨迹和 repeat 契约。 |
| `ros2_ws/src/robot_experiments/config/incremental_mapping.yaml` | 增量建图工作流描述符；NavigateToPose runner 会明确拒绝它。 |
| `ros2_ws/src/robot_experiments/config/incremental_comparison.example.yaml` | 三地图离线比较模板；路径、耗时、真实变化矩形为必填占位。 |

## 26. `robot_experiments` 运行代码

| 文件 | 用途 |
| --- | --- |
| `ros2_ws/src/robot_experiments/robot_experiments/__init__.py` | 实验 Python 包标记。 |
| `ros2_ws/src/robot_experiments/robot_experiments/configuration.py` | 通用配置加载、相对路径解析和严格字段工具。 |
| `ros2_ws/src/robot_experiments/robot_experiments/spawn_poses.py` | ROS 侧读取命名 USD/Map Pose并要求已标定。 |
| `ros2_ws/src/robot_experiments/robot_experiments/scenario.py` | 场景 dataclass/loader、导航场景门、Isaac 动态运行时和物理几何契约校验。 |
| `ros2_ws/src/robot_experiments/robot_experiments/metrics.py` | 单轮成功判定、路径长度、角度 wrap、避障率和增量时间改善等纯指标函数。 |
| `ros2_ws/src/robot_experiments/robot_experiments/report.py` | manifest schema 校验、配置 SHA256，以及单轮 CSV/JSON 报告的原子写入。 |
| `ros2_ws/src/robot_experiments/robot_experiments/initial_pose_publisher.py` | 冷启动/增量 Mapping 发布标定 `/initialpose`，可等待 odom→base TF。 |
| `ros2_ws/src/robot_experiments/robot_experiments/experiment_runner.py` | 自动多轮 Reset、恢复门、NavigateToPose、cancel 隔离、观测/指标和报告主节点。 |
| `ros2_ws/src/robot_experiments/robot_experiments/incremental_map_compare.py` | 离线加载三张 Map Server YAML/PGM，按世界坐标比较耗时、变化恢复和旧区退化。 |
| `ros2_ws/src/robot_experiments/launch/initial_pose.launch.py` | 把 spawn pose 参数传给 initial pose publisher。 |
| `ros2_ws/src/robot_experiments/launch/experiment.launch.py` | 启动 experiment runner并传入场景、出生点、输出目录和可选配置 override。 |

## 27. `robot_experiments` 测试

| 文件 | 用途 |
| --- | --- |
| `ros2_ws/src/robot_experiments/test/fixtures/spawn_poses_calibrated.yaml` | 已标定出生点的最小测试 fixture。 |
| `ros2_ws/src/robot_experiments/test/fixtures/spawn_poses_uncalibrated.yaml` | 未标定出生点 fixture，用于验证 fail-fast。 |
| `ros2_ws/src/robot_experiments/test/test_configuration.py` | 测试三种场景解析、schema、相对路径、静/动态契约和真实物理配置一致性。 |
| `ros2_ws/src/robot_experiments/test/test_metrics.py` | 测试成功/失败原因、路径、角度、率和时间改善指标。 |
| `ros2_ws/src/robot_experiments/test/test_report.py` | 测试 manifest 必需字段、有限数/hash、CSV/JSON 原子替换和输出路径安全。 |
| `ros2_ws/src/robot_experiments/test/test_incremental_map_compare.py` | 用合成 PGM 测试变化恢复、旧区退化、阈值 override 和 CLI 返回码。 |
| `ros2_ws/src/robot_experiments/test/test_package_contract.py` | 检查安装入口、配置/launch 文件和 reset/dynamic contract 关键代码存在。 |
| `ros2_ws/src/robot_experiments/test/test_ros_adapters.py` | 在 ROS 环境中测试消息→内部 sample 的 adapter 和时间戳保存。 |

## 28. `robot_bringup` 配置与入口

| 文件 | 用途 |
| --- | --- |
| `ros2_ws/src/robot_bringup/package.xml` | 顶层 bringup 包元数据，依赖其余 7 个项目包以及 activation gate 所需的 ROS 消息/运行库。 |
| `ros2_ws/src/robot_bringup/setup.py` | 安装配置/launch并注册 `nav2_activation_gate`。 |
| `ros2_ws/src/robot_bringup/setup.cfg` | Python ROS 可执行文件安装规则。 |
| `ros2_ws/src/robot_bringup/resource/robot_bringup` | ament resource index 标记。 |
| `ros2_ws/src/robot_bringup/config/modes.yaml` | 人类可读的四种 operation、里程计和 TF 所有权矩阵。 |
| `ros2_ws/src/robot_bringup/config/activation_gate.yaml` | Nav2 readiness 的 freshness、TF 稳定窗口、容差和 timeout。 |
| `ros2_ws/src/robot_bringup/launch/mapping_bringup.launch.py` | Baseline Mapping 顶层入口。 |
| `ros2_ws/src/robot_bringup/launch/incremental_mapping_bringup.launch.py` | 加载旧 Pose Graph 继续 Mapping 的顶层入口。 |
| `ros2_ws/src/robot_bringup/launch/localization_bringup.launch.py` | Map Server + SLAM Localization 顶层入口。 |
| `ros2_ws/src/robot_bringup/launch/navigation_bringup.launch.py` | Localization + Nav2 + readiness gate 顶层入口。 |
| `ros2_ws/src/robot_bringup/launch/ros_stack.launch.py` | 四种入口共享的组合器；校验模式/文件并按条件包含描述、感知、odom、SLAM、Nav2。 |

## 29. `robot_bringup` 代码与测试

| 文件 | 用途 |
| --- | --- |
| `ros2_ws/src/robot_bringup/robot_bringup/__init__.py` | bringup Python 包标记。 |
| `ros2_ws/src/robot_bringup/robot_bringup/mode_contract.py` | 纯函数校验 operation、Ideal/Realistic、Isaac/RSP、Pose Graph、Map和自定义文件。 |
| `ros2_ws/src/robot_bringup/robot_bringup/readiness.py` | 纯状态机判断 Clock/scan/odom/map 与 map→odom 新鲜和稳定。 |
| `ros2_ws/src/robot_bringup/robot_bringup/activation_gate.py` | ROS 节点实现 readiness 订阅/TF 查询，并请求 Nav2 lifecycle STARTUP。 |
| `ros2_ws/src/robot_bringup/test/test_mode_contract.py` | 覆盖合法/非法模式、缺失地图、后缀归一化和自定义文件入口。 |
| `ros2_ws/src/robot_bringup/test/test_readiness.py` | 覆盖 freshness、TF 抖动、重复时间戳、时钟回退、稳定窗口和 timeout。 |

## 30. 修改文件时的依赖关系

常见改动不是只改一个文件：

| 改动 | 必须同步检查 |
| --- | --- |
| 轮径/轮距/joint | Isaac robot YAML、Wheel Odom YAML、Xacro、Nav2 Footprint和 joint 测试 |
| 传感器外参 | Isaac robot YAML static TF、Xacro sensors、投影高度、Map Pose/地图 |
| 出生点 | `spawn_poses.yaml`、Map Pose 标定、GT 变换、动态障碍 USD↔Map 坐标 |
| 动态障碍 | Isaac physical `dynamic.yaml` 与 ROS scenario `dynamic.yaml` |
| Nav2 footprint/速度 | `nav2_params.yaml`、Collision Monitor polygons和验证场景 |
| 地图版本 | 四个地图工件、manifest、spawn Map Pose和所有 scenario version 字段 |
| 自定义机器人 | custom project/robot YAML、USD、Xacro、Wheel Odom、Nav2、传感器 YAML、出生点和地图 |

更详细的修改流程见 [`user_manual.md`](user_manual.md#18-修改配置时应该改哪里) 和 [`development.md`](development.md)。
