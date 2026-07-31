# 当前实现文件索引

> 最近复核：2026-07-31<br>
> 适用分支：`feat/planning-risk-fusion-v0.1`

本索引只列出当前 Kujiale `warehouse_new` 导航与 4×20 实验的操作入口、权威配置、
实现和测试。构建产物、运行日志、批量证据、外观预览输出和已不再作为操作入口的兼容代码，
均不在此列出。

## 从哪里开始

| 目标 | 首先阅读/运行 |
| --- | --- |
| 快速理解项目与正式结果 | [`../README.md`](../README.md)、[`verification.md`](verification.md) |
| 正常运行、GUI/RViz 单轮或一键批量 | [`user_manual.md`](user_manual.md) |
| 4×20 矩阵、外观、报告、复测 | [`kujiale_4x20_appearance_benchmark_plan.md`](kujiale_4x20_appearance_benchmark_plan.md) |
| 路线、航点、静态障碍和动态 actor | [`kujiale_long_route_map.md`](kujiale_long_route_map.md) |
| 指标定义与交付表述 | [`kujiale_4x20_metric_definitions.md`](kujiale_4x20_metric_definitions.md) |
| 启动、续跑、pilot 或报告问题 | [`troubleshooting.md`](troubleshooting.md)、[`kujiale_4x20_execution_lessons.md`](kujiale_4x20_execution_lessons.md) |
| Topic、TF、模式和配置所有权 | [`interfaces.md`](interfaces.md) |
| Module2 规划/风险如何接入 Nav2 | [`module2_nav2_planning_risk_fusion.md`](module2_nav2_planning_risk_fusion.md) |
| RViz 中如何观察 Module2 | [`module2_rviz_visualization.md`](module2_rviz_visualization.md) |

## 文档

| 文件 | 当前职责 |
| --- | --- |
| `README.md` | 项目总览、结果摘要、视频和常用命令。 |
| `docs/user_manual.md` | 运行、GUI/RViz 单轮、4×20、外观预览与日常排障的用户手册。 |
| `docs/kujiale_4x20_appearance_benchmark_plan.md` | 当前正式实验规格与唯一批量运行流程。 |
| `docs/kujiale_4x20_metric_definitions.md` | 静态/动态避障成功率、路径偏差、导航成功率的通用公式和文字定义。 |
| `docs/kujiale_long_route_map.md` | `warehouse_new` 路线、G1–G5、六个静态障碍与三阶段 actor 参数。 |
| `docs/verification.md` | 正式 campaign 的结果、适用边界和复核方法。 |
| `docs/kujiale_4x20_execution_lessons.md` | pilot、supervisor、续跑、动态重跑、报告与推送的恢复规则。 |
| `docs/interfaces.md` | 运行时 Topic、TF、地图、模式与 Nav2 profile 契约。 |
| `docs/calibration.md` | 当前地图 bundle、出生点和新地图接入流程。 |
| `docs/troubleshooting.md` | 启动、DDS、TF、Reset、RViz、Nav2 和 campaign 的症状式排障。 |
| `docs/development.md` | 开发环境和当前代码改动的验证命令。 |
| `docs/documentation_status.md` | 当前文档的事实来源和维护规则。 |
| `docs/branch_governance.md` | 历史源仓 `main`、开放 PR 分支、发布仓 baseline 与离线 archive 的边界。 |
| `docs/module2_nav2_planning_risk_fusion.md` | Module2 tie-break、Global Costmap 风险、身份门控和传统 fallback。 |
| `docs/module2_rviz_visualization.md` | Marker namespaces、raw costmap 配色、操作与诊断。 |
| `docs/reproduction/planning-risk-fusion-v0.1.md` | Integration underlay、构建测试、身份 profile 和故障回退复现。 |

## 操作脚本

| 文件 | 当前职责 |
| --- | --- |
| `scripts/setup_ros_env.sh` | 加载 ROS 2 Jazzy 与工作区环境；任何 ROS 命令前先 source。 |
| `scripts/build_ros2.sh` | 构建 Module3；融合分支会从 Integration underlay 唯一加载 `bio_nav_interfaces`。 |
| `scripts/generate_bionav_fusion_profile.py` | 用真实 map/qualification/model SHA 生成 fail-closed 的可选融合 profile。 |
| `scripts/run_kujiale_4x20_all.sh` | 推荐的一键监督器：构建、静态40轮、动态40轮、阶段报告和总报告；支持 `--resume` 与 `--dynamic-only`。 |
| `scripts/run_kujiale_4x20.sh` | 4×20 控制器：预检、pilot、配对批次、报告与状态查询。 |
| `scripts/run_kujiale_4x20_isaac.sh` | 静态或动态 4×20 阶段的 Isaac 启动器。 |
| `scripts/run_ros.sh` | Navigation、Localization、Mapping 等 ROS bringup 入口。 |
| `scripts/run_visual_route.sh` | 静态 GUI/RViz 全屋路线 `G2 → G3 → G4 → G5 → G1` 自动发送器。 |
| `scripts/run_kujiale_dynamic_isaac.sh` | 动态 GUI/RViz 的三阶段 actor Isaac 启动器。 |
| `scripts/run_kujiale_three_stage_visual.sh` | 动态 GUI/RViz 全屋或单段 actor 诊断自动发送器。 |
| `scripts/capture_kujiale_appearance_preview.sh` | 固定客厅第三人称机位导出五档外观的高分辨率可点击预览。 |
| `scripts/generate_kujiale_long_route_maps.py` | 从当前地图和场景 YAML 生成静态、动态和 2×2 实验矩阵示意图。 |
| `scripts/preflight.sh` | 对地图、场景、ROS 依赖和运行环境的通用只读预检。 |
| `scripts/diagnose.sh`、`scripts/clean_runtime.sh` | 定位或受管清理已确认的运行时残留；先执行 `--dry-run`。 |

## 实验与场景配置

| 文件 | 当前职责 |
| --- | --- |
| `data/maps/manifests/warehouse_new.yaml` | 当前地图四工件、哈希、场景和标定的权威 bundle 清单。 |
| `data/maps/occupancy/warehouse_new.yaml` / `.pgm` | 4×20、GUI、Nav2 与报告绘图共同使用的 OccupancyGrid。 |
| `data/maps/posegraphs/warehouse_new.posegraph` / `.data` | 与该 OccupancyGrid 绑定的 Pose Graph 工件。 |
| `isaac_sim/configs/environments/kujiale_0026_A_to_B_door_open.spawn.yaml` | `mapping_start`、G1、G2、G5 的当前 USD/map 出生点契约。 |
| `isaac_sim/configs/experiments/kujiale_long_range_static.yaml` | 六个固定静态 RGB-D 障碍的当前物理配置。 |
| `isaac_sim/configs/experiments/kujiale_long_range_dynamic.yaml` | `local_bypass`、`g2_g3_exit`、`g5_g1_crossing` 的当前 actor、gate、运动学与生命周期。 |
| `isaac_sim/configs/experiments/kujiale_appearance_profiles.yaml` | 五档 Session Layer 外观配置及其哈希输入。 |
| `ros2_ws/src/robot_experiments/config/kujiale_4x20_static_pair.yaml` | 静态基准与静态＋外观的40轮矩阵。 |
| `ros2_ws/src/robot_experiments/config/kujiale_4x20_dynamic_pair.yaml` | 动态基准与动态＋外观的40轮矩阵。 |
| `ros2_ws/src/robot_navigation/config/nav2_params.yaml` | `stable` 的静态 Nav2 基线。 |
| `ros2_ws/src/robot_navigation/config/nav2_dynamic_avoidance.yaml` | `dynamic_avoidance` 的 MPPI、STVL 与动态避障覆盖层。 |
| `ros2_ws/src/robot_navigation/config/nav2_bio_nav_planning_only.yaml` | 只启用认知规划 tie-break 的显式 profile。 |
| `ros2_ws/src/robot_navigation/config/nav2_bio_nav_risk_only.yaml` | 只启用 Global Costmap 认知软风险的显式 profile。 |
| `ros2_ws/src/robot_navigation/config/nav2_bio_nav_tiebreak_risk.yaml` | 同时启用规划与风险的显式 fail-closed 模板。 |
| `ros2_ws/src/robot_perception/config/self_filter_optional.yaml` | Navigation 近场安全点云的 padded-footprint 自滤波边界。 |
| `ros2_ws/src/robot_perception/config/pointcloud_to_laserscan_safety.yaml` | `/scan_safety` 的独立投影合同；保留旧投影参数并使用 `range_min=0.05 m`。 |

## 实现与测试

| 文件 | 当前职责 |
| --- | --- |
| `isaac_sim/apps/navigation_sim.py` | Isaac 场景、机器人、传感器、障碍、外观 Session Layer 与 ROS Bridge 的主入口。 |
| `ros2_ws/src/bio_nav_fusion/` | `BioNavGridBased` 与 `CognitiveRiskLayer` 插件；接口来自 Integration。 |
| `isaac_sim/apps/appearance_preview.py` | 无 ROS 的客厅外观预览渲染入口。 |
| `ros2_ws/src/robot_experiments/robot_experiments/experiment_runner.py` | 单轮/批量 reset、目标序列、证据与严格结果写入。 |
| `ros2_ws/src/robot_experiments/robot_experiments/kujiale_4x20_campaign.py` | 4×20 校验、统计、GT 轨迹、HTML/PDF/Markdown 报告生成；已发布快照自动采用 GitHub Raw 图片链接。 |
| `ros2_ws/src/robot_experiments/launch/experiment.launch.py` | 4×20 runner 的 ROS launch 参数类型与运行入口。 |
| `ros2_ws/src/robot_perception/src/lidar_self_filter_node.cpp` | `/lidar/points_raw` 到 `/lidar/points_scan` 的 TF、自体点删除与 fail-closed 节点。 |
| `ros2_ws/src/robot_perception/src/lidar_self_filter_core.cpp` | 保留 PointCloud2 字段/时间戳的 padded-footprint 过滤核心。 |
| `scripts/analyze_collision_evidence.py` | 冻结碰撞证据的 scan/costmap/MPPI/速度链/actor/静态地图关联分析器。 |
| `scripts/probe_nearfield_safety_contract.py` | 自动启动双投影链，验证自滤波、旧 `/scan` 隔离、0.30 m 安全观测与端到端延迟的合成探针。 |
| `isaac_sim/tests/test_appearance.py` | 外观 profile、Session Layer 与状态契约测试。 |
| `isaac_sim/tests/test_appearance_preview.py` | 客厅预览参数、渲染和输出契约测试。 |
| `isaac_sim/tests/test_dynamic_obstacles.py` | 动态 actor、地图坐标、运动学和可见性契约测试。 |
| `ros2_ws/src/robot_experiments/test/test_kujiale_4x20_campaign.py` | 矩阵、证据、门槛、筛选 GT 轨迹、HTML/PDF/PNG/便携报告测试。 |
| `ros2_ws/src/robot_bringup/test/test_nav2_profile_contract.py` | `stable` 与 `dynamic_avoidance` 启动/参数约束测试。 |

## 运行数据（不提交 Git）

| 目录 | 内容 |
| --- | --- |
| `data/experiment_runs/kujiale_4x20_<ID>/` | 原始逐轮证据、orchestrator 日志、隔离的不完整轮次。 |
| `data/reports/kujiale_4x20_<ID>/` | 总报告、静态/动态子报告和可移交的 `index_portable.html`。 |
| `docs/report_assets/README.md`、`docs/report_assets/<report-dir>/` | 已发布 campaign 的 PNG 快照及发布规则；用于让报告 HTML 通过 GitHub Raw 链接跨电脑显示图片。 |
| `data/appearance_previews/` | 手工生成的外观核验图片与预览页面，不是正式实验结果。 |

修改启动参数、地图、actor、外观矩阵、Nav2 profile 或验收规则时，必须在同一提交中更新本索引、
相应运行手册、接口契约和验证台账；受影响的正式结论需要用新的 campaign 重新取得。
