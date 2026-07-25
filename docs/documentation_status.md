# 文档状态与事实来源

> 最近复核：2026-07-25
>
> 适用分支：`codex/kujiale-4x20-appearance-benchmark`

本仓库同时保留可执行手册、运行契约、设计方案和问题复盘。它们的用途不同；
执行命令或判断当前行为时，始终以脚本、配置和测试所对应的“当前”文档为准。

## 当前事实来源（优先级从高到低）

1. 运行脚本与启动校验：`scripts/run_isaac.sh`、`scripts/run_ros.sh`；
2. 配置与实现：`isaac_sim/configs/`、`ros2_ws/src/robot_navigation/config/nav2_params.yaml`、Launch 文件；
3. 自动测试：对应包的 `test/`；
4. 当前运行文档：[`user_manual.md`](user_manual.md)、[`interfaces.md`](interfaces.md)、[`troubleshooting.md`](troubleshooting.md)；
5. 设计方案和历史复盘：仅解释背景、取舍与当时证据，不覆盖以上事实来源。

当前分支的关键运行事实如下：

- Localization/Navigation 默认地图是酷家乐 `warehouse_new`；
- `warehouse_new` 只批准普通 Ideal Localization/Navigation，Realistic 或显式 Pose Graph 定位会被启动脚本拒绝；
- 标准人工导航使用两个终端、受管 RViz 和 **2D Goal Pose**，不存在项目私有目标桥；
- Nav2 有两套明确 profile：默认 `stable` 完整复现静态 20 轮基线（Local + Global 标准 `VoxelLayer`）；`dynamic_avoidance` 使用 Local STVL 时序清除且 Global Costmap 不接收 RGB-D，避免移动 actor 留下全局残影；Collision Monitor 两套 profile 均只使用 `/scan`；
- 三阶段动态可视化的现行入口是 `run_kujiale_dynamic_isaac.sh`、`run_kujiale_three_stage_visual.sh` 与 `nav2_profile:=dynamic_avoidance`。G1→G2、G2→G3、G5→G1 分别使用独立 actor，成功到达下一航点后才退役；
- 当前正式操作入口是 `run_kujiale_4x20_all.sh`：一条命令自动管理静态/动态两套栈、四组各20轮、报告和`--resume`。匿名 USD Session Layer、成对调度、预检和报告均已实现；当前尚未执行该80轮，因此不能宣称四组中的任一组通过或算法已达到90%。
- 单轮 GUI/RViz 诊断提供静态/动态自动完整 G2–G5–G1 路线；它们均不计入4×20证据。
- 历史静态候选批次 `kujiale_long_route_static_20260723-194416` 的静态严格成功、物理无碰撞均为 `20/20 (100%)`，最大路径偏差为 `10.4614%`；它是旧的无外观变化静态证据，不能替代4×20结果。
- 旧全屋批次 `kujiale_long_route_20260722-171828` 使用 `mapping_start`、G1–G8 和旧障碍布局，只能作为历史证据，不能替代当前候选布局的结论。

## 文档分工

| 文档 | 状态 | 使用方式 |
| --- | --- | --- |
| [`../README.md`](../README.md) | 当前入口 | 快速启动和项目总览。 |
| [`user_manual.md`](user_manual.md) | 当前可执行手册 | GUI/RViz 可视化单轮、正式自动化批次、建图、Reset、Camera 和日常操作。 |
| [`interfaces.md`](interfaces.md) | 当前运行契约 | Topic、TF、模式配对、所有权与启动门禁。 |
| [`troubleshooting.md`](troubleshooting.md) | 当前排障手册 | 根据症状执行只读诊断和受管恢复。 |
| [`calibration.md`](calibration.md) | 当前流程 + 历史记录 | 新地图的标定流程；Warehouse v2 数字是历史记录。 |
| [`verification.md`](verification.md) | 当前证据台账 | 当前正式验收和历史能力证据的边界。 |
| [`kujiale_4x20_appearance_benchmark_plan.md`](kujiale_4x20_appearance_benchmark_plan.md) | 当前正式4×20运行手册与规格 | 四组20轮、外观Session Layer、三终端命令、证据、报告与验收门槛。 |
| [`kujiale_long_range_navigation_test_plan.md`](kujiale_long_range_navigation_test_plan.md) | 历史重设计规格与结果边界 | 保留 S/G1 闭环、静态候选批次和旧批次背景；正式执行改用4×20手册。 |
| [`kujiale_three_stage_dynamic_avoidance_plan.md`](kujiale_three_stage_dynamic_avoidance_plan.md) | 动态 actor 编排参考 | 三段 actor 坐标、gate、生命周期和可视化诊断入口；非4×20运行器。 |
| [`kujiale_navigation_dynamic_avoidance_issue_log.md`](kujiale_navigation_dynamic_avoidance_issue_log.md) | 当前问题复盘 | 静态/动态 profile 分离、STVL 清除、触发与 RViz 判读的已知边界。 |
| [`isaac_sim_nav2_rgbd_local_costmap_fusion_plan.md`](isaac_sim_nav2_rgbd_local_costmap_fusion_plan.md) | 历史设计 + 现行差异说明 | 理解 RGB-D 一期决策；不要照抄其中仅 Local Costmap 的旧设计。 |
| [`rviz_workflow_upgrade_plan.md`](rviz_workflow_upgrade_plan.md) | 历史设计记录 | 回溯 RViz/Lifecycle 升级取舍。 |
| [`runtime_reliability_and_performance_upgrade_plan.md`](runtime_reliability_and_performance_upgrade_plan.md) | 历史设计与证据快照 | 回溯可靠性和性能升级；部分地图数字已过时。 |
| [`skid_steer_navigation_solution.md`](skid_steer_navigation_solution.md) | 历史专项复盘 | 回顾滑移转向问题与修复。 |
| [`kujiale_usd_navigation_postmortem_20260717.md`](kujiale_usd_navigation_postmortem_20260717.md) | 历史复盘 | 回顾酷家乐初次适配的问题和处理。 |
| [`../plan.md`](../plan.md) | 原始总体设计 | 理解目标与技术取舍，不作为当前命令手册。 |
| [`repository_index.md`](repository_index.md) | 当前文件索引 | 查找文件职责和修改入口。 |

## 维护规则

改动启动参数、地图默认值、Topic/TF、传感器消费者或验收口径时，必须同一提交中
同步更新当前运行文档与本表；历史文档不需要重写，但必须在顶部标明其历史性质和
指向当前事实来源。
