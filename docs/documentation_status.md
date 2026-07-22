# 文档状态与事实来源

> 最近复核：2026-07-22
>
> 适用分支：`codex/kujiale-long-range-navigation-test`

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
- `rgbd_navigation` 发布 `/camera/front/depth/points`，全局和局部 Costmap 均使用独立 `depth_voxel_layer`；Collision Monitor 仍只使用 `/scan`；
- 当前正式全屋批次结果为静态 `20/20`、动态 `19/20`，物理无碰撞分别为 `20/20` 和 `20/20`。生成的完整报告是本地数据工件，不纳入 Git。

## 文档分工

| 文档 | 状态 | 使用方式 |
| --- | --- | --- |
| [`../README.md`](../README.md) | 当前入口 | 快速启动和项目总览。 |
| [`user_manual.md`](user_manual.md) | 当前可执行手册 | 手动测试、建图、Reset、Camera 和日常操作。 |
| [`interfaces.md`](interfaces.md) | 当前运行契约 | Topic、TF、模式配对、所有权与启动门禁。 |
| [`troubleshooting.md`](troubleshooting.md) | 当前排障手册 | 根据症状执行只读诊断和受管恢复。 |
| [`calibration.md`](calibration.md) | 当前流程 + 历史记录 | 新地图的标定流程；Warehouse v2 数字是历史记录。 |
| [`verification.md`](verification.md) | 当前证据台账 | 当前正式验收和历史能力证据的边界。 |
| [`kujiale_long_range_navigation_test_plan.md`](kujiale_long_range_navigation_test_plan.md) | 当前正式测试规格与结果 | 全屋路线、报告口径、20+20 结果和生成工件。 |
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
