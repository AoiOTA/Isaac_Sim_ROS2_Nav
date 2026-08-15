# 文档状态与事实来源

> 最近复核：2026-08-15
>
> 适用分支：`codex/final-outdoor-navigation`

本仓库只保留当前可执行手册、运行契约、验证台账与恢复手册。执行命令或判断当前行为时，
始终以脚本、配置和测试所对应的当前文档为准。

## 当前事实来源（优先级从高到低）

1. 运行脚本与启动校验：`scripts/run_isaac.sh`、`scripts/run_ros.sh`；
2. 配置与实现：`isaac_sim/configs/`、`ros2_ws/src/robot_navigation/config/nav2_params.yaml`、Launch 文件；
3. Final 权威来源：[`rivermark_outdoor_demo.md`](rivermark_outdoor_demo.md)、[`rivermark_completion_audit.md`](rivermark_completion_audit.md)、`data/rivermark_demo/final_rivermark_metric_contract.yaml`、[`../README.md`](../README.md) 的 Final 状态表，以及配对 Integration 工作树 `/home/lyb/Workspace/Bio_Nav/worktrees/integration/final-indoor-outdoor-navigation/docs/final_closure/`（分支 `codex/final-indoor-outdoor-navigation`）的六文档：`results_indoor_60.md`、`results_outdoor_60.md`、`results_v4_100.md`、`module2_effectiveness_evidence.md`、`results_master_summary.md`、`final_navigation_architecture.md`；
4. 自动测试：对应包的 `test/`；
5. 当前运行文档：[`user_manual.md`](user_manual.md)、[`interfaces.md`](interfaces.md)、[`troubleshooting.md`](troubleshooting.md)；
6. 当前正式结果：[`verification.md`](verification.md) 与同一 campaign 的报告工件。

当前分支的关键运行事实如下。

**Final/Rivermark 主线（当前正式口径）**：

- 当前主线为 Final Rivermark 室外五航点收口；
- Final clean-revision Static/Dynamic/Appearance 各 20 轮均 20/20 成功且无碰撞，分类 `FORMAL_QUALIFICATION_PASS`；
- V4 Q36_04、Q14_45、Q36_51 共 60/60 完成且无碰撞，为场景/arm 依赖工程证据（`ENGINEERING_EVIDENCE_NOT_QUALIFICATION`），不支持 Module2 普遍提升；
- 历史 Attempt31 为 `REPORT_GATES_PASS_WITH_PROVENANCE_CAVEAT`（Static 20/20、Dynamic-v2 18/20、Appearance 20/20、60/60 无碰撞、静态偏差 max 3.256%）；
- Module2 消费 20/20、累计 515.594215 m（非因果，只证明 Module2 输出被 Route Server 有界消费）；
- 八条单终端操作入口（室内、室外各四条）在 Integration final 分支 `codex/final-indoor-outdoor-navigation` 的 `docs/final_closure/` 两份单终端 runbook；
- ROS domain 231 用于 GUI 单轮/演示，232 用于 campaign 批次。

以下为历史兼容入口（Kujiale/4×20）：

- Localization/Navigation 默认地图是酷家乐 `warehouse_new`；
- `warehouse_new` 只批准普通 Ideal Localization/Navigation，Realistic 或显式 Pose Graph 定位会被启动脚本拒绝；
- 标准人工导航使用两个终端、受管 RViz 和 **2D Goal Pose**，不存在项目私有目标桥；
- Nav2 有两套明确 profile：默认 `stable` 使用标准 `VoxelLayer`，`dynamic_avoidance` 使用 Local STVL 时序清除且 Global Costmap 不接收 RGB-D；两套 profile 的 Local Costmap 与 Collision Monitor 使用自滤波后的 `/scan_safety`，SLAM、定位和 Global Costmap 仍使用原 `/scan`；
- 三阶段动态可视化的现行入口是 `run_kujiale_dynamic_isaac.sh`、`run_kujiale_three_stage_visual.sh` 与 `nav2_profile:=dynamic_avoidance`。G1→G2、G2→G3、G5→G1 分别使用独立 actor，成功到达下一航点后才退役；
- 当前正式操作入口是 `run_kujiale_4x20_all.sh`：一条命令自动管理静态/动态两套栈、四组各20轮、报告和`--resume`。静态40轮完成即保留 `static_2x20`，动态40轮完成即保留 `dynamic_2x20`，同一批次四组完成后才写根目录总4×20报告；`--dynamic-only` 可只重跑动态两组。正式批次 `20260725-210035` 已完成并通过：静态两组20/20，动态两组19/20，四组物理无碰撞均20/20。
- 单轮 GUI/RViz 诊断提供静态/动态自动完整 G2–G5–G1 路线；它们均不计入4×20证据。
- 正式结果只引用同一 campaign 的根报告；不同批次的静态或动态子报告不得拼接为新的 4×20 结论。

## 文档分工

| 文档 | 状态 | 使用方式 |
| --- | --- | --- |
| [`../README.md`](../README.md) | 当前入口 | 快速启动和项目总览。 |
| [`rivermark_outdoor_demo.md`](rivermark_outdoor_demo.md) | 当前室外运行手册 | Final 收口命令与 Attempt31 历史基线。 |
| [`rivermark_completion_audit.md`](rivermark_completion_audit.md) | 当前室外指标与证据矩阵 | Rivermark 验收指标、证据出处与 Final/Attempt31 分类口径。 |
| [`user_manual.md`](user_manual.md) | 当前可执行手册（历史兼容，Kujiale 室内） | GUI/RViz 可视化单轮、正式自动化批次、建图、Reset、Camera 和日常操作。 |
| [`interfaces.md`](interfaces.md) | 当前运行契约 | Topic、TF、模式配对、所有权与启动门禁。 |
| [`troubleshooting.md`](troubleshooting.md) | 当前排障手册 | 根据症状执行只读诊断和受管恢复。 |
| [`calibration.md`](calibration.md) | 当前地图标定流程 | 新地图的标定、地图工件和启动配对流程。 |
| [`verification.md`](verification.md) | 历史验证台账（Kujiale/Attempt21~31）；当前室外台账见 [`rivermark_completion_audit.md`](rivermark_completion_audit.md) | 历史正式验收、配置边界、报告交付与复核方法。 |
| [`kujiale_4x20_appearance_benchmark_plan.md`](kujiale_4x20_appearance_benchmark_plan.md) | 当前正式4×20运行手册与规格（历史兼容） | 四组20轮、外观Session Layer、一键批量和三终端单轮可视化命令、证据、报告与验收门槛。 |
| [`kujiale_4x20_execution_lessons.md`](kujiale_4x20_execution_lessons.md) | 当前执行复盘与恢复手册（历史兼容） | supervisor、pilot、续跑/重跑、动态验收、报告和双远程推送问题。 |
| [`repository_index.md`](repository_index.md) | 当前文件索引 | 查找文件职责和修改入口。 |

## 维护规则

改动启动参数、地图默认值、Topic/TF、传感器消费者或验收口径时，必须同一提交中
同步更新当前运行文档与本表。被当前手册替代的方案、旧命令和旧结果不保留为可执行文档。
