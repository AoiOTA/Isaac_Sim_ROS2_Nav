# 文档状态与事实来源

> 最近复核：2026-07-26
>
> 适用分支：`main`

本仓库只保留当前可执行手册、运行契约、验证台账与恢复手册。执行命令或判断当前行为时，
始终以脚本、配置和测试所对应的当前文档为准。

## 当前事实来源（优先级从高到低）

1. 运行脚本与启动校验：`scripts/run_isaac.sh`、`scripts/run_ros.sh`；
2. 配置与实现：`isaac_sim/configs/`、`ros2_ws/src/robot_navigation/config/nav2_params.yaml`、Launch 文件；
3. 自动测试：对应包的 `test/`；
4. 当前运行文档：[`user_manual.md`](user_manual.md)、[`interfaces.md`](interfaces.md)、[`troubleshooting.md`](troubleshooting.md)；
5. 当前正式结果：[`verification.md`](verification.md) 与同一 campaign 的报告工件。

当前分支的关键运行事实如下：

- Localization/Navigation 默认地图是酷家乐 `warehouse_new`；
- `warehouse_new` 只批准普通 Ideal Localization/Navigation，Realistic 或显式 Pose Graph 定位会被启动脚本拒绝；
- 标准人工导航使用两个终端、受管 RViz 和 **2D Goal Pose**，不存在项目私有目标桥；
- Nav2 有两套明确 profile：默认 `stable` 完整复现静态 20 轮基线（Local + Global 标准 `VoxelLayer`）；`dynamic_avoidance` 使用 Local STVL 时序清除且 Global Costmap 不接收 RGB-D，避免移动 actor 留下全局残影；Collision Monitor 两套 profile 均只使用 `/scan`；
- 三阶段动态可视化的现行入口是 `run_kujiale_dynamic_isaac.sh`、`run_kujiale_three_stage_visual.sh` 与 `nav2_profile:=dynamic_avoidance`。G1→G2、G2→G3、G5→G1 分别使用独立 actor，成功到达下一航点后才退役；
- 当前正式操作入口是 `run_kujiale_4x20_all.sh`：一条命令自动管理静态/动态两套栈、四组各20轮、报告和`--resume`。静态40轮完成即保留 `static_2x20`，动态40轮完成即保留 `dynamic_2x20`，同一批次四组完成后才写根目录总4×20报告；`--dynamic-only` 可只重跑动态两组。正式批次 `20260725-210035` 已完成并通过：静态两组20/20，动态两组19/20，四组物理无碰撞均20/20。
- 单轮 GUI/RViz 诊断提供静态/动态自动完整 G2–G5–G1 路线；它们均不计入4×20证据。
- 正式结果只引用同一 campaign 的根报告；不同批次的静态或动态子报告不得拼接为新的 4×20 结论。

## 文档分工

| 文档 | 状态 | 使用方式 |
| --- | --- | --- |
| [`../README.md`](../README.md) | 当前入口 | 快速启动和项目总览。 |
| [`user_manual.md`](user_manual.md) | 当前可执行手册 | GUI/RViz 可视化单轮、正式自动化批次、建图、Reset、Camera 和日常操作。 |
| [`interfaces.md`](interfaces.md) | 当前运行契约 | Topic、TF、模式配对、所有权与启动门禁。 |
| [`troubleshooting.md`](troubleshooting.md) | 当前排障手册 | 根据症状执行只读诊断和受管恢复。 |
| [`calibration.md`](calibration.md) | 当前地图标定流程 | 新地图的标定、地图工件和启动配对流程。 |
| [`verification.md`](verification.md) | 当前证据台账 | 当前正式验收、配置边界、报告交付与复核方法。 |
| [`kujiale_4x20_appearance_benchmark_plan.md`](kujiale_4x20_appearance_benchmark_plan.md) | 当前正式4×20运行手册与规格 | 四组20轮、外观Session Layer、一键批量和三终端单轮可视化命令、证据、报告与验收门槛。 |
| [`kujiale_4x20_execution_lessons.md`](kujiale_4x20_execution_lessons.md) | 当前执行复盘与恢复手册 | supervisor、pilot、续跑/重跑、动态验收、报告和双远程推送问题。 |
| [`repository_index.md`](repository_index.md) | 当前文件索引 | 查找文件职责和修改入口。 |

## 维护规则

改动启动参数、地图默认值、Topic/TF、传感器消费者或验收口径时，必须同一提交中
同步更新当前运行文档与本表。被当前手册替代的方案、旧命令和旧结果不保留为可执行文档。
