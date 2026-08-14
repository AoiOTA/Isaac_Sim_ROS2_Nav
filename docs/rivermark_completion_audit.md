# Attempt31 Rivermark 完成性审计

审计日期：2026-08-14。事实源为 Attempt31 当前代码、Rivermark 实际运行
evidence，以及 `qualification_v3` 派生报告；不以计划意图或单元测试替代真实
导航结果。

## 用户最终范围

- Rivermark 正式实验为 static、dynamic-v2、appearance 三组各 20 轮。
- 路径偏差只约束静态组；动态组不使用路径偏差门禁。
- Module2 四臂静态因果矩阵不在 Rivermark 运行，状态固定为
  `DEFERRED_TO_V4`。
- Rivermark 仍须证明 Module2 输出在真实室外链路中被有界消费，但该证据不作
  OFF/SR/DR/SRDR 因果归因。

## 指标与证据矩阵

| 要求 | 当前结果 | 权威证据 | 状态 |
| --- | --- | --- | --- |
| 静态无碰撞通行率 ≥95% | 20/20，100% | `qualification_v3` 的 `static`；20 个原始 run summary | PASS |
| 动态无碰撞通行率 ≥90% | 20/20，100% | `qualification_v3` 的 `dynamic`；contact、scan、GT 与 actor evidence | PASS |
| 静态同起止点路径偏差 ≤20% | 最大 3.256%，中位 1.416% | 冻结 0.05 m A* reference 和 20 轮 GT 轨迹 | PASS |
| 动态异构导航成功率 ≥90% | 18/20，90%；失败轮 10、20 原样计入 | dynamic-v2 20 轮；四 actor 合同 20/20，完整交互 18/20 | PASS |
| 光照、颜色变化稳定导航 | 20/20；四个 profile 各 5 轮 | appearance 20 轮，20/20 实际修改灯光和材质颜色 | PASS |
| 同等数值质量下收敛学习时间提升 ≥20% | 20 个 paired case，最小 99.473%，中位 99.526% | `convergence_paired.csv` 与 contract summary | PASS |
| A→B 连续地图更新效率提升 ≥30% | 20 次 paired update，最小 30.313%，中位 37.182% | `map_update_paired.csv`、region_22 A/B 数据与 parent-child smoke | PASS |
| 分区域避免重复计算 | 16×16 canvas；动态批次 cache 16 entries / 320 hits / 16 misses | `runtime_tile_cache_contract.json` | PASS |
| 进入新区域时切换且导航连续 | 每轮跨 13–15 个 region；五航点仍是一个连续任务 | dynamic 20 轮 `planning_prior_samples`、CanonicalRoute 和 GT | PASS |
| Module2 确实进入 Route Server 代价链 | 20/20 有健康 prior、正增量应用和选中路线 cost snapshot 变化 | `qualification_v3.module2_runtime_consumption` | PASS，非因果声明 |
| Module2 四臂因果矩阵 | 本次不运行 | `module2_causality.status=DEFERRED_TO_V4` | V4 范围 |

最终报告：

`data/experiment_runs/attempt31_rivermark/formal_20260814_v075_r30_cache/qualification_v3/attempt31_rivermark_qualification.json`

SHA-256：

`8abd0a9a9f98ba0f819035ec1dde91fbdf3c459d7490978b01bfebdd35b9af5a`

## Plan P0–P8 对照

| 阶段 | 结果与证据 | 状态 |
| --- | --- | --- |
| P0 active path 审计 | A21 默认入口保持不变；Module3 拥有地图、TF、GVG、Route、Nav2 与安全控制，Module2 只提供有界 prior | COMPLETE |
| P1 选区和 Collision | 选择 Candidate A 的 80 m × 80 m 区域；RGB、PhysX、height/curb 与 Occupancy 三联图已生成并视觉复核 | COMPLETE |
| P2 基础导航 | 1600×1600、0.05 m/格地图完成五航点真实导航，无穿墙规划 | COMPLETE |
| P3 认知区域 | 16×16 cell canvas、`T_map_canvas`、region ownership、tile cache 与切换已实现 | COMPLETE |
| P4 geometry-only 跨区 | static/off 20/20 连续五航点成功，不重启 Nav2、不重置 localization、不造假 edge | COMPLETE |
| P5 Module2 context | dynamic-v2 每轮跨区域且 applied prior 进入 Route Server；不可用时仍 geometry-only | COMPLETE（四臂因果由 V4） |
| P6 动态障碍 | 迎面、横穿、同向慢行、临时阻塞四阶段物理 actor；20 轮正式结果达标 | COMPLETE |
| P7 persistent remap | `RuntimeEdgeState` blockage 工程入口、parent-child 增量/tombstone 和 A→B 20 次效率基准已完成 | COMPLETE FOR STATED METRIC；非正式传感器因果证据 |
| P8 实验交付 | 3×20 evidence、CSV/JSON、checksum、地图/路线/actor 图、运行脚本和说明已完成 | COMPLETE |

原 plan 中的 OFF/SR/DR/SRDR 四臂实验被用户后续决定明确移至 V4，因此未用
Rivermark 的 `medium` 单臂数据冒充因果矩阵。视频不是当前定量验收指标，也未将
其作为 PASS 所需证据。

## 关键运行合同

- 全场只有一个连续 `map` 和一个全局物理 GVG；region 只切换认知上下文。
- `map→odom→base_link` 不随 region 切换重置；五航点任务不中断。
- Module2 只能增加有上界的 edge cost，不能创建 edge、覆盖 BLOCKED 或拥有最终
  Route。
- 动态临时障碍由传感器、Local Costmap、MPPI 与 Collision Monitor 处理；工程
  blockage 注入不计入正式避障成功率。
- 60 个正式轮次 checksum 全部复验通过；dynamic-v1 的 16/20 STOP 仍保留，
  dynamic-v2 没有替换失败轮。

## 回归结果

- 仓库级 pytest：798 passed，12 skipped；跳过项均因系统 Python 无 Isaac/USD
  `pxr`，实际 Isaac 行为由正式运行 evidence 覆盖。
- `robot_experiments` 安装态测试：310 passed。
- 当前 ROS 测试汇总：591 tests，0 errors，0 failures，1 skipped。
- `qualification_v3` JSON/CSV checksum：全部通过。
