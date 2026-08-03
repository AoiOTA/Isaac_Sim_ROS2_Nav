# Kujiale 4×20 验证台账

> 最近复核：2026-07-26<br>
> 适用分支：`main`<br>
> 当前正式 campaign：`20260725-210035`

本文只保留当前 Kujiale 4×20 光照/颜色鲁棒性实验的可引用结论。运行命令、环境约束和报告结构见
[`kujiale_4x20_appearance_benchmark_plan.md`](kujiale_4x20_appearance_benchmark_plan.md)；指标的通用定义和
LaTeX 公式见 [`kujiale_4x20_metric_definitions.md`](kujiale_4x20_metric_definitions.md)。历史地图、旧路线、
旧脚本和旧批次不构成本台账的一部分，也不能与下列结果混用。

## 1. 正式结果

正式报告的机器可读源为：

```text
data/reports/kujiale_4x20_20260725-210035/benchmark.json
```

该报告记录 `complete=true`、`passed=true`、`issues=[]`。四组均在同一 `warehouse_new` 地图、
Ideal 定位、`long_route_start_g1` 和闭环路线 `G1 → G2 → G3 → G4 → G5 → G1` 下完成：

| 条件 | 场景变量 | 外观变量 | 严格成功 | 物理无碰撞 | 结果 |
| --- | --- | --- | ---: | ---: | --- |
| 静态基准 | 六个低矮 RGB-D 静态障碍 | `baseline` | `20/20 (100%)` | `20/20 (100%)` | 通过 |
| 静态＋外观 | 相同静态几何 | 四种 profile 各 5 轮 | `20/20 (100%)` | `20/20 (100%)` | 通过 |
| 动态基准 | 三阶段 actor | `baseline` | `19/20 (95%)` | `20/20 (100%)` | 通过 |
| 动态＋外观 | 相同 actor 运动学 | 四种 profile 各 5 轮 | `19/20 (95%)` | `20/20 (100%)` | 通过 |

静态两组的最大相对理论最短可行路径偏差分别为 `10.1687%` 与 `10.1442%`，均低于 `20%`。
动态两组的严格门槛为 `18/20`，静态两组的严格门槛为 `19/20`。四组物理无碰撞均为 `20/20`。

上述“物理无碰撞”沿用正式 campaign 的任务状态与 Isaac ContactSensor 口径。后续
SAT 只读复核发现历史静态 40 条中有 4 条出现 `0.45–5.22 mm` 的瞬时边界重叠，均未
触发传感器、卡死或任务失败。该诊断不回写历史 receipt；对外应把 `40/40` 表述为
当前冻结场景中的任务级成功，不能表述为任意几何计算下绝对零接触。

动态基准 seed `7320` 与动态外观 seed `7306` 因
`three_stage_dynamic_behavior_not_observed` 计为严格失败；二者均未发生物理碰撞。低于 `0.10 m` 的
保守净距或 `safety_yield` 会作为风险警告保留在报告中，但不是单独的失败判据。

## 2. 实验配置边界

- 静态组使用 `stable` Nav2 profile、六个固定低矮障碍和 `baseline` 或固定外观 profile。
- 动态组使用 `dynamic_avoidance` Nav2 profile 与 `full_route_three_stage`：
  `local_bypass`、`g2_g3_exit`、`g5_g1_crossing` 按 gate 依序 `waiting → armed → moving → parked → retired`。
- 外观使用匿名 USD Session Layer；只改变灯光强度、色温和材质色相，不改写原始 USD、几何、碰撞、地图或 actor 运动学。
- 外观 profile 为 `baseline`、`dim_warm`、`dim_cool`、`bright_warm`、`bright_cool`；参数和客厅示意图已嵌入报告。
- 导航消费 `/scan` 与 `/camera/front/depth/points`；实验验证环境外观变化下的稳定执行，不宣称导航依赖 RGB 颜色。

任何地图、场景、路线、actor、Nav2 profile、外观矩阵、验收规则或实现代码的变更，都会使本节结果失去可比性，
必须使用新的 campaign ID 重新运行对应实验，不能与本批次拼接。

## 3. 结果与报告交付

每个 campaign 的报告目录为：

```text
data/reports/kujiale_4x20_<CAMPAIGN_ID>/
```

其中 `index.html` 与 `index_portable.html` 在图片快照已发布时均使用 GitHub Raw 外链；可单独复制到其他电脑，
在能访问 GitHub 时预览、筛选并打开原图，无需携带本地 PNG。未发布快照的报告保留相对 PNG 路径；`report.pdf`、`report.md`、`benchmark.json/csv`、`evidence_index.json` 和
`data_dictionary.md` 提供可审阅的固定交付物。报告能按实验组、seed、外观 profile、动态变体和结果筛选逐轮路径：
静态图叠加六个静态障碍，动态图叠加本轮实际触发 actor 的轨迹。

## 4. 复核方法

报告与证据输出不提交 Git。若只更新报告器而不改变原始证据，可重绘既有报告：

```bash
source /home/lyb/Workspace/Bio_Nav/workspace.env
cd "${BIO_NAV_MODULE3_ROOT}"
./scripts/run_kujiale_4x20.sh report 20260725-210035 --replace
```

对调度、外观或报告代码的修改，至少执行：

```bash
source ./scripts/setup_ros_env.sh
/usr/bin/pytest -q \
  ros2_ws/src/robot_experiments/test/test_kujiale_4x20_campaign.py \
  isaac_sim/tests/test_appearance.py \
  isaac_sim/tests/test_appearance_preview.py
bash -n scripts/run_kujiale_4x20.sh scripts/run_kujiale_4x20_isaac.sh \
  scripts/run_kujiale_4x20_all.sh
```

运行新的正式 campaign 使用：

```bash
./scripts/run_kujiale_4x20_all.sh
```

该命令自动生成新的 ID、顺序运行静态与动态阶段并最终写入完整报告。动态专用复测使用
`./scripts/run_kujiale_4x20_all.sh --dynamic-only`；它只生成独立动态 `2×20` 结果，不能与另一个 campaign 的
静态结果拼接为完整 4×20 结论。
