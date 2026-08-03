# Module2 RViz 可视化

当前静态 Attempt-21 建议直接运行：

```bash
bash /home/lyb/Workspace/Bio_Nav/repos/Bio_Nav_Integration/scripts/run_attempt21_static_visual_experiment.sh combined
```

它用一个终端启动 Isaac Sim、Module2、Bridge、Nav2、RViz 与自动全屋路线。四种模式、
Isaac 复用规则和输出审计见
[用户手册 8.0 节](user_manual.md#80-module2-attempt-21-单终端入口)。

## 默认看到什么

`ros2_ws/src/robot_description/rviz/navigation.rviz` 已包含
**Module2 Cognitive Overlay** 分组。主显示为 MarkerArray：

| Namespace | 含义 | 运行时变化 |
|---|---|---|
| `Canvas` | 16×16、1 m/cell 认知画布边界 | 地图/reset 变化时更新 |
| `Motion Belief` | motion-only place belief 的 Top-32 稀疏热度；不是障碍物 | 随机器人运动更新 |
| `Motion Peak` | 当前最高概率认知格的扁平青色圆环；不是机器人 footprint | 随 belief 峰值移动 |
| `Local BEV Prediction` | Module2 在 `base_link` 下的局部风险预测，黄色到红色；尚不等于 Nav2 cost | 每次有效 RGB-D 推理更新 |
| `Dynamic Risk` | 旧 16×16 兼容风险表示 | 兼容路径收到风险时变化 |
| `Status` | health、age、reliability、fallback/identity 摘要 | 默认关闭，需要文字诊断时开启 |
| `Visual Candidate` | 未获 carry 资格的视觉候选 | 默认关闭，诊断用 |

主 topic 为：

```text
/bio_nav/module2/rviz_markers
```

该 MarkerArray 由 Integration bridge 生成。仅启动 Module3/Nav2 时分组存在但不会
变化，这表示没有收到 Module2 可视化消息，不表示 RViz 故障。

## 原始栅格

分组内还有两个默认关闭的 Map 显示：

- `Module2 Dynamic Risk Raw`：
  `/bio_nav/module2/dynamic_cost_grid`
- `Module2 Motion Belief Raw`：
  `/bio_nav/module2/place_belief_grid`

启用后必须将 **Color Scheme** 设为 `costmap`。`map` 配色会把连续低数值显示成
近似白色薄层，肉眼很难判断变化。建议 Alpha 为 `0.20–0.35`，并保持
`Draw Behind=false`。

## 操作步骤

1. 使用项目保存的 `navigation.rviz` 启动 RViz。
2. 展开 `Module2 Cognitive Overlay`，确认 `Module2 Live Overlay` 为 Enabled。
3. 展开 MarkerArray 的 Namespaces；日常观察优先保留 `Motion Belief`、
   `Motion Peak` 和 `Local BEV Prediction`。`Status` 与 `Local BEV Label`
   默认关闭，避免大段文字遮挡地图。
4. 若需要看完整热图，再开启两个 Raw Map，并检查 `Color Scheme=costmap`。
5. 发送 2D Goal Pose 后观察：
   - Motion Peak 是否随机器人移动；
   - 黄色/红色 Local BEV Prediction 是否覆盖 Module2 认为有风险的区域；
   - 紫色 `Projected Global Risk` 是否只在风险被 Module3 门控接受后出现；
   - 如需文字状态，再临时开启 Status，确认不是 stale/hash mismatch/OOD。

## 与传统 RGB-D 体素层一起看

`RGB-D Fusion/Marked Voxels (3D)` 的深绿色 5 cm 方块来自 Nav2
`depth_voxel_layer`，用于检测建图后加入的六个低矮静态障碍。它属于传统
Module3 安全链，不是 Module2 输出，也不会因为 Module2 fail-closed 而消失。

颜色与控制关系如下：

| 画面元素 | 来源 | 是否实际影响 Nav2 |
|---|---|---|
| 深绿色小体素 | RGB-D → Nav2 VoxelLayer | 是，形成传统障碍 cost |
| 黄色/红色局部格 | Module2 `LocalRiskGrid` 预测 | 否，仅表示预测 |
| 紫色全局格 | Module3 接受并投影后的风险层 | 是，但仅为最高 80 的非 lethal 软代价 |
| 青色稀疏格与圆环 | Module1 motion belief / peak | 否，只作定位诊断 |

因此“看见红格”不能证明 Nav2 已使用 Module2；必须同时看到紫色投影，或检查
`/bio_nav/local_risk_layer/status`。反过来，静态障碍即使没有黄色/红色格，只要
绿色体素和 Costmap 已标记，传统 Nav2 仍可绕行。

当前 v13 任务验收以五段路线、timeout 和独立记录的 Isaac ContactSensor 为控制
证据；footprint-vs-box SAT overlap 继续显示和入档，但只作几何诊断。RViz 中视觉上
贴边并不自动等于 ContactSensor 碰撞，报告必须分别列出两者。

v15 静态补充实验中还需打开 `/bio_nav/planner/rviz_markers`。青色 planning marker
只在 `PlannerDecision.cognitive_tiebreak_used=true` 时表示 Module2 planning prior 被采用；
橙色为 stock fallback。combined 必须同时核对青色 planning adoption、紫色 applied
risk 和绿色 RGB-D voxel，三者分别来自不同通道，不能用其中一种颜色替代另两种证据。

v15 最终实测中，planning-only 的 adopted coverage 为 1189/1205（98.67%），combined
为 1175/1209（97.19%），combined risk valid coverage 均值为 97.55%；两组均完成
10/10 路线且 ContactSensor 0。RViz 可以直观看到上述通道，但精确比例仍以冻结的
`PlannerDecision`、risk status 和聚合 receipt 为准。

## 命令行核对

```bash
ros2 topic hz /bio_nav/module2/rviz_markers
ros2 topic echo /bio_nav/module2/rviz_markers --once
ros2 topic hz /bio_nav/module2/dynamic_cost_grid
ros2 topic hz /bio_nav/module2/place_belief_grid
```

Marker topic 没有发布时，先检查 Integration bridge 和 Module2 服务。Raw grid
有数据但 Marker 不变时，检查 bridge 的 health/identity gate。Status 显示拒绝
时不要为了视觉效果关闭安全门控。

RViz 颜色只是诊断，不代表风险已真正进入 Costmap。是否应用以
`RiskLayerStatus`、Global Costmap 和 planner decision 记录为准。
