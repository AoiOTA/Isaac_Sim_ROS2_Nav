# Module2 RViz 可视化

## 默认看到什么

`ros2_ws/src/robot_description/rviz/navigation.rviz` 已包含
**Module2 Cognitive Overlay** 分组。主显示为 MarkerArray：

| Namespace | 含义 | 运行时变化 |
|---|---|---|
| `Canvas` | 16×16、1 m/cell 认知画布边界 | 地图/reset 变化时更新 |
| `Motion Belief` | motion-only place belief 热度 | 随机器人运动更新 |
| `Motion Peak` | 当前最高概率认知格 | 随 belief 峰值移动 |
| `Dynamic Risk` | 通过健康门控的动态风险 | 看到/清除动态交互时变化 |
| `Status` | health、age、reliability、fallback/identity 摘要 | 每次消息更新 |
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
3. 展开 MarkerArray 的 Namespaces；优先保留 `Status`、`Motion Peak` 和
   `Dynamic Risk`。
4. 若需要看完整热图，再开启两个 Raw Map，并检查 `Color Scheme=costmap`。
5. 发送 2D Goal Pose 后观察：
   - Motion Peak 是否随机器人移动；
   - Dynamic Risk 是否只在动态交互窗口出现并清除；
   - Status 是否显示 healthy/accepted，而非 stale/hash mismatch/OOD。

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
