# G5 crossing 时序修复：冻结方案

## 前置证据

`20260729-g2-timing-v1` 已使 G1→G2 的 local-bypass actor 达到 `0.8107 m`
净距，但 G5→G1 的 crossing actor 仍为 `0.0 m`。这次分析不修改已有证据，
也不把 runner 的无接触 `success` 误报为可放行的安全余量。

G5 现有 actor 的轨迹长度为 `0.70 m`、速度上限 `0.32 m/s`、加速度上限
`1.60 m/s²`。cosine rest-to-rest profile 的最小持续时间为：

```text
max(pi * 0.70 / (2 * 0.32), sqrt(pi² * 0.70 / (2 * 1.60))) = 3.436 s
```

然而旧 gate 只在机器人距 actor 起点 `<=1.05 m` 时允许触发；记录显示 actor
`72.5500 s` 启动、`73.3333 s` 已因 `0.1394 m` guard 让行。故根因是可观测和
轨迹完成时间不足，而不是 Module2 域适配、Nav2 全局规划或碰撞判据。

## 唯一变更

只修改 `g5_g1_crossing` 的 gate：

- `y` threshold：`-2.50` → `-2.90`；
- x 车道下界：`-2.00` → `-2.10`，覆盖该路线在新阈值处的已测 x≈`-2.03`；
- `max_distance_to_obstacle_start_m`：`1.05` → `2.00`。

actor 的尺寸、质量、起终点、速度、加速度、variant、停车语义和 safety guard
全部保持。其余两个 actor、Nav2、Costmap、MPPI、Collision Monitor、Module2
和全部验收阈值保持不变。

新 gate 的实测路线位置约在 `69.6 s`，而旧近场冲突约在 `73.3 s`，为既有
3.436 s 轨迹保留约 3.7 s 的完成窗口。它使 actor 仍以可见、可复现的横穿和
停车方式出现，而不是在接触范围临时生成或使用 guard 冻结。

## 验证

1. 配置/gate/静态扫掠测试确认唯一改动。
2. 一次新的完整三段动态 smoke，要求原生结果 `success`（物理碰撞为零）、
   完整 trigger/retire、无 guard abort；每 actor 最小净距仅作为诊断记录。
3. 失败则保留 evidence 并停止；不执行第二次阈值尝试或 20 条 Gate。
4. 只有该 smoke 通过，才可回到新的 G2 authorization 和 batch preregistration。
