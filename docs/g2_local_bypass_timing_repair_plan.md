# G2 local-bypass 时序修复：冻结方案

## 触发条件

此前的单路线 smoke `20260729-g2-safety-v1` 在 `local_bypass_actor`
处记录到 `0.0 m` 最小净距。其原始工件不修改、不复用为通过证据。

这不是 Module2 或视觉域适配问题：该 smoke 没有启动 Module2。也不是首先
调整 Nav2 cost、速度、平滑器或碰撞监控阈值的问题。

## 根因证据

在同一条可复放轨迹中：

| 时间 (s) | 事实 |
| ---: | --- |
| 24.8667 | `local_bypass_actor` 通过旧门 `y=-1.75` 开始移动 |
| 25.9833 | actor 在 `0.1001 m` 处进入 `safety_yield`；机器人最终 `/cmd_vel` 仍约 `0.913 m/s` |
| 26.0000 | Nav2 上游命令置零，但 60 Hz velocity smoother 正在按既有减速度收敛 |
| 26.1667 | actor 的离线净距记录为 `0.0 m` |
| 28.4500 | actor 才在较大分离后恢复移动 |

因此 actor 的“冻结但保持可碰撞”行为与机器人已经承诺的高速扫掠弧发生时序
冲突。直接加大硬 StopZone 会使机器人在 actor 旁停止，而 actor 又只会在机器
人已通过后恢复，构成潜在死锁，故不采用该办法。

## 唯一变更

把 `local_bypass` 的 northbound 空间 gate 从 `y=-1.75` 前移到 `y=-2.60`。

- actor 尺寸、质量、轨迹起终点、速度、加速度、variant、停车语义和 safety
  guard 均不变；
- Module3 的 Nav2、Costmap、MPPI、Collision Monitor 和 `/cmd_vel` 参数均不变；
- Module2 不接入，融合插件不实现或启用；
- 门仍限制相同 northbound 方向、速度和 x 车道。

旧 run 中门触发到近场 guard 只有约 1.12 s；前移 0.85 m 给现有 0.70 m
右移-停车轨迹提供额外可观测时间，使 actor 能在机器人进入近场前到达原先已
认证的停车点，而不是用 safety-yield 冻结在扫掠弧中。

## 验证序列

1. 解析配置与 gate 单元测试，确认仅此门坐标改变。
2. 执行同一单路线动态 smoke，一次、使用新 campaign ID；验收保持
   `local_bypass_actor >= 0.10 m`、全路线原生 `success`、三 actor 均触发/退场、
   无 guard abort。
3. smoke 失败则保存证据并停止；不试探第二个阈值，不启动 20 条 Gate。
4. smoke 通过后才建立独立的 fresh G2 授权和 batch preregistration；通过与否
   均不改变 Module2--Module3 融合的 Shadow-only 边界。
