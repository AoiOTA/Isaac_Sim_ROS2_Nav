# G2 动态安全 smoke：三段路线通过

## 判定

`20260729-g2-timing-v2` 通过。物理碰撞是本阶段唯一安全失败判据：记录的
`/simulation/collision` 共 1089 条，其中 `true` 为 0；runner 结果为 `success`，
三名预期 actor 均触发、完成和退场，且没有 guard abort。

这是一次单路线开发 smoke，不是 G2 authorization 或 20 条 batch Gate。它仅
授权建立新的、独立的 G2 资格链 preregistration；不授权 Confirmation、Module2
快照、Goal-Prior Bridge、`BioNavGridBased` 或主动融合。

## 变更边界

动态障碍物的位置、尺寸、质量、运动轨迹、速度、加速度、variant 和停车语义
没有改变。仅调整两个既有 actor 的触发时机，使其按原轨迹在机器人进入近场前
完成可见横穿并停车：

- `local_bypass`：同一 northbound lane 的 gate `y=-1.75` → `-2.60`；
- `g5_g1_crossing`：gate `y=-2.50` → `-2.90`、x 下界 `-2.00` → `-2.10`、
  可见距离 `1.05 m` → `2.00 m`。

Module2、Nav2 profile、Costmap、MPPI、Collision Monitor、`/cmd_vel` 和验收
碰撞判据未被这些变更修改。

## 固定证据

- 运行实现提交：`4bfeae88011e2fcd5c4dd6151024ed2c4ff52a3c`
- scenario：`kujiale_g2_dynamic_safety_smoke`；seed `12001`；variant `v3`
- profile：`dynamic_avoidance`；appearance：`bright_warm`
- 物理步长：`1/60 s`；RTF：`1.0`
- 动态配置 SHA-256：
  `9c6f1e0092444fd3c39506d6a21cf29db08f8346e883a1f8eea1f43f5f7afd3d`
- manifest SHA-256：
  `81b671a7bcae93a1275859a2c98597824a30596ec0bac47fee08281e4ad48023`
- runner log SHA-256：
  `4d57adf96ef32f6ec21c9af844c151120dbdd5510639ada32bd896904459722a`
- Nav2 log SHA-256：
  `fe1e31ed9b98e8072c255ca0f636d8a0cf2d3c961395390d034febb9d11f974d`

原始运行工件保留在
`data/experiment_runs/g2_dynamic_safety_smoke_20260729-g2-timing-v2`，不纳入
Git。

## 结果

| 项目 | 结果 |
| --- | --- |
| 原生路线结果 | `success` |
| 物理碰撞 | `0` true messages / 1089 sampled messages |
| actor trigger / retire | `3 / 3`，集合完全匹配 |
| guard abort | `false` |
| safety yield | `false` |
| 最大 route recovery | `0` |
| 路线长度 | `38.9736 m` |

净距为可复放诊断而非额外失败阈值：`local_bypass=0.6880 m`、
`g2_g3_exit=0.5989 m`、`g5_g1_crossing=0.7657 m`。它们用于解释时序修复效果，
不改变“无物理碰撞”的验收规则。

## 后续边界

smoke 通过后，下一步是按新 Module3 SHA 建立 fresh G2 authorization-only
合同，然后才可能启动一次正式 G2 batch。为避免重复浪费，开发修复期间没有运行
任何 20 条 batch；正式 batch 仍只在授权链通过后执行一次。
