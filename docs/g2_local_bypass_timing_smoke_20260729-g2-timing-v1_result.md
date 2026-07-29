# G2 local-bypass timing smoke：局部通过，路线仍不放行

## 判定

`20260729-g2-timing-v1` 验证了把 `local_bypass` gate 前移到 `y=-2.60`
这一单一修复：该 actor 的最小净距从前次 `0.0 m` 提升到 `0.8107 m`，并满足
本 smoke 的 `>=0.10 m` 目标。actor 以既有停车语义完成，未发生 safety-yield。

但同一全路线证据中 `g5_g1_crossing_actor` 的最小净距是 `0.0 m`，路线只能
得到 runner 的原生 `success` 和无物理接触信号，仍有低于 `0.10 m` 的安全警告。
因此状态是 **LOCAL_BYPASS_PASS__FULL_ROUTE_HOLD**：不得启动 G2 batch、
Confirmation 或 Module2--Module3 融合。

## 固定输入和证据

- Module3 实现提交：`f9c4c3ab23da1813f5926f457ed18ecb54e89072`
- scenario：`kujiale_g2_dynamic_safety_smoke`；seed `12001`；variant `v3`
- profile：`dynamic_avoidance`；appearance：`bright_warm`
- 动态配置 SHA-256：
  `6d37a7ae512add669ece18a577f13dbc84758d46288277ce9bbb565cd56f655f`
- manifest SHA-256：
  `0a7627b16938349db4e5016498b95326e79eaba5a96be87e402749eda6796d42`
- runner log SHA-256：
  `12e92848e3a93ea3587339cd04e16394eafe204006afbcf7f06751b7ce51d2ec`
- Nav2 log SHA-256：
  `4b5fc199cafea652401d833cd82c298d5d222d2525f65c07f0d66201d39ed045`

原始工件在
`data/experiment_runs/g2_dynamic_safety_smoke_20260729-g2-timing-v1`，不纳入
Git。

| actor | 最小净距 (m) | 结论 |
| --- | ---: | --- |
| `local_bypass_actor` | 0.8107 | 通过本次目标 |
| `g2_g3_exit_actor` | 0.6548 | 有余量 |
| `g5_g1_crossing_actor` | 0.0000 | 保持路线放行 |

`local_bypass_actor` 在 `18.7167 s` 开始，`20.2000 s` 已在原停车点完成；
机器人在该 actor 停车后通过，满足既有的 `passed_after_planned_park` 行为合同。

## 新发现的 G5 根因

G5 actor 在 `72.5500 s` 才开始移动，`73.3333 s` 于 `0.1394 m` 进入
`safety_yield`；该 guard 冻结的 actor 随后记录零净距，直到 `77.8167 s` 才恢复。
它的 gate 同时要求 `max_distance_to_obstacle_start_m: 1.05`，而 0.70 m、
0.32 m/s 的平滑停车轨迹约需 3.44 s，给当前机器人接近时间不足。

下一阶段只能单独冻结 G5 的更早 gate 修复并再运行一次单路线 smoke。不能把
local-bypass 的通过外推为全路线安全，也不能归因于 Module2 域适配。
