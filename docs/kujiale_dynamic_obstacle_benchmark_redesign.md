# Kujiale 动态避障实验重设计方案

## 0. 文档状态

- 开发分支：`codex/dynamic-obstacle-benchmark-redesign`
- 当前阶段：实验装置实现中；正式验收尚未执行
- 当前实现状态：已实现物理动态 actor、单轮可视化、运行矩阵与报告入口；仍需由用户执行最终 `20 + 5` 验收
- 最终验收执行人：用户
- Codex 后续职责：实现实验装置、单轮可视化、自动化测试、最终验收脚本和报告生成入口，但不代替用户执行正式 `20 + 5` 验收，也不宣称正式验收通过

## 1. 重设计原因

现有 Kujiale 动态场景不能有效区分真正的动态避障能力：

1. 两个障碍物速度均为 `0.10 m/s`，横穿结束后永久 `hold`，很快退化为新增静态障碍物。
2. 触发时机只依赖 G2 目标被 Nav2 接受后的固定延迟，机器人实际速度、规划耗时或前序停顿变化后，障碍物可能提前通过或直接撞向机器人。
3. 障碍物是 kinematic rigid body。它与机器人接触时相当于具有不受质量约束的执行器，会把机器人推走，碰撞后的轨迹已不能表示 Nav2 的控制结果。
4. 现有两个障碍都位于 G1→G2 航段，且均为横穿后停车，不能分别验证横穿让行、迎面会车、同向慢行和短时封堵后的恢复能力。
5. 当前结果主要统计整圈导航成功和物理碰撞，没有证明障碍物与机器人确实在运动中形成过有效相遇。障碍物没有赶上机器人时也可能被误判为动态避障成功。
6. 物理配置与 ROS 实验配置重复描述轨迹，容易发生终止状态与验收代码契约漂移。

本方案把“障碍物速度”“相遇时机”“碰撞处理”和“交互有效性”拆开设计，优先保证实验本身可信，再讨论 MPPI 参数是否需要调整。

## 2. 实验目标与边界

### 2.1 目标

- 验证机器人面对正在运动的障碍物时能否及时减速、停车、绕行或让行。
- 验证障碍物清场后机器人能否恢复前进，不出现 Collision Monitor 长时间锁死或 Nav2 永久停滞。
- 分别测量四种典型动态交互，避免只通过一种横穿场景就得出“动态避障性能良好”的结论。
- 保留每轮时空轨迹、最小净距、触发时刻、感知时刻、响应时刻和恢复时刻，使失败能够复盘。
- 提供单轮实时可视化入口，供用户反复观察完整行为，不需要启动正式批量验收。

### 2.2 本阶段不做

- 不在实验装置实现阶段修改 `nav2_params.yaml`、MPPI critic、速度平滑器或 Collision Monitor 参数。
- 不使用低矮 RGB-D 障碍物作为动态实验主体。动态物体统一高于 LiDAR 扫描平面，以隔离验证动态决策能力。
- 不把 Ground Truth 位姿或动态障碍真值输入 Nav2、Local Costmap、Global Costmap 或 Collision Monitor。
- 不研究碰撞后的物理冲击、推挤或机器人抗撞能力。动态障碍一旦进入接触保护边界，该轮直接判为安全失败。
- 不由 Codex 执行最终正式验收批次。

## 3. 固定实验环境

| 项目 | 固定值 |
|---|---|
| Isaac Sim | 6.0.1 |
| ROS 2 | Jazzy |
| 场景 | `kujiale_0026_A_to_B_door_open.usd` |
| 地图 | `data/maps/occupancy/warehouse_new.yaml` |
| 出生点 | `long_route_start_g1` |
| 定位/里程计 | Ideal；Isaac 独占 `/odom` 和 `odom -> base_link` |
| 相机 profile | `rgbd_navigation`，但高动态障碍的主感知来源为 `/scan` |
| 物理步长 | `1/60 s` |
| 目标 RTF | `1.0` |
| 机器人 footprint | `[[0.255,0.210],[0.255,-0.210],[-0.230,-0.210],[-0.230,0.210]]` |
| MPPI | 保持当前配置，2.0 秒预测范围 |
| 动态障碍尺寸 | 标准 case 为 `[0.40, 0.40, 1.00] m`；同向慢车为 `[0.45, 0.45, 1.00] m`，中心高度 `z=0.50 m` |

受控实验与全屋回归均沿用当前 Kujiale 地图和出生点，不引入另一套标准仓库结果。

## 4. 动态障碍公共运动契约

### 4.1 状态机

每个动态障碍必须按以下状态机运行：

```text
waiting -> armed -> moving -> [dwell] -> clearing -> parked
                                  |
                                  +-> safety_yield -> moving
```

- `waiting`：本轮未选择该 case，碰撞和可见性关闭。
- `armed`：对应 Nav2 航段已接受；schema v3 actor 保持隐藏，直到机器人通过空间门，避免提前作为静态全局障碍。
- `moving`：机器人穿过空间触发门后，障碍按照受限速度曲线运动。
- `dwell`：仅 `temporary_block` 允许，最大 1.8 秒。
- `clearing`：障碍已经越过冲突点，继续离开机器人通道。
- `parked`：运动完成后停放在预检通过的安全终点，保持可见和物理碰撞；它不消失，也不再主动移动。
- `retired`：仅兼容旧配置的清理状态；本基准的 schema v3 actor 一律使用 `parked`。
- `safety_yield`：预测净距进入 `0.14 m` 保护带时，actor 原地让行、保持可见和碰撞；机器人离开 `0.34 m` 后才恢复运动。该状态会令本轮实验失败，但不会删除 actor 或取消导航。
- `guard_aborted`：进入接触保护边界，本轮立即失败。

### 4.2 速度与加速度

- 四类障碍的巡航速度限定在 `0.25–0.50 m/s`。
- 最大加速度和最大减速度均为 `0.75 m/s²`。
- 使用梯形速度曲线；短轨迹不足以达到巡航速度时自动使用三角速度曲线。
- 禁止通过每帧固定位置增量造成启动瞬间速度跳变。
- 轨迹的预计到达时间由实际速度曲线积分计算，不再简单使用 `distance / speed` 估算触发时刻。

### 4.3 空间触发

- G2 目标被接受只负责 `arm`，不直接开始运动。
- 障碍开始运动必须同时满足：
  - 机器人进入该 case 的 map 坐标触发门；
  - 机器人运动方向与预期方向一致；
  - 机器人线速度不低于 `0.20 m/s`；
  - 当前 case 和 variant 与本轮运行矩阵一致。
- 触发门使用 Ground Truth 仅控制实验环境时序，不向导航栈发布任何真值。
- 每个障碍每轮最多触发一次，机器人返程经过同一区域时不得重复触发。

### 4.4 防推车保护

每个物理帧计算机器人 footprint 的保守外接圆与障碍 box 之间的二维净距：

- 净距大于 `0.14 m`：障碍按规划继续运动。
- 净距小于等于 `0.14 m`：
  - 立即停止该 actor 的运动，但保持可见和碰撞；
  - 发布 `safety_yield`，让机器人基于正常 LiDAR/Costmap 绕开；
- 净距重新大于等于 `0.34 m` 后才恢复 actor 的轨迹时钟；
  - 本轮记为实验安全失败（`dynamic_actor_safety_yield`），但不取消目标，保留完整复盘证据。

`0.14 m` 保护带仍覆盖 60 Hz 下的单帧相对位移和 footprint 建模余量，同时避免保守外接圆在尚未接近时错误冻结横穿 actor。物理 Contact Sensor 继续独立工作，用于发现保护逻辑之外的场景碰撞。

### 4.5 地图几何预检

正式运行前自动检查：

- 障碍整个 swept footprint 不与 `warehouse_new` 的 occupied/unknown 栅格相交；
- 轨迹与墙体之间至少保留 `0.05 m` 数值余量；
- 障碍初始 footprint 与机器人参考路线的净距至少为 `0.50 m`；
- 冲突点确实位于 G1→G2 无障碍参考路径附近；
- 障碍终点必须完全离开机器人通道，才能允许停放（`parked`）。

任何预检失败都必须阻止实验启动，不能依赖 Isaac 运行后“看起来差不多”。

## 5. 第一层：20 次受控单交互实验

### 5.1 统一路线

- 出生点：`S/G1 = [0.45, -5.35, 90°]`
- 目标：`G2 = [0.80, 4.80, -160°]`
- 每轮只执行 G1→G2，不继续全屋路线。
- 每轮只启用一个动态 case。
- 主要冲突区位于中央大厅；离线地图检查显示无障碍路径在 `y≈0` 时经过 `x≈-0.10`。

### 5.2 四类 case

以下坐标均为 `map` 坐标，三维位置的 `z` 固定为 `0.50 m`。

#### A. 垂直横穿 `crossing`

- 轨迹：`[0.70, 0.00] -> [-0.85, 0.00]`
- 巡航速度：`0.45 m/s`
- 机器人触发门：北向穿过 `y=-2.80`，允许的 `x` 范围为 `[-0.60, 0.40]`
- 冲突点：`[-0.10, 0.00]`
- 五档启动延迟：穿过空间门后 `0.00 / 0.15 / 0.30 / 0.45 / 0.60 s`
- 种子：`7401–7405`
- 期望行为：机器人减速让行或在大厅内绕过障碍，障碍清场后继续前往 G2。

#### B. 迎面会车 `oncoming`

- 轨迹：`[0.70, 0.80] -> [-0.10, 0.50] -> [-0.10, -1.20]`
- 巡航速度：`0.40 m/s`
- 机器人触发门：北向穿过 `y=-1.80`
- 五档启动延迟：`0.00 / 0.15 / 0.30 / 0.45 / 0.60 s`
- 种子：`7411–7415`
- 期望行为：机器人横向让出会车空间，或者停车让障碍先通过；禁止正面接触。

#### C. 动态封堵改道 `same_direction_slow`

- 轨迹：`[-0.35, -0.10] -> [-0.35, 0.65] -> [0.45, 0.65] -> [0.45, -0.55]`。它先占用实测主路线，随后横向驶入右侧安全区；不进入 `y≥1.0` 的窄通道。
- 尺寸：`0.45 × 0.45 × 1.00 m`；巡航速度：`0.18 m/s`，完整运动约 15 秒。
- 机器人触发门：北向穿过 `y=-3.60`。actor 初始位于门前约 3.5 m，使其先进入 Local Costmap，再与机器人形成同向相对速度。
- actor 在空间门前保持隐藏；到门后出现并在前方同向慢行，不会在目标刚接受时被全局规划器当成静态障碍。
- 种子：`7421–7425`
- 期望行为：actor 预先封堵左侧通路时，机器人可选择右侧狭窄通路；这验证动态障碍触发的全局重规划，不计为局部绕行能力。

#### 调优用局部绕障 `local_bypass`（暂不计入正式 20 次）

- 触发：机器人北向通过门口 `y=-2.80` 后 actor 开始横移；它采用速度不超过 `0.40 m/s`、加速度不超过 `0.50 m/s²` 的余弦缓入缓出曲线，约 `3.14 s` 后到达预先验证的绕行点并主动 `parked`。
- 轨迹：`[-1.65, 0.00] -> [-0.85, 0.00]`。actor 从机器人左侧横向向右移动，在左侧通路入口旁停车；右侧保留连续绕行空间，actor 不会进入小车的回转出口。
- 尺寸/速度：`0.40 × 0.40 × 1.00 m`，`0.15 m/s`。
- 期望行为：机器人从 actor 的右侧通过，并继续进入原定左侧狭窄通道；Runner 仅在 actor 仍运动时观察到右侧通过和前向超越才判为有效。

#### D. 临时封堵 `temporary_block`

- 轨迹：`[0.95, 0.15] -> [-0.10, 0.15] -> [0.95, 0.15]`
- 巡航速度：`0.50 m/s`
- 机器人触发门：北向穿过 `y=-1.80`
- 五档中心停留时间：`0.60 / 0.90 / 1.20 / 1.50 / 1.80 s`
- 种子：`7431–7435`
- 期望行为：机器人在障碍占用冲突区时停车或绕行，障碍退出后及时恢复，不发生永久锁死。

### 5.3 正式计数

- 四类各 5 次，共 20 次。
- 每个 case 的五档难度是预先冻结的运行矩阵，不使用运行结果决定下一个参数。
- 无效交互不得算作成功或失败；如果是实验装置问题，修复后必须更换配置哈希并重跑完整 20 次，禁止只补跑失败种子。

## 6. 第二层：5 次全屋回归

全屋路线固定为：

```text
S/G1 -> G2 -> G3 -> G4 -> G5 -> G1
```

- 仅在第一航段放置标准 `crossing`，分别使用五档启动延迟。
- 种子：`7501–7505`。
- 动态交互结束后障碍停放在中央冲突区外，仍保留实体；后续航段不再有主动运动的动态障碍。
- 目的不是再次统计四类能力，而是验证一次动态减速/绕行后，Nav2、Costmap、Collision Monitor 和行为树仍能继续完成全屋路线。

## 7. 单轮动态避障可视化

### 7.1 定位

单轮可视化用于观察和调试，不属于正式验收，不生成“通过/不通过”的正式统计结论。它必须能够反复运行同一个 case/variant，方便比较机器人行为。

### 7.2 计划提供的入口

后续实现一个独立入口：

```bash
./scripts/run_kujiale_dynamic_visual.sh \
  --case crossing \
  --variant 3 \
  --seed 7403
```

支持的参数：

- `--case crossing|oncoming|same_direction_slow|local_bypass|temporary_block`
- `--variant 1|2|3|4|5`
- `--seed SEED`，默认使用该 case/variant 的冻结种子
- `--record`，可选；保存这一轮 MCAP 和轻量 JSON/CSV，默认只观察不落正式证据
- `--no-rviz`，仅在已经打开专用 RViz 时使用

该脚本只发送一次 G1→G2 任务，结束后退出，不自动进入下一轮。

### 7.3 Isaac Sim 中的观察内容

- 动态障碍真实几何及其完整运动；
- 机器人与障碍是否存在物理接触或异常推挤；
- 障碍是否按空间门触发；
- 障碍完成交互后是否在安全终点停放并保持实体；
- 第三人称相机持续跟随机器人，能够同时看到机器人、障碍和前方通道。

### 7.4 RViz 专用布局

计划新增 `dynamic_avoidance.rviz`，默认显示：

- `warehouse_new` 地图；
- RobotModel、TF、LaserScan；
- Local/Global Costmap；
- Global Plan；
- MPPI 最终轨迹 `/optimal_trajectory`；
- Ground Truth Path；
- Collision Monitor Stop/Slowdown Zone；
- 动态障碍当前位置、历史轨迹、剩余轨迹和冲突区；
- 机器人与障碍的最近连线；
- 文本状态栏。

默认不显示 MPPI Candidate Trajectories，避免大量采样线遮挡机器人和障碍的真实行为。

### 7.5 动态 Marker 约定

计划发布 `/experiment/dynamic_obstacles/markers`：

- 灰色：`waiting/parked/retired`
- 紫色：`moving`
- 橙色：`dwell`
- 绿色：`clearing`
- 洋红色：`safety_yield`
- 红色：`guard_aborted`（仅旧配置兼容状态）
- 紫色线：障碍完整规划轨迹
- 黄色圆柱/圆环：冲突区
- 青色线：机器人与当前障碍的实时最短距离
- 白色文字：
  - case / variant / seed
  - 障碍状态和实时速度
  - 当前净距和本轮最小净距
  - 首次感知时间
  - 响应延迟
  - 障碍清场后的恢复时间

可视化只消费真值做显示与评价，不得把 Marker 或真值状态接入 Nav2。

## 8. 评价指标

### 8.1 交互有效性

每轮必须同时满足：

- 机器人进入障碍物 2 m 交互范围时，障碍处于 `moving`；`temporary_block` 允许处于 `dwell`。
- 机器人位于 2 m 范围期间，障碍累计位移至少 `0.60 m`。
- 最近相遇发生在障碍进入 `parked` 之前。
- 除 `temporary_block` 的停留阶段外，最近相遇时障碍速度不低于 `0.15 m/s`。
- 障碍状态、机器人 GT 和传感器/Costmap 数据均完整。

不满足时记为 `invalid_interaction`，属于实验装置问题，不得当作机器人成功。

### 8.2 安全

- `/simulation/collision` 始终为 false；
- 不发生 `safety_yield` 或 `near_contact_abort`；
- 成功轮的最小 footprint 净距不低于 `0.10 m`；
- 障碍物不得向机器人注入可见的推挤位移。

### 8.3 动态响应

以动态障碍 footprint 首次与 Local Costmap 标记重合的时刻作为首次感知：

- 1.0 秒内，机器人必须出现至少 `0.15 m/s` 的降速，或相对无障碍基线路径产生至少 `0.15 m` 的横向偏移；
- 记录 Collision Monitor 的 slowdown/stop，但不要求必须触发；
- 如果机器人在障碍进入 Costmap 前已经提前绕行，按首次路径偏移时刻记录，并在报告中注明。

### 8.4 恢复

- 障碍离开冲突区后 3.0 秒内，机器人线速度恢复到至少 `0.20 m/s`；
- 障碍清场后连续停车不得超过 5.0 秒；
- 最终正常到达当前目标并满足位置、朝向和静止条件。

### 8.5 批量门槛

| 指标 | 门槛 |
|---|---|
| 受控交互有效性 | 20/20 |
| 受控导航成功 | 至少 18/20 |
| 每类成功 | 至少 4/5 |
| 受控物理接触 | 0/20 |
| 受控 `safety_yield` / `near_contact_abort` | 0/20 |
| 全屋交互有效性 | 5/5 |
| 全屋动态安全 | 5/5 |
| 全屋路线成功 | 至少 4/5 |

允许最多两轮导航/超时失败，不允许用“18/20”放宽碰撞要求。

## 9. 证据与报告

每轮保存：

- `run_manifest.json`
- `run_summary.json`
- `events.jsonl`
- `ground_truth.csv.gz`
- `odom.csv.gz`
- `cmd_vel.csv.gz`
- `dynamic_obstacles.csv.gz`，包含 20 Hz 位置、速度、状态和净距
- `leg_metrics.csv`
- Scan、Local Costmap、Global Costmap 快照
- 配置、地图、Nav2 和代码哈希

MCAP 最终交付至少保留：

- 所有失败轮；
- 每个受控 case 一轮最接近中位数的成功样本；
- 一轮全屋成功代表样本；
- 用户明确指定需要保留的单轮可视化记录。

报告必须包含：

- 四类 × 五档结果矩阵；
- 每类成功率、最小净距、响应延迟和恢复延迟；
- 机器人与障碍的时空轨迹；
- 障碍状态甘特图；
- 速度、角速度、Collision Monitor 状态和 Costmap 标记时间线；
- 失败分类：碰撞、保护终止、无效交互、Nav2 失败、超时、恢复锁死、证据缺失；
- 自包含 HTML 以及 JSON、CSV、PNG 导出。

## 10. 最终验收由用户执行

### 10.1 计划提供的脚本

已实现：

```bash
./scripts/run_kujiale_dynamic_acceptance.sh pilot CAMPAIGN_ID
./scripts/run_kujiale_dynamic_acceptance.sh controlled-20 CAMPAIGN_ID
./scripts/run_kujiale_dynamic_acceptance.sh full-route-5 CAMPAIGN_ID
./scripts/run_kujiale_dynamic_acceptance.sh report CAMPAIGN_ID
```

也提供顺序执行入口：

```bash
./scripts/run_kujiale_dynamic_acceptance.sh all CAMPAIGN_ID
```

脚本必须：

- 检查 Isaac、Nav2、地图、出生点、Ideal 里程计和动态配置是否完全匹配；
- 检查 20/5 运行矩阵和种子，没有重复或遗漏；
- 拒绝覆盖已有实验目录；
- 在开始正式批次时冻结并写入全部哈希；
- 无论门槛是否通过都生成报告；
- 门槛未通过时返回非零状态，但保留完整结果；
- 明确打印报告路径和失败轮路径。

### 10.2 执行责任

- Codex 在开发阶段只运行单元测试、静态检查和必要的非正式单轮/集成验证。
- 正式 `controlled-20` 和 `full-route-5` 由用户在确认 GUI、场景和机器人状态后亲自启动。
- Codex 不以 Pilot、单轮可视化或合成测试代替最终验收。
- 用户运行完成后，可以把结果目录交给 Codex做只读分析、报告解读或失败诊断。

## 11. 计划实现阶段

### 阶段 A：配置与运动引擎

- 新增动态 schema v3、case/variant 运行矩阵和单一轨迹真相源；
- 实现多段轨迹、梯形速度曲线、空间触发、安全终点停放和 Reset 隔离；
- 实现 map swept-footprint 预检；
- 实现 60 Hz footprint 净距和防推车保护。

### 阶段 B：Runner 与评价

- Reset 前设置 case、variant 和 seed；
- 采集完整障碍时序；
- 实现交互有效性、最小净距、响应和恢复指标；
- 区分导航失败、动态安全失败和实验装置无效。

### 阶段 C：单轮可视化

- 新增 Marker 发布器和 `dynamic_avoidance.rviz`；
- 新增单轮可视化脚本；
- 验证四类 nominal case 均能在 Isaac 和 RViz 中完整观察。

### 阶段 D：验收脚本与报告

- 新增 `pilot / controlled-20 / full-route-5 / report / all` 入口；
- 新增动态专题 HTML/JSON/CSV/PNG 报告；
- 编写用户操作说明、失败处理说明和结果目录结构。

## 12. 开发测试要求

- schema v1/v2 向后兼容及 v3 严格校验；
- case/variant/seed 确定性；
- 多段轨迹插值、加减速、dwell 和 `parked` 终态；
- 空间门方向与速度条件；
- Reset 后无状态泄漏；
- 最大相对速度下保护带在接触前触发；
- synthetic trace 覆盖有效交互、无效交互、响应延迟、恢复延迟和聚合门槛；
- RViz 配置 topic、QoS、默认显示项和“候选轨迹关闭”契约；
- 所有新增 shell 脚本通过 `bash -n` 和 ShellCheck；
- 相关 Python/ROS 包测试和项目完整静态测试通过。

正式验收数据不属于开发测试，不在 Codex 的实现阶段生成。

## 13. 当前分支交付与操作

本分支已实现 schema v3 物理配置、四类 case 的 20 轮矩阵、5 轮完整路线矩阵、按 `case_id/variant_id/seed` 的 reset 选择、受空间门和速度约束的状态机、加减速轨迹、最大 1.8 秒短暂停留、在安全终点 `parked`（保持可见和物理碰撞）、20 Hz 状态证据，以及接触前的 `safety_yield`。`safety_yield` 会让 actor 原地保持实体、等待机器人离开后再恢复；Runner 会把该轮标为安全失败，但不会删除 actor 或取消导航。

`same_direction_slow` 保留为动态改道验证。局部绕障由 `local_bypass` 单轮调优：Runner 需先观察到 actor 横移，再观察机器人位于 actor 右侧至少 `0.35 m`、并在 actor 仍运动或已按计划 `parked` 后前向超过 actor 至少 `0.35 m`；否则记录 `local_right_bypass_not_observed`。`safety_yield` 仍是失败，不能被计划停车掩盖。

单轮观察：

```bash
./scripts/run_kujiale_dynamic_visual.sh --case crossing --variant 1 --seed 7401
```

在 RViz 中加载 `ros2_ws/src/robot_description/rviz/dynamic_avoidance.rviz`。该配置只显示 MPPI 最优轨迹（不显示候选轨迹），并订阅 `/experiment/dynamic_obstacles/markers` 显示 actor、状态文本、预测路线和最短距离线。它不构成正式验收。

正式验收前，请以启用动态障碍物的 Isaac 实例、`warehouse_new` 地图、`long_route_start_g1` 和 Ideal 里程计启动导航栈；然后由你执行第 10 节脚本。报告会生成 `summary.json`、`runs.csv`、`success_overview.png` 和 `index.html`；每轮原始证据位于 `data/experiment_runs/kujiale_dynamic_<CAMPAIGN_ID>/`。

推荐使用 `./scripts/run_kujiale_dynamic_isaac.sh` 启动 Isaac。该脚本还会启用 `/ground_truth/odom`；这是 Runner 的复位对齐和轨迹证据的必需输入。
