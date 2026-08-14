# Attempt31 Rivermark 室外导航科研 Demo

## 当前结论

Attempt31 在独立 Module3/Integration worktree 中打通 Rivermark 室外科研原型，不修改 A21 工作树，也不改变 A21 室内默认入口。当前选择中央环岛与道路区域（Candidate A），保留完整 **80 m × 80 m** 地图；唯一导航栅格为 **1600 × 1600、0.05 m/格**，不再裁成 48 m × 48 m。

该入口最初作为工程科研原型建立连续全局坐标、物理 Jackal、全局 GVG、Nav2 Route Server、Smac/MPPI、逻辑认知区域切换和可选 Module2 edge prior 的真实调用链。2026-08-14 已在独立 evidence 目录完成正式核心 3×20 qualification；工程原型与正式结果的证据边界见下节。

## 2026-08-14 正式资格结果

最终 fail-closed 报告为 `formal_20260814_v075_r30_cache/qualification_v3/attempt31_rivermark_qualification.json`，报告 SHA-256 为 `8abd0a9a9f98ba0f819035ec1dde91fbdf3c459d7490978b01bfebdd35b9af5a`，状态 **PASS**。v3 在不改动 60 轮原始 evidence 的前提下，增加了 Module2 运行时消费审计；先前 v2 派生报告仍原样保留：

| 组别 | 严格成功 | 物理无碰撞 | 关键附加门禁 |
| --- | ---: | ---: | --- |
| static/off | 20/20 | 20/20 | 静态路径偏差最大 3.256%，要求 ≤20% |
| dynamic/v2 medium | 18/20 | 20/20 | 四 actor close-pairing 20/20；完整交互 18/20，要求 ≥90% |
| appearance/medium | 20/20 | 20/20 | 4 个 profile 各 5 轮，20/20 实际应用 |

最终报告索引 60 个互不重复的 `run_summary.json`，每组 20 个；所有逐轮 `checksums.sha256` 和报告 checksum 均复验通过。动态 v2 的物理配置 SHA-256 为 `3be82e12da0a8048911c77906e608c286547b1038bc2b11d541020d8579fb253`。

动态 v1（配置 SHA-256 `de40eebc36e99f2c1399e2fa2f084654dae0c0e69815edcff39fbc578f33b8f2`）先完成了不可变的 20 轮，结果为 16/20 严格成功、20/20 无碰撞，原报告 `formal_20260814_v075_r30_cache/qualification/attempt31_rivermark_qualification.json` 保持 **STOP**。v2 没有改写 v1 行或放宽 1.5 m 门槛；它在新配置哈希和新 evidence 根下前移 crossing/temporary-block 的预注册触发时序，并以独立 20 轮重新评价。v2 两个失败轮仍原样计入，未补跑替换。

16×16 `region_22` 原始 paired benchmark 各有 20 行且质量检查全通过：收敛时间最小改进 99.473%，地图连续更新最小改进 30.313%。Module2 四臂静态因果矩阵不在 Rivermark 运行，仍标记 `DEFERRED_TO_V4`，不参与本次 PASS。

Module2 是否真正进入室外调用链由独立的 `runtime_consumption_only` 门禁证明，
不采用四臂因果推断：dynamic-v2 的 20/20 轮都有健康 V3.10 prior、跨越至少
13 个 cognitive region、产生并应用正 edge delta，且至少 2 个实际选中路线的
cost snapshot 被 Module2 增量改变。全部请求/代价 snapshot 对齐，最终代价满足
`structural + runtime + applied_module2_delta`，没有突破 Module3 的 requested-delta
上界；20 轮累计应用增量为 527.932 m。该证据只证明“请求被 Route Server 有界
消费并进入选中路线代价”，不证明 SR/DR/SRDR 哪一臂带来因果性能提升，后者仍由
V4 负责。

## 地图与物理语义

- 源场景：`/home/lyb/Rivermark/rivermark.usd`，源文件只读。
- 地图原点：`(-52.0182, 111.603, 0)`；范围为 x `[-52.0182, 27.9818)`、y `[111.603, 191.603)`。
- 生成器使用 top-down RGB/depth、PhysX Occupancy 和局部高度跃迁共同判定。
- 路沿阈值为 0.03 m：即便抬高的人行道能从远处坡道到达，局部路沿的较高侧仍标为占据边界。
- USD 道路网格拼接使用独立的 terrain connection tolerance，避免把正常的抬高道路整片误判成黑色障碍。
- 非物理 `BasisCurves`、`NurbsCurves` 和 `Points` 不参与碰撞体补全，也不参与导航障碍判定。
- 生成时只在匿名 session layer 为可见实体网格补碰撞，不回写源 USD。

实物对齐图：

![Rivermark RGB 与路沿边界对齐](../data/rivermark_demo/rivermark_edge_alignment.png)

三联验证图：

![Rivermark 地图验证](../data/rivermark_demo/rivermark_map_validation.png)

## 路线与区域架构

整张地图只有一个全局 `map` 坐标系和一个全局 GVG。当前 GVG 有 642 个节点、674 条物理边、63 个环、31 个连通分量；Demo 起终点位于同一个可行分量。骨架最小间隙约 0.224 m，略高于当前带 padding 的内切半径 0.215 m。

每个 Module2 cognitive canvas 始终是 **16 × 16 cell、1 m/cell**，即覆盖 16 m × 16 m。逻辑区域的 12 m × 12 m 只是不重叠的 core 与区域中心步长；每个 canvas 在 core 四周各多 2 m halo，因此相邻 canvas 重叠 4 m。这是认知上下文分区，不是独立的 Occupancy 地图。区域切换只更新 `cognitive_tile_id` 与 `T_map_canvas`，不重置 `map→odom`、Nav2 action、全局 Route 或机器人任务。Integration 的 Module2 recurrent context 可在区域边界重置，但 Module3 持续持有物理可达性与最终 Route。

数据流如下：

```text
Rivermark USD + collision/depth
  -> 0.05 m OccupancyGrid (1600 x 1600)
  -> global GVG / nav2_route
  -> CanonicalRoute + route progress
  -> Smac planner + MPPI + Collision Monitor
  -> /cmd_vel -> Isaac Jackal

map pose + current region + CanonicalRoute
  -> 16 x 16 cognitive constraints / route-aligned local goal
  -> optional Module2 edge priors
  -> bounded edge-cost adjustment only
```

GVG footprint 与路线预览：

![GVG footprint 可行性](../data/rivermark_demo/gvg_preview/phase3_footprint_feasibility.png)

![GVG 选中路线](../data/rivermark_demo/gvg_preview/phase5_selected_route.png)

## 五航点任务与三类场景

静态、动态、光照/颜色变化三组实验使用完全相同的五航点任务，不以单一终点或临时 steer 点替代完整任务。Route coordinator 只有在最近一次 GoalUpdater 已发送当前航点的原始最终 pose，并且 ground-truth 进入资格实验统一的 0.25 m 航点门槛后，才发布该段完成；到达中间 lookahead 后继续沿同一 CanonicalRoute 推进。

| 航点 | map x (m) | map y (m) | yaw (deg) |
| --- | ---: | ---: | ---: |
| G1 | 1.521014 | 131.813786 | 135.0000 |
| G2 | -15.524852 | 138.328084 | 90.5258 |
| G3 | -23.443200 | 158.463820 | 90.0000 |
| G4 | -36.675295 | 172.938296 | 138.1799 |
| G5 | -42.643200 | 180.578000 | -98.1300 |

五航点与动态 actor 的实物俯视叠加图如下。左图使用正式静态第 1 轮的
ground-truth 轨迹；右图使用正式 dynamic-v2 第 1 轮的机器人轨迹与四个
物理 actor 的实测轨迹，因此不是手绘直线示意：

![Rivermark 五航点静态与动态实测示意](../data/rivermark_demo/rivermark_five_waypoint_scenarios.png)

该图可从冻结 evidence 重新生成：

```bash
python3 scripts/render_rivermark_scenarios.py
```

每组 20 轮，整轮均执行 `start -> G1 -> G2 -> G3 -> G4 -> G5`：

- 静态：20 个固定 seed；成功率和无碰撞率门槛均为 95%，并要求真实轨迹相对冻结最短路的偏差不超过 20%。
- 动态：20 轮均选择 `full_route_four_stage`，每轮依次包含 G2 迎面、G3 横穿、G4 同向慢行、G5 临时阻塞；5 种 variant 各重复 4 次。成功率门槛 90%，不套用静态路径偏差约束。
- 光照/颜色：`dim_warm`、`dim_cool`、`bright_warm`、`bright_cool` 各 5 轮，成功率和无碰撞率门槛均为 90%。外观 profile 不改变地图、碰撞体、航点或动态运动学。

动态 actor 为 0.8 m × 0.6 m × 1.0 m 的物理可见 box。四条轨迹的完整 XY 外接矩形均以 0.05 m 分辨率逐格检查，不允许起点、终点或扫掠路径压入路沿占据格。运行时另外保留 0.45 m kinematic guard：危险接近时 actor 可见且有碰撞地停止让行，不会推动车辆或瞬间消失。每轮四个 actor 都必须完成触发、运动、退场和传感器证据闭环，并且各自相对机器人 footprint 的最近净间距不大于 1.5 m；只启动但远离机器人通过的 actor 不计为有效动态交互。

静态组的冻结参考使用未知格不可通行、0.34 m 安全膨胀的八邻域 A*。五段 0.05 m 总长为 113.0562 m；在保守细分到 0.025 m 后为 112.9665 m，差异 0.0794%，低于 1% 收敛门槛。生成器和地图、场景、spawn、图像 SHA 一并写入 `rivermark_optimal_reference.json`。

## Module2 验证边界

Rivermark 正式范围为静态、动态、外观三组各 20 轮，不运行 Module2 四臂静态因果矩阵。Module2 的 Baseline、SR-only、DR-only、SRDR matched-arm 因果验证由后续 V4 实验负责，不能从 Rivermark 的单一 `medium` 配置推断四臂结论。Rivermark 仅保存 Module2 在线请求、响应、cost delta、模型 ID、健康状态、CanonicalRoute 和真实轨迹，用于证明室外链路确实消费了 Module2 输出；这些运行证据不替代 V4 因果验证。

Rivermark 启用 tile-local edge projection：覆盖率只在 canonical edge 落入当前 16×16 canvas 的真实采样段上计算，至少需要 3 个局部样本，请求增量再按局部段占整条 edge 的比例缩放。不在当前 tile、物理不可达或覆盖不足的 edge 均保持零增量；该机制不切割/新建 GVG edge，也不改变 Module3 的可达性与最终 cost cap。

16×16 cognitive tile 的收敛/连续更新指标使用真实 Rivermark `region_22` A/B 对：

- 相同数值质量下，缓存 `M_SR @ goal` / `M_DR @ dynamic_cost` 相对经典迭代更新的 20 个 paired case，最小学习时间改进 99.47%，中位 99.53%，门槛为不低于 20%。
- A 到局部持久障碍 B 的 20 次 parent-full / child-delta paired update，最小改进 30.31%，中位 37.18%，门槛为不低于 30%。父状态 hash 保持不变，child delta 删除 6 条 transition，物理支持映射不变。

上述两个计算基准只证明认知计算合同；不能单独替代真实五航点导航和避障证据，也不能替代后续 V4 四臂因果验证。

批量入口示例：

```bash
cd /home/lyb/Workspace/Bio_Nav/worktrees/module3/attempt31-outdoor-nav

# 静态 Baseline；正式批量省略 --run-indices
ROS_DOMAIN_ID=232 ./scripts/run_rivermark_campaign.sh \
  static off --run-indices 1 --output data/experiment_runs/attempt31_rivermark/pilots/static_off

# 异构四阶段动态与外观变化
ROS_DOMAIN_ID=232 ./scripts/run_rivermark_campaign.sh dynamic medium
ROS_DOMAIN_ID=232 ./scripts/run_rivermark_campaign.sh appearance medium

# 只读取已完成 evidence，生成统一 fail-closed 资格结果
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/local_setup.bash
ros2 run robot_experiments attempt31_rivermark_qualification \
  --static-root data/experiment_runs/attempt31_rivermark/static/off/runs \
  --dynamic-root data/experiment_runs/attempt31_rivermark/dynamic/medium/runs \
  --appearance-root data/experiment_runs/attempt31_rivermark/appearance/medium/runs \
  --contract-summary data/rivermark_demo/benchmarks/module2_contracts/contract_benchmark_summary.json \
  --output data/experiment_runs/attempt31_rivermark/qualification/qualification.json
```

一次只运行一个 Isaac 实例。pilot 的航点、碰撞、证据完整性、动态交互或路径偏差失败时不得授权正式批次；正式批次中的失败轮必须原样计入，不能删除、覆盖或用补跑轮替换。配置修复必须使用新哈希、新 evidence 根和独立报告，历史 STOP 保持不可变。

## 准备与启动

首次运行先导入本 worktree 的 Jackal 本地资产并构建两个工作区：

```bash
cd /home/lyb/Workspace/Bio_Nav/worktrees/module3/attempt31-outdoor-nav
./scripts/import_assets.sh
./scripts/prepare_rivermark_demo.sh

cd /home/lyb/Workspace/Bio_Nav/worktrees/integration/attempt31-outdoor-nav/ros2_ws
colcon build --symlink-install

cd /home/lyb/Workspace/Bio_Nav/worktrees/module3/attempt31-outdoor-nav/ros2_ws
colcon build --symlink-install
```

静态几何基线：

```bash
cd /home/lyb/Workspace/Bio_Nav/worktrees/module3/attempt31-outdoor-nav
ROS_DOMAIN_ID=231 ./scripts/run_rivermark_demo.sh off static
```

启用可选 Module2 edge prior：

```bash
ROS_DOMAIN_ID=231 ./scripts/run_rivermark_demo.sh module2 static
```

启用物理移动障碍：

```bash
ROS_DOMAIN_ID=231 ./scripts/run_rivermark_demo.sh off dynamic
```

无显示器 smoke 可附加：

```bash
RIVERMARK_HEADLESS=1 RIVERMARK_MAX_STEPS=3000 \
  ROS_DOMAIN_ID=231 ./scripts/run_rivermark_demo.sh off static
```

启动脚本会拒绝不是 0.05 m/格或不是 1600×1600 的地图，并只加载 Attempt31 Integration/Module3 的 `local_setup.bash`，防止历史 ROS overlay 混入。

## Persistent blockage 工程演示

在路线运行时，另一个终端可注入同一 edge 的重复 blocked/clear 观测：

```bash
ROS_DOMAIN_ID=231 ./scripts/trigger_rivermark_blockage.sh blocked
ROS_DOMAIN_ID=231 ./scripts/trigger_rivermark_blockage.sh clear
```

这是明确标注的 **ENGINEERING INJECTION**，用于演示 `RuntimeEdgeState` 的持久确认、关闭边与重路由；它不是传感器推断结果，也不能作为避障率或 qualification 证据。真实动态障碍仍由 LiDAR/RGB-D、Local Costmap、MPPI 与 Collision Monitor 处理。

## 已知边界

- Rivermark 原始场景部分 foliage point-instancer 缺少 prototype，Isaac 首次加载会输出大量渲染 warning；它不进入导航碰撞或栅格生成，但会使 12 GB 场景冷启动约需几十秒。
- 原始场景有少量缺失材质纹理，仅影响显示，不改变几何/碰撞。
- 当前是 Ideal Odometry 科研 Demo；没有宣称真实传感器定位或室外泛化。
- `module2` 模式失败时按现有 timeout 回退 geometry-only；Module2 从不拥有物理可达性或最终 Route。
