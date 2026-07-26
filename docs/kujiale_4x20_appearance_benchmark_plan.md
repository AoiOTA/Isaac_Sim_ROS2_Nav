# Kujiale 4×20 光照/颜色鲁棒性实验计划

> 基线：`codex/dynamic-obstacle-benchmark-redesign` 的
> `f9cf7cbe45f8a260f5535a4c9689b6fc783451c8`
>
> 实施分支：`codex/kujiale-4x20-appearance-benchmark`
>
> 当前状态：正式批次 `20260725-210035` 已完成80轮；完整报告 `complete=true`、`passed=true`、`issues=[]`。
> 静态两组均20/20，动态两组均19/20，四组物理无碰撞均20/20。

## 1. 目标与实验矩阵

在 Kujiale USD、`warehouse_new`、Ideal 定位、`long_route_start_g1` 和固定闭环
`G1 → G2 → G3 → G4 → G5 → G1` 下，重新运行四组各20轮的实验，不复用历史结果。

| 组别 | 场景 | 外观 | 轮数 | 通过门槛 |
| --- | --- | --- | ---: | --- |
| 静态基准 | 6个低矮RGB-D障碍 | `baseline` | 20 | 严格成功且无碰撞 `≥19/20` |
| 静态＋外观变化 | 同一静态几何 | 4种配置各5轮 | 20 | 严格成功且无碰撞 `≥19/20` |
| 动态基准 | `full_route_three_stage` | `baseline` | 20 | 严格成功且无碰撞 `≥18/20` |
| 动态＋外观变化 | 同一动态运动学 | 4种配置各5轮 | 20 | 严格成功且无碰撞 `≥18/20` |

静态成功轮次的 Ground Truth 路径偏差还必须不超过 `20%`。动态轮次若发生碰撞、
动态交互无效、保护中止或证据缺失，均计为失败。动态最小净距低于 `0.10 m`
或出现 actor `safety_yield` 时保留为风险警告，但只要 `/simulation/collision`
确认没有真实物理接触，就不单独据此判失败。成功率按组独立判定，不能用80轮总平均掩盖单组失败。

对外统计的名称、分子分母和边界以[通用指标定义与对外表述](kujiale_4x20_metric_definitions.md)为准。该文档以 LaTeX 公式完整定义静态/动态避障成功率、路径相对理论最优偏差和导航成功率；静态路径偏差只与同约束的静态理论最短可行参考比较，动态场景不套用该固定参考。

## 2. 测试地图示意图

下图由 `scripts/generate_kujiale_long_route_maps.py` 直接读取 `warehouse_new`、出生点、
静态/动态场景和campaign YAML生成。生成器会拒绝静态与动态路线、航点ID或出生点约定不一致的输入。

![4×20测试地图与实验矩阵](figures/kujiale_4x20_test_matrix_map.png)

四个面板使用同一张 OccupancyGrid、同一条目标发送顺序和同一套航点；青绿色虚线只表示
`S(G1) → G2 → G3 → G4 → G5 → G1` 的发送顺序，不代表 Nav2 实际规划或GT轨迹。
外观变化面板特意不改绘地图或障碍：变化仅通过每轮固定的 USD Session Layer 生效，路线、
几何、碰撞体和动态障碍运动学均保持不变。

### 2.1 静态与动态细节图

![静态场景](figures/kujiale_long_route_static_map.png)

![动态场景](figures/kujiale_long_route_dynamic_map.png)

![三阶段动态交互局部细节](figures/kujiale_three_stage_dynamic_details.png)

静态图中的6个低矮障碍由RGB-D链路识别。动态图保留三个顺序交互：`local_bypass`、
`g2_g3_exit` 与 `g5_g1_crossing`；任一时刻最多一个actor可见且可碰撞。

重新生成所有地图：

```bash
cd /home/lyb/Workspace/Isaac_Sim_ROS2_Nav
python3 scripts/generate_kujiale_long_route_maps.py
```

## 3. 外观变化定义

外观变化使用匿名 USD Session Layer，绝不写回原始USD，不改变场景几何、碰撞、导航地图或actor运动学。
每轮开始前应用一次，并在整轮内保持固定。

| ID | 灯光强度 | 色温 | 材质颜色偏移 |
| --- | ---: | ---: | --- |
| `baseline` | 不覆盖 | 不覆盖 | 不覆盖 |
| `dim_warm` | `0.4×` | `3000 K` | 暖色 `+35°` |
| `dim_cool` | `0.4×` | `7500 K` | 冷色 `-35°` |
| `bright_warm` | `1.6×` | `3000 K` | 暖色 `+35°` |
| `bright_cool` | `1.6×` | `7500 K` | 冷色 `-35°` |

每个外观变化组中，四个非基准配置各出现5次。每轮须记录外观配置、配置哈希、场景对象清单摘要，
并在发送首个目标前保存RGB快照；运行时发布 `/experiment/appearance/state`。

> 导航当前消费 `/scan` 和 `/camera/front/depth/points`。本实验验证外观改变下的任务稳定执行，
> 不宣称导航算法依赖RGB颜色完成导航。

## 4. 调度、证据和运行接口

- 静态两组使用 `stable` Nav2 配置；动态两组使用 `dynamic_avoidance` 配置。
- 基准和变化轮次使用相同seed配对，并按AB/BA交替顺序执行。
- 动态组使用现有5个变体，每个变体4次；动态＋外观变化组让每个动态变体分别覆盖四种外观配置。
- 支持 `--resume`：仅跳过证据和校验均完整的轮次；不完整目录隔离保留，不自动删除。
- 启动前校验ROS环境、地图/场景哈希、话题、TF、Nav2配置及可用空间；小于`120 GiB`时拒绝开始。
- 静态阶段与动态阶段之间必须重启 Isaac/Nav2；正式80轮期间禁止调参。

以下是推荐的实际运行入口。单命令监督器会重新构建ROS工作区、启动静态 Isaac/Nav2、
完成静态 pilot 和40轮后立即生成并保留静态 `2×20` 报告、先关闭 Nav2 再关闭 Isaac、启动新的动态栈、
完成动态 pilot 和40轮并生成动态 `2×20` 报告，最后才生成同一批次的总 `4×20` 报告；不需要另开终端或手动切换：

```bash
cd /home/lyb/Workspace/Isaac_Sim_ROS2_Nav
./scripts/run_kujiale_4x20_all.sh
```

省略 ID 时脚本自动创建 `YYYYMMDD-HHMMSS`。要固定 ID：

```bash
./scripts/run_kujiale_4x20_all.sh 20260725-120000
```

每个阶段会等待 `preflight` 校验磁盘、地图/场景哈希、ROS话题、TF和实际 Nav2 profile；
Isaac 或 Nav2 在启动超时（默认900秒）前退出时，监督器会失败并指出对应日志。日志保存在
`data/experiment_runs/kujiale_4x20_<campaign_id>/orchestrator/`。每种模式的 `pilot` 都运行矩阵中
首个外观变化轮次；pilot 仅验证环境和证据，所有正式报告自动排除它。静态/动态子报告各只统计对应的40轮，
总报告只统计同一批次的80轮。

中断后，保留同一 ID 运行：

```bash
./scripts/run_kujiale_4x20_all.sh 20260725-120000 --resume
```

完整且校验通过的正式轮次会跳过，不完整目录仍隔离保留。`pilot` 是正式40轮的前置门：若先前
pilot 已完整写入但结果失败，`--resume` 会将该 pilot 目录隔离为带 `.incomplete-<UTC>` 后缀的保留证据，
然后用当前配置重新执行它；不会静默复用失败 pilot，也不会改写正式轮次。已经生成报告的 ID 不允许覆盖，请使用新 ID。
如果刚完成构建，可加 `--skip-build`；必要时可用 `--startup-timeout-sec 1200` 调整每阶段启动等待上限。

### 4.1 动态问题后的独立复测（不重跑静态）

静态 40 轮已完成并生成 `static_2x20` 报告后，动态出现问题时不要用 `--resume` 混入使用旧配置的
已完成动态正式轮次。使用新的 campaign ID，只启动动态栈、运行动态 pilot 与40轮、并生成独立的
`dynamic_2x20` 报告：

```bash
./scripts/run_kujiale_4x20_all.sh --dynamic-only
# 已构建工作区时：
./scripts/run_kujiale_4x20_all.sh --dynamic-only --skip-build
```

该模式不启动静态 Isaac/Nav2，也不改写或覆盖已有静态报告。它只验证 `dynamic_baseline` 和
`dynamic_appearance` 两组各20轮；动态子报告不能自动同其他 campaign 的静态子报告合成为完整4×20结论。
为已经完成静态阶段、但尚未产生静态报告的现有 campaign 补报时，执行：

```bash
./scripts/run_kujiale_4x20.sh static-report <已有CAMPAIGN_ID>
```

当前批次静态阶段完成后可使用：

```bash
./scripts/run_kujiale_4x20.sh static-report 20260725-210035
```

`run_kujiale_4x20_isaac.sh`、`run_ros.sh` 和 `run_kujiale_4x20.sh` 的分开调用仍可用于人工观察或单阶段
调试，但不再是正式批次的推荐入口。

### 4.2 单轮 GUI/RViz 诊断（不计入4×20）

四组正式实验之外，可单独观察共同路线 `G2 → G3 → G4 → G5 → G1`。两种模式均自动发送全部航点，
不需要在RViz手工发布Goal。

静态全屋单轮依次执行：

```bash
# 终端 A：Isaac GUI + 六个静态RGB-D障碍
./scripts/run_kujiale_4x20_isaac.sh static

# 终端 B：Navigation + RViz
./scripts/run_ros.sh navigation odometry_mode:=ideal \
  spawn_pose_name:=long_route_start_g1 nav2_profile:=stable

# 终端 C：自动 G2 → G3 → G4 → G5 → G1
./scripts/run_visual_route.sh static
```

动态全屋单轮先停止静态B/A，再依次执行：

```bash
# 终端 A：Isaac GUI + 三阶段动态actor
./scripts/run_kujiale_dynamic_isaac.sh

# 终端 B：Navigation + RViz
./scripts/run_ros.sh navigation odometry_mode:=ideal \
  spawn_pose_name:=long_route_start_g1 nav2_profile:=dynamic_avoidance

# 终端 C：自动全屋航点和三阶段触发
./scripts/run_kujiale_three_stage_visual.sh full --variant 1 --seed 7501
```

完整操作、停止顺序和观察项见
[`user_manual.md` 第8节](user_manual.md#8-可视化单轮全屋长距离测试isaac-gui--rviz)。

这些 GUI 诊断不生成正式 80 轮证据或结论；不能与 `data/experiment_runs/kujiale_4x20_*` 的报告混用。

### 4.3 高分辨率第三人称外观核验（不计入4×20）

`appearance_rgb_before_goal.ppm` 是 320×180 的前向 RGB-D 证据快照，不能替代外观展示图。若需直接比较五种固定外观，在没有其他 Isaac 实例运行时执行：

```bash
./scripts/capture_kujiale_appearance_preview.sh
```

该命令 headless 导出固定**客厅观察位**的 1920×1080 场景视角 `baseline`、`dim_warm`、`dim_cool`、`bright_warm`、`bright_cool` 图片和可点击放大的 `index.html`。画面以客厅家具、墙面、地面和灯光为主体，而非小车前向或随车相机；它复用同一匿名 Session Layer 外观配置，不创建 ROS/Nav2、RTX LiDAR、动态 actor 或正式 evidence；源 USD、几何、碰撞、地图与动态运动学均不改变。

## 5. 自动报告与校验

完整 campaign 在 `data/reports/kujiale_4x20_<campaign_id>/` 生成总 `index.html`、`report.pdf`、`report.md`、
`benchmark.json/csv`、`data_dictionary.md`、`evidence_index.json`、校验和及PNG图；不复制完整MCAP。静态阶段
完成后立即写入 `static_2x20/`，动态阶段完成后写入 `dynamic_2x20/`，从而动态失败不会丢失静态报告。每份报告首页
嵌入本文件的测试地图，支持按实验组、seed、外观配置、动态变体和结果筛选。每个有
`ground_truth.csv.gz` 证据的正式轮次都会生成 `warehouse_new` OccupancyGrid 叠加的实际 GT 路径图；
HTML 的“逐轮实际 GT 路径”选择器随上述筛选联动，绿点为起点、红点为终点。
静态轮次的路径图还会从正式 `kujiale_long_range_static.yaml` 读取六个静态 RGB-D 障碍物的位置和尺寸，
以橙色矩形叠加；因此它们不会因未写入 OccupancyGrid 而在图中缺失。
动态轮次的路径图会读取本轮 `dynamic_obstacles.csv.gz`，仅叠加实际触发的三阶段 actor；彩色虚线为
actor 的实测运动轨迹，方形为起点、X为终点，箭头表示运动方向。始终处于 `waiting` 的未启用 actor
不会绘制，避免把运行时对象清单误当成本轮参与交互的障碍物。
静态/动态 2×20 子报告只显示本范围的两张条件地图；完整4×20总报告拆成“静态两组对比”和“动态两组对比”，不再将四张地图缩放到同一画布。报告首先以 S1–D2 中文卡片解释四组分别控制什么：静态或三阶段动态障碍、基准外观或四种外观配置，以及20轮如何分配。地图、统计图和可筛选的逐轮 GT 路径均可点击，在当前页弹窗放大。除目录内的轻量 `index.html` 外，报告同时生成 `index_portable.html`：将地图、统计图及所有逐轮 GT 轨迹以 Base64 内嵌，复制这一文件到另一台电脑即可离线预览图片和使用轨迹筛选，无需同传任何 PNG，也不依赖浏览器打开大型 data URL。报告摘要之后嵌入对应的4×速避障演示视频（静态/动态子报告各一段，完整报告两段）；指定 GitHub attachment 不可用时自动回退到仓库内的同一 MP4。可筛选的“逐轮实际 GT 路径”置于统计图之前。

报告展示四组成功率和置信区间、耗时分布、路径长度、静态偏差、恢复次数、碰撞、动态交互有效性和失败原因，
并保留基准/外观变化的同seed配对记录。完整报告的 HTML、Markdown 和 PDF 都应在结果前写明实验如何执行及当次指标结果：以 LaTeX 渲染静态/动态避障成功率、静态相对理论最短可行路径偏差及分静态/动态/总体导航成功率公式，并给出每项指标的文字定义、统计边界和适用条件。即使未达到门槛也必须生成报告；验收通过返回0，批次完成但门槛或证据失败返回2。
`static_2x20` 与 `dynamic_2x20` 是各自独立的验收报告，不会自动将不同 campaign 的结果拼接成完整4×20报告。
若报告器升级后需要仅重绘既有报告（不改变原始证据），使用 `--replace`：

```bash
./scripts/run_kujiale_4x20.sh static-report <CAMPAIGN_ID> --replace
```

自动化测试覆盖：80轮矩阵完整性、外观分配、路线一致性、Session Layer应用与恢复、
断点续跑、证据校验、门槛判定，以及HTML/PDF/PNG报告生成。

## 6. 本阶段边界

本分支的运行器、报告器、配置、地图和文档已经由本机独占Isaac/Nav2会话完成正式验证。当前可引用结论仅属于
campaign `20260725-210035` 及其原始校验和；修改地图、actor、Nav2 profile、验收规则或外观矩阵后必须新建证据，
不能沿用本次成功率。执行中已解决的问题和安全恢复方法见
[`kujiale_4x20_execution_lessons.md`](kujiale_4x20_execution_lessons.md)。
