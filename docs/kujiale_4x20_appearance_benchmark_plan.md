# Kujiale 4×20 光照/颜色鲁棒性实验计划

> 基线：`codex/dynamic-obstacle-benchmark-redesign` 的
> `f9cf7cbe45f8a260f5535a4c9689b6fc783451c8`
>
> 实施分支：`codex/kujiale-4x20-appearance-benchmark`
>
> 当前状态：仅完成计划文档和可复现地图示意图；未启动 Isaac/ROS，未执行80轮实验。

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
动态交互无效、保护中止或证据缺失，均计为失败。成功率按组独立判定，不能用80轮总平均掩盖单组失败。

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

## 4. 调度、证据和运行接口（后续实现）

- 静态两组使用 `stable` Nav2 配置；动态两组使用 `dynamic_avoidance` 配置。
- 基准和变化轮次使用相同seed配对，并按AB/BA交替顺序执行。
- 动态组使用现有5个变体，每个变体4次；动态＋外观变化组让每个动态变体分别覆盖四种外观配置。
- 支持 `--resume`：仅跳过证据和校验均完整的轮次；不完整目录隔离保留，不自动删除。
- 启动前校验ROS环境、地图/场景哈希、话题、TF、Nav2配置及可用空间；小于`120 GiB`时拒绝开始。
- 静态阶段与动态阶段之间必须重启 Isaac/Nav2；正式80轮期间禁止调参。

预期的分阶段接口如下。它们是后续实现目标，不是本次文档交付已运行的命令。

```bash
cd /home/lyb/Workspace/Isaac_Sim_ROS2_Nav
export CAMPAIGN_ID="$(date +%Y%m%d-%H%M%S)"

# 静态栈
./scripts/run_kujiale_4x20_isaac.sh static --headless
./scripts/run_kujiale_4x20.sh pilot static "$CAMPAIGN_ID"
./scripts/run_kujiale_4x20.sh static-pair "$CAMPAIGN_ID"

# Ctrl+C结束静态栈后启动动态栈
./scripts/run_kujiale_4x20_isaac.sh dynamic --headless
./scripts/run_kujiale_4x20.sh pilot dynamic "$CAMPAIGN_ID"
./scripts/run_kujiale_4x20.sh dynamic-pair "$CAMPAIGN_ID"

./scripts/run_kujiale_4x20.sh status "$CAMPAIGN_ID"
./scripts/run_kujiale_4x20.sh report "$CAMPAIGN_ID"
```

每种模式先运行一个变化条件的 `pilot`，验证光照、材质、RGB快照和导航证据后才执行正式批次。

## 5. 计划中的报告与校验

完整campaign在单一目录中生成 `index.html`、`report.pdf`、`report.md`、`benchmark.json/csv`、
`data_dictionary.md`、`evidence_index.json`、校验和及PNG图；不复制完整MCAP。报告首页嵌入本文件的
4×20测试地图，支持按实验组、seed、外观配置、动态变体和结果筛选。

报告展示各组成功率和置信区间、耗时、路径长度、偏差、恢复次数、碰撞、动态交互有效性、失败原因，
以及基准/外观变化的配对对比。即使未达到门槛也必须生成报告；验收通过返回0，批次完成但门槛或证据失败返回2。

后续自动化测试至少覆盖：80轮矩阵完整性、外观分配、路线一致性、Session Layer应用与恢复、
断点续跑、证据校验、门槛判定，以及HTML/PDF/PNG报告生成。

## 6. 本阶段边界

本次变更仅创建分支、本文档和地图制图能力/产物。不会启动 Isaac/ROS，不会执行pilot或正式实验，
不会创建80轮运行数据，也不会提交或推送分支。
