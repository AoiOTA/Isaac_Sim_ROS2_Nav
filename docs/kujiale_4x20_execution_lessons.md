# Kujiale 4×20 执行问题复盘与恢复手册

> 最近复核：2026-07-26
>
> 适用分支：`codex/kujiale-4x20-appearance-benchmark`
>
> 当前正式批次：`20260725-210035`

本文归纳4×20实现、实机执行、动态复跑和报告生成期间已经遇到的问题。它解释“为什么失败、当前代码如何避免、再次遇到时怎么恢复”，不替代
[`user_manual.md`](user_manual.md) 的日常命令、[`troubleshooting.md`](troubleshooting.md) 的通用排障或
[`kujiale_4x20_appearance_benchmark_plan.md`](kujiale_4x20_appearance_benchmark_plan.md) 的验收规格。

## 1. 当前结论和证据边界

同一 campaign `20260725-210035` 已完成80个正式轮次，根报告 `complete=true`、`passed=true`、`issues=[]`。
本地唯一机器可读结论是：

```text
data/reports/kujiale_4x20_20260725-210035/benchmark.json
```

| 条件 | 严格成功 | 物理无碰撞 | 验收 |
| --- | ---: | ---: | --- |
| 静态基准 | 20/20 | 20/20 | 通过，最大路径偏差 10.1687% |
| 静态＋外观变化 | 20/20 | 20/20 | 通过，最大路径偏差 10.1442% |
| 动态基准 | 19/20 | 20/20 | 通过，门槛18/20 |
| 动态＋外观变化 | 19/20 | 20/20 | 通过，门槛18/20 |

动态两组各有一轮因 `three_stage_dynamic_behavior_not_observed` 计为严格失败；它们没有物理碰撞。部分动态成功轮出现
`dynamic_min_clearance_below_0_10m` 或 `dynamic_actor_safety_yield`，报告按当前口径保留为风险警告，不伪装成“未发生”。
原始批次与生成报告在 `data/` 下并由 Git 忽略；仓库提交的是生成器、场景、规则、测试和本复盘。

## 2. 已遇到的问题、根因和当前处理

| 问题/症状 | 根因 | 当前处理与恢复 |
| --- | --- | --- |
| `static/dynamic Isaac supervisor exited` | 旧 Isaac 仍持有单实例锁，或一键脚本曾用额外 `setsid` 包装，导致 `$!` 指向短命 helper 而非真实 supervisor | 当前一键脚本把 Isaac/ROS supervisor 保持为可等待的直接子进程，并验证独立进程组。先运行 `diagnose.sh` 和 `clean_runtime.sh --dry-run`，不要 `pkill` 或手删锁。 |
| Ctrl+C 后 Nav2/Isaac 遗留，下一次启动再次报锁 | 只停止了外层 shell，没有向真实 launch/Isaac 进程组发送 SIGINT | 当前 `stop_stage` 定位并停止 ROS launch 与 Isaac 的专用进程组，顺序固定为 Nav2 后 Isaac。 |
| pilot 启动时报 `run_indices` INTEGER/STRING 类型不匹配 | ROS launch 把单值字符串 `"2"` 自动推断成整数，而 runner 参数定义为逗号分隔字符串 | `experiment.launch.py` 使用 `ParameterValue(..., value_type=str)` 固定类型；改动 launch 后必须重新构建。 |
| `ros2 launch` 返回0，但 pilot 节点已异常退出 | launch 本身成功启动不代表子节点完成任务 | pilot 返回后必须验证指定 `run_index` 的 manifest、summary、结果、完整性和校验和；缺失或失败时正式40轮不会开始。 |
| 失败 pilot 在 `--resume` 时被当作“完整”而跳过 | 旧续跑只看证据是否完整，没有要求 pilot 成功 | 当前会把失败/不完整 pilot 隔离为 `.incomplete-<UTC>` 后缀并重新执行；验证器忽略隔离目录。 |
| 切换分支后运行到一半报 `run_kujiale_4x20.sh: No such file or directory` | 一键脚本来自4×20分支，但依赖脚本在切换后的分支不存在 | 正式批次期间不要切换分支。启动前执行 `git status --short --branch`，确认当前分支及三个4×20脚本同时存在。 |
| 动态 pilot/正式轮导航完成却判失败 | 曾把小于0.10 m净距或 actor `safety_yield` 直接作为失败；另一些轮是真正缺少三阶段行为证据 | 当前距离门槛是物理碰撞：真实接触、导航失败、`guard_aborted`、行为/证据缺失仍失败；低净距和让停保留为警告。必须同时看 `failure_reason` 与 `warning_reason`。 |
| 为了让 pilot 通过而大幅移动动态障碍 | 改变actor轨迹会降低交互强度，使实验不再验证原问题 | 已恢复三阶段actor的基线几何、速度、触发和运动学。后续控制优化只在独立动态Nav2 profile中进行；改actor必须重做地图图示、哈希和全部动态证据。 |
| 静态已完成，动态失败却被迫从头跑静态 | 早期只有最终总报告，阶段结果没有独立保留 | 当前静态40轮后立即生成 `static_2x20`，动态40轮后生成 `dynamic_2x20`；`--dynamic-only` 不启动静态栈。 |
| `--resume` 混用修改前后的动态配置 | 完整旧轮会被跳过，导致一个40轮集合包含不同代码/配置 | 同配置的中断才使用 `--resume`。修改动态代码、Nav2参数或验收逻辑后，使用新ID重跑动态，或先把同一ID下全部动态证据和动态报告移出，再从零运行；不能只删失败轮。 |
| 报告命令无输出/没有生成文件 | 报告模块曾缺少模块主入口 | 当前 `python3 -m robot_experiments.kujiale_4x20_campaign` 会执行 `main()`；报告命令仍以输出JSON和退出码为准。 |
| 报告只有统计图，没有每轮实际路径 | 初版报告没有读取 `ground_truth.csv.gz` | 当前HTML可按条件、seed、外观和结果筛选逐轮GT路径。 |
| 静态/动态轨迹图缺少障碍物，或子报告显示四组地图 | 初版只画OccupancyGrid；未区分 scoped report | 静态轨迹叠加六个版本化静态障碍；动态轨迹读取本轮 `dynamic_obstacles.csv.gz`，仅画实际触发actor的实测轨迹、起终点和方向；2×20子报告只显示本范围两张地图。 |
| 只推送到了一个GitHub仓库 | remote 只有单一push URL或只验证了一个ref | `origin` 当前有两个push URL；每次推送后分别用 `git ls-remote --heads` 验证 AoiOTA 与 HDU-ASL 的同名分支指向相同提交。 |

## 3. 续跑和重跑决策

### 3.1 同一代码和配置下被中断

保留不完整证据，用同一ID续跑：

```bash
cd /home/lyb/Workspace/Isaac_Sim_ROS2_Nav
./scripts/run_kujiale_4x20_all.sh <CAMPAIGN_ID> --resume --skip-build
```

若只中断动态阶段：

```bash
./scripts/run_kujiale_4x20_all.sh <CAMPAIGN_ID> --dynamic-only --resume --skip-build
```

完整且校验通过的轮次会跳过；不完整轮次会隔离后重跑。

### 3.2 动态实现或参数已经改变

不要在旧动态正式轮次上使用 `--resume`。优先使用新ID执行：

```bash
./scripts/run_kujiale_4x20_all.sh --dynamic-only --skip-build
```

若必须把新动态证据与同一ID下已完成的静态40轮组成完整4×20报告，应先停止所有项目进程，逐项确认保留
`pilot-static/`、`static/`、`orchestrator/static-*` 和 `static_2x20/`，再把该ID下的
`pilot-dynamic/`、`dynamic/`、`orchestrator/dynamic-*` 与 `dynamic_2x20/` 移入系统回收站。随后不带
`--resume` 从零运行：

```bash
./scripts/run_kujiale_4x20_all.sh <CAMPAIGN_ID> --dynamic-only --skip-build
```

这是一次明确的证据替换操作；不要只删除失败轮，也不要删除整个campaign根目录。

### 3.3 只重绘报告

报告器升级不需要重新运行Isaac：

```bash
./scripts/run_kujiale_4x20.sh static-report  <CAMPAIGN_ID> --replace
./scripts/run_kujiale_4x20.sh dynamic-report <CAMPAIGN_ID> --replace
./scripts/run_kujiale_4x20.sh report         <CAMPAIGN_ID> --replace
```

退出码0表示报告范围通过；2表示报告已生成但门槛或证据未通过；其他非零值才是报告生成错误。

## 4. 日志和每轮输出如何判读

正常启动顺序是：

1. `starting ... Isaac supervisor`
2. `starting ... Nav2 supervisor`
3. `preflight passed`
4. `experiment_runner ... process started`
5. 每轮输出 `completed ...: success|failure`
6. runner clean exit、阶段栈有序停止、报告路径输出

`success` 只说明该轮满足当前严格规则；控制是否平滑还要结合 `cmd_vel.csv.gz`、`ground_truth.csv.gz`、
`maximum_route_recoveries` 和分段后的 `dynamic-nav2.log`。相反，`warning_reason` 非空并不等于失败；
必须以 `strict_success`、`physical_collision_free` 和 `failure_reason` 联合判断。

## 5. 提交与双远程核验

代码、测试和相关文档应在同一阶段提交；生成的80轮数据和报告不进入Git。当前分支推送后执行：

```bash
git ls-remote --heads git@github.com:AoiOTA/Isaac_Sim_ROS2_Nav.git \
  codex/kujiale-4x20-appearance-benchmark
git ls-remote --heads git@github.com:HDU-ASL/Bio_Nav_Module3.git \
  codex/kujiale-4x20-appearance-benchmark
```

两个输出的提交SHA必须一致。
