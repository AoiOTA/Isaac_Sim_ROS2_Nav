# agent.md

## 1. 目标与默认循环

这是**科研性质代码**。第一目标是尽快完成 plan，跑通真实、可测量、可复现的端到端闭环，而不是追求生产级完美。

```text
最小实现
  ↓
focused test
  ↓
按需 live runs
  ↓
看数据 / 图片 / 日志
  ↓
真实 blocker？
 ├─ 是 → 最小定位 → 最小修复
 └─ 否 → 下一任务
```

不要先建设复杂的 contract、gate、validator、runner、provenance 和大套测试，最后才进入 live。

核心原则：

- **闭环优先于完美，真实实验优先于形式流程。**
- 改动保持小、可逆、易回滚，保护已知可运行 baseline。
- 只修影响闭环、实验可信度或下一阶段的 `error / fatal / blocker`；`warning` 按影响处理。
- 不做无真实需求的防御性编程、多层 retry/fallback、兼容层、恢复框架和无关重构。
- 默认不用 SHA256、checksum、receipt、sealed evidence、大型 Evidence Contract 或形式化 provenance 系统。
- 次数、阈值、run 数、agent 数和状态示例都只是参考，应按风险、不确定性、证据和资源动态调整。
- **复杂度必须由已经发生的真实问题证明。**

---

## 2. Multi-Agent V2

### Master

`master` 只负责拆解、调度、汇总、决策和推进，不长期承担探索、编码、长实验或详细 review，避免主对话上下文爆炸。

### 动态角色

角色按任务需要灵活组合，没有固定编组或必选角色：

- `explorer`：只读探索代码、接口、配置、根因和最小修改入口。
- `coder`：唯一允许修改项目代码、运行配置和受版本控制实现的角色。
- `reviewer`：低频按需，通过代码检查、focused test、live run、日志和视觉证据发现问题；不改实现，可生成实验产物并更新本次验证记录。

多个 coder 可并行，但必须使用独立 worktree/branch，或明确互不重叠的写入范围；需要汇总时由 fresh integration coder 集成，master 不直接改代码。

调度原则：

- 大任务拆成边界清晰的原子任务，无依赖任务优先并行。
- 不重复 spawn 完全相同的任务，除非明确做独立方案比较或交叉验证。
- 新任务、新假设和重要返修优先使用 fresh agent。
- agent 完成、失败或废弃后及时留下 handoff 并释放槽位。
- `wait timeout`、运行时间长或暂时无输出不等于失败；仍在运行时可先推进其他任务。
- 只有明确 crash、fatal、进程退出、依赖失效或底层长期无活动时才停止。

### 紧凑任务包

子代理只接收当前任务必要信息：

```text
目标 / 非目标 / 验收条件
repo / branch / worktree / 读写范围
项目规则 / 配置 / 必要接口
已确认事实 / 前序结论 / 证据路径
seed / scene / 数据 / 期望输出
```

配置和项目规则完整继承；历史对话最少继承；只传结论和证据，不传完整推理过程。除非任务明确要求，不进入、不搜索、不 diff、不修改范围外的 branch/worktree。

---

## 3. 构建、版本与 Installed-Space

### 开发阶段

使用：

```text
clean worktree
+ 单一明确 underlay
+ 一个 canonical build/install
+ focused test
+ 真实 live 数据
```

避免：

- 每轮新建 overlay、临时 underlay 或 `/tmp` build；
- 连续 source 多个 workspace 的 `setup.bash`；
- 手工修改 `PYTHONPATH`、`AMENT_PREFIX_PATH` 掩盖安装问题；
- 让 shell 历史决定包解析顺序；
- 把临时 build/install 作为后续构建依赖。

怀疑污染时，回到 clean shell，source 唯一环境入口，确认关键包实际来源；必要时在 canonical workspace clean rebuild，不再叠一层 overlay。

Git HEAD 不能证明实际运行版本。涉及安装、接口或导入问题时，按需检查 package prefix、executable、Python `__file__`、launch/config/message 的 installed-space 来源。

局部实现可增量编译；公共消息、接口、ABI、生成代码或依赖变化时，重编受影响依赖闭包，禁止混合新旧 install 继续运行。

### 多仓与 Final

- 同名 branch、目录名和 worktree 名不代表兼容；关键实验记录各仓具体 commit 组合。
- 关键依赖使用明确 commit/tag/release，不依赖浮动分支。
- 公共接口变化时同步更新生产者、消费者和必要测试，并进行联合 build + focused integration。
- Final 不直接使用频繁切分支和局部编译的开发 `install/`；使用冻结 commit 组合、稳定路径下的 clean isolated build/install、单一 source 链和 installed-space 运行。
- 强隔离只用于 Final 或真实污染问题，不扩散到日常开发。

---

## 4. 配置、资产与 ROS 环境

- 不在脚本中写死用户目录；优先 package share、repo-relative path、显式 config root 或参数。
- 小型关键配置纳入 Git；地图、模型和大型资产记录明确路径、场景/版本和获取方式。
- 代码、地图、模型、配置和场景作为明确组合使用，不手工临时拼接。
- 关键配置缺失时明确报错，不静默回退到旧模型、旧地图、旧模式或默认通信域；可选默认值应记录实际值。
- 开发、Pilot、Final 使用少量明确配置，不扩展成 profile/wrapper/manifest 矩阵。
- socket、PID、lock 放本机 runtime 目录，不放 NAS；bag、图片、模型和大型实验数据放 NAS。

每次 run 的以下参数只能有一个权威来源，并由子进程继承：

```text
ROS_DOMAIN_ID
RMW implementation
use_sim_time
关键路径和 config
```

父进程、子进程、profile 和 wrapper 不得分别覆盖，也不要在缺失时偷偷使用硬编码默认值。

时间语义：

- 仿真事件、运动、数据对齐和仿真 TTL：ROS/simulation time；
- 进程等待和 watchdog：monotonic time；
- 人类记录：wall time。

关键 topic 按需检查 QoS、实际消息数、频率、时间戳单调性和数据新鲜度，不能只检查 topic 是否存在。处理速度不足时使用 bounded queue、latest-wins 或丢弃过期帧，不无界 FIFO 积压。

---

## 5. 启动、Runner 与生命周期

一个 run 尽量只有：

```text
一个 owner
一个启动入口
一个 reset 入口
一个 cleanup 路径
```

### Readiness 与 Runner

- 不用固定 `sleep` 证明就绪；只检查少量核心、可观察条件。
- 服务返回成功不一定表示下游状态和数据流已稳定。
- 启动依赖保持单向，避免循环等待。
- runner/harness 保持薄层，只负责准备环境、启动入口、传 config、收集必要数据和停止 run。
- 不把算法判断、评估业务、数据转换、身份管理和复杂状态机写进 runner。
- 优先一个 canonical runner/launcher + 少量 config + 显式参数，避免 `mixed/shadow/active/collection`、phase-specific wrapper 和 `runner_v2/final/safe` 矩阵。

实验失败时先用最短直接入口区分：

```text
产品逻辑直接运行成功？
 ├─ 是 → runner/harness 问题，优先简化 runner
 └─ 否 → 再调查产品逻辑
```

### 进程与 Reset

- 优先 ROS launch 或明确顶层 owner 管理进程树，不用大型 Bash 手工维护 PID/PGID/socket/lock 矩阵。
- cleanup 只清理由本 run 创建的资源，不使用全局 `pkill`。
- 停止 owner 后做有界重查，处理晚启动子进程。
- 正常退出允许必要数据落盘；故障注入按实验定义快速断流，不混用停止语义。
- 多个组件不能同时拥有 reset 权限；reset owner 协调真正有状态的核心组件。
- Reset 只做：停止 run → 恢复必要状态 → 确认核心 node/topic/pose/state → 下一 run。
- 只有真实出现跨 episode 污染、stale event 或并发冲突时，才增加最小 generation/边界标识。
- 不把停止意图、停止产出数据、socket 断开、进程退出和落盘完成视为同一时刻。

---

## 6. 状态机、Gate 与接口语义

- 状态只表达真实不同的运行行为；行为相同就合并。
- 不为每个异常新增 state、generation、flag 和 gate；新增补丁时删除失效旧路径。
- Feature Flag 只用于 baseline/experimental、必要 A/B 或临时隔离；避免组合矩阵，稳定后删除。
- Gate 是例外：只有核心进程、必需接口、TF/control/data chain、场景/初始状态或实验可信度失效时才 fail-closed。
- 非核心 diagnostic、可选 telemetry、辅助 topic、warning 和局部指标异常默认记录后继续。
- 区分语义身份、接口兼容版本、配置版本和内容 revision；一个字段只表达一种语义。
- 异步 ROS 消费者不要求完全相同 sequence/revision；普通传播延迟不是 blocker。
- 不兼容接口明确报错，不允许静默降级产生不可解释结果。
- 新增 runner、validator、wrapper、state、flag、contract、recovery 或 fallback 前，优先复用、删除、合并或替换现有逻辑。

> **随着项目推进，系统应逐渐收敛，而不是返修越多越复杂。**

---

## 7. 数据、测试与评估

### 数据

在数据生成边界做少量高价值检查：

- 必需 topic 到达后才输出样本；
- 时间戳、同步误差、shape、dtype、标签范围和有效状态正确；
- 图像、位姿、标签和运动窗口属于同一时刻/episode；
- reset、episode 和时间不连续处切断序列。

连续静止段按需下采样；train/validation/test 按 episode、轨迹、场景或时间块划分，不按相邻帧随机切分；accepted/rejected/train/validation/test 显式分离。检查保持直接、轻量，不建设数据验证平台。

### 测试与评估

- mock 只用于局部纯逻辑；跨进程、launch、Python import、消息接口、QoS 和安装路径按需验证真实 installed-space。
- 公共接口或多仓依赖变化时，做一次联合 build + focused integration smoke。
- 不要求每个小修改全量测试，也不把 focused test、单次 live、Pilot 和 Final 混为同一证明等级。
- evaluator 不应比被测系统更复杂；判据在看结果前确定。
- collision、timeout、unreachable 等业务失败可以是有效结果；只有关键数据缺失、损坏或条件不成立才判实验无效。
- 一次判别性实验尽量只修改一个主要因素。
- 产品代码、runner、evaluator 和数据转换工具保持依赖边界；实验工具不成为产品启动的强依赖。
- 稳定主线只保留少量必要 clean build、focused tests 和跨仓 installed-space smoke，不让实验分支进入重型 CI。

---

## 8. 验证、视觉与记录

`Smoke / Pilot / Final` 是可选验证强度，不是固定流水线。

- **Smoke：能不能跑。** 当前 clean worktree、canonical build、少量 focused checks、最短运行路径。
- **Pilot：方案是否值得继续。** 按需记录 commit 组合、config、seed/scene、ROS_DOMAIN_ID、NAS 路径、若干 live runs、核心指标和必要截图。
- **Final：正式结果。** 冻结 commit/config/seed/dataset，使用 clean isolated installed-space 和正式统计；复用 Pilot 已跑通的 runner 和数据路径，不重建证据基础设施。

地图、轨迹、路径、costmap、激光和 TF 难以仅靠数字判断时，按需导出 overlay、对比图、costmap、scan-map、TF/pose 或 failure frame，并保留必要 frame、scale、start/goal、run ID 和 legend。

Single Source of Truth：

```text
实验指标 / run 结论   → docs/handoff/EXPERIMENT_LEDGER.md
bag / 图片 / 原始数据 → NAS
阶段结论 / 接手信息   → handoff
代码 / 配置变化       → Git
master                → 摘要 + 索引
```

重要任务、关键 live run、agent 切换或上下文压缩前留下简洁 handoff，只记录：

```text
branch / worktree / 各仓 commit
实际环境入口 / build/install 来源
config / seed / scene
运行方法
结果与证据路径
blocker / warning
下一步
```

不记录完整推理、终端流水账或 receipt/hash 证明链。

---

## 9. 完成与推进

当前任务满足以下条件即可继续：

- 核心功能能运行；
- 关键接口已接通；
- 最小充分验证已提供足够证据；
- 没有真实 blocker；
- warning 已评估；
- baseline / rollback 路径明确；
- fresh agent 能根据 handoff 接手。

继续同一路径前问：

> **下一次尝试能否提供新的判别信息？**

若连续尝试没有新信息：

```text
不是 blocker → 记录 → 下一任务
是 blocker   → 换假设 / 换方案 / fresh agent
```

始终坚持：

> **最小实现 → focused test → 按需 live runs → 看真实数据 → 只修真实 blocker → 下一任务。**

> **开发阶段相信 clean worktree + 单一 canonical build + focused test + 真实 live 数据；只有 Final 才按需使用冻结的多仓版本、clean isolated installed-space 和正式统计。**

> **科研闭环是主任务，基础设施只为闭环服务。**

`agent.md` 本身也遵循同一原则：**没有真实问题证明需要，就不要继续增加规则。**
