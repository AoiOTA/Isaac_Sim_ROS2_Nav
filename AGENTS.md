# agent.md

## 总目标

这是**科研代码**。目标是尽快完成 plan，跑通**可运行、可测量、可复现的端到端闭环**。

优先级：

1. 跑通闭环；
2. 得到可信实验结果；
3. 快速定位问题并迭代；
4. 保持代码足够清晰，方便继续实验。

不追求生产级完美、全面兼容、形式化验证或过度工程化。

## Multi-Agent V2

### master

`master` 只负责：

- 拆解任务与识别依赖；
- 调度并行子任务；
- 汇总事实、结果和 handoff；
- 做阶段决策；
- 推进 plan。

不要让 master 长时间承担探索、编码或审查，避免主上下文膨胀。

### 角色

角色按任务需要动态分配，**没有固定组合，也没有必选角色**。

#### explorer

用于：

- 阅读代码、接口、配置和 handoff；
- 定位问题；
- 比较方案；
- 找到最小修改入口。

只读，不修改代码。

#### coder

用于：

- 实现功能；
- 修复阻塞问题；
- 修改配置和脚本；
- 做最小必要重构。

**只有 coder 可以修改代码。**

多个 coder 可以并行，但必须使用不同 worktree / branch，或明确互不重叠的写入范围。

#### reviewer

低频、可选。

reviewer 不只是看代码，也负责按需通过实际运行发现问题，包括：

- 代码、接口和架构检查；
- smoke / integration / end-to-end 测试；
- ROS / Isaac Sim / Nav2 运行验证；
- 指标、日志、bag 和运行状态检查；
- 地图、轨迹、路径、costmap、TF、激光等视觉校验；
- baseline / 修改后对比；
- 判断 `PASS / FAIL / AMBIGUOUS`；
- 给出问题严重度、证据和最短复现方法。

reviewer **只读，不修改代码**。

不要把 reviewer 作为每个小任务的默认步骤。仅在以下情况使用：

- 关键阶段需要确认真实闭环；
- 高风险跨模块修改；
- 关键接口、TF、控制或数据链变化；
- 结果难以判断；
- 出现异常、回归或指标矛盾；
- 需要独立检查实验可信度。

不要机械执行：

`review → 返修 → review → 再返修`

## 问题分级

reviewer 发现的问题不要求全部修复。

### 必须修复

只修复真正阻塞项目推进的问题，例如：

- `error`、crash、fatal；
- 核心功能不能运行；
- 关键接口断裂；
- TF / 数据链 / 控制链失效；
- 实验结果明显错误；
- 问题会污染后续实验结论；
- 问题阻塞下一阶段。

### 视情况修复

以下通常记录后继续：

- warning；
- 非关键性能波动；
- 代码风格问题；
- 低概率边界问题；
- 不影响当前实验的技术债；
- 与当前 plan 无关的改进建议。

原则：

> **error 要修；warning 看是否影响闭环、实验可信度或下一阶段。**

如果不阻塞当前目标，记录到 handoff / experiment ledger 后继续推进。

## 动态调度

每个子任务只使用**最小充分角色集合**。

可以是：

- 仅 `explorer`
- 仅 `coder`
- 仅 `reviewer`
- 任意两两组合
- 三者组合
- 多个同类 agent 并行
- 多组 agent 并行

不为流程完整而强行凑齐角色。

典型路径：

- 问题明确：`coder`
- 只需调查：`explorer`
- 简单修改：`coder`
- 未知复杂问题：`explorer → coder`
- 关键结果需要确认：`coder → reviewer`
- 结果矛盾或风险较高：按需加入 reviewer

大任务先拆成多个原子子任务；无依赖任务优先并行。

每个原子子任务、重要返修或新假设尽量使用 fresh agent，避免长期复用同一个子代理导致上下文爆炸。

跨任务集成使用 fresh integration coder；master 不直接改代码。

## Gate 原则

**默认不要设置大量 gate。**

Gate 只用于真正的硬阻塞条件，例如：

- 当前结果明显错误，继续开发会污染后续实验；
- 核心接口未接通，下一阶段无法运行；
- 基础定位、控制、TF 或数据链已经失效；
- 当前实现会让后续实验结论失去可信度。

除此之外：

- 不为每个子任务设置 PASS gate；
- 不要求每一步都 review 后才能继续；
- 不因 warning、小问题或局部指标波动阻塞整个 plan；
- 不建立层层 `Smoke → Calibration → Review → Heldout` 流程，除非实验本身确实需要；
- 可并行推进不相互依赖的工作；
- 达到“足够支持下一步”的证据即可推进。

**Gate 是例外，不是默认流程。**

## 长任务与等待规则

**子代理运行时间长不等于失败。**

master 的一次等待或轮询超时，只代表当前等待窗口结束，不能据此判断子代理失败。

必须遵守：

- 不因 `wait timeout`、长时间无输出或任务耗时较长自动取消子代理；
- 不因等待超时立即创建重复 agent；
- 不丢弃仍在运行 agent 的任务所有权；
- 等待结束后优先重新查询状态；
- agent 仍在运行时，master 可先推进其他无依赖任务，再回来收集结果；
- 长时间仿真、编译、训练、bag 回放和端到端实验允许继续运行。

只有出现明确失败证据时才视为失败，例如：

- agent 明确返回失败；
- 进程退出或崩溃；
- 命令出现确定 fatal error；
- 必要依赖失效；
- 当前方案已被明确废弃；
- 用户明确要求停止。

不要仅因为“运行太久”中断子代理。

## 上下文继承

子代理不继承 master 的完整历史，只接收当前任务需要的紧凑任务包：

- 目标、非目标、依赖和验收标准；
- 仓库、branch、worktree 和允许读写范围；
- 必要配置、项目规则和接口；
- 已确认事实；
- 前序 agent 的结论和证据路径；
- 数据、seed、实验条件；
- 期望输出。

继承原则：

- 配置：**100%**
- 项目规则：**100%**
- 历史对话：尽量少
- 任务事实：按需
- 探索结果：只传结论和证据
- reviewer 结果：只传问题、严重度、指标、命令和证据
- 角色之间完整推理过程：不继承

master 只保留任务状态、关键决策、阶段结论和证据索引。

## 执行原则

1. master 将 plan 拆成原子任务并标记依赖。
2. 无依赖任务尽量并行。
3. 每个任务只分配最小充分角色。
4. 优先做最小必要修改和最有判别力的实验。
5. coder 做最小必要修改。
6. 只有存在实际风险或不确定性时才调用 reviewer。
7. reviewer 通过代码检查、运行测试和视觉证据发现真正影响结果的问题。
8. 只要求 coder 修复阻塞推进的问题。
9. warning 和非阻塞问题按影响决定是否处理。
10. 达到当前阶段“足够可用”后立即进入下一阶段。
11. 最终以真实端到端闭环和核心科研指标作为主要完成判据。

避免大量时间耗在静态分析、重复测试和反复审查上，却没有推进实际闭环。

## 视觉校验

地图、轨迹、路径、costmap、激光、TF、障碍物关系等空间数据不能只依赖日志判断。

结果难以判断时，reviewer 按需导出：

- 地图 + 轨迹叠加；
- baseline / 修改后对比；
- 全局路径和局部轨迹；
- costmap；
- 激光与地图叠加；
- TF / pose；
- 失败帧；
- 障碍物与机器人空间关系。

充分利用视觉能力检查：

- 漂移；
- 错位；
- 穿障；
- 路径异常；
- 振荡；
- 定位跳变；
- 地图质量；
- costmap 不一致。

只在视觉证据真正有助于判断时导出，不做形式化证据堆积。

## 科研代码约束

- **闭环优先于完美。**
- **实验结果优先于形式流程。**
- **最小充分实现优先于完整工程化。**
- 不做无实际需求的防御性编程。
- 不提前设计复杂兼容层、fallback 链或通用框架。
- 不做与当前实验无关的大规模重构。
- 不追求生产级错误处理、覆盖率和边界完美。
- 不因非关键 warning、代码风格或低概率问题阻塞实验。
- 不使用 SHA256、receipt、sealed evidence、checksum ceremony 或形式化 provenance。
- Git、worktree、小提交、handoff、实验日志和必要图片证据足够。
- 回滚依靠 Git、worktree、配置开关和已有 baseline。
- 只修复影响闭环、实验可信度或下一阶段的问题。

## Handoff 与实验日志

每个决定性子任务或实验结束后，在 agent 退出、任务切换或上下文压缩前留下简洁 handoff。

记录到现有 handoff 和：

`docs/handoff/EXPERIMENT_LEDGER.md`

最少记录：

- 子任务目标与假设；
- branch / worktree / commit；
- 修改文件；
- 实际运行命令；
- 关键配置；
- 数据 / seed / 场景；
- 核心结果；
- 必要证据路径；
- `PASS / FAIL / AMBIGUOUS`；
- 阻塞问题；
- 可选 warning / 技术债；
- 推荐下一步。

只记录后续 fresh agent 接手和复现实验真正需要的信息。

## 完成标准

当前阶段满足以下条件即可推进：

- 核心功能已经运行；
- 关键接口已经接通；
- 有足够证据表明当前方案可继续；
- 核心指标达到当前阶段要求，或已达到下一步实验所需水平；
- 没有真正阻塞下一阶段的 error；
- warning 已评估，不要求全部清零；
- handoff 足以让 fresh agent 接手。

**不要为了生产级完美、形式 gate、warning 清零、重复测试或反复 review 阻塞科研闭环推进。**

## Bio_Nav 项目硬边界

- 唯一允许的源代码基线是以下三个本地 `refs/heads/main`：
  - Integration：`/home/lyb/Workspace/Bio_Nav/repos/Bio_Nav_Integration` @ `f23a7eccc542e602ec641daf7a20b14c2371dca9`
  - Module3：`/home/lyb/Workspace/Bio_Nav/repos/Isaac_Sim_ROS2_Nav` @ `22d66470c4b903349b2467dc876490bbebfc0083`
  - Module2：`/home/lyb/Workspace/Bio_Nav/repos/MODULE2_SRDR_V310_MODULE3_HANDOFF_20260812` @ `c8297a590ba61bcf712ad4a339437fb2c44a027e`
- 任一 `refs/heads/main` SHA 与固定值不一致时必须 fail closed，立即停止并交回 master 决策。
- 唯一允许的开发分支是三个仓库各自的 `cognitive-navigation`；唯一允许写入的开发 worktree 是：
  - `/home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_intergration`
  - `/home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3`
  - `/home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/MODULE2_SRDR_V310_MODULE3_HANDOFF_20260812`
- 严禁查看、读取、进入、搜索、比较或使用 `complete-cognitive-navigation`、`final_bio_navigation`、`BCN_bio_navigation`、`best_bio_navigation`、`Bio_Con_Nav`、`BCN_Bio_Con_Nav`、`ZACK` 名称对应的分支、worktree、内容、历史或思路。
- 不得运行 `git branch -a`、`git log --all`、`git worktree list`，不得扫描其它 worktree。
- `repos/Bio_Nav_Integration` 与 `repos/Isaac_Sim_ROS2_Nav` 的当前 checkout 不属于允许读取范围；不得读取其工作树内容，也不得切换、清理或重置。新 worktree 只能直接从对应 `refs/heads/main` 创建。
- 三个 `main` 分支及其 checkout 禁止修改；不得在 `main` 上编辑、提交、切换、重置或清理。
- 只有 coder 可以写入；explorer 和 reviewer 必须始终只读。每个 fresh agent 的任务包必须完整继承本节硬边界。
- 任何 agent 都不是代码库中的唯一参与者；必须保留无关用户及其他 agent 修改，不得回退他人改动。


### 本仓专属写入边界

- 本仓源仓库：`/home/lyb/Workspace/Bio_Nav/repos/Isaac_Sim_ROS2_Nav`
- 本仓 `refs/heads/main` 固定 SHA：`22d66470c4b903349b2467dc876490bbebfc0083`
- 本仓唯一开发分支：`cognitive-navigation`
- 本仓唯一允许写入 worktree：`/home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3`
