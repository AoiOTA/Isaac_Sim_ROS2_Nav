# agent.md

## 总目标

以最快速度完成 plan，跑通**可运行、可测量的端到端闭环**。这是科研代码：优先最小改动、快速实验、可逆迭代和结果验证，不做与当前闭环无关的生产化、通用化或防御性工程。

## Multi-Agent V2

- **master** 只负责拆解、调度、决策、汇总和阶段推进，不承担大范围探索、具体编码或长期审查。
- 先按组件、接口、假设、实验和依赖把大任务拆成原子子任务，形成任务图；独立子任务优先并行。
- **不设固定的 `explorer + coder + reviewer` 组合。** 每个子任务按实际需要选择最小充分角色，可使用：
  - 仅 `explorer`、仅 `coder` 或仅 `reviewer`；
  - 任意两种角色组合；
  - 三种角色组合；
  - 多个同类角色或多组角色并行。
- 角色按任务风险和信息缺口分配：
  - 事实、接口、运行时或数据不清楚时使用 explorer；
  - 需要修改仓库时使用 coder；
  - 结果高风险、存在歧义、需要独立验收或回归检查时使用 reviewer。
- 不为流程完整而强行创建角色。简单且边界清晰的修改可只用 coder；纯调查可只用 explorer；独立审计可只用 reviewer。
- 每个原子子任务、返修轮次或假设变化都使用 fresh agent；不要让同一个 explorer、coder 或 reviewer 长期承载多个任务，避免子代理上下文膨胀。
- **只有 coder 可以修改仓库文件。** explorer 和 reviewer 始终只读，只返回结论、证据和 handoff。
- 可以并行使用多个 coder，但同一 worktree、branch 或文件范围同一时刻只能有一个写入者。并行 coder 必须使用独立 worktree/branch，或严格不重叠的写入范围。
- 多个实现需要汇总时，由 fresh integration coder 集成；是否再启用 reviewer 由风险决定。master 不直接改代码。

## 上下文继承

子代理不继承 master 的完整历史，只接收完成当前子任务所需的紧凑任务包：

- 子任务 ID、目标、非目标、依赖和验收标准；
- 仓库、branch、worktree、允许读取和修改的范围；
- 必要配置、项目规则、接口、命令、数据和 seed；
- 已确认事实、探索结论、证据位置和待验证假设；
- 期望产物及 handoff 格式。

继承原则：

- 配置和项目规则：**100% 继承**；
- 历史对话：尽量不继承；
- 任务事实：按需继承；
- 探索结果：只传结论、证据、文件与行号；
- 角色之间不传推理过程、冗长日志或无关上下文。

master 只保留任务状态、关键决策、阶段结论和证据索引；详细过程及时落盘。

## 动态执行流程

1. master 拆分任务图，明确依赖、并行关系、风险和写入边界。
2. 为每个子任务选择最小充分角色，不套固定流水线。
3. explorer、coder、reviewer 可独立或组合执行；无依赖任务并行推进。
4. coder 只修改指定范围，并运行最小且有判别力的测试。
5. reviewer 仅在确有独立审查价值时启用；发现问题后创建新的返修子任务和 fresh coder。
6. 需要跨任务集成时，创建 fresh integration coder 完成合并和端到端验证。
7. 每个子任务结束立即形成 handoff；需要写入仓库的日志由指定 coder 或 integration coder 落盘。
8. 当前阶段满足验收标准后继续推进，不因形式化流程阻塞闭环。

## 视觉校验

- 地图、轨迹、路径、costmap、激光、TF、障碍物间距等空间数据不能只靠日志判断。
- 结果不明确时，导出截图、轨迹叠加图、地图与激光叠加图、costmap 快照或失败帧，并使用视觉能力校验。
- 优先生成 baseline 与修改后结果的同尺度对比图。
- 同时记录生成命令、数据来源、配置、seed 和证据路径。

## 科研代码约束

- 不增加未经实际失败证明需要的兼容层、fallback 链、重试金字塔、冗余校验或提前抽象。
- 不做无关重构，不扩大修改范围；优先验证关键假设并尽快打通闭环。
- 不使用 SHA256、receipt、sealed evidence、checksum ceremony 或形式化 provenance 流程。
- 普通 Git 历史、独立 worktree、小提交、handoff、实验日志和图片证据足够。
- 回滚依靠小提交、worktree、配置开关或现有 baseline。

## Handoff 与实验日志

每个子任务或决定性实验结束后，必须在 agent 退出、任务切换或上下文压缩前形成结构化 handoff。需要落盘时，由 coder 更新现有 handoff 和 `docs/handoff/EXPERIMENT_LEDGER.md`。

最少记录：

- 子任务 ID、角色配置、目标、假设和依赖；
- branch、worktree、commit 和修改文件；
- 精确命令、配置、数据集和 seed；
- 指标、截图及其他证据路径；
- `PASS / FAIL / AMBIGUOUS`；
- 已知问题、阻塞原因和唯一推荐下一步。

## 决策原则

- 按任务需要分配角色，而不是追求固定人员配置。
- 优先并行验证独立假设，避免单个 agent 顺序处理整个阶段。
- 优先最小且有判别力的实验，避免叠加多个猜测性修改。
- 证据足以决策时立即推进；证据不足时只补最关键的探索、修改或审查。
- 遇到阻塞时保持仓库可运行，并留下精确复现命令、证据和下一批子任务包。

## Bio_Nav 项目硬边界

- 唯一允许的基线是三个本地 `refs/heads/main`：Integration `f23a7eccc542e602ec641daf7a20b14c2371dca9`、Module3 `22d66470c4b903349b2467dc876490bbebfc0083`、Module2 `c8297a590ba61bcf712ad4a339437fb2c44a027e`。任何 SHA 漂移都必须 fail closed 并交回 master 决策。
- 所有开发只能发生在以下三个 `Bio_Con_Nav` worktree：`/home/lyb/Workspace/Bio_Nav/worktrees/Bio_Con_Nav/bio_nav_intergration`、`/home/lyb/Workspace/Bio_Nav/worktrees/Bio_Con_Nav/bio_nav_module3`、`/home/lyb/Workspace/Bio_Nav/worktrees/Bio_Con_Nav/MODULE2_SRDR_V310_MODULE3_HANDOFF_20260812`。
- 严禁查看、读取、比较或使用 `complete-cognitive-navigation`、`final_bio_navigation`、`BCN_bio_navigation`、`best_bio_navigation` 分支及其对应 worktree 的任何内容。
- 只有 coder 可以写入；explorer 和 reviewer 必须始终只读。每个新 agent 的任务包必须明确继承本节全部硬边界。
- 三个 `main` 分支及其 checkout 禁止修改；不得在 `main` 上编辑、提交、切换、重置或清理。
