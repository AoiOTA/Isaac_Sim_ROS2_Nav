# Module3 源仓、发布仓与引用目录

`AoiOTA/Isaac_Sim_ROS2_Nav` 是 Module3 最初的仓库，也是当前唯一的功能事实源。默认本地工作仓
为 `${BIO_NAV_MODULE3_ROOT}`（由统一 `workspace.env` 导出）。`AoiOTA/Bio_Nav_Module3` 与
`HDU-ASL/Bio_Nav_Module3` 都是从该源仓派生的发布仓，不是独立的功能来源。

## 仓库对应关系

| 角色 | Git 仓库或本地目录 | 规则 |
| --- | --- | --- |
| 本地工作仓 | `${BIO_NAV_MODULE3_ROOT}` | 只在这里维护 Module3 源代码；`main` 跟踪 `origin/main`。 |
| 原始源仓 | `AoiOTA/Isaac_Sim_ROS2_Nav` | `origin`；Module3 功能主线和开放 PR 的唯一来源。 |
| AoiOTA 发布仓 | `AoiOTA/Bio_Nav_Module3` | 本地 remote 名为 `module3-aoi`；发布 `main` 必须来自源仓。 |
| HDU-ASL 发布仓 | `HDU-ASL/Bio_Nav_Module3` | 本地 remote 名为 `module3-hdu`；镜像发布 `main`，另保留所有者分支。 |

整理完成后，三个远端的 `main` 必须解析到同一个完整 commit SHA。发布仓不得在自己的 `main`
上产生源仓没有的功能提交；需要发布时，先在源仓审查和合并，再将同一提交快进到两个发布仓。
禁止通过 cherry-pick、squash 或分别点击两个发布仓的合并按钮制造“内容相同、commit 不同”的
伪镜像。

## 本次主线调和的 commit 依据

| ref | commit | tree | 审计结论 |
| --- | --- | --- | --- |
| 源仓 `main`（调和前） | `65b559793ee897a62c3e63a0955a370cdb1f3c06` | `21f67b1e6bb806ba2f9841d98d337c5ee7c84eb7` | 当前 Module3 完整功能 tree；相对共同基点的 4 个独有提交均为治理文档。 |
| 两个发布仓 `main`（调和前） | `2b94a6b7d247349daccdc7f10fc27858bf7b4387` | `daee27efb82f9173890695b38f901d6f2aef2534` | 接入了较早开发历史，但当前 tree 缺少源仓的报告媒体，并保留较旧 campaign reporter。 |
| 本次双父调和提交 | `4b7799ff106ef68a94b920b172dcc654862bfddd` | `21f67b1e6bb806ba2f9841d98d337c5ee7c84eb7` | 第一父为源仓 `65b5597...`，第二父为发布仓 `2b94a6b...`；tree 与源仓完全一致，未改变运行代码。 |

最终 `main` 还会包含本目录修正文档，因此最终 SHA 以源仓 PR 合并结果为准。合并后只允许将该
同一 SHA 快进到两个发布仓，不得重做另一个合并提交。

## 治理后的分支

| 远端 | 长期分支 | 用途 |
| --- | --- | --- |
| 原始源仓 | `main` | 当前 Linux / Isaac Sim / ROS 2 / Nav2 Module3 功能主线。 |
| 原始源仓 | `codex/navigation-quality-fidelity` | 开放 PR #2 的未合并功能线；PR 合并或明确关闭前保留。 |
| AoiOTA 发布仓 | `main` | 源仓 `main` 的同 SHA 发布镜像。 |
| HDU-ASL 发布仓 | `main` | 源仓 `main` 的同 SHA 发布镜像。 |
| HDU-ASL 发布仓 | `codex/windows-isaacsim-zenoh` | 学长发布的较早 Windows / Zenoh 兼容线；所有者保护。 |
| HDU-ASL 发布仓 | `codex/lc-windows-isaacsim-v1` | 学长发布的较新 Windows 稳定化线；所有者保护。 |

`integration/stage2.2-shadow@5da2f2fc27b7e139b2e852acefbea0bdc0b01228` 是冻结的
Integration 输入，不是仍在开发的分支。整理完成后只由 annotated evidence tag 保存，不再在
发布仓重复保留同指向的长期分支。

旧 `codex/kujiale-navigation-mapping@478aa656...` 已被源仓 `main` 完整包含，不再提供独立
功能。发布仓曾有同 tree、不同 commit 的同名分支，继续保留只会制造镜像失配，因此只在 bundle
中恢复。

## 治理后的标签

三个 Module3 远端保留完全相同的 5 个 annotated tag；每个 tag 的 tag object 与 peeled commit
也必须相同：

| annotated tag | 严格用途 |
| --- | --- |
| `release/module3/standalone-navigation/source-sync-20260730` | 本次源仓同步后的独立 Module3 可部署快照；不表示 Module2 接入或 Integration Gate 通过。 |
| `evidence/integration-stage2.2-shadow/module3-input` | 固定 `5da2f2fc27b7e139b2e852acefbea0bdc0b01228`，只表示 Integration Shadow 使用的 Module3 输入。 |
| `legacy/module3/jackal-physics-contact-ab/20260730` | 当前主线没有的旧 Jackal mass/collision profile、runtime provenance 与 contact A/B 工具。 |
| `legacy/module3/runtime-camera-warehouse-v2/20260730` | 当前主线没有的旧 runtime/camera 工作流及 `warehouse_v2` 地图 bundle。 |
| `legacy/module3/webrtc-streaming/20260730` | 当前主线没有的 `scripts/run_isaac_streaming.sh` WebRTC 入口。 |

原 `baseline/module3/standalone-navigation/20260730` 和
`evidence/stage2.2-shadow/input` 名称没有表达“源仓同步”及“Module3 输入”边界。它们先进入
经校验的 bundle，再由上表两个明确名称替代；不得移动原 tag object。

`release/` 固定可部署软件，`evidence/` 固定 campaign 输入或结果，`legacy/` 只固定当前主线
缺少的旧功能。三类 tag 都不可移动。新 campaign 必须记录完整 SHA，并创建
`evidence/<campaign>/input` 和 `evidence/<campaign>/result` annotated tag。

## 本地 remote 和分支规则

```text
origin         -> AoiOTA/Isaac_Sim_ROS2_Nav
module3-aoi    -> AoiOTA/Bio_Nav_Module3
module3-hdu    -> HDU-ASL/Bio_Nav_Module3
```

本地只建立实际开发需要的 local branch：

- `main` 对应 `origin/main`；
- `codex/navigation-quality-fidelity` 对应源仓开放 PR #2；
- 发布仓 `main`、Shadow 历史和 Windows 所有者分支只需保留为 remote-tracking ref，无需复制出
  同名 local branch。

因此“一一对应”指每个 local branch 有且只有一个 upstream，且每个 remote 名只对应一个 GitHub
仓库；不要求把所有远端分支复制成本地分支。

禁止多个 `origin.pushurl`、通配符删除、force-push、历史重写及 `git gc/prune`。原 ref 和 tag
对象由 `/home/lyb/Workspace/Bio_Nav_Module_branch_archive/20260730/` 下的 bundle、SHA256 与
清单恢复。完整跨仓库 policy 与 tag object 校验由 Bio_Nav_Integration 集中维护。
