# Module3 历史源仓分支与标签目录

`AoiOTA/Isaac_Sim_ROS2_Nav` 是 Module3 的历史开发源仓，不是当前双远端发布镜像。需要部署或
独立测试 Module3 时，应使用 `AoiOTA/Bio_Nav_Module3`；本仓只承担源历史和仍在评审的 PR。

## 当前分支

| 分支 | 用途 | 生命周期 |
| --- | --- | --- |
| `main` | 历史源仓的已合并开发主线。 | 长期 |
| `codex/navigation-quality-fidelity` | 开放 PR #2 “Navigation quality and simulation fidelity upgrade foundation”的头分支。 | PR #2 合并或明确关闭前保留 |

旧 `codex/kujiale-navigation-mapping@478aa656...` 已被 `main` 完整包含，不再提供独立功能。
它与 Module3 发布仓的同名分支还存在不同提交历史、相同 tree 的情况，容易被误认为可部署版本，
因此已退役。

## 当前标签

本历史源仓不发布 standalone baseline，也不保留日常 archive tag。独立 Module3 的固定运行
基线和 commit/tree 去重后的 legacy 功能 tag 均发布在 `AoiOTA/Bio_Nav_Module3` 与
`HDU-ASL/Bio_Nav_Module3`；本仓不再复制这些 tag。

原 `archive/module3-source/.../20260730` 与轻量
`backup_before_remote_cleanup_20260727-164915` 已从日常远端退役。其对象和原 ref 映射仍由
`/home/lyb/Workspace/Bio_Nav_Module_branch_archive/20260730/module3-source-20260730.bundle`
及 SHA256 清单恢复。archive bundle 是恢复介质，不是部署入口。

远端分支不是 campaign 版本记录。每个 campaign 必须记录完整 SHA，并创建
`evidence/<campaign>/input` 和 `evidence/<campaign>/result` annotated tag。PR #2 关闭后，其头
分支需要单独复审再删除。

本工作仓的 `origin` 只允许推送到 `AoiOTA/Isaac_Sim_ROS2_Nav`；AoiOTA/HDU-ASL Module3 必须使用
独立命名 remote。禁止多个 `origin.pushurl`、通配符删除、force-push、历史重写及
`git gc/prune`。完整跨仓库 policy 与 tag object 校验由 Bio_Nav_Integration 集中维护。
