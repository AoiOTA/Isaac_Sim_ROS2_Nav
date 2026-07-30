# 历史源仓分支治理

长期保留 `main`、`codex/kujiale-navigation-mapping`，以及开放 PR #2 的
`codex/navigation-quality-fidelity`。PR #2 合并或明确关闭前，后者不得进入删除清单。

其余 WebRTC、RViz、runtime、debug 与 G2 历史线先由 `archive/module3-source/.../20260730`
annotated tag 和 `/home/lyb/Workspace/Bio_Nav_Module_branch_archive/20260730/` bundle 保存，之后才可
在第二次明确确认的逐项 allowlist 中删除。远端分支不是 campaign 版本记录：每个 campaign 必须
记录完整 SHA，并创建 `evidence/<campaign>/input` 和 `evidence/<campaign>/result` annotated tag。

本工作仓的 `origin` 只允许推送到 `AoiOTA/Isaac_Sim_ROS2_Nav`；AoiOTA/HDU-ASL Module3 必须使用
独立命名 remote。禁止多个 `origin.pushurl`、通配符删除、force-push、历史重写及 `git gc/prune`。
完整跨仓库 policy 与 tag object 校验由 Bio_Nav_Integration 集中维护。
