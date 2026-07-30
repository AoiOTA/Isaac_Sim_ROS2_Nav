# Module3 分支治理

当前长期引用为 `main`、`integration/stage2.2-shadow` 和
`codex/kujiale-navigation-mapping`。Shadow 固定在
`5da2f2fc27b7e139b2e852acefbea0bdc0b01228`，并由 annotated tag
`evidence/stage2.2-shadow/input` 固化；它不授权运行实验、接入控制链或宣称导航资格通过。

双远端 `main` 在完成逐文件差异审查并取得所有者批准前不是镜像，禁止自动同步、快进、
force-push 或跨远端合并。AoiOTA 与 HDU-ASL 的 evidence tag 必须同时验证 tag object 与 peeled
commit；任一缺失、轻量、移动或不一致均 fail-closed。

WebRTC、RViz、runtime、debug 与 G2 等短期历史线先以 `archive/module3/.../20260730` annotated
tag、Git bundle、LFS OID 与恢复命令归档；再经过 PR/worktree/硬编码引用复查、显式 allowlist 和
第二次明确确认后才能删除。不得用版本分支记录 campaign：必须记录完整 SHA，并创建
`evidence/<campaign>/input` 和 `evidence/<campaign>/result` annotated tag。

完整跨仓库 policy、所有者例外和日常 SOP 由 Bio_Nav_Integration 集中维护。禁止通配符删除、
历史重写、force-push，以及以分支清理为名删除 LFS 或实验数据。
