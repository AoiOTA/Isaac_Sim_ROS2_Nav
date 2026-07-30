# Module3 分支与标签目录

Module3 有两个日常远端：AoiOTA 是主仓，HDU-ASL 是镜像并额外保存两条学长维护的 Windows
发布线。除明确列出的所有者分支外，同名长期引用必须具有相同 SHA。

## 长期分支

| 分支 | 远端 | 用途 | 是否默认入口 |
| --- | --- | --- | --- |
| `main` | AoiOTA、HDU-ASL | 当前 Linux / Isaac Sim / ROS 2 / Nav2 独立 Module3 主线；两个远端必须同 SHA。 | 是 |
| `integration/stage2.2-shadow` | AoiOTA、HDU-ASL | Module2 × Module3 Stage 2.2 Shadow 的固定兼容输入，tip 为 `5da2f2fc27b7e139b2e852acefbea0bdc0b01228`。 | 否 |
| `codex/windows-isaacsim-zenoh` | 仅 HDU-ASL | 学长发布的较早 Windows / Zenoh 兼容线，tip 为 `8f7c4768...`。 | 否；所有者保护 |
| `codex/lc-windows-isaacsim-v1` | 仅 HDU-ASL | 学长发布的较新 Windows 稳定化线，tip 为 `328991b9...`。 | 否；所有者保护 |

两条 Windows 分支保留原发布名称，以免破坏学长已有的检出链接和协作约定；它们不参与
AoiOTA/HDU-ASL 的 Linux 镜像一致性检查，未经所有者批准不得删除、移动、改写或自动合并。

旧 `codex/kujiale-navigation-mapping` 不再是长期分支。清理前 AoiOTA 与 HDU-ASL 的 tip
分别为 `caae0c08...` 和 `478aa656...`：提交历史 SHA 不同，但 tree 都是
`ca08a923...`，并且两条历史均已由共同 `main` 包含。继续保留同名不同 SHA 的分支只会制造
“镜像失配”，因此已退役。

## 长期标签

| annotated tag | 远端 | 用途 | 是否用于日常测试 |
| --- | --- | --- | --- |
| `baseline/module3/standalone-navigation/20260730` | AoiOTA、HDU-ASL | 固定本轮整理后的独立 Module3 导航基线；两个远端的 tag object 与 peeled commit 必须一致。 | 需要精确复现时使用 |
| `evidence/stage2.2-shadow/input` | AoiOTA、HDU-ASL | 将跨模块 Shadow 输入固定到 `5da2f2f...`；不表示 Confirmation、主动融合或导航资格通过。 | 否 |
| `legacy/module3/jackal-physics-contact-ab/20260730` | AoiOTA、HDU-ASL | 固定 tree `2246428a...`：旧 Jackal mass/collision profiles、runtime provenance 与 contact A/B 工具。 | 仅维护该旧功能 |
| `legacy/module3/runtime-camera-warehouse-v2/20260730` | AoiOTA、HDU-ASL | 固定 tree `4846c140...`：旧 runtime/camera 工作流及 `warehouse_v2` 地图 bundle。 | 仅维护该旧功能 |
| `legacy/module3/webrtc-streaming/20260730` | AoiOTA、HDU-ASL | 固定 tree `cd25caae...`：包含当前 `main` 不具备的 `scripts/run_isaac_streaming.sh`。 | 仅维护 WebRTC |

原来的 8 个 `archive/module3/{aoiota,hdu-asl}/.../20260730` tag 中，同功能的 Aoi/HDU
提交具有不同 commit SHA、但成对具有相同 tree。上表三个 `legacy/...` tag 各保留一个唯一
功能 tree，并在 annotation 中记录两侧原 SHA。旧 RViz tree `4ba9e21d...` 只有
`navigation.rviz` 的旧布局，而当前 `main` 已有后续版本，因此不再保留日常 tag。

全部原 tag object 和提交仍保存在下列经 SHA256 与 `git bundle verify` 核验的离线包中：

- `Bio_Nav_Module_branch_archive/20260730/module3-aoi-20260730.bundle`
- `Bio_Nav_Module_branch_archive/20260730/module3-hdu-20260730.bundle`

## 如何选择

- 只运行或测试 Module3：跟踪 `main`。
- 需要复现 2026-07-30 的独立导航基线：检出
  `baseline/module3/standalone-navigation/20260730` 的 detached HEAD。
- 只有明确需要旧 WebRTC、Jackal 物理/contact A/B 或 warehouse_v2/runtime-camera 时，才检出
  对应 `legacy/...` tag；legacy 不继承当前主线结论。
- 只有在复现登记的 Integration Shadow 输入时，才使用
  `integration/stage2.2-shadow` 并核验 `evidence/stage2.2-shadow/input`。
- Windows 工作由对应所有者分支承担；不要将其当作 Linux 主线或自动镜像。
- 新工作从 `main` 创建短期 `feat/`、`fix/` 或 `codex/` 分支，合并后删除。

不得用永久版本分支记录 campaign。每个 campaign 必须记录完整 SHA，并创建
`evidence/<campaign>/input` 与 `evidence/<campaign>/result` annotated tag。完整跨仓库 policy、
所有者例外和日常 SOP 由 Bio_Nav_Integration 集中维护。禁止通配符删除、历史重写、
force-push，以及以分支清理为名删除 LFS 或实验数据。
