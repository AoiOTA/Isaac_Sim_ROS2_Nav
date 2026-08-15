# 已发布报告图片快照

此目录保存需要通过 GitHub Raw 链接跨电脑访问的报告图片和精选演示视频。原始运行证据、
MCAP、CSV 和本地报告目录仍位于 Git 忽略的 `data/`，不会因发布媒体而进入仓库。

目录名必须与报告输出目录名一致，例如：

```text
docs/report_assets/kujiale_4x20_20260725-210035/figures/
```

当 `kujiale_4x20_campaign.py` 在生成或重绘报告时发现该目录包含与报告 `figures/` 完全对应的
PNG，它会让 `index.html`、`index_portable.html` 和 `report.md` 使用 AoiOTA 主仓库的 GitHub Raw 链接。
这样复制 HTML 到另一台电脑后，只需网络访问 GitHub 即可显示图片、筛选逐轮路径并打开原图。

新 campaign 需要先完成结果审阅，再有意复制其 `figures/` 到本目录、提交并推送；未发布快照的
报告会安全地保留相对 PNG 路径，不会产生失效外链。

Attempt-21 Module2/Nav2 的公开演示媒体位于
`attempt21_module2_nav2_effect/`：

- `combined_isaac_rviz.png`：Isaac Sim 与 RViz Combined 联合截图，2493×1406；
- `combined_navigation_demo.mp4`：H.264/AAC、960×544、约 33.73 秒。

报告使用绑定资产提交 `29c1ec5fff94fd373d8b88c521544988c689f7e5` 的不可变 GitHub
Raw 链接，避免分支后续更新使已发送的 HTML 失去对应媒体。
