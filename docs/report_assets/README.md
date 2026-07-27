# 已发布报告图片快照

此目录只保存需要通过 GitHub Raw 链接跨电脑访问的正式报告 PNG。原始运行证据、MCAP、
CSV 和本地报告目录仍位于 Git 忽略的 `data/`，不会因发布图片快照而进入仓库。

目录名必须与报告输出目录名一致，例如：

```text
docs/report_assets/kujiale_4x20_20260725-210035/figures/
```

当 `kujiale_4x20_campaign.py` 在生成或重绘报告时发现该目录包含与报告 `figures/` 完全对应的
PNG，它会让 `index.html`、`index_portable.html` 和 `report.md` 使用 AoiOTA 主仓库的 GitHub Raw 链接。
这样复制 HTML 到另一台电脑后，只需网络访问 GitHub 即可显示图片、筛选逐轮路径并打开原图。

新 campaign 需要先完成结果审阅，再有意复制其 `figures/` 到本目录、提交并推送；未发布快照的
报告会安全地保留相对 PNG 路径，不会产生失效外链。
