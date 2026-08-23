# V6 Phase 1：odom_static 主线定案 + G2→G3 第一错误层诊断（2026-08-23）

> 承接 `V6_RESET_COLD_BOUNDARY_LIVE_20260822.md`（reset 机制 R1–R5 完成）。本文件覆盖 2026-08-22/23 的 Phase 1（estimated 导航闭环）推进弧：9 次 live 迭代 + A/B 定案 + 新地图 + G2→G3 诊断。

## Status（当前状态一句话）

Phase 1 定位主线冻结为 **`odom_static` + Wheel/IMU/EKF**；当前唯一最高优先级 blocker = **G2→G3 doorway**，第一错误层已诊断完毕、修复未执行。

## 用户定案（2026-08-23）

- **A/B 结论**：AMCL 走廊通过 0/3、leg1 0/3；odom_static 走廊 3/3、leg1 2/3（B2/B3 PASS）→ AMCL 是走廊停死的主要放大器，**定位问题阶段性关闭**（不再做定位对比、不再调 AMCL）。
- **冻结**：AMCL off（`V6_LOCALIZATION_BACKEND=amcl` 保留为回滚/对照）；RF2O 继续 off；`isaac_ros_occupancy_grid_localizer` 暂不加（待全路线漂移数据再定，定位是 checkpoint 级而非高频替代）；地图/GVG/Module1/2/IMU 均不改。
- anchor jitter（B 臂重锚 3–6 cm / ~1°）：**warning，不修**（非 demonstrated blocker）。
- 3×20 campaign 政策：按轮重试（每轮最多 3 次尝试）、确实失败才结束、数学不可达提前停；门槛退回 **Attempt31 口径 95%/90%/90%**；室内只做 3×20（不补第四组）。

## 推进弧（cognitive-navigation commits）

**Module3**（`f8782e5` → `d8649e5`）：
- `f8782e5` 低矮障碍 6→1（`v6_low_box_solo` @ map (-1.15,-0.35)，五腿净空 ≥1.6 m）；
- `6f4efef` runner 状态机修复（`_maybe_goal_ready` 越过 GOAL_READY 不回退）+ boundary checker 解包修复；
- `9c358e62` goal/reset-HOLD 竞态修复（等当前 generation gate released 才派发 goal）；`81efbec` nav2/TF 探测轮询化；
- `20100f5`/`71c1d27`/`34e2eb4`/`9b82ea7` **Isaac omap 新地图 `v6_kujiale_isaacgen_v1`**（从实际 USD 的 PhysX 碰撞几何生成 0.05 m 图，幻影唇行消除，scan 实测吻合 ~5 cm）+ GVG 确定性再生（`v6_kujiale_isaacgen_v1:gvg_v1`，48 边全 ≥0.224 m，五腿 route 齐）+ v6 链路引用切换；旧 warehouse_new 保留可回滚；
- `a322f011`/`9fac367a` Phase 1 期间 critic/layer shadow 化（`V6_COGNITIVE_PROFILE=M1`）——根因：module2 把已建图静态家具报为障碍（conf≈1.0、radius ≤1.12 m）致 critic 翻转 MPPI 持续倒车；
- `f7b5c00` `odom_static` 后端（/initialpose 触发初始对齐 + 合成 /amcl_pose，runner readiness 全兼容）；`d8649e5` session 默认 odom_static。
- AMCL 调参 `dcb51ec`/`ffa85b8a`（round-1/2）保留在对照臂配置中，主线不用。

**Integration**（`9c94c829` → `7c905ac5`）：`c72ce867` **F1**（module2 server 运动输入 50× 单位错误删除——运动中 module1 飞出 canvas 的主根因）+ **F2**（obstacle adapter 注入 `minimum_dynamic_presence=0.0`、`lidar_plane_height_m=0.21`）；`c25de204e` 静止重验证窗口（enrollment initialpose 不再关窗）；`b9bcbf2fd` 静止 arming 窗口（healthy prior 确定性获得 arming 资格，不依赖模型随机 trusted_write 位）；`9c94c829` `experiments/dev_initialpose_offset.py`（Phase 4 Module1 A/B 备用，L0/L2 arm）。

**Module2**（`4e38152a`）：置信度与 dynamic_presence 解耦（静态低矮障碍置信度可达 lethal 带）。

## 关键证据（NAS）

- A/B 汇总：`/mnt/nas_home/Bio_Nav_Data/experiments/analysis/v6_ab_g1g2_20260823/`（6 轮 run 目录同前缀 `v6_ab_g1g2_*`）；
- 地图重生成/采用：`.../analysis/v6_kujiale_map_regen_20260823/`；
- margin 判别：`.../analysis/v6_r5_margin_rootcause_20260822/`；G1 出口判别：`.../analysis/v6_g1_exit_rootcause_20260823/`；
- 10 次 Phase 1 live run 目录：`experiments/runs/v6_reset_cold_boundary_r5_phase1_*`（每个含 REVIEWER_NOTE + boundary 复算；**boundary 六不变量已连续 7 次全 PASS**）。

## 当前 blocker：G2→G3 doorway（第一错误层已诊断，证据 `analysis/v6_g2g3_first_error_layer_20260823/`）

共享结构：leg2 canonical route edges [44,31,21,25,27,18,16,13]（11.25 m），图强制唯一过门；门区物理裕度 ±0.12 m、南走廊收窄段规划带 0.16–0.21 m——接近系统已证实噪声水平。

- **B2 = plan 层可行性**：leg2 起步后 Smac 21 次 "no valid path found"（机器人未动）；静态 A* 复算有路、scan 重建豁免 obstacle_layer，排除法指向 **depth_voxel_layer 瞬态 marking** 封闭门带；BT 恢复链只有 Wait 0.7 s、无改变输入的动作。**最小修复候选**：`navigate_route_lookahead.xml:18` compute_path 失败链加 `ClearEntireCostmap`；验证=parked-at-G2 repro + 录 `/global_costmap/costmap` 与 depth points 定死封闭源。
- **B3 = 定位层（odom_static 漂移）**：G2 终点原地旋转（-66°→-160°）引入 +13° yaw 跳变 → leg2 漂移 0.20→0.37 m 超 ±0.12 m 裕度 → 物理切角撞 cabinet_0003 东北角；/plan 可行且被良好跟踪（est-vs-plan 0.03–0.05 m），MPPI 无切角，safety 迟到非因果。**最小修复候选**：`v6_final_kujiale_static.yaml:34` G2 `yaw_deg: -160 → ≈-5`（对齐到达航向，消除原地旋转打滑源）；验证=B 臂同 seed 连跑 2–3 次。
- 纪律：**一次只改一个**，G2→G3 连跑 2–3 次。

## 下一步（按序）

1. B3 修复（G2 yaw）→ B 臂重跑 2–3 次；2. B2 修复（BT ClearEntireCostmap）→ repro 定死封闭源 → 重跑；3. G2→G3 跑通后全路线 G1→…→G5 做 2–3 个完整 pilot（每 leg 定位误差/累计漂移/成功率）→ 据漂移增长曲线决定 Grid Localizer；4. 之后回 Phase 2（Module2 作用验证 M0/M2 A/B）。

## Phase 2 待办清单（已观察、未处理）

- 箱体 0.75 m 内零观测（3 次复现，最近簇偏 1.47 m）；候选区 wrong-region 偏移（新地图后部分改善至 2.5–10.6 m）；障碍 ID churn（count 不累积 → lethal 难达）；prior cadence 饥饿（p90 0.9–1.25 s > 0.5 s TTL，trusted 流 36 s 后断供）；critic 把已建图静态报为障碍（radius ≤1.12 m、当前帧+revalidated 双重计罚）；edge prior 全超时回退 geometry-only；M1 不 gate route 层 prior（契约空洞，当前零影响）；module2 新 graph_id 先验登记。

## Warnings / 技术债

- anchor jitter（warning，不修）；多 episode 同栈 re-arm 未验证（driver 首败即停；Phase 6 编排时加按轮重试）；门区物理裕度本质边缘可行，如需稳健须用户批准调场景家具；`test_rivermark_reference` 等预存失败与本线无关；Rivermark estimated 仅有 build/unit（`run_v6_rivermark.sh`），无 live。
