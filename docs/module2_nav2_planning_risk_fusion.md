# Module2 × Nav2 规划与风险融合

## 所有权边界

Module3/Nav2 始终负责 SLAM/定位、地图、障碍合法性、全局与局部规划、MPPI、
Collision Monitor 和 `/cmd_vel`。Module2 只提供可拒绝的认知先验：

- goal-conditioned SR grid 用于等主代价候选的 tie-break；
- Attempt-21 的 `base_link` 32×32、0.5 m/cell `LocalRiskGrid` 由 Module3 在消息
  时间戳按权威 TF 投影为 Global Costmap 非 lethal 软风险；旧 16×16 dynamic cost
  只保留兼容；
- 健康、可靠度、OOD、TTL 和身份 hash。

Module2 不修正位姿、不清除 LiDAR/RGB-D 障碍、不写 Local Costmap、不直接控制
底盘。

## 三个显式 profile

| Profile | 规划 | Global 风险 | Local/控制 |
|---|---|---|---|
| `bio_nav_planning_only` | `BioNavGridBased` | 关闭 | 与传统动态导航一致 |
| `bio_nav_risk_only` | stock planner | 开启 | 与传统动态导航一致 |
| `bio_nav_tiebreak_risk` | `BioNavGridBased` | 开启 | 与传统动态导航一致 |

`stable` 和 `dynamic_avoidance` 不变。融合 profile 文件中的全零 SHA 是
fail-closed 模板，不能直接作为合格运行身份。

## 规划路径

`BioNavGridBased` 通过 `/bio_nav/get_goal_planning_prior` 查询目标条件化冻结 SR，
并订阅 `/bio_nav/module2/planning_prior` 检查当前身份。排序键为：

```text
(stock Smac primary f-cost, -cognitive tie-break score, deterministic serial)
```

认知分数不得改变 primary cost。服务超时、消息过期、goal/map/reset/schema/hash
不一致时，插件调用 stock Smac fallback，而不是用自定义比较器模拟传统路径。

## 风险路径

Attempt-21 使用 `LocalRiskGridLayer`，仍然只挂在 Global Costmap：

1. 检查 map/reset/model/qualification hash、消息年龄、finite、health、
   reliability、OOD 及 `risk_rejection_mask`；任一拒绝位非零即回退；
2. 在消息时间戳把 `base_link` 局部格投影到 global frame；低于冻结阈值的格为
   0，其余映射到 `1..80`；
3. 使用 `max(existing_cost, cognitive_cost)`，因此不能清除真实障碍；
4. 健康输入停止后风险最多线性衰减 `0.8 s`；消息年龄超过 `0.5 s`、
   拒绝位非零或身份失败时在下一周期清空整层。

Local Costmap、MPPI 采样和 Collision Monitor 不订阅 Module2 topic。

六个建图后加入的低矮静态障碍仍由 RGB-D → `depth_voxel_layer` → Local/Global
Costmap 的传统 Module3 链负责。Module2 风险只能增加软代价，不能替代绿色体素、
清除传统障碍或把 cost 写成 lethal。

## RViz 中如何确认“预测”和“应用”

- 黄色/红色 `Local BEV Prediction`：Module2 预测，尚未证明 Nav2 接受；
- 紫色 `Projected Global Risk`：Module3 完成身份、时效、health/OOD 与 TF 门控后
  实际加入 Global Costmap 的软风险；
- 深绿色 `Marked Voxels (3D)`：传统 RGB-D VoxelLayer 的真实静态障碍链；
- 青色 `Motion Belief`/`Motion Peak`：定位诊断，不参与正式定位和控制。

最终以 `/bio_nav/local_risk_layer/status` 与 `/bio_nav/planner/decision` 为准，不能只
根据 RViz 某种颜色宣布融合生效。

## 启动

当前 Attempt-21 静态可视化推荐使用 Integration 的单终端入口；它会统一管理
Module2、Bridge、Nav2、RViz 与自动路线，并绑定对应 evidence/profile 身份：

```bash
bash /home/lyb/Workspace/Bio_Nav/repos/Bio_Nav_Integration/scripts/run_attempt21_static_visual_experiment.sh combined
```

支持 `planning-only`、`risk-only`、`combined`、`static-opt-in` 和 `all`；完整说明见
[v16 单终端手册](https://github.com/AoiOTA/Bio_Nav_Integration/blob/main/docs/module2_nav2_visual_experiment_manual_v16.md)。
该入口只生成工程可视化记录，不会重跑或覆盖冻结的 Development/Gate/Confirmation/
Shadow/A-B evidence。

若仅调试 Module3 侧 profile，可先独立启动 Integration bridge/Module2 服务并确认
身份，再显式选择 profile：

```bash
./scripts/run_ros.sh navigation \
  odometry_mode:=ideal \
  spawn_pose_name:=long_route_start_g1 \
  nav2_profile:=bio_nav_tiebreak_risk
```

正式运行前必须用 `scripts/generate_bionav_fusion_profile.py` 生成绑定真实
map、planning qualification、risk model 和 risk qualification SHA 的参数文件，
并通过 `nav2_profile_params_file:=/absolute/path/generated.yaml` 指定。

工程 smoke 可以确认接口、插件加载和 fallback，不能替代真实路线 Gate。

## 验证当前 worktree 的融合配置

在当前 source worktree 中修改 profile、launch contract 或风险层参数后，不能直接让
`pytest` 从旧的 `ros2_ws/install` 导入 `robot_bringup`；旧 install 可能尚未包含
`bio_nav_*` profile，从而产生与源码无关的假失败。先显式把当前源码放在 Python 搜索路径最前：

```bash
cd /absolute/path/to/Isaac_Sim_ROS2_Nav
PYTHONPATH="$PWD/ros2_ws/src/robot_bringup${PYTHONPATH:+:$PYTHONPATH}" \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python3 -m pytest -q \
  ros2_ws/src/robot_bringup/test/test_bio_nav_fusion_profiles.py \
  ros2_ws/src/robot_bringup/test/test_nav2_profile_contract.py \
  ros2_ws/src/robot_bringup/test/test_mode_contract.py
```

该检查验证 combined profile 只向 Global Costmap 插入 `CognitiveRiskLayer`、保持
Local Costmap 不变，并核验 `BioNavGridBased`、MPPI timing 与 mode contract。通过后仍须
运行 `./scripts/build_ros2.sh`，再以新 install 启动 Isaac；未重建的 install 不能作为当前
源码的运行时验证证据。

Attempt-21 v15 静态补充沿用上述所有权边界，但不复用会删掉 Global
`depth_voxel_layer` 的动态 profile。Integration 的 profile 生成器从已验证静态配置、
冻结 planning template 与 v12 risk overlay 合并，并在启动前检查 combined 同时保留
`depth_voxel_layer` 和 `local_rgbd_risk_layer`。该实验只有在 v13 task-level 10 对
PASS 后才允许运行，结果仍为 engineering diagnostic，不修改 `stable`、
`dynamic_avoidance` 或通用 active authorization。

v15 的静态任务判定沿用用户确认的工程口径：五段全屋导航完成、无卡死/timeout，
且独立 Isaac ContactSensor 未触发即为通过。footprint-vs-box SAT 重叠与间隙大小
完整保存但只用于几何诊断，不单独否决任务。

v16 将通过的静态任务级 evidence 冻结为显式 profile
`nav2_bio_nav_rgbd_risk_static_opt_in.yaml`。该 overlay 保留 Global
`depth_voxel_layer`，仅增加最大 cost 80 的 `local_rgbd_risk_layer`，不修改 Local
Costmap、Collision Monitor、`stable` 或 `dynamic_avoidance`。它不是默认入口，也不
代表动态、多场景或 general active fusion；只有操作者明确传入
`nav2_profile:=bio_nav_rgbd_risk_static_opt_in` 才会选择。

首条 `23601/planning_only` 暴露过一个只影响审计显示的问题：规划器内部已经通过
goal-prior 身份 Gate 并在 131/131 条决策中采用 Module2，但 `PlannerDecision` 错把
base risk identity 的全零 qualification SHA 写入消息。修复后，消息改为发布实际被
采用的 goal-prior qualification、motion-core 和 Module3 map SHA；导航行为、任务结果
和首条原始证据均未改写。判断 planning 是否采用仍应同时查看
`cognitive_tiebreak_used`、`fallback_reason` 与身份字段，不能只看单个 SHA。

当前完整架构、域适配/训练流程与全部阶段指标分别见：

- [Bio_Nav 整体类脑导航系统架构](https://github.com/AoiOTA/Bio_Nav_Integration/blob/main/docs/bio_nav_cognitive_navigation_system_architecture.md)；
- [Attempt-21 静态全部实验结果](https://github.com/AoiOTA/Bio_Nav_Integration/blob/main/docs/attempt21_static_all_experiment_results.md)；
- [Module2 Isaac 域适配](https://github.com/AoiOTA/Bio_Nav_Integration/blob/main/docs/module2_miniworld_to_isaac_domain_adaptation.md)；
- [Module2 训练流水线](https://github.com/AoiOTA/Bio_Nav_Integration/blob/main/docs/module2_training_pipeline.md)。
