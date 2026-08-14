# Isaac Sim 6.0.1 + ROS 2 Jazzy：酷家乐 RGB-D 导航

> 当前融合开发状态（2026-07-31）：`feat/planning-risk-fusion-v0.1` 包含可选的
> Module2 规划 tie-break、Global Costmap 软风险层和 RViz 认知叠加。它已通过
> 单元测试与 ROS 2 构建/测试，但不代表真实 Isaac Gate/Confirmation 已通过。
> `stable`、`dynamic_avoidance` 仍是传统导航默认配置；融合 profile 只能显式
> 选择并绑定真实身份 SHA 后使用。

> Attempt-21 当前阶段只验证 Kujiale26 六个建图后加入的静态障碍。冻结 v12 因
> `23505/risk_only` 的 SAT 最大重叠 26.1 mm 按旧 10 mm 规则正确 fail-stop，旧 receipt
> 不回写。用户随后确认采用任务级口径；v13 将 Isaac ContactSensor 与 SAT 分字段：
> 规定全屋路线完成、ContactSensor 未触发且无超时/卡死即通过，SAT 无论数值均保留
> 为只读诊断，不再单独控制结果。v13 最终 baseline/risk-only 均 10/10、ContactSensor
> 与 timeout 均为 0，risk valid coverage 97.76%，工程 task-level 结果为 PASS；它不
> 伪装成 v12 正式 PASS。
> v13 task-level 10 对通过后，Integration 使用全新 23601–23610 完成了 v15 的
> planning-only 与 combined 各 10 条静态诊断：两组均 10/10、ContactSensor 0，规划
> adopted 为 98.67%/97.19%，combined risk valid 为 97.55%。实验全程复用同一 Isaac，
> 未做冷启动；仍保留双 RGB-D VoxelLayer。该结果完成本阶段静态工程任务，但不改写
> 冻结 v12 formal FAIL，也不扩大动态、多场景或通用主动融合授权。v16 现已加入
> `nav2_bio_nav_rgbd_risk_static_opt_in.yaml`：它绑定通过验证的模型/qualification SHA，
> 只在显式选择 `nav2_profile:=bio_nav_rgbd_risk_static_opt_in` 时启用，默认 profile 不变。

## 演示视频（4×速，直接预览）

### 动态避障演示
https://github.com/user-attachments/assets/0fc1c31f-ace7-4b53-a463-b525a2521f4d

### 静态避障演示
https://github.com/user-attachments/assets/39970d48-47df-428b-8d7d-276d2fd7db9d

---

本仓库提供 Clearpath Jackal 在酷家乐室内场景中的 Isaac Sim 导航闭环：2D LiDAR、前向 RGB-D、Nav2、RViz、确定性 Reset，以及可复核的全屋 4×20 鲁棒性实验。

## Attempt31 Rivermark 室外科研 Demo

独立 Attempt31 worktree 已增加 Rivermark 室外原型：保留当前 80 m × 80 m 范围，使用 1600×1600、0.05 m/格的实物对齐栅格、全局 GVG、Route Server、连续认知区域与可选 Module2 edge prior。2026-08-14 核心 static/dynamic-v2/appearance 3×20 fail-closed qualification v3 已 PASS；动态 v1 的 16/20 STOP 仍保留为不可变修复前基线。运行命令、对齐图、正式结果、动态障碍和工程 blockage 注入见 [Attempt31 Rivermark 使用说明](docs/rivermark_outdoor_demo.md)，逐项指标与原 plan 的完成性对照见 [Attempt31 完成性审计](docs/rivermark_completion_audit.md)。该入口不修改 A21 室内默认链路；Module2 四臂因果验证延后到 V4。

当前默认运行组合：

```text
场景:      kujiale_0026_A_to_B_door_open.usd
地图:      warehouse_new
出生点:    long_route_start_g1（map: [0.45, -5.35, 90°]）
定位:      Ideal Odometry；按出生点对齐 map -> odom
导航输入:  /scan + /camera/front/depth/points
```

`warehouse_new` 只支持 Ideal Localization/Navigation；`realistic` 和 `posegraph_calibration:=true` 会被启动契约拒绝。

## 当前正式结果：4×20 外观鲁棒性实验

当前正式实验包含四组、每组20轮：静态基准、静态＋外观变化、动态基准、动态＋外观变化。外观扰动通过匿名 USD Session Layer 改变光照和材质颜色，不修改原始 USD、几何、碰撞、地图或动态障碍运动学。

正式批次 `20260725-210035` 已完成，根报告为 `complete=true`、`passed=true`、`issues=[]`：

| 组别 | 严格成功 | 物理无碰撞 | 结论 |
| --- | ---: | ---: | --- |
| 静态基准 | 20/20 | 20/20 | 最大路径偏差 10.1687% |
| 静态＋外观 | 20/20 | 20/20 | 最大路径偏差 10.1442% |
| 动态基准 | 19/20 | 20/20 | 通过 18/20 门槛 |
| 动态＋外观 | 19/20 | 20/20 | 通过 18/20 门槛 |

结果仅适用于当前冻结的地图、场景、Nav2 profile、外观矩阵、actor 配置和验收规则。完整证据边界见 [验证台账](docs/verification.md)，执行问题与恢复规则见 [4×20 执行复盘](docs/kujiale_4x20_execution_lessons.md)。

对外使用静态/动态避障率、路径偏差和导航成功率时，请采用[通用指标定义与对外表述](docs/kujiale_4x20_metric_definitions.md)。该文档以 LaTeX 给出 \(\mathrm{ASR}_{\mathrm{s}}\)、\(\mathrm{ASR}_{\mathrm{d}}\)、\(\delta_i\) 和 \(\mathrm{NSR}\) 的完整定义、统计边界和适用条件。当前静态避障成功率为 `40/40=100%`、动态避障成功率为 `38/40=95%`、总体导航成功率为 `78/80=97.5%`；理论最优路径偏差仅适用于静态固定障碍参考。

## 首次准备

```bash
cd /your/path/Isaac_Sim_ROS2_Nav
export PROJECT_ROOT="$PWD"
git lfs install
git lfs pull
./scripts/import_assets.sh
export BIO_NAV_INTERFACES_SETUP=/absolute/path/to/Bio_Nav_Integration/ros2_ws/install/setup.bash
./scripts/build_ros2.sh
./scripts/preflight.sh
```

`bio_nav_fusion` 的消息/服务定义由 Integration 仓库的
`bio_nav_interfaces` 唯一维护。先构建 Integration；脚本会自动查找最终统一目录
中的兄弟仓库，也可通过上述环境变量显式指定 underlay。不要把接口源码复制进
Module3。

脚本默认使用 ROS 2 Jazzy、`ROS_DOMAIN_ID=42` 和 `rmw_fastrtps_cpp`。需要直接执行 `ros2` 命令时，先运行：

```bash
source "$PROJECT_ROOT/scripts/setup_ros_env.sh"
```

## 手动导航（Isaac GUI + RViz）

先确认没有另一套项目 Isaac 或 ROS 会话。若提示锁被占用，先执行 `./scripts/diagnose.sh`；不要使用宽泛的 `pkill` 或手动删除锁文件。

终端 A 启动 Isaac GUI：

```bash
cd "$PROJECT_ROOT"
./scripts/run_isaac.sh \
  --environment-usd kujiale_0026_A_to_B_door_open.usd \
  --navigation-mode localization \
  --mode ideal \
  --spawn-pose long_route_start_g1 \
  --camera-profile rgbd_navigation
```

终端 B 启动 Navigation 和 RViz：

```bash
cd "$PROJECT_ROOT"
./scripts/run_ros.sh navigation \
  odometry_mode:=ideal \
  spawn_pose_name:=long_route_start_g1 \
  nav2_profile:=stable
```

等待 `Nav2 lifecycle activation completed` 后，在 RViz 中使用 **2D Goal Pose** 发布目标。完整的 Reset、RGB-D 显示和人工导航步骤见 [用户手册](docs/user_manual.md)。

### Attempt-21 Module2 静态接入：单终端启动

若已按 `/home/lyb/Workspace/Bio_Nav/workspace.env` 配置统一工作区，可在任意目录只用
一个终端启动 Isaac Sim、Module2、Bridge、Nav2、RViz 和自动全屋路线：

```bash
bash /home/lyb/Workspace/Bio_Nav/repos/Bio_Nav_Integration/scripts/run_attempt21_static_visual_experiment.sh combined
```

该入口默认要求并启动 Isaac GUI；若 ROS Domain 中已有 `--headless` Isaac，会明确退出而
不静默复用。使用 `all` 可按 baseline、risk-only、planning-only、combined 运行同一
run-index，并在输出目录生成四臂并排对比。

将末尾模式替换为 `planning-only`、`risk-only`、`static-opt-in`，或使用 `all`
依次观察四条链路。该入口生成的是工程/人工可视化记录，不修改冻结的 Development、
Gate、Confirmation、Shadow 或 A/B 结果。完整模式语义、输出与 Isaac 复用规则见
[用户手册 8.0 节](docs/user_manual.md#80-module2-attempt-21-单终端入口)。

## 正式 4×20 批量运行

一条命令会构建工作区，依次运行静态 pilot＋40轮、生成静态 `2×20` 报告、受控停止静态栈、运行动态 pilot＋40轮、生成动态 `2×20` 报告，最后生成同一批次的总 `4×20` 报告：

```bash
cd "$PROJECT_ROOT"
./scripts/run_kujiale_4x20_all.sh
```

省略批次 ID 时会自动生成 `YYYYMMDD-HHMMSS`。指定 ID 或在相同代码、配置下断点续跑：

```bash
./scripts/run_kujiale_4x20_all.sh 20260726-120000
./scripts/run_kujiale_4x20_all.sh 20260726-120000 --resume --skip-build
```

若只需要在同一配置下续跑动态阶段：

```bash
./scripts/run_kujiale_4x20_all.sh <CAMPAIGN_ID> --dynamic-only --resume --skip-build
```

如果修改了动态代码、Nav2 参数、actor 配置或验收规则，不能对旧动态证据直接使用 `--resume`；应使用新批次 ID 运行 `--dynamic-only`，或严格按 [执行复盘](docs/kujiale_4x20_execution_lessons.md) 替换同一 ID 下的全部动态证据。

报告目录为 `data/reports/kujiale_4x20_<campaign_id>/`。每份报告包含 HTML、PDF、Markdown、PNG、CSV、JSON 和证据索引；报告即使未通过也会生成，退出码 `2` 表示批次完成但未通过门槛或证据不完整。已发布 campaign 的 `index.html` 和 `index_portable.html` 都将地图、统计图、逐轮 GT 路径及五种固定客厅外观示意图引用为 GitHub Raw 链接；复制 HTML 到另一台电脑后只需能访问 GitHub，即可预览、筛选与打开原图，无需携带 `figures/` 目录。未发布图片快照的新报告仍使用相对 PNG 路径。HTML、PDF 和 Markdown 都包含五档外观的灯光强度、色温、材质色相偏移和同一客厅机位示意图；它们说明实验变量，不替代逐轮 RGB 证据。两种 HTML 都将四组实验显示为 S1–D2 中文卡片，明确说明静态/动态障碍与外观变量；地图、统计图、外观图和逐轮轨迹可点击进入当前页放大预览，演示视频则使用明确的“放大视频”按钮进入大尺寸播放器，并均保留在新标签页打开原文件的后备入口。完整4×20报告还内嵌“实验如何执行”和“指标定义与本次结果”：静态/动态避障成功率公式、静态路径偏差公式及静态/动态/总体导航成功率公式。HTML 还可按实验组、seed、外观配置、动态变体和结果筛选每轮路径：静态图叠加六个静态障碍，动态图叠加本轮实际触发 actor 的轨迹、起终点和方向。

完整实验矩阵、外观定义、报告与门槛见 [4×20 运行手册](docs/kujiale_4x20_appearance_benchmark_plan.md)。

## 光照/颜色大图预览（非前向相机）

正式实验保存的 `appearance_rgb_before_goal.ppm` 是 320×180 的前向 RGB-D 证据图，不能用于观察全屋外观。需要核验光照、色温和材质颜色时，使用下面的独立 headless 工具导出固定**客厅观察位**的 **1920×1080 场景视角**，画面以客厅家具、墙面、地面和灯光为主体；它不启动 ROS/Nav2、不运行实验，也不会修改原始 USD 或正式证据。

```bash
cd "$PROJECT_ROOT"
./scripts/capture_kujiale_appearance_preview.sh
```

命令会打印新目录，例如 `data/appearance_previews/kujiale_appearance_<UTC时间>/index.html`。打开该 `index.html`，点击任一图片即可在新页面查看原始分辨率。只看一个配置时：

```bash
./scripts/capture_kujiale_appearance_preview.sh --profile bright_warm
```

截图工具与其他 Isaac 进程互斥；先停止正在运行的 Isaac，再执行。启动时会跳过历史 minidump 上传，避免旧的 Isaac 崩溃转储阻塞本次截图；该图仅用于外观核验，**不是**正式 4×20 统计或运行证据。

## 全屋单轮可视化（不计入正式证据）

两种单轮模式都会自动发送 `G2 → G3 → G4 → G5 → G1`。运行前确保没有其他项目栈；从静态切换到动态时，先按 Ctrl+C 停止静态 ROS，再停止静态 Isaac。

### 静态单轮

```bash
# 终端 A：Isaac GUI + 六个静态 RGB-D 障碍
cd "$PROJECT_ROOT"
./scripts/run_kujiale_4x20_isaac.sh static

# 终端 B：Navigation + RViz
cd "$PROJECT_ROOT"
./scripts/run_ros.sh navigation \
  odometry_mode:=ideal \
  spawn_pose_name:=long_route_start_g1 \
  nav2_profile:=stable

# 终端 C：自动发送全屋航点
cd "$PROJECT_ROOT"
./scripts/run_visual_route.sh static
```

### 动态单轮

```bash
# 终端 A：Isaac GUI + 三阶段动态 actor
cd "$PROJECT_ROOT"
./scripts/run_kujiale_dynamic_isaac.sh

# 终端 B：Navigation + RViz
cd "$PROJECT_ROOT"
./scripts/run_ros.sh navigation \
  odometry_mode:=ideal \
  spawn_pose_name:=long_route_start_g1 \
  nav2_profile:=dynamic_avoidance

# 终端 C：自动发送全屋航点并触发三阶段 actor
cd "$PROJECT_ROOT"
./scripts/run_kujiale_three_stage_visual.sh full --variant 1 --seed 7501
```

动态单轮默认不写正式证据；仅在需要调试记录时，在终端 C 末尾加 `--record`。更详细的观察项、聚焦单段和停止顺序见 [用户手册第8节](docs/user_manual.md#8-可视化单轮全屋长距离测试isaac-gui--rviz)。

## RGB-D 感知边界

`rgbd_navigation` 发布：

```text
/camera/front/image_raw
/camera/front/camera_info
/camera/front/depth/points
```

`stable` profile 在 Local 和 Global Costmap 使用 `depth_voxel_layer`，用于低矮静态障碍；`dynamic_avoidance` 在 Local Costmap 使用时空 STVL，Global Costmap 仅使用静态图和 `/scan`，避免移动 actor 留下全局残影。Collision Monitor 始终只订阅 `/scan`；RGB-D 不参与 SLAM、EKF 或 Collision Monitor。

## 常用操作

| 目标 | 命令 |
| --- | --- |
| 仅启动定位与 TF | `./scripts/run_ros.sh localization odometry_mode:=ideal` |
| 从零建图 | Isaac：`--navigation-mode mapping --mode ideal`；ROS：`./scripts/run_ros.sh mapping odometry_mode:=ideal` |
| 保存新地图 | `./scripts/save_map.sh <新版本名>`，完成标定后才能使用 `initial_pose_source:=auto` |
| 查看受管进程和锁 | `./scripts/diagnose.sh` |
| 安全预览清理 | `./scripts/clean_runtime.sh --dry-run` |

## 文档入口

| 目标 | 文档 |
| --- | --- |
| 执行正式实验、理解外观矩阵与报告 | [4×20 运行手册](docs/kujiale_4x20_appearance_benchmark_plan.md) |
| 排查 pilot、supervisor、续跑或动态复测 | [4×20 执行复盘](docs/kujiale_4x20_execution_lessons.md) |
| 查看地图、航点、静态障碍和动态 actor 路线 | [全屋路线地图](docs/kujiale_long_route_map.md) |
| 查看运行契约、Topic、TF、QoS 和所有权 | [接口文档](docs/interfaces.md) |
| 诊断启动、锁、DDS、RViz、Reset 或 Nav2 问题 | [排障手册](docs/troubleshooting.md) |
| 查看正式结果、配置边界与复核方法 | [验证台账](docs/verification.md) |
| 查看当前文档的事实来源与职责 | [文档状态](docs/documentation_status.md) |
| 查看 Module3 原始源仓、两个发布仓、分支、标签和本地 remote 的一一对应关系 | [分支与标签目录](docs/branch_governance.md) |
| 理解 Module2 如何接入 Nav2、规划与风险如何融合 | [Module2 × Nav2 规划/风险融合](docs/module2_nav2_planning_risk_fusion.md) |
| 操作 RViz 中的 Module2 认知叠加 | [Module2 RViz 可视化](docs/module2_rviz_visualization.md) |
| 查看 Attempt-21 静态接入、RViz 与量化结果 | [Module2 × Nav2 静态图文报告](docs/reports/attempt21-static-v13/index.html) |
| 查看 Combined Isaac Sim + RViz 演示 | [公开报告与内嵌视频](https://aoiota.github.io/Isaac_Sim_ROS2_Nav/#module2-combined-demo) |
| 从全局理解 Module1/Module2/Integration/Module3 架构 | [Module2 × Nav2 规划/风险融合](docs/module2_nav2_planning_risk_fusion.md) |
| 汇总 Development→Gate→Confirmation→Shadow→A/B→Combined 指标 | [公开静态图文报告](https://aoiota.github.io/Isaac_Sim_ROS2_Nav/#metrics) |
| 单终端运行 planning/risk/combined/static opt-in | [用户手册 8.0 节](docs/user_manual.md#80-module2-attempt-21-单终端入口) |
| 复现 v0.1 工程验证 | [规划/风险融合 v0.1 复现](docs/reproduction/planning-risk-fusion-v0.1.md) |

## 验证

```bash
./scripts/test.sh
./scripts/test.sh --with-isaac
git diff --check
```

运行 ROS 集成测试前，停止同一 ROS Domain 42 中的 Isaac 仿真，避免真实 `/clock` 与测试夹具冲突。
