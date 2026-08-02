# 酷家乐 RGB-D 导航使用手册

> 当前可执行手册，适用分支：`main`。
>
> 本手册只描述当前酷家乐 `warehouse_new`、Ideal Odometry 和4×20流程。旧候选批次、旧动态矩阵和旧布局调参命令已不再作为仓库操作入口提供；当前文档职责见 [文档状态](documentation_status.md)。

## 1. 当前运行组合

| 项目 | 当前值 |
| --- | --- |
| 场景 | `kujiale_0026_A_to_B_door_open.usd` |
| 地图 | `warehouse_new`，`154 × 248 @ 0.05 m` |
| 出生点 | `long_route_start_g1`，map `[0.45, -5.35, 90°]` |
| 里程计 | Isaac Ideal `/odom` 与 `odom -> base_link` |
| 定位 TF | 已标定、按出生点对齐的 `map -> odom` |
| 导航输入 | 地图链 `/scan`；近场安全链 `/scan_safety`；RGB-D `/camera/front/depth/points` |
| ROS | Jazzy、Domain `42`、`rmw_fastrtps_cpp` |

`warehouse_new` 只支持 Ideal Localization/Navigation。`realistic`、`posegraph_calibration:=true` 或运行中热切换出生点都会被启动契约拒绝或造成不可复核状态。

## 2. 首次准备

```bash
cd /你的实际路径/Isaac_Sim_ROS2_Nav
export PROJECT_ROOT="$PWD"
git lfs install
git lfs pull
./scripts/import_assets.sh
./scripts/build_ros2.sh
./scripts/preflight.sh
```

预检成功时应出现：

```text
map manifest verified: warehouse_new bundle=<SHA256>
preflight: PASS
```

需要直接使用 ROS CLI 时：

```bash
source "$PROJECT_ROOT/scripts/setup_ros_env.sh"
```

所有终端必须使用同一 ROS Domain 和 RMW：

```text
ROS_DOMAIN_ID=42
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

## 3. 手动导航（Isaac GUI + RViz）

### 3.1 启动前检查

```bash
cd "$PROJECT_ROOT"
./scripts/diagnose.sh
./scripts/clean_runtime.sh --dry-run
```

若已有受管 Isaac、ROS、RViz 或 Teleop 会话，不要再启动第二套。先复用同配置会话，或按第10节停止。不要使用 `pkill`、`kill -9`、删除锁文件或清空 `/dev/shm`。

### 3.2 终端 A：启动 Isaac GUI

```bash
cd "$PROJECT_ROOT"
./scripts/run_isaac.sh \
  --environment-usd kujiale_0026_A_to_B_door_open.usd \
  --navigation-mode localization \
  --mode ideal \
  --spawn-pose long_route_start_g1 \
  --camera-profile rgbd_navigation
```

等待日志出现 `Isaac navigation simulation ready` 与 `spawn=long_route_start_g1`。若日志显示 `mapping_start`，停止当前 Isaac 后重新执行完整命令；出生点不能在进程运行中切换。

### 3.3 终端 B：启动 Navigation 与 RViz

```bash
cd "$PROJECT_ROOT"
./scripts/run_ros.sh navigation \
  odometry_mode:=ideal \
  spawn_pose_name:=long_route_start_g1 \
  nav2_profile:=stable
```

等待：

```text
Nav2 lifecycle activation completed
```

该命令会受管启动 `navigation.rviz`；不要再单独启动 `rviz2` 或 `run_rviz.sh`。

### 3.4 在 RViz 发送目标

1. Fixed Frame 设为 `map`。
2. 确认 **Navigation 2 Safe** 面板显示已激活。
3. 选择 **2D Goal Pose**，在可通行区域拖出目标和朝向。
4. 观察全局/局部路径、MPPI 最优轨迹、Costmap 和 Collision Monitor。
5. 等待面板显示成功且机器人静止。

人工导航使用 RViz 标准 `SetGoal` 与 Nav2 `goal_pose`，没有私有目标桥，也不需要第三个终端。

## 4. RGB-D 感知与当前边界

`rgbd_navigation` 会发布：

```text
/camera/front/image_raw
/camera/front/camera_info
/camera/front/depth/points
```

在 `navigation.rviz` 中展开 **RGB-D Fusion**：

- **Robot Front Camera**：前视 RGB。
- **Depth PointCloud2**：深度点云。
- **Marked Voxels (3D)**：`stable` profile 的 Nav2 VoxelLayer。
- **Temporal Voxels (3D)**：`dynamic_avoidance` profile 的 Local STVL。

`Marked Voxels (3D)` 的深绿色小方块就是相机深度点云经过 VoxelLayer 后标记的
三维占用。它们用于六个建图后加入的低矮静态障碍，和 Module2 黄色/红色预测格
不是同一层。Module2 预测只有在通过门控并显示为紫色 `Projected Global Risk`
后，才表示其软风险已进入 Global Costmap；青色 Motion Belief/Peak 仅作定位诊断。

Attempt-21 v13 的任务级接触证据会把 Isaac ContactSensor 和 50 Hz SAT 几何审计
分开保存：前者与路线/超时共同控制通过，后者只作贴边/穿入诊断。不要再从单一
`physical_collision_free` 字段反推碰撞来源，也不要把 SAT 诊断值隐藏或改写。

`stable` 在 Local 和 Global Costmap 使用 `depth_voxel_layer`，用于低矮静态障碍。`dynamic_avoidance` 用 Local STVL 替代该层，并让 Global Costmap 只使用静态地图和 `/scan`，避免移动 actor 留下全局残影。两套 profile 的 Local Costmap 与 Collision Monitor 消费独立的 `/scan_safety`；SLAM、定位和 Global Costmap 仍消费原 `/scan`。RGB-D 不进入 SLAM、EKF 或里程计。

Navigation 模式会将 `/lidar/points_raw` 变换到 `base_link`，删除物理 footprint 加
`5 mm` padding 内的自体点后发布 `/lidar/points_scan`，再投影为
`range_min=0.05 m` 的 `/scan_safety`。原 `/scan` 的 frame、频率、投影范围和消费者
不变。若安全点云 TF 不可用或数据格式无效，自滤波节点丢弃该帧，不以未过滤点云降级。

快速检查：

```bash
source "$PROJECT_ROOT/scripts/setup_ros_env.sh"
ros2 topic info /camera/front/depth/points --verbose
ros2 topic hz /camera/front/depth/points
ros2 topic echo /local_costmap/voxel_grid --once
ros2 topic hz /scan_safety
ros2 topic echo /scan_safety --once
```

若只看到深度点云但没有体素，先检查相机 profile、相机 TF、点云 QoS 和局部 Costmap；动态 profile 应查看 `Temporal Voxels (3D)`，不要将其与标准 `VoxelGrid` 混用。

## 5. 初始位姿、Reset 与仅定位查看

默认 `initial_pose_source:=auto` 根据 `warehouse_new` Manifest 与出生点标定自动发布初始位姿。每次 Reset 后，系统会等待新鲜的 `/clock`、`/scan`、`/odom` 和稳定 `map -> odom`，再恢复 Nav2；不要在恢复前发送 Goal。

开发未标定的新地图时，才使用人工初始位姿：

```bash
./scripts/run_ros.sh navigation \
  odometry_mode:=ideal \
  initial_pose_source:=rviz
```

随后在 RViz 用 **2D Pose Estimate** 播种位姿；每次 Reset 后都要重新播种。

仅查看定位、地图和 TF：

```bash
# 终端 A 仍按第3.2节启动 Isaac
./scripts/run_ros.sh localization odometry_mode:=ideal
```

仅查看相机：

```bash
# 终端 B
./scripts/run_ros.sh localization odometry_mode:=ideal use_rviz:=false

# 终端 C
./scripts/run_camera_view.sh
```

## 6. 建图新版本

建图不能与 Localization/Navigation 并行。只有修改场景、传感器、机器人或确实需要新地图时才执行：

```bash
# 终端 A
./scripts/run_isaac.sh \
  --environment-usd kujiale_0026_A_to_B_door_open.usd \
  --navigation-mode mapping \
  --mode ideal

# 终端 B：会启动 Mapping RViz 与受管键盘 Teleop
./scripts/run_ros.sh mapping odometry_mode:=ideal
```

Teleop 只属于 Mapping：W/A/S/D 或方向键控制，松键后 `0.18 s` 自动停车，Space 立即停车，Q 退出。保存地图：

```bash
./scripts/save_map.sh <新版本名>
```

该命令生成 OccupancyGrid、Pose Graph 与 Manifest 四件套。新 Manifest 初始未标定，完成 Map/USD 标定、Manifest 更新与冷启动复核前，不能用于 `initial_pose_source:=auto` 或正式实验。详见 [标定手册](calibration.md)。

## 7. 正式4×20批量实验与报告

### 7.1 当前结果与适用范围

正式批次 `20260725-210035` 已完成并通过：静态基准、静态＋外观均为 `20/20` 严格成功；动态基准、动态＋外观均为 `19/20` 严格成功；四组物理无碰撞均为 `20/20`。结果仅适用于当前冻结的地图、actor、Nav2 profile、外观矩阵和验收规则，详见 [验证台账](verification.md)。

四组均从 G1 出发，自动发送 `G2 → G3 → G4 → G5 → G1`：

| 组别 | 场景 | Nav2 profile | 轮数 | 门槛 |
| --- | --- | --- | ---: | --- |
| 静态基准 | 六个低矮 RGB-D 障碍 | `stable` | 20 | 严格成功且无碰撞 ≥19/20 |
| 静态＋外观 | 相同静态几何、四种外观 | `stable` | 20 | 严格成功且无碰撞 ≥19/20 |
| 动态基准 | 三阶段 `full_route_three_stage` | `dynamic_avoidance` | 20 | 严格成功且无碰撞 ≥18/20 |
| 动态＋外观 | 相同动态运动学、四种外观 | `dynamic_avoidance` | 20 | 严格成功且无碰撞 ≥18/20 |

### 7.2 一条命令运行全部内容

该命令会自动构建、运行静态 pilot 和40轮、生成静态 `2×20` 报告、关闭静态栈、运行动态 pilot 和40轮、生成动态 `2×20` 报告，最后生成同一批次总 `4×20` 报告：

```bash
cd "$PROJECT_ROOT"
./scripts/run_kujiale_4x20_all.sh
```

省略 ID 时自动生成 `YYYYMMDD-HHMMSS`。要指定 ID：

```bash
./scripts/run_kujiale_4x20_all.sh 20260726-120000
```

中断后，仅在代码、地图、actor、Nav2 参数与验收规则都未变化时使用同一 ID 续跑：

```bash
./scripts/run_kujiale_4x20_all.sh <CAMPAIGN_ID> --resume --skip-build
```

`pilot` 是正式40轮的前置检查。失败或不完整 pilot 会被隔离为 `.incomplete-<UTC>`，恢复时重新执行；完整且校验通过的正式轮会跳过。

### 7.3 仅续跑或重做动态两组

静态已完成、且只是同配置的动态阶段被中断时：

```bash
./scripts/run_kujiale_4x20_all.sh <CAMPAIGN_ID> \
  --dynamic-only --resume --skip-build
```

如果修改了动态代码、Nav2 参数、actor 配置或验收规则，不能把旧动态证据与新配置混用。应使用新批次 ID：

```bash
./scripts/run_kujiale_4x20_all.sh --dynamic-only --skip-build
```

需要保留同一 ID 下的静态40轮、从零替换全部动态证据时，不要只删除失败轮；严格按 [4×20执行复盘](kujiale_4x20_execution_lessons.md) 的保留/移出清单操作。

### 7.4 报告、状态与退出码

报告写入：

```text
data/reports/kujiale_4x20_<campaign_id>/
├── static_2x20/
├── dynamic_2x20/
├── index.html
├── report.pdf
├── report.md
├── benchmark.json
├── benchmark.csv
├── evidence_index.json
└── figures/
```

静态阶段完成后会立即生成 `static_2x20/`；动态失败不会丢失该报告。报告命令即使未达到门槛也会生成完整输出：退出码 `0` 表示报告范围通过，`2` 表示报告已生成但门槛或证据不完整，其他非零值才表示生成错误。

仅重绘既有报告、不重新运行 Isaac：

```bash
./scripts/run_kujiale_4x20.sh static-report  <CAMPAIGN_ID> --replace
./scripts/run_kujiale_4x20.sh dynamic-report <CAMPAIGN_ID> --replace
./scripts/run_kujiale_4x20.sh report         <CAMPAIGN_ID> --replace
```

HTML 报告在分组摘要后首先展示“实验设计与指标口径”：包含固定地图/路线、四组20轮、静态六障碍、三阶段 actor、外观扰动与重启隔离方法。静态/动态避障成功率、路径偏差和导航成功率均以 LaTeX 渲染公式，并逐项说明评价对象、成功判定、分子分母、统计范围与适用边界；相同内容同步写入支持 LaTeX 的 `report.md` 和带数学排版的 `report.pdf` 首页。图片快照已发布的报告会让 `index.html` 与 `index_portable.html` 统一使用 GitHub Raw 链接：向另一台电脑只需传递 HTML，接收方联网访问 GitHub 即可查看地图、统计图和所有可筛选的逐轮 GT 轨迹，并可打开原图。未发布快照时报告明确提示携带 `figures/` 目录。随后提供对应的避障演示视频：静态子报告播放静态演示，动态子报告播放动态演示，完整4×20报告同时提供两段视频。视频优先使用指定的 GitHub attachment；播放器还配置了仓库内同一 MP4 的原始文件作为回退，因此 attachment 暂不可达时仍可在页面内直接播放。

测试地图不再把四组缩在一张图内：完整报告分为“静态两组对比”和“动态两组对比”，每次只并列两张；静态/动态 2×20 子报告只显示本范围的一对地图。地图、统计图和逐轮 GT 路径均可点击，在当前页弹窗放大；避障演示视频提供“放大视频”按钮，在当前页的大尺寸播放器中播放。所有媒体同时保留新标签页打开原文件的后备入口。已发布快照的 HTML 使用可直接打开的 GitHub Raw 图片链接，不再生成大型 Base64 内容。报告首页的 S1–D2 中文卡片同时写明每组控制变量：静态/动态障碍类型、外观是否改变，以及每组20轮的具体分配。

“逐轮实际 GT 路径”位于测试地图之后、统计图之前，可按实验组、seed、外观 profile 和结果筛选；点击路径图也能查看原尺寸。静态图显示六个静态障碍；动态图显示本轮实际触发 actor 的轨迹、起终点和运动方向。总报告的“光照/颜色配置与客厅示意图”位于实验设计之后，统一给出五档外观的亮度倍率、色温、材质色相偏移与可点击放大的客厅视角；HTML、便携 HTML、Markdown 和 PDF 均包含该说明，图示不替代每轮 RGB 证据。完整矩阵、外观配置和验收口径见 [4×20运行手册](kujiale_4x20_appearance_benchmark_plan.md)。

如需将结果写入交付材料，请使用[通用指标定义与对外表述](kujiale_4x20_metric_definitions.md)。碰撞、任务完成和动态交互是本实验对公式变量的测量实现，不应被写进通用指标公式本身。

## 8. 可视化单轮全屋长距离测试（Isaac GUI + RViz）

单轮 visual runner 自动发送 `G2 → G3 → G4 → G5 → G1`，不需要在 RViz 手动发布目标。这些命令仅用于观察 RGB-D、Costmap、MPPI 与障碍行为；不生成正式80轮证据，也不能替代正式结论。

每次只运行一套 Isaac 和 ROS。静态切换到动态前，先在静态 ROS 终端按 Ctrl+C，等待有序关闭后再停止静态 Isaac。

### 8.1 静态单轮

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

在 GUI 观察低矮障碍与绕行；在 RViz 观察 **Robot Front Camera**、**Depth PointCloud2**、**Marked Voxels (3D)**、路径、Costmap 与 MPPI 最优轨迹。

### 8.2 动态单轮

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

动态 actor 会在 G1→G2、G2→G3、G5→G1 的对应空间 gate 后依次出现、运动、停车，并在到达对应下一航点后退役。需要临时调试证据时，在终端 C 命令末尾加 `--record`；该记录不计入正式统计。

聚焦单段 `g2-g3` 或 `g5-g1` 时，Isaac 和 Navigation 必须一起改用对应的 `long_route_start_g2` 或 `long_route_start_g5`；当前 actor 坐标、gate 与全屋/单段诊断边界见 [4×20运行手册](kujiale_4x20_appearance_benchmark_plan.md) 和 [全屋路线地图](kujiale_long_route_map.md)。

### 8.3 光照/颜色高分辨率客厅场景预览

正式 campaign 的 `appearance_rgb_before_goal.ppm` 是为导航可复核性保留的 320×180 前向 RGB-D 快照，不适合目视比较全屋外观。下面的独立截图工具使用同一份五配置外观 YAML，在固定**客厅观察位**导出 **1920×1080 的场景视角**，以客厅家具、墙面、地面和灯光为主体；不是前向相机，也不启动 ROS、RViz、Nav2 或动态 actor：

```bash
cd "$PROJECT_ROOT"
./scripts/capture_kujiale_appearance_preview.sh
```

输出目录会自动生成在 `data/appearance_previews/kujiale_appearance_<UTC时间>/`，其中 `index.html` 是可点击放大原图的本地图库，`preview_manifest.json` 记录场景、客厅相机坐标、分辨率、外观配置哈希和各 profile 的 Session Layer 状态。只导出指定配置：

```bash
./scripts/capture_kujiale_appearance_preview.sh --profile dim_warm
```

可用 `--width`、`--height` 调整分辨率（范围 320–3840），例如 `--width 2560 --height 1440`。它与 Isaac 单实例锁互斥：先停止已有 Isaac GUI 或正式批次，再截图。启动器会跳过历史 minidump 上传，避免旧 Isaac 崩溃转储阻塞本次截图；预览文件不写入 `data/experiment_runs/`，不应作为正式 4×20 统计证据。

## 9. 常用诊断

| 症状 | 首先执行 |
| --- | --- |
| 启动前失败、地图或 Manifest 错误 | `./scripts/preflight.sh`，再看 [排障手册](troubleshooting.md) |
| 已有实例或锁 | `./scripts/diagnose.sh`，再执行 `./scripts/clean_runtime.sh --dry-run` |
| 看不到 Isaac Topic | `source "$PROJECT_ROOT/scripts/setup_ros_env.sh"`，检查 `/clock` 与 `/lidar/points_raw` |
| Nav2 不激活 | 等待 `/map`、`/clock`、`/scan`、`/odom` 和稳定 `map -> odom`；不要提前发 Goal |
| 没有深度/体素 | 确认 `rgbd_navigation`、点云 Topic、相机 TF 和对应 profile 的 voxel 显示 |
| 批量 pilot 或续跑失败 | 查看 `data/experiment_runs/kujiale_4x20_<ID>/orchestrator/`，再看 [4×20执行复盘](kujiale_4x20_execution_lessons.md) |

Topic、TF、QoS 和唯一发布者要求见 [接口文档](interfaces.md)。

## 10. 正常停止、验证与提交

常规 GUI 会话：先在 ROS/RViz 终端按一次 Ctrl+C，等待 Nav2 有序关闭；再在 Isaac 终端按一次 Ctrl+C。异常退出时，先运行 `diagnose.sh`，只根据 `clean_runtime.sh --dry-run` 的已验证对象清理。

代码或配置变更后：

```bash
git diff --check
./scripts/test.sh
```

包含 Isaac/USD 行为的改动再执行：

```bash
./scripts/test.sh --with-isaac
```

ROS 集成测试前停止真实 Isaac 会话，避免真实 `/clock` 与测试夹具冲突。不要提交 `data/reports/`、`data/experiment_runs/`、rosbag、MCAP 或本地 Isaac 资产。
