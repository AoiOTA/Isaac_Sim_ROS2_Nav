# Kujiale 静态/动态避障问题记录与当前运行基线

本文记录 `warehouse_new` 全屋路线在动态避障重新设计、可视化调试与静态回归期间出现过的问题、根因、处理方式和当前可复现实验入口。它是运行与调优记录，不替代最终人工验收。

## 1. 当前冻结的运行分层

静态和动态不再强行共用 MPPI 控制参数；二者共享地图、TF、车体 footprint、里程计所有权和启动链路，但使用独立 Nav2 profile。

| 项目 | 静态 `stable` | 动态 `dynamic_avoidance` |
|---|---|---|
| 控制频率 | 10 Hz | 15 Hz |
| 预测窗 | `20 × 0.10 s = 2.0 s` | `30 × 1/15 s = 2.0 s` |
| MPPI 样本 | 700 | 500 |
| 速度/转向包络 | `0.75 m/s`、`1.35 rad/s`，历史静态可行性基线 | `1.20 m/s`、`3.40 rad/s`，用于主动绕行 |
| RGB-D 局部层 | 标准 `VoxelLayer` | 具有时效衰减的 STVL |
| 全局 RGB-D 层 | 保留，用于识别低于 LiDAR 平面的静态方块 | 禁用，避免运动方块离开视场后形成全局残留 |

静态主文件 [`nav2_params.yaml`](../ros2_ws/src/robot_navigation/config/nav2_params.yaml) 的参数值已逐项与 `codex/kujiale-mppi-feasibility-tuning` 对比，差异为零。`nav2_stable.yaml` 只重复声明静态 profile 的关键控制值，用于启动时的契约检查。动态增量只写在 [`nav2_dynamic_avoidance.yaml`](../ros2_ws/src/robot_navigation/config/nav2_dynamic_avoidance.yaml)。

## 2. 动态场景与生命周期

三阶段路线为 `G1→G2→G3→G4→G5→G1`：

| 航段 | 方块交互 | 目的 |
|---|---|---|
| G1→G2 | 横向方块由左向右移动并停车 | 验证机器人从方块右侧连续绕行后进入左侧狭窄通道 |
| G2→G3 | 窄道出口附近的方块纵向下移并停车 | 验证机器人跟随、留出出口转向空间及绕行 |
| G5→G1 | 门洞区域方块横穿并向外停车 | 验证机器人在可行侧通过，避免尝试钻入不可通行缝隙 |

每个方块的状态机是 `waiting → armed → moving → parked → retired`：机器人到达该段对应的下一航点才退役方块；目标失败或超时则保留现场，供人工检查。任一时刻只允许一个方块同时可见且可碰撞。

## 3. 已遇到的问题与处理

### 3.1 Isaac 动态配置哈希不一致

**现象**：运行器报 `Isaac dynamic obstacle configuration hash does not match the scenario`。

**根因**：动态 YAML 在 Isaac 启动后被修改；Isaac 在进程创建时已读取旧障碍物配置，运行器读取的是工作区新文件。

**处理**：可视化脚本启动前比较 Isaac 运行时哈希与工作区 YAML。哈希不一致时必须重启 Isaac；只重启 ROS 无效。

### 3.2 `/ground_truth/odom` 不存在或 reset 对齐超时

**现象**：`Unknown topic '/ground_truth/odom'`，或运行器等待出生点对齐时超时。

**根因**：Isaac 未以 Ground Truth 开关启动、ROS Domain/RMW 环境不一致、DDS 发现窗口过短，或 Isaac 尚未完成第一帧发布。

**处理**：动态入口固定导出 `ISAAC_NAV__GROUND_TRUTH__ENABLED=true`；可视化脚本使用连续 15 s 的 `--no-daemon` 发现窗口。开始 runner 前需确认 Isaac 日志出现 `ground_truth=True`。

### 3.3 单段测试仍从 G1 运行

**现象**：执行 `g2-g3` 或 `g5-g1` 时，机器人仍从 G1 出生，导致触发门与方块事件不成立。

**根因**：单段 runner 的场景起点与 Isaac/ROS 的出生点没有同步。

**处理**：`g2-g3` 必须在 `long_route_start_g2` 启动 Isaac 和 Nav2；`g5-g1` 必须使用 `long_route_start_g5`。完整三阶段联测则始终用 G1 出生点。

### 3.4 动态方块不动、触发过早或过晚

**现象**：方块显示后不移动；机器人已经接触方块才触发；或方块提前移动，退化成静态障碍。

**根因**：触发门同时依赖位置、行进方向和最低速度；出生点不对、空间门不对或速度门不满足都会使状态停在 `armed`。触发位置不在机器人可见且仍有机动余量的区域，也会破坏实验意义。

**处理**：每段以 Map 坐标、方向和速度共同触发；单段场景使用经过标定的出生点。方块停车后仍保持物理碰撞，直到到达下一航点，不能用“障碍物消失后继续走”作为成功。

### 3.5 RGB-D 移动历史残留

**现象**：动态方块通过后，在 Local/Global Costmap 留下一条长条障碍，机器人无法重新规划。

**根因**：前视相机的普通 `VoxelLayer` 只有在后续射线再次扫到旧单元时才清除；离开相机视场的运动方块无法保证被回访清除。将点云直接固定注入 Local/Global Costmap 会把错误占用固化，不能作为解决方案。

**处理**：动态 profile 在**局部**使用 STVL：相同深度点云用于 marking，同时通过体素寿命和当前视锥加速衰减清除离场方块。动态**全局**图不接入相机点云，仅保留静态地图和 LiDAR。静态 profile 保留标准 RGB-D VoxelLayer，因为静态低矮方块不会离场。

### 3.6 RViz 中看不到体素层或打开后为空

**现象**：`/local_costmap/voxel_grid` 在 RViz 看不到，或显示为空。

**根因**：该 topic 是 `nav2_msgs/msg/VoxelGrid`，不是 PointCloud2；并且只有标记体素会显示。若方块不在相机视场、未启用 `Marked Voxels (3D)`、或动态层改用 STVL 而 RViz 仍订阅旧 topic，也会显示为空。

**处理**：RViz 使用项目的 `navigation.rviz`；检查 RGB-D Fusion 分组中的 Depth PointCloud2、Marked Voxels (3D) 和对应 STVL topic。空体素显示不等于静态地图或 LiDAR 图为空。

### 3.7 高控制频率带来的卡顿与错过周期

**现象**：机器人出现顿挫、路径不断重发，日志含 `Control loop missed` 或 `Optimizer fail to compute path`。

**根因**：Isaac、RGB-D、Costmap、MPPI 候选轨迹可视化和 RViz 共用计算资源。过高控制频率或密集候选 Marker 序列化会形成追赶周期；动态场景又不能把采样数和转向能力降得过低。

**处理**：不再要求静态和动态使用同一控制包络。静态回归使用已验证的 10 Hz/700 样本基线；动态使用可持续的 15 Hz/500 样本，并仅在动态 profile 下下采样候选轨迹。RViz 默认关闭 Candidate Trajectories，只保留最优轨迹。

### 3.8 全局黄色路径锯齿或不可通行

**现象**：RViz 黄色 `/plan` 产生明显折线，局部控制器重复失败。

**排查结论**：黄色线是 Global Plan；静态运行时的 SmacPlanner2D、平滑器、膨胀和代价参数与历史分支一致。问题不是全局规划参数漂移，而是静态方块位置改变后，全局 RGB-D VoxelLayer 的占用和膨胀区发生变化。

**处理**：为严格复现历史静态 20 轮，将 `rgbd_low_box_east` 恢复为 `[0.366563, 0.667950, 0.08]`。该坐标同步写入 Isaac 静态 YAML、GUI 草案和 campaign。若未来再次修改静态障碍位置，即构成新布局，必须重新做静态验证，不能沿用旧批次结论。

## 4. 当前测试入口

### 静态单轮可视化

```bash
# Terminal A
cd /home/lyb/Workspace/Isaac_Sim_ROS2_Nav
ISAAC_NAV__GROUND_TRUTH__ENABLED=true ./scripts/run_isaac.sh \
  --environment-usd kujiale_0026_A_to_B_door_open.usd \
  --navigation-mode localization --mode ideal \
  --spawn-pose long_route_start_g1 \
  --camera-profile rgbd_navigation \
  --dynamic-obstacle-config isaac_sim/configs/experiments/kujiale_long_range_static.yaml \
  --dynamic-obstacles

# Terminal B
./scripts/run_ros.sh navigation odometry_mode:=ideal \
  spawn_pose_name:=long_route_start_g1 nav2_profile:=stable

# Terminal C
./scripts/run_visual_route.sh static
```

### 动态三阶段整圈可视化

```bash
# Terminal A
cd /home/lyb/Workspace/Isaac_Sim_ROS2_Nav
./scripts/run_kujiale_dynamic_isaac.sh

# Terminal B
./scripts/run_ros.sh navigation odometry_mode:=ideal \
  spawn_pose_name:=long_route_start_g1 nav2_profile:=dynamic_avoidance

# Terminal C
./scripts/run_kujiale_three_stage_visual.sh full \
  --variant 1 --seed 7501 --record
```

每次从静态切换到动态，或更改动态 YAML/出生点后，都要先停止 ROS 和 Isaac，再完整重启。不要在旧 Isaac 进程上热改障碍配置。

## 5. 验收边界

单轮可视化和 `--record` 仅用于观察行为与保存调试证据；最终验收由人工执行。动态成功至少应同时满足：方块实际触发、移动、停车；机器人在方块存在期间通过；没有物理接触、`safety_yield` 或 `guard_abort`；到达下一航点后方块与 RViz 标记才退役，且代价地图在规定时限内清除。
