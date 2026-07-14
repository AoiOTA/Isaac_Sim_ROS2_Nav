# Isaac Sim ROS 2 导航质量、机器人动力学与仿真一致性升级方案

> 建议文件路径：`docs/navigation_quality_and_simulation_fidelity_upgrade_plan.md`
> 建议 Codex Goal 名称：**修复导航质量、底盘动力学与仿真一致性**
> 适用仓库：`AoiOTA/Isaac_Sim_ROS2_Nav`
> 适用环境：Ubuntu 24.04、ROS 2 Jazzy、Isaac Sim 6.0.1.0、RTX 4090、Fast DDS、ROS_DOMAIN_ID=42
> 文档性质：新增的专项实施计划、问题复现记录、根因验证方案、参数调优方案、测试矩阵与验收标准
> 执行方式：在当前 Codex Goal 和当前仓库基础上继续，不重新初始化项目，不推翻现有架构，不覆盖已有计划文档；本文全部阶段属于同一个不可拆分的 Goal，必须一次性连续执行到全部验收、文档回填、提交与推送完成后才停止；不得在任何阶段边界触发 Goal 模式暂停、挂起或等待 `/goal resume`

---

## 0. 给 Codex 的直接执行指令

继续当前 Goal，不要重新创建项目，不要重新设计已经完成的总体架构，也不要把本轮任务简化为只修改几个 YAML 参数。

请先把本文完整保存为：

```text
docs/navigation_quality_and_simulation_fidelity_upgrade_plan.md
```

然后将本文所有阶段视为同一个不可拆分的 Goal，严格按照阶段依赖顺序连续实施、测试、修复、复测、回填、提交和推送。阶段只是执行顺序和内部里程碑，不是需要用户逐阶段确认的独立 Goal。除非遇到无法通过代码、配置、测试重试或现有权限解决的真实外部阻塞，否则不得在任一阶段结束后停止、暂停、挂起、结束当前执行轮次、等待用户回复、要求用户输入“继续”或 `/goal resume`、只汇报阶段进度或把剩余阶段留到下一轮；不得触发 Goal 模式的阶段完成暂停、检查点暂停、handoff、yield 或等待恢复状态，必须在同一连续执行上下文中立即调用下一阶段所需工具和命令，直至本文全部阶段、测试矩阵、正式统计、文档回填、提交与推送全部完成。

本轮任务的核心不是“让测试通过”，而是解决已经在真实 Isaac Sim + ROS 2 运行中复现的以下问题：

1. 空旷环境中的导航轨迹明显弯曲、回绕和反复纠偏；
2. 机器人转弯、原地旋转和倒车速度异常缓慢；
3. 纯角速度、零线速度命令下，四轮滑移转向底盘出现明显车体平移和漂移；
4. 目标附近可能继续运动、反复调整或无法及时稳定停车；
5. RViz 中 `Navigation: inactive` 与实际目标正在执行、Feedback 正常之间矛盾；
6. 前置相机画面严重模糊；
7. Mapping Teleop 只能逐个字符覆盖命令，不能同时按下 `W+D` 等组合键；
8. RTX LiDAR、仿真时间映射、轮胎碰撞体、TGS 求解、旧 OmniGraph 接口等持续产生警告；
9. 当前 Nav2 参数对前进有过强偏好，对倒车和大角度转向不友好；
10. 当前 Collision Monitor、Velocity Smoother、MPPI、Costmap、Planner、底盘物理模型之间缺少联合标定和可复现评价。

### 0.1 执行约束

Codex 必须遵守：

- 先检查当前分支、工作树、最近提交和已合并改动，保留用户现有修改；
- 本文所有阶段必须在同一个 Goal 运行中自动串联完成，不得拆成多个 Goal、多个需要用户确认的阶段性任务或后续待办，也不得使用会在阶段完成后自动暂停并等待 `/goal resume` 的分阶段 Goal、子 Goal 或检查点机制；
- 完成某一阶段后，只能记录阶段结果、提交必要的中间 Commit 并立即进入下一阶段，不得停止、暂停、挂起、结束当前执行轮次，或请求用户再次发送“继续”或 `/goal resume`；
- 阶段内出现测试失败、指标不达标、回归或可修复警告时，必须在同一 Goal 中诊断、修改、重建和复测，不能把失败作为提前结束理由；
- 只有全部阶段和验收项完成，或出现确实需要用户凭据、硬件操作、外部资产且无法绕过的真实外部阻塞时才允许停止；外部阻塞必须提供完整证据和最小解阻动作；
- 不直接修改 NVIDIA 官方资产源文件；
- 机器人和环境物理修复通过项目 Overlay、项目配置和项目代码完成；
- 不覆盖 `plan.md`；
- 不覆盖 `docs/rviz_workflow_upgrade_plan.md`；
- 不覆盖 `docs/runtime_reliability_and_performance_upgrade_plan.md`；
- 新文档与上述文档建立明确关系；
- 保留 Ideal/Realistic 两种里程计模式；
- 保留 Mapping、Incremental Mapping、Localization、Navigation 四种操作；
- 保留唯一 TF 主链：
  `map -> odom -> base_link -> wheel/sensor frames`；
- ROS TF 中不得引入 `world`；
- 保留命令链：
  `/cmd_vel_nav -> /cmd_vel_smoothed -> /cmd_vel -> Isaac`；
- 保留 Map Manifest、Reset epoch、Lifecycle Activation Gate、单实例锁和受管退出机制；
- 不为了消除警告而静默吞掉错误；
- 不把“配置值”当作“实测结果”；
- 不把单元测试通过写成导航效果已经验收；
- 每一个参数修改都必须有基线、修改理由、测试结果和回滚入口；
- 先修复传感器、时间和底盘执行层，再冻结最终 Nav2 参数；
- 不允许仅凭视觉感觉宣布问题解决。

### 0.2 实施提交要求

建议按阶段形成独立提交，至少包括：

1. `docs: add navigation quality and simulation fidelity plan`
2. `test: add navigation and skid-steer baseline diagnostics`
3. `fix: stabilize simulation timing and lidar motion pipeline`
4. `fix: calibrate skid-steer dynamics and wheel contact model`
5. `feat: add navigation quality profiles and rotation shim`
6. `fix: synchronize rviz navigation lifecycle status`
7. `feat: add multi-key mapping teleop`
8. `fix: improve front camera clarity`
9. `refactor: migrate deprecated Isaac graph interfaces`
10. `docs: record runtime evidence and final verification`

不得在没有测试结果的情况下把所有改动压成一个难以审查的大提交。独立提交只用于版本控制、审查和回滚，不代表阶段结束后可以停止、暂停或等待 `/goal resume`；每次中间提交完成后必须在同一连续执行中继续执行下一阶段，直到整个 Goal 全部完成。

---

## 1. 文档定位与现有文档关系

本文件是专项增量计划，重点解决“系统已经能跑，但导航质量、底盘运动、传感器一致性和交互体验不合格”的问题。

现有文档职责保持：

| 文档 | 职责 |
|---|---|
| `plan.md` | 项目总体架构、长期建设目标、模块边界和最终指标 |
| `docs/rviz_workflow_upgrade_plan.md` | RViz 一体化、Lifecycle、Goal Tool、Mapping Teleop 和交互工作流 |
| `docs/runtime_reliability_and_performance_upgrade_plan.md` | 运行时可靠性、性能、Camera、Profiler、Reset、地图和关闭流程 |
| `docs/navigation_quality_and_simulation_fidelity_upgrade_plan.md` | 本轮新增：导航轨迹质量、底盘滑移转向动力学、传感器时间一致性、MPPI/Planner/Costmap/Collision Monitor 联合调参、RViz 状态同步、组合键 Teleop、警告治理 |

本文件不得把前两个升级方案已经完成的内容重新实现一遍。应复用现有脚本、Profiler、Activation Gate、实验框架和测试基础。

---

## 2. 当前确认的项目基线

### 2.1 当前系统结构

Isaac 侧：

```text
Warehouse USD
Jackal Articulation
RTX LiDAR
IMU
Front RGB Camera
Ideal Odom / Ground Truth
/cmd_vel -> DifferentialController -> 四轮 ArticulationController
```

ROS 侧：

```text
/lidar/points_raw
    -> pointcloud_to_laserscan
    -> /scan
    -> SLAM Toolbox / Costmap / Collision Monitor

Ideal:
Isaac /odom + odom->base_link

Realistic:
joint_states -> wheel_odometry -> /wheel/odom
IMU + wheel odom -> EKF -> /odom + odom->base_link

map->odom:
SLAM Toolbox Localization

Navigation:
SmacPlanner2D
MPPI
Velocity Smoother
Collision Monitor
BT Navigator
```

### 2.2 当前 Nav2 关键参数基线

当前主配置 `ros2_ws/src/robot_navigation/config/nav2_params.yaml` 中：

```yaml
controller_frequency: 10.0

progress_checker:
  required_movement_radius: 0.30
  movement_time_allowance: 15.0

goal_checker:
  xy_goal_tolerance: 0.20
  yaw_goal_tolerance: 0.174532925

FollowPath:
  plugin: nav2_mppi_controller::MPPIController
  time_steps: 20
  model_dt: 0.10
  batch_size: 1000
  vx_std: 0.20
  wz_std: 0.40
  vx_max: 1.00
  vx_min: -0.20
  wz_max: 1.50
  regenerate_noises: true
  motion_model: DiffDrive

  PathAlignCritic:
    cost_weight: 14.0
    threshold_to_consider: 0.5
    offset_from_furthest: 20

  PathAngleCritic:
    cost_weight: 2.0
    mode: 0

  PreferForwardCritic:
    enabled: true
    cost_weight: 5.0
```

当前局部 Costmap：

```yaml
width: 4
height: 4
resolution: 0.05
inflation_radius: 0.55
cost_scaling_factor: 3.0
obstacle_max_range: 25.0
raytrace_max_range: 25.0
```

当前 Velocity Smoother：

```yaml
smoothing_frequency: 20.0
scale_velocities: false
feedback: OPEN_LOOP
max_velocity: [1.0, 0.0, 1.5]
min_velocity: [-0.2, 0.0, -1.5]
velocity_timeout: 0.50
```

当前 Collision Monitor：

```yaml
SlowdownZone:
  min_points: 3
  slowdown_ratio: 0.35

ApproachZone:
  time_before_collision: 1.2
  simulation_time_step: 0.1
  min_points: 3
```

这些参数形成明显的行为倾向：

- 前进最高速度 1.0 m/s；
- 倒车只有 -0.2 m/s；
- `PreferForwardCritic` 对倒车额外惩罚；
- `PathAngleCritic mode=0` 偏好前进；
- `PathAlignCritic weight=14` 强制贴合参考路径；
- 转弯时一旦 Collision Monitor SlowdownZone 触发，速度只剩 35%；
- Velocity Smoother 开环使用上次命令而非实测速度；
- 局部 Costmap 只有 4 m × 4 m；
- MPPI 对实际四轮滑移、侧向漂移和有效轮距误差没有建模。

### 2.3 当前机器人基线

```yaml
schema_version: 2
kinematics_profile_id: jackal_legacy_geometric_v1
lifecycle: stable_baseline
wheel_radius: 0.098
wheel_width: 0.040
geometric_track_width: 0.37559
effective_track_width: 0.37559
wheelbase: 0.262

controller:
  max_linear_speed: 1.0
  max_angular_speed: 1.5
```

当前 robot schema 已把物理几何轮距与控制/轮速里程计使用的有效轮距拆成两个显式
字段。稳定 profile 为保持行为不变，暂把二者都设为 `0.37559 m`；这只是迁移基线，
不是有效轮距已标定。Isaac DifferentialController、Robot Description 和 Realistic
Wheel Odom 都从同一个 robot YAML 取得对应字段及 wheel joint。runtime provenance
v5 负责发布 path/SHA/profile/lifecycle/数值身份；motion runner 还完整校验 canonical
topology/contact JSON 与 SHA，Wheel Odom 则握手 schema、robot path/SHA 和七个
kinematics/controller 字段，只有启动前逐项匹配后才创建业务端点。后续候选必须保存为独立 `experimental_candidate` profile，
不得直接覆盖 stable。

---

## 3. 已执行操作、随后出现的现象和报错

本节必须保留为实际复现记录。实施完成后，Codex 应逐项回填“已确认根因”“修改文件”“复测结果”。

---

## 3.1 Ideal 导航启动后进行运行时快速检查

### 执行步骤

终端 A：

```bash
cd /home/lyb/Workspace/Isaac_Sim_ROS2_Nav

./scripts/run_isaac.sh \
  --navigation-mode localization \
  --mode ideal \
  --camera-profile monitoring
```

终端 B：

```bash
cd /home/lyb/Workspace/Isaac_Sim_ROS2_Nav

./scripts/run_ros.sh navigation \
  odometry_mode:=ideal \
  posegraph_file:="$PWD/data/maps/posegraphs/warehouse_v1"
```

随后执行：

```bash
ros2 topic list
ros2 topic hz /clock
ros2 topic hz /lidar/points_raw
ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic info /odom --verbose
```

### 发生的现象

#### `/clock`

```text
average rate: 113.354
min: 0.000s max: 0.019s
```

后续稳定在约 112～113 Hz。

这不等同于 Physics 运行在 113 Hz，但说明 `/clock` 消息到达存在同一墙钟周期内的突发或重复间隔，需要确认：

- `/clock` 是否只有一个发布者；
- 时间戳是否严格单调；
- 每个仿真步是否只推进一次；
- `SimulationManager.step()` 和 `app.update()` 是否造成双重回调或时间映射缺口。

#### `/lidar/points_raw`

刚开始执行测频时出现：

```text
WARNING: topic [/lidar/points_raw] does not appear to be published yet
```

随后恢复：

```text
average rate: 8.410
average rate: 8.913
average rate: 9.103
average rate: 9.181
```

说明 DDS 发现或第一帧等待后能够收到点云，但目标 10 Hz 在墙钟下通常只有约 9 Hz。

#### `/scan`

```text
average rate: 9.532
average rate: 9.390
average rate: 9.406
average rate: 9.441
average rate: 9.311
```

点云到 LaserScan 链路基本可用。

#### `/odom`

```text
average rate: 112.694
average rate: 102.310
average rate: 96.327
average rate: 100.569
min: 0.000s
max: 0.281s / 0.340s
```

Ideal Odom 只有一个发布者：

```text
Publisher count: 1

Node name: _World_Graphs_IdealOdometry_PublishOdometry
```

因此不是重复 `/odom` 发布者问题，但存在突发和偶发长间隔，需要与仿真步进、CPU 调度和 ROS Graph 更新一起排查。

#### `map -> odom`

刚启动 `tf2_echo` 时：

```text
Waiting for transform map -> odom:
Invalid frame ID "map" passed to canTransform argument target_frame - frame does not exist
```

随后立即恢复并持续输出：

```text
Translation: [-0.010, -0.013, -0.000]
Rotation RPY degree: [0.000, -0.000, -0.199]
```

这是新 TF Buffer 启动时的瞬态，不是持续 TF 缺失。

#### `odom -> base_link`

刚启动时：

```text
Waiting for transform odom -> base_link:
Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist
```

随后恢复：

```text
Translation: [2.685, -0.167, -0.000]
Rotation RPY degree: [-0.000, 0.000, -3.364]
```

同样属于新订阅者尚未收到 TF 的瞬态。

### 需要保留的判断

- TF 主链最终存在；
- Ideal `/odom` 所有权正确；
- 不能把初始一次 `Invalid frame ID` 当作根因；
- 需要重点处理时间推进、消息突发和传感器时间一致性。

---

## 3.2 `/scan` QoS 警告

在启动 RViz 或新订阅者后出现：

```text
New subscription discovered on topic '/scan',
requesting incompatible QoS.
Last incompatible policy: RELIABILITY_QOS_POLICY
```

可能原因：

- `/scan` 使用 Sensor Data / Best Effort；
- 某个订阅者使用 Reliable；
- RViz、CLI 或其他节点产生不兼容订阅。

当前核心导航链仍能工作，所以不是所有订阅者都失配。必须通过：

```bash
ros2 topic info /scan --verbose
```

精确记录每一个发布者和订阅者的 QoS，不得仅靠猜测。

验收要求：

- Navigation、Mapping、Localization RViz 中的 LaserScan 均使用 Best Effort；
- PointCloud2 和 Camera RViz Display 使用与发布端一致的 QoS；
- 不再出现项目自有订阅者造成的 QoS 不兼容；
- CLI 调试命令在文档中明确使用传感器 QoS。

---

## 3.3 启动阶段 Message Filter 警告

曾出现：

```text
timestamp on the message is earlier than all the data in the transform cache
```

以及：

```text
discarding message because the queue is full
```

可能是：

- Isaac 已经开始发布；
- ROS、SLAM Toolbox 或 RViz 的 TF Buffer 尚未准备；
- 时间回跳、Reset epoch 或时间映射缺口；
- 新旧传感器消息跨越启动边界；
- RViz 初始化时队列积压。

如果只在启动瞬间出现且后续稳定，可以降级为启动瞬态；如果导航过程中持续出现，则属于 P1 时间一致性故障。

---

## 3.4 SmacPlanner2D Inflation 警告

曾出现：

```text
Inflation layer either not found or inflation is not set sufficiently
for optimized non-circular collision checking capabilities
```

当前源码中的 global/local `inflation_radius` 已经是 `0.55`，大于 Jackal footprint 外接半径约 0.32 m。

因此再次出现时优先排查：

- 实际运行是否加载了旧 `install/` 配置；
- 是否加载了旧 overlay；
- 是否混入其他工作区；
- `AMENT_PREFIX_PATH` 是否包含旧 `jazzy_ws`；
- 是否修改源码后未重新 build；
- 实际节点参数是否仍不是 0.55。

必须执行：

```bash
ros2 param get /global_costmap/global_costmap inflation_layer.inflation_radius
ros2 param get /local_costmap/local_costmap inflation_layer.inflation_radius
```

并把实测值写入验证报告。

---

## 3.5 RViz 显示 `Navigation: inactive`，但实际导航正在运行

截图中同时出现：

```text
Navigation: inactive
Localization: active
Feedback: active
ETA: ...
Distance remaining: ...
```

这说明 NavigateToPose 正在收到反馈，但面板状态标签没有更新。

当前自定义面板在启动时通过 `InitialThread` 查询 Lifecycle Manager 状态，查询完成后线程结束。如果 RViz 查询发生在 Activation Gate 激活 Navigation 之前，面板会记录 `inactive`，之后不再持续刷新。

这是已确认的代码级状态同步缺陷，不是 Nav2 后端一定处于 inactive。

禁止用“手工点击 Startup”规避，因为 Activation Gate 是当前项目的 Lifecycle owner。重复点击可能造成重复 Lifecycle 转换。

修复目标：

- Activation Gate 继续作为唯一 Lifecycle owner；
- RViz Panel 不再拥有重复 startup/reset/pause 控制权，或所有控制都必须委托给 Gate；
- 面板周期刷新或监听 Transition Event；
- 状态至少包含：
  - waiting for readiness
  - activating
  - active
  - recovering
  - paused
  - inactive
  - error
- 状态变化在 1 秒内反映；
- `Feedback: active` 时不允许同时长期显示 `Navigation: inactive`。

---

## 3.6 全局路径简单，但实际历史轨迹弯曲、回绕

用户已经确认：

- 细线是全局规划路径；
- 粗蓝线是机器人实际走过的历史轨迹；
- 并非 MPPI Candidate Trajectories；
- 在空旷区域，目标只需要转向后直行；
- 实际轨迹却出现弯曲、回绕、重复纠偏；
- 转弯和倒车明显比直行慢；
- 目标附近有时继续移动或反复调整。

必须把这个问题视为真实控制质量问题，不能再解释为 RViz 显示误解。

需要区分：

1. 第二个目标抢占第一个目标造成“不在第一个目标停车”；
2. 目标位置满足但最终 yaw 未满足造成继续旋转；
3. Goal Succeeded 后命令链仍有非零速度；
4. Velocity Smoother 缓慢衰减；
5. Collision Monitor 修改命令；
6. 机器人物理滑移造成实际运动偏离控制器预测；
7. MPPI 参数导致不必要的反复纠偏。

验收必须同时观察：

```text
/cmd_vel_nav
/cmd_vel_smoothed
/cmd_vel
/collision_monitor_state
/odom
/ground_truth/odom
/ground_truth/path
/plan
/optimal_trajectory
```

---

## 3.7 纯角速度命令下底盘明显漂移

复现方式：

```text
linear.x = 0
angular.z != 0
```

预期：

- 四轮滑移转向轮胎会发生侧向擦滑；
- 车体中心应基本保持在原地；
- 机器人应以接近零转弯半径旋转。

实际：

- 车体中心有明显平移；
- 旋转轨迹不是围绕固定中心；
- 左右转可能不完全对称；
- Nav2 中转向后实际轨迹偏离理想 DiffDrive 预测。

这不是单纯“轮胎不能滑”的问题。四轮 skid-steer 原地转向本来就需要轮胎侧滑，但车体中心持续大幅漂移说明模型、接触、Joint、质量分布或控制参数至少有一项不对。

当前必须验证：

- 四个轮子 Joint Axis；
- 正速度方向；
- 左右、前后 Joint 顺序；
- Collider 尺寸和轴向；
- 轮胎与地面的物理材料；
- 轮胎各向同性/各向异性摩擦；
- 机器人质心和惯量；
- 前后轮接触是否对称；
- 几何轮距与有效轮距；
- solver position/velocity iterations；
- 60 Hz 与 120 Hz 物理步长对结果的影响；
- Warehouse Mesh 地面与简单 Plane 地面的差异。

---

## 3.8 Mapping Teleop 不能同时按 `W+D`

当前 Teleop 的按键映射是：

```python
'w': (1.0, 0.0)
's': (-1.0, 0.0)
'a': (0.0, 1.0)
'd': (0.0, -1.0)
```

每次收到一个字符都会覆盖整个命令：

```text
按 W:
linear.x > 0
angular.z = 0

随后收到 D:
linear.x = 0
angular.z < 0
```

因此无法产生：

```text
linear.x > 0
angular.z < 0
```

普通 raw terminal 只能稳定读取字符，不具备真正的 KeyDown/KeyUp 状态，所以不能可靠判断两个键是否同时按住。

这属于实现限制，不是用户键盘问题。

---

## 3.9 前置相机严重模糊

当前默认 profile：

```yaml
monitoring:
  width: 640
  height: 360
  publish_rate_hz: 15.0
```

当前光学与曝光：

```yaml
focus_distance_m: 4.0

exposure:
  enabled: true
  time_s: 0.02
  responsivity: 1.10267
  f_stop: 5.0
```

启动日志还出现：

```text
DLSS increasing input dimensions:
Render resolution of (320, 180) is below minimal input resolution of 300.
```

说明 640×360 输出可能使用了 320×180 的内部 DLSS 输入，再进行放大；固定 4 m 焦点、FStop=5 和 20 ms 曝光会继续叠加失焦和运动模糊。

必须分别验证：

- 静止时清晰度；
- 移动时运动模糊；
- monitoring/standard/high_quality 的差异；
- RViz 缩放是否造成额外模糊；
- 光学 FStop 与曝光 FStop 被同一配置值耦合的问题；
- Sensor Render Product 的 DLSS/AA 设置。

---

## 3.10 Isaac Mapping 启动后出现的警告

### 启动模式

日志显示：

```text
Isaac navigation simulation ready:
navigation=mapping,
odometry=ideal,
structure_tf=isaac,
spawn=mapping_start,
dynamic=False,
camera=monitoring,
pacing=realtime,
target_rtf=1.000
```

### USD 诊断被静音

```text
[Warning] [omni.usd] Encountered USD Warnings but USD Diagnostics are currently muted.
To view USD Warnings, please set the
'/persistent/app/usd/muteUsdDiagnostics' setting to false in Preferences page.
```

含义：存在 USD 层警告，但详细信息被隐藏。应提供项目级可选启动参数显示诊断，而不是长期全局静音。

### DLSS 低内部输入分辨率

```text
[Warning] [omni.rtx] DLSS increasing input dimensions:
Render resolution of (320, 180) is below minimal input resolution of 300.
```

可能直接影响 Camera 清晰度。

### 四个轮胎 Collider 使用旧属性

四次出现：

```text
[Warning] [omni.physx.plugin] PhysicsUSD:
Prim at path /World/Robots/Jackal/front_left_wheel_link/collisions
is using obsolete 'customGeometry' attribute.
To toggle convex mesh approximation for cylinders,
use the physics settings option.
```

对应路径还包括：

```text
/World/Robots/Jackal/front_right_wheel_link/collisions
/World/Robots/Jackal/rear_left_wheel_link/collisions
/World/Robots/Jackal/rear_right_wheel_link/collisions
```

这与轮胎碰撞体近似方式有关，必须结合转向漂移问题调查。

### TGS 和 Velocity Iterations

```text
[Warning] [omni.physx.plugin]
Detected an articulation at /World/Robots/Jackal
with more than 4 velocity iterations being added to a TGS scene.
The related behavior changed recently, please consult the changelog.
```

必须记录实际 articulation solver iteration 值，并进行 4 次与当前值的 A/B 测试。
当前非弃用公开 getter 的 backend 为 USD，因此现阶段只把它记为 authored USD
输入证据；PhysX 消费差异由 32/4、32/16 的 TGS 警告和运动 A/B 证明。找到真正的
engine getter 前，不得宣称已直接读回 PhysX 内部 solver 值。

### RTX LiDAR Motion BVH

```text
[Warning] [rtx.rtxsensor.plugin]
Multi-tick is enabled but motion BVH is not active.
This is not supported.
```

以及：

```text
[Warning] [rtx.sensors.lidar.core.plugin]
MotionBVH for lidar model not enabled.
This will result in an incorrect point cloud without motion effects!
```

这是 P1 传感器正确性问题。机器人运动和动态障碍条件下，点云可能缺少正确运动效果。必须使用 Isaac Sim 6.0.1 官方支持的方式启用 Motion BVH，或调整传感器模式避免不支持的组合。不得只屏蔽警告。

### Hydra Texture 重复释放

```text
[Warning] [carb]
Plugin interface for a client: omni.hydratexture.plugin was already released.
```

多次出现。需要判断是 NVIDIA 内部扩展还是项目 Render Product 生命周期重复释放。若项目代码触发，必须修复资源所有权；若确认是上游无害警告，记录白名单和证据。

### Mapping 中 Costmap Reset Service 不存在

```text
[WARN] [isaac_navigation_sim]:
global costmap reset service is unavailable;
continuing with reset event/recovery gate
```

```text
[WARN] [isaac_navigation_sim]:
local costmap reset service is unavailable;
continuing with reset event/recovery gate
```

这次是 Mapping 模式，本来没有 Nav2 global/local costmap。当前 Reset Bridge 无条件创建 Costmap Client 并尝试查询，导致误警告。

修复要求：

- Mapping/Incremental Mapping 不创建或不等待 Nav2 Costmap Client；
- Localization/Navigation 才创建或使用；
- 测试确保 Mapping 启动不再产生这两条警告；
- Reset 行为保持确定性。

### Joint State 旧接口

```text
[Warning] [isaacsim.ros2.nodes]
[ROS2 Publish Joint State]
Reading from targetPrim is deprecated.
Connect an Isaac Read Joint State node and use its outputs instead.
```

需要把 Joint State Graph 迁移为：

```text
Isaac Read Joint State
    -> ROS2 Publish Joint State
```

### RTX LiDAR `fullScan` 弃用

```text
[Warning] [OgnROS2RtxLidarHelper]
fullScan is deprecated.
RTX Lidar now always produces full scans via accumulateOutputs.
This setting is ignored.
```

应从 Graph 配置中删除无效 `fullScan` 参数。

### TF 旧接口

```text
[Warning] [isaacsim.ros2.nodes]
OgnROS2PublishTransformTree:
using targetPrims for internal computation is deprecated.
Connect OgnIsaacComputeTransformTree to
inputs:parentFrames/childFrames/translations/orientations instead.
```

需要迁移为：

```text
Isaac Compute Transform Tree
    -> ROS2 Publish Transform Tree
```

### Timeline Callback 弃用

```text
[Warning] [omni.timeline.plugin]
Deprecated: direct use of ITimeline callbacks is deprecated.
Use ITimeline::getTimeline
(Python: omni.timeline.get_timeline_interface) instead.
```

需要定位警告来源。项目自有代码若使用旧回调必须迁移；如果来自上游扩展，记录来源和无法修复边界。

### 仿真时间单调映射缺失

大量重复出现：

```text
[Warning] [isaacsim.core.simulation_manager.plugin]
getSimulationTimeMonotonicAtTime:
no data found for time 4099999999/1000000000,
returning current sim time
```

随后持续出现类似：

```text
no data found for time 9466666666/1000000000
no data found for time 27533333333/1000000000
no data found for time 58333333333/1000000000
...
no data found for time 1108066666666/1000000000
...
no data found for time 2375266666666/1000000000
```

该警告从启动十几秒后持续到运行 2600 秒以上，不是一次性启动瞬态。

当前仿真循环使用：

```python
SimulationManager.step(steps=1, update_fabric=False)
if render:
    app.update()
```

必须验证：

- `update_fabric=False` 是否导致传感器查询的时间没有映射数据；
- `app.update()` 是否与手动 Physics Step 形成额外时间推进或中间时间点；
- RTX Camera/LiDAR 是否是主要触发源；
- 关闭 Camera 后警告是否减少；
- 关闭 LiDAR 累积后是否减少；
- `update_fabric=True` 是否修复且不破坏 RTF；
- `/clock`、传感器 Header、TF Header 是否单调；
- Reset 后 epoch 是否正确。

不得直接把警告过滤掉。

### X11/输入法按键事件

```text
imDefLkup.c,419:
The application disposed a key event with 406 serial.
```

通常与 GUI 焦点或输入法有关。若采用 Qt KeyPress/KeyRelease Teleop，需要验证不会引入新的输入法冲突；失去焦点必须立即停车。

### Semantics API 弃用

```text
[Warning] [semantics.schema.property.semantics_entry]
Semantics.SemanticsAPI is deprecated,
please use SemanticsLabelsAPI instead.
```

若来自项目自有语义代码则迁移；若来自 NVIDIA Warehouse 资产或属性面板，则记录为上游警告，不修改官方资产。

---

## 3.11 其他已观察到的警告

### Goal/Progress Checker 未显式指定

```text
No goal checker was specified ...
No progress checker was specified ...
Server will use only plugin loaded ...
```

由于当前只加载一个 checker，系统仍能运行，但应在 FollowPath 请求或 BT 配置中显式指定，避免未来多插件时行为不确定。

### BT Error Code 参数使用默认值

```text
Error_code parameters were not set.
Using default values ...
```

应检查 Jazzy 当前 BT Navigator/BT XML 的错误码端口和参数要求。若项目可以显式配置，应补全；若是上游兼容性提示，记录依据。

### CPU Powersave

曾出现：

```text
CPU performance profile is set to powersave
```

这会降低 Isaac、SLAM Toolbox、MPPI 和 RViz 的调度能力。项目诊断脚本应检测并提示：

```bash
powerprofilesctl get
```

但不得在未获用户许可时永久修改系统电源配置。测试报告必须记录当前 profile。

---

## 4. 问题优先级

### P0：导航与机器人运动正确性

- 纯旋转车体明显漂移；
- 实际路径弯曲、回绕；
- 转弯和倒车异常缓慢；
- 终点停止不稳定；
- MPPI 模型与实际底盘运动不匹配。

### P1：传感器与时间一致性

- Motion BVH 未启用；
- `getSimulationTimeMonotonicAtTime` 持续警告；
- `/clock` 和 `/odom` 消息突发；
- 启动/Reset 时 TF 与消息时间边界；
- Collision Monitor 可能受到错误扫描影响。

### P1：Lifecycle 状态可信度

- RViz 显示 Navigation inactive，但实际导航活跃；
- Panel 与 Activation Gate 存在状态所有权重叠。

### P2：导航参数与安全链

- MPPI 前进偏好过强；
- 倒车上限过低；
- PathAlign 权重过高；
- Velocity Smoother 开环；
- Slowdown ratio 过低；
- Local Costmap 范围偏小。

### P2：交互与相机

- Camera 模糊；
- Teleop 不支持组合键；
- Mapping 默认速度偏慢。

### P3：弃用接口和上游警告

- Joint State targetPrim；
- TF targetPrims；
- LiDAR fullScan；
- Timeline callback；
- Semantics API；
- Hydra Texture 重复释放。

实施顺序必须按 P0/P1 优先，不能先花大量时间美化 RViz 或 Camera，而底盘和时间仍错误。

---

## 5. 总体技术原则

### 5.1 先测量再调参

任何 Nav2 参数调整前必须建立固定基线，至少记录：

- Goal 成功/失败；
- 总耗时；
- 规划路径长度；
- 实际 Ground Truth 路径长度；
- 路径效率：
  `actual_length / global_plan_length`；
- 最大横向偏差；
- 最终位置误差；
- 最终 yaw 误差；
- Goal Succeeded 后 0.5 秒内是否所有速度归零；
- `/cmd_vel_nav`、`/cmd_vel_smoothed`、`/cmd_vel` 差异；
- Collision Monitor 状态占比；
- 实际角速度与命令角速度；
- 原地旋转中心漂移；
- `/scan` age 和频率；
- `/odom` age 和频率；
- TF lag；
- RTF；
- Controller missed deadline；
- Recovery 次数。

### 5.2 不用 Nav2 参数掩盖物理错误

如果纯角速度命令都不能稳定原地旋转，则不能通过提高 PathAlign、降低速度或加大 goal tolerance 伪装成“导航稳定”。

正确顺序：

```text
仿真时间和传感器
    -> 轮胎/Joint/Collider/物理材料
    -> 有效轮距和底层控制
    -> Velocity Smoother / Collision Monitor
    -> MPPI / Rotation Shim
    -> Planner / Goal Checker
```

### 5.3 保留基线 Profile

不要直接破坏当前：

```text
stable
performance
```

新增至少两个 profile：

```text
quality
quality_bidirectional
```

- `stable`：保留历史基线；
- `performance`：保留历史高采样基线；
- `quality`：前向优先、先转正再前进、适合日常导航；
- `quality_bidirectional`：允许明显倒车，用于对照和特殊场景。

---

## 6. 阶段 0：建立可复现基线和自动评价

### 6.1 新增固定测试场景矩阵

至少包含：

| ID | 场景 | 目标 |
|---|---|---|
| N01 | 正前方 3 m | 直行、加速、停车 |
| N02 | 正后方 3 m，目标朝向向前 | 先转向再前进 |
| N03 | 左侧 3 m | 左转对齐后直行 |
| N04 | 右侧 3 m | 右转对齐后直行 |
| N05 | 90° 拐角 | 路径跟踪和曲率 |
| N06 | 同位置、目标 yaw +90° | 原地终点旋转 |
| N07 | 允许倒车目标 | 倒车速度和稳定性 |
| N08 | 靠近货架但无碰撞 | Collision Monitor 是否误限速 |
| N09 | 横穿动态障碍 | 动态避障 |
| N10 | Scan 丢帧/恢复 | 安全停车和恢复 |

每个基础场景至少运行：

```text
Ideal: 10 次
Realistic: 10 次
```

最终统计阶段再扩展到用户目标要求的更大样本。

### 6.2 新增或扩展诊断工具

建议增加：

```text
scripts/capture_navigation_diagnostics.sh
ros2_ws/src/robot_experiments/robot_experiments/navigation_quality_profiler.py
ros2_ws/src/robot_experiments/robot_experiments/skid_steer_calibrator.py
```

输出目录：

```text
artifacts/navigation_quality/<timestamp>/
```

输出至少包括：

```text
metadata.yaml
topic_publishers.yaml
lifecycle_states.yaml
clock.csv
odom.csv
ground_truth.csv
cmd_vel_nav.csv
cmd_vel_smoothed.csv
cmd_vel.csv
collision_monitor_state.csv
plan.csv
optimal_trajectory.csv
scan_age.csv
tf_lag.csv
metrics.json
warnings.log
summary.md
```

运行输出必须被 `.gitignore` 排除，不得污染仓库。

### 6.3 基线不可伪造

如果自动测试环境无法启动 Isaac GUI 或 RTX Sensor：

- 单元测试验证算法和配置；
- Headless 验证真实 Isaac；
- GUI/RViz 人工矩阵单独记录；
- 未运行的项目明确标为“未验收”，不得写 PASS。

---

## 7. 阶段 1：修复仿真时间与传感器链

### 7.1 定位 `getSimulationTimeMonotonicAtTime`

必须完成以下 A/B：

1. Camera `off` 与 `monitoring`；
2. LiDAR 开启与关闭；
3. `update_fabric=False` 与 `True`；
4. GUI 与 Headless；
5. realtime 与 unbounded；
6. Physics 60 Hz 与临时 120 Hz；
7. 只 `SimulationManager.step`；
8. step + `app.update`；
9. Reset 前后；
10. 长时间运行至少 15 分钟。

每个组合记录：

- 警告次数；
- `/clock` 单调性；
- `/clock` 每步增量；
- `/scan` Header；
- `/odom` Header；
- TF Header；
- RTF；
- CPU/GPU；
- 是否出现未来外推或旧消息。

### 7.2 修改仿真循环

不得直接假设 `update_fabric=True` 就是最终答案。

最终实现必须满足：

- 一个主循环迭代只产生预期的一个 Physics Step；
- Fabric、RTX Sensors、OmniGraph 和 ROS Bridge 获得一致时间；
- GUI 渲染不额外推进仿真；
- realtime 目标 RTF 与实际测量一致；
- `/clock` 一个发布者；
- `/clock` 时间戳严格单调；
- Reset 后允许新 epoch，但旧消息不得进入新 epoch；
- 不再持续出现 `getSimulationTimeMonotonicAtTime`。

对 `isaac_sim/src/stage/physics_setup.py` 增加针对步进策略的单元测试和 Isaac marker 测试。

### 7.3 Motion BVH

Codex 必须查阅 Isaac Sim 6.0.1 当前安装版本的官方 API/设置，不得凭旧版文档猜测设置名。

完成以下之一：

- 正确启用 RTX Motion BVH，使 Multi-tick LiDAR 受支持；
- 或调整 LiDAR 配置，使其不使用不受支持的 Multi-tick 组合；
- 或使用官方推荐的单帧/累积策略。

验收：

- 启动日志中不再出现两条 Motion BVH 警告；
- 静态墙面在机器人原地旋转时，RViz `/scan` 不产生明显弯曲和漂移；
- 动态障碍的位置随时间连续；
- `/scan` 时间戳单调；
- SLAM 和 Costmap 不因旋转产生虚假障碍带。

### 7.4 Mapping Costmap 误警告

修改 `ResetServiceBridge`：

- Mapping 模式不创建或不检查 global/local costmap client；
- Localization 模式如果 ROS 侧运行的是 Navigation，则通过明确 capability/operation 信息决定是否清图；
- 不通过“服务不存在也继续”产生正常模式警告；
- 保持 Reset transaction 有界和可测试。

增加测试：

```text
mapping: no costmap clients
incremental_mapping: no costmap clients
localization-only: no navigation costmap dependency
navigation: costmap clear clients required/optional according to gate contract
```

---

## 8. 阶段 2：修复四轮滑移转向底盘模型

### 8.1 单轮方向验证

新增真实 Isaac 动态测试，不只验证 Joint 名称。

分别对四个轮子施加：

```text
+1 rad/s
-1 rad/s
```

其余轮子保持零。

记录：

- 轮子实际角速度；
- 接触点速度；
- 车体瞬时受力趋势；
- 正命令是否对应机器人 +X 滚动方向；
- 左右轮符号是否一致；
- 前后轮符号是否一致。

如果某个 Joint Axis 方向相反：

- 通过项目 USD Overlay 修复 Joint frame；
- 或在明确配置中增加 per-wheel direction sign；
- 不在控制代码中散落硬编码负号；
- 增加方向契约测试。

### 8.2 Collider 检查和迁移

对四个轮胎：

- 可视化实际 PhysX Collider；
- 检查是否圆柱；
- 检查轴向；
- 检查半径和宽度；
- 检查是否与底盘重叠；
- 检查左右和前后完全对称；
- 检查 obsolete `customGeometry` 最终采用的近似方式。

优先通过 Overlay 或项目导入资产修复，不修改官方资产。

验收：

- 四条 `obsolete 'customGeometry'` 警告消失，或明确证明由只读官方层产生且 Overlay 已覆盖最终碰撞几何；
- Physics Debug 中四轮 Collider 对称；
- 单轮和纯旋转测试通过。

### 8.3 轮胎与地面接触材料

当前 `wheel_static_friction_effort` 等参数是 DOF 内部摩擦，不是轮胎地面接触摩擦。必须为：

```text
wheel collision material
warehouse/simple-plane ground material
```

建立明确项目配置。

需要验证 Isaac 6.0.1 是否支持适合轮胎的各向异性摩擦。如果支持：

- 纵向摩擦较高；
- 侧向摩擦允许 skid-steer 必要滑移；
- 左右轮完全一致；
- 组合规则明确。

如果不支持各向异性：

- 选择经过测试的统一静/动摩擦；
- 通过有效轮距补偿；
- 文档中说明模型限制。

不得通过把摩擦设得极低来“消除转向阻力”，否则直线制动和碰撞行为会失真。

### 8.4 简单 Plane 与 Warehouse A/B

创建测试 Scene/fixture：

```text
SimplePlane + Jackal
Warehouse + Jackal
```

执行同一命令：

```text
v=0, w=+0.4 rad/s, 旋转 360°
v=0, w=-0.4 rad/s, 旋转 360°
v=+0.5, w=0, 行驶 3 m
v=-0.3, w=0, 行驶 2 m
v=+0.4, w=+0.4, 运行 5 s
v=+0.4, w=-0.4, 运行 5 s
```

如果 SimplePlane 正常而 Warehouse 异常，继续检查：

- Warehouse 地面 Mesh 接缝；
- 地面 Material；
- 碰撞近似；
- 非平面法线；
- Scale 和单位。

### 8.5 标定有效轮距

新增配置字段：

```yaml
geometric_track_width: 0.37559
effective_track_width: <calibrated>
```

并明确用途：

- `geometric_track_width`：几何和模型文档；
- `effective_track_width`：控制器和四轮 skid-steer 里程计；
- footprint 不受 effective track width 影响。

通过纯旋转数据拟合：

```text
effective_track_width =
wheel_radius * (right_wheel_rate - left_wheel_rate) / measured_yaw_rate
```

使用多个角速度、左右方向和多次重复做最小二乘拟合，不能只用一次视觉估算。

同步修改：

- `isaac_sim/configs/robots/jackal.yaml`
- Control Graph
- `robot_odometry` Wheel Odom 配置
- Xacro/文档中的物理参数说明
- 参数一致性测试
- 自定义机器人模板

### 8.6 Solver 和物理频率

对以下组合做 A/B：

```text
Physics 60 Hz / 120 Hz
TGS velocity iterations 当前值 / 4
position iterations 4 / 8
CCD on
stabilization on
```

评价：

- 纯旋转中心漂移；
- 直线偏航；
- 命令跟踪；
- RTF；
- CPU/GPU。

只保留带来显著正确性提升且 RTF 可接受的设置。不得为了消除一条 warning 随意降低求解精度。

### 8.7 底盘验收门槛

在空旷平面、Ideal Ground Truth 下：

| 测试 | 机器门槛 |
|---|---|
| 直行 3 m 横向偏差绝对值 | `≤ 0.05 m` |
| 倒车 2 m 横向偏差绝对值 | `≤ 0.08 m` |
| 左/右原地旋转 360° 中心漂移 | 每个方向均 `≤ 0.10 m` |
| 左右旋转漂移差 | `abs(left-right) / max(left,right) ≤ 0.20`；两者均为零时按 `0` 处理 |
| 左/右稳态角速度误差 | 命令区间后半段实际 `angular_z_radps.mean` 相对目标角速度的 `abs(actual-commanded)/abs(commanded) ≤ 0.10` |
| 六个运动段零命令后的静止窗 | 配置的稳定时长 `≥ 0.5 s`，且每段确认的连续静止窗 `≥` 该配置值 |
| 停止判定配置 | 线速度阈值 `≤ 0.02 m/s`、角速度阈值 `≤ 0.05 rad/s`、轮速阈值 `≤ 0.20 rad/s` |
| 四轮速度方向 | 六段全部符合配置契约 |

机器判定使用 policy `skid_steer_plan_8_7_v1`。motion report 顶层 schema 2 保持
`configuration.schema_version=1`，并在 `actual_velocity.steady_state_window` 保存命令
时间区间后半段的实际角速度分布；旋转门读取该窗口的 `angular_z_radps.mean`，不再用
yaw gain。v5 离线分析结果为 analysis schema 3，内嵌 `physical_acceptance` schema 1。

机器门只适用于同时满足 runtime provenance schema 5、`SimplePlane`、
`simple_plane_only1_v1`、Ideal、每组至少 3 个唯一 repeat、全部 motion report schema 2
的 group。其他 group 必须写 `applicable=false`、`passed=null` 和非空原因，进入
`not_applicable_groups`，不得伪造为物理失败。对适用 group，每个 repeat 的每项检查都
单独计算，任一失败即判定整个 group 失败，不能用均值掩盖最差重复，也不自动排名或
选择 profile。`applicable_groups/not_applicable_groups` 精确划分所有 group，
`passing_groups/failed_groups` 精确划分适用 group；没有适用 group 时
`all_applicable_groups_passed=null`。

batch-summary schema 4 的 `result=success` 只表示证据采集、身份、矩阵与聚合闭合；
物理结论必须另读 `physical_acceptance.all_applicable_groups_passed` 及上述四个列表。
合同已经实现，完整 analyzer 测试文件为 `116 passed`，motion baseline `66 passed`、matrix
script `42 passed / 1 skipped`（缺少 `shellcheck`）；同一 dirty worktree 的
`./scripts/test.sh` 为 exit 0，root `1061 passed / 1 skipped / 34 deselected`，ROS 为
11 packages、861 tests、0 errors、0 failures、1 skipped。但还没有 clean commit
`--with-isaac` 全门、真实新 schema smoke 或正式每组三重复的 54-run/18-group 全
topology 矩阵。历史
`d5840ed` 12-run 保存的是 analysis schema 2、batch-summary schema 3，且 Warehouse、
repeat=1、motion report schema 1 均不满足适用性；它是机制证据，不能写成 `0/12 fail`。

如果不能达到，禁止进入“最终 Nav2 参数冻结”。

---

## 9. 阶段 3：建立新的导航质量 Profile

### 9.1 Profile 设计

保留原 profile，新增：

```text
ros2_ws/src/robot_navigation/config/profiles/quality.yaml
ros2_ws/src/robot_navigation/config/profiles/quality_bidirectional.yaml
```

或使用项目当前已有的 profile/overlay 目录规范。

### 9.2 `quality` 初始候选参数

该 Profile 的策略：

```text
大角度偏差时先安全原地转正
然后由 MPPI 前向跟踪
允许有限倒车，但不鼓励无意义倒车
终点位置满足后由 Rotation Shim 完成最终朝向
```

建议起点：

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 10.0

    progress_checker:
      required_movement_radius: 0.15
      movement_time_allowance: 10.0

    goal_checker:
      stateful: true
      xy_goal_tolerance: 0.15
      yaw_goal_tolerance: 0.261799388  # 15 degrees

    FollowPath:
      plugin: nav2_rotation_shim_controller::RotationShimController

      angular_dist_threshold: 0.52
      angular_disengage_threshold: 0.17
      forward_sampling_distance: 0.40
      rotate_to_heading_angular_vel: 0.55
      max_angular_accel: 1.50
      simulate_ahead_time: 1.0
      rotate_to_goal_heading: true
      rotate_to_heading_once: false
      closed_loop: true
      use_path_orientations: false

      primary_controller:
        plugin: nav2_mppi_controller::MPPIController

        time_steps: 25
        model_dt: 0.10
        batch_size: 1000
        iteration_count: 1

        vx_std: 0.35
        vy_std: 0.0
        wz_std: 0.60

        vx_max: 0.80
        vx_min: -0.35
        vy_max: 0.0
        wz_max: 1.20

        ax_max: 0.75
        ax_min: -1.00
        ay_max: 0.0
        ay_min: 0.0
        az_max: 1.50

        prune_distance: 2.2
        transform_tolerance: 0.30
        temperature: 0.30
        gamma: 0.015
        motion_model: DiffDrive
        visualize: true
        regenerate_noises: false

        critics:
          - ConstraintCritic
          - CostCritic
          - GoalCritic
          - GoalAngleCritic
          - PathAlignCritic
          - PathFollowCritic
          - PathAngleCritic
          - PreferForwardCritic

        ConstraintCritic:
          enabled: true
          cost_power: 1
          cost_weight: 4.0

        CostCritic:
          enabled: true
          cost_power: 1
          cost_weight: 3.81
          critical_cost: 300.0
          consider_footprint: true
          collision_cost: 1000000.0
          near_goal_distance: 0.8
          trajectory_point_step: 2

        GoalCritic:
          enabled: true
          cost_power: 1
          cost_weight: 6.0
          threshold_to_consider: 1.5

        GoalAngleCritic:
          enabled: true
          cost_power: 1
          cost_weight: 4.0
          threshold_to_consider: 0.8

        PathAlignCritic:
          enabled: true
          cost_power: 1
          cost_weight: 8.0
          max_path_occupancy_ratio: 0.10
          trajectory_point_step: 4
          threshold_to_consider: 0.8
          offset_from_furthest: 12
          use_path_orientations: false

        PathFollowCritic:
          enabled: true
          cost_power: 1
          cost_weight: 6.0
          offset_from_furthest: 5
          threshold_to_consider: 1.5

        PathAngleCritic:
          enabled: true
          cost_power: 1
          cost_weight: 2.0
          offset_from_furthest: 4
          threshold_to_consider: 0.8
          max_angle_to_furthest: 1.0
          mode: 0

        PreferForwardCritic:
          enabled: true
          cost_power: 1
          cost_weight: 1.5
          threshold_to_consider: 0.8
```

说明：

- Rotation Shim 用于解决“路径在侧面或后方时，MPPI 低速画弧和反复纠偏”；
- `rotate_to_heading_angular_vel` 不使用官方较高默认值，先以 0.55 rad/s 适配当前 skid-steer；
- `vx_max` 从 1.0 暂时降到 0.8，在动力学未完全稳定时避免高速放大误差；
- `vx_min` 从 -0.2 扩大到 -0.35；
- `PathAlign` 从 14 降到 8；
- `PreferForward` 从 5 降到 1.5；
- `regenerate_noises=false` 减少运行时重新采样抖动；
- 预测窗从 2.0 s 增加到 2.5 s，同时把 Local Costmap 扩大；
- 最终值必须通过矩阵测试决定，不能直接冻结上述候选值。

### 9.3 `quality_bidirectional`

用于验证直接倒车是否优于转向：

```yaml
PathAngleCritic:
  mode: 1

PreferForwardCritic:
  enabled: false

vx_min: -0.50
vx_max: 0.80
```

不得把该 Profile 直接设为日常默认，先比较：

- 总时间；
- 路径长度；
- 反复换向次数；
- 安全距离；
- 用户期望；
- 真实机器人迁移性。

### 9.4 修改参数契约代码

当前项目的 profile 校验可能假设：

```text
FollowPath.model_dt
FollowPath.time_steps
FollowPath.batch_size
```

加入 Rotation Shim 后会变成：

```text
FollowPath.primary_controller.model_dt
FollowPath.primary_controller.time_steps
FollowPath.primary_controller.batch_size
```

必须同步修改：

```text
ros2_ws/src/robot_bringup/robot_bringup/mode_contract.py
ros2_ws/src/robot_bringup/test/test_nav2_profile_contract.py
ros2_ws/src/robot_navigation/test/test_nav2_config.py
相关 launch/overlay 解析
repository_index.md
```

校验必须同时支持：

- 直接 MPPI；
- Rotation Shim + MPPI；
- 正数和有限值；
- `1/controller_frequency <= model_dt` 的现有项目约束；
- Profile 名称和嵌套路径；
- 不允许未知或部分覆盖造成参数缺失。

---

## 10. 阶段 4：Velocity Smoother、Costmap 和 Collision Monitor

### 10.1 Velocity Smoother

在 Ideal 模式优先测试：

```yaml
velocity_smoother:
  ros__parameters:
    smoothing_frequency: 20.0
    scale_velocities: true
    feedback: CLOSED_LOOP

    max_velocity: [0.80, 0.0, 1.20]
    min_velocity: [-0.35, 0.0, -1.20]

    max_accel: [0.75, 0.0, 1.50]
    max_decel: [-1.00, 0.0, -1.50]

    velocity_timeout: 0.25
    odom_duration: 0.10
```

验证：

- CLOSED_LOOP Odom 延迟是否足够低；
- 转弯曲率是否比 OPEN_LOOP 更接近 `/cmd_vel_nav`；
- Goal Succeeded 后是否更快归零；
- Realistic EKF Odom 是否需要独立 overlay；
- `scale_velocities=true` 是否减少线/角速度比例失真。

保留 OPEN_LOOP 回滚 Profile。

### 10.2 Local Costmap

候选：

```yaml
width: 6
height: 6
resolution: 0.05
update_frequency: 10.0
publish_frequency: 5.0

obstacle_layer:
  scan:
    raytrace_max_range: 10.0
    obstacle_max_range: 8.0

inflation_layer:
  inflation_radius: 0.55
  cost_scaling_factor: 3.5
```

理由：

- 2.5 s 预测窗、0.8 m/s 最大速度需要约 2 m 前视；
- 6 m × 6 m 局部地图提供更充足边界；
- Warehouse 室内导航无需每周期处理 25 m 全距离障碍；
- 降低远距离点对局部控制和 CPU 的负担。

必须测量 CPU、Costmap 更新延迟和障碍清除效果。

### 10.3 Global Planner

候选：

```yaml
expected_planner_frequency: 2.0

GridBased:
  tolerance: 0.15
  use_final_approach_orientation: true
  cost_travel_multiplier: 2.0

  smoother:
    max_iterations: 1000
    w_smooth: 0.35
    w_data: 0.20
    tolerance: 1.0e-8
    do_refinement: true
```

Global Planner 不是当前首要根因，不得过度调参。主要目标：

- 路径无不必要锯齿；
- 最终方向与 Goal 一致；
- 不频繁重规划造成路径跳变。

### 10.4 Collision Monitor

先诊断，后改参数。

必须在空旷区域纯旋转时记录：

```text
/collision_monitor_state
/collision_monitor/collision_points_marker
/scan
/cmd_vel_smoothed
/cmd_vel
```

如果无障碍时 Slowdown/Approach 触发：

- 检查自车点；
- 检查 LiDAR Motion BVH；
- 检查 scan 高度切片；
- 检查 sensor frame；
- 检查 footprint 和 polygon；
- 检查动态残影。

确认输入正确后，再测试：

```yaml
SlowdownZone:
  min_points: 5
  slowdown_ratio: 0.55
  release_consecutive_points: 2  # 仅在当前 Jazzy 包实际支持时使用

ApproachZone:
  time_before_collision: 0.80
  simulation_time_step: 0.05
  min_points: 5
```

不得为了让机器人变快而关闭 StopZone。

验收：

- 空旷区域不触发 slowdown/stop；
- 靠近真实障碍按预期限速；
- Scan 持续断流仍及时停车；
- 动态障碍横穿仍保持安全。

---

## 11. 阶段 5：终点停车和命令所有权

### 11.1 明确 Goal 生命周期

自动测试必须区分：

- Goal accepted；
- Goal executing；
- Goal preempted；
- Goal canceled；
- Goal succeeded；
- Goal aborted。

如果用户发送第二个目标导致：

```text
Received goal preemption request
```

则第一个目标点不停车属于预期抢占，不记为停车故障。

### 11.2 Goal Succeeded 后速度链验收

Goal Succeeded 后采集 1 秒：

```text
/cmd_vel_nav
/cmd_vel_smoothed
/cmd_vel
/odom.twist
/ground_truth/odom.twist
```

门槛：

- 0.25 s 内上游命令归零；
- 0.50 s 内 `/cmd_vel` 归零；
- 0.50 s 内 Ground Truth 线速度和角速度接近零；
- 不得有其他 `/cmd_vel` 发布者；
- Idle Brake 不得与 Nav2 非零命令争抢；
- 无目标时 Control Graph 不保持旧轮速。

### 11.3 命令发布者契约

运行时断言：

```text
/cmd_vel_nav: exactly 1 publisher
/cmd_vel_smoothed: exactly 1 publisher
/cmd_vel: exactly 1 normal publisher
```

Reset Bridge 的零速度发布行为需要明确例外和时间窗口，不得长期形成第二个并行控制源。

---

## 12. 阶段 6：修复 RViz Navigation 状态

### 12.1 单一 Lifecycle Owner

Activation Gate 保持唯一 Lifecycle owner。

RViz Panel 改为：

- 状态观察者；
- Goal/Waypoint Action Client；
- 可选调用 Gate 提供的明确服务；
- 不直接与 Gate 并行调用 `startup/reset/pause/resume`。

### 12.2 状态同步方式

优先级：

1. 订阅项目 Activation Gate 状态 Topic；
2. 订阅 Lifecycle transition events；
3. 有界周期轮询 Manager；
4. 不再使用只执行一次的 InitialThread 作为长期状态来源。

新增项目状态 Topic，例如：

```text
/navigation/system_state
```

消息至少包含：

```text
WAITING_READINESS
ACTIVATING
ACTIVE
EXECUTING
RECOVERING
PAUSED
INACTIVE
ERROR
```

### 12.3 UI 验收

- Activation 完成后 1 s 内显示 active；
- Reset 时显示 recovering；
- 定位等待初始位姿时显示 waiting；
- Action Feedback 与 Lifecycle 状态不矛盾；
- 不出现可误点的重复 `Startup`；
- RViz 关闭仍安全，不产生 QThread/QFuture 崩溃。

---

## 13. 阶段 7：实现组合键 Mapping Teleop

### 13.1 输入后端

普通 raw terminal 不能可靠提供 KeyUp，因此不要继续在逐字符模型上叠补丁。

推荐实现：

```text
RViz Mapping Teleop Panel / Tool
```

理由：

- RViz 已依赖 Qt；
- Qt 能提供 keyPressEvent/keyReleaseEvent；
- 不增加 `/dev/input` 权限；
- 不依赖键盘自动重复；
- 可以在 Mapping RViz 中显示当前命令和速度；
- 可在窗口失焦时立即停车。

保留现有终端 Teleop 作为 fallback，但新默认交互使用 Qt Teleop。

### 13.2 按键状态模型

维护：

```text
pressed_keys = set()
```

组合：

| 按键 | linear.x | angular.z |
|---|---:|---:|
| W | +linear | 0 |
| S | -linear | 0 |
| A | 0 | +angular |
| D | 0 | -angular |
| W+A | +linear | +angular |
| W+D | +linear | -angular |
| S+A | -linear | -angular 或按机器人坐标约定验证 |
| S+D | -linear | +angular 或按机器人坐标约定验证 |
| Space | 0 | 0 |

注意：差速小车的 `W+D` 是“前进并右转的圆弧”，不是全向底盘的右上平移。

### 13.3 安全要求

- Mapping/Incremental Mapping 才允许发布；
- Navigation 模式必须禁用；
- 窗口失去焦点立即零速度；
- Key Release 正确更新；
- Space 立即停车；
- Ctrl+C、关闭 RViz、异常退出发布最终零速度；
- 20 Hz 固定发布；
- deadman 作为额外安全层，而不是依赖键盘自动重复；
- 默认线速度可提高到 0.70 m/s；
- 默认角速度先保持 0.60～0.80 rad/s，待底盘标定后再决定。

---

## 14. 阶段 8：修复 Camera 清晰度

### 14.1 分离光学和曝光参数

当前一个 `f_stop` 同时用于 USD Camera 光学景深和曝光。改为：

```yaml
optics:
  projection: perspective
  focal_length_mm: 24.0
  horizontal_aperture_mm: 21.0
  focus_distance_m: 2.0
  f_stop: 16.0

exposure:
  enabled: true
  time_s: 0.005
  responsivity: <calibrated>
  f_stop: <exposure value if API requires>
```

Schema、加载器、测试同步修改。

### 14.2 Profile 策略

保留：

```text
off
monitoring
standard
high_quality
```

可新增：

```text
navigation
```

候选：

```yaml
navigation:
  width: 960
  height: 540
  publish_rate_hz: 15.0
```

但是不要直接提高默认负载。先测试：

- monitoring；
- navigation；
- standard；
- high_quality；
- DLSS 内部输入；
- RTF；
- GPU；
- 清晰度。

### 14.3 Sensor Render Product

检查是否可以为 Camera Render Product：

- 使用原生分辨率；
- 调整或禁用不适合传感器输出的 DLSS；
- 使用适合机器视觉的 AA；
- 避免 320×180 内部输入再上采样。

不得只在 RViz 中放大图像。

### 14.4 验收

- 静止时货架边缘和地面线清晰；
- 运动时无严重拖影；
- 图像不倒置、不镜像；
- CameraInfo 与实际分辨率一致；
- monitoring RTF 不显著下降；
- high_quality 的实际频率明确记录，不把配置 30 Hz 当作实测。

---

## 15. 阶段 9：迁移弃用的 Isaac Graph 接口

### 15.1 Joint State

从：

```text
ROS2 Publish Joint State(targetPrim)
```

迁移到：

```text
Isaac Read Joint State
    -> ROS2 Publish Joint State
```

保持：

- `/joint_states` Topic；
- Joint 名称；
- 时间戳；
- QoS；
- Reset 后连续性。

### 15.2 TF

从：

```text
ROS2 Publish Transform Tree(targetPrims)
```

迁移到：

```text
Isaac Compute Transform Tree
    -> ROS2 Publish Transform Tree
```

保持 TF 所有权不变。

### 15.3 LiDAR

删除被忽略的：

```text
fullScan
```

保留和验证：

```text
accumulateOutputs
```

### 15.4 Timeline

定位旧 ITimeline callback 来源。项目自有代码全部迁移为 Isaac Sim 6.0.1 推荐接口；上游扩展警告记录到 Troubleshooting 的已知问题。

### 15.5 Semantics 和 Hydra

- 项目自有 Semantics API 迁移；
- NVIDIA 只读资产产生的警告不修改源资产；
- Render Product 资源必须只有一个明确 owner；
- cleanup 幂等；
- 不重复 release；
- Reset 不泄漏 Render Product。

---

## 16. 测试与验证

### 16.1 静态测试

必须更新或新增：

```text
isaac_sim/tests/test_camera_contracts.py
isaac_sim/tests/test_joint_mapping.py
isaac_sim/tests/test_control_graph.py
isaac_sim/tests/test_tf_graph.py
isaac_sim/tests/test_sensor_graph.py
isaac_sim/tests/test_physics_setup.py
isaac_sim/tests/test_reset_service.py

ros2_ws/src/robot_navigation/test/test_nav2_config.py
ros2_ws/src/robot_bringup/test/test_nav2_profile_contract.py
ros2_ws/src/robot_bringup/test/test_activation_gate.py
ros2_ws/src/robot_rviz_plugins/test/test_safe_panel_contract.py
ros2_ws/src/robot_teleop/test/test_safety.py
新增 Mapping Teleop Qt 测试
```

### 16.2 构建和全量测试

至少执行：

```bash
./scripts/build_ros2.sh
./scripts/preflight.sh
./scripts/test.sh --with-isaac
```

记录：

- collected；
- passed；
- failed；
- deselected；
- 11/11 package 构建；
- 未运行测试原因。

### 16.3 运行时矩阵

至少完成：

```text
Ideal + Navigation + camera off
Ideal + Navigation + monitoring
Ideal + Navigation + high_quality
Realistic + Navigation + camera off
Mapping + monitoring + Qt Teleop
Mapping + high_quality
SimplePlane skid-steer calibration
Warehouse skid-steer calibration
Dynamic obstacles
Reset during idle
Reset during active goal
Goal preemption
Goal cancel
Scan single/double drop
Scan sustained outage/recovery
```

### 16.4 导航质量验收

第一阶段功能门槛：

| 指标 | 要求 |
|---|---|
| 空旷直线目标成功率 | 10/10 |
| 左转/右转对称场景 | 各 10/10 |
| 后方目标 | 不出现明显回绕，策略可解释 |
| 实际路径/全局路径比 | ≤ 1.20 |
| 无障碍场景 Recovery | 0 |
| Goal 后 0.5 s `/cmd_vel` | 归零 |
| Goal 后 0.5 s Ground Truth 速度 | 接近零 |
| 空旷转弯 Slowdown 触发 | 0 或有明确合理原因 |
| 原地旋转中心漂移 | ≤ 0.10 m/360° |
| RViz Navigation 状态 | 1 s 内正确 |
| Camera | 可清晰识别货架轮廓和地面线 |
| W+D | 稳定产生前进右转命令 |

最终项目统计门槛继续沿用总体目标：

- 静态避障率不低于 95%；
- 动态避障率不低于 90%；
- 路径与理论最优路径偏差不高于 20%；
- 动态异构环境导航成功率不低于 90%。

不得在只做 10 次功能测试后宣称最终统计指标达标。

---

## 17. 文档同步

实施过程中同步更新：

```text
README.md
docs/user_manual.md
docs/troubleshooting.md
docs/interfaces.md
docs/repository_index.md
docs/verification.md
docs/runtime_reliability_and_performance_upgrade_plan.md
docs/rviz_workflow_upgrade_plan.md
plan.md（仅补充引用或状态，不重写总体架构）
```

新增文档导航：

```text
导航质量与仿真一致性：
docs/navigation_quality_and_simulation_fidelity_upgrade_plan.md
```

Troubleshooting 必须包含：

- Navigation inactive 但目标正在执行；
- Motion BVH；
- `getSimulationTimeMonotonicAtTime`；
- wheel `customGeometry`；
- TGS velocity iterations；
- `/scan` QoS；
- Mapping costmap reset warning；
- Camera DLSS 模糊；
- powersave；
- 多工作区污染；
- Goal preemption 与不停车的区别。

---

## 18. 最终交付内容

Codex 完成后必须输出：

1. 修改摘要；
2. 根因表：已确认、已排除、仍待验证；
3. 逐文件修改清单；
4. Nav2 参数前后对照；
5. 底盘物理参数前后对照；
6. 相机参数前后对照；
7. Teleop 交互说明；
8. 所有复现命令；
9. 所有测试命令；
10. 自动测试结果；
11. Isaac 实际运行矩阵结果；
12. 导航质量指标；
13. 警告消除结果；
14. 未解决的上游警告；
15. 回滚方式；
16. 文档更新；
17. Commit/PR 信息。

### 18.1 禁止的最终表述

禁止在缺少证据时写：

```text
导航已经完全优化
机器人已经没有漂移
所有警告已解决
Camera 已达到 30 Hz
动态避障率已达到 90%
```

应写为：

```text
在已执行的 N01-N10 测试矩阵中……
在 Ideal/Realistic 的具体样本数量下……
仍未验收的边界包括……
```

---

## 19. 建议的最终 Goal 文本

可直接把下面内容作为当前 Codex Goal 的后续指令：

```text
继续当前 Goal，不要重新初始化项目，也不要推翻本对话已经完成并合并的架构。

请以新增文档
docs/navigation_quality_and_simulation_fidelity_upgrade_plan.md
为本轮唯一专项执行计划。先把我提供的完整方案写入该文件，再检查当前 main、工作树、最近提交和现有三份计划文档的实施状态。

本轮重点不是再次整理项目，而是基于真实运行现象，系统修复：

1. 空旷环境导航实际轨迹弯曲、回绕和反复纠偏；
2. 转弯、原地旋转和倒车过慢；
3. 零线速度、纯角速度时四轮 skid-steer 车体明显漂移；
4. 目标附近继续运动或停车不稳定；
5. RViz Navigation 显示 inactive，但目标和 Feedback 实际 active；
6. RTX LiDAR Motion BVH 未启用；
7. getSimulationTimeMonotonicAtTime 持续警告；
8. wheel collision obsolete customGeometry；
9. TGS velocity iterations 警告；
10. Mapping 模式误查询 Costmap Reset Service；
11. Camera DLSS/失焦/曝光导致严重模糊；
12. Mapping Teleop 不支持 W+D 等组合键；
13. Joint State、TF、LiDAR、Timeline 等弃用接口；
14. MPPI、Rotation Shim、Velocity Smoother、Costmap、Collision Monitor、Goal Checker 和 Planner 的联合调优。

严格按方案阶段执行：
先建立基线和指标，再修时间/传感器，再修底盘动力学和有效轮距，再新增 quality/quality_bidirectional Nav2 profile，再修 RViz 状态、组合键 Teleop、Camera 和弃用接口。

保留 stable/performance profile，不得直接覆盖基线。
保留 Activation Gate 为唯一 Lifecycle owner。
保留 map->odom->base_link、Ideal/Realistic、四种 operation、Map Manifest、Reset epoch、单实例锁和有序关闭。
不得修改 NVIDIA 官方资产源文件。
所有物理修复通过项目 Overlay、配置和代码完成。
所有参数选择必须有前后对照、实际运行证据和回滚入口。

必须执行：
./scripts/build_ros2.sh
./scripts/preflight.sh
./scripts/test.sh --with-isaac

并完成方案中的 N01-N10 导航矩阵、SimplePlane/Warehouse skid-steer A/B、Goal 后三级 cmd_vel 归零、Collision Monitor 状态、Camera 清晰度、RViz Lifecycle 状态、组合键 Teleop 和警告清理验证。

实施过程中持续回填新方案文档、verification.md、user_manual.md、troubleshooting.md、interfaces.md、repository_index.md 和 README.md。

最终提交和推送前，给出：
根因表、逐文件修改、参数前后对照、测试结果、运行矩阵、导航质量指标、剩余边界和回滚方式。
不要把单元测试通过等同于真实导航效果验收。
```

---

## 20. 2026-07-14 强制补充：Warehouse V2 长距离静态/动态避障、RViz 退出与 Local Plan 可视化

> 本节是在前述方案基础上的增量补充。前述所有内容、约束、阶段划分、测试要求和文档职责保持不变。本节新增要求优先级高于此前仅覆盖短距离、空旷环境或单一目标的简化导航测试。

### 20.1 新增问题背景

现有导航验证仍然过于简单，主要集中在短距离、少障碍或空旷环境中的单目标测试。这类测试可以验证 Topic、TF、Action、Lifecycle 和基本控制链是否连通，但不能充分验证以下参数在真实长距离导航中的联合效果：

- SLAM Toolbox 定位与扫描匹配参数；
- `map -> odom` 连续性与长期漂移；
- SmacPlanner2D 全局规划参数；
- Global Costmap 与 Local Costmap 参数；
- MPPI 预测窗、采样范围、Critic 权重和运动模型；
- Rotation Shim 起步转向与终点转向参数；
- Velocity Smoother 加减速与曲率保持参数；
- Collision Monitor Stop/Slowdown/Approach 区域；
- Progress Checker、Goal Checker 和恢复行为；
- 动态障碍触发后的减速、停车、绕行、重规划和恢复；
- 长时间运行下 `/clock`、TF、Scan、Odom、Costmap 和 Controller 周期稳定性。

短距离成功不能证明导航参数合理。某组参数可能在 3 m 空旷直线目标上成功，但在长走廊、连续转弯、窄通道、静态临时障碍和动态横穿障碍中出现：

- 路径振荡；
- 频繁急停；
- 转向速度过低；
- 局部规划反复回绕；
- 障碍物附近原地犹豫；
- 重新规划不及时；
- 终点附近反复调整；
- 到达后速度不能稳定归零；
- SLAM Toolbox 定位漂移或瞬时跳变；
- Collision Monitor 长时间误限速；
- Controller 超时或漏周期。

因此，本轮必须将 `warehouse_v2` 作为导航质量的主要验收场景，并新增可重复、可统计、可回归的长距离静态与动态避障测试。`warehouse_v1` 和原有 3 m 测试只保留为 Smoke Test，不再作为导航质量最终验收依据。

### 20.2 Warehouse V2 资产与地图契约

#### 20.2.1 启动前必须确认

Codex 必须先检查当前仓库中 `warehouse_v2` 的真实状态，不得假定它已经完整可用。必须确认：

1. Occupancy Map YAML 存在；
2. PGM/PNG 地图图像存在；
3. Pose Graph 主文件存在；
4. Pose Graph `.data` 文件存在；
5. Manifest 存在；
6. 所有文件不是 Git LFS 指针；
7. Manifest 中的 size、SHA256、bundle hash 和地图元数据匹配；
8. `warehouse_v2` 已完成 Spawn Pose 与 Map Pose 标定；
9. `initial_pose_source:=auto` 只有在 Manifest 明确标记为已标定时才允许；
10. 未标定时必须使用 `initial_pose_source:=rviz`，不得静默套用 `warehouse_v1` 的标定结果。

如果 `warehouse_v2` 目前只存在本地工作树、运行输出目录或未追踪文件中，必须先明确记录其路径、来源和是否应纳入 Git/LFS，不得直接覆盖现有 `warehouse_v1`。

#### 20.2.2 物理场景与地图一致性

长距离测试必须区分两类静态障碍：

**A. 地图内静态障碍**

- 障碍物已经存在于 `warehouse_v2` 的物理场景和 Occupancy Map 中；
- 用于验证 SLAM Toolbox 定位、SmacPlanner2D 全局路径选择、Global Costmap、路径平滑和长距离执行；
- 地图与物理场景必须一致，不能使用旧地图测试已永久改变的场景。

**B. 地图外临时静态障碍**

- 障碍物只在运行时通过项目 Overlay 或场景生成器加入物理场景；
- 不写入静态 Occupancy Map；
- 用于验证 LaserScan、Obstacle Layer、Local/Global Costmap 动态标记、局部绕行和必要时的全局重规划；
- Reset 后必须确定性恢复到相同位置；
- 不得修改 NVIDIA 官方 Warehouse 源 USD。

两类测试必须分别统计，不能把“地图中已有的障碍物”与“运行时新出现的障碍物”混为一个指标。

### 20.3 新增长距离静态避障测试

#### 20.3.1 路线设计原则

必须根据 `warehouse_v2` 可通行区域自动或半自动选取长距离起终点对。路线应覆盖尽可能多的地图结构，而不是只选择最容易成功的直线。

目标路线应覆盖：

- 长直走廊；
- 90° 左转；
- 90° 右转；
- 连续 S 形转弯；
- 货架之间的窄通道；
- 需要先旋转再直行的目标；
- 目标位于机器人后方的情况；
- 需要绕过地图内固定障碍的情况；
- 需要绕过地图外临时障碍的情况；
- 原全局路径被阻断后需要重新规划的情况；
- 接近终点后需要调整最终朝向的情况。

优先选择 15–30 m 的实际路径长度。若 `warehouse_v2` 的可通行尺寸无法满足 15 m，则使用地图中可实现的最长无重复主路线，并在报告中明确实际长度。禁止通过让机器人原地绕圈来人为增加距离。

#### 20.3.2 静态障碍布置

至少建立以下可复现静态场景：

| 场景编号 | 场景内容 | 主要验证对象 |
|---|---|---|
| WS01 | Warehouse V2 原始地图长距离导航 | SLAM Toolbox、Smac、MPPI 基线 |
| WS02 | 长走廊中央单个箱体 | 临时障碍标记、局部绕行 |
| WS03 | 两个交错箱体形成 S 形通道 | MPPI 曲率、路径平滑、局部 Costmap |
| WS04 | 通道局部收窄但仍满足安全宽度 | Footprint、Inflation、Collision Monitor |
| WS05 | 原全局路径完全阻断，但存在另一条通路 | Global Costmap 更新、Smac 重规划 |
| WS06 | 转角外侧放置障碍 | PathAlign、PathAngle、转弯减速 |
| WS07 | 目标附近放置非阻断障碍 | Goal Critic、Goal Checker、终点停车 |
| WS08 | 左右镜像障碍布置 | 机器人与参数左右对称性 |

所有障碍物必须：

- 使用项目自有 USD/Prim 或运行时生成几何体；
- 具有明确 Collider；
- 具有明确物理材质或静态刚体属性；
- 不穿透地面；
- 不与货架、墙体或出生点重叠；
- 位置、尺寸、朝向和场景编号写入配置；
- 支持固定 Seed 和 Reset；
- 在测试报告中保存障碍物布局摘要。

#### 20.3.3 静态避障统计验收

最终静态避障正式验收不得少于 **100 次独立运行**，并满足：

- 无碰撞到达次数 / 总运行次数不低于 95%；
- 导航 Action 成功率不低于 95%；
- 每个核心静态场景都必须有样本，不能只重复最简单场景；
- 同一起终点的实际执行路径长度相对基准最优路径偏差不高于 20%；
- 到达后 1.0 s 内三级速度链稳定归零；
- 到达后不得继续产生持续位移或持续旋转；
- 不允许通过把速度限制得极低来换取成功率；
- 不允许禁用 Collision Monitor 来换取成功率；
- 不允许删除复杂场景或只统计成功样本。

建议分两阶段执行：

1. 调参阶段：每个场景至少 3–5 次，用于定位问题；
2. 冻结参数后的正式验收阶段：总计不少于 100 次，使用冻结配置重新运行，不得混入调参过程中的选择性样本。

### 20.4 新增长距离动态避障测试

#### 20.4.1 动态障碍必须具有真实时间过程

动态障碍不能只在某一帧 Teleport 到路径上。必须具有连续位置、速度和方向，并由统一的仿真时间驱动。

动态障碍必须满足：

- 轨迹由配置文件定义；
- 支持固定 Seed；
- Reset 后回到确定初始状态；
- 运动时间戳与 `/clock` 一致；
- 不通过墙体和货架；
- 不瞬移；
- 不在机器人已经无法制动的距离内无条件生成；
- 可记录真实位置和速度；
- 场景结束后可自动恢复或循环；
- 不修改 NVIDIA 官方资产。

#### 20.4.2 动态场景矩阵

至少建立以下动态场景：

| 场景编号 | 场景内容 | 主要验证对象 |
|---|---|---|
| WD01 | 单个障碍横穿长走廊 | Collision Monitor、减速与绕行 |
| WD02 | 单个障碍迎面接近后错身 | 局部重规划、速度控制 |
| WD03 | 障碍同向低速移动 | 跟随、减速、选择绕行时机 |
| WD04 | 两个障碍错时横穿 | MPPI 多时刻预测、振荡抑制 |
| WD05 | 转角盲区出现障碍 | Scan 更新、紧急减速、恢复 |
| WD06 | 临时阻塞后移开 | 等待与重新起步，不长期卡死 |
| WD07 | 左右镜像横穿 | 左右对称性 |
| WD08 | 长距离路线中连续两次动态交互 | 长时间稳定性、重复恢复 |
| WD09 | 动态障碍迫使全局路线改变 | Global Costmap 与 Smac 重规划 |
| WD10 | 目标附近动态障碍经过 | 终点到达、停车与安全边界 |

动态障碍速度应覆盖低速、中速和较高速三个等级。具体数值必须结合机器人最大速度、传感器更新率和可制动距离确定，并写入配置与报告。不得在没有制动距离分析的情况下随意提高障碍速度。

#### 20.4.3 动态避障统计验收

最终动态避障正式验收不得少于 **100 次独立运行**，并满足：

- 无碰撞到达次数 / 总运行次数不低于 90%；
- 动态异构场景总体导航成功率不低于 90%；
- 每个核心动态场景都必须有样本；
- 不得把动态障碍没有实际进入机器人路径的样本计为有效动态避障样本；
- 不得把长时间停止直到测试超时视为安全成功；
- Collision Monitor 触发必须可解释，不能长期处于误 Stop/Slowdown；
- 障碍离开后机器人必须在合理时间内恢复导航；
- 到达目标后速度必须稳定归零；
- 统计中必须区分碰撞、超时、规划失败、控制失败、定位失败、传感器断流和安全锁死。

### 20.5 长距离导航必须采集的指标

每次运行至少记录：

#### 20.5.1 任务结果

- 场景编号；
- Seed；
- 起点与目标点；
- 目标最终朝向；
- Action 最终状态；
- 成功、碰撞、超时、取消或恢复失败；
- 总仿真时间；
- 总墙钟时间；
- 实际 RTF。

#### 20.5.2 路径与到达质量

- 全局路径长度；
- 实际历史轨迹长度；
- 路径长度比；
- 相对基准最优路径偏差；
- 最大横向跟踪误差；
- 平均横向跟踪误差；
- 终点位置误差；
- 终点朝向误差；
- Goal Checker 首次满足时间；
- 到达后稳定停车时间；
- 到达后残余线速度和角速度；
- 到达后 1 s 内累计额外位移和角度。

#### 20.5.3 平滑性与“丝滑”指标

“丝滑”不能只凭截图或主观感受判断。至少记录：

- 线速度峰值、均值和标准差；
- 角速度峰值、均值和标准差；
- 线加速度与角加速度；
- 线速度和角速度的离散 jerk；
- 速度符号切换次数；
- 角速度符号切换次数；
- 每米路径的急停次数；
- 每米路径的明显反向修正次数；
- 局部路径曲率连续性；
- Controller 输出振荡次数；
- Recovery 次数；
- 重新规划次数；
- Stop、Slowdown、Approach 的触发时长与次数。

必须设定明确阈值或至少提供基线与优化后对照。不能只写“看起来更平滑”。

#### 20.5.4 安全指标

- 最小障碍距离；
- 静态障碍最小净空；
- 动态障碍最小净空；
- 碰撞事件；
- Collision Monitor 状态；
- 紧急停车次数；
- 误停车次数；
- 障碍离开后的恢复延迟。

#### 20.5.5 定位与时序指标

- `map -> odom` 平移和 yaw 连续性；
- 定位跳变次数；
- Ground Truth 与估计位姿误差；
- `/scan` Hz 与 Age；
- `/odom` Hz 与 Age；
- TF Lag；
- `/clock` 单调性；
- Controller 实际周期；
- Planner 实际周期；
- Costmap 更新时间；
- `getSimulationTimeMonotonicAtTime` 警告数量；
- Message Filter 丢包数量；
- 控制循环 missed deadline 数量。

### 20.6 参数验证必须从“单参数可运行”升级为“联合效果验证”

#### 20.6.1 SLAM Toolbox

必须在 `warehouse_v2` 长距离路线中验证：

- localization mode 的扫描匹配稳定性；
- `map -> odom` 是否连续；
- 长走廊中的退化定位；
- 转角后的重定位；
- 动态障碍对扫描匹配的影响；
- Scan throttling、Ceres 线程数和 transform timeout；
- Reset 后重新播发 Initial Pose 的稳定性；
- Ground Truth 与定位估计误差；
- 长时间运行是否出现累计漂移。

如果要调整 SLAM Toolbox 参数，必须保持 Occupancy Map 与 Pose Graph 所有权契约不变，并分别报告定位改善和计算负载变化。

#### 20.6.2 SmacPlanner2D

必须验证：

- 长距离全局路径是否选择合理通路；
- 地图内障碍绕行；
- 地图外临时障碍造成全局阻断后的重规划；
- `cost_travel_multiplier` 对贴障与绕远的影响；
- `tolerance` 和 `use_final_approach_orientation` 对目标接近的影响；
- Smoother 对折线路径的改善；
- Planner Frequency 是否足够且不造成无意义高频重规划；
- 全局路径长度相对基准最优路径偏差是否满足 20% 指标。

#### 20.6.3 MPPI

必须在长距离静态与动态场景中联合验证：

- `time_steps`、`model_dt` 和真实控制周期；
- `batch_size` 与运行负载；
- `vx_std`、`wz_std` 与可达速度采样；
- 正向、倒车和原地转向策略；
- PathAlign、PathFollow、PathAngle、Goal、GoalAngle、Cost 和 Constraint Critic；
- `PreferForwardCritic` 是否造成不必要的慢速和绕行；
- 动态障碍附近是否出现左右振荡；
- 窄通道是否过度保守；
- 终点附近是否反复回绕；
- `/optimal_trajectory` 是否真实发布并可视化；
- Candidate Trajectories 只能用于短时诊断，不能作为 Local Plan 的替代。

#### 20.6.4 其他导航参数

还必须联合验证：

- Rotation Shim 的进入阈值、退出阈值、角速度和终点转向；
- Velocity Smoother 的 OPEN_LOOP/CLOSED_LOOP、`scale_velocities`、加减速度和 timeout；
- Local Costmap 尺寸与 MPPI 预测距离；
- Obstacle Layer 的障碍距离和 Raytrace 距离；
- Inflation Radius 与 Cost Scaling；
- Collision Monitor 的多边形、点数阈值和减速比例；
- Progress Checker 是否把合理等待误判为失败；
- Goal Checker 是否造成终点附近长期调整；
- Behavior Tree 与 Recovery 是否在动态障碍移开后恢复；
- Controller、Planner 和 Costmap 的频率是否满足实际运行，而不只是配置值。

### 20.7 参数调优方法与防止过拟合

不得一次性无记录地修改所有参数。必须：

1. 保存当前 `stable` 和 `performance` 基线；
2. 新增版本化实验 Profile；
3. 每次实验记录参数哈希或配置快照；
4. 先定位主因，再做小范围 Sweep；
5. 每次只改变一个参数组；
6. 使用相同场景、相同 Seed 做 A/B 对照；
7. 调参集与正式验收集分离；
8. 正式验收使用未参与调参选择的 Seed；
9. 不得针对一个起终点过拟合；
10. 不得仅依据单次成功保留参数；
11. 任何速度上限降低都必须同时报告耗时变化；
12. 任何安全区域缩小都必须重新验证最小净空和碰撞率。

建议新增：

```text
ros2_ws/src/robot_navigation/config/profiles/quality.yaml
ros2_ws/src/robot_navigation/config/profiles/quality_bidirectional.yaml
isaac_sim/configs/experiments/warehouse_v2_static_long.yaml
isaac_sim/configs/experiments/warehouse_v2_dynamic_long.yaml
isaac_sim/configs/experiments/warehouse_v2_routes.yaml
```

文件名可根据现有仓库规范调整，但必须保持配置分层、可回滚和可追踪。

### 20.8 自动化运行与报告

必须扩展 `robot_experiments`，使其支持：

- 场景配置；
- 起终点配置；
- 障碍物配置；
- 动态轨迹配置；
- Seed；
- 重复次数；
- 超时；
- Ideal/Realistic；
- Nav2 Profile；
- Camera Profile；
- 是否启用 Ground Truth；
- 自动 Reset；
- 等待 Activation Gate；
- 自动发送 Goal；
- 自动判断 Action、碰撞、定位和停车状态；
- 自动保存每次运行结果；
- 自动聚合静态/动态成功率；
- 自动生成参数前后对照报告。

建议提供统一入口，例如：

```bash
./scripts/run_navigation_matrix.sh \
  --map warehouse_v2 \
  --scenario-set static_long \
  --nav2-profile quality \
  --mode ideal \
  --repetitions 100

./scripts/run_navigation_matrix.sh \
  --map warehouse_v2 \
  --scenario-set dynamic_long \
  --nav2-profile quality \
  --mode ideal \
  --repetitions 100
```

实际脚本名可遵循现有仓库约定，但必须支持非交互运行、失败不中断整批、每次运行有唯一 ID、可从中断处继续，并在末尾生成汇总。

运行输出不得直接污染 Git 工作树。原始运行数据、CSV/JSON、截图和日志应写入受忽略的 reports/runtime 目录；只有冻结后的摘要和必要证据索引进入 `docs/verification.md`。

### 20.9 RViz 在终端退出后仍残留

#### 20.9.1 现象

执行 `run_ros.sh` 启动 Mapping、Localization 或 Navigation 后，在对应终端按 `Ctrl+C` 或终端进程结束，ROS 主 Launch 已退出，但 RViz 窗口仍然存在，未随受管运行会话关闭。

#### 20.9.2 这不是可接受行为

当前项目宣称使用受管进程组和有序 Lifecycle Shutdown，因此 RViz 必须属于同一运行会话。终端退出后残留 RViz 会导致：

- 下次启动出现重复 RViz；
- 旧 RViz 继续订阅 Topic；
- QoS 和性能测试受到污染；
- 用户误以为新旧会话属于同一系统；
- 进程锁与实际进程状态不一致；
- 自动测试无法确认会话已完整结束。

#### 20.9.3 必须调查的原因

Codex 必须检查：

- RViz 是否由 Launch 直接启动；
- RViz 是否被 `xterm`、`gnome-terminal`、shell wrapper 或 detached process 启动；
- RViz 是否进入 `run_ros.sh` 的受管进程组；
- Launch 收到 SIGINT 后是否等待 RViz；
- 自定义 RViz Panel 的线程或 Future 是否阻止退出；
- `run_ros.sh` 的 INT→TERM→KILL 是否只作用于部分子进程；
- `setsid`、process group、session ID 和 parent PID 是否符合设计；
- Mapping Teleop 和 RViz 是否采用了不同的终端托管方式；
- 退出时是否存在 ROS Context 已关闭但 Qt Event Loop 仍运行的情况。

#### 20.9.4 修复要求

- 正常一次 `Ctrl+C` 后先完成有序 Lifecycle Shutdown；
- 随后 RViz、Teleop、Launch 子进程和相关终端必须全部退出；
- 不应要求用户手工关闭 RViz；
- 不应依赖全局 `pkill rviz2`；
- 不得杀死其他项目的 RViz；
- 只清理当前受管会话中的进程；
- 超时后按 INT→TERM→KILL 有界升级；
- 清理 PID/lock/SHM 时必须有进程归属证明；
- RViz 自定义 Panel 析构必须有界等待，不得造成 SIGABRT 或永久卡住。

#### 20.9.5 验收

至少连续执行 10 轮：

```text
启动 Navigation → 等待 Active → 打开 RViz → 发送或不发送目标 → Ctrl+C → 完整退出
```

每轮结束后必须满足：

```bash
pgrep -af 'rviz2|robot_rviz_plugins|ros2 launch'
```

不包含本轮残留进程；项目实例锁被释放；下次启动不报告重复实例。

### 20.10 RViz Local Plan 不显示，但 MPPI Candidate Trajectories 可显示

#### 20.10.1 现象

在 Navigation RViz 中：

- 勾选 `MPPI Candidate Trajectories` 后可以看到候选轨迹；
- `Local Plan` 长期没有显示；
- 当前预期的 Local Plan Topic 是 `/optimal_trajectory`；
- Candidate Trajectories 使用 `/trajectories`；
- 两者不是同一个数据源，候选轨迹不能代替最终局部最优轨迹。

#### 20.10.2 必须执行的运行时检查

必须在目标处于执行状态时检查：

```bash
ros2 topic info /optimal_trajectory --verbose
ros2 topic echo /optimal_trajectory --once
ros2 topic hz /optimal_trajectory
ros2 topic info /trajectories --verbose
ros2 topic echo /plan --once
ros2 lifecycle get /controller_server
```

需要记录：

- `/optimal_trajectory` 是否存在 Publisher；
- Publisher 节点名称；
- Publisher QoS；
- RViz Subscription QoS；
- 消息 `frame_id`；
- Pose 数量；
- 时间戳；
- 发布频率；
- 目标结束后 Topic 停止发布是否符合预期；
- Fixed Frame 为 `map` 时，是否存在从消息 Frame 到 `map` 的有效 TF；
- MPPI `visualize` 是否启用；
- `controller_server` 是否 Active；
- 当前加载的 YAML 是否为源码最新版本，而不是旧 `install/` 或其他 Overlay。

#### 20.10.3 可能原因必须逐项排除

- `/optimal_trajectory` 根本没有发布；
- MPPI 只发布 `/trajectories`，但当前版本或参数未启用最优轨迹发布；
- RViz Local Plan 订阅 Topic 名称错误；
- RViz QoS 与 Publisher 不兼容；
- 消息 Frame 为 `odom`，但 RViz 在消息时间无法得到 `map -> odom`；
- 消息时间戳超前或过旧，被 RViz Message Filter 丢弃；
- Buffer Length、Alpha、Line Width 或 Enabled 状态不正确；
- 目标尚未执行或已经结束，因此 Topic 没有新消息；
- 当前运行加载了旧的 `navigation.rviz`；
- `AMENT_PREFIX_PATH` 混入其他工作区；
- `visualize`、Lazy Publisher 或 Subscriber 发现存在启动竞态；
- 自定义 Profile 覆盖了 MPPI 可视化参数；
- `/optimal_trajectory` 消息 Pose 数为零或存在非法值。

#### 20.10.4 修复要求

- `Local Plan` 必须显示 MPPI 最终选中的局部最优轨迹；
- Topic 和消息类型必须在接口文档中冻结；
- RViz QoS 必须与实际 Publisher 匹配；
- RViz 配置必须来自当前工作区；
- Local Plan 和 Candidate Trajectories 必须使用明显不同的名称、颜色和线宽；
- Candidate Trajectories 默认关闭；
- Local Plan 默认开启；
- 若 `/optimal_trajectory` 是版本相关接口，必须建立兼容层或明确固定当前 Isaac/ROS/Nav2 版本下的实现；
- 不得通过把 Candidate Trajectories 重命名为 Local Plan 来规避问题；
- 自动测试至少验证 RViz 配置中的 Topic、QoS 和默认 Enabled 状态；
- 运行测试必须验证活跃目标期间 `/optimal_trajectory` 存在非空消息。

#### 20.10.5 验收

在长距离静态和动态场景中：

- Global Plan 可见；
- Local Plan 可见且持续更新；
- Candidate Trajectories 默认关闭；
- Local Plan 的曲线随机器人运动更新；
- Local Plan 消息在控制器活跃时具有合理 Pose 数；
- Local Plan 不因 QoS 或 TF 问题间歇消失；
- Goal 成功或取消后停止更新属于正常行为，但 RViz 不应保留误导性的无限历史轨迹。

### 20.11 对现有测试矩阵的修改

此前 N01–N10 矩阵继续保留，用作底层运动、基础目标和回归 Smoke Test；新增：

| 编号 | 测试 | 说明 |
|---|---|---|
| N11 | Warehouse V2 原始长距离路线 | 无新增障碍，验证基础长距离导航 |
| N12 | Warehouse V2 地图内复杂静态路线 | 验证 Smac 与定位 |
| N13 | Warehouse V2 临时静态障碍长距离 | 验证在线避障与重规划 |
| N14 | Warehouse V2 窄通道与 S 形路线 | 验证 Footprint、Inflation、MPPI |
| N15 | Warehouse V2 动态横穿长距离 | 验证减速、绕行和恢复 |
| N16 | Warehouse V2 多动态障碍长距离 | 验证连续动态交互 |
| N17 | Warehouse V2 目标附近动态障碍 | 验证到达与稳定停车 |
| N18 | Local Plan 可视化契约 | 验证 `/optimal_trajectory`、QoS、TF 和 RViz |
| N19 | RViz 受管退出 10 轮 | 验证进程组和有序关闭 |
| N20 | 静态 100 次正式统计 | 验证静态避障与路径指标 |
| N21 | 动态 100 次正式统计 | 验证动态避障与导航成功率 |

N20、N21 只能在参数冻结后执行。调参过程中的样本不得混入正式验收统计。

### 20.12 文档与证据回填

本节完成后必须更新：

- `docs/navigation_quality_and_simulation_fidelity_upgrade_plan.md`：持续回填实施状态；
- `docs/verification.md`：记录 N11–N21 的命令、样本数和结果；
- `docs/user_manual.md`：增加 Warehouse V2 长距离测试操作；
- `docs/troubleshooting.md`：增加 RViz 残留与 Local Plan 不显示排障；
- `docs/interfaces.md`：冻结 `/optimal_trajectory`、场景配置和报告接口；
- `docs/repository_index.md`：列出新增场景、脚本、配置和测试文件；
- `README.md`：只增加简洁入口，不复制全部专项方案；
- 相关配置和测试：覆盖场景 schema、参数 profile、RViz QoS、进程退出和报告聚合。

### 20.13 本补充的最终验收标准

只有同时满足以下条件，才能写“Warehouse V2 长距离导航质量已通过本轮验收”：

1. `warehouse_v2` 四工件和 Manifest 合法；
2. 地图与物理场景的永久静态结构一致；
3. 至少 100 次静态正式样本，静态避障率不低于 95%；
4. 至少 100 次动态正式样本，动态避障率和导航成功率不低于 90%；
5. 路径长度偏差不高于 20%；
6. 到达目标后三级速度链和真实底盘均稳定停止；
7. 长距离路线中没有持续轨迹回绕、频繁振荡或不合理低速；
8. Collision Monitor 不发生长期误限速；
9. SLAM Toolbox 定位与 `map -> odom` 在长时间运行中保持稳定；
10. Global Plan 和 Local Plan 均可在 RViz 中正确显示；
11. `/optimal_trajectory` 在活跃目标期间有非空、可变换的消息；
12. 终端退出后 RViz 不残留；
13. 所有结果有原始报告、参数快照、场景编号、Seed 和运行命令；
14. 未通过的场景必须保留并如实报告，不能从统计中删除。

---

## 21. 更新后的最终 Goal 文本

可直接把下面内容作为当前 Codex Goal 的后续指令。该文本替代第 19 节中的旧版本 Goal 文本，但第 19 节原文保留作为历史记录：

```text
继续当前 Goal，不要重新初始化项目，也不要推翻本对话已经完成并合并的架构。

请以新增文档

docs/navigation_quality_and_simulation_fidelity_upgrade_plan.md

为本轮唯一专项执行计划。先确认该文件已经包含 2026-07-14 新增的 Warehouse V2 长距离静态/动态避障、RViz 受管退出和 Local Plan 可视化补充，再检查当前 main、工作树、最近提交和现有三份计划文档的实施状态。

本次请求是一个完整、不可拆分、一次性执行到底的 Codex Goal。下面十三个阶段只是同一 Goal 内部的依赖顺序，不是十三次独立 Goal，也不是逐阶段等待用户确认的检查点。必须在一次 Goal 执行中自动连续完成第一至第十三阶段；完成某一阶段后，只在内部记录结果、完成必要的测试和中间提交，然后立即进入下一阶段。不得停止、暂停、挂起、结束当前执行轮次或把执行权交还给用户，不得触发 Goal 模式的阶段完成暂停、检查点暂停、handoff、yield 或等待恢复状态，不得询问我是否继续，不得要求我再次发送“继续”或 `/goal resume`，不得把阶段总结当作最终答复，也不得把剩余阶段改写为后续任务、建议或待办事项。

若测试失败、指标未达到、警告未消除或出现回归，必须留在同一 Goal 中继续诊断、修改、重建、复测和迭代，直到通过该阶段的退出条件后自动进入下一阶段。允许创建中间 Commit、阶段性报告和临时实验 Profile，但完成这些动作后必须在同一连续执行中继续，不得因此暂停 Goal 或等待 `/goal resume`。只有以下两种情况允许结束本 Goal：

1. 第一至第十三阶段、N01-N21 全部适用测试、正式统计、文档回填、最终提交和推送全部完成，并输出唯一一次最终总报告；
2. 出现必须由用户提供凭据、执行硬件操作、补充不可获取外部资产或处理平台强制中断的真实外部阻塞，且已经穷尽仓库内可行的替代方案，并提供完整证据、已完成项、未完成项和最小解阻动作。

不得把“某一阶段完成”“单元测试通过”“某个场景成功”或“已给出下一阶段计划”作为停止条件。

本轮重点不是再次整理项目，而是基于真实运行现象，系统修复并验证：

1. 空旷环境导航实际轨迹弯曲、回绕和反复纠偏；
2. 转弯、原地旋转和倒车过慢；
3. 零线速度、纯角速度时四轮 skid-steer 车体明显漂移；
4. 目标附近继续运动或停车不稳定；
5. RViz Navigation 显示 inactive，但目标和 Feedback 实际 active；
6. RTX LiDAR Motion BVH 未启用；
7. getSimulationTimeMonotonicAtTime 持续警告；
8. wheel collision obsolete customGeometry；
9. TGS velocity iterations 警告；
10. Mapping 模式误查询 Costmap Reset Service；
11. Camera DLSS/失焦/曝光导致严重模糊；
12. Mapping Teleop 不支持 W+D、W+A、S+D、S+A 等组合键；
13. Joint State、TF、LiDAR、Timeline 等弃用接口；
14. MPPI、Rotation Shim、Velocity Smoother、Costmap、Collision Monitor、Goal Checker、SLAM Toolbox 和 SmacPlanner2D 的联合调优；
15. 现有 3 m、空旷和短距离测试过于简单，无法验证真实导航参数效果；
16. 使用 warehouse_v2 建立长距离、多转弯、窄通道和复杂静态障碍测试；
17. 在 warehouse_v2 中加入可重复的临时静态障碍，验证在线避障、全局重规划和局部绕行；
18. 在 warehouse_v2 中加入由仿真时间驱动的动态障碍，验证减速、停车、绕行、恢复和连续动态交互；
19. 冻结参数后完成不少于 100 次静态正式统计，静态避障率和导航成功率达到方案指标；
20. 冻结参数后完成不少于 100 次动态正式统计，动态避障率和动态导航成功率达到方案指标；
21. 实际执行路径相对基准最优路径的偏差不高于 20%；
22. 在完美到达目标容差范围的同时，实现平滑、连续、无明显振荡的静态与动态避障；
23. 终端退出后 RViz 仍残留，未被受管进程组关闭；
24. RViz 中 MPPI Candidate Trajectories 可以显示，但 Local Plan 不显示；
25. 必须恢复 `/optimal_trajectory` 的真实发布、QoS、TF 和 RViz 可视化契约，Candidate Trajectories 不得替代 Local Plan。

以下十三个阶段必须在同一个 Goal 中按顺序连续执行，全部完成前不得停止：

第一阶段：检查当前仓库、工作树、warehouse_v2 地图四工件、Manifest、LFS、Spawn/Map 标定和现有运行证据。不得假定 warehouse_v2 已经完整可用。

第二阶段：建立现有短距离基线、底层运动基线、三级 cmd_vel 基线、Local Plan Topic 基线和 RViz 进程树基线。

第三阶段：修复仿真时间、Motion BVH、传感器时间戳、轮胎 Collider、接触材料、Joint Axis、有效轮距和 skid-steer 动力学。完成 SimplePlane/Warehouse A/B。

第四阶段：修复 Camera、组合键 Teleop、弃用接口、Mapping 模式 Costmap Reset 误警告和 RViz Navigation 状态竞态。

第五阶段：保留 stable/performance，不直接覆盖基线；新增并版本化 quality/quality_bidirectional 或等价实验 Profile。联合调整 SLAM Toolbox、SmacPlanner2D、Rotation Shim、MPPI、Velocity Smoother、Local/Global Costmap、Planner、Goal Checker、Progress Checker、Behavior Tree 和 Collision Monitor。

第六阶段：建立 warehouse_v2 长距离路线配置。路线必须覆盖长走廊、左右转、连续转弯、窄通道、目标在身后、地图内障碍和地图外临时障碍。优先使用 15–30 m 实际路径；地图不足时使用最长合理路线并记录实际长度。

第七阶段：建立 warehouse_v2 静态场景矩阵。必须同时包含地图内静态障碍和运行时临时静态障碍，验证全局规划、在线 Costmap、局部绕行和重规划。

第八阶段：建立 warehouse_v2 动态场景矩阵。动态障碍必须连续运动、由 `/clock` 驱动、可 Reset、可固定 Seed、不可瞬移，并覆盖横穿、迎面、同向、盲区、临时阻塞、多障碍和目标附近动态交互。

第九阶段：扩展 robot_experiments 和脚本，实现非交互批量运行、自动 Reset、自动发送 Goal、自动记录碰撞/超时/定位/停车/路径/平滑性/安全/RTF/TF/Topic 指标，并生成逐次结果和汇总报告。

第十阶段：修复 RViz Local Plan。活跃目标期间必须验证 `/optimal_trajectory` 有 Publisher、非空消息、合理 Pose 数、正确 Frame、匹配 QoS、有效 TF 和持续更新。Local Plan 默认开启，Candidate Trajectories 默认关闭，不得用候选轨迹冒充局部最优轨迹。

第十一阶段：修复 RViz 受管退出。正常一次 Ctrl+C 后，按顺序完成 Lifecycle Shutdown，并关闭当前会话的 RViz、Teleop、Launch 子进程和终端。不得使用全局 pkill。连续 10 轮启动/退出后不得有残留 RViz 或实例锁。

第十二阶段：完成 N01-N21 测试矩阵。N01-N10 保留为 Smoke/底层回归；N11-N19 为 warehouse_v2 长距离、Local Plan 和退出专项；N20 为不少于 100 次静态正式统计；N21 为不少于 100 次动态正式统计。调参样本不得混入正式验收样本。

第十三阶段：参数冻结后重新执行全部正式统计。不得选择性删除失败样本，不得只重复简单场景，不得通过极低速度、禁用 Collision Monitor 或缩小安全距离换取成功率。

阶段衔接规则：每完成一个阶段，立即执行该阶段要求的构建、测试、真实运行验证和文档回填；若通过，必须在同一连续执行上下文中自动进入下一阶段，不得暂停 Goal、结束当前执行轮次或等待 `/goal resume`；若失败，留在当前阶段修复并重试。不得将任何阶段拆分成新的 Goal、后续对话任务、待办事项或需要用户再次确认的操作。阶段性汇报只能作为内部进度记录，不能作为结束当前 Goal、触发阶段暂停或要求恢复执行的答复。

保留以下架构约束：

- 保留 stable/performance profile，不得直接覆盖基线；
- 保留 Activation Gate 为唯一 Lifecycle owner；
- 保留 map->odom->base_link、Ideal/Realistic、四种 operation、Map Manifest、Reset epoch、单实例锁和有序关闭；
- 不得修改 NVIDIA 官方资产源文件；
- 所有物理修复通过项目 Overlay、配置和代码完成；
- 地图内永久静态结构必须与 warehouse_v2 Occupancy Map 一致；
- 运行时临时静态/动态障碍必须通过项目配置与场景生成器管理；
- 所有参数选择必须有前后对照、实际运行证据、场景编号、Seed、配置快照和回滚入口；
- 不得把单元测试通过等同于真实导航效果验收；
- 不得把单次成功等同于达到统计指标；
- 不得在没有真实运行证据时声称导航、动态避障、Camera、漂移或警告已经完全解决。

必须执行：

./scripts/build_ros2.sh
./scripts/preflight.sh
./scripts/test.sh --with-isaac

并完成：

- N01-N21 全部适用测试；
- SimplePlane/Warehouse skid-steer A/B；
- Goal 后 `/cmd_vel_nav`、`/cmd_vel_smoothed`、`/cmd_vel` 与真实底盘速度归零；
- Collision Monitor 状态与误限速统计；
- Camera 清晰度和实际频率；
- RViz Lifecycle 状态；
- W+D/W+A/S+D/S+A 组合键 Teleop；
- `/optimal_trajectory` Local Plan 发布与 RViz 显示；
- RViz 受管退出连续 10 轮；
- warehouse_v2 长距离静态正式统计不少于 100 次；
- warehouse_v2 长距离动态正式统计不少于 100 次；
- 静态避障率不低于 95%；
- 动态避障率和动态导航成功率不低于 90%；
- 路径长度偏差不高于 20%；
- 到达后稳定停车；
- 平滑性、振荡、急停、恢复、最小净空、定位误差和时序指标报告。

实施过程中持续回填：

- docs/navigation_quality_and_simulation_fidelity_upgrade_plan.md
- docs/verification.md
- docs/user_manual.md
- docs/troubleshooting.md
- docs/interfaces.md
- docs/repository_index.md
- README.md

全部十三个阶段、全部必需测试、正式统计和文档回填均完成后，执行最终提交和推送，并在唯一的最终答复中给出：

1. 根因表；
2. 逐文件修改清单；
3. 参数前后对照；
4. warehouse_v2 地图和场景资产说明；
5. 静态/动态场景矩阵；
6. 每个测试的命令、Seed 和样本数；
7. 静态与动态成功率；
8. 碰撞、超时、定位失败、规划失败、控制失败和安全锁死分类；
9. 全局路径、Local Plan、实际轨迹和最优路径对照；
10. 到达误差、停车时间和残余速度；
11. 平滑性、振荡、急停和恢复指标；
12. Collision Monitor 触发统计；
13. SLAM Toolbox、SmacPlanner2D、MPPI 和其他导航参数的选择依据；
14. RViz Local Plan 修复证据；
15. RViz 连续 10 轮无残留退出证据；
16. 警告消除结果与未解决的上游警告；
17. 剩余边界；
18. 回滚方式；
19. Commit/PR 信息。

不要把“能够跑到目标”当作导航质量完成。最终目标是在 warehouse_v2 的长距离复杂路线中，在满足安全和统计指标的前提下，机器人能够准确到达目标、稳定停车，并平滑、连续地完成静态与动态避障。

在上述全部内容完成、验证、回填、提交并推送之前，不得结束、暂停或挂起本 Goal，不得在阶段边界把执行权交还给用户，也不得输出要求用户再次输入“继续”或 `/goal resume` 的阶段性最终答复。

---

## 2026-07-14–15 连续执行台账（当前工作树）

- 第一阶段审计：`warehouse_v2` 四工件/Manifest/LFS 完整性已验证，但
  `runtime_alignment_verified=false`、`calibrated=false`，因此仍不可用于自动播种
  或正式统计；该边界保持不变。
- 第二阶段底层基线：Warehouse + Ideal 改动前 14 段已保留；本轮又完成项目标准
  Cylinder 下 32/4 与 32/16 各 14 段隔离 A/B，并新增 Isaac 启动 provenance，
  后续报告可以绑定实际输入而不再只哈希 motion YAML。短距离 Nav2、三级 cmd_vel、
  Local Plan 与 RViz 进程树的既有基线仍按验证台账保留。
- 第三阶段当前进度：四个 obsolete 官方轮 collider 已由项目 Overlay 停用并替换为
  对称标准 Cylinder；真实 180 步启动中 `customGeometry` 警告为 0。32/4 与 32/16
  保持直行/停车等价，32/4 的 high-tier 左右旋转更高、Reset latency 无 16 的
  9.07 秒离群点，且 TGS 警告从 1 降为 0，因此冻结 solver `32/4`。SimplePlane/
  Warehouse 历史接触矩阵已完成；其旧 motion report 没有稳态窗口，当前物理 gate
  verdict 为 N/A，不能沿用旧整段均值写成通过或失败。低速左右转向不对称、接触拓扑、
  有效轮距和 Realistic A/B 尚未闭合，第三阶段仍未退出。
- 第三阶段时间进度：确认安装版 ROS 2 helper 默认
  `resetSimulationTimeOnStop=true`；项目此前显式 false 会选择 monotonic 查询，
  且本轮日志中该查询与相邻 Fabric 样本缺失同时出现。false 的 30 分钟基线三类
  计数为 `93 / 0 / 93`；改为 true 后，
  Camera Off、Monitoring 短窗及 900 秒 headless soak 均为 `0 / 0 / 0`，长跑
  `/clock` 51130 样本、点云 8523 样本均无重复/回退。14 次事务 Reset 的
  Clock/Odom/JointState 也无回退，因此已采用 true；真正 Timeline Stop→Play 和
  GUI/headless × realtime/unbounded × 60/120 Hz 完整矩阵仍未完成。
- 第三阶段轮地审计：官方源与项目原始 Jackal SHA256 一致；四轮 Joint Axis、
  局部旋转、位置、Cylinder 轴和控制器 `[left,right]` 映射完全对称，静态证据不
  支持“某侧反轴”。几何轮距 `0.37559 m` 与 USD 轮心一致，但五份 32/4 报告拟合
  总体有效轮距约 `1.0124 m`，且低速左右非线性明显，不能只靠单一轮距修复。
  Warehouse 地面没有显式材质绑定，轮材质仅为 scalar `0.2/0.2`；公开通用刚体
  USD/API 未发现各向异性摩擦。Warehouse 的 friction correlation/offset 阈值
  `.00025/.0004` 又比安装版 schema fallback `.025/.04` 小 100 倍。现在已新增可逆
  `legacy/threshold-only/explicit-material` 匿名 session profile、独立 SimplePlane、
  精确 4 wheel/32 Warehouse ground collider 读回和 2×2 threshold 配置。clean
  schema v3 Warehouse 单轮正负诊断 8/8 通过，normal/contact/friction/body 方向硬门
  全部为真；clean motion 报告 14/14 通过并把 profile、overlay、scene、collider、
  binding、material 与 Git 身份绑定。clean commit `0500f9e` 上已经完成每个 threshold
  点、显式材质和 legacy 在 SimplePlane/Warehouse 的三次锁定输入重复：36/36 run、
  216/216 段、216/216 Reset 成功，12 个 group 各 3 次，分析纳入 36/排除 0；
  Manifest/analysis/summary SHA256 分别为 `f38bdbdf...b7df`、`e8095612...91ea`、
  `f22c0079...3185`。但空旷 SimplePlane 的 36 个纯旋转段中心漂移为
  `0.297486–0.350392 m`，旧整段平均角速度误差为 `60.10%–69.05%`；这些描述值
  超过当时的 `0.10 m/10%` 探索阈值，但报告为 schema 1，没有后半命令稳态窗口，
  所以当前 8.7 机器 verdict 为 N/A，不得写成正式 FAIL。六 Profile 对严重欠转均无
  决定性改善；P11 的约
  `1.012 m` 只作为多速度拟合初值，不能直接冻结。下一步必须先做两环境单轮重复、
  Warehouse GroundPlane/floor-decal collider 拓扑隔离、多角速度有效轮距及
  60/120 Hz、CCD、stabilization 单变量 A/B，之后再做 Ideal/Realistic 复验；
  第三阶段仍未退出。
- 第三阶段运动学契约进度：提交 `dd58c63` 完成零行为迁移。schema-v2 robot YAML
  现在是轮径、轮宽、几何/有效轮距、wheel joint 和质量的单一真源；Control Graph、
  Robot Description、Realistic Wheel Odom 与 contact 分析都经过 exact-key、SHA256、
  provenance v5 和启动握手约束。稳定有效轮距仍为 `0.37559 m`，只是保持旧控制值；
  约 `1.012 m` 尚未写回。11 包构建、preflight、root/ROS/Isaac 全门通过，但这些
  结构证据不代替多速度、两环境和 Realistic 物理 A/B，第三阶段仍未退出。
- 第三阶段候选入口：提交 `ab909b4` 新增不可变
  `jackal_etw_0p989_v1/1p012_v1`，分别取 clean 接触矩阵 Warehouse 候选均值及
  两环境等权均值的三位舍入。两者均为 `experimental_candidate`，与 stable 只差
  profile/lifecycle/有效轮距，URDF 字节等价；clean 11 包 build、preflight、
  root `907`、ROS `725`、Isaac/USD `21` 全门通过。提交 `4b55f90` 已给正式矩阵增加
  受信任的 `--robot-config FILE` 选择和 HEAD-blob 字节锁，但尚未用任一候选执行真实
  A/B，因此不能冻结或覆盖 `0.37559 m` stable。
- 第三阶段地面拓扑入口：提交 `6897712` 增加版本化、匿名层、可逆的
  `simple_plane_only1_v1`、`warehouse_combined32_v1` 与
  `warehouse_plane_only1_v1`；后者从同一 Warehouse 32-collider source 精确禁用 31
  个非 GroundPlane collider。随后 provenance schema v5、ROS live consumer 和严格
  analyzer 把 environment/topology/contact 拆成独立身份与分层 A/B 锁：global、
  environment、跨环境 profile、environment+topology、environment+contact 和最终三元组；Stage 应用、
  fresh readback、canonical hash、非法配对和完整 18 组统计合同均有自动测试。提交
  `a1056c3` 已让严格批处理按 `baseline/all/ID` 选择合法 pair：历史口径 36-run/12-group，
  全 topology 口径 54-run/18-group，并锁定 topology HEAD blob 与 schema-v5 证据。
  clean `a85828f` 的 Warehouse 32-vs-1 × 六 contact profile × 每格一次 12-run
  机制烟测暴露了 runtime-derived RootLayer 摘要的锁定作用域问题；该批 12 个进程
  虽完成，但严格 analyzer 按旧合同失败关闭，没有生成 analysis/summary，不能记为批次
  成功。修复后的 clean `d5840ed` 重跑已闭合 12/12 run、72/72 段及 144/144
  manifest 路径/hash 对，冻结根证据为 manifest
  `a31acc23fa5c4c029ba09b938d417a1ce08d053d4928e271631dcc816634bdcf`、analysis
  `3c39c1593dcdd5cf19d3678410bd734531ebf662ab5600bb588ec2d0301babc9`、summary
  `92e136a8cb968859b72976977a4c129d19813ba897c67807661d695543011852`。Root SHA
  按 combined32 六份、plane-only legacy/explicit/两个 `0.00025` profile 四份、
  plane-only 两个 `0.025` profile 两份形成 `6/4/2` 分布；Kit `[Error]` 为 0，但
  12 份 Isaac 日志各保留一条非致命 absl `E0000`，不得写成“零错误”。该批仍是
  repeat=1 的历史 schema 2/3 机制烟测。其 Warehouse、repeat=1 与旧 motion report
  schema 1 都使计划 8.7 物理判定为 N/A，不能记成 `0/12 fail`；正式每组三重复的
  54-run/18-group 全 topology 矩阵仍未运行，因此第三阶段仍未退出。
- 第三阶段物理机器门：motion report schema 2（configuration 仍为 1）已提供命令区间
  后半段 `steady_state_window`；v5 analysis schema 3、`physical_acceptance` schema 1/
  policy `skid_steer_plan_8_7_v1` 和 batch-summary schema 4 已实现。旋转门按该窗口实际
  angular-z mean 相对命令角速度的绝对误差比例判定，不再用 yaw gain。机器 verdict
  只适用于 provenance 5 + SimplePlane/only1 + Ideal + 每组至少 3 个唯一 repeat +
  motion report schema 2；其他组为 `passed=null` 的 N/A。summary 的证据 `success` 与
  `all_applicable_groups_passed` 及 applicable/not-applicable/passing/failed 四类 group
  必须分别读取。当前完整 analyzer 测试文件为 `116 passed`，motion baseline `66 passed`、
  matrix script `42 passed / 1 skipped`（缺少 `shellcheck`）；这些仍不是 clean commit
  全门。同一 dirty worktree 的 `./scripts/test.sh` 为 exit 0，root
  `1061 passed / 1 skipped / 34 deselected`，ROS 11 packages / 861 tests / 0 errors /
  0 failures / 1 skipped；clean commit `--with-isaac`、真实新 schema smoke 和正式
  54-run 实跑仍待完成。
- 第三阶段 Reset/证据审计：正式接触矩阵的 108 个 report/双日志哈希全部复验通过；
  216 次服务/恢复 latency 均值分别为 `0.1694/0.5427 s`，恢复期 Odom 线/角速度和
  轮速峰值远低于门。119 个 pre-boundary group 与 105 个 JointState receive 回退均
  被 coherent epoch 门拒绝，运动段内无时间戳回退。该 PASS 只属于证据链、Reset、
  停止和聚合合同，不得改写成底盘物理 PASS。
- 证据纪律：调参 JSON 继续由 Git 忽略，摘要、命令、报告 SHA256、失败样本和边界
  回填到 `docs/verification.md`；正式统计必须在 clean commit、冻结输入和独立输出
  集合上重跑，不混入上述调参样本。
- provenance 审查加固：项目环境现在有规范 `environment.id`；schema v2 solver
  只在有效 Stage 属性与初始化后 Articulation wrapper 的 USD 后端读回一致时发布，
  且文档明确这不是 PhysX 引擎内部状态直接读回。真实错误环境标签负向用例在
  创建运动 `/cmd_vel` publisher 前失败，正确 Warehouse + Ideal 再次完成 14/14；
  该历史运行的 24 次单调查询警告现已由上面的独立长时 A/B 定位，不能再当成
  当前 true 配置的警告计数。
