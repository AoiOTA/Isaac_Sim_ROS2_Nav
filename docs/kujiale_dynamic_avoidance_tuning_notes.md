# Kujiale 动态避障调优问题复盘

本文记录 `local_bypass` 单轮可视化调优中发现的问题、根因和当前处理方式。它是调优复盘，不替代正式动态 20 次验收。

## 当前可视化基线

- 场景：`kujiale_0026_A_to_B_door_open.usd`，出生点 `long_route_start_g1`。
- Case：`local_bypass`，`variant=3`，`seed=7443`。
- Actor：`0.40 × 0.40 × 1.00 m`，轨迹 `[-1.65,-0.20] -> [-0.85,-0.20]`，峰值速度 `0.80 m/s`、最大加速度 `1.60 m/s²`，到点后保持 `parked`。
- 触发门：机器人在 `x∈[-0.70,0.30]` 内北向越过 `y=-1.75`，且速度不低于 `0.20 m/s`。
- 导航：静态与动态避障 profile 共用 MPPI 15 Hz、2.0 s 预测范围、Velocity Smoother 60 Hz、`vx_max=1.20 m/s` 与 `wz_max=3.40 rad/s`。两者只在 RGB-D 障碍物生命周期上区分：静态低矮物体保留标准 VoxelLayer 标记；动态 actor 由局部 STVL 进行时序清除。

## 问题与修复

### 1. Actor 在机器人通过后才触发

最初只改动了 `y` 门限，但保留了过窄的横向条件 `x∈[-0.45,0.30]`。实测机器人越过 `y=-1.75` 时位于 `x≈-0.52`，因此未满足空间门，直到后续重新进入该横向窗口才触发。

处理：将局部 case 的横向窗口放宽为 `[-0.70,0.30]`。不要只看纵向阈值；每次调整 gate 后都应从 `ground_truth.csv.gz` 复核机器人越线时的 `(x,y)`。

### 2. 写入 0.80 m/s 但 actor 实际并未达到该速度

Actor 使用余弦缓入缓出轨迹。对于 `0.80 m` 的横移，若最大加速度仍为 `0.50 m/s²`，加速度约束会将峰值速度限制在约 `0.45 m/s`。

处理：当前使用 `speed=0.80`、`max_acceleration=1.60`。对当前 `0.80 m` 横移，该组合的理论持续时间约为 `1.57 s`，并能达到 `0.80 m/s` 峰值速度。

### 3. Actor 在门框/墙边看起来穿透

原轨迹上方会擦到静态占据带，运动方块会在视觉和碰撞上贴近墙体。

处理：轨迹中心移至 `y=-0.20`，使完整 `0.40 m` 方块及其 `0.05 m` 余量保持在预检自由带内。动态 actor 是运动学物体，配置轨迹必须独立避开静态碰撞体，不能依赖接触后修正。

### 4. G5 门框凹槽会卡住轮子

源 USD 在 G5 门框处有一块碰撞底板，顶面比相邻地面低约 `0.10 m`。地图把该处视为可通行，Nav2 会正常规划进入，但实体轮子可能掉进凹槽。

处理：`SceneComposer` 仅针对当前 Kujiale USD 在运行时覆盖层添加静态齐平碰撞补片 `/World/EnvironmentRepairs/kujiale_g5_doorway_floor_fill`，顶面 `z=0`。不修改原始 USD，也不修改静态地图；Isaac 重启后生效。

### 5. 动态避障时车体有台阶式顿挫

实测完整控制链后，顿挫并不是底盘执行能力不足：

- MPPI 的 `/cmd_vel_nav` 每周期都会改变候选最优解，线速度和角速度增量频繁换向。
- 60 Hz Velocity Smoother 能把这些目标变化转换成有界斜坡；Collision Monitor 在没有危险时原样转发，底盘里程计也能跟随最终命令。
- 在完整 Isaac、STVL 和 RViz 负载下，原 `500 × 40` 的 MPPI 更新耗时中位数约 `70 ms`、P90 约 `80 ms`，无法稳定满足 20 Hz 所要求的 `50 ms` 周期。结果是一次长计算后紧跟一次追赶式更新，视觉上呈现“顿一下、追一下”。

处理（静态/动态共用的控制预算）：

- 动态避障 overlay 将 MPPI 调为 15 Hz，并同步设置 `model_dt=1/15 s`、`time_steps=30`，继续保持完整 2.0 s 预测范围和控制周期/模型步长一致。
- 保留 `batch_size=500` 和 `regenerate_noises=true`；固定噪声候选集虽然更平滑，却会降低对新出现动态障碍物的适应性。
- Local Costmap 使用 10 Hz 更新、5 Hz 发布。实测 RGB-D 与 LiDAR 只有约 6–8 Hz，原 20 Hz 只会反复处理相同观测，并与同一 `controller_server` 内的 MPPI 争用计算时间。
- Velocity Smoother 继续使用 60 Hz，与 Isaac 物理步一致；在 `az_max=6.5 rad/s²` 下，每个物理帧的角速度变化约为 `0.108 rad/s`。
- `vx_max=1.20 m/s`、`vx_std=0.90 m/s`，降低室内绕障的过冲和速度饱和采样。
- MPPI `gamma=0.030`，提高连续控制序列的偏好；保持 `wz_max=3.40 rad/s`。
- 候选轨迹发布下采样为 `trajectory_step=25`、`time_step=5`，RViz 默认关闭 Candidate Trajectories，仅显示最优轨迹，避免可视化负担影响控制节奏。

静态与动态不能共用同一个 RGB-D 清除策略：静态 `0.16 m` 低矮物体需要在机器人转向、短暂离开前视相机视野后仍留在 Local/Global Costmap；动态 actor 则必须在离开相机视野后过期，不能形成扫掠残影。因此基础 profile 在 Local 与 Global Costmap 均保留标准 `VoxelLayer`，动态 overlay 只将 **Local** 层替换为 STVL，并明确移除 Global RGB-D 体素层。MPPI、平滑器、碰撞监控、LiDAR 和速度/加速度限制完全相同。

频率 A/B 结果：

| 配置 | 结果 | 结论 |
|---|---|---|
| 20 Hz、40 步 | 避障能通过，但实测计算时间超过 50 ms，产生长间隔与追赶更新 | 控制负载超出实时预算 |
| 12.5 Hz、25 步 | 单段更平滑，但最快横穿 case 出现 `safety_yield`、最小净空降到 0 | 响应间隔过长，拒绝 |
| 15 Hz、30 步 | 单段与完整三阶段均成功；单段最小净空 `0.1425 m`，整圈三段为 `0.4931 / 0.3222 / 0.4250 m`，无 `safety_yield` | 当前动态避障 profile 基线 |

15 Hz 单段中，60 Hz 平滑命令的线/角速度反向变化比例相对 20 Hz 分别下降约 13% 和 26%。这不是用低频掩盖控制问题，而是让请求频率匹配本机可持续计算预算。

### 6. RViz 中“规划是直线，但实际轨迹向左漂”

截图中的两条线不是同一时刻、同一语义的数据：

- 黄色 `/plan` 的 RViz Buffer Length 为 1，只显示**当前时刻向前**的最新全局路径。
- 蓝色 `/odom` 保留约 200 个历史位姿，显示机器人**此前实际走过**的轨迹。

按 rosbag 时间对齐后可以确认：机器人向左运动的那一段，当时的全局路径也已经重规划到左侧；机器人 Ground Truth/odom 跟随了当时路径。机器人驶离该段后，全局规划器又生成了当前较直的黄色路径，于是静态截图把“现在的未来路径”和“过去的历史轨迹”叠在一起，看起来像控制器无故横漂。

因此这不是 TF、里程计或四轮差速底盘的横向失控。复核时必须按时间戳对齐 `/plan`、`/odom` 和 `/ground_truth/odom`，或暂时把 `/odom` 的 Buffer Length 改为 1；不能用单帧截图直接比较历史蓝线和当前黄线。

### 7. 失败的底盘平滑尝试

曾将 `SkidSteerMotionAssist` 的内部跟踪带宽从 `6.0 m/s² / 30 rad/s²` 压低到与 Nav2 相同的 `3.5 m/s² / 6.5 rad/s²`。实测这会使实体车体落后于已平滑的指令，并在门口转弯时出现更明显的追赶式修正。

结论：恢复内部补偿的 `6.0 / 30` 带宽；平滑应在 MPPI 和 Velocity Smoother 层完成，不能让内层跟踪器成为新的低带宽滞后环节。

### 8. 静态方块把局部绕行空间挤成死角

为严格复现 `codex/kujiale-mppi-feasibility-tuning` 的静态 20 轮基线，`rgbd_low_box_east` 使用原始中心 `[0.366563, 0.667950]`。该坐标同时写入 Isaac 物理场景、GUI 布局草案和实验 campaign；不会改动 `warehouse_new`，静态与动态控制参数也完全不变。

## 复测方法

每次只改变一组参数，并重启与改动层相对应的进程：

- 修改 `isaac_sim/configs/robots/jackal.yaml` 或场景补片：重启 Isaac 与导航栈。
- 修改 `nav2_params.yaml`、`nav2_dynamic_avoidance.yaml` 或 RViz：重启导航栈（RViz 由该脚本受管）。
- 动态 actor 配置：重启 Isaac。

单轮可视化：

```bash
./scripts/run_kujiale_dynamic_visual.sh --case local_bypass --variant 3 --seed 7443 --record
```

判读时至少检查 `run_manifest.json` 中的 `failure_reason`、`obstacle_events`、`dynamic_behavior` 和 `command_motion_quality`。成功不等于行为良好；应同时确认没有 `safety_yield`、触发时机正确、右侧绕行成立，且命令加速度没有异常尖峰。

完整三阶段频率回归：

```bash
./scripts/run_kujiale_three_stage_visual.sh full \
  --variant 1 \
  --seed 7501 \
  --record
```

更改 MPPI 频率时必须同时调整 `model_dt`，并通过 `time_steps × model_dt = 2.0 s` 保持预测范围。单段成功不足以验收频率；至少还要用相同 variant/seed 跑完整三阶段，并检查三段最小净空及 `safety_yield/guard_abort` 事件。
