# Kujiale 动态避障调优问题复盘

本文记录 `local_bypass` 单轮可视化调优中发现的问题、根因和当前处理方式。它是调优复盘，不替代正式动态 20 次验收。

## 当前可视化基线

- 场景：`kujiale_0026_A_to_B_door_open.usd`，出生点 `long_route_start_g1`。
- Case：`local_bypass`，`variant=3`，`seed=7443`。
- Actor：`0.40 × 0.40 × 1.00 m`，轨迹 `[-1.65,-0.20] -> [-0.85,-0.20]`，峰值速度 `0.80 m/s`、最大加速度 `1.60 m/s²`，到点后保持 `parked`。
- 触发门：机器人在 `x∈[-0.70,0.30]` 内北向越过 `y=-1.75`，且速度不低于 `0.20 m/s`。
- 导航：MPPI 20 Hz、2.0 s 预测范围；Velocity Smoother 60 Hz；`vx_max=1.20 m/s`、`wz_max=3.40 rad/s`。

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

20 Hz 的速度平滑器在 `az_max=6.5 rad/s²` 下每次最多改变 `0.325 rad/s`，视觉上会出现明显台阶。初始线速度上限和采样范围也偏大，使每次路径修正的横向位移过大。

处理：

- 保持 MPPI 20 Hz 和 `model_dt=0.05`，避免破坏预测模型与控制周期的一致性。
- Velocity Smoother 改为 60 Hz，与 Isaac 物理步一致；同一角加速度被拆为约 `0.108 rad/s` 的每帧变化。
- `vx_max=1.20 m/s`、`vx_std=0.90 m/s`，降低室内绕障的过冲和速度饱和采样。
- MPPI `gamma=0.030`，提高连续控制序列的偏好；保持 `wz_max=3.40 rad/s`。
- 候选轨迹发布下采样为 `trajectory_step=25`、`time_step=5`，RViz 默认关闭 Candidate Trajectories，仅显示最优轨迹，避免可视化负担影响控制节奏。

### 6. 失败的底盘平滑尝试

曾将 `SkidSteerMotionAssist` 的内部跟踪带宽从 `6.0 m/s² / 30 rad/s²` 压低到与 Nav2 相同的 `3.5 m/s² / 6.5 rad/s²`。实测这会使实体车体落后于已平滑的指令，并在门口转弯时出现更明显的追赶式修正。

结论：恢复内部补偿的 `6.0 / 30` 带宽；平滑应在 MPPI 和 Velocity Smoother 层完成，不能让内层跟踪器成为新的低带宽滞后环节。

## 复测方法

每次只改变一组参数，并重启与改动层相对应的进程：

- 修改 `isaac_sim/configs/robots/jackal.yaml` 或场景补片：重启 Isaac 与导航栈。
- 修改 `nav2_params.yaml` 或 RViz：重启导航栈（RViz 由该脚本受管）。
- 动态 actor 配置：重启 Isaac。

单轮可视化：

```bash
./scripts/run_kujiale_dynamic_visual.sh --case local_bypass --variant 3 --seed 7443 --record
```

判读时至少检查 `run_manifest.json` 中的 `failure_reason`、`obstacle_events`、`dynamic_behavior` 和 `command_motion_quality`。成功不等于行为良好；应同时确认没有 `safety_yield`、触发时机正确、右侧绕行成立，且命令加速度没有异常尖峰。
