# Kujiale 三阶段动态避障实验方案（4×20动态组编排参考）

> 三阶段 actor 的几何、gate 和生命周期仍是当前 `full_route_three_stage` 的权威编排来源；
> 但本文中的聚焦可视化与旧 `full-route-5` 命令只用于诊断。正式20轮动态基准和20轮动态＋外观
> 请执行 [`kujiale_4x20_appearance_benchmark_plan.md`](kujiale_4x20_appearance_benchmark_plan.md)。

## 1. 方案概述

保持固定闭环路线：

```text
G1 -> G2 -> G3 -> G4 -> G5 -> G1
```

在三个航段配置三种不同的动态交互，以分别验证横向绕行、窄通道同向跟随与出口转向、门洞横穿后的左侧绕行能力：

```text
G1->G2   (-1.65, -0.20) -----> (-0.95, -0.20)  横向移动并停车

G2->G3        (-0.40, 1.00)
                       |
                       v
               (-0.40, -0.70)                   纵向同向移动并停车

G5->G1   (-0.90, -1.30) -----> (-0.20, -1.30)  门外横穿并停车
```

三个 actor 均为高 LiDAR 可见的运动学方块：

| 属性 | 固定值 |
|---|---|
| 尺寸 | `0.40 x 0.40 x 1.00 m` |
| 质量 | `20 kg` |
| 中心高度 | `z=0.50 m` |
| 完成运动后的状态 | 保持可见、可碰撞的 `parked` |
| 删除时机 | 对应下一航点由 Nav2 成功到达后 |

删除是逻辑退役：立即隐藏 USD actor、禁用碰撞并删除 RViz marker；reset 时重新恢复原始 actor。因此不会破坏下一轮的可重复性。

## 2. 三个动态交互

### 2.1 G1 -> G2：保留当前横向局部绕障

| 项目 | 参数 |
|---|---|
| interaction ID | `g1_g2_local_bypass`（兼容原 `local_bypass`） |
| arm / retire 航点 | `G2` / `G2` |
| 机器人触发门 | `y >= -1.75`、`x in [-0.70, 0.30]`、北向、速度 `>= 0.20 m/s` |
| actor 轨迹 | `[-1.65, -0.20, 0.50] -> [-0.95, -0.20, 0.50]` |
| 峰值速度 / 最大加速度 | `0.80 m/s` / `1.60 m/s^2` |
| 预期行为 | 机器人从 actor 右侧绕过，继续进入原定左侧狭窄通道并到达 G2。 |

这是已经调通的基线，不改变当前 MPPI、Velocity Smoother 或 RViz 候选轨迹下采样参数。为满足 `warehouse_new` 5 cm 栅格中的 `>=0.05 m` 墙体余量，轨迹较原 `y=-0.15` 的视觉基线向下修正 `0.05 m`。

### 2.2 G2 -> G3：y 约 1 m 的纵向同向释放

| 项目 | 参数 |
|---|---|
| interaction ID | `g2_g3_exit` |
| arm / retire 航点 | `G3` / `G3` |
| 机器人触发门 | `y <= 2.60`、`x in [-0.55, -0.25]`、南向、速度 `>= 0.20 m/s` |
| actor 轨迹 | `[-0.40, 1.00, 0.50] -> [-0.40, -0.70, 0.50]` |
| 峰值速度 / 最大加速度 | `0.65 m/s` / `0.90 m/s^2` |
| 预期行为 | actor 在窄通道前方同向向下移动；机器人连续跟随，在出口附近绕过停车 actor 后完成转向并到达 G3。 |

触发门设为 `y=2.60`，让机器人在保持窄通道对准后更接近同向 actor 才释放运动，避免出现仅远距离并行而未形成跟随交互。横向窗口、南向速度门仍要求机器人已经对准狭窄通道，避免在 G2 出发转向阶段提前触发。验收时，运行器仅把 actor 运动中的同车道前方距离 `0.20–1.40 m` 计为跟随；上限 `1.40 m` 覆盖校准试运行的 `1.2301 m` 以及 v2 外观轮的 `1.3564 m` 最小前车距，仍会排除远距离并行。actor 的余弦缓入缓出运动时长约为 `4.11 s`，不会退化为静态突然封堵。

### 2.3 G5 -> G1：门洞内从左向右横穿

| 项目 | 参数 |
|---|---|
| interaction ID | `g5_g1_door_crossing` |
| arm / retire 航点 | `G1` / `G1` |
| 机器人触发门 | 北向通过 `y >= -2.50`、`x in [-2.00, -0.15]`，且距 actor 起点 `<=1.05 m`、速度 `>= 0.20 m/s` |
| actor 轨迹 | `[-0.90, -1.30, 0.50] -> [-0.20, -1.30, 0.50]` |
| 峰值速度 / 最大加速度 | `0.32 m/s` / `1.60 m/s^2` |
| 预期行为 | actor 横穿后停在门洞右侧；机器人从左侧绕过 actor，进入返回 G1 的通道。 |

红框区域不能整段横穿：中间包含墙体和门柱。该 actor 仅在已检查过的门洞外侧自由带 `x=-0.90..-0.20`、`y=-1.30` 内移动。录制轨迹确认仅凭坐标门会使 actor 在车体旁突然出现，因此触发使用北向进场窗与 `1.05 m` 起点前视距离的交集：既不会在宽敞进场刚开始时触发，也不会进入接触距离后才出现。横穿速度为 `0.32 m/s`，给予局部规划器连续绕行时间。0.40 m 方块沿整段扫掠与静态占用区的几何余量约为 `0.170 m`，高于 `0.05 m` 要求，避免视觉或物理上的穿墙。

## 3. 顺序状态机与接口

物理动态配置升级为顺序交互模型。每个 interaction 声明：

- `arm_goal_id`：目标被发送且 Nav2 接受后进入 `armed` 的航点；
- `retire_goal_id`：目标成功到达后退役 actor 的航点；
- 空间 gate、actor 几何、轨迹、速度和加速度；
- 固定的 nominal variant；必要时可按 interaction 添加启动延迟 variant。

单个 actor 的状态机为：

```text
waiting -> armed -> moving -> parked -> retired
```

- `waiting`：隐藏且禁用碰撞。
- `armed`：对应目标已发送，但 actor 仍隐藏；只有机器人通过空间 gate 后才出现。
- `moving`：actor 使用连续余弦缓入缓出轨迹运动。
- `parked`：运动完成后保持可见和碰撞，机器人必须在其仍存在时绕开。
- `retired`：仅在 `retire_goal_id` 对应目标成功后进入；隐藏、禁用碰撞、删除 RViz marker。

运行器在 Nav2 目标成功后调用幂等接口：

```text
/experiment/obstacles/<goal_id>/complete
```

接口只退役其 `retire_goal_id` 匹配的 actor。目标失败、取消或超时时不得提前删除 actor，保留现场、状态和最小净距证据直到本轮 reset。

支持的选择包括：

- `local_bypass`：原 G1->G2 单交互兼容别名；
- `g2_g3_exit`：只启用 G2->G3 交互；
- `g5_g1_crossing`：只启用 G5->G1 交互；
- `full_route_three_stage`：按 G2、G3、G1 顺序启用全部三个交互。

任意时刻至多一个 actor 可见且可碰撞。状态 topic 与 run manifest 必须记录 `armed`、`gate_enter`、`motion_start`、`park` 和 `goal_reached_retire` 事件及其仿真时间。

## 4. 动态避障示意图

动态图必须从 `warehouse_new` OccupancyGrid 生成，保持 map 坐标和实际实验一致，而不使用示意性平面图。

更新后的全图应包含：

- 三条不同颜色的 actor 轨迹，实心方块表示起点，箭头表示运动方向，空心方块表示停车点；
- 三条机器人空间触发线，并标注 `arm` 航点、`retire` 航点和运动参数；
- G2->G3 的 `y≈1` 窄通道局部放大框；
- G5->G1 的 `y≈-1.30` 门洞外侧局部放大框；
- 图例中明确说明 actor 在停车后不会消失，只有抵达对应下一航点后才退役。

输出文件：

- `docs/figures/kujiale_long_route_dynamic_map.png`：全屋三阶段动态避障总览；
- `docs/figures/kujiale_three_stage_dynamic_details.png`：G2->G3 与 G5->G1 两个局部交互区。

## 5. 测试与验收

### 5.1 自动化测试

- schema/parser：三交互顺序配置、重复 ID、非法目标顺序和非法 gate 均应被拒绝；
- 状态机：gate 前隐藏、只触发一次、运动后停车、成功目标后退役、失败目标不退役、reset 恢复；
- runner：目标发送后 arm，只有成功结果后调用 complete；
- RViz：actor 退役后发布 `Marker.DELETE`，不得留下旧方块或旧文本；
- 几何：三条完整 swept footprint 均不得进入 occupied/unknown 栅格，最小墙体余量 `>=0.05 m`；
- 运行证据：每段记录 actor 状态、速度、进度、最小净距和退役事件。

### 5.2 三段单轮可视化

- G1->G2：从 G1 到 G2，仅启用 `local_bypass`；
- G2->G3：从经 `warehouse_new` 标定变换推导的 G2 出生点直接开始，仅启用 `g2_g3_exit`；
- G5->G1：从经 `warehouse_new` 标定变换推导的 G5 出生点直接开始，仅启用 `g5_g1_crossing`；
- 整圈联测：使用 `full_route_three_stage`，按 G2、G3、G1 三次接力交互。

聚焦测试不使用未经标定的临时出生点，避免出生点、TF 或里程计初始化差异掩盖动态避障行为。

启动 Isaac 和原有导航/RViz 后，用以下入口进行人工录制；`--record` 会归档本轮证据，脚本不会将结果宣称为正式验收。

```bash
# 终端 1：G1→G2 与整圈联测的 Isaac（每次动态 YAML 改动后重启）
cd /home/lyb/Workspace/Isaac_Sim_ROS2_Nav
./scripts/run_kujiale_dynamic_isaac.sh

# 终端 2：保留原有 navigation.rviz 的导航栈
cd /home/lyb/Workspace/Isaac_Sim_ROS2_Nav
./scripts/run_ros.sh navigation odometry_mode:=ideal spawn_pose_name:=long_route_start_g1 nav2_profile:=dynamic_avoidance

# 终端 3：一次只运行一个聚焦交互，或整圈接力；均可加 --record
./scripts/run_kujiale_three_stage_visual.sh g1-g2 --variant 3 --seed 7443 --record
./scripts/run_kujiale_three_stage_visual.sh full  --variant 1 --seed 7501 --record
```

三阶段测试必须使用 `nav2_profile:=dynamic_avoidance`：该覆盖配置保持既有 MPPI、速度平滑和 LiDAR 射线清障；局部 RGB-D 感知由标准 `VoxelLayer` 切换为 **STVL（时空体素层）**。STVL 仍消费 `/camera/front/depth/points` 并发布局部体素可视化，但会对离开前视相机视野的旧体素做时间衰减，避免动态 actor 留下长条占用。

首次使用该 profile 前，安装一次 Jazzy 的官方 STVL 二进制包：

```bash
sudo apt install ros-jazzy-spatio-temporal-voxel-layer
```

若未安装，`run_ros.sh` 会在启动前直接给出该命令，不会进入一个插件加载失败的导航栈。动态 profile 只替换滚动 Local Costmap 的 RGB-D 层；Global Costmap 仍保持静态图加 LiDAR，Collision Monitor 仍只使用 `/scan`。

`g2-g3` 使用 `long_route_start_g2`，Isaac 和导航栈必须一起重启到该出生点：

```bash
# 终端 1：Isaac 直接出生于经标定的 G2
./scripts/run_kujiale_dynamic_isaac.sh --spawn-pose long_route_start_g2

# 终端 2：导航/RViz 使用同一 G2 出生点
./scripts/run_ros.sh navigation odometry_mode:=ideal spawn_pose_name:=long_route_start_g2 nav2_profile:=dynamic_avoidance

# 终端 3：正式录像与指标从 G2→G3 开始
./scripts/run_kujiale_three_stage_visual.sh g2-g3 --variant 1 --seed 7461 --record
```

`g5-g1` 使用 `long_route_start_g5`，Isaac 和导航栈必须一起重启到该出生点：

```bash
# 终端 1：Isaac 直接出生于经标定的 G5
./scripts/run_kujiale_dynamic_isaac.sh --spawn-pose long_route_start_g5

# 终端 2：导航/RViz 使用同一 G5 出生点
./scripts/run_ros.sh navigation odometry_mode:=ideal spawn_pose_name:=long_route_start_g5 nav2_profile:=dynamic_avoidance

# 终端 3：正式录像与指标从 G5→G1 开始
./scripts/run_kujiale_three_stage_visual.sh g5-g1 --variant 1 --seed 7471 --record
```

整圈联测仍会真实执行 G1→G2→G3→G4→G5→G1，保证接力交互的进场路线与触发姿态一致。整圈五次联测使用：

```bash
./scripts/run_kujiale_dynamic_acceptance.sh full-route-5 THREE_STAGE_01
```

### 5.3 每段通过条件

每个动态交互必须同时满足：

1. Nav2 成功到达该航段目标；
2. actor 确实通过 `moving` 后进入 `parked`；
3. 没有物理接触、`safety_yield` 或 `guard_aborted`；
4. 最小净距不低于 `0.10 m`；
5. 机器人在 actor 尚未退役时完成绕行，不能等待 actor 删除后才继续；
6. actor 在目标成功后 `0.20 s` 内退役；
7. Local STVL 中对应占用在 actor 离开视野后应按时序衰减；动态 profile 的 Global Costmap 不注入 RGB-D actor，因而不应出现需要等待清除的全局 actor 残影。

最终 Isaac Sim 可视化验收由用户执行。实现阶段只提供启动脚本、三段单测、整圈联测、录制与证据归档入口；不将开发期 smoke test 宣称为正式验收结论。

## 6. 实施边界

动态实验编排与静态导航使用**不同但各自固定**的控制 profile，不能把一套参数误称为两者通用：

| Profile | MPPI | 速度/角速度上限 | Velocity Smoother | RGB-D Costmap 策略 |
|---|---|---|---|---|
| `stable` | `10 Hz`、20 步、`0.10 s`、700 条采样 | `0.75 m/s` / `1.35 rad/s` | `20 Hz` | Local + Global 标准 `VoxelLayer`，保留低矮静态物体。 |
| `dynamic_avoidance` | `15 Hz`、30 步、`1/15 s`、500 条采样 | `1.20 m/s` / `3.40 rad/s` | `60 Hz` | Local STVL（10 Hz 更新、5 Hz 发布）+ `0.60 m` 局部膨胀；Global 仅静态图 + LiDAR。 |

两者均保持 `2.0 s` 预测范围、完整 footprint 碰撞检查与 LiDAR Collision Monitor。动态 profile 还将候选轨迹发布下采样为 `trajectory_step=25`、`time_step=5`，而 RViz 默认只显示最优 MPPI 轨迹。切换 profile 必须重启导航栈；修改 actor YAML 还必须重启 Isaac，避免触发运行时配置 hash 保护。
