# Kujiale 三阶段动态避障实验方案

## 1. 方案概述

保持固定闭环路线：

```text
G1 -> G2 -> G3 -> G4 -> G5 -> G1
```

在三个航段配置三种不同的动态交互，以分别验证横向绕行、窄通道同向跟随与出口转向、门洞横穿后的左侧绕行能力：

```text
G1->G2   (-1.65, -0.20) -----> (-0.85, -0.20)  横向移动并停车

G2->G3        (-0.40, 1.00)
                       |
                       v
               (-0.40, -0.70)                   纵向同向移动并停车

G5->G1   (-0.90, -1.45) -----> (-0.20, -1.45)  门洞横穿并停车
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
| actor 轨迹 | `[-1.65, -0.20, 0.50] -> [-0.85, -0.20, 0.50]` |
| 峰值速度 / 最大加速度 | `0.80 m/s` / `1.60 m/s^2` |
| 预期行为 | 机器人从 actor 右侧绕过，继续进入原定左侧狭窄通道并到达 G2。 |

这是已经调通的基线，不改变当前 MPPI、Velocity Smoother 或 RViz 候选轨迹下采样参数。为满足 `warehouse_new` 5 cm 栅格中的 `>=0.05 m` 墙体余量，轨迹较原 `y=-0.15` 的视觉基线向下修正 `0.05 m`。

### 2.2 G2 -> G3：y 约 1 m 的纵向同向释放

| 项目 | 参数 |
|---|---|
| interaction ID | `g2_g3_exit` |
| arm / retire 航点 | `G3` / `G3` |
| 机器人触发门 | `y <= 2.20`、`x in [-0.55, -0.25]`、南向、速度 `>= 0.20 m/s` |
| actor 轨迹 | `[-0.40, 1.00, 0.50] -> [-0.40, -0.70, 0.50]` |
| 峰值速度 / 最大加速度 | `0.65 m/s` / `0.90 m/s^2` |
| 预期行为 | actor 在窄通道前方同向向下移动；机器人连续跟随，在出口附近绕过停车 actor 后完成转向并到达 G3。 |

该门限依据现有 20 轮静态 Ground Truth：机器人南向经过 `y=2.20` 时位于 `x=-0.47..-0.35`，可在 actor 起动时保留约一段可感知、可规划的安全距离。actor 的余弦缓入缓出运动时长约为 `4.11 s`，不会退化为静态突然封堵。

### 2.3 G5 -> G1：门洞内从左向右横穿

| 项目 | 参数 |
|---|---|
| interaction ID | `g5_g1_door_crossing` |
| arm / retire 航点 | `G1` / `G1` |
| 机器人触发门 | `y <= -0.55`、`x in [-0.75, -0.15]`、南向、速度 `>= 0.20 m/s` |
| actor 轨迹 | `[-0.90, -1.45, 0.50] -> [-0.20, -1.45, 0.50]` |
| 峰值速度 / 最大加速度 | `0.80 m/s` / `1.60 m/s^2` |
| 预期行为 | actor 横穿后停在门洞右侧；机器人从左侧绕过 actor，进入返回 G1 的通道。 |

红框区域不能整段横穿：中间包含墙体和门柱。该 actor 仅在已检查过的门洞自由带 `x=-0.90..-0.20`、`y=-1.45` 内移动。0.40 m 方块沿整段扫掠与静态占用区的几何余量不低于 `0.05 m`，避免视觉或物理上的穿墙。

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
- G5->G1 的 `y≈-1.45` 门洞局部放大框；
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
- G2->G3：先真实行驶至 G2，再仅启用 `g2_g3_exit`；
- G5->G1：保留真实进场路线至 G5，再仅启用 `g5_g1_crossing`；
- 整圈联测：使用 `full_route_three_stage`，按 G2、G3、G1 三次接力交互。

聚焦测试不使用未经标定的临时出生点，避免出生点、TF 或里程计初始化差异掩盖动态避障行为。

启动 Isaac 和原有导航/RViz 后，用以下入口进行人工录制；`--record` 会归档本轮证据，脚本不会将结果宣称为正式验收。

```bash
# 终端 1：Isaac（每次动态 YAML 改动后重启）
cd /home/lyb/Workspace/Isaac_Sim_ROS2_Nav
./scripts/run_kujiale_dynamic_isaac.sh

# 终端 2：保留原有 navigation.rviz 的导航栈
cd /home/lyb/Workspace/Isaac_Sim_ROS2_Nav
./scripts/run_ros.sh navigation odometry_mode:=ideal spawn_pose_name:=long_route_start_g1

# 终端 3：一次只运行一个聚焦交互，或整圈接力；均可加 --record
./scripts/run_kujiale_three_stage_visual.sh g1-g2 --variant 3 --seed 7443 --record
./scripts/run_kujiale_three_stage_visual.sh g2-g3 --variant 1 --seed 7461 --record
./scripts/run_kujiale_three_stage_visual.sh g5-g1 --variant 1 --seed 7471 --record
./scripts/run_kujiale_three_stage_visual.sh full  --variant 1 --seed 7501 --record
```

`g2-g3` 会真实执行 G1→G2→G3；`g5-g1` 会真实执行 G1→G2→G3→G4→G5→G1，保证进场路线和触发姿态都经过标定。整圈五次联测使用：

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
7. 对应占用在 Local Costmap `0.25 s` 内、Global Costmap `1.0 s` 内被正常清除。

最终 Isaac Sim 可视化验收由用户执行。实现阶段只提供启动脚本、三段单测、整圈联测、录制与证据归档入口；不将开发期 smoke test 宣称为正式验收结论。

## 6. 实施边界

本轮仅修改动态实验编排、actor 生命周期、测试、可视化和脚本入口。保持已调通的 MPPI 20 Hz、Velocity Smoother 60 Hz、速度上限、角速度上限、角加速度和 RViz 候选轨迹下采样设置不变。
