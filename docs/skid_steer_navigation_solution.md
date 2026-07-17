# Jackal 四轮滑移底盘转弯困难：问题分析与解决方案

本文记录 2026-07-16 至 2026-07-17 对 Jackal 四轮滑移转向底盘的完整诊断、修复和验收结果。它回答三个问题：最初遇到了什么、为什么会发生、项目最终如何解决。

## 1. 结论摘要

最初的问题不是全局规划器不会生成弯道，也不是四个轮子缺少直行驱动力。决定性故障位于“期望速度到真实车体运动”的执行链：PhysX 的轮地接触对四轮滑移转向产生了很强的横向阻力，左右轮速差只能转换成很小的车体角速度。旧控制图又只在收到新消息时执行控制器，并用固定物理步长积分，导致加速度随 `/cmd_vel` 发布频率变化；Nav2 原有参数同时偏向较高线速度、较小角速度扰动和较晚的朝向修正。这些因素叠加后，就表现为全局路径已经转弯，小车却继续直行、迟转、转弯半径过大，最后脱离路径。

修复不是单独增大某一个 Nav2 权重，而是同时校正四层：

1. 修复轮子碰撞体、轮关节驱动和 PhysX 求解稳定性；
2. 将底盘控制改成每个物理步执行、使用真实仿真 `dt`，并一次原子写入四轮目标；
3. 增加带超时和加速度限制的滑移转向平面运动补偿，使 PhysX 车体响应与差速/MPPI 模型一致；
4. 重新标定有效轮距，调整 MPPI、Velocity Smoother、进度判定和 Ideal 定位链，并用自动化运动原语与约 50 m 复杂导航路线验收。

最终底盘自动运动基准 `10/10` 通过；Ideal 复杂静态路线 `3/3`、复杂动态路线 `3/3` 通过，均完成 6/6 航点、0 恢复、0 碰撞。本阶段按要求关闭导航倒车采样，因此复杂导航结论只覆盖前进优先路径跟踪；底盘直接控制层的倒车和倒车转弯能力仍保留且已单独验证。

## 2. 最初遇到的问题

### 2.1 主要症状

- Jackal 直线前进正常，说明总驱动力和线速度通道基本可用；
- 全局路径已经出现明显弯道时，机器人仍倾向继续直行；
- 转向开始较晚，真实角速度明显小于命令角速度；
- 实际转弯半径远大于期望半径，需要很长时间和很大空间才能完成转弯；
- 在狭窄或连续弯曲路径上无法及时回到路径，最终触发导航失败；
- 键盘或手动控制的原地旋转基本可行，但 Nav2 自动导航下转向明显更弱；
- 原配置只允许较低倒车速度，控制器也不愿主动选择倒车轨迹。

### 2.2 为什么“能原地转”不能排除底盘问题

手动原地旋转通常持续发送幅值较大的纯角速度命令，线速度为零，而且操作者可以等待更长时间。导航控制则要求机器人在保持前进速度的同时，连续、及时、准确地实现一系列曲率，并在左右转之间快速切换。

因此，原地旋转可行只能证明底盘不是完全锁死，不能证明以下能力正常：

- 左右轮速差能否在前进过程中有效转换为角速度；
- 实际曲率能否跟随 `angular.z / linear.x`；
- 低幅值、连续变化的导航指令能否按时执行；
- 控制器、速度平滑器与物理底盘的运动模型是否一致。

这正好解释了“手动能转、导航难转”和“能转但耗时长、空间大”同时存在的现象。

## 3. 诊断方法与关键证据

控制链如下：

```mermaid
flowchart LR
    GlobalPath["全局路径"] --> MPPI["MPPI 局部控制器"]
    MPPI --> Smoother["Velocity Smoother"]
    Smoother --> Safety["Collision Monitor"]
    Safety --> Cmd["/cmd_vel"]
    Cmd --> Graph["OnPhysicsStep 差速控制图"]
    Graph --> Wheels["四轮目标速度"]
    Wheels --> PhysX["PhysX 轮地接触"]
    PhysX --> Motion["真实车体速度与轨迹"]
    Motion --> Odom["Ideal / Realistic 里程计"]
    Odom --> MPPI
```

诊断时先绕过 Nav2，直接向底盘发送固定 `linear.x` 和 `angular.z`，比较命令与 Ground Truth 的稳态线速度、角速度和转弯半径。这样可以把“规划/控制器选择不理想”和“底盘没有执行命令”分开。

修复前，前进圆弧命令 `0.30 m/s, 0.80 rad/s` 的真实稳态响应约为 `0.235 m/s, 0.088 rad/s`：

- 期望半径：`0.30 / 0.80 = 0.375 m`；
- 实际半径：`0.235 / 0.088 ≈ 2.68 m`；
- 真实角速度只达到命令的约 `11%`。

倒车圆弧的实际半径也约为 `2.08 m`。这组数据直接证明，转弯困难的首要原因在底盘执行/物理层，而不是全局路径没有提前转弯。

## 4. 问题产生的原因

### 4.1 已确认的主要根因：PhysX 对四轮滑移转向严重欠转

四轮滑移转向在转弯时必须允许轮胎产生横向滑移。当前 PhysX 轮地接触是各向同性摩擦模型，它对轮胎横向运动施加了过强约束。直行时四轮方向一致，所以影响较小；转弯时左右轮需要形成速度差，前后轮又必须横向擦滑，问题便集中暴露为角速度不足和前向速度损失。

这也是“直行很好、弧线很差”最核心的物理解释。仅提高轮子转速或 Nav2 角速度上限，不能保证这些轮速差真正变成车体偏航。

此外，导入的 Jackal 资产使用了 Isaac Sim 6 已移除的 PhysX `customGeometry` 轮子碰撞属性。旧碰撞体不是可靠的 Isaac Sim 6 基线，需要先替换成受支持且左右对称的标准圆柱碰撞体，才能稳定讨论摩擦与控制标定。

### 4.2 已确认的控制时基错误：控制器实际增量依赖消息频率

旧 OmniGraph 由 `ROS2SubscribeTwist.execOut` 触发 `DifferentialController`。这个执行端口只在收到新消息时脉冲，但控制器使用的 `dt` 却固定为 `1 / 60 s`。如果 `/cmd_vel` 是 10 Hz，控制器每秒只执行约 10 次，却每次只按 1/60 秒积分，等效加速度约缩小为配置值的 `10 / 60`；消息频率变化还会改变底盘动态响应。

旧图还使用前、后两个 Articulation Controller 分别写轮速，增加了同一物理步内四轮目标不同步的风险。对于依赖左右轮速差的滑移转向，这会进一步破坏瞬态曲率和左右快速切换。

修复后的 10 Hz 与 20 Hz `/cmd_vel` 阶跃测试中，轮速 10%–90% 上升时间差只剩 `0.017 s`（直线）和 `0.016 s`（旋转），证明控制动态已基本脱离命令发布频率。

### 4.3 运动模型与里程计标定不一致

几何轮距 `0.37559 m` 描述轮心位置，但滑移转向的轮速差到偏航速度之间存在轮胎擦滑，运动学中的“有效轮距”不等于几何轮距。继续把几何值直接用于差速控制和 Wheel Odometry，会造成命令、轮速里程计与真实偏航之间不一致。

最终将控制器和 Wheel Odometry 的有效轮距统一标定为 `0.800 m`，几何轮距仍保留给 USD/URDF。Realistic EKF 只从 Wheel Odometry 融合车体前向速度，角速度改由 IMU 独占，避免低速转弯时用受轮胎擦滑影响的左右轮差重复污染偏航估计。

### 4.4 Nav2 原参数放大了迟转和大半径

物理层欠转是决定性根因，但旧 MPPI/平滑参数会让问题更明显：

- `vx_max = 1.0 m/s`，线速度相对转向能力过强；
- `wz_std = 0.40`，角速度采样范围偏保守；
- `az_max = 2.0 rad/s²`，左右转切换和入弯角速度建立较慢；
- `PathAngleCritic` 权重只有 `2.0`，允许的最大朝向偏差为 `1.0 rad`，朝向修正偏晚；
- `PathAlignCritic.offset_from_furthest = 20`，路径对齐参考点过远；
- `Velocity Smoother.scale_velocities = false`，线速度受限时不能按比例保持命令曲率；
- `SimpleProgressChecker` 只按较大的平移半径判断进展，原地修正朝向不容易被视为有效进展。

这些设置不能单独解释修复前只有约 11% 的角速度响应，但会让 MPPI 更晚要求转弯、转弯时保留过多前进速度，并降低连续弯道的跟踪余量。

### 4.5 计算周期与 Ideal 定位链的附加影响

在当前工作站的 Isaac 负载下，20 Hz、较大 batch 的 MPPI 多次错过控制周期。控制输出间隔不稳定会带来顿挫和安全停车，也会降低路径跟踪质量。实测保留 2 秒预测窗、改为 10 Hz 后更稳定，最终 batch 进一步降为 500。

Ideal `/odom` 已由 Isaac 精确给出；如果再让 SLAM Toolbox 根据激光扫描持续修正 `map → odom`，相当于在理想位姿上叠加第二套定位修正。它不是最初欠转的物理根因，但会给复杂路线引入不必要的位姿扰动和 Reset 后 TF 新鲜度问题。最终 Ideal Localization 改为持续发布新鲜的 identity `map → odom`；Realistic 模式仍使用 Pose Graph 定位。

## 5. 最终解决方案

### 5.1 修复轮子物理基础

在 [`isaac_sim/assets/robots/jackal/jackal_nav.usda`](../isaac_sim/assets/robots/jackal/jackal_nav.usda) 中完成：

- 停用四个带旧 `customGeometry` 属性的碰撞体；
- 为每个轮子建立半径 `0.098 m`、宽度 `0.04 m` 的标准 Cylinder collider；
- 四轮统一使用 `7 N·m` 最大驱动力矩、约 `50 N·m·s/rad` 阻尼和 `15 rad/s` 最大转速；
- 清除轮关节额外摩擦，避免低速死区；
- 轮地静/动摩擦统一为 `0.20`；
- Articulation 求解迭代设为 position `32`、velocity `4`。

这些设置保证碰撞、驱动和低速响应稳定对称，但各向同性接触仍不能完整表达真实轮胎的纵横向摩擦差异，因此还需要受限运动补偿。

### 5.2 修复底盘控制图

[`isaac_sim/graphs/control_graph.py`](../isaac_sim/graphs/control_graph.py) 的控制拓扑改为：

- `OnPhysicsStep` 在每个物理步读取最新 Twist，而不是等待新 ROS 消息才执行；
- `deltaSimulationTime` 直接连接 `DifferentialController.dt`；
- 左右轮命令构造成 `[left, right, left, right]`；
- 使用一个 Articulation Controller 在同一图步原子写入四轮目标；
- Isaac 内层加速度限制提高为硬保护上限，正常导航平滑由 Nav2 Velocity Smoother 负责。

这样消除了命令频率对有效加速度的缩放，并改善了快速左右转向时的四轮同步。

### 5.3 增加有边界的滑移转向运动补偿

[`isaac_sim/src/robot/skid_steer_motion_assist.py`](../isaac_sim/src/robot/skid_steer_motion_assist.py) 在每个物理步比较目标和真实车体系速度，对平面线速度与偏航速度做加速度受限修正：

- 命令超过 `0.25 s` 未刷新时停止补偿；
- 最大线加速度修正为 `6.0 m/s²`；
- 最大角加速度修正为 `30.0 rad/s²`；
- 原地转向和行进圆弧使用分别标定后平滑混合的偏航比例；
- Idle Brake 生效时不执行运动补偿，Reset 时清空补偿状态。

四个轮子仍然接收差速目标并参与物理接触；该层只修正当前 PhysX 接触模型造成的系统性曲率欠响应。它的目标是让仿真底盘与 Nav2 使用的 `DiffDrive` 模型一致，而不是绕过超时、安全停车或 Reset 约束。

### 5.4 统一控制和里程计参数

[`isaac_sim/configs/robots/jackal.yaml`](../isaac_sim/configs/robots/jackal.yaml) 与 [`ros2_ws/src/robot_odometry/config/wheel_odometry.yaml`](../ros2_ws/src/robot_odometry/config/wheel_odometry.yaml) 同时使用 `0.800 m` 有效轮距。Realistic EKF 不再融合轮速差计算出的偏航速度，只融合轮里程计前向速度和 IMU 偏航速度。

必须保持这三个概念分离：

- `geometric_track_width = 0.37559 m`：真实轮心几何位置；
- `controller.wheel_distance = 0.800 m`：标定后的差速控制有效轮距；
- Wheel Odometry `track_width = 0.800 m`：与控制模型一致的轮速运动学参数。

### 5.5 重新配置 Nav2 以提前、连续地跟随曲率

[`ros2_ws/src/robot_navigation/config/nav2_params.yaml`](../ros2_ws/src/robot_navigation/config/nav2_params.yaml) 的主要最终值如下：

| 项目 | 原值 | 最终值 | 目的 |
| --- | ---: | ---: | --- |
| MPPI 控制频率 | 已有基线 10 Hz | 10 Hz | 在 Isaac 负载下稳定满足周期 |
| 预测窗 / batch | `20 × 0.10 s` / 1000 | `20 × 0.10 s` / 500 | 保留 2 秒视野并降低单周期计算量 |
| `vx_std` | 0.20 | 0.35 m/s | 扩大可用前进速度采样，避免窄道长期选择爬行轨迹 |
| `vx_max` | 1.00 | 0.75 m/s | 保留室内直线效率，同时由曲率与安全层动态降速 |
| `vx_min` | -0.20 | 0.00 m/s | 按本阶段要求关闭导航倒车采样 |
| `wz_std` | 0.40 | 0.80 rad/s | 增加有效转向候选 |
| `wz_max` | 1.50 | 1.20 rad/s | 与已验收底盘/平滑器范围一致 |
| `az_max` | 2.00 | 3.00 rad/s² | 更快建立和切换角速度 |
| `PathAngleCritic` 权重 | 2.0 | 9.5 | 更早修正路径朝向，并允许保持速度通过连续弯道 |
| 最大朝向偏差 | 1.0 | 0.45 rad | 避免偏差过大后才转向 |
| `PathAlign` / `PathFollow` 权重 | 10.0 / 5.0 | 5.0 / 9.0 | 避免过度奖励“对齐但缓慢”，强化沿路径推进 |
| `PathFollow` 前视 offset | 5 | 10 | 让优化器优先选择有实际前进量的轨迹 |
| 速度同比缩放 | false | true | 限速时保持 `wz / vx` 曲率 |

进度检查器改为 `PoseProgressChecker`，同时接受平移和旋转进展；Velocity Smoother 以 20 Hz 输出，线/角加速度分别受 `1.1 m/s²` 和 `3.0 rad/s²` 限制。MPPI 仍使用 `DiffDrive` 模型，并强化 PathAngle、PathAlign、PathFollow 与 PreferForward 的协同。

### 5.6 简化 Ideal 定位

Ideal Localization/Navigation 使用 [`ideal_localization_tf.py`](../ros2_ws/src/robot_bringup/robot_bringup/ideal_localization_tf.py) 以 20 Hz 发布新鲜 identity `map → odom`，不再在 Isaac Ideal `/odom` 上叠加 SLAM Toolbox 的扫描匹配修正。该节点仍提供清定位缓存的空操作服务，保持实验 Reset 事务接口不变。

## 6. 修复结果

### 6.1 底盘稳态响应

| 命令 | 修复后真实稳态响应 | 结果 |
| --- | --- | --- |
| 原地旋转 `0.00, +0.80 rad/s` | 约 `+0.819 rad/s` | 角速度误差约 2.4% |
| 前进圆弧 `+0.30 m/s, +0.80 rad/s` | 约 `+0.287 m/s, +0.806 rad/s` | 半径 `0.356 m`，接近期望 `0.375 m` |
| 倒车圆弧 `-0.25 m/s, +0.80 rad/s` | 约 `-0.239 m/s, +0.798 rad/s` | 半径 `0.299 m`，接近期望 `0.313 m` |
| 倒车直线 `-0.30 m/s` | 约 `-0.300 m/s` | 低速倒车抑制已消失 |

前进圆弧半径由修复前约 `2.68 m` 降到 `0.356 m`，从“大空间慢慢磨过去”变为能按指令跑出紧凑弧线。

### 6.2 自动底盘运动基准

[`motion_benchmark.yaml`](../ros2_ws/src/robot_experiments/config/motion_benchmark.yaml) 自动执行：

- 左/右原地旋转；
- 左/右前进圆；
- 左/右倒车圆；
- 倒车直线；
- 前进、倒车快速蛇形；
- `+1.2 ↔ -1.2 rad/s` 快速原地换向。

最终 `10/10` 运动原语通过，0 碰撞。判定覆盖线/角速度误差、半径误差、跟踪比例、换向延迟、超调和错误转向比例，不只是检查“最后是否到达”。

### 6.3 Ideal 复杂导航验收

本阶段按用户要求只验收 Ideal 里程计，并临时设置 `vx_min = 0.0` 关闭 MPPI 倒车采样。

| 场景 | 结果 | 路线与运动质量 |
| --- | --- | --- |
| 复杂静态路线 | `3/3` | 每轮 6/6 航点；平均 GT 路径 `50.132 m`；平均弧线距离占比 `30.8%`；平均停止时间占比 `2.81%`；0 恢复、0 碰撞、0 指令倒车 |
| 复杂动态路线 | `3/3` | 4 个一次性物理移动障碍；每轮 6/6 航点；平均 GT 路径 `50.108 m`；平均弧线距离占比 `29.4%`；平均停止时间占比 `4.16%`；0 恢复、0 碰撞、0 指令倒车 |

静态最终位置误差均值/最大值为 `0.132/0.143 m`，动态为 `0.137/0.152 m`。这些结果证明当前 Ideal 前进优先配置能够连续完成长距离、左右交替、多弯和动态避障路线，而不再依赖恢复行为把机器人从迟转中救回来。

### 6.4 回归测试

本次增量最终记录：

- `./scripts/preflight.sh`：PASS；
- `./scripts/build_ros2.sh`：9 个 ROS 包构建完成；
- root/pure pytest：317 passed，7 deselected；
- ROS colcon：326 tests，0 errors，0 failures，0 skipped；
- Isaac/USD Stage：4 passed，49 deselected；
- `git diff --check`：PASS。

完整证据台账见 [`verification.md`](verification.md)。

## 7. 当前范围与不能过度解读的结论

- 本轮最终复杂导航只测试 Ideal 里程计，不代表 Realistic 里程计已完成同一套 50 m 多航点泛化验收；
- 本阶段按要求关闭导航倒车采样，所以不能用 `3/3 + 3/3` 结果声称 Nav2 倒车路径选择已经完成最终验收；
- 底盘直接控制层的倒车直线、倒车圆和倒车蛇形已经通过，未来恢复 `vx_min < 0` 后仍应重新执行专门的导航倒车场景；
- 动态场景使用四个有界、一次性、非互惠的脚本物理障碍，不等同于任意人群行为泛化；
- 平面运动补偿是针对 Isaac Sim 6 当前各向同性接触模型的仿真修正。迁移到真车或支持各向异性轮胎模型的仿真器时，应重新辨识有效轮距、轮胎摩擦和控制带宽，而不是直接照搬补偿值。

## 8. 复现入口

完整启动和实验步骤见 [`user_manual.md`](user_manual.md)。最小回归命令为：

```bash
./scripts/preflight.sh
./scripts/build_ros2.sh
./scripts/test.sh --with-isaac
```

Ideal 复杂静态路线：

```bash
./scripts/run_experiment.sh \
  ros2_ws/src/robot_experiments/config/static_complex_route.yaml \
  data/experiment_runs/static_complex_route
```

Ideal 复杂动态路线要求 Isaac 使用对应的四障碍配置：

```bash
ISAAC_NAV__GROUND_TRUTH__ENABLED=true \
ISAAC_NAV__FILES__DYNAMIC_OBSTACLES="$PROJECT_ROOT/isaac_sim/configs/experiments/dynamic_complex_route.yaml" \
  ./scripts/run_isaac.sh --headless \
  --navigation-mode localization \
  --mode ideal \
  --dynamic-obstacles

./scripts/run_experiment.sh \
  ros2_ws/src/robot_experiments/config/dynamic_complex_route.yaml \
  data/experiment_runs/dynamic_complex_route
```

## 9. 后续调参原则

以后再出现“路径会转、车不转”的问题，应按以下顺序定位：

1. 先直接发送固定圆弧命令，用 Ground Truth 计算真实 `v`、`wz` 和半径；
2. 若底盘未执行命令，先查碰撞、轮速、控制时基和车体响应，不要先改 MPPI critic；
3. 底盘曲率通过后，再检查 `/cmd_vel_nav → /cmd_vel_smoothed → /cmd_vel` 各级是否保持曲率；
4. 检查 MPPI 是否按目标频率运行、是否存在 missed-control warning；
5. 最后才调整 PathAngle、PathAlign、速度上限和采样噪声，并用同一路线做修改前后对照；
6. 任何几何轮距、有效轮距、里程计或定位 TF 的变更，都必须同步检查控制与估计模型的一致性。

这一顺序能避免用上层参数掩盖底层执行故障，也能避免在底盘已经正常时反复改动物理参数。
