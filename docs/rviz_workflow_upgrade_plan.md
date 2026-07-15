# RViz 一体化建图、定位与导航工作流升级方案

## 1. 文档定位

本文档描述 `Isaac_Sim_ROS2_Nav` 项目在现有总体架构基础上的增量升级方案，重点解决以下问题：

- Fast DDS 共享内存残留与重复进程问题；
- Lifecycle 重复转换和启动竞态；
- 不同终端 ROS 环境不一致；
- RViz 地图、激光雷达和点云 QoS 不兼容；
- RViz 配置过于简单，无法观察完整建图、定位和导航过程；
- 导航目标仍依赖命令行发送；
- 建图过程依赖手工发布 `/cmd_vel` 指令；
- 仿真时间回跳后出现旧消息和 TF Cache 不匹配；
- MPPI 控制循环长期低于配置频率。

本文档不得替代或覆盖根目录中的 `plan.md`。

- `plan.md`：项目总体架构和完整建设规划；
- `docs/rviz_workflow_upgrade_plan.md`：本轮运行稳定性及 RViz 一体化交互工作流升级方案。

### 1.1 实施状态（2026-07-12）

本方案已经按冻结架构完成实现。本文后续章节保留实施前的问题分析和决策过程；日常操作以 [`user_manual.md`](user_manual.md) 为准，逐文件入口以 [`repository_index.md`](repository_index.md) 为准，最终测试数字和未验收边界以 [`verification.md`](verification.md) 为准。

| 阶段 | 结果 |
| --- | --- |
| 基础稳定性 | 已完成统一 Jazzy/Domain 42/Fast DDS 环境、任意目录入口、四类单实例 PID、只读诊断和有进程证明的 SHM 清理。 |
| Lifecycle/时间 | 已完成唯一 Lifecycle owner、原子状态快照、有限退避、异步代次令牌、Clock 回退/前跳/显式 Reset epoch 和完整 Reset 恢复序列。 |
| RViz | 已完成 `mapping.rviz`、`localization.rviz`、`navigation.rviz`，并锁定 Map 与传感器 QoS。 |
| 初始位姿 | 已完成 `auto/rviz` 所有权策略；RViz 2D Pose Estimate 不会被自动标定位姿覆盖，Reset 后按策略恢复。 |
| 导航目标 | 使用官方 Nav2 Navigation 2 Panel + GoalTool，已满足需求，因此按方案约束没有实现额外 Goal Bridge。 |
| Mapping Teleop | 已完成 W/A/S/D/方向键、0.18 秒稳态 deadman、速度上限、最终零速度、模式互斥和受管终端。 |
| 启动集成 | 四种 `run_ros.sh` 操作默认启动对应 RViz；Mapping 两种模式默认启动 Teleop；`interactive:=false` 可无头运行。 |
| MPPI | 实测选定 10 Hz、20×0.10 秒预测点、1000 batch；保持 2 秒窗，并把定位扫描处理降为每两帧一次。 |
| 文档 | 使用手册、逐文件索引、接口契约、排障手册、README 和验证台账均已同步。 |

实测中 Fast DDS SHM 残留、重复进程、Lifecycle 竞态、RViz QoS、仿真时间 epoch 和 MPPI 负载是可分别定位但会在启动/Reset 时相互放大的问题；修复保持原 TF/Topic 所有权、Ideal/Realistic、四种操作和 Ground Truth 隔离不变。

---

## 2. 当前环境

| 项目 | 配置 |
|---|---|
| 操作系统 | Ubuntu 24.04 |
| ROS 2 | Jazzy |
| ROS 安装路径 | `/opt/ros/jazzy` |
| Isaac Sim | 6.0.1 |
| Isaac Sim 安装方式 | Conda + pip |
| Conda 环境 | `isaacsim` |
| Python | 3.12 |
| GPU | NVIDIA RTX 4090 |
| 项目路径 | `/home/lyb/Workspace/Isaac_Sim_ROS2_Nav` |
| Isaac 资产路径 | `/home/lyb/isaacsim_assets/Assets/Isaac/6.0` |
| ROS Domain | `42` |
| RMW | `rmw_fastrtps_cpp` |
| DDS | Fast DDS |

---

## 3. 必须保留的架构约束

本轮升级不得通过改变现有系统架构来规避问题。

### 3.1 TF 主链

ROS 侧唯一导航主链保持为：

```text
map -> odom -> base_link -> wheel/sensor frames
```

ROS 中不增加 `world` Frame。

USD 中的 `/World` 是 Stage Prim 路径，不是 ROS TF Frame。

### 3.2 Ideal 模式

Ideal 模式保持以下所有权：

```text
Isaac Sim:
  /odom
  odom -> base_link

SLAM Toolbox:
  map -> odom
```

### 3.3 Realistic 模式

Realistic 模式保持以下所有权：

```text
Wheel Odometry:
  /wheel/odom

robot_localization EKF:
  /odom
  odom -> base_link

SLAM Toolbox:
  map -> odom
```

### 3.4 Mapping 模式

Mapping 模式：

* SLAM Toolbox 是 `/map` 的唯一发布者；
* Nav2 不启动；
* 键盘控制直接发布 `/cmd_vel`；
* Isaac Sim 接收 `/cmd_vel` 并驱动底盘；
* 不启动 Navigation Teleop 控制链。

### 3.5 Incremental Mapping 模式

Incremental Mapping 模式：

* 加载已有 Pose Graph；
* 在已有地图基础上继续建图；
* SLAM Toolbox 继续负责 `/map` 和 `map -> odom`；
* 使用与 Mapping 相同的键盘控制和 RViz 配置。

### 3.6 Localization 和 Navigation 模式

Localization 与 Navigation 模式：

* Map Server 是 `/map` 的唯一发布者；
* `/map` 是不可变静态地图；
* SLAM Toolbox localization 负责 `map -> odom`；
* SLAM Toolbox 实时诊断地图发布到：

```text
/slam_toolbox/map
```

`/slam_toolbox/map` 只用于诊断和可视化，不得接入 Nav2 Static Layer。

### 3.7 点云与 LaserScan

传感器处理链保持：

```text
Isaac RTX LiDAR
    -> /lidar/points_raw
pointcloud_to_laserscan
    -> /scan
```

### 3.8 Navigation 控制链

Navigation 模式控制链保持：

```text
Nav2 Controller
    -> /cmd_vel_nav

Velocity Smoother
    -> /cmd_vel_smoothed

Collision Monitor
    -> /cmd_vel

Isaac Sim Wheel Controller
```

### 3.9 Mapping 控制链

Mapping 和 Incremental Mapping 模式控制链：

```text
Keyboard Teleop
    -> /cmd_vel

Isaac Sim Wheel Controller
```

Navigation 模式不得启动 Mapping Teleop，避免多个 `/cmd_vel` 发布者。

---

## 4. 当前实际问题

## 4.1 Fast DDS SHM 初始化错误

运行 Isaac 和 ROS 时曾多次出现：

```text
[RTPS_TRANSPORT_SHM Error]
Failed init_port fastrtps_port7000:
open_and_lock_file failed -> Function open_port_internal

[RTPS_TRANSPORT_SHM Error]
Failed init_port fastrtps_port7001:
open_and_lock_file failed -> Function open_port_internal
```

涉及的进程包括：

```text
isaac_navigation_sim
pointcloud_to_laserscan
map_server
slam_toolbox
controller_server
planner_server
behavior_server
bt_navigator
velocity_smoother
collision_monitor
lifecycle_manager
nav2_activation_gate
```

需要分别检查：

* 上一轮 Isaac 或 ROS 进程是否残留；
* 是否重复启动同一套 ROS 栈；
* `/dev/shm` 中是否有 Fast DDS 残留；
* 文件是否属于 `root`；
* 是否曾使用 `sudo` 启动 ROS 或 Isaac；
* 是否有重复 DDS Participant；
* SHM 失败后通信是否回退到 UDP；
* SHM 错误是否与 Lifecycle 问题独立。

不得通过屏蔽日志处理。

不得在 DDS 活跃时删除 `/dev/shm` 文件。

---

## 4.2 Lifecycle 重复转换

曾出现：

```text
Failed to make transition 'TRANSITION_ACTIVATE'
for LifecycleNode '/map_server'

Failed to make transition 'TRANSITION_CONFIGURE'
for LifecycleNode '/slam_toolbox'
```

随后出现：

```text
No transition matching 3 found for current state active

Unable to start transition 3 from current state active:
Transition is not registered.
```

这说明节点已经处于 `active`，却再次收到了 `activate` 请求。

之后 Nav2 Activation Gate 请求启动：

```text
Readiness gate satisfied;
requesting Nav2 lifecycle STARTUP
```

Lifecycle Manager 失败：

```text
Failed to change state for node:
controller_server

Failed to bring up all requested nodes.
Aborting bringup.
```

Activation Gate 最终抛出：

```text
RuntimeError:
Nav2 lifecycle STARTUP request failed
```

需要明确以下节点的 Lifecycle 所有者：

| 节点                     | 生命周期所有者                           |
| ---------------------- | --------------------------------- |
| `map_server`           | 单一 Mapping/Localization Launch 机制 |
| `slam_toolbox`         | 单一 Mapping/Localization Launch 机制 |
| Nav2 Managed Nodes     | `lifecycle_manager_navigation`    |
| `nav2_activation_gate` | 仅负责 Readiness 检查和请求 STARTUP       |

禁止多个机制同时配置或激活同一 Lifecycle 节点。

---

## 4.3 ROS 环境在不同终端中不一致

未设置正确环境时，RViz 出现：

```text
No tf data.
Frame [map] does not exist
```

只能看到：

```text
/clicked_point
/goal_pose
/initialpose
```

看不到：

```text
/map
/odom
/scan
/tf
/tf_static
```

错误环境下执行导航目标时一直停在：

```text
Waiting for an action server to become available...
```

项目需要统一所有入口，避免每个终端手工执行：

```bash
source /opt/ros/jazzy/setup.bash
source "$PROJECT_ROOT/ros2_ws/install/setup.bash"

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

---

## 4.4 RViz `/map` QoS 错误

RViz 曾显示：

```text
Status: Warn
Topic: /map
Message: No map received
```

当时 RViz Map Display 使用：

```text
Reliability: Reliable
Durability: Volatile
```

对于晚于 Map Server 启动的 RViz，应使用与静态地图发布端兼容的：

```text
Reliability: Reliable
Durability: Transient Local
```

---

## 4.5 RViz `/scan` QoS 错误

出现：

```text
New publisher discovered on topic '/scan',
offering incompatible QoS.

No messages will be sent to it.

Last incompatible policy:
RELIABILITY_QOS_POLICY
```

RViz LaserScan 应根据真实发布端 QoS 配置，预期为：

```text
Reliability: Best Effort
Durability: Volatile
```

实际修改前必须通过以下命令确认：

```bash
ros2 topic info /scan -v
```

---

## 4.6 RViz 点云 QoS 错误

添加 `/lidar/points_raw` 后出现：

```text
New publisher discovered on topic '/lidar/points_raw',
offering incompatible QoS.

No messages will be sent to it.

Last incompatible policy:
RELIABILITY_QOS_POLICY
```

PointCloud2 显示：

```text
Showing [0] points from [0] messages
```

预期 RViz PointCloud2 使用：

```text
Reliability: Best Effort
Durability: Volatile
```

实际修改前需要确认：

```bash
ros2 topic info /lidar/points_raw -v
```

---

## 4.7 仿真时间和 TF Cache 不一致

曾出现：

```text
Message Filter dropping message:
frame 'base_link'
at time 181.200

for reason:
the timestamp on the message is earlier than all the data
in the transform cache
```

该问题出现在：

```text
slam_toolbox
local_costmap
global_costmap
```

常见触发场景：

* Isaac 已运行较长时间后单独重启 ROS；
* Isaac 仿真时间发生 Reset；
* `/clock` 回退；
* 旧传感器消息与新 TF 属于不同时间区间；
* Readiness Gate 沿用了旧状态。

---

## 4.8 MPPI 控制频率不足

配置目标：

```yaml
controller_frequency: 20.0
```

实际多次出现：

```text
Control loop missed its desired rate of 20.0000 Hz.
Current loop rate is 8.5714 Hz.
```

还出现：

```text
5.0000 Hz
4.6154 Hz
```

当前 MPPI 参数大致为：

```yaml
FollowPath:
  time_steps: 40
  model_dt: 0.05
  batch_size: 1500
  iteration_count: 1
```

虽然导航最终成功：

```text
Goal succeeded
error_code: 0
status: SUCCEEDED
```

但长期低于配置频率可能影响动态避障和高速运动安全。

---

# 5. 最终用户工作流

升级完成后，正常用户只需启动两个主要进程：

1. Isaac Sim；
2. 对应模式的 ROS 启动脚本。

ROS 启动脚本负责：

* 启动对应 ROS 模式；
* 自动启动正确的 RViz 配置；
* Mapping 时自动启动键盘控制；
* Localization 时支持 2D Pose Estimate；
* Navigation 时支持 RViz 直接发布导航目标；
* 统一 ROS Domain 和 RMW；
* 管理所有辅助进程退出。

正常用户不再需要：

* 手工启动 RViz；
* 手工添加 RViz Display；
* 手工修改 RViz QoS；
* 使用 `ros2 action send_goal` 作为日常导航方式；
* 使用 `ros2 topic pub` 作为日常建图控制方式；
* 在每个终端重复设置 ROS 环境。

命令行 Goal 仅保留给：

* 自动测试；
* CI；
* Headless；
* 故障诊断。

---

# 6. 统一环境脚本

检查并完善：

```text
scripts/lib/common.sh
```

或等效公共脚本。

公共逻辑必须：

1. 根据脚本自身路径解析 `PROJECT_ROOT`；
2. 不依赖调用者当前目录；
3. 加载：

```bash
source /opt/ros/jazzy/setup.bash
source "$PROJECT_ROOT/ros2_ws/install/setup.bash"
```

4. 设置：

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

5. 验证：

```text
ROS_DISTRO=jazzy
install/setup.bash 存在
必要配置文件存在
项目路径有效
```

6. 提供统一函数：

```text
log_info
log_warn
die
require_file
require_command
```

所有 Shell 脚本使用：

```bash
set -Eeuo pipefail
```

需要检查或新增：

```text
scripts/run_isaac.sh
scripts/run_ros.sh
scripts/run_rviz.sh
scripts/run_teleop.sh
scripts/preflight.sh
scripts/diagnose.sh
scripts/clean_runtime.sh
scripts/save_map.sh
scripts/test.sh
```

---

# 7. `run_ros.sh` 自动启动 RViz

`run_ros.sh` 不再只启动 ROS 节点。

需要支持参数：

```text
use_rviz:=true
rviz_config:=auto
use_teleop:=auto
initial_pose_source:=auto
interactive:=true
```

## 7.1 参数语义

### `use_rviz`

```text
true:
  自动启动 RViz

false:
  不启动 RViz
```

`false` 用于：

* CI；
* Headless；
* 自动实验；
* 性能测试。

### `rviz_config`

```text
auto:
  根据 operation 自动选择
```

映射关系：

```text
mapping             -> mapping.rviz
incremental_mapping -> mapping.rviz
localization        -> localization.rviz
navigation          -> navigation.rviz
```

同时允许用户传入自定义配置文件。

### `use_teleop`

```text
auto:
  mapping             -> true
  incremental_mapping -> true
  localization        -> false
  navigation          -> false
```

### `initial_pose_source`

```text
auto:
  使用项目保存的 calibrated initial pose

rviz:
  等待用户通过 RViz 2D Pose Estimate 发布初始位姿
```

### `interactive`

```text
true:
  启动 RViz 和模式对应的交互组件

false:
  不启动 RViz 和 Teleop
```

## 7.2 RViz 进程管理

优先在顶层 Launch 中将 RViz 作为受管理节点启动：

```python
Node(
    package='rviz2',
    executable='rviz2',
    arguments=['-d', rviz_config],
    parameters=[{'use_sim_time': use_sim_time}],
    output='screen',
)
```

要求：

* RViz 继承正确的 ROS Domain；
* RViz 使用仿真时间；
* 主 ROS 栈退出时 RViz 一并退出；
* 用户主动关闭 RViz时，不强制关闭整个 ROS 栈；
* 不遗留多个 RViz 进程。

同时保留独立入口：

```bash
scripts/run_rviz.sh mapping
scripts/run_rviz.sh localization
scripts/run_rviz.sh navigation
```

---

# 8. RViz 配置拆分

新增或完善：

```text
ros2_ws/src/robot_description/rviz/mapping.rviz
ros2_ws/src/robot_description/rviz/localization.rviz
ros2_ws/src/robot_description/rviz/navigation.rviz
```

可选增加：

```text
ros2_ws/src/robot_description/rviz/debug.rviz
```

检查：

```text
ros2_ws/src/robot_description/CMakeLists.txt
```

确保 `rviz/` 安装到：

```text
share/robot_description/rviz
```

构建后应能使用：

```bash
$(ros2 pkg prefix robot_description)/share/robot_description/rviz/mapping.rviz
```

---

# 9. Mapping RViz 配置

`mapping.rviz` 用于显示完整建图过程。

## 9.1 Global Options

```text
Fixed Frame: map
```

默认视图：

```text
TopDownOrtho
```

视角应适合查看仓库地图扩展。

## 9.2 Panels

至少包含：

```text
Displays
Selection
Tool Properties
Views
Time
```

如果当前 Jazzy 中 SLAM Toolbox RViz Panel 可用，可加入；插件缺失不得阻止 RViz 启动。

## 9.3 Tools

至少包含：

```text
Interact
Move Camera
Select
Focus Camera
Measure
Publish Point
```

Mapping 模式不依赖导航 Goal Tool。

## 9.4 Displays

### Grid

默认开启。

### RobotModel

```text
Description Topic: /robot_description
```

默认开启。

### TF

默认开启，显示：

```text
map
odom
base_link
wheel links
lidar_link
imu_link
camera links
```

### 实时地图

```text
Topic: /map
```

Mapping 模式中由 SLAM Toolbox 发布。

QoS 根据实际发布端确认，预期：

```text
Reliable
Transient Local
```

默认开启。

### LaserScan

```text
Topic: /scan
```

预期 QoS：

```text
Best Effort
Volatile
```

默认开启。

### PointCloud2

```text
Topic: /lidar/points_raw
```

预期 QoS：

```text
Best Effort
Volatile
```

默认关闭，但完整预配置。

### Odometry

```text
Topic: /odom
```

用于显示机器人运动方向和轨迹。

### Ground Truth Path

```text
Topic: /ground_truth/path
```

仅在实际发布时配置，默认关闭。

### Ground Truth Odom

```text
Topic: /ground_truth/odom
```

默认关闭。

## 9.5 Mapping 可视化目标

用户应能实时看到：

* 地图逐步扩展；
* 未知区域逐渐变为自由或占用；
* 激光扫描；
* 原始点云；
* 机器人模型；
* `map -> odom -> base_link`；
* 机器人移动轨迹；
* 仓库货架和障碍轮廓。

---

# 10. Localization RViz 配置

`localization.rviz` 用于定位和人工重定位。

## 10.1 Displays

至少包含：

```text
Grid
RobotModel
TF
/map
/slam_toolbox/map
/scan
/lidar/points_raw
/odom
/ground_truth/path
/ground_truth/odom
```

其中：

### 静态地图

```text
Topic: /map
```

默认开启。

### SLAM Toolbox 诊断地图

```text
Topic: /slam_toolbox/map
```

默认关闭。

不得将该地图误认为 Nav2 静态地图。

### LaserScan

```text
Topic: /scan
```

默认开启。

### PointCloud2

```text
Topic: /lidar/points_raw
```

默认关闭。

## 10.2 2D Pose Estimate

必须配置 RViz 的：

```text
2D Pose Estimate
```

发布：

```text
Topic: /initialpose
Type: geometry_msgs/msg/PoseWithCovarianceStamped
Frame: map
```

验收要求：

1. RViz 点击并拖动方向后发布 `/initialpose`；
2. `frame_id` 为 `map`；
3. Quaternion 合法；
4. SLAM Toolbox 收到消息；
5. `map -> odom` 更新；
6. RobotModel 移动到新的估计位置；
7. 自动初始位姿不会持续覆盖人工位姿。

---

# 11. 初始位姿策略

新增参数：

```text
initial_pose_source:=auto
initial_pose_source:=rviz
```

## 11.1 Auto 模式

```text
initial_pose_source:=auto
```

行为：

1. 使用保存的 calibrated Map Pose；
2. 自动发布有限次数；
3. 完成后自动节点退出或停止发布；
4. 用户之后仍可通过 2D Pose Estimate 重新定位；
5. 自动节点不得周期性覆盖人工位姿。

## 11.2 RViz 模式

```text
initial_pose_source:=rviz
```

行为：

1. 不自动发布初始位姿；
2. RViz 自动打开；
3. 系统等待用户点击 2D Pose Estimate；
4. `/initialpose` 发布；
5. SLAM Toolbox 建立 `map -> odom`；
6. Localization 进入正常状态；
7. Navigation Activation Gate 等待 TF 稳定后激活 Nav2。

默认保留 `auto`，以兼容自动实验和快速启动。

---

# 12. Navigation RViz 配置

`navigation.rviz` 必须显示完整定位、感知、规划、控制和避障过程。

## 12.1 基本显示

### Grid

默认开启。

### RobotModel

```text
Topic: /robot_description
```

默认开启。

### TF

默认开启。

### Static Map

```text
Topic: /map
```

默认开启。

QoS：

```text
Reliable
Transient Local
```

### SLAM Toolbox Diagnostic Map

```text
Topic: /slam_toolbox/map
```

默认关闭。

### LaserScan

```text
Topic: /scan
```

默认开启。

### PointCloud2

```text
Topic: /lidar/points_raw
```

默认关闭。

### Odometry

```text
Topic: /odom
```

默认开启。

## 12.2 Costmap

### Global Costmap

确认实际 Topic，通常为：

```text
/global_costmap/costmap
```

默认开启。

设置较低 Alpha，避免完全遮挡静态地图。

### Local Costmap

确认实际 Topic，通常为：

```text
/local_costmap/costmap
```

默认开启。

局部代价地图与全局代价地图使用不同层次或透明度。

## 12.3 Footprint

配置：

```text
/global_costmap/published_footprint
/local_costmap/published_footprint
```

使用 Polygon Display。

## 12.4 Global Plan

必须通过实际运行确认 Topic：

```bash
ros2 topic list
TOPIC=/plan  # replace after inspecting the list above
ros2 topic type "$TOPIC"
ros2 topic info -v "$TOPIC"
```

可能为：

```text
/plan
```

但不得在未确认前硬编码。

使用 Path Display，默认开启。

## 12.5 Local Plan 或局部控制轨迹

必须检查 MPPI 当前实际发布接口。

优先级：

1. 显示当前控制器选择的实际局部路径；
2. 显示 MPPI 最优轨迹；
3. 最后才考虑候选轨迹集合。

不得为了显示局部轨迹而默认开启全部 MPPI 候选轨迹。

如启用 MPPI `visualize`：

* 对应 Marker Display 默认关闭；
* 评估其对控制频率的影响；
* 性能基准中分别测试开关状态。

## 12.6 Collision Monitor

根据实际 Topic 配置：

```text
/collision_monitor/stop_zone
/collision_monitor/slowdown_zone
```

Approach Zone 需要检查当前插件是否能够发布可视化 Polygon。

如原节点不直接发布：

* 优先复用 Footprint；
* 或新增轻量、只读的可视化节点；
* 不改变 Collision Monitor 安全逻辑。

Collision Points Marker 也必须根据实际发布 Topic 配置。

## 12.7 Ground Truth

预配置：

```text
/ground_truth/path
/ground_truth/odom
```

默认关闭。

## 12.8 Panels

Navigation RViz 至少包含：

```text
Displays
Selection
Tool Properties
Views
Time
Nav2 Panel
```

依赖：

```text
ros-jazzy-nav2-rviz-plugins
```

Preflight 必须检查该包。

---

# 13. RViz 导航目标交互

用户日常导航不再使用：

```bash
ros2 action send_goal /navigate_to_pose ...
```

标准方式：

1. 启动 Isaac；
2. 启动 Navigation ROS 栈；
3. RViz 自动打开；
4. 等待 Nav2 激活；
5. 点击 Nav2 Goal Tool 或 2D Goal Pose；
6. 在地图中点击目标位置并拖动设置方向；
7. 机器人开始导航。

命令行 Action 只保留给自动测试和 Headless 场景。

---

# 14. 2D Goal Pose 实现

## 14.1 优先方案：官方 Nav2 RViz Plugin

优先使用 ROS 2 Jazzy 的：

```text
nav2_rviz_plugins
```

目标工具应直接发送：

```text
Action: /navigate_to_pose
Type: nav2_msgs/action/NavigateToPose
Frame: map
```

实施前必须确认：

1. 插件包是否安装；
2. 当前 Jazzy 的实际插件类名；
3. Plugin Description；
4. 是否需要 Nav2 Panel；
5. Goal Tool 是否直接调用 Action；
6. Goal 取消方式；
7. RViz 配置文件中的正确 Class；
8. 一个点击是否只产生一个 Goal。

不得直接照搬其他 ROS 版本的插件类名。

## 14.2 备用方案：Goal Pose Bridge

只有确认官方 Nav2 Goal Tool 无法满足当前流程时，才实现 Bridge：

```text
RViz 2D Goal Pose
    -> /goal_pose
goal_pose_bridge
    -> /navigate_to_pose
```

Bridge 只在 Navigation 模式启动。

要求：

1. 订阅：

```text
/goal_pose
geometry_msgs/msg/PoseStamped
```

2. 创建 ActionClient：

```text
/navigate_to_pose
nav2_msgs/action/NavigateToPose
```

3. 默认只接受 `map` Frame；
4. 验证 Pose 和 Quaternion；
5. 有界等待 Action Server；
6. Action Server 不可用时明确报错；
7. Goal accepted/rejected/result 均输出日志；
8. 新 Goal 到来时安全取消或替换旧 Goal；
9. 不阻塞 Executor；
10. 不重复转发同一个 RViz 点击；
11. 使用 `use_sim_time`；
12. 添加完整测试。

官方 Goal Tool 和 Bridge 不得同时启用。

---

# 15. Navigation 激活前的 RViz 行为

RViz 可以在 Nav2 激活前启动，以显示：

* 地图；
* RobotModel；
* TF；
* Scan；
* 定位状态。

在 Action Server 尚未就绪时：

* Nav2 Panel 应显示未就绪；
* Goal Tool 不得静默失败；
* 不得无限等待；
* 不得丢弃 Goal 而无任何日志。

Nav2 完成激活：

```text
Nav2 lifecycle activation completed
```

后，Goal Tool 应立即可用。

---

# 16. Mapping 键盘控制

当前通过：

```bash
ros2 topic pub --rate 10 /cmd_vel ...
```

控制机器人，不适合遍历整个仓库。

Mapping 和 Incremental Mapping 模式必须自动提供交互式键盘控制。

## 16.1 按键设计

至少支持：

| 按键        | 功能    |
| --------- | ----- |
| `W` / `↑` | 前进    |
| `S` / `↓` | 后退    |
| `A` / `←` | 左转    |
| `D` / `→` | 右转    |
| `Space`   | 立即停止  |
| `Q`       | 停止并退出 |
| `Ctrl+C`  | 停止并退出 |

可增加：

| 按键    | 功能       |
| ----- | -------- |
| `R/F` | 增加/降低线速度 |
| `T/G` | 增加/降低角速度 |

建议默认值：

```text
linear_speed  = 0.30 m/s
angular_speed = 0.60 rad/s
```

必须受项目速度限制约束。

---

# 17. Teleop 节点实现

先评估 `teleop_twist_keyboard`。

由于项目要求明确的 W/A/S/D 控制，如果标准包不满足操作体验，则新增项目 Teleop 节点，例如：

```text
ros2_ws/src/robot_teleop/
```

或：

```text
keyboard_teleop.py
```

要求：

1. 使用终端原始输入或 `curses`；
2. 支持 W/A/S/D；
3. 固定频率发布 Twist；
4. 发布：

```text
/cmd_vel
geometry_msgs/msg/Twist
```

5. 无按键超过安全超时后自动发布零速度；
6. Space 立即发布零速度；
7. Q 退出前发布零速度；
8. Ctrl+C 退出前发布零速度；
9. 异常退出时尽可能发布零速度；
10. 安全超时使用 Wall Monotonic Time；
11. 不依赖可能暂停或回跳的 `/clock`；
12. 不绕过 Isaac 侧 CmdVel Watchdog；
13. 对线速度和角速度限幅；
14. 终端持续显示当前速度和操作说明；
15. 添加单元测试。

建议超时：

```text
0.3～0.5 秒
```

实际值应与 Isaac Watchdog 协调。

---

# 18. Teleop 独立终端

`run_ros.sh` 需要显示 ROS 日志，不能在同一终端读取键盘。

Mapping 模式自动启动独立终端。

终端优先级：

```text
gnome-terminal
xterm
konsole
```

Ubuntu GNOME 优先：

```text
gnome-terminal
```

窗口标题：

```text
Isaac Nav - Mapping Teleop
```

终端中运行：

```bash
scripts/run_teleop.sh
```

要求：

* 继承 ROS Domain 和 RMW；
* 自动加载工作空间；
* 主 ROS 栈退出时关闭本次启动的 Teleop；
* Teleop 退出时发送零速度；
* 不使用宽泛 `pkill`；
* 仅管理本次启动的 PID 或进程组。

找不到终端模拟器时：

1. 明确报错；
2. 打印手工启动命令；
3. 不静默跳过；
4. `use_teleop:=false` 时允许继续运行。

---

# 19. Teleop 与 Navigation 互斥

启动 Navigation 前检查：

```bash
ros2 topic info /cmd_vel -v
```

如果存在额外人工 Teleop 发布者：

* 拒绝启动 Navigation；
* 或明确提示关闭 Teleop；
* 不允许 Teleop 与 Collision Monitor 同时控制最终 `/cmd_vel`。

Mapping：

```text
Teleop -> /cmd_vel
```

Navigation：

```text
Nav2 -> Velocity Smoother -> Collision Monitor -> /cmd_vel
```

---

# 20. RViz QoS 配置

修改前必须确认：

```bash
ros2 topic info /map -v
ros2 topic info /scan -v
ros2 topic info /lidar/points_raw -v
ros2 topic info /odom -v
ros2 topic info /global_costmap/costmap -v
ros2 topic info /local_costmap/costmap -v
```

预期配置：

| Topic               | Reliability  | Durability      |
| ------------------- | ------------ | --------------- |
| `/map`              | Reliable     | Transient Local |
| `/scan`             | Best Effort  | Volatile        |
| `/lidar/points_raw` | Best Effort  | Volatile        |
| `/tf`               | 与动态 TF 发布端兼容 | Volatile        |
| `/tf_static`        | Reliable     | Transient Local |
| Costmap             | 根据实际发布端      | 根据实际发布端         |

完成后不得继续出现：

```text
incompatible QoS
RELIABILITY_QOS_POLICY
```

晚启动的 RViz 也必须能收到 `/map`。

---

# 21. RViz 默认显示和性能

RViz 配置应完整，但不得默认开启所有高负载显示。

## 21.1 默认开启

```text
Grid
Static/Mapping Map
RobotModel
TF
LaserScan
Odometry
Global Plan
Local Plan
Global Costmap
Local Costmap
Footprint
Collision Monitor Zones
```

## 21.2 默认关闭但预配置

```text
PointCloud2
Ground Truth Path
Ground Truth Odom
SLAM Diagnostic Map
MPPI Candidate Trajectories
高频 Debug Marker
```

## 21.3 显示样式

要求：

* 静态地图使用灰度显示；
* Costmap Alpha 不完全遮挡地图；
* Global Plan 和 Local Plan 视觉明显区分；
* LaserScan 点大小清晰；
* PointCloud2 设置合理 Size 和 Decay Time；
* Stop、Slowdown、Approach 区域视觉可区分；
* 默认视图为 `TopDownOrtho`；
* 在 1920×1080 分辨率下可正常使用。

---

# 22. Lifecycle 稳定性修复

## 22.1 所有权

必须保证：

```text
map_server:
  单一生命周期所有者

slam_toolbox:
  单一生命周期所有者

Nav2 Managed Nodes:
  lifecycle_manager_navigation

nav2_activation_gate:
  Readiness 检查和 STARTUP 请求
```

## 22.2 Activation Gate 条件

STARTUP 前必须确认：

1. `/clock` 非零；
2. `/clock` 新鲜；
3. `/scan` 新鲜；
4. `/odom` 新鲜；
5. 已收到 `/map`；
6. `map -> odom` 存在；
7. TF 时间戳新鲜；
8. TF 在指定时间窗口内稳定；
9. Lifecycle Manager Service 可用；
10. 不存在重复目标节点。

## 22.3 幂等规则

```text
unconfigured + 目标 active:
  configure
  activate

inactive + 目标 active:
  activate

active + 目标 active:
  直接视为成功

finalized/error/unknown:
  终止并输出诊断
```

## 22.4 STARTUP 失败诊断

失败时不能只输出：

```text
Nav2 lifecycle STARTUP request failed
```

必须输出：

* Service 返回结果；
* 受管节点列表；
* 各节点当前 Lifecycle 状态；
* 哪个节点未达到 Active；
* 是否存在重复节点；
* 是否准备重试；
* 当前重试次数；
* 下一次退避时间；
* 最终失败原因。

允许有限重试和退避。

不得无限重试。

不得使用固定 `sleep` 替代状态判断。

---

# 23. 仿真时间回跳处理

需要检测：

1. `/clock` 小于上一帧；
2. `/clock` 从较大值回到零；
3. `/clock` 异常大幅跳变；
4. Scan 时间戳早于 TF Cache；
5. Odom 时间戳属于旧时间区间。

发生回跳后：

1. 清空 Readiness；
2. 清空 TF 稳定累计；
3. 标记旧 `/scan` 无效；
4. 标记旧 `/odom` 无效；
5. 等待新时间区间的 Scan；
6. 等待新时间区间的 Odom；
7. 等待新的 `map -> odom`；
8. 必要时取消当前 NavigateToPose Goal；
9. 清理 Global Costmap；
10. 清理 Local Costmap；
11. 重新执行初始位姿流程；
12. 不允许使用旧状态重新激活 Nav2；
13. 输出明确恢复日志。

不得仅通过无限增大 TF Buffer 解决。

---

# 24. Fast DDS 运行时清理

新增或完善：

```text
scripts/clean_runtime.sh
```

支持：

```bash
scripts/clean_runtime.sh --dry-run
scripts/clean_runtime.sh
scripts/clean_runtime.sh --dds-shm
```

必须：

1. 检查项目相关进程；
2. 检查 Isaac；
3. 检查 ROS Launch；
4. 检查 RViz；
5. 检查 Teleop；
6. 检查重复 ROS 节点；
7. 检查 Fast DDS SHM 文件；
8. 检查文件所有者；
9. 活跃 DDS 进程存在时拒绝删除；
10. `root` 文件只打印人工处理命令；
11. 不自动使用 `sudo`；
12. 不误杀其他 ROS 项目；
13. 输出清理前后状态。

---

# 25. 单实例保护

使用：

```text
flock
PID file
或等效可靠机制
```

锁目录建议：

```text
/tmp/isaac_sim_ros2_nav_${UID}/
```

至少保护：

```text
Isaac
ROS bringup
RViz
Teleop
```

第二次启动应明确提示：

```text
ROS stack is already running
```

不得生成重复节点。

---

# 26. Preflight

扩展：

```text
scripts/preflight.sh
```

检查：

* `PROJECT_ROOT`；
* `ROS_DISTRO`；
* `ROS_DOMAIN_ID`；
* `RMW_IMPLEMENTATION`；
* `install/setup.bash`；
* Isaac Conda 环境；
* Isaac 资产路径；
* Pose Graph；
* Map YAML/PGM；
* RViz；
* `nav2_rviz_plugins`；
* Teleop 依赖；
* 终端模拟器；
* 重复进程；
* 重复 ROS 节点；
* Fast DDS SHM 残留；
* `root` 所有权文件；
* CPU Governor；
* 三个 RViz 配置；
* Goal Tool 插件；
* 所有模式所需 ROS Package。

Preflight 只检查，不修改系统状态。

---

# 27. Diagnose

新增或完善：

```text
scripts/diagnose.sh
```

输出：

## 环境

```text
PROJECT_ROOT
ROS_DISTRO
ROS_DOMAIN_ID
RMW_IMPLEMENTATION
```

## 进程

```text
Isaac
ROS
RViz
Teleop
```

## ROS Graph

```text
节点列表
重复节点
关键 Topic
关键 Service
关键 Action
```

## Lifecycle

```text
map_server
slam_toolbox
controller_server
planner_server
bt_navigator
collision_monitor
velocity_smoother
```

## QoS

```text
/map
/scan
/lidar/points_raw
/odom
/tf
/tf_static
```

## 交互接口

```text
/initialpose
/goal_pose
/navigate_to_pose
```

## TF

```text
map -> odom
odom -> base_link
map -> base_link
```

## 时间

```text
/clock
/scan stamp
/odom stamp
TF stamp
```

## 系统状态

```text
DDS SHM
文件所有者
CPU governor
最近 ROS ERROR/WARN
```

---

# 28. MPPI 性能调优

必须先测量，再修改默认参数。

记录：

* `controller_frequency`；
* 实际平均频率；
* 实际最低频率；
* Missed Rate 次数；
* CPU 占用；
* GPU 占用；
* Isaac RTF；
* RViz 是否开启；
* PointCloud2 是否开启；
* MPPI Visualize 是否开启；
* 导航耗时；
* Goal 是否成功；
* Recovery 次数。

至少比较：

## 方案 A

```yaml
controller_frequency: 20.0

FollowPath:
  model_dt: 0.05
  time_steps: 40
  batch_size: 1000
```

## 方案 B

```yaml
controller_frequency: 20.0

FollowPath:
  model_dt: 0.05
  time_steps: 40
  batch_size: 750
```

## 方案 C

```yaml
controller_frequency: 10.0

FollowPath:
  model_dt: 0.10
  time_steps: 20
  batch_size: 1000
```

三个方案都保持约 2 秒预测时域。

最终选择真实能够稳定达到目标频率的方案。

不得：

* 屏蔽 Missed Rate 日志；
* 未测试就降低频率；
* 只修改 `controller_frequency` 而忽略 `model_dt`；
* 默认开启全部候选轨迹；
* 为 RViz 视觉效果牺牲控制实时性。

---

# 29. 自动测试

## 29.1 脚本路径测试

分别从：

```text
项目根目录
/home/lyb
/tmp
```

调用脚本，确认 `PROJECT_ROOT` 正确。

## 29.2 环境测试

确认所有入口使用：

```text
ROS_DOMAIN_ID=42
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

## 29.3 RViz 配置测试

验证：

```text
mapping.rviz 存在
localization.rviz 存在
navigation.rviz 存在
```

检查：

* Fixed Frame 为 `map`；
* RobotModel 存在；
* TF 存在；
* `/map` 存在；
* `/scan` 存在；
* `/lidar/points_raw` 存在；
* `/map` 为 Transient Local；
* `/scan` 为 Best Effort；
* `/lidar/points_raw` 为 Best Effort；
* Navigation 配置包含 Costmap；
* 包含 Global Plan；
* 包含 Local Plan 或 Local Trajectory；
* 包含 Footprint；
* 包含 Collision Monitor；
* 包含 2D Pose Estimate；
* 包含 Nav2 Goal Tool 或 2D Goal Pose；
* RViz 目录被 CMake 安装。

## 29.4 Initial Pose 测试

覆盖：

```text
initial_pose_source:=auto
initial_pose_source:=rviz
```

验证：

* `/initialpose`；
* `frame_id=map`；
* Quaternion 合法；
* 自动节点有限次数后停止；
* 人工消息不会被覆盖；
* `map -> odom` 建立。

## 29.5 Goal Tool 测试

官方插件：

* Plugin 可以加载；
* 一个点击只产生一个 Goal；
* Goal 可以取消；
* 新 Goal 可以替换旧 Goal。

如使用 Bridge，还需测试：

* 正确 Pose 转发；
* 错误 Frame；
* 非法 Quaternion；
* Action Server 不可用；
* Goal rejected；
* Goal accepted；
* Goal succeeded；
* 不重复发送。

## 29.6 Teleop 测试

测试：

```text
W 前进
S 后退
A 左转
D 右转
Space 停止
Q 停止并退出
Ctrl+C 停止
输入超时停止
速度限幅
退出时最终 Twist 为零
```

确认：

* Mapping 默认启动；
* Navigation 不启动；
* Localization 不启动。

## 29.7 Lifecycle 测试

覆盖：

```text
unconfigured
inactive
active
重复 STARTUP
Service 返回 false
Service 暂不可用
error
finalized
时间回跳
有限重试
重复节点
```

## 29.8 所有权测试

Mapping：

```text
/map 只有 SLAM Toolbox 发布
```

Localization/Navigation：

```text
/map 只有 Map Server 发布
/slam_toolbox/map 只有 SLAM Toolbox 发布
```

Ideal：

```text
/odom 只有 Isaac 发布
odom -> base_link 只有 Isaac 发布
```

Realistic：

```text
/odom 只有 EKF 发布
odom -> base_link 只有 EKF 发布
```

## 29.9 重复启动测试

验证：

* 第二次 ROS Bringup 被拒绝；
* 第二次 Teleop 被拒绝；
* RViz 单实例行为符合设计；
* 不出现重复节点。

## 29.10 连续重启测试

至少连续执行五次：

```text
启动 ROS
等待 Ready
检查 Lifecycle
检查 RViz
停止 ROS
再次启动
```

不得出现非法 Lifecycle Transition。

---

# 30. 人工验收流程

## 30.1 Mapping

终端 A：

```bash
./scripts/run_isaac.sh \
  --navigation-mode mapping \
  --mode ideal
```

等待：

```text
Isaac navigation simulation ready
```

终端 B：

```bash
./scripts/run_ros.sh mapping \
  odometry_mode:=ideal
```

预期：

1. ROS Mapping 启动；
2. `mapping.rviz` 自动打开；
3. Teleop 独立终端自动打开；
4. W/A/S/D 控制机器人；
5. Space 立即停车；
6. `/map` 实时扩展；
7. `/scan` 正常显示；
8. PointCloud2 勾选后正常显示；
9. RobotModel 正常；
10. TF 正常；
11. 不出现 QoS 不兼容；
12. Teleop 退出后机器人停止。

## 30.2 Incremental Mapping

```bash
./scripts/run_isaac.sh \
  --navigation-mode mapping \
  --mode ideal
```

```bash
./scripts/run_ros.sh incremental_mapping \
  odometry_mode:=ideal \
  posegraph_file:=data/maps/posegraphs/warehouse_v1
```

预期：

1. 加载已有 Pose Graph；
2. RViz 显示已有地图；
3. Teleop 自动启动；
4. 可继续探索未建区域；
5. 地图继续更新；
6. 保存后生成新的 Pose Graph 和 Occupancy Map。

## 30.3 Localization Auto

```bash
./scripts/run_isaac.sh \
  --navigation-mode localization \
  --mode ideal
```

```bash
./scripts/run_ros.sh localization \
  odometry_mode:=ideal \
  posegraph_file:=data/maps/posegraphs/warehouse_v1 \
  initial_pose_source:=auto
```

预期：

1. `localization.rviz` 自动打开；
2. 静态地图显示；
3. 自动初始位姿生效；
4. RobotModel 定位正确；
5. `map -> odom` 正常；
6. 用户仍可使用 2D Pose Estimate 修正定位。

## 30.4 Localization RViz

```bash
./scripts/run_ros.sh localization \
  odometry_mode:=ideal \
  posegraph_file:=data/maps/posegraphs/warehouse_v1 \
  initial_pose_source:=rviz
```

预期：

1. RViz 自动打开；
2. 系统等待人工位姿；
3. 用户点击 2D Pose Estimate；
4. `/initialpose` 发布；
5. SLAM Toolbox 接收；
6. `map -> odom` 建立；
7. RobotModel 定位到选择位置。

## 30.5 Navigation

终端 A：

```bash
./scripts/run_isaac.sh \
  --navigation-mode localization \
  --mode ideal
```

终端 B：

```bash
./scripts/run_ros.sh navigation \
  odometry_mode:=ideal \
  posegraph_file:=data/maps/posegraphs/warehouse_v1
```

预期：

1. `navigation.rviz` 自动打开；
2. 静态地图显示；
3. RobotModel 显示；
4. TF 显示；
5. Scan 显示；
6. PointCloud2 可选显示；
7. Global Costmap 显示；
8. Local Costmap 显示；
9. Global Plan 已配置；
10. Local Plan 或 Local Trajectory 已配置；
11. Footprint 显示；
12. Collision Monitor 区域显示；
13. Nav2 Panel 显示；
14. 2D Pose Estimate 可用；
15. Nav2 Goal Tool 或 2D Goal Pose 可用；
16. 不需要命令行发送 Goal。

用户在 RViz 中：

1. 点击 Nav2 Goal Tool 或 2D Goal Pose；
2. 点击目标位置；
3. 拖动设置目标方向；
4. Goal 被 `/navigate_to_pose` 接受；
5. RViz 显示 Global Plan；
6. RViz 显示 Local Plan 或最优轨迹；
7. Local Costmap 随机器人滚动；
8. RobotModel 开始移动；
9. 距离逐渐减小；
10. 最终输出：

```text
Goal succeeded
```

---

# 31. 验收标准

完成后必须满足：

1. 不覆盖现有 `plan.md`；
2. 本文档加入仓库；
3. `run_ros.sh` 默认自动启动 RViz；
4. 根据模式自动选择正确 RViz 配置；
5. Mapping 默认自动启动键盘 Teleop；
6. Navigation 不启动 Teleop；
7. Mapping RViz 显示完整建图过程；
8. Localization RViz 显示完整定位过程；
9. Navigation RViz 显示完整导航过程；
10. 2D Pose Estimate 真正发布 `/initialpose`；
11. 人工初始位姿能更新 SLAM Toolbox 定位；
12. 自动初始位姿不会持续覆盖人工位姿；
13. Nav2 Goal Tool 或 2D Goal Pose 真正调用 NavigateToPose；
14. 用户日常导航不需要命令行 Goal；
15. 一个 RViz 点击只产生一个 Goal；
16. Goal 可以取消；
17. Goal 可以被新 Goal 替换；
18. RViz 能显示静态地图；
19. RViz 能显示实时建图地图；
20. RViz 能显示 RobotModel；
21. RViz 能显示 TF；
22. RViz 能显示 LaserScan；
23. RViz 能显示 PointCloud2；
24. RViz 能显示 Global Costmap；
25. RViz 能显示 Local Costmap；
26. RViz 能显示 Global Plan；
27. RViz 能显示 Local Plan 或实际控制轨迹；
28. RViz 能显示 Footprint；
29. RViz 能显示 Collision Monitor 区域；
30. `/scan` 不再出现 QoS 不兼容；
31. `/lidar/points_raw` 不再出现 QoS 不兼容；
32. 晚启动的 RViz 能收到 `/map`；
33. Teleop 支持 W/A/S/D；
34. Space 立即停车；
35. 无输入超时自动停车；
36. Teleop 退出时停车；
37. Mapping 中 `/cmd_vel` 只有预期人工控制发布者；
38. Navigation 中 `/cmd_vel` 不受 Teleop 干扰；
39. 脚本可从任意目录运行；
40. 所有入口统一 ROS Domain 和 RMW；
41. 重复启动被安全阻止；
42. Fast DDS SHM 可安全诊断和清理；
43. 活跃 DDS 进程存在时不删除 SHM；
44. Lifecycle 不再重复 configure/activate；
45. Activation Gate 失败时给出具体节点状态；
46. `/clock` 回跳能够检测；
47. 时间回跳后不使用旧 Readiness；
48. Ideal 导航成功；
49. Realistic 模式未被破坏；
50. Incremental Mapping 未被破坏；
51. MPPI 参数经过实际测量；
52. 控制循环不长期处于配置 20 Hz、实际仅 4～8 Hz；
53. 已有测试继续通过；
54. 新增测试全部通过；
55. 文档与实际命令一致。

---

# 32. 文档更新

更新：

```text
README.md
docs/user_manual.md
docs/interfaces.md
docs/repository_index.md
docs/troubleshooting.md
docs/rviz_workflow_upgrade_plan.md
```

## README

只保留快速入门：

* Mapping；
* Localization；
* Navigation；
* RViz 自动启动；
* Teleop；
* 保存地图。

## User Manual

详细说明：

* Mapping；
* Incremental Mapping；
* Localization；
* Navigation；
* RViz Displays；
* 2D Pose Estimate；
* Nav2 Goal Tool；
* Teleop；
* 保存地图；
* Headless 模式。

## Troubleshooting

至少包含：

* `Frame [map] does not exist`；
* `No map received`；
* QoS incompatible；
* Goal Tool 无反应；
* Action Server 未就绪；
* Teleop 无法控制；
* 多个 `/cmd_vel` 发布者；
* Fast DDS SHM；
* Lifecycle 重复转换；
* 仿真时间回跳；
* MPPI Missed Rate。

## Interfaces

更新实际：

* Topic；
* Action；
* Service；
* QoS；
* Lifecycle 所有权；
* 控制链；
* RViz 交互接口。

## Repository Index

登记新增：

* RViz 配置；
* Teleop Package；
* Goal Bridge，如实际使用；
* 新脚本；
* 测试；
* 本升级方案。

---

# 33. 实施阶段

## 阶段 1：仓库检查

* 检查 Git 状态；
* 检查已有修改；
* 确认实际 Topic、Action、Service、QoS；
* 确认 Nav2 RViz Plugin；
* 输出差距分析。

## 阶段 2：基础稳定性

* 统一环境；
* 任意目录运行；
* 单实例保护；
* `clean_runtime`；
* `preflight`；
* `diagnose`。

## 阶段 3：Lifecycle 与时间

* 明确 Lifecycle 所有权；
* 幂等状态转换；
* Activation Gate 诊断；
* 有限重试；
* 时间回跳检测；
* 对应测试。

## 阶段 4：RViz

* 创建 `mapping.rviz`；
* 创建 `localization.rviz`；
* 完善 `navigation.rviz`；
* 修复 QoS；
* 加入完整 Displays；
* 加入 Nav2 Panel；
* 加入 2D Pose Estimate；
* 加入官方 Nav2 Goal Tool。

## 阶段 5：Goal 备用桥接

只有官方 Goal Tool 无法满足需求时实施。

不得同时启用两套 Goal 转发机制。

## 阶段 6：Mapping Teleop

* W/A/S/D；
* 安全停车；
* 独立终端；
* 模式互斥；
* 进程清理；
* 单元测试。

## 阶段 7：启动集成

* RViz 集成到 `run_ros.sh`；
* Teleop 集成到 Mapping；
* Headless 开关；
* 子进程生命周期管理。

## 阶段 8：MPPI 性能

* 建立基准；
* 比较候选参数；
* 选择默认方案；
* 记录依据。

## 阶段 9：集成验收

* 自动测试；
* 连续重启；
* Mapping 人工测试；
* Localization 人工测试；
* Navigation RViz Goal 测试；
* 文档更新。

---

# 34. 最终实施报告要求

完成本方案后，实施报告必须包含：

1. 实际确认的根因；
2. 各问题是否相互独立；
3. 修改文件列表；
4. 每个文件的修改目的；
5. 三种 RViz 配置设计；
6. Mapping Teleop 设计；
7. 2D Pose Estimate 实现；
8. 2D Goal Pose/Nav2 Goal Tool 实现；
9. 是否使用 Goal Bridge；
10. Lifecycle 修复；
11. Fast DDS SHM 处理；
12. 时间回跳处理；
13. MPPI 性能结果；
14. 自动测试命令和结果；
15. 实际集成测试结果；
16. 未执行测试及原因；
17. 精确人工验收命令；
18. 剩余风险；
19. 后续工作建议。

不得声称未执行的测试已经通过。
