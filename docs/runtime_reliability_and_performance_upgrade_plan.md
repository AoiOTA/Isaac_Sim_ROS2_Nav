# Isaac Sim ROS 2 导航运行时可靠性、性能与机器人相机视角升级方案

> 文件路径：`docs/runtime_reliability_and_performance_upgrade_plan.md`  
> 适用仓库：`AoiOTA/Isaac_Sim_ROS2_Nav`  
> 适用环境：Ubuntu 24.04、ROS 2 Jazzy、Isaac Sim 6.0.1、RTX 4090  
> 文档性质：实现计划、问题复现记录、设计约束、测试矩阵和验收标准  
> 状态：待实施并持续回填验证结果

---

## 1. 文档定位

本文件用于指导当前项目的运行时可靠性、性能、地图工作流和机器人第一视角相机功能升级。

本文件与其他设计文档的关系如下：

| 文档 | 主要职责 |
|---|---|
| `plan.md` | 系统总体架构、阶段划分、长期目标和最终验收指标 |
| `docs/rviz_workflow_upgrade_plan.md` | RViz、Nav2 Goal、Lifecycle、Mapping Teleop 和交互工作流 |
| `docs/runtime_reliability_and_performance_upgrade_plan.md` | 实际运行问题、时序、TF、传感器延迟、控制性能、CPU/GPU、地图标定、机器人相机视角和退出清理 |

本文件不是一次性任务清单。实施过程中必须持续回填：

- 复现命令；
- 基线结果；
- 已确认根因；
- 被排除的假设；
- 修改文件；
- 参数选择依据；
- 测试结果；
- 性能对比；
- 尚未完成事项；
- 最终验收结果。

---

## 2. 项目环境

| 项目 | 当前配置 |
|---|---|
| 操作系统 | Ubuntu 24.04 LTS |
| GPU | NVIDIA GeForce RTX 4090 |
| NVIDIA Driver | 595.71.05 |
| ROS 2 | Jazzy，通过系统 apt 安装 |
| RMW | `rmw_fastrtps_cpp` |
| ROS Domain | `42` |
| Isaac Sim | 6.0.1.0，通过 pip 安装 |
| Conda 环境 | `isaacsim` |
| Isaac Python | `/home/lyb/miniconda3/envs/isaacsim/bin/python` |
| Isaac 资产根目录 | `/home/lyb/isaacsim_assets/Assets/Isaac/6.0` |
| 项目目录 | `/home/lyb/Workspace/Isaac_Sim_ROS2_Nav` |
| Shell | Bash |
| 机器人基线 | NVIDIA/项目 Jackal 四轮滑移转向底盘 |
| 主要环境 | Warehouse |
| ROS 时间 | 所有仿真节点使用 `use_sim_time=true` |

所有修改必须适配以上环境，不得要求用户改用 Docker、重装 ROS、重装 Isaac Sim 或更换 DDS，除非存在经过验证且无法绕过的兼容性问题。

---

## 3. 本轮升级目标

### 3.1 可靠性目标

1. 新终端能够稳定发现项目 Topic、Service、Action 和 TF。
2. `/clock`、传感器消息、里程计和 TF 使用一致的仿真时间。
3. 不再持续出现 TF 未来外推、旧时间戳和 RViz 队列溢出。
4. Collision Monitor 在正常扫描抖动下不误停车，在真实断流时仍及时停车。
5. Nav2 Controller 能稳定满足经过验证的控制频率。
6. Ideal 和 Realistic 模式保持唯一的 `/odom` 与 TF 所有权。
7. Mapping、Localization、Navigation 和 Reset 工作流可重复运行。
8. Ctrl+C 和 Lifecycle Shutdown 不产生未等待协程、Guard Condition 错误或 RViz 崩溃。
9. 新地图在未标定时不能错误套用旧地图的自动初始位姿。
10. Robot Camera 启用后不破坏原有导航稳定性。

### 3.2 性能目标

1. Interactive 模式的 RTF 接近 1.0。
2. 先稳定 10 Hz Controller，再评估更高频率。
3. MPPI、SLAM Toolbox、PointCloud 投影、RViz 和 Isaac Sim 的资源竞争可测量。
4. CPU powersave/performance 的差异可复现。
5. GPU 的实际使用范围清晰，不虚假声称 CPU 算法已被 CUDA 加速。
6. 相机关闭、相机发布、RViz 显示三种状态的性能差异可量化。
7. Headless 批量实验能够明确记录 RTF、相机状态和性能 Profile。

### 3.3 可用性目标

1. Mapping Teleop 窗口和焦点提示明确。
2. Teleop 速度可在运行过程中直接通过键盘调整。
3. RViz 同时显示地图、机器人、路径、激光雷达和机器人第一视角 RGB 图像。
4. 用户无需打开 Isaac Sim 相机视口，就能在 RViz 查看机器人看到的画面。
5. Camera View 可以在 Mapping、Localization 和 Navigation RViz 配置中使用。
6. 相机可通过启动参数启用、关闭和选择质量 Profile。
7. Camera 只用于观察，不得偷偷进入 Nav2、SLAM、EKF 或安全控制链。

---

## 4. 非目标

本轮明确不做以下内容：

1. 不使用 RGB 图像替代 2D LaserScan 导航。
2. 不实现视觉 SLAM、VIO、目标检测或语义导航。
3. 不把 Camera Ground Truth 接入实时导航。
4. 不把 Nav2 MPPI 重写为 CUDA。
5. 不默认启用高分辨率双目、深度和语义分割。
6. 不为观察功能引入不必要的图像压缩和解压 CPU 开销。
7. 不删除 Collision Monitor、deadman、Progress Checker 或 Costmap 障碍层。
8. 不通过设置超大 TF tolerance、source timeout 或队列深度隐藏根因。
9. 不修改 `/opt/ros/jazzy` 或 Isaac Sim 安装目录中的系统文件。
10. 不提交 `build/`、`install/`、`log/`、运行时报告或 NVIDIA 官方资产。

---

## 5. 必须保留的架构契约

### 5.1 进程边界

#### Isaac Sim 主进程

负责：

- USD Stage；
- PhysX；
- Jackal；
- RTX LiDAR；
- IMU；
- 前向 RGB Camera；
- 可选 Stereo/Depth Camera；
- Render Product；
- ROS 2 Camera Publisher Graph；
- `/clock`；
- Ideal Odom；
- TF；
- `/cmd_vel` 接收；
- Reset；
- Ground Truth；
- 动态障碍物。

#### ROS 2 主栈

负责：

- PointCloud2 到 LaserScan；
- SLAM Toolbox；
- Wheel Odometry；
- robot_localization EKF；
- Map Server；
- Nav2；
- MPPI；
- Velocity Smoother；
- Collision Monitor；
- RViz；
- Camera 图像显示；
- 实验与诊断。

#### Mapping Teleop

只在 Mapping 和 Incremental Mapping 中运行，直接发布 `/cmd_vel`，不能与 Navigation 控制链并存。

### 5.2 TF 契约

主 TF 树：

```text
map
└── odom
    └── base_link
        ├── lidar_link
        ├── imu_link
        ├── camera_link
        │   └── camera_front_link
        │       └── camera_front_optical_frame
        ├── front_left_wheel_link
        ├── front_right_wheel_link
        ├── rear_left_wheel_link
        └── rear_right_wheel_link
```

若复用项目已有双目结构，则允许：

```text
base_link
└── camera_link
    ├── camera_left_link
    │   └── camera_left_optical_frame
    └── camera_right_link
        └── camera_right_optical_frame
```

但必须明确指定哪个 Camera 是 RViz 默认的机器人前向视角。

禁止：

- ROS TF 中增加 `world`；
- 重复发布 `map -> odom`；
- 重复发布 `odom -> base_link`；
- Isaac 与 Robot State Publisher 重复发布相机固定 TF；
- Image 和 CameraInfo 使用不存在的 Frame；
- Camera 消息 `frame_id` 使用 USD Prim 路径；
- Camera 光学坐标系方向不符合 ROS optical frame 约定。

### 5.3 模式所有权

#### Ideal

- Isaac 发布 `/odom`；
- Isaac 发布 `odom -> base_link`；
- EKF 不发布 `/odom`。

#### Realistic

- `/joint_states` 进入 Wheel Odometry；
- Wheel Odometry 发布 `/wheel/odom`；
- `/imu/data` 和 `/wheel/odom` 输入 EKF；
- EKF 唯一发布 `/odom`；
- EKF 唯一发布 `odom -> base_link`；
- Isaac 不发布 Ideal Odom。

#### Mapping

- SLAM Toolbox Mapping 发布 `/map`；
- SLAM Toolbox 发布 `map -> odom`；
- 不启动 Map Server；
- Mapping Teleop 拥有 `/cmd_vel`。

#### Localization/Navigation

- Map Server 唯一发布固定 `/map`；
- SLAM Toolbox Localization 发布 `map -> odom`；
- SLAM Toolbox 诊断地图使用 `/slam_toolbox/map`；
- Navigation 命令链：

```text
MPPI
  -> /cmd_vel_nav
Velocity Smoother
  -> /cmd_vel_smoothed
Collision Monitor
  -> /cmd_vel
Isaac Sim
```

### 5.4 Camera 数据所有权

默认 Camera 只用于观察：

```text
Isaac Camera
  -> Render Product
  -> ROS 2 Camera Helper
  -> /camera/front/image_raw
  -> RViz Image Display
```

CameraInfo：

```text
Camera Intrinsics
  -> ROS 2 CameraInfo Helper
  -> /camera/front/camera_info
  -> RViz Camera Display / downstream inspection
```

默认禁止以下连接：

```text
/camera/front/image_raw -> Nav2
/camera/front/image_raw -> SLAM Toolbox
/camera/front/image_raw -> EKF
/camera/front/image_raw -> Collision Monitor
/camera/front/image_raw -> Initial Pose
```

以后需要视觉算法时，应作为单独设计阶段，不得在本轮观察功能中隐式加入。

---

## 6. 已观测问题和复现记录

### 6.1 Nav2 已激活但 RViz 报旧时间戳

已经观察到：

```text
Nav2 lifecycle activation completed
```

随后 RViz 输出：

```text
Message Filter dropping message:
frame 'odom'
for reason 'discarding message because the queue is full'
```

以及：

```text
Message Filter dropping message:
frame 'base_link'
for reason
'the timestamp on the message is earlier than all the data
in the transform cache'
```

必须检查：

- `/clock` 推进顺序；
- Odom 与 TF Stamp；
- Scan Stamp；
- RViz Display Queue；
- Reset 前旧消息；
- Simulation RTF；
- ROS Executor 延迟。

### 6.2 Mapping Teleop 窗口焦点不明确

用户最初在 `run_ros.sh` 主终端输入 W/A/S/D 和方向键，机器人没有运动。实际 Teleop 在独立窗口中运行，必须先点击该窗口。

该问题属于交互说明不足，不代表控制链完全失效。

### 6.3 Mapping 速度偏低

当前默认约为：

```yaml
linear_speed: 0.30
angular_speed: 0.60
```

大范围 Warehouse 建图效率偏低，并且必须修改参数才能改变速度。

本轮要求增加运行时动态调速。

### 6.4 新终端看不到 Topic

用户只 source ROS 和工作空间，未显式设置 Domain 42 和 Fast DDS，随后出现：

```text
Unknown topic '/map'
Unknown topic '/slam_toolbox/map'
Unknown topic '/odom'
```

以及：

```text
Invalid frame ID "map"
Invalid frame ID "odom"
```

优先检查手动终端与项目运行进程是否使用不同的：

- `ROS_DOMAIN_ID`；
- `RMW_IMPLEMENTATION`；
- ROS daemon；
- 工作空间；
- 项目副本。

### 6.5 Realistic Topic 不可见

手动终端查询：

```text
/wheel/odom
/odom
```

得到 Unknown Topic。

必须先修复环境一致性，再检查 Wheel Odom、IMU、EKF 和模式配对。

### 6.6 warehouse_v2 保存成功但导航失败

OccupancyGrid 和 Pose Graph 均保存成功，但可能没有完成该地图专属的 Map Pose 标定。

必须避免 `warehouse_v2` 无条件复用 `warehouse_v1` 自动初始位姿。

### 6.7 Controller 错过 10 Hz

实际出现过：

```text
Current loop rate is 3.5294 Hz
Current loop rate is 4.0000 Hz
Current loop rate is 4.2857 Hz
Current loop rate is 4.6154 Hz
Current loop rate is 5.0000 Hz
Current loop rate is 7.5000 Hz
Current loop rate is 8.5714 Hz
```

随后出现：

```text
Failed to make progress
Optimizer reset
[follow_path] [ActionServer] Aborting handle
```

需要区分：

- MPPI 计算预算不足；
- Ceres 抢占；
- RTF 大于 1；
- GUI/RTX 负载；
- PointCloud 投影积压；
- TF/Scan 延迟导致停走；
- 速度参数过低；
- 新地图初始位姿错误。

### 6.8 Collision Monitor 误判 Scan 无效

日志包括：

```text
Latest source and current collision monitor node timestamps differ
on 0.516667 seconds. Ignoring the source.
```

延迟还达到：

```text
0.566667
0.633333
0.683333
```

当前 `source_timeout` 约为 0.50 秒。不能只把 timeout 改大，必须先处理 Scan Age、RTF、TF 和 Executor 排队。

### 6.9 TF Future Extrapolation

日志：

```text
Requested time 96.933333
but the latest data is at time 96.900000
```

差值约为 0.033333 秒，可能是一到两帧发布顺序问题。

### 6.10 RViz 不显示 Local Plan

Global Plan 可见，但 `/local_plan` 不稳定或不可见。

需要确认真实 Topic、类型、QoS、TF 和 Controller 是否实际完成控制周期。

### 6.11 Ceres 线程警告

反复出现：

```text
Specified options.num_threads: 50 exceeds maximum available
from the threading model Ceres was compiled with: 20.
```

必须找到 50 的真实来源，不能添加无效参数或过滤日志。

### 6.12 关闭异常

已经出现：

```text
coroutine ... was never awaited
```

和：

```text
failed to create guard condition
```

以及 RViz：

```text
exit code -6
```

需要修复 Goal 取消、Future、Executor、Node、Context 和 RViz 子进程的关闭顺序。

### 6.13 Camera 当前只是预留

当前 Camera 配置处于关闭状态，已有左右 Camera Prim 路径，但没有完成以下闭环：

- 可用 Render Product；
- ROS 2 RGB Publisher；
- CameraInfo Publisher；
- 发布频率控制；
- QoS；
- RViz Image Display；
- Camera 性能基准；
- Camera Topic 诊断；
- Camera 与自定义机器人迁移契约。

本轮必须将 Camera 从“预留配置”升级为“可启用、可观察、可诊断、可关闭”的完整功能。

---

## 7. 升级总体架构

```text
                         ┌────────────────────────────┐
                         │        Isaac Sim 6.0.1     │
                         │                            │
                         │  Physics / Jackal / Stage  │
                         │      │                     │
                         │      ├─ RTX LiDAR          │
                         │      ├─ IMU                │
                         │      ├─ Odom / TF          │
                         │      └─ Front RGB Camera   │
                         │             │              │
                         │        Render Product      │
                         │             │              │
                         │   ROS2 Camera Helper       │
                         └─────────────┼──────────────┘
                                       │ DDS
             ┌─────────────────────────┼─────────────────────────┐
             │                         │                         │
       /lidar/points_raw       /camera/front/image_raw    /camera/front/camera_info
             │                         │                         │
 pointcloud_to_laserscan               └──────────┬──────────────┘
             │                                    │
           /scan                            RViz Image/Camera
             │                                    │
       SLAM / Nav2 / Collision               只用于观察
```

---

## 8. 统一 ROS 手动终端环境

新增：

```text
scripts/setup_ros_env.sh
```

使用：

```bash
cd ~/Workspace/Isaac_Sim_ROS2_Nav
source ./scripts/setup_ros_env.sh
```

要求：

1. 必须被 source；
2. 直接执行时明确报错；
3. 自动解析 `PROJECT_ROOT`；
4. 验证项目根目录；
5. source `/opt/ros/jazzy/setup.bash`；
6. 验证并 source `ros2_ws/install/setup.bash`；
7. 设置或验证：

```bash
ROS_DOMAIN_ID=42
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

8. 输出：

```text
PROJECT_ROOT
ROS_DISTRO
ROS_DOMAIN_ID
RMW_IMPLEMENTATION
AMENT_PREFIX_PATH
ROS_SETUP
WORKSPACE_SETUP
```

9. 支持：

```bash
source ./scripts/setup_ros_env.sh --restart-daemon
```

10. `--restart-daemon` 执行：

```bash
ros2 daemon stop
ros2 daemon start
```

11. 不得默认杀死其他项目的 daemon；
12. 重复 source 幂等；
13. 不重复污染 PATH；
14. 文档中的手工命令全部改用该脚本。

测试：

- 直接执行失败；
- source 成功；
- Domain 正确；
- RMW 正确；
- 未构建时失败；
- 重复 source；
- daemon restart；
- 新终端能够看到 `/map`、`/scan`、`/odom` 和 Camera Topic。

---

## 9. Mapping Teleop 动态调速

### 9.1 默认参数

建议基线：

```yaml
publish_rate_hz: 20.0
command_timeout_sec: 0.18

linear_speed: 0.50
angular_speed: 0.80

min_linear_speed: 0.10
min_angular_speed: 0.20

max_linear_speed: 1.00
max_angular_speed: 1.50

linear_speed_step: 0.05
angular_speed_step: 0.10
```

最终默认值由实际建图测试确认。

### 9.2 按键

#### 运动

| 按键 | 功能 |
|---|---|
| `W` / `↑` | 前进 |
| `S` / `↓` | 后退 |
| `A` / `←` | 左转 |
| `D` / `→` | 右转 |
| `Space` | 立即停止 |
| `Q` | 发布零速度并退出 |
| `Ctrl+C` | 发布零速度并退出 |
| `Ctrl+D` | 发布零速度并退出 |

#### 调速

| 按键 | 功能 |
|---|---|
| `+` / `=` | 同时提高线速度和角速度 |
| `-` | 同时降低线速度和角速度 |
| `]` | 只提高线速度 |
| `[` | 只降低线速度 |
| `.` | 只提高角速度 |
| `,` | 只降低角速度 |
| `0` | 恢复默认速度 |
| `H` / `?` | 显示帮助和当前速度 |

要求：

- 调速立即生效；
- 不需要重启；
- 不突破上限；
- 不低于下限；
- 调速按键不触发运动；
- 调速期间 deadman 仍生效；
- 每次调整显示目标速度；
- 达到上限/下限有提示；
- 非 ASCII 输入不崩溃；
- Arrow Escape Sequence 正常；
- 退出始终发布最终零速度。

### 9.3 Launch 覆盖

支持：

```text
teleop_linear_speed
teleop_angular_speed
teleop_linear_speed_step
teleop_angular_speed_step
teleop_min_linear_speed
teleop_min_angular_speed
teleop_max_linear_speed
teleop_max_angular_speed
```

示例：

```bash
./scripts/run_ros.sh mapping \
  odometry_mode:=ideal \
  teleop_linear_speed:=0.45 \
  teleop_angular_speed:=0.75
```

### 9.4 焦点提示

主终端必须输出：

```text
Mapping Teleop is running in a separate terminal.
Click the window titled "Isaac Nav Mapping Teleop"
before pressing W/A/S/D or the arrow keys.
```

独立窗口标题：

```text
Isaac Nav Mapping Teleop
```

---

## 10. Robot Front Camera 设计

### 10.1 功能目标

增加一个面向机器人前进方向的 RGB Camera，使用户可以在 RViz 中观察：

- 机器人正前方环境；
- 导航时的行驶状态；
- 是否接近货架或障碍；
- 转弯时相机视角变化；
- Collision Monitor 停车是否符合现场画面；
- 机器人在地图中的行为与真实视角是否一致。

该 Camera 默认仅用于人工观察。

### 10.2 默认 Camera 模式

建议提供以下 Profile：

| Profile | 分辨率 | 发布频率 | 用途 |
|---|---:|---:|---|
| `off` | 无 | 0 | 性能基线、无相机批量实验 |
| `monitoring` | 640×360 | 15 Hz | 默认交互观察 |
| `standard` | 640×480 | 20 Hz | 较清晰观察 |
| `high_quality` | 1280×720 | 30 Hz | 人工演示和截图，不用于性能基线 |

默认策略：

- GUI Mapping：`monitoring`；
- GUI Localization：`monitoring`；
- GUI Navigation：`monitoring`；
- Headless：默认 `off`；
- 正式性能 Benchmark：必须显式记录 Profile；
- 批量实验：默认 `off`，除非实验要求记录图像。

### 10.3 配置 Schema

将旧 Camera 配置升级为严格 Schema。示例：

```yaml
schema_version: 2

enabled: true
default_profile: monitoring
primary_camera: front

profiles:
  off:
    enabled: false

  monitoring:
    enabled: true
    width: 640
    height: 360
    publish_rate_hz: 15.0

  standard:
    enabled: true
    width: 640
    height: 480
    publish_rate_hz: 20.0

  high_quality:
    enabled: true
    width: 1280
    height: 720
    publish_rate_hz: 30.0

cameras:
  front:
    enabled: true

    sensor_prim: /World/Robots/Jackal/base_link/camera_link/camera_front_link/camera_front_sensor

    link_frame: camera_front_link
    optical_frame: camera_front_optical_frame

    node_namespace: /camera/front

    rgb:
      enabled: true
      topic_name: image_raw
      encoding: rgb8
      qos_profile: sensor_data
      queue_size: 2

    camera_info:
      enabled: true
      topic_name: camera_info
      qos_profile: sensor_data
      queue_size: 2

    depth:
      enabled: false
      topic_name: depth/image_raw
      qos_profile: sensor_data
      queue_size: 1

    clipping_range_m:
      near: 0.05
      far: 100.0

    optics:
      projection: perspective
      focal_length_mm: 24.0
      horizontal_aperture_mm: 21.0
      vertical_aperture_mm: 16.0
      focus_distance_m: 4.0

    exposure:
      enabled: true
      time_s: 0.02
      responsivity: 1.10267
      f_stop: 5.0

    rviz:
      enabled: true
      transport: raw
      reliability: best_effort
      queue_size: 2
```

若保留现有左右相机，则改为：

```yaml
primary_camera: left

cameras:
  left:
    ...
  right:
    enabled: false
```

不能继续用一个全局 `enabled` 后无条件创建左右两台 Camera。

### 10.4 Camera USD 层级

Camera Prim 应作为机器人固定传感器，不能成为独立动态刚体。

推荐：

```text
/World/Robots/Jackal
└── base_link
    └── camera_link
        └── camera_front_link
            └── camera_front_sensor
```

要求：

1. `camera_front_sensor` 为 `UsdGeom.Camera`；
2. Camera 不添加独立 RigidBody；
3. Camera 不添加碰撞体；
4. Camera 安装位姿由机器人资产或严格配置决定；
5. Camera 不穿入机器人外壳；
6. Camera 视野中不应大面积看到机器人自身；
7. Camera 朝向机器人前方；
8. Camera 高度应适合观察地面障碍和走廊；
9. 近裁剪面避免裁掉近处环境；
10. 远裁剪面覆盖 Warehouse 走廊。

建议初始安装参数仅作为起点，必须在 Isaac GUI 检查：

```text
相对 base_link:
x: +0.20 ～ +0.35 m
y: 0.00 m
z: +0.35 ～ +0.55 m
pitch: 轻微向下 0°～10°
```

最终数值应依据 Jackal 实际模型和现有 Camera Rig。

### 10.5 ROS Camera Frame 约定

必须区分机械安装 Frame 和光学 Frame。

机械 Camera Link：

```text
camera_front_link
x：前
y：左
z：上
```

光学 Frame：

```text
camera_front_optical_frame
x：右
y：下
z：前
```

消息要求：

```text
/camera/front/image_raw.header.frame_id
= camera_front_optical_frame

/camera/front/camera_info.header.frame_id
= camera_front_optical_frame
```

Image 与 CameraInfo：

- Stamp 必须相同或在可接受的同帧误差内；
- Frame ID 必须相同；
- 不得使用 `camera_front_link` 代替 optical frame；
- 不得使用 `/World/...` Prim 路径作为 ROS Frame ID。

### 10.6 Isaac Camera 创建

升级 `SensorFactory`：

1. 严格验证 Camera Schema；
2. 支持 Profile；
3. 只创建 enabled Camera；
4. 设置 Camera Prim 的光学属性；
5. 创建并保存 Render Product；
6. 返回明确的 Camera Bundle；
7. Camera 创建失败时 fail fast；
8. 不允许 Render Product Path 为空；
9. 不允许分辨率非正整数；
10. 不允许发布频率高于仿真可支持上限；
11. 检查 sensor prim 是否位于机器人根 Prim 下；
12. Headless 仍允许 Offscreen Render，但默认关闭 Camera。

建议数据结构：

```python
@dataclass(frozen=True)
class CameraRuntime:
    name: str
    camera_prim_path: str
    render_product_path: str
    optical_frame: str
    width: int
    height: int
    publish_rate_hz: float
```

`SensorBundle` 增加：

```python
cameras: tuple[CameraRuntime, ...]
```

不要只返回裸 `UsdGeom.Camera`。

### 10.7 ROS 2 Camera Publisher Graph

使用 Isaac Sim 6.0.1 当前安装中实际支持的 Camera Helper 节点，不能复制旧版节点名后不验证。

基本图：

```text
Tick / Controlled Publish Trigger
  -> IsaacCreateRenderProduct
      -> ROS2CameraHelper (RGB)
      -> ROS2CameraInfoHelper
```

RGB：

```text
type: rgb
frameId: camera_front_optical_frame
nodeNamespace: /camera/front
topicName: image_raw
useSystemTime: false
```

CameraInfo：

```text
frameId: camera_front_optical_frame
nodeNamespace: /camera/front
topicName: camera_info
useSystemTime: false
```

要求：

1. RGB 和 CameraInfo 使用相同 Render Product；
2. 使用 Simulation Time；
3. `useSystemTime=false`；
4. 不依赖已废弃的 `frameSkipCount` 作为长期频率控制方案；
5. 优先使用 Sensor Tick Rate 或受支持的 Simulation Gate；
6. Camera Publish Rate 与 Profile 一致；
7. CameraInfo 不应以远高于 RGB 的频率无意义发布；
8. RGB 与 CameraInfo 尽量同帧；
9. Reset 时不能发布 Reset 前旧帧；
10. Stop/Play 后时间策略与项目 `/clock` 契约一致；
11. QoS 和 Queue Size 明确；
12. Graph Path 纳入唯一命名和清理机制；
13. Camera Graph 不能与 LiDAR Graph 使用冲突节点路径；
14. Graph Builder 应支持 Camera Disabled。

建议 Graph 路径：

```text
/World/Graphs/ROS2CameraFront
```

或者符合项目现有 Graph 根路径约定的路径。

### 10.8 Camera Topic 契约

默认：

| Topic | 类型 | Frame | 频率 |
|---|---|---|---:|
| `/camera/front/image_raw` | `sensor_msgs/msg/Image` | `camera_front_optical_frame` | 15 Hz |
| `/camera/front/camera_info` | `sensor_msgs/msg/CameraInfo` | `camera_front_optical_frame` | 15 Hz |

可选：

| Topic | 类型 | 默认 |
|---|---|---|
| `/camera/front/depth/image_raw` | `sensor_msgs/msg/Image` | 关闭 |
| `/camera/front/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 关闭 |
| `/camera/right/image_raw` | `sensor_msgs/msg/Image` | 关闭 |

默认不发布：

- Semantic Segmentation；
- Instance Segmentation；
- Bounding Box；
- Depth PointCloud；
- H.264；
- 双目右图。

### 10.9 Image 编码

必须实际验证 Isaac Publisher 输出编码。

目标优先级：

1. `rgb8`；
2. `bgr8`；
3. 若实际为 `rgba8`，确认 RViz 支持并评估是否需要转换。

禁止为观察功能增加每帧不必要的 Python 图像转换。

诊断输出：

```bash
ros2 topic echo /camera/front/image_raw --once \
  --field encoding

ros2 topic echo /camera/front/image_raw --once \
  --field width

ros2 topic echo /camera/front/image_raw --once \
  --field height

ros2 topic echo /camera/front/image_raw --once \
  --field step
```

### 10.10 CameraInfo 校验

CameraInfo 必须与分辨率和 Camera Prim 光学参数一致。

检查：

- `width`；
- `height`；
- `distortion_model`；
- `D`；
- `K`；
- `R`；
- `P`；
- `binning_x/y`；
- `roi`；
- Frame ID；
- Stamp。

无模拟畸变时可使用：

```text
distortion_model: plumb_bob
D: 全零
```

但必须以实际 Helper 输出为准。

若以后启用双目：

- 左右 CameraInfo 必须正确反映 Baseline；
- Right Projection Matrix 的 Tx 必须正确；
- 不得仅在 YAML 写 baseline 而 Publisher 未使用。

### 10.11 Camera QoS

默认：

```text
history: keep_last
depth: 2
reliability: best_effort
durability: volatile
```

适用于观察型高频传感器。

要求：

1. RViz 使用匹配的 Best Effort；
2. 队列不能无限增长；
3. 允许丢旧帧，优先显示最新画面；
4. CameraInfo 与 Image QoS 兼容；
5. 不得因为 Reliable 堵塞导致相机延迟不断增加；
6. 记录网络/本机 DDS 带宽；
7. 同机运行时仍测量序列化和拷贝成本。

### 10.12 RViz Camera 可视化

在以下 RViz 配置中增加机器人前向视角：

```text
mapping.rviz
localization.rviz
navigation.rviz
```

主显示使用：

```text
rviz_default_plugins/Image
```

Display 名称：

```text
Robot Front Camera
```

配置：

```text
Topic: /camera/front/image_raw
Transport Hint: raw
Reliability: Best Effort
Queue Size: 2
Normalize Range: false
```

要求：

1. Camera Image 面板默认停靠在 RViz 右侧或底部；
2. 默认尺寸约 640×360；
3. 不遮挡地图主视图；
4. 用户可以拖拽调整；
5. 配置保存后重启仍保留布局；
6. Camera 未启用时显示 No Image，不导致 RViz 崩溃；
7. Camera Topic 出现后自动恢复显示；
8. Mapping、Localization、Navigation 的布局保持一致；
9. Image Display 不使用错误 Fixed Frame；
10. Image Display QoS 与 Publisher 匹配。

可选增加：

```text
rviz_default_plugins/Camera
```

用于 3D 投影叠加，但默认关闭。

命名：

```text
Robot Camera Projection
```

Camera Display 依赖 CameraInfo，主要用于验证相机内参和视场，不作为主要第一视角窗口。

### 10.13 Camera-only RViz 配置

新增：

```text
ros2_ws/src/robot_description/rviz/camera_view.rviz
```

用途：

- 单独检查相机；
- 不加载 Costmap、LaserScan 和复杂路径显示；
- 快速判断图像 Topic、QoS 和 Frame；
- 性能隔离测试。

可提供：

```bash
./scripts/run_camera_view.sh
```

但必须考虑项目已有 RViz 单实例锁。

推荐策略：

- 主工作流优先使用集成 Camera 面板；
- `run_camera_view.sh` 只在主 RViz 未运行时使用；
- 已有 RViz 时明确拒绝或提示用户使用当前窗口；
- 不偷偷启动第二个高负载 RViz。

### 10.14 Camera 启动参数

Isaac 侧：

```bash
./scripts/run_isaac.sh \
  --navigation-mode localization \
  --mode ideal \
  --camera-profile monitoring
```

关闭：

```bash
./scripts/run_isaac.sh \
  --navigation-mode localization \
  --mode ideal \
  --camera-profile off
```

高质量：

```bash
./scripts/run_isaac.sh \
  --navigation-mode localization \
  --mode ideal \
  --camera-profile high_quality
```

必要时支持：

```text
--camera-width
--camera-height
--camera-rate
```

但建议常规用户优先使用 Profile，避免任意组合缺乏验证。

ROS/RViz 侧：

```bash
./scripts/run_ros.sh navigation \
  odometry_mode:=ideal \
  camera_view:=true
```

如果 Camera 未启用但 RViz Camera View 打开，应给出警告而不是失败。

### 10.15 Camera 运行时开关

第一阶段不要求在仿真运行中热创建/销毁 Render Product。

初始实现允许：

- 启动时选择 `off/monitoring/standard/high_quality`；
- 停止后重新启动切换 Profile。

若实现运行时开关，必须保证：

- 不产生重复 Publisher；
- 不产生残留 Render Product；
- 不造成 GPU 资源泄漏；
- Reset 后恢复正确；
- Topic Publisher 数量始终为 0 或 1；
- 不影响 `/clock` 和导航控制。

### 10.16 Camera 性能隔离

Camera 会增加：

- RTX 渲染负载；
- GPU 显存；
- Render Product 开销；
- ROS 图像序列化；
- DDS 带宽；
- RViz 解码和纹理上传；
- CPU/GPU 同步开销。

必须测试：

| 模式 | Camera Publisher | RViz Camera Display |
|---|---|---|
| A | Off | Off |
| B | Monitoring | Off |
| C | Monitoring | On |
| D | Standard | On |
| E | High Quality | On |

每组记录：

- RTF；
- Controller Hz；
- MPPI 周期；
- Scan Age；
- TF Lag；
- GPU Utilization；
- GPU Memory；
- CPU；
- RViz CPU；
- ROS 带宽；
- Image Hz；
- Image Age；
- 掉帧；
- Navigation 成功。

Camera 默认 Profile 只有在不明显破坏 10 Hz Controller 的情况下才能设为交互默认值。

### 10.17 Camera 诊断

`diagnose.sh` 和 `profile_runtime.sh` 增加：

```text
/camera/front/image_raw
/camera/front/camera_info
```

输出：

- Publisher 数量；
- 类型；
- QoS；
- Frame ID；
- 编码；
- 分辨率；
- 平均频率；
- P95/P99 周期；
- 最大间隔；
- Message Age；
- 最大 Age；
- 图像带宽；
- CameraInfo 是否匹配；
- TF 是否存在；
- Render Product Profile；
- Camera 是否 Enabled；
- RViz 是否订阅；
- GPU 增量。

状态规则示例：

```text
PASS:
Image 和 CameraInfo 均为 1 个 Publisher，
频率与 Profile 匹配，
Frame 和 Stamp 一致，
Age 在阈值内。

WARN:
Camera 关闭，但 RViz Camera Display 打开。

FAIL:
Image 无 Publisher；
CameraInfo 缺失；
Frame 不存在；
Stamp 使用墙钟；
频率持续低于目标；
Publisher 重复；
图像积压导致 Age 持续增长。
```

### 10.18 Camera 与 Reset

Reset 时必须验证：

1. Image Stamp 不回退；
2. 不发布 Reset 前缓冲帧；
3. Camera TF 与机器人新 Pose 一致；
4. RViz 不持续显示旧画面；
5. Camera Publisher 不重复创建；
6. Render Product 不泄漏；
7. Reset 后规定时间内恢复图像；
8. Camera 恢复不阻塞 Nav2 Activation Gate。

Camera 不应成为 Nav2 Activation 的硬依赖，因为它只用于观察。

可以作为可选诊断条件：

```text
camera_enabled=true 时，Camera Topic 未恢复 -> WARN
```

而不是阻止导航恢复。

### 10.19 Camera 与 Headless

Headless 模式仍可进行 Offscreen Render，但成本存在。

默认：

```text
headless + 未显式指定 Camera Profile -> off
```

需要图像录制时显式：

```bash
./scripts/run_isaac.sh \
  --headless \
  --navigation-mode localization \
  --mode ideal \
  --camera-profile monitoring
```

报告必须记录：

```text
headless: true
camera_profile: monitoring
rviz_camera_display: false
```

### 10.20 Camera 与自定义机器人迁移

自定义机器人配置必须显式提供：

```yaml
camera:
  mount_prim: ...
  sensor_prim: ...
  link_frame: ...
  optical_frame: ...
  transform:
    xyz: [...]
    rpy: [...]
```

不得假设：

- Camera 一定在 `base_link/camera_link`；
- Camera 一定使用 Jackal 的 Frame；
- Camera 名称一定是 left/right；
- Camera Prim 是机器人根 Prim 的直接子节点。

迁移验证：

1. Camera Prim 存在；
2. 位于自定义机器人层级内；
3. Frame 名称合法；
4. TF 唯一；
5. Camera 朝向前方；
6. RViz 图像正常；
7. CameraInfo 正常；
8. 不看到大面积自身外壳；
9. 分辨率/频率适配性能；
10. Realistic 模式不受影响。

### 10.21 Camera 测试

#### 配置单元测试

- Schema Version；
- Profile；
- 未知字段拒绝；
- 分辨率；
- 频率；
- Topic；
- Frame；
- QoS；
- Depth 默认关闭；
- Primary Camera；
- Disabled Profile。

#### Isaac/USD 测试

- Camera Prim 创建；
- 类型为 Camera；
- Render Product Path；
- Graph Node；
- RGB Helper；
- CameraInfo Helper；
- 无重复 Prim；
- 无重复 Graph；
- Headless 创建；
- Disabled 时不创建。

#### ROS 集成测试

- `/camera/front/image_raw` 存在；
- `/camera/front/camera_info` 存在；
- Publisher 唯一；
- Frame ID；
- Encoding；
- Width/Height；
- Hz；
- Stamp；
- QoS；
- TF；
- CameraInfo；
- Reset 恢复。

#### RViz 验证

- Image Display 存在；
- Topic 正确；
- Reliability 正确；
- Camera 面板布局；
- No Image 行为；
- 重启后布局保存；
- Local Plan 等现有 Display 不受影响。

### 10.22 Camera 验收标准

1. 在 Navigation RViz 中可实时看到机器人前方画面。
2. 图像方向正确，不上下颠倒、不左右镜像。
3. 图像与机器人转向同步。
4. `monitoring` Profile 达到接近 15 Hz。
5. Image Age 不持续增长。
6. CameraInfo 与 Image 匹配。
7. TF 存在且 Frame 正确。
8. Camera 开启后 Controller 仍满足稳定性目标。
9. Camera 关闭后不保留 Publisher 和 Render Product。
10. Headless Camera Off 基线不受影响。
11. Camera 不进入导航反馈链。
12. Reset 后 Camera 正常恢复。
13. Ctrl+C 后无 Camera Graph 清理异常。
14. 自定义机器人能够通过参数迁移 Camera。

---

## 11. 仿真实时倍率控制

新增：

```yaml
simulation:
  pacing_mode: realtime
  target_realtime_factor: 1.0
```

支持：

### realtime

- 默认交互模式；
- 目标 RTF 1.0；
- 不长期明显快于墙钟；
- Camera On/Off 都必须测量。

### unbounded

- 仅用于明确的 Headless 批量任务；
- 记录实际 RTF；
- 如果 ROS 控制器无法跟上，应降速或拒绝无效实验。

CLI：

```bash
./scripts/run_isaac.sh \
  --navigation-mode localization \
  --mode ideal \
  --target-rtf 1.0
```

必须使用 Isaac Sim 6.0.1 当前支持的 API，不能复制已废弃旧版本 Rate Limiter。

---

## 12. 运行时性能分析工具

新增：

```text
scripts/profile_runtime.sh
```

以及必要的 ROS 节点：

```text
robot_experiments/runtime_profiler.py
```

示例：

```bash
./scripts/profile_runtime.sh \
  --duration 60 \
  --output data/reports/runtime-profile.json
```

### 12.1 系统指标

- CPU 型号；
- 逻辑 CPU 数；
- 每核利用率；
- 总 CPU；
- Governor；
- Scaling Driver；
- 当前频率；
- 最小/最大频率；
- Load Average；
- 内存；
- Swap；
- 温度；
- 降频。

### 12.2 GPU 指标

- GPU 名称；
- Driver；
- GPU Utilization；
- Memory Utilization；
- 显存；
- P-State；
- Graphics Clock；
- Memory Clock；
- 功率；
- 温度；
- Isaac 进程 GPU 使用；
- Camera Render 增量。

### 12.3 Topic 指标

至少：

```text
/clock
/lidar/points_raw
/scan
/imu/data
/joint_states
/wheel/odom
/odom
/plan
/local_plan
/cmd_vel_nav
/cmd_vel_smoothed
/cmd_vel
/camera/front/image_raw
/camera/front/camera_info
```

每个 Topic：

- Publisher；
- 类型；
- QoS；
- 平均 Hz；
- P50/P95/P99；
- 最大间隔；
- Header Stamp；
- Simulation Time；
- Age；
- 最大 Age；
- Stamp 回退；
- 重复 Stamp；
- Future Stamp；
- 带宽。

### 12.4 TF 指标

```text
map -> odom
odom -> base_link
map -> base_link
base_link -> lidar_link
base_link -> imu_link
base_link -> camera_front_link
camera_front_link -> camera_front_optical_frame
```

### 12.5 Nav2 指标

- Lifecycle；
- Controller 目标 Hz；
- 实际 Hz；
- Missed；
- MPPI 耗时；
- Failed to make progress；
- Optimizer Reset；
- Costmap Clear；
- Collision Invalid Source；
- Goal 结果；
- 导航时间。

### 12.6 报告元数据

- Git Commit；
- Dirty；
- 地图；
- Pose Graph；
- Odom 模式；
- TF Source；
- RTF 模式；
- CPU 模式；
- GUI/Headless；
- Camera Profile；
- Camera RViz 是否启用；
- 时间戳。

---

## 13. 修复传感器与 TF 时间链

分析：

```text
Physics Step
  -> Simulation Time
  -> RTX LiDAR
  -> PointCloud2
  -> LaserScan
  -> SLAM Toolbox
  -> map -> odom
  -> Costmap
  -> MPPI
```

Camera：

```text
Simulation Tick
  -> Camera Render
  -> Image + CameraInfo
  -> RViz
```

要求：

1. 全部仿真消息使用 Simulation Time；
2. 不混入墙钟 Header；
3. Odom 与 TF Stamp 一致；
4. Image 与 CameraInfo 同帧；
5. `/clock` 不提前多帧；
6. PointCloud 和 Image 不持续排队；
7. TF 覆盖传感器 Stamp；
8. Reset Epoch 隔离；
9. RViz 不持续 Queue Full；
10. Camera 不加剧 Scan/TF 延迟到不可接受水平。

---

## 14. Collision Monitor 超时

先测量 Scan P99 和最大正常 Age，再计算：

```text
source_timeout =
max(
  允许丢失周期数 × 标称扫描周期,
  P99 Scan Age + 安全裕量
)
```

测试：

1. 正常 10 Hz；
2. 单帧丢失；
3. 多帧丢失；
4. 完全断流；
5. TF 缺失；
6. Reset；
7. Camera Off；
8. Camera Monitoring；
9. Camera High Quality。

Camera 开启后若导致 Scan Age 超阈值，应先处理资源竞争，而不是继续增大 timeout。

---

## 15. MPPI 性能稳定

### 15.1 参数矩阵

Controller：

```text
8 / 10 / 15 Hz
```

Batch：

```text
500 / 750 / 1000
```

Time Steps：

```text
15 / 20
```

Model DT：

```text
0.10
```

预测范围：

```text
至少 1.5 s，优先 2.0 s
```

### 15.2 Camera 维度

每个关键组合还需测试：

```text
camera=off
camera=monitoring, RViz camera off
camera=monitoring, RViz camera on
camera=high_quality, RViz camera on
```

### 15.3 Profile

#### stable

用于 GUI、日常调试和 Camera Monitoring。

#### performance

用于 Headless、CPU Performance 和批量实验。

配置应使用 Overlay 或 Launch 参数，不复制整份 Nav2 YAML。

---

## 16. SLAM Toolbox/Ceres 线程竞争

查明 `num_threads=50` 来源。

测试：

```text
8 / 12 / 16 / 20 threads
```

对比：

- Localization 更新；
- Controller Hz；
- Scan Age；
- TF Lag；
- Camera Image Age；
- CPU；
- Navigation 成功。

不得添加不存在参数或修改系统安装目录。

---

## 17. CPU Performance Mode

新增：

```text
scripts/performance_mode.sh
```

支持：

```bash
./scripts/performance_mode.sh status
sudo ./scripts/performance_mode.sh enable
sudo ./scripts/performance_mode.sh restore
```

要求：

- 不在启动脚本中自动 sudo；
- 保存原状态；
- 支持 powerprofilesctl/cpupower/受控 sysfs；
- 验证切换；
- 可恢复；
- 输出功耗和温度提醒；
- Benchmark 记录 CPU 模式。

---

## 18. GPU 使用与 PhysX GPU Dynamics

明确：

### 通常使用 GPU

- RTX Renderer；
- RTX LiDAR；
- Camera Render Product；
- Camera RGB 渲染；
- 可选 PhysX GPU Dynamics。

### 默认主要使用 CPU

- Nav2 MPPI；
- SLAM Toolbox；
- Ceres；
- PointCloud Projection；
- Costmap；
- TF；
- EKF；
- DDS；
- ROS Executor。

PhysX GPU Dynamics：

- 显式可选；
- 默认关闭；
- 对比 CPU/GPU Dynamics；
- 记录 RTF、GPU、LiDAR、Camera、控制稳定性；
- 无收益时保持 CPU Physics。

---

## 19. 地图 Manifest 与标定

`save_map.sh` 保存后生成：

```yaml
schema_version: 1
map_version: warehouse_v2

occupancy_grid:
  yaml: data/maps/occupancy/warehouse_v2.yaml
  image: data/maps/occupancy/warehouse_v2.pgm

pose_graph:
  posegraph: data/maps/posegraphs/warehouse_v2.posegraph
  data: data/maps/posegraphs/warehouse_v2.data

calibration:
  calibrated: false
  spawn_pose_profile: null
  calibrated_at: null
  calibration_method: null
```

`auto` 初始位姿必须要求匹配标定。

`rviz` 初始位姿允许未标定地图。

Camera 不影响地图标定，但 Camera 画面可用于人工判断机器人朝向，不得代替正式坐标标定。

---

## 20. RViz Local Plan 和 Lifecycle

### 20.1 Local Plan

确认：

```bash
ros2 topic list | grep -E 'plan|trajectory'
ros2 topic info /local_plan -v
ros2 topic echo /local_plan --once
ros2 topic hz /local_plan
ros2 topic info /optimal_trajectory -v
```

修复 Topic、QoS、TF 和 Display。

### 20.2 Camera Panel 与地图布局

Navigation RViz 推荐布局：

```text
左侧：
- Displays
- Tool Properties
- Navigation 2 Panel

中间：
- Map / Robot / Global Plan / Local Plan

右侧：
- Robot Front Camera
```

避免 Camera 面板遮挡地图。

### 20.3 Lifecycle

修复：

```text
Navigation: inactive
Localization: inactive
Feedback: active
```

与实际节点 Active 的矛盾。

Camera 状态不应混入 Nav2 Lifecycle，但可在诊断中显示：

```text
Camera: enabled / publishing / no subscriber
```

---

## 21. 关闭顺序

建议：

1. 停止接受新目标；
2. 取消活跃 NavigateToPose；
3. 等待取消；
4. 暂停 Activation Gate；
5. Deactivate Nav2；
6. 停止 Camera 发布 Trigger；
7. 停止 Camera Helper；
8. 释放 Camera Render Product；
9. 停止异步 Timer；
10. 取消 Future；
11. Remove Node；
12. Destroy Node；
13. Shutdown Executor；
14. Shutdown Context；
15. 关闭 RViz；
16. 关闭 Isaac；
17. 清理 PID/Lock。

验收：

- 无未等待协程；
- 无 Guard Condition；
- 无 RViz -6；
- 无 Camera Render Product 泄漏；
- 连续启动/关闭 5 次。

---

## 22. diagnose.sh 扩展

### 22.1 环境

- Shell Domain/RMW；
- Isaac Domain/RMW；
- ROS Domain/RMW；
- RViz Domain/RMW；
- 一致性。

### 22.2 模式

- Operation；
- Odom；
- TF Source；
- Map；
- Pose Graph；
- Initial Pose；
- Calibration；
- RTF；
- Camera Profile；
- Camera Primary；
- Camera RViz Display。

### 22.3 所有权

- `/map`；
- `/odom`；
- `/cmd_vel`；
- `/wheel/odom`；
- `/local_plan`；
- `/camera/front/image_raw`；
- `/camera/front/camera_info`。

### 22.4 日志统计

```text
Control loop missed
Failed to make progress
Optimizer reset
Ignoring the source
Robot to stop due to invalid source
extrapolation into the future
queue is full
timestamp ... earlier
coroutine ... never awaited
exit code -6
camera
render product
image age
```

每项：

```text
PASS / WARN / FAIL
```

---

## 23. 修改文件范围

可能涉及：

```text
scripts/lib/common.sh
scripts/preflight.sh
scripts/diagnose.sh
scripts/run_isaac.sh
scripts/run_ros.sh
scripts/run_rviz.sh
scripts/run_teleop.sh
scripts/run_teleop_terminal.sh
scripts/save_map.sh
scripts/setup_ros_env.sh
scripts/performance_mode.sh
scripts/profile_runtime.sh
scripts/run_camera_view.sh
```

Isaac：

```text
isaac_sim/apps/navigation_sim.py
isaac_sim/configs/project.yaml
isaac_sim/configs/sensors/camera.yaml
isaac_sim/configs/robots/jackal.yaml
isaac_sim/configs/robots/custom_robot.yaml
isaac_sim/src/config.py
isaac_sim/src/sensors/sensor_factory.py
isaac_sim/src/bridge/
isaac_sim/graphs/
isaac_sim/tests/
```

ROS：

```text
ros2_ws/src/robot_bringup/
ros2_ws/src/robot_navigation/
ros2_ws/src/robot_mapping/
ros2_ws/src/robot_teleop/
ros2_ws/src/robot_experiments/
ros2_ws/src/robot_description/
```

RViz：

```text
ros2_ws/src/robot_description/rviz/mapping.rviz
ros2_ws/src/robot_description/rviz/localization.rviz
ros2_ws/src/robot_description/rviz/navigation.rviz
ros2_ws/src/robot_description/rviz/camera_view.rviz
```

URDF/Xacro：

```text
ros2_ws/src/robot_description/urdf/jackal_sensors.xacro
ros2_ws/src/robot_description/urdf/custom_robot_sensors.xacro
```

文档：

```text
README.md
plan.md
docs/runtime_reliability_and_performance_upgrade_plan.md
docs/rviz_workflow_upgrade_plan.md
docs/user_manual.md
docs/troubleshooting.md
docs/interfaces.md
docs/calibration.md
docs/verification.md
docs/repository_index.md
docs/development.md
```

---

## 24. 实施顺序

### 阶段 1：保护和审查工作树

- `git status`；
- 保留上一个 Goal 修改；
- 当前测试基线；
- 新建本计划。

### 阶段 2：环境一致性

- setup_ros_env；
- Domain/RMW；
- daemon；
- 文档。

### 阶段 3：运行时可观测性

- diagnose；
- profiler；
- RTF；
- Topic Age；
- TF Lag；
- CPU/GPU。

### 阶段 4：Teleop

- 焦点提示；
- 动态调速；
- deadman；
- 测试。

### 阶段 5：Camera 最小闭环

- Camera Schema；
- Camera Prim；
- Render Product；
- RGB；
- CameraInfo；
- Topic；
- TF；
- Camera-only RViz。

### 阶段 6：Camera 集成 RViz

- Mapping；
- Localization；
- Navigation；
- 面板布局；
- QoS；
- 性能。

### 阶段 7：时序和 TF

- Clock；
- LiDAR；
- Scan；
- Odom；
- Camera；
- TF；
- Reset；
- Collision Monitor。

### 阶段 8：MPPI/Ceres

- RTF；
- CPU；
- Camera On/Off；
- Stable/Performance Profile。

### 阶段 9：地图标定

- Manifest；
- auto/rviz；
- warehouse_v2。

### 阶段 10：RViz/Lifecycle/Shutdown

- Local Plan；
- Camera；
- Lifecycle；
- Async Cleanup；
- Render Product Cleanup。

### 阶段 11：完整验证

- 自动测试；
- Isaac/USD；
- Headless；
- GUI 人工验收；
- 文档；
- Git 提交。

---

## 25. 自动测试

### 25.1 Shell

- `bash -n`；
- Source-only；
- 参数解析；
- PID/Lock；
- Camera Profile；
- Performance Mode。

### 25.2 Teleop

- 运动；
- 调速；
- 上限；
- 下限；
- deadman；
- 最终零速度。

### 25.3 Camera Config

- Schema；
- Profile；
- 分辨率；
- Hz；
- Topic；
- Frame；
- QoS；
- Primary Camera；
- Disabled。

### 25.4 Isaac Camera

- Camera Prim；
- Render Product；
- Graph；
- RGB Helper；
- CameraInfo；
- Headless；
- Reset；
- Cleanup。

### 25.5 ROS Camera

- Topic；
- 类型；
- Publisher；
- QoS；
- Stamp；
- Frame；
- CameraInfo；
- TF；
- Hz；
- Age。

### 25.6 时间和 TF

- 单调；
- Odom/TF；
- Image/CameraInfo；
- Scan Age；
- Image Age；
- Reset Epoch；
- Future Extrapolation。

### 25.7 Shutdown

- Goal 执行；
- Camera 发布；
- RViz Camera Display；
- Double Shutdown；
- 连续启动。

---

## 26. 实际验证矩阵

### 26.1 环境

```bash
source ./scripts/setup_ros_env.sh --restart-daemon
```

检查：

```bash
ros2 topic info /map -v
ros2 topic info /scan -v
ros2 topic info /odom -v
ros2 topic info /camera/front/image_raw -v
ros2 topic info /camera/front/camera_info -v
```

### 26.2 Mapping

Camera Monitoring + Teleop 动态调速。

要求：

- Camera 画面；
- W/A/S/D；
- 调速；
- deadman；
- 地图正常；
- TF 正常；
- 无持续延迟。

### 26.3 warehouse_v1 Ideal Navigation

测试：

- Camera Off；
- Monitoring；
- Monitoring + RViz；
- High Quality + RViz；
- GUI；
- Headless；
- 短目标；
- 3 m；
- 长目标。

### 26.4 Realistic

要求：

- Wheel Odom；
- IMU；
- EKF；
- Camera；
- `/odom` 唯一；
- 导航成功。

### 26.5 warehouse_v2

- 未标定 auto 失败；
- 未标定 rviz 成功；
- 标定后 auto；
- Camera 观察朝向；
- Global/Local Plan；
- 成功导航。

### 26.6 Reset

- Goal 中 Reset；
- Camera 图像恢复；
- Stamp 不回退；
- 无旧帧；
- Nav2 恢复。

### 26.7 Shutdown

- Camera Off；
- Camera Monitoring；
- Camera High Quality；
- 目标执行中；
- 连续 5 次。

---

## 27. 性能结果表

| 场景 | GUI/Headless | CPU 模式 | Camera | RViz Image | RTF | Controller 目标 | Controller 实际 | Missed | Scan P99 Age | Image P99 Age | TF P99 Lag | GPU | 成功 |
|---|---|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
| warehouse_v1 short | GUI | powersave | off | off | | 10 | | | | | | | |
| warehouse_v1 short | GUI | performance | monitoring | on | | 10 | | | | | | | |
| warehouse_v1 3 m | Headless | performance | off | off | | 10 | | | | | | | |
| warehouse_v1 3 m | Headless | performance | monitoring | off | | 10 | | | | | | | |
| warehouse_v1 long | GUI | performance | standard | on | | 10 | | | | | | | |
| warehouse_v2 | GUI | performance | monitoring | on | | 10 | | | | | | | |
| realistic | Headless | performance | monitoring | off | | 10 | | | | | | | |

---

## 28. 最低验收标准

### 28.1 运行时

- Interactive RTF 接近 1.0；
- 10 Hz Controller 稳定；
- 不持续掉到 3～5 Hz；
- 无正常 Scan Invalid Source；
- 无持续 Future Extrapolation；
- 无重复 Failed to make progress；
- Local Plan 正常；
- Lifecycle 显示准确；
- Shutdown 无异常。

### 28.2 Camera

- RViz 可实时看到机器人前方；
- 方向正确；
- 与转向同步；
- Monitoring 接近 15 Hz；
- Image Age 不增长；
- CameraInfo 正确；
- TF 正确；
- Publisher 唯一；
- Reset 恢复；
- Camera Off 可完全关闭；
- Headless 默认 Off；
- Camera 不进入导航反馈链；
- Camera Monitoring 不破坏稳定 10 Hz Controller。

### 28.3 地图

- warehouse_v1 成功；
- warehouse_v2 标定契约；
- 未标定 auto fail fast；
- rviz initial pose 正常；
- 标定后 auto 正常。

### 28.4 Realistic

- `/wheel/odom`；
- `/imu/data`；
- EKF；
- `/odom` 唯一；
- Camera 正常；
- 导航成功。

---

## 29. Git 提交建议

```text
docs: add runtime reliability performance and camera upgrade plan
```

```text
feat(env): unify ROS CLI environment and diagnostics
```

```text
feat(teleop): add runtime speed adjustment
```

```text
feat(camera): publish front RGB image and camera info
```

```text
feat(rviz): add robot front camera views
```

```text
feat(runtime): add RTF pacing and profiling
```

```text
fix(nav): stabilize scan TF and collision timing
```

```text
perf(nav): tune MPPI and localization CPU usage
```

```text
feat(mapping): enforce map-specific calibration
```

```text
fix(shutdown): cleanly stop camera ROS and RViz processes
```

```text
docs: update operation verification and troubleshooting guides
```

禁止 Force Push，禁止提交运行时生成目录和大量图像。

---

## 30. 文档更新

### README

增加：

- Camera 快速启动；
- Camera Profile；
- RViz Camera View；
- 性能注意事项。

### user_manual

增加：

- Camera 启用；
- Camera 关闭；
- Camera Profile；
- RViz 面板；
- Camera-only View；
- Teleop 调速；
- 新终端环境；
- Performance Mode；
- 地图标定。

### troubleshooting

按症状：

```text
Camera topic missing
RViz shows No Image
Camera image frozen
Camera image delayed
Camera image upside down
Camera image mirrored
CameraInfo missing
Camera frame missing
Camera publisher duplicated
GPU load too high
Controller slows down after enabling camera
Unknown topic
Invalid frame
Controller missed rate
Scan invalid
Future extrapolation
Local Plan missing
Lifecycle inactive
Map uncalibrated
Teleop too slow
RViz exit code -6
```

每项包含：

- 症状；
- 原因；
- 检查命令；
- 修复；
- 禁止做法。

### interfaces

记录 Camera Topic、Type、QoS、Frame、Owner、Profile。

### verification

增加 Camera 验证矩阵和性能表。

### repository_index

登记新增脚本、配置、节点和 RViz 文件。

---

## 31. 最终完成定义

只有以下条件全部满足才可声明完成：

- 保留当前工作树修改；
- 本计划文件存在并更新；
- 新终端环境一致；
- Teleop 动态调速；
- RTF 可测量和控制；
- Runtime Profiler 可用；
- Controller 频率验证；
- Scan/TF 延迟解决；
- Collision Monitor 不误停车；
- Local Plan 正常；
- Lifecycle 正常；
- warehouse_v2 标定契约；
- Realistic 可验证；
- Camera RGB Publisher 完成；
- CameraInfo 完成；
- Camera TF 完成；
- RViz 第一视角完成；
- Camera Profile 完成；
- Camera 性能验证完成；
- Camera Reset/Shutdown 完成；
- 关闭无协程和 RViz 异常；
- 自动测试通过；
- Headless 运行通过；
- GUI Camera 人工验收通过；
- 文档更新；
- Git 提交完成；
- Push 完成或明确报告权限阻塞。

不得只输出“建议用户手动测试”。

当前环境能够自动执行的检查、Headless 验证、Topic/TF/QoS/Stamp 验证和性能测试必须实际执行。

GUI 中必须人工判断的内容，例如相机朝向、画面是否被机器人外壳遮挡、RViz 面板大小和视觉体验，可以列为最终人工验收项，但必须提供明确步骤和截图清单。
