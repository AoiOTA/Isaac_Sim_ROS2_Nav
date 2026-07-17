# 运行排障手册

本文按“看到什么现象”组织排查步骤。日常操作先看 [`user_manual.md`](user_manual.md)，接口唯一所有权以 [`interfaces.md`](interfaces.md) 为准，已验证边界以 [`verification.md`](verification.md) 为准。

## 1. 先做这三件事

不要一上来 `pkill python`、删除整个 `/dev/shm` 或重复启动一套 ROS。先在仓库根执行：

```bash
./scripts/diagnose.sh | tee /tmp/isaac_nav_diagnose.txt
git status --short --branch
./scripts/clean_runtime.sh --dry-run
```

`diagnose.sh` 只读收集环境、受管 PID、项目进程、ROS 图、重复节点、Lifecycle、QoS、TF、仿真时间、Fast DDS SHM、CPU governor 和近期日志。`clean_runtime.sh --dry-run` 只显示将处理的受管进程。

标准恢复顺序是：

1. 在 ROS 终端按 `Ctrl+C`，等待 RViz/Teleop/Nav2 退出；
2. 在 Isaac 终端按 `Ctrl+C`；
3. 若终端异常消失，再执行 `./scripts/clean_runtime.sh --dds-shm`；
4. 先启动 Isaac，看到 `Isaac navigation simulation ready`；
5. 再启动与其模式一致的 ROS 操作。

## 2. 启动前就失败

### 2.1 地图是 Git LFS 指针

现象包含 `Git LFS artifact is not hydrated`、Pose Graph 太小或 Localization 反序列化失败。

```bash
git lfs install
git lfs pull
./scripts/preflight.sh
```

`warehouse_v2.posegraph`、`.data`、OccupancyGrid YAML/PGM 是同一版本的不可拆分工件。不要只替换其中一个。

### 2.2 ROS 包或可执行文件找不到

```bash
./scripts/build_ros2.sh
source /opt/ros/jazzy/setup.bash
source "$PWD/ros2_ws/install/setup.bash"
ros2 pkg prefix robot_bringup
ros2 pkg prefix robot_teleop
```

修改 Python 包、launch、RViz 或配置后必须重新构建。脚本入口会自动 source 环境；手工 `ros2` 命令不会。

### 2.3 提示已有 Isaac、ROS、RViz 或 Teleop 实例

先确认是否真的有一个正在使用的会话：

```bash
./scripts/diagnose.sh
./scripts/clean_runtime.sh --dry-run
```

确认旧会话已经失效后执行：

```bash
./scripts/clean_runtime.sh
```

清理器只接受仓库运行目录里的 PID 元数据，并再次核对进程命令身份；身份不匹配时会拒绝发送信号。

### 2.4 Fast DDS SHM 锁或端口错误

常见日志包含 `open_and_lock_file failed`、`RTPS_TRANSPORT_SHM`、`fastrtps_port`。先停止 Isaac、ROS 和所有额外的本项目 ROS CLI，再运行：

```bash
./scripts/clean_runtime.sh --dds-shm
```

该命令会停止 ROS 2 CLI daemon，扫描进程映射，并且只删除当前用户拥有的 Fast DDS SHM 工件。只要仍有进程加载 Fast DDS，它就会拒绝删除。不要使用 `rm -rf /dev/shm/*`。

## 3. ROS 看不到 Isaac Topic

所有终端检查：

```bash
echo "$ROS_DISTRO"
echo "$ROS_DOMAIN_ID"
echo "$RMW_IMPLEMENTATION"
```

基线应为 Jazzy、`42`、`rmw_fastrtps_cpp`。然后确认 Isaac 已 ready：

```bash
ros2 topic echo /clock --once
ros2 topic info /lidar/points_raw --verbose
```

仍看不到时：

- 确认没有在一个终端使用 Cyclone DDS、另一个使用 Fast DDS；
- 确认容器/主机没有覆盖 `ROS_LOCALHOST_ONLY`；
- 正常停止两端，清理残留 runtime/SHM，再按 Isaac→ROS 顺序启动；
- 不要靠反复执行 `ros2 daemon start` 掩盖域或 RMW 不一致。

## 4. RViz 没有地图、扫描或机器人

### 4.1 先确认加载了正确配置

| 操作 | 配置 |
| --- | --- |
| Mapping / Incremental Mapping | `mapping.rviz` |
| Localization | `localization.rviz` |
| Navigation | `navigation.rviz` |

默认 `run_ros.sh` 自动选择；手动恢复可执行 `./scripts/run_rviz.sh <operation>`。Fixed Frame 必须是 `map`。

### 4.2 地图 QoS

```bash
ros2 topic info /map --verbose
```

RViz Map 必须使用 Reliable + Transient Local。Mapping 中 `/map` owner 是 SLAM Toolbox；Localization/Navigation 中 owner 必须是 `map_server`。后两种模式的 `/slam_toolbox/map` 只是默认关闭的诊断层。

### 4.3 扫描和点云 QoS

```bash
ros2 topic info /scan --verbose
ros2 topic info /lidar/points_raw --verbose
```

RViz 的 LaserScan/PointCloud2 endpoint 必须是 Best Effort + Volatile。仓库顶层 launch 会先载入 RViz 配置，再延迟 1.5 秒启动扫描投影，避免 RViz display 构造时的瞬时默认 Reliable 警告。最终 endpoint QoS 才是判断依据。

### 4.4 RobotModel 不显示

```bash
ros2 topic echo /robot_description --once
ros2 run tf2_ros tf2_echo base_link lidar_link
```

默认 Isaac 拥有结构 TF，ROS 只发布 `/robot_description`。Realistic + `structure_tf_source:=rsp` 时才由 RSP 发布结构 TF。两端选择不一致会造成缺 TF 或重复 TF。

## 5. TF 或所有权异常

正确主链只有：

```text
map -> odom -> base_link
```

检查：

```bash
ros2 topic info /odom --verbose
ros2 topic info /map --verbose
ros2 topic info /tf --verbose
ros2 topic info /tf_static --verbose
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

常见原因：

- `/odom` 两个 publisher：Isaac 使用 Ideal，但 ROS 同时启动 Realistic EKF；
- `/map` 两个 publisher：同时运行 Mapping 与 Localization/Navigation；
- 结构 TF 重复：Isaac 与 RSP 同时启用；
- `map → odom` 缺失：SLAM 未收到 `/scan`、没有有效初始位姿或 Pose Graph 失败；
- TF 时间落后：仿真刚 Reset，旧缓存尚未清除，等待 Gate 完整恢复而不是发送目标。

没有 ROS `world` frame；USD `/World` 是 Stage 路径，不能作为 Nav2 Fixed Frame。

## 6. Navigation 一直没有激活

Activation Gate 会明确打印尚未满足的条件。逐项检查：

```bash
ros2 topic echo /clock --once
ros2 topic echo /scan --once --field header
ros2 topic echo /odom --once --field header
ros2 topic echo /map --once --field info
ros2 run tf2_ros tf2_echo map odom
ros2 lifecycle get /map_server
```

Gate 要求新鲜 `/clock`、`/scan`、`/odom`、已收到 transient-local `/map`，以及连续稳定至少 1 秒的新鲜 `map → odom`。同一个缓存 TF 被重复读取不会刷新新鲜度。

若使用 `initial_pose_source:=rviz`，必须在 RViz 点击 **2D Pose Estimate** 并在实际位置拖出朝向。默认 `auto` 则要求 `spawn_poses.yaml` 中对应 Map Pose 已标定。

不要关闭 Gate 或把 Nav2 `autostart` 改为 true 来绕过 readiness。这会重新引入 Lifecycle 重复转换和 Reset 竞态。

## 7. Lifecycle 重复转换或节点重名

```bash
ros2 node list | sort | uniq -d
./scripts/diagnose.sh
```

正常情况下 Lifecycle 只有 `nav2_activation_gate` 管理；Nav2 launch 的 autostart 是 false。Gate 会先查询六个受管节点状态，使用代次令牌隔离旧 future，并对服务失败进行有限退避重试。

发现重复 `/controller_server`、`/planner_server`、`/bt_navigator` 或第二个 Gate 时，停止整个 ROS 栈并安全清理，不要逐个手工执行 lifecycle transition。

## 8. Reset 后导航不恢复

先看 Trigger 是否真正成功：

```bash
ros2 service call /simulation/reset std_srvs/srv/Trigger '{}'
```

成功响应只表示物理复位以及已排队的 Wheel Odom、EKF、Costmap 请求完成。之后还需要新的扫描、里程计、初始位姿和稳定 TF。

恢复顺序是：Lifecycle pause/cancel → clear costmaps → 自动 reseed 或等待 RViz 位姿 → readiness → resume。`/simulation/reset_event` 是 Volatile 事件，如需观察应在调用 Reset 前另开终端订阅：

```bash
ros2 topic echo /simulation/reset_event
```

Reset 后检查：

```bash
# 仅 auto 模式存在该 transient-local 状态 Topic
ros2 topic echo /initial_pose/status --once
ros2 lifecycle get /controller_server
ros2 run tf2_ros tf2_echo map odom
```

`initial_pose_source:=rviz` 时不会启动自动位姿节点，也没有 `/initial_pose/status`；每次 Reset 后都必须重新给 2D Pose Estimate。`auto` 模式则会等 Reset 后的新 `/scan`，不会重放 Reset 前的缓存扫描。

重叠 Reset 会被拒绝；不要在前一个 Trigger 未返回时再次调用。服务失败时先阅读响应中的具体下游错误，不要把失败当作完成。

## 9. Mapping Teleop 问题

### 9.1 没有弹出终端

自动窗口需要 `gnome-terminal`、`xterm` 或 `konsole`。可以安装其中一个，或：

```bash
./scripts/run_ros.sh mapping odometry_mode:=ideal use_teleop:=false
# 新的交互终端
./scripts/run_teleop.sh
```

### 9.2 一松键就停车

这是安全设计。按键事件超过 `0.18 s` 未刷新就触发 deadman；应按住或重复按键。`Space` 立即停车，`Q`/`Ctrl+C`/`Ctrl+D` 退出并发送最终零速度。

### 9.3 Teleop 拒绝启动

若 `map_server`、`controller_server` 或 `collision_monitor` 存在，说明 Localization/Navigation 正在运行。Teleop 不能与 Nav2 抢占 `/cmd_vel`。先正常停止当前栈，再启动 Mapping。

## 10. MPPI 超时、机器人顿挫或 Collision Monitor 停车

基线参数经过 Isaac Sim 6.0.1 headless Ideal 3 米目标实测：

```text
controller_frequency = 10 Hz
time_steps = 20
model_dt = 0.10 s
batch_size = 500
prediction horizon = 2.0 s
SLAM localization throttle_scans = 2
```

检查实际参数和日志：

```bash
ros2 param get /controller_server controller_frequency
ros2 param get /controller_server FollowPath.time_steps
ros2 param get /controller_server FollowPath.model_dt
ros2 param get /controller_server FollowPath.batch_size
./scripts/diagnose.sh
```

若出现 `Control loop missed its desired rate`：

- 确认没有同时开启多个 RViz、录制高带宽 bag 或运行额外点云处理；
- 检查 CPU governor，`powersave` 会降低余量；
- 先用 `interactive:=false` 做同一路径对照；
- 检查 `/scan` 是否间歇、时间戳是否在 Reset 后跳变；
- 不要未经同场景基准就把控制频率恢复到 20 Hz 或增大 MPPI batch。

Collision Monitor 因扫描超时停车是安全行为。应修复 `/scan` 新鲜度或系统负载，不要增大 timeout 来掩盖数据中断。

## 11. 异常退出后仍有进程

```bash
./scripts/diagnose.sh
./scripts/clean_runtime.sh --dry-run
./scripts/clean_runtime.sh --dds-shm
```

如果清理器提示 PID 命令身份不匹配，它会保留进程和元数据供人工判断。不要修改 PID 文件去强迫它杀进程。Codex/IDE 自身可能因为工作目录包含仓库路径而出现在“project-related processes”只读列表中；它不是 ROS 组件，也不会通过组件身份校验被停止。

## 12. 提交问题时保留哪些证据

至少附上：

- 完整启动命令，以及 Isaac/ROS 两端的 mode、operation、TF source；
- `./scripts/diagnose.sh` 输出；
- 首次错误前后各约 50 行日志；
- `ros2 topic info --verbose /map /scan /odom` 的相关输出；
- Reset 问题的 Trigger 响应和 Gate 恢复日志；
- 当前 commit：`git rev-parse --short HEAD`；
- 是否修改过地图、出生点、Nav2、SLAM、RViz 或 Teleop 参数。

这样可以区分环境残留、模式冲突、QoS、TF 所有权、仿真时间、生命周期竞态和真实性能不足，避免用一次偶然成功覆盖根因。
