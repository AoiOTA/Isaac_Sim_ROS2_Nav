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

### 2.1 Map Manifest、地图完整性或标定失败

**症状：** 启动在创建 ROS 节点前报 `map manifest error`、
`map bundle SHA256 mismatch`、`map_file does not match map manifest`、
`unhydrated Git LFS pointer`、`uncalibrated`、
`spawn pose ... map_version does not match`，或新地图在
`initial_pose_source:=auto` 下被拒绝。

**检查：** 先验证 manifest 本身，再看三处身份是否一致：

```bash
source "$PWD/scripts/setup_ros_env.sh"
ros2 run robot_bringup map_manifest verify \
  --project-root "$PWD" \
  --manifest "$PWD/data/maps/manifests/warehouse_v1.yaml"
git lfs status
git status --short -- \
  data/maps/manifests data/maps/occupancy data/maps/posegraphs \
  isaac_sim/configs/spawn_poses.yaml
```

确认 `posegraph_file`、`map_file`、`map_manifest_file` 指向同一版本；自动位姿
还必须让 manifest 的 `spawn_pose_profile`/`bundle_sha256` 与
`spawn_poses.yaml` 的 `map_version`/`map_bundle_sha256` 完全一致，并逐值核对
USD/Map position、yaw 和两项标准差。

**常见原因：** Git LFS 工件未 hydrated；四个地图工件只更新了一部分；
Occupancy YAML 指错 PGM；文件大小/SHA256 或 bundle hash 已变化；把旧地图
Map Pose 复制到新版本；`save_map.sh` 新建的 manifest 尚未标定。这四个工件
是一个不可拆分 bundle，新保存成功不等于自动位姿已标定。

**修复：** LFS 问题执行 `git lfs install && git lfs pull`。内容损坏时从同一
commit 恢复完整四件套和 manifest，或用 `./scripts/save_map.sh <新版本>`
重新生成，不覆盖旧版本。新版本先使用 `initial_pose_source:=rviz` 人工定位，
再按 [`calibration.md`](calibration.md) 实测并同步更新 manifest 与出生点绑定；
最后重新 `verify`、冷启动 Localization 和 Navigation。

**禁止操作：** 不要只改 SHA256/`calibrated: true` 来绕过校验，不要混用
不同版本的 YAML/PGM/Pose Graph，不要复制旧 Map Pose，不要把 manifest 或其
父目录做成符号链接，也不要删除 manifest 后让系统“猜”地图。

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
source ./scripts/setup_ros_env.sh
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

## 10. Collision Monitor 扫描超时或 `/scan_fault` 不生效

**症状：** 日志出现 `Robot to stop due to invalid source`、scan source timeout
或 TF lookup 失败，`/cmd_vel` 变为零；或者明明发送了故障命令，机器人仍正常
运动；Reset 后旧命令报 `stale epoch`。

**检查：** 先判断当前是生产接线还是显式故障试验：

```bash
ros2 param get /collision_monitor scan.topic
ros2 param get /collision_monitor source_timeout
ros2 topic info /scan --verbose
ros2 topic hz /scan
ros2 topic echo /scan --once --field header
ros2 run tf2_ros tf2_echo base_link lidar_link

# 仅故障试验
ros2 topic info /scan_fault --verbose
ros2 topic echo /scan_fault/status --once
```

生产值应是 `/scan` 和 `0.40` 秒。故障试验必须同时有
`scan_fault_bridge` 的 `/scan_fault` publisher，且临时 Nav2 overlay 让
`scan.topic` 返回 `/scan_fault`。状态 JSON 的 `state.epoch` 是后续命令应携带
的代次。

**常见原因：** `/scan` 持续中断超过 0.40 秒；Scan frame 无法变换到
`base_link`；GPU/CPU 负载使扫描严重陈旧；只启动 bridge 却没有改 Collision
Monitor 输入；只改了输入却没有启动 bridge；Reset/时间戳回退已经清除故障并
开启新 epoch；`replace_frame_id` 故意制造了无效 TF。

**修复：** 生产运行恢复 `/scan`，修复点云投影、TF 或主机负载，等待日志
`Robot to continue normal operation` 后再发目标。故障试验按
[`user_manual.md`](user_manual.md) 的安全矩阵同时启动 bridge 和临时 overlay，
每条 JSON 命令带当前 `epoch`；用 `resume` 或成功 Reset 清除故障。单丢 1～2
帧本来就在 10 Hz/0.40 秒 freshness 窗口内，应验证仍有非零命令；持续丢弃或
无效 frame 才应触发停车。

**禁止操作：** 不要为了“继续走”增大 `source_timeout`、关闭 Collision
Monitor 或直连绕过 `/cmd_vel_smoothed -> collision_monitor -> /cmd_vel`；不要
把 `/scan_fault` 写进生产默认参数；不要把旧 epoch 命令去掉 epoch 后重发来
绕过隔离。

## 11. RViz 没有 Local Plan 或 MPPI 轨迹拖慢运行

**症状：** RViz 的 **Local Plan** 没有线；把
`/transformed_global_plan` 当成局部轨迹后方向看似错误；启用候选轨迹后帧率
或控制余量明显下降；Profiler 的 `local_plan` 计数为零。

**检查：**

```bash
ros2 param get /controller_server FollowPath.visualize
ros2 topic info /optimal_trajectory --verbose
ros2 topic echo /optimal_trajectory --once --field header
ros2 topic hz /optimal_trajectory
ros2 topic info /transformed_global_plan --verbose
ros2 topic info /trajectories --verbose
```

正常 Navigation 中 `FollowPath.visualize=true`，`/optimal_trajectory` 的
publisher 是 Controller Server，消息类型是 `nav_msgs/msg/Path`、frame 是
`odom`，有活动 FollowPath 目标时约按 controller 频率发布。默认
`/trajectories` 可以有 publisher，但 subscriber 数应为 0。

**常见原因：** Nav2 尚未激活或没有活动路径跟随目标；RViz 加载了旧配置、
显示仍指向 `/local_plan`；手工把 `visualize` 关掉；工作空间修改后未重建；
误把 `/transformed_global_plan`（控制器坐标系里的参考全局路径）当成本地最优
轨迹；长期打开了 `/trajectories` 候选 MarkerArray。

**修复：** 使用仓库 `navigation.rviz`，确认显示 Topic 是
`/optimal_trajectory`，重建并重新 source；等待 Gate 激活后发送一个可达目标，
再观察。参考路径只在需要比较时临时打开。候选轨迹调试结束后立刻关闭显示，
并确认 `/trajectories` subscriber 回到 0。

**禁止操作：** 不要新增虚构的 `/local_plan` remap，不要把
`/transformed_global_plan` 写成 Local Plan，不要在日常导航/benchmark 中订阅
`/trajectories`，也不要通过关闭 `visualize` 让 profiler 与 RViz 失去真实局部
轨迹证据。

## 12. Camera 无图、QoS 不兼容或速率不符合预期

**症状：** RViz 显示 `No Image`；两个 Camera Topic 没有 publisher；出现
QoS incompatibility；Image 与 CameraInfo 数量/时间戳不匹配；画面速率低于
profile 配置；分辨率、frame 或画面方向不对；静止画面失焦、运动画面拖影，
或低分辨率画面出现明显的 DLSS 上采样模糊。

**检查：**

```bash
./scripts/diagnose.sh
ros2 topic info /camera/front/image_raw --verbose
ros2 topic info /camera/front/camera_info --verbose
ros2 topic echo /camera/front/image_raw --once --field header
ros2 topic echo /camera/front/image_raw --once --field encoding
ros2 topic echo /camera/front/camera_info --once
ros2 topic hz /camera/front/image_raw
ros2 run tf2_ros tf2_echo base_link camera_front_optical_frame
```

先看诊断识别出的 `camera=<profile>`。`off` 必须是 Image/CameraInfo 各 0 个
publisher；其他 profile 应各有且仅有 1 个 Isaac publisher。两者应使用
`camera_front_optical_frame`、Best Effort/Volatile、depth 2，Image encoding
为 `rgb8`。GUI 默认 `monitoring`，headless 默认 `off`。
同时确认 `camera.yaml` 是 schema v3：`optics.f_stop=0.0` 与
`exposure.f_stop` 分离，`render_product.anti_aliasing=rtxaa`，Motion Blur、DoF
和 Auto Exposure 均为 `false`。

**常见原因：** headless 未显式传 `--camera-profile`；误选 `off`；RViz 用了
Reliable 或 compressed transport；旧 Isaac 实例造成重复 publisher；GPU/渲染
负载或 RTF 低使实际墙钟速率达不到 15/20/30 Hz 配置目标；消费者按到达序号
而不是 header stamp 配对 Best Effort 消息；只改了 Camera YAML、Topic 或 TF
契约的一侧；仍使用把光学 f-stop 与曝光 f-stop 混在一起的 schema v2；或只改
UI viewport 的全局 AA，而没有给 CameraFront RenderProduct 写局部 RTX 设置。

**修复：** 正常停止 Isaac，在原来的完整启动命令中显式加入
`--camera-profile monitoring` 后重启；不要因此改变原有 navigation/odometry
mode。RViz 使用 raw transport、Best Effort/Volatile depth 2。性能不足时从
`high_quality` 降到 `monitoring`，并用 Profiler 记录 RTF、实际 Hz、age 和
Image/CameraInfo stamp 配对。方向、曝光、遮挡必须实际看图，不能只看 Topic。
重复 publisher 先通过受管清理停止旧实例。模糊问题应修改并验证 Camera schema
v3 的局部配置，不要借机改变 UI viewport 的全局渲染策略；代码和配置测试通过
后仍要抓取真实静止/运动图像验收。

**禁止操作：** 不要把 profile 的目标 Hz 当成实测结论，不要为了消除 RViz
提示把传感器改成 Reliable，不要把 Camera 接入 SLAM/Nav2/Collision Monitor，
不要只改 frame 名或光学外参的一端，也不要在纯导航性能基线中忘记显式
`--camera-profile off`。

## 13. RViz 关闭崩溃、Ctrl+C 卡住或 Lifecycle 没有顺序退出

**症状：** 关闭时出现 ROS context invalid、`QThread`/QtConcurrent 警告、
RViz core dump；Ctrl+C 后 Nav2/SLAM/RViz 残留；日志没有 `ordered shutdown`
步骤；再次启动提示 RViz/ROS lock 已占用。

**检查：**

```bash
./scripts/diagnose.sh
./scripts/clean_runtime.sh --dry-run
ros2 node list | sort
ros2 lifecycle get /controller_server  # Navigation 尚在时
```

正常 `scripts/run_ros.sh` 日志会先打印按模式执行的
`ordered shutdown: <step>: PASS`（失败时是明确 `WARN`）。完整关闭共用 20 秒
总期限，Lifecycle helper 默认最多使用 10 秒并保留最后 1 秒；随后本会话认证的
launch、RViz、Teleop 和 helper 独立进程组按 INT→TERM→KILL 有界停止。Navigation
使用只观察 Lifecycle 的 `robot_rviz_plugins/Navigation 2 Safe`；受管组件都有
PID/PGID、start ticks、boot ID、项目根和 session 元数据。

**常见原因：** 手工 `ros2 launch` 绕过 supervisor；直接 kill 了 launch
child、Lifecycle manager 或 RViz；工作空间没有重建，仍加载上游旧 Nav2 panel；
重复 RViz/ROS 会话；终端被强制关闭，来不及调用 Lifecycle 服务；遗留 PID
元数据其实属于仍在运行的进程。

**修复：** 重新构建后只用 `./scripts/run_ros.sh <operation> ...` 启动。在该
终端按一次 Ctrl+C 并等待：Navigation 先关 Navigation manager 再关
Localization manager；Localization 关其 manager；Mapping 依次 deactivate、
cleanup、shutdown SLAM Toolbox。终端已丢失时先 dry-run，再执行
`./scripts/clean_runtime.sh`；只有还需清理 Fast DDS SHM 且确认没有使用者时才加
`--dds-shm`。清理器会逐个认证遗留的 Isaac/ROS/RViz/Teleop/底盘诊断组，并在每个
信号阶段复核身份。一次 Ctrl+C 走正常顺序；确认卡住时第二次会中断 helper 并对
本会话组请求 TERM，第三次才请求 KILL。若某步是 WARN，保留完整日志并检查对应
服务，而不是假定干净退出。

**禁止操作：** 不要直接 `kill` launch 子进程、手工乱发 lifecycle transition、
删除 PID 文件、`pkill -f ros`/`pkill python`、手工向未认证 PID/PGID 发送
SIGKILL，或在仍有 Fast DDS 进程时清空 `/dev/shm`。只有项目监督器/清理器通过
身份复核后的最终有界 KILL 属于安全契约。

## 14. Profiler 报告为空、CPU 统计异常或性能模式不可复现

**症状：** 报告中关键 Topic 为 0 Hz、Camera/Local Plan 缺失；ROS
registered-process CPU 接近 0 或只统计到 shell wrapper；两次 benchmark 差异
很大；口头称“性能模式”但 governor/EPP/电源 profile 与记录不符；
`performance_mode.sh enable` 后无法再次 enable。

**检查：**

```bash
./scripts/performance_mode.sh status
./scripts/diagnose.sh
./scripts/clean_runtime.sh --dry-run
ros2 param get /controller_server controller_frequency
ros2 param get /controller_server FollowPath.model_dt
ros2 param get /controller_server FollowPath.batch_size
./scripts/profile_runtime.sh \
  --warmup 5 --duration 60 \
  --label navigation_stable \
  --output data/reports/runtime/navigation_stable.json
```

采样前先确认 Isaac、ROS、目标、Camera/RViz/Nav2 profile 都已进入待测稳态。
`stable` 应为 10 Hz、0.10 秒、batch 750；`performance` 应为 10 Hz、0.10 秒、
batch 1000。控制周期大于 `model_dt` 的 overlay 会在节点启动前被拒绝，不是
Profiler 故障。
报告的 `metadata.operation` 应能从 `run_ros.sh <operation>` 识别且与命令一致，
`system.registered_processes.ros` 应采用
`process_group_and_descendants` 聚合并列出真实 ROS member；同时检查 RTF、
Topic Hz/age、TF lag、Lifecycle/参数、missed-control 日志、CPU scaling、温度、
throttle counter 和 GPU start/end snapshot。
若 `cpu_sample_member_set_stable=false`，CPU 百分比应为 `null`，并通过
`cpu_sample_added_members`/`removed_members` 解释进程变动；不要把该窗口的旧式
聚合差分当作有效 CPU。

**常见原因：** Profiler 比待测进程/目标更早启动；warmup 太短或只测了目标
完成后的静止段；Camera `off`、没有活动目标或 RViz 本来关闭；修改 profiler
后未重建/source，仍运行旧实现；受管 PID 元数据 stale；对比运行的 GUI、
Camera、Nav2 profile、Ceres 线程或路径不一致；控制循环日志已有 missed-rate
warning 却只比较平均 Hz；Intel P-state 下 governor 标签
仍显示 `powersave`，但 EPP/电源 profile 已变化，只看一个字段得出错误结论；
上次 enable 的状态文件尚未 restore。

**修复：** 先固定场景、目标和全部 profile，启动 workload 并等待激活，再用
足够 warmup/采样窗运行 Profiler；重建并 source 最新工作空间。正式 benchmark
前执行 `sudo ./scripts/performance_mode.sh enable`，逐项确认输出，结束后无论
成功失败都执行 `sudo ./scripts/performance_mode.sh restore`。若 enable 部分失败，
按脚本提示先 restore，并把实际 governor、EPP 和 power profile 原样写入报告；
不能把失败运行标成性能模式。

**禁止操作：** 不要只看平均 CPU 或单个 target Hz 下结论，不要忽略
`Control loop missed its desired rate` 日志，不要把 GPU
start/end snapshot 写成连续平均，不要比较不同 Camera/RViz/目标/参数的结果，
不要修改报告 JSON 掩盖缺失数据，不要让 benchmark 启动器隐式 sudo/改电源
策略，也不要测试结束后把主机永久留在性能模式。

## 15. 异常退出后仍有进程

```bash
./scripts/diagnose.sh
./scripts/clean_runtime.sh --dry-run
./scripts/clean_runtime.sh --dds-shm
```

如果清理器提示 PID 命令身份不匹配，它会保留进程和元数据供人工判断。不要修改 PID 文件去强迫它杀进程。Codex/IDE 自身可能因为工作目录包含仓库路径而出现在“project-related processes”只读列表中；它不是 ROS 组件，也不会通过组件身份校验被停止。

## 16. 提交问题时保留哪些证据

至少附上：

- 完整启动命令，以及 Isaac/ROS 两端的 mode、operation、TF source；
- `./scripts/diagnose.sh` 输出；
- 首次错误前后各约 50 行日志；
- `ros2 topic info --verbose /map /scan /odom` 的相关输出；
- Reset 问题的 Trigger 响应和 Gate 恢复日志；
- 当前 commit：`git rev-parse --short HEAD`；
- 是否修改过地图、出生点、Nav2、SLAM、RViz 或 Teleop 参数。

这样可以区分环境残留、模式冲突、QoS、TF 所有权、仿真时间、生命周期竞态和真实性能不足，避免用一次偶然成功覆盖根因。
