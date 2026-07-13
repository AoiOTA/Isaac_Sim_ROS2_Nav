# 最终验证台账

本文件记录仓库当前实现的可复核证据。截至 2026-07-13，本轮运行环境为
Ubuntu 24.04、Isaac Sim 6.0.1.0、ROS 2 Jazzy、Fast DDS、Nav2 1.3.12、
RTX 4090，ROS Domain 为 `42`。

本文严格区分三类数据：

- **配置目标**：YAML、Launch 或 CLI 请求的值；
- **实际测量**：运行时报告、Topic/TF 快照或终端日志中的观测值；
- **测试夹具**：自动测试创建的临时地图、伪服务或进程，不等同于真实
  Warehouse 运行。

本轮性能报告来自 `.tmp_runtime/reports/*.json`。这些原始报告是本机临时
证据，不进入正常 Git 历史；关键值已回填到本文。报告测量窗口约 12 秒，
Isaac 使用 headless + realtime pacing，目标 RTF 为 `1.0`。报告元数据把 CPU
模式记为 `powersave`；实际内核为 `intel_pstate`、governor 标签为
`powersave`，同时 EPP 为 `performance`，因此不能把结果描述成纯粹的节能
或纯粹的 performance-governor 对比。

## 验收摘要

| 范围 | 当前结论 | 证据边界 |
| --- | --- | --- |
| Map Manifest | `warehouse_v1` 四工件、逐文件哈希、bundle 哈希和标定绑定通过真实仓库校验 | 未生成或运行真实 `warehouse_v2`；未标定 v2 行为来自测试夹具 |
| Collision Monitor / `scan_fault` | 单帧/双帧丢失不停机，持续断流和 TF 缺失停车，恢复及 Reset 清故障均通过实时测试 | 是显式启用的安全测试桥，不是常驻数据通路 |
| Local Plan | `/optimal_trajectory` 为真实 MPPI 局部轨迹，10/15 Hz 均有实测 | `/transformed_global_plan` 是参考全局计划，不是 Local Plan；候选 `/trajectories` 默认不订阅 |
| MPPI | 10/15 Hz 共 12 个可行组合全部完成 3 m 目标且 missed=0；8 Hz 的 6 个组合被硬约束拒绝 | 8 Hz 没有性能数据；它们在 ROS 节点创建前即为无效配置 |
| Ceres | 8/12/16/20 线程均完成 3 m 目标且控制 missed=0；保留 12 线程默认值 | 20 线程的 RTF、Scan 和 TF 延迟更差，不能据线程数推断性能更高 |
| Camera / RViz | Off、Monitoring×RViz Off/On、High Quality×RViz On 已运行；Monitoring/HQ 的 RGB、CameraInfo、导航控制均有报告 | `standard` 未运行；HQ 配置目标 30 Hz，但 RGB 实测约 15 Hz 仿真时间频率；最终人工 GUI 易用性验收未完成 |
| Realistic odometry | `/wheel/odom`、IMU、EKF 唯一 `/odom` 所有权和 10 Hz 控制在实时报告中成立 | 本轮 12 秒报告结束时目标仍 active，没有记录该目标最终结果 |
| Reset | 性能矩阵逐次 Reset、Camera stamp 恢复和 `scan_fault` epoch 隔离均有实时证据 | 不能用 Trigger 成功替代后续定位/TF readiness 检查 |
| Ordered shutdown | ROS 监督器先发 Lifecycle Shutdown，再向独立 launch 进程组转发信号；重复 Navigation 和一次 Mapping 实跑均干净退出 | 新监督器的“每模式/每相机 Profile/目标执行中各连续 5 次”完整矩阵未单独归档，暂不宣称全部验收 |
| 完整自动测试 | `./scripts/test.sh --with-isaac` exit 0：root 454 passed/5 deselected；ROS 445/445；Isaac marker 3 passed | 详细精确计数见“自动测试证据” |

## Map Manifest 与标定

### `warehouse_v1` 仓库基线

实际执行：

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 run robot_bringup map_manifest verify \
  --project-root "$PWD" \
  --manifest data/maps/manifests/warehouse_v1.yaml
```

输出：

```text
map manifest verified: warehouse_v1 bundle=88b91be7fb0afe4364851c59dc3466f560017df5acc5405f3ab590729ded9bac
```

Manifest 中的四个不可分割工件为：

| 角色 | 路径 | 字节数 | SHA256 |
| --- | --- | ---: | --- |
| Occupancy YAML | `data/maps/occupancy/warehouse_v1.yaml` | 136 | `891fdaec103073711e88cf675c9149e7684eb3d92f49913e6cf406adddf6cb6a` |
| Occupancy PGM | `data/maps/occupancy/warehouse_v1.pgm` | 241203 | `24d4a19c9a8e7f2bfdd548cbdf778e8d5c66711a71ceedb2ec5c12626cf509c5` |
| Pose Graph | `data/maps/posegraphs/warehouse_v1.posegraph` | 27834150 | `ad6a995790553d1a1a9b9ba9298d2d9d6ff7321f802f0c97e1c1e02cf19e2092` |
| Pose Graph data | `data/maps/posegraphs/warehouse_v1.data` | 204812 | `de9f482ce3d871177a2dd65d05f99585d3febb46fa397d3073f7cca9ca4f1422` |

OccupancyGrid 声明为 `398 x 606`、`0.05 m/cell`、origin
`[-14.360, -12.247, 0.0]`。Manifest 的 calibrated bundle、
`spawn_poses.yaml` 的 `mapping_start`、地图版本和 bundle SHA256 必须完全一致，
`initial_pose_source=auto` 才允许启动。

已覆盖的失败条件包括：未 hydration 的 Git LFS pointer、文件大小或 SHA256
不匹配、bundle SHA256 不匹配、越界/符号链接路径、YAML 指向错误 PGM、
PGM 尺寸以及 resolution/origin 不一致。`save_map.sh` 的自动测试夹具还验证了
“staging -> 四工件发布 -> bundle 校验 -> Manifest 最后发布”的事务顺序，
并验证 Pose Graph 序列化失败时不留下半成品。

测试夹具创建了一个未标定的 `warehouse_v2`：`initial_pose_source=auto` 会在
节点启动前失败，而 `initial_pose_source=rviz` 允许进入人工初始位姿流程。
这证明的是契约实现，**不是**真实 Warehouse v2 已建图、已标定或已导航。

## Collision Monitor 与 `scan_fault`

安全测试时显式启用 `scan_fault_bridge`，路径为：

```text
/scan -> scan_fault_bridge -> /scan_fault -> Collision Monitor
```

其他 SLAM/Costmap 消费者继续使用原始 `/scan`。桥接输出与 Collision Monitor
使用 Best Effort/Volatile Sensor Data QoS；`scan.topic` 的实时参数读回为
`/scan_fault`，`source_timeout` 为 `0.40 s`。

| 故障动作 | 实际观察 | 结论 |
| --- | --- | --- |
| 正常转发 | `/scan_fault` 唯一 Publisher 为 bridge，唯一外部 Consumer 为 Collision Monitor | 测试链路所有权正确 |
| `drop_next: 1` | 后续 `/cmd_vel` 仍为非零：linear `0.0132 m/s`、angular `0.0582 rad/s` | 单帧丢失不误停车 |
| `drop_next: 2` | 后续 `/cmd_vel` 仍为非零：linear `0.0736 m/s`、angular `0.0613 rad/s` | 双帧丢失不误停车 |
| `drop_all` | 超过 `0.40 s` 后出现 invalid-source 日志并执行安全停车 | 完全断流会停车 |
| `resume` | 日志恢复 `Robot to continue normal operation`，扫描继续转发 | 故障可恢复 |
| `replace_frame_id: missing_scan_frame` | TF 查询失败，随后 invalid-source 停车 | TF 缺失会停车 |
| 故障中 Reset | epoch 从 1 变为 2；mode 恢复 `normal`，remaining=0，replacement=null | Reset 不继承旧故障 |

Reset 后状态快照记录 `received=2658`、`forwarded=2273`、`dropped=385`，
`last_epoch_reason=reset_event`。Reset 后正常 `/scan_fault` 约 `9.35 Hz`。
测试结束时 Navigation 和 Localization Lifecycle manager 均成功完成有序
Shutdown；该故障目标在断流期间触发 progress/recovery 属预期安全行为，不能
把它当作一次普通导航成功样本。

## RViz Local Plan

实时图验证的 Local Plan 契约为：

| 项目 | 实际值 |
| --- | --- |
| Topic | `/optimal_trajectory` |
| 类型 | `nav_msgs/msg/Path` |
| Publisher | `controller_server`，1 个 |
| Frame | `odom` |
| 单条轨迹 | 20 poses（stable 20-step 配置） |
| 10 Hz stable 实测 | `10.001 Hz`，到达间隔 P99 `101.079 ms` |
| 15 Hz 矩阵实测范围 | `15.000–15.001 Hz`，到达间隔 P99 `67.332–67.784 ms` |

`/transformed_global_plan` 是 Controller 使用的参考全局计划，不能标为局部
轨迹。MPPI 候选轨迹 `/trajectories` 保持默认不显示，RViz 没有订阅，实测
external subscriber count 为 0，避免候选批次序列化和渲染开销。

## MPPI 10/15 Hz 性能矩阵

所有可行行使用同一 `warehouse_v1`、Ideal Localization、Camera Off、RViz Off、
`model_dt=0.10 s`、Ceres 12 线程。每行都是新 ROS 启动、Reset、3 m
NavigateToPose、约 12 秒 profiler 和有序退出。表中为实际测量值，显示时做了
有限小数舍入；原始精度保留在对应 JSON 中。

| Hz | Batch / Steps / Horizon | RTF | Controller 实际 Hz | `/cmd_vel_nav` 间隔 P99 | Scan Hz / Age P99 | Local Plan Hz | Host CPU | ROS CPU（单核口径） | Goal / 时长 | Missed |
| ---: | --- | ---: | ---: | ---: | --- | ---: | ---: | ---: | --- | ---: |
| 10 | 500 / 15 / 1.5 s | 0.9553 | 10.0008 | 102.297 ms | 9.571 / 16.667 ms | 10.0004 | 27.86% | 89.50% | succeeded / 6.261 s | 0 |
| 10 | 500 / 20 / 2.0 s | 0.9585 | 10.0006 | 100.840 ms | 9.578 / 16.667 ms | 10.0006 | 27.56% | 87.83% | succeeded / 6.451 s | 0 |
| 10 | 750 / 15 / 1.5 s | 0.9608 | 9.9982 | 101.881 ms | 9.602 / 16.667 ms | 9.9983 | 27.59% | 86.00% | succeeded / 6.260 s | 0 |
| **10** | **750 / 20 / 2.0 s** | **0.9547** | **10.0011** | **101.079 ms** | **9.542 / 31.333 ms** | **10.0011** | **28.75%** | **88.33%** | **succeeded / 6.550 s** | **0** |
| 10 | 1000 / 15 / 1.5 s | 0.9529 | 9.9999 | 100.848 ms | 9.474 / 50.000 ms | 9.9999 | 27.59% | 87.91% | succeeded / 6.151 s | 0 |
| **10** | **1000 / 20 / 2.0 s** | **0.9543** | **10.0008** | **100.843 ms** | **9.478 / 132.167 ms** | **10.0006** | **27.90%** | **91.75%** | **succeeded / 6.451 s** | **0** |
| 15 | 500 / 15 / 1.5 s | 0.9508 | 15.0004 | 67.380 ms | 9.511 / 31.000 ms | 15.0005 | 28.43% | 90.00% | succeeded / 6.711 s | 0 |
| 15 | 500 / 20 / 2.0 s | 0.9644 | 15.0008 | 67.784 ms | 9.626 / 16.667 ms | 15.0008 | 27.91% | 87.92% | succeeded / 6.980 s | 0 |
| 15 | 750 / 15 / 1.5 s | 0.9555 | 15.0003 | 67.332 ms | 9.571 / 16.667 ms | 15.0001 | 28.55% | 91.33% | succeeded / 6.511 s | 0 |
| 15 | 750 / 20 / 2.0 s | 0.9631 | 15.0002 | 67.734 ms | 9.633 / 16.667 ms | 15.0002 | 28.86% | 90.25% | succeeded / 6.851 s | 0 |
| 15 | 1000 / 15 / 1.5 s | 0.9626 | 15.0003 | 67.558 ms | 9.668 / 33.333 ms | 15.0001 | 28.71% | 90.42% | succeeded / 6.581 s | 0 |
| 15 | 1000 / 20 / 2.0 s | 0.9590 | 15.0013 | 67.748 ms | 9.634 / 16.667 ms | 15.0012 | 27.79% | 88.75% | succeeded / 6.711 s | 0 |

两个报告在结束采样时 Controller 参数服务暂时不可用：
`mppi_c10_b0500_t20.json` 和 `mppi_c15_b1000_t20.json`。这两行的配置值来自
各自被 Launch 读取的 overlay 文件和报告 label；Topic 实际频率、目标结果和
其他测量值来自运行时报告，不能把 overlay 值误称为该次参数服务读回。

### 8 Hz 硬约束拒绝

8 Hz 的所有 `batch={500,750,1000} x steps={15,20}` 组合共享：

```text
controller period = 1 / 8 Hz = 0.125 s
FollowPath.model_dt = 0.100 s
```

Nav2 1.3.12 MPPI 要求 Controller period 不大于 `model_dt`。Launch 在构造
任何 ROS Node action 前执行严格校验，因此这 6 行是 **invalid/rejected**，
不是“性能较差”。代表性真实启动约 0.7 秒内失败，明确提示至少使用 10 Hz，
且没有遗留 ROS 节点。其余五个组合由相同频率/DT 硬约束分类，不存在 RTF、
Goal 或 Missed 等测量值。

### 最终 Profile 选择

- `stable`：10 Hz、750 batch、20 steps、0.10 s DT；
- `performance`：10 Hz、1000 batch、20 steps、0.10 s DT；
- 15 Hz 保留为有效 Benchmark 点，不作为交付默认值；交付 Profile 保持
  Controller period 与 `model_dt` 相等；
- performance 行的 Scan Age P99 为 `132.167 ms`，明显高于 stable 行的
  `31.333 ms`，所以“performance”是显式实验 Profile，不表示每个指标都更优。

## Ceres 8/12/16/20 线程矩阵

以下行固定使用 MPPI stable（10 Hz、750、20、0.10 s）、Camera Off、RViz Off，
每行新启动、Reset、3 m Goal 和约 12 秒采样。

| Ceres 线程 | RTF | Controller Hz | Scan Hz / Age P99 | `map->base_link` Lag P99 | Host CPU | ROS CPU（单核口径） | Goal / 时长 | Missed |
| ---: | ---: | ---: | --- | ---: | ---: | ---: | --- | ---: |
| 8 | 0.9604 | 10.0006 | 9.625 / 33.167 ms | 0.000 ms | 27.81% | 88.25% | succeeded / 6.451 s | 0 |
| **12** | **0.9565** | **10.0002** | **9.595 / 33.333 ms** | **16.667 ms** | **28.38%** | **90.50%** | **succeeded / 6.450 s** | **0** |
| 16 | 0.9607 | 10.0014 | 9.605 / 16.667 ms | 16.667 ms | 29.13% | 93.42% | succeeded / 6.450 s | 0 |
| 20 | 0.9342 | 10.0002 | 9.106 / 32.000 ms | 382.500 ms | 27.82% | 90.00% | succeeded / 6.750 s | 0 |

8 线程报告的 Ceres 参数服务读回为 `service_unavailable`，线程数来自该次
CLI/metadata（`ceres_num_threads="8"`）；其余性能值来自实时 Topic/TF 报告。
20 线程降低 RTF 和 Scan rate，并把 TF P99 拉高到 `382.5 ms`。12 线程因此
保留为保守默认值；本轮没有声称 Ceres、SLAM 或 MPPI 获得 CUDA 加速。

## Camera、RViz 与导航组合

Profile 的配置目标是：Monitoring `640x360 @ 15 Hz`，Standard
`640x480 @ 20 Hz`，High Quality `1280x720 @ 30 Hz`。下面把目标与测量值
分开列出。`Image sim Hz` 由图像 Header stamp period P50 换算；`Image wall Hz`
是 profiler 的墙钟到达率，两者不能混用。

| 组合 | 配置目标 | Image sim / wall Hz | CameraInfo wall Hz | Image Age P99 | RTF | Controller Hz / Missed | Scan Hz / Age P99 | Goal 结果 |
| --- | --- | --- | ---: | ---: | ---: | --- | --- | --- |
| Off + RViz On | 无 Publisher | — / — | — | — | 0.8895 | 10.0012 / 0 | 8.844 / 33.333 ms | 报告未记录 Goal |
| Monitoring + RViz Off | 640x360 @ 15 | 15.000 / 12.431 | 13.684 | 25.167 ms | 0.9118 | 10.0005 / 0 | 9.121 / 33.333 ms | 报告未记录 Goal |
| Monitoring + RViz On | 640x360 @ 15 | 15.000 / 8.995 | 12.830 | 16.667 ms | 0.8556 | 10.0003 / 0 | 8.460 / 33.333 ms | 1 succeeded / 7.261 s |
| High Quality + RViz On | 1280x720 @ 30 | **15.000** / 10.800 | 25.717 | 33.333 ms | 0.8564 | 9.9989 / 0 | 8.542 / 33.333 ms | 1 succeeded / 7.261 s |
| **Standard + RViz On** | **640x480 @ 20** | **未运行** | **未运行** | **未运行** | **未运行** | **未运行** | **未运行** | **未运行** |

运行时消息契约：

- Monitoring 图像为 `640x360 rgb8`，High Quality 为 `1280x720 rgb8`；
- Image 和 CameraInfo 均使用 `camera_front_optical_frame`，每个 Topic 各 1 个
  Publisher；
- RViz On 时 Image 有 1 个 RViz 外部订阅者，RViz Off 时为 0；
- Off 时 Image/CameraInfo Publisher 都是 0，即使 RViz Image Display 正在等待
  Topic 也没有崩溃；
- 每个已收到的 RGB frame 都找到 exact-stamp CameraInfo，ratio 为 `1.0`，
  但 CameraInfo 还分别多出 15、48、179 个无对应 RGB 的样本；不能将其描述成
  两个 Topic 完全一一同频；
- HQ RGB 没达到 30 Hz 配置目标，而是约 15 Hz 仿真时间频率，因此 HQ 频率目标
  尚未验收；Monitoring 在仿真时间口径接近 15 Hz，但墙钟率随 RTF/RViz 负载
  下降；
- HQ 报告中 Camera 静态 TF lookup failure 为 0；Monitoring + RViz On 报告记录
  2 次 Camera 静态 TF lookup failure，不能宣称所有组合的 TF 采样均零失败；
- 两张本机 RGB 抓帧已做图像检查，画面朝前、正立、未镜像且转向后内容改变；
  这只是抓帧检查，不等同于用户完成了 RViz 面板、交互和视觉体验的人工作业。

### GPU 指标的重要限制

Camera 和 Realistic 报告采样时，GPU 上同时存在另一个用户的 Isaac Sim 进程
（PID 825090），另有 Sunshine 进程。因此 `nvidia-smi` 的 device utilization、
总显存和功耗是共享设备快照，**不能**作为本项目 Camera On/Off 的独占增量。

| 组合 | GPU Util | 总显存 | 功耗 | 本项目 Isaac 显存 | 并发用户 Isaac 显存 |
| --- | ---: | ---: | ---: | ---: | ---: |
| Off + RViz On | 46% | 7077 MiB | 113.72 W | 2927 MiB | 2690 MiB |
| Monitoring + RViz Off | 45% | 7746 MiB | 111.66 W | 3086 MiB | 3286 MiB |
| Monitoring + RViz On | 50% | 7837 MiB | 116.05 W | 3085 MiB | 3158 MiB |
| High Quality + RViz On | 56% | 7983 MiB | 130.50 W | 3228 MiB | 3158 MiB |
| Realistic Monitoring + RViz Off | 54% | 7631 MiB | 121.48 W | 3086 MiB | 3222 MiB |

这些数字只用于说明采样当时的资源状态。若要得到可信 Camera GPU 增量，必须在
独占 GPU、停止 PID 825090 和其他图形负载后重跑同一矩阵。

## Realistic odometry

`realistic_monitoring_nav.json` 使用 Realistic、Monitoring、RViz Off、stable
Profile。12 秒窗口中的实际结果：

| 指标 | 实际值 |
| --- | --- |
| `/wheel/odom` | `45.893 Hz` wall，1 个 Publisher：`wheel_odometry` |
| `/odom` | `44.806 Hz` wall，1 个 Publisher：`ekf_filter_node` |
| `/imu/data` | `109.456 Hz` wall；Header stamp P50 `16.67 ms`（约 60 Hz 仿真时间），存在重复 stamp 到达 |
| `odom -> base_link` | EKF 所有，lookup failure=0，future stamp=0，Lag P99 `233.333 ms` |
| Controller | 目标 10 Hz，实际 `9.9982 Hz`，missed=0 |
| Scan | `9.127 Hz`，Age P99 `33.333 ms` |
| Camera | `640x360 rgb8`，Image sim/wall 约 `15.000/11.602 Hz`，exact match ratio=1.0 |
| RTF / Host CPU / ROS CPU | `0.9205` / `38.26%` / `100.00%`（ROS 为单核口径） |
| Navigation snapshot | `active_goal_count=1`，没有 succeeded/failed result |

该报告验证的是 Wheel Odom + IMU + EKF 的所有权、数据流和目标执行期间的控制
稳定性，**不能**声称这个特定目标已完成。此前 2026-07-12 的 1 m Realistic
基线有 4/4 成功记录（GT 终点误差 `0.175–0.187 m`），它是独立的历史 smoke
证据，不替代本轮报告，也不是广泛 start/goal 统计结论。

## Reset 证据

本轮可复核的 Reset 证据包括：

- 12 个可行 MPPI 组合和 4 个 Ceres 组合在各自新进程中先完成 Reset/readiness，
  再发送 3 m Goal；16/16 Goal 均 succeeded；
- 一次实时 Navigation Reset 只在 cancel/pause/clear/reseed/wait/resume 的异步
  Future 完成后返回，六个受管 Nav2 节点恢复 active；
- `scan_fault` 在 fault active 时收到 `/simulation/reset_event`，新 epoch 自动恢复
  `normal`，旧 epoch 命令被拒绝；
- Camera Reset 前后 Image stamp 从 `143.333333666` 前进到 `145.0`，没有回退，
  Image/CameraInfo Publisher 数保持各 1 个；
- 自动测试还覆盖旧 Future、超时/服务失败、重叠 Reset、旧 scan epoch 和 stamp
  rollback，旧 generation callback 不能完成新事务；
- 更早的最终实验批次累计完成 13 次 Ideal/Realistic reset/reseed，均通过 post-reset
  scan、`/simulation/localization_seeded`、新 `map->odom`、新 GT/odom 和稳定 Pose
  门控。

Reset Trigger 的 `success: true` 只表示已提交的物理重置和下游服务调用完成。
调用方仍必须等待新的 seed/manual initial pose、fresh `/odom` 和稳定且新 stamp 的
`map -> odom`，不能单凭 Trigger 返回值开始导航。

## Ordered shutdown 与 RViz 安全退出

当前 `scripts/run_ros.sh` 是顶层监督器：它通过 `setsid` 把 `ros2 launch` 放入
独立进程组。收到 INT/TERM/HUP 后，监督器先运行私有 rclpy Context 和
SingleThreadedExecutor 的 `robot_bringup.ordered_shutdown`，再向 launch 进程组
转发 SIGINT 并等待退出。

顺序契约为：

- Navigation：先 `/lifecycle_manager_navigation/manage_nodes` Shutdown，再
  `/lifecycle_manager_localization/manage_nodes` Shutdown；
- Localization：只关闭 Localization manager；
- Mapping/Incremental Mapping：依次向 SLAM Toolbox 发送 deactivate、cleanup、
  shutdown transition；
- 每个握手的默认完整超时为 20 秒；RViz 使用仓库内安全 Nav2 Panel，避免退出时
  callback/Future/Context 竞态。

实际证据：

- 12 个 MPPI 与 4 个 Ceres 新进程运行均以该监督器有序退出，没有 Controller
  `-6`、未等待协程或残留 ROS/RViz 进程；
- Collision 故障测试的两步 manager Shutdown 实测分别约 `3.239 s` 和
  `3.351 s`，全栈干净退出；
- 新监督器下至少一次 Mapping 实跑完成 SLAM 生命周期有序退出；
- High Quality + RViz 首轮退出暴露了 `robot_description` 的 rclpy Context
  竞态，修复后用同一组合复跑明确 clean；
- Realistic 首轮退出暴露了 `wheel_odometry` 未处理的
  `ExternalShutdownException`，修复后复跑明确 clean；
- 最终 `clean_runtime.sh` 检查项目受管进程为 `none`；
- 安全 RViz Panel 在监督器改造前另做了 Navigation、Mapping、Localization、
  Camera View 各 5 次（20/20）启动/退出测试；该数据验证 Panel 本身，但不能
  冒充新监督器的跨模式 20 次矩阵。

尚未单独归档“新监督器下每个模式、Camera Off/Monitoring/High Quality、目标
执行中各连续 5 次”的完整矩阵，也没有独立的 Render Product 泄漏计数报告。
因此当前结论是重复 smoke 通过，不能把计划第 21/26.7 节的全部 shutdown 矩阵
标为最终完成。

## 发布前审查回归

最终三路只读审查没有发现 P0，但发现了地图发布、进程退出和输入边界的阻塞项。
修复后针对性测试与全量测试均重新执行，不能把审查前结果当作最终结果。新增回归
明确覆盖：

- `save_map.sh` no-clobber hard-link 发布、并发同名工件保留、Manifest 链接后的
  信号窗口回滚、父目录 symlink 和纯点版本拒绝；
- Manifest 正数 resolution、父级 symlink/路径身份、USD/Map pose、yaw、两项
  标准差逐值绑定，以及 Localization Reset profile 不可跨 bundle 切换；
- `run_ros.sh` helper 不继承忽略信号、第二次停止强制 TERM、顽固 launch 组的
  有界升级，以及 Ordered Shutdown 的 20 秒全局 deadline；
- Safe RViz Panel 的空 GoalStatus 和非法/超范围循环文本；
- `/scan_fault` 每条命令强制 epoch，Profiler 识别 supervisor operation，并在
  PID+start-time 成员集合变化时拒绝输出误导 CPU 差分；
- `robot_rviz_plugins` 在 `BUILD_TESTING=OFF` 且不查找测试依赖时独立配置、编译和
  安装成功。

## 自动测试证据

最终交付在所有代码修改结束后执行了 `./scripts/test.sh --with-isaac`，命令
exit 0。以下是本轮最终计数，不沿用旧版本：

| Gate | 最终结果 |
| --- | --- |
| `./scripts/preflight.sh` | PASS，包括仓库环境、资产和 `warehouse_v1` Manifest/LFS 检查 |
| `./scripts/build_ros2.sh` | 11 packages build completed |
| `./scripts/test.sh --with-isaac` 的 pure/root suite | 459 collected：454 passed，5 deselected |
| ROS `colcon test` | 11 packages，445 tests，0 errors，0 failures，0 skipped |
| Isaac/USD marker suite | 61 collected：3 passed，58 deselected |
| RViz config/load smoke | 结构测试包含在 454 个 root tests；安全 Panel 20/20 历史循环及本轮 Off/Monitoring/HQ 实跑组合见上文 |
| `robot_rviz_plugins` production-only build | 独立 `-DBUILD_TESTING=OFF` configure/build/install PASS |
| Repository index set comparison | 267 个交付路径对 267 个索引路径，集合差分无输出 |
| `git diff --check` | PASS；全工作树、未跟踪源码和文档在提交前再执行一次最终审计 |

推荐最终命令：

```bash
./scripts/preflight.sh
./scripts/build_ros2.sh
./scripts/test.sh --with-isaac
```

若单独诊断 ROS 测试，必须先 source ROS 和工作空间；直接从未 source 的 Shell
调用 pytest 会因找不到工作空间包而产生环境性 `ModuleNotFoundError`，这不等同
于代码测试失败。

## 仍未验收或不得外推的事项

| 能力 | 当前边界 |
| --- | --- |
| 真实 `warehouse_v2` | 未建图、未生成四工件、未标定、未运行导航；只有临时测试夹具验证 uncalibrated 契约 |
| Camera Standard | `640x480 @ 20 Hz` 组合未运行 |
| Camera High Quality 频率 | 配置目标 30 Hz，RGB 实测约 15 Hz 仿真时间频率，目标未达到 |
| Camera 人工 GUI 验收 | 抓帧方向检查已做，但用户 click-by-click、面板布局和视觉体验验收未完成 |
| GPU Camera 增量 | 并发另一个用户 Isaac Sim，当前共享 GPU 快照不能做独占增量结论 |
| Realistic 本轮目标结果 | 报告结束时目标 active；不得写成该报告 succeeded |
| Localization 冷启动统计 | 未完成 Ideal/Realistic 各 3 次独立冷启动及 Map Pose 离散度量化 |
| 一般导航成功率 | MPPI/Ceres 是固定 3 m 调参矩阵，不是多 start/goal 的总体成功率统计 |
| Static/Dynamic 大矩阵 | 已有固定场景 smoke，但没有完成计划要求的广泛布局、速度、尺寸和种子统计 |
| Incremental-map 收益 | 尚无真实 changed-region 三地图对照及至少 30% 收益证据 |
| Long-duration soak | 未记录长时间稳定性结果 |
| Custom robot | 只有参数化迁移契约和模板，没有真实自定义 USD、标定和全链路验收 |
| 完整 Shutdown 矩阵 | 新监督器的所有模式/Profile/active-goal 连续 5 次矩阵未单独归档 |

以上固定目标运行可以作为回归、Reset 隔离、控制稳定性和资源竞争基线，但不能
把 16/16 调参 Goal、历史 4/4 Realistic 或 4/4 动态 smoke 外推成一般性的
100% 导航成功率。
