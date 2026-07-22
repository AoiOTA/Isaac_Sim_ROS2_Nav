# Warehouse 地图坐标标定

本文说明如何标定 Isaac USD 世界中的固定出生位姿与 ROS 保存地图中的 Map
位姿。本分支日常 Ideal 导航默认使用酷家乐 `warehouse_new`；`warehouse_v1` 和
`warehouse_v2` 的记录保留用于 Warehouse 历史复现，不是本分支的默认运行入口。

> 文档状态：当前标定流程 + Warehouse 历史测量记录。当前可执行组合以
> `data/maps/manifests/warehouse_new.yaml`、其场景专用出生点文件和
> [`interfaces.md`](interfaces.md) 为准。

## 1. 标定对象

[`isaac_sim/configs/spawn_poses.yaml`](../isaac_sim/configs/spawn_poses.yaml)
为同一个物理出生点保存两套坐标：

- `usd`：Isaac Stage 中的 `base_link` 位姿，用于物理出生与 Reset；
- `map`：保存地图中 `base_link` 的位姿，用于初始定位、Ground Truth
  对齐、重复实验和增量建图。

OccupancyGrid YAML 中的 `origin` 是“栅格左下角在 Map 坐标系中的位置”，
不是机器人的出生位姿。标定必须测量 `map → base_link`，不能把地图
`origin` 抄到 `spawn_poses.yaml`。

平面坐标关系为：

```text
map_T_usd = map_T_base_start * inverse(usd_T_base_start)
map_T_object = map_T_usd * usd_T_object
```

## 2. Warehouse 历史 `warehouse_v2` 标定结果

`warehouse_v2` 与 `warehouse_v1` 来自同一个
`warehouse_multiple_shelves.usd` 场景，但 v2 完成了整个仓库的建图覆盖。
2026-07-17 的最终记录为：

```text
地图版本: warehouse_v2
OccupancyGrid: 406 × 611, 0.05 m/cell
OccupancyGrid origin: [-14.692, -12.294, 0°]

USD base pose: [4.0, 0.0, 0.0635], yaw 0°
Map base pose: [0.0, 0.0], yaw 0°
保守初始位姿标准差: 0.05 m, 5°
```

`0.0635 m` 是 Jackal 在仓库地面上的实测静止 `base_link` 高度。旧的
`0.10 m` 会让底盘先经历落地/接触瞬态，不应恢复使用。

### 2.1 三次冷启动测量

标定不是根据“v1 与 v2 同场景”直接复制，而是显式加载 v2 Pose Graph
并完成三次 Isaac + ROS 全冷启动。每次都把 Jackal 恢复到同一个 USD
出生位姿，等待 `/scan`、Pose Graph 定位和 TF 稳定后采样：

| 冷启动 | `map → base_link` X/Y | Yaw | v2 Pose Graph | v2 `/map` |
| --- | --- | --- | --- | --- |
| 1 | `[0.000, -0.000] m` | `0.000°` | 成功加载 | `406×611` |
| 2 | `[0.000, -0.000] m` | `0.000°` | 成功加载 | `406×611` |
| 3 | `[0.000, -0.000] m` | `0.000°` | 成功加载 | `406×611` |

在 TF 输出的 `0.001 m / 0.001°` 精度内，最大平移 spread 和最大航向
spread 均为 0。`/map` 只有 `map_server` 一个发布者，`/odom` 只有 Isaac
Ideal Odometry 一个发布者。最终仍保留 `0.05 m / 5°` 的保守初始不确定度，
不把有限输出精度下的零 spread 当作零测量误差。

标定和工件完整性记录在
[`data/maps/manifests/warehouse_v2.yaml`](../data/maps/manifests/warehouse_v2.yaml)。

## 3. 为什么标定时要显式启用 Pose Graph

日常 Ideal Navigation 已经拥有 Isaac 发布的精确 `/odom`。为避免在理想
位姿上再次叠加扫描匹配修正，正常 Ideal Localization 使用新鲜的 identity
`map → odom`，不会启动 SLAM Toolbox 定位。

但地图标定必须验证“这份 Pose Graph 的 Map 坐标”与出生点的关系，不能用
日常 identity TF 反过来证明自己。因此项目提供只用于标定的参数：

```text
posegraph_calibration:=true
```

它只允许 `operation=localization + odometry_mode=ideal`，会临时让 SLAM
Toolbox 加载指定 Pose Graph 并拥有 `map → odom`。Navigation 和普通 Ideal
Localization 仍保持原来的 identity 定位链。

## 4. 可复现标定步骤

### 4.1 生成并冻结同版本四件套

从固定 USD 出生点启动 Ideal Mapping：

```bash
./scripts/run_isaac.sh --navigation-mode mapping --mode ideal
./scripts/run_ros.sh mapping odometry_mode:=ideal
```

缓慢覆盖完整区域，包含左右旋转、走廊两侧和至少一次闭环。确认 RViz 中
没有墙体重影、撕裂或错误闭环后，以一个新版本名同时保存两种地图：

```bash
./scripts/save_map.sh warehouse_v2
```

以下四个文件是不可拆分的同一版本：

```text
data/maps/occupancy/warehouse_v2.yaml
data/maps/occupancy/warehouse_v2.pgm
data/maps/posegraphs/warehouse_v2.posegraph
data/maps/posegraphs/warehouse_v2.data
```

不能拿一次建图的 OccupancyGrid 与另一次建图的 Pose Graph 混用。

### 4.2 准备临时 bootstrap 位姿

新地图尚未测量时，先把受影响的 tracked Map Pose 视为
`calibrated: false`。Localization 会拒绝未标定源，因此制作一个仓库外的
临时副本，只在副本中填入合理初值并设置 `calibrated: true`：

```bash
cp isaac_sim/configs/spawn_poses.yaml /tmp/spawn_poses_calibration.yaml
```

同场景同起点的新地图可以把旧 `[0, 0, 0°]` 作为 bootstrap，但它仍不是
最终测量。此阶段保持 Ground Truth 关闭，避免错误初值被记录为真值。

### 4.3 启动专用标定定位

终端 A 启动 Isaac，并确保它与 ROS 使用同一个临时出生点文件：

```bash
ISAAC_NAV__SPAWN__POSES_FILE=/tmp/spawn_poses_calibration.yaml \
  ./scripts/run_isaac.sh \
  --navigation-mode localization \
  --mode ideal \
  --headless
```

终端 B 显式加载待标定的 Pose Graph 和 OccupancyGrid：

```bash
ISAAC_NAV_SPAWN_POSES=/tmp/spawn_poses_calibration.yaml \
  ./scripts/run_ros.sh localization \
  odometry_mode:=ideal \
  posegraph_calibration:=true \
  posegraph_file:="$PWD/data/maps/posegraphs/warehouse_v2" \
  map_file:="$PWD/data/maps/occupancy/warehouse_v2.yaml" \
  interactive:=false
```

日志必须同时出现：

- v2 `.posegraph` 成功 `Load From File`；
- Map Server 读取 v2 PGM 的正确尺寸、分辨率和 origin；
- SLAM Toolbox 进入 Active；
- 自动初始位姿已发布。

如果扫描与地图不重合，使用 RViz `2D Pose Estimate` 修正 bootstrap，再等待
定位稳定。不要通过扩大 TF 容差掩盖坐标错误。

### 4.4 采集 Map Pose 与所有权证据

机器人保持在固定 USD 出生点，记录：

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo map odom
ros2 topic info --verbose /map
ros2 topic info --verbose /odom
```

使用稳定后的 `map → base_link` X/Y 和 yaw 作为测量值。只有在
`odom → base_link` 确认是 identity 时，才可以把 `map → odom` 直接当成
出生 Map Pose。

### 4.5 做三次独立冷启动

完整停止终端 A、B，再从 4.3 开始重复至少三次。每次都要重新创建 Isaac
进程、重新反序列化 Pose Graph、重新发布初始位姿并重新采样 TF。

项目门限为：

- 三次平移结果最大距离差不超过 `0.05 m`；
- 三次 yaw 最大环形角差不超过 `3°`；
- 每次扫描均与保存地图重合；
- TF 无跳变，`/map` 与 `/odom` 所有者唯一。

不满足门限时应修复地图、初值、TF 所有权或出生点，不能把不稳定结果简单
平均后标记为已标定。

### 4.6 提升标定与 manifest

通过重复性门后，更新 tracked 源：

```yaml
spawn_poses:
  mapping_start:
    usd:
      position: [4.0, 0.0, 0.0635]
      yaw_deg: 0.0
    map:
      position: [MEASURED_X, MEASURED_Y]
      yaw_deg: MEASURED_YAW_DEG
      calibrated: true
      position_stddev_m: MEASURED_OR_CONSERVATIVE_STDDEV
      yaw_stddev_deg: MEASURED_OR_CONSERVATIVE_STDDEV_DEG
```

同时为该版本创建 `data/maps/manifests/<version>.yaml`，记录：

- OccupancyGrid 尺寸、分辨率和 origin；
- 四件套的 byte size 与 SHA256；
- USD/Map 位姿、冷启动次数和最大 spread；
- 建图环境、机器人、里程计模式与覆盖范围。

大 `.posegraph` 必须由 Git LFS 管理。执行 `git lfs status`，确认它不是普通
Git blob，也不是未 hydrate 的 LFS 指针。

## 5. 将当前标定地图用于导航

本分支的默认导航版本是 `warehouse_new`，仅批准用于普通 Ideal
Localization/Navigation。完成构建后不传地图参数即可使用：

```bash
# 终端 A
./scripts/run_isaac.sh \
  --environment-usd kujiale_0026_A_to_B_door_open.usd \
  --navigation-mode localization \
  --mode ideal \
  --headless

# 终端 B；run_ros.sh 自动选择 warehouse_new 的 Pose Graph 和 OccupancyGrid
./scripts/run_ros.sh navigation \
  odometry_mode:=ideal \
  interactive:=false
```

显式传 `posegraph_file` 时，脚本仍按 basename 推导同名 OccupancyGrid。需要
回放历史 Warehouse 基线时才显式覆盖默认值，并保证 Isaac 也回到匹配的 Warehouse 场景：

```bash
./scripts/run_ros.sh navigation \
  odometry_mode:=ideal \
  posegraph_file:="$PWD/data/maps/posegraphs/warehouse_v1"
```

正常 Ideal Navigation 不要传 `posegraph_calibration:=true`。

## 6. 标定后的验证

先检查工件和配置：

```bash
./scripts/preflight.sh
./scripts/build_ros2.sh
./scripts/test.sh --with-isaac
```

Navigation 激活后至少验证：

```bash
ros2 topic echo /map --once --field info
ros2 topic info --verbose /map
ros2 topic info --verbose /odom
ros2 run tf2_ros tf2_echo map base_link
```

还应完成一个穿过酷家乐门洞的 `warehouse_new` 导航目标，确认：

- Map Server 发布 `warehouse_new` 的 `154×248` 栅格；
- 全局规划路径位于已建的酷家乐房间区域；
- Nav2 成功结束，最终姿态与地图一致；
- 无重复 `/map`、`/odom` 或 `map → odom` 发布者。

## 7. 动态障碍与 Ground Truth 重对齐

当前 `warehouse_new` 实测 Map 出生位姿为 `[0, 0, 0°]`，对应酷家乐
`mapping_start` 的 USD 位姿 `[2.9, -0.2, 0.0635]`、朝向 `180°`。若未来标定结果发生变化，必须同时更新
Isaac 物理障碍轨迹，并与 ROS 场景配置中的 Map 轨迹逐项核对：ID、shape、
XY 尺寸、起终点、时长和 `repeat` 都必须一致。

修改仓库 Stage、地图原点、出生 USD Pose、`base_link` 定义或 Pose Graph
中的任一项，都会使现有标定失效。此时应先把 `calibrated` 恢复为 `false`，
重新执行本文流程，再运行导航或实验。
