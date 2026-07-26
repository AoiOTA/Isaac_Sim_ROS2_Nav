# `warehouse_new` 地图与出生点标定

> 最近复核：2026-07-26<br>
> 适用分支：`main`

本手册只描述当前分发的 `warehouse_new` 地图 bundle 及其出生点契约。当前 Kujiale 运行、
4×20 实验和单轮 GUI 诊断均使用这一套地图；不要将其他地图名称、旧 Pose Graph 或旧路线参数代入。

## 1. 当前批准的地图 bundle

地图清单为 [`../data/maps/manifests/warehouse_new.yaml`](../data/maps/manifests/warehouse_new.yaml)，它将下列四个工件绑定为一个不可拆分版本：

| 角色 | 路径 |
| --- | --- |
| OccupancyGrid 描述 | `data/maps/occupancy/warehouse_new.yaml` |
| OccupancyGrid 图像 | `data/maps/occupancy/warehouse_new.pgm` |
| Pose Graph | `data/maps/posegraphs/warehouse_new.posegraph` |
| Pose Graph 数据 | `data/maps/posegraphs/warehouse_new.data` |

当前地图分辨率为 `0.05 m`，尺寸为 `154 × 248` 栅格，原点为 `[-5.140, -6.520, 0]`。
清单中记录每个工件的大小、SHA-256、bundle SHA-256、场景资产、机器人、里程计模式与标定信息；
启动和正式实验会校验这些绑定关系。

## 2. 标定与出生点

基础标定点 `mapping_start` 使用 Ideal 里程计：

| 坐标系 | 位置 | 朝向 |
| --- | --- | --- |
| USD | `[2.9, -0.2, 0.0635]` | `180°` |
| map | `[0.0, 0.0]` | `0°` |

清单的标定方法为 `ideal_mapping_anchor_plus_occupancy_grid_scan_correlation`；位置标准差为 `0.05 m`，
朝向标准差为 `1°`。所有长路线出生点均从它通过同一受控变换派生，**不是新的地图标定**：

| 出生点 | map 位姿 `[x, y, yaw]` | 用途 |
| --- | --- | --- |
| `long_route_start_g1` | `[0.45, -5.35, 90°]` | 4×20 和全屋 GUI 路线 |
| `long_route_start_g2` | `[0.80, 4.80, -160°]` | 聚焦 G2→G3 动态诊断 |
| `long_route_start_g5` | `[-2.20, -2.95, -42°]` | 聚焦 G5→G1 动态诊断 |

完整出生点定义由
[`../isaac_sim/configs/environments/kujiale_0026_A_to_B_door_open.spawn.yaml`](../isaac_sim/configs/environments/kujiale_0026_A_to_B_door_open.spawn.yaml)
维护。脚本会将出生点的 `map_version` 和 `map_bundle_sha256` 与 manifest 对照；不一致时拒绝启动。

## 3. 当前使用方式

常规 Ideal 导航使用默认 `warehouse_new`：

```bash
cd /home/lyb/Workspace/Isaac_Sim_ROS2_Nav
source ./scripts/setup_ros_env.sh
./scripts/run_ros.sh navigation odometry_mode:=ideal \
  spawn_pose_name:=long_route_start_g1 nav2_profile:=stable
```

4×20 的一键入口会在启动前自动检查上述四个工件、地图/场景哈希和出生点契约：

```bash
./scripts/run_kujiale_4x20_all.sh
```

`warehouse_new` 当前只批准普通 Ideal localization/navigation。若需要 Realistic 或显式
Pose Graph 定位，必须先创建新的地图 bundle、完成独立标定并更新 manifest；不能复用这一 bundle。

## 4. 创建新地图的最小流程

1. 使用映射 bringup 保存新的 OccupancyGrid、Pose Graph 与数据文件。
2. 创建新的 manifest，绑定四个工件的路径、大小与 SHA-256。
3. 在实际场景中标定 `mapping_start` 的 map/USD 对应关系，并记录误差与方法。
4. 为要使用的出生点写入派生 map/USD 位姿、`map_version` 和 `map_bundle_sha256`。
5. 冷启动复核 `/map`、`map → odom`、`odom → base_link`、地图与出生点绑定。
6. 只有完成上述步骤后，才可将新 bundle 用于 `initial_pose_source:=auto`、正式导航或4×20实验。

改动地图、出生点或标定后，必须重新生成路线示意图并重新运行受影响的正式实验；旧 campaign 不能继承新地图结论。
