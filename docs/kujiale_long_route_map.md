# Kujiale 全屋长距离测试地图与航点

这两张图是静态和动态长距离测试的项目内地图示意。它们直接使用
`data/maps/occupancy/warehouse_new.yaml` 与 `warehouse_new.pgm` 的坐标变换生成，
不是来自 Isaac GUI 截图；因此图中的航点、障碍和 `map` 坐标与 Nav2/runner 使用的
输入一致。

## 静态场景

![静态长距离地图与航点](figures/kujiale_long_route_static_map.png)

静态场景在 `[-0.15, 0.70]` 放置 `rgbd_low_box`（`0.30 × 0.30 × 0.16 m`）。它是
RGB-D VoxelLayer 的低矮障碍，二维 LiDAR Scan 不应被用来否定该障碍的存在。

## 动态场景

![动态长距离地图与航点](figures/kujiale_long_route_dynamic_map.png)

动态图中的紫色箭头是物理障碍的运动区间，而非机器人规划路径：
`central_crossing` 在 G1 后触发、`north_crossing` 在 G2 后触发、
`south_crossing` 在 G6 后触发。三者完成运动后均 retire。

## 冻结航点

坐标单位为米，全部在 `map` 坐标系。`S` 是标定的 `mapping_start`；G8 与 S 坐标重合，
表示全屋路线返回起点。G1 至 G7 在两张图中使用完全相同的图标、尺寸和颜色；G4 只是
普通航点，不使用厕所专属样式。

| 顺序 | 航点 | 区域标签 | `map` 坐标 `[x, y]` | 朝向 |
|---:|---|---|---:|---:|
| 0 | S | mapping_start | `[0.00, 0.00]` | `0°` |
| 1 | G1 | 上方大房间 | `[0.80, 4.80]` | `-135°` |
| 2 | G2 | 左上房间 | `[-3.45, 3.90]` | `0°` |
| 3 | G3 | 最左侧房间 | `[-4.05, 1.15]` | `0°` |
| 4 | G4 | 左侧厕所 | `[-3.25, -0.45]` | `0°` |
| 5 | G5 | 左下房间 | `[-2.50, -3.35]` | `90°` |
| 6 | G6 | 下方中部/右侧房间 | `[0.65, -4.25]` | `-90°` |
| 7 | G7 | 最下方房间 | `[0.45, -5.35]` | `90°` |
| 8 | G8 | 返回起点 | `[0.00, 0.00]` | `0°` |

青绿色虚线只说明 runner 发送 Goal 的顺序 `S → G1 → … → G8`；它不等同于理论最优
路线、首次规划路线或某次运行的 GT 轨迹。正式运行的这些结果仍应以各批次的
`benchmark.json`、`runs/` 和报告地图为准。

## 重新生成

图由项目脚本生成；修改 `warehouse_new`、路线或障碍配置后，应重新生成并审阅图片：

```bash
python3 scripts/generate_kujiale_long_route_maps.py
```

生成器会核对静态与动态 G1–G8 路线是否一致，并按 OccupancyGrid 原点、分辨率和 PGM
上下翻转规则转换 `map` 坐标到像素。
