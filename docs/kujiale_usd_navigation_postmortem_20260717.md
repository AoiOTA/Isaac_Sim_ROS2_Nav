# 2026-07-17 酷家乐 USD 场景导航问题复盘

本文记录 2026-07-17 将项目从官方 `warehouse_v2` 切换到酷家乐自建室内 USD 后，围绕资产加载、仿真组合、RTX LiDAR、TF、建图、地图标定、RViz、跟随相机和狭窄空间导航遇到的问题、根因、修复方法与验证结果。

这不是一份“所有告警都已消失”的报告。本文会明确区分已经解决的运行问题、不会阻塞当前 Ideal 导航的源资产告警，以及尚未进入本阶段验收范围的能力。

## 1. 本次工作的范围和最终状态

本次工作限定在以下范围：

- 使用酷家乐场景 `kujiale_0026_A_to_B_door_open.usd`；
- 使用 Ideal 里程计完成建图、标定和复杂导航验证；
- 地图名称为 `warehouse_new`；
- 导航以前进为主，当前关闭主动倒车，MPPI 的 `vx_min` 保持为 `0.0 m/s`；
- 保留 `codex/rviz-workflow-upgrade` 上已经验证的 `warehouse_v2` 基线，酷家乐适配在独立分支 `codex/kujiale-navigation-mapping` 中开发。

当前已经达到的状态：

- 酷家乐场景可以通过统一脚本选择并以只读源资产 + 可写运行时 overlay 的方式加载；
- Jackal、PhysicsScene、RTX LiDAR、ROS 2 Bridge、Ideal 里程计和 Nav2 能正常协同运行；
- `warehouse_new` 四件套地图和 manifest 已纳入版本管理；
- Map、USD 出生点和 RTX 世界坐标已完成三次冷启动一致性标定；
- RViz 会随 Navigation 启动，能够区分全局路径、局部路径、MPPI 最优轨迹、候选轨迹和实际运动轨迹；
- 相机作为 `base_link` 子坐标系随车运动，并调高、后移和扩大视野；
- 相同方向的窄空间实测路线由约 `49.09 s` 降至 `27.87 s`，耗时下降约 `43%`，最终状态为 `SUCCEEDED`。

本阶段没有宣称完成：

- Realistic 轮速里程计下的 `warehouse_new` 定位和导航验收；
- 使用 `warehouse_new.posegraph` 做真实扫描匹配/回环定位；
- 主动倒车规划与倒车恢复；
- 对酷家乐源资产中的每一条材质、拓扑和碰撞告警进行源文件级修复。

## 2. 为什么 `warehouse_v2` 正常，而自建场景问题很多

`warehouse_v2` 是 Isaac Sim 官方资产，已经基本满足项目默认假设：资源依赖完整、USD 层级和元数据稳定、材质路径可解析、物理场景和碰撞结构明确、网格对 RTX 射线可见，并且仓库中已经存在与它匹配的出生点、地图、Pose Graph、TF 和传感器验证结果。

酷家乐 USD 则是外部导出资产。它主要描述“看起来像一个房间”，但不保证直接满足机器人仿真和导航所需的合同：

- USD 及材质压缩包可能分开交付，相对纹理路径只有在保持原目录结构时才成立；
- 根层可能缺少项目预期的 `metersPerUnit`、Z-up、`defaultPrim` 或唯一 PhysicsScene；
- 墙体可能只有单面法线，人在室内能从视口看到，并不代表 RTX LiDAR 从内侧也一定能命中；
- RTX 点云 writer 的坐标语义不同于普通 `base_link` 局部激光雷达；
- 门等复杂网格可能带动态三角网格碰撞，而 PhysX 不能直接按动态 triangle mesh 求解；
- 房间通道更窄，原来在宽阔仓库中很少触发的 footprint、InflationLayer 和速度过滤区会持续影响控制；
- 自建地图没有天然正确的 Map/World 对齐关系，必须重新标定，不能沿用 `warehouse_v2` 的出生点或 Pose Graph。

因此，切换场景不是只把一个 USD 文件名换掉，而是重新验证一整条链：

```text
源 USD/纹理 -> Stage 组合/物理 -> RTX 几何可见性 -> 点云坐标/TF
            -> LaserScan -> 建图 -> Map/World 标定 -> Costmap -> MPPI -> RViz
```

## 3. 问题、根因和解决方案总览

| 现象 | 根因 | 处理方式 | 当前结果 |
| --- | --- | --- | --- |
| 场景整体呈诡异红色，控制台大量纹理缺失 | USD 与 7 GB 材质包分开，纹理相对路径在目录结构不完整时失效 | 将场景和材质移到 `~/kujiale_usd_rooms_20260717`，按房间根目录原样解压 `Materials/Textures` | 主要材质纹理可以解析；缺失 HDR 仍作为已知源资产告警保留 |
| 自建 USD 不能直接接入原运行脚本 | 脚本此前围绕官方环境和固定 Stage 组织 | 增加绝对路径、场景根相对路径和唯一 basename 解析；为每个资产生成隔离的运行时 Stage | 可用统一参数选择自建场景，且不修改源 USD |
| Stage 单位、轴向或物理场景不稳定 | 外部导出 USD 不保证项目元数据和 PhysicsScene 合同 | 在运行时 root layer 明确设置米制、Z-up、项目 Prim；检查并确保恰好一个 PhysicsScene | 组合层满足导航仿真合同 |
| 有墙体在 LiDAR/建图中“消失” | 室内墙网格为单面，RTX 光线从背面无法稳定命中 | 只在运行时 overlay 中将自定义环境 Mesh 设为 double-sided | 室内边界能被 RTX 扫描稳定观察 |
| 激光扫描面方向不对 | RTX LiDAR 本地扫描平面与 Jackal 水平面不一致 | 将传感器绕 X 轴旋转 90°，把本地 X-Z 扫描平面变换到机器人 X-Y 平面 | 获得水平 360° 扫描 |
| 车运动时点云拖影、地图错位 | RTX writer 输出绝对 USD-world 点，而下游曾按机器人局部点解释 | 引入固定 `rtx_world`；按出生点发布 `odom -> rtx_world` 逆变换；启用 compensated 输出 | 移动扫描保持在正确世界位置 |
| 激光命中车体自身 | 传感器能看到 Jackal 外壳，近距离自回波污染 costmap | `range_min=0.40 m`，并使用高度窗口、720-bin LaserScan 转换 | 扫描有效率和 costmap 输入稳定 |
| 地图能保存但导航位置对不上 | 自建 USD 出生点、Map 原点与旧地图标定无关 | 创建 `warehouse_new` 四件套和 manifest，做三次冷启动端点对齐验证 | Ideal 模式下使用 identity `map -> odom` |
| Navigation 没有自动启动 RViz | 自定义运行路径没有完整复用受管 RViz 启动和配置 | 将 Navigation 的 RViz 恢复为统一脚本托管，并补齐 Navigation 2 面板和 Goal Tool | 启动导航时 RViz 自动出现 |
| 第三人称相机不随车或视角太低 | Camera 位于世界坐标或相对位姿不合适 | Camera 直接作为 `base_link` 子 Prim，放到车后约 3.2 m、上方约 2.2 m，注视前方 | 相机随车相对运动，可看到车体和周围环境 |
| 手动能过窄通道，导航却很慢 | 导航的 footprint、膨胀层和速度区比物理通过条件更保守，且 MPPI 进度权重不足 | 使用实车矩形 footprint；缩小无效冗余 padding；重设 inflation/Stop/Slowdown 区；再根据 `/cmd_vel` 证据调 MPPI | 同方向基准 49.09 s 降至 27.87 s |
| 全局路径太细、与实际轨迹颜色混淆，MPPI 看不到 | RViz 默认样式接近，MPPI 可视化原本关闭 | 开启 `visualize`；全局路径黄色加粗，局部路径洋红，最优轨迹橙色，实际轨迹保留青色 | 能同时观察规划、采样和实际跟踪偏差 |

## 4. 资产与 Stage 问题

### 4.1 红色场景与缺失材质

最早的控制台错误包括：

```text
Failed to upload DomeLight texture ./limpopo_golf_course_4k.hdr
Fabric_Normal01.png: asset can not be found
wood01.jpg: asset can not be found
Marble02.jpeg: asset can not be found
```

整个场景发红不是导航算法造成的，而是光照/材质资源解析失败后的渲染表现。酷家乐房间 USD 内部使用类似 `../Materials/Textures/wood01.jpg` 的相对路径；如果只移动 `.usd`，或者把材质包解压到错误层级，路径就会失效。

处理后本机资源组织为：

```text
/home/lyb/
├── kujiale_usd_rooms_20260717/
│   └── kujiale_0026/
│       ├── *.usd
│       ├── Meshes/
│       └── Materials/Textures/
└── kujiale_room_materials_20260717.tar.gz
```

材质归档不再放在 `Downloads`，并保持每个房间原有的 `Materials/Textures` 层级。运行时只修复确实存在目标文件的畸形 `.../` 路径，不会把一个不存在的资源静默替换为另一个文件。

目前 `limpopo_golf_course_4k.hdr` 在交付包中仍不存在，所以 DomeLight 会继续报告一条源资产告警。这不影响当前室内几何、LaserScan、建图和导航；如果后续要求完全清除渲染告警，应获得原 HDR，或在源资产副本中明确替换/禁用该 DomeLight。不能把“导航可用”等同于“源资产依赖百分之百完整”。

### 4.2 不直接修改酷家乐源 USD

运行时会创建稳定、可写、按源资产 hash 隔离的 Stage，再通过 subLayer 引用酷家乐源 USD，并 reference Jackal。这样做可以避免 Isaac 自动保存污染原始场景，在 overlay 中补充米制、Z-up、PhysicsScene、机器人和 Graph，并防止不同房间复用同一个运行时 Stage。

相关实现见：

- [`environment_selection.py`](../isaac_sim/src/environment_selection.py)
- [`stage_loader.py`](../isaac_sim/src/stage/stage_loader.py)
- [`scene_composer.py`](../isaac_sim/src/stage/scene_composer.py)
- [`physics_setup.py`](../isaac_sim/src/stage/physics_setup.py)

### 4.3 仍可见的非阻塞源资产告警

酷家乐导出文件还可能报告：

- 某些 face-varying `displayColor` / `displayOpacity` 数据损坏或长度不匹配；
- 门等动态三角网格碰撞无法直接用于动态刚体，PhysX 回退到 convex hull；
- 前述缺失 DomeLight HDR。

这些是源资产质量问题。当前已验证它们没有阻塞本阶段静态房间内的 Ideal 导航，但若要让门参与真实开合、做高保真相机数据集或追求完全一致的视觉效果，需要回到导出端重建材质和碰撞，而不是继续调 Nav2 参数。

## 5. RTX LiDAR、TF 和建图问题

### 5.1 单面墙和扫描平面

酷家乐墙面主要为视觉网格，法线方向可能只为从房间外侧渲染准备。RTX LiDAR 从房间内部发射射线时，背面剔除会让墙在点云里消失。解决方法是在自定义环境的运行时 overlay 中设置 double-sided，不修改源 Mesh。

RPLIDAR 配置的本地扫描平面是 X-Z，而 Jackal 导航需要水平 X-Y 平面，因此又增加了绕 X 轴 90° 的安装旋转。只修 double-sided、不修扫描面，或只修扫描面、不修单面墙，都无法得到可靠室内地图。

### 5.2 RTX 点云不是普通局部雷达点

Isaac Sim 6.0.1 当前 RTX writer 输出的是绝对 USD-world 端点。若把这些点直接标成 `base_link` 或传感器局部坐标，车辆一运动，下游就会再次施加机器人位姿，造成点云拖影和地图撕裂。

本项目采用以下关系：

```text
map -> odom -> rtx_world
              └── RTX 点云绝对端点

odom -> base_link
       └── Jackal 运动
```

其中 `odom -> rtx_world` 是由所选 USD 出生点推导的固定逆变换，RTX 输出使用 `COMPENSATED` 运动补偿。`pointcloud_to_laserscan` 再把点转换到 `base_link` 目标 frame，提供 Nav2 期望的 2D LaserScan。

### 5.3 自车回波过滤

点云中距离传感器很近的点主要来自 Jackal 自身。当前转换使用高度窗口约 `0.05–0.50 m`、完整 `[-π, π]` 角度范围、720 个角度 bin，以及 `range_min=0.40 m`、`range_max=25.0 m`。

`0.40 m` 大于渲染车体约 `0.334 m` 的外接半径，因此能过滤自车回波，同时保留真正靠近车体的墙和障碍信息。验证中每帧 RTX 点数约 `3058–3098`，中位数 `3079`，发布频率约 `13 Hz`；静止 LaserScan 有效率中位数约 `97.57%`，运动时中位数约 `97.92%`、最低约 `96.94%`。

## 6. `warehouse_new` 建图与标定

### 6.1 为什么不能复用 `warehouse_v2`

OccupancyGrid 不只是一张图片，还包含分辨率、原点、坐标方向和与机器人出生点的关系。`warehouse_v2` 的 Map 坐标和酷家乐 USD 世界坐标没有任何天然联系，直接复用会导致机器人模型、激光和地图互相错位。

本次生成并版本化了不可拆分的四件套：

- `data/maps/occupancy/warehouse_new.pgm`
- `data/maps/occupancy/warehouse_new.yaml`
- `data/maps/posegraphs/warehouse_new.posegraph`
- `data/maps/posegraphs/warehouse_new.data`

权威尺寸、hash、来源和标定证据记录在 [`warehouse_new.yaml`](../data/maps/manifests/warehouse_new.yaml) manifest 中。OccupancyGrid 为 `154 × 248 @ 0.05 m`，原点为 `[-5.14, -6.52, 0]`。

### 6.2 标定结论

当前使用 Ideal 建图和 Ideal 里程计，三次冷启动检查 identity `map -> odom` 的扫描端点对齐：平均端点距离分别约为 `0.0028 m`、`0.0032 m` 和 `0.0028 m`，所有采样端点均在 `0.075 m` 内，三次最大位姿离散为 `0`。因此本阶段采用 identity Map Pose，并在场景专用 spawn 配置中记录 USD Pose、Map Pose 和标定不确定度。

这项结论只对当前 USD、当前出生点、当前 `warehouse_new` 和 Ideal 模式成立。当前 Pose Graph 主要作为本次 Ideal 建图的来源工件保留，并没有完成 Realistic 模式下的扫描匹配和回环定位验收。

## 7. 狭窄空间导航为什么慢，以及如何调优

### 7.1 “手动能通过”不等于“导航应该高速通过”

手动控制主要受物理车宽和操作者判断约束；Nav2 还会同时考虑 footprint 与墙的真实碰撞、costmap inflation、前向 StopZone 和 SlowdownZone，以及 MPPI 的采样速度、预测时域和路径代价。

在宽阔 `warehouse_v2` 中，这些边界很少同时激活。在酷家乐窄通道中，旧的横向安全区和较保守的速度比例会长时间贴着墙触发，表现为“能走，但非常慢”。因此不能只提高最大速度；必须先确认究竟是安全链限速，还是 MPPI 自己输出偏小。

### 7.2 第一轮：让导航几何和真实底盘一致

主要调整包括：

- 使用 Jackal 真实矩形 footprint，约 `0.485 m × 0.420 m`；
- footprint padding 调整为 `0.005 m`；
- 全局和局部 costmap 使用 `0.40 m` inflation radius、`8.0` cost scaling factor；
- StopZone 收紧为车前约 `0.285 m`、车后约 `0.250 m`、横向约 `±0.230 m`；
- SlowdownZone 收紧为车前约 `0.42 m`、车后约 `0.35 m`、横向约 `±0.235 m`；
- SlowdownZone 至少 4 点才触发，速度比例提高到 `0.85`；
- 保留完整 footprint 的逐点碰撞检查，没有为了速度关闭真实碰撞检查。

这一步解决了窄墙边持续误触发限速的问题，同时仍保留紧急停车边界。

### 7.3 第二轮：用命令链证据定位 MPPI

单独监控一条约 `12.096 m` 路线时，MPPI 输出平均绝对线速度约 `0.294 m/s`，安全链最终输出约 `0.298 m/s`。两者几乎相同，说明剩余瓶颈主要不是 Stop/Slowdown 过滤器，而是 MPPI 为了代价最小化主动选择了较慢的进度。

因此继续调整：

- `vx_std=0.35`、`vx_max=0.75 m/s`；
- `wz_std=0.80`、`wz_max=1.20 rad/s`；
- `ax_max=1.10 m/s²`；
- CostCritic 权重 `2.0`；
- PathAlign 权重 `5.0`；
- PathFollow 权重 `9.0`、offset `10`；
- PathAngle 权重 `9.5`、offset `8`。

这些修改鼓励控制器沿路径向前取得进度，并为窄通道内的小幅连续角速度修正提供足够采样空间，而不是只在接近拐角后才剧烈转向。

### 7.4 实测结果

同方向窄空间路线约从 `[0.98, 4.93]` 到 `[-3.45, 3.84]`：

| 版本 | 耗时 | 结果 |
| --- | ---: | --- |
| 最终 MPPI 进度调优前 | `49.09 s` | 成功，但明显偏慢 |
| 最终调优后 | `27.87 s` | `SUCCEEDED` |

耗时下降约 `43%`。最终一次中没有触发 StopZone，只出现两次约 `0.05 s` 的 SlowdownZone 和三次短暂 ApproachZone。期间曾有单个控制周期报告 optimizer 无法计算路径，但控制器自动恢复，目标最终成功。

这里的“丝滑”不是无限提高速度，而是：路径曲率变化时提前产生连续角速度；安全区不因贴近静态墙而长时间抖动；正常直线段能够恢复速度；短暂采样失败不会演变为导航失败。

## 8. RViz 与第三人称相机

### 8.1 RViz 自动启动

Navigation 入口恢复使用受管 RViz 流程，默认加载 navigation 配置，并包含 Navigation 2 面板、Goal Tool、Map、Costmap、TF、路径和轨迹显示。

### 8.2 规划轨迹颜色和宽度

为避免所有路径都呈蓝色而难以区分，当前约定：

| 内容 | Topic | 样式 |
| --- | --- | --- |
| 全局路径 | `/plan` | 亮黄色，线宽 `0.15` |
| 局部路径 | controller local plan | 洋红色 |
| MPPI 最优轨迹 | `/optimal_trajectory` | 橙色，线宽 `0.10` |
| MPPI 候选轨迹 | `/trajectories` | 默认开启 |
| 小车实际运动轨迹 | odometry trajectory | 青色/蓝色 |

MPPI 的 `visualize: true` 会增加一定计算和显示开销，这是为了满足当前调试阶段“直接看到采样和最优轨迹”的需求。后续做纯性能基准时，应记录它是否开启，不要把开关不同的结果直接比较。

### 8.3 跟随相机

相机不是每帧通过脚本拷贝机器人世界位姿，而是直接作为 Jackal `base_link` 的子 Prim。这样父子变换天然保证相机随车相对运动，结构更简单，也不会产生低频跟随抖动。

当前视角约位于机器人后方 `3.2 m`、上方 `2.2 m`，注视前方约 `1.0 m`、高度约 `0.25 m` 的位置，并使用较广视角。它能同时看到 Jackal、下一段路径附近的墙体和拐角，适合判断控制器是否提前转向。

## 9. 验证结果与边界

| 检查项 | 结果 |
| --- | --- |
| 自定义 Stage 组合、唯一 PhysicsScene、米制/Z-up | 通过 |
| `warehouse_new` 四件套完整性与 manifest | 通过 |
| 三次冷启动 Ideal Map/World 标定 | 通过 |
| RTX 点云数量、频率和移动扫描有效率 | 通过 |
| Navigation 自动启动 RViz | 通过 |
| 跟随相机与 `base_link` 父子关系 | 通过 |
| `/optimal_trajectory` 和 `/trajectories` 实时消息 | 通过 |
| Nav2 controller lifecycle active | 通过 |
| 窄空间同方向基准目标 | `27.87 s`，`SUCCEEDED` |
| 主动倒车 | 本阶段关闭，未验收 |
| Realistic 定位/导航 | 本阶段未验收 |
| Pose Graph 真实扫描匹配/回环 | 未批准用于当前交付 |
| 源 USD 所有材质/拓扑告警清零 | 未完成，也不作为当前导航通过条件 |

Nav2 1.3.12 启动 `SmacPlanner2D` 时仍可能打印 inflation 相关 `ERROR`。这是该版本通用 collision checker 的已知诊断行为；当前双 costmap 的 `0.40 m` inflation radius 仍大于约 `0.337 m` 的带 padding 外接半径，且完整矩形 footprint 碰撞检查保持启用。详细限定见 [`verification.md`](verification.md#nav2-1312-smac-inflation-diagnostic)。

## 10. 当前复现方式

终端一启动 Isaac Sim：

```bash
cd /home/lyb/Workspace/Isaac_Sim_ROS2_Nav
./scripts/run_isaac.sh \
  --environment-usd kujiale_0026_A_to_B_door_open.usd \
  --navigation-mode localization \
  --mode ideal
```

等待场景和 ROS Bridge 就绪后，在终端二启动 Navigation 与受管 RViz：

```bash
cd /home/lyb/Workspace/Isaac_Sim_ROS2_Nav
./scripts/run_ros.sh navigation odometry_mode:=ideal
```

当前酷家乐分支默认选择 `warehouse_new`。如果显式传入其他地图，脚本仍会按 basename 检查地图和场景出生点配对，避免误把 `warehouse_v2` 的标定用于酷家乐场景。

## 11. 以后接入自建 USD 的建议顺序

遇到新场景时，建议严格按以下顺序验证，不要一开始就调 MPPI：

1. 检查 USD、Meshes、Materials/Textures 和环境光资源是否完整；
2. 检查 `metersPerUnit`、up axis、defaultPrim、PhysicsScene 和碰撞；
3. 静止检查 RTX 是否能从室内看到所有墙体，必要时只在 overlay 设置 double-sided；
4. 确认 RTX 输出 frame 和坐标语义，再验证车辆运动时点云是否保持静态贴墙；
5. 过滤自车回波，验证 LaserScan 的频率、有效率和近障碍连续性；
6. 用 Ideal 模式建图，版本化四件套和 manifest；
7. 做至少三次冷启动 Map/World 标定；
8. 先验证 footprint、costmap 和安全区，再用 `/cmd_vel` 链定位 MPPI 是否真的限速；
9. 最后才调整速度、角速度、路径进度和曲率相关权重；
10. 单独验收 Realistic 定位、倒车和动态障碍，不把它们混入 Ideal 基线结论。

这次问题的核心教训是：自建 USD 导航是一项系统集成工作。渲染、物理、传感器、TF、地图和控制器任何一层的坐标或安全边界不一致，最终都会表现为“地图有问题”“车转不动”或“导航很慢”。只有先用数据确定问题发生在哪一层，再修改对应层，才能获得稳定、可复现的丝滑导航。
