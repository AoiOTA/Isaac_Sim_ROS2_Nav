# Module2 × Nav2 规划与风险融合

## 所有权边界

Module3/Nav2 始终负责 SLAM/定位、地图、障碍合法性、全局与局部规划、MPPI、
Collision Monitor 和 `/cmd_vel`。Module2 只提供可拒绝的认知先验：

- goal-conditioned SR grid 用于等主代价候选的 tie-break；
- 校准后的 16×16 dynamic cost 用于 Global Costmap 非 lethal 软风险；
- 健康、可靠度、OOD、TTL 和身份 hash。

Module2 不修正位姿、不清除 LiDAR/RGB-D 障碍、不写 Local Costmap、不直接控制
底盘。

## 三个显式 profile

| Profile | 规划 | Global 风险 | Local/控制 |
|---|---|---|---|
| `bio_nav_planning_only` | `BioNavGridBased` | 关闭 | 与传统动态导航一致 |
| `bio_nav_risk_only` | stock planner | 开启 | 与传统动态导航一致 |
| `bio_nav_tiebreak_risk` | `BioNavGridBased` | 开启 | 与传统动态导航一致 |

`stable` 和 `dynamic_avoidance` 不变。融合 profile 文件中的全零 SHA 是
fail-closed 模板，不能直接作为合格运行身份。

## 规划路径

`BioNavGridBased` 通过 `/bio_nav/get_goal_planning_prior` 查询目标条件化冻结 SR，
并订阅 `/bio_nav/module2/planning_prior` 检查当前身份。排序键为：

```text
(stock Smac primary f-cost, -cognitive tie-break score, deterministic serial)
```

认知分数不得改变 primary cost。服务超时、消息过期、goal/map/reset/schema/hash
不一致时，插件调用 stock Smac fallback，而不是用自定义比较器模拟传统路径。

## 风险路径

`CognitiveRiskLayer` 只挂在 Global Costmap：

1. 检查 map/reset/model/qualification hash、消息年龄、finite、health、
   reliability、OOD 及 `risk_rejection_mask`；任一拒绝位非零即回退；
2. 低于冻结阈值的格为 0，其余映射到 `1..80`；
3. 使用 `max(existing_cost, cognitive_cost)`，因此不能清除真实障碍；
4. 健康输入停止后风险最多线性衰减 `0.8 s`；消息年龄超过 `0.5 s`、
   拒绝位非零或身份失败时在下一周期清空整层。

Local Costmap、MPPI 采样和 Collision Monitor 不订阅 Module2 topic。

## 启动

先启动 Integration bridge/Module2 服务并确认身份，再显式选择 profile：

```bash
./scripts/run_ros.sh navigation \
  odometry_mode:=ideal \
  spawn_pose_name:=long_route_start_g1 \
  nav2_profile:=bio_nav_tiebreak_risk
```

正式运行前必须用 `scripts/generate_bionav_fusion_profile.py` 生成绑定真实
map、planning qualification、risk model 和 risk qualification SHA 的参数文件，
并通过 `nav2_profile_params_file:=/absolute/path/generated.yaml` 指定。

工程 smoke 可以确认接口、插件加载和 fallback，不能替代真实路线 Gate。

## 验证当前 worktree 的融合配置

在当前 source worktree 中修改 profile、launch contract 或风险层参数后，不能直接让
`pytest` 从旧的 `ros2_ws/install` 导入 `robot_bringup`；旧 install 可能尚未包含
`bio_nav_*` profile，从而产生与源码无关的假失败。先显式把当前源码放在 Python 搜索路径最前：

```bash
cd /absolute/path/to/Isaac_Sim_ROS2_Nav
PYTHONPATH="$PWD/ros2_ws/src/robot_bringup${PYTHONPATH:+:$PYTHONPATH}" \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python3 -m pytest -q \
  ros2_ws/src/robot_bringup/test/test_bio_nav_fusion_profiles.py \
  ros2_ws/src/robot_bringup/test/test_nav2_profile_contract.py \
  ros2_ws/src/robot_bringup/test/test_mode_contract.py
```

该检查验证 combined profile 只向 Global Costmap 插入 `CognitiveRiskLayer`、保持
Local Costmap 不变，并核验 `BioNavGridBased`、MPPI timing 与 mode contract。通过后仍须
运行 `./scripts/build_ros2.sh`，再以新 install 启动 Isaac；未重建的 install 不能作为当前
源码的运行时验证证据。
