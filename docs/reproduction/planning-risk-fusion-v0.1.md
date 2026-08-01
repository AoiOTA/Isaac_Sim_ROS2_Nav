# 规划/风险融合 v0.1 复现

## 环境和接口 underlay

```bash
export MODULE3_ROOT=/home/lyb/Workspace/Bio_Nav/repos/Isaac_Sim_ROS2_Nav
export INTEGRATION_ROOT=/home/lyb/Workspace/Bio_Nav/repos/Bio_Nav_Integration
cd "$INTEGRATION_ROOT/ros2_ws"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select bio_nav_interfaces

cd "$MODULE3_ROOT"
export BIO_NAV_INTERFACES_SETUP="$INTEGRATION_ROOT/ros2_ws/install/setup.bash"
./scripts/build_ros2.sh
```

Integration 是消息/服务定义的权威来源；不要在 Module3 创建接口副本。

## 自动验证

```bash
cd "$MODULE3_ROOT"
python3 -m pytest -m 'not isaac and not ros'

source /opt/ros/jazzy/setup.bash
source "$BIO_NAV_INTERFACES_SETUP"
source ros2_ws/install/setup.bash
cd ros2_ws
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

当前工程基线预期：

- Python/unit：649 passed，13 deselected；
- colcon：590 tests，0 errors，0 failures，4 skipped。

数字随新增测试可增加；验收依据是零失败，而不是硬编码总数。

## 生成身份绑定 profile

```bash
cd "$MODULE3_ROOT"
python3 scripts/generate_bionav_fusion_profile.py --help
```

为输出提供真实 `module3_map_sha256`、planning qualification、risk model 和 risk
qualification SHA。生成文件存入新 run 目录，不覆盖仓库模板。

## 可视化导航

终端 A 启动 Isaac；终端 B 启动 Integration bridge 与 Module2；终端 C：

```bash
cd "$MODULE3_ROOT"
./scripts/run_ros.sh navigation \
  odometry_mode:=ideal \
  spawn_pose_name:=long_route_start_g1 \
  nav2_profile:=bio_nav_tiebreak_risk \
  nav2_profile_params_file:="$RUN_DIR/nav2_bio_nav_tiebreak_risk.yaml"
```

在 RViz 发送目标，并按 [Module2 RViz 可视化](../module2_rviz_visualization.md)
检查 markers、raw grids、Global Costmap 和状态。

## 故障回退

依次验证 Module2 kill、100/500 ms 延迟、NaN、旧 goal、reset、换图、损坏
snapshot 和错误 model hash。每项都应拒绝认知输入并保持传统导航可运行。

本流程证明工程构建与故障边界。只有冻结路线 Gate 和独立 Confirmation receipt
均通过，才能发布“规划+风险融合合格”的可选 profile。
