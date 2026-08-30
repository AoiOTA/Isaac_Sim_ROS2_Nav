# AGENTS.md — Isaac Sim ROS 2 Nav / Module3 职责增量

本仓属于 Bio_Nav 多仓工作区。从本仓启动时，行动前必须读取并遵守 `/home/lyb/Workspace/Bio_Nav/AGENTS.md`；本文件只增加 Module3 本仓职责。发生冲突时，更具体的本仓职责可以收紧约束，但不得跳过工作区根文件规定的项目范围、运行和证据边界。通用科研与 Multi-Agent 规则仍由 `~/.codex/AGENTS.md` 提供。

## 本仓所有权

- 负责 Isaac 场景与传感器、里程计、定位、TF、地图、GVG/Nav2、costmap、物理合法性、collision safety、reset、控制和最终 `/cmd_vel`。
- 不复制 Module2 算法，也不承担 Integration 的跨仓编排或状态数据库职责。
- Module1 独立 odom 不取得 canonical TF 所有权；场景 profile 决定 AMCL 或固定 `map -> odom` owner，同一 run 中 TF 与控制 owner 必须明确且唯一。
- ContactSensor `/simulation/collision` 是物理碰撞真值；SAT、AABB 和几何 overlap 只用于诊断。Collision Monitor stop、导航失败和物理接触分别报告。
- reset 和 cleanup 只管理本仓在当前 run 创建及拥有的状态和资源。

## 本仓事实来源

- `docs/RUNBOOK.md` 维护当前可执行环境、启动、reset、cleanup 和实验命令。
- 本仓 `docs/CURRENT_STATE.md` 只记录模块实现边界和模块级证据，并链接 Integration 的跨仓状态。
- 大型场景、地图、bag、图像、视频、完整日志和 run 输出写入 `/mnt/nas_home/Bio_Nav_Data`。
