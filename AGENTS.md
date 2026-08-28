# AGENTS.md — Isaac_Sim_ROS2_Nav / Module3

通用科研工程规则由 `~/.codex/AGENTS.md` 提供；本文件只说明本仓职责。

## 职责

- 负责 Isaac Sim 场景运行、ROS 2 导航运行时、TF/odom/传感器接入及模块级启动与测试。
- 不复制 Integration 的跨仓编排，也不复制 Module2 算法实现。
- 运行时或接口变化影响联合系统时，执行模块 smoke 和跨仓 focused integration。

## 收敛规则

- 只保留一个 canonical build 入口、启动入口、reset 入口和 cleanup 路径。
- bags、experiment_runs、metrics、reports、轨迹输出和大型场景资产写入 NAS；Git 只保留必要索引与小型 fixture。
- 日期化 smoke 结果、repair plan 和逐次故障报告合并到 ledger/current state 后删除。
- 一次性 analyze/audit/generate 脚本完成任务后删除；重复使用的工具应改为稳定、职责单一的名称。
- 地图和场景配置按角色命名，不通过 attempt、日期、v1/v2 文件链保存历史。
- 诊断与 cleanup 保持薄层，不发展成复杂恢复框架或进程状态机。
