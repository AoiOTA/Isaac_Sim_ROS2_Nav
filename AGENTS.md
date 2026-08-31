# AGENTS.md — Isaac Sim ROS 2 Nav / Module3 职责增量

行动前读取 `/home/lyb/Workspace/Bio_Nav/AGENTS.md`；本文件只补充本仓 ownership 与错误语义。

## Ownership

- 拥有 Isaac 场景与传感器、里程计、定位、TF、地图、GVG/Nav2、costmap、物理合法性、collision safety、reset、控制和最终速度；不复制 Module2 算法或 Integration 跨仓编排。
- selected profile 决定全局定位变换 owner；同一 run 的 TF、控制和 collision truth owner 必须分别唯一。Reset 与 cleanup 只管理本仓在当前 run 创建并拥有的状态和资源。
- ContactSensor 或明确实际碰撞 topic 是物理接触真值；SAT、AABB 与几何 overlap 仅作诊断。

## Fail-open / No-catch delta

- 只有调用点明确列出的 optional 输入或具体 TF 暂不可用，且存在定义清楚的安全行为时，才可窄范围降级；不得把其他异常并入该路径。
- callback、action、控制或 safety 内部错误必须到达生命周期 owner；需要时先最小安全停车，再非零失败。不得 log-and-continue。
- collision、Collision Monitor stop、timeout、unreachable 与导航失败保持各自产品语义，不得改写成内部崩溃、invalid 或成功。
