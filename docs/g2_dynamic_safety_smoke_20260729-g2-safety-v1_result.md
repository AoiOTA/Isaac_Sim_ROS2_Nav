# G2 动态安全 smoke 结果：失败并停止

## 判定

`20260729-g2-safety-v1` 是一次预注册的单路线开发 smoke，不是 G2 正式
campaign。它在修复后的动态避障参数下完成了导航和三段 actor 交互，但没有
通过独立的安全净距验收：`local_bypass_actor` 的最小净距为 `0.0 m`，低于
预注册的 `0.10 m`。

因此判定为 **FAIL_SAFETY_SMOKE_STOP**。不得据此启动新的 G2 批量 Gate、
Confirmation 或 Module2--Module3 主动融合；不得继续在本分支调整第二组避障
参数。原始证据保留在数据目录，不提交到 Git。

## 固定输入

- Module3 源提交：`c6cf3169b65945d4a376c915f2d3ff64d4e07dcf`
- profile：`dynamic_avoidance`
- 直接加载的 profile 文件 SHA-256：
  `06dbe1a976b7f35ac94bc9e5c728164474d76741b1f15bbadd685de4786eec09`
- scenario：`kujiale_g2_dynamic_safety_smoke`
- 单一 seed：`12001`
- appearance：`bright_warm`
- 路线：`G1 -> G2 -> G3 -> G5 -> G1`
- 运行根目录：
  `data/experiment_runs/g2_dynamic_safety_smoke_20260729-g2-safety-v1`

## 可复核证据

| 工件 | SHA-256 |
| --- | --- |
| `evidence/kujiale_g2_dynamic_safety_smoke/run-0001-seed-12001/run_manifest.json` | `45e740803445d048f37bc4d8ddf8f7db45a8d7956a5e7c5d064aa72bb11b80e7` |
| `orchestrator/runner.log` | `ce1ef731f108854a6b7dc22e9d4be89be9db5015038ba4cc62781c8ac8d99310` |
| `orchestrator/nav2.log` | `fac1c8b35c0b7a41e4ba4ec896fd7a495bafbac70c7e5cc117df1cb075734a76` |

运行器的原生路线结果为 `success`，三名预期 actor 都触发并退场，且 actor
guard 没有 abort；但是这不足以满足本 smoke 更严格的近接安全条件：

| actor | 最小净距 (m) |
| --- | ---: |
| `local_bypass_actor` | 0.0000 |
| `g2_g3_exit_actor` | 0.6931 |
| `g5_g1_crossing_actor` | 0.0043 |

故 smoke 的独立后验检查以如下信息退出：

```text
G2 dynamic-safety smoke failed: local_bypass_actor clearance=0.0 is below 0.10 m
```

## 对域适配问题的结论

这次未达标**不能归因于 Module2 从 MiniWorld 到 Isaac Sim 的域差异**。该
scenario 只运行 Module3 的 Isaac、Nav2 `dynamic_avoidance` 和动态 actor；
脚本不启动、不订阅也不调用 Module2。失败发生在动态 actor 的近接避障与
碰撞/净距安全层，属于 Module3 物理安全问题。

因此当前没有进入 Module2 域适配的证据或授权条件。域适配只应在 Module2
输入/输出确实接入、并且经过 Shadow/离线对照后出现可重复的感知或 prior
质量退化时另立阶段处理。

## 本次收尾

失败后，外层 smoke 发现两个运行包装器使用独立 session，单独中断包装器会
遗留实际 Isaac/ROS 子进程。本分支仅修复关停传递：先向该 smoke 自己的直接
子进程组发 `SIGINT`，再由包装器正常回收。该修复不改变 Nav2 参数、actor、
route、验收阈值或上述实验结果。已通过 `bash -n` 和 package-contract 测试。

下一步必须是单独审查这次物理安全失败的根因和提出新的、预注册的修复方案；
在获得该方案并通过新的单路线 smoke 前，融合阶段保持停止。
