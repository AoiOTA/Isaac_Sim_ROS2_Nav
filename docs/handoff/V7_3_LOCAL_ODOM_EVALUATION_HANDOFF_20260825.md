# V7.3 local odom 评估停止 handoff（2026-08-25）

## 1. 停止指令与结论

用户已明确要求在 KISS-ICP 最后一轮有界评估后停止：本 handoff 与配套算法适配报告、ledger 索引完成并同步远端后，**不再继续开发、调参、replay、Isaac/ROS live、导航或 qualification**。

最终决策是：

- 当前没有任何 V7.3 local-odom candidate 获得 promotion；
- canonical `/odom` 与 `odom -> base_link` 仍由现有 `wheel + corrected IMU + EKF` baseline 提供；
- stereo+IMU cuVSLAM、stereo-only VO、FAST-LIO2、RF2O、GICP、NDT 和 KISS-ICP 均不得接管 canonical odometry 或 TF；
- Phase 1D direct cutover 未授权，Phase 2 及后续 global/cognitive/formal 工作均未进入；
- 本停止边界不是 V7.3 FINAL 总计划完成，也不是 formal qualification。

评估细节见 [V7_3_ODOM_LOCALIZATION_ALGORITHM_ADAPTATION_REPORT_20260825.md](V7_3_ODOM_LOCALIZATION_ALGORITHM_ADAPTATION_REPORT_20260825.md)。

## 2. 仓库、分支与提交边界

2026-08-25 写文档前复核如下：

| 仓库 | 允许 worktree / branch | 当前 HEAD | 固定 `refs/heads/main` |
|---|---|---|---|
| Integration | `/home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_intergration` / `cognitive-navigation` | `d0f7fab7a9126f456377e096359c3854181bbab1` | `f23a7eccc542e602ec641daf7a20b14c2371dca9` |
| Module3 | `/home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3` / `cognitive-navigation` | `f94114b9f6cdaca89756b8a9f9e906891baa6136` | `22d66470c4b903349b2467dc876490bbebfc0083` |
| Module2 | `/home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/MODULE2_SRDR_V310_MODULE3_HANDOFF_20260812` / `cognitive-navigation` | `98b3ffb4526a55acd318cebdf1462de82939ec05` | `c8297a590ba61bcf712ad4a339437fb2c44a027e` |

三个固定 main ref 均与项目硬边界一致。Module3 写文档前 `origin/cognitive-navigation` 也精确为 `f94114b9f6cdaca89756b8a9f9e906891baa6136`。三个 worktree 中既有未跟踪 build/install/log 目录均保留，未 reset、clean 或回退。

本次只新增本 handoff、配套算法报告并追加 `EXPERIMENT_LEDGER.md`；没有修改源码、配置、launch、tests 或 runtime artifact。

## 3. 当前 Module3 代码状态

与本轮 local-odom 工作直接相关的主要提交如下：

| Commit | 状态 |
|---|---|
| `b0893f1` | opt-in `stereo_vio` 双目 producer |
| `3e1d63e` | Fast DDS UDP receive buffer 4 MiB |
| `f90ff0a`、`e1787c3` | 120 Hz VIO IMU 与最终 physics-cadence 收敛 |
| `6b950b3`、`92563d7` | stereo+IMU cuVSLAM shadow 与单腿 pilot runner |
| `9fac3b8`、`8ac12db` | terminal-zero settling 与 production `Twist` 注册修复 |
| `9f959fb`、`1117bd6` | GICP shadow 后替换为 NDT；GICP product 已移除 |
| `e3e6290`、`4402681` | OS1-128 profile 与 adapter `max_ring=31/127` |
| `d16d2a8` | offline-vendored official KISS-ICP v1.3.0 shadow |
| `f94114b` | Isaac StructureTF 统一拥有 LiDAR `Rz(+90 deg)` |

当前 product surface：

- wheel odometry、corrected IMU 与 EKF baseline 保持原有 canonical `/odom`；
- cuVSLAM stereo+IMU shadow 仍是 opt-in、TF false；
- KISS-ICP vendor/config/launch 仍是 default OFF，输出仅 `/local_odom/kiss_shadow`，不发布 odometry TF；
- PCL GICP/NDT product surface 已移除；
- OS1-32/128 producer 与 adapter 保留，OS1-32 adapter 默认 `max_ring=31`，OS1-128 必须显式 `max_ring=127`；
- Isaac StructureTF 的 `base_link -> lio_lidar_link` 为 `[0.120, 0, 0.333] m` 与 `Rz(+90 deg)`；物理 OS1 prim 姿态仍为 identity；
- rejected/default-OFF FAST-LIO2 配置仍含内部 `Rz(+90 deg)`，因此不得与当前 StructureTF 同时启用，以免重复旋转；本任务未修改该历史工具。

## 4. Phase 状态

| Phase | 最终状态 | 证据边界 |
|---|---|---|
| Phase 0 | 完成 | canonical V7.3 plan 已在 Integration，三仓 branch/HEAD/main 已复核 |
| Phase 1A stereo producer | **ENGINEERING LIVE PASS** | 五路 506 stamps 完全配对、20 Hz、RTF 0.84587；非 formal |
| Phase 1B VIO IMU | **ENGINEERING PASS WITH WARNINGS** | stereo 20 Hz、IMU/joint/wheel 120 Hz、EKF 50 Hz；RTF 0.64053；非 formal |
| Phase 1C cuVSLAM | **DEFERRED / NOT PROMOTED** | route VIO endpoint/max `0.393677/0.445978 m`，差于 EKF `0.214637 m`；paired diagnostic 不提供 promotion |
| Phase 1C terminal-zero | **ENGINEERING PASS ONLY** | exact-UUID cancel 后 `/cmd_vel_sim` stable zero +450.854 ms，post-zero GT 2.278 mm；非 collision closure/formal |
| Alt-1 GICP | **ENGINEERING STOP** | delivery/load pass，几何快速发散；planar projection 仍 STOP；product 已移除 |
| Alt-1 NDT32 | **ENGINEERING STOP** | 0.5/1/5 m crossing `2.00/4.10/19.40 s`；product 已移除 |
| Alt-1 NDT128 | **ENGINEERING STOP** | GT endpoint `2.060 m`、direction `-102.60 deg`、scale `1.866` |
| Alt-1 KISS identity | **ENGINEERING STOP** | 0.5/1/5 m crossing `1.643/2.816/10.275 s`、direction `97.52 deg` |
| Alt-1 KISS common-axis replay | **BOUNDED REPLAY PASS ONLY** | 允许且仅用于最后一次 OS1-128 engineering live；非 promotion |
| Alt-1 KISS OS1-128 live | **SMOKE COMPLETE / USABILITY STOP** | 方向灾难消失，但 scale `3.797`、z/pitch/noisy path 不可用 |
| Phase 1D / Phase 2 | **NOT AUTHORIZED / NOT ENTERED** | 无 local odom 达到 cutover 条件 |
| Formal qualification | **NOT RUN** | V7.3 为 `0` formal runs；不得把 static/replay/live 当 formal |

## 5. 最终 KISS-ICP 决策

最后一轮唯一授权 live 根目录：

`/mnt/nas_home/Bio_Nav_Data/experiments/runs/v73_kiss_os1_128_live_20260825T024523Z`

16 s fixed-motion profile 的 raw/adapted/KISS 均为 `160` 个严格单调样本，adapter-to-KISS stamp delivery 为 `160/160`；RTF `0.648`。KISS 对 GT 的 endpoint/max/p95 XY 为 `0.469/0.494/0.486 m`，方向误差 `-12.633 deg`，但路径为 `5.270 m`，GT 仅 `1.388 m`，scale `3.797`。z endpoint/max/p95 为 `0.152/0.599/0.563 m`，pitch max/p95 为 `0.4741/0.4654 rad`，显示出显著非平面抖动和 wandering。

同窗对照明显更好：wheel endpoint/max/p95 `0.0129/0.0254/0.0240 m`、scale `0.9951`；EKF `0.0283/0.0334/0.0307 m`、scale `0.9986`。因此“未越过 0.5 m”不能抵消 KISS 的路径尺度与姿态失败；KISS 保持 default OFF 且不得进入 G1/G2、canonical `/odom`、TF 或 Phase 1D。

## 6. Baseline 与回滚

当前唯一可继续保留的运行 baseline 是 `estimated_wheel_imu`：

```text
/joint_states -> /wheel/odom
/imu/data_raw -> calibrated /imu/data
/wheel/odom + /imu/data -> EKF /odom
EKF owns odom -> base_link
Grid remains global startup/reset localization
```

在 `stereo_vio` profile 下，wheel/IMU input 实测 120 Hz，EKF `/odom` 约 50 Hz。它在最后短运动窗中远优于 KISS，并是明确 rollback/control baseline；但它仍受 skid/scrub 转弯平移误差影响，不能由此次短窗结果改写成最终 local-odom 算法或 formal-qualified 状态。

回滚不需要删除 KISS vendor 或 common-axis 配置：保持所有候选 default OFF、不要启用其 canonical consumer 即可。不要同时启用 FAST-LIO 内部 `+90 deg` 与 StructureTF `+90 deg`。

## 7. Runtime cleanup

- 所有本轮授权 run-owned Isaac/ROS/replay/recorder/adapter/KISS process groups 均已在各自 run 结束时清理；
- 最终 KISS live 后 domain `208` 两次确认为 empty，Isaac lock free；
- common-axis replay 后 domain `231` empty；
- terminal-zero live 后 domain `224` empty；
- 受保护的 domain-141 PID `3600069` 在所有相关结论中均明确保持 alive，本 docs-only 任务未触碰该进程；
- 本 docs-only 任务未启动任何 ROS、Isaac、bag replay 或 navigation process。

## 8. NAS 权威索引

| 主题 | Root / 主要索引 |
|---|---|
| VIO route | `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v73_phase1c_g1_g2_20260824T135835Z/{conclusion.md,review/phase1c_g1_g2_metrics.json}` |
| VIO paired mode1/mode0 | `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v73_phase1c_paired_motion_mapping_start_20260824T202321Z/{conclusion.md,paired_summary.json}` |
| terminal-zero | `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v73_phase1c_terminal_zero_cancel_20260824T181235Z/{conclusion.md,review/metrics.json}` |
| GICP | `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v73_alt1_gicp_replay_20260824T211335Z/{conclusion.md,review/metrics.json,planar_projection_conclusion.md}` |
| NDT32 | `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v73_alt1_ndt_replay_20260824T220508Z/{conclusion.md,review/metrics.json}` |
| NDT128 | `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v73_alt1_ndt_os1_128_capture_first_20260825T002726Z/metrics/{analysis.json,capture_summary.json}` |
| KISS identity | `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v73_kiss_icp_replay_20260825T010237Z/{conclusion.md,axis_sensitivity_conclusion.md,metrics/metrics.json}` |
| KISS common-axis replay | `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v73_kiss_icp_common_axis_replay_20260825T015934Z/{conclusion.md,metrics/metrics.json}` |
| KISS OS1-128 live | `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v73_kiss_os1_128_live_20260825T024523Z/{conclusion.md,metrics/analysis.json}` |

这些根目录混合包含 logs、bags、figures 与 metrics；本 docs-only 收口只复核 handoff、conclusion 和 metrics，没有读取 bag payload 或图片。

## 9. 未完成项与未来接手边界

未完成且不应被误写为已完成的项目包括：可靠的独立 local odom、Phase 1D direct cutover、VIO 条件下 AMCL re-evaluation、continuous global localization、Phase 2 Module1/2 causal work、CognitivePlaceGraph PRIMARY、室内外 pilots 与正式 120 轮。

本任务不选择也不推荐继续任何一个实验方向。未来若用户重新授权，接手者只能重新选择以下决策之一：继续冻结 wheel/EKF baseline；重新定义新的独立 local-odom 研究假设；或修订 V7.3 总计划与完成定义。任何选择都必须作为新任务、新授权和新证据边界处理，不能把本 handoff 视为自动续跑许可。
