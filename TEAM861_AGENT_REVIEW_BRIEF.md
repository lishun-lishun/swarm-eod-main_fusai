# SICT 861 蓝方策略独立审查说明

请在 E:\code\fusai\swarm-eod-main 审查当前 Swarm-Rescue 蓝方策略。重点复现 Map08、MapMedium01、MapMedium02 的低分场景，说明瓶颈并评估通用改动。优先级为全拆、减少坠毁、提高分数，并核对其他地图有无退化。先提交审查证据，再决定是否修改正式策略。

下文链接相对于仓库根目录。本文件可以直接发给能访问该仓库的 Agent。

## 当前策略和基线

当前工作区使用低血量撤离版。空载无人机血量很低且再次受损时，在具备队友接续作业条件的情况下短暂撤离。主控制文件 SHA-256 为 e43471809bcdfa38396c7f2f37a3338ae6859623b3c29e45cb42986e06260563，与 224 轮仿真快照一致。后面两套候选保存在各自评测目录。

- 官方 10 图沿用原配置，每图 14 轮。用户固定、一共两套普通随机、两套障碍边角和两套聚集布局，每套使用种子 101 与 303。
- 14 张历史改编图每图 6 轮。普通随机、障碍边角、局部聚集各两轮。历史图关闭毁伤区，每图两块无 GPS 区、两块无通信区。四块矩形的面积直接相加，最多占图幅的 50%。复测沿用归档的区域、炸弹布局和种子。
- 每轮 10 架无人机和 10 枚炸弹，通过官方 --competition --headless 运行。基线 224 轮均分 **85.50**，交付 **2218/2240**，**210 轮全拆、199 轮全拆且零坠毁**，合计坠毁 **24 架**。官方图均分 90.87，历史改编图均分 76.55。无墙钟超时。

## 当前策略源码

- [比赛源码目录](src/swarm_rescue/solutions/) · [比赛入口 my_drone_eval.py](src/swarm_rescue/solutions/my_drone_eval.py) · [主控制器 my_drone_rescue.py](src/swarm_rescue/solutions/my_drone_rescue.py)
- [定位 localization.py](src/swarm_rescue/solutions/localization.py) · [探索 coverage.py](src/swarm_rescue/solutions/coverage.py) · [地图与路径规划 nav_grid.py](src/swarm_rescue/solutions/nav_grid.py) · [避障与运动控制 motion_control.py](src/swarm_rescue/solutions/motion_control.py)
- [224 轮正式测试的冻结源码](bench_results/team861_half_area_224/rounds/runtime/src/swarm_rescue/solutions/) · [逐文件 SHA-256](bench_results/team861_half_area_224/rounds/source_sha256.json)

## 三张重点地图的症状和失败日志

| 地图 | 均分 | 交付 | 全拆 | 坠毁 | 平均交付分 | 平均返航健康分 | 平均时间分 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Map08 | 71.73 | 132/140 | 9/14 | 9 | 56.57/60 | 12.06/20 | 3.09/20 |
| MapMedium01 | 70.42 | 59/60 | 5/6 | 3 | 59.00/60 | 1.91/20 | 9.50/20 |
| MapMedium02 | 59.88 | 54/60 | 2/6 | 4 | 54.00/60 | 0.47/20 | 5.41/20 |

这五份日志为开启诊断轨迹的基线复跑，交付、坠毁与分数均复现了正式成绩。

### Map08

- 边角布局种子 101 最终 **7/10、零坠毁、53.2 分**。第 5400 步仍有三架机携弹，问题集中在拾取之后。检查返航重规划是否改变实际航迹，是否反复经过同一区域，局部避障和朝向调整是否压制了实际前进。 [边角失败日志](bench_results/team861_half_area_opt/baseline_trace/Map08_corner_s100_seed101.log)
- 用户固定布局种子 303 最终 **9/10、零坠毁、65.8 分**。一架携弹机重新取得 GPS 后仍在同一区域多次触发脱困。检查位姿恢复后航路是否过时，脱困是否反复选回同一入口。 [固定失败日志](bench_results/team861_half_area_opt/baseline_trace/Map08_user_fixed_seed303.log)

### MapMedium01

- 聚集布局种子 303 最终 **9/10、坠毁 2 架、56.1 分**。请核查最后一枚的任务链，以及两架机坠毁前的控制状态。该图平均机体健康为 **69.47%**，官方返航健康仅贡献 **1.91/20 分**。有独立返航区的历史图，终局返航区外的机体健康不会计入该项。请检查哪些无人机有机会安全返航并且不耽误剩余任务。 [聚集失败日志](bench_results/team861_half_area_opt/baseline_trace/MapMedium01_cluster_seed303.log)

### MapMedium02

- 边角布局种子 101 最终 **9/10、零坠毁、54.0 分**。有一架携弹机在同一带反复脱困，到终局仍未交付。请区分路线确实不可走、规划航点有误、替代路线未被执行这几种情况。 [边角失败日志](bench_results/team861_half_area_opt/baseline_trace/MapMedium02_corner_seed101.log)
- 随机布局种子 303 最终 **8/10、坠毁 3 架、48.0 分**。第 4800 步仍有两架机携弹，另有三架空载机正在接近同一个未被各自直接观测的目标记录。它可能已过期或受定位漂移影响，仍需用消息时间戳和局部语义观测核实。重点查无 GPS、无通信重叠时的任务状态更新。 [随机失败日志](bench_results/team861_half_area_opt/baseline_trace/MapMedium02_random_seed303.log)

## 两套候选的全量结果

| 版本 | 224 轮均分 | 交付 | 全拆 | 全拆且零坠毁 | 坠毁 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 当前正式基线 | 85.50 | 2218/2240 | 210 | 199 | 24 |
| 重抓与停靠优化 | 85.72 | 2220/2240 | 210 | 198 | 19 |
| 覆盖消息过滤 | 85.13 | 2204/2240 | 206 | 198 | 15 |

- **重抓与停靠优化**在无 GPS、反复抓取超时且近期未直接看到目标时暂缓处理该目标。携弹且多次停靠失败时，间歇复用抓取前的语义方位调整朝向。MapMedium01 达到 73.67、6/6 全拆、坠毁 1 架；MapMedium02 达到 64.25、3/6 全拆、坠毁 3 架。Map08 少坠毁 5 架但少一轮全拆，MapFinal2022_23 新增 2 架坠毁，部分其他地图也掉分。 [逐图报告](TEAM861_HALF_AREA_OPT_224_REPORT.md) · [逐轮数据](bench_results/team861_half_area_opt/full/verified_rounds.csv)
- **覆盖消息过滤**在本机和发信队友都没有可信 GPS 定位时，暂缓合并覆盖栅格。MapMedium01 达到 73.85、6/6 全拆、零坠毁，但 Map08 只剩 5/14 轮全拆，一个固定布局从 10/10 降到 5/10。MapFinal_2025_26_03 均分由 75.02 降至 63.40，新增 5 架坠毁。 [逐图报告](TEAM861_HALF_AREA_COVERAGE_224_REPORT.md) · [逐轮数据](bench_results/team861_half_area_opt/coverage_full/verified_rounds.csv)

## 224 轮成绩和复现入口

- [正式基线 224 轮逐轮成绩 CSV，含原始日志路径](bench_results/team861_half_area_224/rounds/verified_rounds.csv) · [完整结果 JSON](bench_results/team861_half_area_224/rounds/results.json) · [24 图汇总报告](TEAM861_HALF_AREA_224_REPORT.md)
- [正式基线原始日志目录](bench_results/team861_half_area_224/rounds/) · [历史图区域坐标与尺寸](bench_results/team861_half_area_224/zones.json) · [历史图面积核验](bench_results/team861_half_area_224/coverage.json)
- [诊断报告](TEAM861_HALF_AREA_OPT_DIAGNOSIS.md) · [评测脚本](bench_tools/run_survival_benchmark.py)

从仓库根目录运行 PowerShell。输出目录需使用新路径。

```powershell
& 'E:\envs\swarm_rescue\python.exe' bench_tools/run_survival_benchmark.py --output bench_results/team861_independent_review --legacy-zone-plan bench_results/team861_half_area_224/zones.json --include-stock --workers 2 --only Map08_corner_s100_seed101 Map08_user_fixed_seed303 MapMedium01_cluster_seed303 MapMedium02_corner_seed101 MapMedium02_random_seed303
```

移除 --only 和后面的场景名单即可跑完整 224 轮。加入 --trace 可记录策略轨迹。若带轨迹触发墙钟上限，用普通模式单独复跑。按地图、布局和种子匹配正式基线 CSV。驱动打印的 baseline_* 字段引用较早的归档，历史图尤其可能不同。

## 请审查并交付

1. 给出有文件行号、对应日志事件和可证伪对照的发现。逐枚区分未发现、发现但未抓取、已抓取未送达、已交付。区分机体健康与官方返航健康。
2. 优先检查 _process_messages、_merge_bomb_observation、_pick_unclaimed_bomb、_recover_stalled_return，以及 control 中的携弹与停靠分支。对照症状出现前后的任务状态、GPS、通信、航点与血量。
3. 如果提出修改，比赛改动限于 src/swarm_rescue/solutions/。仅使用官方允许的本机状态、传感器、里程计和通信。禁止地图 ID 分支、预置墙体与炸弹坐标、true_*、引擎内部对象或跨机 Python 全局状态。评测配置和坐标仅供评测使用。保留官方地图、仿真、启动器、规则和计分。
4. 候选先复现失败局，随后对照三张图的其他布局及稳定地图，最后用相同配置跑完整 224 轮。逐图列出均分、交付、全拆、全拆且零坠毁、坠毁，点名退化场景。保留原始日志，并运行 python scripts/check_submission.py .。
