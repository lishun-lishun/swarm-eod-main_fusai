# SICT 全地图拆除与飞行安全调整

基线是当前项目此前实际完成的 224 轮。全拆 193 轮，其中 185 轮零坠毁，共交付 2144/2240 枚，坠毁 76 架次，均分 82.47。内置十图均分 90.87，Map08 为 71.73。使用相同地图、炸弹布局和种子比较候选，全部仿真继续使用官方 `--competition --headless`。 [基线报告](TEAM861_CURRENT_SOURCE_224_REPORT.md)

## 失败诊断

- Map08 固定布局种子 303 的出发段、搜索段和返航段由日志分别复盘。第 1639 至 2911 步，一架已恢复 GPS 的携弹无人机在同一区域反复触发脱困。第 4800 步仍有三架在搬运。带轨迹的诊断首轮达到墙钟上限，基线成绩以正式 224 轮记录为准。 [原始轨迹](bench_results/team861_low_health_recovery/phase_baseline/Map08_user_fixed_seed303.log)
- Map08 边角布局种子 101 在第 5100 步仍有三架携弹，第 5400 步结束时交付 7/10。前三批候选分别检查返航恢复、近距离重复抓取及低血量撤离。 [原始轨迹](bench_results/team861_low_health_recovery/phase_baseline/Map08_corner_s100_seed101.log)
- 历史改编图里的两张失效区难图占基线 76 架次坠毁中的 38 架次。其余分数差距还受官方计分影响。历史改编 14 图平均机体健康为 73.59%，平均返航健康计分比例为 11.68%。

## 候选策略

| 改动 | 触发条件 |
| --- | --- |
| 暂缓重复抓取 | 反复超时、近期没有直接观测后，本机把该位置暂时移出候选 |
| 低血量短暂撤离 | 空载、剩余血量低且再次受损时，用 LiDAR 寻找局部开口，短暂离开碰撞区域 |
| 识别可能的共同抓取 | 无 GPS 时，仅在长时间携弹、持续看见近距离队友且已触发返航恢复时，才暂时放手 |
| 绕开返航反复卡住的位置 | 重新取得 GPS 后仍在同一处反复脱困时，短期提高该路线前方局部通行代价 |
| 在视线内直线航段提高巡航速度 | 携弹、航向与速度一致、LiDAR 确认前方净空、远离处置动作且附近没有避让要求 |

全部触发条件来自自身传感器、里程计、健康和官方通信，不按地图切换。候选在隔离目录验收，固定布局和其他测试坐标仅传给评测启动器。

## 暂缓重复抓取的全图结果

24 张图各跑一个困难场景。交付从 207/240 升到 210/240，全拆且零坠毁从 12/24 升到 14/24，平均分从 67.24 升到 68.94，坠毁总数为 23。Map01 和 MapFinal_2024_25_03 各多完成一轮全拆，Map08 边角多交付 1 枚。MapFinal_2023_24_03 边角场景增加了 1 架坠毁，独立复跑和与低血量撤离合并后均复现。 [24 图配对](bench_results/team861_low_health_recovery/stale_all/paired.csv) · [独立复跑](bench_results/team861_low_health_recovery/stale_replay/paired.csv) · [两项组合](bench_results/team861_low_health_recovery/combined_pilot/paired.csv)

## 直线提速的结果

8 轮合计交付 74/80 枚，与基线相同，全拆从 4 轮增至 5 轮，平均分从 69.99 升到 71.04，坠毁从 3 架次增至 7 架次。Map07t4 一轮从 2 架坠毁降到 0，Map09 一轮从 79.9 升到 91.8。Map08 固定布局少交付 1 枚，边角布局多坠毁 1 架。两张无 GPS 历史图也增加了坠毁。 [8 轮配对](bench_results/team861_low_health_recovery/speed_pilot/paired.csv)

## 低血量撤离的验证

| 范围 | 交付 | 坠毁 | 全拆 | 全拆且零坠毁 | 均分 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 24 图困难布局 | 207 → 207 / 240 | 23 → 22 | 14 → 14 | 12 → 12 | 67.2375 → 67.2417 |
| 24 图另一套布局 | 234 → 234 / 240 | 11 → 11 | 21 → 21 | 20 → 20 | 82.2792 → 82.2792 |
| 合计 | 441 → 441 / 480 | 34 → 33 | 35 → 35 | 32 → 32 | 74.7583 → 74.7604 |

唯一成绩变化在 Map07t4 的随机布局 `random_s29`，种子 101。交付维持 10/10，坠毁从 2 架变成 1 架，官方分从 77.5 升到 77.6。48 轮都返回完整官方成绩，无墙钟上限轮次。当前项目已合入撤离逻辑，通过语法检查和官方提交检查。 [困难布局](bench_results/team861_low_health_recovery/refuge_all/paired.csv) · [另外一套布局](bench_results/team861_low_health_recovery/refuge_other/paired.csv) · [提交检查](bench_results/team861_low_health_recovery/submission_check.log)

## 共同抓取脱困复跑

单独复跑的 Map08 固定布局种子 303 在场景自然结束时交付 10/10、零坠毁、73.5 分。基线为 9/10、零坠毁、65.8 分。独立边角场景维持 7/10、零坠毁、53.2 分。首轮带轨迹记录的两个 Map08 场景先达到墙钟上限，评估采用无轨迹的复跑结果。 [初筛](bench_results/team861_low_health_recovery/handoff_pilot/results.json) · [独立复跑](bench_results/team861_low_health_recovery/handoff_replay/paired.csv)

将共同抓取脱困与低血量撤离合并后，24 图困难布局交付从 207/240 升至 209/240，全拆从 14 轮增至 16 轮，坠毁从 23 架降至 22 架，均分从 67.24 升至 68.04。MapMedium02 的一个聚集场景由 10/10、零坠毁变为 10/10、坠毁 1 架。使用相同驱动带轨迹复测，低血量撤离版为 80.0 分、零坠毁，合并版为 69.5 分、坠毁 1 架。合并版因此没有用于工作区。 [24 图配对](bench_results/team861_low_health_recovery/handoff_refuge_all/paired.csv) · [撤离版复跑](bench_results/team861_low_health_recovery/medium_trace_refuge/MapMedium02_cluster_seed303.log) · [合并版复跑](bench_results/team861_low_health_recovery/medium_trace_combined/MapMedium02_cluster_seed303.log)

另一组 8 个无 GPS 场景中，合并版交付 80/80，基线为 79/80，均为零坠毁。唯一变化再次出现在 Map08 固定布局种子 303，合并版复现 10/10、73.5 分。 [8 场景配对](bench_results/team861_low_health_recovery/handoff_refuge_other/paired.csv)

本轮共运行 152 次，包含独立复跑。146 次在官方墙钟限制内返回完整成绩。已合入的低血量撤离版有 48 个独立场景的配对证据。前述 224 轮报告对应本轮开始时的基线。
