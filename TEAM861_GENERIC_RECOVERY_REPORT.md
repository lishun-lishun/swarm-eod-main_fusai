# SICT 全地图通用策略调整

碰撞与目标状态调整均从当前正式策略派生。控制器仅使用本机传感器、健康值、实时定位估计与官方通信。比赛源码以外的改动均用于隔离评测。

第一组使用既有 24 张地图上各一组困难布局。五个候选与同驱动基线逐图复用完全相同的布局及仿真种子，采用官方 `--competition --headless` 与原计分。

| 版本 | 交付 | 坠毁 | 全拆且零坠毁 | 平均分 | 交付或坠毁退化图数 | 实际时间上限轮次 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 当前策略 | 207/240 | 23 | 11/24 | 65.83 | 0 | 0 |
| 重复碰撞后寻找有净空的局部出口 | 205/240 | 22 | 10/24 | 64.89 | 3 | 0 |
| 反复确认掉落目标与消息时间戳 | 209/240 | 28 | 12/24 | 68.15 | 3 | 0 |
| 重复碰撞后退回已飞过且经 LiDAR 复核的位置 | 208/240 | 24 | 11/24 | 66.44 | 2 | 0 |
| 定位坐标可信时才用观测修正共享任务 | 207/240 | 23 | 12/24 | 66.92 | 1 | 0 |
| 确认掉落之前保留原有目标追踪 | 208/240 | 22 | 13/24 | 67.88 | 0 | 0 |

各版本数字依次为交付数 / 坠毁数。

| 地图 | 布局 | 种子 | 当前 | 局部出口 | 观察确认 | 已飞航迹 | 定位可信时确认 | 确认前持续追踪 |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Map01 | corner_s105 | 101 | 9/0 | 9/0 | 9/0 | 10/0 | 9/0 | 9/0 |
| Map02 | corner_s100 | 303 | 10/0 | 10/0 | 10/0 | 10/0 | 10/0 | 10/0 |
| Map03 | corner_s100 | 101 | 10/0 | 10/0 | 10/0 | 10/0 | 10/0 | 10/0 |
| Map04 | cluster_s618 | 303 | 10/0 | 10/0 | 10/0 | 10/0 | 10/0 | 10/0 |
| Map05 | cluster_s618 | 303 | 10/0 | 10/0 | 10/0 | 10/0 | 10/0 | 10/0 |
| Map06 | user_fixed | 101 | 10/0 | 10/0 | 10/0 | 10/0 | 10/0 | 10/0 |
| Map07 | cluster_s618 | 303 | 9/0 | 9/0 | 10/0 | 9/0 | 10/0 | 10/0 |
| Map07t4 | random_s29 | 101 | 10/2 | 10/1 | 10/2 | 10/1 | 10/2 | 10/2 |
| Map08 | corner_s100 | 101 | 7/0 | 7/0 | 7/0 | 7/0 | 7/0 | 7/0 |
| Map09 | random_s29 | 303 | 10/0 | 10/0 | 10/0 | 10/0 | 10/0 | 10/0 |
| MapFinal2022_23 | corner | 101 | 9/4 | 9/4 | 9/4 | 9/4 | 9/4 | 9/4 |
| MapFinal_2023_24_01 | corner | 101 | 10/0 | 10/0 | 10/0 | 10/0 | 10/0 | 10/0 |
| MapFinal_2023_24_02 | corner | 101 | 3/2 | 3/2 | 3/2 | 3/2 | 3/2 | 3/2 |
| MapFinal_2023_24_03 | corner | 101 | 9/2 | 8/2 | 10/3 | 10/3 | 9/2 | 9/2 |
| MapFinal_2024_25_01 | cluster | 101 | 10/0 | 10/0 | 10/0 | 10/0 | 10/0 | 10/0 |
| MapFinal_2024_25_02 | cluster | 101 | 0/5 | 0/5 | 0/5 | 0/5 | 0/5 | 0/5 |
| MapFinal_2024_25_03 | cluster | 101 | 10/0 | 9/0 | 9/4 | 9/1 | 9/1 | 10/0 |
| MapFinal_2025_26_01 | cluster | 303 | 10/0 | 10/0 | 10/0 | 10/0 | 10/0 | 10/0 |
| MapFinal_2025_26_02 | cluster | 101 | 3/4 | 3/4 | 3/4 | 3/4 | 3/4 | 3/4 |
| MapFinal_2025_26_03 | cluster | 101 | 9/0 | 9/0 | 10/1 | 9/0 | 9/0 | 9/0 |
| MapIntermediate02 | random | 101 | 9/1 | 9/1 | 9/1 | 9/1 | 9/1 | 9/1 |
| MapMedium01 | corner | 101 | 10/2 | 10/1 | 10/2 | 10/2 | 10/2 | 10/2 |
| MapMedium02 | cluster | 303 | 10/1 | 10/2 | 10/0 | 10/1 | 10/0 | 10/0 |
| MapRandom | cluster | 101 | 10/0 | 10/0 | 10/0 | 10/0 | 10/0 | 10/0 |

## 选择与复测

- 在 MapFinal_2024_25_03 单独用同一诊断驱动复测，当前策略交付 10/10、坠毁 0 架，定位可信版交付 9/10、坠毁 1 架，记录到 0 次最终状态纠正。代码复查发现，等待确认时的提前返回也改变了正常追踪，第五版保留了这段更新。
纳入诊断复跑后，进入第二组的候选为「确认掉落之前保留原有目标追踪」。
- 第二组 当前。交付 234/240，坠毁 11 架，全拆且零坠毁 20/24，均分 82.13。
- 第二组 候选。交付 234/240，坠毁 11 架，全拆且零坠毁 20/24，均分 82.28。
- 第二组退化 0 张图，改善 0 张图，实际时间上限 0 轮。
- 已将通过两组配对验证的 `my_drone_rescue.py` 更新到工作区，并通过官方提交检查。现有提交 ZIP 对应上一版。

| 地图 | 不同布局 | 种子 | 当前交付 | 候选交付 | 当前坠毁 | 候选坠毁 | 当前分数 | 候选分数 |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Map01 | user_fixed | 303 | 10/10 | 10/10 | 0 | 0 | 85.8 | 85.8 |
| Map02 | random_s29 | 303 | 10/10 | 10/10 | 0 | 0 | 98.0 | 98.0 |
| Map03 | corner_s105 | 303 | 10/10 | 10/10 | 0 | 0 | 97.4 | 97.4 |
| Map04 | cluster_s919 | 101 | 10/10 | 10/10 | 0 | 0 | 98.6 | 98.9 |
| Map05 | user_fixed | 303 | 10/10 | 10/10 | 0 | 0 | 98.5 | 98.5 |
| Map06 | random_s11 | 101 | 10/10 | 10/10 | 0 | 0 | 90.1 | 90.1 |
| Map07 | corner_s105 | 101 | 10/10 | 10/10 | 0 | 0 | 89.7 | 89.7 |
| Map07t4 | cluster_s919 | 303 | 10/10 | 10/10 | 0 | 0 | 87.8 | 83.3 |
| Map08 | user_fixed | 303 | 9/10 | 9/10 | 0 | 0 | 65.8 | 65.8 |
| Map09 | random_s11 | 101 | 10/10 | 10/10 | 0 | 0 | 94.6 | 94.6 |
| MapFinal2022_23 | cluster | 101 | 10/10 | 10/10 | 0 | 0 | 77.8 | 83.8 |
| MapFinal_2023_24_01 | cluster | 101 | 10/10 | 10/10 | 0 | 0 | 87.3 | 88.6 |
| MapFinal_2023_24_02 | random | 101 | 10/10 | 10/10 | 2 | 2 | 80.2 | 80.2 |
| MapFinal_2023_24_03 | cluster | 303 | 10/10 | 10/10 | 0 | 0 | 81.5 | 81.9 |
| MapFinal_2024_25_01 | random | 101 | 10/10 | 10/10 | 0 | 0 | 81.9 | 81.9 |
| MapFinal_2024_25_02 | random | 101 | 7/10 | 7/10 | 5 | 5 | 42.0 | 42.0 |
| MapFinal_2024_25_03 | corner | 101 | 10/10 | 10/10 | 0 | 0 | 79.0 | 79.0 |
| MapFinal_2025_26_01 | random | 303 | 10/10 | 10/10 | 0 | 0 | 76.7 | 76.7 |
| MapFinal_2025_26_02 | random | 303 | 8/10 | 8/10 | 4 | 4 | 48.0 | 48.0 |
| MapFinal_2025_26_03 | corner | 101 | 10/10 | 10/10 | 0 | 0 | 79.1 | 79.1 |
| MapIntermediate02 | cluster | 303 | 10/10 | 10/10 | 0 | 0 | 81.9 | 81.9 |
| MapMedium01 | random | 303 | 10/10 | 10/10 | 0 | 0 | 73.3 | 73.3 |
| MapMedium02 | corner | 101 | 10/10 | 10/10 | 0 | 0 | 79.0 | 79.0 |
| MapRandom | random | 303 | 10/10 | 10/10 | 0 | 0 | 97.2 | 97.2 |

## 逐轮结果

- [配对基线](bench_results/team861_survival/all_maps_baseline/results.json)
- [重复碰撞后寻找有净空的局部出口](bench_results/team861_generic_recovery/contact_all_maps/results.json)
- [反复确认掉落目标与消息时间戳](bench_results/team861_generic_recovery/observations_all_maps/results.json)
- [重复碰撞后退回已飞过且经 LiDAR 复核的位置](bench_results/team861_generic_recovery/backtrack_all_maps/results.json)
- [定位坐标可信时才用观测修正共享任务](bench_results/team861_generic_recovery/consistent_pose_all_maps/results.json)
- [确认掉落之前保留原有目标追踪](bench_results/team861_generic_recovery/confirmed_all_maps/results.json)
- [GPS 场景当前策略复跑](bench_results/team861_generic_recovery/gps_replay_control/results.json)
- [GPS 场景候选复跑](bench_results/team861_generic_recovery/gps_replay_candidate/results.json)
- [获益场景任务状态诊断](bench_results/team861_generic_recovery/observation_gain_replay/results.json)
- [第二组当前策略](bench_results/team861_generic_recovery/distinct_baseline/results.json)
- [第二组候选](bench_results/team861_generic_recovery/distinct_candidate/results.json)
- [官方提交检查日志](bench_results/team861_generic_recovery/submission_checks.log)
