# SICT 全地图策略迭代：传感器失效恢复

本轮以当前正式策略为基线，使用相同的地图、炸弹布局和仿真种子运行官方 `--competition --headless`。主评估有 55 个场景，覆盖 10 张内置地图和 14 张历史改编图。24 张图各跑困难布局和另一种布局，7 张设有全传感器失效区的地图再跑第三种布局。

本轮 224 轮全量回归采用已在工作区通过前一轮全地图回归的策略。入口提醒版在三个困难案例有收益，但默认配置的首次配对及两次复跑中，MapFinal_2024_25_03 的全拆率均下降，未达到替换条件。

## 失效区提醒

改动使用无人机自己的 LiDAR、里程计、语义和 GPS。三个传感器同时失效后，若无人机重新取得 GPS，且从失效前到恢复时的位移与入区方向吻合，就在入口附近给路径规划加一个临时禁行点。无人机通过官方通信发送这一观测。地图墙体和定位锚点继续由传感器数据更新，临时点只用于路径规划并按步数过期。

### 首次配对成绩

| 测试范围 | 已交付 | 坠毁架次 | 全拆且零坠毁 | 平均分 |
| --- | ---: | ---: | ---: | ---: |
| 24 张图困难布局 | 208 → 209 / 240 | 22 → 21 | 13 → 13 / 24 | 67.88 → 68.28 |
| 24 张图不同布局 | 234 → 234 / 240 | 11 → 9 | 20 → 20 / 24 | 82.28 → 82.20 |
| 7 张图第三种布局 | 57 → 57 / 70 | 7 → 7 | 4 → 4 / 7 | 60.17 → 60.11 |
| **合计** | **499 → 500 / 550** | **40 → 37** | **37 → 37 / 55** | **73.18 → 73.32** |

| 地图与场景 | 种子 | 已交付 | 坠毁架次 | 分数 |
| --- | ---: | ---: | ---: | ---: |
| MapIntermediate02 普通随机 | 101 | 9 → 10 | 1 → 0 | 54.0 → 72.7 |
| MapFinal_2024_25_02 聚集 | 101 | 0 → 1 | 5 → 4 | 0.0 → 6.0 |
| MapFinal_2025_26_02 普通随机 | 303 | 8 → 8 | 4 → 2 | 48.0 → 48.0 |
| MapFinal_2024_25_03 聚集 | 101 | 10 → 9 | 0 → 1 | 73.3 → 58.2 |
| MapIntermediate02 聚集 | 303 | 10 → 10 | 0 → 0 | 81.9 → 80.0 |
| MapMedium02 普通随机 | 303 | 10 → 10 | 0 → 0 | 76.3 → 76.1 |
| MapIntermediate02 障碍边角 | 303 | 10 → 10 | 0 → 0 | 78.2 → 78.0 |

十张内置地图的 20 轮比赛中，候选与基线的交付数、坠毁数和分数逐轮相同。55 轮首次配对均返回官方计分且均在官方墙钟上限内完成。逐轮表包含步数、健康值以及基线和候选的原始日志路径。 [逐轮表](bench_results/team861_sensor_continuity/paired_rounds.csv) · [分组汇总](bench_results/team861_sensor_continuity/paired_summary.json)

六个有分差或有交付、坠毁变化的场景，各用默认配置重跑两次。两次复跑的交付、坠毁和分数逐项一致。每组候选交付 48/60，坠毁 7 架，均分 56.83，基线为 47/60、10 架、55.58 分。默认配置的 MapFinal_2024_25_03 两次候选均为 9/10、坠毁 1 架，两次基线均为 10/10、零坠毁。 [重复配对表](bench_results/team861_sensor_continuity/paired_repeats.csv)

十张内置图均用用户提供的固定布局和种子 303 做了额外核对。两版交付 99/100，均分 87.15，坠毁均为 0 架，每张图分数一致。Map08 交付 9/10，其余九图全拆。 [固定布局成绩](bench_results/team861_sensor_continuity/fixed_303_pairs.csv)

### GPS 场景复跑

MapFinal_2024_25_03 只有 GPS 中断。一组带轨迹的配对复跑中，基线和入口提醒版都在第 4604 步交付 10/10，坠毁 0 架，得 73.3 分。采样控制量、位置和健康变化相同。另一个将消息发送推迟到首次确认后的候选得到 9/10、坠毁 1 架。三个跑次都没有确认到同时失效的跨区事件。该跑次第一次采样到控制差异是在第 3800 步，横向指令相差约 0.00001，随后轨迹分叉。 [基线轨迹](bench_results/team861_sensor_continuity/gps_trace_base/MapFinal_2024_25_03_cluster_seed101.log) · [入口提醒轨迹](bench_results/team861_sensor_continuity/gps_trace_cross/MapFinal_2024_25_03_cluster_seed101.log) · [消息变体轨迹](bench_results/team861_sensor_continuity/gps_trace_lazy/MapFinal_2024_25_03_cluster_seed101.log)

额外的单线程诊断中，同一基线在该 GPS 场景也得到 9/10、坠毁 1 架。另一个困难聚集场景从默认配置的 0/10 波动到单线程的 10/10。该诊断保留官方计分，用于观察执行环境变化的影响。 [单线程基线](bench_results/team861_sensor_continuity/single_thread_base/results.json) · [单线程候选](bench_results/team861_sensor_continuity/single_thread_cross/results.json)

## 其他改动

- **按采样间隔恢复里程计速度。** MapMedium01 有一次 42 步内累计约 94.8 像素的读数，另一处 72 步累计约 173.8 像素。速度修正保留累计位移用于定位，并使用实际间隔折算速度。连续读数对照的估计结果逐步一致。七个实测场景中合计少坠毁 4 架，但两轮各增坠毁 1 架。 [采样验证](bench_results/team861_sensor_continuity/odometry_probe.txt) · [仿真结果](bench_results/team861_sensor_continuity/odometry_pilot/results.json)
- **无语义观测时停止覆盖登记。** 隔离修正保住正常飞行控制，避免把传感器关闭时段当成已巡查。MapFinal2022_23 障碍边角组少坠毁 3 架，MapMedium01 障碍边角组少坠毁 1 架，MapFinal_2024_25_02 聚集组多坠毁 1 架。 [全图结果](bench_results/team861_sensor_continuity/coverage_all/results.json) · [进程异常复跑](bench_results/team861_sensor_continuity/coverage_replay/results.json)
- **长时间返航停滞时尝试另一条路线。** Map08 固定布局的复测由 9/10 降为 7/10。 [仿真结果](bench_results/team861_sensor_continuity/route_pilot/results.json)
- **重复碰撞后沿已飞过且经 LiDAR 确认的航段后撤。** Map01 障碍边角组由 9/10 提升到 10/10，Map07t4 随机组少坠毁 1 架。GPS 场景也出现退步。Map09 的低分来自先触发墙钟上限，单独复跑为 10/10、零坠毁、79.9 分。 [困难组](bench_results/team861_sensor_continuity/backtrack_pilot/results.json) · [异常复跑](bench_results/team861_sensor_continuity/backtrack_replay/results.json)
- **持续失效时暂停控制、持续追踪失联队友、恢复后沿开放方向驶离。** 分别出现新增坠毁、缺乏交付收益或某轮少交付的情况。 [暂停版](bench_results/team861_sensor_continuity/pause_pilot/results.json) · [失联队友版](bench_results/team861_sensor_continuity/peer_pilot/results.json) · [恢复后离开版](bench_results/team861_sensor_continuity/exit_pilot/results.json)
- **缓存 A* 相邻格代价。** 54 次路径对照验证了两类缓存失效条件，所有输出路径和代价一致。微基准中 18 次搜索的耗时中位数为 0.229 秒和 0.258 秒，缓存版未显示出加速。 [微基准](bench_results/team861_sensor_continuity/planner_cache_probe.txt)

比赛代码的候选均放在隔离目录，策略本身只使用公开传感器、里程计和通信。评测复用已经生成的地图和十枚布局，原始打分和仿真逻辑由官方运行器执行。

入口提醒候选通过语法检查、官方提交检查和 42 项相关仓库测试。当前策略也通过官方提交检查。 [候选检查](bench_results/team861_sensor_continuity/staged_checks.log) · [当前策略检查](bench_results/team861_sensor_continuity/full_224_checks.log)
