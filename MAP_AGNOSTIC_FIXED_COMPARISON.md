# 固定炸弹位置：通用策略配对评估与版本选择

日期：2026-11-17。测试对象为当前仓库先前策略和用户指定外部目录 `E:\code\final_fusai\demo_fusai\swarm-eod-main\src\swarm_rescue\solutions` 的更新策略。两版六个有效模块中，`coverage.py`、`motion_control.py`、`my_drone_rescue.py` 不同；其余相同。两版分别备份在 `bench_results/strategy_snapshots/current_before_external_20261117.zip`、`external_updated_20261117.zip`，并有对应 SHA256 清单。当前工作区最终保留**外部更新版**。

## 测试方法

十张官方地图均用用户给的 `bench_cases_user_v6/MapXX_user_fixed.json`，每图固定 10 枚炸弹，整个配对测试没有生成随机炸弹位置。种子 101、202、303 只固定每轮出生朝向及传感器噪声。离线工具 `bench_tools/seeded_official_launcher.py` 设置测试进程的 Python/NumPy 随机种子后，调用未修改的官方 `swarm_rescue.launcher`，每次带 `--competition --headless`。同一版、同图、同种子重复 Map02 seed101，结果逐项一致。两版使用相同的地图、炸弹文件和种子；策略分支不同可能改变后续随机数消费顺序，因此这是受控的初始种子比较，不保证整段噪声轨迹逐步相同。

测试配置每图 10 机、rescue 模式；Map06 开启无通信区，Map08 开启无 GPS 区，Map09 同时开启两区。未改动官方 simulation、maps、launcher、规则与计分。只部署 `src/swarm_rescue/solutions/` 中的代码。脚本及原始日志属于离线评估文件，不进入提交 ZIP。

## 三种固定朝向的全图结果

每格为三轮合计交付数 / 30、10/10 轮次、平均分。两版各 30 轮。

| 地图 | 先前策略 | 外部更新策略 |
|---|---:|---:|
| Map01 | 30/30；3/3；83.6 | 30/30；3/3；84.8 |
| Map02 | 30/30；3/3；97.9 | 30/30；3/3；97.0 |
| Map03 | 30/30；3/3；97.0 | 30/30；3/3；97.0 |
| Map04 | 30/30；3/3；95.2 | 30/30；3/3；95.3 |
| Map05 | 30/30；3/3；97.4 | 30/30；3/3；97.1 |
| Map06 | 29/30；2/3；73.2 | 28/30；2/3；70.0 |
| Map07 | 30/30；3/3；86.7 | 30/30；3/3；86.5 |
| Map07t4 | 30/30；3/3；78.4 | 30/30；3/3；79.3 |
| Map08 | 24/30；0/3；58.8 | 24/30；0/3；60.8 |
| Map09 | 28/30；2/3；73.5 | 30/30；3/3；85.2 |
| **合计** | **291/300；25/30；84.16** | **292/300；26/30；85.29** |

先前策略最差轮 55.5 分、共坠毁 8 架；外部版最差轮 49.6 分、共坠毁 1 架。外部版总均分与全拆轮次提高，各图全拆轮次均未下降，因此按用户提出的取舍准则保留外部版。**这不意味着每项指标均改善**：Map06 外部版三轮为 10、10、8 枚，先前版为 10、10、9 枚；Map08 两版都只有 24/30，外部版第三种朝向仅 6/10。外部版的最差轮分数也更低。Map09 外部版从 2/3 全拆提升到 3/3，是主要收益。

## 一次未锁种子的固定十图对照

同一批用户固定炸弹文件，当前版此前一轮为 99/100、均分 86.55、坠毁 1 架；外部版一轮为 100/100、均分 90.41、零坠毁。对应 `bench_results/general_compatible_fixed_repeat/`、`bench_results/external_updated_fixed/`。这一对照没有锁定出生朝向和噪声，仅作为补充，不作为配对决策的主要依据。

## 算法与合规性

最终策略是地图无关的在线控制链：GPS/指南针/里程计融合，GPS 失效时以 LiDAR 扫描匹配维持位姿；LiDAR 与语义传感器构建局部占据图；已知炸弹由官方通信共享并去重认领；探索使用可达前沿，返航使用处置中心势场和局部避障；空载探索时根据近墙 LiDAR 距离限速，队友接近时避让。外部版主要增强了近墙空载限速、机间避让和携弹运动控制。其源码没有地图名分支、固定炸弹坐标、墙体几何、`true_*` 调用或跨机 Python 全局共享状态。控制器的通信来自官方 `received_messages` 元组接口。

用户提出的拥堵代价、风险预算窄门、剩余时间任务调度、定位置信度尚未作为最终代码实现。之前试验过更密的无 GPS 返航轨迹、无 GPS 分区、前沿池扩大和携弹提速；这些变体未在跨图测试中稳定改善，均未留在最终策略。尤其 Map08 固定位置仍有 0/3 全拆，不能宣称目标已达成。

## 复核与交付

- 配对原始结果：`bench_results/paired_current_easy_fixed/`、`paired_current_fixed/`、`paired_external_easy_fixed/`、`paired_external_easy_fixed_last/`、`paired_external_fixed/`。每目录含 `results.json`、逐轮原始日志、配置和 `source_sha256.json`。
- 配对命令示例：`E:\envs\swarm_rescue\python.exe bench_tools/run_seeded_fixed.py --repo . --cases bench_cases_user_v6 --output bench_results/paired_external_fixed --maps Map06 Map07 Map07t4 Map08 Map09 --seeds 101 202 303 --python E:\envs\swarm_rescue\python.exe`。
- 最终比赛代码在 `src/swarm_rescue/solutions/`，提交包为 `team006_final.zip`；ZIP 中六个有效模块与工作区逐字节一致。
- 最终源码通过 `py_compile`、官方 `scripts/check_submission.py .`，ZIP 通过官方 `scripts/check_submission.py team006_final.zip`。提交检查验证接口/结构合规，不保证成绩。

当前工作区尚未因此次比较执行 Git 提交或 GitHub 推送。
