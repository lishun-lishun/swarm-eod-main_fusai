# 当前策略：一组固定位置与两组随机位置的官方仿真

日期：2026-11-17（Asia/Singapore）。被测代码为当前工作区 `src/swarm_rescue/solutions/` 中的 `MyDroneRescue` 及其五个依赖模块，测试前后 SHA256 一致，见 `bench_results/current_strategy_source_hashes.json`。每张地图 10 架无人机、10 枚炸弹；Map06 保留无通信区，Map08 保留无 GPS 区，Map09 同时保留两种特殊区。全部通过官方 launcher 的 `--competition --headless` 运行；官方引擎、地图、规则和评分代码未修改。

固定组为 `bench_cases_user_v6/MapXX_user_fixed.json`，其坐标取自用户提供的 V6 完整包中的 `FIXED` 表。随机组为新生成的 `bench_cases_random_triplets/MapXX_random_s11.json` 和 `MapXX_random_s29.json`：离线生成器用公开几何筛除墙体、出生区、处置区及与出生点不连通的点；每组随机选择 10 个相互分离的点。每张图的两组共 20 个坐标互不相同，也不同于该目录预先生成的第三组 `random_s53`。炸弹文件只供 launcher 设定测试案例，策略不读取它们。本次仅运行固定组和两组随机，共 **30 轮**，每个案例一次。

## 汇总

| 组别 | 轮次 | 处置 | 10/10 轮次 | ≥92 分轮次 | 平均分 | 平均健康 | 坠毁总数 |
|---|---:|---:|---:|---:|---:|---:|---:|
| 用户固定位置 | 10 | 97/100 | 9/10 | 4/10 | 86.63 | 76.80% | 1 |
| 随机种子 11 | 10 | 98/100 | 8/10 | 7/10 | 88.85 | 80.08% | 2 |
| 随机种子 29 | 10 | 100/100 | 10/10 | 5/10 | 90.54 | 80.68% | 0 |
| 合计 | 30 | **295/300** | **27/30** | **16/30** | **88.67** | **79.19%** | **3** |

全部 30 轮退出码为 0，没有外部 300 秒超时，原始日志没有 `Traceback`。随机种子用于生成**不同炸弹布局**，不是固定仿真中的无人机初始随机状态；每布局仅一轮，因此此表不是同布局重复试验的方差或稳定成功率。

## 逐地图处置与得分

每格为“处置数/10；官方分数”。

| 地图 | 固定位置 | 随机 11 | 随机 29 |
|---|---:|---:|---:|
| Map01 | 10；85.2 | 10；85.1 | 10；87.1 |
| Map02 | 10；97.1 | 10；95.8 | 10；98.7 |
| Map03 | 10；97.1 | 10；98.4 | 10；98.0 |
| Map04 | 10；91.5 | 10；99.0 | 10；97.6 |
| Map05 | 10；98.4 | 10；98.6 | 10；98.6 |
| Map06 | 10；89.7 | 10；94.5 | 10；98.2 |
| Map07 | 10；86.0 | 10；92.0 | 10；90.1 |
| Map07t4 | 10；77.6 | **9；65.0** | 10；85.4 |
| Map08 | **7；51.0** | **9；64.2** | 10；69.7 |
| Map09 | 10；92.7 | 10；95.9 | 10；82.0 |

## 未全清案例

- **Map08 固定组**：7/10、51.0 分；5400/5400 步、官方墙钟 151 秒、健康 44.8%、坠毁 1 架。
- **Map07t4 随机 11**：9/10、65.0 分；5400/5400 步、官方墙钟 116 秒、健康 54.8%、零坠毁。
- **Map08 随机 11**：9/10、64.2 分；5400/5400 步、官方墙钟 123 秒、健康 50.8%、坠毁 2 架。

Map08 随机 29 虽达到 10/10，仍耗时 5166/5400 步、只得 69.7 分，说明无 GPS 场景的完成速度与健康仍不稳。Map07t4 固定组虽 10/10，也耗时 4409 步。由于日志主要是官方回合汇总、缺乏逐机事件轨迹，不能从本轮证据准确判定残余炸弹是未发现、未抓取、运输受阻还是中心交付失败。

## 可复核文件与命令

- 固定组：`bench_results/current_strategy_user_fixed/results.json`、同目录逐轮 `MapXX_user_fixed_r1.log` 和 `config_MapXX.yml`。
- 随机组：`bench_results/current_strategy_random_two/results.json`、同目录逐轮 `MapXX_random_s11_r1.log`、`MapXX_random_s29_r1.log` 和配置。
- 输入案例：`bench_cases_user_v6/` 与 `bench_cases_random_triplets/`；各自 `manifest.json` 保存案例路径，随机组还保存种子与输入 SHA256。

```powershell
$py='E:\envs\swarm_rescue\python.exe'
& $py bench_tools\run_benchmarks.py --repo . --cases bench_cases_user_v6 --output bench_results\current_strategy_user_fixed --maps Map01 Map02 Map03 Map04 Map05 Map06 Map07 Map07t4 Map08 Map09 --kinds user_fixed --repeats 1 --timeout 300 --python $py
& $py bench_tools\run_benchmarks.py --repo . --cases bench_cases_random_triplets --output bench_results\current_strategy_random_two --maps Map01 Map02 Map03 Map04 Map05 Map06 Map07 Map07t4 Map08 Map09 --kinds random_s11 random_s29 --repeats 1 --timeout 300 --python $py
```

重跑会覆盖同名日志，且无人机初态可能不同。本报告只陈述这 30 轮实际观测，不将一次布局成功宣称为稳定保证。
