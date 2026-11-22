"""Summarize the completed, paired survival experiments from official logs."""

from collections import Counter
import json
from pathlib import Path
from statistics import mean


ROOT = Path(__file__).resolve().parent
REPO = ROOT.parents[1]


def read_run(name, count):
    path = ROOT / name / "results.json"
    rows = json.loads(path.read_bytes())
    assert len(rows) == count, (name, len(rows), count)
    assert len({(r["map"], r["case"], r["seed"]) for r in rows}) == count
    for row in rows:
        assert row["exit_code"] == 0 and row["score"] is not None and row["total_bombs"] == 10, row
        raw = Path(row["log"]).read_text(encoding="utf-8")
        assert f"rescued nb: {row['disposed']}/10" in raw and f"round score: {row['score']:.1f}%" in raw
        row["walltime_limit_reached"] = "is reached first" in raw
        row["destroyed_count"] = round(row["destroyed_percent"] / 10)
    return rows


def totals(rows, baseline=False):
    prefix = "baseline_" if baseline else ""
    delivered = sum(r[prefix + "disposed"] for r in rows)
    lost = sum(r["baseline_destroyed" if baseline else "destroyed_count"] for r in rows)
    passed = sum(r[prefix + "disposed"] == 10 and r["baseline_destroyed" if baseline else "destroyed_count"] == 0
                 for r in rows)
    return f"{delivered}/{10 * len(rows)} | {lost} | {passed}/{len(rows)} | {mean(r[prefix + 'score'] for r in rows):.2f}"


def main():
    diagnostics = read_run("baseline_trace", 5)
    assert all(r["disposed"] == r["baseline_disposed"] and r["destroyed_count"] == r["baseline_destroyed"]
               for r in diagnostics)
    pilots = {version: read_run(f"pilot_v{version}", count) for version, count in
              [(1, 7), (2, 9), (3, 9), (4, 9), (5, 9), (6, 7), (7, 9), (8, 4), (9, 10)]}
    broad = read_run("all_maps_v5", 24)
    guarded = read_run("all_maps_v8", 24)
    baseline = read_run("all_maps_baseline", 24)
    rerun = read_run("pilot_v5_serial", 2)
    repeated_baseline = read_run("baseline_special_zone_repeat", 2)
    assert set(Counter(r["map"] for r in broad).values()) == {1}
    by_map = {r["map"]: r for r in guarded}
    baseline_by_map = {r["map"]: r for r in baseline}
    for index in (by_map, baseline_by_map):
        assert len(index) == 24 and all((r["case"], r["seed"]) == (index[r["map"]]["case"], index[r["map"]]["seed"])
                                       for r in broad)
    run_count = sum(map(len, (diagnostics, broad, guarded, baseline, rerun, repeated_baseline, *pilots.values())))
    lines = [
        "# SICT 全拆与零坠毁配对测试",
        "",
        f"验收目标为每轮交付 10/10、官方坠毁数为零。本次新增 {run_count} 轮有效实测，以既有 224 轮选取布局与仿真种子。使用官方 `--competition --headless` 和原计分。",
        "五个场景通过带诊断日志的基线重跑复现了交付和坠毁。另对两张特殊区地图独立复跑，发现一条归档成绩存在重放差异，因此全图对照采用同一驱动重新跑出的基线。诊断只记录公开传感器、本机健康和策略状态。",
        "",
        "## 覆盖全部 24 张地图的回归",
        "",
        "每张地图选基线中交付最少的一轮。并列时优先选坠毁更多的一轮，再按分数从低到高选。",
        "",
        "| 范围 | 版本 | 交付 | 坠毁 | 同时全拆与零坠毁 | 均分 |",
        "| --- | --- | ---: | ---: | ---: | ---: |",
    ]
    baseline_stock = REPO / "bench_results/team861_current_paired_140/runs/results.json"
    stock_names = {r["map"] for r in json.loads(baseline_stock.read_bytes())}
    for label, group in (("原 10 图", [r for r in broad if r["map"] in stock_names]),
                         ("新增 14 图", [r for r in broad if r["map"] not in stock_names]),
                         ("总计", broad)):
        lines.append(f"| {label} | 同驱动基线 | {totals([baseline_by_map[r['map']] for r in group])} |")
        lines.append(f"| {label} | 第五版 | {totals(group)} |")
        lines.append(f"| {label} | 第八版 | {totals([by_map[r['map']] for r in group])} |")
    lines += ["", "交付与坠毁列按基线、第五版、第八版的顺序排列。",
              "", "| 地图 | 布局 | 种子 | 交付 | 坠毁 | 分数 | 步数 |",
              "| --- | --- | ---: | ---: | ---: | ---: | ---: |"]
    for r in sorted(broad, key=lambda r: r["map"]):
        other = by_map[r["map"]]
        base = baseline_by_map[r["map"]]
        lines.append(f"| {r['map']} | {r['case']} | {r['seed']} | {base['disposed']}/10 → {r['disposed']}/10 → {other['disposed']}/10 | "
                     f"{base['destroyed_count']} → {r['destroyed_count']} → {other['destroyed_count']} | "
                     f"{base['score']:.1f} → {r['score']:.1f} → {other['score']:.1f} | {r['steps']} / {other['steps']} |")
    lines += ["", f"同驱动基线有 {sum(r['walltime_limit_reached'] for r in baseline)} 轮先触发实际时间上限。"
              f"第五版有 {sum(r['walltime_limit_reached'] for r in broad)} 轮。"
              f"第八版有 {sum(r['walltime_limit_reached'] for r in guarded)} 轮。"]
    lines += ["", "## 候选试验与独立复测", "",
              "| 候选 | 版本 | 交付 | 坠毁 | 同时全拆与零坠毁 | 均分 | 实际时间上限轮次 |",
              "| --- | --- | ---: | ---: | ---: | ---: | ---: |"]
    for version, rows in pilots.items():
        lines.append(f"| {version} | 同布局归档基线 | {totals(rows, baseline=True)} | 0 |")
        lines.append(f"| {version} | 候选试验 | {totals(rows)} | {sum(r['walltime_limit_reached'] for r in rows)} |")
    lines += ["", "第五版单独复测初筛中先触发实际时间上限的两轮。", "",
              "| 地图 | 种子 | 基线交付 | 第五版交付 | 第五版坠毁 | 真实耗时 | 实际时间上限 |",
              "| --- | ---: | ---: | ---: | ---: | ---: | --- |"]
    for r in sorted(rerun, key=lambda r: r["map"]):
        lines.append(f"| {r['map']} | {r['seed']} | {r['baseline_disposed']}/10 | {r['disposed']}/10 | "
                     f"{r['destroyed_count']} | {r['walltime_seconds']:.0f} 秒 | {'触发' if r['walltime_limit_reached'] else '—'} |")
    lines += ["", "回归驱动复跑的额外基线。", "",
              "| 地图 | 布局 | 种子 | 归档基线交付 | 复跑交付 | 归档基线坠毁 | 复跑坠毁 |",
              "| --- | --- | ---: | ---: | ---: | ---: | ---: |"]
    for r in repeated_baseline:
        lines.append(f"| {r['map']} | {r['case']} | {r['seed']} | {r['baseline_disposed']}/10 | {r['disposed']}/10 | "
                     f"{r['baseline_destroyed']} | {r['destroyed_count']} |")
    lines += [
        "", "## 策略与故障证据", "",
        "- 第一至第五版试验了队友失联识别、语义观测固定接触、风险点绕行和近距制动。第五版增加实飞位置反证以及同向跟车间距。第六版单独测试了极低健康空载机撤入开阔地并释放任务。第七版把单次失联降为局部临时谨慎项，并对反复卡住的载弹机启用脱困。第八版在传感器同时失效且恢复后才激活绕行。第九版仅根据本机亲历的同时失效生成风险点，并通过官方通信同步。",
        "- `KILL_ZONE` 同时让 LiDAR 和里程计失效，也会屏蔽驱动。基线诊断记录了健康仍在满值附近的无人机突然失去两种观测，随后多架无人机在同一带停滞并发生碰撞。官方坠毁计数按健康耗尽统计，停滞状态单列为故障。",
        "- 官方 LiDAR 和语义传感器的场景渲染关闭了特殊区绘制。绕行机制的输入限定为本机公开传感器、实时定位估计和官方通信中的队友观测。",
        "- 每版全图回归覆盖 24 张地图各一轮。原始 224 轮成绩用于选取配对布局与种子。",
        "", "## 工作区选择", "",
        "第五版整体提升，但 Map06 和 Map08 固定布局的漏拆在单进程复测中仍然存在。第八、九版在 MapMedium01 的同一轮均比复跑基线多坠毁两架。现行比赛源码和提交 ZIP 采用原策略，候选源码与仿真结果分别保存在 `bench_results/team861_survival`。",
        "工作区、提交 ZIP、第五版、第八版、第九版均通过官方提交检查。",
        "", "## 结果文件", "",
        "- [同驱动 24 图基线](bench_results/team861_survival/all_maps_baseline/results.json)",
        "- [第五版逐地图回归](bench_results/team861_survival/all_maps_v5/results.json)",
        "- [第八版逐地图回归](bench_results/team861_survival/all_maps_v8/results.json)",
        "- [第五版单进程复测](bench_results/team861_survival/pilot_v5_serial/results.json)",
        "- [第七版逐轮试验](bench_results/team861_survival/pilot_v7/results.json)",
        "- [第八版逐轮试验](bench_results/team861_survival/pilot_v8/results.json)",
        "- [第九版逐轮试验](bench_results/team861_survival/pilot_v9/results.json)",
        "- [提交检查日志](bench_results/team861_survival/submission_checks.log)",
        "- [原始 224 轮报告](TEAM861_224_ROUND_REPORT.md)",
    ]
    path = REPO / "TEAM861_SURVIVAL_OPTIMIZATION_REPORT.md"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(path)
    print("Baseline", totals(baseline))
    print("Candidate 5", totals(broad))
    print("Candidate 8", totals(guarded))


if __name__ == "__main__":
    main()
