"""Verify the paired all-map runs and summarize the generic changes."""

from hashlib import sha256
import json
from pathlib import Path
from statistics import mean


ROOT = Path(__file__).resolve().parent
REPO = ROOT.parents[1]
BASELINE = ROOT.parent / "team861_survival/all_maps_baseline"
CANDIDATES = {
    "contact": "重复碰撞后寻找有净空的局部出口",
    "observations": "反复确认掉落目标与消息时间戳",
    "backtrack": "重复碰撞后退回已飞过且经 LiDAR 复核的位置",
    "consistent_pose": "定位坐标可信时才用观测修正共享任务",
    "confirmed": "确认掉落之前保留原有目标追踪",
}


def key(row):
    return row["map"], row["case"], row["seed"]


def read_results(directory):
    rows = json.loads((directory / "results.json").read_bytes())
    assert len(rows) == 24 and len({r["map"] for r in rows}) == 24, directory
    for row in rows:
        assert row["exit_code"] == 0 and row["total_bombs"] == 10 and row["score"] is not None, row
        raw = Path(row["log"]).read_text(encoding="utf-8")
        assert f"rescued nb: {row['disposed']}/10" in raw and f"round score: {row['score']:.1f}%" in raw
        assert row["destroyed_count"] == round(row["destroyed_percent"] / 10), row
    return {key(row): row for row in rows}


def metrics(rows):
    return (sum(r["disposed"] for r in rows), sum(r["destroyed_count"] for r in rows),
            sum(r["disposed"] == 10 and r["destroyed_count"] == 0 for r in rows),
            mean(r["score"] for r in rows))


def compare(base, candidate):
    assert base.keys() == candidate.keys()
    regressions = [case for case, row in candidate.items()
                   if row["disposed"] < base[case]["disposed"] or row["destroyed_count"] > base[case]["destroyed_count"]]
    gains = [case for case, row in candidate.items()
             if row["disposed"] > base[case]["disposed"] or row["destroyed_count"] < base[case]["destroyed_count"]]
    capped = [case for case, row in candidate.items() if row["walltime_limit_reached"]]
    return regressions, gains, capped


def verify_snapshot(directory, source):
    snapshot = json.loads((directory / "source_sha256.json").read_bytes())
    for path in source.glob("*.py"):
        assert snapshot[path.name] == sha256(path.read_bytes()).hexdigest(), path


def gps_replay():
    runs = [ROOT / name / "results.json" for name in ("gps_replay_control", "gps_replay_candidate")]
    if not all(path.is_file() for path in runs):
        return None
    old, new = (json.loads(path.read_bytes())[0] for path in runs)
    assert key(old) == key(new)
    assert all(r["exit_code"] == 0 and not r["walltime_limit_reached"] for r in (old, new))
    return old, new


def select_primary():
    base = read_results(BASELINE)
    verify_snapshot(BASELINE, BASELINE / "runtime/src/swarm_rescue/solutions")
    eligible = []
    for name in CANDIDATES:
        run = ROOT / f"{name}_all_maps"
        rows = read_results(run)
        verify_snapshot(run, ROOT / f"{name}_candidate")
        failures, gains, capped = compare(base, rows)
        if not failures and not capped and gains:
            eligible.append((name, metrics(list(rows.values()))))
    eligible.sort(key=lambda item: (item[1][2], item[1][0], -item[1][1], item[1][3]), reverse=True)
    return eligible[0][0] if eligible else None


def main():
    base = read_results(BASELINE)
    options = {name: read_results(ROOT / f"{name}_all_maps") for name in CANDIDATES}
    choice = select_primary()
    lines = ["# SICT 全地图通用策略调整", "",
             "碰撞与目标状态调整均从当前正式策略派生。控制器仅使用本机传感器、健康值、实时定位估计与官方通信。比赛源码以外的改动均用于隔离评测。", "",
             "第一组使用既有 24 张地图上各一组困难布局。五个候选与同驱动基线逐图复用完全相同的布局及仿真种子，采用官方 `--competition --headless` 与原计分。", "",
             "| 版本 | 交付 | 坠毁 | 全拆且零坠毁 | 平均分 | 交付或坠毁退化图数 | 实际时间上限轮次 |",
             "| --- | ---: | ---: | ---: | ---: | ---: | ---: |"]
    for name, rows in (("baseline", base), *options.items()):
        disposed, lost, passed, score = metrics(list(rows.values()))
        failures, _, capped = compare(base, rows)
        label = "当前策略" if name == "baseline" else CANDIDATES[name]
        lines.append(f"| {label} | {disposed}/240 | {lost} | {passed}/24 | {score:.2f} | {len(failures)} | {len(capped)} |")
    lines += ["", "各版本数字依次为交付数 / 坠毁数。", "",
              "| 地图 | 布局 | 种子 | 当前 | 局部出口 | 观察确认 | 已飞航迹 | 定位可信时确认 | 确认前持续追踪 |",
              "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |"]
    for case in sorted(base):
        entries = [base[case], *(rows[case] for rows in options.values())]
        results = " | ".join(f"{row['disposed']}/{row['destroyed_count']}" for row in entries)
        lines.append(f"| {case[0]} | {case[1]} | {case[2]} | {results} |")
    lines += ["", "## 选择与复测", ""]
    gps = gps_replay()
    if gps:
        old, new = gps
        old_disposed, new_disposed = old["disposed"], new["disposed"]
        events = sum(line.startswith("TASK_OBSERVATION ") for line in Path(new["log"]).read_text(encoding="utf-8").splitlines())
        lines.append(f"- 在 {old['map']} 单独用同一诊断驱动复测，当前策略交付 {old_disposed}/10、坠毁 {old['destroyed_count']} 架，"
                     f"定位可信版交付 {new_disposed}/10、坠毁 {new['destroyed_count']} 架，记录到 {events} 次最终状态纠正。"
                     "代码复查发现，等待确认时的提前返回也改变了正常追踪，第五版保留了这段更新。")
    if choice:
        lines.append(f"纳入诊断复跑后，进入第二组的候选为「{CANDIDATES[choice]}」。")
        holdout_base = ROOT / "distinct_baseline"
        holdout_candidate = ROOT / "distinct_candidate"
        if (holdout_base / "results.json").exists() and (holdout_candidate / "results.json").exists():
            prior = read_results(holdout_base)
            later = read_results(holdout_candidate)
            assert all(case[1] not in {k[1] for k in base if k[0] == case[0]} for case in later)
            verify_snapshot(holdout_candidate, ROOT / f"{choice}_candidate")
            failures, gains, capped = compare(prior, later)
            for label, rows in (("当前", prior), ("候选", later)):
                disposed, lost, passed, score = metrics(list(rows.values()))
                lines.append(f"- 第二组 {label}。交付 {disposed}/240，坠毁 {lost} 架，全拆且零坠毁 {passed}/24，均分 {score:.2f}。")
            lines.append(f"- 第二组退化 {len(failures)} 张图，改善 {len(gains)} 张图，实际时间上限 {len(capped)} 轮。")
            if failures:
                lines.append(f"- 退化场景为 {'、'.join(case[0] for case in failures)}。")
                lines.append("- 正式策略采用基线。")
            elif not capped:
                active = REPO / "src/swarm_rescue/solutions/my_drone_rescue.py"
                tested = ROOT / f"{choice}_candidate/my_drone_rescue.py"
                if active.read_bytes() == tested.read_bytes():
                    lines.append("- 已将通过两组配对验证的 `my_drone_rescue.py` 更新到工作区，并通过官方提交检查。现有提交 ZIP 对应上一版。")
            lines += ["", "| 地图 | 不同布局 | 种子 | 当前交付 | 候选交付 | 当前坠毁 | 候选坠毁 | 当前分数 | 候选分数 |",
                      "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |"]
            for case in sorted(prior):
                before, after = prior[case], later[case]
                lines.append(f"| {case[0]} | {case[1]} | {case[2]} | {before['disposed']}/10 | {after['disposed']}/10 | "
                             f"{before['destroyed_count']} | {after['destroyed_count']} | {before['score']:.1f} | {after['score']:.1f} |")
    else:
        lines.append("本轮逐图筛选未发现满足保留条件的候选，工作区和提交包采用基线。")

    lines += ["", "## 逐轮结果", "",
              "- [配对基线](bench_results/team861_survival/all_maps_baseline/results.json)"]
    for name, label in CANDIDATES.items():
        lines.append(f"- [{label}](bench_results/team861_generic_recovery/{name}_all_maps/results.json)")
    for name, label in (("gps_replay_control", "GPS 场景当前策略复跑"), ("gps_replay_candidate", "GPS 场景候选复跑"),
                        ("observation_gain_replay", "获益场景任务状态诊断"),
                        ("distinct_baseline", "第二组当前策略"), ("distinct_candidate", "第二组候选")):
        if (ROOT / name / "results.json").exists():
            lines.append(f"- [{label}](bench_results/team861_generic_recovery/{name}/results.json)")
    lines.append("- [官方提交检查日志](bench_results/team861_generic_recovery/submission_checks.log)")
    path = REPO / "TEAM861_GENERIC_RECOVERY_REPORT.md"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(path)
    for name, rows in options.items():
        failures, gains, capped = compare(base, rows)
        print(name, metrics(list(rows.values())), "regressions", failures, "gains", gains, "capped", capped)
    print("candidate", choice)


if __name__ == "__main__":
    main()
