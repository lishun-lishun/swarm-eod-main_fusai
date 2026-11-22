"""Verify the official results of a full survival benchmark and report them."""

import argparse
from collections import Counter, defaultdict
import csv
import json
from pathlib import Path
import re
from statistics import mean

from run_benchmarks import parse_log
from run_survival_benchmark import REFERENCE, REPO, archived_path


KIND_LABELS = {"user_fixed": "用户固定", "random": "普通随机", "corner": "障碍边角", "cluster": "局部聚集"}
METRICS = ("disposed", "score", "steps", "mean_health_percent", "health_score", "destroyed_percent")


def read(path):
    return json.loads(path.read_bytes())


def key(row):
    return (row["map"], row.get("case", row.get("layout")), row["seed"])


def kind(case):
    return next((k for k in ("random", "corner", "cluster") if case.startswith(k)), case)


def verify(row):
    path = archived_path(row["log"])
    raw = path.read_text(encoding="utf-8", errors="replace")
    data = parse_log(raw)
    if row["exit_code"] or row["total_bombs"] != 10 or "round crashed" in raw.lower():
        raise ValueError(f"No official result for {path}")
    if len(re.findall(r"round score:\s*[\d.]+%", raw)) != 1:
        raise ValueError(f"Expected one official result in {path}")
    if any(row[f] != data[f] or data[f] is None for f in METRICS):
        raise ValueError(f"Result differs from official log in {path}")
    row["step_limit"] = int(re.search(r"elapse timestep:\s*\d+/(\d+)", raw)[1])
    row["callback_warning"] = "Exception ignored from cffi callback" in raw
    row["walltime_limit_reached"] = "is reached first" in raw
    row["destroyed_count"] = round(data["destroyed_percent"] / 10)


def total_line(name, batch):
    old_mean = mean(row["reference_score"] for row in batch)
    avg_score = mean(row["score"] for row in batch)
    full = sum(row["disposed"] == 10 for row in batch)
    safe_full = sum(row["disposed"] == 10 and row["destroyed_count"] == 0 for row in batch)
    return (
        f"| {name} | {len(batch)} | {avg_score:.2f} | {old_mean:.2f} | "
        f"{min(row['score'] for row in batch):.1f} | "
        f"{sum(row['disposed'] for row in batch)}/{10 * len(batch)} | {full} | "
        f"{safe_full} | {sum(row['score'] >= 90 for row in batch)} | "
        f"{sum(row['destroyed_count'] for row in batch)} | "
        f"{mean(row['mean_health_percent'] for row in batch):.2f}% | "
        f"{mean(row['health_score'] for row in batch):.2f}% |"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("full_run", type=Path)
    parser.add_argument("replays", nargs="*", type=Path)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--reference-results", type=Path, help="Prior full run with matching fixtures")
    parser.add_argument("--tested-zip", type=Path, help="Frozen submission archive used for evaluation")
    parser.add_argument("--half-area-overlay", type=Path, help="Verify the capped historical-map overlay")
    args = parser.parse_args()

    baseline = read(REFERENCE / "runs/results.json") + read(REFERENCE / "custom_runs/results.json")
    reference = {key(row): row for row in baseline}
    assert len(reference) == len(baseline) == 224
    assert Counter(Counter(row["map"] for row in baseline).values()) == {14: 10, 6: 14}
    if args.reference_results:
        comparison = read(args.reference_results)
        reference_scores = {key(row): row for row in comparison}
        assert len(comparison) == 224 and set(reference_scores) == set(reference)
        for k, row in reference.items():
            prior = reference_scores[k]
            verify(prior)
            for field in ("score", "disposed", "destroyed_count"):
                row[field] = prior[field]
    raw_runs = read(args.full_run / "results.json")
    found = {key(row): row for row in raw_runs}
    assert len(found) == len(raw_runs) and set(found) == set(reference)
    fingerprints = read(args.full_run / "source_sha256.json")
    replayed = set()
    for folder in args.replays:
        assert read(folder / "source_sha256.json") == fingerprints
        for row in read(folder / "results.json"):
            k = key(row)
            assert k in found
            verify(row)
            prior = found[k]
            if prior["exit_code"] or prior["score"] is None or prior["walltime_limit_reached"]:
                row["first_log"] = prior.get("first_log", prior["log"])
                found[k] = row
                replayed.add(k)

    by_map = defaultdict(list)
    subsets = defaultdict(list)
    rounds = []
    for k, row in sorted(found.items()):
        verify(row)
        row["reference_score"] = reference[k]["score"]
        row["reference_disposed"] = reference[k]["disposed"]
        row["reference_destroyed_count"] = reference[k]["destroyed_count"]
        row["subset"] = "内置地图" if "layout" not in reference[k] else "历史改编图"
        row["layout_kind"] = KIND_LABELS.get(kind(row["case"]), row["case"])
        rounds.append(row)
        by_map[row["map"]].append(row)
        subsets[row["subset"]].append(row)

    report = [
        "# SICT（861）224 轮复测", "",
        "10 张内置地图各 14 轮，14 张历史改编图各 6 轮。每轮 10 架无人机、10 枚炸弹。",
        "复用上一批 224 轮的布局和种子，使用官方 `--competition --headless` 与原计分器。",
        "内置地图含用户固定及各两套随机、边角、聚集布局，以种子 101 和 303 运行。"
        "历史改编图每张覆盖随机、边角和聚集三类各两组。上一批均分作为同场景历史参考。",
        "", "## 总览", "",
    ]
    if args.tested_zip:
        from hashlib import sha256
        from zipfile import ZipFile

        archive = args.tested_zip
        with ZipFile(archive) as opened:
            contents = {name: sha256(opened.read(name)).hexdigest() for name in opened.namelist()
                        if name not in ("req.txt",) and not name.endswith("/")}
            assert (args.full_run / "runtime/req.txt").read_bytes() == opened.read("req.txt")
        assert contents == {name: fingerprints[name] for name in contents}
        extra = fingerprints.keys() - contents.keys()
        report[1:1] = ["", f"评测包 SHA-256 为 `{sha256(archive.read_bytes()).hexdigest()}`。包内策略源码的逐文件哈希与仿真快照一致。"]
        if extra:
            from subprocess import check_output

            assert extra == {"my_drone_place_example.py"}
            example = check_output(["git", "show", "af0bf74:src/swarm_rescue/solutions/my_drone_place_example.py"], cwd=REPO)
            assert sha256(example).hexdigest() == fingerprints["my_drone_place_example.py"]
            report[3:3] = ["ZIP 在隔离目录直接导入时缺少红方示例 `my_drone_place_example.py`，导致启动失败。"
                           "评测运行时补齐了仓库原版示例，全部轮次运行蓝方策略。"]
        check_log = archive.parent / "submission_check.log"
        if check_log.is_file():
            report.append("")
            report.append(f"[ZIP 静态提交检查]({check_log.resolve().relative_to(REPO).as_posix()})")
    if args.reference_results:
        report[report.index("## 总览") - 1:report.index("## 总览") - 1] = [
            "本报告的比较基线为前次完整 224 轮的同场景结果。", ""]
    if args.half_area_overlay:
        zones = read(args.half_area_overlay / "zones.json")
        coverage = read(args.half_area_overlay / "coverage.json")
        historical = {row["map"] for row in subsets["历史改编图"]}
        assert set(zones) == set(coverage) == historical
        for name, plan in zones.items():
            assert Counter(z["kind"] for z in plan) == {"gps": 2, "com": 2}
            width, height = coverage[name]["map_size"]
            area = 0
            for zone in plan:
                w, h = zone["size"]
                x, y = zone["center"]
                assert 2 * abs(x) + w <= width + 0.01 and 2 * abs(y) + h <= height + 0.01
                area += w * h
            assert area == coverage[name]["zone_area_sum"] and 2 * area <= width * height
        for row in rounds:
            log = archived_path(row["log"]).read_text(encoding="utf-8", errors="replace")
            active = [line for line in log.splitlines() if line.startswith("BENCH_ZONES ")]
            if row["map"] not in historical:
                assert not active
                continue
            assert len(active) == 1
            _, name, data = active[0].split(" ", 2)
            info = json.loads(data)
            assert name == row["map"] and info == {"counts": {"gps": 2, "com": 2}, "overlay": zones[name]}
        report[1:1] = [
            "", "本轮使用当前低血量撤离策略。10 张官方地图沿用原设置。"
            "14 张历史改编地图均取消毁伤区，并启用两块无 GPS 区和两块无通信区。"
            "四个矩形的面积总和按不扣除重叠的方式计算，各图不超过图幅面积的 50%。",
            "在上轮面积方案基础上，对超限地图等比例缩小四个矩形，并在新的可放置范围内"
            "沿用每图相同的随机种子。已符合上限的区域布局保持原样。",
        ]
    headers = [
        "| 范围 | 轮次 | 本次均分 | 上批均分 | 最低分 | 交付 | 全拆轮次 | 全拆且零坠毁 | ≥90 分轮次 | 坠毁 | 平均机体健康 | 平均返航健康计分 |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    report += headers + [total_line(label, subsets[label]) for label in ("内置地图", "历史改编图")]
    report += [total_line("全部", rounds), "", "## 按地图", ""] + headers
    report += [total_line(label, batch) for label, batch in by_map.items()]
    report += ["", "## 按布局", ""] + headers
    for label in ("内置地图", "历史改编图"):
        for layout in KIND_LABELS.values():
            batch = [r for r in subsets[label] if r["layout_kind"] == layout]
            if batch:
                report.append(total_line(f"{label} · {layout}", batch))
    report += ["", "## 各套布局和种子", ""] + headers
    for label in ("内置地图", "历史改编图"):
        by_fixture = defaultdict(list)
        for row in subsets[label]:
            by_fixture[row["case"], row["seed"]].append(row)
        for (case, seed), batch in sorted(by_fixture.items()):
            name = f"{label} · {KIND_LABELS.get(case, case)} · 种子 {seed}"
            report.append(total_line(name, batch))
    report += ["", "## 失败与波动", ""]
    for label, batch in sorted(by_map.items(), key=lambda item: mean(r["score"] for r in item[1]))[:5]:
        full = sum(r["disposed"] == 10 for r in batch)
        crashes = sum(r["destroyed_count"] for r in batch)
        report.append(f"- {label} 均分 {mean(r['score'] for r in batch):.2f}，全拆 {full}/{len(batch)} 轮，坠毁 {crashes} 架次。")
    incomplete = [r for r in rounds if r["disposed"] < 10]
    capped = sum(r["steps"] == r["step_limit"] for r in incomplete)
    timed = sum(r["walltime_limit_reached"] for r in rounds)
    warnings = sum(r["callback_warning"] for r in rounds)
    old_delivered = sum(r["reference_disposed"] for r in rounds)
    old_crashes = sum(r["reference_destroyed_count"] for r in rounds)
    old_full_safe = sum(r["reference_disposed"] == 10 and r["reference_destroyed_count"] == 0 for r in rounds)
    delivered = sum(r["disposed"] for r in rounds)
    crashes = sum(r["destroyed_count"] for r in rounds)
    full_safe = sum(r["disposed"] == 10 and r["destroyed_count"] == 0 for r in rounds)
    report += [
        f"- 按同一组 224 个场景比较，交付 {old_delivered} → {delivered}，"
        f"坠毁 {old_crashes} → {crashes} 架次，全拆且零坠毁 {old_full_safe} → {full_safe} 轮。",
        f"- 未全拆 {len(incomplete)} 轮，其中 {capped} 轮达到步数上限。末次有效结果有 {timed} 轮先达墙钟上限。",
        f"- 官方原始日志有 {warnings} 轮出现被忽略的 CFFI 回调异常。这些轮次均返回完整官方成绩。"
        f"另有 {len(replayed)} 个场景使用复跑结果，逐轮表保留首次日志路径。",
        "", "## 交付数最低的场景", "",
        "| 地图 | 布局 | 种子 | 交付 | 坠毁 | 分数 | 步数 | 终止条件 |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | --- |",
    ]
    for row in sorted(incomplete, key=lambda r: (r["disposed"], r["score"], r["map"]))[:12]:
        outcome = "墙钟上限" if row["walltime_limit_reached"] else (
            "步数上限" if row["steps"] >= row["step_limit"] else "仿真结束")
        report.append(
            f"| {row['map']} | {row['case']} | {row['seed']} | {row['disposed']}/10 | "
            f"{row['destroyed_count']} | {row['score']:.1f} | {row['steps']}/{row['step_limit']} | {outcome} |"
        )
    report += ["", "## 评测文件", ""]
    if args.half_area_overlay:
        for name, label in (("coverage.json", "各图区域面积核验"), ("zones.json", "区域位置与尺寸")):
            path = (args.half_area_overlay / name).resolve().relative_to(REPO)
            report.append(f"- [{label}]({path.as_posix()})")
    for name, label in (("verified_rounds.csv", "逐轮成绩和日志路径"), ("results.json", "首轮运行记录"),
                        ("source_sha256.json", "策略源码指纹")):
        path = (args.full_run / name).resolve().relative_to(REPO)
        report.append(f"- [{label}]({path.as_posix()})")
    args.report.write_text("\n".join(report) + "\n", encoding="utf-8")

    columns = (
        "map", "case", "seed", "subset", "layout_kind", "disposed", "total_bombs", "score", "steps", "step_limit",
        "mean_health_percent", "health_score", "destroyed_count", "walltime_seconds", "walltime_limit_reached",
        "callback_warning", "reference_disposed", "reference_score", "reference_destroyed_count", "log", "first_log",
    )
    with (args.full_run / "verified_rounds.csv").open("w", encoding="utf-8-sig", newline="") as file:
        table = csv.DictWriter(file, fieldnames=columns, extrasaction="ignore")
        table.writeheader()
        table.writerows(rounds)
    print(total_line("All", rounds))


if __name__ == "__main__":
    main()
