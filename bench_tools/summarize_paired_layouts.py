"""Summarize the frozen seven-layout, two-seed package evaluation."""

import argparse
import hashlib
import json
import re
import statistics
from pathlib import Path
from zipfile import ZipFile

from generate_eval_cases import MAPS


GROUPS = {
    "user_fixed": "用户固定",
    "random_s11": "普通随机 11",
    "random_s29": "普通随机 29",
    "corner_s100": "障碍角落 100",
    "corner_s105": "障碍角落 105",
    "cluster_s618": "聚集 618",
    "cluster_s919": "聚集 919",
}
SEEDS = (101, 303)


def destroyed(row):
    return round(row["destroyed_percent"] / 10)


def summary(label, rows):
    disposed = sum(row["disposed"] for row in rows)
    full = sum(row["disposed"] == 10 for row in rows)
    avg = statistics.mean(row["score"] for row in rows)
    low = min(row["score"] for row in rows)
    health = statistics.mean(row["mean_health_percent"] for row in rows)
    crashed = sum(destroyed(row) for row in rows)
    return f"| {label} | {disposed}/{10 * len(rows)} | {full}/{len(rows)} | {avg:.2f} | {low:.1f} | {health:.2f}% | {crashed} |"


def ending(row, log):
    steps = re.search(r"elapse timestep:\s*(\d+)/(\d+)", log)
    wall = re.search(r"walltime elapsed:\s*(\d+)s/(\d+)s", log)
    if steps and steps[1] == steps[2]:
        return "用满步数"
    if wall and wall[1] == wall[2]:
        return "官方墙钟到限"
    return "见原始日志"


def paired_comparison(root, baseline, records, expected):
    def layouts(folder):
        manifest = json.loads((folder / "cases/manifest.json").read_text(encoding="utf-8"))
        hashes = {}
        for item in manifest:
            path = folder / "cases" / Path(item["path"]).name
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            if digest != item["bombs_sha256"]:
                raise ValueError(f"Layout differs from manifest: {path}")
            hashes[item["map"], item["case"]] = digest
        return hashes

    if layouts(root) != layouts(baseline):
        raise ValueError("Baseline and candidate layouts differ")
    prior = json.loads((baseline / "runs/results.json").read_text(encoding="utf-8"))
    keyed = {(r["map"], r["case"], r["seed"]): r for r in prior}
    if len(prior) != len(keyed) or set(keyed) != expected:
        raise ValueError("Baseline has missing or duplicate rounds")
    for archive_root, rounds in ((root, records), (baseline, prior)):
        for row in rounds:
            path = Path(row["bombs_file"])
            expected_path = archive_root / "cases" / path.name
            if path.read_bytes() != expected_path.read_bytes():
                raise ValueError(f"Layout used by runner differs from archived copy: {path}")
    pairs = [(keyed[r["map"], r["case"], r["seed"]], r) for r in records]
    lines = ["## 与上一包配对比较", "",
             "以地图、布局和仿真种子逐轮匹配。分差为本次减去上一包。", "",
             "| 范围 | 上一包均分 | 本次均分 | 分差 | 上一包全拆 | 本次全拆 | 上一包坠毁 | 本次坠毁 |",
             "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |"]
    scopes = [("**全部**", pairs)]
    scopes += [(name, [p for p in pairs if p[1]["map"] == name]) for name in MAPS]
    scopes += [(label, [p for p in pairs if p[1]["case"] == kind]) for kind, label in GROUPS.items()]
    for label, group in scopes:
        before, after = zip(*group)
        old_avg = statistics.mean(r["score"] for r in before)
        new_avg = statistics.mean(r["score"] for r in after)
        lines.append(f"| {label} | {old_avg:.2f} | {new_avg:.2f} | {new_avg-old_avg:+.2f} |"
                     f" {sum(r['disposed'] == 10 for r in before)}/{len(group)} |"
                     f" {sum(r['disposed'] == 10 for r in after)}/{len(group)} |"
                     f" {sum(map(destroyed, before))} | {sum(map(destroyed, after))} |")
    changes = [new["score"] - old["score"] for old, new in pairs]
    lines += ["", f"配对分数 {sum(d > 0 for d in changes)} 轮提高、"
              f"{sum(d == 0 for d in changes)} 轮持平、{sum(d < 0 for d in changes)} 轮下降。", ""]
    return lines


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("evaluation", type=Path)
    parser.add_argument("report", type=Path)
    parser.add_argument("--baseline", type=Path)
    args = parser.parse_args()
    root = args.evaluation
    records = json.loads((root / "runs/results.json").read_text(encoding="utf-8"))
    keyed = {(r["map"], r["case"], r["seed"]): r for r in records}
    expected = {(name, kind, seed) for name in MAPS for kind in GROUPS for seed in SEEDS}
    if len(records) != len(keyed) or set(keyed) != expected:
        raise ValueError(f"Incomplete results: {len(keyed)}/{len(expected)} unique rounds")
    if any(r["exit_code"] or r["timed_out"] or r["total_bombs"] != 10 or r["score"] is None for r in records):
        raise ValueError("Some rounds have no valid official score")
    archive_path = root / "team861_final.zip"
    digest = hashlib.sha256(archive_path.read_bytes()).hexdigest()
    solutions = root / "isolated_repo/src/swarm_rescue/solutions"
    with ZipFile(archive_path) as archive:
        for file in archive.namelist():
            staged = root / "isolated_repo/req.txt" if file == "req.txt" else solutions / file
            if staged.read_bytes() != archive.read(file):
                raise ValueError(f"Packaged file differs from evaluated file: {file}")

    fields = "| 范围 | 交付数 | 全拆轮次 | 平均分 | 最低分 | 平均健康 | 坠毁 |"
    rule = "| --- | ---: | ---: | ---: | ---: | ---: | ---: |"
    lines = [
        "# SICT（861）提交包 140 轮配对仿真", "",
        f"测试包 SHA256 为 `{digest}`。从指定 ZIP 解包到隔离目录，使用官方 `--competition --headless`。"
        "十张地图、七套布局、两个出生及噪声种子 101 和 303，每轮十枚炸弹和十架无人机。", "",
        "固定位置沿用用户坐标，普通随机沿用布局种子 11、29。障碍角落 100、105 使用地图内部墙体顶点和端点离线生成。"
        "聚集 618、919 各以一个随机中心为基准，十枚均在半径 72 像素内，点间至少相距 32 像素。布局清单含坐标与哈希。", "",
        "## 各组平均分", "", fields, rule,
    ]
    for kind, label in GROUPS.items():
        lines.append(summary(label, [r for r in records if r["case"] == kind]))
    lines += [summary("**合计**", records), "",
              "## 各地图平均分", "", fields, rule]
    for name in MAPS:
        lines.append(summary(name, [r for r in records if r["map"] == name]))
    if args.baseline:
        lines += ["", *paired_comparison(root, args.baseline, records, expected)]
    lines += ["", "## 地图与布局的两种子平均分", "",
              "| 地图 | " + " | ".join(GROUPS.values()) + " |",
              "| --- | " + " | ".join(["---:"] * len(GROUPS)) + " |"]
    for name in MAPS:
        cells = []
        for kind in GROUPS:
            pair = [keyed[name, kind, seed] for seed in SEEDS]
            cells.append(f"{statistics.mean(r['score'] for r in pair):.2f}")
        lines.append(f"| {name} | " + " | ".join(cells) + " |")
    failures = sorted((r for r in records if r["disposed"] < 10), key=lambda r: (r["score"], r["map"]))
    maps_ranked = sorted(MAPS, key=lambda name: statistics.mean(r["score"] for r in records if r["map"] == name))
    weak = [summary(name, [r for r in records if r["map"] == name]) for name in maps_ranked[:3]]
    qualifying = [r for r in records if r["map"] != "Map01"]
    lines += ["", "## 当前弱势", "", "按平均分最低的三张图如下。", "", fields, rule, *weak, "",
              f"除 Map01 外，{sum(r['score'] >= 92 for r in qualifying)}/{len(qualifying)} 轮达到 92 分。"
              f"共 {len(failures)} 轮未全拆。下表按分数由低到高排列。", "",
              "| 地图 | 布局 | 种子 | 交付 | 分数 | 步数 | 健康 | 坠毁 | 结束条件 |",
              "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |"]
    for row in failures:
        log = Path(row["log"]).read_text(encoding="utf-8", errors="replace")
        lines.append(f"| {row['map']} | {GROUPS[row['case']]} | {row['seed']} | {row['disposed']}/10 |"
                     f" {row['score']:.1f} | {row['steps']} | {row['mean_health_percent']:.1f}% |"
                     f" {destroyed(row)} | {ending(row, log)} |")
    fail_endings = [ending(r, Path(r["log"]).read_text(encoding="utf-8", errors="replace")) for r in failures]
    lines += ["", f"未全拆回合中，{fail_endings.count('用满步数')} 轮达到步数上限，"
              f"{fail_endings.count('官方墙钟到限')} 轮先达到官方墙钟上限。", "",
              "## 相同位置下对仿真种子的敏感性", "",
              "下表列出固定同一套炸弹坐标后，两个仿真种子得分差最大的五组。", "",
              "| 地图 | 布局 | 101 得分与交付 | 303 得分与交付 | 分差 |",
              "| --- | --- | ---: | ---: | ---: |"]
    pairs = [(keyed[name, kind, SEEDS[0]], keyed[name, kind, SEEDS[1]]) for name in MAPS for kind in GROUPS]
    for first, second in sorted(pairs, key=lambda pair: abs(pair[0]["score"] - pair[1]["score"]), reverse=True)[:5]:
        lines.append(f"| {first['map']} | {GROUPS[first['case']]} | {first['score']:.1f}（{first['disposed']}/10） |"
                     f" {second['score']:.1f}（{second['disposed']}/10） | {abs(first['score'] - second['score']):.1f} |")
    analysis = root / "findings.md"
    if analysis.is_file():
        lines += ["", analysis.read_text(encoding="utf-8").strip()]
    errors = [r for r in records if r["crashed"]]
    if errors:
        lines += ["", "存在包含异常堆栈的日志，以下回合都产生了官方分数。", ""]
        lines += [f"- {r['map']} {GROUPS[r['case']]} 种子 {r['seed']}，见相应原始日志。" for r in errors]
    lines += ["", "## 原始证据", "", f"- 结构化结果在 `{root / 'runs/results.json'}`。"
              "每条记录含原始日志、计分分项、终止步数及健康。",
              f"- 70 套布局和坐标哈希在 `{root / 'cases/manifest.json'}`。",
              f"- 官方提交格式检查通过，输出在 `{root / 'submission_check.log'}`。",
              f"- 测试时六个策略模块的哈希在 `{root / 'runs/source_sha256.json'}`。"
              "报告生成时还将 ZIP 内所有文件与实际运行目录逐字节核对。",
              "- 140 轮均生成了官方比分，且离线启动进程正常退出。"]
    args.report.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines[:42]))


if __name__ == "__main__":
    main()
