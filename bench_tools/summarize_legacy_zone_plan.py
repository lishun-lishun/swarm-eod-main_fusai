"""Compare the historical special-zone overlay against official paired logs."""
from collections import defaultdict
import csv
import json
from pathlib import Path
from statistics import mean
import sys

from summarize_survival_224 import key, read, verify


def stats(rows):
    return (f"{mean(r['score'] for r in rows):.2f} | {sum(r['disposed'] for r in rows)}/{10 * len(rows)} | "
            f"{sum(r['disposed'] == 10 for r in rows)}/{len(rows)} | "
            f"{sum(r['destroyed_count'] for r in rows)}")


def main(root, output, reference):
    rows = read(root / "rounds/results.json")
    old = {key(r): r for r in read(reference)}
    assert len(rows) == len({key(r) for r in rows}) == 84
    maps = defaultdict(list)
    layouts = defaultdict(list)
    plan = read(root / "zones.json")
    for row in rows:
        verify(row)
        log = Path(row["log"]).read_text(encoding="utf-8", errors="replace")
        prefix = f"BENCH_ZONES {row['map']} "
        applied = json.loads(next(line[len(prefix):] for line in log.splitlines() if line.startswith(prefix)))
        if isinstance(applied, dict):
            assert applied["counts"] == {"gps": 2, "com": 2}
            applied = applied["overlay"]
        assert applied == plan[row["map"]]
        maps[row["map"]].append(row)
        layouts[row["case"]].append(row)
    assert len(maps) == 14 and all(len(batch) == 6 for batch in maps.values())
    controls = {}
    for folder in ("with_kill", "without_kill"):
        parsed = read(root / folder / "results.json")
        assert len(parsed) == 7
        for row in parsed:
            verify(row)
        controls[folder] = {key(r): r for r in parsed}
    assert set(controls["with_kill"]) == set(controls["without_kill"])
    for folder in ("with_kill", "without_kill", "smoke"):
        assert read(root / folder / "source_sha256.json") == read(root / "rounds/source_sha256.json")
    title = "| 分组 | 旧均分 | 旧交付 | 旧全拆 | 旧坠毁 | 新均分 | 新交付 | 新全拆 | 新坠毁 |"
    rule = "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |"
    report = [
        "# 历史改编图无毁伤区复测", "",
        "使用当前低血量撤离策略，沿用 14 张历史改编图的障碍、出生规则、处置区、两套种子和三类布局。"
        "每图六轮，共 84 轮，每轮十架无人机、十枚炸弹。旧组引用上次 224 轮报告中的 84 轮历史图成绩。",
        "每图固定一套可复现随机位置，启用两块无 GPS 区和两块无通信区，取消原特殊区。"
        "新区域以原同类区域为尺寸基准，缺失同类时用原毁伤区，再缺失时用地图比例。"
        "目标面积为原参考的两倍，按像素取整并裁到场地内。",
        "", "## 逐地图及分组", "", title, rule,
    ]
    for name, batch in [("全部", rows), *sorted(maps.items()), *sorted(layouts.items())]:
        report.append(f"| {name} | {stats([old[key(r)] for r in batch])} | {stats(batch)} |")
    report += ["", "## 只关闭毁伤区的对照", "",
               "各选原启用毁伤区的七张地图中历史坠毁最多的一轮，复用相同策略、布局和种子，保留原来的其他区域。"
               "这是困难案例的配对结果，适用于判断这些场景中的毁伤区影响。", "", title, rule]
    a, b = controls["with_kill"], controls["without_kill"]
    report.append(f"| 7 轮合计 | {stats(list(a.values()))} | {stats(list(b.values()))} |")
    for k in sorted(a):
        report.append(f"| {k[0]} {k[1]} {k[2]} | {stats([a[k]])} | {stats([b[k]])} |")
    cap = sum(r["walltime_limit_reached"] for r in rows)
    report += ["", f"新组 84 轮中有 {cap} 轮先触发官方墙钟上限。", "",
               "[区域清单](bench_results/team861_legacy_zone_overlay/zones.json) · "
               "[逐轮数据与原始日志路径](bench_results/team861_legacy_zone_overlay/paired.csv)"]
    output.write_text("\n".join(report) + "\n", encoding="utf-8")
    fields = ("score", "disposed", "destroyed_count", "steps", "mean_health_percent", "log")
    with (root / "paired.csv").open("w", encoding="utf-8-sig", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["map", "layout", "seed", *("old_" + f for f in fields), *("new_" + f for f in fields),
                         "walltime_limit_reached"])
        for row in sorted(rows, key=key):
            base = old[key(row)]
            writer.writerow([*key(row), *(base[f] for f in fields), *(row[f] for f in fields),
                             row["walltime_limit_reached"]])
    print(report[9])


if __name__ == "__main__":
    main(*(Path(arg) for arg in sys.argv[1:]))
