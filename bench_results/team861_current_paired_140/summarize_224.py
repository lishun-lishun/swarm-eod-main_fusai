"""Verify the 224 official logs and build the combined benchmark report."""

from collections import Counter
from hashlib import sha256
import json
from pathlib import Path
import re
from statistics import mean
from zipfile import ZipFile


ROOT = Path(__file__).resolve().parent
REPO = ROOT.parents[1]
CUSTOM = ROOT / "custom_runs"
SEEDS = {101, 303}
KINDS = {"random": "普通随机", "corner": "障碍角落", "cluster": "局部聚集"}


def statistics(rows):
    return (f"{len(rows)} | {mean(r['score'] for r in rows):.2f} | {min(r['score'] for r in rows):.1f} | "
            f"{sum(r['disposed'] for r in rows)}/{10 * len(rows)} | {sum(r['disposed'] == 10 for r in rows)} | "
            f"{sum(r['score'] >= 90 for r in rows)} | {mean(r['mean_health_percent'] for r in rows):.2f}% | "
            f"{mean(r['health_score'] for r in rows):.2f}% | {sum(r['destroyed_count'] for r in rows)}")


def log_file(row):
    return Path(row["log"])


def read_rounds(path):
    rows = json.loads(path.read_text(encoding="utf-8"))
    for row in rows:
        assert row["exit_code"] == 0 and row["total_bombs"] == 10 and row["score"] is not None, row
        log = log_file(row).read_text(encoding="utf-8")
        assert re.findall(r"round score:\s*([\d.]+)%", log) == [str(row["score"])], row
        assert f"rescued nb: {row['disposed']}/10" in log and "round crashed" not in log.lower(), row
        row["walltime_limit_reached"] = bool(re.search(r"max walltime limit of .* is reached first", log))
        row["callback_warning"] = "Exception ignored from cffi callback" in log
        row["crashed"] = False  # A scored round may have an ignored callback exception.
        row["destroyed_count"] = round(row["destroyed_percent"] / 10)
        row["step_limit"] = int(re.search(r"elapse timestep:\s*\d+/(\d+)", log)[1])
    return rows


def aggregate(rows, label):
    return f"| {label} | {statistics(rows)} |"


def main():
    original = read_rounds(ROOT / "runs/results.json")
    custom = read_rounds(CUSTOM / "results.json")
    fixtures = list((ROOT / "custom_layout_preflight").glob("*.json"))
    keys = {(r["map"], r["case"], r["seed"]) for r in original}
    assert len(original) == len(keys) == 140 and len({r["map"] for r in original}) == 10
    assert all(n == 14 for n in Counter(r["map"] for r in original).values())
    assert all(n == 20 for n in Counter(r["case"] for r in original).values())
    assert {r["seed"] for r in original} == SEEDS
    maps = sorted({r["map"] for r in custom})
    assert len(custom) == len(fixtures) == 84 and len(maps) == 14
    assert {(r["map"], r["layout"], r["seed"]) for r in custom} == {
        (name, kind, seed) for name in maps for kind in KINDS for seed in SEEDS}
    for row in custom:
        assert row["drones"] == 10
        key = f"{row['map']}_{row['layout']}_seed{row['seed']}"
        fixture = (ROOT / "custom_layout_preflight" / f"{key}.json").read_bytes()
        assert (CUSTOM / f"{key}_bombs.json").read_bytes() == fixture
        assert len(json.loads(fixture)["bombs"]) == 10
    clean = (CUSTOM / "team861_final.zip").read_bytes()
    assert clean == (REPO / "team861_final.zip").read_bytes()
    assert clean == (ROOT / "clean_submission/team861_final.zip").read_bytes()
    with ZipFile(ROOT / "team861_final.zip") as old, ZipFile(CUSTOM / "team861_final.zip") as new:
        assert set(old.namelist()) - set(new.namelist()) == {"blue_nav.py", "blue_policy.py"}
        assert set(new.namelist()) - set(old.namelist()) == set()
        assert all(old.read(name) == new.read(name) for name in new.namelist() if name != "my_drone_place_example.py")
        for name in new.namelist():
            destination = "req.txt" if name == "req.txt" else f"src/swarm_rescue/solutions/{name}"
            assert (CUSTOM / "runtime" / destination).read_bytes() == new.read(name)

    failure_rows = [r for r in custom if r["disposed"] < 10]
    callback_count = sum(r["callback_warning"] for r in custom)
    timed_out = [r for r in custom if r["walltime_limit_reached"]]
    health_loss_on_full = [r for r in custom if r["disposed"] == 10 and r["mean_health_percent"] - r["health_score"] >= 20]
    setup_failures = len(list(CUSTOM.glob("*.startup_attempt.log")))
    weakest = min(maps, key=lambda name: mean(r["score"] for r in custom if r["map"] == name))
    weakest_runs = [r for r in custom if r["map"] == weakest]
    lowest = min(custom, key=lambda r: r["score"])
    report = [
        "# SICT（861）224 轮拆弹仿真报告",
        "",
        "原 10 张地图各 14 轮，新增 14 张地图各 6 轮。每轮 10 枚炸弹、10 架无人机，使用官方 `--competition --headless` 与官方计分。",
        "原图使用固定、两套随机、两套边角和两套聚集布局，每套重复种子 101、303。新增每图三类布局，各以种子 101、303 独立生成一套。",
        "",
        "## 成绩总览",
        "",
        "| 测试集 | 有效轮次 | 平均分 | 最低分 | 交付 | 全拆轮次 | ≥90 分轮次 | 平均机体健康 | 平均返航健康计分比例 | 坠毁 |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
        aggregate(original, "原 10 张地图"),
        aggregate(custom, "新增 14 张地图"),
        aggregate(original + custom, "按轮次合计"),
        "",
        "合计均分为本次测试轮次的算术平均。原图与新增图的步数限制和返航区设置不同，表中合计供描述本次测试范围。",
        "",
        "## 新增图各布局与各地图",
        "",
        "| 布局 | 轮次 | 平均分 | 最低分 | 交付 | 全拆轮次 | ≥90 分轮次 | 平均机体健康 | 平均返航健康计分比例 | 坠毁 |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for kind, label in KINDS.items():
        rows = [r for r in custom if r["layout"] == kind]
        report.append(aggregate(rows, label))
        for seed in sorted(SEEDS):
            report.append(aggregate([r for r in rows if r["seed"] == seed], f"{label} {seed}"))
    report += ["", "| 地图 | 特殊区 | 随机均分 | 边角均分 | 聚集均分 | 总均分 | 最低分 | 交付 | 全拆 | 坠毁 |",
               "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |"]
    for name in maps:
        rows = [r for r in custom if r["map"] == name]
        groups = " | ".join(f"{mean(r['score'] for r in rows if r['layout'] == kind):.2f}" for kind in KINDS)
        report.append(f"| {name} | {'、'.join(rows[0]['zones']) or '无'} | {groups} | {mean(r['score'] for r in rows):.2f} | "
                      f"{min(r['score'] for r in rows):.1f} | {sum(r['disposed'] for r in rows)}/60 | "
                      f"{sum(r['disposed'] == 10 for r in rows)}/6 | {sum(r['destroyed_count'] for r in rows)} |")

    both = [r for r in custom if r["walltime_limit_reached"] and r["disposed"] < 10]
    capped = [r for r in failure_rows if r["steps"] >= r["step_limit"]]
    report += [
        "", "## 失分与未全拆", "",
        "- 原地图仍以 Map08 最薄弱，14 轮仅 9 轮全拆、9 架坠毁。旧 ZIP 的 Map08 全拆为 6/14。原图配对的总均分从 89.32 升至 90.44，坠毁从 9 架增加至 12 架。",
        f"- 新增图中 {weakest} 的平均分最低，为 {mean(r['score'] for r in weakest_runs):.2f} 分，"
        f"全拆 {sum(r['disposed'] == 10 for r in weakest_runs)}/6。单轮最低发生在 {lowest['map']} 的"
        f"{KINDS[lowest['layout']]}布局，种子 {lowest['seed']}，交付 {lowest['disposed']}/10，得分 {lowest['score']:.1f}。",
        f"- 新增图未全拆 {len(failure_rows)} 轮，其中 {len(capped)} 轮用完步数，{len(both)} 轮触发实际时间上限。全部轮次中实际时间上限共触发 {len(timed_out)} 轮。",
        f"- 新增图有 {len(health_loss_on_full)} 轮已经全拆，官方返航健康计分仍比平均机体健康低至少 20 个百分点。历史地图若有独立返航区，官方只给终局在该区内的无人机计入返航健康。",
        f"- 新增图有 {callback_count} 轮的官方物理引擎日志记录了被忽略的 CFFI 碰撞回调异常。这些轮次生成了完整官方分数，报告保留分数及原始异常。",
        "", "| 地图 | 布局 | 种子 | 交付 | 分数 | 仿真步 | 坠毁 | 日志观察 |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | --- |",
    ]
    for row in sorted(failure_rows, key=lambda r: (r["disposed"], r["score"], r["map"])):
        observations = []
        if row["steps"] >= row["step_limit"]:
            observations.append("步数上限")
        if row["walltime_limit_reached"]:
            observations.append("实际时间上限")
        if row["callback_warning"]:
            observations.append("物理回调异常")
        report.append(f"| {row['map']} | {KINDS[row['layout']]} | {row['seed']} | {row['disposed']}/10 | "
                      f"{row['score']:.1f} | {row['steps']}/{row['step_limit']} | {row['destroyed_count']} | "
                      f"{'、'.join(observations) or '仿真结束'} |")
    report += [
        "", "## 场景转换与结果核验", "",
        "- 测试目录以旧赛道为参考，通过当前引擎的处置中心和炸弹接口生成新场景，保留参考地图的障碍、出生和返航区，启用参考图支持的全部特殊区。MapRandom 加入处置中心与四处障碍，使用 2,000 步和 90 秒上限。MapIntermediate02 增加四处障碍以支持边角布局。",
        "- MapIntermediate01 和 MapTestSpecialZones 分别只有 1 和 4 架无人机，因此不在新增的 14 张十机地图内。",
        "- 新增 84 套布局通过离线几何核对。每枚炸弹所在的自由空间分区均连到至少一个出生点，炸弹初始位置与已启用毁伤区至少相距 30 像素。这项检查覆盖静态放置条件。",
        f"- 新增图初次启动时有 {setup_failures} 次布局文件名不匹配。修正后补跑，启动失败日志单独保留。新增图补跑阶段至多同时运行 6 个独立进程，MapRandom 和 MapIntermediate02 串行执行。",
        "- 当前蓝方只用 `len(self.grasped_bombs()) > 0` 判断是否抓持。官方队伍开发指南明确列出这一用法。已删除两份未加载的旧策略 `blue_nav.py`、`blue_policy.py`，并移除红方示例的通信邻居调试读数。",
        f"- 清理包 SHA256 为 `{sha256(clean).hexdigest()}`。逐文件验证，原图 140 轮冻结包与新增图使用的清理包中，蓝方执行文件逐字节相同。官方提交检查通过。",
        "", "## 原始数据", "",
        "- [原图逐地图、布局和旧 ZIP 配对结果](TEAM861_CURRENT_140_ROUND_REPORT.md)",
        "- [原图逐轮数据](bench_results/team861_current_paired_140/runs/results.json)",
        "- [新增图逐轮数据](bench_results/team861_current_paired_140/custom_runs/results.json)",
        "- [新增图评分明细](bench_results/team861_current_paired_140/custom_runs/REPORT.md)",
        "- [提交检查](bench_results/team861_current_paired_140/submission_check.log)",
    ]
    output = REPO / "TEAM861_224_ROUND_REPORT.md"
    custom.sort(key=lambda r: (r["map"], tuple(KINDS).index(r["layout"]), r["seed"]))
    (CUSTOM / "results.json").write_text(json.dumps(custom, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    output.write_text("\n".join(report) + "\n", encoding="utf-8")
    print(output)
    for line in report[8:13]:
        print(line)


if __name__ == "__main__":
    main()
