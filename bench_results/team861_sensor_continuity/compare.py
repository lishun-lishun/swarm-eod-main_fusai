"""Join rounds to the preceding live strategy by exact map, layout and seed."""
import argparse
import csv
import json
from pathlib import Path
from statistics import mean

OUT = Path(__file__).resolve().parent
REF = OUT.parent / "team861_generic_recovery"
FIELDS = ("map", "case", "seed")
METRICS = ("disposed", "destroyed_count", "score", "steps", "mean_health_percent")


def rows(path):
    return json.loads((path / "results.json").read_bytes())


def key(row):
    return tuple(row[field] for field in FIELDS)


def totals(rounds):
    return dict(rounds=len(rounds), delivered=sum(r["disposed"] for r in rounds),
                lost=sum(r["destroyed_count"] for r in rounds),
                full=sum(r["disposed"] == 10 for r in rounds),
                no_crash=sum(r["destroyed_count"] == 0 for r in rounds),
                full_safe=sum(r["disposed"] == 10 and r["destroyed_count"] == 0 for r in rounds),
                mean_score=round(mean(r["score"] for r in rounds), 4))


def compare(rounds, references):
    originals = [references[key(r)] for r in rounds]
    changes = {"regressed": [], "improved": [], "lower_score": []}
    for r, b in zip(rounds, originals):
        if r["disposed"] < b["disposed"] or r["destroyed_count"] > b["destroyed_count"]:
            changes["regressed"].append(key(r))
        if r["disposed"] > b["disposed"] or r["destroyed_count"] < b["destroyed_count"]:
            changes["improved"].append(key(r))
        if r["score"] < b["score"]:
            changes["lower_score"].append(key(r))
    return {"base": totals(originals), "trial": totals(rounds), **changes}


def verify(r):
    if r["exit_code"] != 0 or r["score"] is None or r["total_bombs"] != 10:
        raise ValueError(f'Incomplete official result: {r["log"]}')
    if not Path(r["log"]).is_file():
        raise FileNotFoundError(r["log"])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("folders", nargs="*")
    parser.add_argument("--export", action="store_true")
    parser.add_argument("--repeats", type=int, default=0, help="Compare repeated base/candidate runs")
    args = parser.parse_args()
    if args.repeats:
        replicate_rows = []
        for iteration in range(1, args.repeats + 1):
            baseline = rows(OUT / f"paired_base_{iteration}")
            candidate = rows(OUT / f"paired_cross_{iteration}")
            for row in baseline + candidate:
                verify(row)
                if row["walltime_limit_reached"]:
                    raise ValueError(f'Official clock limit: {row["log"]}')
            ref = {key(r): r for r in baseline}
            assert set(ref) == {key(r) for r in candidate}
            print(f"repeat {iteration}", json.dumps(compare(candidate, ref), ensure_ascii=False))
            for r in candidate:
                item = {"repeat": iteration, **dict(zip(FIELDS, key(r)))}
                base = ref[key(r)]
                item.update({f"base_{f}": base[f] for f in (*METRICS, "log")})
                item.update({f"trial_{f}": r[f] for f in (*METRICS, "log")})
                replicate_rows.append(item)
        with (OUT / "paired_repeats.csv").open("w", encoding="utf-8-sig", newline="") as file:
            writer = csv.DictWriter(file, fieldnames=list(replicate_rows[0]))
            writer.writeheader()
            writer.writerows(sorted(replicate_rows, key=lambda r: (r["map"], r["case"], r["repeat"])))
        return
    groups = {"困难布局": rows(REF / "confirmed_all_maps"), "不同布局": rows(REF / "distinct_candidate")}
    assert sum(map(len, groups.values())) == 48
    if (OUT / "new_baseline/results.json").is_file():
        extra = rows(OUT / "new_baseline")
        if args.export:
            manifest = json.loads((OUT / "unseen_layouts.json").read_bytes())
            assert {Path(r["log"]).stem for r in extra} == set(manifest)
            for r in extra:
                verify(r)
                assert not r["walltime_limit_reached"]
            assert json.loads((OUT / "new_baseline/source_sha256.json").read_bytes()) == json.loads(
                (REF / "confirmed_all_maps/source_sha256.json").read_bytes())
        groups["第三种布局"] = extra
    ref = {key(r): r for group in groups.values() for r in group}
    assert len(ref) == sum(map(len, groups.values())) and len({r["map"] for r in ref.values()}) == 24
    group_name = {key(r): title for title, group in groups.items() for r in group}
    folders = [OUT / name for name in args.folders] or sorted(p.parent for p in OUT.glob("*/results.json"))
    paired = {}
    source = None
    for folder in folders:
        official = rows(folder)
        invalid = [r for r in official if r["exit_code"] or r["score"] is None]
        finished = [r for r in official if r["exit_code"] == 0 and r["score"] is not None]
        for row in finished:
            verify(row)
        limited = [r for r in finished if r["walltime_limit_reached"]]
        clean = [r for r in finished if not r["walltime_limit_reached"]]
        if invalid:
            print("process did not return a full score", [(key(r), r["exit_code"]) for r in invalid])
        if clean:
            print(folder.name, json.dumps(compare(clean, ref), ensure_ascii=False))
        if limited:
            print("wall-clock limit", [key(r) for r in limited])
        if args.export:
            fingerprint = json.loads((folder / "source_sha256.json").read_bytes())
            if source is None:
                source = fingerprint
            assert source == fingerprint
            for r in clean:
                if key(r) in paired:
                    assert all(paired[key(r)][f] == r[f] for f in METRICS)
                paired[key(r)] = r
    if not args.export:
        return
    assert set(paired) == set(ref)
    records = []
    summary = {}
    for title, base in groups.items():
        batch = [paired[key(b)] for b in base]
        summary[title] = compare(batch, ref)
    summary["合计"] = compare(list(paired.values()), ref)
    for k, r in sorted(paired.items()):
        b = ref[k]
        item = dict(group=group_name[k], **dict(zip(FIELDS, k)))
        for f in METRICS:
            item["base_" + f], item["trial_" + f] = b[f], r[f]
        item["base_log"], item["trial_log"] = b["log"], r["log"]
        records.append(item)
    (OUT / "paired_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf8")
    with (OUT / "paired_rounds.csv").open("w", encoding="utf-8-sig", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    print(json.dumps(summary, ensure_ascii=False))


if __name__ == "__main__":
    main()
