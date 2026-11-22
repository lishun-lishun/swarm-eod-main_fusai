"""Summarize exact-layout pairs against the current 224 official results."""
from collections import defaultdict
import csv
from hashlib import sha256
import json
from pathlib import Path
from statistics import mean
import sys

HERE = Path(__file__).resolve().parent
PREVIOUS = HERE.parent / "team861_full_224_current"


def rows(folder):
    return json.loads((folder / "results.json").read_bytes())


def key(row):
    return row["map"], row["case"], row["seed"]


def valid(row):
    return row["exit_code"] == 0 and row["score"] is not None and not row["walltime_limit_reached"]


def metrics(batch):
    return {
        "n": len(batch),
        "delivered": sum(r["disposed"] for r in batch),
        "lost": sum(r["destroyed_count"] for r in batch),
        "full": sum(r["disposed"] == 10 for r in batch),
        "safe_full": sum(r["disposed"] == 10 and r["destroyed_count"] == 0 for r in batch),
        "score": round(mean(r["score"] for r in batch), 4),
        "min_score": min(r["score"] for r in batch),
    }


def compare(folder):
    baseline = {key(r): r for r in rows(PREVIOUS)}
    grouped = defaultdict(list)
    table = []
    missing = []
    for r in sorted(rows(folder), key=key):
        if not valid(r):
            missing.append(Path(r["log"]).stem)
            continue
        b = baseline[key(r)]
        grouped[r["map"]].append((b, r))
        entry = dict(zip(("map", "case", "seed"), key(r)))
        for f in ("disposed", "destroyed_count", "score", "steps", "mean_health_percent", "log"):
            entry["base_" + f], entry["trial_" + f] = b[f], r[f]
        table.append(entry)
    if not table:
        return {"pending_replay": missing}
    with (folder / "paired.csv").open("w", encoding="utf-8-sig", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(table[0]))
        writer.writeheader()
        writer.writerows(table)
    both = [pair for group in grouped.values() for pair in group]
    before, after = zip(*both)
    report = {"current": metrics(before), "candidate": metrics(after), "pending_replay": missing,
              "by_map": {m: [metrics(b), metrics(a)] for m, pairs in grouped.items()
                         for b, a in [zip(*pairs)]}}
    report["changed"] = [entry for entry in table if any(entry["base_" + f] != entry["trial_" + f]
                          for f in ("disposed", "destroyed_count", "score"))]
    (folder / "comparison.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    return report


for folder in sys.argv[1:]:
    target = HERE / folder
    fingerprint = json.loads((target / "source_sha256.json").read_bytes())
    for name, content in fingerprint.items():
        assert sha256((target / "runtime/src/swarm_rescue/solutions" / name).read_bytes()).hexdigest() == content
    result = compare(target)
    print(folder, json.dumps({k: v for k, v in result.items() if k not in ("changed", "by_map")}))
    for entry in result.get("changed", ()):
        a, b = [[entry[prefix + f] for f in ("disposed", "destroyed_count", "score")]
                for prefix in ("base_", "trial_")]
        print(" ", entry["map"], entry["case"], entry["seed"], a, "->", b)
