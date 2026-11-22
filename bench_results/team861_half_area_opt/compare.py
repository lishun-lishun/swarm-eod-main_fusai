"""Summarize exact fixture pairs against the 224-round half-area benchmark."""
from collections import defaultdict
import json
from pathlib import Path
from statistics import mean
import sys

ROOT = Path(__file__).resolve().parent
BASELINE = ROOT.parent / "team861_half_area_224/rounds/results.json"


def key(row):
    return row["map"], row["case"], row["seed"]


def totals(rows):
    return (sum(r["disposed"] for r in rows), sum(r["destroyed_count"] for r in rows),
            sum(r["disposed"] == 10 for r in rows), round(mean(r["score"] for r in rows), 3))


old = {key(r): r for r in json.loads(BASELINE.read_bytes())}
for name in sys.argv[1:]:
    file = ROOT / name / "results.json"
    if not file.is_file():
        continue
    raw = json.loads(file.read_bytes())
    clean = [r for r in raw if r["exit_code"] == 0 and r["score"] is not None and not r["walltime_limit_reached"]]
    if not clean:
        print(name, "no complete, uncapped rounds")
        continue
    print(name, len(clean), "/", len(raw), totals([old[key(r)] for r in clean]), "=>", totals(clean))
    for r in clean:
        b = old[key(r)]
        fields = ("disposed", "destroyed_count", "score")
        a, c = tuple(b[f] for f in fields), tuple(r[f] for f in fields)
        if a != c and name != "full":
            print(" ", key(r), a, "=>", c)
    if name == "full":
        groups = defaultdict(list)
        for row in clean:
            groups[row["map"]].append(row)
        for m, rows in sorted(groups.items()):
            print(" ", m, totals([old[key(r)] for r in rows]), "=>", totals(rows))
