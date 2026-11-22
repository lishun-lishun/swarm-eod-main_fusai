"""Freeze seven layout groups for paired evaluations of a submission."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from generate_eval_cases import MAPS, ZONES, _geometry, _rect
from prepare_six_group_cases import CLUSTER_RADIUS, MIN_SEPARATION, cluster_positions


def obstacle_corners(data):
    corners = set()
    for element in data["elements"]:
        if element["type"] == "rect_wall":
            (x0, y0), (x1, y1) = _rect(element)
            corners.update(((x0, y0), (x0, y1), (x1, y0), (x1, y1)))
        elif element["type"] == "line_wall":
            corners.update(tuple(p) for p in element["points"])
    if not corners:
        raise ValueError("No internal obstacles")
    return np.asarray(sorted(corners))


def corner_positions(data, legal, rng, used):
    height, width = legal.shape
    corners = obstacle_corners(data)
    y, x = np.nonzero(legal[30:height - 30:4, 30:width - 30:4])
    points = np.column_stack((30 + 4 * x, 30 + 4 * y))
    distance, nearest = cKDTree(corners).query(points)
    for radius in (72, 88, 112):
        picked = []
        uses = np.zeros(len(corners), dtype=int)
        available = np.flatnonzero(distance <= radius)
        while len(picked) < 10 and available.size:
            # Prefer a new corner when one can accommodate another legal bomb.
            counts = uses[nearest[available]]
            trial = available[counts == counts.min()]
            index = int(rng.choice(trial))
            x0, y0 = map(int, points[index])
            if (x0 - width / 2, height / 2 - y0) in used:
                available = available[available != index]
                continue
            picked.append((x0, y0))
            uses[nearest[index]] += 1
            safe_distance = np.sum((points[available] - points[index]) ** 2, axis=1) >= 42 ** 2
            available = available[safe_distance]
        if len(picked) == 10:
            actual, _ = cKDTree(corners).query(picked)
            return picked, float(actual.max()), int(np.count_nonzero(uses))
    raise ValueError("Unable to place ten separated bombs near obstacle corners")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    source = json.loads(Path("bench_results/team861_six_groups_20261118/cases/manifest.json").read_text())
    index = {(r["map"], r["case"]): r for r in source}
    rows = []

    def save(name, kind, layout, metadata):
        path = out / f"{name}_{kind}.json"
        path.write_bytes(layout)
        bombs = json.loads(layout)["bombs"]
        positions = {(b["x"], b["y"]) for b in bombs}
        if len(bombs) != 10 or len(positions) != 10:
            raise ValueError(f"Invalid bomb count in {path}")
        rows.append({"map": name, "case": kind, "path": str(path),
                     "bombs_sha256": hashlib.sha256(layout).hexdigest(), **metadata})
        return positions

    for i, name in enumerate(MAPS):
        data, legal = _geometry(Path("map_data") / f"map_{name[3:].lower()}.json")
        height, width = legal.shape
        used = set()
        for kind in ("user_fixed", "random_s11", "random_s29", "cluster_s618"):
            row = index[name, kind]
            used.update(save(name, kind, Path(row["path"]).read_bytes(),
                             {k: row[k] for k in ("center", "radius", "minimum_separation") if k in row}))
        for kind, seed in (("corner_s100", 100), ("corner_s105", 105), ("cluster_s919", 919)):
            rng = np.random.default_rng(np.random.SeedSequence([seed, i]))
            if kind.startswith("corner"):
                points, distance, count = corner_positions(data, legal, rng, used)
                meta = {"max_corner_distance": distance, "distinct_corners": count, "minimum_separation": 42}
            else:
                (cx, cy), points = cluster_positions(legal.copy(), rng, used)
                meta = {"center": [cx - width / 2, height / 2 - cy],
                        "radius": CLUSTER_RADIUS, "minimum_separation": MIN_SEPARATION}
            meta["layout_seed"] = seed
            bombs = [{"x": x - width / 2, "y": height / 2 - y, "theta": 0.0} for x, y in points]
            layout = {"map_name": name, "case_kind": kind, "zones_config": ZONES.get(name, []), "bombs": bombs}
            used.update(save(name, kind, (json.dumps(layout, indent=2) + "\n").encode(), meta))
    (out / "manifest.json").write_text(json.dumps(rows, indent=2) + "\n", encoding="utf-8")
    print(f"Prepared {len(rows)} layouts in {out}")


if __name__ == "__main__":
    main()
