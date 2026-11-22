"""Combine the existing five layouts with one clustered layout per map."""

import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from generate_eval_cases import MAPS, ZONES, _geometry


EXISTING_GROUPS = (
    ("bench_cases_user_v6", "user_fixed"),
    ("bench_cases_random_triplets", "random_s11"),
    ("bench_cases_random_triplets", "random_s29"),
    ("bench_cases_hard_100_105", "edge_seed100"),
    ("bench_cases_hard_100_105", "edge_seed105"),
)
CLUSTER_SEED = 618
CLUSTER_RADIUS = 72
MIN_SEPARATION = 32


def cluster_positions(legal, rng, other_positions):
    height, width = legal.shape
    legal[:30, :] = False
    legal[-30:, :] = False
    legal[:, :30] = False
    legal[:, -30:] = False
    ys, xs = np.nonzero(legal)
    radius = CLUSTER_RADIUS
    for index in rng.permutation(len(xs))[:600]:
        cx, cy = int(xs[index]), int(ys[index])
        x0, x1 = max(0, cx - radius), min(width, cx + radius + 1)
        y0, y1 = max(0, cy - radius), min(height, cy + radius + 1)
        dy, dx = np.ogrid[y0 - cy:y1 - cy, x0 - cx:x1 - cx]
        disk = (dx * dx + dy * dy <= radius * radius) & legal[y0:y1, x0:x1]
        connected = disk.astype(np.uint8)
        cv2.floodFill(connected, None, (cx - x0, cy - y0), 2)
        vy, vx = np.nonzero(connected[::4, ::4] == 2)
        for _ in range(3):
            picked = []
            for j in rng.permutation(len(vx)):
                x, y = x0 + 4 * int(vx[j]), y0 + 4 * int(vy[j])
                if (x - width / 2, height / 2 - y) in other_positions:
                    continue
                if all((x - px) ** 2 + (y - py) ** 2 >= MIN_SEPARATION ** 2 for px, py in picked):
                    picked.append((x, y))
                    if len(picked) == 10:
                        return (cx, cy), picked
    raise RuntimeError("Could not place ten separated bombs around a single center")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)

    rows = []
    used_positions = {name: set() for name in MAPS}
    for directory, kind in EXISTING_GROUPS:
        source = json.loads((Path(directory) / "manifest.json").read_text(encoding="utf-8"))
        for name in MAPS:
            case = next(row for row in source if row["map"] == name and row["case"] == kind)
            path = Path(case["path"]).resolve()
            content = path.read_bytes()
            layout = json.loads(content)
            positions = {(bomb["x"], bomb["y"]) for bomb in layout["bombs"]}
            if len(layout["bombs"]) != 10 or len(positions) != 10:
                raise ValueError(f"Invalid bomb count in {path}")
            used_positions[name].update(positions)
            rows.append({**case, "path": str(path), "bombs_sha256": hashlib.sha256(content).hexdigest()})

    kind = f"cluster_s{CLUSTER_SEED}"
    for map_index, name in enumerate(MAPS):
        _, legal = _geometry(Path("map_data") / f"map_{name[3:].lower()}.json")
        height, width = legal.shape
        rng = np.random.default_rng(np.random.SeedSequence([CLUSTER_SEED, map_index]))
        (cx, cy), pixels = cluster_positions(legal, rng, used_positions[name])
        bombs = [{"x": x - width / 2, "y": height / 2 - y, "theta": 0.0} for x, y in pixels]
        path = output / f"{name}_{kind}.json"
        path.write_text(json.dumps({
            "map_name": name, "case_kind": kind, "zones_config": ZONES.get(name, []), "bombs": bombs,
        }, indent=2) + "\n", encoding="utf-8")
        rows.append({
            "map": name, "case": kind, "path": str(path), "seed": CLUSTER_SEED,
            "center": [cx - width / 2, height / 2 - cy], "radius": CLUSTER_RADIUS,
            "minimum_separation": MIN_SEPARATION,
            "bombs_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        })

    (output / "manifest.json").write_text(json.dumps(rows, indent=2) + "\n", encoding="utf-8")
    print(f"Prepared {len(rows)} cases for {len(MAPS)} maps in {output}")


if __name__ == "__main__":
    main()
