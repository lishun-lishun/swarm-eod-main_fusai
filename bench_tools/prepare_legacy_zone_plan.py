"""Generate fixed, randomly placed overlays for the historical benchmark."""
import argparse
import ast
import json
from math import sqrt
from pathlib import Path
from random import Random

import yaml

from run_survival_benchmark import REFERENCE


def describe(path):
    tree = ast.parse(path.read_text(encoding="utf-8-sig"))
    size = None
    templates = {"NoGpsZone": [], "NoComZone": [], "KillZone": []}
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and any(
            isinstance(t, ast.Attribute) and t.attr == "_size_area" for t in node.targets
        ):
            size = ast.literal_eval(node.value)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id in templates:
            arg = next((kw.value for kw in node.keywords if kw.arg == "size"), None)
            if arg is not None:
                templates[node.func.id].append(ast.literal_eval(arg))
    name = next((item.name for item in tree.body if isinstance(item, ast.ClassDef)), None)
    return name, size, templates


def make_zones(name, size, templates, max_fraction):
    width, height = size
    rng = Random(f"historical-zone-overlay-20261122/{name}")
    zones = []
    for kind, source in (("gps", "NoGpsZone"), ("com", "NoComZone")):
        reference = source if templates[source] else "KillZone" if templates["KillZone"] else "map_fraction"
        dimensions = templates.get(reference) or [(round(width * .20), round(height * .23))]
        for i in range(2):
            base_w, base_h = dimensions[i % len(dimensions)]
            area = 2 * base_w * base_h
            w = min(round(base_w * sqrt(2)), width - 2)
            h = min(round(area / w), height - 2)
            if h == height - 2:
                w = min(round(area / h), width - 2)
            zones.append(dict(kind=kind, size=[w, h], reference=reference, reference_size=[base_w, base_h]))
    area_total = sum(w * h for w, h in (zone["size"] for zone in zones))
    cap = int(width * height * max_fraction) if max_fraction is not None else area_total
    scale = min(1.0, sqrt(cap / area_total))
    for zone in zones:
        w, h = (int(dimension * scale) for dimension in zone["size"])
        if min(w, h) <= 50:
            raise ValueError(f"{name}: region too small for the simulator hit box")
        zone["size"] = [w, h]
        zone["center"] = [round(rng.uniform(-(width - w) / 2, (width - w) / 2), 2),
                          round(rng.uniform(-(height - h) / 2, (height - h) / 2), 2)]
        base_w, base_h = zone["reference_size"]
        zone["area_ratio"] = round(w * h / (base_w * base_h), 4)
    assert sum(z["size"][0] * z["size"][1] for z in zones) <= cap
    return zones


def main(output, max_fraction):
    configs = output / "configs"
    configs.mkdir(parents=True, exist_ok=False)
    runtime = REFERENCE / "custom_runs/runtime/src/swarm_rescue/maps"
    fixtures = json.loads((REFERENCE / "custom_runs/results.json").read_bytes())
    names = {row["map"] for row in fixtures}
    assert len(fixtures) == 84 and len(names) == 14
    plans = {}
    coverage = {}
    for source in (REFERENCE / "custom_map_sources").glob("map_*.py"):
        name, size, templates = describe(runtime / source.name)
        if name not in names:
            continue
        zones = plans[name] = make_zones(name, size, templates, max_fraction)
        area = sum(w * h for w, h in (zone["size"] for zone in zones))
        coverage[name] = {"map_size": size, "zone_area_sum": area, "fraction": area / (size[0] * size[1])}
        config = yaml.safe_load((REFERENCE / "custom_runs" / f"{name}.yml").read_bytes())
        config["evaluation_plan"][0]["zones_config"] = []
        (configs / f"{name}.yml").write_text(yaml.safe_dump(config), encoding="utf-8")
    assert set(plans) == names
    (output / "zones.json").write_text(json.dumps(plans, indent=2, sort_keys=True), encoding="utf-8")
    (output / "coverage.json").write_text(json.dumps(coverage, indent=2, sort_keys=True), encoding="utf-8")
    (output / "no_zones.json").write_text(json.dumps(dict.fromkeys(plans, [])), encoding="utf-8")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    parser.add_argument("--max-fraction", type=float)
    args = parser.parse_args()
    if args.max_fraction is not None and not 0 < args.max_fraction <= 1:
        parser.error("--max-fraction must be in (0, 1]")
    main(args.output, args.max_fraction)
