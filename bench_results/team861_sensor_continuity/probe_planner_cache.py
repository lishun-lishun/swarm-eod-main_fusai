"""Verify A* cost and paths before and after walls and shared coverage change."""
import importlib.util
from pathlib import Path
from statistics import median
from time import perf_counter

import numpy as np

OUT = Path(__file__).resolve().parent
BASE = OUT.parent / "team861_generic_recovery/confirmed_all_maps/runtime/src/swarm_rescue/solutions"


def nav_map(source):
    spec = importlib.util.spec_from_file_location("nav_probe", source / "nav_grid.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.OnlineMap((960, 640))


old, new = nav_map(BASE), nav_map(OUT / "planner_cache")
routes = [((-400.0, float(y)), (400.0, float(z))) for y in (-200, 0, 200) for z in (-180, 0, 180)]


def plan(nav):
    return [nav.plan_path(start, goal, carrying=carry, max_expansions=2500, allow_fallback=False)
            for start, goal in routes for carry in (False, True)]


for stage in range(3):
    for nav in (old, new):
        if stage != 1:
            col = nav.gw // 2 + stage * 4
            nav.wall_score[8:nav.gh - 8, col:col + 3] = 10
            nav.free_seen[25:110, :col] = True
            nav._dirty = True
            nav.ensure_fresh(step=10 + stage * 10)
        else:
            highway = np.zeros_like(nav._peer_highway_cells)
            highway[-6:, :] = True
            nav.merge_peer_highway(highway)
    assert plan(old) == plan(new), stage
    reference = [cost * (0.88 if seen or highway else 1.28) for cost, seen, highway
                 in zip(old._p_cost_list, old._p_seen_b, old._p_highway_b)]
    assert reference == new._p_travel_cost, stage

times = [[], []]
for _ in range(5):
    for nav, samples in zip((old, new), times):
        start = perf_counter()
        plan(nav)
        samples.append(perf_counter() - start)

baseline, candidate = (median(samples) for samples in times)
print("All cell costs and navigation results matched across 54 plans and two invalidation paths.")
print(f"Median for 18 A* calls on the same grid (s): {baseline:.4f} -> {candidate:.4f}.")
