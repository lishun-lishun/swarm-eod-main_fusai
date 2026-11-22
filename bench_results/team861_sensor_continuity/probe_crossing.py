"""Check that a locally learned hazard affects navigation only and expires."""
import importlib.util
from pathlib import Path

import numpy as np

source = Path(__file__).resolve().parent / "outage_crossing/nav_grid.py"
spec = importlib.util.spec_from_file_location("nav_probe", source)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
first, other = (module.OnlineMap((320, 240)) for _ in range(2))
point = first.plan_to_world(first.ph // 2, first.pw // 2)
cell = first.world_to_cell(*point)
plan = first.world_to_plan(*point)
walls = first.wall_mask.copy()
anchors = first.gps_wall_mask.copy()
assert first.note_blind_crossing(point, 80)
assert not first.note_blind_crossing(point, 70)
first.ensure_fresh(10)
assert first.obs_mask[cell] and not first.plan_free_unloaded[plan]
assert not other.obs_mask[cell] and other.plan_free_unloaded[plan]
assert not first.get_sync_wall_chunk() and np.array_equal(first.wall_mask, walls)
assert np.array_equal(first.gps_wall_mask, anchors)
first.ensure_fresh(71)
assert first.obs_mask[cell]
first.ensure_fresh(81)
assert not first.obs_mask[cell] and first.plan_free_unloaded[plan]
print("Verified local navigation cost, isolation between drones, stable localization walls, and expiry.")
