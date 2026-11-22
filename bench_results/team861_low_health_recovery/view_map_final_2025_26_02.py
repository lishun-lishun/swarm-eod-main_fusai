"""Open the archived ten-drone case in the official simulator window."""
import json
from pathlib import Path
import sys

repo = Path(__file__).resolve().parents[2]
suite = repo / "bench_results/team861_current_paired_140"
sys.path.insert(0, str(repo / "bench_tools"))

from seeded_official_launcher import main
from swarm_rescue.maps.map_final_2025_26_02 import MapFinal_2025_26_02

name = "MapFinal_2025_26_02"
seed = 101
fixture = suite / "custom_layout_preflight" / f"{name}_cluster_seed{seed}.json"
bombs = json.loads(fixture.read_bytes())["bombs"]


def arena(**kwargs):
    scene = MapFinal_2025_26_02(seed=seed, **kwargs)
    scene.spawn_bombs(bombs)
    assert scene.number_bombs == scene.number_drones == 10
    return scene


if __name__ == "__main__":
    sys.argv = [sys.argv[0], str(seed), "--competition", "--config", str(suite / "custom_runs" / f"{name}.yml")]
    main({name: arena})
