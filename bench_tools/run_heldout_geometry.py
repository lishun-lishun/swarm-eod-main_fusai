"""Build and evaluate custom maps outside the competition source tree."""

import argparse
import importlib
import json
import os
from pathlib import Path
import subprocess
import sys
from zipfile import ZipFile

import yaml

from generate_eval_cases import _geometry, _select
from run_benchmarks import parse_log, write_results
from seeded_official_launcher import main as launch_seeded


SCENES = {
    "corridors": (
        (1440, 1000),
        (55, 100, 215, 260),
        (245, 45, 345, 195),
        [(380, 0, 415, 740), (745, 265, 785, 1000), (1115, 0, 1155, 750),
         (925, 445, 1060, 490), (530, 670, 640, 710)],
        [],
        241,
    ),
    "junction": (
        (1500, 1080),
        (75, 780, 235, 940),
        (270, 847, 385, 990),
        [(375, 160, 420, 700), (670, 0, 715, 420), (670, 560, 715, 935),
         (1000, 190, 1045, 910), (1230, 390, 1335, 455), (875, 855, 940, 1020),
         (65, 450, 255, 490), (870, 195, 945, 355), (1150, 665, 1260, 710)],
        [("no_gps_zone", (535, 300, 1395, 760)), ("no_com_zone", (790, 55, 1455, 525))],
        181,
    ),
    "rooms": (
        (1320, 1080),
        (65, 830, 225, 990),
        (255, 770, 385, 930),
        [(490, 155, 525, 450), (490, 558, 525, 905), (806, 220, 842, 585),
         (806, 700, 842, 1020), (118, 400, 355, 437), (525, 514, 716, 552),
         (965, 434, 1320, 471), (920, 784, 1070, 835), (640, 840, 735, 899)],
        [("no_gps_zone", (88, 100, 590, 660)), ("no_com_zone", (755, 65, 1255, 605))],
        257,
    ),
}


def rectangle(kind, bounds):
    return {"type": kind, "top_left": bounds[:2], "bottom_right": bounds[2:], "color": "#000000"}


def prepare(root):
    from swarm_rescue.tools.json_to_map import generate_map_from_json

    generated = root / "generated"
    generated.mkdir(parents=True, exist_ok=True)
    for name, (size, spawn, center, walls, zones, seed) in SCENES.items():
        width, height = size
        elements = [rectangle("start_area", spawn), rectangle("disposal_center", center)]
        elements.extend(rectangle("rect_wall", wall) for wall in walls)
        elements.extend(rectangle(kind, bounds) for kind, bounds in zones)
        for i, element in enumerate(elements, 1):
            element["id"] = i
        geometry_path = root / f"{name}_geometry.json"
        geometry_path.write_text(json.dumps({
            "version": 1, "canvas": {"width": width, "height": height}, "elements": elements,
        }, indent=2), encoding="utf-8")
        generate_map_from_json(input_json_path=geometry_path, name=name, maps_dir=generated)
        _, legal = _geometry(geometry_path)
        pixels = _select(legal, seed, edge_bias=False)
        zone_names = [kind.upper() for kind, _ in zones]
        map_name = "Map" + "".join(part.capitalize() for part in name.split("_"))
        bombs = [{"x": x - width / 2, "y": height / 2 - y, "theta": 0.0} for x, y in pixels]
        (root / f"{name}_bombs.json").write_text(json.dumps({
            "map_name": map_name, "zones_config": zone_names, "bombs": bombs,
        }, indent=2), encoding="utf-8")
        (root / f"{name}_config.yml").write_text(yaml.safe_dump({
            "stat_saving_enabled": False, "state_recording_enabled": False,
            "video_capture_enabled": False, "time_best_ratio": 0.3,
            "evaluation_plan": [{"map_name": map_name, "zones_config": zone_names,
                                 "team_mode": "rescue", "number_drones": 10, "nb_rounds": 1}],
        }), encoding="utf-8")


def trace_controller():
    from swarm_rescue.solutions.my_drone_rescue import MyDroneRescue

    control = MyDroneRescue.control

    def traced(drone):
        previously_carrying = drone._was_carrying
        command = control(drone)
        carrying = drone._was_carrying
        last_step = getattr(drone, "_step", 0) == 7200
        if carrying != previously_carrying or (carrying and drone._step % 400 == 0) or last_step:
            print("TRACE", drone._step, drone.identifier, drone._state, carrying,
                  (round(drone.pose.x), round(drone.pose.y)), drone.pose.gps_settled,
                  drone._target_bomb_key, drone._waypoints[:1], drone._local_disposed, flush=True)
        return command

    MyDroneRescue.control = traced


def launch(root, name, seed, archived_sources, trace):
    import swarm_rescue.maps
    import swarm_rescue.solutions

    swarm_rescue.maps.__path__.append(str(root / "generated"))
    if archived_sources is not None:
        swarm_rescue.solutions.__path__.insert(0, str(archived_sources))
    if trace:
        trace_controller()
    module = importlib.import_module(f"swarm_rescue.maps.map_{name}")
    map_name = "Map" + "".join(part.capitalize() for part in name.split("_"))
    sys.argv = [sys.argv[0], str(seed), "--competition", "--headless", "--config",
                str(root / f"{name}_config.yml"), "--bombs-file", str(root / f"{name}_bombs.json")]
    launch_seeded({map_name: getattr(module, map_name)})


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    parser.add_argument("--archive", type=Path)
    parser.add_argument("--child", choices=SCENES)
    parser.add_argument("--seed", type=int, default=401)
    parser.add_argument("--scenes", nargs="+", choices=SCENES, default=list(SCENES))
    parser.add_argument("--trace", action="store_true")
    args = parser.parse_args()
    root = args.output.resolve()
    archived_sources = root / "archive_solutions" if args.archive else None
    if args.child:
        launch(root, args.child, args.seed, archived_sources, args.trace)
        return

    prepare(root)
    if args.archive:
        archived_sources.mkdir(exist_ok=True)
        with ZipFile(args.archive) as archive:
            for name in archive.namelist():
                if Path(name).name == name and name.endswith(".py"):
                    (archived_sources / name).write_bytes(archive.read(name))
    env = os.environ.copy()
    label = "archive" if args.archive else "workspace"
    if args.trace:
        label += "_trace"
    rows = []
    for scene in args.scenes:
        command = [sys.executable, str(Path(__file__).resolve()), str(root), "--child", scene,
                   "--seed", str(args.seed)]
        if args.archive:
            command += ["--archive", str(args.archive.resolve())]
        if args.trace:
            command.append("--trace")
        proc = subprocess.run(command, text=True, encoding="utf-8", errors="replace",
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env, timeout=400)
        log = root / f"{label}_{scene}_{args.seed}.log"
        log.write_text(proc.stdout, encoding="utf-8")
        row = {"scene": scene, "seed": args.seed, "exit_code": proc.returncode, "log": str(log)}
        row.update(parse_log(proc.stdout))
        rows.append(row)
        write_results(root / f"{label}_results_{args.seed}.json", rows)
        print(label, scene, row["disposed"], row["score"], row["steps"], row["exit_code"], flush=True)
        if proc.returncode or row["disposed"] is None:
            raise SystemExit(f"Simulation failed, see {log}")


if __name__ == "__main__":
    main()
