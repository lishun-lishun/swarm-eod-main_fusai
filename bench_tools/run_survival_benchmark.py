"""Replay the 224-round fixtures against an isolated copy of the current strategy."""

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from hashlib import sha256
import json
from pathlib import Path, PurePosixPath, PureWindowsPath
import shutil
import subprocess
import sys

from run_benchmarks import parse_log, write_results
from seeded_official_launcher import main as launch_seeded


REPO = Path(__file__).resolve().parents[1]
REFERENCE = REPO / "bench_results/team861_current_paired_140"


def archived_path(value: str) -> Path:
    """Resolve an archived file after cloning the project at another path."""
    portable = PurePosixPath(PureWindowsPath(value).as_posix())
    if "bench_results" in portable.parts:
        index = portable.parts.index("bench_results")
        return REPO.joinpath(*portable.parts[index:])
    return Path(value)


def trace_survival():
    from swarm_rescue.solutions.my_drone_rescue import MyDroneRescue

    original = MyDroneRescue.control
    original_merge = MyDroneRescue._merge_bomb_observation

    def merge(drone, **observation):
        key = original_merge(drone, **observation)
        record = drone._known_bombs[key]
        if observation["from_direct_sensor"] and record.get("released_at") == drone._step:
            event = ("reopened", key, drone._step)
        elif observation["status"] == "DONE" and record["status"] != "DONE":
            event = ("ignored_done", key, observation["seen_step"])
        else:
            return key
        seen = getattr(drone, "_bench_task_events", None)
        if seen is None:
            drone._bench_task_events = seen = set()
        if event not in seen:
            seen.add(event)
            print("TASK_OBSERVATION", drone._step, drone.identifier, event, flush=True)
        return key

    def control(drone):
        health = drone.drone_health
        outage = drone.lidar_is_disabled() and drone.odometer_is_disabled()
        previous_health, previous_outage = getattr(drone, "_bench_status", (health, False))
        if outage and not previous_outage:
            drone._bench_outage_start = drone._step
        elif previous_outage and not outage:
            sample = drone.odometer_values()
            distance = float(sample[0]) if sample is not None else None
            print("RECOVERED_ODOM", drone._step, drone.identifier, drone._step - drone._bench_outage_start,
                  distance, round(drone.pose.vx, 2), round(drone.pose.vy, 2), flush=True)
        if outage != previous_outage or previous_health != health or drone._step % 300 == 0:
            x, y = round(drone.pose.x), round(drone.pose.y)
            print("OBS", drone._step, drone.identifier, health, int(outage), drone._state,
                  x, y, round(drone.pose.vx, 1), round(drone.pose.vy, 1), drone._local_disposed, flush=True)
        drone._bench_status = (health, outage)
        old_points = set(getattr(drone._omap, "outage_points", ()))
        old_crossing = getattr(drone, "_last_crossing", None)
        old_recovery = getattr(drone, "_contact_recovery", None)
        had_gps_outage = drone._had_gps_outage
        command = original(drone)
        crossing = getattr(drone, "_last_crossing", None)
        if crossing != old_crossing:
            print("CROSSING", drone._step, drone.identifier, crossing, flush=True)
        if drone._step <= 3 or drone._step % 200 == 0:
            print("ACT", drone._step, drone.identifier, round(drone.pose.x, 4),
                  round(drone.pose.y, 4), tuple(round(float(v), 5) for v in command.values()), flush=True)
        if drone._had_gps_outage and not had_gps_outage:
            print("GPS_OUTAGE", drone._step, drone.identifier, flush=True)
        new_points = set(getattr(drone._omap, "outage_points", ())) - old_points
        if new_points:
            print("LEARNED", drone._step, drone.identifier, sorted(new_points), flush=True)
        recovery = getattr(drone, "_contact_recovery", None)
        if recovery is not None and recovery != old_recovery:
            print("CONTACT_RECOVERY", drone._step, drone.identifier, health, drone._state, recovery, flush=True)
        return command

    MyDroneRescue._merge_bomb_observation = merge
    MyDroneRescue.control = control


def replay(args):
    if args.opencv_threads is not None:
        import cv2

        cv2.setNumThreads(args.opencv_threads)
    if args.trace:
        trace_survival()
    map_classes = None
    flags = [str(args.seed), "--competition", "--headless", "--config", args.config]
    if args.module:
        from importlib import import_module

        cls = getattr(import_module(f"swarm_rescue.maps.{args.module}"), args.map)
        bombs = json.loads(Path(args.fixture).read_bytes())["bombs"]

        def map_class(**kwargs):
            arena = cls(seed=args.seed, **kwargs)
            arena.spawn_bombs(bombs)
            assert arena.number_bombs == arena.number_drones == 10
            if args.legacy_zone_plan:
                from swarm_rescue.simulation.elements.sensor_disablers import KillZone, NoComZone, NoGpsZone

                plan = json.loads(args.legacy_zone_plan.read_bytes())[args.map]
                types = {"gps": NoGpsZone, "com": NoComZone}
                for zone in plan:
                    arena.playground.add(types[zone["kind"]](size=tuple(zone["size"])),
                                         (tuple(zone["center"]), 0))
                entities = list(arena.playground.elements)
                assert not any(isinstance(e, KillZone) for e in entities)
                counts = {kind: sum(isinstance(e, cls_type) for e in entities)
                          for kind, cls_type in types.items()}
                if plan:
                    assert counts == {"gps": 2, "com": 2}
                print(f"BENCH_ZONES {args.map} {json.dumps({'counts': counts, 'overlay': plan})}", flush=True)
            return arena

        map_classes = {args.map: map_class}
    else:
        flags += ["--bombs-file", args.fixture]
    sys.argv = [sys.argv[0], *flags]
    launch_seeded(map_classes)


def execute(case, output, runtime, env, modules, trace, opencv_threads, legacy_zone_plan):
    custom = "layout" in case
    if not custom:
        legacy_zone_plan = None
    name, seed = case["map"], case["seed"]
    layout = case["layout"] if custom else case["case"]
    key = f"{name}_{layout}_seed{seed}"
    config = REFERENCE / (f"custom_runs/{name}.yml" if custom else f"runs/config_{name}.yml")
    if legacy_zone_plan:
        config = legacy_zone_plan.parent / "configs" / f"{name}.yml"
    fixture = (
        REFERENCE / f"custom_layout_preflight/{key}.json"
        if custom else REFERENCE / "cases" / PureWindowsPath(case["bombs_file"]).name
    )
    cmd = [sys.executable, __file__, "--child", "--map", name, "--seed", str(seed),
           "--config", str(config), "--fixture", str(fixture)]
    if custom:
        cmd += ["--module", modules[name]]
    if trace:
        cmd.append("--trace")
    if opencv_threads is not None:
        cmd += ["--opencv-threads", str(opencv_threads)]
    if legacy_zone_plan:
        cmd += ["--legacy-zone-plan", str(legacy_zone_plan)]
    log = output / f"{key}.log"
    with log.open("wb") as file:
        try:
            code = subprocess.run(cmd, cwd=runtime, env=env, stdout=file, stderr=subprocess.STDOUT,
                                  timeout=1500 if custom else 300).returncode
        except subprocess.TimeoutExpired:
            code = -9
    raw = log.read_text(encoding="utf-8", errors="replace")
    result = parse_log(raw)
    if result["destroyed_percent"] is not None:
        result["destroyed_count"] = round(result["destroyed_percent"] / 10)
    result["walltime_limit_reached"] = "is reached first" in raw
    return {"map": name, "case": layout, "seed": seed, "exit_code": code, "log": str(log), **result,
            "baseline_disposed": case["disposed"], "baseline_destroyed": case["destroyed_count"],
            "baseline_score": case["score"]}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path)
    parser.add_argument("--strategy", type=Path, default=REPO / "src/swarm_rescue/solutions")
    parser.add_argument("--workers", type=int, default=6)
    parser.add_argument("--only", nargs="+", help="Select exact existing log stems")
    parser.add_argument("--one-per-map", action="store_true", help="Replay the weakest baseline round on each map")
    parser.add_argument("--trace", action="store_true")
    parser.add_argument("--opencv-threads", type=int, help="Set OpenCV threads in each isolated evaluation process")
    parser.add_argument("--legacy-zone-plan", type=Path, help="Override historical-map zones for this benchmark")
    parser.add_argument("--include-stock", action="store_true", help="Run the original stock cases with a historical overlay")
    parser.add_argument("--child", action="store_true")
    parser.add_argument("--map")
    parser.add_argument("--seed", type=int)
    parser.add_argument("--module")
    parser.add_argument("--fixture")
    parser.add_argument("--config")
    args = parser.parse_args()
    if args.legacy_zone_plan:
        args.legacy_zone_plan = args.legacy_zone_plan.resolve()
    if args.child:
        replay(args)
        return

    import ast
    import os

    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    converted = REFERENCE / "custom_runs/runtime"
    runtime = output / "runtime"
    shutil.copytree(converted, runtime, ignore=shutil.ignore_patterns("solutions", "__pycache__"))
    target = runtime / "src/swarm_rescue/solutions"
    target.mkdir()
    source_hashes = {}
    for source in args.strategy.glob("*"):
        if source.is_file():
            body = source.read_bytes()
            (target / source.name).write_bytes(body)
            source_hashes[source.name] = sha256(body).hexdigest()
    (output / "source_sha256.json").write_text(json.dumps(source_hashes, indent=2), encoding="utf-8")
    modules = {}
    for path in (REFERENCE / "custom_map_sources").glob("map_*.py"):
        tree = ast.parse(path.read_text(encoding="utf-8-sig"))
        for item in tree.body:
            if isinstance(item, ast.ClassDef):
                modules[item.name] = path.stem
                break
    official = json.loads((REFERENCE / "runs/results.json").read_bytes())
    custom = json.loads((REFERENCE / "custom_runs/results.json").read_bytes())
    cases = official + custom
    if args.legacy_zone_plan and not args.include_stock:
        cases = custom
    if args.only:
        by_key = {PureWindowsPath(case["log"]).stem: case for case in cases}
        cases = [by_key[key] for key in args.only]
    elif args.one_per_map:
        by_result = sorted(cases, key=lambda case: (case["disposed"], -case["destroyed_count"], case["score"]))
        by_map = {}
        for case in by_result:
            by_map.setdefault(case["map"], case)
        cases = list(by_map.values())
    env = dict(os.environ, PYTHONPATH=str(runtime / "src"), PYTHONIOENCODING="utf-8")
    rows = []

    def submit(pool, group):
        return [
            pool.submit(execute, case, output, runtime, env, modules, args.trace, args.opencv_threads,
                        args.legacy_zone_plan)
            for case in group
        ]

    def record(row):
        rows.append(row)
        temp = output / "results.tmp"
        write_results(temp, rows)
        temp.replace(output / "results.json")
        print(f"[{len(rows)}/{len(cases)}] {row['map']} {row['case']} {row['seed']} "
              f"{row['baseline_disposed']}->{row['disposed']} rescued, "
              f"{row['baseline_destroyed']}->{row['destroyed_count']} lost, {row['score']} score exit={row['exit_code']}",
              flush=True)

    # Short wall-time fixtures run alone to prevent load from deciding the score.
    serial = [case for case in cases if case["map"] in {"Map01", "MapRandom", "MapIntermediate02"}]
    stock = [case for case in cases if "layout" not in case and case["map"] != "Map01"]
    legacy = [case for case in cases if "layout" in case and case["map"] not in {"MapRandom", "MapIntermediate02"}]
    for group, workers in ((stock, min(3, args.workers)), (legacy, args.workers)):
        with ThreadPoolExecutor(max_workers=workers) as pool:
            for future in as_completed(submit(pool, group)):
                record(future.result())
    for case in serial:
        record(execute(case, output, runtime, env, modules, args.trace, args.opencv_threads,
                       args.legacy_zone_plan))


if __name__ == "__main__":
    main()
