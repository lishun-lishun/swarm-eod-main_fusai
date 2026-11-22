"""Adapt legacy map fixtures in the evaluation copy and run the stock launcher."""
import argparse
import ast
from concurrent.futures import ThreadPoolExecutor, as_completed
import importlib
from hashlib import sha256
import json
import os
from pathlib import Path
import re
from statistics import mean
import shutil
import subprocess
import sys
from zipfile import ZipFile

import yaml

from run_benchmarks import parse_log, write_results
from seeded_official_launcher import main as launch_seeded

LAYOUTS = ("random", "corner", "cluster")
EXCLUDED = {"MapIntermediate01", "MapTestSpecialZones"}


def adapt(source, destination):
    original = source.read_text(encoding="utf-8-sig")
    tree = ast.parse(original)
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef))
    zones = sorted(set(re.findall(r"if ZoneType\.(\w+) in self\._zones_config", original)))
    code = original.split("\ndef main(", 1)[0].split("\nif __name__", 1)[0]
    constructor = next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == "__init__")
    lines = code.splitlines(keepends=True)
    for loop in reversed(constructor.body):
        if isinstance(loop, ast.For) and ast.unparse(loop.iter) == "range(self._number_wounded_persons)":
            del lines[loop.lineno - 1:loop.end_lineno]
    code = "".join(lines)
    for before, after in (
        ("rescue_center", "disposal_center"), ("RescueCenter", "DisposalCenter"),
        ("wounded_person", "bomb"), ("WoundedPerson", "Bomb"),
    ):
        code = code.replace(before, after)
    code = "import random\n" + code
    code = code.replace("super().__init__(drone_type, zones_config, seed=seed)",
                        "super().__init__(drone_type, zones_config)\n"
                        "        self._rng = random.Random(seed if seed is not None else random.getrandbits(64))")
    code = re.sub(r",\s*(?:use_shaders=use_shaders|seed=self\._seed)(?=\s*[,\)])", "", code)
    if cls.name == "MapRandom":
        code = ("from swarm_rescue.simulation.elements.disposal_center import DisposalCenter\n"
                "from swarm_rescue.simulation.elements.normal_wall import NormalBox\n" + code)
        marker = "        # POSITIONS OF THE DRONES"
        extra = (
            "        self._disposal_center = DisposalCenter(size=(90, 90))\n"
            "        self._playground.add(self._disposal_center, ((0, 0), 0))\n"
            "        for point in ((-430, 145), (300, 160), (-390, -160), (360, -170)):\n"
            "            box = NormalBox(up_left_point=point, width=70, height=54)\n"
            "            self._playground.add(box, box.wall_coordinates)\n"
            "        self._max_timestep_limit = 2000\n"
            "        self._max_walltime_limit = 90\n\n"
        )
        code = code.replace(marker, extra + marker)
        code = code.replace(
            "            x = self._rng.uniform(-self._size_area[0] / 2, self._size_area[0] / 2)\n"
            "            y = self._rng.uniform(-self._size_area[1] / 2, self._size_area[1] / 2)\n",
            "            while True:\n"
            "                x = self._rng.uniform(-self._size_area[0] / 2 + 24, self._size_area[0] / 2 - 24)\n"
            "                y = self._rng.uniform(-self._size_area[1] / 2 + 24, self._size_area[1] / 2 - 24)\n"
            "                occupied = any(shape.point_query((x, y)).distance < 20\n"
            "                               for element in self._playground.elements for shape in element.pm_shapes)\n"
            "                if not occupied and all(math.dist((x, y), drone.true_position()) >= 36 for drone in self._drones):\n"
            "                    break\n",
        )
    anchor = "        misc_data = MiscData("
    assert code.count(anchor) == 1, source
    if cls.name == "MapIntermediate02":
        code = "from swarm_rescue.simulation.elements.normal_wall import NormalBox\n" + code
        code = code.replace(anchor, (
            "        for point in ((-340, 145), (40, 150), (-280, -130), (140, -100)):\n"
            "            box = NormalBox(up_left_point=point, width=55, height=44)\n"
            "            self._playground.add(box, box.wall_coordinates)\n"
        ) + anchor)
    code = code.replace(anchor, "        self._explored_map.initialize_walls(self._playground)\n" + anchor)
    compile(code, str(destination), "exec")
    destination.write_text(code, encoding="utf-8")
    return cls.name, zones


def generate_bombs(arena, name, kind, seed, output):
    """Sample separated fixtures from the benchmark geometry, outside the controller."""
    import math
    import random

    import cv2
    import numpy as np
    from scipy.spatial import cKDTree
    from swarm_rescue.simulation.elements.normal_wall import NormalBox, NormalWall

    free = arena._explored_map._map_playground
    height, width = free.shape
    clearance = cv2.distanceTransform(free, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
    starting_positions = [drone.true_position() for drone in arena.drones]
    center = arena.disposal_center.pm_shapes
    hazards = [value for key, value in vars(arena).items()
               if "kill_zone" in key and hasattr(value, "pm_shapes")]
    shapes = list(center) + [shape for hazard in hazards for shape in hazard.pm_shapes]
    candidates = [(x - width / 2, height / 2 - y)
                  for y in range(30, height - 30, 12)
                  for x in range(30, width - 30, 12) if clearance[y, x] >= 25]
    rng = random.Random(seed + 997 * LAYOUTS.index(kind) + sum(map(ord, name)))
    rng.shuffle(candidates)
    candidates = [p for p in candidates if all(math.dist(p, s) >= 40 for s in starting_positions)
                  and all(shape.point_query(p).distance >= 30 for shape in shapes)]

    def select(pool, separation):
        selected = []
        for point in pool:
            if any(math.dist(point, p) < separation for p in selected):
                continue
            selected.append(point)
            if len(selected) == 10:
                return selected
        return None

    metadata = {}
    if kind == "corner":
        corners = set()
        for element in arena.playground.elements:
            if not isinstance(element, (NormalBox, NormalWall)):
                continue
            for shape in element.pm_shapes:
                vertices = shape.get_vertices() if hasattr(shape, "get_vertices") else (shape.a, shape.b)
                for vertex in vertices:
                    x, y = shape.body.local_to_world(vertex)
                    if abs(x) < width / 2 - 14 and abs(y) < height / 2 - 14:
                        corners.add((round(x, 2), round(y, 2)))
        if not corners:
            raise ValueError(f"No internal obstacle corners on {name}")
        tree = cKDTree(sorted(corners))
        distances, ids = tree.query(candidates)
        selected = None
        for radius in (72, 95, 120):
            grouped = {}
            for p, distance, corner in zip(candidates, distances, ids):
                if distance <= radius:
                    grouped.setdefault(int(corner), []).append(p)
            ordered = [pool[level] for level in range(max(map(len, grouped.values()), default=0))
                       for pool in grouped.values() if level < len(pool)]
            selected = select(ordered, 42)
            if selected:
                ds, used = tree.query(selected)
                metadata = {"max_corner_distance": float(max(ds)), "distinct_corners": len(set(used))}
                break
    elif kind == "cluster":
        points = np.asarray(candidates)
        tree = cKDTree(points)
        selected = None
        for radius in (72, 84, 96, 108):
            for local_center in candidates:
                ids = tree.query_ball_point(local_center, radius)
                if len(ids) < 10:
                    continue
                rng.shuffle(ids)
                selected = select((candidates[index] for index in ids), 32)
                if selected:
                    metadata = {"center": local_center, "radius": radius}
                    break
            if selected:
                break
    else:
        selected = select(candidates, 72) or select(candidates, 48)
    if not selected:
        raise ValueError(f"Could not place 10 separated bombs on {name} ({kind}, {seed})")
    bombs = [{"x": x, "y": y, "theta": 0.0} for x, y in selected]
    arena.spawn_bombs(bombs)
    assert arena.number_bombs == 10
    output.write_text(json.dumps({"map_name": name, "layout": kind, "seed": seed,
                                  "bombs": bombs, **metadata}, indent=2), encoding="utf-8")


def write_report(root, rows):
    digest = sha256((root / "team861_final.zip").read_bytes()).hexdigest()
    report = [
        "# 新增参考地图拆弹测试", "",
        "使用清理后的冻结包，在隔离目录按 14 张历史地图生成拆弹场景。"
        "每图 10 架无人机，每轮 10 枚炸弹。普通随机、障碍角落、局部聚集各使用种子 101 和 303，"
        f"运行官方 `--competition --headless`。ZIP SHA256 为 `{digest}`。", "",
        "MapRandom 原本没有处置中心、内部障碍物或任务目标。测试场景加入处置中心和四个障碍物，"
        "采用 2000 步、90 秒上限，并根据各次随机出生避免重叠。"
        "MapIntermediate02 原图只有边界墙，也加入四个内部障碍物以支持角落布局。", "",
        "| 地图 | 无人机 | 特殊区 | 普通随机均分 | 障碍角落均分 | 聚集均分 | 均分 | 全拆 | 坠毁 |",
        "| --- | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    valid = [row for row in rows if row["exit_code"] == 0 and row["score"] is not None
             and row["total_bombs"] == 10]
    for name in sorted({row["map"] for row in rows}):
        group = [row for row in valid if row["map"] == name]
        source = next(row for row in rows if row["map"] == name)
        scores = [[r["score"] for r in group if r["layout"] == kind] for kind in LAYOUTS]
        averages = " | ".join(f"{mean(values):.2f}" if len(values) == 2 else "—" for values in scores)
        avg = f"{mean(row['score'] for row in group):.2f}" if len(group) == 6 else "—"
        zones = "、".join(source["zones"]) or "无"
        destroyed = sum(row["destroyed_count"] for row in group)
        report.append(f"| {name} | {source['drones'] or '—'} | {zones} | {averages} | "
                      f"{avg} | {sum(r['disposed'] == 10 for r in group)}/6 | {destroyed} |")
    report += ["", f"有效成绩 {len(valid)}/{len(rows)} 轮，全拆 {sum(r['disposed'] == 10 for r in valid)} 轮，"
               f"交付 {sum(r['disposed'] for r in valid)}/{10 * len(valid)} 枚，"
               f"达到 90 分 {sum(r['score'] >= 90 for r in valid)} 轮。", "",
               "| 布局 | 轮数 | 平均分 | 最低分 | 全拆 | 坠毁 |",
               "| --- | ---: | ---: | ---: | ---: | ---: |"]
    for kind, label in zip(LAYOUTS, ("普通随机", "障碍角落", "局部聚集")):
        group = [r for r in valid if r["layout"] == kind]
        if group:
            scores = [r["score"] for r in group]
            report.append(f"| {label} | {len(group)} | {mean(scores):.2f} | {min(scores):.1f} | "
                          f"{sum(r['disposed'] == 10 for r in group)} | {sum(r['destroyed_count'] for r in group)} |")
    report += ["",
               "同目录的 `results.json` 含逐轮健康、步数、坠毁及日志路径。"
               "每轮炸弹坐标记录在对应的 `_bombs.json` 中。"]
    (root / "REPORT.md").write_text("\n".join(report) + "\n", encoding="utf-8")


def run_round(root, repo, env, case):
    module, name, zones, kind, seed, config = case
    log = root / f"{name}_{kind}_seed{seed}.log"
    if log.exists():
        attempt = log.with_suffix(".startup_attempt.log")
        if not attempt.exists():
            shutil.copyfile(log, attempt)
    cmd = [sys.executable, str(Path(__file__).resolve()), str(root),
           "--child", module, name, kind, str(seed), str(config)]
    try:
        result = subprocess.run(cmd, cwd=repo, env=env, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True, encoding="utf-8",
                                errors="replace", timeout=1500)
        output, code = result.stdout, result.returncode
    except subprocess.TimeoutExpired as exc:
        captured = exc.stdout or ""
        output = (captured.decode("utf-8", errors="replace") if isinstance(captured, bytes) else captured) + "\nTIMEOUT\n"
        code = -9
    log.write_text(output, encoding="utf-8")
    row = {"map": name, "layout": kind, "seed": seed, "zones": zones, "log": str(log), "exit_code": code}
    row.update(parse_log(output))
    count = re.search(r"Fixture drone count: (\d+)", output)
    row["drones"] = int(count[1]) if count else None
    if row["destroyed_percent"] is not None and row["drones"]:
        row["destroyed_count"] = round(row["destroyed_percent"] * row["drones"] / 100)
    row["walltime_limit_reached"] = "is reached first" in output
    return row


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=Path)
    parser.add_argument("--sources", type=Path)
    parser.add_argument("--repo", type=Path)
    parser.add_argument("--maps", nargs="+", help="Run or retry specific converted map classes")
    parser.add_argument("--resume", action="store_true", help="Keep completed rounds and retry missing or failed ones")
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--child", nargs=5, metavar=("MODULE", "MAP", "LAYOUT", "SEED", "CONFIG"))
    args = parser.parse_args()
    if args.child:
        module, name, kind, seed, config = args.child
        historical_class = getattr(importlib.import_module(f"swarm_rescue.maps.{module}"), name)

        def map_class(**kwargs):
            arena = historical_class(seed=int(seed), **kwargs)
            filename = f"{name}_{kind}_seed{seed}_bombs.json"
            layout = args.root.parent / "custom_layout_preflight" / f"{name}_{kind}_seed{seed}.json"
            fixture = json.loads(layout.read_text(encoding="utf-8"))
            assert len(fixture["bombs"]) == 10
            arena.spawn_bombs(fixture["bombs"])
            assert arena.number_bombs == 10
            shutil.copyfile(layout, args.root / filename)
            assert arena.number_drones == 10
            print(f"Fixture drone count: {arena.number_drones}", flush=True)
            return arena

        sys.argv = [sys.argv[0], seed, "--competition", "--headless", "--config", config]
        launch_seeded({name: map_class})
        return

    root, original_repo = args.root.resolve(), args.repo.resolve()
    root.mkdir(parents=True, exist_ok=True)
    sources = root.parent / "custom_map_sources"
    sources = sources if sources.is_dir() else args.sources
    repo = root / "runtime"
    if not repo.exists():
        shutil.copytree(original_repo, repo, ignore=shutil.ignore_patterns("__pycache__", "solutions"))
        sol = repo / "src/swarm_rescue/solutions"
        sol.mkdir()
        package = root.parent / "clean_submission/team861_final.zip"
        shutil.copyfile(package, root / package.name)
        with ZipFile(package) as submission:
            for name in submission.namelist():
                assert Path(name).name == name
                (repo / name if name == "req.txt" else sol / name).write_bytes(submission.read(name))
    source_maps = sorted(p for p in sources.glob("map_*.py")
                         if p.stem.startswith(("map_final_", "map_intermediate_", "map_medium_", "map_random", "map_test_special_zones")))
    dest = repo / "src/swarm_rescue/maps"
    for p in sources.glob("walls_*.py"):
        if not (dest / p.name).exists():
            shutil.copyfile(p, dest / p.name)
    plan = [(p.stem, *adapt(p, dest / p.name)) for p in source_maps]
    assert len(plan) == 16
    plan = [entry for entry in plan if entry[1] not in EXCLUDED]
    env = dict(os.environ, PYTHONPATH=str(repo / "src"), PYTHONIOENCODING="utf-8")
    if args.maps:
        wanted = set(args.maps)
        plan = [entry for entry in plan if entry[1] in wanted]
        if {entry[1] for entry in plan} != wanted:
            parser.error("Unknown map in --maps")
    results = root / "results.json"
    previous = json.loads(results.read_text(encoding="utf-8")) if (args.maps or args.resume) and results.exists() else []
    if args.resume:
        rows = [r for r in previous if r["exit_code"] == 0 and r["score"] is not None and r["total_bombs"] == 10]
    else:
        rows = [row for row in previous if row["map"] not in {entry[1] for entry in plan}]
    finished = {(r["map"], r["layout"], r["seed"]) for r in rows}
    cases = []
    for module, name, zones in plan:
        config = root / f"{name}.yml"
        config.write_text(yaml.safe_dump({
            "stat_saving_enabled": False, "state_recording_enabled": False,
            "video_capture_enabled": False, "time_best_ratio": 0.3,
            "evaluation_plan": [{"map_name": name, "team_mode": "rescue",
                                 "nb_rounds": 1, "config_weight": 1, "zones_config": zones}],
        }), encoding="utf-8")
        for kind, seed in ((kind, seed) for kind in LAYOUTS for seed in (101, 303)):
            if (name, kind, seed) in finished:
                continue
            cases.append((module, name, zones, kind, seed, config))

    def record(row):
        rows.append(row)
        temporary = results.with_suffix(".tmp")
        write_results(temporary, rows)
        temporary.replace(results)
        print(f"[{len(rows)}/{total}] {row['map']} {row['layout']} seed={row['seed']}: "
              f"{row['disposed']}/{row['total_bombs']} score={row['score']} exit={row['exit_code']}", flush=True)

    total = len(rows) + len(cases)
    competitive = [case for case in cases if case[1] not in {"MapRandom", "MapIntermediate02"}]
    serial = [case for case in cases if case[1] in {"MapRandom", "MapIntermediate02"}]
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        futures = [pool.submit(run_round, root, repo, env, case) for case in competitive]
        for future in as_completed(futures):
            record(future.result())
    for case in serial:
        record(run_round(root, repo, env, case))
    write_report(root, rows)


if __name__ == "__main__":
    main()
