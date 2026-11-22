"""Run frozen layouts with paired spawn/noise seeds through the official launcher."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import yaml

from run_benchmarks import ZONES, parse_log, write_results


ACTIVE_MODULES = (
    "coverage.py", "localization.py", "motion_control.py",
    "my_drone_rescue.py", "nav_grid.py", "my_drone_eval.py",
)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, default=Path("."))
    parser.add_argument("--cases", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--maps", nargs="+", required=True)
    parser.add_argument("--seeds", nargs="+", type=int, required=True)
    parser.add_argument("--kinds", nargs="+", default=["user_fixed"])
    parser.add_argument("--trace", action="store_true")
    parser.add_argument("--timeout", type=int, default=300)
    parser.add_argument("--python", default=sys.executable)
    args = parser.parse_args()

    repo = args.repo.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    manifest = json.loads((args.cases.resolve() / "manifest.json").read_text(encoding="utf-8"))
    cases = [c for c in manifest if c["case"] in args.kinds and c["map"] in args.maps]
    cases.sort(key=lambda c: (args.maps.index(c["map"]), args.kinds.index(c["case"])))
    if len(cases) != len(args.maps) * len(args.kinds):
        parser.error("Expected one layout for each requested map and case kind")
    solutions = repo / "src" / "swarm_rescue" / "solutions"
    source_hashes = {
        name: hashlib.sha256((solutions / name).read_bytes()).hexdigest()
        for name in ACTIVE_MODULES
    }
    (output / "source_sha256.json").write_text(
        json.dumps(source_hashes, indent=2) + "\n", encoding="utf-8"
    )

    env = os.environ.copy()
    env["PYTHONPATH"] = str(repo / "src")
    if args.trace:
        env["SWARM_RESCUE_BENCH_TRACE"] = "1"
    rows = []
    total = len(cases) * len(args.seeds)
    for case in cases:
        map_name, kind = case["map"], case["case"]
        config = {
            "stat_saving_enabled": False,
            "state_recording_enabled": False,
            "video_capture_enabled": False,
            "time_best_ratio": 0.3,
            "evaluation_plan": [{
                "map_name": map_name,
                "team_mode": "rescue",
                "nb_rounds": 1,
                "config_weight": 1,
                "number_drones": 10,
                "zones_config": ZONES.get(map_name, []),
            }],
        }
        config_path = output / f"config_{map_name}.yml"
        config_path.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
        for seed in args.seeds:
            log_path = output / f"{map_name}_{kind}_seed{seed}.log"
            cmd = [
                args.python, str(repo / "bench_tools" / "seeded_official_launcher.py"),
                str(seed), "--competition", "--headless", "--config", str(config_path),
                "--bombs-file", str(Path(case["path"]).resolve()),
            ]
            started = time.time()
            timed_out = False
            try:
                result = subprocess.run(
                    cmd, cwd=repo, env=env, text=True, encoding="utf-8",
                    errors="replace", stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT, timeout=args.timeout,
                )
                raw_log = result.stdout
                exit_code = result.returncode
            except subprocess.TimeoutExpired as exc:
                timed_out = True
                captured = exc.stdout or ""
                if isinstance(captured, bytes):
                    captured = captured.decode("utf-8", errors="replace")
                raw_log = captured + "\nTIMEOUT\n"
                exit_code = -9
            log_path.write_text(raw_log, encoding="utf-8")
            row = {
                "map": map_name, "case": kind, "seed": seed,
                "bombs_file": str(Path(case["path"]).resolve()),
                "log": str(log_path), "exit_code": exit_code,
                "timed_out": timed_out,
                "runner_seconds": round(time.time() - started, 3),
            }
            row.update(parse_log(raw_log))
            rows.append(row)
            write_results(output / "results.json", rows)
            print(
                f"[{len(rows)}/{total}] {map_name} {kind} seed={seed}: "
                f"disposed={row['disposed']}/10 score={row['score']} "
                f"steps={row['steps']} health={row['mean_health_percent']} "
                f"destroyed={row['destroyed_count']} exit={exit_code} timeout={timed_out}",
                flush=True,
            )


if __name__ == "__main__":
    main()
