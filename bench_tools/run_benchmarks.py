"""Run competition/headless rescue cases and preserve raw logs/results."""
from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

import yaml


ZONES = {
    "Map06": ["NO_COM_ZONE"],
    "Map08": ["NO_GPS_ZONE"],
    "Map09": ["NO_COM_ZONE", "NO_GPS_ZONE"],
}


def match(pattern, text, cast=float, default=None):
    found = re.search(pattern, text, re.I | re.S)
    return cast(found.group(1)) if found else default


def parse_log(text: str):
    return {
        "disposed": match(r"rescued nb:\s*(\d+)/", text, int),
        "total_bombs": match(r"rescued nb:\s*\d+/(\d+)", text, int),
        "health_score": match(r"health return score:\s*([0-9.]+)%", text),
        "time_score": match(r"time score:\s*([0-9.]+)%", text),
        "exploration_score": match(r"explor\. score:\s*([0-9.]+)%", text),
        "walltime_seconds": match(r"walltime elapsed:\s*([0-9.]+)s/", text),
        "steps": match(r"elapse timestep:\s*(\d+)/", text, int),
        "full_disposal_step": match(r"time to dispose all:\s*(\d+)", text, int),
        "score": match(r"round score:\s*([0-9.]+)%", text),
        "destroyed_percent": match(r"percentage of drones destroyed:\s*([0-9.]+)", text),
        "mean_health_percent": match(r"mean percentage of drones health\s*:\s*([0-9.]+)", text),
        "destroyed_count": len(re.findall(r"Drone \d+ destroyed", text)),
        "crashed": bool(re.search(r"round crashed|Traceback", text, re.I)),
    }


def write_results(path: Path, rows):
    path.write_text(json.dumps(rows, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, default=Path("."))
    parser.add_argument("--cases", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--maps", nargs="*")
    parser.add_argument("--kinds", nargs="*", default=["generated_fixed_seed0", "edge_seed11", "edge_seed29", "edge_seed53"])
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--timeout", type=int, default=240)
    parser.add_argument("--python", default=sys.executable)
    args = parser.parse_args()

    repo = args.repo.resolve()
    cases_dir = args.cases.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    selected_maps = set(args.maps or [])
    selected_kinds = set(args.kinds)
    manifest = json.loads((cases_dir / "manifest.json").read_text(encoding="utf-8"))
    cases = [c for c in manifest
             if (not selected_maps or c["map"] in selected_maps)
             and c["case"] in selected_kinds]
    rows = []
    results_path = output / "results.json"
    total = len(cases) * args.repeats
    count = 0
    env = os.environ.copy()
    env["PYTHONPATH"] = str(repo / "src")

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
        for repeat in range(1, args.repeats + 1):
            count += 1
            log_path = output / f"{map_name}_{kind}_r{repeat}.log"
            cmd = [args.python, "-m", "swarm_rescue.launcher",
                   "--competition", "--headless", "--config", str(config_path),
                   "--bombs-file", str(Path(case["path"]).resolve())]
            started = time.time()
            timed_out = False
            try:
                proc = subprocess.run(cmd, cwd=repo, env=env, text=True,
                                      encoding="utf-8", errors="replace",
                                      stdout=subprocess.PIPE,
                                      stderr=subprocess.STDOUT,
                                      timeout=args.timeout)
                text = proc.stdout
                exit_code = proc.returncode
            except subprocess.TimeoutExpired as exc:
                timed_out = True
                captured = exc.stdout or ""
                if isinstance(captured, bytes):
                    captured = captured.decode("utf-8", errors="replace")
                text = captured + "\nTIMEOUT\n"
                exit_code = -9
            log_path.write_text(text, encoding="utf-8")
            row = {"map": map_name, "case": kind, "repeat": repeat,
                   "bombs_file": str(Path(case["path"]).resolve()),
                   "log": str(log_path), "exit_code": exit_code,
                   "timed_out": timed_out,
                   "runner_seconds": round(time.time() - started, 3)}
            row.update(parse_log(text))
            rows.append(row)
            write_results(results_path, rows)
            print(f"[{count}/{total}] {map_name} {kind} r{repeat}: "
                  f"disposed={row['disposed']}/{row['total_bombs']} "
                  f"score={row['score']} steps={row['steps']} "
                  f"health={row['mean_health_percent']} destroyed={row['destroyed_count']} "
                  f"exit={exit_code} timeout={timed_out}", flush=True)
    print(f"Wrote {len(rows)} runs to {results_path}")


if __name__ == "__main__":
    main()
