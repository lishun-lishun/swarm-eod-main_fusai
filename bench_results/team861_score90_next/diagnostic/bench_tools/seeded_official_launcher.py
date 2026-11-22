"""Test-only seed setup, then run the unmodified official launcher."""

from __future__ import annotations

import os
import random
import runpy
import sys

import numpy as np


def trace_mission() -> None:
    """Read controller state for diagnostics without altering its commands."""
    from swarm_rescue.solutions.my_drone_rescue import MyDroneRescue

    original_control = MyDroneRescue.control

    def control(drone):
        previous_carry = drone._was_carrying
        previous_gps = drone.pose.gps_settled
        previous_escape = getattr(drone, "_return_escape_until", 0)
        previous_docks = getattr(drone, "_failed_docks", 0)
        command = original_control(drone)
        if drone._was_carrying and (drone._step % 60 == 0 or previous_gps != drone.pose.gps_settled):
            print("ROUTE", drone._step, drone.identifier, drone.pose.gps_settled,
                  (round(drone.pose.x), round(drone.pose.y)), drone._waypoints[:2],
                  drone._dc_target_pt, len(drone._waypoints), len(drone._no_gps_breadcrumbs),
                  round(drone.pose.vx, 1), round(drone.pose.vy, 1), flush=True)
        escape = getattr(drone, "_return_escape_until", 0)
        docks = getattr(drone, "_failed_docks", 0)
        if escape > previous_escape or docks > previous_docks:
            print("RECOVERY", drone._step, drone.identifier, "escape", escape, "docks", docks,
                  drone.pose.gps_settled, drone.communicator_is_disabled(),
                  round(drone.pose.x), round(drone.pose.y), flush=True)
        if previous_carry != drone._was_carrying or drone._step % 300 == 0:
            target = drone._known_bombs.get(drone._target_bomb_key, {})
            print("MISSION", drone._step, drone.identifier, drone._state, drone._was_carrying,
                  drone.pose.gps_settled, drone.communicator_is_disabled(),
                  round(drone.pose.x), round(drone.pose.y), drone.drone_health,
                  drone._target_bomb_key, target.get("direct_seen"), drone._local_disposed, flush=True)
        return command

    MyDroneRescue.control = control


def main(map_classes=None) -> None:
    seed = int(sys.argv[1])
    launcher_args = sys.argv[2:]
    random.seed(seed)
    np.random.seed(seed)

    # Playground uses default_rng(None); seed each new generator reproducibly.
    original_default_rng = np.random.default_rng
    generator_count = 0

    def seeded_default_rng(value=None):
        nonlocal generator_count
        if value is None:
            generator_count += 1
            value = seed + 1_000_003 * generator_count
        return original_default_rng(value)

    np.random.default_rng = seeded_default_rng
    if os.environ.get("SWARM_RESCUE_BENCH_TRACE") == "1":
        trace_mission()
    sys.argv = ["swarm_rescue.launcher", *launcher_args]
    runpy.run_module("swarm_rescue.launcher", init_globals=map_classes, run_name="__main__")


if __name__ == "__main__":
    main()
