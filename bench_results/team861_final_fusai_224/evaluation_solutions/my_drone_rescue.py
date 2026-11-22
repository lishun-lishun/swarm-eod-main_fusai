"""
Pure sensor-driven multi-drone bomb disposal controller (team_mode: rescue).

All navigation, localization, bomb discovery, disposal-center localization, and
task allocation operate strictly from runtime official sensors (LiDAR, semantic
sensor, GPS, compass, odometer, grasper) and official communicator messages.
Contains zero preloaded map geometry, zero bomb coordinates, and zero
module-level shared state.
"""

from __future__ import annotations

import math
from collections import deque
from typing import Any, Deque, Dict, List, Optional, Tuple

import numpy as np

from swarm_rescue.simulation.drone.controller import CommandsDict
from swarm_rescue.simulation.drone.drone_abstract import DroneAbstract
from swarm_rescue.simulation.ray_sensors.drone_semantic_sensor import DroneSemanticSensor
from swarm_rescue.simulation.utils.misc_data import MiscData
from swarm_rescue.simulation.utils.utils import normalize_angle

from swarm_rescue.solutions.coverage import CoverageTracker
from swarm_rescue.solutions.localization import PoseEstimator
from swarm_rescue.solutions.motion_control import drive_to_target, empty_command
from swarm_rescue.solutions.nav_grid import OnlineMap, WorldPoint

STATE_FANOUT = 0
STATE_EXPLORE = 1
STATE_TO_BOMB = 2
STATE_GRASPING = 3
STATE_CARRYING = 4
STATE_DISPOSING = 5


class MyDroneRescue(DroneAbstract):
    """Coordinated autonomous EOD swarm controller built strictly from online sensors."""

    _UNLOADED_MAX_SPEED = 12.6
    _LOADED_MAX_SPEED = 12.0

    def __init__(
        self,
        identifier: Optional[int] = None,
        misc_data: Optional[MiscData] = None,
        **kwargs: Any,
    ) -> None:
        super().__init__(identifier=identifier, misc_data=misc_data, **kwargs)
        self.pose = PoseEstimator()

        self._omap: Optional[OnlineMap] = None
        self._cov: Optional[CoverageTracker] = None

        self._step = 0
        self._state = STATE_FANOUT
        self._spawn_pos: Optional[WorldPoint] = None
        self._fanout_target: Optional[WorldPoint] = None
        self._explore_target: Optional[WorldPoint] = None
        self._explore_set_step = -100
        self._explore_is_stale = False

        # Active target bomb tracking
        self._target_bomb_key: Optional[Tuple[int, int]] = None
        self._carried_bomb_key: Optional[Tuple[int, int]] = None
        self._grasp_steps = 0
        self._dispose_steps = 0
        self._dc_target_pt: Optional[WorldPoint] = None
        self._dc_hit_pt: Optional[WorldPoint] = None
        self._dc_plan_step = -100

        # Path cache
        self._waypoints: List[WorldPoint] = []
        self._plan_step = -100
        self._plan_goal: Optional[WorldPoint] = None

        # Carrying / disposal state transition detection
        self._was_carrying = False
        self._last_saw_dc_step = -100
        self._got_new_bomb_msg = False
        self._local_disposed = 0

        # Anti-snag / unhook recovery & NO_GPS breadcrumb trail
        self._pos_hist: Deque[WorldPoint] = deque(maxlen=46)
        self._no_gps_breadcrumbs: List[WorldPoint] = []
        self._no_gps_corridor: List[WorldPoint] = []
        self._unhook_steps = 0
        self._unhook_target: Optional[WorldPoint] = None
        self._unhook_cooldown_until = 0
        self._carry_snag_count = 0
        self._carry_snag_pos: Optional[WorldPoint] = None
        self._goal_path_dist = 0.0

        # Dynamic registry of known bombs discovered by semantic sensor / peers
        # key -> {pos, status ("ACTIVE"|"CARRIED"|"DONE"), last_seen, claimer, claim_step, miss_cnt}
        self._known_bombs: Dict[Tuple[int, int], Dict[str, Any]] = {}

        # Peer telemetry from official communicator (cleared/aged naturally)
        self._peers: Dict[int, Dict[str, Any]] = {}

    def define_message_for_all(self) -> Any:
        if not self.pose.initialized or self._omap is None or self._cov is None:
            return None

        bombs_msg = []
        # Prioritize ACTIVE, CARRIED, and recent DONE tombstones so ghosts die swarm-wide
        sorted_bombs = sorted(
            self._known_bombs.items(),
            key=lambda kv: (
                0 if kv[1]["status"] == "ACTIVE" else (
                    1 if kv[1]["status"] == "CARRIED" else (
                        2 if (self._step - kv[1].get("done_step", -999)) <= 300 else 3
                    )
                ),
                -int(kv[1].get("done_step", kv[1]["last_seen"])),
            ),
        )
        for k, b in sorted_bombs[:30]:
            bx, by = b["pos"]
            b_status = b["status"]
            b_claimer = b["claimer"]
            my_int_id = int(self.identifier)
            if b_claimer == my_int_id:
                is_my_active_target = (self._state in (STATE_TO_BOMB, STATE_GRASPING) and k == self._target_bomb_key)
                is_my_carry = (self._state in (STATE_CARRYING, STATE_DISPOSING) and self._was_carrying and k == self._carried_bomb_key)
                if not (is_my_active_target or is_my_carry):
                    b_claimer = None
                    b["claimer"] = None
                    if b_status == "CARRIED":
                        b_status = "ACTIVE"
                        b["status"] = "ACTIVE" 
            bombs_msg.append(
                (
                    k,
                    round(bx, 1),
                    round(by, 1),
                    b_status,
                    int(b["last_seen"]),
                    b_claimer,
                    int(b["claim_step"]),
                )
            )

        send_dc = (self._step <= 5) or ((self._step + int(self.identifier)) % 8 == 0)
        dc_msg = (
            [
                (round(hx, 1), round(hy, 1), round(ax, 1), round(ay, 1))
                for hx, hy, ax, ay in list(self._omap.dc_points.values())[:10]
            ]
            if send_dc
            else []
        )

        wall_chunk = (
            self._omap.get_sync_wall_chunk(max_total=64)
            if ((self._step + int(self.identifier)) % 4 == 0)
            else []
        )
        cov_bytes = self._cov.pack_seen() if ((self._step + int(self.identifier)) % 4 == 0) else None

        active_goal: Optional[WorldPoint] = None
        if self._state == STATE_EXPLORE and self._explore_target is not None:
            active_goal = (round(self._explore_target[0], 1), round(self._explore_target[1], 1))
        elif self._state in (STATE_TO_BOMB, STATE_GRASPING) and self._target_bomb_key in self._known_bombs:
            gp = self._known_bombs[self._target_bomb_key]["pos"]
            active_goal = (round(gp[0], 1), round(gp[1], 1))

        active_claim = None
        target_k = (
            self._carried_bomb_key
            if self._was_carrying
            else (self._target_bomb_key if self._state in (STATE_TO_BOMB, STATE_GRASPING) else None)
        )
        if target_k is not None and target_k in self._known_bombs:
            gp = self._known_bombs[target_k]["pos"]
            active_claim = (round(gp[0], 1), round(gp[1], 1))
        elif target_k is not None:
            active_claim = target_k

        gd_val: Optional[float] = None
        if self._state in (STATE_TO_BOMB, STATE_GRASPING) and self._target_bomb_key in self._known_bombs:
            if self._state == STATE_GRASPING:
                gp_b = self._known_bombs[self._target_bomb_key]["pos"]
                gd_val = round(math.hypot(gp_b[0] - self.pose.x, gp_b[1] - self.pose.y), 1)
            elif self._waypoints:
                acc = 0.0
                prev_p = (self.pose.x, self.pose.y)
                for wp_i in self._waypoints:
                    acc += math.hypot(wp_i[0] - prev_p[0], wp_i[1] - prev_p[1])
                    prev_p = wp_i
                gd_val = round(acc, 1)
            elif self._goal_path_dist > 0.0:
                gd_val = round(self._goal_path_dist, 1)

        return {
            "id": int(self.identifier),
            "step": self._step,
            "x": round(self.pose.x, 2),
            "y": round(self.pose.y, 2),
            "vx": round(self.pose.vx, 2),
            "vy": round(self.pose.vy, 2),
            "gps_ok": self.pose.gps_settled,
            "carry": bool(self._was_carrying or len(self.grasped_bombs()) > 0 or self._state in (STATE_CARRYING, STATE_DISPOSING)),
            "tier": self._current_tier(self._was_carrying),
            "state": self._state,
            "goal": active_goal,
            "claim": active_claim,
            "gd": gd_val,
            "bombs": bombs_msg,
            "dc": dc_msg,
            "walls": wall_chunk,
            "cov": cov_bytes,
        }

    def _current_tier(self, carrying: bool) -> int:
        if self._state == STATE_DISPOSING:
            return 3
        if carrying:
            return 2
        if self._state in (STATE_TO_BOMB, STATE_GRASPING):
            return 1
        return 0

    def _init_maps_if_needed(self) -> None:
        if self._omap is not None:
            return
        size = self.size_area if self.size_area is not None else (1800, 1200)
        self._omap = OnlineMap(size)
        self._omap.comm_disabled = self.communicator_is_disabled()
        self._cov = CoverageTracker(self._omap)

    def _process_messages(self) -> None:
        self._got_new_bomb_msg = False
        if self.communicator_is_disabled() or self._omap is None or self._cov is None:
            return

        for item in self.communicator.received_messages:
            msg = item[1] if isinstance(item, (tuple, list)) and len(item) >= 2 else item
            if not isinstance(msg, dict) or "id" not in msg:
                continue
            pid = int(msg["id"])
            if pid == int(self.identifier):
                continue

            self._peers[pid] = {
                "x": float(msg["x"]),
                "y": float(msg["y"]),
                "vx": float(msg["vx"]),
                "vy": float(msg["vy"]),
                "carry": bool(msg["carry"]),
                "tier": int(msg["tier"]),
                "state": int(msg["state"]),
                "gps_ok": bool(msg.get("gps_ok", False)),
                "goal": msg.get("goal"),
                "claim": msg.get("claim"),
                "gd": msg.get("gd"),
                "rx_step": self._step,
            }

            if msg.get("dc"):
                self._omap.merge_peer_dc_points(msg["dc"])
            if msg.get("walls"):
                # wall_chunk strictly contains GPS-trusted _own_wall_list cells
                self._omap.merge_peer_walls(msg["walls"], peer_gps_trusted=True)
            if msg.get("cov"):
                c_mask = self._cov.merge_packed(msg["cov"], self._step)
                if c_mask is not None:
                    self._omap.merge_peer_highway(c_mask)

            for b_item in msg.get("bombs", ()):
                self._merge_bomb_observation(
                    pos=(float(b_item[1]), float(b_item[2])),
                    status=str(b_item[3]),
                    seen_step=int(b_item[4]),
                    claimer=b_item[5],
                    claim_step=int(b_item[6]),
                    from_direct_sensor=False,
                )

    def _find_matching_bomb_key(self, pos: WorldPoint, max_r: Optional[float] = None) -> Optional[Tuple[int, int]]:
        if max_r is None:
            max_r = 45.0 if self.pose.in_no_gps else 26.0
        best_k: Optional[Tuple[int, int]] = None
        best_d = max_r
        for k, rec in self._known_bombs.items():
            d = math.hypot(rec["pos"][0] - pos[0], rec["pos"][1] - pos[1])
            if d < best_d:
                best_d = d
                best_k = k
        return best_k

    def _merge_bomb_observation(
        self,
        *,
        pos: WorldPoint,
        status: str,
        seen_step: int,
        claimer: Optional[int],
        claim_step: int,
        from_direct_sensor: bool,
    ) -> Tuple[int, int]:
        match_r = 45.0 if self.pose.in_no_gps else 28.0
        k = self._find_matching_bomb_key(pos)
        if k is None:
            # Block stale peer gossips from spawning zombie duplicates near a recent DONE tombstone
            if (not from_direct_sensor) and status != "DONE":
                tomb_r = 45.0 if self.pose.in_no_gps else 32.0
                for kt, rt in self._known_bombs.items():
                    if rt["status"] == "DONE" and rt.get("done_step", -1) >= seen_step - 15:
                        if math.hypot(rt["pos"][0] - pos[0], rt["pos"][1] - pos[1]) < tomb_r:
                            return kt
            k = (int(round(pos[0] / 18.0)), int(round(pos[1] / 18.0)))
            while k in self._known_bombs:
                k = (k[0], k[1] + 1)
            self._known_bombs[k] = {
                "pos": pos,
                "status": status,
                "last_seen": seen_step,
                "direct_seen_step": seen_step if from_direct_sensor else -999,
                "done_step": seen_step if status == "DONE" else -1,
                "claimer": claimer,
                "claim_step": claim_step,
                "miss_cnt": 0,
                "gps_ok": (from_direct_sensor and self.pose.gps_settled),
            }
            if status == "ACTIVE":
                self._got_new_bomb_msg = True
            return k

        rec = self._known_bombs[k]
        if status == "DONE":
            # Accept DONE unless contradicted by a STRICTLY newer direct sensor sighting
            if rec.get("direct_seen_step", -999) <= seen_step:
                rec["status"] = "DONE"
                rec["claimer"] = None
                rec["done_step"] = max(int(rec.get("done_step", -1)), seen_step)
                rec["last_seen"] = max(int(rec["last_seen"]), seen_step)
                for k2, rec2 in self._known_bombs.items():
                    if k2 != k and rec2["status"] != "DONE":
                        if rec2.get("direct_seen_step", -999) <= seen_step:
                            if math.hypot(rec2["pos"][0] - rec["pos"][0], rec2["pos"][1] - rec["pos"][1]) < match_r:
                                rec2["status"] = "DONE"
                                rec2["claimer"] = None
                                rec2["done_step"] = max(int(rec2.get("done_step", -1)), seen_step)
                                rec2["last_seen"] = max(int(rec2["last_seen"]), seen_step)
        elif rec["status"] == "DONE":
            # Direct sensor ungrasped sighting ALWAYS heals falsely marked DONE;
            # peer gossip only heals if it is a genuinely fresh sighting well after done_step
            if from_direct_sensor:
                rec["status"] = "ACTIVE"
                rec["pos"] = pos
                rec["last_seen"] = seen_step
                rec["direct_seen_step"] = seen_step
                rec["miss_cnt"] = 0
                rec["gps_ok"] = self.pose.gps_settled
            elif status == "ACTIVE" and seen_step > int(rec.get("done_step", -1)) + 45:
                rec["status"] = "ACTIVE"
                rec["pos"] = pos
                rec["last_seen"] = seen_step
                rec["miss_cnt"] = 0
                self._got_new_bomb_msg = True
        else:
            if from_direct_sensor:
                rec["direct_seen_step"] = max(int(rec.get("direct_seen_step", -999)), seen_step)
            if seen_step >= rec["last_seen"]:
                curr_gps_ok = (from_direct_sensor and self.pose.gps_settled)
                if curr_gps_ok or not rec.get("gps_ok", False):
                    in_obs = (self._omap is not None and self._omap.obs_clearance_at(pos[0], pos[1]) < 5.0)
                    if not in_obs:
                        if from_direct_sensor:
                            rec["pos"] = pos
                        else:
                            ox, oy = rec["pos"]
                            w = 0.20
                            rec["pos"] = ((1.0 - w) * ox + w * pos[0], (1.0 - w) * oy + w * pos[1])
                        if curr_gps_ok:
                            rec["gps_ok"] = True
                rec["last_seen"] = seen_step
                if from_direct_sensor:
                    rec["miss_cnt"] = 0
            if status == "CARRIED":
                carrier_actually_carrying = False
                if claimer == int(self.identifier):
                    carrier_actually_carrying = self._was_carrying
                elif claimer is not None:
                    p = self._peers.get(claimer)
                    if p is not None and (self._step - p["rx_step"] <= 20):
                        carrier_actually_carrying = bool(p["carry"])
                    else:
                        carrier_actually_carrying = True
                else:
                    carrier_actually_carrying = True

                if not carrier_actually_carrying:
                    # Only revert if not near a recent DONE tombstone
                    rec["status"] = "ACTIVE"
                    rec["claimer"] = None
                    rec["claim_step"] = seen_step
                else:
                    # Physical carrier always trumps an ungrasped drone's claim!
                    rec["status"] = "CARRIED"
                    rec["claimer"] = claimer
                    rec["claim_step"] = max(int(rec.get("claim_step", 0)), claim_step)
                    for k2, rec2 in self._known_bombs.items():
                        if k2 != k and rec2["status"] == "ACTIVE":
                            if math.hypot(rec2["pos"][0] - rec["pos"][0], rec2["pos"][1] - rec["pos"][1]) < match_r:
                                rec2["status"] = "CARRIED"
                                rec2["claimer"] = claimer
                                rec2["claim_step"] = claim_step
            elif status == "ACTIVE":
                if rec["status"] == "CARRIED":
                    claimer_id = rec.get("claimer")
                    claimer_active = False
                    if claimer_id is not None:
                        if claimer_id == int(self.identifier):
                            claimer_active = self._was_carrying
                        else:
                            p = self._peers.get(claimer_id)
                            if p is not None and (self._step - p["rx_step"] <= 20):
                                claimer_active = bool(p["carry"])
                    if not claimer_active and (self._step - rec["claim_step"] > 35):
                        rec["status"] = "ACTIVE"
                        rec["claimer"] = None
                        rec["claim_step"] = seen_step
                elif claimer is not None and claim_step > rec["claim_step"]:
                    rec["claimer"] = claimer
                    rec["claim_step"] = claim_step
        return k

    def _observe_semantic(
        self,
        my_pos: WorldPoint,
        heading: float,
        semantic_vals: Optional[list],
        carrying: bool,
    ) -> Tuple[List[WorldPoint], Optional[Tuple[float, float]]]:
        """
        Process semantic detections for bombs and Disposal Center.
        Returns (visible_ungrasped_bomb_positions, closest_dc_ray_world_vec_and_dist).
        """
        visible_bombs: List[WorldPoint] = []
        best_dc_ray: Optional[Tuple[float, float]] = None
        best_dc_dist = float("inf")

        # Collect angles of visible drones so occluded bombs aren't falsely marked DONE
        vis_drone_angs: List[float] = []
        if semantic_vals:
            vis_drones: List[WorldPoint] = []
            for obj in semantic_vals:
                if obj.entity_type == DroneSemanticSensor.TypeEntity.DRONE:
                    d = float(obj.distance)
                    a = float(obj.angle)
                    if math.isfinite(d) and math.isfinite(a):
                        vis_drone_angs.append(normalize_angle(heading + a))
                        vis_drones.append(
                            (my_pos[0] + d * math.cos(heading + a), my_pos[1] + d * math.sin(heading + a))
                        )

            for obj in semantic_vals:
                d = float(obj.distance)
                a = float(obj.angle)
                if not (math.isfinite(d) and math.isfinite(a)):
                    continue
                if obj.entity_type == DroneSemanticSensor.TypeEntity.DISPOSAL_CENTER:
                    self._last_saw_dc_step = self._step
                    if d < best_dc_dist:
                        best_dc_dist = d
                        wang = heading + a
                        best_dc_ray = (wang, d)
                elif obj.entity_type == DroneSemanticSensor.TypeEntity.BOMB:
                    if getattr(obj, "grasped", False):
                        continue
                    bx = my_pos[0] + d * math.cos(heading + a)
                    by = my_pos[1] + d * math.sin(heading + a)
                    if carrying and math.hypot(bx - my_pos[0], by - my_pos[1]) < 26.0:
                        continue
                    # Simulator sets invisible_when_grasped=True, so only filter exact overlap (<20 px)
                    near_peer = any(math.hypot(bx - dx, by - dy) < 20.0 for dx, dy in vis_drones)
                    if near_peer:
                        continue
                    carrier_peer = any(
                        p["carry"] and math.hypot(bx - p["x"], by - p["y"]) < 24.0
                        for p in self._peers.values()
                        if (self._step - p["rx_step"] <= 15)
                    )
                    if carrier_peer:
                        continue
                    visible_bombs.append((bx, by))
                    self._merge_bomb_observation(
                        pos=(bx, by),
                        status="ACTIVE",
                        seen_step=self._step,
                        claimer=None,
                        claim_step=-1,
                        from_direct_sensor=True,
                    )

        # Negative observation check: clear ghost bombs rapidly when GPS is settled & LOS is unobstructed
        if self._omap is not None and (not self.pose.in_no_gps) and self.pose.gps_settled:
            for k, rec in self._known_bombs.items():
                if rec["status"] != "ACTIVE":
                    continue
                bx, by = rec["pos"]
                dist = math.hypot(bx - my_pos[0], by - my_pos[1])
                if dist < 58.0 and self._omap.has_walls_only_los(my_pos, (bx, by)):
                    ang_b = math.atan2(by - my_pos[1], bx - my_pos[0])
                    if any(abs(normalize_angle(ang_b - da)) < 0.22 for da in vis_drone_angs):
                        continue
                    seen_here = any(math.hypot(vx - bx, vy - by) < 34.0 for vx, vy in visible_bombs)
                    if seen_here:
                        rec["miss_cnt"] = 0
                    else:
                        rec["miss_cnt"] = int(rec.get("miss_cnt", 0)) + 1
                        if rec["miss_cnt"] >= 14:
                            rec["status"] = "DONE"
                            rec["claimer"] = None
                            rec["done_step"] = self._step
                            rec["last_seen"] = self._step
                            if self._target_bomb_key == k:
                                self._target_bomb_key = None

        return (visible_bombs, best_dc_ray)

    def _handle_carry_transition(
        self,
        carrying: bool,
        my_pos: WorldPoint,
        semantic_vals: Optional[list] = None,
        heading: float = 0.0,
    ) -> None:
        if carrying:
            if not self._was_carrying:
                # Just grasped a bomb!
                self._state = STATE_CARRYING
                self._dispose_steps = 0
                self._grasp_steps = 0
                self._waypoints = []
                self._dc_target_pt = None
                self._dc_hit_pt = None
                self._dc_plan_step = -100
                self._pos_hist.clear()
                self._unhook_steps = 0
                self._unhook_cooldown_until = self._step + 30
                self._carry_snag_count = 0
                self._carry_snag_pos = None

                k = self._target_bomb_key
                if k is None or k not in self._known_bombs:
                    k = self._find_matching_bomb_key(my_pos, max_r=48.0)
                if k is None:
                    k = (int(round(my_pos[0] / 18.0)), int(round(my_pos[1] / 18.0)))
                    while k in self._known_bombs:
                        k = (k[0], k[1] + 1)
                    self._known_bombs[k] = {
                        "pos": my_pos,
                        "status": "CARRIED",
                        "last_seen": self._step,
                        "direct_seen_step": self._step,
                        "done_step": -1,
                        "claimer": int(self.identifier),
                        "claim_step": self._step,
                        "miss_cnt": 0,
                    }
                self._carried_bomb_key = k

                match_r = 45.0 if self.pose.in_no_gps else 28.0
                pickup_pos = self._known_bombs[k]["pos"]
                for k2, rec2 in self._known_bombs.items():
                    if rec2["status"] != "DONE":
                        if math.hypot(rec2["pos"][0] - pickup_pos[0], rec2["pos"][1] - pickup_pos[1]) < match_r:
                            rec2["status"] = "CARRIED"
                            rec2["claimer"] = int(self.identifier)
                            rec2["claim_step"] = self._step

            # While carrying, continuously keep CARRIED status and claim_step fresh (keep pickup pos static)
            k = self._carried_bomb_key
            if k is not None and k in self._known_bombs:
                self._known_bombs[k]["status"] = "CARRIED"
                self._known_bombs[k]["claimer"] = int(self.identifier)
                self._known_bombs[k]["claim_step"] = self._step
                self._known_bombs[k]["last_seen"] = self._step

        elif (not carrying) and self._was_carrying:
            # Stopped carrying (either disposed at DC or lost grip)
            has_nearby_bomb = False
            nearby_bomb_pt: Optional[WorldPoint] = None
            if semantic_vals:
                for obj in semantic_vals:
                    if obj.entity_type == DroneSemanticSensor.TypeEntity.BOMB:
                        d = float(obj.distance)
                        a = float(obj.angle)
                        if math.isfinite(d) and d <= 42.0:
                            has_nearby_bomb = True
                            wang = normalize_angle(heading + a)
                            nearby_bomb_pt = (my_pos[0] + d * math.cos(wang), my_pos[1] + d * math.sin(wang))
                            break

            near_dc = (self._step - self._last_saw_dc_step) <= 6
            if (not near_dc) and self._omap is not None:
                for hx, hy, _, _ in self._omap.dc_points.values():
                    if math.hypot(hx - my_pos[0], hy - my_pos[1]) < 32.0:
                        near_dc = True
                        break

            k = self._carried_bomb_key
            if near_dc and not has_nearby_bomb:
                # True disposal confirmed: bomb vanished into DC!
                match_r = 45.0 if self.pose.in_no_gps else 28.0
                pickup_pos = self._known_bombs[k]["pos"] if (k is not None and k in self._known_bombs) else None
                my_int_id = int(self.identifier)
                if k is not None and k in self._known_bombs:
                    self._known_bombs[k]["status"] = "DONE"
                    self._known_bombs[k]["claimer"] = None
                    self._known_bombs[k]["done_step"] = self._step
                    self._known_bombs[k]["last_seen"] = self._step
                    self._local_disposed += 1
                # Also kill every duplicate record that was marked CARRIED by me (prevents orphan ghosts!)
                for k2, rec2 in self._known_bombs.items():
                    if rec2["status"] == "CARRIED" and rec2.get("claimer") == my_int_id:
                        rec2["status"] = "DONE"
                        rec2["claimer"] = None
                        rec2["done_step"] = self._step
                        rec2["last_seen"] = self._step
                if pickup_pos is not None:
                    for k2, rec2 in self._known_bombs.items():
                        if math.hypot(rec2["pos"][0] - pickup_pos[0], rec2["pos"][1] - pickup_pos[1]) < match_r:
                            rec2["status"] = "DONE"
                            rec2["claimer"] = None
                            rec2["done_step"] = self._step
                            rec2["last_seen"] = self._step
                self._carried_bomb_key = None
                self._target_bomb_key = None
                self._dc_target_pt = None
                self._dc_hit_pt = None
                self._waypoints = []
                self._explore_target = None
                self._state = STATE_EXPLORE
                self._pos_hist.clear()
                self._unhook_steps = 0
                self._unhook_cooldown_until = self._step + 20
                self._carry_snag_count = 0
                self._carry_snag_pos = None
            else:
                # Dropped / lost grip: bomb is still on the ground!
                drop_pos = nearby_bomb_pt if nearby_bomb_pt is not None else my_pos
                if k is not None and k in self._known_bombs:
                    self._known_bombs[k]["status"] = "ACTIVE"
                    self._known_bombs[k]["pos"] = drop_pos
                    self._known_bombs[k]["claimer"] = int(self.identifier)
                    self._known_bombs[k]["claim_step"] = self._step
                    self._known_bombs[k]["last_seen"] = self._step
                self._carried_bomb_key = None
                self._target_bomb_key = k
                self._dc_target_pt = None
                self._dc_hit_pt = None
                self._waypoints = []
                self._state = STATE_TO_BOMB
                self._pos_hist.clear()
                self._unhook_steps = 0
                self._unhook_cooldown_until = self._step + 20
                self._carry_snag_count = 0
                self._carry_snag_pos = None

        self._was_carrying = carrying

    def _active_peer_tuples(self) -> List[Tuple[int, float, float, float, float, bool, int]]:
        out = []
        for pid, p in self._peers.items():
            if self._step - p["rx_step"] <= 10:
                out.append((pid, p["x"], p["y"], p["vx"], p["vy"], p["carry"], p["tier"]))
        return out

    def _peer_explore_targets(self, my_pos: WorldPoint, semantic_vals: Optional[list]) -> List[WorldPoint]:
        pts: List[WorldPoint] = []
        for p in self._peers.values():
            if self._step - p["rx_step"] <= 12:
                if p.get("goal") is not None:
                    pts.append(tuple(p["goal"]))
                else:
                    pts.append((p["x"], p["y"]))
        # In NO_COM_ZONE, also add semantically visible peer positions for deconfliction
        if self.communicator_is_disabled() and semantic_vals:
            h = self.pose.heading
            for obj in semantic_vals:
                if obj.entity_type == DroneSemanticSensor.TypeEntity.DRONE:
                    d = float(obj.distance)
                    a = float(obj.angle)
                    if math.isfinite(d) and math.isfinite(a):
                        pts.append((my_pos[0] + d * math.cos(h + a), my_pos[1] + d * math.sin(h + a)))
        return pts

    def _is_bomb_contested(self, k: Tuple[int, int], my_pos: WorldPoint) -> bool:
        rec = self._known_bombs.get(k)
        if rec is None:
            return False
        bx, by = rec["pos"]
        my_id = int(self.identifier)
        d_me_euc = math.hypot(bx - my_pos[0], by - my_pos[1])
        d_me = d_me_euc
        if self._state == STATE_TO_BOMB and self._waypoints:
            acc = 0.0
            prev_p = my_pos
            for wp_i in self._waypoints:
                acc += math.hypot(wp_i[0] - prev_p[0], wp_i[1] - prev_p[1])
                prev_p = wp_i
            if acc > d_me:
                d_me = acc
        claim_r = 45.0 if self.pose.in_no_gps else 28.0
        for pid, p in self._peers.items():
            if pid == my_id or (self._step - p["rx_step"] > 10):
                continue
            d_peer_euc = math.hypot(bx - p["x"], by - p["y"])
            p_gd = p.get("gd")
            d_peer = float(p_gd) if (p_gd is not None and float(p_gd) > d_peer_euc) else d_peer_euc
            p_claim = p.get("claim")
            has_claim = False
            if p_claim is not None and isinstance(p_claim, (tuple, list)) and len(p_claim) == 2:
                if math.hypot(p_claim[0] - bx, p_claim[1] - by) < claim_r:
                    has_claim = True
            elif p.get("claim") == k:
                has_claim = True

            if p["carry"]:
                if has_claim or d_peer_euc < 45.0:
                    return True
                continue
            if d_peer_euc < 32.0 and self._omap is not None and self._omap.has_walls_only_los((p["x"], p["y"]), (bx, by)):
                return True
            if has_claim:
                if d_peer < d_me - 12.0 or (abs(d_peer - d_me) <= 12.0 and pid < my_id):
                    return True
            elif p["state"] in (STATE_TO_BOMB, STATE_GRASPING):
                if p.get("goal") is not None:
                    gx, gy = p["goal"]
                    if math.hypot(gx - bx, gy - by) < claim_r and d_peer < d_me - 12.0:
                        return True
        return False

    def _pick_unclaimed_bomb(self, my_pos: WorldPoint) -> Optional[Tuple[int, int]]:
        if self._omap is None:
            return None
        my_id = int(self.identifier)
        active_peers = [
            (pid, p)
            for pid, p in self._peers.items()
            if (self._step - p["rx_step"] <= 10)
        ]

        max_limit = self._misc_data.max_timestep_limit if (self._misc_data and self._misc_data.max_timestep_limit) else 5400
        rem_steps = max_limit - self._step

        match_r = 45.0 if self.pose.in_no_gps else 28.0
        claim_r = 45.0 if self.pose.in_no_gps else 28.0

        # Release claims held by silent peers OR peers that have abandoned this bomb!
        for k, rec in self._known_bombs.items():
            cid = rec.get("claimer")
            if cid is not None and cid != my_id:
                p = self._peers.get(cid)
                if p is not None:
                    bx, by = rec["pos"]
                    p_claim = p.get("claim")
                    peer_still_claims = False
                    if p_claim is not None and isinstance(p_claim, (tuple, list)) and len(p_claim) == 2:
                        if math.hypot(p_claim[0] - bx, p_claim[1] - by) < claim_r:
                            peer_still_claims = True
                    elif p.get("claim") == k:
                        peer_still_claims = True

                    if p["carry"] and math.hypot(p["x"] - bx, p["y"] - by) < claim_r:
                        peer_still_claims = True

                    peer_silent = (self._step - p["rx_step"] > 20)
                    peer_abandoned = (not peer_still_claims and (self._step - rec["claim_step"] > 8))
                    if peer_silent or peer_abandoned:
                        if rec["status"] == "ACTIVE":
                            rec["claimer"] = None
                elif (self._step - rec["claim_step"]) > 35:
                    if rec["status"] == "ACTIVE":
                        rec["claimer"] = None

            if rec["status"] == "CARRIED":
                cid = rec.get("claimer")
                if cid is not None and cid != my_id:
                    p = self._peers.get(cid)
                    if p is not None and (self._step - p["rx_step"] <= 15) and (not p["carry"]):
                        rec["status"] = "ACTIVE"
                        rec["claimer"] = None

        candidates: List[Tuple[float, Tuple[int, int]]] = []
        for k, rec in self._known_bombs.items():
            if rec["status"] != "ACTIVE":
                continue
            if rec.get("claimer") is not None and rec["claimer"] != my_id:
                # Bomb already claimed by an active peer
                continue
            bx, by = rec["pos"]
            is_dup = any(
                k2 != k and rec2["status"] in ("CARRIED", "DONE")
                and math.hypot(rec2["pos"][0] - bx, rec2["pos"][1] - by) < match_r
                for k2, rec2 in self._known_bombs.items()
            )
            if is_dup:
                continue
            d_me = math.hypot(bx - my_pos[0], by - my_pos[1])
            if self._omap.is_goal_unreachable((bx, by), self._step):
                if (self._step - rec.get("direct_seen_step", -999)) > 4:
                    continue

            # Time-budget feasibility: skip bombs that cannot be fetched and delivered before round timeout
            if rem_steps < 200:
                d_dc = 450.0
                if self._omap.dc_points:
                    d_dc = min(math.hypot(bx - ax, by - ay) for _, _, ax, ay in self._omap.dc_points.values())
                t_est = d_me / 9.5 + d_dc / 8.0 + 30.0
                if t_est > rem_steps * 1.25:
                    continue

            # Check if another peer cleanly owns or is heading to this bomb
            contested = False
            for pid, p in active_peers:
                if pid == my_id:
                    continue
                d_peer = math.hypot(bx - p["x"], by - p["y"])
                p_claim = p.get("claim")
                has_claim = False
                if p_claim is not None and isinstance(p_claim, (tuple, list)) and len(p_claim) == 2:
                    if math.hypot(p_claim[0] - bx, p_claim[1] - by) < claim_r:
                        has_claim = True
                elif p.get("claim") == k:
                    has_claim = True

                if p["carry"]:
                    if has_claim or d_peer < 28.0:
                        contested = True
                        break
                    continue
                if d_peer < 32.0:
                    contested = True
                    break
                if has_claim:
                    contested = True
                    break
                elif p["state"] in (STATE_TO_BOMB, STATE_GRASPING):
                    if p.get("goal") is not None and math.hypot(p["goal"][0] - bx, p["goal"][1] - by) < claim_r:
                        contested = True
                        break
            if contested:
                continue

            candidates.append((d_me, k))

        if not candidates:
            return None
        candidates.sort(key=lambda item: item[0])

        # Refine top candidates with fast local A* (euc * 1.15 for distant candidates to keep FPS high)
        best_k: Optional[Tuple[int, int]] = None
        best_score = float("inf")
        door_ok = (self.drone_health >= 15.0)
        for d_euc, k in candidates[:3]:
            bx, by = self._known_bombs[k]["pos"]
            if self._omap.obs_clearance_at(bx, by) < 5.0:
                continue
            if d_euc < 450.0:
                _, path_d, reached = self._omap.plan_path(
                    my_pos,
                    (bx, by),
                    carrying=False,
                    allow_unknown=True,
                    max_expansions=1200,
                    allow_fallback=False,
                    check_goal_los=True,
                    allow_door=door_ok,
                )
                if not reached:
                    continue
                sc = path_d
            else:
                sc = d_euc * 1.15
            if sc < best_score:
                best_score = sc
                best_k = k
        if best_k is not None:
            self._goal_path_dist = best_score
            return best_k
        return None

    def _pick_fanout_target(
        self,
        my_pos: WorldPoint,
        heading: float,
        lidar_vals: Optional[np.ndarray],
        lidar_angles: Optional[np.ndarray],
    ) -> WorldPoint:
        pref_ang = -math.pi + (2.0 * math.pi * (int(self.identifier) % 10)) / 10.0
        best_ang = pref_ang
        best_dist = 90.0

        if lidar_vals is not None and lidar_angles is not None:
            best_score = -1e9
            for d_raw, a_raw in zip(lidar_vals[::3], lidar_angles[::3]):
                d = float(d_raw)
                if not math.isfinite(d) or d < 36.0:
                    continue
                wang = heading + float(a_raw)
                ang_diff = abs(normalize_angle(wang - pref_ang))
                sc = min(d, 190.0) - 42.0 * ang_diff
                if sc > best_score:
                    best_score = sc
                    best_ang = wang
                    best_dist = min(145.0, d - 22.0)

        return (my_pos[0] + best_dist * math.cos(best_ang), my_pos[1] + best_dist * math.sin(best_ang))

    def _pick_open_lidar_dir(
        self,
        my_pos: WorldPoint,
        heading: float,
        lidar_vals: Optional[np.ndarray],
        lidar_angles: Optional[np.ndarray],
    ) -> WorldPoint:
        if lidar_vals is None or lidar_angles is None:
            return (my_pos[0] - 55.0 * math.cos(heading), my_pos[1] - 55.0 * math.sin(heading))
        n = len(lidar_vals)
        best_i = 0
        best_val = -1.0
        for i in range(0, n, 4):
            # Minimum in small angular window ensures wide corridor opening
            lo = max(0, i - 3)
            hi = min(n, i + 4)
            w_min = float(np.nanmin(lidar_vals[lo:hi]))
            if math.isfinite(w_min) and w_min > best_val:
                best_val = w_min
                best_i = i
        wang = heading + float(lidar_angles[best_i])
        step_d = min(75.0, max(35.0, best_val - 18.0))
        return (my_pos[0] + step_d * math.cos(wang), my_pos[1] + step_d * math.sin(wang))

    def _pick_carrier_unhook_dir(
        self,
        my_pos: WorldPoint,
        heading: float,
        fail_ang: float,
        snag_count: int,
        lidar_vals: Optional[np.ndarray],
        lidar_angles: Optional[np.ndarray],
    ) -> WorldPoint:
        """Reverse tethered bomb off a sharp wall tip by pulling opposite to the stalled pull direction."""
        m = snag_count % 3
        if m == 1:
            pref = normalize_angle(fail_ang + math.pi)
        elif m == 2:
            pref = normalize_angle(fail_ang + 2.15)
        else:
            pref = normalize_angle(fail_ang - 2.15)

        if lidar_vals is None or lidar_angles is None:
            return (my_pos[0] + 65.0 * math.cos(pref), my_pos[1] + 65.0 * math.sin(pref))

        n = len(lidar_vals)
        best_ang = pref
        best_d = 45.0
        best_sc = -1e9
        for i in range(0, n, 3):
            lo = max(0, i - 3)
            hi = min(n, i + 4)
            w_min = float(np.nanmin(lidar_vals[lo:hi]))
            if not math.isfinite(w_min) or w_min < 24.0:
                continue
            wang = heading + float(lidar_angles[i])
            adiff = abs(normalize_angle(wang - pref))
            if adiff > 1.35:
                continue
            sc = min(w_min, 115.0) - 52.0 * adiff
            if sc > best_sc:
                best_sc = sc
                best_ang = wang
                best_d = min(82.0, max(42.0, w_min - 18.0))
        if best_sc <= -1e8:
            return self._pick_open_lidar_dir(my_pos, heading, lidar_vals, lidar_angles)
        return (my_pos[0] + best_d * math.cos(best_ang), my_pos[1] + best_d * math.sin(best_ang))

    def _follow_planned_path(
        self,
        my_pos: WorldPoint,
        goal: WorldPoint,
        carrying: bool,
        allow_unknown: bool = True,
        allow_fallback: bool = True,
        check_goal_los: bool = False,
        allow_door: bool = False,
    ) -> WorldPoint:
        assert self._omap is not None
        need_replan = (
            not self._waypoints
            or self._plan_goal is None
            or math.hypot(self._plan_goal[0] - goal[0], self._plan_goal[1] - goal[1]) > 24.0
            or (self._step - self._plan_step) >= 54
            or (
                (self._step - self._plan_step) >= 8
                and self._omap.path_is_obstructed(my_pos, self._waypoints, carrying=carrying)
            )
        )
        if need_replan:
            self._omap.ensure_fresh(self._step, need_wall_edt=self.pose.in_no_gps)
            wps, path_d, rch = self._omap.plan_path(
                my_pos,
                goal,
                carrying=carrying,
                allow_unknown=allow_unknown,
                max_expansions=3500,
                allow_fallback=allow_fallback,
                check_goal_los=check_goal_los,
                allow_door=allow_door,
            )
            if not rch:
                self._omap.mark_goal_unreachable(goal, self._step + 90)
                self._waypoints = []
                self._plan_goal = goal
                self._plan_step = self._step
                return goal
            self._waypoints = wps
            self._goal_path_dist = path_d
            self._plan_goal = goal
            self._plan_step = self._step

        req_seen = carrying and not self.communicator_is_disabled()
        pop_r = 24.0 if carrying else 30.0
        los_clr = 14.5 if carrying else 13.0
        while len(self._waypoints) > 1:
            wp0 = self._waypoints[0]
            wp1 = self._waypoints[1]
            d_wp0 = math.hypot(wp0[0] - my_pos[0], wp0[1] - my_pos[1])
            if d_wp0 < pop_r:
                self._waypoints.pop(0)
                continue
            if carrying:
                dx1 = wp0[0] - my_pos[0]
                dy1 = wp0[1] - my_pos[1]
                dx2 = wp1[0] - wp0[0]
                dy2 = wp1[1] - wp0[1]
                n2 = math.hypot(dx2, dy2)
                if d_wp0 > 1e-3 and n2 > 1e-3 and ((dx1 * dx2 + dy1 * dy2) / (d_wp0 * n2)) < 0.90:
                    break
            if self._omap.has_los(my_pos, wp1, clearance=los_clr, require_seen=req_seen):
                self._waypoints.pop(0)
            else:
                break

        return self._waypoints[0] if self._waypoints else goal

    def _select_best_dc_target(self, my_pos: WorldPoint) -> Tuple[WorldPoint, WorldPoint]:
        """Choose closest reachable open-face Disposal Center approach point."""
        assert self._omap is not None and self._cov is not None
        if not self._omap.dc_points:
            sp = self._spawn_pos if self._spawn_pos is not None else (0.0, 0.0)
            if math.hypot(my_pos[0] - sp[0], my_pos[1] - sp[1]) > 75.0:
                return (sp, sp)
            # Near spawn without having seen DC yet: sweep local frontiers to find DC
            srch = self._cov.pick_exploration_target(
                my_id=int(self.identifier),
                num_drones=10,
                my_pos=my_pos,
                peer_targets=(),
                current_step=self._step,
            )
            return (srch, srch)

        pts = [
            (hx, hy, ax, ay)
            for hx, hy, ax, ay in self._omap.dc_points.values()
            if self._omap.has_walls_only_los((ax, ay), (hx, hy))
            and self._omap.obs_clearance_at(ax, ay) >= 11.5
        ]
        if not pts:
            pts = list(self._omap.dc_points.values())
        pts.sort(key=lambda item: math.hypot(item[2] - my_pos[0], item[3] - my_pos[1]))

        best_app: Optional[WorldPoint] = None
        best_hit: Optional[WorldPoint] = None
        best_cost = float("inf")

        cur_tgt = getattr(self, "_dc_target_pt", None)
        for hx, hy, ax, ay in pts[:2]:
            cost = math.hypot(ax - my_pos[0], ay - my_pos[1])
            if cur_tgt is not None and math.hypot(ax - cur_tgt[0], ay - cur_tgt[1]) < 30.0:
                cost *= 0.82  # Target hysteresis prevents mid-route flip
            if cost < best_cost:
                best_cost = cost
                best_app = (ax, ay)
                best_hit = (hx, hy)

        if best_app is not None and best_hit is not None:
            return (best_app, best_hit)
        hx, hy, ax, ay = pts[0]
        return ((ax, ay), (hx, hy))

    def control(self) -> CommandsDict:
        self._step += 1
        self._init_maps_if_needed()
        assert self._omap is not None and self._cov is not None

        gps_pos = self.measured_gps_position()
        compass_ang = self.measured_compass_angle()
        odom_vals = self.odometer_values()
        lidar_vals = self.lidar_values()
        lidar_angles = self.lidar_rays_angles()
        semantic_vals = self.semantic_values()
        carrying = len(self.grasped_bombs()) > 0

        peer_xy = [
            (float(p["x"]), float(p["y"]))
            for p in self._peers.values()
            if (self._step - int(p["rx_step"])) <= 4
        ]
        px, py, heading = self.pose.update(
            gps_pos=gps_pos,
            compass_ang=compass_ang,
            odom_vals=odom_vals,
            lidar_vals=lidar_vals,
            lidar_angles=lidar_angles,
            semantic_vals=semantic_vals,
            online_map=self._omap,
            peer_xy=peer_xy if peer_xy else None,
        )
        if not self.pose.initialized:
            return empty_command(grasper=1 if carrying else 0)

        my_pos = (px, py)
        if self._spawn_pos is None:
            self._spawn_pos = my_pos

        # Maintain physically traversed breadcrumb corridor inside NO_GPS_ZONE
        if self.pose.gps_settled:
            if len(self._no_gps_breadcrumbs) >= 3:
                self._no_gps_corridor = list(self._no_gps_breadcrumbs)
            self._no_gps_breadcrumbs = [my_pos]
        elif self.pose.in_no_gps and not carrying:
            if (
                not self._no_gps_breadcrumbs
                or math.hypot(px - self._no_gps_breadcrumbs[-1][0], py - self._no_gps_breadcrumbs[-1][1]) >= 60.0
            ):
                if (
                    len(self._no_gps_breadcrumbs) >= 2
                    and math.hypot(px - self._no_gps_breadcrumbs[-2][0], py - self._no_gps_breadcrumbs[-2][1]) < 155.0
                    and self._omap.has_los(self._no_gps_breadcrumbs[-2], my_pos, clearance=13.5)
                ):
                    self._no_gps_breadcrumbs[-1] = my_pos
                else:
                    self._no_gps_breadcrumbs.append(my_pos)
                    if len(self._no_gps_breadcrumbs) > 45:
                        self._no_gps_breadcrumbs = self._no_gps_breadcrumbs[-45:]

        # Update own SLAM occupancy & coverage strictly from own sensors
        gps_trusted = self.pose.gps_settled
        self._omap.update_from_scan(
            pos=my_pos,
            heading=heading,
            lidar_vals=lidar_vals,
            lidar_angles=lidar_angles,
            semantic_vals=semantic_vals,
            gps_trusted=gps_trusted,
            step=self._step,
        )
        self._cov.update_from_pose(
            pos=my_pos,
            heading=heading,
            lidar_vals=lidar_vals,
            lidar_angles=lidar_angles,
            step=self._step,
        )

        # Merge incoming official communicator messages (no-op in NO_COM_ZONE)
        self._process_messages()

        # Process carry transitions before semantic checks so dropped/disposed updates cleanly
        self._handle_carry_transition(carrying, my_pos, semantic_vals=semantic_vals, heading=heading)
        visible_bombs, best_dc_ray = self._observe_semantic(my_pos, heading, semantic_vals, carrying)

        # Record pose history for stuck/snag detection
        self._pos_hist.append(my_pos)
        snag_win = 34 if carrying else 44
        snag_dist = 10.0 if carrying else 8.0
        if (
            self._unhook_steps == 0
            and self._step >= self._unhook_cooldown_until
            and len(self._pos_hist) >= snag_win
            and self._state not in (STATE_GRASPING, STATE_DISPOSING)
        ):
            ox, oy = self._pos_hist[-snag_win]
            if math.hypot(px - ox, py - oy) < snag_dist:
                if carrying:
                    if (
                        self._carry_snag_pos is not None
                        and math.hypot(px - self._carry_snag_pos[0], py - self._carry_snag_pos[1]) < 55.0
                    ):
                        self._carry_snag_count += 1
                    else:
                        self._carry_snag_count = 1
                    self._carry_snag_pos = my_pos
                    if self._waypoints:
                        fail_ang = math.atan2(self._waypoints[0][1] - py, self._waypoints[0][0] - px)
                    elif self._dc_target_pt is not None:
                        fail_ang = math.atan2(self._dc_target_pt[1] - py, self._dc_target_pt[0] - px)
                    else:
                        fail_ang = heading
                    self._unhook_steps = 20
                    self._unhook_cooldown_until = self._step + 28
                    self._unhook_target = self._pick_carrier_unhook_dir(
                        my_pos, heading, fail_ang, self._carry_snag_count, lidar_vals, lidar_angles
                    )
                else:
                    self._unhook_steps = 12
                    self._unhook_cooldown_until = self._step + 34
                    self._unhook_target = self._pick_open_lidar_dir(my_pos, heading, lidar_vals, lidar_angles)
                self._waypoints = []

        if self._unhook_steps > 0:
            self._unhook_steps -= 1
            utgt = self._unhook_target if self._unhook_target is not None else my_pos
            rot_h = None if carrying else normalize_angle(heading + 0.65)
            return drive_to_target(
                pos=my_pos,
                heading=heading,
                vel=(self.pose.vx, self.pose.vy),
                target=utgt,
                desired_speed=self._LOADED_MAX_SPEED if carrying else 6.5,
                max_speed=self._LOADED_MAX_SPEED if carrying else self._UNLOADED_MAX_SPEED,
                carrying=carrying,
                desired_heading=rot_h,
                lidar_vals=lidar_vals,
                lidar_angles=lidar_angles,
                semantic_vals=semantic_vals,
                my_id=int(self.identifier),
                my_priority_tier=self._current_tier(carrying),
                peer_states=self._active_peer_tuples(),
                grasper=1 if carrying else 0,
            )

        peer_tuples = self._active_peer_tuples()

        # ---------------- CARRYING / DISPOSING ----------------
        if carrying:
            # Enter DISPOSING immediately when semantic sensor sees an unobstructed DC face <= 60 px
            if self._state == STATE_CARRYING:
                if (self._step >= self._unhook_cooldown_until) and best_dc_ray is not None and best_dc_ray[1] <= 60.0:
                    self._state = STATE_DISPOSING
                    self._dispose_steps = 0

            if self._state == STATE_DISPOSING:
                self._dispose_steps += 1
                if self._dispose_steps > 85:
                    # Wedged on wrong side / corner: unhook & try another DC entry
                    self._state = STATE_CARRYING
                    self._dispose_steps = 0
                    self._unhook_steps = 14
                    self._unhook_cooldown_until = self._step + 30
                    self._unhook_target = self._pick_open_lidar_dir(my_pos, heading, lidar_vals, lidar_angles)
                    self._dc_target_pt = None
                    self._waypoints = []
                    return drive_to_target(
                        pos=my_pos,
                        heading=heading,
                        vel=(self.pose.vx, self.pose.vy),
                        target=self._unhook_target,
                        desired_speed=4.0,
                        max_speed=self._LOADED_MAX_SPEED,
                        carrying=True,
                        desired_heading=None,
                        lidar_vals=lidar_vals,
                        lidar_angles=lidar_angles,
                        semantic_vals=semantic_vals,
                        my_id=int(self.identifier),
                        my_priority_tier=2,
                        peer_states=peer_tuples,
                        grasper=1,
                    )

                if best_dc_ray is not None:
                    dc_ang, dc_dist = best_dc_ray
                    ux = math.cos(dc_ang)
                    uy = math.sin(dc_ang)
                    push_tgt = (px + (dc_dist + 52.0) * ux, py + (dc_dist + 52.0) * uy)
                    return drive_to_target(
                        pos=my_pos,
                        heading=heading,
                        vel=(self.pose.vx, self.pose.vy),
                        target=push_tgt,
                        desired_speed=5.6,
                        max_speed=self._LOADED_MAX_SPEED,
                        carrying=True,
                        desired_heading=dc_ang,
                        lidar_vals=lidar_vals,
                        lidar_angles=lidar_angles,
                        semantic_vals=semantic_vals,
                        my_id=int(self.identifier),
                        my_priority_tier=3,
                        peer_states=peer_tuples,
                        disposing_mode=True,
                        grasper=1,
                    )

                fallback_hit = self._dc_hit_pt if self._dc_hit_pt is not None else (self._spawn_pos or my_pos)
                ang_to_dc = math.atan2(fallback_hit[1] - py, fallback_hit[0] - px)
                return drive_to_target(
                    pos=my_pos,
                    heading=heading,
                    vel=(self.pose.vx, self.pose.vy),
                    target=fallback_hit,
                    desired_speed=4.8,
                    max_speed=self._LOADED_MAX_SPEED,
                    carrying=True,
                    desired_heading=ang_to_dc,
                    lidar_vals=lidar_vals,
                    lidar_angles=lidar_angles,
                    semantic_vals=semantic_vals,
                    my_id=int(self.identifier),
                    my_priority_tier=3,
                    peer_states=peer_tuples,
                    disposing_mode=True,
                    grasper=1,
                )

            # STATE_CARRYING: follow Reverse Dijkstra DC potential field (or backtrack breadcrumbs if in NO_GPS)
            need_dc_replan = (
                not self._waypoints
                or (
                    (self._step - self._dc_plan_step) >= 8
                    and self._omap.path_is_obstructed(my_pos, self._waypoints, carrying=True)
                )
                or (self._step - self._dc_plan_step) >= 180
            )
            if need_dc_replan:
                self._omap.ensure_fresh(self._step, need_wall_edt=self.pose.in_no_gps)
                fb_spawn = None
                if not self._omap.dc_points and self._spawn_pos is not None:
                    if math.hypot(px - self._spawn_pos[0], py - self._spawn_pos[1]) > 75.0:
                        fb_spawn = self._spawn_pos
                wps_dc, app_pt, hit_pt, ok_dc = self._omap.get_dc_path(
                    my_pos, self._step, fallback_target=fb_spawn
                )
                if ok_dc and wps_dc and app_pt is not None:
                    self._waypoints = wps_dc
                    self._dc_target_pt = app_pt
                    self._dc_hit_pt = hit_pt
                    self._plan_goal = app_pt
                    self._plan_step = self._step
                    self._dc_plan_step = self._step
                else:
                    self._dc_target_pt, self._dc_hit_pt = self._select_best_dc_target(my_pos)
                    self._dc_plan_step = self._step
                    tgt_fb = self._dc_target_pt if self._dc_target_pt is not None else (self._spawn_pos or my_pos)

                    trail = self._no_gps_breadcrumbs if (len(self._no_gps_breadcrumbs) >= 2) else self._no_gps_corridor
                    used_trail = False
                    if self.pose.in_no_gps and len(trail) >= 2:
                        d_start = math.hypot(trail[0][0] - tgt_fb[0], trail[0][1] - tgt_fb[1])
                        d_now = math.hypot(px - tgt_fb[0], py - tgt_fb[1])
                        if d_start < d_now - 15.0 and math.hypot(trail[-1][0] - px, trail[-1][1] - py) < 120.0:
                            closest_idx = min(
                                range(len(trail)),
                                key=lambda i: math.hypot(trail[i][0] - px, trail[i][1] - py)
                            )
                            next_idx = max(0, closest_idx - 1) if (closest_idx > 0 and math.hypot(trail[closest_idx][0] - px, trail[closest_idx][1] - py) < 25.0) else closest_idx
                            self._waypoints = [trail[i] for i in range(next_idx, -1, -1)]
                            self._plan_goal = self._waypoints[-1]
                            self._plan_step = self._step
                            used_trail = True

                    if not used_trail:
                        wps_fb, _, ok_fb = self._omap.plan_path(
                            my_pos,
                            tgt_fb,
                            carrying=True,
                            allow_unknown=True,
                            allow_fallback=True,
                            allow_door=True,
                            max_expansions=4500,
                        )
                        if ok_fb and wps_fb:
                            self._waypoints = wps_fb
                            self._plan_goal = tgt_fb
                            self._plan_step = self._step

            if self._waypoints:
                req_seen = carrying and not self.communicator_is_disabled()
                while len(self._waypoints) > 1:
                    wp0 = self._waypoints[0]
                    wp1 = self._waypoints[1]
                    d_wp0 = math.hypot(wp0[0] - px, wp0[1] - py)
                    if d_wp0 < 22.0:
                        self._waypoints.pop(0)
                        continue
                    dx1 = wp0[0] - px
                    dy1 = wp0[1] - py
                    dx2 = wp1[0] - wp0[0]
                    dy2 = wp1[1] - wp0[1]
                    n2 = math.hypot(dx2, dy2)
                    is_turn = False
                    if d_wp0 > 1e-3 and n2 > 1e-3:
                        dot_t = (dx1 * dx2 + dy1 * dy2) / (d_wp0 * n2)
                        if dot_t < 0.90:
                            is_turn = True
                    if not is_turn and self._omap.has_los(my_pos, wp1, clearance=14.5, require_seen=req_seen):
                        self._waypoints.pop(0)
                    else:
                        break
                wp = self._waypoints[0]
            else:
                wps_dc, app_pt, hit_pt, ok_dc = self._omap.get_dc_path(my_pos, self._step, fallback_target=self._spawn_pos)
                if ok_dc and wps_dc:
                    self._waypoints = wps_dc
                    wp = self._waypoints[0]
                else:
                    tgt = self._dc_target_pt if self._dc_target_pt is not None else (self._spawn_pos or my_pos)
                    wp = self._follow_planned_path(my_pos, tgt, carrying=True, allow_unknown=True)

            return drive_to_target(
                pos=my_pos,
                heading=heading,
                vel=(self.pose.vx, self.pose.vy),
                target=wp,
                desired_speed=self._LOADED_MAX_SPEED,
                max_speed=self._LOADED_MAX_SPEED,
                carrying=True,
                desired_heading=None,
                lidar_vals=lidar_vals,
                lidar_angles=lidar_angles,
                semantic_vals=semantic_vals,
                my_id=int(self.identifier),
                my_priority_tier=2,
                peer_states=peer_tuples,
                grasper=1,
            )

        # ---------------- UNLOADED: FANOUT / TO_BOMB / GRASPING / EXPLORE ----------------
        # Early fan-out for initial swarm separation (brief 8-step scatter)
        if self._state == STATE_FANOUT:
            if self._step >= 18:
                self._state = STATE_EXPLORE
            else:
                # Immediately claim an uncontested nearby bomb if one appears right at spawn
                b_key = self._pick_unclaimed_bomb(my_pos)
                if b_key is not None:
                    bpos = self._known_bombs[b_key]["pos"]
                    if math.hypot(bpos[0] - my_pos[0], bpos[1] - my_pos[1]) < 115.0:
                        self._target_bomb_key = b_key
                        self._known_bombs[b_key]["claimer"] = int(self.identifier)
                        self._known_bombs[b_key]["claim_step"] = self._step
                        self._state = STATE_TO_BOMB
                        self._waypoints = []

        if self._state == STATE_FANOUT:
            if self._fanout_target is None or (self._step % 6 == 1):
                self._fanout_target = self._pick_fanout_target(my_pos, heading, lidar_vals, lidar_angles)
            # Gentle separation speed on steps 1-6 so clustered swarm fans out cleanly without hull damage
            fan_spd = 4.2 if self._step <= 18 else self._UNLOADED_MAX_SPEED
            return drive_to_target(
                pos=my_pos,
                heading=heading,
                vel=(self.pose.vx, self.pose.vy),
                target=self._fanout_target,
                desired_speed=fan_spd,
                max_speed=self._UNLOADED_MAX_SPEED,
                carrying=False,
                desired_heading=None,
                lidar_vals=lidar_vals,
                lidar_angles=lidar_angles,
                semantic_vals=semantic_vals,
                my_id=int(self.identifier),
                my_priority_tier=0,
                peer_states=peer_tuples,
                grasper=0,
            )

        # Re-check bomb assignment whenever exploring (throttled when no direct sighting for fast FPS)
        if self._state == STATE_EXPLORE:
            if visible_bombs or self._got_new_bomb_msg or ((self._step + int(self.identifier)) % 4 == 0):
                b_key = self._pick_unclaimed_bomb(my_pos)
                if b_key is not None:
                    self._target_bomb_key = b_key
                    self._known_bombs[b_key]["claimer"] = int(self.identifier)
                    self._known_bombs[b_key]["claim_step"] = self._step
                    self._state = STATE_TO_BOMB
                    self._waypoints = []
        elif self._state == STATE_TO_BOMB:
            # Drop stale/completed/unreachable target or switch if contested / closer unclaimed bomb appears
            cur = self._known_bombs.get(self._target_bomb_key) if self._target_bomb_key is not None else None
            if (
                cur is None
                or cur["status"] != "ACTIVE"
                or self._omap.is_goal_unreachable(cur["pos"], self._step)
            ):
                if self._target_bomb_key in self._known_bombs and self._known_bombs[self._target_bomb_key].get("claimer") == int(self.identifier):
                    self._known_bombs[self._target_bomb_key]["claimer"] = None
                self._target_bomb_key = None
                self._state = STATE_EXPLORE
                self._waypoints = []
            elif (self._step % 4 == 0) and self._target_bomb_key is not None and self._is_bomb_contested(self._target_bomb_key, my_pos):
                if self._target_bomb_key in self._known_bombs and self._known_bombs[self._target_bomb_key].get("claimer") == int(self.identifier):
                    self._known_bombs[self._target_bomb_key]["claimer"] = None
                old_k = self._target_bomb_key
                self._target_bomb_key = None
                better_k = self._pick_unclaimed_bomb(my_pos)
                if better_k is not None and better_k != old_k:
                    self._target_bomb_key = better_k
                    self._waypoints = []
                else:
                    self._state = STATE_EXPLORE
                    self._waypoints = []
            elif self._step % 16 == 0:
                better_k = self._pick_unclaimed_bomb(my_pos)
                if better_k is not None and better_k != self._target_bomb_key and cur is not None:
                    cur_d = math.hypot(cur["pos"][0] - my_pos[0], cur["pos"][1] - my_pos[1])
                    new_p = self._known_bombs[better_k]["pos"]
                    new_d = math.hypot(new_p[0] - my_pos[0], new_p[1] - my_pos[1])
                    if new_d + 45.0 < cur_d:
                        if self._target_bomb_key in self._known_bombs and self._known_bombs[self._target_bomb_key].get("claimer") == int(self.identifier):
                            self._known_bombs[self._target_bomb_key]["claimer"] = None
                        self._target_bomb_key = better_k
                        self._waypoints = []
            if self._target_bomb_key in self._known_bombs:
                cur_b = self._known_bombs[self._target_bomb_key]
                if cur_b["status"] == "ACTIVE" or cur_b.get("claimer") == int(self.identifier):
                    cur_b["claimer"] = int(self.identifier)
                    cur_b["claim_step"] = self._step

        # Precision GRASPING
        if self._state in (STATE_TO_BOMB, STATE_GRASPING) and self._target_bomb_key in self._known_bombs:
            rec = self._known_bombs[self._target_bomb_key]
            if rec["status"] != "ACTIVE":
                self._state = STATE_EXPLORE
                self._target_bomb_key = None
                self._waypoints = []
                return empty_command(grasper=0)

            bpos = rec["pos"]
            # Snap to live visible semantic bomb within range (cluster-safe 32 px in GPS, 65 px in NO_GPS)
            live_pt = bpos
            best_vd = 85.0 if self.pose.in_no_gps else 42.0
            has_live = False
            for vp in visible_bombs:
                vd = math.hypot(vp[0] - bpos[0], vp[1] - bpos[1])
                if vd < best_vd:
                    best_vd = vd
                    live_pt = vp
                    has_live = True

            # If any peer is already carrying THIS bomb, abort immediately (cluster-safe 38 px radius)
            claim_abort_r = 45.0 if self.pose.in_no_gps else 38.0
            carrier_peer = any(
                p["carry"] and (
                    p.get("claim") == self._target_bomb_key
                    or rec.get("claimer") == pid
                    or (isinstance(p.get("claim"), (tuple, list)) and len(p["claim"]) == 2 and math.hypot(p["claim"][0] - live_pt[0], p["claim"][1] - live_pt[1]) < claim_abort_r)
                    or math.hypot(p["x"] - live_pt[0], p["y"] - live_pt[1]) < 38.0
                )
                for pid, p in self._peers.items()
                if (self._step - p["rx_step"] <= 15)
            )
            if carrier_peer:
                self._state = STATE_EXPLORE
                self._target_bomb_key = None
                self._waypoints = []
                return empty_command(grasper=0)

            dist_b = math.hypot(live_pt[0] - px, live_pt[1] - py)
            has_los_to_b = has_live or self._omap.has_walls_only_los(my_pos, live_pt)
            near_peer_to_b = any(
                math.hypot(p["x"] - live_pt[0], p["y"] - live_pt[1]) < 32.0
                and (
                    p["carry"]
                    or (
                        p["state"] == STATE_GRASPING
                        and (
                            math.hypot(p["x"] - live_pt[0], p["y"] - live_pt[1]) < dist_b - 4.0
                            or (abs(math.hypot(p["x"] - live_pt[0], p["y"] - live_pt[1]) - dist_b) <= 4.0 and pid < int(self.identifier))
                        )
                    )
                )
                for pid, p in self._peers.items()
                if (self._step - p["rx_step"] <= 10)
            )

            if self._state == STATE_TO_BOMB and has_los_to_b and dist_b < (54.0 if has_live else 44.0):
                self._state = STATE_GRASPING
                self._grasp_steps = 0
                self._pos_hist.clear()
            elif self._state == STATE_GRASPING and not has_los_to_b:
                # A wall separates us from target (e.g. thin maze wall): fall back to A* TO_BOMB!
                self._state = STATE_TO_BOMB
                self._grasp_steps = 0
                self._waypoints = []

            if self._state == STATE_GRASPING:
                # Direct ghost extinction: ONLY if clear walls-only LOS to empty floor with no live bomb!
                # (14 steps in GPS, 36 steps in NO_GPS so drift is tolerated yet NO_GPS ghosts cannot trap the swarm)
                if (not has_live) and dist_b < 52.0:
                    ghost_lim = 36 if self.pose.in_no_gps else 14
                    if self._grasp_steps >= ghost_lim:
                        rec["status"] = "DONE"
                        rec["claimer"] = None
                        rec["done_step"] = self._step
                        rec["last_seen"] = self._step
                        self._state = STATE_EXPLORE
                        self._target_bomb_key = None
                        self._grasp_steps = 0
                        self._waypoints = []
                        return empty_command(grasper=0)

                # Anti-tug-of-war: abort only if another peer already grasped or is closer & grasping
                peer_contested = False
                for pid, p in self._peers.items():
                    if pid == int(self.identifier) or (self._step - p["rx_step"] > 10):
                        continue
                    p_dist = math.hypot(p["x"] - live_pt[0], p["y"] - live_pt[1])
                    p_claim = p.get("claim")
                    claim_match = (p_claim == self._target_bomb_key) or (
                        p_claim is not None and isinstance(p_claim, (tuple, list)) and len(p_claim) == 2 and math.hypot(p_claim[0] - live_pt[0], p_claim[1] - live_pt[1]) < claim_abort_r
                    )
                    if p["carry"] and (claim_match or p_dist < 24.0):
                        peer_contested = True
                        break
                    if p["state"] == STATE_GRASPING and p_dist < dist_b - 6.0 and p_dist < 42.0:
                        peer_contested = True
                        break

                if peer_contested or rec["status"] != "ACTIVE":
                    self._state = STATE_EXPLORE
                    if self._target_bomb_key in self._known_bombs:
                        self._known_bombs[self._target_bomb_key]["claimer"] = None
                    self._target_bomb_key = None
                    self._grasp_steps = 0
                    self._waypoints = []
                    self._unhook_steps = 6
                    self._unhook_cooldown_until = self._step + 15
                    return empty_command(grasper=0)

                self._grasp_steps += 1
                if self._grasp_steps > 54:
                    # Failed grasp: if no live bomb seen, mark DONE; if live corner bomb wedged, back off & retry
                    self._state = STATE_EXPLORE
                    if self._target_bomb_key in self._known_bombs:
                        self._known_bombs[self._target_bomb_key]["claimer"] = None
                        if (not has_live) and has_los_to_b and (not self.pose.in_no_gps) and self.pose.gps_settled:
                            rec["status"] = "DONE"
                            rec["done_step"] = self._step
                            rec["last_seen"] = self._step
                        elif self._omap is not None:
                            self._omap.mark_goal_unreachable(live_pt, self._step + 90)
                        elif self._omap is not None:
                            self._omap.mark_goal_unreachable(live_pt, self._step + 35)
                    self._target_bomb_key = None
                    self._grasp_steps = 0
                    self._unhook_steps = 9
                    self._unhook_cooldown_until = self._step + 20
                    self._unhook_target = self._pick_open_lidar_dir(my_pos, heading, lidar_vals, lidar_angles)
                    return empty_command(grasper=0)

                grasp_on = 1 if (dist_b < 34.0 and not near_peer_to_b and not carrier_peer) else 0
                if dist_b <= 20.0:
                    spd = 0.0  # Firmly in grasping pocket; hold steady without slamming wall
                elif dist_b <= 30.0:
                    spd = 2.7  # Smooth slide into tight 90-deg corner bombs without wall scraping
                elif dist_b <= 45.0:
                    spd = 5.4
                else:
                    spd = 8.4
                ang_to_b = math.atan2(live_pt[1] - py, live_pt[0] - px)
                return drive_to_target(
                    pos=my_pos,
                    heading=heading,
                    vel=(self.pose.vx, self.pose.vy),
                    target=live_pt,
                    desired_speed=spd,
                    max_speed=self._UNLOADED_MAX_SPEED,
                    carrying=False,
                    desired_heading=ang_to_b,
                    lidar_vals=lidar_vals,
                    lidar_angles=lidar_angles,
                    semantic_vals=semantic_vals,
                    my_id=int(self.identifier),
                    my_priority_tier=1,
                    peer_states=peer_tuples,
                    grasping_mode=True,
                    grasper=grasp_on,
                )

            # STATE_TO_BOMB navigation
            wp = self._follow_planned_path(
                my_pos,
                live_pt,
                carrying=False,
                allow_unknown=True,
                allow_fallback=False,
                check_goal_los=(not has_live),
                allow_door=(self.drone_health >= 15.0),
            )
            if (not self._waypoints) or (
                len(self._waypoints) == 1
                and math.hypot(wp[0] - px, wp[1] - py) < 22.0
                and dist_b > 55.0
            ):
                self._omap.mark_goal_unreachable(live_pt, self._step + 90)
                if self._target_bomb_key in self._known_bombs and self._known_bombs[self._target_bomb_key].get("claimer") == int(self.identifier):
                    self._known_bombs[self._target_bomb_key]["claimer"] = None
                self._target_bomb_key = None
                self._state = STATE_EXPLORE
                self._waypoints = []
                return empty_command(grasper=0)
            spd = self._UNLOADED_MAX_SPEED if dist_b > 42.0 else max(5.5, dist_b * 0.32)
            return drive_to_target(
                pos=my_pos,
                heading=heading,
                vel=(self.pose.vx, self.pose.vy),
                target=wp,
                desired_speed=spd,
                max_speed=self._UNLOADED_MAX_SPEED,
                carrying=False,
                desired_heading=None,
                lidar_vals=lidar_vals,
                lidar_angles=lidar_angles,
                semantic_vals=semantic_vals,
                my_id=int(self.identifier),
                my_priority_tier=1,
                peer_states=peer_tuples,
                grasper=1 if (dist_b < 24.0 and not near_peer_to_b and not carrier_peer) else 0,
            )

        # STATE_EXPLORE: coordinated 1-pass BFS frontier sweeping
        self._state = STATE_EXPLORE
        need_new_exp = (
            self._explore_target is None
            or not self._waypoints
            or (self._step - self._explore_set_step) >= 56
            or math.hypot(self._explore_target[0] - px, self._explore_target[1] - py) < 40.0
            or self._omap.is_goal_unreachable(self._explore_target, self._step)
        )
        if not need_new_exp and self._explore_target is not None and not self._explore_is_stale:
            er, ec = self._cov.world_to_cell(*self._explore_target)
            if self._cov.seen[er, ec] and (
                (self._step - self._explore_set_step) >= 28
                or math.hypot(self._explore_target[0] - px, self._explore_target[1] - py) < 45.0
            ):
                need_new_exp = True

        if need_new_exp:
            self._omap.ensure_fresh(self._step, need_wall_edt=self.pose.in_no_gps)
            comm_lost = self.communicator_is_disabled() or (
                self._step > 35
                and not any(self._step - p["rx_step"] <= 15 for p in self._peers.values())
            )
            vn = math.hypot(self.pose.vx, self.pose.vy)
            vdir = (self.pose.vx / vn, self.pose.vy / vn) if vn > 3.0 else None
            busy_tiles = {
                pid % 10
                for pid, p in self._peers.items()
                if p["carry"] and (self._step - p["rx_step"] <= 15)
            }
            tgt, wps = self._cov.pick_exploration_with_path(
                my_id=int(self.identifier),
                num_drones=10,
                my_pos=my_pos,
                peer_targets=self._peer_explore_targets(my_pos, semantic_vals),
                current_step=self._step,
                comm_lost=comm_lost,
                vel_dir=vdir,
                drone_health=self.drone_health,
                in_no_gps=self.pose.in_no_gps,
                comm_disabled=self.communicator_is_disabled(),
                orphan_tiles=busy_tiles,
            )
            self._explore_target = tgt
            self._explore_set_step = self._step
            er_t, ec_t = self._cov.world_to_cell(*tgt)
            self._explore_is_stale = bool(self._cov.seen[er_t, ec_t])
            self._waypoints = wps
            self._plan_goal = tgt
            self._plan_step = self._step

        assert self._explore_target is not None
        wp = self._follow_planned_path(
            my_pos,
            self._explore_target,
            carrying=False,
            allow_unknown=True,
            allow_fallback=True,
            allow_door=(self.drone_health >= 15.0),
        )
        # Health-preserving cruise moderation without shutting down late-game scouts:
        in_open = (self._omap is not None and self._omap.obs_clearance_at(px, py) > 38.0)
        base_exp_spd = 13.5 if in_open else self._UNLOADED_MAX_SPEED
        exp_spd = 7.5 if self.drone_health <= 11.0 else (8.8 if self._explore_is_stale else base_exp_spd)
        return drive_to_target(
            pos=my_pos,
            heading=heading,
            vel=(self.pose.vx, self.pose.vy),
            target=wp,
            desired_speed=exp_spd,
            max_speed=13.5 if in_open else self._UNLOADED_MAX_SPEED,
            carrying=False,
            desired_heading=None,
            lidar_vals=lidar_vals,
            lidar_angles=lidar_angles,
            semantic_vals=semantic_vals,
            my_id=int(self.identifier),
            my_priority_tier=0,
            peer_states=peer_tuples,
            grasper=0,
        )
