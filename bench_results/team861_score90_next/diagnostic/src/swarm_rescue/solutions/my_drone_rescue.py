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

    _UNLOADED_MAX_SPEED = 11.4
    _LOADED_MAX_SPEED = 3.82

    def __init__(
        self,
        identifier: Optional[int] = None,
        misc_data: Optional[MiscData] = None,
        **kwargs: Any,
    ) -> None:
        super().__init__(identifier=identifier, misc_data=misc_data, **kwargs)
        self.pose = PoseEstimator()
        self._had_gps_outage = False
        self._peer_gps_outage = False

        self._omap: Optional[OnlineMap] = None
        self._cov: Optional[CoverageTracker] = None

        self._step = 0
        self._step_limit = misc_data.max_timestep_limit if misc_data is not None else None
        self._state = STATE_FANOUT
        self._spawn_pos: Optional[WorldPoint] = None
        self._fanout_target: Optional[WorldPoint] = None
        self._explore_target: Optional[WorldPoint] = None
        self._explore_set_step = -100
        self._explore_is_stale = False

        # Active target bomb tracking
        self._target_bomb_key: Optional[Tuple[int, int]] = None
        self._carried_bomb_key: Optional[Tuple[int, int]] = None
        self._load_origin: Optional[WorldPoint] = None
        self._yielded_load_to: Optional[int] = None
        self._grasp_steps = 0
        self._dispose_steps = 0
        self._failed_docks = 0
        self._grasp_bearing: Optional[float] = None
        self._grasp_bearing_step = -100
        self._dc_target_pt: Optional[WorldPoint] = None
        self._dc_hit_pt: Optional[WorldPoint] = None
        self._dc_plan_step = -100
        self._time_weighted_return = False
        self._last_return_comparison = -100

        # Path cache
        self._waypoints: List[WorldPoint] = []
        self._plan_step = -100
        self._plan_goal: Optional[WorldPoint] = None

        # Carrying / disposal state transition detection
        self._was_carrying = False
        self._last_saw_dc_step = -100
        self._local_disposed = 0
        self._pickup_step = -100
        self._near_peer_carry_steps = 0

        # Anti-snag / unhook recovery & NO_GPS breadcrumb trail
        self._pos_hist: Deque[WorldPoint] = deque(maxlen=46)
        self._no_gps_breadcrumbs: List[WorldPoint] = []
        self._unhook_steps = 0
        self._unhook_target: Optional[WorldPoint] = None
        self._unhook_cooldown_until = 0
        self._return_checkpoint: Optional[WorldPoint] = None
        self._return_checkpoint_step = 0
        self._return_escape_until = 0
        self._return_escape_headings: Deque[float] = deque(maxlen=4)
        self._last_return_recovery: Optional[WorldPoint] = None

        # Dynamic registry of known bombs discovered by semantic sensor / peers
        # key -> {pos, status ("ACTIVE"|"CARRIED"|"DONE"), last_seen, claimer, claim_step, miss_cnt}
        self._known_bombs: Dict[Tuple[int, int], Dict[str, Any]] = {}

        # Peer telemetry from official communicator (cleared/aged naturally)
        self._peers: Dict[int, Dict[str, Any]] = {}

    def define_message_for_all(self) -> Any:
        if not self.pose.initialized or self._omap is None or self._cov is None:
            return None

        bombs_msg = []
        # Prioritize ACTIVE and CARRIED bombs before DONE so radio capacity never drops live bombs
        sorted_bombs = sorted(
            self._known_bombs.items(),
            key=lambda kv: (0 if kv[1]["status"] == "ACTIVE" else (1 if kv[1]["status"] == "CARRIED" else 2)),
        )
        for k, b in sorted_bombs[:18]:
            bx, by = b["pos"]
            bombs_msg.append(
                (
                    k,
                    round(bx, 1),
                    round(by, 1),
                    b["status"],
                    int(b["last_seen"]),
                    b["claimer"],
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

        return {
            "id": int(self.identifier),
            "step": self._step,
            "x": round(self.pose.x, 2),
            "y": round(self.pose.y, 2),
            "vx": round(self.pose.vx, 2),
            "vy": round(self.pose.vy, 2),
            "gps_ok": self.pose.gps_settled,
            "gps_outage": self._had_gps_outage,
            "carry": bool(self._was_carrying),
            "load": self._load_origin if self._was_carrying else None,
            "tier": self._current_tier(self._was_carrying),
            "state": self._state,
            "goal": active_goal,
            "claim": self._target_bomb_key if self._state in (STATE_TO_BOMB, STATE_GRASPING) else None,
            "bombs": bombs_msg,
            "dc": dc_msg,
            "walls": wall_chunk,
            "cov": cov_bytes,
        }

    def _current_tier(self, carrying: bool) -> int:
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
        self._cov = CoverageTracker(self._omap)

    def _process_messages(self) -> None:
        if self.communicator_is_disabled() or self._omap is None or self._cov is None:
            return

        for item in self.communicator.received_messages:
            msg = item[1] if isinstance(item, (tuple, list)) and len(item) >= 2 else item
            if not isinstance(msg, dict) or "id" not in msg:
                continue
            pid = int(msg["id"])
            if pid == int(self.identifier):
                continue

            self._peer_gps_outage = self._peer_gps_outage or bool(msg.get("gps_outage", False))

            self._peers[pid] = {
                "x": float(msg["x"]),
                "y": float(msg["y"]),
                "vx": float(msg["vx"]),
                "vy": float(msg["vy"]),
                "carry": bool(msg["carry"]),
                "load": msg.get("load"),
                "tier": int(msg["tier"]),
                "state": int(msg["state"]),
                "gps_ok": bool(msg.get("gps_ok", False)),
                "goal": msg.get("goal"),
                "claim": msg.get("claim"),
                "rx_step": self._step,
            }

            if msg.get("dc"):
                self._omap.merge_peer_dc_points(msg["dc"])
            if msg.get("walls"):
                # wall_chunk strictly contains GPS-trusted _own_wall_list cells
                self._omap.merge_peer_walls(msg["walls"], peer_gps_trusted=True)
            if msg.get("cov"):
                c_mask = self._cov.merge_packed(msg["cov"], self._step)
                if c_mask is not None and bool(msg.get("gps_ok", False)):
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

    def _find_matching_bomb_key(self, pos: WorldPoint, max_r: float = 26.0) -> Optional[Tuple[int, int]]:
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
        k = self._find_matching_bomb_key(pos, max_r=26.0)
        if k is None:
            k = (int(round(pos[0] / 18.0)), int(round(pos[1] / 18.0)))
            while k in self._known_bombs:
                k = (k[0], k[1] + 1)
            self._known_bombs[k] = {
                "pos": pos,
                "status": status,
                "last_seen": seen_step,
                "claimer": claimer,
                "claim_step": claim_step,
                "miss_cnt": 0,
                "direct_seen": seen_step if from_direct_sensor else -1,
            }
            return k

        rec = self._known_bombs[k]
        if from_direct_sensor:
            rec["direct_seen"] = seen_step
        if status == "DONE":
            # Only accept DONE if recent or never contradicted by fresh direct sighting
            if not (from_direct_sensor and seen_step > rec["last_seen"]):
                rec["status"] = "DONE"
                rec["claimer"] = None
        elif rec["status"] == "DONE":
            # Direct sensor ungrasped sighting >15 steps after marked DONE heals transient error
            if from_direct_sensor and (seen_step - rec["last_seen"] > 15):
                rec["status"] = "ACTIVE"
                rec["pos"] = pos
                rec["last_seen"] = seen_step
                rec["miss_cnt"] = 0
        else:
            if seen_step >= rec["last_seen"]:
                ox, oy = rec["pos"]
                w = 0.35 if from_direct_sensor else 0.20
                rec["pos"] = ((1.0 - w) * ox + w * pos[0], (1.0 - w) * oy + w * pos[1])
                rec["last_seen"] = seen_step
                rec["miss_cnt"] = 0
            if status == "CARRIED" and claim_step >= rec["claim_step"]:
                rec["status"] = "CARRIED"
                rec["claimer"] = claimer
                rec["claim_step"] = claim_step
            elif status == "ACTIVE" and claimer is not None and claim_step > rec["claim_step"]:
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
                    if carrying and math.hypot(bx - my_pos[0], by - my_pos[1]) < 34.0:
                        continue
                    near_peer = any(math.hypot(bx - dx, by - dy) < 28.0 for dx, dy in vis_drones)
                    if near_peer:
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

        # Negative observation check: verify bombs expected nearby with clear wall+drone LOS
        if (not carrying) and self._omap is not None:
            for k, rec in self._known_bombs.items():
                if rec["status"] != "ACTIVE":
                    continue
                bx, by = rec["pos"]
                dist = math.hypot(bx - my_pos[0], by - my_pos[1])
                if dist < 56.0 and self._omap.has_walls_only_los(my_pos, (bx, by)):
                    ang_b = math.atan2(by - my_pos[1], bx - my_pos[0])
                    if any(abs(normalize_angle(ang_b - da)) < 0.25 for da in vis_drone_angs):
                        continue
                    seen_here = any(math.hypot(vx - bx, vy - by) < 34.0 for vx, vy in visible_bombs)
                    if seen_here:
                        rec["miss_cnt"] = 0
                    else:
                        rec["miss_cnt"] += 1
                        if rec["miss_cnt"] >= (12 if self.pose.in_no_gps else 6):
                            rec["status"] = "DONE"
                            rec["claimer"] = None
                            rec["last_seen"] = self._step

        return (visible_bombs, best_dc_ray)

    def _handle_carry_transition(
        self,
        carrying: bool,
        my_pos: WorldPoint,
    ) -> None:
        if carrying and not self._was_carrying:
            # Just grasped a bomb!
            self._state = STATE_CARRYING
            self._dispose_steps = 0
            self._failed_docks = 0
            self._grasp_steps = 0
            self._waypoints = []
            self._dc_target_pt = None
            self._dc_hit_pt = None
            self._dc_plan_step = -100
            self._time_weighted_return = False
            self._last_return_comparison = -100
            self._pickup_step = self._step
            if self._step - self._grasp_bearing_step > 3:
                self._grasp_bearing = None
            self._near_peer_carry_steps = 0
            self._return_checkpoint = my_pos
            self._return_checkpoint_step = self._step
            self._return_escape_until = 0
            self._return_escape_headings.clear()
            self._last_return_recovery = None
            if self._omap is not None:
                self._omap.set_return_detour(None)

            k = self._target_bomb_key
            if k is None or k not in self._known_bombs:
                k = self._find_matching_bomb_key(my_pos, max_r=45.0)
            self._carried_bomb_key = k
            if k is not None and k in self._known_bombs:
                pos = self._known_bombs[k]["pos"]
                if self.pose.gps_settled and math.dist(my_pos, pos) < 55.0:
                    self._load_origin = pos
                self._known_bombs[k]["status"] = "CARRIED"
                self._known_bombs[k]["claimer"] = int(self.identifier)
                self._known_bombs[k]["claim_step"] = self._step

        elif (not carrying) and self._was_carrying:
            # Stopped carrying (either disposed at DC or lost grip)
            near_dc = (self._step - self._last_saw_dc_step) <= 8
            if (not near_dc) and self._omap is not None:
                for hx, hy, _, _ in self._omap.dc_points.values():
                    if math.hypot(hx - my_pos[0], hy - my_pos[1]) < 56.0:
                        near_dc = True
                        break

            k = self._carried_bomb_key
            if k is not None and k in self._known_bombs:
                if self._yielded_load_to is not None:
                    self._known_bombs[k]["claimer"] = self._yielded_load_to
                elif near_dc:
                    self._known_bombs[k]["status"] = "DONE"
                    self._known_bombs[k]["claimer"] = None
                    self._known_bombs[k]["last_seen"] = self._step
                    self._local_disposed += 1
                else:
                    self._known_bombs[k]["status"] = "ACTIVE"
                    self._known_bombs[k]["pos"] = my_pos
                    self._known_bombs[k]["claimer"] = None
                    self._known_bombs[k]["last_seen"] = self._step

            self._carried_bomb_key = None
            self._load_origin = None
            self._yielded_load_to = None
            self._target_bomb_key = None
            self._dc_target_pt = None
            self._dc_hit_pt = None
            self._waypoints = []
            self._explore_target = None
            self._state = STATE_EXPLORE

        self._was_carrying = carrying

    def _active_peer_tuples(self) -> List[Tuple[int, float, float, float, float, bool, int]]:
        # Inside NO_GPS_ZONE, peer world coordinates can be mutually offset by drift;
        # semantic DRONE detections handle close-range avoidance with zero drift.
        if self.pose.in_no_gps:
            return []
        out = []
        for pid, p in self._peers.items():
            if self._step - p["rx_step"] <= 10 and p.get("gps_ok", False):
                out.append((pid, p["x"], p["y"], p["vx"], p["vy"], p["carry"], p["tier"]))
        return out

    def _shared_load_owner(self, position: WorldPoint) -> Optional[int]:
        """Identify a higher-priority carrier of this load using recent radio reports."""
        origin = self._load_origin
        if origin is None or not self.pose.gps_settled or self.communicator_is_disabled():
            return None
        if self._failed_docks == 0 and (
            len(self._pos_hist) < 44 or math.dist(self._pos_hist[0], position) >= 25.0
        ):
            return None
        if self._step - self._pickup_step < 72:
            return None
        my_id = int(self.identifier)
        for pid, peer in self._peers.items():
            load = peer.get("load")
            if (
                pid < my_id and peer["carry"] and peer["gps_ok"] and load is not None
                and self._step - peer["rx_step"] <= 4
                and math.dist(origin, load) < 20.0
                and math.dist(position, (peer["x"], peer["y"])) < 88.0
            ):
                return pid
        return None

    def _peer_explore_targets(self, my_pos: WorldPoint, semantic_vals: Optional[list]) -> List[WorldPoint]:
        pts: List[WorldPoint] = []
        for p in self._peers.values():
            if self._step - p["rx_step"] <= 12 and not p["carry"]:
                if p.get("goal") is not None:
                    pts.append(tuple(p["goal"]))
                elif p.get("state") == STATE_EXPLORE:
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
        d_me = math.hypot(bx - my_pos[0], by - my_pos[1])
        for pid, p in self._peers.items():
            if pid == my_id or (self._step - p["rx_step"] > 10) or p["carry"]:
                continue
            d_peer = math.hypot(bx - p["x"], by - p["y"])
            if p.get("claim") == k:
                if d_peer < d_me - 12.0 or (abs(d_peer - d_me) <= 12.0 and pid < my_id):
                    return True
            elif p["state"] in (STATE_TO_BOMB, STATE_GRASPING) and p.get("goal") is not None:
                gx, gy = p["goal"]
                if math.hypot(gx - bx, gy - by) < 30.0 and d_peer < d_me - 12.0:
                    return True
        return False

    def _pick_unclaimed_bomb(self, my_pos: WorldPoint) -> Optional[Tuple[int, int]]:
        if self._omap is None:
            return None
        my_id = int(self.identifier)
        active_peers = [
            (pid, p)
            for pid, p in self._peers.items()
            if (self._step - p["rx_step"] <= 10) and not p["carry"]
        ]

        candidates: List[Tuple[float, Tuple[int, int]]] = []
        for k, rec in self._known_bombs.items():
            if rec["status"] != "ACTIVE":
                continue
            bx, by = rec["pos"]
            directly_visible = self._had_gps_outage and rec.get("direct_seen") == self._step
            if self._omap.is_goal_unreachable((bx, by), self._step) and not directly_visible:
                continue
            d_me = math.hypot(bx - my_pos[0], by - my_pos[1])

            # Check if another unloaded peer cleanly owns or is much closer to this bomb
            contested = False
            for pid, p in active_peers:
                if pid == my_id:
                    continue
                d_peer = math.hypot(bx - p["x"], by - p["y"])
                if p.get("claim") == k:
                    if d_peer < d_me + 12.0 or (abs(d_peer - d_me) <= 12.0 and pid < my_id):
                        contested = True
                        break
                elif p["state"] in (STATE_TO_BOMB, STATE_GRASPING) and p.get("goal") is not None:
                    # Peer heading near same bomb
                    gx, gy = p["goal"]
                    if math.hypot(gx - bx, gy - by) < 30.0 and d_peer < d_me + 10.0:
                        contested = True
                        break
            if contested:
                continue

            candidates.append((d_me, k))

        if not candidates:
            return None
        candidates.sort(key=lambda item: item[0])

        # Refine only top 2 uncontested candidates with A* to keep per-step CPU minimal
        best_k: Optional[Tuple[int, int]] = None
        best_score = float("inf")
        for d_euc, k in candidates[:2]:
            rec = self._known_bombs[k]
            bx, by = rec["pos"]
            if d_euc < 500.0:
                _, path_d, reached = self._omap.plan_path(
                    my_pos,
                    (bx, by),
                    carrying=False,
                    allow_unknown=True,
                    max_expansions=4200,
                    allow_fallback=False,
                    check_goal_los=True,
                    allow_door=self.pose.gps_settled,
                )
                if not reached:
                    directly_visible = self._had_gps_outage and rec.get("direct_seen") == self._step
                    if directly_visible and d_euc < 120.0:
                        if d_euc < best_score:
                            best_score = d_euc
                            best_k = k
                        continue
                    self._omap.mark_goal_unreachable((bx, by), self._step + 90)
                    continue
                sc = path_d
            else:
                sc = d_euc * 1.15
            if sc < best_score:
                best_score = sc
                best_k = k
        if best_k is not None:
            return best_k
        return candidates[2][1] if len(candidates) > 2 else None

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
        avoid_headings: Tuple[float, ...] = (),
    ) -> WorldPoint:
        if lidar_vals is None or lidar_angles is None:
            return (my_pos[0] - 55.0 * math.cos(heading), my_pos[1] - 55.0 * math.sin(heading))
        n = len(lidar_vals)
        best_i = 0
        best_val = -1.0
        best_score = -1.0 if not avoid_headings else -math.inf
        for i in range(0, n, 4):
            # Minimum in small angular window ensures wide corridor opening
            lo = max(0, i - 3)
            hi = min(n, i + 4)
            w_min = float(np.nanmin(lidar_vals[lo:hi]))
            if not math.isfinite(w_min):
                continue
            direction = heading + float(lidar_angles[i])
            repeat_penalty = max(
                (max(0.0, math.cos(direction - angle)) ** 4 for angle in avoid_headings),
                default=0.0,
            )
            score = w_min - 115.0 * repeat_penalty
            if score > best_score:
                best_score = score
                best_val = w_min
                best_i = i
        wang = heading + float(lidar_angles[best_i])
        step_d = min(75.0, max(35.0, best_val - 18.0))
        return (my_pos[0] + step_d * math.cos(wang), my_pos[1] + step_d * math.sin(wang))

    def _recover_stalled_return(
        self,
        my_pos: WorldPoint,
        heading: float,
        lidar_vals: Optional[np.ndarray],
        lidar_angles: Optional[np.ndarray],
    ) -> None:
        """Move to a different local opening when a loaded route stops progressing."""
        anchor = self._return_checkpoint
        if anchor is None or math.hypot(my_pos[0] - anchor[0], my_pos[1] - anchor[1]) >= 76.0:
            self._return_checkpoint = my_pos
            self._return_checkpoint_step = self._step
            self._return_escape_headings.clear()
            return

        if (
            self._step - self._return_checkpoint_step < 170
            or self._step - self._last_saw_dc_step <= 12
            or lidar_vals is None or lidar_angles is None
        ):
            return

        last_recovery = self._last_return_recovery
        if (
            self.pose.in_no_gps and self._omap is not None and self._waypoints
            and last_recovery is not None and math.dist(my_pos, last_recovery) < 110.0
        ):
            waypoint = self._waypoints[0]
            distance = math.dist(my_pos, waypoint)
            if distance > 12.0:
                fraction = min(1.0, 36.0 / distance)
                point = (my_pos[0] + fraction * (waypoint[0] - my_pos[0]),
                         my_pos[1] + fraction * (waypoint[1] - my_pos[1]))
                self._omap.set_return_detour(point, self._step + 200)
        self._last_return_recovery = my_pos

        target = self._pick_open_lidar_dir(
            my_pos, heading, lidar_vals, lidar_angles, tuple(self._return_escape_headings)
        )
        self._return_escape_headings.append(math.atan2(target[1] - my_pos[1], target[0] - my_pos[0]))
        self._return_checkpoint_step = self._step
        self._unhook_target = target
        self._unhook_steps = 36
        self._unhook_cooldown_until = self._step + 48
        self._return_escape_until = self._step + self._unhook_steps
        self._waypoints = []

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
            or (self._step - self._plan_step) >= 34
            or (
                (self._step - self._plan_step) >= 8
                and self._omap.path_is_obstructed(my_pos, self._waypoints, carrying=carrying)
            )
        )
        if need_replan:
            self._omap.ensure_fresh(self._step, need_wall_edt=self.pose.in_no_gps)
            wps, _, rch = self._omap.plan_path(
                my_pos,
                goal,
                carrying=carrying,
                allow_unknown=allow_unknown,
                max_expansions=7500,
                allow_fallback=allow_fallback,
                check_goal_los=check_goal_los,
                allow_door=allow_door,
            )
            if not rch and not allow_fallback:
                self._omap.mark_goal_unreachable(goal, self._step + 90)
                if (
                    self._state == STATE_EXPLORE and self._cov is not None
                    and self.communicator_is_disabled() and not self._had_gps_outage
                    and self._step_limit is not None and self._step >= 0.55 * self._step_limit
                ):
                    # The fine pathfinder and coverage search discretize targets differently.
                    # Skip the coarse cell briefly so the next sweep can leave a blocked alcove.
                    self._cov.mark_unreachable(goal, self._step + 180)
            self._waypoints = wps
            self._plan_goal = goal
            self._plan_step = self._step

        while len(self._waypoints) > 1:
            if (
                math.hypot(self._waypoints[0][0] - my_pos[0], self._waypoints[0][1] - my_pos[1]) < 24.0
                or self._omap.has_los(my_pos, self._waypoints[1], clearance=(18.0 if carrying else 14.5), require_seen=True)
            ):
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

        pts = list(self._omap.dc_points.values())
        pts.sort(key=lambda item: math.hypot(item[2] - my_pos[0], item[3] - my_pos[1]))

        best_app: Optional[WorldPoint] = None
        best_hit: Optional[WorldPoint] = None
        best_cost = float("inf")

        for hx, hy, ax, ay in pts[:5]:
            _, pdist, reached = self._omap.plan_path(
                my_pos, (ax, ay), carrying=True, allow_unknown=True, max_expansions=3200
            )
            cost = pdist if reached else pdist + 500.0
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
        if self.pose.in_no_gps:
            self._had_gps_outage = True

        my_pos = (px, py)
        if self._spawn_pos is None:
            self._spawn_pos = my_pos

        # Maintain physically traversed breadcrumb corridor inside NO_GPS_ZONE
        if self.pose.gps_settled:
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
        self._handle_carry_transition(carrying, my_pos)
        visible_bombs, best_dc_ray = self._observe_semantic(my_pos, heading, semantic_vals, carrying)

        # Record pose history for stuck/snag detection
        self._pos_hist.append(my_pos)
        if carrying:
            owner = self._shared_load_owner(my_pos)
            if owner is not None:
                self._yielded_load_to = owner
                self._unhook_steps = 18
                self._unhook_cooldown_until = self._step + 44
                self._unhook_target = self._pick_open_lidar_dir(my_pos, heading, lidar_vals, lidar_angles)
                self._waypoints = []
                return empty_command(grasper=0)
        if carrying and self._state == STATE_CARRYING and self.communicator_is_disabled():
            close_peer = any(
                obj.entity_type == DroneSemanticSensor.TypeEntity.DRONE
                and 0.0 < obj.distance < 56.0
                for obj in (semantic_vals or ())
            )
            if close_peer:
                self._near_peer_carry_steps = min(180, self._near_peer_carry_steps + 1)
            else:
                self._near_peer_carry_steps = max(0, self._near_peer_carry_steps - 2)

            # Two grasps on the same object can oppose each other indefinitely.
            # Yield only with sustained proximity and little progress away from DC.
            if (
                close_peer
                and self._near_peer_carry_steps >= 75
                and self._step - self._pickup_step >= 170
                and self._step - self._last_saw_dc_step > 8
                and self._step % 17 == int(self.identifier) % 17
                and len(self._pos_hist) >= 44
                and math.hypot(px - self._pos_hist[0][0], py - self._pos_hist[0][1]) < 30.0
            ):
                self._unhook_steps = 18
                self._unhook_cooldown_until = self._step + 44
                self._unhook_target = self._pick_open_lidar_dir(my_pos, heading, lidar_vals, lidar_angles)
                self._waypoints = []
                return empty_command(grasper=0)
        else:
            self._near_peer_carry_steps = 0

        if (
            carrying
            and self._state == STATE_CARRYING
            and (self._had_gps_outage or self.communicator_is_disabled())
        ):
            self._recover_stalled_return(my_pos, heading, lidar_vals, lidar_angles)

        if (
            self._unhook_steps == 0
            and self._step >= self._unhook_cooldown_until
            and len(self._pos_hist) >= 44
            and self._state not in (STATE_GRASPING, STATE_DISPOSING)
        ):
            ox, oy = self._pos_hist[0]
            if math.hypot(px - ox, py - oy) < 8.0:
                self._unhook_steps = 12
                self._unhook_cooldown_until = self._step + 34
                self._unhook_target = self._pick_open_lidar_dir(my_pos, heading, lidar_vals, lidar_angles)
                self._waypoints = []

        if self._unhook_steps > 0:
            self._unhook_steps -= 1
            utgt = self._unhook_target if self._unhook_target is not None else my_pos
            rotating_escape = carrying and self._step >= self._return_escape_until
            rot_h = normalize_angle(heading + 0.75) if rotating_escape else None
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
            # Enter DISPOSING immediately when semantic sensor sees an unobstructed DC face <= 64 px
            if self._state == STATE_CARRYING:
                if best_dc_ray is not None and best_dc_ray[1] <= 64.0:
                    self._state = STATE_DISPOSING
                    self._dispose_steps = 0

            if self._state == STATE_DISPOSING:
                self._dispose_steps += 1
                # With little time left, change the docking angle after the first failed attempt.
                late_dock = self._step_limit is not None and self._step >= 0.82 * self._step_limit
                sweep = self._failed_docks >= (1 if late_dock else 2)
                if self._dispose_steps > (108 if sweep else 46):
                    # Wedged on wrong side / corner: unhook & try another DC entry
                    self._failed_docks += 1
                    self._state = STATE_CARRYING
                    self._dispose_steps = 0
                    self._unhook_steps = 10
                    self._unhook_cooldown_until = self._step + 24
                    self._unhook_target = self._pick_open_lidar_dir(my_pos, heading, lidar_vals, lidar_angles)
                    self._dc_target_pt = None
                    self._waypoints = []
                    return empty_command(grasper=1)

                if best_dc_ray is not None:
                    dc_ang, dc_dist = best_dc_ray
                    ux = math.cos(dc_ang)
                    uy = math.sin(dc_ang)
                    if late_dock and self._failed_docks == 1 and self._grasp_bearing is not None and dc_dist <= 34.0:
                        # Aim the last observed pickup side toward the face, with a small scan
                        # to accommodate a shift of the load after grasping.
                        error = dc_dist - 15.5
                        push_tgt = (px + error * ux, py + error * uy)
                        d_head = normalize_angle(dc_ang - self._grasp_bearing + 0.65 * math.sin(self._dispose_steps / 10.0))
                        p_spd = min(2.6, 0.6 * abs(error))
                    elif sweep and dc_dist <= 34.0:
                        # Keep a measured stand-off while the load sweeps the face.
                        # Reverse the sweep after each failed attempt.
                        clearance = 17.5 if self._failed_docks % 2 == 0 else 20.5
                        error = dc_dist - clearance
                        push_tgt = (px + error * ux, py + error * uy)
                        direction = 1.0 if self._failed_docks % 2 == 0 else -1.0
                        d_head = normalize_angle(heading + direction * 0.95)
                        p_spd = min(2.6, 0.6 * abs(error))
                    elif dc_dist > 19.5:
                        push_tgt = (px + (dc_dist + 10.0) * ux, py + (dc_dist + 10.0) * uy)
                        d_head = dc_ang
                        p_spd = 3.2
                    else:
                        # Hover ~18.5 px off DC wall while rotating bomb into contact (saves hull HP)
                        push_tgt = (px + (dc_dist - 18.5) * ux, py + (dc_dist - 18.5) * uy)
                        d_head = normalize_angle(heading + 0.95)
                        p_spd = 2.0
                    return drive_to_target(
                        pos=my_pos,
                        heading=heading,
                        vel=(self.pose.vx, self.pose.vy),
                        target=push_tgt,
                        desired_speed=p_spd,
                        max_speed=self._LOADED_MAX_SPEED,
                        carrying=True,
                        desired_heading=d_head,
                        lidar_vals=lidar_vals,
                        lidar_angles=lidar_angles,
                        semantic_vals=semantic_vals,
                        my_id=int(self.identifier),
                        my_priority_tier=2,
                        peer_states=peer_tuples,
                        disposing_mode=True,
                        grasper=1,
                    )

                fallback_hit = self._dc_hit_pt if self._dc_hit_pt is not None else (self._spawn_pos or my_pos)
                return drive_to_target(
                    pos=my_pos,
                    heading=heading,
                    vel=(self.pose.vx, self.pose.vy),
                    target=fallback_hit,
                    desired_speed=2.6,
                    max_speed=self._LOADED_MAX_SPEED,
                    carrying=True,
                    desired_heading=normalize_angle(heading + 0.95),
                    lidar_vals=lidar_vals,
                    lidar_angles=lidar_angles,
                    semantic_vals=semantic_vals,
                    my_id=int(self.identifier),
                    my_priority_tier=2,
                    peer_states=peer_tuples,
                    disposing_mode=True,
                    grasper=1,
                )

            # STATE_CARRYING: follow Reverse Dijkstra DC potential field (escapes all concave mazes)
            need_dc_replan = (
                not self._waypoints
                or (self._step - self._dc_plan_step) >= 22
                or (
                    (self._step - self._dc_plan_step) >= 10
                    and self._omap.path_is_obstructed(my_pos, self._waypoints, carrying=True)
                )
            )
            if need_dc_replan:
                self._omap.ensure_fresh(self._step, need_wall_edt=self.pose.in_no_gps)
                fb_spawn = None
                if not self._omap.dc_points and self._spawn_pos is not None:
                    if math.hypot(px - self._spawn_pos[0], py - self._spawn_pos[1]) > 75.0:
                        fb_spawn = self._spawn_pos
                wps_dc, app_pt, hit_pt, ok_dc = self._omap.get_dc_path(
                    my_pos, self._step, fallback_target=fb_spawn,
                    time_weighted=self._time_weighted_return,
                )
                if ok_dc and wps_dc and app_pt is not None:
                    if (
                        not self._time_weighted_return and self.pose.gps_settled
                        and not self._had_gps_outage and not self._peer_gps_outage
                        and self._step - self._last_return_comparison >= 110
                    ):
                        path_length = sum(math.dist(a, b) for a, b in zip([my_pos, *wps_dc], wps_dc))
                        direct_length = math.dist(my_pos, app_pt)
                        if path_length >= 850.0 and path_length >= 1.45 * direct_length:
                            self._last_return_comparison = self._step
                            faster = self._omap.get_dc_path(
                                my_pos, self._step, fallback_target=fb_spawn, time_weighted=True,
                            )
                            alternate, alt_app, alt_hit, reachable = faster
                            if reachable and alternate and alt_app is not None:
                                alt_length = sum(math.dist(a, b) for a, b in zip([my_pos, *alternate], alternate))
                                if alt_length <= 0.85 * path_length:
                                    wps_dc, app_pt, hit_pt = alternate, alt_app, alt_hit
                                    self._time_weighted_return = True
                    self._waypoints = wps_dc
                    self._dc_target_pt = app_pt
                    self._dc_hit_pt = hit_pt
                    self._plan_goal = app_pt
                    self._plan_step = self._step
                    self._dc_plan_step = self._step
                else:
                    self._dc_target_pt, self._dc_hit_pt = self._select_best_dc_target(my_pos)
                    self._dc_plan_step = self._step
                    if self.pose.in_no_gps and len(self._no_gps_breadcrumbs) >= 2:
                        # Follow exact physically-traversed breadcrumb trail back toward GPS zone
                        rev_bc = list(reversed(self._no_gps_breadcrumbs))
                        self._waypoints = rev_bc + (
                            [self._dc_target_pt] if self._dc_target_pt is not None else []
                        )
                        self._plan_goal = self._waypoints[-1]
                    else:
                        tgt_fb = self._dc_target_pt if self._dc_target_pt is not None else (self._spawn_pos or my_pos)
                        wps_fb, _, _ = self._omap.plan_path(
                            my_pos, tgt_fb, carrying=True, allow_unknown=True, max_expansions=12000
                        )
                        self._waypoints = wps_fb
                        self._plan_goal = tgt_fb
                    self._plan_step = self._step

            if self._waypoints:
                while len(self._waypoints) > 1 and (
                    math.hypot(self._waypoints[0][0] - px, self._waypoints[0][1] - py) < 28.0
                    or self._omap.has_los(my_pos, self._waypoints[1], clearance=18.0, require_seen=True)
                ):
                    self._waypoints.pop(0)
                wp = self._waypoints[0]
            else:
                tgt = self._dc_target_pt if self._dc_target_pt is not None else (self._spawn_pos or my_pos)
                wp = self._follow_planned_path(my_pos, tgt, carrying=True, allow_unknown=True)

            # A direct semantic return uses the DC bearing relative to the drone.
            # It remains usable after GPS drift if LiDAR confirms the approach.
            if self._had_gps_outage and best_dc_ray is not None and lidar_vals is not None and lidar_angles is not None:
                dc_angle, dc_distance = best_dc_ray
                approach_distance = dc_distance - 38.0
                if approach_distance > 0.0:
                    relative_angles = lidar_angles - (dc_angle - heading)
                    finite = np.isfinite(lidar_vals) & (lidar_vals > 1.0)
                    angles = relative_angles[finite]
                    ranges = lidar_vals[finite]
                    forward = ranges * np.cos(angles)
                    lateral = ranges * np.sin(angles)
                    obstructed = (forward > 0.0) & (forward < approach_distance + 18.0) & (np.abs(lateral) < 20.0)
                    if ranges.size and not np.any(obstructed):
                        wp = (px + approach_distance * math.cos(dc_angle), py + approach_distance * math.sin(dc_angle))

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
            if self._step >= 8:
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
            # Briefly hold gentle speed on steps 1-3 if DC hasn't been received yet via radio relay
            fan_spd = 3.5 if (self._step <= 3 and not self._omap.dc_points and not self.communicator_is_disabled()) else self._UNLOADED_MAX_SPEED
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

        # Re-check bomb assignment whenever exploring or periodically while en route
        if self._state == STATE_EXPLORE:
            b_key = self._pick_unclaimed_bomb(my_pos)
            if b_key is not None:
                self._target_bomb_key = b_key
                self._known_bombs[b_key]["claimer"] = int(self.identifier)
                self._known_bombs[b_key]["claim_step"] = self._step
                self._state = STATE_TO_BOMB
                self._waypoints = []
        elif self._state == STATE_TO_BOMB:
            # Drop stale/completed/unreachable target or switch if a closer unclaimed bomb appears
            cur = self._known_bombs.get(self._target_bomb_key) if self._target_bomb_key is not None else None
            if (
                cur is None
                or cur["status"] != "ACTIVE"
                or self._omap.is_goal_unreachable(cur["pos"], self._step)
            ):
                self._target_bomb_key = None
                self._state = STATE_EXPLORE
                self._waypoints = []
            elif self._step % 8 == 0:
                if self._target_bomb_key is not None and self._is_bomb_contested(self._target_bomb_key, my_pos):
                    better_k = self._pick_unclaimed_bomb(my_pos)
                    self._target_bomb_key = better_k
                    self._waypoints = []
                    if better_k is None:
                        self._state = STATE_EXPLORE
                else:
                    better_k = self._pick_unclaimed_bomb(my_pos)
                    if better_k is not None and better_k != self._target_bomb_key:
                        cur_d = math.hypot(cur["pos"][0] - my_pos[0], cur["pos"][1] - my_pos[1])
                        new_p = self._known_bombs[better_k]["pos"]
                        new_d = math.hypot(new_p[0] - my_pos[0], new_p[1] - my_pos[1])
                        if new_d + 45.0 < cur_d:
                            self._target_bomb_key = better_k
                            self._waypoints = []
            if self._target_bomb_key in self._known_bombs:
                self._known_bombs[self._target_bomb_key]["claimer"] = int(self.identifier)
                self._known_bombs[self._target_bomb_key]["claim_step"] = self._step

        # Precision GRASPING
        if self._state in (STATE_TO_BOMB, STATE_GRASPING) and self._target_bomb_key in self._known_bombs:
            rec = self._known_bombs[self._target_bomb_key]
            bpos = rec["pos"]
            # Snap to live visible semantic bomb within 55 px if available
            live_pt = bpos
            best_vd = 55.0
            has_live = False
            for vp in visible_bombs:
                vd = math.hypot(vp[0] - bpos[0], vp[1] - bpos[1])
                if vd < best_vd:
                    best_vd = vd
                    live_pt = vp
                    has_live = True

            dist_b = math.hypot(live_pt[0] - px, live_pt[1] - py)
            if self._state == STATE_TO_BOMB and dist_b < (52.0 if has_live else 38.0):
                self._state = STATE_GRASPING
                self._grasp_steps = 0

            if self._state == STATE_GRASPING:
                self._grasp_steps += 1
                if self._grasp_steps > 46 or rec["status"] != "ACTIVE":
                    # Failed grasp: back off briefly and re-evaluate
                    self._state = STATE_EXPLORE
                    self._target_bomb_key = None
                    self._grasp_steps = 0
                    self._unhook_steps = 8
                    self._unhook_cooldown_until = self._step + 20
                    self._unhook_target = self._pick_open_lidar_dir(my_pos, heading, lidar_vals, lidar_angles)
                    return empty_command(grasper=0)

                grasp_on = 1 if dist_b < 30.0 else 0
                spd = 9.0 if dist_b > 42.0 else (5.5 if dist_b > 22.0 else 3.2)
                ang_to_b = math.atan2(live_pt[1] - py, live_pt[0] - px)
                if grasp_on and has_live:
                    self._grasp_bearing = normalize_angle(ang_to_b - heading)
                    self._grasp_bearing_step = self._step
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
                allow_fallback=(has_live or self.pose.in_no_gps),
                check_goal_los=(not has_live),
                allow_door=self.pose.gps_settled,
            )
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
                grasper=1 if dist_b < 24.0 else 0,
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
                (self._step - self._explore_set_step) >= 12
                or math.hypot(self._explore_target[0] - px, self._explore_target[1] - py) < 150.0
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
            tgt, wps = self._cov.pick_exploration_with_path(
                my_id=int(self.identifier),
                num_drones=10,
                my_pos=my_pos,
                peer_targets=self._peer_explore_targets(my_pos, semantic_vals),
                current_step=self._step,
                comm_lost=comm_lost,
                vel_dir=vdir,
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
            allow_fallback=False,
            allow_door=(self.pose.gps_settled and self._step >= 1000),
        )
        return drive_to_target(
            pos=my_pos,
            heading=heading,
            vel=(self.pose.vx, self.pose.vy),
            target=wp,
            desired_speed=self._UNLOADED_MAX_SPEED,
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
