"""
Online per-drone 2D occupancy grid, Euclidean distance field, and fast
8-px hierarchical clearance-weighted A* path planner built strictly from
runtime LiDAR and semantic observations (zero preloaded map geometry or
global shared state).
"""

from __future__ import annotations

import heapq
import math
from typing import Dict, Iterable, List, Optional, Tuple

import cv2
import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

WorldPoint = Tuple[float, float]


class OnlineMap:
    """
    Per-drone online occupancy and navigation map.

    Occupancy is tracked at 4 px resolution for sharp wall/doorway fidelity
    and sub-cell scan matching; A* planning operates on a 2x2 min-pooled 8 px
    planning grid so doorways are never skipped while achieving <1.5 ms solves.
    """

    CELL_SIZE = 4
    PLAN_CELL = 8  # 2x2 min-pooled over CELL_SIZE=4
    HARD_CLEARANCE_UNLOADED = 12.8
    HARD_CLEARANCE_CARRYING = 14.5
    SOFT_CLEARANCE_PX = 25.0
    SMOOTH_CLEARANCE_UNLOADED = 14.5
    SMOOTH_CLEARANCE_CARRYING = 18.5

    _RAY_STEPS = np.arange(6.0, 225.0, 5.0, dtype=np.float32)[None, :]

    def __init__(self, size_area: Optional[Tuple[int, int]] = None) -> None:
        if size_area is not None and len(size_area) >= 2:
            self.width = max(100, int(size_area[0]))
            self.height = max(100, int(size_area[1]))
        else:
            self.width = 1800
            self.height = 1200

        cs = self.CELL_SIZE
        self.gw = (self.width + cs - 1) // cs
        self.gh = (self.height + cs - 1) // cs

        # Even dimensions for clean 2x2 pooling to 8-px planning grid
        self.pw = max(2, self.gw // 2)
        self.ph = max(2, self.gh // 2)
        self.p_size = self.pw * self.ph

        # Wall confidence (0..24): >=3 is WALL.
        self.wall_score = np.zeros((self.gh, self.gw), dtype=np.uint8)
        self.wall_score[0, :] = 24
        self.wall_score[-1, :] = 24
        self.wall_score[:, 0] = 24
        self.wall_score[:, -1] = 24

        self.free_seen = np.zeros((self.gh, self.gw), dtype=bool)
        self.dc_mask = np.zeros((self.gh, self.gw), dtype=bool)
        # Only actual GPS+compass-verified walls anchor NO_GPS scan-matching
        # (never pre-seed border lines to avoid false border snaps deep in NO_GPS_ZONE)
        self.gps_wall_mask = np.zeros((self.gh, self.gw), dtype=bool)
        self.gps_wall_count = 0

        self._own_wall_set: set[int] = set()
        self._own_wall_list: List[int] = []
        self._recent_own_walls: List[int] = []
        self._sync_cursor = 0

        # Observed DISPOSAL_CENTER boundary points & open-side approach waypoints
        self.dc_points: Dict[Tuple[int, int], Tuple[float, float, float, float]] = {}

        self._dirty = True
        self._wall_edt_dirty = True
        self._last_rebuild_step = -100
        self.wall_mask = self.wall_score >= 3
        self.obs_mask = self.wall_mask | self.dc_mask
        self.edt_px = np.full((self.gh, self.gw), 100.0, dtype=np.float32)
        self.wall_edt_px = np.full((self.gh, self.gw), 100.0, dtype=np.float32)
        self.total_wall_count = 0

        # 4-px fine masks (for coverage & local clearance checks)
        self.free_unloaded = np.ones((self.gh, self.gw), dtype=bool)
        self.free_carrying = np.ones((self.gh, self.gw), dtype=bool)

        # 8-px planning buffers
        self.plan_free_unloaded = np.ones((self.ph, self.pw), dtype=bool)
        self.plan_free_carrying = np.ones((self.ph, self.pw), dtype=bool)
        self.plan_free_tight = np.ones((self.ph, self.pw), dtype=bool)
        self.plan_free_door = np.ones((self.ph, self.pw), dtype=bool)
        self.plan_no_solid = np.ones((self.ph, self.pw), dtype=bool)
        self._p_seen_arr = np.zeros((self.ph, self.pw), dtype=bool)
        self._p_edt_arr = np.full((self.ph, self.pw), 100.0, dtype=np.float32)
        self._p_cost_arr = np.ones((self.ph, self.pw), dtype=np.float32)
        self._p_free_u_b = bytearray(self.p_size)
        self._p_free_c_b = bytearray(self.p_size)
        self._p_free_tight_b = bytearray(self.p_size)
        self._p_free_door_b = bytearray(self.p_size)
        self._p_no_solid_b = bytearray(self.p_size)
        self._p_seen_b = bytearray(self.p_size)
        self._p_highway_b = bytearray(self.p_size)
        self._highway_grid = np.zeros((self.ph, self.pw), dtype=bool)
        self._peer_highway_cells = np.zeros(((self.ph + 2) // 3, (self.pw + 2) // 3), dtype=bool)
        self._p_cost_list: List[float] = [1.0] * self.p_size
        self._g_scratch: List[float] = [1e18] * self.p_size
        self._closed_scratch = bytearray(self.p_size)

        self.unreachable_until: Dict[Tuple[int, int], int] = {}
        self._dc_field_dirty = True
        self._dc_field_step = -100
        self._return_detour: Optional[Tuple[WorldPoint, int]] = None
        self._dc_time_weighted = False
        self._dc_preferred_face: Optional[WorldPoint] = None
        self._dc_used_fallback = False
        self._dc_parent: List[int] = [-1] * self.p_size
        self._dc_goal_map: Dict[int, WorldPoint] = {}
        self._dc_hit_map: Dict[int, WorldPoint] = {}
        self._csr_cache: Dict[str, csr_matrix] = {}
        self._rebuild_fields(step=0, need_wall_edt=True)

    def world_to_cell(self, wx: float, wy: float) -> Tuple[int, int]:
        cs = self.CELL_SIZE
        c = int((wx + self.width * 0.5) // cs)
        r = int((self.height * 0.5 - wy) // cs)
        if c < 0:
            c = 0
        elif c >= self.gw:
            c = self.gw - 1
        if r < 0:
            r = 0
        elif r >= self.gh:
            r = self.gh - 1
        return (r, c)

    def world_to_plan(self, wx: float, wy: float) -> Tuple[int, int]:
        ps = self.PLAN_CELL
        c = int((wx + self.width * 0.5) // ps)
        r = int((self.height * 0.5 - wy) // ps)
        if c < 0:
            c = 0
        elif c >= self.pw:
            c = self.pw - 1
        if r < 0:
            r = 0
        elif r >= self.ph:
            r = self.ph - 1
        return (r, c)

    def plan_to_world(self, r: int, c: int) -> WorldPoint:
        ps = self.PLAN_CELL
        return (
            (c + 0.5) * ps - self.width * 0.5,
            self.height * 0.5 - (r + 0.5) * ps,
        )

    def _add_own_wall_cell(self, r: int, c: int) -> None:
        if r <= 0 or r >= self.gh - 1 or c <= 0 or c >= self.gw - 1:
            return
        idx = r * self.gw + c
        if idx not in self._own_wall_set:
            self._own_wall_set.add(idx)
            self._own_wall_list.append(idx)
            self._recent_own_walls.append(idx)
            if len(self._recent_own_walls) > 80:
                self._recent_own_walls = self._recent_own_walls[-60:]

    def update_from_scan(
        self,
        pos: WorldPoint,
        heading: float,
        lidar_vals: Optional[np.ndarray],
        lidar_angles: Optional[np.ndarray],
        semantic_vals: Optional[list],
        gps_trusted: bool,
        step: int,
    ) -> None:
        px, py = float(pos[0]), float(pos[1])
        half_w = self.width * 0.5
        half_h = self.height * 0.5
        cs = self.CELL_SIZE
        gh, gw = self.gh, self.gw

        r0, c0 = self.world_to_cell(px, py)
        for rr in range(max(1, r0 - 1), min(gh - 1, r0 + 2)):
            for cc in range(max(1, c0 - 1), min(gw - 1, c0 + 2)):
                self.free_seen[rr, cc] = True
                ws = int(self.wall_score[rr, cc])
                if ws > 0:
                    self.wall_score[rr, cc] = ws - 2 if ws >= 2 else 0

        dc_dirs: List[Tuple[float, float]] = []
        dyn_dirs: List[Tuple[float, float]] = []
        if semantic_vals:
            for obj in semantic_vals:
                ang = float(obj.angle)
                dist = float(obj.distance)
                if not (math.isfinite(ang) and math.isfinite(dist)):
                    continue
                ename = getattr(obj.entity_type, "name", "")
                if ename in ("DRONE", "BOMB"):
                    dyn_dirs.append((ang, dist))
                elif ename == "DISPOSAL_CENTER":
                    dc_dirs.append((ang, dist))
                    if dist > 4.0:
                        wang = heading + ang
                        ux = math.cos(wang)
                        uy = math.sin(wang)
                        hx = px + dist * ux
                        hy = py + dist * uy
                        # Approach point 26 px outside DC face (clean clearance from dc_mask)
                        ax = hx - 26.0 * ux
                        ay = hy - 26.0 * uy
                        self.add_dc_point(hx, hy, ax, ay)
                        for shift in (6.0, 10.0):
                            dr_c, dc_c = self.world_to_cell(hx + shift * ux, hy + shift * uy)
                            if not self.dc_mask[dr_c, dc_c]:
                                self.dc_mask[dr_c, dc_c] = True
                                self._dirty = True

        if lidar_vals is not None and lidar_angles is not None:
            dists_raw = np.asarray(lidar_vals[::2], dtype=np.float32)
            angs_raw = np.asarray(lidar_angles[::2], dtype=np.float32)
            valid = np.isfinite(dists_raw) & (dists_raw > 1.0)
            dists_raw = dists_raw[valid]
            angs_raw = angs_raw[valid]

            if dists_raw.size > 0:
                world_angs = heading + angs_raw
                cos_a = np.cos(world_angs)
                sin_a = np.sin(world_angs)

                free_len = np.minimum(dists_raw - 5.0, 224.0)
                mask_f = self._RAY_STEPS <= free_len[:, None]
                if np.any(mask_f):
                    fx_v = (px + cos_a[:, None] * self._RAY_STEPS)[mask_f]
                    fy_v = (py + sin_a[:, None] * self._RAY_STEPS)[mask_f]
                    cols_f = np.clip(((fx_v + half_w) // cs).astype(np.int32), 1, gw - 2)
                    rows_f = np.clip(((half_h - fy_v) // cs).astype(np.int32), 1, gh - 2)
                    self.free_seen[rows_f, cols_f] = True

                    cur_ws = self.wall_score[rows_f, cols_f]
                    nz = cur_ws > 0
                    if np.any(nz):
                        rz = rows_f[nz]
                        cz = cols_f[nz]
                        sub = cur_ws[nz]
                        was_w = sub >= 3
                        # Protect double-confirmed GPS anchors; carve transient/no-gps hits cleanly
                        step_dec = np.where(self.gps_wall_mask[rz, cz], 1, 3).astype(np.uint8)
                        new_ws = np.where(sub > step_dec, sub - step_dec, 0).astype(np.uint8)
                        self.wall_score[rz, cz] = new_ws
                        if np.any(was_w & (new_ws < 3)):
                            self._dirty = True

                max_wall_d = 180.0 if gps_trusted else 125.0
                hit_mask = (dists_raw > 12.0) & (dists_raw < max_wall_d)
                if np.any(hit_mask):
                    h_d = dists_raw[hit_mask]
                    h_a = angs_raw[hit_mask]
                    h_cos = cos_a[hit_mask]
                    h_sin = sin_a[hit_mask]

                    keep = np.ones(h_d.size, dtype=bool)
                    for o_ang, o_dist in dyn_dirs:
                        d_ang = np.abs(np.arctan2(np.sin(h_a - o_ang), np.cos(h_a - o_ang)))
                        keep &= ~((d_ang < 0.22) & (np.abs(h_d - o_dist) < 28.0))
                    for d_ang_ref, d_dist_ref in dc_dirs:
                        d_ang = np.abs(np.arctan2(np.sin(h_a - d_ang_ref), np.cos(h_a - d_ang_ref)))
                        keep &= ~((d_ang < 0.14) & (np.abs(h_d - d_dist_ref) < 22.0))

                    if np.any(keep):
                        kd = h_d[keep]
                        hx = px + kd * h_cos[keep]
                        hy = py + kd * h_sin[keep]
                        cols_h = np.clip(((hx + half_w) // cs).astype(np.int32), 0, gw - 1)
                        rows_h = np.clip(((half_h - hy) // cs).astype(np.int32), 0, gh - 1)
                        cap = 10 if gps_trusted else 6
                        for rr, cc, rd in zip(rows_h.tolist(), cols_h.tolist(), kd.tolist()):
                            # Require 2 consistent hits >125 px so 4-deg AR compass smear doesn't seal narrow corridors
                            inc = 4 if (gps_trusted and rd <= 125.0) else (2 if gps_trusted else 3)
                            prev = int(self.wall_score[rr, cc])
                            nxt = min(cap, prev + inc)
                            if nxt > prev:
                                self.wall_score[rr, cc] = nxt
                            if prev < 3 <= nxt:
                                self._dirty = True
                            if gps_trusted and rd <= 130.0 and nxt >= 6 and not self.gps_wall_mask[rr, cc]:
                                self.gps_wall_mask[rr, cc] = True
                                self._wall_edt_dirty = True
                            if gps_trusted and rd <= 130.0 and nxt >= 4:
                                self._add_own_wall_cell(rr, cc)

        if self._dirty and (step - self._last_rebuild_step >= 12 or self._last_rebuild_step < 0):
            self._rebuild_fields(step, need_wall_edt=not gps_trusted)
        elif (not gps_trusted) and self._wall_edt_dirty and (step - self._last_rebuild_step >= 8):
            self._rebuild_wall_edt_only()

    def ensure_fresh(self, step: int, need_wall_edt: bool = False) -> None:
        if self._dirty and (step - self._last_rebuild_step >= 4):
            self._rebuild_fields(step, need_wall_edt=need_wall_edt)
        elif need_wall_edt and self._wall_edt_dirty:
            self._rebuild_wall_edt_only()

    def add_dc_point(self, hx: float, hy: float, ax: float, ay: float) -> None:
        key = (int(round(hx / 16.0)), int(round(hy / 16.0)))
        if key not in self.dc_points and len(self.dc_points) < 20:
            self.dc_points[key] = (float(hx), float(hy), float(ax), float(ay))
            self._dc_field_dirty = True
        elif key in self.dc_points:
            ox, oy, oax, oay = self.dc_points[key]
            self.dc_points[key] = (
                0.75 * ox + 0.25 * hx,
                0.75 * oy + 0.25 * hy,
                0.75 * oax + 0.25 * ax,
                0.75 * oay + 0.25 * ay,
            )

    def get_sync_wall_chunk(self, max_total: int = 80) -> List[int]:
        out: List[int] = []
        seen: set[int] = set()
        for idx in reversed(self._recent_own_walls[-24:]):
            r = idx // self.gw
            c = idx % self.gw
            packed = (r << 12) | c
            if packed not in seen:
                seen.add(packed)
                out.append(packed)
        n = len(self._own_wall_list)
        if n > 0:
            rem = max_total - len(out)
            take = min(rem, n)
            for _ in range(take):
                idx = self._own_wall_list[self._sync_cursor % n]
                self._sync_cursor = (self._sync_cursor + 1) % n
                r = idx // self.gw
                c = idx % self.gw
                packed = (r << 12) | c
                if packed not in seen:
                    seen.add(packed)
                    out.append(packed)
        return out

    def merge_peer_walls(self, packed_list: Iterable[int], peer_gps_trusted: bool = False) -> None:
        # Ignore unanchored wall broadcasts from peers inside NO_GPS_ZONE so pose drift
        # cannot paint ghost walls across teammates' maps.
        if not peer_gps_trusted:
            return
        packed = np.fromiter(packed_list, dtype=np.int32)
        rows = (packed >> 12) & 0xFFF
        cols = packed & 0xFFF
        inside = (rows > 0) & (rows < self.gh - 1) & (cols > 0) & (cols < self.gw - 1)
        cells = (rows[inside], cols[inside])
        scores = self.wall_score[cells]
        self._dirty = self._dirty or bool(np.any(scores < 3))
        self._wall_edt_dirty = self._wall_edt_dirty or not bool(np.all(self.gps_wall_mask[cells]))
        self.wall_score[cells] = np.maximum(scores, 6)
        self.gps_wall_mask[cells] = True

    def merge_peer_dc_points(self, dc_list: Iterable[Tuple[float, float, float, float]]) -> None:
        for hx, hy, ax, ay in dc_list:
            self.add_dc_point(hx, hy, ax, ay)
            ux = hx - ax
            uy = hy - ay
            un = math.hypot(ux, uy)
            if un > 1e-3:
                dr_c, dc_c = self.world_to_cell(hx + 6.0 * ux / un, hy + 6.0 * uy / un)
                if not self.dc_mask[dr_c, dc_c]:
                    self.dc_mask[dr_c, dc_c] = True
                    self._dirty = True

    def merge_peer_highway(self, cov_mask_24px: np.ndarray) -> None:
        """Mark 24-px corridors cleared by GPS-trusted peers as preferred open highways."""
        ch = min(self._peer_highway_cells.shape[0], cov_mask_24px.shape[0])
        cw = min(self._peer_highway_cells.shape[1], cov_mask_24px.shape[1])
        known = self._peer_highway_cells[:ch, :cw]
        incoming = cov_mask_24px[:ch, :cw]
        if not np.any(incoming & ~known):
            return
        known |= incoming
        up_p = np.repeat(np.repeat(cov_mask_24px, 3, axis=0), 3, axis=1)
        h_use = min(self.ph, up_p.shape[0])
        w_use = min(self.pw, up_p.shape[1])
        self._highway_grid[:h_use, :w_use] |= up_p[:h_use, :w_use]
        self._p_highway_b = self._highway_grid.ravel().tobytes()

    def _rebuild_wall_edt_only(self) -> None:
        cs = float(self.CELL_SIZE)
        self.gps_wall_count = int(np.count_nonzero(self.gps_wall_mask))
        if self.gps_wall_count >= 25:
            fg_walls = (~self.gps_wall_mask).astype(np.uint8)
            self.wall_edt_px = cv2.distanceTransform(fg_walls, cv2.DIST_L2, cv2.DIST_MASK_PRECISE) * cs
        self._wall_edt_dirty = False

    def _rebuild_fields(self, step: int, need_wall_edt: bool = False) -> None:
        cs = float(self.CELL_SIZE)
        self.wall_mask = self.wall_score >= 3
        obstacles = self.wall_mask | self.dc_mask
        obstacles_changed = self._last_rebuild_step < 0 or not np.array_equal(obstacles, self.obs_mask)
        self.obs_mask = obstacles
        self.total_wall_count = int(np.count_nonzero(self.wall_mask))
        self.gps_wall_count = int(np.count_nonzero(self.gps_wall_mask))

        if obstacles_changed:
            fg_obs = (~self.obs_mask).astype(np.uint8)
            self.edt_px = cv2.distanceTransform(fg_obs, cv2.DIST_L2, cv2.DIST_MASK_PRECISE) * cs

        if need_wall_edt and self._wall_edt_dirty and self.gps_wall_count >= 25:
            fg_walls = (~self.gps_wall_mask).astype(np.uint8)
            self.wall_edt_px = cv2.distanceTransform(fg_walls, cv2.DIST_L2, cv2.DIST_MASK_PRECISE) * cs
            self._wall_edt_dirty = False

        self.free_unloaded = self.edt_px >= self.HARD_CLEARANCE_UNLOADED
        self.free_carrying = self.edt_px >= self.HARD_CLEARANCE_CARRYING

        # 2x2 pool onto 8-px planning grid: use MAX clearance inside each 8x8 block
        # so narrow 36px doorways never close at 8px edges while blocked walls stay 0.
        ph, pw = self.ph, self.pw
        even_rows = slice(0, 2 * ph, 2)
        odd_rows = slice(1, 2 * ph, 2)
        even_cols = slice(0, 2 * pw, 2)
        odd_cols = slice(1, 2 * pw, 2)
        edt = self.edt_px
        plan_edt = np.maximum(edt[even_rows, even_cols], edt[even_rows, odd_cols])
        np.maximum(plan_edt, edt[odd_rows, even_cols], out=plan_edt)
        np.maximum(plan_edt, edt[odd_rows, odd_cols], out=plan_edt)
        seen = self.free_seen
        plan_seen = seen[even_rows, even_cols] | seen[even_rows, odd_cols]
        plan_seen |= seen[odd_rows, even_cols]
        plan_seen |= seen[odd_rows, odd_cols]

        self.plan_free_unloaded = plan_edt >= self.HARD_CLEARANCE_UNLOADED
        self.plan_free_carrying = plan_edt >= self.HARD_CLEARANCE_CARRYING
        self.plan_free_tight = plan_edt >= 9.0
        self.plan_free_door = plan_edt >= 4.5
        solid_4 = self.gps_wall_mask[: ph * 2, : pw * 2] | self.dc_mask[: ph * 2, : pw * 2]
        plan_no_solid = ~solid_4.reshape(ph, 2, pw, 2).all(axis=(1, 3))
        plan_no_solid[0, :] = False
        plan_no_solid[-1, :] = False
        plan_no_solid[:, 0] = False
        plan_no_solid[:, -1] = False
        self.plan_no_solid = plan_no_solid
        self._p_seen_arr = plan_seen
        self._p_edt_arr = plan_edt.astype(np.float32)
        deficit = np.maximum(0.0, self.SOFT_CLEARANCE_PX - plan_edt)
        plan_cost = (1.0 + 0.16 * deficit).astype(np.float32)
        self._p_cost_arr = plan_cost

        self._p_free_u_b = self.plan_free_unloaded.ravel().tobytes()
        self._p_free_c_b = self.plan_free_carrying.ravel().tobytes()
        self._p_free_tight_b = self.plan_free_tight.ravel().tobytes()
        self._p_free_door_b = self.plan_free_door.ravel().tobytes()
        self._p_no_solid_b = plan_no_solid.ravel().tobytes()
        self._p_seen_b = plan_seen.ravel().tobytes()
        self._p_cost_list = plan_cost.ravel().tolist()

        self._dirty = False
        self._last_rebuild_step = step
        self._dc_field_dirty = True
        if obstacles_changed:
            self._csr_cache.clear()

    def get_walk_csr(self, mode: str = "u") -> csr_matrix:
        """Build/cache directed 4-neighbor unweighted CSR graph on 8-px planning grid."""
        cached = self._csr_cache.get(mode)
        if cached is not None:
            return cached
        if mode == "d":
            m = self.plan_free_door
        elif mode == "t":
            m = self.plan_free_tight
        else:
            m = self.plan_free_unloaded
        pw = self.pw
        p_size = self.p_size
        r, c = np.nonzero(m[:-1, :] & m[1:, :])
        i0 = r * pw + c
        i1 = i0 + pw
        rr, cc = np.nonzero(m[:, :-1] & m[:, 1:])
        j0 = rr * pw + cc
        j1 = j0 + 1
        rows = np.concatenate((i0, i1, j0, j1))
        cols = np.concatenate((i1, i0, j1, j0))
        data = np.ones(rows.size, dtype=np.float32)
        csr = csr_matrix((data, (rows, cols)), shape=(p_size, p_size))
        self._csr_cache[mode] = csr
        return csr

    def _rebuild_dc_field(
        self,
        step: int,
        walk_b: Optional[bytes] = None,
        pref_b: Optional[bytes] = None,
        extra_pen: float = 0.0,
        fallback_target: Optional[WorldPoint] = None,
    ) -> None:
        """Multi-source weighted Reverse Dijkstra (Cython csgraph) from DC approach waypoints."""
        self._dc_field_dirty = False
        self._dc_field_step = step
        self._dc_used_fallback = not self.dc_points
        p_size = self.p_size
        ph, pw = self.ph, self.pw
        parent = [-1] * p_size
        goal_map: Dict[int, WorldPoint] = {}
        hit_map: Dict[int, WorldPoint] = {}

        if not self.dc_points and fallback_target is None:
            self._dc_parent = parent
            self._dc_goal_map = goal_map
            self._dc_hit_map = hit_map
            return

        if walk_b is None:
            free_c = self.plan_free_carrying
            walk_arr = self.plan_free_carrying
        elif walk_b is self._p_no_solid_b:
            free_c = self.plan_free_tight
            walk_arr = self.plan_no_solid
        else:
            free_c = self.plan_free_tight
            walk_arr = self.plan_free_tight

        if pref_b is self._p_free_c_b:
            pref_arr: Optional[np.ndarray] = self.plan_free_carrying
        elif pref_b is self._p_free_tight_b:
            pref_arr = self.plan_free_tight
        else:
            pref_arr = None

        seeds = (
            list(self.dc_points.values())
            if self.dc_points
            else [(fallback_target[0], fallback_target[1], fallback_target[0], fallback_target[1])]
        )
        if self._dc_preferred_face is not None and self.dc_points:
            nearby_seeds = [p for p in seeds if math.dist((p[2], p[3]), self._dc_preferred_face) <= 62.0]
            if nearby_seeds:
                seeds = nearby_seeds
        seed_indices: List[int] = []
        seen_seeds: set[int] = set()
        for hx, hy, ax, ay in seeds:
            gr, gc = self.world_to_plan(ax, ay)
            cell = self._nearest_valid_plan(gr, gc, free_c, max_r=4)
            if cell is None:
                continue
            idx = cell[0] * pw + cell[1]
            if not walk_arr[cell[0], cell[1]]:
                continue
            if idx not in seen_seeds:
                seen_seeds.add(idx)
                seed_indices.append(idx)
                goal_map[idx] = (ax, ay)
                hit_map[idx] = (hx, hy)

        if not seed_indices:
            self._dc_parent = parent
            self._dc_goal_map = goal_map
            self._dc_hit_map = hit_map
            return

        # Per-node destination multiplier (strongly prefer seen/highway routes over unseen walls,
        # and avoid long sub-52-px narrow slots like gaps behind tight boxes while carrying)
        known_ok = self._p_seen_arr | self._highway_grid
        squeeze_dc = np.maximum(0.0, np.float32(26.5) - self._p_edt_arr)
        carry_cost = self._p_cost_arr + np.float32(0.35) * (squeeze_dc * squeeze_dc)
        if self._dc_time_weighted:
            # Approximate travel time in hull-clear corridors. Keep a sharp cost
            # below the carrying threshold so a short, unsafe gap is still costly.
            squeeze = np.maximum(0.0, np.float32(15.5) - self._p_edt_arr)
            carry_cost = np.float32(1.0) + 0.10 * squeeze_dc + 0.65 * squeeze * squeeze
            mult_2d = carry_cost * np.where(known_ok, np.float32(1.0), np.float32(2.8))
        else:
            mult_2d = carry_cost * np.where(known_ok, np.float32(0.85), np.float32(5.5))
        if self._return_detour is not None:
            point, expires = self._return_detour
            if step < expires:
                r, c = self.world_to_plan(*point)
                r0, r1 = max(0, r - 5), min(ph, r + 6)
                c0, c1 = max(0, c - 5), min(pw, c + 6)
                dy = np.arange(r0 - r, r1 - r, dtype=np.float32)[:, None]
                dx = np.arange(c0 - c, c1 - c, dtype=np.float32)[None, :]
                mult_2d[r0:r1, c0:c1] += np.maximum(0.0, 1.0 - (dx * dx + dy * dy) / 25.0) * 22.0
            else:
                self._return_detour = None
        pen_2d = (
            np.where(~pref_arr, np.float32(extra_pen), np.float32(0.0))
            if (pref_arr is not None and extra_pen > 0.0)
            else np.zeros((ph, pw), dtype=np.float32)
        )
        mult_f = mult_2d.ravel()
        pen_f = pen_2d.ravel()

        # Vectorized 8-connected directed edges (no corner cutting)
        r_v, c_v = np.nonzero(walk_arr[:-1, :] & walk_arr[1:, :])
        v0 = r_v * pw + c_v
        v1 = v0 + pw

        r_h, c_h = np.nonzero(walk_arr[:, :-1] & walk_arr[:, 1:])
        h0 = r_h * pw + c_h
        h1 = h0 + 1

        sq_ok = walk_arr[:-1, :-1] & walk_arr[1:, :-1] & walk_arr[:-1, 1:] & walk_arr[1:, 1:]
        rd, cd = np.nonzero(sq_ok)
        d00 = rd * pw + cd
        d11 = d00 + pw + 1
        d01 = d00 + 1
        d10 = d00 + pw

        rows = np.concatenate((v0, v1, h0, h1, d00, d11, d01, d10))
        cols = np.concatenate((v1, v0, h1, h0, d11, d00, d10, d01))

        card_n = (v0.size + h0.size) * 2
        card_cols = cols[:card_n]
        diag_cols = cols[card_n:]
        w_card = mult_f[card_cols] + pen_f[card_cols]
        w_diag = np.float32(1.4142135) * mult_f[diag_cols] + pen_f[diag_cols]
        weights = np.concatenate((w_card, w_diag))

        csr = csr_matrix((weights, (rows, cols)), shape=(p_size, p_size))
        _, pred, _ = dijkstra(
            csr, directed=True, indices=seed_indices, min_only=True, return_predecessors=True
        )
        parent = pred.tolist()
        for s_idx in seed_indices:
            parent[s_idx] = s_idx

        self._dc_parent = parent
        self._dc_goal_map = goal_map
        self._dc_hit_map = hit_map

    def set_return_detour(self, point: Optional[WorldPoint], expires: int = 0) -> None:
        """Temporarily route around a locally observed repeated snag on return."""
        detour = (point, expires) if point is not None else None
        if self._return_detour != detour:
            self._return_detour = detour
            self._dc_field_dirty = True

    def get_dc_path(
        self, start: WorldPoint, step: int, fallback_target: Optional[WorldPoint] = None,
        time_weighted: bool = False,
        preferred_face: Optional[WorldPoint] = None,
    ) -> Tuple[List[WorldPoint], Optional[WorldPoint], Optional[WorldPoint], bool]:
        """Follow unified Reverse Dijkstra field back to nearest reachable DC face (or spawn)."""
        if not self.dc_points and fallback_target is None:
            return ([], None, None, False)
        if (
            time_weighted != self._dc_time_weighted
            or preferred_face != self._dc_preferred_face
            or (self.dc_points and self._dc_used_fallback)
            or (self._dc_field_dirty and step - self._dc_field_step >= 12)
            or (step - self._dc_field_step >= 36)
        ):
            self._dc_time_weighted = time_weighted
            self._dc_preferred_face = preferred_face
            self._rebuild_dc_field(
                step,
                walk_b=self._p_free_tight_b,
                pref_b=self._p_free_c_b,
                extra_pen=40.0,
                fallback_target=fallback_target,
            )

        sr, sc = self.world_to_plan(*start)
        s_cell = self._nearest_valid_plan(sr, sc, self.plan_free_tight, max_r=5)
        pw = self.pw
        cur = (s_cell[0] * pw + s_cell[1]) if s_cell is not None else (sr * pw + sc)

        if s_cell is None or self._dc_parent[cur] < 0:
            # Stage 3 fallback: bypass smeared NO_GPS bridges with heavy penalty
            if step - self._dc_field_step >= 10 or self._dc_parent[cur] < 0:
                self._rebuild_dc_field(
                    step,
                    walk_b=self._p_no_solid_b,
                    pref_b=self._p_free_tight_b,
                    extra_pen=450.0,
                    fallback_target=fallback_target,
                )
            cur = sr * pw + sc
            if self._dc_parent[cur] < 0:
                for rr in range(max(1, sr - 2), min(self.ph - 1, sr + 3)):
                    for cc in range(max(1, sc - 2), min(self.pw - 1, sc + 3)):
                        if self._dc_parent[rr * pw + cc] >= 0:
                            cur = rr * pw + cc
                            break
                    if self._dc_parent[cur] >= 0:
                        break
            if self._dc_parent[cur] < 0:
                return ([], None, None, False)

        parent = self._dc_parent
        raw: List[WorldPoint] = [start]
        guard = 0
        while parent[cur] != cur and parent[cur] >= 0 and guard < 800:
            cur = parent[cur]
            rr = cur // pw
            cc = cur % pw
            raw.append(self.plan_to_world(rr, cc))
            guard += 1

        app_pt = self._dc_goal_map.get(cur)
        hit_pt = self._dc_hit_map.get(cur)
        if app_pt is not None:
            raw.append(app_pt)
        smoothed = self.smooth_path(raw, carrying=True)
        wps = smoothed[1:] if len(smoothed) > 1 else ([app_pt] if app_pt else [])
        return (wps, app_pt, hit_pt, True)

    def obs_clearance_at(self, wx: float, wy: float) -> float:
        r, c = self.world_to_cell(wx, wy)
        return float(self.edt_px[r, c])

    def has_walls_only_los(self, p1: WorldPoint, p2: WorldPoint) -> bool:
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        dist = math.hypot(dx, dy)
        if dist < 2.0:
            return True
        n = max(2, int(math.ceil(dist / 6.0)))
        step_x = dx / n
        step_y = dy / n
        wmap = self.wall_mask
        half_w = self.width * 0.5
        half_h = self.height * 0.5
        inv_cs = 1.0 / self.CELL_SIZE
        gw_m1 = self.gw - 1
        gh_m1 = self.gh - 1
        px, py = p1[0], p1[1]
        for i in range(n + 1):
            c = int((px + i * step_x + half_w) * inv_cs)
            r = int((half_h - (py + i * step_y)) * inv_cs)
            if c < 0:
                c = 0
            elif c > gw_m1:
                c = gw_m1
            if r < 0:
                r = 0
            elif r > gh_m1:
                r = gh_m1
            if wmap[r, c]:
                return False
        return True

    def has_los(
        self, p1: WorldPoint, p2: WorldPoint, clearance: float = 14.0, require_seen: bool = False
    ) -> bool:
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        dist = math.hypot(dx, dy)
        if dist < 4.0:
            return True
        n = max(2, int(math.ceil(dist / 5.0)))
        step_x = dx / n
        step_y = dy / n
        edt = self.edt_px
        seen_m = self.free_seen if require_seen else None
        half_w = self.width * 0.5
        half_h = self.height * 0.5
        inv_cs = 1.0 / self.CELL_SIZE
        gw_m1 = self.gw - 1
        gh_m1 = self.gh - 1
        px, py = p1[0], p1[1]
        for i in range(1, n):
            c = int((px + i * step_x + half_w) * inv_cs)
            r = int((half_h - (py + i * step_y)) * inv_cs)
            if c < 0:
                c = 0
            elif c > gw_m1:
                c = gw_m1
            if r < 0:
                r = 0
            elif r > gh_m1:
                r = gh_m1
            if edt[r, c] < clearance:
                return False
            if seen_m is not None and not seen_m[r, c]:
                return False
        return True

    def smooth_path(self, pts: List[WorldPoint], carrying: bool = False) -> List[WorldPoint]:
        if len(pts) <= 2:
            return pts
        if len(pts) > 10:
            pts = pts[::2] + ([pts[-1]] if (len(pts) - 1) % 2 != 0 else [])
        clr = self.SMOOTH_CLEARANCE_CARRYING if carrying else self.SMOOTH_CLEARANCE_UNLOADED
        out = [pts[0]]
        i = 0
        n = len(pts)
        while i < n - 1:
            lo = i + 1
            hi = min(n - 1, i + 28)
            if self.has_los(out[-1], pts[hi], clearance=clr, require_seen=False):
                best = hi
            else:
                best = lo
                l = lo + 1
                r = hi - 1
                while l <= r:
                    mid = (l + r) // 2
                    if self.has_los(out[-1], pts[mid], clearance=clr, require_seen=False):
                        best = mid
                        l = mid + 1
                    else:
                        r = mid - 1
            out.append(pts[best])
            i = best
        return out

    def _nearest_valid_plan(
        self,
        r0: int,
        c0: int,
        free_mask: np.ndarray,
        max_r: int = 5,
        los_ref: Optional[WorldPoint] = None,
    ) -> Optional[Tuple[int, int]]:
        if free_mask[r0, c0]:
            return (r0, c0)
        best: Optional[Tuple[int, int]] = None
        best_d2 = 1e9
        ph, pw = self.ph, self.pw
        for rad in range(1, max_r + 1):
            r_lo = max(1, r0 - rad)
            r_hi = min(ph - 2, r0 + rad)
            c_lo = max(1, c0 - rad)
            c_hi = min(pw - 2, c0 + rad)
            for rr in range(r_lo, r_hi + 1):
                for cc in range(c_lo, c_hi + 1):
                    if free_mask[rr, cc]:
                        if los_ref is not None:
                            wx, wy = self.plan_to_world(rr, cc)
                            # Stop 4 px shy of los_ref so bombs near a wall don't fail on their own wall pixel
                            dx = los_ref[0] - wx
                            dy = los_ref[1] - wy
                            dl = math.hypot(dx, dy)
                            tx = los_ref[0] - (4.0 * dx / dl if dl > 5.0 else 0.0)
                            ty = los_ref[1] - (4.0 * dy / dl if dl > 5.0 else 0.0)
                            if not self.has_walls_only_los((wx, wy), (tx, ty)):
                                continue
                        d2 = (rr - r0) ** 2 + (cc - c0) ** 2
                        if d2 < best_d2:
                            best_d2 = d2
                            best = (rr, cc)
            if best is not None:
                return best
        return None

    def plan_path(
        self,
        start: WorldPoint,
        goal: WorldPoint,
        carrying: bool = False,
        allow_unknown: bool = True,
        max_expansions: int = 8000,
        allow_fallback: bool = True,
        check_goal_los: bool = False,
        allow_door: bool = False,
    ) -> Tuple[List[WorldPoint], float, bool]:
        """Fast 8-px A* with optional relaxed doorway/NO_GPS fallback."""
        clr = self.SMOOTH_CLEARANCE_CARRYING if carrying else self.SMOOTH_CLEARANCE_UNLOADED
        if self.has_los(start, goal, clearance=clr):
            return ([goal], math.hypot(goal[0] - start[0], goal[1] - start[1]), True)

        free_base = self.plan_free_carrying if carrying else self.plan_free_unloaded
        walk_b = self._p_free_c_b if carrying else self._p_free_u_b
        wps, dist, reached = self._plan_path_core(
            start, goal, free_base, walk_b, None, 0.0, carrying, allow_unknown, max_expansions, check_goal_los
        )
        if reached or not allow_unknown:
            return (wps, dist, reached)

        # Always try wall-safe tight clearance (9.0 px) so compass-smeared 40-px tunnels stay enterable
        fb_exp = min(max_expansions, 7500)
        wps2, dist2, rch2 = self._plan_path_core(
            start, goal, self.plan_free_tight, self._p_free_tight_b, walk_b, 40.0, carrying, True, fb_exp, check_goal_los
        )
        if rch2:
            return (wps2, dist2, True)
        if allow_door and (not carrying) and math.hypot(goal[0] - start[0], goal[1] - start[1]) < 160.0:
            # Local non-wall tunnel squeeze (<160 px, GPS-anchored only) for smeared 40-px dead-end alcoves
            wps2b, dist2b, rch2b = self._plan_path_core(
                start, goal, self.plan_free_door, self._p_free_door_b, self._p_free_tight_b, 60.0, carrying, True, 3500, True
            )
            if rch2b:
                return (wps2b, dist2b, True)
        if not allow_fallback:
            return (wps2, dist2, False)

        wps3, dist3, rch3 = self._plan_path_core(
            start, goal, self.plan_free_tight, self._p_no_solid_b, self._p_free_tight_b, 450.0, carrying, True, fb_exp, check_goal_los
        )
        if rch3:
            return (wps3, dist3, True)
        return (wps, dist, False)

    def _plan_path_core(
        self,
        start: WorldPoint,
        goal: WorldPoint,
        free_mask: np.ndarray,
        walkable_b: bytes,
        pref_b: Optional[bytes],
        extra_pen: float,
        carrying: bool,
        allow_unknown: bool,
        max_expansions: int,
        check_goal_los: bool = False,
    ) -> Tuple[List[WorldPoint], float, bool]:
        sr, sc = self.world_to_plan(*start)
        gr, gc = self.world_to_plan(*goal)

        s_cell = self._nearest_valid_plan(sr, sc, free_mask, max_r=4)
        g_cell = self._nearest_valid_plan(
            gr, gc, free_mask, max_r=4, los_ref=(goal if check_goal_los else None)
        )
        if g_cell is None and allow_unknown and not check_goal_los:
            g_cell = self._nearest_valid_plan(gr, gc, free_mask, max_r=5)
        if s_cell is None or g_cell is None:
            s_cell = s_cell or (sr, sc)
            g_cell = g_cell or (gr, gc)
            if not walkable_b[s_cell[0] * self.pw + s_cell[1]] or not walkable_b[g_cell[0] * self.pw + g_cell[1]]:
                return ([start], math.hypot(goal[0] - start[0], goal[1] - start[1]), False)

        sr, sc = s_cell
        gr, gc = g_cell
        if sr == gr and sc == gc:
            return ([goal], math.hypot(goal[0] - start[0], goal[1] - start[1]), True)

        ph, pw = self.ph, self.pw
        p_size = self.p_size
        seen_b = self._p_seen_b
        hw_b = self._p_highway_b
        cost_l = self._p_cost_list
        start_idx = sr * pw + sc
        goal_idx = gr * pw + gc

        # Reuse preallocated flat scratch arrays (fast list multiplication copy in C)
        g_score = [1e18] * p_size
        came_from = [-1] * p_size
        closed = bytearray(p_size)
        g_score[start_idx] = 0.0

        hypot = math.hypot
        heappush = heapq.heappush
        heappop = heapq.heappop

        h0 = hypot(gr - sr, gc - sc)
        heap: List[Tuple[float, float, int]] = [(1.30 * h0, 0.0, start_idx)]

        offsets = (
            (-pw, 1.0, 0, 0),
            (pw, 1.0, 0, 0),
            (-1, 1.0, 0, 0),
            (1, 1.0, 0, 0),
            (-pw - 1, 1.4142135, -pw, -1),
            (-pw + 1, 1.4142135, -pw, 1),
            (pw - 1, 1.4142135, pw, -1),
            (pw + 1, 1.4142135, pw, 1),
        )

        best_idx = start_idx
        best_h = h0
        reached = False
        expansions = 0
        limit_exp = min(max_expansions, 15000)

        while heap and expansions < limit_exp:
            _, cur_g, cur = heappop(heap)
            if closed[cur]:
                continue
            closed[cur] = 1
            expansions += 1

            if cur == goal_idx:
                reached = True
                best_idx = cur
                break

            cr = cur // pw
            cc = cur % pw
            dx0 = gr - cr
            if dx0 < 0:
                dx0 = -dx0
            dy0 = gc - cc
            if dy0 < 0:
                dy0 = -dy0
            ch_val = (dx0 + dy0) - 0.5857864 * (dx0 if dx0 < dy0 else dy0)
            if ch_val < best_h:
                best_h = ch_val
                best_idx = cur

            for d_idx, step_l, c1, c2 in offsets:
                nidx = cur + d_idx
                if nidx < 0 or nidx >= p_size or closed[nidx]:
                    continue
                nc = nidx % pw
                if nc - cc > 1 or cc - nc > 1:
                    continue
                if not walkable_b[nidx]:
                    continue
                if not allow_unknown and not seen_b[nidx]:
                    continue
                if c1 != 0:
                    if not (walkable_b[cur + c1] and walkable_b[cur + c2]):
                        continue
                nr = nidx // pw
                mult = cost_l[nidx]
                if seen_b[nidx] or hw_b[nidx]:
                    mult *= 0.88
                else:
                    mult *= 1.28
                pen = extra_pen if (pref_b is not None and not pref_b[nidx]) else 0.0
                ng = cur_g + step_l * mult + pen
                if ng < g_score[nidx]:
                    g_score[nidx] = ng
                    came_from[nidx] = cur
                    dx = gr - nr
                    if dx < 0:
                        dx = -dx
                    dy = gc - nc
                    if dy < 0:
                        dy = -dy
                    oct_h = (dx + dy) - 0.5857864 * (dx if dx < dy else dy)
                    heappush(heap, (ng + 1.30 * oct_h, ng, nidx))

        if not reached and best_idx == start_idx:
            return ([goal], hypot(goal[0] - start[0], goal[1] - start[1]), False)

        rev_pts: List[WorldPoint] = []
        cur = best_idx
        while cur != start_idx and cur >= 0:
            rr = cur // pw
            cc = cur % pw
            rev_pts.append(self.plan_to_world(rr, cc))
            cur = came_from[cur]
        rev_pts.reverse()

        if reached:
            if rev_pts:
                rev_pts[-1] = goal
            else:
                rev_pts.append(goal)

        raw = [start] + rev_pts
        smoothed = self.smooth_path(raw, carrying=carrying)
        waypoints = smoothed[1:] if len(smoothed) > 1 else [goal]
        total_dist = 0.0
        prev = start
        for wp in waypoints:
            total_dist += hypot(wp[0] - prev[0], wp[1] - prev[1])
            prev = wp
        return (waypoints, total_dist, reached)

    def path_is_obstructed(self, pos: WorldPoint, waypoints: List[WorldPoint], carrying: bool) -> bool:
        if not waypoints:
            return False
        clr = 10.0 if carrying else 9.0
        prev = pos
        for nxt in waypoints[:2]:
            dx = nxt[0] - prev[0]
            dy = nxt[1] - prev[1]
            d = math.hypot(dx, dy)
            if d > 140.0 and d > 1e-3:
                end = (prev[0] + dx * 140.0 / d, prev[1] + dy * 140.0 / d)
                if not self.has_los(prev, end, clearance=clr):
                    return True
                break
            if not self.has_los(prev, nxt, clearance=clr):
                return True
            prev = nxt
        return False

    def mark_goal_unreachable(self, goal: WorldPoint, until_step: int) -> None:
        key = (int(round(goal[0] / 24.0)), int(round(goal[1] / 24.0)))
        self.unreachable_until[key] = max(self.unreachable_until.get(key, 0), until_step)

    def is_goal_unreachable(self, goal: WorldPoint, step: int) -> bool:
        key = (int(round(goal[0] / 24.0)), int(round(goal[1] / 24.0)))
        return self.unreachable_until.get(key, -1) > step
