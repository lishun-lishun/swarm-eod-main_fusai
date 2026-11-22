"""
Online 24-px semantic coverage & dedicated NO_COM_ZONE 10-tile coprime
spatial partitioning with 1-pass BFS frontier extraction (zero preloaded maps).
"""

from __future__ import annotations

import math
from typing import Dict, Iterable, List, Optional, Tuple

import numpy as np
from scipy.sparse.csgraph import dijkstra

from swarm_rescue.solutions.nav_grid import OnlineMap, WorldPoint


class CoverageTracker:
    """Tracks explored semantic sweep area and coordinates zero-comm partitioning."""

    CELL_PX = 24  # Exact 3x multiple of OnlineMap.PLAN_CELL (8 px)
    SEMANTIC_SWEEP_R = 76.0
    _SWEEP_STEPS = np.arange(14.0, 77.0, 16.0, dtype=np.float32)[None, :]

    def __init__(self, online_map: OnlineMap) -> None:
        self.omap = online_map
        self.width = online_map.width
        self.height = online_map.height
        self.cw = max(1, (self.width + self.CELL_PX - 1) // self.CELL_PX)
        self.ch = max(1, (self.height + self.CELL_PX - 1) // self.CELL_PX)

        self.seen = np.zeros((self.ch, self.cw), dtype=bool)
        self.last_seen_step = np.zeros((self.ch, self.cw), dtype=np.int32)
        self._blocked_until = np.zeros((self.ch, self.cw), dtype=np.int32)
        self._cr_grid, self._cc_grid = np.meshgrid(
            np.arange(self.ch, dtype=np.int32),
            np.arange(self.cw, dtype=np.int32),
            indexing="ij",
        )

    def cell_to_world(self, r: int, c: int) -> WorldPoint:
        wx = min(self.width * 0.5 - 16.0, max(-self.width * 0.5 + 16.0, (c + 0.5) * self.CELL_PX - self.width * 0.5))
        wy = min(self.height * 0.5 - 16.0, max(-self.height * 0.5 + 16.0, self.height * 0.5 - (r + 0.5) * self.CELL_PX))
        return (wx, wy)

    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        c = int((x + self.width * 0.5) // self.CELL_PX)
        r = int((self.height * 0.5 - y) // self.CELL_PX)
        if c < 0:
            c = 0
        elif c >= self.cw:
            c = self.cw - 1
        if r < 0:
            r = 0
        elif r >= self.ch:
            r = self.ch - 1
        return (r, c)

    def _tile_of_cell(self, cr: int, cc: int) -> int:
        """Partition coverage grid into 5x2 = 10 spatial tiles for NO_COM_ZONE deconfliction."""
        tcol = min(4, (cc * 5) // max(1, self.cw))
        trow = 0 if cr < max(1, self.ch // 2) else 1
        return trow * 5 + tcol

    def mark_unreachable(self, pt: WorldPoint, until_step: int) -> None:
        r, c = self.world_to_cell(*pt)
        self._blocked_until[r, c] = max(int(self._blocked_until[r, c]), until_step)

    def update_from_pose(
        self,
        pos: WorldPoint,
        heading: float,
        lidar_vals: Optional[np.ndarray],
        lidar_angles: Optional[np.ndarray],
        step: int,
    ) -> None:
        """Vectorized marking of cells swept by semantic sensor along LiDAR rays."""
        r0, c0 = self.world_to_cell(*pos)
        self.seen[r0, c0] = True
        self.last_seen_step[r0, c0] = step

        if lidar_vals is None or lidar_angles is None:
            rad = int(self.SEMANTIC_SWEEP_R // self.CELL_PX)
            for rr in range(max(0, r0 - rad), min(self.ch, r0 + rad + 1)):
                for cc in range(max(0, c0 - rad), min(self.cw, c0 + rad + 1)):
                    wx, wy = self.cell_to_world(rr, cc)
                    if math.hypot(wx - pos[0], wy - pos[1]) <= self.SEMANTIC_SWEEP_R:
                        if self.omap.has_walls_only_los(pos, (wx, wy)):
                            self.seen[rr, cc] = True
                            self.last_seen_step[rr, cc] = step
            return

        dists_raw = np.asarray(lidar_vals[::2], dtype=np.float32)
        angs_raw = np.asarray(lidar_angles[::2], dtype=np.float32)
        d_max = np.where(np.isfinite(dists_raw), np.minimum(self.SEMANTIC_SWEEP_R, dists_raw - 20.0), self.SEMANTIC_SWEEP_R)
        keep = d_max > 8.0
        if not np.any(keep):
            return

        d_max = d_max[keep]
        w_angs = heading + angs_raw[keep]
        mask = self._SWEEP_STEPS <= d_max[:, None]
        if not np.any(mask):
            return

        sx = (pos[0] + np.cos(w_angs)[:, None] * self._SWEEP_STEPS)[mask]
        sy = (pos[1] + np.sin(w_angs)[:, None] * self._SWEEP_STEPS)[mask]
        inv_c = 1.0 / self.CELL_PX
        cols = np.clip(((sx + self.width * 0.5) * inv_c).astype(np.int32), 0, self.cw - 1)
        rows = np.clip(((self.height * 0.5 - sy) * inv_c).astype(np.int32), 0, self.ch - 1)
        self.seen[rows, cols] = True
        self.last_seen_step[rows, cols] = step

    def pack_seen(self) -> bytes:
        return np.packbits(self.seen.ravel()).tobytes()

    def merge_packed(self, packed: bytes, step: int) -> Optional[np.ndarray]:
        arr = np.unpackbits(np.frombuffer(packed, dtype=np.uint8))
        n = self.ch * self.cw
        if arr.size < n:
            return None
        mask = arr[:n].reshape((self.ch, self.cw)).astype(bool)
        newly = mask & (~self.seen)
        self.seen |= mask
        self.last_seen_step[newly] = step
        return mask

    def pick_exploration_with_path(
        self,
        my_id: int,
        num_drones: int,
        my_pos: WorldPoint,
        peer_targets: Iterable[WorldPoint],
        current_step: int,
        comm_lost: bool = False,
        vel_dir: Optional[WorldPoint] = None,
    ) -> Tuple[WorldPoint, List[WorldPoint]]:
        """
        Single-pass O(N) BFS over OnlineMap's 8-px planning grid with dedicated
        NO_COM_ZONE 10-tile coprime partitioning when radio comms are absent.
        """
        omap = self.omap
        ph, pw = omap.ph, omap.pw
        p_size = omap.p_size
        walkable_b = omap._p_free_u_b
        seen_p_b = omap._p_seen_b

        sr, sc = omap.world_to_plan(*my_pos)
        s_cell = omap._nearest_valid_plan(sr, sc, omap.plan_free_unloaded, max_r=5)
        start_mode = "u"
        if s_cell is None:
            s_cell = omap._nearest_valid_plan(sr, sc, omap.plan_free_tight, max_r=5)
            start_mode = "t"
        if s_cell is None:
            s_cell = omap._nearest_valid_plan(sr, sc, omap.plan_free_door, max_r=5)
            start_mode = "d"
        if s_cell is None:
            return (my_pos, [my_pos])

        start_idx = s_cell[0] * pw + s_cell[1]
        unseen = ~self.seen
        not_blocked = self._blocked_until <= current_step
        ch, cw = self.ch, self.cw
        plan_cell_px = float(omap.PLAN_CELL)
        stale_thresh = current_step - 210
        cr_grid = self._cr_grid
        cc_grid = self._cc_grid

        def run_bfs(mode: str):
            csr = omap.get_walk_csr(mode)
            dist_arr, pred = dijkstra(
                csr, directed=True, indices=start_idx, unweighted=True, return_predecessors=True
            )
            d2d = np.full((ch * 3, cw * 3), np.inf, dtype=np.float32)
            d2d[:ph, :pw] = dist_arr.reshape(ph, pw)
            blk_flat = d2d.reshape(ch, 3, cw, 3).transpose(0, 2, 1, 3).reshape(ch, cw, 9)
            best_sub = np.argmin(blk_flat, axis=2)
            best_d = blk_flat[cr_grid, cc_grid, best_sub]
            reach = np.isfinite(best_d) & not_blocked

            f_unseen: Dict[Tuple[int, int], Tuple[int, int, int, float]] = {}
            f_stale: Dict[Tuple[int, int], Tuple[int, int, int, float]] = {}

            unseen_reach = reach & unseen
            if np.any(unseen_reach):
                ru, cu = np.nonzero(unseen_reach)
                du = best_d[ru, cu]
                order = np.argsort(du, kind="mergesort")
                sub_u = best_sub[ru, cu]
                for oi in order.tolist():
                    d_st = int(du[oi])
                    cr = int(ru[oi])
                    cc = int(cu[oi])
                    ckey = (cr, cc) if d_st <= 40 else (1000 + (cr >> 1), 1000 + (cc >> 1))
                    if ckey not in f_unseen:
                        s_sub = int(sub_u[oi])
                        pr = cr * 3 + s_sub // 3
                        pc = cc * 3 + s_sub % 3
                        if pr < ph and pc < pw:
                            f_unseen[ckey] = (cr, cc, pr * pw + pc, d_st * plan_cell_px)
                            if len(f_unseen) >= 550:
                                break
            else:
                stale_reach = reach & (self.last_seen_step < stale_thresh)
                if np.any(stale_reach):
                    rs, cs_arr = np.nonzero(stale_reach)
                    ds = best_d[rs, cs_arr]
                    order = np.argsort(ds, kind="mergesort")
                    sub_s = best_sub[rs, cs_arr]
                    for oi in order[:90].tolist():
                        cr = int(rs[oi])
                        cc = int(cs_arr[oi])
                        s_sub = int(sub_s[oi])
                        pr = cr * 3 + s_sub // 3
                        pc = cc * 3 + s_sub % 3
                        if pr < ph and pc < pw:
                            f_stale[(cr, cc)] = (cr, cc, pr * pw + pc, float(ds[oi]) * plan_cell_px)

            return f_unseen, f_stale, pred

        found_unseen, found_stale, came_from = run_bfs(start_mode)
        if not found_unseen and start_mode == "u":
            fu2, fs2, cf2 = run_bfs("t")
            if fu2:
                found_unseen, found_stale, came_from = fu2, fs2, cf2
                start_mode = "t"
            elif fs2 and not found_stale:
                found_stale, came_from = fs2, cf2
        if not found_unseen and start_mode in ("u", "t") and current_step >= 1000:
            fu3, fs3, cf3 = run_bfs("d")
            if fu3:
                found_unseen, found_stale, came_from = fu3, fs3, cf3
            elif fs3 and not found_stale:
                found_stale, came_from = fs3, cf3

        pool = found_unseen if found_unseen else found_stale
        if not pool:
            return (my_pos, [my_pos])

        # Determine active NO_COM_ZONE tile priority using coprime stride 3
        tile_rank = {((my_id + k * 3) % 10): k for k in range(10)}
        peers = list(peer_targets)
        pref_ang = -math.pi + (2.0 * math.pi * (my_id % max(1, num_drones))) / max(1, num_drones)

        # Find highest-priority tile currently represented in reachable pool
        best_tile_rank = min(tile_rank[self._tile_of_cell(cr, cc)] for cr, cc, _, _ in pool.values())

        best_pidx = -1
        best_score = float("inf")
        best_pt = my_pos

        for cr, cc, pidx, bfs_dist in pool.values():
            if bfs_dist < 20.0 and len(pool) > 1:
                continue
            pr = pidx // pw
            pc = pidx % pw
            wx, wy = omap.plan_to_world(pr, pc)

            r0, r1 = max(0, cr - 2), min(ch, cr + 3)
            c0, c1 = max(0, cc - 2), min(cw, cc + 3)
            density = int(np.count_nonzero(unseen[r0:r1, c0:c1]))
            known_free_bonus = 75.0 if seen_p_b[pidx] else 0.0

            peer_pen = 0.0
            rep_r = 350.0 if not comm_lost else 280.0
            rep_w = 3.2 if not comm_lost else 2.4
            for px, py in peers:
                dp = math.hypot(wx - px, wy - py)
                if dp < rep_r:
                    peer_pen += (rep_r - dp) * rep_w

            # Tile partitioning guides early NO_COM_ZONE dispersion without wasting late steps
            tr = tile_rank[self._tile_of_cell(cr, cc)]
            if comm_lost:
                if current_step < 1400:
                    tile_bonus = 180.0 if tr == best_tile_rank else (80.0 if tr == best_tile_rank + 1 else 0.0)
                else:
                    tile_bonus = 0.0
                ang_w = 55.0
            else:
                tile_bonus = 0.0
                ang_w = 50.0

            ang = math.atan2(wy, wx)
            ang_diff = abs((ang - pref_ang + math.pi) % (2.0 * math.pi) - math.pi)
            score = (
                bfs_dist
                + peer_pen
                - density * 18.0
                - known_free_bonus
                - tile_bonus
                + ang_diff * ang_w
            )
            if score < best_score:
                best_score = score
                best_pidx = pidx
                best_pt = (wx, wy)

        if best_pidx < 0:
            cr, cc, best_pidx, _ = next(iter(pool.values()))
            best_pt = omap.plan_to_world(best_pidx // pw, best_pidx % pw)

        rev: List[WorldPoint] = []
        cur = best_pidx
        guard = 0
        while cur != start_idx and cur >= 0 and guard < 2000:
            rev.append(omap.plan_to_world(cur // pw, cur % pw))
            cur = int(came_from[cur])
            guard += 1
        rev.reverse()
        raw = [my_pos] + rev if rev else [my_pos, best_pt]
        smoothed = omap.smooth_path(raw, carrying=False)
        wps = smoothed[1:] if len(smoothed) > 1 else [best_pt]
        return (best_pt, wps)

    def pick_exploration_target(
        self,
        my_id: int,
        num_drones: int,
        my_pos: WorldPoint,
        peer_targets: Iterable[WorldPoint],
        current_step: int,
        comm_lost: bool = False,
    ) -> WorldPoint:
        tgt, _ = self.pick_exploration_with_path(
            my_id=my_id,
            num_drones=num_drones,
            my_pos=my_pos,
            peer_targets=peer_targets,
            current_step=current_step,
            comm_lost=comm_lost,
        )
        return tgt
