"""
Sensor-fusion pose and velocity estimator with dedicated NO_GPS_ZONE
localization: GPS + Compass + Odometer + Dual-Stage LiDAR Scan-Matching
(GPS-anchored map match + Scan-to-Local-Keyframe submap match to eliminate
odometry heading random walk).
"""

from __future__ import annotations

import math
from collections import deque
from typing import Deque, List, Optional, Tuple

import cv2
import numpy as np

from swarm_rescue.simulation.ray_sensors.drone_semantic_sensor import DroneSemanticSensor
from swarm_rescue.simulation.utils.utils import normalize_angle
from swarm_rescue.solutions.nav_grid import OnlineMap


class PoseEstimator:
    """
    Fuses GPS, compass, odometer, and two-tier LiDAR scan-matching:
      1) Global GPS-anchor scan match whenever GPS-mapped walls are visible.
      2) Local keyframe submap scan match deep inside NO_GPS_ZONE to suppress
         the 1 deg/step odometer heading random walk and cross-track drift.
    """

    _SUBMAP_CELL_PX = 4.0
    _SUBMAP_HALF_CELLS = 56  # 112x112 grid -> 448x448 px local window

    def __init__(self) -> None:
        self.initialized = False
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        self.in_no_gps = False
        self._gps_streak = 0
        self._no_gps_init_wait = 0

        # Local keyframe sliding window + persistent GPS-boundary anchor keyframes
        self._was_no_gps = False
        self._step_cnt = 0
        self._last_recovery_step = -100
        self._last_kf_step = -100
        self._last_kf_pos: Optional[Tuple[float, float]] = None
        self._keyframes: Deque[Tuple[np.ndarray, np.ndarray, bool]] = deque(maxlen=10)
        self._anchor_kfs: Deque[Tuple[np.ndarray, np.ndarray, float, float]] = deque(maxlen=28)
        self._submap_edt: Optional[np.ndarray] = None
        self._submap_origin: Tuple[float, float] = (0.0, 0.0)  # center world (cx, cy)
        self._wall_hist = np.zeros(90, dtype=np.float32)  # GPS-learned dominant wall axis mod 90 deg

    @property
    def gps_settled(self) -> bool:
        return (not self.in_no_gps) and self._gps_streak >= 6

    def update(
        self,
        gps_pos: Optional[np.ndarray],
        compass_ang: Optional[float],
        odom_vals: Optional[np.ndarray],
        lidar_vals: Optional[np.ndarray],
        lidar_angles: Optional[np.ndarray],
        semantic_vals: Optional[list],
        online_map: Optional[OnlineMap],
        peer_xy: Optional[List[Tuple[float, float]]] = None,
    ) -> Tuple[float, float, float]:
        self._step_cnt += 1
        gps_ok = (
            gps_pos is not None
            and len(gps_pos) >= 2
            and math.isfinite(float(gps_pos[0]))
            and math.isfinite(float(gps_pos[1]))
        )
        comp_ok = compass_ang is not None and math.isfinite(float(compass_ang))
        self.in_no_gps = not (gps_ok and comp_ok)
        if not self.in_no_gps:
            self._gps_streak += 1
        else:
            self._gps_streak = 0

        if not self.initialized:
            if gps_ok and comp_ok:
                self.x = float(gps_pos[0])
                self.y = float(gps_pos[1])
                self.heading = float(compass_ang)
                self.initialized = True
                return (self.x, self.y, self.heading)
            self._no_gps_init_wait += 1
            if self._no_gps_init_wait >= 2:
                self.initialized = True
            return (self.x, self.y, self.heading)

        # 1. Odometer prediction (available inside NO_GPS_ZONE)
        prev_heading = self.heading
        if odom_vals is not None and len(odom_vals) >= 3:
            dist = float(odom_vals[0])
            alpha = float(odom_vals[1])
            dtheta = float(odom_vals[2])
            if math.isfinite(dist) and math.isfinite(alpha) and math.isfinite(dtheta):
                move_ang = prev_heading + alpha
                raw_vx = dist * math.cos(move_ang)
                raw_vy = dist * math.sin(move_ang)
                self.x += raw_vx
                self.y += raw_vy
                self.heading = normalize_angle(prev_heading + dtheta)
                self.vx = 0.65 * raw_vx + 0.35 * self.vx
                self.vy = 0.65 * raw_vy + 0.35 * self.vy
                self.omega = dtheta

        # 2. GPS + Compass correction when outside NO_GPS_ZONE
        if gps_ok and comp_ok:
            gx, gy = float(gps_pos[0]), float(gps_pos[1])
            gc = float(compass_ang)
            err_pos = math.hypot(gx - self.x, gy - self.y)
            if self._was_no_gps:
                blend_pos = 1.0
                blend_ang = 1.0
                self._keyframes.clear()
                self._submap_edt = None
                self._was_no_gps = False
            else:
                blend_pos = 0.65 if err_pos > 18.0 else 0.22
                err_ang_chk = normalize_angle(gc - self.heading)
                blend_ang = 0.65 if abs(err_ang_chk) > 0.22 else 0.20

            self.x += blend_pos * (gx - self.x)
            self.y += blend_pos * (gy - self.y)

            err_ang = normalize_angle(gc - self.heading)
            self.heading = normalize_angle(self.heading + blend_ang * err_ang)

            # Record trusted anchor keyframes & online wall orientation histogram right before NO_GPS_ZONE
            if self.gps_settled and lidar_vals is not None and lidar_angles is not None:
                if float(np.sum(self._wall_hist)) < 150.0 or (self._step_cnt % 2 == 0):
                    d_w, a_w = self._extract_wall_rays(
                        lidar_vals, lidar_angles, semantic_vals, peer_xy, stride=1
                    )
                    if d_w is not None and d_w.size >= 24:
                        self._accumulate_wall_orientations(d_w, a_w)
                self._maybe_push_keyframe(lidar_vals, lidar_angles, semantic_vals, trusted=True, peer_xy=peer_xy)
            return (self.x, self.y, self.heading)

        self._was_no_gps = True

        # 3. Dual-Stage LiDAR scan-matching inside NO_GPS_ZONE
        if lidar_vals is not None and lidar_angles is not None:
            dists, angs = self._extract_wall_rays(lidar_vals, lidar_angles, semantic_vals, peer_xy)
            if dists is not None and dists.size >= 10:
                matched_global = False
                if online_map is not None and online_map.gps_wall_count >= 25:
                    matched_global = self._match_global_gps_map(dists, angs, online_map)

                if (not matched_global) and (self._submap_edt is not None):
                    self._match_local_submap(dists, angs)

                d_full, a_full = self._extract_wall_rays(
                    lidar_vals, lidar_angles, semantic_vals, peer_xy, stride=1
                )
                if d_full is not None and d_full.size >= 24:
                    self._stabilize_heading_from_walls(d_full, a_full)
                self._maybe_push_keyframe(lidar_vals, lidar_angles, semantic_vals, trusted=False, peer_xy=peer_xy)

        return (self.x, self.y, self.heading)

    def _extract_wall_rays(
        self,
        lidar_vals: np.ndarray,
        lidar_angles: np.ndarray,
        semantic_vals: Optional[list],
        peer_xy: Optional[List[Tuple[float, float]]] = None,
        stride: int = 3,
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        dists = lidar_vals[::stride]
        angs = lidar_angles[::stride]
        valid = np.isfinite(dists) & (dists > 14.0) & (dists < 210.0)
        if np.count_nonzero(valid) < 10:
            return (None, None)
        dists = dists[valid].astype(np.float32)
        angs = angs[valid].astype(np.float32)

        if semantic_vals:
            dyn_objs = []
            for obj in semantic_vals:
                if obj.entity_type in (
                    DroneSemanticSensor.TypeEntity.DRONE,
                    DroneSemanticSensor.TypeEntity.BOMB,
                ):
                    oa = float(obj.angle)
                    od = float(obj.distance)
                    if math.isfinite(oa) and math.isfinite(od):
                        half_w = min(0.65, max(0.18, math.atan2(26.0, max(20.0, od))))
                        dyn_objs.append((oa, od, half_w))
            if dyn_objs:
                da = np.asarray([o[0] for o in dyn_objs], dtype=np.float32)
                dd = np.asarray([o[1] for o in dyn_objs], dtype=np.float32)
                hw = np.asarray([o[2] for o in dyn_objs], dtype=np.float32)
                diff = np.abs(
                    np.arctan2(
                        np.sin(angs[:, None] - da[None, :]),
                        np.cos(angs[:, None] - da[None, :]),
                    )
                )
                hit_dyn = (diff < hw[None, :]) & (np.abs(dists[:, None] - dd[None, :]) < 36.0)
                keep = ~np.any(hit_dyn, axis=1)
                if np.count_nonzero(keep) < 10:
                    return (None, None)
                dists = dists[keep]
                angs = angs[keep]

        if peer_xy:
            w_ang = self.heading + angs
            hx = self.x + dists * np.cos(w_ang)
            hy = self.y + dists * np.sin(w_ang)
            px_arr = np.asarray([p[0] for p in peer_xy], dtype=np.float32)
            py_arr = np.asarray([p[1] for p in peer_xy], dtype=np.float32)
            d2_peer = (hx[:, None] - px_arr[None, :]) ** 2 + (hy[:, None] - py_arr[None, :]) ** 2
            keep_p = np.all(d2_peer > (32.0 * 32.0), axis=1)
            if np.count_nonzero(keep_p) < 10:
                return (None, None)
            dists = dists[keep_p]
            angs = angs[keep_p]
        return (dists, angs)

    def _maybe_push_keyframe(
        self,
        lidar_vals: np.ndarray,
        lidar_angles: np.ndarray,
        semantic_vals: Optional[list],
        trusted: bool,
        peer_xy: Optional[List[Tuple[float, float]]] = None,
    ) -> None:
        dist_moved = (
            math.hypot(self.x - self._last_kf_pos[0], self.y - self._last_kf_pos[1])
            if self._last_kf_pos is not None
            else 999.0
        )
        if trusted:
            if dist_moved < 55.0 and (self._step_cnt - self._last_kf_step) < 24:
                return
        else:
            # Keep longer keyframe spacing inside NO_GPS_ZONE so early frames persist
            if self._keyframes and dist_moved < 60.0:
                return

        dists, angs = self._extract_wall_rays(lidar_vals, lidar_angles, semantic_vals, peer_xy)
        if dists is None or dists.size < 10:
            return

        # Keep rays <= 170 px so keyframe submap stays crisp
        close = dists <= 175.0
        if np.count_nonzero(close) < 10:
            return
        dists = dists[close]
        angs = angs[close]

        w_ang = self.heading + angs
        wx = self.x + dists * np.cos(w_ang)
        wy = self.y + dists * np.sin(w_ang)
        if trusted:
            # Keep permanent GPS-verified keyframes spaced >= 70 px for loop closure
            if not self._anchor_kfs or min(
                math.hypot(self.x - ax, self.y - ay) for _, _, ax, ay in self._anchor_kfs
            ) >= 70.0:
                self._anchor_kfs.append((wx, wy, self.x, self.y))
        self._keyframes.append((wx, wy, trusted))
        self._last_kf_step = self._step_cnt
        self._last_kf_pos = (self.x, self.y)
        self._rebuild_local_submap_edt()

    def _rebuild_local_submap_edt(self) -> None:
        if not self._keyframes and not self._anchor_kfs:
            return
        half_n = self._SUBMAP_HALF_CELLS
        size_n = half_n * 2
        cs = self._SUBMAP_CELL_PX
        cx, cy = self.x, self.y
        occ = np.zeros((size_n, size_n), dtype=np.uint8)

        # Include nearby persistent GPS-verified anchor keyframes first for loop closure
        for wx, wy, ax, ay in self._anchor_kfs:
            if math.hypot(ax - cx, ay - cy) < 420.0:
                cols = np.rint((wx - cx) / cs + half_n).astype(np.int32)
                rows = np.rint(half_n - (wy - cy) / cs).astype(np.int32)
                inb = (rows >= 0) & (rows < size_n) & (cols >= 0) & (cols < size_n)
                if np.any(inb):
                    occ[rows[inb], cols[inb]] = 1

        for wx, wy, _ in self._keyframes:
            cols = np.rint((wx - cx) / cs + half_n).astype(np.int32)
            rows = np.rint(half_n - (wy - cy) / cs).astype(np.int32)
            inb = (rows >= 0) & (rows < size_n) & (cols >= 0) & (cols < size_n)
            if np.any(inb):
                occ[rows[inb], cols[inb]] = 1

        if np.count_nonzero(occ) < 12:
            self._submap_edt = None
            return
        fg = (occ == 0).astype(np.uint8)
        self._submap_edt = cv2.distanceTransform(fg, cv2.DIST_L2, cv2.DIST_MASK_PRECISE) * np.float32(cs)
        self._submap_origin = (cx, cy)

    def _segment_angles_mod90(self, dists: np.ndarray, angs: np.ndarray) -> Optional[np.ndarray]:
        if dists.size < 16:
            return None
        w_ang = self.heading + angs
        hx = dists * np.cos(w_ang)
        hy = dists * np.sin(w_ang)
        span = 10
        dx_v = hx[span:] - hx[:-span]
        dy_v = hy[span:] - hy[:-span]
        ds2 = dx_v * dx_v + dy_v * dy_v
        mx = 0.5 * (hx[span:] + hx[:-span]) - hx[5:-5]
        my = 0.5 * (hy[span:] + hy[:-span]) - hy[5:-5]
        cont = (ds2 > 900.0) & (ds2 < 8100.0) & ((mx * mx + my * my) < 20.0)
        if np.count_nonzero(cont) < 8:
            return None
        return np.mod(np.arctan2(dy_v[cont], dx_v[cont]), math.pi * 0.5)

    def _accumulate_wall_orientations(self, dists: np.ndarray, angs: np.ndarray) -> None:
        seg_a = self._segment_angles_mod90(dists, angs)
        if seg_a is None:
            return
        bins = np.clip(np.floor(np.degrees(seg_a)).astype(np.int32), 0, 89)
        np.add.at(self._wall_hist, bins, 1.0)

    def _stabilize_heading_from_walls(self, dists: np.ndarray, angs: np.ndarray) -> None:
        tot = float(np.sum(self._wall_hist))
        if tot < 45.0:
            return
        pk = int(np.argmax(self._wall_hist))
        offs = np.arange(-5, 6, dtype=np.int32)
        nbr_idx = (pk + offs) % 90
        pk_mass = float(np.sum(self._wall_hist[nbr_idx]))
        if pk_mass < 0.65 * tot:
            return
        # Weighted circular centroid in [0, pi/2)
        w = self._wall_hist[nbr_idx]
        mean_off = float(np.sum(offs * w) / max(1e-6, pk_mass))
        phi_dom = math.radians((pk + mean_off + 0.5) % 90.0)

        seg_a = self._segment_angles_mod90(dists, angs)
        if seg_a is None:
            return
        half_pi = math.pi * 0.5
        qtr_pi = math.pi * 0.25
        resid = np.mod(seg_a - phi_dom + qtr_pi, half_pi) - qtr_pi
        inl = resid[np.abs(resid) < 0.38]
        if inl.size >= 6:
            med_err = float(np.median(inl))
            corr = max(-0.06, min(0.06, med_err))
            self.heading = normalize_angle(self.heading - 0.45 * corr)

    def _apply_wall_normal_step(
        self,
        bx: float,
        by: float,
        bth: float,
        d_sub: np.ndarray,
        a_sub: np.ndarray,
        max_step: float,
    ) -> None:
        """Apply constrained translation (wall-normal only in straight corridors) and heading."""
        dx = bx - self.x
        dy = by - self.y
        if d_sub.size >= 8:
            w_sub = bth + a_sub
            hx = bx + d_sub * np.cos(w_sub)
            hy = by + d_sub * np.sin(w_sub)
            dx_v = hx[3:] - hx[:-3]
            dy_v = hy[3:] - hy[:-3]
            ds2 = dx_v * dx_v + dy_v * dy_v
            cont = (ds2 > 400.0) & (ds2 < 6400.0)
            if np.count_nonzero(cont) >= 4:
                inv_ds = 1.0 / np.sqrt(ds2[cont])
                nx = -dy_v[cont] * inv_ds
                ny = dx_v[cont] * inv_ds
                cxx = float(np.mean(nx * nx))
                cyy = float(np.mean(ny * ny))
                cxy = float(np.mean(nx * ny))
                disc = math.sqrt(max(0.0, (cxx - cyy) ** 2 + 4.0 * cxy * cxy))
                lmin = 0.5 * ((cxx + cyy) - disc)
                if lmin < 0.12 and (abs(cxy) > 1e-6 or abs(cxx - cyy) > 1e-6):
                    theta_e = 0.5 * math.atan2(2.0 * cxy, cxx - cyy)
                    ex = math.cos(theta_e)
                    ey = math.sin(theta_e)
                    proj = dx * ex + dy * ey
                    dx = proj * ex
                    dy = proj * ey
        step_n = math.hypot(dx, dy)
        if step_n > max_step:
            scale = max_step / step_n
            dx *= scale
            dy *= scale
        self.x += dx
        self.y += dy
        self.heading = normalize_angle(bth)

    def _match_global_gps_map(
        self,
        dists: np.ndarray,
        angs: np.ndarray,
        online_map: OnlineMap,
    ) -> bool:
        edt = online_map.wall_edt_px
        gh, gw = online_map.gh, online_map.gw
        cs = float(online_map.CELL_SIZE)
        half_w = online_map.width * 0.5
        half_h = online_map.height * 0.5

        def sample_edt(hx: np.ndarray, hy: np.ndarray) -> np.ndarray:
            fx = (hx + half_w) / cs - 0.5
            fy = (half_h - hy) / cs - 0.5
            c0 = np.clip(np.floor(fx).astype(np.int32), 0, gw - 2)
            r0 = np.clip(np.floor(fy).astype(np.int32), 0, gh - 2)
            wx = np.clip(fx - c0, 0.0, 1.0)
            wy = np.clip(fy - r0, 0.0, 1.0)
            v00 = edt[r0, c0]
            v01 = edt[r0, c0 + 1]
            v10 = edt[r0 + 1, c0]
            v11 = edt[r0 + 1, c0 + 1]
            return (1.0 - wy) * ((1.0 - wx) * v00 + wx * v01) + wy * ((1.0 - wx) * v10 + wx * v11)

        base_ang = self.heading + angs
        base_d = sample_edt(self.x + dists * np.cos(base_ang), self.y + dists * np.sin(base_ang))
        near_mask = base_d < 15.0
        if np.count_nonzero(near_mask) < 14:
            return False

        d_sub = dists[near_mask]
        a_sub = angs[near_mask]
        bx, by, bth, mse = self._correlative_search(
            sample_edt, d_sub, a_sub, self.x, self.y, self.heading,
            coarse=((-4.5, 0.0, 4.5), (-4.5, 0.0, 4.5), (-0.014, 0.0, 0.014)),
            fine=((-1.5, 0.0, 1.5), (-1.5, 0.0, 1.5), (-0.005, 0.0, 0.005)),
            w_pos=0.35,
            w_rot=340.0,
            clip_px=8.0,
        )
        if mse < 9.0:
            self._apply_wall_normal_step(bx, by, bth, d_sub, a_sub, max_step=2.5)
            return True
        return False

    def _match_local_submap(self, dists: np.ndarray, angs: np.ndarray) -> None:
        edt = self._submap_edt
        if edt is None:
            return
        half_n = self._SUBMAP_HALF_CELLS
        size_n = half_n * 2
        cs = self._SUBMAP_CELL_PX
        ox, oy = self._submap_origin

        def sample_local(hx: np.ndarray, hy: np.ndarray) -> np.ndarray:
            fx = (hx - ox) / cs + half_n - 0.5
            fy = half_n - (hy - oy) / cs - 0.5
            c0 = np.clip(np.floor(fx).astype(np.int32), 0, size_n - 2)
            r0 = np.clip(np.floor(fy).astype(np.int32), 0, size_n - 2)
            wx = np.clip(fx - c0, 0.0, 1.0)
            wy = np.clip(fy - r0, 0.0, 1.0)
            v00 = edt[r0, c0]
            v01 = edt[r0, c0 + 1]
            v10 = edt[r0 + 1, c0]
            v11 = edt[r0 + 1, c0 + 1]
            return (1.0 - wy) * ((1.0 - wx) * v00 + wx * v01) + wy * ((1.0 - wx) * v10 + wx * v11)

        base_ang = self.heading + angs
        base_d = sample_local(self.x + dists * np.cos(base_ang), self.y + dists * np.sin(base_ang))
        near_mask = base_d < 9.5
        if np.count_nonzero(near_mask) < 16:
            return

        d_sub = dists[near_mask]
        a_sub = angs[near_mask]
        bx, by, bth, mse = self._correlative_search(
            sample_local, d_sub, a_sub, self.x, self.y, self.heading,
            coarse=((-2.0, 0.0, 2.0), (-2.0, 0.0, 2.0), (-0.012, 0.0, 0.012)),
            fine=((-0.8, 0.0, 0.8), (-0.8, 0.0, 0.8), (-0.004, 0.0, 0.004)),
            w_pos=0.55,
            w_rot=320.0,
            clip_px=5.5,
        )
        if mse < 5.0:
            self._apply_wall_normal_step(bx, by, bth, d_sub, a_sub, max_step=1.4)

    @staticmethod
    def _correlative_search(
        sampler,
        dists: np.ndarray,
        angs: np.ndarray,
        x0: float,
        y0: float,
        th0: float,
        coarse: Tuple[Tuple[float, ...], Tuple[float, ...], Tuple[float, ...]],
        fine: Tuple[Tuple[float, ...], Tuple[float, ...], Tuple[float, ...]],
        w_pos: float,
        w_rot: float,
        clip_px: float = 9.0,
    ) -> Tuple[float, float, float, float]:
        def eval_grid(cx: float, cy: float, cth: float, grid_tuple):
            dxs, dys, dths = grid_tuple
            gx = np.asarray([cx + dx for dx in dxs for _ in dys for _ in dths], dtype=np.float32)
            gy = np.asarray([cy + dy for _ in dxs for dy in dys for _ in dths], dtype=np.float32)
            gt = np.asarray([cth + dt for _ in dxs for _ in dys for dt in dths], dtype=np.float32)
            world_ang = gt[:, None] + angs[None, :]
            hx = gx[:, None] + dists[None, :] * np.cos(world_ang)
            hy = gy[:, None] + dists[None, :] * np.sin(world_ang)
            wall_d = np.minimum(sampler(hx, hy), clip_px)
            mean_d2 = np.mean(wall_d * wall_d, axis=1)
            prior = w_pos * ((gx - cx) ** 2 + (gy - cy) ** 2) + w_rot * ((gt - cth) ** 2)
            idx = int(np.argmin(mean_d2 + prior))
            return float(gx[idx]), float(gy[idx]), float(gt[idx]), float(mean_d2[idx])

        bx, by, bth, _ = eval_grid(x0, y0, th0, coarse)
        return eval_grid(bx, by, bth, fine)
