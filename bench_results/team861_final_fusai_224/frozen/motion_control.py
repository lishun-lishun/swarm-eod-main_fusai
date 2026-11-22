"""
Holonomic velocity-regulated path tracking, Lidar wall barrier, inter-drone
collision avoidance, and precision bomb grasping / disposal control.
"""

from __future__ import annotations

import math
from typing import Iterable, List, Optional, Tuple

import numpy as np

from swarm_rescue.simulation.drone.controller import CommandsDict
from swarm_rescue.simulation.ray_sensors.drone_semantic_sensor import DroneSemanticSensor

WorldPoint = Tuple[float, float]
_TWO_PI = 2.0 * math.pi


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else (hi if v > hi else v)


def _wrap_angle(a: float) -> float:
    return (a + math.pi) % _TWO_PI - math.pi


def empty_command(grasper: int = 0) -> CommandsDict:
    return {
        "forward": 0.0,
        "lateral": 0.0,
        "rotation": 0.0,
        "grasper": grasper,
    }


def compute_lidar_wall_barrier(
    lidar_vals: Optional[np.ndarray],
    lidar_angles: Optional[np.ndarray],
    heading: float,
    semantic_vals: Optional[list],
    barrier_radius: float,
    move_world_dir: Optional[Tuple[float, float]] = None,
    disposing_mode: bool = False,
) -> Tuple[float, float, List[Tuple[float, float, float]], float, float, float]:
    """
    Compute repulsive vector (rx, ry) in world frame from walls detected by
    Lidar, ignoring rays that correspond to semantic DRONE or BOMB detections
    (and DISPOSAL_CENTER detections when in disposing mode).
    Also returns:
      - close_walls: list of (ux, uy, dist) unit vectors toward close walls
      - front_min: minimum forward wall distance along move_world_dir
      - lat_width: lateral free corridor width (left min + right min)
      - min_wd: global minimum wall distance detected
    """
    if lidar_vals is None or lidar_angles is None:
        return (0.0, 0.0, [], float("inf"), float("inf"), float("inf"))

    dyn_dirs: List[Tuple[float, float]] = []
    dc_ang_disp: Optional[float] = None
    if semantic_vals:
        for obj in semantic_vals:
            if obj.entity_type in (
                DroneSemanticSensor.TypeEntity.DRONE,
                DroneSemanticSensor.TypeEntity.BOMB,
            ):
                ang = float(obj.angle)
                dist = float(obj.distance)
                if math.isfinite(ang) and math.isfinite(dist):
                    dyn_dirs.append((ang, dist))
            elif disposing_mode and obj.entity_type == DroneSemanticSensor.TypeEntity.DISPOSAL_CENTER:
                ang = float(obj.angle)
                if math.isfinite(ang):
                    dc_ang_disp = ang

    d_arr = np.asarray(lidar_vals[::2], dtype=np.float32)
    a_arr = np.asarray(lidar_angles[::2], dtype=np.float32)
    valid = np.isfinite(d_arr) & (d_arr > 1.0)
    if not np.any(valid):
        return (0.0, 0.0, [], float("inf"), float("inf"), float("inf"))
    d_arr = d_arr[valid]
    a_arr = a_arr[valid]
    if disposing_mode and dc_ang_disp is not None:
        diff_dc = np.abs(np.arctan2(np.sin(a_arr - dc_ang_disp), np.cos(a_arr - dc_ang_disp)))
        keep_dc = diff_dc >= 1.25
        d_arr = d_arr[keep_dc]
        a_arr = a_arr[keep_dc]

    if dyn_dirs:
        da = np.asarray([o[0] for o in dyn_dirs], dtype=np.float32)
        dd = np.asarray([o[1] for o in dyn_dirs], dtype=np.float32)
        diff = np.abs(np.arctan2(np.sin(a_arr[:, None] - da[None, :]), np.cos(a_arr[:, None] - da[None, :])))
        is_dyn = np.any((diff < 0.28) & (np.abs(d_arr[:, None] - dd[None, :]) < 38.0), axis=1)
        keep = ~is_dyn
        if not np.any(keep):
            return (0.0, 0.0, [], float("inf"), float("inf"), float("inf"))
        d_arr = d_arr[keep]
        a_arr = a_arr[keep]

    w_ang = np.float32(heading) + a_arr
    front_min = float("inf")
    if move_world_dir is not None:
        move_ang = math.atan2(move_world_dir[1], move_world_dir[0])
        near_front = d_arr < 118.0
        if np.any(near_front):
            d_f = d_arr[near_front]
            w_f = w_ang[near_front]
            da_m = np.abs(np.arctan2(np.sin(w_f - move_ang), np.cos(w_f - move_ang)))
            lat_dist = d_f * np.sin(da_m)
            f_hit = (da_m < 0.60) & ((lat_dist < 12.0) | (da_m < 0.16))
            if np.any(f_hit):
                front_min = float(np.min(d_f[f_hit]))

    # Lateral corridor width strictly perpendicular to direction of motion:
    lat_width = float("inf")
    if move_world_dir is not None and d_arr.size > 0:
        move_ang = math.atan2(move_world_dir[1], move_world_dir[0])
        diff_l = np.abs(np.arctan2(np.sin(w_ang - (move_ang + math.pi / 2.0)), np.cos(w_ang - (move_ang + math.pi / 2.0))))
        diff_r = np.abs(np.arctan2(np.sin(w_ang - (move_ang - math.pi / 2.0)), np.cos(w_ang - (move_ang - math.pi / 2.0))))
        near_l = d_arr[diff_l < 0.40]
        near_r = d_arr[diff_r < 0.40]
        if near_l.size > 0 and near_r.size > 0:
            lat_width = float(np.min(near_l) + np.min(near_r))
    elif d_arr.size > 0:
        diff_l = np.abs(np.arctan2(np.sin(a_arr - math.pi / 2.0), np.cos(a_arr - math.pi / 2.0)))
        diff_r = np.abs(np.arctan2(np.sin(a_arr + math.pi / 2.0), np.cos(a_arr + math.pi / 2.0)))
        near_l = d_arr[diff_l < 0.40]
        near_r = d_arr[diff_r < 0.40]
        if near_l.size > 0 and near_r.size > 0:
            lat_width = float(np.min(near_l) + np.min(near_r))
    min_wd = float(np.min(d_arr)) if d_arr.size > 0 else float("inf")

    close_mask = d_arr < barrier_radius
    if not np.any(close_mask):
        return (0.0, 0.0, [], front_min, lat_width, min_wd)

    dc = d_arr[close_mask]
    wc = w_ang[close_mask]
    ux_c = np.cos(wc)
    uy_c = np.sin(wc)
    close_walls: List[Tuple[float, float, float]] = list(zip(ux_c.tolist(), uy_c.tolist(), dc.tolist()))

    denom = max(4.0, barrier_radius - 13.0)
    pen = np.maximum(0.0, (barrier_radius - dc) / denom)
    mag = pen * pen
    rx = float(-np.sum(ux_c * mag))
    ry = float(-np.sum(uy_c * mag))

    norm = math.hypot(rx, ry)
    if norm > 1.0:
        rx /= norm
        ry /= norm
    return (rx, ry, close_walls, front_min, lat_width, min_wd)


def compute_drone_avoidance(
    my_id: int,
    my_pos: WorldPoint,
    my_heading: float,
    my_carrying: bool,
    my_priority_tier: int,
    my_vel: Tuple[float, float],
    semantic_vals: Optional[list],
    peer_states: Iterable[Tuple[int, float, float, float, float, bool, int]],
    sep_radius: float = 64.0,
    is_corridor: bool = False,
    move_dir: Optional[Tuple[float, float]] = None,
) -> Tuple[float, float, float]:
    """
    Compute velocity-aware predictive inter-drone repulsive + tangential
    avoidance velocity (ax, ay) in world coordinates and a speed-scaling
    factor in [0.0, 1.0].

    In narrow corridors (is_corridor=True), lateral swirl into walls is suppressed;
    non-priority drones actively halt forward motion and back up along the corridor axis.
    """
    neighbors: List[Tuple[float, float, float, float, Optional[int], bool, int]] = []

    # 1. Communicated peers (accurate world coords + velocity up to 180 px)
    for pid, px, py, pvx, pvy, p_carry, p_tier in peer_states:
        if pid == my_id:
            continue
        dx = px - my_pos[0]
        dy = py - my_pos[1]
        d = math.hypot(dx, dy)
        if 1.0 < d < 110.0:
            neighbors.append((px, py, pvx, pvy, pid, p_carry, p_tier))

    # 2. Semantic DRONE detections (works even in NO_COM_ZONE, range 130 px)
    if semantic_vals:
        for obj in semantic_vals:
            if obj.entity_type != DroneSemanticSensor.TypeEntity.DRONE:
                continue
            d = float(obj.distance)
            ang = float(obj.angle)
            if not (math.isfinite(d) and math.isfinite(ang)) or d > 115.0:
                continue
            sx = my_pos[0] + d * math.cos(my_heading + ang)
            sy = my_pos[1] + d * math.sin(my_heading + ang)
            matched = False
            for nx, ny, *_ in neighbors:
                if math.hypot(nx - sx, ny - sy) < 24.0:
                    matched = True
                    break
            if not matched:
                neighbors.append((sx, sy, 0.0, 0.0, None, False, 1))

    ax = 0.0
    ay = 0.0
    speed_scale = 1.0

    for nx, ny, pvx, pvy, pid, p_carry, p_tier in neighbors:
        rx = my_pos[0] - nx
        ry = my_pos[1] - ny
        d = math.hypot(rx, ry)
        if d < 0.5:
            continue

        ux = rx / d  # unit vector pointing FROM peer TO me
        uy = ry / d
        tx = uy      # right-hand tangential swirl
        ty = -ux

        # Is the peer ahead along our path of travel?
        is_ahead = False
        dot_fwd = 0.0
        if move_dir is not None:
            dot_fwd = -rx * move_dir[0] - ry * move_dir[1]
            is_ahead = dot_fwd > 2.0

        # Closing speed along line joining centers (positive = approaching)
        v_close = -(my_vel[0] - pvx) * ux - (my_vel[1] - pvy) * uy

        # Check if peer is moving in roughly the same direction (convoy mode)
        peer_spd = math.hypot(pvx, pvy)
        same_dir = False
        if move_dir is not None and peer_spd > 1.0:
            dot_pdir = (pvx * move_dir[0] + pvy * move_dir[1]) / peer_spd
            same_dir = dot_pdir > 0.35

        # Determine relative priority (spatial right-of-way ONLY for same-direction convoy;
        # oncoming/crossing MUST use strict pid/tiebreaker to avoid mutual yielding deadlock)
        i_have_prio = False
        if my_priority_tier != p_tier:
            i_have_prio = my_priority_tier > p_tier
        elif my_carrying != p_carry:
            i_have_prio = my_carrying
        elif same_dir and move_dir is not None and abs(dot_fwd) > 6.0:
            # Spatial right-of-way in convoy: the drone ahead has priority; trailing drone yields and follows
            i_have_prio = dot_fwd < 0.0
        elif pid is not None:
            i_have_prio = my_id < pid
        else:
            # Deterministic geometric tie-breaker when pid unknown (semantic detection)
            dx_to_peer = -rx
            dy_to_peer = -ry
            if abs(dx_to_peer) > 1.0:
                i_have_prio = dx_to_peer > 0.0
            else:
                i_have_prio = dy_to_peer > 0.0

        if is_corridor:
            # ---------------- CORRIDOR MODE (NO ROOM TO PASS SIDE-BY-SIDE) ----------------
            # Suppress tangential swirl into side walls
            if same_dir:
                # Convoy following: both drones moving in the same direction down corridor
                c_carry = my_carrying or p_carry
                d_fl = 42.0 if c_carry else 33.0
                d_br = 50.0 if c_carry else 38.0
                if i_have_prio:
                    # Lead drone: unobstructed ahead down corridor! Maintain full cruise, never stall for trailing drones
                    pass
                else:
                    # Trailing drone: follow safely behind lead drone without stalling or bumping bomb
                    if d < d_fl:
                        speed_scale = 0.0
                        ax += 3.6 * ux
                        ay += 3.6 * uy
                    elif d < d_br:
                        speed_scale = min(speed_scale, max(0.18, (d - d_fl) / max(1.0, d_br - d_fl)))
                        ax += 2.0 * ux
                        ay += 2.0 * uy
                    elif v_close > 0.15 and d < (d_br + 12.0):
                        speed_scale = min(speed_scale, max(0.35, (d - d_fl) / max(1.0, d_br + 12.0 - d_fl)))
            else:
                # Oncoming or peer stationary: strict yield & reverse
                if i_have_prio:
                    # Priority drone: hold corridor centerline, smooth deceleration when close
                    if d < 40.0:
                        speed_scale = min(speed_scale, max(0.18, (d - 32.0) / 16.0))
                    if d < 33.0:
                        speed_scale = 0.0
                    if d < 36.0:
                        ax += 3.6 * ux
                        ay += 3.6 * uy
                else:
                    # Yielding drone: MUST STOP forward motion and REVERSE away to let priority drone pass
                    if (is_ahead or v_close > 0.15) and d < 56.0:
                        speed_scale = 0.0
                        if d < 48.0:
                            rev_urgency = (48.0 - d) / 20.0
                            rev_mag = 4.8 * _clamp(rev_urgency, 0.4, 1.4)
                            ax += rev_mag * ux
                            ay += rev_mag * uy
                    elif (is_ahead or v_close > 0.4) and d < 78.0:
                        speed_scale = min(speed_scale, max(0.20, (d - 44.0) / 34.0))
                    elif d < 38.0:
                        speed_scale = 0.0
                        ax += 4.0 * ux
                        ay += 4.0 * uy
        else:
            # ---------------- OPEN SPACE MODE (SWIRL & RIGHT-HAND PASSING) ----------------
            # Physical drone contact radius is 16.5 + 16.5 = 33.0 px
            yield_w = 0.45 if i_have_prio else 1.55
            if v_close > 0.5 and d < 95.0:
                ttc = max(0.4, (d - 33.0) / v_close)
                if ttc < 7.5:
                    urgency = (7.5 - ttc) / 7.5
                    dodge = yield_w * 4.8 * urgency
                    ax += dodge * (0.45 * ux + 0.89 * tx)
                    ay += dodge * (0.45 * uy + 0.89 * ty)
                    if not i_have_prio:
                        speed_scale = min(speed_scale, _clamp((d - 34.0) / (v_close * 5.0 + 1e-3), 0.0, 1.0))
                    else:
                        speed_scale = min(speed_scale, _clamp((d - 32.0) / (v_close * 3.5 + 1e-3), 0.35, 1.0))

            eff_sep = 58.0 if my_carrying else sep_radius
            stop_d = 48.0 if my_carrying else 52.0
            if d < eff_sep:
                frac = (eff_sep - d) / max(8.0, eff_sep - 30.0)
                mag = yield_w * 5.8 * (frac ** 1.3)
                ax += mag * (0.82 * ux + 0.58 * tx)
                ay += mag * (0.82 * uy + 0.58 * ty)
                if not i_have_prio and d < stop_d:
                    speed_scale = min(speed_scale, max(0.0, (d - 36.0) / max(10.0, stop_d - 36.0)))
                elif i_have_prio and d < 40.0:
                    speed_scale = min(speed_scale, max(0.20, (d - 34.0) / 14.0))
                elif (not my_carrying) and (not p_carry) and d < 65.0:
                    speed_scale = min(speed_scale, max(0.35, (d - 36.0) / 31.0))

                if (not i_have_prio) and is_ahead and d < 55.0:
                    speed_scale = min(speed_scale, max(0.0, (d - 35.0) / 20.0))
                    if d < 36.0:
                        speed_scale = 0.0
                        ax += 3.5 * ux
                        ay += 3.5 * uy

        # Absolute universal safety floor: drone physical hull contact is at d = 33.0 px
        if (is_ahead or v_close > 0.05) and d < 35.0:
            speed_scale = 0.0
        if d < 33.0:
            speed_scale = 0.0
            ax += 4.5 * ux
            ay += 4.5 * uy

    return (ax, ay, speed_scale)


def drive_to_target(
    *,
    pos: WorldPoint,
    heading: float,
    vel: Tuple[float, float],
    target: WorldPoint,
    desired_speed: float,
    max_speed: float,
    carrying: bool,
    desired_heading: Optional[float],
    lidar_vals: Optional[np.ndarray],
    lidar_angles: Optional[np.ndarray],
    semantic_vals: Optional[list],
    my_id: int,
    my_priority_tier: int,
    peer_states: Iterable[Tuple[int, float, float, float, float, bool, int]],
    grasping_mode: bool = False,
    disposing_mode: bool = False,
    grasper: int = 0,
) -> CommandsDict:
    """
    Compute holonomic thrust (forward, lateral) and rotation command with
    velocity damping, Lidar wall barrier, and corridor-aware inter-drone avoidance.
    """
    dx = target[0] - pos[0]
    dy = target[1] - pos[1]
    dist = math.hypot(dx, dy)

    if dist > 1e-3:
        ux = dx / dist
        uy = dy / dist
    else:
        ux = 0.0
        uy = 0.0

    # 1. Lidar wall safety barrier & corridor width estimation
    barrier_r = 19.5 if grasping_mode else (22.0 if disposing_mode else (29.5 if carrying else 26.0))
    vel_n = math.hypot(vel[0], vel[1])
    check_dir = (vel[0] / vel_n, vel[1] / vel_n) if vel_n > 2.5 else ((ux, uy) if dist > 2.0 else None)
    wx_rep, wy_rep, close_walls, front_min, lat_width, min_wd = compute_lidar_wall_barrier(
        lidar_vals=lidar_vals,
        lidar_angles=lidar_angles,
        heading=heading,
        semantic_vals=semantic_vals,
        barrier_radius=barrier_r,
        move_world_dir=check_dir,
        disposing_mode=disposing_mode,
    )

    is_corridor = (lat_width < 68.0) or (min_wd < 24.0 and len(close_walls) >= 2)
    move_dir = (ux, uy) if dist > 1.0 else None

    # 2. Corridor-aware predictive inter-drone avoidance
    sep_r = 48.0 if (grasping_mode or disposing_mode) else 64.0
    av_x, av_y, speed_scale = compute_drone_avoidance(
        my_id=my_id,
        my_pos=pos,
        my_heading=heading,
        my_carrying=carrying,
        my_priority_tier=my_priority_tier,
        my_vel=vel,
        semantic_vals=semantic_vals,
        peer_states=peer_states,
        sep_radius=sep_r,
        is_corridor=is_corridor,
        move_dir=move_dir,
    )

    v_mag = min(max_speed, desired_speed * speed_scale)
    vx_des = ux * v_mag + av_x
    vy_des = uy * v_mag + av_y

    # 4. Forward braking limit along actual motion/target
    front_lim = 68.0 if carrying else 112.0
    if (not grasping_mode) and (not disposing_mode) and front_min < front_lim:
        brake_a = 0.50 if carrying else 0.35
        min_front_safe = 17.5 if carrying else 16.5
        v_wall_cap = max(2.6 if carrying else 2.2, math.sqrt(2.0 * brake_a * max(0.0, front_min - min_front_safe)))
        cur_n = math.hypot(vx_des, vy_des)
        if cur_n > v_wall_cap and cur_n > 1e-3:
            scale = v_wall_cap / cur_n
            vx_des *= scale
            vy_des *= scale

    if (not grasping_mode) and (not disposing_mode) and close_walls:
        min_wd_close = min(wd for _, _, wd in close_walls)
        thresh = 26.5 if carrying else 24.5
        min_safe = 18.2 if carrying else 17.4
        if min_wd_close < thresh:
            if front_min >= 35.0:
                base_floor = 11.6 if carrying else 12.0
                floor_slow = 6.5 if carrying else 6.0
            else:
                base_floor = 4.2 if carrying else 5.5
                floor_slow = 2.6 if carrying else 3.0
            v_side_cap = max(base_floor if min_wd_close >= min_safe else floor_slow, (min_wd_close - min_safe) * 0.72)
            cur_n = math.hypot(vx_des, vy_des)
            if cur_n > v_side_cap and cur_n > 1e-3:
                scale = v_side_cap / cur_n
                vx_des *= scale
                vy_des *= scale

    # 5. Control Barrier Function (CBF) wall normal velocity clamp
    hard_wall_r = 16.6 if grasping_mode else (17.2 if disposing_mode else (18.2 if carrying else 17.4))
    use_cbf = (not carrying) or disposing_mode
    cbf_outer_r = (20.0 if (grasping_mode or disposing_mode) else 23.0) if use_cbf else hard_wall_r
    for wux, wuy, wd in close_walls:
        if disposing_mode and move_dir is not None:
            if (wux * move_dir[0] + wuy * move_dir[1]) > 0.15:
                continue
        if wd < cbf_outer_r:
            inward = vx_des * wux + vy_des * wuy
            v_in_max = 0.0 if (not use_cbf or wd <= hard_wall_r) else (wd - hard_wall_r) * 1.20
            if inward > v_in_max:
                vx_des -= (inward - v_in_max) * wux
                vy_des -= (inward - v_in_max) * wuy

    # Ensure wall repulsion doesn't push backwards against desired motion
    if move_dir is not None:
        rep_dot = wx_rep * move_dir[0] + wy_rep * move_dir[1]
        if rep_dot < 0.0:
            wx_rep -= rep_dot * move_dir[0]
            wy_rep -= rep_dot * move_dir[1]

    rep_gain = 0.5 if disposing_mode else (2.2 if grasping_mode else (4.5 if carrying else 6.0))
    vx_des += wx_rep * rep_gain
    vy_des += wy_rep * rep_gain

    # Clamp desired velocity magnitude
    v_des_norm = math.hypot(vx_des, vy_des)
    if v_des_norm > max_speed:
        vx_des *= max_speed / v_des_norm
        vy_des *= max_speed / v_des_norm

    # 5. Velocity feedback -> world thrust vector
    kp_v = 1.65 if carrying else 0.90
    ff_v = 0.285 if carrying else 0.088
    ax_w = kp_v * (vx_des - vel[0]) + ff_v * vx_des
    ay_w = kp_v * (vy_des - vel[1]) + ff_v * vy_des

    # Transform world force to drone body frame (forward, lateral)
    c = math.cos(heading)
    s = math.sin(heading)
    fwd = ax_w * c + ay_w * s
    lat = -ax_w * s + ay_w * c

    thrust_norm = math.hypot(fwd, lat)
    if thrust_norm > 1.0:
        fwd /= thrust_norm
        lat /= thrust_norm

    # 4. Heading / rotation control
    if desired_heading is None:
        if math.hypot(vx_des, vy_des) > 0.8:
            desired_heading = math.atan2(vy_des, vx_des)
        else:
            desired_heading = heading

    ang_err = _wrap_angle(desired_heading - heading)
    if grasping_mode:
        rot = _clamp(1.8 * ang_err, -0.9, 0.9)
    elif carrying:
        rot = _clamp(0.75 * ang_err, -0.4, 0.4)
    else:
        rot = _clamp(1.35 * ang_err, -0.8, 0.8)

    return {
        "forward": float(_clamp(fwd, -1.0, 1.0)),
        "lateral": float(_clamp(lat, -1.0, 1.0)),
        "rotation": float(rot),
        "grasper": int(grasper),
    }
