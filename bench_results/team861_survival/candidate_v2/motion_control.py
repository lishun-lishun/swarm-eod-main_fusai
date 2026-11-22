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
) -> Tuple[float, float, List[Tuple[float, float, float]], float]:
    """
    Compute repulsive vector (rx, ry) in world frame from walls detected by
    Lidar, ignoring rays that correspond to semantic DRONE or BOMB detections.
    Also returns list of (ux, uy, dist) unit vectors toward close walls and
    minimum forward wall distance along move_world_dir (within +/-28 deg).
    """
    if lidar_vals is None or lidar_angles is None:
        return (0.0, 0.0, [], float("inf"))

    dyn_dirs: List[Tuple[float, float]] = []
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

    d_arr = np.asarray(lidar_vals[::2], dtype=np.float32)
    a_arr = np.asarray(lidar_angles[::2], dtype=np.float32)
    valid = np.isfinite(d_arr) & (d_arr > 1.0)
    if not np.any(valid):
        return (0.0, 0.0, [], float("inf"))
    d_arr = d_arr[valid]
    a_arr = a_arr[valid]

    if dyn_dirs:
        da = np.asarray([o[0] for o in dyn_dirs], dtype=np.float32)
        dd = np.asarray([o[1] for o in dyn_dirs], dtype=np.float32)
        diff = np.abs(np.arctan2(np.sin(a_arr[:, None] - da[None, :]), np.cos(a_arr[:, None] - da[None, :])))
        is_dyn = np.any((diff < 0.22) & (np.abs(d_arr[:, None] - dd[None, :]) < 28.0), axis=1)
        keep = ~is_dyn
        if not np.any(keep):
            return (0.0, 0.0, [], float("inf"))
        d_arr = d_arr[keep]
        a_arr = a_arr[keep]

    w_ang = np.float32(heading) + a_arr
    front_min = float("inf")
    if move_world_dir is not None:
        move_ang = math.atan2(move_world_dir[1], move_world_dir[0])
        near84 = d_arr < 84.0
        if np.any(near84):
            d_f = d_arr[near84]
            w_f = w_ang[near84]
            da_m = np.abs(np.arctan2(np.sin(w_f - move_ang), np.cos(w_f - move_ang)))
            f_hit = (da_m < 0.72) & ((d_f * np.sin(da_m) < 15.2) | (da_m < 0.22))
            if np.any(f_hit):
                front_min = float(np.min(d_f[f_hit]))

    close_mask = d_arr < barrier_radius
    if not np.any(close_mask):
        return (0.0, 0.0, [], front_min)

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
    return (rx, ry, close_walls, front_min)


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
) -> Tuple[float, float, float]:
    """
    Compute velocity-aware predictive inter-drone repulsive + tangential
    avoidance velocity (ax, ay) in world coordinates and a speed-scaling
    factor in [0.15, 1.0].

    peer_states yields (peer_id, px, py, pvx, pvy, peer_carrying, peer_priority_tier).
    """
    neighbors: List[Tuple[float, float, float, float, Optional[int], bool, int]] = []

    # 1. Communicated peers (accurate world coords + velocity up to 180 px)
    for pid, px, py, pvx, pvy, p_carry, p_tier in peer_states:
        if pid == my_id:
            continue
        dx = px - my_pos[0]
        dy = py - my_pos[1]
        d = math.hypot(dx, dy)
        if 1.0 < d < 105.0:
            neighbors.append((px, py, pvx, pvy, pid, p_carry, p_tier))

    # 2. Semantic DRONE detections (works even in NO_COM_ZONE, range 130 px)
    if semantic_vals:
        for obj in semantic_vals:
            if obj.entity_type != DroneSemanticSensor.TypeEntity.DRONE:
                continue
            d = float(obj.distance)
            ang = float(obj.angle)
            if not (math.isfinite(d) and math.isfinite(ang)) or d > 112.0:
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

        ux = rx / d
        uy = ry / d
        # Right-hand tangential swirl
        tx = uy
        ty = -ux

        # Determine relative priority
        i_have_prio = False
        if my_carrying != p_carry:
            i_have_prio = my_carrying
        elif my_priority_tier != p_tier:
            i_have_prio = my_priority_tier > p_tier
        elif pid is not None:
            i_have_prio = my_id < pid

        yield_w = 0.45 if i_have_prio else 1.55

        # Closing speed along line joining centers (positive = approaching)
        v_close = -(my_vel[0] - pvx) * ux - (my_vel[1] - pvy) * uy
        if v_close > 0.8 and d < 100.0:
            ttc = max(0.4, (d - 28.0) / v_close)
            if ttc < 7.0:
                urgency = (7.0 - ttc) / 7.0
                dodge = yield_w * 4.8 * urgency
                ax += dodge * (0.45 * ux + 0.89 * tx)
                ay += dodge * (0.45 * uy + 0.89 * ty)
                if not i_have_prio:
                    speed_scale = min(speed_scale, _clamp((d - 30.0) / (v_close * 5.2 + 1e-3), 0.18, 1.0))

        # Near-field repulsion barrier (tighter envelope when carrying in 100-px corridors)
        eff_sep = 52.0 if my_carrying else sep_radius
        stop_d = 40.0 if my_carrying else 46.0
        if d < eff_sep:
            frac = (eff_sep - d) / max(8.0, eff_sep - 22.0)
            mag = yield_w * 5.6 * (frac ** 1.4)
            ax += mag * (0.78 * ux + 0.62 * tx)
            ay += mag * (0.78 * uy + 0.62 * ty)
            if not i_have_prio and d < stop_d:
                speed_scale = min(speed_scale, max(0.20, (d - 24.0) / max(12.0, stop_d - 24.0)))
            elif (not my_carrying) and (not p_carry) and d < 58.0:
                speed_scale = min(speed_scale, max(0.55, (d - 24.0) / 34.0))

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
    cautious: bool = False,
    outage_points: Iterable[WorldPoint] = (),
) -> CommandsDict:
    """
    Compute holonomic thrust (forward, lateral) and rotation command with
    velocity damping, Lidar wall barrier, and inter-drone avoidance.
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

    # 1. Predictive inter-drone avoidance
    sep_r = 46.0 if (grasping_mode or disposing_mode) else 64.0
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
    )

    v_mag = min(max_speed, desired_speed * speed_scale)
    if cautious and not (grasping_mode or disposing_mode):
        v_mag = min(v_mag, 3.0 if carrying else 5.8)
    vx_des = ux * v_mag + av_x
    vy_des = uy * v_mag + av_y

    # 2. Lidar wall safety barrier + forward braking limit (along actual motion/target)
    barrier_r = 16.0 if (grasping_mode or disposing_mode) else (20.5 if carrying else 26.0)
    vel_n = math.hypot(vel[0], vel[1])
    check_dir = (vel[0] / vel_n, vel[1] / vel_n) if vel_n > 2.5 else ((ux, uy) if dist > 2.0 else None)
    wx_rep, wy_rep, close_walls, front_min = compute_lidar_wall_barrier(
        lidar_vals=lidar_vals,
        lidar_angles=lidar_angles,
        heading=heading,
        semantic_vals=semantic_vals,
        barrier_radius=barrier_r,
        move_world_dir=check_dir,
    )

    front_lim = 58.0 if carrying else 112.0
    if (not grasping_mode) and (not disposing_mode) and front_min < front_lim:
        brake_a = 0.25 if carrying else 0.28
        v_wall_cap = max(2.2, math.sqrt(2.0 * brake_a * max(0.0, front_min - 16.5)))
        cur_n = math.hypot(vx_des, vy_des)
        if cur_n > v_wall_cap and cur_n > 1e-3:
            vx_des *= v_wall_cap / cur_n
            vy_des *= v_wall_cap / cur_n

    if (not carrying) and (not grasping_mode) and (not disposing_mode) and close_walls:
        min_wd = min(wd for _, _, wd in close_walls)
        if min_wd < 24.5:
            v_side_cap = max(3.8, (min_wd - 14.0) * 0.68)
            cur_n = math.hypot(vx_des, vy_des)
            if cur_n > v_side_cap and cur_n > 1e-3:
                vx_des *= v_side_cap / cur_n
                vy_des *= v_side_cap / cur_n

    # Project out velocity component pushing straight into a very close wall
    hard_wall_r = 14.2 if (grasping_mode or disposing_mode) else 16.0
    for wux, wuy, wd in close_walls:
        if wd < hard_wall_r:
            inward = vx_des * wux + vy_des * wuy
            if inward > 0.0:
                vx_des -= inward * wux
                vy_des -= inward * wuy

    rep_gain = 3.0 if (grasping_mode or disposing_mode) else (3.2 if carrying else 7.2)
    vx_des += wx_rep * rep_gain
    vy_des += wy_rep * rep_gain

    # Project the combined command, including repulsion, away from near contacts.
    # Semantic ranges are measured to the visible surface of a peer.
    for obj in (semantic_vals or ()) if cautious or outage_points else ():
        if obj.entity_type != DroneSemanticSensor.TypeEntity.DRONE:
            continue
        d, angle = float(obj.distance), float(obj.angle)
        if not (0.0 < d < 34.0 and math.isfinite(angle)):
            continue
        nx, ny = math.cos(heading + angle), math.sin(heading + angle)
        limit = max(-1.6, (d - (18.0 if cautious else 14.0)) * 0.27)
        excess = vx_des * nx + vy_des * ny - limit
        if excess > 0.0:
            vx_des -= excess * nx
            vy_des -= excess * ny

    for point in outage_points:
        dx, dy = point[0] - pos[0], point[1] - pos[1]
        d = math.hypot(dx, dy)
        if 0.1 < d < 96.0:
            nx, ny = dx / d, dy / d
            limit = max(-2.0, (d - 40.0) * 0.17)
            excess = vx_des * nx + vy_des * ny - limit
            if excess > 0.0:
                vx_des -= excess * nx
                vy_des -= excess * ny

    for nx, ny, distance in close_walls if cautious or outage_points else ():
        if distance < hard_wall_r + (2.0 if cautious else 0.0):
            inward = vx_des * nx + vy_des * ny
            if inward > 0.0:
                vx_des -= inward * nx
                vy_des -= inward * ny

    # Clamp desired velocity magnitude
    v_des_norm = math.hypot(vx_des, vy_des)
    if v_des_norm > max_speed:
        vx_des *= max_speed / v_des_norm
        vy_des *= max_speed / v_des_norm

    # 3. Velocity feedback -> world thrust vector
    kp_v = 1.65 if carrying else 0.90
    ff_v = 0.262 if carrying else 0.088
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
