"""V6 distributed blue-team rescue controller, measured sensors only.

Every drone stores ONLY its own observations and received radio messages.
The public maps' *disposal center* locations are optional priors. Bomb locations
are NEVER hardcoded and are discovered through semantic sensing.
"""
from __future__ import annotations

from collections import deque
import math
from typing import Any, Dict, Optional, Tuple

from swarm_rescue.simulation.drone.controller import CommandsDict
from swarm_rescue.simulation.drone.drone_abstract import DroneAbstract
from swarm_rescue.simulation.ray_sensors.drone_semantic_sensor import DroneSemanticSensor
from swarm_rescue.solutions.blue_nav import (
    CenterDockController, LocalPilot, Occupancy, PoseTracker, clamp, distance, finite_pair, wrap,
)

Vec = Tuple[float, float]

# Public map definitions: (size, nominal starting center, disposal center, half-sizes).
# ONLY public spawn/disposal-center definitions are used, never bomb/wall geometry.
CENTER_PRIORS = (
    ((1250, 800), (-505, 98), (-505, 302), (113, 91)),
    ((1200, 900), (14, -277), (13, -402), (84, 42)),
    ((1113, 750), (-20, 93), (-20, -47), (105, 55)),
    ((1660, 1122), (-232, -463), (13, -476), (123, 80)),
    ((1200, 1200), (65, -15), (-122, 54), (69, 102)),
    ((1700, 1100), (612, 36), (781, 21), (66, 113)),
    ((1660, 1122), (-520, -411), (-720, -428), (105, 127)),
    ((1660, 1122), (586, 445), (746, 429), (78, 126)),
    ((1700, 1100), (574, -286), (755, -424), (89, 119)),
)


def safe_command(forward=0.0, lateral=0.0, rotation=0.0, grasper=0) -> CommandsDict:
    return dict(forward=clamp(forward), lateral=clamp(lateral),
                rotation=clamp(rotation), grasper=int(bool(grasper)))


class Track:
    __slots__ = ('xy', 'last_seen', 'state', 'owner', 'claimed_at', 'confidence')

    def __init__(self, xy: Vec, tick: int):
        self.xy = xy
        self.last_seen = tick
        self.state = 'free'
        self.owner = -1
        self.claimed_at = -1
        self.confidence = 1.0


class MyDroneRescue(DroneAbstract):
    """Ten fully independent agents with explicit communication-based consensus."""

    def __init__(self, identifier=None, misc_data=None, **kwargs):
        super().__init__(identifier=identifier, misc_data=misc_data, **kwargs)
        self.did = int(self.identifier if self.identifier is not None else 0)
        width, height = self.size_area or (1700, 1100)
        self.grid = Occupancy(int(width), int(height))
        self.pose = PoseTracker()
        self.pilot = LocalPilot()
        self.dock_controller = CenterDockController(self.did)
        self.tracks: Dict[int, Track] = {}
        self.track_serial = 0
        self.peers: Dict[int, dict] = {}
        self.home: Optional[Vec] = None
        self.disposal: Optional[Vec] = None
        self.center_half = (85.0, 85.0)
        self.mode = 'EXPLORE'
        self.target: Optional[Vec] = None
        self.target_track: Optional[int] = None
        self.frontier_id: Optional[int] = None
        self.frontier_avoid: Dict[int, int] = {}
        self.path = []
        self.path_tick = -9999
        self.path_target: Optional[Vec] = None
        self.goal_tick = -9999
        self.was_carrying = False
        self.active_carry_track: Optional[int] = None
        self.prev_position: Optional[Vec] = None
        self.progress_tick = 0
        self.escape_left = 0
        self.escape_dir = 1 if self.did % 2 == 0 else -1
        self.latest_msg: Dict[str, Any] = {}
        self.last_tick = -1
        self.delivery_struggles = 0
        self._dock_stage = None
        self._dock_stage_tick = -1000
        self._last_known_heading = None
        self.progress_distance = float('inf')
        self.progress_goal = None
        self.pickup_started = -1
        self._last_mode = None
        self.last_trace_tick = -999
        self.last_route_failure = -999
        self.dock_sweep = 0
        self.dock_side = 1 if self.did % 2 else -1
        self._semantic_cache = []
        self.relative_center = False
        self.center_observations = 0
        self._dock_history = []
        self._dock_last_progress = -1
        self._dock_recovery_until = -1
        self._dock_phase_reported = -1
        self._health_last = None
        self._health_current = None
        self._health_loss = 0
        self._damage_times = deque(maxlen=20)
        self._failed_route_count = 0

    def _initial_public_anchor(self):
        """If global pose is unavailable, use an internally consistent local frame.

        An arbitrary global spawn prior plus an arbitrary heading=0 is *not*
        a valid world-frame estimate. Never use known global center coordinates
        to route in that unobservable frame.
        """
        if self.pose.xy is not None and self.pose.heading is not None:
            return
        self.pose.seed_relative()
        self.relative_center = True
        self.home = (0.0, 0.0)
        # Relative frame needs a larger workspace; public wall geometry is
        # never loaded. The cell grid remains built exclusively by LiDAR.
        w, h = self.size_area or (1700, 1100)
        self.grid = Occupancy(int(w*2.0), int(h*2.0))
        self.disposal = None

    def _select_center(self, xy: Vec):
        if self.relative_center:
            return
        if self.disposal is not None:
            return
        matches = [(distance(xy, spawn), center, hs)
                   for dims, spawn, center, hs in CENTER_PRIORS
                   if tuple(self.size_area or ()) == dims]
        if matches:
            score, center, half = min(matches, key=lambda it: it[0])
            if score < 250:
                self.disposal = center
                self.center_half = half

    def _receive(self, tick: int):
        if self.communicator_is_disabled():
            return
        for _, msg in self.communicator.received_messages:
            if not isinstance(msg, dict) or msg.get('v') != 1:
                continue
            try:
                did = int(msg['id'])
                age = tick - int(msg['tick'])
            except (ValueError, TypeError, KeyError):
                continue
            if did == self.did or not (0 <= did < 100) or not (-3 <= age <= 240):
                continue
            # Relative odometric frames of different drones are unrelated.
            # Do not merge positions/occupancy unless both are world aligned.
            if bool(msg.get('relative',False)) or self.relative_center:
                continue
            self.peers[did] = msg
            if self.disposal is None and not self.relative_center:
                p = finite_pair(msg.get('home'))
                if p is not None:
                    self.disposal = p
            self.grid.merge(msg.get('cells'))
            for entry in msg.get('bombs', [])[:20]:
                try:
                    x, y, state, owner, seen, claimed = entry
                    xy = (float(x), float(y))
                    seen, claimed, owner = int(seen), int(claimed), int(owner)
                    if not (math.isfinite(xy[0]) and math.isfinite(xy[1])):
                        continue
                except (TypeError, ValueError):
                    continue
                if tick - seen > 750:
                    continue
                key = self._nearest_track(xy, radius=22.0)
                if key is None:
                    key = self._new_track(xy, seen)
                track = self.tracks[key]
                if state == 'done':
                    track.state, track.owner = 'done', owner
                    track.last_seen = max(track.last_seen, seen)
                elif track.state != 'done':
                    # Physical pickup dominates simultaneous claims. Claims use
                    # deterministic owner ID to avoid a last-message-wins race.
                    incoming = state if state in ('free','claimed','carried') else 'free'
                    accept = incoming == 'carried' or (incoming == 'claimed' and
                        track.state != 'carried' and (track.owner < 0 or
                        owner < track.owner or tick-track.claimed_at > 180))
                    if accept:
                        track.state,track.owner,track.claimed_at=incoming,owner,claimed
                        if incoming=='free':track.xy=xy
                track.last_seen = max(track.last_seen, seen)
        self.peers = {k: p for k, p in self.peers.items()
                      if tick - int(p.get('tick', -99999)) < 150}

    def _new_track(self, xy: Vec, tick: int) -> int:
        key = self.track_serial
        self.track_serial += 1
        self.tracks[key] = Track(xy, tick)
        return key

    def _nearest_track(self, xy: Vec, radius: float = 22.0) -> Optional[int]:
        choices = [(distance(t.xy, xy), k) for k, t in self.tracks.items()
                   if distance(t.xy, xy) < radius]
        return min(choices)[1] if choices else None

    def _observe(self, tick: int, xy: Vec, heading: float):
        visible = []
        detections = self._semantic_cache
        for det in detections:
            try:
                dist = float(det.distance)
                ang = float(det.angle)
                if not math.isfinite(dist) or not math.isfinite(ang):
                    continue
            except (TypeError, ValueError):
                continue
            kind = det.entity_type
            world = (xy[0] + dist * math.cos(heading + ang),
                     xy[1] + dist * math.sin(heading + ang))
            if kind == DroneSemanticSensor.TypeEntity.DISPOSAL_CENTER:
                # The semantic sensor detects the VISIBLE SURFACE, not the center.
                # If public map prior exists, keep it, else use surface as an approach target.
                if self.relative_center and dist < 129:
                    # Surface observation in the SAME odometric frame as pose.
                    # Keep the earliest stable landmark and refine it locally;
                    # do not mix radio coordinates from other local frames.
                    if self.disposal is None:
                        self.disposal = world
                        self.center_observations = 1
                    elif distance(self.disposal, world)<65:
                        alpha=.12 if self.center_observations<20 else .03
                        self.disposal = ((1-alpha)*self.disposal[0]+alpha*world[0],
                                         (1-alpha)*self.disposal[1]+alpha*world[1])
                        self.center_observations += 1
                elif self.disposal is None and dist < 125:
                    self.disposal = world
            elif kind == DroneSemanticSensor.TypeEntity.BOMB:
                key = self._nearest_track(world)
                if key is None:
                    key = self._new_track(world, tick)
                track = self.tracks[key]
                # Noisy GPS and moving bomb: fuse, but never resurrect a disposed bomb.
                if track.state != 'done':
                    track.xy = (0.72 * track.xy[0] + 0.28 * world[0],
                                0.72 * track.xy[1] + 0.28 * world[1])
                    track.last_seen = tick
                    if bool(det.grasped):
                        track.state = 'carried'
                    elif track.state == 'carried':
                        # Direct evidence of an ungrasped bomb overrides a stale peer report.
                        track.state, track.owner = 'free', -1
                    if not bool(det.grasped):
                        visible.append((dist, key, det))
        visible.sort(key=lambda item: item[0])
        return visible

    def _is_claimed_by_peer(self, key: int, tick: int, xy: Vec) -> bool:
        tr = self.tracks[key]
        if tr.state == 'done' or tr.state == 'carried':
            return True
        if tr.owner not in (-1, self.did) and tick - tr.claimed_at < 160:
            peer = self.peers.get(tr.owner)
            if peer is not None and not peer.get('carrying'):
                peer_xy = finite_pair(peer.get('pos'))
                if peer_xy is not None and distance(peer_xy, tr.xy) + 12 < distance(xy, tr.xy):
                    return True
        for pid, peer in self.peers.items():
            if peer.get('mode') != 'CHASE' or peer.get('carrying'):
                continue
            dest = finite_pair(peer.get('goal'))
            other = finite_pair(peer.get('pos'))
            if dest is None or other is None or distance(dest, tr.xy) > 22:
                continue
            own_cost = distance(xy, tr.xy)
            theirs = distance(other, tr.xy)
            if theirs + 15 < own_cost or (abs(theirs-own_cost) <= 15 and pid < self.did):
                return True
        return False

    def _choose_bomb(self, tick: int, xy: Vec, visible):
        candidates = []
        for key, tr in self.tracks.items():
            if tr.state in ('done', 'carried'):
                continue
            if tick - tr.last_seen > 420:
                continue
            if self._is_claimed_by_peer(key, tick, xy):
                continue
            # Locally observed targets are less likely to be stale.
            local_bonus = 75 if any(v[1] == key for v in visible) else 0
            cost = distance(xy, tr.xy) - local_bonus
            if self.target_track == key:
                cost -= 80.0
            candidates.append((cost, key))
        if not candidates:
            return None
        return min(candidates)[1]

    def _make_goal(self, tick: int, xy: Vec, visible, carrying: bool):
        if carrying:
            self.mode = 'DELIVER'
            self.target_track = None
            self.frontier_id = None
            self.target = self.disposal or (self.home if self.relative_center else None)
            return
        # Avoid oscillating between targets while approaching an observed bomb.
        target_key = None
        if self.target_track in self.tracks:
            tr0 = self.tracks[self.target_track]
            if (tr0.state not in ('done','carried') and tick-tr0.last_seen<120
                    and distance(xy,tr0.xy)<85):
                target_key = self.target_track
        if target_key is None:
            target_key = self._choose_bomb(tick, xy, visible)
        if target_key is not None:
            if self.target_track != target_key:self.pickup_started=tick
            self.mode = 'CHASE'
            self.target_track = target_key
            self.frontier_id = None
            tr = self.tracks[target_key]
            tr.owner, tr.claimed_at, tr.state = self.did, tick, 'claimed'
            self.target = tr.xy
            return
        was_exploring = self.mode=='EXPLORE'
        self.target_track = None
        self.mode = 'EXPLORE'
        if (was_exploring and self.target is not None
                and distance(xy, self.target) > 32 and tick - self.goal_tick < 105):
            return
        peers = list(self.peers.values())
        idx, frontier = self.grid.frontier(xy, self.home or xy,
                                            self.did, peers, self.frontier_avoid, tick)
        if frontier is not None:
            self.target, self.frontier_id = frontier, idx
            self.goal_tick = tick
        else:
            self.frontier_id = None
            patrol=self.grid.patrol(xy,self.did,tick)
            if patrol is not None:
                self.target=patrol
                self.goal_tick=tick
                return
            # No reachable frontier or patrol point: use id-specific spoke.
            a = 2 * math.pi * (self.did + 0.5) / 10.0
            tx = xy[0] + 120 * math.cos(a)
            ty = xy[1] + 120 * math.sin(a)
            w, h = self.size_area or (1700, 1100)
            self.target = (max(-w/2 + 55, min(w/2 - 55, tx)),
                           max(-h/2 + 55, min(h/2 - 55, ty)))
            self.goal_tick = tick

    def _route(self, tick: int, xy: Vec, target: Vec):
        changed = self.path_target is None or distance(target, self.path_target) > 34
        # V2 replanned EVERY FRAME on an empty route. A* plus repeated 25-cell
        # clearance scans was a major slowdown on narrow maps.
        if changed or tick - self.path_tick >= (62 if self.path else 44):
            self.path = self.grid.astar(xy, target, carrying=self.mode=='DELIVER',
                                        max_expansions=3400)
            self.path_target = target
            self.path_tick = tick
            if not self.path:self.last_route_failure=tick
        # Pop passed waypoints, then safely look ahead along traversable grid.
        while self.path and distance(xy, self.path[0]) < 27:
            self.path.pop(0)
        if self.path:
            candidate = self.path[0]
            for p in self.path[1:9]:
                if distance(xy, p) > 145 or not self._straight_clear(xy, p):
                    break
                candidate = p
            return candidate
        if distance(xy,target)>72:
            # A* failing is NOT evidence that a direct line crosses no wall.
            # V5 fell back to target unconditionally, including through known
            # occupied cells; choose a nearby grid-reachable subgoal instead.
            if tick==self.path_tick:
                self._failed_route_count+=1
                if self._failed_route_count<=3 or self._failed_route_count%20==0:
                    print('BLUE_V6_ROUTE_FAIL drone=%d tick=%d mode=%s target=%s' %
                          (self.did,tick,self.mode,target))
            return self.grid.safe_local_waypoint(xy,target)
        return target

    def _straight_clear(self, a: Vec, b: Vec) -> bool:
        d = distance(a, b)
        for k in range(1, int(d // 12) + 1):
            fraction = min(1.0, k * 12 / max(d, 1))
            p = (a[0] + fraction * (b[0]-a[0]), a[1] + fraction*(b[1]-a[1]))
            idx = self.grid.index(*p)
            if idx is None or not self.grid.passable(idx,carrying=self.mode=='DELIVER'):
                return False
        return True

    def _update_stuck(self, tick: int, xy: Vec, goal: Vec):
        remaining = distance(xy,goal)
        if self.prev_position is None or self.progress_goal is None or distance(self.progress_goal,goal)>90:
            self.prev_position,self.progress_tick,self.progress_distance,self.progress_goal=xy,tick,remaining,goal
            return
        if remaining < self.progress_distance-22:
            self.prev_position,self.progress_tick,self.progress_distance=xy,tick,remaining
        elif tick-self.progress_tick > 58 and remaining>55:
            self.escape_left=max(self.escape_left,18)
            self.escape_dir *= -1
            self.progress_tick = tick
            self.prev_position = xy
            self.progress_distance=remaining
            if self.frontier_id is not None:
                self.frontier_avoid[self.frontier_id] = tick
                self.goal_tick = -9999
            self.path = []

    def _finish_previous_carry(self, tick: int, xy: Optional[Vec], carrying: bool):
        if carrying and not self.was_carrying:
            self.dock_controller.reset()
            self._dock_stage = None
            self._dock_stage_tick = -1000
            self._dock_phase_reported = -1
            # The task selected immediately before pickup is the only bomb
            # to mark as carried; other discovered bombs remain available.
            key = self.target_track
            if key is None and xy is not None:
                key = self._nearest_track(xy, radius=65.0)
            self.active_carry_track = key
            if key is not None and key in self.tracks:
                tr = self.tracks[key]
                tr.state, tr.owner, tr.claimed_at = 'carried', self.did, tick
            print('BLUE_V6_EVENT pickup drone=%d tick=%d track=%s' % (self.did,tick,key))
        elif self.was_carrying and not carrying:
            self.dock_controller.reset()
            self._dock_stage = None
            self._dock_phase_reported = -1
            key = self.active_carry_track
            if key is not None and key in self.tracks:
                tr = self.tracks[key]
                # Empty grasper close to the disposal center normally means
                # the engine auto-disposed the object after physical contact.
                # Do not report 'done' for a remote accidental drop.
                near_center = (xy is not None and self.disposal is not None and
                               distance(xy, self.disposal) < max(self.center_half) + 80)
                tr.state = 'done' if near_center else 'free'
                tr.owner = self.did if near_center else -1
                tr.last_seen = tick
            print('BLUE_V6_EVENT released drone=%d tick=%d track=%s pos=%s' %
                  (self.did,tick,key,xy))
            self.active_carry_track = None
        self.was_carrying = carrying

    def define_message_for_all(self) -> Any:
        if self.communicator_is_disabled():
            return None
        # Called by official framework; never invokes global state or true_*.
        return self.latest_msg or {
            'v': 1, 'id': self.did, 'tick': int(self.elapsed_timestep), 'relative': self.relative_center,
            'pos': list(self.pose.xy) if self.pose.xy else None,
            'home': list(self.disposal) if self.disposal else None,
            'mode': self.mode, 'goal': list(self.target) if self.target else None,
            'carrying': self.was_carrying, 'bombs': [], 'cells': []
        }

    def _prepare_message(self, tick: int, carrying: bool):
        tracks = sorted(self.tracks.values(), key=lambda t: -t.last_seen)[:25]
        self.latest_msg = {
            'v': 1, 'id': self.did, 'tick': tick, 'relative': self.relative_center,
            'pos': list(self.pose.xy) if self.pose.xy else None,
            'home': list(self.disposal) if self.disposal else None,
            'goal': list(self.target) if self.target else None,
            'mode': self.mode, 'carrying': carrying,
            'bombs': [[round(t.xy[0], 1), round(t.xy[1], 1), t.state,
                       t.owner, t.last_seen, t.claimed_at] for t in tracks],
            'cells': self.grid.export(75) if tick % 12 == self.did % 12 else [],
        }

    def _target_avoidance_mask(self):
        """Avoid falsely marking bombs/drones/center as static walls."""
        mask=[]
        for d in self._semantic_cache:
            try:
                if d.entity_type in (DroneSemanticSensor.TypeEntity.BOMB,
                                     DroneSemanticSensor.TypeEntity.DRONE,
                                     DroneSemanticSensor.TypeEntity.DISPOSAL_CENTER):
                    mask.append((float(d.angle),float(d.distance)))
            except (TypeError,ValueError,AttributeError):pass
        return mask

    def _dock_target(self,xy,tick):
        """Plan to a reachable face before contacting the physical center.

        No simulation object is accessed; center dimensions come from the
        public map definitions already used by this strategy.
        """
        if self.disposal is None:return None,False
        if self.relative_center:
            # Center is a point on the *observed surface*, not the global
            # rectangle center. Approach it first and allow contact on sight.
            near=distance(xy,self.disposal)<115
            return self.disposal,near
        cx,cy=self.disposal
        hx,hy=self.center_half
        dx,dy=xy[0]-cx,xy[1]-cy
        inside=max(abs(dx)-hx,abs(dy)-hy)<48
        if inside:
            return self.disposal,True
        if self._dock_stage is None or tick-self._dock_stage_tick>470:
            margin=47.
            candidates=[(cx+hx+margin,cy),(cx-hx-margin,cy),
                        (cx,cy+hy+margin),(cx,cy-hy-margin)]
            candidates=[p for p in candidates if self.grid.index(*p) is not None]
            ranked=[]
            for pt in candidates:
                path=self.grid.astar(xy,pt,carrying=True,max_expansions=1750)
                if path and distance(path[-1],pt)<56:
                    length=sum(distance(a,b) for a,b in zip([xy]+path,path))
                    ranked.append((length,pt))
            if ranked:
                self._dock_stage=min(ranked)[1]
            elif candidates:
                self._dock_stage=min(candidates,key=lambda p:distance(xy,p))
            self._dock_stage_tick=tick
        if self._dock_stage is not None and distance(xy,self._dock_stage)>32:
            return self._dock_stage,False
        return self.disposal,True

    def _pilot_semantics(self, tick, xy, heading, target_track=None, docking=False):
        """Make verified LiDAR suppression masks; NEVER ignore arbitrary walls."""
        ignore=[]
        hazards=[]
        for obj in self._semantic_cache:
            try:
                ang,dist=float(obj.angle),float(obj.distance)
                if not(math.isfinite(ang) and math.isfinite(dist)):
                    continue
                if obj.entity_type==DroneSemanticSensor.TypeEntity.DRONE:
                    hazards.append((ang,dist))
                elif (obj.entity_type==DroneSemanticSensor.TypeEntity.BOMB
                      and target_track is not None and not bool(obj.grasped)
                      and dist<80):
                    tr=self.tracks.get(target_track)
                    point=(xy[0]+dist*math.cos(heading+ang),
                           xy[1]+dist*math.sin(heading+ang))
                    if tr is not None and distance(point,tr.xy)<30:
                        ignore.append((ang,dist))
                elif obj.entity_type==DroneSemanticSensor.TypeEntity.DISPOSAL_CENTER and docking:
                    ignore.append((ang,dist))
            except (ValueError,TypeError,AttributeError):continue
        return ignore,hazards

    def _correct_center_by_semantics(self,xy,heading):
        """Bounded map-relative correction from observed center SURFACE.

        The public center rectangle is a landmark; sensed hit points are NOT
        the rectangle center. Reject inconsistent data and never use truth pose.
        """
        if self.relative_center or self.disposal is None or self.pose.gps_lost_steps<5:
            return xy
        cx,cy=self.disposal
        hx,hy=self.center_half
        errors=[]
        for item in self._semantic_cache:
            if item.entity_type!=DroneSemanticSensor.TypeEntity.DISPOSAL_CENTER:
                continue
            try:
                r,a=float(item.distance),float(item.angle)
                if not(math.isfinite(r) and math.isfinite(a) and 3<r<135):continue
            except (ValueError,TypeError):continue
            px=xy[0]+r*math.cos(heading+a)
            py=xy[1]+r*math.sin(heading+a)
            qx=clamp(px,cx-hx,cx+hx)
            qy=clamp(py,cy-hy,cy+hy)
            if cx-hx<px<cx+hx and cy-hy<py<cy+hy:
                edges=[(abs(px-(cx-hx)),cx-hx,py),
                       (abs(px-(cx+hx)),cx+hx,py),
                       (abs(py-(cy-hy)),px,cy-hy),
                       (abs(py-(cy+hy)),px,cy+hy)]
                _,qx,qy=min(edges)
            ex,ey=qx-px,qy-py
            if math.hypot(ex,ey)<48:errors.append((ex,ey))
        if len(errors)<3:return xy
        errors.sort(key=lambda p:p[0]);ex=errors[len(errors)//2][0]
        errors.sort(key=lambda p:p[1]);ey=errors[len(errors)//2][1]
        norm=math.hypot(ex,ey)
        if norm>40:return xy
        factor=min(.38,9./max(norm,1.))
        corrected=(xy[0]+factor*ex,xy[1]+factor*ey)
        self.pose.xy=corrected
        self.pose.uncertainty=max(7.,self.pose.uncertainty*.92)
        return corrected

    def _telemetry(self,tick,xy,heading,carrying,goal):
        if self.mode!=self._last_mode or tick-self.last_trace_tick>=175:
            v=math.hypot(*self.pose.motion)
            print('BLUE_V6 drone=%d tick=%d state=%s pos=(%.1f,%.1f) speed=%.2f '
                  'carry=%d goal_dist=%.1f route=%d uncertainty=%.1f hp=%s' %
                  (self.did,tick,self.mode,xy[0],xy[1],v,int(carrying),
                   distance(xy,goal) if goal else -1,len(self.path),
                   self.pose.uncertainty,self._health_current))
            self._last_mode,self.last_trace_tick=self.mode,tick

    def control(self) -> CommandsDict:
        tick=int(self.elapsed_timestep)
        self.last_tick=tick
        try:
            self.pose.update(tick,self.measured_gps_position(),
                             self.measured_compass_angle(),self.odometer_values())
            if self.pose.xy is None:
                self._initial_public_anchor()
            # No arbitrary world-frame heading assignment: relative heading
            # is seeded together with a relative position above.
            xy,heading=self.pose.xy,self.pose.heading
            carrying=bool(self.grasped_bombs())
            self._health_loss = 0
            own_health = getattr(self, 'drone_health', None)
            hp = own_health() if callable(own_health) else None
            if hp is not None:
                hp = int(hp)
                if self._health_last is not None and hp < self._health_last:
                    self._health_loss = self._health_last-hp
                    self._damage_times.append(tick)
                    print('BLUE_V6_DAMAGE drone=%d tick=%d loss=%d hp=%d mode=%s phase=%d' %
                          (self.did,tick,self._health_loss,hp,self.mode,
                           self.dock_controller.phase))
                self._health_last = hp
            self._health_current = hp
            self._receive(tick)
            if xy is None or heading is None:
                self._prepare_message(tick,carrying)
                return safe_command(rotation=.25,grasper=int(carrying))
            if self.home is None:self.home=xy
            self._select_center(xy)
            self._finish_previous_carry(tick,xy,carrying)
            lidar,angles=self.lidar_values(),self.lidar_rays_angles()
            self._semantic_cache=self.semantic_values() or []
            # Correct GPS-outage drift only when a globally anchored center is
            # known. Local-frame center detection occurs in _observe below.
            xy=self._correct_center_by_semantics(xy,heading)
            mask=self._target_avoidance_mask()
            self.pose.scan_match(self.grid,tick,lidar,angles,ignore=mask)
            xy,heading=self.pose.xy,self.pose.heading
            if self.pose.uncertainty<100.:
                self.grid.scan(xy,heading,lidar,angles,tick,ignore=mask)
            visible=self._observe(tick,xy,heading)
            for key,tr in self.tracks.items():
                if tr.state=='claimed' and tr.owner==self.did and key!=self.target_track:
                    tr.state,tr.owner='free',-1
            self._make_goal(tick,xy,visible,carrying)
            target=self.target
            if target is None:
                self._prepare_message(tick,carrying)
                return safe_command(rotation=.22,grasper=int(carrying))
            # Stagger launch to avoid initial dense formation collision.
            motion=self.pose.motion
            if tick<8+self.did*5 and not carrying:
                self._prepare_message(tick,carrying)
                return safe_command(rotation=.08 if self.did%2 else -.08)
            self._update_stuck(tick,xy,target)
            docking=False
            if carrying and self.disposal is not None:
                target,docking=self._dock_target(xy,tick)
            else:
                self._dock_stage=None
            if self.escape_left>0 and not docking:
                self.escape_left-=1
                self._prepare_message(tick,carrying)
                sign=float(self.escape_dir)
                rear=self.pilot.clearance(angles,lidar,math.pi)
                side=self.pilot.clearance(angles,lidar,sign*math.pi/2)
                if rear>48:
                    return safe_command(forward=-.28,rotation=.12*sign,grasper=int(carrying))
                if side>52:
                    return safe_command(lateral=.25*sign,rotation=.12*sign,grasper=int(carrying))
                return safe_command(rotation=.28*sign,grasper=int(carrying))
            waypoint=target if docking else self._route(tick,xy,target)
            peers=[p for p in self.peers.values()
                   if tick-int(p.get('tick',-99999))<75]
            close_target=(self.mode=='CHASE' and self.target_track in self.tracks
                          and distance(xy,self.tracks[self.target_track].xy)<75)
            ignored,hazards=self._pilot_semantics(tick,xy,heading,
                                                   self.target_track if close_target else None,
                                                   docking=docking)
            # Use odometer displacement, not measured_velocity's mislabeled
            # px/s interpretation. Engine's motion is per step.
            # Health feedback is informative but not an episode oracle.
            # Apply mild speed scaling only after recent collision events;
            # otherwise preserve V5's successful transport throughput.
            recent_damage=sum(1 for t in self._damage_times if tick-t<180)
            safety_scale=max(.64,1.-.09*recent_damage)
            if hp is not None and hp<=7:
                safety_scale=min(safety_scale,.78)
            if self.pose.uncertainty>32:
                safety_scale=min(safety_scale,.76)
            fwd,lat,rot=self.pilot.command(
                heading,xy,waypoint,lidar,angles,peers=peers,carrying=carrying,
                docking=docking,velocity=motion,
                ignore=ignored,close_target=close_target,
                near_drone=hazards,stuck=self.escape_left>0,
                speed_scale=safety_scale)
            if docking:
                # Directly command the pivot-jointed towing motion. V4's
                # generic pilot capped force at 0.36 and combined a slow
                # inward speed with weak tangential motion; this left many
                # carriers stalled against the disposal rectangle for 1000+
                # steps. Do not confuse a DRONE touching the center with
                # the BOMB touching it: disposal requires the latter.
                hit=None
                center_rays=[]
                for item in self._semantic_cache:
                    try:
                        if item.entity_type!=DroneSemanticSensor.TypeEntity.DISPOSAL_CENTER:
                            continue
                        rr,aa=float(item.distance),float(item.angle)
                        if not(math.isfinite(rr) and math.isfinite(aa) and 2.<rr<130.):
                            continue
                        point=(xy[0]+rr*math.cos(heading+aa),
                               xy[1]+rr*math.sin(heading+aa))
                        center_rays.append((rr,point))
                    except (ValueError,TypeError,AttributeError):
                        continue
                if center_rays:
                    hit=min(center_rays,key=lambda it:it[0])[1]
                ctr=self.dock_controller.action(
                    tick,xy,heading,motion,self.disposal,
                    None if self.relative_center else self.center_half,
                    relative=self.relative_center,lidar=lidar,angles=angles,
                    pilot=self.pilot,semantic_hit=hit,peers=hazards,
                    health_drop=self._health_loss,health=hp)
                if ctr is not None:
                    fwd,lat,rot,phase,face_gap=ctr
                    if (phase!=self._dock_phase_reported or
                            tick%180==self.did%10):
                        print('BLUE_V6_DOCK drone=%d tick=%d phase=%d gap=%.1f '
                              'speed=%.2f force=%.2f hp=%s recoveries=%d' %
                              (self.did,tick,phase,face_gap,math.hypot(*motion),
                               math.hypot(fwd,lat),hp,
                               self.dock_controller.recoveries))
                    self._dock_phase_reported=phase
                self.delivery_struggles+=1
            else:
                # Preserve the state within an approach, but do not apply
                # stale docking forces when we are navigating a corridor.
                self.delivery_struggles=0
            if self.mode=='CHASE' and self.target_track in self.tracks:
                tr=self.tracks[self.target_track]
                near=distance(xy,tr.xy)
                if near<70:
                    a=wrap(math.atan2(tr.xy[1]-xy[1],tr.xy[0]-xy[0])-heading)
                    rot=clamp(1.0*a,-.65,.65)
                    # A bomb is an intended CONTACT target, not a wall. Move
                    # through the observed bomb-surface point with grasper held.
                    if 10<near<57 and math.hypot(fwd,lat)<.18:
                        vf=motion[0]*math.cos(heading)+motion[1]*math.sin(heading)
                        vl=-motion[0]*math.sin(heading)+motion[1]*math.cos(heading)
                        fwd=clamp(.26*math.cos(a)-.07*vf,-.38,.38)
                        lat=clamp(.26*math.sin(a)-.07*vl,-.38,.38)
                    if near<30 and tick-self.pickup_started>80:
                        # Small controlled angular perturbation changes the
                        # grasper contact geometry without blindly accelerating.
                        rot=clamp(rot+(.22 if self.did%2 else -.22),-.65,.65)
            if carrying and self.active_carry_track in self.tracks:
                tr=self.tracks[self.active_carry_track]
                tr.state,tr.owner,tr.claimed_at='carried',self.did,tick
            self._prepare_message(tick,carrying)
            self._telemetry(tick,xy,heading,carrying,target)
            return safe_command(fwd,lat,rot,int(carrying or self.mode=='CHASE'))
        except (TypeError,ValueError,ArithmeticError,IndexError,AttributeError):
            return safe_command(grasper=int(bool(self.grasped_bombs())))
