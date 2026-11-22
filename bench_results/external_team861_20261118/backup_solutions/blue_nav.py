"""Swarm-Rescue V6: independent, legal-sensor navigation.

Sensors: LiDAR, compass/GPS, odometry, semantic detections and received radio.
No engine truth, static map geometry, or bomb placements are ever read.
Engine applies body-frame FORCES, one simulated second per control step.
"""
from __future__ import annotations

import heapq
import math
from collections import deque


def clamp(x, lo=-1.0, hi=1.0):
    return max(lo, min(hi, float(x)))


def wrap(angle):
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def finite_pair(value):
    try:
        x, y = float(value[0]), float(value[1])
        if math.isfinite(x) and math.isfinite(y):
            return x, y
    except (ValueError, TypeError, IndexError):
        pass
    return None


class PoseTracker:
    """SE(2) dead-reckoning, measurement correction, conservative scan matching."""
    def __init__(self):
        self.xy = None
        self.heading = None
        self.motion = (0.0, 0.0)   # px / one official simulation step
        self.uncertainty = float('inf')
        self.gps_lost_steps = 0
        self.last_tick = -1
        self.last_match_tick = -999
        # If both global sensors are unavailable at the first observation,
        # maintain an internally consistent odometric coordinate frame.
        self.relative_frame = False
        self.global_from_local = None  # (heading offset, translation x/y)

    def seed(self, xy, heading=None):
        """Optional public spawn prior if GPS is disabled at initialization."""
        if self.xy is None:
            self.xy = (float(xy[0]), float(xy[1]))
            self.uncertainty = 23.0
        if heading is not None and self.heading is None:
            self.heading = wrap(heading)

    def seed_relative(self):
        if self.relative_frame:
            return
        if self.xy is None or self.heading is None:
            # An isolated GPS measurement without compass does not define the
            # orientation of the local LiDAR/odometry map; start a local frame.
            self.xy, self.heading = (0.0, 0.0), (self.heading if self.heading is not None else 0.0)
            self.uncertainty = 12.0
            self.relative_frame = True

    def update(self, tick, gps, compass, odometer):
        if tick == self.last_tick:
            return
        self.last_tick = tick
        g = finite_pair(gps)
        h = None
        try:
            if compass is not None and math.isfinite(float(compass)):
                h = wrap(float(compass))
        except (ValueError, TypeError):
            pass
        # A GPS coordinate is unusable in a local-only map until the global
        # orientation is known. Infer one SE(2) transform if GPS and compass
        # return together, and convert measurements to the stored local frame.
        if self.relative_frame:
            if self.global_from_local is None and g is not None and h is not None and self.xy is not None and self.heading is not None:
                delta = wrap(h - self.heading)
                c, sn = math.cos(delta), math.sin(delta)
                tx = g[0] - (c * self.xy[0] - sn * self.xy[1])
                ty = g[1] - (sn * self.xy[0] + c * self.xy[1])
                self.global_from_local = (delta, tx, ty)
            if self.global_from_local is not None:
                delta, tx, ty = self.global_from_local
                c, sn = math.cos(delta), math.sin(delta)
                if g is not None:
                    gx, gy = g[0] - tx, g[1] - ty
                    g = (c * gx + sn * gy, -sn * gx + c * gy)
                if h is not None:
                    h = wrap(h - delta)
            else:
                g, h = None, None
        if self.heading is None and h is not None:
            self.heading = h
        if self.xy is None and g is not None:
            self.xy, self.uncertainty = g, 7.0
        self.motion = (0.0, 0.0)
        if odometer is not None and self.xy is not None and self.heading is not None:
            try:
                ds, alpha, theta = map(float, odometer[:3])
                if all(map(math.isfinite, (ds, alpha, theta))):
                    ds = clamp(ds, 0.0, 50.0)
                    a = self.heading + alpha
                    self.motion = (ds * math.cos(a), ds * math.sin(a))
                    self.xy = (self.xy[0] + self.motion[0],
                               self.xy[1] + self.motion[1])
                    self.heading = wrap(self.heading + clamp(theta, -0.7, 0.7))
                    self.uncertainty = min(250.0, self.uncertainty + 0.035 + 0.032 * ds)
            except (ValueError, TypeError, IndexError):
                pass
        if h is not None:
            self.heading = h if self.heading is None else wrap(self.heading + 0.4 * wrap(h - self.heading))
        if g is not None:
            if self.xy is None:
                self.xy = g
            else:
                w = .19 if not self.gps_lost_steps else .42
                if distance(g, self.xy) > 100:
                    w = .52
                self.xy = ((1-w)*self.xy[0] + w*g[0],
                           (1-w)*self.xy[1] + w*g[1])
            self.gps_lost_steps = 0
            self.uncertainty = max(4.5, min(19., self.uncertainty * .72))
        else:
            self.gps_lost_steps += 1

    def scan_match(self, grid, tick, lidar, angles, ignore=()):
        """Sparse correlative LiDAR correction when GPS/compass are absent.

        Reject solutions without enough known occupied endpoints and sufficient
        improvement over odometry. No own current scan is inserted before match.
        """
        if (self.gps_lost_steps < 6 or self.xy is None or self.heading is None
                or tick - self.last_match_tick < 12 or grid.known_obstacles < 18
                or lidar is None or angles is None):
            return False
        self.last_match_tick = tick
        samples = []
        for i in range(0, min(len(lidar), len(angles)), 9):
            try:
                r, a = float(lidar[i]), float(angles[i])
                if not (math.isfinite(r) and math.isfinite(a) and 20 < r < 210):
                    continue
                if any(abs(wrap(a-m)) < .17 and abs(r-d) < 18 for m,d in ignore):
                    continue
                samples.append((r,a))
            except (ValueError,TypeError):
                pass
        if len(samples) < 6:
            return False
        x0,y0 = self.xy
        a0 = self.heading
        def match_score(x,y,a):
            score = 0
            valid = 0
            for r,ang in samples:
                idx = grid.index(x + r*math.cos(a+ang), y + r*math.sin(a+ang))
                if idx is None:
                    continue
                v = grid.log[idx]
                if v >= 2:
                    score += 2
                    valid += 1
                elif v <= -2:
                    score -= 1
            return score, valid
        baseline = match_score(x0,y0,a0)
        best = (baseline[0], 0, 0, 0, baseline[1])
        # Small local corrections only: never perform global map teleportation.
        for dx,dy in ((0,0),(12,0),(-12,0),(0,12),(0,-12),(9,9),(-9,-9)):
            for da in (-.055,0.,.055):
                if dx==dy==0 and da==0:
                    continue
                score, support = match_score(x0+dx,y0+dy,a0+da)
                value = score - .025*(abs(dx)+abs(dy)) - 1.5*abs(da)
                if support >= 4 and value > best[0] + .05:
                    best = (value,dx,dy,da,support)
        if best[4] >= 4 and best[0] > baseline[0] + 2.5:
            self.xy = (x0 + .6*best[1], y0 + .6*best[2])
            self.heading = wrap(a0 + .6*best[3])
            self.uncertainty = max(8., self.uncertainty*.83)
            return True
        return False


class Occupancy:
    """24 px log-odds grid with cached 8-neighbor obstacle-distance field."""
    CELL = 24
    def __init__(self, width, height):
        self.width, self.height = int(width), int(height)
        self.nx = int(math.ceil(self.width/self.CELL))
        self.ny = int(math.ceil(self.height/self.CELL))
        self.n = self.nx * self.ny
        self.log = [0]*self.n
        self.seen = bytearray(self.n)
        self.visits = [0]*self.n
        self.dirty = {}
        self._dist = [8]*self.n
        self._geometry_dirty = True
        self.known_obstacles = 0

    def index(self, x, y):
        ix = int(math.floor((x+self.width*.5)/self.CELL))
        iy = int(math.floor((y+self.height*.5)/self.CELL))
        return iy*self.nx+ix if 0<=ix<self.nx and 0<=iy<self.ny else None

    def center(self,idx):
        return ((idx%self.nx+.5)*self.CELL-self.width*.5,
                (idx//self.nx+.5)*self.CELL-self.height*.5)

    def update(self,idx,value,seen=True,relay=True):
        old=self.log[idx]
        new=int(clamp(value,-6,6))
        if old!=new or (seen and not self.seen[idx]):
            if (old>=3)!=(new>=3):
                self._geometry_dirty=True
                self.known_obstacles+=1 if new>=3 else -1
            self.log[idx]=new
            if seen:self.seen[idx]=1
            if relay:self.dirty[idx]=(new,int(self.seen[idx]))

    def scan(self,xy,heading,values,angles,tick,ignore=()):
        at=self.index(*xy)
        if at is not None:
            self.visits[at]=min(255,self.visits[at]+1)
            self.update(at,min(-3,self.log[at]))
        if values is None or angles is None or tick%6:
            return
        for k in range(0,min(len(values),len(angles)),4):
            try:
                r,a=float(values[k]),float(angles[k])
                if not (math.isfinite(r) and math.isfinite(a)):
                    continue
                if any(abs(wrap(a-ia))<.18 and abs(r-ir)<20 for ia,ir in ignore):
                    continue
            except (ValueError,TypeError):
                continue
            r=clamp(r,0.,230.)
            ca,sa=math.cos(heading+a),math.sin(heading+a)
            prev=None
            for step in range(1,int(max(0.,r-15.)/14.)+1):
                idx=self.index(xy[0]+14*step*ca,xy[1]+14*step*sa)
                if idx is not None and idx!=prev:
                    self.update(idx,self.log[idx]-1)
                prev=idx
            if 15<r<218:
                hit=self.index(xy[0]+r*ca,xy[1]+r*sa)
                if hit is not None:self.update(hit,self.log[hit]+2)

    def merge(self,updates):
        if not isinstance(updates,list):return
        for item in updates[:120]:
            try:idx,val,seen=(int(z) for z in item)
            except (ValueError,TypeError):continue
            if not(0<=idx<self.n and -6<=val<=6):continue
            old=self.log[idx]
            if (val>=3 and old<3) or (abs(val)>abs(old) and not(old>=3 and val<0)):
                self.update(idx,val,bool(seen),relay=False)
            elif seen and not self.seen[idx]:
                self.update(idx,old,True,relay=False)

    def export(self,limit=75):
        data=list(self.dirty.items())[:limit]
        for idx,_ in data:self.dirty.pop(idx,None)
        return [[idx,val,seen] for idx,(val,seen) in data]

    def _rebuild_clearance(self):
        if not self._geometry_dirty:
            return
        # Multi-source 8-connected BFS, O(number of grid cells).
        # Distance measured in grid hops (conservative Chebyshev metric).
        d=[8]*self.n
        q=deque()
        for i in range(self.n):
            x,y=i%self.nx,i//self.nx
            if x==0 or y==0 or x==self.nx-1 or y==self.ny-1 or self.log[i]>=3:
                d[i]=0
                q.append(i)
        while q:
            i=q.popleft()
            if d[i]>=7:continue
            x,y=i%self.nx,i//self.nx
            for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                xx,yy=x+dx,y+dy
                if 0<=xx<self.nx and 0<=yy<self.ny:
                    j=yy*self.nx+xx
                    if d[j]>d[i]+1:
                        d[j]=d[i]+1
                        q.append(j)
        self._dist=d
        self._geometry_dirty=False

    def clearance(self,idx):
        self._rebuild_clearance()
        return self._dist[idx]*self.CELL

    def passable(self,idx,carrying=False):
        self._rebuild_clearance()
        # One neighbor grid cell (~24 px) clearance for empty or loaded robot;
        # loaded robot pays a higher soft cost near walls, not an impassable maze.
        return self._dist[idx]>=1 and self.log[idx]<3

    def _neighbors(self,idx,carrying=False):
        x,y=idx%self.nx,idx//self.nx
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
            xx,yy=x+dx,y+dy
            if not (0<=xx<self.nx and 0<=yy<self.ny):continue
            nxt=yy*self.nx+xx
            if not self.passable(nxt,carrying):continue
            if dx and dy:
                if not(self.passable(y*self.nx+xx,carrying) and self.passable(yy*self.nx+x,carrying)):
                    continue
            yield nxt,math.hypot(dx,dy)

    def astar(self,start,goal,carrying=False,max_expansions=3500):
        src,dst=self.index(*start),self.index(*goal)
        if src is None or dst is None:return []
        self._rebuild_clearance()
        if src==dst:return [goal]
        if not self.passable(dst,carrying):
            tx,ty=dst%self.nx,dst//self.nx
            choices=[]
            for radius in range(1,5):
                for dy in range(-radius,radius+1):
                    for dx in range(-radius,radius+1):
                        x,y=tx+dx,ty+dy
                        if 0<=x<self.nx and 0<=y<self.ny:
                            j=y*self.nx+x
                            if self.passable(j,carrying):choices.append(j)
                if choices:break
            if not choices:return []
            dst=min(choices,key=lambda j:distance(self.center(j),goal))
        costs={src:0.}
        prev={}
        queue=[(0.,src)]
        visited=set()
        best=src
        near=float('inf')
        for _ in range(max_expansions):
            if not queue:break
            _,here=heapq.heappop(queue)
            if here in visited:continue
            visited.add(here)
            hc=distance(self.center(here),self.center(dst))/self.CELL
            if hc<near:near,best=hc,here
            if here==dst:best=here;break
            for nxt,step in self._neighbors(here,carrying):
                v=self.log[nxt]
                # Unknown is traversable but more costly; protect narrow passages.
                clearance=self._dist[nxt]
                cost=1.+(.55 if not self.seen[nxt] else 0.) + (1.4 if clearance==1 else 0.)
                if carrying and clearance<=2:cost+=.8
                if v>=1:cost+=.7
                new=costs[here]+step*cost
                if new<costs.get(nxt,float('inf')):
                    costs[nxt]=new
                    prev[nxt]=here
                    h=distance(self.center(nxt),self.center(dst))/self.CELL
                    heapq.heappush(queue,(new+h,nxt))
        if best==src:return []
        route=[]
        node=best
        while node!=src and len(route)<self.n:
            route.append(self.center(node))
            node=prev.get(node,src)
        route.reverse()
        if best==dst and distance(route[-1],goal)<self.CELL*1.6:
            route.append(goal)
        return route

    def safe_local_waypoint(self,pos,goal,depth=4):
        """Collision-aware local fallback when A* cannot find a route.

        Never steer straight through a known occupied cell merely because
        A* failed. The result is a reachable, *observed or unknown* local
        waypoint chosen from the grid; it is not a map-file prior.
        """
        source=self.index(*pos)
        if source is None:
            return pos
        self._rebuild_clearance()
        queue=deque([(source,0)])
        visited={source}
        candidates=[]
        while queue:
            node,steps=queue.popleft()
            if node!=source and self.passable(node):
                xy=self.center(node)
                d=distance(xy,goal)
                # Prefer progress, but retreat from a wall when no progress is
                # possible; the clearance term prevents repetitive wall hits.
                score=d + 15.*self.visits[node] + 140./(1+self._dist[node])
                candidates.append((score,xy))
            if steps>=depth:
                continue
            for nxt,_ in self._neighbors(node):
                if nxt not in visited:
                    visited.add(nxt)
                    queue.append((nxt,steps+1))
        if not candidates:
            return pos
        return min(candidates,key=lambda z:z[0])[1]

    def frontier(self,pos,home,ident,peers,avoided,tick):
        self._rebuild_clearance()
        choices=[]
        sector=-math.pi + (ident+.5)*2*math.pi/10
        for i in range(self.n):
            if not self.seen[i] or self.log[i]>-1 or not self.passable(i):continue
            x,y=i%self.nx,i//self.nx
            if not(2<=x<self.nx-2 and 2<=y<self.ny-2):continue
            near=(i-1,i+1,i-self.nx,i+self.nx)
            unknown=sum(not self.seen[j] for j in near)
            if unknown==0:continue
            point=self.center(i)
            travel=distance(pos,point)
            if not(45<travel<1250):continue
            ang=math.atan2(point[1]-home[1],point[0]-home[0])
            score=34*unknown - .19*travel - 27*abs(wrap(ang-sector))
            if avoided.get(i,-9999)>tick-300:score-=200
            for peer in peers:
                p=finite_pair(peer.get('goal'))
                if p is not None and distance(p,point)<130:score-=140
            for yy in range(max(1,y-4),min(self.ny-1,y+5),2):
                for xx in range(max(1,x-4),min(self.nx-1,x+5),2):
                    if not self.seen[yy*self.nx+xx]:score+=11.
            choices.append((score,i,point))
        for _,i,point in heapq.nlargest(4,choices):
            path=self.astar(pos,point,max_expansions=1500)
            if path and distance(path[-1],point)<45:return i,point
        return None,None


    def patrol(self,pos,ident,tick=0):
        """Redistribute search when LiDAR frontiers are exhausted.

        A lidar-seen cell is not proof that its nearby bomb was visible in
        the semantic camera.  Patrol open and infrequently visited corridors
        rather than selecting a clamped point on the map boundary forever.
        """
        self._rebuild_clearance()
        candidates=[]
        for i in range(0,self.n,3):
            x,y=i%self.nx,i//self.nx
            if x<3 or y<3 or x>=self.nx-3 or y>=self.ny-3:
                continue
            if not self.passable(i):
                continue
            p=self.center(i)
            d=distance(pos,p)
            if d<145 or d>1450:
                continue
            # Steer individual IDs to different cells even without radio.
            sector=wrap(math.atan2(p[1]-pos[1],p[0]-pos[0])-
                        (-math.pi+(ident+.5)*math.tau/10))
            recent=self.visits[i]
            score=(130. if not self.seen[i] else 0.)+105./(1+recent) \
                  -.065*d-12.*abs(sector)
            # Seed-independent deterministic tie breaker; differs by drone.
            score+=((i*17+ident*71+(tick//220)*29)%41)*.31
            candidates.append((score,p))
        for _,p in heapq.nlargest(9,candidates):
            route=self.astar(pos,p,carrying=False,max_expansions=1850)
            if route and distance(route[-1],p)<55:
                return p
        return None


class LocalPilot:
    """Direction sampling + collision-constrained velocity force controller.

    v*, v are px / simulation-step. The force is a body-frame command.
    Stopping distance is calculated with an effective conservative deceleration.
    """
    def __init__(self):
        self.last_dir=0.
        self.last_action=(0.,0.)

    @staticmethod
    def clearance(angles,ranges,direction,half_angle=.24,ignore=()):
        if ranges is None or angles is None:return 230.
        best=230.
        hit=False
        for i in range(0,min(len(angles),len(ranges)),3):
            try:
                a,r=float(angles[i]),float(ranges[i])
                if not(math.isfinite(a) and math.isfinite(r)):continue
                if abs(wrap(a-direction))>half_angle:continue
                if any(abs(wrap(a-ia))<.16 and abs(r-ir)<18 for ia,ir in ignore):continue
                best=min(best,r)
                hit=True
            except (TypeError,ValueError):pass
        return clamp(best if hit else 230.,0.,230.)

    def command(self,heading,xy,waypoint,ranges,angles,peers=(),carrying=False,
                docking=False,stuck=False,velocity=None,ignore=(),
                close_target=False,force_speed=None,near_drone=(),speed_scale=1.0):
        dx,dy=waypoint[0]-xy[0],waypoint[1]-xy[1]
        dist=math.hypot(dx,dy)
        desired=wrap(math.atan2(dy,dx)-heading) if dist>1 else 0.
        motion=finite_pair(velocity) or (0.,0.)
        c,s=math.cos(heading),math.sin(heading)
        vf=c*motion[0]+s*motion[1]
        vl=-s*motion[0]+c*motion[1]
        speed=math.hypot(vf,vl)
        safe=39. if carrying else 34.
        if docking:safe=24.
        if close_target:safe=16.
        decel=.95 if carrying else 1.65
        braking=safe+8. + speed*.65 + speed*speed/(2.*decel)
        options=[desired]+[wrap(desired+x) for x in (-.25,.25,-.5,.5,-.85,.85,-1.15,1.15,-1.55,1.55,2.1,-2.1,math.pi)]
        best=(-float('inf'),desired,230.)
        # A moving robot cannot instantaneously rotate its inertia vector.
        # Penalize steering into a hazard along the CURRENT travel direction.
        travel_heading=wrap(math.atan2(vl,vf)) if speed>1.0 else desired
        travel_clearance=self.clearance(angles,ranges,travel_heading,ignore=ignore)
        for direction in options:
            free=self.clearance(angles,ranges,direction,ignore=ignore)
            # Barrier risk, without forbidding navigation near a bomb or center.
            risk=max(0.,braking-free)/max(1.,braking)
            score=2.3*math.cos(wrap(direction-desired)) - 5.0*risk*risk
            if free<safe:score-=5.+.22*(safe-free)
            if speed>3 and travel_clearance < braking:
                score -= 1.7 * (1.0 - min(1.0, travel_clearance/max(1.0,braking)))
            for peer in peers:
                p=finite_pair(peer.get('pos'))
                if p is None:continue
                d=distance(xy,p)
                if d<95:
                    ang=wrap(math.atan2(p[1]-xy[1],p[0]-xy[0])-heading)
                    if abs(wrap(direction-ang))<.6:score-=4.*(95-d)/95
            for a,d in near_drone:
                if d<80 and abs(wrap(direction-a))<.7:score-=4.2*(80-d)/80
            if abs(wrap(direction-self.last_dir))<.2:score+=.08
            if score>best[0]:best=(score,direction,free)
        _,direction,free=best
        self.last_dir=direction
        cruise=10.4 if carrying else 15.2
        if docking:cruise=4.8
        if close_target:cruise=4.8
        if force_speed is not None:cruise=float(force_speed)
        # Use the officially exposed own-health signal to slow down only for a
        # short period after confirmed collisions; normal cruise is unchanged.
        cruise *= clamp(speed_scale, .50, 1.0)
        # Velocity envelope from stopping distance: v_safe <= sqrt(2*a*d).
        allowed=max(0.,free-safe-9.)
        # Never accelerate through the dynamic stopping envelope of the
        # current measured velocity, even if the selected new direction is free.
        if speed>2 and travel_clearance<braking:
            allowed=min(allowed,max(0.,travel_clearance-safe-9.))
        v_safe=max(0.,math.sqrt(2*decel*allowed)-.65*decel)
        desired_speed=min(cruise,v_safe)
        if dist<min(100.,2.5*cruise) and not close_target and not docking:
            desired_speed=min(desired_speed,max(0.,dist/3.))
        if close_target and dist>12 and free>safe+10:
            desired_speed=max(desired_speed,min(3.,cruise))
        if close_target and dist<15:
            desired_speed=min(desired_speed,1.8)
        if docking and dist>25 and free>safe+7:
            desired_speed=max(desired_speed,1.5)
        if dist<8 and not docking and not close_target:
            desired_speed=0.
        if stuck and free>safe+20:
            direction=wrap(direction+(.9 if self.last_dir>=0 else -.9))
            desired_speed=min(desired_speed,3.)
        # Dynamic obstacles are visible in semantic rays; reduce approach
        # velocity rather than relying only on direction-sample penalties.
        for a,d in near_drone:
            if d < 58 and abs(wrap(a-direction)) < .75:
                desired_speed *= clamp((d-22.)/36., 0.0, 1.0)
        target_f,target_l=desired_speed*math.cos(direction),desired_speed*math.sin(direction)
        # Physical force = gain*(desired body speed - measured body speed).
        gain=.11 if carrying else .10
        fwd=gain*(target_f-vf)
        lat=gain*(target_l-vl)
        cap=.78 if carrying else .80
        if docking:cap=.60
        m=math.hypot(fwd,lat)
        if m>cap:fwd,lat=fwd*cap/m,lat*cap/m
        rotation=clamp(1.15*desired,-.8,.8)
        if docking:rotation=clamp(.65*desired,-.45,.45)
        fwd,lat=clamp(fwd,-cap,cap),clamp(lat,-cap,cap)
        self.last_action=(fwd,lat)
        return clamp(fwd),clamp(lat),rotation


class CenterDockController:
    """V6 progress-/health-aware controller for a pivot-jointed bomb.

    State: 0 approach, 1 tangential tow, 2 reverse tow, 3 back away.
    It NEVER releases the grasper. The real disposal event is only produced
    by the simulator after bomb/center collision; this controller does not
    infer or report a disposal from distance alone.

    Controls are fractions of a body-frame force, not instantaneous velocities.
    Only public center dimensions or measured semantic hits are used.
    """

    APPROACH, SWEEP, REVERSE, RECOVER = range(4)

    def __init__(self, identifier=0):
        self.identifier=int(identifier)
        self.reset()

    def reset(self):
        self.entered=None
        self.phase=-1
        self.phase_tick=-9999
        self.tangent_sign=1. if self.identifier%2 else -1.
        self.face=None
        self.reversals=0
        self.recoveries=0
        self.damage_pressure=0.
        self.last_tick=None
        self.movement_position=None
        self.last_movement_tick=-999
        self.last_gap=None
        self.last_speed=0.
        self.last_progress=False

    @staticmethod
    def _normal_and_gap(xy, center, half, relative=False):
        dx,dy=xy[0]-center[0],xy[1]-center[1]
        if relative or half is None:
            d=math.hypot(dx,dy)
            if d<1.e-6:return (1.,0.),0.,None
            return (dx/d,dy/d),d,None
        hx,hy=half
        gx,gy=abs(dx)-hx,abs(dy)-hy
        if gx>gy:
            sign=1. if dx>=0 else -1.
            return (sign,0.),gx,('x',sign)
        sign=1. if dy>=0 else -1.
        return (0.,sign),gy,('y',sign)

    def _transition(self,phase,tick):
        if phase==self.RECOVER and self.phase!=self.RECOVER:
            self.recoveries+=1
        if phase==self.REVERSE and self.phase!=self.REVERSE:
            self.reversals+=1
        if phase==self.APPROACH and self.phase==self.RECOVER:
            self.tangent_sign*=-1.
            self.face=None
        self.phase=phase
        self.phase_tick=tick
        self.last_movement_tick=tick

    def action(self,tick,xy,heading,motion,center,half,relative=False,
               lidar=None,angles=None,pilot=None,semantic_hit=None,peers=(),
               health_drop=0,health=None):
        if center is None:return None
        # Semantic camera detects actual center SURFACE, not center coordinate.
        if relative and semantic_hit is not None:
            center=semantic_hit
        n,gap,face=self._normal_and_gap(xy,center,half,relative)
        if relative and semantic_hit is None and gap>110:
            return None
        # Keep a stable face while close. Switching face every step around a
        # rectangle corner prevents a consistent tow trajectory.
        if self.face is not None and face!=self.face and not relative and gap<125:
            axis,sgn=self.face
            n=(sgn,0.) if axis=='x' else (0.,sgn)
            gap=(xy[0]-center[0])*sgn-half[0] if axis=='x' else (xy[1]-center[1])*sgn-half[1]
        elif face is not None and (self.face is None or gap>125):
            self.face=face
        # A direct semantic hit corrects *surface clearance* in a GPS outage;
        # only accept detections on the same outside side of the chosen face.
        if semantic_hit is not None and not relative:
            hx=xy[0]-semantic_hit[0]
            hy=xy[1]-semantic_hit[1]
            hit_gap=hx*n[0]+hy*n[1]
            off=abs(hx*n[1]-hy*n[0])
            if 2<hit_gap<110 and off<48 and abs(hit_gap-gap)<70:
                gap=.65*gap+.35*hit_gap
        if self.entered is None:
            self.entered=tick
            self._transition(self.APPROACH,tick)
            self.face=face
        # Count actual lateral progress, not just elapsed phase time.
        if self.movement_position is None:
            self.movement_position=xy
            self.last_movement_tick=tick
        elif distance(xy,self.movement_position)>12:
            self.movement_position=xy
            self.last_movement_tick=tick
        speed=math.hypot(*(finite_pair(motion) or (0.,0.)))
        self.last_speed=speed
        self.last_gap=gap
        self.last_progress=(tick-self.last_movement_tick<75)
        dt=1 if self.last_tick is None else max(1,min(10,tick-self.last_tick))
        self.last_tick=tick
        self.damage_pressure=max(0.,self.damage_pressure-.008*dt)+max(0.,health_drop)*1.15
        elapsed=tick-self.phase_tick
        # A damage-triggered recovery is much safer than repeatedly applying
        # force against a rigid center or an adjacent wall.
        if self.damage_pressure>2.05 and self.phase!=self.RECOVER and gap<125:
            self._transition(self.RECOVER,tick)
        elif self.phase==self.APPROACH and (gap<35 or elapsed>=62):
            self._transition(self.SWEEP,tick)
        elif self.phase==self.SWEEP and (elapsed>=100 or (elapsed>55 and not self.last_progress)):
            self._transition(self.REVERSE,tick)
        elif self.phase==self.REVERSE and (elapsed>=78 or (elapsed>50 and not self.last_progress)):
            self._transition(self.RECOVER,tick)
        elif self.phase==self.RECOVER and elapsed>=24:
            self._transition(self.APPROACH,tick)
            self.damage_pressure*=.40
        # Hold a narrow standoff lane. V5 continuously pushed inward with an
        # additive 0.23-0.28 force even after colliding with the center.
        # Signed normal feedback avoids that high-damage persistent contact.
        preferred_gap=15. if self.phase!=self.RECOVER else 44.
        if health is not None and health<=8:
            preferred_gap+=4.
        toward=(-n[0],-n[1])
        tangent=(-n[1]*self.tangent_sign,n[0]*self.tangent_sign)
        # Turn around BEFORE dragging a load into the corner of a rectangle.
        if self.face is not None and not relative:
            axis,_=self.face
            span=half[1] if axis=='x' else half[0]
            along=xy[1]-center[1] if axis=='x' else xy[0]-center[0]
            direction=tangent[1] if axis=='x' else tangent[0]
            if abs(along)>max(18.,span-32.) and along*direction>0:
                self.tangent_sign*=-1.
                tangent=(-tangent[0],-tangent[1])
        # Tangential speed remains high for efficient towing; normal speed is
        # feedback on surface clearance. Both units are pixels / step.
        if self.phase==self.APPROACH:
            vn=clamp(.24*(gap-preferred_gap),-2.,6.5)
            vt=1.1
            cap=.76
        elif self.phase==self.SWEEP:
            vn=clamp(.27*(gap-preferred_gap),-3.5,5.5)
            vt=6.4
            cap=.74
        elif self.phase==self.REVERSE:
            vn=clamp(.25*(gap-preferred_gap),-3.5,5.5)
            vt=-5.8
            cap=.79
        else:
            vn=-clamp(.28*(44.-gap),1.8,5.2)
            vt=3.3
            cap=.70
        # If multiple collisions have been observed, avoid a risky run into
        # the surface; this is local to this drone and fully observable.
        if self.damage_pressure>1.1 and self.phase!=self.RECOVER:
            vn=min(vn,1.2)
            vt*=.88
        if gap>95 and self.phase!=self.RECOVER:
            vn=max(vn,5.5)
            vt*=.35
        if gap<4 and self.phase!=self.RECOVER:
            vn=min(vn,-2.2)
        if pilot is not None and lidar is not None and angles is not None:
            a=wrap(math.atan2(tangent[1],tangent[0])-heading)
            clearance=pilot.clearance(angles,lidar,a,half_angle=.21)
            if clearance<44:
                vt*=clamp((clearance-20.)/24.,0.,1.)
                if clearance<29:
                    self.tangent_sign*=-1.
                    if self.phase!=self.RECOVER and elapsed>20:
                        self._transition(self.RECOVER,tick)
            # During recovery, check whether outward direction is itself clear.
            if self.phase==self.RECOVER:
                a_out=wrap(math.atan2(n[1],n[0])-heading)
                outward=pilot.clearance(angles,lidar,a_out,half_angle=.24)
                if outward<28:vn=0.
        for direction,dd in peers:
            if dd<50 and abs(wrap(direction-wrap(math.atan2(tangent[1],tangent[0])-heading)))<.85:
                vt*=clamp((dd-20.)/30.,.30,1.)
        vx=vn*toward[0]+vt*tangent[0]
        vy=vn*toward[1]+vt*tangent[1]
        mv=finite_pair(motion) or (0.,0.)
        # Bounded force, actual control in body frame.
        fx=.16*(vx-mv[0])
        fy=.16*(vy-mv[1])
        magnitude=math.hypot(fx,fy)
        if magnitude>cap:
            fx,fy=cap*fx/magnitude,cap*fy/magnitude
        c,s=math.cos(heading),math.sin(heading)
        forward=fx*c+fy*s
        lateral=-fx*s+fy*c
        yaw=clamp(.35*wrap(math.atan2(vy,vx)-heading),-.38,.38) if math.hypot(vx,vy)>1 else 0.
        return clamp(forward),clamp(lateral),yaw,self.phase,gap
