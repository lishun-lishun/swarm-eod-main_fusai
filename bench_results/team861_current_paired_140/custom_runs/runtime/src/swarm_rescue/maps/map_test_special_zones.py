import random
import gc
import math
import pathlib
import sys
from typing import List, Type, Optional

# Insert the parent directory of the current file's directory into sys.path.
# This allows Python to locate modules that are one level above the current
# script, in this case simulation.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from swarm_rescue.simulation.drone.drone_abstract import DroneAbstract
from swarm_rescue.simulation.drone.drone_motionless import DroneMotionless
from swarm_rescue.simulation.elements.disposal_center import DisposalCenter
from swarm_rescue.simulation.elements.return_area import ReturnArea
from swarm_rescue.simulation.elements.sensor_disablers import (
    ZoneType, NoComZone, KillZone, NoGpsZone
)
from swarm_rescue.simulation.elements.bomb import Bomb
from swarm_rescue.simulation.gui_map.closed_playground import ClosedPlayground
from swarm_rescue.simulation.gui_map.gui_sr import GuiSR
from swarm_rescue.simulation.gui_map.map_abstract import MapAbstract
from swarm_rescue.simulation.reporting.evaluation import ZonesConfig, EvalConfig, EvalPlan
from swarm_rescue.simulation.utils.misc_data import MiscData


class MapTestSpecialZones(MapAbstract):

    def __init__(self,
                 drone_type: Type[DroneAbstract],
                 zones_config: ZonesConfig = (),
                 use_shaders: Optional[bool] = None,
                 seed: Optional[int] = None):
        super().__init__(drone_type, zones_config)
        self._rng = random.Random(seed if seed is not None else random.getrandbits(64))
        self._max_timestep_limit = 1200
        self._max_walltime_limit = 60  # In seconds

        # PARAMETERS MAP
        self._size_area = (600, 300)

        self._disposal_center = DisposalCenter(size=(100, 290))
        self._disposal_center_pos = ((-250, 0), 0)

        self._return_area = ReturnArea(size=(100, 290))
        self._return_area_pos = ((-150, 0), 0)

        self._no_com_zone = NoComZone(size=(80, 290))
        self._no_com_zone_pos = ((-50, 0), 0)

        self._no_gps_zone = NoGpsZone(size=(80, 290))
        self._no_gps_zone_pos = ((50, 0), 0)

        self._kill_zone = KillZone(size=(80, 290))
        self._kill_zone_pos = ((250, 0), 0)

        self._bombs_pos = \
            [(-150, 100), (-50, 100), (50, 100), (150, 100), (250, 100),
             (-150, -100), (-50, -100), (50, -100), (150, -100), (250, -100)]
        self._number_bombs = len(self._bombs_pos)
        self._bombs: List[Bomb] = []

        self._number_drones = 4
        # They are positioned in a square whose side size depends on the total number of drones.
        start_area_drones = (-125, 0)
        nb_per_side = math.ceil(math.sqrt(float(self._number_drones)))
        dist_inter_drone = 50.0
        # print("nb_per_side", nb_per_side)
        # print("dist_inter_drone", dist_inter_drone)
        sx = start_area_drones[0] - (nb_per_side - 1) * 0.5 * dist_inter_drone
        sy = start_area_drones[1] - (nb_per_side - 1) * 0.5 * dist_inter_drone
        # print("sx", sx, "sy", sy)

        self._drones_pos = []
        for i in range(self._number_drones):
            x = sx + (float(i) % nb_per_side) * dist_inter_drone
            y = sy + math.floor(float(i) / nb_per_side) * dist_inter_drone
            angle = self._rng.uniform(-math.pi, math.pi)
            self._drones_pos.append(((x, y), angle))

        self._drones: List[DroneAbstract] = []

        self._playground = ClosedPlayground(
            size=self._size_area
        )

        self._playground.add(self._return_area, self._return_area_pos)
        self._playground.add(self._disposal_center, self._disposal_center_pos)

        # DISABLER ZONES
        if ZoneType.NO_COM_ZONE in self._zones_config:
            self._playground.add(self._no_com_zone, self._no_com_zone_pos)

        if ZoneType.NO_GPS_ZONE in self._zones_config:
            self._playground.add(self._no_gps_zone, self._no_gps_zone_pos)

        if ZoneType.KILL_ZONE in self._zones_config:
            self._playground.add(self._kill_zone, self._kill_zone_pos)

        # POSITIONS OF THE WOUNDED PERSONS

        # POSITIONS OF THE DRONES
        self._explored_map.initialize_walls(self._playground)
        misc_data = MiscData(size_area=self._size_area,
                             number_drones=self._number_drones,
                             max_timestep_limit=self._max_timestep_limit,
                             max_walltime_limit=self._max_walltime_limit)
        for i in range(self._number_drones):
            drone = drone_type(identifier=i, misc_data=misc_data)
            self._drones.append(drone)
            self._playground.add(drone, self._drones_pos[i])

