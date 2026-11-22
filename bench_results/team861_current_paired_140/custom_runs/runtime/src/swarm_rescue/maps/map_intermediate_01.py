import random
import gc
import math
import pathlib
import sys
from typing import List, Type, Optional

import numpy as np

# Insert the parent directory of the current file's directory into sys.path.
# This allows Python to locate modules that are one level above the current
# script, in this case simulation.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from swarm_rescue.simulation.drone.drone_abstract import DroneAbstract
from swarm_rescue.simulation.drone.drone_motionless import DroneMotionless
from swarm_rescue.simulation.elements.disposal_center import DisposalCenter
from swarm_rescue.simulation.elements.return_area import ReturnArea
from swarm_rescue.simulation.elements.sensor_disablers import ZoneType, NoGpsZone
from swarm_rescue.simulation.elements.bomb import Bomb
from swarm_rescue.simulation.gui_map.closed_playground import ClosedPlayground
from swarm_rescue.simulation.gui_map.gui_sr import GuiSR
from swarm_rescue.simulation.gui_map.map_abstract import MapAbstract
from swarm_rescue.simulation.reporting.evaluation import ZonesConfig, EvalPlan, EvalConfig
from swarm_rescue.simulation.utils.misc_data import MiscData
from swarm_rescue.simulation.utils.pose import Pose

from swarm_rescue.maps.walls_intermediate_map_1 import add_walls, add_boxes


class MapIntermediate01(MapAbstract):

    def __init__(self,
                 drone_type: Type[DroneAbstract],
                 zones_config: ZonesConfig = (),
                 use_shaders: Optional[bool] = None,
                 seed: Optional[int] = None):
        super().__init__(drone_type, zones_config)
        self._rng = random.Random(seed if seed is not None else random.getrandbits(64))
        self._max_timestep_limit = 2700
        self._max_walltime_limit = 270  # In seconds

        # PARAMETERS MAP
        self._size_area = (800, 500)

        self._return_area = ReturnArea(size=(200, 120))
        self._return_area_pos = ((295, 100), 0)

        self._disposal_center = DisposalCenter(size=(200, 80))
        self._disposal_center_pos = ((295, 205), 0)

        self._no_gps_zone = NoGpsZone(size=(400, 500))
        self._no_gps_zone_pos = ((-190, 0), 0)

        self._bombs_pos = [(-310, -180)]
        self._bombs_path = [[(-260, -170), (-360, -190)], ]
        self._number_bombs = len(self._bombs_pos)
        self._bombs: List[Bomb] = []

        orient = self._rng.uniform(-math.pi, math.pi)
        self._drones_pos = [((295, 118), orient)]
        self._number_drones = len(self._drones_pos)
        self._drones: List[DroneAbstract] = []

        self._playground = ClosedPlayground(
            size=self._size_area
        )

        self._playground.add(self._return_area, self._return_area_pos)
        self._playground.add(self._disposal_center, self._disposal_center_pos)

        add_walls(self._playground)
        add_boxes(self._playground)

        # DISABLER ZONES
        if ZoneType.NO_GPS_ZONE in self._zones_config:
            self._playground.add(self._no_gps_zone, self._no_gps_zone_pos)

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

