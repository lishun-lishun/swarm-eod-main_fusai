from swarm_rescue.simulation.elements.disposal_center import DisposalCenter
from swarm_rescue.simulation.elements.normal_wall import NormalBox
import random
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
from swarm_rescue.simulation.elements.bomb import Bomb
from swarm_rescue.simulation.gui_map.closed_playground import ClosedPlayground
from swarm_rescue.simulation.gui_map.gui_sr import GuiSR
from swarm_rescue.simulation.gui_map.map_abstract import MapAbstract
from swarm_rescue.simulation.reporting.evaluation import ZonesConfig
from swarm_rescue.simulation.utils.misc_data import MiscData


class MapRandom(MapAbstract):

    def __init__(self,
                 drone_type: Type[DroneAbstract],
                 zones_config: ZonesConfig = (),
                 use_shaders: Optional[bool] = None,
                 seed: Optional[int] = None):
        super().__init__(drone_type, zones_config)
        self._rng = random.Random(seed if seed is not None else random.getrandbits(64))
        self._number_drones = 10
        self._number_bombs = 0
        self._max_timestep_limit = 220
        self._max_walltime_limit = 22  # In seconds
        self._size_area = (1500, 700)
        self._bombs: List[Bomb] = []
        self._drones: List[DroneAbstract] = []

        self._playground = ClosedPlayground(
            size=self._size_area
        )

        self._disposal_center = DisposalCenter(size=(90, 90))
        self._playground.add(self._disposal_center, ((0, 0), 0))
        for point in ((-430, 145), (300, 160), (-390, -160), (360, -170)):
            box = NormalBox(up_left_point=point, width=70, height=54)
            self._playground.add(box, box.wall_coordinates)
        self._max_timestep_limit = 2000
        self._max_walltime_limit = 90

        # POSITIONS OF THE DRONES
        self._explored_map.initialize_walls(self._playground)
        misc_data = MiscData(size_area=self._size_area,
                             number_drones=self._number_drones,
                             max_timestep_limit=self._max_timestep_limit,
                             max_walltime_limit=self._max_walltime_limit)
        for i in range(self._number_drones):
            while True:
                x = self._rng.uniform(-self._size_area[0] / 2 + 24, self._size_area[0] / 2 - 24)
                y = self._rng.uniform(-self._size_area[1] / 2 + 24, self._size_area[1] / 2 - 24)
                occupied = any(shape.point_query((x, y)).distance < 20
                               for element in self._playground.elements for shape in element.pm_shapes)
                if not occupied and all(math.dist((x, y), drone.true_position()) >= 36 for drone in self._drones):
                    break
            angle = self._rng.uniform(-math.pi, math.pi)
            drone = drone_type(identifier=i, misc_data=misc_data)
            self._drones.append(drone)
            self._playground.add(drone, ((x, y), angle))

