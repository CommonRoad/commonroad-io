import itertools
import warnings
from collections import defaultdict
from typing import Union, List, Set, Dict, Tuple
import numpy as np
import enum

from commonroad.common.util import Interval
from commonroad.common.validity import is_real_number, is_real_number_vector, is_valid_orientation
from commonroad.scenario.lanelet import Lanelet
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.obstacle import ObstacleRole
from commonroad.scenario.obstacle import ObstacleType
from commonroad.scenario.obstacle import StaticObstacle, DynamicObstacle, Obstacle
from commonroad.prediction.prediction import Occupancy, SetBasedPrediction
from commonroad.scenario.intersection import Intersection
from commonroad.scenario.traffic_sign import TrafficSign, TrafficLight

__author__ = "Stefanie Manzinger, Moritz Klischat, Sebastian Maierhofer"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["Priority Program SPP 1835 Cooperative Interacting Automobiles"]
__version__ = "2020.2"
__maintainer__ = "Stefanie Manzinger"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "Released"


@enum.unique
class Tag(enum.Enum):
    """ Enum containing all possible tags of a CommonRoad scenario."""
    INTERSTATE = "interstate"
    URBAN = "urban"
    HIGHWAY = "highway"
    COMFORT = "comfort"
    CRITICAL = "critical"
    EVASIVE = "evasive"
    CUT_IN = "cut_in"
    ILLEGAL_CUTIN = "illegal_cutin"
    INTERSECTION = "intersection"
    LANE_CHANGE = "lane_change"
    LANE_FOLLOWING = "lane_following"
    MERGING_LANES = "merging_lanes"
    MULTI_LANE = "multi_lane"
    ONCOMING_TRAFFIC = "oncoming_traffic"
    NO_ONCOMING_TRAFFIC = "no_oncoming_traffic"
    PARALLEL_LANES = "parallel_lanes"
    RACE_TRACK = "race_track"
    ROUNDABOUT = "roundabout"
    RURAL = "rural"
    SIMULATED = "simulated"
    SINGLE_LANE = "single_lane"
    SLIP_ROAD = "slip_road"
    SPEED_LIMIT = "speed_limit"
    TRAFFIC_JAM = "traffic_jam"
    TURN_LEFT = "turn_left"
    TURN_RIGHT = "turn_right"
    TWO_LANE = "two_lane"
    EMERGENCY_BRAKING = "emergency_braking"


class GeoTransformation:
    def __init__(self, geo_reference: str = None, x_translation: float = None, y_translation: float = None,
                 z_rotation: float = None, scaling: float = None):
        """
        Constructor of a location object

        :param geo_reference: proj-string describing transformation from geodetic to projected Cartesian coordinates
        :param x_translation: translation value for x-coordinates
        :param y_translation: translation value for y-coordinates
        :param z_rotation: rotation value around origin
        :param scaling: multiplication value of x- and y-coordinates
        """
        self._geo_reference = geo_reference
        self._x_translation = x_translation
        self._y_translation = y_translation
        self._z_rotation = z_rotation
        self._scaling = scaling

    @property
    def geo_reference(self) -> str:
        return self._geo_reference

    @property
    def x_translation(self) -> float:
        return self._x_translation

    @property
    def y_translation(self) -> float:
        return self._y_translation

    @property
    def z_rotation(self) -> float:
        return self._z_rotation

    @property
    def scaling(self) -> float:
        return self._scaling


class Location:
    def __init__(self, geo_name_id: int = -999, gps_latitude: float = 999, gps_longitude: float = 999,
                 geo_transformation: GeoTransformation = None):
        """
        Constructor of a location object

        :param geo_name_id: GeoName ID
        :param gps_latitude: GPS latitude coordinate
        :param gps_longitude: GPS longitude coordinate
        :param geo_transformation: description of geometric transformation during scenario generation
        """
        self._geo_name_id = geo_name_id
        self._gps_latitude = gps_latitude
        self._gps_longitude = gps_longitude
        self._geo_transformation = geo_transformation

    @property
    def geo_name_id(self) -> int:
        return self._geo_name_id

    @property
    def gps_latitude(self) -> float:
        return self._gps_latitude

    @property
    def gps_longitude(self) -> float:
        return self._gps_longitude

    @property
    def geo_transformation(self) -> GeoTransformation:
        return self._geo_transformation


class Scenario:
    """ Class which describes a Scenario entity according to the CommonRoad specification. Each scenario is described by
     a road network consisting of lanelets (see :class:`commonroad.scenario.lanelet.LaneletNetwork`) and a set of
     obstacles which can be either static or dynamic (see :class:`commonroad.scenario.obstacle.Obstacle`)."""
    def __init__(self, dt: float, benchmark_id: str,
                 author: str = None, tags: Set[Tag] = None, affiliation: str = None, source: str = None,
                 location: Location = None):
        """
        Constructor of a Scenario object

        :param dt: global time step size of the time-discrete scenario
        :param benchmark_id: unique CommonRoad benchmark ID of the scenario
        :param author: authors of the CommonRoad scenario
        :param tags: tags describing and classifying the scenario
        :param affiliation: institution of the authors
        :param source: source of the scenario, e.g. generated by a map converter and a traffic simulator
        :param location: location object of the scenario
        """
        self.dt: float = dt
        self.benchmark_id: str = benchmark_id
        self.lanelet_network: LaneletNetwork = LaneletNetwork()

        self._static_obstacles: Dict[int, StaticObstacle] = defaultdict()
        self._dynamic_obstacles: Dict[int, DynamicObstacle] = defaultdict()

        self._id_set: Set[int] = set()

        # meta data
        self.author = author
        self.tags = tags
        self.affiliation = affiliation
        self.source = source
        self.location = location

    @property
    def dt(self) -> float:
        """ Global time step size of the time-discrete scenario."""
        return self._dt

    @dt.setter
    def dt(self, dt: float):
        assert is_real_number(dt), '<Scenario/dt> argument "dt" of wrong type. ' \
                                   'Expected a real number. Got type: %s.' % type(dt)
        self._dt = dt

    @property
    def benchmark_id(self) -> str:
        """ Unique benchmark ID of a scenario as specified in the CommonRoad XML-file."""
        return self._benchmark_id

    @benchmark_id.setter
    def benchmark_id(self, benchmark_id):
        assert isinstance(benchmark_id, str), '<Scenario/benchmark_id> argument "benchmark_id" of wrong type. ' \
                                              'Expected type: %s. Got type: %s' % (str, type(benchmark_id))
        self._benchmark_id = benchmark_id

    @property
    def lanelet_network(self) -> LaneletNetwork:
        """ Road network composed of lanelets."""
        return self._lanelet_network

    @lanelet_network.setter
    def lanelet_network(self, lanelet_network: LaneletNetwork):
        assert isinstance(lanelet_network, LaneletNetwork), '<Scenario/lanelet_network> argument "lanelet_network"' \
                                                            ' of wrong type. Expected type: %s. Got type: %s.' \
                                                            % (LaneletNetwork, type(lanelet_network))
        self._lanelet_network = lanelet_network

    @property
    def dynamic_obstacles(self) -> List[DynamicObstacle]:
        """ Returns a list of all dynamic obstacles in the scenario."""
        return list(self._dynamic_obstacles.values())

    @property
    def static_obstacles(self) -> List[StaticObstacle]:
        """ Returns a list of all static obstacles in the scenario."""
        return list(self._static_obstacles.values())

    @property
    def obstacles(self) -> List[Obstacle]:
        """ Returns a list of all static and dynamic obstacles in the scenario."""
        return list(itertools.chain(self._static_obstacles.values(),
                                    self._dynamic_obstacles.values()))

    def add_objects(self, scenario_object: Union[List[Union[Obstacle, Lanelet, LaneletNetwork, TrafficSign,
                                                            TrafficLight, Intersection]], Obstacle, Lanelet,
                                                 LaneletNetwork, TrafficSign, TrafficLight, Intersection],
                    lanelet_ids: Set[int] = None):
        """ Function to add objects, e.g., lanelets, dynamic and static obstacles, to the scenario.

            :param scenario_object: object(s) to be added to the scenario
            :param lanelet_ids: lanelet IDs a traffic sign, traffic light should be referenced from
            :raise ValueError: a value error is raised if the type of scenario_object is invalid.
        """
        if isinstance(scenario_object, list):
            for obj in scenario_object:
                self.add_objects(obj)
        elif isinstance(scenario_object, StaticObstacle):
            self._mark_object_id_as_used(scenario_object.obstacle_id)
            self._static_obstacles[scenario_object.obstacle_id] = scenario_object
            self._add_static_obstacle_to_lanelets(scenario_object.obstacle_id,
                                                  scenario_object.initial_shape_lanelet_ids)
        elif isinstance(scenario_object, DynamicObstacle):
            self._mark_object_id_as_used(scenario_object.obstacle_id)
            self._dynamic_obstacles[scenario_object.obstacle_id] = scenario_object
            self._add_dynamic_obstacle_to_lanelets(scenario_object)
        elif isinstance(scenario_object, LaneletNetwork):
            for lanelet in scenario_object.lanelets:
                self._mark_object_id_as_used(lanelet.lanelet_id)
            self.lanelet_network: LaneletNetwork = scenario_object
        elif isinstance(scenario_object, Lanelet):
            self._mark_object_id_as_used(scenario_object.lanelet_id)
            self._lanelet_network.add_lanelet(scenario_object)
        elif isinstance(scenario_object, TrafficSign):
            self._mark_object_id_as_used(scenario_object.traffic_sign_id)
            self._lanelet_network.add_traffic_sign(scenario_object, lanelet_ids)
        elif isinstance(scenario_object, TrafficLight):
            self._mark_object_id_as_used(scenario_object.traffic_light_id)
            self._lanelet_network.add_traffic_light(scenario_object, lanelet_ids)
        elif isinstance(scenario_object, Intersection):
            self._mark_object_id_as_used(scenario_object.intersection_id)
            self._lanelet_network.add_intersection(scenario_object)

        else:
            raise ValueError('<Scenario/add_objects> argument "scenario_object" of wrong type. '
                             'Expected types: %s, %s, %s, and %s. Got type: %s.'
                             % (list, Obstacle, Lanelet, LaneletNetwork, type(scenario_object)))

    def _add_static_obstacle_to_lanelets(self, obstacle_id: int, lanelet_ids: Set[int]):
        """ Adds a static obstacle reference to all lanelets the obstacle is on.

        :param obstacle_id: obstacle ID to be removed
        :param lanelet_ids: list of lanelet IDs on which the obstacle is on
        """
        if lanelet_ids is None or len(self.lanelet_network.lanelets) == 0:
            return
        for l_id in lanelet_ids:
            self.lanelet_network.find_lanelet_by_id(l_id).static_obstacles_on_lanelet.add(obstacle_id)

    def _remove_static_obstacle_from_lanelets(self, obstacle_id: int, lanelet_ids: Set[int]):
        """ Adds a static obstacle reference to all lanelets the obstacle is on.

        :param obstacle_id: obstacle ID to be added
        :param lanelet_ids: list of lanelet IDs on which the obstacle is on
        """
        if lanelet_ids is None:
            return
        for l_id in lanelet_ids:
            self.lanelet_network.find_lanelet_by_id(l_id).static_obstacles_on_lanelet.remove(obstacle_id)

    def _remove_dynamic_obstacle_from_lanelets(self, obstacle: DynamicObstacle):
        """ Removes a dynamic obstacle reference from all lanelets the obstacle is on.

        :param obstacle: obstacle to be removed
        """
        if isinstance(obstacle.prediction, SetBasedPrediction) or len(self.lanelet_network.lanelets) == 0:
            return
        # delete obstacle references from initial time step
        if obstacle.initial_shape_lanelet_ids is not None:
            for lanelet_id in obstacle.initial_shape_lanelet_ids:
                lanelet_dict = self.lanelet_network.find_lanelet_by_id(lanelet_id).dynamic_obstacles_on_lanelet
                lanelet_dict[obstacle.initial_state.time_step].discard(obstacle.obstacle_id)

        # delete obstacle references from prediction
        if obstacle.prediction is not None and obstacle.prediction.shape_lanelet_assignment is not None:
            for time_step, ids in obstacle.prediction.shape_lanelet_assignment.items():
                for lanelet_id in ids:
                    lanelet_dict = self.lanelet_network.find_lanelet_by_id(lanelet_id).dynamic_obstacles_on_lanelet
                    lanelet_dict[time_step].discard(obstacle.obstacle_id)

    def _add_dynamic_obstacle_to_lanelets(self, obstacle: DynamicObstacle):
        """ Adds a dynamic obstacle reference to all lanelets the obstacle is on.

        :param obstacle: obstacle to be added
        """
        if isinstance(obstacle.prediction, SetBasedPrediction) or len(self.lanelet_network.lanelets) == 0:
            return
        # add obstacle references to initial time step
        if obstacle.initial_shape_lanelet_ids is not None:
            for lanelet_id in obstacle.initial_shape_lanelet_ids:
                lanelet_dict = self.lanelet_network.find_lanelet_by_id(lanelet_id).dynamic_obstacles_on_lanelet
                if lanelet_dict.get(obstacle.initial_state.time_step) is None:
                    lanelet_dict[obstacle.initial_state.time_step] = set()
                lanelet_dict[obstacle.initial_state.time_step].add(obstacle.obstacle_id)

        # add obstacle references to prediction
        if obstacle.prediction is not None and obstacle.prediction.shape_lanelet_assignment is not None:
            for time_step, ids in obstacle.prediction.shape_lanelet_assignment.items():
                for lanelet_id in ids:
                    lanelet_dict = self.lanelet_network.find_lanelet_by_id(lanelet_id).dynamic_obstacles_on_lanelet
                    if lanelet_dict.get(time_step) is None:
                        lanelet_dict[time_step] = set()
                    lanelet_dict[time_step].add(obstacle.obstacle_id)

    def remove_obstacle(self, obstacle: Union[Obstacle, List[Obstacle]]):
        """ Removes a static, dynamic or a list of obstacles from the scenario. If the obstacle ID is not assigned,
        a warning message is given.

        :param obstacle: obstacle to be removed
        """
        assert isinstance(obstacle, (list, Obstacle)), '<Scenario/remove_obstacle> argument "obstacle" of wrong type. '\
                                                       'Expected type: %s. Got type: %s.' % (Obstacle, type(obstacle))
        if isinstance(obstacle, list):
            for obs in obstacle:
                self.remove_obstacle(obs)
            return

        if obstacle.obstacle_id in self._static_obstacles:
            self._remove_static_obstacle_from_lanelets(obstacle.obstacle_id, obstacle.initial_shape_lanelet_ids)
            del self._static_obstacles[obstacle.obstacle_id]
            self._id_set.remove(obstacle.obstacle_id)
        elif obstacle.obstacle_id in self._dynamic_obstacles:
            self._remove_dynamic_obstacle_from_lanelets(obstacle)
            del self._dynamic_obstacles[obstacle.obstacle_id]
            self._id_set.remove(obstacle.obstacle_id)
        else:
            warnings.warn('<Scenario/remove_obstacle> Cannot remove obstacle with ID %s, '
                          'since it is not contained in the scenario.' % obstacle.obstacle_id)

    def generate_object_id(self) -> int:
        """ Generates a unique ID which is not assigned to any object in the scenario.

            :return: unique object ID
        """
        return max(self._id_set) + 1

    def occupancies_at_time_step(self, time_step: int, obstacle_role: Union[None, ObstacleRole] = None) \
            -> List[Occupancy]:
        """ Returns the occupancies of all static and dynamic obstacles at a specific time step.

            :param time_step: occupancies of obstacles at this time step
            :param obstacle_role: obstacle role as defined in CommonRoad, e.g., static or dynamic
            :return: list of occupancies of the obstacles
        """
        assert isinstance(time_step, int), '<Scenario/occupancies_at_time> argument "time_step" of wrong type. ' \
                                           'Expected type: %s. Got type: %s.' % (int, type(time_step))
        assert isinstance(obstacle_role, (ObstacleRole, type(None))), \
            '<Scenario/obstacles_by_role_and_type> argument "obstacle_role" of wrong type. Expected types: ' \
            ' %s or %s. Got type: %s.' % (ObstacleRole, None, type(obstacle_role))
        occupancies = list()
        for obstacle in self.obstacles:
            if ((obstacle_role is None or obstacle.obstacle_role == obstacle_role) and
                    obstacle.occupancy_at_time(time_step)):
                occupancies.append(obstacle.occupancy_at_time(time_step))
        return occupancies

    def obstacle_by_id(self, obstacle_id: int) -> Union[Obstacle, None]:
        """
        Finds an obstacle for a given obstacle_id

        :param obstacle_id: ID of the queried obstacle
        :return: the obstacle object if the ID exists, otherwise None
        """
        assert isinstance(obstacle_id, int), '<Scenario/obstacle_by_id> argument "obstacle_id" of wrong type. ' \
                                             'Expected type: %s. Got type: %s.' % (int, type(obstacle_id))
        obstacle = None
        if obstacle_id in self._static_obstacles:
            obstacle = self._static_obstacles[obstacle_id]
        elif obstacle_id in self._dynamic_obstacles:
            obstacle = self._dynamic_obstacles[obstacle_id]
        else:
            warnings.warn('<Scenario/obstacle_by_id> Obstacle with ID %s is not contained in the scenario.'
                          % obstacle_id)
        return obstacle

    def obstacles_by_role_and_type(self, obstacle_role: Union[None, ObstacleRole] = None,
                                   obstacle_type: Union[None, ObstacleType] = None) \
            -> List[Obstacle]:
        """
        Filters the obstacles by their role and type.

        :param obstacle_role: obstacle role as defined in CommonRoad, e.g., static or dynamic
        :param obstacle_type: obstacle type as defined in CommonRoad, e.g., car, train, or bus
        :return: list of all obstacles satisfying the given obstacle_role and obstacle_type
        """
        assert isinstance(obstacle_role, (ObstacleRole, type(None))), \
            '<Scenario/obstacles_by_role_and_type> argument "obstacle_role" of wrong type. Expected types: ' \
            ' %s or %s. Got type: %s.' % (ObstacleRole, None, type(obstacle_role))
        assert isinstance(obstacle_type, (ObstacleType, type(None))), \
            '<Scenario/obstacles_by_role_and_type> argument "obstacle_type" of wrong type. Expected types: ' \
            ' %s or %s. Got type: %s.' % (ObstacleType, None, type(obstacle_type))
        obstacle_list = list()
        for obstacle in self.obstacles:
            if ((obstacle_role is None or obstacle.obstacle_role == obstacle_role)
                    and (obstacle_type is None or obstacle.obstacle_type == obstacle_type)):
                obstacle_list.append(obstacle)
        return obstacle_list

    def obstacles_by_position_intervals(
            self, position_intervals: List[Interval],
            obstacle_role: Tuple[ObstacleRole] = (ObstacleRole.DYNAMIC, ObstacleRole.STATIC),
            time_step: int = None) -> List[Obstacle]:
        """
        Returns obstacles which center is located within in the given x-/y-position intervals.

        :param position_intervals: list of intervals for x- and y-coordinates [interval_x,  interval_y]
        :param obstacle_role: tuple containing the desired obstacle roles
        :return: list of obstacles in the position intervals
        """
        def contained_in_interval(position: np.ndarray):
            if position_intervals[0].contains(position[0]) and position_intervals[1].contains(position[1]):
                return True
            return False

        if time_step is None:
            time_step=0

        obstacle_list = list()
        if ObstacleRole.STATIC in obstacle_role:
            for obstacle in self.static_obstacles:
                if contained_in_interval(obstacle.initial_state.position):
                    obstacle_list.append(obstacle)
        if ObstacleRole.DYNAMIC in obstacle_role:
            for obstacle in self.dynamic_obstacles:
                occ = obstacle.occupancy_at_time(time_step)
                if occ is not None:
                    if not hasattr(occ.shape, 'center'):
                        obstacle_list.append(obstacle)
                    elif contained_in_interval(occ.shape.center):
                        obstacle_list.append(obstacle)
        return obstacle_list

    def translate_rotate(self, translation: np.ndarray, angle: float):
        """ Translates and rotates all objects, e.g., obstacles and road network, in the scenario.

            :param translation: translation vector [x_off, y_off] in x- and y-direction
            :param angle: rotation angle in radian (counter-clockwise)
        """
        assert is_real_number_vector(translation, 2), '<Scenario/translate_rotate>: argument "translation" is ' \
                                                      'not a vector of real numbers of length 2. translation = {}.'\
                                                      .format(translation)
        assert is_valid_orientation(angle), '<Scenario/translate_rotate>: argument "orientation" is not valid. ' \
                                            'angle = {}.'.format(angle)

        self._lanelet_network.translate_rotate(translation, angle)
        for obstacle in self.obstacles:
            obstacle.translate_rotate(translation, angle)

    def _is_object_id_used(self, object_id: int) -> bool:
        """ Checks if an ID is already assigned to an object in the scenario.

            :param object_id: object ID to be checked
            :return: True, if the object ID is already assigned, False otherwise
        """
        return object_id in self._id_set

    def _mark_object_id_as_used(self, object_id: int):
        """ Checks if an ID is assigned to an object in the scenario. If the ID is already assigned an error is
        raised, otherwise, the ID is added to the set of assigned IDs.

        :param object_id: object ID to be checked
        :raise ValueError:  if the object ID is already assigned to another object in the scenario.
        """
        if self._is_object_id_used(object_id):
            raise ValueError("ID %s is already used." % object_id)
        self._id_set.add(object_id)

    def __str__(self):
        traffic_str = "\n"
        traffic_str += "Scenario:\n"
        traffic_str += "- Benchmark ID: {}\n".format(self._benchmark_id)
        traffic_str += "- Time step size: {}\n".format(self._dt)
        traffic_str += "- Number of Obstacles: {}\n".format(len(self.obstacles))
        traffic_str += "- Lanelets:\n"
        traffic_str += str(self._lanelet_network)
        return traffic_str
