import itertools
import re
import warnings
from collections import defaultdict
from typing import Union, List, Set, Dict, Tuple
import numpy as np
import enum
import iso3166
from commonroad import SCENARIO_VERSION, SUPPORTED_COMMONROAD_VERSIONS

from commonroad.common.util import Interval
from commonroad.common.validity import is_real_number, is_real_number_vector, is_valid_orientation, is_natural_number
from commonroad.scenario.lanelet import Lanelet
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.obstacle import ObstacleRole
from commonroad.scenario.obstacle import ObstacleType
from commonroad.scenario.obstacle import StaticObstacle, DynamicObstacle, EnvironmentObstacle, Obstacle, State
from commonroad.prediction.prediction import Occupancy, SetBasedPrediction, TrajectoryPrediction
from commonroad.scenario.intersection import Intersection
from commonroad.scenario.traffic_sign import TrafficSign, TrafficLight

__author__ = "Stefanie Manzinger, Moritz Klischat, Sebastian Maierhofer"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["Priority Program SPP 1835 Cooperative Interacting Automobiles"]
__version__ = "2020.3"
__maintainer__ = "Sebastian Maierhofer"
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


@enum.unique
class TimeOfDay(enum.Enum):
    """ Enum containing all possible time of days."""
    DAY = "day"
    NIGHT = "night"


@enum.unique
class Weather(enum.Enum):
    """ Enum containing all possible weathers."""
    SUNNY = "sunny"
    LIGHT_RAIN = "light_rain"
    HEAVY_RAIN = "heavy_rain"
    FOG = "fog"
    SNOW = "snow"
    HAIL = "hail"


@enum.unique
class Underground(enum.Enum):
    """ Enum containing all possible undergrounds."""
    WET = "wet"
    CLEAN = "clean"
    DIRTY = "dirty"
    DAMAGED = "damaged"
    SNOW = "snow"
    ICE = "ice"


class Time:
    """
    Class which describes the fictive time when a scenario starts.
    """

    def __init__(self, hours: int, minutes: int):
        """
        Constructor of a time object

        :param hours: hours at start of scenario (0-24)
        :param minutes: minutes at start of scenario (0-60)
        """
        self._hours = hours
        self._minutes = minutes

    @property
    def hours(self) -> int:
        return self._hours

    @property
    def minutes(self) -> int:
        return self._minutes


class GeoTransformation:
    """
    Class which describes the transformation from geodetic to projected Cartesian coordinates according to the
    CommonRoad specification
    """

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


class Environment:
    """
    Class which describes the environment where a scenario takes place as specified in the CommonRoad specification.
    """

    def __init__(self, time: Time = None, time_of_day: TimeOfDay = None, weather: Weather = None,
                 underground: Underground = None):
        """
        Constructor of an environment object

        :param time: time in hours
        :param time_of_day: current time of day, i.e., day or night
        :param weather: weather information, e.g., sunny
        :param underground: underground information, e.g., ice
        """
        self._time = time
        self._time_of_day = time_of_day
        self._weather = weather
        self._underground = underground

    @property
    def time(self) -> Time:
        return self._time

    @property
    def time_of_day(self) -> TimeOfDay:
        return self._time_of_day

    @property
    def weather(self) -> Weather:
        return self._weather

    @property
    def underground(self) -> Underground:
        return self._underground


class Location:
    """
    Class which describes a location according to the CommonRoad specification.
    """

    def __init__(self, geo_name_id: int = -999, gps_latitude: float = 999, gps_longitude: float = 999,
                 geo_transformation: GeoTransformation = None, environment: Environment = None):
        """
        Constructor of a location object

        :param geo_name_id: GeoName ID
        :param gps_latitude: GPS latitude coordinate
        :param gps_longitude: GPS longitude coordinate
        :param geo_transformation: description of geometric transformation during scenario generation
        :param environment: environmental information, e.g. weather
        """
        self._geo_name_id = geo_name_id
        self._gps_latitude = gps_latitude
        self._gps_longitude = gps_longitude
        self._geo_transformation = geo_transformation
        self._environment = environment

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

    @property
    def environment(self) -> Environment:
        return self._environment


class ScenarioID:
    def __init__(self, cooperative: bool = False, country_id: str = "ZAM", map_name: str = "Test", map_id: int = 1,
                 configuration_id: Union[None, int] = None, prediction_type: Union[None, str] = None,
                 prediction_id: Union[None, int] = None, scenario_version: str = SCENARIO_VERSION):
        """
        Implements the scenario ID as specified in the scenario documentation
        (see https://gitlab.lrz.de/tum-cps/commonroad-scenarios/-/tree/master/documentation)
        Example for benchmark ID C-USA_US101-33_2_T-1
        :param cooperative: True if scenario contains cooperative planning problem sets with multiple planning problems
        :param country_id: three-letter ID according
        :param map_name: name of the map (e.g. US101)
        :param map_id: index of the map (e.g. 33)
        :param configuration_id: enumerates initial configuration of vehicles on the map (e.g. 2)
        :param prediction_type: type of the prediction for surrounding vehicles (e.g. T)
        :param prediction_id: enumerates different predictions for the same initial configuration (e.g. 1)
        :param scenario_version: scenario version identifier (e.g. 2020a)
        """
        assert scenario_version in SUPPORTED_COMMONROAD_VERSIONS, 'Scenario_version {} not supported.' \
            .format(scenario_version)
        self.scenario_version = scenario_version
        self.cooperative = cooperative
        self._country_id = None
        self.country_id = country_id
        self.map_name = map_name
        self.map_id = map_id
        self.configuration_id = configuration_id
        self.prediction_type = prediction_type
        self.prediction_id = prediction_id

    def __str__(self):
        scenario_id = ""
        if self.cooperative is True:
            scenario_id += "C-"
        if self.country_id is not None:
            scenario_id += self.country_id + "_"
        if self.map_name is not None:
            scenario_id += self.map_name + "-"
        if self.map_id is not None:
            scenario_id += str(self.map_id)
        if self.configuration_id is not None:
            scenario_id += "_" + str(self.configuration_id)
        if self.prediction_type is not None:
            scenario_id += "_" + self.prediction_type + "-"
        if self.prediction_id is not None:
            scenario_id += str(self.prediction_id)
        return scenario_id

    @property
    def country_id(self):
        return self._country_id

    @country_id.setter
    def country_id(self, country_id: str):
        if country_id is None:
            self._country_id = 'ZAM'
        elif country_id in iso3166.countries_by_alpha3 or country_id == 'ZAM':
            self._country_id = country_id
        else:
            raise ValueError('Country ID {} is not in the ISO-3166 three-letter format. '.format(country_id))

    @property
    def country_name(self):
        return iso3166.countries_by_alpha3[self.country_id].name

    @classmethod
    def from_benchmark_id(cls, benchmark_id: str, scenario_version: str):
        """
        Create ScenarioID from benchmark_id and scenario_version in the XML header.
        :param benchmark_id: scenario ID provided as a string
        :param scenario_version: scenario format version (e.g. 2020a)
        :return: 
        """
        if not (benchmark_id.count('_') in (1, 2, 3) and benchmark_id.count('-') in (1, 2, 3)):
            warnings.warn('Not a valid scenario id: ' + benchmark_id)
            return ScenarioID(None, None, benchmark_id, 0, None, None, None)

        # extract sub IDs from string
        if benchmark_id[0:2] == 'C-':
            cooperative = True
            benchmark_id = benchmark_id[2:]
        else:
            cooperative = False

        sub_ids = re.split('_|-', benchmark_id)
        country_id, map_name, map_id = sub_ids[:3]
        map_id = int(map_id)

        configuration_id = prediction_type = prediction_id = None
        if len(sub_ids) > 3:
            configuration_id = int(sub_ids[3])
        if len(sub_ids) > 4:
            assert sub_ids[4] in ('S', 'T', 'I'), "prediction type must be one of (S, T, I) but is {}".format(
                sub_ids[4])
            prediction_type = sub_ids[4]
            prediction_id = int(sub_ids[5])

        return ScenarioID(cooperative, country_id, map_name, map_id, configuration_id, prediction_type, prediction_id,
                          scenario_version)

    def __eq__(self, other: 'ScenarioID'):
        return str(self) == str(other)


class Scenario:
    """ Class which describes a Scenario entity according to the CommonRoad specification. Each scenario is described by
     a road network consisting of lanelets (see :class:`commonroad.scenario.lanelet.LaneletNetwork`) and a set of
     obstacles which can be either static or dynamic (see :class:`commonroad.scenario.obstacle.Obstacle`)."""

    def __init__(self, dt: float, scenario_id: Union[str, ScenarioID],
                 author: str = None, tags: Set[Tag] = None, affiliation: str = None, source: str = None,
                 location: Location = None, benchmark_id: str = None):
        """
        Constructor of a Scenario object

        :param dt: global time step size of the time-discrete scenario
        :param benchmark_id: unique CommonRoad benchmark ID of the scenario
        :param author: authors of the CommonRoad scenario
        :param tags: tags describing and classifying the scenario
        :param affiliation: institution of the authors
        :param source: source of the scenario, e.g. generated by a map converter and a traffic simulator
        :param location: location object of the scenario
        :param benchmark_id: for backwards compatibility
        """
        self.dt: float = dt
        self.scenario_id = scenario_id
        if isinstance(scenario_id, str):
            self.scenario_id = ScenarioID.from_benchmark_id(scenario_id, SCENARIO_VERSION)
        elif scenario_id is None and benchmark_id is not None:
            warnings.warn('Use the  the class commonroad.scenario.ScenarioID to define the scenario id.',
                          DeprecationWarning)
            self.scenario_id = ScenarioID.from_benchmark_id(benchmark_id,
                                                            SCENARIO_VERSION)

        self.lanelet_network: LaneletNetwork = LaneletNetwork()

        self._static_obstacles: Dict[int, StaticObstacle] = defaultdict()
        self._dynamic_obstacles: Dict[int, DynamicObstacle] = defaultdict()
        self._environment_obstacle: Dict[
            int, EnvironmentObstacle] = defaultdict()

        self._id_set: Set[int] = set()
        # Count ids generated but not necessarily added yet
        self._id_counter = None

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
        warnings.warn('benchmark_id is deprecated, use scenario_id instead', DeprecationWarning)
        return str(self.scenario_id)

    @benchmark_id.setter
    def benchmark_id(self, benchmark_id):
        raise ValueError('benchmark_id is deprecated, use scenario_id instead')

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
    def obstacles(self) -> List[Union[Obstacle, StaticObstacle, DynamicObstacle]]:
        """ Returns a list of all static and dynamic obstacles in the scenario."""
        return list(itertools.chain(self._static_obstacles.values(),
                                    self._dynamic_obstacles.values()))

    @property
    def environment_obstacle(self) -> List[EnvironmentObstacle]:
        """ Returns a list of all environment_obstacles in the scenario."""
        return list(self._environment_obstacle.values())

    def add_objects(self, scenario_object: Union[List[Union[Obstacle, Lanelet, LaneletNetwork, TrafficSign,
                                                            TrafficLight, Intersection, EnvironmentObstacle]], Obstacle,
                                                 Lanelet, LaneletNetwork, TrafficSign, TrafficLight, Intersection,
                                                 EnvironmentObstacle],
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
            for traffic_sign in scenario_object.traffic_signs:
                self._mark_object_id_as_used(traffic_sign.traffic_sign_id)
            for traffic_light in scenario_object.traffic_lights:
                self._mark_object_id_as_used(traffic_light.traffic_light_id)
            for intersection in scenario_object.intersections:
                self._mark_object_id_as_used(intersection.intersection_id)
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
        elif isinstance(scenario_object, EnvironmentObstacle):
            self._mark_object_id_as_used(scenario_object.obstacle_id)
            self._environment_obstacle[scenario_object.obstacle_id] = scenario_object

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
        """ Remove a static obstacle reference from all lanelets the obstacle is on.

        :param obstacle_id: obstacle ID to be added
        :param lanelet_ids: list of lanelet IDs on which the obstacle is on
        """
        if lanelet_ids is None:
            return
        obs: StaticObstacle = self.obstacle_by_id(obstacle_id)
        l_ids = obs.initial_center_lanelet_ids
        if l_ids is not None:
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
        assert isinstance(obstacle, (list, Obstacle)), '<Scenario/remove_obstacle> argument "obstacle" of wrong type. ' \
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
        if self._id_counter is None:
            self._id_counter = 0
        if len(self._id_set) > 0:
            max_id_used = max(self._id_set)
            self._id_counter = max(self._id_counter, max_id_used)
        self._id_counter += 1
        return self._id_counter

    def occupancies_at_time_step(self, time_step: int, obstacle_role: Union[None, ObstacleRole] = None) \
            -> List[Occupancy]:
        """ Returns the occupancies of all static and dynamic obstacles at a specific time step.

            :param time_step: occupancies of obstacles at this time step
            :param obstacle_role: obstacle role as defined in CommonRoad, e.g., static or dynamic
            :return: list of occupancies of the obstacles
        """
        assert is_natural_number(time_step), '<Scenario/occupancies_at_time> argument "time_step" of wrong type. ' \
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

    def obstacle_by_id(self, obstacle_id: int) -> Union[Obstacle, DynamicObstacle, StaticObstacle, None]:
        """
        Finds an obstacle for a given obstacle_id

        :param obstacle_id: ID of the queried obstacle
        :return: the obstacle object if the ID exists, otherwise None
        """
        assert is_natural_number(obstacle_id), '<Scenario/obstacle_by_id> argument "obstacle_id" of wrong type. ' \
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
            time_step = 0

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

    def obstacle_states_at_time_step(self, time_step: int) -> Dict[int, State]:
        """
        Returns all obstacle states which exist at a provided time step.

        :param time_step: time step of interest
        :return: dictionary which maps id to obstacle state at time step
        """
        assert is_natural_number(time_step), '<Scenario/obstacle_at_time_step> argument "time_step" of wrong type. ' \
                                             'Expected type: %s. Got type: %s.' % (int, type(time_step))

        obstacle_states = {}
        for obstacle in self.dynamic_obstacles:
            if obstacle.state_at_time(time_step) is not None:
                obstacle_states[obstacle.obstacle_id] = obstacle.state_at_time(time_step)
        for obstacle in self.static_obstacles:
            obstacle_states[obstacle.obstacle_id] = obstacle.initial_state
        return obstacle_states

    def assign_obstacles_to_lanelets(self, time_steps: Union[List[int], None] = None,
                                     obstacle_ids: Union[Set[int], None] = None,
                                     use_center_only=False):
        """
        Assigns center points and shapes of obstacles to lanelets by setting the attributes
        Obstacle.prediction.initial_shape_lanelet_ids, .shape_lanelet_assignment, .initial_center_lanelet_ids,
        & .center_lanelet_assignment, and Lanelet.dynamic_obstacles_on_lanelet & .static_obstacles_on_lanelet.

        :param time_steps: time step for which the obstacles should be assigned. If None, all time_steps are
        assigned.
        :param obstacle_ids: ids for which the assignment should be computed. If None, all obstacles are
        :param use_center_only: if False, the shape is used to find occupied lanelets.
        Otherwise, only the center is used.
        assigned.
        """

        def assign_dynamic_obstacle_shape_at_time(obstacle: DynamicObstacle, time_step):
            # assign center of obstacle
            # print(time_step, [state.time_step for state in obstacle.prediction.trajectory.state_list])
            if time_step == obstacle.initial_state.time_step:
                position = obstacle.initial_state.position
            elif not isinstance(obstacle.prediction, TrajectoryPrediction):
                return
            else:
                position = obstacle.prediction.trajectory.state_at_time_step(time_step).position

            lanelet_ids_center = set(self.lanelet_network.find_lanelet_by_position([position])[0])
            obstacle.prediction.center_lanelet_assignment[time_step] = lanelet_ids_center

            if use_center_only:
                lanelet_ids = lanelet_ids_center
            else:
                # assign shape of obstacle
                shape = obstacle.occupancy_at_time(time_step).shape
                lanelet_ids = set(self.lanelet_network.find_lanelet_by_shape(shape))
                obstacle.prediction.shape_lanelet_assignment[time_step] = lanelet_ids

            if time_step == obstacle.initial_state.time_step:
                if not use_center_only:
                    obstacle.initial_shape_lanelet_ids = lanelet_ids
                obstacle.initial_center_lanelet_ids = lanelet_ids_center
            for l_id in lanelet_ids:
                self.lanelet_network.find_lanelet_by_id(l_id) \
                    .add_dynamic_obstacle_to_lanelet(obstacle_id=obstacle.obstacle_id, time_step=time_step)

        def assign_static_obstacle(obstacle: StaticObstacle):
            shape = obstacle.occupancy_at_time(0).shape
            if not use_center_only:
                lanelet_ids = set(self.lanelet_network.find_lanelet_by_shape(shape))
                obstacle.initial_shape_lanelet_ids = lanelet_ids
            lanelet_ids = set(self.lanelet_network.find_lanelet_by_position([obstacle.initial_state.position])[0])
            obstacle.initial_center_lanelet_ids = lanelet_ids

            for l_id in lanelet_ids:
                self.lanelet_network.find_lanelet_by_id(l_id) \
                    .add_static_obstacle_to_lanelet(obstacle_id=obstacle.obstacle_id)

        if obstacle_ids is None:
            # assign all obstacles
            obstacle_ids = set(self._static_obstacles.keys()).union(set(self._dynamic_obstacles.keys()))

        for obs_id in obstacle_ids:
            obs = self.obstacle_by_id(obs_id)
            if isinstance(obs, DynamicObstacle):
                if time_steps is None:
                    # assign all time steps
                    time_steps_tmp = range(obs.initial_state.time_step,
                                           obs.prediction.final_time_step + 1)
                else:
                    time_steps_tmp = time_steps

                if not use_center_only and obs.prediction.shape_lanelet_assignment is None:
                    obs.prediction.shape_lanelet_assignment = {}
                if obs.prediction.center_lanelet_assignment is None:
                    obs.prediction.center_lanelet_assignment = {}

                for t in time_steps_tmp:
                    assign_dynamic_obstacle_shape_at_time(obs, t)
            else:
                assign_static_obstacle(obs)

    def translate_rotate(self, translation: np.ndarray, angle: float):
        """ Translates and rotates all objects, e.g., obstacles and road network, in the scenario.

            :param translation: translation vector [x_off, y_off] in x- and y-direction
            :param angle: rotation angle in radian (counter-clockwise)
        """
        assert is_real_number_vector(translation, 2), '<Scenario/translate_rotate>: argument "translation" is ' \
                                                      'not a vector of real numbers of length 2. translation = {}.' \
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
        if self._id_counter is None:
            self._id_counter = object_id
        if self._is_object_id_used(object_id):
            raise ValueError("ID %s is already used." % object_id)
        self._id_set.add(object_id)

    def __str__(self):
        traffic_str = "\n"
        traffic_str += "Scenario:\n"
        traffic_str += "- Scenario ID: {}\n".format(str(self.scenario_id))
        traffic_str += "- Time step size: {}\n".format(self._dt)
        traffic_str += "- Number of Obstacles: {}\n".format(len(self.obstacles))
        traffic_str += "- Lanelets:\n"
        traffic_str += str(self._lanelet_network)
        return traffic_str
