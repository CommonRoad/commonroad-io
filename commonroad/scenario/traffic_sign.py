__author__ = "Sebastian Maierhofer"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["CAR@TUM"]
__version__ = "2021.1"
__maintainer__ = "Sebastian Maierhofer"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Release"

import enum
from typing import List, Set, Optional, Tuple
import numpy as np

from commonroad.common.validity import *
import commonroad.geometry.transform
from commonroad.visualization.drawable import IDrawable
from commonroad.visualization.param_server import ParamServer
from commonroad.visualization.renderer import IRenderer

TRAFFIC_SIGN_VALIDITY_START = {'WARNING_DANGER_SPOT', 'WARNING_RIGHT_BEFORE_LEFT', 'WARNING_STEEP_HILL_DOWNWARDS',
                               'WARNING_SLIPPERY_ROAD', 'WARNING_CONSTRUCTION_SITE', 'WARNING_CROSSING_CYCLIST',
                               'WARNING_ANIMAL_CROSSING_RIGHT', 'RAILWAY', 'PRIORITY_OPPOSITE_DIRECTION',
                               'TURN_RIGHT_AHEAD', 'TURN_LEFT_AHEAD', 'ONEWAY_RIGHT', 'ONEWAY_LEFT',
                               'PRESCRIBED_PASSING_LEFT', 'PRESCRIBED_PASSING_RIGHT', 'BIKEWAY', 'SIDEWALK',
                               'PEDESTRIAN_ZONE_START', 'BICYCLE_ROAD_START', 'BUS_LANE', 'BAN_ALL_VEHICLES',
                               'BAN_CARS', 'BAN_TRUCKS', 'BAN_BICYCLE', 'BAN_MOTORCYCLE', 'BAN_BUS', 'BAN_PEDESTRIAN',
                               'BAN_CAR_TRUCK_BUS_MOTORCYCLE', 'BAN_VEHICLES_CARRYING_DANGEROUS_GOODS', 'NO_ENTRY',
                               'MAX_WEIGHT', 'MAX_WIDTH', 'MAX_HEIGHT', 'MAX_LENGTH', 'MAX_SPEED',
                               'MAX_SPEED_ZONE_START', 'MIN_SPEED',  'NO_OVERTAKING_START',
                               'NO_OVERTAKING_TRUCKS_START', 'TRAFFIC_CALMED_AREA_START', 'PRIORITY_OVER_ONCOMING',
                               'TOWN_SIGN', 'TUNNEL', 'INTERSTATE_START', 'HIGHWAY_START', 'PEDESTRIANS_CROSSING'}

TRAFFIC_SIGN_WITH_ADDITIONAL_VALUE = {'MAX_WEIGHT', 'MAX_WIDTH', 'MAX_HEIGHT', 'MAX_LENGTH', 'MAX_SPEED',
                                      'MAX_SPEED_ZONE_START', 'MIN_SPEED', 'ADDITION_VALID_FOR_X_METERS',
                                      'ADDITION_VALID_IN_X_KILOMETERS', 'ADDITION_TIME_PERIOD_PERMITTED'}

LEFT_HAND_TRAFFIC = {'AUS', 'JPN', 'HKG', 'IND', 'JEY', 'IMN', 'IRL', 'JAM',
                     'KEN', 'MLT', 'MYS', 'NPL', 'NZL', 'ZAF', 'SGP', 'THA',
                     'GBR', 'IDN', 'MAC', 'PAK', 'CYP'}


@enum.unique
class SupportedTrafficSignCountry(enum.Enum):
    GERMANY = 'DEU'
    USA = "USA"
    CHINA = "CHN"
    SPAIN = "ESP"
    RUSSIA = "RUS"
    ARGENTINA = "ARG"
    BELGIUM = "BEL"
    FRANCE = "FRA"
    GREECE = "GRC"
    CROATIA = "HRV"
    ITALY = "ITA"
    PUERTO_RICO = "PRI"
    ZAMUNDA = "ZAM"  # default


@enum.unique
class TrafficSignIDGermany(enum.Enum):
    # default traffic sign IDs (similar to German IDs)
    WARNING_DANGER_SPOT = '101'
    WARNING_RIGHT_BEFORE_LEFT = '102'
    WARNING_STEEP_HILL_DOWNWARDS = '108'
    WARNING_SLIPPERY_ROAD = '114'
    WARNING_CONSTRUCTION_SITE = '123'
    WARNING_CROSSING_CYCLIST = '138'
    WARNING_ANIMAL_CROSSING_RIGHT = '142-10'
    RAILWAY = '201'
    YIELD = '205'
    STOP = '206'
    PRIORITY_OPPOSITE_DIRECTION = '208'
    TURN_RIGHT_AHEAD = '209-10'
    TURN_LEFT_AHEAD = '209-20'
    ROUNDABOUT = '215'
    ONEWAY_RIGHT = '220-10'
    ONEWAY_LEFT = '220-20'
    PRESCRIBED_PASSING_LEFT = '222-10'
    PRESCRIBED_PASSING_RIGHT = '222-20'
    BIKEWAY = '237'
    SIDEWALK = '239'
    PEDESTRIAN_ZONE_START = '242.1'
    PEDESTRIAN_ZONE_END = '242.2'
    BICYCLE_ROAD_START = '244.1'
    BICYCLE_ROAD_END = '244.2'
    BUSLANE = '245'
    BAN_ALL_VEHICLES = '250'
    BAN_CARS = '251'
    BAN_TRUCKS = '253'
    BAN_BICYCLE = '254'
    BAN_MOTORCYCLE = '255'
    BAN_BUS = '257-54'
    BAN_PEDESTRIAN = '259'
    BAN_CAR_TRUCK_BUS_MOTORCYCLE = '260'
    BAN_VEHICLES_CARRYING_DANGEROUS_GOODS = '261'
    MAX_WEIGHT = '262'
    MAX_WIDTH = '264'
    MAX_HEIGHT = '265'
    MAX_LENGTH = '266'
    NO_ENTRY = '267'
    U_TURN = '272'
    MAX_SPEED = '274'
    MAX_SPEED_ZONE_START = '274.1'
    MAX_SPEED_ZONE_END = '274.2'
    MIN_SPEED = '275'
    NO_OVERTAKING_START = '276'
    NO_OVERTAKING_TRUCKS_START = '277'
    MAX_SPEED_END = '278'
    NO_OVERTAKING_TRUCKS_END = '281'
    ALL_MAX_SPEED_AND_OVERTAKING_END = '282'
    RIGHT_OF_WAY = '301'
    PRIORITY = '306'
    PRIORITY_OVER_ONCOMING = '308'
    TOWN_SIGN = '310'
    TRAFFIC_CALMED_AREA_START = '325.1'
    TRAFFIC_CALMED_AREA_END = '325.2'
    TUNNEL = '327'
    INTERSTATE_START = '330.1'
    INTERSTATE_END = '330.2'
    HIGHWAY_START = '331.1'
    HIGHWAY_END = '331.2'
    EXIT_BUILT_UP = '333-21'
    EXIT_GENERAL = '333-22'
    PEDESTRIANS_CROSSING = '350'
    DEAD_END = '357'
    DIRECTION_SIGN_LEFT_SINGLE = '625-10'
    DIRECTION_SIGN_LEFT_SMALL = '625-11'
    DIRECTION_SIGN_LEFT_MEDIUM = '625-12'
    DIRECTION_SIGN_LEFT_LARGE = '625-13'
    DIRECTION_SIGN_RIGHT_SINGLE = '625-20'
    DIRECTION_SIGN_RIGHT_SMALL = '625-21'
    DIRECTION_SIGN_RIGHT_MEDIUM = '625-22'
    DIRECTION_SIGN_RIGHT_LARGE = '625-23'
    WARNING_PANEL_RIGHT = '626-10'
    WARNING_PANEL_LEFT = '626-20'
    WARNING_PANEL_STRAIGHT_BROAD = '626-30'
    WARNING_PANEL_STRAIGHT_HIGH = '626-31'
    GREEN_ARROW = '720'
    ADDITION_LEFT_DIRECTION_1 = '1000-10'
    ADDITION_LEFT_DIRECTION_DANGER_POINT = '1000-11'
    ADDITION_RIGHT_DIRECTION_1 = '1000-20'
    ADDITION_RIGHT_DIRECTION_DANGER_POINT = '1000-21'
    ADDITION_BOTH_DIRECTIONS_HORIZONTAL = '1000-30'
    ADDITION_BOTH_DIRECTIONS_VERTICAL = '1000-31'
    ADDITION_VALID_FOR_X_METERS = '1001-30'
    ADDITION_VALID_FOR_X_KILOMETERS = '1001-31'
    ADDITION_LEFT_TURNING_PRIORITY_WITH_OPPOSITE_RIGHT_YIELD = '1002-10'
    ADDITION_LEFT_TRAFFIC_PRIORITY_WITH_STRAIGHT_RIGHT_YIELD = '1002-11'
    ADDITION_LEFT_TURNING_PRIORITY_WITH_OPPOSITE_YIELD = '1002-12'
    ADDITION_LEFT_TURNING_PRIORITY_WITH_RIGHT_YIELD = '1002-13'
    ADDITION_LEFT_TRAFFIC_PRIORITY_WITH_STRAIGHT_YIELD = '1002-14'
    ADDITION_RIGHT_TURNING_PRIORITY_WITH_OPPOSITE_LEFT_YIELD = '1002-20'
    ADDITION_RIGHT_TRAFFIC_PRIORITY_WITH_STRAIGHT_LEFT_YIELD = '1002-21'
    ADDITION_RIGHT_TURNING_PRIORITY_WITH_OPPOSITE_YIELD = '1002-22'
    ADDITION_RIGHT_TURNING_PRIORITY_WITH_LEFT_YIELD = '1002-23'
    ADDITION_RIGHT_TRAFFIC_PRIORITY_WITH_STRAIGHT_YIELD = '1002-24'
    ADDITION_VALID_IN_X_METERS = '1004-30'
    ADDITION_VALID_IN_X_KILOMETERS = '1004-31'
    ADDITION_RESIDENTS_PERMITTED = '1020-30'
    ADDITION_BICYCLES_PERMITTED = '1022-10'
    ADDITION_CARS_PERMITTED = '1024-10'
    ADDITION_AGRICULTURE_PERMITTED = '1026-36'
    ADDITION_FOREST_PERMITTED = '1026-37'
    ADDITION_AGRICULTURE_FOREST_PERMITTED = '1026-38'
    ADDITION_TIME_PERIOD_PERMITTED = '1040-30'
    ALLOWED_MASS_7_5_TONS = '1053-33'
    UNKNOWN = ''


@enum.unique
class TrafficSignIDZamunda(enum.Enum):  # default traffic sign IDs (similar to German IDs)
    WARNING_DANGER_SPOT = '101'
    WARNING_RIGHT_BEFORE_LEFT = '102'
    WARNING_STEEP_HILL_DOWNWARDS = '108'
    WARNING_SLIPPERY_ROAD = '114'
    WARNING_CONSTRUCTION_SITE = '123'
    WARNING_CROSSING_CYCLIST = '138'
    WARNING_ANIMAL_CROSSING_RIGHT = '142-10'
    RAILWAY = '201'
    YIELD = '205'
    STOP = '206'
    PRIORITY_OPPOSITE_DIRECTION = '208'
    TURN_RIGHT_AHEAD = '209-10'
    TURN_LEFT_AHEAD = '209-20'
    ROUNDABOUT = '215'
    ONEWAY_RIGHT = '220-10'
    ONEWAY_LEFT = '220-20'
    PRESCRIBED_PASSING_LEFT = '222-10'
    PRESCRIBED_PASSING_RIGHT = '222-20'
    BIKEWAY = '237'
    SIDEWALK = '239'
    PEDESTRIAN_ZONE_START = '242.1'
    PEDESTRIAN_ZONE_END = '242.2'
    BICYCLE_ROAD_START = '244.1'
    BICYCLE_ROAD_END = '244.2'
    BUSLANE = '245'
    BAN_ALL_VEHICLES = '250'
    BAN_CARS = '251'
    BAN_TRUCKS = '253'
    BAN_BICYCLE = '254'
    BAN_MOTORCYCLE = '255'
    BAN_BUS = '257-54'
    BAN_PEDESTRIAN = '259'
    BAN_CAR_TRUCK_BUS_MOTORCYCLE = '260'
    BAN_VEHICLES_CARRYING_DANGEROUS_GOODS = '261'
    MAX_WEIGHT = '262'
    MAX_WIDTH = '264'
    MAX_HEIGHT = '265'
    MAX_LENGTH = '266'
    NO_ENTRY = '267'
    U_TURN = '272'
    MAX_SPEED = '274'
    MAX_SPEED_ZONE_START = '274.1'
    MAX_SPEED_ZONE_END = '274.2'
    MIN_SPEED = '275'
    NO_OVERTAKING_START = '276'
    NO_OVERTAKING_TRUCKS_START = '277'
    MAX_SPEED_END = '278'
    NO_OVERTAKING_TRUCKS_END = '281'
    ALL_MAX_SPEED_AND_OVERTAKING_END = '282'
    RIGHT_OF_WAY = '301'
    PRIORITY = '306'
    PRIORITY_OVER_ONCOMING = '308'
    TOWN_SIGN = '310'
    TRAFFIC_CALMED_AREA_START = '325.1'
    TRAFFIC_CALMED_AREA_END = '325.2'
    TUNNEL = '327'
    INTERSTATE_START = '330.1'
    INTERSTATE_END = '330.2'
    HIGHWAY_START = '331.1'
    HIGHWAY_END = '331.2'
    EXIT_BUILT_UP = '333-21'
    EXIT_GENERAL = '333-22'
    PEDESTRIANS_CROSSING = '350'
    DEAD_END = '357'
    DIRECTION_SIGN_LEFT_SINGLE = '625-10'
    DIRECTION_SIGN_LEFT_SMALL = '625-11'
    DIRECTION_SIGN_LEFT_MEDIUM = '625-12'
    DIRECTION_SIGN_LEFT_LARGE = '625-13'
    DIRECTION_SIGN_RIGHT_SINGLE = '625-20'
    DIRECTION_SIGN_RIGHT_SMALL = '625-21'
    DIRECTION_SIGN_RIGHT_MEDIUM = '625-22'
    DIRECTION_SIGN_RIGHT_LARGE = '625-23'
    WARNING_PANEL_RIGHT = '626-10'
    WARNING_PANEL_LEFT = '626-20'
    WARNING_PANEL_STRAIGHT_BROAD = '626-30'
    WARNING_PANEL_STRAIGHT_HIGH = '626-31'
    GREEN_ARROW = '720'
    ADDITION_LEFT_DIRECTION_1 = '1000-10'
    ADDITION_LEFT_DIRECTION_DANGER_POINT = '1000-11'
    ADDITION_RIGHT_DIRECTION_1 = '1000-20'
    ADDITION_RIGHT_DIRECTION_DANGER_POINT = '1000-21'
    ADDITION_BOTH_DIRECTIONS_HORIZONTAL = '1000-30'
    ADDITION_BOTH_DIRECTIONS_VERTICAL = '1000-31'
    ADDITION_VALID_FOR_X_METERS = '1001-30'
    ADDITION_VALID_FOR_X_KILOMETERS = '1001-31'
    ADDITION_LEFT_TURNING_PRIORITY_WITH_OPPOSITE_RIGHT_YIELD = '1002-10'
    ADDITION_LEFT_TRAFFIC_PRIORITY_WITH_STRAIGHT_RIGHT_YIELD = '1002-11'
    ADDITION_LEFT_TURNING_PRIORITY_WITH_OPPOSITE_YIELD = '1002-12'
    ADDITION_LEFT_TURNING_PRIORITY_WITH_RIGHT_YIELD = '1002-13'
    ADDITION_LEFT_TRAFFIC_PRIORITY_WITH_STRAIGHT_YIELD = '1002-14'
    ADDITION_RIGHT_TURNING_PRIORITY_WITH_OPPOSITE_LEFT_YIELD = '1002-20'
    ADDITION_RIGHT_TRAFFIC_PRIORITY_WITH_STRAIGHT_LEFT_YIELD = '1002-21'
    ADDITION_RIGHT_TURNING_PRIORITY_WITH_OPPOSITE_YIELD = '1002-22'
    ADDITION_RIGHT_TURNING_PRIORITY_WITH_LEFT_YIELD = '1002-23'
    ADDITION_RIGHT_TRAFFIC_PRIORITY_WITH_STRAIGHT_YIELD = '1002-24'
    ADDITION_VALID_IN_X_METERS = '1004-30'
    ADDITION_VALID_IN_X_KILOMETERS = '1004-31'
    ADDITION_RESIDENTS_PERMITTED = '1020-30'
    ADDITION_BICYCLES_PERMITTED = '1022-10'
    ADDITION_CARS_PERMITTED = '1024-10'
    ADDITION_AGRICULTURE_PERMITTED = '1026-36'
    ADDITION_FOREST_PERMITTED = '1026-37'
    ADDITION_AGRICULTURE_FOREST_PERMITTED = '1026-38'
    ADDITION_TIME_PERIOD_PERMITTED = '1040-30'
    ALLOWED_MASS_7_5_TONS = '1053-33'
    UNKNOWN = ''


@enum.unique
class TrafficSignIDUsa(enum.Enum):
    MAX_SPEED = 'R2-1'
    U_TURN = 'R3-4'
    UNKNOWN = ''


@enum.unique
class TrafficSignIDChina(enum.Enum):
    MAX_SPEED = '274'  # TODO: change to actual ID
    UNKNOWN = ''


@enum.unique
class TrafficSignIDSpain(enum.Enum):
    MAX_SPEED = '274'  # TODO: change to actual ID
    UNKNOWN = ''


@enum.unique
class TrafficSignIDRussia(enum.Enum):
    MAX_SPEED = '274'  # TODO: change to actual ID
    UNKNOWN = ''  # TODO: add actual IDs


@enum.unique
class TrafficSignIDArgentina(enum.Enum):
    MAX_SPEED = '274'  # TODO: change to actual ID
    UNKNOWN = ''  # TODO: add actual IDs


@enum.unique
class TrafficSignIDBelgium(enum.Enum):
    MAX_SPEED = '274'  # TODO: change to actual ID
    UNKNOWN = ''  # TODO: add actual IDs


@enum.unique
class TrafficSignIDFrance(enum.Enum):
    MAX_SPEED = '274'  # TODO: change to actual ID
    UNKNOWN = ''  # TODO: add actual IDs


@enum.unique
class TrafficSignIDGreece(enum.Enum):
    MAX_SPEED = '274'  # TODO: change to actual ID
    UNKNOWN = ''  # TODO: add actual IDs


@enum.unique
class TrafficSignIDCroatia(enum.Enum):
    MAX_SPEED = '274'  # TODO: change to actual ID
    UNKNOWN = ''


@enum.unique
class TrafficSignIDItaly(enum.Enum):
    MAX_SPEED = '274'  # TODO: change to actual ID
    UNKNOWN = ''


@enum.unique
class TrafficSignIDPuertoRico(enum.Enum):
    MAX_SPEED = '274'  # TODO: change to actual ID
    UNKNOWN = ''  # TODO: add actual IDs


@enum.unique
class TrafficLightDirection(enum.Enum):
    """
    Enum for all the possible directions for a traffic signal
    """
    RIGHT = "right"
    STRAIGHT = "straight"
    LEFT = "left"
    LEFT_STRAIGHT = "leftStraight"
    STRAIGHT_RIGHT = "straightRight"
    LEFT_RIGHT = "leftRight"
    ALL = "all"


@enum.unique
class TrafficLightState(enum.Enum):
    """
    Enum for the possible types of traffic light in signals
    """
    RED = "red"
    YELLOW = "yellow"
    RED_YELLOW = "redYellow"
    GREEN = "green"
    INACTIVE = "inactive"


class TrafficSignElement:
    """ Class which represents a collection of traffic signs at one position"""
    def __init__(self, traffic_sign_element_id: Union[TrafficSignIDZamunda, TrafficSignIDUsa, TrafficSignIDSpain,
                                                      TrafficSignIDGermany, TrafficSignIDChina, TrafficSignIDRussia],
                 additional_values: List[str] = []):
        """

        :param traffic_sign_element_id: ID of traffic sign element (must be element of a traffic sign element enum)
        :param additional_values: list of additional values of a traffic sign element, e.g. velocity, time, city name
        """
        self._traffic_sign_element_id = traffic_sign_element_id
        self._additional_values = additional_values

    @property
    def traffic_sign_element_id(self) -> enum:
        return self._traffic_sign_element_id

    @property
    def additional_values(self) -> List[str]:
        return self._additional_values

    def __eq__(self, other: 'TrafficSignElement'):
        if self.traffic_sign_element_id == other.traffic_sign_element_id \
                and self.additional_values == other.additional_values:
            return True
        else:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return hash(str(self. _traffic_sign_element_id) + str(self.additional_values))

    def __str__(self):
        return f"Sign Element with id {self._traffic_sign_element_id} and values {self._additional_values} "

    def __repr__(self):
        return f"Sign Element with id {self._traffic_sign_element_id} and values {self._additional_values} "


class TrafficSign(IDrawable):
    """Class to represent a traffic sign"""

    def __init__(self, traffic_sign_id: int,
                 traffic_sign_elements: List[TrafficSignElement],
                 first_occurrence: Set[int], position: np.ndarray,
                 virtual: bool = False):
        """
        :param traffic_sign_id: ID of traffic sign
        :param traffic_sign_elements: list of traffic sign elements
        :param first_occurrence: lanelet ID where traffic sign first appears
        :param position: position of traffic sign
        :param virtual: boolean indicating if this traffic sign is also
        placed there in the real environment or it
        is added for other reasons (e.g., completeness of scenario)
        """
        self._traffic_sign_id = traffic_sign_id
        self._position = position
        self._traffic_sign_elements = traffic_sign_elements
        self._virtual = virtual
        self._first_occurrence = first_occurrence

    @property
    def traffic_sign_id(self) -> int:
        return self._traffic_sign_id

    @property
    def position(self) -> Union[None,np.ndarray]:
        return self._position

    @property
    def traffic_sign_elements(self) -> List[TrafficSignElement]:
        return self._traffic_sign_elements

    @property
    def virtual(self) -> bool:
        return self._virtual

    @property
    def first_occurrence(self) -> Set[int]:
        return self._first_occurrence

    def translate_rotate(self, translation: np.ndarray, angle: float):
        """
        This method translates and rotates a traffic sign

        :param translation: The translation given as [x_off,y_off] for the x and y translation
        :param angle: The rotation angle in radian (counter-clockwise defined)
        """

        assert is_real_number_vector(translation, 2), '<TrafficSign/translate_rotate>: argument translation is ' \
                                                      'not a vector of real ' \
                                                      'numbers of length 2.'
        assert is_real_number(
            angle), '<TrafficSign/translate_rotate>: argument angle must be a ' \
                    'scalar. ' \
                    'angle = %s' % angle
        assert is_valid_orientation(
                angle), '<TrafficSign/translate_rotate>: argument angle must ' \
                        'be ' \
                        'within the ' \
                        'interval [-2pi, 2pi]. angle = %s' % angle
        self._position = commonroad.geometry.transform.translate_rotate(
                np.array([self._position]), translation, angle)[0]

    def __str__(self):
        return f"Sign At {self._position} with {self._traffic_sign_elements} "

    def draw(self, renderer: IRenderer,
             draw_params: Union[ParamServer, dict, None] = None,
             call_stack: Optional[Tuple[str, ...]] = tuple()):
        renderer.draw_traffic_light_sign(self, draw_params, call_stack)


class TrafficLightCycleElement:
    """Class to represent a traffic light cycle"""
    def __init__(self, state: TrafficLightState, duration: int):
        """
        :param state: state of a traffic light cycle element
        :param duration: duration of traffic light cycle element
        """
        self._state = state
        self._duration = duration

    @property
    def state(self) -> TrafficLightState:
        return self._state

    @property
    def duration(self) -> int:
        return self._duration


class TrafficLight(IDrawable):
    """ Class to represent a traffic light"""

    def __init__(self, traffic_light_id: int,
                 cycle: List[TrafficLightCycleElement], position: np.ndarray,
                 time_offset: int = 0,
                 direction: TrafficLightDirection = TrafficLightDirection.ALL,
                 active: bool = True):
        """
        :param traffic_light_id: ID of traffic light
        :param cycle: list of traffic light cycle elements
        :param time_offset: offset of traffic light cycle
        :param position: position of traffic light
        :param direction: driving directions for which the traffic light is valid
        :param active: boolean indicating if traffic light is currently active
        """
        self._traffic_light_id = traffic_light_id
        if len(cycle) == 0:
            self._cycle = get_default_cycle()
        else:
            self._cycle = cycle
        self._time_offset = time_offset
        self._position = position
        self._direction = direction
        self._active = active

    @property
    def traffic_light_id(self) -> int:
        return self._traffic_light_id

    @property
    def cycle(self) -> List[TrafficLightCycleElement]:
        return self._cycle

    def get_state_at_time_step(self, time_step: int) -> TrafficLightState:
        time_step_mod = ((time_step - self.time_offset) % (self.cycle_init_timesteps[-1] - self.time_offset))\
                        + self.time_offset
        i_cycle = np.argmax(time_step_mod < self.cycle_init_timesteps) - 1
        return self.cycle[i_cycle].state

    @property
    def cycle_init_timesteps(self):
        if not hasattr(self, '_cycle_init_timesteps'):
            durations = [cycle_el.duration for cycle_el in self._cycle]
            self._cycle_init_timesteps = np.cumsum(durations) + self.time_offset
            self._cycle_init_timesteps = np.insert(self._cycle_init_timesteps, 0, self.time_offset)
        return self._cycle_init_timesteps

    @property
    def time_offset(self) -> int:
        return self._time_offset

    @property
    def position(self) -> np.ndarray:
        return self._position

    @property
    def direction(self) -> TrafficLightDirection:
        return self._direction

    @direction.setter
    def direction(self, direction: TrafficLightDirection):
        self._direction = direction

    @property
    def active(self) -> bool:
        return self._active

    def translate_rotate(self, translation: np.ndarray, angle: float):
        """
        This method translates and rotates a traffic light

        :param translation: The translation given as [x_off,y_off] for the x and y translation
        :param angle: The rotation angle in radian (counter-clockwise defined)
        """

        assert is_real_number_vector(translation,
                                     2), '<TrafficLight/translate_rotate>: ' \
                                         'argument translation is ' \
                                         'not a vector of real numbers of ' \
                                         'length 2.'
        assert is_real_number(
                angle), '<TrafficLight/translate_rotate>: argument angle must ' \
                        'be ' \
                        'a scalar. ' \
                        'angle = %s' % angle
        assert is_valid_orientation(
                angle), '<TrafficLight/translate_rotate>: argument angle must ' \
                        'be ' \
                        'within the ' \
                        'interval [-2pi, 2pi]. angle = %s' % angle
        self._position = commonroad.geometry.transform.translate_rotate(
                np.array([self._position]), translation, angle)[0]

    def draw(self, renderer: IRenderer,
             draw_params: Union[ParamServer, dict, None] = None,
             call_stack: Optional[Tuple[str, ...]] = tuple()):
        renderer.draw_traffic_light_sign(self, draw_params, call_stack)


def get_default_cycle():
    """
    Defines default traffic light cycle in case no cycle is provided

    _:returns traffic light cycle element
    """
    cycle = [(TrafficLightState.RED, 60),
             (TrafficLightState.RED_YELLOW, 10),
             (TrafficLightState.GREEN, 60),
             (TrafficLightState.YELLOW, 10)]
    cycle_element_list = [TrafficLightCycleElement(state[0], state[1]) for state in cycle]
    return cycle_element_list
