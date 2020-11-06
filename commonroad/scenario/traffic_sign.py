__author__ = "Sebastian Maierhofer"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["CAR@TUM"]
__version__ = "2020.3"
__maintainer__ = "Sebastian Maierhofer"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "Release"

import enum
from typing import List, Union, Set
import numpy as np

from commonroad.common.validity import *
import commonroad.geometry.transform


TRAFFIC_SIGN_VALIDITY_START = {'MIN_SPEED', 'MAX_SPEED', 'NO_OVERTAKING_START', 'TOWN_SIGN',
                               'BAN_CAR_TRUCK_BUS_MOTORCYCLE'}
LEFT_HAND_TRAFFIC = {'AUS', 'JPN', 'HKG', 'IND', 'JEY', 'IMN', 'IRL', 'JAM', 'KEN', 'MLT', 'MYS', 'NPL', 'NZL', 'ZAF',
                     'SGP', 'THA', 'GBR', 'IDN', 'MAC', 'PAK', 'CYP'}


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
class TrafficSignIDZamunda(enum.Enum):
    # default traffic sign IDs (similar to German IDs)
    DANGER_POINT = '101'
    RIGHT_BEFORE_LEFT = '102'
    SLIPPERY_ROAD = '114'
    CROSSING_CYCLIST = '138'
    YIELD = '205'
    STOP = '206'
    ROUNDABOUT = '215'
    PRESCRIBED_PASSING_LEFT = '222-10'
    PRESCRIBED_PASSING_RIGHT = '222-20'
    BAN_CAR_TRUCK_BUS_MOTORCYCLE = '260'
    U_TURN = '272'
    MAX_SPEED = '274'
    MIN_SPEED = '275'
    NO_OVERTAKING_START = '276'
    NO_OVERTAKING_TRUCKS_START = '277'
    NO_OVERTAKING_TRUCKS_END = '281'
    RIGHT_OF_WAY = '301'
    PRIORITY = '306'
    TOWN_SIGN = '310'
    HIGHWAY_START = '331.1'
    HIGHWAY_END = '331.2'
    EXIT_BUILT_UP = '333-21'
    EXIT_GENERAL = '333-22'
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
    LEFT_TURNING_PRIORITY_WITH_OPPOSITE_RIGHT_YIELD = '1002-10'
    LEFT_TRAFFIC_PRIORITY_WITH_STRAIGHT_RIGHT_YIELD = '1002-11'
    LEFT_TURNING_PRIORITY_WITH_OPPOSITE_YIELD = '1002-12'
    LEFT_TURNING_PRIORITY_WITH_RIGHT_YIELD = '1002-13'
    LEFT_TRAFFIC_PRIORITY_WITH_STRAIGHT_YIELD = '1002-14'
    RIGHT_TURNING_PRIORITY_WITH_OPPOSITE_LEFT_YIELD = '1002-20'
    RIGHT_TRAFFIC_PRIORITY_WITH_STRAIGHT_LEFT_YIELD = '1002-21'
    RIGHT_TURNING_PRIORITY_WITH_OPPOSITE_YIELD = '1002-22'
    RIGHT_TURNING_PRIORITY_WITH_LEFT_YIELD = '1002-23'
    RIGHT_TRAFFIC_PRIORITY_WITH_STRAIGHT_YIELD = '1002-24'
    ALLOWED_MASS_7_5_TONS = '1053-33'
    UNKNOWN = ''


@enum.unique
class TrafficSignIDGermany(enum.Enum):
    DANGER_POINT = '101'
    RIGHT_BEFORE_LEFT = '102'
    SLIPPERY_ROAD = '114'
    CROSSING_CYCLIST = '138'
    YIELD = '205'
    STOP = '206'
    ROUNDABOUT = '215'
    PRESCRIBED_PASSING_LEFT = '222-10'
    PRESCRIBED_PASSING_RIGHT = '222-20'
    BAN_CAR_TRUCK_BUS_MOTORCYCLE = '260'
    U_TURN = '272'
    MAX_SPEED = '274'
    MIN_SPEED = '275'
    NO_OVERTAKING_START = '276'
    NO_OVERTAKING_TRUCKS_START = '277'
    NO_OVERTAKING_TRUCKS_END = '281'
    RIGHT_OF_WAY = '301'
    PRIORITY = '306'
    TOWN_SIGN = '310'
    HIGHWAY_START = '331.1'
    HIGHWAY_END = '331.2'
    EXIT_BUILT_UP = '333-21'
    EXIT_GENERAL = '333-22'
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
    LEFT_TURNING_PRIORITY_WITH_OPPOSITE_RIGHT_YIELD = '1002-10'
    LEFT_TRAFFIC_PRIORITY_WITH_STRAIGHT_RIGHT_YIELD = '1002-11'
    LEFT_TURNING_PRIORITY_WITH_OPPOSITE_YIELD = '1002-12'
    LEFT_TURNING_PRIORITY_WITH_RIGHT_YIELD = '1002-13'
    LEFT_TRAFFIC_PRIORITY_WITH_STRAIGHT_YIELD = '1002-14'
    RIGHT_TURNING_PRIORITY_WITH_OPPOSITE_LEFT_YIELD = '1002-20'
    RIGHT_TRAFFIC_PRIORITY_WITH_STRAIGHT_LEFT_YIELD = '1002-21'
    RIGHT_TURNING_PRIORITY_WITH_OPPOSITE_YIELD = '1002-22'
    RIGHT_TURNING_PRIORITY_WITH_LEFT_YIELD = '1002-23'
    RIGHT_TRAFFIC_PRIORITY_WITH_STRAIGHT_YIELD = '1002-24'
    ALLOWED_MASS_7_5_TONS = '1053-33'
    UNKNOWN = ''


@enum.unique
class TrafficSignIDUsa(enum.Enum):
    MAX_SPEED = 'R2-1'
    U_TURN = 'R3-4'
    UNKNOWN = ''


@enum.unique
class TrafficSignIDChina(enum.Enum):
    UNKNOWN = ''


@enum.unique
class TrafficSignIDSpain(enum.Enum):
    MAX_SPEED = '274'  # TODO: change to actual ID
    UNKNOWN = ''


@enum.unique
class TrafficSignIDRussia(enum.Enum):
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
                 additional_values: List[str]):
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


class TrafficSign:
    """Class to represent a traffic sign"""
    def __init__(self, traffic_sign_id: int, traffic_sign_elements: List[TrafficSignElement],
                 first_occurrence: Set[int], position: np.ndarray, virtual: bool = False):
        """
        :param traffic_sign_id: ID of traffic sign
        :param traffic_sign_elements: list of traffic sign elements
        :param first_occurrence: lanelet ID where traffic sign first appears
        :param position: position of traffic sign
        :param virtual: boolean indicating if this traffic sign is also placed there in the real environment or it
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
                                                      'not a vector of real numbers of length 2.'
        assert is_real_number(angle), '<TrafficSign/translate_rotate>: argument angle must be a scalar. ' \
                                      'angle = %s' % angle
        assert is_valid_orientation(angle), '<TrafficSign/translate_rotate>: argument angle must be within the ' \
                                            'interval [-2pi, 2pi]. angle = %s' % angle
        self._position = commonroad.geometry.transform.translate_rotate(np.array([self._position]),
                                                                        translation, angle)[0]

    def __str__(self):
        return f"Sign At {self._position} with {self._traffic_sign_elements} "


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


class TrafficLight:
    """ Class to represent a traffic light"""
    def __init__(self, traffic_light_id: int, cycle: List[TrafficLightCycleElement], position: np.ndarray,
                 time_offset: int = 0, direction: TrafficLightDirection = TrafficLightDirection.ALL,
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

    @property
    def active(self) -> bool:
        return self._active

    def translate_rotate(self, translation: np.ndarray, angle: float):
        """
        This method translates and rotates a traffic light

        :param translation: The translation given as [x_off,y_off] for the x and y translation
        :param angle: The rotation angle in radian (counter-clockwise defined)
        """

        assert is_real_number_vector(translation, 2), '<TrafficLight/translate_rotate>: argument translation is ' \
                                                      'not a vector of real numbers of length 2.'
        assert is_real_number(angle), '<TrafficLight/translate_rotate>: argument angle must be a scalar. ' \
                                      'angle = %s' % angle
        assert is_valid_orientation(angle), '<TrafficLight/translate_rotate>: argument angle must be within the ' \
                                            'interval [-2pi, 2pi]. angle = %s' % angle
        self._position = commonroad.geometry.transform.translate_rotate(
            np.array([self._position]), translation, angle
        )[0]


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
