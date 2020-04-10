__author__ = "Sebastian Maierhofer"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["CAR@TUM"]
__version__ = "2020.2"
__maintainer__ = "Sebastian Maierhofer"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "Release"

import enum
from typing import List, Union, Set
import numpy as np


TRAFFIC_SIGN_VALIDITY_START = {'MIN_SPEED', 'MAX_SPEED', 'NO_OVERTAKING_START', 'CITY_SIGN'}
LEFT_HAND_TRAFFIC = {'AUS', 'JPN', 'HKG', 'IND', 'JEY', 'IMN', 'IRL', 'JAM', 'KEN', 'MLT', 'MYS', 'NPL', 'NZL', 'ZAF',
                     'SGP', 'THA', 'GBR', 'IDN', 'MAC', 'PAK', 'CYP'}


@enum.unique
class SupportedTrafficSignCountry(enum.Enum):
    GERMANY = 'DEU'
    USA = "USA"
    CHINA = "CHN"
    SPAIN = "ESP"
    RUSSIA = "RUS"
    ZAMUNDA = "ZAM"  # default if


@enum.unique
class TrafficSignIDZamunda(enum.Enum):
    # default traffic sign IDs (similar to German IDs)
    MIN_SPEED = '275'
    MAX_SPEED = '274'
    NO_OVERTAKING_START = '276'
    TOWN_SIGN = '310'
    YIELD = '205'
    STOP = '206'
    PRIORITY = '306'
    RIGHT_OF_WAY = '301'
    GREEN_ARROW = '720'
    RIGHT_BEFORE_LEFT = '102'
    LEFT_TURNING_PRIORITY_WITH_OPPOSITE_RIGHT_YIELD = '1002-10'
    LEFT_TURNING_PRIORITY_WITH_OPPOSITE_YIELD = '1002-12'
    LEFT_TURNING_PRIORITY_WITH_RIGHT_YIELD = '1002-13'
    RIGHT_TURNING_PRIORITY_WITH_OPPOSITE_LEFT_YIELD = '1002-20'
    RIGHT_TURNING_PRIORITY_WITH_OPPOSITE_YIELD = '1002-22'
    RIGHT_TURNING_PRIORITY_WITH_LEFT_YIELD = '1002-23'
    LEFT_TRAFFIC_PRIORITY_WITH_STRAIGHT_RIGHT_YIELD = '1002-11'
    LEFT_TRAFFIC_PRIORITY_WITH_STRAIGHT_YIELD = '1002-14'
    RIGHT_TRAFFIC_PRIORITY_WITH_STRAIGHT_LEFT_YIELD = '1002-21'
    RIGHT_TRAFFIC_PRIORITY_WITH_STRAIGHT_YIELD = '1002-24'
    UNKNOWN = ''


@enum.unique
class TrafficSignIDGermany(enum.Enum):
    MIN_SPEED = '275'
    MAX_SPEED = '274'
    NO_OVERTAKING_START = '276'
    TOWN_SIGN = '310'
    YIELD = '205'
    STOP = '206'
    PRIORITY = '306'
    RIGHT_OF_WAY = '301'
    GREEN_ARROW = '720'
    RIGHT_BEFORE_LEFT = '102'
    LEFT_TURNING_PRIORITY_WITH_OPPOSITE_RIGHT_YIELD = '1002-10'
    LEFT_TURNING_PRIORITY_WITH_OPPOSITE_YIELD = '1002-12'
    LEFT_TURNING_PRIORITY_WITH_RIGHT_YIELD = '1002-13'
    RIGHT_TURNING_PRIORITY_WITH_OPPOSITE_LEFT_YIELD = '1002-20'
    RIGHT_TURNING_PRIORITY_WITH_OPPOSITE_YIELD = '1002-22'
    RIGHT_TURNING_PRIORITY_WITH_LEFT_YIELD = '1002-23'
    LEFT_TRAFFIC_PRIORITY_WITH_STRAIGHT_RIGHT_YIELD = '1002-11'
    LEFT_TRAFFIC_PRIORITY_WITH_STRAIGHT_YIELD = '1002-14'
    RIGHT_TRAFFIC_PRIORITY_WITH_STRAIGHT_LEFT_YIELD = '1002-21'
    RIGHT_TRAFFIC_PRIORITY_WITH_STRAIGHT_YIELD = '1002-24'
    UNKNOWN = ''


@enum.unique
class TrafficSignIDUsa(enum.Enum):
    MAX_SPEED = 'R2-1'
    UNKNOWN = ''


@enum.unique
class TrafficSignIDChina(enum.Enum):
    MAX_SPEED = '274'
    UNKNOWN = ''


@enum.unique
class TrafficSignIDSpain(enum.Enum):
    MAX_SPEED = '274'
    UNKNOWN = ''


@enum.unique
class TrafficSignIDRussia(enum.Enum):
    MAX_SPEED = '274'
    UNKNOWN = ''


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
