__author__ = "Behtarin Ferdousi"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = []
__version__ = "2020.1"
__maintainer__ = "Behtarin Ferdousi"
__email__ = "commonroad@in.tum.de"
__status__ = "Development"

import enum
from typing import List
import numpy as np


@enum.unique
class SupportedTrafficSignCountry(enum.Enum):
    GERMANY = 'DEU'


@enum.unique
class TrafficSignIDGermany(enum.Enum):
    MINSPEED = '275'
    MAXSPEED = '274'
    OVERTAKING = '276'
    CITYLIMIT = '310'
    GIVEWAY = '205'
    STOP = '206'
    UNKNOWN = ''


@enum.unique
class TrafficSignIDUsa(enum.Enum):
    MAXSPEED = '274'
    UNKNOWN = ''


@enum.unique
class TrafficSignIDChina(enum.Enum):
    MAXSPEED = '274'
    UNKNOWN = ''


@enum.unique
class TrafficSignIDSpain(enum.Enum):
    MAXSPEED = '274'
    UNKNOWN = ''


@enum.unique
class TrafficSignIDRussia(enum.Enum):
    MAXSPEED = '274'
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
    """ Class to represent each traffic sign element"""
    def __init__(self, traffic_sign_element_id: str, additional_values: List[str]):
        """
        :param traffic_sign_element_id: ID of traffic sign element (must be element of a traffic sign element enum)
        :param additional_values: list of additional values of a traffic sign element, e.g. velocity, time, city name
        """
        self._traffic_sign_element_id = traffic_sign_element_id
        self._additional_values = additional_values

    @property
    def traffic_sign_element_id(self) -> str:
        return self._traffic_sign_element_id

    @property
    def additional_values(self) -> List[str]:
        return self._additional_values

    def __str__(self):
        return f"Sign Element with id {self._traffic_sign_element_id} and values {self._additional_values} "

    def __repr__(self):
        return f"Sign Element with id {self._traffic_sign_element_id} and values {self._additional_values} "


class TrafficSign:
    """Class to represent traffic sign"""
    def __init__(self, traffic_sign_id: int, traffic_sign_elements: List[TrafficSignElement],
                 position: np.ndarray = None, virtual: bool = False):
        """
        :param traffic_sign_id: ID of traffic sign
        :param traffic_sign_elements: list of traffic sign elements
        :param position: position of traffic sign
        :param virtual: boolean indicating if this traffic sign is also placed there in the real environment or it
        is added for other reasons (e.g., completeness of scenario)
        """
        self._traffic_sign_id = traffic_sign_id
        self._position = position
        self._traffic_sign_elements = traffic_sign_elements
        self._virtual = virtual

    @property
    def traffic_sign_id(self) -> int:
        return self._traffic_sign_id

    @property
    def position(self) -> np.ndarray:
        return self._position

    @property
    def traffic_sign_elements(self) -> List[TrafficSignElement]:
        return self._traffic_sign_elements

    @property
    def virtual(self) -> bool:
        return self._virtual

    def __str__(self):
        return f"Sign At {self._position} with {self._traffic_sign_elements} "


class TrafficLightCycleElement:
    """Class to represent traffic light cycle"""
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
    """ Class to represent Traffic Light"""
    def __init__(self, traffic_light_id: int, cycle: List[TrafficLightCycleElement], time_offset: int = 0,
                 position: np.ndarray = None, direction: TrafficLightDirection = TrafficLightDirection.ALL,
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
