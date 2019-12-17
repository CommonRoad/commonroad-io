__author__ = "Behtarin Ferdousi"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = []
__version__ = "2019.1"
__maintainer__ = "Behtarin Ferdousi"
__email__ = "commonroad@in.tum.de"
__status__ = "Development"

import enum
from typing import List, Union

from shapely.geometry import Point


@enum.unique
class TrafficSignID(enum.Enum):
    MAXSPEED = '274'
    OVERTAKING = '276'
    CITYLIMIT = '310'
    GIVEWAY = '205'
    STOP = '206'
    UNKNOWN = ''


@enum.unique
class Direction(enum.Enum):
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
    def __init__(self, traffic_sign_id: str,
                 additional_values: List[Union[int, float, str]]):
        self.traffic_sign_id = traffic_sign_id
        self.additional_values = additional_values

    def __str__(self):
        return f"Sign Element with id {self.traffic_sign_id} and values {self.additional_values} "

    def __repr__(self):
        return f"Sign Element with id {self.traffic_sign_id} and values {self.additional_values} "


class TrafficSign:
    """Class to represent traffic sign"""
    def __init__(self, id: int,
                 traffic_sign_elements: List[TrafficSignElement],
                 position: Point = None,
                 virtual: bool = False):
        self.id = id
        self.position = position
        self.traffic_sign_elements = traffic_sign_elements
        self.virtual = virtual

    def __str__(self):
        return f"Sign At {self.position} with {self.traffic_sign_elements} "


class TrafficLightCycleElement:
    """Class to represent traffic light cycle"""
    def __init__(self,
                 state : TrafficLightState,
                 duration: float):
        self.state = state
        self.duration = duration


class TrafficLight:
    """ Class to represent Traffic Light"""
    def __init__(self, id: int,
                 cycle: List[TrafficLightCycleElement],
                 offset: int = 0,
                 position: Point = None,
                 direction: Direction = Direction.ALL,
                 active: bool = True):
        self.id = id
        self.cycle = cycle
        self.offset = offset
        self.position = position
        self.direction = direction
        self.active = active




