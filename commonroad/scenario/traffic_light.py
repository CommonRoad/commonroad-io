import enum
from typing import List
import numpy as np

import commonroad.geometry.transform
from commonroad.visualization.draw_params import OptionalSpecificOrAllDrawParams, TrafficLightParams
from commonroad.visualization.drawable import IDrawable
from commonroad.visualization.renderer import IRenderer
from commonroad.common.validity import *


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


class TrafficLightCycleElement:
    """Class to represent a traffic light cycle"""

    def __init__(self, state: TrafficLightState, duration: int):
        """
        :param state: state of a traffic light cycle element
        :param duration: duration of traffic light cycle element
        """
        self._state = state
        self._duration = duration

    def __eq__(self, other):
        if not isinstance(other, TrafficLightCycleElement):
            warnings.warn(f"Inequality between TrafficLightCycleElement {repr(self)} and different type {type(other)}")
            return False

        if self._state == other.state and self._duration == other.duration:
            return True

        warnings.warn(f"Inequality of TrafficLightCycleElement {repr(self)} and the other one {repr(other)}")
        return False

    def __hash__(self):
        return hash((self._state, self._duration))

    def __str__(self):
        return f"TrafficLightCycleElement with state {self._state} and duration {self._duration}"

    def __repr__(self):
        return f"TrafficLightCycleElement(state={self._state}, duration={self._duration})"

    @property
    def state(self) -> TrafficLightState:
        return self._state

    @state.setter
    def state(self, state: TrafficLightState):
        self._state = state

    @property
    def duration(self) -> int:
        return self._duration

    @duration.setter
    def duration(self, duration: int):
        self._duration = duration


class TrafficLight(IDrawable):
    """ Class to represent a traffic light"""

    def __init__(self, traffic_light_id: int, cycle: List[TrafficLightCycleElement] = None, position: np.ndarray = None,
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
        if cycle is None:
            self._cycle = []
        else:
            self._cycle = cycle
        self._time_offset = time_offset
        self._position = position
        self._direction = direction
        self._active = active

    def __eq__(self, other):
        if not isinstance(other, TrafficLight):
            warnings.warn(f"Inequality between TrafficLight {repr(self)} and different type {type(other)}")
            return False

        position_string = None if self._position is None else \
            np.array2string(np.around(self._position.astype(float), 10), precision=10)
        position_other_string = None if other._position is None else \
            np.array2string(np.around(other.position.astype(float), 10), precision=10)

        if self._traffic_light_id == other.traffic_light_id and set(self._cycle) == set(other.cycle) \
                and self._time_offset == other.time_offset and position_string == position_other_string \
                and self._direction == other.direction and self._active == other.active:
            return True

        warnings.warn(f"Inequality of TrafficLight {repr(self)} and the other one {repr(other)}")
        return False

    def __hash__(self):
        position_string = None if self._position is None else \
            np.array2string(np.around(self._position.astype(float), 10), precision=10)
        return hash((self._traffic_light_id, frozenset(self._cycle), self._time_offset, position_string,
                     self._direction, self._active))

    def __str__(self):
        return f"TrafficLight with id {self._traffic_light_id} placed at {self._position}"

    def __repr__(self):
        return f"TrafficLight(traffic_light_id={self._traffic_light_id}, cycle={repr(self._cycle)}, " \
               f"time_offset={self._time_offset}, " \
               f"position={None if self._position is None else self._position.tolist()}, " \
               f"direction={self._direction}, active={self._active})"

    @property
    def traffic_light_id(self) -> int:
        return self._traffic_light_id

    @traffic_light_id.setter
    def traffic_light_id(self, traffic_light_id: int):
        self._traffic_light_id = traffic_light_id

    @property
    def cycle(self) -> Union[None, List[TrafficLightCycleElement]]:
        return self._cycle

    @cycle.setter
    def cycle(self, cycle: Union[None, List[TrafficLightCycleElement]]):
        self._cycle = cycle

    def get_state_at_time_step(self, time_step: int) -> TrafficLightState:
        time_step_mod = ((time_step - self.time_offset) % (
                    self.cycle_init_timesteps[-1] - self.time_offset)) + self.time_offset
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

    @time_offset.setter
    def time_offset(self, time_offset: int):
        self._time_offset = time_offset

    @property
    def position(self) -> np.ndarray:
        return self._position

    @position.setter
    def position(self, position: np.ndarray):
        self._position = position

    @property
    def direction(self) -> TrafficLightDirection:
        return self._direction

    @direction.setter
    def direction(self, direction: TrafficLightDirection):
        self._direction = direction

    @property
    def active(self) -> bool:
        return self._active

    @active.setter
    def active(self, active: bool):
        self._active = active

    def translate_rotate(self, translation: np.ndarray, angle: float):
        """
        This method translates and rotates a traffic light

        :param translation: The translation given as [x_off,y_off] for the x and y translation
        :param angle: The rotation angle in radian (counter-clockwise defined)
        """

        assert is_real_number_vector(translation, 2), '<TrafficLight/translate_rotate>: ' \
                                                      'argument translation is ' \
                                                      'not a vector of real numbers of ' \
                                                      'length 2.'
        assert is_real_number(angle), '<TrafficLight/translate_rotate>: argument angle must ' \
                                      'be ' \
                                      'a scalar. ' \
                                      'angle = %s' % angle
        assert is_valid_orientation(angle), '<TrafficLight/translate_rotate>: argument angle must ' \
                                            'be ' \
                                            'within the ' \
                                            'interval [-2pi, 2pi]. angle = %s' % angle
        self._position = commonroad.geometry.transform.translate_rotate(np.array([self._position]), translation, angle)[
            0]

    def draw(self, renderer: IRenderer, draw_params: OptionalSpecificOrAllDrawParams[TrafficLightParams] = None):
        renderer.draw_traffic_light_sign(self, draw_params)
