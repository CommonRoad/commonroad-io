import enum
import warnings
import numpy as np
from typing import Union, Set, List
from abc import ABC, abstractmethod

from commonroad.common.validity import is_valid_orientation, is_real_number_vector, is_real_number
from commonroad.geometry.shape import Shape
from commonroad.prediction.prediction import Prediction, Occupancy
from commonroad.scenario.trajectory import State


__author__ = "Stefanie Manzinger, Christian Pek"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["Priority Program SPP 1835 Cooperative Interacting Automobiles, BMW Group, KO-HAF"]
__version__ = "2020.2"
__maintainer__ = "Stefanie Manzinger"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "Released"


@enum.unique
class ObstacleRole(enum.Enum):
    """ Enum containing all possible obstacle roles defined in CommonRoad."""
    STATIC = "static"
    DYNAMIC = "dynamic"


@enum.unique
class ObstacleType(enum.Enum):
    """ Enum containing all possible obstacle types defined in CommonRoad."""
    UNKNOWN = "unknown"
    CAR = "car"
    TRUCK = "truck"
    BUS = "bus"
    BICYCLE = "bicycle"
    PEDESTRIAN = "pedestrian"
    PRIORITY_VEHICLE = "priorityVehicle"
    PARKED_VEHICLE = "parkedVehicle"
    CONSTRUCTION_ZONE = "constructionZone"
    TRAIN = "train"
    ROAD_BOUNDARY = "roadBoundary"
    MOTORCYCLE = "motorcycle"


class SignalState:
    """ A signal state is a boolean value indicating the activity of the signal source at a time step.
        The possible signal state elements are defined as slots:

        :ivar horn: boolean indicating activity of horn
        :ivar indicator_left: boolean indicating activity of left indicator
        :ivar indicator_right: boolean indicating activity of right indicator
        :ivar braking_lights: boolean indicating activity of braking lights
        :ivar hazard_warning_lights: boolean indicating activity of hazard warning lights
        :ivar flashing_blue_lights: boolean indicating activity of flashing blue lights (police, ambulance)
        :ivar time_step: the discrete time step. Exact values are given as integers, uncertain values are given as
              :class:`commonroad.common.util.Interval`
    """

    __slots__ = [
        'horn',
        'indicator_left',
        'indicator_right',
        'braking_lights',
        'hazard_warning_lights',
        'flashing_blue_lights',
        'time_step',
    ]

    def __init__(self, **kwargs):
        """ Elements of state vector are determined during runtime."""
        for (field, value) in kwargs.items():
            setattr(self, field, value)


class Obstacle(ABC):
    """ Superclass for dynamic and static obstacles holding common properties defined in CommonRoad."""

    def __init__(self, obstacle_id: int, obstacle_role: ObstacleRole,
                 obstacle_type: ObstacleType, obstacle_shape: Shape, initial_state: State,
                 initial_center_lanelet_ids: Union[None, Set[int]] = None,
                 initial_shape_lanelet_ids: Union[None, Set[int]] = None,
                 initial_signal_state: Union[None, SignalState] = None, signal_series: List[SignalState] = None):
        """
        :param obstacle_id: unique ID of the obstacle
        :param obstacle_role: obstacle role as defined in CommonRoad
        :param obstacle_type: obstacle type as defined in CommonRoad (e.g. PARKED_VEHICLE)
        :param obstacle_shape: occupied area of the obstacle
        :param initial_state: initial state of the obstacle
        :param initial_center_lanelet_ids: initial IDs of lanelets the obstacle center is on
        :param initial_shape_lanelet_ids: initial IDs of lanelets the obstacle shape is on
        :param initial_signal_state: initial signal state of obstacle
        :param signal_series: list of signal states over time
        """
        self.obstacle_id: int = obstacle_id
        self.obstacle_role: ObstacleRole = obstacle_role
        self.obstacle_type: ObstacleType = obstacle_type
        self.obstacle_shape: Shape = obstacle_shape
        self.initial_state: State = initial_state
        self.initial_center_lanelet_ids: Union[None, Set[int]] = initial_center_lanelet_ids
        self.initial_shape_lanelet_ids: Union[None, Set[int]] = initial_shape_lanelet_ids
        self.initial_signal_state: Union[None, SignalState] = initial_signal_state
        self.signal_series: List[SignalState] = signal_series

    @property
    def obstacle_id(self) -> int:
        """ Unique ID of the obstacle."""
        return self._obstacle_id

    @obstacle_id.setter
    def obstacle_id(self, obstacle_id: int):
        assert isinstance(obstacle_id, int), '<Obstacle/obstacle_id>: argument obstacle_id of wrong type.' \
                                             'Expected type: %s. Got type: %s.' % (int, type(obstacle_id))
        if not hasattr(self, '_obstacle_id'):
            self._obstacle_id = obstacle_id
        else:
            warnings.warn('<Obstacle/obstacle_id>: Obstacle ID is immutable.')

    @property
    def obstacle_role(self) -> ObstacleRole:
        """ Obstacle role as defined in CommonRoad."""
        return self._obstacle_role

    @obstacle_role.setter
    def obstacle_role(self, obstacle_role: ObstacleRole):
        assert isinstance(obstacle_role, ObstacleRole), '<Obstacle/obstacle_role>: argument obstacle_role of wrong ' \
                                                        'type. Expected type: %s. Got type: %s.' \
                                                        % (ObstacleRole, type(obstacle_role))
        if not hasattr(self, '_obstacle_role'):
            self._obstacle_role = obstacle_role
        else:
            warnings.warn('<Obstacle/obstacle_role>: Obstacle role is immutable.')

    @property
    def obstacle_type(self) -> ObstacleType:
        """ Obstacle type as defined in CommonRoad."""
        return self._obstacle_type

    @obstacle_type.setter
    def obstacle_type(self, obstacle_type: ObstacleType):
        assert isinstance(obstacle_type, ObstacleType), '<Obstacle/obstacle_type>: argument obstacle_type of wrong ' \
                                                        'type. Expected type: %s. Got type: %s.' \
                                                        % (ObstacleType, type(obstacle_type))
        if not hasattr(self, '_obstacle_type'):
            self._obstacle_type = obstacle_type
        else:
            warnings.warn('<Obstacle/obstacle_type>: Obstacle type is immutable.')

    @property
    def obstacle_shape(self) -> Shape:
        """ Obstacle shape as defined in CommonRoad."""
        return self._obstacle_shape

    @obstacle_shape.setter
    def obstacle_shape(self, shape: Shape):
        assert isinstance(shape,
                          (type(None), Shape)), '<Obstacle/obstacle_shape>: argument shape of wrong type. Expected ' \
                                                'type %s. Got type %s.' % (Shape, type(shape))

        if not hasattr(self, '_obstacle_shape'):
            self._obstacle_shape = shape
        else:
            warnings.warn('<Obstacle/obstacle_shape>: Obstacle shape is immutable.')

    @property
    def initial_state(self) -> State:
        """ Initial state of the obstacle, e.g., obtained through sensor measurements."""
        return self._initial_state

    @initial_state.setter
    def initial_state(self, initial_state: State):
        assert isinstance(initial_state, State), '<Obstacle/initial_state>: argument initial_state of wrong type. ' \
                                                 'Expected types: %s. Got type: %s.' % (State, type(initial_state))
        self._initial_state = initial_state

    @property
    def initial_center_lanelet_ids(self) -> Union[None, Set[int]]:
        """ Initial lanelets of obstacle center, e.g., obtained through localization."""
        return self._initial_center_lanelet_ids

    @initial_center_lanelet_ids.setter
    def initial_center_lanelet_ids(self, initial_center_lanelet_ids: Union[None, Set[int]]):
        assert isinstance(initial_center_lanelet_ids, (set, type(None))), \
            '<Obstacle/initial_center_lanelet_ids>: argument initial_lanelet_ids of wrong type. ' \
            'Expected types: %s, %s. Got type: %s.' % (set, type(None), type(initial_center_lanelet_ids))
        if initial_center_lanelet_ids is not None:
            for lanelet_id in initial_center_lanelet_ids:
                assert isinstance(lanelet_id, int), \
                    '<Obstacle/initial_center_lanelet_ids>: argument initial_lanelet of wrong type. ' \
                    'Expected types: %s. Got type: %s.' % (int, type(lanelet_id))
        self._initial_center_lanelet_ids = initial_center_lanelet_ids

    @property
    def initial_shape_lanelet_ids(self) -> Union[None, Set[int]]:
        """ Initial lanelets of obstacle shape, e.g., obtained through localization."""
        return self._initial_shape_lanelet_ids

    @initial_shape_lanelet_ids.setter
    def initial_shape_lanelet_ids(self, initial_shape_lanelet_ids: Union[None, Set[int]]):
        assert isinstance(initial_shape_lanelet_ids, (set, type(None))), \
            '<Obstacle/initial_shape_lanelet_ids>: argument initial_lanelet_ids of wrong type. ' \
            'Expected types: %s, %s. Got type: %s.' % (set, type(None), type(initial_shape_lanelet_ids))
        if initial_shape_lanelet_ids is not None:
            for lanelet_id in initial_shape_lanelet_ids:
                assert isinstance(lanelet_id, int), \
                    '<Obstacle/initial_shape_lanelet_ids>: argument initial_lanelet of wrong type. ' \
                    'Expected types: %s. Got type: %s.' % (int, type(lanelet_id))
        self._initial_shape_lanelet_ids = initial_shape_lanelet_ids

    @property
    def initial_signal_state(self) -> SignalState:
        """ Signal state as defined in CommonRoad."""
        return self._initial_signal_state

    @initial_signal_state.setter
    def initial_signal_state(self, initial_signal_state: SignalState):
        assert isinstance(initial_signal_state, (SignalState, type(None))), '<Obstacle/initial_signal_state>: ' \
                                                              'argument initial_signal_state of wrong ' \
                                                              'type. Expected types: %s, %s. Got type: %s.' \
                                                              % (SignalState, type(None), type(initial_signal_state))
        if not hasattr(self, '_initial_signal_state'):
            self._initial_signal_state = initial_signal_state
        else:
            warnings.warn('<Obstacle/initial_signal_state>: Initial obstacle signal state is immutable.')

    @property
    def signal_series(self) -> List[SignalState]:
        """ Signal series as defined in CommonRoad."""
        return self._signal_series

    @signal_series.setter
    def signal_series(self, signal_series: List[SignalState]):
        assert isinstance(signal_series, (list, type(None))), '<Obstacle/initial_signal_state>: ' \
                                                              'argument initial_signal_state of wrong ' \
                                                              'type. Expected types: %s, %s. Got type: %s.' \
                                                              % (list, type(None), type(signal_series))
        if not hasattr(self, '_signal_series'):
            self._signal_series = signal_series
        else:
            warnings.warn('<Obstacle/signal_series>: Obstacle signal series is immutable.')

    @abstractmethod
    def occupancy_at_time(self, time_step: int) -> Union[None, Occupancy]:
        pass

    @abstractmethod
    def translate_rotate(self, translation: np.ndarray, angle: float):
        pass

    def signal_state_at_time_step(self, time_step: int) -> Union[SignalState, None]:
        """
        Extracts signal state at a time step

        :param time_step: time step of interest
        :returns: signal state or None if time step does not exist
        """
        if self.initial_signal_state is not None and time_step == self.initial_signal_state.time_step:
            return self.initial_signal_state
        elif self.signal_series is None:
            return None
        else:
            for state in self.signal_series:
                if state.time_step == time_step:
                    return state

        return None


class StaticObstacle(Obstacle):
    """ Class representing static obstacles as defined in CommonRoad."""

    def __init__(self, obstacle_id: int, obstacle_type: ObstacleType,
                 obstacle_shape: Shape, initial_state: State, initial_center_lanelet_ids: Union[None, Set[int]] = None,
                 initial_shape_lanelet_ids: Union[None, Set[int]] = None,
                 initial_signal_state: Union[None, SignalState] = None, signal_series: List[SignalState] = None):
        """
            :param obstacle_id: unique ID of the obstacle
            :param obstacle_type: type of obstacle (e.g. PARKED_VEHICLE)
            :param obstacle_shape: shape of the static obstacle
            :param initial_state: initial state of the static obstacle
            :param initial_center_lanelet_ids: initial IDs of lanelets the obstacle center is on
            :param initial_shape_lanelet_ids: initial IDs of lanelets the obstacle shape is on
            :param initial_signal_state: initial signal state of static obstacle
            :param signal_series: list of signal states over time
        """
        Obstacle.__init__(self, obstacle_id=obstacle_id, obstacle_role=ObstacleRole.STATIC,
                          obstacle_type=obstacle_type, obstacle_shape=obstacle_shape, initial_state=initial_state,
                          initial_center_lanelet_ids=initial_center_lanelet_ids,
                          initial_shape_lanelet_ids=initial_shape_lanelet_ids,
                          initial_signal_state=initial_signal_state, signal_series=signal_series)

    def translate_rotate(self, translation: np.ndarray, angle: float):
        """ First translates the static obstacle, then rotates the static obstacle around the origin.

            :param translation: translation vector [x_off, y_off] in x- and y-direction
            :param angle: rotation angle in radian (counter-clockwise)
        """
        assert is_real_number_vector(translation, 2), '<StaticObstacle/translate_rotate>: argument translation is ' \
                                                      'not a vector of real numbers of length 2.'
        assert is_real_number(angle), '<StaticObstacle/translate_rotate>: argument angle must be a scalar. ' \
                                      'angle = %s' % angle
        assert is_valid_orientation(angle), '<StaticObstacle/translate_rotate>: argument angle must be within the ' \
                                            'interval [-2pi, 2pi]. angle = %s' % angle
        self.initial_state = self._initial_state.translate_rotate(translation, angle)

    def occupancy_at_time(self, time_step: int) -> Occupancy:
        """
        Returns the predicted occupancy of the obstacle at a specific time step.

        :param time_step: discrete time step
        :return: occupancy of the static obstacle at time step
        """
        shape = self.obstacle_shape.rotate_translate_local(
            self.initial_state.position, self.initial_state.orientation)
        return Occupancy(time_step=time_step, shape=shape)

    def __str__(self):
        obs_str = 'Static Obstacle:\n'
        obs_str += '\nid: {}'.format(self.obstacle_id)
        obs_str += '\ninitial state: {}'.format(self.initial_state)
        return obs_str


class DynamicObstacle(Obstacle):
    """ Class representing dynamic obstacles as defined in CommonRoad. Each dynamic obstacle has stored its predicted
    movement in future time steps.
    """

    def __init__(self, obstacle_id: int, obstacle_type: ObstacleType,
                 obstacle_shape: Shape, initial_state: State,
                 prediction: Union[None, Prediction] = None, initial_center_lanelet_ids: Union[None, Set[int]] = None,
                 initial_shape_lanelet_ids: Union[None, Set[int]] = None,
                 initial_signal_state: Union[None, SignalState] = None, signal_series: List[SignalState] = None):
        """
            :param obstacle_id: unique ID of the obstacle
            :param obstacle_type: type of obstacle (e.g. PARKED_VEHICLE)
            :param obstacle_shape: shape of the static obstacle
            :param initial_state: initial state of the static obstacle
            :param prediction: predicted movement of the dynamic obstacle
            :param initial_center_lanelet_ids: initial IDs of lanelets the obstacle center is on
            :param initial_shape_lanelet_ids: initial IDs of lanelets the obstacle shape is on
            :param initial_signal_state: initial signal state of static obstacle
            :param signal_series: list of signal states over time
        """
        Obstacle.__init__(self, obstacle_id=obstacle_id, obstacle_role=ObstacleRole.DYNAMIC,
                          obstacle_type=obstacle_type, obstacle_shape=obstacle_shape, initial_state=initial_state,
                          initial_center_lanelet_ids=initial_center_lanelet_ids,
                          initial_shape_lanelet_ids=initial_shape_lanelet_ids,
                          initial_signal_state=initial_signal_state, signal_series=signal_series)
        self.prediction: Prediction = prediction

    @property
    def prediction(self) -> Union[Prediction, None]:
        """ Prediction describing the movement of the dynamic obstacle over time."""
        return self._prediction

    @prediction.setter
    def prediction(self, prediction: Union[Prediction, None]):
        assert isinstance(prediction, (Prediction, type(None))), '<DynamicObstacle/prediction>: argument prediction ' \
                                                                 'of wrong type. Expected types: %s, %s. Got type: ' \
                                                                 '%s.' % (Prediction, type(None), type(prediction))
        self._prediction = prediction

    def occupancy_at_time(self, time_step: int) -> Union[None, Occupancy]:
        """
        Returns the predicted occupancy of the obstacle at a specific time step.

        :param time_step: discrete time step
        :return: predicted occupancy of the obstacle at time step
        """
        occupancy = None

        if time_step == self.initial_state.time_step:
            shape = self.obstacle_shape.rotate_translate_local(
                self.initial_state.position, self.initial_state.orientation)
            occupancy = Occupancy(time_step, shape)
        elif time_step > self.initial_state.time_step and self._prediction is not None:
            occupancy = self._prediction.occupancy_at_time_step(time_step)
        return occupancy

    def translate_rotate(self, translation: np.ndarray, angle: float):
        """ First translates the dynamic obstacle, then rotates the dynamic obstacle around the origin.

            :param translation: translation vector [x_off, y_off] in x- and y-direction
            :param angle: rotation angle in radian (counter-clockwise)
        """
        assert is_real_number_vector(translation, 2), '<DynamicObstacle/translate_rotate>: argument translation is ' \
                                                      'not a vector of real numbers of length 2.'
        assert is_real_number(angle), '<DynamicObstacle/translate_rotate>: argument angle must be a scalar. ' \
                                      'angle = %s' % angle
        assert is_valid_orientation(angle), '<DynamicObstacle/translate_rotate>: argument angle must be within the ' \
                                            'interval [-2pi, 2pi]. angle = %s' % angle
        if self._prediction is not None:
            self.prediction.translate_rotate(translation, angle)
        self._initial_state = self._initial_state.translate_rotate(translation, angle)

    def __str__(self):
        obs_str = 'Dynamic Obstacle:\n'
        obs_str += '\nid: {}'.format(self.obstacle_id)
        obs_str += '\ninitial state: {}'.format(self.initial_state)
        return obs_str
