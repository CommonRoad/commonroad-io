import enum
import warnings
import math
import numpy as np
from typing import Union, Set, List, Dict, Optional, Tuple
from abc import abstractmethod


from commonroad.common.validity import is_valid_orientation, is_real_number_vector, is_real_number
from commonroad.geometry.shape import Shape, \
    Rectangle, \
    Circle, \
    Polygon, \
    ShapeGroup, \
    occupancy_shape_from_state, \
    shape_group_occupancy_shape_from_state
from commonroad.prediction.prediction import Prediction, Occupancy, SetBasedPrediction, TrajectoryPrediction
from commonroad.scenario.state import TraceState, InitialState, State
from commonroad.visualization.drawable import IDrawable
from commonroad.visualization.renderer import IRenderer
from commonroad.visualization.draw_params import OptionalSpecificOrAllDrawParams, PhantomObstacleParams, \
    EnvironmentObstacleParams, DynamicObstacleParams, StaticObstacleParams


@enum.unique
class ObstacleRole(enum.Enum):
    """ Enum containing all possible obstacle roles defined in CommonRoad."""
    STATIC = "static"
    DYNAMIC = "dynamic"
    ENVIRONMENT = "environment"
    Phantom = "phantom"


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
    TAXI = "taxi"
    BUILDING = "building"
    PILLAR = "pillar"
    MEDIAN_STRIP = "median_strip"


class MetaInformationState:
    """
    Class that keeps the meta information state of an obstacle.
    It freely allows to set attributes on a dynamic obstacle.
    """
    def __init__(self, meta_data_str: Dict[str, str] = None, meta_data_int: Dict[str, int] = None,
                 meta_data_float: Dict[str, float] = None, meta_data_bool: Dict[str, bool] = None):
        """
        :param meta_data_str: dictionary of a key and a string
        :param meta_data_int: dictionary of a key and an int
        :param meta_data_float: dictionary of a key and a float
        :param meta_data_bool: dictionary of a key and a bool
        """
        self._meta_data_str = meta_data_str
        self._meta_data_int = meta_data_int
        self._meta_data_float = meta_data_float
        self._meta_data_bool = meta_data_bool

    @property
    def meta_data_str(self) -> Dict[str, str]:
        """ Dictionary of a key and a string. """
        return self._meta_data_str

    @meta_data_str.setter
    def meta_data_str(self, meta_data_str: Dict[str, str]):
        assert isinstance(meta_data_str, Dict), '<MetaInformationState/meta_data_str>: Provided meta_data_str ' \
                                                'is not valid! id={}'.format(meta_data_str)
        self._meta_data_str = meta_data_str

    @property
    def meta_data_int(self) -> Dict[str, int]:
        """ Dictionary of a key and an int. """
        return self._meta_data_int

    @meta_data_int.setter
    def meta_data_int(self, meta_data_int: Dict[str, int]):
        assert isinstance(meta_data_int, Dict), '<MetaInformationState/meta_data_int>: Provided meta_data_int ' \
                                                'is not valid! id={}'.format(meta_data_int)
        self._meta_data_int = meta_data_int

    @property
    def meta_data_float(self) -> Dict[str, float]:
        """ Dictionary of a key and a float. """
        return self._meta_data_float

    @meta_data_float.setter
    def meta_data_float(self, meta_data_float: Dict[str, float]):
        assert isinstance(meta_data_float, Dict), '<MetaInformationState/meta_data_float>: Provided meta_data_float ' \
                                                'is not valid! id={}'.format(meta_data_float)
        self._meta_data_float = meta_data_float

    @property
    def meta_data_bool(self) -> Dict[str, bool]:
        """ Dictionary of a key and a bool. """
        return self._meta_data_bool

    @meta_data_bool.setter
    def meta_data_bool(self, meta_data_bool: Dict[str, bool]):
        assert isinstance(meta_data_bool, Dict), '<MetaInformationState/meta_data_bool>: Provided meta_data_bool ' \
                                                'is not valid! id={}'.format(meta_data_bool)
        self._meta_data_bool = meta_data_bool


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

    def __eq__(self, other):
        if not isinstance(other, SignalState):
            warnings.warn(f"Inequality between SignalState {repr(self)} and different type {type(other)}")
            return False

        for attr in SignalState.__slots__:
            value = None
            value_other = None

            has_attr = hasattr(self, attr)
            if has_attr:
                value = getattr(self, attr)

            has_attr_other = hasattr(other, attr)
            if has_attr_other:
                value_other = getattr(other, attr)

            if has_attr != has_attr_other or value != value_other:
                return False

        return True

    def __hash__(self):
        values = set()
        for attr in SignalState.__slots__:
            if hasattr(self, attr):
                values.add(getattr(self, attr))

        return hash(frozenset(values))


class Obstacle(IDrawable):
    """ Superclass for dynamic and static obstacles holding common properties
    defined in CommonRoad."""

    def __init__(self, obstacle_id: int, obstacle_role: ObstacleRole,
                 obstacle_type: ObstacleType, obstacle_shape: Shape,
                 initial_state: InitialState = None,
                 initial_center_lanelet_ids: Union[None, Set[int]] = None,
                 initial_shape_lanelet_ids: Union[None, Set[int]] = None,
                 initial_signal_state: Union[None, SignalState] = None,
                 signal_series: List[SignalState] = None):
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
        self._initial_occupancy_shape: Union[None, Shape] = None
        self.obstacle_id: int = obstacle_id
        self.obstacle_role: ObstacleRole = obstacle_role
        self.obstacle_type: ObstacleType = obstacle_type
        self.obstacle_shape: Shape = obstacle_shape
        self.initial_state: InitialState = initial_state
        self.initial_center_lanelet_ids: Union[None, Set[int]] = initial_center_lanelet_ids
        self.initial_shape_lanelet_ids: Union[None, Set[int]] = initial_shape_lanelet_ids
        self.initial_signal_state: Union[None, SignalState] = initial_signal_state
        self.signal_series: List[SignalState] = signal_series

    def __eq__(self, other):
        if not isinstance(other, Obstacle):
            warnings.warn(f"Inequality between Obstacle {repr(self)} and different type {type(other)}")
            return False

        initial_center_lanelet_ids = list() if self._initial_center_lanelet_ids is None \
            else list(self._initial_center_lanelet_ids)
        initial_center_lanelet_ids_other = list() if other.initial_center_lanelet_ids is None \
            else list(other.initial_center_lanelet_ids)

        initial_shape_lanelet_ids = list() if self._initial_shape_lanelet_ids is None \
            else list(self._initial_shape_lanelet_ids)
        initial_shape_lanelet_ids_other = list() if other.initial_shape_lanelet_ids is None \
            else list(other.initial_shape_lanelet_ids)

        obstacle_eq = self._obstacle_id == other.obstacle_id and self._obstacle_role == other.obstacle_role and \
            self._obstacle_type == other.obstacle_type and self._obstacle_shape == other.obstacle_shape and \
            self._initial_state == other.initial_state and \
            initial_center_lanelet_ids == initial_center_lanelet_ids_other and \
            initial_shape_lanelet_ids == initial_shape_lanelet_ids_other and \
            self._initial_signal_state == other.initial_signal_state and self._signal_series == other.signal_series

        return obstacle_eq

    def __hash__(self):
        initial_center_lanelet_ids = None if self._initial_center_lanelet_ids is None \
            else self._initial_center_lanelet_ids
        initial_shape_lanelet_ids = None if self._initial_shape_lanelet_ids is None \
            else self._initial_shape_lanelet_ids
        signal_series = None if self._signal_series is None else self.signal_series

        return hash((self._obstacle_id, self._obstacle_role, self._obstacle_type, self._obstacle_shape,
                     self._initial_state, initial_center_lanelet_ids, initial_shape_lanelet_ids,
                     self._initial_signal_state, signal_series))

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
    def obstacle_shape(self) -> Union[Shape, Rectangle, Circle, Polygon]:
        """ Obstacle shape as defined in CommonRoad."""
        return self._obstacle_shape

    @obstacle_shape.setter
    def obstacle_shape(self, shape: Union[Shape, Rectangle, Circle, Polygon]):
        assert isinstance(shape,
                          (type(None), Shape)), '<Obstacle/obstacle_shape>: argument shape of wrong type. Expected ' \
                                                'type %s. Got type %s.' % (Shape, type(shape))

        if not hasattr(self, '_obstacle_shape'):
            self._obstacle_shape = shape
        else:
            warnings.warn('<Obstacle/obstacle_shape>: Obstacle shape is immutable.')

    @property
    def initial_state(self) -> InitialState:
        """ Initial state of the obstacle, e.g., obtained through sensor measurements."""
        return self._initial_state

    @initial_state.setter
    def initial_state(self, initial_state: InitialState):
        assert isinstance(initial_state, State), '<Obstacle/initial_state>: argument initial_state of ' \
                                                      'wrong type. Expected types: %s. Got type: %s.' \
                                                      % (State, type(initial_state))
        self._initial_state = initial_state
        self._initial_occupancy_shape = occupancy_shape_from_state(self._obstacle_shape, initial_state)

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
        self._initial_signal_state = initial_signal_state

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
        self._signal_series = signal_series

    @abstractmethod
    def occupancy_at_time(self, time_step: int) -> Union[None, Occupancy]:
        pass

    @abstractmethod
    def state_at_time(self, time_step: int) -> Union[None, TraceState]:
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
                 obstacle_shape: Shape, initial_state: InitialState,
                 initial_center_lanelet_ids: Union[None, Set[int]] = None,
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

    def __eq__(self, other):
        if not isinstance(other, StaticObstacle):
            warnings.warn(f"Inequality between StaticObstacle {repr(self)} and different type {type(other)}")
            return False

        return Obstacle.__eq__(self, other)

    def __hash__(self):
        return Obstacle.__hash__(self)

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
        return Occupancy(time_step=time_step, shape=self._initial_occupancy_shape)

    def state_at_time(self, time_step: int) -> TraceState:
        """
        Returns the state the obstacle at a specific time step.

        :param time_step: discrete time step
        :return: state of the static obstacle at time step
        """
        return self.initial_state

    def __str__(self):
        obs_str = 'Static Obstacle:\n'
        obs_str += '\nid: {}'.format(self.obstacle_id)
        obs_str += '\ntype: {}'.format(self.obstacle_type.value)
        obs_str += '\ninitial state: {}'.format(self.initial_state)
        return obs_str

    def draw(self, renderer: IRenderer, draw_params: OptionalSpecificOrAllDrawParams[StaticObstacleParams] = None):
        renderer.draw_static_obstacle(self, draw_params)


class DynamicObstacle(Obstacle):
    """ Class representing dynamic obstacles as defined in CommonRoad. Each dynamic obstacle has stored its predicted
    movement in future time steps.
    """

    def __init__(self, obstacle_id: int, obstacle_type: ObstacleType,
                 obstacle_shape: Shape,
                 initial_state: TraceState,
                 prediction: Union[None, Prediction, TrajectoryPrediction, SetBasedPrediction] = None,
                 initial_center_lanelet_ids: Union[None, Set[int]] = None,
                 initial_shape_lanelet_ids: Union[None, Set[int]] = None,
                 initial_signal_state: Union[None, SignalState] = None,
                 signal_series: List[SignalState] = None,
                 initial_meta_information_state: MetaInformationState = None,
                 meta_information_series: List[MetaInformationState] = None,
                 external_dataset_id: int = None,
                 **kwargs):
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
            :param wheelbase: list of wheelbase lengths
            :param initial_meta_information_state: meta information of the dynamic obstacle
            :param meta_information_series: list of meta information
            :param external_dataset_id: ID of the external dataset
        """
        for (field, value) in kwargs.items():
            setattr(self, field, value)
        Obstacle.__init__(self, obstacle_id=obstacle_id, obstacle_role=ObstacleRole.DYNAMIC,
                          obstacle_type=obstacle_type, obstacle_shape=obstacle_shape, initial_state=initial_state,
                          initial_center_lanelet_ids=initial_center_lanelet_ids,
                          initial_shape_lanelet_ids=initial_shape_lanelet_ids,
                          initial_signal_state=initial_signal_state, signal_series=signal_series)
        self.prediction: Prediction = prediction
        self.initial_meta_information_state = initial_meta_information_state
        self.meta_information_series = meta_information_series
        self.external_dataset_id = external_dataset_id

    def __eq__(self, other):
        if not isinstance(other, DynamicObstacle):
            warnings.warn(f"Inequality between DynamicObstacle {repr(self)} and different type {type(other)}")
            return False

        return self._prediction == other.prediction and Obstacle.__eq__(self, other) and \
            self._initial_meta_information_state == other.initial_meta_information_state and \
            self._meta_information_series == other.meta_information_series and \
            self._external_dataset_id == other.external_dataset_id

    def __hash__(self):
        return hash((self._prediction, self._initial_meta_information_state, self._meta_information_series,
                     self._external_dataset_id, Obstacle.__hash__(self)))

    @property
    def initial_state(self) -> State:
        """ Initial state of the obstacle, e.g., obtained through sensor measurements."""
        return self._initial_state

    @initial_state.setter
    def initial_state(self, initial_state: State):
        assert isinstance(initial_state, State), '<Obstacle/initial_state>: argument initial_state of wrong type. ' \
                                                 'Expected types: %s. Got type: %s.' % (State, type(initial_state))
        self._initial_state = initial_state
        self._initial_occupancy_shape = occupancy_shape_from_state(self._obstacle_shape, initial_state)
        if not hasattr(self, 'wheelbase_lengths'):
            return
        shapes = self.obstacle_shape.shapes
        self._initial_occupancy_shape = shape_group_occupancy_shape_from_state(shapes, initial_state,
                                                                               self.wheelbase_lengths)

    @property
    def prediction(self) -> Union[Prediction, TrajectoryPrediction, SetBasedPrediction, None]:
        """ Prediction describing the movement of the dynamic obstacle over time."""
        return self._prediction

    @prediction.setter
    def prediction(self, prediction: Union[Prediction, TrajectoryPrediction, SetBasedPrediction,  None]):
        assert isinstance(prediction, (Prediction, type(None))), '<DynamicObstacle/prediction>: argument prediction ' \
                                                                 'of wrong type. Expected types: %s, %s. Got type: ' \
                                                                 '%s.' % (Prediction, type(None), type(prediction))
        self._prediction = prediction

    @property
    def initial_meta_information_state(self) -> Union[None, MetaInformationState]:
        """ Meta information of the dynamic obstacle. """
        return self._initial_meta_information_state

    @initial_meta_information_state.setter
    def initial_meta_information_state(self, initial_meta_information_state: Union[MetaInformationState, None]):
        assert isinstance(initial_meta_information_state, (MetaInformationState, type(None))), \
            '<DynamicObstacle/initial_meta_information_state>: argument prediction ' \
            'of wrong type. Expected types: %s, %s. Got type: %s.' \
            % (MetaInformationState, type(None), type(initial_meta_information_state))

        self._initial_meta_information_state = initial_meta_information_state

    @property
    def meta_information_series(self) -> Union[None, List[MetaInformationState]]:
        """ List of meta information."""
        return self._meta_information_series

    @meta_information_series.setter
    def meta_information_series(self, meta_information_series: Union[List[MetaInformationState], None]):
        assert isinstance(meta_information_series, (List, type(None))), \
            '<DynamicObstacle/meta_information_series>: argument prediction ' \
            'of wrong type. Expected types: %s, %s. Got type: %s.' \
            % (List, type(None), type(meta_information_series))

        self._meta_information_series = meta_information_series

    @property
    def external_dataset_id(self) -> Union[int, None]:
        """ ID of the external dataset."""
        return self._external_dataset_id

    @external_dataset_id.setter
    def external_dataset_id(self, external_dataset_id: Union[int, None]):
        assert isinstance(external_dataset_id, (int, type(None))), \
            '<DynamicObstacle/external_dataset_id>: argument prediction of wrong type. ' \
            'Expected types: %s, %s. Got type: %s.' % (int, type(None), type(external_dataset_id))
        self._external_dataset_id = external_dataset_id

    def occupancy_at_time(self, time_step: int) -> Union[None, Occupancy]:
        """
        Returns the predicted occupancy of the obstacle at a specific time step.

        :param time_step: discrete time step
        :return: predicted occupancy of the obstacle at time step
        """
        occupancy = None

        if time_step == self.initial_state.time_step:
            occupancy = Occupancy(time_step, self._initial_occupancy_shape)
        elif time_step > self.initial_state.time_step and self._prediction is not None:
            occupancy = self._prediction.occupancy_at_time_step(time_step)
        return occupancy

    def state_at_time(self, time_step: int) -> Union[None, TraceState]:
        """
        Returns the predicted state of the obstacle at a specific time step.

        :param time_step: discrete time step
        :return: predicted state of the obstacle at time step
        """
        if time_step == self.initial_state.time_step:
            return self.initial_state
        elif type(self._prediction) is SetBasedPrediction:
            warnings.warn("<DynamicObstacle/state_at_time>: Set-based prediction is used. State cannot be returned!")
            return None
        elif time_step > self.initial_state.time_step and self._prediction is not None:
            return self.prediction.trajectory.state_at_time_step(time_step)
        else:
            return None

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

        self.initial_state = self._initial_state.translate_rotate(translation, angle)

    def __str__(self):
        obs_str = 'Dynamic Obstacle:\n'
        obs_str += '\nid: {}'.format(self.obstacle_id)
        obs_str += '\ntype: {}'.format(self.obstacle_type.value)
        obs_str += '\ninitial state: {}'.format(self.initial_state)
        return obs_str

    def draw(self, renderer: IRenderer, draw_params: OptionalSpecificOrAllDrawParams[DynamicObstacleParams] = None):
        renderer.draw_dynamic_obstacle(self, draw_params)


class PhantomObstacle(IDrawable):
    """ Class representing phantom obstacles as defined in CommonRoad. Each phantom obstacle has stored its predicted
    movement in future time steps as occupancy set.
    """

    def __init__(self, obstacle_id: int,
                 prediction: SetBasedPrediction = None):
        """
        Constructor of PhantomObstacle object.

        :param obstacle_id: unique ID of the obstacle
        :param prediction: set-based prediction of phantom obstacle
        """
        self.obstacle_id = obstacle_id
        self.prediction: SetBasedPrediction = prediction
        self.obstacle_role: ObstacleRole = ObstacleRole.Phantom

    def __eq__(self, other):
        if not isinstance(other, PhantomObstacle):
            warnings.warn(f"Inequality between PhantomObstacle {repr(self)} and different type {type(other)}")
            return False

        obstacle_eq = self.obstacle_id == other.obstacle_id and self.prediction == other.prediction

        return obstacle_eq

    def __hash__(self):
        return hash((self.obstacle_id, self._prediction))

    @property
    def prediction(self) -> Union[SetBasedPrediction, None]:
        """ Prediction describing the movement of the dynamic obstacle over time."""
        return self._prediction

    @prediction.setter
    def prediction(self, prediction: Union[Prediction, TrajectoryPrediction, SetBasedPrediction,  None]):
        assert isinstance(prediction, (SetBasedPrediction, type(None))), \
            '<PhantomObstacle/prediction>: argument prediction of wrong type. Expected types: %s, %s. Got type: ' \
            '%s.' % (SetBasedPrediction, type(None), type(prediction))
        self._prediction = prediction

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

    def occupancy_at_time(self, time_step: int) -> Union[None, Occupancy]:
        """
        Returns the predicted occupancy of the obstacle at a specific time step.

        :param time_step: discrete time step
        :return: predicted occupancy of the obstacle at time step
        """
        occupancy = None
        if self._prediction is not None and self._prediction.occupancy_at_time_step(time_step) is not None:
            occupancy = self._prediction.occupancy_at_time_step(time_step)
        else:
            warnings.warn("<PhantomObstacle/occupancy_at_time>: Time step does not exist!")

        return occupancy

    @staticmethod
    def state_at_time() -> Union[None, TraceState]:
        """
        Returns the predicted state of the obstacle at a specific time step.

        :return: predicted state of the obstacle at time step
        """
        warnings.warn("<PhantomObstacle/state_at_time>: Set-based prediction is used. State cannot be returned!")
        return None

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

    def __str__(self):
        obs_str = 'Phantom Obstacle:\n'
        obs_str += '\nid: {}'.format(self.obstacle_id)
        return obs_str

    def draw(self, renderer: IRenderer, draw_params: OptionalSpecificOrAllDrawParams[PhantomObstacleParams] = None):
        renderer.draw_phantom_obstacle(self, draw_params)


class EnvironmentObstacle(IDrawable):
    """ Class representing environment obstacles as defined in CommonRoad."""

    def __init__(self, obstacle_id: int, obstacle_type: ObstacleType,
                 obstacle_shape: Shape):
        """
            :param obstacle_id: unique ID of the obstacle
            :param obstacle_type: type of obstacle (e.g. BUILDING)
            :param obstacle_shape: shape of the static obstacle
        """
        self.obstacle_id: int = obstacle_id
        self.obstacle_role: ObstacleRole = ObstacleRole.ENVIRONMENT
        self.obstacle_type: ObstacleType = obstacle_type
        self.obstacle_shape: Shape = obstacle_shape

    def __eq__(self, other):
        if not isinstance(other, EnvironmentObstacle):
            warnings.warn(f"Inequality between EnvironmentObstacle {repr(self)} and different type {type(other)}")
            return False

        obstacle_eq = self._obstacle_id == other.obstacle_id and self._obstacle_role == other.obstacle_role and \
            self._obstacle_type == other.obstacle_type and self._obstacle_shape == other.obstacle_shape

        return obstacle_eq

    def __hash__(self):
        return hash((self._obstacle_id, self._obstacle_role, self._obstacle_type, self._obstacle_shape))

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
    def obstacle_shape(self) -> Union[Shape, Polygon, Circle, Rectangle]:
        """ Obstacle shape as defined in CommonRoad."""
        return self._obstacle_shape

    @obstacle_shape.setter
    def obstacle_shape(self, shape: Union[Shape, Polygon, Circle, Rectangle]):
        assert isinstance(shape,
                          (type(None), Shape)), '<Obstacle/obstacle_shape>: argument shape of wrong type. Expected ' \
                                                'type %s. Got type %s.' % (Shape, type(shape))

        if not hasattr(self, '_obstacle_shape'):
            self._obstacle_shape = shape
        else:
            warnings.warn('<Obstacle/obstacle_shape>: Obstacle shape is immutable.')

    def occupancy_at_time(self, time_step: int) -> Occupancy:
        """
        Returns the predicted occupancy of the obstacle at a specific time step.

        :param time_step: discrete time step
        :return: occupancy of the static obstacle at time step
        """
        return Occupancy(time_step=time_step, shape=self._obstacle_shape)

    def __str__(self):
        obs_str = 'Environment Obstacle:\n'
        obs_str += '\nid: {}'.format(self.obstacle_id)
        return obs_str

    def draw(self, renderer: IRenderer, draw_params: OptionalSpecificOrAllDrawParams[EnvironmentObstacleParams] = None):
        renderer.draw_environment_obstacle(self, draw_params)
