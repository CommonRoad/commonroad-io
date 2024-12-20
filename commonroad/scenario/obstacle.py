import enum
import warnings
from abc import abstractmethod
from typing import List, Optional, Set, Union

import numpy as np

from commonroad.common.validity import (
    is_real_number,
    is_real_number_vector,
    is_valid_orientation,
)
from commonroad.geometry.shape import (
    Circle,
    Polygon,
    Rectangle,
    Shape,
    occupancy_shape_from_state,
    shape_group_occupancy_shape_from_state,
)
from commonroad.prediction.prediction import (
    Occupancy,
    Prediction,
    SetBasedPrediction,
    TrajectoryPrediction,
)
from commonroad.scenario.state import (
    InitialState,
    MetaInformationState,
    SignalState,
    TraceState,
)
from commonroad.visualization.draw_params import (
    DynamicObstacleParams,
    EnvironmentObstacleParams,
    OptionalSpecificOrAllDrawParams,
    PhantomObstacleParams,
    StaticObstacleParams,
)
from commonroad.visualization.drawable import IDrawable
from commonroad.visualization.renderer import IRenderer


@enum.unique
class ObstacleRole(enum.Enum):
    """Enum containing all possible obstacle roles defined in CommonRoad."""

    STATIC = "static"
    DYNAMIC = "dynamic"
    ENVIRONMENT = "environment"
    Phantom = "phantom"


@enum.unique
class ObstacleType(enum.Enum):
    """Enum containing all possible obstacle types defined in CommonRoad."""

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


class Obstacle(IDrawable):
    """Superclass for dynamic and static obstacles holding common properties
    defined in CommonRoad."""

    def __init__(
        self,
        obstacle_id: int,
        obstacle_role: ObstacleRole,
        obstacle_type: ObstacleType,
        obstacle_shape: Shape,
        initial_state: InitialState = None,
        initial_center_lanelet_ids: Optional[Set[int]] = None,
        initial_shape_lanelet_ids: Optional[Set[int]] = None,
        initial_signal_state: Optional[SignalState] = None,
        signal_series: Optional[List[SignalState]] = None,
    ):
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
        self._initial_occupancy_shape: Optional[Shape] = None
        self.obstacle_id: int = obstacle_id
        self.obstacle_role: ObstacleRole = obstacle_role
        self.obstacle_type: ObstacleType = obstacle_type
        self.obstacle_shape: Shape = obstacle_shape
        self.initial_state: InitialState = initial_state
        self.initial_center_lanelet_ids: Optional[Set[int]] = initial_center_lanelet_ids
        self.initial_shape_lanelet_ids: Optional[Set[int]] = initial_shape_lanelet_ids
        self.initial_signal_state: Optional[SignalState] = initial_signal_state
        self.signal_series: Optional[List[SignalState]] = signal_series

    def __eq__(self, other):
        if not isinstance(other, Obstacle):
            warnings.warn(f"Inequality between Obstacle {repr(self)} and different type {type(other)}")
            return False

        initial_center_lanelet_ids = (
            list() if self._initial_center_lanelet_ids is None else list(self._initial_center_lanelet_ids)
        )
        initial_center_lanelet_ids_other = (
            list() if other.initial_center_lanelet_ids is None else list(other.initial_center_lanelet_ids)
        )

        initial_shape_lanelet_ids = (
            list() if self._initial_shape_lanelet_ids is None else list(self._initial_shape_lanelet_ids)
        )
        initial_shape_lanelet_ids_other = (
            list() if other.initial_shape_lanelet_ids is None else list(other.initial_shape_lanelet_ids)
        )

        obstacle_eq = (
            self._obstacle_id == other.obstacle_id
            and self._obstacle_role == other.obstacle_role
            and self._obstacle_type == other.obstacle_type
            and self._obstacle_shape == other.obstacle_shape
            and self._initial_state == other.initial_state
            and initial_center_lanelet_ids == initial_center_lanelet_ids_other
            and initial_shape_lanelet_ids == initial_shape_lanelet_ids_other
            and self._initial_signal_state == other.initial_signal_state
            and self._signal_series == other.signal_series
        )

        return obstacle_eq

    def __hash__(self):
        initial_center_lanelet_ids = (
            None if self.initial_center_lanelet_ids is None else frozenset(self.initial_center_lanelet_ids)
        )
        initial_shape_lanelet_ids = (
            None if self.initial_shape_lanelet_ids is None else frozenset(self.initial_shape_lanelet_ids)
        )
        signal_series = None if self.signal_series is None else frozenset(self.signal_series)
        return hash(
            (
                self._obstacle_id,
                self._obstacle_role,
                self._obstacle_type,
                self._obstacle_shape,
                self._initial_state,
                initial_center_lanelet_ids,
                initial_shape_lanelet_ids,
                self._initial_signal_state,
                signal_series,
            )
        )

    @property
    def obstacle_id(self) -> int:
        """Unique ID of the obstacle."""
        return self._obstacle_id

    @obstacle_id.setter
    def obstacle_id(self, obstacle_id: int):
        assert isinstance(
            obstacle_id, int
        ), "<Obstacle/obstacle_id>: argument obstacle_id of wrong type." "Expected type: %s. Got type: %s." % (
            int,
            type(obstacle_id),
        )
        if not hasattr(self, "_obstacle_id"):
            self._obstacle_id = obstacle_id
        else:
            warnings.warn("<Obstacle/obstacle_id>: Obstacle ID is immutable.")

    @property
    def obstacle_role(self) -> ObstacleRole:
        """Obstacle role as defined in CommonRoad."""
        return self._obstacle_role

    @obstacle_role.setter
    def obstacle_role(self, obstacle_role: ObstacleRole):
        assert isinstance(
            obstacle_role, ObstacleRole
        ), "<Obstacle/obstacle_role>: argument obstacle_role of wrong " "type. Expected type: %s. Got type: %s." % (
            ObstacleRole,
            type(obstacle_role),
        )
        if not hasattr(self, "_obstacle_role"):
            self._obstacle_role = obstacle_role
        else:
            warnings.warn("<Obstacle/obstacle_role>: Obstacle role is immutable.")

    @property
    def obstacle_type(self) -> ObstacleType:
        """Obstacle type as defined in CommonRoad."""
        return self._obstacle_type

    @obstacle_type.setter
    def obstacle_type(self, obstacle_type: ObstacleType):
        assert isinstance(
            obstacle_type, ObstacleType
        ), "<Obstacle/obstacle_type>: argument obstacle_type of wrong " "type. Expected type: %s. Got type: %s." % (
            ObstacleType,
            type(obstacle_type),
        )
        if not hasattr(self, "_obstacle_type"):
            self._obstacle_type = obstacle_type
        else:
            warnings.warn("<Obstacle/obstacle_type>: Obstacle type is immutable.")

    @property
    def obstacle_shape(self) -> Union[Shape, Rectangle, Circle, Polygon]:
        """Obstacle shape as defined in CommonRoad."""
        return self._obstacle_shape

    @obstacle_shape.setter
    def obstacle_shape(self, shape: Union[Shape, Rectangle, Circle, Polygon]):
        assert isinstance(
            shape, (type(None), Shape)
        ), "<Obstacle/obstacle_shape>: argument shape of wrong type. Expected " "type %s. Got type %s." % (
            Shape,
            type(shape),
        )

        if not hasattr(self, "_obstacle_shape"):
            self._obstacle_shape = shape
        else:
            warnings.warn("<Obstacle/obstacle_shape>: Obstacle shape is immutable.")

    @property
    def initial_state(self) -> InitialState:
        """Initial state of the obstacle, e.g., obtained through sensor measurements."""
        return self._initial_state

    @initial_state.setter
    def initial_state(self, initial_state: InitialState):
        assert isinstance(
            initial_state, InitialState
        ), "<Obstacle/initial_state>: argument initial_state of wrong type. " "Expected types: %s. Got type: %s." % (
            InitialState,
            type(initial_state),
        )
        self._initial_state = initial_state
        self._initial_occupancy_shape = occupancy_shape_from_state(self._obstacle_shape, initial_state)
        if not hasattr(self, "wheelbase_lengths"):
            return
        shapes = self.obstacle_shape.shapes
        self._initial_occupancy_shape = shape_group_occupancy_shape_from_state(
            shapes, initial_state, self.wheelbase_lengths
        )

    @property
    def initial_center_lanelet_ids(self) -> Union[None, Set[int]]:
        """Initial lanelets of obstacle center, e.g., obtained through localization."""
        return self._initial_center_lanelet_ids

    @initial_center_lanelet_ids.setter
    def initial_center_lanelet_ids(self, initial_center_lanelet_ids: Union[None, Set[int]]):
        assert isinstance(initial_center_lanelet_ids, (set, type(None))), (
            "<Obstacle/initial_center_lanelet_ids>: argument initial_lanelet_ids of wrong type. "
            "Expected types: %s, %s. Got type: %s." % (set, type(None), type(initial_center_lanelet_ids))
        )
        if initial_center_lanelet_ids is not None:
            for lanelet_id in initial_center_lanelet_ids:
                assert isinstance(lanelet_id, int), (
                    "<Obstacle/initial_center_lanelet_ids>: argument initial_lanelet of wrong type. "
                    "Expected types: %s. Got type: %s." % (int, type(lanelet_id))
                )
        self._initial_center_lanelet_ids = initial_center_lanelet_ids

    @property
    def initial_shape_lanelet_ids(self) -> Union[None, Set[int]]:
        """Initial lanelets of obstacle shape, e.g., obtained through localization."""
        return self._initial_shape_lanelet_ids

    @initial_shape_lanelet_ids.setter
    def initial_shape_lanelet_ids(self, initial_shape_lanelet_ids: Union[None, Set[int]]):
        assert isinstance(initial_shape_lanelet_ids, (set, type(None))), (
            "<Obstacle/initial_shape_lanelet_ids>: argument initial_lanelet_ids of wrong type. "
            "Expected types: %s, %s. Got type: %s." % (set, type(None), type(initial_shape_lanelet_ids))
        )
        if initial_shape_lanelet_ids is not None:
            for lanelet_id in initial_shape_lanelet_ids:
                assert isinstance(lanelet_id, int), (
                    "<Obstacle/initial_shape_lanelet_ids>: argument initial_lanelet of wrong type. "
                    "Expected types: %s. Got type: %s." % (int, type(lanelet_id))
                )
        self._initial_shape_lanelet_ids = initial_shape_lanelet_ids

    @property
    def initial_signal_state(self) -> SignalState:
        """Signal state as defined in CommonRoad."""
        return self._initial_signal_state

    @initial_signal_state.setter
    def initial_signal_state(self, initial_signal_state: SignalState):
        assert isinstance(initial_signal_state, (SignalState, type(None))), (
            "<Obstacle/initial_signal_state>: "
            "argument initial_signal_state of wrong "
            "type. Expected types: %s, %s. Got type: %s." % (SignalState, type(None), type(initial_signal_state))
        )
        self._initial_signal_state = initial_signal_state

    @property
    def signal_series(self) -> List[SignalState]:
        """Signal series as defined in CommonRoad."""
        return self._signal_series

    @signal_series.setter
    def signal_series(self, signal_series: List[SignalState]):
        assert isinstance(signal_series, (list, type(None))), (
            "<Obstacle/initial_signal_state>: "
            "argument initial_signal_state of wrong "
            "type. Expected types: %s, %s. Got type: %s." % (list, type(None), type(signal_series))
        )
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
    """Class representing static obstacles as defined in CommonRoad."""

    def __init__(
        self,
        obstacle_id: int,
        obstacle_type: ObstacleType,
        obstacle_shape: Shape,
        initial_state: InitialState,
        initial_center_lanelet_ids: Union[None, Set[int]] = None,
        initial_shape_lanelet_ids: Union[None, Set[int]] = None,
        initial_signal_state: Union[None, SignalState] = None,
        signal_series: List[SignalState] = None,
    ):
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
        Obstacle.__init__(
            self,
            obstacle_id=obstacle_id,
            obstacle_role=ObstacleRole.STATIC,
            obstacle_type=obstacle_type,
            obstacle_shape=obstacle_shape,
            initial_state=initial_state,
            initial_center_lanelet_ids=initial_center_lanelet_ids,
            initial_shape_lanelet_ids=initial_shape_lanelet_ids,
            initial_signal_state=initial_signal_state,
            signal_series=signal_series,
        )

    def __eq__(self, other):
        if not isinstance(other, StaticObstacle):
            warnings.warn(f"Inequality between StaticObstacle {repr(self)} and different type {type(other)}")
            return False

        return Obstacle.__eq__(self, other)

    def __hash__(self):
        return Obstacle.__hash__(self)

    def translate_rotate(self, translation: np.ndarray, angle: float):
        """First translates the static obstacle, then rotates the static obstacle around the origin.

        :param translation: translation vector [x_off, y_off] in x- and y-direction
        :param angle: rotation angle in radian (counter-clockwise)
        """
        assert is_real_number_vector(translation, 2), (
            "<StaticObstacle/translate_rotate>: argument translation is " "not a vector of real numbers of length 2."
        )
        assert is_real_number(angle), (
            "<StaticObstacle/translate_rotate>: argument angle must be a scalar. " "angle = %s" % angle
        )
        assert is_valid_orientation(angle), (
            "<StaticObstacle/translate_rotate>: argument angle must be within the "
            "interval [-2pi, 2pi]. angle = %s" % angle
        )
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
        obs_str = "Static Obstacle:\n"
        obs_str += "\nid: {}".format(self.obstacle_id)
        obs_str += "\ntype: {}".format(self.obstacle_type.value)
        obs_str += "\ninitial state: {}".format(self.initial_state)
        return obs_str

    def draw(self, renderer: IRenderer, draw_params: OptionalSpecificOrAllDrawParams[StaticObstacleParams] = None):
        renderer.draw_static_obstacle(self, draw_params)


class DynamicObstacle(Obstacle):
    """Class representing dynamic obstacles as defined in CommonRoad. Each dynamic obstacle has stored its predicted
    movement in future time steps.
    """

    def __init__(
        self,
        obstacle_id: int,
        obstacle_type: ObstacleType,
        obstacle_shape: Shape,
        initial_state: InitialState,
        prediction: Union[None, Prediction, TrajectoryPrediction, SetBasedPrediction] = None,
        initial_center_lanelet_ids: Optional[Set[int]] = None,
        initial_shape_lanelet_ids: Optional[Set[int]] = None,
        initial_signal_state: Optional[SignalState] = None,
        signal_series: List[SignalState] = None,
        initial_meta_information_state: MetaInformationState = None,
        meta_information_series: List[MetaInformationState] = None,
        external_dataset_id: int = None,
        history: Optional[List[TraceState]] = None,
        signal_history: Optional[List[SignalState]] = None,
        center_lanelet_ids_history: Optional[List[Set[int]]] = None,
        shape_lanelet_ids_history: Optional[List[Set[int]]] = None,
        **kwargs,
    ):
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
        :param history: History of actual states
        :param signal_history: History of signal states
        """
        for field, value in kwargs.items():
            setattr(self, field, value)
        Obstacle.__init__(
            self,
            obstacle_id=obstacle_id,
            obstacle_role=ObstacleRole.DYNAMIC,
            obstacle_type=obstacle_type,
            obstacle_shape=obstacle_shape,
            initial_state=initial_state,
            initial_center_lanelet_ids=initial_center_lanelet_ids,
            initial_shape_lanelet_ids=initial_shape_lanelet_ids,
            initial_signal_state=initial_signal_state,
            signal_series=signal_series,
        )
        self.prediction: Prediction = prediction
        self.initial_meta_information_state = initial_meta_information_state
        self.meta_information_series = meta_information_series
        self.external_dataset_id = external_dataset_id
        self.history = history or []
        self.signal_history = signal_history or []
        self.center_lanelet_ids_history = center_lanelet_ids_history or []
        self.shape_lanelet_ids_history = shape_lanelet_ids_history or []

    def __eq__(self, other):
        if not isinstance(other, DynamicObstacle):
            warnings.warn(f"Inequality between DynamicObstacle {repr(self)} and different type {type(other)}")
            return False

        return (
            self._prediction == other.prediction
            and Obstacle.__eq__(self, other)
            and self._initial_meta_information_state == other.initial_meta_information_state
            and self._meta_information_series == other.meta_information_series
            and self._external_dataset_id == other.external_dataset_id
            and self.history == other.history
            and self.signal_history == other.signal_history
            and self.center_lanelet_ids_history == other.center_lanelet_ids_history
            and self.shape_lanelet_ids_history == other.shape_lanelet_ids_history
        )

    def __hash__(self):
        center_lanelet_ids_history = (
            None
            if self.center_lanelet_ids_history is None
            else tuple(frozenset(value) for value in self.center_lanelet_ids_history)
        )
        shape_lanelet_ids_history = (
            None
            if self.shape_lanelet_ids_history is None
            else tuple(frozenset(value) for value in self.shape_lanelet_ids_history)
        )
        meta_information_series = (
            None if self._meta_information_series is None else tuple(self._meta_information_series)
        )

        return hash(
            (
                self._prediction,
                self._initial_meta_information_state,
                meta_information_series,
                self._external_dataset_id,
                tuple(self.history),
                tuple(self.signal_history),
                center_lanelet_ids_history,
                shape_lanelet_ids_history,
                Obstacle.__hash__(self),
            )
        )

    @property
    def prediction(self) -> Union[None, Prediction, TrajectoryPrediction, SetBasedPrediction]:
        """Prediction describing the movement of the dynamic obstacle over time."""
        return self._prediction

    @prediction.setter
    def prediction(self, prediction: Union[Prediction, TrajectoryPrediction, SetBasedPrediction, None]):
        assert isinstance(prediction, (Prediction, type(None))), (
            "<DynamicObstacle/prediction>: argument prediction "
            "of wrong type. Expected types: %s, %s. Got type: "
            "%s." % (Prediction, type(None), type(prediction))
        )
        self._prediction = prediction

    @property
    def initial_meta_information_state(self) -> Union[None, MetaInformationState]:
        """Meta information of the dynamic obstacle."""
        return self._initial_meta_information_state

    @initial_meta_information_state.setter
    def initial_meta_information_state(self, initial_meta_information_state: Union[MetaInformationState, None]):
        assert isinstance(initial_meta_information_state, (MetaInformationState, type(None))), (
            "<DynamicObstacle/initial_meta_information_state>: argument prediction "
            "of wrong type. Expected types: %s, %s. Got type: %s."
            % (MetaInformationState, type(None), type(initial_meta_information_state))
        )

        self._initial_meta_information_state = initial_meta_information_state

    @property
    def meta_information_series(self) -> Union[None, List[MetaInformationState]]:
        """List of meta information."""
        return self._meta_information_series

    @meta_information_series.setter
    def meta_information_series(self, meta_information_series: Union[List[MetaInformationState], None]):
        assert isinstance(meta_information_series, (List, type(None))), (
            "<DynamicObstacle/meta_information_series>: argument prediction "
            "of wrong type. Expected types: %s, %s. Got type: %s." % (List, type(None), type(meta_information_series))
        )

        self._meta_information_series = meta_information_series

    @property
    def external_dataset_id(self) -> Union[int, None]:
        """ID of the external dataset."""
        return self._external_dataset_id

    @external_dataset_id.setter
    def external_dataset_id(self, external_dataset_id: Union[int, None]):
        assert isinstance(external_dataset_id, (int, type(None))), (
            "<DynamicObstacle/external_dataset_id>: argument prediction of wrong type. "
            "Expected types: %s, %s. Got type: %s." % (int, type(None), type(external_dataset_id))
        )
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
        """First translates the dynamic obstacle, then rotates the dynamic obstacle around the origin.

        :param translation: translation vector [x_off, y_off] in x- and y-direction
        :param angle: rotation angle in radian (counter-clockwise)
        """
        assert is_real_number_vector(translation, 2), (
            "<DynamicObstacle/translate_rotate>: argument translation is " "not a vector of real numbers of length 2."
        )
        assert is_real_number(angle), (
            "<DynamicObstacle/translate_rotate>: argument angle must be a scalar. " "angle = %s" % angle
        )
        assert is_valid_orientation(angle), (
            "<DynamicObstacle/translate_rotate>: argument angle must be within the "
            "interval [-2pi, 2pi]. angle = %s" % angle
        )
        if self._prediction is not None:
            self.prediction.translate_rotate(translation, angle)

        self.initial_state = self._initial_state.translate_rotate(translation, angle)

    def update_initial_state(
        self,
        current_state: TraceState,
        current_signal_state: Optional[SignalState] = None,
        current_center_lanelet_ids: Optional[Set[int]] = None,
        current_shape_lanelet_ids: Optional[Set[int]] = None,
        max_history_length: int = 6000,
    ):
        """Updates the initial state to the given current state, appends the current initial state to the history, and
        invalidates the prediction.

        :param current_state: current state of the dynamic obstacle, will become the new initial state
        :param current_signal_state: current signal state of the dynamic obstacle, will become the new initial signal
                                     state
        :param current_center_lanelet_ids: current center lanelet ids of the dynamic obstacle, will become the new
                                           initial center lanelet ids
        :param current_shape_lanelet_ids: current shape lanelet ids of the dynamic obstacle, will become the new
                                          initial shape lanelet ids
        :param max_history_length: maximum length of the history, if the history exceeds this it will be truncated,
                                   dropping the oldest elements first; must be greater than 0
        """
        assert max_history_length > 0, (
            f"<DynamicObstacle/update_initial_state>: argument max_history_length must be"
            f"greater than 0. max_history_length = {max_history_length}"
        )

        # append current initial state to history
        self.history.append(self.initial_state)
        self.signal_history.append(self.initial_signal_state)
        self.center_lanelet_ids_history.append(self.initial_center_lanelet_ids)
        self.shape_lanelet_ids_history.append(self.initial_shape_lanelet_ids)

        # set initial state to current state
        self.initial_state = current_state
        self.initial_signal_state = current_signal_state
        self.initial_center_lanelet_ids = current_center_lanelet_ids
        self.initial_shape_lanelet_ids = current_shape_lanelet_ids

        # invalidate prediction
        self.prediction = None
        self.signal_series = None

        # truncate history if it is longer than desired max. length
        if len(self.history) > max_history_length:
            self.history = self.history[-max_history_length:]
            self.signal_history = self.signal_history[-max_history_length:]
            self.center_lanelet_ids_history = self.center_lanelet_ids_history[-max_history_length:]
            self.shape_lanelet_ids_history = self.shape_lanelet_ids_history[-max_history_length:]

    def update_prediction(
        self,
        prediction: Union[Prediction, SetBasedPrediction, TrajectoryPrediction],
        signal_series: Optional[List[SignalState]] = None,
    ):
        """Updates the prediction of the dynamic obstacle.

        :param prediction: updated movement prediction of the dynamic obstacle
        :param signal_series: updated prediction of the signal state of the dynamic obstacle
        """
        self.prediction = prediction
        self.signal_series = signal_series

    def __str__(self):
        obs_str = "Dynamic Obstacle:\n"
        obs_str += "\nid: {}".format(self.obstacle_id)
        obs_str += "\ntype: {}".format(self.obstacle_type.value)
        obs_str += "\ninitial state: {}".format(self.initial_state)
        return obs_str

    def draw(self, renderer: IRenderer, draw_params: OptionalSpecificOrAllDrawParams[DynamicObstacleParams] = None):
        renderer.draw_dynamic_obstacle(self, draw_params)


class PhantomObstacle(IDrawable):
    """Class representing phantom obstacles as defined in CommonRoad. Each phantom obstacle has stored its predicted
    movement in future time steps as occupancy set.
    """

    def __init__(self, obstacle_id: int, prediction: SetBasedPrediction = None):
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
        """Prediction describing the movement of the dynamic obstacle over time."""
        return self._prediction

    @prediction.setter
    def prediction(self, prediction: Union[Prediction, TrajectoryPrediction, SetBasedPrediction, None]):
        assert isinstance(prediction, (SetBasedPrediction, type(None))), (
            "<PhantomObstacle/prediction>: argument prediction of wrong type. Expected types: %s, %s. Got type: "
            "%s." % (SetBasedPrediction, type(None), type(prediction))
        )
        self._prediction = prediction

    @property
    def obstacle_role(self) -> ObstacleRole:
        """Obstacle role as defined in CommonRoad."""
        return self._obstacle_role

    @obstacle_role.setter
    def obstacle_role(self, obstacle_role: ObstacleRole):
        assert isinstance(
            obstacle_role, ObstacleRole
        ), "<Obstacle/obstacle_role>: argument obstacle_role of wrong " "type. Expected type: %s. Got type: %s." % (
            ObstacleRole,
            type(obstacle_role),
        )
        if not hasattr(self, "_obstacle_role"):
            self._obstacle_role = obstacle_role
        else:
            warnings.warn("<Obstacle/obstacle_role>: Obstacle role is immutable.")

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
        """First translates the dynamic obstacle, then rotates the dynamic obstacle around the origin.

        :param translation: translation vector [x_off, y_off] in x- and y-direction
        :param angle: rotation angle in radian (counter-clockwise)
        """
        assert is_real_number_vector(translation, 2), (
            "<DynamicObstacle/translate_rotate>: argument translation is " "not a vector of real numbers of length 2."
        )
        assert is_real_number(angle), (
            "<DynamicObstacle/translate_rotate>: argument angle must be a scalar. " "angle = %s" % angle
        )
        assert is_valid_orientation(angle), (
            "<DynamicObstacle/translate_rotate>: argument angle must be within the "
            "interval [-2pi, 2pi]. angle = %s" % angle
        )
        if self._prediction is not None:
            self.prediction.translate_rotate(translation, angle)

    def __str__(self):
        obs_str = "Phantom Obstacle:\n"
        obs_str += "\nid: {}".format(self.obstacle_id)
        return obs_str

    def draw(self, renderer: IRenderer, draw_params: OptionalSpecificOrAllDrawParams[PhantomObstacleParams] = None):
        renderer.draw_phantom_obstacle(self, draw_params)


class EnvironmentObstacle(IDrawable):
    """Class representing environment obstacles as defined in CommonRoad."""

    def __init__(self, obstacle_id: int, obstacle_type: ObstacleType, obstacle_shape: Shape):
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

        obstacle_eq = (
            self._obstacle_id == other.obstacle_id
            and self._obstacle_role == other.obstacle_role
            and self._obstacle_type == other.obstacle_type
            and self._obstacle_shape == other.obstacle_shape
        )

        return obstacle_eq

    def __hash__(self):
        return hash((self._obstacle_id, self._obstacle_role, self._obstacle_type, self._obstacle_shape))

    @property
    def obstacle_id(self) -> int:
        """Unique ID of the obstacle."""
        return self._obstacle_id

    @obstacle_id.setter
    def obstacle_id(self, obstacle_id: int):
        assert isinstance(
            obstacle_id, int
        ), "<Obstacle/obstacle_id>: argument obstacle_id of wrong type." "Expected type: %s. Got type: %s." % (
            int,
            type(obstacle_id),
        )
        if not hasattr(self, "_obstacle_id"):
            self._obstacle_id = obstacle_id
        else:
            warnings.warn("<Obstacle/obstacle_id>: Obstacle ID is immutable.")

    @property
    def obstacle_role(self) -> ObstacleRole:
        """Obstacle role as defined in CommonRoad."""
        return self._obstacle_role

    @obstacle_role.setter
    def obstacle_role(self, obstacle_role: ObstacleRole):
        assert isinstance(
            obstacle_role, ObstacleRole
        ), "<Obstacle/obstacle_role>: argument obstacle_role of wrong " "type. Expected type: %s. Got type: %s." % (
            ObstacleRole,
            type(obstacle_role),
        )
        if not hasattr(self, "_obstacle_role"):
            self._obstacle_role = obstacle_role
        else:
            warnings.warn("<Obstacle/obstacle_role>: Obstacle role is immutable.")

    @property
    def obstacle_type(self) -> ObstacleType:
        """Obstacle type as defined in CommonRoad."""
        return self._obstacle_type

    @obstacle_type.setter
    def obstacle_type(self, obstacle_type: ObstacleType):
        assert isinstance(
            obstacle_type, ObstacleType
        ), "<Obstacle/obstacle_type>: argument obstacle_type of wrong " "type. Expected type: %s. Got type: %s." % (
            ObstacleType,
            type(obstacle_type),
        )
        if not hasattr(self, "_obstacle_type"):
            self._obstacle_type = obstacle_type
        else:
            warnings.warn("<Obstacle/obstacle_type>: Obstacle type is immutable.")

    @property
    def obstacle_shape(self) -> Union[Shape, Polygon, Circle, Rectangle]:
        """Obstacle shape as defined in CommonRoad."""
        return self._obstacle_shape

    @obstacle_shape.setter
    def obstacle_shape(self, shape: Union[Shape, Polygon, Circle, Rectangle]):
        assert isinstance(
            shape, (type(None), Shape)
        ), "<Obstacle/obstacle_shape>: argument shape of wrong type. Expected " "type %s. Got type %s." % (
            Shape,
            type(shape),
        )

        if not hasattr(self, "_obstacle_shape"):
            self._obstacle_shape = shape
        else:
            warnings.warn("<Obstacle/obstacle_shape>: Obstacle shape is immutable.")

    def occupancy_at_time(self, time_step: int) -> Occupancy:
        """
        Returns the predicted occupancy of the obstacle at a specific time step.

        :param time_step: discrete time step
        :return: occupancy of the static obstacle at time step
        """
        return Occupancy(time_step=time_step, shape=self._obstacle_shape)

    def __str__(self):
        obs_str = "Environment Obstacle:\n"
        obs_str += "\nid: {}".format(self.obstacle_id)
        return obs_str

    def draw(self, renderer: IRenderer, draw_params: OptionalSpecificOrAllDrawParams[EnvironmentObstacleParams] = None):
        renderer.draw_environment_obstacle(self, draw_params)
