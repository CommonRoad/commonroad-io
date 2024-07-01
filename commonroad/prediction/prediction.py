import abc
import functools
import math
import warnings
from typing import Dict, List, Optional, Set, Union

import numpy as np

from commonroad.common.util import Interval
from commonroad.common.validity import is_real_number_vector, is_valid_orientation
from commonroad.geometry.shape import (
    Circle,
    Polygon,
    Rectangle,
    Shape,
    ShapeGroup,
    occupancy_shape_from_state,
    shape_group_occupancy_shape_from_state,
)
from commonroad.scenario.trajectory import Trajectory
from commonroad.visualization.draw_params import OccupancyParams
from commonroad.visualization.drawable import IDrawable
from commonroad.visualization.renderer import IRenderer


class Occupancy(IDrawable):
    """Class describing an occupied area in the position domain. The
    occupied area can be defined for a certain time
    step or a time interval."""

    def __init__(self, time_step: Union[int, Interval], shape: Shape):
        """
        :param time_step: a time interval or time step for which the
        occupancy is defined
        :param shape: occupied region in the position domain
        """
        self.time_step: Union[int, Interval] = time_step
        self.shape: Shape = shape

    def __eq__(self, other):
        if not isinstance(other, Occupancy):
            warnings.warn(f"Inequality between Occupancy {repr(self)} and different type {type(other)}")
            return False

        return self._time_step == other.time_step and self._shape == other.shape

    def __hash__(self):
        return hash((self._time_step, self._shape))

    @property
    def shape(self) -> Union[Shape, Rectangle, Circle, Polygon, ShapeGroup]:
        """Shape representing an occupied area in the position domain."""
        return self._shape

    @shape.setter
    def shape(self, shape: Union[Shape, Rectangle, Circle, Polygon, ShapeGroup]):
        assert isinstance(
            shape, Shape
        ), '<Occupancy/shape>: argument "shape" of wrong type. Expected type: %s. ' "Got type: %s." % (
            Shape,
            type(shape),
        )
        self._shape = shape

    @property
    def time_step(self) -> Union[int, Interval]:
        """The occupied area is either defined for a certain time step or a time interval."""
        return self._time_step

    @time_step.setter
    def time_step(self, time_step: Union[int, Interval]):
        assert isinstance(
            time_step, (int, Interval)
        ), '<Occupancy/time_step>: argument "time_step" of ' "wrong type. Expected type: %s or %s. Got type: %s." % (
            int,
            Interval,
            type(time_step),
        )
        self._time_step = time_step

    def translate_rotate(self, translation: np.ndarray, angle: float):
        """Translates and rotates the occupied area.

        :param translation: translation vector [x_off, y_off] in x- and y-direction
        :param angle: rotation angle in radian (counter-clockwise)
        """
        assert is_real_number_vector(translation, 2), (
            "<Occupancy/translate_rotate>: " 'argument "translation" is ' "not a vector of real numbers of " "length 2."
        )
        assert is_valid_orientation(angle), '<Occupancy/translate_rotate>: argument "orientation" ' "is " "not valid."

        self._shape = self._shape.translate_rotate(translation, angle)

    def draw(self, renderer: IRenderer, draw_params: Optional[OccupancyParams] = None):
        self.shape.draw(renderer, draw_params.shape if draw_params is not None else None)


class Prediction(abc.ABC):
    """
    Base class for a prediction module.
    """

    @property
    @abc.abstractmethod
    def initial_time_step(self) -> int:
        """Initial time step of the prediction."""
        pass

    @property
    @abc.abstractmethod
    def final_time_step(self) -> Union[int, Interval]:
        """Final time step of the prediction."""
        pass

    @property
    @abc.abstractmethod
    def occupancy_set(self) -> List[Occupancy]:
        """List of occupancies over time."""
        pass

    def occupancy_at_time_step(self, time_step: int) -> Union[None, Occupancy]:
        """Occupancy at a specific time step.

        :param time_step: discrete time step
        :return: occupancy at time_step if time_step is within the time interval of the prediction; otherwise, None
        """
        assert isinstance(time_step, int), (
            '<Prediction/occupancy_at_time_step>: argument "time_step" of '
            "wrong type. Expected type: %s. Got type: %s." % (int, type(time_step))
        )
        for occ in self.occupancy_set:
            if isinstance(occ.time_step, Interval):
                if occ.time_step.contains(time_step):
                    return occ
            elif isinstance(occ.time_step, int):
                if occ.time_step == time_step:
                    return occ
        return None

    @abc.abstractmethod
    def translate_rotate(self, translation: list, angle: float):
        pass


class SetBasedPrediction(Prediction):
    """Class to represent the future behavior of obstacles by bounded occupancy sets."""

    _initial_time_step: int
    _occupancy_set: List[Occupancy]

    def __init__(self, initial_time_step: int, occupancy_set: List[Occupancy]):
        """
        :param initial_time_step: initial time step of the set-based prediction
        :param occupancy_set: list of occupancies defined for different time steps or time intervals.
        """
        self._initial_time_step = initial_time_step
        self._occupancy_set = occupancy_set

    def __eq__(self, other):
        if not isinstance(other, SetBasedPrediction):
            warnings.warn(f"Inequality between SetBasedPrediction {repr(self)} and different type {type(other)}")
            return False

        return self._initial_time_step == other.initial_time_step and self._occupancy_set == other.occupancy_set

    def __hash__(self):
        return hash((self._initial_time_step, frozenset(self._occupancy_set)))

    @property
    def initial_time_step(self) -> int:
        """Initial time step of the prediction."""
        return self._initial_time_step

    @property
    def final_time_step(self) -> Union[int, Interval]:
        """Final time step of the prediction."""
        return max(occ.time_step for occ in self.occupancy_set)

    @property
    def occupancy_set(self) -> List[Occupancy]:
        """List of occupancies over time."""
        return self._occupancy_set

    @occupancy_set.setter
    def occupancy_set(self, occupancy_set: List[Occupancy]):
        assert isinstance(occupancy_set, list), (
            '<Prediction/occupancy_set>: argument "occupancy_set" of wrong type. '
            "Expected type: %s. Got type: %s." % (list, type(occupancy_set))
        )
        assert all(isinstance(occupancy, Occupancy) for occupancy in occupancy_set), (
            "<Prediction/occupancy_set>: "
            'element of "occupancy_set" is '
            "of wrong type. Expected type: "
            "%s." % Occupancy
        )
        self._occupancy_set = occupancy_set

    def translate_rotate(self, translation: np.ndarray, angle: float):
        """Translates and rotates the occupancy set.

        :param translation: translation vector [x_off, y_off] in x- and y-direction
        :param angle: rotation angle in radian (counter-clockwise)
        """
        assert is_real_number_vector(translation, 2), (
            '<SetBasedPrediction/translate_rotate>: argument "translation" '
            "is not a vector of real numbers of length 2."
        )
        assert is_valid_orientation(angle), (
            '<SetBasedPrediction/translate_rotate>: argument "orientation" ' "is not valid."
        )
        for occ in self._occupancy_set:
            occ.translate_rotate(translation, angle)


class TrajectoryPrediction(Prediction):
    """Class to represent the predicted movement of an obstacle using a trajectory. A trajectory is modeled as a
    state sequence over time. The occupancy of an obstacle along a trajectory is uniquely defined given its shape."""

    _shape: Shape
    _trajectory: Trajectory
    _shape_lanelet_assignment: Optional[Dict[int, Set[int]]]
    _center_lanelet_assignment: Optional[Dict[int, Set[int]]]
    _wheelbase_lengths: Optional[List[float]]

    def __init__(
        self,
        trajectory: Trajectory,
        shape: Shape,
        center_lanelet_assignment: Optional[Dict[int, Set[int]]] = None,
        shape_lanelet_assignment: Optional[Dict[int, Set[int]]] = None,
        **kwargs,
    ):
        """
        :param trajectory: predicted trajectory of the obstacle
        :param center_lanelet_assignment: predicted lanelet assignment of obstacle center
        :param shape_lanelet_assignment: predicted lanelet assignment of obstacle shape
        :param shape: shape of the obstacle
        """
        self._shape = shape
        self._trajectory = trajectory
        self._shape_lanelet_assignment = shape_lanelet_assignment
        self._center_lanelet_assignment = center_lanelet_assignment
        self._wheelbase_lengths = None
        for field, value in kwargs.items():
            setattr(self, field, value)

    def __eq__(self, other):
        if not isinstance(other, TrajectoryPrediction):
            warnings.warn(f"Inequality between TrajectoryPrediction {repr(self)} and different type {type(other)}")
            return False

        return (
            self._shape == other.shape
            and self._trajectory == other.trajectory
            and self._center_lanelet_assignment == other.center_lanelet_assignment
            and self._shape_lanelet_assignment == other.shape_lanelet_assignment
        )

    def __hash__(self):
        center_lanelet_assignment = (
            frozenset((key, frozenset(value)) for key, value in self._center_lanelet_assignment.items())
            if self._center_lanelet_assignment is not None
            else None
        )
        shape_lanelet_assignment = (
            frozenset((key, frozenset(value)) for key, value in self._shape_lanelet_assignment.items())
            if self._shape_lanelet_assignment is not None
            else None
        )

        return hash(
            (
                self._trajectory,
                self._shape,
                center_lanelet_assignment,
                shape_lanelet_assignment,
            )
        )

    @property
    def initial_time_step(self) -> int:
        """Initial time step of the prediction."""
        return self._trajectory.initial_time_step

    @property
    def final_time_step(self) -> Union[int, Interval]:
        """Final time step of the prediction."""
        return self._trajectory.final_state.time_step

    @functools.cached_property
    def occupancy_set(self) -> List[Occupancy]:
        """List of occupancies over time."""
        return self._create_occupancy_set()

    def _invalidate_occupancy_set(self):
        # Don't use hasattr for checking whether occupancy_set has been cached, since that would always compute the property
        if "occupancy_set" in self.__dict__:
            del self.occupancy_set

    @property
    def shape(self) -> Shape:
        """Shape of the predicted object."""
        return self._shape

    @shape.setter
    def shape(self, shape: Shape):
        assert isinstance(
            shape, Shape
        ), '<TrajectoryPrediction/shape>: argument "shape" of wrong type. Expected ' "type: %s. Got type: %s." % (
            Shape,
            type(shape),
        )
        self._shape = shape
        self._invalidate_occupancy_set()

    @property
    def trajectory(self) -> Trajectory:
        """Predicted trajectory of the object."""
        return self._trajectory

    @trajectory.setter
    def trajectory(self, trajectory: Trajectory):
        assert isinstance(trajectory, Trajectory), (
            '<TrajectoryPrediction/trajectory>: argument "trajectory" of wrong'
            " type. Expected type: %s. Got type: %s." % (Trajectory, type(trajectory))
        )
        self._trajectory = trajectory
        self._invalidate_occupancy_set()

    @property
    def shape_lanelet_assignment(self) -> Union[None, Dict[int, Set[int]]]:
        """Predicted lanelet assignment of obstacle shape."""
        return self._shape_lanelet_assignment

    @shape_lanelet_assignment.setter
    def shape_lanelet_assignment(self, shape_lanelet_assignment: Union[None, Dict[int, Set[int]]]):
        if shape_lanelet_assignment is not None:
            assert isinstance(shape_lanelet_assignment, dict), (
                "<TrajectoryPrediction/shape_lanelet_assignment>: "
                'argument "shape_lanelet_assignment" of wrong type. '
                "Expected type: %s. Got"
                " type: %s." % (Dict, type(shape_lanelet_assignment))
            )
        self._shape_lanelet_assignment = shape_lanelet_assignment

    @property
    def center_lanelet_assignment(self) -> Union[None, Dict[int, Set[int]]]:
        """Predicted lanelet assignment of obstacle center."""
        return self._center_lanelet_assignment

    @center_lanelet_assignment.setter
    def center_lanelet_assignment(self, center_lanelet_assignment: Union[None, Dict[int, Set[int]]]):
        if center_lanelet_assignment is not None:
            assert isinstance(center_lanelet_assignment, dict), (
                "<TrajectoryPrediction/center_lanelet_assignment>: "
                'argument "center_lanelet_assignment" of wrong type. '
                "Expected type: "
                "%s. Got type: %s." % (Dict, type(center_lanelet_assignment))
            )
        self._center_lanelet_assignment = center_lanelet_assignment

    @property
    def wheelbase_lengths(self) -> Optional[List[float]]:
        """List of wheelbase lengths corresponding to the shape."""
        return self._wheelbase_lengths

    @wheelbase_lengths.setter
    def wheelbase_lengths(self, wheelbase_lenghts: List[float]):
        self._wheelbase_lenghts = wheelbase_lenghts
        self._invalidate_occupancy_set()

    def translate_rotate(self, translation: np.ndarray, angle: float):
        """Translates and rotates all states of the trajectory and re-computes the translated and rotated occupancy
        set.

        :param translation: translation vector [x_off, y_off] in x- and y-direction
        :param angle: rotation angle in radian (counter-clockwise)
        """
        assert is_real_number_vector(translation, 2), (
            "<TrajectoryPrediction/translate_rotate>: argument "
            '"translation" is not a vector of real numbers of length 2.'
        )
        assert is_valid_orientation(angle), (
            '<TrajectoryPrediction/translate_rotate>: argument "orientation" is ' "not valid."
        )

        self._trajectory.translate_rotate(translation, angle)

    def _create_occupancy_set(self) -> List[Occupancy]:
        """Computes the occupancy set over time given the predicted trajectory and shape of the object."""
        occupancy_set = []

        for state in self._trajectory.state_list:
            if not hasattr(state, "orientation"):
                state.orientation = math.atan2(getattr(state, "velocity_y"), state.velocity)

            if self.wheelbase_lengths is not None:
                shapes = self._shape.shapes
                occupied_region = shape_group_occupancy_shape_from_state(shapes, state, self.wheelbase_lengths)
            else:
                occupied_region = occupancy_shape_from_state(self._shape, state)

            occupancy_set.append(Occupancy(state.time_step, occupied_region))
        return occupancy_set
