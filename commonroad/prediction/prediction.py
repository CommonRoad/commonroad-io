import abc
import functools
import math
import warnings
from typing import Dict, Optional, Set, Union

import numpy as np

from commonroad.common.util import Interval
from commonroad.common.validity import is_real_number_vector, is_valid_orientation
from commonroad.geometry.obstacle_shapes.obstacle_shape import ObstacleShape
from commonroad.geometry.occupancy.occupancy import Occupancy
from commonroad.scenario.trajectory import Trajectory

TimeType = Union[int, Interval]


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
    def final_time_step(self) -> TimeType:
        """Final time step of the prediction."""
        pass

    # TODO: maybe use the type frozendict (https://pypi.org/project/frozendict), it is immutable
    @property
    @abc.abstractmethod
    def occupancies(self) -> Dict[TimeType, Occupancy]:
        """Occupancies over time."""
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
        if time_step in self.occupancies:
            return self.occupancies[time_step]

        for t, occ in self.occupancies.items():
            if isinstance(t, Interval):
                if t.contains(time_step):
                    return occ
        return None

    @abc.abstractmethod
    def translate_rotate(self, translation: list, angle: float):
        pass


class SetBasedPrediction(Prediction):
    """Class to represent the future behavior of obstacles by bounded occupancy sets."""

    _initial_time_step: int
    _occupancies: Dict[TimeType, Occupancy]

    def __init__(self, initial_time_step: int, occupancies: Dict[TimeType, Occupancy]):
        """
        :param initial_time_step: initial time step of the set-based prediction
        :param occupancies: list of occupancies defined for different time steps or time intervals.
        """
        self._initial_time_step = initial_time_step
        self._occupancies = occupancies

    def __eq__(self, other):
        if not isinstance(other, SetBasedPrediction):
            warnings.warn(
                f"Inequality between SetBasedPrediction {repr(self)} and different type {type(other)}"
            )
            return False

        return (
            self._initial_time_step == other.initial_time_step
            and self._occupancies == other._occupancies
        )

    def __hash__(self):
        return hash((self._initial_time_step, frozenset(self._occupancies.items())))

    @property
    def initial_time_step(self) -> int:
        """Initial time step of the prediction."""
        return self._initial_time_step

    @property
    def final_time_step(self) -> TimeType:
        """Final time step of the prediction."""
        return max(self._occupancies.keys())

    @property
    def occupancies(self) -> Dict[TimeType, Occupancy]:
        """List of occupancies over time."""
        return self._occupancies

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
        self._occupancies = {
            time_step: occ.translate_rotate(translation[0], translation[1], angle)
            for time_step, occ in self._occupancies.items()
        }


class TrajectoryPrediction(Prediction):
    """Class to represent the predicted movement of an obstacle using a trajectory. A trajectory is modeled as a
    state sequence over time. The occupancy of an obstacle along a trajectory is uniquely defined given its shape."""

    _shape: ObstacleShape
    _trajectory: Trajectory
    _shape_lanelet_assignment: Optional[Dict[int, Set[int]]]
    _center_lanelet_assignment: Optional[Dict[int, Set[int]]]

    def __init__(
        self,
        trajectory: Trajectory,
        shape: ObstacleShape,
        center_lanelet_assignment: Optional[Dict[int, Set[int]]] = None,
        shape_lanelet_assignment: Optional[Dict[int, Set[int]]] = None,
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

    def __eq__(self, other):
        if not isinstance(other, TrajectoryPrediction):
            warnings.warn(
                f"Inequality between TrajectoryPrediction {repr(self)} and different type {type(other)}"
            )
            return False

        return (
            self._shape == other.shape
            and self._trajectory == other.trajectory
            and self._center_lanelet_assignment == other.center_lanelet_assignment
            and self._shape_lanelet_assignment == other.shape_lanelet_assignment
        )

    def __hash__(self):
        center_lanelet_assignment = (
            frozenset(
                (key, frozenset(value)) for key, value in self._center_lanelet_assignment.items()
            )
            if self._center_lanelet_assignment is not None
            else None
        )
        shape_lanelet_assignment = (
            frozenset(
                (key, frozenset(value)) for key, value in self._shape_lanelet_assignment.items()
            )
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
    def occupancies(self) -> Dict[int, Occupancy]:
        """List of occupancies over time."""
        return self._create_occupancies()

    def _invalidate_occupancy_set(self):
        # Don't use hasattr for checking whether occupancies has been cached, since that would always compute the property
        if "occupancies" in self.__dict__:
            del self.occupancies

    @property
    def shape(self) -> ObstacleShape:
        """Shape of the predicted object."""
        return self._shape

    @shape.setter
    def shape(self, shape: ObstacleShape):
        assert isinstance(shape, ObstacleShape), (
            '<TrajectoryPrediction/shape>: argument "shape" of wrong type. Expected '
            "type: %s. Got type: %s."
            % (
                ObstacleShape,
                type(shape),
            )
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
    def center_lanelet_assignment(
        self, center_lanelet_assignment: Union[None, Dict[int, Set[int]]]
    ):
        if center_lanelet_assignment is not None:
            assert isinstance(center_lanelet_assignment, dict), (
                "<TrajectoryPrediction/center_lanelet_assignment>: "
                'argument "center_lanelet_assignment" of wrong type. '
                "Expected type: "
                "%s. Got type: %s." % (Dict, type(center_lanelet_assignment))
            )
        self._center_lanelet_assignment = center_lanelet_assignment

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

    def _create_occupancies(self) -> Dict[int, Occupancy]:
        """Computes the occupancy for each time step given the predicted trajectory and shape of the object."""
        return {
            state.time_step: self._shape.compute_occupancy(with_orientation(state))
            for state in self._trajectory.state_list
        }


def with_orientation(state):
    if not hasattr(state, "orientation"):
        state.orientation = math.atan2(getattr(state, "velocity_y"), state.velocity)
    return state
