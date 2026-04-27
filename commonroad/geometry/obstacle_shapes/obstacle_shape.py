import abc
from dataclasses import dataclass

from commonroad.geometry.occupancy.occupancy import Occupancy
from commonroad.scenario.state import TraceState


@dataclass(frozen=True)
class ObstacleShape:
    """Abstract class for shapes without a position or orientation. Used for
    describing the shapes of obstacles."""

    @abc.abstractmethod
    def compute_occupancy_for_state(self, state: TraceState) -> Occupancy:
        """Compute the occupancy that this shape has if the obstacle is in the given state;
        this includes usually translation and rotation of the shape.

        :param state: state for which to compute the occupancy
        :return: occupancy of the shape in the given state
        """
        pass

    @abc.abstractmethod
    def compute_occupancy_for_state_set(self, state: TraceState) -> Occupancy:
        """Same as :meth:`compute_occupancy_for_state`, but for uncertain states.

        :param state: uncertain state for which to compute the occupancy
        :return: occupancy of the shape in the given uncertain state
        """
        pass

    def compute_occupancy(self, state: TraceState) -> Occupancy:
        """
        Checks if the provided state is uncertain and calls the
        appropriate method to compute the occupancy.

        :param state: state or uncertain state for which to compute the occupancy
        :return: occupancy of the shape in the given state
        """
        if state.is_uncertain_position or state.is_uncertain_orientation:
            return self.compute_occupancy_for_state_set(state)
        else:
            return self.compute_occupancy_for_state(state)
