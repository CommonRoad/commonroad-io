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
        """Compute the occupancy of this shape for a state; this includes usually
        translation and rotation of the shape, but
        e.g. for a semi-trailer truck, also more complex transformations."""
        pass

    @abc.abstractmethod
    def compute_occupancy_for_state_set(self, state: TraceState) -> Occupancy:
        """Same as :compute_occupancy_for_state, but for uncertain states."""
        pass

    def compute_occupancy(self, state: TraceState) -> Occupancy:
        if state.is_uncertain_position or state.is_uncertain_orientation:
            return self.compute_occupancy_for_state_set(state)
        else:
            return self.compute_occupancy_for_state(state)
