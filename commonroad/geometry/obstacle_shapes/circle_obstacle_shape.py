from dataclasses import dataclass

import shapely

from commonroad.geometry.obstacle_shapes.compute_rect_occupancy_for_state_set import (
    compute_rect_occupancy_for_state_set,
)
from commonroad.geometry.obstacle_shapes.obstacle_shape import ObstacleShape
from commonroad.geometry.occupancy.circle_occupancy import CircleOccupancy
from commonroad.geometry.occupancy.occupancy import Occupancy
from commonroad.scenario.state import TraceState


@dataclass(frozen=True)
class CircleObstacleShape(ObstacleShape):
    """
    Circle obstacle shape defined by its radius. The origin is at the center of the circle.
    """

    radius: float

    def compute_occupancy_for_state(self, state: TraceState) -> CircleOccupancy:
        return CircleOccupancy(radius=self.radius, circle_center=shapely.Point(state.position))

    def compute_occupancy_for_state_set(self, state: TraceState) -> Occupancy:
        size = 2.0 * self.radius
        return compute_rect_occupancy_for_state_set(state, size, size)
