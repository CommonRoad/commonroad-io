from dataclasses import dataclass

import shapely

from commonroad.common.util import make_valid_orientation
from commonroad.geometry.obstacle_shapes.obstacle_shape import ObstacleShape
from commonroad.geometry.occupancy.occupancy import Occupancy
from commonroad.geometry.occupancy.rect_occupancy import RectOccupancy
from commonroad.geometry.transform_shapely_shape import rotate_and_translate
from commonroad.scenario.state import TraceState


@dataclass(frozen=True)
class RectObstacleShape(ObstacleShape):
    """
    Rectangular obstacle shape.

    :param width: width of the rectangle (lateral)
    :type width: float
    :param length: length of the rectangle (longitudinal)
    :type length: float
    :param origin_x_shift: shift of the origin in x-direction relative to the rectangle center.
        Default is 0.0. Example: if the origin should be the rear axle, set origin_x_shift to -wheelbase/2
    :type origin_x_shift: float
    """

    width: float
    length: float
    origin_x_shift: float = 0.0

    def __post_init__(self):
        if abs(self.origin_x_shift) > self.length / 2:
            raise ValueError("origin_x_shift must be within the rectangle length.")

    def compute_occupancy_for_state(self, state: TraceState) -> RectOccupancy:
        orientation = make_valid_orientation(state.orientation)
        rect_center = shapely.Point(-self.origin_x_shift, 0.0)
        xoff, yoff = state.position
        rect_center = rotate_and_translate(rect_center, orientation, xoff, yoff)

        return RectOccupancy(
            rect_center=rect_center,
            width=self.width,
            length=self.length,
            orientation=orientation,
        )

    def compute_occupancy_for_state_set(self, state: TraceState) -> Occupancy:
        assert self.origin_x_shift == 0.0, "Origin other than (0.0, 0.0) not supported yet."

        from commonroad.geometry.obstacle_shapes.compute_rect_occupancy_for_state_set import (
            compute_rect_occupancy_for_state_set,
        )

        return compute_rect_occupancy_for_state_set(state, self.width, self.length)
