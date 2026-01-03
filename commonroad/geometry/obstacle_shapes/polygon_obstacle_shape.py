import dataclasses
from dataclasses import dataclass
from typing import Tuple

import numpy as np
import shapely

from commonroad.geometry.obstacle_shapes.compute_rect_occupancy_for_state_set import (
    compute_rect_occupancy_for_state_set,
)
from commonroad.geometry.obstacle_shapes.obstacle_shape import ObstacleShape
from commonroad.geometry.occupancy.occupancy import Occupancy
from commonroad.geometry.occupancy.polygon_occupancy import PolygonOccupancy
from commonroad.geometry.transform_shapely_shape import rotate_and_translate
from commonroad.scenario.state import TraceState


@dataclass(frozen=True)
class PolygonObstacleShape(ObstacleShape):
    # ordered points (clockwise or counterclockwise); the translation of the vertices is considered
    vertices: Tuple[Tuple[float, float], ...]
    _polygon: shapely.Polygon = dataclasses.field(init=False, repr=False, compare=False)

    def __post_init__(self):
        object.__setattr__(self, "_polygon", shapely.Polygon(self.vertices))
        if not self._polygon.is_valid:
            raise ValueError(
                f"<PolygonObstacleShape>: The provided vertices do not form a valid polygon: {self.vertices}"
            )

    def compute_occupancy_for_state(self, state: TraceState) -> PolygonOccupancy:
        xoff, yoff = state.position
        polygon = rotate_and_translate(self._polygon, state.orientation, xoff, yoff)
        return PolygonOccupancy(polygon=polygon)

    def compute_occupancy_for_state_set(self, state: TraceState) -> Occupancy:
        min_x, min_y, max_x, max_y = self._polygon.bounds
        l_v = np.abs(max_x - min_x)
        w_v = np.abs(max_y - min_y)
        return compute_rect_occupancy_for_state_set(state, w_v, l_v)
