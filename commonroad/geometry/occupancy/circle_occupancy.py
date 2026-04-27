from __future__ import annotations

import functools
from dataclasses import dataclass

import numpy as np
import shapely

from commonroad.geometry.occupancy.occupancy import Occupancy
from commonroad.geometry.occupancy.rect_occupancy import RectOccupancy
from commonroad.geometry.round_and_hash import round_and_hash
from commonroad.geometry.transform_shapely_shape import translate_and_rotate
from commonroad.visualization.draw_params import OptionalSpecificOrAllDrawParams, ShapeParams
from commonroad.visualization.renderer import IRenderer


@dataclass(frozen=True)
class CircleOccupancy(Occupancy):
    """Occupancy of a circle shape.

    :param radius: Radius of the circle.
    :type radius: float
    :param circle_center: Center point of the circle.
    :type circle_center: shapely.Point
    """

    radius: float
    circle_center: shapely.Point

    def __eq__(self, other) -> bool:
        if self is other:
            return True
        if not isinstance(other, CircleOccupancy):
            return False
        return np.allclose(self.circle_center.xy, other.circle_center.xy) and np.isclose(
            self.radius, other.radius
        )

    def __hash__(self) -> int:
        return round_and_hash(self.circle_center.x, self.circle_center.y, self.radius)

    @functools.cached_property
    def shapely_object(self) -> shapely.geometry.Polygon:
        return self.circle_center.buffer(self.radius / 2)

    def translate_rotate(self, xoff: float, yoff: float, angle: float) -> CircleOccupancy:
        return CircleOccupancy(
            radius=self.radius,
            circle_center=translate_and_rotate(self.circle_center, xoff, yoff, angle),
        )

    def contains_point(self, point: shapely.Point) -> bool:
        return shapely.distance(self.circle_center, point) <= self.radius

    @property
    def center(self) -> shapely.Point:
        return self.circle_center

    def enclosing_axis_aligned_rect(self) -> RectOccupancy:
        size = 2.0 * self.radius
        return RectOccupancy(
            rect_center=self.circle_center, width=size, length=size, orientation=0.0
        )

    def draw(
        self,
        renderer: IRenderer,
        draw_params: OptionalSpecificOrAllDrawParams[ShapeParams] = None,
    ):
        renderer.draw_ellipse(
            (self.circle_center.x, self.circle_center.y), self.radius, self.radius, draw_params
        )
