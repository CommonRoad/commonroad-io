from __future__ import annotations

import functools
import itertools
from dataclasses import dataclass
from typing import Tuple

import shapely

from commonroad.common.validity import is_valid_orientation
from commonroad.geometry.occupancy.occupancy import Occupancy
from commonroad.geometry.round_and_hash import round_and_hash
from commonroad.geometry.transform_shapely_shape import translate_and_rotate
from commonroad.visualization.draw_params import OptionalSpecificOrAllDrawParams, ShapeParams
from commonroad.visualization.renderer import IRenderer


@dataclass(frozen=True)
class PolygonOccupancy(Occupancy):
    """Occupancy represented by a polygon.

    :param polygon: The polygon representing the occupancy.
    :type polygon: shapely.geometry.Polygon
    """

    polygon: shapely.geometry.Polygon

    def __post_init__(self):
        # ensure that vertices are sorted clockwise
        object.__setattr__(
            self, "polygon", shapely.geometry.polygon.orient(self.polygon, sign=-1.0)
        )

    def __eq__(self, other) -> bool:
        if self is other:
            return True
        if not isinstance(other, PolygonOccupancy):
            return False
        return self.polygon.equals_exact(other.polygon, tolerance=1e-9)

    def __hash__(self) -> int:
        return hash(round_and_hash(*itertools.chain(*self.vertices)))

    @functools.cached_property
    def vertices(self) -> Tuple[Tuple[float, float], ...]:
        """Returns the vertices of the polygon as a tuple of (x, y) coordinates.

        :return: A tuple of (x, y) coordinates representing the vertices of the polygon.
        """
        return tuple(self.shapely_object.exterior.coords)

    @property
    def shapely_object(self) -> shapely.geometry.Polygon:
        return self.polygon

    def translate_rotate(self, xoff: float, yoff: float, angle: float) -> PolygonOccupancy:
        assert is_valid_orientation(angle), (
            '<Rectangle/translate_rotate>: argument "orientation" is not valid.'
            "orientation = {}".format(angle)
        )
        return PolygonOccupancy(polygon=translate_and_rotate(self.polygon, xoff, yoff, angle))

    def draw(
        self,
        renderer: IRenderer,
        draw_params: OptionalSpecificOrAllDrawParams[ShapeParams] = None,
    ):
        renderer.draw_polygon(self.vertices, draw_params)
