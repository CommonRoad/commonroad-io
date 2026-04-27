from __future__ import annotations

import functools
from dataclasses import dataclass
from typing import Tuple

import numpy as np
import shapely

from commonroad.common.util import make_valid_orientation
from commonroad.common.validity import is_valid_orientation
from commonroad.geometry.occupancy.occupancy import Occupancy
from commonroad.geometry.round_and_hash import round_and_hash
from commonroad.geometry.transform_shapely_shape import rotate_and_translate, translate_and_rotate
from commonroad.visualization.draw_params import OptionalSpecificOrAllDrawParams, ShapeParams
from commonroad.visualization.renderer import IRenderer


@dataclass(frozen=True)
class RectOccupancy(Occupancy):
    """A rectangular occupancy.

    :param rect_center: center of the rectangle
    :type rect_center: shapely.Point
    :param width: width of the rectangle
    :type width: float
    :param length: length of the rectangle
    :type length: float
    :param orientation: orientation of the rectangle in radians, counter-clockwise from x-axis
    :type orientation: float
    """

    rect_center: shapely.Point
    width: float
    length: float
    orientation: float

    def __post_init__(self):
        assert is_valid_orientation(
            self.orientation
        ), '<Rectangle/orientation>: argument "orientation" is not valid. orientation = {}'.format(
            self.orientation
        )

    def __eq__(self, other) -> bool:
        if self is other:
            return True
        if not isinstance(other, RectOccupancy):
            return False
        return (
            np.allclose(self.rect_center.xy, other.rect_center.xy)
            and np.isclose(self.width, other.width)
            and np.isclose(self.length, other.length)
            and np.isclose(self.orientation, other.orientation)
        )

    def __hash__(self) -> int:
        return round_and_hash(
            self.rect_center.x, self.rect_center.y, self.width, self.length, self.orientation
        )

    @functools.cached_property
    def vertices(self) -> Tuple[Tuple[float, float], ...]:
        """Vertices of the rectangle: ((x_0, y_0), (x_1, y_1), ...).
        The vertices are sorted clockwise and the first and last point are the same.

        :return: vertices of the rectangle
        """
        return tuple(self.shapely_object.exterior.coords)

    @functools.cached_property
    def shapely_object(self) -> shapely.geometry.Polygon:
        x = 0.5 * self.length
        y = 0.5 * self.width
        vertices = (-x, -y), (-x, y), (x, y), (x, -y)
        polygon = shapely.Polygon(vertices)
        return rotate_and_translate(
            polygon, self.orientation, self.rect_center.x, self.rect_center.y
        )

    def translate_rotate(self, xoff: float, yoff: float, angle: float) -> RectOccupancy:
        assert is_valid_orientation(angle), (
            '<Rectangle/translate_rotate>: argument "orientation" is not valid.'
            "orientation = {}".format(angle)
        )
        new_center = translate_and_rotate(self.rect_center, xoff, yoff, angle)
        new_orientation = make_valid_orientation(self.orientation + angle)
        return RectOccupancy(
            rect_center=new_center,
            width=self.width,
            length=self.length,
            orientation=new_orientation,
        )

    @property
    def center(self) -> shapely.Point:
        """Center of the rectangle.

        :return: center of the rectangle
        """
        return self.rect_center

    def draw(
        self,
        renderer: IRenderer,
        draw_params: OptionalSpecificOrAllDrawParams[ShapeParams] = None,
    ):
        renderer.draw_rectangle(self.vertices, draw_params)
