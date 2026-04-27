from __future__ import annotations

import abc
from dataclasses import dataclass

import numpy as np
import shapely

import commonroad
from commonroad.visualization.drawable import IDrawable


@dataclass(frozen=True)
class Occupancy(IDrawable):
    """Abstract class for an occupied area."""

    @property
    @abc.abstractmethod
    def shapely_object(self) -> shapely.Geometry:
        """
        Creates a shapely geometry object representing the occupancy.

        :return: the occupancy as a shapely geometry object
        """
        pass

    @abc.abstractmethod
    def translate_rotate(self, xoff: float, yoff: float, angle: float) -> Occupancy:
        """Creates a new :class:`Occupancy` by first translating and then rotating the occupancy around the origin.

        :param xoff: offset in x-direction
        :param yoff: offset in y-direction
        :param angle: rotation angle in radian (counter-clockwise)
        :return: transformed occupancy
        """
        pass

    def translate(self, xoff: float, yoff: float) -> Occupancy:
        """Creates a new :class:`Occupancy` by translating the occupancy.

        :param xoff: offset in x-direction
        :param yoff: offset in y-direction
        :return: translated occupancy
        """
        return self.translate_rotate(xoff, yoff, 0.0)

    def rotate(self, angle: float) -> Occupancy:
        """Creates a new :class:`Occupancy` by rotating the occupancy around the origin.

        :param angle: rotation angle in radian (counter-clockwise)
        :return: rotated occupancy
        """
        return self.translate_rotate(0.0, 0.0, angle)

    def contains_point(self, point: shapely.Point) -> bool:
        """Checks if a point is contained in this occupancy.

        :param point: 2D point
        :return: true if the occupancy’s interior or boundary intersects with the given point, otherwise false
        """
        return shapely.intersects(self.shapely_object, point)

    @property
    def center(self) -> shapely.Point:
        """Computes the center point of the occupancy.

        :return: center point of the occupancy
        """
        return shapely.centroid(self.shapely_object)

    def enclosing_axis_aligned_rect(
        self,
    ) -> "commonroad.geometry.occupancy.rect_occupancy.RectOccupancy":
        """Computes the axis-aligned bounding rectangle of the occupancy.

        :return: axis-aligned bounding rectangle
        """
        min_x, min_y, max_x, max_y = shapely.bounds(self.shapely_object)
        length = np.abs(max_x - min_x)
        width = np.abs(max_y - min_y)

        import commonroad.geometry.occupancy.rect_occupancy as absolute_rect  # avoid circular import

        return absolute_rect.RectOccupancy(
            rect_center=self.center, width=width, length=length, orientation=0.0
        )
