from __future__ import annotations

import functools
from dataclasses import dataclass
from typing import Tuple

import shapely.geometry

from commonroad.common.validity import is_valid_orientation
from commonroad.geometry.occupancy.occupancy import Occupancy
from commonroad.visualization.draw_params import OptionalSpecificOrAllDrawParams, ShapeParams
from commonroad.visualization.renderer import IRenderer


@dataclass(frozen=True)
class OccupancyGroup(Occupancy):
    occupancies: Tuple[Occupancy, ...]

    def __hash__(self) -> int:
        return hash(self.occupancies)

    @functools.cached_property
    def shapely_object(self) -> shapely.Geometry:
        # the buffer(0) is to fix potential invalid geometries
        return shapely.geometry.GeometryCollection(
            [occ.shapely_object for occ in self.occupancies]
        ).buffer(0)

    def contains_point(self, point: shapely.Point) -> bool:
        return any(s.contains_point(point) for s in self.occupancies)

    def translate_rotate(self, xoff: float, yoff: float, angle: float) -> OccupancyGroup:
        assert is_valid_orientation(angle), (
            '<OccupancyGroup/translate_rotate>: argument "orientation" is not valid.'
            "orientation = {}".format(angle)
        )
        return OccupancyGroup(
            occupancies=tuple(s.translate_rotate(xoff, yoff, angle) for s in self.occupancies)
        )

    def draw(
        self,
        renderer: IRenderer,
        draw_params: OptionalSpecificOrAllDrawParams[ShapeParams] = None,
    ):
        for s in self.occupancies:
            s.draw(renderer, draw_params)
