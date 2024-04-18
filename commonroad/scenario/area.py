import enum
import warnings
from typing import List, Optional, Set

import numpy as np

from commonroad.common.common_lanelet import LineMarking
from commonroad.common.validity import is_valid_polyline


class AreaType(enum.Enum):
    """
    Enum describing different types of an area
    """

    BUS_STOP = "bus_stop"
    BORDER = "border"
    PARKING = "parking"
    RESTRICTED = "restricted"
    UNKNOWN = "unknown"


class AreaBorder:
    """
    Class which describes a border of an area
    """

    def __init__(
        self,
        area_border_id: int,
        border_vertices: np.ndarray,
        adjacent: Optional[List[int]] = None,
        line_marking: Optional[LineMarking] = None,
    ):
        """
        Constructor of an AreaBorder object

        :param area_border_id: id of the area border
        :param border_vertices: array that contains the coordinates of the border's vertices
        :param adjacent: id of lanelets adjacent to the border
        :param line_marking: line marking of the area border
        """
        self._area_border_id = area_border_id
        self._border_vertices = border_vertices
        self._adjacent = adjacent
        self._line_marking = line_marking

    @property
    def area_border_id(self) -> int:
        """Id of the area border."""
        return self._area_border_id

    @area_border_id.setter
    def area_border_id(self, value: int):
        assert isinstance(value, int), "<AreaBorder/area_border_id>: Provided id is not valid! id={}".format(value)
        self._area_border_id = value

    @property
    def border_vertices(self) -> np.ndarray:
        """Array that contains the coordinates of the border's vertices."""
        return self._border_vertices

    @border_vertices.setter
    def border_vertices(self, value: np.ndarray):
        assert is_valid_polyline(
            value
        ), "<AreaBorder/border_vertices>: The provided polyline " "is not valid! id = {} polyline = {}".format(
            self._area_border_id, value
        )
        self._border_vertices = value

    @property
    def adjacent(self) -> Optional[List[int]]:
        """Id of lanelets adjacent to the border."""
        return self._adjacent

    @adjacent.setter
    def adjacent(self, val: List[int]):
        assert isinstance(val, list) and all(
            isinstance(x, int) for x in val
        ), "<AreaBorder/adjacent>: Provided adjacent is not valid! adjacent={}".format(val)
        self._adjacent = val

    @property
    def line_marking(self) -> LineMarking:
        """Line marking of the area border."""
        return self._line_marking

    @line_marking.setter
    def line_marking(self, value: LineMarking):
        assert isinstance(
            value, LineMarking
        ), "<AreaBorder/line_marking>: Provided lane marking type" "is not valid! type = {}".format(type(value))
        self._line_marking = value

    def __eq__(self, other):
        if not isinstance(other, AreaBorder):
            warnings.warn(f"Inequality between AreaBorder {repr(self)} and different type {type(other)}")
            return False
        polyline_string = np.array2string(np.around(self._border_vertices.astype(float), 10), precision=10)
        polyline_other_string = np.array2string(np.around(other.border_vertices.astype(float), 10), precision=10)
        return (
            self._area_border_id == other.area_border_id
            and polyline_string == polyline_other_string
            and self._adjacent == other.adjacent
            and self._line_marking == other.line_marking
        )

    def __hash__(self):
        return hash(
            (
                self._area_border_id,
                np.array2string(np.around(self._border_vertices.astype(float), 10), precision=10),
                self._adjacent,
                self._line_marking,
            )
        )


class Area:
    """
    Class which describes an area
    """

    def __init__(self, area_id: int, border: List[AreaBorder] = None, area_types: Set[AreaType] = None):
        """
        Constructor of an Area object.

        :param area_id: id of the area
        :param border: list of area borders corresponding to the area
        :param area_types: set of area types corresponding to the area
        """
        self._area_id = area_id
        self._border = border
        self._area_types = area_types

    @property
    def area_id(self) -> int:
        """Id of the area."""
        return self._area_id

    @area_id.setter
    def area_id(self, value: int):
        assert isinstance(value, int), "<Area/area_id>: Provided area_id is not valid! id={}".format(value)
        self._area_id = value

    @property
    def border(self) -> List[AreaBorder]:
        """List of area borders corresponding to the area."""
        return self._border

    @border.setter
    def border(self, value: List[AreaBorder]):
        assert isinstance(
            value, list
        ), "<Area/border>: provided list of area borders is not a " "list! type = {}".format(type(value))
        for val in value:
            assert isinstance(val, AreaBorder), "<Area/border>: Provided border list is not valid! id={}".format(value)
        self._border = value

    @property
    def area_types(self) -> Set[AreaType]:
        """Set of area types corresponding to the area."""
        return self._area_types

    @area_types.setter
    def area_types(self, value: Set[AreaType]):
        assert isinstance(
            value, set
        ), "<Area/area_types>: provided set of area types is not a " "set! type = {}".format(type(value))
        for val in value:
            assert isinstance(val, AreaType), "<Area/area_types>: Provided area type set is not valid! id={}".format(
                value
            )
        self._area_types = value

    def __eq__(self, other):
        if not isinstance(other, Area):
            warnings.warn(f"Inequality between Area {repr(self)} and different type {type(other)}")
            return False
        return self._area_id == other.area_id and self._border == other.border and self._area_types == other.area_types

    def __hash__(self):
        return hash(
            (self._area_id, tuple(self._border), frozenset(self._area_types if self._area_types is not None else set()))
        )
