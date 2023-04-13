import enum
from typing import Set, List

import numpy as np

from commonroad.common.validity import is_valid_polyline
from commonroad.common.common_lanelet import LineMarking


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
    def __init__(self, area_border_id: int, border_vertices: np.ndarray, adjacent: int = None,
                 line_marking: LineMarking = None):
        """
        Constructor of an AreaBorder object

        :param area_border_id: id of the area border
        :param border_vertices: array that contains the coordinates of the border's vertices
        :param adjacent: id of the lanelet adjacent to the border
        :param line_marking: line marking of the area border
        """
        self._area_border_id = area_border_id
        self._border_vertices = border_vertices
        self._adjacent = adjacent
        self._line_marking = line_marking

    @property
    def area_border_id(self) -> int:
        """ Id of the area border."""
        return self._area_border_id

    @area_border_id.setter
    def area_border_id(self, value: int):
        assert isinstance(value, int), '<AreaBorder/area_border_id>: Provided id is not valid! id={}'.format(value)
        self._area_border_id = value

    @property
    def border_vertices(self) -> np.ndarray:
        """ Array that contains the coordinates of the border's vertices."""
        return self._border_vertices

    @border_vertices.setter
    def border_vertices(self, value: np.ndarray):
        assert is_valid_polyline(value), '<AreaBorder/border_vertices>: The provided polyline ' \
                                            'is not valid! id = {} polyline = {}'.format(self._area_border_id, value)
        self._border_vertices = value

    @property
    def adjacent(self) -> int:
        """ Id of the lanelet adjacent to the border."""
        return self._adjacent

    @adjacent.setter
    def adjacent(self, val: int):
        assert isinstance(val, int), '<AreaBorder/adjacent>: Provided adjacent is not valid! adjacent={}'.format(val)
        self._adjacent = val

    @property
    def line_marking(self) -> LineMarking:
        """ Line marking of the area border."""
        return self._line_marking

    @line_marking.setter
    def line_marking(self, value: LineMarking):
        assert isinstance(value, LineMarking), '<AreaBorder/line_marking>: Provided lane marking type' \
                                               'is not valid! type = {}'.format(type(value))
        self._line_marking = value


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
        """ Id of the area."""
        return self._area_id

    @area_id.setter
    def area_id(self, value: int):
        assert isinstance(value, int), '<Area/area_id>: Provided area_id is not valid! id={}'.format(value)
        self._area_id = value

    @property
    def border(self) -> List[AreaBorder]:
        """ List of area borders corresponding to the area."""
        return self._border

    @border.setter
    def border(self, value: List[AreaBorder]):
        assert isinstance(value, list), '<Area/border>: provided list of area borders is not a ' \
                                       'list! type = {}'.format(type(value))
        for val in value:
            assert isinstance(val, AreaBorder), '<Area/border>: Provided border list is not valid! id={}'.format(value)
        self._border = value

    @property
    def area_types(self) -> Set[AreaType]:
        """ Set of area types corresponding to the area."""
        return self._area_types

    @area_types.setter
    def area_types(self, value: Set[AreaType]):
        assert isinstance(value, set), '<Area/area_types>: provided set of area types is not a ' \
                                              'set! type = {}'.format(type(value))
        for val in value:
            assert isinstance(val, AreaType), '<Area/area_types>: Provided area type set is not valid! id={}'\
                .format(value)
        self._area_types = value
