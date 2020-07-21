import numpy as np

from commonroad.common.validity import *
from commonroad.geometry.shape import Polygon


class Building:
    """
    Class which describes a building according to the CommonRoad specification.
    The outline of a building is described by three points.
    """

    def __init__(self, outline: np.ndarray, building_id: int):
        """
        Constructor of a building object
        :param outline: The vertices of the outline are described by a  polyline [[x0,x1,...,xn],[y0,y1,...,yn]]
        """
        self._outline = None
        self._building_id = None
        self.outline = outline
        self.building_id = building_id
        self._shape = Polygon(outline)

    @property
    def outline(self) -> np.ndarray:
        return self._outline

    @outline.setter
    def outline(self, polyline: np.ndarray):
        if self._outline is None:
            assert is_valid_polyline(
                polyline), '<Building/outline>: The provided polyline is not valid! polyline = {}'.format(polyline)
            self._outline = polyline
        else:
            warnings.warn('<Building/outline>: outline of lanelet are immutable!')

    @property
    def building_id(self) -> int:
        return self._building_id

    @building_id.setter
    def building_id(self, b_id: int):
        if self._building_id is None:
            assert is_natural_number(b_id), '<Building/building_id>: ' \
                                            'Provided building_id is not valid! id={}'.format(b_id)
            self._building_id = b_id
        else:
            warnings.warn('<Building/building_id>: building_id of building is immutable')

    @property
    def shape(self) -> Polygon:
        return self._shape
