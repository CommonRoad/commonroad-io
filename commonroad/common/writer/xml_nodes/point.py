from typing import Union

import numpy as np
from lxml import etree

from commonroad.common.writer.xml_nodes.float_to_str import float_to_str


class Point:
    def __init__(
        self, x: Union[int, float], y: Union[int, float], z: Union[int, float, None] = None
    ):
        self.x: Union[int, float] = x
        self.y: Union[int, float] = y
        self.z: Union[int, float] = z

    def as_numpy_array(self):
        if self.z is None:
            return np.array([self.x, self.y])
        else:
            return np.array([self.x, self.y, self.z])

    @classmethod
    def create_from_numpy_array(cls, point: Union[np.array, list]):
        assert 2 <= len(point) <= 3
        if len(point) == 2:
            return cls(point[0], point[1])
        else:
            return cls(point[0], point[1], point[2])

    def create_node(self):
        point_node = etree.Element("point")
        x = etree.Element("x")
        x.text = float_to_str(np.float64(self.x))
        point_node.append(x)
        y = etree.Element("y")
        y.text = float_to_str(np.float64(self.y))
        point_node.append(y)
        if self.z is not None:
            z = etree.Element("z")
            z.text = float_to_str(np.float64(self.z))
            point_node.append(z)
        return point_node
