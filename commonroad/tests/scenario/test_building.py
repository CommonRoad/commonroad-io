import unittest
import numpy as np

from commonroad.scenario.building import Building
from commonroad.geometry.shape import Polygon


class TestBuilding(unittest.TestCase):
    def setUp(self) -> None:
        self._outline = np.array([[0, 0], [1, 0], [2, 0], [3, .5], [4, 1], [4, 2], [3, 1], [2, 1], [1, 1]])
        self._building_id = 1
        self._building = Building(self._outline, self._building_id)

    def test_initialize_building(self):
        self.assertEqual(self._building.building_id, self._building_id)
        np.testing.assert_array_almost_equal(self._building.outline, self._outline)

    def test_shape(self):
        shape = Polygon(self._outline)
        np.testing.assert_array_almost_equal(self._building.shape.vertices, shape.vertices)
