import unittest

import numpy as np

from commonroad.common.common_lanelet import LineMarking
from commonroad.scenario.area import Area, AreaBorder, AreaType


class TestArea(unittest.TestCase):
    def test_initialize_area_border(self):
        # test the initialization without 'adjacent' and 'line_marking' parameters
        area_border = AreaBorder(1, np.array([[1, 2], [3, 4]]))
        self.assertEqual(area_border.area_border_id, 1)
        self.assertEqual(area_border.border_vertices.tolist(), np.array([[1, 2], [3, 4]]).tolist())
        self.assertIsNone(area_border.adjacent)
        self.assertIsNone(area_border.line_marking)

        # test the initialization without 'line_marking' parameter
        area_border = AreaBorder(1, np.array([[1, 2], [3, 4]]), 1)
        self.assertEqual(area_border.area_border_id, 1)
        self.assertEqual(area_border.border_vertices.all(), np.array([[1, 2], [3, 4]]).all())
        self.assertEqual(area_border.adjacent, 1)
        self.assertIsNone(area_border.line_marking)

        # test the initialization without 'adjacent' parameter
        area_border = AreaBorder(1, np.array([[1, 2], [3, 4]]), line_marking=LineMarking.DASHED)
        self.assertEqual(area_border.area_border_id, 1)
        self.assertEqual(area_border.border_vertices.tolist(), np.array([[1, 2], [3, 4]]).tolist())
        self.assertEqual(area_border.line_marking, LineMarking.DASHED)
        self.assertIsNone(area_border.adjacent)

        # test the initialization with all parameters
        area_border = AreaBorder(1, np.array([[1, 2], [3, 4]]), 1, LineMarking.SOLID)
        self.assertEqual(area_border.area_border_id, 1)
        self.assertEqual(area_border.border_vertices.tolist(), np.array([[1, 2], [3, 4]]).tolist())
        self.assertEqual(area_border.adjacent, 1)
        self.assertEqual(area_border.line_marking, LineMarking.SOLID)

    def test_basic_properties_area_border(self):
        area_border = AreaBorder(1, np.array([[1, 2], [3, 4]]))

        # test the properties of area_border_id
        area_border.area_border_id = 2
        with self.assertRaises(AssertionError):
            area_border.area_border_id = "a"
        self.assertEqual(area_border.area_border_id, 2)

        # test the properties of border_vertices
        area_border.border_vertices = np.array([[1, 2, 3], [4, 5, 6]])
        with self.assertRaises(AssertionError):
            area_border.border_vertices = np.array([[1, 2, 3, 4]])
        self.assertEqual(area_border.border_vertices.all(), np.array([[1, 2, 3], [4, 5, 6]]).all())

        # test the properties of adjacent
        area_border.adjacent = 1
        with self.assertRaises(AssertionError):
            area_border.adjacent = "a"
        self.assertEqual(area_border.adjacent, 1)

        # test the properties of line_marking
        area_border.line_marking = LineMarking.DASHED
        with self.assertRaises(AssertionError):
            area_border.line_marking = 1
        with self.assertRaises(AssertionError):
            area_border.line_marking = "dashed"
        self.assertEqual(area_border.line_marking, LineMarking.DASHED)

    def test_initialize_area(self):
        # test the initialization of the area without border and area_types
        area = Area(1)
        self.assertEqual(area.area_id, 1)
        self.assertIsNone(area.border)
        self.assertIsNone(area.area_types)

        # test the initialization of the area without area_types
        area_border_1 = AreaBorder(1, np.array([[1, 2], [3, 4]]))
        area_border_2 = AreaBorder(2, np.array([[1, 2], [3, 4]]))
        area_border_list = [area_border_1, area_border_2]
        area = Area(1, area_border_list)
        self.assertEqual(area.area_id, 1)
        self.assertEqual(area.border, area_border_list)
        self.assertIsNone(area.area_types)

        # test the initialization of the area without border
        area_type_set = {AreaType.BUS_STOP, AreaType.PARKING}
        area = Area(1, area_types=area_type_set)
        self.assertEqual(area.area_id, 1)
        self.assertIsNone(area.border)
        self.assertEqual(area.area_types, area_type_set)

        # test the initialization of the area with all parameters
        area_type_set = {AreaType.BUS_STOP, AreaType.PARKING}
        area = Area(1, area_border_list, area_type_set)
        self.assertEqual(area.area_id, 1)
        self.assertEqual(area.border, area_border_list)
        self.assertEqual(area.area_types, area_type_set)

    def test_basic_properties_area(self):
        area_border_1 = AreaBorder(1, np.array([[1, 2], [3, 4]]))
        area_border_2 = AreaBorder(2, np.array([[1, 2], [3, 4]]))
        area_border_list = [area_border_1, area_border_2]
        area_type_set = {AreaType.BUS_STOP, AreaType.PARKING}

        area = Area(1)

        # test the properties of area_id
        area.area_id = 2
        with self.assertRaises(AssertionError):
            area.area_id = "a"
        self.assertEqual(area.area_id, 2)

        # test the properties of border
        area.border = area_border_list
        integer_list = [1, 2]
        with self.assertRaises(AssertionError):
            area.border = integer_list
        self.assertEqual(area.border, area_border_list)

        # test the properties of area_types
        area.area_types = area_type_set
        integer_set = {1, 2}
        with self.assertRaises(AssertionError):
            area.area_types = integer_set
        self.assertEqual(area.area_types, area_type_set)

    def test_hash_area_border(self):
        area_border1 = AreaBorder(1, np.array([[1, 2], [3, 4]]))
        area_border2 = AreaBorder(1, np.array([[1, 2], [3, 4]]))
        self.assertEqual(area_border1.__hash__(), area_border2.__hash__())

        area_border1.line_marking = LineMarking.SOLID
        self.assertNotEqual(area_border1.__hash__(), area_border2.__hash__())

    def test_hash_area(self):
        area1 = Area(1, [AreaBorder(1, np.array([[1, 2], [3, 4]])), AreaBorder(2, np.array([[1, 2], [3, 4]]))])
        area2 = Area(1, [AreaBorder(1, np.array([[1, 2], [3, 4]])), AreaBorder(2, np.array([[1, 2], [3, 4]]))])
        self.assertEqual(area1.__hash__(), area2.__hash__())

        area1.area_types = {AreaType.BUS_STOP, AreaType.PARKING}
        self.assertNotEqual(area1.__hash__(), area2.__hash__())

    def test_equality_area_border(self):
        area_border1 = AreaBorder(1, np.array([[1, 2], [3, 4]]))
        area_border2 = AreaBorder(1, np.array([[1, 2], [3, 4]]))
        self.assertTrue(area_border1 == area_border2)

        area_border1.line_marking = LineMarking.SOLID
        self.assertFalse(area_border1 == area_border2)

    def test_equality_area(self):
        area1 = Area(1, [AreaBorder(1, np.array([[1, 2], [3, 4]])), AreaBorder(2, np.array([[1, 2], [3, 4]]))])
        area2 = Area(1, [AreaBorder(1, np.array([[1, 2], [3, 4]])), AreaBorder(2, np.array([[1, 2], [3, 4]]))])
        self.assertTrue(area1 == area2)

        area1.area_types = {AreaType.BUS_STOP, AreaType.PARKING}
        self.assertFalse(area1 == area2)
