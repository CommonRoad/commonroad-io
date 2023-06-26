import unittest
import numpy as np
from commonroad.scenario.area import Area, AreaBorder, AreaType
from commonroad.common.common_lanelet import LineMarking


class TestArea(unittest.TestCase):

    def test_initialize_area_border(self):
        # test the initialization without 'adjacent' and 'line_marking' parameters
        area_border = AreaBorder(1, 1)
        self.assertEqual(area_border.area_border_id, 1)
        self.assertEqual(area_border.boundary, 1)
        self.assertIsNone(area_border.adjacent)
        self.assertIsNone(area_border.line_marking)

        # test the initialization without 'line_marking' parameter
        area_border = AreaBorder(1, 1, 1)
        self.assertEqual(area_border.area_border_id, 1)
        self.assertEqual(area_border.boundary, 1)
        self.assertEqual(area_border.adjacent, 1)
        self.assertIsNone(area_border.line_marking)

        # test the initialization without 'adjacent' parameter
        area_border = AreaBorder(1, 1, line_marking=LineMarking.DASHED)
        self.assertEqual(area_border.area_border_id, 1)
        self.assertEqual(area_border.boundary, 1)
        self.assertEqual(area_border.line_marking, LineMarking.DASHED)
        self.assertIsNone(area_border.adjacent)

        # test the initialization with all parameters
        area_border = AreaBorder(1, 1, 1, LineMarking.SOLID)
        self.assertEqual(area_border.area_border_id, 1)
        self.assertEqual(area_border.boundary, 1)
        self.assertEqual(area_border.adjacent, 1)
        self.assertEqual(area_border.line_marking, LineMarking.SOLID)

    def test_basic_properties_area_border(self):
        area_border = AreaBorder(1, 1)

        # test the properties of area_border_id
        area_border.area_border_id = 2
        with self.assertRaises(AssertionError):
            area_border.area_border_id = 'a'
        self.assertEqual(area_border.area_border_id, 2)

        # test the properties of boundary
        area_border.boundary = 2
        with self.assertRaises(AssertionError):
            area_border.boundary = 'test'
        self.assertEqual(area_border.boundary, 2)

        # test the properties of adjacent
        area_border.adjacent = 1
        with self.assertRaises(AssertionError):
            area_border.adjacent = 'a'
        self.assertEqual(area_border.adjacent, 1)

        # test the properties of line_marking
        area_border.line_marking = LineMarking.DASHED
        with self.assertRaises(AssertionError):
            area_border.line_marking = 1
        with self.assertRaises(AssertionError):
            area_border.line_marking = 'dashed'
        self.assertEqual(area_border.line_marking, LineMarking.DASHED)

    def test_initialize_area(self):
        # test the initialization of the area without border and area_types
        area = Area(1)
        self.assertEqual(area.area_id, 1)
        self.assertIsNone(area.border)
        self.assertIsNone(area.area_types)

        # test the initialization of the area without area_types
        area_border_1 = AreaBorder(1, 1)
        area_border_2 = AreaBorder(2, 1)
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
        area_border_1 = AreaBorder(1, 1)
        area_border_2 = AreaBorder(2, 1)
        area_border_list = [area_border_1, area_border_2]
        area_type_set = {AreaType.BUS_STOP, AreaType.PARKING}

        area = Area(1)

        # test the properties of area_id
        area.area_id = 2
        with self.assertRaises(AssertionError):
            area.area_id = 'a'
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
