import unittest

import numpy as np
import shapely

from commonroad.geometry.occupancy.circle_occupancy import CircleOccupancy
from commonroad.geometry.occupancy.occupancy_group import OccupancyGroup
from commonroad.geometry.occupancy.polygon_occupancy import PolygonOccupancy
from commonroad.geometry.occupancy.rect_occupancy import RectOccupancy


class TestRectangle(unittest.TestCase):
    def test_rotate_90deg(self):
        translation = np.array([0.0, 0.0])
        rotation_angle = np.pi / 2

        rect = RectOccupancy(
            length=1, width=1, rect_center=shapely.Point([0.0, 0.0]), orientation=0
        )
        expected_transposed_vertices = np.array(
            [[0.5, -0.5], [-0.5, -0.5], [-0.5, 0.5], [0.5, 0.5], [0.5, -0.5]]
        )

        transposed_vector = rect.translate_rotate(translation[0], translation[1], rotation_angle)

        for tv, etv in zip(transposed_vector.vertices, expected_transposed_vertices):
            for tv_item, etv_item in zip(tv, etv):
                self.assertAlmostEqual(tv_item, etv_item)

    def test_translate(self):
        translation = np.array([5.52, -2.2])
        rotation_angle = 0

        rect = RectOccupancy(
            length=1, width=1, rect_center=shapely.Point([0.0, 0.0]), orientation=0
        )
        expected_transposed_vertices = np.array(
            [[5.02, -2.7], [5.02, -1.7], [6.02, -1.7], [6.02, -2.7], [5.02, -2.7]]
        )

        transposed_vector = rect.translate_rotate(translation[0], translation[1], rotation_angle)

        for tv, etv in zip(transposed_vector.vertices, expected_transposed_vertices):
            for tv_item, etv_item in zip(tv, etv):
                self.assertAlmostEqual(tv_item, etv_item)

    def test_contains_point(self):
        initial_rectangle = RectOccupancy(
            length=1, width=1, rect_center=shapely.Point([0.0, 0.0]), orientation=0
        )
        expected_contained_point = shapely.Point([0, 0])

        self.assertTrue(initial_rectangle.contains_point(expected_contained_point))

    def test__compute_vertices(self):
        initial_rectangle = RectOccupancy(
            length=1, width=1, rect_center=shapely.Point([0.0, 0.0]), orientation=0
        )
        expected_computed_vertices = np.array(
            [[-0.5, -0.5], [-0.5, 0.5], [0.5, 0.5], [0.5, -0.5], [-0.5, -0.5]]
        )

        computed_vertices = initial_rectangle.vertices

        for cv, ecv in zip(computed_vertices, expected_computed_vertices):
            for cv_item, ecv_item in zip(cv, ecv):
                self.assertAlmostEqual(cv_item, ecv_item)

    def test_hash(self):
        rectangle_1 = RectOccupancy(
            length=1, width=1, rect_center=shapely.Point([0.0, 0.0]), orientation=0
        )
        rectangle_2 = RectOccupancy(
            length=1, width=1, rect_center=shapely.Point([0.0, 0.0]), orientation=0
        )
        rectangle_3 = RectOccupancy(
            length=1, width=20, rect_center=shapely.Point([0.0, 0.0]), orientation=0
        )
        self.assertEqual(rectangle_1.__hash__(), rectangle_2.__hash__())
        self.assertNotEqual(rectangle_1.__hash__(), rectangle_3.__hash__())

    def test_equality(self):
        rectangle_1 = RectOccupancy(
            length=1, width=1, rect_center=shapely.Point([0.0, 0.0]), orientation=0
        )
        rectangle_2 = RectOccupancy(
            length=1, width=1, rect_center=shapely.Point([0.0, 0.0]), orientation=0
        )
        rectangle_3 = RectOccupancy(
            length=1, width=20, rect_center=shapely.Point([0.0, 0.0]), orientation=0
        )
        self.assertTrue(rectangle_1.__eq__(rectangle_2))
        self.assertFalse(rectangle_1.__eq__(rectangle_3))


class TestCircle(unittest.TestCase):
    def test_rotate_90deg(self):
        translation = np.array([0.0, 0.0])
        rotation_angle = np.pi / 2

        initial_circle = CircleOccupancy(radius=2.0, circle_center=shapely.Point([0.0, 0.0]))
        expected_transposed_center = shapely.Point([0.0, 0.0])

        transposed_circle = initial_circle.translate_rotate(
            translation[0], translation[1], rotation_angle
        )

        self.assertAlmostEqual(transposed_circle.center.x, expected_transposed_center.x)
        self.assertAlmostEqual(transposed_circle.center.y, expected_transposed_center.y)

    def test_translate(self):
        translation = np.array([5.52, -2.2])
        rotation_angle = 0

        initial_circle = CircleOccupancy(radius=2.0, circle_center=shapely.Point([0.0, 0.0]))
        expected_transposed_center = shapely.Point([5.52, -2.2])

        transposed_circle = initial_circle.translate_rotate(
            translation[0], translation[1], rotation_angle
        )

        self.assertAlmostEqual(transposed_circle.center.x, expected_transposed_center.x)
        self.assertAlmostEqual(transposed_circle.center.y, expected_transposed_center.y)

    def test_contains_point(self):
        initial_circle = CircleOccupancy(radius=0.1, circle_center=shapely.Point([0.0, 0.0]))
        expected_contained_point = shapely.Point([0, 0])

        self.assertTrue(initial_circle.contains_point(expected_contained_point))

    def test_hash(self):
        circle_1 = CircleOccupancy(radius=1.0, circle_center=shapely.Point([0.0, 0.0]))
        circle_2 = CircleOccupancy(radius=1.0, circle_center=shapely.Point([0.0, 0.0]))
        circle_3 = CircleOccupancy(radius=2.0, circle_center=shapely.Point([0.0, 0.0]))
        self.assertEqual(circle_1.__hash__(), circle_2.__hash__())
        self.assertNotEqual(circle_1.__hash__(), circle_3.__hash__())

    def test_equality(self):
        circle_1 = CircleOccupancy(radius=1.0, circle_center=shapely.Point([0.0, 0.0]))
        circle_2 = CircleOccupancy(radius=1.0, circle_center=shapely.Point([0.0, 0.0]))
        circle_3 = CircleOccupancy(radius=2.0, circle_center=shapely.Point([0.0, 0.0]))
        self.assertTrue(circle_1.__eq__(circle_2))
        self.assertFalse(circle_1.__eq__(circle_3))


class TestPolygon(unittest.TestCase):
    def test_rotate_90deg(self):
        translation = np.array([0.0, 0.0])
        rotation_angle = np.pi / 2

        poly = PolygonOccupancy(
            polygon=shapely.Polygon(np.array([[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]]))
        )
        expected_transposed_vertices = np.array(
            [[0.5, -0.5], [-0.5, -0.5], [-0.5, 0.5], [0.5, 0.5], [0.5, -0.5]]
        )

        trans_poly = poly.translate_rotate(translation[0], translation[1], rotation_angle)

        for tv, etv in zip(trans_poly.vertices, expected_transposed_vertices):
            for tv_item, etv_item in zip(tv, etv):
                self.assertAlmostEqual(tv_item, etv_item)

    def test_translate(self):
        translation = np.array([5.52, -2.2])
        rotation_angle = 0

        poly = PolygonOccupancy(
            polygon=shapely.Polygon(np.array([[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]]))
        )
        expected_transposed_vertices = np.array(
            [[5.02, -2.7], [5.02, -1.7], [6.02, -1.7], [6.02, -2.7], [5.02, -2.7]]
        )

        trans_poly = poly.translate_rotate(translation[0], translation[1], rotation_angle)

        for tv, etv in zip(trans_poly.vertices, expected_transposed_vertices):
            for tv_item, etv_item in zip(tv, etv):
                self.assertAlmostEqual(tv_item, etv_item)

    def test_contains_point(self):
        initial_rectangle = PolygonOccupancy(
            polygon=shapely.Polygon(np.array([[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]]))
        )
        expected_contained_point = shapely.Point([0, 0])

        self.assertTrue(initial_rectangle.contains_point(expected_contained_point))

    def test_hash(self):
        polygon_1 = PolygonOccupancy(
            polygon=shapely.Polygon(np.array([[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]]))
        )
        polygon_2 = PolygonOccupancy(
            polygon=shapely.Polygon(np.array([[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]]))
        )
        polygon_3 = PolygonOccupancy(
            polygon=shapely.Polygon(np.array([[-0.5, -0.5], [0.5, -0.5], [1.0, 0.5], [-0.5, 0.5]]))
        )
        self.assertEqual(polygon_1.__hash__(), polygon_2.__hash__())
        self.assertNotEqual(polygon_1.__hash__(), polygon_3.__hash__())

    def test_equality(self):
        polygon_1 = PolygonOccupancy(
            polygon=shapely.Polygon(np.array([[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]]))
        )
        polygon_2 = PolygonOccupancy(
            polygon=shapely.Polygon(np.array([[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]]))
        )
        polygon_3 = PolygonOccupancy(
            polygon=shapely.Polygon(np.array([[-0.5, -0.5], [0.5, -0.5], [1.0, 0.5], [-0.5, 0.5]]))
        )
        self.assertTrue(polygon_1.__eq__(polygon_2))
        self.assertFalse(polygon_1.__eq__(polygon_3))


class TestShapeGroup(unittest.TestCase):
    def test_rotate_90deg(self):
        translation = np.array([0.0, 0.0])
        rotation_angle = np.pi / 2

        initial_occ_one = PolygonOccupancy(
            polygon=shapely.Polygon(np.array([[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]]))
        )
        initial_occ_two = RectOccupancy(
            length=1, width=1, rect_center=shapely.Point([0, 0]), orientation=0
        )
        initial_occ_three = CircleOccupancy(radius=2, circle_center=shapely.Point([0.0, 0.0]))
        occ_group = OccupancyGroup(
            occupancies=(initial_occ_one, initial_occ_two, initial_occ_three)
        )

        expected_transposed_vertices_one = np.array(
            [[0.5, -0.5], [-0.5, -0.5], [-0.5, 0.5], [0.5, 0.5], [0.5, -0.5]]
        )
        expected_transposed_vertices_two = np.array(
            [[0.5, -0.5], [-0.5, -0.5], [-0.5, 0.5], [0.5, 0.5], [0.5, -0.5]]
        )
        expected_transposed_center_three = shapely.Point([0.0, 0.0])
        expected_transposed = list()
        expected_transposed.append(expected_transposed_vertices_one)
        expected_transposed.append(expected_transposed_vertices_two)
        expected_transposed.append(expected_transposed_center_three)

        occ_group = occ_group.translate_rotate(translation[0], translation[1], rotation_angle)

        for sg, et in zip(occ_group.occupancies[0:2], expected_transposed[0:2]):
            for sg1, et1 in zip(sg.vertices, et):
                for sg1_item, et1_item in zip(sg1, et1):
                    self.assertAlmostEqual(sg1_item, et1_item)
        self.assertAlmostEqual(occ_group.occupancies[2].center.x, expected_transposed[2].x)
        self.assertAlmostEqual(occ_group.occupancies[2].center.y, expected_transposed[2].y)

    def test_translate(self):
        translation = np.array([5.52, -2.2])
        rotation_angle = 0

        initial_occ_one = PolygonOccupancy(
            polygon=shapely.Polygon(np.array([[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]]))
        )
        initial_occ_two = RectOccupancy(
            length=1, width=1, rect_center=shapely.Point([0.0, 0.0]), orientation=0
        )
        initial_occ_three = CircleOccupancy(radius=2, circle_center=shapely.Point([0.0, 0.0]))
        occ_group = OccupancyGroup((initial_occ_one, initial_occ_two, initial_occ_three))

        expected_transposed_vertices_one = np.array(
            [[5.02, -2.7], [5.02, -1.7], [6.02, -1.7], [6.02, -2.7], [5.02, -2.7]]
        )
        expected_transposed_vertices_two = np.array(
            [[5.02, -2.7], [5.02, -1.7], [6.02, -1.7], [6.02, -2.7], [5.02, -2.7]]
        )
        expected_transposed_center_three = shapely.Point([5.52, -2.2])
        expected_transposed = list()
        expected_transposed.append(expected_transposed_vertices_one)
        expected_transposed.append(expected_transposed_vertices_two)
        expected_transposed.append(expected_transposed_center_three)

        occ_group = occ_group.translate_rotate(translation[0], translation[1], rotation_angle)

        for sg, et in zip(occ_group.occupancies[0:2], expected_transposed[0:2]):
            for sg1, et1 in zip(sg.vertices, et):
                for sg1_item, et1_item in zip(sg1, et1):
                    self.assertAlmostEqual(sg1_item, et1_item)
        self.assertAlmostEqual(occ_group.occupancies[2].center.x, expected_transposed[2].x)
        self.assertAlmostEqual(occ_group.occupancies[2].center.y, expected_transposed[2].y)

    def test_contains_point(self):
        expected_contained_point = shapely.Point([0, 0])

        initial_occ_one = PolygonOccupancy(
            polygon=shapely.Polygon(np.array([[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]]))
        )
        initial_occ_two = RectOccupancy(
            length=1, width=1, rect_center=shapely.Point([0.0, 0.0]), orientation=0
        )
        initial_occ_three = CircleOccupancy(radius=2, circle_center=shapely.Point([0.0, 0.0]))
        occ_group = OccupancyGroup(
            occupancies=(initial_occ_one, initial_occ_two, initial_occ_three)
        )

        self.assertTrue(occ_group.contains_point(expected_contained_point))

    def test_hash(self):
        occ_group_1 = OccupancyGroup(
            occupancies=(
                PolygonOccupancy(
                    polygon=shapely.Polygon(
                        np.array([[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]])
                    )
                ),
                RectOccupancy(
                    length=1, width=1, rect_center=shapely.Point([0.0, 0.0]), orientation=0
                ),
            )
        )
        occ_group_2 = OccupancyGroup(
            occupancies=(
                PolygonOccupancy(
                    polygon=shapely.Polygon(
                        np.array([[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]])
                    )
                ),
                RectOccupancy(
                    length=1, width=1, rect_center=shapely.Point([0.0, 0.0]), orientation=0
                ),
            )
        )
        occ_group_3 = OccupancyGroup(
            occupancies=(
                PolygonOccupancy(
                    polygon=shapely.Polygon(
                        np.array([[-0.5, -0.5], [0.5, -0.5], [1.0, 0.5], [-0.5, 0.5]])
                    )
                ),
                RectOccupancy(
                    length=1, width=1, rect_center=shapely.Point([0.0, 0.0]), orientation=0
                ),
                RectOccupancy(
                    length=3, width=2, rect_center=shapely.Point([0.0, 0.0]), orientation=0
                ),
            )
        )
        self.assertEqual(occ_group_1.__hash__(), occ_group_2.__hash__())
        self.assertNotEqual(occ_group_1.__hash__(), occ_group_3.__hash__())

    def test_equality(self):
        occ_group_1 = OccupancyGroup(
            occupancies=(
                PolygonOccupancy(
                    polygon=shapely.Polygon(
                        np.array([[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]])
                    )
                ),
                RectOccupancy(
                    length=1, width=1, rect_center=shapely.Point([0.0, 0.0]), orientation=0
                ),
            )
        )
        occ_group_2 = OccupancyGroup(
            occupancies=(
                PolygonOccupancy(
                    polygon=shapely.Polygon(
                        np.array([[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]])
                    )
                ),
                RectOccupancy(
                    length=1, width=1, rect_center=shapely.Point([0.0, 0.0]), orientation=0
                ),
            )
        )
        occ_group_3 = OccupancyGroup(
            occupancies=(
                PolygonOccupancy(
                    polygon=shapely.Polygon(
                        np.array([[-0.5, -0.5], [0.5, -0.5], [1.0, 0.5], [-0.5, 0.5]])
                    )
                ),
                RectOccupancy(
                    length=1, width=1, rect_center=shapely.Point([0.0, 0.0]), orientation=0
                ),
                RectOccupancy(
                    length=3, width=2, rect_center=shapely.Point([0.0, 0.0]), orientation=0
                ),
            )
        )
        self.assertTrue(occ_group_1.__eq__(occ_group_2))
        self.assertFalse(occ_group_1.__eq__(occ_group_3))


if __name__ == "__main__":
    unittest.main()
