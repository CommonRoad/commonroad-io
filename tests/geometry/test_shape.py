import unittest

import numpy as np

from commonroad.geometry.shape import Circle, Polygon, Rectangle, ShapeGroup


class TestRectangle(unittest.TestCase):
    def test_rotate_90deg(self):
        translation = np.array([0.0, 0.0])
        rotation_angle = np.pi / 2

        homogeneous_initial_vector = Rectangle(1, 1)
        expected_transposed_vertices = np.array(
            [[0.5, -0.5], [-0.5, -0.5], [-0.5, 0.5], [0.5, 0.5], [0.5, -0.5]]
        )

        transposed_vector = homogeneous_initial_vector.translate_rotate(translation, rotation_angle)

        for tv, etv in zip(transposed_vector.vertices, expected_transposed_vertices):
            for tv_item, etv_item in zip(tv, etv):
                self.assertAlmostEqual(tv_item, etv_item)

    def test_translate(self):
        translation = np.array([5.52, -2.2])
        rotation_angle = 0

        homogeneous_initial_vector = Rectangle(1, 1)
        expected_transposed_vertices = np.array(
            [[5.02, -2.7], [5.02, -1.7], [6.02, -1.7], [6.02, -2.7], [5.02, -2.7]]
        )

        transposed_vector = homogeneous_initial_vector.translate_rotate(translation, rotation_angle)

        for tv, etv in zip(transposed_vector.vertices, expected_transposed_vertices):
            for tv_item, etv_item in zip(tv, etv):
                self.assertAlmostEqual(tv_item, etv_item)

    def test_contains_point(self):
        initial_rectangle = Rectangle(1, 1)
        expected_contained_point = np.array([0, 0])

        self.assertTrue(initial_rectangle.contains_point(expected_contained_point))

    def test_side_effect(self):
        # until commonroad-io 2021.4 there was a side effect where rectangle_2.center == np.ndarray([1.0, 0.0])
        rectangle_1 = Rectangle(length=4.0, width=2.0, orientation=1.0)
        rectangle_1.center[0] = 1.0
        rectangle_2 = Rectangle(length=4.0, width=2.0, orientation=1.0)
        np.testing.assert_array_equal(np.array([0.0, 0.0]), rectangle_2.center)

    def test__compute_vertices(self):
        initial_rectangle = Rectangle(1, 1)
        expected_computed_vertices = np.array(
            [[-0.5, -0.5], [-0.5, 0.5], [0.5, 0.5], [0.5, -0.5], [-0.5, -0.5]]
        )

        computed_vertices = initial_rectangle._compute_vertices()

        for cv, ecv in zip(computed_vertices, expected_computed_vertices):
            for cv_item, ecv_item in zip(cv, ecv):
                self.assertAlmostEqual(cv_item, ecv_item)

    def test_hash(self):
        rectangle_1 = Rectangle(1, 1)
        rectangle_2 = Rectangle(1, 1)
        rectangle_3 = Rectangle(1, 2)
        self.assertEqual(rectangle_1.__hash__(), rectangle_2.__hash__())
        self.assertNotEqual(rectangle_1.__hash__(), rectangle_3.__hash__())

    def test_equality(self):
        rectangle_1 = Rectangle(1, 1)
        rectangle_2 = Rectangle(1, 1)
        rectangle_3 = Rectangle(1, 2)
        self.assertTrue(rectangle_1.__eq__(rectangle_2))
        self.assertFalse(rectangle_1.__eq__(rectangle_3))


class TestCircle(unittest.TestCase):
    def test_rotate_90deg(self):
        translation = np.array([0.0, 0.0])
        rotation_angle = np.pi / 2

        initial_circle = Circle(2)
        expected_transposed_center = np.array([0.0, 0.0])

        transposed_circle = initial_circle.translate_rotate(translation, rotation_angle)

        for tc, etc in zip(transposed_circle.center, expected_transposed_center):
            self.assertAlmostEqual(tc, etc)

    def test_translate(self):
        translation = np.array([5.52, -2.2])
        rotation_angle = 0

        initial_circle = Circle(2)
        expected_transposed_center = np.array([5.52, -2.2])

        transposed_circle = initial_circle.translate_rotate(translation, rotation_angle)

        for tc, etc in zip(transposed_circle.center, expected_transposed_center):
            self.assertAlmostEqual(tc, etc)

    def test_side_effect(self):
        # until commonroad-io 2021.4 there was a side effect where circle_2.center == np.ndarray([1.0, 0.0])
        circle_1 = Circle(radius=1.0)
        circle_1.center[0] = 1.0
        circle_2 = Circle(radius=1.0)
        np.testing.assert_array_equal(np.array([0.0, 0.0]), circle_2.center)

    def test_contains_point(self):
        initial_circle = Rectangle(0, 0)
        expected_contained_point = np.array([0, 0])

        self.assertTrue(initial_circle.contains_point(expected_contained_point))

    def test_hash(self):
        circle_1 = Circle(radius=1.0)
        circle_2 = Circle(radius=1.0)
        circle_3 = Circle(radius=2.0)
        self.assertEqual(circle_1.__hash__(), circle_2.__hash__())
        self.assertNotEqual(circle_1.__hash__(), circle_3.__hash__())

    def test_equality(self):
        circle_1 = Circle(radius=1.0)
        circle_2 = Circle(radius=1.0)
        circle_3 = Circle(radius=2.0)
        self.assertTrue(circle_1.__eq__(circle_2))
        self.assertFalse(circle_1.__eq__(circle_3))


class TestPolygon(unittest.TestCase):
    def test_rotate_90deg(self):
        translation = np.array([0.0, 0.0])
        rotation_angle = np.pi / 2

        homogeneous_initial_vector = Polygon(
            np.array([[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]])
        )
        expected_transposed_vertices = np.array(
            [[0.5, -0.5], [-0.5, -0.5], [-0.5, 0.5], [0.5, 0.5], [0.5, -0.5]]
        )

        transposed_vector = homogeneous_initial_vector.translate_rotate(translation, rotation_angle)

        for tv, etv in zip(transposed_vector.vertices, expected_transposed_vertices):
            for tv_item, etv_item in zip(tv, etv):
                self.assertAlmostEqual(tv_item, etv_item)

    def test_translate(self):
        translation = np.array([5.52, -2.2])
        rotation_angle = 0

        homogeneous_initial_vector = Polygon(
            np.array([[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]])
        )
        expected_transposed_vertices = np.array(
            [[5.02, -2.7], [5.02, -1.7], [6.02, -1.7], [6.02, -2.7], [5.02, -2.7]]
        )

        transposed_vector = homogeneous_initial_vector.translate_rotate(translation, rotation_angle)

        for tv, etv in zip(transposed_vector.vertices, expected_transposed_vertices):
            for tv_item, etv_item in zip(tv, etv):
                self.assertAlmostEqual(tv_item, etv_item)

    def test_contains_point(self):
        initial_rectangle = Polygon(np.array([[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]]))
        expected_contained_point = np.array([0, 0])

        self.assertTrue(initial_rectangle.contains_point(expected_contained_point))

    def test_hash(self):
        polygon_1 = Polygon(np.array([[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]]))
        polygon_2 = Polygon(np.array([[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]]))
        polygon_3 = Polygon(np.array([[-0.5, -0.5], [0.5, -0.5], [1.0, 0.5], [-0.5, 0.5]]))
        self.assertEqual(polygon_1.__hash__(), polygon_2.__hash__())
        self.assertNotEqual(polygon_1.__hash__(), polygon_3.__hash__())

    def test_equality(self):
        polygon_1 = Polygon(np.array([[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]]))
        polygon_2 = Polygon(np.array([[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]]))
        polygon_3 = Polygon(np.array([[-0.5, -0.5], [0.5, -0.5], [1.0, 0.5], [-0.5, 0.5]]))
        self.assertTrue(polygon_1.__eq__(polygon_2))
        self.assertFalse(polygon_1.__eq__(polygon_3))


class TestShapeGroup(unittest.TestCase):
    def test_rotate_90deg(self):
        translation = np.array([0.0, 0.0])
        rotation_angle = np.pi / 2

        initial_shape_one = Polygon(np.array([[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]]))
        initial_shape_two = Rectangle(1, 1)
        initial_shape_three = Circle(2)
        shape_group = list()
        shape_group.append(initial_shape_one)
        shape_group.append(initial_shape_two)
        shape_group.append(initial_shape_three)
        shape_group = ShapeGroup(shape_group)

        expected_transposed_vertices_one = np.array(
            [[0.5, -0.5], [-0.5, -0.5], [-0.5, 0.5], [0.5, 0.5], [0.5, -0.5]]
        )
        expected_transposed_vertices_two = np.array(
            [[0.5, -0.5], [-0.5, -0.5], [-0.5, 0.5], [0.5, 0.5], [0.5, -0.5]]
        )
        expected_transposed_center_three = np.array([0.0, 0.0])
        expected_transposed = list()
        expected_transposed.append(expected_transposed_vertices_one)
        expected_transposed.append(expected_transposed_vertices_two)
        expected_transposed.append(expected_transposed_center_three)

        shape_group = shape_group.translate_rotate(translation, rotation_angle)

        for sg, et in zip(shape_group.shapes[0:2], expected_transposed[0:2]):
            for sg1, et1 in zip(sg.vertices, et):
                for sg1_item, et1_item in zip(sg1, et1):
                    self.assertAlmostEqual(sg1_item, et1_item)
        for sg, et in zip(shape_group.shapes[2].center, expected_transposed[2]):
            self.assertAlmostEqual(sg, et)

    def test_translate(self):
        translation = np.array([5.52, -2.2])
        rotation_angle = 0

        initial_shape_one = Polygon(np.array([[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]]))
        initial_shape_two = Rectangle(1, 1)
        initial_shape_three = Circle(2)
        shape_group = list()
        shape_group.append(initial_shape_one)
        shape_group.append(initial_shape_two)
        shape_group.append(initial_shape_three)
        shape_group = ShapeGroup(shape_group)

        expected_transposed_vertices_one = np.array(
            [[5.02, -2.7], [5.02, -1.7], [6.02, -1.7], [6.02, -2.7], [5.02, -2.7]]
        )
        expected_transposed_vertices_two = np.array(
            [[5.02, -2.7], [5.02, -1.7], [6.02, -1.7], [6.02, -2.7], [5.02, -2.7]]
        )
        expected_transposed_center_three = np.array([5.52, -2.2])
        expected_transposed = list()
        expected_transposed.append(expected_transposed_vertices_one)
        expected_transposed.append(expected_transposed_vertices_two)
        expected_transposed.append(expected_transposed_center_three)

        shape_group = shape_group.translate_rotate(translation, rotation_angle)

        for sg, et in zip(shape_group.shapes[0:2], expected_transposed[0:2]):
            for sg1, et1 in zip(sg.vertices, et):
                for sg1_item, et1_item in zip(sg1, et1):
                    self.assertAlmostEqual(sg1_item, et1_item)
        for sg, et in zip(shape_group.shapes[2].center, expected_transposed[2]):
            self.assertAlmostEqual(sg, et)

    def test_contains_point(self):
        expected_contained_point = np.array([0, 0])

        initial_shape_one = Polygon(np.array([[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]]))
        initial_shape_two = Rectangle(1, 1)
        initial_shape_three = Circle(2)
        shape_group = list()
        shape_group.append(initial_shape_one)
        shape_group.append(initial_shape_two)
        shape_group.append(initial_shape_three)
        shape_group = ShapeGroup(shape_group)

        self.assertTrue(shape_group.contains_point(expected_contained_point))

    def test_hash(self):
        shape_group_1 = ShapeGroup(
            [
                Polygon(np.array([[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]])),
                Rectangle(1, 1),
            ]
        )
        shape_group_2 = ShapeGroup(
            [
                Polygon(np.array([[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]])),
                Rectangle(1, 1),
            ]
        )
        shape_group_3 = ShapeGroup(
            [
                Polygon(np.array([[-0.5, -0.5], [0.5, -0.5], [1.0, 0.5], [-0.5, 0.5]])),
                Rectangle(1, 1),
                Rectangle(3, 2),
            ]
        )
        self.assertEqual(shape_group_1.__hash__(), shape_group_2.__hash__())
        self.assertNotEqual(shape_group_1.__hash__(), shape_group_3.__hash__())

    def test_equality(self):
        shape_group_1 = ShapeGroup(
            [
                Polygon(np.array([[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]])),
                Rectangle(1, 1),
            ]
        )
        shape_group_2 = ShapeGroup(
            [
                Polygon(np.array([[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]])),
                Rectangle(1, 1),
            ]
        )
        shape_group_3 = ShapeGroup(
            [
                Polygon(np.array([[-0.5, -0.5], [0.5, -0.5], [1.0, 0.5], [-0.5, 0.5]])),
                Rectangle(1, 1),
                Rectangle(3, 2),
            ]
        )
        self.assertTrue(shape_group_1.__eq__(shape_group_2))
        self.assertFalse(shape_group_1.__eq__(shape_group_3))


if __name__ == "__main__":
    unittest.main()
