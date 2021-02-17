import numpy as np
import unittest
from commonroad.geometry.shape import Rectangle, Circle, Polygon, ShapeGroup

__author__ = "Moritz Untersperger"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["Priority Program SPP 1835 Cooperative Interacting Automobiles"]
__version__ = "2020.3"
__maintainer__ = "Moritz Untersperger"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Released"


class TestRectangle(unittest.TestCase):

    def test_rotate_90deg(self):
        translation = np.array([0.0, 0.0])
        rotation_angle = np.pi/2

        homogeneous_initial_vector = Rectangle(1, 1)
        expected_transposed_vertices = np.array([[0.5, -0.5], [-0.5, -0.5], [-0.5, 0.5], [0.5, 0.5], [0.5, -0.5]])

        transposed_vector = homogeneous_initial_vector.translate_rotate(translation, rotation_angle)

        for tv, etv in zip(transposed_vector.vertices, expected_transposed_vertices):
            for tv_item, etv_item in zip(tv, etv):
                self.assertAlmostEqual(tv_item, etv_item)

    def test_translate(self):
        translation = np.array([5.52, -2.2])
        rotation_angle = 0

        homogeneous_initial_vector = Rectangle(1, 1)
        expected_transposed_vertices = np.array([[5.02, -2.7], [5.02, -1.7], [6.02, -1.7], [6.02, -2.7], [5.02, -2.7]])

        transposed_vector = homogeneous_initial_vector.translate_rotate(translation, rotation_angle)

        for tv, etv in zip(transposed_vector.vertices, expected_transposed_vertices):
            for tv_item, etv_item in zip(tv, etv):
                self.assertAlmostEqual(tv_item, etv_item)

    def test_contains_point(self):

        initial_rectangle = Rectangle(1, 1)
        expected_contained_point = np.array([0, 0])

        self.assertTrue(initial_rectangle.contains_point(expected_contained_point))

    def test__compute_vertices(self):

        initial_rectangle = Rectangle(1, 1)
        expected_computed_vertices = np.array([[-0.5, -0.5], [-0.5, 0.5], [0.5, 0.5], [0.5, -0.5], [-0.5, -0.5]])

        computed_vertices = initial_rectangle._compute_vertices()

        for cv, ecv in zip(computed_vertices, expected_computed_vertices):
            for cv_item, ecv_item in zip(cv, ecv):
                self.assertAlmostEqual(cv_item, ecv_item)


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

    def test_contains_point(self):

        initial_circle = Rectangle(0, 0)
        expected_contained_point = np.array([0, 0])

        self.assertTrue(initial_circle.contains_point(expected_contained_point))


class TestPolygon(unittest.TestCase):

    def test_rotate_90deg(self):
        translation = np.array([0.0, 0.0])
        rotation_angle = np.pi / 2

        homogeneous_initial_vector = Polygon(np.array([[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]]))
        expected_transposed_vertices = np.array([[0.5, -0.5], [-0.5, -0.5], [-0.5, 0.5], [0.5, 0.5], [0.5, -0.5]])

        transposed_vector = homogeneous_initial_vector.translate_rotate(translation, rotation_angle)

        for tv, etv in zip(transposed_vector.vertices, expected_transposed_vertices):
            for tv_item, etv_item in zip(tv, etv):
                self.assertAlmostEqual(tv_item, etv_item)

    def test_translate(self):
        translation = np.array([5.52, -2.2])
        rotation_angle = 0

        homogeneous_initial_vector = Polygon(np.array([[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]]))
        expected_transposed_vertices = np.array([[5.02, -2.7], [5.02, -1.7], [6.02, -1.7], [6.02, -2.7], [5.02, -2.7]])

        transposed_vector = homogeneous_initial_vector.translate_rotate(translation, rotation_angle)

        for tv, etv in zip(transposed_vector.vertices, expected_transposed_vertices):
            for tv_item, etv_item in zip(tv, etv):
                self.assertAlmostEqual(tv_item, etv_item)

    def test_contains_point(self):
        initial_rectangle = Polygon(np.array([[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]]))
        expected_contained_point = np.array([0, 0])

        self.assertTrue(initial_rectangle.contains_point(expected_contained_point))


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

        expected_transposed_vertices_one = np.array([[0.5, -0.5], [-0.5, -0.5], [-0.5, 0.5], [0.5, 0.5], [0.5, -0.5]])
        expected_transposed_vertices_two = np.array([[0.5, -0.5], [-0.5, -0.5], [-0.5, 0.5], [0.5, 0.5], [0.5, -0.5]])
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

        expected_transposed_vertices_one = np.array([[5.02, -2.7], [5.02, -1.7], [6.02, -1.7], [6.02, -2.7], [5.02, -2.7]])
        expected_transposed_vertices_two = np.array([[5.02, -2.7], [5.02, -1.7], [6.02, -1.7], [6.02, -2.7], [5.02, -2.7]])
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


if __name__ == '__main__':
    unittest.main()