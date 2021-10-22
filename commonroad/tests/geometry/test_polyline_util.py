import unittest

import numpy as np

from commonroad.geometry import polyline_util
from commonroad.geometry.polyline_util import compare_polylines_equality, is_point_on_polyline, \
    compute_polyline_intersections, resample_polyline_with_number, resample_polyline_with_distance


class TestPolylineUtil(unittest.TestCase):

    def test_compute_polyline_lengths(self):
        polylines = [np.array([[-1, 0], [1, 0], [4, 0]]),
                     np.array([[0, 0], [1, 1], [3, 3], [0, 0]]),
                     np.array([[-7, -7], [7, 7]])]
        lengths_exps = [[0., 2., 5.],
                        [0., 1.41421356, 4.24264069, 8.48528137],
                        [0., 19.79898987322333]]

        for p in range(0, len(polylines)):
            polyline = polylines[p]
            lengths_exp = lengths_exps[p]

            lengths = polyline_util.compute_polyline_lengths(polyline)
            self.assertEqual(len(lengths), len(lengths_exp))
            for i in range(0, len(lengths_exp)):
                self.assertAlmostEqual(lengths[i], lengths_exp[i])

    def test_compute_polyline_complete_length(self):
        polylines = [np.array([[1, 1], [3, 1], [7, 1], [0, 1]]),
                     np.array([[1, 1], [-2, -2], [-3, -2], [0, -2]]),
                     np.array([[-7, -7], [7, 7]])]
        length_exps = [13., 8.242640687119284, 19.79898987322333]

        for i in range(0, len(polylines)):
            polyline = polylines[i]
            length_exp = length_exps[i]
            length = polyline_util.compute_total_polyline_length(polyline)
            self.assertAlmostEqual(length, length_exp)

    def test_compute_polyline_curvatures(self):
        polylines = [np.array([[-7, 0], [-3, 0], [1, 0], [5, 0]]),
                     np.array([[-2, 0], [-1, 0.5], [0, 1], [1, 0.5], [2, 0]]),
                     np.array([[-2, 0], [-1, -0.5], [0, -1], [1, -0.5], [2, 0]]),
                     np.array([[-2, 0], [-1, 1], [0, 0], [1, -1], [2, 0]]),
                     np.array([[0, 0], [1, 1], [2, 2]]),
                     np.array([[1, 1], [2, 3], [3, 6]]),
                     np.array([[4, 6], [5, 5], [6, 3]])]
        curvatures_exps = [[0., 0., 0., 0.],
                           [0., -0.17888544, -0.5, -0.17888544, 0.],
                           [0., 0.17888544, 0.5, 0.17888544, 0.],
                           [-0.35355339, -1., 0., 1., 0.35355339],
                           [0., 0., 0.],
                           [0.044721359549995794, 0.025613150093386463, 0.015811388300841896],
                           [-0.17677669529663687, -0.08533849172695833, -0.044721359549995794]]

        for p in range(0, len(polylines)):
            polyline = polylines[p]
            curvatures = polyline_util.compute_polyline_curvatures(polyline)
            curvatures_exp = curvatures_exps[p]

            self.assertEqual(len(curvatures), len(curvatures_exp))
            for i in range(0, len(curvatures_exp)):
                self.assertAlmostEqual(curvatures[i], curvatures_exp[i])

    def test_compute_polyline_orientations(self):
        polylines = [np.array([[0, 0], [4, 0], [4, 4], [0, 4], [0, 0]]), np.array([[1, 1], [2, 2], [-7, -7]]),
                     np.array([[5, 5], [0, 4]])]
        orientations_exps = [[0., 1.5707963267948966, 3.141592653589793, -1.5707963267948966, -1.5707963267948966],
                             [0.7853981633974483, -2.356194490192345, -2.356194490192345],
                             [-2.9441970937399127, -2.9441970937399127]]

        for p in range(0, len(polylines)):
            polyline = polylines[p]
            orientations = polyline_util.compute_polyline_orientations(polyline)
            orientations_exp = orientations_exps[p]

            self.assertEqual(len(orientations), len(orientations_exp))
            for i in range(0, len(orientations_exp)):
                self.assertAlmostEqual(orientations[i], orientations_exp[i])

    def test_compute_polyline_initial_orientation(self):
        polylines = [np.array([[0, 0], [4, -4]]),
                     np.array([[1, 1], [7, 1]]),
                     np.array([[-1, -1], [-1, 1], [3, 2], [7, 4]]),
                     np.array([[0, 0], [9, 0.1]])]
        orientation_exps = [-0.7853981633974483, 0., 1.5707963267948966, 0.011110653897607473]

        for i in range(0, len(polylines)):
            polyline = polylines[i]
            orientation_exp = orientation_exps[i]
            orientation = polyline_util.compute_polyline_initial_orientation(polyline)

            self.assertAlmostEqual(orientation, orientation_exp)

    def test_is_point_on_polyline(self):
        polyline = np.array([[0, 0], [1, 0], [2, 0], [3, 0], [4, 0]])
        self.assertTrue(is_point_on_polyline(polyline, np.array([2.5, 0])))
        self.assertTrue(is_point_on_polyline(polyline, np.array([0, 0])))
        self.assertTrue(is_point_on_polyline(polyline, np.array([4, 0])))

        polyline = np.array([[-1, -1], [0, 0], [1, 1], [2, 1], [3, 1]])
        self.assertTrue(is_point_on_polyline(polyline, np.array([-0.5, -0.5])))
        self.assertFalse(is_point_on_polyline(polyline, np.array([-1.1, -1.1])))
        self.assertTrue(is_point_on_polyline(polyline, np.array([1.5, 1])))
        self.assertFalse(is_point_on_polyline(polyline, np.array([3.05, 1])))
        self.assertFalse(is_point_on_polyline(polyline, np.array([0.95, 1])))

        polyline = np.array([[-10, -10], [-10, 10]])
        self.assertTrue(is_point_on_polyline(polyline, np.array([-10, 0])))
        self.assertFalse(is_point_on_polyline(polyline, np.array([-9.99, 0])))
        self.assertTrue(is_point_on_polyline(polyline, np.array([-10, -10])))
        self.assertFalse(is_point_on_polyline(polyline, np.array([-10.1, -10])))

    def test_compute_polyline_intersections(self):
        polyline_1 = np.array([[0, 0], [1, 0], [2, 0], [3, 0]])
        polyline_2 = np.array([[0, -1], [1, -1], [2, -1], [3, -1]])
        intersections = compute_polyline_intersections(polyline_1, polyline_2)
        self.assertEqual(intersections.size, 0)

        polyline_2 = np.array([[1.5, 0.5], [1.5, 1], [1.5, 1.5], [1.5, 2]])
        intersections = compute_polyline_intersections(polyline_1, polyline_2)
        print(intersections)
        self.assertEqual(intersections.size, 0)

        polylines_1 = [np.array([[0, 0], [1, 0], [2, 0], [3, 0]]),
                       np.array([[-1, -1], [0, 0], [1, 1]]),
                       np.array([[2, -5], [2, -3], [2, 2]]),
                       np.array([[-2, -2], [2, 2]])]
        polylines_2 = [np.array([[2.5, -1.5], [2.5, 2], [2.5, 3]]),
                       np.array([[1, 0], [1, 0.5], [1, 1]]),
                       np.array([[0, 0], [3, 0], [3, -1], [0, -1], [0, -2], [7, -2]]),
                       np.array([[-2, 2], [2, -2]])]
        intersections_exps = [[[2.5, 0.]], [[1, 1]], [[2, 0], [2, -1], [2, -2]], [[0, 0]]]
        for i in range(0, len(polylines_1)):
            polyline_1 = polylines_1[i]
            polyline_2 = polylines_2[i]
            intersections_exp = intersections_exps[i]
            intersections = compute_polyline_intersections(polyline_1, polyline_2)
            for j in range(0, len(intersections_exp)):
                x_exp, y_exp = intersections_exp[j]
                x, y = intersections[j]
                self.assertAlmostEqual(x, x_exp)
                self.assertAlmostEqual(y, y_exp)

    def test_is_polyline_self_intersection(self):
        polylines = [np.array([[0, 0], [1, 0], [2, 0], [7, 0]]),
                     np.array([[0, 0], [1, 0], [1, 1], [0.5, 1], [0.5, -1]]),
                     np.array([[0, 0], [1, 1], [-1, -1]]),
                     np.array([[0, 0], [1, 0], [1, 1], [0, 1], [0, 0]])]
        self_intersection_exps = [False, True, True, False]

        for i in range(0, len(polylines)):
            polyline = polylines[i]
            self_intersection_exp = self_intersection_exps[i]
            self_intersection = polyline_util.is_polyline_self_intersection(polyline)

            self.assertEqual(self_intersection, self_intersection_exp)

    def test_compare_polylines_equality(self):
        polyline_1 = np.array([[0, 0], [1, 0], [2, 0], [3, 0]])
        polyline_2 = np.array([[0, 0], [1, 0], [2, 0], [3, 0]])
        self.assertTrue(compare_polylines_equality(polyline_1, polyline_2))

        polyline_2 = np.array([[0, 0], [1, 0], [2, 0.0000000000001], [3, 0]])
        self.assertTrue(compare_polylines_equality(polyline_1, polyline_2))

        polyline_2 = np.array([[0, 0], [1, 0], [2, 0.0000001], [3, 0]])
        self.assertFalse(compare_polylines_equality(polyline_1, polyline_2))
        self.assertTrue(compare_polylines_equality(polyline_1, polyline_2, threshold=0.00001))

        polyline_1 = np.array([[0, 0.0000000001], [1.0000000000001, 0], [2, 0], [3.0000000000009, 0]])
        polyline_2 = np.array([[0, 0.00000000001], [0.99999999999, 0], [2, 0], [2.999999999999, 0]])
        self.assertTrue(compare_polylines_equality(polyline_1, polyline_2))

    def test_resample_polyline_with_number_and_distance(self):
        polylines = [np.array([[0, 0], [1, 0], [2, 0], [3, 0]]),
                     np.array([[0., 0.], [0.5, 0.], [1., 0.], [1.5, 0.], [2., 0.], [2.5, 0.], [3, 0.]]),
                     np.array([[0, -1], [0, 0], [0, 1]]),
                     np.array([[-3, -3], [-1, -1], [1, 1], [3, 3]]),
                     np.array([[0, 0], [0, 1], [1, 1], [1, 2], [2, 2]]),
                     np.array([[-1, 0], [0, 0]])]
        resampled_polyline_exps = [[[0., 0.], [0.5, 0.], [1., 0.], [1.5, 0.], [2., 0.], [2.5, 0.], [3, 0.]],
                                   [[0., 0.], [1, 0.], [2, 0.], [3, 0.]],
                                   [[0., -1], [0., 1]],
                                   [[-3, -3], [-1.8, -1.8], [-0.6, -0.6], [0.6, 0.6], [1.8, 1.8], [3, 3]],
                                   [[0., 0.], [0., 0.5], [0., 1], [0.5, 1.], [1., 1.], [1., 1.5], [1., 2.], [1.5, 2.],
                                    [2., 2.]],
                                   [[-1., 0.], [-0.66666667, 0.], [-0.33333333, 0.], [0., 0.]]]
        numbers = [7, 4, 2, 6, 9, 4]
        distances = [0.5, 1., 2., 1.697056274847714, 0.5, 1 / 3]

        for i in range(0, len(polylines)):
            polyline = polylines[i]
            resample_polyline_exp = resampled_polyline_exps[i]
            number = numbers[i]
            distance = distances[i]
            resample_polyline_number = resample_polyline_with_number(polyline, number)
            resample_polyline_distance = resample_polyline_with_distance(polyline, distance)
            for resample_polyline in [resample_polyline_number, resample_polyline_distance]:
                for j in range(0, len(resample_polyline_exp)):
                    x_exp, y_exp = resample_polyline_exp[j]
                    x, y = resample_polyline[j]
                    self.assertAlmostEqual(x, x_exp)
                    self.assertAlmostEqual(y, y_exp)

        polyline = np.array([[0, 0], [1, 0]])
        with self.assertRaises(AssertionError):
            resample_polyline_with_number(polyline, 1)

        with self.assertRaises(AssertionError):
            resample_polyline_with_distance(polyline, 0)

    def test_assert_valid_polyline(self):
        polyline = [[0, 0], [1, 0], [2, 0]]
        with self.assertRaises(AssertionError):
            polyline_util.assert_valid_polyline(polyline)

        polylines = [np.array([[0, 0]]), np.array([[[0, 0], [1, 1]], [[2, 2], [3, 3]], [[4, 4], [5, 5]]]),
                     np.array([[0, 0, 0], [1, 0], [2, 0]])]

        for i in range(0, len(polylines)):
            polyline = polylines[i]
            with self.assertRaises(AssertionError):
                polyline_util.assert_valid_polyline(polyline)

        polyline = np.array([[0, 0], [1, 1]])
        with self.assertRaises(AssertionError):
            polyline_util.assert_valid_polyline(polyline, 3)


if __name__ == '__main__':
    unittest.main()
