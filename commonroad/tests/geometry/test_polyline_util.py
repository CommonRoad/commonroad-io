import unittest

import numpy as np

from commonroad.geometry import polyline_util


class TestPolylineUtil(unittest.TestCase):

    def test_compute_polyline_lengths(self):
        polylines = [np.array([[-1, 0], [1, 0], [4, 0]]),
                     np.array([[0, 0], [1, 1], [3, 3], [0, 0]])]
        lengths_exps = [[0., 2., 5.],
                        [0., 1.41421356, 4.24264069, 8.48528137]]

        for p in range(0, len(polylines)):
            polyline = polylines[p]
            lengths_exp = lengths_exps[p]

            lengths = polyline_util.compute_polyline_lengths(polyline)
            self.assertEqual(len(lengths), len(lengths_exp))
            for i in range(0, len(lengths_exp)):
                self.assertAlmostEqual(lengths[i], lengths_exp[i])

    def test_compute_polyline_length(self):
        polylines = [np.array([[1, 1], [3, 1], [7, 1], [0, 1]]),
                     np.array([[1, 1], [-2, -2], [-3, -2], [0, -2]])]
        length_exps = [13., 8.242640687119284]

        for i in range(0, len(polylines)):
            polyline = polylines[i]
            length_exp = length_exps[i]
            length = polyline_util.compute_polyline_length(polyline)
            self.assertAlmostEqual(length, length_exp)

    def test_compute_polyline_curvatures(self):
        polylines = [np.array([[-7, 0], [-3, 0], [1, 0], [5, 0]]),
                     np.array([[-2, 0], [-1, 0.5], [0, 1], [1, 0.5], [2, 0]]),
                     np.array([[-2, 0], [-1, -0.5], [0, -1], [1, -0.5], [2, 0]]),
                     np.array([[-2, 0], [-1, 1], [0, 0], [1, -1], [2, 0]])]
        curvatures_exps = [[0., 0., 0., 0.],
                           [0., -0.17888544, -0.5, -0.17888544, 0.],
                           [0., 0.17888544, 0.5, 0.17888544, 0.],
                           [-0.35355339, -1., 0., 1., 0.35355339]]

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
        orientations_exps = [[0., 90., 180., -90., -90.],
                             [45., -135., -135.],
                             [-168.69006753, -168.69006753]]

        for p in range(0, len(polylines)):
            polyline = polylines[p]
            orientations = polyline_util.compute_polyline_orientations(polyline)
            orientations_exp = orientations_exps[p]

            self.assertEqual(len(orientations), len(orientations_exp))
            for i in range(0, len(orientations_exp)):
                self.assertAlmostEqual(orientations[i], orientations_exp[i])

    def test_compute_polyline_orientation(self):
        polylines = [np.array([[0, 0], [4, -4]]),
                     np.array([[1, 1], [7, 1]]),
                     np.array([[-1, -1], [-1, 1], [3, 2], [7, 4]]),
                     np.array([[0, 0], [9, 0.1]])]
        orientation_exps = [-45., 0., 90., 0.6365935759634864]

        for i in range(0, len(polylines)):
            polyline = polylines[i]
            orientation_exp = orientation_exps[i]
            orientation = polyline_util.compute_polyline_orientation(polyline)

            self.assertAlmostEqual(orientation, orientation_exp)

    def test_compute_polyline_self_intersection(self):
        polylines = [np.array([[0, 0], [1, 0], [2, 0], [7, 0]]),
                     np.array([[0, 0], [1, 0], [1, 1], [0.5, 1], [0.5, -1]]),
                     np.array([[0, 0], [1, 1], [-1, -1]]),
                     np.array([[0, 0], [1, 0], [1, 1], [0, 1], [0, 0]])]
        self_intersection_exps = [False, True, True, False]

        for i in range(0, len(polylines)):
            polyline = polylines[i]
            self_intersection_exp = self_intersection_exps[i]
            self_intersection = polyline_util.compute_polyline_self_intersection(polyline)

            self.assertEqual(self_intersection, self_intersection_exp)

    def test_assert_valid_polyline(self):
        polyline = [[0, 0], [1, 0], [2, 0]]
        with self.assertRaises(AssertionError):
            polyline_util.assert_valid_polyline(polyline)  # noqa

        polylines = [np.array([[0, 0]]), np.array([[[0, 0], [1, 1]], [[2, 2], [3, 3]], [[4, 4], [5, 5]]]),
                     np.array([[0, 0, 0], [1, 0], [2, 0]])]

        for i in range(0, len(polylines)):
            polyline = polylines[i]
            with self.assertRaises(AssertionError):
                polyline_util.assert_valid_polyline(polyline, 2)

        polyline = [[0, 0], [1, 1]]
        with self.assertRaises(AssertionError):
            polyline_util.assert_valid_polyline(polyline, 3)


if __name__ == '__main__':
    unittest.main()
