import math
import unittest
import numpy as np

from commonroad.common.util import Interval, interpolate_angle, subtract_orientations, vectorized_angle_difference, \
    AngleInterval


class TestUtils(unittest.TestCase):
    def test_subtract_orientations(self):
        self.assertAlmostEqual(subtract_orientations(0.1, -0.1), 0.2)
        self.assertAlmostEqual(subtract_orientations(0.0, -0.1), 0.1)
        self.assertAlmostEqual(subtract_orientations(-0.1, 0.1), -0.2)
        self.assertAlmostEqual(subtract_orientations(0.0, 2 * math.pi), 0.0)
        self.assertAlmostEqual(subtract_orientations(1.9 * math.pi, 0.1 * math.pi), -0.2 * math.pi)

    def test_vectorized_angle_difference(self):
        pi = np.pi

        self.assertEqual(0.0, vectorized_angle_difference(0, 0))
        self.assertEqual(0.0, vectorized_angle_difference(2 * pi, 0))
        self.assertEqual(0.0, vectorized_angle_difference(4 * pi, 0))
        self.assertEqual(0.0, vectorized_angle_difference(-2 * pi, 0))
        self.assertEqual(0.0, vectorized_angle_difference(-4 * pi, 0))

        self.assertEqual(0.0, vectorized_angle_difference(pi / 2, pi / 2))
        self.assertEqual(0.0, vectorized_angle_difference(5 / 2 * pi, pi / 2))
        self.assertEqual(0.0, vectorized_angle_difference(9 / 2 * pi, pi / 2))
        self.assertEqual(0.0, vectorized_angle_difference(-3 * pi / 2, pi / 2))
        self.assertEqual(0.0, vectorized_angle_difference(-7 * pi / 2, pi / 2))

        self.assertEqual(0.0, vectorized_angle_difference(pi, pi))
        self.assertEqual(0.0, vectorized_angle_difference(3 * pi, pi))
        self.assertEqual(0.0, vectorized_angle_difference(5 * pi, pi))
        self.assertEqual(0.0, vectorized_angle_difference(-1 * pi, pi))
        self.assertEqual(0.0, vectorized_angle_difference(-3 * pi, pi))

        self.assertEqual(0.0, vectorized_angle_difference(3 * pi / 2, 3 * pi / 2))
        self.assertEqual(0.0, vectorized_angle_difference(7 * pi / 2, 3 * pi / 2))
        self.assertAlmostEqual(0.0, vectorized_angle_difference(11 * pi / 2, 3 * pi / 2))
        self.assertEqual(0.0, vectorized_angle_difference(-1 * pi / 2, 3 * pi / 2))
        self.assertEqual(0.0, vectorized_angle_difference(-5 * pi / 2, 3 * pi / 2))

        self.assertEqual(pi / 2, vectorized_angle_difference(pi / 2, 0))
        self.assertAlmostEqual(-pi / 2, vectorized_angle_difference(-pi / 2, 0))
        self.assertEqual(pi / 2, vectorized_angle_difference(5 * pi / 2, 0))
        self.assertAlmostEqual(-pi / 2, vectorized_angle_difference(-5 * pi / 2, 0))

        self.assertEqual(pi, vectorized_angle_difference(pi / 2, -pi / 2))


class TestInterval(unittest.TestCase):
    def test_intersection(self):
        # empty intersection right
        a = Interval(-0.5, -0.1)
        b = Interval(0, 0.9)
        self.assertEqual(None, a.intersection(b))

        # empty intersection left
        a = Interval(1, 2)
        b = Interval(0, 0.9)
        self.assertEqual(None, a.intersection(b))

        # single value intersection
        a = Interval(0.8, 2)
        b = Interval(2, 3)
        result = a.intersection(b)
        self.assertEqual(2, result.start)
        self.assertEqual(2, result.end)

        # intersection
        a = Interval(0.8, 2)
        b = Interval(0, 0.9)
        result = a.intersection(b)
        self.assertEqual(0.8, result.start)
        self.assertEqual(0.9, result.end)

    def test_contains(self):
        a = Interval(-0.5, 0.5)
        # float
        self.assertEqual(a.contains(0.1), True)
        self.assertEqual(a.contains(1.1), False)
        self.assertEqual(a.contains(-1.1), False)

        # intervals
        self.assertEqual(a.contains(Interval(-0.1, 0.1)), True)
        self.assertEqual(a.contains(Interval(-1.1, 0.1)), False)
        self.assertEqual(a.contains(Interval(-1.1, 1.1)), False)
        self.assertEqual(a.contains(Interval(0.6, 1.1)), False)

    def test_sum(self):
        a = Interval(-0.5, -0.1)
        result = a + 5.0
        self.assertEqual(4.5, result.start)
        self.assertEqual(4.9, result.end)

    def test_substraction(self):
        a = Interval(-0.5, -0.1)
        result = a - 5.0
        self.assertEqual(-5.5, result.start)
        self.assertEqual(-0.1 - 5.0, result.end)

    def test_product(self):
        a = Interval(-0.5, -0.1)
        result = a * 2.0
        self.assertEqual(-1.0, result.start)
        self.assertEqual(-0.2, result.end)

    def test_product_negative(self):
        a = Interval(-0.5, -0.1)
        result = a * (-2.0)

        self.assertEqual(0.2, result.start)
        self.assertEqual(1.0, result.end)

    def test_division(self):
        a = Interval(-0.5, -0.1)
        result = a / 2.0
        self.assertEqual(-0.25, result.start)
        self.assertEqual(-0.05, result.end)

    def test_division_negative(self):
        a = Interval(-0.5, -0.1)
        result = a / (-2.0)
        self.assertEqual(0.05, result.start)
        self.assertEqual(0.25, result.end)

    def test_round(self):
        a = Interval(-1.1, -0.1)
        result = round(a)
        self.assertEqual(-1, result.start)
        self.assertEqual(0, result.end)

    def test_lt(self):
        a = Interval(-1.1, -0.1)
        b = Interval(0.1, 0.2)
        self.assertEqual(a < b, True)
        self.assertEqual(b < a, False)

    def test_gt(self):
        a = Interval(-1.1, -0.1)
        b = Interval(0.1, 0.2)
        self.assertEqual(a > b, False)
        self.assertEqual(b > a, True)

    def test__contains__(self):
        self.assertTrue(1.0 in Interval(0, 2))
        self.assertTrue(1.0 in Interval(1, 2))
        self.assertTrue(2.0 in Interval(0, 2))
        self.assertFalse(-1.0 in Interval(0, 2))
        self.assertFalse(3.0 in Interval(0, 2))


class TestAngleInterval(unittest.TestCase):

    def test__contains__(self):
        interval = AngleInterval(-np.pi / 2, np.pi / 2)
        self.assertTrue(interval.__contains__(0.0))
        self.assertTrue(interval.__contains__(np.pi / 2))
        self.assertTrue(interval.__contains__(-np.pi / 2))
        self.assertTrue(interval.__contains__(np.pi / 4))
        self.assertTrue(interval.__contains__(-np.pi / 4))
        self.assertTrue(interval.__contains__(5 * np.pi / 2))
        self.assertTrue(interval.__contains__(-5 * np.pi / 2))
        self.assertTrue(interval.__contains__(9 * np.pi / 2))
        self.assertTrue(interval.__contains__(-9 * np.pi / 2))
        self.assertTrue(interval.__contains__(2 * np.pi))
        self.assertTrue(interval.__contains__(-2 * np.pi))
        self.assertFalse(interval.__contains__(np.pi))
        self.assertFalse(interval.__contains__(-np.pi))
        self.assertFalse(interval.__contains__(3 * np.pi))
        self.assertFalse(interval.__contains__(-3 * np.pi))
        self.assertFalse(interval.__contains__(5 * np.pi))
        self.assertFalse(interval.__contains__(-5 * np.pi))

        # Regression test
        interval = AngleInterval(-0.1965, 0.2034)
        self.assertTrue(interval.__contains__(0.0034))

    def test_contains(self):
        interval = AngleInterval(-np.pi / 2, np.pi / 2)
        self.assertTrue(interval.contains(0.0))
        self.assertTrue(interval.contains(np.pi / 2))
        self.assertTrue(interval.contains(-np.pi / 2))
        self.assertTrue(interval.contains(np.pi / 4))
        self.assertTrue(interval.contains(-np.pi / 4))
        self.assertTrue(interval.contains(5 * np.pi / 2))
        self.assertTrue(interval.contains(-5 * np.pi / 2))
        self.assertTrue(interval.contains(9 * np.pi / 2))
        self.assertTrue(interval.contains(-9 * np.pi / 2))
        self.assertTrue(interval.contains(2 * np.pi))
        self.assertTrue(interval.contains(-2 * np.pi))
        self.assertFalse(interval.contains(np.pi))
        self.assertFalse(interval.contains(-np.pi))
        self.assertFalse(interval.contains(3 * np.pi))
        self.assertFalse(interval.contains(-3 * np.pi))
        self.assertFalse(interval.contains(5 * np.pi))
        self.assertFalse(interval.contains(-5 * np.pi))

        # Regression test
        interval = AngleInterval(-0.1965, 0.2034)
        self.assertTrue(interval.contains(0.0034))

    def test_contains_interval(self):
        interval = AngleInterval(-np.pi / 2, np.pi / 2)

        other_interval = AngleInterval(-np.pi / 2, np.pi / 2)
        self.assertTrue(interval.contains(other_interval))

        other_interval = AngleInterval(-2 * np.pi / 3, 2 * np.pi / 3)
        self.assertFalse(interval.contains(other_interval))

        other_interval = AngleInterval(3 * np.pi / 4, 5 * np.pi / 4)
        self.assertFalse(interval.contains(other_interval))

        other_interval = AngleInterval(-np.pi / 2, -np.pi / 4)
        self.assertTrue(interval.contains(other_interval))


if __name__ == '__main__':
    unittest.main()
