import unittest
import numpy as np

from commonroad.common.util import Interval, interpolate_angle


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
        self.assertEqual(a.contains(Interval(-0.1,0.1)), True)
        self.assertEqual(a.contains(Interval(-1.1,0.1)), False)
        self.assertEqual(a.contains(Interval(-1.1,1.1)), False)
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
        self.assertEqual(-0.1-5.0, result.end)

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

    def test_interpolate_angle(self):
        a = interpolate_angle(x=0.5, xp=np.array([0., 1.]), fp=np.array([0., 2 * np.pi]), degrees=False)
        b = interpolate_angle(x=0.5, xp=np.array([0., 1.]), fp=np.array([0., 360.]), degrees=True)
        c = interpolate_angle(x=0.5, xp=np.array([0., 1.]), fp=np.array([-np.pi, np.pi]), degrees=False)
        d = interpolate_angle(x=0.5, xp=np.array([0., 1.]), fp=np.array([-180., 180.]), degrees=True)
        self.assertAlmostEqual(a, 0.)
        self.assertAlmostEqual(b, 0.)
        self.assertAlmostEqual(c, np.pi)
        self.assertAlmostEqual(d, 180.)


if __name__ == '__main__':
    unittest.main()
