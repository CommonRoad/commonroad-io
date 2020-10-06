import unittest

from commonroad.common.util import Interval


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


if __name__ == '__main__':
    unittest.main()
