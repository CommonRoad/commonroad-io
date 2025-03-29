import unittest

import numpy as np

from commonroad.common.validity import (
    is_in_interval,
    is_integer_number,
    is_integer_number_vector,
    is_natural_number,
    is_natural_number_vector,
    is_positive,
    is_real_number,
    is_real_number_vector,
    is_valid_array_of_vertices,
    is_valid_length,
    is_valid_list_of_vertices,
    is_valid_polyline,
    is_valid_velocity,
)


class TestValidity(unittest.TestCase):
    def test_valid_velocities(self):
        """
        This test case tests valid velocity inputs and checks if the result is expected
        :return:
        """

        # all test cases below should return true
        self.assertTrue(is_valid_velocity(0.0))
        self.assertTrue(is_valid_velocity(10))
        self.assertTrue(is_valid_velocity(12.7))
        self.assertTrue(is_valid_velocity(37.0 / 12.0))
        self.assertTrue(is_valid_velocity(129.0))
        self.assertTrue(is_valid_velocity(10000.0))

        self.assertTrue(is_valid_velocity(-0.0))
        self.assertTrue(is_valid_velocity(-10))
        self.assertTrue(is_valid_velocity(-12.7))
        self.assertTrue(is_valid_velocity(-37.0 / 12.0))
        self.assertTrue(is_valid_velocity(-129.0))
        self.assertTrue(is_valid_velocity(-10000.0))

        self.assertTrue(is_valid_velocity(np.float64(12)))
        self.assertTrue(is_valid_velocity(np.float32(12)))
        self.assertTrue(is_valid_velocity(np.float16(12)))

        self.assertTrue(is_valid_velocity(np.int64(12)))
        self.assertTrue(is_valid_velocity(np.int32(12)))
        self.assertTrue(is_valid_velocity(np.int16(12)))
        self.assertTrue(is_valid_velocity(np.int8(12)))

    def test_invalid_velocities(self):
        """
        This test case tests Invalid velocity inputs and checks if the result is expected
        :return:
        """

        # all test cases below should return true
        self.assertFalse(is_valid_velocity("a"))
        self.assertFalse(is_valid_velocity(None))
        self.assertFalse(is_valid_velocity(list()))
        self.assertFalse(is_valid_velocity(list([0, 12.0, "a"])))

    def test_real_number(self):
        """
        This test case tests the function is_real_number
        :return:
        """

        # test cases for true
        self.assertTrue(is_real_number(1))
        self.assertTrue(is_real_number(123.0))
        self.assertTrue(is_real_number(44.78234234))
        self.assertTrue(is_real_number(np.float64(12)))
        self.assertTrue(is_real_number(np.int16(8)))
        self.assertTrue(is_real_number(-123.1))
        self.assertTrue(is_real_number(-1))
        self.assertTrue(is_real_number(-10000.1))

        # test cases for false
        self.assertFalse(is_real_number("a"))
        self.assertFalse(is_real_number(None))
        self.assertFalse(is_real_number(list()))
        self.assertFalse(is_real_number(object()))

    def test_integer_number(self):
        """
        This test case tests the function is_integer_number
        :return:
        """

        # test cases for true
        self.assertTrue(is_integer_number(1))
        self.assertTrue(is_integer_number(np.int16(8)))
        self.assertTrue(is_integer_number(-1))
        self.assertTrue(is_integer_number(-12))
        self.assertTrue(is_integer_number(47))
        self.assertTrue(is_integer_number(123))
        self.assertTrue(is_integer_number(np.int16(12)))
        self.assertTrue(is_integer_number(np.int16(-39)))
        self.assertTrue(is_integer_number(np.int32(445)))
        self.assertTrue(is_integer_number(np.int32(-385)))

        # test cases for false
        self.assertFalse(is_integer_number(1.0))
        self.assertFalse(is_integer_number(12.3))
        self.assertFalse(is_integer_number(-123.1))
        self.assertFalse(is_integer_number(-10000.1))
        self.assertFalse(is_integer_number(44.78234234))
        self.assertFalse(is_integer_number(123.0))
        self.assertFalse(is_integer_number(np.float64(12)))
        self.assertFalse(is_integer_number(np.float64(123)))
        self.assertFalse(is_integer_number("a"))
        self.assertFalse(is_integer_number(None))
        self.assertFalse(is_integer_number(list()))
        self.assertFalse(is_integer_number(object()))

    def test_natural_number(self):
        """
        This test case tests the function is_natural_number
        :return:
        """

        # test cases for true
        self.assertTrue(is_natural_number(0))
        self.assertTrue(is_natural_number(-0))
        self.assertTrue(is_natural_number(1))
        self.assertTrue(is_natural_number(np.int16(8)))
        self.assertTrue(is_natural_number(47))
        self.assertTrue(is_natural_number(123))
        self.assertTrue(is_natural_number(np.int16(12)))
        self.assertTrue(is_natural_number(np.int32(445)))

        # test cases for false
        self.assertFalse(is_natural_number(0.0))
        self.assertFalse(is_natural_number(-0.0))
        self.assertFalse(is_natural_number(-1))
        self.assertFalse(is_natural_number(-12))
        self.assertFalse(is_natural_number(np.int16(-39)))
        self.assertFalse(is_natural_number(np.int32(-385)))
        self.assertFalse(is_natural_number(1.0))
        self.assertFalse(is_natural_number(12.3))
        self.assertFalse(is_natural_number(-123.1))
        self.assertFalse(is_natural_number(-10000.1))
        self.assertFalse(is_natural_number(44.78234234))
        self.assertFalse(is_natural_number(123.0))
        self.assertFalse(is_natural_number(np.float64(12)))
        self.assertFalse(is_natural_number(np.float64(123)))
        self.assertFalse(is_natural_number("a"))
        self.assertFalse(is_natural_number(None))
        self.assertFalse(is_natural_number(list()))
        self.assertFalse(is_natural_number(object()))

    def test_positive(self):
        """
        This test case tests the function is_positive
        :return:
        """

        # test cases for true
        self.assertTrue(is_positive(1))
        self.assertTrue(is_positive(123.0))
        self.assertTrue(is_positive(47))
        self.assertTrue(is_positive(np.int16(12)))
        self.assertTrue(is_positive(np.float64(44467)))

        # test cases for false
        self.assertFalse(is_positive("a"))
        self.assertFalse(is_positive(list()))
        self.assertFalse(is_positive(None))
        self.assertFalse(is_positive(object()))
        self.assertFalse(is_positive(0))
        self.assertFalse(is_positive(-1))
        self.assertFalse(is_positive(-123.4))
        self.assertFalse(is_positive(-np.int16(12)))
        self.assertFalse(is_positive(-np.float64(447423.0)))

    def test_negative(self):
        """
        This test case tests the function is_negative
        :return:
        """

        # test cases for true
        self.assertFalse(is_positive(0))
        self.assertFalse(is_positive(-1))
        self.assertFalse(is_positive(-123.4))
        self.assertFalse(is_positive(-np.int16(12)))
        self.assertFalse(is_positive(-np.float64(447423.0)))

        # test cases for false
        self.assertTrue(is_positive(1))
        self.assertTrue(is_positive(123.0))
        self.assertTrue(is_positive(47))
        self.assertTrue(is_positive(np.int16(12)))
        self.assertTrue(is_positive(np.float64(44467)))
        self.assertFalse(is_positive("a"))
        self.assertFalse(is_positive(None))
        self.assertFalse(is_positive(list()))
        self.assertFalse(is_positive(object()))

    def test_valid_length(self):
        """
        This test case tests the function is_valid_length
        :return:
        """

        # test cases for true
        self.assertTrue(is_valid_length(1))
        self.assertTrue(is_valid_length(12))
        self.assertTrue(is_valid_length(147824))
        self.assertTrue(is_valid_length(np.int32(123)))

        # test cases for false
        self.assertFalse(is_valid_length("a"))
        self.assertFalse(is_valid_length(None))
        self.assertFalse(is_valid_length(list()))
        self.assertFalse(is_valid_length(object()))
        self.assertFalse(is_valid_length(0))
        self.assertFalse(is_valid_length(-12))
        self.assertFalse(is_valid_length(13.4))
        self.assertFalse(is_valid_length(np.float64(123)))
        self.assertFalse(is_valid_length(17.9))

    def test_real_number_vector(self):
        """
        This test case tests the function is_real_number_vector
        :return:
        """

        # test cases for true
        self.assertTrue(is_real_number_vector(np.array([1])))
        self.assertTrue(is_real_number_vector(np.array([1, -2, 3.12, 4, 6])))
        self.assertTrue(is_real_number_vector(np.array([-1, 2, 3, 7.0238420358, 120])))
        self.assertTrue(is_real_number_vector(np.arange(1, 4, 0.25)))
        self.assertTrue(is_real_number_vector(np.zeros(12)))

        # test cases for false
        self.assertFalse(is_real_number_vector("a"))
        self.assertFalse(is_real_number_vector(None))
        self.assertFalse(is_real_number_vector(list()))
        self.assertFalse(is_real_number_vector(object()))
        self.assertFalse(is_real_number_vector([1, "a"]))
        self.assertFalse(is_real_number_vector(["a"]))
        self.assertFalse(is_real_number_vector([1, None]))
        self.assertFalse(is_real_number_vector([None, 123]))

        # test cases for true using the optional parameter LENGTH
        self.assertTrue(is_real_number_vector(np.array([1]), 1))
        self.assertTrue(is_real_number_vector(np.array([1, -2, 3, 4, 6]), 5))
        self.assertTrue(is_real_number_vector(np.array([-1, 2, 3, 7, 120]), 5))
        self.assertTrue(is_real_number_vector(np.arange(1, 4), 3))
        self.assertTrue(is_real_number_vector(np.zeros(12), 12))

        # test cases for false using the optional parameter LENGTH
        self.assertFalse(is_real_number_vector([1], 2))
        self.assertFalse(is_real_number_vector([1, -2, 3, 4, 6], 51))
        self.assertFalse(is_real_number_vector(np.array([-1, 2, 3, 7, 120]), 15))
        self.assertFalse(is_real_number_vector(np.arange(1, 4), 2))
        self.assertFalse(is_real_number_vector(np.zeros(12), 11))

    def test_integer_number_vector(self):
        """
        This test case tests the function is_integer_number_vector
        :return:
        """

        # test cases for true
        self.assertTrue(is_integer_number_vector(np.array([1])))
        self.assertTrue(is_integer_number_vector(np.array([1, -2, 3, 4, 6])))
        self.assertTrue(is_integer_number_vector(np.array([-1, 2, 3, 7, 120])))
        self.assertTrue(is_integer_number_vector(np.arange(1, 4)))
        self.assertTrue(is_integer_number_vector(np.zeros(12, dtype=int)))

        # test cases for false
        self.assertFalse(is_integer_number_vector("a"))
        self.assertFalse(is_integer_number_vector(None))
        self.assertFalse(is_integer_number_vector(list()))
        self.assertFalse(is_integer_number_vector(object()))
        self.assertFalse(is_integer_number_vector([1, "a"]))
        self.assertFalse(is_integer_number_vector(["a"]))
        self.assertFalse(is_integer_number_vector([1, None]))
        self.assertFalse(is_integer_number_vector([None, 123]))
        self.assertFalse(is_integer_number_vector(np.array([-1, 2, 3, 7.0238420358, 120])))
        self.assertFalse(is_integer_number_vector(np.arange(1, 4, 0.25)))
        self.assertFalse(is_integer_number_vector(np.zeros(12)))

        # test cases for true using the optional parameter LENGTH
        self.assertTrue(is_integer_number_vector(np.array([1]), 1))
        self.assertTrue(is_integer_number_vector(np.array([1, -2, 3, 4, 6]), 5))
        self.assertTrue(is_integer_number_vector(np.array([-1, 2, 3, 7, 120]), 5))
        self.assertTrue(is_integer_number_vector(np.arange(1, 4), 3))
        self.assertTrue(is_integer_number_vector(np.zeros(12, dtype=int), 12))

        # test cases for false using the optional parameter LENGTH
        self.assertFalse(is_integer_number_vector([1], 2))
        self.assertFalse(is_integer_number_vector([1, -2, 3, 4, 6], 51))
        self.assertFalse(is_integer_number_vector(np.array([-1, 2, 3, 7, 120], dtype=int), 15))
        self.assertFalse(is_integer_number_vector(np.arange(1, 4, dtype=int), 2))
        self.assertFalse(is_integer_number_vector(np.zeros(12, dtype=int), 11))

    def test_natural_number_vector(self):
        """
        This test case tests the function is_natural_number_vector
        :return:
        """

        # test cases for true
        self.assertTrue(is_natural_number_vector(np.array([1])))
        self.assertTrue(is_natural_number_vector(np.array([0, 1])))
        self.assertTrue(is_natural_number_vector(np.array([-0, 11])))
        self.assertTrue(is_natural_number_vector(np.arange(1, 4)))
        self.assertTrue(is_natural_number_vector(np.zeros(12, dtype=int)))

        # test cases for false
        self.assertFalse(is_natural_number_vector("a"))
        self.assertFalse(is_natural_number_vector(None))
        self.assertFalse(is_natural_number_vector(list()))
        self.assertFalse(is_natural_number_vector(object()))
        self.assertFalse(is_natural_number_vector([1, "a"]))
        self.assertFalse(is_natural_number_vector(["a"]))
        self.assertFalse(is_natural_number_vector([1, None]))
        self.assertFalse(is_natural_number_vector([None, 123]))
        self.assertFalse(is_natural_number_vector(np.array([1, -2, 3, 4, 6])))
        self.assertFalse(is_natural_number_vector(np.array([-1, 2, 3, 7, 120])))
        self.assertFalse(is_natural_number_vector(np.array([-1, 2, 3, 7.0238420358, 120])))
        self.assertFalse(is_natural_number_vector(np.arange(1, 4, 0.25)))
        self.assertFalse(is_natural_number_vector(np.zeros(12)))

        # test cases for true using the optional parameter LENGTH
        self.assertTrue(is_natural_number_vector(np.array([1]), 1))
        self.assertTrue(is_natural_number_vector(np.array([0, 2, 3, 4, 6]), 5))
        self.assertTrue(is_natural_number_vector(np.array([-0, 1, 3, 7, 120]), 5))
        self.assertTrue(is_natural_number_vector(np.arange(1, 4), 3))
        self.assertTrue(is_natural_number_vector(np.zeros(12, dtype=int), 12))

        # test cases for false using the optional parameter LENGTH
        self.assertFalse(is_natural_number_vector(np.array([1, -2, 3, 4, 6]), 5))
        self.assertFalse(is_natural_number_vector(np.array([-1, 2, 3, 7, 120]), 5))
        self.assertFalse(is_natural_number_vector([1], 2))
        self.assertFalse(is_natural_number_vector([1, -2, 3, 4, 6], 51))
        self.assertFalse(is_natural_number_vector(np.array([-1, 2, 3, 7, 120], dtype=int), 15))
        self.assertFalse(is_natural_number_vector(np.arange(1, 4, dtype=int), 2))
        self.assertFalse(is_natural_number_vector(np.zeros(12, dtype=int), 11))

    def test_in_interval(self):
        """
        This test case tests the function is_in_interval
        :return:
        """

        # test cases for true
        self.assertTrue(is_in_interval(1, 0, 2))
        self.assertTrue(is_in_interval(12.123456, -123, 28.86))
        self.assertTrue(is_in_interval(-1, -1, -1))
        self.assertTrue(is_in_interval(0, 0, 0))
        self.assertTrue(is_in_interval(1, 0, None))
        self.assertTrue(is_in_interval(1, None, 2))
        self.assertTrue(is_in_interval(1, None, None))
        self.assertTrue(is_in_interval(np.arange(1, 67), -123, 123))

        # test cases for false
        self.assertFalse(is_in_interval(1, 2, 0))
        self.assertFalse(is_in_interval(1, 1.1, 1.1))
        self.assertFalse(is_in_interval(-1, 0, 1))
        self.assertFalse(is_in_interval(None, 0, 1))
        self.assertFalse(is_in_interval(list(), 0, 1))
        self.assertFalse(is_in_interval(np.arange(1, 67), -123, 50))

    def test_valid_polyline(self):
        """
        This test case tests the function is_valid_polyline
        :return:
        """

        # test cases for true
        self.assertTrue(is_valid_polyline(np.array([[1, 2], [3, 4], [5, 6], [7, 8]])))
        self.assertTrue(is_valid_polyline(np.array([[-1, -2], [-3, -4], [5, 6], [7, 8]])))
        self.assertTrue(is_valid_polyline(np.array([[0, 0], [3, 4], [5, 6], [7, 8]])))
        self.assertTrue(is_valid_polyline(np.array([[100000, 2], [3, 4], [5, 6], [7, 8]])))
        self.assertTrue(
            is_valid_polyline(np.array([[1, 2.2345], [3, 4.234234], [5, 6.23], [7, 8]]))
        )
        self.assertTrue(is_valid_polyline(np.array([[1.12, 2.34], [3.56, 4.78], [5, 6], [-7, 8]])))
        self.assertTrue(is_valid_polyline(np.array([[-1, 2], [-3, 4], [5, -6], [7, -8]])))
        self.assertTrue(is_valid_polyline(np.array([[-1, 2], [3, 4], [10, 5], [23, 12]])))
        self.assertTrue(
            is_valid_polyline(np.array([np.arange(1, 10), np.arange(-1, 3.5, 0.5)]).transpose())
        )
        self.assertTrue(is_valid_polyline(np.array([np.zeros(100), np.ones(100)]).transpose()))
        self.assertTrue(is_valid_polyline(np.array([[-1, 2, 3, 4], np.zeros(4)]).transpose()))
        self.assertTrue(is_valid_polyline(np.array([[-1, 2], [10, 5]]).transpose()))

        #  test cases for true with z-coordinate
        self.assertTrue(is_valid_polyline(np.array([[1, 2, 3], [1, 2, 3], [1, 2, 3]])))
        self.assertTrue(is_valid_polyline(np.array([[-1, 0.2, 3], [11, -22, 3], [1, 2, 3]])))
        self.assertTrue(
            is_valid_polyline(np.array([[-1, 0.2, 3], [11, -22, 3], [1, 2, 3], [0, 0, 0]]))
        )
        self.assertTrue(is_valid_polyline(np.array([[-1, 0.2, 3], [11, -22, 3]])))

        #  test cases for false
        self.assertFalse(is_valid_polyline(None))
        self.assertFalse(is_valid_polyline(list()))
        self.assertFalse(is_valid_polyline("a"))
        self.assertFalse(
            is_valid_polyline(
                [
                    [0, 0, 0],
                    [
                        1,
                        2,
                    ],
                ]
            )
        )
        self.assertFalse(
            is_valid_polyline(
                [
                    [0, 0, 0],
                    [
                        1,
                    ],
                ]
            )
        )
        self.assertFalse(is_valid_polyline([[0], [14]]))
        self.assertFalse(is_valid_polyline([[0, 0]]))
        self.assertFalse(is_valid_polyline([[0, 0], [1, 2, 3], [1, 2, 3, 4]]))
        self.assertFalse(is_valid_polyline([[0, "a"], [1, 2]]))
        self.assertFalse(is_valid_polyline([[0, 0, 0], np.ones(10)]))
        self.assertFalse(is_valid_polyline([[0, 0, 0], [1, 2, None]]))
        self.assertFalse(is_valid_polyline([[0, 0, 0], [1, 2, list()]]))
        self.assertFalse(is_valid_polyline([0, 0, 0]))

        #  test case for false with only 1 point
        self.assertFalse(is_valid_polyline(np.array([[0, 0]])))

        #  test cases for false with z-coordinate
        self.assertFalse(is_valid_polyline(np.array([[1, 2, 3], [1, 2, "a"], [1, 2, 3]])))
        self.assertFalse(is_valid_polyline(np.array([[1, 2, None], [1, 2, 3], [1, 2, 3]])))
        self.assertFalse(is_valid_polyline(np.array([[1, 2, 3, 4], [1, 2, 3, 4], [1, 2, 3, 4]])))
        self.assertFalse(is_valid_polyline(np.array([[-1, 0.2, 3]])))

        #  test case for false with z-coordinate and only 1 point
        self.assertFalse(is_valid_polyline(np.array([[-1, 0.2, 2]])))

        # test cases for true with desired length
        self.assertTrue(is_valid_polyline(np.array([[1, 2], [3, 4], [5, 6], [7, 8]]), length=4))
        self.assertTrue(
            is_valid_polyline(np.array([[-1, -2], [-3, -4], [-5, 5], [6, 7], [8, 9]]), length=5)
        )
        self.assertTrue(is_valid_polyline(np.array([[0, 0], [5, 8]]), length=2))
        self.assertTrue(
            is_valid_polyline(np.array([[-1, 2, 3, 4], [10, 5, 23, 12]]).transpose(), length=4)
        )
        self.assertTrue(
            is_valid_polyline(
                np.array([np.arange(1, 10), np.arange(-1, 3.5, 0.5)]).transpose(), length=9
            )
        )
        self.assertTrue(
            is_valid_polyline(np.array([np.zeros(100), np.ones(100)]).transpose(), length=100)
        )

        #  test cases for true with desired length and z-coordinate
        self.assertTrue(is_valid_polyline(np.array([[1, 2, 3], [1, 2, 3], [1, 2, 3]]), length=3))
        self.assertTrue(
            is_valid_polyline(
                np.array([[-1, 0.2, 3], [11, -22, 3], [1, 2, 3], [-1, -2, 0.5]]), length=4
            )
        )
        self.assertTrue(is_valid_polyline(np.array([[-1, 0.2, 3], [11, -22, 3]]), length=2))

        # test cases for false with desired length
        self.assertFalse(
            is_valid_polyline(np.array([[-1, 2, 3, 4], [10, 5, 23, 12]]).transpose(), length=2)
        )
        self.assertFalse(
            is_valid_polyline(
                np.array([np.arange(1, 10), np.arange(-1, 3.5, 0.5)]).transpose(), length=10
            )
        )
        self.assertFalse(
            is_valid_polyline(
                np.array([np.arange(1, 10), np.arange(-1, 3.5, 0.5)]).transpose(), length=1
            )
        )
        self.assertFalse(
            is_valid_polyline(
                np.array([np.arange(1, 10), np.arange(-1, 3.5, 0.5)]).transpose(), length=1909
            )
        )
        self.assertFalse(
            is_valid_polyline(
                np.array([np.arange(1, 10), np.arange(-1, 3.5, 0.5)]).transpose(), length=1
            )
        )
        self.assertFalse(
            is_valid_polyline(np.array([np.zeros(100), np.ones(100)]).transpose(), length=99)
        )

        #  test cases for false with desired length and z-coordinate
        self.assertFalse(is_valid_polyline(np.array([[1, 2, 3], [1, 2, 3], [1, 2, 3]]), length=22))
        self.assertFalse(
            is_valid_polyline(
                np.array([[-1, 0.2, 3], [11, -22, 3], [1, 2, 3], [-1, -2, 0.5]]), length=3
            )
        )
        self.assertFalse(is_valid_polyline(np.array([[-1, 0.2, 3], [11, -22, 3]]), length=99))
        self.assertFalse(is_valid_polyline(np.array([[-1, 0.2, 3]]), length=1))

    def test_valid_list_of_vertices(self):
        """
        This test case tests the function is_valid_list_of_vertices
        :return:
        """

        #  test cases for true
        self.assertTrue(is_valid_list_of_vertices([[1, 2], [3, 4], [5, 6], [7, 8]]))
        self.assertTrue(is_valid_list_of_vertices([[-1, -2], [-3, -4], [5, 6], [7, 8]]))
        self.assertTrue(is_valid_list_of_vertices([[0, 0], [3, 4], [5, 6], [7, 8]]))
        self.assertTrue(is_valid_list_of_vertices([[100000, 2], [3, 4], [5, 6], [7, 8]]))
        self.assertTrue(is_valid_list_of_vertices([[1, 2.2345], [3, 4.234234], [5, 6.23], [7, 8]]))
        self.assertTrue(is_valid_list_of_vertices([[-1, 2], [-3, 4], [5, -6], [7, -8]]))

        #  test cases for true with only 1 point
        self.assertTrue(is_valid_list_of_vertices([[1, 1]]))

        #  test cases for true with z-coordinate
        self.assertTrue(is_valid_list_of_vertices([[1, 2, 3], [1, 2, 3], [1, 2, 3]]))
        self.assertTrue(is_valid_list_of_vertices([[-1, 0.2, 3], [11, -22, 3], [1, 2, 3]]))
        self.assertTrue(
            is_valid_list_of_vertices([[-1, 0.2, 3], [11, -22, 3], [1, 2, 3], [0, 0, 0]])
        )
        self.assertTrue(is_valid_list_of_vertices([[-1, 0.2, 3], [11, -22, 3]]))

        #  test cases for true with z-coordinate with only 1 point
        self.assertTrue(is_valid_list_of_vertices([[1, 1, 1]]))

        #  test cases for false
        self.assertFalse(is_valid_list_of_vertices(None))
        self.assertFalse(is_valid_list_of_vertices(list()))
        self.assertFalse(is_valid_list_of_vertices("a"))
        self.assertFalse(is_valid_list_of_vertices(np.array([[0, 0], [1, 2]])))
        self.assertFalse(is_valid_list_of_vertices([[0, "a"], [1, 2]]))

        #  test cases for true with desired number of vertices
        self.assertTrue(
            is_valid_list_of_vertices([[1, 2], [3, 4], [5, 6], [7, 8]], number_of_vertices=4)
        )
        self.assertTrue(
            is_valid_list_of_vertices([[-1, -2], [-3, -4], [5, 6], [7, 8]], number_of_vertices=4)
        )
        self.assertTrue(is_valid_list_of_vertices([[0, 0], [3, 4]], number_of_vertices=2))
        self.assertTrue(is_valid_list_of_vertices([[0, 0]], number_of_vertices=1))

        #  test cases for true with desired number of vertices with z-coordinate
        self.assertTrue(is_valid_list_of_vertices([[0, 0, 1], [3, 4, 3]], number_of_vertices=2))
        self.assertTrue(
            is_valid_list_of_vertices(
                [[0, -0, 1], [3, -4, 0.3], [-1, -2, -3]], number_of_vertices=3
            )
        )
        self.assertTrue(is_valid_list_of_vertices([[0, -0, 1]], number_of_vertices=1))

        #  test cases for false with desired number of vertices
        self.assertFalse(
            is_valid_list_of_vertices([[1, 2], [3, 4], [5, 6], [7, 8]], number_of_vertices=2)
        )
        self.assertFalse(
            is_valid_list_of_vertices([[-1, -2], [-3, -4], [5, 6], [7, 8]], number_of_vertices=6)
        )
        self.assertFalse(is_valid_list_of_vertices([[0, 0], [3, 4]], number_of_vertices=1))
        self.assertFalse(is_valid_list_of_vertices([[0, 0, 1], [3, 4, 3]], number_of_vertices=3))

        #  test cases for false with desired number of vertices with z-coordinate
        self.assertFalse(
            is_valid_list_of_vertices([[0, 0, 3], [3, 0.4, 0.4]], number_of_vertices=22)
        )
        self.assertFalse(is_valid_list_of_vertices([[0, 0, 1], [3, 4, 3]], number_of_vertices=101))

    def test_valid_array_of_vertices(self):
        """
        This test case tests the function is_valid_array_of_vertices.
        It is the same function as "is_valid_polyline", with the only difference being in that this function
        will return True when given only 1 point, while the other function will return False, as there have to be
        two points to make a polyline.
        :return:
        """

        # test cases for true with only 1 point
        self.assertTrue(is_valid_array_of_vertices(np.array([[0, 0]])))

        # test case for true with z-coordinate and with only 1 point
        self.assertTrue(is_valid_array_of_vertices(np.array([[0, 0, 0]])))


if __name__ == "__main__":
    unittest.main()
