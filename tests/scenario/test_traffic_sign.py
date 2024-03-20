import unittest

import numpy as np

from commonroad.scenario.traffic_sign import (
    TrafficSign,
    TrafficSignElement,
    TrafficSignID,
    TrafficSignIDCountries,
    TrafficSignIDGermany,
    TrafficSignIDZamunda,
)


class TestTrafficSign(unittest.TestCase):
    def test_translate_rotate(self):
        traffic_sign_max_speed = TrafficSignElement(TrafficSignIDZamunda.MAX_SPEED.value, ["15"])
        traffic_sign = TrafficSign(1, [traffic_sign_max_speed], {5}, np.array([1.0, 1.0]))

        traffic_sign.translate_rotate(np.array([2, -4]), np.pi / 2)

        desired_traffic_light_position = np.array([3, 3])

        np.testing.assert_array_almost_equal(traffic_sign.position, desired_traffic_light_position)

    def test_convert_to_2d(self):
        traffic_sign_max_speed = TrafficSignElement(TrafficSignIDZamunda.MAX_SPEED.value, ["15"])
        traffic_sign = TrafficSign(1, [traffic_sign_max_speed], {3}, np.array([10.0, 7.0, 3.0]))

        traffic_sign.convert_to_2d()

        self.assertEqual(traffic_sign.position.shape, (2,))
        np.testing.assert_array_almost_equal(traffic_sign.position, np.array([10.0, 7.0]))

    def test_equality(self):
        traffic_sign_max_speed = TrafficSignElement(TrafficSignIDGermany.MAX_SPEED, ["15"])
        traffic_sign_one = TrafficSign(1, [traffic_sign_max_speed], {3}, np.array([10.0, 7.0]))
        traffic_sign_two = TrafficSign(1, [traffic_sign_max_speed], {3}, np.array([10.0, 7.0]))
        self.assertTrue(traffic_sign_one == traffic_sign_two)

        traffic_sign_two = TrafficSign(2, [traffic_sign_max_speed], {3}, np.array([10.0, 7.0]))
        self.assertFalse(traffic_sign_one == traffic_sign_two)

        traffic_sign_town_sign = TrafficSignElement(TrafficSignIDGermany.TOWN_SIGN, ["15"])
        traffic_sign_two = TrafficSign(2, [traffic_sign_town_sign], {3}, np.array([10.0, 7.0]))
        self.assertFalse(traffic_sign_one == traffic_sign_two)

        traffic_sign_two = TrafficSign(1, [traffic_sign_max_speed], {4}, np.array([10.0, 7.0]))
        self.assertFalse(traffic_sign_one == traffic_sign_two)

        traffic_sign_two = TrafficSign(1, [traffic_sign_max_speed], {3}, np.array([10.0, 6.99999999999]))
        self.assertTrue(traffic_sign_one == traffic_sign_two)

        traffic_sign_two = TrafficSign(1, [traffic_sign_max_speed], {3}, np.array([10.0, 7.1]))
        self.assertFalse(traffic_sign_one == traffic_sign_two)

        traffic_sign_two = TrafficSign(1, [traffic_sign_max_speed], {3}, np.array([10.0, 7.0]), True)
        self.assertFalse(traffic_sign_one == traffic_sign_two)

        traffic_sign_two = TrafficSign(1, [traffic_sign_max_speed], {3}, np.array([1.0, 1.0]), True)
        self.assertFalse(traffic_sign_one == traffic_sign_two)
        self.assertFalse(traffic_sign_two == traffic_sign_one)

        traffic_sign_one = TrafficSign(1, [traffic_sign_max_speed], {3}, np.array([1.0, 1.0]))
        traffic_sign_two = TrafficSign(1, [traffic_sign_max_speed], {3}, np.array([1.0, 1.0]))
        self.assertTrue(traffic_sign_one == traffic_sign_two)

    def test_hash(self):
        traffic_sign_max_speed = TrafficSignElement(TrafficSignIDGermany.TOWN_SIGN, ["15"])
        traffic_sign_one = TrafficSign(1, [traffic_sign_max_speed], {3}, np.array([10.0, 7.0]))
        traffic_sign_two = TrafficSign(1, [traffic_sign_max_speed], {3}, np.array([10.0, 7.0]))
        self.assertEqual(hash(traffic_sign_one), hash(traffic_sign_two))

        traffic_sign_two = TrafficSign(1, [traffic_sign_max_speed], {3}, np.array([10.0000000001, 7.0]))
        self.assertNotEqual(hash(traffic_sign_one), hash(traffic_sign_two))

        traffic_sign_two = TrafficSign(1, [traffic_sign_max_speed], {3}, np.array([1.0, 1.0]))
        self.assertNotEqual(hash(traffic_sign_one), hash(traffic_sign_two))

        traffic_sign_one = TrafficSign(1, [traffic_sign_max_speed], {3}, np.array([1.0, 1.0]))
        traffic_sign_two = TrafficSign(1, [traffic_sign_max_speed], {3}, np.array([1.0, 1.0]))
        self.assertEqual(hash(traffic_sign_one), hash(traffic_sign_two))

        traffic_sign_one = TrafficSign(1, [traffic_sign_max_speed], set(), np.array([1.0, 1.0]))
        traffic_sign_two = TrafficSign(1, [traffic_sign_max_speed], None, np.array([1.0, 1.0]))
        self.assertEqual(hash(traffic_sign_one), hash(traffic_sign_two))


class TestTrafficSignElement(unittest.TestCase):
    def test_equality(self):
        traffic_sign_element_1 = TrafficSignElement(TrafficSignIDGermany.MAX_SPEED, ["15", "16"])
        traffic_sign_element_2 = TrafficSignElement(TrafficSignIDGermany.MAX_SPEED, ["15", "16"])
        self.assertTrue(traffic_sign_element_1 == traffic_sign_element_2)

        traffic_sign_element_2 = TrafficSignElement(TrafficSignIDGermany.TOWN_SIGN, ["15", "16"])
        self.assertFalse(traffic_sign_element_1 == traffic_sign_element_2)

        traffic_sign_element_2 = TrafficSignElement(TrafficSignIDGermany.MAX_SPEED, ["15", "17"])
        self.assertFalse(traffic_sign_element_1 == traffic_sign_element_2)

    def test_hash(self):
        traffic_sign_element_1 = TrafficSignElement(TrafficSignIDGermany.MAX_SPEED, ["15", "16"])
        traffic_sign_element_2 = TrafficSignElement(TrafficSignIDGermany.MAX_SPEED, ["15", "16"])
        self.assertEqual(hash(traffic_sign_element_1), hash(traffic_sign_element_2))

        traffic_sign_element_2 = TrafficSignElement(TrafficSignIDGermany.DIRECTIONS_SIGN, ["15", "16"])
        self.assertNotEqual(hash(traffic_sign_element_1), hash(traffic_sign_element_2))

    def test_traffic_sign_ids(self):
        # ensure that size of TrafficSignID equals number of all available traffic sign ID names
        signs = set()
        for country in list(TrafficSignIDCountries.values()):
            signs = signs.union(set(country._member_names_))
        self.assertEqual(len(TrafficSignID), len(signs))


if __name__ == "__main__":
    unittest.main()
