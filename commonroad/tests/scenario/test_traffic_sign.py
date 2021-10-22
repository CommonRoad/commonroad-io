import unittest
import numpy as np
from commonroad.scenario.traffic_sign import TrafficLight, TrafficLightCycleElement, TrafficLightState, TrafficSign, \
    TrafficSignElement, TrafficSignIDZamunda, TrafficSignIDGermany


class TestTrafficSign(unittest.TestCase):
    def test_translate_rotate(self):
        traffic_sign_max_speed = TrafficSignElement(TrafficSignIDZamunda.MAX_SPEED.value, ["15"])
        traffic_sign = TrafficSign(1, [traffic_sign_max_speed], {5}, np.array([1.0, 1.0]))

        traffic_sign.translate_rotate(np.array([2, -4]), np.pi/2)

        desired_traffic_light_position = np.array([3, 3])

        np.testing.assert_array_almost_equal(traffic_sign.position, desired_traffic_light_position)

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

        traffic_sign_two = TrafficSign(1, [traffic_sign_max_speed], {3}, np.array([10.0, 6.9999999999]))
        self.assertTrue(traffic_sign_one == traffic_sign_two)

        traffic_sign_two = TrafficSign(1, [traffic_sign_max_speed], {3}, np.array([10.0, 7.1]))
        self.assertFalse(traffic_sign_one == traffic_sign_two)


class TestTrafficLight(unittest.TestCase):
    def test_get_state_at_time_step(self):
        cycle = [TrafficLightCycleElement(TrafficLightState.GREEN, 2),
                 TrafficLightCycleElement(TrafficLightState.YELLOW, 3),
                 TrafficLightCycleElement(TrafficLightState.RED, 2)]
        light0 = TrafficLight(1, cycle, time_offset=0, position=np.array([10., 10.]))
        assert light0.get_state_at_time_step(8) == cycle[0].state

        light1 = TrafficLight(1, cycle, time_offset=3, position=np.array([10., 10.]))
        assert light1.get_state_at_time_step(5) == cycle[1].state

        light2 = TrafficLight(1, cycle, time_offset=10, position=np.array([10., 10.]))
        assert light2.get_state_at_time_step(12) == cycle[1].state

    def test_translate_rotate(self):
        cycle = [TrafficLightCycleElement(TrafficLightState.GREEN, 2),
                 TrafficLightCycleElement(TrafficLightState.YELLOW, 3),
                 TrafficLightCycleElement(TrafficLightState.RED, 2)]
        traffic_light = TrafficLight(1, cycle, time_offset=0, position=np.array([1., 1.]))

        traffic_light.translate_rotate(np.array([2, -4]), np.pi/2)

        desired_traffic_light_position = np.array([3, 3])

        np.testing.assert_array_almost_equal(traffic_light.position, desired_traffic_light_position)


if __name__ == '__main__':
    unittest.main()