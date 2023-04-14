import numpy as np
import unittest

from commonroad.scenario.traffic_light import TrafficLightState, TrafficLightDirection, TrafficLightCycleElement, \
    TrafficLight


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

    def test_equality(self):
        cycle = [TrafficLightCycleElement(TrafficLightState.GREEN, 2)]
        traffic_light_1 = TrafficLight(234, cycle, np.array([10., 10.]), 5)
        traffic_light_2 = TrafficLight(234, cycle, np.array([10., 10.]), 5)
        self.assertTrue(traffic_light_1 == traffic_light_2)

        traffic_light_2 = TrafficLight(235, cycle, np.array([10., 10.]), 5)
        self.assertFalse(traffic_light_1 == traffic_light_2)

        cycle = [TrafficLightCycleElement(TrafficLightState.RED, 2)]
        traffic_light_2 = TrafficLight(234, cycle, np.array([10., 10.]), 5)
        self.assertFalse(traffic_light_1 == traffic_light_2)

        cycle = [TrafficLightCycleElement(TrafficLightState.GREEN, 2)]
        traffic_light_2 = TrafficLight(234, cycle, np.array([10., 2.5 * 4]), 5)
        self.assertTrue(traffic_light_1 == traffic_light_2)

        cycle = [TrafficLightCycleElement(TrafficLightState.GREEN, 2)]
        traffic_light_2 = TrafficLight(234, cycle, np.array([10., 9.95]), 5)
        self.assertFalse(traffic_light_1 == traffic_light_2)

        traffic_light_2 = TrafficLight(234, cycle, np.array([10., 10.]), 6)
        self.assertFalse(traffic_light_1 == traffic_light_2)

        traffic_light_2 = TrafficLight(234, cycle, np.array([10., 10.]), 5, TrafficLightDirection.STRAIGHT)
        self.assertFalse(traffic_light_1 == traffic_light_2)

        traffic_light_2 = TrafficLight(234, cycle, np.array([10., 10.]), 5, active=False)
        self.assertFalse(traffic_light_1 == traffic_light_2)

        traffic_light_2 = TrafficLight(234, cycle, None, 5, active=False)
        self.assertFalse(traffic_light_1 == traffic_light_2)
        self.assertFalse(traffic_light_2 == traffic_light_1)

        traffic_light_1 = TrafficLight(234, cycle, None, 5)
        traffic_light_2 = TrafficLight(234, cycle, None, 5)
        self.assertTrue(traffic_light_1 == traffic_light_2)

        traffic_light_1 = TrafficLight(234)
        traffic_light_2 = TrafficLight(234)
        self.assertTrue(traffic_light_1 == traffic_light_2)

    def test_hash(self):
        cycle = [TrafficLightCycleElement(TrafficLightState.RED, 2)]
        traffic_light_1 = TrafficLight(234, cycle, np.array([10, 10]), 5)
        traffic_light_2 = TrafficLight(234, cycle, np.array([10., 10.]), 5)
        self.assertEqual(hash(traffic_light_1), hash(traffic_light_2))

        traffic_light_2 = TrafficLight(234, cycle, np.array([10., 10.0000000001]), 5)
        self.assertNotEqual(hash(traffic_light_1), hash(traffic_light_2))

        traffic_light_2 = TrafficLight(234, cycle, None, 5)
        self.assertNotEqual(hash(traffic_light_1), hash(traffic_light_2))

        traffic_light_1 = TrafficLight(234, cycle, None, 5)
        traffic_light_2 = TrafficLight(234, cycle, None, 5)
        self.assertEqual(hash(traffic_light_1), hash(traffic_light_2))

        traffic_light_1 = TrafficLight(234, None, None, 5)
        traffic_light_2 = TrafficLight(234, None, None, 5)
        self.assertEqual(hash(traffic_light_1), hash(traffic_light_2))

        traffic_light_1 = TrafficLight(234, cycle, None, 5)
        traffic_light_2 = TrafficLight(234, None, None, 5)
        self.assertNotEqual(hash(traffic_light_1), hash(traffic_light_2))


class TestTrafficLightCycleElement(unittest.TestCase):
    def test_equality(self):
        cycle_element_1 = TrafficLightCycleElement(TrafficLightState.GREEN, 2)
        cycle_element_2 = TrafficLightCycleElement(TrafficLightState.GREEN, 2)
        self.assertTrue(cycle_element_1 == cycle_element_2)

        cycle_element_2 = TrafficLightCycleElement(TrafficLightState.YELLOW, 2)
        self.assertFalse(cycle_element_1 == cycle_element_2)

        cycle_element_2 = TrafficLightCycleElement(TrafficLightState.GREEN, 7)
        self.assertFalse(cycle_element_1 == cycle_element_2)

    def test_hash(self):
        cycle_element_1 = TrafficLightCycleElement(TrafficLightState.GREEN, 2)
        cycle_element_2 = TrafficLightCycleElement(TrafficLightState.GREEN, 2)
        self.assertEqual(hash(cycle_element_1), hash(cycle_element_2))

        cycle_element_2 = TrafficLightCycleElement(TrafficLightState.GREEN, 3)
        self.assertNotEqual(hash(cycle_element_1), hash(cycle_element_2))
