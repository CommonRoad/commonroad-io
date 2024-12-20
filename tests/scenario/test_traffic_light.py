import unittest

import numpy as np

from commonroad.geometry.shape import Rectangle
from commonroad.scenario.traffic_light import (
    TrafficLight,
    TrafficLightCycle,
    TrafficLightCycleElement,
    TrafficLightDirection,
    TrafficLightState,
)


class TestTrafficLightCycle(unittest.TestCase):
    def test_initialization(self):
        cycle = [
            TrafficLightCycleElement(TrafficLightState.GREEN, 2),
            TrafficLightCycleElement(TrafficLightState.YELLOW, 3),
            TrafficLightCycleElement(TrafficLightState.RED, 2),
        ]

        light_cycle = TrafficLightCycle(cycle_elements=cycle, time_offset=0)
        self.assertEqual(light_cycle.cycle_elements, cycle)
        self.assertEqual(light_cycle.time_offset, 0)

    def test_hash(self):
        cycle = [TrafficLightCycleElement(TrafficLightState.GREEN, 2)]
        light_cycle0 = TrafficLightCycle(cycle, 0)

        light_cycle1 = TrafficLightCycle(cycle, 0)
        self.assertTrue(hash(light_cycle0) == hash(light_cycle1))

        light_cycle1 = TrafficLightCycle([TrafficLightCycleElement(TrafficLightState.GREEN, 1)], 0)
        self.assertFalse(hash(light_cycle0) == hash(light_cycle1))

        light_cycle1 = TrafficLightCycle(cycle, 1)
        self.assertFalse(hash(light_cycle0) == hash(light_cycle1))

        light_cycle1 = TrafficLightCycle(cycle, active=False)
        self.assertFalse(hash(light_cycle0) == hash(light_cycle1))

        light_cycle1 = TrafficLightCycle(cycle)
        self.assertTrue(hash(light_cycle0) == hash(light_cycle1))

    def test_equality(self):
        cycle = [
            TrafficLightCycleElement(TrafficLightState.GREEN, 2),
            TrafficLightCycleElement(TrafficLightState.RED, 2),
        ]

        light_cycle0 = TrafficLightCycle(cycle, 0)

        light_cycle1 = TrafficLightCycle(cycle, 0)
        self.assertTrue(light_cycle0 == light_cycle1)

        light_cycle1 = TrafficLightCycle([TrafficLightCycleElement(TrafficLightState.GREEN, 1)], 0)
        self.assertFalse(light_cycle0 == light_cycle1)

        light_cycle1 = TrafficLightCycle(cycle, 1)
        self.assertFalse(light_cycle0 == light_cycle1)

        light_cycle1 = TrafficLightCycle(cycle, active=False)
        self.assertFalse(light_cycle0 == light_cycle1)

        light_cycle1 = TrafficLightCycle(cycle)
        self.assertTrue(light_cycle0 == light_cycle1)

    def test_get_state_at_time_step(self):
        cycle = [
            TrafficLightCycleElement(TrafficLightState.GREEN, 2),
            TrafficLightCycleElement(TrafficLightState.YELLOW, 3),
            TrafficLightCycleElement(TrafficLightState.RED, 2),
        ]

        light_cycle0 = TrafficLightCycle(cycle_elements=cycle, time_offset=0)
        assert light_cycle0.get_state_at_time_step(8) == cycle[0].state

        light_cycle1 = TrafficLightCycle(cycle_elements=cycle, time_offset=3)
        assert light_cycle1.get_state_at_time_step(5) == cycle[1].state

        light_cycle2 = TrafficLightCycle(cycle_elements=cycle, time_offset=10)
        assert light_cycle2.get_state_at_time_step(12) == cycle[1].state

    def test_update_cycle_elements(self):
        # Verify that get_state_at_time_step is still correct after the cycle was updated
        cycle = [
            TrafficLightCycleElement(TrafficLightState.GREEN, 2),
            TrafficLightCycleElement(TrafficLightState.YELLOW, 3),
            TrafficLightCycleElement(TrafficLightState.RED, 2),
        ]
        light_cycle = TrafficLightCycle(cycle_elements=cycle, time_offset=0)
        assert light_cycle.get_state_at_time_step(7) == cycle[0].state

        new_cycle = cycle + [TrafficLightCycleElement(TrafficLightState.RED_YELLOW, 1)]
        light_cycle.cycle_elements = new_cycle
        assert light_cycle.get_state_at_time_step(7) == new_cycle[-1].state

    def test_update_time_offset(self):
        # Verify that get_state_at_time_step is still correct after the time offset was updated
        cycle = [
            TrafficLightCycleElement(TrafficLightState.GREEN, 2),
            TrafficLightCycleElement(TrafficLightState.YELLOW, 3),
            TrafficLightCycleElement(TrafficLightState.RED, 2),
        ]
        light_cycle = TrafficLightCycle(cycle_elements=cycle, time_offset=0)
        assert light_cycle.get_state_at_time_step(1) == cycle[0].state

        light_cycle.time_offset = 5
        assert light_cycle.get_state_at_time_step(1) == cycle[1].state


class TestTrafficLight(unittest.TestCase):
    def test_translate_rotate(self):
        color = [TrafficLightState.GREEN, TrafficLightState.YELLOW, TrafficLightState.RED]

        traffic_light = TrafficLight(1, position=np.array([1.0, 1.0]), color=color)

        traffic_light.translate_rotate(np.array([2, -4]), np.pi / 2)
        desired_traffic_light_position = np.array([3, 3])
        np.testing.assert_array_almost_equal(traffic_light.position, desired_traffic_light_position)

    def test_shape(self):
        color = [TrafficLightState.GREEN, TrafficLightState.YELLOW, TrafficLightState.RED]
        shape = Rectangle(0.5, 0.2, np.array([2, 3]))
        traffic_light = TrafficLight(1, position=np.array([1.0, 1.0]), color=color, shape=shape)

        self.assertEqual(traffic_light.shape.width, 0.2)
        traffic_light.shape = Rectangle(0.4, 0.3, np.array([1, 4]))
        self.assertEqual(traffic_light.shape.width, 0.3)

    def test_convert_to_2d(self):
        color = [TrafficLightState.GREEN, TrafficLightState.YELLOW, TrafficLightState.RED]
        traffic_light = TrafficLight(1, position=np.array([10.0, 7.0, 3.0]), color=color)

        traffic_light.convert_to_2d()

        self.assertEqual(traffic_light.position.shape, (2,))
        np.testing.assert_array_almost_equal(traffic_light.position, np.array([10.0, 7.0]))

    def test_equality(self):
        traffic_light_cycle = TrafficLightCycle([TrafficLightCycleElement(TrafficLightState.RED, 1)])
        color = [TrafficLightState.GREEN]
        traffic_light_1 = TrafficLight(
            234, np.array([10.0, 10.0]), None, color, shape=Rectangle(0.5, 0.2, np.array([2, 3]))
        )
        traffic_light_2 = TrafficLight(
            234, np.array([10.0, 10.0]), None, color, shape=Rectangle(0.5, 0.2, np.array([2, 3]))
        )
        self.assertTrue(traffic_light_1 == traffic_light_2)

        traffic_light_2 = TrafficLight(235, np.array([10.0, 10.0]), None, color)
        self.assertFalse(traffic_light_1 == traffic_light_2)

        color = [TrafficLightState.RED]
        traffic_light_2 = TrafficLight(234, np.array([10.0, 10.0]), None, color)
        self.assertFalse(traffic_light_1 == traffic_light_2)

        color = [TrafficLightState.GREEN]
        traffic_light_1.shape = None
        traffic_light_2 = TrafficLight(234, np.array([10.0, 2.5 * 4]), None, color)
        self.assertTrue(traffic_light_1 == traffic_light_2)

        traffic_light_1 = TrafficLight(234, np.array([1.0, 1.0]), None, color)
        traffic_light_2 = TrafficLight(234, np.array([1.0, 1.0]), None, color)
        self.assertTrue(traffic_light_1 == traffic_light_2)

        traffic_light_1 = TrafficLight(234, np.array([1.0, 1.0]))
        traffic_light_2 = TrafficLight(234, np.array([1.0, 1.0]))
        self.assertTrue(traffic_light_1 == traffic_light_2)

        traffic_light_1 = TrafficLight(234, np.array([10.0, 10.0]), traffic_light_cycle, color)
        traffic_light_2 = TrafficLight(234, np.array([10.0, 10.0]), traffic_light_cycle, color)
        self.assertTrue(traffic_light_1 == traffic_light_2)

        color = [TrafficLightState.GREEN]
        traffic_light_2 = TrafficLight(234, np.array([10.0, 9.95]), None, color)
        self.assertFalse(traffic_light_1 == traffic_light_2)

        traffic_light_2 = TrafficLight(234, np.array([10.0, 10.0]), None, color, True, TrafficLightDirection.LEFT)
        self.assertFalse(traffic_light_1 == traffic_light_2)

        traffic_light_2 = TrafficLight(234, np.array([10.0, 10.0]), None, color, True, TrafficLightDirection.STRAIGHT)
        self.assertFalse(traffic_light_1 == traffic_light_2)

        traffic_light_2 = TrafficLight(234, np.array([10.0, 10.0]), None, color, False, TrafficLightDirection.ALL)
        self.assertFalse(traffic_light_1 == traffic_light_2)

        traffic_light_1 = TrafficLight(234, np.array([10.0, 10.0]), traffic_light_cycle, color)
        traffic_light_2 = TrafficLight(234, np.array([10.0, 10.0]), None, color)
        self.assertFalse(traffic_light_1 == traffic_light_2)

    def test_hash(self):
        traffic_light_cycle = TrafficLightCycle([TrafficLightCycleElement(TrafficLightState.RED, 1)])
        color = [TrafficLightState.RED]
        traffic_light_1 = TrafficLight(234, np.array([10, 10]), None, color)
        traffic_light_2 = TrafficLight(234, np.array([10.0, 10.0]), None, color)
        self.assertEqual(hash(traffic_light_1), hash(traffic_light_2))

        traffic_light_2 = TrafficLight(234, np.array([10.0, 10.0000000001]), None, color)
        self.assertNotEqual(hash(traffic_light_1), hash(traffic_light_2))

        traffic_light_2 = TrafficLight(234, np.array([1.0, 1.0]), None, color)
        self.assertNotEqual(hash(traffic_light_1), hash(traffic_light_2))

        traffic_light_1 = TrafficLight(234, np.array([1.0, 1.0]), None, color)
        traffic_light_2 = TrafficLight(234, np.array([1.0, 1.0]), None, color)
        self.assertEqual(hash(traffic_light_1), hash(traffic_light_2))

        traffic_light_1 = TrafficLight(234, np.array([1.0, 1.0]))
        traffic_light_2 = TrafficLight(234, np.array([1.0, 1.0]))
        self.assertEqual(hash(traffic_light_1), hash(traffic_light_2))

        traffic_light_1 = TrafficLight(234, np.array([1.0, 1.0]), None, color)
        traffic_light_2 = TrafficLight(234, np.array([1.0, 1.0]), None)
        self.assertNotEqual(hash(traffic_light_1), hash(traffic_light_2))

        traffic_light_1 = TrafficLight(234, np.array([10, 10]), traffic_light_cycle, color)
        traffic_light_2 = TrafficLight(234, np.array([10.0, 10.0]), traffic_light_cycle, color)
        self.assertEqual(hash(traffic_light_1), hash(traffic_light_2))

        traffic_light_1 = TrafficLight(234, np.array([10, 10]), traffic_light_cycle, color)
        traffic_light_2 = TrafficLight(234, np.array([10.0, 10.0]), None, color)
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
