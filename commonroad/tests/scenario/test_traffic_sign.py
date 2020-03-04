import unittest
import numpy as np
from commonroad.scenario.traffic_sign import TrafficLight, TrafficLightCycleElement, TrafficLightState


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


if __name__ == '__main__':
    unittest.main()