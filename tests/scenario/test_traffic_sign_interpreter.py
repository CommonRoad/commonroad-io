import unittest

import numpy as np

from commonroad.common.common_lanelet import LaneletType, RoadUser
from commonroad.scenario.lanelet import Lanelet, LaneletNetwork
from commonroad.scenario.traffic_sign import (
    SupportedTrafficSignCountry,
    TrafficSign,
    TrafficSignElement,
    TrafficSignIDZamunda,
)
from commonroad.scenario.traffic_sign_interpreter import TrafficSignInterpreter


class TestTrafficSignInterpreter(unittest.TestCase):
    def setUp(self):
        lanelet_one = Lanelet(
            left_vertices=np.array([[0.0, 0.0], [1.0, 0.0], [2, 0]]),
            center_vertices=np.array([[0.0, 1], [1.0, 1], [2, 1]]),
            right_vertices=np.array([[0.0, 2], [1.0, 2], [2, 2]]),
            lanelet_id=100,
            lanelet_type={LaneletType.HIGHWAY, LaneletType.MAIN_CARRIAGE_WAY},
            user_one_way={RoadUser.VEHICLE},
        )
        lanelet_two = Lanelet(
            left_vertices=np.array([[0.0, 2.0], [1.0, 2.0], [2, 3]]),
            center_vertices=np.array([[0.0, 3], [1.0, 3], [2, 3]]),
            right_vertices=np.array([[0.0, 4], [1.0, 4], [2, 4]]),
            lanelet_id=101,
            lanelet_type={LaneletType.HIGHWAY, LaneletType.MAIN_CARRIAGE_WAY},
            user_one_way={RoadUser.VEHICLE},
        )
        lanelet_three = Lanelet(
            left_vertices=np.array([[0.0, 3.0], [1.0, 3.0], [2, 3]]),
            center_vertices=np.array([[0.0, 4], [1.0, 4], [2, 4]]),
            right_vertices=np.array([[0.0, 5], [1.0, 5], [2, 5]]),
            lanelet_id=102,
            lanelet_type={LaneletType.HIGHWAY, LaneletType.MAIN_CARRIAGE_WAY},
            user_one_way={RoadUser.VEHICLE},
        )
        lanelet_four = Lanelet(
            left_vertices=np.array([[0.0, 3.0], [1.0, 3.0], [2, 3]]),
            center_vertices=np.array([[0.0, 4], [1.0, 4], [2, 4]]),
            right_vertices=np.array([[0.0, 5], [1.0, 5], [2, 5]]),
            lanelet_id=103,
            lanelet_type={LaneletType.HIGHWAY, LaneletType.MAIN_CARRIAGE_WAY},
            user_one_way={RoadUser.VEHICLE},
        )
        lanelet_five = Lanelet(
            left_vertices=np.array([[0.0, 3.0], [1.0, 3.0], [2, 3]]),
            center_vertices=np.array([[0.0, 4], [1.0, 4], [2, 4]]),
            right_vertices=np.array([[0.0, 5], [1.0, 5], [2, 5]]),
            lanelet_id=104,
            lanelet_type={LaneletType.HIGHWAY, LaneletType.MAIN_CARRIAGE_WAY},
            user_one_way={RoadUser.VEHICLE},
        )
        traffic_sign_one = TrafficSign(
            201, [TrafficSignElement(TrafficSignIDZamunda.MAX_SPEED, ["20"])], {100}, np.array([0.0, 2])
        )
        traffic_sign_two = TrafficSign(
            202, [TrafficSignElement(TrafficSignIDZamunda.MAX_SPEED, ["30"])], {101}, np.array([0.0, 4])
        )
        traffic_sign_three = TrafficSign(
            203, [TrafficSignElement(TrafficSignIDZamunda.MAX_SPEED, ["40"])], {102}, np.array([0.0, 5])
        )
        traffic_sign_four = TrafficSign(
            204, [TrafficSignElement(TrafficSignIDZamunda.MIN_SPEED, ["50"])], {103}, np.array([0.0, 5])
        )

        lanelet_network = LaneletNetwork().create_from_lanelet_list(
            [lanelet_one, lanelet_two, lanelet_three, lanelet_four, lanelet_five]
        )
        lanelet_network.add_traffic_sign(traffic_sign_one, {100})
        lanelet_network.add_traffic_sign(traffic_sign_two, {101})
        lanelet_network.add_traffic_sign(traffic_sign_three, {102})
        lanelet_network.add_traffic_sign(traffic_sign_four, {103})

        self.interpreter = TrafficSignInterpreter(SupportedTrafficSignCountry.ZAMUNDA, lanelet_network)

    def test_speed_limit(self):
        self.assertEqual(20, self.interpreter.speed_limit(frozenset({100, 101, 102})))
        self.assertEqual(20, self.interpreter.speed_limit(frozenset({101, 100, 102})))
        self.assertEqual(20, self.interpreter.speed_limit(frozenset({102, 100, 101})))
        self.assertEqual(30, self.interpreter.speed_limit(frozenset({101, 102})))
        self.assertEqual(None, self.interpreter.speed_limit(frozenset({103, 104})))
        self.assertEqual(40, self.interpreter.speed_limit(frozenset({102, 104})))
        self.assertEqual(None, self.interpreter.required_speed(frozenset({102, 104})))
        self.assertEqual(50, self.interpreter.required_speed(frozenset({103, 104})))


if __name__ == "__main__":
    unittest.main()
