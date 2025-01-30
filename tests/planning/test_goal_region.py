import math
import unittest

import numpy as np

from commonroad.common.util import AngleInterval, Interval
from commonroad.geometry.shape import Rectangle
from commonroad.planning.goal import GoalRegion
from commonroad.scenario.state import KSState, PMState, STState


class TestInitialization(unittest.TestCase):
    def test_valid_state_list(self):
        goal_state_1 = STState(time_step=Interval(5, 6))
        goal_state_2 = STState(time_step=Interval(5, 6), orientation=AngleInterval(1.5, 1.8))
        goal_state_3 = STState(time_step=Interval(2, 3))
        goal_state_4 = STState(time_step=Interval(1, 8), position=Rectangle(3, 5))
        goal_state_5 = STState(time_step=Interval(5, 6))
        input_states = [goal_state_1, goal_state_2, goal_state_3, goal_state_4, goal_state_5]
        goal_region = GoalRegion(input_states)
        self.assertEqual(goal_region.state_list[4].time_step, input_states[4].time_step)

    def test_lanelet_indicator(self):
        goal_region = GoalRegion([], {1: [4, 5, 9]})
        self.assertListEqual(goal_region.lanelets_of_goal_position[1], [4, 5, 9])

    def test_invalid_state_list(self):
        goal_state_1 = STState(time_step=5.0)
        goal_state_2 = STState(orientation=Interval(1.5, 1.8))
        goal_state_3 = STState(time_step=Interval(2, 3), yaw_rate=Interval(4, 6))

        with self.assertRaises(ValueError):
            GoalRegion([goal_state_1])
        with self.assertRaises(ValueError):
            GoalRegion([goal_state_2])
        with self.assertRaises(ValueError):
            GoalRegion([goal_state_3])

    def test_non_mutable_lanelet_list(self):
        original_dict = {1: [4, 5, 9]}
        new_dict = {2: [5, 9, 1, 4]}
        goal_region = GoalRegion([], original_dict)
        goal_region.lanelets_of_goal_position = new_dict
        self.assertDictEqual(goal_region.lanelets_of_goal_position, original_dict)

    def test_hash_goal_region(self):
        goal_state_1 = STState(time_step=Interval(5, 6))
        goal_state_2 = STState(time_step=Interval(5, 6), orientation=AngleInterval(1.5, 1.8))
        goal_state_3 = STState(time_step=Interval(2, 3))
        goal_state_4 = STState(time_step=Interval(1, 8), position=Rectangle(3, 5))
        goal_state_5 = STState(time_step=Interval(5, 6))
        input_states = [goal_state_1, goal_state_2, goal_state_3, goal_state_4, goal_state_5]
        goal_region_1 = GoalRegion(input_states)
        goal_region_2 = GoalRegion(input_states)
        self.assertEqual(goal_region_1.__hash__(), goal_region_2.__hash__())

    def test_goal_region_equality(self):
        goal_state_1 = STState(time_step=Interval(5, 6))
        goal_state_2 = STState(time_step=Interval(5, 6), orientation=AngleInterval(1.5, 1.8))
        goal_state_3 = STState(time_step=Interval(2, 3))
        goal_state_4 = STState(time_step=Interval(1, 8), position=Rectangle(3, 5))
        goal_state_5 = STState(time_step=Interval(5, 6))
        input_states = [goal_state_1, goal_state_2, goal_state_3, goal_state_4, goal_state_5]
        goal_region_1 = GoalRegion(input_states)
        goal_region_2 = GoalRegion(input_states)
        self.assertTrue(goal_region_1.__eq__(goal_region_2))


class TestIsReached(unittest.TestCase):
    def test_pm_model_goal_reached(self):
        """
        Test whether state velocities in the format v_x, v_y are in goals provided in format v, orientation.
        :return:
        """
        goal_state_1 = KSState(
            time_step=Interval(3.0, 3.2),
            orientation=AngleInterval(0.1, 1),
            velocity=Interval(20, 30.5),
        )
        # goal_state_2 = KSState(
        #     time_step=Interval(3.0, 3.1), orientation=AngleInterval(0.1, 1), velocity=Interval(15, 25.5)
        # )

        goal_region_1 = GoalRegion([goal_state_1])

        # in goal
        state_1 = PMState(time_step=3.1, velocity=21 * math.cos(0.5), velocity_y=21 * math.sin(0.5))

        self.assertTrue(goal_region_1.is_reached(state_1))

        # outside of goal
        state_3 = PMState(time_step=3.1, velocity=50 * math.cos(0.5), velocity_y=50 * math.sin(0.5))

        self.assertFalse(goal_region_1.is_reached(state_3))

    def test_intervals_reached(self):
        goal_state_1 = KSState(
            time_step=Interval(3.0, 3.2),
            orientation=AngleInterval(0.1, 1),
            velocity=Interval(20, 30.5),
        )
        goal_state_2 = KSState(
            time_step=Interval(3.0, 3.1),
            orientation=AngleInterval(0.1, 1),
            velocity=Interval(15, 25.5),
        )

        goal_region = GoalRegion([goal_state_1, goal_state_2])

        state_1 = KSState(time_step=3.1, orientation=0.5, velocity=25)
        state_2 = KSState(time_step=3.1, orientation=0.5, velocity=17)
        state_7 = KSState(time_step=3.2, orientation=0.5, velocity=30)
        state_8 = KSState(time_step=3.2, orientation=0.5 - 2 * np.pi, velocity=30)
        self.assertTrue(goal_region.is_reached(state_1))
        self.assertTrue(goal_region.is_reached(state_2))
        self.assertTrue(goal_region.is_reached(state_7))
        self.assertTrue(goal_region.is_reached(state_8))

        state_3 = KSState(time_step=2, orientation=0.5, velocity=25)
        state_4 = KSState(time_step=3.1, orientation=0.0, velocity=25)
        state_5 = KSState(time_step=3.1, orientation=0.5, velocity=10)
        state_6 = KSState(time_step=3.2, orientation=0.5, velocity=17)
        self.assertFalse(goal_region.is_reached(state_3))
        self.assertFalse(goal_region.is_reached(state_4))
        self.assertFalse(goal_region.is_reached(state_5))
        self.assertFalse(goal_region.is_reached(state_6))

        # Regression tests
        goal_state = KSState(
            time_step=Interval(3.0, 3.2),
            orientation=AngleInterval(-0.1965, 0.2034),
            velocity=Interval(20, 30.5),
        )
        goal_region = GoalRegion([goal_state])
        state = KSState(time_step=3.1, orientation=0.0034, velocity=25)
        self.assertTrue(goal_region.is_reached(state))


if __name__ == "__main__":
    unittest.main()
