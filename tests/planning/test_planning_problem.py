import unittest

import numpy as np

from commonroad.common.util import AngleInterval, Interval
from commonroad.geometry.shape import Circle, Rectangle
from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.state import InitialState, KSState, STState
from commonroad.scenario.trajectory import Trajectory


class TestTranslateRotate(unittest.TestCase):
    def test_translate(self):
        translation = np.array((10.0, 1.0))
        angle = 0.0
        pos = np.array((1.0, 1.0))
        initial_state = InitialState(
            position=pos, velocity=10.0, orientation=0.0, yaw_rate=0, slip_angle=0, time_step=1
        )

        shape1 = Rectangle(2.0, 4.0, np.array((2.0, 2.0)))
        shape2 = Circle(2.5, np.array((-1.0, 1.0)))

        goal_state_1 = KSState(position=shape1, time_step=Interval(0, 5))
        goal_state_2 = KSState(position=shape2, time_step=Interval(0, 2))
        goal_region = GoalRegion([goal_state_1, goal_state_2])
        planning_problem = PlanningProblem(1, initial_state, goal_region)

        planning_problem.translate_rotate(translation, angle)

        self.assertAlmostEqual(planning_problem.initial_state.position[0], pos[0] + translation[0])
        self.assertAlmostEqual(planning_problem.initial_state.position[1], pos[1] + translation[1])
        self.assertAlmostEqual(planning_problem.initial_state.orientation, 0.0)

        self.assertAlmostEqual(
            planning_problem.goal.state_list[0].position.center[0], shape1.center[0] + translation[0]
        )
        self.assertAlmostEqual(
            planning_problem.goal.state_list[0].position.center[1], shape1.center[1] + translation[1]
        )
        self.assertAlmostEqual(
            planning_problem.goal.state_list[1].position.center[0], shape2.center[0] + translation[0]
        )
        self.assertAlmostEqual(
            planning_problem.goal.state_list[1].position.center[1], shape2.center[1] + translation[1]
        )

    def test_rotate(self):
        translation = np.array((0.0, 0.0))
        angle = np.pi / 4
        pos = np.array((1.0, 1.0))
        initial_state = InitialState(
            position=pos, velocity=10.0, orientation=np.pi / 2, yaw_rate=0, slip_angle=0, time_step=2
        )

        shape1 = Rectangle(2.0, 4.0, np.array((2.0, 2.0)))
        shape2 = Circle(2.5, np.array((-1.0, 1.0)))

        goal_state_1 = STState(
            position=shape1, time_step=Interval(0, 5), orientation=AngleInterval(np.pi / 8, 3 * np.pi / 8)
        )
        goal_state_2 = STState(
            position=shape2, time_step=Interval(0, 2), orientation=AngleInterval(3 * np.pi / 4, np.pi)
        )
        goal_region = GoalRegion([goal_state_1, goal_state_2])
        planning_problem = PlanningProblem(1, initial_state, goal_region)

        planning_problem.translate_rotate(translation, angle)

        self.assertAlmostEqual(planning_problem.initial_state.orientation, np.pi / 2 + angle)

        self.assertAlmostEqual(planning_problem.goal.state_list[0].orientation.start, angle + np.pi / 8)
        self.assertAlmostEqual(planning_problem.goal.state_list[0].orientation.end, angle + 3 * np.pi / 8)
        self.assertAlmostEqual(planning_problem.goal.state_list[1].orientation.start, angle + 3 * np.pi / 4)
        self.assertAlmostEqual(planning_problem.goal.state_list[1].orientation.end, angle + np.pi)

    def test_goal_reached(self):
        pos = np.array((0.0, -3.0))
        initial_state = InitialState(
            position=pos, velocity=10.0, orientation=0.0, yaw_rate=0, slip_angle=0, time_step=1
        )

        shape1 = Rectangle(2.0, 4.0, np.array((2.0, 2.0)))
        shape2 = Circle(2.5, np.array((-1.0, 1.0)))

        goal_state_1 = STState(position=shape1, time_step=Interval(0, 5), orientation=AngleInterval(0.0, 0.1))
        goal_state_2 = STState(position=shape2, time_step=Interval(0, 2), orientation=AngleInterval(0.0, 0.1))
        goal_region = GoalRegion([goal_state_1, goal_state_2])
        planning_problem = PlanningProblem(1, initial_state, goal_region)

        state_reached = STState(
            position=np.array([2, 2]), velocity=10, orientation=0.0, yaw_rate=0, slip_angle=0, time_step=1
        )
        state_not_reached = STState(
            position=np.array([0, -6]), velocity=10, orientation=(3 / 2) * np.pi, yaw_rate=0, slip_angle=0, time_step=1
        )
        trajectory_reached = Trajectory(1, [state_reached])
        trajectory_not_reached = Trajectory(1, [state_not_reached])

        self.assertTrue(planning_problem.goal_reached(trajectory_reached)[0])
        self.assertFalse(planning_problem.goal_reached(trajectory_not_reached)[0])


if __name__ == "__main__":
    unittest.main()
