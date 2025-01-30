import unittest

import numpy as np

from commonroad.common.util import AngleInterval, Interval
from commonroad.geometry.shape import Circle, Rectangle
from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from commonroad.scenario.state import InitialState, State, STState


class TestPlanningProblemSet(unittest.TestCase):
    def test_initialize_empty(self):
        problem_set = PlanningProblemSet()
        self.assertEqual(problem_set.planning_problem_dict, {})
        with self.assertRaises(KeyError):
            problem_set.find_planning_problem_by_id(1)

    def test_initialize_error(self):
        with self.assertRaises(AssertionError):
            PlanningProblemSet([4])
        with self.assertRaises(AssertionError):
            PlanningProblemSet("test")
        with self.assertRaises(AssertionError):
            PlanningProblemSet(1)

    def test_add_planning_problem(self):
        state_1 = InitialState(
            time_step=1,
            position=np.array([3, 5]),
            velocity=14,
            orientation=0,
            yaw_rate=2,
            slip_angle=4,
        )
        state_2 = InitialState(
            time_step=2,
            position=np.array([3, 5]),
            velocity=14,
            orientation=0,
            yaw_rate=2,
            slip_angle=4,
        )
        planning_problem = PlanningProblem(
            1, state_1, GoalRegion([State(time_step=Interval(2, 3))])
        )
        planning_problem_new = PlanningProblem(
            2, state_2, GoalRegion([State(time_step=Interval(1, 5))])
        )
        problem_set = PlanningProblemSet([planning_problem])
        problem_set.add_planning_problem(planning_problem_new)
        self.assertEqual(len(problem_set.planning_problem_dict), 2)
        self.assertEqual(problem_set.planning_problem_dict[2].initial_state.time_step, 2)

    def test_id_already_used(self):
        state = InitialState(
            time_step=1,
            position=np.array([3, 5]),
            velocity=14,
            orientation=0,
            yaw_rate=2,
            slip_angle=4,
        )
        planning_problem = PlanningProblem(1, state, GoalRegion([]))
        planning_problem_new = PlanningProblem(1, state, GoalRegion([]))
        problem_set = PlanningProblemSet([planning_problem])
        with self.assertRaises(ValueError):
            problem_set.add_planning_problem(planning_problem_new)

    def test_immutable_dict(self):
        state = InitialState(
            time_step=1,
            position=np.array([3, 5]),
            velocity=14,
            orientation=0,
            yaw_rate=2,
            slip_angle=4,
        )
        planning_problem = PlanningProblem(1, state, GoalRegion([]))
        problem_set = PlanningProblemSet([planning_problem])
        problem_set.planning_problem_dict = {}
        self.assertEqual(len(problem_set.planning_problem_dict), 1)

    def test_find_planning_problem(self):
        state_1 = InitialState(
            time_step=1,
            position=np.array([3, 5]),
            velocity=14,
            orientation=0,
            yaw_rate=2,
            slip_angle=4,
        )
        state_2 = InitialState(
            time_step=2,
            position=np.array([3, 5]),
            velocity=14,
            orientation=0,
            yaw_rate=2,
            slip_angle=4,
        )
        planning_problem_1 = PlanningProblem(1, state_1, GoalRegion([]))
        planning_problem_2 = PlanningProblem(2, state_2, GoalRegion([]))
        problem_set = PlanningProblemSet([planning_problem_1, planning_problem_2])
        self.assertEqual(problem_set.find_planning_problem_by_id(1).initial_state.time_step, 1)
        self.assertEqual(problem_set.find_planning_problem_by_id(2).initial_state.time_step, 2)
        with self.assertRaises(KeyError):
            print(problem_set.find_planning_problem_by_id(3))

    def test_translate(self):
        translation = np.array((10.0, 1.0))
        angle = 0.0
        pos = np.array((1.0, 1.0))
        initial_state = InitialState(
            position=pos, velocity=10.0, orientation=0.0, yaw_rate=0, slip_angle=0, time_step=1
        )

        shape1 = Rectangle(2.0, 4.0, np.array((2.0, 2.0)))
        shape2 = Circle(2.5, np.array((-1.0, 1.0)))

        goal_state_1 = STState(position=shape1, time_step=Interval(0, 5))
        goal_state_2 = STState(position=shape2, time_step=Interval(0, 2))
        goal_region = GoalRegion([goal_state_1, goal_state_2])
        planning_problem = PlanningProblem(1, initial_state, goal_region)
        problem_set = PlanningProblemSet([planning_problem])
        problem_set.translate_rotate(translation, angle)

        self.assertAlmostEqual(
            problem_set.planning_problem_dict[1].initial_state.position[0], pos[0] + translation[0]
        )
        self.assertAlmostEqual(
            problem_set.planning_problem_dict[1].initial_state.position[1], pos[1] + translation[1]
        )
        self.assertAlmostEqual(problem_set.planning_problem_dict[1].initial_state.orientation, 0.0)

        self.assertAlmostEqual(
            problem_set.planning_problem_dict[1].goal.state_list[0].position.center[0],
            shape1.center[0] + translation[0],
        )
        self.assertAlmostEqual(
            problem_set.planning_problem_dict[1].goal.state_list[0].position.center[1],
            shape1.center[1] + translation[1],
        )
        self.assertAlmostEqual(
            problem_set.planning_problem_dict[1].goal.state_list[1].position.center[0],
            shape2.center[0] + translation[0],
        )
        self.assertAlmostEqual(
            problem_set.planning_problem_dict[1].goal.state_list[1].position.center[1],
            shape2.center[1] + translation[1],
        )

    def test_rotate(self):
        translation = np.array((0.0, 0.0))
        angle = np.pi / 4
        pos = np.array((1.0, 1.0))
        initial_state = InitialState(
            position=pos,
            velocity=10.0,
            orientation=np.pi / 2,
            yaw_rate=0,
            slip_angle=0,
            time_step=2,
        )

        shape1 = Rectangle(2.0, 4.0, np.array((2.0, 2.0)))
        shape2 = Circle(2.5, np.array((-1.0, 1.0)))

        goal_state_1 = STState(
            position=shape1,
            time_step=Interval(0, 5),
            orientation=AngleInterval(np.pi / 8, 3 * np.pi / 8),
        )
        goal_state_2 = STState(
            position=shape2,
            time_step=Interval(0, 2),
            orientation=AngleInterval(3 * np.pi / 4, np.pi),
        )
        goal_region = GoalRegion([goal_state_1, goal_state_2])
        planning_problem = PlanningProblem(1, initial_state, goal_region)

        problem_set = PlanningProblemSet([planning_problem])
        problem_set.translate_rotate(translation, angle)

        self.assertAlmostEqual(
            problem_set.planning_problem_dict[1].initial_state.orientation, np.pi / 2 + angle
        )

        self.assertAlmostEqual(
            problem_set.planning_problem_dict[1].goal.state_list[0].orientation.start,
            angle + np.pi / 8,
        )
        self.assertAlmostEqual(
            problem_set.planning_problem_dict[1].goal.state_list[0].orientation.end,
            angle + 3 * np.pi / 8,
        )
        self.assertAlmostEqual(
            problem_set.planning_problem_dict[1].goal.state_list[1].orientation.start,
            angle + 3 * np.pi / 4,
        )
        self.assertAlmostEqual(
            problem_set.planning_problem_dict[1].goal.state_list[1].orientation.end, angle + np.pi
        )

    def test_hash_problem(self):
        state_1 = InitialState(
            time_step=1,
            position=np.array([3, 5]),
            velocity=14,
            orientation=0,
            yaw_rate=2,
            slip_angle=4,
        )
        state_2 = InitialState(
            time_step=2,
            position=np.array([3, 5]),
            velocity=14,
            orientation=0,
            yaw_rate=2,
            slip_angle=4,
        )
        planning_problem_1 = PlanningProblem(
            1, state_1, GoalRegion([State(time_step=Interval(2, 3))])
        )
        planning_problem_2 = PlanningProblem(
            1, state_1, GoalRegion([State(time_step=Interval(2, 3))])
        )
        planning_problem_3 = PlanningProblem(
            2, state_2, GoalRegion([State(time_step=Interval(1, 5))])
        )
        self.assertEqual(planning_problem_1.__hash__(), planning_problem_2.__hash__())
        self.assertNotEqual(planning_problem_1.__hash__(), planning_problem_3.__hash__())

    def test_equality_problem(self):
        state_1 = InitialState(
            time_step=1,
            position=np.array([3, 5]),
            velocity=14,
            orientation=0,
            yaw_rate=2,
            slip_angle=4,
        )
        state_2 = InitialState(
            time_step=2,
            position=np.array([3, 5]),
            velocity=14,
            orientation=0,
            yaw_rate=2,
            slip_angle=4,
        )
        planning_problem_1 = PlanningProblem(
            1, state_1, GoalRegion([State(time_step=Interval(2, 3))])
        )
        planning_problem_2 = PlanningProblem(
            1, state_1, GoalRegion([State(time_step=Interval(2, 3))])
        )
        planning_problem_3 = PlanningProblem(
            2, state_2, GoalRegion([State(time_step=Interval(1, 5))])
        )
        self.assertTrue(planning_problem_1.__eq__(planning_problem_2))
        self.assertFalse(planning_problem_1.__eq__(planning_problem_3))

    def test_hash_problem_set(self):
        state_1 = InitialState(
            time_step=1,
            position=np.array([3, 5]),
            velocity=14,
            orientation=0,
            yaw_rate=2,
            slip_angle=4,
        )
        state_2 = InitialState(
            time_step=2,
            position=np.array([3, 5]),
            velocity=14,
            orientation=0,
            yaw_rate=2,
            slip_angle=4,
        )
        planning_problem = PlanningProblem(
            1, state_1, GoalRegion([State(time_step=Interval(2, 3))])
        )
        planning_problem_new = PlanningProblem(
            2, state_2, GoalRegion([State(time_step=Interval(1, 5))])
        )
        problem_set_1 = PlanningProblemSet([planning_problem])
        problem_set_2 = PlanningProblemSet([planning_problem])
        problem_set_3 = PlanningProblemSet([planning_problem_new])
        self.assertEqual(problem_set_1.__hash__(), problem_set_2.__hash__())
        self.assertNotEqual(problem_set_1.__hash__(), problem_set_3.__hash__())

    def test_equality_problem_set(self):
        state_1 = InitialState(
            time_step=1,
            position=np.array([3, 5]),
            velocity=14,
            orientation=0,
            yaw_rate=2,
            slip_angle=4,
        )
        state_2 = InitialState(
            time_step=2,
            position=np.array([3, 5]),
            velocity=14,
            orientation=0,
            yaw_rate=2,
            slip_angle=4,
        )
        planning_problem = PlanningProblem(
            1, state_1, GoalRegion([State(time_step=Interval(2, 3))])
        )
        planning_problem_new = PlanningProblem(
            2, state_2, GoalRegion([State(time_step=Interval(1, 5))])
        )
        problem_set_1 = PlanningProblemSet([planning_problem])
        problem_set_2 = PlanningProblemSet([planning_problem])
        problem_set_3 = PlanningProblemSet([planning_problem_new])
        self.assertTrue(problem_set_1.__eq__(problem_set_2))
        self.assertFalse(problem_set_1.__eq__(problem_set_3))


if __name__ == "__main__":
    unittest.main()
