import unittest
from typing import Optional

import numpy as np

from commonroad.planning.goal import GoalRegion, Interval
from commonroad.planning.planner_interface import TrajectoryPlannerInterface
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import InitialState, PMState
from commonroad.scenario.trajectory import Trajectory


class TestPlanner(TrajectoryPlannerInterface):
    def plan(
        self, sc: Scenario, pp: PlanningProblem, ref_path: Optional[np.ndarray] = None
    ) -> Trajectory:
        return Trajectory(
            0,
            [
                PMState(time_step=0, position=np.ndarray([0, 0]), velocity=10, velocity_y=0),
                PMState(time_step=1, position=np.ndarray([1, 0]), velocity=00, velocity_y=0),
            ],
        )


class TestMotionPlannerInterface(unittest.TestCase):
    def test_plan(self):
        planner = TestPlanner()
        traj = planner.plan(
            Scenario(0.1),
            PlanningProblem(
                planning_problem_id=1,
                initial_state=InitialState(
                    position=np.array([0, 0]),
                    velocity=00.0,
                    orientation=0.0,
                    yaw_rate=0,
                    slip_angle=0,
                    time_step=1,
                ),
                goal_region=GoalRegion([PMState(time_step=Interval(1, 4))]),
            ),
            ref_path=np.ndarray([]),
        )
        self.assertEqual(traj.state_list[0].velocity, 10)
        self.assertEqual(traj.state_list[1].velocity, 0)
