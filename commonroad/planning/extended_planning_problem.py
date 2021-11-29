"""

"""
from abc import ABC

from commonroad.scenario.trajectory import State, Trajectory
from commonroad.planning.goal import GoalRegion
from commonroad.visualization.drawable import IDrawable
from commonroad_route_planner.route_planner import RoutePlanner
from planning_problem import PlanningProblem
from commonroad.scenario.scenario import Scenario

from typing import List, Union, Dict


class ExtendedPlanningProblem(IDrawable):
    def _init__(self, planningProblem: PlanningProblem = None, scenario: Scenario = None):
        self.planning_problem_id = planningProblem.planning_problem_id
        self.initial_state = planningProblem.initial_state
        self.goal = planningProblem.goal
        route_planner = RoutePlanner(scenario, planningProblem, backend=RoutePlanner.Backend.NETWORKX_REVERSED)
        candidate_holder = route_planner.plan_routes()
        route = candidate_holder.retrieve_best_route_by_orientation()
        state_list = route.reference_path
        self.state_list = state_list

    def planning_problem_route_coordinates(self) -> List:
        """List of coordinates of the planning problem route"""
        return self.state_list

    @property
    def planning_problem_id(self) -> int:
        """Id of the planning problem"""
        return self.planning_problem_id

    @property
    def initial_state(self) -> State:
        """Initial state of the ego vehicle"""
        return self.initial_state

    @property
    def goal(self) -> GoalRegion:
        """Region that has to be reached"""
        return self.goal


class ExtendedPlanningProblemSet(IDrawable):
    def __init__(self, extended_planning_problem_list: Union[None, List[ExtendedPlanningProblem]] = None):
        if extended_planning_problem_list is None:
            extended_planning_problem_list = []

        self._extended_planning_problem_dict = {extended_planning_problem.planning_problem_id: extended_planning_problem
                                                for extended_planning_problem in extended_planning_problem_list}


    @property
    def planning_problem_dict(self) -> Dict[int, PlanningProblem]:
        """Dict that contains all PlanningProblems that are added. Keys: Ids of planning problems"""
        return self._extended_planning_problem_dict

