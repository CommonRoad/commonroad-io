import warnings
from typing import List, Union

from commonroad.common.common_scenario import ScenarioMetaInformation
from commonroad.planning.planning_problem import PlanningProblem, CooperativePlanningProblem


class ScenarioInterface:
    """
    Class that represents the scenario interface used in the protobuf format
    """

    def __init__(
        self,
        scenario_meta_information: ScenarioMetaInformation,
        map_id: str,
        dynamic_id: str,
        planning_problems: List[PlanningProblem] = None,
        cooperative_planning_problems: List[CooperativePlanningProblem] = None,
    ):
        """
        :param scenario_meta_information: information about the scenario
        :param map_id: id of the map that corresponds to the scenario
        :param dynamic_id: id of the dynamic that corresponds to the scenario
        :param planning_problems: list of planning problems that correspond to the scenario
        :param cooperative_planning_problems: list of cooperative planning problems that correspond to the scenario
        """
        self._scenario_meta_information = scenario_meta_information
        self._map_id = map_id
        self._dynamic_id = dynamic_id
        if planning_problems is None:
            self._planning_problems = []
        else:
            self._planning_problems = planning_problems
        if cooperative_planning_problems is None:
            self._cooperative_planning_problems = []
        else:
            self._cooperative_planning_problems = cooperative_planning_problems

    def __eq__(self, other):
        if not isinstance(other, ScenarioInterface):
            warnings.warn(f"Inequality between ScenarioInterface {repr(self)} and different type {type(other)}")
            return False

        return (
            self._scenario_meta_information == other.scenario_meta_information
            and self._map_id == other.map_id
            and self._dynamic_id == other.dynamic_id
            and self._planning_problems == other.planning_problems
            and self._cooperative_planning_problems == other.cooperative_planning_problems
        )

    def __hash__(self):
        return hash(
            (
                self._scenario_meta_information,
                self._map_id,
                self._dynamic_id,
                frozenset(self._planning_problems),
                frozenset(self._cooperative_planning_problems),
            )
        )

    @property
    def scenario_meta_information(self) -> ScenarioMetaInformation:
        """information about the scenario."""
        return self._scenario_meta_information

    @scenario_meta_information.setter
    def scenario_meta_information(self, information: ScenarioMetaInformation):
        self._scenario_meta_information = information

    @property
    def map_id(self) -> str:
        """id of the map that corresponds to the scenario."""
        return self._map_id

    @map_id.setter
    def map_id(self, map_id: str):
        self._map_id = map_id

    @property
    def dynamic_id(self) -> str:
        """id of the dynamic that corresponds to the scenario."""
        return self._dynamic_id

    @dynamic_id.setter
    def dynamic_id(self, dynamic_id: str):
        self._dynamic_id = dynamic_id

    @property
    def planning_problems(self) -> Union[None, List[PlanningProblem]]:
        """list of planning problems that correspond to the scenario."""
        return self._planning_problems

    @planning_problems.setter
    def planning_problems(self, planning_problems: Union[None, List[PlanningProblem]]):
        if planning_problems is None:
            self._planning_problems = []
        else:
            self._planning_problems = planning_problems

    @property
    def cooperative_planning_problems(self) -> Union[None, List[CooperativePlanningProblem]]:
        """list of cooperative planning problems that correspond to the scenario."""
        return self._cooperative_planning_problems

    @cooperative_planning_problems.setter
    def cooperative_planning_problems(
        self, cooperative_planning_problems: Union[None, List[CooperativePlanningProblem]]
    ):
        if cooperative_planning_problems is None:
            self._cooperative_planning_problems = []
        else:
            self._cooperative_planning_problems = cooperative_planning_problems
