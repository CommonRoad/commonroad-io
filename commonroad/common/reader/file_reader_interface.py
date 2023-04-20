from abc import ABC, abstractmethod
from typing import Tuple

from commonroad.common.util import Path_T
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.scenario import Scenario


class FileReader(ABC):
    """
    Interface for reading CommonRoad files in a specific format.
    """

    def __init__(self, filename: Path_T):
        self._filename = filename

    @abstractmethod
    def open(self, lanelet_assignment: bool = False) -> Tuple[Scenario, PlanningProblemSet]:
        pass

    @abstractmethod
    def open_lanelet_network(self) -> LaneletNetwork:
        pass
