from abc import ABC, abstractmethod
from typing import Tuple

from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.scenario import Scenario

__author__ = "Stefanie Manzinger, Sebastian Maierhofer"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["Priority Program SPP 1835 Cooperative Interacting Automobiles", "CAR@TUM"]
__version__ = "2022.1"
__maintainer__ = "Stefanie Manzinger, Sebastian Maierhofer"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Released"


class FileReader(ABC):
    """
    Interface for reading CommonRoad files in a specific format.
    """

    def __init__(self, filename: str):
        self._filename = filename

    @abstractmethod
    def open(self, lanelet_assignment: bool = False) -> Tuple[Scenario, PlanningProblemSet]:
        pass

    @abstractmethod
    def open_lanelet_network(self) -> LaneletNetwork:
        pass
