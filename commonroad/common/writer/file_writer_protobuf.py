from typing import Set

from commonroad.common.writer.file_writer import FileWriter
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.scenario import Scenario, Tag, Location

__author__ = "Stefanie Manzinger, Moritz Klischat, Sebastian Maierhofer"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["Priority Program SPP 1835 Cooperative Interacting Automobiles"]
__version__ = "2022.1"
__maintainer__ = "Sebastian Maierhofer"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Released"


class ProtobufFileWriter(FileWriter):

    def __init__(self, scenario: Scenario, planning_problem_set: PlanningProblemSet, author: str = None,
                 affiliation: str = None, source: str = None, tags: Set[Tag] = None, location: Location = None,
                 decimal_precision: int = 4):
        super().__init__(scenario, planning_problem_set, author, affiliation, source, tags, location, decimal_precision)
