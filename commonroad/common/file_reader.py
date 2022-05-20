from commonroad.common.file_writer import FileFormat
from commonroad.common.reader.file_reader_protobuf import ProtobufFileReader
from commonroad.common.reader.file_reader_xml import XMLFileReader
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.traffic_sign import *

__author__ = "Stefanie Manzinger, Sebastian Maierhofer"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["Priority Program SPP 1835 Cooperative Interacting Automobiles", "CAR@TUM"]
__version__ = "2022.1"
__maintainer__ = "Stefanie Manzinger, Sebastian Maierhofer"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Released"


class CommonRoadFileReader:
    """ Class which reads CommonRoad XML-files. The XML-files are composed of
    (1) a formal representation of the road network,
    (2) static and dynamic obstacles,
    (3) the planning problem of the ego vehicle(s). """
    def __init__(self, filename: str, file_format: FileFormat = FileFormat.XML):
        self._file_reader = None
        if file_format == FileFormat.XML:
            self._file_reader = XMLFileReader(filename)
        elif file_format == FileFormat.PROTOBUF:
            self._file_reader = ProtobufFileReader(filename)

    def open(self, lanelet_assignment: bool = False) -> Tuple[Scenario, PlanningProblemSet]:
        return self._file_reader.open(lanelet_assignment)

    def open_lanelet_network(self) -> LaneletNetwork:
        return self._file_reader.open_lanelet_network()
