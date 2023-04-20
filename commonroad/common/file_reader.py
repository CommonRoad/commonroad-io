from pathlib import Path
from typing import Tuple, Optional

from commonroad.common.util import FileFormat, Path_T
from commonroad.common.reader.file_reader_protobuf import ProtobufFileReader
from commonroad.common.reader.file_reader_xml import XMLFileReader
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.traffic_sign import *


class CommonRoadFileReader:
    """
    Reads CommonRoad files in XML or protobuf format. The corresponding stored scenario and planning problem set
    are created by the reader.
    """
    def __init__(self, filename: Path_T, file_format: Optional[FileFormat] = None):
        """
        Initializes the FileReader for CommonRoad files.

        :param filename: Name of file
        :param file_format: Format of file. If None, inferred from file suffix.
        :return:
        """
        self._file_reader = None

        if file_format is None:
            file_format = FileFormat(Path(filename).suffix)

        if file_format == FileFormat.XML:
            self._file_reader = XMLFileReader(filename)
        elif file_format == FileFormat.PROTOBUF:
            self._file_reader = ProtobufFileReader(filename)

    def open(self, lanelet_assignment: bool = False) -> Tuple[Scenario, PlanningProblemSet]:
        """
        Opens and loads CommonRoad scenario and planning problems from file.

        :param lanelet_assignment: Activates calculation of lanelets occupied by obstacles
        :return: Scenario and planning problems
        """
        return self._file_reader.open(lanelet_assignment)

    def open_lanelet_network(self) -> LaneletNetwork:
        """
        Opens and loads CommonRoad lanelet network from file.

        :return:
        """
        return self._file_reader.open_lanelet_network()
