from typing import Union, Set

from commonroad.common.writer.file_writer_interface import OverwriteExistingFile, FileFormat
from commonroad.common.writer.file_writer_protobuf import ProtobufFileWriter
from commonroad.common.writer.file_writer_xml import XMLFileWriter
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.scenario import Scenario, Tag, Location

__author__ = "Stefanie Manzinger, Moritz Klischat, Sebastian Maierhofer"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["Priority Program SPP 1835 Cooperative Interacting Automobiles"]
__version__ = "2022.1"
__maintainer__ = "Sebastian Maierhofer"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Released"


class CommonRoadFileWriter:

    def __init__(self, scenario: Scenario, planning_problem_set: PlanningProblemSet, author: str = None,
                 affiliation: str = None, source: str = None, tags: Set[Tag] = None, location: Location = None,
                 decimal_precision: int = 4, file_format: FileFormat = FileFormat.XML):
        """
        Initialize the FileWriter with a scenario and tags for the xml-header

        :param scenario: scenario that should be written later
        :param planning_problem_set: corresponding planning problem to the scenario
        :param author: author's name
        :param affiliation: affiliation of the author
        :param source: source of dataset (d.h. database, handcrafted, etc.)
        :param tags: list of keywords describing the scenario (e.g. road type(one-lane road, multilane),
                required maneuver etc., see commonroad.in.tum.de for full list))
        :param decimal_precision: number of decimal places used when writing float values
        :param file_format: Format of file
        """
        self._file_writer = None
        if file_format == FileFormat.XML:
            self._file_writer = XMLFileWriter(scenario, planning_problem_set, author, affiliation,
                                              source, tags, location, decimal_precision)
        elif file_format == FileFormat.PROTOBUF:
            self._file_writer = ProtobufFileWriter(scenario, planning_problem_set, author, affiliation,
                                                   source, tags, location, decimal_precision)

    def write_to_file(self, filename: Union[str, None] = None,
                      overwrite_existing_file: OverwriteExistingFile = OverwriteExistingFile.ASK_USER_INPUT,
                      check_validity: bool = False):
        self._file_writer.write_to_file(filename, overwrite_existing_file, check_validity)

    def write_scenario_to_file(self, filename: Union[str, None] = None,
                               overwrite_existing_file: OverwriteExistingFile = OverwriteExistingFile.ASK_USER_INPUT):
        self._file_writer.write_scenario_to_file(filename, overwrite_existing_file)

    def check_validity_of_commonroad_file(self, commonroad_str: str):
        self._file_writer.check_validity_of_commonroad_file(commonroad_str)


