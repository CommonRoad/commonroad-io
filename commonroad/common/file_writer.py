from typing import Union, Set

from commonroad.common.util import FileFormat
from commonroad.common.writer.file_writer_interface import OverwriteExistingFile
from commonroad.common.writer.file_writer_protobuf import ProtobufFileWriter
from commonroad.common.writer.file_writer_xml import XMLFileWriter
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.scenario import Scenario, Tag, Location


class CommonRoadFileWriter:
    """
    Writes CommonRoad files in XML or protobuf format. The corresponding scenario and planning problem set
    are stored by the writer.
    """

    def __init__(self, scenario: Scenario, planning_problem_set: PlanningProblemSet, author: str = None,
                 affiliation: str = None, source: str = None, tags: Set[Tag] = None, location: Location = None,
                 decimal_precision: int = 4, file_format: FileFormat = FileFormat.XML):
        """
        Initializes the FileWriter for CommonRoad files.

        :param scenario: Scenario
        :param planning_problem_set: Planning problems
        :param author: Name of author
        :param affiliation: Affiliation of author
        :param source: Source of dataset, e.g., database, handcrafted, etc.
        :param tags: Keywords describing the scenario
        :param decimal_precision: Number of decimal places used when writing float values
        :param file_format: Format of file
        :return:
        """
        self._file_format = file_format
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
        """
        Writes CommonRoad scenario and planning problems to file.

        :param filename: Name of file
        :param overwrite_existing_file: Overwriting mode
        :param check_validity: Validity check or not
        :return: Scenario and planning problems
        """
        self._file_writer.write_to_file(filename, overwrite_existing_file, check_validity)

    def write_scenario_to_file(self, filename: Union[str, None] = None,
                               overwrite_existing_file: OverwriteExistingFile = OverwriteExistingFile.ASK_USER_INPUT):
        """
        Writes CommonRoad scenario to file.

        :param filename: Name of file
        :param overwrite_existing_file: Overwriting mode
        :return: Scenario
        """
        self._file_writer.write_scenario_to_file(filename, overwrite_existing_file)

    @staticmethod
    def check_validity_of_commonroad_file(commonroad_str: Union[str, bytes], file_format: FileFormat = FileFormat.XML) \
            -> bool:
        """
        Checks validity of CommonRoad scenario and planning problem stored in XML or protobuf format.

        :param commonroad_str: Commonroad instance stored as string
        :param file_format: Format of file
        :return: Valid or not
        """
        if file_format == FileFormat.XML:
            is_valid = XMLFileWriter.check_validity_of_commonroad_file(commonroad_str)
        else:
            is_valid = ProtobufFileWriter.check_validity_of_commonroad_file(commonroad_str)

        return is_valid
