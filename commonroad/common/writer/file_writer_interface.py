import enum
import pathlib
from abc import ABC, abstractmethod
from typing import Set, Union, Optional

from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.scenario import Scenario, Tag


class DecimalPrecision:
    decimals = 4


precision = DecimalPrecision


class FileType(enum.Enum):
    """Specifies type of file to read/write"""

    MAP = 1
    SCENARIO = 2
    DYNAMIC = 3


class OverwriteExistingFile(enum.Enum):
    """Specifies whether an existing file will be overwritten or skipped"""

    ASK_USER_INPUT = 0
    ALWAYS = 1
    SKIP = 2


class FileWriter(ABC):
    """
    Interface for writing CommonRoad files in a specific format.
    """

    def __init__(self, scenario: Scenario, planning_problem_set: Optional[PlanningProblemSet],
                 author: Optional[str] = None,
                 affiliation: Optional[str] = None, source: Optional[str] = None, tags: Optional[Set[Tag]] = None,
                 decimal_precision: int = 4):
        assert not (author is None and scenario.file_information.author is None)
        assert not (affiliation is None and scenario.file_information.affiliation is None)
        assert not (source is None and scenario.file_information.source is None)
        assert not (tags is None and scenario.tags is None)

        self.scenario: Scenario = scenario
        self.planning_problem_set: Optional[PlanningProblemSet] = planning_problem_set
        self.author = author if author is not None else scenario.file_information.author
        self.affiliation = affiliation if affiliation is not None else scenario.file_information.affiliation
        self.source = source if source is not None else scenario.file_information.source
        self.tags = tags if tags is not None else scenario.tags

        precision.decimals = decimal_precision

    @property
    def author(self):
        return self._author

    @author.setter
    def author(self, author):
        assert isinstance(author, str), '<CommonRoadFileWriter/author> author must be a string, ' \
                                        'but has type {}'.format(type(author))
        self._author = author

    @property
    def affiliation(self):
        return self._affiliation

    @affiliation.setter
    def affiliation(self, affiliation):
        assert isinstance(affiliation, str), '<CommonRoadFileWriter/affiliation> affiliation must be a string, ' \
                                             'but has type {}'.format(type(affiliation))
        self._affiliation = affiliation

    @property
    def source(self):
        return self._source

    @source.setter
    def source(self, source):
        assert isinstance(source, str), '<CommonRoadFileWriter/source> source must be a string, ' \
                                        'but has type {}'.format(type(source))
        self._source = source

    @property
    def tags(self):
        return self._tags

    @tags.setter
    def tags(self, tags):
        for tag in tags:
            assert isinstance(tag, Tag), '<CommonRoadFileWriter/tags> tag must ' \
                                         'be a enum of type Tag, but has type {}'.format(type(tag))
        self._tags = tags

    @abstractmethod
    def _write_header(self):
        pass

    @abstractmethod
    def _add_all_objects_from_scenario(self):
        pass

    #  method used for 2023 version
    @abstractmethod
    def _add_all_objects_from_scenario_to_map(self):
        pass

    #  method used for 2023 version
    @abstractmethod
    def _add_all_objects_from_scenario_to_dynamic(self):
        pass

    @abstractmethod
    def _add_all_planning_problems_from_planning_problem_set(self):
        pass

    @abstractmethod
    def write_to_file(self, filename: Union[str, None] = None,
                      overwrite_existing_file: OverwriteExistingFile = OverwriteExistingFile.ASK_USER_INPUT,
                      check_validity: bool = False):
        pass

    @abstractmethod
    def write_scenario_to_file(self, filename: Union[str, None] = None,
                               overwrite_existing_file: OverwriteExistingFile = OverwriteExistingFile.ASK_USER_INPUT):
        pass

    @staticmethod
    def check_validity_of_commonroad_file(commonroad_str: Union[str, bytes]) -> bool:
        pass

    @abstractmethod
    def _get_suffix(self) -> str:
        pass

    def _handle_file_path(self, filename: Union[str, None], overwrite_existing_file: OverwriteExistingFile,
                          file_type: FileType) -> str:
        if filename is None and file_type is FileType.DYNAMIC:
            filename = str(self.scenario.scenario_id) + self._get_suffix()
        if filename is None and file_type is FileType.SCENARIO:
            filename = str(self.scenario.scenario_id) + "-SC" + self._get_suffix()
        if filename is None and file_type is FileType.MAP:
            filename = str(self.scenario.lanelet_network.meta_information.complete_map_name) + self._get_suffix()

        if pathlib.Path(filename).is_file():
            if overwrite_existing_file is OverwriteExistingFile.ASK_USER_INPUT:
                overwrite = input('File {} already exists, replace old file (or else skip)? (y/n)'.format(filename))
            elif overwrite_existing_file is OverwriteExistingFile.SKIP:
                overwrite = 'n'
            else:
                overwrite = 'y'

            if overwrite == 'n':
                print('Writing of file {} skipped'.format(filename))
                return ""
            else:
                print('Replace file {}'.format(filename))

        return filename
