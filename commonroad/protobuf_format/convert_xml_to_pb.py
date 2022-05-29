import os
from os.path import isfile, join, isdir

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.file_writer import FileFormat, CommonRoadFileWriter
from commonroad.common.writer.file_writer_interface import OverwriteExistingFile


def convert_xml_to_pb(src_file_path: str, dest_file_path: str, invalid_conversion_path: str = None):
    """
    Converts CommonRoad file from XML to protobuf format. Invalid converted files can be stored in a desired directory.

    :param src_file_path: Source path
    :param dest_file_path: Destination path
    :param invalid_conversion_path: Invalid conversion path
    :return:
    """
    assert os.path.exists(src_file_path), 'The path to source directory is not existent!'
    assert src_file_path.endswith('.xml'), 'No XML file!'
    assert dest_file_path.endswith('.pb'), 'No protobuf file!'
    assert invalid_conversion_path is None or os.path.exists(invalid_conversion_path), \
        'The path to invalid conversion directory is not existent!'

    scenario_xml, planning_problem_set_xml = CommonRoadFileReader(src_file_path, FileFormat.XML).open()

    CommonRoadFileWriter(scenario_xml, planning_problem_set_xml, file_format=FileFormat.PROTOBUF) \
        .write_to_file(dest_file_path, OverwriteExistingFile.ALWAYS)

    if invalid_conversion_path is not None:
        scenario_pb, planning_problem_set_pb = CommonRoadFileReader(dest_file_path, FileFormat.PROTOBUF).open()

        if not (scenario_xml == scenario_pb and planning_problem_set_xml == planning_problem_set_pb):
            path_splits = src_file_path.split('/')
            if path_splits:
                name = path_splits[-1]
            else:
                return

            invalid_conversion_file_path = invalid_conversion_path + "/" + name

            CommonRoadFileWriter(scenario_xml, planning_problem_set_xml, file_format=FileFormat.XML) \
                .write_to_file(invalid_conversion_file_path, OverwriteExistingFile.ALWAYS)

    print("Conversion of " + str(scenario_xml.scenario_id))


def convert_xml_to_pb_in_dirs(src_dir_path: str, dest_dir_path: str, invalid_conversion_path: str = None):
    """
    Converts all CommonRoad files in directory and its subdirectories from XML to protobuf format.
    The hierarchical file structure is copied to the destination directory.

    :param src_dir_path: Source directory
    :param dest_dir_path: Destination directory
    :param invalid_conversion_path: Invalid conversion path
    :return:
    """
    assert os.path.exists(src_dir_path), 'The path to source directory is not existent!'
    assert os.path.exists(dest_dir_path), 'The path to destination directory is not existent!'
    assert invalid_conversion_path is None or os.path.exists(invalid_conversion_path), \
        'The path to invalid conversion directory is not existent!'

    file_names = list()
    dir_names = list()
    for name in os.listdir(src_dir_path):
        if isfile(join(src_dir_path, name)) and name.endswith('.xml'):
            file_names.append(name)
        elif isdir(join(src_dir_path, name)):
            dir_names.append(name)

    for name in file_names:
        src_file_path = join(src_dir_path, name)
        dest_file_path = join(dest_dir_path, name.replace('.xml', '.pb'))

        convert_xml_to_pb(src_file_path, dest_file_path, invalid_conversion_path)

    for name in dir_names:
        src_sub_dir_path = join(src_dir_path, name)
        dest_sub_dir_path = join(dest_dir_path, name)

        if not os.path.exists(dest_sub_dir_path):
            os.mkdir(dest_sub_dir_path)

        convert_xml_to_pb_in_dirs(src_sub_dir_path, dest_sub_dir_path, invalid_conversion_path)
