# Helper script to create protobuf files from old xml test files

import os

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile

base_path = "xml"
xml_files_2020a = [
    "USA_Peach-4_8_T-1",
    "FRA_Anglet-1_1_T-1",
    "DEU_Starnberg-1_1_T-1",
    "ARG_Carcarana-4_5_T-1",
    "ZAM_TestReadingAll-1_1_T-1",
]
xml_files_2018b = [
    "ZAM_TestReadingStdState-1_1_T-1",
    "ZAM_TestReadingStState-1_1_T-1",
    "ZAM_TestReadingPmState-1_1_T-1",
    "ZAM_TestReadingKsState-1_1_T-1",
    "ZAM_TestReadingCustomState-1_1_T-1",
]
output_path = "protobuf"

for file in xml_files_2020a:
    sc, pb = CommonRoadFileReader(os.path.join(base_path, "2020a/" + file + ".xml")).open()
    fw = CommonRoadFileWriter(sc, pb)
    fw.write_map_to_file(
        os.path.join(output_path, sc.lanelet_network.meta_information.complete_map_name + ".pb"),
        overwrite_existing_file=OverwriteExistingFile.ALWAYS,
    )
    fw.write_scenario_to_file(
        os.path.join(output_path, file + "-SC.pb"), overwrite_existing_file=OverwriteExistingFile.ALWAYS
    )
    fw.write_dynamic_to_file(
        os.path.join(output_path, file + ".pb"), overwrite_existing_file=OverwriteExistingFile.ALWAYS
    )

for file in xml_files_2018b:
    sc, pb = CommonRoadFileReader(os.path.join(base_path, "2018b/" + file + ".xml")).open()
    fw = CommonRoadFileWriter(sc, pb)
    fw.write_map_to_file(
        os.path.join(output_path, sc.lanelet_network.meta_information.complete_map_name + ".pb"),
        overwrite_existing_file=OverwriteExistingFile.ALWAYS,
    )
    fw.write_scenario_to_file(
        os.path.join(output_path, file + "-SC.pb"), overwrite_existing_file=OverwriteExistingFile.ALWAYS
    )
    fw.write_dynamic_to_file(
        os.path.join(output_path, file + ".pb"), overwrite_existing_file=OverwriteExistingFile.ALWAYS
    )
