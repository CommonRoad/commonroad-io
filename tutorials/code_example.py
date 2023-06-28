# import functions to read xml file and visualize commonroad objects
import matplotlib.pyplot as plt

from commonroad.common.file_reader import CommonRoadFileReader, CommonRoadReadAll, CommonRoadMapFileReader, CommonRoadDynamicFileReader, CommonRoadScenarioFileReader
from commonroad.common.file_writer import CommonRoadFileWriter, FileFormat, OverwriteExistingFile

# generate path of the file to be opened
file_path_xml = "ZAM_Tutorial-1_1_T-1.xml"
file_path_map = "ZAM_Tutorial-1.pb"
file_path_dy = "ZAM_Tutorial-1_1_T-1.pb"
file_path_sc = "ZAM_Tutorial-1_1_T-1-SC.pb"

# read in the scenario and planning problem set
sc, pb = CommonRoadFileReader(file_path_xml).open()

fw = CommonRoadFileWriter(sc, pb)
fw.write_map_to_file(overwrite_existing_file=OverwriteExistingFile.ALWAYS)
fw.write_scenario_to_file(overwrite_existing_file=OverwriteExistingFile.ALWAYS)
fw.write_dynamic_to_file(overwrite_existing_file=OverwriteExistingFile.ALWAYS)

CommonRoadMapFileReader(file_path_map).open()
CommonRoadScenarioFileReader(file_path_sc).open()
CommonRoadDynamicFileReader(file_path_dy).open()

CommonRoadReadAll(file_path_dy).open()
CommonRoadReadAll(file_path_sc).open()
