from pathlib import Path

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.file_writer import (
    CommonRoadFileWriter,
    FileFormat,
    OverwriteExistingFile,
)

for path in list(Path(__file__).parent.glob("*.xml")):
    file_name_new = Path(str(path).replace("xml", "pb"))
    if file_name_new.exists():
        scenario, planning_problem_set = CommonRoadFileReader(path).open(lanelet_assignment=False)
        file_writer = CommonRoadFileWriter(
            scenario, planning_problem_set, file_format=FileFormat.PROTOBUF
        )
        file_writer.write_to_file(
            str(file_name_new), overwrite_existing_file=OverwriteExistingFile.ALWAYS
        )
