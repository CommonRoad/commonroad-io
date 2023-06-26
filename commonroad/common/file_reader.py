from pathlib import Path
from typing import Tuple, Optional, List

from commonroad.common.util import FileFormat, Path_T
from commonroad.common.reader.file_reader_protobuf import ProtobufFileReaderScenario, \
    ProtobufFileReaderMap, ProtobufFileReaderDynamic, TrajectoryPredictionFactory
from commonroad.common.reader.file_reader_xml import XMLFileReader
from commonroad.planning.planning_problem import PlanningProblemSet, CooperativePlanningProblem
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.scenario import Scenario, ScenarioID
from commonroad.common.reader.dynamic_interface import DynamicInterface
from commonroad.common.reader.scenario_interface import ScenarioInterface


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
        """
        self._file_reader = None

        if file_format is None:
            file_format = FileFormat(Path(filename).suffix)

        if file_format == FileFormat.XML:
            self._file_reader = XMLFileReader(filename)

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
        """
        return self._file_reader.open_lanelet_network()


class CommonRoadDynamicFileReader:
    """
    Reads CommonRoadDynamic files in XML or protobuf format. The corresponding stored scenario is
    created by the reader.
    """
    def __init__(self, filename: Path_T, file_format: Optional[FileFormat] = None):
        """
        Initializes the FileReaderDynamic for CommonRoad files.

        :param filename: Name of file
        :param file_format: Format of file. If None, inferred from file suffix.
        """
        self._file_reader = None

        if file_format is None:
            file_format = FileFormat(Path(filename).suffix)

        if file_format == FileFormat.XML:
            self._file_reader = XMLFileReader(filename)
        elif file_format == FileFormat.PROTOBUF:
            self._file_reader = ProtobufFileReaderDynamic(filename)

    def open(self) -> DynamicInterface:
        """
        Opens and loads CommonRoadDynamic from the file.

        :return: Dynamic
        """
        return self._file_reader.open()


class CommonRoadScenarioFileReader:
    """
    Reads CommonRoadScenario files in XML or protobuf format. The corresponding stored scenario is
    created by the reader.
    """
    def __init__(self, filename: Path_T, file_format: Optional[FileFormat] = None):
        """
        Initializes the FileReaderScenario for CommonRoad files.

        :param filename: Name of file
        :param file_format: Format of file. If None, inferred from file suffix.
        """
        self._file_reader = None

        if file_format is None:
            file_format = FileFormat(Path(filename).suffix)

        if file_format == FileFormat.XML:
            self._file_reader = XMLFileReader(filename)
        elif file_format == FileFormat.PROTOBUF:
            self._file_reader = ProtobufFileReaderScenario(filename)

    def open(self) -> ScenarioInterface:
        """
        Opens and loads CommonRoadScenario from the file.

        :return: Scenario
        """
        return self._file_reader.open()


class CommonRoadMapFileReader:
    """
    Reads CommonRoadMap files in XML or protobuf format. The corresponding stored scenario is
    created by the reader.
    """
    def __init__(self, filename: Path_T, file_format: Optional[FileFormat] = None):
        """
        Initializes the FileReaderMap for CommonRoad files.

        :param filename: Name of file
        :param file_format: Format of file. If None, inferred from file suffix.
        """
        self._file_reader = None

        if file_format is None:
            file_format = FileFormat(Path(filename).suffix)

        if file_format == FileFormat.XML:
            self._file_reader = XMLFileReader(filename)
        elif file_format == FileFormat.PROTOBUF:
            self._file_reader = ProtobufFileReaderMap(filename)

    def open(self) -> LaneletNetwork:
        """
        Opens and loads CommonRoadMap from the file.

        :return: Map
        """
        return self._file_reader.open()


class CommonRoadCombineMapDynamic:
    """
    Reads separate map and dynamic .py classes that the user has obtained from reading the corresponding files.
    Combines them into one scenario .py class.
    """
    def __init__(self, map: LaneletNetwork, dynamic_interface: DynamicInterface, lanelet_assignment: bool = False):
        """
        :param map: Lanelet network that the user has read from the file
        :param dynamic_interface: Dynamic interface class that the user has read from the file
        :param lanelet_assignment: Boolean that is used to assign the lanelets to scenario attributes, i.e.
        static and dynamic obstacles.
        """
        self._map = map
        self._dynamic_interface = dynamic_interface
        self._lanelet_assignment = lanelet_assignment

    def open(self) -> Scenario:
        """
        Combines the new version map and dynamic classes loaded from their respective files into one
        scenario class.

        :return: Scenario containing the lanelet network from map, and obstacles, traffic light cycles and traffic sign
        values from the dynamic.
        """
        scenarioID = ScenarioID.from_benchmark_id(self._dynamic_interface.dynamic_meta_information.benchmark_id,
                                                  self._dynamic_interface.dynamic_meta_information.commonroad_version)
        dt = self._dynamic_interface.dynamic_meta_information.time_step_size
        scenario = Scenario(dt, scenarioID)

        # Adding additional meta information to the scenario from dynamic_interface.information
        scenario.author = self._dynamic_interface.dynamic_meta_information.author
        scenario.affiliation = self._dynamic_interface.dynamic_meta_information.affiliation
        scenario.source = self._dynamic_interface.dynamic_meta_information.source
        scenario.location = self._map.location

        # copying attributes from map to scenario.lanelet_network
        scenario.add_objects(self._map)

        # Adding environment obstacles to the scenario
        for environment_obstacle in self._dynamic_interface.environment_obstacles:
            scenario.add_objects(environment_obstacle)

        # Adding phantom obstacles to the scenario
        for phantom_obstacle in self._dynamic_interface.phantom_obstacles:
            scenario.add_objects(phantom_obstacle)

        # Adding static obstacles to the scenario
        for static_obstacle in self._dynamic_interface.static_obstacles:
            # Appending lanelets to the static obstacle if lanelet_assignment is set to True
            if self._lanelet_assignment is True:
                rotated_shape = static_obstacle.obstacle_shape.rotate_translate_local\
                (static_obstacle.initial_state.position, static_obstacle.initial_state.orientation)
                initial_shape_lanelet_ids = set(scenario.lanelet_network.find_lanelet_by_shape(rotated_shape))
                initial_center_lanelet_ids = set(scenario.lanelet_network.find_lanelet_by_position
                                                 ([static_obstacle.initial_state.position])[0])
                for l_id in initial_shape_lanelet_ids:
                    scenario.lanelet_network.find_lanelet_by_id(l_id).add_static_obstacle_to_lanelet(
                            obstacle_id=static_obstacle.obstacle_id)
                static_obstacle.initial_center_lanelet_ids = initial_center_lanelet_ids
                static_obstacle.initial_shape_lanelet_ids = initial_shape_lanelet_ids
            scenario.add_objects(static_obstacle)

        # Adding dynamic obstacles to the scenario
        for dynamic_obstacle in self._dynamic_interface.dynamic_obstacles:
            # Only append lanelets to the obstacle if the prediction is the trajectory prediction
            # and lanelet_assignment is set to True
            if dynamic_obstacle.prediction is TrajectoryPrediction and self._lanelet_assignment is True:
                rotated_shape = dynamic_obstacle.obstacle_shape.rotate_translate_local\
                (dynamic_obstacle.initial_state.position, dynamic_obstacle.initial_state.orientation)
                initial_shape_lanelet_ids = set(scenario.lanelet_network.find_lanelet_by_shape(rotated_shape))
                initial_center_lanelet_ids = set(scenario.lanelet_network.find_lanelet_by_position
                                                 ([dynamic_obstacle.initial_state.position])[0])
                for l_id in initial_shape_lanelet_ids:
                    scenario.lanelet_network.find_lanelet_by_id(l_id).add_dynamic_obstacle_to_lanelet\
                    (obstacle_id=dynamic_obstacle.obstacle_id,
                     time_step=dynamic_obstacle.initial_state.time_step)
                dynamic_obstacle.initial_shape_lanelet_ids = initial_shape_lanelet_ids
                dynamic_obstacle.initial_center_lanelet_ids = initial_center_lanelet_ids
                # Now have to update the trajectory prediction
                shape_lanelet_assignment = \
                    TrajectoryPredictionFactory.find_obstacle_shape_lanelets\
                    (dynamic_obstacle.initial_state, dynamic_obstacle.prediction.trajectory.state_list,
                     scenario.lanelet_network, dynamic_obstacle.obstacle_id, dynamic_obstacle.obstacle_shape)
                center_lanelet_assignment = \
                    TrajectoryPredictionFactory.find_obstacle_center_lanelets\
                    (dynamic_obstacle.initial_state, dynamic_obstacle.prediction.trajectory.state_list,
                        scenario.lanelet_network)
                dynamic_obstacle.prediction.shape_lanelet_assignment = shape_lanelet_assignment
                dynamic_obstacle.prediction.center_lanelet_assignment = center_lanelet_assignment
            scenario.add_objects(dynamic_obstacle)

        # Adding traffic light cycles from dynamic to the corresponding traffic lights
        light_cycle_id_dict = self._dynamic_interface.light_cycle_id_dict
        for tl_id in light_cycle_id_dict.keys():
            traffic_light = scenario.lanelet_network.find_traffic_light_by_id(tl_id)
            traffic_light.traffic_light_cycle = light_cycle_id_dict[tl_id]

        # Adding traffic sign elements from dynamic to the corresponding traffic signs
        for traffic_sign_value in self._dynamic_interface.traffic_sign_value:
            traffic_sign_id = traffic_sign_value.traffic_sign_id
            traffic_sign = scenario.lanelet_network.find_traffic_sign_by_id(traffic_sign_id)
            traffic_sign.traffic_sign_elements = traffic_sign_value.traffic_sign_elements

        return scenario


class CommonRoadCombineAll:
    """
    Reads all 3 of the separate .py classes that the user has obtained from reading the corresponding files.
    Combines them into one scenario .py class.
    """
    def __init__(self, map: LaneletNetwork, dynamic_interface: DynamicInterface,
                 scenario_interface: ScenarioInterface, lanelet_assignment: bool = False):
        """
        :param map: Lanelet network that the user has read from the file
        :param dynamic_interface: Dynamic interface that the user has read from the file
        :param scenario_interface: Scenario interface that the user has read from the file
        :param lanelet_assignment: Boolean that is used to assign the lanelets to scenario attributes, i.e.
        static and dynamic obstacles.
        """
        self._scenario_interface = scenario_interface
        self._map = map
        self._dynamic_interface = dynamic_interface
        self._lanelet_assignment = lanelet_assignment
        
    def open(self) -> Tuple[Scenario, PlanningProblemSet, List[CooperativePlanningProblem]]:
        """
        Combines the new version scenario, map and dynamic classes loaded from their respective files into one
        scenario class.

        :return: Tuple consisted of the Scenario (containing the lanelet network from map, and obstacles from
        the dynamic), PlanningProblemSet and a list of CooperativePlanningProblems
        """

        scenario = CommonRoadCombineMapDynamic(self._map, self._dynamic_interface, self._lanelet_assignment).open()

        # Update the scenario meta information as in this case we copy it from scenario_interface meta information,
        # instead of dynamic_interface meta information as when combining only map and dynamic
        scenarioID = ScenarioID.from_benchmark_id(self._scenario_interface.scenario_meta_information.benchmark_id,
                                                  self._scenario_interface.scenario_meta_information.commonroad_version)
        scenario.scenario_id = scenarioID
        scenario.dt = self._scenario_interface.scenario_meta_information.time_step_size
        scenario.author = self._scenario_interface.scenario_meta_information.author
        scenario.affiliation = self._scenario_interface.scenario_meta_information.affiliation
        scenario.source = self._scenario_interface.scenario_meta_information.source
        scenario.location = self._map.location

        # Get planning problem set and cooperative planning problems from the scenario_interface
        planning_problem_set = PlanningProblemSet(self._scenario_interface.planning_problems)
        multi_agent = self._scenario_interface.cooperative_planning_problems

        return scenario, planning_problem_set, multi_agent
