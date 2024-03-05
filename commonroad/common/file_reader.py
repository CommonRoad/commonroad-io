from typing import List, Optional, Tuple, Union

from commonroad.common.reader.dynamic_interface import DynamicInterface
from commonroad.common.reader.file_reader_protobuf import (
    ProtobufFileReaderDynamic,
    ProtobufFileReaderMap,
    ProtobufFileReaderScenario,
    TrajectoryPredictionFactory,
)
from commonroad.common.reader.file_reader_xml import XMLFileReader
from commonroad.common.reader.scenario_interface import ScenarioInterface
from commonroad.common.util import Path_T
from commonroad.planning.planning_problem import (
    CooperativePlanningProblem,
    PlanningProblemSet,
)
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.obstacle import EnvironmentObstacle
from commonroad.scenario.scenario import Scenario


class CommonRoadFileReader:
    """
    Reads CommonRoad files in XML (2020a) or protobuf format (2024).
    The corresponding stored scenario and planning problem set are created by the reader.
    """

    def __init__(
        self,
        filename_2020a: Path_T = None,
        filename_map: Path_T = None,
        filename_scenario: Path_T = None,
        filename_dynamic: Path_T = None,
    ):
        """
        Initializes the FileReader for CommonRoad files.
        The user can send 4 filenames (1 for 2020a and 3 for 2024 format)
        Depending on the format of the files, user can call different functions to read the respective files.

        :param filename_2020a: Path of the 2020a xml file
        :param filename_map: Path of the 2024 protobuf map file
        :param filename_scenario: Path of the 2024 protobuf scenario file
        :param filename_dynamic: Path of the 2024 protobuf dynamic file
        """

        self._file_reader = None

        self._filename_2020a = filename_2020a
        self._filename_map = filename_map
        self._filename_scenario = filename_scenario
        self._filename_dynamic = filename_dynamic

    @property
    def file_reader(
        self,
    ) -> Union[XMLFileReader, ProtobufFileReaderDynamic, ProtobufFileReaderMap, ProtobufFileReaderScenario]:
        """
        File reader that reads the file depending on its format.
        """
        return self._file_reader

    @file_reader.setter
    def file_reader(
        self,
        file_reader: Union[XMLFileReader, ProtobufFileReaderDynamic, ProtobufFileReaderScenario, ProtobufFileReaderMap],
    ):
        self._file_reader = file_reader

    @property
    def filename_2020a(self) -> Optional[Path_T]:
        """
        Path of the 2020a xml file path.
        """
        return self._filename_2020a

    @filename_2020a.setter
    def filename_2020a(self, filename_2020a: Optional[Path_T]):
        self._filename_2020a = filename_2020a

    @property
    def filename_map(self) -> Optional[Path_T]:
        """
        Path of the 2024 map file.
        """
        return self._filename_map

    @filename_map.setter
    def filename_map(self, filename_map: Optional[Path_T]):
        self._filename_map = filename_map

    @property
    def filename_scenario(self) -> Optional[Path_T]:
        """
        Path of the 2024 scenario file.
        """
        return self._filename_scenario

    @filename_scenario.setter
    def filename_scenario(self, filename_scenario: Optional[Path_T]):
        self._filename_scenario = filename_scenario

    @property
    def filename_dynamic(self) -> Optional[Path_T]:
        """
        Path of the 2024 dynamic file.
        """
        return self._filename_dynamic

    @filename_dynamic.setter
    def filename_dynamic(self, filename_dynamic: Optional[Path_T]):
        self._filename_dynamic = filename_dynamic

    # 2020a reader
    def open(self, lanelet_assignment: bool = False) -> Tuple[Scenario, PlanningProblemSet]:
        """
        Opens and loads CommonRoad scenario and planning problems from file.

        :param lanelet_assignment: Activates calculation of lanelets occupied by obstacles
        :return: Scenario and planning problems

        :return: Tuple consisted of a Scenario and a PlanningProblemSet
        """

        # this function only works with the 2020a xml files
        if self.filename_2020a is None:
            raise NameError("Filename of the 2020a xml file is missing")
        else:
            self.file_reader = XMLFileReader(self.filename_2020a)
            return self.file_reader.open(lanelet_assignment)

    # 2020a reader
    def open_lanelet_network(self) -> LaneletNetwork:
        """
        Opens and loads CommonRoad lanelet network from file.

        :return: LaneletNetwork
        """

        # this function only works with 2020a xml files
        if self.filename_2020a is None:
            raise NameError("Filename of the 2020a xml file is missing")
        else:
            self.file_reader = XMLFileReader(self.filename_2020a)
            return self.file_reader.open_lanelet_network()

    def open_scenario(self) -> ScenarioInterface:
        """
        Opens and loads CommonRoadScenario from the file.

        :return: ScenarioInterface
        """
        # this function only works with 2024 protobuf files

        if self.filename_scenario is None:
            raise NameError("Filename of the 2024 scenario file is missing")
        else:
            self.file_reader = ProtobufFileReaderScenario(self.filename_scenario)
            return self.file_reader.open()

    def open_dynamic(self) -> DynamicInterface:
        """
        Opens and loads CommonRoadDynamic from the file.

        :return: DynamicInterface
        """
        # this function only works with 2024 protobuf files

        if self.filename_dynamic is None:
            raise NameError("Filename of the 2024 dynamic file is missing")
        else:
            self.file_reader = ProtobufFileReaderDynamic(self.filename_dynamic)
            return self.file_reader.open()

    def open_map(self) -> Tuple[LaneletNetwork, List[EnvironmentObstacle]]:
        """
        Opens and loads CommonRoadMap from the file.

        :return: Tuple with LaneletNetwork and a list of Environment Obstacles
        """
        # this function only works with 2024 protobuf files

        if self.filename_map is None:
            raise NameError("Filename of the 2024 map file is missing")
        else:
            self.file_reader = ProtobufFileReaderMap(self.filename_map)
            return self.file_reader.open()

    def open_map_dynamic(self) -> Scenario:
        """
        Opens and combines CommonRoadMap and CommonRoadDynamic files.
        The user has to provide both map and dynamic filenames in order to call this function.

        :return: Scenario
        """

        # check for the map filename
        if self.filename_map is None:
            raise NameError("Filename of the 2024 map file is missing")

        # check for the dynamic filename
        if self.filename_dynamic is None:
            raise NameError("Filename of the 2024 dynamic file is missing")

        # this function only works with 2024 protobuf files
        road_network, env_obstacles = ProtobufFileReaderMap(filename=self.filename_map).open()
        dynamic = ProtobufFileReaderDynamic(filename=self.filename_dynamic).open()

        dynamic.environment_obstacles = env_obstacles

        return combine_map_dynamic(road_network, dynamic)

    def open_all(self) -> Tuple[Scenario, PlanningProblemSet, List[CooperativePlanningProblem]]:
        """
        Opens and combines CommonRoadMap, CommonRoadDynamic and CommonRoadScenario files.
        The user has to provide scenario, dynamic and map filenames in order to call this function.

        :return: Tuple of a Scenario, PlanningProblemSet and a list of CooperativePlanningProblems
        """
        # check for the map filename
        if self.filename_map is None:
            raise NameError("Filename of the 2024 map file is missing")

        # check for the dynamic filename
        if self.filename_dynamic is None:
            raise NameError("Filename of the 2024 dynamic file is missing")

        # check for the scenario filename
        if self.filename_scenario is None:
            raise NameError("Filename of the 2024 scenario file is missing")

        # this function only works with 2024 protobuf files
        road_network, environment_obstacles = ProtobufFileReaderMap(filename=self.filename_map).open()
        scenario = ProtobufFileReaderScenario(filename=self.filename_scenario).open()
        dynamic_pb = ProtobufFileReaderDynamic(filename=self.filename_dynamic).open()

        dynamic_pb.environment_obstacles = environment_obstacles

        return combine_preloaded_(road_network, dynamic_pb, scenario)


def combine_map_dynamic(
    road_network: LaneletNetwork, dynamic_interface: DynamicInterface, lanelet_assignment: bool = False
):
    """
    Combines map and dynamic into one scenario.

    :param road_network: Lanelet network that the user has read from the file
    :param dynamic_interface: Dynamic interface class that the user has read from the file
    :param lanelet_assignment: Boolean that is used to assign the lanelets to scenario attributes, i.e.
    static and dynamic obstacles.
    :return: Scenario containing the lanelet network from map, and obstacles, traffic light cycles and traffic sign
    values from the dynamic.
    """
    scenario_id = dynamic_interface.dynamic_meta_information.scenario_id
    dt = dynamic_interface.dynamic_meta_information.time_step_size
    scenario = Scenario(dt, scenario_id)

    # Adding additional meta information to the scenario from dynamic_interface.information
    scenario.author = dynamic_interface.dynamic_meta_information.file_information.author
    scenario.affiliation = dynamic_interface.dynamic_meta_information.file_information.affiliation
    scenario.source = dynamic_interface.dynamic_meta_information.file_information.source
    scenario.lanelet_network.location = road_network.location

    # copying attributes from map to scenario.lanelet_network
    scenario.add_objects(road_network)

    # Adding environment obstacles to the scenario
    for environment_obstacle in dynamic_interface.environment_obstacles:
        scenario.add_objects(environment_obstacle)

    # Adding phantom obstacles to the scenario
    for phantom_obstacle in dynamic_interface.phantom_obstacles:
        scenario.add_objects(phantom_obstacle)

    # Adding static obstacles to the scenario
    for static_obstacle in dynamic_interface.static_obstacles:
        # Appending lanelets to the static obstacle if lanelet_assignment is set to True
        if lanelet_assignment is True:
            rotated_shape = static_obstacle.obstacle_shape.rotate_translate_local(
                static_obstacle.initial_state.position, static_obstacle.initial_state.orientation
            )
            initial_shape_lanelet_ids = set(scenario.lanelet_network.find_lanelet_by_shape(rotated_shape))
            initial_center_lanelet_ids = set(
                scenario.lanelet_network.find_lanelet_by_position([static_obstacle.initial_state.position])[0]
            )
            for l_id in initial_shape_lanelet_ids:
                scenario.lanelet_network.find_lanelet_by_id(l_id).add_static_obstacle_to_lanelet(
                    obstacle_id=static_obstacle.obstacle_id
                )
            static_obstacle.initial_center_lanelet_ids = initial_center_lanelet_ids
            static_obstacle.initial_shape_lanelet_ids = initial_shape_lanelet_ids
        scenario.add_objects(static_obstacle)

    # Adding dynamic obstacles to the scenario
    for dynamic_obstacle in dynamic_interface.dynamic_obstacles:
        # Only append lanelets to the obstacle if the prediction is the trajectory prediction
        # and lanelet_assignment is set to True
        if dynamic_obstacle.prediction is TrajectoryPrediction and lanelet_assignment is True:
            rotated_shape = dynamic_obstacle.obstacle_shape.rotate_translate_local(
                dynamic_obstacle.initial_state.position, dynamic_obstacle.initial_state.orientation
            )
            initial_shape_lanelet_ids = set(scenario.lanelet_network.find_lanelet_by_shape(rotated_shape))
            initial_center_lanelet_ids = set(
                scenario.lanelet_network.find_lanelet_by_position([dynamic_obstacle.initial_state.position])[0]
            )
            for l_id in initial_shape_lanelet_ids:
                scenario.lanelet_network.find_lanelet_by_id(l_id).add_dynamic_obstacle_to_lanelet(
                    obstacle_id=dynamic_obstacle.obstacle_id, time_step=dynamic_obstacle.initial_state.time_step
                )
            dynamic_obstacle.initial_shape_lanelet_ids = initial_shape_lanelet_ids
            dynamic_obstacle.initial_center_lanelet_ids = initial_center_lanelet_ids
            # Now have to update the trajectory prediction
            shape_lanelet_assignment = TrajectoryPredictionFactory.find_obstacle_shape_lanelets(
                dynamic_obstacle.initial_state,
                dynamic_obstacle.prediction.trajectory.state_list,
                scenario.lanelet_network,
                dynamic_obstacle.obstacle_id,
                dynamic_obstacle.obstacle_shape,
            )
            center_lanelet_assignment = TrajectoryPredictionFactory.find_obstacle_center_lanelets(
                dynamic_obstacle.initial_state,
                dynamic_obstacle.prediction.trajectory.state_list,
                scenario.lanelet_network,
            )
            dynamic_obstacle.prediction.shape_lanelet_assignment = shape_lanelet_assignment
            dynamic_obstacle.prediction.center_lanelet_assignment = center_lanelet_assignment
        scenario.add_objects(dynamic_obstacle)

    # Adding traffic light cycles from dynamic to the corresponding traffic lights
    light_cycle_id_dict = dynamic_interface.light_cycle_id_dict
    for tl_id in light_cycle_id_dict.keys():
        traffic_light = scenario.lanelet_network.find_traffic_light_by_id(tl_id)
        traffic_light.traffic_light_cycle = light_cycle_id_dict[tl_id]

    # Adding traffic sign elements from dynamic to the corresponding traffic signs
    for traffic_sign_value in dynamic_interface.traffic_sign_value:
        traffic_sign_id = traffic_sign_value.traffic_sign_id
        traffic_sign = scenario.lanelet_network.find_traffic_sign_by_id(traffic_sign_id)
        traffic_sign.traffic_sign_elements = traffic_sign_value.traffic_sign_elements

    return scenario


def combine_preloaded_(
    road_network: LaneletNetwork,
    dynamic_interface: DynamicInterface,
    scenario_interface: ScenarioInterface,
    lanelet_assignment: bool = False,
) -> Tuple[Scenario, PlanningProblemSet, List[CooperativePlanningProblem]]:
    """
    Combines the new version scenario, map and dynamic classes loaded from their respective files into one
    scenario class.

    :param road_network: Lanelet network that the user has read from the file
    :param dynamic_interface: Dynamic interface that the user has read from the file
    :param scenario_interface: Scenario interface that the user has read from the file
    :param lanelet_assignment: Boolean that is used to assign the lanelets to scenario attributes, i.e.
    static and dynamic obstacles.
    :return: Tuple consisted of the Scenario (containing the lanelet network from map, and obstacles from
    the dynamic), PlanningProblemSet and a list of CooperativePlanningProblems
    """

    scenario = combine_map_dynamic(road_network, dynamic_interface, lanelet_assignment)

    # Update the scenario meta information as in this case we copy it from scenario_interface meta information,
    # instead of dynamic_interface meta information as when combining only map and dynamic
    scenario.scenario_id = scenario_interface.scenario_meta_information.scenario_id
    scenario.dt = scenario_interface.scenario_meta_information.time_step_size
    scenario.author = scenario_interface.scenario_meta_information.file_information.author
    scenario.affiliation = scenario_interface.scenario_meta_information.file_information.affiliation
    scenario.source = scenario_interface.scenario_meta_information.file_information.source
    scenario.file_information = scenario_interface.scenario_meta_information.file_information

    # Get planning problem set and cooperative planning problems from the scenario_interface
    planning_problem_set = PlanningProblemSet(scenario_interface.planning_problems)
    multi_agent = scenario_interface.cooperative_planning_problems

    return scenario, planning_problem_set, multi_agent
