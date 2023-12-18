import datetime
import logging
import re
from typing import List, Optional, Set, Union

import numpy as np
from google.protobuf.message import DecodeError

from commonroad.common.common_lanelet import LineMarking, StopLine
from commonroad.common.common_scenario import (
    Environment,
    GeoTransformation,
    Location,
    ScenarioID,
)
from commonroad.common.protobuf.common import (
    scenario_meta_information_pb2,
    state_pb2,
    traffic_light_state_pb2,
    traffic_sign_element_pb2,
    util_pb2,
)
from commonroad.common.protobuf.dynamic import (
    commonroad_dynamic_pb2,
    dynamic_obstacle_pb2,
    environment_obstacle_pb2,
    environment_pb2,
    obstacle_pb2,
    phantom_obstacle_pb2,
    static_obstacle_pb2,
    traffic_light_cycle_pb2,
    traffic_sign_value_pb2,
)
from commonroad.common.protobuf.map import (
    area_pb2,
    commonroad_map_pb2,
    intersection_pb2,
    lanelet_pb2,
    location_pb2,
    traffic_light_pb2,
    traffic_sign_pb2,
)
from commonroad.common.protobuf.scenario import (
    commonroad_scenario_pb2,
    planning_problem_pb2,
    scenario_tags_pb2,
)
from commonroad.common.util import FileFormat, Interval, Time
from commonroad.common.writer.file_writer_interface import (
    FileType,
    FileWriter,
    OverwriteExistingFile,
)
from commonroad.geometry.shape import Circle, Polygon, Rectangle, Shape, ShapeGroup
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from commonroad.prediction.prediction import (
    Occupancy,
    SetBasedPrediction,
    TrajectoryPrediction,
)
from commonroad.scenario.area import Area, AreaBorder
from commonroad.scenario.intersection import (
    CrossingGroup,
    IncomingGroup,
    Intersection,
    OutgoingGroup,
)
from commonroad.scenario.lanelet import Lanelet
from commonroad.scenario.obstacle import (
    DynamicObstacle,
    EnvironmentObstacle,
    PhantomObstacle,
    SignalState,
    StaticObstacle,
)
from commonroad.scenario.scenario import Scenario, Tag
from commonroad.scenario.state import State
from commonroad.scenario.traffic_light import (
    TrafficLight,
    TrafficLightCycle,
    TrafficLightCycleElement,
)
from commonroad.scenario.traffic_sign import (
    TrafficSign,
    TrafficSignElement,
    TrafficSignValue,
)
from commonroad.scenario.trajectory import Trajectory

logger = logging.getLogger(__name__)


class ProtobufFileWriter(FileWriter):
    """
    Writes CommonRoad files in protobuf format.
    """

    #  abstract methods used in previous format xml
    def _write_header(self):
        pass

    #  abstract methods used in previous format xml
    def _add_all_objects_from_scenario(self):
        pass

    #  abstract methods used in previous format xml
    def write_to_file(
        self,
        filename: Union[str, None] = None,
        overwrite_existing_file: OverwriteExistingFile = OverwriteExistingFile.ASK_USER_INPUT,
        check_validity: bool = False,
    ):
        pass

    def __init__(
        self,
        scenario: Scenario,
        planning_problem_set: Optional[PlanningProblemSet] = None,
        author: Optional[str] = None,
        affiliation: Optional[str] = None,
        source: Optional[str] = None,
        tags: Optional[Set[Tag]] = None,
        decimal_precision: int = 4,
    ):
        super().__init__(scenario, planning_problem_set, author, affiliation, source, tags, decimal_precision)

        self._commonroad_dynamic_msg = commonroad_dynamic_pb2.CommonRoadDynamic()
        self._commonroad_map_msg = commonroad_map_pb2.CommonRoadMap()
        self._commonroad_scenario_msg = commonroad_scenario_pb2.CommonRoadScenario()

    def _write_header_map(self):
        """
        Stores all information about 2023 Map as protobuf message.
        """
        map_info_msg = MapInformationMessage.create_message(
            self.scenario.scenario_id,
            self.scenario.lanelet_network.meta_information.file_information.date,
            self.scenario.lanelet_network.meta_information.file_information.author,
            self.scenario.lanelet_network.meta_information.file_information.affiliation,
            self.scenario.lanelet_network.meta_information.file_information.source,
            self.scenario.lanelet_network.meta_information.file_information.license_name,
            self.scenario.lanelet_network.meta_information.file_information.license_text,
        )

        self._commonroad_map_msg.map_meta_information.CopyFrom(map_info_msg)

    def _write_header_dynamic(self):
        """
        Stores all information about 2023 Dynamic as protobuf message.
        """
        dynamic_info_msg = DynamicInformationMessage.create_message(
            self.scenario.scenario_id,
            self.scenario.file_information.license_name,
            self.scenario.file_information.license_text,
            self.scenario.file_information.author,
            self.scenario.file_information.affiliation,
            self.scenario.file_information.source,
            self.scenario.dt,
        )

        self._commonroad_dynamic_msg.dynamic_meta_information.CopyFrom(dynamic_info_msg)

    def _write_header_scenario(self):
        """
        Stores all information about 2023 Scenario as protobuf message.
        """
        scenario_info_msg = ScenarioInformationMessage.create_message(
            self.scenario.scenario_id,
            self.scenario.file_information.license_name,
            self.scenario.file_information.license_text,
            self._author,
            self._affiliation,
            self._source,
            self.scenario.dt,
        )

        self._commonroad_scenario_msg.scenario_meta_information.CopyFrom(scenario_info_msg)

    def _add_all_objects_from_scenario_to_map(self):
        """
        Stores all scenario objects that correspond to the map as protobuf message.
        """
        if self.scenario.lanelet_network.location is not None:
            location_msg = LocationMessage.create_message(self.scenario.lanelet_network.location)
        else:
            location_msg = LocationMessage.create_message(Location())
        self._commonroad_map_msg.location.CopyFrom(location_msg)

        for lanelet in self.scenario.lanelet_network.lanelets:
            lanelet_msg = LaneletMessage.create_message(lanelet)
            left_bound_msg = BoundMessage.create_message(
                lanelet.left_bound, lanelet.left_vertices, lanelet.line_marking_left_vertices
            )
            right_bound_msg = BoundMessage.create_message(
                lanelet.right_bound, lanelet.right_vertices, lanelet.line_marking_right_vertices
            )

            if lanelet.stop_line is not None:
                if lanelet.stop_line_id is None:
                    # We assign the lanelet id as the stop_line_id, as there the stop_line does not contain its own
                    # id as the attribute in the xml format
                    lanelet.stop_line_id = lanelet.lanelet_id
                stop_line_msg = StopLineMessage.create_message(lanelet.stop_line_id, lanelet.stop_line)
                self._commonroad_map_msg.stop_lines.append(stop_line_msg)

            self._commonroad_map_msg.lanelets.append(lanelet_msg)
            self._commonroad_map_msg.boundaries.append(left_bound_msg)
            self._commonroad_map_msg.boundaries.append(right_bound_msg)

        for traffic_sign in self.scenario.lanelet_network.traffic_signs:
            traffic_sign_msg = TrafficSignMessage.create_message(traffic_sign)
            self._commonroad_map_msg.traffic_signs.append(traffic_sign_msg)

        for traffic_light in self.scenario.lanelet_network.traffic_lights:
            traffic_light_msg = TrafficLightMessage.create_message(traffic_light)
            self._commonroad_map_msg.traffic_lights.append(traffic_light_msg)

        for intersection in self.scenario.lanelet_network.intersections:
            intersection_msg = IntersectionMessage.create_message(intersection)
            self._commonroad_map_msg.intersections.append(intersection_msg)

    def _add_all_objects_from_scenario_to_dynamic(self):
        """
        Stores all scenario objects that correspond to the dynamic as protobuf message.
        """
        for static_obstacle in self.scenario.static_obstacles:
            static_obstacle_msg = StaticObstacleMessage.create_message(static_obstacle)
            self._commonroad_dynamic_msg.static_obstacles.append(static_obstacle_msg)

        for dynamic_obstacle in self.scenario.dynamic_obstacles:
            dynamic_obstacle_msg = DynamicObstacleMessage.create_message(dynamic_obstacle)
            self._commonroad_dynamic_msg.dynamic_obstacles.append(dynamic_obstacle_msg)

        for environment_obstacle in self.scenario.environment_obstacle:
            environment_obstacle_msg = EnvironmentObstacleMessage.create_message(environment_obstacle)
            self._commonroad_dynamic_msg.environment_obstacles.append(environment_obstacle_msg)

        for phantom_obstacle in self.scenario.phantom_obstacle:
            phantom_obstacle_msg = PhantomObstacleMessage.create_message(phantom_obstacle)
            self._commonroad_dynamic_msg.phantom_obstacles.append(phantom_obstacle_msg)

        for traffic_light in self.scenario.lanelet_network.traffic_lights:
            if traffic_light.traffic_light_cycle is not None:
                traffic_light_cycle_msg = TrafficLightCycleMessage.create_message(
                    traffic_light.traffic_light_cycle, traffic_light.traffic_light_id
                )
                self._commonroad_dynamic_msg.traffic_light_cycle.append(traffic_light_cycle_msg)

        for traffic_sign in self.scenario.lanelet_network.traffic_signs:
            if traffic_sign.traffic_sign_elements is not None:
                traffic_sign_value = TrafficSignValue(traffic_sign.traffic_sign_id, traffic_sign.traffic_sign_elements)
                traffic_sign_value_msg = TrafficSignValueMessage.create_message(traffic_sign_value)
                self._commonroad_dynamic_msg.traffic_sign_value.append(traffic_sign_value_msg)

        environment = self.scenario.environment if self.scenario.environment is not None else Environment()
        self._commonroad_dynamic_msg.environment.CopyFrom(EnvironmentMessage.create_message(environment))

    def _add_all_planning_problems_from_planning_problem_set(self):
        """
        Stores all planning problems in scenario as protobuf message.
        """
        for planning_problem in self.planning_problem_set.planning_problem_dict.values():
            planning_problem_msg = PlanningProblemMessage.create_message(planning_problem)
            # As mentioned in issue #45, planning problem stores scenario tags
            scenario_tags = ScenarioTagsMessage.create_message(list(self.scenario.tags))
            planning_problem_msg.scenario_tags.CopyFrom(scenario_tags)

            self._commonroad_scenario_msg.planning_problems.append(planning_problem_msg)

    def _serialize_write_msg(
        self, filename_map: str = None, filename_dynamic: str = None, filename_scenario: str = None
    ):
        """
        Serializes the message of type commonroad and writes into file.

        :param filename_map: Name of map file
        :param filename_dynamic: Name of dynamic file
        :param filename_scenario: Name of scenario file
        """
        if filename_map:
            with open(filename_map, "wb+") as f:
                f.write(self._commonroad_map_msg.SerializeToString())
        if filename_dynamic:
            with open(filename_dynamic, "wb+") as f:
                f.write(self._commonroad_dynamic_msg.SerializeToString())
        if filename_scenario:
            with open(filename_scenario, "wb+") as f:
                f.write(self._commonroad_scenario_msg.SerializeToString())

    def _get_suffix(self) -> str:
        return FileFormat.PROTOBUF.value

    def write_map_to_file(
        self,
        filename: Union[str, None] = None,
        overwrite_existing_file: OverwriteExistingFile = OverwriteExistingFile.ASK_USER_INPUT,
        check_validity: bool = False,
    ):
        """
        Writes message of type commonroad_map into file.

        :param filename: Name of file
        :param overwrite_existing_file: Mode of writing
        :param check_validity: Validity checking before writing (only done for xml)
        """
        filename = self._handle_file_path(filename, overwrite_existing_file, FileType.MAP)
        if not filename:
            return

        self._commonroad_map_msg = commonroad_map_pb2.CommonRoadMap()
        self._write_header_map()
        self._add_all_objects_from_scenario_to_map()
        if check_validity:
            pass
        self._serialize_write_msg(filename)

    def write_dynamic_to_file(
        self,
        filename: Union[str, None] = None,
        overwrite_existing_file: OverwriteExistingFile = OverwriteExistingFile.ASK_USER_INPUT,
        check_validity: bool = False,
    ):
        """
        Writes message of type commonroad into file.

        :param filename: Name of file
        :param overwrite_existing_file: Mode of writing
        :param check_validity: Validity checking before writing
        """
        filename = self._handle_file_path(filename, overwrite_existing_file, FileType.DYNAMIC)
        if not filename:
            return

        self._commonroad_dynamic_msg = commonroad_dynamic_pb2.CommonRoadDynamic()

        self._write_header_dynamic()
        self._add_all_objects_from_scenario_to_dynamic()
        if check_validity:
            pass
        self._serialize_write_msg(None, filename)

    def write_scenario_to_file(
        self,
        filename: Union[str, None] = None,
        overwrite_existing_file: OverwriteExistingFile = OverwriteExistingFile.ASK_USER_INPUT,
    ):
        """
        Writes scenario as protobuf message into file.

        :param filename: Name of file
        :param overwrite_existing_file: Mode of writing
        """
        filename = self._handle_file_path(filename, overwrite_existing_file, FileType.SCENARIO)
        if not filename:
            return

        self._commonroad_scenario_msg = commonroad_scenario_pb2.CommonRoadScenario()

        self._write_header_scenario()
        self._add_all_planning_problems_from_planning_problem_set()
        self._commonroad_scenario_msg.map_id = self.scenario.lanelet_network.meta_information.complete_map_name
        self._commonroad_scenario_msg.dynamic_id = str(self.scenario.scenario_id)

        self._serialize_write_msg(None, None, filename)

    @staticmethod
    def check_validity_of_commonroad_file(
        commonroad_map_str: Union[str, bytes] = None,
        commonroad_dynamic_str: Union[str, bytes] = None,
        commonroad_scenario_str: Union[str, bytes] = None,
    ) -> bool:
        """
        Checks validity of CommonRoad messages/files.

        :param commonroad_map_str: Commonroad map instance in form of binary string
        :param commonroad_dynamic_str: Commonroad dynamic instance in form of binary string
        :param commonroad_scenario_str: Commonroad scenario instance in form of binary string
        :return: Valid or not
        """
        try:
            if commonroad_map_str is not None:
                commonroad_map_msg = commonroad_map_pb2.CommonRoadMap()
                commonroad_map_msg.ParseFromString(commonroad_map_str)
            if commonroad_dynamic_str is not None:
                commonroad_dynamic_msg = commonroad_dynamic_pb2.CommonRoadDynamic()
                commonroad_dynamic_msg.ParseFromString(commonroad_dynamic_str)
            if commonroad_scenario_str is not None:
                commonroad_scenario_msg = commonroad_scenario_pb2.CommonRoadScenario()
                commonroad_scenario_msg.ParseFromString(commonroad_scenario_str)
            return True
        except DecodeError:
            return False


class ScenarioIDMessage:
    @classmethod
    def create_message(cls, scenario_id: ScenarioID) -> scenario_meta_information_pb2.ScenarioID:
        scenario_id_msg = scenario_meta_information_pb2.ScenarioID()
        scenario_id_msg.cooperative = scenario_id.cooperative
        map_id = MapIDMessage.create_message(scenario_id)
        scenario_id_msg.map_id.CopyFrom(map_id)
        scenario_id_msg.configuration_id = scenario_id.configuration_id
        if scenario_id.obstacle_behavior is not None:
            scenario_id_msg.obstacle_behavior = scenario_id.obstacle_behavior
        if scenario_id.prediction_id is not None:
            scenario_id_msg.prediction_id = scenario_id.prediction_id
        scenario_id_msg.scenario_version = scenario_id.scenario_version

        return scenario_id_msg


class MapIDMessage:
    @classmethod
    def create_message(cls, scenario_id: ScenarioID) -> scenario_meta_information_pb2.MapID:
        map_id_msg = scenario_meta_information_pb2.MapID()
        map_id_msg.country_id = scenario_id.country_id
        map_id_msg.map_name = scenario_id.map_name
        map_id_msg.map_id = scenario_id.map_id

        return map_id_msg


class FileInformationMessage:
    @classmethod
    def create_message(
        cls, date: Time, author: str, affiliation: str, source: str, license_name: str, license_text: str
    ) -> scenario_meta_information_pb2.FileInformation:
        file_information_msg = scenario_meta_information_pb2.FileInformation()
        time_stamp_msg = TimeStampMessage.create_message(
            datetime.datetime(date.year, date.month, date.day, date.hours, date.minutes)
        )
        file_information_msg.date.CopyFrom(time_stamp_msg)
        file_information_msg.author = author
        file_information_msg.affiliation = affiliation
        file_information_msg.source = source
        file_information_msg.license_name = license_name
        file_information_msg.license_text = license_text

        return file_information_msg


class MapInformationMessage:
    @classmethod
    def create_message(
        cls,
        scenario_id: ScenarioID,
        date: Time,
        author: str,
        affiliation: str,
        source: str,
        license_name: str,
        license_text: str,
    ) -> commonroad_map_pb2.MapInformation:
        map_information_msg = commonroad_map_pb2.MapInformation()
        map_id_msg = MapIDMessage.create_message(scenario_id)
        file_information_msg = FileInformationMessage.create_message(
            date, author, affiliation, source, license_name, license_text
        )
        map_information_msg.map_id.CopyFrom(map_id_msg)
        map_information_msg.file_information.CopyFrom(file_information_msg)

        return map_information_msg


class ScenarioInformationMessage:
    @classmethod
    def create_message(
        cls,
        scenario_id: ScenarioID,
        license_name: str,
        license_text: str,
        author: str,
        affiliation: str,
        source: str,
        time_step_size: float,
    ) -> scenario_meta_information_pb2.ScenarioMetaInformation:
        scenario_information_msg = scenario_meta_information_pb2.ScenarioMetaInformation()
        scenario_id_msg = ScenarioIDMessage.create_message(scenario_id)
        scenario_information_msg.benchmark_id.CopyFrom(scenario_id_msg)
        time = datetime.datetime.now()
        file_information_msg = FileInformationMessage.create_message(
            Time(time.hour, time.minute, time.day, time.month, time.year),
            author,
            affiliation,
            source,
            license_name,
            license_text,
        )
        scenario_information_msg.file_information.CopyFrom(file_information_msg)
        scenario_information_msg.time_step_size = time_step_size

        return scenario_information_msg


class DynamicInformationMessage:
    @classmethod
    def create_message(
        cls,
        scenario_id: ScenarioID,
        license_name: str,
        license_text: str,
        author: str,
        affiliation: str,
        source: str,
        time_step_size: float,
    ) -> scenario_meta_information_pb2.ScenarioMetaInformation:
        dynamic_information_msg = scenario_meta_information_pb2.ScenarioMetaInformation()
        scenario_id_msg = ScenarioIDMessage.create_message(scenario_id)
        dynamic_information_msg.benchmark_id.CopyFrom(scenario_id_msg)
        time = datetime.datetime.today()
        file_information_msg = FileInformationMessage.create_message(
            Time(time.hour, time.minute, time.day, time.month, time.year),
            author,
            affiliation,
            source,
            license_name,
            license_text,
        )
        dynamic_information_msg.file_information.CopyFrom(file_information_msg)
        dynamic_information_msg.time_step_size = time_step_size

        return dynamic_information_msg


class ScenarioTagsMessage:
    @classmethod
    def create_message(cls, tags: List[Tag]) -> scenario_tags_pb2.ScenarioTags:
        scenario_tags_msg = scenario_tags_pb2.ScenarioTags()

        for tag in tags:
            scenario_tags_msg.tags.append(scenario_tags_pb2.TagEnum.Tag.Value(tag.name))

        return scenario_tags_msg


class LocationMessage:
    @classmethod
    def create_message(cls, location: Location) -> location_pb2.Location:
        location_msg = location_pb2.Location()

        location_msg.geo_name_id = location.geo_name_id
        location_msg.gps_latitude = location.gps_latitude
        location_msg.gps_longitude = location.gps_longitude
        if location.geo_transformation is not None:
            geo_transformation_msg = GeoTransformationMessage.create_message(location.geo_transformation)
            location_msg.geo_transformation.CopyFrom(geo_transformation_msg)

        return location_msg


class GeoTransformationMessage:
    @classmethod
    def create_message(cls, geo_transformation: GeoTransformation) -> location_pb2.GeoTransformation:
        geo_transformation_msg = location_pb2.GeoTransformation()

        geo_transformation_msg.geo_reference = geo_transformation.geo_reference
        geo_transformation_msg.x_translation = geo_transformation.x_translation
        geo_transformation_msg.y_translation = geo_transformation.y_translation
        geo_transformation_msg.z_rotation = geo_transformation.z_rotation
        geo_transformation_msg.scaling = geo_transformation.scaling

        return geo_transformation_msg


class EnvironmentMessage:
    @classmethod
    def create_message(cls, environment: Environment) -> environment_pb2.Environment:
        environment_msg = environment_pb2.Environment()

        if environment.time is not None:
            time_stamp_msg = TimeStampMessage.create_message(environment.time)
            environment_msg.time.CopyFrom(time_stamp_msg)
        if environment.time_of_day is not None:
            environment_msg.time_of_day = environment_pb2.TimeOfDayEnum.TimeOfDay.Value(environment.time_of_day.name)
        if environment.weather is not None:
            environment_msg.weather = environment_pb2.WeatherEnum.Weather.Value(environment.weather.name)
        if environment.underground is not None:
            environment_msg.underground = environment_pb2.UndergroundEnum.Underground.Value(
                environment.underground.name
            )

        return environment_msg


class AreaBorderMessage:
    @classmethod
    def create_message(cls, area_border: AreaBorder) -> area_pb2.AreaBorder:
        area_border_msg = area_pb2.AreaBorder()
        area_border_msg.area_border_id = area_border.area_border_id
        area_border_msg.boundary = area_border.boundary
        area_border_msg.adjacent = area_border.adjacent
        area_border_msg.line_marking = lanelet_pb2.LineMarkingEnum.LineMarking.Value(area_border.line_marking.name)

        return area_border_msg


class AreaMessage:
    @classmethod
    def create_message(cls, area: Area) -> area_pb2.Area:
        area_msg = area_pb2.Area()
        area_msg.area_id = area.area_id
        for border in area.border:
            border_msg = AreaBorderMessage.create_message(border)
            area_msg.border.append(border_msg)
        for area_type in area.area_types:
            area_msg.area_type.append(area_pb2.AreaTypeEnum.AreaType.Value(area_type.name))

        return area_msg


class LaneletMessage:
    @classmethod
    def create_message(cls, lanelet: Lanelet) -> lanelet_pb2.Lanelet:
        lanelet_msg = lanelet_pb2.Lanelet()

        lanelet_msg.lanelet_id = lanelet.lanelet_id

        lanelet_msg.left_bound = lanelet.left_bound
        lanelet_msg.right_bound = lanelet.right_bound

        for pre in lanelet.predecessor:
            lanelet_msg.predecessors.append(pre)

        for suc in lanelet.successor:
            lanelet_msg.successors.append(suc)

        if lanelet.adj_left is not None:
            lanelet_msg.adjacent_left = lanelet.adj_left

        if lanelet.adj_right is not None:
            lanelet_msg.adjacent_right = lanelet.adj_right

        if lanelet.adj_left_same_direction is not None:
            if lanelet.adj_left_same_direction:
                lanelet_msg.adjacent_left_opposite_dir = False
            else:
                lanelet_msg.adjacent_left_opposite_dir = True

        if lanelet.adj_right_same_direction is not None:
            if lanelet.adj_right_same_direction:
                lanelet_msg.adjacent_right_opposite_dir = False
            else:
                lanelet_msg.adjacent_right_opposite_dir = True

        if lanelet.stop_line is not None:
            lanelet.stop_line_id = lanelet.stop_line.stop_line_id
            lanelet_msg.stop_line = lanelet.stop_line_id

        for lanelet_type in lanelet.lanelet_type:
            lanelet_msg.lanelet_types.append(lanelet_pb2.LaneletTypeEnum.LaneletType.Value(lanelet_type.name))

        for user_one_way in lanelet.user_one_way:
            lanelet_msg.user_one_way.append(lanelet_pb2.RoadUserEnum.RoadUser.Value(user_one_way.name))

        for user_bidirectional in lanelet.user_bidirectional:
            lanelet_msg.user_bidirectional.append(lanelet_pb2.RoadUserEnum.RoadUser.Value(user_bidirectional.name))

        for ts_ref in lanelet.traffic_signs:
            lanelet_msg.traffic_sign_refs.append(ts_ref)

        for tl_ref in lanelet.traffic_lights:
            lanelet_msg.traffic_light_refs.append(tl_ref)

        for adj_area in lanelet.adjacent_areas:
            lanelet_msg.adjacent_area_refs.append(adj_area)

        return lanelet_msg


class BoundMessage:
    @classmethod
    def create_message(cls, boundary_id: int, vertices: np.ndarray, line_marking: LineMarking) -> lanelet_pb2.Bound:
        bound_msg = lanelet_pb2.Bound()

        bound_msg.boundary_id = boundary_id

        for vertex in vertices:
            point_msg = PointMessage.create_message(vertex)
            bound_msg.points.append(point_msg)

        bound_msg.line_marking = lanelet_pb2.LineMarkingEnum.LineMarking.Value(line_marking.name)

        return bound_msg


class StopLineMessage:
    @classmethod
    def create_message(cls, stop_line_id: int, stop_line: StopLine) -> lanelet_pb2.StopLine:
        stop_line_msg = lanelet_pb2.StopLine()

        stop_line_msg.stop_line_id = stop_line_id

        if stop_line.start is not None:
            point_msg = PointMessage.create_message(stop_line.start)
            stop_line_msg.start_point.CopyFrom(point_msg)
        if stop_line.end is not None:
            point_msg = PointMessage.create_message(stop_line.end)
            stop_line_msg.end_point.CopyFrom(point_msg)

        stop_line_msg.line_marking = lanelet_pb2.LineMarkingEnum.LineMarking.Value(stop_line.line_marking.name)
        return stop_line_msg


class TrafficSignMessage:
    @classmethod
    def create_message(cls, traffic_sign: TrafficSign) -> traffic_sign_pb2.TrafficSign:
        traffic_sign_msg = traffic_sign_pb2.TrafficSign()

        traffic_sign_msg.traffic_sign_id = traffic_sign.traffic_sign_id

        for traffic_sign_element in traffic_sign.traffic_sign_elements:
            traffic_sign_element_msg = TrafficSignElementMessage.create_message(traffic_sign_element)
            traffic_sign_msg.traffic_sign_elements.append(traffic_sign_element_msg)

        if traffic_sign.position is not None:
            point_msg = PointMessage.create_message(traffic_sign.position)
            traffic_sign_msg.position.CopyFrom(point_msg)

        if traffic_sign.virtual is not None:
            traffic_sign_msg.virtual = traffic_sign.virtual

        return traffic_sign_msg


class TrafficSignValueMessage:
    @classmethod
    def create_message(cls, traffic_sign_value: TrafficSignValue) -> traffic_sign_value_pb2.TrafficSignValue:
        traffic_sign_value_msg = traffic_sign_value_pb2.TrafficSignValue()

        traffic_sign_value_msg.traffic_sign_id = traffic_sign_value.traffic_sign_id
        for traffic_sign_element in traffic_sign_value.traffic_sign_elements:
            traffic_sign_element_msg = TrafficSignElementMessage.create_message(traffic_sign_element)
            traffic_sign_value_msg.traffic_sign_elements.append(traffic_sign_element_msg)

        return traffic_sign_value_msg


class TrafficSignElementMessage:
    @classmethod
    def create_message(cls, traffic_sign_element: TrafficSignElement) -> traffic_sign_element_pb2.TrafficSignElement:
        traffic_sign_element_msg = traffic_sign_element_pb2.TrafficSignElement()

        element_id = traffic_sign_element.traffic_sign_element_id

        traffic_sign_element_msg.element_id = traffic_sign_element_pb2.TrafficSignIDEnum.TrafficSignID.Value(
            element_id.name
        )

        for additional_value in traffic_sign_element.additional_values:
            traffic_sign_element_msg.additional_values.append(additional_value)

        return traffic_sign_element_msg


class TrafficLightMessage:
    @classmethod
    def create_message(cls, traffic_light: TrafficLight) -> traffic_light_pb2.TrafficLight:
        traffic_light_msg = traffic_light_pb2.TrafficLight()

        traffic_light_msg.traffic_light_id = traffic_light.traffic_light_id

        if traffic_light.position is not None:
            point_msg = PointMessage.create_message(traffic_light.position)
            traffic_light_msg.position.CopyFrom(point_msg)

        for color in traffic_light.color:
            traffic_light_msg.color.append(
                traffic_light_state_pb2.TrafficLightStateEnum.TrafficLightState.Value(color.name)
            )

        if traffic_light.direction is not None:
            traffic_light_msg.direction = traffic_light_pb2.TrafficLightDirectionEnum.TrafficLightDirection.Value(
                traffic_light.direction.name
            )

        return traffic_light_msg


class TrafficLightCycleMessage:
    @classmethod
    def create_message(
        cls, traffic_light_cycle: TrafficLightCycle, traffic_light_id: int
    ) -> traffic_light_cycle_pb2.TrafficLightCycle:
        traffic_light_cycle_msg = traffic_light_cycle_pb2.TrafficLightCycle()

        traffic_light_cycle_msg.traffic_light_id = traffic_light_id

        for cycle_element in traffic_light_cycle.cycle_elements:
            cycle_element_msg = CycleElementMessage.create_message(cycle_element)
            traffic_light_cycle_msg.cycle_elements.append(cycle_element_msg)

        if traffic_light_cycle.time_offset is not None:
            traffic_light_cycle_msg.time_offset = traffic_light_cycle.time_offset

        if traffic_light_cycle.active is not None:
            traffic_light_cycle_msg.active = traffic_light_cycle.active

        return traffic_light_cycle_msg


class CycleElementMessage:
    @classmethod
    def create_message(cls, cycle_element: TrafficLightCycleElement) -> traffic_light_cycle_pb2.CycleElement:
        cycle_element_msg = traffic_light_cycle_pb2.CycleElement()
        cycle_element_msg.duration = cycle_element.duration
        cycle_element_msg.color = traffic_light_state_pb2.TrafficLightStateEnum.TrafficLightState.Value(
            cycle_element.state.name
        )

        return cycle_element_msg


class IntersectionMessage:
    @classmethod
    def create_message(cls, intersection: Intersection) -> intersection_pb2.Intersection:
        intersection_msg = intersection_pb2.Intersection()

        intersection_msg.intersection_id = intersection.intersection_id

        for incoming in intersection.incomings:
            incoming_msg = IncomingGroupMessage.create_message(incoming)
            intersection_msg.incomings.append(incoming_msg)

        for outgoing in intersection.outgoings:
            outgoing_msg = OutgoingGroupMessage.create_message(outgoing)
            intersection_msg.outgoings.append(outgoing_msg)

        for crossing in intersection.crossings:
            crossing_msg = CrossingGroupMessage.create_message(crossing)
            intersection_msg.crossings.append(crossing_msg)

        return intersection_msg


class OutgoingGroupMessage:
    @classmethod
    def create_message(cls, outgoing: OutgoingGroup) -> intersection_pb2.OutgoingGroup:
        outgoing_msg = intersection_pb2.OutgoingGroup()

        outgoing_msg.outgoing_group_id = outgoing.outgoing_id

        for outgoing_lanelet in outgoing.outgoing_lanelets:
            outgoing_msg.outgoing_lanelets.append(outgoing_lanelet)

        if outgoing.incoming_group_id is not None:
            outgoing_msg.incoming_group_id = outgoing.incoming_group_id

        return outgoing_msg


class CrossingGroupMessage:
    @classmethod
    def create_message(cls, crossing: CrossingGroup) -> intersection_pb2.CrossingGroup:
        crossing_msg = intersection_pb2.CrossingGroup()

        crossing_msg.crossing_group_id = crossing.crossing_id

        if crossing.incoming_group_id is not None:
            crossing_msg.incoming_group_id = crossing.incoming_group_id
        if crossing.outgoing_group_id is not None:
            crossing_msg.outgoing_group_id = crossing.outgoing_group_id

        for crossing_lanelet in crossing.crossing_lanelets:
            crossing_msg.crossing_lanelets.append(crossing_lanelet)

        return crossing_msg


class IncomingGroupMessage:
    @classmethod
    def create_message(cls, incoming: IncomingGroup) -> intersection_pb2.IncomingGroup:
        incoming_msg = intersection_pb2.IncomingGroup()

        incoming_msg.incoming_group_id = incoming.incoming_id

        for incoming_lanelet in incoming.incoming_lanelets:
            incoming_msg.incoming_lanelets.append(incoming_lanelet)

        if incoming.outgoing_group_id is not None:
            incoming_msg.outgoing_group_id = incoming.outgoing_group_id

        for outgoing_right in incoming.outgoing_right:
            incoming_msg.outgoing_right.append(outgoing_right)

        for outgoing_straight in incoming.outgoing_straight:
            incoming_msg.outgoing_straight.append(outgoing_straight)

        for outgoing_left in incoming.outgoing_left:
            incoming_msg.outgoing_left.append(outgoing_left)

        return incoming_msg


class StaticObstacleMessage:
    @classmethod
    def create_message(cls, static_obstacle: StaticObstacle) -> static_obstacle_pb2.StaticObstacle:
        static_obstacle_msg = static_obstacle_pb2.StaticObstacle()

        static_obstacle_msg.static_obstacle_id = static_obstacle.obstacle_id

        static_obstacle_msg.obstacle_type = obstacle_pb2.ObstacleTypeEnum.ObstacleType.Value(
            static_obstacle.obstacle_type.name
        )

        shape_msg = ShapeMessage.create_message(static_obstacle.obstacle_shape)
        static_obstacle_msg.shape.CopyFrom(shape_msg)

        state_msg = StateMessage.create_message(static_obstacle.initial_state)
        static_obstacle_msg.initial_state.CopyFrom(state_msg)

        signal_state_msg = SignalStateMessage.create_message(static_obstacle.initial_signal_state)
        static_obstacle_msg.initial_signal_state.CopyFrom(signal_state_msg)

        for signal_state in static_obstacle.signal_series:
            signal_state_msg = SignalStateMessage.create_message(signal_state)
            static_obstacle_msg.signal_series.append(signal_state_msg)

        return static_obstacle_msg


class StateMessage:
    @classmethod
    def create_message(cls, state: State) -> state_pb2.State:
        state_msg = state_pb2.State()

        for attr in state.used_attributes:
            if getattr(state, attr) is None:
                continue

            if attr == "position":
                if isinstance(state.position, np.ndarray):
                    state_msg.point.CopyFrom(PointMessage.create_message(state.position))
                else:
                    state_msg.shape.CopyFrom(ShapeMessage.create_message(state.position))
            elif attr == "time_step":
                integer_exact_or_interval_msg = IntegerExactOrIntervalMessage.create_message(state.time_step)
                state_msg.time_step.CopyFrom(integer_exact_or_interval_msg)
            else:
                float_exact_or_interval_msg = FloatExactOrIntervalMessage.create_message(getattr(state, attr))
                getattr(state_msg, StateMessage._map_to_pb_prop(attr)).CopyFrom(float_exact_or_interval_msg)

        return state_msg

    @staticmethod
    def _map_to_pb_prop(prop: str) -> str:
        return re.sub("(?<!^)(?=[A-Z])", "_", prop).lower()


class SignalStateMessage:
    @classmethod
    def create_message(cls, signal_state: SignalState) -> state_pb2.SignalState:
        signal_state_msg = state_pb2.SignalState()

        for attr in SignalState.__slots__:
            if hasattr(signal_state, attr):
                if getattr(signal_state, attr) is None:
                    continue

                if attr == "time_step":
                    integer_exact_or_interval_msg = IntegerExactOrIntervalMessage.create_message(signal_state.time_step)
                    signal_state_msg.time_step.CopyFrom(integer_exact_or_interval_msg)
                else:
                    setattr(signal_state_msg, attr, getattr(signal_state, attr))

        return signal_state_msg


class DynamicObstacleMessage:
    @classmethod
    def create_message(cls, dynamic_obstacle: DynamicObstacle) -> dynamic_obstacle_pb2.DynamicObstacle:
        dynamic_obstacle_msg = dynamic_obstacle_pb2.DynamicObstacle()

        dynamic_obstacle_msg.dynamic_obstacle_id = dynamic_obstacle.obstacle_id
        dynamic_obstacle_msg.obstacle_type = obstacle_pb2.ObstacleTypeEnum.ObstacleType.Value(
            dynamic_obstacle.obstacle_type.name
        )

        shape_msg = ShapeMessage.create_message(dynamic_obstacle.obstacle_shape)
        dynamic_obstacle_msg.shape.CopyFrom(shape_msg)

        state_msg = StateMessage.create_message(dynamic_obstacle.initial_state)
        dynamic_obstacle_msg.initial_state.CopyFrom(state_msg)

        if isinstance(dynamic_obstacle.prediction, TrajectoryPrediction):
            trajectory_prediction_msg = TrajectoryPredictionMessage.create_message(dynamic_obstacle.prediction)
            dynamic_obstacle_msg.trajectory_prediction.CopyFrom(trajectory_prediction_msg)
        elif isinstance(dynamic_obstacle.prediction, SetBasedPrediction):
            set_based_prediction_msg = SetBasedPredictionMessage.create_message(dynamic_obstacle.prediction)
            dynamic_obstacle_msg.set_based_prediction.CopyFrom(set_based_prediction_msg)

        if dynamic_obstacle.initial_signal_state is not None:
            signal_state_msg = SignalStateMessage.create_message(dynamic_obstacle.initial_signal_state)
            dynamic_obstacle_msg.initial_signal_state.CopyFrom(signal_state_msg)

        if dynamic_obstacle.signal_series is not None:
            for signal_state in dynamic_obstacle.signal_series:
                signal_state_msg = SignalStateMessage.create_message(signal_state)
                dynamic_obstacle_msg.signal_series.append(signal_state_msg)

        return dynamic_obstacle_msg


class TrajectoryMessage:
    @classmethod
    def create_message(cls, trajectory: Trajectory) -> obstacle_pb2.Trajectory:
        trajectory_msg = obstacle_pb2.Trajectory()

        trajectory_msg.initial_time_step = trajectory.initial_time_step

        for state in trajectory.state_list:
            state_msg = StateMessage.create_message(state)
            trajectory_msg.states.append(state_msg)

        return trajectory_msg


class OccupancySetMessage:
    @classmethod
    def create_message(cls, occupancy_set: List[Occupancy]) -> obstacle_pb2.OccupancySet:
        occupancy_set_msg = obstacle_pb2.OccupancySet()

        for occupancy in occupancy_set:
            occupancy_msg = OccupancyMessage.create_message(occupancy)
            occupancy_set_msg.occupancies.append(occupancy_msg)

        return occupancy_set_msg


class OccupancyMessage:
    @classmethod
    def create_message(cls, occupancy: Occupancy) -> obstacle_pb2.Occupancy:
        occupancy_msg = obstacle_pb2.Occupancy()

        integer_exact_or_interval_msg = IntegerExactOrIntervalMessage.create_message(occupancy.time_step)
        occupancy_msg.time_step.CopyFrom(integer_exact_or_interval_msg)

        shape_msg = ShapeMessage.create_message(occupancy.shape)
        occupancy_msg.shape.CopyFrom(shape_msg)

        return occupancy_msg


class TrajectoryPredictionMessage:
    @classmethod
    def create_message(cls, trajectory_prediction: TrajectoryPrediction) -> obstacle_pb2.TrajectoryPrediction:
        trajectory_prediction_msg = obstacle_pb2.TrajectoryPrediction()

        trajectory_msg = TrajectoryMessage.create_message(trajectory_prediction.trajectory)
        trajectory_prediction_msg.trajectory.CopyFrom(trajectory_msg)

        shape_msg = ShapeMessage.create_message(trajectory_prediction.shape)
        trajectory_prediction_msg.shape.CopyFrom(shape_msg)

        return trajectory_prediction_msg


class SetBasedPredictionMessage:
    @classmethod
    def create_message(cls, set_based_prediction: SetBasedPrediction) -> obstacle_pb2.SetBasedPrediction:
        set_based_prediction_msg = obstacle_pb2.SetBasedPrediction()

        set_based_prediction_msg.initial_time_step = set_based_prediction.initial_time_step

        occupancy_set_msg = OccupancySetMessage.create_message(set_based_prediction.occupancy_set)
        set_based_prediction_msg.occupancy_set.CopyFrom(occupancy_set_msg)

        return set_based_prediction_msg


class EnvironmentObstacleMessage:
    @classmethod
    def create_message(cls, environment_obstacle: EnvironmentObstacle) -> environment_obstacle_pb2.EnvironmentObstacle:
        environment_obstacle_msg = environment_obstacle_pb2.EnvironmentObstacle()

        environment_obstacle_msg.environment_obstacle_id = environment_obstacle.obstacle_id
        environment_obstacle_msg.obstacle_type = obstacle_pb2.ObstacleTypeEnum.ObstacleType.Value(
            environment_obstacle.obstacle_type.name
        )

        shape_msg = ShapeMessage.create_message(environment_obstacle.obstacle_shape)
        environment_obstacle_msg.obstacle_shape.CopyFrom(shape_msg)

        return environment_obstacle_msg


class PhantomObstacleMessage:
    @classmethod
    def create_message(cls, phantom_obstacle: PhantomObstacle) -> phantom_obstacle_pb2.PhantomObstacle:
        phantom_obstacle_msg = phantom_obstacle_pb2.PhantomObstacle()

        phantom_obstacle_msg.obstacle_id = phantom_obstacle.obstacle_id

        if phantom_obstacle.prediction is not None:
            set_based_prediction_msg = SetBasedPredictionMessage.create_message(phantom_obstacle.prediction)
            phantom_obstacle_msg.prediction.CopyFrom(set_based_prediction_msg)

        return phantom_obstacle_msg


class PlanningProblemMessage:
    @classmethod
    def create_message(cls, planning_problem: PlanningProblem) -> planning_problem_pb2.PlanningProblem:
        planning_problem_msg = planning_problem_pb2.PlanningProblem()

        planning_problem_msg.planning_problem_id = planning_problem.planning_problem_id

        state_msg = StateMessage.create_message(planning_problem.initial_state)
        planning_problem_msg.initial_state.CopyFrom(state_msg)

        for i, state in enumerate(planning_problem.goal.state_list):
            if planning_problem.goal.lanelets_of_goal_position is not None:
                goal_state_msg = GoalStateMessage.create_message(
                    state, planning_problem.goal.lanelets_of_goal_position[i]
                )
            else:
                goal_state_msg = GoalStateMessage.create_message(state, list())
            planning_problem_msg.goal_states.append(goal_state_msg)

        return planning_problem_msg


class GoalStateMessage:
    @classmethod
    def create_message(cls, state: State, lanelets_of_goal_position: List[int]) -> planning_problem_pb2.GoalState:
        goal_state_msg = planning_problem_pb2.GoalState()

        state_msg = StateMessage.create_message(state)
        goal_state_msg.state.CopyFrom(state_msg)

        for lanelet_id in lanelets_of_goal_position:
            goal_state_msg.goal_position_lanelets.append(lanelet_id)

        return goal_state_msg


class PointMessage:
    @classmethod
    def create_message(cls, point: np.ndarray) -> util_pb2.Point:
        point_msg = util_pb2.Point()

        point_msg.x = point[0]
        point_msg.y = point[1]

        return point_msg


class ShapeMessage:
    @classmethod
    def create_message(cls, shape: Shape) -> util_pb2.Shape:
        shape_msg = util_pb2.Shape()

        if isinstance(shape, Rectangle):
            shape_msg.rectangle.CopyFrom(RectangleMessage.create_message(shape))
        elif isinstance(shape, Circle):
            shape_msg.circle.CopyFrom(CircleMessage.create_message(shape))
        elif isinstance(shape, Polygon):
            shape_msg.polygon.CopyFrom(PolygonMessage.create_message(shape))
        elif isinstance(shape, ShapeGroup):
            shape_msg.shape_group.CopyFrom(ShapeGroupMessage.create_message(shape))

        return shape_msg


class RectangleMessage:
    @classmethod
    def create_message(cls, rectangle: Rectangle) -> util_pb2.Rectangle:
        rectangle_msg = util_pb2.Rectangle()

        rectangle_msg.length = rectangle.length
        rectangle_msg.width = rectangle.width

        if rectangle.center is not None:
            point_msg = PointMessage.create_message(rectangle.center)
            rectangle_msg.center.CopyFrom(point_msg)

        if rectangle.orientation is not None:
            rectangle_msg.orientation = rectangle.orientation

        return rectangle_msg


class CircleMessage:
    @classmethod
    def create_message(cls, circle: Circle) -> util_pb2.Circle:
        circle_msg = util_pb2.Circle()

        circle_msg.radius = circle.radius

        if circle.center is not None:
            point_msg = PointMessage.create_message(circle.center)
            circle_msg.center.CopyFrom(point_msg)

        return circle_msg


class PolygonMessage:
    @classmethod
    def create_message(cls, polygon: Polygon) -> util_pb2.Polygon:
        polygon_msg = util_pb2.Polygon()

        for vertex in polygon.vertices:
            point_msg = PointMessage.create_message(vertex)
            polygon_msg.vertices.append(point_msg)

        return polygon_msg


class ShapeGroupMessage:
    @classmethod
    def create_message(cls, shape_group: ShapeGroup) -> util_pb2.ShapeGroup:
        shape_group_msg = util_pb2.ShapeGroup()

        for shape in shape_group.shapes:
            shape_msg = ShapeMessage.create_message(shape)
            shape_group_msg.shapes.append(shape_msg)

        return shape_group_msg


class IntegerIntervalMessage:
    @classmethod
    def create_message(cls, interval: Interval) -> util_pb2.IntegerInterval:
        integer_interval_msg = util_pb2.IntegerInterval()

        integer_interval_msg.start = interval.start
        integer_interval_msg.end = interval.end

        return integer_interval_msg


class FloatIntervalMessage:
    @classmethod
    def create_message(cls, interval: Interval) -> util_pb2.FloatInterval:
        float_interval_msg = util_pb2.FloatInterval()

        float_interval_msg.start = interval.start
        float_interval_msg.end = interval.end

        return float_interval_msg


class IntegerExactOrIntervalMessage:
    @classmethod
    def create_message(cls, value: Union[int, Interval]) -> util_pb2.IntegerExactOrInterval:
        integer_exact_or_interval_msg = util_pb2.IntegerExactOrInterval()

        if isinstance(value, int):
            integer_exact_or_interval_msg.exact = value
        else:
            integer_interval_msg = IntegerIntervalMessage.create_message(value)
            integer_exact_or_interval_msg.interval.CopyFrom(integer_interval_msg)

        return integer_exact_or_interval_msg


class FloatExactOrIntervalMessage:
    @classmethod
    def create_message(cls, value: Union[int, Interval]) -> util_pb2.FloatExactOrInterval:
        float_exact_or_interval_msg = util_pb2.FloatExactOrInterval()

        if isinstance(value, float) or isinstance(value, int):
            float_exact_or_interval_msg.exact = value
        else:
            float_interval_msg = FloatIntervalMessage.create_message(value)
            float_exact_or_interval_msg.interval.CopyFrom(float_interval_msg)

        return float_exact_or_interval_msg


class IntegerListMessage:
    @classmethod
    def create_message(cls, values: List[int]) -> util_pb2.IntegerList:
        integer_list_msg = util_pb2.IntegerList()

        for value in values:
            integer_list_msg.values.append(value)

        return integer_list_msg


class FloatListMessage:
    @classmethod
    def create_message(cls, values: List[float]) -> util_pb2.FloatList:
        float_list_msg = util_pb2.FloatList()

        for value in values:
            float_list_msg.values.append(value)

        return float_list_msg


class TimeStampMessage:
    @classmethod
    def create_message(cls, time_stamp: Union[datetime.datetime, Time]):
        time_stamp_msg = util_pb2.TimeStamp()

        if isinstance(time_stamp, datetime.datetime):
            time_stamp_msg.year = time_stamp.year
            time_stamp_msg.month = time_stamp.month
            time_stamp_msg.day = time_stamp.day
            time_stamp_msg.hour = time_stamp.hour
            time_stamp_msg.minute = time_stamp.minute
        else:
            time_stamp_msg.hour = time_stamp.hours
            time_stamp_msg.minute = time_stamp.minutes

        return time_stamp_msg
