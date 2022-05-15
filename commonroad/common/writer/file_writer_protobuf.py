import datetime
from typing import Set, Union, List

import numpy as np

from commonroad import SCENARIO_VERSION
from commonroad.common.writer.file_writer import FileWriter, OverwriteExistingFile
from commonroad.planning.planning_problem import PlanningProblemSet, PlanningProblem
from commonroad.protobuf_format.generated_scripts import commonroad_pb2, scenario_tags_pb2, location_pb2, lanelet_pb2, \
    traffic_sign_pb2, traffic_light_pb2, intersection_pb2, static_obstacle_pb2, dynamic_obstacle_pb2, \
    environment_obstacle_pb2, planning_problem_pb2, util_pb2
from commonroad.scenario.intersection import Intersection
from commonroad.scenario.lanelet import Lanelet, LineMarking, StopLine
from commonroad.scenario.obstacle import StaticObstacle, DynamicObstacle, EnvironmentObstacle
from commonroad.scenario.scenario import Scenario, Tag, Location, GeoTransformation, Environment
from google.protobuf import timestamp_pb2

__author__ = "Stefanie Manzinger, Moritz Klischat, Sebastian Maierhofer"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["Priority Program SPP 1835 Cooperative Interacting Automobiles"]
__version__ = "2022.1"
__maintainer__ = "Sebastian Maierhofer"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Released"

from commonroad.scenario.traffic_sign import TrafficSign, TrafficLight, TrafficSignElement


class ProtobufFileWriter(FileWriter):

    def __init__(self, scenario: Scenario, planning_problem_set: PlanningProblemSet, author: str = None,
                 affiliation: str = None, source: str = None, tags: Set[Tag] = None, location: Location = None,
                 decimal_precision: int = 4):
        super().__init__(scenario, planning_problem_set, author, affiliation, source, tags, location, decimal_precision)

        self._commonroad_msg = commonroad_pb2.CommonRoad()

    def _write_header(self):
        information_msg = ScenarioInformationMessage.create_message(str(self.scenario.scenario_id), self._author,
                                                                    self._affiliation, self._source, self.scenario.dt)

        self._commonroad_msg.information.CopyFrom(information_msg)

    def _add_all_objects_from_scenario(self):
        scenario_tags = ScenarioTagsMessage.create_message(list(self.scenario.tags))
        self._commonroad_msg.scenario_tags.CopyFrom(scenario_tags)

        location_msg = LocationMessage.create_message(self.scenario.location)
        self._commonroad_msg.location.CopyFrom(location_msg)

        for lanelet in self.scenario.lanelet_network.lanelets:
            lanelet_msg = LaneletMessage.create_message(lanelet)
            self._commonroad_msg.lanelets.append(lanelet_msg)

        for traffic_sign in self.scenario.lanelet_network.traffic_signs:
            traffic_sign_msg = TrafficSignMessage.create_message(traffic_sign)
            self._commonroad_msg.traffic_signs.append(traffic_sign_msg)

    def _add_all_planning_problems_from_planning_problem_set(self):
        pass

    def write_to_file(self, filename: Union[str, None] = None,
                      overwrite_existing_file: OverwriteExistingFile = OverwriteExistingFile.ASK_USER_INPUT,
                      check_validity: bool = False):
        self._write_header()
        self._add_all_objects_from_scenario()
        self._add_all_planning_problems_from_planning_problem_set()

    def write_scenario_to_file(self, filename: Union[str, None] = None,
                               overwrite_existing_file: OverwriteExistingFile = OverwriteExistingFile.ASK_USER_INPUT):
        pass

    @staticmethod
    def check_validity_of_commonroad_file(commonroad_str: str):
        pass


class ScenarioInformationMessage:

    @classmethod
    def create_message(cls, benchmark_id: str, author: str, affiliation: str, source: str, time_step_size: float) \
            -> commonroad_pb2.ScenarioInformation:
        scenario_information_msg = commonroad_pb2.ScenarioInformation()
        scenario_information_msg.common_road_version = SCENARIO_VERSION
        scenario_information_msg.benchmark_id = benchmark_id
        # scenario_information_msg.date = timestamp_pb2.TimeStamp(datetime.datetime.today()) TODO
        scenario_information_msg.author = author
        scenario_information_msg.affiliation = affiliation
        scenario_information_msg.source = source
        scenario_information_msg.time_step_size = time_step_size

        return scenario_information_msg


class ScenarioTagsMessage:

    @classmethod
    def create_message(cls, tags: List[Tag]) -> scenario_tags_pb2.ScenarioTags:
        scenario_tags_msg = scenario_tags_pb2.ScenarioTags()

        for tag in tags:
            scenario_tags_msg.tags.append(scenario_tags_pb2.TagEnum.URBAN)
            scenario_tags_pb2.TagEnum.Tag.Value(tag.name)

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
        if location.environment is not None:
            environment_msg = EnvironmentMessage.create_message(location.environment)
            location_msg.environment.CopyFrom(environment_msg)

        return location_msg


class GeoTransformationMessage:

    @classmethod
    def create_message(cls, geo_transformation: GeoTransformation) -> location_pb2.GeoTransformation:
        geo_transformation_msg = location_pb2.GeoTransformation()

        geo_transformation_msg.geoReference = geo_transformation.geo_reference
        geo_transformation_msg.x_translation = geo_transformation.x_translation
        geo_transformation_msg.y_translation = geo_transformation.y_translation
        geo_transformation_msg.z_rotation = geo_transformation.z_rotation
        geo_transformation_msg.scaling = geo_transformation.scaling

        return geo_transformation_msg


class EnvironmentMessage:

    @classmethod
    def create_message(cls, environment: Environment) -> location_pb2.Environment:
        environment_msg = location_pb2.Environment()

        # environment_msg.time TODO
        if environment.time_of_day is not None:
            environment_msg.time_of_day = location_pb2.TimeOfDayEnum.TimeOfDay.Value(environment.time_of_day.name)
        if environment.weather is not None:
            environment_msg.weather = location_pb2.WeatherEnum.Weather.Value(environment.weather.name)
        if environment.underground is not None:
            environment_msg.underground = location_pb2.UndergroundEnum.Underground.Value(environment.weather.name)

        return environment_msg


class LaneletMessage:

    @classmethod
    def create_message(cls, lanelet: Lanelet) -> lanelet_pb2.Lanelet:
        lanelet_msg = lanelet_pb2.Lanelet()

        lanelet_msg.lanelet_id = lanelet.lanelet_id

        bound_msg = BoundMessage.create_message(lanelet.left_vertices, lanelet.line_marking_left_vertices)
        lanelet_msg.left_bound.CopyFrom(bound_msg)

        bound_msg = BoundMessage.create_message(lanelet.right_vertices, lanelet.line_marking_right_vertices)
        lanelet_msg.right_bound.CopyFrom(bound_msg)

        for pre in lanelet.predecessor:
            lanelet_msg.predecessors.append(pre)

        for suc in lanelet.successor:
            lanelet_msg.successors.append(suc)

        if lanelet.adj_left is not None:
            lanelet_msg.adjacent_left = lanelet.adj_left

        if lanelet.adj_right is not None:
            lanelet_msg.adjacent_right = lanelet.adj_right

        # left and right driving dirs TODO
        if lanelet.stop_line is not None:
            stop_line_msg = StopLineMessage.create_message(lanelet.stop_line)
            lanelet_msg.stop_line.CopyFrom(stop_line_msg)

        for lanelet_type in lanelet.lanelet_type:
            lanelet_msg.lanelet_types.append(lanelet_pb2.LaneletTypeEnum.LaneletType.Value(lanelet_type.name))

        for user_one_way in lanelet.user_one_way:
            lanelet_msg.user_one_ways.append(lanelet_pb2.RoadUserEnum.RoadUser.Value(user_one_way.name))

        for user_bidirectional in lanelet.user_bidirectional:
            lanelet_msg.user_bidirectionals.append(lanelet_pb2.RoadUserEnum.RoadUser.Value(user_bidirectional.name))

        for ts_ref in lanelet.traffic_signs:
            lanelet_msg.traffic_sign_refs.append(ts_ref)

        for tl_ref in lanelet.traffic_lights:
            lanelet_msg.traffic_light_refs.append(tl_ref)

        return lanelet_msg


class BoundMessage:

    @classmethod
    def create_message(cls, vertices: np.ndarray, line_marking: LineMarking) -> lanelet_pb2.Bound:
        bound_msg = lanelet_pb2.Bound()

        for vertex in vertices:
            point_msg = PointMessage.create_message(vertex)
            bound_msg.points.append(point_msg)

        if line_marking is not None:
            bound_msg.line_marking = lanelet_pb2.LineMarkingEnum.LineMarking.Value(line_marking.name)

        return bound_msg


class StopLineMessage:

    @classmethod
    def create_message(cls, stop_line: StopLine) -> lanelet_pb2.StopLine:
        stop_line_msg = lanelet_pb2.StopLine()

        if stop_line.start is not None:
            point_msg = util_pb2.Point()
            stop_line_msg.points.append(point_msg)

        if stop_line.end is not None:
            point_msg = util_pb2.Point()
            stop_line_msg.points.append(point_msg)

        stop_line_msg.line_marking = lanelet_pb2.LineMarkingEnum.LineMarking.Value(stop_line.line_marking.name)

        for ts_ref in stop_line.traffic_sign_ref:
            stop_line_msg.traffic_sign_refs.append(ts_ref)

        for tl_ref in stop_line.traffic_light_ref:
            stop_line_msg.traffic_light_refs.append(tl_ref)

        return stop_line


class TrafficSignMessage:

    @classmethod
    def create_message(cls, traffic_sign: TrafficSign) -> traffic_sign_pb2.TrafficSign:
        traffic_sign_msg = traffic_sign_pb2.TrafficSign()

        traffic_sign_msg.traffic_sign_id = traffic_sign.traffic_sign_id

        for traffic_sign_element in traffic_sign.traffic_sign_elements:
            traffic_sign_element_msg = TrafficSignElementMessage.create_message(traffic_sign_element)
            traffic_sign_msg.traffic_sign_elements.append(traffic_sign_element_msg)

        # position TODO
        if traffic_sign.virtual is not None:
            traffic_sign_msg.virtual = traffic_sign.virtual

        return traffic_sign_msg


class TrafficSignElementMessage:

    @classmethod
    def create_message(cls, traffic_sign_element: TrafficSignElement) -> traffic_sign_pb2.TrafficSignElement:
        traffic_sign_element_msg = traffic_sign_pb2.TrafficSignElement()

        # traffic_sign_element_id TODO
        for additional_value in traffic_sign_element.additional_values:
            traffic_sign_element_msg.additional_values.append(additional_value)

        return traffic_sign_element_msg


class TrafficLightMessage:

    @classmethod
    def create_message(cls, traffic_light: TrafficLight) -> traffic_light_pb2.TrafficLight:
        pass


class IntersectionMessage:

    @classmethod
    def create_message(cls, intersection: Intersection) -> intersection_pb2.Intersection:
        pass


class StaticObstacleMessage:

    @classmethod
    def create_message(cls, static_obstacle: StaticObstacle) -> static_obstacle_pb2.StaticObstacle:
        pass


class DynamicObstacleMessage:

    @classmethod
    def create_message(cls, dynamic_obstacle: DynamicObstacle) -> dynamic_obstacle_pb2.DynamicObstacle:
        pass


class EnvironmentObstacleMessage:

    @classmethod
    def create_message(cls, environment_obstacle: EnvironmentObstacle) -> environment_obstacle_pb2.EnvironmentObstacle:
        pass


class PlanningProblemMessage:

    @classmethod
    def create_message(cls, planning_problem: PlanningProblem) -> planning_problem_pb2.PlanningProblem:
        pass


class PointMessage:

    @classmethod
    def create_message(cls, point: np.ndarray) -> util_pb2.Point:
        point_msg = util_pb2.Point()

        point_msg.x = point[0]
        point_msg.y = point[1]

        return point_msg
