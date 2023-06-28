import datetime
import logging
import re
from typing import Tuple, List, Set, Union, Dict

import numpy as np

from commonroad.common.common_scenario import ScenarioMetaInformation, FileInformation, MapMetaInformation
from commonroad.common.reader.file_reader_interface import FileReaderScenario, FileReaderMap, \
    FileReaderDynamic
from commonroad.common.util import Interval, AngleInterval, Path_T, Time
from commonroad.geometry.shape import Rectangle, Circle, Polygon, Shape, ShapeGroup
from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import PlanningProblem, CooperativePlanningProblem
from commonroad.prediction.prediction import Occupancy, TrajectoryPrediction, SetBasedPrediction
from commonroad.scenario.area import Area, AreaBorder, AreaType
from commonroad.scenario.intersection import Intersection, IncomingGroup, OutgoingGroup
from commonroad.scenario.lanelet import LaneletNetwork, Lanelet, Bound
from commonroad.common.common_lanelet import RoadUser, StopLine, LineMarking, LaneletType
from commonroad.scenario.obstacle import StaticObstacle, DynamicObstacle, EnvironmentObstacle, PhantomObstacle, \
    SignalState, ObstacleType, MetaInformationState
from commonroad.scenario.scenario import Tag, ScenarioID
from commonroad.common.common_scenario import TimeOfDay, Weather, Underground, Environment, GeoTransformation, Location
from commonroad.scenario.state import InitialState, TraceState, CustomState, SpecificStateClasses
from commonroad.scenario.traffic_sign import TrafficSignElement, TrafficSignID, TrafficSign, TrafficSignValue
from commonroad.scenario.traffic_light import TrafficLightState, TrafficLightDirection, \
    TrafficLightCycleElement,  TrafficLight, TrafficLightCycle
from commonroad.scenario.trajectory import Trajectory
from commonroad.common.reader.dynamic_interface import DynamicInterface
from commonroad.common.reader.scenario_interface import ScenarioInterface

from commonroad.common.pb_scripts.map import area_pb2, commonroad_map_pb2, \
    intersection_pb2, lanelet_pb2, location_pb2, traffic_light_pb2, traffic_sign_pb2
from commonroad.common.pb_scripts.scenario import commonroad_scenario_pb2, \
    planning_problem_pb2, scenario_tags_pb2
from commonroad.common.pb_scripts.dynamic import commonroad_dynamic_pb2, \
    dynamic_obstacle_pb2, environment_obstacle_pb2, environment_pb2, obstacle_pb2, phantom_obstacle_pb2, \
    static_obstacle_pb2, traffic_light_cycle_pb2, traffic_sign_value_pb2
from commonroad.common.pb_scripts.common import state_pb2, \
    traffic_light_state_pb2, traffic_sign_element_pb2, util_pb2, scenario_meta_information_pb2


logger = logging.getLogger(__name__)


class ProtobufFileReaderScenario(FileReaderScenario):
    """
    Reader for CommonRoadScenario file stored in protobuf format.
    """

    def __init__(self, filename: Path_T):
        super().__init__(filename)

    def open(self) -> ScenarioInterface:
        """
        Reads a CommonRoadScenario file in protobuf format.
        :return: CommonRoadScenario
        """
        with open(self._filename, "rb") as file:
            commonroad_scenario_msg = commonroad_scenario_pb2.CommonRoadScenario()
            commonroad_scenario_msg.ParseFromString(file.read())

        return CommonRoadScenarioFactory.create_from_message(commonroad_scenario_msg)


class ProtobufFileReaderMap(FileReaderMap):
    """
    Reader for CommonRoadMap file stored in protobuf format.
    """

    def __init__(self, filename: Path_T):
        super().__init__(filename)

    def open(self) -> LaneletNetwork:
        """
        Reads a CommonRoadMap file in protobuf format.
        :return: CommonRoadMap
        """
        with open(self._filename, "rb") as file:
            commonroad_map_msg = commonroad_map_pb2.CommonRoadMap()
            commonroad_map_msg.ParseFromString(file.read())

        return CommonRoadMapFactory.create_from_message(commonroad_map_msg)


class ProtobufFileReaderDynamic(FileReaderDynamic):
    """
    Reader for CommonRoadDynamic file stored in protobuf format.
    """

    def __init__(self, filename: Path_T):
        super().__init__(filename)

    def open(self) -> DynamicInterface:
        """
        Reads a CommonRoadDynamic file in protobuf format.
        :return: CommonRoadDynamic
        """
        with open(self._filename, "rb") as file:
            commonroad_dynamic_msg = commonroad_dynamic_pb2.CommonRoadDynamic()
            commonroad_dynamic_msg.ParseFromString(file.read())

        return CommonRoadDynamicFactory.create_from_message(commonroad_dynamic_msg)


class CommonRoadDynamicFactory:
    @classmethod
    def create_from_message(cls, commonroad_dynamic_msg: commonroad_dynamic_pb2.CommonRoadDynamic) -> DynamicInterface:
        dynamic_information_msg = commonroad_dynamic_msg.dynamic_meta_information
        dynamic_information = ScenarioMetaInformationFactory.create_from_message(dynamic_information_msg)
        environment_msg = commonroad_dynamic_msg.environment
        dynamic_environment = EnvironmentFactory.create_from_message(environment_msg)

        dynamic = DynamicInterface(dynamic_information, dynamic_environment)

        for static_obstacle_msg in commonroad_dynamic_msg.static_obstacles:
            static_obstacle = StaticObstacleFactory.create_from_message(static_obstacle_msg)
            dynamic.static_obstacles.append(static_obstacle)

        for dynamic_obstacle_msg in commonroad_dynamic_msg.dynamic_obstacles:
            dynamic_obstacle = DynamicObstacleFactory.create_from_message(dynamic_obstacle_msg)
            dynamic.dynamic_obstacles.append(dynamic_obstacle)

        for environment_obstacle_msg in commonroad_dynamic_msg.environment_obstacles:
            environment_obstacle = EnvironmentObstacleFactory.create_from_message(environment_obstacle_msg)
            dynamic.environment_obstacles.append(environment_obstacle)

        for phantom_obstacle_msg in commonroad_dynamic_msg.phantom_obstacles:
            phantom_obstacle = PhantomObstacleFactory.create_from_message(phantom_obstacle_msg)
            dynamic.phantom_obstacles.append(phantom_obstacle)

        for traffic_sign_value_msg in commonroad_dynamic_msg.traffic_sign_value:
            traffic_sign_value = TrafficSignValueFactory.create_from_message(traffic_sign_value_msg)
            dynamic.traffic_sign_value.append(traffic_sign_value)

        #  list that is used to map traffic light ids with traffic light cycles in the higher-level class.
        light_cycle_light_id_dict = dict()

        for traffic_light_cycle_msg in commonroad_dynamic_msg.traffic_light_cycle:
            traffic_light_cycle, tl_id = TrafficLightCycleFactory.create_from_message(traffic_light_cycle_msg)
            dynamic.traffic_light_cycle.append(traffic_light_cycle)
            light_cycle_light_id_dict[tl_id] = traffic_light_cycle

        dynamic.light_cycle_id_dict = light_cycle_light_id_dict

        return dynamic


class CommonRoadMapFactory:
    @classmethod
    def create_from_message(cls, commonroad_map_msg: commonroad_map_pb2.CommonRoadMap) -> LaneletNetwork:
        map_information_msg = commonroad_map_msg.map_meta_information
        map_information = MapMetaInformationFactory.create_from_message(map_information_msg)
        location_msg = commonroad_map_msg.location
        location = LocationFactory.create_from_message(location_msg)
        lanelet_network = LaneletNetwork(map_information, location)

        # List of bounds is needed to be sent to the lanelet factory,
        # so they could be matched to their respective lanelet, as the protobuf and xml format differ
        bounds = list()
        for bound_msg in commonroad_map_msg.boundaries:
            bound = BoundFactory.create_from_message(bound_msg)
            lanelet_network.add_boundary(bound)
            bounds.append(bound)

        # List of stop lines is needed to be sent to the lanelet factory,
        # so they can be matched to their respective lanelet, as the protobuf and xml format differ
        stop_lines = list()
        for stop_line_msg in commonroad_map_msg.stop_lines:
            stop_line = StopLineFactory.create_from_message(stop_line_msg)
            lanelet_network.add_stop_line(stop_line)
            stop_lines.append(stop_line)

        for lanelet_msg in commonroad_map_msg.lanelets:
            lanelet = LaneletFactory.create_from_message(lanelet_msg, bounds, stop_lines)
            lanelet_network.add_lanelet(lanelet)
        for traffic_sign_msg in commonroad_map_msg.traffic_signs:
            traffic_sign = TrafficSignFactory.create_from_message(traffic_sign_msg)
            lanelet_network.add_traffic_sign(traffic_sign, set())
        for traffic_light_msg in commonroad_map_msg.traffic_lights:
            traffic_light = TrafficLightFactory.create_from_message(traffic_light_msg)
            lanelet_network.add_traffic_light(traffic_light, set())
        for intersection_msg in commonroad_map_msg.intersections:
            intersection = IntersectionFactory.create_from_message(intersection_msg)
            lanelet_network.add_intersection(intersection)
        for area_msg in commonroad_map_msg.areas:
            area = AreaFactory.create_from_message(area_msg)
            lanelet_network.add_area(area, set())

        return lanelet_network


class CommonRoadScenarioFactory:
    @classmethod
    def create_from_message(cls, commonroad_scenario_msg: commonroad_scenario_pb2.CommonRoadScenario) -> \
            ScenarioInterface:
        scenario_meta_information_msg = commonroad_scenario_msg.scenario_meta_information

        information = ScenarioMetaInformationFactory.create_from_message(scenario_meta_information_msg)
        map_id = commonroad_scenario_msg.map_id
        dynamic_id = commonroad_scenario_msg.dynamic_id
        scenario = ScenarioInterface(information, map_id, dynamic_id)

        for planning_problem_msg in commonroad_scenario_msg.planning_problems:
            planning_problem = PlanningProblemFactory.create_from_message(planning_problem_msg)
            scenario.planning_problems.append(planning_problem)

        for cooperative_planning_problem_msg in commonroad_scenario_msg.cooperative_planning_problems:
            cooperative_planning_problem = CooperativePlanningProblemFactory.create_from_message\
                (cooperative_planning_problem_msg)
            scenario.cooperative_planning_problems.append(cooperative_planning_problem)

        return scenario


class MapMetaInformationFactory:
    @classmethod
    def create_from_message(cls, map_meta_information_msg: commonroad_map_pb2.MapInformation) \
            -> MapMetaInformation:
        map_id = MapIDFactory.create_from_message(map_meta_information_msg.map_id)
        file_information = FileInformationFactory.create_from_message(map_meta_information_msg.file_information)

        return MapMetaInformation(map_id, file_information)


class ScenarioMetaInformationFactory:
    @classmethod
    def create_from_message(cls, scenario_meta_information_msg: scenario_meta_information_pb2.ScenarioMetaInformation) \
            -> ScenarioMetaInformation:
        scenario_id = ScenarioIDFactory.create_from_message(scenario_meta_information_msg.benchmark_id)
        file_information = FileInformationFactory.create_from_message(scenario_meta_information_msg.file_information)
        time_step_size = scenario_meta_information_msg.time_step_size
        return ScenarioMetaInformation(scenario_id, file_information, time_step_size)


class ScenarioIDFactory:
    @classmethod
    def create_from_message(cls, scenario_id_msg: scenario_meta_information_pb2.ScenarioID) -> ScenarioID:
        cooperative = scenario_id_msg.cooperative
        scenario_id = MapIDFactory.create_from_message(scenario_id_msg.map_id)
        configuration_id = scenario_id_msg.configuration_id
        obstacle_behavior = scenario_id_msg.obstacle_behavior
        prediction_id = scenario_id_msg.prediction_id
        scenario_version = scenario_id_msg.scenario_version
        return ScenarioID(cooperative, scenario_id.country_id, scenario_id.map_name, scenario_id.map_id,
                          configuration_id, obstacle_behavior,
                          prediction_id, scenario_version)


class MapIDFactory:
    @classmethod
    def create_from_message(cls, map_msg: scenario_meta_information_pb2.MapID) -> ScenarioID:
        return ScenarioID(country_id=map_msg.country_id, map_name=map_msg.map_name, map_id=map_msg.map_id)


class FileInformationFactory:
    @classmethod
    def create_from_message(cls, file_information_msg: scenario_meta_information_pb2.FileInformation) \
            -> FileInformation:
        license_name = file_information_msg.license_name
        if file_information_msg.HasField('license_text'):
            license_text = file_information_msg.license_text
        else:
            license_text = None
        date = TimeStampFactory.create_from_message(file_information_msg.date, cr_time=False)
        author = file_information_msg.author
        affiliation = file_information_msg.affiliation
        source = file_information_msg.source

        return FileInformation(date, author, affiliation, source, license_name, license_text)


class ScenarioTagsFactory:

    @classmethod
    def create_from_message(cls, scenario_tags_msg: scenario_tags_pb2.ScenarioTags) -> Set[Tag]:
        tags = set()
        for scenario_tag in scenario_tags_msg.tags:
            tag = scenario_tags_pb2.TagEnum.Tag.Name(scenario_tag)
            tags.add(Tag[tag])

        return tags


class LocationFactory:

    @classmethod
    def create_from_message(cls, location_msg: location_pb2.Location):
        location = Location()

        location.geo_name_id = location_msg.geo_name_id
        location.gps_latitude = location_msg.gps_latitude
        location.gps_longitude = location_msg.gps_longitude

        if location_msg.HasField('geo_transformation'):
            geo_transformation_msg = location_msg.geo_transformation
            location.geo_transformation = GeoTransformationFactory.create_from_message(geo_transformation_msg)

        return location


class GeoTransformationFactory:

    @classmethod
    def create_from_message(cls, geo_transformation_msg: location_pb2.GeoTransformation) -> GeoTransformation:
        geo_transformation = GeoTransformation()

        if geo_transformation_msg.HasField('geo_reference'):
            geo_transformation.geo_reference = geo_transformation_msg.geo_reference

        if geo_transformation_msg.HasField('x_translation'):
            geo_transformation.x_translation = geo_transformation_msg.x_translation

        if geo_transformation_msg.HasField('y_translation'):
            geo_transformation.y_translation = geo_transformation_msg.y_translation

        if geo_transformation_msg.HasField('z_rotation'):
            geo_transformation.z_rotation = geo_transformation_msg.z_rotation

        if geo_transformation_msg.HasField('scaling'):
            geo_transformation.scaling = geo_transformation_msg.scaling

        return geo_transformation


class EnvironmentFactory:

    @classmethod
    def create_from_message(cls, environment_msg: environment_pb2.Environment) -> Environment:
        environment = Environment()

        if environment_msg.HasField('time'):
            environment.time = TimeStampFactory.create_from_message(environment_msg.time, cr_time=True)

        if environment_msg.HasField('time_of_day'):
            time_of_day = environment_pb2.TimeOfDayEnum.TimeOfDay.Name(environment_msg.time_of_day)
            environment.time_of_day = TimeOfDay[time_of_day]

        if environment_msg.HasField('weather'):
            weather = environment_pb2.WeatherEnum.Weather.Name(environment_msg.weather)
            environment.weather = Weather[weather]

        if environment_msg.HasField('underground'):
            underground = environment_pb2.UndergroundEnum.Underground.Name(environment_msg.underground)
            environment.underground = Underground[underground]

        return environment


class LaneletFactory:

    @classmethod
    def create_from_message(cls, lanelet_msg: lanelet_pb2.Lanelet, bounds: List[Bound],
                            stop_lines: List[StopLine]) -> Lanelet:

        lanelet_id = lanelet_msg.lanelet_id
        left_boundary_id = lanelet_msg.left_bound
        right_boundary_id = lanelet_msg.right_bound

        # Setting the initial values of left and right vertices, although we will always assign them bound.points
        left_vertices = np.array(0)
        right_vertices = np.array(0)
        for bound in bounds:
            if left_boundary_id == bound.boundary_id:
                left_vertices = bound.points
            if right_boundary_id == bound.boundary_id:
                right_vertices = bound.points

        center_vertices = 0.5 * (left_vertices + right_vertices)

        lanelet = Lanelet(left_vertices, center_vertices, right_vertices, lanelet_id)

        predecessor = list(lanelet_msg.predecessors)
        lanelet.predecessor = predecessor

        successor = list(lanelet_msg.successors)
        lanelet.successor = successor

        if lanelet_msg.HasField('adjacent_left'):
            lanelet.adj_left = lanelet_msg.adjacent_left

        if lanelet_msg.HasField('adjacent_right'):
            lanelet.adj_right = lanelet_msg.adjacent_right

        if lanelet_msg.HasField('adjacent_left_opposite_dir'):
            left_dir = lanelet_msg.adjacent_left_opposite_dir
            if left_dir is True:
                lanelet.adj_left_same_direction = False
            else:
                lanelet.adj_left_same_direction = True

        if lanelet_msg.HasField('adjacent_right_opposite_dir'):
            right_dir = lanelet_msg.adjacent_right_opposite_dir
            if right_dir is True:
                lanelet.adj_right_same_direction = False
            else:
                lanelet.adj_right_same_direction = True

        #  Should lanelet_msg.left_bound_line_marking be LineMarking instead of int?
        if lanelet_msg.HasField('left_bound_line_marking'):
            lanelet.line_marking_left_vertices = lanelet_msg.left_bound_line_marking

        #  Should lanelet_msg.right_bound_line_marking be LineMarking instead of int?
        if lanelet_msg.HasField('right_bound_line_marking'):
            lanelet.line_marking_right_vertices = lanelet_msg.right_bound_line_marking

        #  TODO: add left_bound_reverse and right_bound_reverse booleans

        #  TODO: Left line marking and right line marking got removed from Bound

        if lanelet_msg.HasField('stop_line'):
            for stop_line in stop_lines:
                if stop_line.stop_line_id == lanelet_msg.stop_line:
                    lanelet.stop_line = stop_line

                    # Because the stop line in the xml format does not contain the attribute stop_line_id,
                    # the lanelet_id is assigned as its id.
                    if stop_line.stop_line_id is None:
                        stop_line.stop_line_id = lanelet.lanelet_id
                    lanelet.stop_line_id = stop_line.stop_line_id

        lanelet_types = set()
        for lanelet_type in lanelet_msg.lanelet_types:
            lanelet_types.add(LaneletType[lanelet_pb2.LaneletTypeEnum.LaneletType.Name(lanelet_type)])
        lanelet.lanelet_type = lanelet_types

        user_one_ways = set()
        for user_one_way in lanelet_msg.user_one_way:
            user_one_ways.add(RoadUser[lanelet_pb2.RoadUserEnum.RoadUser.Name(user_one_way)])
        lanelet.user_one_way = user_one_ways

        user_bidirectionals = set()
        for user_bidirectional in lanelet_msg.user_bidirectional:
            user_bidirectionals.add(RoadUser[lanelet_pb2.RoadUserEnum.RoadUser.Name(user_bidirectional)])
        lanelet.user_bidirectional = user_bidirectionals

        lanelet.traffic_signs = {ts for ts in lanelet_msg.traffic_sign_refs}
        lanelet.traffic_lights = {tl for tl in lanelet_msg.traffic_light_refs}
        lanelet.adjacent_areas = {ae for ae in lanelet_msg.adjacent_areas}

        return lanelet


class BoundFactory:

    @classmethod
    def create_from_message(cls, bound_msg: lanelet_pb2.Bound) -> Bound:

        boundary_id = bound_msg.boundary_id

        points = list()
        for point_msg in bound_msg.points:
            point = PointFactory.create_from_message(point_msg)
            points.append(point)

        bound = Bound(boundary_id, np.array(points))

        return bound


class StopLineFactory:

    @classmethod
    def create_from_message(cls, stop_line_msg: lanelet_pb2.StopLine) -> StopLine:

        stop_line_id = stop_line_msg.stop_line_id

        start_point = PointFactory.create_from_message(stop_line_msg.start_point)
        end_point = PointFactory.create_from_message(stop_line_msg.end_point)

        line_marking = LineMarking[lanelet_pb2.LineMarkingEnum.LineMarking.Name(stop_line_msg.line_marking)]

        stop_line = StopLine(start_point, end_point, line_marking, stop_line_id=stop_line_id)

        return stop_line


class TrafficSignFactory:

    @classmethod
    def create_from_message(cls, traffic_sign_msg: traffic_sign_pb2.TrafficSign):
        traffic_sign_id = traffic_sign_msg.traffic_sign_id

        traffic_sign_elements = list()
        for traffic_sign_element_msg in traffic_sign_msg.traffic_sign_elements:
            traffic_sign_element = TrafficSignElementFactory.create_from_message(traffic_sign_element_msg)
            traffic_sign_elements.append(traffic_sign_element)

        point = PointFactory.create_from_message(traffic_sign_msg.position)

        traffic_sign = TrafficSign(traffic_sign_id, traffic_sign_elements, set(), point)

        if traffic_sign_msg.HasField('virtual'):
            traffic_sign.virtual = traffic_sign_msg.virtual

        return traffic_sign


class TrafficSignValueFactory:
    @classmethod
    def create_from_message(cls, traffic_sign_value_msg: traffic_sign_value_pb2.TrafficSignValue):
        traffic_sign_id = traffic_sign_value_msg.traffic_sign_id
        traffic_sign_elements = list()
        for traffic_sign_element_msg in traffic_sign_value_msg.traffic_sign_elements:
            traffic_sign_element = TrafficSignElementFactory.create_from_message(traffic_sign_element_msg)
            traffic_sign_elements.append(traffic_sign_element)

        return TrafficSignValue(traffic_sign_id, traffic_sign_elements)


class TrafficSignElementFactory:

    @classmethod
    def create_from_message(cls, traffic_sign_element_msg: traffic_sign_element_pb2.TrafficSignElement):
        element_id = TrafficSignID[traffic_sign_element_pb2.TrafficSignIDEnum.TrafficSignID.Name(
                traffic_sign_element_msg.element_id)]
        traffic_sign_element = TrafficSignElement(element_id)

        traffic_sign_element.additional_values = list(traffic_sign_element_msg.additional_values)

        return traffic_sign_element


class TrafficLightFactory:

    @classmethod
    def create_from_message(cls, traffic_light_msg: traffic_light_pb2.TrafficLight) -> TrafficLight:
        traffic_light_id = traffic_light_msg.traffic_light_id
        point = PointFactory.create_from_message(traffic_light_msg.position)

        traffic_light = TrafficLight(traffic_light_id, point)

        if traffic_light_msg.HasField('direction'):
            traffic_light.direction = TrafficLightDirection[
                traffic_light_pb2.TrafficLightDirectionEnum.TrafficLightDirection.Name(traffic_light_msg.direction)]

        for color in traffic_light_msg.color:
            traffic_light.color.append(TrafficLightState[traffic_light_state_pb2.TrafficLightStateEnum.
                                        TrafficLightState.Name(color)])

        return traffic_light


class TrafficLightCycleFactory:
    @classmethod
    def create_from_message(cls, traffic_light_cycle_msg: traffic_light_cycle_pb2.TrafficLightCycle) -> \
            Tuple[TrafficLightCycle, int]:
        traffic_light_id = traffic_light_cycle_msg.traffic_light_id
        cycle_elements = list()
        for cycle_element_msg in traffic_light_cycle_msg.cycle_elements:
            cycle_element = CycleElementFactory.create_from_message(cycle_element_msg)
            cycle_elements.append(cycle_element)

        traffic_light_cycle = TrafficLightCycle(cycle_elements)

        if traffic_light_cycle_msg.HasField('time_offset'):
            traffic_light_cycle.time_offset = traffic_light_cycle_msg.time_offset

        if traffic_light_cycle_msg.HasField('active'):
            traffic_light_cycle.active = traffic_light_cycle_msg.active

        return traffic_light_cycle, traffic_light_id


class CycleElementFactory:
    @classmethod
    def create_from_message(cls, cycle_element_msg: traffic_light_cycle_pb2.CycleElement) -> TrafficLightCycleElement:
        duration = cycle_element_msg.duration
        color = TrafficLightState[traffic_light_state_pb2.TrafficLightStateEnum.TrafficLightState.Name(
                cycle_element_msg.color)]
        return TrafficLightCycleElement(color, duration)


class AreaFactory:
    @classmethod
    def create_from_message(cls, area_msg: area_pb2.Area) -> Area:
        area_id = area_msg.area_id
        area = Area(area_id)

        border = list()
        for area_border_msg in area_msg.border:
            area_border = AreaBorderFactory.create_from_message(area_border_msg)
            border.append(area_border)
        area.border = border

        area_types = set()
        for area_type_msg in area_msg.area_types:
            area_type = AreaType[area_pb2.AreaTypeEnum.AreaType.Name(area_type_msg)]
            area_types.add(area_type)

        area.area_types = area_types

        return area


class AreaBorderFactory:
    @classmethod
    def create_from_message(cls, area_border_msg: area_pb2.AreaBorder) -> AreaBorder:
        area_border_id = area_border_msg.area_border_id

        boundary = area_border_msg.boundary

        area_border = AreaBorder(area_border_id, boundary)

        if area_border_msg.HasField('adjacent'):
            adjacent = area_border_msg.adjacent
            area_border.adjacent = adjacent

        if area_border_msg.HasField('line_marking'):
            line_marking = LineMarking[lanelet_pb2.LineMarkingEnum.LineMarking.Name(area_border_msg.line_marking)]
            area_border.line_marking = line_marking

        return area_border


class IntersectionFactory:
    @classmethod
    def create_from_message(cls, intersection_msg: intersection_pb2.Intersection) -> Intersection:
        intersection_id = intersection_msg.intersection_id

        incomings = list()
        for incoming_msg in intersection_msg.incomings:
            incoming = IncomingGroupFactory.create_from_message(incoming_msg)
            incomings.append(incoming)

        outgoings = list()
        for outgoing_msg in intersection_msg.outgoings:
            outgoing = OutgoingGroupFactory.create_from_message(outgoing_msg)
            outgoings.append(outgoing)

        intersection = Intersection(intersection_id, incomings, outgoings)

        return intersection


class IncomingGroupFactory:
    @classmethod
    def create_from_message(cls, incoming_group_msg: intersection_pb2.IncomingGroup) -> IncomingGroup:
        incoming = IncomingGroup(incoming_group_msg.incoming_group_id)
        incoming.incoming_lanelets = set(incoming_group_msg.incoming_lanelets)
        if incoming_group_msg.HasField('outgoing_group_id'):
            incoming.outgoing_id = incoming_group_msg.outgoing_group_id
        incoming.outgoing_right = set(incoming_group_msg.outgoing_right)
        incoming.outgoing_straight = set(incoming_group_msg.outgoing_straight)
        incoming.outgoing_left = set(incoming_group_msg.outgoing_left)
        incoming.crossings = set(incoming_group_msg.crossing_lanelets)
        return incoming


class OutgoingGroupFactory:
    @classmethod
    def create_from_message(cls, outgoing_group_msg: intersection_pb2.OutgoingGroup) -> OutgoingGroup:
        outgoing_id = outgoing_group_msg.outgoing_group_id
        outgoing_lanelets = set(outgoing_group_msg.outgoing_lanelets)
        return OutgoingGroup(outgoing_id, outgoing_lanelets)


class StaticObstacleFactory:

    @classmethod
    def create_from_message(cls, static_obstacle_msg: static_obstacle_pb2.StaticObstacle) -> StaticObstacle:
        static_obstacle_id = static_obstacle_msg.static_obstacle_id

        obstacle_type = ObstacleType[obstacle_pb2.ObstacleTypeEnum.ObstacleType.Name(static_obstacle_msg.obstacle_type)]

        shape = ShapeFactory.create_from_message(static_obstacle_msg.shape)

        initial_state = StateFactory.create_from_message(static_obstacle_msg.initial_state, is_initial_state=True)

        static_obstacle = StaticObstacle(static_obstacle_id, obstacle_type, shape, initial_state)

        static_obstacle.initial_center_lanelet_ids = None
        static_obstacle.initial_shape_lanelet_ids = None

        if static_obstacle_msg.HasField('initial_signal_state'):
            initial_signal_state = SignalStateFactory.create_from_message(static_obstacle_msg.initial_signal_state)
            static_obstacle.initial_signal_state = initial_signal_state

        signal_states = list()
        for signal_state_msg in static_obstacle_msg.signal_series:
            signal_state = SignalStateFactory.create_from_message(signal_state_msg)
            signal_states.append(signal_state)
        static_obstacle.signal_series = signal_states

        return static_obstacle


class DynamicObstacleFactory:

    @classmethod
    def create_from_message(cls, dynamic_obstacle_msg: dynamic_obstacle_pb2.DynamicObstacle) -> \
            DynamicObstacle:
        dynamic_obstacle_id = dynamic_obstacle_msg.dynamic_obstacle_id

        obstacle_type = ObstacleType[
            obstacle_pb2.ObstacleTypeEnum.ObstacleType.Name(dynamic_obstacle_msg.obstacle_type)]

        shape = ShapeFactory.create_from_message(dynamic_obstacle_msg.shape)

        initial_state = StateFactory.create_from_message(dynamic_obstacle_msg.initial_state, is_initial_state=True)

        prediction = None
        if dynamic_obstacle_msg.HasField('trajectory_prediction'):
            prediction = TrajectoryPredictionFactory \
                .create_from_message(dynamic_obstacle_msg.trajectory_prediction)
        elif dynamic_obstacle_msg.HasField('set_based_prediction'):
            prediction = SetBasedPredictionFactory.create_from_message(dynamic_obstacle_msg.set_based_prediction)

        dynamic_obstacle = DynamicObstacle(dynamic_obstacle_id, obstacle_type, shape, initial_state, prediction)

        dynamic_obstacle.initial_center_lanelet_ids = None
        dynamic_obstacle.initial_shape_lanelet_ids = None

        if dynamic_obstacle_msg.HasField('initial_signal_state'):
            dynamic_obstacle.initial_signal_state = SignalStateFactory \
                .create_from_message(dynamic_obstacle_msg.initial_signal_state)

        signal_states = list()
        for signal_state_msg in dynamic_obstacle_msg.signal_series:
            signal_state = SignalStateFactory.create_from_message(signal_state_msg)
            signal_states.append(signal_state)
        dynamic_obstacle.signal_series = signal_states

        if dynamic_obstacle_msg.HasField('initial_meta_information_state'):
            dynamic_obstacle.initial_meta_information_state = MetaInformationStateFactory.create_from_message\
            (dynamic_obstacle_msg.meta_information_state)

        for meta_information_state_msg in dynamic_obstacle_msg.meta_information_series:
            meta_information_state = MetaInformationStateFactory.create_from_message(meta_information_state_msg)
            dynamic_obstacle.meta_information_series.append(meta_information_state)

        if dynamic_obstacle_msg.HasField('external_dataset_id'):
            dynamic_obstacle.external_dataset_id = dynamic_obstacle_msg.external_dataset_id

        return dynamic_obstacle


class EnvironmentObstacleFactory:

    @classmethod
    def create_from_message(cls, environment_obstacle_msg: environment_obstacle_pb2.EnvironmentObstacle) \
            -> EnvironmentObstacle:
        environment_obstacle_id = environment_obstacle_msg.environment_obstacle_id

        obstacle_type = ObstacleType[
            obstacle_pb2.ObstacleTypeEnum.ObstacleType.Name(environment_obstacle_msg.obstacle_type)]

        shape = ShapeFactory.create_from_message(environment_obstacle_msg.obstacle_shape)

        return EnvironmentObstacle(environment_obstacle_id, obstacle_type, shape)


class PhantomObstacleFactory:

    @classmethod
    def create_from_message(cls, phantom_obstacle_msg: phantom_obstacle_pb2.PhantomObstacle) -> PhantomObstacle:
        obstacle_id = phantom_obstacle_msg.obstacle_id

        phantom_obstacle = PhantomObstacle(obstacle_id)

        if phantom_obstacle_msg.HasField('prediction'):
            set_based_prediction = SetBasedPredictionFactory.create_from_message(phantom_obstacle_msg.prediction)
            phantom_obstacle.prediction = set_based_prediction

        return phantom_obstacle


class MetaInformationStateFactory:
    @classmethod
    def create_from_message(cls, meta_information_state_msg: obstacle_pb2.MetaInformationState) -> MetaInformationState:
        meta_information_state = MetaInformationState()
        if meta_information_state_msg.HasField('meta_data_str'):
            meta_information_state.meta_data_str = meta_information_state_msg.meta_data_str
        if meta_information_state_msg.HasField('meta_data_int'):
            meta_information_state.meta_data_int = meta_information_state_msg.meta_data_int
        if meta_information_state_msg.HasField('meta_data_float'):
            meta_information_state.meta_data_float = meta_information_state_msg.meta_data_float
        if meta_information_state_msg.HasField('meta_data_bool'):
            meta_information_state.meta_data_bool = meta_information_state_msg.meta_data_bool

        return meta_information_state


class StateFactory:

    @classmethod
    def create_from_message(cls, state_msg: state_pb2.State, is_initial_state: bool = False) -> TraceState:
        states = [state_class() for state_class in SpecificStateClasses]

        used_fields = list()
        for field in [field.name for field in state_msg.DESCRIPTOR.fields]:
            if state_msg.HasField(field):
                if field == 'point' or field == 'shape':
                    used_fields.append('position')
                else:
                    used_fields.append(field)

        matched_state = None
        for state in states:

            if len(state.attributes) != len(used_fields) and not is_initial_state:
                continue

            filled = StateFactory._fill_state(state, state_msg, state.attributes)

            if isinstance(state, InitialState) and is_initial_state:
                state.fill_with_defaults()
                return state

            if filled:
                matched_state = state
                break

        if matched_state is None:
            matched_state = CustomState()
            StateFactory._fill_state(matched_state, state_msg, used_fields)
            logger.debug("State type at time step %s cannot be matched! Creating custom state!",
                         getattr(state_msg, 'time_step'))

        return matched_state

    @classmethod
    def _fill_state(cls, state: TraceState, state_msg: state_pb2.State, attrs: List[str]) -> bool:
        for attr in attrs:
            if (hasattr(state_msg, attr) and state_msg.HasField(attr)) or attr == 'position':
                if attr == 'time_step':
                    setattr(state, attr, IntegerExactOrIntervalFactory.create_from_message(state_msg.time_step))
                elif attr == 'position':
                    if state_msg.HasField('point'):
                        setattr(state, attr, PointFactory.create_from_message(state_msg.point))
                    elif state_msg.HasField('shape'):
                        setattr(state, attr, ShapeFactory.create_from_message(state_msg.shape))
                elif attr == 'orientation':
                    setattr(state, attr,
                            FloatExactOrIntervalFactory.create_from_message(state_msg.orientation, is_angle=True))
                else:
                    setattr(state, attr, FloatExactOrIntervalFactory
                            .create_from_message(getattr(state_msg, StateFactory._map_to_pb_prop(attr))))
            else:
                return False

        return True

    @staticmethod
    def _map_to_pb_prop(prop: str) -> str:
        return re.sub('(?<!^)(?=[A-Z])', '_', prop).lower()


class SignalStateFactory:

    @classmethod
    def create_from_message(cls, signal_state_msg: state_pb2.SignalState) -> SignalState:
        kwargs = dict()

        for attr in SignalState.__slots__:
            if signal_state_msg.HasField(attr):
                if attr == 'time_step':
                    value = IntegerExactOrIntervalFactory.create_from_message(signal_state_msg.time_step)
                else:
                    value = getattr(signal_state_msg, attr)
                kwargs.update({attr: value})

        return SignalState(**kwargs) if kwargs else None


class OccupancyFactory:

    @classmethod
    def create_from_message(cls, occupancy_msg: obstacle_pb2.Occupancy) -> Occupancy:
        time_step = IntegerExactOrIntervalFactory.create_from_message(occupancy_msg.time_step)
        shape = ShapeFactory.create_from_message(occupancy_msg.shape)

        return Occupancy(time_step, shape)


class OccupancySetFactory:

    @classmethod
    def create_from_message(cls, occupancy_set_msg: obstacle_pb2.OccupancySet) -> List[Occupancy]:
        occupancies = list()
        for occupancy_msg in occupancy_set_msg.occupancies:
            occupancy = OccupancyFactory.create_from_message(occupancy_msg)
            occupancies.append(occupancy)

        return occupancies


class TrajectoryFactory:

    @classmethod
    def create_from_message(cls, trajectory_msg: obstacle_pb2.Trajectory) -> Trajectory:
        initial_time_step = trajectory_msg.initial_time_step

        states = list()
        for state_msg in trajectory_msg.states:
            state = StateFactory.create_from_message(state_msg)
            states.append(state)

        return Trajectory(initial_time_step, states)


class TrajectoryPredictionFactory:

    @classmethod
    def create_from_message(cls, trajectory_prediction_msg: obstacle_pb2.TrajectoryPrediction) -> TrajectoryPrediction:
        trajectory = TrajectoryFactory.create_from_message(trajectory_prediction_msg.trajectory)

        shape = ShapeFactory.create_from_message(trajectory_prediction_msg.shape)

        trajectory_prediction = TrajectoryPrediction(trajectory, shape)

        trajectory_prediction.center_lanelet_assignment = None
        trajectory_prediction.shape_lanelet_assignment = None

        return trajectory_prediction

    @staticmethod
    def find_obstacle_shape_lanelets(initial_state: InitialState, state_list: List[TraceState],
                                     lanelet_network: LaneletNetwork,
                                     obstacle_id: int, shape: Shape) -> Dict[int, Set[int]]:

        compl_state_list = [initial_state] + state_list
        lanelet_ids_per_state = {}

        for state in compl_state_list:
            rotated_shape = shape.rotate_translate_local(state.position, state.orientation)
            lanelet_ids = lanelet_network.find_lanelet_by_shape(rotated_shape)
            for l_id in lanelet_ids:
                lanelet_network.find_lanelet_by_id(l_id).add_dynamic_obstacle_to_lanelet(obstacle_id=obstacle_id,
                                                                                         time_step=state.time_step)
            lanelet_ids_per_state[state.time_step] = set(lanelet_ids)

        return lanelet_ids_per_state

    @staticmethod
    def find_obstacle_center_lanelets(initial_state: InitialState, state_list: List[TraceState],
                                      lanelet_network: LaneletNetwork) -> Dict[int, Set[int]]:
        compl_state_list = [initial_state] + state_list
        lanelet_ids_per_state = {}

        for state in compl_state_list:
            lanelet_ids = lanelet_network.find_lanelet_by_position([state.position])[0]
            lanelet_ids_per_state[state.time_step] = set(lanelet_ids)

        return lanelet_ids_per_state


class SetBasedPredictionFactory:

    @classmethod
    def create_from_message(cls, set_based_prediction_msg: obstacle_pb2.SetBasedPrediction) -> SetBasedPrediction:
        initial_time_step = set_based_prediction_msg.initial_time_step

        occupancy_set = OccupancySetFactory.create_from_message(set_based_prediction_msg.occupancy_set)

        return SetBasedPrediction(initial_time_step, occupancy_set)


class CooperativePlanningProblemFactory:
    @classmethod
    def create_from_message(cls, cooperative_planning_problem_msg: planning_problem_pb2.CooperativePlanningProblem) -> \
            CooperativePlanningProblem:
        cooperative_planning_problem_id = cooperative_planning_problem_msg.cooperative_planning_problem_id
        single_planning_problem_id_list = set()
        for single_planning_problem_id in cooperative_planning_problem_msg.single_planning_problem_id:
            single_planning_problem_id_list.add(single_planning_problem_id)

        return CooperativePlanningProblem(cooperative_planning_problem_id, single_planning_problem_id_list)


class PlanningProblemFactory:

    @classmethod
    def create_from_message(cls, planning_problem_msg: planning_problem_pb2.PlanningProblem) -> PlanningProblem:
        planning_problem_id = planning_problem_msg.planning_problem_id

        initial_state = StateFactory.create_from_message(planning_problem_msg.initial_state, is_initial_state=True)

        state_list = list()
        lanelets_of_goal_position = None
        for goal_state_msg in planning_problem_msg.goal_states:
            state, lg_position = GoalStateFactory.create_from_message(goal_state_msg)
            state_list.append(state)
            if lg_position is not None:
                if lanelets_of_goal_position is None:
                    lanelets_of_goal_position = dict()
                lanelets_of_goal_position.update({len(state_list) - 1: lg_position})

        goal_region = GoalRegion(state_list, lanelets_of_goal_position)

        scenario_tags = ScenarioTagsFactory.create_from_message(planning_problem_msg.scenario_tags)

        planning_problem = PlanningProblem(planning_problem_id, initial_state, goal_region, scenario_tags)

        if planning_problem_msg.HasField('ego_id'):
            planning_problem.ego_id = planning_problem_msg.ego_id

        return planning_problem


class GoalStateFactory:

    @classmethod
    def create_from_message(cls, goal_state_msg: planning_problem_pb2.GoalState) -> Tuple[TraceState, List[int]]:
        state = StateFactory.create_from_message(goal_state_msg.state)

        goal_position_lanelets = None
        if goal_state_msg.goal_position_lanelets:
            goal_position_lanelets = list(goal_state_msg.goal_position_lanelets)

        return state, goal_position_lanelets


class PointFactory:

    @classmethod
    def create_from_message(cls, point_msg: util_pb2.Point) -> np.ndarray:
        return np.array([point_msg.x, point_msg.y])


class RectangleFactory:

    @classmethod
    def create_from_message(cls, rectangle_msg: util_pb2.Rectangle) -> Rectangle:
        rectangle = Rectangle(rectangle_msg.length, rectangle_msg.width)

        if rectangle_msg.HasField('center'):
            point = PointFactory.create_from_message(rectangle_msg.center)
            rectangle.center = point

        if rectangle_msg.HasField('orientation'):
            rectangle.orientation = rectangle_msg.orientation

        return rectangle


class CircleFactory:

    @classmethod
    def create_from_message(cls, circle_msg: util_pb2.Circle) -> Circle:
        circle = Circle(circle_msg.radius)

        if circle_msg.HasField('center'):
            point = PointFactory.create_from_message(circle_msg.center)
            circle.center = point

        return circle


class PolygonFactory:

    @classmethod
    def create_from_message(cls, polygon_msg: util_pb2.Polygon) -> Polygon:
        vertices = list()
        for point_msg in polygon_msg.vertices:
            point = PointFactory.create_from_message(point_msg)
            vertices.append(point)
        polygon = Polygon(np.array(vertices))

        return polygon


class ShapeGroupFactory:

    @classmethod
    def create_from_message(cls, shape_group_msg: util_pb2.ShapeGroup) -> ShapeGroup:
        shapes = list()
        for shape_msg in shape_group_msg.shapes:
            shape = ShapeFactory.create_from_message(shape_msg)
            shapes.append(shape)
        shape_group = ShapeGroup(shapes)

        return shape_group


class ShapeFactory:

    @classmethod
    def create_from_message(cls, shape_msg: util_pb2.Shape) -> Shape:
        if shape_msg.HasField('rectangle'):
            shape = RectangleFactory.create_from_message(shape_msg.rectangle)
        elif shape_msg.HasField('circle'):
            shape = CircleFactory.create_from_message(shape_msg.circle)
        elif shape_msg.HasField('polygon'):
            shape = PolygonFactory.create_from_message(shape_msg.polygon)
        else:
            shape = ShapeGroupFactory.create_from_message(shape_msg.shape_group)

        return shape


class IntegerIntervalFactory:

    @classmethod
    def create_from_message(cls, integer_interval_msg: util_pb2.IntegerInterval) -> Interval:
        return Interval(integer_interval_msg.start, integer_interval_msg.end)


class FloatIntervalFactory:

    @classmethod
    def create_from_message(cls, float_interval_msg: util_pb2.FloatInterval, is_angle: bool = False) -> Interval:
        if is_angle:
            interval = AngleInterval(float_interval_msg.start, float_interval_msg.end)
        else:
            interval = Interval(float_interval_msg.start, float_interval_msg.end)
        return interval


class IntegerExactOrIntervalFactory:

    @classmethod
    def create_from_message(cls, integer_exact_or_interval_msg: util_pb2.IntegerExactOrInterval) \
            -> Union[int, Interval]:
        if integer_exact_or_interval_msg.HasField('exact'):
            return integer_exact_or_interval_msg.exact

        if integer_exact_or_interval_msg.HasField('interval'):
            return IntegerIntervalFactory.create_from_message(integer_exact_or_interval_msg.interval)


class FloatExactOrIntervalFactory:

    @classmethod
    def create_from_message(cls, float_exact_or_interval_msg: util_pb2.FloatExactOrInterval, is_angle: bool = False) \
            -> Union[float, Interval]:
        if float_exact_or_interval_msg.HasField('exact'):
            return float_exact_or_interval_msg.exact

        if float_exact_or_interval_msg.HasField('interval'):
            return FloatIntervalFactory.create_from_message(float_exact_or_interval_msg.interval, is_angle)


class IntegerListFactory:

    @classmethod
    def create_from_message(cls, integer_list_msg: util_pb2.IntegerList) -> List[int]:
        return list(integer_list_msg)


class FloatListFactory:

    @classmethod
    def create_from_message(cls, float_list_msg: util_pb2.FloatList) -> List[float]:
        return list(float_list_msg)


class TimeStampFactory:

    @classmethod
    def create_from_message(cls, time_stamp_msg: util_pb2.TimeStamp, cr_time: bool) -> Union[datetime.datetime, Time]:
        year = 2022
        month = day = 1
        minute = hour = 0

        if time_stamp_msg.HasField('year'):
            year = time_stamp_msg.year
        if time_stamp_msg.HasField('month'):
            month = time_stamp_msg.month
        if time_stamp_msg.HasField('day'):
            day = time_stamp_msg.day
        if time_stamp_msg.HasField('hour'):
            hour = time_stamp_msg.hour
        if time_stamp_msg.HasField('minute'):
            minute = time_stamp_msg.minute

        return Time(hour, minute) if cr_time else datetime.datetime(year, month, day, hour, minute)
