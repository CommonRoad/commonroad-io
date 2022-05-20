from typing import Tuple, List, Set, Union

import numpy as np

from commonroad.common.reader.file_reader_interface import FileReader
from commonroad.common.util import Interval
from commonroad.geometry.shape import Rectangle, Circle, Polygon, Shape, ShapeGroup
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.protobuf_format.generated_scripts import commonroad_pb2, scenario_tags_pb2, location_pb2, lanelet_pb2, \
    util_pb2, traffic_sign_pb2, traffic_light_pb2
from commonroad.scenario.lanelet import LaneletNetwork, Lanelet, StopLine, LineMarking, LaneletType, RoadUser
from commonroad.scenario.scenario import Scenario, ScenarioID, Tag, GeoTransformation, Location, Environment, TimeOfDay, \
    Weather, Underground

__author__ = "Stefanie Manzinger, Sebastian Maierhofer"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["Priority Program SPP 1835 Cooperative Interacting Automobiles", "CAR@TUM"]
__version__ = "2022.1"
__maintainer__ = "Stefanie Manzinger, Sebastian Maierhofer"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Released"

from commonroad.scenario.traffic_sign import TrafficSignElement, TrafficSignIDGermany, TrafficSignIDZamunda, \
    TrafficSignIDUsa, TrafficSignIDChina, TrafficSignIDSpain, TrafficSignIDRussia, TrafficSignIDArgentina, \
    TrafficSignIDBelgium, TrafficSignIDFrance, TrafficSignIDGreece, TrafficSignIDCroatia, TrafficSignIDItaly, \
    TrafficSignIDPuertoRico, TrafficSign, TrafficLight, TrafficLightCycleElement


class ProtobufFileReader(FileReader):

    def __init__(self, filename: str):
        super().__init__(filename)

    def open(self, lanelet_assignment: bool = False) -> Tuple[Scenario, PlanningProblemSet]:
        file = open(self._filename, "rb")
        commonroad_msg = commonroad_pb2.CommonRoad()
        commonroad_msg.ParseFromString(file.read())

        return CommonRoadFactory.create_from_message(commonroad_msg)

    def open_lanelet_network(self) -> LaneletNetwork:
        file = open(self._filename, "rb")
        commonroad_msg = commonroad_pb2.CommonRoad()
        commonroad_msg.ParseFromString(file.read())

        scenario, _ = CommonRoadFactory.create_from_message(commonroad_msg)

        return scenario.lanelet_network


class CommonRoadFactory:

    @classmethod
    def create_from_message(cls, commonroad_msg: commonroad_pb2.CommonRoad) -> Tuple[Scenario, PlanningProblemSet]:
        scenario_information_msg = commonroad_msg.information
        common_road_version, benchmark_id, author, affiliation, source, time_step_size = \
            ScenarioInformationFactory.create_from_message(scenario_information_msg)

        scenario = Scenario(time_step_size)
        scenario.scenario_id = ScenarioID.from_benchmark_id(benchmark_id, common_road_version)
        scenario.author = author
        scenario.affiliation = affiliation
        scenario.source = source

        scenario_tags = ScenarioTagsFactory.create_from_message(commonroad_msg.scenario_tags)
        scenario.tags = scenario_tags

        location = LocationFactory.create_from_message(commonroad_msg.location)
        scenario.location = location

        for lanelet_msg in commonroad_msg.lanelets:
            lanelet = LaneletFactory.create_from_message(lanelet_msg)
            scenario.lanelet_network.lanelets.append(lanelet)

        for traffic_sign_msg in commonroad_msg.traffic_signs:
            traffic_sign = TrafficSignFactory.create_from_message(traffic_sign_msg)
            scenario.lanelet_network.traffic_signs.append(traffic_sign)

        planning_problem_set = PlanningProblemSet()

        return scenario, planning_problem_set


class ScenarioInformationFactory:

    @classmethod
    def create_from_message(cls, scenario_information_msg: commonroad_pb2.ScenarioInformation):
        common_road_version = scenario_information_msg.common_road_version
        benchmark_id = scenario_information_msg.benchmark_id
        # date = scenario_information_msg.date TODO
        author = scenario_information_msg.author
        affiliation = scenario_information_msg.affiliation
        source = scenario_information_msg.source
        time_step_size = scenario_information_msg.time_step_size

        return common_road_version, benchmark_id, author, affiliation, source, time_step_size


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

        if location_msg.HasField('environment'):
            environment_msg = location_msg.environment
            location.environment = EnvironmentFactory.create_from_message(environment_msg)

        return location


class GeoTransformationFactory:

    @classmethod
    def create_from_message(cls, geo_transformation_msg: location_pb2.GeoTransformation) -> GeoTransformation:
        geo_transformation = GeoTransformation()

        if geo_transformation_msg.HasField('geoReference'):
            geo_transformation.geo_reference = geo_transformation_msg.geoReference  # TODO

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
    def create_from_message(cls, environment_msg: location_pb2.Environment) -> Environment:
        environment = Environment()

        if environment_msg.HasField('time'):
            pass  # TODO

        if environment_msg.HasField('time_of_day'):
            time_of_day = location_pb2.TimeOfDayEnum.TimeOfDay.Name(environment_msg.time_of_day)
            environment.time_of_day = TimeOfDay[time_of_day]

        if environment_msg.HasField('weather'):
            weather = location_pb2.WeatherEnum.Weather.Name(environment_msg.weather)
            environment.weather = Weather[weather]

        if environment_msg.HasField('underground'):
            underground = location_pb2.UndergroundEnum.Underground.Name(environment_msg.underground)
            environment.underground = Underground[underground]

        return environment


class LaneletFactory:

    @classmethod
    def create_from_message(cls, lanelet_msg: lanelet_pb2.Lanelet) -> Lanelet:
        lanelet_id = lanelet_msg.lanelet_id
        left_vertices, left_line_marking = BoundFactory.create_from_message(lanelet_msg.left_bound)
        right_vertices, right_line_marking = BoundFactory.create_from_message(lanelet_msg.right_bound)
        center_vertices = 0.5 * (left_vertices + right_vertices)

        lanelet = Lanelet(left_vertices, center_vertices, right_vertices, lanelet_id)

        predecessor = [p for p in lanelet_msg.predecessors]
        lanelet.predecessor = predecessor

        successor = [s for s in lanelet_msg.successors]
        lanelet.successor = successor

        if lanelet_msg.HasField('adjacent_left'):
            lanelet.adj_left = lanelet_msg.adjacent_left

        if lanelet_msg.HasField('adjacent_right'):
            lanelet.adj_right = lanelet_msg.adjacent_right

        if lanelet_msg.HasField('adjacent_left_dir'):
            left_dir = lanelet_pb2.DrivingDirEnum.DrivingDir.Name(lanelet_msg.adjacent_left_dir)
            lanelet.adj_left_same_direction = left_dir == 'SAME'

        if lanelet_msg.HasField('adjacent_right_dir'):
            right_dir = lanelet_pb2.DrivingDirEnum.DrivingDir.Name(lanelet_msg.adjacent_right_dir)
            lanelet.adj_right_same_direction = right_dir == 'SAME'

        if lanelet_msg.HasField('stop_line'):
            stop_line = StopLineFactory.create_from_message(lanelet_msg.stop_line)
            lanelet.stop_line = stop_line

        lanelet_types = set()
        for lanelet_type in lanelet_msg.lanelet_types:
            lanelet_types.add(LaneletType[lanelet_pb2.LaneletTypeEnum.LaneletType.Name(lanelet_type)])
        lanelet.lanelet_type = lanelet_types

        user_one_ways = set()
        for user_one_way in lanelet_msg.user_one_ways:
            user_one_ways.add(RoadUser[lanelet_pb2.RoadUserEnum.RoadUser.Name(user_one_way)])
        lanelet.user_one_way = user_one_ways

        user_bidirectionals = set()
        for user_bidirectional in lanelet_msg.user_bidirectionals:
            user_bidirectionals.add(RoadUser[lanelet_pb2.RoadUserEnum.RoadUser.Name(user_bidirectional)])
        lanelet.user_bidirectional = user_bidirectionals

        lanelet.traffic_signs = {ts for ts in lanelet_msg.traffic_sign_refs}
        lanelet.traffic_lights = {tl for tl in lanelet_msg.traffic_light_refs}

        return lanelet_msg


class BoundFactory:

    @classmethod
    def create_from_message(cls, bound_msg: lanelet_pb2.Bound) -> Tuple[np.ndarray, Union[LineMarking, None]]:
        points = list()
        for point_msg in bound_msg.points:
            point = PointFactory.create_from_message(point_msg)
            points.append(point)

        line_marking = None
        if bound_msg.HasField('line_marking'):
            line_marking = lanelet_pb2.LineMarkingEnum.LineMarking.Name(bound_msg.line_marking)

        return np.array(points), line_marking


class StopLineFactory:

    @classmethod
    def create_from_message(cls, stop_line_msg: lanelet_pb2.StopLine) -> StopLine:
        points = list()
        for point_msg in stop_line_msg.points:
            point = PointFactory.create_from_message(point_msg)
            points.append(point)

        line_marking = lanelet_pb2.LineMarkingEnum.LineMarking.Name(stop_line_msg.line_making)

        stop_line = StopLine(points[0], points[1], line_marking)

        stop_line.traffic_sign_ref = {ref for ref in stop_line_msg.traffic_sign_refs}
        stop_line.traffic_light_ref = {ref for ref in stop_line_msg.traffic_light_refs}

        return stop_line


class TrafficSignFactory:

    @classmethod
    def create_from_message(cls, traffic_sign_msg: traffic_sign_pb2.TrafficSign):
        traffic_sign_id = traffic_sign_msg.traffic_sign_id

        traffic_sign_elements = list()
        for traffic_sign_element_msg in traffic_sign_msg.traffic_sign_elements:
            traffic_sign_element = TrafficSignElementFactory.create_from_message(traffic_sign_element_msg)
            traffic_sign_elements.append(traffic_sign_element)

        first_occurrences = set()  # TODO

        traffic_sign = TrafficSign(traffic_sign_id, traffic_sign_elements, first_occurrences)

        if traffic_sign_msg.HasField('position'):
            point = PointFactory.create_from_message(traffic_sign_msg.position)
            traffic_sign.position = point

        if traffic_sign_msg.HasField('virtual'):
            traffic_sign.virtual = traffic_sign_msg.virtual

        return traffic_sign


class TrafficSignElementFactory:

    @classmethod
    def create_from_message(cls, traffic_sign_element_msg: traffic_sign_pb2.TrafficSignElement):
        if traffic_sign_element_msg.HasField('germany_element_id'):
            element_id = TrafficSignIDGermany[traffic_sign_pb2.TrafficSignIDGermanyEnum.TrafficSignIDGermany.Name(
                    traffic_sign_element_msg.germany_element_id)]
        elif traffic_sign_element_msg.HasField('zamunda_element_id'):
            element_id = TrafficSignIDZamunda[traffic_sign_pb2.TrafficSignIDZamundaEnum.TrafficSignIDZamunda.Name(
                    traffic_sign_element_msg.zamunda_element_id)]
        elif traffic_sign_element_msg.HasField('usa_element_id'):
            element_id = TrafficSignIDUsa[traffic_sign_pb2.TrafficSignIDUsaEnum.TrafficSignIDUsa.Name(
                    traffic_sign_element_msg.usa_element_id)]
        elif traffic_sign_element_msg.HasField('china_element_id'):
            element_id = TrafficSignIDChina[traffic_sign_pb2.TrafficSignIDChinaEnum.TrafficSignChina.Name(
                    traffic_sign_element_msg.china_element_id)]
        elif traffic_sign_element_msg.HasField('spain_element_id'):
            element_id = TrafficSignIDSpain[traffic_sign_pb2.TrafficSignIDSpainEnum.TrafficSignIDSpain.Name(
                    traffic_sign_element_msg.spain_element_id)]
        elif traffic_sign_element_msg.HasField('russia_element_id'):
            element_id = TrafficSignIDRussia[traffic_sign_pb2.TrafficSignIDRussiaEnum.TrafficSignIDRussia.Name(
                    traffic_sign_element_msg.russia_element_id)]
        elif traffic_sign_element_msg.HasField('argentina_element_id'):
            element_id = TrafficSignIDArgentina[traffic_sign_pb2.TrafficSignIDArgentinaEnum.TrafficSignIDArgentina.Name(
                    traffic_sign_element_msg.argentina_element_id)]
        elif traffic_sign_element_msg.HasField('belgium_element_id'):
            element_id = TrafficSignIDBelgium[traffic_sign_pb2.TrafficSignIDBelgiumEnum.TrafficSignIDBelgium.Name(
                    traffic_sign_element_msg.belgium_element_id)]
        elif traffic_sign_element_msg.HasField('france_element_id'):
            element_id = TrafficSignIDFrance[traffic_sign_pb2.TrafficSignIDFranceEnum.TrafficSignIDFrance.Name(
                    traffic_sign_element_msg.france_element_id)]
        elif traffic_sign_element_msg.HasField('greece_element_id'):
            element_id = TrafficSignIDGreece[traffic_sign_pb2.TrafficSignIDGreeceEnum.TrafficSignIDGreece.Name(
                    traffic_sign_element_msg.greece_element_id)]
        elif traffic_sign_element_msg.HasField('croatia_element_id'):
            element_id = TrafficSignIDCroatia[traffic_sign_pb2.TrafficSignIDCroatiaEnum.TrafficSignIDCroatia.Name(
                    traffic_sign_element_msg.croatia_element_id)]
        elif traffic_sign_element_msg.HasField('italy_element_id'):
            element_id = TrafficSignIDItaly[traffic_sign_pb2.TrafficSignIDItalyEnum.TrafficSignIDItaly.Name(
                    traffic_sign_element_msg.italy_element_id)]
        else:
            element_id = TrafficSignIDPuertoRico[
                traffic_sign_pb2.TrafficSignIDPuertoRicoEnum.TrafficSignIDPuertoRico.Name(
                        traffic_sign_element_msg.puerto_rico_element_id)]

        traffic_sign_element = TrafficSignElement(element_id)

        traffic_sign_element.additional_values = [a for a in traffic_sign_element_msg.additional_values]

        return traffic_sign_element


class TrafficLightFactory:

    @classmethod
    def create_from_message(cls, traffic_light_msg: traffic_light_pb2.TrafficLight) -> TrafficLight:
        pass


class CycleFactory:

    @classmethod
    def create_from_message(cls, cycle_msg: traffic_light_pb2.Cycle) \
            -> Tuple[List[TrafficLightCycleElement], Union[int, None]]:
        pass


class CycleElementFactory:

    @classmethod
    def create_from_message(cls, cycle_element_msg: traffic_light_pb2.CycleElement) -> TrafficLightCycleElement:
        return TrafficLightCycleElement(cycle_element_msg.duration, cycle_element_msg.color)


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
        for shape_msg in shape_group_msg:
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
    def create_from_message(cls, float_interval_msg: util_pb2.FloatInterval) -> Interval:
        return Interval(float_interval_msg.start, float_interval_msg.end)


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
    def create_from_message(cls, float_exact_or_interval_msg: util_pb2.FloatExactOrInterval) \
            -> Union[float, Interval]:
        if float_exact_or_interval_msg.HasField('exact'):
            return float_exact_or_interval_msg.exact

        if float_exact_or_interval_msg.HasField('interval'):
            return FloatIntervalFactory.create_from_message(float_exact_or_interval_msg.interval)


class IntegerListFactory:

    @classmethod
    def create_from_message(cls, integer_list_msg: util_pb2.IntegerList) -> List[int]:
        return [i for i in integer_list_msg]


class FloatListFactory:

    @classmethod
    def create_from_message(cls, float_list_msg: util_pb2.FloatList) -> List[float]:
        return [f for f in float_list_msg]
