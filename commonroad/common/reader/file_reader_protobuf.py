import datetime
import logging
import re
from typing import Tuple, List, Set, Union, Dict

import numpy as np

from commonroad.common.reader.file_reader_interface import FileReader
from commonroad.common.util import Interval, AngleInterval, Path_T, Time
from commonroad.geometry.shape import Rectangle, Circle, Polygon, Shape, ShapeGroup
from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import PlanningProblemSet, PlanningProblem
from commonroad.prediction.prediction import Occupancy, TrajectoryPrediction, SetBasedPrediction
from commonroad.scenario.intersection import Intersection, IntersectionIncomingElement
from commonroad.scenario.lanelet import LaneletNetwork, Lanelet
from commonroad.common.common_lanelet import RoadUser, StopLine, LineMarking, LaneletType
from commonroad.scenario.obstacle import StaticObstacle, DynamicObstacle, EnvironmentObstacle, PhantomObstacle, \
    SignalState, ObstacleType
from commonroad.scenario.scenario import Scenario, ScenarioID, Tag, GeoTransformation, Location, Environment, \
    TimeOfDay, \
    Weather, Underground
from commonroad.scenario.state import InitialState, TraceState, CustomState, SpecificStateClasses
from commonroad.scenario.traffic_sign import TrafficSignElement, TrafficSignIDGermany, TrafficSignIDZamunda, \
    TrafficSignIDUsa, TrafficSignIDChina, TrafficSignIDSpain, TrafficSignIDRussia, TrafficSignIDArgentina, \
    TrafficSignIDBelgium, TrafficSignIDFrance, TrafficSignIDGreece, TrafficSignIDCroatia, TrafficSignIDItaly, \
    TrafficSignIDPuertoRico, TrafficSign
from commonroad.scenario.traffic_light import TrafficLightState, TrafficLightDirection, \
    TrafficLightCycleElement,  TrafficLight, TrafficLightCycle
from commonroad.scenario.trajectory import Trajectory
from commonroad.scenario_definition.protobuf_format.generated_scripts import commonroad_pb2, util_pb2, \
    phantom_obstacle_pb2
from commonroad.scenario_definition.protobuf_format.generated_scripts import lanelet_pb2, planning_problem_pb2, \
    traffic_sign_pb2, scenario_tags_pb2, environment_obstacle_pb2, obstacle_pb2, intersection_pb2, \
    dynamic_obstacle_pb2, \
    static_obstacle_pb2, traffic_light_pb2, location_pb2

logger = logging.getLogger(__name__)


class ProtobufFileReader(FileReader):
    """
    Reader for CommonRoad file stored in protobuf format.
    """

    def __init__(self, filename: Path_T):
        super().__init__(filename)

    def open(self, lanelet_assignment: bool = False) -> Tuple[Scenario, PlanningProblemSet]:
        """
        Reads a CommonRoad file in protobuf format.

        :param lanelet_assignment: Activates calculation of lanelets occupied by obstacles
        :return: Scenario and planning problems
        """
        with open(self._filename, "rb") as file:
            commonroad_msg = commonroad_pb2.CommonRoad()
            commonroad_msg.ParseFromString(file.read())

        return CommonRoadFactory.create_from_message(commonroad_msg, lanelet_assignment)

    def open_lanelet_network(self) -> LaneletNetwork:
        """
        Reads a CommonRoad file in protobuf format.

        :return: Lanelet network
        """
        with open(self._filename, "rb") as file:
            commonroad_msg = commonroad_pb2.CommonRoad()
            commonroad_msg.ParseFromString(file.read())

        scenario, _ = CommonRoadFactory.create_from_message(commonroad_msg, False)

        return scenario.lanelet_network


class CommonRoadFactory:

    @classmethod
    def create_from_message(cls, commonroad_msg: commonroad_pb2.CommonRoad, lanelet_assignment: bool) \
            -> Tuple[Scenario, PlanningProblemSet]:
        scenario_information_msg = commonroad_msg.information
        common_road_version, benchmark_id, _, author, affiliation, source, time_step_size = \
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
            scenario.add_objects(lanelet)

        for traffic_sign_msg in commonroad_msg.traffic_signs:
            traffic_sign = TrafficSignFactory.create_from_message(traffic_sign_msg)
            scenario.add_objects(traffic_sign, set())

        for traffic_light_msg in commonroad_msg.traffic_lights:
            traffic_light = TrafficLightFactory.create_from_message(traffic_light_msg)
            scenario.add_objects(traffic_light, set())

        for intersection_msg in commonroad_msg.intersections:
            intersection = IntersectionFactory.create_from_message(intersection_msg)
            scenario.add_objects(intersection)

        for static_obstacle_msg in commonroad_msg.static_obstacles:
            static_obstacle = StaticObstacleFactory \
                .create_from_message(static_obstacle_msg, scenario.lanelet_network, lanelet_assignment)
            scenario.add_objects(static_obstacle)

        for dynamic_obstacle_msg in commonroad_msg.dynamic_obstacles:
            dynamic_obstacle = DynamicObstacleFactory \
                .create_from_message(dynamic_obstacle_msg, scenario.lanelet_network, lanelet_assignment)
            scenario.add_objects(dynamic_obstacle)

        for environment_obstacle_msg in commonroad_msg.environment_obstacles:
            environment_obstacle = EnvironmentObstacleFactory.create_from_message(environment_obstacle_msg)
            scenario.add_objects(environment_obstacle)

        for phantom_obstacle_msg in commonroad_msg.phantom_obstacles:
            phantom_obstacle = PhantomObstacleFactory.create_from_message(phantom_obstacle_msg)
            scenario.add_objects(phantom_obstacle)

        planning_problem_set = PlanningProblemSet()
        for planning_problem_msg in commonroad_msg.planning_problems:
            planning_problem = PlanningProblemFactory.create_from_message(planning_problem_msg)
            planning_problem_set.add_planning_problem(planning_problem)

        return scenario, planning_problem_set


class ScenarioInformationFactory:

    @classmethod
    def create_from_message(cls, scenario_information_msg: commonroad_pb2.ScenarioInformation):
        common_road_version = scenario_information_msg.common_road_version
        benchmark_id = scenario_information_msg.benchmark_id
        date = TimeStampFactory.create_from_message(scenario_information_msg.date, cr_time=False)
        author = scenario_information_msg.author
        affiliation = scenario_information_msg.affiliation
        source = scenario_information_msg.source
        time_step_size = scenario_information_msg.time_step_size

        return common_road_version, benchmark_id, date, author, affiliation, source, time_step_size


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
    def create_from_message(cls, environment_msg: location_pb2.Environment) -> Environment:
        environment = Environment()

        if environment_msg.HasField('time'):
            environment.time = TimeStampFactory.create_from_message(environment_msg.time, cr_time=True)

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

        predecessor = list(lanelet_msg.predecessors)
        lanelet.predecessor = predecessor

        successor = list(lanelet_msg.successors)
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

        if left_line_marking is not None:
            lanelet.line_marking_left_vertices = left_line_marking

        if right_line_marking is not None:
            lanelet.line_marking_right_vertices = right_line_marking

        if lanelet_msg.HasField('stop_line'):
            stop_line = StopLineFactory.create_from_message(lanelet_msg.stop_line)
            lanelet.stop_line = stop_line

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

        return lanelet


class BoundFactory:

    @classmethod
    def create_from_message(cls, bound_msg: lanelet_pb2.Bound) -> Tuple[np.ndarray, Union[LineMarking, None]]:
        points = list()
        for point_msg in bound_msg.points:
            point = PointFactory.create_from_message(point_msg)
            points.append(point)

        line_marking = None
        if bound_msg.HasField('line_marking'):
            line_marking = LineMarking[lanelet_pb2.LineMarkingEnum.LineMarking.Name(bound_msg.line_marking)]

        return np.array(points), line_marking


class StopLineFactory:

    @classmethod
    def create_from_message(cls, stop_line_msg: lanelet_pb2.StopLine) -> StopLine:
        points = list()
        for point_msg in stop_line_msg.points:
            point = PointFactory.create_from_message(point_msg)
            points.append(point)

        line_marking = LineMarking[lanelet_pb2.LineMarkingEnum.LineMarking.Name(stop_line_msg.line_marking)]

        stop_line = StopLine(points[0], points[1], line_marking)

        traffic_sign_ref = None
        if stop_line_msg.traffic_sign_refs:
            traffic_sign_ref = {ref for ref in stop_line_msg.traffic_sign_refs}
        stop_line.traffic_sign_ref = traffic_sign_ref

        traffic_light_ref = None
        if stop_line_msg.traffic_light_refs:
            traffic_light_ref = {ref for ref in stop_line_msg.traffic_light_refs}
        stop_line.traffic_light_ref = traffic_light_ref

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
            element_id = TrafficSignIDChina[traffic_sign_pb2.TrafficSignIDChinaEnum.TrafficSignIDChina.Name(
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

        traffic_sign_element.additional_values = list(traffic_sign_element_msg.additional_values)

        return traffic_sign_element


class TrafficLightFactory:

    @classmethod
    def create_from_message(cls, traffic_light_msg: traffic_light_pb2.TrafficLight) -> TrafficLight:
        traffic_light_id = traffic_light_msg.traffic_light_id

        cycle_elements = list()
        for cycle_element_msg in traffic_light_msg.cycle_elements:
            cycle_element = CycleElementFactory.create_from_message(cycle_element_msg)
            cycle_elements.append(cycle_element)

        point = PointFactory.create_from_message(traffic_light_msg.position)

        traffic_light_cycle = TrafficLightCycle(cycle_elements)
        if traffic_light_msg.HasField('time_offset'):
            traffic_light_cycle.time_offset = traffic_light_msg.time_offset

        # extracting the color list from the traffic light cycle
        color = []
        if len(cycle_elements) > 0:
            for element in cycle_elements:
                color.append(element.state)

        traffic_light = TrafficLight(traffic_light_id, point, traffic_light_cycle, color)

        if traffic_light_msg.HasField('direction'):
            traffic_light.direction = TrafficLightDirection[
                traffic_light_pb2.TrafficLightDirectionEnum.TrafficLightDirection.Name(traffic_light_msg.direction)]

        if traffic_light_msg.HasField('active'):
            traffic_light.active = traffic_light_msg.active

        return traffic_light


class CycleElementFactory:

    @classmethod
    def create_from_message(cls, cycle_element_msg: traffic_light_pb2.CycleElement) -> TrafficLightCycleElement:
        color = TrafficLightState[traffic_light_pb2.TrafficLightStateEnum.TrafficLightState.Name(
                cycle_element_msg.color)]
        duration = cycle_element_msg.duration
        return TrafficLightCycleElement(color, duration)


class IntersectionFactory:

    @classmethod
    def create_from_message(cls, intersection_msg: intersection_pb2.Intersection) -> Intersection:
        intersection_id = intersection_msg.intersection_id

        incomings = list()
        for incoming_msg in intersection_msg.incomings:
            incoming = IncomingFactory.create_from_message(incoming_msg)
            incomings.append(incoming)

        intersection = Intersection(intersection_id, incomings)

        intersection.crossings = set(intersection_msg.crossing_lanelets)

        return intersection


class IncomingFactory:

    @classmethod
    def create_from_message(cls, incoming_msg: intersection_pb2.Incoming) -> IntersectionIncomingElement:
        incoming = IntersectionIncomingElement(incoming_msg.incoming_id)

        incoming.incoming_lanelets = set(incoming_msg.incoming_lanelets)
        incoming.successors_right = set(incoming_msg.successors_right)
        incoming.successors_straight = set(incoming_msg.successors_straight)
        incoming.successors_left = set(incoming_msg.successors_left)

        if incoming_msg.HasField('is_left_of'):
            incoming.left_of = incoming_msg.is_left_of

        return incoming


class StaticObstacleFactory:

    @classmethod
    def create_from_message(cls, static_obstacle_msg: static_obstacle_pb2.StaticObstacle,
                            lanelet_network: LaneletNetwork, lanelet_assignment: bool) -> StaticObstacle:
        static_obstacle_id = static_obstacle_msg.static_obstacle_id

        obstacle_type = ObstacleType[obstacle_pb2.ObstacleTypeEnum.ObstacleType.Name(static_obstacle_msg.obstacle_type)]

        shape = ShapeFactory.create_from_message(static_obstacle_msg.shape)

        initial_state = StateFactory.create_from_message(static_obstacle_msg.initial_state, is_initial_state=True)

        static_obstacle = StaticObstacle(static_obstacle_id, obstacle_type, shape, initial_state)

        if lanelet_assignment is True:
            rotated_shape = shape.rotate_translate_local(initial_state.position, initial_state.orientation)
            initial_shape_lanelet_ids = set(lanelet_network.find_lanelet_by_shape(rotated_shape))
            initial_center_lanelet_ids = set(lanelet_network.find_lanelet_by_position([initial_state.position])[0])
            for l_id in initial_shape_lanelet_ids:
                lanelet_network.find_lanelet_by_id(l_id).add_static_obstacle_to_lanelet(obstacle_id=static_obstacle_id)
        else:
            initial_center_lanelet_ids = None
            initial_shape_lanelet_ids = None
        static_obstacle.initial_center_lanelet_ids = initial_center_lanelet_ids
        static_obstacle.initial_shape_lanelet_ids = initial_shape_lanelet_ids

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
    def create_from_message(cls, dynamic_obstacle_msg: dynamic_obstacle_pb2.DynamicObstacle,
                            lanelet_network: LaneletNetwork, lanelet_assignment: bool) -> DynamicObstacle:
        dynamic_obstacle_id = dynamic_obstacle_msg.dynamic_obstacle_id

        obstacle_type = ObstacleType[
            obstacle_pb2.ObstacleTypeEnum.ObstacleType.Name(dynamic_obstacle_msg.obstacle_type)]

        shape = ShapeFactory.create_from_message(dynamic_obstacle_msg.shape)

        initial_state = StateFactory.create_from_message(dynamic_obstacle_msg.initial_state, is_initial_state=True)

        initial_center_lanelet_ids = set()
        initial_shape_lanelet_ids = set()

        prediction = None
        if dynamic_obstacle_msg.HasField('trajectory_prediction'):
            if lanelet_assignment is True:
                rotated_shape = shape.rotate_translate_local(initial_state.position, initial_state.orientation)
                initial_shape_lanelet_ids = set(lanelet_network.find_lanelet_by_shape(rotated_shape))
                initial_center_lanelet_ids = set(lanelet_network.find_lanelet_by_position([initial_state.position])[0])
                for l_id in initial_shape_lanelet_ids:
                    lanelet_network.find_lanelet_by_id(l_id) \
                        .add_dynamic_obstacle_to_lanelet(obstacle_id=dynamic_obstacle_id,
                                                         time_step=initial_state.time_step)
            else:
                initial_shape_lanelet_ids = None
                initial_center_lanelet_ids = None

            prediction = TrajectoryPredictionFactory \
                .create_from_message(dynamic_obstacle_msg.trajectory_prediction, initial_state, lanelet_network,
                                     dynamic_obstacle_id, lanelet_assignment)
        elif dynamic_obstacle_msg.HasField('set_based_prediction'):
            prediction = SetBasedPredictionFactory.create_from_message(dynamic_obstacle_msg.set_based_prediction)

        dynamic_obstacle = DynamicObstacle(dynamic_obstacle_id, obstacle_type, shape, initial_state, prediction)

        dynamic_obstacle.initial_center_lanelet_ids = initial_center_lanelet_ids
        dynamic_obstacle.initial_shape_lanelet_ids = initial_shape_lanelet_ids

        if dynamic_obstacle_msg.HasField('initial_signal_state'):
            dynamic_obstacle.initial_signal_state = SignalStateFactory \
                .create_from_message(dynamic_obstacle_msg.initial_signal_state)

        signal_states = list()
        for signal_state_msg in dynamic_obstacle_msg.signal_series:
            signal_state = SignalStateFactory.create_from_message(signal_state_msg)
            signal_states.append(signal_state)
        dynamic_obstacle.signal_series = signal_states

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


class StateFactory:

    @classmethod
    def create_from_message(cls, state_msg: obstacle_pb2.State, is_initial_state: bool = False) -> TraceState:
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
    def _fill_state(cls, state: TraceState, state_msg: obstacle_pb2.State, attrs: List[str]) -> bool:
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
    def create_from_message(cls, signal_state_msg: obstacle_pb2.SignalState) -> SignalState:
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
    def create_from_message(cls, trajectory_prediction_msg: obstacle_pb2.TrajectoryPrediction,
                            initial_state: InitialState, lanelet_network: LaneletNetwork, obstacle_id: int,
                            lanelet_assignment: bool) -> TrajectoryPrediction:
        trajectory = TrajectoryFactory.create_from_message(trajectory_prediction_msg.trajectory)

        shape = ShapeFactory.create_from_message(trajectory_prediction_msg.shape)

        trajectory_prediction = TrajectoryPrediction(trajectory, shape)

        if lanelet_assignment is True:
            shape_lanelet_assignment = cls.find_obstacle_shape_lanelets(initial_state, trajectory.state_list,
                                                                        lanelet_network, obstacle_id, shape)
            center_lanelet_assignment = cls.find_obstacle_center_lanelets(initial_state, trajectory.state_list,
                                                                          lanelet_network)
        else:
            shape_lanelet_assignment = None
            center_lanelet_assignment = None

        trajectory_prediction.center_lanelet_assignment = center_lanelet_assignment
        trajectory_prediction.shape_lanelet_assignment = shape_lanelet_assignment

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

        return PlanningProblem(planning_problem_id, initial_state, goal_region)


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
