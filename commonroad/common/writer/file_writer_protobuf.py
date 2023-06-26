import datetime
import re
from typing import Set, Union, List
import logging

import numpy as np
from google.protobuf.message import DecodeError

from commonroad.common.util import Interval, FileFormat, Time
from commonroad.common.writer.file_writer_interface import FileWriter, OverwriteExistingFile
from commonroad.geometry.shape import Rectangle, Circle, Polygon, ShapeGroup, Shape
from commonroad.planning.planning_problem import PlanningProblemSet, PlanningProblem
from commonroad.prediction.prediction import Occupancy, TrajectoryPrediction, SetBasedPrediction
from commonroad.scenario_definition.protobuf_format.generated_scripts import commonroad_pb2, util_pb2, \
    phantom_obstacle_pb2
from commonroad.scenario_definition.protobuf_format.generated_scripts import lanelet_pb2, planning_problem_pb2, \
    traffic_sign_pb2, scenario_tags_pb2, environment_obstacle_pb2, obstacle_pb2, intersection_pb2, \
    dynamic_obstacle_pb2, static_obstacle_pb2, traffic_light_pb2, location_pb2
from commonroad.scenario.intersection import Intersection, IntersectionIncomingElement
from commonroad.scenario.lanelet import Lanelet
from commonroad.common.common_lanelet import StopLine, LineMarking
from commonroad.scenario.obstacle import StaticObstacle, DynamicObstacle, EnvironmentObstacle, SignalState, \
    PhantomObstacle
from commonroad.scenario.scenario import Scenario, Tag, Location, GeoTransformation, Environment
from commonroad.scenario.traffic_sign import TrafficSign, TrafficSignElement, \
    TrafficSignIDGermany, TrafficSignIDFrance, TrafficSignIDZamunda, TrafficSignIDUsa, TrafficSignIDChina, \
    TrafficSignIDSpain, TrafficSignIDRussia, TrafficSignIDArgentina, TrafficSignIDBelgium, TrafficSignIDGreece, \
    TrafficSignIDCroatia, TrafficSignIDItaly
from commonroad.scenario.traffic_light import TrafficLightCycleElement, TrafficLight
from commonroad.scenario.trajectory import Trajectory
from commonroad.scenario.state import State

logger = logging.getLogger(__name__)


class ProtobufFileWriter(FileWriter):
    """
    Writes CommonRoad files in protobuf format.
    """

    def __init__(self, scenario: Scenario, planning_problem_set: PlanningProblemSet, author: str = None,
                 affiliation: str = None, source: str = None, tags: Set[Tag] = None, location: Location = None,
                 decimal_precision: int = 4):
        super().__init__(scenario, planning_problem_set, author, affiliation, source, tags, location, decimal_precision)

        self._commonroad_msg = commonroad_pb2.CommonRoad()

    def _write_header(self):
        """
        Stores all information about scenario and planning problems as protobuf message.

        :return:
        """
        information_msg = ScenarioInformationMessage.create_message(self.scenario.scenario_id.scenario_version,
                                                                    str(self.scenario.scenario_id), self._author,
                                                                    self._affiliation, self._source, self.scenario.dt)

        self._commonroad_msg.information.CopyFrom(information_msg)

    def _add_all_objects_from_scenario(self):
        """
        Stores all scenario objects as protobuf message.

        :return:
        """
        scenario_tags = ScenarioTagsMessage.create_message(list(self.scenario.tags))
        self._commonroad_msg.scenario_tags.CopyFrom(scenario_tags)

        if self.location is not None:
            location_msg = LocationMessage.create_message(self.location)
        else:
            location_msg = LocationMessage.create_message(Location())
            logger.warning("Default location will be written to protobuf!")
        self._commonroad_msg.location.CopyFrom(location_msg)

        for lanelet in self.scenario.lanelet_network.lanelets:
            lanelet_msg = LaneletMessage.create_message(lanelet)
            self._commonroad_msg.lanelets.append(lanelet_msg)

        for traffic_sign in self.scenario.lanelet_network.traffic_signs:
            traffic_sign_msg = TrafficSignMessage.create_message(traffic_sign)
            self._commonroad_msg.traffic_signs.append(traffic_sign_msg)

        for traffic_light in self.scenario.lanelet_network.traffic_lights:
            traffic_light_msg = TrafficLightMessage.create_message(traffic_light)
            self._commonroad_msg.traffic_lights.append(traffic_light_msg)

        for intersection in self.scenario.lanelet_network.intersections:
            intersection_msg = IntersectionMessage.create_message(intersection)
            self._commonroad_msg.intersections.append(intersection_msg)

        for static_obstacle in self.scenario.static_obstacles:
            static_obstacle_msg = StaticObstacleMessage.create_message(static_obstacle)
            self._commonroad_msg.static_obstacles.append(static_obstacle_msg)

        for dynamic_obstacle in self.scenario.dynamic_obstacles:
            dynamic_obstacle_msg = DynamicObstacleMessage.create_message(dynamic_obstacle)
            self._commonroad_msg.dynamic_obstacles.append(dynamic_obstacle_msg)

        for environment_obstacle in self.scenario.environment_obstacle:
            environment_obstacle_msg = EnvironmentObstacleMessage.create_message(environment_obstacle)
            self._commonroad_msg.environment_obstacles.append(environment_obstacle_msg)

        for phantom_obstacle in self.scenario.phantom_obstacle:
            phantom_obstacle_msg = PhantomObstacleMessage.create_message(phantom_obstacle)
            self._commonroad_msg.phantom_obstacles.append(phantom_obstacle_msg)

    def _add_all_planning_problems_from_planning_problem_set(self):
        """
        Stores all planning problems as protobuf message.

        :return:
        """
        for planning_problem in self.planning_problem_set.planning_problem_dict.values():
            planning_problem_msg = PlanningProblemMessage.create_message(planning_problem)
            self._commonroad_msg.planning_problems.append(planning_problem_msg)

    def _serialize_write_msg(self, filename: str):
        """
        Serializes the message of type commonroad and writes into file.

        :param filename: Name of file
        :return:
        """
        with open(filename, "wb") as f:
            f.write(self._commonroad_msg.SerializeToString())

    def _get_suffix(self) -> str:
        return FileFormat.PROTOBUF.value

    def write_to_file(self, filename: Union[str, None] = None,
                      overwrite_existing_file: OverwriteExistingFile = OverwriteExistingFile.ASK_USER_INPUT,
                      check_validity: bool = False):
        """
        Writes message of type commonroad into file.

        :param filename: Name of file
        :param overwrite_existing_file: Mode of writing
        :param check_validity: Validity checking before writing
        :return:
        """
        filename = self._handle_file_path(filename, overwrite_existing_file)
        if not filename:
            return

        self._commonroad_msg = commonroad_pb2.CommonRoad()

        self._write_header()
        self._add_all_objects_from_scenario()
        self._add_all_planning_problems_from_planning_problem_set()

        if check_validity:
            pass

        self._serialize_write_msg(filename)

    def write_scenario_to_file(self, filename: Union[str, None] = None,
                               overwrite_existing_file: OverwriteExistingFile = OverwriteExistingFile.ASK_USER_INPUT):
        """
        Writes scenario as protobuf message into file.

        :param filename: Name of file
        :param overwrite_existing_file: Mode of writing
        :return:
        """
        filename = self._handle_file_path(filename, overwrite_existing_file)
        if not filename:
            return

        self._commonroad_msg = commonroad_pb2.CommonRoad()

        self._write_header()
        self._add_all_objects_from_scenario()

        self._serialize_write_msg(filename)

    @staticmethod
    def check_validity_of_commonroad_file(commonroad_str: Union[str, bytes]) -> bool:
        """
        Checks validity of CommonRoad message/file.

        :param commonroad_str: Commonroad instance in form of binary string
        :return: Valid or not
        """
        try:
            commonroad_msg = commonroad_pb2.CommonRoad()
            commonroad_msg.ParseFromString(commonroad_str)
            return True
        except DecodeError:
            return False


class ScenarioInformationMessage:

    @classmethod
    def create_message(cls, commonroad_version: str, benchmark_id: str, author: str, affiliation: str, source: str,
                       time_step_size: float) -> commonroad_pb2.ScenarioInformation:
        scenario_information_msg = commonroad_pb2.ScenarioInformation()
        scenario_information_msg.common_road_version = commonroad_version
        scenario_information_msg.benchmark_id = benchmark_id
        time_stamp_msg = TimeStampMessage.create_message(datetime.datetime.today())
        scenario_information_msg.date.CopyFrom(time_stamp_msg)
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
        if location.environment is not None:
            environment_msg = EnvironmentMessage.create_message(location.environment)
            location_msg.environment.CopyFrom(environment_msg)

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
    def create_message(cls, environment: Environment) -> location_pb2.Environment:
        environment_msg = location_pb2.Environment()

        if environment.time is not None:
            time_stamp_msg = TimeStampMessage.create_message(environment.time)
            environment_msg.time.CopyFrom(time_stamp_msg)
        if environment.time_of_day is not None:
            environment_msg.time_of_day = location_pb2.TimeOfDayEnum.TimeOfDay.Value(environment.time_of_day.name)
        if environment.weather is not None:
            environment_msg.weather = location_pb2.WeatherEnum.Weather.Value(environment.weather.name)
        if environment.underground is not None:
            environment_msg.underground = location_pb2.UndergroundEnum.Underground.Value(environment.underground.name)

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

        if lanelet.adj_left_same_direction is not None:
            if lanelet.adj_left_same_direction:
                lanelet_msg.adjacent_left_dir = lanelet_pb2.DrivingDirEnum.DrivingDir.Value('SAME')
            else:
                lanelet_msg.adjacent_left_dir = lanelet_pb2.DrivingDirEnum.DrivingDir.Value('OPPOSITE')

        if lanelet.adj_right_same_direction is not None:
            if lanelet.adj_right_same_direction:
                lanelet_msg.adjacent_right_dir = lanelet_pb2.DrivingDirEnum.DrivingDir.Value('SAME')
            else:
                lanelet_msg.adjacent_right_dir = lanelet_pb2.DrivingDirEnum.DrivingDir.Value('OPPOSITE')

        if lanelet.stop_line is not None:
            stop_line_msg = StopLineMessage.create_message(lanelet.stop_line)
            lanelet_msg.stop_line.CopyFrom(stop_line_msg)

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
            point_msg = PointMessage.create_message(stop_line.start)
            stop_line_msg.points.append(point_msg)

        if stop_line.end is not None:
            point_msg = PointMessage.create_message(stop_line.end)
            stop_line_msg.points.append(point_msg)

        stop_line_msg.line_marking = lanelet_pb2.LineMarkingEnum.LineMarking.Value(stop_line.line_marking.name)

        if stop_line.traffic_sign_ref is not None:
            for ts_ref in stop_line.traffic_sign_ref:
                stop_line_msg.traffic_sign_refs.append(ts_ref)

        if stop_line.traffic_light_ref is not None:
            for tl_ref in stop_line.traffic_light_ref:
                stop_line_msg.traffic_light_refs.append(tl_ref)

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


class TrafficSignElementMessage:

    @classmethod
    def create_message(cls, traffic_sign_element: TrafficSignElement) -> traffic_sign_pb2.TrafficSignElement:
        traffic_sign_element_msg = traffic_sign_pb2.TrafficSignElement()

        element_id = traffic_sign_element.traffic_sign_element_id
        if isinstance(element_id, TrafficSignIDGermany):
            traffic_sign_element_msg.germany_element_id = traffic_sign_pb2.TrafficSignIDGermanyEnum \
                .TrafficSignIDGermany.Value(element_id.name)
        elif isinstance(element_id, TrafficSignIDZamunda):
            traffic_sign_element_msg.zamunda_element_id = traffic_sign_pb2.TrafficSignIDZamundaEnum \
                .TrafficSignIDZamunda.Value(element_id.name)
        elif isinstance(element_id, TrafficSignIDUsa):
            traffic_sign_element_msg.usa_element_id = traffic_sign_pb2.TrafficSignIDUsaEnum \
                .TrafficSignIDUsa.Value(element_id.name)
        elif isinstance(element_id, TrafficSignIDChina):
            traffic_sign_element_msg.china_element_id = traffic_sign_pb2.TrafficSignIDChinaEnum \
                .TrafficSignIDChina.Value(element_id.name)
        elif isinstance(element_id, TrafficSignIDSpain):
            traffic_sign_element_msg.spain_element_id = traffic_sign_pb2.TrafficSignIDSpainEnum \
                .TrafficSignIDSpain.Value(element_id.name)
        elif isinstance(element_id, TrafficSignIDRussia):
            traffic_sign_element_msg.russia_element_id = traffic_sign_pb2.TrafficSignIDRussiaEnum \
                .TrafficSignIDRussia.Value(element_id.name)
        elif isinstance(element_id, TrafficSignIDArgentina):
            traffic_sign_element_msg.argentina_element_id = traffic_sign_pb2.TrafficSignIDArgentinaEnum \
                .TrafficSignIDArgentina.Value(element_id.name)
        elif isinstance(element_id, TrafficSignIDBelgium):
            traffic_sign_element_msg.belgium_element_id = traffic_sign_pb2.TrafficSignIDBelgiumEnum \
                .TrafficSignIDBelgium.Value(element_id.name)
        elif isinstance(element_id, TrafficSignIDFrance):
            traffic_sign_element_msg.france_element_id = traffic_sign_pb2.TrafficSignIDFranceEnum \
                .TrafficSignIDFrance.Value(element_id.name)
        elif isinstance(element_id, TrafficSignIDGreece):
            traffic_sign_element_msg.greece_element_id = traffic_sign_pb2.TrafficSignIDGreeceEnum \
                .TrafficSignIDGreece.Value(element_id.name)
        elif isinstance(element_id, TrafficSignIDCroatia):
            traffic_sign_element_msg.croatia_element_id = traffic_sign_pb2.TrafficSignIDCroatiaEnum \
                .TrafficSignIDCroatia.Value(element_id.name)
        elif isinstance(element_id, TrafficSignIDItaly):
            traffic_sign_element_msg.italy_element_id = traffic_sign_pb2.TrafficSignIDItalyEnum \
                .TrafficSignIDItaly.Value(element_id.name)
        else:
            traffic_sign_element_msg.puerto_rico_element_id = traffic_sign_pb2.TrafficSignIDPuertoRicoEnum \
                .TrafficSignIDPuertoRico.Value(element_id.name)

        for additional_value in traffic_sign_element.additional_values:
            traffic_sign_element_msg.additional_values.append(additional_value)

        return traffic_sign_element_msg


class TrafficLightMessage:

    @classmethod
    def create_message(cls, traffic_light: TrafficLight) -> traffic_light_pb2.TrafficLight:
        traffic_light_msg = traffic_light_pb2.TrafficLight()

        traffic_light_msg.traffic_light_id = traffic_light.traffic_light_id

        for cycle_element in traffic_light.traffic_light_cycle.cycle_elements:
            cycle_element_msg = CycleElementMessage.create_message(cycle_element)
            traffic_light_msg.cycle_elements.append(cycle_element_msg)

        if traffic_light.position is not None:
            point_msg = PointMessage.create_message(traffic_light.position)
            traffic_light_msg.position.CopyFrom(point_msg)

        if traffic_light.traffic_light_cycle.time_offset is not None:
            traffic_light_msg.time_offset = traffic_light.traffic_light_cycle.time_offset

        if traffic_light.direction is not None:
            traffic_light_msg.direction = traffic_light_pb2.TrafficLightDirectionEnum.TrafficLightDirection \
                .Value(traffic_light.direction.name)

        if traffic_light.active is not None:
            traffic_light_msg.active = traffic_light.active

        return traffic_light_msg


class CycleElementMessage:

    @classmethod
    def create_message(cls, cycle_element: TrafficLightCycleElement) -> traffic_light_pb2.CycleElement:
        cycle_element_msg = traffic_light_pb2.CycleElement()

        cycle_element_msg.duration = cycle_element.duration
        cycle_element_msg.color = traffic_light_pb2.TrafficLightStateEnum.TrafficLightState \
            .Value(cycle_element.state.name)

        return cycle_element_msg


class IntersectionMessage:

    @classmethod
    def create_message(cls, intersection: Intersection) -> intersection_pb2.Intersection:
        intersection_msg = intersection_pb2.Intersection()

        intersection_msg.intersection_id = intersection.intersection_id

        for incoming in intersection.incomings:
            incoming_msg = IncomingMessage.create_message(incoming)
            intersection_msg.incomings.append(incoming_msg)

        for crossing in intersection.crossings:
            intersection_msg.crossing_lanelets.append(crossing)

        return intersection_msg


class IncomingMessage:

    @classmethod
    def create_message(cls, incoming: IntersectionIncomingElement) -> intersection_pb2.Incoming:
        incoming_msg = intersection_pb2.Incoming()

        incoming_msg.incoming_id = incoming.incoming_id
        for incoming_lanelet in incoming.incoming_lanelets:
            incoming_msg.incoming_lanelets.append(incoming_lanelet)

        for successor_right in incoming.successors_right:
            incoming_msg.successors_right.append(successor_right)

        for successor_straight in incoming.successors_straight:
            incoming_msg.successors_straight.append(successor_straight)

        for successor_left in incoming.successors_left:
            incoming_msg.successors_left.append(successor_left)

        if incoming.left_of is not None:
            incoming_msg.is_left_of = incoming.left_of

        return incoming_msg


class StaticObstacleMessage:

    @classmethod
    def create_message(cls, static_obstacle: StaticObstacle) -> static_obstacle_pb2.StaticObstacle:
        static_obstacle_msg = static_obstacle_pb2.StaticObstacle()

        static_obstacle_msg.static_obstacle_id = static_obstacle.obstacle_id

        static_obstacle_msg.obstacle_type = obstacle_pb2.ObstacleTypeEnum.ObstacleType \
            .Value(static_obstacle.obstacle_type.name)

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
    def create_message(cls, state: State) -> obstacle_pb2.State:
        state_msg = obstacle_pb2.State()

        for attr in state.used_attributes:
            if getattr(state, attr) is None:
                continue

            if attr == 'position':
                if isinstance(state.position, np.ndarray):
                    state_msg.point.CopyFrom(PointMessage.create_message(state.position))
                else:
                    state_msg.shape.CopyFrom(ShapeMessage.create_message(state.position))
            elif attr == 'time_step':
                integer_exact_or_interval_msg = IntegerExactOrIntervalMessage.create_message(state.time_step)
                state_msg.time_step.CopyFrom(integer_exact_or_interval_msg)
            else:
                float_exact_or_interval_msg = FloatExactOrIntervalMessage.create_message(getattr(state, attr))
                getattr(state_msg, StateMessage._map_to_pb_prop(attr)).CopyFrom(float_exact_or_interval_msg)

        return state_msg

    @staticmethod
    def _map_to_pb_prop(prop: str) -> str:
        return re.sub('(?<!^)(?=[A-Z])', '_', prop).lower()


class SignalStateMessage:

    @classmethod
    def create_message(cls, signal_state: SignalState) -> obstacle_pb2.SignalState:
        signal_state_msg = obstacle_pb2.SignalState()

        for attr in SignalState.__slots__:
            if hasattr(signal_state, attr):
                if getattr(signal_state, attr) is None:
                    continue

                if attr == 'time_step':
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
        dynamic_obstacle_msg.obstacle_type = obstacle_pb2.ObstacleTypeEnum.ObstacleType \
            .Value(dynamic_obstacle.obstacle_type.name)

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
        environment_obstacle_msg.obstacle_type = obstacle_pb2.ObstacleTypeEnum.ObstacleType \
            .Value(environment_obstacle.obstacle_type.name)

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
                goal_state_msg = GoalStateMessage \
                    .create_message(state, planning_problem.goal.lanelets_of_goal_position[i])
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
