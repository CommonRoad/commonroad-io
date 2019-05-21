from collections import defaultdict
from typing import Union, List, Tuple
from xml.etree import ElementTree
import numpy as np

from commonroad.common.util import Interval, AngleInterval
from commonroad.geometry.shape import Rectangle, Circle, Polygon, ShapeGroup, Shape
from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import PlanningProblemSet, PlanningProblem
from commonroad.prediction.prediction import Occupancy, SetBasedPrediction, TrajectoryPrediction
from commonroad.scenario.lanelet import Lanelet, LaneletNetwork, LineMarking
from commonroad.scenario.obstacle import ObstacleRole, ObstacleType, StaticObstacle, \
    DynamicObstacle, Obstacle
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import State, Trajectory

__author__ = "Stefanie Manzinger"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["Priority Program SPP 1835 Cooperative Interacting Automobiles"]
__version__ = "2019.1"
__maintainer__ = "Stefanie Manzinger"
__email__ = "commonroad@in.tum.de"
__status__ = "Released"


COMMONROAD_VERSION = '2018b'


def read_value_exact_or_interval(xml_node: ElementTree.Element)\
        -> Union[float, Interval]:
    if xml_node.find('exact') is not None:
        value = float(xml_node.find('exact').text)
    elif xml_node.find('intervalStart') is not None \
            and xml_node.find('intervalEnd') is not None:
        value = Interval(
            float(xml_node.find('intervalStart').text),
            float(xml_node.find('intervalEnd').text))
    else:
        raise Exception()
    return value


class CommonRoadFileReader:
    """ Class which reads CommonRoad XML-files. The XML-files are composed of
    (1) a formal representation of the road network,
    (2) static and dynamic obstacles,
    (3) the planning problem of the ego vehicle(s). """
    def __init__(self, filename: str):
        """
        :param filename: full path + filename of the CommonRoad XML-file,
        """
        self._filename = filename
        self._tree = None
        self._dt = None
        self._benchmark_id = None

    def open(self) -> Tuple[Scenario, PlanningProblemSet]:
        """
        Reads a CommonRoad XML-file.

        :return: the scenario containing the road network and the obstacles and the planning problem set \
        containing the planning problems---initial states and goal regions--for all ego vehicles.
        """
        self._read_header()
        scenario = self._open_scenario()
        planning_problem_set = self._open_planning_problem_set(scenario.lanelet_network)
        return scenario, planning_problem_set

    def open_lanelet_network(self) -> LaneletNetwork:
        """
        Reads the lanelet network of a CommonRoad XML-file.

        :return: object of class LaneletNetwork
        """
        self._read_header()
        return LaneletNetworkFactory.create_from_xml_node(self._tree)

    def _open_scenario(self) -> Scenario:
        """
        Reads the lanelet network and obstacles from the CommonRoad XML-file.

        :return: object of class scenario containing the road network and the obstacles
        """
        scenario = ScenarioFactory.create_from_xml_node(self._tree, self._dt, self._benchmark_id)
        return scenario

    def _open_planning_problem_set(self, lanelet_network: LaneletNetwork) \
            -> PlanningProblemSet:
        """
        Reads all planning problems from the CommonRoad XML-file.

        :return: object of class PlanningProblemSet containing the planning problems for all ego vehicles.
        """
        planning_problem_set = PlanningProblemSetFactory.create_from_xml_node(
            self._tree, lanelet_network)
        return planning_problem_set

    def _read_header(self):
        """ Parses the CommonRoad XML-file into element tree; reads the global time step size of the time-discrete
        scenario and the CommonRoad benchmark ID."""
        self._parse_file()
        commonroad_version = self._get_commonroad_version()
        assert commonroad_version == COMMONROAD_VERSION, '<CommonRoadFileReader/_read_header>: CommonRoad version of ' \
                                                         'XML-file {} is not supported. Supported version: {}. ' \
                                                         'Got version: {}.'.format(self._filename,
                                                                                   COMMONROAD_VERSION,
                                                                                   commonroad_version)
        self._dt = self._get_dt()
        self._benchmark_id = self._get_benchmark_id()

    def _parse_file(self):
        """ Parses the CommonRoad XML-file into element tree."""
        self._tree = ElementTree.parse(self._filename)

    def _get_dt(self) -> float:
        """ Reads the time step size of the time-discrete scenario."""
        return float(self._tree.getroot().get('timeStepSize'))

    def _get_benchmark_id(self) -> str:
        """ Reads the unique CommonRoad benchmark ID of the scenario."""
        return self._tree.getroot().get('benchmarkID')

    def _get_commonroad_version(self) -> str:
        """ Reads the CommonRoad version of the XML-file."""
        return self._tree.getroot().get('commonRoadVersion')


class ScenarioFactory:
    """ Class to create an object of class Scenario from an XML element."""
    @classmethod
    def create_from_xml_node(cls, xml_node: ElementTree.Element, dt: float, benchmark_id: str):
        """
        :param xml_node: XML element
        :param dt: time step size of the scenario
        :param benchmark_id: unique CommonRoad benchmark ID
        :return:
        """
        scenario = Scenario(dt, benchmark_id)
        scenario.add_objects(LaneletNetworkFactory.create_from_xml_node(xml_node))
        scenario.add_objects(cls._obstacles(xml_node))
        return scenario

    @classmethod
    def _obstacles(cls, xml_node: ElementTree.Element) -> List[Obstacle]:
        """
        Reads all obstacles specified in a CommonRoad XML-file.
        :param xml_node: XML element
        :param dt: time step size of the scenario
        :return: list of static and dynamic obstacles specified in the CommonRoad XML-file
        """
        obstacles = list()
        for o in xml_node.findall('obstacle'):
            obstacles.append(ObstacleFactory.create_from_xml_node(o))
        return obstacles


class LaneletNetworkFactory:
    @classmethod
    def create_from_xml_node(cls, xml_node: ElementTree.Element):
        """
        Reads all lanelets specified in a CommonRoad XML-file.
        :param xml_node: XML element
        :return: list of lanelets
        """
        lanelets = list()
        for lanelet_node in xml_node.findall('lanelet'):
            lanelets.append(LaneletFactory.create_from_xml_node(lanelet_node))
        return LaneletNetwork.create_from_lanelet_list(lanelets)


class LaneletFactory:
    """ Class to create an object of class Lanelet from an XML element."""
    @classmethod
    def create_from_xml_node(cls, xml_node: ElementTree.Element) -> Lanelet:
        """
        :param xml_node: XML element
        :return: object of class Lanelet according to the CommonRoad specification.
        """
        lanelet_id = int(xml_node.get('id'))

        left_vertices = cls._vertices(xml_node.find('leftBound'))
        right_vertices = cls._vertices(xml_node.find('rightBound'))
        center_vertices = 0.5 * (left_vertices + right_vertices)

        line_marking_left_vertices = cls._line_marking(xml_node.find('leftBound'))
        line_marking_right_vertices = cls._line_marking(xml_node.find('rightBound'))

        predecessors = cls._predecessors(xml_node)
        successors = cls._successors(xml_node)

        adjacent_left, adjacent_left_same_direction = cls._adjacent_left(xml_node)
        adjacent_right, adjacent_right_same_direction = cls._adjacent_right(xml_node)

        speed_limit = cls._speed_limit(xml_node)

        return Lanelet(
            left_vertices=left_vertices, center_vertices=center_vertices, right_vertices=right_vertices,
            lanelet_id=lanelet_id,
            predecessor=predecessors, successor=successors,
            adjacent_left=adjacent_left, adjacent_left_same_direction=adjacent_left_same_direction,
            adjacent_right=adjacent_right, adjacent_right_same_direction=adjacent_right_same_direction,
            speed_limit=speed_limit,
            line_marking_left_vertices=line_marking_left_vertices,
            line_marking_right_vertices=line_marking_right_vertices)

    @classmethod
    def _vertices(cls, xml_node: ElementTree.Element) -> np.ndarray:
        """
        Reads the vertices of the lanelet boundary.
        :param xml_node: XML element
        :return: The vertices of the boundary of the Lanelet described as a polyline
        """
        return PointListFactory.create_from_xml_node(xml_node)

    @classmethod
    def _predecessors(cls, xml_node: ElementTree.Element) -> List[int]:
        """
        Reads all predecessor lanelets.
        :param xml_node: XML element
        :return: list of IDs of all predecessor lanelets
        """
        predecessors = list()
        for l in xml_node.findall('predecessor'):
            predecessors.append(int(l.get('ref')))
        return predecessors

    @classmethod
    def _successors(cls, xml_node: ElementTree.Element) -> List[int]:
        """
        Reads all successor lanelets.
        :param xml_node: XML element
        :return: list of IDs of all successor lanelets
        """
        successors = list()
        for l in xml_node.findall('successor'):
            successors.append(int(l.get('ref')))
        return successors

    @classmethod
    def _adjacent_left(cls, xml_node: ElementTree.Element) -> Tuple[Union[int, None], Union[bool, None]]:
        """
        Reads the ID and the driving direction of a neighboring lanelet which is adjacent to the left.
        :param xml_node: XML element
        :return: the ID of the lanelet which is adjacent to the left (None if not existing);
                 the driving direction of the neighboring lanelet (None if not existing)
        """
        adjacent_left = None
        adjacent_left_same_direction = None
        if xml_node.find('adjacentLeft') is not None:
            adjacent_left = int(xml_node.find('adjacentLeft').get('ref'))
            if xml_node.find('adjacentLeft').get('drivingDir') == 'same':
                adjacent_left_same_direction = True
            else:
                adjacent_left_same_direction = False
        return adjacent_left, adjacent_left_same_direction

    @classmethod
    def _adjacent_right(cls, xml_node: ElementTree.Element) -> Tuple[Union[int, None], Union[bool, None]]:
        """
        Reads the ID and the driving direction of a neighboring lanelet which is adjacent to the right.
        :param xml_node: XML element
        :return: the ID of the lanelet which is adjacent to the right (None if not existing);
                 the driving direction of the neighboring lanelet (None if not existing)
        """
        adjacent_right = None
        adjacent_right_same_direction = None
        if xml_node.find('adjacentRight') is not None:
            adjacent_right = int(xml_node.find('adjacentRight').get('ref'))
            if xml_node.find('adjacentRight').get('drivingDir') == 'same':
                adjacent_right_same_direction = True
            else:
                adjacent_right_same_direction = False
        return adjacent_right, adjacent_right_same_direction

    @classmethod
    def _speed_limit(cls, xml_node: ElementTree.Element) -> Union[float, None]:
        """
        Reads the speed limit of the lanelet.
        :param xml_node: XML element
        :return: speed limit of the lanelet. If the speed limit is not specified in the CommonRoad XML-file,
        it is set to infinity.
        """
        speed_limit = float('inf')
        if xml_node.find('speedLimit') is not None:
            speed_limit = float(xml_node.find('speedLimit').text)
        return speed_limit

    @classmethod
    def _line_marking(cls, xml_node: ElementTree.Element) -> Union[None, LineMarking]:
        """
        Reads the line marking of the left or right lanelet boundary.
        :param xml_node: XML element
        :return: the type of the line marking of the lanelet boundary (None if not specified).
        """
        line_marking = None
        if xml_node.find('lineMarking') is not None:
            line_marking = cls._string_to_line_marking_enum(
                xml_node.find('lineMarking').text)
        return line_marking

    @classmethod
    def _string_to_line_marking_enum(cls, line_marking) -> LineMarking:
        """
        Converts a string to type LineMarking.
        :param xml_node: XML element
        :return: the type of the line marking of the lanelet boundary (None if not specified).
        """
        if line_marking == 'dashed':
            return LineMarking.DASHED
        elif line_marking == 'solid':
            return LineMarking.SOLID
        else:
            raise ValueError('<LaneletFactory/_string_to_line_marking_enum>: Unkown type of line marking: %s.'
                             % line_marking)


class ObstacleFactory:
    @classmethod
    def create_from_xml_node(cls, xml_node: ElementTree.Element) \
            -> Obstacle:
        if xml_node.find('role').text == 'static':
            return StaticObstacleFactory.create_from_xml_node(xml_node)
        elif xml_node.find('role').text == 'dynamic':
            return DynamicObstacleFactory.create_from_xml_node(xml_node)
        else:
            raise ValueError('Role of obstacle is unknown. Valid roles: {}. '
                             'Got role: {}'.format(ObstacleRole, xml_node.find('role').text))

    @classmethod
    def read_role(cls, xml_node: ElementTree.Element) -> ObstacleRole:
        obstacle_role = cls._string_to_obstacle_role_enum(
            xml_node.find('role').text)
        return obstacle_role

    @classmethod
    def read_type(cls, xml_node: ElementTree.Element) -> ObstacleType:
        obstacle_type = cls._string_to_obstacle_type_enum(
            xml_node.find('type').text)
        return obstacle_type

    @classmethod
    def read_id(cls, xml_node: ElementTree.Element) -> int:
        obstacle_id = int(xml_node.get('id'))
        return obstacle_id

    @classmethod
    def read_initial_state(cls, xml_node: ElementTree.Element) -> State:
        initial_state = StateFactory.create_from_xml_node(xml_node)
        return initial_state

    @classmethod
    def read_shape(cls, xml_node: ElementTree.Element) -> Shape:
        shape = ShapeFactory.create_from_xml_node(xml_node)
        return shape

    @classmethod
    def _string_to_obstacle_role_enum(cls, obstacle_role: str) -> ObstacleRole:
        if obstacle_role == 'static':
            return ObstacleRole.STATIC
        elif obstacle_role == 'dynamic':
            return ObstacleRole.DYNAMIC

    @classmethod
    def _string_to_obstacle_type_enum(cls, obstacle_type: str) -> ObstacleType:
        if obstacle_type == 'unknown':
            return ObstacleType.UNKNOWN
        elif obstacle_type == 'car':
            return ObstacleType.CAR
        elif obstacle_type == 'truck':
            return ObstacleType.TRUCK
        elif obstacle_type == 'bus':
            return ObstacleType.BUS
        elif obstacle_type == 'bicycle':
            return ObstacleType.BICYCLE
        elif obstacle_type == 'pedestrian':
            return ObstacleType.PEDESTRIAN
        elif obstacle_type == 'priorityVehicle':
            return ObstacleType.PRIORITY_VEHICLE
        elif obstacle_type == 'parkedVehicle':
            return ObstacleType.PARKED_VEHICLE
        elif obstacle_type == 'constructionZone':
            return ObstacleType.CONSTRUCTION_ZONE
        elif obstacle_type == 'train':
            return ObstacleType.TRAIN
        elif obstacle_type == 'roadBoundary':
            return ObstacleType.ROAD_BOUNDARY
        else:
            raise ValueError('Type of obstacle is unknown. Got type: {}'.format(obstacle_type))


class StaticObstacleFactory:
    @classmethod
    def create_from_xml_node(cls, xml_node: ElementTree.Element) -> StaticObstacle:
        obstacle_type = ObstacleFactory.read_type(xml_node)
        obstacle_id = ObstacleFactory.read_id(xml_node)
        shape = ObstacleFactory.read_shape(xml_node.find('shape'))
        initial_state = ObstacleFactory.read_initial_state(xml_node.find('initialState'))
        return StaticObstacle(obstacle_id=obstacle_id, obstacle_type=obstacle_type,
                              obstacle_shape=shape, initial_state=initial_state)


class DynamicObstacleFactory:
    @classmethod
    def create_from_xml_node(cls, xml_node: ElementTree.Element) -> DynamicObstacle:
        obstacle_type = ObstacleFactory.read_type(xml_node)
        obstacle_id = ObstacleFactory.read_id(xml_node)
        shape = ObstacleFactory.read_shape(xml_node.find('shape'))
        initial_state = ObstacleFactory.read_initial_state(xml_node.find('initialState'))

        if xml_node.find('trajectory') is not None:
            trajectory = TrajectoryFactory.create_from_xml_node(xml_node.find('trajectory'))
            prediction = TrajectoryPrediction(trajectory, shape)
        elif xml_node.find('occupancySet') is not None:
            prediction = SetBasedPredictionFactory.create_from_xml_node(xml_node.find('occupancySet'))
        else:
            prediction = None
        return DynamicObstacle(obstacle_id=obstacle_id, obstacle_type=obstacle_type,
                               obstacle_shape=shape, initial_state=initial_state, prediction=prediction)


class TrajectoryFactory:
    @classmethod
    def create_from_xml_node(cls, xml_node: ElementTree.Element) \
            -> Trajectory:
        state_list = list()
        for state_node in xml_node.findall('state'):
            state_list.append(StateFactory.create_from_xml_node(state_node))
        if isinstance(state_list[0].time_step, Interval):
            t0 = min(state_list[0].time_step)
        else:
            t0 = state_list[0].time_step
        return Trajectory(t0, state_list)


class SetBasedPredictionFactory:
    @classmethod
    def create_from_xml_node(cls, xml_node: ElementTree.Element) -> SetBasedPrediction:
        occupancies = list()
        for occupancy in xml_node.findall('occupancy'):
            occupancies.append(OccupancyFactory.create_from_xml_node(occupancy))
        if isinstance(occupancies[0].time_step, Interval):
            t0 = min(occupancies[0].time_step)
        else:
            t0 = occupancies[0].time_step
        return SetBasedPrediction(t0, occupancies)


class OccupancyFactory:
    @classmethod
    def create_from_xml_node(cls, xml_node: ElementTree.Element) -> Occupancy:
        shape = ShapeFactory.create_from_xml_node(xml_node.find('shape'))
        time = StateFactory.read_time(xml_node.find('time'))
        return Occupancy(time, shape)


class ShapeFactory:
    @classmethod
    def create_from_xml_node(cls, xml_node: ElementTree.Element) -> Shape:
        shape_list = list()
        for c in list(xml_node):
            shape_list.append(cls._read_single_shape(c))
        shape = cls._create_shape_group_if_needed(shape_list)
        return shape

    @classmethod
    def _read_single_shape(cls, xml_node: ElementTree.Element) \
            -> Shape:
        tag_string = xml_node.tag
        if tag_string == 'rectangle':
            return RectangleFactory.create_from_xml_node(xml_node)
        elif tag_string == 'circle':
            return CircleFactory.create_from_xml_node(xml_node)
        elif tag_string == 'polygon':
            return PolygonFactory.create_from_xml_node(xml_node)

    @classmethod
    def _create_shape_group_if_needed(cls, shape_list: List[Shape]) -> Shape:
        if len(shape_list) > 1:
            sg = ShapeGroup(shape_list)
            return sg
        else:
            return shape_list[0]


class RectangleFactory:
    @classmethod
    def create_from_xml_node(cls, xml_node: ElementTree.Element) -> Rectangle:
        length = float(xml_node.find('length').text)
        width = float(xml_node.find('width').text)
        if xml_node.find('orientation') is not None:
            orientation = float(xml_node.find('orientation').text)
        else:
            orientation = 0.0
        if xml_node.find('center') is not None:
            center = PointFactory.create_from_xml_node(
                xml_node.find('center'))
        else:
            center = np.array([0.0, 0.0])
        return Rectangle(length, width, center, orientation)


class CircleFactory:
    @classmethod
    def create_from_xml_node(cls, xml_node: ElementTree.Element) -> Circle:
        radius = float(xml_node.find('radius').text)
        if xml_node.find('center') is not None:
            center = PointFactory.create_from_xml_node(
                xml_node.find('center'))
        else:
            center = np.array([0.0, 0.0])
        return Circle(radius, center)


class PolygonFactory:
    @classmethod
    def create_from_xml_node(cls, xml_node: ElementTree.Element) -> Polygon:
        vertices = PointListFactory.create_from_xml_node(xml_node)
        return Polygon(vertices)


class PlanningProblemSetFactory:
    @classmethod
    def create_from_xml_node(cls, xml_node: ElementTree.Element, lanelet_network: LaneletNetwork) \
            -> PlanningProblemSet:
        planning_problem_set = PlanningProblemSet()
        for p in xml_node.findall('planningProblem'):
            planning_problem_set.add_planning_problem(
                PlanningProblemFactory.create_from_xml_node(p, lanelet_network))
        return planning_problem_set


class PlanningProblemFactory:
    @classmethod
    def create_from_xml_node(cls, xml_node: ElementTree.Element, lanelet_network: LaneletNetwork) \
            -> PlanningProblem:
        planning_problem_id = int(xml_node.get('id'))
        initial_state = cls._add_initial_state(xml_node)
        goal_region = GoalRegionFactory.create_from_xml_node(xml_node, lanelet_network)
        return PlanningProblem(planning_problem_id, initial_state, goal_region)

    @classmethod
    def _add_initial_state(cls, xml_node: ElementTree.Element) \
            -> State:
        initial_state = StateFactory.create_from_xml_node(xml_node.find('initialState'))
        return initial_state


class GoalRegionFactory:
    @classmethod
    def create_from_xml_node(cls, xml_node: ElementTree.Element, lanelet_network: LaneletNetwork)\
            -> GoalRegion:
        state_list = list()
        lanelets_of_goal_position = defaultdict(list)
        for idx, goal_state_node in enumerate(xml_node.findall('goalState')):
            state_list.append(StateFactory.create_from_xml_node(goal_state_node, lanelet_network))
            if goal_state_node.find('position') is not None\
                    and goal_state_node.find('position').find('lanelet') is not None:
                for l in goal_state_node.find('position').findall('lanelet'):
                    lanelets_of_goal_position[idx].append(int(l.get('ref')))
        if not lanelets_of_goal_position:
            lanelets_of_goal_position = None
        return GoalRegion(state_list, lanelets_of_goal_position)


class StateFactory:
    @classmethod
    def create_from_xml_node(cls, xml_node: ElementTree.Element, lanelet_network: Union[LaneletNetwork, None] = None)\
            -> State:
        state_args = dict()
        if xml_node.find('position') is not None:
            position = cls._read_position(xml_node.find('position'), lanelet_network)
            state_args['position'] = position
        if xml_node.find('time') is not None:
            state_args['time_step'] = cls.read_time(xml_node.find('time'))
        if xml_node.find('orientation') is not None:
            orientation = cls._read_orientation(xml_node.find('orientation'))
            state_args['orientation'] = orientation
        if xml_node.find('velocity') is not None:
            speed = read_value_exact_or_interval(xml_node.find('velocity'))
            state_args['velocity'] = speed
        if xml_node.find('acceleration') is not None:
            acceleration = read_value_exact_or_interval(xml_node.find('acceleration'))
            state_args['acceleration'] = acceleration
        if xml_node.find('yawRate') is not None:
            yaw_rate = read_value_exact_or_interval(xml_node.find('yawRate'))
            state_args['yaw_rate'] = yaw_rate
        if xml_node.find('slipAngle') is not None:
            slip_angle = read_value_exact_or_interval(xml_node.find('slipAngle'))
            state_args['slip_angle'] = slip_angle
        return State(**state_args)

    @classmethod
    def _read_position(cls, xml_node: ElementTree.Element,
                       lanelet_network: Union[LaneletNetwork, None] = None) \
            -> Union[np.ndarray, Shape]:
        if xml_node.find('point') is not None:
            position = PointFactory.create_from_xml_node(xml_node.find('point'))
        elif (xml_node.find('rectangle') is not None
              or xml_node.find('circle') is not None
              or xml_node.find('polygon') is not None):
            position = ShapeFactory.create_from_xml_node(xml_node)
        elif lanelet_network is not None and xml_node.find('lanelet') is not None:
            position_list = list()
            for l in xml_node.findall('lanelet'):
                lanelet = lanelet_network.find_lanelet_by_id(int(l.get('ref')))
                polygon = lanelet.convert_to_polygon()
                position_list.append(polygon)
            position = ShapeGroup(position_list)
        else:
            raise Exception()
        return position

    @classmethod
    def _read_orientation(cls, xml_node: ElementTree.Element) -> Union[float, AngleInterval]:
        if xml_node.find('exact') is not None:
            value = float(xml_node.find('exact').text)
        elif xml_node.find('intervalStart') is not None \
                and xml_node.find('intervalEnd') is not None:
            value = AngleInterval(
                float(xml_node.find('intervalStart').text),
                float(xml_node.find('intervalEnd').text))
        else:
            raise Exception()
        return value

    @classmethod
    def read_time(cls, xml_node: ElementTree.Element) -> Union[int, Interval]:
        if xml_node.find('exact') is not None:
            value = int(xml_node.find('exact').text)
        elif xml_node.find('intervalStart') is not None \
                and xml_node.find('intervalEnd') is not None:
            value = Interval(
                int(xml_node.find('intervalStart').text),
                int(xml_node.find('intervalEnd').text))
        else:
            raise Exception()
        return value


class PointListFactory:
    @classmethod
    def create_from_xml_node(cls, xml_node: ElementTree.Element) -> np.ndarray:
        point_list = []
        for point_node in xml_node.findall("point"):
            point_list.append(
                PointFactory.create_from_xml_node(point_node))
        return np.array(point_list)


class PointFactory:
    @classmethod
    def create_from_xml_node(cls, xml_node: ElementTree.Element) -> np.ndarray:
        x = float(xml_node.find('x').text)
        y = float(xml_node.find('y').text)
        return np.array([x, y])
