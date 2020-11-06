import copy
import enum
from typing import *
import numpy as np
import networkx as nx

import commonroad.geometry.transform
from commonroad.common.validity import *
from commonroad.geometry.shape import Polygon, ShapeGroup, Circle, Rectangle, Shape
from commonroad.scenario.obstacle import Obstacle
from commonroad.scenario.traffic_sign import TrafficSign, TrafficLight
from commonroad.scenario.intersection import Intersection

__author__ = "Christian Pek, Sebastian Maierhofer"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["BMW CAR@TUM"]
__version__ = "2020.3"
__maintainer__ = "Sebastian Maierhofer"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "released"


class LineMarking(enum.Enum):
    """
    Enum describing different types of line markings, i.e. dashed or solid lines
    """
    DASHED = 'dashed'
    SOLID = 'solid'
    BROAD_DASHED = 'broad_dashed'
    BROAD_SOLID = 'broad_solid'
    UNKNOWN = 'unknown'
    NO_MARKING = 'no_marking'


class LaneletType(enum.Enum):
    """
    Enum describing different types of lanelets
    """
    URBAN = 'urban'
    COUNTRY = 'country'
    HIGHWAY = 'highway'
    DRIVE_WAY = 'driveWay'
    MAIN_CARRIAGE_WAY = 'mainCarriageWay'
    ACCESS_RAMP = 'accessRamp'
    EXIT_RAMP = 'exitRamp'
    SHOULDER = 'shoulder'
    BUS_LANE = 'busLane'
    BUS_STOP = 'busStop'
    BICYCLE_LANE = 'bicycleLane'
    SIDEWALK = 'sidewalk'
    CROSSWALK = 'crosswalk'
    INTERSTATE = 'interstate'
    UNKNOWN = 'unknown'


class RoadUser(enum.Enum):
    """
    Enum describing different types of road users
    """
    VEHICLE = 'vehicle'
    CAR = 'car'
    TRUCK = 'truck'
    BUS = 'bus'
    PRIORITY_VEHICLE = 'priorityVehicle'
    MOTORCYCLE = 'motorcycle'
    BICYCLE = 'bicycle'
    PEDESTRIAN = 'pedestrian'
    TRAIN = 'train'
    TAXI = 'taxi'


class StopLine:
    """Class which describes the stop line of a lanelet"""
    def __init__(self, start: np.ndarray, end: np.ndarray,
                 line_marking: LineMarking,
                 traffic_sign_ref: Set[int] = None,
                 traffic_light_ref: Set[int] = None):
        self._start = start
        self._end = end
        self._line_marking = line_marking
        self._traffic_sign_ref = traffic_sign_ref
        self._traffic_light_ref = traffic_light_ref

    @property
    def start(self) -> np.ndarray:
        return self._start

    @property
    def end(self) -> np.ndarray:
        return self._end

    @property
    def line_marking(self) -> LineMarking:
        return self._line_marking

    @property
    def traffic_sign_ref(self) -> Set[int]:
        return self._traffic_sign_ref

    @property
    def traffic_light_ref(self) -> Set[int]:
        return self._traffic_light_ref

    def translate_rotate(self, translation: np.ndarray, angle: float):
        """
        This method translates and rotates a stop line

        :param translation: The translation given as [x_off,y_off] for the x and y translation
        :param angle: The rotation angle in radian (counter-clockwise defined)
        """

        assert is_real_number_vector(translation,
                                     2), '<Lanelet/translate_rotate>: provided translation ' \
                                         'is not valid! translation = {}'.format(translation)
        assert is_valid_orientation(
            angle), '<Lanelet/translate_rotate>: provided angle is not valid! angle = {}'.format(angle)

        # create transformation matrix
        t_m = commonroad.geometry.transform.translation_rotation_matrix(translation,
                                                                        angle)
        line_vertices = np.array([self._start, self._end])
        # transform center vertices
        tmp = t_m.dot(np.vstack((line_vertices.transpose(),
                                 np.ones((1, line_vertices.shape[0])))))
        tmp = tmp[0:2, :].transpose()
        self._start, self._end = tmp[0], tmp[1]

    def __str__(self):
        return f'StopLine from {self._start} to  {self._end}'


class Lanelet:
    """
    Class which describes a Lanelet entity according to the CommonRoad specification. Each lanelet is described by a
    left and right boundary (polylines). Furthermore, lanelets have relations to other lanelets, e.g. an adjacent left
    neighbor or a predecessor.
    """

    def __init__(self, left_vertices: np.ndarray, center_vertices: np.ndarray, right_vertices: np.ndarray,
                 lanelet_id: int, predecessor=None, successor=None,
                 adjacent_left=None, adjacent_left_same_direction=None,
                 adjacent_right=None, adjacent_right_same_direction=None,
                 line_marking_left_vertices=LineMarking.NO_MARKING,
                 line_marking_right_vertices=LineMarking.NO_MARKING,
                 stop_line=None,
                 lanelet_type=None,
                 user_one_way=None,
                 user_bidirectional=None,
                 traffic_signs=None,
                 traffic_lights=None,
                 ):
        """
        Constructor of a Lanelet object
        :param left_vertices: The vertices of the left boundary of the Lanelet described as a
        polyline [[x0,x1,...,xn],[y0,y1,...,yn]]
        :param center_vertices: The vertices of the center line of the Lanelet described as a
        polyline [[x0,x1,...,xn],[y0,y1,...,yn]]
        :param right_vertices: The vertices of the right boundary of the Lanelet described as a
        polyline [[x0,x1,...,xn],[y0,y1,...,yn]]
        :param lanelet_id: The unique id (natural number) of the lanelet
        :param predecessor: The list of predecessor lanelets (None if not existing)
        :param successor: The list of successor lanelets (None if not existing)
        :param adjacent_left: The adjacent left lanelet (None if not existing)
        :param adjacent_left_same_direction: True if the adjacent left lanelet has the same driving direction,
        false otherwise (None if no left adjacent lanelet exists)
        :param adjacent_right: The adjacent right lanelet (None if not existing)
        :param adjacent_right_same_direction: True if the adjacent right lanelet has the same driving direction,
        false otherwise (None if no right adjacent lanelet exists)
        :param line_marking_left_vertices: The type of line marking of the left boundary
        :param line_marking_right_vertices: The type of line marking of the right boundary
        :param stop_line: The stop line of the lanelet
        :param lanelet_type: The types of lanelet applicable here
        :param user_one_way: type of users that will use the lanelet as one-way
        :param user_bidirectional: type of users that will use the lanelet as bidirectional way
        :param traffic_signs: Traffic signs to be applied
        :param traffic_lights: Traffic lights to follow
        """

        # Set required properties
        self._left_vertices = None
        self._right_vertices = None
        self._center_vertices = None
        self._lanelet_id = None

        self.lanelet_id = lanelet_id
        self.left_vertices = left_vertices
        self.right_vertices = right_vertices
        self.center_vertices = center_vertices
        # check if length of each polyline is the same
        assert len(left_vertices[0]) == len(center_vertices[0]) == len(
            right_vertices[0]), '<Lanelet/init>: Provided polylines do not share the same length! {}/{}/{}'.format(
            len(left_vertices[0]), len(center_vertices[0]), len(right_vertices[0]))

        # Set lane markings
        self._line_marking_left_vertices = line_marking_left_vertices
        self._line_marking_right_vertices = line_marking_right_vertices

        # Set predecessors and successors
        self._predecessor = None
        if predecessor is None:
            self._predecessor = []
        else:
            self.predecessor = predecessor
        self._successor = None
        if successor is None:
            self._successor = []
        else:
            self.successor = successor

        # Set adjacent lanelets
        self._adj_left = None
        self._adj_left_same_direction = None
        if adjacent_left is not None:
            self.adj_left = adjacent_left
            self.adj_left_same_direction = adjacent_left_same_direction
        self._adj_right = None
        self._adj_right_same_direction = None
        if adjacent_right is not None:
            self.adj_right = adjacent_right
            self.adj_right_same_direction = adjacent_right_same_direction

        self._distance = [0.0]
        for i in range(1, len(self.center_vertices)):
            self._distance.append(self._distance[i - 1] +
                                  np.linalg.norm(np.array(self.center_vertices[i]) -
                                                 np.array(self.center_vertices[i - 1])))
        self._distance = np.array(self._distance)

        # create empty polygon
        self._polygon = None

        self._dynamic_obstacles_on_lanelet = {}
        self._static_obstacles_on_lanelet = set()

        self._stop_line = None
        if stop_line:
            self.stop_line = stop_line

        self._lanelet_type = None
        if lanelet_type is None:
            self._lanelet_type = set()
        else:
            self.lanelet_type = lanelet_type

        self._user_one_way = None
        if user_one_way is None:
            self._user_one_way = set()
        else:
            self.user_one_way = user_one_way

        self._user_bidirectional = None
        if user_bidirectional is None:
            self._user_bidirectional = set()
        else:
            self.user_bidirectional = user_bidirectional

        # Set Traffic Rules
        self._traffic_signs = None
        if traffic_signs is None:
            self._traffic_signs = set()
        else:
            self.traffic_signs = traffic_signs

        self._traffic_lights = None
        if traffic_lights is None:
            self._traffic_lights = set()
        else:
            self.traffic_lights = traffic_lights

    @property
    def distance(self) -> np.ndarray:
        return self._distance

    @distance.setter
    def distance(self, dist: np.ndarray):
        warnings.warn('<Lanelet/distance> distance of lanelet is immutable')

    @property
    def lanelet_id(self) -> int:
        return self._lanelet_id

    @lanelet_id.setter
    def lanelet_id(self, l_id: int):
        if self._lanelet_id is None:
            assert is_natural_number(l_id), '<Lanelet/lanelet_id>: Provided lanelet_id is not valid! id={}'.format(l_id)
            self._lanelet_id = l_id
        else:
            warnings.warn('<Lanelet/lanelet_id>: lanelet_id of lanelet is immutable')

    @property
    def left_vertices(self) -> np.ndarray:
        return self._left_vertices

    @left_vertices.setter
    def left_vertices(self, polyline: np.ndarray):
        if self._left_vertices is None:
            assert is_valid_polyline(
                polyline), '<Lanelet/left_vertices>: The provided polyline is not valid! polyline = {}'.format(polyline)
            self._left_vertices = polyline
        else:
            warnings.warn('<Lanelet/left_vertices>: left_vertices of lanelet are immutable!')

    @property
    def right_vertices(self) -> np.ndarray:
        return self._right_vertices

    @right_vertices.setter
    def right_vertices(self, polyline: np.ndarray):
        if self._right_vertices is None:
            assert is_valid_polyline(
                polyline), '<Lanelet/right_vertices>: The provided polyline is not valid! polyline = {}'.format(
                polyline)
            self._right_vertices = polyline
        else:
            warnings.warn('<Lanelet/right_vertices>: right_vertices of lanelet are immutable!')

    @property
    def center_vertices(self) -> np.ndarray:
        return self._center_vertices

    @center_vertices.setter
    def center_vertices(self, polyline: np.ndarray):
        if self._center_vertices is None:
            assert is_valid_polyline(
                polyline), '<Lanelet/center_vertices>: The provided polyline is not valid! polyline = {}'.format(
                polyline)
            self._center_vertices = polyline
        else:
            warnings.warn('<Lanelet/center_vertices>: center_vertices of lanelet are immutable!')

    @property
    def line_marking_left_vertices(self) -> LineMarking:
        return self._line_marking_left_vertices

    @line_marking_left_vertices.setter
    def line_marking_left_vertices(self, line_marking_left_vertices: LineMarking):
        if self._line_marking_left_vertices is None:
            assert isinstance(line_marking_left_vertices,
                              LineMarking), '<Lanelet/line_marking_left_vertices>: Provided lane marking type of ' \
                                            'left boundary is not valid! type = {}'.format(
                type(line_marking_left_vertices))
            self._line_marking_left_vertices = LineMarking.UNKNOWN
        else:
            warnings.warn('<Lanelet/line_marking_left_vertices>: line_marking_left_vertices of lanelet is immutable!')

    @property
    def line_marking_right_vertices(self) -> LineMarking:
        return self._line_marking_right_vertices

    @line_marking_right_vertices.setter
    def line_marking_right_vertices(self, line_marking_right_vertices: LineMarking):
        if self._line_marking_right_vertices is None:
            assert isinstance(line_marking_right_vertices,
                              LineMarking), '<Lanelet/line_marking_right_vertices>: Provided lane marking type of ' \
                                            'right boundary is not valid! type = {}'.format(
                type(line_marking_right_vertices))
            self._line_marking_right_vertices = LineMarking.UNKNOWN
        else:
            warnings.warn('<Lanelet/line_marking_right_vertices>: line_marking_right_vertices of lanelet is immutable!')

    @property
    def predecessor(self) -> list:
        return self._predecessor

    @predecessor.setter
    def predecessor(self, predecessor: list):
        if self._predecessor is None:
            assert (is_list_of_natural_numbers(predecessor) and len(predecessor) >= 0), '<Lanelet/predecessor>: ' \
                                                                                        'Provided list ' \
                                                                                        'of predecessors is not valid!'\
                                                                                        'predecessors = {}'.format(
                predecessor)
            self._predecessor = predecessor
        else:
            warnings.warn(
                '<Lanelet/predecessor>: predecessor of lanelet is immutable!')

    @property
    def successor(self) -> list:
        return self._successor

    @successor.setter
    def successor(self, successor: list):
        if self._successor is None:
            assert (is_list_of_natural_numbers(successor) and len(successor) >= 0), '<Lanelet/predecessor>: Provided ' \
                                                                                    'list of successors is not valid!' \
                                                                                    'successors = {}'.format(successor)
            self._successor = successor
        else:
            warnings.warn(
                '<Lanelet/successor>: successor of lanelet is immutable!')

    @property
    def adj_left(self) -> int:
        return self._adj_left

    @adj_left.setter
    def adj_left(self, l_id: int):
        if self._adj_left is None:
            assert is_natural_number(l_id), '<Lanelet/adj_left>: provided id is not valid! id={}'.format(l_id)
            self._adj_left = l_id
        else:
            warnings.warn('<Lanelet/adj_left>: adj_left of lanelet is immutable')

    @property
    def adj_left_same_direction(self) -> bool:
        return self._adj_left_same_direction

    @adj_left_same_direction.setter
    def adj_left_same_direction(self, same: bool):
        if self._adj_left_same_direction is None:
            assert isinstance(same,
                              bool), '<Lanelet/adj_left_same_direction>: provided direction ' \
                                     'is not of type bool! type = {}'.format(type(same))
            self._adj_left_same_direction = same
        else:
            warnings.warn('<Lanelet/adj_left_same_direction>: adj_left_same_direction of lanelet is immutable')

    @property
    def adj_right(self) -> int:
        return self._adj_right

    @adj_right.setter
    def adj_right(self, l_id: int):
        if self._adj_right is None:
            assert is_natural_number(l_id), '<Lanelet/adj_right>: provided id is not valid! id={}'.format(l_id)
            self._adj_right = l_id
        else:
            warnings.warn('<Lanelet/adj_right>: adj_right of lanelet is immutable')

    @property
    def adj_right_same_direction(self) -> bool:
        return self._adj_right_same_direction

    @adj_right_same_direction.setter
    def adj_right_same_direction(self, same: bool):
        if self._adj_right_same_direction is None:
            assert isinstance(same,
                              bool), '<Lanelet/adj_right_same_direction>: provided direction ' \
                                     'is not of type bool! type = {}'.format(type(same))
            self._adj_right_same_direction = same
        else:
            warnings.warn('<Lanelet/adj_right_same_direction>: adj_right_same_direction of lanelet is immutable')

    @property
    def dynamic_obstacles_on_lanelet(self) -> Dict[int, Set[int]]:
        return self._dynamic_obstacles_on_lanelet

    @dynamic_obstacles_on_lanelet.setter
    def dynamic_obstacles_on_lanelet(self, obstacle_ids: Dict[int, Set[int]]):
        assert isinstance(obstacle_ids, dict), \
            '<Lanelet/obstacles_on_lanelet>: provided dictionary of ids is not a ' \
            'dictionary! type = {}'.format(type(obstacle_ids))
        self._dynamic_obstacles_on_lanelet = obstacle_ids

    @property
    def static_obstacles_on_lanelet(self) -> Union[None, Set[int]]:
        return self._static_obstacles_on_lanelet

    @static_obstacles_on_lanelet.setter
    def static_obstacles_on_lanelet(self, obstacle_ids: Set[int]):
        assert isinstance(obstacle_ids, set), \
            '<Lanelet/obstacles_on_lanelet>: provided list of ids is not a ' \
            'set! type = {}'.format(type(obstacle_ids))
        self._static_obstacles_on_lanelet = obstacle_ids

    @property
    def stop_line(self) -> StopLine:
        return self._stop_line

    @stop_line.setter
    def stop_line(self, stop_line: StopLine):
        if self._stop_line is None:
            assert isinstance(stop_line, StopLine),\
                '<Lanelet/stop_line>: ''Provided type is not valid! type = {}'.format(type(stop_line))
            self._stop_line = stop_line
        else:
            warnings.warn(
                '<Lanelet/stop_line>: stop_line of lanelet is immutable!')

    @property
    def lanelet_type(self) -> Set[LaneletType]:
        return self._lanelet_type

    @lanelet_type.setter
    def lanelet_type(self, lanelet_type: Set[LaneletType]):
        if self._lanelet_type is None or len(self._lanelet_type) == 0:
            assert isinstance(lanelet_type, set) and all(isinstance(elem, LaneletType) for elem in lanelet_type), \
                '<Lanelet/lanelet_type>: ''Provided type is not valid! type = {}'.format(type(lanelet_type))
            self._lanelet_type = lanelet_type
        else:
            warnings.warn(
                '<Lanelet/lanelet_type>: type of lanelet is immutable!')

    @property
    def user_one_way(self) -> Set[RoadUser]:
        return self._user_one_way

    @user_one_way.setter
    def user_one_way(self, user_one_way: Set[RoadUser]):
        if self._user_one_way is None :
            assert isinstance(user_one_way, set) and all(isinstance(elem, RoadUser) for elem in user_one_way),\
                '<Lanelet/user_one_way>: ''Provided type is not valid! type = {}'.format(
                type(user_one_way))
            self._user_one_way = user_one_way
        else:
            warnings.warn(
                '<Lanelet/user_one_way>: user_one_way of lanelet is immutable!')

    @property
    def user_bidirectional(self) -> Set[RoadUser]:
        return self._user_bidirectional

    @user_bidirectional.setter
    def user_bidirectional(self, user_bidirectional: Set[RoadUser]):
        if self._user_bidirectional is None:
            assert isinstance(user_bidirectional, set) and \
                   all(isinstance(elem, RoadUser) for elem in user_bidirectional), \
                '<Lanelet/user_bidirectional>: ''Provided type is not valid! type = {}'.format(
                    type(user_bidirectional))
            self._user_bidirectional = user_bidirectional
        else:
            warnings.warn(
                '<Lanelet/user_bidirectional>: user_bidirectional of lanelet is immutable!')

    @property
    def traffic_signs(self) -> Set[int]:
        return self._traffic_signs

    @traffic_signs.setter
    def traffic_signs(self, traffic_sign_ids: Set[int]):
        if self._traffic_signs is None:
            assert isinstance(traffic_sign_ids, set), \
                '<Lanelet/traffic_signs>: provided list of ids is not a ' \
                'set! type = {}'.format(type(traffic_sign_ids))
            self._traffic_signs = traffic_sign_ids
        else:
            warnings.warn(
                '<Lanelet/traffic_signs>: traffic_signs of lanelet is immutable!')

    @property
    def traffic_lights(self) -> Set[int]:
        return self._traffic_lights

    @traffic_lights.setter
    def traffic_lights(self, traffic_light_ids: Set[int]):
        if self._traffic_lights is None:
            assert isinstance(traffic_light_ids, set), \
                '<Lanelet/traffic_lights>: provided list of ids is not a ' \
                'set! type = {}'.format(type(traffic_light_ids))
            self._traffic_lights = traffic_light_ids
        else:
            warnings.warn(
                '<Lanelet/traffic_lights>: traffic_lights of lanelet is immutable!')

    def translate_rotate(self, translation: np.ndarray, angle: float):
        """
        This method translates and rotates a lanelet

        :param translation: The translation given as [x_off,y_off] for the x and y translation
        :param angle: The rotation angle in radian (counter-clockwise defined)
        """

        assert is_real_number_vector(translation,
                                     2), '<Lanelet/translate_rotate>: provided translation ' \
                                         'is not valid! translation = {}'.format(translation)
        assert is_valid_orientation(
            angle), '<Lanelet/translate_rotate>: provided angle is not valid! angle = {}'.format(angle)

        # create transformation matrix
        t_m = commonroad.geometry.transform.translation_rotation_matrix(translation,
                                                                        angle)
        # transform center vertices
        tmp = t_m.dot(np.vstack((self.center_vertices.transpose(),
                                 np.ones((1, self.center_vertices.shape[0])))))
        tmp = tmp[0:2, :]
        self._center_vertices = tmp.transpose()

        # transform left vertices
        tmp = t_m.dot(np.vstack((self.left_vertices.transpose(),
                                 np.ones((1, self.left_vertices.shape[0])))))
        tmp = tmp[0:2, :]
        self._left_vertices = tmp.transpose()

        # transform right vertices
        tmp = t_m.dot(np.vstack((self.right_vertices.transpose(),
                                 np.ones((1, self.right_vertices.shape[0])))))
        tmp = tmp[0:2, :]
        self._right_vertices = tmp.transpose()

        # transform the stop line
        if self._stop_line is not None:
            self._stop_line.translate_rotate(translation, angle)

        # recreate polygon in case it existed
        if self._polygon is not None:
            self._polygon = None
            self._polygon = self.convert_to_polygon()

    def interpolate_position(self, distance: float) -> tuple:
        """
        Computes the interpolated positions on the center/right/left polyline of the lanelet for a given distance
        along the lanelet

        :param distance: The distance for the interpolation
        :return: The interpolated positions on the center/right/left polyline
        in the form [[x_c,y_c],[x_r,y_r],[x_l,y_l]]
        """
        assert is_real_number(distance) and np.greater_equal(self.distance[-1],distance)\
               and np.greater_equal(distance, 0), '<Lanelet/interpolate_position>: provided distance is not valid! ' \
                                                 'distance = {}'.format(distance)
        idx = 0

        # find
        while (not (self.distance[idx] <= distance <= self.distance[idx + 1])):
            idx += 1
        r = (distance - self.distance[idx]) / (self.distance[idx + 1] -
                                               self.distance[idx])
        return ((1 - r) * self._center_vertices[idx] +
                r * self._center_vertices[idx + 1],
                (1 - r) * self._right_vertices[idx] +
                r * self._right_vertices[idx + 1],
                (1 - r) * self._left_vertices[idx] +
                r * self._left_vertices[idx + 1],
                idx)

    def convert_to_polygon(self) -> Polygon:
        """
        Converts the given lanelet to a polygon representation

        :return: The polygon of the lanelet
        """
        if self._polygon is None:
            self._polygon = Polygon(np.concatenate((self.right_vertices,
                                                    np.flip(self.left_vertices, 0))))
        return self._polygon

    def contains_points(self, point_list: np.ndarray) -> List[bool]:
        """
        Checks if a list of points is enclosed in the lanelet

        :param point_list: The list of points in the form [[px1,py1],[px2,py2,],...]
        :return: List of bools with True indicating point is enclosed and False otherwise
        """
        assert isinstance(point_list,
                          ValidTypes.ARRAY), '<Lanelet/contains_points>: provided list of points is not a list! type ' \
                                             '= {}'.format(type(point_list))
        assert is_valid_polyline(
            point_list), 'Lanelet/contains_points>: provided list of points is malformed! points = {}'.format(
            point_list)

        # output list
        res = list()

        # get polygon shape of lanelet
        poly = self.convert_to_polygon()
        for p in point_list:
            res.append(poly.contains_point(p))

        return res

    def get_obstacles(self, obstacles: List[Obstacle], time_step: int = 0) -> List[Obstacle]:
        """
        Returns the subset of obstacles,  which are located in the lanelet,  of a given candidate set

        :param obstacles: The set of obstacle candidates
        :param time_step: The time step for the occupancy to check
        :return:
        """

        assert isinstance(obstacles, list) and all(isinstance(o, Obstacle) for o in
                                                   obstacles), '<Lanelet/get_obstacles>: Provided list of obstacles' \
                                                               ' is malformed! obstacles = {}'.format(
            obstacles)

        # output list
        res = list()

        # look at each obstacle
        for o in obstacles:
            o_shape = o.occupancy_at_time(time_step).shape

            # vertices to check
            vertices = list()

            # distinguish between shape and shapegroup and extract vertices
            if isinstance(o_shape, ShapeGroup):
                for sh in o_shape.shapes:
                    # distinguish between type of shape (circle has no vertices)
                    if isinstance(sh, Circle):
                        vertices.append(sh.center)
                    else:
                        vertices.append(sh.vertices)
                        vertices = np.append(vertices, [o_shape.center], axis=0)
            else:
                # distinguish between type of shape (circle has no vertices)
                if isinstance(o_shape, Circle):
                    vertices = o_shape.center
                else:
                    vertices = o_shape.vertices
                    vertices = np.append(vertices, [o_shape.center], axis=0)

            # check if obstacle is in lane
            if any(self.contains_points(np.array(vertices))):
                res.append(o)

        return res

    @staticmethod
    def _merge_static_obstacles_on_lanelet(obstacles_on_lanelet1: Set[int], obstacles_on_lanelet2: Set[int]):
        """
        Merges obstacle IDs of static obstacles on two lanelets

        :param obstacles_on_lanelet1: Obstacle IDs on the first lanelet
        :param obstacles_on_lanelet2: Obstacle IDs on the second lanelet
        :return: Merged obstacle IDs of static obstacles on lanelets
        """
        for obs_id in obstacles_on_lanelet2:
            if obs_id not in obstacles_on_lanelet1:
                obstacles_on_lanelet1.add(obs_id)
        return obstacles_on_lanelet1

    @staticmethod
    def _merge_dynamic_obstacles_on_lanelet(obstacles_on_lanelet1: Dict[int, Set[int]],
                                            obstacles_on_lanelet2: Dict[int, Set[int]]):
        """
        Merges obstacle IDs of static obstacles on two lanelets

        :param obstacles_on_lanelet1: Obstacle IDs on the first lanelet
        :param obstacles_on_lanelet2: Obstacle IDs on the second lanelet
        :return: Merged obstacle IDs of static obstacles on lanelets
        """
        if len(obstacles_on_lanelet2.items()) > 0:
            for time_step, ids in obstacles_on_lanelet2.items():
                for obs_id in ids:
                    if obstacles_on_lanelet1.get(time_step) is not None:
                        if obs_id not in obstacles_on_lanelet1[time_step]:
                            obstacles_on_lanelet1[time_step].add(obs_id)
                    else:
                        obstacles_on_lanelet1[time_step] = {obs_id}

        return obstacles_on_lanelet1

    @classmethod
    def merge_lanelets(cls, lanelet1: 'Lanelet', lanelet2: 'Lanelet') -> 'Lanelet':
        """
        Merges two lanelets which are in predecessor-successor relation

        :param lanelet1: The first lanelet
        :param lanelet2: The second lanelet
        :return: Merged lanelet (predecessor => successor)
        """
        assert isinstance(lanelet1, Lanelet), '<Lanelet/merge_lanelets>: lanelet1 is not a valid lanelet object!'
        assert isinstance(lanelet2, Lanelet), '<Lanelet/merge_lanelets>: lanelet1 is not a valid lanelet object!'
        # check connection via successor / predecessor
        assert lanelet1.lanelet_id in lanelet2.successor or lanelet2.lanelet_id in lanelet1.successor \
            or lanelet1.lanelet_id in lanelet2.predecessor or lanelet2.lanelet_id in lanelet1.predecessor,\
            '<Lanelet/merge_lanelets>: cannot merge two not connected lanelets! successors of l1 = {}, successors ' \
            'of l2 = {}'.format(lanelet1.successor, lanelet2.successor)

        # check pred and successor
        if lanelet1.lanelet_id in lanelet2.predecessor or lanelet2.lanelet_id in lanelet1.successor:
            pred = lanelet1
            suc = lanelet2
        else:
            pred = lanelet2
            suc = lanelet1

        # build new merged lanelet (remove first node of successor if both lanes are connected)
        # check connectedness
        if np.isclose(pred.left_vertices[-1], suc.left_vertices[0]).all():
            idx = 1
        else:
            idx = 0

        # create new lanelet
        left_vertices = np.concatenate((pred.left_vertices, suc.left_vertices[idx:]))
        right_vertices = np.concatenate((pred.right_vertices, suc.right_vertices[idx:]))
        center_vertices = np.concatenate((pred.center_vertices, suc.center_vertices[idx:]))
        lanelet_id = int(str(pred.lanelet_id) + str(suc.lanelet_id))
        predecessor = pred.predecessor
        successor = suc.successor
        static_obstacles_on_lanelet = cls._merge_static_obstacles_on_lanelet(lanelet1.static_obstacles_on_lanelet,
                                                                             lanelet2.static_obstacles_on_lanelet)
        dynamic_obstacles_on_lanelet = cls._merge_dynamic_obstacles_on_lanelet(lanelet1.dynamic_obstacles_on_lanelet,
                                                                               lanelet2.dynamic_obstacles_on_lanelet)

        new_lanelet = Lanelet(left_vertices, center_vertices, right_vertices, lanelet_id, predecessor=predecessor,
                       successor=successor)
        new_lanelet.static_obstacles_on_lanelet = static_obstacles_on_lanelet
        new_lanelet.dynamic_obstacles_on_lanelet = dynamic_obstacles_on_lanelet
        return new_lanelet

    @classmethod
    def all_lanelets_by_merging_successors_from_lanelet(cls, lanelet: 'Lanelet', network: 'LaneletNetwork',
                                                        max_length: float = 150.0, lanelet_type: LaneletType = None) \
            -> Tuple[List['Lanelet'], List[List[int]]]:
        """
        Computes all reachable lanelets starting from a provided lanelet
        and merges them to a single lanelet for each route.

        :param lanelet: The lanelet to start from
        :param network: The network which contains all lanelets
        :param max_length: maxmimal length of merged lanelets can be provided
        :param lanelet_type: allowed type of lanelets which should be merged
        :return: List of merged lanelets, Lists of lanelet ids of which each merged lanelet consists
        """
        assert isinstance(lanelet, Lanelet), '<Lanelet>: provided lanelet is not a valid Lanelet!'
        assert isinstance(network, LaneletNetwork), '<Lanelet>: provided lanelet network is not a ' \
                                                    'valid lanelet network!'
        assert network.find_lanelet_by_id(lanelet.lanelet_id) is not None, '<Lanelet>: lanelet not ' \
                                                                           'contained in network!'

        if lanelet.successor is None or len(lanelet.successor) == 0:
            return [lanelet], [[lanelet.lanelet_id]]

        # Create Graph from network
        Net = nx.DiGraph() 
        lanelets = network._lanelets.values()
        leafs = list()
        for elements in lanelets:
            Net.add_node(elements)
            if elements.successor and not lanelet.lanelet_id in elements.successor:
                for successors in elements.successor:
                    successor = network.find_lanelet_by_id(successors)
                    if lanelet_type is None or lanelet_type in successor.lanelet_type:
                        Net.add_edge(elements, successor)
            # Find leave Nodes
            else:
                leafs.append(elements)

        merge_jobs = list()

        # Get start node for search
        start = network.find_lanelet_by_id(lanelet.lanelet_id)

        # Calculate all paths (i.e. id sequences) to leaf nodes
        for leaf in leafs:
            path = nx.all_simple_paths(Net, start, leaf)
            path = list(path)
            if len(path) < 2 and len(path) > 0:
                merge_jobs.append(path)
            else:
                for i in range(len(path)):

                    merge_jobs.append([path[i]])


        # Create merged lanelets from paths
        merged_lanelets = list()
        merge_jobs_final = []
        for i in range(len(merge_jobs)):
            for j in merge_jobs[i]:
                pred = j[0]
                tmp_length = 0.0
                merge_jobs_tmp = [pred.lanelet_id]
                for k in range(1, len(j)):
                    merge_jobs_tmp.append(j[k].lanelet_id)
                    if k > 0:
                        # do not consider length of inital lanelet for conservativeness
                        tmp_length += j[k].distance[-1]
                    pred = Lanelet.merge_lanelets(pred,j[k])
                    if tmp_length >= max_length:
                        break

                merge_jobs_final.append(merge_jobs_tmp)
            merged_lanelets.append(pred)
            
        return merged_lanelets, merge_jobs_final

    def add_dynamic_obstacle_to_lanelet(self, obstacle_id: int, time_step: int):
        """
        Adds a dynamic obstacle ID to lanelet

        :param obstacle_id: obstacle ID to add
        :param time_step: time step at which the obstacle should be added
        """
        if self.dynamic_obstacles_on_lanelet.get(time_step) is None:
            self.dynamic_obstacles_on_lanelet[time_step] = set()
        self.dynamic_obstacles_on_lanelet[time_step].add(obstacle_id)

    def add_static_obstacle_to_lanelet(self, obstacle_id: int):
        """
        Adds a static obstacle ID to lanelet

        :param obstacle_id: obstacle ID to add
        """
        self.static_obstacles_on_lanelet.add(obstacle_id)

    def add_traffic_sign_to_lanelet(self, traffic_sign_id: int):
        """
        Adds a traffic sign ID to lanelet

        :param traffic_sign_id: traffic sign ID to add
        """
        self.traffic_signs.add(traffic_sign_id)

    def add_traffic_light_to_lanelet(self, traffic_light_id: int):
        """
        Adds a traffic light ID to lanelet

        :param traffic_light_id: traffic light ID to add
        """
        self.traffic_lights.add(traffic_light_id)

    def dynamic_obstacle_by_time_step(self, time_step) -> Set[int]:
        """
        Returns all dynamic obstacles on lanelet at specific time step

        :param time_step: time step of interest
        :returns: list of obstacle IDs
        """
        if self.dynamic_obstacles_on_lanelet.get(time_step) is not None:
            return self.dynamic_obstacles_on_lanelet.get(time_step)
        else:
            return set()

    def __str__(self):
        return 'Lanelet with id:' + str(self.lanelet_id)


class LaneletNetwork:
    """
    Class which represents a network of connected lanelets
    """

    def __init__(self):
        self._lanelets: Dict[int, Lanelet] = {}
        self._intersections: Dict[int, Intersection] = {}
        self._traffic_signs: Dict[int, TrafficSign] = {}
        self._traffic_lights: Dict[int, TrafficLight] = {}

    @property
    def lanelets(self) -> List[Lanelet]:
        return list(self._lanelets.values())

    @lanelets.setter
    def lanelets(self, lanelets: list):
        warnings.warn('<LaneletNetwork/lanelets>: lanelets of network are immutable')

    @property
    def intersections(self) -> List[Intersection]:
        return list(self._intersections.values())

    @property
    def traffic_signs(self) -> List[TrafficSign]:
        return list(self._traffic_signs.values())

    @property
    def traffic_lights(self) -> List[TrafficLight]:
        return list(self._traffic_lights.values())

    @property
    def map_inc_lanelets_to_intersections(self) -> Dict[int,Intersection]:
        """
        :returns: dict that maps lanelet ids to the intersection of which it is an incoming lanelet.
        """
        return {l_id: intersection for intersection in self.intersections
                for l_id in list(intersection.map_incoming_lanelets.keys())}

    @classmethod
    def create_from_lanelet_list(cls, lanelets: list, cleanup_ids: bool=False):
        """
        Creates a LaneletNetwork object from a given list of lanelets

        :param lanelets: The list of lanelets
        :param cleanup_ids: cleans up unused ids
        :return: The LaneletNetwork for the given list of lanelets
        """
        assert isinstance(lanelets, list) and all(isinstance(l, Lanelet) for l in
                                                  lanelets), '<LaneletNetwork/create_from_lanelet_list>:' \
                                                             'Provided list of lanelets is not valid! ' \
                                                             'lanelets = {}'.format(lanelets)

        # create lanelet network
        lanelet_network = cls()

        # add each lanelet to the lanelet network
        for l in lanelets:
            lanelet_network.add_lanelet(copy.deepcopy(l))

        if cleanup_ids:
            lanelet_network.cleanup_lanelet_references()
        return lanelet_network

    @classmethod
    def create_from_lanelet_network(cls, lanelet_network: 'LaneletNetwork'):
        """
        Creates a lanelet network from a given lanelet network (copy)

        :param lanelet_network: The existing lanelet network
        :return: The deep copy of the lanelet network
        """
        new_lanelet_network = cls()
        for l in lanelet_network.lanelets:
            new_lanelet_network.add_lanelet(copy.deepcopy(l))
        return new_lanelet_network

    def cleanup_lanelet_references(self):
        """
        Deletes ids which do not exist in the lanelet network. Useful when cutting out lanelet networks.
        :return:
        """
        exisiting_ids = set(self._lanelets.keys())
        for l in self.lanelets:
            l._predecessor = list(set(l._predecessor).intersection(exisiting_ids))
            l._successor = list(set(l._successor).intersection(exisiting_ids))
            l._adj_left = None if l._adj_left is None or l._adj_left not in exisiting_ids else l._adj_left
            prev = copy.deepcopy(l._adj_left_same_direction)
            l._adj_left_same_direction = None if l._adj_left_same_direction is None \
                                                 or l.adj_left not in exisiting_ids \
                else l._adj_left_same_direction
            l._adj_right = None if l._adj_right is None or l._adj_right not in exisiting_ids else l._adj_right
            prev = copy.deepcopy(l._adj_right_same_direction)
            l._adj_right_same_direction = None if l._adj_right_same_direction is None \
                                                  or l.adj_right not in exisiting_ids \
                else l._adj_right_same_direction

    def find_lanelet_by_id(self, lanelet_id: int) -> Lanelet:
        """
        Finds a lanelet for a given lanelet_id

        :param lanelet_id: The id of the lanelet to find
        :return: The lanelet object if the id exists and None otherwise
        """
        assert is_natural_number(
            lanelet_id), '<LaneletNetwork/find_lanelet_by_id>: provided id is not valid! id = {}'.format(lanelet_id)

        return self._lanelets[lanelet_id] if lanelet_id in self._lanelets else None

    def find_traffic_sign_by_id(self, traffic_sign_id: int) -> TrafficSign:
        """
        Finds a traffic sign for a given traffic_sign_id

        :param traffic_sign_id: The id of the traffic sign to find
        :return: The traffic sign object if the id exists and None otherwise
        """
        assert is_natural_number(
            traffic_sign_id), '<LaneletNetwork/find_traffic_sign_by_id>: provided id is not valid! ' \
                              'id = {}'.format(traffic_sign_id)

        return self._traffic_signs[traffic_sign_id] if traffic_sign_id in self._traffic_signs else None

    def find_traffic_light_by_id(self, traffic_light_id: int) -> TrafficLight:
        """
        Finds a traffic light for a given traffic_light_id

        :param traffic_light_id: The id of the traffic light to find
        :return: The traffic light object if the id exists and None otherwise
        """
        assert is_natural_number(
            traffic_light_id), '<LaneletNetwork/find_traffic_light_by_id>: provided id is not valid! ' \
                              'id = {}'.format(traffic_light_id)

        return self._traffic_lights[traffic_light_id] if traffic_light_id in self._traffic_lights else None

    def find_intersection_by_id(self, intersection_id: int) -> Intersection:
        """
        Finds a intersection for a given intersection_id

        :param intersection_id: The id of the intersection to find
        :return: The intersection object if the id exists and None otherwise
        """
        assert is_natural_number(intersection_id), '<LaneletNetwork/find_intersection_by_id>: ' \
                                                   'provided id is not valid! id = {}'.format(intersection_id)

        return self._intersections[intersection_id] if intersection_id in self._intersections else None

    def add_lanelet(self, lanelet: Lanelet):
        """
        Adds a lanelet to the LaneletNetwork

        :param lanelet: The lanelet to add
        :return: True if the lanelet has successfully been added to the network, false otherwise
        """

        assert isinstance(lanelet, Lanelet), '<LaneletNetwork/add_lanelet>: provided lanelet is not of ' \
                                             'type lanelet! type = {}'.format(type(lanelet))

        # check if lanelet already exists in network and warn user
        if lanelet.lanelet_id in self._lanelets.keys():
            warnings.warn('Lanelet already exists in network! No changes are made.')
            return False
        else:
            self._lanelets[lanelet.lanelet_id] = lanelet
            return True

    def add_traffic_sign(self, traffic_sign: TrafficSign, lanelet_ids: Set[int]):
        """
        Adds a traffic sign to the LaneletNetwork

        :param traffic_sign: The traffic sign to add
        :param lanelet_ids: Lanelets the traffic sign should be referenced from
        :return: True if the traffic sign has successfully been added to the network, false otherwise
        """

        assert isinstance(traffic_sign, TrafficSign), '<LaneletNetwork/add_traffic_sign>: provided traffic sign is ' \
                                                      'not of type traffic_sign! type = {}'.format(type(traffic_sign))

        # check if traffic already exists in network and warn user
        if traffic_sign.traffic_sign_id in self._traffic_signs.keys():
            warnings.warn('Traffic sign with ID {} already exists in network! '
                          'No changes are made.'.format(traffic_sign.traffic_sign_id))
            return False
        else:
            self._traffic_signs[traffic_sign.traffic_sign_id] = traffic_sign
            for lanelet_id in lanelet_ids:
                lanelet = self.find_lanelet_by_id(lanelet_id)
                if lanelet is not None:
                    lanelet.add_traffic_sign_to_lanelet(traffic_sign.traffic_sign_id)
                else:
                    warnings.warn('Traffic sign cannot be referenced to lanelet because the lanelet does not exist.')
            return True

    def add_traffic_light(self, traffic_light: TrafficLight, lanelet_ids: Set[int]):
        """
        Adds a traffic light to the LaneletNetwork

        :param traffic_light: The traffic light to add
        :param lanelet_ids: Lanelets the traffic sign should be referenced from
        :return: True if the traffic light has successfully been added to the network, false otherwise
        """

        assert isinstance(traffic_light, TrafficLight), '<LaneletNetwork/add_traffic_light>: provided traffic light ' \
                                                        'is not of type traffic_light! ' \
                                                        'type = {}'.format(type(traffic_light))

        # check if traffic already exists in network and warn user
        if traffic_light.traffic_light_id in self._traffic_lights.keys():
            warnings.warn('Traffic light already exists in network! No changes are made.')
            return False
        else:
            self._traffic_lights[traffic_light.traffic_light_id] = traffic_light
            for lanelet_id in lanelet_ids:
                lanelet = self.find_lanelet_by_id(lanelet_id)
                if lanelet is not None:
                    lanelet.add_traffic_light_to_lanelet(traffic_light.traffic_light_id)
                else:
                    warnings.warn('Traffic light cannot be referenced to lanelet because the lanelet does not exist.')
            return True

    def add_intersection(self, intersection: Intersection):
        """
        Adds a intersection to the LaneletNetwork

        :param intersection: The intersection to add
        :return: True if the traffic light has successfully been added to the network, false otherwise
        """

        assert isinstance(intersection, Intersection), '<LaneletNetwork/add_intersection>: provided intersection is ' \
                                                       'not of type Intersection! type = {}'.format(type(intersection))

        # check if traffic already exists in network and warn user
        if intersection.intersection_id in self._intersections.keys():
            warnings.warn('Intersection already exists in network! No changes are made.')
            return False
        else:
            self._intersections[intersection.intersection_id] = intersection
            return True

    def add_lanelets_from_network(self, lanelet_network: 'LaneletNetwork'):
        """
        Adds lanelets from a given network object to the current network

        :param lanelet_network: The lanelet network
        :return: True if all lanelets have been added to the network, false otherwise
        """
        flag = True

        # add lanelets to the network
        for l in lanelet_network.lanelets:
            flag = flag and self.add_lanelet(l)

        return flag

    def translate_rotate(self, translation: np.ndarray, angle: float):
        """
        Translates and rotates the complete lanelet network

        :param translation: The translation given as [x_off,y_off] for the x and y translation
        :param angle: The rotation angle in radian (counter-clockwise defined)
        """

        assert is_real_number_vector(translation,
                                     2), '<LaneletNetwork/translate_rotate>: provided translation is not valid! ' \
                                         'translation = {}'.format(translation)
        assert is_valid_orientation(
            angle), '<LaneletNetwork/translate_rotate>: provided angle is not valid! angle = {}'.format(angle)

        # rotate each lanelet
        for lanelet in self._lanelets.values():
            lanelet.translate_rotate(translation, angle)
        for traffic_sign in self._traffic_signs.values():
            traffic_sign.translate_rotate(translation, angle)
        for traffic_light in self._traffic_lights.values():
            traffic_light.translate_rotate(translation, angle)

    def find_lanelet_by_position(self, point_list: List[np.ndarray]) -> List[List[int]]:
        """
        Finds the lanelet id of a given position

        :param point_list: The list of positions to check
        :return: A list of lanelet ids. If the position could not be matched to a lanelet, an empty list is returned
        """
        assert isinstance(point_list,
                          ValidTypes.LISTS), '<Lanelet/contains_points>: provided list of points is not a list! ' \
                                             'type = {}'.format(type(point_list))
        # assert is_valid_polyline(
        #     point_list), 'Lanelet/contains_points>: provided list of points is malformed! points = {}'.format(
        #     point_list)

        # output list
        res = list()

        # look at each lanelet
        polygons = [(l.lanelet_id, l.convert_to_polygon()) for l in self.lanelets]

        for point in point_list:
            mapped = list()
            for lanelet_id, poly in polygons:
                if poly.contains_point(point):
                    mapped.append(lanelet_id)
            res.append(mapped)

        return res

    def find_lanelet_by_shape(self, shape: Shape) -> List[int]:
        """
        Finds the lanelet id of a given shape

        :param shape: The shape to check
        :return: A list of lanelet ids. If the position could not be matched to a lanelet, an empty list is returned
        """
        assert isinstance(shape, (Circle, Polygon, Rectangle)), '<Lanelet/find_lanelet_by_shape>: ' \
                                                                'provided shape is not a shape! ' \
                                                                'type = {}'.format(type(shape))

        # output list
        res = []

        # look at each lanelet
        polygons = [(l.lanelet_id, l.convert_to_polygon()) for l in self.lanelets]


        for lanelet_id, poly in polygons:
            if poly.shapely_object.intersects(shape.shapely_object):
                res.append(lanelet_id)

        return res

    def filter_obstacles_in_network(self, obstacles: List[Obstacle]) -> List[Obstacle]:
        """
        Returns the list of obstacles which are located in the lanelet network

        :param obstacles: The list of obstacles to check
        :return: The list of obstacles which are located in the lanelet network
        """

        res = list()

        map = self.map_obstacles_to_lanelets(obstacles)

        for k in map.keys():
            obs = map[k]
            for o in obs:
                if o not in res:
                    res.append(o)

        return res

    def map_obstacles_to_lanelets(self, obstacles: List[Obstacle]) -> Dict[int, List[Obstacle]]:
        """
        Maps a given list of obstacles to the lanelets of the lanelet network

        :param obstacles: The list of CR obstacles
        :return: A dictionary with the lanelet id as key and the list of obstacles on the lanelet as a List[Obstacles]
        """
        mapping = {}

        for l in self.lanelets:
            # map obstacles to current lanelet
            mapped_objs = l.get_obstacles(obstacles)

            # check if mapping is not empty
            if len(mapped_objs) > 0:
                mapping[l.lanelet_id] = mapped_objs

        return mapping

    def lanelets_in_proximity(self, point: list, radius: float) -> List[Lanelet]:
        """
        Finds all lanelets which intersect a given circle, defined by the center point and radius

        :param point: The center of the circle
        :param radius: The radius of the circle
        :return: The list of lanelets which intersect the given circle
        """

        assert is_real_number_vector(point,
                                     length=2), '<LaneletNetwork/lanelets_in_proximity>: provided point is ' \
                                                'not valid! point = {}'.format(point)
        assert is_positive(
            radius), '<LaneletNetwork/lanelets_in_proximity>: provided radius is not valid! radius = {}'.format(radius)

        # get list of lanelet ids
        ids = self._lanelets.keys()

        # output list
        lanes = dict()

        rad_sqr = radius ** 2

        # distance dict for sorting
        distance_list = list()

        # go through list of lanelets
        for i in ids:

            # if current lanelet has not already been added to lanes list
            if i not in lanes:
                lanelet = self.find_lanelet_by_id(i)

                # compute distances (we are not using the sqrt for computational effort)
                distance = (lanelet.center_vertices - point) ** 2.
                distance = distance[:, 0] + distance[:, 1]

                # check if at least one distance is smaller than the radius
                if any(np.greater_equal(rad_sqr, distance)):
                    lanes[i] = self.find_lanelet_by_id(i)
                    distance_list.append(np.min(distance))

                # check if adjacent lanelets can be added as well
                index_minDist = np.argmin(distance - rad_sqr)

                # check right side of lanelet
                if lanelet.adj_right is not None:
                    p = (lanelet.right_vertices[index_minDist, :] - point) ** 2
                    p = p[0] + p[1]
                    if np.greater(rad_sqr, p) and lanelet.adj_right not in lanes:
                        lanes[lanelet.adj_right] = self.find_lanelet_by_id(lanelet.adj_right)
                        distance_list.append(p)

                # check left side of lanelet
                if lanelet.adj_left is not None:
                    p = (lanelet.left_vertices[index_minDist, :] - point) ** 2
                    p = p[0] + p[1]
                    if np.greater(rad_sqr, p) and lanelet.adj_left not in lanes:
                        lanes[lanelet.adj_left] = self.find_lanelet_by_id(lanelet.adj_left)
                        distance_list.append(p)

        # sort list according to distance
        indices = np.argsort(distance_list)
        lanelets = list(lanes.values())

        # return sorted list
        return [lanelets[i] for i in indices]

    def __str__(self):
        return_str = ''
        for lanelet_id in self._lanelets.keys():
            return_str += '{:8d} lanelet\n'.format(lanelet_id)
        return return_str
