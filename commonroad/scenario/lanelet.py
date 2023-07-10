import copy
import enum
from collections import defaultdict
from typing import *

import numpy as np
from datetime import datetime
from shapely.geometry import Point as ShapelyPoint
from shapely.geometry import Polygon as ShapelyPolygon
from shapely.strtree import STRtree

import commonroad.geometry.transform
from commonroad.common.util import subtract_orientations, Time
from commonroad.common.validity import *
from commonroad.geometry.shape import Polygon, ShapeGroup, Circle, Rectangle, Shape
from commonroad.scenario.intersection import Intersection
from commonroad.scenario.obstacle import Obstacle
from commonroad.scenario.state import TraceState
from commonroad.scenario.traffic_sign import TrafficSign
from commonroad.scenario.traffic_light import TrafficLight
from commonroad.scenario.area import Area
from commonroad.visualization.drawable import IDrawable
from commonroad.visualization.renderer import IRenderer
from commonroad.visualization.draw_params import OptionalSpecificOrAllDrawParams, LaneletNetworkParams
from commonroad.common.common_lanelet import RoadUser, StopLine, LineMarking, LaneletType


__author__ = "Christian Pek, Sebastian Maierhofer"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["BMW CAR@TUM"]
__version__ = "2023.1"
__maintainer__ = "Sebastian Maierhofer"
__email__ = "commonroad@lists.lrz.de"
__status__ = "released"


class Lanelet:
    """
    Class which describes a Lanelet entity according to the CommonRoad specification. Each lanelet is described by a
    left and right boundary (polylines). Furthermore, lanelets have relations to other lanelets, e.g. an adjacent left
    neighbor or a predecessor.
    """

    def __init__(self, left_vertices: np.ndarray, center_vertices: np.ndarray, right_vertices: np.ndarray,
                 lanelet_id: int, predecessor=None, successor=None, adjacent_left=None,
                 adjacent_left_same_direction=None, adjacent_right=None, adjacent_right_same_direction=None,
                 line_marking_left_vertices=LineMarking.NO_MARKING, line_marking_right_vertices=LineMarking.NO_MARKING,
                 stop_line=None, lanelet_type=None, user_one_way=None, user_bidirectional=None, traffic_signs=None,
                 traffic_lights=None, adjacent_areas=None):
        """
        Constructor of a Lanelet object
        :param left_vertices: The vertices of the left boundary of the Lanelet described as a
        polyline [[x0,y0],[x1,y1],...,[xn,yn]]
        :param center_vertices: The vertices of the center line of the Lanelet described as a
        polyline [[x0,y0],[x1,y1],...,[xn,yn]]
        :param right_vertices: The vertices of the right boundary of the Lanelet described as a
        polyline [[x0,y0],[x1,y1],...,[xn,yn]]
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
        :param adjacent_areas: Areas that are adjacent to the lanelet
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

        self._distance = None
        self._inner_distance = None
        # create empty polygon
        self._polygon = Polygon(np.concatenate((self.right_vertices, np.flip(self.left_vertices, 0))))

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

        self._adjacent_areas = None
        if adjacent_areas is None:
            self._adjacent_areas = set()
        else:
            self.adjacent_areas = adjacent_areas

    def __eq__(self, other):
        if not isinstance(other, Lanelet):
            warnings.warn(f"Inequality between Lanelet {repr(self)} and different type {type(other)}")
            return False

        list_elements_eq = self._stop_line == other.stop_line

        lanelet_eq = True
        polylines = [self._left_vertices, self._right_vertices, self._center_vertices]
        polylines_other = [other.left_vertices, other.right_vertices, other.center_vertices]

        for i in range(0, len(polylines)):
            polyline = polylines[i]
            polyline_other = polylines_other[i]
            polyline_string = np.array2string(np.around(polyline.astype(float), 10), precision=10)
            polyline_other_string = np.array2string(np.around(polyline_other.astype(float), 10), precision=10)
            lanelet_eq = lanelet_eq and polyline_string == polyline_other_string

        if lanelet_eq and self.lanelet_id == other.lanelet_id and self._line_marking_left_vertices == \
                other.line_marking_left_vertices and self._line_marking_right_vertices == \
                other.line_marking_right_vertices and set(
                self._predecessor) == set(other.predecessor) and set(self._successor) == set(
                other.successor) and self._adj_left == other.adj_left and self._adj_right == other.adj_right and \
                self._adj_left_same_direction == other.adj_left_same_direction and self._adj_right_same_direction == \
                other.adj_right_same_direction and self._lanelet_type == other.lanelet_type and self._user_one_way ==\
                self.user_one_way and self._user_bidirectional == other.user_bidirectional and self._traffic_signs ==\
                other.traffic_signs and self._traffic_lights == other.traffic_lights and self._adjacent_areas == \
                other.adjacent_areas:
            return list_elements_eq

        warnings.warn(f"Inequality of Lanelet {repr(self)} and the other one {repr(other)}")
        return False

    def __hash__(self):
        polylines = [self._left_vertices, self._right_vertices, self._center_vertices]
        polyline_strings = []
        for polyline in polylines:
            polyline_string = np.array2string(np.around(polyline.astype(float), 10), precision=10)
            polyline_strings.append(polyline_string)

        elements = [self._predecessor, self._successor, self._lanelet_type, self._user_one_way,
                    self._user_bidirectional, self._traffic_signs, self._traffic_lights, self._adjacent_areas]
        frozen_elements = [frozenset(e) for e in elements]

        return hash((self._lanelet_id, tuple(polyline_strings), self._line_marking_left_vertices,
                     self._line_marking_right_vertices, self._stop_line, self._adj_left, self._adj_right,
                     self._adj_left_same_direction, self._adj_right_same_direction, tuple(frozen_elements)))

    def __str__(self):
        return f"Lanelet with id {self._lanelet_id} has predecessors {set(self._predecessor)}, successors " \
               f"{set(self._successor)}, left adjacency {self._adj_left} with " \
               f"{'same' if self._adj_left_same_direction else 'opposite'} direction, and " \
               f"right adjacency with {'same' if self._adj_right_same_direction else 'opposite'} direction"

    def __repr__(self):
        return f"Lanelet(left_vertices={self._left_vertices.tolist()}, " \
               f"center_vertices={self._center_vertices.tolist()}, " \
               f"right_vertices={self._right_vertices.tolist()}, lanelet_id={self._lanelet_id}, " \
               f"predecessor={self._predecessor}, successor={self._successor}, adjacent_left={self._adj_left}, " \
               f"adjacent_left_same_direction={self._adj_left_same_direction}, adjacent_right={self._adj_right}, " \
               f"adjacent_right_same_direction={self._adj_right_same_direction}, " \
               f"line_marking_left_vertices={self._line_marking_left_vertices}, " \
               f"line_marking_right_vertices={self._line_marking_right_vertices}), " \
               f"stop_line={repr(self._stop_line)}, lanelet_type={self._lanelet_type}, " \
               f"user_one_way={self._user_one_way}, " \
               f"user_bidirectional={self._user_bidirectional}, traffic_signs={self._traffic_signs}, " \
               f"traffic_lights={self._traffic_lights}, "\
               f"adjacent_areas={self._adjacent_areas}"

    @property
    def distance(self) -> np.ndarray:
        """
        cumulative distance along center vertices
        """
        if self._distance is None:
            self._distance = self._compute_polyline_cumsum_dist([self.center_vertices])
        return self._distance

    @distance.setter
    def distance(self, distance: np.ndarray):
        self._distance = distance

    @property
    def inner_distance(self) -> np.ndarray:
        """
        minimum cumulative distance along left and right vertices, i.e., along the inner curve:
        """
        if self._inner_distance is None:
            self._inner_distance = self._compute_polyline_cumsum_dist([self.left_vertices, self.right_vertices])

        return self._inner_distance

    @property
    def lanelet_id(self) -> int:
        return self._lanelet_id

    @lanelet_id.setter
    def lanelet_id(self, l_id: int):
        assert is_natural_number(l_id), '<Lanelet/lanelet_id>: Provided lanelet_id is not valid! id={}'.format(l_id)
        self._lanelet_id = l_id

    @property
    def left_vertices(self) -> np.ndarray:
        return self._left_vertices

    @left_vertices.setter
    def left_vertices(self, polyline: np.ndarray):
        """
        If the left vertices are strongly modified, the STRtree cannot be valid anymore!
        """
        assert is_valid_polyline(polyline), '<Lanelet/left_vertices>: The provided polyline ' \
                                            'is not valid! id = {} polyline = {}'.format(self._lanelet_id, polyline)
        self._left_vertices = polyline

    @property
    def right_vertices(self) -> np.ndarray:
        return self._right_vertices

    @right_vertices.setter
    def right_vertices(self, polyline: np.ndarray):
        """
        If the right vertices are strongly modified, the STRtree cannot be valid anymore!
        """
        assert is_valid_polyline(polyline), '<Lanelet/right_vertices>: The provided polyline ' \
                                            'is not valid! id = {}, polyline = {}'.format(self._lanelet_id, polyline)
        self._right_vertices = polyline

    @staticmethod
    def _compute_polyline_cumsum_dist(polylines: List[np.ndarray], comparator=np.amin):
        d = []
        for polyline in polylines:
            d.append(np.diff(polyline, axis=0))
        segment_distances = np.empty((len(polylines[0]), len(polylines)))
        for i, d_tmp in enumerate(d):
            segment_distances[:, i] = np.append([0], np.sqrt((np.square(d_tmp)).sum(axis=1)))

        return np.cumsum(comparator(segment_distances, axis=1))

    @property
    def center_vertices(self) -> np.ndarray:
        return self._center_vertices

    @center_vertices.setter
    def center_vertices(self, polyline: np.ndarray):
        assert is_valid_polyline(
            polyline), '<Lanelet/center_vertices>: The provided polyline is not valid! polyline = {}'.format(polyline)
        self._center_vertices = polyline

    @property
    def line_marking_left_vertices(self) -> LineMarking:
        return self._line_marking_left_vertices

    @line_marking_left_vertices.setter
    def line_marking_left_vertices(self, line_marking_left_vertices: LineMarking):
        assert isinstance(line_marking_left_vertices,
                          LineMarking), '<Lanelet/line_marking_left_vertices>: Provided lane marking type of ' \
                                        'left boundary is not valid! type = {}'.format(type(line_marking_left_vertices))
        self._line_marking_left_vertices = line_marking_left_vertices

    @property
    def line_marking_right_vertices(self) -> LineMarking:
        return self._line_marking_right_vertices

    @line_marking_right_vertices.setter
    def line_marking_right_vertices(self, line_marking_right_vertices: LineMarking):
        assert isinstance(line_marking_right_vertices,
                          LineMarking), '<Lanelet/line_marking_right_vertices>: Provided lane marking type of ' \
                                        'right boundary is not valid! type = {}'.format(
            type(line_marking_right_vertices))
        self._line_marking_right_vertices = line_marking_right_vertices

    @property
    def predecessor(self) -> list:
        return self._predecessor

    @predecessor.setter
    def predecessor(self, predecessor: list):
        assert (is_list_of_natural_numbers(predecessor) and len(
            predecessor) >= 0), \
            '<Lanelet/predecessor>: Provided list of predecessors is not valid! predecessors = {}'.format(predecessor)
        self._predecessor = predecessor

    @property
    def successor(self) -> list:
        return self._successor

    @successor.setter
    def successor(self, successor: list):
        assert (is_list_of_natural_numbers(successor) and len(
            successor) >= 0), '<Lanelet/predecessor>: Provided list of successors is not valid! successors = {}'.format(
            successor)
        self._successor = successor

    @property
    def adj_left(self) -> int:
        return self._adj_left

    @adj_left.setter
    def adj_left(self, l_id: int):
        self._adj_left = l_id

    @property
    def adj_left_same_direction(self) -> bool:
        return self._adj_left_same_direction

    @adj_left_same_direction.setter
    def adj_left_same_direction(self, same: bool):
        assert isinstance(same, bool), '<Lanelet/adj_left_same_direction>: provided direction ' \
                                       'is not of type bool! type = {}'.format(type(same))
        self._adj_left_same_direction = same

    @property
    def adj_right(self) -> int:
        return self._adj_right

    @adj_right.setter
    def adj_right(self, l_id: int):
        self._adj_right = l_id

    @property
    def adj_right_same_direction(self) -> bool:
        return self._adj_right_same_direction

    @adj_right_same_direction.setter
    def adj_right_same_direction(self, same: bool):
        assert isinstance(same, bool), '<Lanelet/adj_right_same_direction>: provided direction ' \
                                       'is not of type bool! type = {}'.format(type(same))
        self._adj_right_same_direction = same

    @property
    def dynamic_obstacles_on_lanelet(self) -> Dict[int, Set[int]]:
        return self._dynamic_obstacles_on_lanelet

    @dynamic_obstacles_on_lanelet.setter
    def dynamic_obstacles_on_lanelet(self, obstacle_ids: Dict[int, Set[int]]):
        assert isinstance(obstacle_ids, dict), '<Lanelet/obstacles_on_lanelet>: provided dictionary of ids is not a ' \
                                               'dictionary! type = {}'.format(type(obstacle_ids))
        self._dynamic_obstacles_on_lanelet = obstacle_ids

    @property
    def static_obstacles_on_lanelet(self) -> Union[None, Set[int]]:
        return self._static_obstacles_on_lanelet

    @static_obstacles_on_lanelet.setter
    def static_obstacles_on_lanelet(self, obstacle_ids: Set[int]):
        assert isinstance(obstacle_ids, set), '<Lanelet/obstacles_on_lanelet>: provided list of ids is not a ' \
                                              'set! type = {}'.format(type(obstacle_ids))
        self._static_obstacles_on_lanelet = obstacle_ids

    @property
    def stop_line(self) -> StopLine:
        return self._stop_line

    @stop_line.setter
    def stop_line(self, stop_line: StopLine):
        assert isinstance(stop_line, StopLine), '<Lanelet/stop_line>: ''Provided type is not valid! type = {}'.format(
            type(stop_line))
        self._stop_line = stop_line

    @property
    def lanelet_type(self) -> Set[LaneletType]:
        return self._lanelet_type

    @lanelet_type.setter
    def lanelet_type(self, lanelet_type: Set[LaneletType]):
        assert isinstance(lanelet_type, set) and all(isinstance(elem, LaneletType) for elem in
                                                     lanelet_type), '<Lanelet/lanelet_type>: ''Provided type is not ' \
                                                                    'valid! type = {}, ' \
                                                                    'expected = Set[LaneletType]'.format(
            type(lanelet_type))
        self._lanelet_type = lanelet_type

    @property
    def user_one_way(self) -> Set[RoadUser]:
        return self._user_one_way

    @user_one_way.setter
    def user_one_way(self, user_one_way: Set[RoadUser]):
        assert isinstance(user_one_way, set) and all(isinstance(elem, RoadUser) for elem in
                                                     user_one_way), '<Lanelet/user_one_way>: Provided type is not ' \
                                                                    'valid! type = {}'.format(
            type(user_one_way))
        self._user_one_way = user_one_way

    @property
    def user_bidirectional(self) -> Set[RoadUser]:
        return self._user_bidirectional

    @user_bidirectional.setter
    def user_bidirectional(self, user_bidirectional: Set[RoadUser]):
        assert isinstance(user_bidirectional, set) and all(isinstance(elem, RoadUser) for elem in
                                                           user_bidirectional), \
            '<Lanelet/user_bidirectional>: Provided type is not valid! type = {}'.format(type(user_bidirectional))
        self._user_bidirectional = user_bidirectional

    @property
    def traffic_signs(self) -> Set[int]:
        return self._traffic_signs

    @traffic_signs.setter
    def traffic_signs(self, traffic_sign_ids: Set[int]):
        assert isinstance(traffic_sign_ids, set), '<Lanelet/traffic_signs>: provided list of ids is not a ' \
                                                  'set! type = {}'.format(type(traffic_sign_ids))
        self._traffic_signs = traffic_sign_ids

    @property
    def traffic_lights(self) -> Set[int]:
        return self._traffic_lights

    @traffic_lights.setter
    def traffic_lights(self, traffic_light_ids: Set[int]):
        assert isinstance(traffic_light_ids, set), '<Lanelet/traffic_lights>: provided list of ids is not a ' \
                                                   'set! type = {}'.format(type(traffic_light_ids))
        self._traffic_lights = traffic_light_ids

    @property
    def adjacent_areas(self) -> Set[int]:
        return self._adjacent_areas

    @adjacent_areas.setter
    def adjacent_areas(self, value: Set[int]):
        assert isinstance(value, set), '<Lanelet/adjacent_areas>: provided list of ids is not a ' \
                                                   'set! type = {}'.format(type(value))
        self._adjacent_areas = value

    @property
    def polygon(self) -> Polygon:
        return self._polygon

    def add_predecessor(self, lanelet: int):
        """
        Adds the ID of a predecessor lanelet to the list of predecessors.
        :param lanelet: Predecessor lanelet ID.
        """
        if lanelet not in self.predecessor:
            self.predecessor.append(lanelet)

    def remove_predecessor(self, lanelet: int):
        """
        Removes the ID of a predecessor lanelet from the list of predecessors.
        :param lanelet: Predecessor lanelet ID.
        """
        if lanelet in self.predecessor:
            self.predecessor.remove(lanelet)

    def add_successor(self, lanelet: int):
        """
        Adds the ID of a successor lanelet to the list of successors.
        :param lanelet: Successor lanelet ID.
        """
        if lanelet not in self.successor:
            self.successor.append(lanelet)

    def remove_successor(self, lanelet: int):
        """
        Removes the ID of a successor lanelet from the list of successors.
        :param lanelet: Successor lanelet ID.
        """
        if lanelet in self.successor:
            self.successor.remove(lanelet)

    def translate_rotate(self, translation: np.ndarray, angle: float):
        """
        This method translates and rotates a lanelet

        :param translation: The translation given as [x_off,y_off] for the x and y translation
        :param angle: The rotation angle in radian (counter-clockwise defined)
        """

        assert is_real_number_vector(translation, 2), '<Lanelet/translate_rotate>: provided translation ' \
                                                      'is not valid! translation = {}'.format(translation)
        assert is_valid_orientation(
                angle), '<Lanelet/translate_rotate>: provided angle is not valid! angle = {}'.format(angle)

        # create transformation matrix
        t_m = commonroad.geometry.transform.translation_rotation_matrix(translation, angle)
        # transform center vertices
        tmp = t_m.dot(np.vstack((self.center_vertices.transpose(), np.ones((1, self.center_vertices.shape[0])))))
        tmp = tmp[0:2, :]
        self._center_vertices = tmp.transpose()

        # transform left vertices
        tmp = t_m.dot(np.vstack((self.left_vertices.transpose(), np.ones((1, self.left_vertices.shape[0])))))
        tmp = tmp[0:2, :]
        self._left_vertices = tmp.transpose()

        # transform right vertices
        tmp = t_m.dot(np.vstack((self.right_vertices.transpose(), np.ones((1, self.right_vertices.shape[0])))))
        tmp = tmp[0:2, :]
        self._right_vertices = tmp.transpose()

        # transform the stop line
        if self._stop_line is not None:
            self._stop_line.translate_rotate(translation, angle)

        # recreate polygon in case it existed
        self._polygon = Polygon(np.concatenate((self.right_vertices, np.flip(self.left_vertices, 0))))

    def interpolate_position(self, distance: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray, int]:
        """
        Computes the interpolated positions on the center/right/left polyline of the lanelet for a given distance
        along the lanelet

        :param distance: The distance for the interpolation
        :return: The interpolated positions on the center/right/left polyline and the segment id of the polyline where
            the interpolation takes place in the form ([x_c,y_c],[x_r,y_r],[x_l,y_l], segment_id)
        """
        assert is_real_number(distance) and np.greater_equal(self.distance[-1], distance) and np.greater_equal(distance,
                                                                                                               0), \
            '<Lanelet/interpolate_position>: provided distance is not valid! distance = {}'.format(
                distance)
        idx = np.searchsorted(self.distance, distance) - 1
        while not self.distance[idx] <= distance:
            idx += 1
        r = (distance - self.distance[idx]) / (self.distance[idx + 1] - self.distance[idx])
        return ((1 - r) * self._center_vertices[idx] + r * self._center_vertices[idx + 1],
                (1 - r) * self._right_vertices[idx] + r * self._right_vertices[idx + 1],
                (1 - r) * self._left_vertices[idx] + r * self._left_vertices[idx + 1], idx)

    def convert_to_polygon(self) -> Polygon:
        """
        Converts the given lanelet to a polygon representation

        :return: The polygon of the lanelet
        """
        warnings.warn("Use the lanelet property <polygon> instead", DeprecationWarning)
        return self._polygon

    def contains_points(self, point_list: np.ndarray) -> List[bool]:
        """
        Checks if a list of points is enclosed in the lanelet

        :param point_list: The list of points in the form [[px1,py1],[px2,py2,],...]
        :return: List of Boolean values with True indicating point is enclosed and False otherwise
        """
        assert isinstance(point_list,
                          ValidTypes.ARRAY), '<Lanelet/contains_points>: provided list of points is not a list! type ' \
                                             '= {}'.format(type(point_list))
        assert is_valid_polyline(
                point_list), 'Lanelet/contains_points>: provided list of points is malformed! points = {}'.format(
                point_list)

        return [self._polygon.contains_point(p) for p in point_list]

    def get_obstacles(self, obstacles: List[Obstacle], time_step: int = 0) -> List[Obstacle]:
        """
        Returns the subset of obstacles,  which are located in the lanelet,  of a given candidate set

        :param obstacles: The set of obstacle candidates
        :param time_step: The time step for the occupancy to check
        :return:
        """

        assert isinstance(obstacles, list) and all(
                isinstance(o, Obstacle) for o in obstacles), '<Lanelet/get_obstacles>: Provided list of obstacles' \
                                                             ' is malformed! obstacles = {}'.format(obstacles)

        # output list
        res = list()
        lanelet_shapely_obj = self._polygon.shapely_object
        # look at each obstacle
        for o in obstacles:
            o_shape = o.occupancy_at_time(time_step).shape

            # vertices to check
            shape_shapely_objects = list()

            # distinguish between shape and shape group and extract vertices
            if isinstance(o_shape, ShapeGroup):
                shape_shapely_objects.extend([sh.shapely_object for sh in o_shape.shapes])
            else:
                shape_shapely_objects.append(o_shape.shapely_object)

            # check if obstacle is in lane
            for shapely_obj in shape_shapely_objects:
                if lanelet_shapely_obj.intersects(shapely_obj):
                    res.append(o)
                    break

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
        assert lanelet1.lanelet_id in lanelet2.successor or \
               lanelet2.lanelet_id in lanelet1.successor or \
               lanelet1.lanelet_id in lanelet2.predecessor or \
               lanelet2.lanelet_id in lanelet1.predecessor, '<Lanelet/merge_lanelets>: cannot merge two not ' \
                                                            'connected lanelets! successors of l1 = {}, successors ' \
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
    def all_lanelets_by_merging_successors_from_lanelet(cls, lanelet: 'Lanelet',
                                                        network: 'LaneletNetwork', max_length: float = 150.0) \
            -> Tuple[List['Lanelet'], List[List[int]]]:
        """
        Computes all reachable lanelets starting from a provided lanelet
        and merges them to a single lanelet for each route.

        :param lanelet: The lanelet to start from
        :param network: The network which contains all lanelets
        :param max_length: maximal length of merged lanelets can be provided
        :return: List of merged lanelets, Lists of lanelet ids of which each merged lanelet consists
        """
        assert isinstance(lanelet, Lanelet), '<Lanelet>: provided lanelet is not a valid Lanelet!'
        assert isinstance(network, LaneletNetwork), '<Lanelet>: provided lanelet network is not a ' \
                                                    'valid lanelet network!'
        assert network.find_lanelet_by_id(lanelet.lanelet_id) is not None, '<Lanelet>: lanelet not ' \
                                                                           'contained in network!'

        if lanelet.successor is None or len(lanelet.successor) == 0:
            return [lanelet], [[lanelet.lanelet_id]]

        merge_jobs = lanelet.find_lanelet_successors_in_range(network, max_length=max_length)
        merge_jobs = [[lanelet] + [network.find_lanelet_by_id(p) for p in path] for path in merge_jobs]

        # Create merged lanelets from paths
        merged_lanelets = list()
        merge_jobs_final = []
        for path in merge_jobs:
            pred = path[0]
            merge_jobs_tmp = [pred.lanelet_id]
            for lanelet in path[1:]:
                merge_jobs_tmp.append(lanelet.lanelet_id)
                pred = Lanelet.merge_lanelets(pred, lanelet)

            merge_jobs_final.append(merge_jobs_tmp)
            merged_lanelets.append(pred)

        return merged_lanelets, merge_jobs_final

    def find_lanelet_successors_in_range(self, lanelet_network: "LaneletNetwork", max_length=50.0) -> List[List[int]]:
        """
        Finds all possible successor paths (id sequences) within max_length.

        :param lanelet_network: lanelet network
        :param max_length: abort once length of path is reached
        :return: list of lanelet IDs
        """
        paths = [[s] for s in self.successor]
        paths_final = []
        lengths = [lanelet_network.find_lanelet_by_id(s).distance[-1] for s in self.successor]
        while paths:
            paths_next = []
            lengths_next = []
            for p, le in zip(paths, lengths):
                successors = lanelet_network.find_lanelet_by_id(p[-1]).successor
                if not successors:
                    paths_final.append(p)
                else:
                    for s in successors:
                        if s in p or s == self.lanelet_id or le >= max_length:
                            # prevent loops and consider length of first successor
                            paths_final.append(p)
                            continue

                        l_next = le + lanelet_network.find_lanelet_by_id(s).distance[-1]
                        if l_next < max_length:
                            paths_next.append(p + [s])
                            lengths_next.append(l_next)
                        else:
                            paths_final.append(p + [s])

            paths = paths_next
            lengths = lengths_next

        return paths_final

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

    def add_adjacent_area_to_lanelet(self, area_id: int):
        """
        Adds an area ID to lanelet

        :param area_id: adjacent area ID to add
        """
        self.adjacent_areas.add(area_id)

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

    def orientation_by_position(self, position: np.ndarray) -> float:
        """
        Returns lanelet orientation closest to a given position
        :param position: position of interest
        :return: orientation in interval [-pi,pi]
        """

        def check_angle(point, point1, point2):
            vector_1 = point - point1
            vector_2 = point2 - point1

            def unit_vector(vector):
                norm = np.linalg.norm(vector)
                if np.isclose(norm, 0.):
                    return vector
                else:
                    return vector / norm

            dot_product = np.dot(unit_vector(vector_1), unit_vector(vector_2))
            return np.rad2deg(np.arccos(dot_product))

        assert check_angle(position, self.center_vertices[-1], self.center_vertices[-2]) <= 90 \
               and check_angle(position, self.center_vertices[0], self.center_vertices[1]) <= 90
        position_diff_square = np.sum((self.center_vertices - position) ** 2, axis=1)

        closest_vertex_index = np.argmin(position_diff_square)

        if closest_vertex_index == len(self.center_vertices) - 1:
            vertex1 = self.center_vertices[closest_vertex_index - 1, :]
            vertex2 = self.center_vertices[closest_vertex_index, :]
        else:
            vertex1 = self.center_vertices[closest_vertex_index, :]
            vertex2 = self.center_vertices[closest_vertex_index + 1, :]

        direction_vector = vertex2 - vertex1

        return np.arctan2(direction_vector[1], direction_vector[0])


class MapInformation:
    """
    Class which represents additional information about Lanelet Network
    """
    def __init__(self, commonroad_version: str = "2023a", map_id: str = "map_id", date: Time = None,
                 author: str = "author", affiliation: str = "affiliation", source: str = "source",
                 licence_name: str = "licence_name", licence_text: str = None):
        """
        Constructor for MapInformation

        :param commonroad_version: version of CommonRoad
        :param map_id: the id of the lanelet network
        :param date: date of the lanelet network
        :param author: author of the lanelet network
        :param affiliation: affiliation of the lanelet network
        :param source: source of the lanelet network
        :param licence_name: licence name of the lanelet network
        :param licence_text: licence text of the lanelet network
        """
        self._commonroad_version = commonroad_version
        self._map_id = map_id
        if date is None:
            time = datetime.now()
            self._date = Time(time.hour, time.minute, time.day, time.month, time.year)
        else:
            self._date = date
        self._author = author
        self._affiliation = affiliation
        self._source = source
        self._licence_name = licence_name
        if licence_text is None:
            self._licence_text = ""
        else:
            self._licence_text = licence_text

    def __eq__(self, other):
        if not isinstance(other, MapInformation):
            warnings.warn(f"Inequality between MapInformation {repr(self)} and different type {type(other)}")
            return False

        if self._commonroad_version == other.commonroad_version and self._map_id == other.map_id and \
           self._date == other.date and self._author == other.author and self._affiliation == other.affiliation and \
           self._source == other.source and self._licence_name == other.licence_name and \
           self._licence_text == other.licence_text:
            return True

        warnings.warn(f"Inequality of MapInformation {repr(self)} and the other one {repr(other)}")
        return False

    def __repr__(self):
        return f"MapInformation(commonroad_version={self._commonroad_version}, map_id={self._map_id}," \
               f" date={self._date}, author={self._author}, affiliation={self._affiliation}, source={self._source}," \
               f" licence_name={self._licence_name}, licence_text={self._licence_text}"

    def __str__(self):
        return f"MapInformation with commonroad version {self._commonroad_version}, map_id {self._map_id}," \
               f" date {self._date}, author {self._author}, affiliation {self._affiliation}, source {self._source}," \
               f" licence name {self._licence_name} and licence text {self._licence_text}"

    def __hash__(self):
        return hash((self._commonroad_version, self._map_id, self._date, self._author, self._affiliation,
                     self._source, self._licence_name, self._licence_text))

    @property
    def commonroad_version(self) -> str:
        """ Version of CommonRoad."""
        return self._commonroad_version

    @commonroad_version.setter
    def commonroad_version(self, commonroad_version: str):
        assert isinstance(commonroad_version, str), '<MapInformation/commonroad_version>: ' \
                                            'Provided commonroad_version is not valid! id={}'.format(commonroad_version)
        self._commonroad_version = commonroad_version

    @property
    def map_id(self) -> str:
        """ The id of the lanelet network."""
        return self._map_id

    @map_id.setter
    def map_id(self, map_id: str):
        assert isinstance(map_id, str), '<MapInformation/map_id>: Provided map_id is not valid! id={}'.format(map_id)
        self._map_id = map_id

    @property
    def date(self) -> Time:
        """ Date of the lanelet network."""
        return self._date

    @date.setter
    def date(self, date: Time):
        assert isinstance(date, Time), '<MapInformation/date>: Provided date is not valid! id={}'.format(date)
        self._date = date

    @property
    def author(self) -> str:
        """ Author of the lanelet network."""
        return self._author

    @author.setter
    def author(self, author: str):
        assert isinstance(author, str), '<MapInformation/author>: Provided author is not valid! id={}'.format(author)
        self._author = author

    @property
    def affiliation(self) -> str:
        """ Affiliation of the lanelet network."""
        return self._affiliation

    @affiliation.setter
    def affiliation(self, affiliation: str):
        assert isinstance(affiliation, str), '<MapInformation/affiliation>: ' \
                                             'Provided affiliation is not valid! id={}'.format(affiliation)
        self._affiliation = affiliation

    @property
    def source(self) -> str:
        """ Source of the lanelet network."""
        return self._source

    @source.setter
    def source(self, source: str):
        assert isinstance(source, str), '<MapInformation/source>: ' \
                                             'Provided source is not valid! id={}'.format(source)
        self._source = source

    @property
    def licence_name(self) -> str:
        """ Licence name of the lanelet network."""
        return self._licence_name

    @licence_name.setter
    def licence_name(self, licence_name: str):
        assert isinstance(licence_name, str), '<MapInformation/licence_name>: ' \
                                        'Provided licence_name is not valid! id={}'.format(licence_name)
        self._licence_name = licence_name

    @property
    def licence_text(self) -> Union[None, str]:
        """ Licence text of the lanelet network."""
        return self._licence_text

    @licence_text.setter
    def licence_text(self, licence_text: Union[None, str]):
        self._licence_text = licence_text


class LaneletNetwork(IDrawable):
    """
    Class which represents a network of connected lanelets
    """

    def __init__(self, information: MapInformation = MapInformation()):
        """
        Constructor for LaneletNetwork

        :param information: map information of the lanelet network
        """
        self._information = information
        self._lanelets: Dict[int, Lanelet] = {}
        # lanelet_id, shapely_polygon
        self._buffered_polygons: Dict[int, ShapelyPolygon] = {}
        self._strtee = None
        # id(shapely_polygon), lanelet_id
        self._lanelet_id_index_by_id: Dict[int, int] = {}

        self._intersections: Dict[int, Intersection] = {}
        self._traffic_signs: Dict[int, TrafficSign] = {}
        self._traffic_lights: Dict[int, TrafficLight] = {}
        self._areas: Dict[int, Area] = {}

    # pickling of STRtree is not supported by shapely at the moment
    # use this workaround described in this issue:
    # https://github.com/Toblerity/Shapely/issues/1033
    def __getstate__(self):
        state = self.__dict__.copy()
        del state["_strtee"]
        return state

    def __setstate__(self, state):
        self.__dict__.update(state)
        self._create_strtree()

    def __deepcopy__(self, memo):
        cls = self.__class__
        result = cls.__new__(cls)
        # reset
        self._strtee = None

        memo[id(self)] = result
        for k, v in self.__dict__.items():
            setattr(result, k, copy.deepcopy(v, memo))

        result._create_strtree()
        # restore
        self._create_strtree()

        return result

    def __eq__(self, other):
        if not isinstance(other, LaneletNetwork):
            warnings.warn(f"Inequality between LaneletNetwork {repr(self)} and different type {type(other)}")
            return False

        list_elements_eq = True
        lanelet_network_eq = True
        elements = [self._lanelets, self._intersections, self._traffic_signs, self._traffic_lights,
                    self._areas]
        elements_other = [other._lanelets, other._intersections, other._traffic_signs,
                          other._traffic_lights, other._areas]
        for i in range(0, len(elements)):
            e = elements[i]
            e_other = elements_other[i]
            lanelet_network_eq = lanelet_network_eq and len(e) == len(e_other)
            for k in e.keys():
                if k not in e_other:
                    lanelet_network_eq = False
                    continue
                if e.get(k) != e_other.get(k):
                    list_elements_eq = False
        if self._information != other._information:
            lanelet_network_eq = False

        if not lanelet_network_eq:
            warnings.warn(f"Inequality of LaneletNetwork {repr(self)} and the other one {repr(other)}")

        return lanelet_network_eq and list_elements_eq

    def __hash__(self):
        return hash((self._information, frozenset(self._lanelets.items()), frozenset(self._intersections.items()),
                     frozenset(self._traffic_signs.items()), frozenset(self._traffic_lights.items()),
                     frozenset(self._areas.items())))

    def __str__(self):
        return f"LaneletNetwork consists of lanelets {set(self._lanelets.keys())}, " \
               f"intersections {set(self._intersections.keys())}, " \
               f"traffic signs {set(self._traffic_signs.keys())}," \
               f"traffic lights {set(self._traffic_lights.keys())}" \
               f"and adjacent areas {set(self._areas.keys())}"

    def __repr__(self):
        return f"LaneletNetwork(information={self._information}, lanelets={repr(self._lanelets)}, " \
               f"intersections={repr(self._intersections)}, traffic_signs={repr(self._traffic_signs)}, " \
               f"traffic_lights={repr(self._traffic_lights)}), areas={repr(self._areas)}"

    def _get_lanelet_id_by_shapely_polygon(self, polygon: ShapelyPolygon) -> int:
        return self._lanelet_id_index_by_id[id(polygon)]

    @property
    def information(self) -> MapInformation:
        """ Map information of the lanelet network."""
        return self._information

    @information.setter
    def information(self, information: MapInformation):
        assert isinstance(information, MapInformation), '<LaneletNetwork/information>: provided information is not ' \
                                                        'valid! information = {}'.format(information)
        self._information = information

    @property
    def lanelets(self) -> List[Lanelet]:
        """ List of lanelets of the lanelet network."""
        return list(self._lanelets.values())

    @property
    def lanelet_polygons(self) -> List[Polygon]:
        """ List of polygons of the lanelet network."""
        return [la.polygon for la in self.lanelets]

    @property
    def intersections(self) -> List[Intersection]:
        """ List of intersections of the lanelet network."""
        return list(self._intersections.values())

    @property
    def traffic_signs(self) -> List[TrafficSign]:
        """ List of traffic signs of the lanelet network."""
        return list(self._traffic_signs.values())

    @property
    def traffic_lights(self) -> List[TrafficLight]:
        """ List of traffic lights of the lanelet network."""
        return list(self._traffic_lights.values())

    @property
    def areas(self) -> List[Area]:
        """ List of areas of the lanelet network."""
        return list(self._areas.values())

    @property
    def map_inc_lanelets_to_intersections(self) -> Dict[int, Intersection]:
        """
        dict that maps lanelet ids to the intersection of which it is an incoming lanelet.
        """
        return {l_id: intersection for intersection in self.intersections for l_id in
                list(intersection.map_incoming_lanelets.keys())}

    @classmethod
    def create_from_lanelet_list(cls, lanelets: list, cleanup_ids: bool = False):
        """
        Creates a LaneletNetwork object from a given list of lanelets

        :param lanelets: The list of lanelets
        :param cleanup_ids: cleans up unused ids
        :return: The LaneletNetwork for the given list of lanelets
        """
        assert isinstance(lanelets, list) and all(
                isinstance(la, Lanelet) for la in lanelets), '<LaneletNetwork/create_from_lanelet_list>:' \
                                                             'Provided list of lanelets is not valid! ' \
                                                             'lanelets = {}'.format(lanelets)

        # create lanelet network
        lanelet_network = cls()

        # add each lanelet to the lanelet network
        for la in lanelets:
            lanelet_network.add_lanelet(copy.deepcopy(la), rtree=False)

        if cleanup_ids:
            lanelet_network.cleanup_lanelet_references()

        lanelet_network._create_strtree()

        return lanelet_network

    @classmethod
    def create_from_lanelet_network(cls, lanelet_network: 'LaneletNetwork', shape_input=None,
                                    exclude_lanelet_types=None):
        """
        Creates a lanelet network from a given lanelet network (copy); adding a shape reduces the lanelets to those
        that intersect the shape provided and specifying a lanelet_type set excludes the lanelet types in the new
        created network.

        :param lanelet_network: The existing lanelet network
        :param shape_input: The lanelets intersecting this shape will be in the new network
        :param exclude_lanelet_types: Removes all lanelets with these lanelet_types
        :return: The new lanelet network
        """
        if exclude_lanelet_types is None:
            exclude_lanelet_types = set()
        new_lanelet_network = cls()
        traffic_sign_ids = set()
        traffic_light_ids = set()
        area_ids = set()
        lanelets = set()

        for la in lanelet_network.lanelets:
            if len(la.lanelet_type.intersection(
                    exclude_lanelet_types)) > 0 or shape_input is not None and not \
                    shape_input.shapely_object.intersects(
                    la.polygon.shapely_object):
                continue

            lanelets.add(la)
            for sign_id in la.traffic_signs:
                traffic_sign_ids.add(sign_id)
            for light_id in la.traffic_lights:
                traffic_light_ids.add(light_id)
            for area_id in la.adjacent_areas:
                area_ids.add(area_id)

        for sign_id in traffic_sign_ids:
            new_lanelet_network.add_traffic_sign(copy.deepcopy(lanelet_network.find_traffic_sign_by_id(sign_id)), set())
        for light_id in traffic_light_ids:
            new_lanelet_network.add_traffic_light(copy.deepcopy(lanelet_network.find_traffic_light_by_id(light_id)),
                                                  set())
        for area_id in area_ids:
            new_lanelet_network.add_area(copy.deepcopy(lanelet_network.find_area_by_id(area_id)),
                                         set())
        for la in lanelets:
            new_lanelet_network.add_lanelet(copy.deepcopy(la), rtree=False)
        new_lanelet_network._create_strtree()

        return new_lanelet_network

    def _create_strtree(self):
        """
        Creates spatial index for lanelets for faster querying the lanelets by position.

        Since it is an immutable object, it has to be recreated after every lanelet addition or it should be done
        once after all lanelets are added.
        """

        # validate buffered polygons
        def assert_shapely_polygon(lanelet_id, polygon):
            if not isinstance(polygon, ShapelyPolygon):
                warnings.warn(
                        f"Lanelet with id {lanelet_id}'s polygon is not a <shapely.geometry.Polygon> object! It will "
                        f"be OMITTED from STRtree, therefore this lanelet will NOT be contained in the results of the "
                        f"find_lanelet_by_<position/shape>() functions!!")
                return False
            else:
                return True

        self._buffered_polygons = {lanelet_id: lanelet_shapely_polygon for lanelet_id, lanelet_shapely_polygon in
                                   self._buffered_polygons.items() if
                                   assert_shapely_polygon(lanelet_id, lanelet_shapely_polygon)}

        self._lanelet_id_index_by_id = {id(lanelet_shapely_polygon): lanelet_id for lanelet_id, lanelet_shapely_polygon
                                        in self._buffered_polygons.items()}
        self._strtee = STRtree(list(self._buffered_polygons.values()))

    def remove_lanelet(self, lanelet_id: int, rtree: bool = True):
        """
        Removes a lanelet from a lanelet network and deletes all references.

        :param lanelet_id: ID of lanelet which should be removed.
        :param rtree: Boolean indicating whether rtree should be initialized
        """
        if lanelet_id in self._lanelets.keys():
            del self._lanelets[lanelet_id]
            del self._buffered_polygons[lanelet_id]
            self.cleanup_lanelet_references()

        if rtree:
            self._create_strtree()

    def cleanup_lanelet_references(self):
        """
        Deletes lanelet IDs which do not exist in the lanelet network. Useful when cutting out lanelet networks.
        """
        existing_ids = set(self._lanelets.keys())
        for la in self.lanelets:
            la._predecessor = list(set(la.predecessor).intersection(existing_ids))
            la._successor = list(set(la.successor).intersection(existing_ids))
            la._adj_left = None if la.adj_left is None or la.adj_left not in existing_ids else la.adj_left

            la._adj_left_same_direction = None \
                if la.adj_left_same_direction is None or la.adj_left not in existing_ids else la.adj_left_same_direction
            la._adj_right = None if la.adj_right is None or la.adj_right not in existing_ids else la.adj_right
            la._adj_right_same_direction = None \
                if la.adj_right_same_direction is None or la.adj_right not in existing_ids else \
                la.adj_right_same_direction

        for inter in self.intersections:
            for inc in inter.incomings:
                inc._incoming_lanelets = set(inc.incoming_lanelets).intersection(existing_ids)
                inc._successors_straight = set(inc.successors_straight).intersection(existing_ids)
                inc._successors_right = set(inc.successors_right).intersection(existing_ids)
                inc._successors_left = set(inc.successors_left).intersection(existing_ids)
            inter._crossings = set(inter.crossings).intersection(existing_ids)

    def remove_traffic_sign(self, traffic_sign_id: int):
        """
        Removes a traffic sign from a lanelet network and deletes all references.

        :param traffic_sign_id: ID of traffic sign which should be removed.
        """
        if traffic_sign_id in self._traffic_signs.keys():
            del self._traffic_signs[traffic_sign_id]
            self.cleanup_traffic_sign_references()

    def cleanup_traffic_sign_references(self):
        """
        Deletes traffic sign IDs which do not exist in the lanelet network. Useful when cutting out lanelet networks.
        """
        existing_ids = set(self._traffic_signs.keys())
        for la in self.lanelets:
            la._traffic_signs = la.traffic_signs.intersection(existing_ids)
            if la.stop_line is not None and la.stop_line.traffic_sign_ref is not None:
                la.stop_line._traffic_sign_ref = la.stop_line.traffic_sign_ref.intersection(existing_ids)

    def remove_traffic_light(self, traffic_light_id: int):
        """
        Removes a traffic light from a lanelet network and deletes all references.

        :param traffic_light_id: ID of traffic sign which should be removed.
        """
        if traffic_light_id in self._traffic_lights.keys():
            del self._traffic_lights[traffic_light_id]
        self.cleanup_traffic_light_references()

    def cleanup_traffic_light_references(self):
        """
        Deletes traffic light IDs which do not exist in the lanelet network. Useful when cutting out lanelet networks.
        """
        existing_ids = set(self._traffic_lights.keys())
        for la in self.lanelets:
            la._traffic_lights = la.traffic_lights.intersection(existing_ids)
            if la.stop_line is not None and la.stop_line.traffic_light_ref is not None:
                la.stop_line._traffic_light_ref = la.stop_line.traffic_light_ref.intersection(existing_ids)

    def remove_area(self, area_id: int):
        """
        Removes an area from a lanelet network and deletes all references.

        @param area_id: ID of area which should be removed.
        """
        if area_id in self._areas.keys():
            del self._areas[area_id]

    def remove_intersection(self, intersection_id: int):
        """
        Removes an intersection from a lanelet network and deletes all references.

        :param intersection_id: ID of intersection which should be removed.
        """
        if intersection_id in self._intersections.keys():
            del self._intersections[intersection_id]

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

    def find_area_by_id(self, area_id: int) -> Area:
        """
        Finds an area for a given area id

        :param area_id: The id of the area to find
        :return: The area object if the id exists and None otherwise
        """
        assert is_natural_number(
                area_id), '<LaneletNetwork/find_area_by_id>: provided id is not valid! ' \
                          'id = {}'.format(area_id)

        return self._areas[area_id] if area_id in self._areas else None

    def find_intersection_by_id(self, intersection_id: int) -> Intersection:
        """
        Finds a intersection for a given intersection_id

        :param intersection_id: The id of the intersection to find
        :return: The intersection object if the id exists and None otherwise
        """
        assert is_natural_number(intersection_id), '<LaneletNetwork/find_intersection_by_id>: ' \
                                                   'provided id is not valid! id = {}'.format(intersection_id)

        return self._intersections[intersection_id] if intersection_id in self._intersections else None

    def add_lanelet(self, lanelet: Lanelet, rtree: bool = True):
        """
        Adds a lanelet to the LaneletNetwork

        :param lanelet: The lanelet to add
        :param rtree: Boolean indicating whether rtree should be initialized
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
            self._buffered_polygons[lanelet.lanelet_id] = lanelet.polygon.shapely_object
            if rtree:
                self._create_strtree()
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

    def add_area(self, area: Area, lanelet_ids: Set[int]):
        """
        Adds an area to the LaneletNetwork

        :param area: The area to add
        :param lanelet_ids: Lanelets the area should be referenced from
        :return: True if the area has successfully been added to the network, false otherwise
        """

        assert isinstance(area, Area), '<LaneletNetwork/add_area>: provided area ' \
                                       'is not of type area! ' \
                                       'type = {}'.format(type(area))

        # check if adjacent area already exists in network and warn user
        if area.area_id in self._areas.keys():
            warnings.warn('Area already exists in network! No changes are made.')
            return False
        else:
            self._areas[area.area_id] = area
            for lanelet_id in lanelet_ids:
                lanelet = self.find_lanelet_by_id(lanelet_id)
                if lanelet is not None:
                    lanelet.add_adjacent_area_to_lanelet(area.area_id)
                else:
                    warnings.warn('Area cannot be referenced to lanelet because the lanelet does not exist.')
            return True

    def add_intersection(self, intersection: Intersection):
        """
        Adds an intersection to the LaneletNetwork

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
        for la in lanelet_network.lanelets:
            flag = flag and self.add_lanelet(la, rtree=False)
        self._create_strtree()

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
                          ValidTypes.LISTS), '<Lanelet/contains_points>: provided list of points is not a list! type ' \
                                             '= {}'.format(type(point_list))
        shapely_points = [ShapelyPoint(p) for p in point_list]
        # Accepted tolerance
        tolerance = 1.0e-15
        geometry_ids = self._strtee.query(shapely_points, predicate="dwithin", distance=tolerance)
        # Convert from [[input_index, str_geometry_id], ...] to
        # [[lanlet_id, ...], ...]
        lanelet_ids = defaultdict(list)
        for input_id, geom_id in zip(*geometry_ids):
            lanelet_shapely_polygon = self._strtee.geometries[geom_id]
            lanelet_id = self._get_lanelet_id_by_shapely_polygon(lanelet_shapely_polygon)
            lanelet_ids[input_id].append(lanelet_id)
        res = [lanelet_ids[i] for i, _ in enumerate(point_list)]
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
        res = []
        for idx in self._strtee.query(shape.shapely_object):
            lanelet_shapely_polygon = self._strtee.geometries[idx]
            if lanelet_shapely_polygon.intersects(shape.shapely_object):
                res.append(self._get_lanelet_id_by_shapely_polygon(lanelet_shapely_polygon))
        return res

    def _sorted_lanelet_ids(self, lanelet_ids, state):
        if len(lanelet_ids) < 2:
            return lanelet_ids
        else:
            lanelet_id_list = np.array(lanelet_ids).astype(int)

            def compute_lanelet_relative_orientation(lanelet_id):
                lanelet = self.find_lanelet_by_id(lanelet_id)
                lanelet_orientation = lanelet.orientation_by_position(state.position)
                return np.abs(subtract_orientations(lanelet_orientation, state.orientation))

            orientation_differences = np.array(list(map(compute_lanelet_relative_orientation, lanelet_id_list)))
            sorted_indices = np.argsort(orientation_differences)

            return list(lanelet_id_list[sorted_indices])

    def find_most_likely_lanelet_by_state(self, state_list: List[TraceState]) -> List[int]:
        """
        Finds the lanelet id of the position of a given state; in case of multiple overlapping lanelets, return the most
        likely lanelet according to the orientation difference between lanelets and given state

        :param state_list: The list of states to check
        :return: A list of lanelet ids. If the position could not be matched to a lanelet, an empty list is returned
        """
        assert isinstance(state_list, ValidTypes.LISTS), '<Lanelet/find_most_likely_lanelet_by_state>: ' \
                                                         'provided list of points is not a list! type = {}'.format(
                type(state_list))
        assert np.all([hasattr(state, "orientation") for state in
                       state_list]), '<Lanelet/find_most_likely_lanelet_by_state>: provided state must have ' \
                                     'orientation!'

        return [self._sorted_lanelet_ids(self.find_lanelet_by_position([state.position])[0], state)[0] for state in
                state_list]

    def filter_obstacles_in_network(self, obstacles: List[Obstacle]) -> List[Obstacle]:
        """
        Returns the list of obstacles which are located in the lanelet network

        :param obstacles: The list of obstacles to check
        :return: The list of obstacles which are located in the lanelet network
        """

        res = list()

        obstacle_to_lanelet_map = self.map_obstacles_to_lanelets(obstacles)

        for k in obstacle_to_lanelet_map.keys():
            obs = obstacle_to_lanelet_map[k]
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

        for la in self.lanelets:
            # map obstacles to current lanelet
            mapped_objs = la.get_obstacles(obstacles)

            # check if mapping is not empty
            if len(mapped_objs) > 0:
                mapping[la.lanelet_id] = mapped_objs

        return mapping

    def lanelets_in_proximity(self, point: np.ndarray, radius: float) -> List[Lanelet]:
        """
        Finds all lanelets which intersect a given circle, defined by the center point and radius

        :param point: The center of the circle
        :param radius: The radius of the circle
        :return: The list of lanelets which intersect the given circle
        """

        assert is_real_number_vector(point, length=2), '<LaneletNetwork/lanelets_in_proximity>: provided point is ' \
                                                       'not valid! point = {}'.format(point)
        assert is_positive(
                radius), '<LaneletNetwork/lanelets_in_proximity>: provided radius is not valid! radius = {}'.format(
                radius)

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
                index_min_dist = np.argmin(distance - rad_sqr)

                # check right side of lanelet
                if lanelet.adj_right is not None:
                    p = (lanelet.right_vertices[index_min_dist, :] - point) ** 2
                    p = p[0] + p[1]
                    if np.greater(rad_sqr, p) and lanelet.adj_right not in lanes:
                        lanes[lanelet.adj_right] = self.find_lanelet_by_id(lanelet.adj_right)
                        distance_list.append(p)

                # check left side of lanelet
                if lanelet.adj_left is not None:
                    p = (lanelet.left_vertices[index_min_dist, :] - point) ** 2
                    p = p[0] + p[1]
                    if np.greater(rad_sqr, p) and lanelet.adj_left not in lanes:
                        lanes[lanelet.adj_left] = self.find_lanelet_by_id(lanelet.adj_left)
                        distance_list.append(p)

        # sort list according to distance
        indices = np.argsort(distance_list)
        lanelets = list(lanes.values())

        # return sorted list
        return [lanelets[i] for i in indices]

    def draw(self, renderer: IRenderer, draw_params: OptionalSpecificOrAllDrawParams[LaneletNetworkParams] = None):
        renderer.draw_lanelet_network(self, draw_params)
