import copy
import enum
from typing import *
import numpy as np
import networkx as nx

import commonroad.geometry.transform
from commonroad.common.validity import *
from commonroad.geometry.shape import Polygon, ShapeGroup, Circle
from commonroad.scenario.obstacle import Obstacle

__author__ = "Christian Pek"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["BMW CAR@TUM"]
__version__ = "2019.1"
__maintainer__ = "Christian Pek"
__email__ = "Christian.Pek@tum.de"
__status__ = "released"


class LineMarking(enum.Enum):
    """
    Enum describing different types of line markings, i.e. dashed or solid lines
    """
    DASHED = 1
    SOLID = 2


class Lanelet:
    """
    Class which describes a Lanelet entity according to the CommonRoad specification. Each lanelet is described by a
    left and right boundary (polylines). Furthermore, lanelets have relations to other lanelets, e.g. an adjacent left
    neighbor or a predecessor.
    """

    def __init__(self, left_vertices: np.ndarray, center_vertices: np.ndarray, right_vertices: np.ndarray,
                 lanelet_id,
                 predecessor=None, successor=None,
                 adjacent_left=None, adjacent_left_same_direction=None,
                 adjacent_right=None, adjacent_right_same_direction=None,
                 speed_limit=np.infty, line_marking_left_vertices=None,
                 line_marking_right_vertices=None):
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
        :param speed_limit: The speed limit of the lanelet
        :param line_marking_left_vertices: The type of line marking of the left boundary
        :param line_marking_right_vertices: The type of line marking of the right boundary
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
        self._line_marking_left_vertices = None
        if line_marking_left_vertices is not None:
            self.line_marking_left_vertices = line_marking_left_vertices
        self._line_marking_right_vertices = None
        if line_marking_right_vertices is not None:
            self.line_marking_right_vertices = line_marking_right_vertices

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

        # Set speed limit
        self._speed_limit = None
        self.speed_limit = speed_limit

        self._distance = [0.0]
        for i in range(1, len(self.center_vertices)):
            self._distance.append(self._distance[i - 1] +
                                  np.linalg.norm(np.array(self.center_vertices[i]) -
                                                 np.array(self.center_vertices[i - 1])))
        self._distance = np.array(self._distance)

        # create empty polygon
        self._polygon = None

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
    def speed_limit(self) -> float:
        return self._speed_limit

    @speed_limit.setter
    def speed_limit(self, limit: float):
        if self._speed_limit is None:
            assert is_valid_velocity(limit, 0.,
                                     None), '<Lanelet/speed_limit>: provided speed_limit is not valid! ' \
                                            'limit = {}'.format(
                limit)
            self._speed_limit = limit
        else:
            warnings.warn('<Lanelet/speed_limit> speed_limit of lanelet is immutable')

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
            self._line_marking_left_vertices = line_marking_left_vertices
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
            self._line_marking_right_vertices = line_marking_right_vertices
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
                                                                                        'of predecessors is not valid!' \
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

    def interpolate_position(self, distance: float) -> tuple:
        """
        Computes the interpolated positions on the center/right/left polyline of the lanelet for a given distance
        along the lanelet

        :param distance: The distance for the interpolation
        :return: The interpolated positions on the center/right/left polyline in the form [[x_c,y_c],[x_r,y_r],[x_l,y_l]]
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
                          ValidTypes.ARRAY), '<Lanelet/contains_points>: provided list of points is not a list! type = {}'.format(
            type(point_list))
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
            else:
                # distinguish between type of shape (circle has no vertices)
                if isinstance(o_shape, Circle):
                    vertices = o_shape.center
                else:
                    vertices = o_shape.vertices

            # check if obstacle is in lane
            if any(self.contains_points(np.array(vertices))):
                res.append(o)

        return res

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
        assert lanelet1.lanelet_id in lanelet2.successor or lanelet2.lanelet_id in lanelet1.successor,\
            '<Lanelet/merge_lanelets>: cannot merge two not connected lanelets! successors of l1 = {}, successors of l2 = {}'.format(
            lanelet1.successor, lanelet2.successor)

        # check pred and successor
        if lanelet1.lanelet_id in lanelet2.successor:
            pred = lanelet2
            suc = lanelet1
        else:
            pred = lanelet1
            suc = lanelet2

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

        return Lanelet(left_vertices, center_vertices, right_vertices, lanelet_id, predecessor=predecessor,
                       successor=successor)

    @classmethod
    def all_lanelets_by_merging_successors_from_lanelet(cls, lanelet: 'Lanelet', network: 'LaneletNetwork',
                                                        max_length: float = 150.0) \
            -> Tuple[List['Lanelet'], List[List[int]]]:
        """
        Computes all reachable lanelets starting from a provided lanelet and merges them to a single lanelet for each route.

        :param lanelet: The lanelet to start from
        :param network: The network which contains all lanelets
        :param max_length: maxmimal length of merged lanelets can be provided
        :return: List of merged lanelets, Lists of lanelet ids of which each merged lanelet consists
        """
        assert isinstance(lanelet, Lanelet), '<Lanelet>: provided lanelet is not a valid Lanelet!'
        assert isinstance(network, LaneletNetwork), '<Lanelet>: provided lanelet network is not a valid lanelet network!'
        assert network.find_lanelet_by_id(lanelet.lanelet_id) is not None, '<Lanelet>: lanelet not contained in network!'

        if lanelet.successor is None or len(lanelet.successor) == 0:
            return [], []

        # Create Graph from network
        Net = nx.DiGraph() 
        lanelets = network._lanelets.values()
        leafs = list()
        for elements in lanelets:
            Net.add_node(elements)
            if elements.successor and not lanelet.lanelet_id in elements.successor:
                for successors in elements.successor:
                    successor = network.find_lanelet_by_id(successors)
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


    def __str__(self):
        return 'Lanelet with id:' + str(self.lanelet_id)


class LaneletNetwork:
    """
    Class which represents a network of connected lanelets
    """

    def __init__(self):
        self._lanelets: Dict = {}

    @property
    def lanelets(self) -> List[Lanelet]:
        return list(self._lanelets.values())

    @lanelets.setter
    def lanelets(self, lanelets: list):
        warnings.warn('<LaneletNetwork/lanelets>: lanelets of network are immutable')

    @classmethod
    def create_from_lanelet_list(cls, lanelets: list):
        """
        Creates a LaneletNetwork object from a given list of lanelets

        :param lanelets: The list of lanelets
        :return: The LaneletNetwork for the given list of lanelets
        """
        assert isinstance(lanelets, list) and all(isinstance(l, Lanelet) for l in
                                                  lanelets), '<LaneletNetwork/create_from_lanelet_list>:' \
                                                             'Provided list of lanelets is not valid! lanelets = {}'.format(
            lanelets)

        # create lanelet network
        lanelet_network = cls()

        # add each lanelet to the lanelet network
        for l in lanelets:
            lanelet_network.add_lanelet(copy.deepcopy(l))

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

    def find_lanelet_by_id(self, lanelet_id: int) -> Lanelet:
        """
        Finds a lanelet for a given lanelet_id

        :param lanelet_id: The id of the lanelet to find
        :return: The lanelet object if the id exists and None otherwise
        """
        assert is_natural_number(
            lanelet_id), '<LaneletNetwork/find_lanelet_by_id>: provided id is not valid! id = {}'.format(lanelet_id)

        return self._lanelets[lanelet_id]

    def add_lanelet(self, lanelet: Lanelet):
        """
        Adds a lanelet to the LaneletNetwork

        :param lanelet: The lanelet to add
        :return: True if the lanelet has successfully been added to the network, false otherwise
        """

        assert isinstance(lanelet,
                          Lanelet), '<LaneletNetwork/add_lanelet>: provided lanelet is not of type lanelet! type = {}'.format(
            type(lanelet))

        # flag if added to lanelet network
        flag = False

        # check if lanelet already exists in network and warn user
        if lanelet.lanelet_id in self._lanelets.keys():
            warnings.warn('Lanelet already exists in network! No changes are made.')
            return False
        else:
            self._lanelets[lanelet.lanelet_id] = lanelet
            flag = True

        return flag

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
                                     2), '<LaneletNetwork/translate_rotate>: provided translation is not valid! translation = {}'.format(
            translation)
        assert is_valid_orientation(
            angle), '<LaneletNetwork/translate_rotate>: provided angle is not valid! angle = {}'.format(angle)

        # rotate each lanelet
        for lanelet in self._lanelets.values():
            lanelet.translate_rotate(translation, angle)

    def find_lanelet_by_position(self, point_list: List[np.ndarray]) -> List[List[int]]:
        """
        Finds the lanelet id of a given position

        :param point_list: The list of positions to check
        :return: A list of lanelet ids. If the position could not be matched to a lanelet, an empty list is returned
        """
        assert isinstance(point_list,
                          ValidTypes.LISTS), '<Lanelet/contains_points>: provided list of points is not a list! type = {}'.format(
            type(point_list))
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
