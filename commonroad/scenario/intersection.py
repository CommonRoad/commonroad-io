__author__ = "Sebastian Maierhofer"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["CAR@TUM"]
__version__ = "2020.2"
__maintainer__ = "Sebastian Maierhofer"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "Release"

from typing import List, Set, Union, Dict


class IntersectionIncomingElement:
    """ Class which represents an incoming element of an intersection"""
    def __init__(self, incoming_id: int, incoming_lanelets: Set[int] = None, successors_right: Set[int] = None,
                 successors_straight: Set[int] = None, successors_left: Set[int] = None,
                 left_of: int = None):
        """
        :param incoming_id: incoming element ID
        :param incoming_lanelets: set of IDs of incoming lanelets
        :param successors_right: set of IDs of incoming lanelets which turn right
        :param successors_straight: set of IDs of incoming lanelets which go straight
        :param successors_left: set of IDs of incoming lanelets which turn left
        :param left_of: incoming element ID left of this incoming element
        """
        self._incoming_id = incoming_id
        self._incoming_lanelets = incoming_lanelets
        self._successors_right = successors_right
        self._successors_straight = successors_straight
        self._successors_left = successors_left
        self._left_of = left_of

    @property
    def incoming_id(self) -> int:
        return self._incoming_id

    @property
    def incoming_lanelets(self) -> Set[int]:
        return self._incoming_lanelets

    @property
    def successors_right(self) -> Set[int]:
        return self._successors_right

    @property
    def successors_straight(self) -> Set[int]:
        return self._successors_straight

    @property
    def successors_left(self) -> Set[int]:
        return self._successors_left

    @property
    def left_of(self) -> int:
        return self._left_of


class Intersection:
    """ Class to represent an intersection"""
    def __init__(self, intersection_id: int, incomings: List[IntersectionIncomingElement], crossings: Set[int] = None):
        """
        :param intersection_id: ID of intersection element
        :param incomings: set of incoming elements in intersection
        :param crossings: set of crossing elements in intersection (currently not supported)
        """
        self._intersection_id = intersection_id
        self._incomings = incomings
        if crossings is None:
            self._crossings = set()
        else:
            self._crossings = crossings

    @property
    def intersection_id(self) -> int:
        return self._intersection_id

    @property
    def incomings(self) -> List[IntersectionIncomingElement]:
        return self._incomings

    @property
    def crossings(self) -> Set[int]:
        return self._crossings

    @property
    def map_incoming_lanelets(self) -> Dict[int,IntersectionIncomingElement]:
        """Maps all incoming lanelet ids to IntersectionIncomingElement"""
        return {l_id: inc for inc in self.incomings for l_id in inc.incoming_lanelets}