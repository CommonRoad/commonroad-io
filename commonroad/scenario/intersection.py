__author__ = "Behtarin Ferdousi"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = []
__version__ = "2020.1"
__maintainer__ = "Behtarin Ferdousi"
__email__ = "commonroad@in.tum.de"
__status__ = "Development"

from typing import List, Set


class IntersectionIncomingElement:
    """ Class for incoming element in an intersection"""
    def __init__(self, incoming_id: int, incoming_lanelets: Set[int] = None, successors_right: Set[int] = None,
                 successors_straight: Set[int] = None, successors_left: Set[int] = None,
                 incomings_left: Set[int] = None):
        """
        :param incoming_id: incoming element ID
        :param incoming_lanelets: set of IDs of incoming lanelets
        :param successors_right: set of IDs of incoming lanelets which turn right
        :param successors_straight: set of IDs of incoming lanelets which go straight
        :param successors_left: set of IDs of incoming lanelets which turn left
        :param incomings_left: set of incoming elements on the left side
        """
        self._incoming_id = incoming_id
        self._incoming_lanelets = incoming_lanelets
        self._successors_right = successors_right
        self._successors_straight = successors_straight
        self._successors_left = successors_left
        self._incomings_left = incomings_left

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
    def incomings_left(self) -> Set[int]:
        return self._incomings_left


class Intersection:
    """ Class to represent intersection"""
    def __init__(self, intersection_id: int, incomings: List[IntersectionIncomingElement], crossings: Set[int] = None):
        """
        :param intersection_id: ID of intersection element
        :param incomings: set of incoming elements in intersection
        :param crossings: set of crossing elements in intersection
        """
        self._intersection_id = intersection_id
        self._incomings = incomings
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
