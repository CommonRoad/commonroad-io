__author__ = "Behtarin Ferdousi"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = []
__version__ = "2019.1"
__maintainer__ = "Behtarin Ferdousi"
__email__ = "commonroad@in.tum.de"
__status__ = "Development"

from typing import List, Set

from commonroad.scenario.lanelet import Lanelet


class IntersectionIncomingElement:
    """ Class for incoming element in an intersection"""
    def __init(self, incoming_id: int,
               incoming_lanelets: Set[int] = None,
               successors_right: Set[int]= None,
               successors_straight: Set[int] = None,
               successors_left: Set[int] = None,
               is_left_of: Set[int] = None
               ):
        self.incoming_id = incoming_id
        self.incoming_lanelets = incoming_lanelets
        self.successors_right = successors_right
        self.successors_straight = successors_straight
        self.successors_left = successors_left
        self.is_left_of = is_left_of


class IntersectionCrossingElement:
    """Class to represent the crossings in an intersection"""
    def __init__(self,
                 crossing_lanelet: Lanelet):
        self.crossing_lanelet = crossing_lanelet


class Intersection:
    """ Class to represent intersection"""
    def __init__(self, id: int,
                 incomings: List[IntersectionIncomingElement],
                 crossings: List[IntersectionCrossingElement] = None):
        self.id = id
        self.incomings = incomings
        self.crossings = crossings

