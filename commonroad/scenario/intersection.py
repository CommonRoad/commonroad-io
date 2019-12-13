__author__ = "Behtarin Ferdousi"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = []
__version__ = "2019.1"
__maintainer__ = "Behtarin Ferdousi"
__email__ = "commonroad@in.tum.de"
__status__ = "Development"

from typing import List

from commonroad.scenario.lanelet import Lanelet


class IntersectionIncomingElement:
    """ Class for incoming element in an intersection"""
    def __init(self, incoming_id: int,
               incoming_lanelets: List[Lanelet],
               successors_right: List[Lanelet] = None,
               successors_straight: List[Lanelet] = None,
               successors_left: List[Lanelet] = None,
               is_left_of=None
               ):
        if is_left_of is None:
            is_left_of = []
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
    def __init__(self, intersection_id: int,
                 incomings: List[IntersectionIncomingElement],
                 crossings: List[IntersectionCrossingElement] = None):
        self.incomings = incomings
        self.crossings = crossings

