from typing import List, Set, Dict
from commonroad.common.validity import *


class IncomingGroup:
    """
    This class represents an incoming element of an intersection.
    An incoming can consist of several adjacent lanelets which lead to an intersection, right, straight,
    and left outgoing lanelets, and a reference to the crossing lanelet.
    The right/straight/left outgoings are used to infer for which lanelets the traffic light
    is valid for and to facilitate the calculation of priorities at intersections.
    The crossing element models lanelets which cross other lanelets, e.g., these are usually lanelets of type crosswalk.
    """

    def __init__(self, incoming_id: int, incoming_lanelets: Set[int] = None, outgoing_id: int = None,
                 outgoing_right: Set[int] = None, outgoing_straight: Set[int] = None, outgoing_left: Set[int] = None,
                 crossings: Set[int] = None):
        """
        :param incoming_id: incoming element ID
        :param incoming_lanelets: set of IDs of incoming lanelets
        :param outgoing_id: ID of the outgoing element that contains outgoing lanelets for this intersection element
        :param outgoing_right: set of IDs of incoming lanelets which turn right
        :param outgoing_straight: set of IDs of incoming lanelets which go straight
        :param outgoing_left: set of IDs of incoming lanelets which turn left
        :param crossings: set of crossing elements in intersection
        """
        self._incoming_id = None
        self._incoming_lanelets = None
        self.incoming_id = incoming_id
        self.incoming_lanelets = incoming_lanelets
        self.outgoing_id = outgoing_id
        self.outgoing_right = outgoing_right
        self.outgoing_straight = outgoing_straight
        self.outgoing_left = outgoing_left
        self.crossings = crossings

    def __eq__(self, other):
        if not isinstance(other, IncomingGroup):
            warnings.warn(f"Inequality between IntersectionIncomingElement "
                          f"{repr(self)} and different type {type(other)}")
            return False

        if self._incoming_id == other.incoming_id and self._incoming_lanelets == other.incoming_lanelets \
                and self._outgoing_id == other.outgoing_id \
                and self._outgoing_right == other.outgoing_right \
                and self._outgoing_straight == other.outgoing_straight \
                and self._outgoing_left == other.outgoing_left\
                and self._crossings == other.crossings:
            return True

        warnings.warn(f"Inequality of IntersectionIncomingElement {repr(self)} and the other one {repr(other)}")
        return False

    def __hash__(self):
        return hash((self._incoming_id, frozenset(self._incoming_lanelets), self._outgoing_id,
                     frozenset(self._outgoing_right), frozenset(self._outgoing_straight),
                     frozenset(self._outgoing_left), frozenset(self._crossings)))

    def __str__(self):
        return f"IncomingGroupElement with id {self._incoming_id} represents the incoming" \
               f" lanelets {self._incoming_lanelets} and has outgoing lanelet {self._outgoing_id}," \
               f" right outgoings {self._outgoing_right}, straight outgoings {self._outgoing_straight}," \
               f" left outgoings {self._outgoing_left} and crossings {self._crossings}"

    def __repr__(self):
        return f"IncomingGroupElement(incoming_id={self._incoming_id}, " \
               f"incoming_lanelets={self._incoming_lanelets}, outgoing_id={self._outgoing_id}," \
               f"outgoing_right={self._outgoing_right}, outgoing_straight={self._outgoing_straight}, " \
               f"outgoing_left={self._outgoing_left}), crossings={self._crossings}"

    @property
    def incoming_id(self) -> int:
        """
        returns ID of incoming
        """
        return self._incoming_id

    @incoming_id.setter
    def incoming_id(self, i_id: int):
        """
        :param i_id: ID of incoming
        """
        assert is_natural_number(i_id), '<IntersectionIncomingElement/incoming_id>: Provided incoming_id is not ' \
                                        'valid! id={}'.format(i_id)
        self._incoming_id = i_id

    @property
    def incoming_lanelets(self) -> Set[int]:
        """
        returns set of IDs of incoming lanelets
        """
        return self._incoming_lanelets

    @incoming_lanelets.setter
    def incoming_lanelets(self, incoming_lanelets: Set[int]):
        """
        :param incoming_lanelets: set of IDs of incoming lanelets
        """
        self._incoming_lanelets = incoming_lanelets

    @property
    def outgoing_id(self) -> Union[None, int]:
        """
        returns an ID of the outgoing element that contains outgoing lanelets for this intersection element
        """
        return self._outgoing_id

    @outgoing_id.setter
    def outgoing_id(self, outgoing_id: Union[None, int]):
        """
        :param outgoing_id: id of the outgoing lanelet
        """
        self._outgoing_id = outgoing_id

    @property
    def outgoing_right(self) -> Set[int]:
        """
        returns set of IDs of incoming lanelets which turn right
        """
        return self._outgoing_right

    @outgoing_right.setter
    def outgoing_right(self, outgoing_right: Set[int]):
        """
        :param outgoing_right: set of IDs of incoming lanelets which turn right
        """
        if outgoing_right is None:
            self._outgoing_right = set()
        else:
            self._outgoing_right = outgoing_right

    @property
    def outgoing_straight(self) -> Set[int]:
        """
        returns set of IDs of incoming lanelets which go straight
        """
        return self._outgoing_straight

    @outgoing_straight.setter
    def outgoing_straight(self, outgoing_straight: Set[int]):
        """
        :param outgoing_straight: set of IDs of incoming lanelets which go straight
        """
        if outgoing_straight is None:
            self._outgoing_straight = set()
        else:
            self._outgoing_straight = outgoing_straight

    @property
    def outgoing_left(self) -> Set[int]:
        """
        returns set of IDs of incoming lanelets which turn left
        """
        return self._outgoing_left

    @outgoing_left.setter
    def outgoing_left(self, outgoing_left: Set[int]):
        """
        :param outgoing_left: set of IDs of incoming lanelets which turn left
        """
        if outgoing_left is None:
            self._outgoing_left = set()
        else:
            self._outgoing_left = outgoing_left

    @property
    def crossings(self) -> Set[int]:
        """
        returns set of crossing elements in intersection
        """
        return self._crossings

    @crossings.setter
    def crossings(self, crossings: Set[int]):
        """
        :param crossings: set of crossing elements in intersection
        """
        if crossings is None:
            self._crossings = set()
        else:
            self._crossings = crossings


class OutgoingGroup:
    """
    This class represents an outgoing element of an intersection.
    It contains a group of lanelets that are going out of a lanelet in an intersection.
    """

    def __init__(self, outgoing_id: int, outgoing_lanelets: Set[int] = None):
        """
        :param outgoing_id: id of the outgoing group
        :param outgoing_lanelets: set of outgoing lanelets
        """
        self._outgoing_id = outgoing_id
        if outgoing_lanelets is None:
            self._outgoing_lanelets = set()
        else:
            self._outgoing_lanelets = outgoing_lanelets

    def __eq__(self, other):
        if not isinstance(other, OutgoingGroup):
            warnings.warn(f"Inequality between OutgoingGroup element "
                          f"{repr(self)} and different type {type(other)}")
            return False

        if self._outgoing_id == other.outgoing_id and self._outgoing_lanelets == other.outgoing_lanelets:
            return True

        warnings.warn(f"Inequality of OutgoingGroupElement {repr(self)} and the other one {repr(other)}")
        return False

    def __hash__(self):
        return hash((self.outgoing_id, frozenset(self._outgoing_lanelets)))

    def __str__(self):
        return f"OutgoingGroupElement with id {self._outgoing_id} represents the outgoing" \
               f" lanelets {self._outgoing_lanelets}"

    @property
    def outgoing_id(self) -> int:
        return self._outgoing_id

    @outgoing_id.setter
    def outgoing_id(self, outgoing_id: int):
        assert is_natural_number(outgoing_id), '<OutgoingGroup/outgoing_id>: Provided outgoing_id is not ' \
                                        'valid! id={}'.format(outgoing_id)
        self._outgoing_id = outgoing_id

    @property
    def outgoing_lanelets(self) -> Union[None, Set[int]]:
        return self._outgoing_lanelets

    @outgoing_lanelets.setter
    def outgoing_lanelets(self, outgoing_lanelets: Union[None, Set[int]]):
        if outgoing_lanelets is None:
            self._outgoing_lanelets = set()
        else:
            self._outgoing_lanelets = outgoing_lanelets


class Intersection:
    """
    This class represent an intersection.
    An intersection element is defined by at least one incoming and an optional outgoing group element.
    """

    def __init__(self, intersection_id: int, incomings: List[IncomingGroup],
                 outgoings: List[OutgoingGroup] = None):
        """
        :param intersection_id: ID of intersection element
        :param incomings: set of incoming elements in intersection
        :param outgoings: set of outgoing elements in intersection
        """
        self._intersection_id = None
        self._incomings = None

        self.intersection_id = intersection_id
        self.incomings = incomings

        if outgoings is None:
            self._outgoings = []
        else:
            self.outgoings = outgoings

    def __eq__(self, other):
        if not isinstance(other, Intersection):
            warnings.warn(f"Inequality between Intersection {repr(self)} and different type {type(other)}")
            return False

        list_elements_eq = True
        incomings = {incoming.incoming_id: incoming for incoming in self._incomings}
        incomings_other = {incoming.incoming_id: incoming for incoming in other.incomings}
        outgoings = {outgoing.outgoing_id: outgoing for outgoing in self._outgoings}
        outgoings_other = {outgoing.outgoing_id: outgoing for outgoing in other.outgoings}
        intersection_eq = len(incomings) == len(incomings_other) and len(outgoings) == len(outgoings_other)
        for k in incomings.keys():
            if k not in incomings_other:
                intersection_eq = False
                continue
            if incomings.get(k) != incomings_other.get(k):
                list_elements_eq = False

        for k in outgoings.keys():
            if k not in outgoings_other:
                intersection_eq = False
                continue
            if outgoings.get(k) != outgoings_other.get(k):
                list_elements_eq = False

        if intersection_eq and self._intersection_id == other.intersection_id:
            return list_elements_eq

        warnings.warn(f"Inequality of Intersection {repr(self)} and the other one {repr(other)}")
        return False

    def __hash__(self):
        return hash((self._intersection_id, frozenset(self._incomings), frozenset(self._outgoings)))

    def __str__(self):
        return f"Intersection with id {self._intersection_id} consisting of {len(self._incomings)} incoming elements " \
               f"and outgoing elements {self._outgoings}"

    def __repr__(self):
        return f"Intersection(intersection_id={self._intersection_id}, incomings={repr(self._incomings)}, " \
               f"outgoings={self._outgoings})"

    @property
    def intersection_id(self) -> int:
        """
        returns ID of intersection element
        """
        return self._intersection_id

    @intersection_id.setter
    def intersection_id(self, i_id: int):
        """
        :param i_id: ID of intersection element
        """
        assert is_natural_number(i_id), '<Intersection/intersection_id>: Provided intersection_id is not ' \
                                        'valid! id={}'.format(i_id)
        self._intersection_id = i_id

    @property
    def incomings(self) -> List[IncomingGroup]:
        """
        returns set of incoming elements in intersection
        """
        return self._incomings

    @incomings.setter
    def incomings(self, incomings: List[IncomingGroup]):
        """
        :param incomings: i_id ID of intersection element
        """
        self._incomings = incomings

    @property
    def outgoings(self) -> List[OutgoingGroup]:
        """
        returns set of outgoing elements in intersection
        """
        return self._outgoings

    @outgoings.setter
    def outgoings(self, outgoings: List[OutgoingGroup]):
        self._outgoings = outgoings

    @property
    def map_incoming_lanelets(self) -> Dict[int, IncomingGroup]:
        """
        Maps all incoming lanelet ids to IntersectionIncomingElement
        :returns dictionary mapping lanelet IDs to incomings
        """
        return {l_id: inc for inc in self.incomings for l_id in inc.incoming_lanelets}
