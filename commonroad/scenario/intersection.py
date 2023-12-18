from __future__ import annotations

import warnings
from typing import TYPE_CHECKING, Dict, List, Optional, Set, Union

from commonroad.common.validity import is_natural_number

if TYPE_CHECKING:
    from commonroad.scenario.lanelet import LaneletNetwork


class IncomingGroup:
    """
    This class represents an incoming element of an intersection.
    An incoming can consist of several adjacent lanelets which lead to an intersection, right, straight,
    and left outgoing lanelets, and a reference to the crossing lanelet.
    The right/straight/left outgoings are used to infer for which lanelets the traffic light
    is valid for and to facilitate the calculation of priorities at intersections.
    The crossing element models lanelets which cross other lanelets, e.g., these are usually lanelets of type crosswalk.
    """

    def __init__(
        self,
        incoming_id: int,
        incoming_lanelets: Optional[Set[int]] = None,
        outgoing_group_id: Optional[int] = None,
        outgoing_right: Optional[Set[int]] = None,
        outgoing_straight: Optional[Set[int]] = None,
        outgoing_left: Optional[Set[int]] = None,
    ):
        """
        :param incoming_id: incoming element ID
        :param incoming_lanelets: set of IDs of incoming lanelets
        :param outgoing_group_id: ID of the outgoing group that contains outgoing lanelets for this incoming element
        :param outgoing_right: set of IDs of incoming lanelets which turn right
        :param outgoing_straight: set of IDs of incoming lanelets which go straight
        :param outgoing_left: set of IDs of incoming lanelets which turn left
        """
        self._incoming_id = None
        self._incoming_lanelets = None
        self.incoming_id = incoming_id
        self.incoming_lanelets = incoming_lanelets
        self.outgoing_group_id = outgoing_group_id
        self.outgoing_right = outgoing_right
        self.outgoing_straight = outgoing_straight
        self.outgoing_left = outgoing_left

    def __eq__(self, other):
        if not isinstance(other, IncomingGroup):
            warnings.warn(f"Inequality between IncomingGroup element {repr(self)} and different type {type(other)}")
            return False

        return (
            self._incoming_id == other.incoming_id
            and self._incoming_lanelets == other.incoming_lanelets
            and self._outgoing_group_id == other.outgoing_group_id
            and self._outgoing_right == other.outgoing_right
            and self._outgoing_straight == other.outgoing_straight
            and self._outgoing_left == other.outgoing_left
        )

    def __hash__(self):
        return hash(
            (
                self._incoming_id,
                frozenset(self._incoming_lanelets),
                self._outgoing_group_id,
                frozenset(self._outgoing_right),
                frozenset(self._outgoing_straight),
                frozenset(self._outgoing_left),
            )
        )

    def __str__(self):
        return (
            f"IncomingGroup with id {self._incoming_id} represents the incoming"
            f" lanelets {self._incoming_lanelets} and has outgoing lanelet {self._outgoing_group_id},"
            f" right outgoings {self._outgoing_right}, straight outgoings {self._outgoing_straight},"
            f" left outgoings {self._outgoing_left}"
        )

    def __repr__(self):
        return (
            f"IncomingGroup(incoming_id={self._incoming_id}, "
            f"incoming_lanelets={self._incoming_lanelets}, outgoing_id={self._outgoing_group_id},"
            f"outgoing_right={self._outgoing_right}, outgoing_straight={self._outgoing_straight}, "
            f"outgoing_left={self._outgoing_left})"
        )

    @property
    def incoming_id(self) -> int:
        """
        ID of incoming
        """
        return self._incoming_id

    @incoming_id.setter
    def incoming_id(self, i_id: int):
        """
        :param i_id: ID of incoming
        """
        assert is_natural_number(
            i_id
        ), "<IncomingGroup/incoming_id>: Provided incoming_id is not " "valid! id={}".format(i_id)
        self._incoming_id = i_id

    @property
    def incoming_lanelets(self) -> Set[int]:
        """
        set of IDs of incoming lanelets
        """
        return self._incoming_lanelets

    @incoming_lanelets.setter
    def incoming_lanelets(self, incoming_lanelets: Set[int]):
        """
        :param incoming_lanelets: set of IDs of incoming lanelets
        """
        self._incoming_lanelets = incoming_lanelets

    @property
    def outgoing_group_id(self) -> Optional[int]:
        """
        ID of the outgoing group element that contains outgoing lanelets for this incoming element
        """
        return self._outgoing_group_id

    @outgoing_group_id.setter
    def outgoing_group_id(self, outgoing_group_id: Optional[int]):
        """
        :param outgoing_group_id: id of the corresponding outgoing group of the incoming group
        """
        self._outgoing_group_id = outgoing_group_id

    @property
    def outgoing_right(self) -> Set[int]:
        """
        set of IDs of incoming lanelets which turn right
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
        set of IDs of incoming lanelets which go straight
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
        set of IDs of outgoing lanelets which turn left
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


class OutgoingGroup:
    """
    This class represents an outgoing element of an intersection.
    It contains a group of lanelets that are going out of a lanelet in an intersection.
    """

    def __init__(
        self, outgoing_id: int, outgoing_lanelets: Optional[Set[int]] = None, incoming_group_id: Optional[int] = None
    ):
        """
        :param outgoing_id: id of the outgoing group
        :param outgoing_lanelets: set of outgoing lanelets
        :param incoming_group_id: ID of incoming group belonging to this outgoing group
        """
        self._outgoing_id = outgoing_id
        if outgoing_lanelets is None:
            self._outgoing_lanelets = set()
        else:
            self._outgoing_lanelets = outgoing_lanelets
        if incoming_group_id is None:
            self._incoming_group_id = None
        else:
            self._incoming_group_id = incoming_group_id

    def __eq__(self, other):
        if not isinstance(other, OutgoingGroup):
            warnings.warn(f"Inequality between OutgoingGroup element {repr(self)} and different type {type(other)}")
            return False

        return self._outgoing_id == other.outgoing_id and self._outgoing_lanelets == other.outgoing_lanelets

    def __hash__(self):
        return hash((self.outgoing_id, frozenset(self._outgoing_lanelets)))

    def __repr__(self):
        return f"OutgoingGroup(outgoing_id={self._outgoing_id}, outgoing_lanelets={repr(self._outgoing_lanelets)}"

    def __str__(self):
        return f"OutgoingGroup with id {self._outgoing_id} represents the outgoing lanelets {self._outgoing_lanelets}"

    @property
    def outgoing_id(self) -> int:
        return self._outgoing_id

    @outgoing_id.setter
    def outgoing_id(self, outgoing_id: int):
        assert is_natural_number(
            outgoing_id
        ), "<OutgoingGroup/outgoing_id>: Provided outgoing_id is not " "valid! id={}".format(outgoing_id)
        self._outgoing_id = outgoing_id

    @property
    def outgoing_lanelets(self) -> Optional[Set[int]]:
        return self._outgoing_lanelets

    @outgoing_lanelets.setter
    def outgoing_lanelets(self, outgoing_lanelets: Optional[Set[int]]):
        if outgoing_lanelets is None:
            self._outgoing_lanelets = set()
        else:
            self._outgoing_lanelets = outgoing_lanelets

    @property
    def incoming_group_id(self) -> Optional[int]:
        """
        ID of the corresponding incoming group of this outgoing group
        """
        return self._incoming_group_id

    @incoming_group_id.setter
    def incoming_group_id(self, incoming_group_id: Optional[int]):
        """
        :param incoming_group_id: id of the corresponding incoming group of this outgoing group
        """
        self._incoming_group_id = incoming_group_id


class CrossingGroup:
    """
    This class represents a crossing group of an intersection.
    A crossing group can consist of several lanelets. The crossing group references an incoming and/or outgoing group.
    """

    def __init__(
        self,
        crossing_id: int,
        crossing_lanelets: Optional[Set[int]] = None,
        incoming_group_id: Optional[int] = None,
        outgoing_group_id: Optional[int] = None,
    ):
        """
        :param crossing_id: crossing group ID
        :param crossing_lanelets: set of IDs of crossing lanelets
        :param incoming_group_id: ID of the incoming group the crossing group belongs to
        :param outgoing_group_id: ID of the outgoing group the crossing group belongs to
        """
        self._crossing_id = None
        self._crossing_lanelets = None
        self.crossing_id = crossing_id
        self.crossing_lanelets = crossing_lanelets
        self.incoming_group_id = incoming_group_id
        self.outgoing_group_id = outgoing_group_id

    def __eq__(self, other):
        if not isinstance(other, CrossingGroup):
            warnings.warn(f"Inequality between CrossingGroup {repr(self)} and different type {type(other)}")
            return False

        return (
            self._crossing_id == other.crossing_id
            and self._crossing_lanelets == other.crossing_lanelets
            and self._outgoing_group_id == other.outgoing_group_id
            and self._incoming_group_id == other.incoming_group_id
        )

    def __hash__(self):
        return hash((self._crossing_id, frozenset(self._crossing_lanelets), self._outgoing_group_id))

    def __str__(self):
        return (
            f"CrossingGroup with id {self._crossing_id} represents the crossing"
            f" lanelets {self._crossing_lanelets} and references the incoming group {self.incoming_group_id},"
            f" and the outgoing group {self.outgoing_group_id}"
        )

    def __repr__(self):
        return (
            f"CrossingGroup(crossing_id={self._crossing_id}, "
            f"crossing_lanelets={self._crossing_lanelets}, incoming_group_id={self.incoming_group_id},"
            f"outgoing_group_id={self.outgoing_group_id}"
        )

    @property
    def crossing_id(self) -> int:
        """
        ID of crossing group
        """
        return self._crossing_id

    @crossing_id.setter
    def crossing_id(self, c_id: int):
        """
        :param c_id: ID of crossing
        """
        assert is_natural_number(
            c_id
        ), "<CrossingGroup/incoming_id>: Provided crossing_id is not " "valid! id={}".format(c_id)
        self._crossing_id = c_id

    @property
    def crossing_lanelets(self) -> Set[int]:
        """
        set of IDs of incoming lanelets
        """
        return self._crossing_lanelets

    @crossing_lanelets.setter
    def crossing_lanelets(self, crossing_lanelets: Set[int]):
        """
        :param crossing_lanelets: set of IDs of crossing lanelets
        """
        self._crossing_lanelets = crossing_lanelets

    @property
    def outgoing_group_id(self) -> Optional[int]:
        """
        ID of the outgoing element that contains outgoing lanelets for this intersection element
        """
        return self._outgoing_group_id

    @outgoing_group_id.setter
    def outgoing_group_id(self, outgoing_group_id: Optional[int]):
        """
        :param outgoing_group_id: ID of the referenced outgoing group
        """
        self._outgoing_group_id = outgoing_group_id

    @property
    def incoming_group_id(self) -> Optional[int]:
        """
        ID of the referenced incoming group
        """
        return self._incoming_group_id

    @incoming_group_id.setter
    def incoming_group_id(self, incoming_group_id: Optional[int]):
        """
        :param incoming_group_id: id of the referenced incoming group
        """
        self._incoming_group_id = incoming_group_id


class Intersection:
    """
    This class represent an intersection.
    An intersection element is defined by at least one incoming and an optional outgoing group element.
    """

    def __init__(
        self,
        intersection_id: int,
        incomings: List[IncomingGroup],
        outgoings: Optional[List[OutgoingGroup]] = None,
        crossings: Optional[List[CrossingGroup]] = None,
    ):
        """
        :param intersection_id: ID of intersection element
        :param incomings: set of incoming elements in intersection
        :param outgoings: set of outgoing elements in intersection
        :param crossings: set of crossing elements in intersection
        """
        self._intersection_id = None
        self._incomings = None
        self._crossings = None

        self.intersection_id = intersection_id
        self.incomings = incomings

        if outgoings is None:
            self._outgoings = []
        else:
            self.outgoings = outgoings
        if crossings is None:
            self._crossings = []
        else:
            self.crossings = crossings

    def __eq__(self, other):
        if not isinstance(other, Intersection):
            warnings.warn(f"Inequality between Intersection {repr(self)} and different type {type(other)}")
            return False

        list_elements_eq = True
        incomings = {incoming.incoming_id: incoming for incoming in self._incomings}
        incomings_other = {incoming.incoming_id: incoming for incoming in other.incomings}
        outgoings = {outgoing.outgoing_id: outgoing for outgoing in self._outgoings}
        outgoings_other = {outgoing.outgoing_id: outgoing for outgoing in other.outgoings}
        crossings = {crossing.crossing_id: crossing for crossing in self._crossings}
        crossings_other = {crossing.crossing_id: crossing for crossing in other.crossings}
        intersection_eq = (
            len(incomings) == len(incomings_other)
            and len(outgoings) == len(outgoings_other)
            and len(crossings) == len(crossings_other)
        )
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

        for k in crossings.keys():
            if k not in crossings_other:
                intersection_eq = False
                continue
            if crossings.get(k) != crossings_other.get(k):
                list_elements_eq = False

        return list_elements_eq and intersection_eq and self._intersection_id == other.intersection_id

    def __hash__(self):
        return hash(
            (self._intersection_id, frozenset(self._incomings), frozenset(self._outgoings), frozenset(self._crossings))
        )

    def __str__(self):
        return (
            f"Intersection with id {self._intersection_id} consisting of {len(self._incomings)} incoming elements "
            f"and outgoing elements {self._outgoings} and crossing elements {self._crossings}"
        )

    def __repr__(self):
        return (
            f"Intersection(intersection_id={self._intersection_id}, incomings={repr(self._incomings)}, "
            f"outgoings={self._outgoings}, crossings={self._crossings})"
        )

    @property
    def intersection_id(self) -> int:
        """
        ID of intersection element
        """
        return self._intersection_id

    @intersection_id.setter
    def intersection_id(self, i_id: int):
        """
        :param i_id: ID of intersection element
        """
        assert is_natural_number(
            i_id
        ), "<Intersection/intersection_id>: Provided intersection_id is not " "valid! id={}".format(i_id)
        self._intersection_id = i_id

    @property
    def incomings(self) -> List[IncomingGroup]:
        """
        set of incoming elements in intersection
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
        set of outgoing elements in intersection
        """
        return self._outgoings

    @outgoings.setter
    def outgoings(self, outgoings: List[OutgoingGroup]):
        self._outgoings = outgoings

    @property
    def crossings(self) -> List[CrossingGroup]:
        """
        set of crossing elements in intersection
        """
        return self._crossings

    @crossings.setter
    def crossings(self, crossings: List[CrossingGroup]):
        self._crossings = crossings

    @property
    def map_incoming_lanelets(self) -> Dict[int, IncomingGroup]:
        """
        Maps all incoming lanelet ids to IntersectionIncomingElement
        """
        return {l_id: inc for inc in self.incomings for l_id in inc.incoming_lanelets}

    @property
    def map_outgg_lanelets(self) -> Dict[int, OutgoingGroup]:
        """
        Maps all outgoing group lanelet ids to IntersectionOutgoingElement
        """
        return {l_id: outgg for outgg in self.outgoings for l_id in outgg.outgoing_lanelets}

    def find_incoming_by_id(self, incoming_id: int) -> Union[IncomingGroup, None]:
        """
        Finds incoming of intersection by ID.
        :return: Incoming of intersection.
        """
        for incoming in self._incomings:
            if incoming.incoming_id == incoming_id:
                return incoming
        return None

    def compute_member_lanelets(self, lanelet_network: LaneletNetwork):
        """
        Returns all lanelets of an intersection
        :return: Set of lanelets IDs
        """
        outgoing_lanelets = set()
        incoming_lanelets = set()
        intermediate_lanelets = set()

        for incoming_group in self.incomings:
            incoming_lanelets = incoming_lanelets.union(incoming_group.incoming_lanelets)

            outgoing_lanelets = outgoing_lanelets.union(incoming_group.outgoing_left)
            for lanelet_id in incoming_group.outgoing_left:
                if lanelet_network.find_lanelet_by_id(lanelet_id) is not None:
                    outgoing_lanelets = outgoing_lanelets.union(
                        set(lanelet_network.find_lanelet_by_id(lanelet_id).successor)
                    )

            outgoing_lanelets = outgoing_lanelets.union(incoming_group.outgoing_right)
            for lanelet_id in incoming_group.outgoing_right:
                if lanelet_network.find_lanelet_by_id(lanelet_id) is not None:
                    outgoing_lanelets = outgoing_lanelets.union(
                        set(lanelet_network.find_lanelet_by_id(lanelet_id).successor)
                    )

            outgoing_lanelets = outgoing_lanelets.union(incoming_group.outgoing_straight)
            for lanelet_id in incoming_group.outgoing_straight:
                if lanelet_network.find_lanelet_by_id(lanelet_id) is not None:
                    outgoing_lanelets = outgoing_lanelets.union(
                        set(lanelet_network.find_lanelet_by_id(lanelet_id).successor)
                    )

        if self.outgoings is not None:
            for outgoing_group in self.outgoings:
                if outgoing_group.outgoing_lanelets is not None:
                    outgoing_lanelets = outgoing_lanelets.union(outgoing_group.outgoing_lanelets)
        # find all intermediate lanelets in the intersection
        for inc_lanelet in incoming_lanelets:
            tmp_lanelets = set()
            tmp_lanelets.add(inc_lanelet)
            while len(tmp_lanelets) > 0:
                tmp_lanelet = tmp_lanelets.pop()
                if tmp_lanelet not in outgoing_lanelets:
                    intermediate_lanelets.add(tmp_lanelet)

                    tmp_succesor_lanelets = (
                        lanelet_network.find_lanelet_by_id(tmp_lanelet).successor
                        if lanelet_network.find_lanelet_by_id(tmp_lanelet) is not None
                        else None
                    )
                    if tmp_succesor_lanelets is not None:
                        for tmp_suc_lanelet in tmp_succesor_lanelets:
                            if tmp_suc_lanelet not in outgoing_lanelets:
                                intermediate_lanelets.add(tmp_suc_lanelet)

        return incoming_lanelets.union(intermediate_lanelets, outgoing_lanelets)
