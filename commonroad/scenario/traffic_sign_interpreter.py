from typing import Set, Union, FrozenSet
import enum
from commonroad.scenario.traffic_sign import TrafficSignIDSpain, TrafficSignIDUsa, TrafficSignIDGermany, \
    TrafficSignIDZamunda, TrafficSignIDChina, TrafficSignIDRussia, SupportedTrafficSignCountry
from commonroad.scenario.lanelet import LaneletNetwork

__author__ = "Sebastian Maierhofer"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["CAR@TUM"]
__version__ = "2020.1"
__maintainer__ = "Sebastian Maierhofer"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "Release"


class TrafficSigInterpreter:
    """ Class to extract traffic sign information from the road network"""
    def __init__(self, country: SupportedTrafficSignCountry, lanelet_network: LaneletNetwork):
        """
        Constructor

        :param country: country of CommonRoad scenario
        :param lanelet_network: CommonRoad lanelet network
        """
        self.country = country
        self.traffic_sign_ids = self._relevant_traffic_sign_ids(country)
        self._lanelet_network = lanelet_network
        self._traffic_sign_history = {}

    @staticmethod
    def _relevant_traffic_sign_ids(country: SupportedTrafficSignCountry) -> enum:
        """
        Extracts country specific traffic sign enum

        :param country: country of CommonRoad scenario
        :returns country specific traffic sign enum
        """
        if country == SupportedTrafficSignCountry.SPAIN:
            return TrafficSignIDSpain
        elif country == SupportedTrafficSignCountry.GERMANY:
            return TrafficSignIDGermany
        elif country == SupportedTrafficSignCountry.ZAMUNDA:
            return TrafficSignIDZamunda
        elif country == SupportedTrafficSignCountry.USA:
            return TrafficSignIDUsa
        elif country == SupportedTrafficSignCountry.CHINA:
            return TrafficSignIDChina
        elif country == SupportedTrafficSignCountry.RUSSIA:
            return TrafficSignIDRussia
        else:
            return TrafficSignIDZamunda

    def _in_history(self, traffic_element_id: int, lanelet_ids: FrozenSet[int]) -> bool:
        """
        Evaluates if traffic element was already evaluated with these lanelets

        :param traffic_element_id: ID of CommonRoad traffic element
        :param lanelet_ids: set of lanelets which should be considered
        :returns boolean indicating if information is stored
        """
        if self._traffic_sign_history.get(traffic_element_id) is not None \
                and self._traffic_sign_history.get(traffic_element_id).get(frozenset(lanelet_ids)) \
                is not None:
            return True
        else:
            return False

    def _add_to_history(self, traffic_element_id: int, lanelet_ids: FrozenSet[int], value: Union[int, str, None]):
        """
        Adds value to history for faster evaluation

        :param traffic_element_id: ID of CommonRoad traffic element
        :param lanelet_ids: set of lanelets which should be considered
        :param value: value to add
        :returns boolean indicating if information is stored
        """
        if self._traffic_sign_history.get(traffic_element_id) is None:
            lanelets_value = {frozenset(lanelet_ids): value}
            self._traffic_sign_history[traffic_element_id] = lanelets_value
        else:
            self._traffic_sign_history[traffic_element_id][frozenset(lanelet_ids)] = value

    def speed_limit(self, lanelet_ids: FrozenSet[int]) -> Union[float, None]:
        """
        Extracts the maximum speed limit of provided lanelets

        :param lanelet_ids: set of lanelets which should be considered
        :returns speed limit of provided lanelets or None if no speed limit exists
        """
        if self._in_history(self.traffic_sign_ids.MAXSPEED.value, frozenset(lanelet_ids)):
            return self._traffic_sign_history.get(self.traffic_sign_ids.MAXSPEED.value).get(frozenset(lanelet_ids))

        speed_limits = []
        for lanelet_id in lanelet_ids:
            lanelet = self._lanelet_network.find_lanelet_by_id(lanelet_id)
            for traffic_sign_id in lanelet.traffic_signs:
                traffic_sign = self._lanelet_network.find_traffic_sign_by_id(traffic_sign_id)
                for elem in traffic_sign.traffic_sign_elements:
                    if elem.traffic_sign_element_id == self.traffic_sign_ids.MAXSPEED:
                        speed_limits.append(float(elem.additional_values[0]))

        if len(speed_limits) == 0:
            speed_limit = None
        else:
            speed_limit = min(speed_limits)
        self._add_to_history(self.traffic_sign_ids.MAXSPEED.value, lanelet_ids, speed_limit)
        return speed_limit

    def required_speed(self, lanelet_ids: FrozenSet[int]) -> Union[float, None]:
        """
        Extracts the required speed a vehicle has to drive on a set of lanelets

        :param lanelet_ids: IDs of lanelets the vehicle is on
        :returns minimum required speed of provided lanelets or None if no required speed exists
        """
        if self._in_history(self.traffic_sign_ids.MINSPEED.value, frozenset(lanelet_ids)):
            return self._traffic_sign_history.get(self.traffic_sign_ids.MINSPEED.value).get(frozenset(lanelet_ids))

        required_velocities = []
        for lanelet_id in lanelet_ids:
            lanelet = self._lanelet_network.find_lanelet_by_id(lanelet_id)
            for traffic_sign_id in lanelet.traffic_signs:
                traffic_sign = self._lanelet_network.find_traffic_sign_by_id(traffic_sign_id)
                for elem in traffic_sign.traffic_sign_elements:
                    if elem.traffic_sign_element_id == self.traffic_sign_ids.MINSPEED:
                        required_velocities.append(float(elem.additional_values[0]))

        if len(required_velocities) == 0:
            required_velocity = None
        else:
            required_velocity = max(required_velocities)
        self._add_to_history(self.traffic_sign_ids.MINSPEED.value, lanelet_ids, required_velocity)

        return required_velocity
