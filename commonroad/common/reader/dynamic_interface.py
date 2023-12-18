import warnings
from typing import List, Union, Dict

from commonroad.common.common_scenario import ScenarioMetaInformation
from commonroad.scenario.obstacle import StaticObstacle, DynamicObstacle, EnvironmentObstacle, PhantomObstacle
from commonroad.common.common_scenario import Environment
from commonroad.scenario.traffic_light import TrafficLightCycle
from commonroad.scenario.traffic_sign import TrafficSignValue


class DynamicInterface:
    """
    Class that represents the dynamic interface used in the protobuf format
    """

    def __init__(
        self,
        dynamic_meta_information: ScenarioMetaInformation,
        environment: Environment,
        traffic_light_cycle: List[TrafficLightCycle] = None,
        traffic_sign_value: List[TrafficSignValue] = None,
        static_obstacles: List[StaticObstacle] = None,
        dynamic_obstacles: List[DynamicObstacle] = None,
        environment_obstacles: List[EnvironmentObstacle] = None,
        phantom_obstacles: List[PhantomObstacle] = None,
    ):
        """
        :param dynamic_meta_information: information about the dynamic
        :param environment: environment of the dynamic
        :param traffic_light_cycle: list of traffic light cycles of the dynamic
        :param traffic_sign_value: list of traffic sign values of the dynamic
        :param static_obstacles: list of static obstacles of the dynamic
        :param dynamic_obstacles: list of dynamic obstacles of the dynamic
        :param environment_obstacles: list of environment obstacles of the dynamic
        :param phantom_obstacles: list of phantom obstacles of the dynamic
        """
        self._dynamic_meta_information = dynamic_meta_information
        self._environment = environment

        if traffic_light_cycle is None:
            self._traffic_light_cycle = []
        else:
            self._traffic_light_cycle = traffic_light_cycle

        if traffic_sign_value is None:
            self._traffic_sign_value = []
        else:
            self._traffic_sign_value = traffic_sign_value

        if static_obstacles is None:
            self._static_obstacles = []
        else:
            self._static_obstacles = static_obstacles

        if dynamic_obstacles is None:
            self._dynamic_obstacles = []
        else:
            self._dynamic_obstacles = dynamic_obstacles

        if environment_obstacles is None:
            self._environment_obstacles = []
        else:
            self._environment_obstacles = environment_obstacles

        if phantom_obstacles is None:
            self._phantom_obstacles = []
        else:
            self._phantom_obstacles = phantom_obstacles

        # Dictionary that maps traffic light cycles with their corresponding traffic light id's used
        # in the protobuf format.
        self._light_cycle_id_dict = Dict

    def __eq__(self, other):
        if not isinstance(other, DynamicInterface):
            warnings.warn(f"Inequality between DynamicInterface {repr(self)} and different type {type(other)}")
            return False

        return (
            self.dynamic_meta_information == other.dynamic_meta_information
            and self._environment == other.environment
            and self._static_obstacles == other.static_obstacles
            and self._traffic_light_cycle == other._traffic_light_cycle
            and self._traffic_sign_value == other._traffic_sign_value
            and self._dynamic_obstacles == other.dynamic_obstacles
            and self._environment_obstacles == other.environment_obstacles
            and self._phantom_obstacles == other.phantom_obstacles
        )

    def __hash__(self):
        return hash(
            (
                self._dynamic_meta_information,
                self._environment,
                frozenset(self._traffic_light_cycle),
                frozenset(self._traffic_sign_value),
                frozenset(self._static_obstacles),
                frozenset(self._dynamic_obstacles),
                frozenset(self._environment_obstacles),
                frozenset(self._phantom_obstacles),
            )
        )

    @property
    def dynamic_meta_information(self) -> ScenarioMetaInformation:
        """information about the dynamic."""
        return self._dynamic_meta_information

    @dynamic_meta_information.setter
    def dynamic_meta_information(self, information: ScenarioMetaInformation):
        assert isinstance(information, ScenarioMetaInformation), (
            "<DynamicInterface/information>: provided information"
            " is not a ScenarioMetaInformation! type = {}".format(type(information))
        )
        self._dynamic_meta_information = information

    @property
    def environment(self) -> Environment:
        """environment of the dynamic."""
        return self._environment

    @environment.setter
    def environment(self, environment: Environment):
        assert isinstance(
            environment, Environment
        ), "<DynamicInterface/environment>: provided environment is not an " "Environment! type = {},".format(
            type(environment)
        )
        self._environment = environment

    @property
    def traffic_light_cycle(self) -> Union[None, List[TrafficLightCycle]]:
        """list of traffic light cycles of the dynamic."""
        return self._traffic_light_cycle

    @traffic_light_cycle.setter
    def traffic_light_cycle(self, traffic_light_cycle: Union[None, TrafficLightCycle]):
        if traffic_light_cycle is None:
            self._traffic_light_cycle = []
        else:
            self._traffic_light_cycle = traffic_light_cycle

    @property
    def traffic_sign_value(self) -> Union[None, List[TrafficSignValue]]:
        """list of traffic sign values of the dynamic."""
        return self._traffic_sign_value

    @traffic_sign_value.setter
    def traffic_sign_value(self, traffic_sign_value: Union[None, List[TrafficSignValue]]):
        if traffic_sign_value is None:
            self._traffic_sign_value = []
        else:
            self._traffic_sign_value = traffic_sign_value

    @property
    def static_obstacles(self) -> Union[None, List[StaticObstacle]]:
        """list of static obstacles of the dynamic."""
        return self._static_obstacles

    @static_obstacles.setter
    def static_obstacles(self, static_obstacles: Union[None, List[StaticObstacle]]):
        if static_obstacles is None:
            self._static_obstacles = []
        else:
            self._static_obstacles = static_obstacles

    @property
    def dynamic_obstacles(self) -> Union[None, List[DynamicObstacle]]:
        """list of dynamic obstacles of the dynamic."""
        return self._dynamic_obstacles

    @dynamic_obstacles.setter
    def dynamic_obstacles(self, dynamic_obstacles: Union[None, List[DynamicObstacle]]):
        if dynamic_obstacles is None:
            self._dynamic_obstacles = []
        else:
            self._dynamic_obstacles = dynamic_obstacles

    @property
    def environment_obstacles(self) -> Union[None, List[EnvironmentObstacle]]:
        """list of environment obstacles of the dynamic."""
        return self._environment_obstacles

    @environment_obstacles.setter
    def environment_obstacles(self, environment_obstacles: Union[None, List[EnvironmentObstacle]]):
        if environment_obstacles is None:
            self._environment_obstacles = []
        else:
            self._environment_obstacles = environment_obstacles

    @property
    def phantom_obstacles(self) -> Union[None, List[PhantomObstacle]]:
        """list of phantom obstacles of the dynamic."""
        return self._phantom_obstacles

    @phantom_obstacles.setter
    def phantom_obstacles(self, phantom_obstacles: Union[List, PhantomObstacle]):
        if phantom_obstacles is None:
            self._phantom_obstacles = []
        else:
            self._phantom_obstacles = phantom_obstacles

    @property
    def light_cycle_id_dict(self) -> Dict[int, TrafficLightCycle]:
        """
        Dictionary that maps traffic light cycles with their corresponding traffic light id's used
        in the protobuf format.
        """
        return self._light_cycle_id_dict

    @light_cycle_id_dict.setter
    def light_cycle_id_dict(self, light_cycle_id_dict: Dict[int, TrafficLightCycle]):
        self._light_cycle_id_dict = light_cycle_id_dict
