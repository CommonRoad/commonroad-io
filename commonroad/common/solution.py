import os
import platform
import re
import subprocess
import warnings
from xml.dom import minidom
import numpy as np
import xml.etree.ElementTree as et
from enum import Enum, unique
from typing import List, Tuple, Union
from datetime import datetime

from commonroad import SUPPORTED_COMMONROAD_VERSIONS
from commonroad.scenario.trajectory import State, Trajectory

__author__ = "Murat Üste, Christina Miller, Moritz Klischat"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["BMW CAR@TUM"]
__version__ = "2020.1"
__maintainer__ = "Moritz Klischat"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "Released"

@unique
class VehicleType(Enum):
    FORD_ESCORT = 1
    BMW_320i = 2
    VW_VANAGON = 3


@unique
class VehicleModel(Enum):
    PM = 0
    ST = 1
    KS = 2
    MB = 3


@unique
class CostFunction(Enum):
    JB1 = 0
    SA1 = 1
    WX1 = 2
    SM1 = 3
    SM2 = 4
    SM3 = 5


@unique
class StateFields(Enum):
    """
    State Fields enum class for defining the state fields for vehicle models for different trajectory types.

    PM | ST | KS | MB -> Corresponding state fields for trajectory states
    Input             -> Input fields for ST, KS, and MB vehicle models
    PMInput           -> Input fields for PM vehicle model.

    Note: If you change the order of field names, don't forget to change the order on the XMLStateFields enum as well,
    because the indexes have to match.
    """
    PM = ['position', 'velocity', 'velocity_y', 'time_step']
    ST = ['position', 'steering_angle', 'velocity', 'orientation', 'yaw_rate', 'slip_angle', 'time_step']
    KS = ['position', 'steering_angle', 'velocity', 'orientation', 'time_step']
    MB = ['position', 'steering_angle', 'velocity', 'orientation', 'yaw_rate', 'roll_angle', 'roll_rate', 'pitch_angle',
          'pitch_rate', 'velocity_y', 'position_z', 'velocity_z', 'roll_angle_front', 'roll_rate_front',
          'velocity_y_front', 'position_z_front', 'velocity_z_front', 'roll_angle_rear', 'roll_rate_rear',
          'velocity_y_rear', 'position_z_rear', 'velocity_z_rear', 'left_front_wheel_angular_speed',
          'right_front_wheel_angular_speed', 'left_rear_wheel_angular_speed', 'right_rear_wheel_angular_speed',
          'delta_y_f', 'delta_y_r', 'time_step']
    Input = ['steering_angle_speed', 'acceleration', 'time_step']
    PMInput = ['acceleration', 'acceleration_y', 'time_step']


@unique
class XMLStateFields(Enum):
    """
    XML names of the state fields for vehicle models for different trajectory types.

    PM | ST | KS | MB -> Corresponding xml names of the state fields for trajectory states
    Input             -> XML names of the input fields for ST, KS, and MB vehicle models
    PMInput           -> XML names of the input fields for PM vehicle model.

    Note: If you change the order of xml names, don't forget to change the order on the StateFields enum as well,
    because the indexes have to match.
    """
    PM = [('x', 'y'), 'xVelocity', 'yVelocity', 'time']
    ST = [('x', 'y'), 'steeringAngle', 'velocity', 'orientation', 'yawRate', 'slipAngle', 'time']
    KS = [('x', 'y'), 'steeringAngle', 'velocity', 'orientation', 'time']
    MB = [('x', 'y'), 'steeringAngle', 'velocity', 'orientation', 'yawRate', 'rollAngle', 'rollRate', 'pitchAngle',
          'pitchRate', 'yVelocity', 'zPosition', 'zVelocity', 'rollAngleFront', 'rollRateFront',
          'yVelocityFront', 'zPositionFront', 'zVelocityFront', 'rollAngleRear', 'rollRateRear',
          'yVelocityRear', 'zPositionRear', 'zVelocityRear', 'leftFrontWheelAngularSpeed',
          'rightFrontWheelAngularSpeed', 'leftRearWheelAngularSpeed', 'rightRearWheelAngularSpeed',
          'deltaYf', 'deltaYr', 'time']
    Input = ['steeringAngleSpeed', 'acceleration', 'time']
    PMInput = ['xAcceleration', 'yAcceleration', 'time']


@unique
class StateType(Enum):
    """
    State Type enum class.

    PM | ST | KS | MB -> Corresponding state type for trajectory states
    Input             -> Input type for ST, KS, and MB vehicle models
    PMInput           -> Input type for PM vehicle model.
    """
    PM = 'pmState'
    ST = 'stState'
    KS = 'ksState'
    MB = 'mbState'
    Input = 'input'
    PMInput = 'pmInput'

    @property
    def fields(self) -> List[str]:
        """
        Returns the state fields for the state type.

        :return: List[str] - State fields as list
        """
        return StateFields[self.name].value

    @property
    def xml_fields(self) -> List[str]:
        """
        Returns the xml state fields for the state type.

        :return: List[str] - XML names of the state fields as list
        """
        return XMLStateFields[self.name].value

    @classmethod
    def get_state_type(cls, state: State) -> 'StateType':
        """
        Returns the corresponding StateType for the given State object by matching State object's attributes
        to the state fields.

        :param state: State - CommonRoad State object
        :return: State-Type
        """
        attrs = state.attributes
        for state_fields in StateFields:
            if not len(attrs) == len(state_fields.value): continue
            if not all([attr in state_fields.value for attr in attrs]): continue
            return cls[state_fields.name]
        raise Exception('Given state is not valid!')


@unique
class TrajectoryType(Enum):
    """
    Trajectory Type enum class.

    PM | ST | KS | MB -> Corresponding trajectory type for the vehicle models
    Input             -> InputVector type for ST, KS, and MB vehicle models
    PMInput           -> InputVector type for PM vehicle model.
    """
    PM = 'pmTrajectory'
    ST = 'stTrajectory'
    KS = 'ksTrajectory'
    MB = 'mbTrajectory'
    Input = 'inputVector'
    PMInput = 'pmInputVector'

    @property
    def state_type(self) -> StateType:
        """
        Returns the StateType corresponding to the TrajectoryType

        :return: StateType
        """
        return StateType[self.name]

    @classmethod
    def get_trajectory_type(cls, trajectory: Trajectory) -> 'TrajectoryType':
        """
        Returns the corresponding TrajectoryType for the given Trajectory object based on the StateType of its states.

        :param trajectory: Trajectory - CommonRoad Trajectory object
        :return: TrajectoryType
        """
        state_type = StateType.get_state_type(trajectory.state_list[0])
        return cls[state_type.name]

    def valid_vehicle_model(self, vehicle_model: VehicleModel) -> bool:
        """
        Checks whether given vehicle model is valid for the TrajectoryType.

        :param vehicle_model: VehicleModel - CommonRoad enum for vehicle models
        :return: bool - True if the vehicle model is valid for the TrajectoryType
        """
        return any([
            self.name == 'Input' and vehicle_model in [VehicleModel.KS, VehicleModel.ST, VehicleModel.MB],
            self.name == 'PMInput' and vehicle_model == VehicleModel.PM,
            self.name == vehicle_model.name
        ])


class SupportedCostFunctions(Enum):
    """
    Enum class for specifying which cost functions are supported for which vehicle model
    """
    PM = [CostFunction.JB1, CostFunction.WX1]
    ST = [cost_function for cost_function in CostFunction]  # Supports all cost functions
    KS = [cost_function for cost_function in CostFunction]  # Supports all cost functions
    MB = [cost_function for cost_function in CostFunction]  # Supports all cost functions


class PlanningProblemSolution:
    def __init__(self,
                 planning_problem_id: int,
                 vehicle_model: VehicleModel,
                 vehicle_type: VehicleType,
                 cost_function: CostFunction,
                 trajectory: Trajectory):
        """
        Constructor for the PlanningProblemSolution class.

        :param planning_problem_id: ID of the planning problem
        :param vehicle_model: VehicleModel used for the solution
        :param vehicle_type: VehicleType used for the solution
        :param cost_function: CostFunction the solution will be evaluated with
        :param trajectory: Ego vehicle's trajectory for the solution.
        """

        self.planning_problem_id = planning_problem_id
        self._vehicle_model = vehicle_model
        self.vehicle_type = vehicle_type
        self._cost_function = cost_function
        self._trajectory = trajectory
        self._trajectory_type = TrajectoryType.get_trajectory_type(self._trajectory)

        self._check_trajectory_supported(self._vehicle_model, self._trajectory_type)
        self._check_cost_supported(self._vehicle_model, self._cost_function)

    @staticmethod
    def _check_cost_supported(vehicle_model: VehicleModel, cost_function: CostFunction) -> bool:
        """
        Checks whether given cost function is supported by the given vehicle model.

        :param vehicle_model: VehicleModel
        :param cost_function: CostFunction
        :return: bool - True if supported.
        """
        supported_costs = SupportedCostFunctions[vehicle_model.name].value
        if cost_function not in supported_costs:
            raise Exception("Cost function %s isn't supported for %s model!" % (cost_function.name, vehicle_model.name))
        return True

    @staticmethod
    def _check_trajectory_supported(vehicle_model: VehicleModel, trajectory_type: TrajectoryType) -> bool:
        """
        Checks whether given vehicle model is valid for the given trajectory type.

        :param vehicle_model: VehicleModel
        :param trajectory_type: TrajectoryType
        :return: bool - True if valid.
        """
        if not trajectory_type.valid_vehicle_model(vehicle_model):
            raise Exception('Vehicle model %s is not valid for the trajectory type %s!'
                            % (vehicle_model.name, trajectory_type.name))
        return True

    @property
    def vehicle_model(self) -> VehicleModel:
        """ VehicleModel of the PlanningProblemSolution """
        return self._vehicle_model

    @vehicle_model.setter
    def vehicle_model(self, vehicle_model: VehicleModel):
        self._check_trajectory_supported(vehicle_model, self._trajectory_type)
        self._check_cost_supported(vehicle_model, self.cost_function)

        self._vehicle_model = vehicle_model

    @property
    def cost_function(self) -> CostFunction:
        """ CostFunction of the PlanningProblemSolution """
        return self._cost_function

    @cost_function.setter
    def cost_function(self, cost_function: CostFunction):
        self._check_cost_supported(self.vehicle_model, cost_function)
        self._cost_function = cost_function

    @property
    def trajectory(self) -> Trajectory:
        """ Trajectory of the PlanningProblemSolution """
        return self._trajectory

    @trajectory.setter
    def trajectory(self, trajectory: Trajectory):
        trajectory_type = TrajectoryType.get_trajectory_type(trajectory)
        self._check_trajectory_supported(self.vehicle_model, trajectory_type)

        self._trajectory = trajectory
        self._trajectory_type = trajectory_type

    @property
    def trajectory_type(self) -> TrajectoryType:
        """
        TrajectoryType of the PlanningProblemSolution.
        Dynamically assigned when there is a change of trajectory.
        """
        return self._trajectory_type

    @property
    def vehicle_id(self) -> str:
        """
        Returns the vehicle id as string.

        Example:
            VehicleModel = PM
            VehicleType = FORD_ESCORT
            Vehicle ID = PM1

        :return: Vehicle model ID
        """
        return self.vehicle_model.name + str(self.vehicle_type.value)

    @property
    def cost_id(self) -> str:
        """
        Returns cost function id as str.

        Example:
            CostFunction = JB1
            Cost ID = JB1

        :return: Cost function ID
        """
        return self.cost_function.name


class Solution:
    """Stores a solution to a CommonRoad benchmark and additional meta data."""
    def __init__(self,
                 scenario_id: str,
                 commonroad_version: str,
                 planning_problem_solutions: List[PlanningProblemSolution],
                 date: datetime = datetime.today(),
                 computation_time: Union[float, None] = None,
                 processor_name: Union[str, None] = None):
        """
        :param scenario_id: Scenario ID of the Solution
        :param commonroad_version: valid CommonRoad Version (see `:py:data:`~common.file_reader.SUPPORTED_COMMONROAD_VERSIONS`)
        :param planning_problem_solutions: List of PlanningProblemSolution for corresponding
        to the planning problems of the scenario
        :param date: The date solution was produced. Default=datetime.today()
        :param computation_time: The computation time it took for the Solution. Default=None
        :param processor_name: The processor model used for the Solution. Determined automatically if set to 'auto'.
        Default=None.
        """
        assert commonroad_version in SUPPORTED_COMMONROAD_VERSIONS
        self.scenario_id = scenario_id
        self.commonroad_version = commonroad_version
        self.planning_problem_solutions = planning_problem_solutions
        self.date = date
        self.computation_time = computation_time
        self.processor_name = processor_name

    @property
    def benchmark_id(self) -> str:
        """
        Returns the benchmark id of the solution as string.

        Example:
            Scenario ID = TEST
            VehicleModel = PM
            VehicleType = FORD_ESCORT
            CostFunction = JB1
            Version = 2018b

            Benchmark ID = PM1:JB1:TEST:2018b

        Collaborative Solution Example:
            Scenario ID = TEST
            1st VehicleModel = PM
            1st VehicleType = FORD_ESCORT
            1st CostFunction = JB1
            2nd VehicleModel = PM
            2nd VehicleType = VW_VANAGON
            2nd CostFunction = SA1
            Version = 2018b

            Benchmark ID = [PM1,PM3]:[JB1,SA1]:TEST:2020a

        :return: Benchmark ID
        """
        vehicle_ids = self.vehicle_ids
        cost_ids = self.cost_ids
        vehicles_str = vehicle_ids[0] if len(vehicle_ids) == 1 else '[%s]' % ','.join(vehicle_ids)
        costs_str = cost_ids[0] if len(cost_ids) == 1 else '[%s]' % ','.join(cost_ids)
        return '%s:%s:%s:%s' % (vehicles_str, costs_str, self.scenario_id, self.commonroad_version)

    @property
    def vehicle_ids(self) -> List[str]:
        """
        Returns the list of vehicle ids of all PlanningProblemSolutions of the Solution

        Example:
            1st PlanningProblemSolution Vehicle ID = PM1
            2nd PlanningProblemSolution Vehicle ID = PM3

            Vehicle IDS = [PM1, PM3]

        :return: List of vehicle IDs
        """
        return [pp_solution.vehicle_id for pp_solution in self.planning_problem_solutions]

    @property
    def cost_ids(self) -> List[str]:
        """
        Returns the list of cost ids of all PlanningProblemSolutions of the Solution

        Example:
            1st PlanningProblemSolution Cost ID = JB1
            2nd PlanningProblemSolution Cost ID = SA1

            Cost IDS = [JB1, SA1]

        :return: List of cost function IDs
        """
        return [pp_solution.cost_id for pp_solution in self.planning_problem_solutions]

    @property
    def planning_problem_ids(self) -> List[int]:
        """
        Returns the list of planning problem ids of all PlanningProblemSolutions of the Solution

        Example:
            1st PlanningProblemSolution planning_problem_id = 0
            2nd PlanningProblemSolution planning_problem_id = 1

            planning_problem_ids = [0, 1]

        :return: List of planning problem ids
        """
        return [pp_solution.planning_problem_id for pp_solution in self.planning_problem_solutions]

    @property
    def trajectory_types(self) -> List[TrajectoryType]:
        """
        Returns the list of trajectory types of all PlanningProblemSolutions of the Solution

        Example:
            1st PlanningProblemSolution trajectory_type = TrajectoryType.PM
            2nd PlanningProblemSolution trajectory_type = TrajectoryType.KS

            trajectory_types = [TrajectoryType.PM, TrajectoryType.KS]

        :return: List of trajectory types
        """
        return [pp_solution.trajectory_type for pp_solution in self.planning_problem_solutions]


class CommonRoadSolutionReader:
    """Reads solution xml files created with the CommonRoadSolutionWriter"""
    # TODO Add parser check
    # TODO Prepare proper error messages to show to the user.

    @classmethod
    def open(cls, filepath: str) -> Solution:
        """
        Opens and parses the Solution XML file located on the given path.

        :param filepath: Path to the file.
        :return: Solution
        """
        tree = et.parse(filepath)
        root_node = tree.getroot()
        return cls._parse_solution(root_node)

    @classmethod
    def fromstring(cls, file: str) -> Solution:
        """
        Parses the given Solution XML string.

        :param file: xml file as string
        :return: Solution
        """
        root_node = et.fromstring(file)
        return cls._parse_solution(root_node)

    @classmethod
    def _parse_solution(cls, root_node: et.Element) -> Solution:
        """ Parses the Solution XML root node. """
        benchmark_id, date, computation_time, processor_name = cls._parse_header(root_node)
        vehicle_ids, cost_ids, scenario_id, version = cls._parse_benchmark_id(benchmark_id)
        pp_solutions = [cls._parse_planning_problem_solution(vehicle_ids[idx], cost_ids[idx], trajectory_node)
                        for idx, trajectory_node in enumerate(root_node)]
        return Solution(scenario_id, version, pp_solutions, date, computation_time, processor_name)

    @staticmethod
    def _parse_header(root_node: et.Element) -> Tuple[str, Union[None, datetime], Union[None, float], Union[None, str]]:
        """ Parses the header attributes for the given Solution XML root node. """
        benchmark_id = root_node.get('benchmark_id')
        date = root_node.attrib.get('date')  # None if not found
        if date is not None: date = datetime.strptime(date, '%Y-%m-%d')
        computation_time = root_node.attrib.get('computation_time')
        if computation_time is not None: computation_time = float(computation_time)
        processor_name = root_node.attrib.get('processor_name')
        return benchmark_id, date, computation_time, processor_name

    @classmethod
    def _parse_planning_problem_solution(cls, vehicle_id: str, cost_id: str,
                                         trajectory_node: et.Element) -> PlanningProblemSolution:
        """ Parses PlanningProblemSolution from the given XML node. """
        vehicle_model, vehicle_type = cls._parse_vehicle_id(vehicle_id)
        cost_function = CostFunction[cost_id]
        pp_id, trajectory = cls._parse_trajectory(trajectory_node)
        return PlanningProblemSolution(pp_id, vehicle_model, vehicle_type, cost_function, trajectory)

    @classmethod
    def _parse_trajectory(cls, trajectory_node: et.Element) -> Tuple[int, Trajectory]:
        """ Parses Trajectory and planning problem id from the given XML node. """
        trajectory_type = TrajectoryType(trajectory_node.tag)
        planning_problem_id = int(trajectory_node.get('planningProblem'))
        state_list = [cls._parse_state(trajectory_type.state_type, state_node) for state_node in trajectory_node]
        state_list = sorted(state_list, key=lambda state: state.time_step)
        return planning_problem_id, Trajectory(initial_time_step=state_list[0].time_step, state_list=state_list)

    @classmethod
    def _parse_sub_element(cls, state_node: et.Element, name: str, as_float: bool = True) -> Union[float, int]:
        """ Parses the sub elements from the given XML node. """
        elem = state_node.find(name)
        if elem is None:
            raise Exception("Element '%s' couldn't be found in the xml node!" % name)
        value = float(elem.text) if as_float else int(elem.text)
        return value

    @classmethod
    def _parse_state(cls, state_type: StateType, state_node: et.Element) -> State:
        """ Parses State from the given XML node. """
        if not state_node.tag == state_type.value:
            raise Exception("Given xml node is not a '%s' node!" % state_type.value)

        state_vals = {}
        for mapping in list(zip(state_type.xml_fields, state_type.fields)):
            xml_name = mapping[0]
            field_name = mapping[1]
            if isinstance(xml_name, tuple):
                state_vals[field_name] = np.array([cls._parse_sub_element(state_node, name) for name in xml_name])
            else:
                state_vals[field_name] = cls._parse_sub_element(state_node, xml_name, as_float=(not xml_name == 'time'))
        return State(**state_vals)

    @staticmethod
    def _parse_benchmark_id(benchmark_id: str) -> (List[str], List[str], str, str):
        """ Parses the given benchmark id string. """
        segments = benchmark_id.replace(' ', '').split(':')

        if len(segments) != 4:
            raise Exception("Invalid Benchmark ID: " + benchmark_id)

        vehicle_model_ids = re.sub(r'[\[\]]', '', segments[0]).split(',')
        cost_function_ids = re.sub(r'[\[\]]', '', segments[1]).split(',')
        scenario_id = segments[2]
        version = segments[3]

        return vehicle_model_ids, cost_function_ids, scenario_id, version

    @staticmethod
    def _parse_vehicle_id(vehicle_id: str) -> Tuple[VehicleModel, VehicleType]:
        """ Parses the given vehicle id string. """
        if not len(vehicle_id) == 3:
            raise Exception("Invalid Vehicle ID: " + vehicle_id)

        return VehicleModel[vehicle_id[:2]], VehicleType(int(vehicle_id[2]))


class CommonRoadSolutionWriter:
    # TODO Add parser check
    # TODO Prepare proper error messages to show to the user.

    def __init__(self, solution: Solution):
        """
        Creates the xml file for the given solution that can be dumped as string, or written to file later on.

        :param solution: Solution.
        """
        self.solution = solution
        self._solution_root = self._serialize_solution(self.solution)

    @staticmethod
    def _get_processor_name() -> Tuple[str, None]:
        # TODO: compare cpu name with list also used on the web server

        delete_from_cpu_name = ['(R)', '(TM)']

        def strip_substrings(string: str):
            for del_str in delete_from_cpu_name:
                string = string.replace(del_str, '')
            return string

        if platform.system() == "Windows":
            name_tmp = platform.processor()
            for del_str in delete_from_cpu_name:
                name_tmp.replace(del_str, '')
            return strip_substrings(name_tmp)
        elif platform.system() == "Darwin":
            os.environ['PATH'] = os.environ['PATH'] + os.pathsep + '/usr/sbin'
            command = "sysctl -n machdep.cpu.brand_string"
            return subprocess.check_output(command).strip()
        elif platform.system() == "Linux":
            command = "cat /proc/cpuinfo"
            all_info = str(subprocess.check_output(command, shell=True).strip())
            for line in all_info.split("\\n"):
                if "model name" in line:
                    name_tmp = re.sub(".*model name.*: ", "", line, 1)
                    return strip_substrings(name_tmp)
        return None

    @classmethod
    def _serialize_solution(cls, solution: Solution) -> et.Element:
        """ Serializes the given solution. """
        root_node = cls._create_root_node(solution)
        for pp_solution in solution.planning_problem_solutions:
            trajectory_node = cls._create_trajectory_node(pp_solution.trajectory_type,
                                                          pp_solution.planning_problem_id,
                                                          pp_solution.trajectory)
            root_node.append(trajectory_node)
        return root_node

    @classmethod
    def _create_root_node(cls, solution: Solution) -> et.Element:
        """ Creates the root node of the Solution XML. """
        root_node = et.Element('CommonRoadSolution')
        root_node.set('benchmark_id', solution.benchmark_id)
        if solution.date is not None: root_node.set('date', solution.date.strftime('%Y-%m-%d'))
        if solution.computation_time is not None: root_node.set('computation_time', str(solution.computation_time))
        processor_name = cls._get_processor_name() if solution.processor_name == 'auto' else solution.processor_name
        if processor_name is not None: root_node.set('processor_name', processor_name)
        return root_node

    @classmethod
    def _create_trajectory_node(cls, trajectory_type: TrajectoryType, pp_id: int, trajectory: Trajectory) -> et.Element:
        """ Creates the Trajectory XML Node for the given trajectory. """
        trajectory_node = et.Element(trajectory_type.value)
        trajectory_node.set('planningProblem', str(pp_id))
        for state in trajectory.state_list:
            state_node = cls._create_state_node(trajectory_type.state_type, state)
            trajectory_node.append(state_node)
        return trajectory_node

    @classmethod
    def _create_sub_element(cls, name: str, value: Union[float, int]) -> et.Element:
        """ Creates an XML element for the given value. """
        element = et.Element(name)
        element.text = str(np.float64(value) if isinstance(value, float) else value)
        return element

    @classmethod
    def _create_state_node(cls, state_type: StateType, state: State) -> et.Element:
        """ Creates XML nodes for the States of the Trajectory. """
        state_node = et.Element(state_type.value)
        for mapping in list(zip(state_type.xml_fields, state_type.fields)):
            xml_name = mapping[0]
            state_val = getattr(state, mapping[1])
            if isinstance(xml_name, tuple):
                for idx, name in enumerate(xml_name):
                    state_node.append(cls._create_sub_element(name, state_val[idx]))
            else:
                state_node.append(cls._create_sub_element(xml_name, state_val))
        return state_node

    def dump(self, pretty: bool = True) -> str:
        """
        Dumps the Solution XML as string.

        :param pretty: If set to true, prettifies the xml string.
        :return: string - Solution XML as string.
        """
        rough_string = et.tostring(self._solution_root, encoding='utf-8')
        if not pretty: return rough_string
        parsed = minidom.parseString(rough_string)
        return parsed.toprettyxml(indent="  ")

    def write_to_file(self, output_path: str = './', filename: str = None,
                      overwrite: bool = False, pretty: bool = True):
        """
        Writes the Solution XML to a file.

        :param output_path: Output dir where the Solution XML file should be written to. \
            Writes to the same folder where it is called from if not specified.
        :param filename: Name of the Solution XML file. If not specified, sets the name as 'solution_BENCHMARKID.xml' \
            where the BENCHMARKID is the benchmark_id of the solution.
        :param overwrite: If set to True, overwrites the file if it already exists.
        :param pretty: If set to True, prettifies the Solution XML string before writing to file.
        """
        filename = filename if filename is not None else 'solution_%s.xml' % self.solution.benchmark_id
        fullpath = os.path.join(output_path, filename) if filename is not None else os.path.join(output_path, filename)

        if not os.path.exists(os.path.dirname(fullpath)):
            raise NotADirectoryError("Directory %s does not exist." % os.path.dirname(fullpath))

        if os.path.exists(fullpath) and not overwrite:
            warnings.warn("File %s already exists. If you want to overwrite it set overwrite=True." % fullpath)

        else:
            with open(fullpath, 'w') as f:
                f.write(self.dump(pretty))
