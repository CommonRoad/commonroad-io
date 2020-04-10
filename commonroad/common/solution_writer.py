"""
Solution writer for uploading motion plans to commonroad.in.tum.de
"""
import xml.etree.ElementTree as et
from xml.dom import minidom
import numpy as np
from typing import List, Union
import warnings
import datetime
import copy

from commonroad import SCENARIO_VERSION
from lxml import etree, objectify

from commonroad.common.solution import VehicleType, CostFunction, VehicleModel
from commonroad.geometry.transform import rotate_translate
from commonroad.scenario.trajectory import Trajectory

__author__ = "Christina Miller"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["BMW CAR@TUM"]
__version__ = "2020.2"
__maintainer__ = "Moritz Klischat"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "Released"

import os, platform, subprocess, re

delete_from_cpu_name = ['(R)', '(TM)']


class CommonRoadSolutionWriter:
    def __init__(self, output_dir: str, scenario_id: str, step_size: float,
                 vehicle_type: VehicleType = VehicleType.FORD_ESCORT, vehicle_model: VehicleModel = VehicleModel.KS,
                 cost_function: CostFunction = CostFunction.JB1, computation_time: float = None,
                 processor_name: Union[str, None] = 'auto'):
        """
        Write solution xml files to upload at commonroad.in.tum.de.

        Note: the given IDs and cost functions are not checked for existence.

        :param output_dir: directory in which the xml solution is created.
        :param scenario_id: Id of the scenario that is solved
        :param step_size: Used for conversion of time_step to time
        :param vehicle_type: Vehicle that is used for evaluating the trajectory (also used for benchmark ID)
        :param vehicle_model: The states that are necessary for this vehicle model are written to the xml file
        :param cost_function: this cost function is written to the benchmark ID. Note: not checked for existence here.
        :param computation_time: computation time required to obtain solution (optional)
        :param processor_name: name of processor time required to obtain solution (optional). Determined automatically
        if set to 'auto'.
        """
        warnings.warn(
            'Module commonroad.common.solution_writer is deprecated. Use commonroad.common.solution module instead',
            DeprecationWarning
        )

        if cost_function not in CostFunction:
            warnings.warn('Cost function not listed. May cannot be evaluated.')

        self.step_size = step_size
        self.vehicle_model = vehicle_model
        self.benchmark_id = self._create_benchmark_id(scenario_id, vehicle_type, vehicle_model, cost_function)
        self.output_path = os.path.join(output_dir, 'solution_' + self.benchmark_id + '.xml')
        self.root_node = et.Element('CommonRoadSolution')

        if processor_name == 'auto':
            processor_name = self._get_processor_name()
        self._write_header(computation_time, processor_name)

    def add_solution_trajectory(self, trajectory: Trajectory, planning_problem_id: int):
        """
        Add a trajectory to the xml tree.

        :param trajectory: Trajectory with states for the given vehicle_model.
        :param planning_problem_id: Id of the planning problem that is solved with the trajectory.
        :return: Error, if not enough states for the given vehicle model
        """

        trajectory_node = None
        if self.vehicle_model == VehicleModel.PM:
            trajectory_node = PMTrajectoryXMLNode.create_node(trajectory)
        elif self.vehicle_model == VehicleModel.ST:
            trajectory_node = STTrajectoryXMLNode.create_node(trajectory)
        elif self.vehicle_model == VehicleModel.KS:
            trajectory_node = KSTrajectoryXMLNode.create_node(trajectory)
        elif self.vehicle_model == VehicleModel.MB:
            trajectory_node = MBTrajectoryXMLNode.create_node(trajectory)
        else:
            ValueError('Invalid vehicle model.')
        trajectory_node.set('planningProblem', str(planning_problem_id))
        self.root_node.append(trajectory_node)

    def add_solution_input_vector(self, input_vector: Trajectory, planning_problem_id: int):
        """
        Add an input vector to the xml tree

        Note: the entries of the input vector is not checked. It is assumed to contain the correct states in correct
        order:
        For PointMass Model: x_acceleration, y_acceleration, time (not time_step!).
        For Other Models: acceleration, steering_angle_speed, time (not time_step!).
        The cost function is not checked for existence.

        :param input_vector: list of states, each state is a list of the three above named values
        :param planning_problem_id: Id of the planning problem that is solved with the trajectory.
        """
        input_vector_node = None
        if self.vehicle_model == VehicleModel.PM:
            input_vector_node = PMInputVectorXMLNode.create_node(input_vector)
        elif self.vehicle_model in [VehicleModel.ST, VehicleModel.KS, VehicleModel.MB]:
            input_vector_node = InputVectorXMLNode.create_node(input_vector)
        else:
            ValueError('Invlaid vehicle model.')

        input_vector_node.set('planningProblem', str(planning_problem_id))
        self.root_node.append(input_vector_node)

    def _write_header(self, computation_time: float, processor_name: Union[str, None]):
        self.root_node.set('benchmark_id', self.benchmark_id)
        self.root_node.set('date', datetime.datetime.today().strftime('%Y-%m-%d'))
        if computation_time is not None:
            self.root_node.set('computation_time', "{:.4f}".format(computation_time))

        if isinstance(processor_name, str):
            self.root_node.set('processor_name', processor_name)

    def write_to_file(self, overwrite: bool = False):
        """
        Write xml file to ouput_dir

        :param overwrite: if true, existing files are overwritten, else not
        """
        if not os.path.exists(os.path.dirname(self.output_path)):
            NotADirectoryError("Directory {} does not exist.".format(os.path.dirname(self.output_path)))
        if os.path.exists(self.output_path) and not overwrite:
            warnings.warn("File {} already exists. If you want to overwrite it set overwrite=True."
                          .format(self.output_path))
        else:
            with open(self.output_path, 'w') as f:
                f.write(self._dump())

    def check_validity_of_solution_file(self):
        """Check the validity of a generated xml_string with the CommonRoadSolution_schema.xsd schema.
        Throw an error if it is not valid.
        """
        with open(
                os.path.dirname(os.path.abspath(__file__)) + '/CommonRoadSolution_schema.xsd',
                'rb',
        ) as schema_file:
            schema = etree.XMLSchema(etree.parse(schema_file))

        parser = objectify.makeparser(schema=schema, encoding='utf-8')

        try:
            etree.fromstring(et.tostring(self.root_node, encoding='utf-8'), parser)
        except etree.XMLSyntaxError as error:
            raise Exception(
                'Could not produce valid CommonRoadSolution file! Error: {}'.format(error.msg)
            )

    def _dump(self):
        # rough_string = etree.tostring(
        #     self.root_node, pretty_print=True, encoding='unicode'
        # )
        # return rough_string

        rough_string = et.tostring(self.root_node, encoding='utf-8')
        parsed = minidom.parseString(rough_string)
        return parsed.toprettyxml(indent="  ")

    @staticmethod
    def _create_benchmark_id(scenario_id: str, vehicle_type: VehicleType, vehicle_model: VehicleModel,
                             cost_function: CostFunction) -> str:
        return '{0}{1}:{2}:{3}:{4}'.format(vehicle_model.name, vehicle_type.value, cost_function.name, scenario_id,
                                           SCENARIO_VERSION)

    @staticmethod
    def _get_processor_name():
        # TODO: compare cpu name with list also used on the web server
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


class PMTrajectoryXMLNode:
    @classmethod
    def create_node(cls, trajectory: Trajectory) -> et.Element:
        try:
            mandatory_fields = ['position', 'velocity', 'velocity_y', 'time_step']
            is_valid_trajectory(trajectory, mandatory_fields)
        except AssertionError:
            mandatory_fields = ['position', 'velocity', 'orientation', 'time_step']
            is_valid_trajectory(trajectory, mandatory_fields)
            trajectory = split_velocity_to_xy(trajectory)

        trajectory_node = et.Element('pmTrajectory')
        for state in trajectory.state_list:
            state_node = et.SubElement(trajectory_node, 'pmState')

            x_node = et.SubElement(state_node, 'x')
            x_node.text = str(np.float64(state.position[0]))
            y_node = et.SubElement(state_node, 'y')
            y_node.text = str(np.float64(state.position[1]))
            x_vel_node = et.SubElement(state_node, 'xVelocity')
            x_vel_node.text = str(np.float64(state.velocity))
            y_vel_node = et.SubElement(state_node, 'yVelocity')
            y_vel_node.text = str(np.float64(state.velocity_y))
            time_node = et.SubElement(state_node, 'time')
            time_node.text = str(state.time_step)
        return trajectory_node


def split_velocity_to_xy(trajectory: Trajectory) -> Trajectory:
    """Converts trajectory from [v,orientation] ot [v_x,v_y]"""
    trajectory = copy.deepcopy(trajectory)

    for state in trajectory.state_list:
        v_temp = np.array([[state.velocity, 0.0]])
        v_temp = rotate_translate(v_temp, np.array([0.0, 0.0]), state.orientation)
        state.velocity = v_temp[0, 0]
        state.velocity_y = v_temp[0, 1]

    return trajectory


class STTrajectoryXMLNode:
    @classmethod
    def create_node(cls, trajectory: Trajectory) -> et.Element:
        mandatory_fields = ['position', 'orientation', 'yaw_rate', 'velocity', 'steering_angle', 'slip_angle',
                            'time_step']
        is_valid_trajectory(trajectory, mandatory_fields)

        trajectory_node = et.Element('stTrajectory')
        for state in trajectory.state_list:
            state_node = et.SubElement(trajectory_node, 'stState')

            x_node = et.SubElement(state_node, 'x')
            x_node.text = str(np.float64(state.position[0]))
            y_node = et.SubElement(state_node, 'y')
            y_node.text = str(np.float64(state.position[1]))
            orientation_node = et.SubElement(state_node, 'orientation')
            orientation_node.text = str(np.float64(state.orientation))
            yaw_rate_node = et.SubElement(state_node, 'yawRate')
            yaw_rate_node.text = str(np.float64(state.yaw_rate))
            x_vel_node = et.SubElement(state_node, 'velocity')
            x_vel_node.text = str(np.float64(state.velocity))
            steering_angle_node = et.SubElement(state_node, 'steeringAngle')
            steering_angle_node.text = str(np.float64(state.steering_angle))
            slip_angle_node = et.SubElement(state_node, 'slipAngle')
            slip_angle_node.text = str(np.float64(state.slip_angle))
            time_node = et.SubElement(state_node, 'time')
            time_node.text = str(state.time_step)
        return trajectory_node


class KSTrajectoryXMLNode:
    @classmethod
    def create_node(cls, trajectory: Trajectory) -> et.Element:
        mandatory_fields = ['position', 'orientation', 'velocity', 'steering_angle', 'time_step']
        is_valid_trajectory(trajectory, mandatory_fields)

        trajectory_node = et.Element('ksTrajectory')
        for state in trajectory.state_list:
            state_node = et.SubElement(trajectory_node, 'ksState')

            x_node = et.SubElement(state_node, 'x')
            x_node.text = str(np.float64(state.position[0]))
            y_node = et.SubElement(state_node, 'y')
            y_node.text = str(np.float64(state.position[1]))
            orientation_node = et.SubElement(state_node, 'orientation')
            orientation_node.text = str(np.float64(state.orientation))
            x_vel_node = et.SubElement(state_node, 'velocity')
            x_vel_node.text = str(np.float64(state.velocity))
            steering_angle_node = et.SubElement(state_node, 'steeringAngle')
            steering_angle_node.text = str(np.float64(state.steering_angle))
            time_node = et.SubElement(state_node, 'time')
            time_node.text = str(state.time_step)
        return trajectory_node


class MBTrajectoryXMLNode:
    @classmethod
    def create_node(cls, trajectory: Trajectory) -> et.Element:
        mandatory_fields = ['position', 'orientation', 'yawRate', 'velocity', 'steering_angle', 'slip_angle',
                            'time_step',
                            'roll_angle', 'roll_rate', 'pitch_angle', 'pitch_rate', 'y_velocity', 'z_position',
                            'z_velocity',
                            'roll_angle_front', 'roll_rate_front', 'y_velocity_front', 'z_position_front',
                            'z_velocity_front',
                            'roll_angle_rear', 'roll_rate_rear', 'y_velocity_rear', 'z_position_rear',
                            'z_velocity_rear',
                            'left_front_wheel_angular_speed', 'right_front_wheel_angular_speed',
                            'left_rear_wheel_angular_speed',
                            'right_rear_wheel_angular_speed', 'delta_y_f', 'delta_y_r']
        is_valid_trajectory(trajectory, mandatory_fields)

        trajectory_node = et.Element('mbTrajectory')
        for state in trajectory.state_list:
            state_node = et.SubElement(trajectory_node, 'mbState')

            x_node = et.SubElement(state_node, 'x')
            x_node.text = str(np.float64(state.position[0]))
            y_node = et.SubElement(state_node, 'y')
            y_node.text = str(np.float64(state.position[1]))
            orientation_node = et.SubElement(state_node, 'orientation')
            orientation_node.text = str(np.float64(state.orientation))
            yaw_rate_node = et.SubElement(state_node, 'yawRate')
            yaw_rate_node.text = str(np.float64(state.yaw_rate))
            x_vel_node = et.SubElement(state_node, 'velocity')
            x_vel_node.text = str(np.float64(state.velocity))
            steering_angle_node = et.SubElement(state_node, 'steeringAngle')
            steering_angle_node.text = str(np.float64(state.steering_angle))
            slip_angle_node = et.SubElement(state_node, 'slipAngle')
            slip_angle_node.text = str(np.float64(state.slip_angle))
            time_node = et.SubElement(state_node, 'time')
            time_node.text = str(state.time_step)

            roll_angle_node = et.SubElement(state_node, 'rollAngle')
            roll_angle_node.text = str(state.roll_angle)
            roll_rate_node = et.SubElement(state_node, 'rollRate')
            roll_rate_node.text = str(state.roll_rate)
            pitch_angle_node = et.SubElement(state_node, 'pitchAngle')
            pitch_angle_node.text = str(state.pitch_angle)
            state_node_node = et.SubElement(state_node, 'pitchRate')
            state_node_node.text = str(state.pitch_rate)

            velocity_y_node = et.SubElement(state_node, 'yVelocity')
            velocity_y_node.text = str(state.velocity_y)
            position_z_node = et.SubElement(state_node, 'zPosition')
            position_z_node.text = str(state.position_z)
            velocity_z_node = et.SubElement(state_node, 'zVelocity')
            velocity_z_node.text = str(state.velocity_z)
            roll_angle_front_node = et.SubElement(state_node, 'rollAngleFront')
            roll_angle_front_node.text = str(state.roll_angle_front)
            roll_rate_front_node = et.SubElement(state_node, 'rollRateFront')
            roll_rate_front_node.text = str(state.roll_rate_front)
            velocity_y_front_node = et.SubElement(state_node, 'yVelocityFront')
            velocity_y_front_node.text = str(state.velocity_y_front)
            position_z_front_node = et.SubElement(state_node, 'zPositionFront')
            position_z_front_node.text = str(state.position_z_front)
            velocity_z_front_node = et.SubElement(state_node, 'zVelocityFront')
            velocity_z_front_node.text = str(state.velocity_z_front)

            roll_angle_rear_node = et.SubElement(state_node, 'rollAngleRear')
            roll_angle_rear_node.text = str(state.roll_angle_rear)
            roll_rate_rear_node = et.SubElement(state_node, 'rollRateRear')
            roll_rate_rear_node.text = str(state.roll_rate_rear)
            velocity_y_rear_node = et.SubElement(state_node, 'yVelocityRear')
            velocity_y_rear_node.text = str(state.velocity_y_rear)
            position_z_rear_node = et.SubElement(state_node, 'zPositionRear')
            position_z_rear_node.text = str(state.position_z_rear)
            velocity_z_rear_node = et.SubElement(state_node, 'zVelocityRear')
            velocity_z_rear_node.text = str(state.velocity_z_rear)

            left_front_wheel_angular_speed_node = et.SubElement(state_node, 'leftFrontWheelAngularSpeed')
            left_front_wheel_angular_speed_node.text = str(state.left_front_wheel_angular_speed)
            left_rear_wheel_angular_speed_node = et.SubElement(state_node, 'leftRearWheelAngularSpeed')
            left_rear_wheel_angular_speed_node.text = str(state.left_rear_wheel_angular_speed)
            right_front_wheel_angular_speed_node = et.SubElement(state_node, 'rightFrontWheelAngularSpeed')
            right_front_wheel_angular_speed_node.text = str(state.right_front_wheel_angular_speed)
            right_rear_wheel_angular_speed_node = et.SubElement(state_node, 'rightRearWheelAngularSpeed')
            right_rear_wheel_angular_speed_node.text = str(state.right_rear_wheel_angular_speed)
            delta_y_f_node = et.SubElement(state_node, 'deltaYf')
            delta_y_f_node.text = str(state.delta_y_f)
            delta_y_r_node = et.SubElement(state_node, 'deltaYr')
            delta_y_r_node.text = str(state.delta_y_r)
        return trajectory_node


class PMInputVectorXMLNode:
    @classmethod
    def create_node(cls, trajectory: Trajectory) -> et.Element:
        mandatory_fields = ['acceleration', 'orientation', 'time_step']
        is_valid_trajectory(trajectory, mandatory_fields)

        input_vector_node = et.Element('pmInputVector')
        for state in trajectory.state_list:
            input_node = et.SubElement(input_vector_node, 'input')
            x_acceleration_node = et.SubElement(input_node, 'xAcceleration')
            x_acceleration_node.text = str(state.acceleration * np.cos(state.orientation))
            y_acceleration_node = et.SubElement(input_node, 'yAcceleration')
            y_acceleration_node.text = str(state.acceleration * np.sin(state.orientation))
            time_node = et.SubElement(input_node, 'time')
            time_node.text = str(state.time_step)
        return input_vector_node


class InputVectorXMLNode:
    @classmethod
    def create_node(cls, trajectory: Trajectory) -> et.Element:
        mandatory_fields = ['acceleration', 'steering_angle_speed', 'time_step']
        is_valid_trajectory(trajectory, mandatory_fields)

        input_vector_node = et.Element('inputVector')
        for state in trajectory.state_list:
            input_node = et.SubElement(input_vector_node, 'input')
            acceleration_node = et.SubElement(input_node, 'acceleration')
            acceleration_node.text = str(state.acceleration)
            steering_angle_speed_node = et.SubElement(input_node, 'steeringAngleSpeed')
            steering_angle_speed_node.text = str(state.steering_angle_speed)
            time_node = et.SubElement(input_node, 'time')
            time_node.text = str(state.time_step)
        return input_vector_node


def is_valid_trajectory(trajectory: Trajectory, mandatory_fields: List):
    for state in trajectory.state_list:
        for field in mandatory_fields:
            assert (hasattr(state, field)), '<PlanningProblem/initial_state> fields [{}] are mandatory. ' \
                                            'No {} attribute found.'.format(', '.join(mandatory_fields), field)
