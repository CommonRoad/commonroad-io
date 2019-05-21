"""
Solution writer for uploading motion plans to commonroad.in.tum.de
"""
import xml.etree.ElementTree as et
from xml.dom import minidom
import numpy as np
from typing import List
from enum import Enum, unique
import os
import warnings
import datetime

from commonroad.scenario.trajectory import Trajectory
from commonroad.common.validity import is_real_number_vector

__author__ = "Christina Miller"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["BMW CAR@TUM"]
__version__ = "2019.1"
__maintainer__ = "Moritz Klischat"
__email__ = "commonroad@in.tum.de"
__status__ = "Released"


SCENARIO_VERSION = '2018b'


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


class CommonRoadSolutionWriter:
    def __init__(self, output_dir: str, scenario_id: str, step_size: float,
                 vehicle_type: VehicleType=VehicleType.FORD_ESCORT, vehicle_model: VehicleModel=VehicleModel.KS,
                 cost_function: CostFunction=CostFunction.JB1):
        """
        Write solution xml files to upload at commonroad.in.tum.de.

        Note: the given IDs and cost functions are not checked for existence.

        :param output_dir: directory in which the xml solution is created.
        :param scenario_id: Id of the scenario that is solved
        :param step_size: Used for conversion of time_step to time
        :param vehicle_type: Vehicle that is used for evaluating the trajectory (also used for benchmark ID)
        :param vehicle_model: The states that are necessary for this vehicle model are written to the xml file
        :param cost_function: this cost function is written to the benchmark ID. Note: not checked for existence here.
        """

        if cost_function not in CostFunction:
            warnings.warn('Cost function not listed. May cannot be evaluated.')

        self.step_size = step_size
        self.vehicle_model = vehicle_model
        self.benchmark_id = self._create_benchmark_id(scenario_id, vehicle_type, vehicle_model, cost_function)
        self.output_path = os.path.join(output_dir, 'solution_' + self.benchmark_id + '.xml')
        self.root_node = et.Element('CommonRoadSolution')
        self._write_header()

    def add_solution_trajectory(self,  trajectory: Trajectory, planning_problem_id: int):
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

    def add_solution_input_vector(self, input_vector: np.ndarray, planning_problem_id: int):
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
        assert all([is_real_number_vector(input_vector_i,length=3) for input_vector_i in input_vector]),\
            '<CommonRoadSolutionWriter/add_solution_input_vector>: input_vector has to be numpy vector of real numbers with shape=[n,3]'

        input_vector_node = None
        if self.vehicle_model == VehicleModel.PM:
            input_vector_node = PMInputVectorXMLNode.create_node(input_vector)
        elif self.vehicle_model in [VehicleModel.ST, VehicleModel.KS, VehicleModel.MB]:
            input_vector_node = InputVectorXMLNode.create_node(input_vector)
        else:
            ValueError('Invlaid vehicle model.')

        input_vector_node.set('planningProblem', str(planning_problem_id))
        self.root_node.append(input_vector_node)

    def _write_header(self):
        self.root_node.set('benchmark_id', self.benchmark_id)
        self.root_node.set('date', datetime.datetime.today().strftime('%Y-%m-%d'))

    def write_to_file(self, overwrite: bool=False):
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
                f.write(self._dump(self.root_node))

    @staticmethod
    def _dump(root_node):
        rough_string = et.tostring(root_node, encoding='utf-8')
        parsed = minidom.parseString(rough_string)
        return parsed.toprettyxml(indent="  ")

    @staticmethod
    def _create_benchmark_id(scenario_id: str, vehicle_type: VehicleType, vehicle_model: VehicleModel,
                             cost_function: CostFunction) -> str:
        return '{0}{1}:{2}:{3}:{4}'.format(vehicle_model.name, vehicle_type.value, cost_function.name, scenario_id,
                                           SCENARIO_VERSION)


class PMTrajectoryXMLNode:
    @classmethod
    def create_node(cls, trajectory: Trajectory) -> et.Element:
        mandatory_fields = ['position', 'velocity', 'velocity_y', 'time_step']
        is_valid_trajectory(trajectory, mandatory_fields)

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
            time_node.text = str(state.time_step * step_size)

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
    def create_node(cls, input_vector: np.ndarray) -> et.Element:
        assert (input_vector.shape[1]==3), '<PMInputVectorXMLNode/create_node>: input_vector contains' \
                                                          'lists of length 3: xAcceleration: float, yAcceleration: ' \
                                                          'float, time: float.'
        input_node = et.Element('pmInputVector')
        for state in input_vector:
            x_acceleration_node = et.SubElement(input_node, 'xAcceleration')
            x_acceleration_node.text = str(state[0])
            y_acceleration_node = et.SubElement(input_node, 'yAcceleration')
            y_acceleration_node.text = str(state[1])
            time_node = et.SubElement(input_node, 'time')
            time_node.text = str(state[2])
        return input_node


class InputVectorXMLNode:
    @classmethod
    def create_node(cls, input_vector: np.ndarray) -> et.Element:
        assert (input_vector.shape[1]==3), '<InputVectorXMLNode/create_node>: input_vector contains' \
                                                           'lists of length 3: acceleration: float, ' \
                                                           'steeringAngleSpeed: float, time: float.'
        input_node = et.Element('inputVector')
        for state in input_vector:
            acceleration_node = et.SubElement(input_node, 'acceleration')
            acceleration_node.text = str(state[0])
            steering_angle_speed_node = et.SubElement(input_node, 'steeringAngleSpeed')
            steering_angle_speed_node.text = str(state[1])
            time_node = et.SubElement(input_node, 'time')
            time_node.text = str(state[2])
        return input_node


def is_valid_trajectory(trajectory: Trajectory, mandatory_fields: List):
    for state in trajectory.state_list:
        for field in mandatory_fields:
            assert(hasattr(state, field)), '<PlanningProblem/initial_state> fields [{}] are mandatory. ' \
                                           'No {} attribute found.'.format(', '.join(mandatory_fields), field)
