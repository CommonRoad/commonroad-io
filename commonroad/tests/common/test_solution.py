import os
import random
import re
import unittest
import warnings
from glob import glob

import numpy as np
from commonroad.common.solution import StateFields, XMLStateFields, StateType, TrajectoryType, PlanningProblemSolution, \
    Solution, CommonRoadSolutionWriter, CommonRoadSolutionReader, VehicleModel, VehicleType, CostFunction
from commonroad.scenario.scenario import ScenarioID
from commonroad.scenario.trajectory import State, Trajectory


class DummyDataGenerator:

    @staticmethod
    def create_random_float(lower, upper):
        return random.uniform(lower, upper)

    @staticmethod
    def create_random_int(lower, upper):
        return random.randint(lower, upper)

    @classmethod
    def create_random_pm_state(cls, time_step=0):
        return State(
            position=np.array([cls.create_random_float(-100, 100),
                               cls.create_random_float(-100, 100)]),
            velocity=cls.create_random_float(-5, 5),
            velocity_y=cls.create_random_float(-5, 5),
            time_step=time_step
        )

    @classmethod
    def create_random_ks_state(cls, time_step=0):
        return State(
            position=np.array([cls.create_random_float(-100, 100),
                               cls.create_random_float(-100, 100)]),
            steering_angle=cls.create_random_float(-np.math.pi / 10, np.math.pi / 10),
            velocity=cls.create_random_float(-5, 5),
            orientation=cls.create_random_float(-np.math.pi, np.math.pi),
            time_step=time_step
        )

    @classmethod
    def create_random_st_state(cls, time_step=0):
        return State(
            position=np.array([cls.create_random_float(-100, 100),
                               cls.create_random_float(-100, 100)]),
            steering_angle=cls.create_random_float(-np.math.pi / 10, np.math.pi / 10),
            velocity=cls.create_random_float(-5, 5),
            orientation=cls.create_random_float(-np.math.pi, np.math.pi),
            yaw_rate=cls.create_random_float(-np.math.pi / 10, np.math.pi / 10),
            slip_angle=cls.create_random_float(-np.math.pi / 10, np.math.pi / 10),
            time_step=time_step
        )

    @classmethod
    def create_random_mb_state(cls, time_step=0):
        return State(
            position=np.array([cls.create_random_float(-100, 100),
                               cls.create_random_float(-100, 100)]),
            steering_angle=cls.create_random_float(-np.math.pi / 10, np.math.pi / 10),
            velocity=cls.create_random_float(-5, 5),
            orientation=cls.create_random_float(-np.math.pi, np.math.pi),
            yaw_rate=cls.create_random_float(-np.math.pi / 10, np.math.pi / 10),
            roll_angle=cls.create_random_float(-np.math.pi / 10, np.math.pi / 10),
            roll_rate=cls.create_random_float(-np.math.pi / 10, np.math.pi / 10),
            pitch_angle=cls.create_random_float(-np.math.pi / 10, np.math.pi / 10),
            pitch_rate=cls.create_random_float(-np.math.pi / 10, np.math.pi / 10),
            velocity_y=cls.create_random_float(-5, 5),
            position_z=cls.create_random_float(-10, 10),
            velocity_z=cls.create_random_float(-5, 5),
            roll_angle_front=cls.create_random_float(-np.math.pi / 10, np.math.pi / 10),
            roll_rate_front=cls.create_random_float(-np.math.pi / 10, np.math.pi / 10),
            velocity_y_front=cls.create_random_float(-5, 5),
            position_z_front=cls.create_random_float(-10, 10),
            velocity_z_front=cls.create_random_float(-5, 5),
            roll_angle_rear=cls.create_random_float(-np.math.pi / 10, np.math.pi / 10),
            roll_rate_rear=cls.create_random_float(-np.math.pi / 10, np.math.pi / 10),
            velocity_y_rear=cls.create_random_float(-5, 5),
            position_z_rear=cls.create_random_float(-10, 10),
            velocity_z_rear=cls.create_random_float(-5, 5),
            left_front_wheel_angular_speed=cls.create_random_float(-5, 5),
            right_front_wheel_angular_speed=cls.create_random_float(-5, 5),
            left_rear_wheel_angular_speed=cls.create_random_float(-5, 5),
            right_rear_wheel_angular_speed=cls.create_random_float(-5, 5),
            delta_y_f=cls.create_random_float(-10, 10),
            delta_y_r=cls.create_random_float(-10, 10),
            time_step=time_step
        )

    @classmethod
    def create_random_input(cls, time_step=0):
        return State(
            acceleration=cls.create_random_float(-5, 5),
            steering_angle_speed=cls.create_random_float(-np.math.pi / 10, np.math.pi / 10),
            time_step=time_step
        )

    @classmethod
    def create_random_pm_input(cls, time_step=0):
        return State(
            acceleration=cls.create_random_float(-5, 5),
            acceleration_y=cls.create_random_float(-5, 5),
            time_step=time_step
        )

    @classmethod
    def create_random_pm_trajectory(cls, state_count=5):
        return Trajectory(initial_time_step=0, state_list=[cls.create_random_pm_state(ts) for ts in range(state_count)])

    @classmethod
    def create_random_ks_trajectory(cls, state_count=5):
        return Trajectory(initial_time_step=0, state_list=[cls.create_random_ks_state(ts) for ts in range(state_count)])

    @classmethod
    def create_random_st_trajectory(cls, state_count=5):
        return Trajectory(initial_time_step=0, state_list=[cls.create_random_st_state(ts) for ts in range(state_count)])

    @classmethod
    def create_random_mb_trajectory(cls, state_count=5):
        return Trajectory(initial_time_step=0, state_list=[cls.create_random_mb_state(ts) for ts in range(state_count)])

    @classmethod
    def create_random_input_vector(cls, input_count=5):
        return Trajectory(initial_time_step=0, state_list=[cls.create_random_input(ts) for ts in range(input_count)])

    @classmethod
    def create_random_pm_input_vector(cls, input_count=5):
        return Trajectory(initial_time_step=0, state_list=[cls.create_random_pm_input(ts) for ts in range(input_count)])

    @classmethod
    def create_pm_state_xml(cls, state: State):
        return '''
        <pmState>
            <x>%s</x>
            <y>%s</y>
            <xVelocity>%s</xVelocity>
            <yVelocity>%s</yVelocity>
            <time>%s</time>
        </pmState>
        ''' % (
            str(state.position[0]),
            str(state.position[1]),
            str(state.velocity),
            str(state.velocity_y),
            str(state.time_step)
        )

    @classmethod
    def create_ks_state_xml(cls, state: State):
        return '''
        <ksState>
            <x>%s</x>
            <y>%s</y>
            <steeringAngle>%s</steeringAngle>
            <velocity>%s</velocity>
            <orientation>%s</orientation>
            <time>%s</time>
        </ksState>
        ''' % (
            str(state.position[0]),
            str(state.position[1]),
            str(state.steering_angle),
            str(state.velocity),
            str(state.orientation),
            str(state.time_step)
        )

    @classmethod
    def create_st_state_xml(cls, state: State):
        return '''
        <stState>
            <x>%s</x>
            <y>%s</y>
            <steeringAngle>%s</steeringAngle>
            <velocity>%s</velocity>
            <orientation>%s</orientation>
            <yawRate>%s</yawRate>
            <slipAngle>%s</slipAngle>
            <time>%s</time>
        </stState>
        ''' % (
            str(state.position[0]),
            str(state.position[1]),
            str(state.steering_angle),
            str(state.velocity),
            str(state.orientation),
            str(state.yaw_rate),
            str(state.slip_angle),
            str(state.time_step)
        )

    @classmethod
    def create_mb_state_xml(cls, state: State):
        return '''
        <mbState>
            <x>%s</x>
            <y>%s</y>
            <steeringAngle>%s</steeringAngle>
            <velocity>%s</velocity>
            <orientation>%s</orientation>
            <yawRate>%s</yawRate>
            <rollAngle>%s</rollAngle>
            <rollRate>%s</rollRate>
            <pitchAngle>%s</pitchAngle>
            <pitchRate>%s</pitchRate>
            <yVelocity>%s</yVelocity>
            <zPosition>%s</zPosition>
            <zVelocity>%s</zVelocity>
            <rollAngleFront>%s</rollAngleFront>
            <rollRateFront>%s</rollRateFront>
            <yVelocityFront>%s</yVelocityFront>
            <zPositionFront>%s</zPositionFront>
            <zVelocityFront>%s</zVelocityFront>
            <rollAngleRear>%s</rollAngleRear>
            <rollRateRear>%s</rollRateRear>
            <yVelocityRear>%s</yVelocityRear>
            <zPositionRear>%s</zPositionRear>
            <zVelocityRear>%s</zVelocityRear>
            <leftFrontWheelAngularSpeed>%s</leftFrontWheelAngularSpeed>
            <rightFrontWheelAngularSpeed>%s</rightFrontWheelAngularSpeed>
            <leftRearWheelAngularSpeed>%s</leftRearWheelAngularSpeed>
            <rightRearWheelAngularSpeed>%s</rightRearWheelAngularSpeed>
            <deltaYf>%s</deltaYf>
            <deltaYr>%s</deltaYr>
            <time>%s</time>
        </mbState>
        ''' % (
            str(state.position[0]),
            str(state.position[1]),
            str(state.steering_angle),
            str(state.velocity),
            str(state.orientation),
            str(state.yaw_rate),
            str(state.roll_angle),
            str(state.roll_rate),
            str(state.pitch_angle),
            str(state.pitch_rate),
            str(state.velocity_y),
            str(state.position_z),
            str(state.velocity_z),
            str(state.roll_angle_front),
            str(state.roll_rate_front),
            str(state.velocity_y_front),
            str(state.position_z_front),
            str(state.velocity_z_front),
            str(state.roll_angle_rear),
            str(state.roll_rate_rear),
            str(state.velocity_y_rear),
            str(state.position_z_rear),
            str(state.velocity_z_rear),
            str(state.left_front_wheel_angular_speed),
            str(state.right_front_wheel_angular_speed),
            str(state.left_rear_wheel_angular_speed),
            str(state.right_rear_wheel_angular_speed),
            str(state.delta_y_f),
            str(state.delta_y_r),
            str(state.time_step)
        )

    @classmethod
    def create_input_xml(cls, state: State):
        return '''
        <input>
            <steeringAngleSpeed>%s</steeringAngleSpeed>
            <acceleration>%s</acceleration>
            <time>%s</time>
        </input>
        ''' % (
            str(state.steering_angle_speed),
            str(state.acceleration),
            str(state.time_step)
        )

    @classmethod
    def create_pm_input_xml(cls, state: State):
        return '''
        <pmInput>
            <xAcceleration>%s</xAcceleration>
            <yAcceleration>%s</yAcceleration>
            <time>%s</time>
        </pmInput>
        ''' % (
            str(state.acceleration),
            str(state.acceleration_y),
            str(state.time_step)
        )

    @classmethod
    def create_trajectory_xml(cls, trajectory_type: str, planning_problem_id: int, trajectory: Trajectory):
        state_serializer = None
        if trajectory_type == 'pmTrajectory': state_serializer = cls.create_pm_state_xml
        if trajectory_type == 'stTrajectory': state_serializer = cls.create_st_state_xml
        if trajectory_type == 'ksTrajectory': state_serializer = cls.create_ks_state_xml
        if trajectory_type == 'mbTrajectory': state_serializer = cls.create_mb_state_xml
        if trajectory_type == 'inputVector': state_serializer = cls.create_input_xml
        if trajectory_type == 'pmInputVector': state_serializer = cls.create_pm_input_xml
        return '''
        <%s planningProblem="%s">
            %s
        </%s>
        ''' % (
            trajectory_type,
            str(planning_problem_id),
            ''.join([state_serializer(state) for state in trajectory.state_list]),
            trajectory_type
        )

    @classmethod
    def create_solution_xml(cls, solution: Solution):
        benchmark_id = solution.benchmark_id
        date_str = 'date="%s"' % solution.date.strftime('%Y-%m-%d')
        processor_str = '' if solution.processor_name is None else 'processor_name="%s"' % solution.processor_name
        computation_str = '' if solution.computation_time is None else 'computation_time="%s"' % str(
            solution.computation_time)
        trajectory_xmls = [
            cls.create_trajectory_xml(pp_sol.trajectory_type.value, pp_sol.planning_problem_id, pp_sol.trajectory)
            for pp_sol in solution.planning_problem_solutions
        ]
        solution_xml = '''
        <?xml version="1.0" ?>
        <CommonRoadSolution benchmark_id="%s" %s %s %s>
            %s
        </CommonRoadSolution>
        ''' % (benchmark_id,
               date_str,
               computation_str,
               processor_str,
               ''.join(trajectory_xmls))
        return solution_xml.strip()


class TestStateFields(unittest.TestCase):
    """
    These tests are for making sure the state fields are not changed accidentally, as inconsistencies on the
    state fields may break whole solution mechanism.
    """

    def test_pm_state_fields(self):
        assert StateFields.PM.value == ['position', 'velocity', 'velocity_y', 'time_step']

    def test_st_state_fields(self):
        assert StateFields.ST.value == ['position', 'steering_angle', 'velocity', 'orientation',
                                        'yaw_rate', 'slip_angle', 'time_step']

    def test_ks_state_fields(self):
        assert StateFields.KS.value == ['position', 'steering_angle', 'velocity', 'orientation', 'time_step']

    def test_mb_state_fields(self):
        assert StateFields.MB.value == ['position', 'steering_angle', 'velocity', 'orientation', 'yaw_rate',
                                        'roll_angle', 'roll_rate', 'pitch_angle', 'pitch_rate', 'velocity_y',
                                        'position_z', 'velocity_z', 'roll_angle_front', 'roll_rate_front',
                                        'velocity_y_front', 'position_z_front', 'velocity_z_front', 'roll_angle_rear',
                                        'roll_rate_rear', 'velocity_y_rear', 'position_z_rear', 'velocity_z_rear',
                                        'left_front_wheel_angular_speed', 'right_front_wheel_angular_speed',
                                        'left_rear_wheel_angular_speed', 'right_rear_wheel_angular_speed',
                                        'delta_y_f', 'delta_y_r', 'time_step']

    def test_input_fields(self):
        assert StateFields.Input.value == ['steering_angle_speed', 'acceleration', 'time_step']

    def test_pm_input_fields(self):
        assert StateFields.PMInput.value == ['acceleration', 'acceleration_y', 'time_step']


class TestXMLStateFields(unittest.TestCase):
    """
    These tests are for making sure the xml state fields are not changed accidentally, as inconsistencies on the
    state fields may break whole solution mechanism.
    """

    def test_pm_xml_state_fields(self):
        assert XMLStateFields.PM.value == [('x', 'y'), 'xVelocity', 'yVelocity', 'time']

    def test_st_xml_state_fields(self):
        assert XMLStateFields.ST.value == [('x', 'y'), 'steeringAngle', 'velocity', 'orientation', 'yawRate',
                                           'slipAngle', 'time']

    def test_ks_xml_state_fields(self):
        assert XMLStateFields.KS.value == [('x', 'y'), 'steeringAngle', 'velocity', 'orientation', 'time']

    def test_mb_xml_state_fields(self):
        assert XMLStateFields.MB.value == [('x', 'y'), 'steeringAngle', 'velocity', 'orientation', 'yawRate',
                                           'rollAngle',
                                           'rollRate', 'pitchAngle', 'pitchRate', 'yVelocity', 'zPosition', 'zVelocity',
                                           'rollAngleFront', 'rollRateFront', 'yVelocityFront', 'zPositionFront',
                                           'zVelocityFront', 'rollAngleRear', 'rollRateRear', 'yVelocityRear',
                                           'zPositionRear', 'zVelocityRear', 'leftFrontWheelAngularSpeed',
                                           'rightFrontWheelAngularSpeed', 'leftRearWheelAngularSpeed',
                                           'rightRearWheelAngularSpeed', 'deltaYf', 'deltaYr', 'time']

    def test_input_xml_fields(self):
        assert XMLStateFields.Input.value == ['steeringAngleSpeed', 'acceleration', 'time']

    def test_pm_input_xml_fields(self):
        assert XMLStateFields.PMInput.value == ['xAcceleration', 'yAcceleration', 'time']


class TestStateType(unittest.TestCase):

    @staticmethod
    def create_dummy_state(state_fields: StateFields) -> State:
        values = {field: 0 for field in state_fields.value}
        return State(**values)

    def test_pm_state_type(self):
        assert StateType.PM.value == 'pmState'
        assert StateType.PM.fields == StateFields.PM.value
        assert StateType.PM.xml_fields == XMLStateFields.PM.value
        assert StateType.PM == StateType.get_state_type(DummyDataGenerator.create_random_pm_state(),
                                                        desired_vehicle_model=VehicleModel.PM)

    def test_st_state_type(self):
        assert StateType.ST.value == 'stState'
        assert StateType.ST.fields == StateFields.ST.value
        assert StateType.ST.xml_fields == XMLStateFields.ST.value
        assert StateType.ST == StateType.get_state_type(DummyDataGenerator.create_random_st_state(),
                                                        desired_vehicle_model=VehicleModel.ST)

    def test_ks_state_type(self):
        assert StateType.KS.value == 'ksState'
        assert StateType.KS.fields == StateFields.KS.value
        assert StateType.KS.xml_fields == XMLStateFields.KS.value
        assert StateType.KS == StateType.get_state_type(DummyDataGenerator.create_random_ks_state(),
                                                        desired_vehicle_model=VehicleModel.KS)

    def test_ks_state_type_other_type(self):
        assert StateType.KS.value == 'ksState'
        assert StateType.KS.fields == StateFields.KS.value
        assert StateType.KS.xml_fields == XMLStateFields.KS.value
        assert StateType.KS == StateType.get_state_type(DummyDataGenerator.create_random_mb_state(),
                                                        desired_vehicle_model=VehicleModel.KS)

    def test_mb_state_type(self):
        assert StateType.MB.value == 'mbState'
        assert StateType.MB.fields == StateFields.MB.value
        assert StateType.MB.xml_fields == XMLStateFields.MB.value
        assert StateType.MB == StateType.get_state_type(DummyDataGenerator.create_random_mb_state(),
                                                        desired_vehicle_model=VehicleModel.MB)

    def test_input_state_type(self):
        assert StateType.Input.value == 'input'
        assert StateType.Input.fields == StateFields.Input.value
        assert StateType.Input.xml_fields == XMLStateFields.Input.value
        assert StateType.Input == StateType.get_state_type(DummyDataGenerator.create_random_input(),
                                                           desired_vehicle_model=VehicleModel.MB)
        assert StateType.Input == StateType.get_state_type(DummyDataGenerator.create_random_input(),
                                                           desired_vehicle_model=VehicleModel.KS)
        assert StateType.Input == StateType.get_state_type(DummyDataGenerator.create_random_input(),
                                                           desired_vehicle_model=VehicleModel.ST)

    def test_pm_input_state_type(self):
        assert StateType.PMInput.value == 'pmInput'
        assert StateType.PMInput.fields == StateFields.PMInput.value
        assert StateType.PMInput.xml_fields == XMLStateFields.PMInput.value
        assert StateType.PMInput == StateType.get_state_type(DummyDataGenerator.create_random_pm_input(),
                                                             desired_vehicle_model=VehicleModel.PM)


class TestTrajectoryType(unittest.TestCase):

    def test_pm_trajectory_type(self):
        dummy_trajectory = DummyDataGenerator.create_random_pm_trajectory()
        assert TrajectoryType.PM.value == 'pmTrajectory'
        assert TrajectoryType.PM.state_type == StateType.PM
        assert TrajectoryType.PM == TrajectoryType.get_trajectory_type(dummy_trajectory)
        assert TrajectoryType.PM.valid_vehicle_model(VehicleModel.PM)
        assert not TrajectoryType.PM.valid_vehicle_model(VehicleModel.ST)
        assert not TrajectoryType.PM.valid_vehicle_model(VehicleModel.KS)
        assert not TrajectoryType.PM.valid_vehicle_model(VehicleModel.MB)

    def test_st_trajectory_type(self):
        dummy_trajectory = DummyDataGenerator.create_random_st_trajectory()
        assert TrajectoryType.ST.value == 'stTrajectory'
        assert TrajectoryType.ST.state_type == StateType.ST
        assert TrajectoryType.ST == TrajectoryType.get_trajectory_type(dummy_trajectory)
        assert TrajectoryType.ST.valid_vehicle_model(VehicleModel.ST)
        assert not TrajectoryType.ST.valid_vehicle_model(VehicleModel.PM)
        assert not TrajectoryType.ST.valid_vehicle_model(VehicleModel.KS)
        assert not TrajectoryType.ST.valid_vehicle_model(VehicleModel.MB)

    def test_ks_trajectory_type(self):
        dummy_trajectory = DummyDataGenerator.create_random_ks_trajectory()
        assert TrajectoryType.KS.value == 'ksTrajectory'
        assert TrajectoryType.KS.state_type == StateType.KS
        assert TrajectoryType.KS == TrajectoryType.get_trajectory_type(dummy_trajectory)
        assert TrajectoryType.KS.valid_vehicle_model(VehicleModel.KS)
        assert not TrajectoryType.KS.valid_vehicle_model(VehicleModel.PM)
        assert not TrajectoryType.KS.valid_vehicle_model(VehicleModel.ST)
        assert not TrajectoryType.KS.valid_vehicle_model(VehicleModel.MB)

    def test_mb_trajectory_type(self):
        dummy_trajectory = DummyDataGenerator.create_random_mb_trajectory()
        assert TrajectoryType.MB.value == 'mbTrajectory'
        assert TrajectoryType.MB.state_type == StateType.MB
        assert TrajectoryType.MB == TrajectoryType.get_trajectory_type(dummy_trajectory)
        assert TrajectoryType.MB.valid_vehicle_model(VehicleModel.MB)
        assert not TrajectoryType.MB.valid_vehicle_model(VehicleModel.PM)
        assert not TrajectoryType.MB.valid_vehicle_model(VehicleModel.ST)
        assert not TrajectoryType.MB.valid_vehicle_model(VehicleModel.KS)

    def test_input_vector_type(self):
        dummy_trajectory = DummyDataGenerator.create_random_input_vector()
        assert TrajectoryType.Input.value == 'inputVector'
        assert TrajectoryType.Input.state_type == StateType.Input
        assert TrajectoryType.Input == TrajectoryType.get_trajectory_type(dummy_trajectory)
        assert TrajectoryType.Input.valid_vehicle_model(VehicleModel.ST)
        assert TrajectoryType.Input.valid_vehicle_model(VehicleModel.KS)
        assert TrajectoryType.Input.valid_vehicle_model(VehicleModel.MB)
        assert not TrajectoryType.Input.valid_vehicle_model(VehicleModel.PM)

    def test_pm_input_vector_type(self):
        dummy_trajectory = DummyDataGenerator.create_random_pm_input_vector()
        assert TrajectoryType.PMInput.value == 'pmInputVector'
        assert TrajectoryType.PMInput.state_type == StateType.PMInput
        assert TrajectoryType.PMInput == TrajectoryType.get_trajectory_type(dummy_trajectory)
        assert TrajectoryType.PMInput.valid_vehicle_model(VehicleModel.PM)
        assert not TrajectoryType.PMInput.valid_vehicle_model(VehicleModel.ST)
        assert not TrajectoryType.PMInput.valid_vehicle_model(VehicleModel.KS)
        assert not TrajectoryType.PMInput.valid_vehicle_model(VehicleModel.MB)


class TestPlanningProblemSolution(unittest.TestCase):

    def test_planning_problem_solution(self):
        pp_solution = PlanningProblemSolution(
            planning_problem_id=1,
            vehicle_model=VehicleModel.PM,
            vehicle_type=VehicleType.FORD_ESCORT,
            cost_function=CostFunction.JB1,
            trajectory=DummyDataGenerator.create_random_pm_trajectory()
        )

        assert pp_solution.planning_problem_id == 1
        assert pp_solution.vehicle_model == VehicleModel.PM
        assert pp_solution.vehicle_type == VehicleType.FORD_ESCORT
        assert pp_solution.vehicle_id == 'PM1'
        assert pp_solution.cost_function == CostFunction.JB1
        assert pp_solution.cost_id == 'JB1'
        assert pp_solution.trajectory_type == TrajectoryType.PM

    def test_planning_problem_solution_unsupported_cost(self):
        pp_solution = PlanningProblemSolution(
            planning_problem_id=1,
            vehicle_model=VehicleModel.PM,
            vehicle_type=VehicleType.FORD_ESCORT,
            cost_function=CostFunction.JB1,
            trajectory=DummyDataGenerator.create_random_pm_trajectory()
        )

        with self.assertRaises(Exception):
            pp_solution.cost_function = CostFunction.SA1

    def test_planning_problem_solution_invalid_vehicle_model(self):
        pp_solution = PlanningProblemSolution(
            planning_problem_id=1,
            vehicle_model=VehicleModel.PM,
            vehicle_type=VehicleType.FORD_ESCORT,
            cost_function=CostFunction.JB1,
            trajectory=DummyDataGenerator.create_random_pm_trajectory()
        )

        with self.assertRaises(Exception):
            pp_solution.vehicle_model = VehicleModel.MB

    def test_planning_problem_solution_invalid_trajectory(self):
        pp_solution = PlanningProblemSolution(
            planning_problem_id=1,
            vehicle_model=VehicleModel.PM,
            vehicle_type=VehicleType.FORD_ESCORT,
            cost_function=CostFunction.JB1,
            trajectory=DummyDataGenerator.create_random_pm_trajectory()
        )

        with self.assertRaises(Exception):
            pp_solution.trajectory = DummyDataGenerator.create_random_ks_trajectory()


class TestSolution(unittest.TestCase):

    def setUp(self) -> None:
        self.scenario_id = ScenarioID.from_benchmark_id('USA_US101-33_2_T-1', "2020a")

    def test_solution_vehicle_ids(self):
        pp_solution_1 = PlanningProblemSolution(
            planning_problem_id=1,
            vehicle_model=VehicleModel.KS,
            vehicle_type=VehicleType.FORD_ESCORT,
            cost_function=CostFunction.JB1,
            trajectory=DummyDataGenerator.create_random_ks_trajectory()
        )

        pp_solution_2 = PlanningProblemSolution(
            planning_problem_id=2,
            vehicle_model=VehicleModel.ST,
            vehicle_type=VehicleType.BMW_320i,
            cost_function=CostFunction.SA1,
            trajectory=DummyDataGenerator.create_random_st_trajectory()
        )

        solution_single = Solution(scenario_id=self.scenario_id,
                                   planning_problem_solutions=[pp_solution_1])
        solution_collab = Solution(scenario_id=self.scenario_id,
                                   planning_problem_solutions=[pp_solution_1, pp_solution_2])

        assert solution_single.vehicle_ids == ['KS1']
        assert solution_collab.vehicle_ids == ['KS1', 'ST2']

    def test_solution_cost_ids(self):
        pp_solution_1 = PlanningProblemSolution(
            planning_problem_id=1,
            vehicle_model=VehicleModel.KS,
            vehicle_type=VehicleType.FORD_ESCORT,
            cost_function=CostFunction.JB1,
            trajectory=DummyDataGenerator.create_random_ks_trajectory()
        )

        pp_solution_2 = PlanningProblemSolution(
            planning_problem_id=2,
            vehicle_model=VehicleModel.ST,
            vehicle_type=VehicleType.BMW_320i,
            cost_function=CostFunction.SA1,
            trajectory=DummyDataGenerator.create_random_st_trajectory()
        )

        solution_single = Solution(scenario_id=self.scenario_id,
                                   planning_problem_solutions=[pp_solution_1])
        solution_collab = Solution(scenario_id=self.scenario_id,
                                   planning_problem_solutions=[pp_solution_1, pp_solution_2])

        assert solution_single.cost_ids == ['JB1']
        assert solution_collab.cost_ids == ['JB1', 'SA1']

    def test_solution_benchmark_id(self):
        pp_solution_1 = PlanningProblemSolution(
            planning_problem_id=1,
            vehicle_model=VehicleModel.KS,
            vehicle_type=VehicleType.FORD_ESCORT,
            cost_function=CostFunction.JB1,
            trajectory=DummyDataGenerator.create_random_ks_trajectory()
        )

        pp_solution_2 = PlanningProblemSolution(
            planning_problem_id=2,
            vehicle_model=VehicleModel.ST,
            vehicle_type=VehicleType.BMW_320i,
            cost_function=CostFunction.SA1,
            trajectory=DummyDataGenerator.create_random_st_trajectory()
        )

        solution_single = Solution(scenario_id=self.scenario_id,
                                   planning_problem_solutions=[pp_solution_1])
        solution_collab = Solution(scenario_id=self.scenario_id,
                                   planning_problem_solutions=[pp_solution_1, pp_solution_2])

        assert solution_single.benchmark_id == 'KS1:JB1:USA_US101-33_2_T-1:2020a'
        assert solution_collab.benchmark_id == '[KS1,ST2]:[JB1,SA1]:USA_US101-33_2_T-1:2020a'

    def test_orientation_computation(self):
        pp_solution_1 = PlanningProblemSolution(
            planning_problem_id=1,
            vehicle_model=VehicleModel.PM,
            vehicle_type=VehicleType.FORD_ESCORT,
            cost_function=CostFunction.JB1,
            trajectory=DummyDataGenerator.create_random_pm_trajectory()
        )

        solution_single = Solution(scenario_id=self.scenario_id,
                                   planning_problem_solutions=[pp_solution_1])
        assert all(hasattr(s, 'orientation')
                   for s in solution_single.planning_problem_solutions[0]._trajectory.state_list)


class TestCommonRoadSolutionWriter(unittest.TestCase):

    @staticmethod
    def remove_whitespaces(xml_string: str):
        return re.sub(r"\s+", "", xml_string, flags=re.UNICODE)

    def setUp(self):
        self.pp_solution1 = PlanningProblemSolution(
            planning_problem_id=0,
            vehicle_model=VehicleModel.PM,
            vehicle_type=VehicleType.FORD_ESCORT,
            cost_function=CostFunction.JB1,
            trajectory=DummyDataGenerator.create_random_pm_trajectory()
        )
        self.pp_solution2 = PlanningProblemSolution(
            planning_problem_id=1,
            vehicle_model=VehicleModel.KS,
            vehicle_type=VehicleType.BMW_320i,
            cost_function=CostFunction.SA1,
            trajectory=DummyDataGenerator.create_random_ks_trajectory()
        )
        self.scenario_id = ScenarioID.from_benchmark_id('USA_US101-33_2_T-1', "2020a")

        self.solution_single: Solution = Solution(scenario_id=self.scenario_id,
                                        planning_problem_solutions=[self.pp_solution1])
        self.solution_collab: Solution = Solution(scenario_id=self.scenario_id,
                                        planning_problem_solutions=[self.pp_solution1,
                                                                    self.pp_solution2])

    def tearDown(self):
        for file in glob('./solution_*.xml'):
            os.remove(file)

    def test_dump(self):
        expected_solution_xml_single = DummyDataGenerator.create_solution_xml(self.solution_single)
        expected_solution_xml_collab = DummyDataGenerator.create_solution_xml(self.solution_collab)
        solution_xml_single = CommonRoadSolutionWriter(self.solution_single).dump()
        solution_xml_collab = CommonRoadSolutionWriter(self.solution_collab).dump()

        assert self.remove_whitespaces(solution_xml_single) == self.remove_whitespaces(expected_solution_xml_single)
        assert self.remove_whitespaces(solution_xml_collab) == self.remove_whitespaces(expected_solution_xml_collab)

    def test_dump_with_attribs(self):
        self.solution_single.processor_name = 'TEST_CPU'
        self.solution_collab.processor_name = 'TEST_CPU'
        self.solution_single.computation_time = 42.042
        self.solution_collab.computation_time = 42.042
        expected_solution_xml_single = DummyDataGenerator.create_solution_xml(self.solution_single)
        expected_solution_xml_collab = DummyDataGenerator.create_solution_xml(self.solution_collab)
        solution_xml_single = CommonRoadSolutionWriter(self.solution_single).dump()
        solution_xml_collab = CommonRoadSolutionWriter(self.solution_collab).dump()

        assert self.remove_whitespaces(solution_xml_single) == self.remove_whitespaces(expected_solution_xml_single)
        assert self.remove_whitespaces(solution_xml_collab) == self.remove_whitespaces(expected_solution_xml_collab)

    def test_write_to_file(self):
        CommonRoadSolutionWriter(self.solution_single).write_to_file()
        CommonRoadSolutionWriter(self.solution_collab).write_to_file()

        assert os.path.exists('./solution_' + self.solution_single.benchmark_id + '.xml')
        assert os.path.exists('./solution_' + self.solution_collab.benchmark_id + '.xml')

    def test_write_to_file_overwrite_not_allowed(self):
        CommonRoadSolutionWriter(self.solution_single).write_to_file()
        CommonRoadSolutionWriter(self.solution_collab).write_to_file()

        with self.assertRaises(FileExistsError):
            CommonRoadSolutionWriter(self.solution_single).write_to_file()

        with self.assertRaises(FileExistsError):
            CommonRoadSolutionWriter(self.solution_collab).write_to_file()

    def test_write_to_file_invalid_output_path(self):
        with self.assertRaises(NotADirectoryError):
            CommonRoadSolutionWriter(self.solution_single).write_to_file(output_path='./tests/')

    def test_write_version(self):
        self.solution_single.scenario_id.scenario_version = '2018b'
        path = "solution_test_file_writer_version.xml"
        CommonRoadSolutionWriter(self.solution_single).write_to_file(filename=path, overwrite=True)
        parsed_solution_single = CommonRoadSolutionReader.open("./" + path)

        assert parsed_solution_single.scenario_id.scenario_version == self.solution_single.scenario_id.scenario_version


class TestCommonRoadSolutionReader(unittest.TestCase):

    def setUp(self):
        self.pp_solution1 = PlanningProblemSolution(
            planning_problem_id=0,
            vehicle_model=VehicleModel.PM,
            vehicle_type=VehicleType.FORD_ESCORT,
            cost_function=CostFunction.JB1,
            trajectory=DummyDataGenerator.create_random_pm_trajectory()
        )
        self.pp_solution2 = PlanningProblemSolution(
            planning_problem_id=1,
            vehicle_model=VehicleModel.KS,
            vehicle_type=VehicleType.BMW_320i,
            cost_function=CostFunction.SA1,
            trajectory=DummyDataGenerator.create_random_ks_trajectory()
        )
        self.scenario_id = ScenarioID.from_benchmark_id('USA_US101-33_2_T-1', "2020a")
        self.solution_single = Solution(scenario_id=self.scenario_id,
                                        planning_problem_solutions=[self.pp_solution1])
        self.solution_collab = Solution(scenario_id=self.scenario_id,
                                        planning_problem_solutions=[self.pp_solution1,
                                                                    self.pp_solution2])

        CommonRoadSolutionWriter(self.solution_single).write_to_file(overwrite=True)
        CommonRoadSolutionWriter(self.solution_collab).write_to_file(overwrite=True)

        self.solution_single_path = './solution_' + self.solution_single.benchmark_id + '.xml'
        self.solution_collab_path = './solution_' + self.solution_collab.benchmark_id + '.xml'

    def tearDown(self):
        for file in glob('./solution_*.xml'):
            os.remove(file)

    def test_fromstring(self):
        solution_xml_single = DummyDataGenerator.create_solution_xml(self.solution_single)
        solution_xml_collab = DummyDataGenerator.create_solution_xml(self.solution_collab)

        parsed_solution_single = CommonRoadSolutionReader.fromstring(solution_xml_single)
        parsed_solution_collab = CommonRoadSolutionReader.fromstring(solution_xml_collab)

        assert str(parsed_solution_single.scenario_id) == str(self.solution_single.scenario_id)
        assert str(parsed_solution_collab.scenario_id) == str(self.solution_collab.scenario_id)
        assert parsed_solution_single.date.strftime('%Y-%m-%d') == self.solution_single.date.strftime('%Y-%m-%d')
        assert parsed_solution_collab.date.strftime('%Y-%m-%d') == self.solution_collab.date.strftime('%Y-%m-%d')
        assert parsed_solution_single.computation_time == self.solution_single.computation_time
        assert parsed_solution_collab.computation_time == self.solution_collab.computation_time
        assert parsed_solution_single.processor_name == self.solution_single.processor_name
        assert parsed_solution_collab.processor_name == self.solution_collab.processor_name
        assert parsed_solution_single.scenario_id.scenario_version == self.solution_single.scenario_id.scenario_version
        assert parsed_solution_collab.scenario_id.scenario_version == self.solution_collab.scenario_id.scenario_version
        assert parsed_solution_single.vehicle_ids == self.solution_single.vehicle_ids
        assert parsed_solution_collab.vehicle_ids == self.solution_collab.vehicle_ids
        assert parsed_solution_single.cost_ids == self.solution_single.cost_ids
        assert parsed_solution_collab.cost_ids == self.solution_collab.cost_ids
        assert parsed_solution_single.planning_problem_ids == self.solution_single.planning_problem_ids
        assert parsed_solution_collab.planning_problem_ids == self.solution_collab.planning_problem_ids
        assert parsed_solution_single.trajectory_types == self.solution_single.trajectory_types
        assert parsed_solution_collab.trajectory_types == self.solution_collab.trajectory_types
        assert parsed_solution_single.scenario_id == self.solution_single.scenario_id
        assert parsed_solution_collab.scenario_id == self.solution_collab.scenario_id
        assert all(hasattr(s, 'orientation')
                   for s in parsed_solution_single.planning_problem_solutions[0]._trajectory.state_list)

    def test_fromstring_computation_time(self):
        self.solution_single.computation_time = 1.10
        solution_xml_single = DummyDataGenerator.create_solution_xml(self.solution_single)
        parsed_solution_single = CommonRoadSolutionReader.fromstring(solution_xml_single)
        assert parsed_solution_single.computation_time == self.solution_single.computation_time

    def test_fromstring_with_attribs(self):
        self.solution_single.processor_name = 'TEST_CPU'
        self.solution_collab.processor_name = 'TEST_CPU'
        self.solution_single.computation_time = 42.042
        self.solution_collab.computation_time = 42.042
        solution_xml_single = DummyDataGenerator.create_solution_xml(self.solution_single)
        solution_xml_collab = DummyDataGenerator.create_solution_xml(self.solution_collab)

        parsed_solution_single = CommonRoadSolutionReader.fromstring(solution_xml_single)
        parsed_solution_collab = CommonRoadSolutionReader.fromstring(solution_xml_collab)

        assert str(parsed_solution_single.scenario_id) == str(self.solution_single.scenario_id)
        assert str(parsed_solution_collab.scenario_id) == str(self.solution_collab.scenario_id)
        assert parsed_solution_single.date.strftime('%Y-%m-%d') == self.solution_single.date.strftime('%Y-%m-%d')
        assert parsed_solution_collab.date.strftime('%Y-%m-%d') == self.solution_collab.date.strftime('%Y-%m-%d')
        assert parsed_solution_single.computation_time == self.solution_single.computation_time
        assert parsed_solution_collab.computation_time == self.solution_collab.computation_time
        assert parsed_solution_single.processor_name == self.solution_single.processor_name
        assert parsed_solution_collab.processor_name == self.solution_collab.processor_name
        assert parsed_solution_single.scenario_id.scenario_version == self.solution_single.scenario_id.scenario_version
        assert parsed_solution_collab.scenario_id.scenario_version == self.solution_collab.scenario_id.scenario_version
        assert parsed_solution_single.vehicle_ids == self.solution_single.vehicle_ids
        assert parsed_solution_collab.vehicle_ids == self.solution_collab.vehicle_ids
        assert parsed_solution_single.cost_ids == self.solution_single.cost_ids
        assert parsed_solution_collab.cost_ids == self.solution_collab.cost_ids
        assert parsed_solution_single.planning_problem_ids == self.solution_single.planning_problem_ids
        assert parsed_solution_collab.planning_problem_ids == self.solution_collab.planning_problem_ids
        assert parsed_solution_single.trajectory_types == self.solution_single.trajectory_types
        assert parsed_solution_collab.trajectory_types == self.solution_collab.trajectory_types
        assert parsed_solution_single.benchmark_id == self.solution_single.benchmark_id
        assert parsed_solution_collab.benchmark_id == self.solution_collab.benchmark_id

    def test_open(self):
        parsed_solution_single = CommonRoadSolutionReader.open(self.solution_single_path)
        parsed_solution_collab = CommonRoadSolutionReader.open(self.solution_collab_path)

        assert str(parsed_solution_single.scenario_id) == str(self.solution_single.scenario_id)
        assert str(parsed_solution_collab.scenario_id) == str(self.solution_collab.scenario_id)
        assert parsed_solution_single.date.strftime('%Y-%m-%d') == self.solution_single.date.strftime('%Y-%m-%d')
        assert parsed_solution_collab.date.strftime('%Y-%m-%d') == self.solution_collab.date.strftime('%Y-%m-%d')
        assert parsed_solution_single.computation_time == self.solution_single.computation_time
        assert parsed_solution_collab.computation_time == self.solution_collab.computation_time
        assert parsed_solution_single.processor_name == self.solution_single.processor_name
        assert parsed_solution_collab.processor_name == self.solution_collab.processor_name
        assert parsed_solution_single.scenario_id.scenario_version == self.solution_single.scenario_id.scenario_version
        assert parsed_solution_collab.scenario_id.scenario_version == self.solution_collab.scenario_id.scenario_version
        assert parsed_solution_single.vehicle_ids == self.solution_single.vehicle_ids
        assert parsed_solution_collab.vehicle_ids == self.solution_collab.vehicle_ids
        assert parsed_solution_single.cost_ids == self.solution_single.cost_ids
        assert parsed_solution_collab.cost_ids == self.solution_collab.cost_ids
        assert parsed_solution_single.planning_problem_ids == self.solution_single.planning_problem_ids
        assert parsed_solution_collab.planning_problem_ids == self.solution_collab.planning_problem_ids
        assert parsed_solution_single.trajectory_types == self.solution_single.trajectory_types
        assert parsed_solution_collab.trajectory_types == self.solution_collab.trajectory_types
        assert parsed_solution_single.benchmark_id == self.solution_single.benchmark_id
        assert parsed_solution_collab.benchmark_id == self.solution_collab.benchmark_id

    def test_open_invalid_path(self):
        with self.assertRaises(FileNotFoundError):
            parsed_solution_single = CommonRoadSolutionReader.open(self.solution_single_path + 'invalid')


if __name__ == '__main__':
    unittest.main()
