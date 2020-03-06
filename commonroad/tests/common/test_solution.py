import unittest

from commonroad.common.solution import StateFields, XMLStateFields, StateType, TrajectoryType, PlanningProblemSolution, \
    Solution
from commonroad.common.solution_writer import VehicleModel, VehicleType, CostFunction
from commonroad.scenario.trajectory import State, Trajectory


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
        assert StateFields.PMInput.value == ['acceleration_x', 'acceleration_y', 'time_step']


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
        assert StateType.PM == StateType.get_state_type(self.create_dummy_state(StateFields.PM))

    def test_st_state_type(self):
        assert StateType.ST.value == 'stState'
        assert StateType.ST.fields == StateFields.ST.value
        assert StateType.ST.xml_fields == XMLStateFields.ST.value
        assert StateType.ST == StateType.get_state_type(self.create_dummy_state(StateFields.ST))

    def test_ks_state_type(self):
        assert StateType.KS.value == 'ksState'
        assert StateType.KS.fields == StateFields.KS.value
        assert StateType.KS.xml_fields == XMLStateFields.KS.value
        assert StateType.KS == StateType.get_state_type(self.create_dummy_state(StateFields.KS))

    def test_mb_state_type(self):
        assert StateType.MB.value == 'mbState'
        assert StateType.MB.fields == StateFields.MB.value
        assert StateType.MB.xml_fields == XMLStateFields.MB.value
        assert StateType.MB == StateType.get_state_type(self.create_dummy_state(StateFields.MB))

    def test_input_state_type(self):
        assert StateType.Input.value == 'input'
        assert StateType.Input.fields == StateFields.Input.value
        assert StateType.Input.xml_fields == XMLStateFields.Input.value
        assert StateType.Input == StateType.get_state_type(self.create_dummy_state(StateFields.Input))

    # def test_pm_input_state_type(self):
    #     assert StateType.PMInput.value == 'pmInput'
    #     assert StateType.PMInput.fields == StateFields.PMInput.value
    #     assert StateType.PMInput.xml_fields == XMLStateFields.PMInput.value
    #     assert StateType.PMInput == StateType.get_state_type(self.create_dummy_state(StateFields.PMInput))


class TestTrajectoryType(unittest.TestCase):

    @staticmethod
    def create_dummy_state(state_type: StateType, time_step: int) -> State:
        values = {field: 0.0 for field in state_type.fields}
        values['time_step'] = time_step
        return State(**values)

    @classmethod
    def create_dummy_trajectory(cls, state_type: StateType, count=5) -> Trajectory:
        state_list = [cls.create_dummy_state(state_type, time_step) for time_step in range(count)]
        return Trajectory(initial_time_step=state_list[0].time_step, state_list=state_list)

    def test_pm_trajectory_type(self):
        assert TrajectoryType.PM.value == 'pmTrajectory'
        assert TrajectoryType.PM.state_type == StateType.PM
        assert TrajectoryType.PM == TrajectoryType.get_trajectory_type(self.create_dummy_trajectory(StateType.PM))
        assert TrajectoryType.PM.valid_vehicle_model(VehicleModel.PM)
        assert not TrajectoryType.PM.valid_vehicle_model(VehicleModel.ST)
        assert not TrajectoryType.PM.valid_vehicle_model(VehicleModel.KS)
        assert not TrajectoryType.PM.valid_vehicle_model(VehicleModel.MB)

    def test_st_trajectory_type(self):
        assert TrajectoryType.ST.value == 'stTrajectory'
        assert TrajectoryType.ST.state_type == StateType.ST
        assert TrajectoryType.ST == TrajectoryType.get_trajectory_type(self.create_dummy_trajectory(StateType.ST))
        assert TrajectoryType.ST.valid_vehicle_model(VehicleModel.ST)
        assert not TrajectoryType.ST.valid_vehicle_model(VehicleModel.PM)
        assert not TrajectoryType.ST.valid_vehicle_model(VehicleModel.KS)
        assert not TrajectoryType.ST.valid_vehicle_model(VehicleModel.MB)

    def test_ks_trajectory_type(self):
        assert TrajectoryType.KS.value == 'ksTrajectory'
        assert TrajectoryType.KS.state_type == StateType.KS
        assert TrajectoryType.KS == TrajectoryType.get_trajectory_type(self.create_dummy_trajectory(StateType.KS))
        assert TrajectoryType.KS.valid_vehicle_model(VehicleModel.KS)
        assert not TrajectoryType.KS.valid_vehicle_model(VehicleModel.PM)
        assert not TrajectoryType.KS.valid_vehicle_model(VehicleModel.ST)
        assert not TrajectoryType.KS.valid_vehicle_model(VehicleModel.MB)

    def test_mb_trajectory_type(self):
        assert TrajectoryType.MB.value == 'mbTrajectory'
        assert TrajectoryType.MB.state_type == StateType.MB
        assert TrajectoryType.MB == TrajectoryType.get_trajectory_type(self.create_dummy_trajectory(StateType.MB))
        assert TrajectoryType.MB.valid_vehicle_model(VehicleModel.MB)
        assert not TrajectoryType.MB.valid_vehicle_model(VehicleModel.PM)
        assert not TrajectoryType.MB.valid_vehicle_model(VehicleModel.ST)
        assert not TrajectoryType.MB.valid_vehicle_model(VehicleModel.KS)

    def test_input_vector_type(self):
        dummy_trajectory = self.create_dummy_trajectory(StateType.Input)
        assert TrajectoryType.Input.value == 'inputVector'
        assert TrajectoryType.Input.state_type == StateType.Input
        assert TrajectoryType.Input == TrajectoryType.get_trajectory_type(dummy_trajectory)
        assert TrajectoryType.Input.valid_vehicle_model(VehicleModel.ST)
        assert TrajectoryType.Input.valid_vehicle_model(VehicleModel.KS)
        assert TrajectoryType.Input.valid_vehicle_model(VehicleModel.MB)
        assert not TrajectoryType.Input.valid_vehicle_model(VehicleModel.PM)

    # def test_pm_input_vector_type(self):
    #     dummy_trajectory = self.create_dummy_trajectory(StateType.PMInput)
    #     assert TrajectoryType.PMInput.value == 'pmInputVector'
    #     assert TrajectoryType.PMInput.state_type == StateType.PMInput
    #     assert TrajectoryType.PMInput == TrajectoryType.get_trajectory_type(dummy_trajectory)
    #     assert TrajectoryType.PMInput.valid_vehicle_model(VehicleModel.PM)
    #     assert not TrajectoryType.PMInput.valid_vehicle_model(VehicleModel.ST)
    #     assert not TrajectoryType.PMInput.valid_vehicle_model(VehicleModel.KS)
    #     assert not TrajectoryType.PMInput.valid_vehicle_model(VehicleModel.MB)


class TestPlanningProblemSolution(unittest.TestCase):

    @staticmethod
    def create_dummy_state(state_type: StateType, time_step: int) -> State:
        values = {field: 0.0 for field in state_type.fields}
        values['time_step'] = time_step
        return State(**values)

    @classmethod
    def create_dummy_trajectory(cls, trajectory_type: TrajectoryType, count=5) -> Trajectory:
        state_list = [cls.create_dummy_state(trajectory_type.state_type, time_step) for time_step in range(count)]
        return Trajectory(initial_time_step=state_list[0].time_step, state_list=state_list)

    def test_planning_problem_solution(self):
        pp_solution = PlanningProblemSolution(
            planning_problem_id=1,
            vehicle_model=VehicleModel.PM,
            vehicle_type=VehicleType.FORD_ESCORT,
            cost_function=CostFunction.JB1,
            trajectory=self.create_dummy_trajectory(TrajectoryType.PM)
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
            trajectory=self.create_dummy_trajectory(TrajectoryType.PM)
        )

        with self.assertRaises(Exception):
            pp_solution.cost_function = CostFunction.SA1

    def test_planning_problem_solution_invalid_vehicle_model(self):
        pp_solution = PlanningProblemSolution(
            planning_problem_id=1,
            vehicle_model=VehicleModel.PM,
            vehicle_type=VehicleType.FORD_ESCORT,
            cost_function=CostFunction.JB1,
            trajectory=self.create_dummy_trajectory(TrajectoryType.PM)
        )

        with self.assertRaises(Exception):
            pp_solution.vehicle_model = VehicleModel.MB

    def test_planning_problem_solution_invalid_trajectory(self):
        pp_solution = PlanningProblemSolution(
            planning_problem_id=1,
            vehicle_model=VehicleModel.PM,
            vehicle_type=VehicleType.FORD_ESCORT,
            cost_function=CostFunction.JB1,
            trajectory=self.create_dummy_trajectory(TrajectoryType.PM)
        )

        with self.assertRaises(Exception):
            pp_solution.trajectory = self.create_dummy_trajectory(TrajectoryType.KS)


class TestSolution(unittest.TestCase):

    @staticmethod
    def create_dummy_state(state_type: StateType, time_step: int) -> State:
        values = {field: 0.0 for field in state_type.fields}
        values['time_step'] = time_step
        return State(**values)

    @classmethod
    def create_dummy_trajectory(cls, trajectory_type: TrajectoryType, count=5) -> Trajectory:
        state_list = [cls.create_dummy_state(trajectory_type.state_type, time_step) for time_step in range(count)]
        return Trajectory(initial_time_step=state_list[0].time_step, state_list=state_list)

    @classmethod
    def create_dummy_pp_solution(cls,
                                 pp_id: int,
                                 vehicle_model: VehicleModel,
                                 vehicle_type: VehicleType,
                                 cost_function: CostFunction,
                                 trajectory_type: TrajectoryType):
        trajectory = cls.create_dummy_trajectory(trajectory_type)
        return PlanningProblemSolution(pp_id, vehicle_model, vehicle_type, cost_function, trajectory)

    def test_solution_vehicle_ids(self):
        pp_solution_1 = self.create_dummy_pp_solution(
            pp_id=1,
            vehicle_model=VehicleModel.KS,
            vehicle_type=VehicleType.FORD_ESCORT,
            cost_function=CostFunction.JB1,
            trajectory_type=TrajectoryType.KS
        )

        pp_solution_2 = self.create_dummy_pp_solution(
            pp_id=2,
            vehicle_model=VehicleModel.ST,
            vehicle_type=VehicleType.BMW_320i,
            cost_function=CostFunction.SA1,
            trajectory_type=TrajectoryType.ST
        )

        solution_single = Solution(scenario_id='TEST', planning_problem_solutions=[pp_solution_1])
        solution_collab = Solution(scenario_id='TEST', planning_problem_solutions=[pp_solution_1, pp_solution_2])

        assert solution_single.vehicle_ids == ['KS1']
        assert solution_collab.vehicle_ids == ['KS1', 'ST2']

    def test_solution_cost_ids(self):
        pp_solution_1 = self.create_dummy_pp_solution(
            pp_id=1,
            vehicle_model=VehicleModel.KS,
            vehicle_type=VehicleType.FORD_ESCORT,
            cost_function=CostFunction.JB1,
            trajectory_type=TrajectoryType.KS
        )

        pp_solution_2 = self.create_dummy_pp_solution(
            pp_id=2,
            vehicle_model=VehicleModel.ST,
            vehicle_type=VehicleType.BMW_320i,
            cost_function=CostFunction.SA1,
            trajectory_type=TrajectoryType.ST
        )

        solution_single = Solution(scenario_id='TEST', planning_problem_solutions=[pp_solution_1])
        solution_collab = Solution(scenario_id='TEST', planning_problem_solutions=[pp_solution_1, pp_solution_2])

        assert solution_single.cost_ids == ['JB1']
        assert solution_collab.cost_ids == ['JB1', 'SA1']

    def test_solution_benchmark_id(self):
        pp_solution_1 = self.create_dummy_pp_solution(
            pp_id=1,
            vehicle_model=VehicleModel.KS,
            vehicle_type=VehicleType.FORD_ESCORT,
            cost_function=CostFunction.JB1,
            trajectory_type=TrajectoryType.KS
        )

        pp_solution_2 = self.create_dummy_pp_solution(
            pp_id=2,
            vehicle_model=VehicleModel.ST,
            vehicle_type=VehicleType.BMW_320i,
            cost_function=CostFunction.SA1,
            trajectory_type=TrajectoryType.ST
        )

        solution_single = Solution(scenario_id='TEST', planning_problem_solutions=[pp_solution_1])
        solution_collab = Solution(scenario_id='TEST', planning_problem_solutions=[pp_solution_1, pp_solution_2])

        assert solution_single.benchmark_id == 'KS1:JB1:TEST:2018b'
        assert solution_collab.benchmark_id == '[KS1,ST2]:[JB1,SA1]:TEST:2018b'


# class TestCommonroadSolutionWriter(unittest.TestCase):
#     def setUp(self):
#         """set up for testing: generate state lists and trajectories for various vehicle models"""
#         self.output_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, '.pytest_cache')
#         if not os.path.exists(self.output_dir):
#             os.makedirs(self.output_dir)
#         for file in os.listdir(self.output_dir):
#             if file.endswith('.xml'):
#                 os.remove(os.path.join(self.output_dir, file))
#
#         pm_state_list = list()
#         ks_state_list = list()
#         st_state_list = list()
#         invalid_trajectory_list = list()
#         for i in range(10):
#             pm_state_list.append(State(**{'position': [i, -i], 'velocity': i*.2, 'velocity_y': i*0.001, 'time_step': i}))
#             ks_state = State(**{'position': [i, -i], 'velocity': i*.2, 'orientation': i*0.001, 'steering_angle': 0.0,
#                              'time_step': i})
#             ks_state_list.append(ks_state)
#             st_state = State(**{'position': [i, -i], 'velocity': i*.2, 'orientation': i*0.001, 'steering_angle': 0.0,
#                                 'yaw_rate': i*0.2, 'slip_angle': 0.1, 'time_step': i})
#             st_state_list.append(st_state)
#             invalid_trajectory_list.append(State(**{'position': [i, -i], 'velocity': i*.2, 'time_step': i}))
#
#         self.trajectory_pm = Trajectory(0, pm_state_list)
#         self.trajectory_ks = Trajectory(0, ks_state_list)
#         self.trajectory_st = Trajectory(0, st_state_list)
#         self.invalid_trajectory = Trajectory(0, invalid_trajectory_list)
#
#         pm_input_list = []
#         input_list = []
#         for i in range(10):
#             pm_input = State(**{'orientation': 0.1, 'acceleration': 5.0, 'time_step': i})
#             pm_input_list.append(pm_input)
#             input = State(**{'steering_angle_speed': 0.1, 'acceleration': 5.0, 'time_step': i})
#             input_list.append(input)
#         self.input_traj = Trajectory(0, input_list)
#         self.pm_input_traj = Trajectory(0, pm_input_list)
#
#     def test_write_trajectories_to_file(self):
#         """
#         test whether a file is generated from a state trajectory in setUp
#         :return:
#         """
#         csw = CommonRoadSolutionWriter(self.output_dir, scenario_id='test_scenario', step_size=0.1)
#         csw.add_solution_trajectory(self.trajectory_ks, 5)
#         csw.write_to_file()
#         benchmark_id = CommonRoadSolutionWriter._create_benchmark_id('test_scenario', VehicleType.FORD_ESCORT,
#                                                                      VehicleModel.KS, CostFunction.JB1)
#         assert(os.path.exists(os.path.join(self.output_dir, 'solution_' + benchmark_id + '.xml')))
#
#         with self.assertRaises(AssertionError):
#             csw.add_solution_trajectory(self.invalid_trajectory, 2)
#
#     def test_write_input_to_file(self):
#         """
#         test whether a file is generated from an input trajectory in setUp
#         :return:
#         """
#         csw = CommonRoadSolutionWriter(self.output_dir, scenario_id='test_scenario', step_size=0.1,
#                                        cost_function=CostFunction.SA1)
#         csw.add_solution_input_vector(self.input_traj, 5)
#         csw.write_to_file()
#         benchmark_id = CommonRoadSolutionWriter._create_benchmark_id('test_scenario', VehicleType.FORD_ESCORT,
#                                                                      VehicleModel.KS, CostFunction.SA1)
#         assert (os.path.exists(os.path.join(self.output_dir, 'solution_' + benchmark_id + '.xml')))
#
#     def test_against_xsd(self):
#         csw = CommonRoadSolutionWriter(self.output_dir, scenario_id='test_scenario', step_size=0.1,
#                                        vehicle_model=VehicleModel.ST, cost_function=CostFunction.WX1,
#                                        vehicle_type=VehicleType.VW_VANAGON, computation_time=0.001)
#         csw.add_solution_trajectory(self.trajectory_st, 5)
#         csw.write_to_file(overwrite=True)
#         csw.check_validity_of_solution_file()
#
#         csw = CommonRoadSolutionWriter(self.output_dir, scenario_id='test_scenario', step_size=0.1,
#                                        vehicle_model=VehicleModel.ST, cost_function=CostFunction.WX1,
#                                        vehicle_type=VehicleType.VW_VANAGON)
#         csw.add_solution_input_vector(self.input_traj, 5)
#         csw.write_to_file(overwrite=True)
#         csw.check_validity_of_solution_file()
#
#         csw = CommonRoadSolutionWriter(self.output_dir, scenario_id='test_scenario', step_size=0.1,
#                                        vehicle_model=VehicleModel.KS, cost_function=CostFunction.WX1,
#                                        vehicle_type=VehicleType.VW_VANAGON)
#         csw.add_solution_input_vector(self.input_traj, 5)
#         csw.write_to_file(overwrite=True)
#         csw.check_validity_of_solution_file()
#
#     def test_write_traejctory_and_input(self):
#         """
#         test to see whether a state and input trajectory can be submitted at the same time. As a result, it is not only
#         checked whether the solution is drivable, but also whether the state and input trajectory are consistent. The
#         latter is not tested in this unit test
#         :return:
#         """
#         csw = CommonRoadSolutionWriter(self.output_dir, scenario_id='test_scenario', step_size=0.1,
#                                        vehicle_model=VehicleModel.ST, cost_function=CostFunction.WX1,
#                                        vehicle_type=VehicleType.VW_VANAGON)
#         csw.add_solution_trajectory(self.trajectory_st, 5)
#         csw.add_solution_input_vector(self.input_traj, 5)
#         csw.write_to_file()
#         benchmark_id = CommonRoadSolutionWriter._create_benchmark_id('test_scenario', VehicleType.VW_VANAGON,
#                                                                      VehicleModel.ST, CostFunction.WX1)
#         assert (os.path.exists(os.path.join(self.output_dir, 'solution_' + benchmark_id + '.xml')))


if __name__ == '__main__':
    unittest.main()
