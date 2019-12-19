import unittest
import os

import numpy as np
from commonroad.scenario.trajectory import Trajectory, State
from commonroad.common.solution_writer import CommonRoadSolutionWriter, VehicleModel, VehicleType, CostFunction

class TestCommonroadSolutionWriter(unittest.TestCase):
    def setUp(self):
        """set up for testing: generate state lists and trajectories for various vehicle models"""
        self.output_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, '.pytest_cache')
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
        for file in os.listdir(self.output_dir):
            if file.endswith('.xml'):
                os.remove(os.path.join(self.output_dir, file))

        pm_state_list = list()
        ks_state_list = list()
        st_state_list = list()
        invalid_trajectory_list = list()
        for i in range(10):
            pm_state_list.append(State(**{'position': [i, -i], 'velocity': i*.2, 'velocity_y': i*0.001, 'time_step': i}))
            ks_state = State(**{'position': [i, -i], 'velocity': i*.2, 'orientation': i*0.001, 'steering_angle': 0.0,
                             'time_step': i})
            ks_state_list.append(ks_state)
            st_state = State(**{'position': [i, -i], 'velocity': i*.2, 'orientation': i*0.001, 'steering_angle': 0.0,
                                'yaw_rate': i*0.2, 'slip_angle': 0.1, 'time_step': i})
            st_state_list.append(st_state)
            invalid_trajectory_list.append(State(**{'position': [i, -i], 'velocity': i*.2, 'time_step': i}))

        self.trajectory_pm = Trajectory(0, pm_state_list)
        self.trajectory_ks = Trajectory(0, ks_state_list)
        self.trajectory_st = Trajectory(0, st_state_list)
        self.invalid_trajectory = Trajectory(0, invalid_trajectory_list)

        pm_input_list = []
        input_list = []
        for i in range(10):
            pm_input = State(**{'orientation': 0.1, 'acceleration': 5.0, 'time_step': i})
            pm_input_list.append(pm_input)
            input = State(**{'steering_angle_speed': 0.1, 'acceleration': 5.0, 'time_step': i})
            input_list.append(input)
        self.input_traj = Trajectory(0, input_list)
        self.pm_input_traj = Trajectory(0, pm_input_list)

    def test_write_trajectories_to_file(self):
        """
        test whether a file is generated from a state trajectory in setUp
        :return:
        """
        csw = CommonRoadSolutionWriter(self.output_dir, scenario_id='test_scenario', step_size=0.1)
        csw.add_solution_trajectory(self.trajectory_ks, 5)
        csw.write_to_file()
        benchmark_id = CommonRoadSolutionWriter._create_benchmark_id('test_scenario', VehicleType.FORD_ESCORT,
                                                                     VehicleModel.KS, CostFunction.JB1)
        assert(os.path.exists(os.path.join(self.output_dir, 'solution_' + benchmark_id + '.xml')))

        with self.assertRaises(AssertionError):
            csw.add_solution_trajectory(self.invalid_trajectory, 2)

    def test_write_input_to_file(self):
        """
        test whether a file is generated from an input trajectory in setUp
        :return:
        """
        csw = CommonRoadSolutionWriter(self.output_dir, scenario_id='test_scenario', step_size=0.1,
                                       cost_function=CostFunction.SA1)
        csw.add_solution_input_vector(self.input_traj, 5)
        csw.write_to_file()
        benchmark_id = CommonRoadSolutionWriter._create_benchmark_id('test_scenario', VehicleType.FORD_ESCORT,
                                                                     VehicleModel.KS, CostFunction.SA1)
        assert (os.path.exists(os.path.join(self.output_dir, 'solution_' + benchmark_id + '.xml')))

    def test_against_xsd(self):
        csw = CommonRoadSolutionWriter(self.output_dir, scenario_id='test_scenario', step_size=0.1,
                                       vehicle_model=VehicleModel.ST, cost_function=CostFunction.WX1,
                                       vehicle_type=VehicleType.VW_VANAGON, computation_time=0.001)
        csw.add_solution_trajectory(self.trajectory_st, 5)
        csw.write_to_file(overwrite=True)
        csw.check_validity_of_solution_file()

        csw = CommonRoadSolutionWriter(self.output_dir, scenario_id='test_scenario', step_size=0.1,
                                       vehicle_model=VehicleModel.ST, cost_function=CostFunction.WX1,
                                       vehicle_type=VehicleType.VW_VANAGON)
        csw.add_solution_input_vector(self.input_traj, 5)
        csw.write_to_file(overwrite=True)
        csw.check_validity_of_solution_file()

        csw = CommonRoadSolutionWriter(self.output_dir, scenario_id='test_scenario', step_size=0.1,
                                       vehicle_model=VehicleModel.KS, cost_function=CostFunction.WX1,
                                       vehicle_type=VehicleType.VW_VANAGON)
        csw.add_solution_input_vector(self.input_traj, 5)
        csw.write_to_file(overwrite=True)
        csw.check_validity_of_solution_file()

    def test_write_traejctory_and_input(self):
        """
        test to see whether a state and input trajectory can be submitted at the same time. As a result, it is not only
        checked whether the solution is drivable, but also whether the state and input trajectory are consistent. The
        latter is not tested in this unit test
        :return:
        """
        csw = CommonRoadSolutionWriter(self.output_dir, scenario_id='test_scenario', step_size=0.1,
                                       vehicle_model=VehicleModel.ST, cost_function=CostFunction.WX1,
                                       vehicle_type=VehicleType.VW_VANAGON)
        csw.add_solution_trajectory(self.trajectory_st, 5)
        csw.add_solution_input_vector(self.input_traj, 5)
        csw.write_to_file()
        benchmark_id = CommonRoadSolutionWriter._create_benchmark_id('test_scenario', VehicleType.VW_VANAGON,
                                                                     VehicleModel.ST, CostFunction.WX1)
        assert (os.path.exists(os.path.join(self.output_dir, 'solution_' + benchmark_id + '.xml')))


if __name__ == '__main__':
    unittest.main()
