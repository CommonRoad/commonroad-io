import os
import unittest

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.geometry.shape import *
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet, GoalRegion
from commonroad.prediction.prediction import *
from commonroad.scenario.lanelet import Lanelet, LaneletNetwork, LineMarking
from commonroad.scenario.obstacle import *
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import *


class TestFileReader(unittest.TestCase):
    def setUp(self):

        self.cwd_path = os.path.dirname(os.path.abspath(__file__))
        self.filename_all = self.cwd_path + '/test_reading_all.xml'
        self.filename_lanelets = self.cwd_path + '/test_reading_lanelets.xml'
        self.filename_obstacle = self.cwd_path + '/test_reading_obstacles.xml'
        self.filename_planning_problem = self.cwd_path + '/test_reading_planning_problem.xml'

        rectangle = Rectangle(4.3, 8.9, center=np.array([0.1, 0.5]), orientation=1.7)
        polygon = Polygon(
            np.array([np.array((0.0, 0.0)), np.array((0.0, 1.0)), np.array((1.0, 1.0)), np.array((1.0, 0.0))]))
        circ = Circle(2.0, np.array([0.0, 0.0]))
        sg = ShapeGroup([circ, rectangle])
        occupancy_list = list()
        occupancy_list.append(Occupancy(0, rectangle))
        occupancy_list.append(Occupancy(1, circ))
        occupancy_list.append(Occupancy(2, polygon))
        occupancy_list.append(Occupancy(3, circ))

        set_pred = SetBasedPrediction(0, occupancy_list)

        states = list()
        states.append(State(time_step=0, orientation=0, position=np.array([0, 0])))
        states.append(State(time_step=1, orientation=0, position=np.array([0, 1])))
        trajectory = Trajectory(0, states)

        init_state = State(time_step=0, orientation=0, position=np.array([0, 0]))

        traj_pred = TrajectoryPrediction(trajectory, rectangle)

        static_obs = StaticObstacle(0, ObstacleType(0), obstacle_shape=circ, initial_state=init_state)
        dyn_set_obs = DynamicObstacle(1, ObstacleType(0),
                                      initial_state=traj_pred.trajectory.state_at_time_step(0),
                                      prediction=set_pred, obstacle_shape=rectangle)
        dyn_traj_obs = DynamicObstacle(2, ObstacleType(0),
                                       initial_state=traj_pred.trajectory.state_at_time_step(0),
                                       prediction=traj_pred, obstacle_shape=rectangle)
        lanelet1 = Lanelet(np.array([[0.0, 0.0], [1.0, 0.0], [2, 0]]), np.array([[0.0, 1], [1.0, 1], [2, 1]]),
                           np.array([[0.0, 2], [1.0, 2], [2, 2]]), 100,
                           [101], [101], 101, False, 101, True, 10.0,
                           LineMarking.DASHED, LineMarking.DASHED)
        lanelet2 = Lanelet(np.array([[0.0, 0.0], [1.0, 0.0], [2, 0]]), np.array([[0.0, 1], [1.0, 1], [2, 1]]),
                           np.array([[0.0, 2], [1.0, 2], [2, 2]]), 101,
                           [100], [100], 100, False, 100, True, 10.0,
                           LineMarking.DASHED, LineMarking.DASHED)

        self.lanelet_network = LaneletNetwork().create_from_lanelet_list(list([lanelet1, lanelet2]))
        self.scenario = Scenario(0.1, 'ZAM_test_0-0-1')
        self.scenario.add_objects([static_obs, dyn_set_obs, dyn_traj_obs, self.lanelet_network])

        goal_region = GoalRegion([State(time_step=Interval(0, 1), velocity=Interval(0.0, 1), position=rectangle),
                                  State(time_step=Interval(1, 2), velocity=Interval(0.0, 1), position=circ)],
                                 {0: [101, 102], 1: list([102])})
        planning_problem = PlanningProblem(1000, State(velocity=0.1, position=np.array([[0], [0]]), orientation=0,
                                                       yaw_rate=0, slip_angle=0, time_step=0), goal_region)
        self.planning_problem_set = PlanningProblemSet(list([planning_problem]))

    def test_open_lanelets(self):
        lanelets = CommonRoadFileReader(self.filename_lanelets).open()

        exp_lanelet_zero_center_vertices = self.scenario.lanelet_network.lanelets[0].center_vertices
        exp_lanelet_zero_left_vertices = self.scenario.lanelet_network.lanelets[0].left_vertices
        exp_lanelet_zero_right_vertices = self.scenario.lanelet_network.lanelets[0].right_vertices
        exp_lanelet_zero_adj_left = self.scenario.lanelet_network.lanelets[0].adj_left
        exp_lanelet_zero_adj_right = self.scenario.lanelet_network.lanelets[0].adj_right
        exp_lanelet_zero_adj_left_same_direction = self.scenario.lanelet_network.lanelets[0].adj_left_same_direction
        exp_lanelet_zero_adj_right_same_direction = self.scenario.lanelet_network.lanelets[0].adj_right_same_direction
        exp_lanelet_zero_predecessor = self.scenario.lanelet_network.lanelets[0].predecessor
        exp_lanelet_zero_succesor = self.scenario.lanelet_network.lanelets[0].successor
        exp_lanelet_zero_speed_limit = self.scenario.lanelet_network.lanelets[0].speed_limit

        np.testing.assert_array_equal(exp_lanelet_zero_center_vertices,
                                      lanelets[0].lanelet_network.lanelets[0].center_vertices)
        np.testing.assert_array_equal(exp_lanelet_zero_left_vertices,
                                      lanelets[0].lanelet_network.lanelets[0].left_vertices)
        np.testing.assert_array_equal(exp_lanelet_zero_right_vertices,
                                      lanelets[0].lanelet_network.lanelets[0].right_vertices)
        self.assertEqual(exp_lanelet_zero_adj_left, lanelets[0].lanelet_network.lanelets[0].adj_left)
        self.assertEqual(exp_lanelet_zero_adj_right, lanelets[0].lanelet_network.lanelets[0].adj_right)
        self.assertEqual(exp_lanelet_zero_adj_left_same_direction,
                         lanelets[0].lanelet_network.lanelets[0].adj_left_same_direction)
        self.assertEqual(exp_lanelet_zero_adj_right_same_direction,
                         lanelets[0].lanelet_network.lanelets[0].adj_right_same_direction)
        self.assertEqual(exp_lanelet_zero_predecessor, lanelets[0].lanelet_network.lanelets[0].predecessor)
        self.assertEqual(exp_lanelet_zero_succesor, lanelets[0].lanelet_network.lanelets[0].successor)
        self.assertEqual(exp_lanelet_zero_speed_limit, lanelets[0].lanelet_network.lanelets[0].speed_limit)

    def test_open_obstacles(self):
        obstacles = CommonRoadFileReader(self.filename_obstacle).open()

        exp_obstacle_zero_id = self.scenario.obstacles[0].obstacle_id
        exp_obstacle_zero_type = self.scenario.obstacles[0].obstacle_type
        exp_obstacle_zero_role = self.scenario.obstacles[0].obstacle_role
        exp_obstacle_zero_shape = self.scenario.obstacles[0].obstacle_shape.__class__
        exp_obstacle_zero_radius = self.scenario.obstacles[0].obstacle_shape.radius
        exp_obstacle_zero_center = self.scenario.obstacles[0].obstacle_shape.center

        exp_obstacle_one_id = self.scenario.obstacles[1].obstacle_id
        exp_obstacle_one_type = self.scenario.obstacles[1].obstacle_type
        exp_obstacle_one_role = self.scenario.obstacles[1].obstacle_role
        exp_obstacle_one_shape = self.scenario.obstacles[1].obstacle_shape.__class__
        exp_obstacle_one_attributes = len(self.scenario.obstacles[1].initial_state.attributes)
        exp_obstacle_one_orientation = self.scenario.obstacles[1].initial_state.orientation
        exp_obstacle_one_predicition_zero_shape_center = self.scenario.obstacles[1].prediction.occupancy_set[
            0].shape.center

        exp_obstacle_two_id = self.scenario.obstacles[2].obstacle_id
        exp_obstacle_two_type = self.scenario.obstacles[2].obstacle_type
        exp_obstacle_two_role = self.scenario.obstacles[2].obstacle_role
        exp_obstacle_two_shape = self.scenario.obstacles[2].obstacle_shape.__class__
        exp_obstacle_two_shape_prediction_state_list_len = len(
            self.scenario.obstacles[2].prediction.trajectory.state_list)
        exp_obstacle_two_shape_prediction_final_state_attributes_len = len(
            self.scenario.obstacles[2].prediction.trajectory.final_state.attributes)

        self.assertEqual(exp_obstacle_zero_id, obstacles[0].obstacles[0].obstacle_id)
        self.assertEqual(exp_obstacle_zero_type, obstacles[0].obstacles[0].obstacle_type)
        self.assertEqual(exp_obstacle_zero_role, obstacles[0].obstacles[0].obstacle_role)
        self.assertEqual(exp_obstacle_zero_shape, obstacles[0].obstacles[0].obstacle_shape.__class__)
        self.assertEqual(exp_obstacle_zero_radius, obstacles[0].obstacles[0].obstacle_shape.radius)
        self.assertEqual(exp_obstacle_zero_center[0], obstacles[0].obstacles[0].obstacle_shape.center[0])
        self.assertEqual(exp_obstacle_zero_center[1], obstacles[0].obstacles[0].obstacle_shape.center[1])

        self.assertEqual(exp_obstacle_one_id, obstacles[0].obstacles[1].obstacle_id)
        self.assertEqual(exp_obstacle_one_type, obstacles[0].obstacles[1].obstacle_type)
        self.assertEqual(exp_obstacle_one_role, obstacles[0].obstacles[1].obstacle_role)
        self.assertEqual(exp_obstacle_one_shape, obstacles[0].obstacles[1].obstacle_shape.__class__)
        self.assertEqual(exp_obstacle_one_attributes, len(obstacles[0].obstacles[1].initial_state.attributes))
        self.assertEqual(exp_obstacle_one_orientation, obstacles[0].obstacles[1].initial_state.orientation)
        self.assertEqual(exp_obstacle_one_predicition_zero_shape_center[0],
                         obstacles[0].obstacles[1].prediction.occupancy_set[
                             0].shape.center[0])
        self.assertEqual(exp_obstacle_one_predicition_zero_shape_center[1],
                         obstacles[0].obstacles[1].prediction.occupancy_set[
                             0].shape.center[1])

        self.assertEqual(exp_obstacle_two_id, obstacles[0].obstacles[2].obstacle_id)
        self.assertEqual(exp_obstacle_two_type, obstacles[0].obstacles[2].obstacle_type)
        self.assertEqual(exp_obstacle_two_role, obstacles[0].obstacles[2].obstacle_role)
        self.assertEqual(exp_obstacle_two_shape, obstacles[0].obstacles[2].obstacle_shape.__class__)
        self.assertEqual(exp_obstacle_two_shape_prediction_state_list_len, len(
            obstacles[0].obstacles[2].prediction.trajectory.state_list))
        self.assertEqual(exp_obstacle_two_shape_prediction_final_state_attributes_len, len(
            obstacles[0].obstacles[2].prediction.trajectory.final_state.attributes))

    def test_open_planning_problem(self):
        planning_problem = CommonRoadFileReader(self.filename_planning_problem).open()

        exp_planning_problem_id = self.planning_problem_set.planning_problem_dict[1000].planning_problem_id
        exp_planning_problem_initial_state_velocity = self.planning_problem_set.planning_problem_dict[
            1000].initial_state.velocity
        exp_planning_problem_initial_state_slip_angle = self.planning_problem_set.planning_problem_dict[
            1000].initial_state.slip_angle
        exp_planning_problem_initial_state_yaw_rate = self.planning_problem_set.planning_problem_dict[
            1000].initial_state.yaw_rate

        self.assertEqual(exp_planning_problem_id, planning_problem[1].planning_problem_dict[1000].planning_problem_id)
        self.assertEqual(exp_planning_problem_initial_state_velocity,
                         planning_problem[1].planning_problem_dict[1000].initial_state.velocity)
        self.assertEqual(exp_planning_problem_initial_state_slip_angle,
                         planning_problem[1].planning_problem_dict[1000].initial_state.slip_angle)
        self.assertEqual(exp_planning_problem_initial_state_yaw_rate,
                         planning_problem[1].planning_problem_dict[1000].initial_state.yaw_rate)

    def test_open_all(self):

        exp_num_lanelet_scenario = len(self.scenario.lanelet_network.lanelets)
        exp_num_obstacles_scenario = len(self.scenario.obstacles)
        exp_num_planning_problems = len(self.planning_problem_set.planning_problem_dict)
        exp_benchmark_id = self.scenario.benchmark_id
        exp_dt = self.scenario.dt

        exp_obstacle_zero_id = self.scenario.obstacles[0].obstacle_id
        exp_obstacle_zero_type = self.scenario.obstacles[0].obstacle_type
        exp_obstacle_zero_role = self.scenario.obstacles[0].obstacle_role
        exp_obstacle_zero_shape = self.scenario.obstacles[0].obstacle_shape.__class__
        exp_obstacle_zero_radius = self.scenario.obstacles[0].obstacle_shape.radius
        exp_obstacle_zero_center = self.scenario.obstacles[0].obstacle_shape.center

        exp_obstacle_one_id = self.scenario.obstacles[1].obstacle_id
        exp_obstacle_one_type = self.scenario.obstacles[1].obstacle_type
        exp_obstacle_one_role = self.scenario.obstacles[1].obstacle_role
        exp_obstacle_one_shape = self.scenario.obstacles[1].obstacle_shape.__class__
        exp_obstacle_one_attributes = len(self.scenario.obstacles[1].initial_state.attributes)
        exp_obstacle_one_orientation = self.scenario.obstacles[1].initial_state.orientation
        exp_obstacle_one_predicition_zero_shape_center = self.scenario.obstacles[1].prediction.occupancy_set[0].shape.center

        exp_obstacle_two_id = self.scenario.obstacles[2].obstacle_id
        exp_obstacle_two_type = self.scenario.obstacles[2].obstacle_type
        exp_obstacle_two_role = self.scenario.obstacles[2].obstacle_role
        exp_obstacle_two_shape = self.scenario.obstacles[2].obstacle_shape.__class__
        exp_obstacle_two_shape_prediction_state_list_len = len(self.scenario.obstacles[2].prediction.trajectory.state_list)
        exp_obstacle_two_shape_prediction_final_state_attributes_len = len(self.scenario.obstacles[2].prediction.trajectory.final_state.attributes)

        exp_lanelet_zero_center_vertices = self.scenario.lanelet_network.lanelets[0].center_vertices
        exp_lanelet_zero_left_vertices = self.scenario.lanelet_network.lanelets[0].left_vertices
        exp_lanelet_zero_right_vertices = self.scenario.lanelet_network.lanelets[0].right_vertices
        exp_lanelet_zero_adj_left = self.scenario.lanelet_network.lanelets[0].adj_left
        exp_lanelet_zero_adj_right = self.scenario.lanelet_network.lanelets[0].adj_right
        exp_lanelet_zero_adj_left_same_direction = self.scenario.lanelet_network.lanelets[0].adj_left_same_direction
        exp_lanelet_zero_adj_right_same_direction = self.scenario.lanelet_network.lanelets[0].adj_right_same_direction
        exp_lanelet_zero_predecessor = self.scenario.lanelet_network.lanelets[0].predecessor
        exp_lanelet_zero_succesor = self.scenario.lanelet_network.lanelets[0].successor
        exp_lanelet_zero_speed_limit = self.scenario.lanelet_network.lanelets[0].speed_limit

        exp_static_obstacles_on_lanelet_zero = {self.scenario.static_obstacles[0].obstacle_id}
        exp_static_obstacles_on_lanelet_one = {self.scenario.static_obstacles[0].obstacle_id}
        exp_dynamic_obstacles_on_lanelet_zero = {0: {self.scenario.dynamic_obstacles[1].obstacle_id},
                                                 1: {self.scenario.dynamic_obstacles[1].obstacle_id}}
        exp_dynamic_obstacles_on_lanelet_one = {0: {self.scenario.dynamic_obstacles[1].obstacle_id},
                                                1: {self.scenario.dynamic_obstacles[1].obstacle_id}}
        exp_lanelet_of_static_obstacle = {100, 101}
        exp_lanelet_of_dynamic_obstacle_initial = {100, 101}
        exp_lanelet_of_dynamic_obstacle_prediction = {0: {100, 101},
                                                      1: {100, 101}}


        exp_planning_problem_id = self.planning_problem_set.planning_problem_dict[1000].planning_problem_id
        exp_planning_problem_initial_state_velocity = self.planning_problem_set.planning_problem_dict[1000].initial_state.velocity
        exp_planning_problem_initial_state_slip_angle = self.planning_problem_set.planning_problem_dict[1000].initial_state.slip_angle
        exp_planning_problem_initial_state_yaw_rate = self.planning_problem_set.planning_problem_dict[1000].initial_state.yaw_rate

        xml_file = CommonRoadFileReader(self.filename_all).open()

        self.assertEqual(exp_num_lanelet_scenario,len(xml_file[0].lanelet_network.lanelets))
        self.assertEqual(exp_num_obstacles_scenario,len(xml_file[0].obstacles))
        self.assertEqual(exp_num_planning_problems, len(xml_file[1].planning_problem_dict))
        self.assertEqual(exp_benchmark_id, xml_file[0].benchmark_id)
        self.assertEqual(exp_dt, xml_file[0].dt)

        self.assertEqual(exp_obstacle_zero_id, xml_file[0].obstacles[0].obstacle_id)
        self.assertEqual(exp_obstacle_zero_type, xml_file[0].obstacles[0].obstacle_type)
        self.assertEqual(exp_obstacle_zero_role, xml_file[0].obstacles[0].obstacle_role)
        self.assertEqual(exp_obstacle_zero_shape, xml_file[0].obstacles[0].obstacle_shape.__class__)
        self.assertEqual(exp_obstacle_zero_radius, xml_file[0].obstacles[0].obstacle_shape.radius)
        self.assertEqual(exp_obstacle_zero_center[0], xml_file[0].obstacles[0].obstacle_shape.center[0])
        self.assertEqual(exp_obstacle_zero_center[1], xml_file[0].obstacles[0].obstacle_shape.center[1])

        self.assertEqual(exp_obstacle_one_id, xml_file[0].obstacles[1].obstacle_id)
        self.assertEqual(exp_obstacle_one_type, xml_file[0].obstacles[1].obstacle_type)
        self.assertEqual(exp_obstacle_one_role, xml_file[0].obstacles[1].obstacle_role)
        self.assertEqual(exp_obstacle_one_shape, xml_file[0].obstacles[1].obstacle_shape.__class__)
        self.assertEqual(exp_obstacle_one_attributes, len(xml_file[0].obstacles[1].initial_state.attributes))
        self.assertEqual(exp_obstacle_one_orientation, xml_file[0].obstacles[1].initial_state.orientation)
        self.assertEqual(exp_obstacle_one_predicition_zero_shape_center[0], xml_file[0].obstacles[1].prediction.occupancy_set[
            0].shape.center[0])
        self.assertEqual(exp_obstacle_one_predicition_zero_shape_center[1],
                         xml_file[0].obstacles[1].prediction.occupancy_set[
                             0].shape.center[1])

        self.assertEqual(exp_obstacle_two_id, xml_file[0].obstacles[2].obstacle_id)
        self.assertEqual(exp_obstacle_two_type, xml_file[0].obstacles[2].obstacle_type)
        self.assertEqual(exp_obstacle_two_role, xml_file[0].obstacles[2].obstacle_role)
        self.assertEqual(exp_obstacle_two_shape, xml_file[0].obstacles[2].obstacle_shape.__class__)
        self.assertEqual(exp_obstacle_two_shape_prediction_state_list_len, len(
            xml_file[0].obstacles[2].prediction.trajectory.state_list))
        self.assertEqual(exp_obstacle_two_shape_prediction_final_state_attributes_len, len(
            xml_file[0].obstacles[2].prediction.trajectory.final_state.attributes))

        np.testing.assert_array_equal(exp_lanelet_zero_center_vertices, xml_file[0].lanelet_network.lanelets[0].center_vertices)
        np.testing.assert_array_equal(exp_lanelet_zero_left_vertices, xml_file[0].lanelet_network.lanelets[0].left_vertices)
        np.testing.assert_array_equal(exp_lanelet_zero_right_vertices, xml_file[0].lanelet_network.lanelets[0].right_vertices)
        self.assertEqual(exp_lanelet_zero_adj_left, xml_file[0].lanelet_network.lanelets[0].adj_left)
        self.assertEqual(exp_lanelet_zero_adj_right, xml_file[0].lanelet_network.lanelets[0].adj_right)
        self.assertEqual(exp_lanelet_zero_adj_left_same_direction, xml_file[0].lanelet_network.lanelets[0].adj_left_same_direction)
        self.assertEqual(exp_lanelet_zero_adj_right_same_direction, xml_file[0].lanelet_network.lanelets[0].adj_right_same_direction)
        self.assertEqual(exp_lanelet_zero_predecessor, xml_file[0].lanelet_network.lanelets[0].predecessor)
        self.assertEqual(exp_lanelet_zero_succesor, xml_file[0].lanelet_network.lanelets[0].successor)
        self.assertEqual(exp_lanelet_zero_speed_limit, xml_file[0].lanelet_network.lanelets[0].speed_limit)

        self.assertSetEqual(exp_static_obstacles_on_lanelet_zero,
                            xml_file[0].lanelet_network.lanelets[0].static_obstacles_on_lanelet)
        self.assertSetEqual(exp_static_obstacles_on_lanelet_one,
                            xml_file[0].lanelet_network.lanelets[1].static_obstacles_on_lanelet)
        self.assertEqual(exp_dynamic_obstacles_on_lanelet_zero,
                         xml_file[0].lanelet_network.lanelets[0].dynamic_obstacles_on_lanelet)
        self.assertEqual(exp_dynamic_obstacles_on_lanelet_one,
                         xml_file[0].lanelet_network.lanelets[1].dynamic_obstacles_on_lanelet)
        self.assertSetEqual(exp_lanelet_of_static_obstacle,
                            xml_file[0].obstacle_by_id(0).initial_lanelet_ids)
        self.assertSetEqual(exp_lanelet_of_dynamic_obstacle_initial,
                            xml_file[0].obstacle_by_id(2).initial_lanelet_ids)
        self.assertEqual(exp_lanelet_of_dynamic_obstacle_prediction,
                         xml_file[0].obstacle_by_id(2).prediction.lanelet_assignment)

        self.assertEqual(exp_planning_problem_id, xml_file[1].planning_problem_dict[1000].planning_problem_id)
        self.assertEqual(exp_planning_problem_initial_state_velocity, xml_file[1].planning_problem_dict[1000].initial_state.velocity)
        self.assertEqual(exp_planning_problem_initial_state_slip_angle, xml_file[1].planning_problem_dict[1000].initial_state.slip_angle)
        self.assertEqual(exp_planning_problem_initial_state_yaw_rate, xml_file[1].planning_problem_dict[1000].initial_state.yaw_rate)

    # def test_open_all_scenarios(self):
    #     scenarios = self.cwd_path
    #     scenarios = scenarios[:len(scenarios)-51] + "/scenarios"
    #     cooperative = scenarios + "/cooperative"
    #     hand_crafted = scenarios + "/hand-crafted"
    #     NGSIM_Lankershim = scenarios + "/NGSIM/Lankershim"
    #     NGSIM_US101 = scenarios + "/NGSIM/US101"
    #
    #     for scenario in os.listdir(hand_crafted):
    #         full_path = hand_crafted + "/" + scenario
    #         CommonRoadFileReader(full_path).open()
    #
    #     for scenario in os.listdir(NGSIM_Lankershim):
    #         full_path = NGSIM_Lankershim + "/" + scenario
    #         CommonRoadFileReader(full_path).open()
    #
    #     for scenario in os.listdir(NGSIM_US101):
    #         full_path = NGSIM_US101 + "/" + scenario
    #         CommonRoadFileReader(full_path).open()
    #
    #     for scenario in os.listdir(cooperative):
    #         full_path = cooperative + "/" + scenario
    #         CommonRoadFileReader(full_path).open()



if __name__ == '__main__':
    unittest.main()
